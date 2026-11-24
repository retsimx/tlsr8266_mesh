//! The radio surface: everything the firmware drives the radio through.
//!
//! `app.rs` and `mesh/` name the radio only through this module (re-exported at
//! the crate root). The radio statics themselves are `pub(crate)`: visible to
//! the rest of the radio crate and rejected by rustc everywhere else, so a
//! module outside the crate that names one of them —
//! `tlsr8266_radio::ble::pair::PAIR_LOGIN_OK`, say — fails to compile instead of
//! merely failing review. The crate is the fence; nothing can widen it back by
//! accident.

// The read path and the write path are both public: the firmware reads the
// active profile (`app::RADIO_CONFIG` is a copy of it) and installs a new one
// through `set_radio_config`.
pub use crate::config::radio_config;
pub use crate::config::{set_radio_config, RadioConfig};

use core::cmp::min;

use crate::ble::connection::{
    BLE_PERIPHERAL_CONNECTION_ACTIVE, BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP,
};
use crate::ble::flash_config::access_code;
use crate::ble::pair::{
    MESH_PAIR_ENABLE, PAIR_AC, PAIR_CONFIG_MESH_LTK, PAIR_CONFIG_MESH_NAME, PAIR_CONFIG_MESH_PWD,
    PAIR_CONFIG_PWD_ENCODE_SK, PAIR_LOGIN_OK, PAIR_SETTING_FLAG, PAIR_STATE, SECURITY_ENABLE,
};
use crate::ble::status::{rf_link_slave_read_status_stop, SLAVE_READ_STATUS_BUSY};
use crate::mesh_ll::packet_processing::{is_add_packet_buf_ready, rf_link_add_tx_packet};
use crate::rf::drv_8266::{
    ADV_PRI_DATA, ADV_RSP_PRI_DATA, DEVICE_ADDRESS, FLASH_CONFIGURATION_INDEX, GROUP_ADDRESS,
    MAC_ID, MESH_DEVICE_ADDRESS_VALIDATION_PENDING, PKT_TERMINATE,
};
use crate::types::MAX_GROUP_COUNT;
use mesh_protocol::packet_types::Packet;
use tlsr8266_hal::common::sync::SimplifyLS;
use tlsr8266_hal::drivers::flash::flash_write_page;
use tlsr8266_hal::mcu::clock::clock_time_exceed;
use tlsr8266_hal::mcu::crypto::encode_password;

/// Types the facade signatures carry, re-exported so callers need no deeper
/// import path.
pub use crate::ble::pair::PairStateData;
pub use crate::types::ePairState;
pub use mesh_protocol::packet_types::{AdvPrivate, AdvRspPrivate};

// Same-purpose free functions already exist in the submodules; re-exporting
// them keeps one implementation while the facade supplies the name.
//
// All nine are named by the C-10 facade contract (design 031 §3.4), so they stay
// as zero-code `pub use` aliases. The firmware reaches most of the radio through
// them; the remaining direct `app.rs` / `mesh/` call sites name the concrete
// module because that is where the function is defined.
pub use crate::ble::flash_config::pair_load_key as load_pair_key;
pub use crate::ble::pair::pair_save_key as save_pair_key;
pub use crate::ble::pairing::rf_link_delete_pair as delete_pair_key;
pub use crate::irq::mesh::configure_rf_for_mesh_listening as rf_switch_to_listen_channel;
pub use crate::mesh_ll::node_status::mesh_node_table_init;
pub use crate::mesh_ll::node_status::mesh_report_status_enable as report_mesh_status;
pub use crate::queues::set_uart_status_reporting_enabled;
pub use crate::rf::drv_8266::rf_clear_rx_irq;
pub use crate::rf::drv_8266::rf_set_ble_access_code as rf_set_access_code;

//------------------------------------------------------------------------------
// Identity
//------------------------------------------------------------------------------

/// The device's 16-bit mesh address.
pub fn device_address() -> u16 {
    DEVICE_ADDRESS.get()
}

/// The 6-byte MAC address loaded from flash.
pub fn mac_id() -> [u8; 6] {
    *MAC_ID.lock()
}

/// The group addresses this device answers to.
pub fn group_addresses() -> [u16; MAX_GROUP_COUNT as usize] {
    *GROUP_ADDRESS.lock()
}

/// Flash slot of the active pairing record, or `None` when none exists.
///
/// The storage layer keeps the "no record" case as the `-1` sentinel; the
/// facade turns it into `None` so callers never arithmetic on a sentinel.
pub fn flash_config_index() -> Option<u32> {
    let index = FLASH_CONFIGURATION_INDEX.get();
    if index < 0 {
        None
    } else {
        Some(index as u32)
    }
}

/// Whether a device-address configuration is still awaiting MAC validation.
///
/// Non-consuming: the mesh pairing effect reads the flag without clearing it,
/// because the clear belongs to the validation-completed path.
pub fn dev_addr_validation_pending() -> bool {
    MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get()
}

/// Consume the "device address awaits MAC validation" flag.
///
/// Returns the value the flag held and clears it, so exactly one caller acts on
/// a pending validation. The read-and-clear is one critical section because
/// `thumbv6m` has no atomic swap.
pub fn take_dev_addr_validation_pending() -> bool {
    critical_section::with(|_| {
        let pending = MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get();
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);
        pending
    })
}

/// Install both private advertising payloads and the manufacturer bytes the
/// advertisement filter matches on.
///
/// The order is fixed: the primary payload is stored, then the manufacturer
/// bytes the ceiling packet carries, then the scan-response payload.
pub fn set_adv_private_data(pri: AdvPrivate, pri_bytes: &[u8; 8], rsp: AdvRspPrivate) {
    *ADV_PRI_DATA.lock() = pri;
    crate::mesh_ll::adv::set_advertisement_manufacturer_data(pri_bytes);
    *ADV_RSP_PRI_DATA.lock() = rsp;
}

//------------------------------------------------------------------------------
// Pairing
//------------------------------------------------------------------------------

/// Whether the connected host has completed the pairing login.
pub fn is_paired_login() -> bool {
    PAIR_LOGIN_OK.get()
}

/// Whether mesh packet encryption is enabled.
pub fn is_security_enabled() -> bool {
    SECURITY_ENABLE.get()
}

/// Enable or disable mesh packet encryption.
pub fn set_security_enabled(enabled: bool) {
    SECURITY_ENABLE.set(enabled)
}

/// Whether the device accepts mesh pairing commands.
pub fn is_mesh_pair_enabled() -> bool {
    MESH_PAIR_ENABLE.get()
}

/// Enable or disable mesh pairing.
pub fn set_mesh_pair_enabled(enabled: bool) {
    MESH_PAIR_ENABLE.set(enabled)
}

/// Copy the whole pairing credential state out of the lock.
///
/// `mesh/` re-arms the pairing FSM from `pair_nn` / `pair_pass` /
/// `pair_ltk_mesh`, and the effect phases need the rest of the snapshot; the
/// guard is released before the caller touches flash or the radio.
pub fn pair_credentials_snapshot() -> PairStateData {
    let state = PAIR_STATE.lock();
    PairStateData {
        pair_ltk: state.pair_ltk,
        pair_sk: state.pair_sk,
        pair_work: state.pair_work,
        pair_nn: state.pair_nn,
        pair_pass: state.pair_pass,
        pair_ltk_mesh: state.pair_ltk_mesh,
        pair_sk_copy: state.pair_sk_copy,
        pair_rands: state.pair_rands,
        pair_randm: state.pair_randm,
    }
}

/// Current mesh pairing FSM state.
pub fn pair_setting_flag() -> ePairState {
    *PAIR_SETTING_FLAG.lock()
}

/// Move the mesh pairing FSM to `flag`.
pub fn set_pair_setting_flag(flag: ePairState) {
    *PAIR_SETTING_FLAG.lock() = flag;
}

/// Access code the radio currently filters on.
pub fn current_access_code() -> u32 {
    PAIR_AC.get()
}

/// Install newly paired mesh credentials into the pairing state.
///
/// `use_pair_ltk` keeps the immediate-effect command's semantics: the mesh LTK
/// the device already holds is what gets applied, not the buffered one.
pub fn apply_new_mesh_credentials(
    name: &[u8; 16],
    pwd: &[u8; 16],
    ltk: &[u8; 16],
    use_pair_ltk: bool,
) {
    let mut state = PAIR_STATE.lock();

    if !use_pair_ltk {
        state.pair_nn.copy_from_slice(name);
        state.pair_pass.copy_from_slice(pwd);
        state.pair_ltk.copy_from_slice(ltk);
    } else {
        let pair_ltk_mesh = state.pair_ltk_mesh;
        state.pair_ltk.copy_from_slice(&pair_ltk_mesh);
    }
}

/// Point the pairing state at the configured default mesh: access code and LTK
/// only, leaving the network name and password untouched.
pub fn switch_to_default_mesh_credentials() {
    // Copy the credentials out first: `access_code` runs AES, which masks
    // interrupts, so no credential lock may still be held here.
    let (name, pwd, ltk) = {
        let name = *PAIR_CONFIG_MESH_NAME.lock();
        let pwd = *PAIR_CONFIG_MESH_PWD.lock();
        let ltk = *PAIR_CONFIG_MESH_LTK.lock();
        (name, pwd, ltk)
    };

    PAIR_AC.set(access_code(&name, &pwd));
    PAIR_STATE.lock().pair_ltk.copy_from_slice(&ltk);
}

/// Re-write the out-of-mesh pairing credentials after a kick-out erased the
/// configuration sectors.
///
/// Four 16-byte blocks, in the slot layout the pairing code reads: flags at
/// `+0`, network name at `+16`, encoded password at `+32`, LTK at `+48`.
pub fn write_out_of_mesh_credentials() {
    let cfg = crate::config::radio_config();
    let mut flags: [u8; 16] = [0; 16];
    let mut name: [u8; 16] = [0; 16];
    let mut password: [u8; 16] = [0; 16];

    flags[0] = cfg.pair_valid_flag;
    flags[15] = cfg.pair_valid_flag;
    let len = min(cfg.out_of_mesh_name.len(), name.len());
    name[0..len].copy_from_slice(&cfg.out_of_mesh_name.as_bytes()[0..len]);
    let len = min(cfg.default_mesh_pwd.len(), password.len());
    password[0..len].copy_from_slice(&cfg.default_mesh_pwd.as_bytes()[0..len]);

    // Encrypt the password before storing. The key is copied out of the mutex
    // first so the lock is released before `encode_password` runs (its AES call
    // masks interrupts).
    let key = *PAIR_CONFIG_PWD_ENCODE_SK.lock();
    let encoded_password = encode_password(&key, &password);

    // Only write the rejoin flag when mesh pairing is enabled.
    if MESH_PAIR_ENABLE.get() {
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(true);
        flags[1] = 1;
    }

    let mut ltk: [u8; 16] = *PAIR_CONFIG_MESH_LTK.lock();
    flash_write_page(cfg.flash_adr_pairing + 48, 16, ltk.as_mut_ptr());
    flash_write_page(cfg.flash_adr_pairing + 32, 16, encoded_password.as_ptr());
    flash_write_page(cfg.flash_adr_pairing + 16, 16, name.as_ptr());
    flash_write_page(cfg.flash_adr_pairing, 16, flags.as_ptr());
}

//------------------------------------------------------------------------------
// Link / mesh
//------------------------------------------------------------------------------

/// Whether a BLE central is currently connected.
pub fn is_ble_connected() -> bool {
    BLE_PERIPHERAL_CONNECTION_ACTIVE.get()
}

/// Enforce the "log in within `auth_time_s` of the first connection" rule.
///
/// Clears the first-connection timestamp once the window has expired while
/// security is on and the host has not logged in; the connection itself is kept.
pub fn check_auth_timeout(auth_time_s: u32) {
    if SECURITY_ENABLE.get()
        && !PAIR_LOGIN_OK.get()
        && BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.get() != 0
        && clock_time_exceed(
            BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.get(),
            auth_time_s * 1000 * 1000,
        )
    {
        BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(0);
    }
}

/// Abort an in-flight status read so the link is free for another operation.
pub fn stop_slave_read_status_if_busy() {
    if SLAVE_READ_STATUS_BUSY.get() != 0 {
        rf_link_slave_read_status_stop();
    }
}

/// Queue the connection-termination packet.
///
/// `true` only when the packet was actually queued, so a full buffer neither
/// arms a countdown nor reboots.
pub fn try_send_terminate_packet() -> bool {
    if !is_add_packet_buf_ready() {
        return false;
    }
    let _ = rf_link_add_tx_packet(&PKT_TERMINATE);
    true
}

//------------------------------------------------------------------------------
// Test-only state access
//
// Production never writes these outside the crate; host tests do. They are gated
// on `cfg(not(target_os = "none"))`, not `cfg(test)`, so the firmware's own test
// fixture can drive them while the target build keeps no write surface for state
// the radio owns.
//------------------------------------------------------------------------------

/// Restore every radio static the facade exposes to its power-on value.
///
/// The config-driven GATT profile is process-wide too, so it is re-seeded from
/// the factory config here: a test that injects a name or version and forgets
/// to restore it would otherwise leak into every later test in the run.
#[cfg(not(target_os = "none"))]
pub fn reset_radio_state_for_test() {
    set_radio_config(crate::config::DEFAULT);
    crate::gatt::service_table::init_gatt_profile(&crate::config::DEFAULT);
    DEVICE_ADDRESS.set(0);
    *MAC_ID.lock() = [0; 6];
    *GROUP_ADDRESS.lock() = [0; MAX_GROUP_COUNT as usize];
    FLASH_CONFIGURATION_INDEX.set(0);
    MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);
    *PAIR_STATE.lock() = crate::ble::pair::DEFAULT_PAIR_STATE;
    PAIR_LOGIN_OK.set(false);
    PAIR_AC.set(0);
    *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
    SECURITY_ENABLE.set(false);
    MESH_PAIR_ENABLE.set(false);
    BLE_PERIPHERAL_CONNECTION_ACTIVE.set(false);
    BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(0);
    SLAVE_READ_STATUS_BUSY.set(0);
}

#[cfg(not(target_os = "none"))]
pub fn set_device_address(address: u16) {
    DEVICE_ADDRESS.set(address);
}

#[cfg(not(target_os = "none"))]
pub fn set_mac_id(mac: [u8; 6]) {
    *MAC_ID.lock() = mac;
}

#[cfg(not(target_os = "none"))]
pub fn set_group_addresses(groups: [u16; MAX_GROUP_COUNT as usize]) {
    *GROUP_ADDRESS.lock() = groups;
}

#[cfg(not(target_os = "none"))]
pub fn set_flash_config_index(index: i32) {
    FLASH_CONFIGURATION_INDEX.set(index);
}

#[cfg(not(target_os = "none"))]
pub fn set_dev_addr_validation_pending(pending: bool) {
    MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(pending);
}

#[cfg(not(target_os = "none"))]
pub fn set_pair_credentials(credentials: PairStateData) {
    *PAIR_STATE.lock() = credentials;
}

#[cfg(not(target_os = "none"))]
pub fn set_paired_login(paired: bool) {
    PAIR_LOGIN_OK.set(paired);
}

#[cfg(not(target_os = "none"))]
pub fn set_current_access_code(code: u32) {
    PAIR_AC.set(code);
}

/// Replace the seeded default-mesh credential buffers with raw bytes.
#[cfg(not(target_os = "none"))]
pub fn set_pair_config_mesh_credentials(name: [u8; 16], pwd: [u8; 16], ltk: [u8; 16]) {
    *PAIR_CONFIG_MESH_NAME.lock() = name;
    *PAIR_CONFIG_MESH_PWD.lock() = pwd;
    *PAIR_CONFIG_MESH_LTK.lock() = ltk;
}

#[cfg(not(target_os = "none"))]
pub fn set_ble_connected(connected: bool) {
    BLE_PERIPHERAL_CONNECTION_ACTIVE.set(connected);
}

#[cfg(not(target_os = "none"))]
pub fn set_first_connection_timestamp(timestamp: u32) {
    BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(timestamp);
}

/// The tick at which the current BLE connection was first seen, or `0` when the
/// auth window has already been closed. Host tests assert the timeout rule
/// through this read rather than the static.
#[cfg(not(target_os = "none"))]
pub fn first_connection_timestamp() -> u32 {
    BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.get()
}

#[cfg(not(target_os = "none"))]
pub fn set_slave_read_status_busy(busy: u8) {
    SLAVE_READ_STATUS_BUSY.set(busy);
}

/// Read both private advertising payloads, for tests that assert what the
/// vendoring step wrote.
#[cfg(not(target_os = "none"))]
pub fn adv_private_data() -> (AdvPrivate, AdvRspPrivate) {
    (*ADV_PRI_DATA.lock(), *ADV_RSP_PRI_DATA.lock())
}

/// Read one staged response packet, for tests that assert what a response
/// handler produced.
#[cfg(not(target_os = "none"))]
pub fn buffered_response(index: usize) -> Packet {
    crate::ble::connection::BUFF_RESPONSE.lock()[index]
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::ble::connection::SLAVE_DATA_VALID;
    use crate::ble::status::{
        DEVICE_STATUS_BUFFER_READ_POINTER, DEVICE_STATUS_BUFFER_WRITE_POINTER,
        DEVICE_STATUS_READ_UNICAST_MODE,
    };

    /// The storage layer keeps "no pairing record" as `-1`; the facade owns the
    /// mapping to `None`, so it needs a test of its own.
    #[test]
    fn flash_config_index_maps_the_negative_sentinel_to_none() {
        set_flash_config_index(-1);
        assert_eq!(flash_config_index(), None, "-1 means no record");

        set_flash_config_index(0);
        assert_eq!(flash_config_index(), Some(0), "slot 0 is a real record");

        set_flash_config_index(10);
        assert_eq!(flash_config_index(), Some(10));

        set_flash_config_index(0);
    }

    /// The busy branch of `stop_slave_read_status_if_busy` moved into this
    /// module with the refactor, so both branches are covered here.
    ///
    /// The abort is observed through the state the real
    /// `rf_link_slave_read_status_stop` clears, rather than through a mock of
    /// the callee: a busy link loses its busy/unicast/pending-data state and
    /// its buffer pointers, an idle link keeps all of them.
    #[test]
    fn stop_slave_read_status_if_busy_aborts_only_a_busy_read() {
        set_slave_read_status_busy(1);
        SLAVE_DATA_VALID.set(1);
        DEVICE_STATUS_READ_UNICAST_MODE.set(true);
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(3);
        DEVICE_STATUS_BUFFER_READ_POINTER.set(2);

        stop_slave_read_status_if_busy();

        assert_eq!(SLAVE_READ_STATUS_BUSY.get(), 0, "the busy flag is cleared");
        assert_eq!(SLAVE_DATA_VALID.get(), 0, "pending status data is dropped");
        assert_eq!(
            DEVICE_STATUS_READ_UNICAST_MODE.get(),
            false,
            "the unicast window is closed"
        );
        assert_eq!(
            (
                DEVICE_STATUS_BUFFER_WRITE_POINTER.get(),
                DEVICE_STATUS_BUFFER_READ_POINTER.get()
            ),
            (0, 0),
            "the status response buffer is reinitialised"
        );

        // An idle link must not be re-initialised: keep non-default state and
        // prove the second call leaves it alone.
        set_slave_read_status_busy(0);
        SLAVE_DATA_VALID.set(3);
        DEVICE_STATUS_READ_UNICAST_MODE.set(true);
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(5);

        stop_slave_read_status_if_busy();

        assert_eq!(SLAVE_DATA_VALID.get(), 3, "idle link keeps pending data");
        assert_eq!(
            DEVICE_STATUS_READ_UNICAST_MODE.get(),
            true,
            "idle link keeps its unicast window"
        );
        assert_eq!(
            DEVICE_STATUS_BUFFER_WRITE_POINTER.get(),
            5,
            "idle link keeps its buffer position"
        );

        set_slave_read_status_busy(0);
        SLAVE_DATA_VALID.set(0);
        DEVICE_STATUS_READ_UNICAST_MODE.set(false);
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(0);
    }
}
