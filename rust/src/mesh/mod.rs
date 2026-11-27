//! The mesh subsystem: packets in, packets out, and the pairing state machine's
//! hardware side.
//!
//! The dependency direction is strictly downward. This directory owns the mesh
//! transport (the send/receive tasks, the retransmission queue and the relay
//! path) and the flash/radio half of re-pairing. It may reach `tlsr8266_hal`,
//! `mesh_protocol`, `crate::config`, `crate::logging` and the `tlsr8266_radio` glue
//! it drives; it never names `app`, `irq`, `light`, `uart`, `ota`,
//! `factory_reset`, `diagnostics` or the PAC register crate
//! (`sdk/check_layering.py` enforces it).
//!
//! Three things the subsystem cannot do itself belong to its consumers: report
//! a light event (the LED and the node table), publish the lamp's status, and
//! relay a received packet to the UART bridge. Those arrive through
//! [`MeshEventHooks`], registered once by `app::main_task`, and every call goes
//! through a named `#[inline(never)]` dispatcher so `sdk/stack_analysis.py` can
//! resolve the edge in the linked ELF.
//!
//! The pairing protocol itself — every transition, timer and payload byte — is
//! in [`fsm`], which names no hardware at all.

mod fsm;

use core::ptr::addr_of;
use core::sync::atomic::{AtomicU32, Ordering};

#[cfg(not(test))]
use embassy_time::{Duration, Instant, Timer, WithTimeout};
use heapless::Vec;

use crate::config::{FLASH_ADR_PAIRING, VENDOR_ID};
use crate::{uprintln, BIT};
pub use fsm::{
    MeshPairAction, MeshPairFsm, MeshPairState, MESH_PAIR_CMD_INTERVAL, MESH_PAIR_TIMEOUT,
};
use mesh_protocol::mesh_types::*;
use mesh_protocol::packet_types::{Packet, PacketAttCmd, PacketAttValue, PacketL2capHead};
use tlsr8266_hal::common::sync::CriticalSectionMutex;
use tlsr8266_hal::drivers::flash::flash_write_page;
use tlsr8266_hal::mcu::clock::{clock_time, clock_time64, sleep_us, CLOCK_SYS_CLOCK_1US};
use tlsr8266_radio::ble::flash_config::{pair_load_key, SYS_CHN_LISTEN};
use tlsr8266_radio::ble::pair::{pair_enc_packet_mesh, pair_save_key};
use tlsr8266_radio::mesh_ll::node_status::{
    mesh_construct_packet, mesh_node_table_init, rf_link_match_group_mac,
};
use tlsr8266_radio::mesh_ll::packet_processing::{
    is_add_packet_buf_ready, rf_link_add_tx_packet, rf_link_rc_data,
};
use tlsr8266_radio::queues::{self, SendPkt, MESH_RX_CHANNEL, MESH_TX_QUEUE, MESH_TX_WAKE};
#[cfg(test)]
use tlsr8266_radio::reset_radio_state_for_test;
use tlsr8266_radio::rf::drv_8266::{
    is_gatt_spp_write_packet, rf_link_slave_data_write_task, rf_set_ble_access_code,
    rf_set_ble_channel, rf_set_ble_crc_adv, rf_set_rxmode, rf_set_tx_rx_off, rf_start_srx2tx,
};
use tlsr8266_radio::types::*;
use tlsr8266_radio::{
    apply_new_mesh_credentials, current_access_code, dev_addr_validation_pending, device_address,
    flash_config_index, is_ble_connected, is_mesh_pair_enabled, is_paired_login,
    is_security_enabled, pair_credentials_snapshot, pair_setting_flag, set_mesh_pair_enabled,
    set_pair_setting_flag, set_security_enabled, switch_to_default_mesh_credentials,
    take_dev_addr_validation_pending,
};

/// The application edges the mesh subsystem must not compile in.
///
/// `on_light_event` is the LED/mesh-timeout event (`app`'s
/// `rf_link_light_event_callback`), `on_device_status_update` publishes the
/// lamp state to the node table (owned by `light`), and `on_uart_mesh_rx`
/// relays one received packet to the UART bridge.
#[derive(Copy, Clone)]
pub struct MeshEventHooks {
    pub on_light_event: fn(u8),
    pub on_device_status_update: fn(),
    pub on_uart_mesh_rx: fn(&Packet),
}

/// Registered once by the composition root before any mesh traffic exists.
static MESH_EVENT_HOOKS: CriticalSectionMutex<Option<MeshEventHooks>> =
    CriticalSectionMutex::new(None);

/// Install the application hooks. Called by `app::main_task` before
/// `rf_drv_init(true)`, so the slots are never observed empty in production.
pub fn set_mesh_event_hooks(hooks: MeshEventHooks) {
    *MESH_EVENT_HOOKS.lock() = Some(hooks);
}

/// Report a light/mesh event (LED indication, node-table reset, pairing timeout).
///
/// `#[inline(never)]` on purpose: the call is indirect, so the symbol is what
/// lets `sdk/stack_analysis.py`'s `INDIRECT_EDGES` entry `dispatch_mesh_light_event`
/// resolve to `rf_link_light_event_callback`. The hook set is copied out of the
/// mutex first: the critical section must not cover the callback, which touches
/// the node table and the LED.
#[inline(never)]
fn dispatch_mesh_light_event(cmd: u8) {
    let hooks = *MESH_EVENT_HOOKS.lock();
    if let Some(hooks) = hooks {
        (hooks.on_light_event)(cmd);
    }
}

/// Publish the lamp's on/off state to the mesh node table.
///
/// `#[inline(never)]`: the `INDIRECT_EDGES` key `publish_light_status` resolves
/// to `light::device_status_update`. The name says *what* it publishes rather
/// than naming the callee, so the caller pattern is not a substring of the
/// callee's own (module-qualified) symbol and both resolve uniquely.
#[inline(never)]
fn publish_light_status() {
    let hooks = *MESH_EVENT_HOOKS.lock();
    if let Some(hooks) = hooks {
        (hooks.on_device_status_update)();
    }
}

/// Hand one received mesh packet to the UART bridge.
///
/// `#[inline(never)]`: the `INDIRECT_EDGES` key `dispatch_uart_mesh_rx` resolves
/// to `uart_mesh_rx_hook`, which is the same hook the SDK's bridge calls.
#[inline(never)]
fn dispatch_uart_mesh_rx(pkt: &Packet) {
    let hooks = *MESH_EVENT_HOOKS.lock();
    if let Some(hooks) = hooks {
        (hooks.on_uart_mesh_rx)(pkt);
    }
}

/// Sole owner of the mesh pairing state.
pub static MESH_PAIR: CriticalSectionMutex<MeshPairFsm> =
    CriticalSectionMutex::new(MeshPairFsm::new());

/// Last `MESH_RX_CHANNEL` drop count turned into a log line.
///
/// Shared by the receive and send tasks, so it is module state rather than a
/// task local: an idle node whose receive task overflows the channel must
/// report the drop even when the send task never runs.
static LAST_REPORTED_RX_DROPS: AtomicU32 = AtomicU32::new(0);

/// The pairing-machine fields the root tests in `app.rs` assert on.
///
/// The fields themselves are `pub(super)` so they never become a writable
/// public surface; this snapshot is the one read window the composition-root
/// tests need.
#[cfg(test)]
#[derive(Default)]
pub struct PairFsmView {
    pub new_mesh_name: [u8; 16],
    pub effect_new_mesh: u8,
    pub mesh_pair_cmd_interval: u32,
    pub mesh_pair_timeout: u32,
}

/// Snapshot the pairing machine's state for `app.rs`'s tests.
#[cfg(test)]
#[coverage(off)]
pub fn pair_fsm_view() -> PairFsmView {
    let pair = MESH_PAIR.lock();
    PairFsmView {
        new_mesh_name: pair.new_mesh_name,
        effect_new_mesh: pair.effect_new_mesh,
        mesh_pair_cmd_interval: pair.mesh_pair_cmd_interval,
        mesh_pair_timeout: pair.mesh_pair_timeout,
    }
}

/// Arm the immediate-effect path for `app.rs`'s tests.
#[cfg(test)]
#[coverage(off)]
pub fn set_pair_effect_new_mesh(value: u8) {
    MESH_PAIR.lock().effect_new_mesh = value;
}

/// Recording hooks the host tests install in place of the composition root's.
///
/// The three event edges are function pointers registered by `app::main_task`,
/// so a test observes them exactly the way production does: it registers
/// functions that count and record. The counters live here rather than in
/// `mod tests` because [`reset_for_test`] has to clear them.
#[cfg(test)]
mod test_hooks {
    use super::{MeshEventHooks, Packet};
    use core::sync::atomic::{AtomicU32, Ordering};

    pub static LIGHT_EVENTS: AtomicU32 = AtomicU32::new(0);
    pub static LAST_LIGHT_EVENT: AtomicU32 = AtomicU32::new(0);
    pub static STATUS_UPDATES: AtomicU32 = AtomicU32::new(0);
    pub static RELAYED_PACKETS: AtomicU32 = AtomicU32::new(0);
    pub static RELAYED_SRC: AtomicU32 = AtomicU32::new(0);
    pub static RELAYED_DST: AtomicU32 = AtomicU32::new(0);

    fn record_light_event(cmd: u8) {
        LIGHT_EVENTS.store(LIGHT_EVENTS.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
        LAST_LIGHT_EVENT.store(cmd as u32, Ordering::Relaxed);
    }

    fn record_device_status_update() {
        STATUS_UPDATES.store(
            STATUS_UPDATES.load(Ordering::Relaxed) + 1,
            Ordering::Relaxed,
        );
    }

    fn record_uart_mesh_rx(pkt: &Packet) {
        RELAYED_PACKETS.store(
            RELAYED_PACKETS.load(Ordering::Relaxed) + 1,
            Ordering::Relaxed,
        );
        RELAYED_SRC.store(pkt.ll_app().value.src as u32, Ordering::Relaxed);
        RELAYED_DST.store(pkt.ll_app().value.dst as u32, Ordering::Relaxed);
    }

    /// The hook set a test starts from.
    pub fn hooks() -> MeshEventHooks {
        MeshEventHooks {
            on_light_event: record_light_event,
            on_device_status_update: record_device_status_update,
            on_uart_mesh_rx: record_uart_mesh_rx,
        }
    }

    /// Forget every call the previous test made.
    pub fn reset() {
        LIGHT_EVENTS.store(0, Ordering::Relaxed);
        LAST_LIGHT_EVENT.store(0, Ordering::Relaxed);
        STATUS_UPDATES.store(0, Ordering::Relaxed);
        RELAYED_PACKETS.store(0, Ordering::Relaxed);
        RELAYED_SRC.store(0, Ordering::Relaxed);
        RELAYED_DST.store(0, Ordering::Relaxed);
    }
}

/// Restore the module-level pairing state and queues between host tests.
///
/// Host tests share one process, so a test that leaves `MESH_PAIR`, the mesh
/// queues, the radio statics or a recorded hook call behind would leak into the
/// next one. The radio state is reset through the facade, and the recording
/// hooks are reinstalled because production leaves the slot empty until
/// `app::main_task` fills it.
#[cfg(test)]
pub fn reset_for_test() {
    *MESH_PAIR.lock() = MeshPairFsm::new();
    queues::reset_for_test();
    reset_radio_state_for_test();
    test_hooks::reset();
    set_mesh_event_hooks(test_hooks::hooks());
    critical_section::with(|_| LAST_REPORTED_RX_DROPS.store(0, Ordering::Relaxed));
}

// p_cmd : cmd[3]+para[10]
// para    : dst
pub fn light_slave_tx_command(
    p_cmd: &Vec<u8, 13>,
    para: u16,
    retransmit_count: u8,
    send_ack: bool,
) -> Packet {
    let mut cmd_op_para: [u8; 13] = [0; 13];
    let cmd_sno = clock_time() + device_address() as u32;

    cmd_op_para[0..13].copy_from_slice(&p_cmd[0..13]);

    cmd_op_para[0] |= 0xc0;
    cmd_op_para[1] = (VENDOR_ID & 0xFF) as u8;
    cmd_op_para[2] = (VENDOR_ID >> 8) as u8;

    let dst = para;
    mesh_construct_packet(cmd_sno, dst, &cmd_op_para, retransmit_count, send_ack)
}

/// Queue one constructed packet for local delivery and/or transmission.
///
/// A packet this node matches itself is handed to the receive side as well, so a
/// locally generated command runs through the same decoder; the transmit half
/// is skipped only for a device-addressed packet, which has nowhere to go.
fn queue_mesh_packet(pkt: &Packet) {
    let (group_match, device_match) = rf_link_match_group_mac(pkt);
    if group_match || device_match {
        queues::enqueue_mesh_rx(pkt);
        if !device_match {
            queues::enqueue_mesh_tx(
                pkt,
                0,
                pkt.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
            );
        }
    } else {
        queues::enqueue_mesh_tx(
            pkt,
            0,
            pkt.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
        );
    }
}

/// Send a message to the mesh and return the sequence number it was given.
///
/// `data`: 13 bytes of data usually in form `[op, vendor_id_hi, vendor_id_lo, params...]`
/// `destination`: the address of the destination
/// `retransmit_count`: how many times the packet should be rebroadcast at each node
/// `send_ack`: whether the destination device should send an ack after handling it
pub fn send_mesh_message(
    data: &Vec<u8, 13>,
    destination: u16,
    retransmit_count: u8,
    send_ack: bool,
) -> [u8; 3] {
    let pkt = light_slave_tx_command(data, destination, retransmit_count, send_ack);
    queue_mesh_packet(&pkt);

    pkt.mesh().sno
}

/// Enable the re-pairing state machine and arm its command interval and timeout.
pub fn mesh_pair_init() {
    set_mesh_pair_enabled(true);

    // The broadcast machine transmits the credentials of whichever network the
    // SDK currently holds, so it keeps its own copy; `mesh_pair_proc` refreshes
    // it while a broadcast sequence is running.
    let credentials = pair_credentials_snapshot();
    let (name, pwd, ltk) = (
        credentials.pair_nn,
        credentials.pair_pass,
        credentials.pair_ltk_mesh,
    );

    // Sample the SDK's pairing flag before taking the FSM's mutex: one
    // critical-section lock must never be taken while another is held.
    let flag = pair_setting_flag();

    let mut pair = MESH_PAIR.lock();
    pair.mesh_pair_cmd_interval = MESH_PAIR_CMD_INTERVAL;
    pair.mesh_pair_timeout = MESH_PAIR_TIMEOUT;
    pair.set_credentials(&name, &pwd, &ltk);
    pair.sync_flag(flag);
}

/// Apply a pending switch to the new mesh, if one is due.
///
/// The due check and the payload are the state machine's; the flash save that
/// follows takes its own locks and copies the credentials out of the action,
/// so no pairing guard is alive while the sector is erased or written.
pub fn mesh_pair_proc_effect() {
    let action = {
        let pair = MESH_PAIR.lock();
        pair.effect_action(clock_time(), dev_addr_validation_pending())
    };

    let Some(action) = action else {
        return;
    };

    match action {
        MeshPairAction::ReloadSavedKey => reload_saved_key(),
        MeshPairAction::SaveAndApply {
            name,
            pwd,
            ltk,
            use_pair_ltk,
        } => save_and_apply(name, pwd, ltk, use_pair_ltk),
        // The remaining arms belong to the timer and broadcast phases.
        MeshPairAction::None
        | MeshPairAction::SwitchToDefaultMesh { .. }
        | MeshPairAction::Broadcast(_)
        | MeshPairAction::TimedOut => {}
    }

    let mut pair = MESH_PAIR.lock();
    pair.effect_new_mesh = 0;
    pair.effect_new_mesh_delay_time = 0;
}

/// Marks device address configuration as successfully validated.
/// Called when LGT_CMD_CONFIG_DEV_ADDR is processed and device address is accepted.
/// Consumes the validation flag, allowing mesh credentials to be saved.
pub fn mesh_device_address_validation_completed() {
    take_dev_addr_validation_pending(); // validation completed successfully
    if is_mesh_pair_enabled() {
        // Only write to flash when a configuration record is loaded: the storage
        // layer reports "no config" as `None`, and then nothing may be written.
        // The header starts at the record's flash offset and byte [1] holds the
        // validation state, so that byte is programmed to zero.
        if let Some(index) = flash_config_index() {
            let mut data: [u8; 1] = [0];
            // The address validation is reported to the peer over BLE, not by a
            // return value; a failed byte program is retried by the next
            // validation cycle, as before.
            let _ = flash_write_page(
                (FLASH_ADR_PAIRING as i32 + index as i32 + 1) as u32,
                1,
                data.as_mut_ptr(),
            );
        }
    }
}

/// Feed one received mesh pairing command into the pairing state machine.
pub fn mesh_pair_cb(params: &Vec<u8, 10>) {
    let mut chunk = [0u8; 8];
    chunk.copy_from_slice(&params[1..9]);
    let now = clock_time();
    let cmd = MeshPairState::try_from(params[0]);

    // Receiving a chunk never acts: the credentials are persisted only once the
    // delayed-effect window armed by `MeshPairEffectDelay` expires, in
    // `mesh_pair_proc_effect`. That window is what gives the tool time to call
    // the switch off, so this keeps the pre-split timing exactly — and it is why
    // the accumulators' `MeshPairAction` return value is dropped here.
    let mut pair = MESH_PAIR.lock();

    // A command from the network keeps the default-mesh window alive even
    // when it is not one this firmware understands, exactly as before the
    // split: the refresh is not part of the command dispatch.
    if pair.default_mesh_time_ref != 0 {
        pair.default_mesh_time_ref = now | 1;
    }

    match cmd {
        Ok(state @ (MeshPairState::MeshPairName1 | MeshPairState::MeshPairName2)) => {
            pair.accumulate_name(state, &chunk, now);
        }
        Ok(state @ (MeshPairState::MeshPairPwd1 | MeshPairState::MeshPairPwd2)) => {
            pair.accumulate_pwd(state, &chunk);
        }
        Ok(state @ (MeshPairState::MeshPairLtk1 | MeshPairState::MeshPairLtk2)) => {
            pair.accumulate_ltk(state, &chunk);
        }
        Ok(MeshPairState::MeshPairEffectDelay) => {
            pair.accumulate_effect_delay(now);
        }
        Ok(MeshPairState::MeshPairEffect) => {
            pair.accumulate_effect();
        }
        Ok(MeshPairState::MeshPairDefaultMesh) => {
            pair.accumulate_default_mesh(chunk[0], now);
        }
        Err(()) => {}
    }
}

fn mesh_cmd_notify(op: u8, p: &Vec<u8, 10>, dev_adr: u16) -> i32 {
    let mut err = -1;
    if is_ble_connected() && is_paired_login() {
        if p.len() > 10 {
            //max length of par is 10
            return -1;
        }

        let mut pkt_notify = PacketAttCmd {
            head: PacketL2capHead {
                dma_len: 0x1d,   // dma_len
                _type: 0x02,     // type
                rf_len: 0x1b,    // rf_len
                l2cap_len: 0x17, // u16
                chan_id: 0x04,   // chanid
            },
            opcode: 0x1b, // notify
            handle: 0x12,
            handle1: 0x00, // status handler
            value: PacketAttValue {
                sno: [op | 0xc0, (VENDOR_ID & 0xff) as u8, (VENDOR_ID >> 8) as u8],
                src: [(dev_adr & 0xFF) as u8, (dev_adr >> 8) as u8],
                dst: [0; 2],
                val: [0; 23],
            },
        };

        pkt_notify.value.val[3..3 + p.len()].copy_from_slice(&p[0..p.len()]);

        if is_add_packet_buf_ready()
            && rf_link_add_tx_packet(&Packet {
                att_cmd: pkt_notify,
            })
            .is_ok()
        {
            err = 0;
        }
    }

    err
}

fn mesh_pair_complete_notify() -> i32 {
    let par = [CMD_NOTIFY_MESH_PAIR_END];
    // One byte always fits; the `Err` arm is defence in depth, not a routine
    // branch, and reports the same failure `mesh_cmd_notify` uses.
    match Vec::from_slice(&par) {
        Ok(msg) => mesh_cmd_notify(LGT_CMD_MESH_CMD_NOTIFY, &msg, device_address()),
        Err(_) => -1,
    }
}

/// Clear the pending credentials and mark pairing as settled.
fn safe_effect_new_mesh_finish() {
    {
        let mut pair = MESH_PAIR.lock();
        pair.new_mesh_name = [0; 16];
        pair.new_mesh_pwd = [0; 16];
        pair.new_mesh_ltk = [0; 16];
        pair.mesh_pair_start_time = 0;
    }
    set_pair_setting_flag(PairAuthState::PairSetted);
}

/// Drop the pending credentials and fall back to the persisted key.
///
/// This is the "we are still inside a default-mesh window (or the device
/// address has not been validated yet), so do not overwrite the pairing sector"
/// arm: it re-reads the saved key, re-initialises the node table and republishes
/// the lamp status.
fn reload_saved_key() {
    mesh_pair_complete_notify();
    sleep_us(1000);
    /* Switch to normal mesh */
    pair_load_key(); // flash read: MESH_PAIR is not held
    MESH_PAIR.lock().default_mesh_time_ref = 0;

    mesh_node_table_init();
    publish_light_status();
    safe_effect_new_mesh_finish();
}

/// Persist the credentials collected by the pairing state machine and use them.
///
/// Every value the flash path needs arrives in the action, so no pairing guard
/// is alive when `pair_save_key` erases and writes the pairing sector.
/// The credential state itself belongs to the radio layer: `apply_new_mesh_credentials`
/// writes it, and `use_pair_ltk` keeps the immediate-effect command's semantics
/// (the mesh LTK already held by the device is what gets saved, not the
/// buffered one). This function keeps only the mesh-layer bookkeeping: the
/// completion notification, the flash save, the radio re-tuning and the
/// application event.
///
/// Locking contract: nothing here runs under a held lock. The blocking flash
/// write, the application callback and the 1 ms settle delay are all unlocked;
/// only the two single-register RF writes take a critical section of their own.
/// The statement order is load-bearing — the new access code is installed
/// before the callback, and the stale-RX clear still follows the 1 ms settle
/// window.
fn save_and_apply(name: [u8; 16], pwd: [u8; 16], ltk: [u8; 16], use_pair_ltk: bool) {
    apply_new_mesh_credentials(&name, &pwd, &ltk, use_pair_ltk);

    mesh_pair_complete_notify();

    // make sure not receive legacy mesh data from now on
    pair_save_key(); // flash write, unlocked

    critical_section::with(|_| rf_set_ble_access_code(current_access_code())); // use new access code at once.

    dispatch_mesh_light_event(LGT_CMD_SET_MESH_INFO); // clear online status :mesh_node_table_init(), unlocked

    sleep_us(1000); // unlocked settle window for the new access code

    critical_section::with(|_| tlsr8266_radio::rf::drv_8266::rf_clear_rx_irq()); // clear current rx in buffer

    safe_effect_new_mesh_finish();
}

/// Point the radio at the configured default mesh and arm its timeout.
///
/// The window bookkeeping is the mesh layer's; the credentials (access code and
/// LTK, and deliberately not the name or password) are the radio layer's, so
/// the switch itself is delegated.
fn switch_to_default_mesh(delay_s: u8) {
    {
        let mut pair = MESH_PAIR.lock();
        pair.default_mesh_time_ref = clock_time() | 1;
        pair.default_mesh_time = delay_s as u32 * 1000;
    }

    /* Only change AC and LTK */
    switch_to_default_mesh_credentials();
}

/// Execute one timer-phase [`MeshPairAction`].
///
/// All three arms are deliberately here rather than inside the state machine:
/// they read or write flash, the radio state and the application hooks, and
/// every one of them runs with the pairing guard already dropped.
fn apply_timer_action(action: MeshPairAction) {
    match action {
        MeshPairAction::None => {}
        MeshPairAction::ReloadSavedKey => pair_load_key(),
        MeshPairAction::SwitchToDefaultMesh { delay_s } => {
            switch_to_default_mesh(delay_s);
            // Keep the pre-split behaviour: the reference is refreshed once more
            // after the access code has been recomputed.
            MESH_PAIR.lock().default_mesh_time_ref = clock_time() | 1;
        }
        MeshPairAction::TimedOut => {
            //mesh pair time out
            pair_load_key();
            set_pair_setting_flag(PairAuthState::PairSetted);
            dispatch_mesh_light_event(LGT_CMD_MESH_PAIR_TIMEOUT);
        }
        // The effect and broadcast phases execute their own arms.
        MeshPairAction::SaveAndApply { .. } | MeshPairAction::Broadcast(_) => {}
    }
}

/// Periodic pairing state-machine service, called from the BLE-app callback seam.
///
/// `#[inline(never)]` is load-bearing: `app::mesh_pair_proc_hook` wraps this
/// function, and `sdk/stack_analysis.py`'s `INDIRECT_EDGES` matches the
/// `on_mesh_pair_proc` edge by this symbol name. Inlining it into the wrapper
/// would erase it from the linked ELF.
///
/// Each phase takes its own short [`MESH_PAIR`] lock. The flash reads and writes
/// (`pair_load_key`, `pair_save_key`), the broadcast itself and the application
/// hooks all run with the guard dropped.
#[inline(never)]
pub fn mesh_pair_proc() {
    // Phase 1: the default-mesh window.
    let now = clock_time();
    let action = MESH_PAIR.lock().timer_step(now);
    apply_timer_action(action);

    // Phase 2: has the pairing window expired?
    let timed_out = MESH_PAIR.lock().timed_out(clock_time());
    if timed_out {
        apply_timer_action(MeshPairAction::TimedOut);
        return;
    }

    // Phase 3: advance the re-pair broadcast machine.
    let flag = pair_setting_flag();
    if flag as u8 >= PairAuthState::PairSetMeshTxStart as u8 {
        // The SDK rewrites the pairing credentials at runtime — a client sets
        // the new network name, password and LTK immediately before arming the
        // broadcast — so the copy taken at boot is refreshed here, outside the
        // pairing lock, and only while a broadcast sequence is live.
        let credentials = pair_credentials_snapshot();
        let (name, pwd, ltk) = (
            credentials.pair_nn,
            credentials.pair_pass,
            credentials.pair_ltk_mesh,
        );
        MESH_PAIR.lock().set_credentials(&name, &pwd, &ltk);
    }

    let now = clock_time();
    let (action, flag_after) = {
        let mut pair = MESH_PAIR.lock();
        pair.sync_flag(flag);
        let action = pair.interval_tick(now);
        (action, pair.pair_setting_flag())
    };
    // The state machine owns the flag's transitions; publish them only when one
    // happened, so a concurrent SDK write is never overwritten with a stale
    // copy.
    if flag_after != flag {
        set_pair_setting_flag(flag_after);
    }

    let Some(MeshPairAction::Broadcast(op_para)) = action else {
        return;
    };

    // 13 bytes always fit; a failure would only skip this pairing broadcast.
    if let Ok(msg) = Vec::from_slice(&op_para[..13]) {
        let pkt = light_slave_tx_command(&msg, 0xFFFF, 0, false);
        queue_mesh_packet(&pkt);
    }
}

/// Enable or disable mesh payload encryption.
pub fn mesh_security_enable(enable: bool) {
    set_security_enabled(enable);
}

/// Surface mesh TX queue overflow, one log line per change.
///
/// The SDK cannot log — it must not reach the application's UART sink — so it
/// only counts drops. Turning a counter delta into a line here keeps a
/// sustained overload visible from RTT and the UART host without a write per
/// iteration. The "last reported" mark is owned by the sender task.
fn report_tx_queue_drops(dropped_mesh_tx_seen: &mut u32) {
    let tx = queues::dropped_mesh_tx();
    if tx != *dropped_mesh_tx_seen {
        uprintln!(
            "mesh queue full: tx +{} (total {})",
            tx.wrapping_sub(*dropped_mesh_tx_seen),
            tx
        );
        *dropped_mesh_tx_seen = tx;
    }
}

/// Surface mesh RX queue overflow, one log line per change.
///
/// Both mesh tasks call this, so the "last reported" mark is module state
/// ([`LAST_REPORTED_RX_DROPS`]) rather than a task local. TC32 has no 32-bit
/// atomic read-modify-write, so the compare and the store share one
/// `critical_section::with`, the same idiom `queues::count_drop` uses: a task
/// that claims a delta advances the mark in the same section, so the other
/// task cannot log the same delta again. The line is written after the section
/// so the mask is never held across formatting or a queue push.
fn report_rx_queue_drops() {
    let rx = queues::dropped_mesh_rx();
    let delta = critical_section::with(|_| {
        let last = LAST_REPORTED_RX_DROPS.load(Ordering::Relaxed);
        if rx == last {
            None
        } else {
            LAST_REPORTED_RX_DROPS.store(rx, Ordering::Relaxed);
            Some(rx.wrapping_sub(last))
        }
    });

    if let Some(delta) = delta {
        uprintln!("mesh queue full: rx +{} (total {})", delta, rx);
    }
}

/// Outcome of the send-buffer scan, computed inside the send critical section.
enum SendScan {
    /// Nothing is queued: park on the wake signal until a producer pushes.
    Empty,
    /// Packets are queued but none is due: park until the earliest due tick.
    Wait(u64),
    /// The first due packet, removed from the buffer.
    Ready(SendPkt),
}

/// Process a single iteration of the mesh message sending task.
///
/// The scheduling scan is a three-state machine driven by one critical
/// section:
/// - [`SendScan::Empty`]: nothing queued; the buffer check happens before
///   `clock_time64()`, so host tests never touch MMIO. Park on
///   [`MESH_TX_WAKE`] until [`queues::enqueue_mesh_tx`] pushes a packet.
/// - [`SendScan::Wait`]: packets are queued but none is due yet; park on
///   [`MESH_TX_WAKE`] with a deadline of the earliest due tick.
/// - [`SendScan::Ready`]: the first due packet was removed and is
///   transmitted inline below.
///
/// A packet addressed to this device by this device is *report-only*: it is
/// handed straight to the UART bridge and neither transmitted nor
/// re-queued. Because that check runs here — in task context, after the pop
/// — the retransmit re-queue below can never loop a self-addressed packet.
///
/// The parked arms `return`, so the outer `loop` in [`mesh_packet_sender`]
/// re-enters and rescans instead of looping internally.
async fn mesh_packet_send_iteration(dropped_mesh_tx_seen: &mut u32) {
    report_tx_queue_drops(dropped_mesh_tx_seen);
    report_rx_queue_drops();

    let result = {
        // Reset before the scan: a push that lands after the scan leaves
        // the signal set, so the wake cannot be lost.
        MESH_TX_WAKE.reset();

        // Only the buffer scan runs with interrupts masked. `clock_time64`
        // nests one more critical section, exactly as before this queue
        // moved to module scope.
        let mut send_queue = MESH_TX_QUEUE.lock();

        if send_queue.is_empty() {
            SendScan::Empty
        } else {
            // Sample the clock exactly once for the whole scan.
            let now = clock_time64();

            match send_queue
                .iter()
                .enumerate()
                .find(|(_, elem)| now >= elem.delay)
            {
                Some((index, _)) => SendScan::Ready(send_queue.swap_remove(index)),
                None => match send_queue.iter().map(|e| e.delay).min() {
                    Some(earliest) => SendScan::Wait(earliest),
                    // Unreachable: `is_empty()` above gates this arm.
                    None => SendScan::Empty,
                },
            }
        }
    };

    let result = match result {
        SendScan::Empty => {
            MESH_TX_WAKE.wait().await;
            return;
        }
        SendScan::Wait(earliest) => {
            #[cfg(not(test))]
            let _ = MESH_TX_WAKE
                .wait()
                .with_deadline(Instant::from_ticks(earliest))
                .await;

            #[cfg(test)]
            {
                // Host tests have no time driver deadline; the wake signal
                // from `enqueue_mesh_tx` is the only exit.
                let _ = earliest;
                MESH_TX_WAKE.wait().await;
            }

            return;
        }
        SendScan::Ready(pkt) => pkt,
    };

    // Self-addressed packets are report-only: they exist so the host sees
    // locally generated commands, not so the radio repeats them.
    if result.pkt.ll_app().value.src == device_address()
        && result.pkt.ll_app().value.dst == device_address()
    {
        dispatch_uart_mesh_rx(&result.pkt);
        return;
    }

    let mut pkt = result.pkt;

    pkt.mesh_mut().src_tx = device_address();
    pkt.mesh_mut().handle1 = 0;

    // Encrypt the packet if required
    if is_security_enabled() {
        pkt.head_mut()._type |= BIT!(7);
        pair_enc_packet_mesh(&mut pkt);
    }

    // Pick a random start channel
    let start_chan_idx = tlsr8266_hal::mcu::random::rand() as usize;

    // Send the packet on each channel
    for channel_index in start_chan_idx..start_chan_idx + SYS_CHN_LISTEN.len() {
        // This block needs to operate in a critical section
        critical_section::with(|_| {
            // Configure the BLE mode
            rf_set_tx_rx_off();
            rf_set_ble_access_code(current_access_code());
            rf_set_ble_crc_adv();

            rf_set_ble_channel(SYS_CHN_LISTEN[channel_index % SYS_CHN_LISTEN.len()]);
            rf_start_srx2tx(
                addr_of!(pkt) as u32,
                clock_time() + CLOCK_SYS_CLOCK_1US * 30,
            );
        });

        // Let the transmission on this channel finish before hopping.
        // Host tests have no time driver, so they skip the spacing delay.
        #[cfg(not(test))]
        Timer::after(Duration::from_micros(600)).await;
    }

    rf_set_rxmode();

    if result.send_count != 0 {
        // Random delay to avoid congestion between 0us and 8ms
        let delay = 8000 - ((tlsr8266_hal::mcu::random::rand() % 16) * 500);

        queues::enqueue_mesh_tx(
            &result.pkt,
            clock_time64() + (delay as u64 * CLOCK_SYS_CLOCK_1US as u64),
            result.send_count - 1,
        );
    }
}

/// Mesh transmit task: owns [`MESH_TX_QUEUE`] and the TX drop-log bookkeeping.
#[coverage(off)]
#[embassy_executor::task]
pub async fn mesh_packet_sender() {
    let mut dropped_mesh_tx_seen: u32 = 0;

    #[cfg(not(test))]
    loop {
        mesh_packet_send_iteration(&mut dropped_mesh_tx_seen).await;
    }

    #[cfg(test)]
    {
        // In test builds, just run one iteration to allow testing
        mesh_packet_send_iteration(&mut dropped_mesh_tx_seen).await;
    }
}

/// Shape-classify one `MESH_RX_CHANNEL` packet in task context.
fn is_gatt_spp_rx_packet(packet: &Packet) -> bool {
    is_gatt_spp_write_packet(packet)
}

/// Process a single iteration of the mesh message receiving task.
///
/// RX queue overflow is reported here as well as in the send iteration: an idle
/// node may overflow [`MESH_RX_CHANNEL`] without ever running the send task.
async fn mesh_packet_rcv_iteration() {
    report_rx_queue_drops();

    let mut result = MESH_RX_CHANNEL.receive().await;

    if is_gatt_spp_rx_packet(&result) {
        rf_link_slave_data_write_task(&mut result);
    } else {
        rf_link_rc_data(&mut result);
    }
}

/// Mesh receive task: hands [`MESH_RX_CHANNEL`] packets to the mesh decoder.
#[coverage(off)]
#[embassy_executor::task]
pub async fn mesh_packet_rcv() {
    #[cfg(not(test))]
    loop {
        mesh_packet_rcv_iteration().await;
    }

    #[cfg(test)]
    {
        // In test builds, just run one iteration to allow testing
        mesh_packet_rcv_iteration().await;
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::light::transitions::LIGHT_CMD_CHANNEL;
    use core::sync::atomic::Ordering;
    use heapless::Vec;
    use tlsr8266_hal::common::sync::CriticalSectionMutexGuard;
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::clock::fake_clock;
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_pac::FLD_RF_IRQ_MASK;
    use tlsr8266_radio::ble::flash_config::access_code;
    use tlsr8266_radio::callbacks::{set_ble_app_hooks, BleAppHooks};
    use tlsr8266_radio::{
        set_ble_connected, set_current_access_code, set_dev_addr_validation_pending,
        set_device_address, set_flash_config_index, set_group_addresses, set_mesh_pair_enabled,
        set_pair_config_mesh_credentials, set_pair_credentials, set_paired_login,
        set_security_enabled,
    };

    use tlsr8266_radio::types::{PairAuthState, LGT_CMD_MESH_PAIR_TIMEOUT, LGT_CMD_SET_MESH_INFO};

    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../tests/fixtures/rust_parser.rs"
    ));

    /// `reg_system_tick`: the free-running tick `clock_time()` reads.
    const REG_SYSTEM_TICK: u32 = 0x740;
    /// `reg_rnd_number`: the hardware entropy source `rand()` XORs in.
    const REG_RND_NUMBER: u32 = 0x448;

    /// Ticks in one broadcast interval, at the HAL's 32 ticks/us.
    const INTERVAL_TICKS: u32 = MESH_PAIR_CMD_INTERVAL * 1000 * CLOCK_SYS_CLOCK_1US;

    /// The HAL host seams are process-wide singletons, so every test that
    /// observes them starts from a known-empty bank and tick count.
    fn reset_fakes() {
        FakeMmio::reset();
        fake_clock::reset();
        FakeFlash::reset();
    }

    /// Borrow the pairing state for one field access.
    ///
    /// Every use is a single statement (`pair().field = value;` or an
    /// assertion), so the temporary guard is always released before the next
    /// call that locks [`MESH_PAIR`].
    fn pair() -> CriticalSectionMutexGuard<'static, MeshPairFsm> {
        MESH_PAIR.lock()
    }

    /// Helper function to create a test packet with mesh data
    fn create_test_packet(src: u16, dst: u16, sno: [u8; 3]) -> Packet {
        Packet {
            att_cmd: PacketAttCmd {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: 0,
                    rf_len: 0,
                    l2cap_len: 0,
                    chan_id: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: PacketAttValue {
                    sno,
                    src: src.to_le_bytes(),
                    dst: dst.to_le_bytes(),
                    val: [0; 23],
                },
            },
        }
    }

    /// Arm the machine the way `mesh_pair_init` does, with a broadcast due.
    fn arm_pairing() {
        mesh_pair_init();
        fake_clock::set_ticks(1_000);
        pair().mesh_pair_time = 1_000;
    }

    /// `reg_dma_tx_rptr` / `reg_dma_tx_wptr`: the DMA TX ring pointers that
    /// `is_add_packet_buf_ready` and `rf_link_add_tx_packet` read.
    const REG_DMA_TX_RPTR: u32 = 0x52a;
    const REG_DMA_TX_WPTR: u32 = 0x52b;
    /// `reg_rf_channel`: written once per `rf_set_ble_channel` hop.
    const REG_RF_CHANNEL: u32 = 0x40d;
    /// `reg_dma_tx_fifo`: the DMA TX queue registration write.
    const REG_DMA_TX_FIFO: u32 = 0x52c;

    /// Leave `used` slots occupied in the DMA TX ring, then forget the writes
    /// that seeding produced.
    fn seed_tx_ring(used: u8) {
        FakeMmio::set_reg8(REG_DMA_TX_RPTR, 0);
        FakeMmio::set_reg8(REG_DMA_TX_WPTR, used);
        FakeMmio::clear_writes();
    }

    /// How many writes the bank recorded at `addr`.
    fn writes_at(addr: u32) -> usize {
        FakeMmio::writes()
            .iter()
            .filter(|write| write.addr == addr | tlsr8266_pac::REG_BASE_ADDR)
            .count()
    }

    /// The hook set the SDK's `on_uart_mesh_rx` dispatcher records into.
    ///
    /// The packet decoder's relay path is an indirect call registered by the
    /// composition root, so a test observes it exactly the way production does.
    mod relay {
        use super::Packet;
        use core::sync::atomic::{AtomicU32, Ordering};

        pub static PACKETS: AtomicU32 = AtomicU32::new(0);

        fn record(_pkt: &Packet) {
            PACKETS.fetch_add(1, Ordering::Relaxed);
        }

        pub fn hooks() -> super::BleAppHooks {
            super::BleAppHooks {
                on_mesh_pair_proc: noop,
                on_light_data: noop_light,
                on_light_response: |_, _| Err(()),
                on_uart_mesh_rx: record,
            }
        }

        fn noop() {}

        fn noop_light(_pkt: &Packet) {}

        /// Zero the counter and install the recording hooks.
        pub fn install() {
            PACKETS.store(0, Ordering::Relaxed);
            super::set_ble_app_hooks(hooks());
        }

        pub fn count() -> u32 {
            PACKETS.load(Ordering::Relaxed)
        }
    }

    /// The `on_light_event` hook that proves the pairing effect's callback runs
    /// with no critical section held and with the switch fully applied.
    ///
    /// `save_and_apply` used to wrap the flash write, the radio re-tune, the
    /// application callback and the 1 ms settle delay in one
    /// `critical_section::with`. The host build can now see that window:
    /// `tlsr8266_hal::common::sync::cs_depth` reads the HAL's one depth counter,
    /// which the host `critical_section` platform impl maintains. The probe
    /// records the depth it observed, so a guard held across the callback fails
    /// the test mechanically instead of merely being documented.
    ///
    /// It also pins the locks the pairing path owns (`MESH_PAIR`, `PAIR_STATE`
    /// through the facade, `MESH_EVENT_HOOKS`): none may be alive while the
    /// callback runs. The host `CriticalSectionMutex::lock` aborts on a
    /// re-entrant acquisition, so an inherited guard fails here too.
    mod unlocked_light_event {
        use super::*;
        use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

        pub static CALLS: AtomicU32 = AtomicU32::new(0);
        pub static LAST_CMD: AtomicU32 = AtomicU32::new(0);
        /// Access-code (`0x408`) writes already in the MMIO log when the hook ran.
        pub static ACCESS_CODE_WRITES: AtomicU32 = AtomicU32::new(0);
        /// Stale-RX (`0xf20`) writes already in the MMIO log when the hook ran.
        pub static RX_CLEAR_WRITES: AtomicU32 = AtomicU32::new(0);
        /// `cs_depth()` captured by the test just before it drives the effect.
        pub static BASE_DEPTH: AtomicU8 = AtomicU8::new(0);
        /// `cs_depth()` observed from inside the callback.
        pub static CALLBACK_DEPTH: AtomicU8 = AtomicU8::new(0);

        fn record(cmd: u8) {
            // Mechanical proof that the callback adds no critical-section level.
            // Compared against the captured baseline, not a literal `0`: a test
            // that panicked inside a `critical_section::with` earlier in this
            // process would leak a level, and only the delta is meaningful.
            let depth = tlsr8266_hal::common::sync::cs_depth();
            CALLBACK_DEPTH.store(depth, Ordering::Relaxed);
            assert_eq!(
                depth,
                BASE_DEPTH.load(Ordering::Relaxed),
                "the mesh-info callback must run outside any critical section"
            );

            CALLS.fetch_add(1, Ordering::Relaxed);
            LAST_CMD.store(cmd as u32, Ordering::Relaxed);
            // Statement order, observed from the inside: the radio has already
            // been retuned, and the stale-RX clear has not happened yet.
            ACCESS_CODE_WRITES.store(writes_at(0x408) as u32, Ordering::Relaxed);
            RX_CLEAR_WRITES.store(writes_at(0xf20) as u32, Ordering::Relaxed);

            // Every lock in the switch path must be free here. A guard inherited
            // across the call trips the host build's re-entrancy probe.
            drop(MESH_PAIR.lock());
            drop(MESH_EVENT_HOOKS.lock());
            let _ = pair_credentials_snapshot(); // PAIR_STATE
        }

        /// Zero the counters and install the probe in place of the default
        /// recording hooks. Call after `reset_for_test`, which reinstalls the
        /// default set, and store [`BASE_DEPTH`] before driving the effect.
        pub fn install() {
            CALLS.store(0, Ordering::Relaxed);
            LAST_CMD.store(0, Ordering::Relaxed);
            ACCESS_CODE_WRITES.store(0, Ordering::Relaxed);
            RX_CLEAR_WRITES.store(0, Ordering::Relaxed);
            BASE_DEPTH.store(0, Ordering::Relaxed);
            CALLBACK_DEPTH.store(0, Ordering::Relaxed);
            set_mesh_event_hooks(MeshEventHooks {
                on_light_event: record,
                ..test_hooks::hooks()
            });
        }
    }

    // --- send_mesh_message Tests ---

    /// A group address this node belongs to: the packet is delivered locally
    /// *and* re-transmitted, and the returned sequence number is the one
    /// `light_slave_tx_command` stamped into the constructed packet.
    #[test]
    fn compatibility_fixture_pins_ingress_routing_and_opcode_matrix() {
        let source = include_str!("../../../tests/fixtures/mesh_uart_compat.txt");
        let mut p = Parser::parse(source).unwrap();

        assert_eq!(p.str("mesh.spp_handle_21.ingress").unwrap(), "mesh_routing");
        assert_eq!(p.str("mesh.uart_02.ingress").unwrap(), "mesh_routing");
        assert_eq!(
            p.str("mesh.route.foreign_unicast").unwrap(),
            "queue_tx_only"
        );
        assert_eq!(
            p.str("mesh.route.self_unicast").unwrap(),
            "local_delivery_only"
        );
        assert_eq!(
            p.str("mesh.route.matching_group_or_broadcast").unwrap(),
            "local_delivery_and_queue_tx"
        );

        let payload = Vec::<u8, 13>::from_slice(&[
            LGT_CMD_LIGHT_ONOFF,
            0,
            0,
            LIGHT_ON_PARAM,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ])
        .unwrap();

        reset_for_test();
        set_device_address(0x1234);
        fake_clock::set_ticks(0x5000);
        let _ = send_mesh_message(&payload, 0x5678, 1, false);
        assert_eq!(MESH_RX_CHANNEL.len(), 0);
        assert_eq!(MESH_TX_QUEUE.lock().len(), 1);

        reset_for_test();
        set_device_address(0x1234);
        fake_clock::set_ticks(0x5000);
        let _ = send_mesh_message(&payload, 0x1234, 0, false);
        assert_eq!(MESH_RX_CHANNEL.len(), 1);
        assert_eq!(MESH_TX_QUEUE.lock().len(), 0);

        reset_for_test();
        set_device_address(0x1234);
        let mut groups = [0u16; MAX_GROUP_COUNT as usize];
        groups[0] = 0x8001;
        set_group_addresses(groups);
        fake_clock::set_ticks(0x5000);
        let _ = send_mesh_message(&payload, 0x8001, 1, false);
        assert_eq!(MESH_RX_CHANNEL.len(), 1);
        assert_eq!(MESH_TX_QUEUE.lock().len(), 1);

        use mesh_protocol::mesh_types as m;
        let matrix = [
            ("mesh.opcode.product.light_onoff", m::LGT_CMD_LIGHT_ONOFF),
            ("mesh.opcode.product.set_light", m::LGT_CMD_SET_LIGHT),
            (
                "mesh.opcode.product.light_read_status",
                m::LGT_CMD_LIGHT_READ_STATUS,
            ),
            ("mesh.opcode.product.light_status", m::LGT_CMD_LIGHT_STATUS),
            ("mesh.opcode.admin.notify_mesh", m::LGT_CMD_NOTIFY_MESH),
            ("mesh.opcode.admin.mesh_pair", m::LGT_CMD_MESH_PAIR),
            (
                "mesh.opcode.admin.mesh_cmd_notify",
                m::LGT_CMD_MESH_CMD_NOTIFY,
            ),
            ("mesh.opcode.admin.grp_rsp1", m::LGT_CMD_LIGHT_GRP_RSP1),
            ("mesh.opcode.admin.grp_rsp2", m::LGT_CMD_LIGHT_GRP_RSP2),
            ("mesh.opcode.admin.grp_rsp3", m::LGT_CMD_LIGHT_GRP_RSP3),
            ("mesh.opcode.admin.config_grp", m::LGT_CMD_LIGHT_CONFIG_GRP),
            ("mesh.opcode.admin.grp_req", m::LGT_CMD_LIGHT_GRP_REQ),
            (
                "mesh.opcode.admin.config_dev_addr",
                m::LGT_CMD_CONFIG_DEV_ADDR,
            ),
            ("mesh.opcode.admin.dev_addr_rsp", m::LGT_CMD_DEV_ADDR_RSP),
            ("mesh.opcode.admin.kick_out", m::LGT_CMD_KICK_OUT),
            ("mesh.opcode.admin.ota_start_req", m::LGT_CMD_START_OTA_REQ),
            ("mesh.opcode.admin.ota_start_rsp", m::LGT_CMD_START_OTA_RSP),
            ("mesh.opcode.admin.ota_data_req", m::LGT_CMD_OTA_DATA_REQ),
            ("mesh.opcode.admin.ota_data_rsp", m::LGT_CMD_OTA_DATA_RSP),
            ("mesh.opcode.admin.ota_end_req", m::LGT_CMD_END_OTA_REQ),
            ("mesh.opcode.admin.ota_end_rsp", m::LGT_CMD_END_OTA_RSP),
            (
                "mesh.opcode.admin.user_notify_req",
                m::LGT_CMD_USER_NOTIFY_REQ,
            ),
            (
                "mesh.opcode.admin.user_notify_rsp",
                m::LGT_CMD_USER_NOTIFY_RSP,
            ),
            ("mesh.opcode.admin.set_mac_addr", m::LGT_CMD_SET_MAC_ADDR),
            ("mesh.opcode.admin.power_on", m::LGT_POWER_ON),
            ("mesh.opcode.admin.panic_msg", m::LGT_PANIC_MSG),
            ("mesh.opcode.admin.light_ack", m::LGT_CMD_LIGHT_ACK),
            ("mesh.opcode.admin.set_mesh_info", m::LGT_CMD_SET_MESH_INFO),
            ("mesh.opcode.admin.set_dev_addr", m::LGT_CMD_SET_DEV_ADDR),
            ("mesh.opcode.admin.del_pair", m::LGT_CMD_DEL_PAIR),
            (
                "mesh.opcode.admin.mesh_pair_timeout",
                m::LGT_CMD_MESH_PAIR_TIMEOUT,
            ),
        ];
        for (key, actual) in matrix {
            assert_eq!(p.u8(key).unwrap(), actual, "opcode mismatch for {key}");
        }
    }

    #[test]
    fn test_send_mesh_message_group_match() {
        // Setup
        reset_for_test();
        set_device_address(0x1234);
        let mut groups = [0u16; MAX_GROUP_COUNT as usize];
        groups[0] = 0x8001;
        set_group_addresses(groups);
        // `light_slave_tx_command` seeds the sequence number with
        // `clock_time() + device_address()`: 0x5000 + 0x1234.
        fake_clock::set_ticks(0x5000);

        let test_data: Vec<u8, 13> =
            Vec::from_slice(&[1, 2, 3, 4, 5, 0, 0, 0, 0, 0, 0, 0, 0]).unwrap();

        // Execute
        let result = send_mesh_message(&test_data, 0x8001, 3, true);

        // Verify: the sequence number comes from the real packet, and a group
        // address is both handed to the receive side and re-transmitted.
        assert_eq!(result, [0x34, 0x62, 0x00]);
        let local = MESH_RX_CHANNEL
            .try_receive()
            .expect("a group-addressed packet is delivered locally");
        let local_dst = local.ll_app().value.dst;
        assert_eq!(local_dst, 0x8001);
        let tx = MESH_TX_QUEUE.lock();
        assert_eq!(tx.len(), 1, "a group address is re-transmitted");
        assert_eq!(tx[0].send_count, 3);
        let tx_dst = tx[0].pkt.ll_app().value.dst;
        assert_eq!(tx_dst, 0x8001);
    }

    /// This node's own unicast address: delivered locally, never re-transmitted.
    #[test]
    fn test_send_mesh_message_device_match() {
        // Setup
        reset_for_test();
        set_device_address(0x0001);
        fake_clock::set_ticks(0x5000);

        let test_data: Vec<u8, 13> =
            Vec::from_slice(&[1, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).unwrap();

        // Execute
        let result = send_mesh_message(&test_data, 0x0001, 0, false);

        // Verify - device match means no retransmission
        assert_eq!(result, [0x01, 0x50, 0x00]);
        let local = MESH_RX_CHANNEL
            .try_receive()
            .expect("a self-addressed packet is delivered locally");
        let local_dst = local.ll_app().value.dst;
        assert_eq!(local_dst, 0x0001);
        assert_eq!(
            MESH_TX_QUEUE.lock().len(),
            0,
            "a device-addressed packet has nowhere to be re-transmitted"
        );
    }

    /// Neither this device nor a member group: the packet only goes to the wire.
    #[test]
    fn test_send_mesh_message_no_match() {
        // Setup
        reset_for_test();
        set_device_address(0x0001);
        fake_clock::set_ticks(0x5000);

        let test_data: Vec<u8, 13> =
            Vec::from_slice(&[1, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).unwrap();

        // Execute
        let result = send_mesh_message(&test_data, 0x0003, 2, false);

        // Verify: no local delivery, one queued transmission.
        assert_eq!(result, [0x01, 0x50, 0x00]);
        assert_eq!(MESH_RX_CHANNEL.len(), 0);
        let tx = MESH_TX_QUEUE.lock();
        assert_eq!(tx.len(), 1);
        let tx_dst = tx[0].pkt.ll_app().value.dst;
        assert_eq!(tx_dst, 0x0003);
        assert_eq!(tx[0].send_count, 2);
    }

    // --- light_slave_tx_command Tests ---

    /// Integration test: verifies the full path from a UART LightCtrl payload, through
    /// `mesh_construct_packet` and `parse_ble_packet_op_params`, to
    /// `app::rf_link_data_callback` dispatching the light-on command.
    ///
    /// Guards against a regression where `mesh_construct_packet` was changed from a raw flat
    /// memory copy to explicit struct field assignments. `MeshPkt` is `repr(C, align(4))`, which
    /// inserts 1 byte of padding between `op` (offset 20) and `vendor_id` (offset 22). With
    /// struct field assignments, `vendor_id_lo` lands at offset 22 (the struct field) instead of
    /// offset 21 (what the `val[]` overlay reads). `parse_ble_packet_op_params` reads via `val[]`
    /// and sees `val[1] = padding = 0x00`, computing `vendor_id = 0x1100 ≠ VENDOR_ID`, causing
    /// `rf_link_data_callback` to silently discard the command on every node in the mesh.
    ///
    /// The fix is in construction: `mesh_construct_packet` uses a raw flat byte copy from
    /// `&pkt.op`, placing `vendor_id_lo` at offset 21 (padding slot) and `vendor_id_hi` at
    /// offset 22 — exactly where `val[1]` and `val[2]` read from.
    #[test]
    fn test_uart_light_ctrl_to_light_on_integration() {
        use tlsr8266_radio::mesh_ll::packet_processing::parse_ble_packet_op_params;

        crate::light::reset_for_test();
        set_device_address(0x1234);

        // Set the tick, which light_slave_tx_command uses to seed the mesh sequence number.
        fake_clock::set_ticks(0x5000_u64);

        // Step 1: Construct the 13-byte payload that the UART bridge extracts from a
        //         LightCtrl message's payload range. Layout: [op, vid_lo*, vid_hi*, par0..par9]
        //         (* vid bytes are overwritten by light_slave_tx_command with VENDOR_ID)
        let uart_payload: [u8; 13] = [
            LGT_CMD_LIGHT_ONOFF, // op = 0x10 (will become 0xd0 after |= 0xc0)
            0x00,                // vendor_id_lo placeholder — overwritten to VENDOR_ID lo
            0x00,                // vendor_id_hi placeholder — overwritten to VENDOR_ID hi
            LIGHT_ON_PARAM,      // par[0] = 0x01 (ON)
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];
        let destination: u16 = 0x1234; // Same as the stamped device address — targeting this device

        // Step 2: Build the packet as send_mesh_message does:
        //         light_slave_tx_command sets op |= 0xc0, fills VENDOR_ID bytes,
        //         then calls mesh_construct_packet which does a raw flat copy from &pkt.op.
        let payload_vec = Vec::<u8, 13>::from_slice(&uart_payload).unwrap();
        let packet = light_slave_tx_command(&payload_vec, destination, 0, false);

        // Step 3: Verify the wire-format bytes via the val[] overlay — the same view that
        //         parse_ble_packet_op_params uses. The raw copy writes:
        //           byte 20 (val[0]) = op | 0xc0
        //           byte 21 (val[1]) = VENDOR_ID & 0xFF = 0x11  ← vendor_id_lo in padding slot
        //           byte 22 (val[2]) = VENDOR_ID >> 8   = 0x02  ← vendor_id_hi
        //           byte 23 (val[3]) = LIGHT_ON_PARAM           ← par[0]
        //         If construction used struct fields instead, val[1] would be 0x00 (padding)
        //         and the vendor_id check in rf_link_data_callback would compute 0x1100 ≠ VENDOR_ID.
        let val = &packet.att_write().value.val;
        assert_eq!(val[0] & 0x3f, LGT_CMD_LIGHT_ONOFF, "val[0] must hold op");
        assert_eq!(
            val[1],
            (VENDOR_ID & 0xFF) as u8,
            "val[1] must hold vendor_id_lo (not padding)"
        );
        assert_eq!(
            val[2],
            (VENDOR_ID >> 8) as u8,
            "val[2] must hold vendor_id_hi"
        );
        assert_eq!(val[3], LIGHT_ON_PARAM, "val[3] must hold par[0]");

        // Step 4: Run the REAL parse_ble_packet_op_params (not mocked) end-to-end.
        let (success, op_cmd, op_len, params, _) = parse_ble_packet_op_params(&packet, true);
        assert!(success, "parse_ble_packet_op_params should succeed");
        assert_eq!(op_len, LightOpType::OpType3 as u8, "op_len must be 3");
        let parsed_vendor_id = (op_cmd[2] as u16) << 8 | op_cmd[1] as u16;
        assert_eq!(
            parsed_vendor_id, VENDOR_ID,
            "vendor_id reconstructed from op_cmd[1..3] must equal VENDOR_ID \
             (regression: struct field assignment leaves val[1]=padding=0x00 → vendor_id=0x1100)"
        );
        assert_eq!(
            op_cmd[0] & 0x3f,
            LGT_CMD_LIGHT_ONOFF,
            "op must be LGT_CMD_LIGHT_ONOFF"
        );
        assert_eq!(
            params[0], LIGHT_ON_PARAM,
            "params[0] must be LIGHT_ON_PARAM"
        );

        // Step 5: Run the REAL rf_link_data_callback (not mocked).
        //         With the bug: vendor_id check fails → early return → light never turns on.
        //         With the fix: vendor_id passes → LGT_CMD_LIGHT_ONOFF dispatched correctly.
        crate::app::rf_link_data_callback(&packet);

        let msg = LIGHT_CMD_CHANNEL
            .try_receive()
            .expect("LGT_CMD_LIGHT_ONOFF must be queued for the light task");
        assert_eq!(
            msg.cmd, LGT_CMD_LIGHT_ONOFF,
            "the parsed command must reach the light task"
        );
        assert_eq!(
            msg.params[0], LIGHT_ON_PARAM,
            "the UART payload must be passed through unchanged"
        );
    }

    // --- mesh_pair_init Tests ---

    /// Tests mesh_pair_init sets proper values
    #[test]
    fn test_mesh_pair_init() {
        // Setup
        reset_for_test();
        set_mesh_pair_enabled(false);

        // Execute
        mesh_pair_init();

        // Verify
        assert!(is_mesh_pair_enabled());
        assert_eq!(pair().mesh_pair_cmd_interval, MESH_PAIR_CMD_INTERVAL);
        assert_eq!(pair().mesh_pair_timeout, MESH_PAIR_TIMEOUT);
    }

    /// `mesh_pair_init` takes the credentials the broadcast machine will send
    /// from the SDK's pairing state, and the flag it gates on.
    #[test]
    fn test_mesh_pair_init_seeds_credentials_and_flag() {
        // Setup
        reset_for_test();
        let mut credentials = pair_credentials_snapshot();
        credentials.pair_nn = [1; 16];
        credentials.pair_pass = [2; 16];
        credentials.pair_ltk_mesh = [3; 16];
        set_pair_credentials(credentials);
        set_pair_setting_flag(PairAuthState::PairSetMeshTxStart);

        // Execute
        mesh_pair_init();

        // Verify
        assert_eq!(
            pair().pair_setting_flag(),
            PairAuthState::PairSetMeshTxStart
        );
        assert_eq!(pair().mesh_name, [1; 16]);
        assert_eq!(pair().mesh_pwd, [2; 16]);
        assert_eq!(pair().mesh_ltk, [3; 16]);
    }

    // --- mesh_event_hooks Tests ---

    /// Before the composition root registers the hooks every edge is a no-op
    /// rather than a call through a dangling pointer.
    #[test]
    fn test_event_hooks_are_inert_before_registration() {
        reset_for_test();
        *MESH_EVENT_HOOKS.lock() = None;

        dispatch_mesh_light_event(LGT_CMD_SET_MESH_INFO);
        publish_light_status();
        dispatch_uart_mesh_rx(&create_test_packet(1, 2, [3, 3, 3]));

        assert_eq!(test_hooks::LIGHT_EVENTS.load(Ordering::Relaxed), 0);
        assert_eq!(test_hooks::STATUS_UPDATES.load(Ordering::Relaxed), 0);
        assert_eq!(test_hooks::RELAYED_PACKETS.load(Ordering::Relaxed), 0);

        // And the recorded hooks start counting again once reinstalled.
        set_mesh_event_hooks(test_hooks::hooks());
        dispatch_mesh_light_event(LGT_CMD_SET_MESH_INFO);
        assert_eq!(test_hooks::LIGHT_EVENTS.load(Ordering::Relaxed), 1);
        assert_eq!(
            test_hooks::LAST_LIGHT_EVENT.load(Ordering::Relaxed),
            LGT_CMD_SET_MESH_INFO as u32
        );
    }

    // --- mesh_pair_proc_effect Tests ---

    /// The immediate-effect command: credentials are persisted, the radio is
    /// re-tuned to the new access code and the node table is cleared.
    #[test]
    fn test_mesh_pair_proc_effect_with_effect_new_mesh() {
        // Setup
        reset_for_test();
        set_dev_addr_validation_pending(false);
        set_mesh_pair_enabled(false);
        set_current_access_code(0x12345678);
        set_ble_connected(true);
        set_paired_login(true);
        set_device_address(0x0001);
        pair().effect_new_mesh = 1;

        reset_fakes();
        seed_tx_ring(0);
        FakeMmio::set_reg16(0xf20, 0);

        // Execute
        mesh_pair_proc_effect();

        // Verify: the credentials were written through the real save path at
        // the next 64-byte slot, and the switch cleared the pending RX interrupt.
        assert_eq!(pair().effect_new_mesh, 0);
        assert_eq!(pair().effect_new_mesh_delay_time, 0);
        assert_eq!(
            flash_config_index(),
            Some(0x40),
            "the real save path advances the wear-levelling slot"
        );
        let mut header = [0u8; 16];
        FakeFlash::read_slice(FLASH_ADR_PAIRING + 0x40, &mut header);
        assert_eq!(header[0], PAIR_CONFIG_VALID_FLAG);
        assert_eq!(header[15], PAIR_CONFIG_VALID_FLAG);
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_PAIRING),
            0,
            "a slot inside the sector is written without erasing"
        );
        assert_eq!(
            test_hooks::LAST_LIGHT_EVENT.load(Ordering::Relaxed),
            LGT_CMD_SET_MESH_INFO as u32,
            "the mesh switch must clear the online status through the hook"
        );
        assert_eq!(
            FakeMmio::reg16(0xf20),
            FLD_RF_IRQ_MASK::IRQ_RX.bits(),
            "the pending RX interrupt must be acknowledged during the mesh switch"
        );
    }

    /// The delayed-effect window expired while the device is still inside a
    /// default-mesh window: the persisted key is re-read instead.
    #[test]
    fn test_mesh_pair_proc_effect_with_delay_exceeded() {
        // Setup
        reset_for_test();
        set_dev_addr_validation_pending(false);
        set_mesh_pair_enabled(false);
        set_current_access_code(0x12345678);
        set_ble_connected(true);
        set_paired_login(true);
        set_device_address(0x0001);

        // The blank pairing sector means `pair_load_key` falls back to the
        // configured default credentials; seed distinct ones so the reload is
        // observable rather than looking like untouched state.
        set_pair_config_mesh_credentials([0xA1; 16], [0xA2; 16], [0xA3; 16]);

        pair().effect_new_mesh = 0;
        pair().effect_new_mesh_delay_time = 1000;
        pair().mesh_pair_cmd_interval = 500;

        reset_fakes();
        // The 500 ms interval (16_000_000 ticks) has elapsed since the delay ref.
        fake_clock::set_ticks(1000 + 500 * 1000 * 32 + 1);
        seed_tx_ring(0);
        FakeMmio::set_reg16(0xf20, 0);

        // Execute
        mesh_pair_proc_effect();

        // Verify
        assert_eq!(pair().effect_new_mesh_delay_time, 0);
        assert_eq!(
            FakeMmio::reg16(0xf20),
            FLD_RF_IRQ_MASK::IRQ_RX.bits(),
            "the pending RX interrupt must be acknowledged during the mesh switch"
        );
        assert_eq!(
            flash_config_index(),
            Some(0x40),
            "the expired window persists the buffered credentials"
        );
    }

    /// Tests mesh_pair_proc_effect with no effect needed
    #[test]
    fn test_mesh_pair_proc_effect_no_effect() {
        // Setup
        reset_for_test();
        pair().effect_new_mesh = 0;
        pair().effect_new_mesh_delay_time = 1000;
        pair().mesh_pair_cmd_interval = 500;

        reset_fakes();
        // Park the clock at the delay reference: nothing has elapsed yet.
        fake_clock::set_ticks(1000);

        // Execute
        mesh_pair_proc_effect();

        // Verify - nothing should change
        assert_eq!(pair().effect_new_mesh_delay_time, 1000);
    }

    // --- mesh_device_address_validation_completed Tests ---

    /// Tests mesh_device_address_validation_completed with mesh pair enabled
    #[test]
    fn test_mesh_device_address_validation_completed_pair_enabled() {
        // Setup
        reset_fakes();
        reset_for_test();
        set_dev_addr_validation_pending(true);
        set_mesh_pair_enabled(true);
        set_flash_config_index(10);

        // Execute
        mesh_device_address_validation_completed();

        // Verify: the validation flag byte in the stored header is cleared.
        assert!(!dev_addr_validation_pending());
        let addr = (FLASH_ADR_PAIRING as i32 + 10 + 1) as u32;
        let mut cleared = [0xFFu8; 1];
        FakeFlash::read_slice(addr, &mut cleared);
        assert_eq!(
            cleared[0], 0x00,
            "byte [1] of the config header must be programmed to zero"
        );
    }

    /// Tests mesh_device_address_validation_completed without mesh pair
    #[test]
    fn test_mesh_device_address_validation_completed_pair_disabled() {
        // Setup
        reset_for_test();
        set_dev_addr_validation_pending(true);
        set_mesh_pair_enabled(false);

        // Execute
        mesh_device_address_validation_completed();

        // Verify
        assert!(!dev_addr_validation_pending());
    }

    // --- mesh_pair_cb Tests ---

    /// Tests mesh_pair_cb with MeshPairName1
    #[test]
    fn test_mesh_pair_cb_name1() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]).unwrap();
        fake_clock::set_ticks(1000);

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(pair().mesh_pair_start_time, 1001);
        assert_eq!(&pair().new_mesh_name[0..8], &[1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(
            test_hooks::LIGHT_EVENTS.load(Ordering::Relaxed),
            0,
            "a received chunk must not touch flash, the radio or the LED"
        );
    }

    /// Tests mesh_pair_cb with MeshPairName2
    #[test]
    fn test_mesh_pair_cb_name2() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[1, 11, 12, 13, 14, 15, 16, 17, 18]).unwrap();

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(
            &pair().new_mesh_name[8..16],
            &[11, 12, 13, 14, 15, 16, 17, 18]
        );
    }

    /// Tests mesh_pair_cb with MeshPairPwd1
    #[test]
    fn test_mesh_pair_cb_pwd1() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[2, 21, 22, 23, 24, 25, 26, 27, 28]).unwrap();

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(
            &pair().new_mesh_pwd[0..8],
            &[21, 22, 23, 24, 25, 26, 27, 28]
        );
    }

    /// Tests mesh_pair_cb with MeshPairPwd2
    #[test]
    fn test_mesh_pair_cb_pwd2() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[3, 31, 32, 33, 34, 35, 36, 37, 38]).unwrap();

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(
            &pair().new_mesh_pwd[8..16],
            &[31, 32, 33, 34, 35, 36, 37, 38]
        );
    }

    /// Tests mesh_pair_cb with MeshPairLtk1
    #[test]
    fn test_mesh_pair_cb_ltk1() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[4, 41, 42, 43, 44, 45, 46, 47, 48]).unwrap();

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(
            &pair().new_mesh_ltk[0..8],
            &[41, 42, 43, 44, 45, 46, 47, 48]
        );
    }

    /// Tests mesh_pair_cb with MeshPairLtk2
    #[test]
    fn test_mesh_pair_cb_ltk2() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[5, 51, 52, 53, 54, 55, 56, 57, 58]).unwrap();

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(
            &pair().new_mesh_ltk[8..16],
            &[51, 52, 53, 54, 55, 56, 57, 58]
        );
    }

    /// Tests mesh_pair_cb with MeshPairEffectDelay
    #[test]
    fn test_mesh_pair_cb_effect_delay() {
        // Setup
        reset_for_test();
        pair().default_mesh_time_ref = 500;
        pair().mesh_pair_cmd_interval = 250;
        let params: Vec<u8, 10> = Vec::from_slice(&[6, 0, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        fake_clock::set_ticks(2000);

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(pair().effect_new_mesh_delay_time, 2001);
        assert_eq!(pair().default_mesh_time, 500); // 250 * 2
    }

    /// Tests mesh_pair_cb with MeshPairEffect
    #[test]
    fn test_mesh_pair_cb_effect() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[7, 0, 0, 0, 0, 0, 0, 0, 0]).unwrap();

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(pair().effect_new_mesh, 1);
    }

    /// Tests mesh_pair_cb with MeshPairDefaultMesh
    #[test]
    fn test_mesh_pair_cb_default_mesh() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[8, 5, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        fake_clock::set_ticks(3000);

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(pair().default_mesh_effect_delay_ref, 3001);
        assert_eq!(pair().default_mesh_time, 5000); // 5 * 1000
    }

    /// Tests mesh_pair_cb with invalid command
    #[test]
    fn test_mesh_pair_cb_invalid_command() {
        // Setup
        reset_for_test();
        pair().default_mesh_time_ref = 100;
        let params: Vec<u8, 10> = Vec::from_slice(&[99, 0, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        fake_clock::set_ticks(4000);

        // Execute - should return early without panic
        mesh_pair_cb(&params);

        // Verify - no credential state should change, but the command still
        // keeps the default-mesh window alive (it always did).
        assert_eq!(pair().mesh_pair_start_time, 0);
        assert_eq!(pair().default_mesh_time_ref, 4001);
        assert_eq!(pair().new_mesh_name, [0; 16]);
    }

    /// Tests mesh_pair_cb updates default_mesh_time_ref when it's set
    #[test]
    fn test_mesh_pair_cb_updates_default_mesh_time_ref() {
        // Setup
        reset_for_test();
        pair().default_mesh_time_ref = 100;
        let params: Vec<u8, 10> = Vec::from_slice(&[1, 1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        fake_clock::set_ticks(5000);

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(pair().default_mesh_time_ref, 5001);
    }

    // --- mesh_cmd_notify Tests ---

    /// Tests mesh_cmd_notify when BLE is not connected
    #[test]
    fn test_mesh_cmd_notify_ble_not_connected() {
        // Setup
        reset_for_test();
        set_ble_connected(false);
        set_paired_login(true);
        set_device_address(0x1234);

        let params: Vec<u8, 10> = Vec::from_slice(&[1, 2, 3]).unwrap();

        // Execute
        let result = mesh_cmd_notify(0x10, &params, 0x1234);

        // Verify
        assert_eq!(result, -1);
    }

    /// Tests mesh_cmd_notify when pair login is not ok
    #[test]
    fn test_mesh_cmd_notify_pair_login_not_ok() {
        // Setup
        reset_for_test();
        set_ble_connected(true);
        set_paired_login(false);
        set_device_address(0x1234);

        let params: Vec<u8, 10> = Vec::from_slice(&[1, 2, 3]).unwrap();

        // Execute
        let result = mesh_cmd_notify(0x10, &params, 0x1234);

        // Verify
        assert_eq!(result, -1);
    }

    /// `Vec<u8, 10>` makes the `p.len() > 10` guard unreachable, so a full
    /// ten-byte parameter block is still accepted and queued.
    #[test]
    fn test_mesh_cmd_notify_params_too_long() {
        // Setup
        reset_for_test();
        set_ble_connected(true);
        set_paired_login(true);
        set_device_address(0x1234);

        let params: Vec<u8, 10> = Vec::from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]).unwrap();

        reset_fakes();
        seed_tx_ring(0);

        // Execute
        let result = mesh_cmd_notify(0x10, &params, 0x1234);

        // With Vec<u8, 10>, the length can never exceed 10, so the early return
        // is unreachable and the notification is queued instead.
        assert_eq!(result, 0);
        assert_eq!(writes_at(REG_DMA_TX_FIFO), 2, "empty marker plus packet");
    }

    /// Tests mesh_cmd_notify with successful notification
    #[test]
    fn test_mesh_cmd_notify_success() {
        // Setup
        reset_for_test();
        set_ble_connected(true);
        set_paired_login(true);
        set_device_address(0x1234);

        let params: Vec<u8, 10> = Vec::from_slice(&[1, 2, 3]).unwrap();

        reset_fakes();
        seed_tx_ring(0);

        // Execute
        let result = mesh_cmd_notify(0x10, &params, 0x1234);

        // Verify: the notification reached the DMA TX queue.
        assert_eq!(result, 0);
        assert_eq!(writes_at(REG_DMA_TX_FIFO), 2, "empty marker plus packet");
    }

    /// Tests mesh_cmd_notify when the DMA ring has no free slot.
    #[test]
    fn test_mesh_cmd_notify_buffer_not_ready() {
        // Setup
        reset_for_test();
        set_ble_connected(true);
        set_paired_login(true);
        set_device_address(0x1234);

        let params: Vec<u8, 10> = Vec::from_slice(&[1, 2, 3]).unwrap();

        reset_fakes();
        // Three slots in use: `is_add_packet_buf_ready` reports false.
        seed_tx_ring(3);

        // Execute
        let result = mesh_cmd_notify(0x10, &params, 0x1234);

        // Verify
        assert_eq!(result, -1);
        assert_eq!(writes_at(REG_DMA_TX_FIFO), 0, "nothing may be queued");
    }

    // --- mesh_pair_complete_notify Tests ---

    /// Tests mesh_pair_complete_notify
    #[test]
    fn test_mesh_pair_complete_notify() {
        // Setup
        reset_for_test();
        set_ble_connected(true);
        set_paired_login(true);
        set_device_address(0x5678);

        reset_fakes();
        seed_tx_ring(0);

        // Execute
        let result = mesh_pair_complete_notify();

        // Verify: the completion notification reached the DMA TX queue.
        assert_eq!(result, 0);
        assert_eq!(writes_at(REG_DMA_TX_FIFO), 2, "empty marker plus packet");
    }

    // --- safe_effect_new_mesh_finish Tests ---

    /// Tests safe_effect_new_mesh_finish clears state
    #[test]
    fn test_safe_effect_new_mesh_finish() {
        // Setup
        reset_for_test();
        pair().new_mesh_name = [1; 16];
        pair().new_mesh_pwd = [2; 16];
        pair().new_mesh_ltk = [3; 16];
        pair().mesh_pair_start_time = 1000;
        set_pair_setting_flag(PairAuthState::PairSetting);

        // Execute
        safe_effect_new_mesh_finish();

        // Verify
        assert_eq!(pair().new_mesh_name, [0; 16]);
        assert_eq!(pair().new_mesh_pwd, [0; 16]);
        assert_eq!(pair().new_mesh_ltk, [0; 16]);
        assert_eq!(pair().mesh_pair_start_time, 0);
        assert_eq!(pair_setting_flag(), PairAuthState::PairSetted);
    }

    // --- save / reload path Tests ---

    /// Device back in its default-mesh window: the pairing sector is left alone
    /// and the persisted key is re-read instead.
    #[test]
    fn test_save_effect_new_mesh_default_mesh_active() {
        // Setup
        reset_for_test();
        set_dev_addr_validation_pending(false);
        set_ble_connected(true);
        set_paired_login(true);
        set_device_address(0x0001);
        // Distinct configured defaults, so the real `pair_load_key` fallback is
        // observable rather than looking like untouched state.
        set_pair_config_mesh_credentials([0xB1; 16], [0xB2; 16], [0xB3; 16]);
        pair().default_mesh_time_ref = 1000; // Set to non-zero
                                             // The delayed-effect window has expired, so the save is due.
        pair().effect_new_mesh_delay_time = 1000;
        pair().mesh_pair_cmd_interval = 500;
        reset_fakes();
        fake_clock::set_ticks(1000 + INTERVAL_TICKS as u64 + 1);
        seed_tx_ring(0);

        // Execute
        mesh_pair_proc_effect();

        // Verify: no slot was consumed in the pairing sector, the saved key was
        // reloaded and the node table was rebuilt around this device.
        assert_eq!(pair().default_mesh_time_ref, 0);
        assert_eq!(pair().mesh_pair_start_time, 0);
        assert_eq!(
            flash_config_index(),
            None,
            "a blank pairing sector has no configuration record"
        );
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_PAIRING),
            0,
            "no erase may happen inside a default-mesh window"
        );
        assert_eq!(
            pair_credentials_snapshot().pair_nn,
            [0xB1; 16],
            "the persisted key is re-read"
        );
        assert_eq!(
            test_hooks::STATUS_UPDATES.load(Ordering::Relaxed),
            1,
            "the lamp status is republished through the registered hook"
        );
    }

    /// Device address validation still pending: the pairing sector is left alone.
    #[test]
    fn test_save_effect_new_mesh_validation_pending() {
        // Setup
        reset_for_test();
        set_dev_addr_validation_pending(true);
        set_ble_connected(true);
        set_paired_login(true);
        set_device_address(0x0001);
        set_pair_config_mesh_credentials([0xC1; 16], [0xC2; 16], [0xC3; 16]);
        pair().default_mesh_time_ref = 0;
        pair().effect_new_mesh_delay_time = 1000;
        pair().mesh_pair_cmd_interval = 500;
        reset_fakes();
        fake_clock::set_ticks(1000 + INTERVAL_TICKS as u64 + 1);
        seed_tx_ring(0);

        // Execute
        mesh_pair_proc_effect();

        // Verify: the persisted key is re-read and nothing is written.
        assert_eq!(
            flash_config_index(),
            None,
            "a pending address validation must not write the pairing sector"
        );
        assert_eq!(pair_credentials_snapshot().pair_nn, [0xC1; 16]);
    }

    /// Nothing in the way: the collected credentials are written and used.
    #[test]
    fn test_save_effect_new_mesh_normal_save() {
        // Setup
        reset_for_test();
        set_dev_addr_validation_pending(false);
        set_ble_connected(true);
        set_paired_login(true);
        set_device_address(0x0001);
        set_current_access_code(0xABCDEF12);

        pair().default_mesh_time_ref = 0;
        pair().effect_new_mesh = 0;
        pair().effect_new_mesh_delay_time = 1000;
        pair().mesh_pair_cmd_interval = 500;
        pair().new_mesh_name = [1; 16];
        pair().new_mesh_pwd = [2; 16];
        pair().new_mesh_ltk = [3; 16];

        reset_fakes();
        fake_clock::set_ticks(1000 + INTERVAL_TICKS as u64 + 1);
        seed_tx_ring(0);
        FakeMmio::set_reg16(0xf20, 0);

        // Execute
        mesh_pair_proc_effect();

        // Verify: the new credentials landed in the next pairing slot and the
        // radio was re-tuned to the recomputed access code.
        assert_eq!(pair().mesh_pair_start_time, 0);
        assert_eq!(
            flash_config_index(),
            Some(0x40),
            "the real save path advances the wear-levelling slot"
        );
        let mut slot = [0u8; 16];
        FakeFlash::read_slice(FLASH_ADR_PAIRING + 0x50, &mut slot);
        assert_eq!(slot, [1; 16], "the mesh name is persisted");
        assert_eq!(
            test_hooks::LAST_LIGHT_EVENT.load(Ordering::Relaxed),
            LGT_CMD_SET_MESH_INFO as u32,
            "the switch clears the online status through the registered hook"
        );
        let credentials = pair_credentials_snapshot();
        assert_eq!(
            credentials.pair_ltk, [3; 16],
            "use_pair_ltk == false writes the received LTK"
        );
        assert_eq!(credentials.pair_nn, [1; 16]);
        assert_eq!(credentials.pair_pass, [2; 16]);
        assert_eq!(
            current_access_code(),
            access_code(&[1; 16], &[2; 16]),
            "the access code is recomputed from the credentials just saved"
        );
        assert_eq!(
            FakeMmio::reg16(0xf20),
            FLD_RF_IRQ_MASK::IRQ_RX.bits(),
            "the pending RX interrupt must be acknowledged during the mesh switch"
        );
    }

    /// The pairing effect fires the application callback with no pairing lock
    /// and no critical section held, and still saves the credentials and
    /// re-tunes the radio.
    ///
    /// The stale-RX clear and the access-code write are each their own short
    /// critical section; the flash write, the callback and the 1 ms settle
    /// delay between them run unlocked. The probe installed as `on_light_event`
    /// asserts that the HAL's critical-section depth it inherits equals the
    /// baseline captured just before the effect was driven (so the callback adds
    /// no level), and takes every pairing-path mutex from inside the callback,
    /// which the host build only allows when no guard was inherited. The
    /// assertions below pin the effects on either side of it (persisted
    /// credentials, access-code register, stale-RX acknowledgement).
    #[test]
    fn test_save_effect_new_mesh_callback_runs_without_locks() {
        // Setup: the deterministic normal-save arm, with the probe hook.
        reset_for_test();
        set_dev_addr_validation_pending(false);
        set_ble_connected(true);
        set_paired_login(true);
        set_device_address(0x0001);
        set_current_access_code(0xABCDEF12);
        unlocked_light_event::install();

        pair().default_mesh_time_ref = 0;
        pair().effect_new_mesh = 0;
        pair().effect_new_mesh_delay_time = 1000;
        pair().mesh_pair_cmd_interval = 500;
        pair().new_mesh_name = [1; 16];
        pair().new_mesh_pwd = [2; 16];
        pair().new_mesh_ltk = [3; 16];

        reset_fakes();
        fake_clock::set_ticks(1000 + INTERVAL_TICKS as u64 + 1);
        seed_tx_ring(0);
        FakeMmio::set_reg16(0xf20, 0);

        // Execute
        let base_depth = tlsr8266_hal::common::sync::cs_depth();
        unlocked_light_event::BASE_DEPTH.store(base_depth, Ordering::Relaxed);
        mesh_pair_proc_effect();

        // Verify: the callback ran exactly once, received the mesh-info event,
        // and inherited neither a pairing lock nor a critical section. The
        // counters prove the probe was actually reached.
        assert_eq!(
            unlocked_light_event::CALLBACK_DEPTH.load(Ordering::Relaxed),
            base_depth,
            "the callback must observe the baseline critical-section depth"
        );
        assert_eq!(
            unlocked_light_event::CALLS.load(Ordering::Relaxed),
            1,
            "the mesh-info callback must run exactly once"
        );
        assert_eq!(
            unlocked_light_event::LAST_CMD.load(Ordering::Relaxed),
            LGT_CMD_SET_MESH_INFO as u32,
            "the callback receives the mesh-info command"
        );
        // Statement order, from the callback's own viewpoint: the access code
        // was already installed, the stale-RX clear had not run yet.
        assert_eq!(
            unlocked_light_event::ACCESS_CODE_WRITES.load(Ordering::Relaxed),
            1,
            "the radio is retuned before the application is told"
        );
        assert_eq!(
            unlocked_light_event::RX_CLEAR_WRITES.load(Ordering::Relaxed),
            0,
            "the stale-RX clear still follows the callback and its settle window"
        );

        // The credentials were still persisted: the real save path advanced the
        // wear-levelling slot and wrote the network name.
        assert_eq!(
            flash_config_index(),
            Some(0x40),
            "the save path still advances the wear-levelling slot"
        );
        let mut slot = [0u8; 16];
        FakeFlash::read_slice(FLASH_ADR_PAIRING + 0x50, &mut slot);
        assert_eq!(slot, [1; 16], "the mesh name is persisted");

        // The radio was re-tuned: `rf_set_ble_access_code` byte-swaps the new
        // access code into `reg_rf_access_code` (0x408), exactly once.
        assert_eq!(
            writes_at(0x408),
            1,
            "the access code register is written once, inside its own critical section"
        );
        assert_eq!(
            FakeMmio::reg32(0x408),
            access_code(&[1; 16], &[2; 16]).swap_bytes(),
            "the register holds the newly recomputed access code"
        );

        // The stale-RX clear also happened, and only after the settle window.
        assert_eq!(
            writes_at(0xf20),
            1,
            "the pending RX interrupt is acknowledged once"
        );
        assert_eq!(
            FakeMmio::reg16(0xf20),
            FLD_RF_IRQ_MASK::IRQ_RX.bits(),
            "the pending RX interrupt is acknowledged during the mesh switch"
        );
    }

    /// The receive-only immediate-effect command saves the mesh LTK the device
    /// already holds, not the one it just buffered.
    #[test]
    fn test_save_effect_new_mesh_uses_the_mesh_ltk_for_the_immediate_effect() {
        // Setup
        reset_for_test();
        set_dev_addr_validation_pending(false);
        set_ble_connected(true);
        set_paired_login(true);
        set_device_address(0x0001);
        set_current_access_code(0xABCDEF12);
        let mut credentials = pair_credentials_snapshot();
        credentials.pair_ltk_mesh = [9; 16];
        set_pair_credentials(credentials);

        pair().default_mesh_time_ref = 0;
        pair().effect_new_mesh = 1;
        pair().new_mesh_ltk = [3; 16];

        reset_fakes();
        seed_tx_ring(0);

        // Execute
        mesh_pair_proc_effect();

        // Verify
        assert_eq!(pair_credentials_snapshot().pair_ltk, [9; 16]);
        // The persisted LTK slot is the fourth block of the written record.
        let mut ltk = [0u8; 16];
        FakeFlash::read_slice(FLASH_ADR_PAIRING + 0x70, &mut ltk);
        assert_eq!(ltk, [9; 16], "the saved LTK is the held mesh LTK");
    }

    // --- switch_to_default_mesh Tests ---

    /// Tests switch_to_default_mesh: the window bookkeeping is the mesh layer's
    /// and the access code / LTK are recomputed from the configured defaults.
    #[test]
    fn test_switch_to_default_mesh() {
        // Setup
        reset_for_test();
        let test_name = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
        let test_pwd = [
            17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
        ];
        let test_ltk = [
            33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
        ];

        set_pair_config_mesh_credentials(test_name, test_pwd, test_ltk);
        // A sentinel, so "the code was recomputed" cannot be confused with
        // "the code was left alone".
        set_current_access_code(0x1111_1111);

        fake_clock::set_ticks(5000);

        // Execute
        switch_to_default_mesh(10);

        // Verify
        assert_eq!(pair().default_mesh_time_ref, 5001);
        assert_eq!(pair().default_mesh_time, 10000);
        assert_eq!(
            current_access_code(),
            access_code(&test_name, &test_pwd),
            "the access code is recomputed from the configured defaults"
        );
        assert_eq!(pair_credentials_snapshot().pair_ltk, test_ltk);
    }

    // --- mesh_pair_proc Tests ---

    /// Tests mesh_pair_proc with timeout: the persisted key is re-read and the
    /// application is told through the event hook.
    #[test]
    fn test_mesh_pair_proc_timeout() {
        // Setup
        reset_for_test();
        set_pair_config_mesh_credentials([0xD1; 16], [0xD2; 16], [0xD3; 16]);
        pair().mesh_pair_start_time = 1000;
        pair().mesh_pair_timeout = 10;
        set_pair_setting_flag(PairAuthState::PairSetting);

        reset_fakes();
        // 10 s (320_000_000 ticks) have elapsed since the pairing window opened.
        fake_clock::set_ticks(1000 + 10 * 1000 * 1000 * 32 + 1);

        // Execute
        mesh_pair_proc();

        // Verify
        assert_eq!(pair_setting_flag(), PairAuthState::PairSetted);
        assert_eq!(
            pair_credentials_snapshot().pair_nn,
            [0xD1; 16],
            "the timeout re-reads the persisted key"
        );
        assert_eq!(
            test_hooks::LAST_LIGHT_EVENT.load(Ordering::Relaxed),
            LGT_CMD_MESH_PAIR_TIMEOUT as u32,
            "the timeout event reaches the application through the hook"
        );
        assert_eq!(test_hooks::LIGHT_EVENTS.load(Ordering::Relaxed), 1);
    }

    /// Tests mesh_pair_proc default mesh effect delay
    #[test]
    fn test_mesh_pair_proc_default_mesh_effect_delay() {
        // Setup
        reset_for_test();
        set_pair_config_mesh_credentials([0xE1; 16], [0xE2; 16], [0xE3; 16]);
        pair().default_mesh_effect_delay_ref = 1000;
        pair().default_mesh_time = 0;
        pair().mesh_pair_start_time = 0;
        pair().default_mesh_time_ref = 0; // Ensure we don't skip the condition

        reset_fakes();
        // The pairing command interval (500 ms = 16_000_000 ticks) has elapsed.
        fake_clock::set_ticks(1000 + MESH_PAIR_CMD_INTERVAL as u64 * 1000 * 32 + 1);

        // Execute
        mesh_pair_proc();

        // Verify
        assert_eq!(pair().default_mesh_effect_delay_ref, 0);
        assert_eq!(pair().default_mesh_time_ref, 0);
        assert_eq!(
            pair_credentials_snapshot().pair_nn,
            [0xE1; 16],
            "the reload re-reads the persisted key"
        );
    }

    /// Tests mesh_pair_proc default mesh effect delay with switch
    #[test]
    fn test_mesh_pair_proc_default_mesh_effect_delay_with_switch() {
        // Setup
        reset_for_test();
        let name = [0xF1; 16];
        let pwd = [0xF2; 16];
        set_pair_config_mesh_credentials(name, pwd, [0xF3; 16]);
        set_current_access_code(0x1111_1111);
        pair().default_mesh_effect_delay_ref = 1000;
        pair().default_mesh_time = 5000;
        pair().mesh_pair_start_time = 0;

        reset_fakes();
        // The first read drives the interval predicate (must be past 500 ms); the
        // next two stamp the switch reference (once inside the switch, once in
        // the refresh that follows it).
        FakeMmio::on_read32(
            REG_SYSTEM_TICK,
            |call_idx| {
                if call_idx == 0 {
                    u32::MAX
                } else {
                    2000
                }
            },
        );

        // Execute
        mesh_pair_proc();

        // Verify
        assert_eq!(pair().default_mesh_effect_delay_ref, 0);
        assert_eq!(pair().default_mesh_time_ref, 2001);
        assert_eq!(
            current_access_code(),
            access_code(&name, &pwd),
            "the switch recomputes the access code from the configured defaults"
        );
    }

    /// Tests mesh_pair_proc default mesh timeout
    #[test]
    fn test_mesh_pair_proc_default_mesh_timeout() {
        // Setup
        reset_for_test();
        set_pair_config_mesh_credentials([0x11; 16], [0x12; 16], [0x13; 16]);
        pair().default_mesh_effect_delay_ref = 0;
        pair().default_mesh_time_ref = 1000;
        pair().default_mesh_time = 5000;
        pair().mesh_pair_start_time = 0;

        reset_fakes();
        // 5 s (160_000_000 ticks) have elapsed since the default-mesh reference.
        fake_clock::set_ticks(1000 + 5000 * 1000 * 32 + 1);

        // Execute
        mesh_pair_proc();

        // Verify
        assert_eq!(pair().default_mesh_time_ref, 0);
        assert_eq!(
            pair_credentials_snapshot().pair_nn,
            [0x11; 16],
            "the timeout re-reads the persisted key"
        );
    }

    /// Tests mesh_pair_proc default mesh timeout with 255s delay
    #[test]
    fn test_mesh_pair_proc_default_mesh_timeout_255s() {
        // Setup
        reset_for_test();
        pair().default_mesh_effect_delay_ref = 0;
        pair().default_mesh_time_ref = 1000;
        pair().default_mesh_time = 255000;
        pair().mesh_pair_start_time = 0;

        reset_fakes();
        // `255 s` exceeds the documented `CLOCK_MAX_US`, so the tick span wraps
        // through 32 bits; the real predicate compares against the wrapped value.
        let now = 1000u32.wrapping_add((255_000u32 * 1000).wrapping_mul(32));
        fake_clock::set_ticks(now.wrapping_add(1) as u64);

        // Execute
        mesh_pair_proc();

        // Verify - should reset ref time, not clear it. The reference is
        // refreshed to the tick this step observed, because the clock is now an
        // argument of the state machine rather than a read inside it.
        assert_eq!(pair().default_mesh_time_ref, now.wrapping_add(1) | 1);
    }

    /// Tests mesh_pair_proc state machine - Name1
    #[test]
    fn test_mesh_pair_proc_state_name1() {
        // Setup
        reset_for_test();
        arm_pairing();
        pair().mesh_pair_state = MeshPairState::MeshPairName1;

        set_pair_setting_flag(PairAuthState::PairSetMeshTxStart);
        let mut credentials = pair_credentials_snapshot();
        credentials.pair_nn = [1; 16];
        set_pair_credentials(credentials);

        reset_fakes();
        fake_clock::set_ticks(1_000 + INTERVAL_TICKS as u64 + 1);

        // Execute
        mesh_pair_proc();

        // Verify - should advance to Name2 and broadcast this chunk.
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairName2);
        assert_eq!(MESH_RX_CHANNEL.len(), 1, "the broadcast is queued");
    }

    /// Tests mesh_pair_proc state machine - transitions through all states
    #[test]
    fn test_mesh_pair_proc_state_transitions() {
        // Setup
        reset_for_test();
        arm_pairing();

        set_pair_setting_flag(PairAuthState::PairSetMeshTxStart);
        let mut credentials = pair_credentials_snapshot();
        credentials.pair_nn = [1; 16];
        credentials.pair_pass = [2; 16];
        credentials.pair_ltk_mesh = [3; 16];
        set_pair_credentials(credentials);

        reset_fakes();

        // Every step needs its own due tick: the machine stamps `mesh_pair_time`
        // with the tick it was handed, so a frozen clock would park the next one.
        let mut now = 1_000 + INTERVAL_TICKS as u64 + 1;

        // Test Name1 -> Name2
        fake_clock::set_ticks(now);
        pair().mesh_pair_state = MeshPairState::MeshPairName1;
        mesh_pair_proc();
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairName2);

        // Test Name2 -> Pwd1
        now += INTERVAL_TICKS as u64 + 1;
        fake_clock::set_ticks(now);
        pair().mesh_pair_state = MeshPairState::MeshPairName2;
        mesh_pair_proc();
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairPwd1);

        // Test Pwd1 -> Pwd2
        now += INTERVAL_TICKS as u64 + 1;
        fake_clock::set_ticks(now);
        pair().mesh_pair_state = MeshPairState::MeshPairPwd1;
        mesh_pair_proc();
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairPwd2);

        // Test Pwd2 -> Ltk1
        now += INTERVAL_TICKS as u64 + 1;
        fake_clock::set_ticks(now);
        pair().mesh_pair_state = MeshPairState::MeshPairPwd2;
        mesh_pair_proc();
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairLtk1);

        // Test Ltk1 -> Ltk2
        now += INTERVAL_TICKS as u64 + 1;
        fake_clock::set_ticks(now);
        pair().mesh_pair_state = MeshPairState::MeshPairLtk1;
        mesh_pair_proc();
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairLtk2);

        // Test Ltk2 -> Name1 and state change to TxDone
        now += INTERVAL_TICKS as u64 + 1;
        fake_clock::set_ticks(now);
        pair().mesh_pair_state = MeshPairState::MeshPairLtk2;
        mesh_pair_proc();
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairName1);
        assert_eq!(pair_setting_flag(), PairAuthState::PairSetMeshTxDone);

        // Six intervals elapsed, so six broadcast chunks were queued.
        assert_eq!(MESH_RX_CHANNEL.len(), 6, "every step broadcasts one chunk");
    }

    /// The broadcast carries the bytes the state machine built, not the buffer
    /// the packet was queued in.
    #[test]
    fn test_mesh_pair_proc_broadcast_uses_the_machine_payload() {
        // Setup
        reset_for_test();
        arm_pairing();
        pair().mesh_pair_state = MeshPairState::MeshPairName1;
        set_pair_setting_flag(PairAuthState::PairSetMeshTxStart);
        let mut credentials = pair_credentials_snapshot();
        credentials.pair_nn = [
            0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e,
            0x4f, 0x50,
        ];
        set_pair_credentials(credentials);

        reset_fakes();
        fake_clock::set_ticks(1_000 + INTERVAL_TICKS as u64 + 1);

        // Execute
        mesh_pair_proc();

        // Verify: the broadcast went to 0xFFFF with no retransmit and no ack,
        // and it carries the pairing command's first chunk.
        let pkt = MESH_RX_CHANNEL
            .try_receive()
            .expect("the broadcast is queued for the receive side");
        let mesh = pkt.mesh();
        assert_eq!(mesh.dst_adr, 0xFFFF);
        assert_eq!(mesh.internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT], 0);
        assert_eq!(mesh.internal_par1[INTERNAL_PAR_SEND_ACK], 0);
        let val = &pkt.att_write().value.val;
        assert_eq!(val[0] & 0x3f, LGT_CMD_MESH_PAIR);
        assert_eq!(val[3], MeshPairState::MeshPairName1 as u8);
        assert_eq!(
            &val[4..12],
            &[0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48]
        );
    }

    /// Tests mesh_pair_proc PairSetMeshTxDone state
    #[test]
    fn test_mesh_pair_proc_tx_done_state() {
        // Setup
        reset_for_test();
        arm_pairing();

        set_pair_setting_flag(PairAuthState::PairSetMeshTxDone);

        reset_fakes();
        fake_clock::set_ticks(1_000 + INTERVAL_TICKS as u64 + 1);

        // Execute
        mesh_pair_proc();

        // Verify - should advance to RxDone state
        assert_eq!(pair_setting_flag(), PairAuthState::PairSetMeshRxDone);
    }

    /// Tests mesh_pair_proc PairSetMeshRxDone state
    #[test]
    fn test_mesh_pair_proc_rx_done_state() {
        // Setup
        reset_for_test();
        arm_pairing();
        pair().mesh_pair_timeout = 60; // Timeout in seconds - won't overflow when multiplied
        pair().mesh_pair_start_time = 5000;

        set_pair_setting_flag(PairAuthState::PairSetMeshRxDone);

        reset_fakes();
        // Inside the 60 s pairing window, and past the 500 ms broadcast interval.
        fake_clock::set_ticks(1_000_000_000);

        // Execute
        mesh_pair_proc();

        // Verify - should reset state and clear start time
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairName1);
        // mesh_pair_start_time should be cleared when transitioning from PairSetMeshRxDone
        assert_eq!(pair().mesh_pair_start_time, 0);
        assert_eq!(pair_setting_flag(), PairAuthState::PairSetted);
    }

    // --- mesh_security_enable Tests ---

    /// Tests mesh_security_enable with true
    #[test]
    fn test_mesh_security_enable_true() {
        // Setup
        reset_for_test();
        set_security_enabled(false);

        // Execute
        mesh_security_enable(true);

        // Verify
        assert!(is_security_enabled());
    }

    /// Tests mesh_security_enable with false
    #[test]
    fn test_mesh_security_enable_false() {
        // Setup
        reset_for_test();
        set_security_enabled(true);

        // Execute
        mesh_security_enable(false);

        // Verify
        assert!(!is_security_enabled());
    }

    // --- queues::enqueue_mesh_tx Tests ---

    /// A self-addressed packet is report-only: the TX task pops it, hands it to
    /// the UART bridge and never transmits or re-queues it.
    #[test]
    fn test_send_mesh_msg_iteration_self_addressed_is_report_only() {
        use futures::executor::block_on;

        // Setup
        reset_for_test();
        set_device_address(0x1234);
        set_security_enabled(false);
        let mut dropped_tx_seen = 0;
        let test_pkt = create_test_packet(0x1234, 0x1234, [1, 2, 3]);

        reset_fakes();

        // Execute
        assert!(queues::enqueue_mesh_tx(&test_pkt, 0, 0));
        block_on(mesh_packet_send_iteration(&mut dropped_tx_seen));

        // Verify - reported over the relay hook, popped, and the radio was never
        // touched: a report-only packet writes no RF register.
        assert_eq!(test_hooks::RELAYED_PACKETS.load(Ordering::Relaxed), 1);
        assert_eq!(test_hooks::RELAYED_SRC.load(Ordering::Relaxed), 0x1234);
        assert_eq!(test_hooks::RELAYED_DST.load(Ordering::Relaxed), 0x1234);
        assert_eq!(MESH_TX_QUEUE.lock().len(), 0);
        assert_eq!(
            writes_at(REG_RF_CHANNEL),
            0,
            "a self-addressed packet must not reach the transmitter"
        );
    }

    /// Tests queues::enqueue_mesh_tx normal add
    #[test]
    fn test_enqueue_mesh_tx_normal() {
        // Setup
        reset_for_test();
        set_device_address(0x1234);
        let test_pkt = create_test_packet(0x5678, 0xABCD, [1, 2, 3]);

        // Execute
        assert!(queues::enqueue_mesh_tx(&test_pkt, 1000, 3));

        // Verify
        let send_queue = MESH_TX_QUEUE.lock();
        assert_eq!(send_queue.len(), 1);
        assert_eq!(send_queue[0].delay, 1000);
        assert_eq!(send_queue[0].send_count, 3);
    }

    /// Tests queues::enqueue_mesh_tx buffer full
    #[test]
    fn test_enqueue_mesh_tx_buffer_full() {
        // Setup
        reset_for_test();
        set_device_address(0x1234);
        let test_pkt = create_test_packet(0x5678, 0xABCD, [1, 2, 3]);

        // Fill the buffer (capacity is 8)
        for i in 0..8 {
            let pkt = create_test_packet(i as u16, 0xFFFF, [i as u8, 0, 0]);
            assert!(queues::enqueue_mesh_tx(&pkt, i as u64, 0));
        }

        // Execute - try to add one more
        assert!(!queues::enqueue_mesh_tx(&test_pkt, 1000, 3));

        // Verify - buffer should still be at max capacity
        assert_eq!(MESH_TX_QUEUE.lock().len(), 8);
    }

    // --- queues::enqueue_mesh_rx Tests ---

    /// Tests queues::enqueue_mesh_rx normal add
    #[test]
    fn test_enqueue_mesh_rx_normal() {
        // Setup
        reset_for_test();
        let test_pkt = create_test_packet(0x1234, 0x5678, [1, 2, 3]);

        // Execute
        assert!(queues::enqueue_mesh_rx(&test_pkt));

        // Verify
        assert_eq!(MESH_RX_CHANNEL.len(), 1);
    }

    /// Tests queues::enqueue_mesh_rx buffer full
    #[test]
    fn test_enqueue_mesh_rx_buffer_full() {
        // Setup
        reset_for_test();

        // Fill the buffer (capacity is 8)
        for i in 0..8 {
            let pkt = create_test_packet(i as u16, 0xFFFF, [i as u8, 0, 0]);
            assert!(queues::enqueue_mesh_rx(&pkt));
        }

        // Execute - try to add one more
        let test_pkt = create_test_packet(0x9999, 0x8888, [9, 9, 9]);
        assert!(!queues::enqueue_mesh_rx(&test_pkt));

        // Verify - buffer should still be at max capacity
        assert_eq!(MESH_RX_CHANNEL.len(), 8);
    }

    /// Tests send_mesh_msg_iteration parks on the wake and wakes on a push
    #[test]
    fn test_send_mesh_msg_iteration_no_packets() {
        use core::future::Future;
        use core::task::{Context, Poll};
        use futures::task::noop_waker;

        // Setup
        reset_for_test();
        let mut dropped_tx_seen = 0;

        // The TX queue is empty, so the scan takes the `SendScan::Empty` arm
        // (the `min()` `None` arm) and the iteration parks on the wake.
        assert!(MESH_TX_QUEUE.lock().is_empty());
        let future = mesh_packet_send_iteration(&mut dropped_tx_seen);
        futures::pin_mut!(future);

        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);

        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Pending));

        // Enqueue a ready packet; `enqueue_mesh_tx` signals the wake
        let test_pkt = create_test_packet(0x1234, 0x5678, [1, 2, 3]);
        assert!(queues::enqueue_mesh_tx(&test_pkt, 0, 0));
        assert_eq!(MESH_TX_QUEUE.lock().len(), 1);

        // The wake signal releases the parked iteration without sending; the
        // outer task loop re-enters and picks the packet up.
        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Ready(())));

        // Verify - the packet is still queued for the next iteration
        assert_eq!(MESH_TX_QUEUE.lock().len(), 1);
    }

    /// Tests send_mesh_msg_iteration with ready packet
    #[test]
    fn test_send_mesh_msg_iteration_with_ready_packet() {
        use futures::executor::block_on;

        // Setup
        reset_for_test();
        let mut dropped_tx_seen = 0;
        let test_pkt = create_test_packet(0x1234, 0x5678, [1, 2, 3]);

        // Ensure security is disabled to avoid encryption overflow
        set_security_enabled(false);

        // Add a packet that's ready to send (delay is in the past)
        assert!(queues::enqueue_mesh_tx(&test_pkt, 0, 0)); // delay=0 means ready to send immediately

        assert_eq!(MESH_TX_QUEUE.lock().len(), 1);

        // `rand()` is the real `clock_time() as u16 ^ reg_rnd_number`, so seed
        // both halves: 1000 ^ 0x5555 keeps the channel selection and congestion
        // delay identical.
        reset_fakes();
        fake_clock::set_ticks(1000);
        FakeMmio::set_reg16(REG_RND_NUMBER, 0x5555);

        // Execute
        block_on(mesh_packet_send_iteration(&mut dropped_tx_seen));

        // Verify - the packet was transmitted on every listen channel and
        // removed from the buffer.
        assert_eq!(MESH_TX_QUEUE.lock().len(), 0);
        assert_eq!(
            writes_at(REG_RF_CHANNEL),
            SYS_CHN_LISTEN.len(),
            "one channel hop per listen channel"
        );
    }

    /// The congestion backoff re-queue must keep the pre-migration 0..8 ms
    /// spread: `8000 - (rand() % 16) * 500` microseconds past `clock_time64()`.
    ///
    /// `rand()` used to be spelled `system_tick as u16 ^ rnd_number`; this pins
    /// both the value and the single draw it takes per retransmission.
    #[test]
    fn test_send_mesh_msg_iteration_requeues_with_congestion_delay() {
        use futures::executor::block_on;

        // Setup
        reset_for_test();
        let mut dropped_tx_seen = 0;
        let test_pkt = create_test_packet(0x1234, 0x5678, [1, 2, 3]);

        set_security_enabled(false);

        // One retransmission left, due immediately.
        assert!(queues::enqueue_mesh_tx(&test_pkt, 0, 1));

        // 0x03E8 ^ 0x5555 = 0x56BD; 0x56BD % 16 == 13 -> 8000 - 13 * 500 = 1500 us.
        // `rand()` is the real `clock_time() as u16 ^ reg_rnd_number`.
        reset_fakes();
        fake_clock::set_ticks(1000);
        FakeMmio::set_reg16(REG_RND_NUMBER, 0x5555);

        // Execute
        block_on(mesh_packet_send_iteration(&mut dropped_tx_seen));

        // Verify: transmitted once, then re-queued one step before the last send.
        assert_eq!(
            writes_at(REG_RF_CHANNEL),
            SYS_CHN_LISTEN.len(),
            "one channel hop per listen channel"
        );
        let send_queue = MESH_TX_QUEUE.lock();
        assert_eq!(send_queue.len(), 1, "the retransmission is re-queued");
        assert_eq!(send_queue[0].send_count, 0);
        assert_eq!(
            send_queue[0].delay,
            1000 + 1500 * CLOCK_SYS_CLOCK_1US as u64,
            "the congestion delay must stay 8000 - (rand % 16) * 500 us past the clock"
        );
        drop(send_queue);
    }

    /// Tests send_mesh_msg_iteration parks on the earliest queued deadline and
    /// wakes when a new packet is pushed.
    ///
    /// This is the `SendScan::Wait(earliest)` arm: the buffer is non-empty but
    /// nothing is due yet, so the only exit is the wake signal. Every mesh
    /// retransmission takes this path.
    #[test]
    fn test_send_mesh_msg_iteration_parks_until_earliest_deadline() {
        use core::future::Future;
        use core::task::{Context, Poll};
        use futures::task::noop_waker;

        // Setup
        reset_for_test();
        let mut dropped_tx_seen = 0;

        // The scan samples the clock once (after the empty check).
        reset_fakes();

        // Deadline far in the future: the buffer is not empty, but no packet is
        // due, so the scan must take `SendScan::Wait(earliest)`, not `Empty`.
        let far_pkt = create_test_packet(0x1234, 0x5678, [1, 2, 3]);
        assert!(queues::enqueue_mesh_tx(&far_pkt, u64::MAX / 2, 0));
        assert_eq!(MESH_TX_QUEUE.lock().len(), 1);

        let future = mesh_packet_send_iteration(&mut dropped_tx_seen);
        futures::pin_mut!(future);

        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);

        // First poll: parked on the deadline wait.
        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Pending));

        // The not-yet-due packet must NOT have been consumed by the scan.
        assert_eq!(MESH_TX_QUEUE.lock().len(), 1);

        // A due packet signals the wake, which releases the parked wait.
        let ready_pkt = create_test_packet(0x9abc, 0xdef0, [4, 5, 6]);
        assert!(queues::enqueue_mesh_tx(&ready_pkt, 0, 0));
        assert_eq!(MESH_TX_QUEUE.lock().len(), 2);

        // Second poll: the wake completes the iteration without sending; the
        // outer task loop re-enters and rescans.
        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Ready(())));

        // Nothing was consumed by this iteration.
        assert_eq!(MESH_TX_QUEUE.lock().len(), 2);
    }

    // --- rcv_mesh_msg_iteration Tests ---

    /// A packet the decoder accepts: a real 13-byte mesh command with the
    /// documented `l2cap_len` and a distinct sequence number.
    fn decodable_packet(sno: u32) -> Packet {
        let mut payload = [0u8; 13];
        payload[0] = LGT_CMD_LIGHT_ONOFF | 0xc0;
        payload[3] = LIGHT_ON_PARAM;
        mesh_construct_packet(sno, 0xFFFF, &payload, 0, false)
    }

    /// The decoder caches opcode+sequence pairs, so a suite that reuses a
    /// sequence number silently drops the second packet as a duplicate.
    fn forget_decoded_packets() {
        tlsr8266_radio::mesh_ll::node_status::RC_PKT_BUF
            .lock()
            .clear();
    }

    /// Tests rcv_mesh_msg_iteration parks on receive and wakes on a push
    #[test]
    fn test_rcv_mesh_msg_iteration_no_packets() {
        use core::future::Future;
        use core::task::{Context, Poll};
        use futures::task::noop_waker;

        // Setup
        reset_for_test();
        forget_decoded_packets();
        relay::install();

        // The RX channel is empty by default, so the iteration parks on receive()
        let future = mesh_packet_rcv_iteration();
        futures::pin_mut!(future);

        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);

        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Pending));

        // Enqueue a packet; Channel::try_send wakes the parked receiver
        let test_pkt = decodable_packet(0x11_2233);
        assert!(queues::enqueue_mesh_rx(&test_pkt));
        assert_eq!(MESH_RX_CHANNEL.len(), 1);

        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Ready(())));

        // Verify - the packet was received and handed to the decoder, which
        // relays every accepted command to the UART bridge.
        assert_eq!(MESH_RX_CHANNEL.len(), 0);
        assert_eq!(relay::count(), 1);
    }

    /// Tests rcv_mesh_msg_iteration with packet to receive
    #[test]
    fn test_rcv_mesh_msg_iteration_with_packet() {
        use futures::executor::block_on;

        // Setup
        reset_for_test();
        forget_decoded_packets();
        relay::install();
        let test_pkt = decodable_packet(0x44_5566);

        // Add a packet to receive
        assert!(queues::enqueue_mesh_rx(&test_pkt));
        assert_eq!(MESH_RX_CHANNEL.len(), 1);

        // Execute
        block_on(mesh_packet_rcv_iteration());

        // Verify - packet was processed, removed from the buffer, and relayed.
        assert_eq!(MESH_RX_CHANNEL.len(), 0);
        assert_eq!(relay::count(), 1);
    }

    #[test]
    fn test_gatt_spp_shape_classifier_and_task_route() {
        use futures::executor::block_on;
        use mesh_protocol::packet_types::PacketAttWrite;

        reset_for_test();
        relay::install();

        let mut gatt_write = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: 2,
                    rf_len: 0x11,
                    l2cap_len: 0x11,
                    chan_id: 4,
                },
                opcode: 0x12,
                handle: 21,
                handle1: 0,
                value: PacketAttValue::default(),
            },
        };

        assert!(is_gatt_spp_rx_packet(&gatt_write));
        gatt_write.att_write_mut().opcode = 0x52;
        assert!(is_gatt_spp_rx_packet(&gatt_write));
        gatt_write.att_write_mut().handle = 22;
        assert!(!is_gatt_spp_rx_packet(&gatt_write));
        gatt_write.att_write_mut().handle = 21;
        gatt_write.att_write_mut().head.chan_id = 0xff03;
        assert!(!is_gatt_spp_rx_packet(&gatt_write));

        // Restore a valid handle-21 Write Request, queue it, and run the receive task.
        gatt_write.att_write_mut().head.chan_id = 4;
        gatt_write.att_write_mut().opcode = 0x12;
        assert!(queues::enqueue_mesh_rx(&gatt_write));
        block_on(mesh_packet_rcv_iteration());

        assert_eq!(MESH_RX_CHANNEL.len(), 0);
        assert_eq!(relay::count(), 0, "GATT SPP must not enter rf_link_rc_data");

        let ordinary = decodable_packet(0x77_8899);
        assert!(!is_gatt_spp_rx_packet(&ordinary));
    }

    /// Whether any frame queued on the UART log transport contains `needle`.
    ///
    /// Test builds route `uprintln!` through the default log sinks into
    /// [`queues::UART_TX_CHANNEL`], so a test reads the line where the host
    /// would. The channel is drained: the caller owns the log for its test.
    fn uart_log_contains(needle: &str) -> bool {
        let needle = needle.as_bytes();
        let mut found = false;
        while let Ok(frame) = queues::UART_TX_CHANNEL.try_receive() {
            let payload = &frame.data[mesh_protocol::uart::UART_PAYLOAD_OFFSET..];
            if payload.windows(needle.len()).any(|window| window == needle) {
                found = true;
            }
        }
        found
    }

    /// An idle node that overflows [`MESH_RX_CHANNEL`] must report the drop
    /// from the receive task alone.
    ///
    /// `report_queue_drops` used to run only inside the send iteration, so a
    /// node with no TX traffic never logged RX overflow. The send task is never
    /// polled here: the line must come from `mesh_packet_rcv_iteration`.
    #[test]
    fn test_rcv_mesh_msg_iteration_reports_rx_drops_without_tx() {
        use futures::executor::block_on;

        // Setup
        reset_for_test();
        forget_decoded_packets();
        relay::install();

        // Fill the channel, then overflow it once so the SDK counts one RX drop.
        for sno in 0..queues::MESH_RX_QUEUE_CAPACITY as u32 {
            assert!(queues::enqueue_mesh_rx(&decodable_packet(0x11_0000 + sno)));
        }
        assert!(!queues::enqueue_mesh_rx(&decodable_packet(0x11_ffff)));
        assert_eq!(queues::dropped_mesh_rx(), 1);

        // Execute: only the receive task runs.
        block_on(mesh_packet_rcv_iteration());

        // Verify: the queued packet was still decoded, and the overflow was
        // logged over the UART sink with no TX activity.
        assert_eq!(relay::count(), 1);
        assert!(
            uart_log_contains("mesh queue full: rx +1 (total 1)"),
            "the receive task must report RX queue overflow without TX activity"
        );
    }
}
