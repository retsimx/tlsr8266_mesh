//! Mesh node status records, pairing protocol constants, light control commands
//! and OTA wire constants.
//!
//! Both ends of the wire agree on these byte layouts and discriminants: the
//! status advertisement payload, the pairing opcodes the central writes, the
//! pairing state machine the firmware reports back, the light control commands
//! and their parameters, and the OTA opcodes and completion states exchanged
//! during a firmware transfer.

use bytemuck::{Pod, Zeroable};

pub const MESH_NODE_ST_VAL_LEN: usize = 4;
// MIN: 4,   MAX: 10
pub const MESH_NODE_ST_PAR_LEN: usize = MESH_NODE_ST_VAL_LEN - 2;

/// Size of the status value region in one status advertisement packet.
///
/// `mesh_node_adv_status` fills `MESH_STATUS_VALUE_LEN` bytes and
/// `rf_link_rc_data` decodes exactly the same span, so this is the single
/// source of truth for the per-packet record capacity.
pub const MESH_STATUS_VALUE_LEN: usize = 24;

#[derive(Clone, Copy, Debug)]
#[repr(C, packed)]
pub struct MeshNodeStValT {
    pub dev_adr: u8,
    // don't change include type
    pub sn: u8,
    // don't change include type
    pub par: [u8; MESH_NODE_ST_PAR_LEN], //lumen-rsv,
}

// SAFETY: This struct is safe to use with bytemuck because:
// 1. It's #[repr(C, packed)] so it has a stable memory layout
// 2. All fields are u8 which are safe to transmute
// 3. The struct size is fixed and known at compile time
unsafe impl Pod for MeshNodeStValT {}
// SAFETY: (4) audited packed-wire view: all-zero bytes are a valid `MeshNodeStValT`
// (`dev_adr` and `sn` are `u8`, `par` is `[u8; MESH_NODE_ST_PAR_LEN]`), which is all
// `Zeroable` requires.
unsafe impl Zeroable for MeshNodeStValT {}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct MeshNodeStT {
    /// Sweep-age counter: `0` = offline/never seen, `>= 1` = online with age
    /// `miss - 1` status sweeps since the last observation. All-zero init in
    /// `.bss` therefore means offline without an explicit runtime fill.
    pub miss: u8,
    // don't change include type
    pub val: MeshNodeStValT,
}

/// A node is online iff it has a non-zero sweep-age counter (`miss`).
pub fn is_online(node: &MeshNodeStT) -> bool {
    node.miss != 0
}

/// Represents the current state of the BLE pairing process
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PairState {
    Idle = 0x00,                  // Initial/idle state
    AwaitingRandom = 0x02,        // Waiting for random challenge
    ReceivingMeshName = 0x05,     // Receiving mesh name
    ReceivingMeshPassword = 0x06, // Receiving mesh password
    ReceivingMeshLtk = 0x07,      // Receiving long-term key
    RequestingLtk = 0x09,         // Requesting LTK
    ResetMesh = 0x0A,             // Reset mesh configuration
    DeletePairing = 0x0B,         // Deleting pairing
    RandomConfirmation = 0x0C,    // Random value confirmation
    SessionKeyExchange = 0x0D,    // Session key exchange
    Init = 0x0E,                  // Initialization state
    Completed = 0x0F,             // Pairing process completed
}

impl From<u8> for PairState {
    fn from(value: u8) -> Self {
        match value {
            0x00 => Self::Idle,
            0x02 => Self::AwaitingRandom,
            0x05 => Self::ReceivingMeshName,
            0x06 => Self::ReceivingMeshPassword,
            0x07 => Self::ReceivingMeshLtk,
            0x09 => Self::RequestingLtk,
            0x0A => Self::ResetMesh,
            0x0B => Self::DeletePairing,
            0x0C => Self::RandomConfirmation,
            0x0D => Self::SessionKeyExchange,
            0x0E => Self::Init,
            0x0F => Self::Completed,
            _ => Self::Idle, // Default to Idle for unrecognized states
        }
    }
}

// Pairing Opcodes - Used in pair_write to identify the command type
pub const PAIR_OP_EXCHANGE_RANDOM: u8 = 0x01; // Exchange random challenge
pub const PAIR_OP_SET_MESH_NAME: u8 = 0x04; // Set mesh network name
pub const PAIR_OP_SET_MESH_PASSWORD: u8 = 0x05; // Set mesh network password
pub const PAIR_OP_SET_MESH_LTK: u8 = 0x06; // Set long-term key
pub const PAIR_OP_GET_MESH_LTK: u8 = 0x08; // Request long-term key
pub const PAIR_OP_RESET_MESH: u8 = 0x0A; // Reset mesh configuration
pub const PAIR_OP_VERIFY_CREDENTIALS: u8 = 0x0C; // Verify credentials
pub const PAIR_OP_DELETE_PAIRING: u8 = 0x0E; // Delete pairing information

// --- Light control commands and mesh status timing ------------------------

pub const ONLINE_STATUS_TIMEOUT: u32 = 3000;

/// Effective measured status advertisement period in ms.
///
/// The configured interval is `SEND_MESH_STATUS_INTERVAL_MS` (200 ms) but the
/// measured mean is ~226 ms and the worst case ~277 ms, so the deadline derives
/// from this conservative round-up rather than the optimistic nominal value.
pub const MESH_STATUS_TX_CADENCE_MS: u32 = 250;

/// Loss/jitter safety margin applied to the coverage period.
///
/// This is deliberately *not* a hop-count multiplier: a relayed record is
/// refreshed once per coverage sweep, so the margin absorbs packet loss and
/// jitter (one successful window plus three missed windows).
pub const MESH_STATUS_TIMEOUT_SAFETY_FACTOR: u32 = 4;

/// Remote node records carried per status advertisement packet.
///
/// A status value region holds `MESH_STATUS_VALUE_LEN / MESH_NODE_ST_VAL_LEN`
/// records and the first is always this device, leaving 5 remote records. The
/// `const _` assertion in `mesh_management` fails the build if this drifts from
/// the wire capacity used by `mesh_node_adv_status`/`rf_link_rc_data`.
pub const MESH_STATUS_RECORDS_PER_PACKET: u32 = 5;

/// Cadence-derived offline deadline in ms for a mesh of `node_count` known
/// nodes (self included), clamped to [`ONLINE_STATUS_TIMEOUT`].
///
/// Every remote record is exposed once per round-robin coverage sweep:
/// `ceil((node_count - 1) / MESH_STATUS_RECORDS_PER_PACKET)` packets at
/// `MESH_STATUS_TX_CADENCE_MS`. A fixed 3 s deadline is crossed by this sweep
/// for relayed nodes at larger `node_count`, so the deadline scales with it.
pub fn mesh_status_timeout_ms(node_count: u32) -> u32 {
    // L1: callers pass MESH_NODE_MAX, bounded by MESH_NODE_MAX_NUM (64). At that
    // bound the product stays far inside u32 (remotes=63, sweeps=13,
    // coverage=3250 ms, scaled deadline=13000), so no wrapping can occur.
    debug_assert!(
        node_count <= MESH_NODE_MAX_NUM as u32,
        "mesh_status_timeout_ms assumes at most MESH_NODE_MAX_NUM nodes"
    );
    let remotes = node_count.saturating_sub(1);
    let sweeps = remotes.div_ceil(MESH_STATUS_RECORDS_PER_PACKET);
    let coverage_ms = sweeps * MESH_STATUS_TX_CADENCE_MS;
    (coverage_ms * MESH_STATUS_TIMEOUT_SAFETY_FACTOR).max(ONLINE_STATUS_TIMEOUT)
}

/// Status sweep cadence in ms: `mesh_node_flush_status` runs at most this often.
///
/// Each run advances every online remote's `miss` counter by one, so a node's
/// age in sweeps is `miss - 1` and the offline deadline in sweeps is the
/// deadline in ms divided by this cadence.
pub const MESH_STATUS_SWEEP_MS: u32 = 500;

/// Offline deadline expressed in [`MESH_STATUS_SWEEP_MS`] sweeps for a mesh of
/// `node_count` nodes, clamped to `1..=254`.
///
/// Derived from [`mesh_status_timeout_ms`] so the two cannot disagree. The cap
/// keeps an online `miss` (which starts at 1 and increments once per sweep)
/// from wrapping before it is reset to 0 on timeout.
pub fn mesh_status_offline_sweeps(node_count: u32) -> u8 {
    let sweeps = mesh_status_timeout_ms(node_count).div_ceil(MESH_STATUS_SWEEP_MS);
    sweeps.clamp(1, 254) as u8
}

pub const MAX_GROUP_COUNT: u8 = 8;

//app use
pub const PAR_READ_MESH_PAIR_CONFIRM: u8 = 0x0a;

// op cmd 11xxxxxxzzzzzzzzzzzzzzzz z's=VENDOR_ID  xxxxxx=LGT_CMD_
pub const LGT_CMD_NOTIFY_MESH: u8 = 0x02;
//internal use
pub const LGT_CMD_MESH_PAIR: u8 = 0x09;
pub const LGT_CMD_MESH_CMD_NOTIFY: u8 = 0x0a;
pub const CMD_NOTIFY_MESH_PAIR_END: u8 = 0x00;

pub const LGT_CMD_LIGHT_ONOFF: u8 = 0x10;
//internal use
pub const LGT_CMD_LIGHT_GRP_RSP1: u8 = 0x14;
//get group rsp: 8groups low 1bytes
pub const LGT_CMD_LIGHT_GRP_RSP2: u8 = 0x15;
//get group rsp: front 4groups 2bytes
pub const LGT_CMD_LIGHT_GRP_RSP3: u8 = 0x16;
//get group rsp: behind 4groups 2bytes
pub const LGT_CMD_LIGHT_CONFIG_GRP: u8 = 0x17;
//add or del group
//internal use
pub const LGT_CMD_LIGHT_READ_STATUS: u8 = 0x1a;
//get status req
pub const LGT_CMD_LIGHT_STATUS: u8 = 0x1b;
//get status rsp
pub const LGT_CMD_LIGHT_GRP_REQ: u8 = 0x1d;
//get group req
pub const LGT_CMD_CONFIG_DEV_ADDR: u8 = 0x20;
//add device address
pub const DEV_ADDR_PAR_WITH_MAC: u8 = 0x01;
pub const LGT_CMD_DEV_ADDR_RSP: u8 = 0x21;
//rsp
pub const LGT_CMD_KICK_OUT: u8 = 0x23;

pub const LGT_CMD_USER_NOTIFY_REQ: u8 = 0x2a;
pub const LGT_CMD_USER_NOTIFY_RSP: u8 = 0x2b;

pub const LGT_CMD_SET_LIGHT: u8 = 0x30;
pub const LGT_CMD_SET_MAC_ADDR: u8 = 0x31;
pub const LGT_POWER_ON: u8 = 0x32;
pub const LGT_PANIC_MSG: u8 = 0x34;

// Sent to acknowledge a non-notify message
pub const LGT_CMD_LIGHT_ACK: u8 = 0x35;

pub const GET_STATUS: u8 = 0;
pub const GET_GROUP1: u8 = 1;
// return 8 GROUP_ADDRESS(low 1byte)
pub const GET_GROUP2: u8 = 2;
// return front 4 GROUP_ADDRESS
pub const GET_GROUP3: u8 = 3;
// return behind 4 GROUP_ADDRESS
pub const GET_DEV_ADDR: u8 = 4;
// return device address
pub const GET_USER_NOTIFY: u8 = 7;
// return user notify info
pub const CMD_START_OTA: u8 = 8;
pub const CMD_OTA_DATA: u8 = 9;
pub const CMD_END_OTA: u8 = 10;

pub const LGT_CMD_SET_MESH_INFO: u8 = 0xc5;
pub const LGT_CMD_SET_DEV_ADDR: u8 = 0xc6;
pub const LGT_CMD_DEL_PAIR: u8 = 0xc7;
pub const LGT_CMD_MESH_PAIR_TIMEOUT: u8 = 0xcf;

pub const LIGHT_OFF_PARAM: u8 = 0x00;
pub const LIGHT_ON_PARAM: u8 = 0x01;

pub const LIGHT_ADD_GRP_PARAM: u8 = 0x01;
pub const LIGHT_DEL_GRP_PARAM: u8 = 0x00;

pub const DEVICE_ADDR_MASK_DEFAULT: u16 = 0x7FFF;

pub const MESH_NODE_MAX_NUM: usize = 64;
pub const MESH_NODE_MASK_LEN: usize = (MESH_NODE_MAX_NUM + 31) >> 5;

#[inline(always)]
pub fn is_unicast_addr(p_addr: &[u8]) -> bool {
    p_addr[1] & 0x80 == 0
}

// --- OTA transfer ---------------------------------------------------------

/// OTA transfer completion state, shared by both ends of the wire.
#[repr(C)]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum OtaState {
    Continue = 0,
    // must zero
    Ok = 1,
    Error = 2,
    MasterOtaRebootOnly = 3,
}

/// Boot flag word written into the new-firmware header.
pub const START_UP_FLAG: u32 = 0x544c4e4b;

/// Default mesh-OTA supervision timeout, in seconds.
pub const RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS: u16 = 30;

// OTA opcodes; `LGT_CMD_*` shares the `11xxxxxxzzzzzzzzzzzzzzzz` command space
// with the light commands. The `_REQ`/`_RSP` pairs are contiguous by design.
pub const LGT_CMD_START_OTA_REQ: u8 = 0x24;
pub const LGT_CMD_START_OTA_RSP: u8 = 0x25;
pub const LGT_CMD_OTA_DATA_REQ: u8 = 0x26;
pub const LGT_CMD_OTA_DATA_RSP: u8 = 0x27;
pub const LGT_CMD_END_OTA_REQ: u8 = 0x28;
pub const LGT_CMD_END_OTA_RSP: u8 = 0x29;

// Product-neutral aliases for the three OTA response opcodes. The OTA engine is
// owned by `tlsr8266-platform`, which may not name `lgt_*` identifiers
// (`sdk/check_layering.py` bans them there). The aliases carry the *same*
// values as the `LGT_CMD_*` constants, so there is still exactly one wire
// contract.
pub const OTA_CMD_START_RSP: u8 = LGT_CMD_START_OTA_RSP;
pub const OTA_CMD_DATA_RSP: u8 = LGT_CMD_OTA_DATA_RSP;
pub const OTA_CMD_END_RSP: u8 = LGT_CMD_END_OTA_RSP;

#[cfg(test)]
mod tests {
    use super::*;

    /// The completion states cross the wire as these exact discriminants.
    #[test]
    fn ota_state_discriminants_are_pinned() {
        assert_eq!(OtaState::Continue as u8, 0);
        assert_eq!(OtaState::Ok as u8, 1);
        assert_eq!(OtaState::Error as u8, 2);
        assert_eq!(OtaState::MasterOtaRebootOnly as u8, 3);
    }

    /// The boot flag and supervision timeout are shared with the host tooling
    /// and the new-firmware header, so their values are part of the contract.
    #[test]
    fn ota_wire_constants_are_pinned() {
        assert_eq!(START_UP_FLAG, 0x544c4e4b);
        assert_eq!(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS, 30);
    }

    /// Six contiguous opcodes: start, data and end each with a request and a
    /// response.
    #[test]
    fn ota_opcodes_are_pinned() {
        assert_eq!(LGT_CMD_START_OTA_REQ, 0x24);
        assert_eq!(LGT_CMD_START_OTA_RSP, 0x25);
        assert_eq!(LGT_CMD_OTA_DATA_REQ, 0x26);
        assert_eq!(LGT_CMD_OTA_DATA_RSP, 0x27);
        assert_eq!(LGT_CMD_END_OTA_REQ, 0x28);
        assert_eq!(LGT_CMD_END_OTA_RSP, 0x29);
    }

    /// The product-neutral aliases exist so `tlsr8266-platform` can name the
    /// response opcodes without `lgt_*`; they must pin the same wire values.
    #[test]
    fn ota_response_aliases_match_legacy_opcodes() {
        assert_eq!(OTA_CMD_START_RSP, LGT_CMD_START_OTA_RSP);
        assert_eq!(OTA_CMD_DATA_RSP, LGT_CMD_OTA_DATA_RSP);
        assert_eq!(OTA_CMD_END_RSP, LGT_CMD_END_OTA_RSP);
        assert_eq!(OTA_CMD_START_RSP, 0x25);
        assert_eq!(OTA_CMD_DATA_RSP, 0x27);
        assert_eq!(OTA_CMD_END_RSP, 0x29);
    }

    /// Tests the is_unicast_addr function.
    ///
    /// This test verifies that is_unicast_addr correctly identifies
    /// unicast vs group BLE addresses based on the MSB of the second byte.
    ///
    /// # Algorithm
    ///
    /// 1. Test unicast addresses (MSB of second byte = 0)
    /// 2. Test group addresses (MSB of second byte = 1)
    /// 3. Test edge cases
    ///
    /// # Notes
    ///
    /// * BLE unicast addresses have the MSB of the second byte clear
    /// * Group addresses have the MSB of the second byte set
    #[test]
    fn test_is_unicast_addr() {
        // Test unicast addresses (second byte MSB = 0)
        assert!(is_unicast_addr(&[0x01, 0x02, 0x03, 0x04, 0x05, 0x06]));
        assert!(is_unicast_addr(&[0xFF, 0x7F, 0x03, 0x04, 0x05, 0x06])); // 0x7F = 01111111
        assert!(is_unicast_addr(&[0x00, 0x00, 0x03, 0x04, 0x05, 0x06]));

        // Test group addresses (second byte MSB = 1)
        assert!(!is_unicast_addr(&[0x01, 0x82, 0x03, 0x04, 0x05, 0x06])); // 0x82 = 10000010
        assert!(!is_unicast_addr(&[0xFF, 0xFF, 0x03, 0x04, 0x05, 0x06])); // 0xFF = 11111111
        assert!(!is_unicast_addr(&[0x00, 0x80, 0x03, 0x04, 0x05, 0x06])); // 0x80 = 10000000

        // Test edge cases
        assert!(is_unicast_addr(&[0x00, 0x7F, 0x00, 0x00, 0x00, 0x00])); // Maximum unicast
        assert!(!is_unicast_addr(&[0x00, 0x80, 0x00, 0x00, 0x00, 0x00])); // Minimum group
    }

    /// Tests the cadence-derived offline deadline, including the documented
    /// examples M=40 -> 8 s and M=64 -> 13 s and the small-mesh floor.
    #[test]
    fn test_mesh_status_timeout_ms_floor_and_scaling() {
        // Small meshes stay on the historical floor.
        assert_eq!(mesh_status_timeout_ms(0), ONLINE_STATUS_TIMEOUT);
        assert_eq!(mesh_status_timeout_ms(1), ONLINE_STATUS_TIMEOUT);
        assert_eq!(mesh_status_timeout_ms(5), ONLINE_STATUS_TIMEOUT);
        // 5 remotes = one full sweep, whose x4 margin is still under the floor.
        assert_eq!(mesh_status_timeout_ms(6), ONLINE_STATUS_TIMEOUT);
        // M=40: ceil(39/5)=8 sweeps -> 8*250*4 = 8000.
        assert_eq!(mesh_status_timeout_ms(40), 8000);
        // M=41: ceil(40/5)=8 sweeps -> 8000.
        assert_eq!(mesh_status_timeout_ms(41), 8000);
        // M=42: ceil(41/5)=9 sweeps -> 9000.
        assert_eq!(mesh_status_timeout_ms(42), 9000);
        // M=64: ceil(63/5)=13 sweeps -> 13000.
        assert_eq!(mesh_status_timeout_ms(64), 13000);
    }

    /// L1: `MESH_NODE_MAX_NUM` is the documented bound the deadline math relies
    /// on; at that bound the sweep product is far inside u32 and yields 13 s.
    #[test]
    fn test_mesh_status_timeout_ms_max_mesh_bound() {
        assert_eq!(MESH_NODE_MAX_NUM, 64, "deadline math assumes <= 64 nodes");
        assert_eq!(mesh_status_timeout_ms(MESH_NODE_MAX_NUM as u32), 13000);
    }

    /// The sweep deadline is the ms deadline rounded up to whole sweeps, so the
    /// two derivations cannot drift apart.
    #[test]
    fn test_mesh_status_offline_sweeps_scaling() {
        assert_eq!(MESH_STATUS_SWEEP_MS, 500);
        // Floor deadline 3000 ms -> 6 sweeps.
        assert_eq!(mesh_status_offline_sweeps(0), 6);
        assert_eq!(mesh_status_offline_sweeps(1), 6);
        assert_eq!(mesh_status_offline_sweeps(6), 6);
        // M=40/41: 8000 ms -> 16 sweeps.
        assert_eq!(mesh_status_offline_sweeps(40), 16);
        assert_eq!(mesh_status_offline_sweeps(41), 16);
        // M=42: 9000 ms -> 18 sweeps.
        assert_eq!(mesh_status_offline_sweeps(42), 18);
        // M=64: 13000 ms -> 26 sweeps.
        assert_eq!(mesh_status_offline_sweeps(64), 26);
        // The cap exists so an online miss (1..=deadline) never wraps to 0.
        assert!(mesh_status_offline_sweeps(MESH_NODE_MAX_NUM as u32) <= 254);
    }
}
