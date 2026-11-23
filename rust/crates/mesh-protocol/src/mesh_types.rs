//! Mesh node status records, pairing protocol constants and OTA wire constants.
//!
//! Both ends of the wire agree on these byte layouts and discriminants: the
//! status advertisement payload, the pairing opcodes the central writes, the
//! pairing state machine the firmware reports back, and the OTA opcodes and
//! completion states exchanged during a firmware transfer.

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

/// OTA state as observed by a layer that may not own it.
///
/// Not part of the wire contract: it is the read-only projection the BLE link
/// layer needs, so a seam can carry three flags in one `fn() -> ..` call and
/// `mergefunc` has nothing to fold.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct OtaStatusSnapshot {
    pub in_progress: bool,
    pub finish_status: OtaState,
    pub timeout_expired: bool,
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

    /// The snapshot is an ordinary `Copy` value type, not a wire struct: it is
    /// constructed field-by-field and compared by value.
    #[test]
    fn ota_status_snapshot_projects_state() {
        let idle = OtaStatusSnapshot {
            in_progress: false,
            finish_status: OtaState::Continue,
            timeout_expired: false,
        };
        assert_eq!(
            idle,
            OtaStatusSnapshot {
                in_progress: false,
                finish_status: OtaState::Continue,
                timeout_expired: false,
            }
        );
        assert_ne!(
            idle,
            OtaStatusSnapshot {
                in_progress: true,
                finish_status: OtaState::Ok,
                timeout_expired: true,
            }
        );
    }
}
