//! Mesh node status records and pairing protocol constants.
//!
//! Both ends of the wire agree on these byte layouts and discriminants: the
//! status advertisement payload, the pairing opcodes the central writes, and
//! the pairing state machine the firmware reports back.

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
