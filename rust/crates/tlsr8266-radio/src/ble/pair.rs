use core::cmp::min;
use core::ptr::{addr_of, addr_of_mut, slice_from_raw_parts_mut};
use core::slice;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use crate::ble::flash_config::{pair_flash_clean, pair_load_key, pair_update_key, save_pair_info};
use crate::ble::pairing::rf_link_delete_pair;
use crate::config::RadioConfig;
use crate::rf::drv_8266::{
    FLASH_CONFIGURATION_INDEX, MAC_ID, MAX_MESH_NAME_LEN, MESH_DEVICE_ADDRESS_VALIDATION_PENDING,
};
use crate::types::*;
use mesh_protocol::mesh_types::{
    PairState, PAIR_OP_DELETE_PAIRING, PAIR_OP_EXCHANGE_RANDOM, PAIR_OP_GET_MESH_LTK,
    PAIR_OP_RESET_MESH, PAIR_OP_SET_MESH_LTK, PAIR_OP_SET_MESH_NAME, PAIR_OP_SET_MESH_PASSWORD,
    PAIR_OP_VERIFY_CREDENTIALS,
};
use mesh_protocol::packet_types::{
    AppCmdValue, Packet, PacketAttReadRsp, PacketAttValue, PacketAttWrite, PacketL2capHead,
    PacketLlApp,
};
use tlsr8266_hal::common::sync::{CriticalSectionMutex, SimplifyLS};
use tlsr8266_hal::mcu::crypto::{
    aes_att_decryption, aes_att_decryption_packet, aes_att_encryption, aes_att_encryption_packet,
    encode_password,
};
use tlsr8266_pac::read_reg_system_tick;

//------------------------------------------------------------------------------
// BLE Pairing and Security Module
//
// This module implements Bluetooth Low Energy pairing and security features for
// the TLSR8266 mesh light system. It provides functions for packet encryption
// and decryption, key management, and pairing handshake operations.
//
// The pairing sequence follows a multi-step protocol with specific state transitions
// handled through opcodes in pair_write and responses in pair_proc.
//
// The pairing protocol consists of the following phases:
//
// 1. Random Exchange
//    - Client sends PAIR_OP_EXCHANGE_RANDOM with random challenge (pair_randm)
//    - Server responds with its own random challenge (pair_rands)
//
// 2. Authentication
//    - Client sends PAIR_OP_VERIFY_CREDENTIALS with proof derived from credentials
//    - Server verifies proof and responds with session key material
//
// 3. Provisioning (after successful authentication)
//    - Client sends PAIR_OP_SET_MESH_NAME with mesh network name
//    - Client sends PAIR_OP_SET_MESH_PASSWORD with mesh network password
//    - Client sends PAIR_OP_SET_MESH_LTK with long-term key
//
// 4. Key Management
//    - Client can request LTK with PAIR_OP_GET_MESH_LTK
//    - Client can reset mesh with PAIR_OP_RESET_MESH
//
// 5. Maintenance
//    - Client can delete pairing with PAIR_OP_DELETE_PAIRING
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Pairing state and configuration storage
//
// These statics and types are owned by the pairing module (design 019 §1, batch
// 5). Readers outside this file import them by name from
// `crate::ble::pair`.
//------------------------------------------------------------------------------

/// Mesh network name used for pairing and access control.
/// 16-byte string identifier that devices must match to join the mesh network.
///
/// Visibility is `pub(crate)`: the facade module is the only entry point from
/// outside this crate, and the crate boundary is the fence C-10 established.
pub(crate) static PAIR_CONFIG_MESH_NAME: CriticalSectionMutex<[u8; 16]> =
    CriticalSectionMutex::new([0; 16]);

/// Mesh network password used for authentication during pairing.
/// 16-byte password that must be provided to join the mesh network.
pub(crate) static PAIR_CONFIG_MESH_PWD: CriticalSectionMutex<[u8; 16]> =
    CriticalSectionMutex::new([0; 16]);

/// Long-Term Key (LTK) for mesh network encryption.
/// 16-byte cryptographic key used to encrypt/decrypt mesh communications.
pub(crate) static PAIR_CONFIG_MESH_LTK: CriticalSectionMutex<[u8; 16]> =
    CriticalSectionMutex::new([0; 16]);

/// Current state of the mesh pairing process.
/// Tracks progress through pairing sequence: idle, setting, transmitting, receiving, etc.
pub(crate) static PAIR_SETTING_FLAG: CriticalSectionMutex<PairAuthState> =
    CriticalSectionMutex::new(PairAuthState::PairSetted);

/// Initialization Vector for Master (IVM) used in mesh encryption.
/// 8-byte nonce used with AES encryption for securing mesh communications from master.
static PAIR_IVM: CriticalSectionMutex<[u8; 8]> =
    CriticalSectionMutex::new([0, 0, 0, 0, 1, 0, 0, 0]);

/// Session key derived from mesh password for secure pairing.
/// 16-byte key used during the pairing process to encrypt configuration data.
pub(crate) static PAIR_CONFIG_PWD_ENCODE_SK: CriticalSectionMutex<[u8; 16]> =
    CriticalSectionMutex::new([0; 16]);

/// Seed the pairing configuration buffers from an injected [`RadioConfig`].
///
/// Each 16-byte buffer is rebuilt from zero, so a short configured value leaves
/// trailing NULs and a value longer than the buffer is clamped with `min`,
/// never overflowing it.
pub(crate) fn seed_pair_config(cfg: &RadioConfig) {
    let mut encode_sk = [0u8; 16];
    let len = min(cfg.default_mesh_pwd_encode_sk.len(), encode_sk.len());
    encode_sk[..len].copy_from_slice(&cfg.default_mesh_pwd_encode_sk.as_bytes()[..len]);
    *PAIR_CONFIG_PWD_ENCODE_SK.lock() = encode_sk;

    let mut mesh_name = [0u8; 16];
    let len = min(cfg.default_mesh_name.len(), mesh_name.len());
    mesh_name[..len].copy_from_slice(&cfg.default_mesh_name.as_bytes()[..len]);
    *PAIR_CONFIG_MESH_NAME.lock() = mesh_name;

    let mut mesh_pwd = [0u8; 16];
    let len = min(cfg.default_mesh_pwd.len(), mesh_pwd.len());
    mesh_pwd[..len].copy_from_slice(&cfg.default_mesh_pwd.as_bytes()[..len]);
    *PAIR_CONFIG_MESH_PWD.lock() = mesh_pwd;

    *PAIR_CONFIG_MESH_LTK.lock() = cfg.default_mesh_ltk;
}

/// Initialization Vector for Slave (IVS) used in mesh encryption.
/// 8-byte nonce used with AES encryption for securing mesh communications from slave.
static PAIR_IVS: CriticalSectionMutex<[u8; 8]> = CriticalSectionMutex::new([0; 8]);

pub struct PairStateData {
    pub pair_ltk: [u8; 16],
    pub pair_sk: [u8; 16],
    pub pair_work: [u8; 16],
    pub pair_nn: [u8; 16],
    pub pair_pass: [u8; 16],
    pub pair_ltk_mesh: [u8; 16],
    pub pair_sk_copy: [u8; 16],
    pub pair_rands: [u8; 8],
    pub pair_randm: [u8; 8],
}

/// Power-on pairing state, shared by the static initialiser and the test reset
/// so a reset cannot drift from a fresh boot.
pub(crate) const DEFAULT_PAIR_STATE: PairStateData = PairStateData {
    pair_ltk: [0; 16],
    pair_sk: [0; 16],
    pair_work: [0; 16],
    pair_nn: [0; 16],
    pair_pass: [0; 16],
    pair_ltk_mesh: [0; 16],
    pair_sk_copy: [0; 16],
    pair_rands: [0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7],
    pair_randm: [0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7],
};

/// Comprehensive pairing state data containing all cryptographic material.
/// Stores LTK, session keys, work buffers, network names, passwords, and random values
/// used during the mesh pairing and authentication process.
pub(crate) static PAIR_STATE: CriticalSectionMutex<PairStateData> =
    CriticalSectionMutex::new(DEFAULT_PAIR_STATE);

/// Global flag indicating whether mesh pairing mode is currently active.
/// When true, the device accepts pairing commands and can join new mesh networks.
pub(crate) static MESH_PAIR_ENABLE: AtomicBool = AtomicBool::new(false);

/// Current state of the BLE pairing finite state machine.
/// Tracks progress through pairing steps: idle, random exchange, key exchange, etc.
static BLE_PAIR_ST: AtomicPairState = AtomicPairState::new(PairState::Idle);

/// Flag indicating successful completion of pairing login process.
/// Set to true when device has been authenticated and can access mesh functions.
pub(crate) static PAIR_LOGIN_OK: AtomicBool = AtomicBool::new(false);

/// Flag indicating whether mesh encryption is currently enabled.
/// When true, all mesh communications are encrypted using the configured LTK.
static PAIR_ENC_ENABLE: AtomicBool = AtomicBool::new(false);

/// Access code derived from mesh name and password for RF communication.
/// 32-bit value used to identify and filter mesh network traffic.
pub(crate) static PAIR_AC: AtomicU32 = AtomicU32::new(0);

/// Flag indicating a pairing read operation is waiting for response.
/// Used to track pending configuration read requests during pairing.
static PAIR_READ_PENDING: AtomicBool = AtomicBool::new(false);

/// Global security enable flag for mesh communications.
/// When true, all mesh packets are encrypted before transmission.
pub(crate) static SECURITY_ENABLE: AtomicBool = AtomicBool::new(false);

// Replace the raw atomic with an atomic PairState wrapper
pub(crate) struct AtomicPairState(AtomicU8);

impl AtomicPairState {
    pub const fn new(state: PairState) -> Self {
        Self(AtomicU8::new(state as u8))
    }

    pub fn get(&self) -> PairState {
        PairState::from(self.0.load(Ordering::Relaxed))
    }

    pub fn set(&self, value: PairState) {
        self.0.store(value as u8, Ordering::Relaxed)
    }
}

// Packet field constants
const ATT_READ_RESPONSE_OPCODE: u8 = 0x0B; // ATT Read Response opcode
const MESH_BROADCAST_CHANNEL: u16 = 0xFFFF; // Channel ID for broadcast packets
const PACKET_TYPE_ENCRYPTED: u8 = 0x80; // Flag indicating packet is encrypted

// Buffer size constants
const KEY_SIZE: usize = 0x10; // Size of cryptographic keys (16 bytes)
const RANDOM_CHALLENGE_SIZE: usize = 8; // Size of random challenge values (8 bytes)
const HEADER_SIZE: u16 = 2; // Size of header in value field (2 bytes)

// Response packet size constants
const RESP_WITH_RANDOM: u16 = 10; // 8 bytes random + 2 header bytes
const RESP_WITH_KEY: u16 = 0x12; // 16 bytes key data + 2 header bytes

// Constants for flash operations
const SECTOR_SIZE: i32 = 0x40; // Size of each flash storage sector as i32

// Command codes for light events
const LGT_CMD_DEL_PAIR: u8 = 0xc7; // Delete pairing information
const LGT_CMD_PAIR_OK: u8 = 0xc5; // Pairing successful

/// Decrypt an incoming BLE packet using the current pairing session key
///
/// This function is called for encrypted packets when security is enabled.
/// It extracts initialization vectors and source information from the packet,
/// then uses the current pairing session key (pair_sk) to decrypt the payload.
///
/// @param ps The packet to decrypt
/// @return true if decryption was successful, false otherwise
pub fn pair_dec_packet(ps: &mut Packet) -> Result<(), ()> {
    // Early return if security is not enabled
    if !SECURITY_ENABLE.get() {
        return Err(());
    }

    let mut pair_ivm = PAIR_IVM.lock();

    // Extract sequence number from packet as part of IV
    pair_ivm[5..5 + 3].copy_from_slice(&ps.att_write().value.sno);

    // Extract source address from packet
    let src = ps.att_write().value.src;

    // Calculate data length from packet header
    let l2len = ps.head().l2cap_len as usize;

    // Bound the peer-controlled `l2cap_len` locally, from the wire layout, so
    // the in-place slice below cannot leave `PacketAttWrite`:
    // `DST_OFFSET` (18) is where the encrypted payload starts, and
    // `MAX_ENC_PAYLOAD` (25) is every byte from there to the end of the packet
    // struct. `l2cap_len` itself counts the 8-byte L2CAP header plus the
    // encrypted payload, so the accepted range is 9..=33.
    const DST_OFFSET: usize =
        core::mem::offset_of!(PacketAttWrite, value) + core::mem::offset_of!(PacketAttValue, dst);
    const MAX_ENC_PAYLOAD: usize = core::mem::size_of::<PacketAttWrite>() - DST_OFFSET;
    const MIN_L2CAP_LEN: usize = 9;
    const MAX_L2CAP_LEN: usize = 8 + MAX_ENC_PAYLOAD;
    if !(MIN_L2CAP_LEN..=MAX_L2CAP_LEN).contains(&l2len) {
        return Err(());
    }

    // Create a slice pointing to the payload data that needs decryption
    let data_ptr = addr_of_mut!(ps.att_write_mut().value.dst) as *mut u8;
    let data_len = l2len - 8;

    // Get current session key for decryption
    let pair_sk = &PAIR_STATE.lock().pair_sk;

    // Perform decryption using AES-CCM.
    //
    // SAFETY: `data_ptr` is `PacketAttValue::dst` in the caller's `&mut Packet`
    // (packet offset 18) and `data_len = l2cap_len - 8`. The local check above
    // requires `9 <= l2cap_len <= 8 + MAX_ENC_PAYLOAD`, so `1 <= data_len <= 25`
    // and `[18, 18 + data_len)` ends at most at 43, inside the 48-byte `Packet`
    // union (25 bytes remain from `dst` to the end of `PacketAttValue`). The
    // range is disjoint from the `pair_ivm` and `src` buffers and from the
    // session key, and the slice is consumed only by the in-place AES-CCM call,
    // so no other view of those bytes is live across it.
    let result = unsafe {
        // Create data slice and decrypt in-place
        let data = slice_from_raw_parts_mut(data_ptr, data_len);
        aes_att_decryption_packet(
            pair_sk,    // Session key
            &pair_ivm,  // IV modifier
            &src,       // Expected MIC (Message Integrity Check)
            &mut *data, // Data to decrypt (in place)
        )
    };
    if result.is_ok() {
        Ok(())
    } else {
        Err(())
    }
}

/// Encrypt an outgoing BLE packet using the current pairing session key
///
/// This function is called for packets that need to be encrypted when security is enabled.
/// It uses a timestamp as a sequence number to prevent replay attacks and updates the
/// initialization vector. The packet is then encrypted with the current session key.
///
/// @param ps The packet to encrypt
pub fn pair_enc_packet(ps: &mut Packet) {
    // Constants for encryption conditions and offsets
    const ENCRYPTION_CHANNEL: u16 = 4;
    const ENCRYPTION_OPCODE: u8 = 0x1b;
    const ENCRYPTION_HANDLE: u8 = 0x12;
    const SNO_LENGTH: usize = 3;
    const IV_SEQ_OFFSET: usize = 3;
    const IV_SRC_OFFSET: usize = 6;
    const L2CAP_HEADER_LEN: u16 = 10;

    // Check if encryption is needed and packet meets encryption criteria
    if !SECURITY_ENABLE.get()
        || ps.head().chan_id != ENCRYPTION_CHANNEL
        || ps.ll_app().opcode != ENCRYPTION_OPCODE
        || ps.ll_app().handle != ENCRYPTION_HANDLE
    {
        return;
    }

    // Create a sequence number from system tick to prevent replay attacks
    let tick = read_reg_system_tick();

    // Set sequence number in packet
    let sno = &mut ps.ll_app_mut().value.sno;
    sno[0] = tick as u8;
    sno[1] = (tick >> 8) as u8;
    sno[2] = (tick >> 16) as u8;

    // Update initialization vector with sequence number and source address
    let mut pair_ivs = PAIR_IVS.lock();
    pair_ivs[IV_SEQ_OFFSET..IV_SEQ_OFFSET + SNO_LENGTH].copy_from_slice(sno);

    // Extract source address (little-endian representation)
    let src = ps.ll_app().value.src;
    pair_ivs[IV_SRC_OFFSET] = src as u8; // Low byte
    pair_ivs[IV_SRC_OFFSET + 1] = (src >> 8) as u8; // High byte

    // Get session key for encryption
    let session_key = &PAIR_STATE.lock().pair_sk;

    // Calculate data length from L2CAP header length.
    //
    // `l2cap_len` is peer-controlled, so subtract checked (a low byte below 10
    // would otherwise underflow) and clamp to the bytes that actually follow
    // `value.op`: `OP_OFFSET` (20) is where the ciphertext starts and
    // `MAX_ENC_DATA_LEN` (23) is every byte from there to the end of
    // `PacketLlApp`. No `rf_len` check: these ATT packets carry `rf_len == 0`
    // until the link layer fills it in.
    const OP_OFFSET: usize =
        core::mem::offset_of!(PacketLlApp, value) + core::mem::offset_of!(AppCmdValue, op);
    const MAX_ENC_DATA_LEN: usize = core::mem::size_of::<PacketLlApp>() - OP_OFFSET;
    let Some(data_len) =
        ((ps.head().l2cap_len & 0xff) as usize).checked_sub(L2CAP_HEADER_LEN as usize)
    else {
        return;
    };
    if data_len > MAX_ENC_DATA_LEN {
        return;
    }

    // Create destination and payload slices for encryption.
    //
    let dst_slice =
        // SAFETY: `ps.ll_app().value.dst` is the 2-byte `AppCmdValue::dst` field at
        // packet offset 18, so `[18, 20)` lies inside the 48-byte `Packet` union and
        // is exactly `size_of::<u16>()`. The slice is the MIC output of the AES-CCM
        // call and never overlaps `data_slice`, which starts at `value.op` (packet
        // offset 20).
        unsafe { slice::from_raw_parts_mut(addr_of!(ps.ll_app().value.dst) as *mut u8, 2) };

    let data_slice =
        // SAFETY: `addr_of_mut!(ps.ll_app_mut().value.op)` is `AppCmdValue::op` at
        // packet offset 20 and `data_len = (l2cap_len & 0xff) - 10`, clamped above
        // to `<= MAX_ENC_DATA_LEN` (23), so `[20, 20 + data_len)` ends at most at
        // 43, inside the 48-byte `Packet` union (`AppCmdValue` ends at 33 and
        // `PacketLlApp::rsv` follows). The range is disjoint from `dst_slice` at
        // [18, 20) and is written in place by AES-CCM with no other live view of
        // those bytes.
        unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.ll_app_mut().value.op), data_len) };

    // Perform encryption using AES-CCM
    aes_att_encryption_packet(
        session_key, // Session key
        &pair_ivs,   // IV modifier
        dst_slice,   // Output buffer for the calculated MIC
        data_slice,  // Data to encrypt (in place)
    );
}

/// Decrypt a mesh network packet using the long-term key (LTK)
///
/// This function handles decryption of mesh packets, which use a different
/// encryption scheme than regular BLE packets. Mesh packets are encrypted with
/// the long-term key rather than the session key used for direct connections.
///
/// @param ps The mesh packet to decrypt
/// @return true if decryption was successful, false otherwise
pub fn pair_dec_packet_mesh(ps: &mut Packet) -> Result<(), ()> {
    // Early return if security is disabled - we treat packets as already decrypted
    if !SECURITY_ENABLE.get() {
        return Ok(());
    }

    // Validate this packet has the encryption flag set
    if ps.head()._type & PACKET_TYPE_ENCRYPTED == 0 {
        return Err(());
    }

    // Check if packet length is within valid range for decryption. `rf_len` is
    // peer-controlled, so the subtraction is checked: a short frame must be
    // rejected rather than underflow.
    let rf_len = ps.head().rf_len;
    let Some(payload_len) = (rf_len as usize).checked_sub(0x12) else {
        return Err(());
    };
    if payload_len > 0x13 {
        return Err(());
    }

    // Get the long-term key for mesh decryption
    let ltk = &PAIR_STATE.lock().pair_ltk;

    // Decrypt the packet based on its type (broadcast vs directed)
    let is_broadcast = ps.head().chan_id == MESH_BROADCAST_CHANNEL;

    let result = if is_broadcast {
        // Broadcast packet structure:
        // - IV: first 8 bytes starting from rf_len
        // - Expected MIC: 2 bytes from internal_par2[1]
        // - Data: 0x1c bytes starting from sno field
        aes_att_decryption_packet(
            ltk, // Long-term key
            // SAFETY: the broadcast IV is the 8 raw wire bytes starting at
            // `rf_len` (packet offset 5), i.e. `[5, 13)`: `rf_len`, `l2cap_len`,
            // `chan_id`, `src_tx` and `handle1` are read as one byte block because
            // that is how the wire format defines the nonce. `[u8; 8]` needs only
            // alignment 1, the range is inside the 48-byte `Packet`, and it is
            // read-only for the duration of the call.
            unsafe { &*(addr_of!(ps.head().rf_len) as *const [u8; 8]) }, // IV
            // SAFETY: the expected MIC is `internal_par2[1]` at packet offset 41;
            // the 2-byte `[41, 43)` range is inside the 48-byte `Packet`, is only
            // read here, and does not overlap the IV `[5, 13)` or the data block
            // `[13, 41)`.
            unsafe { slice::from_raw_parts(addr_of!(ps.mesh().internal_par2[1]), 2) }, // Expected MIC
            // SAFETY: the data block is the fixed 0x1c-byte mesh body starting at
            // `sno` (packet offset 13), i.e. `[13, 41)`. That is inside the
            // 48-byte `Packet` and ends exactly where the MIC starts, so the
            // in-place AES-CCM write cannot touch the IV or the MIC buffer.
            unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().sno) as *mut u8, 0x1c) }, // Data
        )
    } else {
        // Direct mesh packet structure:
        // - IV: 8 bytes starting from handle1 field
        // - Expected MIC: 4 bytes at position sno + (rf_len - 0xb)
        // - Data: payload from op field with length (rf_len - 0x12)
        aes_att_decryption_packet(
            ltk, // Long-term key
            // SAFETY: the direct-mesh IV is the 8 raw wire bytes starting at
            // `handle1` (packet offset 12), i.e. `[12, 20)`, covering `handle1`,
            // `sno`, `src_adr` and `dst_adr` as the wire nonce. `[u8; 8]` needs
            // only alignment 1, the range is inside the 48-byte `Packet`, and it
            // is read-only for the call.
            unsafe { &*(addr_of!(ps.mesh().handle1) as *const [u8; 8]) }, // IV
            // SAFETY: the expected MIC is 4 bytes at `sno + (rf_len - 0xb)`,
            // i.e. packet offset `rf_len + 2`. Above, `rf_len - 0x12` is computed
            // with `checked_sub` and rejected when it exceeds 0x13, so
            // `0x12 <= rf_len <= 0x25` and `[rf_len + 2, rf_len + 6)` ends at most
            // at 43, inside the 48-byte `Packet`. Provenance is preserved by
            // `addr_of!().cast::<u8>().add(n)` and `rf_len - 0xb <= 0x1a`, so the
            // result stays inside the 48-byte `Packet`. The range is only read and
            // does not overlap the IV or the data block.
            unsafe {
                slice::from_raw_parts(
                    addr_of!(ps.mesh().sno)
                        .cast::<u8>()
                        .add(rf_len as usize - 0xb),
                    4,
                )
            }, // Expected MIC
            // SAFETY: the payload is `payload_len = rf_len - 0x12 <= 0x13` bytes
            // (checked above) starting at `op` (packet offset 20), i.e.
            // `[20, rf_len + 2)`. With `rf_len <= 0x25` that ends at most at 39,
            // inside the 48-byte `Packet`, and it ends exactly where the MIC range
            // starts, so the in-place AES-CCM write does not disturb the IV or the
            // MIC.
            unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().op), payload_len) }, // Data
        )
    };
    if result.is_ok() {
        Ok(())
    } else {
        Err(())
    }
}

/// This function handles encryption of mesh packets before transmission.
/// Similar to decryption, it uses the long-term key rather than the session key.
///
/// @param ps The mesh packet to encrypt
/// @return true if encryption was successful, false otherwise
pub fn pair_enc_packet_mesh(ps: &mut Packet) {
    // Only proceed with encryption if security is enabled
    if SECURITY_ENABLE.get() {
        // Get access to the long-term key used for mesh encryption
        let pair_ltk = &PAIR_STATE.lock().pair_ltk;

        // Determine encryption method based on packet type (broadcast vs direct)
        let is_broadcast = ps.head().chan_id == MESH_BROADCAST_CHANNEL;

        if is_broadcast {
            // Broadcast packet encryption
            // - Use IV from packet header's rf_len field (8 bytes)
            // SAFETY: same 8-byte wire nonce as the decrypt path: the raw block
            // starting at `rf_len` (packet offset 5), `[5, 13)`, read-only, inside
            // the 48-byte `Packet`; `[u8; 8]` needs only alignment 1.
            let iv = unsafe { &*(addr_of!(ps.head().rf_len) as *const [u8; 8]) };

            // - Use destination address from internal_par2[1] as the MIC output buffer (2 bytes)
            // SAFETY: `internal_par2[1]` is packet offset 41; the 2-byte MIC
            // output `[41, 43)` is inside `Packet`, is written only by AES-CCM,
            // and does not overlap the IV `[5, 13)` or the payload `[13, 41)`.
            let dst = unsafe {
                slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().internal_par2[1]), 2)
            };

            // - Encrypt payload starting from sno field (fixed size 0x1c bytes)
            // SAFETY: `sno` is packet offset 13 and the encrypted body is a fixed
            // 0x1c bytes, `[13, 41)`, inside `Packet`; it ends where the MIC
            // starts, so the in-place write leaves the IV and MIC intact.
            let payload = unsafe {
                slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().sno) as *mut u8, 0x1c)
            };

            // Perform encryption using AES-CCM
            aes_att_encryption_packet(pair_ltk, iv, dst, payload); // dst is the mic_output buffer
        } else {
            // Direct mesh packet encryption
            // - Use IV from packet's handle1 field (8 bytes)
            // SAFETY: the 8-byte wire nonce starts at `handle1` (packet offset
            // 12), `[12, 20)`, read-only and inside the 48-byte `Packet`;
            // alignment 1 suffices for `[u8; 8]`.
            let iv = unsafe { &*(addr_of!(ps.mesh().handle1) as *const [u8; 8]) };

            // - Bound the peer-controlled `rf_len` before any pointer arithmetic:
            // the direct payload is `rf_len - 0x12` bytes and the MIC starts at
            // `rf_len + 2`, so a short frame must be dropped (checked subtraction)
            // and a long one is rejected past the 0x13-byte payload ceiling.
            let Some(payload_len) = (ps.head().rf_len as usize).checked_sub(0x12) else {
                return;
            };
            if payload_len > 0x13 {
                return;
            }

            // - Use destination address from position calculated based on rf_len as the MIC output buffer (4 bytes)
            // SAFETY: pointer arithmetic only, no dereference: `sno` is packet
            // offset 13 and the local bound above guarantees `rf_len <= 0x25`, so
            // `rf_len - 0xb <= 0x1a` and the result is at most offset 39 inside
            // the 48-byte `Packet` (never past one-past-the-end). Provenance is
            // preserved by `addr_of_mut!().cast::<u8>().add(n)`, which also keeps
            // the mutable borrow that the later write requires.
            let dst_ptr = unsafe {
                addr_of_mut!(ps.mesh_mut().sno)
                    .cast::<u8>()
                    .add(ps.head().rf_len as usize - 0xb)
            };
            // SAFETY: the 4-byte MIC output starts at `rf_len + 2` and the
            // payload below ends there, so `[rf_len + 2, rf_len + 6)` is inside the
            // 48-byte `Packet` for every `rf_len <= 0x25` (enforced above) and
            // never overlaps the IV `[12, 20)` or the payload `[20, rf_len + 2)`.
            let dst = unsafe { slice::from_raw_parts_mut(dst_ptr, 4) };

            // - Encrypt payload starting from op field with length based on rf_len
            let payload =
                // SAFETY: `op` is packet offset 20 and `payload_len = rf_len - 0x12`
                // is `<= 0x13` (enforced above), so the payload is
                // `[20, rf_len + 2)` and stays inside the 48-byte `Packet`, flush
                // against the MIC buffer.
                unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().op), payload_len) };

            // Perform encryption using AES-CCM
            aes_att_encryption_packet(pair_ltk, iv, dst, payload); // dst is the mic_output buffer
        }
    }
}

/// Save pairing configuration data to flash memory
///
/// This function handles the saving of pairing data segments to flash memory.
/// It manages the flash sector rotation for wear-leveling when a new set of
/// pairing data needs to be stored.
///
/// @param addr The offset within the pairing data structure
/// @param data The data to save to flash
pub fn pair_flash_save_config(addr: u32, data: &[u8]) {
    // Flash storage offset constants
    const OFFSET_HEADER: u32 = 0x00; // Pairing header with validation flags

    // For a new config set (addr=0), clean the old sector and increment the index
    if addr == OFFSET_HEADER {
        // Cast to i32 to match FLASH_CONFIGURATION_INDEX type
        // Special case: if index is -1 (no config), set to 0, otherwise increment by SECTOR_SIZE
        let current_index = FLASH_CONFIGURATION_INDEX.get();
        if current_index < 0 {
            FLASH_CONFIGURATION_INDEX.set(0);
        } else {
            FLASH_CONFIGURATION_INDEX.set(current_index + SECTOR_SIZE);
        }

        // Clean the flash (handles wraparound if needed)
        // Note: pair_flash_clean() may reset index back to 0 if we wrapped around
        pair_flash_clean();
    }

    // Save the data to flash at the calculated address
    save_pair_info(addr, data);
}

/// Save the current pairing keys to flash memory
///
/// This function saves all the pairing information (mesh name, password, and
/// long-term key) to persistent storage in flash memory. It also includes
/// validation flags and mesh-specific configuration.
pub fn pair_save_key() {
    // Flash storage offset constants
    const OFFSET_HEADER: u32 = 0x00; // Pairing header with validation flags
    const OFFSET_NAME: u32 = 0x10; // Mesh network name
    const OFFSET_PASSWORD: u32 = 0x20; // Mesh network password (encrypted)
    const OFFSET_LTK: u32 = 0x30; // Long-term key

    let mut pass = [0u8; 16];

    // Set validation flags that indicate valid pairing data
    pass[0] = PAIR_CONFIG_VALID_FLAG;
    pass[15] = PAIR_CONFIG_VALID_FLAG;

    // Store mesh MAC address configuration if mesh pairing is enabled
    if MESH_PAIR_ENABLE.get() {
        pass[1] = if MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get() {
            1
        } else {
            0
        };
    }

    let pair_state = PAIR_STATE.lock();

    // Save header with validation flags first
    pair_flash_save_config(OFFSET_HEADER, &pass);

    // Save mesh name
    pair_flash_save_config(OFFSET_NAME, &pair_state.pair_nn);

    // Make a copy of the password before encoding
    pass = pair_state.pair_pass;

    // Encode password before saving to flash for security.
    // Copy the key out of the mutex first so the lock is released before
    // encode_password runs (its AES call masks interrupts).
    let key = *PAIR_CONFIG_PWD_ENCODE_SK.lock();
    pass = encode_password(&key, &pass);

    // Save encoded password
    pair_flash_save_config(OFFSET_PASSWORD, &pass);

    // Save long-term key
    pair_flash_save_config(OFFSET_LTK, &pair_state.pair_ltk);

    // Release the pairing state before `pair_update_key` re-locks it. On the
    // device a nested lock on the same mutex is allowed, but the host build's
    // re-entrancy probe would otherwise abort the call. The window is
    // deliberate: every ISR-context reader of `PAIR_STATE` is read-only, and a
    // concurrent pairing command is superseded by this reset.
    drop(pair_state);

    // Update runtime keys and access codes based on the new pairing data
    pair_update_key();
}

/// Initialize pairing state for a new connection
///
/// This function resets the pairing state and prepares initialization vectors
/// for a new BLE connection. It uses the device's MAC address as part of the
/// initialization vectors for encryption.
pub fn pair_init() {
    // Reset pairing state to initial value
    BLE_PAIR_ST.set(PairState::Idle);
    PAIR_ENC_ENABLE.set(false);

    // Initialize the master and slave initialization vectors with MAC ID
    const MAC_ID_SIZE: usize = 4; // Using first 4 bytes of MAC address for IVs
    PAIR_IVM.lock()[0..MAC_ID_SIZE].copy_from_slice(&MAC_ID.lock()[0..MAC_ID_SIZE]);
    PAIR_IVS.lock()[0..MAC_ID_SIZE].copy_from_slice(&MAC_ID.lock()[0..MAC_ID_SIZE]);
}

/// The two initialization vectors as `(master, slave)`, for host tests.
///
/// `pair_init` seeds both from `MAC_ID`, but the statics are private to this
/// module. Callers outside it (the connection tests) read the result through
/// here rather than widening the statics.
#[cfg(not(target_os = "none"))]
pub fn pair_iv_for_test() -> ([u8; 8], [u8; 8]) {
    (*PAIR_IVM.lock(), *PAIR_IVS.lock())
}

/// Initialize pairing parameters from stored keys
///
/// This function sets the pairing state to a special initialization value
/// and loads previously stored keys from flash memory.
pub fn pair_par_init() {
    // Set pairing state to the initialization state
    BLE_PAIR_ST.set(PairState::Init);

    // Load keys from flash memory into the pairing state
    pair_load_key();
}

/// Process pairing responses for BLE connection
///
/// This function generates appropriate responses to pairing-related read requests
/// based on the current pairing state. It implements the server-side of the pairing
/// protocol, responding to client requests with the appropriate security tokens.
///
/// The pairing state machine (BLE_PAIR_ST) controls the flow of the pairing process:
/// - PairState::AwaitingRandom: Initial pairing response with random challenge
/// - PairState::ReceivingMeshLtk: Compute and send encrypted pairing data
/// - PairState::SessionKeyExchange: Exchange session keys for encryption
/// - PairState::RequestingLtk: Provide the long-term key
/// - PairState::DeletePairing: Handle pairing deletion
/// - PairState::Init: Special initialization state
///
/// @return An optional packet containing the pairing response, or None if no response needed
pub fn pair_proc() -> Option<Packet> {
    // Only generate responses when a read request is pending
    let pair_st = BLE_PAIR_ST.get();
    if !PAIR_READ_PENDING.get() {
        return None;
    }
    PAIR_READ_PENDING.set(false);

    let mut pair_state = PAIR_STATE.lock();

    // Initialize response packet with standard header
    let mut pkt_read_rsp = Packet {
        att_read_rsp: PacketAttReadRsp {
            head: PacketL2capHead {
                dma_len: 0x1d,
                _type: 2,
                rf_len: 0x1b,
                l2cap_len: 0x17,
                chan_id: 0x4,
            },
            opcode: ATT_READ_RESPONSE_OPCODE, // ATT Read Response opcode
            value: [0; 22],
        },
    };

    // Special case for Init state - has a different return path
    if BLE_PAIR_ST.get() == PairState::Init {
        // Return status and reset state
        pkt_read_rsp.head_mut().l2cap_len = HEADER_SIZE;
        pkt_read_rsp.head_mut().dma_len = 8;
        pkt_read_rsp.head_mut().rf_len = 0x6;

        // Return current pairing state as a raw u8 value
        pkt_read_rsp.att_read_rsp_mut().value[0] = pair_st as u8;

        // Reset pairing state
        BLE_PAIR_ST.set(PairState::Idle);

        return Some(pkt_read_rsp);
    }

    // Common security check for multiple states
    let security_check_needed = BLE_PAIR_ST.get() == PairState::AwaitingRandom
        || BLE_PAIR_ST.get() == PairState::ReceivingMeshLtk;

    if security_check_needed && !SECURITY_ENABLE.get() && !PAIR_LOGIN_OK.get() {
        // `pair_par_init` re-locks `PAIR_STATE` (through `pair_load_key`), so the
        // guard must not be held across it.
        drop(pair_state);
        pair_par_init();
        PAIR_ENC_ENABLE.set(false);
        return None;
    }

    // Process each state with its specific logic
    match BLE_PAIR_ST.get() {
        PairState::AwaitingRandom => {
            // Send random challenge for authentication
            pkt_read_rsp.head_mut().l2cap_len = RESP_WITH_RANDOM;
            pkt_read_rsp.att_read_rsp_mut().value[1..1 + RANDOM_CHALLENGE_SIZE]
                .copy_from_slice(&pair_state.pair_rands);
            BLE_PAIR_ST.set(PairState::RandomConfirmation);
        }

        PairState::ReceivingMeshLtk => {
            // Compute mesh network keys - XOR network name, password, and LTK
            pkt_read_rsp.head_mut().l2cap_len = RESP_WITH_KEY;

            // Compute obfuscated key material by XORing identity values
            for index in 0..KEY_SIZE {
                pair_state.pair_work[index] = pair_state.pair_nn[index]
                    ^ pair_state.pair_pass[index]
                    ^ pair_state.pair_ltk[index];
            }

            // Either send as plaintext or encrypt based on security mode
            if !SECURITY_ENABLE.get() {
                pkt_read_rsp.att_read_rsp_mut().value[1..1 + KEY_SIZE]
                    .copy_from_slice(&pair_state.pair_work);
            } else {
                let encrypted_work = aes_att_encryption(&pair_state.pair_sk, &pair_state.pair_work);
                pkt_read_rsp.att_read_rsp_mut().value[1..1 + KEY_SIZE]
                    .copy_from_slice(&encrypted_work);
            }

            BLE_PAIR_ST.set(PairState::Completed);
        }

        PairState::SessionKeyExchange => {
            pkt_read_rsp.head_mut().l2cap_len = RESP_WITH_KEY;

            if !SECURITY_ENABLE.get() {
                // Simple mode: Just XOR network name and password
                for index in 0..KEY_SIZE {
                    pair_state.pair_work[index] =
                        pair_state.pair_nn[index] ^ pair_state.pair_pass[index];
                }

                // Send obfuscated credentials
                pkt_read_rsp.att_read_rsp_mut().value[1..1 + KEY_SIZE]
                    .copy_from_slice(&pair_state.pair_work);

                BLE_PAIR_ST.set(PairState::Completed);
                PAIR_ENC_ENABLE.set(false);
            } else {
                // Secure mode: Generate session key using AES
                // Generate random challenge using system tick
                pair_state.pair_rands[0..4]
                    .copy_from_slice(bytemuck::bytes_of(&read_reg_system_tick()));

                // Prepare padded buffers for AES operations
                let mut key_padded = [0u8; KEY_SIZE];
                let mut source_padded = [0u8; KEY_SIZE];
                key_padded[0..RANDOM_CHALLENGE_SIZE].copy_from_slice(&pair_state.pair_randm);
                source_padded[0..RANDOM_CHALLENGE_SIZE].copy_from_slice(&pair_state.pair_rands);

                // Generate intermediate session key material using padded values
                pair_state.pair_sk = aes_att_encryption(&key_padded, &source_padded);

                // Extract first half of encryption result as new slave random
                let mut tmp = [0; RANDOM_CHALLENGE_SIZE];
                tmp.copy_from_slice(&pair_state.pair_sk[0..RANDOM_CHALLENGE_SIZE]);
                pair_state.pair_rands.copy_from_slice(&tmp);

                // Zero out second half of session key for security
                pair_state.pair_sk[RANDOM_CHALLENGE_SIZE..KEY_SIZE].fill(0);

                // Prepare work buffer with XORed credential data
                for index in 0..KEY_SIZE {
                    pair_state.pair_work[index] =
                        pair_state.pair_nn[index] ^ pair_state.pair_pass[index];
                }

                // Generate proof of possession by encrypting credentials with session key
                pair_state.pair_work =
                    aes_att_encryption(&pair_state.pair_sk, &pair_state.pair_work);

                // Pack slave random challenge and encrypted proof into response
                pkt_read_rsp.att_read_rsp_mut().value[1..1 + RANDOM_CHALLENGE_SIZE]
                    .copy_from_slice(&pair_state.pair_rands);
                pkt_read_rsp.att_read_rsp_mut().value[9..9 + RANDOM_CHALLENGE_SIZE]
                    .copy_from_slice(&pair_state.pair_work[0..RANDOM_CHALLENGE_SIZE]);

                // Prepare work buffer again for session key derivation
                for index in 0..KEY_SIZE {
                    pair_state.pair_work[index] =
                        pair_state.pair_nn[index] ^ pair_state.pair_pass[index];
                }

                // Derive final session key using both random values
                let pair_randm = pair_state.pair_randm;
                let pair_rands = pair_state.pair_rands;
                pair_state.pair_sk[0..RANDOM_CHALLENGE_SIZE].copy_from_slice(&pair_randm);
                pair_state.pair_sk[RANDOM_CHALLENGE_SIZE..KEY_SIZE].copy_from_slice(&pair_rands);

                pair_state.pair_sk = aes_att_encryption(&pair_state.pair_work, &pair_state.pair_sk);

                // Set state to completed and enable encryption
                BLE_PAIR_ST.set(PairState::Completed);
                PAIR_ENC_ENABLE.set(true);
            }
        }

        PairState::RequestingLtk => {
            // Provide the LTK either directly or encrypted based on security mode
            pkt_read_rsp.head_mut().l2cap_len = RESP_WITH_KEY;

            if !SECURITY_ENABLE.get() {
                // Simple mode: Send LTK directly
                pkt_read_rsp.att_read_rsp_mut().value[1..1 + KEY_SIZE]
                    .copy_from_slice(&pair_state.pair_ltk);
            } else {
                // Secure mode: Encrypt LTK with a derived key
                // Use master random as part of key derivation
                let pair_randm = pair_state.pair_randm;
                pair_state.pair_work[0..RANDOM_CHALLENGE_SIZE].copy_from_slice(&pair_randm);
                pair_state.pair_work[RANDOM_CHALLENGE_SIZE..KEY_SIZE].fill(0);

                // Create encryption key by XORing credentials with random material
                for index in 0..KEY_SIZE {
                    pair_state.pair_sk[index] = pair_state.pair_nn[index]
                        ^ pair_state.pair_pass[index]
                        ^ pair_state.pair_work[index];
                }

                // Encrypt the LTK with the derived key
                let encrypted_ltk = aes_att_encryption(&pair_state.pair_sk, &pair_state.pair_ltk);
                pkt_read_rsp.att_read_rsp_mut().value[1..1 + KEY_SIZE]
                    .copy_from_slice(&encrypted_ltk);
                BLE_PAIR_ST.set(PairState::Completed);

                // Restore saved session key
                pair_state.pair_sk = pair_state.pair_sk_copy;
            }
        }

        PairState::DeletePairing => {
            // Remove pairing information and notify application
            pkt_read_rsp.head_mut().l2cap_len = HEADER_SIZE;
            BLE_PAIR_ST.set(PairState::Idle);

            // Enable MAC address handling for mesh if needed
            if MESH_PAIR_ENABLE.get() {
                MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(true);
            }

            // `rf_link_delete_pair` re-enters `PAIR_STATE` (pair_set_key /
            // pair_save_key), so the guard must be released first.
            drop(pair_state);

            // Delete pairing data and notify application
            rf_link_delete_pair();
            crate::mesh_ll::adv::emit_light_event(LGT_CMD_DEL_PAIR);
        }

        PairState::Completed => {
            // For completed state, send a simple status response
            pkt_read_rsp.head_mut().l2cap_len = HEADER_SIZE;
            // State will be set below in the common code
        }

        _ => {
            // For any other states, use default packet configuration
        }
    }

    // Calculate packet length fields based on content
    pkt_read_rsp.head_mut().rf_len = pkt_read_rsp.head_mut().l2cap_len as u8 + 0x4;
    pkt_read_rsp.head_mut().dma_len = pkt_read_rsp.head_mut().l2cap_len as u32 + 6;

    // Set first byte of value to current pairing state as a raw u8 value
    pkt_read_rsp.att_read_rsp_mut().value[0] = pair_st as u8;

    Some(pkt_read_rsp)
}

/// Set pairing keys from provided key material
///
/// This function is used to set the mesh network name, password, and long-term key
/// from a provided buffer containing all three keys in sequence. After setting
/// the keys in memory, it calls pair_update_key to compute derived values.
///
/// @param key Buffer containing the mesh network name (16 bytes), password (16 bytes),
///            and long-term key (16 bytes)
pub fn pair_set_key(key: &[u8]) {
    {
        let mut pair_state = PAIR_STATE.lock();
        // Set mesh network name (limited by MAX_MESH_NAME_LEN)
        pair_state.pair_nn[0..MAX_MESH_NAME_LEN].copy_from_slice(&key[0..MAX_MESH_NAME_LEN]);
        // Set mesh network password
        pair_state.pair_pass.copy_from_slice(&key[16..16 + 16]);
        // Set mesh long-term key
        pair_state.pair_ltk.copy_from_slice(&key[32..32 + 16]);
    }

    // Update derived key material and access codes
    pair_update_key();
}

/// Mark a pairing read request as pending
///
/// This simple function sets a flag indicating that a pairing read request
/// has been received and needs to be processed by the pair_proc function.
///
/// @param _ Unused packet parameter
pub fn pair_read(_: &Packet) {
    PAIR_READ_PENDING.set(true);
}

/// Process pairing commands from a connected BLE device
///
/// This function implements the client-side of the BLE pairing protocol, processing
/// incoming commands and managing the pairing state machine. The first byte of the
/// request payload contains an opcode that identifies the pairing command:
///
/// Opcodes:
/// - PAIR_OP_EXCHANGE_RANDOM: Exchange random challenge for authentication
/// - PAIR_OP_SET_MESH_NAME: Set mesh network name
/// - PAIR_OP_SET_MESH_PASSWORD: Set mesh network password
/// - PAIR_OP_SET_MESH_LTK: Set long-term key for mesh encryption
/// - PAIR_OP_GET_MESH_LTK: Request long-term key
/// - PAIR_OP_RESET_MESH: Reset mesh configuration to defaults
/// - PAIR_OP_VERIFY_CREDENTIALS: Verify credentials with challenge-response
/// - PAIR_OP_DELETE_PAIRING: Delete all pairing information
///
/// The pairing flow consists of these primary phases:
/// 1. Random exchange - Exchange random values to derive session key
/// 2. Authentication - Verify identity with challenge-response
/// 3. Provisioning - Set up network parameters (name, password, key)
/// 4. Key management - Manage encryption keys
/// 5. Maintenance - Delete pairing information or reset mesh
///
/// @param data Packet containing the pairing command
/// Processing pair_write commands always completes; the function has void return.
pub fn pair_write(data: &Packet) {
    let pktdata = &data.att_val().value;
    let opcode = pktdata[0];
    let src = &pktdata[1..];
    let mut pair_state = PAIR_STATE.lock();

    // Process each opcode to handle pairing steps
    match opcode {
        // --- Step 1: Random Challenge Exchange ---
        PAIR_OP_EXCHANGE_RANDOM => {
            // Store the master random challenge and await slave response
            pair_state
                .pair_randm
                .copy_from_slice(&src[0..RANDOM_CHALLENGE_SIZE]);
            BLE_PAIR_ST.set(PairState::AwaitingRandom);
        }

        // --- Step 2: Network Provisioning - Set Network Name ---
        PAIR_OP_SET_MESH_NAME => {
            // Security Mode handling
            if !SECURITY_ENABLE.get() {
                // Simple mode: Must be logged in and in completed state
                if PAIR_LOGIN_OK.get() && BLE_PAIR_ST.get() == PairState::Completed {
                    pair_state.pair_nn.copy_from_slice(&src[0..KEY_SIZE]);
                    *PAIR_SETTING_FLAG.lock() = PairAuthState::PairSetting;
                    BLE_PAIR_ST.set(PairState::ReceivingMeshName);
                    return;
                }
            } else if BLE_PAIR_ST.get() == PairState::Completed {
                // Secure mode: Decrypt and validate the mesh name
                pair_state.pair_work.copy_from_slice(&src[0..KEY_SIZE]);
                pair_state.pair_nn = aes_att_decryption(&pair_state.pair_sk, &pair_state.pair_work);

                // Find name length (up to null terminator or full length)
                let name_len = match pair_state.pair_nn.iter().position(|r| *r == 0) {
                    Some(v) => v,
                    None => pair_state.pair_nn.len(),
                };

                // Only accept valid mesh names
                if name_len <= MAX_MESH_NAME_LEN {
                    *PAIR_SETTING_FLAG.lock() = PairAuthState::PairSetting;
                    BLE_PAIR_ST.set(PairState::ReceivingMeshName);
                    return;
                }
            }

            // Invalid mesh name or state - reset pairing
            // `pair_par_init` re-locks `PAIR_STATE` through `pair_load_key`.
            drop(pair_state);
            pair_par_init();
            PAIR_ENC_ENABLE.set(false);
        }

        // --- Step 3: Network Provisioning - Set Network Password ---
        PAIR_OP_SET_MESH_PASSWORD => {
            // Input validation based on security mode
            if SECURITY_ENABLE.get() {
                // Secure mode: Must be in ReceivingMeshName state
                if BLE_PAIR_ST.get() != PairState::ReceivingMeshName {
                    drop(pair_state);
                    pair_par_init();
                    PAIR_ENC_ENABLE.set(false);
                    return;
                }

                // Decrypt password using session key
                pair_state.pair_work.copy_from_slice(&src[0..KEY_SIZE]);
                pair_state.pair_pass =
                    aes_att_decryption(&pair_state.pair_sk, &pair_state.pair_work);
            } else {
                // Simple mode: Must be logged in and in correct state
                if !PAIR_LOGIN_OK.get() || BLE_PAIR_ST.get() != PairState::ReceivingMeshName {
                    drop(pair_state);
                    pair_par_init();
                    PAIR_ENC_ENABLE.set(false);
                    return;
                }

                // Store plaintext password
                pair_state.pair_pass.copy_from_slice(&src[0..KEY_SIZE]);
            }

            // Advance to next state after validation passes
            BLE_PAIR_ST.set(PairState::ReceivingMeshPassword);

            // Security check: Password must differ from network name
            if pair_state.pair_nn != pair_state.pair_pass {
                return;
            }

            // Password identical to name - security violation
            drop(pair_state);
            pair_par_init();
            PAIR_ENC_ENABLE.set(false);
        }

        // --- Step 4: Network Provisioning - Set LTK ---
        PAIR_OP_SET_MESH_LTK => {
            // Constants for mesh flag parsing
            const MESH_FLAG_OFFSET: usize = 0x11;
            const MIN_PACKET_LEN_WITH_MESH: u16 = 0x14;
            let has_mesh_flag = MIN_PACKET_LEN_WITH_MESH < data.head().l2cap_len;

            // Secure mode handling
            if SECURITY_ENABLE.get() {
                // Must be in correct state
                if BLE_PAIR_ST.get() != PairState::ReceivingMeshPassword {
                    drop(pair_state);
                    pair_par_init();
                    *PAIR_SETTING_FLAG.lock() = PairAuthState::PairSetted;
                    PAIR_ENC_ENABLE.set(false);
                    return;
                }

                // Process encrypted LTK
                pair_state.pair_work.copy_from_slice(&src[0..KEY_SIZE]);
                BLE_PAIR_ST.set(PairState::ReceivingMeshLtk);

                // Check for mesh network mode
                if MESH_PAIR_ENABLE.get() && has_mesh_flag && pktdata[MESH_FLAG_OFFSET] != 0 {
                    // Mesh mode: Decrypt mesh LTK
                    pair_state.pair_ltk_mesh =
                        aes_att_decryption(&pair_state.pair_sk, &pair_state.pair_work);
                    *PAIR_SETTING_FLAG.lock() = PairAuthState::PairSetMeshTxStart;
                    return;
                }

                // Regular mode: Decrypt standard LTK
                pair_state.pair_ltk =
                    aes_att_decryption(&pair_state.pair_sk, &pair_state.pair_work);
                // `pair_save_key` re-locks `PAIR_STATE`, so release it first.
                drop(pair_state);
                pair_save_key();
                *PAIR_SETTING_FLAG.lock() = PairAuthState::PairSetted;
                crate::mesh_ll::adv::emit_light_event(LGT_CMD_PAIR_OK);
                return;
            }
            // Simple mode handling
            else if PAIR_LOGIN_OK.get() && BLE_PAIR_ST.get() == PairState::ReceivingMeshPassword {
                // Clear and prepare LTK
                pair_state.pair_ltk.fill(0);
                BLE_PAIR_ST.set(PairState::ReceivingMeshLtk);

                // Check for mesh network mode
                if MESH_PAIR_ENABLE.get()
                    && has_mesh_flag
                    && (pktdata[MESH_FLAG_OFFSET] & 0x80) != 0
                {
                    // Mesh mode: Store plaintext mesh LTK
                    pair_state.pair_ltk_mesh.copy_from_slice(&src[0..KEY_SIZE]);
                    *PAIR_SETTING_FLAG.lock() = PairAuthState::PairSetMeshTxStart;
                    return;
                }

                // Regular mode: Store plaintext standard LTK
                pair_state.pair_ltk.copy_from_slice(&src[0..KEY_SIZE]);
                // `pair_save_key` re-locks `PAIR_STATE`, so release it first.
                drop(pair_state);
                pair_save_key();
                *PAIR_SETTING_FLAG.lock() = PairAuthState::PairSetted;
                crate::mesh_ll::adv::emit_light_event(LGT_CMD_PAIR_OK);
                return;
            }

            // Invalid state - reset pairing
            drop(pair_state);
            pair_par_init();
            *PAIR_SETTING_FLAG.lock() = PairAuthState::PairSetted;
            PAIR_ENC_ENABLE.set(false);
        }

        // --- Step 5: Pair Maintenance - Delete Pairing ---
        PAIR_OP_DELETE_PAIRING => {
            PAIR_ENC_ENABLE.set(false);
            BLE_PAIR_ST.set(PairState::Idle);
        }

        // --- Step 5: Pair Maintenance - Reset Mesh ---
        PAIR_OP_RESET_MESH => {
            // No specific action yet for reset mesh
        }

        // --- Combined handling for LTK Request and Credential Verification ---
        PAIR_OP_GET_MESH_LTK | PAIR_OP_VERIFY_CREDENTIALS => {
            let is_get_ltk = opcode == PAIR_OP_GET_MESH_LTK;
            let is_verify = opcode == PAIR_OP_VERIFY_CREDENTIALS;

            // For GET_MESH_LTK: Save the original session key before modification
            if is_get_ltk {
                pair_state.pair_sk_copy = pair_state.pair_sk;
            }

            // Store master random and prepare session key
            pair_state
                .pair_randm
                .copy_from_slice(&src[0..RANDOM_CHALLENGE_SIZE]);
            let randm = pair_state.pair_randm;
            pair_state.pair_sk[0..RANDOM_CHALLENGE_SIZE].copy_from_slice(&randm);
            pair_state.pair_sk[RANDOM_CHALLENGE_SIZE..KEY_SIZE].fill(0);

            // Disable encryption during verification
            PAIR_ENC_ENABLE.set(false);

            // Create verification material by XORing network name and password
            for index in 0..KEY_SIZE {
                pair_state.pair_work[index] =
                    pair_state.pair_pass[index] ^ pair_state.pair_nn[index];
            }

            // Verify proof of knowledge based on security mode
            let verification_succeeded = if !SECURITY_ENABLE.get() {
                // Simple mode: Direct comparison of XORed credentials
                pair_state.pair_work == src[RANDOM_CHALLENGE_SIZE..RANDOM_CHALLENGE_SIZE + KEY_SIZE]
            } else {
                // Secure mode: Encrypt and compare with client proof
                const CLIENT_PROOF_OFFSET: usize = 9;
                pair_state.pair_work =
                    aes_att_encryption(&pair_state.pair_sk, &pair_state.pair_work);
                pair_state.pair_work[0..RANDOM_CHALLENGE_SIZE]
                    == pktdata[CLIENT_PROOF_OFFSET..CLIENT_PROOF_OFFSET + RANDOM_CHALLENGE_SIZE]
            };

            // Handle GET_MESH_LTK: Only for already-logged-in users
            if is_get_ltk && PAIR_LOGIN_OK.get() && verification_succeeded {
                // Verification succeeded: transition to RequestingLtk
                BLE_PAIR_ST.set(PairState::RequestingLtk);
                return;
            }
            // GET_MESH_LTK failed: either not logged in or verification failed
            // Fall through to reset pairing

            // Handle VERIFY_CREDENTIALS: Authenticate the user
            if is_verify {
                if verification_succeeded {
                    // Mark as logged in and move to key exchange
                    PAIR_LOGIN_OK.set(true);
                    BLE_PAIR_ST.set(PairState::SessionKeyExchange);
                    return;
                }
                // Verification failed: clear any login status
                PAIR_LOGIN_OK.set(false);
            }

            // Reset pairing on verification failure or if we reach here
            drop(pair_state);
            pair_par_init();
        }

        // --- Default: Unknown or unsupported opcode ---
        _ => {
            // No specific handling for unknown opcodes
        }
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::ble::flash_config::access_code;
    use core::sync::atomic::AtomicUsize;
    use mesh_protocol::packet_types::{
        MeshPkt, PacketAttRawValue, PacketAttReadRsp, PacketL2capData, PacketL2capHead, PacketLlApp,
    };
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::clock::fake_clock;
    use tlsr8266_hal::mcu::crypto::MicMismatch;
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_pac::write_reg_system_tick;

    /// Session key installed by `setup_pair_state`.
    const TEST_SESSION_SK: [u8; 16] = [0xBB; 16];

    /// Reset the PAC/HAL host fakes so a test never inherits another test's
    /// register bank, analog bank, clock or AES transport (the suite runs
    /// `--test-threads=1`, so all of these are process-wide singletons).
    fn reset_host_fakes() {
        FakeMmio::reset();
        tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog::reset();
        fake_clock::reset();
        tlsr8266_hal::mcu::crypto::fake_crypto::reset();
        FakeFlash::reset();
    }

    /// Pairing sector base, taken from the factory default config so the tests
    /// track the shipped layout without importing the crate-root `config`
    /// module.
    const FLASH_ADR_PAIRING: u32 = crate::config::DEFAULT.flash_adr_pairing;

    /// The 64-byte pairing configuration slot at wear-levelling `index`, i.e.
    /// what `save_pair_info` wrote for that slot of the fake NOR device.
    fn pairing_slot(index: i32) -> [u8; 0x40] {
        let mut slot = [0u8; 0x40];
        FakeFlash::read_slice((FLASH_ADR_PAIRING as i32 + index) as u32, &mut slot);
        slot
    }

    /// Last light-event code delivered to the recording handler.
    static RECORDED_LIGHT_EVENT: AtomicUsize = AtomicUsize::new(usize::MAX);

    /// Number of light events delivered to the recording handler.
    static RECORDED_LIGHT_EVENTS: AtomicUsize = AtomicUsize::new(0);

    /// Recording handler for the `emit_light_event` seam.
    fn record_light_event(code: u8) {
        RECORDED_LIGHT_EVENT.store(code as usize, Ordering::SeqCst);
        RECORDED_LIGHT_EVENTS.fetch_add(1, Ordering::SeqCst);
    }

    /// Install the recording light-event handler and clear its counters.
    ///
    /// `LIGHT_EVENT_CB` is a process-wide slot with no "unset" accessor, so every
    /// test that cares installs this recorder on entry; the counters are cleared
    /// here so a test never inherits another test's deliveries.
    fn install_light_event_recorder() {
        RECORDED_LIGHT_EVENT.store(usize::MAX, Ordering::SeqCst);
        RECORDED_LIGHT_EVENTS.store(0, Ordering::SeqCst);
        crate::mesh_ll::adv::set_light_event_handler(record_light_event);
    }

    /// The 48-byte wire image of a `Packet`, for before/after comparisons that
    /// do not depend on which union field the code under test wrote.
    fn packet_bytes(packet: &Packet) -> Vec<u8> {
        // SAFETY: `Packet` is a `#[repr(C, align(4))]` union of wire-format
        // structs; reading it as a byte slice of its own size is the same view
        // the RF driver takes, and the reference is live for the whole call.
        unsafe {
            core::slice::from_raw_parts(
                core::ptr::addr_of!(*packet) as *const u8,
                core::mem::size_of::<Packet>(),
            )
            .to_vec()
        }
    }

    /// Offset of `field` inside `packet`, used to pin the wire layout that the
    /// AES calls index into.
    fn field_offset<T>(packet: &Packet, field: *const T) -> usize {
        field as usize - core::ptr::addr_of!(*packet) as usize
    }

    /// Encrypt the ATT payload of `packet` in place and store the resulting MIC
    /// in `src`, exactly as `pair_enc_packet` would, so `pair_dec_packet`
    /// authenticates it with the real AES-CCM implementation.
    fn seal_pair_packet(packet: &mut Packet) {
        // `pair_dec_packet` copies the sequence number into IVM[5..8] first.
        let mut ivm = [0xAAu8; 8];
        ivm[5..8].copy_from_slice(&packet.att_write().value.sno);

        let data_len = packet.head().l2cap_len as usize - 8;
        let mut mic = [0u8; 2];

        // SAFETY: `dst` starts the encrypted payload and `data_len` is the same
        // `l2cap_len - 8` slice `pair_dec_packet` builds, so the range stays
        // inside the caller's `Packet`; it is written only by AES-CCM.
        unsafe {
            let data = core::slice::from_raw_parts_mut(
                core::ptr::addr_of_mut!(packet.att_write_mut().value.dst) as *mut u8,
                data_len,
            );
            aes_att_encryption_packet(&TEST_SESSION_SK, &ivm, &mut mic, data);
        }
        packet.att_write_mut().value.src = mic;
    }

    /// Decrypt the LL-app payload of `packet` in place with `key`/`ivm` and
    /// report whether the MIC in `value.dst` authenticates.
    fn unseal_ll_packet(
        packet: &mut Packet,
        key: &[u8; 16],
        ivm: &[u8; 8],
    ) -> Result<(), MicMismatch> {
        let data_len = ((packet.head().l2cap_len & 0xff) - 10) as usize;
        let mic = packet.ll_app().value.dst.to_le_bytes();

        // SAFETY: `op` starts the ciphertext and `data_len = l2cap_len - 10` is
        // the length `pair_enc_packet` used, so the range stays inside `Packet`.
        unsafe {
            let data = core::slice::from_raw_parts_mut(
                core::ptr::addr_of_mut!(packet.ll_app_mut().value.op),
                data_len,
            );
            aes_att_decryption_packet(key, ivm, &mic, data)
        }
    }

    #[test]
    fn test_pair_state_from_known_values() {
        assert_eq!(PairState::from(0x0A), PairState::ResetMesh);
        assert_eq!(PairState::from(0xFF), PairState::Idle);
    }

    #[test]
    fn test_atomic_pair_state_get_set() {
        let state = AtomicPairState::new(PairState::Init);
        assert_eq!(state.get(), PairState::Init);

        state.set(PairState::Completed);
        assert_eq!(state.get(), PairState::Completed);
    }

    // Helper function to create a test packet with ATT write structure
    fn create_test_packet() -> Packet {
        let mut packet = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Set up the L2CAP header
        packet.head_mut().l2cap_len = 16;

        // Set up the write value structure with test data
        packet.att_write_mut().value.sno = [0x01, 0x02, 0x03]; // Sequence number
        packet.att_write_mut().value.src = [0x34, 0x12]; // Source address

        packet
    }

    // Helper to set up global state for testing
    fn setup_pair_state() {
        // Set up security enable flag
        SECURITY_ENABLE.set(true);

        // Initialize pair initialization vector
        let mut pair_ivm = PAIR_IVM.lock();
        pair_ivm.fill(0xAA); // Fill with recognizable pattern

        // Initialize session key
        let mut pair_state = PAIR_STATE.lock();
        pair_state.pair_sk.fill(0xBB); // Fill with recognizable pattern
    }

    #[test]
    fn test_pair_dec_packet_security_disabled() {
        // Arrange
        reset_host_fakes();
        SECURITY_ENABLE.set(false);
        let mut packet = create_test_packet();
        let before = packet_bytes(&packet);

        // Act
        let result = pair_dec_packet(&mut packet);

        // Assert
        assert!(result.is_err());
        assert_eq!(
            packet_bytes(&packet),
            before,
            "no crypto may run while security is disabled"
        );
    }

    #[test]
    fn test_pair_dec_packet_security_enabled_success() {
        // Arrange
        reset_host_fakes();
        setup_pair_state();
        let mut packet = create_test_packet();
        // Seal the payload with the session key and the IV pair_dec_packet builds,
        // so the real AES-CCM authentication can succeed.
        seal_pair_packet(&mut packet);

        // Act
        let result = pair_dec_packet(&mut packet);

        // Assert
        assert!(result.is_ok());

        // Verify the IV was updated with sequence number
        let pair_ivm = PAIR_IVM.lock();
        assert_eq!(pair_ivm[5], 0x01);
        assert_eq!(pair_ivm[6], 0x02);
        assert_eq!(pair_ivm[7], 0x03);
    }

    #[test]
    fn test_pair_dec_packet_security_enabled_failure() {
        // Arrange
        reset_host_fakes();
        setup_pair_state();
        let mut packet = create_test_packet();
        seal_pair_packet(&mut packet);
        // A sealed packet with one MIC byte flipped can never authenticate:
        // the compared tag byte differs from the expected byte by construction.
        packet.att_write_mut().value.src[0] ^= 0xff;

        // Act
        let result = pair_dec_packet(&mut packet);

        // Assert
        assert!(result.is_err());
    }

    #[test]
    fn test_pair_dec_packet_l2cap_len_bounds() {
        // `l2cap_len` is peer-controlled and `data_len = l2cap_len - 8`, so the
        // function must reject lengths that underflow it or make the in-place
        // slice leave `PacketAttWrite` (accepted range 9..=33).
        for l2cap_len in [0u16, 8, 34, 255] {
            reset_host_fakes();
            setup_pair_state();
            let mut packet = create_test_packet();
            packet.head_mut().l2cap_len = l2cap_len;
            let before = packet_bytes(&packet);

            assert_eq!(
                pair_dec_packet(&mut packet),
                Err(()),
                "l2cap_len {l2cap_len} must be rejected"
            );
            assert_eq!(
                packet_bytes(&packet),
                before,
                "l2cap_len {l2cap_len}: a rejected packet must not be decrypted"
            );
        }

        // Both boundaries are legitimate: seal them so the real AES-CCM
        // authentication succeeds on the smallest and largest payload.
        for l2cap_len in [9u16, 33] {
            reset_host_fakes();
            setup_pair_state();
            let mut packet = create_test_packet();
            packet.head_mut().l2cap_len = l2cap_len;
            seal_pair_packet(&mut packet);

            assert!(
                pair_dec_packet(&mut packet).is_ok(),
                "l2cap_len {l2cap_len} must be accepted"
            );
        }
    }

    #[test]
    fn test_pair_dec_packet_iv_updates() {
        // Arrange
        reset_host_fakes();
        setup_pair_state();
        let mut packet = create_test_packet();
        packet.att_write_mut().value.sno = [0xF1, 0xF2, 0xF3]; // Different sequence number
        seal_pair_packet(&mut packet);

        // Act
        let result = pair_dec_packet(&mut packet);

        // Assert
        assert!(result.is_ok());

        // Verify IV was updated with new sequence number
        let pair_ivm = PAIR_IVM.lock();
        assert_eq!(pair_ivm[5], 0xF1);
        assert_eq!(pair_ivm[6], 0xF2);
        assert_eq!(pair_ivm[7], 0xF3);
    }

    #[test]
    fn test_pair_dec_packet_parameter_passing() {
        // Arrange
        reset_host_fakes();
        setup_pair_state();
        let mut packet = create_test_packet();

        // The arguments `pair_dec_packet` passes to AES are pinned by sealing the
        // packet with exactly the session key and the IVM the function builds
        // (`PAIR_IVM` with the sequence number copied into [5..8]) and by putting
        // the MIC where it reads it (`value.src`). Any other key, IV or MIC
        // position fails authentication below.
        seal_pair_packet(&mut packet);

        // Act
        let result = pair_dec_packet(&mut packet);

        // Assert
        assert!(result.is_ok());
    }

    // Helper function to create a test packet with LL app structure suitable for encryption
    fn create_test_ll_packet() -> Packet {
        let mut packet = Packet {
            ll_app: PacketLlApp {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: 0,
                    rf_len: 0,
                    l2cap_len: 20,
                    chan_id: 4,
                },
                opcode: 0x1b,
                handle: 0x12,
                handle1: 0,
                value: Default::default(),
                rsv: [0; 10],
            },
        };

        // Set up source as a u16 value (little endian in memory)
        packet.ll_app_mut().value.src = 0x1234;

        packet
    }

    #[test]
    fn test_pair_enc_packet_security_disabled() {
        // Arrange
        reset_host_fakes();
        SECURITY_ENABLE.set(false);
        let mut packet = create_test_ll_packet();
        let before = packet_bytes(&packet);

        // Act
        pair_enc_packet(&mut packet);

        // Assert
        assert_eq!(
            packet_bytes(&packet),
            before,
            "no crypto may run while security is disabled"
        );
    }

    #[test]
    fn test_pair_enc_packet_security_enabled() {
        // Arrange
        reset_host_fakes();
        setup_pair_state();
        let mut packet = create_test_ll_packet();

        // Seed the system-tick register instead of mocking the accessor: off
        // target, `read_reg_system_tick` reads the FakeMmio bank, so the
        // sequence number is derived from this value.
        let test_tick: u32 = 0x123456;
        write_reg_system_tick(test_tick);

        // Act
        pair_enc_packet(&mut packet);

        // Assert
        // Verify sequence number was set from system tick
        assert_eq!(packet.ll_app().value.sno[0], (test_tick & 0xFF) as u8); // 0x56
        assert_eq!(
            packet.ll_app().value.sno[1],
            ((test_tick >> 8) & 0xFF) as u8
        ); // 0x34
        assert_eq!(
            packet.ll_app().value.sno[2],
            ((test_tick >> 16) & 0xFF) as u8
        ); // 0x12

        // The packet was encrypted with the session key and the IV the function
        // published in PAIR_IVS: decrypting it must authenticate.
        let ivm = *PAIR_IVS.lock();
        assert!(
            unseal_ll_packet(&mut packet, &TEST_SESSION_SK, &ivm).is_ok(),
            "the packet must decrypt with the session key and the published IV"
        );
    }

    #[test]
    fn test_pair_enc_packet_iv_updates() {
        // Arrange
        reset_host_fakes();
        setup_pair_state();
        let mut packet = create_test_ll_packet();

        // Seed the system-tick register; the IV must carry this value.
        let test_tick: u32 = 0xABCDEF;
        write_reg_system_tick(test_tick);

        // Fill PAIR_IVS with recognizable pattern for testing
        PAIR_IVS.lock().fill(0xAA);

        // Act
        pair_enc_packet(&mut packet);

        // Assert
        // Verify IV was updated with sequence number
        let pair_ivs = *PAIR_IVS.lock();
        assert_eq!(pair_ivs[3], (test_tick & 0xFF) as u8); // 0xEF
        assert_eq!(pair_ivs[4], ((test_tick >> 8) & 0xFF) as u8); // 0xCD
        assert_eq!(pair_ivs[5], ((test_tick >> 16) & 0xFF) as u8); // 0xAB

        // Verify IV was updated with source address
        assert_eq!(pair_ivs[6], 0x34); // Low byte of 0x1234
        assert_eq!(pair_ivs[7], 0x12); // High byte of 0x1234

        // The IV that was published is the one the payload was encrypted with
        assert!(
            unseal_ll_packet(&mut packet, &TEST_SESSION_SK, &pair_ivs).is_ok(),
            "the payload must decrypt with the IV the function published"
        );
    }

    #[test]
    fn test_pair_enc_packet_parameter_passing() {
        // Arrange
        reset_host_fakes();
        setup_pair_state();
        let mut packet = create_test_ll_packet();

        // Seed the system-tick register; the IV must carry this value.
        let test_tick: u32 = 0x123456;
        write_reg_system_tick(test_tick);

        // Set recognizable session key pattern
        const PARAM_SK: [u8; 16] = [0xCC; 16];
        PAIR_STATE.lock().pair_sk = PARAM_SK;

        // Act
        pair_enc_packet(&mut packet);

        // Assert
        // The IV carries the tick and the packet source address
        let ivm = *PAIR_IVS.lock();
        assert_eq!(ivm[3], 0x56); // Low byte of tick
        assert_eq!(ivm[4], 0x34); // Middle byte of tick
        assert_eq!(ivm[5], 0x12); // High byte of tick
        assert_eq!(ivm[6], 0x34); // Low byte of source
        assert_eq!(ivm[7], 0x12); // High byte of source

        // The MIC went to the 2-byte `value.dst` field and the payload length is
        // `l2cap_len - 10`: decrypting with that exact key, IV and MIC succeeds.
        assert!(
            unseal_ll_packet(&mut packet, &PARAM_SK, &ivm).is_ok(),
            "the packet must decrypt with the session key the state holds"
        );
    }

    #[test]
    fn test_pair_enc_packet_wrong_channel() {
        // Arrange
        reset_host_fakes();
        setup_pair_state();
        let mut packet = create_test_ll_packet();
        packet.head_mut().chan_id = 5; // Different channel than required (4)
        let before = packet_bytes(&packet);

        // Act
        pair_enc_packet(&mut packet);

        // Assert
        assert_eq!(
            packet_bytes(&packet),
            before,
            "a packet that fails the encryption gate must not be touched"
        );
    }

    #[test]
    fn test_pair_enc_packet_wrong_opcode() {
        // Arrange
        reset_host_fakes();
        setup_pair_state();
        let mut packet = create_test_ll_packet();
        packet.ll_app_mut().opcode = 0x1c; // Different opcode than required (0x1b)
        let before = packet_bytes(&packet);

        // Act
        pair_enc_packet(&mut packet);

        // Assert
        assert_eq!(
            packet_bytes(&packet),
            before,
            "a packet that fails the encryption gate must not be touched"
        );
    }

    #[test]
    fn test_pair_enc_packet_wrong_handle() {
        // Arrange
        reset_host_fakes();
        setup_pair_state();
        let mut packet = create_test_ll_packet();
        packet.ll_app_mut().handle = 0x13; // Different handle than required (0x12)
        let before = packet_bytes(&packet);

        // Act
        pair_enc_packet(&mut packet);

        // Assert
        assert_eq!(
            packet_bytes(&packet),
            before,
            "a packet that fails the encryption gate must not be touched"
        );
    }

    #[test]
    fn test_pair_enc_packet_l2cap_len_bounds() {
        // The low byte of `l2cap_len` drives `data_len = (l2cap_len & 0xff) - 10`:
        // below 10 it would underflow, above 33 it would leave `PacketLlApp`.
        // Both must return before the AES call instead of panicking, leaving the
        // MIC and payload bytes untouched (`rf_len` is 0 here, as it is on the
        // real ATT path before the link layer fills it in).
        for l2cap_len in [9u16, 0, 34] {
            reset_host_fakes();
            setup_pair_state();
            write_reg_system_tick(0x123456);
            let mut packet = create_test_ll_packet();
            packet.head_mut().l2cap_len = l2cap_len;
            let before = packet_bytes(&packet);

            pair_enc_packet(&mut packet);

            let after = packet_bytes(&packet);
            assert_eq!(
                &after[18..],
                &before[18..],
                "l2cap_len {l2cap_len}: the MIC and payload must not be written"
            );
        }

        // The legitimate length still encrypts: 20 -> data_len 10.
        reset_host_fakes();
        setup_pair_state();
        write_reg_system_tick(0x123456);
        let mut packet = create_test_ll_packet(); // l2cap_len == 20
        let before = packet_bytes(&packet);

        pair_enc_packet(&mut packet);

        let after = packet_bytes(&packet);
        assert_ne!(
            &after[20..30],
            &before[20..30],
            "l2cap_len 20 must still encrypt its 10-byte payload"
        );
    }

    // Helper function to create a test mesh packet
    fn create_test_mesh_packet(is_broadcast: bool) -> Packet {
        let mut packet = Packet {
            mesh: MeshPkt {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: PACKET_TYPE_ENCRYPTED, // Set encrypted flag
                    rf_len: 30,
                    l2cap_len: 20,
                    chan_id: if is_broadcast {
                        MESH_BROADCAST_CHANNEL
                    } else {
                        0x0001
                    },
                },
                src_tx: 0,
                handle1: 0,
                sno: [0; 3],
                src_adr: 0,
                op: 0,
                vendor_id: 0,
                par: [0; 10],
                internal_par1: [0; 5],
                ttl: 0,
                internal_par2: [0; 4],
                dst_adr: 0,
                no_use: [0; 4],
            },
        };
        packet
    }

    #[test]
    fn test_pair_dec_packet_mesh_security_disabled() {
        // Arrange
        reset_host_fakes();
        SECURITY_ENABLE.set(false);
        let mut packet = create_test_mesh_packet(false);
        let before = packet_bytes(&packet);

        // Act
        let result = pair_dec_packet_mesh(&mut packet);

        // Assert
        assert!(result.is_ok()); // Should return true when security is disabled
        assert_eq!(
            packet_bytes(&packet),
            before,
            "no crypto may run while security is disabled"
        );
    }

    #[test]
    fn test_pair_dec_packet_mesh_no_encryption_flag() {
        // Arrange
        reset_host_fakes();
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(false);
        packet.head_mut()._type = 0; // Clear encryption flag
        let before = packet_bytes(&packet);

        // Act
        let result = pair_dec_packet_mesh(&mut packet);

        // Assert
        assert!(result.is_err()); // Should return false when encryption flag is not set
        assert_eq!(
            packet_bytes(&packet),
            before,
            "a packet without the encryption flag must not be decrypted"
        );
    }

    #[test]
    fn test_pair_dec_packet_mesh_invalid_length() {
        // Arrange
        reset_host_fakes();
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(false);
        packet.head_mut().rf_len = 0x26; // This makes rf_len - 0x12 > 0x13
        let before = packet_bytes(&packet);

        // Act
        let result = pair_dec_packet_mesh(&mut packet);

        // Assert
        assert!(result.is_err()); // Should return false for invalid length
        assert_eq!(
            packet_bytes(&packet),
            before,
            "an over-long packet must not be decrypted"
        );
    }

    #[test]
    fn test_pair_dec_packet_mesh_direct_rf_len_bounds() {
        // Same bound on the decrypt side: a short frame must be rejected by the
        // checked subtraction instead of underflowing, an over-long one by the
        // 0x13 ceiling that protects the `rf_len - 0xb` MIC pointer.
        for rf_len in [0x11u8, 0x26] {
            reset_host_fakes();
            SECURITY_ENABLE.set(true);
            let mut packet = create_test_mesh_packet(false);
            packet.head_mut().rf_len = rf_len;
            let before = packet_bytes(&packet);

            assert_eq!(
                pair_dec_packet_mesh(&mut packet),
                Err(()),
                "rf_len {rf_len:#04x} must be rejected"
            );
            assert_eq!(
                packet_bytes(&packet),
                before,
                "rf_len {rf_len:#04x}: a rejected frame must not be decrypted"
            );
        }
    }

    #[test]
    fn test_pair_dec_packet_mesh_broadcast_success() {
        // Arrange
        reset_host_fakes();
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(true); // Create broadcast packet

        // Set up pairing state with LTK
        PAIR_STATE.lock().pair_ltk.fill(0xDD); // Recognizable pattern

        // Encrypt with that LTK: the LTK, the 8-byte wire IV at `rf_len` and the
        // 0x1c-byte body from `sno` are all pinned by the round trip, because
        // decryption only authenticates if every one of them matches.
        pair_enc_packet_mesh(&mut packet);

        // Act
        let result = pair_dec_packet_mesh(&mut packet);

        // Assert
        assert!(result.is_ok());
    }

    #[test]
    fn test_pair_dec_packet_mesh_broadcast_failure() {
        // Arrange
        reset_host_fakes();
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(true); // Create broadcast packet

        // Set up pairing state with LTK
        PAIR_STATE.lock().pair_ltk.fill(0xDD);

        // Seal the packet, then break the MIC: one flipped MIC byte can never
        // match the computed tag byte.
        pair_enc_packet_mesh(&mut packet);
        packet.mesh_mut().internal_par2[1] ^= 0xff;

        // Act
        let result = pair_dec_packet_mesh(&mut packet);

        // Assert
        assert!(result.is_err());
    }

    #[test]
    fn test_pair_dec_packet_mesh_broadcast_mic_offset() {
        // Verify that broadcast decryption reads the MIC from internal_par2[1] (absolute offset 41).
        // MeshPkt is repr(C, align(4)): vendor_id is u16 so there is 1 byte of alignment padding
        // at offset 21. This makes internal_par2 land at offset 40, and internal_par2[1] at
        // offset 41. The 0x1c-byte data region covers sno(13)..internal_par2[0](40) inclusive.
        reset_host_fakes();
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(true); // broadcast
        PAIR_STATE.lock().pair_ltk.fill(0xDD);

        // Pin the offsets the AES call indexes into.
        assert_eq!(
            field_offset(&packet, core::ptr::addr_of!(packet.mesh().sno)),
            13,
            "the encrypted body must start at sno (offset 13)"
        );
        assert_eq!(
            field_offset(&packet, core::ptr::addr_of!(packet.mesh().internal_par2)),
            40,
            "internal_par2 must sit at offset 40"
        );

        pair_enc_packet_mesh(&mut packet);

        // Corrupting the byte at offset 41 must break authentication, which is
        // what proves that offset is where the MIC is read from.
        let mic_byte = packet.mesh().internal_par2[1];
        packet.mesh_mut().internal_par2[1] ^= 0xff;
        assert!(
            pair_dec_packet_mesh(&mut packet).is_err(),
            "a flipped MIC byte at offset 41 must fail authentication"
        );

        // Restore the MIC and re-seal, so the same packet authenticates.
        packet.mesh_mut().internal_par2[1] = mic_byte;
        pair_enc_packet_mesh(&mut packet);
        assert!(pair_dec_packet_mesh(&mut packet).is_ok());
    }

    /// Integration test: verifies that pair_enc/dec_packet_mesh passes the correct byte
    /// contents to AES when the MeshPkt struct fields are set. This catches regressions
    /// if the struct layout is changed (e.g. padding removed, fields reordered) because
    /// the data slice would no longer contain the expected bytes at the expected positions.
    ///
    /// Broadcast payload layout (0x1c = 28 bytes from sno@13 to internal_par2[0]@40 inclusive):
    ///   data[0..3]   = sno              (offsets 13-15)
    ///   data[3..5]   = src_adr          (offsets 16-17, little-endian)
    ///   data[5..7]   = dst_adr          (offsets 18-19, little-endian)
    ///   data[7]      = op               (offset 20)
    ///   data[8]      = PADDING          (offset 21 — u16 alignment padding, value is undefined)
    ///   data[9]      = vendor_id lo     (offset 22, little-endian lo byte)
    ///   data[10]     = vendor_id hi     (offset 23, little-endian hi byte)
    ///   data[11..21] = par[0..10]       (offsets 24-33)
    ///   data[21..26] = internal_par1    (offsets 34-38)
    ///   data[26]     = ttl              (offset 39)
    ///   data[27]     = internal_par2[0] (offset 40)
    ///   MIC written to internal_par2[1..3] (offsets 41-42)
    ///
    /// REGRESSION SIGNAL: vendor_id is at data[9..11], NOT data[8..10]. If MeshPkt.vendor_id
    /// is changed to [u8; 2] (removing the padding), vendor_id lo shifts to data[8] and the
    /// assertion `data[9] == vendor_id_lo` fails with the wrong byte value. Additionally, the
    /// data length would shrink to 0x1b, catching both issues simultaneously.
    #[test]
    fn test_pair_enc_packet_mesh_broadcast_struct_layout_integration() {
        // The broadcast AES call indexes into the `MeshPkt` wire layout, so the
        // offsets below are the contract this test pins. Previously they were
        // asserted indirectly, through the byte slice handed to a mocked AES;
        // they are asserted directly here, which is stronger, and the encrypted
        // region is then checked against them.
        //
        // Broadcast payload layout (0x1c = 28 bytes from sno@13 to
        // internal_par2[0]@40 inclusive):
        //   data[0..3]   = sno              (offsets 13-15)
        //   data[3..5]   = src_adr          (offsets 16-17, little-endian)
        //   data[5..7]   = dst_adr          (offsets 18-19, little-endian)
        //   data[7]      = op               (offset 20)
        //   data[8]      = PADDING          (offset 21 — u16 alignment padding)
        //   data[9..11]  = vendor_id        (offsets 22-23, little-endian)
        //   data[11..21] = par[0..10]       (offsets 24-33)
        //   data[21..26] = internal_par1    (offsets 34-38)
        //   data[26]     = ttl              (offset 39)
        //   data[27]     = internal_par2[0] (offset 40)
        //   MIC written to internal_par2[1..3] (offsets 41-42)
        //
        // REGRESSION SIGNAL: vendor_id is at offset 22, NOT 21. If `MeshPkt.vendor_id`
        // is changed to `[u8; 2]` (removing the padding), the offset assertion fails.
        reset_host_fakes();
        SECURITY_ENABLE.set(true);
        PAIR_STATE.lock().pair_ltk.fill(0x00);

        let mut packet = Packet {
            mesh: MeshPkt {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: PACKET_TYPE_ENCRYPTED,
                    rf_len: 30,
                    l2cap_len: 20,
                    chan_id: MESH_BROADCAST_CHANNEL,
                },
                src_tx: 0,
                handle1: 0,
                sno: [0x01, 0x02, 0x03],
                src_adr: 0x0504, // little-endian: [0x04, 0x05]
                dst_adr: 0x0706, // little-endian: [0x06, 0x07]
                op: 0x08,
                vendor_id: 0x2211, // lo=0x11 at offset 22, hi=0x22 at offset 23
                par: [0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC],
                internal_par1: [0x15, 0x16, 0x17, 0x18, 0x19],
                ttl: 0x1A,
                internal_par2: [0x1B, 0x00, 0x00, 0x00], // [0] in data, [1..3] overwritten by MIC
                no_use: [0; 4],
            },
        };

        // Pin every offset the AES call slices.
        assert_eq!(
            field_offset(&packet, core::ptr::addr_of!(packet.mesh().sno)),
            13,
            "the encrypted body starts at sno (offset 13)"
        );
        assert_eq!(
            field_offset(&packet, core::ptr::addr_of!(packet.mesh().src_adr)),
            16,
            "src_adr sits at offset 16"
        );
        assert_eq!(
            field_offset(&packet, core::ptr::addr_of!(packet.mesh().dst_adr)),
            18,
            "dst_adr sits at offset 18"
        );
        assert_eq!(
            field_offset(&packet, core::ptr::addr_of!(packet.mesh().op)),
            20,
            "op sits at offset 20"
        );
        assert_eq!(
            field_offset(&packet, core::ptr::addr_of!(packet.mesh().vendor_id)),
            22,
            "vendor_id sits at offset 22, after the u16 alignment padding at 21"
        );
        assert_eq!(
            field_offset(&packet, core::ptr::addr_of!(packet.mesh().par)),
            24,
            "par sits at offset 24"
        );
        assert_eq!(
            field_offset(&packet, core::ptr::addr_of!(packet.mesh().internal_par1)),
            34,
            "internal_par1 sits at offset 34"
        );
        assert_eq!(
            field_offset(&packet, core::ptr::addr_of!(packet.mesh().ttl)),
            39,
            "ttl sits at offset 39"
        );
        assert_eq!(
            field_offset(&packet, core::ptr::addr_of!(packet.mesh().internal_par2)),
            40,
            "internal_par2 sits at offset 40"
        );

        let before = packet_bytes(&packet);

        pair_enc_packet_mesh(&mut packet);

        let after = packet_bytes(&packet);

        // Exactly [13, 41) is encrypted (0x1c bytes, sno through
        // internal_par2[0]) and exactly [41, 43) holds the MIC; everything else
        // is untouched.
        for offset in 0..13 {
            assert_eq!(
                after[offset], before[offset],
                "byte {offset} is outside the encrypted region"
            );
        }
        for offset in 43..after.len() {
            assert_eq!(
                after[offset], before[offset],
                "byte {offset} is outside the encrypted region and the MIC"
            );
        }
        assert_ne!(
            &after[13..41],
            &before[13..41],
            "the 0x1c-byte broadcast body must be encrypted in place"
        );

        // The MIC and body are consistent with the LTK and wire IV.
        assert!(
            pair_dec_packet_mesh(&mut packet).is_ok(),
            "the encrypted broadcast packet must authenticate"
        );
    }

    #[test]
    fn test_pair_dec_packet_mesh_direct_success() {
        // Arrange
        reset_host_fakes();
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(false); // Create direct packet
        packet.head_mut().rf_len = 0x20; // Set suitable RF length for testing

        // Set up pairing state with LTK
        PAIR_STATE.lock().pair_ltk.fill(0xEE);

        // Encrypt with that LTK: the key, the 8-byte IV at `handle1` and the
        // `rf_len - 0x12` byte payload are all pinned by the round trip.
        pair_enc_packet_mesh(&mut packet);

        // Act
        let result = pair_dec_packet_mesh(&mut packet);

        // Assert
        assert!(result.is_ok());
    }

    #[test]
    fn test_pair_dec_packet_mesh_direct_failure() {
        // Arrange
        reset_host_fakes();
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(false); // Create direct packet

        // Set up pairing state with LTK
        PAIR_STATE.lock().pair_ltk.fill(0xEE);

        // Seal the packet, then break one MIC byte: the direct MIC is the 4-byte
        // field at offset `rf_len + 2`.
        pair_enc_packet_mesh(&mut packet);
        let rf_len = packet.head().rf_len as usize;
        let base = core::ptr::addr_of!(packet) as *mut u8;
        // SAFETY: `rf_len + 2` is the first MIC byte the decrypt path reads, and
        // `rf_len <= 0x25` keeps it inside the 48-byte `Packet`.
        unsafe {
            *base.add(rf_len + 2) ^= 0xff;
        }

        // Act
        let result = pair_dec_packet_mesh(&mut packet);

        // Assert
        assert!(result.is_err());
    }

    #[test]
    fn test_pair_enc_packet_mesh_security_disabled() {
        // Arrange
        reset_host_fakes();
        SECURITY_ENABLE.set(false);
        let mut packet = create_test_mesh_packet(false);
        let before = packet_bytes(&packet);

        // Act
        pair_enc_packet_mesh(&mut packet);

        // Assert
        assert_eq!(
            packet_bytes(&packet),
            before,
            "no crypto may run while security is disabled"
        );
    }

    #[test]
    fn test_pair_enc_packet_mesh_broadcast() {
        // Arrange
        reset_host_fakes();
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(true); // Create broadcast packet

        // Set up pairing state with LTK
        PAIR_STATE.lock().pair_ltk.fill(0xDD); // Recognizable pattern

        let before = packet_bytes(&packet);

        // Act
        pair_enc_packet_mesh(&mut packet);

        // Assert
        assert_ne!(
            &packet_bytes(&packet)[13..41],
            &before[13..41],
            "the broadcast body must be encrypted"
        );
    }

    #[test]
    fn test_pair_enc_packet_mesh_broadcast_parameter_passing() {
        // Arrange
        reset_host_fakes();
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(true); // Create broadcast packet

        // Set up pairing state with LTK
        PAIR_STATE.lock().pair_ltk.fill(0xDD); // Recognizable pattern

        // Act
        pair_enc_packet_mesh(&mut packet);

        // Assert
        // The LTK, the 8-byte IV at `rf_len`, the 2-byte MIC at
        // `internal_par2[1]` and the 0x1c-byte body are all pinned by the round
        // trip: a different key, IV, MIC length or body length fails to
        // authenticate.
        assert!(
            pair_dec_packet_mesh(&mut packet).is_ok(),
            "the broadcast packet must decrypt with the LTK the state holds"
        );
    }

    #[test]
    fn test_pair_enc_packet_mesh_direct() {
        // Arrange
        reset_host_fakes();
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(false); // Create direct packet
        packet.head_mut().rf_len = 0x20; // Set specific RF length for testing

        // Set up pairing state with LTK
        PAIR_STATE.lock().pair_ltk.fill(0xEE); // Different recognizable pattern

        let before = packet_bytes(&packet);

        // Act
        pair_enc_packet_mesh(&mut packet);

        // Assert
        // The direct payload is `rf_len - 0x12` bytes from `op` (offset 20).
        assert_ne!(
            &packet_bytes(&packet)[20..20 + (0x20 - 0x12)],
            &before[20..20 + (0x20 - 0x12)],
            "the direct payload must be encrypted"
        );
    }

    #[test]
    fn test_pair_enc_packet_mesh_direct_parameter_passing() {
        // Arrange
        reset_host_fakes();
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(false); // Create direct packet
        packet.head_mut().rf_len = 0x22; // Set specific RF length for testing

        // Set up pairing state with LTK
        PAIR_STATE.lock().pair_ltk.fill(0xEE); // Different recognizable pattern

        // Act
        pair_enc_packet_mesh(&mut packet);

        // Assert
        // The LTK, the 8-byte IV at `handle1`, the 4-byte MIC at `rf_len + 2` and
        // the `rf_len - 0x12` byte payload are all pinned by the round trip.
        assert!(
            pair_dec_packet_mesh(&mut packet).is_ok(),
            "the direct packet must decrypt with the LTK the state holds"
        );
    }

    #[test]
    fn test_pair_enc_packet_mesh_direct_rf_len_bounds() {
        // Direct branch only: `payload_len = rf_len - 0x12` and the MIC pointer
        // sits at `sno + (rf_len - 0xb)`, so a short or over-long `rf_len` must
        // return before any pointer arithmetic or AES call.
        for rf_len in [0x11u8, 0x26] {
            reset_host_fakes();
            SECURITY_ENABLE.set(true);
            PAIR_STATE.lock().pair_ltk.fill(0xEE);
            let mut packet = create_test_mesh_packet(false);
            packet.head_mut().rf_len = rf_len;
            let before = packet_bytes(&packet);

            pair_enc_packet_mesh(&mut packet);

            assert_eq!(
                packet_bytes(&packet),
                before,
                "rf_len {rf_len:#04x}: an out-of-range direct frame must not be encrypted"
            );
        }

        // 0x1e is a legitimate direct frame (payload_len 0x0c) and still encrypts.
        reset_host_fakes();
        SECURITY_ENABLE.set(true);
        PAIR_STATE.lock().pair_ltk.fill(0xEE);
        let mut packet = create_test_mesh_packet(false); // rf_len == 0x1e
        let before = packet_bytes(&packet);

        pair_enc_packet_mesh(&mut packet);

        let after = packet_bytes(&packet);
        assert_ne!(
            &after[20..20 + (0x1e - 0x12)],
            &before[20..20 + (0x1e - 0x12)],
            "a 0x1e direct frame must still be encrypted"
        );
    }

    #[test]
    fn test_pair_enc_packet_mesh_varying_rf_lengths() {
        // Test with different RF lengths to verify calculation of payload size.
        //
        // The payload length is observable through the encrypted region's
        // boundaries: `rf_len - 0x12` bytes from `op` (offset 20) are encrypted
        // and the 4-byte MIC lands at `rf_len + 2`. Everything else — including
        // the bytes between the payload end and the MIC — must be untouched.
        let test_cases = [(0x15u8, 0x03usize), (0x18, 0x06), (0x1F, 0x0D)];

        for (rf_len, expected_size) in test_cases {
            reset_host_fakes();
            SECURITY_ENABLE.set(true);
            PAIR_STATE.lock().pair_ltk.fill(0xEE);

            let mut packet = create_test_mesh_packet(false); // Create direct packet
            packet.head_mut().rf_len = rf_len; // Set specific RF length for testing

            let before = packet_bytes(&packet);

            pair_enc_packet_mesh(&mut packet);

            let after = packet_bytes(&packet);
            let payload_start = 20usize;
            let payload_end = payload_start + expected_size;
            let mic_start = rf_len as usize + 2;
            let mic_end = mic_start + 4;

            for offset in 0..after.len() {
                if (payload_start..payload_end).contains(&offset)
                    || (mic_start..mic_end).contains(&offset)
                {
                    continue;
                }
                assert_eq!(
                    after[offset], before[offset],
                    "rf_len {rf_len:#04x}: byte {offset} is outside the payload and MIC"
                );
            }

            assert!(
                pair_dec_packet_mesh(&mut packet).is_ok(),
                "rf_len {rf_len:#04x}: the encrypted packet must authenticate"
            );
        }
    }

    // Helper to set up global state for testing pair_flash_save_config
    fn setup_flash_config_state(initial_idx: i32) {
        FLASH_CONFIGURATION_INDEX.set(initial_idx);
    }

    #[test]
    fn test_pair_flash_save_config_new_set() {
        // Arrange
        const OFFSET_HEADER: u32 = 0x00;
        const SECTOR_SIZE: i32 = 0x40;
        reset_host_fakes();
        let initial_idx = 100; // Arbitrary initial index
        setup_flash_config_state(initial_idx);
        let test_addr = OFFSET_HEADER;
        let test_data = [1, 2, 3, 4];

        // Act
        pair_flash_save_config(test_addr, &test_data);

        // Assert
        // The header write advanced the wear-levelling index by one slot (and
        // `pair_flash_clean` left it alone, being far from the wrap point)...
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), initial_idx + SECTOR_SIZE);

        // ...and `save_pair_info` put the bytes in that slot of the NOR device.
        let slot = pairing_slot(initial_idx + SECTOR_SIZE);
        assert_eq!(
            &slot[test_addr as usize..test_addr as usize + test_data.len()],
            &test_data
        );
    }

    #[test]
    fn test_pair_flash_save_config_existing_set() {
        // Arrange
        const OFFSET_NAME: u32 = 0x10; // Use a non-header offset
        reset_host_fakes();
        let initial_idx = 200; // Arbitrary initial index
        setup_flash_config_state(initial_idx);
        let test_addr = OFFSET_NAME;
        let test_data = [5, 6, 7, 8];

        // Act
        pair_flash_save_config(test_addr, &test_data);

        // Assert
        // A non-header write neither rotates the slot nor rewrites the header...
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), initial_idx);

        // ...it only lands the bytes in the current slot.
        let slot = pairing_slot(initial_idx);
        assert_eq!(
            &slot[test_addr as usize..test_addr as usize + test_data.len()],
            &test_data
        );
    }

    #[test]
    fn test_pair_flash_save_config_data_passing() {
        // Arrange
        const OFFSET_PASSWORD: u32 = 0x20;
        reset_host_fakes();
        setup_flash_config_state(0); // Index doesn't matter here
        let test_addr = OFFSET_PASSWORD;
        let test_data = [0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6];

        // Act
        pair_flash_save_config(test_addr, &test_data);

        // Assert
        // The offset and the payload reach the NOR device unchanged.
        let slot = pairing_slot(0);
        assert_eq!(
            &slot[test_addr as usize..test_addr as usize + test_data.len()],
            &test_data
        );
    }

    // Helper to set up global state for testing pair_save_key
    fn setup_save_key_state(
        mesh_pair_enable: bool,
        device_addr_validation_pending: bool,
        nn: [u8; 16],
        pass: [u8; 16],
        ltk: [u8; 16],
    ) {
        MESH_PAIR_ENABLE.set(mesh_pair_enable);
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(device_addr_validation_pending);
        let mut pair_state = PAIR_STATE.lock();
        pair_state.pair_nn = nn;
        pair_state.pair_pass = pass;
        pair_state.pair_ltk = ltk;
    }

    #[test]
    fn test_pair_save_key_mesh_enabled_mac_enabled() {
        // Arrange
        const OFFSET_HEADER: u32 = 0x00;
        const OFFSET_NAME: u32 = 0x10;
        const OFFSET_PASSWORD: u32 = 0x20;
        const OFFSET_LTK: u32 = 0x30;
        let test_nn = [0x11; 16];
        let test_pass = [0x22; 16];
        let test_ltk = [0x33; 16];

        reset_host_fakes();
        FLASH_CONFIGURATION_INDEX.set(-1); // No configuration stored yet
        setup_save_key_state(true, true, test_nn, test_pass, test_ltk);

        // The password is stored encoded; the real AES-CCM block cipher with the
        // configured key produces the bytes that must reach flash.
        let encoded_pass = aes_att_encryption(&PAIR_CONFIG_PWD_ENCODE_SK.lock(), &test_pass);

        // Act
        pair_save_key();

        // Assert
        // The first save starts a fresh slot at offset 0 and writes the header,
        // name, encoded password and LTK into it.
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), 0);
        let slot = pairing_slot(0);

        let mut expected_header = [0u8; 16];
        expected_header[0] = PAIR_CONFIG_VALID_FLAG;
        expected_header[15] = PAIR_CONFIG_VALID_FLAG;
        expected_header[1] = 1; // MESH_PAIR_ENABLE=true, MESH_DEVICE_ADDRESS_VALIDATION_PENDING=true
        assert_eq!(&slot[OFFSET_HEADER as usize..][..16], &expected_header);
        assert_eq!(&slot[OFFSET_NAME as usize..][..16], &test_nn);
        assert_eq!(&slot[OFFSET_PASSWORD as usize..][..16], &encoded_pass);
        assert_eq!(&slot[OFFSET_LTK as usize..][..16], &test_ltk);

        // The save also refreshed the runtime keys: the access code is the real
        // derivation over the credentials just written.
        assert_eq!(PAIR_AC.get(), access_code(&test_nn, &test_pass));
    }

    #[test]
    fn test_pair_save_key_mesh_enabled_mac_disabled() {
        // Arrange
        const OFFSET_HEADER: u32 = 0x00;
        const OFFSET_NAME: u32 = 0x10;
        const OFFSET_PASSWORD: u32 = 0x20;
        const OFFSET_LTK: u32 = 0x30;
        let test_nn = [0xAA; 16];
        let test_pass = [0xBB; 16];
        let test_ltk = [0xCC; 16];

        reset_host_fakes();
        FLASH_CONFIGURATION_INDEX.set(-1); // No configuration stored yet
        setup_save_key_state(true, false, test_nn, test_pass, test_ltk);

        // Real AES with the configured encode key produces the stored bytes
        let encoded_pass = aes_att_encryption(&PAIR_CONFIG_PWD_ENCODE_SK.lock(), &test_pass);

        // Act
        pair_save_key();

        // Assert
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), 0);
        let slot = pairing_slot(0);

        let mut expected_header = [0u8; 16];
        expected_header[0] = PAIR_CONFIG_VALID_FLAG;
        expected_header[15] = PAIR_CONFIG_VALID_FLAG;
        expected_header[1] = 0; // MESH_PAIR_ENABLE=true, MESH_DEVICE_ADDRESS_VALIDATION_PENDING=false
        assert_eq!(&slot[OFFSET_HEADER as usize..][..16], &expected_header);
        assert_eq!(&slot[OFFSET_NAME as usize..][..16], &test_nn);
        assert_eq!(&slot[OFFSET_PASSWORD as usize..][..16], &encoded_pass);
        assert_eq!(&slot[OFFSET_LTK as usize..][..16], &test_ltk);

        assert_eq!(PAIR_AC.get(), access_code(&test_nn, &test_pass));
    }

    #[test]
    fn test_pair_save_key_mesh_disabled() {
        // Arrange
        const OFFSET_HEADER: u32 = 0x00;
        const OFFSET_NAME: u32 = 0x10;
        const OFFSET_PASSWORD: u32 = 0x20;
        const OFFSET_LTK: u32 = 0x30;
        let test_nn = [0x1A; 16];
        let test_pass = [0x2B; 16];
        let test_ltk = [0x3C; 16];

        reset_host_fakes();
        FLASH_CONFIGURATION_INDEX.set(-1); // No configuration stored yet
        setup_save_key_state(false, true, test_nn, test_pass, test_ltk); // MESH_DEVICE_ADDRESS_VALIDATION_PENDING doesn't matter if mesh is disabled

        // Real AES with the configured encode key produces the stored bytes
        let encoded_pass = aes_att_encryption(&PAIR_CONFIG_PWD_ENCODE_SK.lock(), &test_pass);

        // Act
        pair_save_key();

        // Assert
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), 0);
        let slot = pairing_slot(0);

        let mut expected_header = [0u8; 16];
        expected_header[0] = PAIR_CONFIG_VALID_FLAG;
        expected_header[15] = PAIR_CONFIG_VALID_FLAG;
        // expected_header[1] remains 0 as MESH_PAIR_ENABLE is false
        assert_eq!(&slot[OFFSET_HEADER as usize..][..16], &expected_header);
        assert_eq!(&slot[OFFSET_NAME as usize..][..16], &test_nn);
        assert_eq!(&slot[OFFSET_PASSWORD as usize..][..16], &encoded_pass);
        assert_eq!(&slot[OFFSET_LTK as usize..][..16], &test_ltk);

        assert_eq!(PAIR_AC.get(), access_code(&test_nn, &test_pass));
    }

    #[test]
    fn test_pair_init() {
        // Arrange
        // Set initial non-default states
        BLE_PAIR_ST.set(PairState::Completed);
        PAIR_ENC_ENABLE.set(true);
        MAC_ID
            .lock()
            .copy_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
        PAIR_IVM.lock().fill(0xFF);
        PAIR_IVS.lock().fill(0xFF);
        const MAC_ID_SIZE: usize = 4;

        // Act
        pair_init();

        // Assert
        // Check state reset
        assert_eq!(BLE_PAIR_ST.get(), PairState::Idle);
        assert_eq!(PAIR_ENC_ENABLE.get(), false);

        // Check IV initialization from MAC ID
        let mac_id = MAC_ID.lock();
        let pair_ivm = PAIR_IVM.lock();
        let pair_ivs = PAIR_IVS.lock();
        assert_eq!(pair_ivm[0..MAC_ID_SIZE], mac_id[0..MAC_ID_SIZE]);
        assert_eq!(pair_ivs[0..MAC_ID_SIZE], mac_id[0..MAC_ID_SIZE]);

        // Check that the rest of the IVs are untouched (or verify if they should be zeroed)
        // Assuming they should remain untouched based on the code:
        // Assuming IVs are 8 bytes based on usage elsewhere and panic message
        const IV_SIZE: usize = 8;
        assert_eq!(
            pair_ivm[MAC_ID_SIZE..IV_SIZE],
            [0xFF; IV_SIZE - MAC_ID_SIZE]
        );
        assert_eq!(
            pair_ivs[MAC_ID_SIZE..IV_SIZE],
            [0xFF; IV_SIZE - MAC_ID_SIZE]
        );
    }

    #[test]
    fn test_pair_par_init() {
        // Arrange
        reset_host_fakes();
        // Set initial state to something other than Init
        BLE_PAIR_ST.set(PairState::Idle);
        // A known default credential set: with a blank pairing sector the real
        // `pair_load_key` copies it into the runtime state, so seeing it there
        // proves the load ran.
        PAIR_CONFIG_MESH_NAME.lock().fill(0);
        PAIR_CONFIG_MESH_NAME.lock()[0..11].copy_from_slice(b"out_of_mesh");

        // Act
        pair_par_init();

        // Assert
        // Check state is set to Init
        assert_eq!(BLE_PAIR_ST.get(), PairState::Init);
        // The empty sector made `pair_load_key` publish the defaults.
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), -1);
        assert_eq!(&PAIR_STATE.lock().pair_nn[0..11], b"out_of_mesh");
    }

    // Helper to set up global state for pair_proc tests
    fn setup_proc_state(
        pair_st: PairState,
        read_pending: bool,
        security_enable: bool,
        login_ok: bool,
        mesh_pair_enable: bool,
        device_addr_validation_pending: bool,
    ) {
        BLE_PAIR_ST.set(pair_st);
        PAIR_READ_PENDING.set(read_pending);
        SECURITY_ENABLE.set(security_enable);
        PAIR_LOGIN_OK.set(login_ok);
        MESH_PAIR_ENABLE.set(mesh_pair_enable);
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(device_addr_validation_pending);

        // Reset relevant parts of PAIR_STATE if needed for specific tests
        let mut pair_state = PAIR_STATE.lock();
        pair_state.pair_rands.fill(0xAA); // Default random slave
        pair_state.pair_randm.fill(0xBB); // Default random master
        pair_state.pair_nn.fill(0x11); // Default network name
        pair_state.pair_pass.fill(0x22); // Default password
        pair_state.pair_ltk.fill(0x33); // Default LTK
        pair_state.pair_sk.fill(0x44); // Default session key
        pair_state.pair_sk_copy.fill(0x55); // Default session key copy
        pair_state.pair_work.fill(0x00); // Clear work buffer
    }

    // --- pair_proc Tests ---

    #[test]
    fn test_pair_proc_no_read_pending() {
        // Arrange
        setup_proc_state(PairState::AwaitingRandom, false, true, true, false, false);

        // Act
        let result = pair_proc();

        // Assert
        assert!(result.is_none());
    }

    #[test]
    fn test_pair_proc_awaiting_random_reset_needed() {
        // Arrange: Security disabled AND not logged in
        reset_host_fakes();
        setup_proc_state(PairState::AwaitingRandom, true, false, false, false, false);

        // Act
        let result = pair_proc();

        // Assert
        assert!(result.is_none()); // Should return None after reset
        assert_eq!(PAIR_READ_PENDING.get(), false); // Flag should be cleared
        assert_eq!(PAIR_ENC_ENABLE.get(), false); // Encryption should be disabled

        // The reset really ran `pair_par_init`: the state machine moved from
        // AwaitingRandom to the Init state it publishes.
        assert_eq!(BLE_PAIR_ST.get(), PairState::Init);
    }

    #[test]
    fn test_pair_proc_awaiting_random_send_challenge() {
        // Arrange: Security enabled OR logged in
        setup_proc_state(PairState::AwaitingRandom, true, true, false, false, false); // Security enabled
                                                                                      // setup_proc_state(PairState::AwaitingRandom, true, false, true, false, false); // Logged in

        let initial_rands = PAIR_STATE.lock().pair_rands;

        // Act
        let result = pair_proc();

        // Assert
        assert!(result.is_some());
        let pkt = result.unwrap();

        // Check packet header
        let l2cap_len = pkt.head().l2cap_len;
        assert_eq!(l2cap_len, RESP_WITH_RANDOM);
        assert_eq!(pkt.head().rf_len, RESP_WITH_RANDOM as u8 + 4);
        let dma_len = pkt.head().dma_len;
        assert_eq!(dma_len, RESP_WITH_RANDOM as u32 + 6);
        assert_eq!(pkt.att_read_rsp().opcode, ATT_READ_RESPONSE_OPCODE);

        // Check packet value
        assert_eq!(pkt.att_read_rsp().value[0], PairState::AwaitingRandom as u8);
        assert_eq!(
            pkt.att_read_rsp().value[1..1 + RANDOM_CHALLENGE_SIZE],
            initial_rands
        );

        // Check state transition
        assert_eq!(BLE_PAIR_ST.get(), PairState::RandomConfirmation);
        assert_eq!(PAIR_READ_PENDING.get(), false); // Flag should be cleared
    }

    #[test]
    fn test_pair_proc_receiving_mesh_ltk_reset_needed() {
        // Arrange: Security disabled AND not logged in
        reset_host_fakes();
        setup_proc_state(
            PairState::ReceivingMeshLtk,
            true,
            false,
            false,
            false,
            false,
        );

        // Act
        let result = pair_proc();

        // Assert
        assert!(result.is_none()); // Should return None after reset
        assert_eq!(PAIR_READ_PENDING.get(), false);
        assert_eq!(PAIR_ENC_ENABLE.get(), false);
        // `pair_par_init` ran: ReceivingMeshLtk became Init.
        assert_eq!(BLE_PAIR_ST.get(), PairState::Init);
    }

    #[test]
    fn test_pair_proc_receiving_mesh_ltk_plaintext() {
        // Arrange: Security disabled AND logged in
        setup_proc_state(PairState::ReceivingMeshLtk, true, false, true, false, false);

        let (initial_nn, initial_pass, initial_ltk) = {
            let state = PAIR_STATE.lock();
            (state.pair_nn, state.pair_pass, state.pair_ltk)
        };
        let mut expected_work = [0u8; KEY_SIZE];
        for i in 0..KEY_SIZE {
            expected_work[i] = initial_nn[i] ^ initial_pass[i] ^ initial_ltk[i];
        }

        // Act
        let result = pair_proc();

        // Assert
        assert!(result.is_some());
        let pkt = result.unwrap();

        // Check packet header
        let l2cap_len = pkt.head().l2cap_len;
        assert_eq!(l2cap_len, RESP_WITH_KEY);
        assert_eq!(pkt.head().rf_len, RESP_WITH_KEY as u8 + 4);
        let dma_len = pkt.head().dma_len;
        assert_eq!(dma_len, RESP_WITH_KEY as u32 + 6);
        assert_eq!(pkt.att_read_rsp().opcode, ATT_READ_RESPONSE_OPCODE);

        // Check packet value (plaintext XORed keys)
        assert_eq!(
            pkt.att_read_rsp().value[0],
            PairState::ReceivingMeshLtk as u8
        );
        assert_eq!(pkt.att_read_rsp().value[1..1 + 10], expected_work[0..10]); // Only first 10 bytes sent

        // Check state transition
        assert_eq!(BLE_PAIR_ST.get(), PairState::Completed);
        assert_eq!(PAIR_READ_PENDING.get(), false);
    }

    #[test]
    fn test_pair_proc_receiving_mesh_ltk_encrypted() {
        // Arrange: Security enabled
        setup_proc_state(PairState::ReceivingMeshLtk, true, true, true, false, false); // Login status doesn't matter here

        let (initial_nn, initial_pass, initial_ltk, initial_sk) = {
            let state = PAIR_STATE.lock();
            (
                state.pair_nn,
                state.pair_pass,
                state.pair_ltk,
                state.pair_sk,
            )
        };
        let mut expected_work = [0u8; KEY_SIZE];
        for i in 0..KEY_SIZE {
            expected_work[i] = initial_nn[i] ^ initial_pass[i] ^ initial_ltk[i];
        }
        // The proof the device must send is the real AES-CCM block cipher over
        // the XORed credentials, keyed by the session key. Computing it here
        // with the same inputs pins which key and which plaintext production
        // uses: any other pair yields different bytes below.
        let encrypted_work = aes_att_encryption(&initial_sk, &expected_work);

        // Act
        let result = pair_proc();

        // Assert
        assert!(result.is_some());
        let pkt = result.unwrap();

        // Check packet header
        let l2cap_len = pkt.head().l2cap_len;
        assert_eq!(l2cap_len, RESP_WITH_KEY);
        assert_eq!(pkt.head().rf_len, RESP_WITH_KEY as u8 + 4);
        let dma_len = pkt.head().dma_len;
        assert_eq!(dma_len, RESP_WITH_KEY as u32 + 6);
        assert_eq!(pkt.att_read_rsp().opcode, ATT_READ_RESPONSE_OPCODE);

        // Check packet value (encrypted XORed keys)
        assert_eq!(
            pkt.att_read_rsp().value[0],
            PairState::ReceivingMeshLtk as u8
        );
        assert_eq!(pkt.att_read_rsp().value[1..1 + KEY_SIZE], encrypted_work);

        // Check state transition
        assert_eq!(BLE_PAIR_ST.get(), PairState::Completed);
        assert_eq!(PAIR_READ_PENDING.get(), false);
    }

    #[test]
    fn test_pair_proc_session_key_exchange_plaintext() {
        // Arrange: Security disabled
        setup_proc_state(
            PairState::SessionKeyExchange,
            true,
            false,
            true,
            false,
            false,
        ); // Must be logged in

        let (initial_nn, initial_pass) = {
            let state = PAIR_STATE.lock();
            (state.pair_nn, state.pair_pass)
        };
        let mut expected_work = [0u8; KEY_SIZE];
        for i in 0..KEY_SIZE {
            expected_work[i] = initial_nn[i] ^ initial_pass[i];
        }

        // Act
        let result = pair_proc();

        // Assert
        assert!(result.is_some());
        let pkt = result.unwrap();

        // Check packet header
        let l2cap_len = pkt.head().l2cap_len;
        assert_eq!(l2cap_len, RESP_WITH_KEY);
        assert_eq!(pkt.head().rf_len, RESP_WITH_KEY as u8 + 4);
        let dma_len = pkt.head().dma_len;
        assert_eq!(dma_len, RESP_WITH_KEY as u32 + 6);
        assert_eq!(pkt.att_read_rsp().opcode, ATT_READ_RESPONSE_OPCODE);

        // Check packet value (plaintext XORed name/pass)
        assert_eq!(
            pkt.att_read_rsp().value[0],
            PairState::SessionKeyExchange as u8
        );
        assert_eq!(pkt.att_read_rsp().value[1..1 + KEY_SIZE], expected_work);

        // Check state transition and flags
        assert_eq!(BLE_PAIR_ST.get(), PairState::Completed);
        assert_eq!(PAIR_ENC_ENABLE.get(), false);
        assert_eq!(PAIR_READ_PENDING.get(), false);
    }

    #[test]
    fn test_pair_proc_session_key_exchange_secure() {
        // Arrange: Security enabled
        reset_host_fakes();
        setup_proc_state(
            PairState::SessionKeyExchange,
            true,
            true,
            true,
            false,
            false,
        ); // Must be logged in

        // Seed the system-tick register; the derived rands below use it.
        let test_tick: u32 = 0xABCDEF01;
        write_reg_system_tick(test_tick);

        let (initial_nn, initial_pass, initial_randm) = {
            let state = PAIR_STATE.lock();
            (state.pair_nn, state.pair_pass, state.pair_randm)
        };

        // The three chained AES calls are evaluated here with the real block
        // cipher, using the inputs the state machine is expected to pass. Any
        // deviation in that chain changes the bytes below.
        // 1. First AES: Generate intermediate SK and new rands
        let mut key_padded = [0u8; KEY_SIZE];
        key_padded[0..RANDOM_CHALLENGE_SIZE].copy_from_slice(&initial_randm);
        let mut source_padded = [0u8; KEY_SIZE];
        // rands is generated inside the function using tick
        source_padded[0..4].copy_from_slice(&test_tick.to_le_bytes());
        source_padded[4..RANDOM_CHALLENGE_SIZE].fill(0xAA);
        let sk_intermediate = aes_att_encryption(&key_padded, &source_padded);

        // 2. Second AES: Generate proof
        let mut expected_work_proof = [0u8; KEY_SIZE];
        for i in 0..KEY_SIZE {
            expected_work_proof[i] = initial_nn[i] ^ initial_pass[i];
        }
        let mut expected_sk_for_proof = sk_intermediate;
        expected_sk_for_proof[RANDOM_CHALLENGE_SIZE..KEY_SIZE].fill(0); // Second half zeroed
        let encrypted_proof = aes_att_encryption(&expected_sk_for_proof, &expected_work_proof);

        // 3. Third AES: Generate final SK
        let mut expected_work_final = [0u8; KEY_SIZE];
        for i in 0..KEY_SIZE {
            expected_work_final[i] = initial_nn[i] ^ initial_pass[i];
        }
        let mut expected_sk_input_final = [0u8; KEY_SIZE];
        expected_sk_input_final[0..RANDOM_CHALLENGE_SIZE].copy_from_slice(&initial_randm);
        // Use the derived rands (first half of sk_intermediate)
        expected_sk_input_final[RANDOM_CHALLENGE_SIZE..KEY_SIZE]
            .copy_from_slice(&sk_intermediate[0..RANDOM_CHALLENGE_SIZE]);
        let sk_final = aes_att_encryption(&expected_work_final, &expected_sk_input_final);

        // Act
        let result = pair_proc();

        // Assert
        assert!(result.is_some());
        let pkt = result.unwrap();

        // Check packet header
        let l2cap_len = pkt.head().l2cap_len;
        assert_eq!(l2cap_len, RESP_WITH_KEY); // Uses RESP_WITH_KEY size
        assert_eq!(pkt.head().rf_len, RESP_WITH_KEY as u8 + 4);
        let dma_len = pkt.head().dma_len;
        assert_eq!(dma_len, RESP_WITH_KEY as u32 + 6);
        assert_eq!(pkt.att_read_rsp().opcode, ATT_READ_RESPONSE_OPCODE);

        // Check packet value (new rands + encrypted proof)
        let expected_rands_new = &sk_intermediate[0..RANDOM_CHALLENGE_SIZE];
        let expected_proof_part = &encrypted_proof[0..RANDOM_CHALLENGE_SIZE];
        assert_eq!(
            pkt.att_read_rsp().value[0],
            PairState::SessionKeyExchange as u8
        );
        assert_eq!(
            pkt.att_read_rsp().value[1..1 + RANDOM_CHALLENGE_SIZE],
            *expected_rands_new
        );
        assert_eq!(
            pkt.att_read_rsp().value[9..9 + RANDOM_CHALLENGE_SIZE],
            *expected_proof_part
        );

        // Check state transition and flags
        assert_eq!(BLE_PAIR_ST.get(), PairState::Completed);
        assert_eq!(PAIR_ENC_ENABLE.get(), true); // Encryption should be enabled
        assert_eq!(PAIR_READ_PENDING.get(), false);

        // Verify final session key was stored
        assert_eq!(PAIR_STATE.lock().pair_sk, sk_final);
    }

    #[test]
    fn test_pair_proc_requesting_ltk_plaintext() {
        // Arrange: Security disabled
        setup_proc_state(PairState::RequestingLtk, true, false, true, false, false); // Must be logged in

        let initial_ltk = PAIR_STATE.lock().pair_ltk;

        // Act
        let result = pair_proc();

        // Assert
        assert!(result.is_some());
        let pkt = result.unwrap();

        // Check packet header
        let l2cap_len = pkt.head().l2cap_len;
        assert_eq!(l2cap_len, RESP_WITH_KEY);
        assert_eq!(pkt.head().rf_len, RESP_WITH_KEY as u8 + 4);
        let dma_len = pkt.head().dma_len;
        assert_eq!(dma_len, RESP_WITH_KEY as u32 + 6);
        assert_eq!(pkt.att_read_rsp().opcode, ATT_READ_RESPONSE_OPCODE);

        // Check packet value (plaintext LTK)
        assert_eq!(pkt.att_read_rsp().value[0], PairState::RequestingLtk as u8);
        assert_eq!(pkt.att_read_rsp().value[1..1 + KEY_SIZE], initial_ltk);

        // Check state transition (remains Completed in simple mode?) - Code doesn't change state here for simple mode
        // assert_eq!(BLE_PAIR_ST.get(), PairState::Completed); // Let's assume it should go to completed based on secure mode logic
        assert_eq!(PAIR_READ_PENDING.get(), false);
    }

    #[test]
    fn test_pair_proc_requesting_ltk_secure() {
        // Arrange: Security enabled
        setup_proc_state(PairState::RequestingLtk, true, true, true, false, false); // Must be logged in

        let (initial_nn, initial_pass, initial_ltk, initial_randm, initial_sk_copy) = {
            let state = PAIR_STATE.lock();
            (
                state.pair_nn,
                state.pair_pass,
                state.pair_ltk,
                state.pair_randm,
                state.pair_sk_copy,
            )
        };

        // Calculate expected key for encryption
        let mut expected_work = [0u8; KEY_SIZE];
        expected_work[0..RANDOM_CHALLENGE_SIZE].copy_from_slice(&initial_randm);
        let mut expected_sk_for_ltk = [0u8; KEY_SIZE];
        for i in 0..KEY_SIZE {
            expected_sk_for_ltk[i] = initial_nn[i] ^ initial_pass[i] ^ expected_work[i];
        }

        // The LTK is sent encrypted under the derived session key; evaluating the
        // same AES call here pins the key and plaintext production uses.
        let encrypted_ltk = aes_att_encryption(&expected_sk_for_ltk, &initial_ltk);

        // Act
        let result = pair_proc();

        // Assert
        assert!(result.is_some());
        let pkt = result.unwrap();

        // Check packet header
        let l2cap_len = pkt.head().l2cap_len;
        assert_eq!(l2cap_len, RESP_WITH_KEY);
        assert_eq!(pkt.head().rf_len, RESP_WITH_KEY as u8 + 4);
        let dma_len = pkt.head().dma_len;
        assert_eq!(dma_len, RESP_WITH_KEY as u32 + 6);
        assert_eq!(pkt.att_read_rsp().opcode, ATT_READ_RESPONSE_OPCODE);

        // Check packet value (encrypted LTK)
        assert_eq!(pkt.att_read_rsp().value[0], PairState::RequestingLtk as u8);
        assert_eq!(pkt.att_read_rsp().value[1..1 + KEY_SIZE], encrypted_ltk);

        // Check state transition
        assert_eq!(BLE_PAIR_ST.get(), PairState::Completed);
        assert_eq!(PAIR_READ_PENDING.get(), false);

        // Verify session key was restored from copy
        assert_eq!(PAIR_STATE.lock().pair_sk, initial_sk_copy);
    }

    #[test]
    fn test_pair_proc_delete_pairing_mesh_disabled() {
        // Arrange
        reset_host_fakes();
        install_light_event_recorder();
        setup_proc_state(PairState::DeletePairing, true, true, true, false, false); // Mesh disabled

        // `rf_link_delete_pair` re-saves the configured credentials, so give the
        // configuration buffers a known content.
        *PAIR_CONFIG_MESH_NAME.lock() = *b"MeshName\0\0\0\0\0\0\0\0";
        *PAIR_CONFIG_MESH_PWD.lock() = *b"NetPassw\0\0\0\0\0\0\0\0";
        FLASH_CONFIGURATION_INDEX.set(-1);

        // Act
        let result = pair_proc();

        // Assert
        assert!(result.is_some());
        let pkt = result.unwrap();
        // `rf_link_delete_pair` really ran: it revoked the login and persisted
        // the configured credentials to the pairing sector.
        assert_eq!(PAIR_LOGIN_OK.get(), false);
        let slot = pairing_slot(0);
        assert_eq!(slot[0], PAIR_CONFIG_VALID_FLAG);
        assert_eq!(&slot[0x10..0x18], b"MeshName");
        // ...and the light event reached the registered handler.
        assert_eq!(RECORDED_LIGHT_EVENTS.load(Ordering::SeqCst), 1);
        assert_eq!(
            RECORDED_LIGHT_EVENT.load(Ordering::SeqCst),
            LGT_CMD_DEL_PAIR as usize
        );

        // Check packet header (minimal response)
        let l2cap_len = pkt.head().l2cap_len;
        assert_eq!(l2cap_len, HEADER_SIZE);
        assert_eq!(pkt.head().rf_len, HEADER_SIZE as u8 + 4);
        let dma_len = pkt.head().dma_len;
        assert_eq!(dma_len, HEADER_SIZE as u32 + 6);
        assert_eq!(pkt.att_read_rsp().opcode, ATT_READ_RESPONSE_OPCODE);

        // Check packet value (only state byte)
        assert_eq!(pkt.att_read_rsp().value[0], PairState::DeletePairing as u8);
        // Rest should be zero or default
        assert!(pkt.att_read_rsp().value[1..].iter().all(|&x| x == 0));

        // Check state transition and flags
        assert_eq!(BLE_PAIR_ST.get(), PairState::Idle);
        assert_eq!(MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get(), false); // Should not be set if mesh disabled
        assert_eq!(PAIR_READ_PENDING.get(), false);
    }

    #[test]
    fn test_pair_proc_delete_pairing_mesh_enabled() {
        // Arrange
        reset_host_fakes();
        install_light_event_recorder();
        setup_proc_state(PairState::DeletePairing, true, true, true, true, false); // Mesh enabled, MESH_DEVICE_ADDRESS_VALIDATION_PENDING initially false

        *PAIR_CONFIG_MESH_NAME.lock() = *b"MeshName\0\0\0\0\0\0\0\0";
        *PAIR_CONFIG_MESH_PWD.lock() = *b"NetPassw\0\0\0\0\0\0\0\0";
        FLASH_CONFIGURATION_INDEX.set(-1);

        // Act
        let result = pair_proc();

        // Assert
        assert!(result.is_some());
        let pkt = result.unwrap();
        // `rf_link_delete_pair` really ran: it revoked the login and persisted
        // the configured credentials to the pairing sector.
        assert_eq!(PAIR_LOGIN_OK.get(), false);
        let slot = pairing_slot(0);
        assert_eq!(slot[0], PAIR_CONFIG_VALID_FLAG);
        assert_eq!(&slot[0x10..0x18], b"MeshName");
        // ...and the light event reached the registered handler.
        assert_eq!(RECORDED_LIGHT_EVENTS.load(Ordering::SeqCst), 1);
        assert_eq!(
            RECORDED_LIGHT_EVENT.load(Ordering::SeqCst),
            LGT_CMD_DEL_PAIR as usize
        );

        // Check packet header (minimal response)
        let l2cap_len = pkt.head().l2cap_len;
        assert_eq!(l2cap_len, HEADER_SIZE);
        assert_eq!(pkt.head().rf_len, HEADER_SIZE as u8 + 4);
        let dma_len = pkt.head().dma_len;
        assert_eq!(dma_len, HEADER_SIZE as u32 + 6);
        assert_eq!(pkt.att_read_rsp().opcode, ATT_READ_RESPONSE_OPCODE);

        // Check packet value (only state byte)
        assert_eq!(pkt.att_read_rsp().value[0], PairState::DeletePairing as u8);

        // Check state transition and flags
        assert_eq!(BLE_PAIR_ST.get(), PairState::Idle);
        assert_eq!(MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get(), true); // Should be set if mesh enabled
        assert_eq!(PAIR_READ_PENDING.get(), false);
    }

    #[test]
    fn test_pair_proc_init_state() {
        // Arrange
        setup_proc_state(PairState::Init, true, true, true, false, false);

        // Act
        let result = pair_proc();

        // Assert
        assert!(result.is_some());
        let pkt = result.unwrap();

        // Check packet header (special short packet)

        let l2cap_len = pkt.head().l2cap_len;
        assert_eq!(l2cap_len, HEADER_SIZE);
        assert_eq!(pkt.head().rf_len, 0x6); // Special rf_len for Init
        let dma_len = pkt.head().dma_len;
        assert_eq!(dma_len, 8); // Special dma_len for Init
        assert_eq!(pkt.att_read_rsp().opcode, ATT_READ_RESPONSE_OPCODE);

        // Check packet value (only state byte)
        assert_eq!(pkt.att_read_rsp().value[0], PairState::Init as u8);
        // Rest should be zero or default
        assert!(pkt.att_read_rsp().value[1..].iter().all(|&x| x == 0));

        // Check state transition
        assert_eq!(BLE_PAIR_ST.get(), PairState::Idle);
        assert_eq!(PAIR_READ_PENDING.get(), false); // Flag should be cleared even though we returned a packet
    }

    #[test]
    #[test]
    fn test_pair_proc_completed_case() {
        // Arrange: Use the Completed state which is explicitly handled in the match statement
        setup_proc_state(PairState::Completed, true, true, true, false, false);

        // Act
        let result = pair_proc();

        // Assert
        assert!(result.is_some(), "Expected Some(pkt) for Completed state");

        let pkt = result.unwrap();

        // Verify the state byte in the returned packet matches the initial state
        assert_eq!(
            pkt.att_read_rsp().value[0],
            PairState::Completed as u8,
            "State byte should match the initial pairing state"
        );

        // Verify Completed handler sets l2cap_len to HEADER_SIZE (2)
        let l2cap_len = pkt.head().l2cap_len;
        assert_eq!(
            l2cap_len, 2,
            "Completed state should set l2cap_len to HEADER_SIZE (2)"
        );

        // Verify rf_len and dma_len are calculated from the modified l2cap_len
        let rf_len = pkt.head().rf_len;
        let dma_len = pkt.head().dma_len;
        assert_eq!(rf_len, 2 + 0x4, "rf_len should be l2cap_len + 0x4");
        assert_eq!(dma_len, 2 + 6, "dma_len should be l2cap_len + 6");

        // Flag should be cleared
        assert_eq!(
            PAIR_READ_PENDING.get(),
            false,
            "PAIR_READ_PENDING should be cleared"
        );
    }

    #[test]
    fn test_pair_proc_other_state() {
        // Arrange: Use a state not explicitly handled in the match statement (line 655 default case)
        // RandomConfirmation is not in the explicit match arms, so it falls through to default
        setup_proc_state(
            PairState::RandomConfirmation,
            true,
            true,
            true,
            false,
            false,
        );

        // Act
        let result = pair_proc();

        // Assert
        // Unhandled states fall through and return Some(pkt) with default packet configuration
        assert!(result.is_some(), "Expected Some(pkt) for unhandled state");

        let pkt = result.unwrap();

        // Verify the state byte in the returned packet matches the initial state
        assert_eq!(
            pkt.att_read_rsp().value[0],
            PairState::RandomConfirmation as u8,
            "State byte should match the initial pairing state"
        );

        // Verify default packet configuration is used (not modified by any specific state handler)
        // Copy packed fields to local variables to avoid alignment issues
        let l2cap_len = pkt.head().l2cap_len;
        let rf_len = pkt.head().rf_len;
        let dma_len = pkt.head().dma_len;

        // Default l2cap_len is 0x17 (23) from packet initialization at line 456
        assert_eq!(l2cap_len, 0x17, "Default l2cap_len should be 0x17");

        // rf_len = l2cap_len + 0x4 (line 660)
        assert_eq!(rf_len, 0x17 + 0x4, "rf_len should be l2cap_len + 0x4");

        // dma_len = l2cap_len + 6 (line 661)
        assert_eq!(dma_len, 0x17 + 6, "dma_len should be l2cap_len + 6");

        // Flag should be cleared
        assert_eq!(
            PAIR_READ_PENDING.get(),
            false,
            "PAIR_READ_PENDING should be cleared"
        );
    }

    #[test]
    fn test_pair_set_key() {
        // Arrange
        let mut key = [0u8; 48];
        key[0..16].copy_from_slice(b"TestNetworkName0"); // 16 bytes name
        key[16..32].copy_from_slice(b"TestPassword1234"); // 16 bytes password
        key[32..48].copy_from_slice(b"TestLongTermKey5"); // 16 bytes LTK

        // Set MAX_MESH_NAME_LEN for the test (assuming it's usually 16 or less)
        // Note: MAX_MESH_NAME_LEN is now a const, so no need to set it

        reset_host_fakes();

        // Clear state before test
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state.pair_nn.fill(0);
            pair_state.pair_pass.fill(0);
            pair_state.pair_ltk.fill(0);
        }

        // Act
        pair_set_key(&key);

        // Assert
        {
            let pair_state = PAIR_STATE.lock();
            // Check that the name was copied correctly up to MAX_MESH_NAME_LEN
            assert_eq!(
                &pair_state.pair_nn[0..MAX_MESH_NAME_LEN],
                &key[0..MAX_MESH_NAME_LEN]
            );
            // Check that the password was copied correctly
            assert_eq!(pair_state.pair_pass, &key[16..32]);
            // Check that the LTK was copied correctly
            assert_eq!(pair_state.pair_ltk, &key[32..48]);
        }

        // `pair_set_key` refreshed the derived RF identity: the access code is
        // the real derivation over the keys just stored.
        assert_eq!(
            PAIR_AC.get(),
            access_code(&key[0..16], &key[16..32]),
            "pair_update_key must derive the access code from the new keys"
        );
    }

    #[test]
    fn test_pair_read_sets_pending_flag() {
        // Arrange
        PAIR_READ_PENDING.set(false); // Ensure the flag is initially false
        let dummy_packet = Packet {
            /* ... initialize with dummy data if needed, though pair_read doesn't use it ... */
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Act
        pair_read(&dummy_packet);

        // Assert
        assert_eq!(PAIR_READ_PENDING.get(), true); // Verify the flag is set
    }

    #[test]
    fn test_pair_write_exchange_random() {
        // Arrange
        setup_proc_state(PairState::Idle, false, true, true, false, false);

        // Create a packet with EXCHANGE_RANDOM opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + random challenge
        const PAIR_OP_EXCHANGE_RANDOM: u8 = 0x01;
        let test_random = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
        pkt.att_val_mut().value[0] = PAIR_OP_EXCHANGE_RANDOM;
        pkt.att_val_mut().value[1..1 + test_random.len()].copy_from_slice(&test_random);

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::AwaitingRandom);

        // Verify the random challenge was stored
        let pair_state = PAIR_STATE.lock();
        assert_eq!(pair_state.pair_randm, test_random);
    }

    #[test]
    fn test_pair_write_set_mesh_name_security_enabled_completed_state() {
        // Arrange
        setup_proc_state(PairState::Completed, false, true, false, false, false);

        // Create a packet with SET_MESH_NAME opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + encrypted mesh name
        const PAIR_OP_SET_MESH_NAME: u8 = 0x04;
        let decrypted_name = b"TestNetwork\0\0\0\0\0";
        // Secure mode decrypts with the session key, so the wire bytes are the
        // real AES-CCM block cipher over the expected plaintext.
        let session_sk = PAIR_STATE.lock().pair_sk;
        let encrypted_name = aes_att_encryption(&session_sk, decrypted_name);
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_NAME;
        pkt.att_val_mut().value[1..1 + encrypted_name.len()].copy_from_slice(&encrypted_name);

        // Set MAX_MESH_NAME_LEN
        // Note: MAX_MESH_NAME_LEN is now a const, so no need to set it

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::ReceivingMeshName);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), PairAuthState::PairSetting);

        // Verify mesh name was stored after decryption
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_nn[0..11], b"TestNetwork");
    }

    #[test]
    fn test_pair_write_set_mesh_name_security_disabled_login_ok() {
        // Arrange
        setup_proc_state(PairState::Completed, true, false, true, false, false);

        // Create a packet with SET_MESH_NAME opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + plaintext mesh name
        const PAIR_OP_SET_MESH_NAME: u8 = 0x04;
        let plaintext_name = b"PlainNetwork\0\0\0\0\0";
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_NAME;
        pkt.att_val_mut().value[1..1 + plaintext_name.len()].copy_from_slice(plaintext_name);

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::ReceivingMeshName);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), PairAuthState::PairSetting);

        // Verify mesh name was stored directly
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_nn[0..12], b"PlainNetwork");
    }

    #[test]
    fn test_pair_write_set_mesh_name_security_disabled_not_logged_in() {
        // Arrange
        setup_proc_state(PairState::Completed, false, false, false, false, false);

        // Create a packet with SET_MESH_NAME opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + plaintext mesh name
        const PAIR_OP_SET_MESH_NAME: u8 = 0x04;
        let plaintext_name = b"FailNetwork\0\0\0\0\0\0";
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_NAME;
        pkt.att_val_mut().value[1..1 + plaintext_name.len()].copy_from_slice(plaintext_name);

        // Mock pair_par_init for the reset

        // Act
        pair_write(&pkt);

        // Assert
        // `pair_par_init` really ran: the pairing state machine was reset to Init.
        assert_eq!(BLE_PAIR_ST.get(), PairState::Init);
        assert_eq!(PAIR_ENC_ENABLE.get(), false);
    }

    #[test]
    fn test_pair_write_set_mesh_name_wrong_state() {
        // Arrange
        setup_proc_state(PairState::Idle, false, true, false, false, false);

        // Create a packet with SET_MESH_NAME opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + encrypted mesh name
        const PAIR_OP_SET_MESH_NAME: u8 = 0x04;
        let encrypted_name = [
            0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE,
            0xFF, 0x00,
        ];
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_NAME;
        pkt.att_val_mut().value[1..1 + encrypted_name.len()].copy_from_slice(&encrypted_name);

        // Mock pair_par_init for the reset

        // Act
        pair_write(&pkt);

        // Assert
        // `pair_par_init` really ran: the pairing state machine was reset to Init.
        assert_eq!(BLE_PAIR_ST.get(), PairState::Init);
        assert_eq!(PAIR_ENC_ENABLE.get(), false);
    }

    #[test]
    fn test_pair_write_set_mesh_password_security_enabled() {
        // Arrange
        setup_proc_state(
            PairState::ReceivingMeshName,
            false,
            true,
            false,
            false,
            false,
        );

        // Create a packet with SET_MESH_PASSWORD opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + encrypted password
        const PAIR_OP_SET_MESH_PASSWORD: u8 = 0x05;
        let decrypted_pass = b"SecretPassword12";
        // Secure mode decrypts with the session key, so the wire bytes are the
        // real AES-CCM block cipher over the expected plaintext.
        let session_sk = PAIR_STATE.lock().pair_sk;
        let encrypted_pass = aes_att_encryption(&session_sk, decrypted_pass);
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_PASSWORD;
        pkt.att_val_mut().value[1..1 + encrypted_pass.len()].copy_from_slice(&encrypted_pass);

        // Set up network name
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state.pair_nn = *b"TestNetwork\0\0\0\0\0";
        }

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::ReceivingMeshPassword);

        // Verify password was stored after decryption
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_pass, decrypted_pass);
    }

    #[test]
    fn test_pair_write_set_mesh_password_security_enabled_wrong_state() {
        // Arrange - Not in ReceivingMeshName state
        setup_proc_state(PairState::Idle, false, true, false, false, false);

        // Create a packet with SET_MESH_PASSWORD opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + encrypted password
        const PAIR_OP_SET_MESH_PASSWORD: u8 = 0x05;
        let encrypted_pass = [
            0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE,
            0xFF, 0x00,
        ];
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_PASSWORD;
        pkt.att_val_mut().value[1..1 + encrypted_pass.len()].copy_from_slice(&encrypted_pass);

        // Mock pair_par_init for the reset

        // Act
        pair_write(&pkt);

        // Assert
        // `pair_par_init` really ran: the pairing state machine was reset to Init.
        assert_eq!(BLE_PAIR_ST.get(), PairState::Init);
        assert_eq!(PAIR_ENC_ENABLE.get(), false);
    }

    #[test]
    fn test_pair_write_set_mesh_password_security_disabled_login_ok() {
        // Arrange
        setup_proc_state(
            PairState::ReceivingMeshName,
            true,
            false,
            true,
            false,
            false,
        );

        // Create a packet with SET_MESH_PASSWORD opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + plaintext password
        const PAIR_OP_SET_MESH_PASSWORD: u8 = 0x05;
        let plaintext_pass = b"PlainPassword123";
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_PASSWORD;
        pkt.att_val_mut().value[1..1 + plaintext_pass.len()].copy_from_slice(plaintext_pass);

        // Set up network name (different than password)
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state.pair_nn = *b"DifferentNetwork";
        }

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::ReceivingMeshPassword);

        // Verify password was stored directly
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_pass, plaintext_pass);
    }

    #[test]
    fn test_pair_write_set_mesh_password_identical_to_name() {
        // Arrange
        setup_proc_state(
            PairState::ReceivingMeshName,
            true,
            false,
            true,
            false,
            false,
        );

        // Create a packet with SET_MESH_PASSWORD opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + plaintext password
        const PAIR_OP_SET_MESH_PASSWORD: u8 = 0x05;
        let same_value = b"SameNetAndPass12";
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_PASSWORD;
        pkt.att_val_mut().value[1..1 + same_value.len()].copy_from_slice(same_value);

        // Set up network name identical to password
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state.pair_nn = *same_value;
        }

        // Mock pair_par_init for the reset

        // Act
        pair_write(&pkt);

        // Assert
        // `pair_par_init` really ran: the pairing state machine was reset to Init.
        assert_eq!(BLE_PAIR_ST.get(), PairState::Init);
        assert_eq!(PAIR_ENC_ENABLE.get(), false);
    }

    #[test]
    fn test_pair_write_set_mesh_ltk_security_enabled() {
        // Arrange
        reset_host_fakes();
        install_light_event_recorder();
        FLASH_CONFIGURATION_INDEX.set(-1); // No configuration stored yet
        setup_proc_state(
            PairState::ReceivingMeshPassword,
            false,
            true,
            false,
            false,
            false,
        );

        // Create a packet with SET_MESH_LTK opcode
        let mut pkt = Packet {
            att_val: PacketAttRawValue {
                head: PacketL2capHead {
                    l2cap_len: 0x13,
                    chan_id: 0,
                    _type: 0,
                    dma_len: 0,
                    rf_len: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + encrypted LTK
        const PAIR_OP_SET_MESH_LTK: u8 = 0x06;
        let decrypted_ltk = b"DecryptedLTK1234";
        // Secure mode decrypts with the session key, so the wire bytes are the
        // real AES-CCM block cipher over the expected plaintext.
        let session_sk = PAIR_STATE.lock().pair_sk;
        let encrypted_ltk = aes_att_encryption(&session_sk, decrypted_ltk);
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_LTK;
        pkt.att_val_mut().value[1..1 + encrypted_ltk.len()].copy_from_slice(&encrypted_ltk);

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::ReceivingMeshLtk);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), PairAuthState::PairSetted);

        // Verify LTK was stored after decryption
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_ltk, decrypted_ltk);

        // `pair_save_key` really ran: the header and the saved LTK are in the
        // first pairing slot of the NOR device.
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), 0);
        let slot = pairing_slot(0);
        assert_eq!(slot[0], PAIR_CONFIG_VALID_FLAG);
        assert_eq!(&slot[0x30..0x40], &pair_state.pair_ltk);

        // ...and the "pair OK" event reached the registered handler.
        assert_eq!(RECORDED_LIGHT_EVENTS.load(Ordering::SeqCst), 1);
        assert_eq!(
            RECORDED_LIGHT_EVENT.load(Ordering::SeqCst),
            LGT_CMD_PAIR_OK as usize
        );
    }

    #[test]
    fn test_pair_write_set_mesh_ltk_security_enabled_mesh_flag() {
        // Arrange
        setup_proc_state(
            PairState::ReceivingMeshPassword,
            false,
            true,
            false,
            true,
            false,
        );

        // Create a packet with SET_MESH_LTK opcode and mesh flag
        let mut pkt = Packet {
            att_val: PacketAttRawValue {
                head: PacketL2capHead {
                    l2cap_len: 0x15,
                    chan_id: 0,
                    _type: 0,
                    dma_len: 0,
                    rf_len: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + encrypted LTK + mesh flag
        const PAIR_OP_SET_MESH_LTK: u8 = 0x06;
        const MESH_FLAG_OFFSET: usize = 0x11;
        let decrypted_mesh_ltk = b"DecryptMeshLTK12";
        // Secure mode decrypts with the session key, so the wire bytes are the
        // real AES-CCM block cipher over the expected plaintext.
        let session_sk = PAIR_STATE.lock().pair_sk;
        let encrypted_ltk = aes_att_encryption(&session_sk, decrypted_mesh_ltk);
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_LTK;
        pkt.att_val_mut().value[1..1 + encrypted_ltk.len()].copy_from_slice(&encrypted_ltk);
        pkt.att_val_mut().value[MESH_FLAG_OFFSET] = 0x01; // Set mesh flag

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::ReceivingMeshLtk);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), PairAuthState::PairSetMeshTxStart);

        // Verify mesh LTK was stored after decryption
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_ltk_mesh, decrypted_mesh_ltk);
    }

    #[test]
    fn test_pair_write_set_mesh_ltk_security_disabled() {
        // Arrange
        reset_host_fakes();
        install_light_event_recorder();
        FLASH_CONFIGURATION_INDEX.set(-1); // No configuration stored yet
        setup_proc_state(
            PairState::ReceivingMeshPassword,
            true,
            false,
            true,
            false,
            false,
        );

        // Create a packet with SET_MESH_LTK opcode
        let mut pkt = Packet {
            att_val: PacketAttRawValue {
                head: PacketL2capHead {
                    l2cap_len: 0x13,
                    chan_id: 0,
                    _type: 0,
                    dma_len: 0,
                    rf_len: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + plaintext LTK
        const PAIR_OP_SET_MESH_LTK: u8 = 0x06;
        let plaintext_ltk = b"PlaintextLTK1234";
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_LTK;
        pkt.att_val_mut().value[1..1 + plaintext_ltk.len()].copy_from_slice(plaintext_ltk);

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::ReceivingMeshLtk);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), PairAuthState::PairSetted);

        // Verify LTK was stored directly
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_ltk, plaintext_ltk);

        // `pair_save_key` really ran: the header and the saved LTK are in the
        // first pairing slot of the NOR device.
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), 0);
        let slot = pairing_slot(0);
        assert_eq!(slot[0], PAIR_CONFIG_VALID_FLAG);
        assert_eq!(&slot[0x30..0x40], &pair_state.pair_ltk);

        // ...and the "pair OK" event reached the registered handler.
        assert_eq!(RECORDED_LIGHT_EVENTS.load(Ordering::SeqCst), 1);
        assert_eq!(
            RECORDED_LIGHT_EVENT.load(Ordering::SeqCst),
            LGT_CMD_PAIR_OK as usize
        );
    }

    #[test]
    fn test_pair_write_set_mesh_ltk_wrong_state() {
        // Arrange - Wrong state
        setup_proc_state(PairState::Idle, false, true, false, false, false);

        // Create a packet with SET_MESH_LTK opcode
        let mut pkt = Packet {
            att_val: PacketAttRawValue {
                head: PacketL2capHead {
                    l2cap_len: 0,
                    chan_id: 0,
                    _type: 0,
                    dma_len: 0,
                    rf_len: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data
        const PAIR_OP_SET_MESH_LTK: u8 = 0x06;
        let encrypted_ltk = [
            0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE,
            0xFF, 0x00,
        ];
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_LTK;
        pkt.att_val_mut().value[1..1 + encrypted_ltk.len()].copy_from_slice(&encrypted_ltk);

        // Mock pair_par_init for the reset

        // Act
        pair_write(&pkt);

        // Assert
        // `pair_par_init` really ran: the pairing state machine was reset to Init.
        assert_eq!(BLE_PAIR_ST.get(), PairState::Init);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), PairAuthState::PairSetted); // Should be set to PairSetted upon failure
        assert_eq!(PAIR_ENC_ENABLE.get(), false);
    }

    #[test]
    fn test_pair_write_set_mesh_ltk_simple_mode_fallthrough_not_logged_in() {
        // Arrange - Simple mode but not logged in (fallthrough to line 890)
        setup_proc_state(
            PairState::ReceivingMeshPassword,
            true,
            false, // security disabled (simple mode)
            false, // NOT logged in - triggers fallthrough
            false,
            false,
        );

        // Create a packet with SET_MESH_LTK opcode
        let mut pkt = Packet {
            att_val: PacketAttRawValue {
                head: PacketL2capHead {
                    l2cap_len: 0x13,
                    chan_id: 0,
                    _type: 0,
                    dma_len: 0,
                    rf_len: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data
        const PAIR_OP_SET_MESH_LTK: u8 = 0x06;
        let plaintext_ltk = b"PlaintextLTK1234";
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_LTK;
        pkt.att_val_mut().value[1..1 + plaintext_ltk.len()].copy_from_slice(plaintext_ltk);

        // Mock pair_par_init for the reset

        // Act
        pair_write(&pkt);

        // Assert - Verify fallthrough path (lines 890-892)
        // `pair_par_init` really ran: the pairing state machine was reset to Init.
        assert_eq!(BLE_PAIR_ST.get(), PairState::Init);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), PairAuthState::PairSetted); // Line 891: PAIR_SETTING_FLAG set
        assert_eq!(PAIR_ENC_ENABLE.get(), false); // Line 892: PAIR_ENC_ENABLE cleared
    }

    #[test]
    fn test_pair_write_delete_pairing() {
        // Arrange
        setup_proc_state(PairState::Completed, false, true, true, false, false);

        // Create a packet with DELETE_PAIRING opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data
        const PAIR_OP_DELETE_PAIRING: u8 = 0x0E;
        pkt.att_val_mut().value[0] = PAIR_OP_DELETE_PAIRING;

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::Idle);
        assert_eq!(PAIR_ENC_ENABLE.get(), false);
    }

    #[test]
    fn test_pair_write_reset_mesh() {
        // Arrange
        setup_proc_state(PairState::Completed, false, true, true, false, false);
        PAIR_ENC_ENABLE.set(true);

        // Create a packet with RESET_MESH opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data
        const PAIR_OP_RESET_MESH: u8 = 0x0A;
        pkt.att_val_mut().value[0] = PAIR_OP_RESET_MESH;

        // Act
        pair_write(&pkt);

        // Assert
        // RESET_MESH currently has no arm action: pairing state and the
        // encryption flag stay exactly as they were.
        assert_eq!(BLE_PAIR_ST.get(), PairState::Completed);
        assert_eq!(PAIR_ENC_ENABLE.get(), true);
    }

    #[test]
    fn test_pair_write_verify_credentials_security_disabled() {
        // Arrange
        setup_proc_state(PairState::Idle, false, false, false, false, false);

        // Create a packet with VERIFY_CREDENTIALS opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + random challenge + credentials
        const PAIR_OP_VERIFY_CREDENTIALS: u8 = 0x0C;
        let test_random = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

        // Set up network name and password with known values
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state.pair_nn = *b"TestNetwork\0\0\0\0\0";
            pair_state.pair_pass = *b"TestPassword\0\0\0\0";
        }

        // In simple mode, the client sends XOR(nn, pass)
        let mut expected_credential_proof = [0u8; 16];
        for i in 0..16 {
            expected_credential_proof[i] =
                b"TestNetwork\0\0\0\0\0\0"[i] ^ b"TestPassword\0\0\0\0\0"[i];
        }

        pkt.att_val_mut().value[0] = PAIR_OP_VERIFY_CREDENTIALS;
        pkt.att_val_mut().value[1..1 + test_random.len()].copy_from_slice(&test_random);

        // Simple mode sends the full credential proof directly
        pkt.att_val_mut().value[9..9 + expected_credential_proof.len()]
            .copy_from_slice(&expected_credential_proof);

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::SessionKeyExchange);
        assert_eq!(PAIR_LOGIN_OK.get(), true);

        // Verify the random challenge was stored
        let pair_state = PAIR_STATE.lock();
        assert_eq!(pair_state.pair_randm, test_random);
    }

    #[test]
    fn test_pair_write_verify_credentials_security_enabled() {
        // Arrange
        setup_proc_state(PairState::Idle, false, true, false, false, false);

        // Create a packet with VERIFY_CREDENTIALS opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + random challenge + encrypted credentials
        const PAIR_OP_VERIFY_CREDENTIALS: u8 = 0x0C;
        let test_random = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
        let encrypted_credential_proof = [0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8];

        pkt.att_val_mut().value[0] = PAIR_OP_VERIFY_CREDENTIALS;
        pkt.att_val_mut().value[1..1 + test_random.len()].copy_from_slice(&test_random);
        pkt.att_val_mut().value[9..9 + encrypted_credential_proof.len()]
            .copy_from_slice(&encrypted_credential_proof);

        // Set up network name and password with known values
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state.pair_nn = *b"TestNetwork\0\0\0\0\0";
            pair_state.pair_pass = *b"TestPassword\0\0\0\0";
        }

        // In secure mode the client proof is the AES-CCM block cipher over the
        // XORed credentials, keyed by `randm || zeros`. Building it with the real
        // implementation pins both the key and the plaintext production uses.
        let mut proof_key = [0u8; 16];
        proof_key[0..RANDOM_CHALLENGE_SIZE].copy_from_slice(&test_random);
        let mut proof_plain = [0u8; 16];
        for index in 0..16 {
            proof_plain[index] = b"TestPassword\0\0\0\0"[index] ^ b"TestNetwork\0\0\0\0\0"[index];
        }
        let expected_work = aes_att_encryption(&proof_key, &proof_plain);
        pkt.att_val_mut().value[9..9 + RANDOM_CHALLENGE_SIZE]
            .copy_from_slice(&expected_work[0..RANDOM_CHALLENGE_SIZE]);

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::SessionKeyExchange);
        assert_eq!(PAIR_LOGIN_OK.get(), true);
        assert_eq!(PAIR_ENC_ENABLE.get(), false); // Temporarily disabled during verification

        // Verify the random challenge was stored
        let pair_state = PAIR_STATE.lock();
        assert_eq!(pair_state.pair_randm, test_random);
    }

    #[test]
    fn test_pair_write_verify_credentials_failure() {
        // Arrange
        setup_proc_state(PairState::Idle, false, true, false, false, false);

        // Create a packet with VERIFY_CREDENTIALS opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + random challenge + incorrect encrypted credentials
        const PAIR_OP_VERIFY_CREDENTIALS: u8 = 0x0C;
        let test_random = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
        let incorrect_credential_proof = [0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8];

        pkt.att_val_mut().value[0] = PAIR_OP_VERIFY_CREDENTIALS;
        pkt.att_val_mut().value[1..1 + test_random.len()].copy_from_slice(&test_random);
        pkt.att_val_mut().value[9..9 + incorrect_credential_proof.len()]
            .copy_from_slice(&incorrect_credential_proof);

        // Set up network name and password
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state.pair_nn = *b"TestNetwork\0\0\0\0\0";
            pair_state.pair_pass = *b"TestPassword\0\0\0\0";
        }

        // The packet carries a proof that the real AES-CCM block cipher does not
        // produce for these credentials, so verification must fail.

        // Mock pair_par_init for the reset

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(PAIR_LOGIN_OK.get(), false); // Login should fail

        // `pair_par_init` really ran: the pairing state machine was reset to Init.
        assert_eq!(BLE_PAIR_ST.get(), PairState::Init);
    }

    #[test]
    fn test_pair_write_get_mesh_ltk_security_disabled_logged_in() {
        // Arrange
        setup_proc_state(PairState::Idle, false, false, true, false, false);

        // Create a packet with GET_MESH_LTK opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + random challenge + credentials
        const PAIR_OP_GET_MESH_LTK: u8 = 0x08;
        let test_random = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

        // Set up network name and password with known values
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state.pair_nn = *b"TestNetwork\0\0\0\0\0";
            pair_state.pair_pass = *b"TestPassword\0\0\0\0";
            // Save session key for copy operation
            pair_state.pair_sk_copy.fill(0x44);
        }

        // In simple mode, the client sends XOR(nn, pass)
        let mut expected_credential_proof = [0u8; 16];
        for i in 0..16 {
            expected_credential_proof[i] =
                b"TestNetwork\0\0\0\0\0\0"[i] ^ b"TestPassword\0\0\0\0\0"[i];
        }

        pkt.att_val_mut().value[0] = PAIR_OP_GET_MESH_LTK;
        pkt.att_val_mut().value[1..1 + test_random.len()].copy_from_slice(&test_random);

        // Simple mode sends the full credential proof directly
        pkt.att_val_mut().value[9..9 + expected_credential_proof.len()]
            .copy_from_slice(&expected_credential_proof);

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::RequestingLtk);

        // Verify the random challenge was stored
        let pair_state = PAIR_STATE.lock();
        assert_eq!(pair_state.pair_randm, test_random);
    }

    #[test]
    fn test_pair_write_get_mesh_ltk_verification_failed() {
        // Arrange - Verification will fail
        setup_proc_state(PairState::Idle, true, false, true, false, false);

        // Create a packet with GET_MESH_LTK opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + random challenge + incorrect credentials
        const PAIR_OP_GET_MESH_LTK: u8 = 0x08;
        let test_random = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
        let incorrect_credential_proof = [
            0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6,
            0xC7, 0xC8,
        ];

        pkt.att_val_mut().value[0] = PAIR_OP_GET_MESH_LTK;
        pkt.att_val_mut().value[1..1 + test_random.len()].copy_from_slice(&test_random);
        pkt.att_val_mut().value[9..9 + incorrect_credential_proof.len()]
            .copy_from_slice(&incorrect_credential_proof);

        // Set up network name and password with values that won't match
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state.pair_nn = *b"DifferentNetwork";
            pair_state.pair_pass = *b"DifferentPass123";
        }

        // Mock pair_par_init for the reset

        // Act
        pair_write(&pkt);

        // Assert
        // `pair_par_init` really ran: the pairing state machine was reset to Init.
        assert_eq!(BLE_PAIR_ST.get(), PairState::Init);
    }

    #[test]
    fn test_pair_write_get_mesh_ltk_saves_session_key() {
        // Arrange - Test that session key is saved (line 916)
        setup_proc_state(PairState::Idle, false, false, true, false, false);

        // Create a packet with GET_MESH_LTK opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + random challenge + credentials
        const PAIR_OP_GET_MESH_LTK: u8 = 0x08;
        let test_random = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

        // Set up network name and password with known values
        let original_session_key = [0x55u8; 16];
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state.pair_nn = *b"TestNetwork\0\0\0\0\0";
            pair_state.pair_pass = *b"TestPassword\0\0\0\0";
            // Set a specific session key that we'll verify gets copied
            pair_state.pair_sk = original_session_key;
            pair_state.pair_sk_copy.fill(0x00); // Initially empty
        }

        // In simple mode, the client sends XOR(nn, pass)
        let mut expected_credential_proof = [0u8; 16];
        for i in 0..16 {
            expected_credential_proof[i] =
                b"TestNetwork\0\0\0\0\0\0"[i] ^ b"TestPassword\0\0\0\0\0"[i];
        }

        pkt.att_val_mut().value[0] = PAIR_OP_GET_MESH_LTK;
        pkt.att_val_mut().value[1..1 + test_random.len()].copy_from_slice(&test_random);
        pkt.att_val_mut().value[9..9 + expected_credential_proof.len()]
            .copy_from_slice(&expected_credential_proof);

        // Act
        pair_write(&pkt);

        // Assert

        // Verify that session key was copied (line 916)
        // The copy should contain the original session key before modification
        let pair_state = PAIR_STATE.lock();
        assert_eq!(pair_state.pair_sk_copy, original_session_key, "Session key copy should contain the original session key from before GET_MESH_LTK processing");

        // Verify that pair_sk was modified to contain the master random (lines 924-925)
        let mut expected_sk = [0u8; 16];
        expected_sk[0..8].copy_from_slice(&test_random);
        // Rest is filled with 0 (line 926)
        assert_eq!(
            pair_state.pair_sk, expected_sk,
            "Session key should be updated with master random"
        );
    }

    #[test]
    fn test_pair_write_get_mesh_ltk_fallthrough_verification_failed_not_logged_in() {
        // Arrange - Test fallthrough when verification fails and not logged in (lines 964-968)
        setup_proc_state(PairState::Idle, false, false, false, false, false); // Not logged in

        // Create a packet with GET_MESH_LTK opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + random challenge + INCORRECT credentials
        const PAIR_OP_GET_MESH_LTK: u8 = 0x08;
        let test_random = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

        // Set up network name and password with known values
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state.pair_nn = *b"TestNetwork\0\0\0\0\0";
            pair_state.pair_pass = *b"TestPassword\0\0\0\0";
        }

        pkt.att_val_mut().value[0] = PAIR_OP_GET_MESH_LTK;
        pkt.att_val_mut().value[1..1 + test_random.len()].copy_from_slice(&test_random);

        // Send INCORRECT credential proof (not matching XOR(nn, pass))
        let incorrect_proof = [0xFFu8; 16];
        pkt.att_val_mut().value[9..9 + incorrect_proof.len()].copy_from_slice(&incorrect_proof);

        // Mock pair_par_init for the reset

        // Act
        pair_write(&pkt);

        // Assert - Verify fallthrough path (lines 964-968)
        // Since we're not logged in and this is GET_MESH_LTK (not verify),
        // the condition at line 950 (is_verify || PAIR_LOGIN_OK.get()) fails
        // So we skip the block and fall through to pair_par_init() at line 964
        // `pair_par_init` really ran: the pairing state machine was reset to Init.
        assert_eq!(BLE_PAIR_ST.get(), PairState::Init);
    }

    #[test]
    fn test_pair_write_set_mesh_name_no_null_terminator() {
        // Arrange
        setup_proc_state(PairState::Completed, false, true, false, false, false);

        // Create a packet with SET_MESH_NAME opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + encrypted mesh name
        const PAIR_OP_SET_MESH_NAME: u8 = 0x04;
        // A mesh name WITHOUT null terminator: all bytes non-zero, to exercise
        // the `None => pair_state.pair_nn.len()` case
        let decrypted_name = [
            0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E,
            0x4F, 0x50,
        ];
        // Secure mode decrypts with the session key, so the wire bytes are the
        // real AES-CCM block cipher over the expected plaintext.
        let session_sk = PAIR_STATE.lock().pair_sk;
        let encrypted_name = aes_att_encryption(&session_sk, &decrypted_name);
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_NAME;
        pkt.att_val_mut().value[1..1 + encrypted_name.len()].copy_from_slice(&encrypted_name);

        // Set MAX_MESH_NAME_LEN to be exactly the buffer length to accept the name
        // Note: MAX_MESH_NAME_LEN is now a const

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::ReceivingMeshName);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), PairAuthState::PairSetting);

        // Verify mesh name was stored after decryption
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_nn, &decrypted_name);
    }

    #[test]
    fn test_pair_write_set_mesh_name_no_null_terminator_too_long() {
        // Arrange
        setup_proc_state(PairState::Completed, false, true, false, false, false);

        // Create a packet with SET_MESH_NAME opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + encrypted mesh name
        const PAIR_OP_SET_MESH_NAME: u8 = 0x04;
        // A mesh name WITHOUT null terminator
        let decrypted_name = [
            0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E,
            0x4F, 0x50,
        ];
        // Secure mode decrypts with the session key, so the wire bytes are the
        // real AES-CCM block cipher over the expected plaintext.
        let session_sk = PAIR_STATE.lock().pair_sk;
        let encrypted_name = aes_att_encryption(&session_sk, &decrypted_name);
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_NAME;
        pkt.att_val_mut().value[1..1 + encrypted_name.len()].copy_from_slice(&encrypted_name);

        // MAX_MESH_NAME_LEN is now a const (16), so this 16-byte name should be accepted

        // Act
        pair_write(&pkt);

        // Assert

        // Verify pairing accepted the mesh name and transitioned to next state
        assert_eq!(BLE_PAIR_ST.get(), PairState::ReceivingMeshName);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), PairAuthState::PairSetting);
    }

    #[test]
    fn test_pair_write_set_mesh_password_security_disabled_not_logged_in() {
        // Arrange
        setup_proc_state(
            PairState::ReceivingMeshName,
            false,
            false,
            false,
            false,
            false,
        ); // Security disabled, NOT logged in

        // Create a packet with SET_MESH_PASSWORD opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + plaintext password
        const PAIR_OP_SET_MESH_PASSWORD: u8 = 0x05;
        let plaintext_pass = b"PlainPassword123";
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_PASSWORD;
        pkt.att_val_mut().value[1..1 + plaintext_pass.len()].copy_from_slice(plaintext_pass);

        // Mock pair_par_init for the reset

        // Act
        pair_write(&pkt);

        // Assert
        // `pair_par_init` really ran: the pairing state machine was reset to Init.
        assert_eq!(BLE_PAIR_ST.get(), PairState::Init);
        assert_eq!(PAIR_ENC_ENABLE.get(), false);
    }

    #[test]
    fn test_pair_write_set_mesh_password_security_disabled_wrong_state() {
        // Arrange
        setup_proc_state(PairState::Idle, false, false, true, false, false); // Security disabled, logged in, but WRONG STATE

        // Create a packet with SET_MESH_PASSWORD opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + plaintext password
        const PAIR_OP_SET_MESH_PASSWORD: u8 = 0x05;
        let plaintext_pass = b"PlainPassword123";
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_PASSWORD;
        pkt.att_val_mut().value[1..1 + plaintext_pass.len()].copy_from_slice(plaintext_pass);

        // Mock pair_par_init for the reset

        // Act
        pair_write(&pkt);

        // Assert
        // `pair_par_init` really ran: the pairing state machine was reset to Init.
        assert_eq!(BLE_PAIR_ST.get(), PairState::Init);
        assert_eq!(PAIR_ENC_ENABLE.get(), false);
    }

    #[test]
    fn test_pair_write_set_mesh_ltk_security_disabled_mesh_flag_0x80() {
        // Arrange
        reset_host_fakes();
        install_light_event_recorder();
        FLASH_CONFIGURATION_INDEX.set(-1); // No configuration stored yet
        setup_proc_state(
            PairState::ReceivingMeshPassword,
            true,
            false,
            true,
            true,
            false,
        );

        // Create a packet with SET_MESH_LTK opcode and mesh flag with 0x80 bit set
        let mut pkt = Packet {
            att_val: PacketAttRawValue {
                head: PacketL2capHead {
                    l2cap_len: 0x15, // Larger than MIN_PACKET_LEN_WITH_MESH
                    chan_id: 0,
                    _type: 0,
                    dma_len: 0,
                    rf_len: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + plaintext LTK + mesh flag with 0x80 bit
        const PAIR_OP_SET_MESH_LTK: u8 = 0x06;
        const MESH_FLAG_OFFSET: usize = 0x11;
        let plaintext_mesh_ltk = b"MeshLongTermKey0";
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_LTK;
        pkt.att_val_mut().value[1..1 + plaintext_mesh_ltk.len()]
            .copy_from_slice(plaintext_mesh_ltk);
        pkt.att_val_mut().value[MESH_FLAG_OFFSET] = 0x80; // Set the high bit for mesh flag

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::ReceivingMeshLtk);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), PairAuthState::PairSetMeshTxStart);

        // Verify mesh LTK was stored
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_ltk_mesh, plaintext_mesh_ltk);

        // Neither the save nor the event ran on this path: the pairing sector
        // is untouched and no event was delivered.
        assert_eq!(pairing_slot(0)[0], 0xFF);
        assert_eq!(RECORDED_LIGHT_EVENTS.load(Ordering::SeqCst), 0);
    }

    #[test]
    fn test_pair_write_set_mesh_ltk_security_disabled_mesh_flag_other_bits() {
        // Arrange
        reset_host_fakes();
        install_light_event_recorder();
        FLASH_CONFIGURATION_INDEX.set(-1); // No configuration stored yet
        setup_proc_state(
            PairState::ReceivingMeshPassword,
            true,
            false,
            true,
            true,
            false,
        );

        // Create a packet with SET_MESH_LTK opcode and mesh flag with other bits (not 0x80) set
        let mut pkt = Packet {
            att_val: PacketAttRawValue {
                head: PacketL2capHead {
                    l2cap_len: 0x15, // Larger than MIN_PACKET_LEN_WITH_MESH
                    chan_id: 0,
                    _type: 0,
                    dma_len: 0,
                    rf_len: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + plaintext LTK + mesh flag with other bits
        const PAIR_OP_SET_MESH_LTK: u8 = 0x06;
        const MESH_FLAG_OFFSET: usize = 0x11;
        let plaintext_ltk = b"RegularLTK_01234";
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_LTK;
        pkt.att_val_mut().value[1..1 + plaintext_ltk.len()].copy_from_slice(plaintext_ltk);
        pkt.att_val_mut().value[MESH_FLAG_OFFSET] = 0x7F; // Set all bits except the high bit

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::ReceivingMeshLtk);

        // Should use the normal LTK flow, not the mesh LTK
        assert_eq!(*PAIR_SETTING_FLAG.lock(), PairAuthState::PairSetted);

        // Verify regular LTK was stored, not mesh LTK
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_ltk, plaintext_ltk);

        // `pair_save_key` really ran: the header and the saved LTK are in the
        // first pairing slot of the NOR device.
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), 0);
        let slot = pairing_slot(0);
        assert_eq!(slot[0], PAIR_CONFIG_VALID_FLAG);
        assert_eq!(&slot[0x30..0x40], &pair_state.pair_ltk);

        // ...and the "pair OK" event reached the registered handler.
        assert_eq!(RECORDED_LIGHT_EVENTS.load(Ordering::SeqCst), 1);
        assert_eq!(
            RECORDED_LIGHT_EVENT.load(Ordering::SeqCst),
            LGT_CMD_PAIR_OK as usize
        );
    }

    #[test]
    fn test_pair_write_set_mesh_ltk_security_disabled_mesh_disabled() {
        // Arrange
        reset_host_fakes();
        install_light_event_recorder();
        FLASH_CONFIGURATION_INDEX.set(-1); // No configuration stored yet
        setup_proc_state(
            PairState::ReceivingMeshPassword,
            true,
            false,
            true,
            false,
            false,
        ); // Mesh pairing DISABLED

        // Create a packet with SET_MESH_LTK opcode and mesh flag
        let mut pkt = Packet {
            att_val: PacketAttRawValue {
                head: PacketL2capHead {
                    l2cap_len: 0x15, // Larger than MIN_PACKET_LEN_WITH_MESH
                    chan_id: 0,
                    _type: 0,
                    dma_len: 0,
                    rf_len: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + plaintext LTK + mesh flag with 0x80 bit
        const PAIR_OP_SET_MESH_LTK: u8 = 0x06;
        const MESH_FLAG_OFFSET: usize = 0x11;
        let plaintext_ltk = b"RegularLTK_ABCDE";
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_LTK;
        pkt.att_val_mut().value[1..1 + plaintext_ltk.len()].copy_from_slice(plaintext_ltk);
        pkt.att_val_mut().value[MESH_FLAG_OFFSET] = 0x80; // Set the high bit for mesh flag

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::ReceivingMeshLtk);

        // Should use the normal LTK flow even with 0x80 bit set, because mesh is disabled
        assert_eq!(*PAIR_SETTING_FLAG.lock(), PairAuthState::PairSetted);

        // Verify regular LTK was stored
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_ltk, plaintext_ltk);

        // `pair_save_key` really ran: the header and the saved LTK are in the
        // first pairing slot of the NOR device.
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), 0);
        let slot = pairing_slot(0);
        assert_eq!(slot[0], PAIR_CONFIG_VALID_FLAG);
        assert_eq!(&slot[0x30..0x40], &pair_state.pair_ltk);

        // ...and the "pair OK" event reached the registered handler.
        assert_eq!(RECORDED_LIGHT_EVENTS.load(Ordering::SeqCst), 1);
        assert_eq!(
            RECORDED_LIGHT_EVENT.load(Ordering::SeqCst),
            LGT_CMD_PAIR_OK as usize
        );
    }

    #[test]
    fn test_pair_write_set_mesh_ltk_security_disabled_packet_too_short() {
        // Arrange
        reset_host_fakes();
        install_light_event_recorder();
        FLASH_CONFIGURATION_INDEX.set(-1); // No configuration stored yet
        setup_proc_state(
            PairState::ReceivingMeshPassword,
            true,
            false,
            true,
            true,
            false,
        );

        // Create a packet with SET_MESH_LTK opcode and mesh flag, but packet too short
        let mut pkt = Packet {
            att_val: PacketAttRawValue {
                head: PacketL2capHead {
                    l2cap_len: 0x13, // NOT larger than MIN_PACKET_LEN_WITH_MESH
                    chan_id: 0,
                    _type: 0,
                    dma_len: 0,
                    rf_len: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + plaintext LTK
        // Even though the MESH_FLAG_OFFSET might be beyond the packet length in real code,
        // we'll set it anyway for the test to demonstrate the packet length check
        const PAIR_OP_SET_MESH_LTK: u8 = 0x06;
        const MESH_FLAG_OFFSET: usize = 0x11;
        let plaintext_ltk = b"RegularLTK_Short";
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_LTK;
        pkt.att_val_mut().value[1..1 + plaintext_ltk.len()].copy_from_slice(plaintext_ltk);
        pkt.att_val_mut().value[MESH_FLAG_OFFSET] = 0x80; // Set the high bit, but packet length will cause this to be ignored

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::ReceivingMeshLtk);

        // Should use the normal LTK flow because packet is too short
        assert_eq!(*PAIR_SETTING_FLAG.lock(), PairAuthState::PairSetted);

        // Verify regular LTK was stored
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_ltk, plaintext_ltk);

        // `pair_save_key` really ran: the header and the saved LTK are in the
        // first pairing slot of the NOR device.
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), 0);
        let slot = pairing_slot(0);
        assert_eq!(slot[0], PAIR_CONFIG_VALID_FLAG);
        assert_eq!(&slot[0x30..0x40], &pair_state.pair_ltk);

        // ...and the "pair OK" event reached the registered handler.
        assert_eq!(RECORDED_LIGHT_EVENTS.load(Ordering::SeqCst), 1);
        assert_eq!(
            RECORDED_LIGHT_EVENT.load(Ordering::SeqCst),
            LGT_CMD_PAIR_OK as usize
        );
    }

    #[test]
    fn test_pair_write_set_mesh_ltk_security_enabled_mesh_flag_0x80() {
        // Arrange
        reset_host_fakes();
        install_light_event_recorder();
        FLASH_CONFIGURATION_INDEX.set(-1); // No configuration stored yet
        setup_proc_state(
            PairState::ReceivingMeshPassword,
            false,
            true,
            false,
            true,
            false,
        );

        // Create a packet with SET_MESH_LTK opcode and mesh flag with 0x80 bit set
        let mut pkt = Packet {
            att_val: PacketAttRawValue {
                head: PacketL2capHead {
                    l2cap_len: 0x15, // Larger than MIN_PACKET_LEN_WITH_MESH
                    chan_id: 0,
                    _type: 0,
                    dma_len: 0,
                    rf_len: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data: opcode + encrypted LTK + mesh flag with 0x80 bit
        const PAIR_OP_SET_MESH_LTK: u8 = 0x06;
        const MESH_FLAG_OFFSET: usize = 0x11;
        let decrypted_mesh_ltk = b"DecryptedMeshLTK";
        // Secure mode decrypts with the session key, so the wire bytes are the
        // real AES-CCM block cipher over the expected plaintext.
        let session_sk = PAIR_STATE.lock().pair_sk;
        let encrypted_mesh_ltk = aes_att_encryption(&session_sk, decrypted_mesh_ltk);
        pkt.att_val_mut().value[0] = PAIR_OP_SET_MESH_LTK;
        pkt.att_val_mut().value[1..1 + encrypted_mesh_ltk.len()]
            .copy_from_slice(&encrypted_mesh_ltk);
        pkt.att_val_mut().value[MESH_FLAG_OFFSET] = 0x80; // Set the high bit for mesh flag

        // Act
        pair_write(&pkt);

        // Assert
        assert_eq!(BLE_PAIR_ST.get(), PairState::ReceivingMeshLtk);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), PairAuthState::PairSetMeshTxStart);

        // Verify mesh LTK was stored after decryption
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_ltk_mesh, decrypted_mesh_ltk);

        // The save did not run on this path: the pairing sector is untouched.
        assert_eq!(pairing_slot(0)[0], 0xFF);
    }

    #[test]
    fn test_pair_write_unknown_opcode() {
        // Arrange - Test default case for unknown opcode (line 978-980)
        setup_proc_state(PairState::Idle, false, false, false, false, false);

        // Create a packet with an unknown opcode
        let mut pkt = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 0,
                chan_id: 0,
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: [0; 30],
            },
        };

        // Setup packet data with unknown/unsupported opcode
        const UNKNOWN_OPCODE: u8 = 0xFF; // Not any of the handled opcodes
        pkt.att_val_mut().value[0] = UNKNOWN_OPCODE;

        // Act
        pair_write(&pkt);

        // Assert - Default case with unknown opcode is handled gracefully
    }
}
