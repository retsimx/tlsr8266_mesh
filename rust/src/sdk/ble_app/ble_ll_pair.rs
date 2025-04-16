use core::ptr::{addr_of, addr_of_mut, slice_from_raw_parts_mut};
use core::slice;

use crate::common::{pair_flash_clean, pair_load_key, pair_update_key, save_pair_info};
use crate::main_light::rf_link_light_event_callback;
use crate::sdk::ble_app::light_ll::rf_link_delete_pair;
use crate::sdk::light::*;
use crate::sdk::mcu::crypto::{aes_att_decryption, aes_att_decryption_packet, aes_att_encryption, aes_att_encryption_packet, encode_password};
use crate::sdk::mcu::register::read_reg_system_tick;
use crate::sdk::packet_types::{Packet, PacketAttReadRsp, PacketAttValue, PacketL2capHead};
use crate::state::{*, PairState};

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

// Pairing Opcodes - Used in pair_write to identify the command type
const PAIR_OP_EXCHANGE_RANDOM: u8 = 0x01;     // Exchange random challenge
const PAIR_OP_SET_MESH_NAME: u8 = 0x04;       // Set mesh network name
const PAIR_OP_SET_MESH_PASSWORD: u8 = 0x05;   // Set mesh network password
const PAIR_OP_SET_MESH_LTK: u8 = 0x06;        // Set long-term key
const PAIR_OP_GET_MESH_LTK: u8 = 0x08;        // Request long-term key
const PAIR_OP_RESET_MESH: u8 = 0x0A;          // Reset mesh configuration
const PAIR_OP_VERIFY_CREDENTIALS: u8 = 0x0C;  // Verify credentials
const PAIR_OP_DELETE_PAIRING: u8 = 0x0E;      // Delete pairing information

// Packet field constants
const ATT_READ_RESPONSE_OPCODE: u8 = 0x0B;  // ATT Read Response opcode
const MESH_BROADCAST_CHANNEL: u16 = 0xFFFF; // Channel ID for broadcast packets
const PACKET_TYPE_ENCRYPTED: u8 = 0x80;     // Flag indicating packet is encrypted

// Buffer size constants
const KEY_SIZE: usize = 0x10;               // Size of cryptographic keys (16 bytes)
const RANDOM_CHALLENGE_SIZE: usize = 8;     // Size of random challenge values (8 bytes)
const HEADER_SIZE: u16 = 2;                 // Size of header in value field (2 bytes)

// Response packet size constants
const RESP_WITH_RANDOM: u16 = 10;           // 8 bytes random + 2 header bytes
const RESP_WITH_KEY: u16 = 0x12;            // 16 bytes key data + 2 header bytes

// Constants for flash operations
const SECTOR_SIZE: i32 = 0x40;        // Size of each flash storage sector as i32

// Command codes for light events
const LGT_CMD_DEL_PAIR: u8 = 0xc7;    // Delete pairing information
const LGT_CMD_PAIR_OK: u8 = 0xc5;    // Pairing successful

/// Decrypt an incoming BLE packet using the current pairing session key
/// 
/// This function is called for encrypted packets when security is enabled.
/// It extracts initialization vectors and source information from the packet,
/// then uses the current pairing session key (pair_sk) to decrypt the payload.
///
/// @param ps The packet to decrypt
/// @return true if decryption was successful, false otherwise
pub fn pair_dec_packet(ps: &mut Packet) -> bool {
    // Early return if security is not enabled
    if !SECURITY_ENABLE.get() {
        return false;
    }

    let mut pair_ivm = PAIR_IVM.lock();
    
    // Extract sequence number from packet as part of IV
    pair_ivm[5..5 + 3].copy_from_slice(&ps.att_write().value.sno);

    // Extract source address from packet
    let src = ps.att_write().value.src;

    // Calculate data length from packet header
    let l2len = ps.head().l2cap_len as usize;
    
    // Create a slice pointing to the payload data that needs decryption
    let data_ptr = addr_of_mut!(ps.att_write_mut().value.dst) as *mut u8;
    let data_len = l2len - 8;
    
    // Get current session key for decryption
    let pair_sk = &PAIR_STATE.lock().pair_sk;
    
    // Perform decryption using AES-CCM
    unsafe {
        // Create data slice and decrypt in-place
        let data = slice_from_raw_parts_mut(data_ptr, data_len);
        aes_att_decryption_packet(
            pair_sk,      // Session key
            &*pair_ivm,   // IV modifier
            &src,         // Expected MIC (Message Integrity Check)
            &mut *data,   // Data to decrypt (in place)
        )
    }
}

/// Encrypt an outgoing BLE packet using the current pairing session key
/// 
/// This function is called for packets that need to be encrypted when security is enabled.
/// It uses a timestamp as a sequence number to prevent replay attacks and updates the
/// initialization vector. The packet is then encrypted with the current session key.
///
/// @param ps The packet to encrypt
pub fn pair_enc_packet(ps: &mut Packet)
{
    // Constants for encryption conditions and offsets
    const ENCRYPTION_CHANNEL: u16 = 4;
    const ENCRYPTION_OPCODE: u8 = 0x1b;
    const ENCRYPTION_HANDLE: u8 = 0x12;
    const SNO_LENGTH: usize = 3;
    const IV_SEQ_OFFSET: usize = 3;
    const IV_SRC_OFFSET: usize = 6;
    const L2CAP_HEADER_LEN: u16 = 10;
    
    // Check if encryption is needed and packet meets encryption criteria
    if !SECURITY_ENABLE.get() || 
       ps.head().chan_id != ENCRYPTION_CHANNEL || 
       ps.ll_app().opcode != ENCRYPTION_OPCODE || 
       ps.ll_app().handle != ENCRYPTION_HANDLE {
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
    pair_ivs[IV_SRC_OFFSET] = src as u8;         // Low byte
    pair_ivs[IV_SRC_OFFSET + 1] = (src >> 8) as u8;  // High byte
    
    // Get session key for encryption
    let session_key = &PAIR_STATE.lock().pair_sk;
    
    // Calculate data length from L2CAP header length
    let data_len = ((ps.head().l2cap_len & 0xff) - L2CAP_HEADER_LEN) as usize;
    
    // Create destination and payload slices for encryption
    let dst_slice = unsafe { 
        slice::from_raw_parts_mut(addr_of!(ps.ll_app().value.dst) as *mut u8, 2) 
    };
    
    let data_slice = unsafe { 
        slice::from_raw_parts_mut(addr_of_mut!(ps.ll_app_mut().value.op), data_len)
    };
    
    // Perform encryption using AES-CCM
    aes_att_encryption_packet(
        session_key, // Session key
        &*pair_ivs,  // IV modifier
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
pub fn pair_dec_packet_mesh(ps: &mut Packet) -> bool {
    // Early return if security is disabled - we treat packets as already decrypted
    if !SECURITY_ENABLE.get() {
        return true;
    }

    // Validate this packet has the encryption flag set
    if ps.head()._type & PACKET_TYPE_ENCRYPTED == 0 {
        return false;
    }

    // Check if packet length is within valid range for decryption
    let rf_len = ps.head().rf_len;
    let payload_len = rf_len - 0x12;
    if payload_len > 0x13 {
        return false;
    }

    // Get the long-term key for mesh decryption
    let ltk = &PAIR_STATE.lock().pair_ltk;
    
    // Decrypt the packet based on its type (broadcast vs directed)
    let is_broadcast = ps.head().chan_id == MESH_BROADCAST_CHANNEL;
    
    if is_broadcast {
        // Broadcast packet structure:
        // - IV: first 8 bytes starting from rf_len
        // - Expected MIC: 2 bytes from internal_par2[1]
        // - Data: 0x1c bytes starting from sno field
        aes_att_decryption_packet(
            ltk, // Long-term key
            unsafe { &*(addr_of!(ps.head().rf_len) as *const [u8; 8]) }, // IV
            unsafe { slice::from_raw_parts(addr_of!(ps.mesh().internal_par2[1]), 2) }, // Expected MIC
            unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().sno) as *mut u8, 0x1c) }, // Data
        )
    } else {
        // Direct mesh packet structure:
        // - IV: 8 bytes starting from handle1 field
        // - Expected MIC: 4 bytes at position sno + (rf_len - 0xb)
        // - Data: payload from op field with length (rf_len - 0x12)
        aes_att_decryption_packet(
            ltk, // Long-term key
            unsafe { &*(addr_of!(ps.mesh().handle1) as *const [u8; 8]) }, // IV
            unsafe { slice::from_raw_parts((addr_of!(ps.mesh().sno) as u32 + (rf_len as u32 - 0xb)) as *const u8, 4) }, // Expected MIC
            unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().op), payload_len as usize) }, // Data
        )
    }
}

/// This function handles encryption of mesh packets before transmission.
/// Similar to decryption, it uses the long-term key rather than the session key.
///
/// @param ps The mesh packet to encrypt
/// @return true if encryption was successful, false otherwise
pub fn pair_enc_packet_mesh(ps: &mut Packet) -> bool
{
    // Only proceed with encryption if security is enabled
    if SECURITY_ENABLE.get() {
        // Get access to the long-term key used for mesh encryption
        let pair_ltk = &PAIR_STATE.lock().pair_ltk;
        
        // Determine encryption method based on packet type (broadcast vs direct)
        let is_broadcast = ps.head().chan_id == MESH_BROADCAST_CHANNEL;
        
        if is_broadcast {
            // Broadcast packet encryption
            // - Use IV from packet header's rf_len field (8 bytes)
            let iv = unsafe { &*(addr_of!(ps.head().rf_len) as *const [u8; 8]) };
            
            // - Use destination address from internal_par2[1] as the MIC output buffer (2 bytes)
            let dst = unsafe { 
                slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().internal_par2[1]), 2) 
            };
            
            // - Encrypt payload starting from sno field (fixed size 0x1c bytes)
            let payload = unsafe { 
                slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().sno) as *mut u8, 0x1c) 
            };
            
            // Perform encryption using AES-CCM
            aes_att_encryption_packet(pair_ltk, iv, dst, payload); // dst is the mic_output buffer
            
            return true;
        } else {
            // Direct mesh packet encryption
            // - Use IV from packet's handle1 field (8 bytes)
            let iv = unsafe { &*(addr_of!(ps.mesh().handle1) as *const [u8; 8]) };
            
            // - Use destination address from position calculated based on rf_len as the MIC output buffer (4 bytes)
            let dst_ptr = unsafe { 
                (addr_of!(ps.mesh().sno) as u32 + (ps.head().rf_len as u32 - 0xb)) as *mut u8 
            };
            let dst = unsafe { slice::from_raw_parts_mut(dst_ptr, 4) };
            
            // - Encrypt payload starting from op field with length based on rf_len
            let payload_len = ps.head().rf_len as usize - 0x12;
            let payload = unsafe { 
                slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().op), payload_len) 
            };
            
            // Perform encryption using AES-CCM
            aes_att_encryption_packet(pair_ltk, iv, dst, payload); // dst is the mic_output buffer
            
            return true;
        }
    }

    false
}

/// Save pairing configuration data to flash memory
/// 
/// This function handles the saving of pairing data segments to flash memory.
/// It manages the flash sector rotation for wear-leveling when a new set of 
/// pairing data needs to be stored.
///
/// @param addr The offset within the pairing data structure
/// @param data The data to save to flash
pub fn pair_flash_save_config(addr: u32, data: &[u8])
{
    // Flash storage offset constants
    const OFFSET_HEADER: u32 = 0x00;        // Pairing header with validation flags
    const OFFSET_NAME: u32 = 0x10;          // Mesh network name
    const OFFSET_PASSWORD: u32 = 0x20;      // Mesh network password (encrypted)
    const OFFSET_LTK: u32 = 0x30;           // Long-term key

    // For a new config set (addr=0), clean the old sector and increment the index
    if addr == OFFSET_HEADER {
        pair_flash_clean();
        // Cast to i32 to match ADR_FLASH_CFG_IDX type
        ADR_FLASH_CFG_IDX.set(ADR_FLASH_CFG_IDX.get() + SECTOR_SIZE);
    }

    // Save the data to flash at the calculated address
    save_pair_info(addr, data);
}

/// Save the current pairing keys to flash memory
/// 
/// This function saves all the pairing information (mesh name, password, and
/// long-term key) to persistent storage in flash memory. It also includes
/// validation flags and mesh-specific configuration.
pub fn pair_save_key()
{
    // Flash storage offset constants
    const OFFSET_HEADER: u32 = 0x00;        // Pairing header with validation flags
    const OFFSET_NAME: u32 = 0x10;          // Mesh network name
    const OFFSET_PASSWORD: u32 = 0x20;      // Mesh network password (encrypted)
    const OFFSET_LTK: u32 = 0x30;           // Long-term key
    
    let mut pass = [0u8; 16];

    // Set validation flags that indicate valid pairing data
    pass[0] = PAIR_CONFIG_VALID_FLAG;
    pass[15] = PAIR_CONFIG_VALID_FLAG;

    // Store mesh MAC address configuration if mesh pairing is enabled
    if MESH_PAIR_ENABLE.get() {
        pass[1] = if GET_MAC_EN.get() { 1 } else { 0 };
    }

    let mut pair_state = PAIR_STATE.lock();

    // Save header with validation flags first
    pair_flash_save_config(OFFSET_HEADER, &pass);
    
    // Save mesh name
    pair_flash_save_config(OFFSET_NAME, &pair_state.pair_nn);

    // Make a copy of the password before encoding
    pass = pair_state.pair_pass;

    // Encode password before saving to flash for security
    pass = encode_password(&pass);

    // Save encoded password
    pair_flash_save_config(OFFSET_PASSWORD, &pass);
    
    // Save long-term key
    pair_flash_save_config(OFFSET_LTK, &pair_state.pair_ltk);

    // Update runtime keys and access codes based on the new pairing data
    pair_update_key();
}

/// Initialize pairing state for a new connection
/// 
/// This function resets the pairing state and prepares initialization vectors
/// for a new BLE connection. It uses the device's MAC address as part of the
/// initialization vectors for encryption.
pub fn pair_init()
{
    // Reset pairing state to initial value
    BLE_PAIR_ST.set(PairState::Idle);
    PAIR_ENC_ENABLE.set(false);
    
    // Initialize the master and slave initialization vectors with MAC ID
    const MAC_ID_SIZE: usize = 4; // Using first 4 bytes of MAC address for IVs
    PAIR_IVM.lock()[0..MAC_ID_SIZE].copy_from_slice(&MAC_ID.lock()[0..MAC_ID_SIZE]);
    PAIR_IVS.lock()[0..MAC_ID_SIZE].copy_from_slice(&MAC_ID.lock()[0..MAC_ID_SIZE]);
}

/// Initialize pairing parameters from stored keys
/// 
/// This function sets the pairing state to a special initialization value
/// and loads previously stored keys from flash memory.
pub fn pair_par_init()
{
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
pub fn pair_proc() -> Option<Packet>
{
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
            opcode: ATT_READ_RESPONSE_OPCODE,   // ATT Read Response opcode
            value: [0; 22],
        }
    };

    if BLE_PAIR_ST.get() == PairState::AwaitingRandom {
        // Generate random challenge and move to next state
        
        // Validate security state or reset if needed
        if SECURITY_ENABLE.get() == false && !PAIR_LOGIN_OK.get() {
            pair_par_init();
            PAIR_ENC_ENABLE.set(false);
            return None;
        }
        
        // Send random challenge for authentication
        pkt_read_rsp.head_mut().l2cap_len = RESP_WITH_RANDOM;  // 8 bytes of random data + 2 header bytes
        pkt_read_rsp.att_read_rsp_mut().value[1..1 + RANDOM_CHALLENGE_SIZE].copy_from_slice(&pair_state.pair_rands);
        BLE_PAIR_ST.set(PairState::RandomConfirmation);  // Advance to confirmation state
    } else if BLE_PAIR_ST.get() == PairState::ReceivingMeshLtk {
        // Compute mesh network keys - XOR network name, password, and LTK
        
        // Validate security state or reset if needed
        if SECURITY_ENABLE.get() == false && !PAIR_LOGIN_OK.get() {
            pair_par_init();
            PAIR_ENC_ENABLE.set(false);
            return None;
        }
        
        pkt_read_rsp.head_mut().l2cap_len = RESP_WITH_KEY;  // 16 bytes of key data + 2 header bytes
        
        // Compute obfuscated key material by XORing identity values
        for index in 0..KEY_SIZE {
            pair_state.pair_work[index] = pair_state.pair_nn[index] ^ pair_state.pair_pass[index] ^ pair_state.pair_ltk[index];
        }

        // Either send as plaintext or encrypt based on security mode
        if SECURITY_ENABLE.get() == false {
            pkt_read_rsp.att_read_rsp_mut().value[1..1 + KEY_SIZE].copy_from_slice(&pair_state.pair_work[0..10])
        } else {
            let encrypted_work = aes_att_encryption(&pair_state.pair_sk, &pair_state.pair_work);
            pkt_read_rsp.att_read_rsp_mut().value[1..1 + KEY_SIZE].copy_from_slice(&encrypted_work);
        }
        BLE_PAIR_ST.set(PairState::Completed);  // Advance to completed state
    } else if BLE_PAIR_ST.get() == PairState::SessionKeyExchange {
        // Different handling based on security mode enabled/disabled
        
        if SECURITY_ENABLE.get() == false {
            // Simple mode: Just XOR network name and password
            pkt_read_rsp.head_mut().l2cap_len = RESP_WITH_KEY;
            for index in 0..KEY_SIZE {
                pair_state.pair_work[index] = pair_state.pair_nn[index] ^ pair_state.pair_pass[index];
            }

            // Send obfuscated credentials
            pkt_read_rsp.att_read_rsp_mut().value[1..1 + KEY_SIZE].copy_from_slice(&pair_state.pair_work);

            BLE_PAIR_ST.set(PairState::Completed);  // Advance to completed state
            PAIR_ENC_ENABLE.set(false);
        } else {
            // Secure mode: Generate session key using AES
            pkt_read_rsp.head_mut().l2cap_len = RESP_WITH_KEY;
            
            // Generate random challenge using system tick
            pair_state.pair_rands[0..4].copy_from_slice(bytemuck::bytes_of(&read_reg_system_tick()));

            // Generate session key using master and slave random values
            // Pad randm and rands to 16 bytes for AES encryption
            let mut key_padded = [0u8; KEY_SIZE];
            key_padded[0..RANDOM_CHALLENGE_SIZE].copy_from_slice(&pair_state.pair_randm);

            let mut source_padded = [0u8; KEY_SIZE];
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
                pair_state.pair_work[index] = pair_state.pair_nn[index] ^ pair_state.pair_pass[index];
            }

            // Generate proof of possession by encrypting credentials with session key
            pair_state.pair_work = aes_att_encryption(&pair_state.pair_sk, &pair_state.pair_work);

            // Pack slave random challenge and encrypted proof into response
            pkt_read_rsp.att_read_rsp_mut().value[1..1 + RANDOM_CHALLENGE_SIZE].copy_from_slice(&pair_state.pair_rands);
            pkt_read_rsp.att_read_rsp_mut().value[9..9 + RANDOM_CHALLENGE_SIZE].copy_from_slice(&pair_state.pair_work[0..RANDOM_CHALLENGE_SIZE]);

            // Prepare work buffer again for session key derivation
            for index in 0..KEY_SIZE {
                pair_state.pair_work[index] = pair_state.pair_nn[index] ^ pair_state.pair_pass[index];
            }

            // Derive final session key using both random values and credential material
            let pair_randm = pair_state.pair_randm;
            let pair_rands = pair_state.pair_rands;
            pair_state.pair_sk[0..RANDOM_CHALLENGE_SIZE].copy_from_slice(&pair_randm);
            pair_state.pair_sk[RANDOM_CHALLENGE_SIZE..KEY_SIZE].copy_from_slice(&pair_rands);

            pair_state.pair_sk = aes_att_encryption(&pair_state.pair_work, &pair_state.pair_sk);

            // Set state to completed and enable encryption
            BLE_PAIR_ST.set(PairState::Completed);
            PAIR_ENC_ENABLE.set(true);
        }
    } else if BLE_PAIR_ST.get() == PairState::RequestingLtk {
        // Provide the LTK either directly or encrypted based on security mode
        
        pkt_read_rsp.head_mut().l2cap_len = RESP_WITH_KEY;
        
        if SECURITY_ENABLE.get() == false {
            // Simple mode: Send LTK directly
            pkt_read_rsp.att_read_rsp_mut().value[1..1 + KEY_SIZE].copy_from_slice(&pair_state.pair_ltk);
        } else {
            // Secure mode: Encrypt LTK with a derived key
            
            // Use master random as part of key derivation
            let pair_randm = pair_state.pair_randm;
            pair_state.pair_work[0..RANDOM_CHALLENGE_SIZE].copy_from_slice(&pair_randm);
            pair_state.pair_work[RANDOM_CHALLENGE_SIZE..KEY_SIZE].fill(0);

            // Create encryption key by XORing credentials with random material
            for index in 0..KEY_SIZE {
                pair_state.pair_sk[index] = pair_state.pair_nn[index] ^ pair_state.pair_pass[index] ^ pair_state.pair_work[index];
            }

            // Encrypt the LTK with the derived key
            let encrypted_ltk = aes_att_encryption(&pair_state.pair_sk, &pair_state.pair_ltk);
            pkt_read_rsp.att_read_rsp_mut().value[1..1 + KEY_SIZE].copy_from_slice(&encrypted_ltk);
            BLE_PAIR_ST.set(PairState::Completed);

            // Restore saved session key
            pair_state.pair_sk = pair_state.pair_sk_copy;
        }
    } else if BLE_PAIR_ST.get() == PairState::DeletePairing {
        // Remove pairing information and notify application
        
        pkt_read_rsp.head_mut().l2cap_len = HEADER_SIZE;
        BLE_PAIR_ST.set(PairState::Idle);
        
        // Enable MAC address handling for mesh if needed
        if MESH_PAIR_ENABLE.get() {
            GET_MAC_EN.set(true);
        }
        
        // Delete pairing data and notify application
        rf_link_delete_pair();
        rf_link_light_event_callback(LGT_CMD_DEL_PAIR);
    } else if BLE_PAIR_ST.get() == PairState::Init {
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

    // Calculate packet length fields based on content
    pkt_read_rsp.head_mut().rf_len = pkt_read_rsp.head_mut().l2cap_len as u8 + 0x4;
    pkt_read_rsp.head_mut().dma_len = pkt_read_rsp.head_mut().l2cap_len as u32 + 6;

    // Set first byte of value to current pairing state as a raw u8 value
    pkt_read_rsp.att_read_rsp_mut().value[0] = pair_st as u8;

    return Some(pkt_read_rsp);
}

/// Set pairing keys from provided key material
/// 
/// This function is used to set the mesh network name, password, and long-term key
/// from a provided buffer containing all three keys in sequence. After setting
/// the keys in memory, it calls pair_update_key to compute derived values.
///
/// @param key Buffer containing the mesh network name (16 bytes), password (16 bytes),
///            and long-term key (16 bytes)
pub fn pair_set_key(key: &[u8])
{
    {
        let mut pair_state = PAIR_STATE.lock();
        // Set mesh network name (limited by MAX_MESH_NAME_LEN)
        pair_state.pair_nn[0..MAX_MESH_NAME_LEN.get()].copy_from_slice(&key[0..MAX_MESH_NAME_LEN.get()]);
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
/// @return Always returns true to indicate success
pub fn pair_read(_: &Packet) -> bool
{
    PAIR_READ_PENDING.set(true);
    true
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
/// @return Always returns true to indicate the command was processed
pub fn pair_write(data: &Packet) -> bool
{
    let pktdata = unsafe { &*slice_from_raw_parts_mut(addr_of!(data.att_write().value) as *mut u8, size_of::<PacketAttValue>()) };

    // Extract the opcode and data from the packet
    let opcode = pktdata[0];
    let src = &pktdata[1..];

    let mut pair_state = PAIR_STATE.lock();

    // PAIR_OP_EXCHANGE_RANDOM: Exchange random challenge for authentication
    // Client sends its random challenge to begin the authentication process
    if opcode == PAIR_OP_EXCHANGE_RANDOM {
        // Store the master random challenge
        pair_state.pair_randm.copy_from_slice(&src[0..RANDOM_CHALLENGE_SIZE]);
        
        // Set state to await slave challenge response
        BLE_PAIR_ST.set(PairState::AwaitingRandom);
        return true;
    }

    // PAIR_OP_SET_MESH_NAME: Set mesh network name
    // Set the mesh network name, with different behavior based on security mode
    if opcode == PAIR_OP_SET_MESH_NAME {
        if SECURITY_ENABLE.get() == false {
            // Simple mode: Only allow if already logged in
            if PAIR_LOGIN_OK.get() && BLE_PAIR_ST.get() == PairState::Completed {
                // Set mesh name directly from plaintext
                pair_state.pair_nn.copy_from_slice(&src[0..KEY_SIZE]);
                // Mark pairing as in progress
                *PAIR_SETTING_FLAG.lock() = ePairState::PairSetting;
                // Advance to password reception state
                BLE_PAIR_ST.set(PairState::ReceivingMeshName);
                return true;
            }
        } else if BLE_PAIR_ST.get() == PairState::Completed {
            // Secure mode: Decrypt the mesh name using the session key
            pair_state.pair_work.copy_from_slice(&src[0..KEY_SIZE]);
            pair_state.pair_nn = aes_att_decryption(&pair_state.pair_sk, &pair_state.pair_work);

            // Validate the mesh name has a valid length (with null terminator)
            let name_len = match pair_state.pair_nn.iter().position(|r| *r == 0) {
                Some(v) => v,
                None => pair_state.pair_nn.len()
            };

            // Only accept valid mesh names
            if name_len <= MAX_MESH_NAME_LEN.get() {
                *PAIR_SETTING_FLAG.lock() = ePairState::PairSetting;
                BLE_PAIR_ST.set(PairState::ReceivingMeshName);
                return true;
            }
        }

        // Invalid mesh name or state - reset pairing
        pair_par_init();
        PAIR_ENC_ENABLE.set(false);
        return true;
    }
    
    // PAIR_OP_SET_MESH_PASSWORD: Set mesh network password
    // Set the mesh network password, with different behavior based on security mode
    if opcode == PAIR_OP_SET_MESH_PASSWORD {
        if SECURITY_ENABLE.get() {
            // Secure mode: Only allow in the correct state
            if BLE_PAIR_ST.get() != PairState::ReceivingMeshName {
                pair_par_init();
                PAIR_ENC_ENABLE.set(false);
                return true;
            }
            
            // Decrypt the password using the session key
            pair_state.pair_work.copy_from_slice(&src[0..KEY_SIZE]);
            pair_state.pair_pass = aes_att_decryption(&pair_state.pair_sk, &pair_state.pair_work);
        } else {
            // Simple mode: Only allow if already logged in
            if !PAIR_LOGIN_OK.get() || BLE_PAIR_ST.get() != PairState::ReceivingMeshName {
                pair_par_init();
                PAIR_ENC_ENABLE.set(false);
                return true;
            }

            // Set password directly from plaintext
            pair_state.pair_pass.copy_from_slice(&src[0..KEY_SIZE]);
        }

        // Advance to LTK reception state
        BLE_PAIR_ST.set(PairState::ReceivingMeshPassword);
        
        // Validate that password and name are not identical (security check)
        if (if pair_state.pair_nn == pair_state.pair_pass { usize::MAX } else { 0 }) == 0 {
            return true;
        }

        // Password identical to name - security violation, reset pairing
        pair_par_init();
        PAIR_ENC_ENABLE.set(false);
        return true;
    }
    
    // PAIR_OP_SET_MESH_LTK: Set long-term key for mesh encryption
    // Set the long-term encryption key, with different behavior based on security mode
    if opcode == PAIR_OP_SET_MESH_LTK {
        // Mesh flag position in extended data
        const MESH_FLAG_OFFSET: usize = 0x11;
        // Minimum packet length for mesh flag
        const MIN_PACKET_LEN_WITH_MESH: u16 = 0x14;
        
        if SECURITY_ENABLE.get() {
            // Secure mode: Only allow in the correct state
            if BLE_PAIR_ST.get() == PairState::ReceivingMeshPassword {
                // Receive encrypted LTK and decrypt it
                pair_state.pair_work.copy_from_slice(&src[0..KEY_SIZE]);
                BLE_PAIR_ST.set(PairState::ReceivingMeshLtk);

                // Special handling for mesh network - check if mesh flag is set
                if MESH_PAIR_ENABLE.get() && MIN_PACKET_LEN_WITH_MESH < data.head().l2cap_len && pktdata[MESH_FLAG_OFFSET] != 0 {
                    // This is a mesh LTK - decrypt and prepare for mesh mode
                    pair_state.pair_ltk_mesh = aes_att_decryption(&pair_state.pair_sk, &pair_state.pair_work);
                    *PAIR_SETTING_FLAG.lock() = ePairState::PairSetMeshTxStart;
                    return true;
                }

                // Standard LTK - decrypt, save to flash, and notify application
                pair_state.pair_ltk = aes_att_decryption(&pair_state.pair_sk, &pair_state.pair_work);
                pair_save_key();
                *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
                rf_link_light_event_callback(LGT_CMD_PAIR_OK);
                return true;
            }
        } else if PAIR_LOGIN_OK.get() && BLE_PAIR_ST.get() == PairState::ReceivingMeshPassword {
            // Simple mode: Only allow if already logged in
            
            // Clear existing LTK
            pair_state.pair_ltk.fill(0);

            // Advance to next state
            BLE_PAIR_ST.set(PairState::ReceivingMeshLtk);
            
            // Special handling for mesh network - check if mesh flag is set
            if MESH_PAIR_ENABLE.get() && MIN_PACKET_LEN_WITH_MESH < data.head().l2cap_len && (((pktdata[MESH_FLAG_OFFSET] as u32) << 0x1f) as i32) < 0 {
                // This is a mesh LTK - store and prepare for mesh mode
                pair_state.pair_ltk_mesh.copy_from_slice(&src[0..KEY_SIZE]);
                *PAIR_SETTING_FLAG.lock() = ePairState::PairSetMeshTxStart;
                return true;
            }

            // Standard LTK - store, save to flash, and notify application
            pair_state.pair_ltk.copy_from_slice(&src[0..KEY_SIZE]);
            pair_save_key();
            *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
            rf_link_light_event_callback(LGT_CMD_PAIR_OK);
            return true;
        }
        
        // Invalid state - reset pairing
        pair_par_init();
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
        PAIR_ENC_ENABLE.set(false);
        return true;
    }

    // Determine if we're handling GET_LTK or VERIFY_CREDENTIALS
    let mut is_get_ltk = if opcode == PAIR_OP_GET_MESH_LTK { u8::MAX } else { 0 };
    let is_verify = if opcode == PAIR_OP_VERIFY_CREDENTIALS { u8::MAX } else { 0 };
    
    // Check for maintenance opcodes
    if is_verify == 0 {
        if is_get_ltk == 0 {
            // PAIR_OP_RESET_MESH: Reset mesh configuration to defaults
            if opcode == PAIR_OP_RESET_MESH {
                // Must track state transitions (no action here yet)
                is_get_ltk = 0;
            } else if opcode == PAIR_OP_DELETE_PAIRING {
                // PAIR_OP_DELETE_PAIRING: Delete all pairing information
                // Delete pairing information and reset
                PAIR_ENC_ENABLE.set(false);
                BLE_PAIR_ST.set(PairState::Idle);
                return true;
            }
            return true;
        }
    } else if is_get_ltk != 0 {
        // Save current session key for later restoration
        pair_state.pair_sk_copy = pair_state.pair_sk;
    }

    // Common code for both PAIR_OP_GET_MESH_LTK and PAIR_OP_VERIFY_CREDENTIALS
    // Both operations require credential verification
    
    // Store the master random challenge
    pair_state.pair_randm.copy_from_slice(&src[0..RANDOM_CHALLENGE_SIZE]);
    let pair_randm = pair_state.pair_randm;
    
    // Generate session key from master random
    pair_state.pair_sk[0..RANDOM_CHALLENGE_SIZE].copy_from_slice(&pair_randm);
    pair_state.pair_sk[RANDOM_CHALLENGE_SIZE..KEY_SIZE].fill(0);

    // Disable encryption during verification
    PAIR_ENC_ENABLE.set(false);

    // Create verification material by XORing network name and password
    for index in 0..KEY_SIZE {
        pair_state.pair_work[index] = pair_state.pair_pass[index] ^ pair_state.pair_nn[index];
    }

    // Verify the client's proof of knowledge based on security mode
    let index = if SECURITY_ENABLE.get() == false {
        // Simple mode: Direct comparison of XORed credentials
        if pair_state.pair_work == src[0..KEY_SIZE] { u8::MAX } else { 0 }
    } else {
        // Position of client proof in the packet
        const CLIENT_PROOF_OFFSET: usize = 9;
        
        // Secure mode: Encrypt the verification material and compare with client proof
        pair_state.pair_work = aes_att_encryption(&pair_state.pair_sk, &pair_state.pair_work);
        if &pair_state.pair_work[0..RANDOM_CHALLENGE_SIZE] == &pktdata[CLIENT_PROOF_OFFSET..CLIENT_PROOF_OFFSET + RANDOM_CHALLENGE_SIZE] { u8::MAX } else { 0 }
    };
    
    // Process verification result
    if is_verify != 0 || PAIR_LOGIN_OK.get() {
        if index != 0 {
            // Verification succeeded - determine next action
            if opcode == PAIR_OP_GET_MESH_LTK {
                // Client verified, proceed to send the LTK
                BLE_PAIR_ST.set(PairState::RequestingLtk);
                return true;
            }
            
            // Handle credential verification
            if is_verify == 0 {
                // Something unusual happened - verify flag is wrong
                BLE_PAIR_ST.set(PairState::DeletePairing);
                return true;
            }
            
            // Verification successful - mark as logged in
            PAIR_LOGIN_OK.set(true);
            BLE_PAIR_ST.set(PairState::SessionKeyExchange);
            return true;
        }
        
        // Verification failed
        if is_verify != 0 {
            // For verification, clear login status
            PAIR_LOGIN_OK.set(false);
        }
    }
    
    // Reset pairing on failure or completion
    pair_par_init();
    return true;
}

#[cfg(test)]
mod tests {
    use super::*;
    use mry::{self, Any};
    use crate::sdk::mcu::crypto::mock_aes_att_decryption_packet;
    use core::mem::size_of;
    use mry::send_wrapper::SendWrapper;
    use crate::sdk::packet_types::{PacketL2capData, PacketLlApp};

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
            }
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
    #[mry::lock(aes_att_decryption_packet)]
    fn test_pair_dec_packet_security_disabled() {
        // Arrange
        SECURITY_ENABLE.set(false);
        let mut packet = create_test_packet();
        
        // Act
        let result = pair_dec_packet(&mut packet);
        
        // Assert
        assert_eq!(result, false);
        mock_aes_att_decryption_packet(Any, Any, Any, Any).assert_called(0);
    }
    
    #[test]
    #[mry::lock(aes_att_decryption_packet)]
    fn test_pair_dec_packet_security_enabled_success() {
        // Arrange
        setup_pair_state();
        let mut packet = create_test_packet();
        
        // Mock successful decryption with Any for all arguments
        mock_aes_att_decryption_packet(Any, Any, Any, Any)
            .returns(true);
        
        // Act
        let result = pair_dec_packet(&mut packet);
        
        // Assert
        assert_eq!(result, true);
        
        // Verify the IV was updated with sequence number
        let pair_ivm = PAIR_IVM.lock();
        assert_eq!(pair_ivm[5], 0x01);
        assert_eq!(pair_ivm[6], 0x02);
        assert_eq!(pair_ivm[7], 0x03);
        
        // Verify mock was called with correct parameters
        mock_aes_att_decryption_packet(Any, Any, Any, Any).assert_called(1);
    }
    
    #[test]
    #[mry::lock(aes_att_decryption_packet)]
    fn test_pair_dec_packet_security_enabled_failure() {
        // Arrange
        setup_pair_state();
        let mut packet = create_test_packet();
        
        // Mock failed decryption with Any for all arguments
        mock_aes_att_decryption_packet(Any, Any, Any, Any)
            .returns(false);
        
        // Act
        let result = pair_dec_packet(&mut packet);
        
        // Assert
        assert_eq!(result, false);
        mock_aes_att_decryption_packet(Any, Any, Any, Any).assert_called(1);
    }
    
    #[test]
    #[mry::lock(aes_att_decryption_packet)]
    fn test_pair_dec_packet_iv_updates() {
        // Arrange
        setup_pair_state();
        let mut packet = create_test_packet();
        packet.att_write_mut().value.sno = [0xF1, 0xF2, 0xF3]; // Different sequence number
        
        // Mock decryption with Any for all arguments
        mock_aes_att_decryption_packet(Any, Any, Any, Any)
            .returns(true);
        
        // Act
        let result = pair_dec_packet(&mut packet);
        
        // Assert
        assert_eq!(result, true);
        
        // Verify IV was updated with new sequence number
        let pair_ivm = PAIR_IVM.lock();
        assert_eq!(pair_ivm[5], 0xF1);
        assert_eq!(pair_ivm[6], 0xF2);
        assert_eq!(pair_ivm[7], 0xF3);
        
        mock_aes_att_decryption_packet(Any, Any, Any, Any).assert_called(1);
    }
    
    #[test]
    #[mry::lock(aes_att_decryption_packet)]
    fn test_pair_dec_packet_parameter_passing() {
        // Arrange
        setup_pair_state();
        let mut packet = create_test_packet();
        
        // Setup expectation with parameter validation
        mock_aes_att_decryption_packet(Any, Any, Any, Any)
            .returns_with(move |sk: Vec<u8>, iv: Vec<u8>, src: Vec<u8>, data: Vec<u8>| -> bool {
                // Validate session key
                assert_eq!(sk, &[0xBB; 16]);
                
                // Validate IV (should include the sequence number from packet)
                assert_eq!(iv[5], 0x01);
                assert_eq!(iv[6], 0x02);
                assert_eq!(iv[7], 0x03);
                
                // Validate source address from packet
                assert_eq!(*src, [0x34, 0x12]); // Little endian representation of 0x1234
                
                // Payload data length check (not checking contents due to complexity)
                assert_eq!(data.len(), packet.head().l2cap_len as usize - 8);
                
                true
            });
        
        // Act
        let result = pair_dec_packet(&mut packet);
        
        // Assert
        assert_eq!(result, true);
        mock_aes_att_decryption_packet(Any, Any, Any, Any).assert_called(1);
    }

    use crate::sdk::mcu::crypto::mock_aes_att_encryption_packet;
    use crate::sdk::mcu::register::mock_read_reg_system_tick;

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
            }
        };
        
        // Set up source as a u16 value (little endian in memory)
        packet.ll_app_mut().value.src = 0x1234;
        
        packet
    }
    
    #[test]
    #[mry::lock(aes_att_encryption_packet)]
    fn test_pair_enc_packet_security_disabled() {
        // Arrange
        SECURITY_ENABLE.set(false);
        let mut packet = create_test_ll_packet();
        
        // Act
        pair_enc_packet(&mut packet);
        
        // Assert
        mock_aes_att_encryption_packet(Any, Any, Any, Any).assert_called(0);
    }
    
    #[test]
    #[mry::lock(aes_att_encryption_packet, read_reg_system_tick)]
    fn test_pair_enc_packet_security_enabled() {
        // Arrange
        setup_pair_state();
        let mut packet = create_test_ll_packet();
        
        // Mock system tick to return a predictable value
        let test_tick: u32 = 0x123456;
        mock_read_reg_system_tick().returns(test_tick);
        
        // Mock encryption function
        mock_aes_att_encryption_packet(Any, Any, Any, Any).returns(());
        
        // Act
        pair_enc_packet(&mut packet);
        
        // Assert
        // Verify sequence number was set from system tick
        assert_eq!(packet.ll_app().value.sno[0], (test_tick & 0xFF) as u8);        // 0x56
        assert_eq!(packet.ll_app().value.sno[1], ((test_tick >> 8) & 0xFF) as u8); // 0x34
        assert_eq!(packet.ll_app().value.sno[2], ((test_tick >> 16) & 0xFF) as u8); // 0x12
        
        // Verify encryption function was called
        mock_aes_att_encryption_packet(Any, Any, Any, Any).assert_called(1);
    }
    
    #[test]
    #[mry::lock(aes_att_encryption_packet, read_reg_system_tick)]
    fn test_pair_enc_packet_iv_updates() {
        // Arrange
        setup_pair_state();
        let mut packet = create_test_ll_packet();
        
        // Set a known system tick value
        let test_tick: u32 = 0xABCDEF;
        mock_read_reg_system_tick().returns(test_tick);
        
        // Mock encryption function
        mock_aes_att_encryption_packet(Any, Any, Any, Any).returns(());
        
        // Fill PAIR_IVS with recognizable pattern for testing
        PAIR_IVS.lock().fill(0xAA);
        
        // Act
        pair_enc_packet(&mut packet);
        
        // Assert
        // Verify IV was updated with sequence number
        let pair_ivs = PAIR_IVS.lock();
        assert_eq!(pair_ivs[3], (test_tick & 0xFF) as u8);        // 0xEF
        assert_eq!(pair_ivs[4], ((test_tick >> 8) & 0xFF) as u8); // 0xCD
        assert_eq!(pair_ivs[5], ((test_tick >> 16) & 0xFF) as u8); // 0xAB
        
        // Verify IV was updated with source address
        assert_eq!(pair_ivs[6], 0x34); // Low byte of 0x1234
        assert_eq!(pair_ivs[7], 0x12); // High byte of 0x1234
        
        mock_aes_att_encryption_packet(Any, Any, Any, Any).assert_called(1);
    }
    
    #[test]
    #[mry::lock(aes_att_encryption_packet, read_reg_system_tick)]
    fn test_pair_enc_packet_parameter_passing() {
        // Arrange
        setup_pair_state();
        let mut packet = create_test_ll_packet();
        
        // Set a known system tick value
        let test_tick: u32 = 0x123456;
        mock_read_reg_system_tick().returns(test_tick);
        
        // Set recognizable session key pattern
        PAIR_STATE.lock().pair_sk.fill(0xCC);
        
        // Setup encryption mock with parameter validation
        mock_aes_att_encryption_packet(Any, Any, Any, Any)
            .returns_with(move |sk: Vec<u8>, iv: Vec<u8>, dst: Vec<u8>, data: Vec<u8>| -> () {
                // Validate session key
                assert_eq!(sk, &[0xCC; 16]);
                
                // Validate IV (should include the sequence number and source from packet)
                assert_eq!(iv[3], 0x56); // Low byte of tick
                assert_eq!(iv[4], 0x34); // Middle byte of tick
                assert_eq!(iv[5], 0x12); // High byte of tick
                assert_eq!(iv[6], 0x34); // Low byte of source
                assert_eq!(iv[7], 0x12); // High byte of source
                
                // Validate destination address buffer
                assert_eq!(dst.len(), 2);
                
                // Validate data length (l2cap_len - 10)
                assert_eq!(data.len(), (packet.head().l2cap_len - 10) as usize);
            });
        
        // Act
        pair_enc_packet(&mut packet);
        
        // Assert
        mock_aes_att_encryption_packet(Any, Any, Any, Any).assert_called(1);
    }
    
    #[test]
    #[mry::lock(aes_att_encryption_packet)]
    fn test_pair_enc_packet_wrong_channel() {
        // Arrange
        setup_pair_state();
        let mut packet = create_test_ll_packet();
        packet.head_mut().chan_id = 5; // Different channel than required (4)
        
        // Mock encryption function
        mock_aes_att_encryption_packet(Any, Any, Any, Any).returns(());
        
        // Act
        pair_enc_packet(&mut packet);
        
        // Assert
        mock_aes_att_encryption_packet(Any, Any, Any, Any).assert_called(0);
    }
    
    #[test]
    #[mry::lock(aes_att_encryption_packet)]
    fn test_pair_enc_packet_wrong_opcode() {
        // Arrange
        setup_pair_state();
        let mut packet = create_test_ll_packet();
        packet.ll_app_mut().opcode = 0x1c; // Different opcode than required (0x1b)
        
        // Mock encryption function
        mock_aes_att_encryption_packet(Any, Any, Any, Any).returns(());
        
        // Act
        pair_enc_packet(&mut packet);
        
        // Assert
        mock_aes_att_encryption_packet(Any, Any, Any, Any).assert_called(0);
    }
    
    #[test]
    #[mry::lock(aes_att_encryption_packet)]
    fn test_pair_enc_packet_wrong_handle() {
        // Arrange
        setup_pair_state();
        let mut packet = create_test_ll_packet();
        packet.ll_app_mut().handle = 0x13; // Different handle than required (0x12)
        
        // Mock encryption function
        mock_aes_att_encryption_packet(Any, Any, Any, Any).returns(());
        
        // Act
        pair_enc_packet(&mut packet);
        
        // Assert
        mock_aes_att_encryption_packet(Any, Any, Any, Any).assert_called(0);
    }

    use crate::sdk::packet_types::{MeshPkt, PacketL2capHead};

    // Helper function to create a test mesh packet
    fn create_test_mesh_packet(is_broadcast: bool) -> Packet {
        let mut packet = Packet {
            mesh: MeshPkt {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: PACKET_TYPE_ENCRYPTED, // Set encrypted flag
                    rf_len: 30,
                    l2cap_len: 20,
                    chan_id: if is_broadcast { MESH_BROADCAST_CHANNEL } else { 0x0001 },
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
            }
        };
        packet
    }
    
    #[test]
    #[mry::lock(aes_att_decryption_packet)]
    fn test_pair_dec_packet_mesh_security_disabled() {
        // Arrange
        SECURITY_ENABLE.set(false);
        let mut packet = create_test_mesh_packet(false);
        
        // Act
        let result = pair_dec_packet_mesh(&mut packet);
        
        // Assert
        assert_eq!(result, true); // Should return true when security is disabled
        mock_aes_att_decryption_packet(Any, Any, Any, Any).assert_called(0);
    }
    
    #[test]
    #[mry::lock(aes_att_decryption_packet)]
    fn test_pair_dec_packet_mesh_no_encryption_flag() {
        // Arrange
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(false);
        packet.head_mut()._type = 0; // Clear encryption flag
        
        // Act
        let result = pair_dec_packet_mesh(&mut packet);
        
        // Assert
        assert_eq!(result, false); // Should return false when encryption flag is not set
        mock_aes_att_decryption_packet(Any, Any, Any, Any).assert_called(0);
    }
    
    #[test]
    #[mry::lock(aes_att_decryption_packet)]
    fn test_pair_dec_packet_mesh_invalid_length() {
        // Arrange
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(false);
        packet.head_mut().rf_len = 0x26; // This makes rf_len - 0x12 > 0x13
        
        // Act
        let result = pair_dec_packet_mesh(&mut packet);
        
        // Assert
        assert_eq!(result, false); // Should return false for invalid length
        mock_aes_att_decryption_packet(Any, Any, Any, Any).assert_called(0);
    }
    
    #[test]
    #[mry::lock(aes_att_decryption_packet)]
    fn test_pair_dec_packet_mesh_broadcast_success() {
        // Arrange
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(true); // Create broadcast packet
        
        // Set up pairing state with LTK
        let mut pair_state = PAIR_STATE.lock();
        pair_state.pair_ltk.fill(0xDD); // Recognizable pattern

        // Mock successful decryption
        mock_aes_att_decryption_packet(Any, Any, Any, Any).returns_with(move |sk: Vec<u8>, iv: Vec<u8>, src: Vec<u8>, data: Vec<u8>| -> bool {
            // Check LTK was passed correctly
            assert_eq!(sk, &[0xDD; 16]);
            // Check IV and src aren't empty (detailed checking is complex due to pointers)
            assert_eq!(iv.is_empty(), false);
            assert_eq!(src.is_empty(), false);
            assert_eq!(data.is_empty(), false);
            // Check data length for broadcast packets
            assert_eq!(data.len(), 0x1c);
            
            true
        });
        
        // Act
        drop(pair_state); // Release the lock before calling the function
        let result = pair_dec_packet_mesh(&mut packet);
        
        // Assert
        assert_eq!(result, true);
        
        // Verify mock was called with correct parameters
        mock_aes_att_decryption_packet(Any, Any, Any, Any).assert_called(1)
    }
    
    #[test]
    #[mry::lock(aes_att_decryption_packet)]
    fn test_pair_dec_packet_mesh_broadcast_failure() {
        // Arrange
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(true); // Create broadcast packet
        
        // Set up pairing state with LTK
        PAIR_STATE.lock().pair_ltk.fill(0xDD);
        
        // Mock failed decryption
        mock_aes_att_decryption_packet(Any, Any, Any, Any).returns(false);
        
        // Act
        let result = pair_dec_packet_mesh(&mut packet);
        
        // Assert
        assert_eq!(result, false);
        mock_aes_att_decryption_packet(Any, Any, Any, Any).assert_called(1);
    }
    
    #[test]
    #[mry::lock(aes_att_decryption_packet)]
    fn test_pair_dec_packet_mesh_direct_success() {
        // Arrange
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(false); // Create direct packet
        packet.head_mut().rf_len = 0x20; // Set suitable RF length for testing
        
        // Set up pairing state with LTK
        PAIR_STATE.lock().pair_ltk.fill(0xEE);
        
        // Mock successful decryption
        mock_aes_att_decryption_packet(Any, Any, Any, Any).returns_with(move |sk: Vec<u8>, iv: Vec<u8>, src: Vec<u8>, data: Vec<u8>| -> bool {
            // Check LTK was passed correctly
            assert_eq!(sk, &[0xEE; 16]);
            // Check IV and src aren't empty (detailed checking is complex due to pointers)
            assert_eq!(iv.is_empty(), false);
            assert_eq!(src.is_empty(), false);
            assert_eq!(data.is_empty(), false);
            // Check data length for broadcast packets
            assert_eq!(data.len(), 0x20 - 0x12);

            true
        });
        
        // Act
        let result = pair_dec_packet_mesh(&mut packet);
        
        // Assert
        assert_eq!(result, true);
        
        // Verify mock was called with correct parameters for direct packets
        mock_aes_att_decryption_packet(Any, Any, Any, Any).assert_called(1)
    }
    
    #[test]
    #[mry::lock(aes_att_decryption_packet)]
    fn test_pair_dec_packet_mesh_direct_failure() {
        // Arrange
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(false); // Create direct packet
        
        // Set up pairing state with LTK
        PAIR_STATE.lock().pair_ltk.fill(0xEE);
        
        // Mock failed decryption
        mock_aes_att_decryption_packet(Any, Any, Any, Any).returns(false);
        
        // Act
        let result = pair_dec_packet_mesh(&mut packet);
        
        // Assert
        assert_eq!(result, false);
        mock_aes_att_decryption_packet(Any, Any, Any, Any).assert_called(1);
    }

    #[test]
    #[mry::lock(aes_att_encryption_packet)]
    fn test_pair_enc_packet_mesh_security_disabled() {
        // Arrange
        SECURITY_ENABLE.set(false);
        let mut packet = create_test_mesh_packet(false);
        
        // Act
        let result = pair_enc_packet_mesh(&mut packet);
        
        // Assert
        assert_eq!(result, false); // Should return false when security is disabled
        mock_aes_att_encryption_packet(Any, Any, Any, Any).assert_called(0);
    }
    
    #[test]
    #[mry::lock(aes_att_encryption_packet)]
    fn test_pair_enc_packet_mesh_broadcast() {
        // Arrange
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(true); // Create broadcast packet
        
        // Set up pairing state with LTK
        PAIR_STATE.lock().pair_ltk.fill(0xDD); // Recognizable pattern
        
        // Mock encryption function
        mock_aes_att_encryption_packet(Any, Any, Any, Any).returns(());
        
        // Act
        let result = pair_enc_packet_mesh(&mut packet);
        
        // Assert
        assert_eq!(result, true);
        mock_aes_att_encryption_packet(Any, Any, Any, Any).assert_called(1);
    }
    
    #[test]
    #[mry::lock(aes_att_encryption_packet)]
    fn test_pair_enc_packet_mesh_broadcast_parameter_passing() {
        // Arrange
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(true); // Create broadcast packet
        
        // Set up pairing state with LTK
        PAIR_STATE.lock().pair_ltk.fill(0xDD); // Recognizable pattern
        
        // Mock encryption function with parameter validation
        mock_aes_att_encryption_packet(Any, Any, Any, Any)
            .returns_with(move |sk: Vec<u8>, iv: Vec<u8>, dst: Vec<u8>, data: Vec<u8>| -> () {
                // Check LTK was passed correctly
                assert_eq!(sk, &[0xDD; 16]);
                
                // Check IV is not empty and has 8 bytes (detailed checking is complex due to pointers)
                assert_eq!(iv.len(), 8);
                
                // Check destination is 2 bytes for broadcast packets
                assert_eq!(dst.len(), 2);
                
                // Check data length for broadcast packets
                assert_eq!(data.len(), 0x1c);
            });
        
        // Act
        let result = pair_enc_packet_mesh(&mut packet);
        
        // Assert
        assert_eq!(result, true);
        mock_aes_att_encryption_packet(Any, Any, Any, Any).assert_called(1);
    }
    
    #[test]
    #[mry::lock(aes_att_encryption_packet)]
    fn test_pair_enc_packet_mesh_direct() {
        // Arrange
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(false); // Create direct packet
        packet.head_mut().rf_len = 0x20; // Set specific RF length for testing
        
        // Set up pairing state with LTK
        PAIR_STATE.lock().pair_ltk.fill(0xEE); // Different recognizable pattern
        
        // Mock encryption function
        mock_aes_att_encryption_packet(Any, Any, Any, Any).returns(());
        
        // Act
        let result = pair_enc_packet_mesh(&mut packet);
        
        // Assert
        assert_eq!(result, true);
        mock_aes_att_encryption_packet(Any, Any, Any, Any).assert_called(1);
    }
    
    #[test]
    #[mry::lock(aes_att_encryption_packet)]
    fn test_pair_enc_packet_mesh_direct_parameter_passing() {
        // Arrange
        SECURITY_ENABLE.set(true);
        let mut packet = create_test_mesh_packet(false); // Create direct packet
        packet.head_mut().rf_len = 0x22; // Set specific RF length for testing
        
        // Set up pairing state with LTK
        PAIR_STATE.lock().pair_ltk.fill(0xEE); // Different recognizable pattern
        
        // Mock encryption function with parameter validation
        mock_aes_att_encryption_packet(Any, Any, Any, Any)
            .returns_with(move |sk: Vec<u8>, iv: Vec<u8>, dst: Vec<u8>, data: Vec<u8>| -> () {
                // Check LTK was passed correctly
                assert_eq!(sk, &[0xEE; 16]);
                
                // Check IV is not empty and has 8 bytes
                assert_eq!(iv.len(), 8);
                
                // Check destination is 4 bytes for direct packets
                assert_eq!(dst.len(), 4);
                
                // Check data length for direct packets: rf_len - 0x12
                assert_eq!(data.len(), 0x22 - 0x12);
            });
        
        // Act
        let result = pair_enc_packet_mesh(&mut packet);
        
        // Assert
        assert_eq!(result, true);
        mock_aes_att_encryption_packet(Any, Any, Any, Any).assert_called(1);
    }
    
    #[test]
    #[mry::lock(aes_att_encryption_packet)]
    fn test_pair_enc_packet_mesh_varying_rf_lengths() {
        // Test with different RF lengths to verify calculation of payload size
        
        // Test values: rf_len, expected payload size
        let test_cases = [(0x15, 0x03), (0x18, 0x06), (0x1F, 0x0D)];

        // Use an Arc<Mutex> to share the expected size between test iterations and mock callback
        let expected_payload_size = std::sync::Arc::new(std::sync::Mutex::new(0));
        let expected_payload_size_clone = expected_payload_size.clone();

        // Mock encryption function with parameter validation
        mock_aes_att_encryption_packet(Any, Any, Any, Any)
            .returns_with(move |_: Vec<u8>, _: Vec<u8>, _: Vec<u8>, data: Vec<u8>| -> () {
                // Check data length calculation: rf_len - 0x12
                let expected = *expected_payload_size_clone.lock().unwrap();
                assert_eq!(data.len(), expected);
            });

        for (rf_len, expected_size) in test_cases {
            // Update the expected size for the current iteration
            *expected_payload_size.lock().unwrap() = expected_size;

            // Arrange
            SECURITY_ENABLE.set(true);
            let mut packet = create_test_mesh_packet(false); // Create direct packet
            packet.head_mut().rf_len = rf_len; // Set specific RF length for testing
            
            // Act
            let result = pair_enc_packet_mesh(&mut packet);
            
            // Assert
            assert_eq!(result, true);
        }

        mock_aes_att_encryption_packet(Any, Any, Any, Any).assert_called(3);
    }
}