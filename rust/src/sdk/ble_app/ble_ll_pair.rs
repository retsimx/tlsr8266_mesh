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
    if SECURITY_ENABLE.get() {
        let mut pair_ivm = PAIR_IVM.lock();
        // Extract sequence number from packet as part of IV
        pair_ivm[5..5 + 3].copy_from_slice(&ps.att_write().value.sno);

        // Extract source address from packet
        let src = ps.att_write().value.src;

        let l2len = ps.head().l2cap_len as usize;

        let data = slice_from_raw_parts_mut(addr_of_mut!(ps.att_write_mut().value.dst) as *mut u8, l2len - 8);
        
        // Perform decryption using AES-CCM
        unsafe {
            aes_att_decryption_packet(
                &PAIR_STATE.lock().pair_sk,
                &*pair_ivm,
                &src,
                &mut *data,
            )
        }
    } else {
        false
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
    if SECURITY_ENABLE.get() && ps.head().chan_id == 4 && ps.ll_app().opcode == 0x1b && ps.ll_app().handle == 0x12 {
        // Create a sequence number from system tick to prevent replay attacks
        let tick = read_reg_system_tick();
        ps.ll_app_mut().value.sno[0] = tick as u8;
        ps.ll_app_mut().value.sno[1] = (tick >> 8) as u8;
        ps.ll_app_mut().value.sno[2] = (tick >> 16) as u8;

        let mut pair_ivs = PAIR_IVS.lock();

        // Update initialization vector with sequence number and addressing information
        pair_ivs[3..3 + 3].copy_from_slice(&ps.ll_app().value.sno);
        pair_ivs[6] = ps.ll_app().value.src as u8;
        pair_ivs[7] = (ps.ll_app().value.src >> 8) as u8;

        // Perform encryption using AES-CCM
        aes_att_encryption_packet(
            &PAIR_STATE.lock().pair_sk,
            &*pair_ivs,
            unsafe { slice::from_raw_parts_mut(addr_of!(ps.ll_app().value.dst) as *mut u8, 2) },
            unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.ll_app_mut().value.op), ((ps.head().l2cap_len & 0xff) - 10) as usize) },
        );
    }
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
    let mut ltk = [0u8; 16];

    // If security is not enabled, we consider packets as already decrypted
    if !SECURITY_ENABLE.get() {
        return true;
    }

    // Check if packet has the mesh encryption flag set
    if ps.head()._type & PACKET_TYPE_ENCRYPTED == 0 {
        return false;
    }

    // Verify the packet length is within valid range for decryption
    let rf_len = ps.head().rf_len;
    if 0x13 < rf_len - 0x12 {
        return false;
    }

    // Get the long-term key for mesh decryption
    ltk.copy_from_slice(&PAIR_STATE.lock().pair_ltk);

    // Handle two types of mesh packets with different structures:
    // 1. Broadcast packets (channel ID 0xFFFF)
    // 2. Direct mesh packets (other channel IDs)
    if ps.head().chan_id == MESH_BROADCAST_CHANNEL {
        aes_att_decryption_packet(
            &ltk,
            unsafe { slice::from_raw_parts(addr_of!(ps.head().rf_len), 8) },
            unsafe { slice::from_raw_parts(addr_of!(ps.mesh().internal_par2[1]), 2) },
            unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().sno) as *mut u8, 0x1c) },
        )
    } else {
        aes_att_decryption_packet(
            &ltk,
            unsafe { slice::from_raw_parts(addr_of!(ps.mesh().handle1), 8) },
            unsafe { slice::from_raw_parts((addr_of!(ps.mesh().sno) as u32 + (rf_len as u32 - 0xb)) as *const u8, 4) },
            unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().op), rf_len as usize - 0x12) },
        )
    }
}

/// Encrypt a mesh network packet using the long-term key (LTK)
/// 
/// This function handles encryption of mesh packets before transmission.
/// Similar to decryption, it uses the long-term key rather than the session key.
///
/// @param ps The mesh packet to encrypt
/// @return true if encryption was successful, false otherwise
pub fn pair_enc_packet_mesh(ps: &mut Packet) -> bool
{
    if SECURITY_ENABLE.get() {
        let mut pair_state = PAIR_STATE.lock();

        // Handle two types of mesh packets with different structures
        if ps.head().chan_id == MESH_BROADCAST_CHANNEL {
            // Broadcast packet encryption
            aes_att_encryption_packet(
                &pair_state.pair_ltk,
                unsafe { slice::from_raw_parts(addr_of!(ps.head().rf_len), 8) },
                unsafe { slice::from_raw_parts_mut(addr_of!(ps.mesh().internal_par2[1]) as *mut u8, 2) },
                unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().sno) as *mut u8, 0x1c) },
            );

            return true;
        } else {
            // Direct mesh packet encryption
            aes_att_encryption_packet(
                &pair_state.pair_ltk,
                unsafe { slice::from_raw_parts(addr_of!(ps.mesh().handle1), 8) },
                unsafe { slice::from_raw_parts_mut((addr_of!(ps.mesh().sno) as u32 + (ps.head().rf_len as u32 - 0xb)) as *mut u8, 4) },
                unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().op), ps.head().rf_len as usize - 0x12) },
            );

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
    encode_password(&mut pass);

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
            aes_att_encryption(&pair_state.pair_sk, &pair_state.pair_work, &mut pkt_read_rsp.att_read_rsp_mut().value[1..]);
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
            aes_att_encryption(&pair_state.pair_randm.clone(), &pair_state.pair_rands.clone(), &mut pair_state.pair_sk);
            
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
            aes_att_encryption(&pair_state.pair_sk.clone(), &pair_state.pair_work.clone(), &mut pair_state.pair_work);

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

            aes_att_encryption(&pair_state.pair_work.clone(), &pair_state.pair_sk.clone(), &mut pair_state.pair_sk);

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
            aes_att_encryption(&pair_state.pair_sk, &pair_state.pair_ltk, &mut pkt_read_rsp.att_read_rsp_mut().value[1..1 + KEY_SIZE]);
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
            aes_att_decryption(&pair_state.pair_sk.clone(), &pair_state.pair_work.clone(), &mut pair_state.pair_nn);

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
            aes_att_decryption(&pair_state.pair_sk.clone(), &pair_state.pair_work.clone(), &mut pair_state.pair_pass);
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
                    aes_att_decryption(&pair_state.pair_sk.clone(), &pair_state.pair_work.clone(), &mut pair_state.pair_ltk_mesh);
                    *PAIR_SETTING_FLAG.lock() = ePairState::PairSetMeshTxStart;
                    return true;
                }

                // Standard LTK - decrypt, save to flash, and notify application
                aes_att_decryption(&pair_state.pair_sk.clone(), &pair_state.pair_work.clone(), &mut pair_state.pair_ltk);
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
        aes_att_encryption(&pair_state.pair_sk.clone(), &pair_state.pair_work.clone(), &mut pair_state.pair_work);
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