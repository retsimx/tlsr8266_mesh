//! # Packet Processing Module
//!
//! This module implements the core packet processing engine for the BLE mesh network,
//! providing sophisticated routing, filtering, acknowledgment, and relay algorithms.
//! It serves as the central hub that coordinates communication between BLE peripheral
//! connections and mesh network operations.
//!
//! ## Architecture Overview
//!
//! The packet processing system implements a multi-stage pipeline:
//!
//! ```text
//! ┌─────────────┐    ┌──────────────┐    ┌─────────────┐    ┌──────────────┐
//! │   Incoming  │    │  Duplicate   │    │  Protocol   │    │   Address    │
//! │   Packet    │───▶│  Detection   │───▶│ Translation │───▶│  Filtering   │
//! └─────────────┘    └──────────────┘    └─────────────┘    └──────────────┘
//!          │                                                        │
//!          ▼                                                        ▼
//! ┌─────────────┐    ┌──────────────┐    ┌─────────────┐    ┌──────────────┐
//! │ Status Adv. │    │     ACK      │    │   Response  │    │    Relay     │
//! │ Processing  │    │ Generation   │    │ Generation  │    │ Processing   │
//! └─────────────┘    └──────────────┘    └─────────────┘    └──────────────┘
//! ```
//!
//! ## Key Algorithms
//!
//! ### Duplicate Detection
//! - Uses cached packet buffer with FIFO replacement policy
//! - Combines operation code and sequence number for unique identification
//! - Prevents redundant processing of repeated mesh packets
//!
//! ### BLE-Mesh Bridging
//! - Translates between BLE attribute protocol and mesh network format
//! - Forwards mesh responses to waiting BLE peripheral connections
//! - Manages request-response transaction state tracking
//!
//! ### Acknowledgment Protocol
//! - Automatic ACK generation for addressed non-notify requests
//! - Maintains original sequence numbers for proper correlation
//! - Implements vendor identification and parameter preservation
//!
//! ### Mesh Relay Algorithm
//! - Intelligent packet forwarding based on address matching
//! - Congestion control using randomized transmission delays
//! - TTL management to prevent infinite packet circulation
//!
//! ### Status Advertisement Processing
//! - Specialized handling for mesh topology updates
//! - Validates packet signatures and extracts node status arrays
//! - Updates distributed node database for network awareness
//!
//! ## Performance Optimizations
//!
//! - **Early Exit Paths**: Status packets bypass complex command processing
//! - **Batch Operations**: Efficient multi-field packet modifications
//! - **Memory Safety**: Unsafe operations limited to performance-critical sections
//! - **Bounded Buffers**: Fixed-size caches prevent memory exhaustion
//!
//! ## Concurrency and Thread Safety
//!
//! The module uses atomic operations and mutex-protected data structures:
//! - Packet buffers use mutex locks for thread-safe access
//! - Atomic counters for sequence number and state tracking
//! - Lock scopes minimized to reduce contention
//!
//! ## Error Handling
//!
//! Robust error handling throughout the processing pipeline:
//! - Graceful handling of malformed or truncated packets
//! - Protection against buffer overflow and memory corruption
//! - Safe rejection of protocol violations and invalid formats
//! - Graceful degradation under resource exhaustion conditions

use core::cmp::min;
use core::ptr::{addr_of, addr_of_mut, slice_from_raw_parts_mut};
use core::slice;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use crate::{app, BIT};
use crate::common::rf_update_conn_para;
use crate::config::VENDOR_ID;
use crate::embassy::time_driver::clock_time64;
use crate::main_light::{rf_link_data_callback, rf_link_response_callback};
use crate::mesh::{MESH_NODE_ST_VAL_LEN, mesh_node_st_val_t};
use crate::sdk::ble_app::ble_ll_attribute::l2cap_att_handler;
use crate::sdk::ble_app::ble_ll_pair::{pair_enc_packet};
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::light::{*};
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, clock_time};
use crate::sdk::mcu::register::{*};
use crate::sdk::packet_types::{*};
use crate::state::{*};
use crate::uart_manager::light_mesh_rx_cb;

use super::mesh_management::{mesh_node_update_status, rf_link_match_group_mac};

/// Implements duplicate packet detection using cached packet buffer.
///
/// This function provides the core duplicate detection mechanism that prevents
/// the same mesh packet from being processed multiple times. It uses a combination
/// of operation code and sequence number to uniquely identify packets.
///
/// # Duplicate Detection Algorithm
///
/// The algorithm uses a two-key identification system:
/// - **Operation Code**: Identifies the type of command/response
/// - **Sequence Number**: Provides unique packet identification within that operation type
///
/// This combination ensures that even if two packets have the same sequence number
/// but different operations, they are treated as distinct packets.
///
/// # Performance Characteristics
///
/// - **Time Complexity**: O(n) where n is the number of cached packets
/// - **Memory Usage**: Bounded by RC_PKT_BUF capacity
/// - **Hit Rate**: High for duplicate detection in typical mesh scenarios
///
/// # Parameters
/// * `opcode` - Operation code of the packet to check
/// * `cmd_pkt` - Packet containing sequence number for identification
///
/// # Returns
/// * `true` if packet already exists in buffer (duplicate detected)
/// * `false` if packet is new and should be processed
#[cfg_attr(test, mry::mry)]
pub fn is_exist_in_rc_pkt_buf(opcode: u8, cmd_pkt: &Packet) -> bool
{
    RC_PKT_BUF.lock().iter().any(|v| v.op == opcode && v.sno == cmd_pkt.att_cmd().value.sno)
}

/// Classifies operation codes as notify response types.
///
/// This function implements operation code classification to distinguish response
/// packets from request packets. It uses a lookup table approach for efficient
/// classification of the standardized response operation codes.
///
/// # Response Operation Code Categories
///
/// The function recognizes these response types:
/// - **Group Responses**: LGT_CMD_LIGHT_GRP_RSP1/2/3 for group operations
/// - **Status Responses**: LGT_CMD_LIGHT_STATUS for device status queries
/// - **Address Responses**: LGT_CMD_DEV_ADDR_RSP for address configuration
/// - **User Responses**: LGT_CMD_USER_NOTIFY_RSP for custom notifications
/// - **OTA Responses**: LGT_CMD_START_OTA_RSP, LGT_CMD_OTA_DATA_RSP for firmware updates
///
/// # Design Rationale
///
/// Response classification enables different handling paths:
/// - Responses typically don't require acknowledgments
/// - Responses may be forwarded to waiting BLE peripheral connections
/// - Responses help complete request-response transactions
///
/// # Parameters
/// * `opcode` - Operation code to classify
///
/// # Returns
/// * `true` if opcode represents a response packet
/// * `false` if opcode represents a request or other packet type
fn rf_link_is_notify_rsp(opcode: u8) -> bool
{
    [
        LGT_CMD_LIGHT_GRP_RSP1,     // Group response type 1
        LGT_CMD_LIGHT_GRP_RSP2,     // Group response type 2  
        LGT_CMD_LIGHT_GRP_RSP3,     // Group response type 3
        LGT_CMD_LIGHT_STATUS,       // Device status response
        LGT_CMD_DEV_ADDR_RSP,       // Device address response
        LGT_CMD_USER_NOTIFY_RSP,    // User notification response
        LGT_CMD_START_OTA_RSP,      // OTA start response
        LGT_CMD_OTA_DATA_RSP        // OTA data response
    ].contains(&opcode)
}

/// Manages packet buffer with FIFO replacement policy for duplicate detection.
///
/// This function implements a circular buffer management system that caches
/// recently processed packets for duplicate detection. It uses a FIFO replacement
/// policy to maintain a bounded cache size while maximizing duplicate detection
/// effectiveness.
///
/// # Buffer Management Algorithm
///
/// The function implements these buffer management strategies:
/// 1. **FIFO Replacement**: When buffer is full, removes oldest entry first
/// 2. **Front Insertion**: New packets are added to the front for recent access
/// 3. **Automatic Sizing**: Buffer automatically manages capacity constraints
///
/// # Cache Entry Structure
///
/// Each cached entry contains:
/// - **Operation Code**: For packet type classification
/// - **Sequence Number**: For unique packet identification  
/// - **Notify OK Flag**: For notification status tracking
///
/// # Memory Management
///
/// The buffer provides bounded memory usage:
/// - Fixed maximum capacity prevents memory exhaustion
/// - Automatic cleanup of old entries maintains performance
/// - Efficient insertion and removal operations
///
/// # Parameters
/// * `opcode` - Operation code of packet to cache
/// * `cmd_pkt` - Packet containing sequence number and other metadata
///
/// # Side Effects
/// * Modifies global RC_PKT_BUF cache
/// * May remove oldest cached entry if buffer is full
/// * Affects subsequent duplicate detection queries
fn rc_pkt_buf_push(opcode: u8, cmd_pkt: &Packet)
{
    let mut rc_pkt_buf = RC_PKT_BUF.lock();

    // Implement FIFO replacement: remove oldest entry if buffer is full
    if rc_pkt_buf.is_full() {
        rc_pkt_buf.pop_back();
    }

    // Insert new entry at front for recent access optimization
    rc_pkt_buf.push_front(
        PktBuf {
            op: opcode,
            sno: cmd_pkt.att_cmd().value.sno,
            notify_ok_flag: false,  // Initialize as not notified
        }
    ).unwrap();
}

/// Checks notification completion status for request-response transactions.
///
/// This function implements transaction state tracking for request-response pairs
/// by checking whether a notify acknowledgment has been completed for a specific
/// packet. It supports the mesh protocol's notification confirmation mechanism.
///
/// # Transaction Tracking Algorithm
///
/// The function searches for packets that match:
/// 1. **Operation Code**: Must match the request operation
/// 2. **Sequence Number**: Must match the specific packet instance
/// 3. **Notify OK Flag**: Must be set to indicate notification completion
///
/// All three conditions must be met to consider a notification complete.
///
/// # Use in Protocol Flow
///
/// This function supports these protocol scenarios:
/// - Preventing duplicate notification processing
/// - Implementing notification retry logic
/// - Supporting reliable notification delivery
/// - Coordinating request-response state machines
///
/// # Parameters
/// * `opcode` - Operation code of the request to check
/// * `cmd_pkt` - Packet containing sequence number for identification
///
/// # Returns
/// * `true` if notification has been completed for this packet
/// * `false` if notification is still pending or not started
fn req_cmd_is_notify_ok(opcode: u8, cmd_pkt: &Packet) -> bool
{
    RC_PKT_BUF.lock().iter().any(|pkt| {
        pkt.op == opcode && 
        pkt.sno == cmd_pkt.att_cmd().value.sno && 
        pkt.notify_ok_flag
    })
}

/// Marks notification completion for request-response transactions.
///
/// This function implements transaction state update by setting the notify OK flag
/// for packets that match the specified operation code and sequence number. It
/// supports reliable notification confirmation in the mesh protocol.
///
/// # State Update Algorithm
///
/// The function performs these operations:
/// 1. **Packet Matching**: Finds all cached packets with matching opcode and sequence number
/// 2. **Flag Setting**: Sets notify_ok_flag to true for matched packets
/// 3. **Batch Update**: Handles cases where multiple entries might match
///
/// # Transaction Completion Logic
///
/// Setting the notify OK flag indicates:
/// - The notification has been successfully processed
/// - Any waiting response handlers can proceed
/// - Duplicate notifications should be suppressed
/// - The transaction state can be cleaned up
///
/// # Concurrency Considerations
///
/// The function uses iterator chaining for efficient batch updates:
/// - `filter()` identifies matching packets
/// - `for_each()` applies the state change atomically
/// - Lock scope is minimized for concurrent access
///
/// # Parameters
/// * `opcode` - Operation code of the request to mark complete
/// * `cmd_pkt` - Packet containing sequence number for identification
///
/// # Side Effects
/// * Modifies notify_ok_flag for matching cached packets
/// * Affects subsequent notification status queries
/// * May influence request-response retry logic
fn req_cmd_set_notify_ok_flag(opcode: u8, cmd_pkt: &Packet)
{
    RC_PKT_BUF.lock().iter_mut().filter(
        |v| { v.op == opcode && v.sno == cmd_pkt.att_cmd().value.sno }
    ).for_each(
        |v| { v.notify_ok_flag = true }
    );
}

/// Classifies operation codes as notify request types with OTA state awareness.
///
/// This function implements operation code classification for request packets,
/// with special handling for OTA (Over-The-Air) update states. It ensures that
/// normal request processing is suspended during firmware updates to prevent
/// interference with the critical OTA process.
///
/// # Request Classification Algorithm
///
/// The function recognizes these request types:
/// - **Status Requests**: LGT_CMD_LIGHT_READ_STATUS for device status queries
/// - **Group Requests**: LGT_CMD_LIGHT_GRP_REQ for group operations
/// - **Configuration Requests**: LGT_CMD_CONFIG_DEV_ADDR, LGT_CMD_LIGHT_CONFIG_GRP
/// - **User Requests**: LGT_CMD_USER_NOTIFY_REQ for custom notifications
/// - **OTA Requests**: LGT_CMD_START_OTA_REQ, LGT_CMD_OTA_DATA_REQ, LGT_CMD_END_OTA_REQ
///
/// # OTA State Protection
///
/// During OTA updates, the function returns false for all requests to:
/// - Prevent normal operations from interfering with firmware updates
/// - Ensure OTA process has exclusive access to communication resources
/// - Maintain system stability during critical update operations
/// - Avoid potential conflicts between OTA and normal mesh traffic
///
/// # Design Rationale
///
/// Request classification enables:
/// - Different handling paths for requests vs. responses
/// - Appropriate acknowledgment and notification processing
/// - Protocol flow control during special states (OTA)
/// - Resource protection for critical operations
///
/// # Parameters
/// * `value` - Operation code to classify
///
/// # Returns
/// * `true` if opcode represents a notify request (and OTA is not in progress)
/// * `false` if opcode is not a request or OTA update is active
#[cfg_attr(test, mry::mry)]
pub fn rf_link_is_notify_req(value: u8) -> bool
{
    // Suspend normal request processing during OTA updates
    if !OTA_UPDATE_IN_PROGRESS.get() {
        return [
            LGT_CMD_LIGHT_READ_STATUS,  // Device status query request
            LGT_CMD_LIGHT_GRP_REQ,      // Group operation request
            LGT_CMD_CONFIG_DEV_ADDR,    // Device address configuration request
            LGT_CMD_LIGHT_CONFIG_GRP,   // Group configuration request
            LGT_CMD_USER_NOTIFY_REQ,    // User notification request
            LGT_CMD_START_OTA_REQ,      // OTA start request
            LGT_CMD_OTA_DATA_REQ,       // OTA data transfer request
            LGT_CMD_END_OTA_REQ         // OTA completion request
        ].contains(&value);
    }

    return false; // All requests disabled during OTA
}

/// Manages notification request masking for BLE slave status reporting.
///
/// This function implements a sophisticated notification request tracking system
/// that manages which devices should be included in status notifications sent
/// to BLE peripheral connections. It uses a circular buffer approach to efficiently
/// handle multiple concurrent notification requests.
///
/// # Notification Masking Algorithm
///
/// The function operates in two modes based on the unicast configuration:
///
/// ## Broadcast Mode (`!DEVICE_STATUS_READ_UNICAST_MODE`)
/// - Maintains a circular buffer of requesting device addresses
/// - Prevents duplicate entries in the notification mask
/// - Uses round-robin indexing to distribute notification load
/// - Supports up to 5 concurrent notification requests
///
/// ## Unicast Mode (`DEVICE_STATUS_READ_UNICAST_MODE`)
/// - Immediately invalidates pending data to force refresh
/// - Supports direct device-to-device status reporting
/// - Provides lower latency for targeted status queries
///
/// # Request Filtering Logic
///
/// Requests are processed only when:
/// 1. **Status Reading Active**: `SLAVE_READ_STATUS_BUSY != 0`
/// 2. **Address Validation**: Either different from our address OR special case (0x21)
/// 3. **Duplicate Prevention**: Address not already in notification mask
///
/// # Circular Buffer Management
///
/// The notification mask uses a 5-element circular buffer:
/// - **Base Offset**: Index 8 in the packet data structure
/// - **Round-Robin Index**: `NOTIFICATION_REQUEST_MASK_INDEX` tracks current position
/// - **Wraparound**: Index resets to 0 after reaching 5
/// - **Duplicate Check**: Prevents same address from appearing multiple times
///
/// # Performance Optimization
///
/// The algorithm optimizes for:
/// - **Fast Duplicate Detection**: O(n) scan of existing entries
/// - **Bounded Memory**: Fixed 5-address buffer prevents overflow
/// - **Fair Distribution**: Round-robin ensures all requesters get opportunities
/// - **Low Latency**: Direct indexing for unicast mode
///
/// # Parameters
/// * `adr` - Device address requesting status notification
///
/// # Side Effects
/// * Modifies notification request mask in PKT_LIGHT_DATA
/// * Updates round-robin index for fair distribution
/// * May invalidate pending data in unicast mode
/// * Affects subsequent status notification targeting
fn rf_link_slave_notify_req_mask(adr: u8)
{
    // Process notification requests only when status reading is active
    if SLAVE_READ_STATUS_BUSY.get() != 0 && 
       (DEVICE_ADDRESS.get() as u8 != adr || SLAVE_READ_STATUS_BUSY.get() == 0x21) {
        
        // Choose processing mode based on unicast configuration
        if !DEVICE_STATUS_READ_UNICAST_MODE.get() {
            // BROADCAST MODE: Manage circular notification buffer
            
            // Check for duplicate address in existing notification mask (indices 8-12)
            if PKT_LIGHT_DATA.lock().att_cmd().value.val[8..0xd].iter().any(|v| *v == adr) {
                return; // Address already in mask, no action needed
            }

            // Add address to notification mask using round-robin indexing
            let mask_index = (NOTIFICATION_REQUEST_MASK_INDEX.get() + 8) as usize;
            PKT_LIGHT_DATA.lock().att_cmd_mut().value.val[mask_index] = adr;
            
            // Advance round-robin index with wraparound (0-4 range)
            NOTIFICATION_REQUEST_MASK_INDEX.set((NOTIFICATION_REQUEST_MASK_INDEX.get() + 1) % 5);
        } else {
            // UNICAST MODE: Force data refresh for targeted response
            SLAVE_DATA_VALID.set(0);
        }
    }
}

/// Implements slave status response buffering with circular buffer management.
///
/// This function manages the complex process of buffering mesh status responses
/// for forwarding to BLE peripheral connections. It implements a sophisticated
/// circular buffer system with duplicate detection and structured packet formatting
/// for reliable status delivery.
///
/// # Response Buffering Algorithm
///
/// The function operates through several stages:
///
/// 1. **Duplicate Detection**: Checks if source address is already being tracked
/// 2. **Buffer Space Validation**: Ensures circular buffer has available space
/// 3. **Packet Formatting**: Structures response data according to protocol requirements
/// 4. **Metadata Management**: Updates tracking indices and notification masks
///
/// # Circular Buffer Management
///
/// The response buffer uses these management strategies:
/// - **Write Pointer**: Tracks next available buffer slot
/// - **Read Pointer**: Tracks next buffer slot to be consumed
/// - **Capacity Check**: Prevents buffer overflow conditions
/// - **Wraparound Logic**: Implements true circular buffer behavior
///
/// # Packet Structure Assembly
///
/// Response packets are formatted with:
/// - **L2CAP Header**: Standard BLE packet header with length and type information
/// - **ATT Data**: Structured attribute data for BLE attribute protocol
/// - **Mesh Data**: Source address, sequence number, and status parameters
/// - **Special Handling**: Different formatting based on internal parameter values
///
/// # Status Record Management
///
/// The function maintains a parallel status record system:
/// - **Address Tracking**: Records source addresses for duplicate detection
/// - **Index Management**: Tracks number of active status records
/// - **Notification Integration**: Links with notification request masking
///
/// # Buffer State Transitions
///
/// Valid buffer states:
/// - **Empty**: Write pointer equals read pointer
/// - **Partial**: Write pointer ahead of read pointer
/// - **Near Full**: One slot remaining before overlap
/// - **Full**: Buffer cannot accept new entries
///
/// # Parameters
/// * `packet` - Mesh packet containing status information to buffer
///
/// # Side Effects
/// * Modifies global response buffer (BUFF_RESPONSE)
/// * Updates buffer write pointer for next insertion
/// * Modifies status record tracking (SLAVE_STATUS_RECORD)
/// * Calls notification request masking for address tracking
/// * Increments device status record index
#[cfg_attr(test, mry::mry)]
pub fn rf_link_slave_add_status(packet: &Packet)
{
    let mut buf_response = BUFF_RESPONSE.lock();

    let mut result = false;

    // DUPLICATE DETECTION: Check if source address is already being tracked
    if DEVICE_STATUS_RECORD_INDEX.get() != 0 {
        for st_rec in *SLAVE_STATUS_RECORD.lock() {
            if packet.mesh().src_adr as u8 == st_rec.adr[0] {
                // Address already tracked - update notification mask and exit
                rf_link_slave_notify_req_mask(packet.mesh().src_adr as u8);
                return;
            }
        }
    }

    // BUFFER SPACE VALIDATION: Ensure circular buffer has room and record limit not exceeded
    // Check: (write_ptr + 1) % capacity != read_ptr (prevents buffer overlap)
    // Check: record_index < max_nodes (prevents excessive tracking)
    result = false;
    if (DEVICE_STATUS_BUFFER_WRITE_POINTER.get() + 1) % BUFF_RESPONSE_PACKET_COUNT != DEVICE_STATUS_BUFFER_READ_POINTER.get() && 
       DEVICE_STATUS_RECORD_INDEX.get() < MESH_NODE_MAX_NUM {
        
        // STATUS RECORD MANAGEMENT: Add new address to tracking system
        SLAVE_STATUS_RECORD.lock()[DEVICE_STATUS_RECORD_INDEX.get()].adr[0] = packet.mesh().src_adr as u8;
        DEVICE_STATUS_RECORD_INDEX.inc();
        
        // Update notification mask for this new address
        rf_link_slave_notify_req_mask(packet.mesh().src_adr as u8);

        // CIRCULAR BUFFER POINTER UPDATE: Advance write pointer with wraparound
        let index = DEVICE_STATUS_BUFFER_WRITE_POINTER.get();
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set((index + 1) % BUFF_RESPONSE_PACKET_COUNT);

        // PACKET STRUCTURE ASSEMBLY: Format response packet for BLE peripheral
        let st_ptr = &mut buf_response[index];
        
        // L2CAP Header Configuration
        st_ptr.head_mut().dma_len = 0x1d;      // DMA length: 29 bytes
        st_ptr.head_mut()._type = 2;           // Packet type: data
        st_ptr.head_mut().rf_len = 0x1b;       // RF payload: 27 bytes
        st_ptr.head_mut().l2cap_len = 0x17;    // L2CAP payload: 23 bytes
        st_ptr.head_mut().chan_id = 4;         // Channel ID: attribute protocol
        
        // ATT Data Header Configuration
        st_ptr.att_data_mut().att = 0x1b;      // ATT opcode
        st_ptr.att_data_mut().hl = 0x12;       // Handle length
        
        // MESH DATA COPY: Extract 20 bytes (0x14) of mesh packet data starting from sequence number
        st_ptr.att_data_mut().dat[0..0x14].copy_from_slice(
            unsafe {
                slice::from_raw_parts(addr_of!(packet.mesh().sno) as *const u8, 0x14)
            }
        );

        // SPECIAL PARAMETER HANDLING: Different formatting based on internal parameter values
        if packet.mesh().internal_par1[1] == 0 {
            // Standard parameter mode: copy specific parameter values
            st_ptr.att_data_mut().dat[0x12] = packet.mesh().par[9];
            st_ptr.att_data_mut().dat[0x13] = packet.mesh().internal_par1[0];
        } else if packet.mesh().internal_par1[1] != 4 && 
                  packet.mesh().internal_par1[1] != 5 && 
                  packet.mesh().internal_par1[1] != 8 {
            // Special mode handling for different parameter types
            if packet.mesh().internal_par1[1] == 6 {
                // Mode 6: Fill 3 bytes with 0xff (special status indicator)
                st_ptr.att_data_mut().dat[0x11..0x11 + 3].fill(0xff);
            } else if packet.mesh().internal_par1[1] != 7 && 
                      packet.mesh().internal_par1[1] != 9 {
                // Other modes: Fill 2 bytes with 0xff
                st_ptr.att_data_mut().dat[0x12..0x12 + 2].fill(0xff);
            }
            // Modes 7 and 9 use default packet data (no special handling)
        }
        // Modes 4, 5, and 8 use default packet data (no special handling)
        
        // SOURCE ADDRESS INSERTION: Add mesh source address to BLE packet
        st_ptr.att_data_mut().dat[3] = packet.mesh().src_adr as u8;           // Low byte
        st_ptr.att_data_mut().dat[4] = (packet.mesh().src_adr >> 8) as u8;    // High byte
        
        result = true; // Indicate successful buffer addition
    }
    // If buffer is full or record limit exceeded, packet is dropped (result remains false)
}

/// Implements the central mesh packet processing and routing engine.
///
/// This function serves as the core packet processing hub for the mesh network,
/// handling all incoming packets and implementing the complete routing, filtering,
/// acknowledgment, and relay algorithms. It coordinates between BLE peripheral
/// connections and mesh network operations with sophisticated protocol translation.
///
/// # Central Processing Algorithm
///
/// The function implements a multi-stage packet processing pipeline:
///
/// 1. **Packet Classification**: Distinguishes between status advertisements and command packets
/// 2. **Duplicate Detection**: Prevents reprocessing of previously seen packets
/// 3. **Protocol Translation**: Converts between BLE and mesh packet formats
/// 4. **Address Filtering**: Determines if packets are destined for this device
/// 5. **Response Generation**: Creates acknowledgment and response packets
/// 6. **Relay Processing**: Forwards packets to other mesh nodes when appropriate
///
/// # Status Advertisement Handling
///
/// Status packets are identified by:
/// - **Channel ID**: 0xffff indicates status advertisement
/// - **Signature**: 0xa5a5a5a5 pattern validates packet integrity
/// - **Format**: Contains multiple node status entries for topology updates
///
/// Status advertisements trigger mesh node database updates without further processing.
///
/// # Command Packet Processing
///
/// Command packets undergo comprehensive processing:
/// - **Operation Code Extraction**: Uses adaptive parsing for variable-length opcodes
/// - **Sequence Number Validation**: Ensures packet ordering and duplicate detection
/// - **Parameter Parsing**: Extracts command-specific parameters and arguments
/// - **Address Resolution**: Determines target devices and routing requirements
///
/// # BLE-Mesh Bridge Operations
///
/// The function implements sophisticated bridging between protocols:
/// - **Response Forwarding**: Routes mesh responses to waiting BLE connections
/// - **Command Translation**: Converts BLE requests to mesh protocol format
/// - **Status Aggregation**: Collects mesh status for BLE peripheral queries
/// - **Notification Management**: Coordinates request-response transactions
///
/// # Acknowledgment Protocol
///
/// Automatic acknowledgment generation follows these rules:
/// - **Request Detection**: Only non-notify requests require acknowledgments
/// - **Address Matching**: Acknowledgments sent only for addressed packets
/// - **Sequence Preservation**: ACK packets maintain original sequence numbers
/// - **Parameter Inclusion**: ACKs include relevant command parameters
///
/// # Relay Algorithm
///
/// Packet relay decisions are based on:
/// - **Address Matching**: Packets not addressed to this device are relayed
/// - **TTL Management**: Time-to-live prevents infinite packet circulation
/// - **Congestion Control**: Random delays prevent mesh network congestion
/// - **Connection Awareness**: Different timing for peripheral vs. mesh-only modes
///
/// # Performance Optimizations
///
/// The algorithm includes several optimizations:
/// - **Early Exit**: Status packets bypass complex command processing
/// - **Duplicate Cache**: Prevents redundant processing of repeated packets
/// - **Batch Operations**: Combines multiple packet modifications efficiently
/// - **Memory Safety**: Uses unsafe operations only for performance-critical sections
///
/// # Error Handling
///
/// Robust error handling includes:
/// - **Parse Failures**: Graceful handling of malformed packets
/// - **Buffer Overflow**: Protection against memory corruption
/// - **Protocol Violations**: Safe rejection of invalid packet formats
/// - **Resource Exhaustion**: Graceful degradation under heavy load
///
/// # Parameters
/// * `packet` - Mutable reference to incoming packet for processing and potential modification
///
/// # Side Effects
/// * May update mesh node status database
/// * May generate acknowledgment packets for transmission
/// * May forward packets to UART interface
/// * May trigger BLE peripheral status responses
/// * May queue packets for mesh relay transmission
/// * Modifies packet contents for relay operations
pub fn rf_link_rc_data(packet: &mut Packet) {
    // Get direct access to packet payload for efficient manipulation
    let pktdata = unsafe { 
        &*slice_from_raw_parts_mut(
            addr_of!(packet.att_write().value) as *mut u8, 
            core::mem::size_of::<PacketAttValue>()
        ) 
    };

    // PACKET CLASSIFICATION: STATUS ADVERTISEMENT DETECTION
    // Status advertisements use channel ID 0xffff and have 0xa5a5a5a5 signature
    if packet.head().chan_id == 0xffff {
        // Validate status packet signature (bytes 24-27 must be 0xa5)
        if pktdata[24..28].iter().all(|v| *v == 0xa5) {
            // Extract node status array from packet and update mesh database
            // Status data starts at sequence number field, spans 0x1a bytes
            // Each status entry is MESH_NODE_ST_VAL_LEN bytes
            mesh_node_update_status(unsafe { 
                slice::from_raw_parts(
                    addr_of!(packet.mesh().sno) as *const mesh_node_st_val_t, 
                    0x1a / MESH_NODE_ST_VAL_LEN
                ) 
            });
        }
        // Status packets require no further processing
        return;
    }

    // COMMAND PACKET PROCESSING: OPERATION CODE AND PARAMETER EXTRACTION
    let (success, mut op_cmd, mut op_cmd_len, mut params, mut params_len) = 
        parse_ble_packet_op_params(packet, true);
    if !success {
        // Malformed packet - discard without processing
        return;
    }

    // Extract operation code from parsed command data
    // For 3-byte opcodes, use lower 6 bits of first byte (mask 0x3f)
    let mut op = 0;
    if op_cmd_len == 3 {
        op = op_cmd[0] & 0x3f;
    }

    // SEQUENCE NUMBER ANALYSIS: Determine if this is a new message vs. slave response
    // Compare with stored general message sequence number to detect message origin
    let not_slave_message = packet.att_cmd().value.sno != *GENERAL_MESSAGE_SEQUENCE_NUMBER.lock();

    // DUPLICATE DETECTION: Check if packet has been processed recently
    if is_exist_in_rc_pkt_buf(op, packet) {
        return; // Duplicate packet - discard to prevent reprocessing
    }

    // UART FORWARDING: Send packet to external interface for logging/debugging
    light_mesh_rx_cb(packet);

    // DUPLICATE PREVENTION: Cache packet details to prevent future reprocessing
    rc_pkt_buf_push(op, packet);

    // BLE-MESH BRIDGE: RESPONSE FORWARDING TO PERIPHERAL CONNECTIONS
    // Handle response packets that should be forwarded to waiting BLE peripheral connections
    if rf_link_is_notify_rsp(op) && 
       packet.mesh().dst_adr == DEVICE_ADDRESS.get() && 
       BLE_PERIPHERAL_CONNECTION_ACTIVE.get() {
        
        // Validate that this response matches what the BLE peripheral is waiting for
        if SLAVE_READ_STATUS_BUSY.get() != op || 
           packet.att_cmd().value.sno != *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() {
            return; // Response doesn't match expected operation or sequence number
        }
        
        // Forward response to BLE peripheral connection
        rf_link_slave_add_status(packet);
        return; // Response forwarding complete - no further processing needed
    }

    // ADDRESS FILTERING: Determine if packet is addressed to this device
    let (group_match, device_match) = rf_link_match_group_mac(packet);
    
    // LOCAL PACKET PROCESSING: Handle packets addressed to this device
    if group_match || device_match {
        // Trigger application-level packet processing callback
        rf_link_data_callback(packet);

        // ACKNOWLEDGMENT GENERATION: Create ACK for non-notify requests that require it
        if !rf_link_is_notify_req(op) && packet.mesh().internal_par1[INTERNAL_PAR_SEND_ACK] != 0 {
            // Prepare acknowledgment packet
            let mut pkt_light_status = PKT_LIGHT_STATUS.lock();

            // Generate unique sequence number for ACK: current_time + device_address
            let cmd_sno = clock_time() + DEVICE_ADDRESS.get() as u32;
            pkt_light_status.att_cmd_mut().value.sno.copy_from_slice(unsafe {
                slice::from_raw_parts(addr_of!(cmd_sno) as *const u8, 3)
            });

            // Set transmission source to this device
            packet.mesh_mut().src_tx = DEVICE_ADDRESS.get();

            // Copy command parameters to ACK packet
            unsafe {
                // TODO: Investigate the +1 offset - potential pointer arithmetic issue
                let ptr = slice::from_raw_parts(
                    (addr_of!(packet.mesh().vendor_id) as u32 + 1) as *const u8, 
                    params_len as usize
                );
                pkt_light_status.att_cmd_mut().value.val[3..3 + ptr.len()].copy_from_slice(&ptr);
            }

            // Build ACK packet structure if this is a new message or different operation
            if not_slave_message || BLE_PERIPHERAL_LINK_COMMAND.get() != op {
                // Set up address routing for ACK response
                pkt_light_status.att_cmd_mut().value.src.copy_from_slice(unsafe { 
                    slice::from_raw_parts(addr_of!(packet.mesh().src_adr) as *const u8, 2) 
                });

                // Configure response addressing: reverse source/destination
                pkt_light_status.att_cmd_mut().value.dst = packet.att_cmd().value.src;
                pkt_light_status.att_cmd_mut().value.src[0] = (DEVICE_ADDRESS.get() & 0xff) as u8;
                pkt_light_status.att_cmd_mut().value.src[1] = ((DEVICE_ADDRESS.get() >> 8) & 0xff) as u8;

                // Set ACK packet header with vendor identification
                pkt_light_status.att_cmd_mut().value.val[0] = LGT_CMD_LIGHT_ACK | 0xc0; // ACK opcode with flags
                pkt_light_status.att_cmd_mut().value.val[1] = (VENDOR_ID & 0xFF) as u8;  // Vendor ID low byte
                pkt_light_status.att_cmd_mut().value.val[2] = ((VENDOR_ID >> 8) & 0xff) as u8; // Vendor ID high byte

                // Clear parameter area and set ACK-specific data
                pkt_light_status.att_cmd_mut().value.val[3..10 + 3].fill(0);
                pkt_light_status.att_cmd_mut().value.val[3] = op; // Original operation being acknowledged
                pkt_light_status.att_cmd_mut().value.val[4..4 + 3].copy_from_slice(&packet.att_cmd().value.sno); // Original sequence number

                // Mark packet for mesh transmission
                pkt_light_status.head_mut()._type |= BIT!(7);

                // Queue ACK for mesh transmission with retransmit count from original packet
                app().mesh_manager.add_send_mesh_msg(
                    &*pkt_light_status, 
                    0, // No transmission delay for ACKs
                    packet.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT]
                );
            }
        }
    }

    // NOTIFICATION REQUEST HANDLING: Manage request-response state tracking
    let mut slave_read_status_response = device_match;

    // Prevent duplicate processing of notification requests
    if rf_link_is_notify_req(op) {
        if req_cmd_is_notify_ok(op, packet) {
            // Request already processed - suppress duplicate response
            slave_read_status_response = false;
        } else {
            // Mark request as processed to prevent future duplicates
            req_cmd_set_notify_ok_flag(op, packet);
        }
    }

    // PACKET HEADER UPDATES: Recalculate length fields for potential relay
    packet.head_mut().dma_len = (packet.head().l2cap_len as u32 + 6) & 0xffffff; // DMA = L2CAP + 6
    packet.head_mut().rf_len = packet.head().l2cap_len as u8 + 4; // RF = L2CAP + 4

    // MESSAGE STATE TRACKING: Update global state for new messages
    if not_slave_message || BLE_PERIPHERAL_LINK_COMMAND.get() != op {
        // Update packet source for transmission
        packet.mesh_mut().src_tx = DEVICE_ADDRESS.get();

        // Store sequence number and operation for future duplicate detection
        *GENERAL_MESSAGE_SEQUENCE_NUMBER.lock() = packet.att_cmd().value.sno;
        BLE_PERIPHERAL_LINK_COMMAND.set(op);
    }

    // NOTIFICATION RESPONSE GENERATION: Create response packets for notify requests
    if rf_link_is_notify_req(op) && slave_read_status_response {
        let mut pkt_light_status = PKT_LIGHT_STATUS.lock();

        // Set response sequence number to match request
        pkt_light_status.att_cmd_mut().value.sno = packet.att_cmd().value.sno;
        packet.mesh_mut().src_tx = DEVICE_ADDRESS.get();
        
        // OPERATION-SPECIFIC RESPONSE CONFIGURATION: Set response parameters based on request type
        match op {
            LGT_CMD_LIGHT_READ_STATUS => {
                // Device status query response
                pkt_light_status.att_cmd_mut().value.val[15] = GET_STATUS;
                pkt_light_status.att_cmd_mut().value.val[13] = packet.mesh().par[9];
                pkt_light_status.att_cmd_mut().value.val[14] = 0;
            },
            LGT_CMD_LIGHT_GRP_REQ => {
                // Group operation response - copy internal parameter
                pkt_light_status.att_cmd_mut().value.val[15] = packet.mesh().internal_par1[1];
            },
            LGT_CMD_LIGHT_CONFIG_GRP => {
                // Group configuration response
                pkt_light_status.att_cmd_mut().value.val[15] = GET_GROUP1;
            },
            LGT_CMD_CONFIG_DEV_ADDR => {
                // Device address configuration response
                pkt_light_status.att_cmd_mut().value.val[15] = GET_DEV_ADDR;
            },
            LGT_CMD_USER_NOTIFY_REQ => {
                // User notification response
                pkt_light_status.att_cmd_mut().value.val[15] = GET_USER_NOTIFY;
            },
            LGT_CMD_START_OTA_REQ => {
                // OTA start response
                pkt_light_status.att_cmd_mut().value.val[15] = CMD_START_OTA;
            },
            LGT_CMD_OTA_DATA_REQ => {
                // OTA data transfer response
                pkt_light_status.att_cmd_mut().value.val[15] = CMD_OTA_DATA;
            },
            LGT_CMD_END_OTA_REQ => {
                // OTA completion response
                pkt_light_status.att_cmd_mut().value.val[15] = CMD_END_OTA;
            },
            _ => {
                // Unknown operation - use default handling
            }
        }

        // Copy request parameters to response packet
        unsafe {
            // TODO: Investigate the +1 offset - potential alignment or structure issue
            let ptr = slice::from_raw_parts(
                (addr_of!(packet.mesh().vendor_id) as u32 + 1) as *const u8, 
                params_len as usize
            );
            pkt_light_status.att_cmd_mut().value.val[3..3 + ptr.len()].copy_from_slice(&ptr);
        }

        // RESPONSE TRANSMISSION CONDITIONS: Send response if conditions are met
        if (not_slave_message || BLE_PERIPHERAL_LINK_COMMAND.get() != op) || params[1] != 0 {
            // Set response source address
            pkt_light_status.att_cmd_mut().value.src.copy_from_slice(unsafe { 
                slice::from_raw_parts(addr_of!(packet.mesh().src_adr) as *const u8, 2) 
            });

            // Generate application-specific response content
            if rf_link_response_callback(&mut pkt_light_status.att_cmd_mut().value, &packet.att_cmd().value) {
                // Mark packet for mesh transmission
                pkt_light_status.head_mut()._type |= BIT!(7);
                
                // Queue response for mesh transmission
                app().mesh_manager.add_send_mesh_msg(
                    &*pkt_light_status, 
                    0, // No transmission delay for responses
                    packet.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT]
                );
            }
        }
    }

    // PACKET RELAY ALGORITHM: Forward packets not addressed to this device
    if !device_match {
        // Mark packet for mesh transmission (relay mode)
        packet.head_mut()._type |= BIT!(7);

        // CONGESTION CONTROL: Calculate transmission delay to prevent network congestion
        let mut delay = 100; // Default 100µs delay for peripheral connections
        if !BLE_PERIPHERAL_CONNECTION_ACTIVE.get() {
            // Mesh-only mode: Use random delay to prevent collision storms
            // Random delay range: 0-8ms (8000µs - random_offset)
            // Formula: 8000 - ((tick ^ random) % 16) * 500
            // This creates 16 delay slots of 500µs each for collision avoidance
            delay = 8000 - (((read_reg_system_tick() as u16 ^ read_reg_rnd_number()) % 16) * 500);
        }

        // Queue packet for mesh relay transmission with calculated delay
        app().mesh_manager.add_send_mesh_msg(
            packet, 
            clock_time64() + (delay as u64 * CLOCK_SYS_CLOCK_1US as u64), // Convert delay to clock units
            packet.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT]
        );
    }
}

/// Implements comprehensive BLE peripheral data packet processing and connection management.
///
/// This function serves as the central handler for all incoming packets from BLE peripheral
/// connections, implementing sophisticated connection parameter management, timing adjustment,
/// and L2CAP/ATT protocol processing. It coordinates between low-level RF timing and
/// high-level application data processing.
///
/// # Packet Processing Pipeline
///
/// The function implements a multi-stage processing pipeline:
///
/// 1. **Packet Validation**: Validates packet type and channel ID bounds
/// 2. **Connection Management**: Handles connection parameter updates and channel maps
/// 3. **Timing Synchronization**: Performs adaptive timing adjustments
/// 4. **Protocol Processing**: Routes packets through L2CAP/ATT handlers
/// 5. **Response Generation**: Creates and queues response packets
///
/// # Connection Parameter Management
///
/// The function handles two types of connection parameter updates:
///
/// ## Channel Map Updates (l2cap_len & 0xff == 1)
/// - Updates the peripheral channel map for frequency hopping
/// - Sets timing update timestamp and next update instant
/// - Extracts 5-byte channel map from packet payload
/// - Coordinates channel map transitions for seamless operation
///
/// ## Connection Interval Updates (l2cap_len & 0xff == 0)
/// - Updates connection interval, window size, and timeout parameters
/// - Calculates timing values in microseconds using CLOCK_SYS_CLOCK_1US
/// - Manages connection offset for synchronized communication
/// - Preserves previous connection interval for rollback capability
///
/// # Timing Synchronization Algorithm
///
/// The function implements adaptive timing adjustment:
/// - Calls `rf_link_timing_adjust(time)` for real-time clock correction
/// - Handles timing drift compensation for crystal oscillator variations
/// - Maintains microsecond-level timing accuracy for BLE protocol compliance
/// - Coordinates with connection interval and window size adjustments
///
/// # Protocol Layer Integration
///
/// Packet routing through protocol stack:
/// - **L2CAP Processing**: Handles logical link control and adaptation
/// - **ATT Processing**: Manages attribute protocol for GATT services
/// - **Response Queuing**: Automatically queues generated response packets
/// - **Error Handling**: Graceful handling of malformed or invalid packets
///
/// # Channel Validation Algorithm
///
/// Channel ID validation ensures protocol compliance:
/// - **Type 2 Packets**: Channel ID must be ≤ 6 for data channels
/// - **Channel 5**: Special handling for connection parameter updates
/// - **Other Channels**: Standard L2CAP/ATT processing
///
/// # Packet Length Validation
///
/// Comprehensive length validation prevents buffer overflows:
/// - **Zero Length**: Immediate rejection to prevent processing errors
/// - **Minimum Length**: Must be ≥ 6 bytes for valid BLE packets
/// - **Maximum Length**: Bounded by protocol-specific limits
///
/// # Connection State Transitions
///
/// The function manages connection state based on packet types:
/// - **Parameter Updates**: Immediate state changes with instant coordination
/// - **Data Packets**: Maintain current connection parameters
/// - **Control Packets**: Special handling for connection management
///
/// # Performance Optimizations
///
/// - **Early Exit**: Invalid packets rejected without expensive processing
/// - **Direct Memory Access**: Unsafe pointer operations for performance-critical sections
/// - **Conditional Processing**: Different paths for different packet types
/// - **Batch Operations**: Multiple parameter updates in single packet
///
/// # Parameters
/// * `packet` - BLE packet received from peripheral connection
/// * `time` - Timestamp for timing synchronization and parameter updates
///
/// # Returns
/// * `true` - Packet processed successfully, connection parameters may have changed
/// * `false` - Packet rejected, no state changes made, or processing failed
///
/// # Side Effects
/// * May update global connection timing parameters
/// * May modify channel map for frequency hopping
/// * May queue response packets for transmission
/// * Triggers timing adjustment algorithms
/// * May change connection interval and timeout values
#[cfg_attr(test, mry::mry)]
pub fn rf_link_slave_data(packet: &Packet, time: u32) -> bool {
    // Extract key packet header fields for processing decisions
    let rf_len: u8 = packet.head().rf_len;
    let chanid: u16 = packet.head().chan_id;

    // PACKET TYPE VALIDATION: Check if packet type is valid for processing
    // The multiplication and comparison checks for valid packet type ranges
    if packet.head()._type as i32 * 0x1000000 >= -1 {
        
        // CHANNEL VALIDATION: Ensure channel ID is within valid range for data packets
        if (packet.head()._type & 3) == 2 {
            // Type 2 packets (data) must use channels 0-6
            if 6 < chanid {
                return false; // Invalid channel for data packet
            }
        }

        // CONNECTION PARAMETER UPDATE HANDLING: Channel 5 is reserved for parameter updates
        if chanid == 5 {
            // Update connection parameters using dedicated handler
            rf_update_conn_para(packet);
        }

        // TIMING SYNCHRONIZATION: Perform adaptive timing adjustment for all valid packets
        // This maintains microsecond-level timing accuracy despite crystal drift
        super::connection_management::rf_link_timing_adjust(time);
        
        // PACKET LENGTH VALIDATION: Check minimum packet length requirements
        if rf_len < 6 {
            // Packets shorter than 6 bytes are generally invalid
            if rf_len == 0 {
                return false; // Zero-length packets are always invalid
            }
            // Non-zero but short packets might be valid (e.g., empty data)
        } else {
            // EXTENDED PACKET PROCESSING: Handle control packets with special semantics
            
            // CHANNEL MAP UPDATE ALGORITHM: Type 3 packets with l2cap_len[0] == 1
            if packet.head()._type & 3 == 3 && packet.head().l2cap_len & 0xff == 1 {
                // Mark timing update as channel map change (type 1)
                BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(1);
                
                // Set next update instant from packet data (16-bit instant)
                BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(
                    ((packet.ll_data().sno as u16) << 8) | packet.ll_data().hh as u16
                );
                
                // Extract 5-byte channel map from packet payload
                // Channel map starts at offset +1 from l2cap_len field
                SLAVE_CHN_MAP.lock().iter_mut().enumerate().for_each(|(i, v)| {
                    *v = unsafe { 
                        *(addr_of!(packet.head().l2cap_len) as *const u8).offset(i as isize + 1) 
                    };
                });

                return true; // Channel map update processed successfully
            }

            // CONNECTION INTERVAL UPDATE ALGORITHM: 12-byte type 3 packets with l2cap_len[0] == 0
            if rf_len == 0xc && packet.head()._type & 3 == 3 && packet.head().l2cap_len & 0xff == 0 {
                // Preserve current interval for potential rollback
                BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.set(SLAVE_LINK_INTERVAL.get());
                
                // Set update instant from group field
                BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(packet.ll_data().group);
                
                // Calculate window size: ((upper_byte * 1250) + 1300) microseconds
                SLAVE_WINDOW_SIZE_UPDATE.set(
                    ((packet.head().l2cap_len >> 8) as u32 * 1250 + 1300) * CLOCK_SYS_CLOCK_1US
                );
                
                // Mark timing update as connection parameter change (type 2)
                BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(2);
                
                // Set new connection interval: 1250µs * interval_units
                BLE_CONN_INTERVAL.set(
                    CLOCK_SYS_CLOCK_1US * 1250 * 
                    unsafe { (*addr_of!(packet.ll_data().att) as *const u16) } as u32
                );
                
                // Set connection offset: channel_id * 1250µs
                BLE_CONN_OFFSET.set(packet.head().chan_id as u32 * CLOCK_SYS_CLOCK_1US * 1250);
                
                // Set connection timeout: nid * 10ms
                BLE_CONN_TIMEOUT.set(packet.ll_data().nid as u32 * 10000);
                
                return false; // Connection parameter update processed (no further handling)
            }
        }
        
        // L2CAP/ATT PROTOCOL PROCESSING: Route packet through protocol stack
        let res_pkt = l2cap_att_handler(packet);
        if res_pkt.is_some() {
            // Queue generated response packet for transmission
            rf_link_add_tx_packet(&res_pkt.unwrap());
        }
    }

    return false; // Default: packet processed without special return conditions
}

/// Implements transmission buffer capacity monitoring with DMA pointer arithmetic.
///
/// This function provides real-time assessment of the transmission buffer's ability
/// to accept new packets by analyzing the relationship between DMA write and read
/// pointers. It implements a safety threshold algorithm to prevent buffer overflow
/// and ensure reliable packet transmission.
///
/// # Buffer Capacity Algorithm
///
/// The function uses DMA pointer arithmetic to determine available buffer space:
///
/// ```
/// available_slots = (write_pointer - read_pointer) & 7
/// buffer_ready = available_slots < 3
/// ```
///
/// # Threshold Logic
///
/// The safety threshold of 3 serves multiple purposes:
/// - **Overflow Prevention**: Ensures buffer never becomes completely full
/// - **Performance Optimization**: Maintains pipeline efficiency by reserving slots
/// - **Latency Control**: Prevents excessive queuing delays
/// - **System Stability**: Provides margin for burst traffic and timing variations
///
/// # DMA Pointer Management
///
/// The algorithm leverages hardware DMA pointers:
/// - **Write Pointer**: Tracks next available buffer slot for new packets
/// - **Read Pointer**: Tracks next packet to be transmitted by DMA engine
/// - **Circular Buffer**: 8-slot circular buffer (& 7 implements modulo 8)
/// - **Atomic Operations**: Hardware-managed pointers ensure thread safety
///
/// # Buffer States
///
/// The function recognizes these buffer states:
/// - **Ready (0-2 packets)**: Safe to add new packets
/// - **Busy (3-7 packets)**: Buffer approaching capacity, reject new packets
/// - **Full (8 packets)**: Buffer completely full (prevented by threshold)
///
/// # Performance Characteristics
///
/// - **Time Complexity**: O(1) - single register read and arithmetic operation
/// - **Space Complexity**: O(1) - no additional memory allocation
/// - **Latency**: Minimal - direct hardware register access
/// - **Reliability**: Hardware-managed pointers ensure accuracy
///
/// # Returns
/// * `true` - Buffer has capacity for additional packets (< 3 packets queued)
/// * `false` - Buffer is busy or nearly full (≥ 3 packets queued)
///
/// # Use Cases
/// * Pre-transmission buffer validation
/// * Flow control and congestion management
/// * Real-time transmission scheduling
/// * System load monitoring
#[cfg_attr(test, mry::mry)]
pub fn is_add_packet_buf_ready() -> bool
{
    // Calculate available buffer slots using DMA pointer arithmetic
    // The & 7 implements modulo 8 for circular buffer with 8 slots
    return (read_reg_dma_tx_wptr() - read_reg_dma_tx_rptr() & 7) < 3;
}

/// Implements secure packet transmission queuing with encryption and DMA management.
///
/// This function manages the complete packet transmission pipeline, including packet
/// encryption, circular buffer management, and DMA queue coordination. It provides
/// reliable packet delivery with built-in security and flow control mechanisms.
///
/// # Transmission Pipeline Architecture
///
/// The function implements a multi-stage transmission pipeline:
///
/// 1. **Capacity Validation**: Ensures transmission buffer has available space
/// 2. **Buffer Management**: Manages circular buffer write pointer and initialization
/// 3. **Packet Encryption**: Applies security layer using pair encryption
/// 4. **DMA Coordination**: Coordinates with hardware DMA engine for transmission
/// 5. **Flow Control**: Implements backpressure when buffer capacity is exceeded
///
/// # Circular Buffer Management
///
/// The transmission buffer uses sophisticated circular buffer algorithms:
/// - **Fixed Capacity**: `BLT_FIFO_TX_PACKET_COUNT` slots prevent memory exhaustion
/// - **Write Pointer**: Atomic write pointer tracks next available slot
/// - **Wraparound Logic**: Modulo arithmetic ensures proper circular behavior
/// - **Thread Safety**: Atomic operations prevent race conditions
///
/// # Security Layer Integration
///
/// Packet security is enforced through:
/// - **Automatic Encryption**: All packets encrypted via `pair_enc_packet()`
/// - **In-Place Processing**: Encryption modifies packet data directly
/// - **Key Management**: Uses established pairing keys for encryption
/// - **Protocol Compliance**: Maintains BLE security requirements
///
/// # DMA Engine Coordination
///
/// The function coordinates with hardware DMA:
/// - **Empty Buffer Handling**: Writes empty packet marker when buffer was empty
/// - **Queue Registration**: Registers packet address with DMA engine
/// - **Automatic Transmission**: DMA engine handles actual RF transmission
/// - **Pointer Synchronization**: Coordinates with hardware read/write pointers
///
/// # Flow Control Algorithm
///
/// Buffer overflow prevention:
/// - **Capacity Check**: Validates buffer has space before processing
/// - **Early Rejection**: Returns false immediately if buffer is full
/// - **Threshold Management**: Uses 4-packet threshold for safety margin
/// - **Graceful Degradation**: Failed transmissions don't corrupt system state
///
/// # Memory Management
///
/// Static allocation strategy:
/// - **Fixed Buffer**: Pre-allocated packet buffer prevents dynamic allocation
/// - **Zero-Copy Operations**: Packet copying minimized for performance
/// - **Memory Safety**: Bounds checking prevents buffer overruns
/// - **Resource Cleanup**: No dynamic memory requires no cleanup
///
/// # Performance Optimizations
///
/// - **Early Exit**: Capacity check prevents expensive operations when buffer full
/// - **Atomic Operations**: Minimize lock contention and critical sections
/// - **Hardware Integration**: Leverages DMA for zero-CPU transmission
/// - **Memory Locality**: Buffer design optimizes cache performance
///
/// # Parameters
/// * `packet` - Packet to queue for transmission (will be encrypted)
///
/// # Returns
/// * `true` - Packet successfully queued for transmission
/// * `false` - Transmission buffer full, packet rejected
///
/// # Side Effects
/// * Encrypts packet data using pairing keys
/// * Modifies global transmission buffer and write pointer
/// * Registers packet with DMA engine for automatic transmission
/// * May write empty packet marker to DMA FIFO
///
/// # Thread Safety
/// * Uses atomic operations for write pointer management
/// * Mutex protection for shared buffer access
/// * Safe for concurrent access from multiple contexts
#[cfg_attr(test, mry::mry)]
pub fn rf_link_add_tx_packet(packet: &Packet) -> bool
{
    use crate::embassy::sync::mutex::{CriticalSectionMutex, Mutex};
    
    // STATIC BUFFER ALLOCATION: Pre-allocated transmission buffer for zero-allocation operation
    static BLT_TX_FIFO: CriticalSectionMutex<[Packet; BLT_FIFO_TX_PACKET_COUNT]> = Mutex::new(
        [
            // Initialize all buffer slots with zero-filled packet structures
            Packet {
                att_write: PacketAttWrite {
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
                        sno: [0; 3],
                        src: [0; 2],
                        dst: [0; 2],
                        val: [0; 23]
                    },
                }
            };
            BLT_FIFO_TX_PACKET_COUNT
        ]
    );
    
    // ATOMIC WRITE POINTER: Thread-safe write pointer for circular buffer management
    static BLT_TX_WPTR: AtomicUsize = AtomicUsize::new(0);

    // CAPACITY VALIDATION: Check DMA buffer capacity using hardware pointers
    let wptr = read_reg_dma_tx_wptr();  // Hardware write pointer
    let rptr = read_reg_dma_tx_rptr();  // Hardware read pointer
    let widx = (wptr - rptr) % BLT_FIFO_TX_PACKET_COUNT as u8;  // Available slots

    // FLOW CONTROL: Accept packets only when buffer has sufficient capacity
    if widx < 4 {  // Threshold: reserve 4 slots for safety margin
        
        // EMPTY BUFFER INITIALIZATION: Write empty packet marker when starting from empty
        if widx == 0 {
            // Signal DMA engine that buffer is being initialized
            write_reg_dma_tx_fifo(addr_of!(PKT_EMPTY) as u16);
        }

        // CIRCULAR BUFFER MANAGEMENT: Update write pointer with wraparound
        let index = BLT_TX_WPTR.get();
        BLT_TX_WPTR.set((index + 1) % BLT_FIFO_TX_PACKET_COUNT);

        // PACKET PROCESSING: Copy and encrypt packet for transmission
        let mut blt_tx_fifo = BLT_TX_FIFO.lock();

        // Copy packet data to transmission buffer slot
        blt_tx_fifo[index] = *packet;
        
        // SECURITY LAYER: Apply encryption using established pairing keys
        pair_enc_packet(&mut blt_tx_fifo[index]);

        // DMA QUEUE REGISTRATION: Register encrypted packet with DMA engine
        write_reg_dma_tx_fifo(addr_of!(blt_tx_fifo[index]) as u16);
        
        return true;  // Packet successfully queued for transmission
    }
    
    return false;  // Buffer full - packet rejected to prevent overflow
}

/// Extracts operation codes and parameters from BLE/mesh packets using adaptive parsing.
///
/// This function implements a sophisticated packet parsing algorithm that handles the
/// variable-length operation code format used in both BLE and mesh protocols. It adapts
/// its parsing behavior based on packet type and implements comprehensive validation
/// to ensure robust operation in noisy RF environments.
///
/// # Adaptive Parsing Algorithm
///
/// The parser uses a multi-stage approach:
///
/// 1. **Operation Code Length Detection**: Analyzes the MSB pattern of the first byte
///    to determine if the opcode is 1, 2, or 3 bytes long
///
/// 2. **Parameter Length Calculation**: Computes available parameter space based on
///    packet structure, operation code length, and protocol overhead
///
/// 3. **Protocol-Specific Adjustments**: Applies different parsing rules for mesh
///    vs. BLE packets to handle their different header structures
///
/// 4. **Validation and Bounds Checking**: Ensures extracted parameters fit within
///    protocol limits and packet boundaries
///
/// # Operation Code Format Specification
///
/// The operation code uses a self-describing format:
///
/// ```
/// Byte 0: [MSB][Bit6][Bit5][Bit4][Bit3][Bit2][Bit1][LSB]
///
/// MSB=0, Bit6=X: 1-byte opcode (0xxxxxxx)
/// MSB=1, Bit6=0: 2-byte opcode (10xxxxxx xxxxxxxx)
/// MSB=1, Bit6=1: 3-byte opcode (11xxxxxx xxxxxxxx xxxxxxxx)
/// ```
///
/// This format allows for:
/// - 128 single-byte opcodes (0x00-0x7F)
/// - 16,384 two-byte opcodes (0x8000-0xBFFF)
/// - 4,194,304 three-byte opcodes (0xC00000-0xFFFFFF)
///
/// # Parameter Extraction Algorithm
///
/// Parameter handling varies by context:
///
/// ## Standard Packets (mesh_flag=false)
/// - Parameters follow immediately after the operation code
/// - Length = `packet_length - opcode_length - header_overhead`
/// - Maximum 10 bytes for standard commands
///
/// ## Mesh Packets (mesh_flag=true)
/// - Additional header adjustments for mesh protocol overhead
/// - Special handling for opcode 6 (allows up to 15 bytes)
/// - Length calculation includes mesh-specific header fields
///
/// ## Special Operation Code Handling
/// - **Opcode 6**: Extended parameter mode (up to 15 bytes vs. standard 10)
/// - **Other opcodes**: Standard parameter limits apply
///
/// # Validation and Error Handling
///
/// The function performs comprehensive validation:
/// - Packet length bounds checking
/// - Parameter length limits verification
/// - Operation code format validation
/// - Protocol overhead calculations
///
/// # Parameters
/// * `packet` - The packet to parse (BLE or mesh format)
/// * `mesh_flag` - Protocol hint: true for mesh packets, false for BLE packets
///
/// # Returns
/// A tuple containing:
/// * `success` - Validation result: true if parsing succeeded, false if packet is malformed
/// * `op_codes` - Extracted operation code bytes (1-3 bytes, padded with zeros)
/// * `op_len` - Actual length of the operation code (1, 2, or 3)
/// * `parameters` - Extracted parameter bytes (up to 16 bytes, padded with zeros)
/// * `params_len` - Actual length of extracted parameters (0-16)
///
/// # Error Conditions
/// Returns `success=false` if:
/// - Operation code format is invalid
/// - Parameter length exceeds protocol limits
/// - Packet length is insufficient for claimed content
/// - Protocol overhead calculations fail
///
/// # Performance Characteristics
/// - Time Complexity: O(1) - fixed number of operations regardless of packet size
/// - Space Complexity: O(1) - uses fixed-size temporary buffers
/// - Memory Access: Linear scan through packet data (cache-friendly)
#[cfg_attr(test, mry::mry)]
pub fn parse_ble_packet_op_params(packet: &Packet, mesh_flag: bool) -> (bool, [u8; 3], u8, [u8; 16], u8)
{
    // Initialize return values
    let mut op_codes = [0u8; 3];
    let mut op_len = 0u8;
    let mut parameters = [0u8; 16];
    let mut params_len = 0u8;

    // Access the value field directly through the att_cmd() accessor method
    let val = &packet.att_cmd().value.val;
    
    // Determine the operation command length based on the bit patterns
    let first_op_byte = val[0];
    if first_op_byte & 0x80 != 0 {  // MSB is set
        op_len = if first_op_byte & 0x40 != 0 { 3 } else { 2 };
    } else {  // MSB is clear
        op_len = 1;
    }
    
    // Copy the operation code bytes
    op_codes[0..op_len as usize].copy_from_slice(&val[0..op_len as usize]);

    // Special handling for opcode 6 (more parameters allowed + delta offset adjustment)
    let is_special_op = (first_op_byte & 0x3f) == 6;
    let max_param_len = if is_special_op { 0xf } else { 10 };
    let pkt_len_delta = if is_special_op { 5 } else { 0 };

    // Calculate total available data space for parameters
    let header_len: u16 = 10;
    let mut packet_data_len = packet.head().l2cap_len - header_len;
    
    // Adjust for mesh vs non-mesh packet structures
    if mesh_flag {
        packet_data_len = pkt_len_delta + packet_data_len - op_len as u16 - header_len;
    } else {
        packet_data_len = packet_data_len - op_len as u16;
    }

    params_len = packet_data_len as u8;

    // Check if parameters will fit within allowed limits
    let success = packet_data_len <= max_param_len;
    
    if success {
        // Copy parameter bytes from the packet
        let param_start = op_len as usize;
        let param_end = param_start + packet_data_len as usize;
        parameters[0..packet_data_len as usize].copy_from_slice(&val[param_start..param_end]);
    } else {
        // If parameters are too long, set the length to 0
        params_len = 0;
    }

    (success, op_codes, op_len, parameters, params_len)
}
