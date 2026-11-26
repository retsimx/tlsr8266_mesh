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
//! - **Memory Safety**: Safe field access and type conversions throughout
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

use core::ptr::addr_of;
use core::sync::atomic::AtomicUsize;

use bytemuck;

use crate::ble::channel_selection::SLAVE_CHN_MAP;
use crate::ble::connection::{
    BLE_CONN_INTERVAL, BLE_CONN_OFFSET, BLE_CONN_TIMEOUT, BLE_PERIPHERAL_CONNECTION_ACTIVE,
    BLE_PERIPHERAL_LINK_COMMAND, BLE_PERIPHERAL_NEXT_UPDATE_INSTANT,
    BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL, BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP,
    BUFF_RESPONSE, SLAVE_DATA_VALID, SLAVE_LINK_INTERVAL, SLAVE_WINDOW_SIZE_UPDATE,
};
use crate::ble::flash_config::rf_update_conn_para;
use crate::ble::pair::pair_enc_packet;
use crate::callbacks::{dispatch_ota_snapshot, on_light_data, on_light_response, on_uart_mesh_rx};
use crate::facade::radio_config;
use crate::gatt::attribute::l2cap_att_handler;
use crate::rf::drv_8266::{DEVICE_ADDRESS, PKT_EMPTY};
use crate::types::*;
use mesh_protocol::mesh_types::*;
use mesh_protocol::packet_types::*;
use tlsr8266_hal::common::sync::SimplifyLS;
use tlsr8266_hal::mcu::clock::{clock_time, clock_time64, CLOCK_SYS_CLOCK_1US};
use tlsr8266_pac::BIT;
use tlsr8266_pac::*;

use crate::ble::status::{
    DEVICE_STATUS_BUFFER_READ_POINTER, DEVICE_STATUS_BUFFER_WRITE_POINTER,
    DEVICE_STATUS_READ_UNICAST_MODE, DEVICE_STATUS_RECORD_INDEX, NOTIFICATION_REQUEST_MASK_INDEX,
    SLAVE_READ_STATUS_BUSY, SLAVE_STATUS_RECORD,
};
use crate::mesh_ll::node_status::{
    mesh_node_update_status, rf_link_match_group_mac, GENERAL_MESSAGE_SEQUENCE_NUMBER,
    PKT_LIGHT_DATA, PKT_LIGHT_STATUS, RC_PKT_BUF, STATUS_MESSAGE_SEQUENCE_NUMBER,
};

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
pub fn is_exist_in_rc_pkt_buf(opcode: u8, cmd_pkt: &Packet) -> bool {
    RC_PKT_BUF
        .lock()
        .iter()
        .any(|v| v.op == opcode && v.sno == cmd_pkt.att_cmd().value.sno)
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
fn rf_link_is_notify_rsp(opcode: u8) -> bool {
    [
        LGT_CMD_LIGHT_GRP_RSP1,  // Group response type 1
        LGT_CMD_LIGHT_GRP_RSP2,  // Group response type 2
        LGT_CMD_LIGHT_GRP_RSP3,  // Group response type 3
        LGT_CMD_LIGHT_STATUS,    // Device status response
        LGT_CMD_DEV_ADDR_RSP,    // Device address response
        LGT_CMD_USER_NOTIFY_RSP, // User notification response
        LGT_CMD_START_OTA_RSP,   // OTA start response
        LGT_CMD_OTA_DATA_RSP,    // OTA data response
    ]
    .contains(&opcode)
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
fn rc_pkt_buf_push(opcode: u8, cmd_pkt: &Packet) {
    let mut rc_pkt_buf = RC_PKT_BUF.lock();

    // Implement FIFO replacement: remove oldest entry if buffer is full
    if rc_pkt_buf.is_full() {
        rc_pkt_buf.pop_back();
    }

    // Insert new entry at front for recent access optimization.
    // The `is_full` pop above guarantees the push cannot fail.
    let _ = rc_pkt_buf.push_front(PktBuf {
        op: opcode,
        sno: cmd_pkt.att_cmd().value.sno,
        notify_ok_flag: false, // Initialize as not notified
    });
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
fn req_cmd_is_notify_ok(opcode: u8, cmd_pkt: &Packet) -> bool {
    RC_PKT_BUF
        .lock()
        .iter()
        .any(|pkt| pkt.op == opcode && pkt.sno == cmd_pkt.att_cmd().value.sno && pkt.notify_ok_flag)
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
fn req_cmd_set_notify_ok_flag(opcode: u8, cmd_pkt: &Packet) {
    RC_PKT_BUF
        .lock()
        .iter_mut()
        .filter(|v| v.op == opcode && v.sno == cmd_pkt.att_cmd().value.sno)
        .for_each(|v| v.notify_ok_flag = true);
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
pub fn rf_link_is_notify_req(value: u8) -> bool {
    // Suspend normal request processing during OTA updates
    if dispatch_ota_snapshot().in_progress {
        return false; // All requests disabled during OTA
    }

    [
        LGT_CMD_LIGHT_READ_STATUS, // Device status query request
        LGT_CMD_LIGHT_GRP_REQ,     // Group operation request
        LGT_CMD_CONFIG_DEV_ADDR,   // Device address configuration request
        LGT_CMD_LIGHT_CONFIG_GRP,  // Group configuration request
        LGT_CMD_USER_NOTIFY_REQ,   // User notification request
        LGT_CMD_START_OTA_REQ,     // OTA start request
        LGT_CMD_OTA_DATA_REQ,      // OTA data transfer request
        LGT_CMD_END_OTA_REQ,       // OTA completion request
    ]
    .contains(&value)
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
fn rf_link_slave_notify_req_mask(adr: u8) {
    let status_busy = SLAVE_READ_STATUS_BUSY.get();
    let device_addr = DEVICE_ADDRESS.get() as u8;

    // Process notification requests only when status reading is active
    let should_process = status_busy != 0 && (device_addr != adr || status_busy == 0x21);
    if !should_process {
        return;
    }

    // Choose processing mode based on unicast configuration
    if DEVICE_STATUS_READ_UNICAST_MODE.get() {
        // UNICAST MODE: Force data refresh for targeted response
        SLAVE_DATA_VALID.set(0);
        return;
    }

    // BROADCAST MODE: Manage circular notification buffer

    // Check for duplicate address in existing notification mask (indices 8-12)
    {
        let pkt_data = PKT_LIGHT_DATA.lock();
        let notification_mask = &pkt_data.att_cmd().value.val[8..0xd];
        if notification_mask.contains(&adr) {
            return; // Address already in mask, no action needed
        }
    }

    // Add address to notification mask using round-robin indexing
    let mask_index = (NOTIFICATION_REQUEST_MASK_INDEX.get() + 8) as usize;
    PKT_LIGHT_DATA.lock().att_cmd_mut().value.val[mask_index] = adr;

    // Advance round-robin index with wraparound (0-4 range)
    NOTIFICATION_REQUEST_MASK_INDEX.set((NOTIFICATION_REQUEST_MASK_INDEX.get() + 1) % 5);
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
pub fn rf_link_slave_add_status(packet: &Packet) {
    let mut buf_response = BUFF_RESPONSE.lock();

    // DUPLICATE DETECTION: Check if source address is already being tracked
    let source_address = packet.mesh().src_adr as u8;

    if DEVICE_STATUS_RECORD_INDEX.get() != 0 {
        let status_records = SLAVE_STATUS_RECORD.lock();
        let address_already_tracked = status_records
            .iter()
            .any(|st_rec| st_rec.adr[0] == source_address);

        if address_already_tracked {
            // Address already tracked - update notification mask and exit
            rf_link_slave_notify_req_mask(source_address);
            return;
        }
    }

    // BUFFER SPACE VALIDATION: Ensure circular buffer has room and record limit not exceeded
    let write_ptr = DEVICE_STATUS_BUFFER_WRITE_POINTER.get();
    let read_ptr = DEVICE_STATUS_BUFFER_READ_POINTER.get();
    let record_index = DEVICE_STATUS_RECORD_INDEX.get();

    let buffer_has_space = (write_ptr + 1) % BUFF_RESPONSE_PACKET_COUNT != read_ptr;
    let within_node_limit = record_index < MESH_NODE_MAX_NUM;

    if !buffer_has_space || !within_node_limit {
        return;
    }

    // STATUS RECORD MANAGEMENT: Add new address to tracking system
    SLAVE_STATUS_RECORD.lock()[record_index].adr[0] = source_address;
    DEVICE_STATUS_RECORD_INDEX.inc();

    // Update notification mask for this new address
    rf_link_slave_notify_req_mask(source_address);

    // CIRCULAR BUFFER POINTER UPDATE: Advance write pointer with wraparound
    DEVICE_STATUS_BUFFER_WRITE_POINTER.set((write_ptr + 1) % BUFF_RESPONSE_PACKET_COUNT);

    // PACKET STRUCTURE ASSEMBLY: Format response packet for BLE peripheral
    let st_ptr = &mut buf_response[write_ptr];

    // L2CAP Header Configuration
    st_ptr.head_mut().dma_len = 0x1d; // DMA length: 29 bytes
    st_ptr.head_mut()._type = 2; // Packet type: data
    st_ptr.head_mut().rf_len = 0x1b; // RF payload: 27 bytes
    st_ptr.head_mut().l2cap_len = 0x17; // L2CAP payload: 23 bytes
    st_ptr.head_mut().chan_id = 4; // Channel ID: attribute protocol

    // ATT Data Header Configuration
    st_ptr.att_data_mut().att = 0x1b; // ATT opcode
    st_ptr.att_data_mut().hl = 0x12; // Handle length

    // MESH DATA COPY: Extract 20 bytes (0x14) of mesh packet data starting from sequence number
    // Copy the sequence number (3 bytes)
    st_ptr.att_data_mut().dat[0..3].copy_from_slice(&packet.mesh().sno);
    // Copy the destination address (2 bytes)
    let dst_bytes = packet.mesh().dst_adr.to_le_bytes();
    st_ptr.att_data_mut().dat[3..5].copy_from_slice(&dst_bytes);
    // Copy the source address (2 bytes)
    let src_bytes = packet.mesh().src_adr.to_le_bytes();
    st_ptr.att_data_mut().dat[5..7].copy_from_slice(&src_bytes);
    // Copy op (1 byte), vendor_id (2 bytes), and parameters (10 bytes) from the flat
    // PacketAttValue.val overlay (MeshPkt is align(4) and inserts a 1-byte padding
    // hole between op and vendor_id at offset 21).
    let val = &packet.att_cmd().value.val;
    st_ptr.att_data_mut().dat[7] = val[0];
    st_ptr.att_data_mut().dat[8..10].copy_from_slice(&val[1..3]);
    st_ptr.att_data_mut().dat[10..20].copy_from_slice(&val[3..13]);

    // SPECIAL PARAMETER HANDLING: Only format mode 0 is used in practice
    let format_mode = packet.mesh().internal_par1[INTERNAL_PAR_PACKET_FORMAT_MODE];
    if format_mode == PACKET_FORMAT_STANDARD_PARAMS {
        // Standard parameter mode: copy specific parameter values to response
        st_ptr.att_data_mut().dat[PKT_ATT_DATA_STATUS_PARAM] =
            packet.mesh().par[MESH_PAR_STATUS_TICK];
        st_ptr.att_data_mut().dat[PKT_ATT_DATA_STATUS_DATA] =
            packet.mesh().internal_par1[INTERNAL_PAR_STATUS_DATA];
    } else {
        // All packets initialize with internal_par1: [0; 5], so this should never happen
        // but we'll use the fallback behavior for safety
        st_ptr.att_data_mut().dat[PKT_ATT_DATA_STATUS_PARAM..=PKT_ATT_DATA_STATUS_DATA].fill(0xff);
    }

    // SOURCE ADDRESS INSERTION: Add mesh source address to BLE packet
    st_ptr.att_data_mut().dat[3] = packet.mesh().src_adr as u8; // Low byte
    st_ptr.att_data_mut().dat[4] = (packet.mesh().src_adr >> 8) as u8; // High byte
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
    // PACKET CLASSIFICATION: STATUS ADVERTISEMENT DETECTION
    // Status advertisements use channel ID 0xffff and have 0xa5a5a5a5 signature
    if packet.head().chan_id == 0xffff {
        // Validate status packet signature at val[17..21] (which maps to bytes 24-27 in the original pktdata)
        const SIGNATURE: [u8; 4] = [0xa5, 0xa5, 0xa5, 0xa5];
        let signature_slice = &packet.att_write().value.val[17..21];

        if signature_slice == SIGNATURE {
            // Extract node status array from packet and update mesh database
            // Status data is in the first MESH_STATUS_VALUE_LEN bytes of PacketAttValue (written by mesh_node_adv_status)
            // This overlays the sno, src, dst, and val fields of PacketAttValue
            // We need to extract from the value field, not from MeshPkt metadata fields

            // Get raw bytes of PacketAttValue starting from the beginning
            // SAFETY: (4) audited packed-wire view. `packet` is the caller's live
            // `&mut Packet`; the slice starts at its `value` field and spans
            // `MESH_STATUS_VALUE_LEN` (24) of the 30 `PacketAttValue` bytes, which the
            // signature check above proves were written by `mesh_node_adv_status`.
            let value_bytes = unsafe {
                core::slice::from_raw_parts(
                    addr_of!(packet.att_write().value) as *const u8,
                    MESH_STATUS_VALUE_LEN, // Status data bytes
                )
            };

            // Convert to mesh_node_st_val_t array safely using bytemuck
            let status_entries = MESH_STATUS_VALUE_LEN / MESH_NODE_ST_VAL_LEN;
            let bytes_needed = status_entries * MESH_NODE_ST_VAL_LEN;
            let status_bytes = &value_bytes[0..bytes_needed];
            let status_slice = bytemuck::cast_slice::<u8, MeshNodeStValT>(status_bytes);
            mesh_node_update_status(status_slice);
        }
        // Status packets require no further processing
        return;
    }

    // COMMAND PACKET PROCESSING: OPERATION CODE AND PARAMETER EXTRACTION
    let (success, op_cmd, op_cmd_len, params, params_len) =
        parse_ble_packet_op_params(packet, true);
    if !success {
        // Malformed packet - discard without processing
        return;
    }

    // Extract operation code from parsed command data
    // Handle different operation code lengths (only 1-byte and 3-byte supported)
    let op = match op_cmd_len {
        1 => op_cmd[0],
        3 => {
            // For 3-byte opcodes, use lower 6 bits of first byte (mask 0x3f)
            // This extracts the original opcode from response packets (opcode | 0xc0)
            op_cmd[0] & 0x3f
        }
        _ => 0, // 2-byte opcodes are not supported and will result in op = 0
    };

    // SEQUENCE NUMBER ANALYSIS: Determine if this is a new message vs. slave response
    // Compare with stored general message sequence number to detect message origin
    let not_slave_message = packet.att_cmd().value.sno != *GENERAL_MESSAGE_SEQUENCE_NUMBER.lock();

    // DUPLICATE DETECTION: Check if packet has been processed recently
    if is_exist_in_rc_pkt_buf(op, packet) {
        return; // Duplicate packet - discard to prevent reprocessing
    }

    // UART FORWARDING: Send packet to external interface for logging/debugging
    on_uart_mesh_rx(packet);

    // DUPLICATE PREVENTION: Cache packet details to prevent future reprocessing
    rc_pkt_buf_push(op, packet);

    // BLE-MESH BRIDGE: RESPONSE FORWARDING TO PERIPHERAL CONNECTIONS
    // Handle response packets that should be forwarded to waiting BLE peripheral connections
    if rf_link_is_notify_rsp(op)
        && packet.mesh().dst_adr == DEVICE_ADDRESS.get()
        && BLE_PERIPHERAL_CONNECTION_ACTIVE.get()
    {
        // Validate that this response matches what the BLE peripheral is waiting for
        if SLAVE_READ_STATUS_BUSY.get() != op
            || packet.att_cmd().value.sno != *STATUS_MESSAGE_SEQUENCE_NUMBER.lock()
        {
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
        on_light_data(packet);

        // ACKNOWLEDGMENT GENERATION: Create ACK for non-notify requests that require it
        if !rf_link_is_notify_req(op) && packet.mesh().internal_par1[INTERNAL_PAR_SEND_ACK] != 0 {
            // Prepare acknowledgment packet
            let mut pkt_light_status = PKT_LIGHT_STATUS.lock();

            // Generate unique sequence number for ACK: current_time + device_address
            let cmd_sno = clock_time() + DEVICE_ADDRESS.get() as u32;
            let sno_bytes = cmd_sno.to_le_bytes();
            pkt_light_status
                .att_cmd_mut()
                .value
                .sno
                .copy_from_slice(&sno_bytes[0..3]);

            // Set transmission source to this device
            packet.mesh_mut().src_tx = DEVICE_ADDRESS.get();

            // Copy command parameters to ACK packet
            // vendor_id[1] is the high byte; followed by op and parameters
            let vendor_high_byte = ((packet.mesh().vendor_id >> 8) & 0xFF) as u8;
            let mut param_data = [0u8; 12]; // Max size for vendor_high + op + 10 param bytes
            param_data[0] = vendor_high_byte;
            param_data[1] = packet.mesh().op;
            let param_bytes_to_copy = (params_len as usize).saturating_sub(2).min(10);
            param_data[2..2 + param_bytes_to_copy]
                .copy_from_slice(&packet.mesh().par[0..param_bytes_to_copy]);

            let copy_len = (params_len as usize).min(12);
            pkt_light_status.att_cmd_mut().value.val[3..3 + copy_len]
                .copy_from_slice(&param_data[0..copy_len]);

            // Build ACK packet structure if this is a new message or different operation
            if not_slave_message || BLE_PERIPHERAL_LINK_COMMAND.get() != op {
                // Set up address routing for ACK response
                let src_bytes = packet.mesh().src_adr.to_le_bytes();
                pkt_light_status
                    .att_cmd_mut()
                    .value
                    .src
                    .copy_from_slice(&src_bytes);

                // Configure response addressing: reverse source/destination
                pkt_light_status.att_cmd_mut().value.dst = packet.att_cmd().value.src;
                pkt_light_status.att_cmd_mut().value.src[0] = (DEVICE_ADDRESS.get() & 0xff) as u8;
                pkt_light_status.att_cmd_mut().value.src[1] =
                    ((DEVICE_ADDRESS.get() >> 8) & 0xff) as u8;

                // Set ACK packet header with vendor identification
                let vendor_id = radio_config().vendor_id;
                pkt_light_status.att_cmd_mut().value.val[0] = LGT_CMD_LIGHT_ACK | 0xc0; // ACK opcode with flags
                pkt_light_status.att_cmd_mut().value.val[1] = (vendor_id & 0xFF) as u8; // Vendor ID low byte
                pkt_light_status.att_cmd_mut().value.val[2] = ((vendor_id >> 8) & 0xff) as u8; // Vendor ID high byte

                // Clear parameter area and set ACK-specific data
                pkt_light_status.att_cmd_mut().value.val[3..10 + 3].fill(0);
                pkt_light_status.att_cmd_mut().value.val[3] = op; // Original operation being acknowledged
                pkt_light_status.att_cmd_mut().value.val[4..4 + 3]
                    .copy_from_slice(&packet.att_cmd().value.sno); // Original sequence number

                // Mark packet for mesh transmission
                pkt_light_status.head_mut()._type |= BIT!(7);

                // Queue ACK for mesh transmission with retransmit count from original packet
                crate::queues::enqueue_mesh_tx(
                    &pkt_light_status,
                    0, // No transmission delay for ACKs
                    packet.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
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
    packet.head_mut().dma_len = packet.head().l2cap_len as u32 + 6; // DMA = L2CAP + 6
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
                pkt_light_status.att_cmd_mut().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE] =
                    GET_STATUS;
                pkt_light_status.att_cmd_mut().value.val[PKT_VAL_MAC_APP_TTC] =
                    packet.mesh().par[MESH_PAR_STATUS_TICK];
                pkt_light_status.att_cmd_mut().value.val[PKT_VAL_MAC_APP_HOP_COUNT] = 0;
            }
            LGT_CMD_LIGHT_GRP_REQ => {
                // Group operation response - copy packet format mode parameter
                pkt_light_status.att_cmd_mut().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE] =
                    packet.mesh().internal_par1[INTERNAL_PAR_PACKET_FORMAT_MODE];
            }
            LGT_CMD_LIGHT_CONFIG_GRP => {
                // Group configuration response
                pkt_light_status.att_cmd_mut().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE] =
                    GET_GROUP1;
            }
            LGT_CMD_CONFIG_DEV_ADDR => {
                // Device address configuration response
                pkt_light_status.att_cmd_mut().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE] =
                    GET_DEV_ADDR;
            }
            LGT_CMD_USER_NOTIFY_REQ => {
                // User notification response
                pkt_light_status.att_cmd_mut().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE] =
                    GET_USER_NOTIFY;
            }
            LGT_CMD_START_OTA_REQ => {
                // OTA start response
                pkt_light_status.att_cmd_mut().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE] =
                    CMD_START_OTA;
            }
            LGT_CMD_OTA_DATA_REQ => {
                // OTA data transfer response
                pkt_light_status.att_cmd_mut().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE] =
                    CMD_OTA_DATA;
            }
            LGT_CMD_END_OTA_REQ => {
                // OTA completion response
                pkt_light_status.att_cmd_mut().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE] =
                    CMD_END_OTA;
            }
            _ => {
                // Unknown operation - use default handling
            }
        }

        // Copy request parameters to response packet
        // vendor_id[1] is the high byte; followed by op and parameters
        let vendor_high_byte = ((packet.mesh().vendor_id >> 8) & 0xFF) as u8;
        let mut param_data = [0u8; 12]; // Max size for vendor_high + op + 10 param bytes
        param_data[0] = vendor_high_byte;
        param_data[1] = packet.mesh().op;
        let param_bytes_to_copy = (params_len as usize).saturating_sub(2).min(10);
        param_data[2..2 + param_bytes_to_copy]
            .copy_from_slice(&packet.mesh().par[0..param_bytes_to_copy]);

        let copy_len = (params_len as usize).min(12);
        pkt_light_status.att_cmd_mut().value.val[3..3 + copy_len]
            .copy_from_slice(&param_data[0..copy_len]);

        // RESPONSE TRANSMISSION CONDITIONS: Send response if conditions are met
        if (not_slave_message || BLE_PERIPHERAL_LINK_COMMAND.get() != op) || params[1] != 0 {
            // Set response source address
            let src_bytes = packet.mesh().src_adr.to_le_bytes();
            pkt_light_status
                .att_cmd_mut()
                .value
                .src
                .copy_from_slice(&src_bytes);

            // Generate application-specific response content
            if on_light_response(
                &mut pkt_light_status.att_cmd_mut().value,
                &packet.att_cmd().value,
            )
            .is_ok()
            {
                // Mark packet for mesh transmission
                pkt_light_status.head_mut()._type |= BIT!(7);

                // Queue response for mesh transmission
                crate::queues::enqueue_mesh_tx(
                    &pkt_light_status,
                    0, // No transmission delay for responses
                    packet.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
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
        crate::queues::enqueue_mesh_tx(
            packet,
            clock_time64() + (delay as u64 * CLOCK_SYS_CLOCK_1US as u64), // Convert delay to clock units
            packet.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
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
pub fn rf_link_slave_data(packet: &Packet, time: u32) {
    // Extract key packet header fields for processing decisions
    let header = packet.head();
    let rf_len = header.rf_len;
    let chanid = header.chan_id;
    let packet_type = header._type;

    // CHANNEL VALIDATION: Ensure channel ID is within valid range for data packets
    if (packet_type & 3) == 2 && chanid > 6 {
        return; // Invalid channel for data packet
    }

    // CONNECTION PARAMETER UPDATE HANDLING: Channel 5 is reserved for parameter updates
    if chanid == 5 {
        // Update connection parameters using dedicated handler
        rf_update_conn_para(packet);
    }

    // TIMING SYNCHRONIZATION: Perform adaptive timing adjustment for all valid packets
    // This maintains microsecond-level timing accuracy despite crystal drift
    crate::ble::connection::rf_link_timing_adjust(time);

    // PACKET LENGTH VALIDATION: Check minimum packet length requirements
    if rf_len < 6 {
        // Packets shorter than 6 bytes are generally invalid
        if rf_len == 0 {
            return; // Zero-length packets are always invalid
        }
        // Non-zero but short packets might be valid (e.g., empty data)
    } else {
        // EXTENDED PACKET PROCESSING: Handle control packets with special semantics

        // CHANNEL MAP UPDATE ALGORITHM: Type 3 packets with l2cap_len[0] == 1
        if packet_type & 3 == 3 && packet.head().l2cap_len & 0xff == 1 {
            // Mark timing update as channel map change (type 1)
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(1);

            // Set next update instant from packet data (16-bit instant)
            BLE_PERIPHERAL_NEXT_UPDATE_INSTANT
                .set(((packet.ll_data().sno as u16) << 8) | packet.ll_data().hh as u16);

            // Extract 5-byte channel map from packet using the correctly aligned structure
            let channel_map_data = packet.channel_map_update().channel_map;
            let mut channel_map = SLAVE_CHN_MAP.lock();
            channel_map.copy_from_slice(&channel_map_data);

            return; // Channel map update processed successfully
        }

        // CONNECTION INTERVAL UPDATE ALGORITHM: 12-byte type 3 packets with l2cap_len[0] == 0
        if rf_len == 0xc && packet_type & 3 == 3 && packet.head().l2cap_len & 0xff == 0 {
            // Preserve current interval for potential rollback
            BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.set(SLAVE_LINK_INTERVAL.get());

            // Set update instant from group field
            BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(packet.ll_data().group);

            SLAVE_WINDOW_SIZE_UPDATE
                .set(((packet.head().l2cap_len >> 8) as u32 * 1250 + 1300) * CLOCK_SYS_CLOCK_1US);

            // Mark timing update as connection parameter change (type 2)
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(2);

            // Set new connection interval: 1250µs * interval_units
            // Extract interval value from the att field safely
            BLE_CONN_INTERVAL.set(CLOCK_SYS_CLOCK_1US * 1250 * packet.ll_data().att as u32);

            // Set connection offset: channel_id * 1250µs
            BLE_CONN_OFFSET.set(chanid as u32 * CLOCK_SYS_CLOCK_1US * 1250);

            // Set connection timeout: nid * 10ms
            BLE_CONN_TIMEOUT.set(packet.ll_data().nid as u32 * 10000);

            return; // Connection parameter update processed (no further handling)
        }
    }

    // L2CAP/ATT PROTOCOL PROCESSING: Route packet through protocol stack
    if let Some(res_pkt) = l2cap_att_handler(packet) {
        // Queue the generated response for transmission. A full TX FIFO drops
        // it: this path is best-effort and the peer retries the ATT request, so
        // there is no recovery action and no logging seam in this crate.
        if let Err(()) = rf_link_add_tx_packet(&res_pkt) {
            // Dropped: see above.
        }
    }

    // Default: packet processed without special conditions
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
/// ```text
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
pub fn is_add_packet_buf_ready() -> bool {
    // Calculate available buffer slots using DMA pointer arithmetic
    // The & 7 implements modulo 8 for circular buffer with 8 slots
    let write_ptr = read_reg_dma_tx_wptr();
    let read_ptr = read_reg_dma_tx_rptr();
    let used_slots = (write_ptr - read_ptr) & 7;

    used_slots < 3
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
pub fn rf_link_add_tx_packet(packet: &Packet) -> Result<(), ()> {
    use tlsr8266_hal::common::sync::CriticalSectionMutex;

    // STATIC BUFFER ALLOCATION: Pre-allocated transmission buffer for zero-allocation operation
    static BLT_TX_FIFO: CriticalSectionMutex<[Packet; BLT_FIFO_TX_PACKET_COUNT]> =
        CriticalSectionMutex::new(
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
        ],
        );

    // ATOMIC WRITE POINTER: Thread-safe write pointer for circular buffer management
    static BLT_TX_WPTR: AtomicUsize = AtomicUsize::new(0);

    // CAPACITY VALIDATION: Check DMA buffer capacity using hardware pointers
    let wptr = read_reg_dma_tx_wptr(); // Hardware write pointer
    let rptr = read_reg_dma_tx_rptr(); // Hardware read pointer
    let widx = (wptr - rptr) % BLT_FIFO_TX_PACKET_COUNT as u8; // Available slots

    // FLOW CONTROL: Accept packets only when buffer has sufficient capacity
    if widx < 4 {
        // Threshold: reserve 4 slots for safety margin

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

        return Ok(()); // Packet successfully queued for transmission
    }

    Err(()) // Buffer full - packet rejected to prevent overflow
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
/// ```text
/// Byte 0: [MSB][Bit6][Bit5][Bit4][Bit3][Bit2][Bit1][LSB]
///
/// MSB=0, Bit6=X: 1-byte opcode (0xxxxxxx)
/// MSB=1, Bit6=1: 3-byte opcode (11xxxxxx xxxxxxxx xxxxxxxx)
/// ```
/// Note: 2-byte opcodes are not supported in this implementation.
///
/// This format allows for:
/// - 128 single-byte opcodes (0x00-0x7F)
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
pub fn parse_ble_packet_op_params(
    packet: &Packet,
    mesh_flag: bool,
) -> (bool, [u8; 3], u8, [u8; 16], u8) {
    // Access the value field directly through the att_write() accessor method
    let val = &packet.att_write().value.val;

    // Determine the operation command length based on the bit patterns
    // Only support 1-byte and 3-byte opcodes (2-byte opcodes are not used)
    let first_op_byte = val[0];
    let op_len = match (first_op_byte & 0x80 != 0, first_op_byte & 0x40 != 0) {
        (false, _) => 1,   // MSB is clear -> 1-byte opcode
        (true, true) => 3, // Both MSB and 0x40 set -> 3-byte opcode
        (true, false) => {
            // 2-byte opcodes are not supported - treat as invalid
            return (false, [0u8; 3], 0, [0u8; 16], 0);
        }
    };

    // Copy the operation code bytes from the val[] flat byte overlay.
    // MeshPkt is repr(C, align(4)): the 1-byte padding after op (offset 21) means
    // mesh_construct_packet writes vendor_id_lo there via val[1], so the receiver
    // correctly reads val[1]=vendor_id_lo, val[2]=vendor_id_hi, val[3..]=params.
    // Both old and new firmware use this same val[] layout for mesh commands.
    let mut op_codes = [0u8; 3];
    op_codes[0] = first_op_byte;
    if op_len > 1 {
        op_codes[1..op_len].copy_from_slice(&val[1..op_len]);
    }

    // Special handling for opcode 6 (more parameters allowed + delta offset adjustment)
    let is_special_op = (first_op_byte & 0x3f) == 6;
    let (max_param_len, pkt_len_delta) = if is_special_op { (0xf, 5) } else { (10, 0) };

    // Calculate total available data space for parameters
    let header_len: u16 = 10;
    let base_packet_len = packet.head().l2cap_len - header_len;

    let packet_data_len = if mesh_flag {
        pkt_len_delta + base_packet_len - op_len as u16 - header_len
    } else {
        base_packet_len - op_len as u16
    };

    // Check if parameters will fit within allowed limits
    let success = packet_data_len <= max_param_len;

    let (parameters, params_len) = if success {
        // Copy parameter bytes from the packet
        let mut parameters = [0u8; 16];
        let param_count = packet_data_len as usize;
        let param_start = op_len;
        parameters[0..param_count].copy_from_slice(&val[param_start..param_start + param_count]);
        (parameters, packet_data_len as u8)
    } else {
        // If parameters are too long, return empty parameters
        ([0u8; 16], 0)
    };

    (success, op_codes, op_len as u8, parameters, params_len)
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::callbacks::{set_ble_app_hooks, BleAppHooks};
    use crate::mesh_ll::node_status::{
        MESH_NODE_MASK, MESH_NODE_MAX, MESH_NODE_REPORT_ENABLE, MESH_NODE_ST,
    };
    use core::sync::atomic::Ordering;
    use mesh_protocol::mesh_types::{MeshNodeStT, MeshNodeStValT, MESH_NODE_ST_PAR_LEN};
    use tlsr8266_hal::mcu::clock::fake_clock;
    use tlsr8266_pac::fake_mmio::FakeMmio;

    // ================================================================================
    // Application seams: recording hooks for the `callbacks` dispatchers
    // ================================================================================

    /// Count of `on_light_data` dispatches recorded by the installed hook.
    static LIGHT_DATA_CALLS: AtomicUsize = AtomicUsize::new(0);
    /// Count of `on_light_response` dispatches recorded by the installed hook.
    static LIGHT_RESPONSE_CALLS: AtomicUsize = AtomicUsize::new(0);
    /// Count of `on_uart_mesh_rx` dispatches recorded by the installed hook.
    static UART_MESH_RX_CALLS: AtomicUsize = AtomicUsize::new(0);

    fn noop_mesh_pair_proc() {}

    fn noop_light_data(_: &Packet) {}

    /// "No application response" is the only safe no-op: `Ok(())` would make the
    /// SDK queue a response packet the application never built.
    fn noop_light_response(_: &mut PacketAttValue, _: &PacketAttValue) -> Result<(), ()> {
        Err(())
    }

    fn noop_uart_mesh_rx(_: &Packet) {}

    fn record_light_data(_: &Packet) {
        LIGHT_DATA_CALLS.fetch_add(1, Ordering::Relaxed);
    }

    /// Answers `Ok(())`, which is the application behaviour that makes the SDK
    /// queue the built response packet.
    fn record_light_response(_: &mut PacketAttValue, _: &PacketAttValue) -> Result<(), ()> {
        LIGHT_RESPONSE_CALLS.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    fn record_uart_mesh_rx(_: &Packet) {
        UART_MESH_RX_CALLS.fetch_add(1, Ordering::Relaxed);
    }

    /// Installs hooks that record the three seams these tests assert on, and
    /// zeroes their counters.
    fn install_recording_hooks() -> RecordingHooksGuard {
        LIGHT_DATA_CALLS.store(0, Ordering::Relaxed);
        LIGHT_RESPONSE_CALLS.store(0, Ordering::Relaxed);
        UART_MESH_RX_CALLS.store(0, Ordering::Relaxed);
        set_ble_app_hooks(BleAppHooks {
            on_mesh_pair_proc: noop_mesh_pair_proc,
            on_light_data: record_light_data,
            on_light_response: record_light_response,
            on_uart_mesh_rx: record_uart_mesh_rx,
        });
        RecordingHooksGuard
    }

    /// Restores the neutral hook set the rest of the suite assumes.
    fn install_noop_hooks() {
        set_ble_app_hooks(BleAppHooks {
            on_mesh_pair_proc: noop_mesh_pair_proc,
            on_light_data: noop_light_data,
            on_light_response: noop_light_response,
            on_uart_mesh_rx: noop_uart_mesh_rx,
        });
    }

    /// Restores the neutral hook set when a test that installed recording hooks
    /// ends, including while unwinding from a failed assertion.
    struct RecordingHooksGuard;

    impl Drop for RecordingHooksGuard {
        fn drop(&mut self) {
            install_noop_hooks();
        }
    }

    fn light_data_calls() -> usize {
        LIGHT_DATA_CALLS.load(Ordering::Relaxed)
    }

    fn light_response_calls() -> usize {
        LIGHT_RESPONSE_CALLS.load(Ordering::Relaxed)
    }

    fn uart_mesh_rx_calls() -> usize {
        UART_MESH_RX_CALLS.load(Ordering::Relaxed)
    }

    /// Helper function to create a test packet with specified values.
    fn create_test_packet(opcode: u8, sno: [u8; 3], src_adr: u16, dst_adr: u16) -> Packet {
        Packet {
            mesh: MeshPkt {
                head: PacketL2capHead {
                    dma_len: 0x1B, // 27 bytes
                    _type: 2,
                    rf_len: 0x19,    // 25 bytes
                    l2cap_len: 0x15, // 21 bytes (10 header + 1 opcode + 10 params = 21)
                    chan_id: 4,
                },
                src_tx: 0,
                handle1: 0,
                sno,
                src_adr,
                dst_adr,
                op: opcode,
                vendor_id: radio_config().vendor_id,
                par: [0; 10],
                internal_par1: [0; 5],
                ttl: 0,
                internal_par2: [0; 4],
                no_use: [0; 4],
            },
        }
    }

    /// Helper function to create a mesh status advertisement packet.
    fn create_status_adv_packet() -> Packet {
        Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead {
                    dma_len: 0x27,
                    _type: 2,
                    rf_len: 0x25,
                    l2cap_len: 0x21,
                    chan_id: 0xffff, // Status advertisement channel
                },
                opcode: 0x12,
                handle: 0,
                handle1: 0,
                value: PacketAttValue {
                    sno: [0x01, 0x02, 0x03],
                    src: [0x10, 0x20],
                    dst: [0x30, 0x40],
                    val: [
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xa5,
                        0xa5, // Status signature bytes at position 21-22 (adjusted for safe array access)
                    ],
                },
            },
        }
    }

    /// Reset global state to known values for test isolation.
    /// Reset the PAC/HAL host fakes so a test never inherits another test's
    /// register bank, analog bank, clock or AES transport (the suite runs
    /// `--test-threads=1`, so all of these are process-wide singletons).
    fn reset_host_fakes() {
        FakeMmio::reset();
        tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog::reset();
        fake_clock::reset();
        tlsr8266_hal::mcu::crypto::fake_crypto::reset();
    }

    /// Datasheet offsets of the DMA TX queue registers, as `read_reg_dma_tx_*`
    /// and `write_reg_dma_tx_fifo` reach them.
    const REG_DMA_TX_RPTR: u32 = 0x52a;
    const REG_DMA_TX_WPTR: u32 = 0x52b;
    const REG_DMA_TX_FIFO: u32 = 0x52c;

    /// Number of `write_reg_dma_tx_fifo` registrations the run performed.
    fn dma_tx_fifo_writes() -> usize {
        FakeMmio::writes()
            .iter()
            .filter(|w| w.addr == (REG_DMA_TX_FIFO | tlsr8266_pac::REG_BASE_ADDR))
            .count()
    }

    fn reset_test_state() {
        reset_host_fakes();
        install_noop_hooks();
        RC_PKT_BUF.lock().clear();
        DEVICE_ADDRESS.set(0x1234);
        DEVICE_STATUS_RECORD_INDEX.set(0);
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(0);
        DEVICE_STATUS_BUFFER_READ_POINTER.set(0);
        NOTIFICATION_REQUEST_MASK_INDEX.set(0);
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(false);
        SLAVE_READ_STATUS_BUSY.set(0);
        BLE_PERIPHERAL_LINK_COMMAND.set(0);
        DEVICE_STATUS_READ_UNICAST_MODE.set(false);
        SLAVE_DATA_VALID.set(0);
        *GENERAL_MESSAGE_SEQUENCE_NUMBER.lock() = [0x01, 0x02, 0x03];
        *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() = [0x04, 0x05, 0x06];
        crate::callbacks::test_ota::set_in_progress_for_test(false);

        // Reset PKT_LIGHT_DATA to clean state
        *PKT_LIGHT_DATA.lock() = Packet {
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
                    sno: [0; 3],
                    src: [0; 2],
                    dst: [0; 2],
                    val: [0; 23],
                },
            },
        };

        // Reset SLAVE_STATUS_RECORD
        let mut status_record = SLAVE_STATUS_RECORD.lock();
        for record in status_record.iter_mut() {
            record.adr[0] = 0;
            record.alarm_id = 0;
        }

        // Reset PKT_LIGHT_STATUS to clean state - CRITICAL for test isolation
        *PKT_LIGHT_STATUS.lock() = Packet {
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
                    sno: [0; 3],
                    src: [0; 2],
                    dst: [0; 2],
                    val: [0; 23],
                },
            },
        };

        // Reset the shared queues (this also drains MESH_TX_QUEUE and
        // MESH_RX_CHANNEL) so each test starts from a known-clean transport
        // state. The UART statics belong to the firmware's fixture, not to this
        // crate, so they are deliberately not touched here.
        crate::queues::reset_for_test();
    }

    /// Puts the mesh node table in the state a fresh connection leaves it: one
    /// entry (this device, address low byte `0x34`) and no status records.
    fn reset_mesh_node_table() {
        MESH_NODE_MAX.set(1);
        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            for i in 0..mesh_node_st.len() {
                mesh_node_st[i] = MeshNodeStT {
                    miss: 0,
                    val: MeshNodeStValT {
                        dev_adr: if i == 0 { 0x34 } else { 0 },
                        sn: 0,
                        par: [0; MESH_NODE_ST_PAR_LEN],
                    },
                };
            }
        }
        MESH_NODE_MASK.lock().fill(0);
    }

    // ================================================================================
    // Tests for is_exist_in_rc_pkt_buf function
    // ================================================================================

    /// Tests duplicate detection with empty buffer.
    #[test]
    fn test_is_exist_in_rc_pkt_buf_empty_buffer() {
        reset_test_state();

        let packet = create_test_packet(LGT_CMD_LIGHT_ONOFF, [0x01, 0x02, 0x03], 0x1234, 0x5678);

        let result = is_exist_in_rc_pkt_buf(LGT_CMD_LIGHT_ONOFF, &packet);

        assert!(!result, "Empty buffer should not contain any packets");
    }

    /// Tests duplicate detection with matching packet.
    #[test]
    fn test_is_exist_in_rc_pkt_buf_matching_packet() {
        reset_test_state();

        let packet = create_test_packet(LGT_CMD_LIGHT_ONOFF, [0x01, 0x02, 0x03], 0x1234, 0x5678);

        // Add packet to buffer manually
        let mut rc_pkt_buf = RC_PKT_BUF.lock();
        rc_pkt_buf
            .push_front(PktBuf {
                op: LGT_CMD_LIGHT_ONOFF,
                sno: [0x01, 0x02, 0x03],
                notify_ok_flag: false,
            })
            .unwrap();
        drop(rc_pkt_buf);

        let result = is_exist_in_rc_pkt_buf(LGT_CMD_LIGHT_ONOFF, &packet);

        assert!(result, "Should find matching packet in buffer");
    }

    /// Tests duplicate detection with different operation codes.
    #[test]
    fn test_is_exist_in_rc_pkt_buf_different_opcode() {
        reset_test_state();

        let packet = create_test_packet(LGT_CMD_LIGHT_STATUS, [0x01, 0x02, 0x03], 0x1234, 0x5678);

        // Add packet with different opcode to buffer
        let mut rc_pkt_buf = RC_PKT_BUF.lock();
        rc_pkt_buf
            .push_front(PktBuf {
                op: LGT_CMD_LIGHT_ONOFF,
                sno: [0x01, 0x02, 0x03],
                notify_ok_flag: false,
            })
            .unwrap();
        drop(rc_pkt_buf);

        let result = is_exist_in_rc_pkt_buf(LGT_CMD_LIGHT_STATUS, &packet);

        assert!(!result, "Should not find packet with different opcode");
    }

    /// Tests duplicate detection with different sequence numbers.
    #[test]
    fn test_is_exist_in_rc_pkt_buf_different_sno() {
        reset_test_state();

        let packet = create_test_packet(LGT_CMD_LIGHT_ONOFF, [0x04, 0x05, 0x06], 0x1234, 0x5678);

        // Add packet with different sequence number to buffer
        let mut rc_pkt_buf = RC_PKT_BUF.lock();
        rc_pkt_buf
            .push_front(PktBuf {
                op: LGT_CMD_LIGHT_ONOFF,
                sno: [0x01, 0x02, 0x03],
                notify_ok_flag: false,
            })
            .unwrap();
        drop(rc_pkt_buf);

        let result = is_exist_in_rc_pkt_buf(LGT_CMD_LIGHT_ONOFF, &packet);

        assert!(
            !result,
            "Should not find packet with different sequence number"
        );
    }

    // ================================================================================
    // Tests for rf_link_is_notify_rsp function
    // ================================================================================

    /// Tests response classification for known response opcodes.
    #[test]
    fn test_rf_link_is_notify_rsp_valid_responses() {
        assert!(rf_link_is_notify_rsp(LGT_CMD_LIGHT_GRP_RSP1));
        assert!(rf_link_is_notify_rsp(LGT_CMD_LIGHT_GRP_RSP2));
        assert!(rf_link_is_notify_rsp(LGT_CMD_LIGHT_GRP_RSP3));
        assert!(rf_link_is_notify_rsp(LGT_CMD_LIGHT_STATUS));
        assert!(rf_link_is_notify_rsp(LGT_CMD_DEV_ADDR_RSP));
        assert!(rf_link_is_notify_rsp(LGT_CMD_USER_NOTIFY_RSP));
        assert!(rf_link_is_notify_rsp(LGT_CMD_START_OTA_RSP));
        assert!(rf_link_is_notify_rsp(LGT_CMD_OTA_DATA_RSP));
    }

    /// Tests response classification for non-response opcodes.
    #[test]
    fn test_rf_link_is_notify_rsp_non_responses() {
        assert!(!rf_link_is_notify_rsp(LGT_CMD_LIGHT_ONOFF));
        assert!(!rf_link_is_notify_rsp(LGT_CMD_LIGHT_READ_STATUS));
        assert!(!rf_link_is_notify_rsp(LGT_CMD_CONFIG_DEV_ADDR));
        assert!(!rf_link_is_notify_rsp(LGT_CMD_START_OTA_REQ));
        assert!(!rf_link_is_notify_rsp(0xFF)); // Invalid opcode
    }

    // ================================================================================
    // Tests for rf_link_is_notify_req function
    // ================================================================================

    /// Tests request classification during normal operation.
    #[test]
    fn test_rf_link_is_notify_req_normal_operation() {
        reset_test_state();
        crate::callbacks::test_ota::set_in_progress_for_test(false);

        assert!(rf_link_is_notify_req(LGT_CMD_LIGHT_READ_STATUS));
        assert!(rf_link_is_notify_req(LGT_CMD_LIGHT_GRP_REQ));
        assert!(rf_link_is_notify_req(LGT_CMD_CONFIG_DEV_ADDR));
        assert!(rf_link_is_notify_req(LGT_CMD_LIGHT_CONFIG_GRP));
        assert!(rf_link_is_notify_req(LGT_CMD_USER_NOTIFY_REQ));
        assert!(rf_link_is_notify_req(LGT_CMD_START_OTA_REQ));
        assert!(rf_link_is_notify_req(LGT_CMD_OTA_DATA_REQ));
        assert!(rf_link_is_notify_req(LGT_CMD_END_OTA_REQ));
    }

    /// Tests request classification during OTA update.
    #[test]
    fn test_rf_link_is_notify_req_during_ota() {
        reset_test_state();
        crate::callbacks::test_ota::set_in_progress_for_test(true);

        // All requests should be blocked during OTA
        assert!(!rf_link_is_notify_req(LGT_CMD_LIGHT_READ_STATUS));
        assert!(!rf_link_is_notify_req(LGT_CMD_LIGHT_GRP_REQ));
        assert!(!rf_link_is_notify_req(LGT_CMD_CONFIG_DEV_ADDR));
        assert!(!rf_link_is_notify_req(LGT_CMD_START_OTA_REQ));
    }

    /// Tests request classification for non-request opcodes.
    #[test]
    fn test_rf_link_is_notify_req_non_requests() {
        reset_test_state();
        crate::callbacks::test_ota::set_in_progress_for_test(false);

        assert!(!rf_link_is_notify_req(LGT_CMD_LIGHT_STATUS));
        assert!(!rf_link_is_notify_req(LGT_CMD_DEV_ADDR_RSP));
        assert!(!rf_link_is_notify_req(LGT_CMD_LIGHT_ONOFF));
        assert!(!rf_link_is_notify_req(0xFF)); // Invalid opcode
    }

    // ================================================================================
    // Tests for rc_pkt_buf_push function
    // ================================================================================

    /// Tests adding packets to empty buffer.
    #[test]
    fn test_rc_pkt_buf_push_empty_buffer() {
        reset_test_state();

        let packet = create_test_packet(LGT_CMD_LIGHT_ONOFF, [0x01, 0x02, 0x03], 0x1234, 0x5678);

        rc_pkt_buf_push(LGT_CMD_LIGHT_ONOFF, &packet);

        let rc_pkt_buf = RC_PKT_BUF.lock();
        assert_eq!(rc_pkt_buf.len(), 1, "Buffer should contain one packet");

        let pkt = rc_pkt_buf.iter().next().unwrap();
        assert_eq!(pkt.op, LGT_CMD_LIGHT_ONOFF);
        assert_eq!(pkt.sno, [0x01, 0x02, 0x03]);
        assert!(!pkt.notify_ok_flag);
    }

    /// Tests FIFO behavior when buffer becomes full.
    #[test]
    fn test_rc_pkt_buf_push_fifo_replacement() {
        reset_test_state();

        // Fill buffer to capacity (20 packets)
        for i in 0..20 {
            let packet = create_test_packet(LGT_CMD_LIGHT_ONOFF, [i as u8, 0, 0], 0x1234, 0x5678);
            rc_pkt_buf_push(LGT_CMD_LIGHT_ONOFF, &packet);
        }

        let rc_pkt_buf = RC_PKT_BUF.lock();
        assert_eq!(rc_pkt_buf.len(), 20, "Buffer should be at capacity");
        let first_pkt = rc_pkt_buf.iter().next().unwrap();
        assert_eq!(
            first_pkt.sno[0], 19,
            "Most recent packet should be at front"
        );
        drop(rc_pkt_buf);

        // Add one more packet - should trigger FIFO replacement
        let packet = create_test_packet(LGT_CMD_LIGHT_STATUS, [99, 0, 0], 0x1234, 0x5678);
        rc_pkt_buf_push(LGT_CMD_LIGHT_STATUS, &packet);

        let rc_pkt_buf = RC_PKT_BUF.lock();
        assert_eq!(rc_pkt_buf.len(), 20, "Buffer should still be at capacity");
        let first_pkt = rc_pkt_buf.iter().next().unwrap();
        assert_eq!(first_pkt.sno[0], 99, "New packet should be at front");
        assert_eq!(
            first_pkt.op, LGT_CMD_LIGHT_STATUS,
            "New packet opcode should match"
        );
    }

    // ================================================================================
    // Tests for req_cmd_is_notify_ok and req_cmd_set_notify_ok_flag functions
    // ================================================================================

    /// Tests notification status checking with pending notification.
    #[test]
    fn test_req_cmd_is_notify_ok_pending() {
        reset_test_state();

        let packet = create_test_packet(
            LGT_CMD_LIGHT_READ_STATUS,
            [0x01, 0x02, 0x03],
            0x1234,
            0x5678,
        );

        // Add packet without notify_ok_flag set
        let mut rc_pkt_buf = RC_PKT_BUF.lock();
        rc_pkt_buf
            .push_front(PktBuf {
                op: LGT_CMD_LIGHT_READ_STATUS,
                sno: [0x01, 0x02, 0x03],
                notify_ok_flag: false,
            })
            .unwrap();
        drop(rc_pkt_buf);

        let result = req_cmd_is_notify_ok(LGT_CMD_LIGHT_READ_STATUS, &packet);
        assert!(
            !result,
            "Should return false when notify_ok_flag is not set"
        );
    }

    /// Tests notification status checking with completed notification.
    #[test]
    fn test_req_cmd_is_notify_ok_completed() {
        reset_test_state();

        let packet = create_test_packet(
            LGT_CMD_LIGHT_READ_STATUS,
            [0x01, 0x02, 0x03],
            0x1234,
            0x5678,
        );

        // Add packet with notify_ok_flag set
        let mut rc_pkt_buf = RC_PKT_BUF.lock();
        rc_pkt_buf
            .push_front(PktBuf {
                op: LGT_CMD_LIGHT_READ_STATUS,
                sno: [0x01, 0x02, 0x03],
                notify_ok_flag: true,
            })
            .unwrap();
        drop(rc_pkt_buf);

        let result = req_cmd_is_notify_ok(LGT_CMD_LIGHT_READ_STATUS, &packet);
        assert!(result, "Should return true when notify_ok_flag is set");
    }

    /// Tests setting notification OK flag.
    #[test]
    fn test_req_cmd_set_notify_ok_flag() {
        reset_test_state();

        let packet = create_test_packet(
            LGT_CMD_LIGHT_READ_STATUS,
            [0x01, 0x02, 0x03],
            0x1234,
            0x5678,
        );

        // Add packet without notify_ok_flag set
        let mut rc_pkt_buf = RC_PKT_BUF.lock();
        rc_pkt_buf
            .push_front(PktBuf {
                op: LGT_CMD_LIGHT_READ_STATUS,
                sno: [0x01, 0x02, 0x03],
                notify_ok_flag: false,
            })
            .unwrap();
        drop(rc_pkt_buf);

        // Set notify OK flag
        req_cmd_set_notify_ok_flag(LGT_CMD_LIGHT_READ_STATUS, &packet);

        // Verify flag was set
        let rc_pkt_buf = RC_PKT_BUF.lock();
        let first_pkt = rc_pkt_buf.iter().next().unwrap();
        assert!(first_pkt.notify_ok_flag, "notify_ok_flag should be set");
    }

    // ================================================================================
    // Tests for rf_link_slave_add_status function
    // ================================================================================

    /// Tests adding status when buffer has space.
    #[test]
    fn test_rf_link_slave_add_status_success() {
        reset_test_state();

        let packet = create_test_packet(LGT_CMD_LIGHT_STATUS, [0x01, 0x02, 0x03], 0x5678, 0x1234);

        rf_link_slave_add_status(&packet);

        // Verify status record was added
        assert_eq!(
            DEVICE_STATUS_RECORD_INDEX.get(),
            1,
            "Status record index should increment"
        );

        let status_record = SLAVE_STATUS_RECORD.lock();
        assert_eq!(
            status_record[0].adr[0], 0x78,
            "Status record should contain source address low byte"
        );

        // Verify buffer write pointer advanced
        assert_eq!(
            DEVICE_STATUS_BUFFER_WRITE_POINTER.get(),
            1,
            "Write pointer should advance"
        );
    }

    /// Tests duplicate address handling in status buffer.
    #[test]
    fn test_rf_link_slave_add_status_duplicate_address() {
        reset_test_state();

        let packet = create_test_packet(LGT_CMD_LIGHT_STATUS, [0x01, 0x02, 0x03], 0x5678, 0x1234);

        // Add first packet
        rf_link_slave_add_status(&packet);

        let initial_record_index = DEVICE_STATUS_RECORD_INDEX.get();
        let initial_write_pointer = DEVICE_STATUS_BUFFER_WRITE_POINTER.get();

        // Add packet with same source address
        rf_link_slave_add_status(&packet);

        // Verify no new record was added
        assert_eq!(
            DEVICE_STATUS_RECORD_INDEX.get(),
            initial_record_index,
            "Record index should not change for duplicate address"
        );
        assert_eq!(
            DEVICE_STATUS_BUFFER_WRITE_POINTER.get(),
            initial_write_pointer,
            "Write pointer should not advance for duplicate address"
        );
    }

    /// Tests buffer overflow protection.
    #[test]
    fn test_rf_link_slave_add_status_buffer_full() {
        reset_test_state();

        // Fill buffer to near capacity
        DEVICE_STATUS_BUFFER_READ_POINTER.set(0);
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(BUFF_RESPONSE_PACKET_COUNT - 1);

        let packet = create_test_packet(LGT_CMD_LIGHT_STATUS, [0x01, 0x02, 0x03], 0x5678, 0x1234);

        let initial_record_index = DEVICE_STATUS_RECORD_INDEX.get();

        rf_link_slave_add_status(&packet);

        // Buffer should reject packet when full
        assert_eq!(
            DEVICE_STATUS_RECORD_INDEX.get(),
            initial_record_index,
            "Record index should not change when buffer is full"
        );
    }

    /// Tests that the assembled BLE notification packet has op at dat[7],
    /// vendor_id at dat[8..10], and params at dat[10..18] from the flat
    /// PacketAttValue.val overlay (matching rf_link_response_callback /
    /// mesh_construct_packet and the Python client's decrypt_notification parser).
    #[test]
    fn test_rf_link_slave_add_status_packet_layout() {
        reset_test_state();
        SLAVE_READ_STATUS_BUSY.set(LGT_CMD_LIGHT_STATUS); // allow notify req mask

        let mut packet = create_test_packet(
            LGT_CMD_LIGHT_STATUS | 0xc0,
            [0xAA, 0xBB, 0xCC],
            0x001F, // src_adr = 31
            0x0000, // dst_adr
        );
        // Populate the flat PacketAttValue.val[0..13] overlay just as
        // rf_link_response_callback and mesh_construct_packet do (without MeshPkt's
        // align(4) padding hole at offset 21).
        {
            let val = &mut packet.att_cmd_mut().value.val;
            val[0] = LGT_CMD_LIGHT_STATUS | 0xc0;
            val[1..3].copy_from_slice(&radio_config().vendor_id.to_le_bytes());
            val[3] = 0x11; // par[0] = CW lo
            val[4] = 0x22; // par[1] = CW hi
            val[5] = 0x33; // par[2] = WW lo
            val[6] = 0x44; // par[3] = WW hi
            val[7] = 0x55; // par[4] = Brightness lo
            val[8] = 0x66; // par[5] = Brightness hi
        }

        rf_link_slave_add_status(&packet);

        assert_eq!(
            DEVICE_STATUS_BUFFER_WRITE_POINTER.get(),
            1,
            "Write pointer should advance"
        );

        let buf = BUFF_RESPONSE.lock();
        let st = &buf[0];

        // dat[0..3] = sno
        assert_eq!(
            &st.att_data().dat[0..3],
            &[0xAA, 0xBB, 0xCC],
            "sno mismatch"
        );
        // dat[3..5] = src_adr (overwritten from dst then from src at the end)
        assert_eq!(st.att_data().dat[3], 0x1F, "src_adr low byte");
        assert_eq!(st.att_data().dat[4], 0x00, "src_adr high byte");
        // dat[5..7] = src_adr (original src copy)
        assert_eq!(st.att_data().dat[5], 0x1F, "dat[5] src low");
        assert_eq!(st.att_data().dat[6], 0x00, "dat[6] src high");
        // dat[7] = op — MUST come before vendor_id so Python parser reads it at index 0
        // of the decrypted payload (after sno+src+mic are stripped)
        assert_eq!(
            st.att_data().dat[7],
            LGT_CMD_LIGHT_STATUS | 0xc0,
            "dat[7] must be opcode (not vendor_id)"
        );
        // dat[8..10] = vendor_id (little-endian)
        assert_eq!(
            st.att_data().dat[8],
            (radio_config().vendor_id & 0xFF) as u8,
            "vendor_id lo"
        );
        assert_eq!(
            st.att_data().dat[9],
            ((radio_config().vendor_id >> 8) & 0xFF) as u8,
            "vendor_id hi"
        );
        // dat[10..16] = unshifted CW / WW / Brightness bytes
        assert_eq!(
            &st.att_data().dat[10..16],
            &[0x11, 0x22, 0x33, 0x44, 0x55, 0x66],
            "par[0..6] must not be shifted by MeshPkt padding"
        );
    }

    // ================================================================================

    /// Tests transmission buffer ready status with empty buffer.
    #[test]
    fn test_is_add_packet_buf_ready_empty_buffer() {
        reset_host_fakes();
        FakeMmio::set_reg8(REG_DMA_TX_WPTR, 0);
        FakeMmio::set_reg8(REG_DMA_TX_RPTR, 0);

        let result = is_add_packet_buf_ready();

        assert!(result, "Empty buffer should be ready for packets");
    }

    /// Tests transmission buffer ready status with buffer near capacity.
    #[test]
    fn test_is_add_packet_buf_ready_near_full() {
        reset_host_fakes();
        FakeMmio::set_reg8(REG_DMA_TX_WPTR, 5);
        FakeMmio::set_reg8(REG_DMA_TX_RPTR, 2);

        let result = is_add_packet_buf_ready();

        // (5 - 2) & 7 = 3, which is not < 3
        assert!(!result, "Buffer with 3+ packets should not be ready");
    }

    /// Tests transmission buffer ready status with buffer having capacity.
    #[test]
    fn test_is_add_packet_buf_ready_has_capacity() {
        reset_host_fakes();
        FakeMmio::set_reg8(REG_DMA_TX_WPTR, 4);
        FakeMmio::set_reg8(REG_DMA_TX_RPTR, 2);

        let result = is_add_packet_buf_ready();

        // (4 - 2) & 7 = 2, which is < 3
        assert!(result, "Buffer with <3 packets should be ready");
    }

    // ================================================================================
    // Tests for rf_link_add_tx_packet function
    // ================================================================================

    /// Tests successful packet transmission queuing.
    #[test]
    fn test_rf_link_add_tx_packet_success() {
        reset_host_fakes();
        FakeMmio::set_reg8(REG_DMA_TX_WPTR, 2);
        FakeMmio::set_reg8(REG_DMA_TX_RPTR, 0);

        let packet = create_test_packet(LGT_CMD_LIGHT_ONOFF, [0x01, 0x02, 0x03], 0x1234, 0x5678);

        let result = rf_link_add_tx_packet(&packet);

        assert!(result.is_ok(), "Should successfully queue packet");

        // Verify DMA registration was called exactly once (widx != 0, so no
        // empty-buffer marker write)
        assert_eq!(dma_tx_fifo_writes(), 1, "one FIFO registration expected");
    }

    /// Tests packet rejection when buffer is full.
    #[test]
    fn test_rf_link_add_tx_packet_buffer_full() {
        reset_host_fakes();
        FakeMmio::set_reg8(REG_DMA_TX_WPTR, 7);
        FakeMmio::set_reg8(REG_DMA_TX_RPTR, 3);

        let packet = create_test_packet(LGT_CMD_LIGHT_ONOFF, [0x01, 0x02, 0x03], 0x1234, 0x5678);

        let result = rf_link_add_tx_packet(&packet);

        // (7 - 3) % 8 = 4, which is >= 4 (threshold)
        assert!(result.is_err(), "Should reject packet when buffer is full");
    }

    /// Tests empty buffer initialization.
    #[test]
    fn test_rf_link_add_tx_packet_empty_buffer_init() {
        reset_host_fakes();
        FakeMmio::set_reg8(REG_DMA_TX_WPTR, 0);
        FakeMmio::set_reg8(REG_DMA_TX_RPTR, 0);

        let packet = create_test_packet(LGT_CMD_LIGHT_ONOFF, [0x01, 0x02, 0x03], 0x1234, 0x5678);

        let result = rf_link_add_tx_packet(&packet);

        assert!(
            result.is_ok(),
            "Should successfully initialize empty buffer and add packet"
        );

        // Verify DMA was called twice: the empty-buffer marker, then the packet
        assert_eq!(dma_tx_fifo_writes(), 2, "marker + packet registrations");
    }

    // ================================================================================
    // Tests for parse_ble_packet_op_params function
    // ================================================================================

    /// Tests parsing single-byte operation codes.
    #[test]
    fn test_parse_ble_packet_op_params_single_byte_op() {
        // Create a packet with shorter length to fit within parameter limits
        let packet = Packet {
            att_cmd: PacketAttCmd {
                head: PacketL2capHead {
                    dma_len: 0x1A, // 26 bytes
                    _type: 2,
                    rf_len: 0x18,    // 24 bytes
                    l2cap_len: 0x14, // 20 bytes (10 header + 1 opcode + 9 params = 20)
                    chan_id: 4,
                },
                opcode: 0x12,
                handle: 0,
                handle1: 0,
                value: PacketAttValue {
                    sno: [0x01, 0x02, 0x03],
                    src: [0x34, 0x12],
                    dst: [0x78, 0x56],
                    val: {
                        let mut val = [0u8; 23];
                        val[0] = 0x30; // Single-byte opcode (0x30 & 0x80 == 0)
                        val[1] = (radio_config().vendor_id & 0xFF) as u8;
                        val[2] = ((radio_config().vendor_id >> 8) & 0xFF) as u8;
                        // Add some test parameters
                        val[3..12]
                            .iter_mut()
                            .enumerate()
                            .for_each(|(i, v)| *v = (i + 3) as u8);
                        val
                    },
                },
            },
        };

        let (success, op_codes, op_len, _parameters, params_len) =
            parse_ble_packet_op_params(&packet, false);

        assert!(success, "Should successfully parse single-byte opcode");
        assert_eq!(op_len, 1, "Should detect single-byte opcode");
        assert_eq!(op_codes[0], 0x30, "Should extract correct opcode");
        assert_eq!(params_len, 9, "Should extract 9 parameters");
    }

    /// Tests that two-byte operation codes are rejected (not supported).
    #[test]
    fn test_parse_ble_packet_op_params_two_byte_op_rejected() {
        // Create a packet with 2-byte opcode format (should be rejected)
        let packet = Packet {
            att_cmd: PacketAttCmd {
                head: PacketL2capHead {
                    dma_len: 0x1B, // 27 bytes
                    _type: 2,
                    rf_len: 0x19,    // 25 bytes
                    l2cap_len: 0x15, // 21 bytes
                    chan_id: 4,
                },
                opcode: 0x12,
                handle: 0,
                handle1: 0,
                value: PacketAttValue {
                    sno: [0x01, 0x02, 0x03],
                    src: [0x34, 0x12],
                    dst: [0x78, 0x56],
                    val: {
                        let mut val = [0u8; 23];
                        val[0] = 0x80; // Two-byte opcode format (0x80 & 0x80 != 0, 0x80 & 0x40 == 0)
                        val[1] = 0x45; // Second opcode byte
                        val[2] = (radio_config().vendor_id & 0xFF) as u8;
                        val
                    },
                },
            },
        };

        let (success, _op_codes, _op_len, _parameters, _params_len) =
            parse_ble_packet_op_params(&packet, false);

        assert!(!success, "Should reject two-byte opcode format");
    }

    /// Tests parsing three-byte operation codes.
    #[test]
    fn test_parse_ble_packet_op_params_three_byte_op() {
        // Create a packet with shorter length to fit within parameter limits
        let packet = Packet {
            att_cmd: PacketAttCmd {
                head: PacketL2capHead {
                    dma_len: 0x1C, // 28 bytes
                    _type: 2,
                    rf_len: 0x1A,    // 26 bytes
                    l2cap_len: 0x16, // 22 bytes (10 header + 3 opcode + 9 params = 22)
                    chan_id: 4,
                },
                opcode: 0x12,
                handle: 0,
                handle1: 0,
                value: PacketAttValue {
                    sno: [0x01, 0x02, 0x03],
                    src: [0x34, 0x12],
                    dst: [0x78, 0x56],
                    val: {
                        let mut val = [0u8; 23];
                        val[0] = 0xc0; // Three-byte opcode (0xc0 & 0x80 != 0, 0xc0 & 0x40 != 0)
                        val[1] = 0x45; // Second opcode byte
                        val[2] = 0x67; // Third opcode byte
                                       // Add some test parameters
                        val[3..12]
                            .iter_mut()
                            .enumerate()
                            .for_each(|(i, v)| *v = (i + 3) as u8);
                        val
                    },
                },
            },
        };

        let (success, op_codes, op_len, _parameters, params_len) =
            parse_ble_packet_op_params(&packet, false);

        assert!(success, "Should successfully parse three-byte opcode");
        assert_eq!(op_len, 3, "Should detect three-byte opcode");
        assert_eq!(
            op_codes[0], 0xc0,
            "Should extract correct first opcode byte"
        );
        assert_eq!(
            op_codes[1], 0x45,
            "Should extract correct second opcode byte"
        );
        assert_eq!(
            op_codes[2], 0x67,
            "Should extract correct third opcode byte"
        );
        assert_eq!(params_len, 9, "Should extract 9 parameters");
    }

    /// Tests parameter length validation for special opcode 6.
    #[test]
    fn test_parse_ble_packet_op_params_special_op6() {
        let mut packet = create_test_packet(0x06, [0x01, 0x02, 0x03], 0x1234, 0x5678);
        packet.att_cmd_mut().value.val[0] = 0x06 | 0xc0; // Set opcode to 6 with 3-byte format
        packet.att_cmd_mut().head.l2cap_len = 30; // Set length to test parameter extraction

        let (success, op_codes, _op_len, _parameters, _params_len) =
            parse_ble_packet_op_params(&packet, true);

        assert!(success, "Should handle special opcode 6");
        assert_eq!(op_codes[0] & 0x3f, 0x06, "Should extract opcode 6");
    }

    /// Tests parameter length overflow detection.
    #[test]
    fn test_parse_ble_packet_op_params_overflow() {
        let mut packet = create_test_packet(0x30, [0x01, 0x02, 0x03], 0x1234, 0x5678);
        packet.att_cmd_mut().head.l2cap_len = 100; // Set unreasonably large length

        let (success, _op_codes, _op_len, _parameters, params_len) =
            parse_ble_packet_op_params(&packet, false);

        assert!(!success, "Should detect parameter length overflow");
        assert_eq!(params_len, 0, "Should set params_len to 0 on overflow");
    }

    // ================================================================================
    // Tests for rf_link_slave_data function
    // ================================================================================

    /// Tests valid packet processing structure.
    #[test]
    fn test_rf_link_slave_data_valid_packet() {
        let packet = create_test_packet(LGT_CMD_LIGHT_ONOFF, [0x01, 0x02, 0x03], 0x1234, 0x5678);
        let time = 12345678;

        rf_link_slave_data(&packet, time);
    }

    /// Tests connection parameter update channel handling.
    #[test]
    fn test_rf_link_slave_data_connection_param_update() {
        let mut packet =
            create_test_packet(LGT_CMD_LIGHT_ONOFF, [0x01, 0x02, 0x03], 0x1234, 0x5678);
        packet.head_mut().chan_id = 5; // Channel 5 is for connection parameter updates
        let time = 12345678;

        rf_link_slave_data(&packet, time);

        // Function should handle connection parameter updates
    }

    /// Tests invalid channel ID rejection.
    #[test]
    fn test_rf_link_slave_data_invalid_channel() {
        let mut packet =
            create_test_packet(LGT_CMD_LIGHT_ONOFF, [0x01, 0x02, 0x03], 0x1234, 0x5678);
        packet.head_mut()._type = 2; // Data packet type
        packet.head_mut().chan_id = 10; // Invalid channel (> 6)
        let time = 12345678;

        rf_link_slave_data(&packet, time);
    }

    /// Tests zero-length packet handling.
    #[test]
    fn test_rf_link_slave_data_zero_length() {
        let mut packet =
            create_test_packet(LGT_CMD_LIGHT_ONOFF, [0x01, 0x02, 0x03], 0x1234, 0x5678);
        packet.head_mut().rf_len = 0;
        let time = 12345678;

        rf_link_slave_data(&packet, time);

        // void function: no return value to check
    }

    /// Tests BLE channel map update processing (lines 1094-1111).
    /// This tests the specific code path that handles type 3 packets with l2cap_len[0] == 1.
    #[test]
    fn test_rf_link_slave_data_channel_map_update() {
        reset_test_state();

        // Create a packet that meets the channel map update conditions:
        // 1. packet.head()._type & 3 == 3 (type 3 packet)
        // 2. packet.head().l2cap_len & 0xff == 1 (l2cap_len low byte is 1)
        let mut packet =
            create_test_packet(LGT_CMD_LIGHT_ONOFF, [0x01, 0x02, 0x03], 0x1234, 0x5678);

        // Set packet type to 3 (bottom 2 bits = 3)
        packet.head_mut()._type = 0x03; // Type 3 packet

        // Set l2cap_len low byte to 1 for channel map update detection
        packet.head_mut().l2cap_len = 0x0001; // Low byte = 1

        // Set up test data in the packet for timing instant extraction
        // The timing instant comes from: ((sno as u16) << 8) | hh as u16
        packet.ll_data_mut().sno = 0x12; // High byte of timing instant
        packet.ll_data_mut().hh = 0x34; // Low byte of timing instant

        // Set up the 5-byte channel map data that should be copied
        // The channel map starts at offset +1 from l2cap_len field
        // We need to use ptr::addr_of! to avoid unaligned reference issues
        unsafe {
            let l2cap_len_ptr = core::ptr::addr_of!(packet.head().l2cap_len) as *mut u8;
            // Write test channel map data at offset +1 from l2cap_len
            *l2cap_len_ptr.offset(1) = 0xAA; // Channel map byte 0
            *l2cap_len_ptr.offset(2) = 0xBB; // Channel map byte 1
            *l2cap_len_ptr.offset(3) = 0xCC; // Channel map byte 2
            *l2cap_len_ptr.offset(4) = 0xDD; // Channel map byte 3
            *l2cap_len_ptr.offset(5) = 0xEE; // Channel map byte 4
        }

        let time = 12345678;

        // Clear the state variables to ensure they get set by the function
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(0);
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(0);
        SLAVE_CHN_MAP.lock().fill(0);

        rf_link_slave_data(&packet, time);

        // Verify timing state updated (channel map update sets timestamp to 1)

        // Verify timing update timestamp was set to 1 (channel map change type)
        assert_eq!(
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(),
            1,
            "Should set timing update timestamp to 1 for channel map change"
        );

        // Verify next update instant was extracted correctly
        // Expected: ((0x12 as u16) << 8) | 0x34 as u16 = 0x1234
        let expected_instant = ((0x12u16) << 8) | 0x34u16;
        assert_eq!(
            BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.get(),
            expected_instant,
            "Should set next update instant from packet sno and hh fields"
        );

        // Verify the 5-byte channel map was copied correctly
        let channel_map = SLAVE_CHN_MAP.lock();
        assert_eq!(
            channel_map[0], 0xAA,
            "Channel map byte 0 should be copied correctly"
        );
        assert_eq!(
            channel_map[1], 0xBB,
            "Channel map byte 1 should be copied correctly"
        );
        assert_eq!(
            channel_map[2], 0xCC,
            "Channel map byte 2 should be copied correctly"
        );
        assert_eq!(
            channel_map[3], 0xDD,
            "Channel map byte 3 should be copied correctly"
        );
        assert_eq!(
            channel_map[4], 0xEE,
            "Channel map byte 4 should be copied correctly"
        );
    }

    // ================================================================================
    // Integration Tests for rf_link_rc_data function
    // ================================================================================

    /// Tests status advertisement processing.
    #[test]
    fn test_rf_link_rc_data_status_advertisement() {
        reset_test_state();

        let mut packet = create_status_adv_packet();

        rf_link_rc_data(&mut packet);

        // Verify packet processing completed without error
        // The actual mesh_node_update_status call is tested in integration
    }

    /// Tests status advertisement processing with a valid signature: the real
    /// `mesh_node_update_status` must append the status records the packet
    /// carries, and nothing after the terminating zero record.
    #[test]
    fn test_rf_link_rc_data_status_advertisement_with_mesh_update() {
        reset_test_state();
        reset_mesh_node_table();

        // Create a status advertisement packet with proper signature
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead {
                    dma_len: 0x27,
                    _type: 2,
                    rf_len: 0x25,
                    l2cap_len: 0x21,
                    chan_id: 0xffff, // Status advertisement channel
                },
                opcode: 0x12,
                handle: 0,
                handle1: 0,
                value: PacketAttValue {
                    sno: [0x01, 0x02, 0x03],
                    src: [0x10, 0x20],
                    dst: [0x30, 0x40],
                    val: {
                        let mut val = [0u8; 23];
                        // Set signature bytes at positions 17-20 (which map to pktdata[24-27])
                        val[17] = 0xa5;
                        val[18] = 0xa5;
                        val[19] = 0xa5;
                        val[20] = 0xa5;
                        // Add some test status data in the area that gets passed to mesh_node_update_status
                        val[0] = 0x11; // Test status data
                        val[1] = 0x22;
                        val[2] = 0x33;
                        val
                    },
                },
            },
        };

        rf_link_rc_data(&mut packet);

        // The status region is the first 24 bytes of `PacketAttValue`
        // (sno + src + dst + val[0..17]) read as six 4-byte node records:
        // 0x01, 0x20, 0x22, then a zero-address record that terminates the
        // list. The real mesh_node_update_status must append exactly those
        // three remote nodes.
        assert_eq!(
            MESH_NODE_MAX.get(),
            4,
            "the three records before the zero terminator must be appended"
        );
        {
            let mesh_node_st = MESH_NODE_ST.lock();
            for (index, expected) in [(1usize, 0x01u8), (2, 0x20), (3, 0x22)] {
                assert_eq!(
                    mesh_node_st[index].val.dev_adr, expected,
                    "appended node {index} dev_adr must come from the packet"
                );
                assert_ne!(
                    mesh_node_st[index].miss, 0,
                    "appended node {index} must be online"
                );
            }
            assert_eq!(
                mesh_node_st[1].val.sn, 0x02,
                "the second byte of each record is the node sequence number"
            );
        }
        assert_eq!(
            MESH_NODE_MASK.lock()[0] & 0b1110,
            0b1110,
            "mask bits 1..3 should be set for the appended nodes"
        );
    }

    /// Tests end-to-end status advertisement processing: rf_link_rc_data receives a
    /// status packet, calls the REAL mesh_node_update_status (not mocked), and verifies
    /// that the remote nodes actually appear in MESH_NODE_ST with MESH_NODE_MASK bits set.
    #[test]
    fn test_rf_link_rc_data_status_advertisement_end_to_end() {
        reset_test_state();
        // `clock_time()` reads the fake system tick register.
        fake_clock::set_ticks(0x12345678);

        // Set up mesh state: DEVICE_ADDRESS is 0x1234 (set by reset_test_state)
        // Low byte is 0x34, so any node with dev_adr != 0x34 will be accepted
        MESH_NODE_REPORT_ENABLE.set(true);
        reset_mesh_node_table();

        // Create a status advertisement packet with two remote node entries.
        // The status data occupies the first 24 bytes of PacketAttValue (sno + src + dst + val[0..17]).
        // Each MeshNodeStValT is 4 bytes: [dev_adr, sn, par0, par1]
        //
        // Layout in PacketAttValue:
        //   sno[0..3] + src[0..2] + dst[0..2] + val[0..17] = 24 bytes = 6 entries
        //
        // Entry 0 (bytes 0-3): sno[0]=dev_adr, sno[1]=sn, sno[2]=par0, src[0]=par1
        // Entry 1 (bytes 4-7): src[1]=dev_adr, dst[0]=sn, dst[1]=par0, val[0]=par1
        // Entry 2 (bytes 8-11): val[1..4] (set to 0 to terminate)
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead {
                    dma_len: 0x27,
                    _type: 2,
                    rf_len: 0x25,
                    l2cap_len: 0x21,
                    chan_id: 0xffff, // Status advertisement channel
                },
                opcode: 0x12,
                handle: 0,
                handle1: 0,
                value: PacketAttValue {
                    // Entry 0: node addr=0x0A, sn=1, par=[0x80, 0x00]
                    sno: [0x0A, 0x01, 0x80],
                    // Entry 0 par[1], then Entry 1: node addr=0x14, sn=5, par=[0xFF, ...]
                    src: [0x00, 0x14],
                    dst: [0x05, 0xFF],
                    val: {
                        let mut val = [0u8; 23];
                        val[0] = 0x01; // Entry 1 par[1]
                                       // val[1..4] = Entry 2: dev_adr=0 terminates the list
                                       // Signature at positions 17-20
                        val[17] = 0xa5;
                        val[18] = 0xa5;
                        val[19] = 0xa5;
                        val[20] = 0xa5;
                        val
                    },
                },
            },
        };

        rf_link_rc_data(&mut packet);

        // Verify MESH_NODE_MAX expanded to include the two new nodes
        assert_eq!(
            MESH_NODE_MAX.get(),
            3,
            "MESH_NODE_MAX should be 3 (own + 2 remote)"
        );

        // Verify MESH_NODE_ST has the remote nodes
        {
            let mesh_node_st = MESH_NODE_ST.lock();

            // Node at index 1: addr=0x0A (10), sn=1, par=[0x80, 0x00]
            assert_eq!(
                mesh_node_st[1].val.dev_adr, 0x0A,
                "First remote node dev_adr"
            );
            assert_eq!(mesh_node_st[1].val.sn, 0x01, "First remote node sn");
            assert_eq!(
                mesh_node_st[1].val.par,
                [0x80, 0x00],
                "First remote node par"
            );
            let miss1 = mesh_node_st[1].miss;
            assert_ne!(miss1, 0, "First remote node should be online");

            // Node at index 2: addr=0x14 (20), sn=5, par=[0xFF, 0x01]
            assert_eq!(
                mesh_node_st[2].val.dev_adr, 0x14,
                "Second remote node dev_adr"
            );
            assert_eq!(mesh_node_st[2].val.sn, 0x05, "Second remote node sn");
            assert_eq!(
                mesh_node_st[2].val.par,
                [0xFF, 0x01],
                "Second remote node par"
            );
            let miss2 = mesh_node_st[2].miss;
            assert_ne!(miss2, 0, "Second remote node should be online");
        }

        // Verify MESH_NODE_MASK has bits set for the new nodes
        {
            let mask = MESH_NODE_MASK.lock();
            assert_ne!(
                mask[0] & (1 << 1),
                0,
                "Mask bit 1 should be set for node at index 1"
            );
            assert_ne!(
                mask[0] & (1 << 2),
                0,
                "Mask bit 2 should be set for node at index 2"
            );
        }
    }

    /// Tests status advertisement processing with an invalid signature: the
    /// mesh node table must be left exactly as it was.
    #[test]
    fn test_rf_link_rc_data_status_advertisement_invalid_signature() {
        reset_test_state();
        reset_mesh_node_table();

        // Create a status advertisement packet with INVALID signature
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead {
                    dma_len: 0x27,
                    _type: 2,
                    rf_len: 0x25,
                    l2cap_len: 0x21,
                    chan_id: 0xffff, // Status advertisement channel
                },
                opcode: 0x12,
                handle: 0,
                handle1: 0,
                value: PacketAttValue {
                    sno: [0x01, 0x02, 0x03],
                    src: [0x10, 0x20],
                    dst: [0x30, 0x40],
                    val: {
                        let mut val = [0u8; 23];
                        // Set INVALID signature bytes (not all 0xa5)
                        val[17] = 0xa5;
                        val[18] = 0xa5;
                        val[19] = 0xa5;
                        val[20] = 0x00; // Invalid - should be 0xa5
                        val
                    },
                },
            },
        };

        rf_link_rc_data(&mut packet);

        // A rejected signature must not touch the mesh node database at all.
        assert_eq!(
            MESH_NODE_MAX.get(),
            1,
            "no node may be appended for an invalid signature"
        );
        assert_eq!(
            MESH_NODE_ST.lock()[1].val.dev_adr,
            0,
            "the first free node slot must stay empty"
        );
        assert_eq!(
            MESH_NODE_MASK.lock()[0],
            0,
            "no mask bit may be set for an invalid signature"
        );
    }

    /// Tests malformed packet early return without further processing.
    #[test]
    fn test_rf_link_rc_data_malformed_packet_early_return() {
        reset_test_state();
        reset_mesh_node_table();
        let _guard = install_recording_hooks();

        // Create a malformed packet that will cause parse_ble_packet_op_params to return success=false
        let mut packet = create_test_packet(0x30, [0x01, 0x02, 0x03], 0x1234, 0x5678);
        packet.att_cmd_mut().head.l2cap_len = 100; // Set unreasonably large length to trigger overflow

        rf_link_rc_data(&mut packet);

        // The parse failure returns before duplicate caching, UART forwarding,
        // address matching, application dispatch and the mesh database update.
        assert_eq!(light_data_calls(), 0, "on_light_data must not be called");
        assert_eq!(
            uart_mesh_rx_calls(),
            0,
            "on_uart_mesh_rx must not be called"
        );
        assert_eq!(
            RC_PKT_BUF.lock().len(),
            0,
            "a malformed packet must not enter the duplicate cache"
        );
        assert_eq!(
            MESH_NODE_MAX.get(),
            1,
            "a malformed packet must not update the mesh node table"
        );
        assert_eq!(
            crate::queues::MESH_TX_QUEUE.lock().len(),
            0,
            "a malformed packet must not be relayed"
        );
    }

    /// Tests 1-byte opcode extraction and processing.
    #[test]
    fn test_opcode_extraction_1_byte() {
        reset_test_state();
        let _guard = install_recording_hooks();

        // Create a packet with 1-byte opcode format addressed to this device
        // (0x1234): `rf_link_match_group_mac` reports a device match, so the
        // packet takes the local-processing path instead of the relay path.
        let mut packet =
            create_test_packet(LGT_CMD_LIGHT_ONOFF, [0xAA, 0xBB, 0xCC], 0x1234, 0x1234);

        rf_link_rc_data(&mut packet);

        // 1-byte opcodes use op_cmd[0] directly, so the command reaches the
        // application unchanged.
        assert_eq!(
            light_data_calls(),
            1,
            "a 1-byte opcode packet addressed to us must reach on_light_data"
        );
    }

    /// Tests 3-byte opcode extraction with masking for response packets.
    #[test]
    fn test_opcode_extraction_3_byte_response_format() {
        reset_test_state();
        let _guard = install_recording_hooks();

        // Create a packet that simulates a response with 3-byte opcode format
        // This mimics the format used in mesh/mod.rs: opcode | 0xc0
        let mut packet =
            create_test_packet(LGT_CMD_LIGHT_STATUS, [0x11, 0x22, 0x33], 0xABCD, 0x1234);

        // Update packet lengths FIRST to accommodate 3-byte opcode
        // 10 header + 3 opcode + 11 params = 24 bytes (generous sizing)
        packet.att_cmd_mut().head.l2cap_len = 0x18; // 24 bytes
        packet.att_cmd_mut().head.rf_len = 0x1C; // 28 bytes
        packet.att_cmd_mut().head.dma_len = 0x1E; // 30 bytes

        // THEN set the packet to use 3-byte opcode format
        packet.att_cmd_mut().value.val[0] = LGT_CMD_LIGHT_STATUS | 0xc0; // 0x1b | 0xc0 = 0xdb
        packet.att_cmd_mut().value.val[1] = 0x44; // Second byte
        packet.att_cmd_mut().value.val[2] = 0x55; // Third byte

        rf_link_rc_data(&mut packet);

        // For 3-byte opcodes the masked opcode (0x1b) drives the routing, so the
        // packet is delivered locally even though its LSB byte is 0xdb.
        assert_eq!(
            light_data_calls(),
            1,
            "masked 3-byte opcode must reach on_light_data"
        );
        assert_eq!(
            RC_PKT_BUF.lock().iter().next().map(|pkt| pkt.op),
            Some(LGT_CMD_LIGHT_STATUS),
            "the duplicate cache must key on the masked opcode"
        );
    }

    /// Tests that 2-byte opcode format results in early return (not processed).
    #[test]
    fn test_opcode_extraction_2_byte_format_ignored() {
        reset_test_state();
        let _guard = install_recording_hooks();

        // Create a packet that would have 2-byte opcode format (0x80 bit set, 0x40 bit clear)
        // But since 2-byte opcodes are not supported, this should result in early return
        let mut packet =
            create_test_packet(LGT_CMD_LIGHT_STATUS, [0xDD, 0xEE, 0xFF], 0x9ABC, 0xDEF0);

        // Force the packet structure to create an unsupported 2-byte opcode scenario
        // This will be rejected by parse_ble_packet_op_params and result in early return
        packet.att_cmd_mut().value.val[0] = 0x80; // 2-byte format (unsupported)
        packet.att_cmd_mut().value.val[1] = 0x1B; // Second byte

        rf_link_rc_data(&mut packet);

        // The parse failure returns before address matching and caching.
        assert_eq!(
            light_data_calls(),
            0,
            "an unsupported 2-byte opcode must not be delivered"
        );
        assert_eq!(
            RC_PKT_BUF.lock().len(),
            0,
            "an unsupported 2-byte opcode must not be cached"
        );
    }

    /// Tests early return when BLE peripheral response doesn't match expected operation or sequence number.
    #[test]
    fn test_ble_peripheral_response_mismatch_early_return() {
        reset_test_state();
        let _guard = install_recording_hooks();

        // Set up BLE peripheral connection and status reading
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        SLAVE_READ_STATUS_BUSY.set(LGT_CMD_LIGHT_STATUS); // Expecting status response
        *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() = [0x01, 0x02, 0x03]; // Expected sequence
        DEVICE_ADDRESS.set(0x1234); // Set device address

        // Create a notify response packet that should trigger the BLE peripheral path
        let mut packet =
            create_test_packet(LGT_CMD_LIGHT_GRP_RSP1, [0x99, 0x88, 0x77], 0x5678, 0x1234);

        // rf_link_is_notify_rsp(LGT_CMD_LIGHT_GRP_RSP1) is true, dst matches our
        // address and the peripheral link is active, so the bridge branch is
        // entered - but the pending status read expects LGT_CMD_LIGHT_STATUS.

        rf_link_rc_data(&mut packet);

        // The mismatch returns before forwarding, before the address match and
        // before any local dispatch. The packet is already in the duplicate
        // cache, which proves the bridge branch was reached.
        assert_eq!(
            RC_PKT_BUF.lock().len(),
            1,
            "the packet must be cached before the bridge branch"
        );
        assert_eq!(
            DEVICE_STATUS_RECORD_INDEX.get(),
            0,
            "rf_link_slave_add_status must not record a status responder"
        );
        assert_eq!(light_data_calls(), 0, "on_light_data must not be called");
        assert_eq!(
            crate::queues::MESH_TX_QUEUE.lock().len(),
            0,
            "nothing may be queued for transmission"
        );
    }

    /// Tests early return when BLE peripheral response has wrong sequence number.
    #[test]
    fn test_ble_peripheral_sequence_mismatch_early_return() {
        reset_test_state();
        let _guard = install_recording_hooks();

        // Set up BLE peripheral connection and status reading
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        SLAVE_READ_STATUS_BUSY.set(LGT_CMD_LIGHT_GRP_RSP1); // Expecting this specific response
        *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() = [0x01, 0x02, 0x03]; // Expected sequence
        DEVICE_ADDRESS.set(0x1234); // Set device address

        // Create a notify response packet with WRONG sequence number
        let mut packet =
            create_test_packet(LGT_CMD_LIGHT_GRP_RSP1, [0x99, 0x88, 0x77], 0x5678, 0x1234);

        // Operation matches (LGT_CMD_LIGHT_GRP_RSP1 == LGT_CMD_LIGHT_GRP_RSP1)
        // But sequence number is wrong: [0x99, 0x88, 0x77] != [0x01, 0x02, 0x03]

        rf_link_rc_data(&mut packet);

        // The mismatch returns before forwarding and before local dispatch.
        assert_eq!(
            RC_PKT_BUF.lock().len(),
            1,
            "the packet must be cached before the bridge branch"
        );
        assert_eq!(
            DEVICE_STATUS_RECORD_INDEX.get(),
            0,
            "rf_link_slave_add_status must not record a status responder"
        );
        assert_eq!(light_data_calls(), 0, "on_light_data must not be called");
        assert_eq!(
            crate::queues::MESH_TX_QUEUE.lock().len(),
            0,
            "nothing may be queued for transmission"
        );
    }

    /// Tests duplicate packet filtering behavior.
    #[test]
    fn test_rf_link_rc_data_duplicate_filtering() {
        reset_test_state();

        // Addressed to this device (0x1234), so the packet is processed locally
        // instead of being relayed.
        let mut packet =
            create_test_packet(LGT_CMD_LIGHT_ONOFF, [0x01, 0x02, 0x03], 0x1234, 0x1234);

        // Process packet first time
        rf_link_rc_data(&mut packet);

        // Check that packet was added to buffer
        let rc_pkt_buf = RC_PKT_BUF.lock();
        assert_eq!(
            rc_pkt_buf.len(),
            1,
            "First packet should be added to buffer"
        );
        drop(rc_pkt_buf);

        // Process same packet again - should be filtered as duplicate
        rf_link_rc_data(&mut packet);

        // Buffer should still only contain one packet (duplicate was filtered)
        let rc_pkt_buf = RC_PKT_BUF.lock();
        assert_eq!(rc_pkt_buf.len(), 1, "Duplicate packet should be filtered");
    }

    /// Tests BLE-mesh bridge response forwarding setup.
    #[test]
    fn test_rf_link_rc_data_response_forwarding() {
        reset_test_state();

        // Set up BLE peripheral connection and status reading
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        SLAVE_READ_STATUS_BUSY.set(LGT_CMD_LIGHT_STATUS);
        *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() = [0x01, 0x02, 0x03];

        let mut packet =
            create_test_packet(LGT_CMD_LIGHT_STATUS, [0x01, 0x02, 0x03], 0x1234, 0x1234);

        rf_link_rc_data(&mut packet);

        // The matching response must be buffered as a status responder for the
        // waiting BLE peripheral connection.
        assert_eq!(
            DEVICE_STATUS_RECORD_INDEX.get(),
            1,
            "one status responder should be recorded"
        );
        assert_eq!(
            DEVICE_STATUS_BUFFER_WRITE_POINTER.get(),
            1,
            "the response buffer write pointer should advance"
        );
    }

    /// Tests local packet processing structure.
    #[test]
    fn test_rf_link_rc_data_local_processing_with_ack() {
        reset_test_state();
        let _guard = install_recording_hooks();
        // Seed the fake tick: the ACK sequence number is clock_time() + device address
        fake_clock::set_ticks(12345);

        // Use peripheral mode to avoid hardware register access
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);

        let mut packet =
            create_test_packet(LGT_CMD_LIGHT_ONOFF, [0x01, 0x02, 0x03], 0x1234, 0x1234);
        // Set ACK request parameter in mesh internal_par1 field
        packet.mesh_mut().internal_par1[INTERNAL_PAR_SEND_ACK] = 1; // Request ACK

        rf_link_rc_data(&mut packet);

        // Verify local processing callback was called
        assert_eq!(light_data_calls(), 1, "on_light_data must be called once");

        // Verify the UART bridge saw the same packet before it was filtered for
        // local processing (this used to be the un-mocked `light_mesh_rx_cb`).
        assert_eq!(
            uart_mesh_rx_calls(),
            1,
            "on_uart_mesh_rx must be called once"
        );

        // Verify ACK generation consumed clock_time(): the status packet's
        // sequence number is clock_time() + DEVICE_ADDRESS, truncated to 3 bytes.
        let expected_sno = (12345u32 + 0x1234u32).to_le_bytes();
        assert_eq!(
            PKT_LIGHT_STATUS.lock().att_cmd().value.sno,
            [expected_sno[0], expected_sno[1], expected_sno[2]],
            "ACK sequence number must be derived from the fake clock"
        );

        // Verify ACK was queued for mesh transmission
        assert_eq!(crate::queues::MESH_TX_QUEUE.lock().len(), 1);
    }

    /// Tests mesh relay packet marking.
    #[test]
    fn test_rf_link_rc_data_mesh_relay() {
        reset_test_state();

        // No group membership and an address that is neither ours nor a
        // broadcast: `rf_link_match_group_mac` reports (false, false), which
        // triggers the relay path.
        // `read_reg_rnd_number()` reads the fake register bank directly.
        FakeMmio::set_reg16(0x448, 42);
        // clock_time64() is clock_time() plus its overflow accumulator, and
        // clock_time() is the fake system tick register.
        fake_clock::set_ticks(1_000_000);

        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(false); // Mesh-only mode for random delay

        let mut packet =
            create_test_packet(LGT_CMD_LIGHT_ONOFF, [0x01, 0x02, 0x03], 0x5678, 0x9ABC);

        rf_link_rc_data(&mut packet);

        // Verify packet was marked for mesh transmission (bit 7 set)
        assert_eq!(
            packet.head()._type & BIT!(7),
            BIT!(7),
            "Packet should be marked for mesh transmission"
        );

        // Verify the relay was queued for mesh transmission
        assert_eq!(crate::queues::MESH_TX_QUEUE.lock().len(), 1);
    }

    /// Tests notification request response structure.
    #[test]
    fn test_rf_link_rc_data_notification_response() {
        reset_test_state();
        let _guard = install_recording_hooks();

        // Use peripheral mode to avoid hardware register access
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);

        let mut packet = create_test_packet(
            LGT_CMD_LIGHT_READ_STATUS,
            [0x04, 0x05, 0x06],
            0x1234,
            0x1234,
        );

        // Set up packet parameters to ensure response callback is triggered
        // For single-byte opcode, params[0] = val[1], params[1] = val[2], etc.
        packet.att_cmd_mut().value.val[2] = 0x01; // Ensure params[1] != 0 condition is met
        packet.att_cmd_mut().value.val[9] = 0x42; // Set status parameter for response

        rf_link_rc_data(&mut packet);

        // Verify callbacks were called
        assert_eq!(light_data_calls(), 1, "on_light_data must be called once");
        assert_eq!(
            light_response_calls(),
            1,
            "on_light_response must be called once"
        );
        assert_eq!(
            crate::queues::MESH_TX_QUEUE.lock().len(),
            1,
            "the accepted response must be queued for transmission"
        );
    }

    /// Tests complex workflow processing with proper callback verification.
    #[test]
    fn test_rf_link_rc_data_complex_workflow() {
        reset_test_state();
        let _guard = install_recording_hooks();

        // Create an ATT packet for the complex workflow test; it is addressed to
        // this device, and a fresh duplicate cache means the real duplicate
        // detection lets it through.
        let mut packet =
            create_test_packet(LGT_CMD_LIGHT_GRP_REQ, [0x04, 0x05, 0x06], 0x1234, 0x1234);

        // Set up packet parameters to ensure response callback is triggered
        // For single-byte opcode, params[0] = val[1], params[1] = val[2], etc.
        packet.att_cmd_mut().value.val[1] = 0x01; // params[0] - non-zero to trigger response
        packet.att_cmd_mut().value.val[2] = 0x01; // params[1] - non-zero for condition check

        rf_link_rc_data(&mut packet);

        // Verify that local processing was triggered
        assert_eq!(light_data_calls(), 1, "on_light_data must be called once");

        // Verify that response generation was triggered for notify request
        assert_eq!(
            light_response_calls(),
            1,
            "on_light_response must be called once"
        );

        // The request must be cached and marked as notified by the real
        // `rc_pkt_buf_push` / `req_cmd_set_notify_ok_flag`, and the accepted
        // response queued.
        {
            let rc_pkt_buf = RC_PKT_BUF.lock();
            assert_eq!(rc_pkt_buf.len(), 1, "the request must be cached once");
            assert_eq!(
                rc_pkt_buf
                    .iter()
                    .next()
                    .map(|pkt| (pkt.op, pkt.notify_ok_flag)),
                Some((LGT_CMD_LIGHT_GRP_REQ, true)),
                "the cached request must be flagged as notified"
            );
        }
        assert_eq!(
            crate::queues::MESH_TX_QUEUE.lock().len(),
            1,
            "the accepted response must be queued for transmission"
        );
    }

    // ================================================================================
    // Tests for rf_link_slave_notify_req_mask function
    // ================================================================================

    /// Tests notification request masking in broadcast mode with new address.
    #[test]
    fn test_rf_link_slave_notify_req_mask_broadcast_mode_new_address() {
        reset_test_state();

        // Set up broadcast mode and active status reading
        DEVICE_STATUS_READ_UNICAST_MODE.set(false);
        SLAVE_READ_STATUS_BUSY.set(1);
        DEVICE_ADDRESS.set(0x1234);
        NOTIFICATION_REQUEST_MASK_INDEX.set(0);

        // Call function with new address
        rf_link_slave_notify_req_mask(0x56);

        // Verify address was added to notification mask at index 8 (0 + 8)
        let pkt_data = PKT_LIGHT_DATA.lock();
        assert_eq!(
            pkt_data.att_cmd().value.val[8],
            0x56,
            "Address should be added to mask at index 8"
        );
        drop(pkt_data);

        // Verify round-robin index was advanced
        assert_eq!(
            NOTIFICATION_REQUEST_MASK_INDEX.get(),
            1,
            "Round-robin index should advance to 1"
        );
    }

    /// Tests notification request masking in broadcast mode with duplicate address.
    #[test]
    fn test_rf_link_slave_notify_req_mask_broadcast_mode_duplicate_address() {
        reset_test_state();

        // Set up broadcast mode and active status reading
        DEVICE_STATUS_READ_UNICAST_MODE.set(false);
        SLAVE_READ_STATUS_BUSY.set(1);
        DEVICE_ADDRESS.set(0x1234);
        NOTIFICATION_REQUEST_MASK_INDEX.set(0);

        // Pre-populate mask with address 0x56 at index 9
        PKT_LIGHT_DATA.lock().att_cmd_mut().value.val[9] = 0x56;

        // Call function with duplicate address
        rf_link_slave_notify_req_mask(0x56);

        // Verify round-robin index was NOT advanced (no new address added)
        assert_eq!(
            NOTIFICATION_REQUEST_MASK_INDEX.get(),
            0,
            "Round-robin index should not advance for duplicates"
        );

        // Verify original address position unchanged
        let pkt_data = PKT_LIGHT_DATA.lock();
        assert_eq!(
            pkt_data.att_cmd().value.val[9],
            0x56,
            "Original address should remain at index 9"
        );
        assert_eq!(
            pkt_data.att_cmd().value.val[8],
            0,
            "Index 8 should remain empty"
        );
    }

    /// Tests notification request masking round-robin wraparound in broadcast mode.
    #[test]
    fn test_rf_link_slave_notify_req_mask_broadcast_mode_wraparound() {
        reset_test_state();

        // Set up broadcast mode and active status reading
        DEVICE_STATUS_READ_UNICAST_MODE.set(false);
        SLAVE_READ_STATUS_BUSY.set(1);
        DEVICE_ADDRESS.set(0x1234);
        NOTIFICATION_REQUEST_MASK_INDEX.set(4); // At end of circular buffer

        // Call function with new address
        rf_link_slave_notify_req_mask(0x78);

        // Verify address was added to notification mask at index 12 (4 + 8)
        let pkt_data = PKT_LIGHT_DATA.lock();
        assert_eq!(
            pkt_data.att_cmd().value.val[12],
            0x78,
            "Address should be added to mask at index 12"
        );
        drop(pkt_data);

        // Verify round-robin index wrapped around to 0
        assert_eq!(
            NOTIFICATION_REQUEST_MASK_INDEX.get(),
            0,
            "Round-robin index should wrap to 0"
        );
    }

    /// Tests notification request masking in unicast mode.
    #[test]
    fn test_rf_link_slave_notify_req_mask_unicast_mode() {
        reset_test_state();

        // Set up unicast mode and active status reading
        DEVICE_STATUS_READ_UNICAST_MODE.set(true);
        SLAVE_READ_STATUS_BUSY.set(1);
        DEVICE_ADDRESS.set(0x1234);
        SLAVE_DATA_VALID.set(1); // Set to 1 to verify it gets reset

        // Call function with address
        rf_link_slave_notify_req_mask(0x56);

        // Verify SLAVE_DATA_VALID was reset to force data refresh
        assert_eq!(
            SLAVE_DATA_VALID.get(),
            0,
            "SLAVE_DATA_VALID should be reset to 0 in unicast mode"
        );

        // Verify notification mask was not modified (unicast mode doesn't use mask)
        let pkt_data = PKT_LIGHT_DATA.lock();
        assert_eq!(
            pkt_data.att_cmd().value.val[8],
            0,
            "Notification mask should not be modified in unicast mode"
        );
    }

    /// Tests notification request masking with inactive status reading.
    #[test]
    fn test_rf_link_slave_notify_req_mask_inactive_status_reading() {
        reset_test_state();

        // Set up with inactive status reading
        DEVICE_STATUS_READ_UNICAST_MODE.set(false);
        SLAVE_READ_STATUS_BUSY.set(0); // Inactive
        DEVICE_ADDRESS.set(0x1234);
        NOTIFICATION_REQUEST_MASK_INDEX.set(0);

        // Call function with address
        rf_link_slave_notify_req_mask(0x56);

        // Verify no action was taken (mask not modified)
        let pkt_data = PKT_LIGHT_DATA.lock();
        assert_eq!(
            pkt_data.att_cmd().value.val[8],
            0,
            "Notification mask should not be modified when status reading inactive"
        );
        drop(pkt_data);

        // Verify round-robin index was not advanced
        assert_eq!(
            NOTIFICATION_REQUEST_MASK_INDEX.get(),
            0,
            "Round-robin index should not advance when inactive"
        );
    }

    /// Tests notification request masking with same device address (special case 0x21).
    #[test]
    fn test_rf_link_slave_notify_req_mask_same_device_address_special_case() {
        reset_test_state();

        // Set up broadcast mode with special status value 0x21
        DEVICE_STATUS_READ_UNICAST_MODE.set(false);
        SLAVE_READ_STATUS_BUSY.set(0x21); // Special case that allows same device address
        DEVICE_ADDRESS.set(0x1234);
        NOTIFICATION_REQUEST_MASK_INDEX.set(0);

        // Call function with same device address (lower byte)
        rf_link_slave_notify_req_mask(0x34);

        // Verify address was added despite being same device (special case)
        let pkt_data = PKT_LIGHT_DATA.lock();
        assert_eq!(
            pkt_data.att_cmd().value.val[8],
            0x34,
            "Address should be added even for same device in special case"
        );
        drop(pkt_data);

        // Verify round-robin index was advanced
        assert_eq!(
            NOTIFICATION_REQUEST_MASK_INDEX.get(),
            1,
            "Round-robin index should advance in special case"
        );
    }

    /// Tests notification request masking with same device address (normal case blocked).
    #[test]
    fn test_rf_link_slave_notify_req_mask_same_device_address_blocked() {
        reset_test_state();

        // Set up broadcast mode with normal status value
        DEVICE_STATUS_READ_UNICAST_MODE.set(false);
        SLAVE_READ_STATUS_BUSY.set(1); // Normal status (not 0x21)
        DEVICE_ADDRESS.set(0x1234);
        NOTIFICATION_REQUEST_MASK_INDEX.set(0);

        // Call function with same device address (lower byte)
        rf_link_slave_notify_req_mask(0x34);

        // Verify no action was taken (same device address blocked in normal case)
        let pkt_data = PKT_LIGHT_DATA.lock();
        assert_eq!(
            pkt_data.att_cmd().value.val[8],
            0,
            "Address should not be added for same device in normal case"
        );
        drop(pkt_data);

        // Verify round-robin index was not advanced
        assert_eq!(
            NOTIFICATION_REQUEST_MASK_INDEX.get(),
            0,
            "Round-robin index should not advance when blocked"
        );
    }

    /// Tests notification response generation for LGT_CMD_LIGHT_CONFIG_GRP.
    #[test]
    fn test_notification_response_light_config_grp() {
        reset_test_state();
        let _guard = install_recording_hooks();

        // Use peripheral mode to avoid hardware register access
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);

        let mut packet =
            create_test_packet(LGT_CMD_LIGHT_CONFIG_GRP, [0x04, 0x05, 0x06], 0x1234, 0x1234);

        // Set up packet parameters to ensure response callback is triggered
        // For single-byte opcode, params[0] = val[1], params[1] = val[2], etc.
        packet.att_cmd_mut().value.val[2] = 0x01; // Ensure params[1] != 0 condition is met
        packet.att_cmd_mut().value.val[9] = 0x42; // Set status parameter for response

        rf_link_rc_data(&mut packet);

        // Verify callbacks were called
        assert_eq!(light_data_calls(), 1, "on_light_data must be called once");
        assert_eq!(
            light_response_calls(),
            1,
            "on_light_response must be called once"
        );

        // Check that the response value was set correctly
        let pkt_status = PKT_LIGHT_STATUS.lock();
        assert_eq!(
            pkt_status.att_cmd().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE],
            GET_GROUP1,
            "Response should contain GET_GROUP1"
        );
    }

    /// Tests notification response generation for LGT_CMD_CONFIG_DEV_ADDR.
    #[test]
    fn test_notification_response_config_dev_addr() {
        reset_test_state();
        let _guard = install_recording_hooks();

        // Use peripheral mode to avoid hardware register access
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);

        let mut packet =
            create_test_packet(LGT_CMD_CONFIG_DEV_ADDR, [0x04, 0x05, 0x06], 0x1234, 0x1234);

        // Set up packet parameters to ensure response callback is triggered
        // For single-byte opcode, params[0] = val[1], params[1] = val[2], etc.
        packet.att_cmd_mut().value.val[2] = 0x01; // Ensure params[1] != 0 condition is met
        packet.att_cmd_mut().value.val[9] = 0x42; // Set status parameter for response

        rf_link_rc_data(&mut packet);

        // Verify callbacks were called
        assert_eq!(light_data_calls(), 1, "on_light_data must be called once");
        assert_eq!(
            light_response_calls(),
            1,
            "on_light_response must be called once"
        );

        // Check that the response value was set correctly
        let pkt_status = PKT_LIGHT_STATUS.lock();
        assert_eq!(
            pkt_status.att_cmd().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE],
            GET_DEV_ADDR,
            "Response should contain GET_DEV_ADDR"
        );
    }

    /// Tests notification response generation for LGT_CMD_USER_NOTIFY_REQ.
    #[test]
    fn test_notification_response_user_notify_req() {
        reset_test_state();
        let _guard = install_recording_hooks();

        // Use peripheral mode to avoid hardware register access
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);

        let mut packet =
            create_test_packet(LGT_CMD_USER_NOTIFY_REQ, [0x04, 0x05, 0x06], 0x1234, 0x1234);

        // Set up packet parameters to ensure response callback is triggered
        // For single-byte opcode, params[0] = val[1], params[1] = val[2], etc.
        packet.att_cmd_mut().value.val[2] = 0x01; // Ensure params[1] != 0 condition is met
        packet.att_cmd_mut().value.val[9] = 0x42; // Set status parameter for response

        rf_link_rc_data(&mut packet);

        // Verify callbacks were called
        assert_eq!(light_data_calls(), 1, "on_light_data must be called once");
        assert_eq!(
            light_response_calls(),
            1,
            "on_light_response must be called once"
        );

        // Check that the response value was set correctly
        let pkt_status = PKT_LIGHT_STATUS.lock();
        assert_eq!(
            pkt_status.att_cmd().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE],
            GET_USER_NOTIFY,
            "Response should contain GET_USER_NOTIFY"
        );
    }

    /// Tests notification response generation for LGT_CMD_START_OTA_REQ.
    #[test]
    fn test_notification_response_start_ota_req() {
        reset_test_state();
        let _guard = install_recording_hooks();

        // Use peripheral mode to avoid hardware register access
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);

        let mut packet =
            create_test_packet(LGT_CMD_START_OTA_REQ, [0x04, 0x05, 0x06], 0x1234, 0x1234);

        // Set up packet parameters to ensure response callback is triggered
        // For single-byte opcode, params[0] = val[1], params[1] = val[2], etc.
        packet.att_cmd_mut().value.val[2] = 0x01; // Ensure params[1] != 0 condition is met
        packet.att_cmd_mut().value.val[9] = 0x42; // Set status parameter for response

        rf_link_rc_data(&mut packet);

        // Verify callbacks were called
        assert_eq!(light_data_calls(), 1, "on_light_data must be called once");
        assert_eq!(
            light_response_calls(),
            1,
            "on_light_response must be called once"
        );

        // Check that the response value was set correctly
        let pkt_status = PKT_LIGHT_STATUS.lock();
        assert_eq!(
            pkt_status.att_cmd().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE],
            CMD_START_OTA,
            "Response should contain CMD_START_OTA"
        );
    }

    /// Tests notification response generation for LGT_CMD_OTA_DATA_REQ.
    #[test]
    fn test_notification_response_ota_data_req() {
        reset_test_state();
        let _guard = install_recording_hooks();

        // Use peripheral mode to avoid hardware register access
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);

        let mut packet =
            create_test_packet(LGT_CMD_OTA_DATA_REQ, [0x04, 0x05, 0x06], 0x1234, 0x1234);

        // Set up packet parameters to ensure response callback is triggered
        // For single-byte opcode, params[0] = val[1], params[1] = val[2], etc.
        packet.att_cmd_mut().value.val[2] = 0x01; // Ensure params[1] != 0 condition is met
        packet.att_cmd_mut().value.val[9] = 0x42; // Set status parameter for response

        rf_link_rc_data(&mut packet);

        // Verify callbacks were called
        assert_eq!(light_data_calls(), 1, "on_light_data must be called once");
        assert_eq!(
            light_response_calls(),
            1,
            "on_light_response must be called once"
        );

        // Check that the response value was set correctly
        let pkt_status = PKT_LIGHT_STATUS.lock();
        assert_eq!(
            pkt_status.att_cmd().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE],
            CMD_OTA_DATA,
            "Response should contain CMD_OTA_DATA"
        );
    }

    /// Tests notification response generation for LGT_CMD_END_OTA_REQ.
    #[test]
    fn test_notification_response_end_ota_req() {
        reset_test_state();
        let _guard = install_recording_hooks();

        // Use peripheral mode to avoid hardware register access
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);

        let mut packet =
            create_test_packet(LGT_CMD_END_OTA_REQ, [0x04, 0x05, 0x06], 0x1234, 0x1234);

        // Set up packet parameters to ensure response callback is triggered
        // For single-byte opcode, params[0] = val[1], params[1] = val[2], etc.
        packet.att_cmd_mut().value.val[2] = 0x01; // Ensure params[1] != 0 condition is met
        packet.att_cmd_mut().value.val[9] = 0x42; // Set status parameter for response

        rf_link_rc_data(&mut packet);

        // Verify callbacks were called
        assert_eq!(light_data_calls(), 1, "on_light_data must be called once");
        assert_eq!(
            light_response_calls(),
            1,
            "on_light_response must be called once"
        );

        // Check that the response value was set correctly
        let pkt_status = PKT_LIGHT_STATUS.lock();
        assert_eq!(
            pkt_status.att_cmd().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE],
            CMD_END_OTA,
            "Response should contain CMD_END_OTA"
        );
    }

    /// Tests fallthrough to line 979 where notification response transmission is skipped.
    /// This occurs when the condition on line 955 evaluates to false:
    /// !(not_slave_message || BLE_PERIPHERAL_LINK_COMMAND.get() != op || params[1] != 0)
    #[test]
    fn test_notification_response_transmission_skipped() {
        reset_test_state();
        let _guard = install_recording_hooks();

        // Use peripheral mode to avoid hardware register access
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);

        let mut packet = create_test_packet(
            LGT_CMD_LIGHT_READ_STATUS,
            [0x01, 0x02, 0x03], // Same sequence number as GENERAL_MESSAGE_SEQUENCE_NUMBER
            0x1234,
            0x1234,
        );

        // Set up conditions to make the transmission condition false:
        // 1. not_slave_message = false (packet sno matches GENERAL_MESSAGE_SEQUENCE_NUMBER)
        *GENERAL_MESSAGE_SEQUENCE_NUMBER.lock() = [0x01, 0x02, 0x03];
        // 2. BLE_PERIPHERAL_LINK_COMMAND.get() == op (same command)
        BLE_PERIPHERAL_LINK_COMMAND.set(LGT_CMD_LIGHT_READ_STATUS);
        // 3. params[1] == 0 (parameter is zero)
        packet.att_cmd_mut().value.val[2] = 0x00; // params[1] = val[2] = 0

        rf_link_rc_data(&mut packet);

        // Verify local processing callback was called
        assert_eq!(light_data_calls(), 1, "on_light_data must be called once");

        // Verify response callback was NOT called (fell through to line 979)
        assert_eq!(
            light_response_calls(),
            0,
            "on_light_response must not be called"
        );

        // Verify mesh message was NOT sent (fell through to line 979)
        assert_eq!(
            crate::queues::MESH_TX_QUEUE.lock().len(),
            0,
            "no response may be queued"
        );
    }
}
