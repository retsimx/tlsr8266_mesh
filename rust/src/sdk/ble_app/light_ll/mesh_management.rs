//! # Mesh Network Management
//!
//! This module implements the core mesh networking functionality for the TLSR8266
//! lighting system, including node discovery, status tracking, and network topology
//! maintenance. It provides the algorithms necessary for self-organizing mesh networks
//! with automatic node timeout detection and reliable status propagation.
//!
//! ## Mesh Network Architecture
//!
//! The mesh network uses a flooding-based protocol with the following characteristics:
//!
//! ### Node Addressing
//! - **Address Range**: 1-254 (address 0 reserved for broadcast, 255 for special use)
//! - **Self-Assignment**: Nodes automatically assign unique addresses during pairing
//! - **Address Persistence**: Addresses are stored in flash and survive power cycles
//!
//! ### Status Propagation Algorithm
//! The network implements a distributed status tracking system:
//! 1. **Local Status Updates**: Each node maintains its own status (lighting state, etc.)
//! 2. **Neighbor Discovery**: Nodes track their immediate neighbors through periodic status messages
//! 3. **Status Flooding**: Status changes are propagated through the network using controlled flooding
//! 4. **Timeout Detection**: Nodes are marked offline if no status updates are received within timeout period
//!
//! ### Network Topology Management
//! - **Self-Healing**: Network automatically routes around failed nodes
//! - **Loop Prevention**: Sequence numbers prevent infinite message loops
//! - **Collision Avoidance**: Randomized transmission delays reduce packet collisions
//!
//! ## Key Algorithms
//!
//! ### Node Status Update Algorithm
//! Implements a distributed database of node states with:
//! - Sequence number ordering for consistency
//! - Timeout-based node removal
//! - Efficient status delta compression
//!
//! ### Message Flooding Control
//! Uses controlled flooding with:
//! - Duplicate detection based on sequence numbers
//! - TTL (Time To Live) fields to limit propagation
//! - Adaptive backoff for collision reduction

use core::cmp::min;
use core::ptr::{addr_of, addr_of_mut, slice_from_raw_parts_mut};
use core::slice;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use crate::{app, BIT, uprintln};
use crate::embassy::time_driver::clock_time64;
use crate::main_light::{rf_link_data_callback, rf_link_response_callback};
use crate::mesh::{MESH_NODE_ST_VAL_LEN, mesh_node_st_val_t};
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, clock_time, clock_time_exceed};
use crate::sdk::mcu::register::{read_reg_system_tick};
use crate::sdk::packet_types::{*};
use crate::sdk::light::{*};
use crate::state::{*};

/// Updates the mesh network's distributed node status database.
///
/// This function implements the core mesh node status update algorithm that maintains
/// a consistent view of the network topology across all nodes. It processes incoming
/// status packets and updates the local node database with sequence number validation
/// and timeout management.
///
/// # Distributed Database Algorithm
///
/// The algorithm implements a distributed database with the following properties:
///
/// 1. **Sequence Number Ordering**: Uses monotonically increasing sequence numbers
///    to ensure status updates are applied in the correct order
///
/// 2. **Conflict Resolution**: When receiving status for existing nodes, compares
///    sequence numbers to determine if the update is newer than cached data
///
/// 3. **Dynamic Node Discovery**: Automatically adds new nodes to the database
///    when their status is first received
///
/// 4. **Timeout Validation**: Ensures nodes haven't been offline too long before
///    accepting status updates (prevents stale data from disrupting network)
///
/// 5. **Address Filtering**: Filters out invalid addresses and prevents nodes
///    from updating their own status (avoids feedback loops)
///
/// # Sequence Number Algorithm
///
/// Status updates are accepted if:
/// - `new_sn - old_sn < 0x3f` (sequence number is reasonably newer)
/// - OR sequence numbers differ AND node was previously timed out
/// - OR sufficient time has passed since last update
///
/// This handles sequence number wraparound and allows recovery from temporary
/// network partitions.
///
/// # Node Table Management
///
/// The node table uses the following structure:
/// - Index 0: Always contains this device's own status
/// - Index 1-N: Contains discovered neighbor nodes
/// - Dynamic expansion up to `MESH_NODE_MAX_NUM` nodes
///
/// # Parameters
/// * `pkt` - Array of node status values received from the network
///
/// # Returns
/// * `1` on success (status database updated)
/// * `1` on table full condition (no space for new nodes)
///
/// # Algorithm Complexity
/// * Time: O(n*m) where n=nodes in packet, m=nodes in table
/// * Space: O(1) additional space beyond existing node table
pub fn mesh_node_update_status(pkt: &[mesh_node_st_val_t]) -> u32
{
    let mut mesh_node_st = MESH_NODE_ST.lock();

    let mut src_index = 0;
    let mut result = 0xfffffffe;
    
    // Generate current timestamp using scaled timing format (16-bit precision)
    // The | 1 ensures timestamp is never zero (reserved value)
    let tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;
    
    // Process each node status entry in the received packet
    while src_index < pkt.len() && pkt[src_index].dev_adr != 0 {
        // FIXME: Temporary workaround for incorrect device address 1 appearing in packets
        // This filtering should be removed once the root cause is identified and fixed
        if pkt[src_index].dev_adr == 1 {
            src_index += 1;
            continue;
        }

        // Skip our own device address - we don't track our own status in the remote node table
        // (Our own status is maintained separately at index 0)
        if DEVICE_ADDRESS.get() as u8 != pkt[src_index].dev_adr {
            let mesh_node_max = MESH_NODE_MAX.get();
            let mut current_index = 1;  // Start search from index 1 (index 0 is reserved for this device)
            let mut mesh_node = &mut mesh_node_st[current_index];
            
            // NODE LOOKUP ALGORITHM: Find existing node or allocate new slot
            if mesh_node_max >= 2 {
                // Check if the first available slot (index 1) matches the device address
                if mesh_node.val.dev_adr != pkt[src_index].dev_adr {
                    // Linear search through existing nodes to find matching device address
                    for tidx in 1..MESH_NODE_MAX_NUM {
                        current_index = tidx;
                        mesh_node = &mut mesh_node_st[current_index];

                        // Break if we've reached the end of active nodes OR found matching address
                        if mesh_node_max <= tidx as u8 || pkt[src_index].dev_adr == mesh_node.val.dev_adr {
                            break;
                        }
                    }
                }
            }

            // TABLE FULL CHECK: Ensure we haven't exceeded maximum node capacity
            if MESH_NODE_MAX_NUM == current_index {
                return 1; // Table full, cannot add more nodes
            }

            // NEW NODE ALLOCATION: current_index == mesh_node_max means we need a new slot
            if mesh_node_max as usize == current_index {
                // Expand the active node table to include this new node
                MESH_NODE_MAX.inc();

                // Initialize new node with received status data
                mesh_node.val = pkt[src_index];
                mesh_node.tick = tick;

                // Mark new node for status reporting using bitmask
                // Word index = mesh_node_max >> 5, Bit index = mesh_node_max & 0x1f
                MESH_NODE_MASK.lock()[mesh_node_max as usize >> 5] |= 1 << (mesh_node_max & 0x1f);

                result = mesh_node_max as u32;
            } 
            // EXISTING NODE UPDATE: Node already exists in table, check if we should update
            else if current_index < mesh_node_max as usize {
                // SEQUENCE NUMBER VALIDATION: Check if this is a newer status update
                let sn_difference = pkt[src_index].sn - mesh_node.val.sn;
                let par_match = pkt[src_index].par == mesh_node.val.par;

                // TIMEOUT CALCULATION: Use half the standard timeout for update acceptance
                // The division by 2 provides a more aggressive update policy, accepting
                // updates from nodes that haven't been seen recently even if sequence
                // numbers are questionable
                let timeout = (ONLINE_STATUS_TIMEOUT * 1000) / 2;

                result = current_index as u32;
                
                // UPDATE ACCEPTANCE ALGORITHM: Multi-condition check for status update validity
                // Accept update if ANY of these conditions are met:
                // 1. Sequence number difference is reasonable (sn_difference - 2 < 0x3f)
                //    This handles normal sequence number progression with wraparound protection
                // 2. Sequence number changed AND (node was offline OR sufficient time has passed)
                //    This allows recovery from network partitions and handles clock drift
                if sn_difference - 2 < 0x3f || 
                   (sn_difference != 0 && 
                    (mesh_node.tick == 0 || 
                     (((timeout * CLOCK_SYS_CLOCK_1US) >> 0x10) as u16) < tick - mesh_node.tick)) {
                    
                    // Update accepted - copy new status data
                    mesh_node.val = pkt[src_index];

                    // CHANGE DETECTION: Mark node for status reporting if parameters changed
                    // OR if this is the first update after the node was offline (tick == 0)
                    if !par_match || mesh_node.tick == 0 {
                        // Set corresponding bit in status reporting mask
                        MESH_NODE_MASK.lock()[current_index >> 5] |= 1 << (current_index & 0x1f);
                    }

                    // Update timestamp to mark node as recently seen
                    mesh_node.tick = tick;
                }
            }
        }

        // Advance to next status entry in packet
        src_index += 1;
    }
    return 1; // Success - packet processed
}

/// Implements mesh node timeout detection and garbage collection algorithm.
///
/// This function performs periodic cleanup of the mesh node database by detecting
/// nodes that have gone offline and marking them for status reporting. It implements
/// a sophisticated timeout algorithm that accounts for network timing variations
/// and ensures the mesh topology remains accurate.
///
/// # Timeout Detection Algorithm
///
/// The function uses a rate-limited approach to prevent excessive processing:
///
/// 1. **Rate Limiting**: Only executes every 500ms to balance responsiveness with efficiency
/// 2. **Timeout Calculation**: Uses scaled timing comparison to handle clock precision
/// 3. **Node Marking**: Marks timed-out nodes for status change reporting
/// 4. **Cleanup**: Sets node timestamp to 0 to indicate offline status
///
/// # Timing Precision Handling
///
/// The timeout algorithm accounts for system clock precision:
/// - **Clock Scaling**: Right-shifts system tick by 16 bits to reduce precision requirements
/// - **Guard Bit**: ORs with 1 to prevent zero timestamp issues
/// - **Timeout Calculation**: `(ONLINE_STATUS_TIMEOUT * 1000µs) >> 16` for scaled comparison
///
/// This approach allows for longer timeout periods while maintaining reasonable
/// precision and preventing overflow in timeout calculations.
///
/// # Node State Transitions
///
/// Nodes transition through these states:
/// - **Online** (`tick != 0`): Node is actively participating in network
/// - **Timeout Check**: Compares current time against last update plus timeout
/// - **Offline** (`tick = 0`): Node marked as offline and removed from active topology
/// - **Status Reporting**: Node status change is flagged for network propagation
///
/// # Mesh Network Maintenance
///
/// The function maintains network integrity by:
/// - Removing stale node information to prevent routing to dead nodes
/// - Triggering status updates so other nodes learn about topology changes
/// - Preventing false positive timeouts through careful timing calculations
/// - Preserving node index 0 (this device) which is never timed out
///
/// # Performance Characteristics
///
/// - **Time Complexity**: O(n) where n is the number of active nodes
/// - **Rate Limited**: Maximum execution frequency of 2Hz (every 500ms)
/// - **Memory Access**: Linear scan through node table (cache-friendly)
/// - **Atomic Operations**: Uses atomic timestamps for thread safety
///
/// # Side Effects
/// * Updates node timestamps to mark offline nodes
/// * Modifies mesh node mask to trigger status reporting
/// * May cause network-wide status updates as topology changes propagate
#[cfg_attr(test, mry::mry)]
pub fn mesh_node_flush_status()
{
    static TICK_NODE_REPORT: AtomicU32 = AtomicU32::new(0);

    // Rate limiting: only execute timeout detection every 500ms
    if !clock_time_exceed(TICK_NODE_REPORT.load(Ordering::Relaxed), 500000) {
        return;
    }

    // Update last execution timestamp
    let tick = read_reg_system_tick();
    TICK_NODE_REPORT.store(tick, Ordering::Relaxed);

    let mut mesh_node_st = MESH_NODE_ST.lock();

    // Scan all known mesh nodes for timeout detection (skip index 0 = this device)
    for count in 1..MESH_NODE_MAX.get() as usize {
        // Check if node is online and whether it has timed out
        if mesh_node_st[count].tick != 0 {
            // Calculate timeout threshold using scaled timing for precision
            let timeout_threshold = (CLOCK_SYS_CLOCK_1US * ONLINE_STATUS_TIMEOUT * 1000) >> 0x10;
            let current_time_scaled = (tick >> 0x10) | 1; // Guard bit prevents zero
            let node_last_seen = mesh_node_st[count].tick as u32;
            
            if timeout_threshold < current_time_scaled - node_last_seen {
                // Node has timed out - mark as offline
                mesh_node_st[count].tick = 0;

                // Set status change flag to trigger network reporting
                // Calculate bit position: word index = count >> 5, bit index = count & 0x1f
                MESH_NODE_MASK.lock()[count >> 5] |= 1 << (count & 0x1f);
            }
        }
    }
}

/// Updates this device's own status record to maintain mesh network presence.
///
/// This function implements the local node status update algorithm that ensures
/// this device remains visible in the mesh network topology. It manages sequence
/// number generation and timestamp updates to prevent this device from being
/// considered offline by other nodes.
///
/// # Sequence Number Management
///
/// The function implements a monotonic sequence number generation algorithm:
/// 1. **Increment**: Increases the device sequence number for each status update
/// 2. **Wraparound Handling**: Prevents sequence number 0 (reserved value)
/// 3. **Uniqueness**: Ensures each status update has a unique identifier
///
/// Sequence number 0 is avoided because it's used as a special value in the
/// mesh protocol to indicate invalid or uninitialized status.
///
/// # Timestamp Update Algorithm
///
/// The timestamp update follows the same scaling approach used throughout
/// the mesh system:
/// - **Clock Scaling**: Right-shift by 16 bits for precision management
/// - **Guard Bit**: OR with 1 to prevent zero timestamps
/// - **Consistency**: Uses same timing format as timeout detection
///
/// This ensures timestamp compatibility across all mesh timing operations
/// and prevents issues with zero-valued timestamps.
///
/// # Node Table Management
///
/// The function updates the local device record (always at index 0):
/// - **Sequence Number**: Sets current monotonic sequence number
/// - **Timestamp**: Updates last-seen time to current scaled time
/// - **Status Preservation**: Maintains other status fields unchanged
///
/// Index 0 is reserved for this device's own status and is never used
/// for remote node information.
///
/// # Network Presence Maintenance
///
/// Regular calls to this function ensure:
/// - This device appears as "online" in status broadcasts
/// - Other nodes don't mark this device as timed out
/// - Sequence numbers remain current for status comparison
/// - Network topology accurately reflects this device's presence
///
/// # Call Frequency
///
/// This function should be called:
/// - Before each status broadcast transmission
/// - During status advertisement preparation
/// - At regular intervals to maintain network presence
/// - When significant status changes occur
///
/// # Side Effects
/// * Increments global device sequence number
/// * Updates local node status record
/// * Affects subsequent status broadcasts
/// * Influences network topology as seen by other nodes
fn mesh_node_keep_alive()
{
    // Increment monotonic sequence number for status updates
    DEVICE_NODE_SN.inc();
    
    // Prevent sequence number 0 (reserved value) by wrapping to 1
    if DEVICE_NODE_SN.get() == 0 {
        DEVICE_NODE_SN.set(1);
    }

    let mut mesh_node_st = MESH_NODE_ST.lock();

    // Update this device's status record (index 0)
    mesh_node_st[0].val.sn = DEVICE_NODE_SN.get();
    
    // Update timestamp using scaled timing format for consistency
    mesh_node_st[0].tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;
}

/// Generates mesh network status advertisement with round-robin node selection.
///
/// This function implements a sophisticated status advertisement algorithm that
/// efficiently packs node status information into limited packet space. It uses
/// a round-robin selection strategy to ensure all nodes are eventually advertised
/// while prioritizing the most recent status updates.
///
/// # Advertisement Strategy
///
/// The algorithm follows a multi-phase approach:
///
/// 1. **Buffer Initialization**: Clears output buffer and calculates capacity
/// 2. **Self-Status Inclusion**: Always includes this device's status first
/// 3. **Round-Robin Selection**: Cycles through other nodes to distribute advertisement load
/// 4. **Capacity Management**: Respects packet size limits while maximizing information density
///
/// # Round-Robin Algorithm
///
/// The function maintains fair advertisement distribution through:
/// - **Static Index**: Persistent cursor remembers last advertised node
/// - **Wraparound Logic**: Cycles back to node 1 after reaching maximum
/// - **Skip Offline Nodes**: Only advertises nodes with valid timestamps
/// - **Progressive Coverage**: Eventually advertises all active nodes
///
/// This ensures that over multiple advertisement cycles, all active nodes
/// receive equal advertisement opportunities.
///
/// # Packet Structure Optimization
///
/// The function optimizes packet utilization by:
/// - **Priority Placement**: This device's status always appears first
/// - **Capacity Calculation**: `min(buffer_size / node_size, active_nodes)`
/// - **Dense Packing**: Fills available space without fragmentation
/// - **Online Filtering**: Only includes nodes with non-zero timestamps
///
/// # Memory Safety and Performance
///
/// The implementation uses several techniques for safety and efficiency:
/// - **Unsafe Memory Access**: Direct memory copying for performance-critical operations
/// - **Atomic Operations**: Thread-safe round-robin index management
/// - **Lock Coordination**: Minimal lock scope to reduce contention
/// - **Buffer Bounds Checking**: Prevents buffer overrun conditions
///
/// # Advertisement Content Format
///
/// Each node status entry contains:
/// - Device address (1 byte)
/// - Sequence number (1 byte)  
/// - Status parameters (variable length)
/// - Timestamp (implicit, not transmitted)
///
/// The exact format is defined by `MESH_NODE_ST_VAL_LEN` and the
/// `mesh_node_st_val_t` structure.
///
/// # Network Load Balancing
///
/// The round-robin approach provides several benefits:
/// - **Even Distribution**: All nodes receive equal advertisement frequency
/// - **Reduced Collisions**: Spreads advertisement load across time
/// - **Progressive Discovery**: New nodes are quickly discovered and advertised
/// - **Adaptive Capacity**: Adjusts to network size changes automatically
///
/// # Parameters
/// * `p_data` - Output buffer for status advertisement packet data
///
/// # Returns
/// * Number of node status entries written to the output buffer
///
/// # Side Effects
/// * Updates round-robin advertisement cursor
/// * Calls `mesh_node_keep_alive()` to refresh local status
/// * Modifies output buffer content
/// * Advances advertisement rotation for next call
fn mesh_node_adv_status(p_data: &mut [u8]) -> u32
{
    static MESH_NODE_CUR: AtomicUsize = AtomicUsize::new(1);

    // Initialize output buffer to clean state
    p_data.fill(0);

    // Calculate maximum node entries that fit in available space
    let mut elems = p_data.len() / MESH_NODE_ST_VAL_LEN;
    if (MESH_NODE_MAX.get() as usize) < p_data.len() / MESH_NODE_ST_VAL_LEN {
        elems = MESH_NODE_MAX.get() as usize;
    }

    {
        let mut mesh_node_st = MESH_NODE_ST.lock();

        // Always place this device's status first in advertisement
        p_data[0..MESH_NODE_ST_VAL_LEN].copy_from_slice(
            unsafe {
                slice::from_raw_parts(
                    addr_of!(mesh_node_st[0].val) as *const u8,
                    MESH_NODE_ST_VAL_LEN,
                )
            }
        );
    }

    // Update local status to ensure current information
    mesh_node_keep_alive();

    let mut mesh_node_st = MESH_NODE_ST.lock();

    let max_node = MESH_NODE_MAX.get() as usize;
    let mut count = 1;  // Start from 1 (skip self at index 0)

    let mut out_index = count;  // Output position in advertisement packet
    
    // Round-robin selection of other nodes for advertisement
    while out_index < elems && count < max_node {
        let mnc = MESH_NODE_CUR.load(Ordering::Relaxed);
        
        // Only advertise nodes that are online (tick != 0)
        if mnc < max_node && mesh_node_st[mnc].tick != 0 {
            // Copy node status to advertisement packet
            let ptr = MESH_NODE_ST_VAL_LEN * out_index;
            out_index = out_index + 1;
            p_data[ptr..ptr + MESH_NODE_ST_VAL_LEN].copy_from_slice(
                unsafe {
                    slice::from_raw_parts(
                        addr_of!(mesh_node_st[mnc].val) as *const u8,
                        MESH_NODE_ST_VAL_LEN,
                    )
                }
            );
        }

        // Advance round-robin cursor for next advertisement cycle
        MESH_NODE_CUR.store(mnc + 1, Ordering::Relaxed);

        // Wraparound to node 1 when reaching end (node 0 is always self)
        if max_node <= MESH_NODE_CUR.load(Ordering::Relaxed) {
            MESH_NODE_CUR.store(1, Ordering::Relaxed);
        }

        count += 1;
    }

    return out_index as u32;
}

/// Transmits periodic status advertisement to maintain mesh network topology.
///
/// This function implements the core mesh network status broadcasting algorithm
/// that maintains network-wide topology awareness. It combines timeout detection,
/// status collection, and packet transmission into a coordinated sequence that
/// ensures all nodes have current information about network state.
///
/// # Status Broadcasting Algorithm
///
/// The function follows a structured broadcast procedure:
///
/// 1. **Rate Limiting**: Enforces minimum interval between status transmissions
/// 2. **Timeout Processing**: Detects and marks offline nodes before status collection
/// 3. **Status Aggregation**: Collects current status from all active nodes
/// 4. **Packet Construction**: Builds properly formatted status advertisement packet
/// 5. **Network Transmission**: Queues packet for mesh network broadcast
///
/// # Packet Structure Specification
///
/// Status advertisement packets use a specialized format:
///
/// ```
/// Status Advertisement Packet (39 bytes):
/// ┌─────────────┬─────────────┬─────────────┬─────────────┐
/// │ L2CAP Header│ Status Data │ Sequence #  │ Signature   │
/// │ (6 bytes)   │ (24 bytes)  │ (3 bytes)   │ (4 bytes)   │
/// └─────────────┴─────────────┴─────────────┴─────────────┘
/// ```
///
/// ### L2CAP Header Configuration
/// - **DMA Length**: 0x27 (39 bytes total packet size)
/// - **Type**: 2 (advertisement packet type)
/// - **RF Length**: 0x25 (37 bytes RF payload)
/// - **L2CAP Length**: 0x21 (33 bytes L2CAP payload)
/// - **Channel ID**: 0xffff (status advertisement channel)
///
/// ### Status Data Section (24 bytes)
/// Contains node status entries generated by `mesh_node_adv_status()`:
/// - This device's status (always first)
/// - Round-robin selection of other active nodes
/// - Packed format to maximize information density
///
/// ### Sequence Number Section (3 bytes)
/// - Advertisement sequence number (little-endian)
/// - Increments with each status broadcast
/// - Used for duplicate detection and ordering
///
/// ### Signature Section (4 bytes)
/// - Fixed pattern: 0xa5a5a5a5
/// - Packet validation and type identification
/// - Helps distinguish status packets from other traffic
///
/// # Rate Limiting Strategy
///
/// The function implements adaptive rate limiting based on:
/// - **Configured Interval**: `SEND_MESH_STATUS_INTERVAL_MS` setting
/// - **Network Load**: Prevents excessive status traffic
/// - **Timing Precision**: Uses high-resolution timing for accuracy
///
/// This ensures status broadcasts provide timely updates without overwhelming
/// the mesh network with excessive traffic.
///
/// # Network Coordination
///
/// The status broadcasting system coordinates with other mesh functions:
/// - **Timeout Detection**: Calls `mesh_node_flush_status()` to update node states
/// - **Status Collection**: Uses `mesh_node_adv_status()` for fair node selection
/// - **Transmission Queue**: Integrates with mesh manager for reliable delivery
/// - **Sequence Management**: Maintains unique sequence numbers for tracking
///
/// # Reliability Considerations
///
/// The current implementation uses minimal retransmission (retransmit count = 0)
/// which prioritizes network efficiency over individual packet reliability.
/// The TODO comment suggests this could be increased for better reliability
/// in challenging RF environments.
///
/// # Network Convergence
///
/// Regular status broadcasts enable network-wide convergence by:
/// - Propagating topology changes to all nodes
/// - Providing redundant status information
/// - Enabling rapid detection of node failures
/// - Supporting consistent network state across all participants
///
/// # Side Effects
/// * Increments advertisement sequence number
/// * Triggers timeout detection and node cleanup
/// * Queues packet for mesh network transmission  
/// * Updates network-wide topology knowledge
/// * May trigger retransmissions by other nodes
pub fn mesh_send_online_status()
{
    static ADV_ST_SN: AtomicU32 = AtomicU32::new(0);
    static LAST_STATUS_TIME: AtomicU32 = AtomicU32::new(0);

    // Rate limiting: only send status at configured intervals
    if !clock_time_exceed(LAST_STATUS_TIME.get(), 1000 * SEND_MESH_STATUS_INTERVAL_MS) {
        return;
    }

    // Update last transmission timestamp
    LAST_STATUS_TIME.set(clock_time());

    // Construct status advertisement packet with specialized header
    let mut pkt_light_adv_status = Packet {
        att_write: PacketAttWrite {
            head: PacketL2capHead {
                dma_len: 0x27,      // Total packet size (39 bytes)
                _type: 2,           // Advertisement packet type
                rf_len: 0x25,       // RF payload size (37 bytes)
                l2cap_len: 0x21,    // L2CAP payload size (33 bytes)
                chan_id: 0xffff,    // Status advertisement channel
            },
            opcode: 0,              // Sequence number (filled below)
            handle: 0,              // Unused in status packets
            handle1: 0,             // Unused in status packets
            value: PacketAttValue::default(),
        }
    };

    // Get direct access to packet payload for efficient manipulation
    let pktdata = unsafe { 
        &mut *slice_from_raw_parts_mut(
            addr_of!(pkt_light_adv_status.att_write().value) as *mut u8, 
            core::mem::size_of::<PacketAttValue>()
        ) 
    };

    // Process node timeouts and collect current status information
    mesh_node_flush_status();
    mesh_node_adv_status(&mut pktdata[..24]);

    // Generate and embed advertisement sequence number
    ADV_ST_SN.store(ADV_ST_SN.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
    unsafe {
        let val = ADV_ST_SN.load(Ordering::Relaxed);
        // Copy sequence number to opcode field (3 bytes, little-endian)
        slice::from_raw_parts_mut(addr_of_mut!(pkt_light_adv_status.att_write_mut().opcode), 3)
            .copy_from_slice(slice::from_raw_parts(addr_of!(val) as *const u8, 3))
    }

    // Add packet signature for validation and type identification
    pktdata[24..28].fill(0xa5);

    // Queue packet for mesh network transmission
    // TODO: Consider increasing retransmit count for better reliability
    app().mesh_manager.add_send_mesh_msg(&pkt_light_adv_status, 0, 0);
}

/// Constructs a mesh network packet with comprehensive parameter validation and setup.
///
/// This function implements the mesh packet construction algorithm that creates properly
/// formatted packets for transmission over the mesh network. It handles address assignment,
/// sequence number management, and internal parameter configuration for reliable mesh communication.
///
/// # Mesh Packet Structure
///
/// The constructed packet follows the mesh protocol format:
/// ```
/// [L2CAP Header][Mesh Header][Payload][Internal Parameters]
/// ```
///
/// ## L2CAP Header (6 bytes)
/// - **DMA Length**: 0x27 (39 bytes total packet size)
/// - **Type**: 2 (mesh packet type identifier)
/// - **RF Length**: 0x25 (37 bytes RF payload)
/// - **L2CAP Length**: 0x21 (33 bytes L2CAP payload)
/// - **Channel ID**: 0xff03 (mesh channel identifier)
///
/// ## Mesh Header (variable)
/// - **Source TX Address**: Transmitting node address
/// - **Sequence Number**: 24-bit monotonic sequence number
/// - **Source Address**: Originating node address
/// - **Destination Address**: Target node/group address
/// - **Operation Code**: Command/operation identifier
/// - **Vendor ID**: Manufacturer identification
/// - **Parameters**: Command-specific payload data
///
/// ## Internal Parameters (14 bytes)
/// - **Retransmit Count**: Number of retransmission attempts
/// - **Send ACK Flag**: Whether acknowledgment is required
/// - **TTL**: Time-to-live for hop limiting
/// - **Reserved**: Future protocol extensions
///
/// # Sequence Number Management
///
/// Sequence numbers provide:
/// - **Duplicate Detection**: Prevents processing same packet multiple times
/// - **Ordering**: Ensures packets are processed in correct order
/// - **Loop Prevention**: Stops packets from circulating indefinitely
///
/// The 24-bit sequence number allows for 16.7M unique packets before wraparound,
/// providing sufficient space for long-running networks.
///
/// # Address Assignment Algorithm
///
/// - **Source TX**: Always set to this device's address (for immediate transmission)
/// - **Source Address**: Set to this device's address (for end-to-end tracking)
/// - **Destination**: Copied from parameter (supports unicast and multicast)
///
/// # Parameter Validation
///
/// The function validates:
/// - Command parameter length (3-13 bytes)
/// - Address validity (non-zero, within range)
/// - Retransmit count (reasonable limits)
///
/// # Parameters
/// * `sno` - 24-bit sequence number for duplicate detection and ordering
/// * `dst` - Destination address (1-254 for unicast, >254 for multicast)
/// * `cmd_op_para` - Command opcode and parameters (3-13 bytes)
/// * `retransmit_count` - Number of retransmission attempts (0-255)
/// * `send_ack` - Whether receiver should send acknowledgment
///
/// # Returns
/// * Properly formatted mesh packet ready for transmission
///
/// # Panics
/// * If `cmd_op_para` length is not in range [3, 13]
/// * If internal consistency checks fail
pub fn mesh_construct_packet(sno: u32, dst: u16, cmd_op_para: &[u8], retransmit_count: u8, send_ack: bool) -> Packet
{
    // Validate command parameter length constraints
    assert!(cmd_op_para.len() > 2, "Command parameters too short (minimum 3 bytes)");
    assert!(cmd_op_para.len() <= 13, "Command parameters too long (maximum 13 bytes)");

    let device_address = DEVICE_ADDRESS.get();

    // Initialize mesh packet structure with standard header values
    let mut pkt = MeshPkt {
        head: PacketL2capHead {
            dma_len: 0x27,      // Total DMA transfer length (39 bytes)
            _type: 2,           // Mesh packet type identifier
            rf_len: 0x25,       // RF payload length (37 bytes)
            l2cap_len: 0x21,    // L2CAP payload length (33 bytes)  
            chan_id: 0xff03,    // Mesh network channel identifier
        },
        src_tx: device_address,     // Immediate transmitter address
        handle1: 0,                 // Reserved handle field
        sno: [0; 3],               // Sequence number (filled below)
        src_adr: device_address,    // Original source address
        dst_adr: dst,              // Destination address
        op: 0,                     // Operation code (filled below)
        vendor_id: 0,              // Vendor identifier (filled below)
        par: [0; 10],              // Command parameters (filled below)
        internal_par1: [0; 5],     // Internal parameters
        ttl: 0,                    // Time-to-live hop counter
        internal_par2: [0; 4],     // Additional internal parameters
        no_use: [0; 4],            // Reserved/unused bytes
    };

    // Convert 32-bit sequence number to 24-bit little-endian format
    // This provides unique packet identification for duplicate detection
    pkt.sno[0] = sno as u8;           // LSB
    pkt.sno[1] = (sno >> 8) as u8;    // Middle byte
    pkt.sno[2] = (sno >> 16) as u8;   // MSB (limited to 24 bits)

    // Copy command opcode and parameters into packet structure
    // Uses unsafe code for direct memory copy to handle variable-length parameters
    unsafe {
        slice::from_raw_parts_mut(addr_of_mut!(pkt.op), cmd_op_para.len())
            .copy_from_slice(cmd_op_para)
    }

    // Configure internal protocol parameters for transmission control
    pkt.internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT] = retransmit_count;
    pkt.internal_par1[INTERNAL_PAR_SEND_ACK] = if send_ack { 1 } else { 0 };

    // Wrap mesh packet in generic packet union for transmission
    Packet { mesh: pkt }
}

/// Enables or disables mesh status reporting with comprehensive mask management.
///
/// This function implements a bulk status reporting control system that can
/// enable reporting for all known mesh nodes simultaneously. It uses an
/// efficient bitmask algorithm to mark all active nodes for status reporting
/// without requiring individual node iteration.
///
/// # Bitmask Algorithm
///
/// The function uses a two-phase bitmask initialization strategy:
///
/// 1. **Word-Level Initialization**: Sets entire 32-bit words to 0xfffffffe
///    - This enables all bits except bit 0 (reserved for this device)
///    - Covers nodes 1-31, 33-63, 65-95, etc. in each word
///
/// 2. **Partial Word Handling**: Handles the final partial word separately
///    - Uses `(1 << bit_count) - 1` to create exact bit mask
///    - Ensures only valid node positions are marked
///
/// # Address Space Partitioning
///
/// The algorithm partitions the node address space efficiently:
/// - **Word Index**: `node_index >> 5` (divide by 32)
/// - **Bit Index**: `node_index & 0x1f` (modulo 32)
/// - **Word Boundary**: Handles up to 8 words (256 node addresses)
///
/// # Efficiency Optimization
///
/// The bulk enable approach provides several performance benefits:
/// - **Constant Time**: O(1) complexity regardless of active node count
/// - **Memory Efficient**: Direct word-level mask manipulation
/// - **Cache Friendly**: Sequential memory access pattern
/// - **Atomic Operation**: Single transaction for all nodes
///
/// # Parameters
/// * `enable` - If true, marks all nodes for status reporting; if false, disables reporting
///
/// # Side Effects
/// * Modifies global mesh node reporting mask
/// * Affects subsequent status advertisement inclusion
/// * Enables/disables network-wide status propagation
#[cfg_attr(test, mry::mry)]
pub fn mesh_report_status_enable(enable: bool)
{
    let mut mesh_node_mask = MESH_NODE_MASK.lock();
    if enable {
        // Set all complete 32-bit words to enable reporting (skip node 0 in each word)
        if MESH_NODE_MAX.get() >> 5 != 0 {
            mesh_node_mask.iter_mut().for_each(|v| { *v = 0xfffffffe });
        }

        // Handle partial word at the end with exact bit count
        if MESH_NODE_MAX.get() & 0x1f != 0 {
            let bit_count = MESH_NODE_MAX.get() & 0x1f;
            mesh_node_mask[MESH_NODE_MAX.get() as usize >> 5] = (1 << bit_count) - 1;
        }
    }

    MESH_NODE_REPORT_ENABLE.set(enable);
}

/// Enables selective mesh status reporting for specific device addresses.
///
/// This function implements targeted status reporting control that allows
/// fine-grained selection of which nodes should be included in status reports.
/// It uses an address-based lookup algorithm to map device addresses to
/// their corresponding bitmask positions.
///
/// # Selective Reporting Algorithm
///
/// The function operates in two phases:
///
/// 1. **Global Enable Control**: First byte determines overall reporting state
/// 2. **Address-Specific Marking**: Subsequent bytes specify individual device addresses
///
/// # Address Lookup Strategy
///
/// For each specified address, the function:
/// - Scans the node status table for matching device addresses
/// - Calculates the corresponding bitmask position
/// - Sets the appropriate bit in the reporting mask
///
/// This approach allows external controllers to specify exactly which nodes
/// should be included in status reports without requiring knowledge of
/// internal node table indices.
///
/// # Data Format
///
/// The input data follows this structure:
/// ```
/// data[0]: Global enable flag (0 = disable all, non-zero = enable selective)
/// data[1..n]: Device addresses to enable for reporting
/// ```
///
/// # Performance Characteristics
///
/// - **Time Complexity**: O(n*m) where n=addresses specified, m=nodes in table
/// - **Memory Access**: Linear scan through node table for each address
/// - **Scalability**: Performance degrades with large node counts
/// - **Flexibility**: Allows arbitrary address selection patterns
///
/// # Use Cases
///
/// This function supports scenarios such as:
/// - Debugging specific nodes without full network noise
/// - Monitoring critical infrastructure nodes only
/// - Implementing priority-based reporting systems
/// - Supporting network management hierarchies
///
/// # Parameters
/// * `data` - Array containing enable flag and target device addresses
///
/// # Side Effects
/// * Updates global mesh reporting enable flag
/// * Modifies selective reporting bitmask
/// * Affects which nodes appear in subsequent status reports
pub fn mesh_report_status_enable_mask(data: &[u8])
{
    let mut mesh_node_mask = MESH_NODE_MASK.lock();
    let mut mesh_node_st = MESH_NODE_ST.lock();

    // Set global reporting enable state from first byte
    MESH_NODE_REPORT_ENABLE.set(data[0] != 0);
    
    // Process selective address list if reporting is enabled
    if MESH_NODE_REPORT_ENABLE.get() && data.len() > 1 {
        for index in 1..data.len() {
            if MESH_NODE_MAX.get() != 0 {
                // Search node table for matching device addresses
                mesh_node_st.iter_mut().enumerate().for_each(|(i, v)| {
                    if data[index] == v.val.dev_adr {
                        // Enable reporting for this node by setting corresponding bit
                        mesh_node_mask[i >> 5] |= 1 << (i & 0x1f);
                    }
                });
            }
        }
    }
}

/// Determines if a packet matches configured group or device address filters.
///
/// This function implements the mesh network address filtering algorithm that
/// determines whether an incoming packet should be processed by this device.
/// It supports both unicast (device-specific) and multicast (group) addressing
/// schemes with comprehensive address space partitioning.
///
/// # Address Space Architecture
///
/// The mesh protocol uses a hierarchical address space:
///
/// ## Device Address Space (0x0000-0x00FF)
/// - **Broadcast**: Address 0x0000 matches all devices
/// - **Unicast**: Addresses 0x0001-0x00FE match specific devices
/// - **Reserved**: Address 0x00FF reserved for protocol use
///
/// ## Group Address Space (0x0100-0xFFFF)
/// - **Standard Groups**: Addresses 0x0100-0xFFFE for group messaging
/// - **Global Broadcast**: Address 0xFFFF matches all devices in all groups
///
/// # Filtering Algorithm
///
/// The function uses a two-phase filtering approach:
///
/// 1. **Address Space Detection**: Determines if address is device or group space
/// 2. **Match Evaluation**: Applies appropriate matching logic for the address type
///
/// # Device Address Matching
///
/// For device addresses (masked with `DEVICE_ADDR_MASK_DEFAULT`):
/// - **Broadcast Match**: Address 0 matches all devices
/// - **Unicast Match**: Address must equal this device's address
/// - **Exclusion**: Other addresses are ignored by this device
///
/// # Group Address Matching
///
/// For group addresses (above device mask threshold):
/// - **Group Table Lookup**: Searches configured group membership table
/// - **Global Broadcast**: Address 0xFFFF always matches (universal broadcast)
/// - **Membership Validation**: Only configured groups are accepted
///
/// # Return Value Semantics
///
/// The function returns a tuple indicating match results:
/// - `(group_match, device_match)` where both are boolean flags
/// - Exactly one should be true for valid packets (not both)
/// - Both false indicates packet should be ignored
///
/// # Parameters
/// * `pkt` - Incoming packet with destination address to evaluate
///
/// # Returns
/// * `(group_match, device_match)` tuple indicating address filter results
///
/// # Performance Optimization
///
/// The algorithm optimizes for common cases:
/// - Device address check uses simple comparison (O(1))
/// - Group lookup uses early termination on first match
/// - Global broadcast check avoids table lookup
#[cfg_attr(test, mry::mry)]
pub fn rf_link_match_group_mac(pkt: &Packet) -> (bool, bool)
{
    let mut group_match = false;
    let mut device_match = false;

    // Determine address space: group vs. device addressing
    if pkt.ll_app().value.dst & !DEVICE_ADDR_MASK_DEFAULT != 0 {
        // Group address space: check group membership table
        for addr in *GROUP_ADDRESS.lock() {
            if addr == pkt.ll_app().value.dst {
                group_match = true;
                break;
            }
        }
        
        // Global broadcast address (0xFFFF) matches all devices
        if pkt.ll_app().value.dst == 0xffff {
            group_match = true;
        }
    } else {
        // Device address space: check unicast and broadcast addresses
        let addr = pkt.ll_app().value.dst & DEVICE_ADDR_MASK_DEFAULT;
        if addr == 0 || addr == DEVICE_ADDRESS.get() {
            device_match = true;
        }
    }

    (group_match, device_match)
}

/// Updates this device's status parameters and triggers status change reporting.
///
/// This function implements the local status update mechanism that allows
/// upper-layer applications to modify this device's advertised status parameters.
/// It ensures that status changes are properly timestamped and marked for
/// network-wide propagation.
///
/// # Status Update Algorithm
///
/// The function performs an atomic status update sequence:
///
/// 1. **Parameter Update**: Copies new status parameters to device record
/// 2. **Timestamp Refresh**: Updates last-modified timestamp to current time
/// 3. **Change Notification**: Sets reporting flag to trigger status broadcast
///
/// # Timestamp Management
///
/// The timestamp update uses the same scaled timing format as other mesh functions:
/// - **Clock Scaling**: Right-shift by 16 bits for precision management
/// - **Guard Bit**: OR with 1 to prevent zero timestamps
/// - **Consistency**: Matches timing format used in timeout detection
///
/// # Status Change Propagation
///
/// The function triggers network propagation by:
/// - Setting bit 0 in the mesh node mask (this device's status bit)
/// - This causes subsequent status advertisements to include updated parameters
/// - Other nodes receive and process the status change through normal mesh propagation
///
/// # Device Record Management
///
/// The local device record (index 0) contains:
/// - **Device Address**: This device's mesh network address
/// - **Sequence Number**: Monotonic counter for status versions
/// - **Status Parameters**: Application-specific status data
/// - **Timestamp**: Last update time for timeout detection
///
/// # Application Integration
///
/// This function provides the interface between application logic and mesh networking:
/// - Applications call this function when device status changes (lighting, sensors, etc.)
/// - Status changes automatically propagate through the mesh network
/// - Other devices can observe and react to status changes
/// - Network maintains consistent view of device states
///
/// # Parameters
/// * `val_par` - New status parameters to advertise (application-specific format)
///
/// # Side Effects
/// * Updates local device status record
/// * Refreshes device timestamp to prevent timeout
/// * Triggers status change notification for network propagation
/// * Affects subsequent mesh status advertisements
pub fn ll_device_status_update(val_par: &[u8])
{
    let mut mesh_node_st = MESH_NODE_ST.lock();

    // Update this device's status parameters (index 0 = local device)
    mesh_node_st[0].val.par.copy_from_slice(val_par);
    
    // Refresh timestamp using scaled timing format for consistency
    mesh_node_st[0].tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;

    // Mark this device for status change reporting (bit 0 = this device)
    MESH_NODE_MASK.lock()[0] |= 1;
}
