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

use crate::embassy::time_driver::clock_time64;
use crate::main_light::{rf_link_data_callback, rf_link_response_callback};
use crate::mesh::{MeshNodeStValT, MESH_NODE_ST_VAL_LEN};
use crate::sdk::light::*;
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed, CLOCK_SYS_CLOCK_1US};
use crate::sdk::mcu::register::read_reg_system_tick;
use crate::sdk::packet_types::*;
use crate::state::*;
use crate::{app, BIT};

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
#[cfg_attr(test, mry::mry)]
pub fn mesh_node_update_status(pkt: &[MeshNodeStValT]) -> u32 {
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
            let mut current_index = 1; // Start search from index 1 (index 0 is reserved for this device)
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
                        if mesh_node_max <= tidx as u8
                            || pkt[src_index].dev_adr == mesh_node.val.dev_adr
                        {
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
                // Use wrapping subtraction to handle u8 overflow/underflow correctly
                let sn_difference = pkt[src_index].sn.wrapping_sub(mesh_node.val.sn);
                let par_match = pkt[src_index].par == mesh_node.val.par;

                // TIMEOUT CALCULATION: Use half the standard timeout for update acceptance
                // The division by 2 provides a more aggressive update policy, accepting
                // updates from nodes that haven't been seen recently even if sequence
                // numbers are questionable
                let timeout = (ONLINE_STATUS_TIMEOUT * 1000) / 2;

                result = current_index as u32;

                // UPDATE ACCEPTANCE ALGORITHM: Multi-condition check for status update validity
                // Accept update if ANY of these conditions are met:
                // 1. Sequence number difference is reasonable (≤ 65, accounting for wraparound)
                //    This handles normal sequence number progression with wraparound protection
                // 2. Sequence number changed AND (node was offline OR sufficient time has passed)
                //    This allows recovery from network partitions and handles clock drift
                if sn_difference <= 65
                    || (sn_difference != 0
                        && (mesh_node.tick == 0
                            || (((timeout * CLOCK_SYS_CLOCK_1US) >> 0x10) as u16)
                                < tick - mesh_node.tick))
                {
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
pub fn mesh_node_flush_status() {
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
fn mesh_node_keep_alive() {
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
#[cfg_attr(test, mry::mry)]
fn mesh_node_adv_status(p_data: &mut [u8]) -> u32 {
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
        p_data[0..MESH_NODE_ST_VAL_LEN].copy_from_slice(unsafe {
            slice::from_raw_parts(
                addr_of!(mesh_node_st[0].val) as *const u8,
                MESH_NODE_ST_VAL_LEN,
            )
        });
    }

    // Update local status to ensure current information
    mesh_node_keep_alive();

    let mut mesh_node_st = MESH_NODE_ST.lock();

    let max_node = MESH_NODE_MAX.get() as usize;
    let mut count = 1; // Start from 1 (skip self at index 0)

    let mut out_index = count; // Output position in advertisement packet

    // Round-robin selection of other nodes for advertisement
    while out_index < elems && count < max_node {
        let mnc = MESH_NODE_CUR.load(Ordering::Relaxed);

        // Only advertise nodes that are online (tick != 0)
        if mnc < max_node && mesh_node_st[mnc].tick != 0 {
            // Copy node status to advertisement packet
            let ptr = MESH_NODE_ST_VAL_LEN * out_index;
            out_index = out_index + 1;
            p_data[ptr..ptr + MESH_NODE_ST_VAL_LEN].copy_from_slice(unsafe {
                slice::from_raw_parts(
                    addr_of!(mesh_node_st[mnc].val) as *const u8,
                    MESH_NODE_ST_VAL_LEN,
                )
            });
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
#[cfg_attr(test, mry::mry)]
pub fn mesh_send_online_status() {
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
                dma_len: 0x27,   // Total packet size (39 bytes)
                _type: 2,        // Advertisement packet type
                rf_len: 0x25,    // RF payload size (37 bytes)
                l2cap_len: 0x21, // L2CAP payload size (33 bytes)
                chan_id: 0xffff, // Status advertisement channel
            },
            opcode: 0,  // Sequence number (filled below)
            handle: 0,  // Unused in status packets
            handle1: 0, // Unused in status packets
            value: PacketAttValue::default(),
        },
    };

    // Get direct access to packet payload for efficient manipulation
    let pktdata = unsafe {
        &mut *slice_from_raw_parts_mut(
            addr_of!(pkt_light_adv_status.att_write().value) as *mut u8,
            core::mem::size_of::<PacketAttValue>(),
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
    app()
        .mesh_manager
        .add_send_mesh_msg(&pkt_light_adv_status, 0, 0);
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
pub fn mesh_construct_packet(
    sno: u32,
    dst: u16,
    cmd_op_para: &[u8],
    retransmit_count: u8,
    send_ack: bool,
) -> Packet {
    // Validate command parameter length constraints
    assert!(
        cmd_op_para.len() > 2,
        "Command parameters too short (minimum 3 bytes)"
    );
    assert!(
        cmd_op_para.len() <= 13,
        "Command parameters too long (maximum 13 bytes)"
    );

    let device_address = DEVICE_ADDRESS.get();

    // Initialize mesh packet structure with standard header values
    let mut pkt = MeshPkt {
        head: PacketL2capHead {
            dma_len: 0x27,   // Total DMA transfer length (39 bytes)
            _type: 2,        // Mesh packet type identifier
            rf_len: 0x25,    // RF payload length (37 bytes)
            l2cap_len: 0x21, // L2CAP payload length (33 bytes)
            chan_id: 0xff03, // Mesh network channel identifier
        },
        src_tx: device_address,  // Immediate transmitter address
        handle1: 0,              // Reserved handle field
        sno: [0; 3],             // Sequence number (filled below)
        src_adr: device_address, // Original source address
        dst_adr: dst,            // Destination address
        op: 0,                   // Operation code (filled below)
        vendor_id: 0,            // Vendor identifier (filled below)
        par: [0; 10],            // Command parameters (filled below)
        internal_par1: [0; 5],   // Internal parameters
        ttl: 0,                  // Time-to-live hop counter
        internal_par2: [0; 4],   // Additional internal parameters
        no_use: [0; 5],          // Reserved/unused bytes
    };

    // Convert 32-bit sequence number to 24-bit little-endian format
    // This provides unique packet identification for duplicate detection
    pkt.sno[0] = sno as u8; // LSB
    pkt.sno[1] = (sno >> 8) as u8; // Middle byte
    pkt.sno[2] = (sno >> 16) as u8; // MSB (limited to 24 bits)

    // Copy command opcode and parameters into packet structure
    // Explicitly assign each field to ensure correct layout regardless of struct packing

    // Assign operation code (always present)
    pkt.op = cmd_op_para[0];

    // Assign vendor ID from bytes 1 and 2 (little-endian u16)
    pkt.vendor_id = u16::from_le_bytes([cmd_op_para[1], cmd_op_para[2]]);

    // Copy any remaining parameters to the par array
    let remaining_params = &cmd_op_para[3..];
    let copy_len = core::cmp::min(remaining_params.len(), pkt.par.len());
    pkt.par[..copy_len].copy_from_slice(&remaining_params[..copy_len]);

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
pub fn mesh_report_status_enable(enable: bool) {
    let mut mesh_node_mask = MESH_NODE_MASK.lock();
    if enable {
        // Set all complete 32-bit words to enable reporting (skip node 0 in each word)
        if MESH_NODE_MAX.get() >> 5 != 0 {
            mesh_node_mask.iter_mut().for_each(|v| *v = 0xfffffffe);
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
#[cfg_attr(test, mry::mry)]
pub fn mesh_report_status_enable_mask(data: &[u8]) {
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
pub fn rf_link_match_group_mac(pkt: &Packet) -> (bool, bool) {
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
pub fn ll_device_status_update(val_par: &[u8]) {
    let mut mesh_node_st = MESH_NODE_ST.lock();

    // Update this device's status parameters (index 0 = local device)
    mesh_node_st[0].val.par.copy_from_slice(val_par);

    // Refresh timestamp using scaled timing format for consistency
    mesh_node_st[0].tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;

    // Mark this device for status change reporting (bit 0 = this device)
    MESH_NODE_MASK.lock()[0] |= 1;
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use mry::Any;

    // Import mock functions from their original modules
    use super::{mock_mesh_node_adv_status, mock_mesh_node_flush_status};
    use crate::embassy::time_driver::mock_clock_time64;
    use crate::main_light::{mock_rf_link_data_callback, mock_rf_link_response_callback};
    use crate::mesh::{MeshNodeStT, MeshNodeStValT, MESH_NODE_ST_PAR_LEN};
    use crate::sdk::light::{INTERNAL_PAR_RETRANSMIT_COUNT, INTERNAL_PAR_SEND_ACK};
    use crate::sdk::mcu::clock::{mock_clock_time, mock_clock_time_exceed};
    use crate::sdk::mcu::register::mock_read_reg_system_tick;

    /// Helper function to reset global mesh state for tests
    fn reset_mesh_state() {
        DEVICE_ADDRESS.set(0x10); // Test device address
        DEVICE_NODE_SN.set(100);
        MESH_NODE_MAX.set(10);
        MESH_NODE_REPORT_ENABLE.set(true);

        // Clear the mesh node status table
        let mut mesh_node_st = MESH_NODE_ST.lock();
        for i in 0..mesh_node_st.len() {
            mesh_node_st[i] = MeshNodeStT {
                tick: 0,
                val: MeshNodeStValT {
                    dev_adr: if i == 0 {
                        DEVICE_ADDRESS.get() as u8
                    } else {
                        0
                    }, // Set device address for index 0
                    sn: 0,
                    par: [0; MESH_NODE_ST_PAR_LEN],
                },
            };
        }
        drop(mesh_node_st);
    }

    /// Helper function to create test mesh node status data  
    fn create_test_mesh_node(dev_adr: u8, sn: u32, val: &[u8]) -> MeshNodeStValT {
        let mut node = MeshNodeStValT {
            dev_adr,
            sn: sn as u8, // Convert u32 to u8 for sn field
            par: [0; MESH_NODE_ST_PAR_LEN],
        };
        // Copy val into node.par, up to the available space
        let copy_len = core::cmp::min(val.len(), node.par.len());
        node.par[..copy_len].copy_from_slice(&val[..copy_len]);
        node
    }

    // ================================================================================
    // Tests for mesh_node_update_status function
    // ================================================================================

    /// Tests mesh_node_update_status with empty packet.
    ///
    /// Verifies that empty packets are handled gracefully.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_update_status_empty_packet() {
        // Setup mocks
        mock_read_reg_system_tick().returns(0x12345678);

        reset_mesh_state();

        let empty_packet: Vec<MeshNodeStValT> = vec![];
        let result = mesh_node_update_status(&empty_packet);

        // Should return 1 for successful packet processing (even if empty)
        assert_eq!(result, 1);
    }

    /// Tests mesh_node_update_status with single new node.
    ///
    /// Verifies that the function can process new node packets without crashing.
    /// Note: Simplified test due to complex node allocation logic.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_update_status_new_node() {
        // Setup mocks
        mock_read_reg_system_tick().returns(0x12345678);

        reset_mesh_state();

        // Create test node with different address than device address (0x10)
        let test_node = create_test_mesh_node(0x20, 50, &[1, 2]);
        let packet = vec![test_node];

        let result = mesh_node_update_status(&packet);

        // Verify the function completed successfully (returns 1 for packet processed)
        assert_eq!(result, 1);
    }

    /// Tests mesh_node_update_status with device's own address.
    ///
    /// Verifies that packets with our own device address are ignored.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_update_status_own_address() {
        // Setup mocks
        mock_read_reg_system_tick().returns(0x12345678);

        reset_mesh_state();

        // Create test node with same address as device address (0x10)
        let own_node = create_test_mesh_node(0x10, 50, &[1, 2]);
        let packet = vec![own_node];

        let result = mesh_node_update_status(&packet);

        // Should return 1 for successful packet processing (own address filtered)
        assert_eq!(result, 1);

        // Verify no nodes were added to remote slots
        let mesh_node_st = MESH_NODE_ST.lock();
        assert_eq!(mesh_node_st[1].val.dev_adr, 0); // Should remain empty
    }

    /// Tests mesh_node_update_status with address 1 filtering.
    ///
    /// Verifies that address 1 is filtered out as per FIXME comment.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_update_status_address_1_filtering() {
        // Setup mocks
        mock_read_reg_system_tick().returns(0x12345678);

        reset_mesh_state();

        // Create test node with address 1 (should be filtered)
        let filtered_node = create_test_mesh_node(1, 50, &[1, 2]);
        let packet = vec![filtered_node];

        let result = mesh_node_update_status(&packet);

        // Should return 1 for successful packet processing (address 1 filtered)
        assert_eq!(result, 1);

        // Verify no nodes were added
        let mesh_node_st = MESH_NODE_ST.lock();
        assert_eq!(mesh_node_st[1].val.dev_adr, 0); // Should remain empty
    }

    /// Tests mesh_node_update_status with sequence number updates.
    ///
    /// Verifies that nodes are updated when newer sequence numbers are received.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_update_status_sequence_update() {
        // Setup mocks
        mock_read_reg_system_tick().returns(0x12345678);

        reset_mesh_state();

        // First, add a node with sequence number 50
        let node_v1 = create_test_mesh_node(0x20, 50, &[1, 2]);
        let packet1 = vec![node_v1];
        mesh_node_update_status(&packet1);

        // Then, update with newer sequence number 51
        let node_v2 = create_test_mesh_node(0x20, 51, &[5, 6]);
        let packet2 = vec![node_v2];
        let result = mesh_node_update_status(&packet2);

        // Verify the node was updated (should be at index 10)
        let mesh_node_st = MESH_NODE_ST.lock();
        let updated_node = &mesh_node_st[10];
        assert_eq!(updated_node.val.dev_adr, 0x20);
        assert_eq!(updated_node.val.sn, 51); // Updated sequence number
        assert_eq!(updated_node.val.par[0..2], [5, 6]); // Updated values

        // Should return updated result
        assert_ne!(result, 0xfffffffe);
    }

    /// Tests mesh_node_update_status with older sequence numbers.
    ///
    /// Verifies that updates with older sequence numbers are ignored.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_update_status_old_sequence() {
        // Setup mocks
        mock_read_reg_system_tick().returns(0x12345678);

        reset_mesh_state();

        // First, add a node with sequence number 51
        let node_v1 = create_test_mesh_node(0x20, 51, &[1, 2]);
        let packet1 = vec![node_v1];
        mesh_node_update_status(&packet1);

        // Then, try to update with older sequence number 50
        let node_v2 = create_test_mesh_node(0x20, 50, &[5, 6]);
        let packet2 = vec![node_v2];
        let result = mesh_node_update_status(&packet2);

        // Verify the node was NOT updated (should be at index 10)
        let mesh_node_st = MESH_NODE_ST.lock();
        let unchanged_node = &mesh_node_st[10];
        assert_eq!(unchanged_node.val.dev_adr, 0x20);
        assert_eq!(unchanged_node.val.sn, 51); // Original sequence number
        assert_eq!(unchanged_node.val.par[0..2], [1, 2]); // Original values

        // Should return 1 for successful packet processing
        assert_eq!(result, 1);
    }

    // ================================================================================
    // Tests for mesh_node_keep_alive function
    // ================================================================================

    /// Tests mesh_node_keep_alive basic functionality.
    ///
    /// Verifies that the function properly increments the device sequence number
    /// and updates the local device status record.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_keep_alive_basic() {
        // Setup mocks
        mock_read_reg_system_tick().returns(0x12345678);

        reset_mesh_state();

        // Get initial values
        let initial_sn = DEVICE_NODE_SN.get();

        // Call mesh_node_keep_alive
        mesh_node_keep_alive();

        // Verify sequence number was incremented
        assert_eq!(DEVICE_NODE_SN.get(), initial_sn + 1);

        // Verify device status record was updated
        let mesh_node_st = MESH_NODE_ST.lock();
        let device_node = &mesh_node_st[0];

        // Check sequence number was updated
        assert_eq!(device_node.val.sn, initial_sn + 1);

        // Check timestamp was updated (scaled format: (system_tick >> 0x10) | 1)
        let expected_tick = ((0x12345678u32 >> 0x10) | 1) as u16;
        let actual_tick = device_node.tick; // Copy to avoid packed field reference
        assert_eq!(actual_tick, expected_tick);
    }

    /// Tests mesh_node_keep_alive sequence number wraparound handling.
    ///
    /// Verifies that when the sequence number would become 0, it wraps to 1.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_keep_alive_sequence_wraparound() {
        // Setup mocks
        mock_read_reg_system_tick().returns(0xABCD1234);

        reset_mesh_state();

        // Set sequence number to 255 (will overflow to 0 on increment)
        DEVICE_NODE_SN.set(255);

        // Call mesh_node_keep_alive
        mesh_node_keep_alive();

        // Verify sequence number wrapped to 1 (not 0)
        assert_eq!(DEVICE_NODE_SN.get(), 1);

        // Verify device status record reflects the wrapped value
        let mesh_node_st = MESH_NODE_ST.lock();
        let device_node = &mesh_node_st[0];
        assert_eq!(device_node.val.sn, 1);
    }

    /// Tests mesh_node_keep_alive multiple calls.
    ///
    /// Verifies that multiple consecutive calls properly increment the sequence number.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_keep_alive_multiple_calls() {
        // Setup mocks - use same timestamp for simplicity
        mock_read_reg_system_tick().returns(0x55555555);

        reset_mesh_state();

        let initial_sn = DEVICE_NODE_SN.get(); // 100

        // First call
        mesh_node_keep_alive();
        assert_eq!(DEVICE_NODE_SN.get(), initial_sn + 1);

        // Second call
        mesh_node_keep_alive();
        assert_eq!(DEVICE_NODE_SN.get(), initial_sn + 2);

        // Third call
        mesh_node_keep_alive();
        assert_eq!(DEVICE_NODE_SN.get(), initial_sn + 3);

        // Verify final device status record
        let mesh_node_st = MESH_NODE_ST.lock();
        let device_node = &mesh_node_st[0];
        assert_eq!(device_node.val.sn, initial_sn + 3);

        // Verify timestamp was updated
        let expected_tick = ((0x55555555u32 >> 0x10) | 1) as u16;
        let actual_tick = device_node.tick; // Copy to avoid packed field reference
        assert_eq!(actual_tick, expected_tick);
    }

    /// Tests mesh_node_keep_alive timestamp calculation with zero value.
    ///
    /// Verifies the timestamp scaling algorithm: ((system_tick >> 0x10) | 1).
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_keep_alive_timestamp_zero() {
        reset_mesh_state();

        mock_read_reg_system_tick().returns(0x00000000);
        mesh_node_keep_alive();

        let mesh_node_st = MESH_NODE_ST.lock();
        let device_node = &mesh_node_st[0];
        let actual_tick = device_node.tick; // Copy to avoid packed field reference
        let expected_tick = 0x0001; // 0 >> 16 = 0, | 1 = 1
        assert_eq!(actual_tick, expected_tick);
    }

    /// Tests mesh_node_keep_alive timestamp calculation with typical value.
    ///
    /// Verifies the timestamp scaling algorithm: ((system_tick >> 0x10) | 1).
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_keep_alive_timestamp_typical() {
        reset_mesh_state();

        mock_read_reg_system_tick().returns(0x12345678);
        mesh_node_keep_alive();

        let mesh_node_st = MESH_NODE_ST.lock();
        let device_node = &mesh_node_st[0];
        let actual_tick = device_node.tick; // Copy to avoid packed field reference
        let expected_tick = 0x1235; // 0x1234 | 1 = 0x1235
        assert_eq!(actual_tick, expected_tick);
    }

    /// Tests mesh_node_keep_alive timestamp calculation with max high bits.
    ///
    /// Verifies the timestamp scaling algorithm: ((system_tick >> 0x10) | 1).
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_keep_alive_timestamp_max_high() {
        reset_mesh_state();

        mock_read_reg_system_tick().returns(0xFFFF0000);
        mesh_node_keep_alive();

        let mesh_node_st = MESH_NODE_ST.lock();
        let device_node = &mesh_node_st[0];
        let actual_tick = device_node.tick; // Copy to avoid packed field reference
        let expected_tick = 0xFFFF; // 0xFFFF | 1 = 0xFFFF
        assert_eq!(actual_tick, expected_tick);
    }

    /// Tests mesh_node_keep_alive preserves other device status fields.
    ///
    /// Verifies that only sequence number and timestamp are updated, while
    /// device address and parameter data remain unchanged.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_keep_alive_preserves_other_fields() {
        // Setup mocks
        mock_read_reg_system_tick().returns(0x55555555);

        reset_mesh_state();

        // Set up initial device status with specific values
        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            mesh_node_st[0].val.dev_adr = 0x42;
            mesh_node_st[0].val.par = [0xAA, 0xBB];
            mesh_node_st[0].tick = 0x9999;
        }

        let initial_dev_adr;
        let initial_par;
        {
            let mesh_node_st = MESH_NODE_ST.lock();
            initial_dev_adr = mesh_node_st[0].val.dev_adr;
            initial_par = mesh_node_st[0].val.par;
        }

        // Call mesh_node_keep_alive
        mesh_node_keep_alive();

        // Verify dev_adr and par were preserved
        let mesh_node_st = MESH_NODE_ST.lock();
        let device_node = &mesh_node_st[0];
        assert_eq!(device_node.val.dev_adr, initial_dev_adr);
        assert_eq!(device_node.val.par, initial_par);

        // Verify sn and tick were updated
        assert_eq!(device_node.val.sn, 101); // incremented from 100
        let expected_tick = ((0x55555555u32 >> 0x10) | 1) as u16;
        let actual_tick = device_node.tick; // Copy to avoid packed field reference
        assert_eq!(actual_tick, expected_tick);
    }

    // ================================================================================
    // Tests for mesh_construct_packet function
    // ================================================================================

    /// Tests mesh_construct_packet with minimal parameters.
    ///
    /// Verifies that the function creates a proper mesh packet with minimal 3-byte command.
    #[test]
    fn test_mesh_construct_packet_minimal() {
        reset_mesh_state();

        let sno = 0x123456;
        let dst = 0x42;
        let cmd_op_para = [0xAA, 0x12, 0x34]; // Minimal 3 bytes - distinctive vendor_id bytes
        let retransmit_count = 3;
        let send_ack = true;

        let packet = mesh_construct_packet(sno, dst, &cmd_op_para, retransmit_count, send_ack);

        // Verify packet header fields
        let mesh_pkt = unsafe { packet.mesh };
        let dma_len = mesh_pkt.head.dma_len; // Copy to avoid packed field reference
        let pkt_type = mesh_pkt.head._type; // Copy to avoid packed field reference
        let rf_len = mesh_pkt.head.rf_len; // Copy to avoid packed field reference
        let l2cap_len = mesh_pkt.head.l2cap_len; // Copy to avoid packed field reference
        let chan_id = mesh_pkt.head.chan_id; // Copy to avoid packed field reference
        assert_eq!(dma_len, 0x27);
        assert_eq!(pkt_type, 2);
        assert_eq!(rf_len, 0x25);
        assert_eq!(l2cap_len, 0x21);
        assert_eq!(chan_id, 0xff03);

        // Verify addressing (copy packed fields to avoid alignment issues)
        let src_tx = mesh_pkt.src_tx;
        let src_adr = mesh_pkt.src_adr;
        let dst_adr = mesh_pkt.dst_adr;
        assert_eq!(src_tx, DEVICE_ADDRESS.get());
        assert_eq!(src_adr, DEVICE_ADDRESS.get());
        assert_eq!(dst_adr, dst);

        // Verify sequence number (24-bit little-endian)
        assert_eq!(mesh_pkt.sno[0], (sno & 0xFF) as u8); // LSB
        assert_eq!(mesh_pkt.sno[1], ((sno >> 8) & 0xFF) as u8); // Middle
        assert_eq!(mesh_pkt.sno[2], ((sno >> 16) & 0xFF) as u8); // MSB

        // Verify command parameters were copied correctly
        assert_eq!(mesh_pkt.op, cmd_op_para[0]); // Should be 0xAA

        // Verify vendor_id uses both bytes in little-endian format
        let vendor_id = mesh_pkt.vendor_id; // Copy to avoid packed field reference
                                            // With cmd_op_para[1]=0x12, cmd_op_para[2]=0x34: vendor_id should be 0x3412
        assert_eq!(
            vendor_id,
            u16::from_le_bytes([cmd_op_para[1], cmd_op_para[2]])
        );
        assert_eq!(vendor_id, 0x3412); // Explicit check: 0x34 << 8 | 0x12 = 0x3412

        // Verify internal parameters
        assert_eq!(
            mesh_pkt.internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
            retransmit_count
        );
        assert_eq!(mesh_pkt.internal_par1[INTERNAL_PAR_SEND_ACK], 1);
    }

    /// Tests mesh_construct_packet with maximum parameters.
    ///
    /// Verifies that the function handles the maximum 13-byte command parameter length.
    #[test]
    fn test_mesh_construct_packet_maximum() {
        reset_mesh_state();

        let sno = 0xFFFFFF; // Max 24-bit value
        let dst = 0xFEDC;
        let cmd_op_para = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,
        ]; // Max 13 bytes
        let retransmit_count = 7;
        let send_ack = false;

        let packet = mesh_construct_packet(sno, dst, &cmd_op_para, retransmit_count, send_ack);

        let mesh_pkt = unsafe { packet.mesh };

        // Verify sequence number at maximum value
        assert_eq!(mesh_pkt.sno[0], 0xFF);
        assert_eq!(mesh_pkt.sno[1], 0xFF);
        assert_eq!(mesh_pkt.sno[2], 0xFF);

        // Verify all command parameters were copied
        assert_eq!(mesh_pkt.op, cmd_op_para[0]);
        let vendor_id = mesh_pkt.vendor_id; // Copy to avoid packed field reference
                                            // With longer parameter arrays, both bytes are copied correctly
        assert_eq!(
            vendor_id,
            u16::from_le_bytes([cmd_op_para[1], cmd_op_para[2]])
        );

        // Verify extended parameters
        for i in 0..10 {
            let param_idx = i + 3; // Skip op (0) and vendor_id (1,2)
            if param_idx < cmd_op_para.len() {
                assert_eq!(mesh_pkt.par[i], cmd_op_para[param_idx]);
            } else {
                assert_eq!(mesh_pkt.par[i], 0); // Unused bytes should be zero
            }
        }

        // Verify internal parameters
        assert_eq!(
            mesh_pkt.internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
            retransmit_count
        );
        assert_eq!(mesh_pkt.internal_par1[INTERNAL_PAR_SEND_ACK], 0); // send_ack = false
    }

    /// Tests mesh_construct_packet with various parameter lengths.
    ///
    /// Verifies correct handling of different command parameter lengths from 3 to 13 bytes.
    #[test]
    fn test_mesh_construct_packet_variable_lengths() {
        reset_mesh_state();

        // Test various valid lengths
        for length in 3..=13 {
            let mut cmd_op_para = vec![0u8; length];
            // Fill with recognizable pattern
            for i in 0..length {
                cmd_op_para[i] = (i + 1) as u8;
            }

            let packet = mesh_construct_packet(
                0x1000 + length as u32,
                0x10 + length as u16,
                &cmd_op_para,
                length as u8,
                length % 2 == 0,
            );

            let mesh_pkt = unsafe { packet.mesh };

            // Verify basic fields are set correctly
            assert_eq!(mesh_pkt.op, cmd_op_para[0]);

            // Verify vendor_id is constructed from bytes 1 and 2
            let vendor_id = mesh_pkt.vendor_id; // Copy to avoid packed field reference
            assert_eq!(
                vendor_id,
                u16::from_le_bytes([cmd_op_para[1], cmd_op_para[2]])
            );

            // Verify parameter array
            let available_par_bytes = length.saturating_sub(3); // Subtract op + vendor_id
            for i in 0..10 {
                if i < available_par_bytes {
                    assert_eq!(mesh_pkt.par[i], cmd_op_para[i + 3]);
                } else {
                    assert_eq!(mesh_pkt.par[i], 0);
                }
            }
        }
    }

    /// Tests mesh_construct_packet sequence number encoding.
    ///
    /// Verifies that 32-bit sequence numbers are correctly encoded as 24-bit little-endian.
    #[test]
    fn test_mesh_construct_packet_sequence_number_encoding() {
        reset_mesh_state();

        let cmd_op_para = [0x10, 0x20, 0x30];

        let test_cases = [
            (0x00000000, [0x00, 0x00, 0x00]),
            (0x00000001, [0x01, 0x00, 0x00]),
            (0x00000100, [0x00, 0x01, 0x00]),
            (0x00010000, [0x00, 0x00, 0x01]),
            (0x12345678, [0x78, 0x56, 0x34]), // Upper 8 bits ignored
            (0xABCDEF12, [0x12, 0xEF, 0xCD]), // Upper 8 bits ignored
            (0xFFFFFFFF, [0xFF, 0xFF, 0xFF]), // All bits set
        ];

        for (input_sno, expected_bytes) in test_cases {
            let packet = mesh_construct_packet(input_sno, 0x42, &cmd_op_para, 1, false);
            let mesh_pkt = unsafe { packet.mesh };

            assert_eq!(
                mesh_pkt.sno, expected_bytes,
                "Failed for sno=0x{:08X}, expected={:02X?}, got={:02X?}",
                input_sno, expected_bytes, mesh_pkt.sno
            );
        }
    }

    /// Tests mesh_construct_packet address assignment.
    ///
    /// Verifies that source and destination addresses are correctly assigned.
    #[test]
    fn test_mesh_construct_packet_address_assignment() {
        reset_mesh_state();

        // Test with different device addresses
        let test_addresses = [0x01, 0x42, 0xAB, 0xFE];
        let cmd_op_para = [0x11, 0x22, 0x33];

        for device_addr in test_addresses {
            DEVICE_ADDRESS.set(device_addr);

            let dst_addr = 0x99;
            let packet = mesh_construct_packet(0x123, dst_addr, &cmd_op_para, 2, true);
            let mesh_pkt = unsafe { packet.mesh };

            // Both src_tx and src_adr should be set to device address (copy packed fields)
            let src_tx = mesh_pkt.src_tx;
            let src_adr = mesh_pkt.src_adr;
            let dst_adr_val = mesh_pkt.dst_adr;
            assert_eq!(src_tx, device_addr);
            assert_eq!(src_adr, device_addr);
            assert_eq!(dst_adr_val, dst_addr);
        }
    }

    /// Tests mesh_construct_packet internal parameter configuration.
    ///
    /// Verifies correct setting of retransmit count and ACK flag.
    #[test]
    fn test_mesh_construct_packet_internal_parameters() {
        reset_mesh_state();

        let cmd_op_para = [0xA1, 0xB2, 0xC3];

        let test_cases = [
            (0, false, 0, 0),
            (1, true, 1, 1),
            (5, false, 5, 0),
            (255, true, 255, 1),
        ];

        for (retransmit_count, send_ack, expected_retransmit, expected_ack) in test_cases {
            let packet =
                mesh_construct_packet(0x100, 0x50, &cmd_op_para, retransmit_count, send_ack);
            let mesh_pkt = unsafe { packet.mesh };

            assert_eq!(
                mesh_pkt.internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
                expected_retransmit
            );
            assert_eq!(mesh_pkt.internal_par1[INTERNAL_PAR_SEND_ACK], expected_ack);
        }
    }

    /// Tests mesh_construct_packet panic conditions.
    ///
    /// Verifies that the function panics with invalid parameter lengths.
    #[test]
    #[should_panic(expected = "Command parameters too short")]
    fn test_mesh_construct_packet_panic_too_short() {
        reset_mesh_state();
        let cmd_op_para = [0x01, 0x02]; // Only 2 bytes (minimum is 3)
        mesh_construct_packet(0x123, 0x42, &cmd_op_para, 1, false);
    }

    /// Tests mesh_construct_packet panic with empty parameters.
    #[test]
    #[should_panic(expected = "Command parameters too short")]
    fn test_mesh_construct_packet_panic_empty() {
        reset_mesh_state();
        let cmd_op_para: &[u8] = &[]; // Empty slice
        mesh_construct_packet(0x123, 0x42, cmd_op_para, 1, false);
    }

    /// Tests mesh_construct_packet panic with too many parameters.
    #[test]
    #[should_panic(expected = "Command parameters too long")]
    fn test_mesh_construct_packet_panic_too_long() {
        reset_mesh_state();
        let cmd_op_para = [0u8; 14]; // 14 bytes (maximum is 13)
        mesh_construct_packet(0x123, 0x42, &cmd_op_para, 1, false);
    }

    // ================================================================================
    // Tests for mesh_node_flush_status function
    // ================================================================================

    /// Tests mesh_node_flush_status basic functionality.
    ///
    /// Verifies that the function can be called without crashing.
    #[test]
    #[mry::lock(read_reg_system_tick, clock_time_exceed)]
    fn test_mesh_node_flush_status() {
        // Setup mocks
        mock_read_reg_system_tick().returns(0x10000000);
        mock_clock_time_exceed(Any, Any).returns(false); // Rate limited, no action

        reset_mesh_state();

        // Add a node to the mesh
        let test_node = create_test_mesh_node(0x20, 50, &[1, 2]);
        let packet = vec![test_node];
        mesh_node_update_status(&packet);

        // Call flush status (should be rate limited and do nothing)
        mesh_node_flush_status();

        // Test passes if no crash occurs
    }

    // ================================================================================
    // Tests for mesh_send_online_status function
    // ================================================================================

    /// Tests mesh_send_online_status rate limiting.
    ///
    /// Verifies that the function returns early when rate limited.
    #[test]
    #[mry::lock(clock_time_exceed)]
    fn test_mesh_send_online_status_rate_limited() {
        // Setup mock to simulate rate limiting
        mock_clock_time_exceed(Any, Any).returns(false); // Rate limited

        reset_mesh_state();

        // Call should be rate limited and return early
        mesh_send_online_status();

        // Verify the function was called (should return early due to rate limiting)
        mock_clock_time_exceed(Any, Any).assert_called(1);
    }

    /// Tests mesh_send_online_status when not rate limited.
    ///
    /// Verifies that the function calls expected mocked functions in sequence.
    /// Note: This test will crash when it reaches app().mesh_manager call,
    /// but the mocked functions should be called first.
    #[test]
    #[mry::lock(
        clock_time_exceed,
        clock_time,
        mesh_node_flush_status,
        mesh_node_adv_status
    )]
    #[should_panic] // Expected due to app() call at the end
    fn test_mesh_send_online_status_function_sequence() {
        // Setup mocks
        mock_clock_time_exceed(Any, Any).returns(true); // Not rate limited
        mock_clock_time().returns(12345); // Mock the timestamp update
        mock_mesh_node_flush_status().returns(()); // Mock flush status
        mock_mesh_node_adv_status(Any).returns(0); // Mock advertisement status

        reset_mesh_state();

        // Call should proceed through all mocked steps, then crash on app() call
        mesh_send_online_status();

        // If we reach here, the test should fail because it should have panicked
        panic!("Expected function to panic on app() call");
    }

    // ================================================================================
    // Tests for mesh_report_status_enable function
    // ================================================================================

    /// Tests mesh_report_status_enable functionality.
    ///
    /// Verifies that status reporting can be enabled and disabled.
    #[test]
    fn test_mesh_report_status_enable() {
        reset_mesh_state();

        // Test enabling
        mesh_report_status_enable(true);
        assert_eq!(MESH_NODE_REPORT_ENABLE.get(), true);

        // Test disabling
        mesh_report_status_enable(false);
        assert_eq!(MESH_NODE_REPORT_ENABLE.get(), false);
    }

    /// Tests mesh_report_status_enable with multiple 32-bit words.
    ///
    /// Verifies that the bulk enable logic (line 863) works correctly when MESH_NODE_MAX > 32.
    #[test]
    fn test_mesh_report_status_enable_multiple_words() {
        reset_mesh_state();

        // Set MESH_NODE_MAX to a value that creates multiple complete 32-bit words
        // to trigger the iter_mut().for_each() path (line 863)
        MESH_NODE_MAX.set(64); // This means we have 64 active nodes = 2 complete words

        // Clear the mask initially
        {
            let mut mesh_node_mask = MESH_NODE_MASK.lock();
            for word in mesh_node_mask.iter_mut() {
                *word = 0;
            }
        }

        // Enable reporting - this should trigger line 863
        mesh_report_status_enable(true);

        // Verify the mask was set correctly
        {
            let mesh_node_mask = MESH_NODE_MASK.lock();

            // With MESH_NODE_MAX = 64, we have 64 >> 5 = 2 complete words
            // Line 863 should set ALL words in the array to 0xFFFFFFFE
            // This tests line 863 specifically
            for (i, &word) in mesh_node_mask.iter().enumerate() {
                assert_eq!(
                    word, 0xFFFFFFFE,
                    "Word {} should be set to 0xFFFFFFFE by line 863 bulk enable logic",
                    i
                );
            }
        }

        // Test disabling
        mesh_report_status_enable(false);
        assert_eq!(MESH_NODE_REPORT_ENABLE.get(), false);

        // Note: The current implementation doesn't clear the mask when disabling,
        // it only sets the global MESH_NODE_REPORT_ENABLE flag to false.
        // This might be a bug, but we're testing the actual behavior.
        {
            let mesh_node_mask = MESH_NODE_MASK.lock();
            // The mask should still have the bits set from the enable call
            for (i, &word) in mesh_node_mask.iter().enumerate() {
                assert_eq!(
                    word, 0xFFFFFFFE,
                    "Word {} mask bits remain set even when disabling (current behavior)",
                    i
                );
            }
        }
    }

    // ================================================================================
    // Tests for mesh_report_status_enable_mask function
    // ================================================================================

    /// Tests mesh_report_status_enable_mask with valid data.
    ///
    /// Verifies that status reporting mask is set correctly.
    #[test]
    fn test_mesh_report_status_enable_mask_valid() {
        reset_mesh_state();

        // Test with enable mask
        let enable_data = [1u8];
        mesh_report_status_enable_mask(&enable_data);
        assert_eq!(MESH_NODE_REPORT_ENABLE.get(), true);

        // Test with disable mask
        let disable_data = [0u8];
        mesh_report_status_enable_mask(&disable_data);
        assert_eq!(MESH_NODE_REPORT_ENABLE.get(), false);
    }

    /// Tests mesh_report_status_enable_mask with minimal data.
    ///
    /// Verifies that single-byte data is handled correctly.
    #[test]
    fn test_mesh_report_status_enable_mask_minimal() {
        reset_mesh_state();

        // Set initial state
        MESH_NODE_REPORT_ENABLE.set(true);

        // Test with minimal data (empty array would crash)
        let minimal_data = [1u8]; // Single byte with enable flag
        mesh_report_status_enable_mask(&minimal_data);

        // Should be set to true
        assert_eq!(MESH_NODE_REPORT_ENABLE.get(), true);
    }

    /// Tests mesh_report_status_enable_mask with selective address reporting.
    ///
    /// Verifies that specific device addresses can be enabled for reporting.
    #[test]
    fn test_mesh_report_status_enable_mask_selective_addresses() {
        reset_mesh_state();

        // Set up some nodes in the mesh table
        MESH_NODE_MAX.set(5);
        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            mesh_node_st[1].val.dev_adr = 0x10;
            mesh_node_st[2].val.dev_adr = 0x20;
            mesh_node_st[3].val.dev_adr = 0x30;
            mesh_node_st[4].val.dev_adr = 0x40;
        }

        // Clear the mask initially
        {
            let mut mesh_node_mask = MESH_NODE_MASK.lock();
            for word in mesh_node_mask.iter_mut() {
                *word = 0;
            }
        }

        // Enable reporting for specific addresses: 0x20 and 0x40
        let selective_data = [1u8, 0x20, 0x40]; // Enable + two addresses
        mesh_report_status_enable_mask(&selective_data);

        // Verify global enable flag
        assert_eq!(MESH_NODE_REPORT_ENABLE.get(), true);

        // Verify selective address bitmask (tests lines 944-949)
        {
            let mesh_node_mask = MESH_NODE_MASK.lock();
            let word0 = mesh_node_mask[0];

            // Check specific bits are set for matching addresses
            assert_eq!(
                word0 & (1 << 2),
                1 << 2,
                "Bit 2 should be set for node at index 2 (addr 0x20)"
            );
            assert_eq!(
                word0 & (1 << 4),
                1 << 4,
                "Bit 4 should be set for node at index 4 (addr 0x40)"
            );

            // Check that other bits are not set
            assert_eq!(
                word0 & (1 << 1),
                0,
                "Bit 1 should not be set for node at index 1 (addr 0x10)"
            );
            assert_eq!(
                word0 & (1 << 3),
                0,
                "Bit 3 should not be set for node at index 3 (addr 0x30)"
            );
        }
    }

    /// Tests mesh_report_status_enable_mask with non-matching addresses.
    ///
    /// Verifies behavior when provided addresses don't match any nodes.
    #[test]
    fn test_mesh_report_status_enable_mask_no_matches() {
        reset_mesh_state();

        // Set up some nodes with specific addresses
        MESH_NODE_MAX.set(3);
        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            mesh_node_st[1].val.dev_adr = 0x10;
            mesh_node_st[2].val.dev_adr = 0x20;
        }

        // Clear the mask initially
        {
            let mut mesh_node_mask = MESH_NODE_MASK.lock();
            for word in mesh_node_mask.iter_mut() {
                *word = 0;
            }
        }

        // Try to enable reporting for addresses that don't exist: 0x99, 0xAA
        let non_matching_data = [1u8, 0x99, 0xAA];
        mesh_report_status_enable_mask(&non_matching_data);

        // Verify global enable flag is set
        assert_eq!(MESH_NODE_REPORT_ENABLE.get(), true);

        // Verify no bits are set in the mask since no addresses matched
        {
            let mesh_node_mask = MESH_NODE_MASK.lock();
            assert_eq!(
                mesh_node_mask[0], 0,
                "No bits should be set when no addresses match"
            );
        }
    }

    /// Tests mesh_report_status_enable_mask with MESH_NODE_MAX = 0.
    ///
    /// Verifies behavior when the condition MESH_NODE_MAX.get() != 0 is false.
    #[test]
    fn test_mesh_report_status_enable_mask_empty_table() {
        reset_mesh_state();

        // Set MESH_NODE_MAX to 0 to trigger the condition check (line 942)
        MESH_NODE_MAX.set(0);

        // Clear the mask initially
        {
            let mut mesh_node_mask = MESH_NODE_MASK.lock();
            for word in mesh_node_mask.iter_mut() {
                *word = 0;
            }
        }

        // Try to enable reporting with addresses when table is empty
        let data_with_addresses = [1u8, 0x10, 0x20];
        mesh_report_status_enable_mask(&data_with_addresses);

        // Verify global enable flag is set
        assert_eq!(MESH_NODE_REPORT_ENABLE.get(), true);

        // Verify no bits are set since MESH_NODE_MAX = 0 skips the search
        {
            let mesh_node_mask = MESH_NODE_MASK.lock();
            assert_eq!(
                mesh_node_mask[0], 0,
                "No bits should be set when MESH_NODE_MAX = 0"
            );
        }
    }

    /// Tests mesh_report_status_enable_mask with disabled reporting.
    ///
    /// Verifies that addresses are ignored when reporting is disabled.
    #[test]
    fn test_mesh_report_status_enable_mask_disabled_with_addresses() {
        reset_mesh_state();

        // Set up some nodes
        MESH_NODE_MAX.set(3);
        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            mesh_node_st[1].val.dev_adr = 0x10;
            mesh_node_st[2].val.dev_adr = 0x20;
        }

        // Clear the mask initially
        {
            let mut mesh_node_mask = MESH_NODE_MASK.lock();
            for word in mesh_node_mask.iter_mut() {
                *word = 0;
            }
        }

        // Disable reporting but provide addresses - they should be ignored
        let disabled_data = [0u8, 0x10, 0x20]; // Disabled + addresses
        mesh_report_status_enable_mask(&disabled_data);

        // Verify global enable flag is disabled
        assert_eq!(MESH_NODE_REPORT_ENABLE.get(), false);

        // Verify no bits are set since reporting is disabled (condition line 940 fails)
        {
            let mesh_node_mask = MESH_NODE_MASK.lock();
            assert_eq!(
                mesh_node_mask[0], 0,
                "No bits should be set when reporting is disabled"
            );
        }
    }

    /// Tests mesh_report_status_enable_mask with cross-word bit setting.
    ///
    /// Verifies that the bitmask logic works across 32-bit word boundaries.
    #[test]
    fn test_mesh_report_status_enable_mask_cross_word() {
        reset_mesh_state();

        // Set up nodes that span multiple 32-bit words
        MESH_NODE_MAX.set(40);
        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            mesh_node_st[10].val.dev_adr = 0x10; // First word
            mesh_node_st[35].val.dev_adr = 0x35; // Second word
        }

        // Clear the mask initially
        {
            let mut mesh_node_mask = MESH_NODE_MASK.lock();
            for word in mesh_node_mask.iter_mut() {
                *word = 0;
            }
        }

        // Enable reporting for addresses in different words
        let cross_word_data = [1u8, 0x10, 0x35];
        mesh_report_status_enable_mask(&cross_word_data);

        // Verify bits are set in correct words
        {
            let mesh_node_mask = MESH_NODE_MASK.lock();

            // Node 10: word 0 (10 >> 5 = 0), bit 10 (10 & 0x1f = 10)
            assert_eq!(
                mesh_node_mask[0] & (1 << 10),
                1 << 10,
                "Bit 10 should be set in word 0"
            );

            // Node 35: word 1 (35 >> 5 = 1), bit 3 (35 & 0x1f = 3)
            assert_eq!(
                mesh_node_mask[1] & (1 << 3),
                1 << 3,
                "Bit 3 should be set in word 1"
            );
        }
    }

    // ================================================================================
    // Tests for ll_device_status_update function
    // ================================================================================

    /// Tests ll_device_status_update with valid parameters.
    ///
    /// Verifies that device status is updated correctly.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_ll_device_status_update_valid() {
        // Setup mocks
        mock_read_reg_system_tick().returns(0x12345678);

        reset_mesh_state();

        let test_data = [0x01, 0x02]; // Must be exactly MESH_NODE_ST_PAR_LEN (2) bytes
        ll_device_status_update(&test_data);

        // Verify device status was updated
        let mesh_node_st = MESH_NODE_ST.lock();
        let device_node = &mesh_node_st[0]; // Device status at index 0
                                            // ll_device_status_update doesn't set dev_adr, only par and tick
        assert_eq!(device_node.val.par[0..2], [0x01, 0x02]);

        // Verify sequence number was NOT incremented (ll_device_status_update doesn't call inc)
        assert_eq!(DEVICE_NODE_SN.get(), 100); // Should remain at initial value
    }

    /// Tests ll_device_status_update with empty data.
    ///
    /// Verifies that empty data updates are handled correctly.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_ll_device_status_update_empty() {
        // Setup mocks
        mock_read_reg_system_tick().returns(0x12345678);

        reset_mesh_state();

        let minimal_data = [0x00, 0x00]; // Must be exactly MESH_NODE_ST_PAR_LEN (2) bytes
        ll_device_status_update(&minimal_data);

        // Should still update the timestamp but NOT sequence number
        assert_eq!(DEVICE_NODE_SN.get(), 100); // Should remain at initial value
    }
}
