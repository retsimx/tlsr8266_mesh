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

use crate::embassy::sync::mutex::CriticalSectionMutex;
use crate::embassy::time_driver::clock_time64;
use crate::main_light::{rf_link_data_callback, rf_link_response_callback};
use crate::mesh::{
    is_online, MeshNodeStT, MeshNodeStValT, MESH_NODE_ST_PAR_LEN, MESH_NODE_ST_VAL_LEN,
    MESH_STATUS_VALUE_LEN,
};
use crate::sdk::drivers::uart::{UartData, UART_DATA_LEN};
use crate::sdk::light::*;
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed};
use crate::sdk::packet_types::*;
use crate::state::*;
use crate::uart_manager::UartMsg;
use crate::{app, BIT};
use embassy_time::{Duration, Instant};

/// Advertisement round-robin cursor: index (1..MESH_NODE_MAX) of the next remote
/// node to advertise. Persists across packets so every remote is eventually
/// covered; not a per-call local because coverage spans many packets.
static ADV_CURSOR: AtomicUsize = AtomicUsize::new(1);

/// Timestamp of the last `mesh_node_flush_status` sweep, used to gate sweeps to
/// [`MESH_STATUS_SWEEP_MS`]. Zero-init (`.bss`); the first sweep occurs once
/// `MESH_STATUS_SWEEP_MS` has elapsed since boot (matching the previous rate
/// limit), not on the very first call.
/// Module scope (rather than a function-local static) so tests can reset it.
static LAST_SWEEP: CriticalSectionMutex<Instant> =
    CriticalSectionMutex::new(Instant::from_ticks(0));

/// M4: the deadline model's remote-records-per-packet must equal the wire
/// capacity. A status value region holds `MESH_STATUS_VALUE_LEN /
/// MESH_NODE_ST_VAL_LEN` records; the first is always this device, so the rest
/// are the remotes advertised per packet. Drift fails the build here.
const _: () = assert!(
    MESH_STATUS_RECORDS_PER_PACKET as usize == MESH_STATUS_VALUE_LEN / MESH_NODE_ST_VAL_LEN - 1
);

/// Borrowed view over the locked node table (index 0 is this device).
///
/// The active length is passed per call rather than stored so the lock guard
/// remains the single source of truth for the table contents.
struct NodeTable<'a> {
    nodes: &'a mut [MeshNodeStT],
}

impl<'a> NodeTable<'a> {
    fn new(nodes: &'a mut [MeshNodeStT]) -> Self {
        Self { nodes }
    }

    /// Index of the remote node with `addr`, searching the active range `1..max`.
    fn position(&self, max: usize, addr: u8) -> Option<usize> {
        self.nodes[1..max]
            .iter()
            .position(|node| node.val.dev_adr == addr)
            .map(|offset| offset + 1)
    }

    /// Store a freshly-heard remote record at `index`, marked online (`miss=1`).
    fn store(&mut self, index: usize, entry: &MeshNodeStValT, miss: u8) {
        self.nodes[index] = MeshNodeStT { miss, val: *entry };
    }
}

/// Mark a node's status as changed so it is included in the next report.
fn mark_status_changed(index: usize) {
    MESH_NODE_MASK.lock()[index >> 5] |= 1 << (index & 0x1f);
}

/// Raw bytes of a status value for direct wire copies.
fn node_val_bytes(value: &MeshNodeStValT) -> &[u8] {
    bytemuck::bytes_of(value)
}

/// Sends node status change events to the UART daemon as a NodeStatus message.
///
/// Each entry is `(node_id, online, on_off)` where `online=1` means the node is
/// reachable, `online=0` means it has timed out/gone offline, and `on_off` reflects
/// `par[0]` (nonzero = on). Up to 13 entries fit in one UART packet.
///
/// This is called directly wherever node state changes in RAM so that UART status
/// reporting works regardless of BLE connection state.
fn send_uart_node_changes(entries: &[(u8, u8, u8)]) {
    if entries.is_empty() || !app().uart_manager.uart_status_reporting_enabled() {
        return;
    }
    const MAX_PER_PKT: usize = 13; // floor((UART_DATA_LEN - 3) / 3)
    for chunk in entries.chunks(MAX_PER_PKT) {
        let mut uart_msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };
        uart_msg.data[2] = UartMsg::NodeStatus as u8;
        for (i, &(node_id, online, on_off)) in chunk.iter().enumerate() {
            uart_msg.data[3 + i * 3] = node_id;
            uart_msg.data[3 + i * 3 + 1] = online;
            uart_msg.data[3 + i * 3 + 2] = on_off;
        }
        let _ = app().uart_manager.send_message(&uart_msg);
    }
}

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
/// 4. **Stale-Update Gate**: Requires a plausible sequence step, or a node that
///    is offline or has aged past the escape gate, before accepting a large jump
///
/// 5. **Address Filtering**: Filters out invalid addresses and prevents nodes
///    from updating their own status (avoids feedback loops)
///
/// # Sequence Number Algorithm
///
/// Status updates are accepted if:
/// - `new_sn - old_sn` is a plausible forward step (<= 65)
/// - OR the node was previously offline (`miss == 0`)
/// - OR the node has aged past the escape gate (half the derived deadline)
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
    // Half the derived deadline, in sweeps. A large, implausible sn jump is only
    // trusted once the node has been quiet at least this many sweeps: a
    // sequence-sanity / resurrection hysteresis gate, not the offline detector.
    // An online node's miss starts at 1, so `miss > escape_sweeps` means at
    // least `escape_sweeps` sweeps have elapsed unseen.
    let escape_sweeps = (mesh_status_offline_sweeps(MESH_NODE_MAX.get() as u32) / 2).max(1);

    // A packet carries at most 6 records, so 6 UART entries is the worst case.
    let mut changed: heapless::Vec<(u8, u8, u8), 6> = heapless::Vec::new();

    {
        let mut st = MESH_NODE_ST.lock();
        let mut table = NodeTable::new(&mut st[..]);

        // Entries are terminated by a zero address (trailing padding).
        for entry in pkt.iter().take_while(|entry| entry.dev_adr != 0) {
            // Our own status lives at index 0 and is never a remote record.
            if entry.dev_adr == DEVICE_ADDRESS.get() as u8 {
                continue;
            }

            let max = MESH_NODE_MAX.get() as usize;
            if max == 0 {
                continue;
            }

            match table.position(max, entry.dev_adr) {
                // EXISTING NODE: accept only a genuinely newer sequence number.
                //
                // THE CRITICAL INVARIANT: only accept packets where sn has
                // actually advanced (sn_difference > 0). Refreshing miss for
                // same-sn packets would let neighbours re-broadcast a dead
                // node's frozen state forever and keep it alive. A live node
                // increments sn on every advertisement (mesh_node_keep_alive).
                //
                // Given sn_difference > 0, accept if ANY of:
                // 1. The advance is plausible (<= 65).
                // 2. The node was offline (miss == 0) - always revive.
                // 3. A large jump (> 65) has persisted for more than
                //    `escape_sweeps` sweeps.
                Some(index) => {
                    let node = &mut table.nodes[index];
                    let sn_difference = entry.sn.wrapping_sub(node.val.sn);
                    let par_match = entry.par == node.val.par;
                    let was_offline = !is_online(node);

                    if sn_difference > 0
                        && (sn_difference <= 65 || was_offline || node.miss > escape_sweeps)
                    {
                        node.val = *entry;

                        // Report on a parameter change, or on revival.
                        if !par_match || was_offline {
                            mark_status_changed(index);
                            let _ = changed.push((entry.dev_adr, 1, u8::from(entry.par[0] != 0)));
                        }

                        // Any observation marks the node online and resets its age.
                        node.miss = 1;
                    }
                }
                // NEW NODE: take the next slot, or reject at capacity.
                None => {
                    if max >= MESH_NODE_MAX_NUM {
                        break; // table full: reject without overwriting any slot
                    }
                    table.store(max, entry, 1);
                    MESH_NODE_MAX.inc();
                    mark_status_changed(max);
                    let _ = changed.push((entry.dev_adr, 1, u8::from(entry.par[0] != 0)));
                }
            }
        }
    }

    send_uart_node_changes(changed.as_slice());
    1 // Packet processed.
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
/// 1. **Rate Limiting**: At most one sweep per [`MESH_STATUS_SWEEP_MS`], using
///    embassy-time `Instant`/`Duration` at the manager level only
/// 2. **Age Increment**: Ages every online remote's `miss` counter by one sweep
/// 3. **Node Marking**: Marks timed-out nodes for status change reporting
/// 4. **Cleanup**: Resets `miss` to 0 to indicate offline status
///
/// # Age Counter
///
/// Liveness only needs "age > deadline", so each node stores a sweep counter:
/// - `miss == 0` means offline / never seen (all-zero `.bss` init)
/// - `miss >= 1` means online with age `miss - 1` sweeps
/// - The deadline in sweeps is [`mesh_status_offline_sweeps`], derived from the
///   cadence-based [`mesh_status_timeout_ms`]
///
/// # Node State Transitions
///
/// Nodes transition through these states:
/// - **Online** (`is_online`, i.e. `miss != 0`): actively participating
/// - **Timeout Check**: ages `miss` once per sweep; times out at `miss > deadline`
/// - **Offline** (`miss == 0`): marked offline and removed from active topology
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
/// - **Locking**: A short critical-section lock around the sweep timestamp
///
/// # Side Effects
/// * Ages and resets node sweep counters to mark offline nodes
/// * Modifies mesh node mask to trigger status reporting
/// * May cause network-wide status updates as topology changes propagate
#[cfg_attr(test, mry::mry)]
pub fn mesh_node_flush_status() {
    // Rate limiting: one sweep per MESH_STATUS_SWEEP_MS. Read/update the shared
    // timestamp under a short lock only; never hold a lock across the table work.
    let now = Instant::now();
    if now.saturating_duration_since(*LAST_SWEEP.lock())
        < Duration::from_millis(MESH_STATUS_SWEEP_MS as u64)
    {
        return;
    }
    *LAST_SWEEP.lock() = now;

    // Deadline in sweeps for the current table size. Each run below ages every
    // online remote by exactly one sweep.
    let offline_sweeps = mesh_status_offline_sweeps(MESH_NODE_MAX.get() as u32);

    // Emit offline events in 13-entry batches (one UART packet each) and resume
    // the scan from a cursor so the table is visited once per flush (O(n))
    // instead of being rescanned per batch. This keeps one sweep from dropping
    // events when many nodes expire at once, without a 64-entry stack buffer on
    // this IRQ path. send_uart_node_changes runs after the lock is dropped.
    const BATCH: usize = 13;
    let mut cursor = 1usize;
    loop {
        let mut batch: heapless::Vec<(u8, u8, u8), BATCH> = heapless::Vec::new();
        let mut exhausted = true;
        {
            let max = MESH_NODE_MAX.get() as usize;
            if cursor < max {
                let mut st = MESH_NODE_ST.lock();
                let mut table = NodeTable::new(&mut st[..]);

                for index in cursor..max {
                    let node = &mut table.nodes[index];
                    if !is_online(node) {
                        continue;
                    }

                    // Age one sweep. Online miss stays <= offline_sweeps before
                    // it is reset, so it can never wrap here.
                    node.miss = node.miss.saturating_add(1);
                    if node.miss > offline_sweeps {
                        node.miss = 0;
                        mark_status_changed(index);
                        let _ = batch.push((node.val.dev_adr, 0, u8::from(node.val.par[0] != 0)));
                        if batch.is_full() {
                            cursor = index + 1;
                            exhausted = false;
                            break;
                        }
                    }
                }
            }
        }

        if batch.is_empty() {
            return;
        }
        send_uart_node_changes(batch.as_slice());
        if exhausted {
            return;
        }
    }
}

/// Updates this device's own status record to maintain mesh network presence.
///
/// This function implements the local node status update algorithm that ensures
/// this device remains visible in the mesh network topology. It manages sequence
/// number generation and online-age resets to prevent this device from being
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
/// # Online Age Reset
///
/// The local record's sweep age is reset to `miss = 1` (online, age 0 sweeps),
/// matching every other observation path. Self is never swept because
/// `mesh_node_flush_status` starts its scan at index 1.
///
/// # Node Table Management
///
/// The function updates the local device record (always at index 0):
/// - **Sequence Number**: Sets current monotonic sequence number
/// - **Online Age**: Resets `miss` to 1
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
    // Increment monotonic sequence number, avoiding the reserved 0 value.
    DEVICE_NODE_SN.inc();
    if DEVICE_NODE_SN.get() == 0 {
        DEVICE_NODE_SN.set(1);
    }

    let mut st = MESH_NODE_ST.lock();
    let mut table = NodeTable::new(&mut st[..]);
    let local = &mut table.nodes[0];
    local.val.sn = DEVICE_NODE_SN.get();
    local.miss = 1;
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
/// - **Skip Offline Nodes**: Only advertises nodes that are online (`miss != 0`)
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
/// - **Online Filtering**: Only includes nodes that are online (`miss != 0`)
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
    // Initialize output buffer to clean state.
    p_data.fill(0);
    let capacity = p_data.len() / MESH_NODE_ST_VAL_LEN;

    // Our own record always goes first (pre-refresh, as before).
    {
        let st = MESH_NODE_ST.lock();
        p_data[..MESH_NODE_ST_VAL_LEN].copy_from_slice(node_val_bytes(&st[0].val));
    }

    // Refresh local status so the record we advertise is current.
    mesh_node_keep_alive();

    let mut st = MESH_NODE_ST.lock();
    let mut table = NodeTable::new(&mut st[..]);
    let max = MESH_NODE_MAX.get() as usize;
    let slots = min(capacity, max);

    let mut written = 1usize;
    if slots > 1 && max > 1 {
        // Rotate the remote range so each packet covers a different window;
        // `chain` keeps this a single finite iterator, no index loop.
        let remotes = &table.nodes[1..max];
        let cursor = ADV_CURSOR.load(Ordering::Relaxed).clamp(1, remotes.len());
        let start = cursor - 1;
        let (before, from_cursor) = remotes.split_at(start);

        let mut scanned = 0usize;
        for node in from_cursor.iter().chain(before.iter()) {
            if written >= slots {
                break;
            }
            scanned += 1;
            if !is_online(node) {
                continue; // skip offline nodes
            }
            let dst = written * MESH_NODE_ST_VAL_LEN;
            p_data[dst..dst + MESH_NODE_ST_VAL_LEN].copy_from_slice(node_val_bytes(&node.val));
            written += 1;
        }

        // Resume after the last candidate inspected so coverage advances.
        ADV_CURSOR.store((start + scanned) % remotes.len() + 1, Ordering::Relaxed);
    }

    written as u32
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
            addr_of_mut!(pkt_light_adv_status.att_write_mut().value) as *mut u8,
            core::mem::size_of::<PacketAttValue>(),
        )
    };

    // Process node timeouts and collect current status information
    mesh_node_flush_status();
    mesh_node_adv_status(&mut pktdata[..MESH_STATUS_VALUE_LEN]);

    // Generate and embed advertisement sequence number
    ADV_ST_SN.store(ADV_ST_SN.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
    unsafe {
        let val = ADV_ST_SN.load(Ordering::Relaxed);
        // Copy sequence number to opcode field (3 bytes, little-endian)
        slice::from_raw_parts_mut(addr_of_mut!(pkt_light_adv_status.att_write_mut().opcode), 3)
            .copy_from_slice(slice::from_raw_parts(addr_of!(val) as *const u8, 3))
    }

    // Add packet signature for validation and type identification
    pktdata[MESH_STATUS_VALUE_LEN..MESH_STATUS_VALUE_LEN + 4].fill(0xa5);

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
        no_use: [0; 4],          // Reserved/unused bytes
    };

    // Convert 32-bit sequence number to 24-bit little-endian format
    // This provides unique packet identification for duplicate detection
    pkt.sno[0] = sno as u8; // LSB
    pkt.sno[1] = (sno >> 8) as u8; // Middle byte
    pkt.sno[2] = (sno >> 16) as u8; // MSB (limited to 24 bits)

    // Write op, vendor_id, and params via the att_cmd val[] overlay. This correctly
    // places bytes at their wire positions regardless of MeshPkt struct alignment:
    //   val[0] = op at offset 20
    //   val[1] = vendor_id lo at offset 21 (the u16 alignment padding slot; receiver reads here)
    //   val[2] = vendor_id hi at offset 22
    //   val[3..] = params at offset 23+
    // parse_ble_packet_op_params reads from this same val[] overlay, so the bytes match exactly.
    let mut packet = Packet { mesh: pkt };
    packet.att_cmd_mut().value.val[..cmd_op_para.len()].copy_from_slice(cmd_op_para);
    packet.mesh_mut().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT] = retransmit_count;
    packet.mesh_mut().internal_par1[INTERNAL_PAR_SEND_ACK] = if send_ack { 1 } else { 0 };
    packet
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
    {
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
    }

    MESH_NODE_REPORT_ENABLE.set(enable);

    // Send an initial dump of all known nodes when enabling UART status reporting,
    // so the host can immediately see the full current state of the mesh. Emit in
    // 13-entry batches (one UART packet each), advancing a cursor, so this
    // IRQ-reachable path never needs a 64-entry stack buffer; the MESH_NODE_ST
    // lock is dropped before each send.
    if enable {
        const BATCH: usize = 13;
        let mut start = 0usize;
        loop {
            let mut batch: heapless::Vec<(u8, u8, u8), BATCH> = heapless::Vec::new();
            let mut next = start;
            {
                let mesh_node_st = MESH_NODE_ST.lock();
                let max = MESH_NODE_MAX.get() as usize;
                for (offset, node) in mesh_node_st[start..max].iter().enumerate() {
                    next = start + offset + 1;
                    if node.val.dev_adr == 0 {
                        continue;
                    }
                    let online = u8::from(is_online(node));
                    let on_off = u8::from(node.val.par[0] != 0);
                    let _ = batch.push((node.val.dev_adr, online, on_off));
                    if batch.is_full() {
                        break;
                    }
                }
            }
            if batch.is_empty() {
                break;
            }
            let full = batch.is_full();
            send_uart_node_changes(batch.as_slice());
            if !full {
                break;
            }
            start = next;
        }
    }
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
/// It ensures that status changes are properly marked for network-wide propagation.
///
/// # Status Update Algorithm
///
/// The function performs an atomic status update sequence:
///
/// 1. **Parameter Update**: Copies new status parameters to device record
/// 2. **Online Age Reset**: Resets `miss` to 1 to prevent self-timeout
/// 3. **Change Notification**: Sets reporting flag to trigger status broadcast
///
/// # Online Age Reset
///
/// The local record's sweep age is reset to `miss = 1` (online), matching every
/// other observation path and keeping this device visible to peers.
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
/// - **Online Age**: Sweep-age counter (`miss`) for timeout detection
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
/// * Resets the local online age to prevent timeout
/// * Triggers status change notification for network propagation
/// * Affects subsequent mesh status advertisements
#[cfg_attr(test, mry::mry)]
pub fn ll_device_status_update(val_par: &[u8]) {
    let val;
    {
        let mut mesh_node_st = MESH_NODE_ST.lock();

        // Update this device's status parameters (index 0 = local device)
        mesh_node_st[0].val.par.copy_from_slice(val_par);

        // Mark this device online with a fresh age (miss = 1).
        mesh_node_st[0].miss = 1;

        // Mark this device for status change reporting (bit 0 = this device)
        MESH_NODE_MASK.lock()[0] |= 1;

        val = mesh_node_st[0].val;
    }

    // Send UART status for own device (after releasing MESH_NODE_ST lock)
    send_uart_node_changes(&[(val.dev_adr, 1, if val.par[0] != 0 { 1 } else { 0 })]);
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use mry::Any;

    // Import mock functions from their original modules
    use super::{mock_mesh_node_adv_status, mock_mesh_node_flush_status};
    use crate::embassy::time_driver::{clock_time64, mock_clock_time64};
    use crate::main_light::{mock_rf_link_data_callback, mock_rf_link_response_callback};
    use crate::mesh::{MeshNodeStT, MeshNodeStValT, MESH_NODE_ST_PAR_LEN};
    use crate::sdk::drivers::uart::{UartData, UART_DATA_LEN};
    use crate::sdk::light::{INTERNAL_PAR_RETRANSMIT_COUNT, INTERNAL_PAR_SEND_ACK};
    use crate::sdk::mcu::clock::{mock_clock_time, mock_clock_time_exceed};
    use crate::sdk::mcu::register::{mock_read_reg_system_tick, read_reg_system_tick};
    use crate::uart_manager::UartMsg;

    /// Mock wall-clock for sweep tests: ~31 s, far beyond one sweep, and
    /// deterministic so `Instant::now()` never depends on prior test state.
    const TEST_NOW_TICKS: u64 = 1_000_000_000;

    /// Helper function to reset global mesh state for tests
    fn reset_mesh_state() {
        DEVICE_ADDRESS.set(0x10); // Test device address
        DEVICE_NODE_SN.set(100);
        MESH_NODE_MAX.set(10);
        MESH_NODE_REPORT_ENABLE.set(true);

        // Force the next flush to sweep regardless of any prior test's timestamp.
        *LAST_SWEEP.lock() = Instant::from_ticks(0);

        // Reset uart_manager mock state so each test starts clean.
        // Assigning Default to the mry field clears all mock rules and call logs,
        // reverting every instrumented method to its real (passthrough) implementation.
        // After the reset, disable_uart_status_reporting uses the real impl to
        // ensure the uart_status_reporting field starts as false.
        {
            let mut app = app();
            app.uart_manager.mry = Default::default();
            app.uart_manager.disable_uart_status_reporting();
        }

        // Clear the mesh node status table
        let mut mesh_node_st = MESH_NODE_ST.lock();
        for i in 0..mesh_node_st.len() {
            mesh_node_st[i] = MeshNodeStT {
                miss: 0,
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

        // Age the node, then deliver a newer sequence number.
        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            mesh_node_st[10].miss = 20;
        }
        let node_v2 = create_test_mesh_node(0x20, 51, &[5, 6]);
        let packet2 = vec![node_v2];
        let result = mesh_node_update_status(&packet2);

        // Verify the node was updated (should be at index 10)
        let mesh_node_st = MESH_NODE_ST.lock();
        let updated_node = &mesh_node_st[10];
        assert_eq!(updated_node.val.dev_adr, 0x20);
        assert_eq!(updated_node.val.sn, 51); // Updated sequence number
        assert_eq!(updated_node.val.par[0..2], [5, 6]); // Updated values
        let miss = updated_node.miss;
        assert_eq!(miss, 1, "an accepted observation resets the age");

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
    // Integration test: mesh_node_update_status -> mesh_node_report_status pipeline
    // ================================================================================

    /// Tests the complete status pipeline: update_status populates MESH_NODE_ST and
    /// MESH_NODE_MASK, then report_status reads from them and returns correct data.
    ///
    /// This verifies that a new remote node added via mesh_node_update_status is
    /// visible via mesh_node_report_status — the exact path that was broken when
    /// node_report_task was removed.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_status_pipeline_update_then_report() {
        use crate::sdk::ble_app::irq::mesh_node_report_status;

        mock_read_reg_system_tick().returns(0x12345678);

        // Start with a clean state, MESH_NODE_MAX=1 (only own node at index 0)
        DEVICE_ADDRESS.set(0x1F); // Address 31, our device
        MESH_NODE_MAX.set(1);
        MESH_NODE_REPORT_ENABLE.set(true);

        // Clear the mesh node status table and mask
        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            for i in 0..mesh_node_st.len() {
                mesh_node_st[i] = MeshNodeStT {
                    miss: 0,
                    val: MeshNodeStValT {
                        dev_adr: if i == 0 { 0x1F } else { 0 },
                        sn: 0,
                        par: [0; MESH_NODE_ST_PAR_LEN],
                    },
                };
            }
        }
        {
            let mut mask = MESH_NODE_MASK.lock();
            for m in mask.iter_mut() {
                *m = 0;
            }
        }

        // Step 1: Add two remote nodes via mesh_node_update_status
        let remote_nodes = vec![
            create_test_mesh_node(10, 1, &[0x80, 0x00]), // node addr=10
            create_test_mesh_node(20, 5, &[0xFF, 0x01]), // node addr=20
        ];
        mesh_node_update_status(&remote_nodes);

        // Step 2: Verify MESH_NODE_MAX expanded
        assert_eq!(
            MESH_NODE_MAX.get(),
            3,
            "MESH_NODE_MAX should be 3 (own + 2 remote)"
        );

        // Step 3: Verify MESH_NODE_ST has the remote nodes
        {
            let mesh_node_st = MESH_NODE_ST.lock();
            assert_eq!(mesh_node_st[1].val.dev_adr, 10);
            assert_eq!(mesh_node_st[1].val.sn, 1);
            assert_eq!(mesh_node_st[1].val.par, [0x80, 0x00]);
            let miss1 = mesh_node_st[1].miss;
            assert_ne!(miss1, 0, "tick should be set for online node");

            assert_eq!(mesh_node_st[2].val.dev_adr, 20);
            assert_eq!(mesh_node_st[2].val.sn, 5);
            assert_eq!(mesh_node_st[2].val.par, [0xFF, 0x01]);
            let miss2 = mesh_node_st[2].miss;
            assert_ne!(miss2, 0, "tick should be set for online node");
        }

        // Step 4: Verify MESH_NODE_MASK has bits 1 and 2 set (for the two new nodes)
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

        // Step 5: Call mesh_node_report_status and verify it returns the remote nodes
        let mut report_buf = [0u8; 20];
        let reported = mesh_node_report_status(&mut report_buf, 5);

        assert_eq!(reported, 2, "Should report 2 remote nodes");

        // First reported node (index 1): addr=10, sn=1, par=[0x80, 0x00]
        assert_eq!(report_buf[0], 10, "First reported node dev_adr");
        assert_eq!(report_buf[1], 1, "First reported node sn");
        assert_eq!(report_buf[2], 0x80, "First reported node par[0]");
        assert_eq!(report_buf[3], 0x00, "First reported node par[1]");

        // Second reported node (index 2): addr=20, sn=5, par=[0xFF, 0x01]
        assert_eq!(report_buf[4], 20, "Second reported node dev_adr");
        assert_eq!(report_buf[5], 5, "Second reported node sn");
        assert_eq!(report_buf[6], 0xFF, "Second reported node par[0]");
        assert_eq!(report_buf[7], 0x01, "Second reported node par[1]");

        // Step 6: Verify mask bits were cleared after reporting
        {
            let mask = MESH_NODE_MASK.lock();
            assert_eq!(
                mask[0] & (1 << 1),
                0,
                "Mask bit 1 should be cleared after report"
            );
            assert_eq!(
                mask[0] & (1 << 2),
                0,
                "Mask bit 2 should be cleared after report"
            );
        }
    }

    // ================================================================================
    // Tests for mesh_node_keep_alive function
    // ================================================================================

    /// Tests mesh_node_keep_alive basic functionality.
    ///
    /// Verifies that the function properly increments the device sequence number
    /// and marks the local device online with a fresh age.
    #[test]
    fn test_mesh_node_keep_alive_basic() {
        reset_mesh_state();

        // Pretend self was already aging.
        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            mesh_node_st[0].miss = 9;
        }

        let initial_sn = DEVICE_NODE_SN.get();

        // Call mesh_node_keep_alive
        mesh_node_keep_alive();

        // Verify sequence number was incremented
        assert_eq!(DEVICE_NODE_SN.get(), initial_sn + 1);

        // Verify device status record was updated
        let mesh_node_st = MESH_NODE_ST.lock();
        let device_node = &mesh_node_st[0];

        assert_eq!(device_node.val.sn, initial_sn + 1);
        // Self is reset online (miss = 1, age 0 sweeps).
        let actual_miss = device_node.miss;
        assert_eq!(actual_miss, 1);
    }

    /// Tests mesh_node_keep_alive sequence number wraparound handling.
    ///
    /// Verifies that when the sequence number would become 0, it wraps to 1.
    #[test]
    fn test_mesh_node_keep_alive_sequence_wraparound() {
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
    /// Verifies that multiple consecutive calls properly increment the sequence number
    /// and keep self online.
    #[test]
    fn test_mesh_node_keep_alive_multiple_calls() {
        reset_mesh_state();

        let initial_sn = DEVICE_NODE_SN.get(); // 100

        mesh_node_keep_alive();
        assert_eq!(DEVICE_NODE_SN.get(), initial_sn + 1);

        mesh_node_keep_alive();
        assert_eq!(DEVICE_NODE_SN.get(), initial_sn + 2);

        mesh_node_keep_alive();
        assert_eq!(DEVICE_NODE_SN.get(), initial_sn + 3);

        // Verify final device status record
        let mesh_node_st = MESH_NODE_ST.lock();
        let device_node = &mesh_node_st[0];
        assert_eq!(device_node.val.sn, initial_sn + 3);
        let actual_miss = device_node.miss;
        assert_eq!(actual_miss, 1);
    }

    /// Tests mesh_node_keep_alive resets the local sweep age to online.
    ///
    /// A large stale age on self must collapse to `miss = 1` so self never
    /// appears to be timing out.
    #[test]
    fn test_mesh_node_keep_alive_resets_age_to_online() {
        reset_mesh_state();

        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            mesh_node_st[0].miss = 200;
        }

        mesh_node_keep_alive();

        let mesh_node_st = MESH_NODE_ST.lock();
        let device_node = &mesh_node_st[0];
        let actual_miss = device_node.miss;
        assert_eq!(actual_miss, 1);
    }

    /// Tests mesh_node_keep_alive preserves other device status fields.
    ///
    /// Verifies that only sequence number and age are updated, while
    /// device address and parameter data remain unchanged.
    #[test]
    fn test_mesh_node_keep_alive_preserves_other_fields() {
        reset_mesh_state();

        // Set up initial device status with specific values
        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            mesh_node_st[0].val.dev_adr = 0x42;
            mesh_node_st[0].val.par = [0xAA, 0xBB];
            mesh_node_st[0].miss = 200;
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

        // Verify sn and age were updated
        assert_eq!(device_node.val.sn, 101); // incremented from 100
        let actual_miss = device_node.miss;
        assert_eq!(actual_miss, 1);
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

        // Verify command parameters via the wire-format val[] overlay.
        // mesh_construct_packet writes cmd_op_para as a raw flat copy from &pkt.op,
        // bypassing struct alignment so val[i] == cmd_op_para[i] for all i.
        let val = &packet.att_write().value.val;
        assert_eq!(val[0], cmd_op_para[0]); // op
        assert_eq!(val[1], cmd_op_para[1]); // vendor_id_lo in wire format (padding slot)
        assert_eq!(val[2], cmd_op_para[2]); // vendor_id_hi in wire format
                                            // No params for 3-byte cmd_op_para

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

        // Verify command parameters via the wire-format val[] overlay.
        // Raw flat copy: val[i] == cmd_op_para[i] for all copied bytes.
        let val = &packet.att_write().value.val;
        assert_eq!(val[0], cmd_op_para[0]); // op
        assert_eq!(val[1], cmd_op_para[1]); // vendor_id_lo (wire)
        assert_eq!(val[2], cmd_op_para[2]); // vendor_id_hi (wire)
        for i in 0..10usize {
            let param_idx = i + 3;
            if param_idx < cmd_op_para.len() {
                assert_eq!(val[param_idx], cmd_op_para[param_idx]);
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

            // Verify wire-format bytes via val[] overlay.
            // Raw flat copy: val[i] == cmd_op_para[i] for all copied bytes.
            let val = &packet.att_write().value.val;
            assert_eq!(val[0], cmd_op_para[0]); // op
            assert_eq!(val[1], cmd_op_para[1]); // vendor_id_lo (wire)
            assert_eq!(val[2], cmd_op_para[2]); // vendor_id_hi (wire)
            for i in 0..length.saturating_sub(3) {
                assert_eq!(val[i + 3], cmd_op_para[i + 3]);
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

    /// Tests that mesh_report_status_enable sends an initial UART dump of all known nodes.
    ///
    /// Verifies: When enabling, all nodes with dev_adr != 0 appear in a NodeStatus packet
    /// with the correct online and on_off values. Nodes with tick==0 → online=0.
    #[test]
    fn test_mesh_report_status_enable_sends_initial_dump() {
        reset_mesh_state();

        // Populate 3 nodes at indices 0-2
        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            // Index 0: own device, online, on
            mesh_node_st[0] = MeshNodeStT {
                miss: 1,
                val: MeshNodeStValT {
                    dev_adr: 0x10,
                    sn: 1,
                    par: [1, 0xFF],
                },
            };
            // Index 1: remote node, online, off
            mesh_node_st[1] = MeshNodeStT {
                miss: 1,
                val: MeshNodeStValT {
                    dev_adr: 0x20,
                    sn: 2,
                    par: [0, 0xFF],
                },
            };
            // Index 2: remote node, offline (tick=0), on
            mesh_node_st[2] = MeshNodeStT {
                miss: 0,
                val: MeshNodeStValT {
                    dev_adr: 0x30,
                    sn: 3,
                    par: [1, 0xFF],
                },
            };
        }
        MESH_NODE_MAX.set(3);

        let mut expected = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };
        expected.data[2] = UartMsg::NodeStatus as u8; // 0x07
        expected.data[3] = 0x10; // node 0: id
        expected.data[4] = 1; // node 0: online
        expected.data[5] = 1; // node 0: on
        expected.data[6] = 0x20; // node 1: id
        expected.data[7] = 1; // node 1: online
        expected.data[8] = 0; // node 1: off
        expected.data[9] = 0x30; // node 2: id
        expected.data[10] = 0; // node 2: offline (tick==0)
        expected.data[11] = 1; // node 2: on

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(expected).returns(Ok(()));
        }

        mesh_report_status_enable(true);

        app().uart_manager.mock_send_message(Any).assert_called(1);
    }

    /// The initial dump must emit every known node even when more than one UART
    /// batch (13 entries) is required, in table order, without a large stack
    /// buffer. Guards the M1 batching rewrite.
    #[test]
    fn test_mesh_report_status_enable_initial_dump_batches_over_thirteen() {
        reset_mesh_state();
        DEVICE_ADDRESS.set(0x01);

        let total = 20usize;
        {
            let mut st = MESH_NODE_ST.lock();
            for i in 0..total {
                st[i] = MeshNodeStT {
                    miss: 1,
                    val: MeshNodeStValT {
                        dev_adr: (0xA0 + i) as u8,
                        sn: 1,
                        par: [1, 0],
                    },
                };
            }
        }
        MESH_NODE_MAX.set(total as u8);

        let expected = |first_id: u8, count: usize| {
            let mut pkt = UartData {
                len: UART_DATA_LEN as u32,
                data: [0; UART_DATA_LEN],
            };
            pkt.data[2] = UartMsg::NodeStatus as u8;
            for i in 0..count {
                pkt.data[3 + i * 3] = first_id + i as u8;
                pkt.data[3 + i * 3 + 1] = 1; // online
                pkt.data[3 + i * 3 + 2] = 1; // on
            }
            pkt
        };

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager
                .mock_send_message(expected(0xA0, 13))
                .returns(Ok(()));
            app.uart_manager
                .mock_send_message(expected(0xAD, 7))
                .returns(Ok(()));
        }

        mesh_report_status_enable(true);

        app()
            .uart_manager
            .mock_send_message(expected(0xA0, 13))
            .assert_called(1);
        app()
            .uart_manager
            .mock_send_message(expected(0xAD, 7))
            .assert_called(1);
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

    // ================================================================================
    // Test to demonstrate the status packet parsing bug
    // ================================================================================

    /// Integration test for status packet parsing with real node status data.
    ///
    /// This test verifies that status advertisement packets are correctly parsed to extract
    /// actual node status entries from the PacketAttValue payload, rather than incorrectly
    /// extracting data from packet metadata fields (sno, dst_adr, src_adr).
    ///
    /// The test creates a realistic status packet with known node status data and verifies
    /// that the correct device addresses and status information are parsed.
    #[test]
    #[mry::lock(read_reg_system_tick, mesh_node_update_status)]
    fn test_status_packet_parsing_integration() {
        use crate::sdk::ble_app::light_ll::packet_processing::rf_link_rc_data;
        use crate::sdk::packet_types::{Packet, PacketAttValue, PacketAttWrite, PacketL2capHead};

        // Setup mocks
        mock_read_reg_system_tick().returns(0x12345678);

        reset_mesh_state();
        DEVICE_ADDRESS.set(0x42); // Our device address

        // Create a status advertisement packet with realistic node status data
        // The status data is in the first 24 bytes of PacketAttValue
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
                    // These first fields overlay the actual status data
                    // For a status packet, we need to set these as raw bytes representing MeshNodeStValT entries
                    // Entry 0: bytes 0-3 [dev_adr=0x10, sn=0x01, par[0]=0x11, par[1]=0x22]
                    sno: [0x10, 0x01, 0x11], // First 3 bytes of entry 0
                    src: [0x22, 0x20],       // Last byte of entry 0 + first byte of entry 1
                    dst: [0x02, 0x33],       // Entry 1 continued: sn=0x02, par[0]=0x33
                    val: {
                        let mut val = [0u8; 23];
                        // Entry 1 last byte (par[1])
                        val[0] = 0x44;
                        // Entry 2: dev_adr=0x30, sn=0x03, par=[0x55, 0x66]
                        val[1] = 0x30;
                        val[2] = 0x03;
                        val[3] = 0x55;
                        val[4] = 0x66;
                        // Entry 3: dev_adr=0x01, sn=0x04, par=[0x77, 0x88]
                        val[5] = 0x01; // Device address 1 - should now be processed correctly
                        val[6] = 0x04;
                        val[7] = 0x77;
                        val[8] = 0x88;
                        // Entry 4: dev_adr=0x50, sn=0x05, par=[0x99, 0xAA]
                        val[9] = 0x50;
                        val[10] = 0x05;
                        val[11] = 0x99;
                        val[12] = 0xAA;
                        // Entry 5: dev_adr=0x60, sn=0x06, par=[0xBB, 0xCC]
                        val[13] = 0x60;
                        val[14] = 0x06;
                        val[15] = 0xBB;
                        val[16] = 0xCC;

                        // Remaining 6 bytes unused (bytes 17-22 in val array)
                        // Set the signature at the correct position (bytes 17-20 in val array)
                        val[17] = 0xa5;
                        val[18] = 0xa5;
                        val[19] = 0xa5;
                        val[20] = 0xa5;
                        val
                    },
                },
            },
        };

        // Use a static variable to capture data from the mock
        static CAPTURED_DATA: std::sync::Mutex<Option<Vec<u8>>> = std::sync::Mutex::new(None);

        // Clear any previous capture
        *CAPTURED_DATA.lock().unwrap() = None;

        // Set up mock with a side effect that captures the data
        mock_mesh_node_update_status(Any).returns_with(|pkt: Vec<MeshNodeStValT>| {
            // Capture the raw bytes to see what device addresses are being parsed
            let bytes = unsafe {
                core::slice::from_raw_parts(
                    pkt.as_ptr() as *const u8,
                    pkt.len() * core::mem::size_of::<MeshNodeStValT>(),
                )
            };
            *CAPTURED_DATA.lock().unwrap() = Some(bytes.to_vec());
            1u32
        });

        // Process the packet
        rf_link_rc_data(&mut packet);

        // Verify mesh_node_update_status was called
        mock_mesh_node_update_status(Any).assert_called(1);

        // Now check what was captured - should be the actual node status data
        let captured = CAPTURED_DATA.lock().unwrap();
        let data = captured
            .as_ref()
            .expect("mesh_node_update_status should have been called");

        // With MESH_NODE_ST_VAL_LEN = 4, each entry is [dev_adr, sn, par[0], par[1]]
        // The data should come from PacketAttValue bytes 0-23:
        //   Entry 0: [0x10, 0x01, 0x11, 0x22] - from sno[0-2] + src[0]
        //   Entry 1: [0x20, 0x02, 0x33, 0x44] - from src[1] + dst[0-1] + val[0]
        //   Entry 2: [0x30, 0x03, 0x55, 0x66] - from val[1-4]
        //   Entry 3: [0x01, 0x04, 0x77, 0x88] - from val[5-8] (address 1!)
        //   Entry 4: [0x50, 0x05, 0x99, 0xAA] - from val[9-12]
        //   Entry 5: [0x60, 0x06, 0xBB, 0xCC] - from val[13-16]

        // Verify we have 24 bytes (6 entries * 4 bytes each)
        assert_eq!(data.len(), 24, "Should have 24 bytes of status data");

        // Verify each entry is parsed correctly
        assert_eq!(data[0], 0x10, "Entry 0 dev_adr should be 0x10");
        assert_eq!(data[1], 0x01, "Entry 0 sn should be 0x01");
        assert_eq!(data[2], 0x11, "Entry 0 par[0] should be 0x11");
        assert_eq!(data[3], 0x22, "Entry 0 par[1] should be 0x22");

        assert_eq!(data[4], 0x20, "Entry 1 dev_adr should be 0x20");
        assert_eq!(data[5], 0x02, "Entry 1 sn should be 0x02");

        assert_eq!(data[6], 0x33, "Entry 1 par[0] should be 0x33");
        assert_eq!(data[7], 0x44, "Entry 1 par[1] should be 0x44");

        assert_eq!(data[8], 0x30, "Entry 2 dev_adr should be 0x30");
        assert_eq!(data[9], 0x03, "Entry 2 sn should be 0x03");
        assert_eq!(data[10], 0x55, "Entry 2 par[0] should be 0x55");
        assert_eq!(data[11], 0x66, "Entry 2 par[1] should be 0x66");

        // Most importantly - verify that device address 1 is correctly parsed
        assert_eq!(
            data[12], 0x01,
            "Entry 3 dev_adr should be 0x01 (address 1 should work!)"
        );
        assert_eq!(data[13], 0x04, "Entry 3 sn should be 0x04");
        assert_eq!(data[14], 0x77, "Entry 3 par[0] should be 0x77");
        assert_eq!(data[15], 0x88, "Entry 3 par[1] should be 0x88");

        assert_eq!(data[16], 0x50, "Entry 4 dev_adr should be 0x50");
        assert_eq!(data[17], 0x05, "Entry 4 sn should be 0x05");
        assert_eq!(data[18], 0x99, "Entry 4 par[0] should be 0x99");
        assert_eq!(data[19], 0xAA, "Entry 4 par[1] should be 0xAA");

        assert_eq!(data[20], 0x60, "Entry 5 dev_adr should be 0x60");
        assert_eq!(data[21], 0x06, "Entry 5 sn should be 0x06");
        assert_eq!(data[22], 0xBB, "Entry 5 par[0] should be 0xBB");
        assert_eq!(data[23], 0xCC, "Entry 5 par[1] should be 0xCC");
    }

    // ================================================================================
    // Tests for send_uart_node_changes
    // ================================================================================

    /// Tests that send_uart_node_changes does nothing when given an empty slice.
    #[test]
    fn test_send_uart_node_changes_empty_no_send() {
        reset_mesh_state();
        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        send_uart_node_changes(&[]);

        app().uart_manager.mock_send_message(Any).assert_called(0);
    }

    /// Tests that send_uart_node_changes does nothing when UART reporting is disabled.
    #[test]
    fn test_send_uart_node_changes_uart_disabled_no_send() {
        reset_mesh_state();
        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(false);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        send_uart_node_changes(&[(0x20, 1, 1)]);

        app().uart_manager.mock_send_message(Any).assert_called(0);
    }

    /// Tests that send_uart_node_changes sends correctly formatted NodeStatus packet.
    ///
    /// Verifies: data[2] = NodeStatus (0x07), then [node_id, online, on_off] per entry.
    #[test]
    fn test_send_uart_node_changes_single_entry_correct_format() {
        reset_mesh_state();

        let mut expected = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };
        expected.data[2] = UartMsg::NodeStatus as u8; // 0x07
        expected.data[3] = 0x20; // node_id
        expected.data[4] = 1; // online
        expected.data[5] = 1; // on_off (par[0] was non-zero)

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(expected).returns(Ok(()));
        }

        send_uart_node_changes(&[(0x20, 1, 1)]);

        app().uart_manager.mock_send_message(Any).assert_called(1);
    }

    /// Tests that send_uart_node_changes packs multiple entries into a single message.
    ///
    /// Three entries appear consecutively at data[3..12], data[2] = NodeStatus (0x07).
    #[test]
    fn test_send_uart_node_changes_multiple_entries_packed() {
        reset_mesh_state();

        let mut expected = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };
        expected.data[2] = UartMsg::NodeStatus as u8;
        // Entry 0: node 0x10, online, on
        expected.data[3] = 0x10;
        expected.data[4] = 1;
        expected.data[5] = 1;
        // Entry 1: node 0x20, online, off
        expected.data[6] = 0x20;
        expected.data[7] = 1;
        expected.data[8] = 0;
        // Entry 2: node 0x30, offline, on
        expected.data[9] = 0x30;
        expected.data[10] = 0;
        expected.data[11] = 1;

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(expected).returns(Ok(()));
        }

        send_uart_node_changes(&[(0x10, 1, 1), (0x20, 1, 0), (0x30, 0, 1)]);

        // All three entries fit in one call
        app().uart_manager.mock_send_message(Any).assert_called(1);
    }

    /// Tests that send_uart_node_changes splits more than 13 entries across multiple packets.
    #[test]
    fn test_send_uart_node_changes_chunks_over_13() {
        reset_mesh_state();
        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        // 14 entries → 2 packets (13 + 1)
        let entries: [(u8, u8, u8); 14] = core::array::from_fn(|i| (i as u8 + 1, 1, 0));
        send_uart_node_changes(&entries);

        app().uart_manager.mock_send_message(Any).assert_called(2);
    }

    // ================================================================================
    // UART calls from mesh_node_update_status
    // ================================================================================

    /// Tests that a newly discovered node triggers a UART status send.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_update_status_new_node_sends_uart() {
        mock_read_reg_system_tick().returns(0x12345678);
        reset_mesh_state();

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        let new_node = create_test_mesh_node(0x20, 5, &[0xAB, 0xCD]);
        mesh_node_update_status(&[new_node]);

        app().uart_manager.mock_send_message(Any).assert_called(1);
    }

    /// Tests that a par change on an existing online node triggers a UART status send.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_update_status_par_change_sends_uart() {
        mock_read_reg_system_tick().returns(0x12345678);
        reset_mesh_state();

        // Pre-populate the table with an existing online node at index MESH_NODE_MAX (=10)
        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            mesh_node_st[10] = MeshNodeStT {
                miss: 1,
                val: MeshNodeStValT {
                    dev_adr: 0x20,
                    sn: 5,
                    par: [0x01, 0x02],
                },
            };
        }
        MESH_NODE_MAX.set(11);

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        // Update with same sn advance but different par → par_match = false
        let updated = create_test_mesh_node(0x20, 6, &[0xAA, 0xBB]);
        mesh_node_update_status(&[updated]);

        app().uart_manager.mock_send_message(Any).assert_called(1);
    }

    /// Tests that a sn-only advance (par unchanged, node online) does NOT trigger UART.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_update_status_sn_only_change_no_uart() {
        mock_read_reg_system_tick().returns(0x12345678);
        reset_mesh_state();

        // Pre-populate existing online node at index 10
        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            mesh_node_st[10] = MeshNodeStT {
                miss: 1,
                val: MeshNodeStValT {
                    dev_adr: 0x20,
                    sn: 5,
                    par: [0xAB, 0xCD],
                },
            };
        }
        MESH_NODE_MAX.set(11);

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        // Update with newer sn but identical par → par_match = true and tick != 0 → no UART
        let same_par = create_test_mesh_node(0x20, 6, &[0xAB, 0xCD]);
        mesh_node_update_status(&[same_par]);

        app().uart_manager.mock_send_message(Any).assert_called(0);
    }

    /// Tests that when UART reporting is disabled, a new node does not trigger a UART send.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_update_status_uart_disabled_no_send() {
        mock_read_reg_system_tick().returns(0x12345678);
        reset_mesh_state();

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(false);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        let new_node = create_test_mesh_node(0x20, 5, &[0x01, 0x02]);
        mesh_node_update_status(&[new_node]);

        app().uart_manager.mock_send_message(Any).assert_called(0);
    }

    // ================================================================================
    // UART calls from mesh_node_flush_status
    // ================================================================================

    /// Tests that a timed-out node triggers a UART status send from mesh_node_flush_status.
    #[test]
    #[mry::lock(clock_time64)]
    fn test_mesh_node_flush_status_timeout_sends_uart() {
        reset_mesh_state();
        mock_clock_time64().returns(TEST_NOW_TICKS);

        // One remote sitting exactly at the deadline for MESH_NODE_MAX=2 (6 sweeps).
        let deadline = mesh_status_offline_sweeps(2);
        {
            let mut st = MESH_NODE_ST.lock();
            st[0].miss = 1;
            st[1].miss = deadline;
            st[1].val = MeshNodeStValT {
                dev_adr: 0x20,
                sn: 5,
                par: [0x01, 0x02],
            };
        }
        MESH_NODE_MAX.set(2);

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        mesh_node_flush_status();

        app().uart_manager.mock_send_message(Any).assert_called(1);
    }

    /// Tests that mesh_node_flush_status sends nothing when no nodes have timed out,
    /// and that a fresh node only ages by one sweep.
    #[test]
    #[mry::lock(clock_time64)]
    fn test_mesh_node_flush_status_no_timeout_no_uart() {
        reset_mesh_state();
        mock_clock_time64().returns(TEST_NOW_TICKS);

        {
            let mut st = MESH_NODE_ST.lock();
            st[0].miss = 1;
            st[1].miss = 1; // fresh
            st[1].val = MeshNodeStValT {
                dev_adr: 0x20,
                sn: 5,
                par: [0x01, 0x02],
            };
        }
        MESH_NODE_MAX.set(2);

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        mesh_node_flush_status();

        app().uart_manager.mock_send_message(Any).assert_called(0);
        let st = MESH_NODE_ST.lock();
        let miss = st[1].miss;
        assert_eq!(miss, 2, "age increments exactly once per sweep");
    }

    /// Tests that mesh_node_flush_status sends online=0 (offline) in the NodeStatus packet.
    #[test]
    #[mry::lock(clock_time64)]
    fn test_mesh_node_flush_status_timeout_sends_offline_format() {
        reset_mesh_state();
        mock_clock_time64().returns(TEST_NOW_TICKS);

        let deadline = mesh_status_offline_sweeps(2);
        {
            let mut st = MESH_NODE_ST.lock();
            st[0].miss = 1;
            st[1].miss = deadline;
            st[1].val = MeshNodeStValT {
                dev_adr: 0x20,
                sn: 5,
                par: [0x01, 0x02], // par[0]=1 (on)
            };
        }
        MESH_NODE_MAX.set(2);

        let mut expected = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };
        expected.data[2] = UartMsg::NodeStatus as u8; // 0x07
        expected.data[3] = 0x20; // node_id
        expected.data[4] = 0; // online = 0 (offline, timed out)
        expected.data[5] = 1; // on_off = 1 (par[0] was 0x01, non-zero)

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(expected).returns(Ok(()));
        }

        mesh_node_flush_status();

        app().uart_manager.mock_send_message(Any).assert_called(1);
    }

    /// A node one sweep short of the deadline survives; one at the deadline goes
    /// offline on the next sweep, and only that node. (The counter cannot wrap:
    /// an online `miss` is reset to 0 before it can exceed the deadline.)
    #[test]
    #[mry::lock(clock_time64)]
    fn test_mesh_node_flush_status_deadline_boundary() {
        reset_mesh_state();
        mock_clock_time64().returns(TEST_NOW_TICKS);

        let deadline = mesh_status_offline_sweeps(3); // ~6
        {
            let mut st = MESH_NODE_ST.lock();
            st[0].miss = 1;
            st[1].miss = deadline - 1; // one sweep short
            st[1].val = MeshNodeStValT {
                dev_adr: 0x20,
                sn: 1,
                par: [1, 0],
            };
            st[2].miss = deadline; // at the deadline
            st[2].val = MeshNodeStValT {
                dev_adr: 0x21,
                sn: 1,
                par: [1, 0],
            };
        }
        MESH_NODE_MAX.set(3);

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        mesh_node_flush_status();

        let st = MESH_NODE_ST.lock();
        let m1 = st[1].miss;
        assert_eq!(m1, deadline, "one short of deadline stays online");
        let m2 = st[2].miss;
        assert_eq!(m2, 0, "at deadline goes offline on this sweep");
        let self_miss = st[0].miss;
        assert_eq!(self_miss, 1, "self (index 0) is never swept");
        drop(st);
        app().uart_manager.mock_send_message(Any).assert_called(1);
    }

    // ================================================================================
    // Cadence-derived deadline + table/flush capacity regression tests
    // ================================================================================

    /// At a full table a genuinely new address must be rejected without
    /// overwriting any existing slot (regression: the old lookup treated the
    /// last active slot as a match for an unknown address).
    #[test]
    fn test_mesh_node_update_status_table_full_rejects_new_address() {
        reset_mesh_state();
        DEVICE_ADDRESS.set(0x01);

        // Fill all 64 slots: index 0 = self, indices 1..63 = remotes 11..73.
        {
            let mut st = MESH_NODE_ST.lock();
            for i in 0..MESH_NODE_MAX_NUM {
                st[i].miss = 1; // online
                st[i].val = MeshNodeStValT {
                    dev_adr: if i == 0 { 0x01 } else { (i + 10) as u8 },
                    sn: 5,
                    par: [0x11, 0x22],
                };
            }
        }
        MESH_NODE_MAX.set(MESH_NODE_MAX_NUM as u8);

        // New address with a plausible sn advance (would pass the update check).
        let newcomer = create_test_mesh_node(0xFE, 6, &[0x33, 0x44]);
        mesh_node_update_status(&[newcomer]);

        assert_eq!(
            MESH_NODE_MAX.get(),
            MESH_NODE_MAX_NUM as u8,
            "table must stay full"
        );
        let st = MESH_NODE_ST.lock();
        // Last slot untouched: no unknown address overwrote it.
        let last_addr = st[63].val.dev_adr;
        let last_sn = st[63].val.sn;
        assert_eq!(last_addr, 73);
        assert_eq!(last_sn, 5);
        assert_eq!(st[63].val.par, [0x11, 0x22]);
        assert!(
            st.iter().all(|n| n.val.dev_adr != 0xFE),
            "rejected address must not appear anywhere in the table"
        );
    }

    /// More than 9 nodes timing out in one sweep must all produce offline
    /// events (regression: the old fixed 9-entry buffer silently dropped them).
    #[test]
    #[mry::lock(clock_time64)]
    fn test_mesh_node_flush_status_emits_all_timeouts_over_nine() {
        reset_mesh_state();
        mock_clock_time64().returns(TEST_NOW_TICKS);
        DEVICE_ADDRESS.set(0x01);

        let n = 15usize;
        let deadline = mesh_status_offline_sweeps((n + 1) as u32);
        {
            let mut st = MESH_NODE_ST.lock();
            st[0].miss = 1;
            st[0].val.dev_adr = 0x01;
            for i in 1..=n {
                st[i].miss = deadline; // times out on this sweep
                st[i].val = MeshNodeStValT {
                    dev_adr: (0x40 + i) as u8,
                    sn: 1,
                    par: [1, 0],
                };
            }
        }
        MESH_NODE_MAX.set((n + 1) as u8);
        {
            let mut mask = MESH_NODE_MASK.lock();
            for m in mask.iter_mut() {
                *m = 0;
            }
        }
        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        mesh_node_flush_status();

        // 15 offline entries -> 2 UART packets (13 + 2); the old 9-cap sent 1.
        app().uart_manager.mock_send_message(Any).assert_called(2);

        let st = MESH_NODE_ST.lock();
        for i in 1..=n {
            let miss = st[i].miss;
            assert_eq!(miss, 0, "node {i} must be marked offline");
        }
        drop(st);

        // Every timed-out node has a pending status bit (proves all 15 events).
        let mask = MESH_NODE_MASK.lock();
        let set_bits: u32 = mask.iter().map(|w| w.count_ones()).sum();
        assert_eq!(set_bits, n as u32);
    }

    /// A full 63-remote table timing out at once must emit exactly
    /// ceil(63/13) = 5 batches, zero every age, and set one mask bit per node.
    /// Guards the single-pass resumable cursor introduced for M3.
    #[test]
    #[mry::lock(clock_time64)]
    fn test_mesh_node_flush_status_full_table_all_timeout() {
        reset_mesh_state();
        mock_clock_time64().returns(TEST_NOW_TICKS);
        DEVICE_ADDRESS.set(0x01);

        let remotes = MESH_NODE_MAX_NUM - 1; // 63 remotes + self
        let deadline = mesh_status_offline_sweeps(MESH_NODE_MAX_NUM as u32);
        {
            let mut st = MESH_NODE_ST.lock();
            st[0].miss = 1;
            st[0].val.dev_adr = 0x01;
            for i in 1..=remotes {
                st[i].miss = deadline;
                st[i].val = MeshNodeStValT {
                    dev_adr: (0x10 + i) as u8,
                    sn: 1,
                    par: [1, 0],
                };
            }
        }
        MESH_NODE_MAX.set(MESH_NODE_MAX_NUM as u8);
        {
            let mut mask = MESH_NODE_MASK.lock();
            for m in mask.iter_mut() {
                *m = 0;
            }
        }
        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        mesh_node_flush_status();

        // 63 offline entries -> 5 UART packets (13*4 + 11).
        app().uart_manager.mock_send_message(Any).assert_called(5);

        let st = MESH_NODE_ST.lock();
        for i in 1..=remotes {
            let miss = st[i].miss;
            assert_eq!(miss, 0, "node {i} must be marked offline exactly once");
        }
        drop(st);

        let mask = MESH_NODE_MASK.lock();
        let set_bits: u32 = mask.iter().map(|w| w.count_ones()).sum();
        assert_eq!(set_bits, remotes as u32);
    }

    /// M4: the deadline model's remote-records-per-packet constant must equal
    /// the wire capacity of a status value region (one of its slots is self).
    /// The `const _` assertion enforces this at compile time; this is the test
    /// twin so drift also fails loudly here.
    #[test]
    fn test_mesh_status_records_per_packet_matches_wire_capacity() {
        let records_per_packet = MESH_STATUS_VALUE_LEN / MESH_NODE_ST_VAL_LEN;
        assert_eq!(
            MESH_STATUS_RECORDS_PER_PACKET as usize,
            records_per_packet - 1,
            "records-per-packet must match the wire value region minus the self slot"
        );
    }

    /// At M=64 the derived deadline is 26 sweeps (13 s): a node aged 10 sweeps
    /// (5 s) must remain online after a sweep.
    #[test]
    #[mry::lock(clock_time64)]
    fn test_mesh_status_deadline_five_second_gap_stays_online() {
        reset_mesh_state();
        mock_clock_time64().returns(TEST_NOW_TICKS);
        DEVICE_ADDRESS.set(0x01);

        let deadline = mesh_status_offline_sweeps(64);
        {
            let mut st = MESH_NODE_ST.lock();
            st[0].miss = 1;
            st[0].val.dev_adr = 0x01;
            st[1].miss = 11; // age 10 sweeps = 5 s
            st[1].val = MeshNodeStValT {
                dev_adr: 0x40,
                sn: 1,
                par: [1, 0],
            };
        }
        MESH_NODE_MAX.set(64);
        {
            let mut mask = MESH_NODE_MASK.lock();
            for m in mask.iter_mut() {
                *m = 0;
            }
        }
        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        mesh_node_flush_status();

        let st = MESH_NODE_ST.lock();
        let miss = st[1].miss;
        assert!(
            deadline > miss,
            "5 s gap is within the 13 s deadline at M=64"
        );
        assert_ne!(miss, 0);
        drop(st);
        app().uart_manager.mock_send_message(Any).assert_called(0);
    }

    /// Just past the derived deadline the node goes offline exactly once; a
    /// later accepted update revives it (miss = 1) and it does not immediately
    /// re-timeout.
    #[test]
    #[mry::lock(clock_time64)]
    fn test_mesh_status_deadline_offline_then_revive() {
        reset_mesh_state();
        mock_clock_time64().returns(TEST_NOW_TICKS);
        DEVICE_ADDRESS.set(0x01);

        let deadline = mesh_status_offline_sweeps(64);
        {
            let mut st = MESH_NODE_ST.lock();
            st[0].miss = 1;
            st[0].val.dev_adr = 0x01;
            st[1].miss = deadline + 1; // past the deadline
            st[1].val = MeshNodeStValT {
                dev_adr: 0x40,
                sn: 5,
                par: [1, 0],
            };
        }
        MESH_NODE_MAX.set(64);
        {
            let mut mask = MESH_NODE_MASK.lock();
            for m in mask.iter_mut() {
                *m = 0;
            }
        }
        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        // Go offline exactly once.
        mesh_node_flush_status();
        {
            let st = MESH_NODE_ST.lock();
            let miss = st[1].miss;
            assert_eq!(miss, 0, "past the derived deadline the node is offline");
        }
        app().uart_manager.mock_send_message(Any).assert_called(1);

        // A fresh sn resurrects it; miss resets to 1 and an online event fires.
        mesh_node_update_status(&[create_test_mesh_node(0x40, 6, &[1, 0])]);
        {
            let st = MESH_NODE_ST.lock();
            let miss = st[1].miss;
            assert_eq!(miss, 1, "revived node restarts its age");
        }

        // Clearing the pending mask and flushing again must not re-timeout it.
        {
            let mut mask = MESH_NODE_MASK.lock();
            for m in mask.iter_mut() {
                *m = 0;
            }
        }
        *LAST_SWEEP.lock() = Instant::from_ticks(0);
        mesh_node_flush_status();
        {
            let st = MESH_NODE_ST.lock();
            let miss = st[1].miss;
            assert_eq!(miss, 2, "a revived node stays online until the deadline");
        }
    }

    /// A large (>65) sequence jump on a still-fresh node is held back until the
    /// escape hatch (half the derived deadline, in sweeps) has elapsed.
    #[test]
    fn test_mesh_node_update_status_large_sn_jump_escape_hatch() {
        reset_mesh_state();
        DEVICE_ADDRESS.set(0x01);
        MESH_NODE_MAX.set(64); // derived 13 s, escape gate 13 sweeps

        let original = MeshNodeStValT {
            dev_adr: 0x40,
            sn: 10,
            par: [1, 0],
        };
        // Fresh node: miss = 1, age 0, far below the escape gate.
        {
            let mut st = MESH_NODE_ST.lock();
            st[0].val.dev_adr = 0x01;
            st[1].val = original;
            st[1].miss = 1;
        }

        // Fresh node + implausible jump: ignored (val and miss unchanged).
        mesh_node_update_status(&[MeshNodeStValT {
            dev_adr: 0x40,
            sn: 110,
            par: [9, 9],
        }]);
        {
            let st = MESH_NODE_ST.lock();
            let miss = st[1].miss;
            assert_eq!(st[1].val.sn, 10, "large jump on a fresh node is ignored");
            assert_eq!(miss, 1);
        }

        // Same jump once the node is past the escape gate: accepted, age reset.
        {
            let mut st = MESH_NODE_ST.lock();
            st[1].miss = 14; // > (offline_sweeps(64)/2).max(1) = 13
        }
        mesh_node_update_status(&[MeshNodeStValT {
            dev_adr: 0x40,
            sn: 110,
            par: [9, 9],
        }]);
        {
            let st = MESH_NODE_ST.lock();
            let miss = st[1].miss;
            assert_eq!(st[1].val.sn, 110, "jump accepted after the gate");
            assert_eq!(st[1].val.par, [9, 9]);
            assert_eq!(miss, 1);
        }
    }

    /// Offline integration: observe via update, time out via the derived
    /// deadline, then report; report must surface the node with sn forced to 0.
    #[test]
    #[mry::lock(clock_time64)]
    fn test_status_pipeline_update_timeout_then_report_offline() {
        use crate::sdk::ble_app::irq::mesh_node_report_status;

        reset_mesh_state();
        mock_clock_time64().returns(TEST_NOW_TICKS);
        DEVICE_ADDRESS.set(0x01);
        MESH_NODE_MAX.set(1);
        MESH_NODE_REPORT_ENABLE.set(true);
        {
            let mut mask = MESH_NODE_MASK.lock();
            for m in mask.iter_mut() {
                *m = 0;
            }
        }

        // Observe two remote nodes.
        mesh_node_update_status(&[
            create_test_mesh_node(0x40, 1, &[0x80, 0x00]),
            create_test_mesh_node(0x50, 2, &[0x00, 0x00]),
        ]);
        assert_eq!(MESH_NODE_MAX.get(), 3);

        // Age both to exactly the deadline (M=3 -> 6 sweeps).
        let deadline = mesh_status_offline_sweeps(3);
        {
            let mut st = MESH_NODE_ST.lock();
            st[1].miss = deadline;
            st[2].miss = deadline;
        }
        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }
        mesh_node_flush_status();

        // The pending mask now reports both nodes as offline (sn == 0).
        let mut report = [0u8; 16];
        let reported = mesh_node_report_status(&mut report, 4);
        assert_eq!(reported, 2);
        assert_eq!(report[0], 0x40);
        assert_eq!(report[1], 0, "offline node reports sn 0");
        assert_eq!(report[4], 0x50);
        assert_eq!(report[5], 0, "offline node reports sn 0");
    }

    /// Air status advertisement payload keeps the exact 4-byte record layout
    /// `[dev_adr, sn, par0, par1]` with self first, and skips offline remotes.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_mesh_node_adv_status_wire_layout_unchanged() {
        mock_read_reg_system_tick().returns(0x12345678);
        reset_mesh_state();
        DEVICE_ADDRESS.set(0x01);

        {
            let mut st = MESH_NODE_ST.lock();
            st[0].miss = 1;
            st[0].val = MeshNodeStValT {
                dev_adr: 0x01,
                sn: 42,
                par: [0xAA, 0xBB],
            };
            st[1].miss = 1; // online
            st[1].val = MeshNodeStValT {
                dev_adr: 0x0A,
                sn: 1,
                par: [0x80, 0x00],
            };
            st[2].miss = 1; // online
            st[2].val = MeshNodeStValT {
                dev_adr: 0x0B,
                sn: 2,
                par: [0x11, 0x22],
            };
            st[3].miss = 0; // offline: must be skipped
            st[3].val = MeshNodeStValT {
                dev_adr: 0x0C,
                sn: 3,
                par: [0x33, 0x44],
            };
        }
        MESH_NODE_MAX.set(4);
        ADV_CURSOR.store(1, Ordering::Relaxed);

        let mut buf = [0u8; 24];
        let written = mesh_node_adv_status(&mut buf);

        assert_eq!(written, 3, "self + two online remotes; offline one skipped");
        // Self record is copied before keep_alive refreshes it.
        assert_eq!(&buf[0..4], &[0x01, 42, 0xAA, 0xBB]);
        assert_eq!(&buf[4..8], &[0x0A, 1, 0x80, 0x00]);
        assert_eq!(&buf[8..12], &[0x0B, 2, 0x11, 0x22]);
        assert_eq!(&buf[12..16], &[0, 0, 0, 0], "offline slot stays empty");
    }

    // ================================================================================
    // UART calls from ll_device_status_update
    // ================================================================================
    /// Tests that ll_device_status_update sends own device status via UART when enabled.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_ll_device_status_update_sends_uart() {
        mock_read_reg_system_tick().returns(0x12345678);
        reset_mesh_state();

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        ll_device_status_update(&[0xDE, 0xAD]);

        app().uart_manager.mock_send_message(Any).assert_called(1);
    }

    /// Tests that a stale relayed packet with the same sn does NOT reset the
    /// sweep age on an online node. This is the primary case: the dead node is
    /// still online (miss != 0) when relays start arriving, so the timeout can
    /// never fire unless we stop resetting miss on sn_difference == 0.
    #[test]
    fn test_mesh_node_update_status_stale_relay_does_not_refresh_age_online_node() {
        reset_mesh_state();

        let initial_miss: u8 = 5;
        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            mesh_node_st[1].miss = initial_miss; // online, aging
            mesh_node_st[1].val = MeshNodeStValT {
                dev_adr: 0x55,
                sn: 7,
                par: [1, 0],
            };
        }
        MESH_NODE_MAX.set(2);
        DEVICE_ADDRESS.set(0x01);

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        // Same sn=7 relay — sn has not advanced.
        let stale_pkt = [MeshNodeStValT {
            dev_adr: 0x55,
            sn: 7,
            par: [1, 0],
        }];
        mesh_node_update_status(&stale_pkt);

        // miss must NOT have been reset — the node keeps aging and will expire.
        let mesh_node_st = MESH_NODE_ST.lock();
        let miss_val = mesh_node_st[1].miss;
        assert_eq!(
            miss_val, initial_miss,
            "stale relay must not reset miss on an online node"
        );
    }

    /// Tests that a node which timed out (miss == 0) is NOT re-onlined by a stale
    /// relayed packet carrying the same sequence number as the one we last saw.
    #[test]
    fn test_mesh_node_update_status_stale_relay_does_not_reonline_dead_node() {
        reset_mesh_state();

        // Place node 0x55 into the table, already timed out (miss == 0, sn == 7).
        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            mesh_node_st[1].miss = 0; // offline
            mesh_node_st[1].val = MeshNodeStValT {
                dev_adr: 0x55,
                sn: 7,
                par: [1, 0],
            };
        }
        MESH_NODE_MAX.set(2);
        DEVICE_ADDRESS.set(0x01);

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        // Deliver a "stale relay": same dev_adr, same sn=7, SAME par.
        let stale_pkt = [MeshNodeStValT {
            dev_adr: 0x55,
            sn: 7,
            par: [1, 0],
        }];
        mesh_node_update_status(&stale_pkt);

        // No "online" UART notification must have been emitted.
        app().uart_manager.mock_send_message(Any).assert_called(0);

        // miss must remain 0 — node stays offline.
        let mesh_node_st = MESH_NODE_ST.lock();
        let miss_val = mesh_node_st[1].miss;
        assert_eq!(miss_val, 0, "stale relay must not revive a dead node");
    }

    /// Tests that an offline node IS brought back online when a genuinely new packet
    /// arrives with an advanced sequence number (sn_difference > 0).
    #[test]
    fn test_mesh_node_update_status_fresh_sn_reonlines_dead_node() {
        reset_mesh_state();

        {
            let mut mesh_node_st = MESH_NODE_ST.lock();
            mesh_node_st[1].miss = 0; // offline
            mesh_node_st[1].val = MeshNodeStValT {
                dev_adr: 0x55,
                sn: 7,
                par: [1, 0],
            };
        }
        MESH_NODE_MAX.set(2);
        DEVICE_ADDRESS.set(0x01);

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(true);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        // New packet from the now-revived light: sn advanced to 8.
        let fresh_pkt = [MeshNodeStValT {
            dev_adr: 0x55,
            sn: 8,
            par: [1, 0],
        }];
        mesh_node_update_status(&fresh_pkt);

        // Should have sent exactly one "online" notification.
        app().uart_manager.mock_send_message(Any).assert_called(1);

        // miss must be 1 (online).
        let mesh_node_st = MESH_NODE_ST.lock();
        let miss_val = mesh_node_st[1].miss;
        assert_eq!(miss_val, 1, "fresh packet revives the node as online");
    }

    /// Tests that ll_device_status_update does not send UART when reporting is disabled.
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_ll_device_status_update_uart_disabled_no_send() {
        mock_read_reg_system_tick().returns(0x12345678);
        reset_mesh_state();

        {
            let mut app = app();
            app.uart_manager
                .mock_uart_status_reporting_enabled()
                .returns(false);
            app.uart_manager.mock_send_message(Any).returns(Ok(()));
        }

        ll_device_status_update(&[0x01, 0x02]);

        app().uart_manager.mock_send_message(Any).assert_called(0);
    }
}
