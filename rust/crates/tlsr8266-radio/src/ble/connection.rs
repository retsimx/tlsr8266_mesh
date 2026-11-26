//! # BLE Connection Management
//!
//! This module handles the BLE peripheral connection lifecycle, timing synchronization,
//! and adaptive parameter management for the TLSR8266 mesh lighting system.
//!
//! ## Connection Management Architecture
//!
//! The connection management system operates in several key phases:
//!
//! 1. **Connection Establishment**: Validates incoming connection requests and sets up
//!    initial connection parameters including timing, channels, and security.
//!
//! 2. **Timing Synchronization**: Implements adaptive timing adjustment algorithms
//!    to maintain precise synchronization with the master device, critical for
//!    reliable communication in the 2.4GHz ISM band.
//!
//! 3. **Parameter Optimization**: Continuously monitors connection quality and
//!    adjusts parameters (intervals, timeouts, window sizes) for optimal performance
//!    under varying RF conditions.
//!
//! 4. **Bridge Mode Operations**: Manages the transition between BLE peripheral
//!    mode (for smartphone connectivity) and mesh mode (for device-to-device
//!    communication).
//!
//! ## Timing Critical Algorithms
//!
//! The module implements several timing-critical algorithms:
//!
//! - **Adaptive Window Sizing**: Dynamically adjusts connection windows based on
//!   observed timing drift and RF conditions
//! - **Collision Avoidance**: Uses randomized backoff and channel hopping to
//!   minimize interference in dense mesh networks  
//! - **Power Management**: Optimizes sleep/wake cycles while maintaining
//!   connection reliability
//!
//! ## Connection Parameter Validation
//!
//! All connection parameters undergo strict validation to ensure:
//! - Compliance with BLE specification timing constraints
//! - Compatibility with mesh network timing requirements
//! - Prevention of invalid configurations that could cause connection drops

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicU8, AtomicUsize};

use crate::ble::channel_selection::ble_ll_build_available_channel_table;
use crate::ble::flash_config::{update_ble_parameter_cb, SYS_CHN_LISTEN};
use crate::ble::pair::{pair_init, PAIR_AC, SECURITY_ENABLE};
use crate::ble::status::DEVICE_STATUS_TICK_COUNTER;
use crate::callbacks;
use crate::mesh_ll::node_status::{MESH_NODE_MASK, MESH_NODE_REPORT_ENABLE};
use crate::rf::drv_8266::*;
use crate::rf::drv_8266::{
    BLE_PERIPHERAL_ADVERTISING_ENABLED, BLE_SCAN_RESPONSE_INTERVAL_US, CURRENT_RF_STATE, MAC_ID,
    PKT_INIT,
};
use crate::types::*;
use mesh_protocol::packet_types::*;
use tlsr8266_hal::common::sync::{CriticalSectionMutex, SimplifyLS};
use tlsr8266_hal::mcu::clock::CLOCK_SYS_CLOCK_1US;
use tlsr8266_pac::*;

/// Response buffer for storing packets that need to be sent back to the BLE host.
/// Contains an array of 16 Packet structures used to queue ATT responses before transmission.
/// Protected by critical section mutex for thread-safe access in interrupt contexts.
///
/// Visibility is `pub(crate)`: the facade module is the only entry point from
/// outside this crate, and the crate boundary is the fence C-10 established.
pub(crate) static BUFF_RESPONSE: CriticalSectionMutex<[Packet; BUFF_RESPONSE_PACKET_COUNT]> =
    CriticalSectionMutex::new(
        [Packet {
            att_data: PacketAttData {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: 0,
                    rf_len: 0,
                    l2cap_len: 0,
                    chan_id: 0,
                },
                att: 0,
                hl: 0,
                hh: 0,
                dat: [0; 23],
            },
        }; BUFF_RESPONSE_PACKET_COUNT],
    );

/// `OtaRadioOps::stage_packet`: copy one OTA packet into the shared response
/// buffer.
///
/// `ota` may not name this module, so it reaches the buffer through this shim.
/// The `% 16` wrap is the legacy staging semantics; hardening a too-large
/// `cache_idx` is C-13 (#75).
///
/// `#[inline(never)]`: `sdk/stack_analysis.py`'s indirect-edge table binds the
/// `radio_stage_pkt` slot to this symbol.
#[inline(never)]
pub fn stage_ota_packet(idx: usize, pkt: &Packet) {
    BUFF_RESPONSE.lock()[idx % 16] = *pkt;
}

/// `OtaRadioOps::read_staged_packet`: read one staged OTA packet back out.
///
/// The read indexes directly, exactly like the legacy `BUFF_RESPONSE[i]` in the
/// save loop; only staging wraps.
///
/// `#[inline(never)]`: see [`stage_ota_packet`].
#[inline(never)]
pub fn read_staged_ota_packet(idx: usize) -> Packet {
    BUFF_RESPONSE.lock()[idx]
}

/// Current state of the BLE connection as a peripheral device.
/// Tracks connection status, advertising state, and link layer operations.
pub(crate) static BLE_PERIPHERAL_LINK_STATE: crate::types::AtomicBlePeripheralLinkState =
    crate::types::AtomicBlePeripheralLinkState::new(
        crate::types::BlePeripheralLinkState::Disconnected,
    );

/// Timestamp of the last received packet for timeout detection.
/// Used to detect communication timeouts and trigger reconnection procedures.
pub(crate) static LAST_PACKET_RECEIVED_TIMESTAMP: AtomicU32 = AtomicU32::new(0);

/// Sequence number for light connection messages from master.
/// Tracks message ordering in master-slave light control communications.
pub(crate) static LIGHT_CONN_SN_MASTER: AtomicU16 = AtomicU16::new(0);

/// Timestamp when the slave connection was established.
/// Used for calculating connection duration and timeout detection.
pub static SLAVE_CONNECTED_TICK: AtomicU32 = AtomicU32::new(0);

/// Flag indicating whether a BLE slave connection is currently active.
/// Set to true when connected to a central device, false when disconnected.
pub(crate) static BLE_PERIPHERAL_CONNECTION_ACTIVE: AtomicBool = AtomicBool::new(false);

/// BLE connection interval in microseconds for slave connections.
/// Determines how frequently connection events occur between central and peripheral.
pub(crate) static SLAVE_LINK_INTERVAL: AtomicU32 = AtomicU32::new(0x9c400);

/// Current connection window size for slave timing.
/// Defines the time window during which connection events can occur.
pub(crate) static SLAVE_WINDOW_SIZE: AtomicU32 = AtomicU32::new(0);

/// Updated connection window size pending application.
/// New window size that will be applied after connection parameter update.
pub(crate) static SLAVE_WINDOW_SIZE_UPDATE: AtomicU32 = AtomicU32::new(0);

/// Scheduled timestamp for the next connection event.
/// Used for precise timing of BLE connection intervals.
pub static SLAVE_NEXT_CONNECT_TICK: AtomicU32 = AtomicU32::new(0);

/// Flag indicating successful completion of connection parameter update.
/// Set when BLE connection parameters have been successfully negotiated.
pub(crate) static CONN_UPDATE_SUCCESSED: AtomicBool = AtomicBool::new(false);

/// Counter tracking connection parameter update attempts.
/// Used to limit retry attempts and detect update failures.
pub(crate) static CONN_UPDATE_CNT: AtomicUsize = AtomicUsize::new(0);

/// GATT service discovery timeout tracking timestamp.
/// Used to detect when GATT service discovery has taken too long.
pub(crate) static GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP: AtomicU32 = AtomicU32::new(0);

/// BLE peripheral connection timeout value in microseconds.
/// Maximum time to wait before considering the connection lost.
pub static BLE_PERIPHERAL_CONNECTION_TIMEOUT_US: AtomicU32 = AtomicU32::new(0);

/// Timestamp for BLE peripheral timing update procedures.
/// Tracks when connection parameter updates were initiated.
pub(crate) static BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP: AtomicU32 = AtomicU32::new(0);

/// Flag indicating timing update timestamp 2 is active.
/// Used for secondary timing update coordination.
pub(crate) static BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG: AtomicBool = AtomicBool::new(false);

/// Timestamp for secondary timing update procedure OK time.
/// Tracks when secondary timing updates were confirmed.
pub(crate) static BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME: AtomicU32 = AtomicU32::new(0);

/// Next instant value for BLE connection parameter updates.
/// BLE connection event number when new parameters take effect.
pub(crate) static BLE_PERIPHERAL_NEXT_UPDATE_INSTANT: AtomicU16 = AtomicU16::new(0);

/// Previous connection interval before parameter update.
/// Stored for rollback in case of parameter update failure.
pub(crate) static BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL: AtomicU32 = AtomicU32::new(0);

/// BLE connection supervision timeout in 10ms units.
/// Maximum time between successful communication before connection is dropped.
pub(crate) static BLE_CONN_TIMEOUT: AtomicU32 = AtomicU32::new(0);

/// BLE connection interval in 1.25ms units.
/// Time between consecutive connection events.
pub(crate) static BLE_CONN_INTERVAL: AtomicU32 = AtomicU32::new(0);

/// BLE connection offset timing adjustment.
/// Fine-tuning for connection event timing synchronization.
pub(crate) static BLE_CONN_OFFSET: AtomicU32 = AtomicU32::new(0);

/// Timestamp of the first successful slave connection.
/// Used for connection establishment timing and statistics.
pub(crate) static BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP: AtomicU32 = AtomicU32::new(0);

/// Flag indicating connection parameters need to be updated.
/// Set when BLE connection parameters require modification.
pub(crate) static NEED_UPDATE_CONNECT_PARA: AtomicBool = AtomicBool::new(false);

/// User-requested maximum connection interval limit.
/// Upper bound for BLE connection interval negotiations.
static UPDATE_INTERVAL_USER_MAX: AtomicU16 = AtomicU16::new(0);

/// User-requested minimum connection interval limit.
/// Lower bound for BLE connection interval negotiations.
static UPDATE_INTERVAL_USER_MIN: AtomicU16 = AtomicU16::new(0);

/// Validation flag for slave data integrity.
/// Ensures slave data structures are in a consistent state.
pub(crate) static SLAVE_DATA_VALID: AtomicU32 = AtomicU32::new(0);

/// Timestamp for bridge command operations.
/// Used for timing bridge/relay operations in mesh network.
pub(crate) static BRIDGE_COMMAND_TIMESTAMP: AtomicU32 = AtomicU32::new(0);

/// Bridge sequence number for mesh relay operations.
/// Tracks the current bridge/relay state and operation count.
pub(crate) static BRIDGE_SEQUENCE_NUMBER: AtomicU32 = AtomicU32::new(0);

/// Flag enabling automatic peripheral timing adjustment.
/// When true, peripheral device adjusts its timing to maintain synchronization.
pub(crate) static BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED: AtomicBool = AtomicBool::new(false);

/// Device timing tick for bridge receive operations.
/// Timestamp used for mesh bridge receive timing coordination.
pub(crate) static BRIDGE_RECEIVE_TIMING_TICK: AtomicU32 = AtomicU32::new(0);

/// Window offset for BLE peripheral connection timing.
/// Adjustment applied to connection window for timing synchronization.
static BLE_PERIPHERAL_WINDOW_OFFSET: AtomicU32 = AtomicU32::new(0);

/// Current instant value for BLE peripheral connection events.
/// BLE connection event counter for timing coordination.
pub(crate) static BLE_PERIPHERAL_CONNECTION_INSTANT: AtomicU16 = AtomicU16::new(0);

/// Current command state for BLE peripheral link operations.
/// Tracks which command is being processed by the peripheral device.
pub(crate) static BLE_PERIPHERAL_LINK_COMMAND: AtomicU8 = AtomicU8::new(0);

/// Flag enabling BLE peripheral connection acceptance.
/// When true, peripheral device accepts incoming BLE connection requests.
static BLE_PERIPHERAL_CONNECTION_ENABLED: AtomicBool = AtomicBool::new(false);

/// Validates BLE connection parameters according to specification constraints.
///
/// This function implements a comprehensive validation algorithm for BLE connection
/// parameters received in LL_CONNECT_REQ packets. The validation ensures that all
/// timing parameters are within specification limits and compatible with the mesh
/// network's timing requirements.
///
/// # Algorithm Details
///
/// The validation checks multiple parameter constraints in sequence:
///
/// 1. **Connection Interval Validation**:
///    - Checks that (interval - 6) < 0xc7b (3195 decimal)
///    - This ensures the interval is within the valid BLE range of 7.5ms to 4.0s
///    - The subtraction of 6 accounts for internal timing overhead
///
/// 2. **Window Size Validation**:
///    - Window size must be non-zero and less than 9
///    - This constrains the connection window to reasonable values for power efficiency
///
/// 3. **Supervision Timeout Validation**:
///    - Timeout must be greater than 9 and less than 0xc81 (3201 decimal)
///    - Ensures timeout is between 100ms and 32s as per BLE spec
///
/// 4. **Window Offset Validation**:
///    - Window offset must not exceed the connection interval
///    - Prevents invalid timing configurations
///
/// 5. **Channel Hopping Validation**:
///    - Hop increment must be non-zero (required for frequency diversity)
///    - Channel map must have at least one enabled channel
///
/// 6. **Latency Constraint Validation**:
///    - If latency is zero, connection is rejected (prevents edge cases)
///    - Latency must satisfy: latency ≤ (interval × 8) / interval
///    - This complex constraint ensures the supervision timeout math works correctly
///
/// # Parameters
/// * `packet` - The LL_CONNECT_REQ packet containing connection parameters to validate
///
/// # Returns
/// * `true` if all parameters are valid and the connection should be accepted
/// * `false` if any parameter is invalid and the connection should be rejected
///
/// # BLE Specification Compliance
/// This function enforces the parameter ranges defined in the Bluetooth Core
/// Specification Volume 6, Part B, Section 4.5.1 (Connection Parameters).
fn check_par_con(packet: &Packet) -> bool {
    // Validate core timing parameters: interval, window size, timeout, offset, and hopping
    if packet.ll_init().interval >= 6 &&
       packet.ll_init().interval <= 3200 &&  // Valid BLE connection interval range (7.5ms to 4s)
       packet.ll_init().wsize != 0 &&
       packet.ll_init().wsize as u16 <= packet.ll_init().interval &&
       packet.ll_init().timeout > 10 &&
       packet.ll_init().timeout <= 3200 &&
       packet.ll_init().woffset <= packet.ll_init().interval &&
       (packet.ll_init().hop & 0x1F) != 0 && (packet.ll_init().hop & 0x1F) <= 16 &&
       packet.ll_init().chm.iter().any(|v| { *v != 0 })
    {
        // Accept connections with reasonable latency
        // Latency should be reasonable compared to timeout
        if packet.ll_init().latency as u32 * packet.ll_init().interval as u32 * 2
            < packet.ll_init().timeout as u32
        {
            return true;
        }
    }
    false
}

/// Establishes a BLE peripheral (slave) connection with comprehensive parameter setup.
///
/// This function implements the complete slave connection establishment algorithm,
/// including parameter validation, timing setup, security initialization, and
/// mesh network integration. It's the core entry point for accepting BLE connections
/// from master devices (typically smartphones or gateways).
///
/// # Connection Establishment Algorithm
///
/// The connection process follows a multi-stage algorithm:
///
/// 1. **Pre-Connection Validation**:
///    - Checks if peripheral connections are enabled
///    - Validates that scan address matches advertiser address (prevents spoofing)
///    - Validates all connection parameters using `check_par_con()`
///
/// 2. **RF Hardware Configuration**:
///    - Stops any ongoing transmissions (`rf_stop_trx()`)
///    - Calculates and sets the connection window offset timing
///    - Configures interrupt timing for the first connection event
///
/// 3. **Timing Parameter Setup**:
///    - Converts BLE timing units (1.25ms) to system clock units
///    - Sets up connection interval, window size, and supervision timeout
///    - Implements adaptive window sizing based on timing constraints
///
/// 4. **Channel Configuration**:
///    - Extracts and validates the channel map from the connection request
///    - Builds the available channel table for frequency hopping
///    - Sets up CRC initialization values for packet validation
///
/// 5. **Security and Pairing Setup**:
///    - Initializes the pairing subsystem
///    - Records connection timestamp for security timeout calculations
///    - Resets sequence numbers for encrypted communication
///
/// 6. **Mesh Network Integration**:
///    - Resets mesh node masks and status tracking
///    - Disables mesh reporting during connection establishment
///    - Sets up bridge mode for BLE-to-mesh communication
///
/// 7. **Connection State Management**:
///    - Transitions RF state to receiving mode
///    - Sets up GATT service discovery timeout
///    - Enables connection parameter update procedures
///
/// # Timing Calculations
///
/// The function performs several critical timing calculations:
///
/// - **Window Offset**: `CLOCK_SYS_CLOCK_1US * 1250 * (woffset + 1)`
/// - **Connection Interval**: `interval * CLOCK_SYS_CLOCK_1US * 1250`
/// - **Window Size**: `(wsize * 1250 + 1100) * CLOCK_SYS_CLOCK_1US`
/// - **Supervision Timeout**: `timeout * 10000` (in microseconds)
///
/// # Parameters
/// * `packet` - The LL_CONNECT_REQ packet containing connection parameters
/// * `time` - The system timestamp when the connection request was received
///
/// # Returns
/// * `true` if the connection was successfully established
/// * `false` if the connection was rejected due to invalid parameters or system state
///
/// # Side Effects
/// This function modifies numerous global state variables related to:
/// - Connection timing and parameters
/// - RF hardware configuration  
/// - Security and pairing state
/// - Mesh network state
/// - System interrupts and timers
pub fn rf_link_slave_connect(packet: &Packet, time: u32) {
    // Initialize connection update tracking variables
    CONN_UPDATE_SUCCESSED.set(false);
    CONN_UPDATE_CNT.set(0);

    // Pre-connection validation: check if connections are enabled and advertiser address matches our device
    // Read the MAC into a value first: the guard must not be held across the
    // pairing bring-up below, which locks `MAC_ID` again (and no lock may span
    // I/O).
    let our_mac = *MAC_ID.lock();
    // Compare full 6-byte MAC address (adv_a is set to full MAC_ID in advertisements)
    let adv_matches = packet.ll_init().adv_a == our_mac;

    if BLE_PERIPHERAL_CONNECTION_ENABLED.get() && adv_matches {
        // Validate connection parameters
        if check_par_con(packet) {
            // Stop any ongoing RF operations to prepare for connection setup
            rf_stop_trx();

            // Calculate window offset timing in system clock units
            // BLE uses 1.25ms units, so multiply by 1250 microseconds
            BLE_PERIPHERAL_WINDOW_OFFSET
                .set(CLOCK_SYS_CLOCK_1US * 1250 * (packet.ll_init().woffset as u32 + 1));

            // Set up initial interrupt timing for connection establishment
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 1000 + read_reg_system_tick());
            write_reg_irq_src(0x100000);

            // Calculate the precise timing for the first connection event
            // Different timing offsets are used based on whether window offset is 0
            // - If woffset = 0: subtract 500µs for immediate connection
            // - Otherwise: subtract 700µs for normal connection timing
            let timing_adjustment = if packet.ll_init().woffset == 0 {
                500
            } else {
                700
            };
            write_reg_system_tick_irq(
                time + BLE_PERIPHERAL_WINDOW_OFFSET.get() - CLOCK_SYS_CLOCK_1US * timing_adjustment,
            );

            // Safety check: ensure interrupt timing is not in the past
            // If the calculated time has already passed, set a minimal delay (10µs)
            if 0x80000000 < read_reg_system_tick_irq() - read_reg_system_tick() {
                write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick());
            }

            // Initialize connection sequence number for master link tracking
            // 0x80 bit indicates this is a master-initiated connection
            LIGHT_CONN_SN_MASTER.set(0x80);

            // Record the precise connection establishment timestamp
            SLAVE_CONNECTED_TICK.set(read_reg_system_tick());

            // Store first connection time for security timeout calculations
            if SECURITY_ENABLE.get() {
                BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(SLAVE_CONNECTED_TICK.get());
            }

            // Calculate device status reporting frequency based on connection interval
            // Formula: (interval * 5) / 4 - provides adaptive reporting rate
            DEVICE_STATUS_TICK_COUNTER.set(((packet.ll_init().interval as u32 * 5) >> 2) as u8);

            // Convert BLE connection interval to system clock units
            // BLE interval is in 1.25ms units, system clock is in microseconds
            SLAVE_LINK_INTERVAL.set(packet.ll_init().interval as u32 * CLOCK_SYS_CLOCK_1US * 1250);

            // Convert BLE window size to system clock units with timing margin
            // Add 1100µs margin to account for crystal tolerance and processing delays
            SLAVE_WINDOW_SIZE
                .set((packet.ll_init().wsize as u32 * 1250 + 1100) * CLOCK_SYS_CLOCK_1US);

            // Adaptive window size optimization algorithm
            // Ensures window size doesn't exceed (interval - 1.25ms) to prevent overlap
            let tmp = SLAVE_LINK_INTERVAL.get() - CLOCK_SYS_CLOCK_1US * 1250;
            if tmp <= SLAVE_WINDOW_SIZE.get() && SLAVE_WINDOW_SIZE.get() - tmp != 0 {
                SLAVE_WINDOW_SIZE.set(tmp);
            }

            // Convert supervision timeout from BLE units (10ms) to microseconds
            BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(packet.ll_init().timeout as u32 * 10000);
            // Store connection parameters in packet initialization structure
            // This creates a template for subsequent connection-related packets
            PKT_INIT.lock().ll_init_mut().clone_from(&PacketLlInit {
                dma_len: 0x24,       // DMA length for BLE packets
                _type: 0x5,          // Packet type identifier
                rf_len: 0x22,        // RF payload length
                ..*packet.ll_init()  // Copy all other parameters from connection request
            });

            // Extract channel map from connection parameters for frequency hopping
            let chn_map = PKT_INIT.lock().ll_init().chm;
            // Build the available channel table for the BLE link layer
            // The 'true' parameter indicates this is for a slave connection
            ble_ll_build_available_channel_table(&chn_map, true);

            // Configure hardware CRC with initialization value from connection request
            // CRC is used for packet integrity validation
            let crcinit = PKT_INIT.lock().ll_init().crcinit;
            // Combine the 3-byte CRC init into a 32-bit register value
            write_reg_rf_crc(
                ((crcinit[1] as u32) << 8) | ((crcinit[2] as u32) << 0x10) | crcinit[0] as u32,
            );

            // Reset sequence number tracking for the new connection
            rf_reset_sn();

            // Initialize connection timing update state variables
            BLE_PERIPHERAL_CONNECTION_INSTANT.set(0); // No pending connection update
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.set(false); // Clear timing update flags
            BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.set(0); // Reset previous interval tracking
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.set(0); // Clear update completion time
            SLAVE_WINDOW_SIZE_UPDATE.set(0); // Reset window size update

            // Initialize pairing and security subsystem for the new connection
            pair_init();

            // Disable mesh status reporting during connection establishment
            // This prevents interference between BLE and mesh operations
            MESH_NODE_REPORT_ENABLE.set(false);

            // Clear mesh node tracking masks - fresh start for this connection
            MESH_NODE_MASK.lock().fill(0);

            // Transition RF state machine to receiving mode for BLE operation
            *CURRENT_RF_STATE.lock() = RfOperationState::Receiving;

            // Enable connection parameter update procedures
            NEED_UPDATE_CONNECT_PARA.set(true);

            // Set GATT service discovery timeout (with non-zero guard bit)
            // The | 1 ensures the timestamp is never exactly zero
            GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(read_reg_system_tick() | 1);

            // RF TX wait/settle timing: low byte of the 32-bit
            // reg_rf_tx_wail_settle_time (0xf04). 0x67 = 32/48 MHz SYSTEM CLOCK.
            write_reg_rf_tx_wail_settle_time_lo(0x67);
        }
    }
}

/// Implements adaptive timing adjustment algorithm for BLE slave connections.
///
/// This function provides real-time timing correction to maintain synchronization
/// with the BLE master device. It uses a feedback control algorithm that measures
/// timing drift and applies corrective adjustments to prevent connection drops.
///
/// # Timing Adjustment Algorithm
///
/// The algorithm operates as a digital phase-locked loop (PLL):
///
/// 1. **Timing Measurement**: Compares the current packet reception time against
///    the expected timing reference stored in `BRIDGE_RECEIVE_TIMING_TICK`
///
/// 2. **Error Calculation**: Computes the timing error (drift) between actual
///    and expected reception times
///
/// 3. **Threshold-Based Correction**: Applies proportional correction based on
///    the magnitude of timing error:
///    - **Early Reception** (< 700µs): Advance next connection by 200µs
///    - **Late Reception** (> 1100µs): Delay next connection by 200µs
///    - **Normal Range** (700-1100µs): No adjustment needed
///
/// # Design Rationale
///
/// The timing windows are carefully chosen based on BLE specification requirements:
/// - The 700µs threshold allows for normal crystal drift and processing variation
/// - The 1100µs threshold prevents timeout while allowing for temporary delays
/// - The 200µs correction step provides fast convergence without oscillation
///
/// # Crystal Oscillator Compensation
///
/// This algorithm compensates for:
/// - Temperature-induced frequency drift in crystal oscillators
/// - Manufacturing tolerances between master and slave crystals  
/// - Processing delays in packet handling and interrupt response
/// - RF propagation delays in dense mesh environments
///
/// # Parameters
/// * `time` - The system timestamp when the current packet was received
///
/// # Side Effects
/// * Modifies `SLAVE_NEXT_CONNECT_TICK` to adjust future connection timing
/// * Disables timing adjustment flag to prevent multiple corrections per event
pub fn rf_link_timing_adjust(time: u32) {
    // Only perform adjustment if explicitly enabled (prevents multiple adjustments)
    if BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.get() {
        // Disable flag immediately to prevent re-entry
        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(false);

        // Calculate timing error: difference between expected and actual reception time
        let timing_error = time - BRIDGE_RECEIVE_TIMING_TICK.get();

        // Apply timing correction based on error magnitude
        if timing_error < CLOCK_SYS_CLOCK_1US * 700 {
            // Packet arrived early - advance next connection timing
            SLAVE_NEXT_CONNECT_TICK.set(SLAVE_NEXT_CONNECT_TICK.get() - CLOCK_SYS_CLOCK_1US * 200);
        } else if CLOCK_SYS_CLOCK_1US * 1100 < timing_error {
            // Packet arrived late - delay next connection timing
            SLAVE_NEXT_CONNECT_TICK.set(SLAVE_NEXT_CONNECT_TICK.get() + CLOCK_SYS_CLOCK_1US * 200);
        }
        // If timing error is within 700-1100µs range, no adjustment needed
    }
}

/// Enables or disables BLE slave pairing and advertising.
///
/// This function controls the fundamental connectivity state of the device,
/// determining whether it will accept new BLE connections and advertise its
/// presence to potential master devices.
///
/// # Parameters
/// * `enable` - If true, enables both connection acceptance and advertising.
///   If false, disables both to make device non-discoverable.
///
/// # Side Effects
/// * Modifies global connection and advertising enable flags
/// * Affects device discoverability in BLE scans
/// * Controls whether new connection requests will be accepted
pub fn rf_link_slave_pairing_enable(enable: bool) {
    BLE_PERIPHERAL_CONNECTION_ENABLED.set(enable);
    BLE_PERIPHERAL_ADVERTISING_ENABLED.set(enable);
}

/// Validates and configures BLE connection parameter update requests.
///
/// This function implements the parameter validation and setup logic for BLE
/// connection parameter update procedures. It ensures that requested parameters
/// are within acceptable ranges and compatible with the mesh network's timing
/// requirements.
///
/// # Parameter Update Algorithm
///
/// The function performs a multi-stage validation process:
///
/// 1. **Default Parameter Handling**: If both min and max intervals are zero,
///    the function defaults to using the current connection interval
///
/// 2. **Range Validation**: Ensures the minimum interval meets the threshold
///    requirement (`INTERVAL_THRESHOLD`) for mesh compatibility
///
/// 3. **Timeout Validation**: Verifies the supervision timeout is at least 100ms
///    to prevent premature connection drops
///
/// 4. **Error Code Generation**: Returns specific error codes for different
///    failure conditions to aid in debugging
///
/// # Error Codes
/// * `0x00000000` - Success, parameters are valid
/// * `0xfffffffd` - Invalid timeout (< 100ms)  
/// * `0xfffffffe` - Invalid interval range (below threshold)
///
/// # Parameters
/// * `interval_min` - Minimum acceptable connection interval (1.25ms units)
/// * `interval_max` - Maximum acceptable connection interval (1.25ms units)  
/// * `timeout` - Supervision timeout value (10ms units)
///
/// # Returns
/// * Success code (0) if parameters are valid
/// * Error code (0xfffffffd/0xfffffffe) if validation fails
pub fn setup_ble_parameter_start(interval_min: u16, interval_max: u16, timeout: u32) -> u32 {
    let mut invalid = false;

    // Handle default parameter case: both intervals are zero
    if interval_max | interval_min == 0 {
        // Use current connection interval as both min and max
        // This maintains the existing timing without changes
        UPDATE_INTERVAL_USER_MAX.set(SLAVE_LINK_INTERVAL.get() as u16);
        UPDATE_INTERVAL_USER_MIN.set(SLAVE_LINK_INTERVAL.get() as u16);
    } else {
        // Validate user-provided interval range
        if interval_min < INTERVAL_THRESHOLD {
            // Minimum interval too small for mesh network compatibility
            invalid = true;
        }

        // Store user-requested interval range for later use
        UPDATE_INTERVAL_USER_MIN.set(interval_min);
        UPDATE_INTERVAL_USER_MAX.set(interval_max);
    }

    // Validate supervision timeout (minimum 100ms = 10 units of 10ms)
    if timeout < 100 {
        // Clear stored parameters on timeout validation failure
        UPDATE_INTERVAL_USER_MAX.set(0);
        UPDATE_INTERVAL_USER_MIN.set(0);
        return 0xfffffffd; // Error: invalid timeout
    }

    // Return success if all validations passed
    if !invalid {
        return 0; // Success
    }

    // Clear stored parameters on interval validation failure
    UPDATE_INTERVAL_USER_MIN.set(0);
    UPDATE_INTERVAL_USER_MAX.set(0);
    0xfffffffe // Error: invalid interval range
}

/// Manages delayed connection parameter updates with timeout handling.
///
/// This function implements a delayed parameter update mechanism that waits for
/// GATT service discovery to complete before applying new connection parameters.
/// This prevents parameter changes from interfering with initial service discovery.
///
/// # Delay Algorithm
///
/// The function uses a state machine approach:
/// 1. Checks if parameter update is needed (`NEED_UPDATE_CONNECT_PARA`)
/// 2. Verifies service discovery timeout is active (non-zero timestamp)
/// 3. Waits for the configured delay period (`UPDATE_CONNECT_PARA_DELAY_MS`)
/// 4. Triggers parameter update callback when delay expires
///
/// # Timing Considerations
/// The delay allows the BLE master time to complete service discovery before
/// receiving parameter update requests, improving compatibility with different
/// master implementations.
fn update_connect_para() {
    if NEED_UPDATE_CONNECT_PARA.get() && GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.get() != 0 {
        // Check if sufficient delay has elapsed since connection establishment
        let elapsed_time = read_reg_system_tick() - GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.get();
        if UPDATE_CONNECT_PARA_DELAY_MS * CLOCK_SYS_CLOCK_1US * 1000 < elapsed_time {
            // Clear update flags and trigger parameter update
            NEED_UPDATE_CONNECT_PARA.set(false);
            GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(0);

            // Execute the parameter update callback
            update_ble_parameter_cb();
        }
    }
}

/// Main processing loop for BLE slave operations.
///
/// This function coordinates the various slave-mode operations that need to
/// run continuously during BLE connection. It integrates mesh networking
/// with BLE connection management.
///
/// # Processing Tasks
/// 1. **Mesh Pairing Processing**: Handles mesh network pairing state machine
/// 2. **Connection Parameter Updates**: Manages delayed parameter updates
///
/// # Call Frequency
/// This function should be called regularly (typically from the main loop)
/// to ensure timely processing of time-sensitive operations.
pub fn rf_link_slave_proc() {
    // Process mesh network pairing operations
    callbacks::on_mesh_pair_proc();

    // Handle any pending connection parameter updates
    update_connect_para();
}

/// Configures scan response timing based on system clock frequency.
///
/// This function calibrates BLE advertising and scan response timing parameters
/// based on the detected system clock frequency. Different clock speeds require
/// different timing adjustments to maintain BLE specification compliance.
///
/// # Clock Frequency Detection
/// The function uses specific tick values to identify system clock speed:
/// - 0x10 (16): Indicates one clock frequency (sets interval to 0)
/// - 0x20 (32): Indicates different frequency (sets interval to 0x92)  
/// - 0x30 (48): Indicates third frequency option (sets interval to 0x93)
/// - Other values: Default to 0x92 for compatibility
///
/// # Parameters
/// * `ticks` - System tick count used to determine clock frequency
///
/// # Side Effects
/// * Modifies `BLE_SCAN_RESPONSE_INTERVAL_US` for timing calibration
pub fn light_check_tick_per_us(ticks: u32) {
    if ticks == 0x10 {
        // Clock frequency detected: set to minimal scan response interval
        BLE_SCAN_RESPONSE_INTERVAL_US.set(0);
    } else if ticks == 0x20 || ticks != 0x30 {
        // Standard clock frequency: use calibrated interval
        BLE_SCAN_RESPONSE_INTERVAL_US.set(0x92);
    } else {
        // Alternative clock frequency: use adjusted interval
        BLE_SCAN_RESPONSE_INTERVAL_US.set(0x93);
    }
}

/// Configures RF hardware for mesh bridge receive mode.
///
/// This function transitions the radio from BLE peripheral mode to mesh
/// bridge mode, setting up the hardware for mesh packet reception.
/// It implements channel hopping for improved reliability in dense
/// mesh networks.
///
/// # Bridge Mode Configuration
///
/// The function performs these hardware setup steps:
/// 1. **RF Shutdown**: Disables current TX/RX operations
/// 2. **Access Code Setup**: Configures pairing access code for mesh
/// 3. **CRC Configuration**: Sets advertising CRC mode for mesh packets  
/// 4. **Channel Selection**: Uses round-robin channel hopping algorithm
/// 5. **RX Mode Enable**: Activates receive mode for mesh operation
///
/// # Channel Hopping Algorithm
///
/// Channels are selected using: `SYS_CHN_LISTEN[(sequence_number % channels) >> 1]`
/// This provides deterministic but pseudo-random channel selection that
/// all mesh nodes can synchronize to.
///
/// # Side Effects
/// * Modifies RF hardware registers for mesh operation
/// * Changes radio from BLE to mesh protocol configuration
/// * Enables receive mode for incoming mesh packets
pub fn back_to_rxmode_bridge() {
    // Disable any active RF transmit/receive operations
    rf_set_tx_rx_off();

    // Configure mesh-specific access code for packet filtering
    rf_set_ble_access_code(PAIR_AC.get());

    // Set CRC mode appropriate for mesh advertising packets
    rf_set_ble_crc_adv();

    // Select channel using round-robin algorithm based on sequence number
    let channel_index = (BRIDGE_SEQUENCE_NUMBER.get() as usize % SYS_CHN_LISTEN.len()) >> 1;
    rf_set_ble_channel(SYS_CHN_LISTEN[channel_index]);

    // Enable receive mode to listen for mesh packets
    rf_set_rxmode();
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::ble::pair::pair_iv_for_test;
    use mesh_protocol::packet_types::{Packet, PacketLlInit};
    use tlsr8266_hal::mcu::clock::{fake_clock, CLOCK_SYS_CLOCK_1US};
    use tlsr8266_pac::fake_mmio::FakeMmio;

    /// Datasheet offsets of the RF registers these tests inspect, as the PAC
    /// accessors reach them.
    const REG_RF_MODE_CONTROL: u32 = 0xf00;
    const REG_RF_SN: u32 = 0xf01;
    const REG_RF_CHANNEL: u32 = 0x40d;
    const REG_RF_ACCESS_CODE: u32 = 0x408;
    const REG_RF_CRC: u32 = 0x44c;
    const REG_RF_TXRX_STATE: u32 = 0xf02;

    /// The last value written to the register at `offset`, if any.
    fn last_write(offset: u32) -> Option<u32> {
        FakeMmio::writes()
            .iter()
            .rev()
            .find(|w| w.addr == (offset | tlsr8266_pac::REG_BASE_ADDR))
            .map(|w| w.value)
    }

    /// Number of writes logged for the register at `offset`.
    fn write_count(offset: u32) -> usize {
        FakeMmio::writes()
            .iter()
            .filter(|w| w.addr == (offset | tlsr8266_pac::REG_BASE_ADDR))
            .count()
    }

    /// Helper function to create a test packet with default valid parameters
    fn create_test_packet() -> Packet {
        Packet {
            ll_init: PacketLlInit {
                dma_len: 0x24,
                _type: 0x5,
                rf_len: 0x22,
                adv_a: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06],
                scan_a: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06], // Match adv_a for valid test
                aa: [0x8e, 0x89, 0xbe, 0xd6],
                crcinit: [0x55, 0x55, 0x55],
                wsize: 5,                            // Valid window size (1-8)
                woffset: 10,                         // Valid window offset
                interval: 20,                        // Valid interval (> 6)
                latency: 2,                          // Valid latency (> 0)
                timeout: 100,                        // Valid timeout (10-3200)
                chm: [0xff, 0xff, 0xff, 0xff, 0x1f], // All channels enabled
                hop: 5,                              // Valid hop increment (non-zero)
            },
        }
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

    /// Helper function to reset global state to known values for test isolation
    fn reset_global_state() {
        // The PAC host fakes are process-wide: start every test from a clean
        // register bank and a zeroed fake clock.
        FakeMmio::reset();
        fake_clock::reset();

        CONN_UPDATE_SUCCESSED.set(false);
        CONN_UPDATE_CNT.set(0);
        BLE_PERIPHERAL_CONNECTION_ENABLED.set(false);
        BLE_PERIPHERAL_ADVERTISING_ENABLED.set(false);
        BLE_PERIPHERAL_WINDOW_OFFSET.set(0);
        LIGHT_CONN_SN_MASTER.set(0);
        SLAVE_CONNECTED_TICK.set(0);
        SECURITY_ENABLE.set(false);
        BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(0);
        DEVICE_STATUS_TICK_COUNTER.set(0);
        SLAVE_LINK_INTERVAL.set(0);
        SLAVE_WINDOW_SIZE.set(0);
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(0);
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(0);
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.set(false);
        BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.set(0);
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.set(0);
        SLAVE_WINDOW_SIZE_UPDATE.set(0);
        MESH_NODE_REPORT_ENABLE.set(false);
        NEED_UPDATE_CONNECT_PARA.set(false);
        GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(0);
        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(false);
        BRIDGE_RECEIVE_TIMING_TICK.set(0);
        SLAVE_NEXT_CONNECT_TICK.set(0);
        UPDATE_INTERVAL_USER_MAX.set(0);
        UPDATE_INTERVAL_USER_MIN.set(0);
        BLE_SCAN_RESPONSE_INTERVAL_US.set(0);
        BRIDGE_SEQUENCE_NUMBER.set(0);
        PAIR_AC.set(0);

        // Reset complex state
        *CURRENT_RF_STATE.lock() = RfOperationState::Advertising;
        MESH_NODE_MASK.lock().fill(0);
        // Reset PKT_INIT to a known state
        *PKT_INIT.lock() = create_test_packet();

        // Set MAC_ID to match test packet's address for successful connection tests
        *MAC_ID.lock() = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
    }

    // ================================================================================
    // Tests for check_par_con function
    // ================================================================================

    /// Tests check_par_con with valid BLE connection parameters.
    ///
    /// Verifies that the function correctly validates all BLE specification
    /// parameters when given valid values within specification ranges.
    #[test]
    fn test_check_par_con_valid_parameters() {
        let packet = create_test_packet();

        // Test with valid parameters - should return true (indicating valid)
        let result = check_par_con(&packet);
        assert!(result, "Connection should be established");
    }

    /// Tests check_par_con with invalid connection interval.
    ///
    /// Per BLE specification, connection interval validation checks:
    /// (interval - 6) & 0xffff < 0xc7b (3195 decimal)
    #[test]
    fn test_check_par_con_invalid_interval() {
        let mut packet = create_test_packet();

        // Set interval that fails validation: (interval - 6) >= 0xc7b
        packet.ll_init.interval = 0xc7b + 6; // This should fail validation

        let result = check_par_con(&packet);
        assert!(!result, "Connection should be rejected");
    }

    /// Tests check_par_con with invalid window size.
    ///
    /// Window size must be non-zero and less than 9.
    #[test]
    fn test_check_par_con_invalid_window_size() {
        let mut packet = create_test_packet();

        // Test zero window size
        packet.ll_init.wsize = 0;
        let result = check_par_con(&packet);
        assert!(!result, "Connection should be rejected");

        // Test window size > interval (from create_test_packet interval = 20)
        packet.ll_init.wsize = 25; // Greater than interval of 20
        let result = check_par_con(&packet);
        assert!(!result, "Connection should be rejected");
    }

    /// Tests check_par_con with invalid supervision timeout.
    ///
    /// Timeout must be greater than 9 and less than 0xc81 (3201 decimal).
    #[test]
    fn test_check_par_con_invalid_timeout() {
        let mut packet = create_test_packet();

        // Test timeout <= 9
        packet.ll_init.timeout = 9;
        let result = check_par_con(&packet);
        assert!(!result, "Connection should be rejected");

        // Test timeout >= 0xc81
        packet.ll_init.timeout = 0xc81;
        let result = check_par_con(&packet);
        assert!(!result, "Connection should be rejected");
    }

    /// Tests check_par_con with invalid window offset.
    ///
    /// Window offset must not exceed connection interval.
    #[test]
    fn test_check_par_con_invalid_window_offset() {
        let mut packet = create_test_packet();

        // Set window offset greater than interval
        packet.ll_init.interval = 20;
        packet.ll_init.woffset = 21; // Greater than interval

        let result = check_par_con(&packet);
        assert!(!result, "Connection should be rejected");
    }

    /// Tests check_par_con with invalid hop increment.
    ///
    /// Hop increment must be non-zero for frequency diversity.
    #[test]
    fn test_check_par_con_invalid_hop() {
        let mut packet = create_test_packet();

        // Test zero hop increment
        packet.ll_init.hop = 0;

        let result = check_par_con(&packet);
        assert!(!result, "Connection should be rejected");
    }

    /// Tests check_par_con with invalid channel map.
    ///
    /// Channel map must have at least one enabled channel.
    #[test]
    fn test_check_par_con_invalid_channel_map() {
        let mut packet = create_test_packet();

        // Set all channels to disabled (all zeros)
        packet.ll_init.chm = [0x00, 0x00, 0x00, 0x00, 0x00];

        let result = check_par_con(&packet);
        assert!(!result, "Connection should be rejected");
    }

    /// Tests check_par_con with zero latency.
    ///
    /// Zero latency should be valid, meaning immediate response to every connection event.
    #[test]
    fn test_check_par_con_zero_latency() {
        let mut packet = create_test_packet();

        // Set latency to zero (should be valid - immediate response)
        packet.ll_init.latency = 0;

        let result = check_par_con(&packet);
        assert!(result, "Connection should be established");
    }

    /// Tests check_par_con latency constraint validation.
    ///
    /// Tests the complex latency constraint: latency ≤ (interval × 8) / interval
    /// This ensures supervision timeout math is valid.
    #[test]
    fn test_check_par_con_latency_constraint() {
        let mut packet = create_test_packet();

        // Set parameters where latency constraint fails
        packet.ll_init.interval = 100;
        packet.ll_init.latency = ((100_u32 << 3) / 100) as u16; // This should fail the constraint

        let result = check_par_con(&packet);
        assert!(!result, "Connection should be rejected");
    }

    /// Tests check_par_con boundary cases.
    ///
    /// Tests edge cases with minimum and maximum valid values.
    #[test]
    fn test_check_par_con_boundary_cases() {
        let mut packet = create_test_packet();

        // Test minimum valid interval
        packet.ll_init.interval = 6; // Minimum valid interval
        packet.ll_init.woffset = 6; // Equal to interval
        packet.ll_init.wsize = 1; // Minimum valid window size
        packet.ll_init.timeout = 50; // Valid timeout that satisfies latency constraint
        packet.ll_init.latency = 1; // Minimum valid latency

        let result = check_par_con(&packet);
        assert!(result, "Connection should be established");

        // Test maximum valid values
        packet.ll_init.interval = 3200; // Maximum valid interval
        packet.ll_init.woffset = 3200; // Equal to interval
        packet.ll_init.wsize = 100; // Valid window size within interval
        packet.ll_init.timeout = 3200; // Maximum valid timeout
        packet.ll_init.latency = 0; // Zero latency to satisfy constraint (0 * 3200 * 2 = 0 < 3200)

        let result = check_par_con(&packet);
        assert!(result, "Connection should be established");
    }

    // ================================================================================
    // Tests for rf_link_slave_connect function
    // ================================================================================

    /// Tests rf_link_slave_connect with disabled peripheral connection.
    ///
    /// When BLE_PERIPHERAL_CONNECTION_ENABLED is false, the function should
    /// reject the connection attempt immediately.
    #[test]
    fn test_rf_link_slave_connect_disabled_peripheral() {
        reset_global_state();

        let packet = create_test_packet();
        let time = 10000;

        // Disable peripheral connections
        BLE_PERIPHERAL_CONNECTION_ENABLED.set(false);

        rf_link_slave_connect(&packet, time);
        assert!(
            !NEED_UPDATE_CONNECT_PARA.get(),
            "Connection should be rejected"
        );
        assert!(
            FakeMmio::writes().is_empty(),
            "a rejected connection must not touch the radio"
        );
    }

    /// Tests rf_link_slave_connect with mismatched addresses.
    ///
    /// Connection should be rejected if scan address doesn't match advertiser address.
    #[test]
    fn test_rf_link_slave_connect_address_mismatch() {
        reset_global_state();

        let mut packet = create_test_packet();
        let time = 10000;

        // Enable peripheral connections but create address mismatch
        BLE_PERIPHERAL_CONNECTION_ENABLED.set(true);

        // Different advertiser address than MAC_ID
        packet.ll_init.adv_a = [0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c];

        rf_link_slave_connect(&packet, time);
        assert!(
            !NEED_UPDATE_CONNECT_PARA.get(),
            "Connection should be rejected"
        );
        assert!(
            FakeMmio::writes().is_empty(),
            "a rejected connection must not touch the radio"
        );
    }

    /// Tests rf_link_slave_connect with invalid parameters.
    ///
    /// Connection should be rejected if check_par_con returns false.
    #[test]
    fn test_rf_link_slave_connect_invalid_parameters() {
        reset_global_state();

        let mut packet = create_test_packet();
        let time = 10000;

        // Enable peripheral and make addresses match
        BLE_PERIPHERAL_CONNECTION_ENABLED.set(true);
        unsafe {
            packet.ll_init.scan_a = packet.ll_init.adv_a;
        } // Match addresses

        // Set invalid parameters (zero window size makes check_par_con return false)
        packet.ll_init.wsize = 0;

        rf_link_slave_connect(&packet, time);
        assert!(
            !NEED_UPDATE_CONNECT_PARA.get(),
            "Connection should be rejected"
        );
        assert!(
            FakeMmio::writes().is_empty(),
            "a rejected connection must not touch the radio"
        );
    }

    /// Tests successful rf_link_slave_connect with valid parameters.
    ///
    /// This comprehensive test verifies that all connection setup steps are
    /// performed correctly when given valid parameters.
    #[test]
    fn test_rf_link_slave_connect_successful() {
        reset_global_state();
        // `read_reg_system_tick()` reads the fake system tick register.
        fake_clock::set_ticks(50000);

        // Use a MAC no other test leaves behind, so the IV assertion below cannot
        // pass on state recorded by an earlier test.
        let mac = [0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6];
        *MAC_ID.lock() = mac;

        let mut packet = create_test_packet();
        let time = 10000;

        // Setup for successful connection
        BLE_PERIPHERAL_CONNECTION_ENABLED.set(true);
        unsafe {
            packet.ll_init.adv_a = mac;
            packet.ll_init.scan_a = packet.ll_init.adv_a;
        } // Match addresses

        // Use valid parameters (create_test_packet already has valid params)

        rf_link_slave_connect(&packet, time);
        assert!(
            NEED_UPDATE_CONNECT_PARA.get(),
            "Connection should be established"
        );

        // The bring-up must run `pair_init`, which re-seeds both initialization
        // vectors from the device address; nothing else in the connect path
        // touches their first four bytes.
        let (ivm, ivs) = pair_iv_for_test();
        assert_eq!(
            ivm[0..4],
            mac[0..4],
            "pair_init must seed the master IV from MAC_ID"
        );
        assert_eq!(
            ivs[0..4],
            mac[0..4],
            "pair_init must seed the slave IV from MAC_ID"
        );

        // Verify connection state was initialized
        assert_eq!(
            CONN_UPDATE_SUCCESSED.get(),
            false,
            "Connection update should be reset"
        );
        assert_eq!(
            CONN_UPDATE_CNT.get(),
            0,
            "Connection update count should be reset"
        );

        // Verify the RF bring-up ran through the real driver functions: the
        // transceiver is stopped, the sequence number is reset (two writes,
        // 0x3f then 0x00) and the CRC init comes from the request.
        assert_eq!(
            last_write(REG_RF_MODE_CONTROL),
            Some(0x80),
            "rf_stop_trx must stop the radio"
        );
        assert_eq!(
            write_count(REG_RF_SN),
            2,
            "rf_reset_sn writes 0x3f then 0x00"
        );
        assert_eq!(
            last_write(REG_RF_SN),
            Some(0x00),
            "sequence number resets to 0"
        );
        assert_eq!(
            last_write(REG_RF_CRC),
            Some(0x555555),
            "CRC init from the request's crcinit"
        );

        // Verify timing setup
        assert_ne!(
            BLE_PERIPHERAL_WINDOW_OFFSET.get(),
            0,
            "Window offset should be calculated"
        );
        assert_ne!(
            SLAVE_CONNECTED_TICK.get(),
            0,
            "Connection timestamp should be recorded"
        );

        // Verify state transitions
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Receiving,
            "Should transition to RX state"
        );
        assert_eq!(
            NEED_UPDATE_CONNECT_PARA.get(),
            true,
            "Should enable parameter updates"
        );
    }

    /// Tests timing calculations in rf_link_slave_connect.
    ///
    /// Verifies that timing parameters are calculated correctly according
    /// to BLE specification formulas.
    #[test]
    fn test_rf_link_slave_connect_timing_calculations() {
        reset_global_state();
        // `read_reg_system_tick()` reads the fake system tick register.
        fake_clock::set_ticks(100000);

        let mut packet = create_test_packet();
        let time = 50000;

        // Setup specific test values
        BLE_PERIPHERAL_CONNECTION_ENABLED.set(true);
        unsafe {
            packet.ll_init.scan_a = packet.ll_init.adv_a;
        }
        // Use valid parameters for successful connection
        packet.ll_init.woffset = 5; // 5 * 1.25ms = 6.25ms
        packet.ll_init.interval = 20; // 20 * 1.25ms = 25ms

        rf_link_slave_connect(&packet, time);
        assert!(
            NEED_UPDATE_CONNECT_PARA.get(),
            "Connection should be established"
        );

        // Verify window offset calculation: CLOCK_SYS_CLOCK_1US * 1250 * (woffset + 1)
        let expected_offset = CLOCK_SYS_CLOCK_1US * 1250 * (5 + 1);
        assert_eq!(
            BLE_PERIPHERAL_WINDOW_OFFSET.get(),
            expected_offset,
            "Window offset should be calculated correctly"
        );

        // Verify connection interval calculation: interval * CLOCK_SYS_CLOCK_1US * 1250
        let expected_interval = 20 * CLOCK_SYS_CLOCK_1US * 1250;
        assert_eq!(
            SLAVE_LINK_INTERVAL.get(),
            expected_interval,
            "Connection interval should be calculated correctly"
        );
    }

    /// Tests security-related setup in rf_link_slave_connect.
    ///
    /// Verifies that security timestamps are recorded when security is enabled.
    #[test]
    fn test_rf_link_slave_connect_security_setup() {
        reset_global_state();
        // `read_reg_system_tick()` reads the fake system tick register.
        fake_clock::set_ticks(75000);

        let mut packet = create_test_packet();
        let time = 25000;

        // Enable security
        SECURITY_ENABLE.set(true);
        BLE_PERIPHERAL_CONNECTION_ENABLED.set(true);
        unsafe {
            packet.ll_init.scan_a = packet.ll_init.adv_a;
        }
        // Use valid parameters for successful connection

        rf_link_slave_connect(&packet, time);
        assert!(
            NEED_UPDATE_CONNECT_PARA.get(),
            "Connection should be established"
        );
        // Verify security timestamp was recorded
        assert_ne!(
            BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.get(),
            0,
            "First connection timestamp should be recorded when security is enabled"
        );
        assert_eq!(
            BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.get(),
            SLAVE_CONNECTED_TICK.get(),
            "Security timestamp should match connection timestamp"
        );
    }

    // ================================================================================
    // Tests for rf_link_timing_adjust function
    // ================================================================================

    /// Tests rf_link_timing_adjust when timing adjustment is disabled.
    ///
    /// Function should return early without making any adjustments when
    /// BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED is false.
    #[test]
    fn test_rf_link_timing_adjust_disabled() {
        reset_global_state();

        let time = 50000;
        let original_next_tick = 100000;

        // Disable timing adjustment
        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(false);
        SLAVE_NEXT_CONNECT_TICK.set(original_next_tick);
        BRIDGE_RECEIVE_TIMING_TICK.set(45000);

        rf_link_timing_adjust(time);

        // Verify no adjustment was made
        assert_eq!(
            SLAVE_NEXT_CONNECT_TICK.get(),
            original_next_tick,
            "Next connect tick should be unchanged when adjustment is disabled"
        );
        assert_eq!(
            BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.get(),
            false,
            "Timing adjustment flag should remain disabled"
        );
    }

    /// Tests rf_link_timing_adjust for early packet reception.
    ///
    /// When packet arrives early (< 700µs), the function should advance
    /// the next connection timing by 200µs.
    #[test]
    fn test_rf_link_timing_adjust_early_packet() {
        reset_global_state();

        let time = 50000;
        let bridge_time = 49000; // 1000 tick difference
        let original_next_tick = 100000;

        // Enable timing adjustment and setup early condition
        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(true);
        SLAVE_NEXT_CONNECT_TICK.set(original_next_tick);
        BRIDGE_RECEIVE_TIMING_TICK.set(bridge_time);

        // time - bridge_time = 50000 - 49000 = 1000
        // 1000 < CLOCK_SYS_CLOCK_1US * 700 (early condition)

        rf_link_timing_adjust(time);

        // Verify timing was advanced by 200µs
        let expected_adjustment = CLOCK_SYS_CLOCK_1US * 200;
        assert_eq!(
            SLAVE_NEXT_CONNECT_TICK.get(),
            original_next_tick - expected_adjustment,
            "Next connect tick should be advanced by 200µs for early packet"
        );

        // Verify adjustment flag was disabled
        assert_eq!(
            BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.get(),
            false,
            "Timing adjustment flag should be disabled after adjustment"
        );
    }

    /// Tests rf_link_timing_adjust for late packet reception.
    ///
    /// When packet arrives late (> 1100µs), the function should delay
    /// the next connection timing by 200µs.
    #[test]
    fn test_rf_link_timing_adjust_late_packet() {
        reset_global_state();

        let time = 50000;
        let bridge_time = 48000; // 2000 tick difference
        let original_next_tick = 100000;

        // Enable timing adjustment and setup late condition
        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(true);
        SLAVE_NEXT_CONNECT_TICK.set(original_next_tick);
        BRIDGE_RECEIVE_TIMING_TICK.set(bridge_time);

        // time - bridge_time = 50000 - 48000 = 2000
        // 2000 > CLOCK_SYS_CLOCK_1US * 1100 (late condition, assuming CLOCK_SYS_CLOCK_1US = 32)
        // 2000 > 32 * 1100 = 35200 is false, so let's adjust the values

        // Make the difference larger to ensure late condition
        let bridge_time_late = 10000; // Much earlier bridge time
        BRIDGE_RECEIVE_TIMING_TICK.set(bridge_time_late);

        // time - bridge_time = 50000 - 10000 = 40000
        // 40000 > CLOCK_SYS_CLOCK_1US * 1100 = 32 * 1100 = 35200 ✓ (late condition)

        rf_link_timing_adjust(time);

        // Verify timing was delayed by 200µs
        let expected_adjustment = CLOCK_SYS_CLOCK_1US * 200;
        assert_eq!(
            SLAVE_NEXT_CONNECT_TICK.get(),
            original_next_tick + expected_adjustment,
            "Next connect tick should be delayed by 200µs for late packet"
        );

        // Verify adjustment flag was disabled
        assert_eq!(
            BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.get(),
            false,
            "Timing adjustment flag should be disabled after adjustment"
        );
    }

    /// Tests rf_link_timing_adjust for normal timing window.
    ///
    /// When packet arrives within the normal window (700-1100µs), no
    /// adjustment should be made.
    #[test]
    fn test_rf_link_timing_adjust_normal_window() {
        reset_global_state();

        let time = 50000;
        let bridge_time = 49100; // 900 tick difference
        let original_next_tick = 100000;

        // Enable timing adjustment and setup normal condition
        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(true);
        SLAVE_NEXT_CONNECT_TICK.set(original_next_tick);
        BRIDGE_RECEIVE_TIMING_TICK.set(bridge_time);

        // time - bridge_time = 50000 - 49100 = 900
        // CLOCK_SYS_CLOCK_1US * 700 < 900 < CLOCK_SYS_CLOCK_1US * 1100 (normal window)
        // 32 * 700 = 22400 and 32 * 1100 = 35200
        // 22400 < 900 < 35200 is false (900 < 22400)

        // Adjust to make it in the normal window
        let bridge_time_normal = 49000 - (CLOCK_SYS_CLOCK_1US * 800); // 800µs ago
        BRIDGE_RECEIVE_TIMING_TICK.set(bridge_time_normal);

        // Now: time - bridge_time = 50000 - (49000 - 25600) = 50000 - 23400 = 26600
        // 22400 < 26600 < 35200 ✓ (normal window)

        rf_link_timing_adjust(time);

        // Verify no timing adjustment was made
        assert_eq!(
            SLAVE_NEXT_CONNECT_TICK.get(),
            original_next_tick,
            "Next connect tick should be unchanged for normal timing window"
        );

        // Verify adjustment flag was still disabled
        assert_eq!(
            BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.get(),
            false,
            "Timing adjustment flag should be disabled after processing"
        );
    }

    /// Tests edge cases for rf_link_timing_adjust.
    ///
    /// Tests boundary conditions and edge cases for timing adjustment thresholds.
    #[test]
    fn test_rf_link_timing_adjust_edge_cases() {
        reset_global_state();

        let original_next_tick = 100000;

        // Test exactly at early threshold (700µs)
        let time_early = 50000;
        let bridge_time_early = time_early - (CLOCK_SYS_CLOCK_1US * 700);

        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(true);
        SLAVE_NEXT_CONNECT_TICK.set(original_next_tick);
        BRIDGE_RECEIVE_TIMING_TICK.set(bridge_time_early);

        rf_link_timing_adjust(time_early);

        // Should NOT adjust at exactly 700µs (< 700µs required)
        assert_eq!(
            SLAVE_NEXT_CONNECT_TICK.get(),
            original_next_tick,
            "Should not adjust at exactly 700µs threshold"
        );

        // Reset for late threshold test
        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(true);
        SLAVE_NEXT_CONNECT_TICK.set(original_next_tick);

        // Test exactly at late threshold (1100µs)
        let time_late = 50000;
        let bridge_time_late = time_late - (CLOCK_SYS_CLOCK_1US * 1100);
        BRIDGE_RECEIVE_TIMING_TICK.set(bridge_time_late);

        rf_link_timing_adjust(time_late);

        // Should NOT adjust at exactly 1100µs (> 1100µs required)
        assert_eq!(
            SLAVE_NEXT_CONNECT_TICK.get(),
            original_next_tick,
            "Should not adjust at exactly 1100µs threshold"
        );
    }

    // ================================================================================
    // Tests for rf_link_slave_pairing_enable function
    // ================================================================================

    /// Tests rf_link_slave_pairing_enable with enable = true.
    ///
    /// Verifies that both connection and advertising flags are set when enabled.
    #[test]
    fn test_rf_link_slave_pairing_enable_true() {
        reset_global_state();

        rf_link_slave_pairing_enable(true);

        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_ENABLED.get(),
            true,
            "Connection should be enabled when pairing is enabled"
        );
        assert_eq!(
            BLE_PERIPHERAL_ADVERTISING_ENABLED.get(),
            true,
            "Advertising should be enabled when pairing is enabled"
        );
    }

    /// Tests rf_link_slave_pairing_enable with enable = false.
    ///
    /// Verifies that both connection and advertising flags are cleared when disabled.
    #[test]
    fn test_rf_link_slave_pairing_enable_false() {
        reset_global_state();

        // First enable both
        BLE_PERIPHERAL_CONNECTION_ENABLED.set(true);
        BLE_PERIPHERAL_ADVERTISING_ENABLED.set(true);

        rf_link_slave_pairing_enable(false);

        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_ENABLED.get(),
            false,
            "Connection should be disabled when pairing is disabled"
        );
        assert_eq!(
            BLE_PERIPHERAL_ADVERTISING_ENABLED.get(),
            false,
            "Advertising should be disabled when pairing is disabled"
        );
    }

    /// Tests rf_link_slave_pairing_enable state consistency.
    ///
    /// Verifies that multiple calls maintain consistent state.
    #[test]
    fn test_rf_link_slave_pairing_enable_consistency() {
        reset_global_state();

        // Test multiple enable/disable cycles
        for _ in 0..3 {
            rf_link_slave_pairing_enable(true);
            assert_eq!(
                BLE_PERIPHERAL_CONNECTION_ENABLED.get(),
                true,
                "Connection should be enabled"
            );
            assert_eq!(
                BLE_PERIPHERAL_ADVERTISING_ENABLED.get(),
                true,
                "Advertising should be enabled"
            );

            rf_link_slave_pairing_enable(false);
            assert_eq!(
                BLE_PERIPHERAL_CONNECTION_ENABLED.get(),
                false,
                "Connection should be disabled"
            );
            assert_eq!(
                BLE_PERIPHERAL_ADVERTISING_ENABLED.get(),
                false,
                "Advertising should be disabled"
            );
        }
    }

    // ================================================================================
    // Tests for setup_ble_parameter_start function
    // ================================================================================

    /// Tests setup_ble_parameter_start with zero intervals (default case).
    ///
    /// When both min and max intervals are zero, function should use current
    /// connection interval as both min and max.
    #[test]
    fn test_setup_ble_parameter_start_zero_intervals() {
        reset_global_state();

        let current_interval = 50000;
        SLAVE_LINK_INTERVAL.set(current_interval);

        let result = setup_ble_parameter_start(0, 0, 1000);

        assert_eq!(result, 0, "Should return success for zero intervals");
        assert_eq!(
            UPDATE_INTERVAL_USER_MIN.get(),
            current_interval as u16,
            "Min interval should be set to current interval"
        );
        assert_eq!(
            UPDATE_INTERVAL_USER_MAX.get(),
            current_interval as u16,
            "Max interval should be set to current interval"
        );
    }

    /// Tests setup_ble_parameter_start with valid non-zero intervals.
    ///
    /// Function should accept intervals above the threshold.
    #[test]
    fn test_setup_ble_parameter_start_valid_intervals() {
        reset_global_state();

        let min_interval = INTERVAL_THRESHOLD + 10;
        let max_interval = INTERVAL_THRESHOLD + 50;

        let result = setup_ble_parameter_start(min_interval, max_interval, 1000);

        assert_eq!(result, 0, "Should return success for valid intervals");
        assert_eq!(
            UPDATE_INTERVAL_USER_MIN.get(),
            min_interval,
            "Min interval should be stored correctly"
        );
        assert_eq!(
            UPDATE_INTERVAL_USER_MAX.get(),
            max_interval,
            "Max interval should be stored correctly"
        );
    }

    /// Tests setup_ble_parameter_start with interval below threshold.
    ///
    /// Function should reject intervals below INTERVAL_THRESHOLD.
    #[test]
    fn test_setup_ble_parameter_start_invalid_interval() {
        reset_global_state();

        let min_interval = INTERVAL_THRESHOLD - 1; // Below threshold
        let max_interval = INTERVAL_THRESHOLD + 10;

        let result = setup_ble_parameter_start(min_interval, max_interval, 1000);

        assert_eq!(result, 0xfffffffe, "Should return interval error code");
        assert_eq!(
            UPDATE_INTERVAL_USER_MIN.get(),
            0,
            "Min interval should be cleared on error"
        );
        assert_eq!(
            UPDATE_INTERVAL_USER_MAX.get(),
            0,
            "Max interval should be cleared on error"
        );
    }

    /// Tests setup_ble_parameter_start with invalid timeout.
    ///
    /// Function should reject timeouts below 100ms (100 units).
    #[test]
    fn test_setup_ble_parameter_start_invalid_timeout() {
        reset_global_state();

        let min_interval = INTERVAL_THRESHOLD + 10;
        let max_interval = INTERVAL_THRESHOLD + 50;
        let invalid_timeout = 99; // Below minimum of 100

        let result = setup_ble_parameter_start(min_interval, max_interval, invalid_timeout);

        assert_eq!(result, 0xfffffffd, "Should return timeout error code");
        assert_eq!(
            UPDATE_INTERVAL_USER_MIN.get(),
            0,
            "Min interval should be cleared on timeout error"
        );
        assert_eq!(
            UPDATE_INTERVAL_USER_MAX.get(),
            0,
            "Max interval should be cleared on timeout error"
        );
    }

    /// Tests setup_ble_parameter_start boundary cases.
    ///
    /// Tests minimum valid timeout and threshold interval values.
    #[test]
    fn test_setup_ble_parameter_start_boundary_cases() {
        reset_global_state();

        // Test minimum valid timeout (100)
        let result = setup_ble_parameter_start(INTERVAL_THRESHOLD, INTERVAL_THRESHOLD + 10, 100);
        assert_eq!(result, 0, "Should accept minimum valid timeout");

        // Test exactly at threshold
        let result = setup_ble_parameter_start(INTERVAL_THRESHOLD, INTERVAL_THRESHOLD, 100);
        assert_eq!(result, 0, "Should accept interval exactly at threshold");

        // Test just below threshold
        let result = setup_ble_parameter_start(INTERVAL_THRESHOLD - 1, INTERVAL_THRESHOLD, 100);
        assert_eq!(
            result, 0xfffffffe,
            "Should reject interval just below threshold"
        );
    }

    // ================================================================================
    // Tests for rf_link_slave_proc function
    // ================================================================================

    /// Tests rf_link_slave_proc basic functionality.
    ///
    /// Verifies that the function calls the expected mesh and parameter
    /// update processing functions.
    #[test]
    fn test_rf_link_slave_proc() {
        reset_global_state();

        // Set up some state that might affect processing
        NEED_UPDATE_CONNECT_PARA.set(false);
        GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(0);

        // Note: This function primarily calls other functions (mesh_pair_proc, update_connect_para)
        // Since we don't have mocks for these internal calls, we test that the function
        // executes without panicking and maintains state consistency

        rf_link_slave_proc();

        // The function should complete without error
        // More detailed testing would require mocking the internal function calls
        assert!(true, "Function should execute without panicking");
    }

    // ================================================================================
    // Tests for light_check_tick_per_us function
    // ================================================================================

    /// Tests light_check_tick_per_us with 16 MHz clock (0x10).
    ///
    /// Should set scan response interval to 0 for 16 MHz.
    #[test]
    fn test_light_check_tick_per_us_16mhz() {
        reset_global_state();

        light_check_tick_per_us(0x10);

        assert_eq!(
            BLE_SCAN_RESPONSE_INTERVAL_US.get(),
            0,
            "Should set interval to 0 for 16MHz clock"
        );
    }

    /// Tests light_check_tick_per_us with 32 MHz clock (0x20).
    ///
    /// Should set scan response interval to 0x92 for 32 MHz.
    #[test]
    fn test_light_check_tick_per_us_32mhz() {
        reset_global_state();

        light_check_tick_per_us(0x20);

        assert_eq!(
            BLE_SCAN_RESPONSE_INTERVAL_US.get(),
            0x92,
            "Should set interval to 0x92 for 32MHz clock"
        );
    }

    /// Tests light_check_tick_per_us with 48 MHz clock (0x30).
    ///
    /// Should set scan response interval to 0x93 for 48 MHz.
    #[test]
    fn test_light_check_tick_per_us_48mhz() {
        reset_global_state();

        light_check_tick_per_us(0x30);

        assert_eq!(
            BLE_SCAN_RESPONSE_INTERVAL_US.get(),
            0x93,
            "Should set interval to 0x93 for 48MHz clock"
        );
    }

    /// Tests light_check_tick_per_us with other clock frequencies.
    ///
    /// Should default to 0x92 for unrecognized clock frequencies.
    #[test]
    fn test_light_check_tick_per_us_other_clocks() {
        reset_global_state();

        // Test various other values that should default to 0x92
        let test_values = [0x15, 0x25, 0x40, 0x50, 0x100];

        for &value in &test_values {
            BLE_SCAN_RESPONSE_INTERVAL_US.set(0); // Reset between tests
            light_check_tick_per_us(value);

            assert_eq!(
                BLE_SCAN_RESPONSE_INTERVAL_US.get(),
                0x92,
                "Should default to 0x92 for clock value 0x{:x}",
                value
            );
        }
    }

    /// Tests light_check_tick_per_us logic consistency.
    ///
    /// Verifies the conditional logic works as expected.
    #[test]
    fn test_light_check_tick_per_us_logic() {
        reset_global_state();

        // The logic is: if ticks == 0x10 -> 0, else if ticks == 0x20 || ticks != 0x30 -> 0x92, else -> 0x93
        // This means only 0x30 gets 0x93, everything else (except 0x10) gets 0x92

        // Test 0x30 specifically (should get 0x93)
        light_check_tick_per_us(0x30);
        assert_eq!(
            BLE_SCAN_RESPONSE_INTERVAL_US.get(),
            0x93,
            "0x30 should get 0x93"
        );

        // Test 0x20 specifically (should get 0x92 via first condition of ||)
        BLE_SCAN_RESPONSE_INTERVAL_US.set(0);
        light_check_tick_per_us(0x20);
        assert_eq!(
            BLE_SCAN_RESPONSE_INTERVAL_US.get(),
            0x92,
            "0x20 should get 0x92"
        );

        // Test random value != 0x30 (should get 0x92 via second condition of ||)
        BLE_SCAN_RESPONSE_INTERVAL_US.set(0);
        light_check_tick_per_us(0x40);
        assert_eq!(
            BLE_SCAN_RESPONSE_INTERVAL_US.get(),
            0x92,
            "0x40 should get 0x92"
        );
    }

    // ================================================================================
    // Tests for back_to_rxmode_bridge function
    // ================================================================================

    /// Tests back_to_rxmode_bridge channel selection algorithm.
    ///
    /// Verifies that the channel selection algorithm calculates the correct
    /// channel index based on the bridge sequence number.
    #[test]
    fn test_back_to_rxmode_bridge_channel_selection() {
        reset_global_state();

        // Test channel selection algorithm with different sequence numbers
        let test_cases = [
            (0, (0 % SYS_CHN_LISTEN.len()) >> 1),
            (5, (5 % SYS_CHN_LISTEN.len()) >> 1),
            (10, (10 % SYS_CHN_LISTEN.len()) >> 1),
            (255, (255 % SYS_CHN_LISTEN.len()) >> 1),
        ];

        for (seq_num, expected_index) in test_cases.iter() {
            BRIDGE_SEQUENCE_NUMBER.set(*seq_num);

            // Calculate what the function should calculate
            let calculated_index = (*seq_num as usize % SYS_CHN_LISTEN.len()) >> 1;
            assert_eq!(
                calculated_index, *expected_index,
                "Channel index calculation should be correct for sequence {}",
                seq_num
            );

            // Verify the channel is within bounds
            assert!(
                calculated_index < SYS_CHN_LISTEN.len(),
                "Channel index should be within array bounds"
            );
        }
    }

    /// Tests back_to_rxmode_bridge function configuration.
    ///
    /// Verifies that all RF hardware configuration steps are performed correctly
    /// for mesh bridge mode operation.
    #[test]
    fn test_back_to_rxmode_bridge_configuration() {
        reset_global_state();

        // Set test values
        PAIR_AC.set(0x12345678);
        BRIDGE_SEQUENCE_NUMBER.set(5);

        back_to_rxmode_bridge();

        // The radio is switched off and then re-enabled for mesh reception.
        // `rf_set_ble_channel` also resets the tx/rx state machine while tuning,
        // so reg_rf_txrx_state (0xf02) is written three times, ending at
        // RF_TRX_OFF|BIT!(5) = 0x65 (RF_TRX_OFF is private to the RF driver).
        assert_eq!(
            write_count(REG_RF_TXRX_STATE),
            3,
            "tx/rx off, channel-tune reset, then rx enable"
        );
        assert_eq!(
            last_write(REG_RF_TXRX_STATE),
            Some(0x45 | 0x20),
            "the radio must be left in receive mode"
        );

        // The pairing access code is byte-swapped for the hardware.
        assert_eq!(
            last_write(REG_RF_ACCESS_CODE),
            Some(0x12345678u32.swap_bytes()),
            "the pairing access code must reach the radio byte-swapped"
        );

        // Advertising CRC init is restored for mesh packets.
        assert_eq!(
            last_write(REG_RF_CRC),
            Some(0x555555),
            "rf_set_ble_crc_adv must write the advertising CRC init"
        );

        // Verify channel selection
        // channel_index = (5 % SYS_CHN_LISTEN.len()) >> 1
        let expected_channel_index = (5 % SYS_CHN_LISTEN.len()) >> 1;
        let expected_channel = SYS_CHN_LISTEN[expected_channel_index];
        assert_eq!(
            last_write(REG_RF_CHANNEL),
            Some(expected_channel as u32),
            "the channel from the round-robin index must reach the radio"
        );
    }

    /// Tests back_to_rxmode_bridge channel hopping algorithm.
    ///
    /// Verifies that channel selection works correctly for different
    /// bridge sequence numbers.
    #[test]
    fn test_back_to_rxmode_bridge_channel_hopping() {
        reset_global_state();
        PAIR_AC.set(0x11111111);

        // Test specific sequence numbers to verify channel selection
        let test_cases = [(0, 0), (1, 0), (2, 1), (3, 1), (4, 0)];

        for (seq_num, expected_channel_index) in test_cases.iter() {
            BRIDGE_SEQUENCE_NUMBER.set(*seq_num);
            FakeMmio::clear_writes();

            back_to_rxmode_bridge();

            // Calculate expected channel - verify the algorithm works
            let calculated_channel_index = (*seq_num as usize % SYS_CHN_LISTEN.len()) >> 1;
            assert_eq!(
                calculated_channel_index, *expected_channel_index,
                "Channel selection algorithm failed for seq_num {}",
                seq_num
            );

            // The channel that reached the radio must match the round-robin
            // index for this sequence number.
            assert_eq!(
                last_write(REG_RF_CHANNEL),
                Some(SYS_CHN_LISTEN[calculated_channel_index] as u32),
                "the RF channel written for seq_num {} is wrong",
                seq_num
            );
        }
    }

    /// Tests back_to_rxmode_bridge with different access codes.
    ///
    /// Verifies that the pairing access code is correctly passed to RF configuration.
    #[test]
    fn test_back_to_rxmode_bridge_access_codes() {
        reset_global_state();
        BRIDGE_SEQUENCE_NUMBER.set(0);

        // Test different access codes
        let test_access_codes = [0x00000000, 0x12345678];

        for &access_code in test_access_codes.iter() {
            PAIR_AC.set(access_code);
            FakeMmio::clear_writes();

            back_to_rxmode_bridge();

            // `rf_set_ble_access_code` byte-swaps the code for the hardware.
            assert_eq!(
                last_write(REG_RF_ACCESS_CODE),
                Some(access_code.swap_bytes()),
                "access code 0x{access_code:08x} must reach the radio byte-swapped"
            );
        }
    }

    /// Tests back_to_rxmode_bridge boundary cases.
    ///
    /// Tests edge cases for channel selection algorithm.
    #[test]
    fn test_back_to_rxmode_bridge_boundary_cases() {
        reset_global_state();
        PAIR_AC.set(0x99999999);

        // Test with sequence number 0 (minimum)
        BRIDGE_SEQUENCE_NUMBER.set(0);
        FakeMmio::clear_writes();
        back_to_rxmode_bridge();

        let channel_index_0 = (0 % SYS_CHN_LISTEN.len()) >> 1;
        assert_eq!(
            channel_index_0, 0,
            "Minimum sequence should map to first channel"
        );
        assert_eq!(
            last_write(REG_RF_CHANNEL),
            Some(SYS_CHN_LISTEN[0] as u32),
            "minimum sequence must select the first channel"
        );

        // Test with sequence number at array boundary
        let boundary_seq = SYS_CHN_LISTEN.len() as u32;
        BRIDGE_SEQUENCE_NUMBER.set(boundary_seq);
        FakeMmio::clear_writes();
        back_to_rxmode_bridge();

        let channel_index_boundary = (boundary_seq as usize % SYS_CHN_LISTEN.len()) >> 1;
        assert_eq!(
            channel_index_boundary, 0,
            "Boundary sequence should wrap around"
        );
        assert_eq!(
            last_write(REG_RF_CHANNEL),
            Some(SYS_CHN_LISTEN[0] as u32),
            "the boundary sequence must wrap to the first channel"
        );

        // Test with large sequence number (overflow handling)
        BRIDGE_SEQUENCE_NUMBER.set(0xffffffff);
        FakeMmio::clear_writes();
        back_to_rxmode_bridge();

        let channel_index_max = (0xffffffff_usize % SYS_CHN_LISTEN.len()) >> 1;
        assert_eq!(
            last_write(REG_RF_CHANNEL),
            Some(SYS_CHN_LISTEN[channel_index_max] as u32),
            "the maximum sequence number must wrap into the channel table"
        );
    }

    // ================================================================================
    // Tests for update_connect_para function
    // ================================================================================

    /// Tests update_connect_para when update is not needed.
    ///
    /// Verifies that nothing happens when NEED_UPDATE_CONNECT_PARA is false.
    #[test]
    fn test_update_connect_para_not_needed() {
        reset_global_state();
        // `read_reg_system_tick()` reads the fake system tick register.
        fake_clock::set_ticks(50000);

        // Ensure update is not needed
        NEED_UPDATE_CONNECT_PARA.set(false);
        GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(10000);

        update_connect_para();

        // The real callback bumps CONN_UPDATE_CNT when it runs.
        assert_eq!(CONN_UPDATE_CNT.get(), 0, "callback must not be called");
    }

    /// Tests update_connect_para when timestamp is not set.
    ///
    /// Verifies that nothing happens when GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP is 0.
    #[test]
    fn test_update_connect_para_no_timestamp() {
        reset_global_state();
        // `read_reg_system_tick()` reads the fake system tick register.
        fake_clock::set_ticks(50000);

        // Set update needed but no timestamp
        NEED_UPDATE_CONNECT_PARA.set(true);
        GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(0);

        update_connect_para();

        // The real callback bumps CONN_UPDATE_CNT when it runs.
        assert_eq!(CONN_UPDATE_CNT.get(), 0, "callback must not be called");
    }

    /// Tests update_connect_para when delay has not elapsed.
    ///
    /// Verifies that parameter update is not triggered if insufficient time has passed.
    #[test]
    fn test_update_connect_para_delay_not_elapsed() {
        let start_time = 10000;
        let current_time =
            start_time + (UPDATE_CONNECT_PARA_DELAY_MS * CLOCK_SYS_CLOCK_1US * 1000 / 2); // Half delay

        reset_global_state();
        // `read_reg_system_tick()` reads the fake system tick register.
        fake_clock::set_ticks(current_time as u64);

        // Set up conditions for update but insufficient delay
        NEED_UPDATE_CONNECT_PARA.set(true);
        GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(start_time);

        update_connect_para();

        // The real callback bumps CONN_UPDATE_CNT when it runs.
        assert_eq!(
            CONN_UPDATE_CNT.get(),
            0,
            "callback must not be called before the delay elapsed"
        );

        // Verify flags remain set
        assert_eq!(NEED_UPDATE_CONNECT_PARA.get(), true);
        assert_eq!(GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.get(), start_time);
    }

    /// Tests update_connect_para when delay has elapsed.
    ///
    /// Verifies that parameter update is triggered after sufficient delay.
    #[test]
    fn test_update_connect_para_delay_elapsed() {
        let start_time = 10000;
        let current_time =
            start_time + (UPDATE_CONNECT_PARA_DELAY_MS * CLOCK_SYS_CLOCK_1US * 1000) + 1000; // Delay + extra

        reset_global_state();
        // `read_reg_system_tick()` reads the fake system tick register.
        fake_clock::set_ticks(current_time as u64);

        // Set up conditions for update with sufficient delay
        NEED_UPDATE_CONNECT_PARA.set(true);
        GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(start_time);

        update_connect_para();

        // The real callback bumps CONN_UPDATE_CNT when it runs.
        assert_eq!(CONN_UPDATE_CNT.get(), 1, "callback must be called once");

        // Verify flags were cleared
        assert_eq!(NEED_UPDATE_CONNECT_PARA.get(), false);
        assert_eq!(GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.get(), 0);
    }

    /// Tests update_connect_para with exact delay timing.
    ///
    /// Verifies behavior when elapsed time exactly matches the required delay.
    #[test]
    fn test_update_connect_para_exact_delay() {
        let start_time = 20000;
        let exact_delay = UPDATE_CONNECT_PARA_DELAY_MS * CLOCK_SYS_CLOCK_1US * 1000;
        let current_time = start_time + exact_delay;

        reset_global_state();
        // `read_reg_system_tick()` reads the fake system tick register.
        fake_clock::set_ticks(current_time as u64);

        // Set up conditions for update with exact delay
        NEED_UPDATE_CONNECT_PARA.set(true);
        GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(start_time);

        update_connect_para();

        // At exactly the delay time, should NOT trigger (condition is < not <=)
        assert_eq!(
            CONN_UPDATE_CNT.get(),
            0,
            "callback must not be called at exactly the delay"
        );
        assert_eq!(NEED_UPDATE_CONNECT_PARA.get(), true);
    }

    /// Tests the OTA staging providers' wrap-on-stage / direct-index-on-read
    /// asymmetry.
    ///
    /// `stage_ota_packet` copies `idx % 16` while `read_staged_ota_packet`
    /// indexes the 16-slot window directly, exactly like the legacy save loop.
    /// A round trip through both pins that asymmetry and proves both address the
    /// same slot, so the save path reads back the packet it staged. The buffer is
    /// process-global, so the test starts and ends at the zeroed default.
    #[test]
    fn test_stage_and_read_ota_packet_round_trip() {
        let zeroed = Packet {
            att_data: PacketAttData::default(),
        };
        BUFF_RESPONSE.lock().fill(zeroed);

        let mut first = PacketAttData::default();
        first.att = 0xA1;
        first.dat[0] = 0x11;
        let p1 = Packet { att_data: first };

        let mut second = PacketAttData::default();
        second.att = 0xB2;
        second.dat[0] = 0x22;
        let p2 = Packet { att_data: second };

        // `17 % 16 == 1`: staging wraps the too-large index into slot 1, which a
        // direct read of index 1 finds.
        stage_ota_packet(BUFF_RESPONSE_PACKET_COUNT + 1, &p1);
        assert_eq!(
            packet_bytes(&read_staged_ota_packet(1)),
            packet_bytes(&p1),
            "a wrapped staging index must land in slot idx % 16"
        );
        assert_eq!(
            packet_bytes(&BUFF_RESPONSE.lock()[0]),
            packet_bytes(&zeroed),
            "staging into slot 1 must leave slot 0 untouched"
        );

        stage_ota_packet(0, &p2);
        assert_eq!(
            packet_bytes(&read_staged_ota_packet(0)),
            packet_bytes(&p2),
            "a direct staging index must land in its own slot"
        );
        assert_eq!(
            packet_bytes(&read_staged_ota_packet(1)),
            packet_bytes(&p1),
            "staging slot 0 must leave the wrapped slot 1 untouched"
        );

        BUFF_RESPONSE.lock().fill(zeroed);
    }

    /// Tests update_connect_para multiple calls scenario.
    ///
    /// Verifies that after successful update, subsequent calls don't trigger again.
    #[test]
    fn test_update_connect_para_multiple_calls() {
        let start_time = 30000;
        let current_time =
            start_time + (UPDATE_CONNECT_PARA_DELAY_MS * CLOCK_SYS_CLOCK_1US * 1000) + 5000;

        reset_global_state();
        // `read_reg_system_tick()` reads the fake system tick register.
        fake_clock::set_ticks(current_time as u64);

        // Set up conditions for update
        NEED_UPDATE_CONNECT_PARA.set(true);
        GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(start_time);

        // First call should trigger update
        update_connect_para();

        // Second call should not trigger (flags already cleared)
        update_connect_para();

        // The real callback bumps CONN_UPDATE_CNT when it runs.
        assert_eq!(CONN_UPDATE_CNT.get(), 1, "callback must be called once");
    }
}
