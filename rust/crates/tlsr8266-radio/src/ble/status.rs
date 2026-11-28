//! # Device Status and Reporting Management
//!
//! This module implements the status reporting and bridging subsystem for the TLSR8266 mesh
//! lighting system. It coordinates status reporting between BLE peripheral connections and
//! the mesh network, implementing bridge algorithms and response buffering for reliable
//! data transfer between different network protocols.
//!
//! ## Bridge Architecture
//!
//! The status management system acts as a protocol bridge between two networks:
//!
//! ### BLE Peripheral Network
//! - **Connection-Oriented**: Maintains persistent connections with master devices (smartphones)
//! - **Request-Response Pattern**: Supports synchronous status queries and updates
//! - **Low Latency**: Optimized for real-time user interactions
//! - **Single Master**: Typically connected to one smartphone/gateway at a time
//!
//! ### Mesh Network
//! - **Connectionless**: Uses advertising-based communication without persistent connections
//! - **Flooding Protocol**: Status updates propagate through the entire network
//! - **High Reliability**: Multiple redundant paths ensure message delivery
//! - **Multi-Node**: Supports dozens of devices in a single network
//!
//! ## Status Reporting Algorithms
//!
//! ### Bridge Command Processing
//! The system implements a sophisticated command bridging algorithm:
//! 1. **Command Reception**: Receives commands from BLE peripheral interface
//! 2. **Protocol Translation**: Converts BLE commands to mesh protocol format
//! 3. **Timing Optimization**: Applies adaptive delays to prevent mesh congestion
//! 4. **Transmission Scheduling**: Queues commands for mesh transmission
//!
//! ### Response Buffering
//! Status responses are buffered to handle timing differences between protocols:
//! - **Circular Buffer**: Efficient storage for multiple pending responses
//! - **Timeout Management**: Automatic cleanup of stale responses
//! - **Priority Handling**: Ensures critical status updates are not lost
//!
//! ### Online Status Broadcasting
//! Regular status broadcasts maintain network topology awareness:
//! - **Periodic Transmission**: Scheduled status broadcasts at regular intervals
//! - **Adaptive Frequency**: Transmission rate adapts to network conditions
//! - **Channel Hopping**: Uses multiple RF channels for improved reliability
//!
//! ## Timing Coordination
//!
//! The bridge manages timing between different protocol domains:
//!
//! ```text
//! BLE Protocol Domain     Bridge Layer        Mesh Protocol Domain
//! ┌─────────────────┐    ┌─────────────┐    ┌─────────────────┐
//! │ Fast Response   │◄───┤ Buffering & │───►│ Periodic Flood  │
//! │ (ms latency)    │    │ Translation │    │ (seconds)       │
//! └─────────────────┘    └─────────────┘    └─────────────────┘
//!                            ▲     ▲
//!                     Status │     │ Commands
//!                   Response │     │
//!                            ▼     ▼
//!                    ┌─────────────────┐
//!                    │ Local Processing│
//!                    │ & State Mgmt    │
//!                    └─────────────────┘
//! ```
//!
//! ## Congestion Control
//!
//! The system implements mesh congestion control through:
//! - **Adaptive Delays**: Dynamic adjustment of transmission timing
//! - **Random Backoff**: Collision avoidance in dense networks
//! - **Load Balancing**: Distribution of traffic across multiple channels

use core::cmp::min;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use crate::ble::connection::{BRIDGE_COMMAND_TIMESTAMP, SLAVE_DATA_VALID};
use crate::ble::pair::PAIR_AC;
use crate::coex::RadioCoexistence;
use crate::rf::drv_8266::*;
use crate::types::*;
use tlsr8266_hal::common::sync::SimplifyLS;
use tlsr8266_hal::mcu::clock::{sleep_us, CLOCK_SYS_CLOCK_1US};
use tlsr8266_pac::read_reg_system_tick;

use crate::mesh_ll::node_status::{
    mesh_send_online_status, MESH_LISTEN_INTERVAL_US, PKT_LIGHT_DATA,
    STATUS_MESSAGE_SEQUENCE_NUMBER,
};
use crate::mesh_ll::packet_processing::rf_link_is_notify_req;

/// Status indicating slave is busy reading mesh node states.
/// Prevents concurrent status reading operations.
///
/// Visibility is `pub(crate)`: the facade module is the only entry point from
/// outside this crate, and the crate boundary is the fence C-10 established.
pub(crate) static SLAVE_READ_STATUS_BUSY: AtomicU8 = AtomicU8::new(0);

/// Flag indicating unicast status reading is active.
/// When true, status reads are directed to specific devices rather than broadcast.
pub(crate) static DEVICE_STATUS_READ_UNICAST_MODE: AtomicBool = AtomicBool::new(false);

/// Status tick counter for device state machine.
/// Used for timing device status operations and state transitions.
pub(crate) static DEVICE_STATUS_TICK_COUNTER: AtomicU8 = AtomicU8::new(0);

/// Notification request mask index for tracking pending requests.
/// Used to track which devices have pending notification requests.
pub(crate) static NOTIFICATION_REQUEST_MASK_INDEX: AtomicU8 = AtomicU8::new(0);

/// Timestamp when device became busy reading status information.
/// Used to detect and timeout stuck status reading operations.
pub(crate) static DEVICE_STATUS_READ_BUSY_TIMESTAMP: AtomicU32 = AtomicU32::new(0);

/// Initializes BLE slave status reporting parameters to default state.
///
/// This function resets all status reporting buffers and sequence numbers to
/// prepare for a new status reporting session. It ensures clean state when
/// starting status operations or recovering from error conditions.
///
/// # Initialization Process
/// - **Buffer Pointers**: Resets read/write pointers to synchronize buffer access
/// - **Sequence Numbers**: Clears message sequence tracking for fresh start
/// - **State Cleanup**: Ensures no residual state from previous operations
///
/// # Use Cases
/// This function is called during:
/// - Initial system startup
/// - Connection establishment
/// - Error recovery procedures
/// - Status reporting session reset
fn rf_link_slave_read_status_params_reset() {
    // Clear status message sequence number for fresh tracking
    *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() = [0; 3];
}

pub fn rf_link_slave_read_status_par_init<C: RadioCoexistence>() -> bool {
    let started = C::start_status_window();
    rf_link_slave_read_status_params_reset();
    started
}

/// Abort a status read without changing the platform lifecycle lock.
///
/// OTA begin calls this radio-only core while it already holds the platform
/// transition lock, then claims OTA mode itself. Normal disconnect and timeout
/// paths use `rf_link_slave_read_status_stop`, which also stops the platform
/// status window.
pub fn rf_link_slave_read_status_abort() {
    // Clear status reporting busy flag to mark operation as inactive
    SLAVE_READ_STATUS_BUSY.set(0);

    // Disable unicast mode to return to default broadcast behavior
    DEVICE_STATUS_READ_UNICAST_MODE.set(false);

    // Invalidate any pending slave data to prevent stale responses
    SLAVE_DATA_VALID.set(0);

    // Reinitialize status reporting parameters for clean state.
    rf_link_slave_read_status_params_reset();
}

/// Terminates BLE slave status reporting and stops the platform status window.
/// This is the normal disconnect and timeout cleanup path.
pub fn rf_link_slave_read_status_stop<C: RadioCoexistence>() {
    rf_link_slave_read_status_abort();
    C::stop_status_window();

    // Keep this explicit for normal stop callers and tests that reset sequence
    // state independently of the radio-only abort path.
    rf_link_slave_read_status_params_reset();
}

/// Processes bridge commands with adaptive timing and mesh transmission.
///
/// This function implements the core bridge command processing algorithm that
/// translates BLE commands into mesh network messages. It applies sophisticated
/// timing optimization to prevent mesh network congestion while ensuring reliable
/// command delivery.
///
/// # Bridge Processing Algorithm
///
/// The function operates on a validated command queue:
/// 1. **Data Validation**: Checks if valid command data is pending transmission
/// 2. **Timing Calculation**: Computes adaptive relay delay based on processing time
/// 3. **Protocol Conversion**: Marks packet for mesh transmission (sets bit 7)
/// 4. **Congestion Control**: Applies calculated delays to prevent mesh flooding
/// 5. **Message Queuing**: Submits command to mesh transmission manager
///
/// # Adaptive Timing Algorithm
///
/// The timing calculation implements congestion control:
/// ```text
/// relay_time = min(((current_time - bridge_cmd_time) / 1µs + 500) >> 10, 255)
/// ```
///
/// This formula:
/// - Measures processing delay since command reception
/// - Adds 500µs base delay for stability
/// - Divides by 1024 (>> 10) to convert to appropriate time units
/// - Clamps to maximum value of 255 to prevent overflow
///
/// # Command State Management
///
/// The function manages a countdown-based state machine:
/// - **SLAVE_DATA_VALID > 0**: Command is pending transmission
/// - **Countdown Decrement**: Reduces counter each call until transmission
/// - **Zero State**: Special handling for immediate transmission requirements
/// - **Negative Values**: Indicates command timeout or error conditions
///
/// # Mesh Integration
///
/// Commands are formatted for mesh transmission by:
/// - Setting mesh transmission flag (bit 7 in packet type)
/// - Preserving original command structure and parameters
/// - Adding timing information for relay optimization
/// - Queuing through mesh manager for reliable delivery
///
/// # Parameters
/// * `bridge_cmd_time` - Timestamp when the bridge command was initially received
///
/// # Side Effects
/// * Modifies packet headers for mesh transmission
/// * Updates command timing fields
/// * Queues messages in mesh transmission system
/// * Decrements pending command counters
pub fn app_bridge_cmd_handle(bridge_cmd_time: u32) {
    let mut pkt_light_data = PKT_LIGHT_DATA.lock();

    // Check if valid command data is pending transmission
    if SLAVE_DATA_VALID.get() != 0 {
        // Decrement the valid data counter (countdown to transmission)
        SLAVE_DATA_VALID.dec();

        // Process command if not in final countdown or if conditions allow transmission
        if SLAVE_DATA_VALID.get() == 0 {
            // Special case: immediate transmission required
        } else if SLAVE_READ_STATUS_BUSY.get() == 0 {
            // Only process if not busy and we have valid data remaining
            // Mark packet for mesh transmission (set mesh transmission flag)
            pkt_light_data.head_mut()._type |= tlsr8266_pac::BIT!(7);

            // Apply adaptive timing for notify request commands
            if rf_link_is_notify_req(pkt_light_data.att_cmd().value.val[0] & 0x3f) {
                // Calculate relay timing based on processing delay
                // Formula: (elapsed_time_us + 500) / 1024, clamped to 255
                let relay_time = min(
                    (((read_reg_system_tick() - bridge_cmd_time) / CLOCK_SYS_CLOCK_1US) + 500)
                        >> 10,
                    0xff,
                );

                // Store timing information in packet for mesh relay optimization
                pkt_light_data.att_cmd_mut().value.val[PKT_VAL_MAC_APP_RELAY_TIMING] =
                    relay_time as u8;
            }

            // Queue the command for mesh transmission (no delay, no retransmit)
            crate::queues::enqueue_mesh_tx(&pkt_light_data, 0, 0);
        }
    }
}

/// Coordinates bridge packet transmission with periodic status reporting.
///
/// This function implements the main bridge transmission coordinator that manages
/// both regular status broadcasts and command bridging operations. It applies
/// RF hardware configuration, timing coordination, and adaptive scheduling to
/// maintain reliable mesh network operation.
///
/// # Bridge Transmission Algorithm
///
/// The function operates in multiple phases:
///
/// 1. **RF Hardware Setup**: Configures radio for mesh operation
/// 2. **Timing Synchronization**: Waits for hardware stabilization
/// 3. **Protocol Configuration**: Sets up mesh-specific RF parameters
/// 4. **Status Broadcast Management**: Handles periodic status transmission
/// 5. **Command Processing**: Processes any pending bridge commands
///
/// # Hardware Configuration Sequence
///
/// The RF setup follows a specific sequence for reliable operation:
/// - **TX/RX Disable**: Stops current operations to prevent interference
/// - **Stabilization Delay**: 100µs wait for hardware state transition
/// - **Access Code Setup**: Configures mesh network identification
/// - **CRC Configuration**: Sets up packet integrity checking
///
/// # Periodic Status Broadcasting
///
/// Status broadcasts maintain network awareness through:
/// - **Interval Calculation**: Based on listen interval and scaling factors
/// - **Compensation Timing**: Accounts for transmission and processing delays
/// - **Adaptive Scheduling**: Adjusts timing based on network conditions
///
/// The broadcast interval formula:
/// ```text
/// broadcast_interval = MESH_LISTEN_INTERVAL_US * ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL
///                     - ONLINE_STATUS_COMP * 1000µs
/// ```
///
/// # Command Bridge Integration
///
/// Pending commands are processed after status management by calling the
/// bridge command handler with the stored command timestamp. This ensures
/// proper timing coordination between status broadcasts and command relay.
///
/// # RF Channel Management
///
/// The function configures the radio for mesh operation but does not
/// explicitly set the channel, relying on the mesh manager to handle
/// channel selection and hopping as needed.
///
/// # Side Effects
/// * Modifies RF hardware registers for mesh operation
/// * May trigger status broadcast transmission
/// * Processes pending bridge commands
/// * Updates transmission timing tracking
pub fn tx_packet_bridge() {
    // Static variable to track last bridge report transmission time
    static TICK_BRIDGE_REPORT: AtomicU32 = AtomicU32::new(0);

    // Disable RF transmit/receive operations for hardware reconfiguration
    rf_set_tx_rx_off();

    // Wait for hardware stabilization (100µs settling time)
    sleep_us(100);

    // Configure RF hardware for mesh network operation
    rf_set_ble_access_code(PAIR_AC.get()); // Set mesh network access code
    rf_set_ble_crc_adv(); // Configure CRC for advertising packets

    // Check if periodic status broadcast is due
    let tick = read_reg_system_tick();
    let broadcast_interval = (MESH_LISTEN_INTERVAL_US.get()
        * ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL as u32)
        .saturating_sub(ONLINE_STATUS_COMP * CLOCK_SYS_CLOCK_1US * 1000);

    if broadcast_interval < tick.saturating_sub(TICK_BRIDGE_REPORT.load(Ordering::Relaxed)) {
        // Update last broadcast time and trigger status transmission
        TICK_BRIDGE_REPORT.store(tick, Ordering::Relaxed);
        mesh_send_online_status();
    }

    // Process any pending bridge commands with stored command timestamp
    app_bridge_cmd_handle(BRIDGE_COMMAND_TIMESTAMP.get());
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use tlsr8266_hal::mcu::clock::fake_clock;
    use tlsr8266_pac::fake_mmio::{FakeMmio, RegWrite};
    use tlsr8266_pac::{write_reg_system_tick, REG_BASE_ADDR};

    /// Datasheet offsets of the RF registers `tx_packet_bridge` drives.
    const REG_RF_ACCESS_CODE: u32 = 0x408;
    const REG_RF_RX_MODE: u32 = 0x428;
    const REG_RF_CRC: u32 = 0x44c;
    const REG_RF_TXRX_STATE: u32 = 0xf02;
    const REG_RF_MODE: u32 = 0xf16;

    /// Reset the PAC/HAL host fakes so a test never inherits another test's
    /// register bank, analog bank, clock or AES transport (the suite runs
    /// `--test-threads=1`, so all of these are process-wide singletons).
    fn reset_host_fakes() {
        FakeMmio::reset();
        tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog::reset();
        tlsr8266_hal::mcu::clock::fake_clock::reset();
        tlsr8266_hal::mcu::crypto::fake_crypto::reset();
    }

    fn setup_test_state() {
        crate::coex::TestRadioCoexistence::reset();
        reset_host_fakes();
        // Reset all status management state
        crate::coex::TestRadioCoexistence::set_write_pointer(0);
        crate::coex::TestRadioCoexistence::set_read_pointer(0);
        STATUS_MESSAGE_SEQUENCE_NUMBER.lock().fill(0);
        SLAVE_READ_STATUS_BUSY.set(0);
        DEVICE_STATUS_READ_UNICAST_MODE.set(false);
        SLAVE_DATA_VALID.set(0);
        BRIDGE_COMMAND_TIMESTAMP.set(1000);
        PAIR_AC.set(0x12345678);
        MESH_LISTEN_INTERVAL_US.set(100000); // 100ms

        // Global queues outlive every test in this process.
        crate::queues::reset_for_test();

        // Clear packet data to ensure clean state
        let mut packet_data = PKT_LIGHT_DATA.lock();
        packet_data.head_mut()._type = 0x00; // Clear all flags including mesh flag
        drop(packet_data);
    }

    /// Assert the real RF setup sequence reached the fake register bank: one
    /// write per renamed accessor, with the byte-swapped access code and the
    /// advertising CRC value checked exactly.
    fn assert_rf_setup_writes(access_code: u32) {
        let writes = FakeMmio::writes();
        let wrote = |offset: u32| writes.iter().any(|w| w.addr == (offset | REG_BASE_ADDR));

        assert!(wrote(REG_RF_MODE), "rf_mode not written: {writes:?}");
        assert!(wrote(REG_RF_RX_MODE), "rf_rx_mode not written: {writes:?}");
        assert!(
            wrote(REG_RF_TXRX_STATE),
            "rf_txrx_state not written: {writes:?}"
        );
        assert!(
            writes.contains(&RegWrite {
                addr: REG_RF_ACCESS_CODE | REG_BASE_ADDR,
                width: 4,
                value: access_code.swap_bytes(),
            }),
            "byte-swapped access code not written: {writes:?}"
        );
        assert!(
            writes.contains(&RegWrite {
                addr: REG_RF_CRC | REG_BASE_ADDR,
                width: 4,
                value: 0x555555,
            }),
            "advertising CRC not written: {writes:?}"
        );
    }

    /// Drive `mesh_send_online_status`'s interval limiter to a known baseline:
    /// one request at `tick` leaves `LAST_STATUS_TIME` at `tick`, so the next
    /// request at the same tick is always rate limited. Queues are cleared
    /// afterwards so only the call under test is observable.
    fn normalize_status_interval(tick: u32) {
        fake_clock::set_ticks(tick as u64);
        mesh_send_online_status();
        crate::queues::reset_for_test();
    }

    /// One queued bridge command, ready for `app_bridge_cmd_handle` to transmit
    /// on the next countdown step.
    fn queue_one_bridge_command(opcode: u8) {
        let mut packet_data = PKT_LIGHT_DATA.lock();
        packet_data.head_mut()._type = 0x00;
        packet_data.att_cmd_mut().value.val[0] = opcode;
        packet_data.att_cmd_mut().value.val[PKT_VAL_MAC_APP_RELAY_TIMING] = 0;
        drop(packet_data);

        SLAVE_DATA_VALID.set(2);
        SLAVE_READ_STATUS_BUSY.set(0);
    }

    #[test]
    fn test_rf_link_slave_read_status_par_init() {
        // Setup initial non-zero state
        crate::coex::TestRadioCoexistence::set_write_pointer(5);
        crate::coex::TestRadioCoexistence::set_read_pointer(3);
        *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() = [1, 2, 3];

        // Execute function
        rf_link_slave_read_status_par_init::<crate::coex::TestRadioCoexistence>();

        // Verify state is reset
        assert_eq!(crate::coex::TestRadioCoexistence::write_pointer(), 0);
        assert_eq!(crate::coex::TestRadioCoexistence::read_pointer(), 0);
        assert_eq!(*STATUS_MESSAGE_SEQUENCE_NUMBER.lock(), [0, 0, 0]);
    }

    #[test]
    fn test_rf_link_slave_read_status_abort_is_radio_only() {
        SLAVE_READ_STATUS_BUSY.set(1);
        DEVICE_STATUS_READ_UNICAST_MODE.set(true);
        SLAVE_DATA_VALID.set(5);
        crate::coex::TestRadioCoexistence::set_write_pointer(7);
        crate::coex::TestRadioCoexistence::set_read_pointer(4);
        *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() = [3, 2, 1];

        rf_link_slave_read_status_abort();

        assert_eq!(SLAVE_READ_STATUS_BUSY.get(), 0);
        assert!(!DEVICE_STATUS_READ_UNICAST_MODE.get());
        assert_eq!(SLAVE_DATA_VALID.get(), 0);
        assert_eq!(*STATUS_MESSAGE_SEQUENCE_NUMBER.lock(), [0, 0, 0]);
        assert_eq!(crate::coex::TestRadioCoexistence::write_pointer(), 7);
        assert_eq!(crate::coex::TestRadioCoexistence::read_pointer(), 4);
    }

    #[test]
    fn test_rf_link_slave_read_status_stop() {
        // Setup initial state with active status reporting
        SLAVE_READ_STATUS_BUSY.set(1);
        DEVICE_STATUS_READ_UNICAST_MODE.set(true);
        SLAVE_DATA_VALID.set(5);
        crate::coex::TestRadioCoexistence::set_write_pointer(7);
        crate::coex::TestRadioCoexistence::set_read_pointer(4);
        *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() = [3, 2, 1];

        // Execute function
        rf_link_slave_read_status_stop::<crate::coex::TestRadioCoexistence>();

        // Verify cleanup occurred
        assert_eq!(SLAVE_READ_STATUS_BUSY.get(), 0);
        assert!(!DEVICE_STATUS_READ_UNICAST_MODE.get());
        assert_eq!(SLAVE_DATA_VALID.get(), 0);

        // The real parameter reset ran as the last step of the shutdown: the
        // buffer pointers and sequence numbers it owns are zeroed.
        assert_eq!(crate::coex::TestRadioCoexistence::write_pointer(), 0);
        assert_eq!(crate::coex::TestRadioCoexistence::read_pointer(), 0);
        assert_eq!(*STATUS_MESSAGE_SEQUENCE_NUMBER.lock(), [0, 0, 0]);
    }

    #[test]
    fn test_app_bridge_cmd_handle_no_valid_data() {
        setup_test_state();
        SLAVE_DATA_VALID.set(0); // No valid data

        // Function should return early without doing anything
        app_bridge_cmd_handle(1000);

        // Nothing may be transmitted for a zero countdown.
        assert_eq!(crate::queues::MESH_TX_QUEUE.lock().len(), 0);
    }

    #[test]
    fn test_app_bridge_cmd_handle_countdown_decrements() {
        setup_test_state();
        SLAVE_DATA_VALID.set(3); // Set countdown value

        // Execute function
        app_bridge_cmd_handle(1000);

        // Verify countdown decremented
        assert_eq!(SLAVE_DATA_VALID.get(), 2);
    }

    #[test]
    fn test_app_bridge_cmd_handle_immediate_transmission() {
        setup_test_state();
        let mut packet_data = PKT_LIGHT_DATA.lock();

        // Set up test packet data
        packet_data.head_mut()._type = 0x00; // Clear mesh flag
        packet_data.att_cmd_mut().value.val[0] = 0x10; // Test command
        drop(packet_data); // Release lock

        SLAVE_DATA_VALID.set(1); // Will become 0 after decrement

        // Execute function - immediate transmission case
        app_bridge_cmd_handle(1000);

        // Verify countdown reached zero
        assert_eq!(SLAVE_DATA_VALID.get(), 0);

        // The immediate-transmission branch returns before queueing anything.
        assert_eq!(crate::queues::MESH_TX_QUEUE.lock().len(), 0);
    }

    #[test]
    fn test_app_bridge_cmd_handle_notify_request_timing() {
        setup_test_state();
        // A real notify-request opcode, so the real rf_link_is_notify_req has to
        // recognise it.
        queue_one_bridge_command(LGT_CMD_LIGHT_READ_STATUS);

        // Mock system tick for timing calculation
        // To get timing value 1: ((tick_diff / 32) + 500) >> 10 = 1
        // So: (tick_diff / 32) + 500 = 1024
        // tick_diff / 32 = 524
        // tick_diff = 524 * 32 = 16768
        write_reg_system_tick(1000 + 16768); // 16768 ticks elapsed since bridge_cmd_time=1000

        // Execute function
        app_bridge_cmd_handle(1000);

        // Verify packet was modified for mesh transmission
        let packet_data = PKT_LIGHT_DATA.lock();
        assert_ne!(packet_data.head()._type & 0x80, 0); // Mesh flag should be set
                                                        // Verify timing calculation: (16768/32 + 500) / 1024 = (524 + 500) / 1024 = 1024/1024 = 1
        assert_eq!(
            packet_data.att_cmd().value.val[PKT_VAL_MAC_APP_RELAY_TIMING],
            1
        );
        drop(packet_data);

        // The relay-timed command was queued for mesh transmission.
        assert_eq!(crate::queues::MESH_TX_QUEUE.lock().len(), 1);
    }

    #[test]
    fn test_app_bridge_cmd_handle_non_notify_request() {
        setup_test_state();
        // 0x30 is a genuine non-notify command, so no timing branch may run.
        queue_one_bridge_command(LGT_CMD_SET_LIGHT);

        // A tick that would produce timing 1 if the notify branch wrongly fired.
        write_reg_system_tick(1000 + 16768);

        // Execute function
        app_bridge_cmd_handle(1000);

        // Verify packet mesh flag was set but no timing applied
        let packet_data = PKT_LIGHT_DATA.lock();
        assert_ne!(packet_data.head()._type & 0x80, 0); // Mesh flag should be set
        assert_eq!(
            packet_data.att_cmd().value.val[PKT_VAL_MAC_APP_RELAY_TIMING],
            0
        );
        drop(packet_data);

        assert_eq!(crate::queues::MESH_TX_QUEUE.lock().len(), 1);
    }

    #[test]
    fn test_app_bridge_cmd_handle_busy_state_blocks_transmission() {
        setup_test_state();
        SLAVE_DATA_VALID.set(2);
        SLAVE_READ_STATUS_BUSY.set(1); // Busy state

        // Execute function
        app_bridge_cmd_handle(1000);

        // Verify countdown decremented but no processing occurred
        assert_eq!(SLAVE_DATA_VALID.get(), 1);

        // Verify packet was not modified (mesh flag should remain clear)
        let packet_data = PKT_LIGHT_DATA.lock();
        assert_eq!(packet_data.head()._type & 0x80, 0);
        drop(packet_data);

        assert_eq!(crate::queues::MESH_TX_QUEUE.lock().len(), 0);
    }

    #[test]
    fn test_app_bridge_cmd_handle_zero_data_early_exit() {
        setup_test_state();
        SLAVE_DATA_VALID.set(0); // No valid data, should exit early

        // Execute function
        app_bridge_cmd_handle(1000);

        // Verify value unchanged since function exits early for zero
        assert_eq!(SLAVE_DATA_VALID.get(), 0);

        // Verify packet was not modified
        let packet_data = PKT_LIGHT_DATA.lock();
        assert_eq!(packet_data.head()._type & 0x80, 0);
        drop(packet_data);

        assert_eq!(crate::queues::MESH_TX_QUEUE.lock().len(), 0);
    }

    #[test]
    fn test_tx_packet_bridge_basic_setup() {
        setup_test_state();
        normalize_status_interval(0);
        // A tick just after the last broadcast keeps the status broadcast out of
        // this test, leaving only the bridge command in the queue.
        fake_clock::set_ticks(1000);
        queue_one_bridge_command(LGT_CMD_SET_LIGHT);

        // Execute function
        tx_packet_bridge();

        // Verify the real RF setup sequence through the fake register bank.
        assert_rf_setup_writes(PAIR_AC.get());
        // The 100us RF settling delay is observable through the fake clock
        assert_eq!(fake_clock::sleeps(), vec![100]);

        // The bridge handler ran with the stored command timestamp; its command
        // is the only packet in the queue because the status interval was reset.
        let queue = crate::queues::MESH_TX_QUEUE.lock();
        assert_eq!(queue.len(), 1);
        let pkt_type = queue[0].pkt.head()._type;
        assert_ne!(pkt_type & 0x80, 0, "bridge command marked for mesh TX");
        drop(queue);
        assert_eq!(SLAVE_DATA_VALID.get(), 1);
    }

    #[test]
    fn test_tx_packet_bridge_status_broadcast_triggered() {
        setup_test_state();
        normalize_status_interval(0);

        // Set up timing to trigger status broadcast
        // broadcast_interval = (100000 * 8).saturating_sub(3 * 32 * 1000) = 704000
        // and the status rate limit is 200 ms (6.4 M ticks), so 30 M ticks after
        // the reset baseline is well past both thresholds.
        let current_tick = 30_000_000u32;
        fake_clock::set_ticks(current_tick as u64);
        queue_one_bridge_command(LGT_CMD_SET_LIGHT);

        // Execute function
        tx_packet_bridge();

        // Status should be sent because timing interval is met; the bridge
        // command follows it in the same queue.
        let queue = crate::queues::MESH_TX_QUEUE.lock();
        assert_eq!(queue.len(), 2, "status advertisement + bridge command");

        // First entry: the online-status advertisement built by
        // mesh_send_online_status (advertising type, status channel, signature).
        let adv_type = queue[0].pkt.head()._type;
        let adv_chan = queue[0].pkt.head().chan_id;
        assert_eq!(adv_type, 2, "status advertisement packet type");
        assert_eq!(adv_chan, 0xffff, "status advertisement channel");
        // The signature sits at packet bytes 24..28, i.e. val[17..21].
        let adv_val = queue[0].pkt.att_write().value.val;
        assert_eq!(
            &adv_val[17..21],
            &[0xa5, 0xa5, 0xa5, 0xa5],
            "status advertisement signature"
        );

        // Second entry: the bridge command.
        let cmd_type = queue[1].pkt.head()._type;
        assert_ne!(cmd_type & 0x80, 0, "bridge command marked for mesh TX");
    }

    #[test]
    fn test_tx_packet_bridge_status_broadcast_not_due() {
        setup_test_state();
        normalize_status_interval(0);

        // Set up timing where status broadcast is not due yet: 1000 ticks after
        // the rate-limit baseline is far inside both intervals.
        fake_clock::set_ticks(1000);
        queue_one_bridge_command(LGT_CMD_SET_LIGHT);

        // Execute function
        tx_packet_bridge();

        // Only the bridge command is queued; no status advertisement was built.
        let queue = crate::queues::MESH_TX_QUEUE.lock();
        assert_eq!(queue.len(), 1);
        let pkt_type = queue[0].pkt.head()._type;
        assert_ne!(pkt_type & 0x80, 0, "bridge command marked for mesh TX");
        assert_ne!(
            pkt_type, 2,
            "the single packet is not a status advertisement"
        );
    }

    #[test]
    fn test_tx_packet_bridge_function_calls() {
        setup_test_state();
        normalize_status_interval(0);

        // 20 M ticks: past the 704000-tick broadcast interval and the 200 ms
        // status rate limit, so every stage of tx_packet_bridge takes its real
        // path. (The broadcast timestamp is a function-local static, so the due
        // tests use ascending ticks, matching this module's test order.)
        let current_tick = 20_000_000u32;
        fake_clock::set_ticks(current_tick as u64);
        queue_one_bridge_command(LGT_CMD_LIGHT_READ_STATUS);

        // Execute function
        tx_packet_bridge();

        // Every RF setup step ran for real...
        assert_rf_setup_writes(PAIR_AC.get());
        // ...the 100us RF settling delay was requested...
        assert_eq!(fake_clock::sleeps(), vec![100]);

        let queue = crate::queues::MESH_TX_QUEUE.lock();
        // ...the online status was queued...
        assert_eq!(queue.len(), 2, "status advertisement + bridge command");
        let adv_type = queue[0].pkt.head()._type;
        assert_eq!(adv_type, 2, "status advertisement packet type");

        // ...and the bridge handler ran with BRIDGE_COMMAND_TIMESTAMP, which the
        // clamped relay timing derived from the seeded tick proves.
        let expected_timing = min(
            (((current_tick - BRIDGE_COMMAND_TIMESTAMP.get()) / CLOCK_SYS_CLOCK_1US) + 500) >> 10,
            0xff,
        );
        assert_eq!(expected_timing, 255, "30 M ticks clamp the relay timing");
        assert_eq!(
            queue[1].pkt.att_cmd().value.val[PKT_VAL_MAC_APP_RELAY_TIMING],
            expected_timing as u8
        );
    }

    #[test]
    fn test_app_bridge_cmd_handle_timing_calculation_edge_cases() {
        setup_test_state();
        queue_one_bridge_command(LGT_CMD_LIGHT_READ_STATUS);

        // Test maximum timing value clamping
        let bridge_cmd_time = 1000;
        let large_tick = bridge_cmd_time + (0xff << 10) * CLOCK_SYS_CLOCK_1US + 1000; // Should clamp to 255

        write_reg_system_tick(large_tick);

        app_bridge_cmd_handle(bridge_cmd_time);

        // Verify timing was clamped to maximum (255)
        let packet_data = PKT_LIGHT_DATA.lock();
        assert_eq!(
            packet_data.att_cmd().value.val[PKT_VAL_MAC_APP_RELAY_TIMING],
            255
        );
        drop(packet_data);

        assert_eq!(crate::queues::MESH_TX_QUEUE.lock().len(), 1);
    }
}
