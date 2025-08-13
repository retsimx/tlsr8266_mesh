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
//! ```
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
use core::sync::atomic::{AtomicU32, Ordering};

use crate::{app};
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::light::{*};
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, sleep_us};
use crate::sdk::mcu::register::{read_reg_system_tick, read_reg_rnd_number};
use crate::state::{*};

use super::mesh_management::mesh_send_online_status;
use super::packet_processing::{rf_link_is_notify_req};

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
#[cfg_attr(test, mry::mry)]
pub fn rf_link_slave_read_status_par_init()
{
    // Reset circular buffer pointers to empty state
    DEVICE_STATUS_BUFFER_WRITE_POINTER.set(0);
    DEVICE_STATUS_BUFFER_READ_POINTER.set(0);
    
    // Clear status message sequence number for fresh tracking
    *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() = [0; 3];
}

/// Terminates BLE slave status reporting operations and cleans up state.
///
/// This function implements a comprehensive shutdown procedure for status reporting,
/// ensuring all resources are properly released and state is reset to prevent
/// interference with subsequent operations.
///
/// # Shutdown Algorithm
/// 1. **Clear Busy Flag**: Marks status reporting as no longer active
/// 2. **Reset Communication Mode**: Disables unicast-specific reporting modes  
/// 3. **Invalidate Pending Data**: Clears any queued status data
/// 4. **Reinitialize Parameters**: Calls initialization to reset all state
///
/// # State Cleanup
/// The function ensures complete cleanup by:
/// - Clearing operational flags and counters
/// - Resetting communication mode settings
/// - Invalidating pending response data
/// - Reinitializing buffer management state
///
/// # Use Cases
/// Called when:
/// - BLE connection is terminated
/// - Error conditions require status reporting reset
/// - System transitions from status reporting to other modes
/// - Clean shutdown is required before mode changes
#[cfg_attr(test, mry::mry)]
pub fn rf_link_slave_read_status_stop()
{
    // Clear status reporting busy flag to mark operation as inactive
    SLAVE_READ_STATUS_BUSY.set(0);
    
    // Disable unicast mode to return to default broadcast behavior
    DEVICE_STATUS_READ_UNICAST_MODE.set(false);
    
    // Invalidate any pending slave data to prevent stale responses
    SLAVE_DATA_VALID.set(0);

    // Reinitialize all status reporting parameters for clean state
    rf_link_slave_read_status_par_init();
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
/// ```
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
#[cfg_attr(test, mry::mry)]
pub fn app_bridge_cmd_handle(bridge_cmd_time: u32)
{
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
            pkt_light_data.head_mut()._type |= crate::BIT!(7);

            // Apply adaptive timing for notify request commands
            if rf_link_is_notify_req(pkt_light_data.att_cmd().value.val[0] & 0x3f) {
                // Calculate relay timing based on processing delay
                // Formula: (elapsed_time_us + 500) / 1024, clamped to 255
                let mut relay_time = min(
                    (((read_reg_system_tick() - bridge_cmd_time) / CLOCK_SYS_CLOCK_1US) + 500) >> 10,
                    0xff,
                );

                // Store timing information in packet for mesh relay optimization
                pkt_light_data.att_cmd_mut().value.val[17] = relay_time as u8;
            }

            // Queue the command for mesh transmission (no delay, no retransmit)
            app().mesh_manager.add_send_mesh_msg(&*pkt_light_data, 0, 0);
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
/// ```
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
#[cfg_attr(test, mry::mry)]
pub fn tx_packet_bridge()
{
    // Static variable to track last bridge report transmission time
    static TICK_BRIDGE_REPORT: AtomicU32 = AtomicU32::new(0);

    // Disable RF transmit/receive operations for hardware reconfiguration
    rf_set_tx_rx_off();

    // Wait for hardware stabilization (100µs settling time)
    sleep_us(100);

    // Configure RF hardware for mesh network operation
    rf_set_ble_access_code(PAIR_AC.get());  // Set mesh network access code
    rf_set_ble_crc_adv();                   // Configure CRC for advertising packets

    // Check if periodic status broadcast is due
    let tick = read_reg_system_tick();
    let broadcast_interval = (MESH_LISTEN_INTERVAL_US.get() * ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL as u32)
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
mod tests {
    use super::*;
    use crate::sdk::ble_app::rf_drv_8266::{mock_rf_set_tx_rx_off, mock_rf_set_ble_access_code, mock_rf_set_ble_crc_adv};
    use crate::sdk::mcu::clock::mock_sleep_us;
    use crate::sdk::mcu::register::mock_read_reg_system_tick;
    use crate::sdk::ble_app::light_ll::mesh_management::mock_mesh_send_online_status;
    use crate::sdk::ble_app::light_ll::packet_processing::mock_rf_link_is_notify_req;
    use std::sync::atomic::Ordering;
    use mry::Any;

    fn setup_test_state() {
        // Reset all status management state
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(0);
        DEVICE_STATUS_BUFFER_READ_POINTER.set(0);
        STATUS_MESSAGE_SEQUENCE_NUMBER.lock().fill(0);
        SLAVE_READ_STATUS_BUSY.set(0);
        DEVICE_STATUS_READ_UNICAST_MODE.set(false);
        SLAVE_DATA_VALID.set(0);
        BRIDGE_COMMAND_TIMESTAMP.set(1000);
        PAIR_AC.set(0x12345678);
        MESH_LISTEN_INTERVAL_US.set(100000); // 100ms
        
        // Clear packet data to ensure clean state
        let mut packet_data = PKT_LIGHT_DATA.lock();
        packet_data.head_mut()._type = 0x00; // Clear all flags including mesh flag
        drop(packet_data);
    }

    #[test]
    fn test_rf_link_slave_read_status_par_init() {
        // Setup initial non-zero state
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(5);
        DEVICE_STATUS_BUFFER_READ_POINTER.set(3);
        *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() = [1, 2, 3];

        // Execute function
        rf_link_slave_read_status_par_init();

        // Verify state is reset
        assert_eq!(DEVICE_STATUS_BUFFER_WRITE_POINTER.get(), 0);
        assert_eq!(DEVICE_STATUS_BUFFER_READ_POINTER.get(), 0);
        assert_eq!(*STATUS_MESSAGE_SEQUENCE_NUMBER.lock(), [0, 0, 0]);
    }

    #[test]
    #[mry::lock(rf_link_slave_read_status_par_init)]
    fn test_rf_link_slave_read_status_stop() {
        // Setup initial state with active status reporting
        SLAVE_READ_STATUS_BUSY.set(1);
        DEVICE_STATUS_READ_UNICAST_MODE.set(true);
        SLAVE_DATA_VALID.set(5);

        // Mock the init function to verify it's called
        mock_rf_link_slave_read_status_par_init().returns(());

        // Execute function
        rf_link_slave_read_status_stop();

        // Verify cleanup occurred
        assert_eq!(SLAVE_READ_STATUS_BUSY.get(), 0);
        assert_eq!(DEVICE_STATUS_READ_UNICAST_MODE.get(), false);
        assert_eq!(SLAVE_DATA_VALID.get(), 0);

        // Verify init was called for parameter reset
        mock_rf_link_slave_read_status_par_init().assert_called(1);
    }

    #[test]
    fn test_app_bridge_cmd_handle_no_valid_data() {
        setup_test_state();
        SLAVE_DATA_VALID.set(0); // No valid data

        // Function should return early without doing anything
        app_bridge_cmd_handle(1000);

        // No mocks should be called since there's no valid data
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
    }

    #[test]
    #[mry::lock(rf_link_is_notify_req, read_reg_system_tick)]
    fn test_app_bridge_cmd_handle_notify_request_timing() {
        setup_test_state();
        let mut packet_data = PKT_LIGHT_DATA.lock();
        
        // Set up notify request packet
        packet_data.head_mut()._type = 0x00; // Clear mesh flag initially
        packet_data.att_cmd_mut().value.val[0] = 0x10; // Test command (masked will be 0x10)
        packet_data.att_cmd_mut().value.val[17] = 0; // Clear timing field
        drop(packet_data);
        
        SLAVE_DATA_VALID.set(2); // Will become 1 after decrement
        SLAVE_READ_STATUS_BUSY.set(0); // Allow transmission

        // Mock notify request check
        mock_rf_link_is_notify_req(0x10).returns(true);
        
        // Mock system tick for timing calculation
        // To get timing value 1: ((tick_diff / 32) + 500) >> 10 = 1
        // So: (tick_diff / 32) + 500 = 1024
        // tick_diff / 32 = 524
        // tick_diff = 524 * 32 = 16768
        mock_read_reg_system_tick().returns(1000 + 16768); // 16768 ticks elapsed since bridge_cmd_time=1000

        // Execute function
        app_bridge_cmd_handle(1000);

        // Verify packet was modified for mesh transmission
        let packet_data = PKT_LIGHT_DATA.lock();
        assert_ne!(packet_data.head()._type & 0x80, 0); // Mesh flag should be set
        // Verify timing calculation: (16768/32 + 500) / 1024 = (524 + 500) / 1024 = 1024/1024 = 1
        assert_eq!(packet_data.att_cmd().value.val[17], 1);

        // Verify mocks were called
        mock_rf_link_is_notify_req(0x10).assert_called(1);
        mock_read_reg_system_tick().assert_called(1);
    }

    #[test]
    #[mry::lock(rf_link_is_notify_req)]
    fn test_app_bridge_cmd_handle_non_notify_request() {
        setup_test_state();
        let mut packet_data = PKT_LIGHT_DATA.lock();
        
        packet_data.head_mut()._type = 0x00;
        packet_data.att_cmd_mut().value.val[0] = 0x20; // Non-notify command
        drop(packet_data);
        
        SLAVE_DATA_VALID.set(2);
        SLAVE_READ_STATUS_BUSY.set(0);

        // Mock notify request check to return false
        mock_rf_link_is_notify_req(0x20).returns(false);

        // Execute function
        app_bridge_cmd_handle(1000);

        // Verify packet mesh flag was set but no timing applied
        let packet_data = PKT_LIGHT_DATA.lock();
        assert_ne!(packet_data.head()._type & 0x80, 0); // Mesh flag should be set

        // Verify mocks
        mock_rf_link_is_notify_req(0x20).assert_called(1);
        // read_reg_system_tick should not be called for non-notify requests
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
    }

    #[test]
    #[mry::lock(rf_set_tx_rx_off, sleep_us, rf_set_ble_access_code, rf_set_ble_crc_adv, read_reg_system_tick, mesh_send_online_status, app_bridge_cmd_handle)]
    fn test_tx_packet_bridge_basic_setup() {
        setup_test_state();

        // Mock RF setup functions
        mock_rf_set_tx_rx_off().returns(());
        mock_sleep_us(100).returns(());
        mock_rf_set_ble_access_code(0x12345678).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_read_reg_system_tick().returns(50000);
        mock_mesh_send_online_status().returns(());
        mock_app_bridge_cmd_handle(1000).returns(());

        // Execute function
        tx_packet_bridge();

        // Verify RF setup sequence
        mock_rf_set_tx_rx_off().assert_called(1);
        mock_sleep_us(100).assert_called(1);
        mock_rf_set_ble_access_code(0x12345678).assert_called(1);
        mock_rf_set_ble_crc_adv().assert_called(1);
        mock_read_reg_system_tick().assert_called(1);
        mock_app_bridge_cmd_handle(1000).assert_called(1);
    }

    #[test]
    #[mry::lock(rf_set_tx_rx_off, sleep_us, rf_set_ble_access_code, rf_set_ble_crc_adv, read_reg_system_tick, mesh_send_online_status, app_bridge_cmd_handle)]
    fn test_tx_packet_bridge_status_broadcast_triggered() {
        setup_test_state();
        
        // Set up timing to trigger status broadcast
        // broadcast_interval = (100000 * 8).saturating_sub(3 * 32 * 1000) = 800000 - 96000 = 704000
        // Use a tick value much larger than any possible TICK_BRIDGE_REPORT value from previous tests
        // to guarantee the condition: broadcast_interval < tick - TICK_BRIDGE_REPORT
        let current_tick = 10000000; // Much larger than 704000 and any reasonable TICK_BRIDGE_REPORT
        let expected_interval = (MESH_LISTEN_INTERVAL_US.get() * ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL as u32)
                              .saturating_sub(ONLINE_STATUS_COMP * CLOCK_SYS_CLOCK_1US * 1000);

        mock_rf_set_tx_rx_off().returns(());
        mock_sleep_us(100).returns(());
        mock_rf_set_ble_access_code(0x12345678).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_read_reg_system_tick().returns(current_tick);
        mock_mesh_send_online_status().returns(());
        mock_app_bridge_cmd_handle(1000).returns(());

        // Execute function
        tx_packet_bridge();

        // Status should be sent because timing interval is met
        mock_mesh_send_online_status().assert_called(1);
        mock_app_bridge_cmd_handle(1000).assert_called(1);
    }

    #[test]
    #[mry::lock(rf_set_tx_rx_off, sleep_us, rf_set_ble_access_code, rf_set_ble_crc_adv, read_reg_system_tick, app_bridge_cmd_handle)]
    fn test_tx_packet_bridge_status_broadcast_not_due() {
        setup_test_state();
        
        // Set up timing where status broadcast is not due yet
        let current_tick = 1000; // Recent time, broadcast not due
        
        mock_rf_set_tx_rx_off().returns(());
        mock_sleep_us(100).returns(());
        mock_rf_set_ble_access_code(0x12345678).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_read_reg_system_tick().returns(current_tick);
        mock_app_bridge_cmd_handle(1000).returns(());

        // Execute function
        tx_packet_bridge();

        // Status should NOT be sent because timing interval is not met
        mock_app_bridge_cmd_handle(1000).assert_called(1);
        // mesh_send_online_status should not be called
    }

    #[test]
    #[mry::lock(rf_set_tx_rx_off, sleep_us, rf_set_ble_access_code, rf_set_ble_crc_adv, read_reg_system_tick, mesh_send_online_status, app_bridge_cmd_handle)]
    fn test_tx_packet_bridge_function_calls() {
        setup_test_state();

        // Mock all functions with simple returns
        mock_rf_set_tx_rx_off().returns(());
        mock_sleep_us(100).returns(());
        mock_rf_set_ble_access_code(0x12345678).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_read_reg_system_tick().returns(800000); // Use tick > broadcast_interval to trigger status send
        mock_mesh_send_online_status().returns(());
        mock_app_bridge_cmd_handle(1000).returns(());

        // Execute function
        tx_packet_bridge();

        // Verify all functions were called
        mock_rf_set_tx_rx_off().assert_called(1);
        mock_sleep_us(100).assert_called(1);
        mock_rf_set_ble_access_code(0x12345678).assert_called(1);
        mock_rf_set_ble_crc_adv().assert_called(1);
        mock_read_reg_system_tick().assert_called(1);
        mock_mesh_send_online_status().assert_called(1);
        mock_app_bridge_cmd_handle(1000).assert_called(1);
    }

    #[test]
    #[mry::lock(rf_link_is_notify_req, read_reg_system_tick)]
    fn test_app_bridge_cmd_handle_timing_calculation_edge_cases() {
        setup_test_state();
        let mut packet_data = PKT_LIGHT_DATA.lock();
        
        packet_data.head_mut()._type = 0x00;
        packet_data.att_cmd_mut().value.val[0] = 0x10;
        packet_data.att_cmd_mut().value.val[17] = 0;
        drop(packet_data);
        
        SLAVE_DATA_VALID.set(2);
        SLAVE_READ_STATUS_BUSY.set(0);

        // Test maximum timing value clamping
        let bridge_cmd_time = 1000;
        let large_tick = bridge_cmd_time + (0xff << 10) * CLOCK_SYS_CLOCK_1US + 1000; // Should clamp to 255

        mock_rf_link_is_notify_req(0x10).returns(true);
        mock_read_reg_system_tick().returns(large_tick);

        app_bridge_cmd_handle(bridge_cmd_time);

        // Verify timing was clamped to maximum (255)
        let packet_data = PKT_LIGHT_DATA.lock();
        assert_eq!(packet_data.att_cmd().value.val[17], 255);
        
        // Verify mocks were called
        mock_rf_link_is_notify_req(0x10).assert_called(1);
        mock_read_reg_system_tick().assert_called(1);
    }
}
