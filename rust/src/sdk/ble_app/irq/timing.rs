//! # Timing and Synchronization Module
//!
//! This module handles BLE receive state configuration, channel hopping,
//! and timing calculations for the TLSR8266-based mesh lighting system.
//!
//! ## Key Responsibilities:
//! - BLE receive state configuration and timing
//! - Channel hopping and frequency selection
//! - Connection event timing management
//! - RF timing parameter configuration

use core::ptr::addr_of;

use crate::sdk::ble_app::ble_ll_channel_selection::ble_ll_select_next_data_channel;
use crate::sdk::ble_app::rf_drv_8266::*;
use crate::sdk::light::RfOperationState;
use crate::sdk::mcu::clock::{sleep_us, CLOCK_SYS_CLOCK_1US};
use crate::sdk::mcu::register::*;
use crate::state::*;

/// Handles the BLE receive state interrupt.
///
/// This function configures the radio for receiving the next packet from the BLE master
/// during an active connection. It sets up the correct channel, access code, CRC, and
/// timing parameters for the upcoming connection event.
///
/// ## BLE Connection Event Flow:
/// 1. Configure RF for the next data channel (frequency hopping)
/// 2. Set connection-specific access code and CRC initialization
/// 3. Calculate timing for the next connection event
/// 4. Start buffered receive (BRX) mode
/// 5. Process any pending timing updates
///
/// ## Timing Management:
/// - Schedules next connection event based on connection interval
/// - Handles window size for slave listening duration
/// - Adjusts timing for OTA operations
/// - Enables timing adjustment mechanisms
///
/// ## Channel Hopping:
/// - Selects next channel using BLE frequency hopping algorithm
/// - Uses connection-specific channel map and hop increment
#[cfg_attr(test, mry::mry)]
pub fn configure_ble_receive_state() {
    // Clear RF interrupt status and stop current radio operations
    write_reg8(0x080050f, 0x80);
    rf_stop_trx();

    // Set link state to BLE receive mode (state 7)
    BLE_PERIPHERAL_LINK_STATE.set(crate::sdk::light::BlePeripheralLinkState::Receiving);

    // Configure RF timing: TX wait and settle time for optimal performance
    write_reg8(0x00800f04, 0x67);

    // Determine the next data channel using BLE frequency hopping
    let chn_map = PKT_INIT.lock().ll_init().chm;
    let next_chan =
        ble_ll_select_next_data_channel(&chn_map, PKT_INIT.lock().ll_init().hop & 0x1f) as u8;
    rf_set_ble_channel(next_chan);

    // Configure connection-specific access code for packet filtering
    // Access code is a 32-bit value that must match for packet reception
    let aa = PKT_INIT.lock().ll_init().aa;
    let crcinit = PKT_INIT.lock().ll_init().crcinit;

    // Reconstruct 32-bit access code from byte array (little-endian format)
    write_reg_rf_access_code(
        ((aa[2] as u32) << 8)
            | ((aa[1] as u32) << 0x10)
            | (aa[3] as u32)
            | ((aa[0] as u32) << 0x18),
    );

    // Configure CRC initialization value for packet integrity checking
    write_reg_rf_crc(
        ((crcinit[1] as u32) << 8) | ((crcinit[2] as u32) << 0x10) | crcinit[0] as u32,
    );

    // Calculate next connection event timing
    SLAVE_NEXT_CONNECT_TICK.set(SLAVE_LINK_INTERVAL.get() + read_reg_system_tick_irq());

    // Configure interrupt timing based on operational mode
    if OTA_UPDATE_IN_PROGRESS.get() == false {
        // Normal operation: use window size or minimum timing (0xed8 = ~3.8ms)
        if SLAVE_WINDOW_SIZE.get() == 0 || SLAVE_WINDOW_SIZE.get() <= CLOCK_SYS_CLOCK_1US * 0xed8 {
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 0xed8 + read_reg_system_tick());
        } else {
            write_reg_system_tick_irq(read_reg_system_tick() + SLAVE_WINDOW_SIZE.get());
        }
    } else {
        // OTA operation in progress: use longer timeout (10ms)
        write_reg_system_tick_irq(10000 * CLOCK_SYS_CLOCK_1US + read_reg_system_tick());
    }

    // Set next state to bridge mode for connection event processing
    *CURRENT_RF_STATE.lock() = RfOperationState::Connected;

    // Configure buffered receive timing (start in 100µs)
    BRIDGE_RECEIVE_TIMING_TICK.set(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());

    // Enable timing adjustment for connection synchronization
    BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(true);

    // Start buffered receive mode with empty packet buffer
    rf_start_brx(addr_of!(PKT_EMPTY) as u32, BRIDGE_RECEIVE_TIMING_TICK.get());

    // Short delay to ensure radio configuration is stable
    sleep_us(2);

    // Process any pending connection timing updates
    super::connection::handle_ble_connection_parameter_updates();
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::sdk::ble_app::ble_ll_channel_selection::mock_ble_ll_select_next_data_channel;
    use crate::sdk::ble_app::irq::connection::{
        handle_ble_connection_parameter_updates, mock_handle_ble_connection_parameter_updates,
    };
    use crate::sdk::ble_app::rf_drv_8266::{
        mock_rf_set_ble_channel, mock_rf_start_brx, mock_rf_stop_trx,
    };
    use crate::sdk::light::{BlePeripheralLinkState, RfOperationState};
    use crate::sdk::mcu::clock::{mock_sleep_us, CLOCK_SYS_CLOCK_1US};
    use mry::Any;

    use crate::sdk::mcu::register::{
        mock_read_reg_system_tick, mock_read_reg_system_tick_irq, mock_write_reg8,
        mock_write_reg_rf_access_code, mock_write_reg_rf_crc, mock_write_reg_system_tick_irq,
    };

    /// Helper function to reset global state to known values for test isolation.
    fn reset_global_state() {
        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Disconnected);
        SLAVE_LINK_INTERVAL.set(100000); // 100ms default
        SLAVE_WINDOW_SIZE.set(50000); // 50ms default
        SLAVE_NEXT_CONNECT_TICK.set(0);
        BRIDGE_RECEIVE_TIMING_TICK.set(0);
        OTA_UPDATE_IN_PROGRESS.set(false);
        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(false);
        *CURRENT_RF_STATE.lock() = RfOperationState::MeshListening;

        // Reset PKT_INIT structure with mock values
        let mut pkt_init = PKT_INIT.lock();
        pkt_init.ll_init_mut().chm = [0xFF, 0xFF, 0xFF, 0xFF, 0x1F]; // All channels enabled (5 bytes)
        pkt_init.ll_init_mut().hop = 0x12; // Hop increment
        pkt_init.ll_init_mut().aa = [0xAA, 0xBB, 0xCC, 0xDD]; // Access address
        pkt_init.ll_init_mut().crcinit = [0x11, 0x22, 0x33]; // CRC init
    }

    /// Helper function to set up RF and register mocks for consistent testing.
    fn setup_rf_and_register_mocks() {
        // RF function mocks
        mock_rf_stop_trx().returns(());
        mock_rf_set_ble_channel(Any).returns(());
        mock_rf_start_brx(Any, Any).returns(());

        // Register operation mocks
        mock_write_reg8(Any, Any).returns(());
        mock_write_reg_rf_access_code(Any).returns(());
        mock_write_reg_rf_crc(Any).returns(());
        mock_read_reg_system_tick().returns(50000);
        mock_write_reg_system_tick_irq(Any).returns(());
        mock_read_reg_system_tick_irq().returns(100000);

        // Clock function mocks
        mock_sleep_us(Any).returns(());

        // BLE LL function mocks
        mock_ble_ll_select_next_data_channel(Any, Any).returns(12);

        // Connection function mocks
        mock_handle_ble_connection_parameter_updates().returns(());
    }

    // ================================================================================
    // Tests for configure_ble_receive_state function
    // ================================================================================

    /// Tests basic BLE receive state configuration.
    ///
    /// Verifies that the function:
    /// - Sets the correct link state
    /// - Configures RF parameters
    /// - Configures access code and CRC
    /// - Transitions to bridge state
    #[test]
    #[mry::lock(rf_stop_trx)]
    #[mry::lock(rf_set_ble_channel)]
    #[mry::lock(rf_start_brx)]
    #[mry::lock(write_reg8)]
    #[mry::lock(write_reg_rf_access_code)]
    #[mry::lock(write_reg_rf_crc)]
    #[mry::lock(read_reg_system_tick)]
    #[mry::lock(write_reg_system_tick_irq)]
    #[mry::lock(read_reg_system_tick_irq)]
    #[mry::lock(sleep_us)]
    #[mry::lock(ble_ll_select_next_data_channel)]
    #[mry::lock(handle_ble_connection_parameter_updates)]
    fn test_configure_ble_receive_state_basic() {
        // Setup mocks
        setup_rf_and_register_mocks();

        // Reset state for clean test
        reset_global_state();

        // Execute function
        configure_ble_receive_state();

        // Verify link state was set to receiving
        assert_eq!(
            BLE_PERIPHERAL_LINK_STATE.get(),
            BlePeripheralLinkState::Receiving,
            "Link state should be set to receiving"
        );

        // Verify RF operations were called
        mock_rf_stop_trx().assert_called(1);
        mock_rf_set_ble_channel(Any).assert_called(1); // Channel selection is not mocked

        // Verify register operations
        mock_write_reg8(0x080050f, 0x80).assert_called(1); // RF interrupt clear
        mock_write_reg8(0x00800f04, 0x67).assert_called(1); // RF timing

        // Verify access code calculation: ((aa[2] << 8) | (aa[1] << 16) | aa[3] | (aa[0] << 24))
        // aa = [0xAA, 0xBB, 0xCC, 0xDD]
        // Expected: (0xCC << 8) | (0xBB << 16) | 0xDD | (0xAA << 24) = 0xAABBCCDD
        mock_write_reg_rf_access_code(0xAABBCCDD).assert_called(1);

        // Verify CRC calculation: ((crcinit[1] << 8) | (crcinit[2] << 16) | crcinit[0])
        // crcinit = [0x11, 0x22, 0x33]
        // Expected: (0x22 << 8) | (0x33 << 16) | 0x11 = 0x00332211
        mock_write_reg_rf_crc(0x00332211).assert_called(1);

        // Verify state transition
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Connected,
            "State should transition to bridge mode"
        );

        // Verify timing adjustment is enabled
        assert_eq!(
            BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.get(),
            true,
            "Timing adjustment should be enabled"
        );
    }

    /// Tests BLE receive state configuration with normal timing (no OTA).
    ///
    /// Verifies that when OTA_UPDATE_IN_PROGRESS is false:
    /// - Uses window size if it's greater than minimum timing
    /// - Uses minimum timing (0xed8 = ~3.8ms) if window size is too small
    /// - Calculates correct next connection event timing
    #[test]
    #[mry::lock(rf_stop_trx)]
    #[mry::lock(rf_set_ble_channel)]
    #[mry::lock(rf_start_brx)]
    #[mry::lock(write_reg8)]
    #[mry::lock(write_reg_rf_access_code)]
    #[mry::lock(write_reg_rf_crc)]
    #[mry::lock(read_reg_system_tick)]
    #[mry::lock(write_reg_system_tick_irq)]
    #[mry::lock(read_reg_system_tick_irq)]
    #[mry::lock(sleep_us)]
    #[mry::lock(ble_ll_select_next_data_channel)]
    #[mry::lock(handle_ble_connection_parameter_updates)]
    fn test_configure_ble_receive_state_normal_timing() {
        // Setup mocks
        setup_rf_and_register_mocks();

        // Reset state for clean test
        reset_global_state();

        // Setup: Normal operation, no OTA
        OTA_UPDATE_IN_PROGRESS.set(false);
        SLAVE_WINDOW_SIZE.set(150000); // 150ms, greater than minimum (121600)

        // Execute function
        configure_ble_receive_state();

        // Verify timing calculation: current_tick + window_size
        // read_reg_system_tick() returns 50000, window_size is 150000
        let expected_timing = 50000 + 150000;
        mock_write_reg_system_tick_irq(expected_timing).assert_called(1);

        // Verify next connection event timing
        // SLAVE_LINK_INTERVAL (100000) + read_reg_system_tick_irq() (100000) = 200000
        assert_eq!(
            SLAVE_NEXT_CONNECT_TICK.get(),
            200000,
            "Next connection tick should be calculated correctly"
        );
    }

    /// Tests BLE receive state configuration with minimum timing.
    ///
    /// Verifies that when window size is 0 or too small:
    /// - Uses minimum timing (0xed8 = ~3.8ms)
    /// - Calculates timing based on CLOCK_SYS_CLOCK_1US
    #[test]
    #[mry::lock(rf_stop_trx)]
    #[mry::lock(rf_set_ble_channel)]
    #[mry::lock(rf_start_brx)]
    #[mry::lock(write_reg8)]
    #[mry::lock(write_reg_rf_access_code)]
    #[mry::lock(write_reg_rf_crc)]
    #[mry::lock(read_reg_system_tick)]
    #[mry::lock(write_reg_system_tick_irq)]
    #[mry::lock(read_reg_system_tick_irq)]
    #[mry::lock(sleep_us)]
    #[mry::lock(ble_ll_select_next_data_channel)]
    #[mry::lock(handle_ble_connection_parameter_updates)]
    fn test_configure_ble_receive_state_minimum_timing() {
        // Setup mocks
        setup_rf_and_register_mocks();

        // Reset state for clean test
        reset_global_state();

        // Setup: Window size too small (less than 0xed8 * CLOCK_SYS_CLOCK_1US)
        OTA_UPDATE_IN_PROGRESS.set(false);
        SLAVE_WINDOW_SIZE.set(1000); // Very small, should trigger minimum timing

        // Execute function
        configure_ble_receive_state();

        // Verify minimum timing is used: 0xed8 * CLOCK_SYS_CLOCK_1US + current_tick
        // 0xed8 = 3800, CLOCK_SYS_CLOCK_1US = 32, current_tick = 50000
        let expected_min_timing = 3800 * 32 + 50000;
        mock_write_reg_system_tick_irq(expected_min_timing).assert_called(1);
    }

    /// Tests BLE receive state configuration during OTA update.
    ///
    /// Verifies that when OTA_UPDATE_IN_PROGRESS is true:
    /// - Uses longer timeout (10ms) instead of normal timing
    /// - Ignores window size settings
    /// - Provides more time for OTA operations
    #[test]
    #[mry::lock(rf_stop_trx)]
    #[mry::lock(rf_set_ble_channel)]
    #[mry::lock(rf_start_brx)]
    #[mry::lock(write_reg8)]
    #[mry::lock(write_reg_rf_access_code)]
    #[mry::lock(write_reg_rf_crc)]
    #[mry::lock(read_reg_system_tick)]
    #[mry::lock(write_reg_system_tick_irq)]
    #[mry::lock(read_reg_system_tick_irq)]
    #[mry::lock(sleep_us)]
    #[mry::lock(ble_ll_select_next_data_channel)]
    #[mry::lock(handle_ble_connection_parameter_updates)]
    fn test_configure_ble_receive_state_ota_timing() {
        // Setup mocks
        setup_rf_and_register_mocks();

        // Reset state for clean test
        reset_global_state();

        // Setup: OTA update in progress
        OTA_UPDATE_IN_PROGRESS.set(true);
        SLAVE_WINDOW_SIZE.set(75000); // Should be ignored during OTA

        // Execute function
        configure_ble_receive_state();

        // Verify OTA timing is used: 10000 * CLOCK_SYS_CLOCK_1US + current_tick
        // 10000 * 32 + 50000 = 370000
        let expected_ota_timing = 10000 * 32 + 50000;
        mock_write_reg_system_tick_irq(expected_ota_timing).assert_called(1);
    }
}
