//! # BLE Advertisement Module
//!
//! This module handles BLE advertisement state management, scan response generation,
//! and advertisement channel cycling for the TLSR8266-based mesh lighting system.
//!
//! ## Key Responsibilities:
//! - BLE advertisement sequence management
//! - Scan response packet generation and transmission
//! - Advertisement channel cycling (37, 38, 39)
//! - State transitions between advertisement and listening modes

use core::ptr::addr_of;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::common::SYS_CHN_ADV;
use crate::sdk::ble_app::light_ll::mesh_management::mesh_send_online_status;
use crate::sdk::ble_app::rf_drv_8266::*;
use crate::sdk::light::RfOperationState;
use crate::sdk::mcu::clock::CLOCK_SYS_CLOCK_1US;
use crate::sdk::mcu::register::*;
use crate::state::*;

/// Returns the number of GATT advertisement cycles to perform.
///
/// This function defines how many advertisement packets should be sent
/// during each advertisement sequence. The value 3 corresponds to advertising
/// on all three primary BLE advertisement channels (37, 38, 39).
///
/// ## BLE Advertisement Channels:
/// - Channel 37: 2402 MHz
/// - Channel 38: 2426 MHz  
/// - Channel 39: 2480 MHz
///
/// ## Returns:
/// Always returns 3 to ensure complete coverage of all advertisement channels
pub fn get_ble_advertisement_channel_count() -> u32 {
    return 3;
}

/// Handles the BLE advertisement state interrupt.
///
/// This function manages the BLE advertisement process, cycling through all three
/// advertisement channels and handling online status reporting. It implements
/// the standard BLE advertisement procedure with mesh-specific extensions.
///
/// ## Advertisement Sequence:
/// 1. Advertise on channel 37 (2402 MHz)
/// 2. Advertise on channel 38 (2426 MHz)  
/// 3. Advertise on channel 39 (2480 MHz)
/// 4. Send mesh online status (if scheduled)
/// 5. Return to listening mode
///
/// ## State Management:
/// - Uses static counter to track current advertisement channel
/// - Manages transition between advertisement and listening states
/// - Handles mesh online status reporting
///
/// ## Timing Control:
/// - Uses precise timing for advertisement intervals (1200µs = 0x4b0)
/// - Short delays for status reporting (100µs) vs listening (500µs)
/// - Enables STX2RX mode for potential scan response reception
#[cfg_attr(test, mry::mry)]
pub fn handle_ble_advertisement_state() {
    // Static counter to track which advertisement channel we're currently using
    static ST_PNO: AtomicU32 = AtomicU32::new(0);

    // Clear RF interrupt status and stop any ongoing transmission/reception
    write_reg8(0x80050f, 0);
    rf_stop_trx();

    // Set link state to advertisement mode (state 1)
    BLE_PERIPHERAL_LINK_STATE.set(crate::sdk::light::BlePeripheralLinkState::Advertising);

    // Check if advertisement sequence is complete or not requested
    if !BLE_ADVERTISING_ENABLED.get()
        || get_ble_advertisement_channel_count() <= ST_PNO.load(Ordering::Relaxed)
    {
        // Advertisement sequence is complete, clean up and transition

        // Clear advertisement flag and reset channel counter
        BLE_ADVERTISING_ENABLED.set(false);
        ST_PNO.store(0, Ordering::Relaxed);

        // Handle online status reporting if scheduled
        if MESH_DEVICE_ONLINE_STATUS.get() {
            // Send mesh online status packet to inform network of our presence
            mesh_send_online_status();
            // Short delay before returning to listening (100µs)
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
        } else {
            // No online status to send, use longer delay before listening (500µs)
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 500 + read_reg_system_tick());
        }

        // Clear online status flag since we've handled it
        MESH_DEVICE_ONLINE_STATUS.set(false);

        // Transition back to listening state
        *CURRENT_RF_STATE.lock() = RfOperationState::MeshListening;

        // Clear RF interrupt status
        write_reg_rf_irq_status(1);
    } else {
        // Continue advertisement sequence on next channel

        // Configure RF for BLE advertisement
        rf_set_ble_access_code_adv(); // Standard BLE advertisement access code
        rf_set_ble_crc_adv(); // Standard BLE advertisement CRC

        // Set the advertisement channel (cycling through 37, 38, 39)
        rf_set_ble_channel(SYS_CHN_ADV[ST_PNO.load(Ordering::Relaxed) as usize % 3]);

        // Schedule next interrupt for advertisement interval (1200µs)
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 0x4b0 + read_reg_system_tick());

        // Clear RF interrupt status
        write_reg_rf_irq_status(1);

        // Start transmission with scan response capability (if not on last channel)
        if ST_PNO.load(Ordering::Relaxed) < 3 {
            // Get the advertisement packet data
            let tmp = *PKT_ADV.lock();

            // Start STX2RX mode: transmit advertisement then switch to RX for scan responses
            // This allows the device to receive and respond to scan requests
            rf_start_stx2rx(
                addr_of!(tmp) as u32,
                CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick(),
            );
        }

        // Increment channel counter for next advertisement
        ST_PNO.store(ST_PNO.load(Ordering::Relaxed) + 1, Ordering::Relaxed);

        // Check if we've completed all advertisement channels
        if get_ble_advertisement_channel_count() <= ST_PNO.load(Ordering::Relaxed) {
            // Reset counter and clear advertisement flag for next sequence
            ST_PNO.store(0, Ordering::Relaxed);
            BLE_ADVERTISING_ENABLED.set(false);
        }

        // Set next state to listening (will be processed after advertisement completes)
        *CURRENT_RF_STATE.lock() = RfOperationState::MeshListening;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mry::Any;

    // Import mock functions from their original modules
    use crate::sdk::ble_app::rf_drv_8266::mock_rf_stop_trx;
    use crate::sdk::mcu::analog::{analog_write, mock_analog_write};
    use crate::sdk::mcu::clock::CLOCK_SYS_CLOCK_1US;
    use crate::sdk::mcu::register::{
        mock_read_reg_rf_mode, mock_read_reg_system_tick, mock_write_reg8,
        mock_write_reg_dma3_addr, mock_write_reg_pll_rx_fine_div_tune,
        mock_write_reg_rf_access_code, mock_write_reg_rf_channel, mock_write_reg_rf_crc,
        mock_write_reg_rf_irq_status, mock_write_reg_rf_mode, mock_write_reg_rf_mode_control,
        mock_write_reg_rf_rx_mode, mock_write_reg_rf_sched_tick, mock_write_reg_rf_txrx_state,
        mock_write_reg_system_tick_irq,
    };

    /// Helper function to reset global state to known values for test isolation.
    /// This ensures each test starts with a clean state.
    /// Note: Call reset_advertisement_internal_state() separately after setting up mocks.
    fn reset_global_state() {
        BLE_ADVERTISING_ENABLED.set(false);
        BLE_PERIPHERAL_LINK_STATE.set(crate::sdk::light::BlePeripheralLinkState::Disconnected);
        MESH_DEVICE_ONLINE_STATUS.set(false);
        *CURRENT_RF_STATE.lock() = RfOperationState::MeshListening;
    }

    /// Helper function to reset the internal ST_PNO static counter in handle_ble_advertisement_state.
    /// This should be called from within tests that have mocks set up.
    fn reset_advertisement_internal_state() {
        // Save current state
        let saved_adv_enabled = BLE_ADVERTISING_ENABLED.get();
        let saved_online_status = MESH_DEVICE_ONLINE_STATUS.get();
        let saved_handler_state = *CURRENT_RF_STATE.lock();

        // Set state to trigger the completion path (line 76-81) which resets ST_PNO to 0
        BLE_ADVERTISING_ENABLED.set(false);
        MESH_DEVICE_ONLINE_STATUS.set(false);

        // This will trigger the completion path which resets ST_PNO to 0
        handle_ble_advertisement_state();

        // Restore original state
        BLE_ADVERTISING_ENABLED.set(saved_adv_enabled);
        MESH_DEVICE_ONLINE_STATUS.set(saved_online_status);
        *CURRENT_RF_STATE.lock() = saved_handler_state;
    }

    /// Helper function to set up all the mocks needed for RF BLE functions.
    /// This mocks the internal register operations of rf_set_ble_access_code_adv,
    /// rf_set_ble_crc_adv, rf_set_ble_channel, and rf_start_stx2rx.
    fn setup_rf_ble_function_mocks() {
        // Mock rf_set_ble_access_code_adv internals
        mock_write_reg_rf_access_code(0xd6be898e).returns(());

        // Mock rf_set_ble_crc_adv internals
        mock_write_reg_rf_crc(0x555555).returns(());

        // Mock rf_set_ble_channel internals (will be called with different channels)
        mock_write_reg_rf_channel(Any).returns(());
        mock_analog_write(6, 0).returns(());
        mock_write_reg_rf_mode(0x29).returns(());
        mock_write_reg_rf_rx_mode(0).returns(());
        mock_write_reg_rf_txrx_state(Any).returns(());
        mock_write_reg_pll_rx_fine_div_tune(Any).returns(());
        mock_analog_write(0x93, Any).returns(());

        // Mock rf_start_stx2rx internals
        mock_read_reg_rf_mode().returns(0);
        mock_write_reg_rf_sched_tick(Any).returns(());
        mock_write_reg_rf_mode(Any).returns(());
        mock_write_reg_rf_mode_control(0x87).returns(());
        mock_write_reg_dma3_addr(Any).returns(());
    }

    // ================================================================================
    // Tests for get_ble_advertisement_channel_count function
    // ================================================================================

    /// Tests that get_ble_advertisement_channel_count returns the correct value.
    ///
    /// This function should always return 3 to ensure coverage of all three
    /// BLE primary advertisement channels (37, 38, 39).
    #[test]
    fn test_get_ble_advertisement_channel_count() {
        let count = get_ble_advertisement_channel_count();
        assert_eq!(
            count, 3,
            "Should return 3 for all three advertisement channels"
        );
    }

    // ================================================================================
    // Tests for handle_ble_advertisement_state function - Advertisement Completion
    // ================================================================================

    /// Tests advertisement sequence completion with online status reporting.
    ///
    /// Verifies that when advertisement is complete and online status is scheduled:
    /// - Advertisement flag is cleared and channel counter is reset
    /// - Online status is sent via mesh_send_online_status (simulated)
    /// - Short delay (100µs) is scheduled before listening
    /// - Online status flag is cleared after handling
    /// - State transitions to Listen mode
    #[test]
    #[mry::lock(
        write_reg8,
        rf_stop_trx,
        write_reg_system_tick_irq,
        read_reg_system_tick,
        write_reg_rf_irq_status
    )]
    fn test_advertisement_completion_with_online_status() {
        // Setup mocks
        mock_write_reg8(Any, Any).returns(());
        mock_rf_stop_trx().returns(());
        mock_write_reg_system_tick_irq(Any).returns(());
        mock_read_reg_system_tick().returns(10000);
        mock_write_reg_rf_irq_status(Any).returns(());

        // Reset state for clean test
        reset_global_state();

        // Reset internal static state (must be done after mocks are set up)
        reset_advertisement_internal_state();

        // Setup: Advertisement disabled (sequence complete) with online status
        BLE_ADVERTISING_ENABLED.set(false);
        MESH_DEVICE_ONLINE_STATUS.set(true);

        // Execute function
        handle_ble_advertisement_state();

        // Verify link state is set to advertising mode initially
        assert_eq!(
            BLE_PERIPHERAL_LINK_STATE.get(),
            crate::sdk::light::BlePeripheralLinkState::Advertising,
            "Link state should be set to advertising mode"
        );

        // Verify RF operations are stopped and status cleared
        // Note: rf_stop_trx is called twice - once during reset and once during the actual test
        mock_rf_stop_trx().assert_called(2);
        mock_write_reg8(0x80050f, 0).assert_called(2);

        // Verify advertisement flag remains cleared
        assert_eq!(
            BLE_ADVERTISING_ENABLED.get(),
            false,
            "Advertisement flag should remain cleared after completion"
        );

        // Verify short delay is scheduled (100µs) - called during both reset and test
        let expected_tick = CLOCK_SYS_CLOCK_1US * 100 + 10000;
        mock_write_reg_system_tick_irq(expected_tick).assert_called(1);

        // Verify online status flag is cleared
        assert_eq!(
            MESH_DEVICE_ONLINE_STATUS.get(),
            false,
            "Online status flag should be cleared after handling"
        );

        // Verify state transitions to Listen
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::MeshListening,
            "State should transition to Listen mode"
        );

        // Verify RF interrupt status is cleared - called during both reset and test
        mock_write_reg_rf_irq_status(1).assert_called(2);
    }

    /// Tests advertisement sequence completion without online status reporting.
    ///
    /// Verifies that when advertisement is complete but no online status is scheduled:
    /// - Advertisement flag is cleared and channel counter is reset
    /// - No online status is sent
    /// - Longer delay (500µs) is scheduled before listening
    /// - State transitions to Listen mode
    #[test]
    #[mry::lock(
        write_reg8,
        rf_stop_trx,
        write_reg_system_tick_irq,
        read_reg_system_tick,
        write_reg_rf_irq_status
    )]
    fn test_advertisement_completion_without_online_status() {
        // Setup mocks
        mock_write_reg8(Any, Any).returns(());
        mock_rf_stop_trx().returns(());
        mock_write_reg_system_tick_irq(Any).returns(());
        mock_read_reg_system_tick().returns(15000);
        mock_write_reg_rf_irq_status(Any).returns(());

        // Reset state for clean test
        reset_global_state();

        // Reset internal static state (must be done after mocks are set up)
        reset_advertisement_internal_state();

        // Setup: Advertisement disabled (sequence complete) without online status
        BLE_ADVERTISING_ENABLED.set(false);
        MESH_DEVICE_ONLINE_STATUS.set(false);

        // Execute function
        handle_ble_advertisement_state();

        // Verify link state is set to advertising mode initially
        assert_eq!(
            BLE_PERIPHERAL_LINK_STATE.get(),
            crate::sdk::light::BlePeripheralLinkState::Advertising,
            "Link state should be set to advertising mode"
        );

        // Verify RF operations are stopped
        // Note: rf_stop_trx is called twice - once during reset and once during the actual test
        mock_rf_stop_trx().assert_called(2);
        mock_write_reg8(0x80050f, 0).assert_called(2);

        // Verify longer delay is scheduled (500µs) - called during both reset and test
        let expected_tick = CLOCK_SYS_CLOCK_1US * 500 + 15000;
        mock_write_reg_system_tick_irq(expected_tick).assert_called(2);

        // Verify online status flag remains cleared
        assert_eq!(
            MESH_DEVICE_ONLINE_STATUS.get(),
            false,
            "Online status flag should remain cleared"
        );

        // Verify state transitions to Listen
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::MeshListening,
            "State should transition to Listen mode"
        );
    }

    // ================================================================================
    // Tests for handle_ble_advertisement_state function - Advertisement Continuation
    // ================================================================================

    /// Tests advertisement continuation on first channel.
    ///
    /// Verifies that when advertisement is enabled and we're on the first channel:
    /// - RF is configured for BLE advertisement
    /// - Correct advertisement channel is set (channel 37)
    /// - Advertisement interval timing is scheduled (1200µs)
    /// - STX2RX mode is started for scan response capability
    /// - State transitions to Listen mode for next iteration
    #[test]
    #[mry::lock(
        write_reg8,
        rf_stop_trx,
        write_reg_rf_access_code,
        write_reg_rf_crc,
        write_reg_rf_channel,
        write_reg_rf_mode,
        write_reg_rf_rx_mode,
        write_reg_rf_txrx_state,
        write_reg_pll_rx_fine_div_tune,
        analog_write,
        read_reg_rf_mode,
        write_reg_rf_sched_tick,
        write_reg_rf_mode_control,
        write_reg_dma3_addr,
        write_reg_system_tick_irq,
        read_reg_system_tick,
        write_reg_rf_irq_status
    )]
    fn test_advertisement_continuation_first_channel() {
        // Setup basic mocks
        mock_write_reg8(Any, Any).returns(());
        mock_rf_stop_trx().returns(());
        mock_write_reg_system_tick_irq(Any).returns(());
        mock_read_reg_system_tick().returns(20000);
        mock_write_reg_rf_irq_status(Any).returns(());

        // Setup RF BLE function mocks
        setup_rf_ble_function_mocks();

        // Reset state for clean test
        reset_global_state();

        // Reset internal static state (must be done after mocks are set up)
        reset_advertisement_internal_state();

        // Setup: Advertisement enabled, first channel
        BLE_ADVERTISING_ENABLED.set(true);

        // Execute function
        handle_ble_advertisement_state();

        // Verify link state is set to advertising mode
        assert_eq!(
            BLE_PERIPHERAL_LINK_STATE.get(),
            crate::sdk::light::BlePeripheralLinkState::Advertising,
            "Link state should be set to advertising mode"
        );

        // Verify RF operations are stopped initially
        // Note: rf_stop_trx is called twice - once during reset and once during the actual test
        mock_rf_stop_trx().assert_called(2);
        mock_write_reg8(0x80050f, 0).assert_called(2);

        // Verify BLE advertisement access code is set
        mock_write_reg_rf_access_code(0xd6be898e).assert_called(1);

        // Verify BLE advertisement CRC is set
        mock_write_reg_rf_crc(0x555555).assert_called(1);

        // Verify correct advertisement channel is set (first channel from SYS_CHN_ADV)
        mock_write_reg_rf_channel(SYS_CHN_ADV[0]).assert_called(1);

        // Verify advertisement interval timing is scheduled (1200µs = 0x4b0)
        let expected_tick = CLOCK_SYS_CLOCK_1US * 0x4b0 + 20000;
        mock_write_reg_system_tick_irq(expected_tick).assert_called(1);

        // Verify STX2RX mode is started (since we're not on last channel)
        mock_write_reg_rf_mode_control(0x87).assert_called(1);

        // Verify state transitions to Listen for next iteration
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::MeshListening,
            "State should transition to Listen mode"
        );

        // Verify RF interrupt status is cleared - called during both reset and test
        mock_write_reg_rf_irq_status(1).assert_called(2);
    }

    /// Tests advertisement channel cycling through multiple calls.
    ///
    /// This test verifies that each individual call to the advertisement function
    /// works correctly for advertising state, since the internal static counter
    /// persists between calls.
    #[test]
    #[mry::lock(
        write_reg8,
        rf_stop_trx,
        write_reg_rf_access_code,
        write_reg_rf_crc,
        write_reg_rf_channel,
        write_reg_rf_mode,
        write_reg_rf_rx_mode,
        write_reg_rf_txrx_state,
        write_reg_pll_rx_fine_div_tune,
        analog_write,
        read_reg_rf_mode,
        write_reg_rf_sched_tick,
        write_reg_rf_mode_control,
        write_reg_dma3_addr,
        write_reg_system_tick_irq,
        read_reg_system_tick,
        write_reg_rf_irq_status
    )]
    fn test_advertisement_channel_cycling() {
        // Setup basic mocks
        mock_write_reg8(Any, Any).returns(());
        mock_rf_stop_trx().returns(());
        mock_write_reg_system_tick_irq(Any).returns(());
        mock_read_reg_system_tick().returns(25000);
        mock_write_reg_rf_irq_status(Any).returns(());

        // Setup RF BLE function mocks
        setup_rf_ble_function_mocks();

        // Reset state for clean test
        reset_global_state();

        // Reset internal static state (must be done after mocks are set up)
        reset_advertisement_internal_state();

        // Test first call - should use first channel
        BLE_ADVERTISING_ENABLED.set(true);
        handle_ble_advertisement_state();

        // Verify first channel was set
        mock_write_reg_rf_channel(SYS_CHN_ADV[0]).assert_called(1);

        // Verify link state is correct
        assert_eq!(
            BLE_PERIPHERAL_LINK_STATE.get(),
            crate::sdk::light::BlePeripheralLinkState::Advertising,
            "Link state should be advertising"
        );

        // Note: Advertisement flag remains true after the first call since we haven't completed all 3 channels yet
        // The flag is only cleared when ST_PNO >= 3 (after incrementing to 1, 2, then 3)
        assert_eq!(
            BLE_ADVERTISING_ENABLED.get(),
            true,
            "Advertisement flag should remain true after processing first channel"
        );

        // Verify state transitions to Listen
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::MeshListening,
            "State should be in Listen mode"
        );
    }

    /// Tests timing calculations with different system tick values.
    ///
    /// Verifies that timing calculations work correctly with various system tick values
    /// and that the proper delays are applied for different scenarios.
    #[test]
    #[mry::lock(
        write_reg8,
        rf_stop_trx,
        write_reg_system_tick_irq,
        read_reg_system_tick,
        write_reg_rf_irq_status
    )]
    fn test_timing_calculations() {
        // Setup mocks
        mock_write_reg8(Any, Any).returns(());
        mock_rf_stop_trx().returns(());
        mock_write_reg_system_tick_irq(Any).returns(());
        mock_write_reg_rf_irq_status(Any).returns(());

        // Reset state for test
        reset_global_state();
        mock_read_reg_system_tick().returns(1000);

        // Test with online status enabled
        BLE_ADVERTISING_ENABLED.set(false);
        MESH_DEVICE_ONLINE_STATUS.set(true);

        handle_ble_advertisement_state();

        // Verify correct timing calculation for online status (100µs delay)
        let expected_tick = CLOCK_SYS_CLOCK_1US * 100 + 1000;
        mock_write_reg_system_tick_irq(expected_tick).assert_called(1);

        // Verify online status flag is cleared
        assert_eq!(
            MESH_DEVICE_ONLINE_STATUS.get(),
            false,
            "Online status flag should be cleared after handling"
        );
    }

    /// Tests edge case behavior when advertisement is repeatedly enabled.
    ///
    /// This test verifies that the function handles rapid advertisement cycles gracefully.
    #[test]
    #[mry::lock(
        write_reg8,
        rf_stop_trx,
        write_reg_rf_access_code,
        write_reg_rf_crc,
        write_reg_rf_channel,
        write_reg_rf_mode,
        write_reg_rf_rx_mode,
        write_reg_rf_txrx_state,
        write_reg_pll_rx_fine_div_tune,
        analog_write,
        read_reg_rf_mode,
        write_reg_rf_sched_tick,
        write_reg_rf_mode_control,
        write_reg_dma3_addr,
        write_reg_system_tick_irq,
        read_reg_system_tick,
        write_reg_rf_irq_status
    )]
    fn test_rapid_advertisement_cycles() {
        // Setup basic mocks
        mock_write_reg8(Any, Any).returns(());
        mock_rf_stop_trx().returns(());
        mock_write_reg_system_tick_irq(Any).returns(());
        mock_read_reg_system_tick().returns(30000);
        mock_write_reg_rf_irq_status(Any).returns(());

        // Setup RF BLE function mocks
        setup_rf_ble_function_mocks();

        // Reset state for clean test
        reset_global_state();

        // Reset internal static state (must be done after mocks are set up)
        reset_advertisement_internal_state();

        // Run multiple advertisement cycles rapidly
        for cycle in 0..3 {
            BLE_ADVERTISING_ENABLED.set(true);
            handle_ble_advertisement_state();

            // Verify each cycle works correctly
            assert_eq!(
                BLE_PERIPHERAL_LINK_STATE.get(),
                crate::sdk::light::BlePeripheralLinkState::Advertising,
                "Link state should be advertising in cycle {}",
                cycle
            );

            // Advertisement remains enabled until the complete 3-channel sequence is done
            // Since we reset ST_PNO to 0 before this test, each call advances through channels 0,1,2
            // Only after channel 2 (the 3rd call) does BLE_ADVERTISING_ENABLED get cleared
            if cycle < 2 {
                assert_eq!(
                    BLE_ADVERTISING_ENABLED.get(),
                    true,
                    "Advertisement should remain enabled during sequence cycle {}",
                    cycle
                );
            } else {
                assert_eq!(
                    BLE_ADVERTISING_ENABLED.get(),
                    false,
                    "Advertisement should be auto-disabled after complete sequence (cycle {})",
                    cycle
                );
            }
        }

        // Verify final state is consistent
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::MeshListening,
            "Final state should be Listen mode"
        );
    }
}
