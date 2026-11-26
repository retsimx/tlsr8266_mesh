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

use crate::ble::connection::BLE_PERIPHERAL_LINK_STATE;
use crate::ble::flash_config::SYS_CHN_ADV;
use crate::mesh_ll::node_status::{mesh_send_online_status, MESH_DEVICE_ONLINE_STATUS};
use crate::rf::drv_8266::*;
use crate::rf::drv_8266::{BLE_ADVERTISING_ENABLED, CURRENT_RF_STATE, PKT_ADV};
use crate::types::RfOperationState;
use tlsr8266_hal::common::sync::SimplifyLS;
use tlsr8266_hal::mcu::clock::CLOCK_SYS_CLOCK_1US;
use tlsr8266_pac::*;

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
    3
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
pub fn handle_ble_advertisement_state() {
    // Static counter to track which advertisement channel we're currently using
    static ST_PNO: AtomicU32 = AtomicU32::new(0);

    // Clear DMA3 (RF TX) mode byte before stopping TRX
    write_reg_dma3_mode(0);
    rf_stop_trx();

    // Set link state to advertisement mode (state 1)
    BLE_PERIPHERAL_LINK_STATE.set(crate::types::BlePeripheralLinkState::Advertising);

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
#[coverage(off)]
mod tests {
    use super::*;

    use crate::ble::connection::BLE_PERIPHERAL_LINK_STATE;
    use crate::types::BlePeripheralLinkState;
    use tlsr8266_hal::mcu::clock::CLOCK_SYS_CLOCK_1US;
    use tlsr8266_pac::fake_mmio::FakeMmio;

    /// Datasheet offset of the free-running system tick this module schedules from.
    const REG_SYSTEM_TICK: u32 = 0x740;

    /// Reset the PAC/HAL host fakes so a test never inherits another test's
    /// register bank, analog bank, clock or AES transport (the suite runs
    /// `--test-threads=1`, so all of these are process-wide singletons).
    fn reset_host_fakes() {
        FakeMmio::reset();
        tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog::reset();
        tlsr8266_hal::mcu::clock::fake_clock::reset();
        tlsr8266_hal::mcu::crypto::fake_crypto::reset();
        crate::queues::reset_for_test();
    }

    /// Helper function to reset global state to known values for test isolation.
    /// This ensures each test starts with a clean state.
    /// Note: Call reset_advertisement_internal_state() separately, because the
    /// channel counter lives inside `handle_ble_advertisement_state`.
    fn reset_global_state() {
        reset_host_fakes();
        BLE_ADVERTISING_ENABLED.set(false);
        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Disconnected);
        MESH_DEVICE_ONLINE_STATUS.set(false);
        *CURRENT_RF_STATE.lock() = RfOperationState::MeshListening;
    }

    /// Helper function to reset the internal ST_PNO static counter in handle_ble_advertisement_state.
    /// The completion branch zeroes it, so drive that branch once and then
    /// discard the register writes it produced.
    fn reset_advertisement_internal_state() {
        // Save current state
        let saved_adv_enabled = BLE_ADVERTISING_ENABLED.get();
        let saved_online_status = MESH_DEVICE_ONLINE_STATUS.get();
        let saved_handler_state = *CURRENT_RF_STATE.lock();

        // Set state to trigger the completion path, which resets ST_PNO to 0
        BLE_ADVERTISING_ENABLED.set(false);
        MESH_DEVICE_ONLINE_STATUS.set(false);

        handle_ble_advertisement_state();

        // Restore original state
        BLE_ADVERTISING_ENABLED.set(saved_adv_enabled);
        MESH_DEVICE_ONLINE_STATUS.set(saved_online_status);
        *CURRENT_RF_STATE.lock() = saved_handler_state;

        // The reset run's register writes are not part of the test's subject.
        FakeMmio::clear_writes();
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
    /// - Short delay (100µs) is scheduled before listening
    /// - Online status flag is cleared after handling
    /// - State transitions to Listen mode
    #[test]
    fn test_advertisement_completion_with_online_status() {
        reset_global_state();
        FakeMmio::set_reg32(REG_SYSTEM_TICK, 10_000);
        reset_advertisement_internal_state();
        FakeMmio::set_reg32(REG_SYSTEM_TICK, 10_000);

        // Setup: Advertisement disabled (sequence complete) with online status
        BLE_ADVERTISING_ENABLED.set(false);
        MESH_DEVICE_ONLINE_STATUS.set(true);

        handle_ble_advertisement_state();

        // Verify link state is set to advertising mode initially
        assert_eq!(
            BLE_PERIPHERAL_LINK_STATE.get(),
            BlePeripheralLinkState::Advertising,
            "Link state should be set to advertising mode"
        );

        // Verify the radio was stopped and the DMA3 (RF TX) mode byte cleared.
        assert_eq!(
            read_reg_dma3_mode(),
            0,
            "the RF TX DMA mode byte must be cleared before stopping TRX"
        );
        assert_eq!(
            read_reg_rf_mode_control(),
            0x80,
            "rf_stop_trx must leave the radio in stop mode"
        );

        // Verify advertisement flag remains cleared
        assert!(
            !BLE_ADVERTISING_ENABLED.get(),
            "Advertisement flag should remain cleared after completion"
        );

        // Verify short delay is scheduled (100µs)
        let expected_tick = CLOCK_SYS_CLOCK_1US * 100 + 10_000;
        assert_eq!(
            read_reg_system_tick_irq(),
            expected_tick,
            "the 100µs completion delay must be armed"
        );

        // Verify online status flag is cleared
        assert!(
            !MESH_DEVICE_ONLINE_STATUS.get(),
            "Online status flag should be cleared after handling"
        );

        // Verify state transitions to Listen
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::MeshListening,
            "State should transition to Listen mode"
        );

        // Verify RF interrupt status is cleared
        assert_eq!(
            read_reg_rf_irq_status(),
            1,
            "the RX interrupt must be acknowledged"
        );
    }

    /// Tests advertisement sequence completion without online status reporting.
    ///
    /// Verifies that when advertisement is complete but no online status is scheduled:
    /// - Advertisement flag is cleared and channel counter is reset
    /// - No online status is sent
    /// - Longer delay (500µs) is scheduled before listening
    /// - State transitions to Listen mode
    #[test]
    fn test_advertisement_completion_without_online_status() {
        reset_global_state();
        FakeMmio::set_reg32(REG_SYSTEM_TICK, 15_000);
        reset_advertisement_internal_state();
        FakeMmio::set_reg32(REG_SYSTEM_TICK, 15_000);

        // Setup: Advertisement disabled (sequence complete) without online status
        BLE_ADVERTISING_ENABLED.set(false);
        MESH_DEVICE_ONLINE_STATUS.set(false);

        handle_ble_advertisement_state();

        // Verify link state is set to advertising mode initially
        assert_eq!(
            BLE_PERIPHERAL_LINK_STATE.get(),
            BlePeripheralLinkState::Advertising,
            "Link state should be set to advertising mode"
        );

        // Verify RF operations are stopped
        assert_eq!(
            read_reg_dma3_mode(),
            0,
            "the RF TX DMA mode byte must be cleared before stopping TRX"
        );
        assert_eq!(
            read_reg_rf_mode_control(),
            0x80,
            "rf_stop_trx must leave the radio in stop mode"
        );

        // Verify longer delay is scheduled (500µs)
        let expected_tick = CLOCK_SYS_CLOCK_1US * 500 + 15_000;
        assert_eq!(
            read_reg_system_tick_irq(),
            expected_tick,
            "the 500µs completion delay must be armed"
        );

        // Verify online status flag remains cleared
        assert!(
            !MESH_DEVICE_ONLINE_STATUS.get(),
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
    fn test_advertisement_continuation_first_channel() {
        reset_global_state();
        FakeMmio::set_reg32(REG_SYSTEM_TICK, 20_000);
        reset_advertisement_internal_state();
        FakeMmio::set_reg32(REG_SYSTEM_TICK, 20_000);

        // Setup: Advertisement enabled, first channel
        BLE_ADVERTISING_ENABLED.set(true);

        handle_ble_advertisement_state();

        // Verify link state is set to advertising mode
        assert_eq!(
            BLE_PERIPHERAL_LINK_STATE.get(),
            BlePeripheralLinkState::Advertising,
            "Link state should be set to advertising mode"
        );

        // Verify RF operations are stopped initially
        assert_eq!(
            read_reg_dma3_mode(),
            0,
            "the RF TX DMA mode byte must be cleared before stopping TRX"
        );

        // Verify BLE advertisement access code is set
        assert_eq!(
            read_reg_rf_access_code(),
            0xd6be898e,
            "the BLE advertisement access code must be programmed"
        );

        // Verify BLE advertisement CRC is set
        assert_eq!(
            read_reg_rf_crc(),
            0x555555,
            "the BLE advertisement CRC init must be programmed"
        );

        // Verify correct advertisement channel is set (first channel from SYS_CHN_ADV)
        assert_eq!(
            read_reg_rf_channel(),
            SYS_CHN_ADV[0],
            "the first advertisement channel must be selected"
        );

        // Verify advertisement interval timing is scheduled (1200µs = 0x4b0)
        let expected_tick = CLOCK_SYS_CLOCK_1US * 0x4b0 + 20_000;
        assert_eq!(
            read_reg_system_tick_irq(),
            expected_tick,
            "the 1200µs advertisement interval must be armed"
        );

        // Verify STX2RX mode is started (since we're not on last channel)
        assert_eq!(
            read_reg_rf_mode_control(),
            0x87,
            "the radio must be scheduled for stx2rx"
        );

        // Verify state transitions to Listen for next iteration
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::MeshListening,
            "State should transition to Listen mode"
        );

        // Verify RF interrupt status is cleared
        assert_eq!(
            read_reg_rf_irq_status(),
            1,
            "the RX interrupt must be acknowledged"
        );
    }

    /// Tests advertisement channel cycling through multiple calls.
    ///
    /// This test drives three consecutive calls: each one must select the next
    /// primary advertisement channel, and the third must close the sequence.
    #[test]
    fn test_advertisement_channel_cycling() {
        reset_global_state();
        FakeMmio::set_reg32(REG_SYSTEM_TICK, 25_000);
        reset_advertisement_internal_state();
        FakeMmio::set_reg32(REG_SYSTEM_TICK, 25_000);

        for (cycle, expected_channel) in SYS_CHN_ADV.iter().enumerate() {
            BLE_ADVERTISING_ENABLED.set(true);

            handle_ble_advertisement_state();

            assert_eq!(
                read_reg_rf_channel(),
                *expected_channel,
                "cycle {} must advertise on channel {}",
                cycle,
                expected_channel
            );

            // Verify link state is correct
            assert_eq!(
                BLE_PERIPHERAL_LINK_STATE.get(),
                BlePeripheralLinkState::Advertising,
                "Link state should be advertising"
            );

            if cycle < SYS_CHN_ADV.len() - 1 {
                // The flag stays set until the whole channel sequence is done.
                assert!(
                    BLE_ADVERTISING_ENABLED.get(),
                    "Advertisement flag should remain true after channel {}",
                    cycle
                );
            } else {
                assert!(
                    !BLE_ADVERTISING_ENABLED.get(),
                    "the last channel closes the advertisement sequence"
                );
            }

            // Verify state transitions to Listen
            assert_eq!(
                *CURRENT_RF_STATE.lock(),
                RfOperationState::MeshListening,
                "State should be in Listen mode"
            );
        }
    }

    /// Tests timing calculations with different system tick values.
    ///
    /// Verifies that timing calculations work correctly with various system tick values
    /// and that the proper delays are applied for different scenarios.
    #[test]
    fn test_timing_calculations() {
        reset_global_state();
        FakeMmio::set_reg32(REG_SYSTEM_TICK, 1000);
        reset_advertisement_internal_state();
        FakeMmio::set_reg32(REG_SYSTEM_TICK, 1000);

        // Test with online status enabled
        BLE_ADVERTISING_ENABLED.set(false);
        MESH_DEVICE_ONLINE_STATUS.set(true);

        handle_ble_advertisement_state();

        // Verify correct timing calculation for online status (100µs delay)
        let expected_tick = CLOCK_SYS_CLOCK_1US * 100 + 1000;
        assert_eq!(
            read_reg_system_tick_irq(),
            expected_tick,
            "the online-status completion delay must be 100µs"
        );

        // Verify online status flag is cleared
        assert!(
            !MESH_DEVICE_ONLINE_STATUS.get(),
            "Online status flag should be cleared after handling"
        );
    }

    /// Tests edge case behavior when advertisement is repeatedly enabled.
    ///
    /// This test verifies that the function handles rapid advertisement cycles gracefully.
    #[test]
    fn test_rapid_advertisement_cycles() {
        reset_global_state();
        FakeMmio::set_reg32(REG_SYSTEM_TICK, 30_000);
        reset_advertisement_internal_state();
        FakeMmio::set_reg32(REG_SYSTEM_TICK, 30_000);

        // Run multiple advertisement cycles rapidly
        for cycle in 0..3 {
            BLE_ADVERTISING_ENABLED.set(true);
            handle_ble_advertisement_state();

            // Verify each cycle works correctly
            assert_eq!(
                BLE_PERIPHERAL_LINK_STATE.get(),
                BlePeripheralLinkState::Advertising,
                "Link state should be advertising in cycle {}",
                cycle
            );

            // Advertisement remains enabled until the complete 3-channel sequence is done
            // Since we reset ST_PNO to 0 before this test, each call advances through channels 0,1,2
            // Only after channel 2 (the 3rd call) does BLE_ADVERTISING_ENABLED get cleared
            if cycle < 2 {
                assert!(
                    BLE_ADVERTISING_ENABLED.get(),
                    "Advertisement should remain enabled during sequence cycle {}",
                    cycle
                );
            } else {
                assert!(
                    !BLE_ADVERTISING_ENABLED.get(),
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
