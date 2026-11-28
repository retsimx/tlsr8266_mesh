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

use crate::ble::channel_selection::ble_ll_select_next_data_channel;
use crate::ble::connection::{
    BLE_PERIPHERAL_LINK_STATE, BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED,
    BRIDGE_RECEIVE_TIMING_TICK, SLAVE_LINK_INTERVAL, SLAVE_NEXT_CONNECT_TICK, SLAVE_WINDOW_SIZE,
};
use crate::coex::RadioCoexistence;
use crate::rf::drv_8266::*;
use crate::rf::drv_8266::{CURRENT_RF_STATE, PKT_EMPTY, PKT_INIT};
use crate::types::RfOperationState;
use tlsr8266_hal::common::sync::SimplifyLS;
use tlsr8266_hal::mcu::clock::{sleep_us, CLOCK_SYS_CLOCK_1US};
use tlsr8266_pac::*;

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
/// - Asks the coexistence implementation for a receive-window override
/// - Enables timing adjustment mechanisms
///
/// ## Channel Hopping:
/// - Selects next channel using BLE frequency hopping algorithm
/// - Uses connection-specific channel map and hop increment
pub fn configure_ble_receive_state<C: RadioCoexistence>() {
    // DMA3 (RF TX) mode byte (reg_dma3_ctrl[15:8]): bit7 is part of FIFO8 mode, exact role undocumented
    write_reg_dma3_mode(0x80);
    rf_stop_trx();

    // Set link state to BLE receive mode (state 7)
    BLE_PERIPHERAL_LINK_STATE.set(crate::types::BlePeripheralLinkState::Receiving);

    // RF TX wait/settle timing: low byte of the 32-bit reg_rf_tx_wail_settle_time (0xf04).
    // 0x67 = 32/48 MHz SYSTEM CLOCK (not the crystal frequency).
    write_reg_rf_tx_wail_settle_time_lo(0x67);

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

    // Configure interrupt timing: an override keeps the receiver open longer,
    // otherwise the window size (or the 0xed8 = ~3.8ms minimum) applies. The
    // policy behind the override lives in the platform.
    match C::receive_window_override_us() {
        None => {
            // Normal operation: use window size or minimum timing (0xed8 = ~3.8ms)
            if SLAVE_WINDOW_SIZE.get() == 0
                || SLAVE_WINDOW_SIZE.get() <= CLOCK_SYS_CLOCK_1US * 0xed8
            {
                write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 0xed8 + read_reg_system_tick());
            } else {
                write_reg_system_tick_irq(read_reg_system_tick() + SLAVE_WINDOW_SIZE.get());
            }
        }
        Some(us) => write_reg_system_tick_irq(us * CLOCK_SYS_CLOCK_1US + read_reg_system_tick()),
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
    use crate::types::{BlePeripheralLinkState, RfOperationState};

    use crate::ble::connection::BLE_PERIPHERAL_CONNECTION_INSTANT;
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_pac::{write_reg_system_tick, write_reg_system_tick_irq, REG_BASE_ADDR};

    /// Datasheet offsets of the registers `configure_ble_receive_state` drives.
    const REG_RF_ACCESS_CODE: u32 = 0x408;
    const REG_RF_CHANNEL: u32 = 0x40d;
    const REG_RF_CRC: u32 = 0x44c;
    const REG_DMA3_MODE: u32 = 0x50f;
    const REG_RF_MODE_CONTROL: u32 = 0xf00;
    const REG_RF_TX_WAIL_SETTLE_TIME_LO: u32 = 0xf04;
    const REG_SYSTEM_TICK_IRQ: u32 = 0x744;

    /// Values written to `offset` since the last clear, oldest first.
    fn writes_to(offset: u32) -> Vec<u32> {
        let addr = offset | REG_BASE_ADDR;
        FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == addr)
            .map(|w| w.value)
            .collect()
    }

    /// Number of writes of exactly `value` to `offset`.
    fn count_writes(offset: u32, value: u32) -> usize {
        writes_to(offset)
            .into_iter()
            .filter(|written| *written == value)
            .count()
    }

    /// Reset the PAC/HAL host fakes so a test never inherits another test's
    /// register bank, analog bank, clock or AES transport (the suite runs
    /// `--test-threads=1`, so all of these are process-wide singletons).
    fn reset_host_fakes() {
        FakeMmio::reset();
        tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog::reset();
        tlsr8266_hal::mcu::clock::fake_clock::reset();
        tlsr8266_hal::mcu::crypto::fake_crypto::reset();
    }

    /// Helper function to reset global state to known values for test isolation.
    fn reset_global_state() {
        reset_host_fakes();
        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Disconnected);
        SLAVE_LINK_INTERVAL.set(100000); // 100ms default
        SLAVE_WINDOW_SIZE.set(50000); // 50ms default
        SLAVE_NEXT_CONNECT_TICK.set(0);
        BRIDGE_RECEIVE_TIMING_TICK.set(0);
        crate::coex::TestRadioCoexistence::reset();
        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(false);
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(0);
        *CURRENT_RF_STATE.lock() = RfOperationState::MeshListening;

        // Reset PKT_INIT structure with mock values
        let mut pkt_init = PKT_INIT.lock();
        pkt_init.ll_init_mut().chm = [0xFF, 0xFF, 0xFF, 0xFF, 0x1F]; // All channels enabled (5 bytes)
        pkt_init.ll_init_mut().hop = 0x12; // Hop increment
        pkt_init.ll_init_mut().aa = [0xAA, 0xBB, 0xCC, 0xDD]; // Access address
        pkt_init.ll_init_mut().crcinit = [0x11, 0x22, 0x33]; // CRC init
        drop(pkt_init);

        // Arm the real clock registers the way the hardware would on entry to
        // the receive state: tick 50000, compare register 100000.
        write_reg_system_tick(50000);
        write_reg_system_tick_irq(100000);
        // Seeding is setup, not behaviour under test, so drop its log entries:
        // the assertions below must only see writes made by the function.
        FakeMmio::clear_writes();
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
    fn test_configure_ble_receive_state_basic() {
        // Reset state for clean test
        reset_global_state();

        // Execute function
        configure_ble_receive_state::<crate::coex::TestRadioCoexistence>();

        // Verify link state was set to receiving
        assert_eq!(
            BLE_PERIPHERAL_LINK_STATE.get(),
            BlePeripheralLinkState::Receiving,
            "Link state should be set to receiving"
        );

        // Verify RF operations were called
        assert_eq!(
            count_writes(REG_RF_MODE_CONTROL, 0x80),
            1,
            "rf_stop_trx should run exactly once"
        );
        assert_eq!(
            writes_to(REG_RF_CHANNEL).len(),
            1,
            "the hopper's channel selection should reach the radio exactly once"
        );

        // Verify register operations
        assert_eq!(
            FakeMmio::reg8(REG_DMA3_MODE | REG_BASE_ADDR),
            0x80,
            "DMA3 (RF TX) mode byte"
        );
        assert_eq!(
            FakeMmio::reg8(REG_RF_TX_WAIL_SETTLE_TIME_LO | REG_BASE_ADDR),
            0x67,
            "RF TX wait/settle time"
        );

        // Verify access code calculation: ((aa[2] << 8) | (aa[1] << 16) | aa[3] | (aa[0] << 24))
        // aa = [0xAA, 0xBB, 0xCC, 0xDD]
        // Expected: (0xCC << 8) | (0xBB << 16) | 0xDD | (0xAA << 24) = 0xAABBCCDD
        assert_eq!(
            FakeMmio::reg32(REG_RF_ACCESS_CODE | REG_BASE_ADDR),
            0xAABBCCDD,
            "access code assembled from aa"
        );

        // Verify CRC calculation: ((crcinit[1] << 8) | (crcinit[2] << 16) | crcinit[0])
        // crcinit = [0x11, 0x22, 0x33]
        // Expected: (0x22 << 8) | (0x33 << 16) | 0x11 = 0x00332211
        assert_eq!(
            FakeMmio::reg32(REG_RF_CRC | REG_BASE_ADDR),
            0x00332211,
            "CRC init assembled from crcinit"
        );

        // Verify the real connection-parameter handler ran: it bumps the event
        // counter on every call.
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_INSTANT.get(),
            1,
            "handle_ble_connection_parameter_updates should run once"
        );

        // Verify state transition
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Connected,
            "State should transition to bridge mode"
        );

        // Verify timing adjustment is enabled
        assert!(
            BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.get(),
            "Timing adjustment should be enabled"
        );
    }

    /// Tests BLE receive state configuration with the normal window.
    ///
    /// Verifies that when the coexistence implementation asks for no override:
    /// - Uses window size if it's greater than minimum timing
    /// - Uses minimum timing (0xed8 = ~3.8ms) if window size is too small
    /// - Calculates correct next connection event timing
    #[test]
    fn test_configure_ble_receive_state_normal_timing() {
        // Reset state for clean test
        reset_global_state();

        SLAVE_WINDOW_SIZE.set(150000); // 150ms, greater than minimum (121600)

        // Execute function
        configure_ble_receive_state::<crate::coex::TestRadioCoexistence>();

        // Verify timing calculation: current_tick + window_size
        // read_reg_system_tick() reads 50000, window_size is 150000
        let expected_timing = 50000 + 150000;
        assert_eq!(
            writes_to(REG_SYSTEM_TICK_IRQ),
            [expected_timing],
            "the compare register should be armed once with tick + window size"
        );

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
    fn test_configure_ble_receive_state_minimum_timing() {
        // Reset state for clean test
        reset_global_state();

        // Setup: Window size too small (less than 0xed8 * CLOCK_SYS_CLOCK_1US)
        SLAVE_WINDOW_SIZE.set(1000); // Very small, should trigger minimum timing

        // Execute function
        configure_ble_receive_state::<crate::coex::TestRadioCoexistence>();

        // Verify minimum timing is used: 0xed8 * CLOCK_SYS_CLOCK_1US + current_tick
        // 0xed8 = 3800, CLOCK_SYS_CLOCK_1US = 32, current_tick = 50000
        let expected_min_timing = 3800 * 32 + 50000;
        assert_eq!(
            writes_to(REG_SYSTEM_TICK_IRQ),
            [expected_min_timing],
            "the compare register should be armed once with the minimum window"
        );
    }

    /// Tests the receive-window override path.
    ///
    /// Verifies that when the coexistence implementation asks for a longer
    /// window:
    /// - The override is used instead of the normal timing
    /// - The window-size setting is ignored
    /// - The override is converted from microseconds with the system clock
    #[test]
    fn test_configure_ble_receive_state_window_override() {
        // Reset state for clean test
        reset_global_state();

        // Setup: A 10 ms override, exactly what the platform adapter asks for
        // while a GATT transfer is running.
        crate::coex::TestRadioCoexistence::set_receive_window_override_for_test(10_000);
        SLAVE_WINDOW_SIZE.set(75000); // ignored while the override is set

        // Execute function
        configure_ble_receive_state::<crate::coex::TestRadioCoexistence>();

        // Verify the override is used: 10000 * CLOCK_SYS_CLOCK_1US + current_tick
        // 10000 * 32 + 50000 = 370000
        let expected_override_timing = 10_000 * 32 + 50_000;
        assert_eq!(
            writes_to(REG_SYSTEM_TICK_IRQ),
            [expected_override_timing],
            "the compare register should be armed once with the override window"
        );
    }
}
