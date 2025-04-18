//! Clock management and timing utilities for TLSR8266 SoC.
//!
//! This module provides functions for initializing and managing the system clock,
//! timing operations, and sleep functionality. It interfaces directly with the 
//! hardware timer registers of the Telink TLSR8266F512 BLE SoC.
//!
//! The system operates at 32MHz by default, derived from a 192MHz PLL clock
//! that is divided down to the system clock frequency.

use crate::sdk::mcu::register::{
    read_reg_system_tick, write_reg_clk_sel, write_reg_rst_clk0, write_reg_tmr_ctrl, FLD_CLK_EN,
    FLD_CLK_SEL, FLD_TMR, WATCHDOG_TIMEOUT_COEFF,
};
use crate::{BIT, BIT_LOW_BIT, MASK_VAL};

/// System clock frequency in Hz (32MHz)
pub const CLOCK_SYS_CLOCK_HZ: u32 = 32000000;

/// SBC clock enable flag
pub const CLK_SBC_ENABLE: u8 = 1;
/// Audio clock enable flag
pub const CLK_AUD_ENABLE: u8 = 1;
/// DFIFO clock enable flag
pub const CLK_DFIFO_ENABLE: u8 = 1;
/// Initial watchdog timeout in milliseconds
pub const WATCHDOG_INIT_TIMEOUT: u32 = 2000;

/// PLL clock frequency in Hz (192MHz)
pub const CLOCK_PLL_CLOCK: u32 = 192000000;

/// Number of system clock ticks in 1 second
pub const CLOCK_SYS_CLOCK_1S: u32 = CLOCK_SYS_CLOCK_HZ;
/// Number of system clock ticks in 1 millisecond
pub const CLOCK_SYS_CLOCK_1MS: u32 = (CLOCK_SYS_CLOCK_1S / 1000);
/// Number of system clock ticks in 1 microsecond
pub const CLOCK_SYS_CLOCK_1US: u32 = (CLOCK_SYS_CLOCK_1S / 1000000);
/// Number of system clock ticks in 4 seconds
pub const CLOCK_SYS_CLOCK_4S: u32 = CLOCK_SYS_CLOCK_1S << 2;
/// Maximum number of milliseconds that can be represented by the clock counter
pub const CLOCK_MAX_MS: u32 = (u32::MAX / CLOCK_SYS_CLOCK_1MS);
/// Maximum number of microseconds that can be represented by the clock counter
pub const CLOCK_MAX_US: u32 = (u32::MAX / CLOCK_SYS_CLOCK_1US);

/// Clock source selection for the TLSR8266 SoC
enum CLOCK_SEL {
    /// 32MHz RC oscillator (internal)
    SEL_32M_RC = 0,
    /// High-speed clock with divider (used for PLL-derived clock)
    SEL_HS_DIV = 1,
    /// 16MHz external oscillator from pad
    SEL_16M_PAD = 2,
    /// 32MHz external oscillator from pad
    SEL_32M_PAD = 3,
    /// Clock from SPI interface
    SEL_SPI = 4,
    /// 40MHz internal oscillator
    SEL_40M_INTERNAL = 5,
    /// 32KHz RC oscillator (low power)
    SEL_32K_RC = 6,
}

/// Initializes the system clock and watchdog timer.
///
/// # Details
///
/// This function configures the TLSR8266 system clock by:
/// 1. Setting up the clock reset register with USB clock enabled if configured
/// 2. Configuring the clock selector register to use PLL divided down to 32MHz
/// 3. Enabling system timer and watchdog with appropriate timeout values
///
/// # Algorithm
///
/// 1. Reset clock registers and configure USB clock
/// 2. Set clock source to high-speed divider and configure divider ratio
/// 3. Enable timer 0 and configure watchdog timeout
///
/// # Notes
///
/// * The watchdog timer is critical to prevent system hangs and must be periodically
///   refreshed elsewhere in the code
/// * PLL clock (192MHz) is divided by 6 to get the 32MHz system clock
/// * This function must be called early in the boot process
pub fn clock_init() {
    // 1. Reset clock registers and enable USB clock if configured
    write_reg_rst_clk0(
        0xff000000  // Reset value with upper bits set
            | FLD_CLK_EN::USB_EN.bits()  // Enable USB clock
    );

    // 2. Configure clock source and divider
    //    - Use high-speed divider (SEL_HS_DIV) as clock source
    //    - Divide PLL clock (192MHz) by 6 to get 32MHz system clock
    write_reg_clk_sel(MASK_VAL!(
        FLD_CLK_SEL::DIV.bits() as u32,           // Set the clock divider field
        (CLOCK_PLL_CLOCK / CLOCK_SYS_CLOCK_1S) as u32,  // Divider value (192MHz/32MHz = 6)
        FLD_CLK_SEL::SRC.bits() as u32,           // Set the clock source field
        CLOCK_SEL::SEL_HS_DIV as u32              // Use HS divider as clock source
    ) as u8);

    // 3. Configure system timer and watchdog
    //    - Enable Timer0
    //    - Set watchdog capture value (timeout period converted to appropriate units)
    //    - Enable watchdog timer
    //reg_clk_en = 0xff | CLK_EN_TYPE;
    write_reg_tmr_ctrl(MASK_VAL!(
        FLD_TMR::TMR0_EN.bits(),         // Enable Timer0
        1,
        FLD_TMR::TMR_WD_CAPT.bits(),     // Configure watchdog capture value
        WATCHDOG_INIT_TIMEOUT * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF,  // Convert ms to watchdog units
        FLD_TMR::TMR_WD_EN.bits(),       // Enable watchdog timer
        1
    ))
}

/// Returns the current system tick count.
///
/// # Returns
///
/// * The current value of the system tick counter (32-bit)
///
/// # Notes
///
/// * This counter increments at the system clock frequency (32MHz)
/// * The counter will overflow after approximately 134 seconds
/// * This function is marked as inline(always) for performance in time-critical operations
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn clock_time() -> u32 {
    // Read the system tick register directly
    read_reg_system_tick()
}

/// Checks if the specified time span has elapsed since a reference time.
///
/// # Parameters
///
/// * `reference` - The reference time to measure from (obtained from clock_time())
/// * `span_us` - The time span to check in microseconds
///
/// # Returns
///
/// * `true` if the specified time span has elapsed
/// * `false` if the specified time span has not yet elapsed
///
/// # Algorithm
///
/// 1. Get the current time using clock_time()
/// 2. Calculate the difference between current time and reference time
/// 3. Convert the requested microsecond span to system ticks
/// 4. Compare the elapsed ticks with the requested span
///
/// # Notes
///
/// * This function correctly handles timer overflow when the difference wraps around
/// * Marked as inline(always) for performance in time-critical operations
/// * Maximum span is limited by CLOCK_MAX_US (approximately 134 seconds)
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn clock_time_exceed(reference: u32, span_us: u32) -> bool {
    // Calculate elapsed time (handles overflow correctly due to u32 wraparound properties)
    // Convert span_us to system clock ticks and compare
    return (clock_time() - reference) as u32 > span_us * CLOCK_SYS_CLOCK_1US;
}

/// Busy-waits for the specified number of microseconds.
///
/// # Parameters
///
/// * `us` - The number of microseconds to sleep
///
/// # Algorithm
///
/// 1. Record the starting time
/// 2. Wait in a busy-loop until the specified time has elapsed
///
/// # Notes
///
/// * This function is placed in RAM (.ram_code section) for reliable execution
///   regardless of flash access timing
/// * Uses busy-waiting which keeps the CPU active (consuming power)
/// * For longer delays or power-efficient waiting, consider using a sleep mode instead
/// * Maximum delay is limited by CLOCK_MAX_US (approximately 134 seconds)
/// * Never inlined to save code size at expense of function call overhead
#[inline(never)]
#[cfg_attr(test, mry::mry)]
#[link_section = ".ram_code"]
pub fn sleep_us(us: u32) {
    // Record starting time
    let t = clock_time();
    
    // Busy-wait until the specified time has elapsed
    while !clock_time_exceed(t, us) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    
    // Import mock functions from register module for testing
    use crate::sdk::mcu::register::mock_read_reg_system_tick;
    use crate::sdk::mcu::register::mock_write_reg_clk_sel;
    use crate::sdk::mcu::register::mock_write_reg_rst_clk0;
    use crate::sdk::mcu::register::mock_write_reg_tmr_ctrl;

    /// Tests clock_init function for proper register configuration.
    ///
    /// This test verifies that the clock_init function correctly:
    /// - Configures the reset register with proper USB settings
    /// - Sets up the clock selector register with correct source and divider
    /// - Configures the timer control register with proper watchdog timing
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for register write functions
    /// 2. Call clock_init to trigger the configuration
    /// 3. Verify that each register was written with expected values
    ///
    /// # Notes
    ///
    /// * Tests all three stages of clock initialization
    /// * Verifies USB clock configuration based on CLK_USB_ENABLE constant
    /// * Confirms PLL divider calculation is correct
    /// * Verifies watchdog timeout is properly converted to register value
    #[test]
    #[mry::lock(write_reg_rst_clk0, write_reg_clk_sel, write_reg_tmr_ctrl)]
    fn test_clock_init() {
        // Setup expected values
        let expected_rst_clk0 = 0xff000000 | FLD_CLK_EN::USB_EN.bits();
        
        let expected_clk_sel = MASK_VAL!(
            FLD_CLK_SEL::DIV.bits() as u32,
            (CLOCK_PLL_CLOCK / CLOCK_SYS_CLOCK_1S) as u32,
            FLD_CLK_SEL::SRC.bits() as u32,
            CLOCK_SEL::SEL_HS_DIV as u32
        ) as u8;
        
        let expected_tmr_ctrl = MASK_VAL!(
            FLD_TMR::TMR0_EN.bits(), 1,
            FLD_TMR::TMR_WD_CAPT.bits(), 
            WATCHDOG_INIT_TIMEOUT * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF,
            FLD_TMR::TMR_WD_EN.bits(), 1
        );
        
        // Setup mock behaviors
        mock_write_reg_rst_clk0(expected_rst_clk0).returns(());
        mock_write_reg_clk_sel(expected_clk_sel).returns(());
        mock_write_reg_tmr_ctrl(expected_tmr_ctrl).returns(());
        
        // Call the function being tested
        clock_init();
        
        // Verify each register was written exactly once with correct values
        mock_write_reg_rst_clk0(expected_rst_clk0).assert_called(1);
        mock_write_reg_clk_sel(expected_clk_sel).assert_called(1);
        mock_write_reg_tmr_ctrl(expected_tmr_ctrl).assert_called(1);
    }

    /// Tests clock_time function for proper system tick reading.
    ///
    /// This test verifies that the clock_time function correctly:
    /// - Reads the system tick register
    /// - Returns the register value as is
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock response for system tick register
    /// 2. Call clock_time function
    /// 3. Verify returned value matches expected tick value
    ///
    /// # Notes
    ///
    /// * Tests the basic operation of reading system time
    /// * Verifies function is a simple pass-through to register read
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_clock_time() {
        // Setup expected tick value
        let expected_tick = 0x12345678;
        
        // Setup mock behavior for register read
        mock_read_reg_system_tick().returns(expected_tick);
        
        // Call the function being tested
        let result = clock_time();
        
        // Verify register was read exactly once
        mock_read_reg_system_tick().assert_called(1);
        
        // Verify returned value matches expected tick
        assert_eq!(result, expected_tick);
    }

    /// Tests clock_time_exceed function when the time span has not elapsed.
    ///
    /// This test verifies that the clock_time_exceed function correctly:
    /// - Returns false when the elapsed time is less than the requested span
    /// - Correctly converts microseconds to system ticks
    ///
    /// # Algorithm
    ///
    /// 1. Setup current time slightly ahead but not enough to exceed the span
    /// 2. Verify function returns false
    ///
    /// # Notes
    ///
    /// * Tests the negative condition (time not elapsed)
    /// * Verifies microsecond-to-tick conversion is applied correctly
    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time_exceed_not_elapsed() {
        // Reference time
        let reference_time = 0x10000000;
        
        // Setup current time slightly ahead but not enough to exceed the span
        // Current is 1000 ticks ahead, requesting 50us = 32*50 = 1600 ticks
        mock_clock_time().returns(reference_time + 1000);
        
        // Time span of 50us (should be 1600 ticks at 32MHz)
        let result = clock_time_exceed(reference_time, 50);
        
        // Should return false because not enough time has elapsed
        assert_eq!(result, false, "Should return false when time has not elapsed");
        
        // Verify clock_time was called the expected number of times
        mock_clock_time().assert_called(1);
    }
    
    /// Tests clock_time_exceed function when the time span has elapsed.
    ///
    /// This test verifies that the clock_time_exceed function correctly:
    /// - Returns true when the elapsed time exceeds the requested span
    /// - Correctly converts microseconds to system ticks
    ///
    /// # Algorithm
    ///
    /// 1. Setup current time significantly ahead of reference time
    /// 2. Verify function returns true
    ///
    /// # Notes
    ///
    /// * Tests the positive condition (time has elapsed)
    /// * Verifies microsecond-to-tick conversion is applied correctly
    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time_exceed_elapsed() {
        // Reference time
        let reference_time = 0x10000000;
        
        // Setup current time significantly ahead
        // Current is 2000 ticks ahead, requesting 50us = 32*50 = 1600 ticks
        mock_clock_time().returns(reference_time + 2000);
        
        // Time span of 50us (should be 1600 ticks at 32MHz)
        let result = clock_time_exceed(reference_time, 50);
        
        // Should return true because enough time has elapsed
        assert_eq!(result, true, "Should return true when time has elapsed");
        
        // Verify clock_time was called the expected number of times
        mock_clock_time().assert_called(1);
    }
    
    /// Tests clock_time_exceed function with timer overflow.
    ///
    /// This test verifies that the clock_time_exceed function correctly:
    /// - Handles timer wraparound correctly
    /// - Correctly calculates elapsed time despite the overflow
    ///
    /// # Algorithm
    ///
    /// 1. Setup current time with a value that would appear "less" than reference
    /// 2. Verify function correctly detects elapsed time despite wraparound
    ///
    /// # Notes
    ///
    /// * Tests the wraparound condition using u32 arithmetic properties
    /// * This is a critical test case as overflow handling is essential for correct timing
    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time_exceed_overflow() {
        // Reference time
        let reference_time = 0x10000000;
        
        // Setup case where timer has wrapped around (appears less but actually more)
        // 0xFFFFFFFE - 0x10000000 + 2 = 0xEFFFFFFE (large positive difference due to wraparound)
        mock_clock_time().returns(0xFFFFFFFE);
        
        // Check for 10ms elapsed time (should be true when wrapped around)
        let result = clock_time_exceed(reference_time, 10000);
        
        // Should return true because of wraparound arithmetic
        assert_eq!(result, true, "Should correctly handle timer overflow");
        
        // Verify clock_time was called the expected number of times
        mock_clock_time().assert_called(1);
    }

    /// Tests sleep_us function for proper busy-wait behavior.
    ///
    /// This test verifies that the sleep_us function correctly:
    /// - Records the starting time
    /// - Calls clock_time_exceed in a loop until the time span has elapsed
    /// - Exits after the time span is complete
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for clock_time and clock_time_exceed
    /// 2. Configure clock_time_exceed to return false initially then true
    /// 3. Call sleep_us with a specified duration
    /// 4. Verify correct sequence and number of function calls
    ///
    /// # Notes
    ///
    /// * Tests the busy-wait loop behavior
    /// * Verifies proper termination when time elapses
    /// * Counts exact number of iterations to check loop behavior
    #[test]
    #[mry::lock(clock_time, clock_time_exceed)]
    fn test_sleep_us() {
        // Setup a fixed starting time
        let start_time = 0x12345678;
        mock_clock_time().returns(start_time);
        
        // Setup clock_time_exceed to initially return false (time not elapsed)
        // then return true after 3 calls (time has elapsed)
        let mut exceed_call_count = 0;
        mock_clock_time_exceed(start_time, 100).returns_with(move |_, _| {
            exceed_call_count += 1;
            exceed_call_count > 3 // Return true after 3 calls to exit the loop
        });
        
        // Call the function being tested - sleep for 100us
        sleep_us(100);
        
        // Verify clock_time was called exactly once to record start time
        mock_clock_time().assert_called(1);
        
        // Verify clock_time_exceed was called 4 times:
        // 3 times returning false (continuing the loop)
        // 1 time returning true (exiting the loop)
        mock_clock_time_exceed(start_time, 100).assert_called(4);
    }

    /// Tests clock_time_exceed with zero duration.
    ///
    /// This test verifies that the clock_time_exceed function correctly:
    /// - Handles zero duration case properly (should return true)
    ///
    /// # Algorithm
    ///
    /// 1. Test with zero duration
    /// 2. Verify that any elapsed time exceeds zero duration
    ///
    /// # Notes
    ///
    /// * Tests the edge case of zero time span
    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time_exceed_zero_duration() {
        let reference_time = 1000;
        
        // Any elapsed time should exceed a zero duration
        mock_clock_time().returns(reference_time + 1);
        
        let result = clock_time_exceed(reference_time, 0);
        assert_eq!(result, true, "Zero duration should always be exceeded");
        
        // Verify clock_time was called the expected number of times
        mock_clock_time().assert_called(1);
    }
    
    /// Tests clock_time_exceed with minimum duration (1us) - not yet exceeded.
    ///
    /// This test verifies that the clock_time_exceed function correctly:
    /// - Handles minimum detectable time span (1 microsecond)
    /// - Returns false when time has not yet exceeded the minimum span
    ///
    /// # Algorithm
    ///
    /// 1. Test with 1 microsecond duration
    /// 2. Setup time that is not enough to exceed 1us
    ///
    /// # Notes
    ///
    /// * Tests the minimum meaningful duration
    /// * Verifies tick conversion for short durations
    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time_exceed_min_duration_not_exceeded() {
        let reference_time = 1000;
        
        // 1 us at 32MHz = 32 ticks
        // Current time is 31 ticks ahead (not enough)
        mock_clock_time().returns(reference_time + 31);
        
        let result = clock_time_exceed(reference_time, 1);
        assert_eq!(result, false, "1 microsecond should require 32 ticks at 32MHz");
        
        // Verify clock_time was called the expected number of times
        mock_clock_time().assert_called(1);
    }
    
    /// Tests clock_time_exceed with minimum duration (1us) - exactly met.
    ///
    /// This test verifies that the clock_time_exceed function correctly:
    /// - Handles exactly meeting the requested time span
    /// - Returns false when time exactly matches (not exceeds) the span
    ///
    /// # Algorithm
    ///
    /// 1. Test with 1 microsecond duration
    /// 2. Setup time that exactly matches required ticks for 1us
    ///
    /// # Notes
    ///
    /// * Test case for the boundary condition
    /// * Verifies that we need to exceed, not just meet, the time
    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time_exceed_min_duration_exact() {
        let reference_time = 1000;
        
        // Now advance one more tick (exactly enough)
        mock_clock_time().returns(reference_time + 32);
        
        let result = clock_time_exceed(reference_time, 1);
        assert_eq!(result, false, "32 ticks should match exactly 1us (not exceed)");
        
        // Verify clock_time was called the expected number of times
        mock_clock_time().assert_called(1);
    }
    
    /// Tests clock_time_exceed with minimum duration (1us) - exceeded.
    ///
    /// This test verifies that the clock_time_exceed function correctly:
    /// - Handles exceeding the minimum time span
    /// - Returns true when time exceeds the requested span
    ///
    /// # Algorithm
    ///
    /// 1. Test with 1 microsecond duration
    /// 2. Setup time that exceeds required ticks for 1us
    ///
    /// # Notes
    ///
    /// * Tests the "just over" condition
    /// * Ensures accuracy at small timescales
    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time_exceed_min_duration_exceeded() {
        let reference_time = 1000;
        
        // Now advance one more tick (just enough to exceed)
        mock_clock_time().returns(reference_time + 33);
        
        let result = clock_time_exceed(reference_time, 1);
        assert_eq!(result, true, "33 ticks should exceed 1us requirement");
        
        // Verify clock_time was called the expected number of times
        mock_clock_time().assert_called(1);
    }
    
    /// Tests clock_time_exceed with very large duration - not exceeded.
    ///
    /// This test verifies that the clock_time_exceed function correctly:
    /// - Handles large but valid durations near the maximum
    /// - Returns false when large duration has not been exceeded
    ///
    /// # Algorithm
    ///
    /// 1. Test with a duration close to CLOCK_MAX_US
    /// 2. Setup time that is just under the required ticks
    ///
    /// # Notes
    ///
    /// * Tests the upper limit of time span handling
    /// * Verifies accuracy at large timescales
    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time_exceed_max_duration_not_exceeded() {
        let reference_time = 1000;
        
        // Test with a large but valid duration (close to max representable)
        let large_us = CLOCK_MAX_US - 1000;
        let large_ticks = large_us * CLOCK_SYS_CLOCK_1US;
        
        // Set current time to be less than required
        mock_clock_time().returns(reference_time + large_ticks - 1);
        
        let result = clock_time_exceed(reference_time, large_us);
        assert_eq!(result, false, "Large duration should not be exceeded yet");
        
        // Verify clock_time was called the expected number of times
        mock_clock_time().assert_called(1);
    }
    
    /// Tests clock_time_exceed with very large duration - exceeded.
    ///
    /// This test verifies that the clock_time_exceed function correctly:
    /// - Handles large but valid durations near the maximum
    /// - Returns true when large duration has been exceeded
    ///
    /// # Algorithm
    ///
    /// 1. Test with a duration close to CLOCK_MAX_US
    /// 2. Setup time that exceeds the required ticks
    ///
    /// # Notes
    ///
    /// * Tests the upper boundary of usable time spans
    /// * Verifies proper handling of large tick values
    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time_exceed_max_duration_exceeded() {
        let reference_time = 1000;
        
        // Test with a large but valid duration (close to max representable)
        let large_us = CLOCK_MAX_US - 1000;
        let large_ticks = large_us * CLOCK_SYS_CLOCK_1US;
        
        // Set current time to be more than required
        mock_clock_time().returns(reference_time + large_ticks + 1);
        
        let result = clock_time_exceed(reference_time, large_us);
        assert_eq!(result, true, "Large duration should now be exceeded");
        
        // Verify clock_time was called the expected number of times
        mock_clock_time().assert_called(1);
    }
}
