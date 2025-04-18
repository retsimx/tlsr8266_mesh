use crate::sdk::mcu::clock::CLOCK_SYS_CLOCK_1MS;
use crate::sdk::mcu::register::WATCHDOG_TIMEOUT_COEFF;
use crate::sdk::mcu::register::{read_reg_tmr_ctrl, write_reg_tmr2_tick, write_reg_tmr_ctrl, FLD_TMR};
use crate::CLR_FLD;
use crate::{
    BIT, BIT_LOW_BIT, BM_CLR, BM_MASK_VAL, BM_SET, BM_SET_MASK_FLD, MASK_VAL, SET_FLD, SET_FLD_V,
};

/// Watchdog Timer Module
///
/// This module provides control functions for the hardware watchdog timer,
/// which uses Timer 2 of the TLSR8266 microcontroller. The watchdog timer
/// triggers a system reset when it reaches its timeout value, protecting
/// the system from software hangs or infinite loops.
///
/// The timer operates based on the system clock divided by the watchdog timeout
/// coefficient defined in the register module.

/// Sets the watchdog timer timeout interval.
///
/// Configures Timer 2 to operate as a watchdog timer with the specified timeout
/// period. The timeout is calculated by converting milliseconds to watchdog timer
/// ticks based on the system clock frequency and the watchdog coefficient.
///
/// # Parameters
///
/// * `ms` - Timeout interval in milliseconds. Must result in a non-zero timer value
///          after conversion to timer ticks.
///
/// # Notes
///
/// * Uses Timer 2 in watchdog mode (mode 0)
/// * Resets the timer tick counter to 0 after configuration
/// * Does not enable the watchdog - call `wd_start()` after setting the interval
/// * Asserts if the requested timeout would result in a zero timer value
#[cfg_attr(test, mry::mry)]
pub fn wd_set_interval(ms: u32) //  in ms
{
    // Verify that the timeout value will be non-zero after conversion to timer ticks
    // A zero timer value would cause immediate timeout, which is not useful
    assert!((ms * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF) > 0);
    
    // Read the current timer control register value
    let mut val = read_reg_tmr_ctrl();
    
    // Configure two fields in the timer control register:
    // 1. Set the watchdog capture value (timeout) based on the ms parameter
    // 2. Set Timer 2 mode to 0 (watchdog mode)
    SET_FLD_V!(
        val,
        FLD_TMR::TMR_WD_CAPT.bits(),                       // Field 1: Watchdog capture/timeout value
        (ms * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF), // Converted to timer ticks
        FLD_TMR::TMR2_MODE.bits(),                         // Field 2: Timer 2 mode
        0                                                   // Mode 0: watchdog mode
    );
    
    // Write the updated configuration back to the timer control register
    write_reg_tmr_ctrl(val);
    
    // Reset the timer tick counter to start from 0
    write_reg_tmr2_tick(0);
}

/// Enables (starts) the watchdog timer.
///
/// Activates the previously configured watchdog timer by setting the watchdog
/// enable bit in the timer control register. Once started, the system will
/// reset if `wd_clear()` is not called before the timeout interval expires.
///
/// # Notes
///
/// * The watchdog interval must be configured with `wd_set_interval()` before calling this function
/// * After enabling, the watchdog must be periodically cleared with `wd_clear()` to prevent system reset
/// * This function is marked inline(always) for time-critical applications
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn wd_start() {
    // Read the current timer control register value
    let mut val = read_reg_tmr_ctrl();
    
    // Set the watchdog enable bit in the timer control register
    SET_FLD!(val, FLD_TMR::TMR_WD_EN.bits());
    
    // Write the updated value back to the timer control register
    write_reg_tmr_ctrl(val);
}

/// Disables (stops) the watchdog timer.
///
/// Deactivates the watchdog timer by clearing the watchdog enable bit
/// in the timer control register. This prevents the watchdog from
/// triggering a system reset.
///
/// # Notes
///
/// * Use this function when watchdog protection is no longer needed
/// * The timer configuration remains intact and can be re-enabled with `wd_start()`
/// * This function is marked inline(always) for time-critical applications
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn wd_stop() {
    // Read the current timer control register value
    let mut val = read_reg_tmr_ctrl();
    
    // Clear the watchdog enable bit in the timer control register
    CLR_FLD!(val, FLD_TMR::TMR_WD_EN.bits());
    
    // Write the updated value back to the timer control register
    write_reg_tmr_ctrl(val);
}

/// Resets (clears) the watchdog timer counter.
///
/// Resets the watchdog timer by setting the clear watchdog bit in the
/// timer control register. This prevents the system from resetting due
/// to a watchdog timeout. In normal operation, this function should be
/// called periodically, before the watchdog timeout occurs.
///
/// # Notes
///
/// * Must be called periodically to prevent system reset when watchdog is enabled
/// * The clear bit is automatically reset by hardware after the operation
/// * This function is marked inline(always) for time-critical applications
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn wd_clear() {
    // Read the current timer control register value
    let mut val = read_reg_tmr_ctrl();
    
    // Set the clear watchdog bit in the timer control register
    // This bit is self-clearing in hardware after the operation completes
    SET_FLD!(val, FLD_TMR::CLR_WD.bits());
    
    // Write the updated value back to the timer control register
    write_reg_tmr_ctrl(val);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdk::mcu::register::{
        mock_read_reg_tmr_ctrl, mock_write_reg_tmr2_tick, mock_write_reg_tmr_ctrl,
    };

    /// Tests the watchdog interval setting functionality.
    ///
    /// This test verifies that the wd_set_interval function correctly:
    /// - Calculates the proper timer tick value from milliseconds
    /// - Sets the watchdog capture value appropriately
    /// - Configures Timer 2 in watchdog mode (mode 0)
    /// - Resets the timer tick counter to zero
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for register operations
    /// 2. Call wd_set_interval with a test interval value
    /// 3. Verify the timer control register is updated with the correct values
    /// 4. Verify the timer tick counter is reset to zero
    ///
    /// # Notes
    ///
    /// * Tests conversion from milliseconds to timer ticks
    /// * Verifies Timer 2 is set to mode 0 (watchdog mode)
    #[test]
    #[mry::lock(read_reg_tmr_ctrl, write_reg_tmr_ctrl, write_reg_tmr2_tick)]
    fn test_wd_set_interval() {
        // Initial register value
        mock_read_reg_tmr_ctrl().returns(0x00000000);
        
        // Use static mutable variables to store values written to registers
        static mut WRITTEN_CTRL_VAL: u32 = 0;
        static mut WRITTEN_TICK_VAL: u32 = 0;
        
        // Capture the values written to the registers using the static variables
        mock_write_reg_tmr_ctrl(mry::Any).returns_with(|val: u32| {
            // Using unsafe because we're accessing static mutable variables
            unsafe { WRITTEN_CTRL_VAL = val; }
        });
        
        mock_write_reg_tmr2_tick(mry::Any).returns_with(|val: u32| {
            // Using unsafe because we're accessing static mutable variables
            unsafe { WRITTEN_TICK_VAL = val; }
        });
        
        // Call the function with a 1000ms (1 second) timeout
        wd_set_interval(1000);
        
        // Calculate the expected timer value (same calculation as in the function)
        let expected_timer_value = (1000 * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF) << 9;
        
        // Access static variables safely within an unsafe block
        unsafe {
            // Check that the timer control register was set correctly
            // - Timer 2 mode should be 0 (watchdog mode)
            // - Watchdog capture value should match our calculation
            assert_eq!(
                WRITTEN_CTRL_VAL & FLD_TMR::TMR2_MODE.bits(),
                0, // Mode 0 (watchdog mode)
                "Timer 2 mode should be set to 0 (watchdog mode)"
            );
            
            assert_eq!(
                WRITTEN_CTRL_VAL & FLD_TMR::TMR_WD_CAPT.bits(),
                expected_timer_value & FLD_TMR::TMR_WD_CAPT.bits(),
                "Watchdog capture value should be set correctly"
            );
            
            // Check that the timer tick counter was reset to zero
            assert_eq!(WRITTEN_TICK_VAL, 0, "Timer tick should be reset to 0");
        }
        
        // Verify all functions were called the expected number of times
        mock_read_reg_tmr_ctrl().assert_called(1);
        mock_write_reg_tmr_ctrl(mry::Any).assert_called(1);
        mock_write_reg_tmr2_tick(0).assert_called(1);
    }
    
    /// Tests the watchdog timer start functionality.
    ///
    /// This test verifies that the wd_start function correctly:
    /// - Reads the current timer control register value
    /// - Sets the watchdog enable bit without affecting other bits
    /// - Writes the updated value back to the timer control register
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for register operations with an initial value
    /// 2. Call wd_start to enable the watchdog
    /// 3. Verify the watchdog enable bit is set in the updated register value
    /// 4. Verify other register bits remain unchanged
    ///
    /// # Notes
    ///
    /// * Tests the atomic bit-set operation for enabling the watchdog
    /// * Confirms the register read-modify-write sequence is performed correctly
    #[test]
    #[mry::lock(read_reg_tmr_ctrl, write_reg_tmr_ctrl)]
    fn test_wd_start() {
        // Initial register value with some bits already set
        let initial_reg_value = 0x12345678;
        mock_read_reg_tmr_ctrl().returns(initial_reg_value);
        
        // Use static mutable variable to store value written to register
        static mut WRITTEN_VAL: u32 = 0;
        
        // Capture the value written to the register using the static variable
        mock_write_reg_tmr_ctrl(mry::Any).returns_with(|val: u32| {
            // Using unsafe because we're accessing static mutable variable
            unsafe { WRITTEN_VAL = val; }
        });
        
        // Call the function to start the watchdog
        wd_start();
        
        // Access static variable safely within an unsafe block
        unsafe {
            // Check that only the watchdog enable bit was set,
            // and all other bits remain unchanged
            assert_eq!(
                WRITTEN_VAL,
                initial_reg_value | FLD_TMR::TMR_WD_EN.bits(),
                "Only the watchdog enable bit should be set"
            );
        }
        
        // Verify all functions were called the expected number of times
        mock_read_reg_tmr_ctrl().assert_called(1);
        mock_write_reg_tmr_ctrl(mry::Any).assert_called(1);
    }
    
    /// Tests the watchdog timer stop functionality.
    ///
    /// This test verifies that the wd_stop function correctly:
    /// - Reads the current timer control register value
    /// - Clears the watchdog enable bit without affecting other bits
    /// - Writes the updated value back to the timer control register
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for register operations with an initial value
    /// 2. Call wd_stop to disable the watchdog
    /// 3. Verify the watchdog enable bit is cleared in the updated register value
    /// 4. Verify other register bits remain unchanged
    ///
    /// # Notes
    ///
    /// * Tests the atomic bit-clear operation for disabling the watchdog
    /// * Confirms the register read-modify-write sequence is performed correctly
    #[test]
    #[mry::lock(read_reg_tmr_ctrl, write_reg_tmr_ctrl)]
    fn test_wd_stop() {
        // Initial register value with watchdog enable bit already set
        let initial_reg_value = 0x12345678 | FLD_TMR::TMR_WD_EN.bits();
        mock_read_reg_tmr_ctrl().returns(initial_reg_value);
        
        // Use static mutable variable to store value written to register
        static mut WRITTEN_VAL: u32 = 0;
        
        // Capture the value written to the register using the static variable
        mock_write_reg_tmr_ctrl(mry::Any).returns_with(|val: u32| {
            // Using unsafe because we're accessing static mutable variable
            unsafe { WRITTEN_VAL = val; }
        });
        
        // Call the function to stop the watchdog
        wd_stop();
        
        // Access static variable safely within an unsafe block
        unsafe {
            // Check that only the watchdog enable bit was cleared,
            // and all other bits remain unchanged
            assert_eq!(
                WRITTEN_VAL,
                initial_reg_value & !FLD_TMR::TMR_WD_EN.bits(),
                "Only the watchdog enable bit should be cleared"
            );
        }
        
        // Verify all functions were called the expected number of times
        mock_read_reg_tmr_ctrl().assert_called(1);
        mock_write_reg_tmr_ctrl(mry::Any).assert_called(1);
    }
    
    /// Tests the watchdog timer clear functionality.
    ///
    /// This test verifies that the wd_clear function correctly:
    /// - Reads the current timer control register value
    /// - Sets the clear watchdog bit without affecting other bits
    /// - Writes the updated value back to the timer control register
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for register operations with an initial value
    /// 2. Call wd_clear to reset the watchdog counter
    /// 3. Verify the clear watchdog bit is set in the updated register value
    /// 4. Verify other register bits remain unchanged
    ///
    /// # Notes
    ///
    /// * Tests the atomic bit-set operation for clearing the watchdog
    /// * In hardware, the clear bit is self-clearing after the operation
    /// * Confirms the register read-modify-write sequence is performed correctly
    #[test]
    #[mry::lock(read_reg_tmr_ctrl, write_reg_tmr_ctrl)]
    fn test_wd_clear() {
        // Initial register value with some bits already set
        let initial_reg_value = 0x12345678;
        mock_read_reg_tmr_ctrl().returns(initial_reg_value);
        
        // Use static mutable variable to store value written to register
        static mut WRITTEN_VAL: u32 = 0;
        
        // Capture the value written to the register using the static variable
        mock_write_reg_tmr_ctrl(mry::Any).returns_with(|val: u32| {
            // Using unsafe because we're accessing static mutable variable
            unsafe { WRITTEN_VAL = val; }
        });
        
        // Call the function to clear the watchdog
        wd_clear();
        
        // Access static variable safely within an unsafe block
        unsafe {
            // Check that only the clear watchdog bit was set,
            // and all other bits remain unchanged
            assert_eq!(
                WRITTEN_VAL,
                initial_reg_value | FLD_TMR::CLR_WD.bits(),
                "Only the clear watchdog bit should be set"
            );
        }
        
        // Verify all functions were called the expected number of times
        mock_read_reg_tmr_ctrl().assert_called(1);
        mock_write_reg_tmr_ctrl(mry::Any).assert_called(1);
    }
}
