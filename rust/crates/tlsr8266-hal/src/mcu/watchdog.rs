//! Watchdog timer module
//!
//! This module provides control functions for the hardware watchdog timer,
//! which uses Timer 2 of the TLSR8266 microcontroller. The watchdog timer
//! triggers a system reset when it reaches its timeout value, protecting
//! the system from software hangs or infinite loops.
//!
//! The timer operates based on the system clock divided by the watchdog timeout
//! coefficient defined in the register module.

use crate::mcu::clock::CLOCK_SYS_CLOCK_1MS;
use crate::{CLR_FLD, SET_FLD, SET_FLD_V};
use tlsr8266_pac::WATCHDOG_TIMEOUT_COEFF;
use tlsr8266_pac::{read_reg_tmr_ctrl, write_reg_tmr2_tick, write_reg_tmr_ctrl, FLD_TMR};

/// Sets the watchdog timer timeout interval.
///
/// Configures Timer 2 to operate as a watchdog timer with the specified timeout
/// period. The timeout is calculated by converting milliseconds to watchdog timer
/// ticks based on the system clock frequency and the watchdog coefficient.
///
/// # Parameters
///
/// * `ms` - Timeout interval in milliseconds. Must result in a non-zero timer value
///   after conversion to timer ticks.
///
/// # Notes
///
/// * Uses Timer 2 in watchdog mode (mode 0)
/// * Resets the timer tick counter to 0 after configuration
/// * Does not enable the watchdog - call `wd_start()` after setting the interval
/// * Asserts if the requested timeout would result in a zero timer value
pub fn wd_set_interval(ms: u32) //  in ms
{
    // Verify that the timeout value will be non-zero after conversion to timer ticks
    // A zero timer value would cause immediate timeout, which is not useful
    assert!(((ms * CLOCK_SYS_CLOCK_1MS) >> WATCHDOG_TIMEOUT_COEFF) > 0);

    // Read the current timer control register value
    let mut val = read_reg_tmr_ctrl();

    // Configure two fields in the timer control register:
    // 1. Set the watchdog capture value (timeout) based on the ms parameter
    // 2. Set Timer 2 mode to 0 (watchdog mode)
    SET_FLD_V!(
        val,
        FLD_TMR::TMR_WD_CAPT.bits(), // Field 1: Watchdog capture/timeout value
        ((ms * CLOCK_SYS_CLOCK_1MS) >> WATCHDOG_TIMEOUT_COEFF), // Converted to timer ticks
        FLD_TMR::TMR2_MODE.bits(),   // Field 2: Timer 2 mode
        0                            // Mode 0: watchdog mode
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
#[coverage(off)]
mod tests {
    use super::*;
    use tlsr8266_pac::fake_mmio::{FakeMmio, RegWrite};
    use tlsr8266_pac::REG_BASE_ADDR;

    /// `reg_tmr_ctrl` (32-bit view) and `reg_tmr2_tick`.
    const TMR_CTRL: u32 = 0x620;
    const TMR2_TICK: u32 = 0x638;

    /// `FakeMmio` is process-wide and the suite runs with `--test-threads=1`,
    /// so every test starts from a zeroed bank.
    fn reset_fakes() {
        FakeMmio::reset();
    }

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
    /// 1. Reset the fake bank so `reg_tmr_ctrl` reads as 0
    /// 2. Call wd_set_interval with a test interval value
    /// 3. Verify the timer control register holds the expected values
    /// 4. Verify the timer tick counter is reset to zero
    ///
    /// # Notes
    ///
    /// * Tests conversion from milliseconds to timer ticks
    /// * Verifies Timer 2 is set to mode 0 (watchdog mode)
    #[test]
    fn test_wd_set_interval() {
        reset_fakes();

        // Call the function with a 1000ms (1 second) timeout
        wd_set_interval(1000);

        // Calculate the expected timer value (same calculation as in the function)
        let expected_timer_value = ((1000 * CLOCK_SYS_CLOCK_1MS) >> WATCHDOG_TIMEOUT_COEFF) << 9;

        let written_ctrl_val = FakeMmio::reg32(TMR_CTRL);

        // Timer 2 mode should be 0 (watchdog mode)
        assert_eq!(
            written_ctrl_val & FLD_TMR::TMR2_MODE.bits(),
            0,
            "Timer 2 mode should be set to 0 (watchdog mode)"
        );

        // Watchdog capture value should match our calculation
        assert_eq!(
            written_ctrl_val & FLD_TMR::TMR_WD_CAPT.bits(),
            expected_timer_value & FLD_TMR::TMR_WD_CAPT.bits(),
            "Watchdog capture value should be set correctly"
        );

        // The timer tick counter must be reset to zero
        assert_eq!(
            FakeMmio::reg32(TMR2_TICK),
            0,
            "Timer tick should be reset to 0"
        );
        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: TMR_CTRL | REG_BASE_ADDR,
                    width: 4,
                    value: written_ctrl_val,
                },
                RegWrite {
                    addr: TMR2_TICK | REG_BASE_ADDR,
                    width: 4,
                    value: 0,
                },
            ],
            "exactly one control write followed by one tick reset"
        );
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
    /// 1. Seed the control register with a value whose watchdog bit is clear
    /// 2. Call wd_start to enable the watchdog
    /// 3. Verify only the watchdog enable bit changed
    ///
    /// # Notes
    ///
    /// * Tests the atomic bit-set operation for enabling the watchdog
    /// * Confirms the register read-modify-write sequence is performed correctly
    #[test]
    fn test_wd_start() {
        reset_fakes();

        // Initial register value with some bits already set
        let initial_reg_value = 0x12345678;
        FakeMmio::set_reg32(TMR_CTRL, initial_reg_value);
        FakeMmio::clear_writes();

        // Call the function to start the watchdog
        wd_start();

        // Check that only the watchdog enable bit was set,
        // and all other bits remain unchanged
        assert_eq!(
            FakeMmio::reg32(TMR_CTRL),
            initial_reg_value | FLD_TMR::TMR_WD_EN.bits(),
            "Only the watchdog enable bit should be set"
        );
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: TMR_CTRL | REG_BASE_ADDR,
                width: 4,
                value: initial_reg_value | FLD_TMR::TMR_WD_EN.bits(),
            }],
            "one read-modify-write of the control register"
        );
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
    /// 1. Seed the control register with the watchdog bit already set
    /// 2. Call wd_stop to disable the watchdog
    /// 3. Verify only the watchdog enable bit changed
    ///
    /// # Notes
    ///
    /// * Tests the atomic bit-clear operation for disabling the watchdog
    /// * Confirms the register read-modify-write sequence is performed correctly
    #[test]
    fn test_wd_stop() {
        reset_fakes();

        // Initial register value with watchdog enable bit already set
        let initial_reg_value = 0x12345678 | FLD_TMR::TMR_WD_EN.bits();
        FakeMmio::set_reg32(TMR_CTRL, initial_reg_value);
        FakeMmio::clear_writes();

        // Call the function to stop the watchdog
        wd_stop();

        // Check that only the watchdog enable bit was cleared,
        // and all other bits remain unchanged
        assert_eq!(
            FakeMmio::reg32(TMR_CTRL),
            initial_reg_value & !FLD_TMR::TMR_WD_EN.bits(),
            "Only the watchdog enable bit should be cleared"
        );
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: TMR_CTRL | REG_BASE_ADDR,
                width: 4,
                value: initial_reg_value & !FLD_TMR::TMR_WD_EN.bits(),
            }],
            "one read-modify-write of the control register"
        );
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
    /// 1. Seed the control register with a value whose clear bit is clear
    /// 2. Call wd_clear to reset the watchdog counter
    /// 3. Verify only the clear watchdog bit changed
    ///
    /// # Notes
    ///
    /// * Tests the atomic bit-set operation for clearing the watchdog
    /// * In hardware, the clear bit is self-clearing after the operation
    /// * Confirms the register read-modify-write sequence is performed correctly
    #[test]
    fn test_wd_clear() {
        reset_fakes();

        // Initial register value with some bits already set
        let initial_reg_value = 0x12345678;
        FakeMmio::set_reg32(TMR_CTRL, initial_reg_value);
        FakeMmio::clear_writes();

        // Call the function to clear the watchdog
        wd_clear();

        // Check that only the clear watchdog bit was set,
        // and all other bits remain unchanged
        assert_eq!(
            FakeMmio::reg32(TMR_CTRL),
            initial_reg_value | FLD_TMR::CLR_WD.bits(),
            "Only the clear watchdog bit should be set"
        );
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: TMR_CTRL | REG_BASE_ADDR,
                width: 4,
                value: initial_reg_value | FLD_TMR::CLR_WD.bits(),
            }],
            "one read-modify-write of the control register"
        );
    }
}
