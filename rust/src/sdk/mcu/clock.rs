//! Clock management and timing utilities for TLSR8266 SoC.
//!
//! This module provides functions for initializing and managing the system clock,
//! timing operations, and sleep functionality. It interfaces directly with the
//! hardware timer registers of the Telink TLSR8266F512 BLE SoC.
//!
//! The system operates at 32MHz by default, derived from a 192MHz PLL clock
//! that is divided down to the system clock frequency.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::sdk::mcu::analog::analog_read;
use crate::sdk::mcu::register::{
    read_reg_clk_sel, read_reg_system_tick, write_reg_clk_sel, write_reg_rst_clk0,
    write_reg_tmr_ctrl, FLD_CLK_EN, FLD_CLK_SEL, FLD_TMR, WATCHDOG_TIMEOUT_COEFF,
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

/// Analog register address of the low byte of the free-running 32K_RC counter
const ANA_32K_COUNTER_LO: u8 = 0x40;
/// Analog register address of the high byte of the free-running 32K_RC counter
const ANA_32K_COUNTER_HI: u8 = 0x41;

/// Number of 32K_RC ticks measured by the crystal detection window (~1ms)
const XTAL_DETECT_WINDOW_32K_TICKS: u16 = 32;

/// System tick count at or above which the external crystal detected at boot is
/// 16MHz
///
/// Measured over a 32 tick 32K_RC window with the system clock switched to
/// `SEL_16M_PAD`: a 12MHz board yields 12,483-12,535 ticks and a 16MHz board
/// yields 16,286-16,656 ticks. The midpoint leaves roughly 14% margin on each
/// side of the ~31% separation.
const XTAL_DETECT_16MHZ_THRESHOLD: u32 = 14_500;

/// Maximum number of 32K_RC polls allowed in a single loop of the detection path
///
/// The counter advances once every ~30.5us, so the 32 tick measurement window
/// needs ~32 polls and each poll (three analog reads) takes a few microseconds:
/// 4096 leaves roughly two orders of magnitude of headroom. The bound exists
/// because detection runs before the watchdog is armed, where a stuck loop is a
/// hang that cannot be recovered without SWire.
const XTAL_DETECT_MAX_POLLS: u16 = 4096;

/// Largest 32K_RC sample delta that is counted as forward progress
///
/// The counter free-runs forward, so a delta in the top half of its 16-bit range
/// can only be a backward glitch (a garbled sample): a real forward step inside
/// the 32 tick window is a handful of ticks at most. Without this guard a glitch
/// wraps to a huge delta, closes the window after a couple of samples and
/// reports a 16MHz part as 12MHz.
const XTAL_DETECT_MAX_SAMPLE_STEP: u16 = 0x7fff;

/// Whether the external crystal detected at boot is 16MHz (vs. the default 12MHz)
///
/// Written once during [`clock_init`] and only read afterwards, so a `Relaxed`
/// load/store pair is sufficient.
static IS_16MHZ_XTAL: AtomicBool = AtomicBool::new(false);

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
/// 2. Detecting whether the external crystal is 12MHz or 16MHz
/// 3. Configuring the clock selector register to use PLL divided down to 32MHz
/// 4. Enabling system timer and watchdog with appropriate timeout values
///
/// # Algorithm
///
/// 1. Reset clock registers and configure USB clock
/// 2. Measure the external crystal frequency and cache the result
/// 3. Set clock source to high-speed divider and configure divider ratio
/// 4. Enable timer 0 and configure watchdog timeout
///
/// # Notes
///
/// * The watchdog timer is critical to prevent system hangs and must be periodically
///   refreshed elsewhere in the code
/// * PLL clock (192MHz) is divided by 6 to get the 32MHz system clock
/// * This function must be called early in the boot process
/// * The crystal measurement in step 2 runs with interrupts masked and before
///   any interrupt source is enabled (`irq_init()`/`irq_enable()` happen later
///   in `App::run`), so nothing waits on that mask
pub fn clock_init() {
    // 1. Reset clock registers and enable USB clock if configured
    write_reg_rst_clk0(
        0xff000000  // Reset value with upper bits set
            | FLD_CLK_EN::USB_EN.bits(), // Enable USB clock
    );

    // 2. Detect the external crystal before the PLL divider is configured.
    //    Detection restores reg_clk_sel, so step 3 still starts from the
    //    reset-default clock selection.
    //
    //    The whole measure-and-publish sequence runs masked because it is a
    //    read-measure-restore of the clock selection: an ISR that changed the
    //    source mid-measurement would invalidate the count and have its own
    //    change clobbered by the restore. Nesting is deliberate and safe:
    //    analog_read() takes its own critical section, and the project impl
    //    (irq_i::TlsrCriticalSection) saves and restores the interrupt-enable
    //    register, so the inner acquire sees "already disabled" and restores
    //    that state faithfully. The section polls registers only (no blocking
    //    I/O), is bounded by XTAL_DETECT_MAX_POLLS, and runs before irq_init()/
    //    irq_enable() in `App::run`, so no interrupt waits on the mask.
    critical_section::with(|_| {
        IS_16MHZ_XTAL.store(detect_xtal_is_16mhz(), Ordering::Relaxed);
    });

    // 3. Configure clock source and divider
    //    - Use high-speed divider (SEL_HS_DIV) as clock source
    //    - Divide PLL clock (192MHz) by 6 to get 32MHz system clock
    write_reg_clk_sel(MASK_VAL!(
        FLD_CLK_SEL::DIV.bits() as u32, // Set the clock divider field
        (CLOCK_PLL_CLOCK / CLOCK_SYS_CLOCK_1S) as u32, // Divider value (192MHz/32MHz = 6)
        FLD_CLK_SEL::SRC.bits() as u32, // Set the clock source field
        CLOCK_SEL::SEL_HS_DIV as u32    // Use HS divider as clock source
    ) as u8);

    // 4. Configure system timer and watchdog
    //    - Enable Timer0
    //    - Set watchdog capture value (timeout period converted to appropriate units)
    //    - Enable watchdog timer
    //reg_clk_en = 0xff | CLK_EN_TYPE;
    write_reg_tmr_ctrl(MASK_VAL!(
        FLD_TMR::TMR0_EN.bits(), // Enable Timer0
        1,
        FLD_TMR::TMR_WD_CAPT.bits(), // Configure watchdog capture value
        WATCHDOG_INIT_TIMEOUT * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF, // Convert ms to watchdog units
        FLD_TMR::TMR_WD_EN.bits(), // Enable watchdog timer
        1
    ))
}

/// Returns whether the external crystal detected at boot is 16MHz.
///
/// # Returns
///
/// * `true` if the boot-time measurement identified a 16MHz crystal
/// * `false` for the default 12MHz crystal, or before [`clock_init`] has run
///
/// # Notes
///
/// * Written once by [`clock_init`] and only read afterwards
/// * Uses a `Relaxed` load because no ordering with other memory is required
#[cfg_attr(test, mry::mry)]
pub fn is_16mhz_xtal() -> bool {
    IS_16MHZ_XTAL.load(Ordering::Relaxed)
}

/// Reads the free-running 16-bit 32K_RC counter through the 8-bit analog interface.
///
/// # Returns
///
/// * The current 32K_RC counter value, combined big-endian from `0x41:0x40`
///
/// # Algorithm
///
/// 1. Read the high byte (`0x41`)
/// 2. Read the low byte (`0x40`)
/// 3. Read the high byte again (`0x41`)
/// 4. Retry from step 1 while the two high-byte samples differ
///
/// # Notes
///
/// * The counter is 16 bits wide but each analog read returns a single byte, so
///   the low-byte read can straddle a carry into the high byte
/// * Retrying while the high byte is stable discards those torn samples instead
///   of returning a value that is 256 too high
/// * A high/low pair is only accepted once the same high byte has been observed
///   on both sides of the low-byte read
/// * Retrying stops after [`XTAL_DETECT_MAX_POLLS`] attempts: the last low byte
///   is paired with the newest high byte and returned best effort, because this
///   runs before the watchdog is armed and must not be able to hang
fn read_ana_32k() -> u16 {
    let mut polls: u16 = 0;
    loop {
        let hi1 = analog_read(ANA_32K_COUNTER_HI);
        let lo = analog_read(ANA_32K_COUNTER_LO);
        let hi2 = analog_read(ANA_32K_COUNTER_HI);

        if hi1 == hi2 {
            return u16::from_be_bytes([hi1, lo]);
        }

        if polls >= XTAL_DETECT_MAX_POLLS {
            // Never settled: hand back the most recent pairing rather than hang
            return u16::from_be_bytes([hi2, lo]);
        }

        polls += 1;
    }
}

/// Measures the external crystal frequency and reports whether it is 16MHz.
///
/// # Returns
///
/// * `true` if the external crystal is 16MHz
/// * `false` if the external crystal is 12MHz
///
/// # Algorithm
///
/// 1. Save the current clock selector value
/// 2. Switch the clock source to the crystal pad (`SEL_16M_PAD`) with divider 0
/// 3. Align to a 32K_RC tick edge
/// 4. Count system ticks over a fixed window of 32K_RC ticks (~1ms)
/// 5. Restore the saved clock selector value
/// 6. Report whether the count reached [`XTAL_DETECT_16MHZ_THRESHOLD`]
///
/// # Notes
///
/// * `reg_clk_sel` is written as a whole byte, so the measurement write also
///   sets `DIV = 0`. That is intentional: the pad clock is measured undivided
///   (÷1), which is how the threshold was derived. The value read before the
///   measurement is written back verbatim afterwards, divider included
/// * The window starts on a 32K_RC tick edge because the threshold was derived
///   from measurements taken from a tick boundary; starting mid-tick would shift
///   the window by up to one tick and bias the count
/// * `wrapping_sub` deltas are accumulated so a skipped 32K_RC sample advances
///   the window by more than one tick instead of stalling the loop; a delta that
///   can only be a backward glitch is treated as a re-read instead of closing
///   the window early
/// * Every loop is bounded by [`XTAL_DETECT_MAX_POLLS`]. A bound being reached
///   abandons the measurement, still restores `reg_clk_sel` and reports 12MHz: a
///   16MHz board then loads the 12MHz RF table and loses the radio but still
///   boots and can be reflashed over SWire, whereas a hang here cannot be
///   recovered without SWire
/// * Runs from normal `.text` (not `.ram_code`) so it costs no SRAM
#[cfg_attr(test, mry::mry)]
pub fn detect_xtal_is_16mhz() -> bool {
    // 1. Remember the clock selection so it can be restored afterwards
    let previous = read_reg_clk_sel();

    // 2. Run the system clock straight from the external crystal pad. The
    //    register is written as one byte, so DIV is deliberately 0 (÷1): the pad
    //    clock must be measured undivided for the threshold to hold, and step 5
    //    restores `previous` (divider included) verbatim.
    write_reg_clk_sel(MASK_VAL!(
        FLD_CLK_SEL::SRC.bits() as u32, // DIV = 0, SRC = SEL_16M_PAD
        CLOCK_SEL::SEL_16M_PAD as u32   // Use the 16MHz pad clock
    ) as u8);

    // 3. Align to a 32K_RC tick edge before starting the measurement window
    let edge = read_ana_32k();
    let mut last = read_ana_32k();
    let mut polls: u16 = 0;
    while last == edge && polls < XTAL_DETECT_MAX_POLLS {
        last = read_ana_32k();
        polls += 1;
    }

    // 4. Count system ticks over the fixed 32K_RC window
    let t0 = read_reg_system_tick();
    let mut elapsed_32k: u16 = 0;
    let mut polls: u16 = 0;
    while elapsed_32k < XTAL_DETECT_WINDOW_32K_TICKS && polls < XTAL_DETECT_MAX_POLLS {
        let sample = read_ana_32k();
        let delta = sample.wrapping_sub(last);

        if delta < XTAL_DETECT_MAX_SAMPLE_STEP {
            // Plausible forward step: count it and move the reference forward
            elapsed_32k = elapsed_32k.wrapping_add(delta);
            last = sample;
        }
        // Otherwise the counter appears to have gone backwards or the sample is
        // garbled; keep `last` so the next sample is compared with the last good
        // one, and let the poll bound cover a counter that never recovers
        polls += 1;
    }
    let window_complete = elapsed_32k >= XTAL_DETECT_WINDOW_32K_TICKS;
    let elapsed = read_reg_system_tick().wrapping_sub(t0);

    // 5. Restore the clock selection the caller had before the measurement
    write_reg_clk_sel(previous);

    // 6. ~12,500 ticks means 12MHz, ~16,400 ticks means 16MHz. A window that
    //    never completed is not a measurement, so it reports the 12MHz default
    //    (see the fail-safe note above)
    window_complete && elapsed >= XTAL_DETECT_16MHZ_THRESHOLD
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

#[cfg(test)]
static CLOCK_TIME_UPPER: AtomicU32 = AtomicU32::new(0);

#[cfg(test)]
static LAST_CLOCK_TIME: AtomicU32 = AtomicU32::new(0);

/// Resets the 64-bit clock accumulator and the overflow detector to zero.
///
/// Test-only: proof tests seed the hardware tick register through `FakeMmio` and need
/// `clock_time64()` to restart from an exact, known base instead of inheriting state
/// from an earlier test in the same `--test-threads=1` process.
#[cfg(test)]
pub fn reset_clock_time64_for_test() {
    CLOCK_TIME_UPPER.store(0, Ordering::Relaxed);
    LAST_CLOCK_TIME.store(0, Ordering::Relaxed);
}

/// Provides a 64-bit monotonic clock based on the 32-bit clock_time()
///
/// This function extends the 32-bit hardware timer to a 64-bit logical timer by
/// tracking overflows of the underlying 32-bit counter. It uses atomic operations
/// to safely handle concurrent access without locks in the common case.
///
/// # Algorithm
/// 1. Inside a critical section (interrupts disabled), read the current hardware time
///    and the previously seen time atomically together
/// 2. If current time < last time, the 32-bit hardware counter has overflowed - increment
///    the upper 32 bits to extend the counter
/// 3. Update the last seen time
/// 4. Return a combined 64-bit timestamp (upper 32 bits | lower 32 bits)
///
/// # Why clock_time() must be called inside the critical section
///
/// Previously, clock_time() was called BEFORE entering the critical section. This created
/// a race condition: if a hardware interrupt fired between reading clock_time() and reading
/// LAST_CLOCK_TIME, and that interrupt handler itself called clock_time64() (e.g. to step
/// a light transition), it would update LAST_CLOCK_TIME to a newer value. When the
/// interrupted code resumed, LAST_CLOCK_TIME was now genuinely larger than the stale
/// current_time, so the code falsely detected a 32-bit overflow and incremented the upper
/// bits. At 16-24 MHz this adds ~180-268 seconds to Instant::now() instantaneously, which
/// blew past any in-progress transition timestamp and caused the light to snap to its final
/// state. By reading clock_time() inside the critical section, interrupts are physically
/// incapable of wedging themselves between the hardware read and the comparison.
#[cfg_attr(test, mry::mry)]
pub fn clock_time64() -> u64 {
    // When not testing, these static variables are defined here
    // When testing, they're defined at module level for test access
    #[cfg(not(test))]
    static CLOCK_TIME_UPPER: AtomicU32 = AtomicU32::new(0);
    #[cfg(not(test))]
    static LAST_CLOCK_TIME: AtomicU32 = AtomicU32::new(0);

    critical_section::with(|_| {
        // Read the hardware time inside the critical section so that no interrupt can update
        // LAST_CLOCK_TIME between this read and the comparison below.
        let current_time = clock_time();
        let last_time = LAST_CLOCK_TIME.load(Ordering::Relaxed);

        if current_time < last_time {
            // Overflow confirmed - increment upper bits.
            // Use load-modify-store instead of fetch_add since fetch_add might not be supported
            // on all platforms or with all atomics implementations.
            let upper = CLOCK_TIME_UPPER.load(Ordering::Relaxed);
            CLOCK_TIME_UPPER.store(upper + 1, Ordering::Relaxed);
        }

        // Always update the last seen time so subsequent calls can detect the next overflow.
        LAST_CLOCK_TIME.store(current_time, Ordering::Relaxed);

        // Combine upper and lower bits to form the 64-bit timestamp.
        // Upper 32 bits track the number of overflows.
        // Lower 32 bits are the current hardware timer value.
        (CLOCK_TIME_UPPER.load(Ordering::Relaxed) as u64) << 32 | current_time as u64
    })
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
#[coverage(off)]
mod tests {
    use super::*;

    // Import mock functions from register module for testing
    use crate::sdk::mcu::analog::mock_analog_read;
    use crate::sdk::mcu::register::mock_read_reg_clk_sel;
    use crate::sdk::mcu::register::mock_read_reg_system_tick;
    use crate::sdk::mcu::register::mock_write_reg_clk_sel;
    use crate::sdk::mcu::register::mock_write_reg_rst_clk0;
    use crate::sdk::mcu::register::mock_write_reg_tmr_ctrl;
    use mry::Any;
    use std::sync::atomic::AtomicU32;
    use std::sync::{Arc, Mutex};

    /// Tests clock_init function for proper register configuration.
    ///
    /// This test verifies that the clock_init function correctly:
    /// - Configures the reset register with proper USB settings
    /// - Detects and caches the external crystal frequency exactly once
    /// - Sets up the clock selector register with correct source and divider
    /// - Configures the timer control register with proper watchdog timing
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for register write functions and crystal detection
    /// 2. Call clock_init to trigger the configuration
    /// 3. Verify that each register was written with expected values
    ///
    /// # Notes
    ///
    /// * Tests all four stages of clock initialization
    /// * Verifies USB clock configuration based on CLK_USB_ENABLE constant
    /// * Confirms the detected crystal is published through is_16mhz_xtal()
    /// * Confirms PLL divider calculation is correct
    /// * Verifies watchdog timeout is properly converted to register value
    #[test]
    #[mry::lock(
        write_reg_rst_clk0,
        write_reg_clk_sel,
        write_reg_tmr_ctrl,
        detect_xtal_is_16mhz
    )]
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
            FLD_TMR::TMR0_EN.bits(),
            1,
            FLD_TMR::TMR_WD_CAPT.bits(),
            WATCHDOG_INIT_TIMEOUT * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF,
            FLD_TMR::TMR_WD_EN.bits(),
            1
        );

        // Setup mock behaviors
        mock_write_reg_rst_clk0(expected_rst_clk0).returns(());
        mock_write_reg_clk_sel(expected_clk_sel).returns(());
        mock_write_reg_tmr_ctrl(expected_tmr_ctrl).returns(());
        mock_detect_xtal_is_16mhz().returns(true);

        // Call the function being tested
        clock_init();

        // Verify each register was written exactly once with correct values
        mock_write_reg_rst_clk0(expected_rst_clk0).assert_called(1);
        mock_write_reg_clk_sel(expected_clk_sel).assert_called(1);
        mock_write_reg_tmr_ctrl(expected_tmr_ctrl).assert_called(1);

        // Verify the crystal was detected exactly once and its result published
        mock_detect_xtal_is_16mhz().assert_called(1);
        assert_eq!(
            is_16mhz_xtal(),
            true,
            "clock_init should publish the detected crystal"
        );

        // Leave the shared state at its power-on default for other tests
        IS_16MHZ_XTAL.store(false, Ordering::Relaxed);
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
        assert_eq!(
            result, false,
            "Should return false when time has not elapsed"
        );

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
        assert_eq!(
            result, false,
            "1 microsecond should require 32 ticks at 32MHz"
        );

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
        assert_eq!(
            result, false,
            "32 ticks should match exactly 1us (not exceed)"
        );

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

    /// Tests that read_ana_32k retries when the high byte changes mid-read.
    ///
    /// This test verifies that the read_ana_32k function correctly:
    /// - Rejects a sample whose high byte differs before and after the low byte
    /// - Retries until the same high byte is observed on both sides
    /// - Combines the accepted high/low bytes big-endian
    ///
    /// # Algorithm
    ///
    /// 1. Return `0x12` for the first high-byte read and `0x13` for both reads of
    ///    the second attempt
    /// 2. Return a constant low byte
    /// 3. Call read_ana_32k and verify the second attempt's value is returned
    ///
    /// # Notes
    ///
    /// * The two low-byte reads prove the retry loop actually ran
    /// * The final value `0x1334` is only reachable if the stable high byte is
    ///   combined with the low byte
    #[test]
    #[mry::lock(analog_read)]
    fn test_read_ana_32k_retries_on_torn_high_byte() {
        // Attempt 1 reads high (0x12), low, high (0x13) and is rejected; attempt
        // 2 reads high (0x13), low, high (0x13) and is accepted.
        let mut hi_samples = [0x12u8, 0x13, 0x13, 0x13].into_iter();
        mock_analog_read(ANA_32K_COUNTER_HI).returns_with(move |_: u8| -> u8 {
            hi_samples.next().expect("too many high-byte reads")
        });
        mock_analog_read(ANA_32K_COUNTER_LO).returns(0x34);

        let value = read_ana_32k();

        assert_eq!(
            value, 0x1334,
            "should accept only the high byte that stayed stable"
        );

        // Two high-byte reads per attempt, one attempt rejected and one accepted
        mock_analog_read(ANA_32K_COUNTER_HI).assert_called(4);
        // One low-byte read per attempt proves the retry loop was exercised
        mock_analog_read(ANA_32K_COUNTER_LO).assert_called(2);
    }

    /// Tests that read_ana_32k stops retrying when the high byte never settles.
    ///
    /// This test verifies that the read_ana_32k function correctly:
    /// - Stops after [`XTAL_DETECT_MAX_POLLS`] attempts instead of hanging
    /// - Still returns a value, pairing the last low byte with the newest high
    ///   byte, so the pre-watchdog boot path cannot stall
    ///
    /// # Algorithm
    ///
    /// 1. Alternate the high byte on every read so `hi1 != hi2` always
    /// 2. Call read_ana_32k and verify a best-effort value comes back
    ///
    /// # Notes
    ///
    /// * 4097 low-byte reads pin the cap; without it this test would never return
    #[test]
    #[mry::lock(analog_read)]
    fn test_read_ana_32k_bounded_when_high_byte_never_settles() {
        let mut hi_reads: u32 = 0;
        mock_analog_read(ANA_32K_COUNTER_HI).returns_with(move |_: u8| -> u8 {
            hi_reads += 1;
            if hi_reads % 2 == 1 {
                0x12
            } else {
                0x13
            }
        });
        mock_analog_read(ANA_32K_COUNTER_LO).returns(0x34);

        let value = read_ana_32k();

        assert_eq!(
            value, 0x1334,
            "should return the last sample instead of retrying forever"
        );
        // One attempt past the cap, and every attempt reads the low byte once
        mock_analog_read(ANA_32K_COUNTER_LO).assert_called(XTAL_DETECT_MAX_POLLS as usize + 1);
    }

    /// Tests that is_16mhz_xtal reflects the value stored at boot.
    ///
    /// This test verifies that the is_16mhz_xtal function correctly:
    /// - Returns true after a 16MHz result has been stored
    /// - Returns false after a 12MHz result has been stored
    ///
    /// # Notes
    ///
    /// * Sets the shared state explicitly rather than relying on test order
    /// * Restores the power-on default so later tests start from a known state
    #[test]
    fn test_is_16mhz_xtal_reflects_stored_value() {
        IS_16MHZ_XTAL.store(true, Ordering::Relaxed);
        assert_eq!(is_16mhz_xtal(), true, "Should report a stored 16MHz result");

        IS_16MHZ_XTAL.store(false, Ordering::Relaxed);
        assert_eq!(
            is_16mhz_xtal(),
            false,
            "Should report a stored 12MHz result"
        );
    }

    /// Tests that detect_xtal_is_16mhz reports false for a 12MHz measurement.
    ///
    /// This test verifies that the detect_xtal_is_16mhz function correctly:
    /// - Aligns to a 32K_RC tick edge before measuring
    /// - Counts system ticks over the 32K_RC window
    /// - Reports false when the count is below the 16MHz threshold
    ///
    /// # Algorithm
    ///
    /// 1. Mock the 32K_RC counter to advance 1 tick to the edge, then 32 ticks
    /// 2. Mock the system tick to advance 12,500 ticks over the window
    /// 3. Call the function and verify it reports false
    ///
    /// # Notes
    ///
    /// * 12,500 is the measured centre of the 12MHz board range
    #[test]
    #[mry::lock(analog_read, read_reg_clk_sel, write_reg_clk_sel, read_reg_system_tick)]
    fn test_detect_xtal_is_16mhz_below_threshold() {
        let start_tick = 0x1000_0000u32;
        let elapsed = 12_500u32;

        mock_read_reg_clk_sel().returns(0x63);

        // 32K_RC samples: the edge (100), the first sample past the edge (101),
        // then the sample that closes the 32 tick window (133)
        let mut lo_samples = [100u8, 101, 133].into_iter();
        mock_analog_read(ANA_32K_COUNTER_LO).returns_with(move |_: u8| -> u8 {
            lo_samples.next().expect("unexpected 32K_RC low-byte read")
        });
        mock_analog_read(ANA_32K_COUNTER_HI).returns(0);

        let mut tick_calls = 0;
        mock_read_reg_system_tick().returns_with(move || -> u32 {
            tick_calls += 1;
            if tick_calls == 1 {
                start_tick
            } else {
                start_tick + elapsed
            }
        });
        mock_write_reg_clk_sel(Any).returns(());

        let result = detect_xtal_is_16mhz();

        assert_eq!(result, false, "12,500 ticks should be reported as 12MHz");
        mock_read_reg_clk_sel().assert_called(1);
        mock_read_reg_system_tick().assert_called(2);
        // One write to switch sources, one to restore
        mock_write_reg_clk_sel(Any).assert_called(2);
    }

    /// Tests that detect_xtal_is_16mhz reports true for a 16MHz measurement.
    ///
    /// This test verifies that the detect_xtal_is_16mhz function correctly:
    /// - Reports true when the count is above the 16MHz threshold
    ///
    /// # Notes
    ///
    /// * 16,400 is the measured centre of the 16MHz board range
    #[test]
    #[mry::lock(analog_read, read_reg_clk_sel, write_reg_clk_sel, read_reg_system_tick)]
    fn test_detect_xtal_is_16mhz_above_threshold() {
        let start_tick = 0x2000_0000u32;
        let elapsed = 16_400u32;

        mock_read_reg_clk_sel().returns(0x63);

        let mut lo_samples = [100u8, 101, 133].into_iter();
        mock_analog_read(ANA_32K_COUNTER_LO).returns_with(move |_: u8| -> u8 {
            lo_samples.next().expect("unexpected 32K_RC low-byte read")
        });
        mock_analog_read(ANA_32K_COUNTER_HI).returns(0);

        let mut tick_calls = 0;
        mock_read_reg_system_tick().returns_with(move || -> u32 {
            tick_calls += 1;
            if tick_calls == 1 {
                start_tick
            } else {
                start_tick + elapsed
            }
        });
        mock_write_reg_clk_sel(Any).returns(());

        let result = detect_xtal_is_16mhz();

        assert_eq!(result, true, "16,400 ticks should be reported as 16MHz");
        mock_read_reg_system_tick().assert_called(2);
    }

    /// Tests the exact threshold boundary of detect_xtal_is_16mhz.
    ///
    /// This test verifies that the detect_xtal_is_16mhz function correctly:
    /// - Treats a count exactly equal to the threshold as 16MHz
    ///
    /// # Notes
    ///
    /// * The threshold is inclusive, so a count of exactly 14,500 is 16MHz
    #[test]
    #[mry::lock(analog_read, read_reg_clk_sel, write_reg_clk_sel, read_reg_system_tick)]
    fn test_detect_xtal_is_16mhz_at_threshold() {
        assert_eq!(
            XTAL_DETECT_16MHZ_THRESHOLD, 14_500,
            "the documented threshold must not drift"
        );

        let start_tick = 0x3000_0000u32;
        let elapsed = XTAL_DETECT_16MHZ_THRESHOLD;

        mock_read_reg_clk_sel().returns(0x63);

        let mut lo_samples = [100u8, 101, 133].into_iter();
        mock_analog_read(ANA_32K_COUNTER_LO).returns_with(move |_: u8| -> u8 {
            lo_samples.next().expect("unexpected 32K_RC low-byte read")
        });
        mock_analog_read(ANA_32K_COUNTER_HI).returns(0);

        let mut tick_calls = 0;
        mock_read_reg_system_tick().returns_with(move || -> u32 {
            tick_calls += 1;
            if tick_calls == 1 {
                start_tick
            } else {
                start_tick + elapsed
            }
        });
        mock_write_reg_clk_sel(Any).returns(());

        let result = detect_xtal_is_16mhz();

        assert_eq!(
            result, true,
            "A count exactly at the threshold should be reported as 16MHz"
        );
    }

    /// Tests that detect_xtal_is_16mhz restores the previous clock selection.
    ///
    /// This test verifies that the detect_xtal_is_16mhz function correctly:
    /// - Reads the existing reg_clk_sel value before changing it
    /// - Switches only the source field to the 16MHz pad for the measurement
    /// - Writes the previously read value back afterwards
    ///
    /// # Algorithm
    ///
    /// 1. Return `0x63` (divider 3, 32MHz pad) from read_reg_clk_sel
    /// 2. Record every write_reg_clk_sel value in call order
    /// 3. Verify the writes are [SEL_16M_PAD source byte, previous value]
    ///
    /// # Notes
    ///
    /// * The recorded order proves the restore is the second and final write
    /// * A 16,400 tick count is used so the measurement completes normally
    #[test]
    #[mry::lock(analog_read, read_reg_clk_sel, write_reg_clk_sel, read_reg_system_tick)]
    fn test_detect_xtal_is_16mhz_restores_previous_clk_sel() {
        let previous = 0x63u8;
        let measurement_src = MASK_VAL!(
            FLD_CLK_SEL::SRC.bits() as u32,
            CLOCK_SEL::SEL_16M_PAD as u32
        ) as u8;

        mock_read_reg_clk_sel().returns(previous);

        let writes: Arc<Mutex<Vec<u8>>> = Arc::new(Mutex::new(Vec::new()));
        let recorded_writes = Arc::clone(&writes);
        mock_write_reg_clk_sel(Any).returns_with(move |value: u8| {
            recorded_writes.lock().unwrap().push(value);
        });

        let mut lo_samples = [100u8, 101, 133].into_iter();
        mock_analog_read(ANA_32K_COUNTER_LO).returns_with(move |_: u8| -> u8 {
            lo_samples.next().expect("unexpected 32K_RC low-byte read")
        });
        mock_analog_read(ANA_32K_COUNTER_HI).returns(0);

        let start_tick = 0x4000_0000u32;
        let mut tick_calls = 0;
        mock_read_reg_system_tick().returns_with(move || -> u32 {
            tick_calls += 1;
            if tick_calls == 1 {
                start_tick
            } else {
                start_tick + 16_400
            }
        });

        let result = detect_xtal_is_16mhz();

        assert_eq!(result, true, "16,400 ticks should be reported as 16MHz");
        assert_eq!(
            *writes.lock().unwrap(),
            vec![measurement_src, previous],
            "detection must restore the previously read reg_clk_sel value"
        );
    }

    /// Tests that detect_xtal_is_16mhz fails safe when the 32K_RC counter stalls.
    ///
    /// This test verifies that the detect_xtal_is_16mhz function correctly:
    /// - Bounds both the edge-alignment and the measurement loops
    /// - Restores the previous clock selection on that timeout path
    /// - Reports 12MHz even when the system ticks read high, because an
    ///   incomplete 32K window is not a measurement
    ///
    /// # Algorithm
    ///
    /// 1. Hold the 32K_RC counter at a constant value so neither loop progresses
    /// 2. Have the system tick counter report 16,400 ticks between t0 and the end
    /// 3. Call the function and verify it returns false and restores the register
    ///
    /// # Notes
    ///
    /// * Without the fail-safe the 16,400 tick reading would be reported as
    ///   16MHz; without the bounds the test would never return
    #[test]
    #[mry::lock(analog_read, read_reg_clk_sel, write_reg_clk_sel, read_reg_system_tick)]
    fn test_detect_xtal_is_16mhz_reports_12mhz_when_32k_stalls() {
        mock_read_reg_clk_sel().returns(0x63);
        mock_analog_read(ANA_32K_COUNTER_HI).returns(0);
        mock_analog_read(ANA_32K_COUNTER_LO).returns(100);

        let start_tick = 0x5000_0000u32;
        let mut tick_calls = 0;
        mock_read_reg_system_tick().returns_with(move || -> u32 {
            tick_calls += 1;
            if tick_calls == 1 {
                start_tick
            } else {
                start_tick + 16_400
            }
        });
        mock_write_reg_clk_sel(Any).returns(());

        let result = detect_xtal_is_16mhz();

        assert_eq!(
            result, false,
            "an incomplete 32K_RC window must report the 12MHz default"
        );
        // One write to switch sources, one to restore
        mock_write_reg_clk_sel(Any).assert_called(2);
        // Two polls for the edge plus the cap on each loop
        mock_analog_read(ANA_32K_COUNTER_LO).assert_called(2 * XTAL_DETECT_MAX_POLLS as usize + 2);
    }

    /// Tests that detect_xtal_is_16mhz ignores a backward 32K_RC glitch.
    ///
    /// This test verifies that the detect_xtal_is_16mhz function correctly:
    /// - Discards a sample that appears to move the counter backwards
    /// - Keeps counting from the last good sample, so the window still spans 32
    ///   ticks and the measured tick count stays correct
    ///
    /// # Algorithm
    ///
    /// 1. Emit the edge (100), a sample past it (101), a backward glitch (99),
    ///    then one incrementing sample per poll
    /// 2. Model the system tick counter as proportional to the number of 32K_RC
    ///    samples the measurement actually consumed (~512 system ticks per 32K
    ///    tick, i.e. a 16MHz part)
    /// 3. Verify the result is true
    ///
    /// # Notes
    ///
    /// * Without the guard the glitch wraps to a delta of 65,534, closes the
    ///   window after three samples and reads ~512 ticks, i.e. 12MHz
    #[test]
    #[mry::lock(analog_read, read_reg_clk_sel, write_reg_clk_sel, read_reg_system_tick)]
    fn test_detect_xtal_is_16mhz_ignores_backward_glitch() {
        // ~16MHz of system clock per 32K_RC tick
        const SYS_TICKS_PER_32K_TICK: u32 = 512;

        let start_tick = 0x6000_0000u32;
        let samples_read = Arc::new(AtomicU32::new(0));

        mock_read_reg_clk_sel().returns(0x63);
        mock_analog_read(ANA_32K_COUNTER_HI).returns(0);

        let mut call: u32 = 0;
        let counted = Arc::clone(&samples_read);
        mock_analog_read(ANA_32K_COUNTER_LO).returns_with(move |_: u8| -> u8 {
            call += 1;
            counted.fetch_add(1, Ordering::Relaxed);
            match call {
                1 => 100, // edge
                2 => 101, // first sample past the edge
                3 => 99,  // backward glitch: 99 - 101 wraps to 65,534
                n => (100 + n) as u8,
            }
        });

        let counted = Arc::clone(&samples_read);
        mock_read_reg_system_tick().returns_with(move || -> u32 {
            start_tick + counted.load(Ordering::Relaxed) * SYS_TICKS_PER_32K_TICK
        });
        mock_write_reg_clk_sel(Any).returns(());

        let result = detect_xtal_is_16mhz();

        assert_eq!(
            result, true,
            "a backward glitch must not truncate the 32K window"
        );
        // The window completed, so the samples span the full 32 ticks; a
        // truncated window would stop after the glitch (three samples)
        assert!(
            samples_read.load(Ordering::Relaxed) >= XTAL_DETECT_WINDOW_32K_TICKS as u32,
            "the ignored glitch should cost one poll, not the window"
        );
    }

    // Helper function to reset static variables for testing without using unsafe
    fn reset_static_vars() {
        CLOCK_TIME_UPPER.store(0, Ordering::Relaxed);
        LAST_CLOCK_TIME.store(0, Ordering::Relaxed);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_no_overflow() {
        // Arrange
        reset_static_vars();

        // Set initial value
        let initial_value = 1000;
        LAST_CLOCK_TIME.store(initial_value, Ordering::Relaxed);

        // Simulate non-overflow case
        let next_value = initial_value + 100;

        // Mock clock_time with the correct format
        mock_clock_time().returns(next_value);

        // Act
        let result = clock_time64();

        // Assert
        assert_eq!(result, next_value as u64);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 0);
        assert_eq!(LAST_CLOCK_TIME.load(Ordering::Relaxed), next_value);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_with_overflow() {
        // Arrange
        reset_static_vars();

        // Set up overflow scenario
        let initial_value = 0xFFFFFF00_u32;
        LAST_CLOCK_TIME.store(initial_value, Ordering::Relaxed);

        // Simulate overflow: next value will be less than previous
        let next_value = 100_u32;

        // Mock clock_time with the correct format
        mock_clock_time().returns(next_value);

        // Act
        let result = clock_time64();

        // Assert
        // The upper 32 bits should be incremented to 1, and lower 32 bits should be our next value
        assert_eq!(result, (1_u64 << 32) | next_value as u64);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 1);
        assert_eq!(LAST_CLOCK_TIME.load(Ordering::Relaxed), next_value);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_normal_increment() {
        // Arrange
        reset_static_vars();
        LAST_CLOCK_TIME.store(1000, Ordering::Relaxed);
        mock_clock_time().returns(1500);

        // Act
        let result = clock_time64();

        // Assert
        assert_eq!(result, 1500);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 0);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_first_overflow() {
        // Arrange
        reset_static_vars();
        LAST_CLOCK_TIME.store(0xFFFFFF00, Ordering::Relaxed);
        mock_clock_time().returns(100);

        // Act
        let result = clock_time64();

        // Assert
        assert_eq!(result, (1_u64 << 32) | 100);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 1);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_after_overflow() {
        // Arrange
        reset_static_vars();
        LAST_CLOCK_TIME.store(100, Ordering::Relaxed);
        CLOCK_TIME_UPPER.store(1, Ordering::Relaxed);
        mock_clock_time().returns(200);

        // Act
        let result = clock_time64();

        // Assert
        assert_eq!(result, (1_u64 << 32) | 200);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 1);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_second_overflow() {
        // Arrange
        reset_static_vars();
        LAST_CLOCK_TIME.store(0xFFFFFF00, Ordering::Relaxed);
        CLOCK_TIME_UPPER.store(1, Ordering::Relaxed);
        mock_clock_time().returns(300);

        // Act
        let result = clock_time64();

        // Assert
        assert_eq!(result, (2_u64 << 32) | 300);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 2);
        mock_clock_time().assert_called(1);
    }

    #[test]
    fn test_reset_clock_time64_for_test_clears_state() {
        // Arrange: leave non-zero accumulator state behind, as a prior proof test would
        CLOCK_TIME_UPPER.store(7, Ordering::Relaxed);
        LAST_CLOCK_TIME.store(0x1234_5678, Ordering::Relaxed);

        // Act
        reset_clock_time64_for_test();

        // Assert
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 0);
        assert_eq!(LAST_CLOCK_TIME.load(Ordering::Relaxed), 0);
    }
}
