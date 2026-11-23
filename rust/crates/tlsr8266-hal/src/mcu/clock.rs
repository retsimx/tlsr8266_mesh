//! Clock management and timing utilities for TLSR8266 SoC.
//!
//! This module provides functions for initializing and managing the system clock,
//! timing operations, and sleep functionality. It interfaces directly with the
//! hardware timer registers of the Telink TLSR8266F512 BLE SoC.
//!
//! The system operates at 32MHz by default, derived from a 192MHz PLL clock
//! that is divided down to the system clock frequency.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::mcu::analog::analog_read;
use crate::MASK_VAL;
use tlsr8266_pac::{
    read_reg_clk_sel, read_reg_system_tick, write_reg_clk_sel, write_reg_rst_clk0,
    write_reg_tmr_ctrl, FLD_CLK_EN, FLD_CLK_SEL, FLD_TMR, WATCHDOG_TIMEOUT_COEFF,
};

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
pub const CLOCK_SYS_CLOCK_1MS: u32 = CLOCK_SYS_CLOCK_1S / 1000;
/// Number of system clock ticks in 1 microsecond
pub const CLOCK_SYS_CLOCK_1US: u32 = CLOCK_SYS_CLOCK_1S / 1000000;
/// Number of system clock ticks in 4 seconds
pub const CLOCK_SYS_CLOCK_4S: u32 = CLOCK_SYS_CLOCK_1S << 2;
/// Maximum number of milliseconds that can be represented by the clock counter
pub const CLOCK_MAX_MS: u32 = u32::MAX / CLOCK_SYS_CLOCK_1MS;
/// Maximum number of microseconds that can be represented by the clock counter
pub const CLOCK_MAX_US: u32 = u32::MAX / CLOCK_SYS_CLOCK_1US;

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
///
/// Variant names are the datasheet `reg_clk_sel.SRC` field values, so the
/// datasheet spelling is kept; only the two sources this firmware selects are
/// defined here.
#[allow(non_camel_case_types)]
enum CLOCK_SEL {
    /// High-speed clock with divider (used for PLL-derived clock)
    SEL_HS_DIV = 1,
    /// 16MHz external oscillator from pad
    SEL_16M_PAD = 2,
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
        FLD_CLK_SEL::DIV.bits() as u32,       // Set the clock divider field
        CLOCK_PLL_CLOCK / CLOCK_SYS_CLOCK_1S, // Divider value (192MHz/32MHz = 6)
        FLD_CLK_SEL::SRC.bits() as u32,       // Set the clock source field
        CLOCK_SEL::SEL_HS_DIV as u32          // Use HS divider as clock source
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
        (WATCHDOG_INIT_TIMEOUT * CLOCK_SYS_CLOCK_1MS) >> WATCHDOG_TIMEOUT_COEFF, // Convert ms to watchdog units
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
pub fn clock_time() -> u32 {
    // Read the system tick register directly
    read_reg_system_tick()
}

static CLOCK_TIME_UPPER: AtomicU32 = AtomicU32::new(0);
static LAST_CLOCK_TIME: AtomicU32 = AtomicU32::new(0);

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
pub fn clock_time64() -> u64 {
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
pub fn clock_time_exceed(reference: u32, span_us: u32) -> bool {
    // Calculate elapsed time (handles overflow correctly due to u32 wraparound properties)
    // Convert span_us to system clock ticks and compare
    clock_time().wrapping_sub(reference) > span_us.wrapping_mul(CLOCK_SYS_CLOCK_1US)
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
/// * On the device this function is placed in RAM (.ram_code section) for
///   reliable execution regardless of flash access timing
/// * Uses busy-waiting which keeps the CPU active (consuming power)
/// * For longer delays or power-efficient waiting, consider using a sleep mode instead
/// * Maximum delay is limited by CLOCK_MAX_US (approximately 134 seconds)
/// * Never inlined to save code size at expense of function call overhead
/// * Off-target there is no register to spin on: the clock advances by `us` and
///   the duration is recorded in `fake_clock::sleeps` instead of blocking
#[inline(never)]
#[cfg_attr(target_os = "none", link_section = ".ram_code")]
pub fn sleep_us(us: u32) {
    #[cfg(target_os = "none")]
    {
        // Record starting time
        let t = clock_time();

        // Busy-wait until the specified time has elapsed
        while !clock_time_exceed(t, us) {}
    }

    #[cfg(not(target_os = "none"))]
    {
        // Never spin on host: advance the fake tick and log the requested delay
        fake_clock::advance_ticks(us as u64 * CLOCK_SYS_CLOCK_1US as u64);
        fake_clock::record_sleep(us);
    }
}

/// Host-side clock seam: the tick register, the 64-bit accumulator and the
/// recorded `sleep_us` durations.
///
/// Production code reads the free-running system tick register (`0x740`) and
/// masks interrupts to detect its 32-bit overflow. Off-target both are emulated:
/// the register lives in `tlsr8266_pac::FakeMmio` and the accumulator words are
/// the same module-scope statics [`clock_time64`] uses.
///
/// [`advance_ticks`] deliberately leaves `LAST_CLOCK_TIME` alone so
/// [`clock_time64`] observes a single 32-bit wraparound and bumps
/// `CLOCK_TIME_UPPER` exactly once, like silicon.
#[cfg(not(target_os = "none"))]
pub mod fake_clock {
    use super::{CLOCK_TIME_UPPER, IS_16MHZ_XTAL, LAST_CLOCK_TIME};
    use core::sync::atomic::Ordering;
    use std::sync::{Mutex, MutexGuard};
    use std::vec::Vec;
    use tlsr8266_pac::fake_mmio::FakeMmio;

    /// `reg_system_tick`: the free-running 32-bit system tick counter.
    const REG_SYSTEM_TICK: u32 = 0x740;

    /// Durations passed to `sleep_us`, in call order.
    static SLEEPS: Mutex<Vec<u32>> = Mutex::new(Vec::new());

    fn sleeps_lock() -> MutexGuard<'static, Vec<u32>> {
        SLEEPS.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Zero the tick register, both accumulator words and the sleep log.
    pub fn reset() {
        FakeMmio::set_reg32(REG_SYSTEM_TICK, 0);
        LAST_CLOCK_TIME.store(0, Ordering::Relaxed);
        CLOCK_TIME_UPPER.store(0, Ordering::Relaxed);
        sleeps_lock().clear();
    }

    /// Set the tick register and both accumulator words to `ticks`, leaving no
    /// pending overflow for `clock_time64()` to observe.
    pub fn set_ticks(ticks: u64) {
        FakeMmio::set_reg32(REG_SYSTEM_TICK, ticks as u32);
        LAST_CLOCK_TIME.store(ticks as u32, Ordering::Relaxed);
        CLOCK_TIME_UPPER.store((ticks >> 32) as u32, Ordering::Relaxed);
    }

    /// Advance the tick register by `delta`, adding whole 32-bit wraps to the
    /// upper word.
    ///
    /// `LAST_CLOCK_TIME` is intentionally untouched: the next `clock_time64()`
    /// call sees the low word go backwards and bumps the upper word once, which
    /// is exactly what the hardware path does.
    pub fn advance_ticks(delta: u64) {
        let ticks = FakeMmio::reg32(REG_SYSTEM_TICK).wrapping_add(delta as u32);
        FakeMmio::set_reg32(REG_SYSTEM_TICK, ticks);
        let upper = (CLOCK_TIME_UPPER.load(Ordering::Relaxed)).wrapping_add((delta >> 32) as u32);
        CLOCK_TIME_UPPER.store(upper, Ordering::Relaxed);
    }

    /// Seed the cached crystal result, as `clock_init()` would publish it.
    pub fn set_xtal_16mhz(is_16mhz: bool) {
        IS_16MHZ_XTAL.store(is_16mhz, Ordering::Relaxed);
    }

    /// The durations requested from `sleep_us`, oldest first.
    pub fn sleeps() -> Vec<u32> {
        sleeps_lock().clone()
    }

    /// Record one `sleep_us(us)` request. Called by `sleep_us`; not part of the
    /// test-facing API.
    pub(crate) fn record_sleep(us: u32) {
        sleeps_lock().push(us);
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::mcu::analog::fake_analog::FakeAnalog;
    use tlsr8266_pac::fake_mmio::{FakeMmio, RegWrite};
    use tlsr8266_pac::REG_BASE_ADDR;

    /// `reg_system_tick`, the free-running 32-bit counter `clock_time` reads.
    const SYSTEM_TICK: u32 = 0x740;
    /// `reg_clk_sel`, the clock source/divider byte.
    const CLK_SEL: u32 = 0x66;
    /// `reg_rst_clk0` (32-bit view) and `reg_tmr_ctrl` (32-bit view).
    const RST_CLK0: u32 = 0x60;
    const TMR_CTRL: u32 = 0x620;

    /// `FakeMmio`, `FakeAnalog` and `fake_clock` are process-wide singletons and
    /// the suite runs with `--test-threads=1`, so every test starts from zero.
    fn reset_fakes() {
        FakeMmio::reset();
        FakeAnalog::reset();
        fake_clock::reset();
    }

    /// Move the hardware tick register without telling the 64-bit accumulator.
    ///
    /// This is the host equivalent of silicon advancing the counter between two
    /// `clock_time64()` calls: `LAST_CLOCK_TIME` keeps the previous observation,
    /// so a value that went backwards is read as a 32-bit overflow.
    fn move_tick_register(ticks: u32) {
        FakeMmio::set_reg32(SYSTEM_TICK, ticks);
    }

    /// Base value the scripted system tick starts from.
    const SCRIPTED_TICK_BASE: u32 = 0x1000_0000;
    /// System ticks the scripted measurement window is reported to span.
    ///
    /// A `static` rather than a closure capture: `FakeMmio::on_read32` takes a
    /// plain `fn` pointer, so the hook cannot capture anything.
    static SCRIPTED_ELAPSED_TICKS: AtomicU32 = AtomicU32::new(0);

    /// Script the 32K_RC counter so crystal detection completes a full window
    /// with the supplied system-tick count, as a real board would.
    fn script_xtal_measurement(system_ticks: u32) {
        // Edge alignment: the edge (100), the first sample past it (101), then
        // the sample that closes the 32 tick window (133).
        FakeAnalog::push_reads(ANA_32K_COUNTER_LO, &[100, 101, 133]);
        FakeAnalog::set(ANA_32K_COUNTER_HI, 0);
        FakeMmio::set_reg8(CLK_SEL, 0x63);
        SCRIPTED_ELAPSED_TICKS.store(system_ticks, Ordering::Relaxed);
        FakeMmio::on_read32(SYSTEM_TICK, |call_idx| {
            if call_idx == 0 {
                SCRIPTED_TICK_BASE
            } else {
                SCRIPTED_TICK_BASE + SCRIPTED_ELAPSED_TICKS.load(Ordering::Relaxed)
            }
        });
        FakeMmio::clear_writes();
    }

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
    /// 1. Script the 32K_RC counter and the system tick so detection reports 16MHz
    /// 2. Call clock_init to trigger the configuration
    /// 3. Verify the ordered register writes and the published crystal result
    ///
    /// # Notes
    ///
    /// * Tests all four stages of clock initialization
    /// * Verifies USB clock configuration based on CLK_USB_ENABLE constant
    /// * Confirms the detected crystal is published through is_16mhz_xtal()
    /// * Confirms PLL divider calculation is correct
    /// * Verifies watchdog timeout is properly converted to register value
    #[test]
    fn test_clock_init() {
        reset_fakes();
        script_xtal_measurement(16_400);

        // Setup expected values
        let expected_rst_clk0 = 0xff000000 | FLD_CLK_EN::USB_EN.bits();

        let expected_clk_sel = MASK_VAL!(
            FLD_CLK_SEL::DIV.bits() as u32,
            CLOCK_PLL_CLOCK / CLOCK_SYS_CLOCK_1S,
            FLD_CLK_SEL::SRC.bits() as u32,
            CLOCK_SEL::SEL_HS_DIV as u32
        ) as u8;

        let expected_tmr_ctrl = MASK_VAL!(
            FLD_TMR::TMR0_EN.bits(),
            1,
            FLD_TMR::TMR_WD_CAPT.bits(),
            (WATCHDOG_INIT_TIMEOUT * CLOCK_SYS_CLOCK_1MS) >> WATCHDOG_TIMEOUT_COEFF,
            FLD_TMR::TMR_WD_EN.bits(),
            1
        );

        // The clock source detection switches to the crystal pad and restores
        // the value it read first.
        let measurement_src = MASK_VAL!(
            FLD_CLK_SEL::SRC.bits() as u32,
            CLOCK_SEL::SEL_16M_PAD as u32
        ) as u8;

        // Call the function being tested
        clock_init();

        // Verify the ordered write sequence: reset, measure, restore, configure,
        // arm the watchdog.
        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: RST_CLK0 | REG_BASE_ADDR,
                    width: 4,
                    value: expected_rst_clk0,
                },
                RegWrite {
                    addr: CLK_SEL | REG_BASE_ADDR,
                    width: 1,
                    value: measurement_src as u32,
                },
                RegWrite {
                    addr: CLK_SEL | REG_BASE_ADDR,
                    width: 1,
                    value: 0x63,
                },
                RegWrite {
                    addr: CLK_SEL | REG_BASE_ADDR,
                    width: 1,
                    value: expected_clk_sel as u32,
                },
                RegWrite {
                    addr: TMR_CTRL | REG_BASE_ADDR,
                    width: 4,
                    value: expected_tmr_ctrl,
                },
            ]
        );

        // Verify the crystal was detected once and its result published
        assert!(
            is_16mhz_xtal(),
            "clock_init should publish the detected crystal"
        );

        // Leave the shared state at its power-on default for other tests
        fake_clock::set_xtal_16mhz(false);
    }

    /// Tests clock_time function for proper system tick reading.
    ///
    /// This test verifies that the clock_time function correctly:
    /// - Reads the system tick register
    /// - Returns the register value as is
    ///
    /// # Algorithm
    ///
    /// 1. Seed the system tick register
    /// 2. Call clock_time function
    /// 3. Verify returned value matches the seeded tick value
    ///
    /// # Notes
    ///
    /// * Tests the basic operation of reading system time
    /// * Verifies function is a simple pass-through to register read
    #[test]
    fn test_clock_time() {
        reset_fakes();

        // Setup expected tick value
        let expected_tick = 0x12345678;
        move_tick_register(expected_tick);

        // Call the function being tested
        let result = clock_time();

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
    /// 1. Seed the tick register slightly ahead but not enough to exceed the span
    /// 2. Verify function returns false
    ///
    /// # Notes
    ///
    /// * Tests the negative condition (time not elapsed)
    /// * Verifies microsecond-to-tick conversion is applied correctly
    #[test]
    fn test_clock_time_exceed_not_elapsed() {
        reset_fakes();

        // Reference time
        let reference_time = 0x10000000;

        // Current is 1000 ticks ahead, requesting 50us = 32*50 = 1600 ticks
        fake_clock::set_ticks(reference_time as u64 + 1000);

        // Time span of 50us (should be 1600 ticks at 32MHz)
        let result = clock_time_exceed(reference_time, 50);

        // Should return false because not enough time has elapsed
        assert!(!result, "Should return false when time has not elapsed");
    }

    /// Tests clock_time_exceed function when the time span has elapsed.
    ///
    /// This test verifies that the clock_time_exceed function correctly:
    /// - Returns true when the elapsed time exceeds the requested span
    /// - Correctly converts microseconds to system ticks
    ///
    /// # Algorithm
    ///
    /// 1. Seed the tick register significantly ahead of the reference time
    /// 2. Verify function returns true
    ///
    /// # Notes
    ///
    /// * Tests the positive condition (time has elapsed)
    /// * Verifies microsecond-to-tick conversion is applied correctly
    #[test]
    fn test_clock_time_exceed_elapsed() {
        reset_fakes();

        // Reference time
        let reference_time = 0x10000000;

        // Current is 2000 ticks ahead, requesting 50us = 32*50 = 1600 ticks
        fake_clock::set_ticks(reference_time as u64 + 2000);

        // Time span of 50us (should be 1600 ticks at 32MHz)
        let result = clock_time_exceed(reference_time, 50);

        // Should return true because enough time has elapsed
        assert!(result, "Should return true when time has elapsed");
    }

    /// Tests clock_time_exceed function with timer overflow.
    ///
    /// This test verifies that the clock_time_exceed function correctly:
    /// - Handles timer wraparound correctly
    /// - Correctly calculates elapsed time despite the overflow
    ///
    /// # Algorithm
    ///
    /// 1. Seed the tick register with a value that would appear "less" than the
    ///    reference time
    /// 2. Verify function correctly detects elapsed time despite wraparound
    ///
    /// # Notes
    ///
    /// * Tests the wraparound condition using u32 arithmetic properties
    /// * This is a critical test case as overflow handling is essential for correct timing
    #[test]
    fn test_clock_time_exceed_overflow() {
        reset_fakes();

        // Reference time
        let reference_time = 0x10000000;

        // 0xFFFFFFFE - 0x10000000 + 2 = 0xEFFFFFFE (large positive difference)
        fake_clock::set_ticks(0xFFFFFFFE);

        // Check for 10ms elapsed time (should be true when wrapped around)
        let result = clock_time_exceed(reference_time, 10000);

        // Should return true because of wraparound arithmetic
        assert!(result, "Should correctly handle timer overflow");
    }

    /// Tests sleep_us function for the host timing contract.
    ///
    /// This test verifies that the sleep_us function correctly:
    /// - Advances the emulated system tick by the requested duration
    /// - Records the requested duration for assertion
    ///
    /// # Algorithm
    ///
    /// 1. Seed a known tick value
    /// 2. Call sleep_us with a specified duration
    /// 3. Verify the tick advanced by `us * 32` and the duration was recorded
    ///
    /// # Notes
    ///
    /// * The busy-wait loop itself is `cfg(target_os = "none")` only; off-target
    ///   the seam advances instantly and logs the request, so the host contract
    ///   is exactly "tick advanced once, duration recorded once"
    #[test]
    fn test_sleep_us() {
        reset_fakes();

        // Setup a fixed starting time
        let start_time: u32 = 0x12345678;
        fake_clock::set_ticks(start_time as u64);

        // Call the function being tested - sleep for 100us
        sleep_us(100);

        // Verify the tick advanced by the full duration (100us * 32 ticks/us)
        assert_eq!(
            clock_time(),
            start_time.wrapping_add(100 * CLOCK_SYS_CLOCK_1US)
        );

        // Verify the duration was recorded exactly once
        assert_eq!(fake_clock::sleeps(), vec![100]);
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
    fn test_clock_time_exceed_zero_duration() {
        reset_fakes();

        let reference_time = 1000;

        // Any elapsed time should exceed a zero duration
        fake_clock::set_ticks(reference_time as u64 + 1);

        let result = clock_time_exceed(reference_time, 0);
        assert!(result, "Zero duration should always be exceeded");
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
    fn test_clock_time_exceed_min_duration_not_exceeded() {
        reset_fakes();

        let reference_time = 1000;

        // 1 us at 32MHz = 32 ticks; current time is 31 ticks ahead (not enough)
        fake_clock::set_ticks(reference_time as u64 + 31);

        let result = clock_time_exceed(reference_time, 1);
        assert!(!result, "1 microsecond should require 32 ticks at 32MHz");
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
    fn test_clock_time_exceed_min_duration_exact() {
        reset_fakes();

        let reference_time = 1000;

        // Now advance to exactly enough ticks
        fake_clock::set_ticks(reference_time as u64 + 32);

        let result = clock_time_exceed(reference_time, 1);
        assert!(!result, "32 ticks should match exactly 1us (not exceed)");
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
    fn test_clock_time_exceed_min_duration_exceeded() {
        reset_fakes();

        let reference_time = 1000;

        // Now advance one more tick (just enough to exceed)
        fake_clock::set_ticks(reference_time as u64 + 33);

        let result = clock_time_exceed(reference_time, 1);
        assert!(result, "33 ticks should exceed 1us requirement");
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
    fn test_clock_time_exceed_max_duration_not_exceeded() {
        reset_fakes();

        let reference_time: u32 = 1000;

        // Test with a large but valid duration (close to max representable)
        let large_us = CLOCK_MAX_US - 1000;
        let large_ticks = large_us * CLOCK_SYS_CLOCK_1US;

        // Set current time to be less than required
        fake_clock::set_ticks(reference_time.wrapping_add(large_ticks - 1) as u64);

        let result = clock_time_exceed(reference_time, large_us);
        assert!(!result, "Large duration should not be exceeded yet");
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
    fn test_clock_time_exceed_max_duration_exceeded() {
        reset_fakes();

        let reference_time: u32 = 1000;

        // Test with a large but valid duration (close to max representable)
        let large_us = CLOCK_MAX_US - 1000;
        let large_ticks = large_us * CLOCK_SYS_CLOCK_1US;

        // Set current time to be more than required
        fake_clock::set_ticks(reference_time.wrapping_add(large_ticks + 1) as u64);

        let result = clock_time_exceed(reference_time, large_us);
        assert!(result, "Large duration should now be exceeded");
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
    /// 1. Queue `0x12` for the first high-byte read and `0x13` for the next three
    /// 2. Return a constant low byte
    /// 3. Call read_ana_32k and verify the second attempt's value is returned
    ///
    /// # Notes
    ///
    /// * The queued high bytes are consumed in read order, so the returned value
    ///   `0x1334` is only reachable if the retry loop actually ran: accepting the
    ///   first attempt would return `0x1234`, and reading a fifth sample would
    ///   fall through to the zeroed bank and change the result again
    #[test]
    fn test_read_ana_32k_retries_on_torn_high_byte() {
        reset_fakes();

        // Attempt 1 reads high (0x12), low, high (0x13) and is rejected; attempt
        // 2 reads high (0x13), low, high (0x13) and is accepted.
        FakeAnalog::push_reads(ANA_32K_COUNTER_HI, &[0x12, 0x13, 0x13, 0x13]);
        FakeAnalog::set(ANA_32K_COUNTER_LO, 0x34);

        let value = read_ana_32k();

        assert_eq!(
            value, 0x1334,
            "should accept only the high byte that stayed stable"
        );
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
    /// 1. Queue an alternating high-byte sequence so `hi1 != hi2` always
    /// 2. Call read_ana_32k and verify a best-effort value comes back
    ///
    /// # Notes
    ///
    /// * The queue holds exactly the samples the bounded loop is allowed to
    ///   consume; without the cap this test would never return
    #[test]
    fn test_read_ana_32k_bounded_when_high_byte_never_settles() {
        reset_fakes();

        // Two high-byte reads per attempt, and one attempt past the cap.
        let attempts = XTAL_DETECT_MAX_POLLS as usize + 1;
        let alternating: Vec<u8> = (0..attempts * 2)
            .map(|i| if i % 2 == 0 { 0x12 } else { 0x13 })
            .collect();
        FakeAnalog::push_reads(ANA_32K_COUNTER_HI, &alternating);
        FakeAnalog::set(ANA_32K_COUNTER_LO, 0x34);

        let value = read_ana_32k();

        assert_eq!(
            value, 0x1334,
            "should return the last sample instead of retrying forever"
        );
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
        reset_fakes();

        fake_clock::set_xtal_16mhz(true);
        assert!(is_16mhz_xtal(), "Should report a stored 16MHz result");

        fake_clock::set_xtal_16mhz(false);
        assert!(!is_16mhz_xtal(), "Should report a stored 12MHz result");
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
    /// 1. Script the 32K_RC counter to advance 1 tick to the edge, then 32 ticks
    /// 2. Script the system tick to advance 12,500 ticks over the window
    /// 3. Call the function and verify it reports false and restores the register
    ///
    /// # Notes
    ///
    /// * 12,500 is the measured centre of the 12MHz board range
    #[test]
    fn test_detect_xtal_is_16mhz_below_threshold() {
        reset_fakes();
        script_xtal_measurement(12_500);

        let result = detect_xtal_is_16mhz();

        assert!(!result, "12,500 ticks should be reported as 12MHz");
        // One write to switch sources, one to restore
        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: CLK_SEL | REG_BASE_ADDR,
                    width: 1,
                    value: MASK_VAL!(
                        FLD_CLK_SEL::SRC.bits() as u32,
                        CLOCK_SEL::SEL_16M_PAD as u32
                    ),
                },
                RegWrite {
                    addr: CLK_SEL | REG_BASE_ADDR,
                    width: 1,
                    value: 0x63,
                },
            ]
        );
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
    fn test_detect_xtal_is_16mhz_above_threshold() {
        reset_fakes();
        script_xtal_measurement(16_400);

        let result = detect_xtal_is_16mhz();

        assert!(result, "16,400 ticks should be reported as 16MHz");
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
    fn test_detect_xtal_is_16mhz_at_threshold() {
        assert_eq!(
            XTAL_DETECT_16MHZ_THRESHOLD, 14_500,
            "the documented threshold must not drift"
        );

        reset_fakes();
        script_xtal_measurement(XTAL_DETECT_16MHZ_THRESHOLD);

        let result = detect_xtal_is_16mhz();

        assert!(
            result,
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
    /// 1. Seed `reg_clk_sel` with `0x63` (divider 3, 32MHz pad)
    /// 2. Record every `reg_clk_sel` write in call order
    /// 3. Verify the writes are [SEL_16M_PAD source byte, previous value]
    ///
    /// # Notes
    ///
    /// * The recorded order proves the restore is the second and final write
    /// * A 16,400 tick count is used so the measurement completes normally
    #[test]
    fn test_detect_xtal_is_16mhz_restores_previous_clk_sel() {
        let previous = 0x63u8;
        let measurement_src = MASK_VAL!(
            FLD_CLK_SEL::SRC.bits() as u32,
            CLOCK_SEL::SEL_16M_PAD as u32
        ) as u8;

        reset_fakes();
        script_xtal_measurement(16_400);

        let result = detect_xtal_is_16mhz();

        assert!(result, "16,400 ticks should be reported as 16MHz");
        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: CLK_SEL | REG_BASE_ADDR,
                    width: 1,
                    value: measurement_src as u32,
                },
                RegWrite {
                    addr: CLK_SEL | REG_BASE_ADDR,
                    width: 1,
                    value: previous as u32,
                },
            ],
            "detection must restore the previously read reg_clk_sel value"
        );
        assert_eq!(FakeMmio::reg8(CLK_SEL), previous);
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
    fn test_detect_xtal_is_16mhz_reports_12mhz_when_32k_stalls() {
        reset_fakes();
        FakeAnalog::set(ANA_32K_COUNTER_HI, 0);
        FakeAnalog::set(ANA_32K_COUNTER_LO, 100);
        FakeMmio::set_reg8(CLK_SEL, 0x63);
        FakeMmio::on_read32(SYSTEM_TICK, |call_idx| {
            if call_idx == 0 {
                0x5000_0000
            } else {
                0x5000_0000 + 16_400
            }
        });
        FakeMmio::clear_writes();

        let result = detect_xtal_is_16mhz();

        assert!(
            !result,
            "an incomplete 32K_RC window must report the 12MHz default"
        );
        // One write to switch sources, one to restore
        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: CLK_SEL | REG_BASE_ADDR,
                    width: 1,
                    value: MASK_VAL!(
                        FLD_CLK_SEL::SRC.bits() as u32,
                        CLOCK_SEL::SEL_16M_PAD as u32
                    ),
                },
                RegWrite {
                    addr: CLK_SEL | REG_BASE_ADDR,
                    width: 1,
                    value: 0x63,
                },
            ],
            "the timeout path must still restore reg_clk_sel"
        );
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
    /// 1. Queue the edge (100), a sample past it (101), a backward glitch (99),
    ///    then one incrementing sample per poll
    /// 2. Model the system tick counter as a 16MHz measurement
    /// 3. Verify the result is true and the queue was fully consumed
    ///
    /// # Notes
    ///
    /// * Without the guard the glitch wraps to a delta of 65,534, closes the
    ///   window after three samples and leaves most of the queue behind; the
    ///   trailing read observes the leftover sample instead of the zeroed bank
    #[test]
    fn test_detect_xtal_is_16mhz_ignores_backward_glitch() {
        reset_fakes();

        // 32K_RC samples in read order: edge, first sample past it, glitch, then
        // one incrementing sample per poll (104, 105, ... 133).
        let mut samples = vec![100u8, 101, 99];
        samples.extend(104u8..=133);
        FakeAnalog::push_reads(ANA_32K_COUNTER_LO, &samples);
        FakeAnalog::set(ANA_32K_COUNTER_HI, 0);
        FakeMmio::set_reg8(CLK_SEL, 0x63);
        FakeMmio::on_read32(SYSTEM_TICK, |call_idx| {
            if call_idx == 0 {
                0x6000_0000
            } else {
                0x6000_0000 + 16_400
            }
        });
        FakeMmio::clear_writes();

        let result = detect_xtal_is_16mhz();

        assert!(result, "a backward glitch must not truncate the 32K window");
        // The bank byte is 0, so an empty queue is the only way this read can
        // return 0: every queued sample must have been consumed.
        assert_eq!(
            FakeAnalog::get(ANA_32K_COUNTER_LO),
            0,
            "the ignored glitch should cost one poll, not the window"
        );
    }

    #[test]
    fn test_clock_time64_no_overflow() {
        reset_fakes();

        // The accumulator last saw 1000; the counter has moved 100 ticks on.
        fake_clock::set_ticks(1000);
        move_tick_register(1100);

        let result = clock_time64();

        assert_eq!(result, 1100);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 0);
        assert_eq!(LAST_CLOCK_TIME.load(Ordering::Relaxed), 1100);
    }

    #[test]
    fn test_clock_time64_with_overflow() {
        reset_fakes();

        // The counter was near the top of its range, then wrapped to 100.
        fake_clock::set_ticks(0xFFFFFF00);
        move_tick_register(100);

        let result = clock_time64();

        // The upper 32 bits should be incremented to 1, and lower 32 bits should be our next value
        assert_eq!(result, (1_u64 << 32) | 100);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 1);
        assert_eq!(LAST_CLOCK_TIME.load(Ordering::Relaxed), 100);
    }

    #[test]
    fn test_clock_time64_normal_increment() {
        reset_fakes();

        fake_clock::set_ticks(1000);
        move_tick_register(1500);

        let result = clock_time64();

        assert_eq!(result, 1500);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_clock_time64_first_overflow() {
        reset_fakes();

        fake_clock::set_ticks(0xFFFFFF00);
        move_tick_register(100);

        let result = clock_time64();

        assert_eq!(result, (1_u64 << 32) | 100);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_clock_time64_after_overflow() {
        reset_fakes();

        // Already one wrap deep, and moving forward inside the same epoch.
        fake_clock::set_ticks((1_u64 << 32) | 100);
        move_tick_register(200);

        let result = clock_time64();

        assert_eq!(result, (1_u64 << 32) | 200);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_clock_time64_second_overflow() {
        reset_fakes();

        // One wrap deep and near the top of the range again.
        fake_clock::set_ticks((1_u64 << 32) | 0xFFFFFF00);
        move_tick_register(300);

        let result = clock_time64();

        assert_eq!(result, (2_u64 << 32) | 300);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 2);
    }

    #[test]
    fn test_fake_clock_reset_clears_state() {
        reset_fakes();

        // Arrange: leave non-zero accumulator state behind, as a prior proof test would
        CLOCK_TIME_UPPER.store(7, Ordering::Relaxed);
        LAST_CLOCK_TIME.store(0x1234_5678, Ordering::Relaxed);
        move_tick_register(0xDEAD_BEEF);
        sleep_us(5);

        // Act
        fake_clock::reset();

        // Assert
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 0);
        assert_eq!(LAST_CLOCK_TIME.load(Ordering::Relaxed), 0);
        assert_eq!(clock_time(), 0);
        assert!(fake_clock::sleeps().is_empty());
    }

    #[test]
    fn test_fake_clock_advance_ticks_wraps_low_word_once() {
        reset_fakes();

        // Last observation sits at the top of the low word; `advance_ticks`
        // rolls the register over but deliberately leaves LAST_CLOCK_TIME
        // alone, so `clock_time64` sees one wrap and bumps the upper word once.
        fake_clock::set_ticks(0xFFFF_FFF0);
        fake_clock::advance_ticks(0x20);

        let result = clock_time64();

        assert_eq!(result, (1_u64 << 32) | 0x10);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 1);
        assert_eq!(LAST_CLOCK_TIME.load(Ordering::Relaxed), 0x10);

        // A second read observes no further wrap.
        assert_eq!(clock_time64(), (1_u64 << 32) | 0x10);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_fake_clock_advance_ticks_carries_multiple_wraps() {
        reset_fakes();

        // A delta spanning more than one low-word period must add every whole
        // 32-bit wrap to the upper word in the same `advance_ticks` call.
        fake_clock::advance_ticks(0x3_0000_0020);

        let result = clock_time64();

        assert_eq!(result, (3_u64 << 32) | 0x20);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 3);
        assert_eq!(LAST_CLOCK_TIME.load(Ordering::Relaxed), 0x20);
    }
}
