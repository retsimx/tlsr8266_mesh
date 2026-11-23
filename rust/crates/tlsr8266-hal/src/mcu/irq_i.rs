//!
//! # TLSR8266 Interrupt Control Interface
//!
//! This module provides low-level functions for managing the interrupt system
//! on the TLSR8266 microcontroller. These functions enable critical sections
//! through interrupt masking and restoration.
//!
//! The module supports the following operations:
//! - Enabling/disabling global interrupts
//! - Saving and restoring interrupt state
//! - Initializing the interrupt system with default masks
//!

use critical_section::RawRestoreState;
use tlsr8266_pac::{
    read_reg_irq_en, read_reg_irq_mask, read_reg_irq_src, write_reg_irq_en, write_reg_irq_mask,
    FLD_IRQ,
};

/// The crate-wide `critical_section` implementation.
///
/// Interrupt masking is owned by this module, so the `critical_section` crate's
/// single global impl lives here: `acquire` masks interrupts and returns the
/// previous enable state, `release` restores that saved state. Off-target
/// (`cfg!(target_os = "none")` is false) no hardware is touched and the state is
/// always `0`.
struct TlsrCriticalSection;
critical_section::set_impl!(TlsrCriticalSection);

#[coverage(off)]
// SAFETY: (2) `critical_section::Impl` contract. The trait is unsafe because
// `acquire`/`release` must nest 1:1 and mask every preemption source; this impl
// delegates to `irq_disable`/`irq_restore` (whole enable register saved and
// restored) and it is the crate's single registered impl.
unsafe impl critical_section::Impl for TlsrCriticalSection {
    // SAFETY: (2) critical-section acquire. Masks interrupts and returns the
    // previous enable state for `release`; it never allocates or panics, and the
    // `critical_section` depth counter is the only caller.
    unsafe fn acquire() -> RawRestoreState {
        if cfg!(target_os = "none") {
            return irq_disable();
        };
        0
    }

    // SAFETY: (2) critical-section release. Restores the state returned by the
    // matching `acquire`; pairing is enforced by the `critical_section` crate.
    unsafe fn release(state: RawRestoreState) {
        if cfg!(target_os = "none") {
            irq_restore(state)
        }
    }
}

/// Default interrupt configuration value used during initialization.
///
/// This value enables two specific interrupt sources:
/// - Timer1 interrupts (FLD_IRQ::TMR1_EN)
/// - ZigBee/BLE RF real-time interrupts (FLD_IRQ::ZB_RT_EN)
///
/// These interrupts are essential for the mesh networking and timing functions.
pub const IRQ_INIT_VALUE: u32 = FLD_IRQ::TMR1_EN.bits() | FLD_IRQ::ZB_RT_EN.bits();

/// Enables global interrupts on the TLSR8266 MCU.
///
/// # Algorithm
/// 1. Reads the current interrupt enable register state
/// 2. Enables global interrupts by writing 1 to the enable register
/// 3. Returns the previous state for later restoration
///
/// # Returns
///
/// * `u8` - The previous interrupt enable register value (0 or 1)
///
/// # Notes
///
/// * Used to start a non-critical section after a critical section
/// * Should be paired with irq_restore() when the non-critical section ends
/// * The return value can be passed to irq_restore() to restore previous state
pub fn irq_enable() -> u8 {
    // Step 1: Read current interrupt enable state before modification
    let r = read_reg_irq_en(); // don't worry, the compiler will optimize the return value if not used
                               // Step 2: Enable global interrupts by writing 1 to enable register
    write_reg_irq_en(1);
    // Step 3: Return previous state so it can be restored later
    r
}

/// Disables global interrupts on the TLSR8266 MCU.
///
/// # Algorithm
/// 1. Reads the current interrupt enable register state
/// 2. Disables global interrupts by writing 0 to the enable register
/// 3. Returns the previous state for later restoration
///
/// # Returns
///
/// * `u8` - The previous interrupt enable register value (0 or 1)
///
/// # Notes
///
/// * Used to start a critical section where interrupts should not occur
/// * Should be paired with irq_restore() when the critical section ends
/// * When interrupts are disabled, time-sensitive operations can be
///   performed without being interrupted, but this should be kept as
///   short as possible to avoid missing important interrupts
pub fn irq_disable() -> u8 {
    // Step 1: Read current interrupt enable state before modification
    let r = read_reg_irq_en(); // don't worry, the compiler will optimize the return value if not used
                               // Step 2: Disable global interrupts by writing 0 to enable register
    write_reg_irq_en(0);
    // Step 3: Return previous state so it can be restored later
    r
}

/// Restores the global interrupt state to a previously saved value.
///
/// # Parameters
///
/// * `en` - The interrupt enable value to restore (typically obtained from
///   irq_enable() or irq_disable())
///
/// # Algorithm
/// 1. Writes the provided value to the interrupt enable register
///
/// # Notes
///
/// * Used to end a critical section by restoring the interrupt state
///   that existed before the critical section began
/// * Should be paired with irq_disable() or irq_enable() which provides
///   the previous state to restore
pub fn irq_restore(en: u8) {
    // Write saved interrupt enable state back to the register
    write_reg_irq_en(en);
}

/// Initializes the interrupt system with default configuration.
///
/// # Algorithm
/// 1. Writes the IRQ_INIT_VALUE to the interrupt mask register to
///    enable specific interrupt sources needed by the application
///
/// # Notes
///
/// * This function should be called early in the boot process
/// * Enables only the necessary interrupts defined in IRQ_INIT_VALUE:
///   - Timer1 interrupts for timing functions
///   - ZigBee/BLE real-time interrupts for wireless communication
/// * Other interrupts can be enabled later as needed by the application
pub fn irq_init() {
    // Configure which specific interrupt sources are enabled
    write_reg_irq_mask(IRQ_INIT_VALUE);
}

/// Snapshot of the pending interrupt sources (`reg_irq_src`).
///
/// # Notes
///
/// * A newtype rather than a bare `u32` keeps "which bit means what" inside
///   this module, so callers ask predicates instead of re-deriving masks
/// * Constructed by [`read_irq_sources`] and consumed immediately by the
///   interrupt dispatcher; it owns the value, so it is not `Copy`
pub struct IrqSources(u32);

/// Reads the pending interrupt sources.
///
/// # Returns
///
/// * [`IrqSources`] wrapping the current `reg_irq_src` value
pub fn read_irq_sources() -> IrqSources {
    IrqSources(read_reg_irq_src())
}

impl IrqSources {
    /// Returns whether the system timer interrupt is pending.
    #[inline(always)]
    pub fn is_system_timer(&self) -> bool {
        self.0 & FLD_IRQ::SYSTEM_TIMER.bits() != 0
    }

    /// Returns whether the TMR0 interrupt is pending.
    #[inline(always)]
    pub fn is_tmr0(&self) -> bool {
        self.0 & FLD_IRQ::TMR0_EN.bits() != 0
    }

    /// Returns whether the TMR1 interrupt is pending.
    #[inline(always)]
    pub fn is_tmr1(&self) -> bool {
        self.0 & FLD_IRQ::TMR1_EN.bits() != 0
    }
}

/// Unmasks the TMR0 interrupt source.
///
/// # Notes
///
/// * Read-modify-write of `reg_irq_mask`, so already enabled sources stay
///   enabled
pub fn irq_enable_tmr0() {
    write_reg_irq_mask(read_reg_irq_mask() | FLD_IRQ::TMR0_EN.bits());
}

/// Unmasks the TMR1 interrupt source.
///
/// # Notes
///
/// * Read-modify-write of `reg_irq_mask`, so already enabled sources stay
///   enabled
pub fn irq_enable_tmr1() {
    write_reg_irq_mask(read_reg_irq_mask() | FLD_IRQ::TMR1_EN.bits());
}

/// Test suite for the interrupt control interface
#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use tlsr8266_pac::fake_mmio::{FakeMmio, RegWrite};
    use tlsr8266_pac::REG_BASE_ADDR;

    /// `reg_irq_en` (global interrupt enable) and `reg_irq_mask`.
    const IRQ_EN: u32 = 0x643;
    const IRQ_MASK: u32 = 0x640;

    /// `FakeMmio` is process-wide and the suite runs with `--test-threads=1`,
    /// so every test starts from a zeroed bank.
    fn reset_fakes() {
        FakeMmio::reset();
    }

    /// Tests enabling global interrupts.
    ///
    /// This test verifies that irq_enable correctly:
    /// - Reads the current interrupt state
    /// - Enables interrupts by writing 1 to the register
    /// - Returns the previous state
    ///
    /// # Algorithm
    ///
    /// 1. Seed `reg_irq_en` with 0 (interrupts disabled)
    /// 2. Call irq_enable()
    /// 3. Verify the return value is the previous state
    /// 4. Verify the enable register was written with 1
    ///
    /// # Notes
    ///
    /// * Tests the case where interrupts were previously disabled
    #[test]
    fn test_irq_enable_when_disabled() {
        reset_fakes();
        // Step 1: interrupts disabled
        FakeMmio::set_reg8(IRQ_EN, 0);
        FakeMmio::clear_writes();

        // Step 2: Call the function being tested
        let previous_state = irq_enable();

        // Step 3: Verify the previous state is returned correctly
        assert_eq!(
            previous_state, 0,
            "irq_enable should return the previous state (0)"
        );

        // Step 4: Verify the enable register was written with the correct value
        assert_eq!(FakeMmio::reg8(IRQ_EN), 1);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: IRQ_EN | REG_BASE_ADDR,
                width: 1,
                value: 1,
            }]
        );
    }

    /// Tests enabling global interrupts when already enabled.
    ///
    /// This test verifies that irq_enable correctly:
    /// - Reads the current interrupt state
    /// - Still writes to the enable register (idempotent operation)
    /// - Returns the previous state
    ///
    /// # Algorithm
    ///
    /// 1. Seed `reg_irq_en` with 1 (enabled)
    /// 2. Call irq_enable()
    /// 3. Verify the return value is 1 and the register still holds 1
    ///
    /// # Notes
    ///
    /// * Tests the case where interrupts were already enabled
    /// * Confirms the function is idempotent
    #[test]
    fn test_irq_enable_when_already_enabled() {
        reset_fakes();
        // Step 1: interrupts enabled
        FakeMmio::set_reg8(IRQ_EN, 1);
        FakeMmio::clear_writes();

        // Step 2: Call the function being tested
        let previous_state = irq_enable();

        // Step 3: Verify the previous state is returned correctly
        assert_eq!(
            previous_state, 1,
            "irq_enable should return the previous state (1)"
        );

        // Step 4: Verify the write still happened with value 1
        assert_eq!(FakeMmio::reg8(IRQ_EN), 1);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: IRQ_EN | REG_BASE_ADDR,
                width: 1,
                value: 1,
            }]
        );
    }

    /// Tests disabling global interrupts.
    ///
    /// This test verifies that irq_disable correctly:
    /// - Reads the current interrupt state
    /// - Disables interrupts by writing 0 to the register
    /// - Returns the previous state
    ///
    /// # Algorithm
    ///
    /// 1. Seed `reg_irq_en` with 1 (enabled)
    /// 2. Call irq_disable()
    /// 3. Verify the return value is 1 and the register now holds 0
    ///
    /// # Notes
    ///
    /// * Tests the case where interrupts were previously enabled
    #[test]
    fn test_irq_disable_when_enabled() {
        reset_fakes();
        // Step 1: interrupts enabled
        FakeMmio::set_reg8(IRQ_EN, 1);
        FakeMmio::clear_writes();

        // Step 2: Call the function being tested
        let previous_state = irq_disable();

        // Step 3: Verify the previous state is returned correctly
        assert_eq!(
            previous_state, 1,
            "irq_disable should return the previous state (1)"
        );

        // Step 4: Verify the enable register was written with 0
        assert_eq!(FakeMmio::reg8(IRQ_EN), 0);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: IRQ_EN | REG_BASE_ADDR,
                width: 1,
                value: 0,
            }]
        );
    }

    /// Tests disabling global interrupts when already disabled.
    ///
    /// This test verifies that irq_disable correctly:
    /// - Reads the current interrupt state
    /// - Still writes to the enable register (idempotent operation)
    /// - Returns the previous state
    ///
    /// # Algorithm
    ///
    /// 1. Seed `reg_irq_en` with 0 (disabled)
    /// 2. Call irq_disable()
    /// 3. Verify the return value is 0 and the register still holds 0
    ///
    /// # Notes
    ///
    /// * Tests the case where interrupts were already disabled
    /// * Confirms the function is idempotent
    #[test]
    fn test_irq_disable_when_already_disabled() {
        reset_fakes();
        // Step 1: interrupts disabled
        FakeMmio::set_reg8(IRQ_EN, 0);
        FakeMmio::clear_writes();

        // Step 2: Call the function being tested
        let previous_state = irq_disable();

        // Step 3: Verify the previous state is returned correctly
        assert_eq!(
            previous_state, 0,
            "irq_disable should return the previous state (0)"
        );

        // Step 4: Verify the write still happened with value 0
        assert_eq!(FakeMmio::reg8(IRQ_EN), 0);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: IRQ_EN | REG_BASE_ADDR,
                width: 1,
                value: 0,
            }]
        );
    }

    /// Tests restoring global interrupts to enabled state.
    ///
    /// This test verifies that irq_restore correctly:
    /// - Writes the provided state (1) to the enable register
    ///
    /// # Algorithm
    ///
    /// 1. Reset the bank
    /// 2. Call irq_restore(1) to restore enabled state
    /// 3. Verify the enable register holds 1
    ///
    /// # Notes
    ///
    /// * Tests restoring to enabled state (1)
    #[test]
    fn test_irq_restore_to_enabled() {
        reset_fakes();

        // Call the function being tested
        irq_restore(1);

        // Verify the enable register was written with 1
        assert_eq!(FakeMmio::reg8(IRQ_EN), 1);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: IRQ_EN | REG_BASE_ADDR,
                width: 1,
                value: 1,
            }]
        );
    }

    /// Tests restoring global interrupts to disabled state.
    ///
    /// This test verifies that irq_restore correctly:
    /// - Writes the provided state (0) to the enable register
    ///
    /// # Algorithm
    ///
    /// 1. Seed the enable register with 1 so the write is visible
    /// 2. Call irq_restore(0) to restore disabled state
    /// 3. Verify the enable register holds 0
    ///
    /// # Notes
    ///
    /// * Tests restoring to disabled state (0)
    #[test]
    fn test_irq_restore_to_disabled() {
        reset_fakes();
        FakeMmio::set_reg8(IRQ_EN, 1);
        FakeMmio::clear_writes();

        // Call the function being tested
        irq_restore(0);

        // Verify the enable register was written with 0
        assert_eq!(FakeMmio::reg8(IRQ_EN), 0);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: IRQ_EN | REG_BASE_ADDR,
                width: 1,
                value: 0,
            }]
        );
    }

    /// Tests initializing the interrupt system.
    ///
    /// This test verifies that irq_init correctly:
    /// - Writes the IRQ_INIT_VALUE to the interrupt mask register
    ///
    /// # Algorithm
    ///
    /// 1. Reset the bank
    /// 2. Call irq_init()
    /// 3. Verify the mask register holds IRQ_INIT_VALUE
    ///
    /// # Notes
    ///
    /// * Verifies the correct initialization value is used
    #[test]
    fn test_irq_init() {
        reset_fakes();

        // Call the function being tested
        irq_init();

        // Verify the mask register was written with the correct value
        assert_eq!(FakeMmio::reg32(IRQ_MASK), IRQ_INIT_VALUE);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: IRQ_MASK | REG_BASE_ADDR,
                width: 4,
                value: IRQ_INIT_VALUE,
            }]
        );
    }

    /// Tests a typical critical section pattern using the interrupt functions.
    ///
    /// This test verifies the typical usage pattern:
    /// - Disable interrupts and save previous state
    /// - Perform operations in critical section (simulated)
    /// - Restore previous interrupt state
    ///
    /// # Algorithm
    ///
    /// 1. Seed the enable register with 1 (enabled)
    /// 2. Call irq_disable() to start the critical section
    /// 3. Call irq_restore() with the returned state
    /// 4. Verify the ordered writes are [0, 1] and the register ends enabled
    ///
    /// # Notes
    ///
    /// * Demonstrates and tests the common critical section pattern
    #[test]
    fn test_critical_section_pattern() {
        reset_fakes();
        // Step 1: interrupts were enabled
        FakeMmio::set_reg8(IRQ_EN, 1);
        FakeMmio::clear_writes();

        // Step 2: Call irq_disable() to start the critical section
        let previous_state = irq_disable();

        // Simulate some operations in the critical section...

        // Step 3: Call irq_restore() to end the critical section
        irq_restore(previous_state);

        // Step 4: Verify the enable register was written correctly for both operations
        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: IRQ_EN | REG_BASE_ADDR,
                    width: 1,
                    value: 0,
                },
                RegWrite {
                    addr: IRQ_EN | REG_BASE_ADDR,
                    width: 1,
                    value: 1,
                },
            ],
            "disable then restore, in that order"
        );
        assert_eq!(FakeMmio::reg8(IRQ_EN), 1, "interrupts end enabled");
    }

    // Proof slices for the semantic IRQ helpers: these exercise the real
    // accessor bodies, which route through the in-process `FakeMmio` bank. Each
    // test resets the bank first because it is process-wide
    // (`--test-threads=1`).

    /// Tests that the system timer predicate follows bit 20 of `reg_irq_src`.
    #[test]
    fn test_read_irq_sources_reports_system_timer() {
        FakeMmio::reset();
        FakeMmio::set_reg32(0x648, 0x0010_0000); // FLD_IRQ::SYSTEM_TIMER

        let sources = read_irq_sources();

        assert!(sources.is_system_timer(), "bit 20 is the system timer");
        assert!(!sources.is_tmr0(), "TMR0 must not be reported");
        assert!(!sources.is_tmr1(), "TMR1 must not be reported");
    }

    /// Tests that the TMR0 predicate follows bit 0 of `reg_irq_src`.
    #[test]
    fn test_read_irq_sources_reports_tmr0() {
        FakeMmio::reset();
        FakeMmio::set_reg32(0x648, 0x0000_0001); // FLD_IRQ::TMR0_EN

        let sources = read_irq_sources();

        assert!(sources.is_tmr0(), "bit 0 is TMR0");
        assert!(
            !sources.is_system_timer(),
            "the system timer must not be reported"
        );
        assert!(!sources.is_tmr1(), "TMR1 must not be reported");
    }

    /// Tests that the TMR1 predicate follows bit 1 of `reg_irq_src`.
    #[test]
    fn test_read_irq_sources_reports_tmr1() {
        FakeMmio::reset();
        FakeMmio::set_reg32(0x648, 0x0000_0002); // FLD_IRQ::TMR1_EN

        let sources = read_irq_sources();

        assert!(sources.is_tmr1(), "bit 1 is TMR1");
        assert!(
            !sources.is_system_timer(),
            "the system timer must not be reported"
        );
        assert!(!sources.is_tmr0(), "TMR0 must not be reported");
    }

    /// Tests that a cleared `reg_irq_src` reports no pending source.
    #[test]
    fn test_read_irq_sources_zero_reports_nothing() {
        FakeMmio::reset();
        FakeMmio::set_reg32(0x648, 0);

        let sources = read_irq_sources();

        assert!(!sources.is_system_timer());
        assert!(!sources.is_tmr0());
        assert!(!sources.is_tmr1());
    }

    /// Tests that irq_enable_tmr0 ORs only the TMR0 mask bit.
    ///
    /// # Notes
    ///
    /// * The seeded mask holds unrelated bits (9, 11 and 13), which must survive
    ///   the read-modify-write untouched
    #[test]
    fn test_irq_enable_tmr0_preserves_existing_mask_bits() {
        const EXISTING_MASK: u32 = 0x0000_2A00;

        FakeMmio::reset();
        FakeMmio::set_reg32(0x640, EXISTING_MASK);
        FakeMmio::clear_writes();

        irq_enable_tmr0();

        assert_eq!(
            FakeMmio::reg32(0x640),
            EXISTING_MASK | FLD_IRQ::TMR0_EN.bits()
        );
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: 0x640 | REG_BASE_ADDR,
                width: 4,
                value: EXISTING_MASK | FLD_IRQ::TMR0_EN.bits(),
            }]
        );
    }

    /// Tests that irq_enable_tmr1 ORs only the TMR1 mask bit.
    ///
    /// # Notes
    ///
    /// * The seeded mask holds unrelated bits (9, 11 and 13), which must survive
    ///   the read-modify-write untouched
    #[test]
    fn test_irq_enable_tmr1_preserves_existing_mask_bits() {
        const EXISTING_MASK: u32 = 0x0000_2A00;

        FakeMmio::reset();
        FakeMmio::set_reg32(0x640, EXISTING_MASK);
        FakeMmio::clear_writes();

        irq_enable_tmr1();

        assert_eq!(
            FakeMmio::reg32(0x640),
            EXISTING_MASK | FLD_IRQ::TMR1_EN.bits()
        );
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: 0x640 | REG_BASE_ADDR,
                width: 4,
                value: EXISTING_MASK | FLD_IRQ::TMR1_EN.bits(),
            }]
        );
    }
}
