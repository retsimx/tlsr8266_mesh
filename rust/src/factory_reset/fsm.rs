//! Pure bit-mask model of the factory-reset power-cycle sequence.
//!
//! This module owns the constants, the [`ResetStep`] bit algebra and the two
//! transition helpers, and nothing else. It performs no I/O, touches no global
//! state and needs no mocks: `factory_reset` reads and writes the reset-count
//! sector, while `fsm` only answers "what is the next flash value?".
//!
//! # Live progression
//!
//! The whole sequence lives in one flash byte. Every power cycle clears one
//! more bit; flash can only clear bits (a program is an AND), which is why the
//! sequence is a countdown of `1`s rather than a counter:
//!
//! ```text
//! value  step           bit cleared            when
//! 0xFF   Clear          -                      erased sector / idle
//! 0xFE   Step1          bit 0 (0x01)           0-3 s power cycle
//! 0xFC   Step2          bit 1 (0x02)           0-3 s power cycle
//! 0xF8   Step3          bit 2 (0x04)           0-3 s power cycle
//! 0xF0   Step4          bit 3 (0x08)           3-30 s power cycle
//! 0xD0   Step4 (timed)  bit 5 (0x20)           3 s minimum time validated
//! 0xC0   Step5          bit 4 (0x10)           3-30 s power cycle
//! 0x80   FactoryReset   bit 6 (0x40)           3 s minimum time validated
//! ```
//!
//! Bit 7 is the sequence-valid flag. Invalidating an aborted sequence clears it
//! (`value & `[`INVALIDATION_MASK`]), so `0xF0` becomes `0x70`; the sector scan
//! then moves on to the next erased slot and the sequence starts over.
//!
//! The two bits above the step-completion bits are *timing* bits, allocated
//! only to the steps whose minimum power-on time is non-zero (steps 4 and 5:
//! bit 5 and bit 6). `ResetStep::flash_value()` returns a step's base value as
//! if it were written onto an erased byte, i.e. `Step5` is `0xE0`; the live
//! path applies `apply_to` to the byte on the card instead, which keeps the
//! step-4 timing bit set and yields `0xC0`.

/// Timing sequence for factory reset detection (in seconds).
///
/// Format: `(min_time, max_time)` per power cycle step.
///
/// * First number: minimum seconds the device must stay powered on.
/// * Second number: maximum seconds the device may stay powered on.
///
/// `(0, 3)` means "turn it off within 3 seconds"; `(3, 30)` means "leave it on
/// for at least 3 s but no longer than 30 s". Following every window in order
/// triggers the factory reset; missing one aborts the sequence.
pub const POWER_CYCLE_TIMING: [(u8, u8); 5] = [
    (0, 3),  // Step 1: off within 3 s
    (0, 3),  // Step 2: off within 3 s
    (0, 3),  // Step 3: off within 3 s
    (3, 30), // Step 4: on for 3-30 s
    (3, 30), // Step 5: on for 3-30 s
];

/// Compile-time validation of the timing table: every step needs one
/// completion bit and every step with a minimum time needs one timing bit, and
/// bit 7 is reserved for the valid flag.
const _: () = {
    let len = POWER_CYCLE_TIMING.len();
    assert!(len >= 2, "POWER_CYCLE_TIMING must have at least 2 entries");
    assert!(len <= 5, "POWER_CYCLE_TIMING must have at most 5 entries");
    let mut timing_bits_needed = 0;
    let mut i = 0;
    while i < len {
        if POWER_CYCLE_TIMING[i].0 > 0 {
            timing_bits_needed += 1;
        }
        i += 1;
    }
    assert!(
        len + timing_bits_needed <= 7,
        "Not enough bits for all steps and timing validation"
    );
};

/// Number of steps in the power cycle sequence (derived from the timing table).
pub const POWER_CYCLE_COUNT: u8 = POWER_CYCLE_TIMING.len() as u8;

/// The flash value that triggers the factory reset: every completion and
/// timing bit cleared, bit 7 (valid flag) still set.
pub const FACTORY_RESET_FLAG: u8 = 0x80;

/// Bit 7 cleared: the state of a sequence that was never started (see
/// [`ResetStep::is_flash_value_invalidated`], which reads the same bit).
pub const RESET_SEQUENCE_CLEAR: u8 = 0;

/// Invalidation mask: clears bit 7, aborting the sequence in one bit flip.
pub const INVALIDATION_MASK: u8 = 0x7F;

/// One position in the power-cycle sequence, derived from the flash byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetStep {
    /// No sequence in progress (flash erased or invalidated).
    Clear,
    /// Step 1 complete: `0xFF` -> `0xFE` (clear bit 0).
    Step1,
    /// Step 2 complete: `0xFE` -> `0xFC` (clear bit 1).
    Step2,
    /// Step 3 complete: `0xFC` -> `0xF8` (clear bit 2).
    Step3,
    /// Step 4 complete: `0xF8` -> `0xF0` (clear bit 3).
    Step4,
    /// Step 5 complete: `0xD0` -> `0xC0` (clear bit 4).
    Step5,
    /// All steps and both timing windows complete (`0x80`).
    FactoryReset,
}

impl ResetStep {
    /// Bit position of a step's completion bit (step 1 -> bit 0, ...).
    pub(super) const fn step_completion_bit(step_number: u8) -> u8 {
        step_number - 1
    }

    /// Bit position of a step's timing-validation bit, if the step has one.
    ///
    /// Timing bits are allocated after every completion bit, in step order.
    pub(super) const fn step_timing_bit(step_number: u8) -> Option<u8> {
        // Only steps with min_time > 0 get a timing validation bit.
        if step_number == 0 || step_number > POWER_CYCLE_COUNT {
            return None;
        }

        let timing = POWER_CYCLE_TIMING[step_number as usize - 1];
        if timing.0 == 0 {
            return None;
        }

        // Count how many steps before this one also need timing validation.
        let mut timing_bit_offset = POWER_CYCLE_COUNT; // Start after all completion bits.
        let mut i = 0;
        while i < step_number - 1 {
            if POWER_CYCLE_TIMING[i as usize].0 > 0 {
                timing_bit_offset += 1;
            }
            i += 1;
        }

        Some(timing_bit_offset)
    }

    /// The factory-reset trigger value: bit 7 set, every other bit cleared.
    pub(super) const fn factory_reset_value() -> u8 {
        FACTORY_RESET_FLAG
    }

    /// The bitmask that clears this step's completion bit.
    pub(super) const fn bitmask(self) -> u8 {
        match self {
            ResetStep::Clear => 0xFF,
            ResetStep::Step1 => !(1 << Self::step_completion_bit(1)),
            ResetStep::Step2 => !(1 << Self::step_completion_bit(2)),
            ResetStep::Step3 => !(1 << Self::step_completion_bit(3)),
            ResetStep::Step4 => !(1 << Self::step_completion_bit(4)),
            ResetStep::Step5 => !(1 << Self::step_completion_bit(5)),
            ResetStep::FactoryReset => Self::factory_reset_value(),
        }
    }

    /// The flash value this step leaves behind when written onto an erased
    /// (`0xFF`) byte: every completion bit up to and including this step's is
    /// cleared and the timing bits stay set.
    pub const fn flash_value(self) -> u8 {
        match self {
            ResetStep::FactoryReset => Self::factory_reset_value(),
            step => {
                let mut value = 0xFF;
                let mut bit = 0;
                while bit < step.step_number() {
                    value &= !(1 << bit);
                    bit += 1;
                }
                value
            }
        }
    }

    /// Apply this step's completion bitmask to the byte currently on the card.
    pub const fn apply_to(self, value: u8) -> u8 {
        value & self.bitmask()
    }

    /// Whether this step completes the power cycle sequence.
    pub const fn is_sequence_complete(self) -> bool {
        matches!(self, ResetStep::FactoryReset)
    }

    /// The timing window for this step, if it has one.
    pub const fn timing_window(self) -> Option<(u8, u8)> {
        match self {
            ResetStep::Clear => None,
            ResetStep::Step1 => Some(POWER_CYCLE_TIMING[0]),
            ResetStep::Step2 => Some(POWER_CYCLE_TIMING[1]),
            ResetStep::Step3 => Some(POWER_CYCLE_TIMING[2]),
            ResetStep::Step4 => Some(POWER_CYCLE_TIMING[3]),
            ResetStep::Step5 => Some(POWER_CYCLE_TIMING[4]),
            ResetStep::FactoryReset => None,
        }
    }

    /// Abort a sequence by clearing bit 7 (`0xF0` -> `0x70`).
    pub const fn invalidate_flash_value(value: u8) -> u8 {
        value & INVALIDATION_MASK
    }

    /// Whether a flash value is an aborted sequence: bit 7 clear and not the
    /// erased value.
    pub const fn is_flash_value_invalidated(value: u8) -> bool {
        (value & FACTORY_RESET_FLAG) == RESET_SEQUENCE_CLEAR && value != 0xFF
    }

    /// Clear a step's timing-validation bit (mark its minimum window met).
    pub const fn mark_step_timing_validated(value: u8, step_number: u8) -> u8 {
        if let Some(timing_bit) = Self::step_timing_bit(step_number) {
            value & !(1 << timing_bit)
        } else {
            value // No timing validation needed for this step.
        }
    }

    /// Whether a step's timing-validation bit is cleared. Steps without a
    /// timing bit, and every invalidated value, report `false`/`true` as
    /// documented below.
    pub const fn is_step_timing_validated(value: u8, step_number: u8) -> bool {
        if Self::is_flash_value_invalidated(value) {
            return false;
        }

        if let Some(timing_bit) = Self::step_timing_bit(step_number) {
            (value & (1 << timing_bit)) == 0
        } else {
            true // No timing validation needed, always considered validated.
        }
    }

    /// Whether a flash value triggers the factory reset (`0x80` exactly).
    pub const fn should_factory_reset(value: u8) -> bool {
        value == Self::factory_reset_value()
    }

    /// Whether a step requires timing validation (has a minimum time).
    pub const fn step_requires_timing(step_number: u8) -> bool {
        if step_number == 0 || step_number > POWER_CYCLE_COUNT {
            return false;
        }
        POWER_CYCLE_TIMING[step_number as usize - 1].0 > 0
    }

    /// Convert a flash value back to its step.
    ///
    /// An invalidated value is reported as [`ResetStep::Clear`]: the sequence
    /// has to start over.
    pub const fn from_flash_value(value: u8) -> Self {
        // First check if sequence is invalidated.
        if Self::is_flash_value_invalidated(value) {
            return ResetStep::Clear;
        }

        // The lowest still-set completion bit is the step not done yet; with
        // bits 0..=4 tracked, `value & 0x1F` decides it.
        match value & 0x1F {
            val if (val & 0x01) != 0 => ResetStep::Clear, // Step 1 not done yet.
            val if (val & 0x02) != 0 => ResetStep::Step1, // Step 1 done, step 2 not.
            val if (val & 0x04) != 0 => ResetStep::Step2, // Steps 1-2 done, step 3 not.
            val if (val & 0x08) != 0 => ResetStep::Step3, // Steps 1-3 done, step 4 not.
            val if (val & 0x10) != 0 => ResetStep::Step4, // Steps 1-4 done, step 5 not.
            // All completion bits clear: either the flag itself or a step 5
            // that still has a timing bit set (`0xE0`/`0xD0`/`0xC0`).
            _ => match value {
                FACTORY_RESET_FLAG => ResetStep::FactoryReset,
                _ => ResetStep::Step5,
            },
        }
    }

    /// The step number (0 for none, 1..=5 for the steps, 6 for the flag).
    pub const fn step_number(self) -> u8 {
        match self {
            ResetStep::Clear => 0,
            ResetStep::Step1 => 1,
            ResetStep::Step2 => 2,
            ResetStep::Step3 => 3,
            ResetStep::Step4 => 4,
            ResetStep::Step5 => 5,
            ResetStep::FactoryReset => 6, // Special flag for factory reset ready.
        }
    }

    /// The inverse of [`ResetStep::step_number`]; anything else is `Clear`.
    pub const fn from_step_number(step: u8) -> Self {
        match step {
            0 => ResetStep::Clear,
            1 => ResetStep::Step1,
            2 => ResetStep::Step2,
            3 => ResetStep::Step3,
            4 => ResetStep::Step4,
            5 => ResetStep::Step5,
            FACTORY_RESET_FLAG => ResetStep::FactoryReset,
            _ => ResetStep::Clear,
        }
    }
}

/// The byte to write for the next step, given the byte just observed at boot.
///
/// `Clear` (erased or invalidated) starts the sequence; a step below
/// [`POWER_CYCLE_COUNT`] advances to the next step's completion bit applied to
/// `current`. `None` means the boot path must not write: the sequence is
/// already complete, or it is already at its last step.
pub fn boot_step_value(current: u8) -> Option<u8> {
    let step = ResetStep::from_flash_value(current);
    if step == ResetStep::Clear {
        // Start the sequence by completing step 1.
        return Some(ResetStep::Step1.flash_value());
    }

    // The flag itself needs no advance write; the caller performs the reset.
    if step.is_sequence_complete() {
        return None;
    }

    let next_step_number = step.step_number() + 1;
    if next_step_number > POWER_CYCLE_COUNT {
        return None;
    }

    Some(ResetStep::from_step_number(next_step_number).apply_to(current))
}

/// The new flash value once a step's minimum-time window has been validated.
///
/// `Some(new)` only when a timing bit is actually cleared, so the caller never
/// programs the byte it already holds.
pub fn timing_validated_value(value: u8, step_number: u8) -> Option<u8> {
    if ResetStep::is_step_timing_validated(value, step_number) {
        return None;
    }

    let validated = ResetStep::mark_step_timing_validated(value, step_number);
    if validated == value {
        None
    } else {
        Some(validated)
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;

    /// The live sequence is pinned end to end: every boot-advance write and
    /// both in-boot timing-validation writes, in the order a user produces them.
    #[test]
    fn test_live_progression_boot_and_timing_writes() {
        // Step 1: erased sector, first boot writes the step 1 completion bit.
        assert_eq!(boot_step_value(0xFF), Some(0xFE));

        // Steps 2-4: each boot clears the next completion bit.
        assert_eq!(boot_step_value(0xFE), Some(0xFC));
        assert_eq!(boot_step_value(0xFC), Some(0xF8));
        assert_eq!(boot_step_value(0xF8), Some(0xF0));

        // Step 4 minimum time (3 s) validated during the boot: bit 5 cleared.
        assert_eq!(ResetStep::from_flash_value(0xF0), ResetStep::Step4);
        assert_eq!(timing_validated_value(0xF0, 4), Some(0xD0));

        // Step 5: the next boot advances to 0xC0, keeping the timed bit 5.
        assert_eq!(boot_step_value(0xD0), Some(0xC0));

        // Step 5 minimum time (3 s) validated during the boot: bit 6 cleared.
        assert_eq!(timing_validated_value(0xC0, 5), Some(0x80));

        // 0x80 is the factory-reset flag: no further writes, and the boot path
        // reports the reset instead.
        assert_eq!(boot_step_value(0x80), None);
        assert!(ResetStep::should_factory_reset(0x80));
    }

    /// Each of the five power cycles clears exactly its own completion bit, and
    /// the sequence ends on the factory-reset flag.
    #[test]
    fn test_each_step_clears_its_own_completion_bit() {
        let mut value = 0xFFu8;
        for step_number in 1..=POWER_CYCLE_COUNT {
            let next = boot_step_value(value).expect("every step advances");
            assert_eq!(
                value & !next,
                1 << ResetStep::step_completion_bit(step_number),
                "step {step_number} must clear exactly its own completion bit"
            );
            value = next;

            // Steps 4 and 5 also validate their minimum window in the same boot.
            if ResetStep::step_requires_timing(step_number) {
                let validated =
                    timing_validated_value(value, step_number).expect("timing bit must clear");
                assert_eq!(
                    value & !validated,
                    1 << ResetStep::step_timing_bit(step_number).expect("timing bit allocated")
                );
                value = validated;
            }
        }

        assert_eq!(value, FACTORY_RESET_FLAG);
        assert!(ResetStep::should_factory_reset(value));
    }

    /// Steps 1-3 use the 0-3 s window, steps 4-5 the 3-30 s window.
    #[test]
    fn test_timing_windows_per_step() {
        for step in 1..=POWER_CYCLE_COUNT {
            let (min_time, max_time) = ResetStep::from_step_number(step)
                .timing_window()
                .expect("every step has a window");
            match step {
                1..=3 => {
                    assert_eq!((min_time, max_time), (0, 3), "step {step} is a quick cycle");
                    assert!(!ResetStep::step_requires_timing(step));
                }
                _ => {
                    assert_eq!((min_time, max_time), (3, 30), "step {step} is a long cycle");
                    assert!(ResetStep::step_requires_timing(step));
                }
            }
        }
        assert_eq!(ResetStep::Clear.timing_window(), None);
        assert_eq!(ResetStep::FactoryReset.timing_window(), None);
    }

    /// Only the 3-30 s steps have a timing bit, and the 0-3 s steps never
    /// produce a timing write.
    #[test]
    fn test_timing_writes_only_for_long_windows() {
        for step in 1..=3 {
            assert_eq!(timing_validated_value(0xFF, step), None);
        }

        // A step whose timing bit is already clear needs no write.
        assert_eq!(timing_validated_value(0xD0, 4), None);
        assert_eq!(timing_validated_value(0x80, 5), None);
        // An invalidated value never reads as validated, but a bit that is
        // already clear still means there is nothing to program.
        assert_eq!(timing_validated_value(0x10, 5), None);

        // Steps 4 and 5 clear exactly one bit each.
        assert_eq!(timing_validated_value(0xF0, 4), Some(0xD0));
        assert_eq!(timing_validated_value(0xC0, 5), Some(0x80));
    }

    /// Invalidation aborts the sequence in one bit flip, and an invalidated
    /// value reads back as "start over".
    #[test]
    fn test_invalidation_aborts_then_restarts() {
        for &(valid, invalidated) in &[
            (0xFEu8, 0x7Eu8),
            (0xFC, 0x7C),
            (0xF8, 0x78),
            (0xF0, 0x70),
            (0xD0, 0x50),
            (0xC0, 0x40),
        ] {
            assert_eq!(ResetStep::invalidate_flash_value(valid), invalidated);
            assert!(ResetStep::is_flash_value_invalidated(invalidated));
            assert!(!ResetStep::is_flash_value_invalidated(valid));
            assert_eq!(
                ResetStep::from_flash_value(invalidated),
                ResetStep::Clear,
                "0x{invalidated:02X} must restart the sequence"
            );
            // A restart writes step 1 again.
            assert_eq!(boot_step_value(invalidated), Some(0xFE));
        }

        // The erased value is not "invalidated": it is an unused slot.
        assert!(!ResetStep::is_flash_value_invalidated(0xFF));
    }

    /// The sequence is only complete at exactly `0x80`.
    #[test]
    fn test_completion_requires_both_windows() {
        // Every completion bit cleared but no timing validation.
        assert_eq!(ResetStep::Step5.flash_value(), 0xE0);
        assert!(!ResetStep::should_factory_reset(0xE0));

        // Step 4 timed only.
        assert!(!ResetStep::should_factory_reset(0xC0));

        // Both timed.
        assert_eq!(timing_validated_value(0xC0, 5), Some(0x80));
        assert!(ResetStep::should_factory_reset(0x80));
        assert!(ResetStep::from_flash_value(0x80).is_sequence_complete());

        // A never-started or invalidated value is not a reset.
        assert!(!ResetStep::should_factory_reset(0xFF));
        assert!(!ResetStep::should_factory_reset(0x00));
    }

    /// No write is produced once the sequence has run out of steps.
    #[test]
    fn test_no_write_past_the_last_step() {
        assert_eq!(boot_step_value(0xC0), None); // Step 5 done, timing pending.
        assert_eq!(boot_step_value(0xE0), None); // Step 5 done, base value.
        assert_eq!(boot_step_value(0x80), None); // Flag.
    }
}
