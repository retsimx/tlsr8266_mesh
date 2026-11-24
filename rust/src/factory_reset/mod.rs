//! Factory-reset detection: policy here, bit algebra in [`fsm`].
//!
//! The device triggers a factory reset when the user power-cycles it along the
//! five-step timing sequence. Progress lives in a single byte of the
//! reset-count sector; each boot advances it by clearing one bit. [`fsm`] owns
//! that bit algebra (which bit, which timing window, what the next value is),
//! and this module owns everything that touches the device: the
//! [`FactoryResetState`] atomics, the sector scan, the flash writes and the
//! returned actions.
//!
//! # Live progressions
//!
//! Successful sequence, one power cycle per arrow (bits only ever clear):
//!
//! ```text
//! 0xFF → 0xFE → 0xFC → 0xF8 → 0xF0 → 0xD0 → 0xC0 → 0x80
//!        step1  step2  step3  step4  step4  step5  flag
//!                                      timed         timed
//! ```
//!
//! `0xF0 → 0xD0` and `0xC0 → 0x80` are written by [`check_timing_step`] during
//! the boot that *entered* the step: they are the 3 s minimum-time validations
//! for steps 4 and 5 (bits 5 and 6). The next boot after `0x80` reports
//! [`ResetAction::FactoryResetAndReboot`].
//!
//! Aborted sequence (timing violation or an invalid step):
//!
//! ```text
//! 0xC0 → 0x40 (0xC0 & INVALIDATION_MASK), then the scan moves to the next
//! erased slot and the sequence starts over at 0xFE on the following boot.
//! ```
//!
//! # Side effects belong to the caller
//!
//! This module never disables interrupts, blinks an LED, reboots the part or
//! writes pairing credentials: it returns [`ResetAction`] / [`KickoutAction`]
//! and the application layer executes them. That keeps the module free of
//! `tlsr8266_radio`, `crate::light`, `crate::ota` and the PAC crate, and lets the
//! whole policy run against the host flash/clock fakes.

use core::sync::atomic::{AtomicU32, AtomicU8};

use tlsr8266_hal::common::sync::SimplifyLS;
use tlsr8266_hal::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};
use tlsr8266_hal::mcu::clock::clock_time_exceed;

use crate::config::FLASH_ADR_RESET_CNT;

mod fsm;

use fsm::{
    boot_step_value, timing_validated_value, ResetStep, POWER_CYCLE_COUNT, POWER_CYCLE_TIMING,
};

/// Maximum address in flash memory.
const FLASH_ADR_PAR_MAX: u32 = 0x80000;

/// Base address of the MAC/configuration sector in 512 KiB flash. The first
/// sector of the configuration area is preserved across a factory reset; every
/// sector above it is erased. Aliased to the config constant so the address has
/// one definition.
const CFG_SECTOR_ADR_MAC_CODE: u32 = crate::config::FLASH_ADR_MAC;

/// Size of one erasable flash sector.
const SECTOR_SIZE: u32 = 0x1000;

/// Bytes of the reset-count sector read per [`flash_read_page`] call by
/// [`init_reset_sequence`]. One call per chunk instead of one per byte keeps the
/// 4 KiB scan off the 4,096-call path.
const RESET_SECTOR_SCAN_CHUNK: usize = 256;

/// Slots in the reset-count sector: once all of them are used the sector is
/// erased and the sequence starts again at slot 0.
const RESET_SECTOR_SLOTS: u32 = SECTOR_SIZE;

/// Factory reset detection state.
///
/// Guards the scan cursor and the timing check that spans the gap between two
/// power cycles. All four fields are process-global because the boot path and
/// the main-loop tick are separate call sites.
struct FactoryResetState {
    /// Flash address index (slot inside the reset-count sector) in use.
    flash_address_index: AtomicU32,
    /// Step the device is currently on (0 when no sequence is in progress).
    consecutive_reset_count: AtomicU8,
    /// Timing check state: 0 = idle, 1 = waiting for the minimum time,
    /// 2 = waiting for the maximum time.
    clear_state: AtomicU8,
    /// Timestamp (seconds) of the timing check's current limit.
    timing_check_timestamp: AtomicU32,
}

impl FactoryResetState {
    const fn new() -> Self {
        Self {
            flash_address_index: AtomicU32::new(0),
            consecutive_reset_count: AtomicU8::new(0),
            clear_state: AtomicU8::new(3),
            timing_check_timestamp: AtomicU32::new(0),
        }
    }
}

/// Global factory reset detection state.
static FACTORY_RESET_STATE: FactoryResetState = FactoryResetState::new();

/// What the boot path must do after [`handle_boot_sequence`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResetAction {
    /// No reset pending: the caller continues booting normally.
    None,
    /// The sequence completed: erase the configuration and reboot.
    FactoryResetAndReboot,
}

/// Why the device is being removed from its mesh.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KickoutReason {
    /// Device is being removed from its current mesh network.
    OutOfMesh = 0,
    /// Device has a default name that needs to be changed.
    DefaultName = 1,
    /// Maximum mode value (used for bounds checking).
    ModeMax = 2,
}

impl TryFrom<u32> for KickoutReason {
    type Error = ();

    /// Convert the wire value from `LGT_CMD_KICK_OUT`; anything else fails.
    fn try_from(v: u32) -> Result<Self, Self::Error> {
        match v {
            x if x == KickoutReason::OutOfMesh as u32 => Ok(KickoutReason::OutOfMesh),
            x if x == KickoutReason::DefaultName as u32 => Ok(KickoutReason::DefaultName),
            x if x == KickoutReason::ModeMax as u32 => Ok(KickoutReason::ModeMax),
            _ => Err(()),
        }
    }
}

/// What the caller must do after [`plan_kick_out`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KickoutAction {
    /// Erase the configuration and reboot; nothing needs re-writing.
    ErasePairingAndReboot,
    /// Erase first, then re-write the out-of-mesh pairing credentials, then
    /// reboot.
    WriteOutOfMeshCredentialsAndReboot,
}

/// Read one byte of flash through the HAL.
///
/// The HAL routes this to `FakeFlash` in host tests, so the module needs no
/// raw-pointer access of its own.
fn read_flash_byte(addr: u32) -> u8 {
    let mut byte = 0u8;
    flash_read_page(addr, 1, &mut byte);
    byte
}

/// Program one byte of flash: the only write shape the sequence needs.
fn write_flash_byte(addr: u32, value: u8) {
    let data = [value];
    flash_write_page(addr, 1, data.as_ptr());
}

/// Address of the slot currently in use.
fn active_slot() -> u32 {
    FLASH_ADR_RESET_CNT + FACTORY_RESET_STATE.flash_address_index.get()
}

/// Advance the slot cursor, erasing the sector once every slot is used.
fn advance_slot() {
    FACTORY_RESET_STATE.flash_address_index.inc();
    if FACTORY_RESET_STATE.flash_address_index.get() >= RESET_SECTOR_SLOTS {
        flash_erase_sector(FLASH_ADR_RESET_CNT);
        FACTORY_RESET_STATE.flash_address_index.set(0);
    }
}

/// Write the current step in the reset sequence.
///
/// Test-only: the boot path advances through [`boot_step_value`], so this
/// direct "program step `step`" view exists only to pin what each step leaves
/// on an erased byte, including the sector wrap at the last slot.
#[cfg(test)]
fn write_reset_sequence(step: u8) {
    let reset_step = ResetStep::from_step_number(step);
    let current_addr = active_slot();
    let current_value = read_flash_byte(current_addr);

    // A step write starts from the step's base value, so a timing bit left over
    // from an earlier step cannot leak into the new one.
    let new_value = if reset_step == ResetStep::Clear || reset_step == ResetStep::FactoryReset {
        reset_step.apply_to(current_value)
    } else {
        reset_step.flash_value()
    };

    if new_value != current_value {
        write_flash_byte(current_addr, new_value);
    }

    // A completed sequence moves to the next slot.
    if reset_step.is_sequence_complete() {
        advance_slot();
    }
}

/// Mark the current step as having satisfied its minimum power-on time.
fn write_timing_validation() {
    let current_addr = active_slot();
    let current_value = read_flash_byte(current_addr);
    let step_number = FACTORY_RESET_STATE.consecutive_reset_count.get();

    if let Some(validated_value) = timing_validated_value(current_value, step_number) {
        write_flash_byte(current_addr, validated_value);
    }
}

/// Read the step number stored in the active slot.
///
/// Test-only: production reads the raw byte so it can also test for the
/// factory-reset flag, but the byte-to-step conversion is what the sequence
/// scan and the timing check both depend on.
#[cfg(test)]
fn read_reset_sequence() -> u8 {
    ResetStep::from_flash_value(read_flash_byte(active_slot())).step_number()
}

/// Abort the sequence: invalidate the active slot and move to the next one.
fn clear_reset_sequence() {
    let current_addr = active_slot();
    let current_value = read_flash_byte(current_addr);

    // Only an in-progress sequence has anything to invalidate.
    if current_value != 0xFF {
        write_flash_byte(
            current_addr,
            ResetStep::invalidate_flash_value(current_value),
        );
    }

    advance_slot();
}

/// Find the slot holding the latest sequence and prime the state from it.
///
/// The 4 KiB sector is read in [`RESET_SECTOR_SCAN_CHUNK`]-byte chunks; the
/// last programmed slot wins, whatever its value. If the last one was
/// invalidated the sequence continues in the next (erased) slot, and a
/// completely full sector is erased to make room.
fn init_reset_sequence() {
    // Start scanning from the beginning of the sector.
    FACTORY_RESET_STATE.flash_address_index.set(0);

    let mut latest_idx = 0u32;
    let mut latest_value = 0xFFu8; // Erased state.
    let mut found_any_sequence = false;

    let mut chunk = [0xFFu8; RESET_SECTOR_SCAN_CHUNK];
    let mut scanned = 0u32;
    while scanned < SECTOR_SIZE {
        flash_read_page(
            FLASH_ADR_RESET_CNT + scanned,
            RESET_SECTOR_SCAN_CHUNK as u32,
            chunk.as_mut_ptr(),
        );
        for (offset, &value) in chunk.iter().enumerate() {
            // Skip completely erased locations; the last programmed slot is
            // the latest one whether it is valid or invalidated.
            if value != 0xFF {
                latest_idx = scanned + offset as u32;
                latest_value = value;
                found_any_sequence = true;
            }
        }
        scanned += RESET_SECTOR_SCAN_CHUNK as u32;
    }

    // A sequence that failed its timing check continues in the next slot.
    if found_any_sequence && ResetStep::is_flash_value_invalidated(latest_value) {
        latest_idx += 1;
        latest_value = 0xFF; // Next position should be erased.
    }

    // No erased slot left (or the cursor ran past the sector): start over.
    if latest_idx >= RESET_SECTOR_SLOTS - 1 {
        flash_erase_sector(FLASH_ADR_RESET_CNT);
        latest_idx = 0;
        latest_value = 0xFF;
    }

    FACTORY_RESET_STATE.flash_address_index.set(latest_idx);
    FACTORY_RESET_STATE
        .consecutive_reset_count
        .set(ResetStep::from_flash_value(latest_value).step_number());
}

/// Run the boot half of the sequence: scan the sector, advance one step.
///
/// Returns [`ResetAction::FactoryResetAndReboot`] *without writing anything*
/// when the active slot already holds the factory-reset flag — the caller owns
/// the erase/LED/reboot sequence. Otherwise it writes at most one byte (the
/// next step's value) and returns [`ResetAction::None`].
pub fn handle_boot_sequence() -> ResetAction {
    init_reset_sequence();

    // Check for the factory reset condition before any step processing.
    let flash_value = read_flash_byte(active_slot());
    if ResetStep::should_factory_reset(flash_value) {
        return ResetAction::FactoryResetAndReboot;
    }

    // Start (or continue) the timing check for the step this boot observed.
    match ResetStep::from_flash_value(flash_value) {
        ResetStep::Clear => {
            FACTORY_RESET_STATE.consecutive_reset_count.set(1);
            FACTORY_RESET_STATE.clear_state.set(1);

            if let Some((min_time, _max_time)) = ResetStep::Step1.timing_window() {
                FACTORY_RESET_STATE
                    .timing_check_timestamp
                    .set(min_time as u32);
            }
        }
        step => {
            let next_step_number = step.step_number() + 1;
            if next_step_number <= POWER_CYCLE_COUNT {
                FACTORY_RESET_STATE
                    .consecutive_reset_count
                    .set(next_step_number);
                FACTORY_RESET_STATE.clear_state.set(1);

                let next_step = ResetStep::from_step_number(next_step_number);
                if let Some((min_time, _max_time)) = next_step.timing_window() {
                    FACTORY_RESET_STATE
                        .timing_check_timestamp
                        .set(min_time as u32);
                }
            }
        }
    }

    // Advance the sequence by clearing the next completion bit.
    if let Some(new_value) = boot_step_value(flash_value) {
        write_flash_byte(active_slot(), new_value);
    }

    ResetAction::None
}

/// Run the main-loop half of the sequence: validate the current step's timing.
///
/// State 1 waits for the minimum power-on time and then writes the step's
/// timing bit. State 2 waits for the maximum time and aborts the sequence if it
/// is exceeded. Every reachable path leaves the device running, so this always
/// returns [`ResetAction::None`]; the signature keeps both call sites uniform.
pub fn check_timing_step() -> ResetAction {
    // No timing check active.
    if FACTORY_RESET_STATE.clear_state.get() == 0 {
        return ResetAction::None;
    }

    let current_step_number = FACTORY_RESET_STATE.consecutive_reset_count.get();
    if current_step_number == 0 || current_step_number > POWER_CYCLE_COUNT {
        // Invalid state, clear the sequence.
        clear_reset_sequence();
        FACTORY_RESET_STATE.clear_state.set(0);
        return ResetAction::None;
    }

    let (min_time, max_time) = POWER_CYCLE_TIMING[current_step_number as usize - 1];

    match FACTORY_RESET_STATE.clear_state.get() {
        1 => {
            // State 1: the minimum time must have elapsed.
            if clock_time_exceed(0, min_time as u32 * 1000 * 1000) {
                // Only steps with a minimum time have a bit to write.
                if ResetStep::step_requires_timing(current_step_number) {
                    write_timing_validation();
                }

                FACTORY_RESET_STATE.clear_state.set(2);
                FACTORY_RESET_STATE
                    .timing_check_timestamp
                    .set(max_time as u32);
            }
        }
        2 => {
            // State 2: the device must be switched off before the maximum time.
            if clock_time_exceed(
                0,
                FACTORY_RESET_STATE.timing_check_timestamp.get() * 1000 * 1000,
            ) {
                clear_reset_sequence();
                FACTORY_RESET_STATE.clear_state.set(0);
            }
        }
        _ => {
            // Invalid clear state.
            clear_reset_sequence();
            FACTORY_RESET_STATE.clear_state.set(0);
        }
    }

    ResetAction::None
}

/// Decide what a kick-out reason needs. Pure: the caller erases and reboots.
pub fn plan_kick_out(reason: KickoutReason) -> KickoutAction {
    match reason {
        KickoutReason::OutOfMesh => KickoutAction::WriteOutOfMeshCredentialsAndReboot,
        KickoutReason::DefaultName | KickoutReason::ModeMax => KickoutAction::ErasePairingAndReboot,
    }
}

/// Erase every configuration sector, preserving the MAC sector.
///
/// The reset-count sector is erased **last**: if power is lost partway through
/// the reset, a partially erased configuration is still recoverable because the
/// sequence state that points at it survives.
pub fn erase_config_sectors() {
    // Erasing is a multi-step hardware sequence that must not be interrupted.
    critical_section::with(|_| {
        // Every 4 KiB sector between the MAC sector and the end of flash; the
        // first one is skipped to preserve the MAC address.
        for i in 1..((FLASH_ADR_PAR_MAX - CFG_SECTOR_ADR_MAC_CODE) / SECTOR_SIZE) {
            let adr = CFG_SECTOR_ADR_MAC_CODE + i * SECTOR_SIZE;

            // Skip the active reset-count slot's sector during the main pass.
            if active_slot() != adr {
                flash_erase_sector(adr);
            }
        }

        flash_erase_sector(active_slot());
    });
}

/// Reset the module's globals for host tests: no sequence in progress.
///
/// The flash and clock fakes are the tests' own business; this only clears the
/// process-wide state so one test cannot inherit a timing check from the boot
/// path another test exercised.
#[cfg(test)]
pub fn reset_for_test() {
    FACTORY_RESET_STATE.flash_address_index.set(0);
    FACTORY_RESET_STATE.consecutive_reset_count.set(0);
    FACTORY_RESET_STATE.clear_state.set(0);
    FACTORY_RESET_STATE.timing_check_timestamp.set(0);
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::config::FLASH_ADR_PAIRING;
    use core::sync::atomic::Ordering;
    use tlsr8266_hal::drivers::flash::fake_flash::{FakeFlash, SECTOR_SIZE_BYTES};
    use tlsr8266_hal::mcu::clock::fake_clock;

    /// The HAL host seams are process-wide singletons, so every test that
    /// observes them starts from a blank flash and a zeroed tick counter.
    fn reset_fakes() {
        fake_clock::reset();
        FakeFlash::reset();
    }

    /// Reset the module's own globals to "booted on a blank sector".
    fn reset_state() {
        reset_for_test();
    }

    /// Program `value` into `offset` the way an earlier boot would have.
    fn seed_slot(offset: u32, value: u8) {
        FakeFlash::write_slice(FLASH_ADR_RESET_CNT + offset, &[value]);
    }

    /// Overwrite a slot with an exact value (programming alone can only clear
    /// bits, so the sector is erased first).
    fn set_slot(offset: u32, value: u8) {
        FakeFlash::erase_sector(FLASH_ADR_RESET_CNT + offset);
        seed_slot(offset, value);
    }

    /// Read back one byte of the reset-count sector.
    fn read_slot(offset: u32) -> u8 {
        let mut got = [0u8; 1];
        FakeFlash::read_slice(FLASH_ADR_RESET_CNT + offset, &mut got);
        got[0]
    }

    // ========================================================================
    // Bit algebra
    // ========================================================================

    /// The shipped timing table is the 5-step one.
    #[test]
    fn test_power_cycle_timing_validation() {
        assert!(POWER_CYCLE_TIMING.len() >= 2);
        assert!(POWER_CYCLE_TIMING.len() <= 5);

        assert_eq!(POWER_CYCLE_TIMING.len(), 5);
        assert_eq!(POWER_CYCLE_TIMING[0], (0, 3)); // Steps 1-3: quick cycles.
        assert_eq!(POWER_CYCLE_TIMING[1], (0, 3));
        assert_eq!(POWER_CYCLE_TIMING[2], (0, 3));
        assert_eq!(POWER_CYCLE_TIMING[3], (3, 30)); // Steps 4-5: timing validation required.
        assert_eq!(POWER_CYCLE_TIMING[4], (3, 30));
    }

    /// Step completion bits are sequential from bit 0.
    #[test]
    fn test_step_completion_bits() {
        assert_eq!(ResetStep::step_completion_bit(1), 0); // Step 1 -> bit 0
        assert_eq!(ResetStep::step_completion_bit(2), 1); // Step 2 -> bit 1
        assert_eq!(ResetStep::step_completion_bit(3), 2); // Step 3 -> bit 2
        assert_eq!(ResetStep::step_completion_bit(4), 3); // Step 4 -> bit 3
        assert_eq!(ResetStep::step_completion_bit(5), 4); // Step 5 -> bit 4
    }

    /// Timing bits start after the completion bits; only steps 4-5 get one.
    #[test]
    fn test_timing_validation_bits() {
        assert_eq!(ResetStep::step_timing_bit(1), None);
        assert_eq!(ResetStep::step_timing_bit(2), None);
        assert_eq!(ResetStep::step_timing_bit(3), None);

        assert_eq!(ResetStep::step_timing_bit(4), Some(5));
        assert_eq!(ResetStep::step_timing_bit(5), Some(6));
    }

    /// Only steps with a minimum time require validation.
    #[test]
    fn test_step_requires_timing() {
        assert!(!ResetStep::step_requires_timing(0)); // Invalid step
        assert!(!ResetStep::step_requires_timing(1)); // Step 1: min_time = 0
        assert!(!ResetStep::step_requires_timing(2)); // Step 2: min_time = 0
        assert!(!ResetStep::step_requires_timing(3)); // Step 3: min_time = 0
        assert!(ResetStep::step_requires_timing(4)); // Step 4: min_time = 3
        assert!(ResetStep::step_requires_timing(5)); // Step 5: min_time = 3
        assert!(!ResetStep::step_requires_timing(6)); // Invalid step
    }

    /// Base values written onto an erased byte, as documented in `fsm`.
    #[test]
    fn test_flash_values() {
        assert_eq!(ResetStep::Clear.flash_value(), 0xFF); // Erased state
        assert_eq!(ResetStep::Step1.flash_value(), 0xFE); // Bit 0 cleared
        assert_eq!(ResetStep::Step2.flash_value(), 0xFC); // Bits 0,1 cleared
        assert_eq!(ResetStep::Step3.flash_value(), 0xF8); // Bits 0,1,2 cleared
        assert_eq!(ResetStep::Step4.flash_value(), 0xF0); // Bits 0,1,2,3 cleared
        assert_eq!(ResetStep::Step5.flash_value(), 0xE0); // Bits 0,1,2,3,4 cleared
        assert_eq!(ResetStep::FactoryReset.flash_value(), 0x80); // Factory reset ready
    }

    /// Each step's mask clears exactly its own bit.
    #[test]
    fn test_bitmask_method() {
        assert_eq!(ResetStep::Clear.bitmask(), 0xFF);
        assert_eq!(ResetStep::Step1.bitmask(), 0xFE); // !(1 << 0) = 0xFE
        assert_eq!(ResetStep::Step2.bitmask(), 0xFD); // !(1 << 1) = 0xFD
        assert_eq!(ResetStep::Step3.bitmask(), 0xFB); // !(1 << 2) = 0xFB
        assert_eq!(ResetStep::Step4.bitmask(), 0xF7); // !(1 << 3) = 0xF7
        assert_eq!(ResetStep::Step5.bitmask(), 0xEF); // !(1 << 4) = 0xEF
        assert_eq!(ResetStep::FactoryReset.bitmask(), 0x80);
    }

    /// `apply_to` is the same mask applied to the byte on the card.
    #[test]
    fn test_apply_to_method() {
        let base_value = 0xFF;

        assert_eq!(ResetStep::Clear.apply_to(base_value), 0xFF);
        assert_eq!(ResetStep::Step1.apply_to(base_value), 0xFE);
        assert_eq!(ResetStep::Step2.apply_to(base_value), 0xFD);
        assert_eq!(ResetStep::Step3.apply_to(base_value), 0xFB);
        assert_eq!(ResetStep::Step4.apply_to(base_value), 0xF7);
        assert_eq!(ResetStep::Step5.apply_to(base_value), 0xEF);
        assert_eq!(ResetStep::FactoryReset.apply_to(base_value), 0x80);
    }

    /// Only the exact flag triggers the reset.
    #[test]
    fn test_factory_reset_condition() {
        assert!(!ResetStep::should_factory_reset(0xFF)); // Erased
        assert!(!ResetStep::should_factory_reset(0xFE)); // Step 1 only
        assert!(!ResetStep::should_factory_reset(0xE0)); // All steps, no timing validation
        assert!(!ResetStep::should_factory_reset(0xC0)); // Steps + Step 4 timing only
        assert!(ResetStep::should_factory_reset(0x80)); // All steps + all timing ✓
        assert!(!ResetStep::should_factory_reset(0x00)); // Invalidated
    }

    /// Timing validation clears the step's bit; invalidated values never read
    /// as validated.
    #[test]
    fn test_timing_validation_marking() {
        // Step 4 timing validation: 0xF0 -> 0xD0 (clear bit 5)
        let step4_complete = 0xF0;
        let step4_with_timing = ResetStep::mark_step_timing_validated(step4_complete, 4);
        assert_eq!(step4_with_timing, 0xD0);
        assert!(ResetStep::is_step_timing_validated(step4_with_timing, 4));

        // Step 5 timing validation: 0xE0 -> 0xA0 (clear bit 6)
        let step5_complete = 0xE0;
        let step5_with_timing = ResetStep::mark_step_timing_validated(step5_complete, 5);
        assert_eq!(step5_with_timing, 0xA0);
        assert!(ResetStep::is_step_timing_validated(step5_with_timing, 5));

        // Steps 1-3 don't need timing validation.
        assert!(ResetStep::is_step_timing_validated(0xFE, 1)); // Always validated
        assert!(ResetStep::is_step_timing_validated(0xFC, 2)); // Always validated
        assert!(ResetStep::is_step_timing_validated(0xF8, 3)); // Always validated

        // Invalidated values are never validated.
        assert!(!ResetStep::is_step_timing_validated(0x78, 4)); // Step 3 invalidated (bit 7 clear)
        assert!(!ResetStep::is_step_timing_validated(0x50, 5)); // Step 4 invalidated (bit 7 clear)
        assert!(!ResetStep::is_step_timing_validated(0x20, 1)); // Step 5 invalidated (bit 7 clear)
    }

    /// The whole live sequence, step by step, ends on the reset flag.
    #[test]
    fn test_complete_sequence() {
        let mut flash_value = 0xFF; // Start erased.

        // Step 1: 0xFF -> 0xFE
        flash_value = ResetStep::Step1.apply_to(flash_value);
        assert_eq!(flash_value, 0xFE);

        // Step 2: 0xFE -> 0xFC
        flash_value = ResetStep::Step2.apply_to(flash_value);
        assert_eq!(flash_value, 0xFC);

        // Step 3: 0xFC -> 0xF8
        flash_value = ResetStep::Step3.apply_to(flash_value);
        assert_eq!(flash_value, 0xF8);

        // Step 4: 0xF8 -> 0xF0, then timing validation -> 0xD0
        flash_value = ResetStep::Step4.apply_to(flash_value);
        assert_eq!(flash_value, 0xF0);
        flash_value = ResetStep::mark_step_timing_validated(flash_value, 4);
        assert_eq!(flash_value, 0xD0);

        // Step 5: applied to the timed value -> 0xC0, then timing -> 0x80
        flash_value = ResetStep::Step5.apply_to(flash_value);
        assert_eq!(flash_value, 0xC0);
        flash_value = ResetStep::mark_step_timing_validated(flash_value, 5);
        assert_eq!(flash_value, 0x80); // Factory reset ready!

        assert!(ResetStep::should_factory_reset(flash_value));
    }

    /// Invalidation clears bit 7; the erased value is not "invalidated".
    #[test]
    fn test_invalidation() {
        let valid_sequence = 0xF0; // Step 4 complete
        let invalidated = ResetStep::invalidate_flash_value(valid_sequence);
        assert_eq!(invalidated, 0x70); // Bit 7 cleared
        assert!(ResetStep::is_flash_value_invalidated(invalidated));
        assert!(!ResetStep::is_flash_value_invalidated(valid_sequence));

        assert!(!ResetStep::is_flash_value_invalidated(0xFF));
    }

    /// Flash value -> step.
    #[test]
    fn test_from_flash_value() {
        assert_eq!(ResetStep::from_flash_value(0xFF), ResetStep::Clear);
        assert_eq!(ResetStep::from_flash_value(0xFE), ResetStep::Step1);
        assert_eq!(ResetStep::from_flash_value(0xFC), ResetStep::Step2);
        assert_eq!(ResetStep::from_flash_value(0xF8), ResetStep::Step3);
        assert_eq!(ResetStep::from_flash_value(0xF0), ResetStep::Step4);
        assert_eq!(ResetStep::from_flash_value(0xE0), ResetStep::Step5);
        assert_eq!(ResetStep::from_flash_value(0x80), ResetStep::FactoryReset);

        // Invalidated sequences start over.
        assert_eq!(ResetStep::from_flash_value(0x70), ResetStep::Clear); // Invalidated Step 4
        assert_eq!(ResetStep::from_flash_value(0x60), ResetStep::Clear); // Invalidated Step 5
    }

    /// Step -> number, including the flag's pseudo-number 6.
    #[test]
    fn test_step_numbers() {
        assert_eq!(ResetStep::Clear.step_number(), 0);
        assert_eq!(ResetStep::Step1.step_number(), 1);
        assert_eq!(ResetStep::Step2.step_number(), 2);
        assert_eq!(ResetStep::Step3.step_number(), 3);
        assert_eq!(ResetStep::Step4.step_number(), 4);
        assert_eq!(ResetStep::Step5.step_number(), 5);
        assert_eq!(ResetStep::FactoryReset.step_number(), 6);
    }

    /// Number -> step, with everything unknown mapping to `Clear`.
    #[test]
    fn test_from_step_number_invalid_inputs() {
        assert_eq!(ResetStep::from_step_number(0), ResetStep::Clear);
        assert_eq!(ResetStep::from_step_number(1), ResetStep::Step1);
        assert_eq!(ResetStep::from_step_number(2), ResetStep::Step2);
        assert_eq!(ResetStep::from_step_number(3), ResetStep::Step3);
        assert_eq!(ResetStep::from_step_number(4), ResetStep::Step4);
        assert_eq!(ResetStep::from_step_number(5), ResetStep::Step5);
        assert_eq!(ResetStep::from_step_number(0x80), ResetStep::FactoryReset);

        assert_eq!(ResetStep::from_step_number(6), ResetStep::Clear); // Invalid step
        assert_eq!(ResetStep::from_step_number(7), ResetStep::Clear); // Invalid step
        assert_eq!(ResetStep::from_step_number(255), ResetStep::Clear); // Invalid step
        assert_eq!(ResetStep::from_step_number(100), ResetStep::Clear); // Invalid step
    }

    /// Timing windows per step.
    #[test]
    fn test_timing_windows() {
        assert_eq!(ResetStep::Clear.timing_window(), None);
        assert_eq!(ResetStep::Step1.timing_window(), Some((0, 3)));
        assert_eq!(ResetStep::Step2.timing_window(), Some((0, 3)));
        assert_eq!(ResetStep::Step3.timing_window(), Some((0, 3)));
        assert_eq!(ResetStep::Step4.timing_window(), Some((3, 30)));
        assert_eq!(ResetStep::Step5.timing_window(), Some((3, 30)));
        assert_eq!(ResetStep::FactoryReset.timing_window(), None);
    }

    /// Out-of-range steps never allocate a timing bit and never change a value.
    #[test]
    fn test_edge_cases() {
        assert_eq!(ResetStep::step_timing_bit(0), None);
        assert_eq!(ResetStep::step_timing_bit(6), None);
        assert_eq!(ResetStep::step_timing_bit(100), None);

        let unchanged = ResetStep::mark_step_timing_validated(0xFE, 1);
        assert_eq!(unchanged, 0xFE); // Step 1 doesn't need timing validation

        assert_eq!(ResetStep::factory_reset_value(), 0x80);
    }

    // ========================================================================
    // Flash I/O: sector scan, step writes, timing writes, invalidation
    // ========================================================================

    /// The scan stops at the last programmed slot.
    #[test]
    fn test_init_reset_sequence_find_location() {
        reset_fakes();
        // Three used slots, then erased ones.
        FakeFlash::write_slice(FLASH_ADR_RESET_CNT, &[0xFE; 3]);

        init_reset_sequence();

        // The last used slot wins because it is not invalidated.
        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), 2);
    }

    /// A full sector is erased during the scan and the cursor restarts at 0.
    #[test]
    fn test_init_reset_sequence_sector_full() {
        reset_fakes();
        FakeFlash::write_slice(FLASH_ADR_RESET_CNT, &[0xFE; SECTOR_SIZE_BYTES]);

        init_reset_sequence();

        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), 0);
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_RESET_CNT),
            1,
            "a full sector must be erased to make room"
        );
    }

    /// An invalidated slot is skipped: the sequence restarts in the next one.
    #[test]
    fn test_init_reset_sequence_skips_invalidated_slot() {
        reset_fakes();
        seed_slot(0, ResetStep::Step1.flash_value()); // 0xFE
        seed_slot(1, 0x7E); // Step 1 invalidated by a timing violation

        init_reset_sequence();

        assert_eq!(
            FACTORY_RESET_STATE.flash_address_index.get(),
            2,
            "the aborted sequence must not be resumed"
        );
        assert_eq!(
            FACTORY_RESET_STATE.consecutive_reset_count.get(),
            0,
            "the next slot is erased, so the sequence starts over"
        );
    }

    /// The byte on the card maps to the documented step number.
    #[test]
    fn test_read_reset_sequence_integration() {
        reset_fakes();
        FACTORY_RESET_STATE.flash_address_index.set(5);

        // Erased slot: no sequence in progress.
        assert_eq!(read_reset_sequence(), 0);

        // Step 3 value.
        set_slot(5, ResetStep::Step3.flash_value());
        assert_eq!(read_reset_sequence(), 3);

        // Invalidated sequence.
        set_slot(5, 0x78);
        assert_eq!(read_reset_sequence(), 0);

        // Factory reset ready.
        set_slot(5, 0x80);
        assert_eq!(read_reset_sequence(), 6); // FACTORY_RESET_FLAG
    }

    /// Step writes really land in the active slot.
    #[test]
    fn test_write_reset_sequence_integration() {
        reset_fakes();
        FACTORY_RESET_STATE.flash_address_index.set(10);

        write_reset_sequence(1);
        assert_eq!(
            read_slot(10),
            0xFE,
            "Step 1 must clear the first completion bit"
        );

        write_reset_sequence(2);
        assert_eq!(
            read_slot(10),
            0xFC,
            "Step 2 must clear the second completion bit"
        );

        // Re-writing the step already on the card programs nothing.
        write_reset_sequence(2);
        assert_eq!(read_slot(10), 0xFC, "an unchanged step is a no-op");
    }

    /// Completing the sequence on the last slot erases the sector.
    #[test]
    fn test_write_reset_sequence_sector_boundary() {
        reset_fakes();
        FACTORY_RESET_STATE.flash_address_index.set(4095);

        write_reset_sequence(0x80);

        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_RESET_CNT),
            1,
            "the completed sequence must wrap the sector"
        );
        assert_eq!(
            read_slot(4095),
            0xFF,
            "the sector erase must leave the page blank"
        );
        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), 0);
    }

    /// The timing write clears exactly the current step's timing bit.
    #[test]
    fn test_write_timing_validation_integration() {
        reset_fakes();
        // Step 4 complete, waiting for its minimum time.
        FACTORY_RESET_STATE.flash_address_index.set(15);
        FACTORY_RESET_STATE.consecutive_reset_count.set(4);
        seed_slot(15, ResetStep::Step4.flash_value()); // 0xF0

        write_timing_validation();
        assert_eq!(
            read_slot(15),
            0xD0,
            "Step 4 timing bit must be cleared in flash"
        );

        // Step 5 complete, waiting for its minimum time.
        FACTORY_RESET_STATE.consecutive_reset_count.set(5);
        write_timing_validation();
        assert_eq!(
            read_slot(15),
            0x90,
            "Step 5 timing bit must be cleared in flash"
        );
    }

    /// Invalidation writes the bit-7-cleared value and moves on one slot.
    #[test]
    fn test_clear_reset_sequence_integration() {
        reset_fakes();
        FACTORY_RESET_STATE.flash_address_index.set(20);
        seed_slot(20, ResetStep::Step3.flash_value()); // 0xF8

        let initial_index = FACTORY_RESET_STATE.flash_address_index.get();

        clear_reset_sequence();

        assert_eq!(
            read_slot(20),
            0x78,
            "invalidation must clear bit 7 in flash"
        );
        assert_eq!(
            FACTORY_RESET_STATE.flash_address_index.get(),
            initial_index + 1
        );
    }

    /// Invalidating the last slot erases the sector.
    #[test]
    fn test_clear_reset_sequence_sector_boundary() {
        reset_fakes();
        FACTORY_RESET_STATE.flash_address_index.set(4095);
        seed_slot(4095, ResetStep::Step2.flash_value());

        clear_reset_sequence();

        assert_eq!(FakeFlash::erase_count(FLASH_ADR_RESET_CNT), 1);
        assert_eq!(
            read_slot(4095),
            0xFF,
            "the sector erase must leave the page blank"
        );
        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), 0);
    }

    // ========================================================================
    // Boot sequence
    // ========================================================================

    /// A mid-sequence boot advances the step and primes the timing check.
    #[test]
    fn test_handle_boot_sequence_complete_flow() {
        reset_fakes();
        reset_state();

        // Slots 0, 1, 2 hold step 1; everything after is erased.
        FakeFlash::write_slice(FLASH_ADR_RESET_CNT, &[0xFE; 3]);

        assert_eq!(handle_boot_sequence(), ResetAction::None);

        // The Step 2 bit really landed in flash.
        assert_eq!(
            read_slot(2),
            0xFC,
            "Step 2 must be written at the active slot"
        );

        assert_eq!(FACTORY_RESET_STATE.consecutive_reset_count.get(), 2); // Step1 -> Step2
        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 1);
        assert_eq!(
            FACTORY_RESET_STATE.timing_check_timestamp.get(),
            POWER_CYCLE_TIMING[1].0 as u32
        );
        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), 2);
    }

    /// A boot that observes step 2 advances to step 3.
    #[test]
    fn test_handle_boot_sequence_step_progression() {
        reset_fakes();
        let current_step = 2;

        FACTORY_RESET_STATE.flash_address_index.set(5);
        FACTORY_RESET_STATE.consecutive_reset_count.set(0);
        FACTORY_RESET_STATE.clear_state.set(0);

        seed_slot(5, ResetStep::Step2.flash_value()); // Step 2 in progress at slot 5

        assert_eq!(handle_boot_sequence(), ResetAction::None);

        assert_eq!(
            read_slot(5),
            0xF8,
            "Step 3 must be written at the active slot"
        );

        assert_eq!(
            FACTORY_RESET_STATE.consecutive_reset_count.get(),
            current_step + 1
        );
        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 1);
        let timing_idx = (current_step + 1) - 1; // 0-indexed array
        assert_eq!(
            FACTORY_RESET_STATE.timing_check_timestamp.get(),
            POWER_CYCLE_TIMING[timing_idx as usize].0 as u32
        );
    }

    /// A sequence that reached the flag reports the reset and changes nothing.
    ///
    /// The interrupt disable, sector erase, LED blink and reboot that the old
    /// `factory_reset_handle` performed are now the caller's job (`user_init`
    /// in `app.rs`), and are asserted there. Here the contract is exactly
    /// "return the action, write nothing".
    #[test]
    fn test_handle_boot_sequence_completed_sequence() {
        reset_fakes();
        reset_state();

        seed_slot(10, 0x80); // FACTORY_RESET_FLAG at slot 10

        assert_eq!(
            handle_boot_sequence(),
            ResetAction::FactoryResetAndReboot,
            "the flag must request the factory reset"
        );

        assert_eq!(read_slot(10), 0x80, "the flag itself must not be rewritten");
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_RESET_CNT),
            0,
            "erasing belongs to the caller"
        );
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_PAIRING),
            0,
            "erasing belongs to the caller"
        );
        assert!(
            fake_clock::sleeps().is_empty(),
            "the LED blink belongs to the caller"
        );
        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 0);
    }

    /// Booting on an erased sector starts the sequence at step 1.
    #[test]
    fn test_handle_boot_sequence_clear_case() {
        reset_fakes();
        reset_state();

        assert_eq!(handle_boot_sequence(), ResetAction::None);

        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), 0);
        assert_eq!(FACTORY_RESET_STATE.consecutive_reset_count.get(), 1);
        assert!(FACTORY_RESET_STATE.clear_state.get() > 0); // Timing started
        assert_eq!(read_slot(0), 0xFE, "Clear case must write Step 1 to flash");
    }

    // ========================================================================
    // Timing check
    // ========================================================================

    /// An invalid step number aborts the sequence and moves to the next slot.
    #[test]
    fn test_check_timing_step_invalid_state() {
        reset_fakes();
        FACTORY_RESET_STATE.consecutive_reset_count.set(0); // Invalid
        FACTORY_RESET_STATE.clear_state.set(1); // But active timing check
        FACTORY_RESET_STATE.flash_address_index.set(15);

        seed_slot(15, ResetStep::Step3.flash_value());

        assert_eq!(check_timing_step(), ResetAction::None);

        assert_eq!(
            read_slot(15),
            0x78,
            "the invalidated step must land in flash"
        );
        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 0);
        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), 16); // Incremented
    }

    /// Nothing is programmed while the minimum window is still pending.
    #[test]
    fn test_check_timing_step_min_time_not_elapsed() {
        // State 1 for step 4, which requires timing validation.
        FACTORY_RESET_STATE.consecutive_reset_count.set(4);
        FACTORY_RESET_STATE.clear_state.set(1);
        FACTORY_RESET_STATE.flash_address_index.set(25);
        let sentinel_timestamp = 1234;
        FACTORY_RESET_STATE
            .timing_check_timestamp
            .set(sentinel_timestamp);

        let min_time_us = (POWER_CYCLE_TIMING[3].0 as u32) * 1_000_000;

        reset_fakes();
        // The clock starts at tick 0, so the minimum window has not elapsed.
        fake_clock::set_ticks(0);
        assert!(
            !clock_time_exceed(0, min_time_us),
            "precondition: the minimum window must not have elapsed at tick 0"
        );

        assert_eq!(check_timing_step(), ResetAction::None);

        // Nothing may be programmed anywhere in the reset-count sector while the
        // window is pending: no timing bit and no erase.
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_RESET_CNT),
            0,
            "the pending window must not erase the reset-count sector"
        );
        let mut sector = [0u8; SECTOR_SIZE_BYTES];
        FakeFlash::read_slice(FLASH_ADR_RESET_CNT, &mut sector);
        assert_eq!(
            sector.iter().position(|&b| b != 0xFF),
            None,
            "no byte of the reset-count sector may be programmed too early"
        );
        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 1);
        assert_eq!(
            FACTORY_RESET_STATE.timing_check_timestamp.get(),
            sentinel_timestamp
        );
    }

    /// Minimum time elapsed: state 2 plus the step's timing bit.
    #[test]
    fn test_check_timing_step_timing_state_machine() {
        FACTORY_RESET_STATE.consecutive_reset_count.set(4); // Step 4
        FACTORY_RESET_STATE.clear_state.set(1);
        FACTORY_RESET_STATE
            .timing_check_timestamp
            .set(POWER_CYCLE_TIMING[3].0 as u32);
        FACTORY_RESET_STATE.flash_address_index.set(25);

        // The minimum window has elapsed: park the fake tick at the top of the
        // 32-bit range, far past any configured microsecond span.
        reset_fakes();
        fake_clock::set_ticks(u32::MAX as u64);
        seed_slot(25, ResetStep::Step4.flash_value());

        assert_eq!(check_timing_step(), ResetAction::None);

        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 2);
        assert_eq!(
            FACTORY_RESET_STATE.timing_check_timestamp.get(),
            POWER_CYCLE_TIMING[3].1 as u32
        );
        assert_eq!(
            read_slot(25),
            0xD0,
            "Step 4 timing bit must be cleared in flash"
        );
    }

    /// Maximum time exceeded: the sequence is invalidated.
    #[test]
    fn test_check_timing_step_max_time_exceeded() {
        FACTORY_RESET_STATE.consecutive_reset_count.set(4);
        FACTORY_RESET_STATE.clear_state.set(2);
        FACTORY_RESET_STATE
            .timing_check_timestamp
            .set(POWER_CYCLE_TIMING[3].1 as u32);
        FACTORY_RESET_STATE.flash_address_index.set(20);

        reset_fakes();
        fake_clock::set_ticks(u32::MAX as u64);
        seed_slot(20, ResetStep::Step4.flash_value());

        assert_eq!(check_timing_step(), ResetAction::None);

        assert_eq!(
            read_slot(20),
            0x70,
            "the invalidated step must land in flash"
        );
        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 0);
    }

    /// No timing check active: an immediate, side-effect-free return.
    #[test]
    fn test_check_timing_step_early_return_when_inactive() {
        FACTORY_RESET_STATE.clear_state.set(0); // No timing check active
        FACTORY_RESET_STATE.consecutive_reset_count.set(1);
        FACTORY_RESET_STATE.flash_address_index.set(5);

        assert_eq!(check_timing_step(), ResetAction::None);

        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 0);
        assert_eq!(FACTORY_RESET_STATE.consecutive_reset_count.get(), 1);
        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), 5);
    }

    /// An unknown clear state aborts the sequence.
    #[test]
    fn test_check_timing_step_invalid_clear_state() {
        reset_fakes();
        FACTORY_RESET_STATE.clear_state.set(3); // Not 0, 1 or 2
        FACTORY_RESET_STATE.consecutive_reset_count.set(2);
        FACTORY_RESET_STATE.flash_address_index.set(10);

        seed_slot(10, ResetStep::Step2.flash_value());

        assert_eq!(check_timing_step(), ResetAction::None);

        assert_eq!(
            read_slot(10),
            0x7C,
            "the invalidated step must land in flash"
        );
        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 0);
    }

    /// Five step writes plus both timing writes reach the flag in flash.
    #[test]
    fn test_complete_sequence_with_timing_validation() {
        reset_fakes();
        FACTORY_RESET_STATE.flash_address_index.set(30);

        for step in 1..=5 {
            write_reset_sequence(step);
        }

        // Now validate the two 3-30 s windows.
        FACTORY_RESET_STATE.consecutive_reset_count.set(4);
        write_timing_validation(); // 0xE0 -> 0xC0

        FACTORY_RESET_STATE.consecutive_reset_count.set(5);
        write_timing_validation(); // 0xC0 -> 0x80

        assert_eq!(
            read_slot(30),
            ResetStep::FactoryReset.flash_value(),
            "the full sequence must be programmed into flash"
        );
    }

    /// Completing every step without the timing windows does not reset.
    #[test]
    fn test_rapid_reset_attack_prevention_integration() {
        let all_steps_complete = ResetStep::Step5.flash_value(); // 0xE0

        assert!(!ResetStep::should_factory_reset(all_steps_complete));

        // Step 4 timing alone is not enough either.
        let with_step4_timing = ResetStep::mark_step_timing_validated(all_steps_complete, 4);
        assert_eq!(with_step4_timing, 0xC0);
        assert!(!ResetStep::should_factory_reset(with_step4_timing));

        // Only with both timing validations does it complete.
        let with_both_timing = ResetStep::mark_step_timing_validated(with_step4_timing, 5);
        assert_eq!(with_both_timing, 0x80);
        assert!(ResetStep::should_factory_reset(with_both_timing));
    }

    // ========================================================================
    // Kick-out policy and configuration erase
    // ========================================================================

    /// The configuration erase covers the pairing sector and the reset counter.
    #[test]
    fn test_erase_config_sectors_erases_configuration() {
        reset_fakes();
        FACTORY_RESET_STATE.flash_address_index.set(0);

        erase_config_sectors();

        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_RESET_CNT),
            1,
            "the reset counter sector must be erased last"
        );
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_PAIRING),
            1,
            "the pairing sector must be erased too"
        );
        assert_eq!(
            FakeFlash::erase_count(CFG_SECTOR_ADR_MAC_CODE),
            0,
            "the MAC sector is preserved"
        );
    }

    /// `KickoutReason` round-trips the wire values and rejects the rest.
    #[test]
    fn test_kickout_reason_try_from() {
        assert_eq!(KickoutReason::try_from(0), Ok(KickoutReason::OutOfMesh));
        assert_eq!(KickoutReason::try_from(1), Ok(KickoutReason::DefaultName));
        assert_eq!(KickoutReason::try_from(2), Ok(KickoutReason::ModeMax));

        assert_eq!(KickoutReason::try_from(3), Err(()));
        assert_eq!(KickoutReason::try_from(255), Err(()));
        assert_eq!(KickoutReason::try_from(u32::MAX), Err(()));
    }

    /// Only `OutOfMesh` needs the credentials re-written.
    #[test]
    fn test_plan_kick_out_reasons() {
        assert_eq!(
            plan_kick_out(KickoutReason::OutOfMesh),
            KickoutAction::WriteOutOfMeshCredentialsAndReboot
        );
        assert_eq!(
            plan_kick_out(KickoutReason::DefaultName),
            KickoutAction::ErasePairingAndReboot
        );
        assert_eq!(
            plan_kick_out(KickoutReason::ModeMax),
            KickoutAction::ErasePairingAndReboot
        );

        // A plain erase leaves the pairing sector blank: no credentials are
        // written for a non-OutOfMesh kick-out.
        reset_fakes();
        FACTORY_RESET_STATE.flash_address_index.set(0);
        assert_eq!(
            plan_kick_out(KickoutReason::DefaultName),
            KickoutAction::ErasePairingAndReboot
        );
        erase_config_sectors();
        assert_eq!(FakeFlash::erase_count(FLASH_ADR_PAIRING), 1);
        let mut flags = [0u8; 16];
        FakeFlash::read_slice(FLASH_ADR_PAIRING, &mut flags);
        assert_eq!(
            flags, [0xFF; 16],
            "no pairing data may be written for a plain erase"
        );
    }

    /// Boot defaults of the state struct.
    #[test]
    fn test_factory_reset_state_new_defaults() {
        let state = FactoryResetState::new();

        assert_eq!(state.flash_address_index.load(Ordering::Relaxed), 0);
        assert_eq!(state.consecutive_reset_count.load(Ordering::Relaxed), 0);
        assert_eq!(state.clear_state.load(Ordering::Relaxed), 3);
        assert_eq!(state.timing_check_timestamp.load(Ordering::Relaxed), 0);
    }
}
