//! # Factory Reset Algorithm for TLSR8266 Mesh Devices
//!
//! This module implements a flash-optimized factory reset detection algorithm that uses
//! power cycle timing patterns to trigger device reset while minimizing flash wear.
//!
//! ## Algorithm Overview
//!
//! The factory reset sequence requires the user to power cycle the device following
//! specific timing patterns. The device tracks progress through a 5-step sequence,
//! and if all steps are completed correctly, performs a factory reset.
//!
//! ### Power Cycle Sequence
//!
//! 1. **Steps 1-3**: Quick power cycles (0-3 seconds on)
//! 2. **Steps 4-5**: Longer power cycles (3-30 seconds on)
//!
//! Each step has strict timing requirements:
//! - **Minimum time**: Device must stay on for at least this long
//! - **Maximum time**: Device must be turned off before this time elapses
//!
//! ### Flash Storage Strategy
//!
//! The algorithm uses a novel bit-clearing approach optimized for flash memory:
//!
//! ```text
//! Flash Value    Step         Description                 Bit Pattern
//! -----------    ----         -----------                 -----------
//! 0xFF          Clear        Erased state                11111111
//! 0xFE          Step1        Clear bit 0                 11111110
//! 0xFC          Step2        Clear bit 1                 11111100
//! 0xF8          Step3        Clear bit 2                 11111000
//! 0xF0          Step4        Clear bit 3                 11110000
//! 0xE0          Step5        Clear bit 4                 11100000
//! 0x60          FactoryReset Clear bit 7                 01100000
//! 0xXX & 0xBF   Invalidated  Clear bit 6 (invalidation)  XX0XXXXX
//! ```
//!
//! ## Flash Optimization Benefits
//!
//! - **Minimal bit flips**: Only 6 bit flips total for complete sequence
//! - **Single location**: Entire sequence uses one flash byte until completion
//! - **Fast invalidation**: Single bit flip to abort sequence
//! - **No erase cycles**: Until sector is full (4096 sequences per 4KB sector)
//!
//! ## Example Scenarios
//!
//! ### Successful Factory Reset
//!
//! ```text
//! 1. Device powers on → Flash: 0xFF (erased)
//! 2. Start sequence → Write 0xFE (step 1)
//! 3. Power cycle within 0-3s → Write 0xFC (step 2)
//! 4. Power cycle within 0-3s → Write 0xF8 (step 3)
//! 5. Power cycle within 0-3s → Write 0xF0 (step 4)
//! 6. Stay on 3-30s, cycle → Write 0xE0 (step 5)
//! 7. Stay on 3-30s, cycle → Write 0x60 (factory reset flag)
//! 8. Next boot → Detect 0x60 → FACTORY RESET EXECUTED
//! ```
//!
//! ### Sequence Invalidation (Timing Violation)
//!
//! ```text
//! 1. Device at step 3 → Flash: 0xF8
//! 2. User leaves device on > 3 seconds → TIMING VIOLATION
//! 3. Algorithm invalidates → Write 0xB8 (0xF8 & 0xBF)
//! 4. Move to next flash location
//! 5. Next sequence starts fresh at new location
//! ```
//!
//! ### Flash Sector Management
//!
//! ```text
//! Address: FLASH_ADR_RESET_CNT + FACTORY_RESET_STATE.flash_address_index.get() + index
//! 
//! [0x0000] 0xB8  ← Invalidated sequence (step 3 failed)
//! [0x0001] 0xBC  ← Invalidated sequence (step 2 failed)  
//! [0x0002] 0xE0  ← Valid step 5 (current position)
//! [0x0003] 0xFF  ← Erased
//! [0x0004] 0xFF  ← Erased
//! ...
//! [4095] 0x60    ← Factory reset flag (triggers sector erase)
//! ```
//!
//! When the sector is full (index ≥ 4096), the entire sector is erased
//! and the index resets to 0, providing fresh space for new sequences.
//!
//! ## State Machine
//!
//! The algorithm operates as a state machine with these states:
//!
//! - **FACTORY_RESET_STATE.clear_state = 0**: No timing check active
//! - **FACTORY_RESET_STATE.clear_state = 1**: Checking minimum time requirement
//! - **FACTORY_RESET_STATE.clear_state = 2**: Checking maximum time limit
//!
//! ## Error Handling
//!
//! - **Timing violations**: Sequence invalidated, moved to next location
//! - **Power loss**: Sequence preserved in flash, continues on next boot
//! - **Corrupted flash**: Invalid values treated as clear state
//! - **Sector full**: Automatic sector erase and index reset
//!
//! ## Thread Safety
//!
//! Flash operations use critical sections to prevent interruption during
//! multi-byte writes and ensure atomic updates to the sequence state.

use core::cmp::min;
use core::convert::TryFrom;

use crate::app;
use crate::BIT;
use crate::uprintln;
use crate::config::{FLASH_ADR_PAIRING, FLASH_ADR_RESET_CNT, MESH_PWD, OUT_OF_MESH, PAIR_VALID_FLAG};
use crate::sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};
use crate::sdk::mcu::clock::clock_time_exceed;
use crate::sdk::mcu::crypto::encode_password;
use critical_section;
use crate::sdk::pm::light_sw_reboot;
use crate::state::{*};
use crate::sdk::mcu::irq_i::irq_disable;

/// Timing sequence for factory reset detection (in seconds)
/// Format: [min_time, max_time] pairs for each power cycle step
/// 
/// Each pair defines a power cycle timing window:
/// - First number: Minimum seconds device must be powered on
/// - Second number: Maximum seconds device must be powered on
/// 
/// For example:
/// - [0, 3]: Device must be turned off within 3 seconds of turning on
/// - [3, 30]: Device must be on for at least 3 seconds but not more than 30 seconds
/// 
/// If all timing windows are correctly followed in sequence, factory reset is triggered.
/// If any timing window is not followed correctly, the reset sequence is aborted.
const POWER_CYCLE_TIMING: [(u8, u8); 5] = [
    (0, 3),   // Step 1: Device must be turned off within 3 seconds of turning on
    (0, 3),   // Step 2: Device must be turned off within 3 seconds of turning on
    (0, 3),   // Step 3: Device must be turned off within 3 seconds of turning on
    (3, 30),  // Step 4: Device must be on for 3-30 seconds before turning off
    (3, 30),  // Step 5: Device must be on for 3-30 seconds before turning off
];

/// Compile-time validation of power cycle timing configuration
const _: () = {
    let len = POWER_CYCLE_TIMING.len();
    assert!(len >= 2, "POWER_CYCLE_TIMING must have at least 2 entries");
    assert!(len <= 5, "POWER_CYCLE_TIMING must have at most 5 entries");
    // Ensure we have enough bits for all steps (need 1 bit per step + 1 timing bit per step with min_time > 0)
    let mut timing_bits_needed = 0;
    let mut i = 0;
    while i < len {
        if POWER_CYCLE_TIMING[i].0 > 0 {
            timing_bits_needed += 1;
        }
        i += 1;
    }
    assert!(len + timing_bits_needed <= 7, "Not enough bits for all steps and timing validation");
};

/// Number of steps in the power cycle sequence (derived from timing array)
const POWER_CYCLE_COUNT: u8 = POWER_CYCLE_TIMING.len() as u8;

/// Flash-optimized reset sequence steps using progressive bit clearing
/// 
/// # Dynamic Bit Allocation Strategy
/// 
/// - Bit 7: Invalid bit (0 = invalidated sequence)
/// - Bits 6-0: Dynamically allocated based on POWER_CYCLE_TIMING array
///   - Each step gets one "completion" bit
///   - Each step with min_time > 0 gets one "timing validation" bit
/// 
/// Factory reset triggers when all required bits are cleared and bit 7 is set
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetStep {
    /// No sequence in progress (flash erased or cleared)
    Clear,
    /// Step 1: 0xFF → 0xFE (clear bit 0)
    Step1,
    /// Step 2: 0xFE → 0xFC (clear bit 1)
    Step2,
    /// Step 3: 0xFC → 0xF8 (clear bit 2)
    Step3,
    /// Step 4: 0xF8 → 0xF0 (clear bit 3)
    Step4,
    /// Step 5: 0xF0 → 0xE0 (clear bit 4, but bit 5 tracks completion)
    Step5,
    /// Factory reset ready: All steps + timing complete (0x80)
    FactoryReset,
}

impl ResetStep {
    /// Invalidation bitmask: clears bit 7 (0xxxxxxx)
    const INVALIDATION_MASK: u8 = 0b01111111;
    
    /// Get the bit position for step completion (step 1 = bit 0, step 2 = bit 1, etc.)
    const fn step_completion_bit(step_number: u8) -> u8 {
        step_number - 1
    }
    
    /// Get the bit position for timing validation for a given step
    /// Timing bits are allocated after all step completion bits
    const fn step_timing_bit(step_number: u8) -> Option<u8> {
        // Only steps with min_time > 0 get timing validation bits
        if step_number == 0 || step_number > POWER_CYCLE_COUNT {
            return None;
        }
        
        let timing = POWER_CYCLE_TIMING[step_number as usize - 1];
        if timing.0 == 0 {
            return None; // No minimum time requirement
        }
        
        // Count how many steps before this one have timing requirements
        let mut timing_bit_offset = POWER_CYCLE_COUNT; // Start after all completion bits
        let mut i = 0;
        while i < step_number - 1 {
            if POWER_CYCLE_TIMING[i as usize].0 > 0 {
                timing_bit_offset += 1;
            }
            i += 1;
        }
        
        Some(timing_bit_offset as u8)
    }
    
    /// Calculate the factory reset trigger value (all required bits cleared, bit 7 set)
    const fn factory_reset_value() -> u8 {
        let mut value = 0x80; // Start with bit 7 set (not invalid)
        
        // Clear all step completion bits (bits 0 to POWER_CYCLE_COUNT-1)
        // These bits are already 0 in our starting value of 0x80
        
        // Clear all timing validation bits for steps that require them
        // These bits are already 0 in our starting value of 0x80
        
        value
    }
    
    /// Get the bitmask that clears the appropriate bit for this step
    const fn bitmask(self) -> u8 {
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
    
    /// Get the expected flash value after applying this step's completion bitmask
    /// This calculates what the flash should look like when this step is marked complete
    pub const fn flash_value(self) -> u8 {
        match self {
            ResetStep::Clear => 0xFF,
            ResetStep::Step1 => 0xFF & !(1 << Self::step_completion_bit(1)), // Clear bit 0
            ResetStep::Step2 => 0xFF & !(1 << Self::step_completion_bit(1)) & !(1 << Self::step_completion_bit(2)), // Clear bits 0,1
            ResetStep::Step3 => 0xFF & !(1 << Self::step_completion_bit(1)) & !(1 << Self::step_completion_bit(2)) & !(1 << Self::step_completion_bit(3)), // Clear bits 0,1,2
            ResetStep::Step4 => 0xFF & !(1 << Self::step_completion_bit(1)) & !(1 << Self::step_completion_bit(2)) & !(1 << Self::step_completion_bit(3)) & !(1 << Self::step_completion_bit(4)), // Clear bits 0,1,2,3
            ResetStep::Step5 => 0xFF & !(1 << Self::step_completion_bit(1)) & !(1 << Self::step_completion_bit(2)) & !(1 << Self::step_completion_bit(3)) & !(1 << Self::step_completion_bit(4)) & !(1 << Self::step_completion_bit(5)), // Clear bits 0,1,2,3,4
            ResetStep::FactoryReset => Self::factory_reset_value(),
        }
    }
    
    /// Apply this step's bitmask to a flash value
    pub const fn apply_to(self, value: u8) -> u8 {
        value & self.bitmask()
    }
    
    /// Check if this step completes the power cycle sequence
    pub const fn is_sequence_complete(self) -> bool {
        matches!(self, ResetStep::FactoryReset)
    }
    
    /// Get timing window for this step
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
    
    /// Create an invalidated version of a flash value (clear bit 6)
    pub const fn invalidate_flash_value(value: u8) -> u8 {
        value & Self::INVALIDATION_MASK
    }
    
    /// Check if a flash value represents an invalidated sequence
    pub const fn is_flash_value_invalidated(value: u8) -> bool {
        // Check if bit 7 is cleared AND it's not erased/clear state
        (value & 0b10000000) == 0 && value != 0xFF
    }
    
    /// Mark a flash value as having satisfied minimum timing requirements for a given step
    pub const fn mark_step_timing_validated(value: u8, step_number: u8) -> u8 {
        if let Some(timing_bit) = Self::step_timing_bit(step_number) {
            value & !(1 << timing_bit)
        } else {
            value // No timing validation needed for this step
        }
    }
    
    /// Check if timing validation is complete for a given step
    pub const fn is_step_timing_validated(value: u8, step_number: u8) -> bool {
        if Self::is_flash_value_invalidated(value) {
            return false;
        }
        
        if let Some(timing_bit) = Self::step_timing_bit(step_number) {
            (value & (1 << timing_bit)) == 0
        } else {
            true // No timing validation needed, always considered validated
        }
    }
    
    /// Check if factory reset should be triggered
    pub const fn should_factory_reset(value: u8) -> bool {
        value == Self::factory_reset_value()
    }
    
    /// Check if a step requires timing validation (has min_time > 0)
    pub const fn step_requires_timing(step_number: u8) -> bool {
        if step_number == 0 || step_number > POWER_CYCLE_COUNT {
            return false;
        }
        POWER_CYCLE_TIMING[step_number as usize - 1].0 > 0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetStepError {
    InvalidStepNumber,
}

impl ResetStep {
    /// Convert flash value back to reset step
    pub const fn from_flash_value(value: u8) -> Self {
        // First check if sequence is invalidated
        if Self::is_flash_value_invalidated(value) {
            return ResetStep::Clear;
        }
        
        // Check for factory reset condition: 0x80 (all steps + timing complete)
        if value == 0x80 {
            return ResetStep::FactoryReset;
        }
        
        // Determine the highest completed step based on which completion bits are cleared
        // For 5-step config: bits 0,1,2,3,4 track completion of steps 1,2,3,4,5
        // When a step is complete, its bit is cleared
        if (value & 0x01) != 0 {
            ResetStep::Clear  // Step 1 not done yet
        } else if (value & 0x02) != 0 {
            ResetStep::Step1  // Step 1 done, Step 2 not done
        } else if (value & 0x04) != 0 {
            ResetStep::Step2  // Steps 1-2 done, Step 3 not done
        } else if (value & 0x08) != 0 {
            ResetStep::Step3  // Steps 1-3 done, Step 4 not done
        } else if (value & 0x10) != 0 {
            ResetStep::Step4  // Steps 1-4 done, Step 5 not done
        } else {
            // All step completion bits clear, check if timing requirements met
            if value == 0x80 {
                ResetStep::FactoryReset
            } else {
                ResetStep::Step5  // Step 5 completed
            }
        }
    }
    
    /// Get the step number as a u8 (for legacy compatibility)
    pub const fn step_number(self) -> u8 {
        match self {
            ResetStep::Clear => 0,
            ResetStep::Step1 => 1,
            ResetStep::Step2 => 2,
            ResetStep::Step3 => 3,
            ResetStep::Step4 => 4,
            ResetStep::Step5 => 5,
            ResetStep::FactoryReset => 6, // Special flag for factory reset ready
        }
    }
    
    /// Create a ResetStep from a step number (for legacy compatibility)
    pub const fn from_step_number(step: u8) -> Self {
        match step {
            0 => ResetStep::Clear,
            1 => ResetStep::Step1,
            2 => ResetStep::Step2,
            3 => ResetStep::Step3,
            4 => ResetStep::Step4,
            5 => ResetStep::Step5,
            0x80 => ResetStep::FactoryReset,
            _ => ResetStep::Clear,
        }
    }
}

/// Legacy constants for backward compatibility
pub const RESET_SEQUENCE_CLEAR: u8 = 0;
pub const FACTORY_RESET_FLAG: u8 = 0x80;

/// Maximum address in flash memory
pub const FLASH_ADR_PAR_MAX: u32 = 0x80000;

/// Base address for MAC and configuration data in 512KB flash
pub const CFG_ADR_MAC_512K_FLASH: u32 = 0x76000;

/// Sector address for MAC code configuration
pub const CFG_SECTOR_ADR_MAC_CODE: u32 = CFG_ADR_MAC_512K_FLASH;

/// Write the current step in the reset sequence to flash memory using bitmask approach
///
/// # Parameters
///
/// * `step` - The current step in the reset sequence (1-5) or FACTORY_RESET_FLAG
///
/// # Flash-Optimized Algorithm
///
/// This function uses a bitmask approach to minimize bit flips in flash:
/// - Flash starts erased (0xFF - all bits set to 1)
/// - Each step clears exactly one bit (1→0 transition)
/// - Sequence: 0xFF → 0xFE → 0xFC → 0xF8 → 0xF0 → 0xE0 → 0x60
/// - Only actual bit flips count as "writes" on this MCU
/// - Maximum 6 bit flips total vs 24+ bit flips in the old approach
///
/// # Notes
///
/// * Uses the SAME flash location for the entire sequence
/// * No need to increment address until sector cleanup
/// * Much more flash-friendly than writing full bytes
#[cfg_attr(test, mry::mry)]
fn write_reset_sequence(step: u8) {
    let reset_step = ResetStep::from_step_number(step);
    
    // Get current value at the reset counter location
    let current_addr = FLASH_ADR_RESET_CNT + FACTORY_RESET_STATE.flash_address_index.get();
    let current_value = read_flash_byte(current_addr);
    
    // Calculate new value by applying the step's bitmask
    let mut new_value = reset_step.apply_to(current_value);
    
    // IMPORTANT: When writing a new step, ensure timing validation bit is set to 1 (unvalidated)
    // This prevents inheriting validation state from previous steps
    if reset_step != ResetStep::Clear && reset_step != ResetStep::FactoryReset {
        // Set bit 5 to 1 (unvalidated) for new steps
        // Note: We can only set bits during step transitions by using the step's base flash value
        new_value = reset_step.flash_value(); // Use the step's base value with bit 5 = 1
    }
    
    
    // Only write if we're actually changing bits (1→0 transitions)
    if new_value != current_value {
        let data = [new_value];
        flash_write_page(current_addr, 1, data.as_ptr());
    }
    
    // Check if this sequence is complete (reset flag written)
    if reset_step.is_sequence_complete() {
        // Move to next location for the next factory reset sequence
        FACTORY_RESET_STATE.flash_address_index.inc();
        
        // Check if we're at the end of the sector
        if FACTORY_RESET_STATE.flash_address_index.get() >= 4096 {
            flash_erase_sector(FLASH_ADR_RESET_CNT);
            FACTORY_RESET_STATE.flash_address_index.set(0);
        }
    }
}

/// Mark the current step as having satisfied minimum timing requirements
///
/// This function clears the appropriate timing bit to indicate that
/// the minimum time requirement for the current step has been met.
///
/// # Flash Bit Usage
/// - Dynamically calculated based on POWER_CYCLE_TIMING array
/// - Only steps with min_time > 0 get timing validation bits
#[cfg_attr(test, mry::mry)]
fn write_timing_validation() {
    let current_addr = FLASH_ADR_RESET_CNT + FACTORY_RESET_STATE.flash_address_index.get();
    let current_value = read_flash_byte(current_addr);
    let current_step_number = FACTORY_RESET_STATE.consecutive_reset_count.get();
    
    
    // Check if this step requires timing validation
    if !ResetStep::step_requires_timing(current_step_number) {
        return;
    }
    
    let validated_value = ResetStep::mark_step_timing_validated(current_value, current_step_number);
    
    // Only write if we're actually changing bits (1→0 transitions)
    if validated_value != current_value {
        let data = [validated_value];
        flash_write_page(current_addr, 1, data.as_ptr());
        
        // Check if this completes the factory reset sequence
        if ResetStep::should_factory_reset(validated_value) {
        }
    } else {
    }
}

/// Read the current step in the reset sequence from flash memory
///
/// # Returns
///
/// * The current step in the reset sequence
///
/// # Notes
///
/// * Returns RESET_SEQUENCE_CLEAR (0) if no sequence is in progress
/// * Returns FACTORY_RESET_FLAG (0x80) if factory reset should be performed
#[cfg_attr(test, mry::mry)]
fn read_flash_byte(addr: u32) -> u8 {
    unsafe {
        // Cast the address to a raw pointer and perform a volatile read
        let flash_ptr = addr as *const u8;
        core::ptr::read_volatile(flash_ptr)
    }
}

#[cfg_attr(test, mry::mry)]
fn read_reset_sequence() -> u8 {
    // Read the current value from flash at the current sequence location
    let current_addr = FLASH_ADR_RESET_CNT + FACTORY_RESET_STATE.flash_address_index.get();
    let flash_value = read_flash_byte(current_addr);
    
    
    // Convert flash bitmask value back to step number
    let reset_step = ResetStep::from_flash_value(flash_value);
    let step_number = reset_step.step_number();
    
    
    step_number
}

/// Clear the reset sequence by setting invalidation bit and moving to next location
///
/// # Single-Bit Invalidation Algorithm
///
/// Instead of writing 0x00 (multiple bit flips), we use bit 6 as an invalidation flag:
/// - Clear bit 6 (1→0) to mark sequence as invalid
/// - Move to next flash location for fresh sequence
/// - Only 1 bit flip required for invalidation!
///
/// Examples:
/// - 0xF8 (step 3) → 0xB8 (invalidated step 3) - 1 bit flip
/// - 0xFE (step 1) → 0xBE (invalidated step 1) - 1 bit flip
///
/// This aborts any in-progress factory reset detection sequence
#[cfg_attr(test, mry::mry)]
fn clear_reset_sequence() {
    // Invalidate current sequence by clearing bit 7 (invalidation flag)
    let current_addr = FLASH_ADR_RESET_CNT + FACTORY_RESET_STATE.flash_address_index.get();
    let current_value = read_flash_byte(current_addr);
    
    
    // Only invalidate if there's actually a sequence to invalidate
    if current_value != 0xFF {
        let invalidated_value = ResetStep::invalidate_flash_value(current_value);
        let invalidation_data = [invalidated_value];
        flash_write_page(current_addr, 1, invalidation_data.as_ptr());
    }
    
    // Move to next location for fresh sequence
    FACTORY_RESET_STATE.flash_address_index.inc();
    
    
    // Check if we're at the end of the sector
    if FACTORY_RESET_STATE.flash_address_index.get() >= 4096 {
        flash_erase_sector(FLASH_ADR_RESET_CNT);
        FACTORY_RESET_STATE.flash_address_index.set(0);
    }
}

/// Initialize the reset sequence tracking
///
/// # Single-Bit Invalidation Algorithm
///
/// Scans through flash to find the latest VALID reset sequence entry:
/// - 0xFF = erased state (valid for new sequence)
/// - Bit 6 cleared = invalidated sequence (failed sequence, skip)
/// - Valid bitmask patterns = active sequences
///
/// This ensures we never continue from a failed/invalidated sequence.
///
/// # Notes
///
/// * Scans flash memory to find the latest reset sequence step
/// * Sets up FACTORY_RESET_STATE.flash_address_index for subsequent operations
/// * Skips invalidated sequences (bit 6 = 0)
#[cfg_attr(test, mry::mry)]
fn init_reset_sequence() {
    // Start scanning from the beginning of the sector
    FACTORY_RESET_STATE.flash_address_index.set(0);
    
    
    // Scan through flash sector to find the latest reset sequence entry
    let mut latest_idx = 0;
    let mut latest_value = 0xFF; // Erased state
    let mut found_any_sequence = false;
    
    // Scan through the entire 4KB sector
    for i in 0..4096 {
        let value = read_flash_byte(FLASH_ADR_RESET_CNT + i);
        
        // Skip completely erased locations
        if value != 0xFF {
            
            // This is the latest used position regardless of whether it's valid or invalidated
            latest_idx = i;
            latest_value = value;
            found_any_sequence = true;
        }
    }
    
    if found_any_sequence {
        // If we found sequences, check if the latest one is invalidated
        if ResetStep::is_flash_value_invalidated(latest_value) {
            // Latest sequence was invalidated, move to next position
            latest_idx += 1;
            latest_value = 0xFF; // Next position should be erased
        } else {
            // Latest sequence is still valid, continue from there
        }
    } else {
    }
    
    
    // Check if we're at the end of the sector or no erased space left
    if latest_idx >= 4095 {
        flash_erase_sector(FLASH_ADR_RESET_CNT);
        latest_idx = 0;
        latest_value = 0xFF;
    }
    
    // Set the index to the determined position
    FACTORY_RESET_STATE.flash_address_index.set(latest_idx);
    
    // Convert the flash value to step number and update global state
    let reset_step = ResetStep::from_flash_value(latest_value);
    let step_number = reset_step.step_number();
    FACTORY_RESET_STATE.consecutive_reset_count.set(step_number);
}

/// Handle the factory reset process on device boot
///
/// # Algorithm
///
/// 1. Initialize the reset sequence tracking
/// 2. Read the current reset sequence step
/// 3. If the step equals FACTORY_RESET_FLAG, perform a factory reset
/// 4. Otherwise, advance to the next step in the sequence based on timing
///
/// # Notes
///
/// * This function should be called during device boot
pub fn factory_reset_handle() {
    // Initialize the reset sequence tracking
    init_reset_sequence();
    
    // Read the flash value directly to check for factory reset condition FIRST
    let current_addr = FLASH_ADR_RESET_CNT + FACTORY_RESET_STATE.flash_address_index.get();
    let flash_value = read_flash_byte(current_addr);
    
    
    // Check for factory reset condition BEFORE any other processing
    if ResetStep::should_factory_reset(flash_value) {
        
        // Disable interrupts to ensure the reset process isn't interrupted
        irq_disable();
        
        // Execute the factory reset to erase configuration data
        factory_reset();
        
        // Signal completion via LED indicators
        app().ota_manager.rf_led_ota_ok();
        
        // Reboot the device to apply the factory defaults
        light_sw_reboot();
    }
    
    // If we reach here, no factory reset needed - process normal step advancement
    let current_step = ResetStep::from_flash_value(flash_value);
    
    
    match current_step {
        ResetStep::Clear => {
            
            // Start sequence by marking Step 1 as complete
            FACTORY_RESET_STATE.consecutive_reset_count.set(1);
            FACTORY_RESET_STATE.clear_state.set(1);
            
            // Set up timing check for Step 1 (though it has no minimum time)
            if let Some((min_time, max_time)) = ResetStep::Step1.timing_window() {
                FACTORY_RESET_STATE.timing_check_timestamp.set(min_time as u32);
            }
            
            // Write Step 1 completion bit (clear bit 0: 0xFF → 0xFE)
            let current_addr = FLASH_ADR_RESET_CNT + FACTORY_RESET_STATE.flash_address_index.get();
            let new_value = ResetStep::Step1.flash_value();
            let data = [new_value];
            flash_write_page(current_addr, 1, data.as_ptr());
        }
        step => {
            
            // For existing steps, advance to the next step by clearing the next completion bit
            let next_step_number = step.step_number() + 1;
            
            if next_step_number <= POWER_CYCLE_COUNT {
                
                // Set up state for the new step
                FACTORY_RESET_STATE.consecutive_reset_count.set(next_step_number);
                FACTORY_RESET_STATE.clear_state.set(1);
                
                // Set up timing check
                let next_step = ResetStep::from_step_number(next_step_number);
                if let Some((min_time, max_time)) = next_step.timing_window() {
                    FACTORY_RESET_STATE.timing_check_timestamp.set(min_time as u32);
                }
                
                // Mark this step as complete by clearing its completion bit
                let current_addr = FLASH_ADR_RESET_CNT + FACTORY_RESET_STATE.flash_address_index.get();
                let current_value = read_flash_byte(current_addr);
                let new_value = next_step.apply_to(current_value);
                
                if new_value != current_value {
                    let data = [new_value];
                    flash_write_page(current_addr, 1, data.as_ptr());
                }
            } else {
            }
        }
        step if step.step_number() == POWER_CYCLE_COUNT => {
            
            // All steps completed successfully, set the factory reset flag
            write_reset_sequence(FACTORY_RESET_FLAG);
            
            // Reboot to trigger the actual factory reset
            light_sw_reboot();
        }
        _ => {
            
            // Invalid state, clear the sequence
            clear_reset_sequence();
        }
    }
}

/// Check and validate the factory reset timing sequence during device operation
///
/// # Algorithm
///
/// 1. If FACTORY_RESET_STATE.clear_state is 0, no timing check is active, exit early
/// 2. If FACTORY_RESET_STATE.clear_state is 1, check if the minimum time has elapsed:
///    - If passed, move to state 2 and set maximum time limit
/// 3. If FACTORY_RESET_STATE.clear_state is 2, check if the maximum time has been exceeded:
///    - If exceeded, abort the sequence
///
/// # Notes
///
/// * This function implements the timing verification logic for the factory reset sequence
/// * It validates each power cycle against specific time windows defined in POWER_CYCLE_TIMING
/// * If any timing window is not followed correctly, the entire sequence is reset
pub fn factory_reset_cnt_check() {
    // If no timing check is active, exit early
    if FACTORY_RESET_STATE.clear_state.get() == 0 {
        return;
    }
    
    // Get the current step in the reset sequence
    let current_step_number = FACTORY_RESET_STATE.consecutive_reset_count.get();
    let current_step = ResetStep::from_step_number(current_step_number);
    let clear_state = FACTORY_RESET_STATE.clear_state.get();
    
    // Validate the current step
    if current_step_number == 0 || current_step_number > POWER_CYCLE_COUNT {
        // Invalid state, clear the sequence
        clear_reset_sequence();
        FACTORY_RESET_STATE.clear_state.set(0);
        return;
    }
    
    // Get the timing window for this step from the configuration array
    let timing_window = POWER_CYCLE_TIMING[current_step_number as usize - 1];
    let (min_time, max_time) = timing_window;
    
    match FACTORY_RESET_STATE.clear_state.get() {
        1 => {
            // State 1: Check if minimum time has elapsed
            if clock_time_exceed(0, min_time as u32 * 1000 * 1000) {
                
                // Mark timing validation in flash (only for steps that require it)
                if ResetStep::step_requires_timing(current_step_number) {
                    write_timing_validation();
                }
                
                // Move to state 2
                FACTORY_RESET_STATE.clear_state.set(2);
                
                // Set up the maximum time limit
                FACTORY_RESET_STATE.timing_check_timestamp.set(max_time as u32);
            }
        }
        2 => {
            // State 2: Check if maximum time has been exceeded
            if clock_time_exceed(0, FACTORY_RESET_STATE.timing_check_timestamp.get() * 1000 * 1000) {
                // Maximum time exceeded, invalidate the sequence
                clear_reset_sequence();
                FACTORY_RESET_STATE.clear_state.set(0);
            }
            // Note: No else clause - we just wait for user to power cycle or max time to exceed
        }
        _ => {
            // Invalid clear state
            clear_reset_sequence();
            FACTORY_RESET_STATE.clear_state.set(0);
        }
    }
}

/// Advance to the next step in the factory reset sequence if timing requirements are met
///
/// This function handles the logic for moving from one step to the next when:
/// 1. Minimum time requirements have been satisfied
/// 2. User initiates a power cycle (detected through various means)
///
/// # Notes
///
/// * This is called from factory_reset_cnt_check() when in state 2 (valid timing window)
/// * It ensures steps only advance when timing requirements are properly met
/// Perform a factory reset by erasing configuration sectors in flash memory
///
/// # Algorithm
///
/// 1. Enter a critical section to prevent interrupts during flash operations
/// 2. Iterate through all flash sectors from CFG_SECTOR_ADR_MAC_CODE to FLASH_ADR_PAR_MAX
/// 3. Skip the reset counter sector to preserve it during the process
/// 4. Erase all other configuration sectors
/// 5. Finally erase the reset counter sector
///
/// # Notes
///
/// * Erases all configuration data except the MAC address (preserved in CFG_SECTOR_ADR_MAC_CODE)
/// * The reset counter sector is erased last to provide some recovery if power is lost during reset
/// * Each flash sector is 4KB (0x1000 bytes)
#[cfg_attr(test, mry::mry)]
fn factory_reset() {
    // Enter a critical section to prevent interrupts during flash operations
    // This is essential because flash operations can take time and must not be interrupted
    critical_section::with(|_| {
        // Calculate how many 4KB sectors to erase between MAC code sector and max address
        // We start from index 1 to preserve the MAC address in the first sector
        for i in 1..((FLASH_ADR_PAR_MAX - CFG_SECTOR_ADR_MAC_CODE) / 4096) {
            // Calculate the address of each sector
            let adr = CFG_SECTOR_ADR_MAC_CODE + i * 0x1000;
            
            // Skip the reset counter sector to preserve it during the main erase process
            // This enhances reliability if power is lost during the reset process
            if FLASH_ADR_RESET_CNT + FACTORY_RESET_STATE.flash_address_index.get() != adr {
                // Erase the current sector to clear all configuration data
                flash_erase_sector(adr);
            }
        }

        // Finally, erase the reset counter sector
        // We do this last to ensure other sectors are erased first
        // If power is lost during reset, we can still recover from partial reset
        flash_erase_sector(FLASH_ADR_RESET_CNT + FACTORY_RESET_STATE.flash_address_index.get()); // at last should be better, when power off during factory reset erase.
    });
}

/// Reasons for removing a device from the mesh network
#[derive(PartialEq, Debug)]
pub enum KickoutReason {
    /// Device is being removed from its current mesh network
    OutOfMesh = 0,
    /// Device has a default name that needs to be changed
    DefaultName,
    /// Maximum mode value (used for bounds checking)
    ModeMax,
}

impl TryFrom<u32> for KickoutReason {
    type Error = ();

    /// Convert a u32 value to a KickoutReason enum
    ///
    /// # Parameters
    ///
    /// * `v` - The u32 value to convert
    ///
    /// # Returns
    ///
    /// * `Ok(KickoutReason)` - Successfully converted value
    /// * `Err(())` - Conversion failed (invalid value)
    fn try_from(v: u32) -> Result<Self, Self::Error> {
        match v {
            x if x == KickoutReason::OutOfMesh as u32 => Ok(KickoutReason::OutOfMesh),
            x if x == KickoutReason::DefaultName as u32 => Ok(KickoutReason::DefaultName),
            x if x == KickoutReason::ModeMax as u32 => Ok(KickoutReason::ModeMax),
            _ => Err(()),
        }
    }
}

/// Remove a device from its current mesh network configuration
///
/// # Parameters
///
/// * `par` - The reason for removing the device from the mesh
///
/// # Algorithm
///
/// 1. Perform a factory reset to clear all configuration data
/// 2. If the reason is OutOfMesh:
///    - Save the mesh long-term key (LTK) to flash
///    - Encode and save the mesh password
///    - Save the "out of mesh" network name
///    - Set up pairing configuration flags
///    - Enable MAC address retrieval if mesh pairing is enabled
/// 3. Signal completion via LEDs
///
/// # Notes
///
/// * This preserves essential pairing information while removing device-specific settings
/// * For OutOfMesh, it saves critical credentials to allow rejoining the network later
/// * Data is written in 16-byte blocks aligned to flash page boundaries
pub fn kick_out(par: KickoutReason) {
    // First perform a factory reset to clear all configuration
    factory_reset();

    // Only preserve network credentials for OutOfMesh case
    if par == KickoutReason::OutOfMesh {
        let pairing_addr = FLASH_ADR_PAIRING;
        
        // --- Store the mesh long-term key (LTK) at offset +48 ---
        // Get current mesh LTK from global state
        let mut buff: [u8; 16] = *PAIR_CONFIG_MESH_LTK.lock();
        // Write LTK to flash at pairing_addr + 48
        flash_write_page(pairing_addr + 48, 16, buff.as_mut_ptr());

        // --- Store the encrypted mesh password at offset +32 ---
        // Create fresh buffer for mesh password
        let mut buff: [u8; 16] = [0; 16];
        // Take mesh password from config, ensuring we don't exceed buffer size
        let len = min(MESH_PWD.len(), buff.len());
        // Copy password bytes into the buffer
        buff[0..len].copy_from_slice(&MESH_PWD.as_bytes()[0..len]);
        // Encrypt the password before storing
        buff = encode_password(&buff);
        // Write encrypted password to flash at pairing_addr + 32
        flash_write_page(pairing_addr + 32, 16, buff.as_mut_ptr());

        // --- Store the "out of mesh" network name at offset +16 ---
        // Create fresh buffer for network name
        let mut buff: [u8; 16] = [0; 16];
        // Take network name from config, ensuring we don't exceed buffer size
        let len = min(OUT_OF_MESH.len(), buff.len());
        // Copy network name bytes into the buffer
        buff[0..len].copy_from_slice(&OUT_OF_MESH.as_bytes()[0..len]);
        // Write network name to flash at pairing_addr + 16
        flash_write_page(pairing_addr + 16, 16, buff.as_mut_ptr());

        // --- Store pairing configuration flags at offset +0 ---
        // Create fresh buffer for configuration flags
        let mut buff: [u8; 16] = [0; 16];
        // Set valid pairing flags at both ends of the buffer (redundancy)
        buff[0] = PAIR_VALID_FLAG;
        buff[15] = PAIR_VALID_FLAG;

        // If mesh pairing is enabled, set additional flag and enable MAC address retrieval
        if MESH_PAIR_ENABLE.get() {
            // Enable MAC address retrieval for mesh reconnection
            MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(true);
            // Set bit 1 to indicate mesh pairing is enabled
            buff[1] = 1;
        }
        // Write configuration flags to flash at base pairing_addr
        flash_write_page(pairing_addr, 16, buff.as_mut_ptr());
    }

    // Signal operation completion via LED indicators
    app().ota_manager.rf_led_ota_ok();
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::slice;
    use mry::send_wrapper::SendWrapper;
    use crate::sdk::drivers::flash::{mock_flash_write_page, mock_flash_erase_sector};
    use crate::sdk::mcu::clock::mock_clock_time_exceed;
    use crate::sdk::mcu::crypto::mock_encode_password;
    use crate::sdk::mcu::irq_i::mock_irq_disable;
    use crate::sdk::pm::mock_light_sw_reboot;
    use crate::{mock_app_mocker, app_mocker, App};
    // factory_reset is defined in this module, so we can reference mock_factory_reset directly
    
    /// Test the compile-time validation of power cycle timing configuration
    #[test]
    fn test_power_cycle_timing_validation() {
        // Current configuration should be valid
        assert!(POWER_CYCLE_TIMING.len() >= 2);
        assert!(POWER_CYCLE_TIMING.len() <= 5);
        
        // Test that we have the expected configuration
        assert_eq!(POWER_CYCLE_TIMING.len(), 5);
        assert_eq!(POWER_CYCLE_TIMING[0], (0, 3));  // Steps 1-3: quick cycles
        assert_eq!(POWER_CYCLE_TIMING[1], (0, 3));
        assert_eq!(POWER_CYCLE_TIMING[2], (0, 3));
        assert_eq!(POWER_CYCLE_TIMING[3], (3, 30)); // Steps 4-5: timing validation required
        assert_eq!(POWER_CYCLE_TIMING[4], (3, 30));
    }

    /// Test dynamic bit allocation for step completion
    #[test]
    fn test_step_completion_bits() {
        // Step completion bits should be sequential starting from bit 0
        assert_eq!(ResetStep::step_completion_bit(1), 0); // Step 1 -> bit 0
        assert_eq!(ResetStep::step_completion_bit(2), 1); // Step 2 -> bit 1  
        assert_eq!(ResetStep::step_completion_bit(3), 2); // Step 3 -> bit 2
        assert_eq!(ResetStep::step_completion_bit(4), 3); // Step 4 -> bit 3
        assert_eq!(ResetStep::step_completion_bit(5), 4); // Step 5 -> bit 4
    }

    /// Test dynamic bit allocation for timing validation
    #[test]
    fn test_timing_validation_bits() {
        // Steps 1-3 have no timing validation (min_time = 0)
        assert_eq!(ResetStep::step_timing_bit(1), None);
        assert_eq!(ResetStep::step_timing_bit(2), None);
        assert_eq!(ResetStep::step_timing_bit(3), None);
        
        // Steps 4-5 have timing validation (min_time = 3)
        assert_eq!(ResetStep::step_timing_bit(4), Some(5)); // First timing bit after all completion bits
        assert_eq!(ResetStep::step_timing_bit(5), Some(6)); // Second timing bit
    }

    /// Test step requirement checking
    #[test]
    fn test_step_requires_timing() {
        assert!(!ResetStep::step_requires_timing(0)); // Invalid step
        assert!(!ResetStep::step_requires_timing(1)); // Step 1: min_time = 0
        assert!(!ResetStep::step_requires_timing(2)); // Step 2: min_time = 0
        assert!(!ResetStep::step_requires_timing(3)); // Step 3: min_time = 0
        assert!(ResetStep::step_requires_timing(4));  // Step 4: min_time = 3
        assert!(ResetStep::step_requires_timing(5));  // Step 5: min_time = 3
        assert!(!ResetStep::step_requires_timing(6)); // Invalid step
    }

    /// Test flash value calculations for step completion
    #[test]
    fn test_flash_values() {
        assert_eq!(ResetStep::Clear.flash_value(), 0xFF);      // Erased state
        assert_eq!(ResetStep::Step1.flash_value(), 0xFE);      // Bit 0 cleared
        assert_eq!(ResetStep::Step2.flash_value(), 0xFC);      // Bits 0,1 cleared
        assert_eq!(ResetStep::Step3.flash_value(), 0xF8);      // Bits 0,1,2 cleared
        assert_eq!(ResetStep::Step4.flash_value(), 0xF0);      // Bits 0,1,2,3 cleared
        assert_eq!(ResetStep::Step5.flash_value(), 0xE0);      // Bits 0,1,2,3,4 cleared
        assert_eq!(ResetStep::FactoryReset.flash_value(), 0x80); // Factory reset ready
    }

    /// Test factory reset trigger condition
    #[test]
    fn test_factory_reset_condition() {
        assert!(!ResetStep::should_factory_reset(0xFF)); // Erased
        assert!(!ResetStep::should_factory_reset(0xFE)); // Step 1 only
        assert!(!ResetStep::should_factory_reset(0xE0)); // All steps, no timing validation
        assert!(!ResetStep::should_factory_reset(0xC0)); // Steps + Step 4 timing only
        assert!(ResetStep::should_factory_reset(0x80));  // All steps + all timing ✓
        assert!(!ResetStep::should_factory_reset(0x00)); // Invalidated
    }

    /// Test timing validation marking
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
        
        // Steps 1-3 don't need timing validation
        assert!(ResetStep::is_step_timing_validated(0xFE, 1)); // Always validated
        assert!(ResetStep::is_step_timing_validated(0xFC, 2)); // Always validated
        assert!(ResetStep::is_step_timing_validated(0xF8, 3)); // Always validated
        
        // Test invalidated values - should always return false (line 312)
        assert!(!ResetStep::is_step_timing_validated(0x78, 4)); // Step 3 invalidated (bit 7 clear)
        assert!(!ResetStep::is_step_timing_validated(0x50, 5)); // Step 4 invalidated (bit 7 clear)
        assert!(!ResetStep::is_step_timing_validated(0x20, 1)); // Step 5 invalidated (bit 7 clear)
    }

    /// Test complete factory reset sequence progression
    #[test]
    fn test_complete_sequence() {
        let mut flash_value = 0xFF; // Start erased
        
        // Step 1: 0xFF -> 0xFE
        flash_value = ResetStep::Step1.flash_value();
        assert_eq!(flash_value, 0xFE);
        
        // Step 2: 0xFE -> 0xFC  
        flash_value = ResetStep::Step2.flash_value();
        assert_eq!(flash_value, 0xFC);
        
        // Step 3: 0xFC -> 0xF8
        flash_value = ResetStep::Step3.flash_value();
        assert_eq!(flash_value, 0xF8);
        
        // Step 4: 0xF8 -> 0xF0, then timing validation -> 0xD0
        flash_value = ResetStep::Step4.flash_value();
        assert_eq!(flash_value, 0xF0);
        flash_value = ResetStep::mark_step_timing_validated(flash_value, 4);
        assert_eq!(flash_value, 0xD0);
        
        // Step 5: 0xD0 -> 0xC0, then timing validation -> 0x80
        flash_value = ResetStep::Step5.flash_value();
        assert_eq!(flash_value, 0xE0);
        // Note: Step 5 flash value is calculated from 0xFF, not from previous step
        // In practice, we'd apply it to the current flash value
        flash_value = 0xD0 & !(1 << ResetStep::step_completion_bit(5)); // Apply to current value
        assert_eq!(flash_value, 0xC0);
        flash_value = ResetStep::mark_step_timing_validated(flash_value, 5);
        assert_eq!(flash_value, 0x80); // Factory reset ready!
        
        assert!(ResetStep::should_factory_reset(flash_value));
    }

    /// Test invalidation
    #[test]
    fn test_invalidation() {
        let valid_sequence = 0xF0; // Step 4 complete
        let invalidated = ResetStep::invalidate_flash_value(valid_sequence);
        assert_eq!(invalidated, 0x70); // Bit 7 cleared
        assert!(ResetStep::is_flash_value_invalidated(invalidated));
        assert!(!ResetStep::is_flash_value_invalidated(valid_sequence));
        
        // Erased state is not considered invalidated
        assert!(!ResetStep::is_flash_value_invalidated(0xFF));
    }

    /// Test from_flash_value conversion
    #[test]
    fn test_from_flash_value() {
        assert_eq!(ResetStep::from_flash_value(0xFF), ResetStep::Clear);
        assert_eq!(ResetStep::from_flash_value(0xFE), ResetStep::Step1);
        assert_eq!(ResetStep::from_flash_value(0xFC), ResetStep::Step2);
        assert_eq!(ResetStep::from_flash_value(0xF8), ResetStep::Step3);
        assert_eq!(ResetStep::from_flash_value(0xF0), ResetStep::Step4);
        assert_eq!(ResetStep::from_flash_value(0xE0), ResetStep::Step5);
        assert_eq!(ResetStep::from_flash_value(0x80), ResetStep::FactoryReset);
        
        // Invalidated sequences should return Clear
        assert_eq!(ResetStep::from_flash_value(0x70), ResetStep::Clear); // Invalidated Step 4
        assert_eq!(ResetStep::from_flash_value(0x60), ResetStep::Clear); // Invalidated Step 5
    }

    /// Test step number conversion
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

    /// Test timing window retrieval
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

    /// Test edge cases and error conditions
    #[test] 
    fn test_edge_cases() {
        // Invalid step numbers
        assert_eq!(ResetStep::step_timing_bit(0), None);
        assert_eq!(ResetStep::step_timing_bit(6), None);
        assert_eq!(ResetStep::step_timing_bit(100), None);
        
        // Timing validation on steps that don't need it
        let unchanged = ResetStep::mark_step_timing_validated(0xFE, 1);
        assert_eq!(unchanged, 0xFE); // Step 1 doesn't need timing validation
        
        // Factory reset value calculation
        assert_eq!(ResetStep::factory_reset_value(), 0x80);
    }

    // ============================================================================
    // INTEGRATION TESTS - Testing actual runtime functions with mocked dependencies
    // ============================================================================

    /// Test init_reset_sequence finding the first available location
    #[test]
    #[mry::lock(read_flash_byte)]
    fn test_init_reset_sequence_find_location() {
        // Mock a partially used sector - some entries used, then erased
        let base_addr = FLASH_ADR_RESET_CNT;
        
        // Using returns_with to handle all flash reads in the sector scan
        {
            mock_read_flash_byte(mry::Any).returns_with(move |addr: u32| {
                let offset = addr - base_addr;
                if offset < 3 {
                    0xFE // Used entries at positions 0, 1, 2
                } else {
                    0xFF // Erased entries from position 3 onwards
                }
            });
        }

        init_reset_sequence();

        // Should find the last used position (index 2) since it's not invalidated
        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), 2);
    }

    /// Test init_reset_sequence when sector is full
    #[test]
    #[mry::lock(read_flash_byte, flash_erase_sector)]
    fn test_init_reset_sequence_sector_full() {
        let base_addr = FLASH_ADR_RESET_CNT;
        
        // Mock entire sector as used (no 0xFF values)
        for i in 0..4096 {
            mock_read_flash_byte(base_addr + i).returns(0xFE);
        }
        
        // Mock sector erase
        mock_flash_erase_sector(base_addr).returns(());

        init_reset_sequence();

        // Should erase sector and reset index to 0
        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), 0);
        mock_flash_erase_sector(base_addr).assert_called(1);
    }

    /// Test read_reset_sequence with different flash values
    #[test]
    #[mry::lock(read_flash_byte)]
    fn test_read_reset_sequence_integration() {
        // Set up flash index
        FACTORY_RESET_STATE.flash_address_index.set(5);
        let addr = FLASH_ADR_RESET_CNT + 5;
        
        // Using returns_with with a static counter to return different values on each call
        {
            static mut CALL_COUNTER: usize = 0;
            let return_values = [0xFF, ResetStep::Step3.flash_value(), 0x78, 0x80];
            mock_read_flash_byte(addr).returns_with(move |_| {
                unsafe {
                    let val = return_values[CALL_COUNTER];
                    CALL_COUNTER += 1;
                    val
                }
            });
        }
        
        // Test erased flash
        assert_eq!(read_reset_sequence(), 0);
        
        // Test Step 3 value
        assert_eq!(read_reset_sequence(), 3);
        
        // Test invalidated sequence
        assert_eq!(read_reset_sequence(), 0);
        
        // Test factory reset ready
        assert_eq!(read_reset_sequence(), 6); // FACTORY_RESET_FLAG
    }

    /// Test write_reset_sequence progression
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page)]
    fn test_write_reset_sequence_integration() {
        FACTORY_RESET_STATE.flash_address_index.set(10);
        let addr = FLASH_ADR_RESET_CNT + 10;
        
        // Test Step 1 on erased flash
        mock_read_flash_byte(addr).returns(0xFF);
        mock_flash_write_page(addr, 1, mry::Any).returns(());
        
        write_reset_sequence(1);
        
        // Test Step 2 progression
        mock_read_flash_byte(addr).returns(0xFE); // Previous Step 1
        mock_flash_write_page(addr, 1, mry::Any).returns(());
        
        write_reset_sequence(2);
        
        // Should have been called twice total (once for Step 1, once for Step 2)
        mock_flash_write_page(addr, 1, mry::Any).assert_called(2);
    }

    /// Test write_reset_sequence sector boundary handling
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page, flash_erase_sector)]
    fn test_write_reset_sequence_sector_boundary() {
        // Set up at sector boundary
        FACTORY_RESET_STATE.flash_address_index.set(4095);
        let addr = FLASH_ADR_RESET_CNT + 4095;
        
        mock_read_flash_byte(addr).returns(0xFF);
        mock_flash_write_page(addr, 1, mry::Any).returns(());
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).returns(());

        write_reset_sequence(0x80); // Write FactoryReset step to complete sequence

        // Should trigger sector management
        mock_flash_write_page(addr, 1, mry::Any).assert_called(1);
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).assert_called(1);
        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), 0);
    }

    /// Test timing validation integration
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page)]
    fn test_write_timing_validation_integration() {
        // Set up Step 4 complete
        FACTORY_RESET_STATE.flash_address_index.set(15);
        FACTORY_RESET_STATE.consecutive_reset_count.set(4);
        let addr = FLASH_ADR_RESET_CNT + 15;
        
        mock_read_flash_byte(addr).returns(ResetStep::Step4.flash_value()); // 0xF0
        mock_flash_write_page(addr, 1, mry::Any).returns(());

        write_timing_validation();

        // Should write timing validation bit for Step 4
        
        // Test Step 5 timing validation
        FACTORY_RESET_STATE.consecutive_reset_count.set(5);
        mock_read_flash_byte(addr).returns(0xD0); // Step 4 with timing
        mock_flash_write_page(addr, 1, mry::Any).returns(());

        write_timing_validation();

        // Should have been called twice total (once for Step 4, once for Step 5)
        mock_flash_write_page(addr, 1, mry::Any).assert_called(2);
    }

    /// Test clear_reset_sequence integration
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page)]
    fn test_clear_reset_sequence_integration() {
        FACTORY_RESET_STATE.flash_address_index.set(20);
        let addr = FLASH_ADR_RESET_CNT + 20;
        
        mock_read_flash_byte(addr).returns(ResetStep::Step3.flash_value()); // 0xF8
        mock_flash_write_page(addr, 1, mry::Any).returns(());

        let initial_index = FACTORY_RESET_STATE.flash_address_index.get();

        clear_reset_sequence();

        // Should write invalidation and increment index
        mock_flash_write_page(addr, 1, mry::Any).assert_called(1);
        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), initial_index + 1);
    }

    /// Test clear_reset_sequence at sector boundary
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page, flash_erase_sector)]
    fn test_clear_reset_sequence_sector_boundary() {
        FACTORY_RESET_STATE.flash_address_index.set(4095);
        let addr = FLASH_ADR_RESET_CNT + 4095;
        
        mock_read_flash_byte(addr).returns(ResetStep::Step2.flash_value());
        mock_flash_write_page(addr, 1, mry::Any).returns(());
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).returns(());

        clear_reset_sequence();

        // Should trigger sector management
        mock_flash_write_page(addr, 1, mry::Any).assert_called(1);
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).assert_called(1);
        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), 0);
    }

    /// Test factory_reset_handle complete workflow
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page)]
    fn test_factory_reset_handle_complete_flow() {
        // Reset state
        FACTORY_RESET_STATE.consecutive_reset_count.set(0);
        FACTORY_RESET_STATE.clear_state.set(0);
        FACTORY_RESET_STATE.flash_address_index.set(0);
        
        let base_addr = FLASH_ADR_RESET_CNT;
        
        // Mock the sector scan that init_reset_sequence performs
        // Simulate used entries at positions 0, 1, 2 and erased from position 3 onwards
        {
            static mut CALL_COUNT: usize = 0;
            mock_read_flash_byte(mry::Any).returns_with(move |addr: u32| {
                let offset = addr - base_addr;
                unsafe {
                    CALL_COUNT += 1;
                    if offset < 3 {
                        0xFE // Used entries at positions 0, 1, 2
                    } else {
                        0xFF // Erased entries from position 3 onwards
                    }
                }
            });
        }
        
        // Mock write operation for Step 1
        let current_addr = FLASH_ADR_RESET_CNT + 2; // Last used position 
        mock_flash_write_page(current_addr, 1, mry::Any).returns(());

        factory_reset_handle();

        // Verify write operation occurred
        mock_flash_write_page(current_addr, 1, mry::Any).assert_called(1);
        
        // Verify state setup
        assert_eq!(FACTORY_RESET_STATE.consecutive_reset_count.get(), 2); // Step1 -> Step2
        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 1);
        assert_eq!(FACTORY_RESET_STATE.timing_check_timestamp.get(), POWER_CYCLE_TIMING[1].0 as u32); // Step 2 timing
        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), 2); // Position of last used entry
    }

    /// Test factory_reset_handle step progression
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page)]
    fn test_factory_reset_handle_step_progression() {
        let current_step = 2;
        
        // Set up initial state - simulate sector already initialized
        FACTORY_RESET_STATE.flash_address_index.set(5);
        FACTORY_RESET_STATE.consecutive_reset_count.set(0);
        FACTORY_RESET_STATE.clear_state.set(0);
        
        let current_addr = FLASH_ADR_RESET_CNT + 5;
        let base_addr = FLASH_ADR_RESET_CNT;
        
        // Mock the sector scan that init_reset_sequence performs
        // Using returns_with to handle all flash reads during sector scan
        {
            static mut CALL_COUNT: usize = 0;
            mock_read_flash_byte(mry::Any).returns_with(move |addr: u32| {
                let offset = addr - base_addr;
                unsafe {
                    CALL_COUNT += 1;
                    if offset == 5 {
                        ResetStep::Step2.flash_value() // Step 2 in progress at position 5
                    } else {
                        0xFF // Erased everywhere else
                    }
                }
            });
        }
        
        // Mock write for next step (Step 3)
        mock_flash_write_page(current_addr, 1, mry::Any).returns(());

        factory_reset_handle();

        // Verify write operation happened
        mock_flash_write_page(current_addr, 1, mry::Any).assert_called(1);
        
        // Verify state updates
        assert_eq!(FACTORY_RESET_STATE.consecutive_reset_count.get(), current_step + 1);
        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 1);
        let timing_idx = (current_step + 1) - 1; // 0-indexed array
        assert_eq!(FACTORY_RESET_STATE.timing_check_timestamp.get(), POWER_CYCLE_TIMING[timing_idx as usize].0 as u32);
    }

    /// Test factory_reset_handle factory reset completion
    #[test]
    #[mry::lock(read_flash_byte, flash_erase_sector, irq_disable, factory_reset, light_sw_reboot, app_mocker)]
    fn test_factory_reset_handle_completed_sequence() {
        // Set up initial state
        FACTORY_RESET_STATE.flash_address_index.set(10);
        FACTORY_RESET_STATE.consecutive_reset_count.set(0);
        FACTORY_RESET_STATE.clear_state.set(0);
        
        let current_addr = FLASH_ADR_RESET_CNT + 10;
        let base_addr = FLASH_ADR_RESET_CNT;
        
        // Mock the sector scan that init_reset_sequence performs
        {
            mock_read_flash_byte(mry::Any).returns_with(move |addr: u32| {
                let offset = addr - base_addr;
                if offset == 10 {
                    0x80 // FACTORY_RESET_FLAG at position 10
                } else {
                    0xFF // Erased everywhere else
                }
            });
        }
        
        // Set up mock app with mocked ota_manager
        let mut app = App::default();
        app.ota_manager.mock_rf_led_ota_ok().returns(());
        mock_app_mocker().returns(&mut app);
        
        // Mock all the functions called during factory reset
        mock_irq_disable().returns(0);
        mock_factory_reset().returns(());
        mock_light_sw_reboot().returns(());
        mock_flash_erase_sector(mry::Any).returns(());

        factory_reset_handle();

        // Verify factory reset functions were called
        mock_irq_disable().assert_called(1);
        mock_factory_reset().assert_called(1);
        app.ota_manager.mock_rf_led_ota_ok().assert_called(1);
        mock_light_sw_reboot().assert_called(1);
        // Don't check flash_erase_sector for now - it might be called from within factory_reset()
        
        // Verify state cleared
        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 0);
    }

    /// Test factory_reset_handle with fresh sector (Clear case)
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page, app_mocker)]
    fn test_factory_reset_handle_clear_case() {
        // Set up initial state - fresh sector
        FACTORY_RESET_STATE.flash_address_index.set(0);
        FACTORY_RESET_STATE.consecutive_reset_count.set(0);
        FACTORY_RESET_STATE.clear_state.set(0);
        
        let base_addr = FLASH_ADR_RESET_CNT;
        
        // Mock the sector scan that init_reset_sequence performs
        // All flash reads return 0xFF (erased sector)
        {
            mock_read_flash_byte(mry::Any).returns_with(move |_addr: u32| {
                0xFF // Entire sector is erased
            });
        }
        
        // Set up mock app with mocked ota_manager  
        let mut app = App::default();
        app.ota_manager.mock_rf_led_ota_ok().returns(());
        mock_app_mocker().returns(&mut app);
        
        // Mock flash_write_page (called during Clear case)
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).returns(());
        
        // Call factory_reset_handle (no parameters)
        factory_reset_handle();
        
        // Should have initialized the state properly after finding erased sector
        // The Clear case (line 724) should have been triggered since entire sector is 0xFF
        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), 0);
        assert_eq!(FACTORY_RESET_STATE.consecutive_reset_count.get(), 1);
        assert!(FACTORY_RESET_STATE.clear_state.get() > 0); // Timing started
        
        // Verify the mocks were called appropriately
        // The Clear case writes Step1 to flash but doesn't trigger LED indication
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).assert_called(1); // Clear case writes Step1
    }

    /// Test factory_reset_cnt_check with invalid state
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page)]
    fn test_factory_reset_cnt_check_invalid_state() {
        // Set up invalid state
        FACTORY_RESET_STATE.consecutive_reset_count.set(0); // Invalid
        FACTORY_RESET_STATE.clear_state.set(1); // But active timing check
        FACTORY_RESET_STATE.flash_address_index.set(15);
        
        let current_addr = FLASH_ADR_RESET_CNT + 15;
        
        // Mock reading current sequence and invalidation write
        mock_read_flash_byte(current_addr).returns(ResetStep::Step3.flash_value());
        mock_flash_write_page(current_addr, 1, mry::Any).returns(());

        factory_reset_cnt_check();

        // Should clear sequence due to invalid state
        mock_read_flash_byte(current_addr).assert_called(1);
        mock_flash_write_page(current_addr, 1, mry::Any).assert_called(1);
        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 0);
        assert_eq!(FACTORY_RESET_STATE.flash_address_index.get(), 16); // Incremented
    }

    /// Test timing check remains pending when minimum time has not elapsed
    #[test]
    #[mry::lock(clock_time_exceed, write_timing_validation, flash_write_page)]
    fn test_factory_reset_cnt_check_min_time_not_elapsed() {
        // Set up state 1 for step 4 (requires timing validation)
        FACTORY_RESET_STATE.consecutive_reset_count.set(4);
        FACTORY_RESET_STATE.clear_state.set(1);
        FACTORY_RESET_STATE.flash_address_index.set(25);
        let sentinel_timestamp = 1234;
        FACTORY_RESET_STATE.timing_check_timestamp.set(sentinel_timestamp);

        let min_time_us = (POWER_CYCLE_TIMING[3].0 as u32) * 1_000_000;

        mock_clock_time_exceed(0, min_time_us).returns(false);
        mock_write_timing_validation().returns(());
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).returns(());

        factory_reset_cnt_check();

        mock_clock_time_exceed(0, min_time_us).assert_called(1);
        mock_write_timing_validation().assert_called(0);
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).assert_called(0);
        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 1);
        assert_eq!(FACTORY_RESET_STATE.timing_check_timestamp.get(), sentinel_timestamp);
    }

    /// Test factory_reset_cnt_check timing state machine
    #[test]
    #[mry::lock(clock_time_exceed, read_flash_byte, flash_write_page)]
    fn test_factory_reset_cnt_check_timing_state_machine() {
        // Set up valid state 1 (checking minimum time)
        FACTORY_RESET_STATE.consecutive_reset_count.set(4); // Step 4
        FACTORY_RESET_STATE.clear_state.set(1);
        FACTORY_RESET_STATE.timing_check_timestamp.set(POWER_CYCLE_TIMING[3].0 as u32);
        FACTORY_RESET_STATE.flash_address_index.set(25);
        
        let current_addr = FLASH_ADR_RESET_CNT + 25;
        
        // Mock minimum time elapsed
        mock_clock_time_exceed(0, POWER_CYCLE_TIMING[3].0 as u32 * 1000 * 1000).returns(true);
        mock_read_flash_byte(current_addr).returns(ResetStep::Step4.flash_value());
        mock_flash_write_page(current_addr, 1, mry::Any).returns(());

        factory_reset_cnt_check();

        // Should transition to state 2 and write timing validation
        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 2);
        assert_eq!(FACTORY_RESET_STATE.timing_check_timestamp.get(), POWER_CYCLE_TIMING[3].1 as u32);
        mock_clock_time_exceed(0, POWER_CYCLE_TIMING[3].0 as u32 * 1000 * 1000).assert_called(1);
        mock_read_flash_byte(current_addr).assert_called(1);
        mock_flash_write_page(current_addr, 1, mry::Any).assert_called(1);
    }

    /// Test factory_reset_cnt_check maximum time exceeded
    #[test]
    #[mry::lock(clock_time_exceed, read_flash_byte, flash_write_page)]
    fn test_factory_reset_cnt_check_max_time_exceeded() {
        // Set up state 2 (checking maximum time)
        FACTORY_RESET_STATE.consecutive_reset_count.set(4);
        FACTORY_RESET_STATE.clear_state.set(2);
        FACTORY_RESET_STATE.timing_check_timestamp.set(POWER_CYCLE_TIMING[3].1 as u32);
        FACTORY_RESET_STATE.flash_address_index.set(20);
        
        let current_addr = FLASH_ADR_RESET_CNT + 20;
        
        // Mock maximum time exceeded
        mock_clock_time_exceed(0, POWER_CYCLE_TIMING[3].1 as u32 * 1000 * 1000).returns(true);
        mock_read_flash_byte(current_addr).returns(ResetStep::Step4.flash_value());
        mock_flash_write_page(current_addr, 1, mry::Any).returns(());

        factory_reset_cnt_check();

        // Should clear sequence and reset state
        mock_clock_time_exceed(0, POWER_CYCLE_TIMING[3].1 as u32 * 1000 * 1000).assert_called(1);
        mock_read_flash_byte(current_addr).assert_called(1);
        mock_flash_write_page(current_addr, 1, mry::Any).assert_called(1);
        assert_eq!(FACTORY_RESET_STATE.clear_state.get(), 0);
    }

    /// Test complete sequence with timing validation
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page)]
    fn test_complete_sequence_with_timing_validation() {
        FACTORY_RESET_STATE.flash_address_index.set(30);
        let addr = FLASH_ADR_RESET_CNT + 30;
        
        // Start with erased flash
        mock_read_flash_byte(addr).returns(0xFF);
        mock_flash_write_page(addr, 1, mry::Any).returns(());
        
        // Progress through all steps
        for step in 1..=5 {
            mock_read_flash_byte(addr).returns(match step {
                1 => 0xFF,
                2 => 0xFE,
                3 => 0xFC,
                4 => 0xF8,
                5 => 0xF0,
                _ => unreachable!(),
            });
            mock_flash_write_page(addr, 1, mry::Any).returns(());
            
            write_reset_sequence(step);
        }
        
        // Now test timing validation for steps 4 and 5
        FACTORY_RESET_STATE.consecutive_reset_count.set(4);
        mock_read_flash_byte(addr).returns(0xE0); // All steps complete
        mock_flash_write_page(addr, 1, mry::Any).returns(());
        
        write_timing_validation(); // Add Step 4 timing
        
        FACTORY_RESET_STATE.consecutive_reset_count.set(5);
        mock_read_flash_byte(addr).returns(0xC0); // Step 4 timing added
        mock_flash_write_page(addr, 1, mry::Any).returns(());
        
        write_timing_validation(); // Add Step 5 timing
        
        // Verify all flash operations occurred
        // Each step writes once, plus 2 timing validations = 7 total writes
        mock_flash_write_page(addr, 1, mry::Any).assert_called(7);
    }

    /// Test attack prevention - rapid reset without timing
    #[test]
    fn test_rapid_reset_attack_prevention_integration() {
        // Test that rapid completion of all steps without timing validation fails
        let all_steps_complete = ResetStep::Step5.flash_value(); // 0xE0
        
        // This should NOT trigger factory reset
        assert!(!ResetStep::should_factory_reset(all_steps_complete));
        
        // Add only Step 4 timing - still not enough
        let with_step4_timing = ResetStep::mark_step_timing_validated(all_steps_complete, 4);
        assert_eq!(with_step4_timing, 0xC0);
        assert!(!ResetStep::should_factory_reset(with_step4_timing));
        
        // Only with both timing validations does it work
        let with_both_timing = ResetStep::mark_step_timing_validated(with_step4_timing, 5);
        assert_eq!(with_both_timing, 0x80);
        assert!(ResetStep::should_factory_reset(with_both_timing));
        
        // This proves the algorithm prevents rapid reset attacks
    }

    /// Test factory reset function execution
    #[test]
    #[mry::lock(flash_erase_sector)]
    fn test_factory_reset_execution() {
        // Mock sector erasure for all the configuration sectors
        mock_flash_erase_sector(mry::Any).returns(());

        factory_reset();

        // Verify reset counter sector was erased (should be last operation)
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).assert_called(1);
        
        // Verify other configuration sectors were also erased
        // (The exact number depends on the configuration, but there should be multiple calls)
        // We can't easily get a call count with mry, so we just verify the reset sector was called
    }

    /// Test that kick_out writes default credentials and pairing flags to flash
    #[test]
    #[mry::lock(flash_erase_sector, flash_write_page, encode_password, app_mocker)]
    fn test_kick_out_out_of_mesh_writes_credentials_and_flags() {
        // Ensure a known LTK is stored before invoking kick_out
        let expected_ltk = [0xAAu8; 16];
        {
            let mut ltk = PAIR_CONFIG_MESH_LTK.lock();
            *ltk = expected_ltk;
        }

        // Configure mesh pairing state to enable pairing flag path
        MESH_PAIR_ENABLE.set(true);
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);

        let encoded_password = [0x5Au8; 16];
        mock_encode_password(mry::Any).returns(encoded_password);

        let mut expected_name = [0u8; 16];
        let name_bytes = OUT_OF_MESH.as_bytes();
        let copy_len = name_bytes.len().min(expected_name.len());
        expected_name[..copy_len].copy_from_slice(&name_bytes[..copy_len]);

        let mut expected_flags = [0u8; 16];
        expected_flags[0] = PAIR_VALID_FLAG;
        expected_flags[15] = PAIR_VALID_FLAG;
        expected_flags[1] = 1; // Mesh pairing enabled flag

        let pairing_addr = FLASH_ADR_PAIRING;

        mock_flash_erase_sector(mry::Any).returns(());

        let expected_flags_closure = expected_flags;
        let expected_name_closure = expected_name;
        let expected_ltk_closure = expected_ltk;
        let encoded_password_closure = encoded_password;

        mock_flash_write_page(mry::Any, mry::Any, mry::Any).returns_with(move |addr: u32, len: u32, buf: SendWrapper<*const u8>| {
            assert_eq!(len, 16);
            let data = unsafe { slice::from_raw_parts(*buf, len as usize) };
            match addr {
                a if a == pairing_addr + 48 => assert_eq!(data, &expected_ltk_closure),
                a if a == pairing_addr + 32 => assert_eq!(data, &encoded_password_closure),
                a if a == pairing_addr + 16 => assert_eq!(data, &expected_name_closure),
                a if a == pairing_addr => assert_eq!(data, &expected_flags_closure),
                a => panic!("unexpected flash write address: 0x{:x}", a),
            }
        });

        let mut app = App::default();
        app.ota_manager.mock_rf_led_ota_ok().returns(());
        mock_app_mocker().returns(&mut app);

        kick_out(KickoutReason::OutOfMesh);

        mock_flash_write_page(mry::Any, mry::Any, mry::Any).assert_called(4);
        mock_encode_password(mry::Any).assert_called(1);
        assert!(MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get());
        app.ota_manager.mock_rf_led_ota_ok().assert_called(1);

        // Restore mesh pairing flags for other tests
        MESH_PAIR_ENABLE.set(false);
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);
    }
}
