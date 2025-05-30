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
//! Address: FLASH_ADR_RESET_CNT + index
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
//! - **CLEAR_ST = 0**: No timing check active
//! - **CLEAR_ST = 1**: Checking minimum time requirement
//! - **CLEAR_ST = 2**: Checking maximum time limit
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
    (0, 3),   // Device must be turned off within 3 seconds of turning on
    (0, 3),   // Device must be turned off within 3 seconds of turning on
    (0, 3),   // Device must be turned off within 3 seconds of turning on
    (3, 30),  // Device must be on for 3-30 seconds before turning off
    (3, 30),  // Device must be on for 3-30 seconds before turning off
];

/// Flash-optimized reset sequence steps using progressive bit clearing
/// 
/// # Bit Allocation Strategy
/// 
/// - Bit 7: Factory Reset flag
/// - Bit 6: Invalidation flag (0 = invalidated sequence)  
/// - Bits 4-0: Step progression (each step clears one bit)
/// 
/// This approach minimizes bit flips since flash can only write 1→0 transitions.
/// Starting from 0xFF, each step clears exactly one bit, requiring only 6 total
/// bit flips for the complete sequence vs 24+ in traditional approaches.
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
    /// Step 5: 0xF0 → 0xE0 (clear bit 4)
    Step5,
    /// Factory reset ready: 0xE0 → 0x60 (clear bit 7)
    FactoryReset,
}

impl ResetStep {
    /// Number of steps in the power cycle sequence
    pub const POWER_CYCLE_COUNT: u8 = 5;
    
    /// Invalidation bitmask: clears bit 6 (xxxx → xx0x)
    const INVALIDATION_MASK: u8 = 0b10111111;
    
    /// Get the bitmask that clears the appropriate bit for this step
    const fn bitmask(self) -> u8 {
        match self {
            ResetStep::Clear => 0xFF,
            ResetStep::Step1 => 0b11111110,  // Clear bit 0
            ResetStep::Step2 => 0b11111101,  // Clear bit 1
            ResetStep::Step3 => 0b11111011,  // Clear bit 2
            ResetStep::Step4 => 0b11110111,  // Clear bit 3
            ResetStep::Step5 => 0b11101111,  // Clear bit 4
            ResetStep::FactoryReset => 0b01111111,  // Clear bit 7
        }
    }
    
    /// Get the expected flash value after applying this step's bitmask
    pub const fn flash_value(self) -> u8 {
        match self {
            ResetStep::Clear => 0xFF,
            ResetStep::Step1 => 0xFE,
            ResetStep::Step2 => 0xFC,
            ResetStep::Step3 => 0xF8,
            ResetStep::Step4 => 0xF0,
            ResetStep::Step5 => 0xE0,
            ResetStep::FactoryReset => 0x60,
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
    
    /// Get the next step in the sequence
    pub const fn next(self) -> Option<ResetStep> {
        match self {
            ResetStep::Clear => Some(ResetStep::Step1),
            ResetStep::Step1 => Some(ResetStep::Step2),
            ResetStep::Step2 => Some(ResetStep::Step3),
            ResetStep::Step3 => Some(ResetStep::Step4),
            ResetStep::Step4 => Some(ResetStep::Step5),
            ResetStep::Step5 => Some(ResetStep::FactoryReset),
            ResetStep::FactoryReset => None,
        }
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
        // Check if bit 6 is cleared AND it's not erased/clear state
        (value & 0b01000000) == 0 && value != 0xFF && value != 0x00
    }
}

impl From<ResetStep> for u8 {
    fn from(step: ResetStep) -> u8 {
        step.step_number()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetStepError {
    InvalidStepNumber,
}

impl TryFrom<u8> for ResetStep {
    type Error = ResetStepError;
    
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        let step = ResetStep::from_step_number(value);
        if step.step_number() == value {
            Ok(step)
        } else {
            Err(ResetStepError::InvalidStepNumber)
        }
    }
}

impl ResetStep {
    /// Convert flash value back to reset step
    pub const fn from_flash_value(value: u8) -> Self {
        // First check if sequence is invalidated
        if Self::is_flash_value_invalidated(value) {
            return ResetStep::Clear;
        }
        
        match value {
            0xFF => ResetStep::Clear,
            0xFE => ResetStep::Step1,
            0xFC => ResetStep::Step2,
            0xF8 => ResetStep::Step3,
            0xF0 => ResetStep::Step4,
            0xE0 => ResetStep::Step5,
            0x60 => ResetStep::FactoryReset,
            _ => ResetStep::Clear, // Invalid/corrupted state
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
            ResetStep::FactoryReset => 0x80,
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
pub const POWER_CYCLE_COUNT: u8 = ResetStep::POWER_CYCLE_COUNT;

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
    let current_addr = FLASH_ADR_RESET_CNT + ADR_RESET_CNT_IDX.get();
    let current_value = read_flash_byte(current_addr);
    
    // Calculate new value by applying the step's bitmask
    let new_value = reset_step.apply_to(current_value);
    
    // Only write if we're actually changing bits (1→0 transitions)
    if new_value != current_value {
        let data = [new_value];
        flash_write_page(current_addr, 1, data.as_ptr());
    }
    
    // Check if this sequence is complete (reset flag written)
    if reset_step.is_sequence_complete() {
        // Move to next location for the next factory reset sequence
        ADR_RESET_CNT_IDX.inc();
        
        // Check if we're at the end of the sector
        if ADR_RESET_CNT_IDX.get() >= 4096 {
            flash_erase_sector(FLASH_ADR_RESET_CNT);
            ADR_RESET_CNT_IDX.set(0);
        }
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
    let current_addr = FLASH_ADR_RESET_CNT + ADR_RESET_CNT_IDX.get();
    let flash_value = read_flash_byte(current_addr);
    
    // Convert flash bitmask value back to step number
    let reset_step = ResetStep::from_flash_value(flash_value);
    reset_step.step_number()
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
    // Invalidate current sequence by clearing bit 6 (invalidation flag)
    let current_addr = FLASH_ADR_RESET_CNT + ADR_RESET_CNT_IDX.get();
    let current_value = read_flash_byte(current_addr);
    
    // Only invalidate if there's actually a sequence to invalidate
    if current_value != 0xFF {
        let invalidated_value = ResetStep::invalidate_flash_value(current_value);
        let invalidation_data = [invalidated_value];
        flash_write_page(current_addr, 1, invalidation_data.as_ptr());
    }
    
    // Move to next location for fresh sequence
    ADR_RESET_CNT_IDX.inc();
    
    // Check if we're at the end of the sector
    if ADR_RESET_CNT_IDX.get() >= 4096 {
        flash_erase_sector(FLASH_ADR_RESET_CNT);
        ADR_RESET_CNT_IDX.set(0);
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
/// * Sets up ADR_RESET_CNT_IDX for subsequent operations
/// * Skips invalidated sequences (bit 6 = 0)
#[cfg_attr(test, mry::mry)]
fn init_reset_sequence() {
    // Start scanning from the beginning of the sector
    ADR_RESET_CNT_IDX.set(0);
    
    // Scan through flash sector to find the latest valid reset sequence entry
    let mut latest_idx = 0;
    let mut latest_value = 0xFF; // Erased state
    
    // Scan through the entire 4KB sector
    for i in 0..4096 {
        let value = read_flash_byte(FLASH_ADR_RESET_CNT + i);
        
        // Skip erased locations and invalidated sequences
        if value != 0xFF && !ResetStep::is_flash_value_invalidated(value) {
            // Check if it's a valid sequence pattern
            let reset_step = ResetStep::from_flash_value(value);
            if reset_step != ResetStep::Clear {
                latest_idx = i;
                latest_value = value;
            }
        }
    }
    
    // Set the index to the latest valid entry (or 0 if all erased/invalid)
    ADR_RESET_CNT_IDX.set(latest_idx);
    
    // Convert the flash value to step number and update global state
    let reset_step = ResetStep::from_flash_value(latest_value);
    let step_number = reset_step.step_number();
    RESET_CNT.set(step_number);
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
    
    // Get the current reset sequence step
    let current_step_number = read_reset_sequence();
    let current_step = ResetStep::from_step_number(current_step_number);
    
    match current_step {
        ResetStep::FactoryReset => {
            // Disable interrupts to ensure the reset process isn't interrupted
            irq_disable();
            
            // Execute the factory reset to erase configuration data
            factory_reset();
            
            // Signal completion via LED indicators
            app().ota_manager.rf_led_ota_ok();
            
            // Reboot the device to apply the factory defaults
            light_sw_reboot();
        }
        ResetStep::Clear => {
            // No reset sequence in progress, initialize step 1
            let next_step = ResetStep::Step1;
            RESET_CNT.set(next_step.step_number());
            CLEAR_ST.set(1);
            
            // Set up timing check for the first step
            if let Some((min_time, _)) = next_step.timing_window() {
                RESET_CHECK_TIME.set(min_time as u32);
            }
            
            // Record the first step in flash
            write_reset_sequence(next_step.step_number());
        }
        step if step.step_number() > 0 && step.step_number() < POWER_CYCLE_COUNT => {
            // Reset sequence in progress, move to the next step
            if let Some(next_step) = step.next() {
                if next_step == ResetStep::FactoryReset {
                    // All steps completed successfully, set the factory reset flag
                    write_reset_sequence(FACTORY_RESET_FLAG);
                    
                    // Reboot to trigger the actual factory reset
                    light_sw_reboot();
                } else {
                    // Continue with the next step
                    RESET_CNT.set(next_step.step_number());
                    CLEAR_ST.set(1);
                    
                    // Set up timing check for this step
                    if let Some((min_time, _)) = next_step.timing_window() {
                        RESET_CHECK_TIME.set(min_time as u32);
                    }
                    
                    // Record the next step in flash
                    write_reset_sequence(next_step.step_number());
                }
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
/// 1. If CLEAR_ST is 0, no timing check is active, exit early
/// 2. If CLEAR_ST is 1, check if the minimum time has elapsed:
///    - If passed, move to state 2 and set maximum time limit
/// 3. If CLEAR_ST is 2, check if the maximum time has been exceeded:
///    - If exceeded, abort the sequence
///
/// # Notes
///
/// * This function implements the timing verification logic for the factory reset sequence
/// * It validates each power cycle against specific time windows defined in POWER_CYCLE_TIMING
/// * If any timing window is not followed correctly, the entire sequence is reset
pub fn factory_reset_cnt_check() {
    // If no timing check is active, exit early
    if CLEAR_ST.get() == 0 {
        return;
    }
    
    // Get the current step in the reset sequence
    let current_step_number = RESET_CNT.get();
    let current_step = ResetStep::from_step_number(current_step_number);
    
    // Validate the current step
    if current_step_number == 0 || current_step_number > POWER_CYCLE_COUNT {
        // Invalid state, clear the sequence
        clear_reset_sequence();
        CLEAR_ST.set(0);
        return;
    }
    
    // Get the timing window for this step
    if let Some((min_time, max_time)) = current_step.timing_window() {
        match CLEAR_ST.get() {
            1 => {
                // State 1: Check if minimum time has elapsed
                if clock_time_exceed(0, min_time as u32 * 1000 * 1000) {
                    // Minimum time elapsed, move to state 2
                    CLEAR_ST.set(2);
                    
                    // Set up the maximum time limit
                    RESET_CHECK_TIME.set(max_time as u32);
                }
            }
            2 => {
                // State 2: Check if maximum time has been exceeded
                if clock_time_exceed(0, RESET_CHECK_TIME.get() * 1000 * 1000) {
                    // Maximum time exceeded, abort the sequence
                    clear_reset_sequence();
                    CLEAR_ST.set(0);
                }
            }
            _ => {
                // Invalid clear state
                clear_reset_sequence();
                CLEAR_ST.set(0);
            }
        }
    } else {
        // No timing window defined for this step, invalid state
        clear_reset_sequence();
        CLEAR_ST.set(0);
    }
}

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
            if FLASH_ADR_RESET_CNT != adr {
                // Erase the current sector to clear all configuration data
                flash_erase_sector(adr);
            }
        }

        // Finally, erase the reset counter sector
        // We do this last to ensure other sectors are erased first
        // If power is lost during reset, we can still recover from partial reset
        flash_erase_sector(FLASH_ADR_RESET_CNT); // at last should be better, when power off during factory reset erase.
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
            GET_MAC_EN.set(true);
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
    use std::vec::Vec;
    
    // Import mock functions from their original modules
    use crate::sdk::drivers::flash::{
        mock_flash_erase_sector, mock_flash_read_page, mock_flash_write_page
    };
    use crate::sdk::mcu::clock::mock_clock_time_exceed;
    use crate::sdk::mcu::crypto::mock_encode_password;
    use crate::sdk::mcu::irq_i::mock_irq_disable;
    use crate::sdk::pm::mock_light_sw_reboot;
    use crate::app::App;
    use crate::{mock_app_mocker, app_mocker};
    
    /// Tests the read_flash_byte function.
    ///
    /// This test verifies that read_flash_byte correctly reads a value from flash
    /// memory using memory-mapped access.
    ///
    /// # Algorithm
    ///
    /// 1. Lock the function for mocking
    /// 2. Set up expected return value
    /// 3. Call the function with a test address
    /// 4. Verify the returned value matches the expected value
    #[test]
    #[mry::lock(read_flash_byte)]
    fn test_read_flash_byte() {
        // Set up mock to return test values
        mock_read_flash_byte(0x1000).returns(0x42);
        mock_read_flash_byte(0x1001).returns(0x55);
        
        // Call function and verify results
        let result1 = read_flash_byte(0x1000);
        let result2 = read_flash_byte(0x1001);
        
        // Verify expected results
        assert_eq!(result1, 0x42);
        assert_eq!(result2, 0x55);
        
        // Verify function was called exactly once with each address
        mock_read_flash_byte(0x1000).assert_called(1);
        mock_read_flash_byte(0x1001).assert_called(1);
    }
    
    /// Tests the read_reset_sequence function when erased flash is present.
    ///
    /// This test verifies that read_reset_sequence returns RESET_SEQUENCE_CLEAR (0)
    /// when flash contains erased value (0xFF).
    ///
    /// # Algorithm
    ///
    /// 1. Set up ADR_RESET_CNT_IDX to point to a valid location
    /// 2. Set up read_flash_byte to return 0xFF (erased)
    /// 3. Call read_reset_sequence()
    /// 4. Verify it returns RESET_SEQUENCE_CLEAR
    #[test]
    #[mry::lock(read_flash_byte)]
    fn test_read_reset_sequence_when_erased() {
        // Set up initial state
        ADR_RESET_CNT_IDX.set(10);
        
        // Set up read_flash_byte to return 0xFF (erased value)
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 10).returns(0xFF);
        
        // Call function under test
        let result = read_reset_sequence();
        
        // Verify results
        assert_eq!(result, RESET_SEQUENCE_CLEAR);
        
        // Verify read_flash_byte was called with the correct address
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 10).assert_called(1);
    }

    /// Tests the read_reset_sequence function when a valid bitmask sequence is in progress.
    ///
    /// This test verifies that read_reset_sequence correctly converts flash bitmask
    /// values back to step numbers using the new algorithm.
    ///
    /// # Algorithm
    ///
    /// 1. Set up ADR_RESET_CNT_IDX to point to a valid location
    /// 2. Set up read_flash_byte to return a valid bitmask value (e.g., 0xF8 = step 3)
    /// 3. Call read_reset_sequence()
    /// 4. Verify it returns the correct step number
    #[test]
    #[mry::lock(read_flash_byte)]
    fn test_read_reset_sequence_with_bitmask_sequence() {
        // Set up initial state (sequence index at position 10)
        ADR_RESET_CNT_IDX.set(10);
        
        // Set up read_flash_byte to return step 3 bitmask value
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 10).returns(ResetStep::Step3.flash_value()); // 0xF8
        
        // Call function under test
        let result = read_reset_sequence();
        
        // Verify results - should return step 3
        assert_eq!(result, 3);
        
        // Verify read_flash_byte was called with the correct address
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 10).assert_called(1);
    }

    /// Tests the read_reset_sequence function when an invalidated sequence is present.
    ///
    /// This test verifies that read_reset_sequence returns RESET_SEQUENCE_CLEAR
    /// when flash contains an invalidated sequence (bit 6 cleared).
    ///
    /// # Algorithm
    ///
    /// 1. Set up ADR_RESET_CNT_IDX to point to a valid location
    /// 2. Set up read_flash_byte to return an invalidated value (e.g., 0xB8 = invalidated step 3)
    /// 3. Call read_reset_sequence()
    /// 4. Verify it returns RESET_SEQUENCE_CLEAR
    #[test]
    #[mry::lock(read_flash_byte)]
    fn test_read_reset_sequence_with_invalidated_sequence() {
        // Set up initial state
        ADR_RESET_CNT_IDX.set(10);
        
        // Set up read_flash_byte to return invalidated step 3
        let invalidated_step3 = ResetStep::invalidate_flash_value(ResetStep::Step3.flash_value()); // 0xB8
        let expected_addr = FLASH_ADR_RESET_CNT + 10;
        mock_read_flash_byte(expected_addr).returns(invalidated_step3);
        
        // Call function under test
        let result = read_reset_sequence();
        
        // Verify results - should return clear since sequence is invalidated
        assert_eq!(result, RESET_SEQUENCE_CLEAR);
        
        // Verify read_flash_byte was called with the correct address
        mock_read_flash_byte(expected_addr).assert_called(1);
    }
    
    /// Tests the write_reset_sequence function for a normal reset step with bitmask approach.
    ///
    /// This test verifies that write_reset_sequence correctly:
    /// - Reads current flash value and applies the step's bitmask
    /// - Writes the new bitmask value to the same flash location
    /// - Does not increment index for normal steps (only for factory reset)
    ///
    /// # Algorithm
    ///
    /// 1. Set up initial state with current flash containing step 1 (0xFE)
    /// 2. Set up mocks for read and write operations
    /// 3. Call write_reset_sequence with step 2
    /// 4. Verify correct bitmask operation (0xFE & STEP_2_MASK = 0xFC)
    /// 5. Verify flash_write_page was called with correct value
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page, flash_erase_sector)]
    fn test_write_reset_sequence_bitmask_step() {
        // Set up initial state
        ADR_RESET_CNT_IDX.set(10);
        let current_addr = FLASH_ADR_RESET_CNT + 10;
        
        // Mock current flash value as step 1 (0xFE)
        mock_read_flash_byte(current_addr).returns(ResetStep::Step1.flash_value()); // 0xFE
        
        // Set up expectations
        mock_flash_write_page(current_addr, 1, mry::Any).returns(());
        mock_flash_erase_sector(mry::Any).returns(());
        
        // Call function under test with step 2
        write_reset_sequence(2);
        
        // Verify index was NOT incremented (only happens for factory reset)
        assert_eq!(ADR_RESET_CNT_IDX.get(), 10);
        
        // Verify flash_write_page was called at same address
        mock_flash_write_page(current_addr, 1, mry::Any).assert_called(1);
        
        // Verify flash_erase_sector was not called
        mock_flash_erase_sector(mry::Any).assert_called(0);
    }

    /// Tests the write_reset_sequence function when writing factory reset flag.
    ///
    /// This test verifies that write_reset_sequence correctly:
    /// - Applies factory reset bitmask to current value
    /// - Increments address index after writing factory reset flag
    /// - Handles sector cleanup if approaching end
    ///
    /// # Algorithm
    ///
    /// 1. Set up initial state with step 5 completed (0xE0)
    /// 2. Call write_reset_sequence with FACTORY_RESET_FLAG
    /// 3. Verify factory reset bitmask is applied and index incremented
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page, flash_erase_sector)]
    fn test_write_reset_sequence_factory_reset_flag() {
        // Set up initial state
        ADR_RESET_CNT_IDX.set(100);
        let current_addr = FLASH_ADR_RESET_CNT + 100;
        
        // Mock current flash value as step 5 (0xE0)
        mock_read_flash_byte(current_addr).returns(ResetStep::Step5.flash_value()); // 0xE0
        
        // Set up expectations
        mock_flash_write_page(current_addr, 1, mry::Any).returns(());
        mock_flash_erase_sector(mry::Any).returns(());
        
        // Call function under test with factory reset flag
        write_reset_sequence(FACTORY_RESET_FLAG);
        
        // Verify index was incremented (moves to next sequence location)
        assert_eq!(ADR_RESET_CNT_IDX.get(), 101);
        
        // Verify flash_write_page was called
        mock_flash_write_page(current_addr, 1, mry::Any).assert_called(1);
        
        // Verify flash_erase_sector was not called (not near sector end)
        mock_flash_erase_sector(mry::Any).assert_called(0);
    }
    
    /// Tests the write_reset_sequence function when writing factory reset flag near sector end.
    ///
    /// This test verifies that write_reset_sequence correctly:
    /// - Detects when approaching the end of a sector after writing factory reset flag
    /// - Erases the sector and resets the index in this case
    /// - Properly handles the bitmask write before sector cleanup
    ///
    /// # Algorithm
    ///
    /// 1. Set up initial state near the end of the sector
    /// 2. Set up flash read/write mocks for factory reset flag write
    /// 3. Call write_reset_sequence with FACTORY_RESET_FLAG
    /// 4. Verify bitmask was applied and sector cleanup occurred
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page, flash_erase_sector)]
    fn test_write_reset_sequence_near_sector_end() {
        // Set up initial state near the end of the sector
        ADR_RESET_CNT_IDX.set(4095);
        let current_addr = FLASH_ADR_RESET_CNT + 4095;
        
        // Mock current flash value as step 5 (0xE0)
        mock_read_flash_byte(current_addr).returns(ResetStep::Step5.flash_value()); // 0xE0
        
        // Set up expectations
        mock_flash_write_page(current_addr, 1, mry::Any).returns(());
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).returns(());
        
        // Call function under test with factory reset flag
        write_reset_sequence(FACTORY_RESET_FLAG);
        
        // Verify index was reset to 0 after sector erase (4095 + 1 = 4096 triggers cleanup)
        assert_eq!(ADR_RESET_CNT_IDX.get(), 0);
        
        // Verify flash_write_page was called first (bitmask operation)
        mock_flash_write_page(current_addr, 1, mry::Any).assert_called(1);
        
        // Verify flash_erase_sector was called for cleanup
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).assert_called(1);
    }
    
    /// Tests the clear_reset_sequence function with single-bit invalidation.
    ///
    /// This test verifies that clear_reset_sequence correctly:
    /// - Uses single-bit invalidation (clears bit 6) instead of writing full 0x00
    /// - Reads current flash value and applies invalidation mask
    /// - Writes the invalidated value back to flash
    ///
    /// # Algorithm
    ///
    /// 1. Set up ADR_RESET_CNT_IDX and mock current flash value  
    /// 2. Set up expectations for read and write operations
    /// 3. Call clear_reset_sequence()
    /// 4. Verify the invalidation mask was applied correctly
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page, flash_erase_sector)]
    fn test_clear_reset_sequence() {
        // Set up initial state
        ADR_RESET_CNT_IDX.set(100);
        let current_addr = FLASH_ADR_RESET_CNT + 100;
        
        // Mock current flash value as step 3 (0xF8)
        mock_read_flash_byte(current_addr).returns(ResetStep::Step3.flash_value()); // 0xF8
        
        // Set up expectations
        mock_flash_write_page(current_addr, 1, mry::Any).returns(());
        mock_flash_erase_sector(mry::Any).returns(());
        
        // Call function under test
        clear_reset_sequence();
        
        // Verify flash_write_page was called with invalidation applied
        // The value should be ResetStep::Step3.flash_value() & INVALIDATION_MASK = 0xF8 & 0xBF = 0xB8
        mock_flash_write_page(current_addr, 1, mry::Any).assert_called(1);
        
        // Verify no sector erase was triggered
        mock_flash_erase_sector(mry::Any).assert_called(0);
        
        // Verify index was incremented
        assert_eq!(ADR_RESET_CNT_IDX.get(), 101);
    }
    
    /// Tests the init_reset_sequence function.
    ///
    /// This test verifies that init_reset_sequence correctly:
    /// - Scans through flash memory to find the latest valid sequence entry
    /// - Skips invalidated sequences (bit 6 cleared)
    /// - Updates ADR_RESET_CNT_IDX with the position of the latest entry
    /// - Updates RESET_CNT with the step value of the latest entry
    ///
    /// # Algorithm
    ///
    /// 1. Set up read_flash_byte to return specific bitmask values at different addresses
    /// 2. Include invalidated sequences to test they are skipped
    /// 3. Call init_reset_sequence()
    /// 4. Verify ADR_RESET_CNT_IDX and RESET_CNT were updated correctly
    #[test]
    #[mry::lock(read_flash_byte)]
    fn test_init_reset_sequence() {
        // Initial state
        ADR_RESET_CNT_IDX.set(0);
        RESET_CNT.set(0);
        
        // Set up read_flash_byte to handle different addresses using a closure
        mock_read_flash_byte(mry::Any).returns_with(|addr: u32| {
            match addr {
                // Place a valid step 2 sequence at index 50
                addr if addr == FLASH_ADR_RESET_CNT + 50 => ResetStep::Step2.flash_value(), // 0xFC
                
                // Place an invalidated step 3 at index 100 (should be skipped)
                addr if addr == FLASH_ADR_RESET_CNT + 100 => ResetStep::invalidate_flash_value(ResetStep::Step3.flash_value()), // 0xB8
                
                // Place a valid step 4 sequence at index 200 (latest valid)
                addr if addr == FLASH_ADR_RESET_CNT + 200 => ResetStep::Step4.flash_value(), // 0xF0
                
                // Place the factory reset flag at index 300 (highest priority)
                addr if addr == FLASH_ADR_RESET_CNT + 300 => ResetStep::FactoryReset.flash_value(), // 0x60
                
                // Default return value for all other addresses (erased)
                _ => 0xFF
            }
        });
        
        // Call function under test
        init_reset_sequence();
        
        // Verify ADR_RESET_CNT_IDX was updated to the position of the factory reset flag
        // which has the highest priority and appears last in the scan
        assert_eq!(ADR_RESET_CNT_IDX.get(), 300);
        
        // Verify RESET_CNT was updated with the correct step value (factory reset flag)
        assert_eq!(RESET_CNT.get(), FACTORY_RESET_FLAG);
    }
    
    /// Tests the init_reset_sequence function with a mix of valid, invalid, and erased entries.
    ///
    /// This test verifies that init_reset_sequence correctly:
    /// - Finds the latest valid sequence entry among mixed entries
    /// - Skips invalidated and erased entries
    /// - Sets ADR_RESET_CNT_IDX to the position of the latest valid entry
    /// - Updates RESET_CNT with the step value of the latest valid entry
    ///
    /// # Algorithm
    ///
    /// 1. Set up read_flash_byte to return specific values for mixed entries
    /// 2. Call init_reset_sequence()
    /// 3. Verify ADR_RESET_CNT_IDX and RESET_CNT were updated to the latest valid entry
    #[test]
    #[mry::lock(read_flash_byte)]
    fn test_init_reset_sequence_mixed_valid_invalid() {
        // Test initialization with mix of valid, invalid, and erased entries
        
        // Set up a complex scenario in flash
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 0).returns(0xFF); // Erased
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 1).returns(0xFE); // Valid Step1
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 2).returns(0xBE); // Invalidated Step1
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 3).returns(0xFC); // Valid Step2
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 4).returns(0x55); // Corrupted
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 5).returns(0xF8); // Valid Step3 (latest)
        
        // Set up remaining addresses as erased
        for i in 6..4096 {
            mock_read_flash_byte(FLASH_ADR_RESET_CNT + i).returns(0xFF);
        }
        
        // Call function under test
        init_reset_sequence();
        
        // Verify latest valid entry was found (index 5, Step3)
        assert_eq!(ADR_RESET_CNT_IDX.get(), 5);
        assert_eq!(RESET_CNT.get(), 3); // Step3
    }
    
    /// Tests the init_reset_sequence function when all entries are invalid or erased.
    ///
    /// This test verifies that init_reset_sequence correctly:
    /// - Defaults to index 0 and clear step when all entries are invalidated or erased
    /// - Skips over invalidated and erased entries
    /// - Sets ADR_RESET_CNT_IDX to 0
    /// - Sets RESET_CNT to 0 (clear step)
    ///
    /// # Algorithm
    ///
    /// 1. Set up read_flash_byte to return invalidated or erased values
    /// 2. Call init_reset_sequence()
    /// 3. Verify ADR_RESET_CNT_IDX is 0 and RESET_CNT is 0 (clear step)
    #[test]
    #[mry::lock(read_flash_byte)]
    fn test_init_reset_sequence_all_invalid() {
        // Test initialization when all entries are invalid or erased
        
        // Set up all entries as either erased or invalidated
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 0).returns(0xFF); // Erased
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 1).returns(0xBE); // Invalidated
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 2).returns(0xBC); // Invalidated
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 3).returns(0x55); // Corrupted
        
        // Set up remaining addresses as erased
        for i in 4..4096 {
            mock_read_flash_byte(FLASH_ADR_RESET_CNT + i).returns(0xFF);
        }
        
        // Call function under test
        init_reset_sequence();
        
        // Verify defaults were set (index 0, Clear step)
        assert_eq!(ADR_RESET_CNT_IDX.get(), 0);
        assert_eq!(RESET_CNT.get(), 0); // Clear
    }
    
    /// Tests the factory_reset_handle function when a factory reset flag is set.
    ///
    /// This test verifies that factory_reset_handle correctly:
    /// - Performs a factory reset when FACTORY_RESET_FLAG is detected
    /// - Disables interrupts during the reset operation
    /// - Reboots the device after reset
    ///
    /// # Algorithm
    ///
    /// 1. Lock dependencies for mocking
    /// 2. Set up read_reset_sequence to return FACTORY_RESET_FLAG
    /// 3. Set up expectations for factory_reset, irq_disable, etc.
    /// 4. Call factory_reset_handle()
    /// 5. Verify each step was performed in correct order
    #[test]
    #[mry::lock(read_reset_sequence, init_reset_sequence, factory_reset, 
                irq_disable, light_sw_reboot, app_mocker)]
    fn test_factory_reset_handle_with_reset_flag() {
        // Set up initial state
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);

        // Set up mocks
        mock_init_reset_sequence().returns(());
        mock_read_reset_sequence().returns(FACTORY_RESET_FLAG);
        mock_irq_disable().returns(0);
        mock_factory_reset().returns(());
        mock_light_sw_reboot().returns(());
        app.ota_manager.mock_rf_led_ota_ok().returns(());
        
        // Call function under test
        factory_reset_handle();
        
        // Verify correct sequence of operations
        mock_init_reset_sequence().assert_called(1);
        mock_read_reset_sequence().assert_called(1);
        mock_irq_disable().assert_called(1);
        mock_factory_reset().assert_called(1);
        mock_light_sw_reboot().assert_called(1);
    }
    
    /// Tests the factory_reset_handle function when starting a new sequence.
    ///
    /// This test verifies that factory_reset_handle correctly:
    /// - Initializes step 1 of the sequence when RESET_SEQUENCE_CLEAR is detected
    /// - Sets up the timing check for the first step
    /// - Records the first step in flash
    ///
    /// # Algorithm
    ///
    /// 1. Lock dependencies for mocking
    /// 2. Set up read_reset_sequence to return RESET_SEQUENCE_CLEAR
    /// 3. Set up expectations for write_reset_sequence
    /// 4. Call factory_reset_handle()
    /// 5. Verify global state was updated correctly
    /// 6. Verify write_reset_sequence was called with correct step
    #[test]
    #[mry::lock(read_reset_sequence, init_reset_sequence, write_reset_sequence)]
    fn test_factory_reset_handle_starting_sequence() {
        // Set up mocks
        mock_init_reset_sequence().returns(());
        mock_read_reset_sequence().returns(RESET_SEQUENCE_CLEAR);
        mock_write_reset_sequence(1).returns(());
        
        // Call function under test
        factory_reset_handle();
        
        // Verify correct sequence of operations
        mock_init_reset_sequence().assert_called(1);
        mock_read_reset_sequence().assert_called(1);
        mock_write_reset_sequence(1).assert_called(1);
        
        // Verify global state was updated correctly
        assert_eq!(RESET_CNT.get(), 1);
        assert_eq!(CLEAR_ST.get(), 1);
        assert_eq!(RESET_CHECK_TIME.get(), POWER_CYCLE_TIMING[0].0 as u32);
    }
    
    /// Tests the factory_reset_handle function when continuing an existing sequence.
    ///
    /// This test verifies that factory_reset_handle correctly:
    /// - Advances to the next step when a valid sequence step is detected
    /// - Sets up timing checks for the new step
    /// - Records the next step in flash
    ///
    /// # Algorithm
    ///
    /// 1. Lock dependencies for mocking
    /// 2. Set up read_reset_sequence to return a mid-sequence value (e.g., 2)
    /// 3. Set up expectations for write_reset_sequence
    /// 4. Call factory_reset_handle()
    /// 5. Verify global state was updated correctly for next step
    /// 6. Verify write_reset_sequence was called with correct next step
    #[test]
    #[mry::lock(read_reset_sequence, init_reset_sequence, write_reset_sequence)]
    fn test_factory_reset_handle_continuing_sequence() {
        // Set up mocks
        mock_init_reset_sequence().returns(());
        mock_read_reset_sequence().returns(2); // Mid-sequence
        mock_write_reset_sequence(3).returns(());
        
        // Call function under test
        factory_reset_handle();
        
        // Verify correct sequence of operations
        mock_init_reset_sequence().assert_called(1);
        mock_read_reset_sequence().assert_called(1);
        mock_write_reset_sequence(3).assert_called(1);
        
        // Verify global state was updated correctly for next step
        assert_eq!(RESET_CNT.get(), 3);
        assert_eq!(CLEAR_ST.get(), 1);
        
        // Timing index for step 3 (index 2) 
        let timing_idx = 2;
        assert_eq!(RESET_CHECK_TIME.get(), POWER_CYCLE_TIMING[timing_idx].0 as u32);
    }
    
    /// Tests the factory_reset_handle function when all sequence steps are completed.
    ///
    /// This test verifies that factory_reset_handle correctly:
    /// - Sets the factory reset flag when the final step is detected
    /// - Reboots the device to trigger the actual reset
    ///
    /// # Algorithm
    ///
    /// 1. Lock dependencies for mocking
    /// 2. Set up read_reset_sequence to return the final sequence step
    /// 3. Set up expectations for write_reset_sequence and light_sw_reboot
    /// 4. Call factory_reset_handle()
    /// 5. Verify factory reset flag was set and reboot was triggered
    #[test]
    #[mry::lock(read_reset_sequence, init_reset_sequence, write_reset_sequence, 
                light_sw_reboot)]
    fn test_factory_reset_handle_final_step() {
        // Set up mocks
        mock_init_reset_sequence().returns(());
        mock_read_reset_sequence().returns(POWER_CYCLE_COUNT); // Final step
        mock_write_reset_sequence(FACTORY_RESET_FLAG).returns(());
        mock_light_sw_reboot().returns(());
        
        // Call function under test
        factory_reset_handle();
        
        // Verify factory reset flag was set and reboot was triggered
        mock_init_reset_sequence().assert_called(1);
        mock_read_reset_sequence().assert_called(1);
        mock_write_reset_sequence(FACTORY_RESET_FLAG).assert_called(1);
        mock_light_sw_reboot().assert_called(1);
    }
    
    /// Tests the factory_reset_handle function when transitioning from Step5 to FactoryReset.
    ///
    /// This test verifies the specific code path where:
    /// - Current step is Step5 (step number 5) 
    /// - next_step becomes ResetStep::FactoryReset
    /// - The factory reset flag is written and device reboots
    ///
    /// This tests the branch at lines 555-560 where next_step == ResetStep::FactoryReset.
    ///
    /// # Algorithm
    ///
    /// 1. Mock read_reset_sequence to return step 5 (Step5)
    /// 2. Call factory_reset_handle()
    /// 3. Verify factory reset flag is written and reboot is triggered
    #[test]
    #[mry::lock(read_reset_sequence, init_reset_sequence, write_reset_sequence, 
                light_sw_reboot)]
    fn test_factory_reset_handle_step5_to_factory_reset_transition() {
        // Set up mocks
        mock_init_reset_sequence().returns(());
        mock_read_reset_sequence().returns(5); // Step5 (not POWER_CYCLE_COUNT which is also 5 but different code path)
        mock_write_reset_sequence(FACTORY_RESET_FLAG).returns(());
        mock_light_sw_reboot().returns(());
        
        // Call function under test
        factory_reset_handle();
        
        // Verify factory reset flag was set and reboot was triggered
        mock_init_reset_sequence().assert_called(1);
        mock_read_reset_sequence().assert_called(1);
        mock_write_reset_sequence(FACTORY_RESET_FLAG).assert_called(1);
        mock_light_sw_reboot().assert_called(1);
    }

    /// Tests the factory_reset_cnt_check function when no timing check is active.
    ///
    /// This test verifies that factory_reset_cnt_check correctly:
    /// - Exits early when CLEAR_ST is 0 (no timing check active)
    /// - Does not modify any state
    ///
    /// # Algorithm
    ///
    /// 1. Set up initial state with CLEAR_ST=0
    /// 2. Call factory_reset_cnt_check()
    /// 3. Verify function returns early without changing state
    #[test]
    fn test_factory_reset_cnt_check_no_active_check() {
        // Set up initial state
        CLEAR_ST.set(0); // No active timing check
        
        // Remember initial state to verify no changes
        let initial_reset_cnt = RESET_CNT.get();
        let initial_clear_st = CLEAR_ST.get();
        
        // Call function under test
        factory_reset_cnt_check();
        
        // Verify state remains unchanged
        assert_eq!(RESET_CNT.get(), initial_reset_cnt);
        assert_eq!(CLEAR_ST.get(), initial_clear_st);
    }
    
    /// Tests the factory_reset_cnt_check function when timing check state is invalid.
    ///
    /// This test verifies that factory_reset_cnt_check correctly:
    /// - Clears the sequence when RESET_CNT is 0 or beyond valid range
    /// - Resets CLEAR_ST to 0
    ///
    /// # Algorithm
    ///
    /// 1. Set up initial state with CLEAR_ST=1 and invalid RESET_CNT
    /// 2. Lock clear_reset_sequence for mocking
    /// 3. Call factory_reset_cnt_check()
    /// 4. Verify sequence was cleared and CLEAR_ST was reset
    #[test]
    #[mry::lock(clear_reset_sequence, clock_time_exceed)]
    fn test_factory_reset_cnt_check_invalid_reset_cnt() {
        // Set up mock - ensure clear_reset_sequence is called
        mock_clear_reset_sequence().returns(());
        mock_clock_time_exceed(mry::Any, mry::Any).returns(false); // Don't let time checks interfere
        
        // Set up initial state with active timing check but invalid RESET_CNT
        CLEAR_ST.set(1);
        RESET_CNT.set(POWER_CYCLE_COUNT + 1); // Invalid value (6, when max is 5)
        
        // Call function under test
        factory_reset_cnt_check();
        
        // Verify sequence was cleared and CLEAR_ST was reset
        mock_clear_reset_sequence().assert_called(1);
        assert_eq!(CLEAR_ST.get(), 0);
    }
    
    /// Tests the factory_reset_cnt_check function when min time has elapsed in state 1.
    ///
    /// This test verifies that factory_reset_cnt_check correctly:
    /// - Transitions from state 1 to state 2 when min time has elapsed
    /// - Updates RESET_CHECK_TIME with the max time for the current step
    ///
    /// # Algorithm
    ///
    /// 1. Set up initial state with CLEAR_ST=1, valid RESET_CNT
    /// 2. Set up clock_time_exceed to indicate min time has passed
    /// 3. Call factory_reset_cnt_check()
    /// 4. Verify state transition to CLEAR_ST=2 and updated time limit
    #[test]
    #[mry::lock(clock_time_exceed, clear_reset_sequence)]
    fn test_factory_reset_cnt_check_min_time_elapsed() {
        // Set up mocks
        mock_clock_time_exceed(mry::Any, mry::Any).returns(true); // Time has elapsed
        mock_clear_reset_sequence().returns(());
        
        // Set up initial state - first step, waiting for min time
        CLEAR_ST.set(1);
        RESET_CNT.set(1);
        RESET_CHECK_TIME.set(POWER_CYCLE_TIMING[0].0 as u32); // Min time for step 1
        
        // Call function under test
        factory_reset_cnt_check();
        
        // Verify state transition and updated time
        assert_eq!(CLEAR_ST.get(), 2);
        
        // Max time for step 1 is the second element of the tuple
        assert_eq!(RESET_CHECK_TIME.get(), POWER_CYCLE_TIMING[0].1 as u32);
        
        // Verify clear_reset_sequence was not called
        mock_clear_reset_sequence().assert_called(0);
    }
    
    /// Tests the factory_reset_cnt_check function when max time exceeded in state 2.
    ///
    /// This test verifies that factory_reset_cnt_check correctly:
    /// - Aborts the sequence when max time has been exceeded in state 2
    /// - Resets CLEAR_ST to 0
    ///
    /// # Algorithm
    ///
    /// 1. Set up initial state with CLEAR_ST=2, valid RESET_CNT
    /// 2. Set up clock_time_exceed to indicate max time has been exceeded
    /// 3. Call factory_reset_cnt_check()
    /// 4. Verify sequence was cleared and CLEAR_ST was reset
    #[test]
    #[mry::lock(clock_time_exceed, clear_reset_sequence)]
    fn test_factory_reset_cnt_check_max_time_exceeded() {
        // Set up mocks
        mock_clock_time_exceed(mry::Any, mry::Any).returns(true); // Time has been exceeded
        mock_clear_reset_sequence().returns(());
        
        // Set up initial state - first step, max time check
        CLEAR_ST.set(2);
        RESET_CNT.set(1); // Valid step number
        RESET_CHECK_TIME.set(POWER_CYCLE_TIMING[0].1 as u32); // Max time for step 1
        
        // Call function under test
        factory_reset_cnt_check();
        
        // Verify sequence was cleared and CLEAR_ST was reset
        mock_clear_reset_sequence().assert_called(1);
        assert_eq!(CLEAR_ST.get(), 0);
    }
    
    /// Tests the factory_reset function.
    ///
    /// This test verifies that factory_reset correctly:
    /// - Erases all configuration sectors in flash memory
    /// - Skips the reset counter sector initially
    /// - Erases the reset counter sector at the end
    ///
    /// # Algorithm
    ///
    /// 1. Lock flash_erase_sector for mocking
    /// 2. Call factory_reset()
    /// 3. Verify flash_erase_sector was called for all sectors
    #[test]
    #[mry::lock(flash_erase_sector)]
    fn test_factory_reset() {
        // Set up mock
        mock_flash_erase_sector(mry::Any).returns(());
        
        // Call function under test
        factory_reset();
        
        // Calculate number of sectors to erase
        let sector_count = (FLASH_ADR_PAR_MAX - CFG_SECTOR_ADR_MAC_CODE) / 4096;
        
        // Verify flash_erase_sector was called for each sector, including reset counter sector
        let mut calls = 0;
        
        // Count calls and check each sector address
        for i in 1..sector_count {
            let sector_addr = CFG_SECTOR_ADR_MAC_CODE + i * 0x1000;
            
            if sector_addr != FLASH_ADR_RESET_CNT {
                // Regular sectors called first
                mock_flash_erase_sector(sector_addr).assert_called(1);
                calls += 1;
            }
        }
        
        // Reset counter sector should be erased last
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).assert_called(1);
        calls += 1;
        
        // Verify total number of calls
        mock_flash_erase_sector(mry::Any).assert_called(calls);
    }
    
    /// Tests the kick_out function when mode is OutOfMesh.
    ///
    /// This test verifies that kick_out correctly:
    /// - Performs a factory reset
    /// - Stores the mesh LTK, password, and network name when OutOfMesh
    /// - Sets up proper pairing configuration
    /// - Signals completion via LED indicators
    ///
    /// # Algorithm
    ///
    /// 1. Lock required functions for mocking
    /// 2. Set up app mock for rf_led_ota_ok
    /// 3. Set up initial state with mesh credentials
    /// 4. Call kick_out with OutOfMesh reason
    /// 5. Verify flash operations were performed in correct order
    /// 6. Verify rf_led_ota_ok was called
    #[test]
    #[mry::lock(factory_reset, flash_write_page, encode_password, app_mocker)]
    fn test_kick_out_out_of_mesh() {
        // Set up initial state
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        
        // Set up mocks
        mock_factory_reset().returns(());
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).returns(());
        
        // Set up mock for password encoding
        let mut password_buffer = [0u8; 16];
        let encoded_password = [0x55u8; 16]; // Simple encoded pattern
        mock_encode_password(mry::Any).returns(encoded_password);
        app.ota_manager.mock_rf_led_ota_ok().returns(());
        
        // Set up test mesh LTK and enable mesh pairing
        *PAIR_CONFIG_MESH_LTK.lock() = [0xAA; 16];
        MESH_PAIR_ENABLE.set(true);
        
        // Call function under test
        kick_out(KickoutReason::OutOfMesh);
        
        // Verify factory_reset was called first
        mock_factory_reset().assert_called(1);
        
        // Verify mesh LTK was written to flash at offset +48
        mock_flash_write_page(FLASH_ADR_PAIRING + 48, 16, mry::Any).assert_called(1);
        
        // Verify encrypted password was written to flash at offset +32
        mock_encode_password(mry::Any).assert_called(1);
        mock_flash_write_page(FLASH_ADR_PAIRING + 32, 16, mry::Any).assert_called(1);
        
        // Verify network name was written to flash at offset +16
        mock_flash_write_page(FLASH_ADR_PAIRING + 16, 16, mry::Any).assert_called(1);
        
        // Verify pairing config flags were written at base address
        mock_flash_write_page(FLASH_ADR_PAIRING, 16, mry::Any).assert_called(1);
        
        // Verify GET_MAC_EN was set to true
        assert_eq!(GET_MAC_EN.get(), true);
        
        // Verify rf_led_ota_ok was called
        app.ota_manager.mock_rf_led_ota_ok().assert_called(1);
    }
    
    /// Tests the kick_out function when mode is DefaultName.
    ///
    /// This test verifies that kick_out correctly:
    /// - Performs a factory reset
    /// - Does not store any mesh credentials when reason is not OutOfMesh
    /// - Signals completion via LED indicators
    ///
    /// # Algorithm
    ///
    /// 1. Lock required functions for mocking
    /// 2. Set up app mock for rf_led_ota_ok
    /// 3. Call kick_out with DefaultName reason
    /// 4. Verify only factory_reset was called, no flash writes
    /// 5. Verify rf_led_ota_ok was called
    #[test]
    #[mry::lock(factory_reset, flash_write_page, app_mocker)]
    fn test_kick_out_default_name() {
        // Set up initial state
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        
        // Set up mocks
        mock_factory_reset().returns(());
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).returns(());
        app.ota_manager.mock_rf_led_ota_ok().returns(());
        
        // Call function under test
        kick_out(KickoutReason::DefaultName);
        
        // Verify factory_reset was called
        mock_factory_reset().assert_called(1);
        
        // Verify no flash writes were performed
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).assert_called(0);
        
        // Verify rf_led_ota_ok was called
        app.ota_manager.mock_rf_led_ota_ok().assert_called(1);
    }
    
    /// Tests the kick_out function when mode is ModeMax.
    ///
    /// This test verifies that kick_out correctly:
    /// - Performs a factory reset
    /// - Does not store any mesh credentials when reason is not OutOfMesh
    /// - Signals completion via LED indicators
    ///
    /// # Algorithm
    ///
    /// 1. Lock required functions for mocking
    /// 2. Set up app mock for rf_led_ota_ok
    /// 3. Call kick_out with ModeMax reason
    /// 4. Verify only factory_reset was called, no flash writes
    /// 5. Verify rf_led_ota_ok was called
    #[test]
    #[mry::lock(factory_reset, app_mocker)]
    fn test_kick_out_mode_max() {
        // Test kick_out with ModeMax reason
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        
        mock_factory_reset().returns(());
        app.ota_manager.mock_rf_led_ota_ok().returns(());
        
        // Call function under test with ModeMax
        kick_out(KickoutReason::ModeMax);
        
        // Verify factory reset was called but no special mesh handling
        mock_factory_reset().assert_called(1);
        app.ota_manager.mock_rf_led_ota_ok().assert_called(1);
        
        // Should not have saved any mesh data for ModeMax
        // (no mesh-specific mocks should be called)
    }

    /// Tests for ResetStep enum methods that lack direct coverage
    #[test]
    fn test_reset_step_flash_value() {
        // Test all flash values match expected bitmask results
        assert_eq!(ResetStep::Clear.flash_value(), 0xFF);
        assert_eq!(ResetStep::Step1.flash_value(), 0xFE);
        assert_eq!(ResetStep::Step2.flash_value(), 0xFC);
        assert_eq!(ResetStep::Step3.flash_value(), 0xF8);
        assert_eq!(ResetStep::Step4.flash_value(), 0xF0);
        assert_eq!(ResetStep::Step5.flash_value(), 0xE0);
        assert_eq!(ResetStep::FactoryReset.flash_value(), 0x60);
    }
    
    #[test]
    fn test_reset_step_apply_to() {
        // Test bitmask application works correctly
        assert_eq!(ResetStep::Step1.apply_to(0xFF), 0xFE); // Clear bit 0
        assert_eq!(ResetStep::Step2.apply_to(0xFE), 0xFC); // Clear bit 1 from previous
        assert_eq!(ResetStep::Step3.apply_to(0xFC), 0xF8); // Clear bit 2 from previous
        assert_eq!(ResetStep::Step4.apply_to(0xF8), 0xF0); // Clear bit 3 from previous
        assert_eq!(ResetStep::Step5.apply_to(0xF0), 0xE0); // Clear bit 4 from previous
        assert_eq!(ResetStep::FactoryReset.apply_to(0xE0), 0x60); // Clear bit 7 from previous
        
        // Test applying to corrupted values
        assert_eq!(ResetStep::Step1.apply_to(0x00), 0x00); // Already cleared
        assert_eq!(ResetStep::Step2.apply_to(0x55), 0x55); // 0x55 & 0xFD = 0x55 (bit 1 already 0)
    }
    
    #[test]
    fn test_reset_step_is_sequence_complete() {
        // Test sequence completion detection
        assert!(!ResetStep::Clear.is_sequence_complete());
        assert!(!ResetStep::Step1.is_sequence_complete());
        assert!(!ResetStep::Step2.is_sequence_complete());
        assert!(!ResetStep::Step3.is_sequence_complete());
        assert!(!ResetStep::Step4.is_sequence_complete());
        assert!(!ResetStep::Step5.is_sequence_complete());
        assert!(ResetStep::FactoryReset.is_sequence_complete());
    }
    
    #[test]
    fn test_reset_step_next() {
        // Test sequence progression
        assert_eq!(ResetStep::Clear.next(), Some(ResetStep::Step1));
        assert_eq!(ResetStep::Step1.next(), Some(ResetStep::Step2));
        assert_eq!(ResetStep::Step2.next(), Some(ResetStep::Step3));
        assert_eq!(ResetStep::Step3.next(), Some(ResetStep::Step4));
        assert_eq!(ResetStep::Step4.next(), Some(ResetStep::Step5));
        assert_eq!(ResetStep::Step5.next(), Some(ResetStep::FactoryReset));
        assert_eq!(ResetStep::FactoryReset.next(), None);
    }
    
    #[test]
    fn test_reset_step_timing_window() {
        // Test timing window retrieval
        assert_eq!(ResetStep::Clear.timing_window(), None);
        assert_eq!(ResetStep::Step1.timing_window(), Some(POWER_CYCLE_TIMING[0]));
        assert_eq!(ResetStep::Step2.timing_window(), Some(POWER_CYCLE_TIMING[1]));
        assert_eq!(ResetStep::Step3.timing_window(), Some(POWER_CYCLE_TIMING[2]));
        assert_eq!(ResetStep::Step4.timing_window(), Some(POWER_CYCLE_TIMING[3]));
        assert_eq!(ResetStep::Step5.timing_window(), Some(POWER_CYCLE_TIMING[4]));
        assert_eq!(ResetStep::FactoryReset.timing_window(), None);
    }
    
    #[test]
    fn test_reset_step_invalidate_flash_value() {
        // Test invalidation bit clearing (bit 6)
        assert_eq!(ResetStep::invalidate_flash_value(0xFF), 0xBF); // 11111111 → 10111111
        assert_eq!(ResetStep::invalidate_flash_value(0xFE), 0xBE); // 11111110 → 10111110
        assert_eq!(ResetStep::invalidate_flash_value(0xFC), 0xBC); // 11111100 → 10111100
        assert_eq!(ResetStep::invalidate_flash_value(0xF8), 0xB8); // 11111000 → 10111000
        assert_eq!(ResetStep::invalidate_flash_value(0xF0), 0xB0); // 11110000 → 10110000
        assert_eq!(ResetStep::invalidate_flash_value(0xE0), 0xA0); // 11100000 → 10100000
        assert_eq!(ResetStep::invalidate_flash_value(0x60), 0x20); // 01100000 → 00100000
        
        // Test already invalidated values remain invalidated
        assert_eq!(ResetStep::invalidate_flash_value(0xBF), 0xBF); // Already invalidated
    }
    
    #[test]
    fn test_reset_step_is_flash_value_invalidated() {
        // Test invalidation detection (bit 6 cleared, not erased/clear)
        assert!(!ResetStep::is_flash_value_invalidated(0xFF)); // Erased state
        assert!(!ResetStep::is_flash_value_invalidated(0x00)); // Clear state
        assert!(!ResetStep::is_flash_value_invalidated(0xFE)); // Valid step 1
        assert!(!ResetStep::is_flash_value_invalidated(0xFC)); // Valid step 2
        assert!(!ResetStep::is_flash_value_invalidated(0xF8)); // Valid step 3
        assert!(!ResetStep::is_flash_value_invalidated(0xF0)); // Valid step 4
        assert!(!ResetStep::is_flash_value_invalidated(0xE0)); // Valid step 5
        assert!(!ResetStep::is_flash_value_invalidated(0x60)); // Valid factory reset
        
        // Test invalidated values are detected
        assert!(ResetStep::is_flash_value_invalidated(0xBE)); // Invalidated step 1
        assert!(ResetStep::is_flash_value_invalidated(0xBC)); // Invalidated step 2
        assert!(ResetStep::is_flash_value_invalidated(0xB8)); // Invalidated step 3
        assert!(ResetStep::is_flash_value_invalidated(0xB0)); // Invalidated step 4
        assert!(ResetStep::is_flash_value_invalidated(0xA0)); // Invalidated step 5
        assert!(ResetStep::is_flash_value_invalidated(0x20)); // Invalidated factory reset
        
        // Test arbitrary invalidated values
        assert!(ResetStep::is_flash_value_invalidated(0x30)); // Bit 6 cleared, not erased/clear
        assert!(ResetStep::is_flash_value_invalidated(0x01)); // Bit 6 cleared, not erased/clear
    }
    
    #[test]
    fn test_reset_step_from_flash_value_edge_cases() {
        // Test known valid values
        assert_eq!(ResetStep::from_flash_value(0xFF), ResetStep::Clear);
        assert_eq!(ResetStep::from_flash_value(0xFE), ResetStep::Step1);
        assert_eq!(ResetStep::from_flash_value(0xFC), ResetStep::Step2);
        assert_eq!(ResetStep::from_flash_value(0xF8), ResetStep::Step3);
        assert_eq!(ResetStep::from_flash_value(0xF0), ResetStep::Step4);
        assert_eq!(ResetStep::from_flash_value(0xE0), ResetStep::Step5);
        assert_eq!(ResetStep::from_flash_value(0x60), ResetStep::FactoryReset);
        
        // Test invalidated values return Clear
        assert_eq!(ResetStep::from_flash_value(0xBE), ResetStep::Clear); // Invalidated step 1
        assert_eq!(ResetStep::from_flash_value(0xBC), ResetStep::Clear); // Invalidated step 2
        assert_eq!(ResetStep::from_flash_value(0xB8), ResetStep::Clear); // Invalidated step 3
        assert_eq!(ResetStep::from_flash_value(0xA0), ResetStep::Clear); // Invalidated step 5
        
        // Test corrupted/invalid values return Clear
        assert_eq!(ResetStep::from_flash_value(0x00), ResetStep::Clear); // All bits cleared
        assert_eq!(ResetStep::from_flash_value(0x01), ResetStep::Clear); // Arbitrary invalid
        assert_eq!(ResetStep::from_flash_value(0x55), ResetStep::Clear); // Arbitrary invalid
        assert_eq!(ResetStep::from_flash_value(0xAA), ResetStep::Clear); // Arbitrary invalid
    }
    
    #[test]
    fn test_reset_step_try_from_and_from_conversion() {
        // Test valid conversions
        assert_eq!(ResetStep::try_from(0), Ok(ResetStep::Clear));
        assert_eq!(ResetStep::try_from(1), Ok(ResetStep::Step1));
        assert_eq!(ResetStep::try_from(2), Ok(ResetStep::Step2));
        assert_eq!(ResetStep::try_from(3), Ok(ResetStep::Step3));
        assert_eq!(ResetStep::try_from(4), Ok(ResetStep::Step4));
        assert_eq!(ResetStep::try_from(5), Ok(ResetStep::Step5));
        assert_eq!(ResetStep::try_from(0x80), Ok(ResetStep::FactoryReset));
        
        // Test invalid conversions
        assert_eq!(ResetStep::try_from(6), Err(ResetStepError::InvalidStepNumber));
        assert_eq!(ResetStep::try_from(255), Err(ResetStepError::InvalidStepNumber));
        
        // Test From implementation (round trip)
        assert_eq!(u8::from(ResetStep::Clear), 0);
        assert_eq!(u8::from(ResetStep::Step1), 1);
        assert_eq!(u8::from(ResetStep::Step2), 2);
        assert_eq!(u8::from(ResetStep::Step3), 3);
        assert_eq!(u8::from(ResetStep::Step4), 4);
        assert_eq!(u8::from(ResetStep::Step5), 5);
        assert_eq!(u8::from(ResetStep::FactoryReset), 0x80);
    }

    #[test]
    #[mry::lock(read_reset_sequence, init_reset_sequence, clear_reset_sequence, write_reset_sequence)]
    fn test_factory_reset_handle_invalid_state() {
        // Test handling of invalid step numbers beyond valid range
        mock_init_reset_sequence().returns(());
        mock_read_reset_sequence().returns(255); // Invalid step number
        mock_clear_reset_sequence().returns(());
        mock_write_reset_sequence(1).returns(()); // Step1 when Clear->Step1 transition
        
        // Call function under test
        factory_reset_handle();
        
        // Verify sequence initialization occurred (255 becomes Clear, Clear starts Step1)
        mock_init_reset_sequence().assert_called(1);
        mock_read_reset_sequence().assert_called(1);
        mock_write_reset_sequence(1).assert_called(1);
    }
    
    #[test]
    #[mry::lock(clear_reset_sequence)]
    fn test_factory_reset_cnt_check_invalid_clear_state() {
        // Test handling of invalid CLEAR_ST values
        CLEAR_ST.set(99); // Invalid clear state
        RESET_CNT.set(3); // Valid reset count
        mock_clear_reset_sequence().returns(());
        
        // Call function under test
        factory_reset_cnt_check();
        
        // Verify invalid clear state triggers sequence clearing
        mock_clear_reset_sequence().assert_called(1);
        assert_eq!(CLEAR_ST.get(), 0); // Should be reset
    }
    
    #[test]
    #[mry::lock(clear_reset_sequence)]
    fn test_factory_reset_cnt_check_step_without_timing() {
        // Test handling of step that has no timing window (edge case)
        CLEAR_ST.set(1);
        RESET_CNT.set(0); // Clear step which has no timing window
        
        // Ensure clear_reset_sequence is called
        mock_clear_reset_sequence().returns(());
        
        // Call function under test
        factory_reset_cnt_check();
        
        // Verify steps without timing windows trigger sequence clearing
        mock_clear_reset_sequence().assert_called(1);
        assert_eq!(CLEAR_ST.get(), 0);
    }
    
    /// Tests the write_reset_sequence function when no change is needed.
    ///
    /// This test verifies that write_reset_sequence correctly skips the write operation
    /// if the flash already contains the correct value for the current step.
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page)]
    fn test_write_reset_sequence_no_change_needed() {
        // Test when flash already has the correct value (no write needed)
        let current_addr = FLASH_ADR_RESET_CNT + ADR_RESET_CNT_IDX.get();
        
        // Set up mock to return value that won't change with Step2 bitmask
        mock_read_flash_byte(current_addr).returns(0xFC); // Step2 value
        
        // Set up mock for flash_write_page but it should not be called
        mock_flash_write_page(current_addr, 1, mry::Any).returns(());
        
        // Call function under test with Step2 (should not write since value is already correct)
        write_reset_sequence(2);
        
        // Verify read was called but write was not (no change needed)
        mock_read_flash_byte(current_addr).assert_called(1);
        mock_flash_write_page(current_addr, 1, mry::Any).assert_called(0);
    }
    
    /// Tests the clear_reset_sequence function when flash is already erased.
    ///
    /// This test verifies that clear_reset_sequence correctly skips the write operation
    /// if the flash is already in the erased state (0xFF).
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page, flash_erase_sector)]
    fn test_clear_reset_sequence_already_erased() {
        // Test clearing when flash is already erased (no write needed)
        ADR_RESET_CNT_IDX.set(0); // Start at index 0
        let current_addr = FLASH_ADR_RESET_CNT + 0;
        
        mock_read_flash_byte(current_addr).returns(0xFF); // Already erased
        mock_flash_write_page(current_addr, 1, mry::Any).returns(());
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).returns(());
        
        // Call function under test
        clear_reset_sequence();
        
        // Verify read was called but write was not (already erased)
        mock_read_flash_byte(current_addr).assert_called(1);
        mock_flash_write_page(current_addr, 1, mry::Any).assert_called(0);
        
        // Verify index was incremented (from 0 to 1)
        assert_eq!(ADR_RESET_CNT_IDX.get(), 1);
        
        // Verify no sector erase was called (not at boundary)
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).assert_called(0);
    }

    #[test]
    #[mry::lock(read_flash_byte, flash_write_page, flash_erase_sector)]
    fn test_write_reset_sequence_exactly_at_sector_boundary() {
        // Test behavior when exactly at sector boundary (4096)
        ADR_RESET_CNT_IDX.set(4095); // At the last valid position
        
        let current_addr = FLASH_ADR_RESET_CNT + 4095;
        mock_read_flash_byte(current_addr).returns(ResetStep::Step5.flash_value()); // 0xE0
        mock_flash_write_page(current_addr, 1, mry::Any).returns(());
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).returns(());
        
        // Write factory reset flag (which should trigger sector management)
        write_reset_sequence(FACTORY_RESET_FLAG);
        
        // Verify factory reset was written
        mock_flash_write_page(current_addr, 1, mry::Any).assert_called(1);
        
        // Verify index was incremented to 4096, which triggers reset to 0 after sector erase
        assert_eq!(ADR_RESET_CNT_IDX.get(), 0);
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).assert_called(1);
    }
    
    #[test]
    #[mry::lock(read_flash_byte, flash_write_page, flash_erase_sector)]
    fn test_clear_reset_sequence_exactly_at_sector_boundary() {
        // Test clear behavior when exactly at sector boundary
        ADR_RESET_CNT_IDX.set(4095); // At the last valid position
        
        let current_addr = FLASH_ADR_RESET_CNT + 4095;
        mock_read_flash_byte(current_addr).returns(ResetStep::Step3.flash_value()); // 0xF8
        mock_flash_write_page(current_addr, 1, mry::Any).returns(());
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).returns(());
        
        // Clear the sequence
        clear_reset_sequence();
        
        // Verify invalidation was written
        mock_flash_write_page(current_addr, 1, mry::Any).assert_called(1);
        
        // Verify index wrapped and sector was erased
        assert_eq!(ADR_RESET_CNT_IDX.get(), 0); // Reset to 0
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).assert_called(1);
    }
    
    /// Tests the factory_reset sector preservation.
    ///
    /// This test verifies that the reset counter sector is preserved during the main
    /// factory reset process, and only erased at the end.
    #[test]
    #[mry::lock(flash_erase_sector)]
    fn test_factory_reset_sector_preservation() {
        // Set up mock
        mock_flash_erase_sector(mry::Any).returns(());
        
        // Call factory reset
        factory_reset();
        
        // Verify reset counter sector was erased last
        let last_call_addr = FLASH_ADR_RESET_CNT;
        mock_flash_erase_sector(last_call_addr).assert_called(1);
    }
}
