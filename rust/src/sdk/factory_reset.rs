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

/// Number of sequential reset patterns defined in the timing sequence array
/// Must be less than 7 due to bit counting limitations in the reset counter
const SERIALS_CNT: u8 = 5;

/// Timing sequence for factory reset detection (in seconds)
/// Format: [min_time, max_time] pairs for each reset sequence step
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
const FACTORY_RESET_SERIALS: [u8; (SERIALS_CNT * 2) as usize] = [
    0, 3, // Device must be turned off within 3 seconds of turning on
    0, 3, // Device must be turned off within 3 seconds of turning on
    0, 3, // Device must be turned off within 3 seconds of turning on
    3, 30, // Device must be on for 3-30 seconds before turning off
    3, 30, // Device must be on for 3-30 seconds before turning off
];

/// Flag indicating reset count should be recounted (cleared)
pub const RESET_CNT_RECOUNT_FLAG: u8 = 0;

/// Flag indicating a factory reset should be performed
pub const RESET_FLAG: u8 = 0x80;

/// Maximum address in flash memory
pub const FLASH_ADR_PAR_MAX: u32 = 0x80000;

/// Base address for MAC and configuration data in 512KB flash
pub const CFG_ADR_MAC_512K_FLASH: u32 = 0x76000;

/// Sector address for MAC code configuration
pub const CFG_SECTOR_ADR_MAC_CODE: u32 = CFG_ADR_MAC_512K_FLASH;

/// Clean up the reset counter storage in flash when approaching capacity
///
/// This function erases the flash sector containing reset counters when the index
/// approaches the maximum safe value (3840), preventing overflow and ensuring
/// continuous operation of the factory reset detection mechanism.
///
/// # Notes
///
/// * Flash memory has limited write cycles, so we use a sequential write pattern
///   and only erase when necessary to extend flash lifespan
fn reset_cnt_clean() {
    // Check if we're still within safe range (less than 3840 bytes used in the 4KB sector)
    // This avoids unnecessary flash erase operations to extend flash lifespan
    if ADR_RESET_CNT_IDX.get() < 3840 {
        return;
    }
    
    // Erase the entire reset counter sector when we've nearly filled it
    flash_erase_sector(FLASH_ADR_RESET_CNT);
    
    // Reset index to 0 so we can start writing from the beginning of the freshly erased sector
    ADR_RESET_CNT_IDX.set(0);
}

/// Write a reset counter value to flash memory
///
/// # Parameters
///
/// * `cnt` - The reset counter value to write
///
/// # Notes
///
/// * Writes to the current index position within the reset counter flash sector
fn write_reset_cnt(cnt: u8) {
    // Create a single-byte array with the counter value
    let data = [cnt];
    
    // Write the counter value to the current position in the reset counter sector
    // Parameters:
    // - Address: Base reset counter address + current index offset
    // - Length: 1 byte
    // - Data: Pointer to the counter value
    flash_write_page(
        FLASH_ADR_RESET_CNT + ADR_RESET_CNT_IDX.get(),
        1,
        data.as_ptr(),
    );
}

/// Clear the reset counter by writing the RESET_CNT_RECOUNT_FLAG
///
/// This resets the device's tracking of power cycle sequences, effectively
/// canceling any in-progress factory reset detection sequence.
fn clear_reset_cnt() {
    // Write the reset flag (0) to flash to clear the power cycle tracking sequence
    // This cancels any in-progress factory reset detection sequence
    write_reset_cnt(RESET_CNT_RECOUNT_FLAG);
}

/// Find the current index position for reset counter operations
///
/// # Algorithm
///
/// 1. Start from the beginning of the reset counter sector
/// 2. Scan through memory looking for a non-zero value (not RESET_CNT_RECOUNT_FLAG)
/// 3. Validate the found value - if invalid patterns are detected, clear the counter
/// 4. Set the global ADR_RESET_CNT_IDX to the found position for future operations
/// 5. Perform cleanup if necessary to ensure flash sector has available space
///
/// # Notes
///
/// * Uses direct pointer manipulation for efficiency on limited hardware
/// * The flash memory approach allows the sequence to persist across power cycles
fn reset_cnt_get_idx() {
    // Get a raw pointer to the beginning of the reset counter flash sector
    let pf = FLASH_ADR_RESET_CNT as *const u8;
    
    // Start scanning from the beginning of the sector
    ADR_RESET_CNT_IDX.set(0);
    
    // Scan through the entire 4KB sector looking for a valid reset counter value
    while ADR_RESET_CNT_IDX.get() < 4096 {
        // Read the byte at the current index position
        // Using unsafe code for direct memory access to optimize performance on limited hardware
        let restcnt_bit = unsafe { *pf.offset(ADR_RESET_CNT_IDX.get() as isize) };
        
        // If the value is not the reset flag (0), we've found a potential counter value
        if restcnt_bit != RESET_CNT_RECOUNT_FLAG {
            // Validate the pattern - checking for invalid bit patterns
            // These patterns represent invalid states in our bit-based counter
            if ((!(BIT!(0)|BIT!(1)|BIT!(2)|BIT!(3))) as u8 == restcnt_bit)  // the fourth bit not valid
        	 || ((!(BIT!(0)|BIT!(1)|BIT!(2)|BIT!(3)|BIT!(4)|BIT!(5))) as u8 == restcnt_bit)  // the fifth bit not valid
            {
                // If an invalid pattern is detected, clear the counter to start fresh
                clear_reset_cnt();
            } else {
                // Valid pattern found - exit the loop and keep the current index
                break;
            }
        }
        // Move to the next byte in the flash sector
        ADR_RESET_CNT_IDX.inc();
    }

    // After finding the current position, check if we need to clean up the sector
    // This prevents overflow by erasing when we approach the end of the sector
    reset_cnt_clean();
}

/// Retrieve the current reset counter value from flash memory
///
/// # Returns
///
/// * The current reset counter bit pattern from flash memory
///
/// # Notes
///
/// * Reads the reset counter value at the current index position
/// * Updates the global RESET_CNT state with the retrieved value
/// * The bit pattern encodes the current state of reset sequence detection
fn get_reset_cnt_bit() -> u8 {
    // Create a buffer to hold the read data
    let mut data = [0];
    
    // Read one byte from the flash memory at the current index position
    // Parameters:
    // - Address: Base reset counter address + current index offset
    // - Length: 1 byte
    // - Buffer: Where to store the read value
    flash_read_page(
        FLASH_ADR_RESET_CNT + ADR_RESET_CNT_IDX.get(),
        1,
        data.as_mut_ptr(),
    );
    
    // Update the global reset counter state with the read value
    RESET_CNT.set(data[0]);
    
    // Return the counter value
    return RESET_CNT.get();
}

/// Increment the reset counter state based on bit pattern
///
/// # Algorithm
///
/// 1. Get the current reset counter bit pattern
/// 2. Find the first set bit in the pattern (from bit 0 to 7)
/// 3. Based on which bit is set, update the RESET_CNT global state:
///    - Bits 0-2: Set RESET_CNT to the bit position (0, 1, or 2)
///    - Bits 3-4: Set RESET_CNT to 3 (fourth state)
///    - Bits 5-6: Set RESET_CNT to 4 (fifth state)
/// 4. Clear the found bit in the pattern and write it back to flash
///
/// # Notes
///
/// * This implements a bit-shifting counter where each successful power cycle advances
///   to the next state by clearing one bit in sequence
/// * The counter is encoded as a bit pattern where '1' bits represent
///   remaining steps in the sequence
/// * Each power cycle must follow the timing window defined in FACTORY_RESET_SERIALS
///   for that particular step to advance the sequence
fn increase_reset_cnt() {
    // Retrieve the current reset counter bit pattern from flash
    let mut restcnt_bit = get_reset_cnt_bit();
    
    // Scan through all bits (0-7) to find the first '1' bit
    for i in 0..8 {
        // Check if the current bit position is set
        if restcnt_bit & BIT!(i) != 0 {
            // Update the RESET_CNT state based on which bit was found
            // This maps bit positions to logical states in the reset sequence
            if i < 3 {
                // First 3 bits (0,1,2) map directly to initial sequence steps
                RESET_CNT.set(i);
            } else if i < 5 {
                // Bits 3-4 both map to sequence state 3 (critical timing sequence)
                RESET_CNT.set(3);
            } else if i < 7 {
                // Bits 5-6 both map to sequence state 4 (critical timing sequence)
                RESET_CNT.set(4);
            }

            // Clear the found bit to mark this step as completed
            // This uses a bitwise operation to set only this bit to 0
            restcnt_bit &= !(BIT!(i));
            
            // Write the updated bit pattern back to flash
            // This preserves our position in the reset sequence
            write_reset_cnt(restcnt_bit);
            
            // Exit after handling the first set bit
            break;
        }
    }
}

/// Handle the factory reset process based on reset counter state
///
/// # Algorithm
///
/// 1. Determine the current reset counter index position
/// 2. Get the current reset counter value
/// 3. If the value equals RESET_FLAG (0x80), perform a factory reset:
///    - Disable interrupts to prevent interference
///    - Execute the factory reset (erase configuration)
///    - Signal completion via LEDs
///    - Reboot the device
/// 4. Otherwise, increment the reset counter to track sequence progress
///
/// # Notes
///
/// * This function should be called during device boot to check if the
///   factory reset sequence has been completed
pub fn factory_reset_handle() {
    // Find the current position in flash for reset counter operations
    reset_cnt_get_idx();
    
    // Get the current reset counter value from that position
    let restcnt_bit = get_reset_cnt_bit();
    
    // Check if the reset flag is set (0x80), indicating a factory reset is needed
    if restcnt_bit == RESET_FLAG {
        // Disable interrupts to ensure the reset process isn't interrupted
        // This is critical for flash operations to complete properly
        irq_disable();
        
        // Execute the factory reset to erase configuration data
        factory_reset();
        
        // Signal completion via LED indicators
        // This provides visual feedback that the reset was successful
        app().ota_manager.rf_led_ota_ok();
        
        // Reboot the device to apply the factory defaults
        light_sw_reboot();
    } else {
        // If we're not at the reset flag yet, increment the counter
        // This tracks progress through the power cycle sequence
        increase_reset_cnt();
    }
}

/// Check and manage factory reset counter state during device operation
///
/// # Algorithm
///
/// This implements a state machine to detect a specific power-cycle pattern:
/// 1. If CLEAR_ST is 0, no reset sequence is in progress
/// 2. If CLEAR_ST is 3, initialize the first timing window check:
///    - Set up the minimum power-on time from FACTORY_RESET_SERIALS
/// 3. If CLEAR_ST is 2 and the minimum time has elapsed:
///    - Move to state 1
///    - Set up the maximum power-on time from FACTORY_RESET_SERIALS
///    - If in critical sequence states (3 or 4), mark for counter increment
/// 4. If CLEAR_ST is 1 and the maximum time window has expired:
///    - Reset the state machine
///    - Mark for counter reset (sequence failed)
///
/// # Notes
///
/// * This function implements the timing verification logic for the factory reset sequence
/// * It validates each power cycle against specific time windows:
///   - Device must be on for at least FACTORY_RESET_SERIALS[n*2] seconds
///   - Device must be off before FACTORY_RESET_SERIALS[n*2+1] seconds elapse
/// * If any timing window is not followed correctly, the entire sequence is reset
/// * A complete sequence of correctly timed power cycles triggers the factory reset
pub fn factory_reset_cnt_check() {
    // These flags track whether we need to update the counter state
    let mut increase_cnt = false;  // Set if we need to advance to the next step
    let mut clear_cnt = false;     // Set if we need to abort the sequence
    
    // If CLEAR_ST is 0, no reset sequence is in progress, so exit early
    if CLEAR_ST.get() == 0 {
        return;
    }

    // State 3: Initialize the timing window
    if CLEAR_ST.get() == 3 {
        // Transition to state 2
        CLEAR_ST.dec();
        
        // Set up the minimum power-on time for this step of the sequence
        // This is the minimum time the device must remain on
        RESET_CHECK_TIME.set(FACTORY_RESET_SERIALS[RESET_CNT.get() as usize * 2] as u32);
    }

    // State 2: Check if the minimum time has elapsed
    if CLEAR_ST.get() == 2 && clock_time_exceed(0, RESET_CHECK_TIME.get() * 1000 * 1000) {
        // Minimum time has elapsed, transition to state 1
        CLEAR_ST.dec();
        
        // Set up the maximum power-on time for this step of the sequence
        // The device must be powered off before this time expires
        RESET_CHECK_TIME.set(FACTORY_RESET_SERIALS[RESET_CNT.get() as usize * 2 + 1] as u32);
        
        // For critical steps in the sequence (steps 3 and 4), mark for counter increment
        // These are the steps with specific timing requirements (3-30 seconds)
        if RESET_CNT.get() == 3 || RESET_CNT.get() == 4 {
            increase_cnt = true;
        }
    }

    // State 1: Check if the maximum time window has expired
    if CLEAR_ST.get() == 1 && clock_time_exceed(0, RESET_CHECK_TIME.get() * 1000 * 1000) {
        // Maximum time has elapsed - the user didn't power off in time
        // Reset the state machine by returning to state 0
        CLEAR_ST.set(0);
        
        // Mark for counter reset - the sequence has failed
        clear_cnt = true;
    }

    // Apply counter changes based on the flags set above
    if increase_cnt {
        // Advance to the next step in the sequence
        increase_reset_cnt();
    }

    if clear_cnt {
        // Abort the sequence and start over
        clear_reset_cnt();
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
#[derive(PartialEq)]
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
