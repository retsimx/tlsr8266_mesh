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

/// Number of sequential power cycle patterns defined in the timing sequence array
const POWER_CYCLE_COUNT: u8 = 5;

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
const POWER_CYCLE_TIMING: [u8; (POWER_CYCLE_COUNT * 2) as usize] = [
    0, 3, // Device must be turned off within 3 seconds of turning on
    0, 3, // Device must be turned off within 3 seconds of turning on
    0, 3, // Device must be turned off within 3 seconds of turning on
    3, 30, // Device must be on for 3-30 seconds before turning off
    3, 30, // Device must be on for 3-30 seconds before turning off
];

/// Flag indicating reset sequence should be cleared (restarted)
pub const RESET_SEQUENCE_CLEAR: u8 = 0;

/// Flag indicating a factory reset should be performed
pub const FACTORY_RESET_FLAG: u8 = 0x80;

/// Maximum address in flash memory
pub const FLASH_ADR_PAR_MAX: u32 = 0x80000;

/// Base address for MAC and configuration data in 512KB flash
pub const CFG_ADR_MAC_512K_FLASH: u32 = 0x76000;

/// Sector address for MAC code configuration
pub const CFG_SECTOR_ADR_MAC_CODE: u32 = CFG_ADR_MAC_512K_FLASH;

/// Write the current step in the reset sequence to flash memory
///
/// # Parameters
///
/// * `step` - The current step in the reset sequence (0-based index)
///
/// # Notes
///
/// * A value of RESET_SEQUENCE_CLEAR (0) indicates the sequence is cleared
/// * A value of FACTORY_RESET_FLAG (0x80) indicates factory reset should be performed
/// * Values 1-POWER_CYCLE_COUNT indicate the current step in the sequence
#[cfg_attr(test, mry::mry)]
fn write_reset_sequence(step: u8) {
    // Create a buffer with the step value
    let data = [step];
    
    // Increment the index for the next write
    // This creates a sequential record of reset steps in flash
    ADR_RESET_CNT_IDX.inc();
    
    // Check if we're approaching the end of the sector and need to clean up
    if ADR_RESET_CNT_IDX.get() >= 4080 {
        // Erase the sector and reset index to start of sector
        flash_erase_sector(FLASH_ADR_RESET_CNT);
        ADR_RESET_CNT_IDX.set(0);
    }
    
    // Write the step value to flash at the current index
    flash_write_page(
        FLASH_ADR_RESET_CNT + ADR_RESET_CNT_IDX.get(),
        1,
        data.as_ptr(),
    );
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
    // If there are no entries yet, return 0 (sequence clear)
    if ADR_RESET_CNT_IDX.get() == 0 {
        return RESET_SEQUENCE_CLEAR;
    }
    
    // Read the latest entry from flash using direct memory access
    read_flash_byte(FLASH_ADR_RESET_CNT + ADR_RESET_CNT_IDX.get())
}

/// Clear the reset sequence by writing RESET_SEQUENCE_CLEAR
///
/// This aborts any in-progress factory reset detection sequence
#[cfg_attr(test, mry::mry)]
fn clear_reset_sequence() {
    write_reset_sequence(RESET_SEQUENCE_CLEAR);
}

/// Initialize the reset sequence tracking
///
/// # Notes
///
/// * Scans flash memory to find the latest reset sequence step
/// * Sets up ADR_RESET_CNT_IDX for subsequent operations
#[cfg_attr(test, mry::mry)]
fn init_reset_sequence() {
    // Start scanning from the beginning of the sector
    ADR_RESET_CNT_IDX.set(0);
    
    // Scan through flash sector to find the latest reset sequence entry
    let mut latest_idx = 0;
    let mut latest_value = RESET_SEQUENCE_CLEAR;
    
    // Scan through the entire 4KB sector
    for i in 0..4096 {
        // Read byte directly from flash memory 
        let value = read_flash_byte(FLASH_ADR_RESET_CNT + i);
        
        // If we find a non-zero value, it's a potential sequence step
        if value != RESET_SEQUENCE_CLEAR {
            // Check if the value is valid (between 1 and POWER_CYCLE_COUNT or FACTORY_RESET_FLAG)
            if value <= POWER_CYCLE_COUNT || value == FACTORY_RESET_FLAG {
                latest_idx = i;
                latest_value = value;
            }
        }
    }
    
    // Set the index to the latest valid entry
    ADR_RESET_CNT_IDX.set(latest_idx);
    
    // Update the global reset counter state
    RESET_CNT.set(latest_value);
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
    let current_step = read_reset_sequence();
    
    // Check if we should perform a factory reset
    if current_step == FACTORY_RESET_FLAG {
        // Disable interrupts to ensure the reset process isn't interrupted
        irq_disable();
        
        // Execute the factory reset to erase configuration data
        factory_reset();
        
        // Signal completion via LED indicators
        app().ota_manager.rf_led_ota_ok();
        
        // Reboot the device to apply the factory defaults
        light_sw_reboot();
    } else if current_step == RESET_SEQUENCE_CLEAR {
        // No reset sequence in progress, initialize step 1
        // Start checking the first timing window
        RESET_CNT.set(1);
        CLEAR_ST.set(1);
        RESET_CHECK_TIME.set(POWER_CYCLE_TIMING[0] as u32);
        
        // Record the first step in flash
        write_reset_sequence(1);
    } else if current_step > 0 && current_step < POWER_CYCLE_COUNT {
        // Reset sequence in progress, move to the next step
        // Start checking the corresponding timing window
        let next_step = current_step + 1;
        RESET_CNT.set(next_step);
        CLEAR_ST.set(1);
        
        // Set up timing check for this step
        let timing_idx = (next_step - 1) as usize * 2;
        RESET_CHECK_TIME.set(POWER_CYCLE_TIMING[timing_idx] as u32);
        
        // Record the next step in flash
        write_reset_sequence(next_step);
    } else if current_step == POWER_CYCLE_COUNT {
        // All steps completed successfully, set the factory reset flag
        write_reset_sequence(FACTORY_RESET_FLAG);
        
        // Reboot to trigger the actual factory reset
        light_sw_reboot();
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
    let current_step = RESET_CNT.get();
    if current_step == 0 || current_step > POWER_CYCLE_COUNT {
        // Invalid state, clear the sequence
        clear_reset_sequence();
        CLEAR_ST.set(0);
        return;
    }
    
    // Calculate the timing index for this step
    let timing_idx = (current_step - 1) as usize * 2;
    
    // State 1: Check if minimum time has elapsed
    if CLEAR_ST.get() == 1 && clock_time_exceed(0, RESET_CHECK_TIME.get() * 1000 * 1000) {
        // Minimum time elapsed, move to state 2
        CLEAR_ST.set(2);
        
        // Set up the maximum time limit
        RESET_CHECK_TIME.set(POWER_CYCLE_TIMING[timing_idx + 1] as u32);
    }
    
    // State 2: Check if maximum time has been exceeded
    else if CLEAR_ST.get() == 2 && clock_time_exceed(0, RESET_CHECK_TIME.get() * 1000 * 1000) {
        // Maximum time exceeded, abort the sequence
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
    
    /// Tests the read_reset_sequence function for the case where no reset sequence has been started.
    ///
    /// This test verifies that read_reset_sequence returns RESET_SEQUENCE_CLEAR (0)
    /// when the reset sequence index is zero, indicating no sequence is in progress.
    ///
    /// # Algorithm
    ///
    /// 1. Set up ADR_RESET_CNT_IDX.get() to return 0
    /// 2. Call read_reset_sequence()
    /// 3. Verify it returns RESET_SEQUENCE_CLEAR
    #[test]
    #[mry::lock(read_flash_byte)]
    fn test_read_reset_sequence_when_empty() {
        // Set up initial state
        ADR_RESET_CNT_IDX.set(0);
        
        // Call function under test
        let result = read_reset_sequence();
        
        // Verify results
        assert_eq!(result, RESET_SEQUENCE_CLEAR);
        
        // Verify read_flash_byte was not called (exits early due to index being 0)
        mock_read_flash_byte(mry::Any).assert_called(0);
    }

    /// Tests the read_reset_sequence function when a sequence is already in progress.
    ///
    /// This test verifies that read_reset_sequence correctly reads the reset sequence
    /// value from flash memory when a sequence has been started (index > 0).
    ///
    /// # Algorithm
    ///
    /// 1. Set up ADR_RESET_CNT_IDX.get() to return a non-zero value
    /// 2. Set up read_flash_byte to return a test value
    /// 3. Call read_reset_sequence()
    /// 4. Verify it returns the expected value from flash
    #[test]
    #[mry::lock(read_flash_byte)]
    fn test_read_reset_sequence_with_existing_sequence() {
        // Set up initial state (sequence index at position 10)
        ADR_RESET_CNT_IDX.set(10);
        
        // Set up read_flash_byte to return a test value
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 10).returns(3);
        
        // Call function under test
        let result = read_reset_sequence();
        
        // Verify results
        assert_eq!(result, 3);
        
        // Verify read_flash_byte was called with the correct address
        mock_read_flash_byte(FLASH_ADR_RESET_CNT + 10).assert_called(1);
    }
    
    /// Tests the write_reset_sequence function for a normal reset step.
    ///
    /// This test verifies that write_reset_sequence correctly:
    /// - Writes the provided step value to flash
    /// - Increments the reset counter index
    /// - Doesn't trigger sector erase for normal index values
    ///
    /// # Algorithm
    ///
    /// 1. Set up initial state with a valid index
    /// 2. Set up flash_write_page to verify it's called with correct params
    /// 3. Call write_reset_sequence with a test step value
    /// 4. Verify the function increments the index
    /// 5. Verify flash_write_page was called with correct params
    #[test]
    #[mry::lock(flash_write_page, flash_erase_sector)]
    fn test_write_reset_sequence_normal_case() {
        // Set up initial state
        ADR_RESET_CNT_IDX.set(100);
        
        // Set up expectations
        mock_flash_write_page(FLASH_ADR_RESET_CNT + 101, 1, mry::Any).returns(());
        mock_flash_erase_sector(mry::Any).returns(());
        
        // Call function under test
        write_reset_sequence(2);
        
        // Verify index was incremented
        assert_eq!(ADR_RESET_CNT_IDX.get(), 101);
        
        // Verify flash_write_page was called with correct parameters
        mock_flash_write_page(FLASH_ADR_RESET_CNT + 101, 1, mry::Any).assert_called(1);
        
        // Verify flash_erase_sector was not called (we're not near sector end)
        mock_flash_erase_sector(mry::Any).assert_called(0);
    }
    
    /// Tests the write_reset_sequence function when approaching the end of a sector.
    ///
    /// This test verifies that write_reset_sequence correctly:
    /// - Detects when the counter is approaching the end of a sector (≥ 4080)
    /// - Erases the sector and resets the index in this case
    /// - Writes the step value to the beginning of the fresh sector
    ///
    /// # Algorithm
    ///
    /// 1. Set up initial state with an index near sector end (4080)
    /// 2. Set up flash_erase_sector and flash_write_page expectations
    /// 3. Call write_reset_sequence
    /// 4. Verify sector was erased and index was reset to 0
    /// 5. Verify flash_write_page was called with new index of 0
    #[test]
    #[mry::lock(flash_write_page, flash_erase_sector)]
    fn test_write_reset_sequence_near_sector_end() {
        // Set up initial state near the end of the sector
        ADR_RESET_CNT_IDX.set(4080);
        
        // Set up expectations
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).returns(());
        mock_flash_write_page(FLASH_ADR_RESET_CNT, 1, mry::Any).returns(());
        
        // Call function under test
        write_reset_sequence(FACTORY_RESET_FLAG);
        
        // Verify index was reset to 0 after sector erase
        assert_eq!(ADR_RESET_CNT_IDX.get(), 0);
        
        // Verify flash_erase_sector was called with the correct sector address
        mock_flash_erase_sector(FLASH_ADR_RESET_CNT).assert_called(1);
        
        // Verify flash_write_page was called with reset index of 0
        mock_flash_write_page(FLASH_ADR_RESET_CNT, 1, mry::Any).assert_called(1);
    }
    
    /// Tests the clear_reset_sequence function.
    ///
    /// This test verifies that clear_reset_sequence correctly:
    /// - Writes the RESET_SEQUENCE_CLEAR (0) value to flash
    ///
    /// # Algorithm
    ///
    /// 1. Lock the write_reset_sequence function for mocking
    /// 2. Set up expectation that it's called with RESET_SEQUENCE_CLEAR
    /// 3. Call clear_reset_sequence()
    /// 4. Verify write_reset_sequence was called with correct parameter
    #[test]
    #[mry::lock(write_reset_sequence)]
    fn test_clear_reset_sequence() {
        // Set up expectations
        mock_write_reset_sequence(RESET_SEQUENCE_CLEAR).returns(());
        
        // Call function under test
        clear_reset_sequence();
        
        // Verify write_reset_sequence was called with RESET_SEQUENCE_CLEAR (0)
        mock_write_reset_sequence(RESET_SEQUENCE_CLEAR).assert_called(1);
    }
    
    /// Tests the init_reset_sequence function.
    ///
    /// This test verifies that init_reset_sequence correctly:
    /// - Scans through flash memory to find the latest valid sequence entry
    /// - Updates ADR_RESET_CNT_IDX with the position of the latest entry
    /// - Updates RESET_CNT with the value of the latest entry
    ///
    /// # Algorithm
    ///
    /// 1. Set up read_flash_byte to return specific values at different addresses
    /// 2. Call init_reset_sequence()
    /// 3. Verify ADR_RESET_CNT_IDX and RESET_CNT were updated correctly
    #[test]
    #[mry::lock(read_flash_byte)]
    fn test_init_reset_sequence() {
        // Initial state
        ADR_RESET_CNT_IDX.set(0);
        RESET_CNT.set(0);
        
        // Set up read_flash_byte to handle different addresses using a closure
        // This ensures more specific address matches are applied correctly
        mock_read_flash_byte(mry::Any).returns_with(|addr: u32| {
            match addr {
                // Place a valid reset sequence value (3) at index 100
                addr if addr == FLASH_ADR_RESET_CNT + 100 => 3,
                
                // Place an invalid value (too large) at index 200
                addr if addr == FLASH_ADR_RESET_CNT + 200 => 10,
                
                // Place the factory reset flag at index 300
                addr if addr == FLASH_ADR_RESET_CNT + 300 => FACTORY_RESET_FLAG,
                
                // Default return value for all other addresses
                _ => 0
            }
        });
        
        // Call function under test
        init_reset_sequence();
        
        // Verify ADR_RESET_CNT_IDX was updated to the position of the factory reset flag
        // which has the highest priority
        assert_eq!(ADR_RESET_CNT_IDX.get(), 300);
        
        // Verify RESET_CNT was updated with the value at that position
        assert_eq!(RESET_CNT.get(), FACTORY_RESET_FLAG);
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
        assert_eq!(RESET_CHECK_TIME.get(), POWER_CYCLE_TIMING[0] as u32);
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
        
        // Timing index for step 3 (index 2) starts at position 4 in the timing array
        let timing_idx = 4;
        assert_eq!(RESET_CHECK_TIME.get(), POWER_CYCLE_TIMING[timing_idx] as u32);
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
    #[mry::lock(clear_reset_sequence)]
    fn test_factory_reset_cnt_check_invalid_reset_cnt() {
        // Set up mock
        mock_clear_reset_sequence().returns(());
        
        // Set up initial state with active timing check but invalid RESET_CNT
        CLEAR_ST.set(1);
        RESET_CNT.set(POWER_CYCLE_COUNT + 1); // Invalid value
        
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
        RESET_CHECK_TIME.set(POWER_CYCLE_TIMING[0] as u32); // Min time for step 1
        
        // Call function under test
        factory_reset_cnt_check();
        
        // Verify state transition and updated time
        assert_eq!(CLEAR_ST.get(), 2);
        
        // Max time for step 1 is at index 1 in the timing array
        assert_eq!(RESET_CHECK_TIME.get(), POWER_CYCLE_TIMING[1] as u32);
        
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
        RESET_CNT.set(1);
        RESET_CHECK_TIME.set(POWER_CYCLE_TIMING[1] as u32); // Max time for step 1
        
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
    
    /// Tests the TryFrom implementation for KickoutReason.
    ///
    /// This test verifies that the TryFrom trait correctly:
    /// - Converts valid u32 values to KickoutReason enum values
    /// - Returns an error for invalid values
    ///
    /// # Algorithm
    ///
    /// 1. Test conversion of valid values (0, 1, 2)
    /// 2. Test conversion of an invalid value
    /// 3. Verify expected results in each case
    #[test]
    fn test_kickout_reason_try_from() {
        // Test valid conversions
        assert_eq!(KickoutReason::try_from(0), Ok(KickoutReason::OutOfMesh));
        assert_eq!(KickoutReason::try_from(1), Ok(KickoutReason::DefaultName));
        assert_eq!(KickoutReason::try_from(2), Ok(KickoutReason::ModeMax));
        
        // Test invalid conversion
        assert!(KickoutReason::try_from(3).is_err());
    }
}
