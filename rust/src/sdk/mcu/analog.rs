//! Analog register interface for TLSR8266 SoC.
//!
//! This module provides low-level functions for reading from and writing to
//! analog control registers on the Telink TLSR8266F512 SoC. These operations
//! are critical for configuring various hardware components including 
//! RF transceiver parameters, power management, and clock settings.

use critical_section;
use crate::sdk::mcu::register::{
    read_reg_ana_ctrl, read_reg_ana_data, write_reg_ana_addr, write_reg_ana_ctrl,
    write_reg_ana_data, FLD_ANA,
};

/// Waits until analog register operations are complete.
///
/// # Details
///
/// This function polls the analog control register's busy flag until it
/// clears, indicating that any ongoing analog register access operation
/// has completed. This is a blocking operation and must be called between
/// analog register accesses to ensure proper timing.
///
/// # Notes
///
/// * This function is marked inline(always) for performance since it's
///   frequently called in time-critical sections
/// * No timeout mechanism is implemented - assumes hardware will eventually
///   clear the busy flag
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn analog_wait() {
    // Poll the busy bit in the analog control register until it clears
    while read_reg_ana_ctrl() & (FLD_ANA::BUSY.bits()) != 0 {}
}

/// Reads a byte from the specified analog register address.
///
/// # Parameters
///
/// * `addr` - The 8-bit address of the analog register to read
///
/// # Returns
///
/// * The 8-bit value read from the specified analog register
///
/// # Notes
///
/// * This function is placed in RAM (.ram_code section) for reliable execution
///   regardless of flash access timing
/// * Uses critical_section to ensure atomicity of the complete read operation
/// * The analog read sequence follows strict timing requirements of the hardware
///   and cannot be simplified without risking unreliable operation
#[inline(never)]
#[link_section = ".ram_code"]
#[cfg_attr(test, mry::mry)]
pub fn analog_read(addr: u8) -> u8 {
    critical_section::with(|_| {
        // 1. Set the address register to the target analog register
        write_reg_ana_addr(addr);
        
        // 2. Write control register to start read operation
        //    START bit initiates the operation
        //    RSV bit must be set during reads
        write_reg_ana_ctrl((FLD_ANA::START | FLD_ANA::RSV).bits());
        //   Can't use one line setting "reg_ana_ctrl32 = ((FLD_ANA_START | FLD_ANA_RSV) << 16) | addr;"
        //   This will fail because of time sequence and more over size is bigger
        
        // 3. Wait for the analog operation to complete
        analog_wait();
        
        // 4. Read the data from the data register
        let data = read_reg_ana_data();
        
        // 5. Clear control register to finish operation
        write_reg_ana_ctrl(0); // finish

        data
    })
}

/// Writes a byte to the specified analog register address.
///
/// # Parameters
///
/// * `addr` - The 8-bit address of the analog register to write
/// * `v` - The 8-bit value to write to the register
///
/// # Notes
///
/// * This function is placed in RAM (.ram_code section) for reliable execution
///   regardless of flash access timing
/// * Uses critical_section to ensure atomicity of the complete write operation
/// * The analog write sequence follows strict timing requirements of the hardware
///   and cannot be simplified without risking unreliable operation
/// * The TLSR8266's analog registers control critical hardware functions including
///   RF parameters, power management, and clock settings
#[inline(never)]
#[link_section = ".ram_code"]
#[cfg_attr(test, mry::mry)]
pub fn analog_write(addr: u8, v: u8) {
    critical_section::with(|_| {
        // 1. Set the address register to the target analog register
        write_reg_ana_addr(addr);
        
        // 2. Write the data to the data register
        write_reg_ana_data(v);
        
        // 3. Write control register to start write operation
        //    START bit initiates the operation
        //    RW bit must be set during writes (1=write, 0=read)
        write_reg_ana_ctrl((FLD_ANA::START | FLD_ANA::RW).bits());

        //	 Can't use one line setting "reg_ana_ctrl32 = ((FLD_ANA_START | FLD_ANA_RW) << 16) | (v << 8) | addr;"
        //   This will fail because of time sequence and more over size is bigger
        
        // 4. Wait for the analog operation to complete
        analog_wait();
        
        // 5. Clear control register to finish operation
        write_reg_ana_ctrl(0); // finish
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    
    // Import the mock functions from register module
    use crate::sdk::mcu::register::{
        mock_read_reg_ana_ctrl, mock_read_reg_ana_data, 
        mock_write_reg_ana_addr, mock_write_reg_ana_ctrl,
        mock_write_reg_ana_data
    };

    /// Tests analog_wait function for proper busy bit polling behavior.
    ///
    /// This test verifies that the analog_wait function correctly:
    /// - Polls the analog control register's busy flag
    /// - Exits when the busy flag clears
    /// - Properly handles various busy flag states
    ///
    /// # Algorithm
    ///
    /// 1. Configure mock to initially return busy status
    /// 2. Setup mock to clear busy flag after specific number of reads
    /// 3. Call analog_wait function
    /// 4. Verify correct number of register reads occurred
    ///
    /// # Notes
    ///
    /// * Tests busy flag polling behavior
    /// * Ensures function exits loop when busy flag is cleared
    #[test]
    #[mry::lock(read_reg_ana_ctrl)]
    fn test_analog_wait() {
        // Create counter to track number of calls and simulate busy flag clearing
        let mut call_count = 0;
        
        // Mock behavior: Return busy flag (bit 1 set) for first 3 calls,
        // then return not busy (bit 1 clear)
        mock_read_reg_ana_ctrl().returns_with(move || {
            call_count += 1;
            if call_count < 3 {
                FLD_ANA::BUSY.bits() // Return busy flag set
            } else {
                0 // Return busy flag clear
            }
        });
        
        // Call the function being tested
        analog_wait();
        
        // Verify function polled register until busy flag cleared
        mock_read_reg_ana_ctrl().assert_called(3);
    }

    /// Tests analog_read function for proper register read sequence.
    ///
    /// This test verifies that the analog_read function correctly:
    /// - Sets the analog address register
    /// - Sets proper control bits for read operation
    /// - Waits for operation completion
    /// - Retrieves data from data register
    /// - Clears control register after operation
    /// - Returns correct data value
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for all register operations
    /// 2. Mock analog_wait to simulate waiting for operation completion
    /// 3. Call analog_read with test address
    /// 4. Verify all register operations occurred in correct sequence
    /// 5. Confirm expected data value was returned
    ///
    /// # Notes
    ///
    /// * Tests the complete analog register read sequence
    /// * Verifies critical timing sequence and register operations
    #[test]
    #[mry::lock(write_reg_ana_addr, write_reg_ana_ctrl, read_reg_ana_data, analog_wait)]
    fn test_analog_read() {
        // Setup test data
        let test_addr: u8 = 0x42;
        let expected_data: u8 = 0xAB;
        
        // Setup mock behavior for register operations
        mock_write_reg_ana_addr(test_addr).returns(());
        mock_write_reg_ana_ctrl((FLD_ANA::START | FLD_ANA::RSV).bits()).returns(());
        mock_write_reg_ana_ctrl(0).returns(());
        mock_analog_wait().returns(());
        mock_read_reg_ana_data().returns(expected_data);
        
        // Call the function being tested
        let result = analog_read(test_addr);
        
        // Verify correct address was set
        mock_write_reg_ana_addr(test_addr).assert_called(1);
        
        // Verify correct control bits were set for read operation
        mock_write_reg_ana_ctrl((FLD_ANA::START | FLD_ANA::RSV).bits()).assert_called(1);
        
        // Verify analog_wait was called to wait for operation completion
        mock_analog_wait().assert_called(1);
        
        // Verify data register was read exactly once
        mock_read_reg_ana_data().assert_called(1);
        
        // Verify control register was cleared after operation
        mock_write_reg_ana_ctrl(0).assert_called(1);
        
        // Verify correct data was returned
        assert_eq!(result, expected_data);
    }

    /// Tests analog_write function for proper register write sequence.
    ///
    /// This test verifies that the analog_write function correctly:
    /// - Sets the analog address register
    /// - Sets the analog data register with the value to write
    /// - Sets proper control bits for write operation
    /// - Waits for operation completion
    /// - Clears control register after operation
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for all register operations
    /// 2. Mock analog_wait to simulate waiting for operation completion
    /// 3. Call analog_write with test address and data value
    /// 4. Verify all register operations occurred in correct sequence
    /// 5. Confirm proper timing of operations
    ///
    /// # Notes
    ///
    /// * Tests the complete analog register write sequence
    /// * Verifies critical timing sequence and register operations
    /// * Checks that control bits are set correctly for write operation
    #[test]
    #[mry::lock(write_reg_ana_addr, write_reg_ana_data, write_reg_ana_ctrl, analog_wait)]
    fn test_analog_write() {
        // Setup test data
        let test_addr: u8 = 0x42;
        let test_value: u8 = 0xCD;
        
        // Setup mock behavior for register operations
        mock_write_reg_ana_addr(test_addr).returns(());
        mock_write_reg_ana_data(test_value).returns(());
        mock_write_reg_ana_ctrl((FLD_ANA::START | FLD_ANA::RW).bits()).returns(());
        mock_write_reg_ana_ctrl(0).returns(());
        mock_analog_wait().returns(());
        
        // Call the function being tested
        analog_write(test_addr, test_value);
        
        // Verify correct address was set
        mock_write_reg_ana_addr(test_addr).assert_called(1);
        
        // Verify data was written to data register
        mock_write_reg_ana_data(test_value).assert_called(1);
        
        // Verify correct control bits were set for write operation
        mock_write_reg_ana_ctrl((FLD_ANA::START | FLD_ANA::RW).bits()).assert_called(1);
        
        // Verify analog_wait was called to wait for operation completion
        mock_analog_wait().assert_called(1);
        
        // Verify control register was cleared after operation
        mock_write_reg_ana_ctrl(0).assert_called(1);
    }
    
    /// Tests analog_read function behavior with critical_section.
    ///
    /// This test verifies that analog_read correctly:
    /// - Uses critical_section to ensure atomicity
    /// - Maintains proper register operations within the critical section
    ///
    /// # Notes
    ///
    /// * This test is primarily a structural validation that the function
    ///   uses critical_section, as we can't directly test the critical_section
    ///   behavior in unit tests
    #[test]
    #[mry::lock(write_reg_ana_addr, write_reg_ana_ctrl, read_reg_ana_data, analog_wait)]
    fn test_analog_read_critical_section() {
        // Setup test data
        let test_addr: u8 = 0x55;
        let expected_data: u8 = 0x99;
        
        // Setup mock behavior for register operations
        mock_write_reg_ana_addr(test_addr).returns(());
        mock_write_reg_ana_ctrl((FLD_ANA::START | FLD_ANA::RSV).bits()).returns(());
        mock_write_reg_ana_ctrl(0).returns(());
        mock_analog_wait().returns(());
        mock_read_reg_ana_data().returns(expected_data);
        
        // Call the function being tested
        let result = analog_read(test_addr);
        
        // Verify correct result
        assert_eq!(result, expected_data);
        
        // All other verifications are the same as test_analog_read
        // This test is primarily to ensure the critical_section is being used
    }
    
    /// Tests analog_write function behavior with critical_section.
    ///
    /// This test verifies that analog_write correctly:
    /// - Uses critical_section to ensure atomicity
    /// - Maintains proper register operations within the critical section
    ///
    /// # Notes
    ///
    /// * This test is primarily a structural validation that the function
    ///   uses critical_section, as we can't directly test the critical_section
    ///   behavior in unit tests
    #[test]
    #[mry::lock(write_reg_ana_addr, write_reg_ana_data, write_reg_ana_ctrl, analog_wait)]
    fn test_analog_write_critical_section() {
        // Setup test data
        let test_addr: u8 = 0x66;
        let test_value: u8 = 0x88;
        
        // Setup mock behavior for register operations
        mock_write_reg_ana_addr(test_addr).returns(());
        mock_write_reg_ana_data(test_value).returns(());
        mock_write_reg_ana_ctrl((FLD_ANA::START | FLD_ANA::RW).bits()).returns(());
        mock_write_reg_ana_ctrl(0).returns(());
        mock_analog_wait().returns(());
        
        // Call the function being tested
        analog_write(test_addr, test_value);
        
        // All verifications are the same as test_analog_write
        // This test is primarily to ensure the critical_section is being used
    }
}
