use crate::sdk::mcu::register::{
    read_reg_master_spi_ctrl, read_reg_master_spi_data, write_reg_master_spi_ctrl,
    write_reg_master_spi_data, FLD_MASTER_SPI,
};

/// Sets the SPI chip select (CS) signal high, deactivating the current SPI device.
///
/// This function drives the CS pin high by writing the CS bit to the SPI control register.
/// In SPI protocol, a high CS signal indicates the end of a transaction and deselects
/// the peripheral device.
///
/// # Notes
///
/// * This function is marked as `inline(always)` to optimize for speed in time-critical
///   SPI transactions, as it's typically called in tight loops.
/// * For proper SPI operation, ensure mspi_high() is called after completing a transaction.
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn mspi_high() {
    write_reg_master_spi_ctrl(FLD_MASTER_SPI::CS.bits());
}

/// Sets the SPI chip select (CS) signal low, activating the target SPI device.
///
/// This function drives the CS pin low by clearing all bits in the SPI control register.
/// In SPI protocol, a low CS signal initiates a transaction and selects the peripheral
/// device for communication.
///
/// # Notes
///
/// * This function is marked as `inline(always)` to optimize for speed in time-critical
///   SPI transactions.
/// * Always call mspi_high() after completing the transaction to deselect the device.
/// * The TLSR8266 SPI peripheral uses active-low CS signals, which is the standard for
///   most SPI devices.
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn mspi_low() {
    write_reg_master_spi_ctrl(0);
}

/// Writes a byte to the SPI data register to be transmitted.
///
/// This function sends a single byte over the SPI bus by writing it to the SPI data register.
/// The byte will be shifted out on the MOSI (Master Out Slave In) line while simultaneously 
/// receiving a byte on the MISO (Master In Slave Out) line.
///
/// # Parameters
///
/// * `c` - The byte to be transmitted over SPI
///
/// # Notes
///
/// * This function only initiates the transmission; it does not wait for completion.
/// * After calling this function, you should call mspi_wait() to ensure the byte 
///   has been fully transmitted before proceeding with further operations.
/// * The byte written will be shifted out MSB (Most Significant Bit) first, which
///   is the standard SPI protocol.
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn mspi_write(c: u8) {
    write_reg_master_spi_data(c);
}

/// Waits until the SPI peripheral completes the current operation.
///
/// This function implements a busy-waiting loop that continuously polls the SPI control
/// register's BUSY flag until it is cleared, indicating that the SPI operation has completed.
/// This synchronization point ensures that all SPI operations have finished before proceeding.
///
/// # Algorithm
///
/// 1. Reads the SPI control register
/// 2. Masks it with the BUSY bit flag
/// 3. If the result is non-zero (BUSY flag set), continues looping
/// 4. Once the BUSY flag is clear, returns to the caller
///
/// # Notes
///
/// * This is a blocking function that will not return until the SPI operation completes.
/// * In time-critical applications, consider the execution time impact of this busy-wait.
/// * The function is inlined to minimize function call overhead in timing-sensitive operations.
/// * No timeout mechanism is implemented - in case of hardware failure, this could deadlock.
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn mspi_wait() {
    while read_reg_master_spi_ctrl() & (FLD_MASTER_SPI::BUSY.bits()) != 0 {}
}

/// Directly writes a value to the SPI control register.
///
/// This low-level function provides direct access to the SPI control register,
/// allowing fine-grained control over the SPI peripheral's behavior. It can be used
/// to set multiple control bits in a single operation.
///
/// # Parameters
///
/// * `c` - The byte value to write to the SPI control register
///
/// # Notes
///
/// * Use with caution as improper values may disrupt SPI operation.
/// * Common control bits include:
///   - CS bit: Controls the chip select signal
///   - Other control bits specific to the TLSR8266 SPI implementation
/// * For standard SPI operations, prefer the higher-level functions like
///   mspi_high() and mspi_low() which handle specific control operations.
/// * This function is marked as `inline(always)` to maintain consistent timing
///   in time-critical SPI operations.
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn mspi_ctrl_write(c: u8) {
    write_reg_master_spi_ctrl(c);
}

/// Reads a byte from the SPI data register that was received during the last transmission.
///
/// This function retrieves the byte that was shifted in on the MISO (Master In Slave Out) line
/// during the most recent SPI data transfer. It directly accesses the SPI data register.
///
/// # Returns
///
/// * The byte value read from the SPI data register
///
/// # Notes
///
/// * This function should typically be called after mspi_write() and mspi_wait() to retrieve
///   the response byte from a SPI slave device.
/// * The function only returns the current value in the data register; it does not initiate
///   any new SPI transfers.
/// * For full-duplex operation, each byte sent (via mspi_write) results in one byte received,
///   which can be accessed via this function.
/// * This function is marked as `inline(always)` to minimize function call overhead in
///   timing-sensitive SPI operations.
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn mspi_get() -> u8 {
    read_reg_master_spi_data()
}

/// Performs a complete SPI read operation by sending a dummy byte and reading the response.
///
/// This function combines three operations into a convenient single call:
/// 1. Writes a dummy byte (0) to generate clock signals
/// 2. Waits for the SPI transmission to complete
/// 3. Reads and returns the received byte
///
/// # Returns
///
/// * The byte received from the SPI slave device
///
/// # Algorithm
///
/// 1. Send a dummy byte (0) to generate SPI clock pulses
/// 2. Wait for the transmission to complete by polling the BUSY flag
/// 3. Read the received data from the SPI data register
/// 4. Return the received byte to the caller
///
/// # Notes
///
/// * This function is useful for reading data from SPI devices where the
///   content of the transmitted byte doesn't matter (only clock pulses are needed).
/// * The SPI peripheral operates in full-duplex mode, meaning data is both sent and
///   received simultaneously. This function focuses on the receive aspect.
/// * This is a blocking function that will not return until the SPI read completes.
/// * This function is marked as `inline(always)` for performance in timing-critical
///   operations.
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn mspi_read() -> u8 {
    mspi_write(0); // Send dummy byte to generate clock pulses
    mspi_wait();   // Wait for transmission to complete
    return mspi_get(); // Read and return the received byte
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdk::mcu::register::{
        mock_read_reg_master_spi_ctrl, mock_read_reg_master_spi_data, 
        mock_write_reg_master_spi_ctrl, mock_write_reg_master_spi_data,
    };
    
    /// Tests that mspi_high sets the SPI CS high by writing the correct bit to the control register.
    #[test]
    #[mry::lock(write_reg_master_spi_ctrl)]
    fn test_mspi_high() {
        // Set up expectations
        mock_write_reg_master_spi_ctrl(FLD_MASTER_SPI::CS.bits()).returns(());
        
        // Execute function under test
        mspi_high();
        
        // Verify the control register was written correctly
        mock_write_reg_master_spi_ctrl(FLD_MASTER_SPI::CS.bits()).assert_called(1);
    }
    
    /// Tests that mspi_low clears the SPI CS bit by writing 0 to the control register.
    #[test]
    #[mry::lock(write_reg_master_spi_ctrl)]
    fn test_mspi_low() {
        // Set up expectations
        mock_write_reg_master_spi_ctrl(0).returns(());
        
        // Execute function under test
        mspi_low();
        
        // Verify the control register was written correctly
        mock_write_reg_master_spi_ctrl(0).assert_called(1);
    }
    
    /// Tests that mspi_write correctly writes data to the SPI data register.
    #[test]
    #[mry::lock(write_reg_master_spi_data)]
    fn test_mspi_write() {
        // Set up expectations for different data values
        mock_write_reg_master_spi_data(0x42).returns(());
        mock_write_reg_master_spi_data(0xFF).returns(());
        
        // Execute function under test with different values
        mspi_write(0x42);
        mspi_write(0xFF);
        
        // Verify each call was made correctly
        mock_write_reg_master_spi_data(0x42).assert_called(1);
        mock_write_reg_master_spi_data(0xFF).assert_called(1);
    }
    
    /// Tests that mspi_wait polls the busy bit until the operation completes.
    #[test]
    #[mry::lock(read_reg_master_spi_ctrl)]
    fn test_mspi_wait() {
        // Use a static mutable counter to ensure proper tracking across function calls
        static mut CALL_COUNT: usize = 0;
        
        // Reset the counter at the start of the test
        unsafe { CALL_COUNT = 0 };
        
        // Set up the behavior of the mocked function using our static counter
        mock_read_reg_master_spi_ctrl().returns_with(|| {
            // Safely increment the counter - this is within test code only
            let count = unsafe { 
                CALL_COUNT += 1; 
                CALL_COUNT 
            };
            
            if count <= 2 {
                FLD_MASTER_SPI::BUSY.bits() // First two calls: busy
            } else {
                0 // Third call: no longer busy
            }
        });
        
        // Execute function under test
        mspi_wait();
        
        // Verify control register was read exactly 3 times
        mock_read_reg_master_spi_ctrl().assert_called(3);
    }
    
    /// Tests that mspi_ctrl_write correctly writes to the SPI control register.
    #[test]
    #[mry::lock(write_reg_master_spi_ctrl)]
    fn test_mspi_ctrl_write() {
        // Set up expectations for different control values
        mock_write_reg_master_spi_ctrl(0x0A).returns(());
        mock_write_reg_master_spi_ctrl(0x01).returns(());
        
        // Execute function under test with different values
        mspi_ctrl_write(0x0A);
        mspi_ctrl_write(0x01);
        
        // Verify each call was made correctly
        mock_write_reg_master_spi_ctrl(0x0A).assert_called(1);
        mock_write_reg_master_spi_ctrl(0x01).assert_called(1);
    }
    
    /// Tests that mspi_get correctly reads data from the SPI data register.
    #[test]
    #[mry::lock(read_reg_master_spi_data)]
    fn test_mspi_get() {
        // Set up expectations for data register reads
        mock_read_reg_master_spi_data().returns(0x42);
        
        // Execute function under test
        let result = mspi_get();
        
        // Verify result and call count
        assert_eq!(result, 0x42, "mspi_get should return the value from the data register");
        mock_read_reg_master_spi_data().assert_called(1);
    }
    
    /// Tests that mspi_read performs the complete read sequence:
    /// 1. Writes a dummy byte to generate clock
    /// 2. Waits for operation to complete
    /// 3. Gets the received data
    #[test]
    #[mry::lock(mspi_write, mspi_wait, mspi_get)]
    fn test_mspi_read() {
        // Set up expectations
        mock_mspi_write(0).returns(());
        mock_mspi_wait().returns(());
        mock_mspi_get().returns(0xA5);
        
        // Execute function under test
        let result = mspi_read();
        
        // Verify result and calls
        assert_eq!(result, 0xA5, "mspi_read should return the value from mspi_get");
        mock_mspi_write(0).assert_called(1);
        mock_mspi_wait().assert_called(1);
        mock_mspi_get().assert_called(1);
    }
    
    /// Tests the integration between mspi functions in a typical SPI transaction.
    #[test]
    #[mry::lock(
        mspi_high, mspi_low, mspi_write, 
        mspi_wait, mspi_get
    )]
    fn test_spi_transaction() {
        // Set up for a typical SPI command-response transaction
        // For functions called multiple times, use counters and closures
        
        mock_mspi_high().returns(());
        mock_mspi_low().returns(());
        
        // Handle different mspi_write parameters
        mock_mspi_write(0x9F).returns(());  // JEDEC ID command
        
        mock_mspi_write(0).returns(());
        
        mock_mspi_wait().returns(());
        
        // Track mspi_get calls with different return values
        let mut get_count = 0;
        mock_mspi_get().returns_with(move || {
            get_count += 1;
            match get_count {
                1 => 0xEF, // First byte of JEDEC ID (manufacturer)
                _ => 0x40, // Second byte of JEDEC ID (device type)
            }
        });
        
        // Execute a sequence that resembles a typical SPI transaction
        mspi_high();               // Ensure CS starts high
        mspi_low();                // Begin transaction by activating CS
        mspi_write(0x9F);          // Send JEDEC ID command
        mspi_wait();               // Wait for command to send
        
        // Read first byte of response
        mspi_write(0);             // Send dummy byte to clock in data
        mspi_wait();               // Wait for clock cycles
        let byte1 = mspi_get();    // Read received data
        
        // Read second byte of response
        mspi_write(0);             // Send dummy byte to clock in data
        mspi_wait();               // Wait for clock cycles
        let byte2 = mspi_get();    // Read received data
        
        mspi_high();               // End transaction by deactivating CS
        
        // Verify results
        assert_eq!(byte1, 0xEF, "First JEDEC ID byte should be 0xEF");
        assert_eq!(byte2, 0x40, "Second JEDEC ID byte should be 0x40");
        
        // Verify call sequence
        mock_mspi_high().assert_called(2);
        mock_mspi_low().assert_called(1);
        mock_mspi_write(0x9F).assert_called(1);
        mock_mspi_write(0).assert_called(2);
        mock_mspi_wait().assert_called(3);
        mock_mspi_get().assert_called(2);
    }
}