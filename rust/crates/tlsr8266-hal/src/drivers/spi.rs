use tlsr8266_pac::{
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
pub fn mspi_read() -> u8 {
    mspi_write(0); // Send dummy byte to generate clock pulses
    mspi_wait(); // Wait for transmission to complete
    mspi_get() // Read and return the received byte
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use tlsr8266_pac::fake_mmio::FakeMmio;

    /// `reg_master_spi_data`: the byte shifted out/in on the SPI bus.
    const REG_MASTER_SPI_DATA: u32 = 0x0c;
    /// `reg_master_spi_ctrl`: chip select and busy bits.
    const REG_MASTER_SPI_CTRL: u32 = 0x0d;

    /// The register bank is a process-wide singleton, so every test starts from
    /// a known-empty device.
    fn reset_fakes() {
        FakeMmio::reset();
    }

    /// Values written to `addr`, oldest first, in the order the SPI peripheral
    /// would have shifted them onto the wire.
    fn values_written_to(addr: u32) -> Vec<u32> {
        FakeMmio::writes()
            .into_iter()
            .filter(|write| write.addr == addr | tlsr8266_pac::REG_BASE_ADDR)
            .map(|write| write.value)
            .collect()
    }

    /// Reads of the SPI control register observed by [`busy_then_idle`].
    static CTRL_POLLS: AtomicUsize = AtomicUsize::new(0);

    /// Reports BUSY for the first two polls, then "transfer complete". The hook
    /// is a plain `fn` pointer, so the poll count lives in a static.
    fn busy_then_idle(_call_idx: usize) -> u8 {
        let polls = CTRL_POLLS.fetch_add(1, Ordering::Relaxed) + 1;
        if polls <= 2 {
            FLD_MASTER_SPI::BUSY.bits()
        } else {
            0
        }
    }

    /// Tests that mspi_high sets the SPI CS high by writing the correct bit to the control register.
    #[test]
    fn test_mspi_high() {
        reset_fakes();

        // Execute function under test
        mspi_high();

        // The control register now holds exactly the CS bit, and that single
        // write is the only traffic on the bus.
        assert_eq!(
            FakeMmio::reg8(REG_MASTER_SPI_CTRL),
            FLD_MASTER_SPI::CS.bits()
        );
        assert_eq!(
            values_written_to(REG_MASTER_SPI_CTRL),
            vec![u32::from(FLD_MASTER_SPI::CS.bits())]
        );
    }

    /// Tests that mspi_low clears the SPI CS bit by writing 0 to the control register.
    #[test]
    fn test_mspi_low() {
        reset_fakes();
        // Start from an asserted CS so the clear is observable.
        FakeMmio::set_reg8(REG_MASTER_SPI_CTRL, FLD_MASTER_SPI::CS.bits());

        // Execute function under test
        mspi_low();

        // Verify the control register was written correctly
        assert_eq!(FakeMmio::reg8(REG_MASTER_SPI_CTRL), 0);
        assert_eq!(values_written_to(REG_MASTER_SPI_CTRL), vec![0]);
    }

    /// Tests that mspi_write correctly writes data to the SPI data register.
    #[test]
    fn test_mspi_write() {
        reset_fakes();

        // Execute function under test with different values
        mspi_write(0x42);
        mspi_write(0xFF);

        // Both bytes reached the data register, in call order.
        assert_eq!(FakeMmio::reg8(REG_MASTER_SPI_DATA), 0xFF);
        assert_eq!(values_written_to(REG_MASTER_SPI_DATA), vec![0x42, 0xFF]);
    }

    /// Tests that mspi_wait polls the busy bit until the operation completes.
    #[test]
    fn test_mspi_wait() {
        reset_fakes();
        // The bank starts with the peripheral busy; the hook below reports the
        // transfer completing on the third poll.
        FakeMmio::set_reg8(REG_MASTER_SPI_CTRL, FLD_MASTER_SPI::BUSY.bits());
        CTRL_POLLS.store(0, Ordering::Relaxed);
        FakeMmio::on_read8(REG_MASTER_SPI_CTRL, busy_then_idle);

        // Execute function under test
        mspi_wait();

        // Two BUSY polls then one idle poll: the loop observed the flag and only
        // returned once it cleared.
        assert_eq!(
            CTRL_POLLS.load(Ordering::Relaxed),
            3,
            "mspi_wait must poll the BUSY flag until it clears"
        );
    }

    /// Tests that mspi_ctrl_write correctly writes to the SPI control register.
    #[test]
    fn test_mspi_ctrl_write() {
        reset_fakes();

        // Execute function under test with different values
        mspi_ctrl_write(0x0A);
        mspi_ctrl_write(0x01);

        // The last value wins in the register; both writes are on the wire.
        assert_eq!(FakeMmio::reg8(REG_MASTER_SPI_CTRL), 0x01);
        assert_eq!(values_written_to(REG_MASTER_SPI_CTRL), vec![0x0A, 0x01]);
    }

    /// Tests that mspi_get correctly reads data from the SPI data register.
    #[test]
    fn test_mspi_get() {
        reset_fakes();
        // Seed the byte the slave shifted in during the previous transfer.
        FakeMmio::set_reg8(REG_MASTER_SPI_DATA, 0x42);

        // Execute function under test
        let result = mspi_get();

        // Verify result
        assert_eq!(
            result, 0x42,
            "mspi_get should return the value from the data register"
        );
    }

    /// Tests that mspi_read performs the complete read sequence:
    /// 1. Writes a dummy byte to generate clock
    /// 2. Waits for operation to complete
    /// 3. Gets the received data
    #[test]
    fn test_mspi_read() {
        reset_fakes();
        // The slave answers every clocked byte with 0xA5.
        FakeMmio::on_read8(REG_MASTER_SPI_DATA, |_| 0xA5);

        // Execute function under test
        let result = mspi_read();

        // The dummy byte was clocked out and the received byte returned.
        assert_eq!(
            result, 0xA5,
            "mspi_read should return the byte shifted in from the slave"
        );
        assert_eq!(
            values_written_to(REG_MASTER_SPI_DATA),
            vec![0],
            "mspi_read must clock out exactly one dummy byte"
        );
    }

    /// Tests the integration between mspi functions in a typical SPI transaction.
    #[test]
    fn test_spi_transaction() {
        reset_fakes();
        // The slave shifts out the two JEDEC ID bytes on successive reads.
        FakeMmio::on_read8(REG_MASTER_SPI_DATA, |call_idx| match call_idx {
            0 => 0xEF, // First byte of JEDEC ID (manufacturer)
            _ => 0x40, // Second byte of JEDEC ID (device type)
        });

        // Execute a sequence that resembles a typical SPI transaction
        mspi_high(); // Ensure CS starts high
        mspi_low(); // Begin transaction by activating CS
        mspi_write(0x9F); // Send JEDEC ID command
        mspi_wait(); // Wait for command to send

        // Read first byte of response
        mspi_write(0); // Send dummy byte to clock in data
        mspi_wait(); // Wait for clock cycles
        let byte1 = mspi_get(); // Read received data

        // Read second byte of response
        mspi_write(0); // Send dummy byte to clock in data
        mspi_wait(); // Wait for clock cycles
        let byte2 = mspi_get(); // Read received data

        mspi_high(); // End transaction by deactivating CS

        // Verify results
        assert_eq!(byte1, 0xEF, "First JEDEC ID byte should be 0xEF");
        assert_eq!(byte2, 0x40, "Second JEDEC ID byte should be 0x40");

        // Verify the wire sequence: CS high, CS low, JEDEC command, two dummy
        // bytes, then CS high again to end the transaction.
        assert_eq!(
            values_written_to(REG_MASTER_SPI_CTRL),
            vec![
                u32::from(FLD_MASTER_SPI::CS.bits()),
                0,
                u32::from(FLD_MASTER_SPI::CS.bits()),
            ],
            "chip select must be asserted for the command and released at the end"
        );
        assert_eq!(
            values_written_to(REG_MASTER_SPI_DATA),
            vec![0x9F, 0, 0],
            "the JEDEC command must be followed by two dummy clock bytes"
        );
    }
}
