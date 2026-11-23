//! DMA Controller Module for TLSR8266
//!
//! This module provides functions to initialize and control the Direct Memory Access (DMA)
//! controller on the TLSR8266 SoC. The DMA controller allows for efficient data transfers
//! between memory and peripherals without CPU intervention.
//!
//! The TLSR8266 DMA controller provides multiple channels for data transfer operations,
//! helping to offload the CPU and improve system performance for I/O operations.

use tlsr8266_pac::write_reg_dma_chn_irq_msk;

/// Initialize the DMA controller.
///
/// This function prepares the DMA controller for operation by disabling all DMA channel
/// interrupts. Setting the interrupt mask register to 0 ensures that no DMA channel
/// can generate interrupts until explicitly enabled by other functions.
///
/// # Notes
///
/// * Must be called before using any DMA functionality
/// * On the TLSR8266, disabling DMA interrupts is part of the standard initialization sequence
/// * Does not configure any specific DMA channels - that must be done separately
/// * DMA channels must be individually configured for their specific transfer requirements
///
/// # Returns
///
/// * This function does not return a value
pub fn dma_init() {
    // Disable all DMA channel interrupts by writing 0 to the channel interrupt mask register
    write_reg_dma_chn_irq_msk(0)
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use tlsr8266_pac::fake_mmio::{FakeMmio, RegWrite};
    use tlsr8266_pac::REG_BASE_ADDR;

    /// `reg_dma_chn_irq_msk`, the register `dma_init` must clear.
    const DMA_CHN_IRQ_MSK: u32 = 0x521;

    /// The `FakeMmio` bank is process-wide, so every test starts from zero.
    fn reset_fakes() {
        FakeMmio::reset();
    }

    /// Tests the DMA initialization function.
    ///
    /// This test verifies that the dma_init function correctly:
    /// - Disables all DMA channel interrupts by writing 0 to the interrupt mask register
    ///
    /// # Algorithm
    ///
    /// 1. Reset the fake register bank
    /// 2. Call the dma_init function
    /// 3. Verify the write log holds exactly one 8-bit write of 0 to the mask register
    #[test]
    fn test_dma_init() {
        reset_fakes();

        // Call the function being tested
        dma_init();

        // Verify the register write happened exactly once with value 0
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: DMA_CHN_IRQ_MSK | REG_BASE_ADDR,
                width: 1,
                value: 0,
            }]
        );
    }

    /// Tests that dma_init specifically disables interrupts with value 0.
    ///
    /// This test verifies that the dma_init function:
    /// - Specifically writes the value 0 to the DMA channel interrupt mask register
    /// - Will not write any other value to the register
    ///
    /// # Algorithm
    ///
    /// 1. Seed the mask register with a non-zero value so a skipped write is visible
    /// 2. Call the dma_init function
    /// 3. Verify the register reads back as 0 and no other register was written
    #[test]
    fn test_dma_init_uses_zero_value() {
        reset_fakes();
        // Pre-set every mask bit: a "write 0" is the only way back to 0.
        FakeMmio::set_reg8(DMA_CHN_IRQ_MSK, 0xff);
        FakeMmio::clear_writes();

        // Call the function being tested
        dma_init();

        // Verify the register holds 0 and not any other value
        assert_eq!(FakeMmio::reg8(DMA_CHN_IRQ_MSK), 0);

        // Total call count should be 1
        let writes = FakeMmio::writes();
        assert_eq!(writes.len(), 1, "dma_init must write exactly one register");
        assert_eq!(writes[0].addr, DMA_CHN_IRQ_MSK | REG_BASE_ADDR);
        assert_eq!(writes[0].value, 0);
    }
}
