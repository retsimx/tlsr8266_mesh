//! DMA Controller Module for TLSR8266
//!
//! This module provides functions to initialize and control the Direct Memory Access (DMA)
//! controller on the TLSR8266 SoC. The DMA controller allows for efficient data transfers
//! between memory and peripherals without CPU intervention.
//!
//! The TLSR8266 DMA controller provides multiple channels for data transfer operations,
//! helping to offload the CPU and improve system performance for I/O operations.

use crate::sdk::mcu::register::write_reg_dma_chn_irq_msk;

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
mod tests {
    use super::*;
    use crate::sdk::mcu::register::mock_write_reg_dma_chn_irq_msk;
    use mry::Any;

    /// Tests the DMA initialization function.
    ///
    /// This test verifies that the dma_init function correctly:
    /// - Disables all DMA channel interrupts by writing 0 to the interrupt mask register
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock for the register write function
    /// 2. Call the dma_init function
    /// 3. Verify that the register write function was called exactly once with value 0
    #[test]
    #[mry::lock(write_reg_dma_chn_irq_msk)]
    fn test_dma_init() {
        // Setup mock for the register write function
        mock_write_reg_dma_chn_irq_msk(0).returns(());
        
        // Call the function being tested
        dma_init();
        
        // Verify the register write function was called exactly once with value 0
        mock_write_reg_dma_chn_irq_msk(0).assert_called(1);
    }

    /// Tests that dma_init specifically disables interrupts with value 0.
    ///
    /// This test verifies that the dma_init function:
    /// - Specifically writes the value 0 to the DMA channel interrupt mask register
    /// - Will not write any other value to the register
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock to accept any value but verify only 0 is passed
    /// 2. Call the dma_init function
    /// 3. Verify the correct value (0) was used and no other values were used
    #[test]
    #[mry::lock(write_reg_dma_chn_irq_msk)]
    fn test_dma_init_uses_zero_value() {
        // Setup mock with Any matcher to verify that only 0 is used
        mock_write_reg_dma_chn_irq_msk(Any).returns(());
        
        // Call the function being tested
        dma_init();
        
        // Verify the register write was called with 0 and not any other value
        mock_write_reg_dma_chn_irq_msk(0).assert_called(1);
        
        // Total call count should be 1
        mock_write_reg_dma_chn_irq_msk(Any).assert_called(1);
    }
}
