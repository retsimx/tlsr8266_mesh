//!
//! # TLSR8266 Interrupt Control Interface
//!
//! This module provides low-level functions for managing the interrupt system
//! on the TLSR8266 microcontroller. These functions enable critical sections
//! through interrupt masking and restoration.
//!
//! The module supports the following operations:
//! - Enabling/disabling global interrupts
//! - Saving and restoring interrupt state
//! - Initializing the interrupt system with default masks
//!

use crate::sdk::mcu::register::{read_reg_irq_en, write_reg_irq_en, write_reg_irq_mask, FLD_IRQ};

/// Default interrupt configuration value used during initialization.
///
/// This value enables two specific interrupt sources:
/// - Timer1 interrupts (FLD_IRQ::TMR1_EN)
/// - ZigBee/BLE RF real-time interrupts (FLD_IRQ::ZB_RT_EN)
///
/// These interrupts are essential for the mesh networking and timing functions.
pub const IRQ_INIT_VALUE: u32 = FLD_IRQ::TMR1_EN.bits() | FLD_IRQ::ZB_RT_EN.bits();

/// Enables global interrupts on the TLSR8266 MCU.
///
/// # Algorithm
/// 1. Reads the current interrupt enable register state
/// 2. Enables global interrupts by writing 1 to the enable register
/// 3. Returns the previous state for later restoration
///
/// # Returns
///
/// * `u8` - The previous interrupt enable register value (0 or 1)
///
/// # Notes
///
/// * Used to start a non-critical section after a critical section
/// * Should be paired with irq_restore() when the non-critical section ends
/// * The return value can be passed to irq_restore() to restore previous state
#[cfg_attr(test, mry::mry)]
pub fn irq_enable() -> u8 {
    // Step 1: Read current interrupt enable state before modification
    let r = read_reg_irq_en(); // don't worry, the compiler will optimize the return value if not used
    // Step 2: Enable global interrupts by writing 1 to enable register
    write_reg_irq_en(1);
    // Step 3: Return previous state so it can be restored later
    return r;
}

/// Disables global interrupts on the TLSR8266 MCU.
///
/// # Algorithm
/// 1. Reads the current interrupt enable register state
/// 2. Disables global interrupts by writing 0 to the enable register
/// 3. Returns the previous state for later restoration
///
/// # Returns
///
/// * `u8` - The previous interrupt enable register value (0 or 1)
///
/// # Notes
///
/// * Used to start a critical section where interrupts should not occur
/// * Should be paired with irq_restore() when the critical section ends
/// * When interrupts are disabled, time-sensitive operations can be
///   performed without being interrupted, but this should be kept as
///   short as possible to avoid missing important interrupts
#[cfg_attr(test, mry::mry)]
pub fn irq_disable() -> u8 {
    // Step 1: Read current interrupt enable state before modification
    let r = read_reg_irq_en(); // don't worry, the compiler will optimize the return value if not used
    // Step 2: Disable global interrupts by writing 0 to enable register
    write_reg_irq_en(0);
    // Step 3: Return previous state so it can be restored later
    return r;
}

/// Restores the global interrupt state to a previously saved value.
///
/// # Parameters
///
/// * `en` - The interrupt enable value to restore (typically obtained from
///          irq_enable() or irq_disable())
///
/// # Algorithm
/// 1. Writes the provided value to the interrupt enable register
///
/// # Notes
///
/// * Used to end a critical section by restoring the interrupt state
///   that existed before the critical section began
/// * Should be paired with irq_disable() or irq_enable() which provides
///   the previous state to restore
#[cfg_attr(test, mry::mry)]
pub fn irq_restore(en: u8) {
    // Write saved interrupt enable state back to the register
    write_reg_irq_en(en);
}

/// Initializes the interrupt system with default configuration.
///
/// # Algorithm
/// 1. Writes the IRQ_INIT_VALUE to the interrupt mask register to
///    enable specific interrupt sources needed by the application
///
/// # Notes
///
/// * This function should be called early in the boot process
/// * Enables only the necessary interrupts defined in IRQ_INIT_VALUE:
///   - Timer1 interrupts for timing functions
///   - ZigBee/BLE real-time interrupts for wireless communication
/// * Other interrupts can be enabled later as needed by the application
#[cfg_attr(test, mry::mry)]
pub fn irq_init() {
    // Configure which specific interrupt sources are enabled
    write_reg_irq_mask(IRQ_INIT_VALUE);
}

/// Test suite for the interrupt control interface
#[cfg(test)]
mod tests {
    use super::*;
    use mry::Any;
    use crate::sdk::mcu::register::{
        mock_read_reg_irq_en, mock_write_reg_irq_en, mock_write_reg_irq_mask
    };

    /// Tests enabling global interrupts.
    ///
    /// This test verifies that irq_enable correctly:
    /// - Reads the current interrupt state
    /// - Enables interrupts by writing 1 to the register
    /// - Returns the previous state
    ///
    /// # Algorithm
    /// 
    /// 1. Mock the read_reg_irq_en function to return a known value (0)
    /// 2. Mock the write_reg_irq_en function (no return value needed)
    /// 3. Call irq_enable()
    /// 4. Verify the return value is the mocked value from read_reg_irq_en
    /// 5. Verify that write_reg_irq_en was called with the parameter 1
    ///
    /// # Notes
    ///
    /// * Tests the case where interrupts were previously disabled
    #[test]
    #[mry::lock(read_reg_irq_en, write_reg_irq_en)]
    fn test_irq_enable_when_disabled() {
        // Step 1: Mock read_reg_irq_en to return 0 (interrupts disabled)
        mock_read_reg_irq_en().returns(0);
        // Step 2: Mock write_reg_irq_en (void function)
        mock_write_reg_irq_en(Any).returns(());
        
        // Step 3: Call the function being tested
        let previous_state = irq_enable();
        
        // Step 4: Verify the previous state is returned correctly
        assert_eq!(previous_state, 0, "irq_enable should return the previous state (0)");
        
        // Step 5: Verify write_reg_irq_en was called with the correct parameter
        mock_write_reg_irq_en(1).assert_called(1);
    }
    
    /// Tests enabling global interrupts when already enabled.
    ///
    /// This test verifies that irq_enable correctly:
    /// - Reads the current interrupt state
    /// - Still writes to the enable register (idempotent operation)
    /// - Returns the previous state
    ///
    /// # Algorithm
    ///
    /// 1. Mock the read_reg_irq_en function to return 1 (enabled)
    /// 2. Mock the write_reg_irq_en function
    /// 3. Call irq_enable()
    /// 4. Verify the return value is 1
    /// 5. Verify write_reg_irq_en was still called with parameter 1
    ///
    /// # Notes
    ///
    /// * Tests the case where interrupts were already enabled
    /// * Confirms the function is idempotent
    #[test]
    #[mry::lock(read_reg_irq_en, write_reg_irq_en)]
    fn test_irq_enable_when_already_enabled() {
        // Step 1: Mock read_reg_irq_en to return 1 (interrupts enabled)
        mock_read_reg_irq_en().returns(1);
        // Step 2: Mock write_reg_irq_en (void function)
        mock_write_reg_irq_en(Any).returns(());
        
        // Step 3: Call the function being tested
        let previous_state = irq_enable();
        
        // Step 4: Verify the previous state is returned correctly
        assert_eq!(previous_state, 1, "irq_enable should return the previous state (1)");
        
        // Step 5: Verify write_reg_irq_en was called with the correct parameter
        mock_write_reg_irq_en(1).assert_called(1);
    }
    
    /// Tests disabling global interrupts.
    ///
    /// This test verifies that irq_disable correctly:
    /// - Reads the current interrupt state
    /// - Disables interrupts by writing 0 to the register
    /// - Returns the previous state
    ///
    /// # Algorithm
    ///
    /// 1. Mock read_reg_irq_en to return 1 (enabled)
    /// 2. Mock write_reg_irq_en
    /// 3. Call irq_disable()
    /// 4. Verify the return value is 1
    /// 5. Verify write_reg_irq_en was called with parameter 0
    ///
    /// # Notes
    ///
    /// * Tests the case where interrupts were previously enabled
    #[test]
    #[mry::lock(read_reg_irq_en, write_reg_irq_en)]
    fn test_irq_disable_when_enabled() {
        // Step 1: Mock read_reg_irq_en to return 1 (interrupts enabled)
        mock_read_reg_irq_en().returns(1);
        // Step 2: Mock write_reg_irq_en (void function)
        mock_write_reg_irq_en(Any).returns(());
        
        // Step 3: Call the function being tested
        let previous_state = irq_disable();
        
        // Step 4: Verify the previous state is returned correctly
        assert_eq!(previous_state, 1, "irq_disable should return the previous state (1)");
        
        // Step 5: Verify write_reg_irq_en was called with the correct parameter
        mock_write_reg_irq_en(0).assert_called(1);
    }
    
    /// Tests disabling global interrupts when already disabled.
    ///
    /// This test verifies that irq_disable correctly:
    /// - Reads the current interrupt state
    /// - Still writes to the enable register (idempotent operation)
    /// - Returns the previous state
    ///
    /// # Algorithm
    ///
    /// 1. Mock read_reg_irq_en to return 0 (disabled)
    /// 2. Mock write_reg_irq_en
    /// 3. Call irq_disable()
    /// 4. Verify the return value is 0
    /// 5. Verify write_reg_irq_en was still called with parameter 0
    ///
    /// # Notes
    ///
    /// * Tests the case where interrupts were already disabled
    /// * Confirms the function is idempotent
    #[test]
    #[mry::lock(read_reg_irq_en, write_reg_irq_en)]
    fn test_irq_disable_when_already_disabled() {
        // Step 1: Mock read_reg_irq_en to return 0 (interrupts disabled)
        mock_read_reg_irq_en().returns(0);
        // Step 2: Mock write_reg_irq_en (void function)
        mock_write_reg_irq_en(Any).returns(());
        
        // Step 3: Call the function being tested
        let previous_state = irq_disable();
        
        // Step 4: Verify the previous state is returned correctly
        assert_eq!(previous_state, 0, "irq_disable should return the previous state (0)");
        
        // Step 5: Verify write_reg_irq_en was called with the correct parameter
        mock_write_reg_irq_en(0).assert_called(1);
    }
    
    /// Tests restoring global interrupts to enabled state.
    ///
    /// This test verifies that irq_restore correctly:
    /// - Writes the provided state (1) to the enable register
    ///
    /// # Algorithm
    ///
    /// 1. Mock write_reg_irq_en
    /// 2. Call irq_restore(1) to restore enabled state
    /// 3. Verify write_reg_irq_en was called with parameter 1
    ///
    /// # Notes
    ///
    /// * Tests restoring to enabled state (1)
    #[test]
    #[mry::lock(write_reg_irq_en)]
    fn test_irq_restore_to_enabled() {
        // Step 1: Mock write_reg_irq_en (void function)
        mock_write_reg_irq_en(Any).returns(());
        
        // Step 2: Call the function being tested
        irq_restore(1);
        
        // Step 3: Verify write_reg_irq_en was called with the correct parameter
        mock_write_reg_irq_en(1).assert_called(1);
    }
    
    /// Tests restoring global interrupts to disabled state.
    ///
    /// This test verifies that irq_restore correctly:
    /// - Writes the provided state (0) to the enable register
    ///
    /// # Algorithm
    ///
    /// 1. Mock write_reg_irq_en
    /// 2. Call irq_restore(0) to restore disabled state
    /// 3. Verify write_reg_irq_en was called with parameter 0
    ///
    /// # Notes
    ///
    /// * Tests restoring to disabled state (0)
    #[test]
    #[mry::lock(write_reg_irq_en)]
    fn test_irq_restore_to_disabled() {
        // Step 1: Mock write_reg_irq_en (void function)
        mock_write_reg_irq_en(Any).returns(());
        
        // Step 2: Call the function being tested
        irq_restore(0);
        
        // Step 3: Verify write_reg_irq_en was called with the correct parameter
        mock_write_reg_irq_en(0).assert_called(1);
    }
    
    /// Tests initializing the interrupt system.
    ///
    /// This test verifies that irq_init correctly:
    /// - Writes the IRQ_INIT_VALUE to the interrupt mask register
    ///
    /// # Algorithm
    ///
    /// 1. Mock write_reg_irq_mask
    /// 2. Call irq_init()
    /// 3. Verify write_reg_irq_mask was called with IRQ_INIT_VALUE
    ///
    /// # Notes
    ///
    /// * Verifies the correct initialization value is used
    #[test]
    #[mry::lock(write_reg_irq_mask)]
    fn test_irq_init() {
        // Step 1: Mock write_reg_irq_mask (void function)
        mock_write_reg_irq_mask(Any).returns(());
        
        // Step 2: Call the function being tested
        irq_init();
        
        // Step 3: Verify write_reg_irq_mask was called with the correct parameter
        mock_write_reg_irq_mask(IRQ_INIT_VALUE).assert_called(1);
    }
    
    /// Tests a typical critical section pattern using the interrupt functions.
    ///
    /// This test verifies the typical usage pattern:
    /// - Disable interrupts and save previous state
    /// - Perform operations in critical section (simulated)
    /// - Restore previous interrupt state
    ///
    /// # Algorithm
    ///
    /// 1. Mock read_reg_irq_en to return 1 (enabled)
    /// 2. Mock write_reg_irq_en
    /// 3. Call irq_disable() to start critical section
    /// 4. Call irq_restore() with returned state
    /// 5. Verify both functions were called with correct parameters
    ///
    /// # Notes
    ///
    /// * Demonstrates and tests the common critical section pattern
    #[test]
    #[mry::lock(read_reg_irq_en, write_reg_irq_en)]
    fn test_critical_section_pattern() {
        // Step 1: Mock read_reg_irq_en to return 1 (interrupts were enabled)
        mock_read_reg_irq_en().returns(1);
        // Step 2: Mock write_reg_irq_en (void function)
        mock_write_reg_irq_en(Any).returns(());
        
        // Step 3: Call irq_disable() to start critical section
        let previous_state = irq_disable();
        
        // Simulate some operations in the critical section...
        
        // Step 4: Call irq_restore() to end critical section
        irq_restore(previous_state);
        
        // Step 5: Verify write_reg_irq_en was called correctly for both operations
        mock_write_reg_irq_en(0).assert_called(1); // First call with 0 to disable interrupts
        mock_write_reg_irq_en(1).assert_called(1); // Second call with 1 to restore enabled state
    }
}
