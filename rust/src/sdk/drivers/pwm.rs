use crate::sdk::mcu::register::{
    read_reg_pwm_enable, write_reg_pwm_cmp, write_reg_pwm_cycle, write_reg_pwm_enable, FLD_PWM,
};
use crate::{BIT, BIT_LOW_BIT, MASK_VAL};

/// Sets the compare value for a PWM channel.
///
/// The compare value determines the point within the PWM cycle at which 
/// the output transitions. This effectively controls the duty cycle of 
/// the PWM signal.
///
/// # Parameters
///
/// * `id` - The PWM channel ID (0-7)
/// * `cmp` - The compare value to set
///
/// # Algorithm
///
/// 1. Calculate the register offset based on PWM ID (id << 2)
/// 2. Write the compare value to the appropriate register
///
/// # Notes
///
/// * The register offset is calculated as (id << 2) because each PWM channel's
///   registers are spaced 4 bytes apart in the register map
/// * Lower compare values result in shorter duty cycles
pub fn pwm_set_cmp(id: u32, cmp: u16) {
    // Each PWM channel's registers are spaced 4 bytes apart
    // Calculate the offset by shifting the ID left by 2 (multiplying by 4)
    write_reg_pwm_cmp(cmp, id << 2)
}

/// Sets both the maximum and compare values for a PWM channel.
///
/// This function configures the complete PWM timing cycle by setting both
/// the maximum counter value (which determines the frequency) and the compare
/// value (which determines the duty cycle).
///
/// # Parameters
///
/// * `id` - The PWM channel ID (0-7)
/// * `max_tick` - The maximum counter value for the PWM cycle
/// * `cmp_tick` - The compare value that determines duty cycle
///
/// # Algorithm
///
/// 1. Combine max_tick and cmp_tick into a single register value using MASK_VAL
/// 2. Calculate the register offset based on PWM ID (id << 2)
/// 3. Write the combined value to the PWM cycle register
///
/// # Notes
///
/// * The duty cycle percentage is approximately (max_tick - cmp_tick) / max_tick * 100%
/// * The PWM frequency is determined by the system clock divided by max_tick
/// * The register offset is calculated as (id << 2) because each PWM channel's
///   registers are spaced 4 bytes apart
pub fn pwm_set_duty(id: u32, max_tick: u16, cmp_tick: u16) {
    // Combine the max and compare values into a single 32-bit register value
    // FLD_PWM::CMP and FLD_PWM::MAX define the bit positions for these values
    // MASK_VAL! macro handles the bit manipulation for placing values at the correct positions
    write_reg_pwm_cycle(
        MASK_VAL!(
            FLD_PWM::CMP.bits(),
            cmp_tick as u32,
            FLD_PWM::MAX.bits(),
            max_tick as u32
        ),
        // Calculate register offset (each channel is 4 bytes apart)
        id << 2,
    );
}

/// Starts a PWM channel.
///
/// Enables the specified PWM channel by setting its corresponding bit
/// in the PWM enable register.
///
/// # Parameters
///
/// * `id` - The PWM channel ID (0-7)
///
/// # Algorithm
///
/// 1. Read the current value of the PWM enable register
/// 2. Set the bit corresponding to the specified channel ID
/// 3. Write the updated value back to the PWM enable register
///
/// # Notes
///
/// * The function preserves the state of other PWM channels
/// * Each bit in the enable register corresponds to one PWM channel
/// * The channel must be properly configured before starting
pub fn pwm_start(id: u32) {
    // Read current enable register value to preserve other channel states
    let current = read_reg_pwm_enable();
    
    // Set the bit for this channel using the BIT! macro
    // This creates a bitmask with only the specified bit set to 1
    let new_value = current | BIT!(id);
    
    // Write the new value back to the enable register
    write_reg_pwm_enable(new_value);
}

/// Stops a PWM channel.
///
/// Disables the specified PWM channel by clearing its corresponding bit
/// in the PWM enable register.
///
/// # Parameters
///
/// * `id` - The PWM channel ID (0-7)
///
/// # Algorithm
///
/// 1. Read the current value of the PWM enable register
/// 2. Clear the bit corresponding to the specified channel ID
/// 3. Write the updated value back to the PWM enable register
///
/// # Notes
///
/// * The function preserves the state of other PWM channels
/// * Each bit in the enable register corresponds to one PWM channel
/// * The PWM configuration is retained and can be restarted with pwm_start
pub fn pwm_stop(id: u32) {
    // Read current enable register value to preserve other channel states
    let current = read_reg_pwm_enable();
    
    // Clear the bit for this channel
    // !BIT!(id) creates a bitmask with all bits set to 1 except the specified bit
    let new_value = current & !BIT!(id);
    
    // Write the new value back to the enable register
    write_reg_pwm_enable(new_value);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdk::mcu::register::{
        mock_read_reg_pwm_enable, mock_write_reg_pwm_cmp, mock_write_reg_pwm_cycle, 
        mock_write_reg_pwm_enable,
    };
    
    /// Tests the pwm_set_cmp function.
    ///
    /// This test verifies that the pwm_set_cmp function correctly:
    /// - Calculates the register offset based on the PWM ID
    /// - Passes the correct compare value to the register write function
    ///
    /// # Algorithm
    ///
    /// 1. Mock the write_reg_pwm_cmp function
    /// 2. Call pwm_set_cmp with test parameters
    /// 3. Verify that write_reg_pwm_cmp was called with the correct parameters
    ///
    /// # Notes
    ///
    /// * The register offset is calculated as (id << 2)
    /// * The compare value is passed directly to the register
    #[test]
    #[mry::lock(write_reg_pwm_cmp)]
    fn test_pwm_set_cmp() {
        // Configure mock behavior
        mock_write_reg_pwm_cmp(500, 4).returns(());
        
        // Call the function being tested
        pwm_set_cmp(1, 500);
        
        // Verify the register was written with correct parameters:
        // - cmp value of 500
        // - offset of 4 (1 << 2 = 4)
        mock_write_reg_pwm_cmp(500, 4).assert_called(1);
    }
    
    /// Tests the pwm_set_duty function.
    ///
    /// This test verifies that the pwm_set_duty function correctly:
    /// - Calculates the register offset based on the PWM ID
    /// - Properly combines the max and compare values using MASK_VAL
    /// - Calls the register write function with the correct parameters
    ///
    /// # Algorithm
    ///
    /// 1. Mock the write_reg_pwm_cycle function
    /// 2. Call pwm_set_duty with test parameters
    /// 3. Calculate the expected masked value manually
    /// 4. Verify that write_reg_pwm_cycle was called with the correct parameters
    ///
    /// # Notes
    ///
    /// * The register offset is calculated as (id << 2)
    /// * The masked value combines both max_tick and cmp_tick
    #[test]
    #[mry::lock(write_reg_pwm_cycle)]
    fn test_pwm_set_duty() {
        // Test parameters
        let pwm_id = 2;
        let max_tick = 1000;
        let cmp_tick = 250;
        
        // Calculate expected masked value
        let expected_value = MASK_VAL!(
            FLD_PWM::CMP.bits(),
            cmp_tick as u32,
            FLD_PWM::MAX.bits(),
            max_tick as u32
        );
        
        // Configure mock behavior
        mock_write_reg_pwm_cycle(expected_value, 8).returns(());
        
        // Call the function being tested
        pwm_set_duty(pwm_id, max_tick, cmp_tick);
        
        // Verify the register was written with correct parameters:
        // - calculated mask value
        // - offset of 8 (2 << 2 = 8)
        mock_write_reg_pwm_cycle(expected_value, 8).assert_called(1);
    }
    
    /// Tests the pwm_start function.
    ///
    /// This test verifies that the pwm_start function correctly:
    /// - Reads the current PWM enable register value
    /// - Sets the appropriate bit for the specified PWM ID
    /// - Writes the updated value back to the register
    ///
    /// # Algorithm
    ///
    /// 1. Mock the read_reg_pwm_enable and write_reg_pwm_enable functions
    /// 2. Configure read_reg_pwm_enable to return a test value
    /// 3. Call pwm_start with a test PWM ID
    /// 4. Verify that write_reg_pwm_enable was called with the correctly modified value
    ///
    /// # Notes
    ///
    /// * This test verifies the bit manipulation logic for enabling a PWM channel
    #[test]
    #[mry::lock(read_reg_pwm_enable, write_reg_pwm_enable)]
    fn test_pwm_start() {
        // Setup existing register value (bits 0 and 2 already enabled)
        let existing_value = 0b0101;
        mock_read_reg_pwm_enable().returns(existing_value);
        
        // We're going to enable PWM channel 1 (bit 1)
        let pwm_id = 1;
        let expected_new_value = existing_value | BIT!(pwm_id);  // Should be 0b0111
        
        // Configure mock write
        mock_write_reg_pwm_enable(expected_new_value).returns(());
        
        // Call the function being tested
        pwm_start(pwm_id);
        
        // Verify register was read
        mock_read_reg_pwm_enable().assert_called(1);
        
        // Verify register was written with correct new value
        mock_write_reg_pwm_enable(expected_new_value).assert_called(1);
    }
    
    /// Tests the pwm_stop function.
    ///
    /// This test verifies that the pwm_stop function correctly:
    /// - Reads the current PWM enable register value
    /// - Clears the appropriate bit for the specified PWM ID
    /// - Writes the updated value back to the register
    ///
    /// # Algorithm
    ///
    /// 1. Mock the read_reg_pwm_enable and write_reg_pwm_enable functions
    /// 2. Configure read_reg_pwm_enable to return a test value
    /// 3. Call pwm_stop with a test PWM ID
    /// 4. Verify that write_reg_pwm_enable was called with the correctly modified value
    ///
    /// # Notes
    ///
    /// * This test verifies the bit manipulation logic for disabling a PWM channel
    #[test]
    #[mry::lock(read_reg_pwm_enable, write_reg_pwm_enable)]
    fn test_pwm_stop() {
        // Setup existing register value (bits 0, 1, and 2 enabled)
        let existing_value = 0b0111;
        mock_read_reg_pwm_enable().returns(existing_value);
        
        // We're going to disable PWM channel 1 (bit 1)
        let pwm_id = 1;
        let expected_new_value = existing_value & !BIT!(pwm_id);  // Should be 0b0101
        
        // Configure mock write
        mock_write_reg_pwm_enable(expected_new_value).returns(());
        
        // Call the function being tested
        pwm_stop(pwm_id);
        
        // Verify register was read
        mock_read_reg_pwm_enable().assert_called(1);
        
        // Verify register was written with correct new value
        mock_write_reg_pwm_enable(expected_new_value).assert_called(1);
    }
}
