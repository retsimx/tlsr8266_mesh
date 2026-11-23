use crate::{BIT, MASK_VAL};
use tlsr8266_pac::{
    read_reg_pwm_enable, write_reg_pwm_cmp, write_reg_pwm_cycle, write_reg_pwm_enable, FLD_PWM,
};

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
#[coverage(off)]
mod tests {
    use super::*;
    use tlsr8266_pac::fake_mmio::FakeMmio;

    /// `reg_pwm_enable`: one enable bit per channel.
    const REG_PWM_ENABLE: u32 = 0x780;
    /// `reg_pwm_cycle` / `reg_pwm_cmp` base: each channel is 4 bytes apart.
    const REG_PWM_CYCLE_BASE: u32 = 0x794;

    /// The register bank is a process-wide singleton, so every test starts from
    /// a known-empty device.
    fn reset_fakes() {
        FakeMmio::reset();
    }

    /// Tests the pwm_set_cmp function.
    ///
    /// This test verifies that the pwm_set_cmp function correctly:
    /// - Calculates the register offset based on the PWM ID
    /// - Writes the compare value to the channel's compare register
    ///
    /// # Notes
    ///
    /// * The register offset is calculated as (id << 2)
    /// * The compare value is written as a 16-bit field
    #[test]
    fn test_pwm_set_cmp() {
        reset_fakes();

        // Call the function being tested
        pwm_set_cmp(1, 500);

        // Channel 1's compare register sits at the base + (1 << 2).
        let cmp_addr = REG_PWM_CYCLE_BASE + 4;
        assert_eq!(
            FakeMmio::reg16(cmp_addr),
            500,
            "the compare value must land in the channel's compare register"
        );

        let writes = FakeMmio::writes();
        assert_eq!(writes.len(), 1, "pwm_set_cmp performs exactly one write");
        assert_eq!(writes[0].addr, cmp_addr | tlsr8266_pac::REG_BASE_ADDR);
        assert_eq!(writes[0].width, 2, "the compare field is 16 bits wide");
        assert_eq!(writes[0].value, 500);
    }

    /// Tests the pwm_set_duty function.
    ///
    /// This test verifies that the pwm_set_duty function correctly:
    /// - Calculates the register offset based on the PWM ID
    /// - Properly combines the max and compare values using MASK_VAL
    /// - Writes the combined value to the channel's cycle register
    ///
    /// # Notes
    ///
    /// * The register offset is calculated as (id << 2)
    /// * The masked value combines both max_tick and cmp_tick
    #[test]
    fn test_pwm_set_duty() {
        reset_fakes();

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

        // Call the function being tested
        pwm_set_duty(pwm_id, max_tick, cmp_tick);

        // Channel 2's cycle register sits at the base + (2 << 2).
        let cycle_addr = REG_PWM_CYCLE_BASE + 8;
        assert_eq!(
            FakeMmio::reg32(cycle_addr),
            expected_value,
            "the cycle register must hold both the max and compare fields"
        );

        let writes = FakeMmio::writes();
        assert_eq!(writes.len(), 1, "pwm_set_duty performs exactly one write");
        assert_eq!(writes[0].addr, cycle_addr | tlsr8266_pac::REG_BASE_ADDR);
        assert_eq!(writes[0].width, 4, "the cycle register is 32 bits wide");
        assert_eq!(writes[0].value, expected_value);
    }

    /// Tests the pwm_start function.
    ///
    /// This test verifies that the pwm_start function correctly:
    /// - Reads the current PWM enable register value
    /// - Sets the appropriate bit for the specified PWM ID
    /// - Writes the updated value back to the register
    ///
    /// # Notes
    ///
    /// * This test verifies the bit manipulation logic for enabling a PWM channel
    #[test]
    fn test_pwm_start() {
        reset_fakes();

        // Setup existing register value (bits 0 and 2 already enabled)
        let existing_value = 0b0101u8;
        FakeMmio::set_reg8(REG_PWM_ENABLE, existing_value);

        // We're going to enable PWM channel 1 (bit 1)
        let pwm_id = 1;
        let expected_new_value = existing_value | BIT!(pwm_id); // Should be 0b0111

        // Call the function being tested
        pwm_start(pwm_id);

        // The channel is now enabled and the unrelated channels are untouched.
        assert_eq!(FakeMmio::reg8(REG_PWM_ENABLE), expected_new_value);

        let writes = FakeMmio::writes();
        assert_eq!(writes.len(), 1, "pwm_start performs exactly one write");
        assert_eq!(writes[0].addr, REG_PWM_ENABLE | tlsr8266_pac::REG_BASE_ADDR);
        assert_eq!(writes[0].value, u32::from(expected_new_value));
    }

    /// Tests the pwm_stop function.
    ///
    /// This test verifies that the pwm_stop function correctly:
    /// - Reads the current PWM enable register value
    /// - Clears the appropriate bit for the specified PWM ID
    /// - Writes the updated value back to the register
    ///
    /// # Notes
    ///
    /// * This test verifies the bit manipulation logic for disabling a PWM channel
    #[test]
    fn test_pwm_stop() {
        reset_fakes();

        // Setup existing register value (bits 0, 1, and 2 enabled)
        let existing_value = 0b0111u8;
        FakeMmio::set_reg8(REG_PWM_ENABLE, existing_value);

        // We're going to disable PWM channel 1 (bit 1)
        let pwm_id = 1;
        let expected_new_value = existing_value & !BIT!(pwm_id); // Should be 0b0101

        // Call the function being tested
        pwm_stop(pwm_id);

        // Only the target channel was disabled.
        assert_eq!(FakeMmio::reg8(REG_PWM_ENABLE), expected_new_value);

        let writes = FakeMmio::writes();
        assert_eq!(writes.len(), 1, "pwm_stop performs exactly one write");
        assert_eq!(writes[0].addr, REG_PWM_ENABLE | tlsr8266_pac::REG_BASE_ADDR);
        assert_eq!(writes[0].value, u32::from(expected_new_value));
    }
}
