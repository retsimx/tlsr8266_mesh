//! Random number generation for TLSR8266 MCU.
//!
//! This module provides functions for generating pseudo-random numbers
//! suitable for non-cryptographic applications on the TLSR8266 platform.

use crate::mcu::clock::clock_time;
use tlsr8266_pac::read_reg_rnd_number;

/// Generates a 16-bit pseudo-random number.
///
/// # Algorithm
///
/// This function combines two sources of entropy to generate a random number:
/// 1. The current system clock time (truncated to 16 bits)
/// 2. The hardware random number generator register value
///
/// The two values are combined using XOR (^) to produce the final random number.
/// This approach improves entropy by mixing the time-based value with the hardware
/// RNG value, providing better randomness than either source alone.
///
/// # Returns
///
/// * A 16-bit unsigned integer (`u16`) containing the generated random number
///
/// # Notes
///
/// * This is suitable for non-cryptographic applications only
/// * The quality of randomness depends on both clock_time() variability and
///   the hardware RNG implementation in the TLSR8266
/// * The function is marked as inline(always) for performance in time-critical sections
#[inline(always)]
pub fn rand() -> u16 {
    clock_time() as u16 ^ read_reg_rnd_number()
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::mcu::clock::fake_clock;
    use tlsr8266_pac::fake_mmio::FakeMmio;

    /// `reg_rnd_number`, the hardware entropy source.
    const RND_NUMBER: u32 = 0x448;

    /// `fake_clock`, `FakeMmio` and `FakeAnalog` are process-wide singletons and
    /// the suite runs with `--test-threads=1`, so each test starts from zero.
    fn reset_fakes() {
        FakeMmio::reset();
        fake_clock::reset();
    }

    /// Tests that the rand function correctly combines clock_time and hardware RNG values.
    ///
    /// This test verifies that the rand function properly:
    /// - Reads clock_time() from the system tick register
    /// - Reads read_reg_rnd_number() from the hardware RNG register
    /// - Combines the values using XOR operation
    /// - Returns a 16-bit unsigned integer
    ///
    /// # Algorithm
    ///
    /// 1. Seed the system tick register and the hardware RNG register
    /// 2. Call rand() function to generate random number
    /// 3. Verify the returned value matches the expected XOR combination
    ///
    /// # Notes
    ///
    /// * This test confirms the core algorithm of the rand function
    /// * The test uses fixed values to enable deterministic verification
    #[test]
    fn test_rand_basic() {
        reset_fakes();

        // Seed the system time and the hardware RNG with fixed values
        fake_clock::set_ticks(0x1234);
        FakeMmio::set_reg16(RND_NUMBER, 0xABCD);

        // Calculate expected result: time (truncated to 16 bits) XOR hardware RNG
        let expected = 0x1234_u16 ^ 0xABCD;

        // Call the function we're testing
        let result = rand();

        // Verify result matches expected XOR combination
        assert_eq!(
            result, expected,
            "The random value should be the XOR of clock time and hardware RNG"
        );
    }

    /// Tests the rand function with different input values.
    ///
    /// This test verifies the behavior of rand() with different input values
    /// and confirms the XOR operation works correctly in all cases.
    ///
    /// # Algorithm
    ///
    /// 1. For each test case, seed the tick register and the RNG register
    /// 2. Call rand() and verify the result matches the expected XOR combination
    ///
    /// # Notes
    ///
    /// * This test checks boundary cases (0, max u16 value)
    /// * Tests cases where clock_time and RNG have the same/different values
    /// * Verifies the XOR operation works correctly for all input combinations
    #[test]
    fn test_rand_with_different_values() {
        reset_fakes();

        // Define test cases: (clock_time value, rng value, expected result)
        let test_cases = [
            // Zero values
            (0u32, 0u16, 0u16),
            // Maximum values (clock_time truncated to 16 bits)
            (0xFFFFFFFF, 0xFFFF, 0),
            // Only clock_time has value
            (0x5555, 0, 0x5555),
            // Only RNG has value
            (0, 0xAAAA, 0xAAAA),
            // Mixed values
            (0x1234, 0x5678, 0x1234 ^ 0x5678),
            // Large clock_time value (truncated to 16 bits)
            (0x12345678, 0xABCD, (0x5678 ^ 0xABCD)),
        ];

        // Run through all test cases
        for (i, &(time, rng, expected_result)) in test_cases.iter().enumerate() {
            fake_clock::set_ticks(time as u64);
            FakeMmio::set_reg16(RND_NUMBER, rng);

            // Call function under test
            let result = rand();

            // Verify correct XOR operation
            assert_eq!(
                result,
                expected_result,
                "Test case {}: For clock_time()=0x{:X} and RNG=0x{:X}, expected 0x{:X} but got 0x{:X}",
                i, time, rng, expected_result, result
            );
        }
    }

    /// Tests the randomness distribution of rand() with simulated values.
    ///
    /// This test verifies that the rand function produces a reasonably
    /// distributed set of random values when provided with varying input values.
    ///
    /// # Algorithm
    ///
    /// 1. Simulate a progressing system time and a cycling hardware RNG pattern
    /// 2. Generate multiple random numbers by calling rand()
    /// 3. Verify that the set of generated values has expected distribution properties
    ///
    /// # Notes
    ///
    /// * This test simulates time progression with incrementing values
    /// * RNG values alternate to simulate hardware randomness
    /// * We verify the distribution across high and low bits
    /// * This is a simple distribution test, not a full statistical analysis
    #[test]
    fn test_rand_distribution() {
        reset_fakes();

        // Simulate varying hardware RNG patterns
        let rng_patterns = [0x1234, 0xAAAA, 0x5555, 0xFFFF, 0x0000, 0x9876];

        // Generate a set of random numbers
        const SAMPLE_SIZE: usize = 100;
        let mut random_values = Vec::with_capacity(SAMPLE_SIZE);

        for i in 0..SAMPLE_SIZE {
            // Simulate a monotonically advancing system clock
            fake_clock::set_ticks(i as u64);
            FakeMmio::set_reg16(RND_NUMBER, rng_patterns[i % rng_patterns.len()]);
            random_values.push(rand());
        }

        // Basic distribution test - check for variance in the output
        // Count occurrences of set bits in specific positions
        let mut bit_counts = [0; 16];

        for value in &random_values {
            for (bit, count) in bit_counts.iter_mut().enumerate() {
                if (value & (1 << bit)) != 0 {
                    *count += 1;
                }
            }
        }

        // Verify reasonable distribution (not all bits are always 0 or always 1)
        // Each bit should be set in some values and clear in others
        for (bit_pos, &count) in bit_counts.iter().enumerate() {
            assert!(
                count > 0 && count < SAMPLE_SIZE,
                "Bit position {} is always {} across all samples",
                bit_pos,
                if count == 0 { 0 } else { 1 }
            );
        }

        // Additional check: verify we have a good variety of values
        let unique_values = random_values
            .iter()
            .collect::<std::collections::HashSet<_>>()
            .len();
        assert!(
            unique_values > SAMPLE_SIZE / 2,
            "Expected more unique random values, got only {}/{}",
            unique_values,
            SAMPLE_SIZE
        );
    }

    /// Integration test simulating complete random number generation from both sources.
    ///
    /// This test verifies the rand() function in a more realistic scenario by
    /// simulating progressive clock time changes and varying hardware RNG values.
    ///
    /// # Algorithm
    ///
    /// 1. Seed a sequence of changing values for both time and RNG sources
    /// 2. Call rand() multiple times and verify each result
    /// 3. Verify the overall pattern of random numbers generated
    ///
    /// # Notes
    ///
    /// * Simulates incrementing system time as would occur in real usage
    /// * Models hardware RNG values with varying patterns
    /// * Verifies changing inputs produce expected changing outputs
    /// * More comprehensive than individual unit tests
    #[test]
    fn test_rand_integration() {
        reset_fakes();

        // Create sequences of time and RNG values to simulate a calling pattern
        let time_sequence = [0x0001, 0x0002, 0x0003, 0x0004, 0x0005];
        let rng_sequence = [0xA000, 0xB000, 0xC000, 0xD000, 0xE000];

        // Pre-calculate expected results
        let expected_results: Vec<u16> = time_sequence
            .iter()
            .zip(rng_sequence.iter())
            .map(|(&t, &r)| (t as u16) ^ r)
            .collect();

        // Generate sequence of random numbers and verify against expected results
        for (i, expected) in expected_results.iter().enumerate() {
            fake_clock::set_ticks(time_sequence[i] as u64);
            FakeMmio::set_reg16(RND_NUMBER, rng_sequence[i]);

            let result = rand();
            assert_eq!(
                result, *expected,
                "Random value does not match expected pattern"
            );
        }
    }
}
