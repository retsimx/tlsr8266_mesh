//! Random number generation for TLSR8266 MCU.
//!
//! This module provides functions for generating pseudo-random numbers
//! suitable for non-cryptographic applications on the TLSR8266 platform.

use crate::sdk::mcu::clock::clock_time;
use crate::sdk::mcu::register::read_reg_rnd_number;

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
#[cfg_attr(test, mry::mry)]
pub fn rand() -> u16 {
    return clock_time() as u16 ^ read_reg_rnd_number();
}

#[cfg(test)]
mod tests {
    use super::*;
    
    // Import mocked versions of the functions from their original modules
    use crate::sdk::mcu::clock::mock_clock_time;
    use crate::sdk::mcu::register::mock_read_reg_rnd_number;

    /// Tests that the rand function correctly combines clock_time and hardware RNG values.
    ///
    /// This test verifies that the rand function properly:
    /// - Calls clock_time() function
    /// - Calls read_reg_rnd_number() function
    /// - Combines the values using XOR operation
    /// - Returns a 16-bit unsigned integer
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for clock_time and hardware RNG
    /// 2. Call rand() function to generate random number
    /// 3. Verify the returned value matches the expected XOR combination
    /// 4. Confirm both source functions were called exactly once
    ///
    /// # Notes
    ///
    /// * This test confirms the core algorithm of the rand function
    /// * The test uses fixed values to enable deterministic verification
    #[test]
    #[mry::lock(clock_time, read_reg_rnd_number)]
    fn test_rand_basic() {
        // Setup mock responses with fixed values for deterministic testing
        mock_clock_time().returns(0x1234);  // Mock system time
        mock_read_reg_rnd_number().returns(0xABCD);  // Mock hardware RNG
        
        // Calculate expected result: time (truncated to 16 bits) XOR hardware RNG
        let expected = (0x1234 as u16) ^ 0xABCD;
        
        // Call the function we're testing
        let result = rand();
        
        // Verify result matches expected XOR combination
        assert_eq!(result, expected, "The random value should be the XOR of clock time and hardware RNG");
        
        // Verify each dependency was called exactly once
        mock_clock_time().assert_called(1);
        mock_read_reg_rnd_number().assert_called(1);
    }

    /// Tests the rand function with different mock values.
    ///
    /// This test verifies the behavior of rand() with different input values
    /// and confirms the XOR operation works correctly in all cases.
    ///
    /// # Algorithm
    ///
    /// 1. Setup test cases with different combinations of time and RNG values
    /// 2. Create closures for mocks to return different values on each call
    /// 3. Call rand() multiple times to test various input combinations
    /// 4. Verify the results match expected XOR combinations
    ///
    /// # Notes
    ///
    /// * This test checks boundary cases (0, max u16 value)
    /// * Tests cases where clock_time and RNG have the same/different values
    /// * Verifies the XOR operation works correctly for all input combinations
    /// * Uses closures to provide different return values for each call
    #[test]
    #[mry::lock(clock_time, read_reg_rnd_number)]
    fn test_rand_with_different_values() {
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
            (0x12345678, 0xABCD, (0x5678 ^ 0xABCD))
        ];
        
        // Create vectors to hold the test values
        let time_values: Vec<u32> = test_cases.iter().map(|&(t, _, _)| t).collect();
        let rng_values: Vec<u16> = test_cases.iter().map(|&(_, r, _)| r).collect();
        let expected_results: Vec<u16> = test_cases.iter().map(|&(_, _, e)| e).collect();
       
        let time_values_copy = time_values.clone();
        let rng_values_copy = rng_values.clone();

        // Use closures to provide different return values on each call
        let mut time_index = 0;
        mock_clock_time().returns_with(move || {
            let value = time_values_copy[time_index];
            time_index += 1;
            value
        });
        
        let mut rng_index = 0;
        mock_read_reg_rnd_number().returns_with(move || {
            let value = rng_values_copy[rng_index];
            rng_index += 1;
            value
        });
        
        // Run through all test cases
        for (i, expected_result) in expected_results.iter().enumerate() {
            // Call function under test
            let result = rand();
            
            // Verify correct XOR operation
            assert_eq!(
                result, 
                *expected_result, 
                "Test case {}: For clock_time()=0x{:X} and RNG=0x{:X}, expected 0x{:X} but got 0x{:X}",
                i, time_values[i], rng_values[i], expected_result, result
            );
        }
        
        // Verify each source function was called the correct number of times
        mock_clock_time().assert_called(test_cases.len());
        mock_read_reg_rnd_number().assert_called(test_cases.len());
    }

    /// Tests the randomness distribution of rand() with simulated values.
    ///
    /// This test verifies that the rand function produces a reasonably
    /// distributed set of random values when provided with varying input values.
    ///
    /// # Algorithm
    ///
    /// 1. Setup simulation of changing time values and hardware RNG values
    /// 2. Generate multiple random numbers by calling rand()
    /// 3. Verify that the set of generated values has expected distribution properties
    /// 4. Confirm the source functions are called the expected number of times
    ///
    /// # Notes
    ///
    /// * This test simulates time progression with incrementing values
    /// * RNG values alternate to simulate hardware randomness
    /// * We verify the distribution across high and low bits
    /// * This is a simple distribution test, not a full statistical analysis
    #[test]
    #[mry::lock(clock_time, read_reg_rnd_number)]
    fn test_rand_distribution() {
        // Setup simulated time progression
        let mut simulated_time = 0u32;
        
        // Simulate changing clock time with a closure
        mock_clock_time().returns_with(move || {
            let current_time = simulated_time;
            simulated_time += 1;  // Increment for next call
            current_time
        });
        
        // Simulate varying hardware RNG patterns
        let rng_patterns = [0x1234, 0xAAAA, 0x5555, 0xFFFF, 0x0000, 0x9876];
        let mut rng_index = 0;
        
        mock_read_reg_rnd_number().returns_with(move || {
            let value = rng_patterns[rng_index % rng_patterns.len()];
            rng_index += 1;
            value
        });
        
        // Generate a set of random numbers
        const SAMPLE_SIZE: usize = 100;
        let mut random_values = Vec::with_capacity(SAMPLE_SIZE);
        
        for _ in 0..SAMPLE_SIZE {
            random_values.push(rand());
        }
        
        // Verify expected call counts
        mock_clock_time().assert_called(SAMPLE_SIZE);
        mock_read_reg_rnd_number().assert_called(SAMPLE_SIZE);
        
        // Basic distribution test - check for variance in the output
        // Count occurrences of set bits in specific positions
        let mut bit_counts = [0; 16];
        
        for value in &random_values {
            for bit in 0..16 {
                if (value & (1 << bit)) != 0 {
                    bit_counts[bit] += 1;
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
        let unique_values = random_values.iter().collect::<std::collections::HashSet<_>>().len();
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
    /// 1. Setup a sequence of changing values for both time and RNG sources
    /// 2. Create expected result patterns based on XOR of these values
    /// 3. Call rand() multiple times and verify each result
    /// 4. Verify the overall pattern of random numbers generated
    ///
    /// # Notes
    ///
    /// * Simulates incrementing system time as would occur in real usage
    /// * Models hardware RNG values with varying patterns
    /// * Verifies changing inputs produce expected changing outputs
    /// * More comprehensive than individual unit tests
    #[test]
    #[mry::lock(clock_time, read_reg_rnd_number)]
    fn test_rand_integration() {
        // Create sequences of time and RNG values to simulate a calling pattern
        let time_sequence = [0x0001, 0x0002, 0x0003, 0x0004, 0x0005];
        let rng_sequence = [0xA000, 0xB000, 0xC000, 0xD000, 0xE000];
        
        // Pre-calculate expected results
        let expected_results: Vec<u16> = time_sequence.iter()
            .zip(rng_sequence.iter())
            .map(|(&t, &r)| (t as u16) ^ r)
            .collect();
        
        // Setup mock with sequences
        let mut time_index = 0;
        mock_clock_time().returns_with(move || {
            let value = time_sequence[time_index];
            time_index = (time_index + 1) % time_sequence.len();
            value as u32
        });
        
        let mut rng_index = 0;
        mock_read_reg_rnd_number().returns_with(move || {
            let value = rng_sequence[rng_index];
            rng_index = (rng_index + 1) % rng_sequence.len();
            value
        });
        
        // Generate sequence of random numbers and verify against expected results
        for expected in &expected_results {
            let result = rand();
            assert_eq!(
                result, 
                *expected, 
                "Random value does not match expected pattern"
            );
        }
        
        // Verify correct number of calls to source functions
        mock_clock_time().assert_called(expected_results.len());
        mock_read_reg_rnd_number().assert_called(expected_results.len());
    }
}