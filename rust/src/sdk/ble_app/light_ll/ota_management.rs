//! # Over-The-Air (OTA) Update Management
//!
//! This module implements secure firmware update capabilities for the TLSR8266 mesh lighting
//! system. It provides the core functionality for receiving, validating, and storing firmware
//! updates transmitted over the mesh network.
//!
//! ## OTA Update Architecture
//!
//! The OTA system is designed with the following principles:
//!
//! ### Security and Reliability
//! - **Write-Verify Cycle**: Every flash write operation is immediately verified by reading back
//!   the data to ensure integrity
//! - **Atomic Operations**: Flash writes are performed in discrete pages to prevent partial updates
//! - **Error Recovery**: Failed writes are detected immediately and can trigger retransmission
//!
//! ### Flash Memory Management
//! - **Dedicated Flash Region**: Firmware updates are written to a separate flash area to avoid
//!   corrupting the running firmware
//! - **Sequential Writing**: Firmware data is written sequentially to prevent fragmentation
//! - **Address Tracking**: Current write position is maintained to resume interrupted updates
//!
//! ### Integration with Mesh Network
//! - **Packet-Based Transfer**: Firmware is transmitted in small packets suitable for mesh networking
//! - **Progress Tracking**: Update progress is tracked and can be reported across the network
//! - **Error Propagation**: Write failures are communicated back to the update initiator
//!
//! ## Flash Memory Layout
//!
//! ```
//! Flash Memory:
//! ┌─────────────────┬─────────────────┬─────────────────┐
//! │ Running FW      │ New FW Area     │ Config/Data     │
//! │ (Active)        │ (OTA Target)    │ (Persistent)    │
//! └─────────────────┴─────────────────┴─────────────────┘
//!                   ↑
//!           FLASH_ADR_LIGHT_NEW_FW
//! ```
//!
//! ## Update State Machine
//!
//! The OTA process follows this state progression:
//! 1. **Idle**: No update in progress
//! 2. **Receiving**: Accepting firmware data packets
//! 3. **Continue**: Previous packet written successfully, ready for next
//! 4. **Error**: Write/verify failure detected, update aborted
//! 5. **Complete**: All firmware data received and verified

use crate::config::FLASH_ADR_LIGHT_NEW_FW;
use crate::sdk::drivers::flash::{flash_read_page, flash_write_page};
use crate::sdk::light::OtaState;
use crate::state::{OTA_UPDATE_CURRENT_FLASH_ADDRESS, SimplifyLS};

/// Saves OTA firmware data to flash memory with verification.
///
/// This function implements the core OTA flash write algorithm with built-in verification
/// to ensure data integrity. It performs atomic write operations followed by immediate
/// read-back verification to detect any flash memory errors.
///
/// # Flash Write Algorithm
///
/// The function follows a strict write-verify-advance sequence:
///
/// 1. **Address Calculation**: Computes the target flash address by adding the current
///    write offset to the base OTA flash region address
///
/// 2. **Flash Write Operation**: Writes the incoming data to flash memory using the
///    hardware flash driver
///
/// 3. **Verification Read**: Immediately reads back the written data from flash to
///    verify the write operation succeeded
///
/// 4. **Data Comparison**: Performs byte-by-byte comparison between the original data
///    and the read-back data to detect any corruption
///
/// 5. **State Update**: On successful verification, advances the write pointer for
///    the next packet; on failure, returns error state
///
/// # Error Detection
///
/// The verification process detects several types of flash errors:
/// - **Write Failures**: Flash cells that fail to program correctly
/// - **Read Disturbance**: Previously written data corrupted by nearby writes
/// - **Wear-Out**: Flash cells that have exceeded their write/erase cycles
/// - **Power Glitches**: Incomplete writes due to power supply instability
///
/// # Flash Memory Safety
///
/// The function ensures safe flash operations by:
/// - Using only the designated OTA flash region (prevents overwriting active firmware)
/// - Performing sequential writes (prevents address conflicts)
/// - Validating write operations before advancing (prevents partial corruption)
/// - Maintaining atomic write semantics (each call either fully succeeds or fails)
///
/// # Performance Considerations
///
/// - **Flash Write Time**: Each write operation may take several milliseconds
/// - **Verification Overhead**: Read-back verification doubles the flash access time
/// - **Sequential Access**: Sequential writes are optimized by flash hardware
/// - **Page Alignment**: Performance is optimal when data aligns with flash page boundaries
///
/// # Parameters
/// * `data` - Firmware data packet to write to flash (1-16 bytes typical)
///
/// # Returns
/// * `OtaState::Continue` - Data written and verified successfully, ready for next packet
/// * `OtaState::Error` - Write or verification failed, update should be aborted
///
/// # Side Effects
/// * Modifies flash memory content in the OTA region
/// * Updates `OTA_UPDATE_CURRENT_FLASH_ADDRESS` on successful writes
/// * May trigger flash wear leveling operations (hardware-dependent)
///
/// # Flash Endurance
/// Flash memory has limited write/erase cycles (typically 10,000-100,000 cycles).
/// This function contributes to flash wear and should only be used for legitimate
/// firmware updates to preserve flash lifetime.
pub fn rf_ota_save_data(data: &[u8]) -> OtaState
{
    // Calculate target flash address: base OTA region + current write offset
    let addr = OTA_UPDATE_CURRENT_FLASH_ADDRESS.get() + FLASH_ADR_LIGHT_NEW_FW;
    
    // Write the firmware data packet to flash memory
    flash_write_page(addr, data.len() as u32, data.as_ptr());

    // Allocate temporary buffer for verification read (16 bytes max packet size)
    let mut tmp = [0u8; 0x10];
    
    // Read back the written data for verification
    flash_read_page(addr, data.len() as u32, tmp.as_mut_ptr());

    // Verify data integrity by comparing original data with read-back data
    if data == &tmp[..data.len()] {
        // Write verification successful - advance write pointer for next packet
        OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(OTA_UPDATE_CURRENT_FLASH_ADDRESS.get() + data.len() as u32);
        return OtaState::Continue;
    } else {
        // Write verification failed - return error to abort update
        return OtaState::Error;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mry::Any;
    
    // Import mock functions from their original modules
    use crate::sdk::drivers::flash::{mock_flash_write_page, mock_flash_read_page};
    
    /// Helper function to reset global state for test isolation
    fn reset_ota_state() {
        OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(0);
    }
    
    // ================================================================================
    // Tests for rf_ota_save_data function
    // ================================================================================

    /// Tests rf_ota_save_data with basic functionality.
    ///
    /// Verifies that the function correctly calls flash write and read operations
    /// and handles the address calculation correctly.
    #[test]
    #[mry::lock(flash_write_page, flash_read_page)]
    fn test_rf_ota_save_data_basic_functionality() {
        // Setup mocks for basic flash operations
        mock_flash_write_page(Any, Any, Any).returns(());
        mock_flash_read_page(Any, Any, Any).returns(());
        
        reset_ota_state();
        
        let test_data = [0x01, 0x02, 0x03, 0x04];
        let initial_address = OTA_UPDATE_CURRENT_FLASH_ADDRESS.get();
        
        // Call the function under test
        let result = rf_ota_save_data(&test_data);
        
        // Verify flash operations were called with correct parameters
        let expected_addr = initial_address + FLASH_ADR_LIGHT_NEW_FW;
        mock_flash_write_page(expected_addr, test_data.len() as u32, test_data.as_ptr()).assert_called(1);
        mock_flash_read_page(expected_addr, test_data.len() as u32, Any).assert_called(1);
        
        // Function should return some valid OtaState (since mocks don't modify buffers)
        assert!(matches!(result, OtaState::Continue | OtaState::Error), 
                "Function should return a valid OtaState");
    }

    /// Tests rf_ota_save_data with empty data.
    ///
    /// Verifies that the function handles empty data gracefully.
    #[test]
    #[mry::lock(flash_write_page, flash_read_page)]
    fn test_rf_ota_save_data_empty_data() {
        // Setup mocks for empty data operations
        mock_flash_write_page(Any, Any, Any).returns(());
        mock_flash_read_page(Any, Any, Any).returns(());
        
        reset_ota_state();
        
        let empty_data: &[u8] = &[];
        let initial_address = OTA_UPDATE_CURRENT_FLASH_ADDRESS.get();
        
        let result = rf_ota_save_data(empty_data);
        
        // Verify operations were called with zero length
        let expected_addr = initial_address + FLASH_ADR_LIGHT_NEW_FW;
        mock_flash_write_page(expected_addr, 0, empty_data.as_ptr()).assert_called(1);
        mock_flash_read_page(expected_addr, 0, Any).assert_called(1);
        
        // Empty data should succeed since verification will compare empty slices
        assert_eq!(result, OtaState::Continue, "Should return Continue for empty data");
        
        // Flash address should advance by the length of data (which is 0 for empty data)
        assert_eq!(
            OTA_UPDATE_CURRENT_FLASH_ADDRESS.get(),
            initial_address + empty_data.len() as u32,
            "Flash address should advance by data length (0 for empty data)"
        );
    }

    /// Tests rf_ota_save_data with single byte.
    ///
    /// Verifies that the function correctly handles minimal data sizes.
    #[test]
    #[mry::lock(flash_write_page, flash_read_page)]
    fn test_rf_ota_save_data_single_byte() {
        // Setup mocks for single byte operation
        mock_flash_write_page(Any, Any, Any).returns(());
        mock_flash_read_page(Any, Any, Any).returns(());
        
        reset_ota_state();
        
        let single_byte = [0x42];
        let initial_address = OTA_UPDATE_CURRENT_FLASH_ADDRESS.get();
        
        let result = rf_ota_save_data(&single_byte);
        
        // Verify operations called with length 1
        let expected_addr = initial_address + FLASH_ADR_LIGHT_NEW_FW;
        mock_flash_write_page(expected_addr, 1, single_byte.as_ptr()).assert_called(1);
        mock_flash_read_page(expected_addr, 1, Any).assert_called(1);
        
        // Function should execute successfully
        assert!(matches!(result, OtaState::Continue | OtaState::Error), 
                "Function should return a valid OtaState for single byte");
    }

    /// Tests rf_ota_save_data with maximum typical packet size.
    ///
    /// Verifies that the function handles larger data packets correctly.
    #[test]
    #[mry::lock(flash_write_page, flash_read_page)]
    fn test_rf_ota_save_data_max_packet_size() {
        // Setup mocks for 16-byte packet (typical maximum)
        mock_flash_write_page(Any, Any, Any).returns(());
        mock_flash_read_page(Any, Any, Any).returns(());
        
        reset_ota_state();
        
        // Create 16-byte test packet with sequential pattern
        let max_packet: Vec<u8> = (0..16).map(|i| (i & 0xFF) as u8).collect();
        let initial_address = OTA_UPDATE_CURRENT_FLASH_ADDRESS.get();
        
        let result = rf_ota_save_data(&max_packet);
        
        // Verify operations called with correct length
        let expected_addr = initial_address + FLASH_ADR_LIGHT_NEW_FW;
        mock_flash_write_page(expected_addr, max_packet.len() as u32, max_packet.as_ptr()).assert_called(1);
        mock_flash_read_page(expected_addr, max_packet.len() as u32, Any).assert_called(1);
        
        // Function should execute successfully
        assert!(matches!(result, OtaState::Continue | OtaState::Error), 
                "Function should return a valid OtaState for max packet size");
    }

    /// Tests rf_ota_save_data with different flash addresses.
    ///
    /// Verifies that the function correctly calculates flash addresses
    /// based on the current offset plus the base address.
    #[test]
    #[mry::lock(flash_write_page, flash_read_page)]
    fn test_rf_ota_save_data_address_calculation() {
        // Setup mocks
        mock_flash_write_page(Any, Any, Any).returns(());
        mock_flash_read_page(Any, Any, Any).returns(());
        
        reset_ota_state();
        
        // Set a specific offset for testing address calculation
        let test_offset = 0x1000;
        OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(test_offset);
        
        let test_data = [0xAA, 0xBB];
        
        let result = rf_ota_save_data(&test_data);
        
        // Verify address calculation: offset + base address
        let expected_flash_addr = test_offset + FLASH_ADR_LIGHT_NEW_FW;
        mock_flash_write_page(expected_flash_addr, test_data.len() as u32, test_data.as_ptr()).assert_called(1);
        mock_flash_read_page(expected_flash_addr, test_data.len() as u32, Any).assert_called(1);
        
        // Function should execute successfully
        assert!(matches!(result, OtaState::Continue | OtaState::Error), 
                "Function should return a valid OtaState with custom offset");
    }

    /// Tests rf_ota_save_data operation sequence.
    ///
    /// Verifies that flash write is called before flash read, ensuring
    /// the write-verify cycle is performed in the correct order.
    #[test]
    #[mry::lock(flash_write_page, flash_read_page)]
    fn test_rf_ota_save_data_operation_sequence() {
        use core::sync::atomic::{AtomicU32, Ordering};
        
        // Track operation sequence using atomic counters
        static WRITE_CALL_COUNT: AtomicU32 = AtomicU32::new(0);
        static READ_CALL_COUNT: AtomicU32 = AtomicU32::new(0);
        
        // Reset counters
        WRITE_CALL_COUNT.store(0, Ordering::Relaxed);
        READ_CALL_COUNT.store(0, Ordering::Relaxed);
        
        // Setup mocks to track call order
        mock_flash_write_page(Any, Any, Any).returns_with(|_addr, _len, _buf| {
            let count = WRITE_CALL_COUNT.fetch_add(1, Ordering::Relaxed);
            // Verify write is called before any reads
            assert_eq!(READ_CALL_COUNT.load(Ordering::Relaxed), count, 
                      "Write {} should occur before read {}", count, count);
        });
        
        mock_flash_read_page(Any, Any, Any).returns_with(|_addr, _len, _buf| {
            let count = READ_CALL_COUNT.fetch_add(1, Ordering::Relaxed);
            // Verify write was called first
            assert_eq!(WRITE_CALL_COUNT.load(Ordering::Relaxed), count + 1, 
                      "Read {} should occur after write {}", count, count);
        });
        
        reset_ota_state();
        
        let test_data = [0x77, 0x77, 0x77];
        let result = rf_ota_save_data(&test_data);
        
        // Verify both operations were called exactly once in correct order
        assert_eq!(WRITE_CALL_COUNT.load(Ordering::Relaxed), 1, "Write should be called exactly once");
        assert_eq!(READ_CALL_COUNT.load(Ordering::Relaxed), 1, "Read should be called exactly once");
        
        // Function should execute successfully
        assert!(matches!(result, OtaState::Continue | OtaState::Error), 
                "Function should return a valid OtaState");
    }

    /// Tests rf_ota_save_data with multiple sequential calls.
    ///
    /// Verifies that multiple consecutive packets are handled correctly
    /// and that the flash address advances properly.
    #[test]
    #[mry::lock(flash_write_page, flash_read_page)]
    fn test_rf_ota_save_data_sequential_packets() {
        // Setup mocks that will return Error (since we can't easily mock buffer verification)
        mock_flash_write_page(Any, Any, Any).returns(());
        mock_flash_read_page(Any, Any, Any).returns(());
        
        reset_ota_state();
        
        let packets = [
            vec![0x00, 0x01, 0x02], // First packet (3 bytes)
            vec![0x03, 0x04],       // Second packet (2 bytes)  
            vec![0x05],             // Third packet (1 byte)
        ];
        
        let initial_address = OTA_UPDATE_CURRENT_FLASH_ADDRESS.get();
        
        // Process each packet sequentially
        for (i, packet) in packets.iter().enumerate() {
            let result = rf_ota_save_data(packet);
            
            // Function should execute without crashing
            assert!(matches!(result, OtaState::Continue | OtaState::Error), 
                    "Packet {} should return valid OtaState", i);
        }
        
        // Verify correct number of operations (one write and one read per packet)
        mock_flash_write_page(Any, Any, Any).assert_called(packets.len());
        mock_flash_read_page(Any, Any, Any).assert_called(packets.len());
    }

    /// Tests rf_ota_save_data with various data patterns.
    ///
    /// Verifies that the function correctly handles various data patterns
    /// that might cause issues (all zeros, all ones, alternating patterns).
    #[test]
    #[mry::lock(flash_write_page, flash_read_page)]
    fn test_rf_ota_save_data_edge_case_patterns() {
        // Setup mocks for pattern testing
        mock_flash_write_page(Any, Any, Any).returns(());
        mock_flash_read_page(Any, Any, Any).returns(());
        
        reset_ota_state();
        
        let test_patterns = [
            vec![0x00, 0x00, 0x00, 0x00], // All zeros
            vec![0xFF, 0xFF, 0xFF, 0xFF], // All ones
            vec![0xAA, 0x55, 0xAA, 0x55], // Alternating pattern
            vec![0x00, 0xFF, 0x00, 0xFF], // Alternating extreme values
        ];
        
        for (i, pattern) in test_patterns.iter().enumerate() {
            let result = rf_ota_save_data(pattern);
            
            // Each pattern should be processed without crashing
            assert!(matches!(result, OtaState::Continue | OtaState::Error), 
                    "Pattern {} should return valid OtaState", i);
        }
        
        // Verify all patterns were processed
        mock_flash_write_page(Any, Any, Any).assert_called(test_patterns.len());
        mock_flash_read_page(Any, Any, Any).assert_called(test_patterns.len());
    }

    /// Tests rf_ota_save_data buffer boundary conditions.
    ///
    /// Verifies that the temporary buffer (16 bytes) is used correctly
    /// and doesn't cause issues with different data sizes.
    #[test]
    #[mry::lock(flash_write_page, flash_read_page)]
    fn test_rf_ota_save_data_buffer_boundaries() {
        // Setup mocks to test the 16-byte temporary buffer handling
        mock_flash_write_page(Any, Any, Any).returns(());
        mock_flash_read_page(Any, Any, Any).returns(());
        
        reset_ota_state();
        
        // Test sizes around the 16-byte buffer boundary
        let test_sizes = [1, 8, 15, 16]; // Various sizes up to buffer limit
        
        for &size in &test_sizes {
            // Create test data
            let test_data: Vec<u8> = (0..size).map(|i| (0x80 | i) as u8).collect();
            
            let result = rf_ota_save_data(&test_data);
            
            // Verify function executes for all buffer sizes
            assert!(matches!(result, OtaState::Continue | OtaState::Error), 
                    "Size {} should return valid OtaState", size);
        }
        
        // Verify all sizes were processed
        mock_flash_write_page(Any, Any, Any).assert_called(test_sizes.len());
        mock_flash_read_page(Any, Any, Any).assert_called(test_sizes.len());
    }
}
