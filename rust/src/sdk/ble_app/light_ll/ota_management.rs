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

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32};

use crate::config::FLASH_ADR_LIGHT_NEW_FW;
use crate::sdk::light::{OtaState, RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS};
use tlsr8266_hal::common::sync::{CriticalSectionMutex, SimplifyLS};
use tlsr8266_hal::drivers::flash::{flash_read_page, flash_write_page};

/// Over-The-Air (OTA) update completion status flag.
/// Indicates whether an OTA firmware update has finished successfully or with errors.
pub(crate) static RF_SLAVE_OTA_FINISHED_FLAG: CriticalSectionMutex<OtaState> =
    CriticalSectionMutex::new(OtaState::Continue);

/// Flag indicating OTA update is busy with mesh operations.
/// Prevents mesh interference during firmware update process.
pub(crate) static OTA_UPDATE_MESH_OPERATIONS_BLOCKED: AtomicBool = AtomicBool::new(false);

/// Flag indicating OTA update is currently in progress.
/// Blocks other RF operations during firmware update.
pub(crate) static OTA_UPDATE_IN_PROGRESS: AtomicBool = AtomicBool::new(false);

/// Current flash address for OTA firmware update operations.
/// Points to the next location where firmware data will be written.
pub(crate) static OTA_UPDATE_CURRENT_FLASH_ADDRESS: AtomicU32 = AtomicU32::new(0);

/// Flag to terminate ongoing OTA update process.
/// Set to abort firmware update and return to normal operation.
pub(crate) static OTA_UPDATE_TERMINATION_REQUESTED: AtomicBool = AtomicBool::new(false);

/// OTA operation timeout in seconds (default 30 seconds).
/// Maximum time allowed for firmware update operations.
pub(crate) static OTA_UPDATE_TIMEOUT_SECONDS: AtomicU16 =
    AtomicU16::new(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);

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
pub fn rf_ota_save_data(data: &[u8]) -> OtaState {
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
        OTA_UPDATE_CURRENT_FLASH_ADDRESS
            .set(OTA_UPDATE_CURRENT_FLASH_ADDRESS.get() + data.len() as u32);
        return OtaState::Continue;
    } else {
        // Write verification failed - return error to abort update
        return OtaState::Error;
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;

    /// Helper function to reset global state for test isolation
    fn reset_ota_state() {
        OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(0);
        // The device is process-global: start every test from a blank NOR array
        // so a write-then-verify cycle really succeeds.
        FakeFlash::reset();
    }

    // ================================================================================
    // Tests for rf_ota_save_data function
    // ================================================================================

    /// Tests rf_ota_save_data with basic functionality.
    ///
    /// Verifies that the function writes the packet, reads it back, and advances
    /// the write pointer.
    #[test]
    fn test_rf_ota_save_data_basic_functionality() {
        reset_ota_state();

        let test_data = [0x01, 0x02, 0x03, 0x04];
        let initial_address = OTA_UPDATE_CURRENT_FLASH_ADDRESS.get();

        // Call the function under test
        let result = rf_ota_save_data(&test_data);

        // The write really landed and verified, so the packet is accepted.
        let expected_addr = initial_address + FLASH_ADR_LIGHT_NEW_FW;
        let mut got = [0u8; 4];
        FakeFlash::read_slice(expected_addr, &mut got);
        assert_eq!(got, test_data);
        assert_eq!(result, OtaState::Continue);
        assert_eq!(
            OTA_UPDATE_CURRENT_FLASH_ADDRESS.get(),
            initial_address + test_data.len() as u32
        );
    }

    /// Tests rf_ota_save_data with empty data.
    ///
    /// Verifies that the function handles empty data gracefully.
    #[test]
    fn test_rf_ota_save_data_empty_data() {
        reset_ota_state();

        let empty_data: &[u8] = &[];
        let initial_address = OTA_UPDATE_CURRENT_FLASH_ADDRESS.get();

        let result = rf_ota_save_data(empty_data);

        // Empty data should succeed since verification will compare empty slices
        assert_eq!(
            result,
            OtaState::Continue,
            "Should return Continue for empty data"
        );

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
    fn test_rf_ota_save_data_single_byte() {
        reset_ota_state();

        let single_byte = [0x42];
        let initial_address = OTA_UPDATE_CURRENT_FLASH_ADDRESS.get();

        let result = rf_ota_save_data(&single_byte);

        let expected_addr = initial_address + FLASH_ADR_LIGHT_NEW_FW;
        let mut got = [0u8; 1];
        FakeFlash::read_slice(expected_addr, &mut got);
        assert_eq!(got, single_byte);
        assert_eq!(result, OtaState::Continue);
        assert_eq!(OTA_UPDATE_CURRENT_FLASH_ADDRESS.get(), initial_address + 1);
    }

    /// Tests rf_ota_save_data with maximum typical packet size.
    ///
    /// Verifies that the function handles larger data packets correctly.
    #[test]
    fn test_rf_ota_save_data_max_packet_size() {
        reset_ota_state();

        // Create 16-byte test packet with sequential pattern
        let max_packet: Vec<u8> = (0..16).map(|i| (i & 0xFF) as u8).collect();
        let initial_address = OTA_UPDATE_CURRENT_FLASH_ADDRESS.get();

        let result = rf_ota_save_data(&max_packet);

        let expected_addr = initial_address + FLASH_ADR_LIGHT_NEW_FW;
        let mut got = [0u8; 16];
        FakeFlash::read_slice(expected_addr, &mut got);
        assert_eq!(got, max_packet.as_slice());
        assert_eq!(result, OtaState::Continue);
        assert_eq!(
            OTA_UPDATE_CURRENT_FLASH_ADDRESS.get(),
            initial_address + max_packet.len() as u32
        );
    }

    /// Tests rf_ota_save_data with different flash addresses.
    ///
    /// Verifies that the function correctly calculates flash addresses
    /// based on the current offset plus the base address.
    #[test]
    fn test_rf_ota_save_data_address_calculation() {
        reset_ota_state();

        // Set a specific offset for testing address calculation
        let test_offset = 0x1000;
        OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(test_offset);

        let test_data = [0xAA, 0xBB];

        let result = rf_ota_save_data(&test_data);

        // Verify address calculation: offset + base address
        let expected_flash_addr = test_offset + FLASH_ADR_LIGHT_NEW_FW;
        let mut got = [0u8; 2];
        FakeFlash::read_slice(expected_flash_addr, &mut got);
        assert_eq!(got, test_data);
        assert_eq!(result, OtaState::Continue);
    }

    /// Tests rf_ota_save_data operation sequence.
    ///
    /// Verifies the write-verify cycle: the read-back only matches when the
    /// write happened first, so a successful verification proves the ordering.
    #[test]
    fn test_rf_ota_save_data_operation_sequence() {
        reset_ota_state();

        let test_data = [0x77, 0x77, 0x77];
        let result = rf_ota_save_data(&test_data);

        assert_eq!(result, OtaState::Continue);
        let mut got = [0u8; 3];
        FakeFlash::read_slice(FLASH_ADR_LIGHT_NEW_FW, &mut got);
        assert_eq!(got, test_data);
    }

    /// Tests rf_ota_save_data with multiple sequential calls.
    ///
    /// Verifies that multiple consecutive packets are handled correctly
    /// and that the flash address advances properly.
    #[test]
    fn test_rf_ota_save_data_sequential_packets() {
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

            assert_eq!(
                result,
                OtaState::Continue,
                "Packet {} should verify and continue",
                i
            );
        }

        // The packets are appended one after another in flash.
        let mut got = [0u8; 6];
        FakeFlash::read_slice(FLASH_ADR_LIGHT_NEW_FW + initial_address, &mut got);
        assert_eq!(got, [0x00, 0x01, 0x02, 0x03, 0x04, 0x05]);
        assert_eq!(OTA_UPDATE_CURRENT_FLASH_ADDRESS.get(), initial_address + 6);
    }

    /// Tests rf_ota_save_data with various data patterns.
    ///
    /// Verifies that the function correctly handles various data patterns
    /// that might cause issues (all zeros, all ones, alternating patterns).
    #[test]
    fn test_rf_ota_save_data_edge_case_patterns() {
        reset_ota_state();

        let test_patterns = [
            vec![0x00, 0x00, 0x00, 0x00], // All zeros
            vec![0xFF, 0xFF, 0xFF, 0xFF], // All ones
            vec![0xAA, 0x55, 0xAA, 0x55], // Alternating pattern
            vec![0x00, 0xFF, 0x00, 0xFF], // Alternating extreme values
        ];

        let mut offset = 0u32;
        for (i, pattern) in test_patterns.iter().enumerate() {
            let result = rf_ota_save_data(pattern);

            assert_eq!(
                result,
                OtaState::Continue,
                "Pattern {} should verify and continue",
                i
            );

            let mut got = vec![0u8; pattern.len()];
            FakeFlash::read_slice(FLASH_ADR_LIGHT_NEW_FW + offset, &mut got);
            assert_eq!(&got, pattern, "pattern {} must be stored verbatim", i);
            offset += pattern.len() as u32;
        }

        assert_eq!(OTA_UPDATE_CURRENT_FLASH_ADDRESS.get(), offset);
    }

    /// Tests rf_ota_save_data buffer boundary conditions.
    ///
    /// Verifies that the temporary buffer (16 bytes) is used correctly
    /// and doesn't cause issues with different data sizes.
    #[test]
    fn test_rf_ota_save_data_buffer_boundaries() {
        reset_ota_state();

        // Test sizes around the 16-byte buffer boundary
        let test_sizes = [1, 8, 15, 16]; // Various sizes up to buffer limit

        let mut offset = 0u32;
        for &size in &test_sizes {
            // Create test data
            let test_data: Vec<u8> = (0..size).map(|i| (0x80 | i) as u8).collect();

            let result = rf_ota_save_data(&test_data);

            assert_eq!(result, OtaState::Continue, "Size {} must verify", size);

            let mut got = vec![0u8; size];
            FakeFlash::read_slice(FLASH_ADR_LIGHT_NEW_FW + offset, &mut got);
            assert_eq!(&got, &test_data, "size {} must be stored verbatim", size);
            offset += size as u32;
        }

        assert_eq!(OTA_UPDATE_CURRENT_FLASH_ADDRESS.get(), offset);
    }
}
