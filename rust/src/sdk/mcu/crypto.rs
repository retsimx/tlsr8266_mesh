use core::cmp::min;
use core::ptr::addr_of;
use crate::sdk::mcu::register::{read_reg_aes_ctrl, read_reg_aes_data, write_reg_aes_ctrl, write_reg_aes_data, write_reg_aes_key};
use crate::state::{PAIR_CONFIG_PWD_ENCODE_SK};

/// Enum representing the direction of AES operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AesDirection {
    Encrypt = 0,
    Decrypt = 1,
}

/// Low-level AES encryption/decryption function for the TLSR8266 hardware.
///
/// This function performs AES operations using the hardware crypto module of the TLSR8266 chip.
/// It handles both encryption and decryption operations depending on the direction parameter.
/// The function implements the necessary register operations and byte order conversions required
/// by the TLSR8266 hardware implementation.
///
/// # Parameters
/// * `key` - The AES key (must be at least 16 bytes)
/// * `source` - The data to encrypt/decrypt (must be at least 16 bytes)
/// * `direction` - Operation mode: `AesDirection::Encrypt` or `AesDirection::Decrypt`
///
/// # Returns
/// A 16-byte array containing the encrypted/decrypted result
///
/// # AES Implementation Notes
/// - Uses hardware AES acceleration
/// - The TLSR8266 requires keys to be written in reverse byte order
/// - Data is written and read in 32-bit words with specific endianness requirements
/// - The implementation follows TLSR8266 hardware-specific register layout and timing
#[cfg_attr(test, mry::mry)] // Added for mocking in tests
pub fn aes_ll_encryption(key: &[u8], source: &[u8], direction: AesDirection) -> [u8; 16]
{
    let mut result = [0u8; 16];

    // Execute AES operation in a critical section to prevent interrupts
    // during hardware crypto operations
    critical_section::with(|_| {
        // Set operation mode based on the enum variant
        write_reg_aes_ctrl(direction as u8);

        // Write key to AES key registers
        // Note: TLSR8266 requires writing the key in reverse order (15..0)
        for i in 0..16 {
            write_reg_aes_key(key[15 - i], i as u32);
        }

        // Write source data to AES data registers
        // Each register accepts a 32-bit word, and we need to write 4 words (16 bytes)
        // Note: Data is written in big-endian format with the most significant byte first
        // Loop goes backwards through the indexes to write from high to low addresses
        for idx in (0..16).step_by(4).rev() {
            write_reg_aes_data(
                (source[idx] as u32) << 0x18 |        // MSB (most significant byte)
                    ((source[idx + 1] as u32) << 0x10) | // 2nd byte
                    ((source[idx + 2] as u32) << 0x8) |  // 3rd byte
                    (source[idx + 3] as u32)             // LSB (least significant byte)
            );
        }

        // Wait for AES operation to complete
        // Bit 2 (value 4) of the AES_CTRL register is set when operation is done
        while read_reg_aes_ctrl() & 4 == 0 {}

        // Read the result from AES data registers
        // Each register returns a 32-bit word, and we need to read 4 words (16 bytes)
        // Convert from big-endian 32-bit words to byte array
        for idx in (0..16).step_by(4) {
            let tmp = read_reg_aes_data();

            // Store result in destination buffer
            // Extract individual bytes from the 32-bit register value
            // in little-endian order (LSB first)
            result[idx] = tmp as u8;                 // LSB (least significant byte)
            result[idx + 1] = (tmp >> 8) as u8;      // 2nd byte
            result[idx + 2] = (tmp >> 16) as u8;     // 3rd byte
            result[idx + 3] = (tmp >> 24) as u8;     // MSB (most significant byte)
        }
    });
    
    result
}



/// Performs AES-128 encryption suitable for Bluetooth Attribute Protocol (ATT) operations.
///
/// This function encrypts a 16-byte block of data using a provided 16-byte key.
/// It utilizes the low-level hardware-accelerated AES encryption (`aes_ll_encryption`)
/// and then reverses the byte order of the result.
/// The byte reversal is necessary to match the byte order expected by certain
/// hardware or protocol layers, particularly in the context of the TLSR8266 chip.
///
/// # Algorithm
/// 1. Call `aes_ll_encryption` with the provided key, source data, and `AesDirection::Encrypt`.
/// 2. Reverse the byte order of the 16-byte result from the encryption step.
/// 3. Return the byte-reversed encrypted data.
///
/// # Parameters
/// * `key` - A 16-byte array representing the AES encryption key.
/// * `source` - A 16-byte array representing the plaintext data to be encrypted.
///
/// # Returns
/// A 16-byte array containing the encrypted and byte-swapped data.
#[cfg_attr(test, mry::mry)]
pub fn aes_att_encryption(key: &[u8; 16], source: &[u8; 16]) -> [u8; 16]
{
    // Perform low-level encryption using hardware acceleration.
    let mut result = aes_ll_encryption(key, source, AesDirection::Encrypt);
    // Reverse the bytes of the result to match the required endianness/format.
    // This is often needed for compatibility with specific hardware or protocols.
    result.reverse();
    // Return the final encrypted and byte-reversed result.
    result
}

/// Performs AES-128 decryption suitable for Bluetooth Attribute Protocol (ATT) operations.
///
/// This function decrypts a 16-byte block of data using a provided 16-byte key.
/// It utilizes the low-level hardware-accelerated AES decryption (`aes_ll_encryption`
/// with `AesDirection::Decrypt`) and then reverses the byte order of the result.
/// The byte reversal corrects the byte order after the hardware
/// decryption to match the expected format.
///
/// # Algorithm
/// 1. Call `aes_ll_encryption` with the provided key, source data (ciphertext), and `AesDirection::Decrypt`.
/// 2. Reverse the byte order of the 16-byte result from the decryption step.
/// 3. Return the byte-reversed decrypted data (plaintext).
///
/// # Parameters
/// * `key` - A 16-byte array representing the AES decryption key.
/// * `source` - A 16-byte array representing the ciphertext data to be decrypted.
///
/// # Returns
/// A 16-byte array containing the decrypted and byte-swapped data (plaintext).
#[cfg_attr(test, mry::mry)]
pub fn aes_att_decryption(key: &[u8; 16], source: &[u8; 16]) -> [u8; 16]
{
    // Perform low-level decryption using hardware acceleration.
    // Note: The same low-level function is used for both encryption and decryption,
    // controlled by the AesDirection enum.
    let mut result = aes_ll_encryption(key, source, AesDirection::Decrypt);
    // Reverse the bytes of the result to match the required endianness/format.
    result.reverse();
    // Return the final decrypted and byte-reversed result.
    result
}

/// Encodes (encrypts) a 16-byte password using a predefined global key.
///
/// This function retrieves a specific, globally defined AES key (`PAIR_CONFIG_PWD_ENCODE_SK`)
/// and uses it to encrypt the provided password data via `aes_att_encryption`.
/// This is typically used for securing passwords during pairing or configuration processes.
///
/// # Parameters
/// * `password` - A 16-byte array representing the plaintext password to be encoded.
///
/// # Returns
/// A 16-byte array containing the encrypted password.
#[cfg_attr(test, mry::mry)]
pub fn encode_password(password: &[u8; 16]) -> [u8; 16]
{
    // Lock the mutex protecting the global password encoding key.
    // This ensures thread-safe access to the key.
    let key_array: &[u8; 16] = &*PAIR_CONFIG_PWD_ENCODE_SK.lock();
    // Perform AES ATT encryption using the retrieved global key and the provided password.
    aes_att_encryption(key_array, password)
}

/// Decodes (decrypts) a 16-byte password using a predefined global key.
///
/// This function retrieves a specific, globally defined AES key (`PAIR_CONFIG_PWD_ENCODE_SK`)
/// and uses it to decrypt the provided encoded password data via `aes_att_decryption`.
/// This is the counterpart to `encode_password` and is used to recover the original
/// password from its encrypted form.
///
/// # Parameters
/// * `password` - A 16-byte array representing the encrypted password to be decoded.
///
/// # Returns
/// A 16-byte array containing the decrypted (plaintext) password.
#[cfg_attr(test, mry::mry)]
pub fn decode_password(password: &[u8; 16]) -> [u8; 16]
{
    // Lock the mutex protecting the global password encoding/decoding key.
    let key_array: &[u8; 16] = &*PAIR_CONFIG_PWD_ENCODE_SK.lock();
    // Perform AES ATT decryption using the retrieved global key and the provided encrypted password.
    aes_att_decryption(key_array, password)
}

/// Decrypts a packet using the AES-128 algorithm with authenticated encryption (CCM-like mode).
///
/// This function implements a combined AES counter (CTR) mode for decryption with a MAC
/// (Message Authentication Code) for integrity verification. The process follows these phases:
///
/// 1. For empty packets, generate authentication tag directly and verify against `expected_mic`
/// 2. For non-empty packets:
///   a. Phase 1: Decrypt the packet data using AES-CTR mode
///   b. Phase 2: Calculate authentication tag (MIC) on the decrypted data
///   c. Phase 3: Verify integrity by comparing the calculated MIC with `expected_mic`
///
/// A MIC (Message Integrity Check), also known as an authentication tag, is a short piece of
/// information used to authenticate a message—in other words, to confirm that the message
/// came from the stated sender (its authenticity) and has not been changed (its integrity).
///
/// The authentication tag is derived through a CBC-MAC-like process on the decrypted data,
/// and successful verification confirms both the data integrity and authenticity.
///
/// # Algorithm Details
/// - AES-CTR mode: Uses counter blocks with IVM as base and incrementing counter
/// - Authentication: Processes data in 16-byte blocks, applying AES after each block or at the end
/// - Verification: Compares the derived authentication tag (MIC) with the provided `expected_mic`
///
/// # Parameters
/// * `key` - The AES-128 encryption key (16 bytes)
/// * `ivm` - The initialization vector modifier (8 bytes)
/// * `expected_mic` - The expected MIC received with the packet (typically 2-4 bytes). Used for verification.
/// * `packet_data` - The encrypted packet data to be decrypted (modified in place)
///
/// # Returns
/// * `true` if the calculated MIC matches the `expected_mic` (verification passed)
/// * `false` if verification failed, indicating corrupted data or wrong encryption key
#[cfg_attr(test, mry::mry)]
pub fn aes_att_decryption_packet(key: &[u8; 16], ivm: &[u8; 8], expected_mic: &[u8], packet_data: &mut [u8]) -> bool
{
    // Phase 0: Handle empty packet special case
    if packet_data.len() == 0 {
        // For empty packets, create an authentication block with specific format
        // Format: [IVM(8 bytes) | zeros(8 bytes)] with packet_len = 0
        let mut ivs = [0u8; 16];
        ivs[0..8].copy_from_slice(ivm);
        ivs[8] = 0;    // packet_data.len() as u8
        ivs[12] = 0;   // Another zero at position 12 (specific to empty packets)
        
        // Generate authentication tag (MIC) directly (no data to process)
        // This is similar to CBC-MAC of an empty message
        let auth_tag = aes_att_encryption(key, &ivs);
        
        // Verify expected MIC matches the calculated tag by comparing the first n bytes
        // where n is the length of expected_mic
        return expected_mic[0..expected_mic.len()] == auth_tag[0..expected_mic.len()];
    }
    
    // For non-empty packets, decryption happens in multiple phases:
    
    // Phase 1: Decrypt packet data using AES-CTR mode
    // Note: This happens BEFORE authentication (decrypt-then-authenticate pattern)
    
    // Initialize counter block for AES-CTR mode
    // Format: [counter(1 byte) | IVM(8 bytes) | zeros(7 bytes)]
    let mut counter_block = [0u8; 16];
    counter_block[1..9].copy_from_slice(ivm);
    
    // Generate initial keystream block for the first 16 bytes
    let mut keystream_block = aes_att_encryption(key, &counter_block);
    
    // Process data in blocks of 16 bytes
    let mut index = 0;
    
    // Decrypt all bytes in the packet using AES-CTR mode
    while index < packet_data.len() {
        // When crossing a 16-byte boundary (except at the beginning),
        // increment counter and generate a new keystream block
        if index & 0xf == 0 && index > 0 {
            // Increment counter for next block
            counter_block[0] += 1;
            keystream_block = aes_att_encryption(key, &counter_block);
        }
        
        // Apply keystream to decrypt data using XOR
        // This is the core CTR mode operation: plaintext = ciphertext ⊕ keystream
        packet_data[index] = packet_data[index] ^ keystream_block[index & 0xf];
        index += 1;
    }
    
    // Phase 2: Calculate authentication tag for verification using CBC-MAC-like approach
    
    // Initialize authentication block with IVM and packet length
    // Format: [IVM(8 bytes) | packet_len(1 byte) | zeros(7 bytes)]
    let mut auth_block = [0u8; 16];
    auth_block[0..8].copy_from_slice(ivm);
    auth_block[8] = packet_data.len() as u8;
    // Note: remaining bytes are implicitly zero
    
    // Generate initial authentication value by encrypting the auth block
    // This creates the starting state for authentication
    let mut auth_value = aes_att_encryption(key, &auth_block);
    
    // Process packet data to compute authentication tag using CBC-MAC-like approach
    // Each byte is XORed with the current authentication value at the corresponding position
    // in the 16-byte block, and the block is encrypted when full or at the end
    index = 0;
    while index < packet_data.len() {
        // XOR current byte with auth value (circular indexing with & 0xf)
        auth_value[index & 0xf] ^= packet_data[index];
        
        // When we reach the end of a 16-byte block or the end of the data:
        // - index & 0xf == 0xf checks if we're at byte 15 of a block (0-based)
        // - index == packet_data.len() - 1 handles the final byte
        if (index & 0xf) == 0xf || index == packet_data.len() - 1 {
            // Process the block by encrypting the current state
            // This is similar to a CBC-MAC operation
            auth_value = aes_att_encryption(key, &auth_value);
        }
        
        index += 1;
    }
    
    // Verify the integrity by comparing the expected MIC with the calculated authentication tag (MIC)
    expected_mic[0..expected_mic.len()] == auth_value[0..expected_mic.len()]
}

/// Encrypts a packet using AES-128 algorithm with authenticated encryption (CCM-like mode).
///
/// This function implements a combined AES counter (CTR) mode for encryption with a MAC
/// (Message Authentication Code) for integrity protection. The process follows these phases:
///
/// 1. Phase 1: Generate authentication tag (MIC) on the plaintext data
/// 2. Phase 2: Copy the calculated MIC to the `mic_output` buffer (if provided)
/// 3. Phase 3: Encrypt the packet data using AES-CTR mode
///
/// A MIC (Message Integrity Check), also known as an authentication tag, is a short piece of
/// information used to authenticate a message—in other words, to confirm that the message
/// came from the stated sender (its authenticity) and has not been changed (its integrity).
///
/// The critical ordering of operations (authenticate-then-encrypt) ensures that the
/// MIC is derived from plaintext data and then encryption is applied.
/// This is the complementary function to `aes_att_decryption_packet`.
///
/// # Algorithm Details
/// - Authentication: Processes plaintext in 16-byte blocks, applying AES after each block or at the end
/// - AES-CTR mode: Uses counter blocks with IVM as base and incrementing counter
/// - The calculated MIC is stored in the `mic_output` buffer for packet transmission
///
/// # Parameters
/// * `key` - The AES-128 encryption key (16 bytes)
/// * `ivm` - The initialization vector modifier (8 bytes)
/// * `mic_output` - Output buffer to store the generated MIC (typically 2-4 bytes). The length
///                  of this slice determines how many bytes of the MIC are copied.
/// * `packet_data` - The plaintext packet data to be encrypted (modified in place)
#[cfg_attr(test, mry::mry)]
pub fn aes_att_encryption_packet(key: &[u8; 16], ivm: &[u8; 8], mic_output: &mut [u8], packet_data: &mut [u8])
{
    // Phase 0: Handle empty packet special case
    if packet_data.len() == 0 {
        // For empty packets, create an authentication block with specific format
        let mut auth_block = [0u8; 16];
        auth_block[0..8].copy_from_slice(ivm);
        auth_block[8] = 0;    // packet_data.len() as u8
        
        // Generate authentication tag (MIC) directly (no data to process)
        let auth_value = aes_att_encryption(key, &auth_block);
        
        // Copy MIC to output buffer (if provided)
        if mic_output.len() != 0 {
            let len = mic_output.len();
            mic_output[0..len].copy_from_slice(&auth_value[0..len]);
        }
        
        return;
    }
    
    // Phase 1: Generate authentication tag using CBC-MAC-like approach
    
    // Initialize authentication block with IVM and packet length
    // Format: [IVM(8 bytes) | packet_len(1 byte) | zeros(7 bytes)]
    let mut auth_block = [0u8; 16];
    auth_block[0..8].copy_from_slice(ivm);
    auth_block[8] = packet_data.len() as u8;
    // Note: remaining bytes are implicitly zero

    // Generate initial authentication value by encrypting the auth block
    // This creates the starting state for authentication
    let mut auth_value = aes_att_encryption(key, &auth_block);

    // Process packet data to compute authentication tag using CBC-MAC-like approach
    // Each byte is XORed with the current authentication value at the corresponding position
    // in the 16-byte block, and the block is encrypted when full or at the end
    let mut index = 0;
    while index < packet_data.len() {
        // XOR current byte with auth value (circular indexing with & 0xf)
        auth_value[index & 0xf] ^= packet_data[index];
        
        // When we reach the end of a 16-byte block or the end of the data:
        // - index & 0xf == 0xf checks if we're at byte 15 of a block (0-based)
        // - index == packet_data.len() - 1 handles the final byte
        if (index & 0xf) == 0xf || index == packet_data.len() - 1 {
            // Process the block by encrypting the current state
            // This is similar to a CBC-MAC operation
            auth_value = aes_att_encryption(key, &auth_value);
        }
        
        index += 1;
    }

    // Phase 2: Copy authentication tag (MIC) to output buffer (if provided)
    // Only copies the number of bytes that fit in the mic_output buffer
    if mic_output.len() != 0 {
        let len = mic_output.len();
        mic_output[0..len].copy_from_slice(&auth_value[0..len]);
    }

    // Phase 3: Encrypt packet data using AES-CTR mode
    
    // Initialize counter block for AES-CTR mode
    // Format: [counter(1 byte) | IVM(8 bytes) | zeros(7 bytes)]
    let mut counter_block = [0u8; 16];
    counter_block[1..9].copy_from_slice(ivm);

    // Generate initial keystream block for the first 16 bytes
    let mut keystream_block = aes_att_encryption(key, &counter_block);
    
    // Process data in blocks of 16 bytes
    let mut index = 0;
    
    // Encrypt all bytes in the packet using AES-CTR mode
    while index < packet_data.len() {
        // When crossing a 16-byte boundary (except at the beginning),
        // increment counter and generate a new keystream block
        if index & 0xf == 0 && index > 0 {
            // Increment counter for next block
            counter_block[0] += 1;
            keystream_block = aes_att_encryption(key, &counter_block);
        }
        
        // Apply keystream to encrypt data using XOR
        // This is the core CTR mode operation: ciphertext = plaintext ⊕ keystream
        packet_data[index] ^= keystream_block[index & 0xf];
        index += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mry::{self, Any};
    use crate::sdk::mcu::register::{mock_read_reg_aes_ctrl, mock_read_reg_aes_data, 
                                    mock_write_reg_aes_ctrl, mock_write_reg_aes_data, 
                                    mock_write_reg_aes_key};
    use crate::sdk::mcu::crypto::{mock_aes_ll_encryption, mock_aes_att_encryption, mock_aes_att_decryption};
    use crate::state::PAIR_CONFIG_PWD_ENCODE_SK;

    #[test]
    #[mry::lock(write_reg_aes_ctrl, write_reg_aes_key, write_reg_aes_data, read_reg_aes_ctrl, read_reg_aes_data)]
    fn test_aes_ll_encryption_basic() {
        // Arrange
        let key = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                   0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F];
        let source = [0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                      0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F];
        
        // Direction Encrypt
        let direction = AesDirection::Encrypt;
        
        // Mock the register writes
        mock_write_reg_aes_ctrl(Any).returns(());
        
        // Mock all key register writes
        for i in 0..16 {
            mock_write_reg_aes_key(Any, i).returns(());
        }
        
        // Mock the data register writes
        mock_write_reg_aes_data(Any).returns(());
        
        // Mock the control register read for completion check
        // Using returns_with with a static counter to return different values on each call
        {
            static mut CTRL_COUNTER: usize = 0;
            mock_read_reg_aes_ctrl().returns_with(move || {
                unsafe {
                    let val = if CTRL_COUNTER == 0 { 0 } else { 4 };
                    CTRL_COUNTER += 1;
                    val
                }
            });
        }
        
        // Mock the data register reads with test values
        // Using returns_with with a static counter to return different values on each call
        {
            static mut DATA_COUNTER: usize = 0;
            let return_values = [0x20212223, 0x24252627, 0x28292A2B, 0x2C2D2E2F];
            mock_read_reg_aes_data().returns_with(move || {
                unsafe {
                    let val = return_values[DATA_COUNTER];
                    DATA_COUNTER += 1;
                    val
                }
            });
        }
        
        // Act
        let result = aes_ll_encryption(&key, &source, direction);
        
        // Assert
        // Verify the direction was set correctly
        mock_write_reg_aes_ctrl(direction as u8).assert_called(1);
        
        // Verify key was written in reverse order
        for i in 0..16 {
            mock_write_reg_aes_key(key[15 - i], i as u32).assert_called(1);
        }
        
        // Verify data was written correctly
        mock_write_reg_aes_data(0x10111213).assert_called(1);
        mock_write_reg_aes_data(0x14151617).assert_called(1);
        mock_write_reg_aes_data(0x18191A1B).assert_called(1);
        mock_write_reg_aes_data(0x1C1D1E1F).assert_called(1);
        
        // Verify the control register was read for completion check
        mock_read_reg_aes_ctrl().assert_called(2);
        
        // Verify data register was read for results
        mock_read_reg_aes_data().assert_called(4);
        
        // Verify the result was correctly stored
        assert_eq!(result, [0x23, 0x22, 0x21, 0x20, 0x27, 0x26, 0x25, 0x24,
                          0x2B, 0x2A, 0x29, 0x28, 0x2F, 0x2E, 0x2D, 0x2C]);
    }

    #[test]
    #[mry::lock(write_reg_aes_ctrl, write_reg_aes_key, write_reg_aes_data, read_reg_aes_ctrl, read_reg_aes_data)]
    fn test_aes_ll_encryption_decrypt_mode() {
        // Arrange
        let key = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                   0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F];
        let source = [0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
                      0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F];
        
        // Direction Decrypt
        let direction = AesDirection::Decrypt;
        
        // Mock the register writes
        mock_write_reg_aes_ctrl(Any).returns(());
        
        // Mock all key register writes
        for i in 0..16 {
            mock_write_reg_aes_key(Any, i).returns(());
        }
        
        // Mock the data register writes
        mock_write_reg_aes_data(Any).returns(());
        
        // Mock the control register read for completion check
        // Using returns_with with a static counter to return different values on each call
        {
            static mut CTRL_COUNTER: usize = 0;
            mock_read_reg_aes_ctrl().returns_with(move || {
                unsafe {
                    let val = if CTRL_COUNTER == 0 { 0 } else { 4 };
                    CTRL_COUNTER += 1;
                    val
                }
            });
        }
        
        // Mock the data register reads with test values
        // Using returns_with with a static counter to return different values on each call
        {
            static mut DATA_COUNTER: usize = 0;
            let return_values = [0x10111213, 0x14151617, 0x18191A1B, 0x1C1D1E1F];
            mock_read_reg_aes_data().returns_with(move || {
                unsafe {
                    let val = return_values[DATA_COUNTER];
                    DATA_COUNTER += 1;
                    val
                }
            });
        }
        
        // Act
        let result = aes_ll_encryption(&key, &source, direction); // Capture the returned result
        
        // Assert
        // Verify the direction was set correctly
        mock_write_reg_aes_ctrl(direction as u8).assert_called(1);
        
        // Verify key was written in reverse order
        for i in 0..16 {
            mock_write_reg_aes_key(key[15 - i], i as u32).assert_called(1);
        }
        
        // Verify data was written correctly in reverse 4-byte groups
        mock_write_reg_aes_data(0x20212223).assert_called(1);
        mock_write_reg_aes_data(0x24252627).assert_called(1);
        mock_write_reg_aes_data(0x28292A2B).assert_called(1);
        mock_write_reg_aes_data(0x2C2D2E2F).assert_called(1);
        
        // Verify the result was correctly stored - with correct byte ordering
        assert_eq!(result, [0x13, 0x12, 0x11, 0x10, 0x17, 0x16, 0x15, 0x14, // Check the returned result
                          0x1B, 0x1A, 0x19, 0x18, 0x1F, 0x1E, 0x1D, 0x1C]);
    }

    #[test]
    #[mry::lock(write_reg_aes_ctrl, write_reg_aes_key, write_reg_aes_data, read_reg_aes_ctrl, read_reg_aes_data)]
    fn test_aes_ll_encryption_multiple_polls() {
        // Arrange
        let key = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                   0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F];
        let source = [0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                      0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F];
        
        // Direction 0 = encrypt
        let direction = AesDirection::Encrypt;
        
        // Mock the register writes
        mock_write_reg_aes_ctrl(Any).returns(());
        
        // Mock all key register writes
        for i in 0..16 {
            mock_write_reg_aes_key(Any, i).returns(());
        }
        
        // Mock the data register writes
        mock_write_reg_aes_data(Any).returns(());
        
        // Mock the control register read for completion check - simulate multiple polls before completion
        // Using returns_with with a static counter to return different values on each call
        {
            static mut CTRL_COUNTER: usize = 0;
            mock_read_reg_aes_ctrl().returns_with(move || {
                unsafe {
                    let val = if CTRL_COUNTER < 4 { 0 } else { 4 };
                    CTRL_COUNTER += 1;
                    val
                }
            });
        }
        
        // Mock the data register reads with test values
        // Using returns_with with a static counter to return different values on each call
        {
            static mut DATA_COUNTER: usize = 0;
            let return_values = [0x20212223, 0x24252627, 0x28292A2B, 0x2C2D2E2F];
            mock_read_reg_aes_data().returns_with(move || {
                unsafe {
                    let val = return_values[DATA_COUNTER];
                    DATA_COUNTER += 1;
                    val
                }
            });
        }
        
        // Act
        let result = aes_ll_encryption(&key, &source, direction); // Capture the returned result
        
        // Assert
        // Verify the direction was set correctly
        mock_write_reg_aes_ctrl(direction as u8).assert_called(1);
        
        // Verify key was written in reverse order
        for i in 0..16 {
            mock_write_reg_aes_key(key[15 - i], i as u32).assert_called(1);
        }
        
        // Verify data was written correctly
        mock_write_reg_aes_data(0x10111213).assert_called(1);
        mock_write_reg_aes_data(0x14151617).assert_called(1);
        mock_write_reg_aes_data(0x18191A1B).assert_called(1);
        mock_write_reg_aes_data(0x1C1D1E1F).assert_called(1);
        
        // Verify that the control register was polled multiple times
        mock_read_reg_aes_ctrl().assert_called(5);
        
        // Verify the result was correctly stored - with correct byte ordering
        assert_eq!(result, [0x23, 0x22, 0x21, 0x20, 0x27, 0x26, 0x25, 0x24,
                          0x2B, 0x2A, 0x29, 0x28, 0x2F, 0x2E, 0x2D, 0x2C]);
    }

    #[test]
    #[mry::lock(write_reg_aes_ctrl, write_reg_aes_key, write_reg_aes_data, read_reg_aes_ctrl, read_reg_aes_data)]
    fn test_aes_ll_encryption_byte_order() {
        // This test verifies that the byte ordering in the AES implementation is correct
        // The TLSR8266 uses a specific byte ordering for AES operations
        
        // Arrange
        let key = [0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
                   0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10];
                   
        let source = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11,
                      0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99];
        
        // Direction 0 = encrypt
        let direction = AesDirection::Encrypt;
        
        // Mock the register writes
        mock_write_reg_aes_ctrl(Any).returns(());
        
        // Mock all key register writes
        for i in 0..16 {
            mock_write_reg_aes_key(Any, i).returns(());
        }
        
        // Mock the data register writes
        mock_write_reg_aes_data(Any).returns(());
        
        // Mock the control register read for completion check
        // Using returns_with with a static counter to return different values on each call
        {
            static mut CTRL_COUNTER: usize = 0;
            mock_read_reg_aes_ctrl().returns_with(move || {
                unsafe {
                    let val = if CTRL_COUNTER == 0 { 0 } else { 4 };
                    CTRL_COUNTER += 1;
                    val
                }
            });
        }
        
        // Mock the data register reads with specific test values to verify byte order
        // Using returns_with with a static counter to return different values on each call
        {
            static mut DATA_COUNTER: usize = 0;
            let return_values = [0xAABBCCDD, 0xEEFF0011, 0x22334455, 0x66778899];
            mock_read_reg_aes_data().returns_with(move || {
                unsafe {
                    let val = return_values[DATA_COUNTER];
                    DATA_COUNTER += 1;
                    val
                }
            });
        }
        
        // Act
        let result = aes_ll_encryption(&key, &source, direction); // Capture the returned result
        
        // Assert
        // Verify that the key bytes were written in reverse order
        mock_write_reg_aes_key(0x10, 0).assert_called(1);
        mock_write_reg_aes_key(0x32, 1).assert_called(1);
        mock_write_reg_aes_key(0x54, 2).assert_called(1);
        mock_write_reg_aes_key(0x76, 3).assert_called(1);
        mock_write_reg_aes_key(0x98, 4).assert_called(1);
        mock_write_reg_aes_key(0xBA, 5).assert_called(1);
        mock_write_reg_aes_key(0xDC, 6).assert_called(1);
        mock_write_reg_aes_key(0xFE, 7).assert_called(1);
        mock_write_reg_aes_key(0xEF, 8).assert_called(1);
        mock_write_reg_aes_key(0xCD, 9).assert_called(1);
        mock_write_reg_aes_key(0xAB, 10).assert_called(1);
        mock_write_reg_aes_key(0x89, 11).assert_called(1);
        mock_write_reg_aes_key(0x67, 12).assert_called(1);
        mock_write_reg_aes_key(0x45, 13).assert_called(1);
        mock_write_reg_aes_key(0x23, 14).assert_called(1);
        mock_write_reg_aes_key(0x01, 15).assert_called(1);
        
        // Verify data was packed correctly with the most significant byte first
        mock_write_reg_aes_data(0xAABBCCDD).assert_called(1);
        mock_write_reg_aes_data(0xEEFF0011).assert_called(1);
        mock_write_reg_aes_data(0x22334455).assert_called(1);
        mock_write_reg_aes_data(0x66778899).assert_called(1);
        
        // Verify the output data is correctly extracted from the 32-bit registers
        // In the implementation, bytes are stored in little-endian format from each register:
        // dest[idx] = tmp as u8;            (lowest byte)
        // dest[idx + 1] = (tmp >> 8) as u8; (second byte)
        // dest[idx + 2] = (tmp >> 16) as u8; (third byte)
        // dest[idx + 3] = (tmp >> 24) as u8; (highest byte)
        
        // First register: 0xAABBCCDD -> little-endian: [0xDD, 0xCC, 0xBB, 0xAA]
        assert_eq!(result[0], 0xDD);   // Lowest byte of first word
        assert_eq!(result[1], 0xCC);   // Second byte of first word
        assert_eq!(result[2], 0xBB);   // Third byte of first word
        assert_eq!(result[3], 0xAA);   // Highest byte of first word
        
        // Second register: 0xEEFF0011 -> little-endian: [0x11, 0x00, 0xFF, 0xEE]
        assert_eq!(result[4], 0x11);   // Lowest byte of second word
        assert_eq!(result[5], 0x00);   // Second byte of second word
        assert_eq!(result[6], 0xFF);   // Third byte of second word
        assert_eq!(result[7], 0xEE);   // Highest byte of second word
        
        // Third register: 0x22334455 -> little-endian: [0x55, 0x44, 0x33, 0x22]
        assert_eq!(result[8], 0x55);
        assert_eq!(result[9], 0x44);
        assert_eq!(result[10], 0x33);
        assert_eq!(result[11], 0x22);
        
        // Fourth register: 0x66778899 -> little-endian: [0x99, 0x88, 0x77, 0x66]
        assert_eq!(result[12], 0x99);
        assert_eq!(result[13], 0x88);
        assert_eq!(result[14], 0x77);
        assert_eq!(result[15], 0x66);
    }

    // Tests for aes_ll_swap have been removed as we're now using the standard .reverse() method instead

    #[test]
    #[mry::lock(aes_ll_encryption)]
    fn test_aes_att_encryption() {
        // Arrange
        let key = [1u8; 16];
        let source = [2u8; 16];
        // Create an array and its reversed version for testing
        let mut encrypted_ll = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
        let mut expected_result = encrypted_ll;
        expected_result.reverse(); // Pre-compute the reversed array

        // Mock aes_ll_encryption to return a predefined value
        mock_aes_ll_encryption(&key, &source, AesDirection::Encrypt).returns(encrypted_ll);

        // Act
        let result = aes_att_encryption(&key, &source);

        // Assert
        // Verify aes_ll_encryption was called correctly
        mock_aes_ll_encryption(&key, &source, AesDirection::Encrypt).assert_called(1);
        // Verify the result is the reversed version of what aes_ll_encryption returned
        assert_eq!(result, expected_result);
    }

    #[test]
    #[mry::lock(aes_ll_encryption)]
    fn test_aes_att_decryption() {
        // Arrange
        let key = [5u8; 16];
        let source = [6u8; 16]; // This is the ciphertext
        // Create an array and its reversed version for testing
        let mut decrypted_ll = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
        let mut expected_result = decrypted_ll;
        expected_result.reverse(); // Pre-compute the reversed array

        // Mock aes_ll_encryption for decryption
        mock_aes_ll_encryption(&key, &source, AesDirection::Decrypt).returns(decrypted_ll);

        // Act
        let result = aes_att_decryption(&key, &source);

        // Assert
        mock_aes_ll_encryption(&key, &source, AesDirection::Decrypt).assert_called(1);
        // Verify the result is the reversed version of what aes_ll_encryption returned
        assert_eq!(result, expected_result);
    }

    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_encode_password() {
        // Arrange
        let password = [0xAAu8; 16];
        let encoded_password = [0xBBu8; 16];
        // Get the actual key from the mutex for verification
        let expected_key = *PAIR_CONFIG_PWD_ENCODE_SK.lock();

        // Mock aes_att_encryption to return the expected encoded password
        // when called with the correct key and password
        mock_aes_att_encryption(&expected_key, &password).returns(encoded_password);

        // Act
        let result = encode_password(&password);

        // Assert
        // Verify aes_att_encryption was called with the global key and the password
        mock_aes_att_encryption(&expected_key, &password).assert_called(1);
        // Verify the result is what the mocked function returned
        assert_eq!(result, encoded_password);
    }

    #[test]
    #[mry::lock(aes_att_decryption)]
    fn test_decode_password() {
        // Arrange
        let encoded_password = [0xCCu8; 16];
        let decoded_password = [0xDDu8; 16];
        // Get the actual key from the mutex for verification
        let expected_key = *PAIR_CONFIG_PWD_ENCODE_SK.lock();

        // Mock aes_att_decryption to return the expected decoded password
        mock_aes_att_decryption(&expected_key, &encoded_password).returns(decoded_password);

        // Act
        let result = decode_password(&encoded_password);

        // Assert
        // Verify aes_att_decryption was called with the global key and the encoded password
        mock_aes_att_decryption(&expected_key, &encoded_password).assert_called(1);
        // Verify the result is what the mocked function returned
        assert_eq!(result, decoded_password);
    }

    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_aes_att_decryption_packet_empty() {
        // Arrange
        let key = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let expected_mic = [0x31, 0x32, 0x33]; // MIC to validate against
        let mut packet_data: [u8; 0] = []; // Empty packet data
        
        // Mock aes_att_encryption to return specific value for empty packet case
        let expected_encrypted = [0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
                                 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50];
        
        // For empty packets, the function sets up ivs and performs one encryption call
        mock_aes_att_encryption(Any, Any).returns(expected_encrypted);

        // Act
        let result = aes_att_decryption_packet(&key, &ivm, &expected_mic, &mut packet_data);

        // Assert
        // Verify aes_att_encryption was called once with the proper parameters
        
        // Check that the encryption was called with properly initialized ivs array
        let mut expected_ivs = [0u8; 16];
        expected_ivs[0..8].copy_from_slice(&ivm[0..8]);
        expected_ivs[8] = 0; // packet_data.len() as u8
        expected_ivs[12] = 0; // packet_data.len() as u8
        
        mock_aes_att_encryption(&key, &expected_ivs).assert_called(1);
        
        // Verify correct result (should match first 3 bytes of expected_encrypted)
        assert_eq!(result, expected_encrypted[0..3] == expected_mic[0..3]);
    }

    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_aes_att_decryption_packet_small() {
        // Arrange
        let key = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let expected_mic = [0x31, 0x32, 0x33, 0x34]; // MIC to validate against
        let mut packet_data = [0xA1, 0xA2, 0xA3, 0xA4, 0xA5]; // Small packet data (less than 16 bytes)
        
        // Set up mock values for two encryption calls
        let first_encryption = [0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
                               0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50];
        let second_encryption = [0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
                                0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60];
        
        // Set up mock to return different values on each call
        static mut CALL_COUNT: usize = 0;
        mock_aes_att_encryption(Any, Any).returns_with(move |_, _| {
            unsafe {
                let result = if CALL_COUNT == 0 {
                    first_encryption
                } else {
                    second_encryption
                };
                CALL_COUNT += 1;
                result
            }
        });

        // Expected packet_data after decryption (XOR with first_encryption)
        let expected_packet_data = [
            packet_data[0] ^ first_encryption[0], // 0xA1 ^ 0x41
            packet_data[1] ^ first_encryption[1], // 0xA2 ^ 0x42
            packet_data[2] ^ first_encryption[2], // 0xA3 ^ 0x43
            packet_data[3] ^ first_encryption[3], // 0xA4 ^ 0x44
            packet_data[4] ^ first_encryption[4], // 0xA5 ^ 0x45
        ];

        // Save original packet_data for verification
        let original_packet_data = packet_data;
        
        // Act
        let result = aes_att_decryption_packet(&key, &ivm, &expected_mic, &mut packet_data);
        
        // Assert
        // Check if packet data was properly transformed
        assert_eq!(packet_data[0], expected_packet_data[0]);
        assert_eq!(packet_data[1], expected_packet_data[1]);
        assert_eq!(packet_data[2], expected_packet_data[2]);
        assert_eq!(packet_data[3], expected_packet_data[3]);
        assert_eq!(packet_data[4], expected_packet_data[4]);
        
        // The second encryption call is used to compute ivs for validation
        // Verify the final result based on expected_mic and ivs
        assert_eq!(result, second_encryption[0..expected_mic.len()] == expected_mic);
    }
    
    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_aes_att_decryption_packet_large() {
        // Arrange
        let key = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let expected_mic: [u8; 0] = []; // No validation
        let mut packet_data = [0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 
                              0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB0, 
                              0xB1, 0xB2, 0xB3, 0xB4]; // 20 bytes, crosses 16-byte boundary
        
        // Set up encryption results for multiple calls
        let encrypt_results = [
            // First call for initial block decryption
            [0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
             0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50],
            // Second call after incrementing ivs[0]
            [0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
             0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60],
            // Third call for final ivs encryption
            [0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
             0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70]
        ];
        
        // Mock aes_att_encryption to return different values on each call
        static mut CALL_COUNT: usize = 0;
        mock_aes_att_encryption(Any, Any).returns_with(move |_, _| {
            unsafe {
                let result = encrypt_results[CALL_COUNT % encrypt_results.len()];
                CALL_COUNT += 1;
                result
            }
        });
        
        // Act
        let result = aes_att_decryption_packet(&key, &ivm, &expected_mic, &mut packet_data);
        
        // Assert
        // For empty expected_mic, should always return true
        assert_eq!(result, true);
        
        // Verify packet data was properly transformed
        // First 16 bytes are processed using first encryption result
        assert_eq!(packet_data[0], 0xA1 ^ encrypt_results[0][0]);
        // The remaining bytes should be processed using second encryption result
        assert_eq!(packet_data[16], 0xB1 ^ encrypt_results[1][0]);
        
        // Check a few more spots to ensure proper XOR was applied
        assert_eq!(packet_data[1], 0xA2 ^ encrypt_results[0][1]);
        assert_eq!(packet_data[15], 0xB0 ^ encrypt_results[0][15]);
        assert_eq!(packet_data[17], 0xB2 ^ encrypt_results[1][1]);
        assert_eq!(packet_data[19], 0xB4 ^ encrypt_results[1][3]);
    }

    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_aes_att_decryption_packet_validation_success() {
        // Arrange
        let key = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        
        // The validation will succeed because we'll make encryption return this exact value
        let expected_mic = [0x31, 0x32, 0x33, 0x34];
        
        let mut packet_data = [0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8];
        
        // Final encryption result that will be compared with expected_mic
        let final_encryption = [0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
                               0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40];
        
        // Set up mock to return values that will make validation succeed
        static mut CALL_COUNT: usize = 0;
        mock_aes_att_encryption(Any, Any).returns_with(move |_, _| {
            unsafe {
                CALL_COUNT += 1;
                if CALL_COUNT == 3 { // On the third call (final validation check)
                    final_encryption
                } else {
                    [0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
                     0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50]
                }
            }
        });
        
        // Act
        let result = aes_att_decryption_packet(&key, &ivm, &expected_mic, &mut packet_data);
        
        // Assert
        // Should return true because expected_mic matches the first 4 bytes of final_encryption
        assert_eq!(result, true);
    }
    
    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_aes_att_decryption_packet_validation_failure() {
        // Arrange
        let key = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        
        // These values won't match what encryption returns
        let expected_mic = [0xFF, 0xFF, 0xFF, 0xFF];
        
        let mut packet_data = [0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8];
        
        // Final encryption result that will be compared with expected_mic
        let final_encryption = [0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
                               0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40];
        
        // Set up mock to return values that will make validation fail
        static mut CALL_COUNT: usize = 0;
        mock_aes_att_encryption(Any, Any).returns_with(move |_, _| {
            unsafe {
                CALL_COUNT += 1;
                if CALL_COUNT == 2 { // On the second call (final validation check)
                    final_encryption
                } else {
                    [0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
                     0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50]
                }
            }
        });
        
        // Act
        let result = aes_att_decryption_packet(&key, &ivm, &expected_mic, &mut packet_data);
        
        // Assert
        // Should return false because expected_mic does not match the first 4 bytes of final_encryption
        assert_eq!(result, false);
    }
    
    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_aes_att_decryption_packet_alignment_boundaries() {
        // This test verifies behavior around the 16-byte (0xF) alignment boundaries
        
        // Arrange
        let key = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let expected_mic: [u8; 0] = []; // No validation needed
        
        // Create packet data with exactly 16 bytes (one full block)
        let mut packet_data = [0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 
                              0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB0];
        
        // Create encryption results for testing
        let encrypt_results = [
            [0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
             0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50],
            [0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
             0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60]
        ];
        
        // Mock encryption function
        static mut CALL_COUNT: usize = 0;
        mock_aes_att_encryption(Any, Any).returns_with(move |_, _| {
            unsafe {
                let result = encrypt_results[CALL_COUNT % encrypt_results.len()];
                CALL_COUNT += 1;
                result
            }
        });
        
        // Make copies of original data for verification
        let original_packet_data = packet_data;
        
        // Act
        let result = aes_att_decryption_packet(&key, &ivm, &expected_mic, &mut packet_data);
        
        // Assert
        assert_eq!(result, true);
        
        // Verify all bytes were properly transformed
        for i in 0..16 {
            assert_eq!(packet_data[i], original_packet_data[i] ^ encrypt_results[0][i]);
        }
    }
    
    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_aes_att_decryption_packet_complex_encryption_flow() {
        // Arrange
        let key = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let expected_mic = [0x31, 0x32, 0x33]; // Validation data
        
        // Packet data exactly 17 bytes (crosses 16-byte boundary)
        let mut packet_data = [
            0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 
            0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB0,
            0xB1
        ];
        
        // Encryption results for each call in the function
        let encryption_results = [
            // 1st call: initial decryption
            [0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
             0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50],
            // 2nd call: after ivs[0]++
            [0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
             0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60],
            // 3rd call: for block with index 0x0F
            [0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
             0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70],
            // 4th call: first ivs update in validation loop
            [0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
             0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F, 0x80],
            // 5th call: final validation value (should match expected_mic)
            [0x31, 0x32, 0x33, 0x64, 0x65, 0x66, 0x67, 0x68,
             0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70],
        ];
        static mut CALL_COUNT: usize = 0;
        mock_aes_att_encryption(Any, Any).returns_with(move |_, _| {
            unsafe {
                let result = encryption_results[CALL_COUNT];
                CALL_COUNT += 1;
                result
            }
        });
        
        // Act
        let result = aes_att_decryption_packet(&key, &ivm, &expected_mic, &mut packet_data);
        
        // Assert
        // The validation should succeed as we've set up the fifth encryption result
        // to match the expected_mic value in its first three bytes
        assert_eq!(result, true);
        
        // Verify that proper number of encryption calls were made
        // The exact number depends on the complexity of the algorithm
        mock_aes_att_encryption(Any, Any).assert_called(5);
    }

    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_aes_att_encryption_packet_empty() {
        // Arrange
        let key = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let mut src = [0xA1, 0xA2, 0xA3, 0xA4]; // Source data to be filled
        let mut packet_data: [u8; 0] = []; // Empty packet data
        
        // Mock aes_att_encryption to return specific value for the initial encoding
        let initial_encoded = [0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
                             0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50];
        
        mock_aes_att_encryption(Any, Any).returns(initial_encoded);

        // Save original src for comparison
        let original_src = src;
        
        // Act
        aes_att_encryption_packet(&key, &ivm, &mut src, &mut packet_data);

        // Assert
        // With empty packet data, aes_att_encryption should be called exactly once
        // for the initial IV generation
        mock_aes_att_encryption(Any, Any).assert_called(1);
        
        // Verify that src was filled with bytes from the encryption
        assert_eq!(src, [0x41, 0x42, 0x43, 0x44]);
        
        // Verify the call was made with correct initialization vector
        let mut expected_initial_iv = [0u8; 16];
        expected_initial_iv[0..8].copy_from_slice(&ivm);
        expected_initial_iv[8] = 0; // packet_data.len() as u8
        mock_aes_att_encryption(&key, &expected_initial_iv).assert_called(1);
    }

    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_aes_att_encryption_packet_small() {
        // Arrange
        let key = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let mut src = [0xA1, 0xA2, 0xA3, 0xA4]; // Source data to be filled
        let mut packet_data = [0xB1, 0xB2, 0xB3, 0xB4, 0xB5]; // Small packet data

        // Mock encryption results
        let initial_encoded = [0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
                             0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50];
        let encryption_after_update = [0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
                                     0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60];
        let final_result = [0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
                          0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70];
        
        static mut CALL_COUNT: usize = 0;
        mock_aes_att_encryption(Any, Any).returns_with(move |_, _| {
            unsafe {
                let result = match CALL_COUNT {
                    0 => initial_encoded,
                    1 => encryption_after_update,
                    _ => final_result
                };
                CALL_COUNT += 1;
                result
            }
        });

        // Save original data for comparison
        let original_src = src;
        let original_packet_data = packet_data;
        
        // Act
        aes_att_encryption_packet(&key, &ivm, &mut src, &mut packet_data);

        // Assert
        // Verify src was updated with bytes from encryption_after_update
        assert_eq!(src, [0x51, 0x52, 0x53, 0x54]);
        
        // Verify packet_data was properly transformed with the final_result
        for i in 0..packet_data.len() {
            assert_eq!(packet_data[i], original_packet_data[i] ^ final_result[i & 0xf]);
        }
        
        // Verify correct number of encryption calls were made
        mock_aes_att_encryption(Any, Any).assert_called(3);
    }

    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_aes_att_encryption_packet_large() {
        // Arrange
        let key = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let mut src = [0xA1, 0xA2]; // Source data to be filled (2 bytes)
        // Large packet that crosses 16-byte boundary
        let mut packet_data = [
            0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8,
            0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF, 0xC0, // 16 bytes
            0xC1, 0xC2, 0xC3, 0xC4 // 4 more bytes
        ];

        // Mock encryption results for different stages
        let initial_encoded = [0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
                             0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50];
        let encoded_after_first_block = [0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
                                       0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60];
        let src_result = [0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
                        0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70];
        let first_result_block = [0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
                                0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F, 0x80];
        let second_result_block = [0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88,
                                 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x90];
        
        static mut CALL_COUNT: usize = 0;
        mock_aes_att_encryption(Any, Any).returns_with(move |_, _| {
            unsafe {
                let result = match CALL_COUNT {
                    0 => initial_encoded,
                    1 => encoded_after_first_block,
                    2 => src_result,
                    3 => first_result_block,
                    _ => second_result_block
                };
                CALL_COUNT += 1;
                result
            }
        });

        // Save original values for comparison
        let original_packet_data = packet_data;
        
        // Act
        aes_att_encryption_packet(&key, &ivm, &mut src, &mut packet_data);

        // Assert
        // Verify src was updated correctly
        assert_eq!(src, [0x61, 0x62]);
        
        // Verify first 16 bytes of packet_data were transformed with first_result_block
        for i in 0..16 {
            assert_eq!(packet_data[i], original_packet_data[i] ^ first_result_block[i]);
        }
        
        // Verify remaining bytes were transformed with second_result_block
        for i in 16..packet_data.len() {
            assert_eq!(packet_data[i], original_packet_data[i] ^ second_result_block[i & 0xf]);
        }
        
        // Verify correct number of encryption calls
        mock_aes_att_encryption(Any, Any).assert_called(5);
    }

    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_aes_att_encryption_packet_alignment_boundary() {
        // Arrange
        let key = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let mut src = [0xA1, 0xA2, 0xA3, 0xA4]; // Source data to be filled
        // Packet data exactly 16 bytes (one full block)
        let mut packet_data = [
            0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8,
            0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF, 0xC0
        ];

        // Mock encryption results
        let initial_encoded = [0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
                             0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50];
        let encoded_after_xor = [0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
                               0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60];
        let final_encoded = [0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
                           0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70];
        
        static mut CALL_COUNT: usize = 0;
        mock_aes_att_encryption(Any, Any).returns_with(move |_, _| {
            unsafe {
                let result = match CALL_COUNT {
                    0 => initial_encoded,
                    1 => encoded_after_xor,
                    _ => final_encoded
                };
                CALL_COUNT += 1;
                result
            }
        });

        // Save original values for comparison
        let original_packet_data = packet_data;
        
        // Act
        aes_att_encryption_packet(&key, &ivm, &mut src, &mut packet_data);

        // Assert
        // Verify src was updated
        assert_eq!(src, [0x51, 0x52, 0x53, 0x54]);
        
        // Verify packet_data was transformed
        for i in 0..packet_data.len() {
            assert_eq!(packet_data[i], original_packet_data[i] ^ final_encoded[i]);
        }
        
        // Verify correct number of encryption calls
        mock_aes_att_encryption(Any, Any).assert_called(3);
    }

    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_aes_att_encryption_packet_empty_src() {
        // Arrange
        let key = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let mut src: [u8; 0] = []; // Empty source buffer
        let mut packet_data = [0xB1, 0xB2, 0xB3, 0xB4, 0xB5]; // Small packet data

        // Mock encryption results
        let initial_encoded = [0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
                             0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50];
        let encryption_after_update = [0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
                                     0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60];
        let final_result = [0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
                          0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70];
        
        static mut CALL_COUNT: usize = 0;
        mock_aes_att_encryption(Any, Any).returns_with(move |_, _| {
            unsafe {
                let result = match CALL_COUNT {
                    0 => initial_encoded,
                    1 => encryption_after_update,
                    _ => final_result
                };
                CALL_COUNT += 1;
                result
            }
        });

        // Save original data for comparison
        let original_packet_data = packet_data;
        
        // Act
        aes_att_encryption_packet(&key, &ivm, &mut src, &mut packet_data);

        // Assert
        // src should remain empty
        assert_eq!(src.len(), 0);
        
        // Verify packet_data was properly transformed
        for i in 0..packet_data.len() {
            assert_eq!(packet_data[i], original_packet_data[i] ^ final_result[i & 0xf]);
        }
        
        // Verify correct number of encryption calls
        mock_aes_att_encryption(Any, Any).assert_called(3);
    }
}