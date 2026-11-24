use tlsr8266_pac::{
    read_reg_aes_ctrl, read_reg_aes_data, write_reg_aes_ctrl, write_reg_aes_data, write_reg_aes_key,
};

/// Enum representing the direction of AES operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AesDirection {
    Encrypt = 0,
    Decrypt = 1,
}

/// The decrypted packet's MIC did not match the expected MIC.
///
/// A unit type rather than `()` so the failure is nameable at call sites and in
/// type signatures; there is no additional information to report.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MicMismatch;

/// Maximum number of done-bit polls the AES completion wait performs.
pub const AES_WAIT_MAX_ITERS: u32 = 10_000;

/// Runs the SoC's AES write/poll/read sequence against the `0x540` (control),
/// `0x548` (data) and `0x550` (key) MMIO registers.
///
/// The sequence is unchanged from the pre-extraction code; it is only factored
/// out so the device path and the host `MmioRegisters` fake drive the same
/// registers in the same order.
///
/// # Returns
///
/// `true` once the hardware sets its done bit, `false` if it never did within
/// [`AES_WAIT_MAX_ITERS`] polls. `out` is written only on success, so an
/// infallible caller can pre-load it with the zero-block fallback.
///
/// The block is written through `out` rather than returned in a `Result` so the
/// ISR callers pay no stack for the timeout plumbing (the 17-byte `Result`
/// return needs a scratch slot; see the RAM gate in `make check`).
#[inline(always)]
fn aes_mmio_run_into(
    key: &[u8],
    source: &[u8],
    direction: AesDirection,
    out: &mut [u8; 16],
) -> bool {
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
                    (source[idx + 3] as u32), // LSB (least significant byte)
        );
    }

    // Wait for AES operation to complete
    // Bit 2 (value 4) of the AES_CTRL register is set when operation is done.
    // The budget bounds the spin: a stuck block reports a timeout instead of
    // hanging the firmware.
    let mut done = false;
    for _ in 0..AES_WAIT_MAX_ITERS {
        if read_reg_aes_ctrl() & 4 != 0 {
            done = true;
            break;
        }
    }
    if !done {
        return false;
    }

    // Read the result from AES data registers
    // Each register returns a 32-bit word, and we need to read 4 words (16 bytes)
    // Convert from big-endian 32-bit words to byte array
    for idx in (0..16).step_by(4) {
        let tmp = read_reg_aes_data();

        // Store result in destination buffer
        // Extract individual bytes from the 32-bit register value
        // in little-endian order (LSB first)
        out[idx] = tmp as u8; // LSB (least significant byte)
        out[idx + 1] = (tmp >> 8) as u8; // 2nd byte
        out[idx + 2] = (tmp >> 16) as u8; // 3rd byte
        out[idx + 3] = (tmp >> 24) as u8; // MSB (most significant byte)
    }

    true
}

/// [`aes_mmio_run_into`] with the timeout reported as `Result`.
///
/// # Returns
///
/// * `Ok` with the 16-byte block once the hardware sets its done bit
/// * `Err(())` if the done bit never appeared within [`AES_WAIT_MAX_ITERS`]
///   polls, i.e. the crypto block is stuck
fn aes_mmio_run(key: &[u8], source: &[u8], direction: AesDirection) -> Result<[u8; 16], ()> {
    let mut result = [0u8; 16];
    if aes_mmio_run_into(key, source, direction, &mut result) {
        Ok(result)
    } else {
        Err(())
    }
}

/// Host/device dispatch for the AES block, writing the result through `out`.
///
/// Returns `false` on a hardware timeout; `out` is then untouched, so the
/// caller's pre-loaded zero block is the documented fallback.
fn aes_run_into(key: &[u8], source: &[u8], direction: AesDirection, out: &mut [u8; 16]) -> bool {
    #[cfg(not(target_os = "none"))]
    {
        fake_crypto::encryption_into(key, source, direction, out)
    }
    #[cfg(target_os = "none")]
    {
        // Execute AES operation in a critical section to prevent interrupts
        // during hardware crypto operations
        critical_section::with(|_| aes_mmio_run_into(key, source, direction, out))
    }
}

/// Low-level AES encryption/decryption, reporting a hardware timeout.
///
/// This is [`aes_ll_encryption`] with the stuck-hardware case surfaced to the
/// caller instead of being collapsed into a zero block. Use it where a silent
/// all-zero block would be mistaken for a valid result (e.g. MIC checks, which
/// must fail closed).
///
/// # Parameters
/// * `key` - The AES key (at least 16 bytes)
/// * `source` - The data to encrypt/decrypt (at least 16 bytes)
/// * `direction` - Operation mode: `AesDirection::Encrypt` or `AesDirection::Decrypt`
///
/// # Returns
/// * `Ok` with the 16-byte encrypted/decrypted result
/// * `Err(())` if the AES block never reported completion (see
///   [`AES_WAIT_MAX_ITERS`])
// The project convention for fallible hardware calls is `Result<_, ()>`
// (docs/checklists/firmware.md); there is no extra information to carry, so a
// bespoke error type would be noise.
#[allow(clippy::result_unit_err)]
pub fn aes_ll_encryption_checked(
    key: &[u8],
    source: &[u8],
    direction: AesDirection,
) -> Result<[u8; 16], ()> {
    #[cfg(not(target_os = "none"))]
    {
        fake_crypto::encryption_checked(key, source, direction)
    }
    #[cfg(target_os = "none")]
    {
        critical_section::with(|_| aes_mmio_run(key, source, direction))
    }
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
/// A 16-byte array containing the encrypted/decrypted result, or `[0u8; 16]` if
/// the hardware never reported completion ([`aes_ll_encryption_checked`] is the
/// form that reports the timeout).
///
/// # AES Implementation Notes
/// - On device: uses hardware AES acceleration; the TLSR8266 requires keys to be
///   written in reverse byte order, and data is written and read as 32-bit words
///   with hardware-specific endianness.
/// - On host: dispatches through [`fake_crypto`]. `SoftwareAes` (default) runs a
///   pure-Rust FIPS-197 AES-128 and never touches MMIO, so it also never spins
///   on the poll bit; `MmioRegisters` keeps the register loop for tests that are
///   about the bus protocol.
pub fn aes_ll_encryption(key: &[u8], source: &[u8], direction: AesDirection) -> [u8; 16] {
    // Pre-loaded with the fallback: `aes_run_into` leaves it untouched on a
    // hardware timeout, so the caller never sees a partially written block.
    let mut result = [0u8; 16];
    let _ = aes_run_into(key, source, direction, &mut result);
    result
}

/// [`aes_att_encryption`] that reports a hardware timeout instead of returning a
/// zero block, and writes through `out`.
///
/// Byte order is the same as [`aes_att_encryption`]. Taking `out` rather than
/// returning a `Result` keeps the fail-closed MIC path at the same stack cost as
/// the infallible one (see [`aes_mmio_run_into`]).
#[inline(always)]
fn aes_att_encryption_into(key: &[u8; 16], source: &[u8; 16], out: &mut [u8; 16]) -> bool {
    if !aes_run_into(key, source, AesDirection::Encrypt, out) {
        return false;
    }
    // Reverse the bytes of the result to match the required endianness/format.
    out.reverse();
    true
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
/// A 16-byte array containing the encrypted and byte-swapped data, or the zero
/// block if the AES hardware timed out (the peer rejects it; [`aes_att_decryption_packet`]
/// is the fail-closed path).
pub fn aes_att_encryption(key: &[u8; 16], source: &[u8; 16]) -> [u8; 16] {
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
pub fn aes_att_decryption(key: &[u8; 16], source: &[u8; 16]) -> [u8; 16] {
    // Perform low-level decryption using hardware acceleration.
    // Note: The same low-level function is used for both encryption and decryption,
    // controlled by the AesDirection enum.
    let mut result = aes_ll_encryption(key, source, AesDirection::Decrypt);
    // Reverse the bytes of the result to match the required endianness/format.
    result.reverse();
    // Return the final decrypted and byte-reversed result.
    result
}

/// Encodes (encrypts) a 16-byte password using a caller-supplied key.
///
/// This function uses the provided AES key to encrypt the password data via
/// `aes_att_encryption`. The caller owns the key: this module never reads global
/// state, so the caller is responsible for copying the key out of any lock and
/// releasing that lock before calling this function (the AES call masks
/// interrupts inside a critical section).
///
/// # Parameters
/// * `key` - A 16-byte array representing the AES key.
/// * `password` - A 16-byte array representing the plaintext password to be encoded.
///
/// # Returns
/// A 16-byte array containing the encrypted password.
pub fn encode_password(key: &[u8; 16], password: &[u8; 16]) -> [u8; 16] {
    aes_att_encryption(key, password)
}

/// Decodes (decrypts) a 16-byte password using a caller-supplied key.
///
/// This function uses the provided AES key to decrypt the encoded password data
/// via `aes_att_decryption`. This is the counterpart to `encode_password` and is
/// used to recover the original password from its encrypted form. The caller owns
/// the key: this module never reads global state, so copy the key out of any lock
/// and release that lock before calling (the AES call masks interrupts inside a
/// critical section).
///
/// # Parameters
/// * `key` - A 16-byte array representing the AES key.
/// * `password` - A 16-byte array representing the encrypted password to be decoded.
///
/// # Returns
/// A 16-byte array containing the decrypted (plaintext) password.
pub fn decode_password(key: &[u8; 16], password: &[u8; 16]) -> [u8; 16] {
    aes_att_decryption(key, password)
}

/// Decrypts a packet using the AES-128 algorithm with authenticated encryption (CCM-like mode).
///
/// This function implements a combined AES counter (CTR) mode for decryption with a MAC
/// (Message Authentication Code) for integrity verification. The process follows these phases:
///
/// 1. For empty packets, generate authentication tag directly and verify against `expected_mic`
/// 2. For non-empty packets:
///    a. Phase 1: Decrypt the packet data using AES-CTR mode
///    b. Phase 2: Calculate authentication tag (MIC) on the decrypted data
///    c. Phase 3: Verify integrity by comparing the calculated MIC with `expected_mic`
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
/// * `Ok(())` if the calculated MIC matches the `expected_mic` (verification passed)
/// * `Err(MicMismatch)` if verification failed, indicating corrupted data or wrong
///   encryption key
///
/// A hardware AES timeout also returns `Err(MicMismatch)`: a MIC check must fail
/// closed rather than validate against a block the hardware never computed.
pub fn aes_att_decryption_packet(
    key: &[u8; 16],
    ivm: &[u8; 8],
    expected_mic: &[u8],
    packet_data: &mut [u8],
) -> Result<(), MicMismatch> {
    // Phase 0: Handle empty packet special case
    if packet_data.is_empty() {
        // For empty packets, create an authentication block with specific format
        // Format: [IVM(8 bytes) | zeros(8 bytes)] with packet_len = 0
        let mut ivs = [0u8; 16];
        ivs[0..8].copy_from_slice(ivm);
        ivs[8] = 0; // packet_data.len() as u8
        ivs[12] = 0; // Another zero at position 12 (specific to empty packets)

        // Generate authentication tag (MIC) directly (no data to process)
        // This is similar to CBC-MAC of an empty message
        let mut auth_tag = [0u8; 16];
        if !aes_att_encryption_into(key, &ivs, &mut auth_tag) {
            // A hardware AES hang must never look like a valid tag.
            return Err(MicMismatch);
        }

        // Verify expected MIC matches the calculated tag by comparing the first n bytes
        // where n is the length of expected_mic
        if expected_mic[0..expected_mic.len()] == auth_tag[0..expected_mic.len()] {
            return Ok(());
        }
        return Err(MicMismatch);
    }

    // For non-empty packets, decryption happens in multiple phases:

    // Phase 1: Decrypt packet data using AES-CTR mode
    // Note: This happens BEFORE authentication (decrypt-then-authenticate pattern)
    // A hardware AES timeout maps to `Err(MicMismatch)`, so a stuck crypto block
    // can never be mistaken for a valid (all-zero keystream) packet.

    // Initialize counter block for AES-CTR mode
    // Format: [counter(1 byte) | IVM(8 bytes) | zeros(7 bytes)]
    let mut counter_block = [0u8; 16];
    counter_block[1..9].copy_from_slice(ivm);

    // Generate initial keystream block for the first 16 bytes
    let mut keystream_block = [0u8; 16];
    if !aes_att_encryption_into(key, &counter_block, &mut keystream_block) {
        return Err(MicMismatch);
    }

    // Process data in blocks of 16 bytes
    let mut index = 0;

    // Decrypt all bytes in the packet using AES-CTR mode
    while index < packet_data.len() {
        // When crossing a 16-byte boundary (except at the beginning),
        // increment counter and generate a new keystream block
        if index & 0xf == 0 && index > 0 {
            // Increment counter for next block
            counter_block[0] += 1;
            if !aes_att_encryption_into(key, &counter_block, &mut keystream_block) {
                return Err(MicMismatch);
            }
        }

        // Apply keystream to decrypt data using XOR
        // This is the core CTR mode operation: plaintext = ciphertext ⊕ keystream
        packet_data[index] ^= keystream_block[index & 0xf];
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
    let mut auth_value = [0u8; 16];
    if !aes_att_encryption_into(key, &auth_block, &mut auth_value) {
        return Err(MicMismatch);
    }

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
            // This is similar to a CBC-MAC operation. The tag is both input and
            // output, so it round-trips through one scratch block.
            let mut next_auth_value = [0u8; 16];
            if !aes_att_encryption_into(key, &auth_value, &mut next_auth_value) {
                return Err(MicMismatch);
            }
            auth_value = next_auth_value;
        }

        index += 1;
    }

    // Verify the integrity by comparing the expected MIC with the calculated authentication tag (MIC)
    if expected_mic[0..expected_mic.len()] == auth_value[0..expected_mic.len()] {
        Ok(())
    } else {
        Err(MicMismatch)
    }
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
///   of this slice determines how many bytes of the MIC are copied.
/// * `packet_data` - The plaintext packet data to be encrypted (modified in place)
pub fn aes_att_encryption_packet(
    key: &[u8; 16],
    ivm: &[u8; 8],
    mic_output: &mut [u8],
    packet_data: &mut [u8],
) {
    // Phase 0: Handle empty packet special case
    if packet_data.is_empty() {
        // For empty packets, create an authentication block with specific format
        let mut auth_block = [0u8; 16];
        auth_block[0..8].copy_from_slice(ivm);
        auth_block[8] = 0; // packet_data.len() as u8

        // Generate authentication tag (MIC) directly (no data to process)
        let auth_value = aes_att_encryption(key, &auth_block);

        // Copy MIC to output buffer (if provided)
        if !mic_output.is_empty() {
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
    if !mic_output.is_empty() {
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

/// Host-side AES transport + pure-Rust fallback (`cfg(not(target_os = "none"))`).
///
/// The moved unit tests and the downstream `ble_ll_pair` / `flash_config` /
/// `wire_contract_tests` suites need a real AES without mocking the hardware
/// register loop, so `SoftwareAes` is the default. `MmioRegisters` remains for
/// tests whose subject *is* the `0x540`/`0x548`/`0x550` write/poll/read sequence.
#[cfg(not(target_os = "none"))]
pub mod fake_crypto {
    use super::{software_aes, AesDirection};
    use std::sync::{Mutex, MutexGuard};

    /// How host [`super::aes_ll_encryption`] produces its 16-byte output.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum FakeAesMode {
        /// Pure-Rust FIPS-197 AES-128 with TLSR8266 byte-reversal parity (default).
        SoftwareAes,
        /// Drives the real MMIO write/poll/read loop against tlsr8266_pac::FakeMmio.
        MmioRegisters,
    }

    static MODE: Mutex<FakeAesMode> = Mutex::new(FakeAesMode::SoftwareAes);

    /// Recover from a poisoned lock so one panicking test cannot poison the rest
    /// of a `--test-threads=1` run (the `FakeMmio` convention).
    fn lock() -> MutexGuard<'static, FakeAesMode> {
        MODE.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn mode() -> FakeAesMode {
        *lock()
    }

    /// Select the host AES transport.
    pub fn set_mode(mode: FakeAesMode) {
        *lock() = mode;
    }

    /// Restore the default transport (`SoftwareAes`).
    pub fn reset() {
        *lock() = FakeAesMode::SoftwareAes;
    }

    /// Host `aes_mmio_run_into` body: writes the block through `out`.
    ///
    /// The pure-Rust transport cannot time out, so it always reports success;
    /// the `MmioRegisters` transport surfaces the bounded AES completion wait.
    pub(crate) fn encryption_into(
        key: &[u8],
        source: &[u8],
        direction: AesDirection,
        out: &mut [u8; 16],
    ) -> bool {
        match mode() {
            FakeAesMode::SoftwareAes => {
                *out = software_aes::ll_parity(key, source, direction);
                true
            }
            FakeAesMode::MmioRegisters => super::aes_mmio_run_into(key, source, direction, out),
        }
    }

    /// Host `aes_ll_encryption_checked` body.
    pub(crate) fn encryption_checked(
        key: &[u8],
        source: &[u8],
        direction: AesDirection,
    ) -> Result<[u8; 16], ()> {
        match mode() {
            FakeAesMode::SoftwareAes => Ok(software_aes::ll_parity(key, source, direction)),
            FakeAesMode::MmioRegisters => super::aes_mmio_run(key, source, direction),
        }
    }
}

/// Pure-Rust FIPS-197 AES-128 (host only, no extra crate).
///
/// `aes_ll_encryption` hands the chip's byte order straight through: the chip
/// takes a reversed key and a reversed data block and returns the block in the
/// order `aes_att_encryption`/`aes_att_decryption` then reverse. So this module
/// exposes exactly `FIPS197_AES128(reverse(key), reverse(source), dir)` and
/// returns it **without** an output reversal; `tests/wire_vectors.txt` pins that
/// convention byte for byte.
#[cfg(not(target_os = "none"))]
mod software_aes {
    use super::AesDirection;

    /// FIPS-197 S-box.
    #[rustfmt::skip]
    const SBOX: [u8; 256] = [
        0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
        0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
        0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
        0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
        0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
        0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
        0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
        0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
        0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
        0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
        0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
        0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
        0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
        0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
        0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
        0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
    ];

    /// FIPS-197 inverse S-box.
    #[rustfmt::skip]
    const INV_SBOX: [u8; 256] = [
        0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
        0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
        0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
        0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
        0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
        0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
        0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
        0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
        0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
        0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
        0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
        0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
        0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
        0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
        0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
        0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d,
    ];

    /// Multiplication in GF(2^8) modulo the AES polynomial `x^8+x^4+x^3+x+1`.
    fn gmul(mut a: u8, mut b: u8) -> u8 {
        let mut product = 0u8;
        for _ in 0..8 {
            if b & 1 != 0 {
                product ^= a;
            }
            let carry = a & 0x80;
            a <<= 1;
            if carry != 0 {
                a ^= 0x1b;
            }
            b >>= 1;
        }
        product
    }

    /// FIPS-197 `xtime`.
    fn xtime(b: u8) -> u8 {
        (b << 1) ^ if b & 0x80 != 0 { 0x1b } else { 0 }
    }

    /// FIPS-197 §5.2 key expansion: 11 round keys of 16 bytes.
    fn expand_key(key: &[u8; 16]) -> [u8; 176] {
        let mut w = [0u8; 176];
        w[..16].copy_from_slice(key);
        let mut rcon = 1u8;
        for i in (16..176).step_by(4) {
            let mut t = [w[i - 4], w[i - 3], w[i - 2], w[i - 1]];
            if i % 16 == 0 {
                // RotWord -> SubWord -> Rcon
                t = [
                    SBOX[t[1] as usize] ^ rcon,
                    SBOX[t[2] as usize],
                    SBOX[t[3] as usize],
                    SBOX[t[0] as usize],
                ];
                rcon = xtime(rcon);
            }
            for j in 0..4 {
                w[i + j] = w[i - 16 + j] ^ t[j];
            }
        }
        w
    }

    /// `state[r][c]` is byte `4*c + r` of the block, per FIPS-197 §3.4.
    fn add_round_key(state: &mut [u8; 16], w: &[u8; 176], round: usize) {
        for i in 0..16 {
            state[i] ^= w[round * 16 + i];
        }
    }

    fn sub_bytes(state: &mut [u8; 16], inverse: bool) {
        let table = if inverse { &INV_SBOX } else { &SBOX };
        for b in state.iter_mut() {
            *b = table[*b as usize];
        }
    }

    fn shift_rows(state: &mut [u8; 16], inverse: bool) {
        let mut out = [0u8; 16];
        for c in 0..4 {
            for r in 0..4 {
                let src = if inverse {
                    (c + 4 - r) % 4
                } else {
                    (c + r) % 4
                };
                out[c * 4 + r] = state[src * 4 + r];
            }
        }
        *state = out;
    }

    fn mix_columns(state: &mut [u8; 16], inverse: bool) {
        for c in 0..4 {
            let a = [
                state[c * 4],
                state[c * 4 + 1],
                state[c * 4 + 2],
                state[c * 4 + 3],
            ];
            let mut out = [0u8; 4];
            for i in 0..4 {
                out[i] = if inverse {
                    gmul(a[i], 14)
                        ^ gmul(a[(i + 1) % 4], 11)
                        ^ gmul(a[(i + 2) % 4], 13)
                        ^ gmul(a[(i + 3) % 4], 9)
                } else {
                    gmul(a[i], 2) ^ gmul(a[(i + 1) % 4], 3) ^ a[(i + 2) % 4] ^ a[(i + 3) % 4]
                };
            }
            state[c * 4..c * 4 + 4].copy_from_slice(&out);
        }
    }

    /// One AES-128 block in FIPS-197 byte order.
    fn aes128(key: &[u8; 16], block: &[u8; 16], direction: AesDirection) -> [u8; 16] {
        let w = expand_key(key);
        let mut state = *block;
        if direction == AesDirection::Encrypt {
            add_round_key(&mut state, &w, 0);
            for round in 1..10 {
                sub_bytes(&mut state, false);
                shift_rows(&mut state, false);
                mix_columns(&mut state, false);
                add_round_key(&mut state, &w, round);
            }
            sub_bytes(&mut state, false);
            shift_rows(&mut state, false);
            add_round_key(&mut state, &w, 10);
        } else {
            add_round_key(&mut state, &w, 10);
            for round in (1..10).rev() {
                shift_rows(&mut state, true);
                sub_bytes(&mut state, true);
                add_round_key(&mut state, &w, round);
                mix_columns(&mut state, true);
            }
            shift_rows(&mut state, true);
            sub_bytes(&mut state, true);
            add_round_key(&mut state, &w, 0);
        }
        state
    }

    /// TLSR8266 parity for `aes_ll_encryption`.
    ///
    /// The chip consumes reversed key/data and returns the block in the order
    /// the ATT wrappers then reverse, so the equivalent computation is
    /// `reverse(FIPS197_AES128(reverse(key), reverse(source)))` — expressed here
    /// as "feed the cipher the reversed inputs, return its output as-is".
    pub(crate) fn ll_parity(key: &[u8], source: &[u8], direction: AesDirection) -> [u8; 16] {
        let mut reversed_key = [0u8; 16];
        let mut reversed_source = [0u8; 16];
        for i in 0..16 {
            reversed_key[i] = key[15 - i];
            reversed_source[i] = source[15 - i];
        }
        aes128(&reversed_key, &reversed_source, direction)
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_pac::REG_BASE_ADDR;

    /// The three registers of the AES block.
    const AES_CTRL: u32 = 0x540;
    const AES_DATA: u32 = 0x548;
    const AES_KEY: u32 = 0x550;

    /// `FakeMmio` and `fake_crypto` are process-wide and the suite runs with
    /// `--test-threads=1`, so every test starts from a zeroed bank running the
    /// pure-Rust `SoftwareAes` transport.
    fn reset_fakes() {
        FakeMmio::reset();
        fake_crypto::reset();
    }

    /// Switch the host AES transport to the real `0x540`/`0x548`/`0x550` loop.
    fn use_mmio_transport() {
        reset_fakes();
        fake_crypto::set_mode(fake_crypto::FakeAesMode::MmioRegisters);
    }

    /// Every 32-bit write to the AES data register, in call order.
    fn aes_data_writes() -> Vec<u32> {
        FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == AES_DATA | REG_BASE_ADDR)
            .map(|w| w.value)
            .collect()
    }

    /// The key bytes left in `reg_aes_key0..15`.
    fn aes_key_bytes() -> [u8; 16] {
        let mut key = [0u8; 16];
        for (i, byte) in key.iter_mut().enumerate() {
            *byte = FakeMmio::reg8(AES_KEY + i as u32);
        }
        key
    }

    /// The CTR keystream block the packet helpers generate for `counter`.
    ///
    /// Block format: `[counter(1 byte) | IVM(8 bytes) | zeros(7 bytes)]`.
    fn ctr_block(key: &[u8; 16], ivm: &[u8; 8], counter: u8) -> [u8; 16] {
        let mut counter_block = [0u8; 16];
        counter_block[0] = counter;
        counter_block[1..9].copy_from_slice(ivm);
        aes_att_encryption(key, &counter_block)
    }

    /// The CBC-MAC-style tag the packet helpers derive over `data`.
    ///
    /// Block format: `[IVM(8 bytes) | len(1 byte) | zeros(7 bytes)]`, then each
    /// byte is XORed into the running tag and the block is re-encrypted at every
    /// 16-byte boundary (and at the end).
    fn auth_tag(key: &[u8; 16], ivm: &[u8; 8], data: &[u8]) -> [u8; 16] {
        let mut auth_block = [0u8; 16];
        auth_block[0..8].copy_from_slice(ivm);
        auth_block[8] = data.len() as u8;

        let mut auth_value = aes_att_encryption(key, &auth_block);
        for (index, byte) in data.iter().enumerate() {
            auth_value[index & 0xf] ^= *byte;
            if (index & 0xf) == 0xf || index == data.len() - 1 {
                auth_value = aes_att_encryption(key, &auth_value);
            }
        }
        auth_value
    }

    /// CTR-apply the keystream the packet helpers use to `data` in place.
    fn ctr_apply(key: &[u8; 16], ivm: &[u8; 8], data: &mut [u8]) {
        let mut counter: u8 = 0;
        let mut keystream = ctr_block(key, ivm, counter);
        for index in 0..data.len() {
            if index & 0xf == 0 && index > 0 {
                counter += 1;
                keystream = ctr_block(key, ivm, counter);
            }
            data[index] ^= keystream[index & 0xf];
        }
    }

    #[test]
    fn test_aes_ll_encryption_basic() {
        use_mmio_transport();

        // Arrange
        let key = [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,
            0x0E, 0x0F,
        ];
        let source = [
            0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D,
            0x1E, 0x1F,
        ];
        let direction = AesDirection::Encrypt;

        // The completion poll reports "busy" once, then "done"
        FakeMmio::on_read8(AES_CTRL, |call_idx| if call_idx == 0 { 0 } else { 4 });
        // The result words the hardware would present, in read order
        const RESULT_WORDS: [u32; 4] = [0x20212223, 0x24252627, 0x28292A2B, 0x2C2D2E2F];
        FakeMmio::on_read32(AES_DATA, |call_idx| RESULT_WORDS[call_idx]);

        // Act
        let result = aes_ll_encryption(&key, &source, direction);

        // Assert
        // The direction was written to the control register
        assert_eq!(FakeMmio::reg8(AES_CTRL), direction as u8);

        // The key was written in reverse order
        assert_eq!(aes_key_bytes(), {
            let mut expected = [0u8; 16];
            for i in 0..16 {
                expected[i] = key[15 - i];
            }
            expected
        });

        // The data words were written high-to-low address, big-endian packed
        assert_eq!(
            aes_data_writes(),
            vec![0x1C1D1E1F, 0x18191A1B, 0x14151617, 0x10111213]
        );

        // The result words were unpacked little-endian
        assert_eq!(
            result,
            [
                0x23, 0x22, 0x21, 0x20, 0x27, 0x26, 0x25, 0x24, 0x2B, 0x2A, 0x29, 0x28, 0x2F, 0x2E,
                0x2D, 0x2C
            ]
        );
    }

    #[test]
    fn test_aes_ll_encryption_decrypt_mode() {
        use_mmio_transport();

        // Arrange
        let key = [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,
            0x0E, 0x0F,
        ];
        let source = [
            0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D,
            0x2E, 0x2F,
        ];
        let direction = AesDirection::Decrypt;

        FakeMmio::on_read8(AES_CTRL, |call_idx| if call_idx == 0 { 0 } else { 4 });
        const RESULT_WORDS: [u32; 4] = [0x10111213, 0x14151617, 0x18191A1B, 0x1C1D1E1F];
        FakeMmio::on_read32(AES_DATA, |call_idx| RESULT_WORDS[call_idx]);

        // Act
        let result = aes_ll_encryption(&key, &source, direction);

        // Assert
        assert_eq!(FakeMmio::reg8(AES_CTRL), direction as u8);
        assert_eq!(
            aes_data_writes(),
            vec![0x2C2D2E2F, 0x28292A2B, 0x24252627, 0x20212223]
        );
        assert_eq!(
            result,
            [
                0x13, 0x12, 0x11, 0x10, 0x17, 0x16, 0x15, 0x14, 0x1B, 0x1A, 0x19, 0x18, 0x1F, 0x1E,
                0x1D, 0x1C
            ]
        );
    }

    #[test]
    fn test_aes_ll_encryption_multiple_polls() {
        use_mmio_transport();

        // Arrange
        let key = [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,
            0x0E, 0x0F,
        ];
        let source = [
            0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D,
            0x1E, 0x1F,
        ];
        let direction = AesDirection::Encrypt;

        // The completion poll stays "busy" for four reads: the loop can only
        // leave once it has polled a fifth time.
        FakeMmio::on_read8(AES_CTRL, |call_idx| if call_idx < 4 { 0 } else { 4 });
        const RESULT_WORDS: [u32; 4] = [0x20212223, 0x24252627, 0x28292A2B, 0x2C2D2E2F];
        FakeMmio::on_read32(AES_DATA, |call_idx| RESULT_WORDS[call_idx]);

        // Act
        let result = aes_ll_encryption(&key, &source, direction);

        // Assert
        assert_eq!(FakeMmio::reg8(AES_CTRL), direction as u8);
        assert_eq!(
            aes_data_writes(),
            vec![0x1C1D1E1F, 0x18191A1B, 0x14151617, 0x10111213]
        );
        assert_eq!(
            result,
            [
                0x23, 0x22, 0x21, 0x20, 0x27, 0x26, 0x25, 0x24, 0x2B, 0x2A, 0x29, 0x28, 0x2F, 0x2E,
                0x2D, 0x2C
            ]
        );
    }

    #[test]
    fn test_aes_ll_encryption_byte_order() {
        // This test verifies that the byte ordering in the AES implementation is correct
        // The TLSR8266 uses a specific byte ordering for AES operations
        use_mmio_transport();

        // Arrange
        let key = [
            0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54,
            0x32, 0x10,
        ];
        let source = [
            0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
            0x88, 0x99,
        ];
        let direction = AesDirection::Encrypt;

        FakeMmio::on_read8(AES_CTRL, |call_idx| if call_idx == 0 { 0 } else { 4 });
        const RESULT_WORDS: [u32; 4] = [0xAABBCCDD, 0xEEFF0011, 0x22334455, 0x66778899];
        FakeMmio::on_read32(AES_DATA, |call_idx| RESULT_WORDS[call_idx]);

        // Act
        let result = aes_ll_encryption(&key, &source, direction);

        // Assert
        // Verify the key bytes were written in reverse order
        assert_eq!(
            aes_key_bytes(),
            [
                0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE, 0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45,
                0x23, 0x01
            ]
        );

        // Verify data was packed correctly with the most significant byte first
        assert_eq!(
            aes_data_writes(),
            vec![0x66778899, 0x22334455, 0xEEFF0011, 0xAABBCCDD]
        );

        // Verify the output data is correctly extracted from the 32-bit registers
        assert_eq!(
            result,
            [
                0xDD, 0xCC, 0xBB, 0xAA, 0x11, 0x00, 0xFF, 0xEE, 0x55, 0x44, 0x33, 0x22, 0x99, 0x88,
                0x77, 0x66
            ]
        );
    }

    /// Reads of `reg_aes_ctrl` observed by [`stuck_done_bit`].
    static AES_CTRL_POLLS: AtomicUsize = AtomicUsize::new(0);

    /// Reports an AES block that never sets its done bit (bit 2 of `reg_aes_ctrl`).
    fn stuck_done_bit(_call_idx: usize) -> u8 {
        AES_CTRL_POLLS.fetch_add(1, Ordering::Relaxed);
        0
    }

    /// A done bit that never appears is reported by the checked form, and the
    /// infallible form falls back to the zero block.
    #[test]
    fn test_aes_ll_encryption_checked_times_out() {
        use_mmio_transport();
        AES_CTRL_POLLS.store(0, Ordering::Relaxed);
        FakeMmio::on_read8(AES_CTRL, stuck_done_bit);

        let key = [0x11u8; 16];
        let source = [0x22u8; 16];

        // Act
        let checked = aes_ll_encryption_checked(&key, &source, AesDirection::Encrypt);

        // Assert
        assert_eq!(
            checked,
            Err(()),
            "a stuck done bit must be reported, not silently zeroed"
        );
        assert_eq!(
            AES_CTRL_POLLS.load(Ordering::Relaxed),
            AES_WAIT_MAX_ITERS as usize,
            "the completion wait must stop after AES_WAIT_MAX_ITERS polls"
        );

        // The infallible wrapper collapses the same timeout to the zero block.
        assert_eq!(
            aes_ll_encryption(&key, &source, AesDirection::Encrypt),
            [0u8; 16],
            "the infallible form must fall back to the zero block"
        );
    }

    /// A hardware AES hang must fail `aes_att_decryption_packet` closed rather
    /// than validating a packet against an all-zero (never computed) tag.
    #[test]
    fn test_aes_att_decryption_packet_fails_closed_on_hang() {
        use_mmio_transport();
        AES_CTRL_POLLS.store(0, Ordering::Relaxed);
        FakeMmio::on_read8(AES_CTRL, stuck_done_bit);

        let key = [0x01u8; 16];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        // An all-zero expected MIC would "match" a zeroed tag, so this test
        // proves the hang is reported before any comparison happens.
        let expected_mic = [0x00u8; 4];
        let mut packet_data = [0xA1u8, 0xA2, 0xA3];

        // Act
        let result = aes_att_decryption_packet(&key, &ivm, &expected_mic, &mut packet_data);

        // Assert
        assert_eq!(
            result,
            Err(MicMismatch),
            "a hardware AES timeout must be rejected as a MIC mismatch"
        );
    }

    /// `aes_att_encryption` must be exactly the reversed `aes_ll_encryption`
    /// output, and that convention must land on the published FIPS-197 vector.
    ///
    /// The FIPS-197 AES-128 example is key `000102…0f`, plaintext `001122…ff`,
    /// ciphertext `69c4e0d8…c55a`. The ATT helpers feed the chip reversed key and
    /// reversed data, so reversing that vector's inputs and output is the
    /// TLSR8266 convention this test pins.
    #[test]
    fn test_aes_att_encryption() {
        reset_fakes();

        // Arrange: the FIPS-197 vector, byte-reversed on input and output
        let key = [
            0x0F, 0x0E, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02,
            0x01, 0x00,
        ];
        let source = [
            0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22,
            0x11, 0x00,
        ];
        let expected = [
            0x5A, 0xC5, 0xB4, 0x70, 0x80, 0xB7, 0xCD, 0xD8, 0x30, 0x04, 0x7B, 0x6A, 0xD8, 0xE0,
            0xC4, 0x69,
        ];

        // Act
        let result = aes_att_encryption(&key, &source);

        // Assert: the published ciphertext, byte-reversed
        assert_eq!(result, expected);

        // Assert: the ATT wrapper is the reversed low-level output
        let mut ll = aes_ll_encryption(&key, &source, AesDirection::Encrypt);
        ll.reverse();
        assert_eq!(
            result, ll,
            "aes_att_encryption must reverse the aes_ll_encryption output"
        );
    }

    /// `aes_att_decryption` must be exactly the reversed `aes_ll_encryption`
    /// output for `AesDirection::Decrypt`, and must invert encryption.
    #[test]
    fn test_aes_att_decryption() {
        reset_fakes();

        // Arrange
        let key = [
            0x0F, 0x0E, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02,
            0x01, 0x00,
        ];
        // The FIPS-197 ciphertext, byte-reversed
        let source = [
            0x5A, 0xC5, 0xB4, 0x70, 0x80, 0xB7, 0xCD, 0xD8, 0x30, 0x04, 0x7B, 0x6A, 0xD8, 0xE0,
            0xC4, 0x69,
        ];
        let expected_plaintext = [
            0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22,
            0x11, 0x00,
        ];

        // Act
        let result = aes_att_decryption(&key, &source);

        // Assert
        assert_eq!(result, expected_plaintext);

        let mut ll = aes_ll_encryption(&key, &source, AesDirection::Decrypt);
        ll.reverse();
        assert_eq!(
            result, ll,
            "aes_att_decryption must reverse the aes_ll_encryption output"
        );
    }

    #[test]
    fn test_encode_password() {
        reset_fakes();

        // Arrange
        let password = [0xAAu8; 16];
        // Test-local key: this module is a pure leaf HAL and reads no global state
        let key = [0x42u8; 16];

        // Act
        let result = encode_password(&key, &password);

        // Assert
        // The result is what aes_att_encryption produces for the supplied key
        // and password, so a swapped or dropped argument changes the value.
        assert_eq!(result, aes_att_encryption(&key, &password));
        assert_ne!(result, password, "encoding must not be the identity");
        assert_ne!(
            result,
            aes_att_encryption(&[0x43u8; 16], &password),
            "the caller's key must reach the cipher"
        );
    }

    #[test]
    fn test_decode_password() {
        reset_fakes();

        // Arrange
        let plaintext = [0xDDu8; 16];
        // Test-local key: this module is a pure leaf HAL and reads no global state
        let key = [0x43u8; 16];
        let encoded_password = encode_password(&key, &plaintext);

        // Act
        let result = decode_password(&key, &encoded_password);

        // Assert
        assert_eq!(result, plaintext, "decoding must recover the plaintext");
        assert_eq!(
            result,
            aes_att_decryption(&key, &encoded_password),
            "decode_password must forward to aes_att_decryption"
        );
    }

    #[test]
    fn test_aes_att_decryption_packet_empty() {
        reset_fakes();

        // Arrange
        let key = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10,
        ];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let mut packet_data: [u8; 0] = []; // Empty packet data

        // The empty-packet tag is the encryption of `[IVM | zeros]`
        let mut ivs = [0u8; 16];
        ivs[0..8].copy_from_slice(&ivm);
        let expected_tag = aes_att_encryption(&key, &ivs);
        let expected_mic = &expected_tag[0..3];

        // Act
        let result = aes_att_decryption_packet(&key, &ivm, expected_mic, &mut packet_data);

        // Assert
        assert!(
            result.is_ok(),
            "an empty packet must validate against its own tag"
        );

        // A tag that differs in the last compared byte must be rejected
        let mut wrong_mic = [expected_mic[0], expected_mic[1], expected_mic[2]];
        wrong_mic[2] ^= 0x01;
        assert!(
            aes_att_decryption_packet(&key, &ivm, &wrong_mic, &mut packet_data).is_err(),
            "a mismatched MIC must be rejected"
        );
    }

    #[test]
    fn test_aes_att_decryption_packet_small() {
        reset_fakes();

        // Arrange
        let key = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10,
        ];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];

        // A 5-byte plaintext (less than one AES block)
        let plaintext = [0xA1, 0xA2, 0xA3, 0xA4, 0xA5];
        let mut packet_data = plaintext;
        ctr_apply(&key, &ivm, &mut packet_data); // ciphertext
        let ciphertext = packet_data;
        let mic = auth_tag(&key, &ivm, &plaintext);

        // Act
        let result = aes_att_decryption_packet(&key, &ivm, &mic[0..4], &mut packet_data);

        // Assert
        assert!(result.is_ok(), "the derived MIC must validate");
        assert_eq!(
            packet_data, plaintext,
            "CTR keystream must be XORed into every byte"
        );

        // Re-encrypting the recovered plaintext must reproduce the ciphertext and
        // the same tag, so the two directions really are inverses.
        let mut round_trip = plaintext;
        let mut round_trip_mic = [0u8; 4];
        aes_att_encryption_packet(&key, &ivm, &mut round_trip_mic, &mut round_trip);
        assert_eq!(round_trip, ciphertext);
        assert_eq!(round_trip_mic, mic[0..4]);
    }

    #[test]
    fn test_aes_att_decryption_packet_large() {
        reset_fakes();

        // Arrange
        let key = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10,
        ];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let expected_mic: [u8; 0] = []; // No validation

        // 20 bytes, crossing the 16-byte boundary
        let plaintext = [
            0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE,
            0xAF, 0xB0, 0xB1, 0xB2, 0xB3, 0xB4,
        ];

        // Build the ciphertext from the documented keystream blocks: the first
        // block uses counter 0, the four trailing bytes counter 1.
        let block0 = ctr_block(&key, &ivm, 0);
        let block1 = ctr_block(&key, &ivm, 1);
        assert_ne!(block0, block1, "the counter must change the keystream");
        let mut packet_data = plaintext;
        for i in 0..16 {
            packet_data[i] ^= block0[i];
        }
        for i in 16..20 {
            packet_data[i] ^= block1[i - 16];
        }

        // Act
        let result = aes_att_decryption_packet(&key, &ivm, &expected_mic, &mut packet_data);

        // Assert
        assert!(result.is_ok());
        assert_eq!(
            packet_data, plaintext,
            "the second block must use the incremented counter"
        );
    }

    #[test]
    fn test_aes_att_decryption_packet_validation_success() {
        reset_fakes();

        // Arrange
        let key = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10,
        ];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];

        let plaintext = [0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8];
        let mut packet_data = plaintext;
        ctr_apply(&key, &ivm, &mut packet_data);

        // The tag of the *decrypted* data is what the receiver compares
        let tag = auth_tag(&key, &ivm, &plaintext);

        // Act
        let result = aes_att_decryption_packet(&key, &ivm, &tag[0..4], &mut packet_data);

        // Assert
        assert!(result.is_ok(), "a matching MIC must validate");
    }

    #[test]
    fn test_aes_att_decryption_packet_validation_failure() {
        reset_fakes();

        // Arrange
        let key = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10,
        ];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];

        let plaintext = [0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8];
        let mut packet_data = plaintext;
        ctr_apply(&key, &ivm, &mut packet_data);

        // These values won't match what encryption returns
        let expected_mic = [0xFF, 0xFF, 0xFF, 0xFF];

        // Act
        let result = aes_att_decryption_packet(&key, &ivm, &expected_mic, &mut packet_data);

        // Assert
        assert!(result.is_err(), "a mismatched MIC must be rejected");
    }

    #[test]
    fn test_aes_att_decryption_packet_alignment_boundaries() {
        // This test verifies behavior around the 16-byte (0xF) alignment boundaries

        reset_fakes();

        // Arrange
        let key = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10,
        ];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let expected_mic: [u8; 0] = []; // No validation needed

        // Exactly 16 bytes (one full block)
        let plaintext = [
            0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE,
            0xAF, 0xB0,
        ];

        // Every byte must use the first keystream block: the counter must not be
        // bumped inside the only block.
        let block0 = ctr_block(&key, &ivm, 0);
        let mut packet_data = plaintext;
        for i in 0..16 {
            packet_data[i] ^= block0[i];
        }

        // Act
        let result = aes_att_decryption_packet(&key, &ivm, &expected_mic, &mut packet_data);

        // Assert
        assert!(result.is_ok());
        assert_eq!(packet_data, plaintext);
    }

    #[test]
    fn test_aes_att_decryption_packet_complex_encryption_flow() {
        reset_fakes();

        // Arrange
        let key = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10,
        ];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];

        // 17 bytes: one full block plus a single byte in the next counter block
        let plaintext = [
            0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE,
            0xAF, 0xB0, 0xB1,
        ];
        let mut packet_data = plaintext;
        ctr_apply(&key, &ivm, &mut packet_data);
        let tag = auth_tag(&key, &ivm, &plaintext);

        // Act
        let result = aes_att_decryption_packet(&key, &ivm, &tag[0..3], &mut packet_data);

        // Assert
        // The whole flow (decrypt, CBC-MAC over 17 bytes, tag compare) succeeds
        assert!(result.is_ok());
        assert_eq!(packet_data, plaintext);
    }

    #[test]
    fn test_aes_att_encryption_packet_empty() {
        reset_fakes();

        // Arrange
        let key = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10,
        ];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let mut src = [0xA1, 0xA2, 0xA3, 0xA4]; // MIC output buffer
        let mut packet_data: [u8; 0] = []; // Empty packet data

        // With empty packet data the tag is the encryption of `[IVM | zeros]`
        let mut auth_block = [0u8; 16];
        auth_block[0..8].copy_from_slice(&ivm);
        let expected = aes_att_encryption(&key, &auth_block);

        // Act
        aes_att_encryption_packet(&key, &ivm, &mut src, &mut packet_data);

        // Assert
        // src was filled with the first four bytes of the tag
        assert_eq!(src, [expected[0], expected[1], expected[2], expected[3]]);
    }

    #[test]
    fn test_aes_att_encryption_packet_small() {
        reset_fakes();

        // Arrange
        let key = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10,
        ];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let mut src = [0xA1, 0xA2, 0xA3, 0xA4]; // MIC output buffer
        let plaintext = [0xB1, 0xB2, 0xB3, 0xB4, 0xB5];
        let mut packet_data = plaintext;

        // The tag is derived from the plaintext, then CTR is applied
        let expected_tag = auth_tag(&key, &ivm, &plaintext);
        let mut expected_packet = plaintext;
        ctr_apply(&key, &ivm, &mut expected_packet);

        // Act
        aes_att_encryption_packet(&key, &ivm, &mut src, &mut packet_data);

        // Assert
        assert_eq!(src, expected_tag[0..4], "MIC comes from the plaintext");
        assert_eq!(packet_data, expected_packet);
    }

    #[test]
    fn test_aes_att_encryption_packet_large() {
        reset_fakes();

        // Arrange
        let key = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10,
        ];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let mut src = [0xA1, 0xA2]; // 2-byte MIC output buffer
                                    // Large packet that crosses the 16-byte boundary
        let plaintext = [
            0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE,
            0xBF, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4,
        ];
        let mut packet_data = plaintext;

        let expected_tag = auth_tag(&key, &ivm, &plaintext);
        let block0 = ctr_block(&key, &ivm, 0);
        let block1 = ctr_block(&key, &ivm, 1);

        // Act
        aes_att_encryption_packet(&key, &ivm, &mut src, &mut packet_data);

        // Assert
        assert_eq!(src, expected_tag[0..2]);
        for i in 0..16 {
            assert_eq!(packet_data[i], plaintext[i] ^ block0[i]);
        }
        for i in 16..plaintext.len() {
            assert_eq!(packet_data[i], plaintext[i] ^ block1[i - 16]);
        }
    }

    #[test]
    fn test_aes_att_encryption_packet_alignment_boundary() {
        reset_fakes();

        // Arrange
        let key = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10,
        ];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let mut src = [0xA1, 0xA2, 0xA3, 0xA4]; // MIC output buffer
                                                // Exactly 16 bytes (one full block)
        let plaintext = [
            0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE,
            0xBF, 0xC0,
        ];
        let mut packet_data = plaintext;

        let expected_tag = auth_tag(&key, &ivm, &plaintext);
        let block0 = ctr_block(&key, &ivm, 0);

        // Act
        aes_att_encryption_packet(&key, &ivm, &mut src, &mut packet_data);

        // Assert
        assert_eq!(src, expected_tag[0..4]);
        for i in 0..plaintext.len() {
            assert_eq!(packet_data[i], plaintext[i] ^ block0[i]);
        }
    }

    #[test]
    fn test_aes_att_encryption_packet_empty_src() {
        reset_fakes();

        // Arrange
        let key = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10,
        ];
        let ivm = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
        let mut src: [u8; 0] = []; // Empty MIC output buffer
        let plaintext = [0xB1, 0xB2, 0xB3, 0xB4, 0xB5];
        let mut packet_data = plaintext;

        let mut expected_packet = plaintext;
        ctr_apply(&key, &ivm, &mut expected_packet);

        // Act
        aes_att_encryption_packet(&key, &ivm, &mut src, &mut packet_data);

        // Assert
        // src stays empty
        assert_eq!(src.len(), 0);

        // The packet is still encrypted with the counter-mode keystream
        assert_eq!(packet_data, expected_packet);
    }
}
