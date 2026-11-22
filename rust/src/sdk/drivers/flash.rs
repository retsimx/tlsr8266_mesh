use crate::sdk::drivers::spi::{
    mspi_ctrl_write, mspi_get, mspi_high, mspi_low, mspi_read, mspi_wait, mspi_write,
};
use crate::sdk::mcu::clock::sleep_us;
use crate::sdk::mcu::watchdog::wd_clear;
use core::mem::transmute;
use core::ptr::{null, null_mut};

#[cfg(test)]
use self::fake_flash::FakeFlash;

pub const PAGE_SIZE: u32 = 256;
pub const PAGE_SIZE_OTP: u32 = 256;
pub const FLASH_LOCK_EN: u32 = 0;

/// Maximum number of iterations for the flash busy wait loop.
/// Set to a lower value for testing to speed up tests.
#[cfg(test)]
pub const FLASH_WAIT_ITERATIONS: u32 = 1000;
#[cfg(not(test))]
pub const FLASH_WAIT_ITERATIONS: u32 = 10000000;

/*
 * @brief     flash command definition
 */
enum FLASH_CMD {
    //common cmd
    WRITE_CMD = 0x02,
    READ_CMD = 0x03,
    WRITE_SECURITY_REGISTERS_CMD = 0x42,
    READ_SECURITY_REGISTERS_CMD = 0x48,

    SECT_ERASE_CMD = 0x20,
    ERASE_SECURITY_REGISTERS_CMD = 0x44,

    READ_UID_CMD_GD_PUYA_ZB_UT = 0x4B, //Flash Type = GD/PUYA/ZB/UT
    READ_UID_CMD_XTX = 0x5A,           //Flash Type = XTX

    GET_JEDEC_ID = 0x9F,

    //special cmd
    WRITE_STATUS_CMD_LOWBYTE = 0x01,
    WRITE_STATUS_CMD_HIGHBYTE = 0x31,

    READ_STATUS_CMD_LOWBYTE = 0x05,
    READ_STATUS_CMD_HIGHBYTE = 0x35,

    WRITE_DISABLE_CMD = 0x04,
    WRITE_ENABLE_CMD = 0x06,
}

// Implement From trait for more idiomatic conversion from enum to u8
impl From<FLASH_CMD> for u8 {
    fn from(cmd: FLASH_CMD) -> u8 {
        cmd as u8
    }
}

/*
 * @brief     flash status type definition
 */
enum FLASH_STATUS {
    TYPE_8BIT_STATUS = 0,
    TYPE_16BIT_STATUS_ONE_CMD = 1,
    TYPE_16BIT_STATUS_TWO_CMD = 2,
}

/*
 * @brief     flash uid cmd definition
 */
enum FLASH_UID_CMD {
    UID_CMD_GD_PUYA = 0x4b,
    XTX_READ_UID_CMD = 0x5A,
}

/*
 * @brief	flash capacity definition
 *			Call flash_read_mid function to get the size of flash capacity.
 *			Example is as follows:
 *			unsigned int mid = flash_read_mid();
 *			The value of (mid&0x00ff0000)>>16 reflects flash capacity.
 */
enum FLASH_CAPACITY {
    SIZE_64K = 0x10,
    SIZE_128K = 0x11,
    SIZE_256K = 0x12,
    SIZE_512K = 0x13,
    SIZE_1M = 0x14,
    SIZE_2M = 0x15,
    SIZE_4M = 0x16,
    SIZE_8M = 0x17,
}

/**
 * @brief	flash voltage definition
 */
enum FLASH_VOLTAGE {
    VOLTAGE_1V95 = 0x07,
    VOLTAGE_1V9 = 0x06,
    VOLTAGE_1V85 = 0x05,
    VOLTAGE_1V8 = 0x04,
    VOLTAGE_1V75 = 0x03,
    VOLTAGE_1V7 = 0x02,
    VOLTAGE_1V65 = 0x01,
    VOLTAGE_1V6 = 0x00,
}

/// Sends a command byte to the flash memory over SPI.
///
/// This function handles the SPI protocol for sending a command to the flash chip.
/// It ensures proper timing and signal transitions for reliable communication.
///
/// # Parameters
///
/// * `cmd` - The flash command to send (from FLASH_CMD enum)
///
/// # Algorithm
///
/// 1. Set chip select high (deactivated) to ensure clean state
/// 2. Wait minimum required time between transactions (1μs)
/// 3. Set chip select low (activated) to begin transaction
/// 4. Write command byte to SPI bus
/// 5. Wait for transmission to complete
///
/// # Notes
///
/// * Placed in RAM to ensure operation during flash programming
/// * Never inlined to save code space at the expense of function call overhead
/// * Critical timing function for all flash operations
///
/// # Safety
///
/// This function interacts directly with hardware and depends on correct
/// timing for reliable flash operations. Caller must ensure that the SPI
/// peripheral is properly initialized before calling this function.
#[inline(never)]
#[link_section = ".ram_code"]
pub fn flash_send_cmd(cmd: FLASH_CMD) {
    mspi_high(); // Deactivate chip select
    sleep_us(1); // Ensure minimum CS high time
    mspi_low(); // Activate chip select
    mspi_write(u8::from(cmd)); // Send command byte using idiomatic conversion
    mspi_wait(); // Wait for completion
}

/// Sends a 24-bit memory address to the flash over SPI.
///
/// After sending a command that requires an address (e.g., read, write, erase),
/// this function transmits the 24-bit address in big-endian format (MSB first).
///
/// # Parameters
///
/// * `addr` - The 24-bit flash memory address to send
///
/// # Algorithm
///
/// 1. Extract and send the high byte (bits 23-16) of address
/// 2. Extract and send the middle byte (bits 15-8) of address
/// 3. Extract and send the low byte (bits 7-0) of address
///
/// # Notes
///
/// * Placed in RAM to ensure reliable operation during flash programming
/// * Assumes chip select is already active from previous command
/// * Flash protocol requires each byte to complete before sending the next
/// * Supports 16MB address space (24-bit addressing)
#[inline(never)]
#[link_section = ".ram_code"]
pub fn flash_send_addr(addr: u32) {
    // Extract address bytes in big-endian order (MSB first)
    let addr_bytes = [
        ((addr >> 16) & 0xff) as u8, // High byte (bits 23-16)
        ((addr >> 8) & 0xff) as u8,  // Middle byte (bits 15-8)
        (addr & 0xff) as u8,         // Low byte (bits 7-0)
    ];

    // Send each byte and wait for transmission to complete
    for &byte in addr_bytes.iter() {
        mspi_write(byte);
        mspi_wait();
    }
}

/// Reads data from flash memory or reads the status of flash.
///
/// Executes a complete SPI flash read sequence, handling all necessary protocol
/// requirements including command, address, dummy bytes, and data transfer.
/// This function must execute from RAM to ensure proper operation when reading
/// instructions from flash.
///
/// # Parameters
///
/// * `addr` - Starting address in flash memory to read from
/// * `len` - Number of bytes to read
/// * `buf` - Pointer to buffer where read data will be stored
///
/// # Algorithm
///
/// 1. Disable interrupts to prevent timing issues during SPI communication
/// 2. Send the read command
/// 3. Send 24-bit address if addr_en is set
/// 4. Send required dummy bytes based on flash specification
/// 5. Issue an additional clock cycle
/// 6. Switch SPI to auto mode for efficient reading
/// 7. Read requested number of bytes into buffer
/// 8. Deactivate chip select
/// 9. Restore interrupts
///
/// # Notes
///
/// * Must run from RAM (link_section = ".ram_code")
/// * Uses unsafe code to write to the data buffer via raw pointer
/// * Handles interrupts properly to ensure atomic SPI operations
/// * Auto mode increases read efficiency for bulk data transfers
///
/// # Safety
///
/// This function is unsafe because:
/// * It writes to memory through a raw pointer
/// * Caller must ensure `buf` points to valid memory of at least `len` bytes size
/// * Requires proper initialization of the SPI peripheral
/// * Must not be interrupted during operation
#[inline(never)]
#[cfg_attr(test, mry::mry)]
#[link_section = ".ram_code"]
pub fn flash_read_page(addr: u32, len: u32, buf: *mut u8) {
    #[cfg(test)]
    if FakeFlash::mode() == fake_flash::FakeFlashMode::InMemory {
        // Pointer safety: never build a slice from a null pointer or a zero length.
        if len == 0 || buf.is_null() {
            return;
        }
        // SAFETY: (2) raw-pointer buffer ABI. `len != 0` and `buf` is non-null, so the
        // caller contract gives a `len`-byte writable region; the slice never escapes.
        FakeFlash::read_slice(addr, unsafe {
            core::slice::from_raw_parts_mut(buf, len as usize)
        });
        return;
    }

    // Use critical section for atomic SPI operation
    critical_section::with(|_| {
        // Start transaction by sending read command
        flash_send_cmd(FLASH_CMD::READ_CMD);

        // Send address (always required for flash_read_page)
        flash_send_addr(addr);

        // Issue additional clock cycle to prepare for data reading
        mspi_write(0x00);
        mspi_wait();

        // Set SPI controller to auto mode (0x0a) for efficient reading
        mspi_ctrl_write(0x0a);
        mspi_wait();

        // Read requested data bytes into the provided buffer using a safer pattern
        // Still unsafe due to raw pointer usage, but more contained and idiomatic
        // SAFETY: (2) raw-pointer buffer ABI. The caller contract requires `buf` to be
        // valid for writes of `len` bytes; the slice is used only by the loop below and
        // never escapes this critical section.
        unsafe {
            // Create a slice to the uninitialized buffer for safer indexing
            let buffer = core::slice::from_raw_parts_mut(buf, len as usize);

            // Use iterator instead of raw pointer arithmetic
            for i in 0..len as usize {
                buffer[i] = mspi_get();
                mspi_wait();
            }
        }

        // End SPI transaction
        mspi_high();
    });
}

/// Writes data or status to flash memory.
///
/// Handles the complete SPI flash write sequence including the write enable command,
/// main command, address, and data bytes. This function must run from
/// RAM to ensure proper operation during flash programming.
///
/// # Parameters
///
/// * `cmd` - The write command to use (e.g., WRITE_CMD, WRITE_STATUS_CMD_LOWBYTE)
/// * `addr` - Starting address in flash for write operation
/// * `data` - Pointer to data buffer containing bytes to write
/// * `data_len` - Number of bytes to write
///
/// # Algorithm
///
/// 1. Disable interrupts to prevent timing issues during SPI communication
/// 2. Send WRITE_ENABLE command (required before any write operation)
/// 3. Send the main write command
/// 4. Send 24-bit address
/// 5. Send all data bytes from the provided buffer
/// 6. Deactivate chip select to start the internal write operation
/// 7. Wait for the write operation to complete
/// 8. Restore interrupts
///
/// # Notes
///
/// * Must run from RAM (link_section = ".ram_code")
/// * IMPORTANT: Data buffer must NOT reside in flash memory since flash will be unavailable during write
/// * All flash write operations require the WRITE_ENABLE command first
/// * Blocks until write operation is complete (flash_wait_done)
/// * Maximum data_len is typically limited by page size (256 bytes)
/// * Uses safe code with slices when possible to access the data buffer
#[inline(never)]
#[link_section = ".ram_code"]
fn flash_mspi_write_ram(cmd: FLASH_CMD, addr: u32, data: *const u8, data_len: u32) {
    // Use critical section for atomic SPI operation
    critical_section::with(|_| {
        // Send write enable command (required before any write operation)
        flash_send_cmd(FLASH_CMD::WRITE_ENABLE_CMD);

        // Send the main write command
        flash_send_cmd(cmd);

        // Send address
        flash_send_addr(addr);

        // Send all data bytes from buffer - use a safer pattern if data is present
        if !data.is_null() && data_len > 0 {
            // Create a temporary slice from the raw pointer for safer access
            // SAFETY: (2) raw-pointer buffer ABI. The `!data.is_null() && data_len > 0`
            // guard makes `data` valid for reads of `data_len` bytes, per the documented
            // caller contract of `flash_write_page`.
            let data_slice = unsafe { core::slice::from_raw_parts(data, data_len as usize) };

            // Use iterator for more idiomatic access
            for &byte in data_slice {
                mspi_write(byte);
                mspi_wait();
            }
        }

        // End SPI transaction - initiates the internal write operation
        mspi_high();

        // Wait for write operation to complete with timeout protection
        // Initial delay to allow flash operation to start
        sleep_us(100);

        // Send command to read flash status register
        flash_send_cmd(FLASH_CMD::READ_STATUS_CMD_LOWBYTE);

        // Poll busy flag with timeout protection using a more idiomatic approach
        const BUSY_BIT: u8 = 0x01;
        for _ in 0..FLASH_WAIT_ITERATIONS {
            // Check if status bit is clear (not busy)
            if mspi_read() & BUSY_BIT == 0 {
                break; // Exit loop when flash is no longer busy
            }

            // No explicit delay here - reading takes time and prevents tight loop
        }

        // Deactivate SPI chip select regardless of success/failure
        mspi_high(); // Use a more structured approach to waiting for completion
    });
}

/// Reads data from flash memory into a buffer.
///
/// High-level function for reading data from flash memory. This function provides a
/// simplified interface over the lower-level flash_mspi_read_ram function.
///
/// # Parameters
///
/// * `addr` - Starting flash memory address to read from (24-bit address)
/// * `len` - Number of bytes to read
/// * `buf` - Pointer to buffer where read data will be stored
///
/// # Algorithm
///
/// This function calls flash_mspi_read_ram with the following parameters:
/// - Uses standard READ_CMD (0x03)
/// - Sets addr_en=1 to enable address transmission
/// - Sets dummy_cnt=0 as standard read has no dummy bytes
///
/// # Notes
///
/// * Marked as inline_always for performance in critical code paths
/// * Has special mry attribute for testing to allow mocking in unit tests
/// * No practical limit on read size (can span multiple pages)
/// * Address alignment is not required
/// * IMPORTANT: Power supply voltage must be sufficient for reliable flash operation
/// * Low power conditions may cause errors in flash operations
///
/// # Safety Considerations
///
/// Flash operations may fail under low voltage conditions which could corrupt data.
/// Ensure proper power supply voltage before calling this function.

/// Writes data from a buffer to flash memory with cross-page support.
///
/// This function handles writing data to flash memory, automatically managing page boundaries.
/// Most flash memory devices can only write within a single page (256 bytes) in one operation,
/// so this function splits larger writes into multiple page-aligned operations.
///
/// # Parameters
///
/// * `addr` - Starting flash address to write to (24-bit address)
/// * `len` - Number of bytes to write
/// * `buf` - Pointer to buffer containing data to be written
///
/// # Algorithm
///
/// 1. Calculate remaining bytes in the current page
/// 2. For each chunk of data to be written:
///    a. Determine write size: either remaining page size or remaining data length
///    b. Perform page write using flash_mspi_write_ram
///    c. Update address, buffer pointer, and remaining length
///    d. Reset page size to full PAGE_SIZE for subsequent pages
/// 3. Continue until all data is written
///
/// # Notes
///
/// * Supports cross-page writing for data larger than PAGE_SIZE (256 bytes)
/// * Handles unaligned start addresses by calculating correct first page size
/// * IMPORTANT: Source buffer must NOT reside in flash memory
/// * Power supply voltage must be sufficient for reliable flash operation
///
/// # Safety Considerations
///
/// Flash operations may fail under low voltage conditions which could corrupt data.
/// Ensure proper power supply voltage before calling this function.
///
/// This function safely handles raw pointers by ensuring proper bounds checking
/// and memory access within the constraints of flash page boundaries.
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn flash_write_page(addr: u32, len: u32, buf: *const u8) {
    // Early return if there's nothing to write
    if len == 0 || buf.is_null() {
        return;
    }

    #[cfg(test)]
    if FakeFlash::mode() == fake_flash::FakeFlashMode::InMemory {
        // SAFETY: (2) raw-pointer buffer ABI. The guard above proves `buf` is valid for
        // reads of `len` bytes; the slice never escapes this function.
        FakeFlash::write_slice(addr, unsafe {
            core::slice::from_raw_parts(buf, len as usize)
        });
        return;
    }

    // Create a safe slice from the raw input buffer
    // SAFETY: (2) raw-pointer buffer ABI. `len == 0` and a null `buf` return early
    // above, so `buf` is valid for reads of `len` bytes for the whole loop and the
    // slice never escapes this function.
    let data = unsafe { core::slice::from_raw_parts(buf, len as usize) };

    // Track current position in data and address
    let mut bytes_written: usize = 0;
    let mut current_addr = addr;
    let mut bytes_remaining = len;

    // Calculate remaining bytes in current page by finding offset within page
    // and subtracting from PAGE_SIZE
    let mut current_page_remaining = PAGE_SIZE - (addr & (PAGE_SIZE - 1));

    // Continue until all data is written
    while bytes_remaining > 0 {
        // Determine write size: either remaining page size or remaining data
        let write_size = core::cmp::min(bytes_remaining, current_page_remaining);

        // Get pointer to current position in the buffer
        // SAFETY: (2) raw-pointer buffer ABI. `bytes_written` grows only by
        // `write_size`, and the loop stops once it reaches the `len` bytes proven valid
        // at entry, so the offset stays inside the allocation.
        let current_data_ptr = unsafe { buf.add(bytes_written) };

        // Write current chunk to flash
        flash_mspi_write_ram(
            FLASH_CMD::WRITE_CMD,
            current_addr,
            current_data_ptr,
            write_size,
        );

        // Update tracking variables
        bytes_written += write_size as usize;
        current_addr += write_size;
        bytes_remaining -= write_size;

        // For subsequent pages, use full page size
        current_page_remaining = PAGE_SIZE;
    }
}

/// Erases a flash memory sector (typically 4KB).
///
/// Flash memory can only be erased in blocks (sectors). This function erases an entire
/// sector, setting all bytes to 0xFF. Any write operation to flash memory requires the
/// target bytes to be in an erased state (0xFF) first.
///
/// # Parameters
///
/// * `addr` - Any address within the sector to be erased (sector aligned automatically)
///
/// # Algorithm
///
/// 1. Clear the watchdog timer to prevent resets during long erase operation
/// 2. Call flash_mspi_write_ram with SECT_ERASE_CMD (0x20)
///    - Passes the sector address to erase
///    - No data is needed for erase command (null pointer, 0 length)
///
/// # Notes
///
/// * Sector erase operations can take significant time (~100ms typical)
/// * The watchdog is cleared before erasing to prevent timeouts
/// * Flash must be erased before writing new data (bytes can only go from 1→0, not 0→1)
/// * A sector is typically 4KB in size and will be aligned by the flash device
/// * All data in the sector will be lost, regardless of the specific address provided
/// * IMPORTANT: Power supply voltage must be sufficient for reliable erase operation
///
/// # Safety Considerations
///
/// Flash operations may fail under low voltage conditions which could corrupt data.
/// Ensure proper power supply voltage before calling this function.
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn flash_erase_sector(addr: u32) {
    // Clear watchdog timer to prevent resets during long erase operation
    wd_clear();

    #[cfg(test)]
    if FakeFlash::mode() == fake_flash::FakeFlashMode::InMemory {
        FakeFlash::erase_sector(addr);
        return;
    }

    // Send sector erase command with the target address
    // No data is needed for erase operation (null pointer, 0 length)
    flash_mspi_write_ram(FLASH_CMD::SECT_ERASE_CMD, addr, null_mut(), 0);
}

/// In-process 512 KiB NOR flash model that the public `flash_*` entry points
/// drive directly under `cfg(test)` when [`FakeFlashMode::InMemory`] is selected.
///
/// The default mode is [`FakeFlashMode::SpiRegisters`], so the existing
/// SPI-mocking tests keep exercising the real `mspi_*` transactions untouched;
/// `InMemory` is opt-in per test. All state lives behind one `Mutex`, and every
/// lock recovers from poisoning, so a single panicking test cannot break the
/// rest of a `--test-threads=1` run.
#[cfg(test)]
pub mod fake_flash {
    use std::collections::BTreeMap;
    use std::sync::{Mutex, MutexGuard};
    use std::vec::Vec;

    /// Size of the modelled NOR array (512 KiB).
    pub const FLASH_SIZE_BYTES: usize = 512 * 1024;
    /// Size of one erasable sector (4 KiB).
    pub const SECTOR_SIZE_BYTES: usize = 4096;

    /// Where the flash entry points send their work.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum FakeFlashMode {
        /// Drives real `mspi_*` transactions (the default, so existing SPI-mocking tests run untouched).
        SpiRegisters,
        /// Executes reads/writes/erases directly against the 512 KiB in-memory NOR array.
        InMemory,
    }

    /// Handle to the process-wide fake NOR device.
    ///
    /// The device is global (one 512 KiB array per test binary), so these are
    /// associated functions grouped under a name for readable call sites.
    pub struct FakeFlash;

    struct FakeFlashState {
        bytes: Vec<u8>,
        mode: FakeFlashMode,
        erase_counts: BTreeMap<u32, u32>,
        fail_next_write: bool,
    }

    impl FakeFlashState {
        const fn new() -> Self {
            Self {
                bytes: Vec::new(),
                mode: FakeFlashMode::SpiRegisters,
                erase_counts: BTreeMap::new(),
                fail_next_write: false,
            }
        }

        /// Fill the array lazily, so a test that forgot `reset()` still works.
        fn ensure(&mut self) {
            if self.bytes.len() != FLASH_SIZE_BYTES {
                self.bytes.clear();
                self.bytes.resize(FLASH_SIZE_BYTES, 0xFF);
            }
        }
    }

    static STATE: Mutex<FakeFlashState> = Mutex::new(FakeFlashState::new());

    /// Lock the device, recovering the inner state if an earlier test panicked
    /// while holding the lock.
    fn lock() -> MutexGuard<'static, FakeFlashState> {
        STATE.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Sector base for any address inside the sector.
    fn sector_base(addr: u32) -> u32 {
        addr & !(SECTOR_SIZE_BYTES as u32 - 1)
    }

    impl FakeFlash {
        /// Restore a blank device: all `0xFF`, no erase counts, no pending write
        /// fault, and the default `SpiRegisters` mode.
        pub fn reset() {
            let mut state = lock();
            state.bytes.clear();
            state.bytes.resize(FLASH_SIZE_BYTES, 0xFF);
            state.erase_counts.clear();
            state.fail_next_write = false;
            state.mode = FakeFlashMode::SpiRegisters;
        }

        /// Select where the `flash_*` entry points send their work. Tests that do
        /// not opt in to `InMemory` keep driving the real `mspi_*` transactions.
        pub fn set_mode(mode: FakeFlashMode) {
            lock().mode = mode;
        }

        /// The mode currently selected by the last [`FakeFlash::set_mode`].
        pub fn mode() -> FakeFlashMode {
            lock().mode
        }

        /// Copy `out.len()` bytes starting at absolute flash offset `addr`.
        /// Reads past the end of the array return `0xFF` instead of panicking.
        pub fn read_slice(addr: u32, out: &mut [u8]) {
            let mut state = lock();
            state.ensure();
            let start = (addr as usize).min(FLASH_SIZE_BYTES);
            for (offset, byte) in out.iter_mut().enumerate() {
                *byte = state.bytes.get(start + offset).copied().unwrap_or(0xFF);
            }
        }

        /// Apply `data` at absolute flash offset `addr` with NOR `AND` semantics
        /// (`dst &= src`): writing can only clear bits. The range is clamped to
        /// the array instead of panicking.
        pub fn write_slice(addr: u32, data: &[u8]) {
            let mut state = lock();
            state.ensure();
            let start = addr as usize;
            if start >= FLASH_SIZE_BYTES {
                return;
            }
            let count = core::cmp::min(data.len(), FLASH_SIZE_BYTES - start);
            let corrupt = state.fail_next_write;
            // The injected fault is one-shot, like a single bad page program.
            state.fail_next_write = false;
            for (offset, &src) in data[..count].iter().enumerate() {
                state.bytes[start + offset] &= src;
            }
            if corrupt && count > 0 {
                state.bytes[start] ^= 0x01;
            }
        }

        /// Fill the 4 KiB sector containing `addr` with `0xFF` and count the erase.
        pub fn erase_sector(addr: u32) {
            let mut state = lock();
            state.ensure();
            let base = sector_base(addr) as usize;
            let end = core::cmp::min(base + SECTOR_SIZE_BYTES, FLASH_SIZE_BYTES);
            if base >= end {
                return;
            }
            state.bytes[base..end].fill(0xFF);
            *state.erase_counts.entry(base as u32).or_insert(0) += 1;
        }

        /// Number of erases performed on the sector containing `addr`.
        pub fn erase_count(addr: u32) -> u32 {
            *lock().erase_counts.get(&sector_base(addr)).unwrap_or(&0)
        }

        /// Corrupt byte 0 of the next [`FakeFlash::write_slice`] so a read-back
        /// verification fails; the corruption persists in the array.
        pub fn fail_verify_next_write() {
            lock().fail_next_write = true;
        }
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use std::vec::Vec;

    use super::fake_flash::{FakeFlash, FakeFlashMode};

    /// Restores the SPI-backed default mode even if the owning test panics, so a
    /// failure cannot strand later tests in `InMemory` mode.
    struct RestoreSpi;

    impl Drop for RestoreSpi {
        fn drop(&mut self) {
            FakeFlash::set_mode(FakeFlashMode::SpiRegisters);
        }
    }

    // Import mocked versions of the functions from their original modules
    use crate::sdk::drivers::spi::{
        mock_mspi_ctrl_write, mock_mspi_get, mock_mspi_high, mock_mspi_low, mock_mspi_read,
        mock_mspi_wait, mock_mspi_write,
    };
    use crate::sdk::mcu::clock::mock_sleep_us;
    use crate::sdk::mcu::watchdog::mock_wd_clear;

    // This is required because we're working with raw pointers in our testing
    // It helps to ensure safe operations when mocking hardware interactions
    fn vec_to_raw_mut(v: &mut Vec<u8>) -> *mut u8 {
        v.as_mut_ptr()
    }

    fn vec_to_raw(v: &Vec<u8>) -> *const u8 {
        v.as_ptr()
    }

    /// Tests the flash_read_page function.
    ///
    /// This test verifies that the flash_read_page function correctly:
    /// - Sends the proper READ_CMD command
    /// - Transmits the address correctly
    /// - Sets the SPI controller to auto mode for efficient reading
    /// - Reads data into the provided buffer
    /// - Handles chip select and interrupt signals properly
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for SPI and interrupt functions
    /// 2. Configure mock_mspi_get to return test pattern data
    /// 3. Call flash_read_page with test address and buffer
    /// 4. Verify buffer contains expected data after read
    /// 5. Verify proper command sequence, chip select signals and timing
    /// 6. Confirm SPI controller was set to auto mode
    /// 7. Verify interrupt handling was performed correctly
    ///
    /// # Notes
    ///
    /// * Tests the standard READ_CMD (0x03) flash read command
    /// * Covers the complete read sequence including interrupt handling
    #[test]
    #[mry::lock(
        mspi_read,
        mspi_write,
        mspi_get,
        mspi_ctrl_write,
        mspi_high,
        mspi_low,
        mspi_wait,
        sleep_us
    )]
    fn test_flash_read_page() {
        // Setup mock responses for SPI functions

        // Simulate the busy bit being clear (not busy)
        mock_mspi_read().returns(0);
        mock_mspi_write(0x03).returns(()); // READ_CMD
        mock_mspi_write(0x10).returns(()); // addr[23:16]
        mock_mspi_write(0x00).returns(()); // addr[15:8] addr[7:0] dummy write for reading

        mock_sleep_us(1).returns(());

        // Configure chip select behavior
        mock_mspi_high().returns(());
        mock_mspi_low().returns(());

        // Configure wait behavior
        mock_mspi_wait().returns(());

        // Configure auto mode write
        mock_mspi_ctrl_write(0x0a).returns(());

        // Setup sequential byte responses for flash read data
        let expected_data = [0xAA, 0xBB, 0xCC, 0xDD];

        // Use a closure to return different values on each call
        let mut get_calls = 0;
        mock_mspi_get().returns_with(move || {
            let val = expected_data[get_calls];
            get_calls += 1;
            val
        });

        // Create a buffer to hold read data
        let mut buffer = vec![0; 4];
        let buf_ptr = vec_to_raw_mut(&mut buffer);

        // Call the function we're testing
        flash_read_page(0x1000, 4, buf_ptr);

        // Verify expected buffer contents after read
        assert_eq!(buffer, expected_data);

        // Verify proper command sequence with exact call counts
        mock_mspi_high().assert_called(2); // Initial deactivate + final deactivate
        mock_mspi_low().assert_called(1); // Activation for command

        // Verify SPI was set to auto mode for reading
        mock_mspi_ctrl_write(0x0a).assert_called(1);

        // Verify we waited for each operation with exact count
        mock_mspi_wait().assert_called(10);

        // Verify all expected data bytes were read
        mock_mspi_get().assert_called(4);
    }

    /// Tests flash_write_page function with small data that fits within one page.
    ///
    /// This test verifies that the flash_write_page function correctly:
    /// - Handles writes that do not cross page boundaries
    /// - Properly sequences SPI signals during write operations
    /// - Correctly handles interrupt state during flash writes
    /// - Successfully transmits all data bytes
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for SPI and interrupt functions
    /// 2. Create a small test data buffer (4 bytes)
    /// 3. Call flash_write_page with an aligned address
    /// 4. Verify proper command sequence and chip select signals
    /// 5. Confirm interrupt disable/restore was performed
    /// 6. Verify each data byte was written correctly
    ///
    /// # Notes
    ///
    /// * This test focuses on the simple case where data fits within page boundaries
    /// * Page size is 256 bytes for this device
    #[test]
    #[mry::lock(mspi_read, mspi_write, mspi_high, mspi_low, mspi_wait, sleep_us)]
    fn test_flash_write_page_small() {
        // Setup mock responses

        // Simulate the busy bit being clear after a write
        mock_mspi_read().returns(0);

        // Setup other mocks needed for the write operation
        mock_mspi_high().returns(());
        mock_mspi_low().returns(());
        mock_mspi_wait().returns(());
        mock_sleep_us(1).returns(());
        mock_sleep_us(100).returns(());

        // Create test data to write
        let data = vec![0x11, 0x22, 0x33, 0x44];

        // Setup expectations for each data byte write
        mock_mspi_write(0x06).returns(()); // WRITE_ENABLE_CMD
        mock_mspi_write(0x02).returns(()); // WRITE_CMD
        mock_mspi_write(0x10).returns(()); // addr[23:16]
        mock_mspi_write(0x00).returns(()); // addr[15:8] addr[7:0]
        mock_mspi_write(0x05).returns(()); // READ_STATUS_CMD_LOWBYTE

        // Setup expectations for each data byte
        for byte in &data {
            mock_mspi_write(*byte).returns(());
        }

        let data_ptr = vec_to_raw(&data);

        // Call function with address at start of a page boundary
        flash_write_page(0x1000, 4, data_ptr);

        // Verify proper command sequence for write with exact counts
        mock_mspi_high().assert_called(5); // Initial + after WRITE_ENABLE + after WRITE + final
        mock_mspi_low().assert_called(3); // For WRITE_ENABLE_CMD and WRITE_CMD

        // Verify commands were sent
        mock_mspi_write(0x06).assert_called(1); // WRITE_ENABLE_CMD
        mock_mspi_write(0x02).assert_called(1); // WRITE_CMD

        // Verify address was sent
        mock_mspi_write(0x10).assert_called(1); // addr[23:16]
        mock_mspi_write(0x00).assert_called(2); // addr[15:8] and addr[7:0]

        mock_mspi_write(0x05).assert_called(1); // READ_STATUS_CMD_LOWBYTE

        // Verify data was written
        for byte in data {
            mock_mspi_write(byte).assert_called(1);
        }

        // Verify wait was called appropriate number of times
        mock_mspi_wait().assert_called(10); // 1 for each write + additional waits
    }

    /// Tests flash_write_page function with data that crosses page boundaries.
    ///
    /// This test verifies that the flash_write_page function correctly:
    /// - Handles writes that span multiple flash pages
    /// - Correctly splits the operation into separate page writes
    /// - Advances the buffer pointer correctly between page operations
    /// - Properly manages interrupt state for each page operation
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for SPI and interrupt functions
    /// 2. Create test data large enough to span multiple pages (300 bytes)
    /// 3. Start writing at address 0x10FA (6 bytes before a page boundary)
    /// 4. Call flash_write_page to initiate multi-page write
    /// 5. Verify write function was called multiple times (once per page)
    /// 6. Confirm interrupt state was managed for each page operation
    ///
    /// # Notes
    ///
    /// * Flash can only write within a single page (256 bytes) in one operation
    /// * The test uses a special pattern in the data to help verify correct buffer position
    /// * This test focuses specifically on the page-spanning behavior
    #[test]
    #[mry::lock(mspi_read, mspi_write, mspi_high, mspi_low, mspi_wait, sleep_us)]
    fn test_flash_write_page_cross_boundary() {
        // Setup mock responses
        mock_mspi_read().returns(0); // Not busy

        // Setup other mocks needed for the write operation
        mock_mspi_high().returns(());
        mock_mspi_low().returns(());
        mock_mspi_wait().returns(());
        mock_sleep_us(1).returns(());
        mock_sleep_us(100).returns(());

        // Create test data to write (spans two pages)
        // Use a special pattern to verify correct buffer position is used
        let mut data = Vec::new();
        for i in 0..300 {
            data.push((i & 0xFF) as u8);
        }

        // Setup expectations for command bytes
        mock_mspi_write(0x06).returns(()); // WRITE_ENABLE_CMD
        mock_mspi_write(0x02).returns(()); // WRITE_CMD

        // Setup expectations for address bytes for first page
        mock_mspi_write(0x10).returns(()); // addr[23:16]
        mock_mspi_write(0x0F).returns(()); // addr[15:8]
        mock_mspi_write(0xA).returns(()); // addr[7:0] = 0xFA

        // Setup expectations for address bytes for second page
        mock_mspi_write(0x11).returns(()); // addr[23:16]
        mock_mspi_write(0x00).returns(()); // addr[15:8]
        mock_mspi_write(0x00).returns(()); // addr[7:0]

        // Setup expectations for data bytes (use wildcard since there are too many)
        // We'll use the Any matcher for the data bytes
        mock_mspi_write(mry::Any).returns(());

        let data_ptr = vec_to_raw(&data);

        // Start near the end of a page to force crossing boundary
        // PAGE_SIZE is 256, so we'll start at offset 250 (0xFA)
        let start_addr = 0x10FA; // 6 bytes before page boundary

        // Call function to write across page boundary
        flash_write_page(start_addr, 300, data_ptr);

        // Verify high/low signals for chip select with exact counts
        mock_mspi_high().assert_called(15); // 4 per page (2 pages)
        mock_mspi_low().assert_called(9); // 2 per page (2 pages)

        // Verify command bytes were sent
        mock_mspi_write(0x06).assert_called(5); // WRITE_ENABLE_CMD called twice (once per page)
        mock_mspi_write(0x02).assert_called(5); // WRITE_CMD called twice (once per page)

        // Verify correct number of wait calls
        // 6 + 300 bytes = 306 bytes of data writing -> 306 wait calls
        // Plus additional waits for commands and addresses
        mock_mspi_wait().assert_called(318); // 306 data bytes + commands + addresses
    }

    /// Tests the flash_erase_sector function.
    ///
    /// This test verifies that the sector erase operation correctly:
    /// - Clears the watchdog timer before erasing
    /// - Properly sequences chip select signals
    /// - Handles interrupts correctly during flash operations
    ///
    /// # Algorithm
    ///
    /// 1. Setup mocked responses for low-level functions
    /// 2. Call flash_erase_sector with a test address
    /// 3. Verify watchdog was cleared
    /// 4. Verify correct chip select signal sequence
    /// 5. Verify interrupt disable/restore was performed
    ///
    /// # Notes
    ///
    /// * The function should ensure the watchdog is cleared before long erase operations
    /// * SPI protocol requires specific chip select sequence
    #[test]
    #[mry::lock(
        mspi_read, mspi_write, mspi_high, mspi_low, mspi_wait, sleep_us, wd_clear
    )]
    fn test_flash_erase_sector() {
        // Setup mock responses
        mock_mspi_read().returns(0); // Not busy

        // Setup other required mocks
        mock_mspi_high().returns(());
        mock_mspi_low().returns(());
        mock_mspi_wait().returns(());
        mock_sleep_us(1).returns(());
        mock_sleep_us(100).returns(());
        mock_wd_clear().returns(());

        // Setup command expectations
        mock_mspi_write(0x06).returns(()); // WRITE_ENABLE_CMD
        mock_mspi_write(0x20).returns(()); // SECT_ERASE_CMD addr[23:16]
        mock_mspi_write(0x05).returns(()); // READ_STATUS_CMD_LOWBYTE

        // Setup address bytes expectations
        mock_mspi_write(0x00).returns(()); // addr[15:8] addr[7:0]

        // Call function to erase sector
        flash_erase_sector(0x2000);

        // Verify watchdog was cleared
        mock_wd_clear().assert_called(1);

        // Verify proper command sequence with exact call counts
        mock_mspi_high().assert_called(5); // Initial + after WRITE_ENABLE + after ERASE + final + flash wait done
        mock_mspi_low().assert_called(3); // For WRITE_ENABLE_CMD and SECT_ERASE_CMD + flash wait done

        // Verify commands were sent
        mock_mspi_write(0x06).assert_called(1); // WRITE_ENABLE_CMD
        mock_mspi_write(0x20).assert_called(2); // SECT_ERASE_CMD and addr[23:16]
        mock_mspi_write(0x00).assert_called(2); // addr[15:8] and addr[7:0]
        mock_mspi_write(0x05).assert_called(1); // READ_STATUS_CMD_LOWBYTE

        // Verify correct number of wait calls
        mock_mspi_wait().assert_called(6); // Commands + addresses + additional waits
    }

    /// Tests the flash_send_addr function.
    ///
    /// This test verifies that the flash_send_addr function correctly transmits
    /// a 24-bit address in big-endian format (MSB first) over SPI.
    ///
    /// # Algorithm
    ///
    /// 1. Setup mocked responses for SPI write operations
    /// 2. Call flash_send_addr with test address 0x123456
    /// 3. Verify each byte was sent in correct order (0x12, 0x34, 0x56)
    /// 4. Verify proper timing with mspi_wait() after each byte
    ///
    /// # Notes
    ///
    /// * Flash protocol requires MSB-first transmission of addresses
    /// * Each byte transmission must complete before sending the next byte
    #[test]
    #[mry::lock(mspi_write, mspi_wait)]
    fn test_flash_send_addr() {
        // Setup expected behavior for mocked functions with proper return values
        mock_mspi_write(0x12).returns(());
        mock_mspi_write(0x34).returns(());
        mock_mspi_write(0x56).returns(());
        mock_mspi_wait().returns(());

        // Test address: 0x123456 - should send bytes 0x12, 0x34, 0x56 in that order
        flash_send_addr(0x123456);

        // Verify each byte was sent in correct order (MSB first)
        mock_mspi_write(0x12).assert_called(1);
        mock_mspi_wait().assert_called(3); // Total of 3 waits (one after each byte)
        mock_mspi_write(0x34).assert_called(1);
        mock_mspi_write(0x56).assert_called(1);
    }

    /// Integration test simulating a complete flash erase-write-read cycle.
    ///
    /// This test verifies the full operational sequence of flash memory operations:
    /// 1. Reading erased flash (all 0xFF)
    /// 2. Erasing a sector
    /// 3. Writing data to flash
    /// 4. Reading back the data to confirm it was written correctly
    ///
    /// # Algorithm
    ///
    /// 1. Read from flash address and verify it contains erased pattern (0xFF)
    /// 2. Erase the sector at the target address
    /// 3. Write test data to the same address
    /// 4. Configure mock to return the expected data on subsequent reads
    /// 5. Read back the data from the same address
    /// 6. Verify the read data matches what was written
    ///
    /// # Notes
    ///
    /// * This test mocks all low-level operations to simulate hardware interactions
    /// * The watchdog is verified to be cleared during erase operation
    /// * The test uses a specific pattern of data to ensure proper write operations
    #[test]
    #[mry::lock(
        mspi_read,
        mspi_write,
        mspi_get,
        mspi_ctrl_write,
        mspi_high,
        mspi_low,
        mspi_wait,
        sleep_us,
        wd_clear
    )]
    fn test_flash_read_write_cycle() {
        // Setup mock responses
        mock_mspi_read().returns(0); // Not busy

        // Setup other required mocks
        mock_mspi_high().returns(());
        mock_mspi_low().returns(());
        mock_mspi_wait().returns(());
        mock_mspi_ctrl_write(0x0a).returns(());
        mock_sleep_us(mry::Any).returns(());
        mock_wd_clear().returns(());

        // Setup command byte expectations
        mock_mspi_write(mry::Any).returns(());

        // ======== FIRST PHASE: Reading erased flash ========

        // Simulate reading erased flash (all 0xFF)
        let erased_pattern = 0xFF;

        // Setup a closure to return 0xFF for the first 4 reads (erased flash)
        let mut read_phases = 0;
        let write_data = vec![0xA1, 0xB2, 0xC3, 0xD4];
        let write_data_copy = write_data.clone();

        mock_mspi_get().returns_with(move || {
            if read_phases < 4 {
                // First phase: return erased pattern
                read_phases += 1;
                erased_pattern
            } else {
                // Second phase: return the written data pattern
                let index = read_phases - 4;
                read_phases += 1;
                write_data_copy[index]
            }
        });

        // Step 1: Verify sector is "erased" (all 0xFF)
        let mut read_buffer = vec![0; 4];
        flash_read_page(0x3000, 4, vec_to_raw_mut(&mut read_buffer));
        assert!(read_buffer.iter().all(|&b| b == erased_pattern));

        // Step 2: Erase sector to prepare for write
        flash_erase_sector(0x3000);
        mock_wd_clear().assert_called(1);

        // Step 3: Write data to flash
        flash_write_page(0x3000, 4, vec_to_raw(&write_data));

        // Step 4: Read back the data
        let mut verify_buffer = vec![0; 4];
        flash_read_page(0x3000, 4, vec_to_raw_mut(&mut verify_buffer));

        // Step 5: Verify the data matches what we wrote
        assert_eq!(verify_buffer, write_data);

        // Verify the mspi_get was called exactly 8 times (4 for erased read + 4 for verification)
        mock_mspi_get().assert_called(8);

        // Verify that chip select operations occurred correct number of times
        // Each flash operation starts and ends with chip select operations
        mock_mspi_high().assert_called(14); // Multiple operations
        mock_mspi_low().assert_called(8); // One per major operation

        // Verify commands were sent
        mock_mspi_write(0x03).assert_called(2); // READ_CMD called twice
        mock_mspi_write(0x06).assert_called(2); // WRITE_ENABLE_CMD called twice
    }

    /// Tests the flash busy flag timeout protection in flash_mspi_write_ram.
    ///
    /// This test verifies that the function handles the scenario where the flash
    /// busy flag never clears (mspi_read() & 0x01 is always 1), ensuring that the
    /// loop eventually exits after FLASH_WAIT_ITERATIONS attempts.
    ///
    /// # Algorithm
    ///
    /// 1. Setup mocks to simulate a flash device that never becomes ready
    /// 2. Configure mspi_read to always return 1 (busy flag always set)
    /// 3. Call flash_mspi_write_ram with test data
    /// 4. Verify the function completes without hanging
    /// 5. Verify mspi_read was called exactly FLASH_WAIT_ITERATIONS times
    #[test]
    #[mry::lock(mspi_read, mspi_write, mspi_high, mspi_low, mspi_wait, sleep_us)]
    fn test_flash_busy_flag_timeout() {
        // Setup mock responses

        // Critical: Always return 1 to simulate flash never clearing the busy flag
        // This will force the polling loop to run for the full FLASH_WAIT_ITERATIONS
        mock_mspi_read().returns(1);

        // Setup other mocks needed for the write operation
        mock_mspi_high().returns(());
        mock_mspi_low().returns(());
        mock_mspi_wait().returns(());
        mock_sleep_us(1).returns(());
        mock_sleep_us(100).returns(());

        // Setup expectations for command bytes
        mock_mspi_write(0x06).returns(()); // WRITE_ENABLE_CMD
        mock_mspi_write(0x02).returns(()); // WRITE_CMD
        mock_mspi_write(0x10).returns(()); // addr[23:16]
        mock_mspi_write(0x00).returns(()); // addr[15:8] addr[7:0]
        mock_mspi_write(0x05).returns(()); // READ_STATUS_CMD_LOWBYTE

        // Test data to write
        let data = vec![0x11, 0x22, 0x33, 0x44];
        for byte in &data {
            mock_mspi_write(*byte).returns(());
        }

        // Call the function - it should complete after FLASH_WAIT_ITERATIONS cycles
        // without hanging indefinitely
        let data_ptr = vec_to_raw(&data);
        flash_write_page(0x1000, 4, data_ptr);

        // Verify mspi_read was called exactly FLASH_WAIT_ITERATIONS times
        // This is the critical check - making sure it exits the loop after the timeout
        mock_mspi_read().assert_called(FLASH_WAIT_ITERATIONS as usize);

        // Verify high/low signals for chip select
        mock_mspi_high().assert_called(5); // Initial + after commands + final
        mock_mspi_low().assert_called(3); // For WRITE_ENABLE, WRITE_CMD, READ_STATUS

        // Verify wait calls occurred for all operations
        mock_mspi_wait().assert_called(10); // For commands + data
    }

    /// Tests the early return condition in flash_write_page function.
    ///
    /// This test verifies that the function correctly exits early when:
    /// 1. The length parameter is 0, or
    /// 2. The buffer pointer is null
    ///
    /// In either case, the function should return immediately without making
    /// any calls to the underlying flash_mspi_write_ram function.
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock expectations for functions that should NOT be called
    /// 2. Call flash_write_page with zero length but valid buffer
    /// 3. Call flash_write_page with null buffer but non-zero length
    /// 4. Verify no calls were made to any mocked functions
    ///
    /// # Notes
    ///
    /// * This test specifically targets the early return condition added for safety
    /// * Ensuring the function does nothing when given empty/invalid parameters
    /// * Prevents potential undefined behavior from null pointer operations
    #[test]
    #[mry::lock(mspi_read, mspi_write, mspi_high, mspi_low, mspi_wait, sleep_us)]
    fn test_flash_write_page_early_return() {
        // Create a valid test data buffer (won't be used with length 0)
        let data = vec![0x11, 0x22, 0x33, 0x44];
        let data_ptr = vec_to_raw(&data);

        // Case 1: Test with zero length but valid buffer
        flash_write_page(0x1000, 0, data_ptr);

        // Case 2: Test with null buffer but non-zero length
        flash_write_page(0x1000, 4, std::ptr::null());

        // Verify that no SPI or interrupt functions were called
        // This confirms the early return is working correctly

        // Check SPI control functions weren't called
        mock_mspi_high().assert_called(0);
        mock_mspi_low().assert_called(0);
        mock_mspi_wait().assert_called(0);

        // Check that no data was written
        mock_mspi_write(mry::Any).assert_called(0);

        // Check that no status was read (no flash operation occurred)
        mock_mspi_read().assert_called(0);

        // No delays should have been called either
        mock_sleep_us(mry::Any).assert_called(0);
    }

    /// Proves the real flash entry points work against the in-memory NOR model:
    /// erase reads back `0xFF`, a written pattern reads back byte-exact, and
    /// sequential writes obey NOR `AND` semantics without any SPI mocking.
    #[test]
    fn test_fake_flash_erase_write_read_round_trip() {
        FakeFlash::reset();
        FakeFlash::set_mode(FakeFlashMode::InMemory);
        let _restore = RestoreSpi;

        const ADDR: u32 = 0x1_0000;

        // Erased silicon reads 0xFF.
        flash_erase_sector(ADDR);
        let mut erased = [0u8; 8];
        flash_read_page(ADDR, 8, erased.as_mut_ptr());
        assert_eq!(erased, [0xFF; 8]);

        // A pattern written through the public API reads back byte-exact.
        let pattern = [0xA5u8, 0x5A, 0x0F, 0xF0];
        flash_write_page(ADDR, pattern.len() as u32, pattern.as_ptr());
        let mut readback = [0u8; 4];
        flash_read_page(ADDR, 4, readback.as_mut_ptr());
        assert_eq!(readback, pattern);

        // NOR writes can only clear bits: 0xFF & 0x0F & 0xF0 == 0x00.
        flash_erase_sector(ADDR);
        let ones_nibble = [0x0Fu8];
        flash_write_page(ADDR, 1, ones_nibble.as_ptr());
        let high_nibble = [0xF0u8];
        flash_write_page(ADDR, 1, high_nibble.as_ptr());
        let mut merged = [0xFFu8; 1];
        flash_read_page(ADDR, 1, merged.as_mut_ptr());
        assert_eq!(merged[0], 0x00);

        // Two erases of the same sector were performed above.
        assert_eq!(FakeFlash::erase_count(ADDR), 2);
    }

    /// Direct unit coverage of the model's NOR `AND` write semantics.
    #[test]
    fn test_fake_flash_nor_and_semantics() {
        FakeFlash::reset();
        FakeFlash::set_mode(FakeFlashMode::InMemory);
        let _restore = RestoreSpi;

        const ADDR: u32 = 0x4_0000;
        let mut byte = [0u8; 1];

        FakeFlash::write_slice(ADDR, &[0xFF]);
        FakeFlash::write_slice(ADDR, &[0x0F]);
        FakeFlash::write_slice(ADDR, &[0xF0]);
        FakeFlash::read_slice(ADDR, &mut byte);
        assert_eq!(byte[0], 0x00);

        // An already-cleared bit cannot be set again by a later write.
        const CLEARED: u32 = ADDR + 1;
        FakeFlash::write_slice(CLEARED, &[0x00]);
        FakeFlash::write_slice(CLEARED, &[0xFF]);
        FakeFlash::read_slice(CLEARED, &mut byte);
        assert_eq!(byte[0], 0x00);
    }

    /// The injected write fault corrupts exactly one write and persists in RAM.
    #[test]
    fn test_fake_flash_fail_verify_next_write() {
        FakeFlash::reset();
        FakeFlash::set_mode(FakeFlashMode::InMemory);
        let _restore = RestoreSpi;

        const ADDR: u32 = 0x2_0000;
        let mut byte = [0u8; 1];

        FakeFlash::fail_verify_next_write();
        FakeFlash::write_slice(ADDR, &[0xFF]);
        FakeFlash::read_slice(ADDR, &mut byte);
        assert_eq!(byte[0], 0xFE);

        // The fault is one-shot: a later write is not corrupted. A fresh byte is
        // used because a NOR write cannot turn the corrupted 0xFE back into 0xFF.
        FakeFlash::write_slice(ADDR + 1, &[0xFF]);
        FakeFlash::read_slice(ADDR + 1, &mut byte);
        assert_eq!(byte[0], 0xFF);
    }

    /// The raw-pointer guard in `flash_read_page` rejects a zero length and a
    /// null buffer in `InMemory` mode without touching memory.
    #[test]
    fn test_fake_flash_zero_len_and_null_guards() {
        FakeFlash::reset();
        FakeFlash::set_mode(FakeFlashMode::InMemory);
        let _restore = RestoreSpi;

        const ADDR: u32 = 0x3_0000;
        FakeFlash::write_slice(ADDR, &[0xA5]);

        let mut buf = [0x11u8; 4];
        flash_read_page(ADDR, 0, buf.as_mut_ptr());
        assert_eq!(buf, [0x11; 4]);

        flash_read_page(ADDR, 4, std::ptr::null_mut());

        let mut byte = [0u8; 1];
        FakeFlash::read_slice(ADDR, &mut byte);
        assert_eq!(byte[0], 0xA5);
        assert_eq!(FakeFlash::erase_count(ADDR), 0);
    }
}
