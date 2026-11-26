use super::spi::{
    mspi_ctrl_write, mspi_get, mspi_high, mspi_low, mspi_read, mspi_wait, mspi_write,
};
use crate::mcu::clock::sleep_us;
use crate::mcu::watchdog::wd_clear;
use core::ptr::null_mut;

#[cfg(not(target_os = "none"))]
use self::fake_flash::FakeFlash;

pub const PAGE_SIZE: u32 = 256;
pub const PAGE_SIZE_OTP: u32 = 256;
pub const FLASH_LOCK_EN: u32 = 0;

/// Maximum number of iterations for the flash busy wait loop.
/// Set to a lower value on the host so a run cannot spin for long.
#[cfg(not(target_os = "none"))]
pub const FLASH_WAIT_ITERATIONS: u32 = 1000;
#[cfg(target_os = "none")]
pub const FLASH_WAIT_ITERATIONS: u32 = 10000000;

/*
 * @brief     flash command definition
 *
 * The names and codes are the datasheet/TC32 SDK command table, so they keep
 * the datasheet spelling instead of Rust's camel case. Only a subset is used by
 * this firmware; the rest document the chip's command surface.
 */
#[allow(non_camel_case_types, dead_code)]
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

/// Sends a command byte to the flash memory over SPI.
///
/// This function handles the SPI protocol for sending a command to the flash chip.
/// It ensures proper timing and signal transitions for reliable communication.
///
/// # Parameters
///
/// * `cmd` - The flash command to send (from FLASH_CMD enum)
///
/// # Returns
///
/// * `true` if the command byte was fully transmitted
/// * `false` if the SPI busy flag never cleared (see [`mspi_wait`])
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
fn flash_send_cmd(cmd: FLASH_CMD) -> bool {
    mspi_high(); // Deactivate chip select
    sleep_us(1); // Ensure minimum CS high time
    mspi_low(); // Activate chip select
    mspi_write(u8::from(cmd)); // Send command byte using idiomatic conversion
    mspi_wait() // Report whether the transmission completed
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
/// # Returns
///
/// * `true` if all three address bytes were fully transmitted
/// * `false` if the SPI busy flag never cleared (see [`mspi_wait`])
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
fn flash_send_addr(addr: u32) -> bool {
    // Extract address bytes in big-endian order (MSB first)
    let addr_bytes = [
        ((addr >> 16) & 0xff) as u8, // High byte (bits 23-16)
        ((addr >> 8) & 0xff) as u8,  // Middle byte (bits 15-8)
        (addr & 0xff) as u8,         // Low byte (bits 7-0)
    ];

    // Send each byte and wait for transmission to complete
    for &byte in addr_bytes.iter() {
        mspi_write(byte);
        if !mspi_wait() {
            return false;
        }
    }
    true
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
#[link_section = ".ram_code"]
// The documented caller contract is a raw-pointer buffer ABI (`buf` valid for
// `len` writable bytes), kept as a safe fn to preserve the firmware API.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub fn flash_read_page(addr: u32, len: u32, buf: *mut u8) {
    #[cfg(not(target_os = "none"))]
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
        // A stuck bus aborts the transfer: every failure path below releases
        // chip select before returning, so the flash part cannot stay selected.
        if !flash_send_cmd(FLASH_CMD::READ_CMD) {
            mspi_high();
            return;
        }

        // Send address (always required for flash_read_page)
        if !flash_send_addr(addr) {
            mspi_high();
            return;
        }

        // Issue additional clock cycle to prepare for data reading
        mspi_write(0x00);
        if !mspi_wait() {
            mspi_high();
            return;
        }

        // Set SPI controller to auto mode (0x0a) for efficient reading
        mspi_ctrl_write(0x0a);
        if !mspi_wait() {
            mspi_high();
            return;
        }

        // Read requested data bytes into the provided buffer using a safer pattern
        // Still unsafe due to raw pointer usage, but more contained and idiomatic
        // SAFETY: (2) raw-pointer buffer ABI. The caller contract requires `buf` to be
        // valid for writes of `len` bytes; the slice is used only by the loop below and
        // never escapes this critical section.
        unsafe {
            // Create a slice to the uninitialized buffer for safer indexing
            let buffer = core::slice::from_raw_parts_mut(buf, len as usize);

            // Use iterator instead of raw pointer arithmetic
            for byte in buffer.iter_mut() {
                *byte = mspi_get();
                if !mspi_wait() {
                    break;
                }
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
/// # Returns
///
/// * `true` if every byte was transmitted and the status poll ran to completion
/// * `false` if the SPI busy flag never cleared (see [`mspi_wait`]); the caller
///   is responsible for releasing chip select
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
fn flash_mspi_write_ram(cmd: FLASH_CMD, addr: u32, data: *const u8, data_len: u32) -> bool {
    // Use critical section for atomic SPI operation
    critical_section::with(|_| {
        // Send write enable command (required before any write operation)
        if !flash_send_cmd(FLASH_CMD::WRITE_ENABLE_CMD) {
            return false;
        }

        // Send the main write command
        if !flash_send_cmd(cmd) {
            return false;
        }

        // Send address
        if !flash_send_addr(addr) {
            return false;
        }

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
                if !mspi_wait() {
                    return false;
                }
            }
        }

        // End SPI transaction - initiates the internal write operation
        mspi_high();

        // Wait for write operation to complete with timeout protection
        // Initial delay to allow flash operation to start
        sleep_us(100);

        // Send command to read flash status register
        if !flash_send_cmd(FLASH_CMD::READ_STATUS_CMD_LOWBYTE) {
            return false;
        }

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
        true
    })
}

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
/// # Returns
///
/// * `Ok(())` when every page was programmed
/// * `Err(())` when the SPI bus stalled; chip select is released and the
///   remaining pages are not written
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
// The documented caller contract is a raw-pointer buffer ABI (`buf` valid for
// `len` readable bytes), kept as a safe fn to preserve the firmware API.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
// The project convention for fallible hardware calls is `Result<_, ()>`
// (docs/checklists/firmware.md item 23); the only failure is a bus timeout, so a
// bespoke error type would carry no information.
#[allow(clippy::result_unit_err)]
pub fn flash_write_page(addr: u32, len: u32, buf: *const u8) -> Result<(), ()> {
    // Early return if there's nothing to write
    if len == 0 || buf.is_null() {
        return Ok(());
    }

    #[cfg(not(target_os = "none"))]
    if FakeFlash::mode() == fake_flash::FakeFlashMode::InMemory {
        // SAFETY: (2) raw-pointer buffer ABI. The guard above proves `buf` is valid for
        // reads of `len` bytes; the slice never escapes this function.
        FakeFlash::write_slice(addr, unsafe {
            core::slice::from_raw_parts(buf, len as usize)
        });
        return Ok(());
    }

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
        if !flash_mspi_write_ram(
            FLASH_CMD::WRITE_CMD,
            current_addr,
            current_data_ptr,
            write_size,
        ) {
            // A stuck bus aborts the rest of the write: release chip select and
            // stop instead of driving a dead peripheral through more pages.
            mspi_high();
            return Err(());
        }

        // Update tracking variables
        bytes_written += write_size as usize;
        current_addr += write_size;
        bytes_remaining -= write_size;

        // For subsequent pages, use full page size
        current_page_remaining = PAGE_SIZE;
    }

    Ok(())
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
/// # Returns
///
/// * `Ok(())` when the erase command completed on the wire
/// * `Err(())` when the SPI bus stalled; chip select is released
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
// The project convention for fallible hardware calls is `Result<_, ()>`
// (docs/checklists/firmware.md item 23); the only failure is a bus timeout, so a
// bespoke error type would carry no information.
#[allow(clippy::result_unit_err)]
pub fn flash_erase_sector(addr: u32) -> Result<(), ()> {
    // Clear watchdog timer to prevent resets during long erase operation
    wd_clear();

    #[cfg(not(target_os = "none"))]
    if FakeFlash::mode() == fake_flash::FakeFlashMode::InMemory {
        FakeFlash::erase_sector(addr);
        return Ok(());
    }

    // Send sector erase command with the target address
    // No data is needed for erase operation (null pointer, 0 length)
    if !flash_mspi_write_ram(FLASH_CMD::SECT_ERASE_CMD, addr, null_mut(), 0) {
        // A stuck bus aborts the erase: release chip select before returning.
        mspi_high();
        return Err(());
    }

    Ok(())
}

/// In-process 512 KiB NOR flash model that the public `flash_*` entry points
/// drive directly when [`FakeFlashMode::InMemory`] is selected.
///
/// This is a **host-only device**: it is gated on `not(target_os = "none")`, so
/// it is absent from the firmware image and reachable from any host test binary
/// as `tlsr8266_hal::drivers::flash::fake_flash::FakeFlash`. The default mode is
/// [`FakeFlashMode::InMemory`]; [`FakeFlashMode::SpiRegisters`] is opt-in for
/// tests that need to observe the real `mspi_*` protocol transactions.
/// The device is process-global (one 512 KiB array per test binary): all state
/// lives behind one `Mutex`, and every lock recovers from poisoning, so a single
/// panicking test cannot break the rest of a `--test-threads=1` run.
#[cfg(not(target_os = "none"))]
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
        /// Drives real `mspi_*` transactions; opt-in for SPI-protocol tests that
        /// need to assert the wire sequence.
        SpiRegisters,
        /// Executes reads/writes/erases directly against the 512 KiB in-memory
        /// NOR array (the default).
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
                mode: FakeFlashMode::InMemory,
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
        /// fault, and the default `InMemory` mode.
        pub fn reset() {
            let mut state = lock();
            state.bytes.clear();
            state.bytes.resize(FLASH_SIZE_BYTES, 0xFF);
            state.erase_counts.clear();
            state.fail_next_write = false;
            state.mode = FakeFlashMode::InMemory;
        }

        /// Select where the `flash_*` entry points send their work. `InMemory` is
        /// the default; `SpiRegisters` is opt-in for SPI-protocol tests.
        pub fn set_mode(mode: FakeFlashMode) {
            lock().mode = mode;
        }

        /// The mode currently selected by the last [`FakeFlash::set_mode`] (or
        /// [`FakeFlash::reset`], which restores `InMemory`).
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
            // The injected fault is one-shot, like a single bad page program, and
            // is consumed by *this* write even when the range is out of bounds:
            // leaving it armed would corrupt an unrelated later write.
            let corrupt = state.fail_next_write;
            state.fail_next_write = false;
            let start = addr as usize;
            if start >= FLASH_SIZE_BYTES {
                return;
            }
            let count = core::cmp::min(data.len(), FLASH_SIZE_BYTES - start);
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
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::vec::Vec;

    use super::fake_flash::{FakeFlash, FakeFlashMode};
    use crate::drivers::spi::MSPI_WAIT_MAX_ITERS;
    use crate::mcu::clock::fake_clock;
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_pac::FLD_MASTER_SPI;

    /// Restores the host default (the in-memory NOR device) even if the owning
    /// test panics, so a failure cannot strand later tests in `SpiRegisters`
    /// mode. The device is process-global, hence the guard.
    struct RestoreDefault;

    impl Drop for RestoreDefault {
        fn drop(&mut self) {
            FakeFlash::set_mode(FakeFlashMode::InMemory);
        }
    }

    /// `reg_tmr_ctrl`: the watchdog clear bit lives here.
    const REG_TMR_CTRL: u32 = 0x620;
    /// `reg_master_spi_data`: command, address and payload bytes.
    const REG_MASTER_SPI_DATA: u32 = 0x0c;
    /// `reg_master_spi_ctrl`: chip select and busy bits.
    const REG_MASTER_SPI_CTRL: u32 = 0x0d;

    /// The HAL host seams are process-wide singletons, so every test that
    /// observes them starts from a known-empty bank, tick count and sleep log.
    fn reset_fakes() {
        FakeMmio::reset();
        fake_clock::reset();
    }

    /// Start a protocol test from a blank, deterministic device.
    fn reset_protocol_fakes() {
        reset_fakes();
        FakeFlash::reset();
    }

    /// Number of `reg_tmr_ctrl` writes, i.e. `wd_clear()` calls.
    fn watchdog_clears() -> usize {
        FakeMmio::writes()
            .into_iter()
            .filter(|write| write.addr == REG_TMR_CTRL | tlsr8266_pac::REG_BASE_ADDR)
            .count()
    }

    /// Values written to `addr`, oldest first, in the order the flash chip would
    /// have seen them on the SPI bus.
    fn values_written_to(addr: u32) -> Vec<u32> {
        FakeMmio::writes()
            .into_iter()
            .filter(|write| write.addr == addr | tlsr8266_pac::REG_BASE_ADDR)
            .map(|write| write.value)
            .collect()
    }

    /// How many times `value` was written to `addr`.
    fn count_writes_of(addr: u32, value: u32) -> usize {
        values_written_to(addr)
            .into_iter()
            .filter(|written| *written == value)
            .count()
    }

    /// The chip-select-asserted value of the SPI control register.
    fn cs_asserted() -> u32 {
        u32::from(FLD_MASTER_SPI::CS.bits())
    }

    /// Reads of `reg_master_spi_data` observed by [`always_busy`], i.e. status
    /// polls of the busy bit.
    static STATUS_POLLS: AtomicUsize = AtomicUsize::new(0);

    /// Reports a flash chip that never finishes its internal write.
    fn always_busy(_call_idx: usize) -> u8 {
        STATUS_POLLS.fetch_add(1, Ordering::Relaxed);
        0x01
    }

    // This is required because we're working with raw pointers in our testing
    // It helps to ensure safe operations when mocking hardware interactions
    fn vec_to_raw_mut(v: &mut Vec<u8>) -> *mut u8 {
        v.as_mut_ptr()
    }

    fn vec_to_raw(v: &[u8]) -> *const u8 {
        v.as_ptr()
    }

    /// Tests the flash_read_page function.
    ///
    /// This test verifies that the flash_read_page function correctly:
    /// - Sends the proper READ_CMD command
    /// - Transmits the address correctly
    /// - Sets the SPI controller to auto mode for efficient reading
    /// - Reads data into the provided buffer
    /// - Handles chip select and sleep timing properly
    ///
    /// # Notes
    ///
    /// * Runs against the real `mspi_*` accessors, so the assertions are the
    ///   byte stream the flash chip would observe on the wire.
    #[test]
    fn test_flash_read_page() {
        reset_protocol_fakes();
        FakeFlash::set_mode(FakeFlashMode::SpiRegisters);
        let _restore = RestoreDefault;

        // The flash chip answers successive data clocks with a test pattern.
        const PATTERN: [u8; 4] = [0xAA, 0xBB, 0xCC, 0xDD];
        FakeMmio::on_read8(REG_MASTER_SPI_DATA, |call_idx| {
            PATTERN[call_idx.min(PATTERN.len() - 1)]
        });

        // Create a buffer to hold read data
        let mut buffer = vec![0; 4];
        let buf_ptr = vec_to_raw_mut(&mut buffer);

        // Call the function we're testing
        flash_read_page(0x1000, 4, buf_ptr);

        // Verify expected buffer contents after read
        assert_eq!(buffer, PATTERN);

        // The wire sequence is READ_CMD, the 24-bit address, then one dummy byte
        // to clock the first response in.
        assert_eq!(
            values_written_to(REG_MASTER_SPI_DATA),
            vec![0x03, 0x00, 0x10, 0x00, 0x00]
        );

        // Chip select is asserted for the command, the controller switches to
        // auto mode for the bulk read, and the transaction ends deselected.
        assert_eq!(
            values_written_to(REG_MASTER_SPI_CTRL),
            vec![cs_asserted(), 0, 0x0a, cs_asserted()]
        );

        // Only the chip-select setup delay is requested.
        assert_eq!(fake_clock::sleeps(), vec![1]);
    }

    /// Tests flash_write_page function with small data that fits within one page.
    ///
    /// This test verifies that the flash_write_page function correctly:
    /// - Handles writes that do not cross page boundaries
    /// - Sequences the write-enable, write, address and payload bytes
    /// - Polls the status register until the internal write completes
    ///
    /// # Notes
    ///
    /// * This test focuses on the simple case where data fits within page boundaries
    /// * Page size is 256 bytes for this device
    #[test]
    fn test_flash_write_page_small() {
        reset_protocol_fakes();
        FakeFlash::set_mode(FakeFlashMode::SpiRegisters);
        let _restore = RestoreDefault;

        // Create test data to write
        let data = vec![0x11, 0x22, 0x33, 0x44];
        let data_ptr = vec_to_raw(&data);

        // Call function with address at start of a page boundary
        assert!(flash_write_page(0x1000, 4, data_ptr).is_ok());

        // WRITE_ENABLE, WRITE_CMD, the address, the payload, then the status
        // command and its dummy clock.
        assert_eq!(
            values_written_to(REG_MASTER_SPI_DATA),
            vec![0x06, 0x02, 0x00, 0x10, 0x00, 0x11, 0x22, 0x33, 0x44, 0x05, 0x00]
        );

        // One chip-select pulse per command, plus the release after the payload
        // and the final release once the write completed.
        assert_eq!(count_writes_of(REG_MASTER_SPI_CTRL, cs_asserted()), 5);
        assert_eq!(count_writes_of(REG_MASTER_SPI_CTRL, 0), 3);
        assert_eq!(
            values_written_to(REG_MASTER_SPI_CTRL).last().copied(),
            Some(cs_asserted()),
            "the transaction must end with the chip deselected"
        );

        // Two chip-select setups, the 100 us program delay, then the status poll.
        assert_eq!(fake_clock::sleeps(), vec![1, 1, 100, 1]);
    }

    /// Tests flash_write_page function with data that crosses page boundaries.
    ///
    /// This test verifies that the flash_write_page function correctly:
    /// - Splits the write into one page program per 256-byte page
    /// - Advances the address and the buffer position between pages
    /// - Sends each page's payload bytes in order
    ///
    /// # Notes
    ///
    /// * Flash can only write within a single page (256 bytes) in one operation
    /// * Starting at 0x10FA makes the first chunk 6 bytes, then 256, then 38
    #[test]
    fn test_flash_write_page_cross_boundary() {
        reset_protocol_fakes();
        FakeFlash::set_mode(FakeFlashMode::SpiRegisters);
        let _restore = RestoreDefault;

        // Create test data to write (spans three pages)
        let data: Vec<u8> = (0..300u32).map(|i| (i & 0xFF) as u8).collect();

        let data_ptr = vec_to_raw(&data);

        // Start near the end of a page to force crossing the boundary.
        let start_addr = 0x10FA; // 6 bytes before page boundary
        assert!(flash_write_page(start_addr, 300, data_ptr).is_ok());

        // Reconstruct the expected byte stream: one write-enable + write command
        // + 24-bit address + payload + status command per page.
        let chunks: [(u32, &[u8]); 3] = [
            (start_addr, &data[0..6]),
            (0x1100, &data[6..262]),
            (0x1200, &data[262..300]),
        ];
        let mut expected = Vec::new();
        for (addr, chunk) in chunks {
            expected.extend([
                0x06,
                0x02,
                (addr >> 16) & 0xff,
                (addr >> 8) & 0xff,
                addr & 0xff,
            ]);
            expected.extend(chunk.iter().map(|byte| u32::from(*byte)));
            expected.extend([0x05, 0x00]);
        }
        assert_eq!(values_written_to(REG_MASTER_SPI_DATA), expected);
    }

    /// Tests the flash_erase_sector function.
    ///
    /// This test verifies that the sector erase operation correctly:
    /// - Clears the watchdog timer before erasing
    /// - Sends the write-enable and sector-erase commands with the address
    /// - Polls the status register until the erase completes
    ///
    /// # Notes
    ///
    /// * The function should ensure the watchdog is cleared before long erase operations
    /// * SPI protocol requires specific chip select sequence
    #[test]
    fn test_flash_erase_sector() {
        reset_protocol_fakes();
        FakeFlash::set_mode(FakeFlashMode::SpiRegisters);
        let _restore = RestoreDefault;

        // Call function to erase sector
        assert!(flash_erase_sector(0x2000).is_ok());

        // Verify watchdog was cleared
        assert_eq!(
            watchdog_clears(),
            1,
            "the erase path must kick the watchdog once"
        );

        // WRITE_ENABLE, SECT_ERASE with its 24-bit address, no payload, then the
        // status command and its dummy clock.
        assert_eq!(
            values_written_to(REG_MASTER_SPI_DATA),
            vec![0x06, 0x20, 0x00, 0x20, 0x00, 0x05, 0x00]
        );
        assert_eq!(count_writes_of(REG_MASTER_SPI_CTRL, cs_asserted()), 5);
        assert_eq!(count_writes_of(REG_MASTER_SPI_CTRL, 0), 3);
        assert_eq!(
            values_written_to(REG_MASTER_SPI_CTRL).last().copied(),
            Some(cs_asserted()),
            "the erase transaction must end with the chip deselected"
        );
    }

    /// Tests the flash_send_addr function.
    ///
    /// This test verifies that the flash_send_addr function correctly transmits
    /// a 24-bit address in big-endian format (MSB first) over SPI.
    ///
    /// # Notes
    ///
    /// * Flash protocol requires MSB-first transmission of addresses
    #[test]
    fn test_flash_send_addr() {
        reset_protocol_fakes();
        FakeFlash::set_mode(FakeFlashMode::SpiRegisters);
        let _restore = RestoreDefault;

        // Test address: 0x123456 - should send bytes 0x12, 0x34, 0x56 in that order
        assert!(
            flash_send_addr(0x123456),
            "the address phase must report success"
        );

        assert_eq!(
            values_written_to(REG_MASTER_SPI_DATA),
            vec![0x12, 0x34, 0x56],
            "the address must be clocked out MSB first"
        );
    }

    /// Integration test of a complete flash erase-write-read cycle.
    ///
    /// This test verifies the full operational sequence of flash memory operations
    /// through the public API and the in-memory NOR device:
    /// 1. Reading erased flash (all 0xFF)
    /// 2. Erasing the target sector without disturbing its neighbour
    /// 3. Writing data to flash
    /// 4. Reading back the data to confirm it was written correctly
    ///
    /// # Notes
    ///
    /// * The watchdog is verified to be cleared during the erase operation
    /// * The device is real storage now, so read-back proves the write
    #[test]
    fn test_flash_read_write_cycle() {
        reset_fakes();
        FakeFlash::reset();
        FakeFlash::set_mode(FakeFlashMode::InMemory);
        let _restore = RestoreDefault;

        const ADDR: u32 = 0x3000;
        const NEIGHBOUR: u32 = 0x2000;

        // A pattern in the neighbouring sector must survive the erase.
        let neighbour_pattern = [0x5Au8, 0xA5];
        FakeFlash::write_slice(NEIGHBOUR, &neighbour_pattern);

        // Step 1: the target sector starts erased.
        let mut read_buffer = [0u8; 4];
        flash_read_page(ADDR, 4, read_buffer.as_mut_ptr());
        assert_eq!(read_buffer, [0xFF; 4]);

        // Step 2: erase the sector to prepare for the write.
        assert!(flash_erase_sector(ADDR).is_ok());
        assert_eq!(
            watchdog_clears(),
            1,
            "the erase path must kick the watchdog once"
        );

        // Step 3: write data to flash.
        let write_data = [0xA1u8, 0xB2, 0xC3, 0xD4];
        assert!(flash_write_page(ADDR, 4, write_data.as_ptr()).is_ok());

        // Step 4: read the data back.
        let mut verify_buffer = [0u8; 4];
        flash_read_page(ADDR, 4, verify_buffer.as_mut_ptr());

        // Step 5: verify the data matches what we wrote.
        assert_eq!(verify_buffer, write_data);

        // The erase hit exactly one sector, and the neighbour is untouched.
        assert_eq!(FakeFlash::erase_count(ADDR), 1);
        assert_eq!(FakeFlash::erase_count(NEIGHBOUR), 0);
        let mut neighbour = [0u8; 2];
        FakeFlash::read_slice(NEIGHBOUR, &mut neighbour);
        assert_eq!(neighbour, neighbour_pattern);
    }

    /// Tests the flash busy flag timeout protection in flash_mspi_write_ram.
    ///
    /// This test verifies that the function handles the scenario where the flash
    /// busy flag never clears (the status read always reports busy), ensuring that
    /// the loop eventually exits after FLASH_WAIT_ITERATIONS attempts.
    #[test]
    fn test_flash_busy_flag_timeout() {
        reset_protocol_fakes();
        FakeFlash::set_mode(FakeFlashMode::SpiRegisters);
        let _restore = RestoreDefault;

        // Critical: always report "busy" so the polling loop must exhaust its
        // iteration budget instead of exiting early.
        STATUS_POLLS.store(0, Ordering::Relaxed);
        FakeMmio::on_read8(REG_MASTER_SPI_DATA, always_busy);

        // Test data to write
        let data = vec![0x11, 0x22, 0x33, 0x44];
        let data_ptr = vec_to_raw(&data);

        // Call the function - it must complete after FLASH_WAIT_ITERATIONS cycles
        // without hanging indefinitely. The bus itself never stalled, so the
        // page program reports success even though the part stayed busy.
        assert!(flash_write_page(0x1000, 4, data_ptr).is_ok());

        // Every budgeted poll happened, and no more.
        assert_eq!(
            STATUS_POLLS.load(Ordering::Relaxed),
            FLASH_WAIT_ITERATIONS as usize,
            "the busy wait must stop after FLASH_WAIT_ITERATIONS polls"
        );

        // The transaction still ends with the chip deselected.
        assert_eq!(
            values_written_to(REG_MASTER_SPI_CTRL).last().copied(),
            Some(cs_asserted()),
            "a timed-out write must still release chip select"
        );
    }

    /// Reads of `reg_master_spi_ctrl` observed by [`stuck_spi`].
    static CTRL_POLLS: AtomicUsize = AtomicUsize::new(0);

    /// Reports an SPI peripheral whose BUSY flag never clears, i.e. a bus that
    /// hangs mid-transaction.
    fn stuck_spi(_call_idx: usize) -> u8 {
        CTRL_POLLS.fetch_add(1, Ordering::Relaxed);
        FLD_MASTER_SPI::BUSY.bits()
    }

    /// A stuck SPI bus aborts `flash_read_page`: only the command byte reaches
    /// the wire, the caller's buffer is untouched, and chip select is released.
    #[test]
    fn test_flash_read_page_stuck_spi_releases_cs() {
        reset_protocol_fakes();
        FakeFlash::set_mode(FakeFlashMode::SpiRegisters);
        let _restore = RestoreDefault;

        CTRL_POLLS.store(0, Ordering::Relaxed);
        FakeMmio::on_read8(REG_MASTER_SPI_CTRL, stuck_spi);

        let mut buffer = [0x11u8; 4];
        flash_read_page(0x1000, 4, buffer.as_mut_ptr());

        // The command byte was clocked out; the address phase never started.
        assert_eq!(
            values_written_to(REG_MASTER_SPI_DATA),
            vec![0x03],
            "a timed-out read must not send the address or dummy bytes"
        );

        // The caller's buffer was not written from a dead bus.
        assert_eq!(
            buffer, [0x11; 4],
            "a timed-out read must leave the buffer untouched"
        );

        // The budget was spent exactly once and the chip is deselected.
        assert_eq!(
            CTRL_POLLS.load(Ordering::Relaxed),
            MSPI_WAIT_MAX_ITERS as usize,
            "the completion wait must stop after MSPI_WAIT_MAX_ITERS polls"
        );
        assert_eq!(
            values_written_to(REG_MASTER_SPI_CTRL).last().copied(),
            Some(cs_asserted()),
            "a timed-out read must still release chip select"
        );
    }

    /// A stuck SPI bus aborts `flash_write_page` before any payload byte reaches
    /// the wire, and chip select is released.
    #[test]
    fn test_flash_write_page_stuck_spi_releases_cs() {
        reset_protocol_fakes();
        FakeFlash::set_mode(FakeFlashMode::SpiRegisters);
        let _restore = RestoreDefault;

        CTRL_POLLS.store(0, Ordering::Relaxed);
        FakeMmio::on_read8(REG_MASTER_SPI_CTRL, stuck_spi);

        let data = vec![0x11, 0x22, 0x33, 0x44];
        assert!(
            flash_write_page(0x1000, 4, vec_to_raw(&data)).is_err(),
            "a stalled bus must report a failed page program"
        );

        // The write-enable command was clocked out; the write command, address
        // and payload never were.
        assert_eq!(
            values_written_to(REG_MASTER_SPI_DATA),
            vec![0x06],
            "a timed-out write must not send command, address or payload"
        );
        assert_eq!(
            values_written_to(REG_MASTER_SPI_CTRL).last().copied(),
            Some(cs_asserted()),
            "a timed-out write must still release chip select"
        );
    }

    /// A stuck SPI bus aborts `flash_erase_sector` with chip select deasserted,
    /// and the watchdog is still kicked first.
    #[test]
    fn test_flash_erase_sector_stuck_spi_releases_cs() {
        reset_protocol_fakes();
        FakeFlash::set_mode(FakeFlashMode::SpiRegisters);
        let _restore = RestoreDefault;

        CTRL_POLLS.store(0, Ordering::Relaxed);
        FakeMmio::on_read8(REG_MASTER_SPI_CTRL, stuck_spi);

        assert!(
            flash_erase_sector(0x2000).is_err(),
            "a stalled bus must report a failed sector erase"
        );

        assert_eq!(
            watchdog_clears(),
            1,
            "the erase path must kick the watchdog"
        );
        assert_eq!(
            values_written_to(REG_MASTER_SPI_DATA),
            vec![0x06],
            "a timed-out erase must not send the erase command or address"
        );
        assert_eq!(
            values_written_to(REG_MASTER_SPI_CTRL).last().copied(),
            Some(cs_asserted()),
            "a timed-out erase must still release chip select"
        );
    }

    /// Tests the early return condition in flash_write_page function.
    ///
    /// This test verifies that the function correctly exits early when:
    /// 1. The length parameter is 0, or
    /// 2. The buffer pointer is null
    ///
    /// In either case, the function should return immediately without touching
    /// the SPI bus or the device.
    #[test]
    fn test_flash_write_page_early_return() {
        reset_protocol_fakes();
        FakeFlash::set_mode(FakeFlashMode::SpiRegisters);
        let _restore = RestoreDefault;

        // Create a valid test data buffer (won't be used with length 0)
        let data = vec![0x11, 0x22, 0x33, 0x44];
        let data_ptr = vec_to_raw(&data);

        // Case 1: Test with zero length but valid buffer
        assert!(flash_write_page(0x1000, 0, data_ptr).is_ok());

        // Case 2: Test with null buffer but non-zero length
        assert!(flash_write_page(0x1000, 4, std::ptr::null()).is_ok());

        // No register was touched at all: no chip select, no command, no poll.
        assert!(
            FakeMmio::writes().is_empty(),
            "the early return must not touch the SPI bus"
        );

        // No delays should have been called either
        assert!(
            fake_clock::sleeps().is_empty(),
            "the early return must not delay"
        );
    }

    /// Proves the real flash entry points work against the in-memory NOR model:
    /// erase reads back `0xFF` across the whole 4 KiB sector, a written pattern
    /// reads back byte-exact, and sequential writes obey NOR `AND` semantics
    /// without any SPI mocking.
    #[test]
    fn test_fake_flash_erase_write_read_round_trip() {
        FakeFlash::reset();
        FakeFlash::set_mode(FakeFlashMode::InMemory);
        let _restore = RestoreDefault;

        const ADDR: u32 = 0x1_0000;

        // Dirty the last byte of the sector so the erase is observably 4 KiB wide.
        let last_byte_in_sector = ADDR + fake_flash::SECTOR_SIZE_BYTES as u32 - 1;
        FakeFlash::write_slice(last_byte_in_sector, &[0x00]);

        // Erased silicon reads 0xFF.
        assert!(flash_erase_sector(ADDR).is_ok());
        let mut erased = [0u8; 8];
        flash_read_page(ADDR, 8, erased.as_mut_ptr());
        assert_eq!(erased, [0xFF; 8]);

        // ... including the far end of the 4 KiB sector.
        let mut tail = [0u8; 1];
        flash_read_page(last_byte_in_sector, 1, tail.as_mut_ptr());
        assert_eq!(tail, [0xFF], "the erase must clear the whole 4 KiB sector");

        // A pattern written through the public API reads back byte-exact.
        let pattern = [0xA5u8, 0x5A, 0x0F, 0xF0];
        assert!(flash_write_page(ADDR, pattern.len() as u32, pattern.as_ptr()).is_ok());
        let mut readback = [0u8; 4];
        flash_read_page(ADDR, 4, readback.as_mut_ptr());
        assert_eq!(readback, pattern);

        // NOR writes can only clear bits: 0xFF & 0x0F & 0xF0 == 0x00.
        assert!(flash_erase_sector(ADDR).is_ok());
        let ones_nibble = [0x0Fu8];
        assert!(flash_write_page(ADDR, 1, ones_nibble.as_ptr()).is_ok());
        let high_nibble = [0xF0u8];
        assert!(flash_write_page(ADDR, 1, high_nibble.as_ptr()).is_ok());
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
        let _restore = RestoreDefault;

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
        let _restore = RestoreDefault;

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

        // An out-of-range write consumes the pending fault instead of stranding
        // it for an unrelated later write.
        FakeFlash::fail_verify_next_write();
        FakeFlash::write_slice(fake_flash::FLASH_SIZE_BYTES as u32, &[0x00]);
        FakeFlash::write_slice(ADDR + 2, &[0xFF]);
        FakeFlash::read_slice(ADDR + 2, &mut byte);
        assert_eq!(byte[0], 0xFF, "a stranded fault must not hit a later write");
    }

    /// The raw-pointer guard in `flash_read_page` rejects a zero length and a
    /// null buffer in `InMemory` mode without touching memory.
    #[test]
    fn test_fake_flash_zero_len_and_null_guards() {
        FakeFlash::reset();
        FakeFlash::set_mode(FakeFlashMode::InMemory);
        let _restore = RestoreDefault;

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
