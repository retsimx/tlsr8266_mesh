//! Reset-surviving diagnostics.
//!
//! Three pieces of on-target observability that cost nothing when unused:
//!
//! * `rtt` (feature-gated) — a 272 B byte ring in `.bss` exposed as `_TLSR_RTT`,
//!   so a SWire-attached host can stream log output live.
//! * `HEARTBEAT` (in `lib.rs`) — a 4 B counter bumped once per main-loop pass.
//! * `analog 0x3b` / `analog 0x3c` — hardware-retained reset sentinel and last
//!   breadcrumb, so the next boot can report why the previous one ended.
//!
//! This module is the application layer's leaf: it reports values, never log
//! lines. `app.rs` decides how a reason is printed, and `lib.rs` owns the
//! workspace panic handler, so nothing here names a sibling subsystem.
//!
//! See `docs/plans/designs/005-rtt-target-hooks.md` (sections 4.1-4.5) for the
//! contract with the host tools.

use core::ptr::addr_of_mut;

use crate::config::{FLASH_ADR_PANIC_INFO, PANIC_VALID_FLAG};
use tlsr8266_hal::drivers::flash::{flash_erase_sector, flash_read_page};
use tlsr8266_hal::mcu::analog::{analog_read, analog_write};

// Panic-record writes exist only on the target build and in unit tests (which
// drive the writer), so the plain host library build never sees these.
#[cfg(any(target_os = "none", test))]
use core::ptr::addr_of;
#[cfg(any(target_os = "none", test))]
use tlsr8266_hal::drivers::flash::flash_write_page;

#[cfg(any(feature = "rtt", test))]
pub mod rtt;

/// Analog register holding why the previous boot ended.
///
/// The register addresses and sentinel values are hardware contracts owned by
/// the software-reboot path, so they are defined in `sdk::pm` and re-exported
/// here for existing callers.
pub use tlsr8266_hal::pm::{
    REGA_BREADCRUMB, REGA_RESET_SENTINEL, SENTINEL_PANIC, SENTINEL_RUNNING,
};
// Only this module's tests read the clean-reboot sentinel, so the re-export is
// test-only rather than a blanket lint allowance on a production item.
#[cfg(test)]
pub use tlsr8266_hal::pm::SENTINEL_CLEAN_REBOOT;

pub const CRUMB_BOOT: u8 = 1;
pub const CRUMB_INIT_DONE: u8 = 2;
pub const CRUMB_MAIN_LOOP: u8 = 3;
/// Written by the panic handler, which only exists on the target build.
#[cfg(any(target_os = "none", test))]
pub const CRUMB_PANIC: u8 = 0xFF;

/// Why the previous boot ended, read from the hardware-retained sentinel.
///
/// `Clean` covers "no retained sentinel"; `Watchdog` carries the sentinel a
/// watchdog reset leaves behind; `Panic` carries the panic handler's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BootResetReason {
    Clean,
    Watchdog { breadcrumb: u8 },
    Panic { breadcrumb: u8 },
}

/// The panic message still held in flash, clipped for a single log line.
///
/// `len` is the number of bytes in `buf` and `truncated` says the marker is
/// owed: either the record is longer than the bounded read, or the clip cut it.
pub struct PanicPreview {
    pub buf: [u8; PANIC_READ_MAX],
    pub len: usize,
    pub truncated: bool,
}

/// The application's UART log stream holds 128 bytes and rejects a write that
/// would overflow it instead of panicking (the formatter then stops at the first
/// failed `write_str`, so an over-long log line is clipped rather than
/// rebooting). A panic message read from flash (up to 256 bytes) must still be
/// clipped first: a panic inside the boot report would reboot into the same
/// panic forever.
pub(crate) const PANIC_LOG_BODY_MAX: usize = 72;

/// Only the first `PANIC_LOG_BODY_MAX` bytes are ever logged, so the flash read
/// buffer is bounded by that (plus UTF-8 slack) — a 256 B buffer cost 180+ B of
/// main-stack margin for bytes that are never printed.
///
/// The 4 slack bytes let a character that straddles the clip point decode
/// whole, so the cut is decided by the clip (a visible char boundary) rather
/// than by the read bound; they are never printed themselves.
pub(crate) const PANIC_READ_MAX: usize = PANIC_LOG_BODY_MAX + 4;

/// Records how far this boot got, so the next boot can report where it died.
pub fn record_breadcrumb(tag: u8) {
    analog_write(REGA_BREADCRUMB, tag);
}

/// Boot-time diagnostics: start the RTT sink (when compiled in), report why the
/// previous boot ended, and mark this one as running.
///
/// Runs before the version banner so a `[PANIC]` line is the first thing the
/// host sees, while the panic sector is still intact.
pub fn boot_init() -> BootResetReason {
    #[cfg(feature = "rtt")]
    rtt::rtt_init();

    let prev_sentinel = analog_read(REGA_RESET_SENTINEL);
    let prev_crumb = analog_read(REGA_BREADCRUMB);

    let reason = match prev_sentinel {
        SENTINEL_RUNNING => BootResetReason::Watchdog {
            breadcrumb: prev_crumb,
        },
        SENTINEL_PANIC => BootResetReason::Panic {
            breadcrumb: prev_crumb,
        },
        _ => BootResetReason::Clean,
    };

    // This boot is alive from here on: a later watchdog reset inherits 0xA5.
    analog_write(REGA_RESET_SENTINEL, SENTINEL_RUNNING);
    record_breadcrumb(CRUMB_BOOT);

    reason
}

/// Reads the panic message left in flash for the boot report.
///
/// Non-erasing: `read_panic_report()` still owns consuming the sector. The read
/// is bounded to [`PANIC_READ_MAX`] and clipped to the UART budget, so the body
/// always fits one log line. `Ok(None)` means the sector holds no trusted
/// record; `Err(())` means the flash bus failed, so the caller reports the
/// message as unreadable rather than printing a partial body.
pub fn read_panic_preview() -> Result<Option<PanicPreview>, ()> {
    let mut flag: u8 = 0;
    flash_read_page(FLASH_ADR_PANIC_INFO, 1, addr_of_mut!(flag))?;
    if flag != PANIC_VALID_FLAG {
        return Ok(None);
    }

    let mut len_bytes: [u8; 2] = [0; 2];
    flash_read_page(FLASH_ADR_PANIC_INFO + 1, 2, len_bytes.as_mut_ptr())?;
    let len = u16::from_le_bytes(len_bytes) as usize;
    if len > 256 {
        return Ok(None);
    }

    // Only the logged prefix is read, and `len` is checked against the 256-byte
    // sector record above.
    let read = len.min(PANIC_READ_MAX);
    let mut message: [u8; PANIC_READ_MAX] = [0; PANIC_READ_MAX];
    flash_read_page(FLASH_ADR_PANIC_INFO + 3, read as u32, message.as_mut_ptr())?;

    // The bounded read can end mid-codepoint, so keep only the prefix that
    // decodes to valid UTF-8.
    let bytes = &message[..read];
    let text = match core::str::from_utf8(bytes) {
        Ok(text) => text,
        Err(err) => core::str::from_utf8(&bytes[..err.valid_up_to()]).unwrap_or(""),
    };

    let body = clip_to_uart_budget(text);
    // The marker is owed whenever bytes were dropped: by the bounded read (the
    // message is longer than `PANIC_READ_MAX`) or by the stream clip.
    let truncated = len > read || text.len() > body.len();

    let mut preview = PanicPreview {
        buf: [0; PANIC_READ_MAX],
        len: body.len(),
        truncated,
    };
    preview.buf[..body.len()].copy_from_slice(body.as_bytes());

    Ok(Some(preview))
}

/// Keeps the longest prefix of `msg` that fits the UART stream, on a char
/// boundary.
fn clip_to_uart_budget(msg: &str) -> &str {
    let mut end = msg.len().min(PANIC_LOG_BODY_MAX);
    while !msg.is_char_boundary(end) {
        end -= 1;
    }

    &msg[..end]
}

/// Reads the stored panic message into `out`, returning its length.
///
/// `Ok(None)` means the sector holds no complete record: either the flag is
/// blank, or the length word is zero / larger than `out`. A blank flag leaves the
/// sector untouched (nothing was ever written), while a corrupt record erases it
/// — such a record can never be delivered, so erasing it stops it being
/// re-examined on every boot. That makes [`clear_panic_report`] the normal eraser
/// rather than the only one. `Err(())` means the flash bus failed; `out` may
/// hold a partial prefix and must be discarded, and the sector is left
/// intact so the record can be retried after the next reset.
pub fn read_panic_report(out: &mut [u8; 256]) -> Result<Option<usize>, ()> {
    let mut flag: u8 = 0;
    flash_read_page(FLASH_ADR_PANIC_INFO, 1, addr_of_mut!(flag))?;

    // If the panic info flag isn't set, there's nothing to do
    if flag != PANIC_VALID_FLAG {
        return Ok(None);
    }

    let mut len_bytes: [u8; 2] = [0; 2];
    flash_read_page(FLASH_ADR_PANIC_INFO + 1, 2, len_bytes.as_mut_ptr())?;
    let length = u16::from_le_bytes(len_bytes) as usize;

    // A zero length or a length beyond the sector record cannot be a record this
    // build wrote, so it is rejected rather than read past `out`; the corrupt
    // record is erased so it is not retried on every boot.
    if length == 0 || length > out.len() {
        // The corrupt record is erased so it is not retried on every boot; a
        // failed erase still leaves the same reject path, so it is discarded.
        let _ = flash_erase_sector(FLASH_ADR_PANIC_INFO);
        return Ok(None);
    }

    flash_read_page(FLASH_ADR_PANIC_INFO + 3, length as u32, out.as_mut_ptr())?;

    Ok(Some(length))
}

/// Erases the panic sector once its report has been delivered.
pub fn clear_panic_report() {
    // A failure here only means the report may be delivered again after the
    // next reset, which is the pre-existing behaviour on a bad bus.
    let _ = flash_erase_sector(FLASH_ADR_PANIC_INFO);
}

/// Persists the panic message for the next boot.
///
/// Sector layout — a contract with the host tools:
/// `valid_flag: u8 + 0`, `length: u16 + 1`, `data: [u8; 256] + 3`.
///
/// The valid flag is written last: a reset part-way through the erase/write
/// sequence then leaves a sector without a valid flag, so the next boot reports
/// nothing rather than a half-written message.
#[cfg(any(target_os = "none", test))]
pub fn write_panic_bytes(bytes: &[u8]) {
    let panic_addr = FLASH_ADR_PANIC_INFO;

    // Clear the panic info sector for writing just in case. This runs in the
    // panic handler: with `panic = "abort"` a hardware failure here is already
    // fatal, and the valid flag below is never written on a partial record.
    let _ = flash_erase_sector(panic_addr);

    // Panic section looks as follows
    // valid_flag: u8   + 0
    // length: u16      + 1
    // data: [u8; 256]  + 3

    // Write the length first
    let length = bytes.len() as u16;
    let _ = flash_write_page(panic_addr + 1, 2, addr_of!(length) as *const u8);

    // Then write the panic message
    let _ = flash_write_page(panic_addr + 3, length as u32, bytes.as_ptr());

    // Finally write the panic valid flag last
    let flag = PANIC_VALID_FLAG;
    let _ = flash_write_page(panic_addr, 1, addr_of!(flag));
}

/// Marks the retained sentinel and breadcrumb as "this boot panicked".
///
/// The panic handler calls this first: a double panic must not cost the next
/// boot its breadcrumb.
#[cfg(any(target_os = "none", test))]
pub fn record_panic_sentinel() {
    analog_write(REGA_RESET_SENTINEL, SENTINEL_PANIC);
    analog_write(REGA_BREADCRUMB, CRUMB_PANIC);
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use tlsr8266_hal::drivers::flash::fake_flash::{FakeFlash, FakeFlashMode};
    use tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog;
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_pac::FLD_MASTER_SPI;

    /// `reg_master_spi_ctrl`: chip select and busy bits.
    const REG_MASTER_SPI_CTRL: u32 = 0x0d;

    /// Restores the host default (the in-memory NOR device) even if the owning
    /// test panics, so a failure cannot strand later tests in `SpiRegisters`
    /// mode. The device is process-global, hence the guard.
    struct RestoreInMemory;

    impl Drop for RestoreInMemory {
        fn drop(&mut self) {
            FakeFlash::set_mode(FakeFlashMode::InMemory);
        }
    }

    /// Seed the panic-info sector with the layout `write_panic_bytes` produces:
    /// valid flag at `+0`, `len` at `+1`, `body` at `+3`.
    fn seed_panic_info(body: &[u8], len: u16) {
        FakeFlash::write_slice(FLASH_ADR_PANIC_INFO, &[PANIC_VALID_FLAG]);
        FakeFlash::write_slice(FLASH_ADR_PANIC_INFO + 1, &len.to_le_bytes());
        FakeFlash::write_slice(FLASH_ADR_PANIC_INFO + 3, body);
    }

    /// `boot_init` reports a watchdog reset and still marks this boot alive.
    #[test]
    fn test_boot_init_wd_reset() {
        FakeAnalog::reset();
        FakeFlash::reset();

        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_RUNNING);
        FakeAnalog::set(REGA_BREADCRUMB, CRUMB_MAIN_LOOP);

        assert_eq!(
            boot_init(),
            BootResetReason::Watchdog {
                breadcrumb: CRUMB_MAIN_LOOP
            }
        );

        assert_eq!(
            FakeAnalog::get(REGA_RESET_SENTINEL),
            SENTINEL_RUNNING,
            "the analog register must hold the written value"
        );
        assert_eq!(
            FakeAnalog::get(REGA_BREADCRUMB),
            CRUMB_BOOT,
            "the analog register must hold the written value"
        );
    }

    #[test]
    fn test_boot_init_clean_reboot() {
        FakeAnalog::reset();
        FakeFlash::reset();

        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_CLEAN_REBOOT);
        FakeAnalog::set(REGA_BREADCRUMB, CRUMB_MAIN_LOOP);

        assert_eq!(boot_init(), BootResetReason::Clean);

        assert_eq!(
            FakeAnalog::get(REGA_RESET_SENTINEL),
            SENTINEL_RUNNING,
            "the analog register must hold the written value"
        );
        assert_eq!(
            FakeAnalog::get(REGA_BREADCRUMB),
            CRUMB_BOOT,
            "the analog register must hold the written value"
        );
    }

    /// `boot_init` is the hook `app::main_task` calls: it must report the panic
    /// crate and breadcrumb without consuming the panic sector.
    #[test]
    fn test_boot_init_panic_leaves_sector_intact() {
        FakeAnalog::reset();
        FakeFlash::reset();

        let body = b"panic body";

        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_PANIC);
        FakeAnalog::set(REGA_BREADCRUMB, CRUMB_PANIC);
        seed_panic_info(body, body.len() as u16);

        assert_eq!(
            boot_init(),
            BootResetReason::Panic {
                breadcrumb: CRUMB_PANIC
            }
        );

        assert_eq!(
            FakeAnalog::get(REGA_RESET_SENTINEL),
            SENTINEL_RUNNING,
            "the analog register must hold the written value"
        );
        assert_eq!(
            FakeAnalog::get(REGA_BREADCRUMB),
            CRUMB_BOOT,
            "the analog register must hold the written value"
        );
        // `read_panic_report()` owns erasing the sector, not the boot report.
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_PANIC_INFO),
            0,
            "the boot report must leave the panic sector intact"
        );
    }

    #[test]
    fn test_read_panic_preview_message() {
        FakeFlash::reset();

        let body = b"panic body";
        seed_panic_info(body, body.len() as u16);

        let preview = read_panic_preview()
            .expect("the flash read must succeed")
            .expect("a valid record yields a preview");

        assert_eq!(&preview.buf[..preview.len], body);
        assert!(!preview.truncated);
    }

    /// A panic body longer than the stream budget is clipped and marked.
    #[test]
    fn test_read_panic_preview_truncated() {
        FakeFlash::reset();

        let body = [b'x'; 200];
        seed_panic_info(&body, body.len() as u16);

        let preview = read_panic_preview()
            .expect("the flash read must succeed")
            .expect("a valid record yields a preview");

        assert_eq!(&preview.buf[..preview.len], &body[..PANIC_LOG_BODY_MAX]);
        assert!(preview.truncated);
    }

    /// A message longer than the flash read window is marked even when the
    /// bytes that *were* read decode to exactly the 72-byte budget, so a cut can
    /// never look complete.
    #[test]
    fn test_read_panic_preview_marks_short_body_cut_by_read() {
        FakeFlash::reset();

        // 72 printable bytes followed by a byte that is invalid UTF-8, then more
        // data beyond the read window: the decoded prefix is exactly the budget.
        let mut body = [b'a'; 200];
        body[PANIC_LOG_BODY_MAX] = 0xFF;
        seed_panic_info(&body, body.len() as u16);

        let preview = read_panic_preview()
            .expect("the flash read must succeed")
            .expect("a valid record yields a preview");

        assert_eq!(&preview.buf[..preview.len], &body[..PANIC_LOG_BODY_MAX]);
        assert!(preview.truncated);
    }

    /// A message that fits the flash read window exactly but not the 72-byte
    /// stream budget is still marked, i.e. the clip alone can require `...`.
    #[test]
    fn test_read_panic_preview_marks_body_clipped_within_read() {
        FakeFlash::reset();

        let body = [b'w'; PANIC_READ_MAX];
        seed_panic_info(&body, body.len() as u16);

        let preview = read_panic_preview()
            .expect("the flash read must succeed")
            .expect("a valid record yields a preview");

        assert_eq!(&preview.buf[..preview.len], &body[..PANIC_LOG_BODY_MAX]);
        assert!(preview.truncated);
    }

    /// A dead flash bus is reported as unreadable: the readers return `Err(())`
    /// instead of `Ok(None)` (no record) or a partially filled body.
    #[test]
    fn test_panic_readers_unreadable_bus() {
        FakeFlash::reset();
        FakeMmio::reset();
        FakeFlash::set_mode(FakeFlashMode::SpiRegisters);
        let _restore = RestoreInMemory;

        // The SPI busy flag never clears: every transfer gives up on its budget.
        FakeMmio::on_read8(REG_MASTER_SPI_CTRL, |_| FLD_MASTER_SPI::BUSY.bits());

        assert!(
            matches!(read_panic_preview(), Err(())),
            "a dead bus must not look like an absent panic record"
        );

        let mut buf = [0x5Au8; 256];
        assert!(
            matches!(read_panic_report(&mut buf), Err(())),
            "a dead bus must not report a length for a partial read"
        );
    }

    /// AC4: an invalid panic flag means the sector is not trusted at all.
    #[test]
    fn test_read_panic_preview_invalid_flag() {
        FakeFlash::reset();

        FakeFlash::write_slice(FLASH_ADR_PANIC_INFO, &[PANIC_VALID_FLAG ^ 0xFF]);

        // None at all: the invalid flag short-circuits before the length read.
        assert!(matches!(read_panic_preview(), Ok(None)));
    }

    /// AC4: a length beyond the 256-byte sector record is rejected before any
    /// body bytes are read.
    #[test]
    fn test_read_panic_preview_length_over_limit() {
        FakeFlash::reset();

        let len: u16 = 257;
        seed_panic_info(&[b'x'; 8], len);

        assert!(matches!(read_panic_preview(), Ok(None)));
    }

    /// AC4: the bounded read can end inside a multi-byte character. The partial
    /// code point is dropped (`from_utf8` error clamp), never replaced and never
    /// logged raw, and the clipped body is marked.
    #[test]
    fn test_read_panic_preview_truncated_utf8() {
        FakeFlash::reset();

        // 75 ASCII bytes, then `é` (0xC3 0xA9) starting at byte 75 so its second
        // byte is cut by the 76-byte read, then more data that is never read.
        let mut body = [b'a'; 200];
        body[PANIC_READ_MAX - 1] = 0xC3;
        body[PANIC_READ_MAX] = 0xA9;
        let len = body.len() as u16;

        // The clamp decodes 75 ASCII bytes, then `clip_to_uart_budget` keeps the
        // 72 the stream budget allows and the marker says the cut happened.
        seed_panic_info(&body, len);

        let preview = read_panic_preview()
            .expect("the flash read must succeed")
            .expect("a valid record yields a preview");
        let decoded = &preview.buf[..preview.len];

        assert_eq!(decoded, &body[..PANIC_LOG_BODY_MAX]);
        assert!(preview.truncated);
        // Byte-exact: no replacement character (0xEF 0xBF 0xBD) and no stray
        // lead byte (0xC3) can appear in the reported body.
        assert!(!decoded.windows(3).any(|w| w == [0xEF, 0xBF, 0xBD]));
        assert!(!decoded.contains(&0xC3));
        assert_eq!(&decoded[decoded.len() - 1..], b"a");
    }

    /// `clip_to_uart_budget` never splits a character: back off to the previous
    /// char boundary, clip ASCII exactly, and leave short text alone.
    #[test]
    fn test_clip_to_uart_budget_backs_off_to_char_boundary() {
        // 71 ASCII bytes, then a 2-byte char starting at PANIC_LOG_BODY_MAX - 1.
        let mut straddling = String::new();
        straddling.push_str(&"a".repeat(PANIC_LOG_BODY_MAX - 1));
        straddling.push('é');
        straddling.push_str("tail");

        let clipped = clip_to_uart_budget(&straddling);
        assert_eq!(clipped.len(), PANIC_LOG_BODY_MAX - 1);
        assert_eq!(clipped, "a".repeat(PANIC_LOG_BODY_MAX - 1));
        // The returned slice is a valid `&str` by construction; prove it decodes.
        assert!(core::str::from_utf8(clipped.as_bytes()).is_ok());

        let ascii = "b".repeat(PANIC_LOG_BODY_MAX + 10);
        assert_eq!(clip_to_uart_budget(&ascii).len(), PANIC_LOG_BODY_MAX);

        assert_eq!(clip_to_uart_budget("short"), "short");
    }

    #[test]
    fn test_write_panic_bytes() {
        FakeFlash::reset();

        let test_data = b"Panic occurred!";

        write_panic_bytes(test_data);

        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_PANIC_INFO),
            1,
            "the panic sector must be erased once before the record is written"
        );

        let mut flag = [0u8; 1];
        FakeFlash::read_slice(FLASH_ADR_PANIC_INFO, &mut flag);
        assert_eq!(flag[0], PANIC_VALID_FLAG, "the valid flag is written last");

        let mut len_bytes = [0u8; 2];
        FakeFlash::read_slice(FLASH_ADR_PANIC_INFO + 1, &mut len_bytes);
        assert_eq!(
            u16::from_le_bytes(len_bytes),
            test_data.len() as u16,
            "the length word sits at +1"
        );

        let mut body = [0u8; 32];
        FakeFlash::read_slice(FLASH_ADR_PANIC_INFO + 3, &mut body);
        assert_eq!(&body[..test_data.len()], test_data, "the body sits at +3");

        // Only `len` body bytes were programmed; the rest of the erased sector
        // must still read as 0xFF.
        let mut tail = [0u8; 1];
        FakeFlash::read_slice(FLASH_ADR_PANIC_INFO + 3 + test_data.len() as u32, &mut tail);
        assert_eq!(tail[0], 0xFF);
    }

    #[test]
    fn test_read_panic_report_no_valid_flag() {
        FakeFlash::reset();

        let mut buf = [0u8; 256];

        // A blank sector reads 0xFF, which is not a valid panic flag.
        assert_eq!(read_panic_report(&mut buf), Ok(None));

        // Nothing was reported, so the sector must not be cleared.
        assert_eq!(FakeFlash::erase_count(FLASH_ADR_PANIC_INFO), 0);
    }

    #[test]
    fn test_read_panic_report_with_valid_data() {
        FakeFlash::reset();

        let test_message = b"Test panic message";
        seed_panic_info(test_message, test_message.len() as u16);

        let mut buf = [0u8; 256];
        let len = read_panic_report(&mut buf)
            .expect("the flash read must succeed")
            .expect("the seeded record is valid");

        assert_eq!(len, test_message.len());
        assert_eq!(&buf[..len], test_message);
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_PANIC_INFO),
            0,
            "reading the report must not erase the sector"
        );

        clear_panic_report();

        // The report ends by erasing the sector, so the flag is back to blank.
        assert_eq!(FakeFlash::erase_count(FLASH_ADR_PANIC_INFO), 1);
        let mut flag = [0u8; 1];
        FakeFlash::read_slice(FLASH_ADR_PANIC_INFO, &mut flag);
        assert_eq!(flag[0], 0xFF, "the panic sector is cleared after reporting");
    }

    /// A length word beyond the 256-byte record is rejected instead of reading
    /// past `out`, and the corrupt record is erased.
    #[test]
    fn test_read_panic_report_length_over_limit() {
        FakeFlash::reset();

        seed_panic_info(&[b'x'; 8], 257);

        let mut buf = [0u8; 256];
        assert_eq!(read_panic_report(&mut buf), Ok(None));
        assert_eq!(buf, [0u8; 256]);
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_PANIC_INFO),
            1,
            "a corrupt length word must erase the panic sector so it is not retried"
        );
    }

    /// A zero length word is not a record any writer produces: reject and erase,
    /// without touching the caller's buffer.
    #[test]
    fn test_read_panic_report_zero_length() {
        FakeFlash::reset();

        seed_panic_info(b"ignored body", 0);

        let mut buf = [0xAAu8; 256];
        assert_eq!(read_panic_report(&mut buf), Ok(None));
        assert_eq!(buf, [0xAAu8; 256], "the caller's buffer is left untouched");
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_PANIC_INFO),
            1,
            "a zero length word must erase the panic sector"
        );
    }

    /// The all-ones length word of a half-erased sector is rejected with a single
    /// erase and no body read into `out`.
    #[test]
    fn test_read_panic_report_length_max_u16() {
        FakeFlash::reset();

        seed_panic_info(b"stale tail", 0xFFFF);

        let mut buf = [0x55u8; 256];
        assert_eq!(read_panic_report(&mut buf), Ok(None));
        assert_eq!(
            buf, [0x55u8; 256],
            "a rejected length must not overwrite the 256-byte buffer"
        );
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_PANIC_INFO),
            1,
            "the oversized length word must erase the panic sector once"
        );
    }

    #[test]
    fn test_record_panic_sentinel() {
        FakeAnalog::reset();

        record_panic_sentinel();

        assert_eq!(FakeAnalog::get(REGA_RESET_SENTINEL), SENTINEL_PANIC);
        assert_eq!(FakeAnalog::get(REGA_BREADCRUMB), CRUMB_PANIC);
    }
}
