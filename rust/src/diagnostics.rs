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
//! See `docs/plans/designs/005-rtt-target-hooks.md` (sections 4.1-4.5) for the
//! contract with the host tools.

use core::ptr::addr_of_mut;

use crate::config::{FLASH_ADR_PANIC_INFO, PANIC_VALID_FLAG};
use crate::sdk::drivers::flash::flash_read_page;
use crate::sdk::mcu::analog::{analog_read, analog_write};

/// Analog register holding why the previous boot ended.
pub const REGA_RESET_SENTINEL: u8 = 0x3b;
/// Analog register holding the last breadcrumb written before the reset.
pub const REGA_BREADCRUMB: u8 = 0x3c;

/// The previous boot ended cleanly (power-on or software reboot).
pub const SENTINEL_CLEAN_REBOOT: u8 = 0x00;
/// The firmware is running; a watchdog reset leaves this value behind.
pub const SENTINEL_RUNNING: u8 = 0xA5;
/// The firmware panicked; `FLASH_ADR_PANIC_INFO` holds the message.
pub const SENTINEL_PANIC: u8 = 0xE1;

pub const CRUMB_BOOT: u8 = 1;
pub const CRUMB_INIT_DONE: u8 = 2;
pub const CRUMB_MAIN_LOOP: u8 = 3;
/// Written by the panic handler, which only exists on the target build.
#[allow(dead_code)]
pub const CRUMB_PANIC: u8 = 0xFF;

/// The UART stream behind `uprintln!` holds 128 bytes and panics on overflow, so
/// a panic message read from flash (up to 256 bytes) must be clipped first: a
/// panic inside the boot report would reboot into the same panic forever.
const PANIC_LOG_BODY_MAX: usize = 72;

/// Only the first `PANIC_LOG_BODY_MAX` bytes are ever logged, so the flash read
/// buffer is bounded by that (plus UTF-8 slack) — a 256 B buffer cost 180+ B of
/// main-stack margin for bytes that are never printed.
///
/// The 4 slack bytes let a character that straddles the clip point decode
/// whole, so the cut is decided by the clip (a visible char boundary) rather
/// than by the read bound; they are never printed themselves.
const PANIC_READ_MAX: usize = PANIC_LOG_BODY_MAX + 4;

/// Marker appended to a panic body that was clipped to fit the UART stream.
const PANIC_LOG_TRUNCATED: &str = "...";

/// Records how far this boot got, so the next boot can report where it died.
pub fn record_breadcrumb(tag: u8) {
    analog_write(REGA_BREADCRUMB, tag);
}

/// Boot-time diagnostics: start the RTT sink (when compiled in) and report why
/// the previous boot ended. Runs before the version banner so a `[PANIC]` line
/// is the first thing the host sees, while the panic sector is still intact.
pub fn boot_init() {
    #[cfg(feature = "rtt")]
    rtt::rtt_init();

    report_reset_reason();
}

/// Reports the previous boot's reset reason from the retained sentinel, then
/// marks this boot as running.
pub fn report_reset_reason() {
    let prev_sentinel = analog_read(REGA_RESET_SENTINEL);
    let prev_crumb = analog_read(REGA_BREADCRUMB);

    match prev_sentinel {
        SENTINEL_RUNNING => {
            crate::uprintln!("[WD_RESET] sentinel=0xa5 breadcrumb={}", prev_crumb);
        }
        SENTINEL_PANIC => log_panic_info(prev_crumb),
        _ => {}
    }

    // This boot is alive from here on: a later watchdog reset inherits 0xA5.
    analog_write(REGA_RESET_SENTINEL, SENTINEL_RUNNING);
    record_breadcrumb(CRUMB_BOOT);
}

/// Streams the panic message left in flash by the panic handler. The sector is
/// left untouched: `check_panic_info()` still owns reading and erasing it.
fn log_panic_info(crumb: u8) {
    let mut flag: u8 = 0;
    flash_read_page(FLASH_ADR_PANIC_INFO, 1, addr_of_mut!(flag));
    if flag != PANIC_VALID_FLAG {
        return;
    }

    let mut len_bytes: [u8; 2] = [0; 2];
    flash_read_page(FLASH_ADR_PANIC_INFO + 1, 2, len_bytes.as_mut_ptr());
    let len = u16::from_le_bytes(len_bytes) as usize;
    if len > 256 {
        return;
    }

    // Only the logged prefix is read, and `len` is checked against the 256-byte
    // sector record above.
    let read = len.min(PANIC_READ_MAX);
    let mut message: [u8; PANIC_READ_MAX] = [0; PANIC_READ_MAX];
    flash_read_page(FLASH_ADR_PANIC_INFO + 3, read as u32, message.as_mut_ptr());

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
    let ellipsis = if len > read || text.len() > body.len() {
        PANIC_LOG_TRUNCATED
    } else {
        ""
    };

    crate::uprintln!(
        "[PANIC] sentinel=0xe1 breadcrumb={} msg={}{}",
        crumb,
        body,
        ellipsis
    );
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

#[cfg(any(feature = "rtt", test))]
pub mod rtt {
    //! Fixed-layout byte ring shared with the SWire host tool.

    use core::sync::atomic::{AtomicU16, Ordering};

    /// Capacity of the data area; ring indices are taken modulo this.
    pub const RTT_CAP: usize = 256;
    const FLAG_OVERFLOW: u16 = 1;

    /// Control block layout, consumed by `probe-tlsr rtt`:
    /// `magic` +0, `cap` +8, `wr` +10, `rd` +12, `flags` +14, `buf` +16.
    ///
    /// `magic`, `cap`, `flags` and `buf` are read by the host tool rather than by
    /// firmware code, so the compiler sees them as write-only.
    #[repr(C)]
    pub struct RttControlBlock {
        magic: [u8; 8],
        cap: u16,
        wr: AtomicU16,
        /// Advanced by the host as it drains; firmware only ever reads it.
        rd: AtomicU16,
        /// Bit 0 = overflow has occurred. Sticky: firmware only sets it and the
        /// set is a plain read-modify-write, so the host must treat it read-only.
        flags: u16,
        buf: [u8; RTT_CAP],
    }

    impl RttControlBlock {
        /// Fully zero-initialised, so the static lives in `.bss` and costs no
        /// flash image bytes. `cap == 0` means "not initialised yet" to a reader.
        pub const fn zeroed() -> Self {
            Self {
                magic: [0; 8],
                cap: 0,
                wr: AtomicU16::new(0),
                rd: AtomicU16::new(0),
                flags: 0,
                buf: [0; RTT_CAP],
            }
        }

        /// Advertises the control block to the host. Runs a few ms into boot,
        /// long before anything could be streaming.
        pub fn init(&mut self) {
            self.magic = *b"TLSR_RTT";
            self.cap = RTT_CAP as u16;
        }

        /// Copies as many bytes as fit into the ring, then drops the rest.
        ///
        /// Never blocks, never panics, never grows: a full ring sets the overflow
        /// flag and the tail of `bytes` is lost. Corrupt indices (a host writing
        /// `rd` out of range, or uninitialised state) are treated as 0. `rd` is
        /// sampled once per call, so a host that drains mid-write can trip a
        /// spurious overflow flag — staleness only ever costs log bytes, it can
        /// never make the writer overwrite unread data.
        pub fn write(&mut self, bytes: &[u8]) {
            let mut wr = self.wr.load(Ordering::Relaxed) as usize;
            if wr >= RTT_CAP {
                wr = 0;
            }

            let mut rd = self.rd.load(Ordering::Relaxed) as usize;
            if rd >= RTT_CAP {
                rd = 0;
            }

            for &byte in bytes {
                let next = (wr + 1) % RTT_CAP;

                // The host has not consumed the slot we would overwrite.
                if next == rd {
                    self.flags |= FLAG_OVERFLOW;
                    break;
                }

                // SAFETY: `wr` is always a valid index (it is reduced modulo
                // RTT_CAP above, and a corrupt value is reset to 0). A checked
                // store would put `panic_bounds_check` on the panic handler's
                // call graph => rejected by the static stack gate.
                unsafe { *self.buf.get_unchecked_mut(wr) = byte };
                wr = next;
                self.wr.store(wr as u16, Ordering::Relaxed);
            }
        }
    }

    /// Host-tool contract: this exact symbol name and layout are consumed by
    /// `probe-tlsr rtt`, so it may not be renamed.
    #[no_mangle]
    #[allow(non_upper_case_globals)]
    pub static mut _TLSR_RTT: RttControlBlock = RttControlBlock::zeroed();

    #[cfg(feature = "rtt")]
    pub fn rtt_init() {
        critical_section::with(|_| unsafe {
            let control_block = &mut *core::ptr::addr_of_mut!(_TLSR_RTT);
            control_block.init();
        });
    }

    #[cfg(feature = "rtt")]
    pub fn rtt_write_bytes(bytes: &[u8]) {
        critical_section::with(|_| unsafe {
            let control_block = &mut *core::ptr::addr_of_mut!(_TLSR_RTT);
            control_block.write(bytes);
        });
    }

    #[cfg(test)]
    #[coverage(off)]
    mod tests {
        use core::mem::{offset_of, size_of};

        use super::*;

        #[test]
        fn test_rtt_control_block_layout() {
            assert_eq!(size_of::<RttControlBlock>(), 272);
            assert_eq!(offset_of!(RttControlBlock, magic), 0);
            assert_eq!(offset_of!(RttControlBlock, cap), 8);
            assert_eq!(offset_of!(RttControlBlock, wr), 10);
            assert_eq!(offset_of!(RttControlBlock, rd), 12);
            assert_eq!(offset_of!(RttControlBlock, flags), 14);
            assert_eq!(offset_of!(RttControlBlock, buf), 16);
        }

        #[test]
        fn test_rtt_zeroed_is_all_zero() {
            let control_block = RttControlBlock::zeroed();

            assert_eq!(control_block.cap, 0);
            assert_eq!(control_block.wr.load(Ordering::Relaxed), 0);
            assert_eq!(control_block.rd.load(Ordering::Relaxed), 0);
            assert_eq!(control_block.flags, 0);
            assert!(control_block.buf.iter().all(|&byte| byte == 0));
        }

        #[test]
        fn test_rtt_write_and_drain() {
            let mut control_block = RttControlBlock::zeroed();
            control_block.init();

            control_block.write(b"hello");

            assert_eq!(&control_block.magic, b"TLSR_RTT");
            assert_eq!(control_block.cap, RTT_CAP as u16);
            assert_eq!(control_block.wr.load(Ordering::Relaxed), 5);
            assert_eq!(control_block.rd.load(Ordering::Relaxed), 0);
            assert_eq!(&control_block.buf[..5], b"hello");
            assert_eq!(control_block.flags, 0);
        }

        #[test]
        fn test_rtt_wraparound() {
            let mut control_block = RttControlBlock::zeroed();
            control_block.init();

            // The host drained everything up to 200, so both indices point at 200.
            control_block.wr.store(200, Ordering::Relaxed);
            control_block.rd.store(200, Ordering::Relaxed);

            let data: [u8; 100] = core::array::from_fn(|i| i as u8);
            control_block.write(&data);

            assert_eq!(control_block.wr.load(Ordering::Relaxed), 44);
            assert_eq!(control_block.flags, 0);
            assert_eq!(control_block.buf[255], data[55]);
            for i in 0..44 {
                assert_eq!(control_block.buf[i], data[56 + i]);
            }
        }

        #[test]
        fn test_rtt_overflow_drops_tail() {
            let mut control_block = RttControlBlock::zeroed();
            control_block.init();

            let data: [u8; 300] = core::array::from_fn(|i| i as u8);
            control_block.write(&data);

            // One slot is always kept free to tell "full" from "empty".
            assert_eq!(control_block.wr.load(Ordering::Relaxed), 255);
            assert_eq!(control_block.flags & FLAG_OVERFLOW, FLAG_OVERFLOW);
            assert_eq!(&control_block.buf[..255], &data[..255]);
        }

        #[test]
        fn test_rtt_corrupt_indices_treated_as_empty() {
            let mut control_block = RttControlBlock::zeroed();
            control_block.init();

            control_block.wr.store(0xFFFF, Ordering::Relaxed);
            control_block.rd.store(0xFFFF, Ordering::Relaxed);

            control_block.write(b"x");

            assert_eq!(control_block.wr.load(Ordering::Relaxed), 1);
            assert_eq!(control_block.buf[0], b'x');
        }
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::sdk::drivers::flash::{
        flash_erase_sector, mock_flash_erase_sector, mock_flash_read_page,
    };
    use crate::sdk::drivers::uart::{UartData, UART_DATA_LEN};
    use crate::sdk::mcu::analog::{mock_analog_read, mock_analog_write};
    use crate::uart_manager::UartMsg;
    use crate::{app_mocker, mock_app_mocker, App};
    use mry::{send_wrapper::SendWrapper, Any};

    /// Payload bytes per UART packet (`UART_PAYLOAD_MAX_SIZE` in `compat.rs`).
    const PAYLOAD_CHUNK: usize = 39;

    fn expected_print(bytes: &[u8]) -> UartData {
        let mut message = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };

        message.data[2] = UartMsg::PrintMessage as u8;
        message.data[3..3 + bytes.len()].copy_from_slice(bytes);

        message
    }

    /// Asserts the mocked UART queue received exactly `text`, chunked the way
    /// `UartStream::send` packets it.
    fn assert_uart_print(text: &[u8]) {
        for chunk in text.chunks(PAYLOAD_CHUNK) {
            crate::app()
                .uart_manager
                .mock_send_message(expected_print(chunk))
                .assert_called(1);
        }

        crate::app()
            .uart_manager
            .mock_send_message(Any)
            .assert_called(text.len().div_ceil(PAYLOAD_CHUNK));
    }

    fn mock_app_with_uart(app: &mut App) {
        app.uart_manager.mock_send_message(Any).returns(Ok(()));
        mock_app_mocker().returns(app);
    }

    #[test]
    #[mry::lock(app_mocker, analog_read, analog_write)]
    fn test_boot_sentinel_clean_reboot() {
        let mut app = App::default();
        mock_app_with_uart(&mut app);

        mock_analog_read(REGA_RESET_SENTINEL).returns(SENTINEL_CLEAN_REBOOT);
        mock_analog_read(REGA_BREADCRUMB).returns(CRUMB_MAIN_LOOP);
        mock_analog_write(Any, Any).returns(());

        report_reset_reason();

        crate::app()
            .uart_manager
            .mock_send_message(Any)
            .assert_called(0);
        mock_analog_write(REGA_RESET_SENTINEL, SENTINEL_RUNNING).assert_called(1);
        mock_analog_write(REGA_BREADCRUMB, CRUMB_BOOT).assert_called(1);
    }

    #[test]
    #[mry::lock(app_mocker, analog_read, analog_write)]
    fn test_boot_sentinel_wd_reset() {
        let mut app = App::default();
        mock_app_with_uart(&mut app);

        mock_analog_read(REGA_RESET_SENTINEL).returns(SENTINEL_RUNNING);
        mock_analog_read(REGA_BREADCRUMB).returns(CRUMB_MAIN_LOOP);
        mock_analog_write(Any, Any).returns(());

        report_reset_reason();

        assert_uart_print(b"[WD_RESET] sentinel=0xa5 breadcrumb=3\n");
        mock_analog_write(REGA_RESET_SENTINEL, SENTINEL_RUNNING).assert_called(1);
        mock_analog_write(REGA_BREADCRUMB, CRUMB_BOOT).assert_called(1);
    }

    /// `boot_init` is the hook `App::run` calls: it must run the reset report
    /// (and, when compiled in, initialise the ring) without touching flash.
    #[test]
    #[mry::lock(app_mocker, analog_read, analog_write)]
    fn test_boot_init_reports_reset_reason() {
        let mut app = App::default();
        mock_app_with_uart(&mut app);

        mock_analog_read(REGA_RESET_SENTINEL).returns(SENTINEL_RUNNING);
        mock_analog_read(REGA_BREADCRUMB).returns(CRUMB_INIT_DONE);
        mock_analog_write(Any, Any).returns(());

        boot_init();

        assert_uart_print(b"[WD_RESET] sentinel=0xa5 breadcrumb=2\n");
        mock_analog_write(REGA_RESET_SENTINEL, SENTINEL_RUNNING).assert_called(1);
        mock_analog_write(REGA_BREADCRUMB, CRUMB_BOOT).assert_called(1);
    }

    #[test]
    #[mry::lock(
        app_mocker,
        analog_read,
        analog_write,
        flash_read_page,
        flash_erase_sector
    )]
    fn test_boot_sentinel_panic_message() {
        let mut app = App::default();
        mock_app_with_uart(&mut app);

        let body = b"panic body";
        let len = body.len() as u16;

        mock_analog_read(REGA_RESET_SENTINEL).returns(SENTINEL_PANIC);
        mock_analog_read(REGA_BREADCRUMB).returns(CRUMB_PANIC);
        mock_analog_write(Any, Any).returns(());

        mock_flash_read_page(FLASH_ADR_PANIC_INFO, 1, Any).returns_with(
            |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::write_bytes(*buf, PANIC_VALID_FLAG, 1);
            },
        );
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 1, 2, Any).returns_with(
            move |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::copy_nonoverlapping(len.to_le_bytes().as_ptr(), *buf, 2);
            },
        );
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 3, len as u32, Any).returns_with(
            move |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::copy_nonoverlapping(body.as_ptr(), *buf, body.len());
            },
        );

        report_reset_reason();

        assert_uart_print(b"[PANIC] sentinel=0xe1 breadcrumb=255 msg=panic body\n");
        mock_analog_write(REGA_RESET_SENTINEL, SENTINEL_RUNNING).assert_called(1);
        mock_analog_write(REGA_BREADCRUMB, CRUMB_BOOT).assert_called(1);
        // `check_panic_info()` owns erasing the sector, not the boot report.
        mock_flash_erase_sector(Any).assert_called(0);
    }

    /// A panic body longer than the stream budget is clipped and marked, and the
    /// whole line still fits the 128-byte `UartStream` used by `uprintln!`.
    #[test]
    #[mry::lock(app_mocker, analog_read, analog_write, flash_read_page)]
    fn test_boot_panic_message_truncated() {
        let mut app = App::default();
        mock_app_with_uart(&mut app);

        let body = [b'x'; 200];
        let len = body.len() as u16;

        mock_analog_read(REGA_RESET_SENTINEL).returns(SENTINEL_PANIC);
        mock_analog_read(REGA_BREADCRUMB).returns(CRUMB_PANIC);
        mock_analog_write(Any, Any).returns(());

        mock_flash_read_page(FLASH_ADR_PANIC_INFO, 1, Any).returns_with(
            |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::write_bytes(*buf, PANIC_VALID_FLAG, 1);
            },
        );
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 1, 2, Any).returns_with(
            move |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::copy_nonoverlapping(len.to_le_bytes().as_ptr(), *buf, 2);
            },
        );
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 3, PANIC_READ_MAX as u32, Any).returns_with(
            move |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::copy_nonoverlapping(body.as_ptr(), *buf, PANIC_READ_MAX);
            },
        );

        let mut expected = Vec::new();
        expected.extend_from_slice(b"[PANIC] sentinel=0xe1 breadcrumb=255 msg=");
        expected.extend_from_slice(&body[..PANIC_LOG_BODY_MAX]);
        expected.extend_from_slice(b"...\n");

        // Worst case: crumb 255 and a full-length body. The stream is 128 bytes,
        // and the fixed prefix alone spans more than one 39-byte payload, so the
        // clipped line is packetised, never truncated by the stream itself.
        assert_eq!(expected.len(), 117);
        assert!(expected.len() <= 128);
        assert_eq!(&expected[expected.len() - 4..], b"...\n");
        assert_eq!(expected.len().div_ceil(PAYLOAD_CHUNK), 3);

        report_reset_reason();

        assert_uart_print(&expected);
    }

    /// A message longer than the flash read window is marked with `...` even
    /// when the bytes that *were* read decode to exactly the 72-byte budget, so
    /// a cut can never look complete.
    #[test]
    #[mry::lock(app_mocker, analog_read, analog_write, flash_read_page)]
    fn test_boot_panic_message_marks_short_body_cut_by_read() {
        let mut app = App::default();
        mock_app_with_uart(&mut app);

        // 72 printable bytes followed by a byte that is invalid UTF-8, then more
        // data beyond the read window: the decoded prefix is exactly the budget.
        let mut body = [b'a'; 200];
        body[PANIC_LOG_BODY_MAX] = 0xFF;
        let len = body.len() as u16;

        mock_analog_read(REGA_RESET_SENTINEL).returns(SENTINEL_PANIC);
        mock_analog_read(REGA_BREADCRUMB).returns(CRUMB_PANIC);
        mock_analog_write(Any, Any).returns(());

        mock_flash_read_page(FLASH_ADR_PANIC_INFO, 1, Any).returns_with(
            |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::write_bytes(*buf, PANIC_VALID_FLAG, 1);
            },
        );
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 1, 2, Any).returns_with(
            move |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::copy_nonoverlapping(len.to_le_bytes().as_ptr(), *buf, 2);
            },
        );
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 3, PANIC_READ_MAX as u32, Any).returns_with(
            move |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::copy_nonoverlapping(body.as_ptr(), *buf, PANIC_READ_MAX);
            },
        );

        let mut expected = Vec::new();
        expected.extend_from_slice(b"[PANIC] sentinel=0xe1 breadcrumb=255 msg=");
        expected.extend_from_slice(&body[..PANIC_LOG_BODY_MAX]);
        expected.extend_from_slice(b"...\n");

        assert_eq!(expected.len(), 117);

        report_reset_reason();

        assert_uart_print(&expected);
    }

    /// A message that fits the flash read window exactly but not the 72-byte
    /// stream budget is still marked, i.e. the clip alone can require `...`.
    #[test]
    #[mry::lock(app_mocker, analog_read, analog_write, flash_read_page)]
    fn test_boot_panic_message_marks_body_clipped_within_read() {
        let mut app = App::default();
        mock_app_with_uart(&mut app);

        let body = [b'w'; PANIC_READ_MAX];
        let len = body.len() as u16;

        mock_analog_read(REGA_RESET_SENTINEL).returns(SENTINEL_PANIC);
        mock_analog_read(REGA_BREADCRUMB).returns(CRUMB_PANIC);
        mock_analog_write(Any, Any).returns(());

        mock_flash_read_page(FLASH_ADR_PANIC_INFO, 1, Any).returns_with(
            |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::write_bytes(*buf, PANIC_VALID_FLAG, 1);
            },
        );
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 1, 2, Any).returns_with(
            move |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::copy_nonoverlapping(len.to_le_bytes().as_ptr(), *buf, 2);
            },
        );
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 3, PANIC_READ_MAX as u32, Any).returns_with(
            move |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::copy_nonoverlapping(body.as_ptr(), *buf, PANIC_READ_MAX);
            },
        );

        let mut expected = Vec::new();
        expected.extend_from_slice(b"[PANIC] sentinel=0xe1 breadcrumb=255 msg=");
        expected.extend_from_slice(&body[..PANIC_LOG_BODY_MAX]);
        expected.extend_from_slice(b"...\n");

        assert_eq!(expected.len(), 117);

        report_reset_reason();

        assert_uart_print(&expected);
    }

    /// AC4: an invalid panic flag means the sector is not trusted at all, so the
    /// boot report stays silent but still marks this boot as running.
    #[test]
    #[mry::lock(app_mocker, analog_read, analog_write, flash_read_page)]
    fn test_boot_sentinel_panic_invalid_flag() {
        let mut app = App::default();
        mock_app_with_uart(&mut app);

        mock_analog_read(REGA_RESET_SENTINEL).returns(SENTINEL_PANIC);
        mock_analog_read(REGA_BREADCRUMB).returns(CRUMB_MAIN_LOOP);
        mock_analog_write(Any, Any).returns(());

        mock_flash_read_page(FLASH_ADR_PANIC_INFO, 1, Any).returns_with(
            |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::write_bytes(*buf, PANIC_VALID_FLAG ^ 0xFF, 1);
            },
        );

        report_reset_reason();

        // No line at all, and the length/body were never read.
        crate::app()
            .uart_manager
            .mock_send_message(Any)
            .assert_called(0);
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 1, Any, Any).assert_called(0);
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 3, Any, Any).assert_called(0);
        mock_analog_write(REGA_RESET_SENTINEL, SENTINEL_RUNNING).assert_called(1);
        mock_analog_write(REGA_BREADCRUMB, CRUMB_BOOT).assert_called(1);
    }

    /// AC4: a length beyond the 256-byte sector record is rejected before any
    /// body bytes are read, and nothing is logged.
    #[test]
    #[mry::lock(app_mocker, analog_read, analog_write, flash_read_page)]
    fn test_boot_sentinel_panic_length_over_limit() {
        let mut app = App::default();
        mock_app_with_uart(&mut app);

        let len: u16 = 257;

        mock_analog_read(REGA_RESET_SENTINEL).returns(SENTINEL_PANIC);
        mock_analog_read(REGA_BREADCRUMB).returns(CRUMB_MAIN_LOOP);
        mock_analog_write(Any, Any).returns(());

        mock_flash_read_page(FLASH_ADR_PANIC_INFO, 1, Any).returns_with(
            |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::write_bytes(*buf, PANIC_VALID_FLAG, 1);
            },
        );
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 1, 2, Any).returns_with(
            move |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::copy_nonoverlapping(len.to_le_bytes().as_ptr(), *buf, 2);
            },
        );
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 3, Any, Any).returns(());

        report_reset_reason();

        crate::app()
            .uart_manager
            .mock_send_message(Any)
            .assert_called(0);
        // The over-limit length must short-circuit before the body read.
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 3, Any, Any).assert_called(0);
        mock_analog_write(REGA_RESET_SENTINEL, SENTINEL_RUNNING).assert_called(1);
        mock_analog_write(REGA_BREADCRUMB, CRUMB_BOOT).assert_called(1);
    }

    /// AC4: the bounded read can end inside a multi-byte character. The partial
    /// code point is dropped (`from_utf8` error clamp), never replaced and never
    /// logged raw, and the clipped line is marked with `...`.
    #[test]
    #[mry::lock(app_mocker, analog_read, analog_write, flash_read_page)]
    fn test_boot_sentinel_panic_truncated_utf8() {
        let mut app = App::default();
        mock_app_with_uart(&mut app);

        // 75 ASCII bytes, then `é` (0xC3 0xA9) starting at byte 75 so its second
        // byte is cut by the 76-byte read, then more data that is never read.
        let mut body = Vec::new();
        body.extend_from_slice(&[b'a'; PANIC_READ_MAX - 1]);
        body.extend_from_slice("é".as_bytes());
        body.extend_from_slice(&[b'z'; 20]);
        let len = body.len() as u16;

        // The clamp decodes 75 ASCII bytes, then `clip_to_uart_budget` keeps the
        // 72 the stream budget allows and the ellipsis marks the cut.
        let mut expected = Vec::new();
        expected.extend_from_slice(b"[PANIC] sentinel=0xe1 breadcrumb=255 msg=");
        expected.extend_from_slice(&body[..PANIC_LOG_BODY_MAX]);
        expected.extend_from_slice(b"...\n");

        assert_eq!(body[PANIC_READ_MAX - 1], 0xC3);
        assert_eq!(expected.len(), 117);

        mock_analog_read(REGA_RESET_SENTINEL).returns(SENTINEL_PANIC);
        mock_analog_read(REGA_BREADCRUMB).returns(CRUMB_PANIC);
        mock_analog_write(Any, Any).returns(());

        mock_flash_read_page(FLASH_ADR_PANIC_INFO, 1, Any).returns_with(
            |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::write_bytes(*buf, PANIC_VALID_FLAG, 1);
            },
        );
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 1, 2, Any).returns_with(
            move |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::copy_nonoverlapping(len.to_le_bytes().as_ptr(), *buf, 2);
            },
        );
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 3, PANIC_READ_MAX as u32, Any).returns_with(
            move |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::copy_nonoverlapping(body.as_ptr(), *buf, PANIC_READ_MAX);
            },
        );

        report_reset_reason();

        // Byte-exact: no replacement character (0xEF 0xBF 0xBD) and no stray
        // lead byte (0xC3) can appear in the logged line.
        assert_uart_print(&expected);
        assert!(!expected.windows(3).any(|w| w == [0xEF, 0xBF, 0xBD]));
        assert!(!expected.contains(&0xC3));
        assert_eq!(&expected[expected.len() - 5..], b"a...\n");
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
}
