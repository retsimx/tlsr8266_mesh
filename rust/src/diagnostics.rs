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

use core::cmp::min;
use core::ptr::addr_of_mut;
use core::slice;

use crate::config::{FLASH_ADR_PANIC_INFO, PANIC_VALID_FLAG};
use crate::sdk::light::LGT_PANIC_MSG;
use tlsr8266_hal::drivers::flash::{flash_erase_sector, flash_read_page};
use tlsr8266_hal::mcu::analog::{analog_read, analog_write};

// Panic-path imports: `write_panic_info`/`panic` exist only on the target build
// and in unit tests (which drive the writer), so the plain host library build
// never sees these.
#[cfg(any(target_os = "none", test))]
use crate::logging::UartStream;
#[cfg(target_os = "none")]
use core::fmt::Write;
#[cfg(target_os = "none")]
use core::panic::PanicInfo;
#[cfg(any(target_os = "none", test))]
use core::ptr::addr_of;
#[cfg(any(target_os = "none", test))]
use tlsr8266_hal::drivers::flash::flash_write_page;
#[cfg(target_os = "none")]
use tlsr8266_hal::pm::light_sw_reboot;

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
#[allow(dead_code)]
pub const CRUMB_PANIC: u8 = 0xFF;

/// The UART stream behind `uprintln!` holds 128 bytes and rejects a write that
/// would overflow it instead of panicking (the formatter then stops at the first
/// failed `write_str`, so an over-long log line is clipped rather than
/// rebooting). A panic message read from flash (up to 256 bytes) must still be
/// clipped first: a panic inside the boot report would reboot into the same
/// panic forever.
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

/// Persists the panic message for the next boot.
///
/// Sector layout — a contract with the host tools:
/// `valid_flag: u8 + 0`, `length: u16 + 1`, `data: [u8; 256] + 3`.
///
/// The valid flag is written last: a reset part-way through the erase/write
/// sequence then leaves a sector without a valid flag, so the next boot reports
/// nothing rather than a half-written message.
#[cfg(any(target_os = "none", test))]
fn write_panic_info<const N: usize>(info: &UartStream<N>) {
    let panic_addr = FLASH_ADR_PANIC_INFO;

    // Clear the panic info sector for writing just in case
    flash_erase_sector(panic_addr);

    // Panic section looks as follows
    // valid_flag: u8   + 0
    // length: u16      + 1
    // data: [u8; 256]  + 3

    // Write the length first
    let length = info.length as u16;
    flash_write_page(panic_addr + 1, 2, addr_of!(length) as *const u8);

    // Then write the panic message
    flash_write_page(panic_addr + 3, length as u32, info.out_buffer.as_ptr());

    // Finally write the panic valid flag last
    let flag = PANIC_VALID_FLAG;
    flash_write_page(panic_addr, 1, addr_of!(flag));
}

/// Sends any stored panic message to the mesh, then clears the sector.
///
/// Runs once per boot (`app.rs`): the body is chunked into 10-byte
/// `LGT_PANIC_MSG` payloads with a 100 ms gap so the radio keeps up, an empty
/// `LGT_PANIC_MSG` terminates the report, and only then is the sector erased.
pub async fn check_panic_info() {
    // Function-local so this does not shadow `std::vec::Vec` in the tests below.
    use heapless::Vec;

    let panic_addr = FLASH_ADR_PANIC_INFO;

    let mut flag: u8 = 0;
    flash_read_page(panic_addr, 1, addr_of_mut!(flag));

    // If the panic info flag isn't set, there's nothing to do
    if flag != PANIC_VALID_FLAG {
        return;
    }

    // There is panic info, send it in to the mesh

    // Read the length
    let mut length: u16 = 0;
    flash_read_page(panic_addr + 1, 2, addr_of_mut!(length) as *mut u8);

    // Read the message
    let mut message: [u8; 256] = [0; 256];
    flash_read_page(panic_addr + 3, length as u32, message.as_mut_ptr());

    // SAFETY: (4) audited panic-report wire view. `message` is a live 256-byte
    // array and `length` is the sector's length word, written by `write_panic_info`
    // from a `UartStream<256>`, so `length <= 256 == message.len()`.
    let mut buffer = unsafe { slice::from_raw_parts(message.as_ptr(), length as usize) };
    while !buffer.is_empty() {
        // Wait a moment to send the next message
        #[cfg(not(test))]
        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;

        let len = min(10, buffer.len());

        let mut data = [0u8; 13];
        data[0] = LGT_PANIC_MSG;
        data[3..3 + len].copy_from_slice(&buffer[0..len]);

        crate::mesh::send_mesh_message(&Vec::from_slice(&data).unwrap(), 0xffff, 3, false);

        // SAFETY: (4) audited panic-report wire view. `len <= buffer.len()`, so the
        // advanced pointer stays inside the same `message` allocation and the new
        // length remains in bounds; these are the same persisted wire bytes.
        buffer = unsafe { slice::from_raw_parts(buffer.as_ptr().add(len), buffer.len() - len) }
    }

    // Wait a moment to send the next message
    #[cfg(not(test))]
    embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;

    // Send an empty panic message to indicate the end of the message
    let mut data = [0u8; 13];
    data[0] = LGT_PANIC_MSG;

    crate::mesh::send_mesh_message(&Vec::from_slice(&data).unwrap(), 0xffff, 3, false);

    // Finally clear the panic info
    flash_erase_sector(panic_addr);
}

/// Panic handler: mark the reset, persist the message, reboot.
///
/// This is the emergency path, so the ordering is the contract: the retained
/// sentinel and breadcrumb are written *before* any formatting, because
/// formatting the panic info into the 256-byte stream is work that can itself
/// fail and a double panic must not cost the next boot its breadcrumb. The
/// message then goes out on the blocking sink (never the async channel) and to
/// flash, and only then does the device reboot.
#[cfg(target_os = "none")]
#[panic_handler]
pub fn panic(info: &PanicInfo) -> ! {
    critical_section::with(|_| {
        // Mark the panic before anything that can itself fail.
        analog_write(REGA_RESET_SENTINEL, SENTINEL_PANIC);
        analog_write(REGA_BREADCRUMB, CRUMB_PANIC);

        let mut stream = UartStream::<256>::new();

        let _ = write!(stream, "{}", info);

        stream.send_blocking(true);
        write_panic_info(&stream);

        light_sw_reboot();
    });

    loop {}
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

                // SAFETY: (5) panic/RTT call graph. `wr` is a valid index: it is
                // reduced modulo `RTT_CAP` above and a corrupt value is reset to
                // 0, so `wr < RTT_CAP`. A checked store would put
                // `panic_bounds_check` on the panic handler's call graph and the
                // static stack gate rejects it.
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
    pub static _TLSR_RTT: critical_section::Mutex<core::cell::UnsafeCell<RttControlBlock>> =
        critical_section::Mutex::new(core::cell::UnsafeCell::new(RttControlBlock::zeroed()));

    // The host tool reads the control block straight from the symbol address, so
    // the mutex wrapper must not add a tag byte or any padding in front of it.
    const _: () = assert!(
        core::mem::size_of::<critical_section::Mutex<core::cell::UnsafeCell<RttControlBlock>>>()
            == core::mem::size_of::<RttControlBlock>()
    );
    // `probe-tlsr rtt` hard-codes the sampled size of the ring.
    const _: () = assert!(core::mem::size_of::<RttControlBlock>() == 272);

    #[cfg(feature = "rtt")]
    pub fn rtt_init() {
        critical_section::with(|cs| {
            // SAFETY: (4) audited `Sync`-wrapped static. `borrow` yields the
            // control-block cell for exactly this critical section: the cell is a
            // `static` (never null, aligned for `RttControlBlock`) and interrupts
            // are masked for the whole closure, so no other context can hold a
            // reference to it. The dereference cannot be made safe because the
            // mutex deliberately exposes no `&mut T`.
            let control_block = unsafe { &mut *_TLSR_RTT.borrow(cs).get() };
            control_block.init();
        });
    }

    #[cfg(feature = "rtt")]
    pub fn rtt_write_bytes(bytes: &[u8]) {
        critical_section::with(|cs| {
            // SAFETY: (4) as in `rtt_init`: the critical section makes this the
            // only accessor of the `static` cell for the duration of the call.
            let control_block = unsafe { &mut *_TLSR_RTT.borrow(cs).get() };
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
    use crate::mesh::{mock_send_mesh_message, send_mesh_message};
    use crate::uart_manager::{mock_send_message, reset_for_test, send_message};
    use mesh_protocol::uart::{UartData, UartMsg, UART_DATA_LEN};
    use mry::Any;
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog;
    use tlsr8266_pac::fake_mmio::FakeMmio;

    /// Payload bytes per UART packet (`UART_PAYLOAD_MAX_SIZE` in
    /// `mesh_protocol::uart`).
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
            mock_send_message(expected_print(chunk)).assert_called(1);
        }

        mock_send_message(Any).assert_called(text.len().div_ceil(PAYLOAD_CHUNK));
    }

    /// Clears the module statics and the HAL host seams, then mocks the UART
    /// sink, so a log line lands in the mock instead of the real 12-slot
    /// channel. `FakeAnalog` is a process-wide singleton, so every test in this
    /// module starts from a zeroed analog bank.
    fn mock_uart_print() {
        reset_for_test();
        FakeMmio::reset();
        FakeAnalog::reset();
        FakeFlash::reset();
        mock_send_message(Any).returns(Ok(()));
    }

    /// Seed the panic-info sector with the layout `write_panic_info` produces:
    /// valid flag at `+0`, `len` at `+1`, `body` at `+3`.
    fn seed_panic_info(body: &[u8], len: u16) {
        FakeFlash::write_slice(FLASH_ADR_PANIC_INFO, &[PANIC_VALID_FLAG]);
        FakeFlash::write_slice(FLASH_ADR_PANIC_INFO + 1, &len.to_le_bytes());
        FakeFlash::write_slice(FLASH_ADR_PANIC_INFO + 3, body);
    }

    #[test]
    #[mry::lock(send_message)]
    fn test_boot_sentinel_clean_reboot() {
        mock_uart_print();

        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_CLEAN_REBOOT);
        FakeAnalog::set(REGA_BREADCRUMB, CRUMB_MAIN_LOOP);

        report_reset_reason();

        mock_send_message(Any).assert_called(0);
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
    #[mry::lock(send_message)]
    fn test_boot_sentinel_wd_reset() {
        mock_uart_print();

        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_RUNNING);
        FakeAnalog::set(REGA_BREADCRUMB, CRUMB_MAIN_LOOP);

        report_reset_reason();

        assert_uart_print(b"[WD_RESET] sentinel=0xa5 breadcrumb=3\n");
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

    /// `boot_init` is the hook `App::run` calls: it must run the reset report
    /// (and, when compiled in, initialise the ring) without touching flash.
    #[test]
    #[mry::lock(send_message)]
    fn test_boot_init_reports_reset_reason() {
        mock_uart_print();

        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_RUNNING);
        FakeAnalog::set(REGA_BREADCRUMB, CRUMB_INIT_DONE);

        boot_init();

        assert_uart_print(b"[WD_RESET] sentinel=0xa5 breadcrumb=2\n");
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
    #[mry::lock(send_message)]
    fn test_boot_sentinel_panic_message() {
        mock_uart_print();

        let body = b"panic body";

        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_PANIC);
        FakeAnalog::set(REGA_BREADCRUMB, CRUMB_PANIC);
        seed_panic_info(body, body.len() as u16);

        report_reset_reason();

        assert_uart_print(b"[PANIC] sentinel=0xe1 breadcrumb=255 msg=panic body\n");
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
        // `check_panic_info()` owns erasing the sector, not the boot report.
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_PANIC_INFO),
            0,
            "the boot report must leave the panic sector intact"
        );
    }

    /// A panic body longer than the stream budget is clipped and marked, and the
    /// whole line still fits the 128-byte `UartStream` used by `uprintln!`.
    #[test]
    #[mry::lock(send_message)]
    fn test_boot_panic_message_truncated() {
        mock_uart_print();

        let body = [b'x'; 200];

        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_PANIC);
        FakeAnalog::set(REGA_BREADCRUMB, CRUMB_PANIC);
        seed_panic_info(&body, body.len() as u16);

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
    #[mry::lock(send_message)]
    fn test_boot_panic_message_marks_short_body_cut_by_read() {
        mock_uart_print();

        // 72 printable bytes followed by a byte that is invalid UTF-8, then more
        // data beyond the read window: the decoded prefix is exactly the budget.
        let mut body = [b'a'; 200];
        body[PANIC_LOG_BODY_MAX] = 0xFF;

        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_PANIC);
        FakeAnalog::set(REGA_BREADCRUMB, CRUMB_PANIC);
        seed_panic_info(&body, body.len() as u16);

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
    #[mry::lock(send_message)]
    fn test_boot_panic_message_marks_body_clipped_within_read() {
        mock_uart_print();

        let body = [b'w'; PANIC_READ_MAX];

        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_PANIC);
        FakeAnalog::set(REGA_BREADCRUMB, CRUMB_PANIC);
        seed_panic_info(&body, body.len() as u16);

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
    #[mry::lock(send_message)]
    fn test_boot_sentinel_panic_invalid_flag() {
        mock_uart_print();

        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_PANIC);
        FakeAnalog::set(REGA_BREADCRUMB, CRUMB_MAIN_LOOP);
        FakeFlash::write_slice(FLASH_ADR_PANIC_INFO, &[PANIC_VALID_FLAG ^ 0xFF]);

        report_reset_reason();

        // No line at all: the invalid flag short-circuits before the length read.
        mock_send_message(Any).assert_called(0);
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

    /// AC4: a length beyond the 256-byte sector record is rejected before any
    /// body bytes are read, and nothing is logged.
    #[test]
    #[mry::lock(send_message)]
    fn test_boot_sentinel_panic_length_over_limit() {
        mock_uart_print();

        let len: u16 = 257;

        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_PANIC);
        FakeAnalog::set(REGA_BREADCRUMB, CRUMB_MAIN_LOOP);
        seed_panic_info(&[b'x'; 8], len);

        report_reset_reason();

        mock_send_message(Any).assert_called(0);
        // The over-limit length must short-circuit before the body read, so the
        // seeded body is never logged.
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

    /// AC4: the bounded read can end inside a multi-byte character. The partial
    /// code point is dropped (`from_utf8` error clamp), never replaced and never
    /// logged raw, and the clipped line is marked with `...`.
    #[test]
    #[mry::lock(send_message)]
    fn test_boot_sentinel_panic_truncated_utf8() {
        mock_uart_print();

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

        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_PANIC);
        FakeAnalog::set(REGA_BREADCRUMB, CRUMB_PANIC);
        seed_panic_info(&body, len);

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

    #[test]
    fn test_write_panic_info() {
        FakeFlash::reset();

        // Create a test stream with some panic data
        let mut stream = UartStream::<128>::new();
        let test_data = b"Panic occurred!";
        let _ = stream.write_all(test_data);

        write_panic_info(&stream);

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
    fn test_check_panic_info_no_valid_flag() {
        reset_for_test();
        // A blank sector reads 0xFF, which is not a valid panic flag.
        FakeFlash::reset();

        // Run check_panic_info (should return early)
        let future = check_panic_info();
        futures::executor::block_on(future);

        // Nothing was reported, so the sector must not be cleared.
        assert_eq!(FakeFlash::erase_count(FLASH_ADR_PANIC_INFO), 0);
    }

    #[test]
    #[mry::lock(send_mesh_message)]
    fn test_check_panic_info_with_valid_data() {
        reset_for_test();

        let test_message = b"Test panic message";
        let message_len = test_message.len() as u16;
        seed_panic_info(test_message, message_len);

        // Mock the mesh send so the report never touches the radio
        mock_send_mesh_message(Any, Any, Any, Any).returns([1, 2, 3]);

        // Run check_panic_info
        let future = check_panic_info();
        futures::executor::block_on(future);

        // Verify mesh messages were sent (3: two data chunks + one empty terminator)
        mock_send_mesh_message(Any, Any, Any, Any).assert_called(3);

        // The report ends by erasing the sector, so the flag is back to blank.
        assert_eq!(FakeFlash::erase_count(FLASH_ADR_PANIC_INFO), 1);
        let mut flag = [0u8; 1];
        FakeFlash::read_slice(FLASH_ADR_PANIC_INFO, &mut flag);
        assert_eq!(flag[0], 0xFF, "the panic sector is cleared after reporting");
    }
}
