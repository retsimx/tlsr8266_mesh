//! # UART Log Sink
//!
//! The application-layer `uprintln!` logging sink for the TLSR8266 mesh
//! lighting firmware.
//!
//! ## Why this lives outside the SDK
//!
//! Formatting a log line and pushing it at the UART manager is application
//! work: it needs the global `app()` singleton for the queue/driver and it is
//! reached from the panic handler. Keeping it here leaves `crate::sdk/` free of
//! upward imports.
//!
//! ## Key Responsibilities:
//! - `UartStream<const N: usize>`: a fixed-capacity byte sink implementing
//!   `core::fmt::Write`
//! - Chunked framing into 44-byte built-in-UART packets (`build_uart_frame`)
//! - The async (`send_async`) and panic-path (`send_blocking`) send sinks
//! - `uprintln!`, `print_to_uart` and `get_uart_stream`

use core::cmp::min;
use core::fmt::{self, Write};
use core::slice;

use crate::app;
use crate::sdk::drivers::uart::{UartData, UART_DATA_LEN};
use crate::uart_manager::UartMsg;

// UART packet structure constants
// Packet layout: [counter(1)][cmd_type(1)][cmd_code(1)][payload(39)][crc(2)] = 44 bytes total
pub(crate) const UART_PAYLOAD_OFFSET: usize = 3; // Payload starts at index 3
pub(crate) const UART_CRC_OFFSET: usize = 42; // CRC starts at index 42
const UART_PAYLOAD_MAX_SIZE: usize = UART_CRC_OFFSET - UART_PAYLOAD_OFFSET; // 39 bytes

// Define a function to get a new UartStream rather than using a static mutable
#[inline]
pub fn get_uart_stream() -> UartStream<128> {
    UartStream::new()
}

// Helper function to print and send data to the uart
#[inline]
pub fn print_to_uart<F>(doasync: bool, f: F)
where
    F: FnOnce(&mut UartStream<128>) -> fmt::Result,
{
    critical_section::with(|_| {
        let mut stream = get_uart_stream();
        let _ = f(&mut stream);
        if doasync {
            stream.send_async(false);
        } else {
            stream.send_blocking(false);
        }
    });
}

#[macro_export]
macro_rules! uprintln {
    ( $($arg:tt)* ) => {
        {
            use core::fmt::Write;
            use $crate::logging::print_to_uart;

            print_to_uart(true, |stream| {
                core::writeln!(stream, $($arg)*)
            });
        }
    };
}

/// A byte stream to the host (e.g., host's stdout or stderr).
#[derive(Clone, Copy)]
pub struct UartStream<const N: usize> {
    pub length: usize,
    pub out_buffer: [u8; N],
}

/// Builds one chunked UART frame from the head of `buffer`.
///
/// Deliberately uses raw pointer copies: a bounds check here would put
/// `panic_fmt` on the panic handler's call graph (the same reason `as_bytes`
/// uses `from_raw_parts`), and the static stack gate rejects the resulting
/// cycle. `copy_from_slice` is not usable for the same reason: its length
/// mismatch check carries a panic edge.
#[inline]
fn build_uart_frame(buffer: &[u8], is_panic: bool) -> UartData {
    // Use the correct payload size to avoid overwriting CRC
    let len = min(UART_PAYLOAD_MAX_SIZE, buffer.len());

    let mut msg = UartData {
        len: UART_DATA_LEN as u32,
        data: [0; UART_DATA_LEN],
    };

    msg.data[2] = if is_panic {
        UartMsg::PanicMessage
    } else {
        UartMsg::PrintMessage
    } as u8;

    // SAFETY: `len <= UART_PAYLOAD_MAX_SIZE` so `3..3 + len` is inside the
    // 44-byte `data` array, and `len <= buffer.len()` so the source range is
    // in bounds. `copy_nonoverlapping` cannot panic.
    unsafe {
        core::ptr::copy_nonoverlapping(
            buffer.as_ptr(),
            msg.data.as_mut_ptr().add(UART_PAYLOAD_OFFSET),
            len,
        );
    }

    msg
}

impl<const N: usize> UartStream<N> {
    pub const fn new() -> UartStream<N> {
        UartStream::<N> {
            length: 0,
            out_buffer: [0; N],
        }
    }

    /// Appends `buffer` to the stream.
    ///
    /// Returns `Err(())` when the bytes would not fit. Deliberately avoids slice
    /// indexing and `copy_from_slice`: their bounds/length checks put
    /// `slice_end_index_len_fail` / `panic_fmt` on the panic handler's call graph,
    /// which the static stack gate rejects.
    pub fn write_all(&mut self, buffer: &[u8]) -> Result<(), ()> {
        let remaining = self.out_buffer.len().saturating_sub(self.length);
        if buffer.len() > remaining {
            return Err(());
        }
        // SAFETY: `self.length <= out_buffer.len()` (invariant maintained by this
        // function) and `buffer.len() <= remaining`, so both ranges are in bounds.
        unsafe {
            core::ptr::copy_nonoverlapping(
                buffer.as_ptr(),
                self.out_buffer.as_mut_ptr().add(self.length),
                buffer.len(),
            );
        }
        self.length += buffer.len();
        Ok(())
    }

    /// The bytes written so far.
    ///
    /// Uses `from_raw_parts` (as the send sinks already do) because an upstream
    /// bounds check would put `slice_end_index_len_fail` on the panic handler's
    /// call graph, which the static stack gate rejects.
    pub fn as_bytes(&self) -> &[u8] {
        unsafe { slice::from_raw_parts(self.out_buffer.as_ptr(), self.length) }
    }

    /// Queues the stream on the async UART channel, one chunk at a time.
    ///
    /// Every log line is mirrored into the SWire ring as it is sent. This sink
    /// is the one that can reach the channel's `try_send`, so it must never be
    /// called from the panic handler.
    pub fn send_async(&self, is_panic: bool) {
        let mut buffer = self.as_bytes();

        #[cfg(feature = "rtt")]
        crate::diagnostics::rtt::rtt_write_bytes(buffer);

        while !buffer.is_empty() {
            let len = min(UART_PAYLOAD_MAX_SIZE, buffer.len());

            let _ = app()
                .uart_manager
                .send_message(&build_uart_frame(buffer, is_panic));

            buffer = unsafe { slice::from_raw_parts(buffer.as_ptr().add(len), buffer.len() - len) };
        }
    }

    /// Hands the stream straight to the UART driver, one chunk at a time.
    ///
    /// This is the panic-path sink. It must never touch the async channel: a
    /// panic while sending would re-enter `panic_fmt` through `try_send`, a
    /// call-graph cycle the static stack gate rejects.
    pub fn send_blocking(&self, is_panic: bool) {
        let mut buffer = self.as_bytes();

        // Panic text is mirrored too, so it is readable in the ring before the
        // panic sector is written and the device reboots.
        #[cfg(feature = "rtt")]
        crate::diagnostics::rtt::rtt_write_bytes(buffer);

        while !buffer.is_empty() {
            let len = min(UART_PAYLOAD_MAX_SIZE, buffer.len());

            app()
                .uart_manager
                .driver
                .send(&build_uart_frame(buffer, is_panic));

            buffer = unsafe { slice::from_raw_parts(buffer.as_ptr().add(len), buffer.len() - len) };
        }
    }
}

impl<const N: usize> Default for UartStream<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Write for UartStream<N> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.write_all(s.as_bytes()).map_err(|_| fmt::Error)
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::sdk::drivers::uart::UartDriver;
    use crate::uart_manager::UartManager;
    use crate::{app_mocker, mock_app_mocker};
    use mry::Any;

    #[test]
    #[mry::lock(app_mocker)]
    fn test_uart_stream_send_synchronous() {
        // Create a test stream with some data
        let mut stream = UartStream::<128>::new();
        let test_data = b"Hello UART";
        let _ = stream.write_all(test_data);

        // Create a UartDriver (mry makes it mockable)
        let mut driver = UartDriver::default();
        // Mock the send method to return true
        driver.mock_send(Any).returns(());

        // Create a UartManager with the mocked driver
        let mut uart_manager = UartManager::default();
        uart_manager.driver = driver;

        // Set up mock app with the uart_manager
        let mut app = crate::App::default();
        app.uart_manager = uart_manager;
        mock_app_mocker().returns(&mut app);

        // Call the blocking sink with doasync = false (synchronous) to cover the
        // driver path
        stream.send_blocking(false);

        // Verify that the synchronous send was called
        app.uart_manager.driver.mock_send(Any).assert_called(1);
    }

    #[test]
    #[mry::lock(app_mocker)]
    fn test_uart_stream_send_panic_mode() {
        // Create a test stream with some data
        let mut stream = UartStream::<128>::new();
        let test_data = b"Panic message!";
        let _ = stream.write_all(test_data);

        // Create a UartDriver (mry makes it mockable)
        let mut driver = UartDriver::default();
        // Mock the send method to return true
        driver.mock_send(Any).returns(());

        // Create a UartManager with the mocked driver
        let mut uart_manager = UartManager::default();
        uart_manager.driver = driver;

        // Set up mock app with the uart_manager
        let mut app = crate::App::default();
        app.uart_manager = uart_manager;
        mock_app_mocker().returns(&mut app);

        // Call the blocking panic sink with is_panic = true to cover the panic
        // message type
        stream.send_blocking(true);

        // Verify that the synchronous send was called
        app.uart_manager.driver.mock_send(Any).assert_called(1);
    }

    #[test]
    #[mry::lock(app_mocker)]
    fn test_print_to_uart_blocking_sink() {
        let mut driver = UartDriver::default();
        driver.mock_send(Any).returns(());

        let mut uart_manager = UartManager::default();
        uart_manager.driver = driver;

        let mut app = crate::App::default();
        app.uart_manager = uart_manager;
        mock_app_mocker().returns(&mut app);

        // A default stream is empty; this also covers the `Default` impl.
        assert_eq!(UartStream::<128>::default().as_bytes(), b"");

        // doasync = false routes straight to the driver instead of the queue.
        print_to_uart(false, |stream| write!(stream, "sync"));

        app.uart_manager.driver.mock_send(Any).assert_called(1);
    }

    #[test]
    fn test_write_all_overflow_rejected() {
        let mut stream = UartStream::<4>::new();
        stream.write_all(b"abc").unwrap();
        let length_before = stream.length;

        // 2 more bytes would need 5 of 4 slots: rejected, not truncated.
        assert_eq!(stream.write_all(b"de"), Err(()));

        // Nothing was appended: length and the already-written bytes are intact.
        assert_eq!(stream.length, length_before);
        assert_eq!(stream.as_bytes(), b"abc");
        assert_eq!(&stream.out_buffer[..], &[b'a', b'b', b'c', 0]);
    }

    #[test]
    fn test_write_all_exact_capacity() {
        let mut stream = UartStream::<4>::new();
        let test_data = b"abcd";

        assert_eq!(stream.write_all(test_data), Ok(()));

        assert_eq!(stream.length, 4);
        assert_eq!(stream.as_bytes(), test_data);
    }
}
