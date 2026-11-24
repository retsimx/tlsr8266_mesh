//! # UART Log Sink
//!
//! The application-layer `uprintln!` logging sink for the TLSR8266 mesh
//! lighting firmware.
//!
//! ## Why the transport is injected
//!
//! Formatting a log line is application work and it is reached from the panic
//! handler, but the UART transport itself sits *above* this module: the async
//! sink queues through `queues::enqueue_uart_tx` and the panic sink writes the
//! DMA buffer directly. `logging` therefore names no transport at all. [`LogSinks`]
//! is installed once by `app::main_task` (before the boot banner), and every send
//! routes through it. With nothing registered the sends are a no-op, which is
//! what makes `logging` sit below `uart` in the layering graph.
//!
//! ## Key Responsibilities:
//! - `UartStream<const N: usize>`: a fixed-capacity byte sink implementing
//!   `core::fmt::Write`
//! - Chunked framing into 44-byte built-in-UART packets (`build_uart_frame`)
//! - The async (`send_async`), registered-blocking (`send_blocking`) and direct
//!   panic-path (`send_blocking_with`) send sinks, each with one RTT mirror
//! - `uprintln!`, `print_to_uart` and `get_uart_stream`

use core::cell::Cell;
use core::cmp::min;
use core::fmt::{self, Write};
use core::slice;

use critical_section::Mutex;
use mesh_protocol::uart::{
    UartData, UartMsg, UART_DATA_LEN, UART_PAYLOAD_MAX_SIZE, UART_PAYLOAD_OFFSET,
};

/// The UART transport sinks every log line is handed to.
///
/// Every slot is a plain `fn` pointer so the struct is `Copy` and can be copied
/// out of the critical section in one go — the lock is never held across a sink
/// call. Ownership runs `app.rs` → `logging`: the providers live above this
/// module, so the composition root installs them once, before the boot banner,
/// and a line logged earlier is dropped rather than routed somewhere unexpected.
#[derive(Copy, Clone)]
pub struct LogSinks {
    /// Queue one frame on the async transport; `true` only when it was queued.
    pub async_tx: fn(&UartData) -> bool,
    /// Write one frame on the blocking transport (panic path).
    pub blocking_tx: fn(&UartData),
}

/// Test-only stand-ins for the sinks `app::main_task` registers.
///
/// Before this module became sink-injected it reached `uart` directly,
/// so a host test in another module that logs without registering (the UART RX
/// drop-report line) still expects its frame. These adapters keep that
/// behaviour for test builds only; [`LOG_SINKS`] starts empty in production, so
/// a line logged before `app::main_task` registers is dropped.
#[cfg(test)]
mod test_sinks {
    use super::UartData;

    pub(super) fn async_tx(msg: &UartData) -> bool {
        crate::uart::send_message(msg).is_ok()
    }

    pub(super) fn blocking_tx(msg: &UartData) {
        crate::uart::send_sync_panic(msg)
    }
}

#[cfg(test)]
const TEST_LOG_SINKS: LogSinks = LogSinks {
    async_tx: test_sinks::async_tx,
    blocking_tx: test_sinks::blocking_tx,
};

#[cfg(not(test))]
static LOG_SINKS: Mutex<Cell<Option<LogSinks>>> = Mutex::new(Cell::new(None));

#[cfg(test)]
static LOG_SINKS: Mutex<Cell<Option<LogSinks>>> = Mutex::new(Cell::new(Some(TEST_LOG_SINKS)));

/// Registers the UART sinks.
///
/// Called once from `app::main_task` before the boot banner. `#[inline(never)]`
/// keeps the one-off registration out of `main_task`'s future; the dispatchers
/// below carry the symbol names `sdk/stack_analysis.py` resolves.
#[inline(never)]
pub fn set_log_sinks(sinks: LogSinks) {
    critical_section::with(|cs| LOG_SINKS.borrow(cs).set(Some(sinks)));
}

/// Copies the registered sinks out of the critical section, if any.
fn log_sinks() -> Option<LogSinks> {
    critical_section::with(|cs| LOG_SINKS.borrow(cs).get())
}

/// Hands one frame to the async sink. `#[inline(never)]` is load-bearing:
/// `sdk/stack_analysis.py`'s indirect-edge table binds `dispatch_log_async` to
/// `app::log_async_tx_hook`, and `OPT_INLINE_THRESHOLD=50` would otherwise fold
/// this one-call shim into its caller and erase the symbol from the ELF.
#[inline(never)]
fn dispatch_log_async(frame: &UartData) -> bool {
    match log_sinks() {
        Some(sinks) => (sinks.async_tx)(frame),
        None => false,
    }
}

/// Hands one frame to the blocking sink; a no-op before registration.
#[inline(never)]
fn dispatch_log_blocking(frame: &UartData) {
    if let Some(sinks) = log_sinks() {
        (sinks.blocking_tx)(frame);
    }
}

#[cfg(test)]
fn reset_log_sinks_for_test() {
    critical_section::with(|cs| LOG_SINKS.borrow(cs).set(None));
}

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

    // SAFETY: (5) panic/RTT call graph. `len <= UART_PAYLOAD_MAX_SIZE` so
    // `3..3 + len` is inside the 44-byte `data` array, and `len <= buffer.len()`
    // so the source range is in bounds. `copy_nonoverlapping` cannot panic.
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
        // SAFETY: (5) panic/RTT call graph. `self.length <= out_buffer.len()`
        // (invariant maintained by this function) and `buffer.len() <= remaining`,
        // so both ranges are in bounds.
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
        // SAFETY: (5) panic/RTT call graph. `self.length` only advances in `write_all`
        // after the fit check, so `length <= N`; a checked `&out_buffer[..length]`
        // would put `slice_end_index_len_fail` on the panic handler's call graph, which
        // the static stack gate rejects.
        unsafe { slice::from_raw_parts(self.out_buffer.as_ptr(), self.length) }
    }

    /// Queues the stream on the async UART channel, one chunk at a time.
    ///
    /// Every log line is mirrored into the SWire ring exactly once, up front.
    /// The chunks then go to the registered [`LogSinks::async_tx`] (which
    /// reaches the transport queue), so this sink must never be called from the
    /// panic handler.
    pub fn send_async(&self, is_panic: bool) {
        let mut buffer = self.as_bytes();

        #[cfg(feature = "rtt")]
        crate::diagnostics::rtt::rtt_write_bytes(buffer);

        while !buffer.is_empty() {
            let len = min(UART_PAYLOAD_MAX_SIZE, buffer.len());

            let _ = dispatch_log_async(&build_uart_frame(buffer, is_panic));

            // SAFETY: (5) panic/RTT call graph. `len == min(UART_PAYLOAD_MAX_SIZE,
            // buffer.len())`, so `add(len)` stays inside `buffer` and the remaining length
            // is in bounds; checked slicing would add a panic edge the stack gate rejects.
            buffer = unsafe { slice::from_raw_parts(buffer.as_ptr().add(len), buffer.len() - len) };
        }
    }

    /// Hands the stream to the registered blocking UART sink, one chunk at a
    /// time, after one RTT mirror.
    ///
    /// A thin front for [`UartStream::send_blocking_with`]: the only difference
    /// between the registered and the direct path is which sink receives the
    /// chunks, and sharing the loop keeps the audited raw-pointer copy in one
    /// place. It must never touch the async channel: a panic while sending
    /// would re-enter `panic_fmt` through `try_send`, a call-graph cycle the
    /// static stack gate rejects. The panic handler calls `send_blocking_with`
    /// directly, so nothing in the firmware reaches this route today and the
    /// linker drops it from the image; it stays for the registered blocking
    /// path, tested on the host.
    pub fn send_blocking(&self, is_panic: bool) {
        self.send_blocking_with(is_panic, dispatch_log_blocking);
    }

    /// Hands the stream to `sink` directly, one chunk at a time, after one RTT
    /// mirror.
    ///
    /// This is the panic handler's path: `lib.rs` passes
    /// `uart::send_sync_panic` by name, so the call is a direct edge on
    /// the panic call graph (the static stack gate and
    /// `sdk/stack_analysis.py` both rely on it) rather than an indirect one
    /// through the registered sinks. `sink` is called on a monomorphised
    /// generic, so nothing is lost to a dynamic dispatch.
    ///
    /// The mirror happens exactly once, here: the caller must not mirror the
    /// same bytes again.
    pub fn send_blocking_with<F: Fn(&UartData)>(&self, is_panic: bool, sink: F) {
        let mut buffer = self.as_bytes();

        // Panic text is mirrored too, so it is readable in the ring before the
        // panic sector is written and the device reboots.
        #[cfg(feature = "rtt")]
        crate::diagnostics::rtt::rtt_write_bytes(buffer);

        while !buffer.is_empty() {
            let len = min(UART_PAYLOAD_MAX_SIZE, buffer.len());

            sink(&build_uart_frame(buffer, is_panic));

            // SAFETY: (5) panic/RTT call graph. As in `send_async`: `len` is
            // `min(UART_PAYLOAD_MAX_SIZE, buffer.len())`, so the advanced pointer and the
            // remaining length both stay inside the same slice.
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
    use crate::uart::reset_for_test;
    use core::sync::atomic::{AtomicU32, Ordering};
    use std::sync::{Mutex as StdMutex, MutexGuard};
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_radio::queues::UART_TX_CHANNEL;

    /// The 44-byte frame `build_uart_frame` must produce for `payload`.
    ///
    /// Built independently of `build_uart_frame` so the assertions pin the wire
    /// layout (command byte at 2, payload from 3, declared length) rather than
    /// restating the production code.
    fn expected_frame(payload: &[u8], cmd: UartMsg) -> UartData {
        let mut data = [0u8; UART_DATA_LEN];
        data[2] = cmd as u8;
        data[3..3 + payload.len()].copy_from_slice(payload);

        UartData {
            len: UART_DATA_LEN as u32,
            data,
        }
    }

    /// Installs the test stand-ins for `app::main_task`'s hooks, so the
    /// assertions below observe the frames `logging` produced through the
    /// registered sinks.
    fn install_test_sinks() {
        set_log_sinks(TEST_LOG_SINKS);
    }

    /// Frames the recording sinks were handed, in call order.
    ///
    /// `LogSinks` is the module's documented hook seam: [`set_log_sinks`] is
    /// what `app::main_task` calls in production, so a test that wants to see
    /// the exact bytes `logging` produces installs a recorder and asserts on
    /// the recording instead of mocking the transport one layer down.
    static RECORDED_ASYNC: StdMutex<Vec<UartData>> = StdMutex::new(Vec::new());
    static RECORDED_BLOCKING: StdMutex<Vec<UartData>> = StdMutex::new(Vec::new());

    const RECORDING_SINKS: LogSinks = LogSinks {
        async_tx: record_async,
        blocking_tx: record_blocking,
    };

    fn record_async(msg: &UartData) -> bool {
        recorded(&RECORDED_ASYNC).push(*msg);
        true
    }

    fn record_blocking(msg: &UartData) {
        recorded(&RECORDED_BLOCKING).push(*msg);
    }

    fn recorded(sink: &'static StdMutex<Vec<UartData>>) -> MutexGuard<'static, Vec<UartData>> {
        sink.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn async_frames() -> Vec<UartData> {
        recorded(&RECORDED_ASYNC).clone()
    }

    fn blocking_frames() -> Vec<UartData> {
        recorded(&RECORDED_BLOCKING).clone()
    }

    /// Installs [`RECORDING_SINKS`] and restores the test default on drop.
    ///
    /// The sinks are process-wide, so an RAII guard keeps a failing assertion
    /// from leaking the recorder into the next test.
    struct RecordingSinks;

    impl RecordingSinks {
        fn install() -> Self {
            recorded(&RECORDED_ASYNC).clear();
            recorded(&RECORDED_BLOCKING).clear();
            set_log_sinks(RECORDING_SINKS);
            Self
        }
    }

    impl Drop for RecordingSinks {
        fn drop(&mut self) {
            install_test_sinks();
        }
    }

    /// `reg_dma_tx_rdy0`: the DMA1 transmit trigger `uart::send_sync_panic` arms.
    const REG_DMA_TX_RDY0: u32 = 0x524;

    /// Number of DMA1 transmit triggers recorded in the fake register bank.
    fn dma1_arms() -> usize {
        FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == REG_DMA_TX_RDY0 | tlsr8266_pac::REG_BASE_ADDR)
            .count()
    }

    /// Calls made to the direct sink in the `send_blocking_with` test.
    static DIRECT_SINK_CALLS: AtomicU32 = AtomicU32::new(0);

    fn counting_sink(_msg: &UartData) {
        DIRECT_SINK_CALLS.fetch_add(1, Ordering::Relaxed);
    }

    #[test]
    fn test_uart_stream_send_synchronous() {
        reset_for_test();
        let _sinks = RecordingSinks::install();

        let mut stream = UartStream::<128>::new();
        let test_data = b"Hello UART";
        assert_eq!(stream.write_all(test_data), Ok(()));

        // doasync = false routes to the blocking panic sink, never the queue.
        stream.send_blocking(false);

        assert_eq!(
            blocking_frames(),
            vec![expected_frame(test_data, UartMsg::PrintMessage)],
            "the exact frame must reach the blocking sink, exactly once"
        );
        assert!(
            async_frames().is_empty(),
            "a blocking send must never reach the async sink"
        );
    }

    #[test]
    fn test_uart_stream_send_panic_mode() {
        reset_for_test();
        let _sinks = RecordingSinks::install();

        let mut stream = UartStream::<128>::new();
        let test_data = b"Panic message!";
        assert_eq!(stream.write_all(test_data), Ok(()));

        // is_panic = true flags the frame as a panic message.
        stream.send_blocking(true);

        assert_eq!(
            blocking_frames(),
            vec![expected_frame(test_data, UartMsg::PanicMessage)],
            "the exact frame must reach the blocking sink, exactly once"
        );
        assert!(async_frames().is_empty());
    }

    #[test]
    fn test_print_to_uart_blocking_sink() {
        reset_for_test();
        let _sinks = RecordingSinks::install();

        // A default stream is empty; this also covers the `Default` impl.
        assert_eq!(UartStream::<128>::default().as_bytes(), b"");

        // doasync = false routes straight to the panic sink instead of the queue.
        print_to_uart(false, |stream| write!(stream, "sync"));

        assert_eq!(
            blocking_frames(),
            vec![expected_frame(b"sync", UartMsg::PrintMessage)]
        );
        assert!(async_frames().is_empty());
    }

    #[test]
    fn test_uart_stream_send_async_routes_through_queue() {
        reset_for_test();
        install_test_sinks();

        let mut stream = UartStream::<128>::new();
        let test_data = b"Hello async";
        assert_eq!(stream.write_all(test_data), Ok(()));

        // doasync = true routes through the transport queue, never the driver.
        stream.send_async(false);

        let queued = UART_TX_CHANNEL.try_receive().expect("one frame queued");
        assert_eq!(queued, expected_frame(test_data, UartMsg::PrintMessage));
        assert!(
            UART_TX_CHANNEL.try_receive().is_err(),
            "one chunk means exactly one frame"
        );
    }

    /// `send_blocking_with` must hand every chunk to the sink it was given and
    /// mirror the whole line into the ring exactly once: it neither consults the
    /// registered sinks nor routes the frames a second time, so a 100-byte line
    /// produces three frames, not six.
    #[test]
    fn test_send_blocking_with_hands_each_chunk_to_the_given_sink_once() {
        reset_for_test();
        // The registered sinks are installed and must stay untouched.
        let _sinks = RecordingSinks::install();
        DIRECT_SINK_CALLS.store(0, Ordering::Relaxed);

        let mut stream = UartStream::<128>::new();
        // 44-byte payload chunks: 44 + 44 + 12.
        let test_data = [b'x'; 100];
        assert_eq!(stream.write_all(&test_data), Ok(()));

        stream.send_blocking_with(true, counting_sink);

        assert_eq!(
            DIRECT_SINK_CALLS.load(Ordering::Relaxed),
            3,
            "one call per chunk, and the mirror/registration paths add none"
        );
        assert!(
            blocking_frames().is_empty() && async_frames().is_empty(),
            "the registered sinks must not see the frames of a direct send"
        );
    }

    /// Before `app::main_task` registers the sinks, a log line must be dropped:
    /// no transport may be reached from this module.
    ///
    /// This is the one test that clears the crate-wide test default, so it puts
    /// it back: other modules' tests log without registering (see `test_sinks`).
    #[test]
    fn test_unregistered_sinks_drop_log_lines() {
        reset_for_test();
        FakeMmio::reset();
        reset_log_sinks_for_test();

        let mut stream = UartStream::<128>::new();
        assert_eq!(stream.write_all(b"lost line"), Ok(()));

        stream.send_async(false);
        stream.send_blocking(false);

        assert!(
            UART_TX_CHANNEL.try_receive().is_err(),
            "an unregistered sink must not reach the async transport"
        );
        assert_eq!(
            dma1_arms(),
            0,
            "an unregistered sink must not arm the blocking transport"
        );

        install_test_sinks();
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
