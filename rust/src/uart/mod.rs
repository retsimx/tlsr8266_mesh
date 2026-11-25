//! # UART bridge: hardware, tasks and seams
//!
//! The TLSR8266 built-in UART carries a 44-byte framed bridge between the
//! firmware and the host tooling (`utilities/meshutils`). This module owns the
//! peripheral and the two async tasks that move frames; the frame layout itself
//! lives in [`protocol`], which is pure and hardware-free.
//!
//! ## Why the application edges are seams
//!
//! A UART frame can ask the device to do three things the transport must not
//! implement itself — send a mesh command, enable status reporting, or announce
//! that a host has attached — and the transport has to hand received frames
//! back out to the same layers. Naming `mesh`, `light` or the SDK here would
//! make the transport depend on its own consumers, so [`UartTxOps`],
//! [`set_command_handler`] and [`on_mesh_rx_packet`] are the only edges out:
//! `app::main_task` installs them once, right after [`init`] and before the
//! first log line.
//!
//! ## Key Responsibilities
//!
//! - [`UART_HW`]: the one owner of the UART peripheral registers
//! - [`take_rx_and_ack_tx_irq`]: the interrupt body, which only moves DMA bytes
//!   and bumps a counter — never validates, logs or touches the mesh
//! - `uart_sender` / `uart_receiver`: the async retry/ack and validate/handle
//!   paths, both in task context
//! - [`send_message`] / [`send_sync_panic`]: the async and panic-path sinks
//! - [`on_mesh_rx_packet`]: relay of a received mesh packet to the host

mod protocol;

use core::mem::size_of;
use core::ptr::addr_of;
use core::slice;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
#[cfg(test)]
use embassy_time::Instant;
#[cfg(not(test))]
use embassy_time::{Duration, Instant, WithTimeout};
use heapless::Deque;
// `AtomicBool::swap` is a read-modify-write, which `thumbv6m` cannot do in
// hardware; `portable-atomic` (built with `critical-section` here) falls back to
// a short interrupt-masked sequence.
use portable_atomic::AtomicBool;

use crate::uprintln;
use mesh_protocol::packet_types::{AppCmdValue, Packet};
#[cfg(test)]
use mesh_protocol::uart::compute_crc;
use mesh_protocol::uart::{verify_crc, UartData, UartMsg, UART_DATA_LEN};
use tlsr8266_hal::common::sync::CriticalSectionMutex;
use tlsr8266_hal::drivers::uart::{UartDriver, UartIrqMask};

pub use protocol::UartCommand;
use protocol::{
    ack_retry_frame, build_ack, decode_command, fingerprint, outgoing_fingerprint, Fingerprint,
    MSG_TYPE_INDEX,
};
#[cfg(test)]
use protocol::{
    CRC_RANGE, CTRL_ACK_FLAG_INDEX, CTRL_DESTINATION_RANGE, CTRL_PAYLOAD_RANGE,
    CTRL_RETRANSMIT_INDEX, OUTGOING_FINGERPRINT_RANGE, SNO_RANGE,
};

/// UART frames the sender task can hold.
///
/// The task takes the channel as an argument, so this has to equal the
/// application queue's capacity for `app.rs` to pass it in; the argument type is
/// the check.
const UART_TX_QUEUE_CAPACITY: usize = 12;

/// Raw mutex backing the UART channels/signals: masks interrupts for the
/// (short, allocation-free) critical sections inside `embassy-sync`.
type CsRawMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/// The UART peripheral, owned by this module.
///
/// Every register access goes through this one guard. It is only ever held for
/// a handful of register reads/writes — never across an `.await`, a blocking
/// flash call or another `CriticalSectionMutex`.
pub static UART_HW: CriticalSectionMutex<UartDriver> =
    CriticalSectionMutex::new(UartDriver::default_const());

/// Serialises the two async transmitters (`uart_sender` / `uart_receiver`).
///
/// The synchronous panic sink bypasses it on purpose: a panic must not park on
/// an async mutex.
static UART_TX_LOCK: Mutex<CsRawMutex, ()> = Mutex::new(());

/// Latched TX-complete notification, signalled by [`take_rx_and_ack_tx_irq`].
static UART_TX_DONE: Signal<CsRawMutex, ()> = Signal::new();

/// Raw frames straight out of the DMA buffer, waiting for the [`uart_receiver`]
/// task, which validates them in task context.
pub static UART_RX_CHANNEL: Channel<CsRawMutex, UartData, 12> = Channel::new();

/// Frames dropped because [`UART_RX_CHANNEL`] was full.
///
/// Bumped from interrupt context by [`take_rx_and_ack_tx_irq`]; the
/// [`uart_receiver`] task turns the delta into a log line.
static UART_RX_DROPPED: AtomicU32 = AtomicU32::new(0);

/// Fingerprints of the frames this device has already handled, so a mesh echo
/// of our own command is not forwarded to the host twice.
pub static UART_SENT_DEDUP: CriticalSectionMutex<Deque<Fingerprint, 12>> =
    CriticalSectionMutex::new(Deque::new());

/// Whether a host has spoken on UART yet. The sender task stays parked until
/// the first valid frame arrives, so boot banners on a headless light cost
/// nothing.
static UART_SENDER_STARTED: AtomicBool = AtomicBool::new(false);

/// Wakes the parked [`uart_sender`] task on the first valid frame.
static UART_STARTED_SIGNAL: Signal<CsRawMutex, ()> = Signal::new();

/// Latched ack counter from the host, awaited by the retry loop.
static UART_ACK_SIGNAL: Signal<CsRawMutex, u8> = Signal::new();

/// Most recent ack counter received from the host.
static UART_LAST_ACK: AtomicU8 = AtomicU8::new(0);

/// Total transmit attempts per frame: one initial transmit plus up to two
/// retransmits.
///
/// A host that never ACKs must not wedge `uart_sender`.
pub const MAX_TX_ATTEMPTS: u8 = 3;

/// Retransmits only: `0` when the first attempt is ACKed, `2` when all three
/// attempts expire.
pub static UART_TX_RETRIES: AtomicU32 = AtomicU32::new(0);

/// Frames that exhausted [`MAX_TX_ATTEMPTS`] and were dropped.
pub static UART_TX_DROPPED: AtomicU32 = AtomicU32::new(0);

/// ACKs that arrived while no frame was awaiting one, or that carried a
/// counter other than the frame in flight.
pub static UART_LATE_ACKS: AtomicU32 = AtomicU32::new(0);

/// Counter of the frame currently awaiting an ACK.
static UART_PENDING_COUNTER: AtomicU8 = AtomicU8::new(0);

/// Whether a frame is currently awaiting an ACK.
static UART_AWAITING_ACK: AtomicBool = AtomicBool::new(false);

/// Set while a drop log is outstanding. A host that never acks must not turn
/// each dropped frame into another frame that is itself dropped and logged.
static UART_DROP_LOG_PENDING: AtomicBool = AtomicBool::new(false);

/// TX-side operations the application owns: queueing a frame and reading the
/// dropped-frame counter.
///
/// `#[derive(Copy, Clone)]` so the struct can be copied out of the critical
/// section — the lock is never held across a sink. `app::main_task` registers
/// this once, right after [`init`], because the SDK queue it wraps sits above
/// this module in the layering graph.
#[derive(Copy, Clone)]
pub struct UartTxOps {
    /// Queue one frame; `true` only when it was queued.
    pub enqueue_tx: fn(&UartData) -> bool,
    /// How many frames the transport has dropped because the queue was full.
    pub dropped_tx: fn() -> u32,
}

/// Test-only stand-in for the ops `app::main_task` registers.
///
/// Before this module dropped its SDK import every frame went straight to the
/// queue, and host tests in other modules still expect their frames to land
/// there. These adapters keep that behaviour for test builds only; in
/// production the static starts empty until `main_task` registers, so a frame
/// produced before registration is reported as dropped rather than routed to a
/// transport nobody installed.
#[cfg(test)]
mod test_ops {
    use super::UartData;

    pub(super) fn enqueue_tx(msg: &UartData) -> bool {
        tlsr8266_radio::queues::enqueue_uart_tx(msg)
    }

    pub(super) fn dropped_tx() -> u32 {
        tlsr8266_radio::queues::dropped_uart_tx()
    }
}

#[cfg(test)]
const TEST_TX_OPS: UartTxOps = UartTxOps {
    enqueue_tx: test_ops::enqueue_tx,
    dropped_tx: test_ops::dropped_tx,
};

#[cfg(not(test))]
static UART_TX_OPS: CriticalSectionMutex<Option<UartTxOps>> = CriticalSectionMutex::new(None);

#[cfg(test)]
static UART_TX_OPS: CriticalSectionMutex<Option<UartTxOps>> =
    CriticalSectionMutex::new(Some(TEST_TX_OPS));

/// Signature of the application's UART command handler.
///
/// A plain `fn` pointer, never a closure: a closure would mangle to
/// `main_task::{{closure}}`, which `sdk/stack_analysis.py` rejects.
pub type UartCommandHandler = fn(UartCommand) -> [u8; 3];

/// The application's command handler, installed by `app::main_task`.
static UART_COMMAND_HANDLER: CriticalSectionMutex<Option<UartCommandHandler>> =
    CriticalSectionMutex::new(None);

/// Registers the TX-side operations.
///
/// Called once from `app::main_task` immediately after [`init`], so every log
/// line from the boot banner onwards has a transport. `#[inline(never)]` keeps
/// the one-off registration out of `main_task`'s future.
#[inline(never)]
pub fn set_tx_ops(ops: UartTxOps) {
    *UART_TX_OPS.lock() = Some(ops);
}

/// Copies the registered operations out, if any.
fn tx_ops() -> Option<UartTxOps> {
    *UART_TX_OPS.lock()
}

/// Enqueues one frame through the registered ops.
///
/// `#[inline(never)]` is load-bearing: `sdk/stack_analysis.py`'s indirect-edge
/// table (`INDIRECT_EDGES` key `uart_tx_ops_push`) binds this symbol to
/// `app::enqueue_uart_tx_hook`, and the optimiser would otherwise fold this
/// one-call shim into its caller and erase the dispatcher from the ELF. No
/// closure is used anywhere in this seam for the same reason.
#[inline(never)]
fn uart_tx_ops_push(msg: &UartData) -> bool {
    match tx_ops() {
        Some(ops) => (ops.enqueue_tx)(msg),
        None => false,
    }
}

/// Reads the transport's dropped-frame counter through the registered ops.
///
/// `#[inline(never)]` for the same reason as [`uart_tx_ops_push`]; it is the
/// `INDIRECT_EDGES` key `uart_tx_ops_dropped`.
#[inline(never)]
fn uart_tx_ops_dropped() -> u32 {
    match tx_ops() {
        Some(ops) => (ops.dropped_tx)(),
        None => 0,
    }
}

/// Registers the handler that executes host commands.
///
/// `app::main_task` installs `app::uart_command_hook` once, immediately after
/// [`init`]. With no handler installed every command is acknowledged with a
/// zero mesh sequence number and nothing else happens.
#[inline(never)]
pub fn set_command_handler(handler: UartCommandHandler) {
    *UART_COMMAND_HANDLER.lock() = Some(handler);
}

/// Copies the registered command handler out, if any.
fn command_handler() -> Option<UartCommandHandler> {
    *UART_COMMAND_HANDLER.lock()
}

/// Hands one decoded command to the application and returns the 3-byte mesh
/// sequence number its ACK frame must carry.
///
/// `#[inline(never)]` is load-bearing: `sdk/stack_analysis.py`'s indirect-edge
/// table (`INDIRECT_EDGES` key `dispatch_uart_cmd`) binds this symbol to
/// `app::uart_command_hook`, and inlining it into `handle_received_frame` would
/// erase the dispatcher from the ELF and fail the table closed. No closure is
/// used anywhere in this seam for the same reason.
#[inline(never)]
fn dispatch_uart_cmd(cmd: UartCommand) -> [u8; 3] {
    match command_handler() {
        Some(handler) => handler(cmd),
        None => [0; 3],
    }
}

/// Forwards one received mesh packet to the host.
///
/// The SDK's UART bridge and the mesh TX path both reach this through
/// `app::uart_mesh_rx_hook`, so the transport stays free of both. `own_dev_addr`
/// is the mesh address of this device, passed in by the caller rather than read
/// from the SDK here.
///
/// Packets this device sent are skipped (the host already knows about them) and
/// so are frames already forwarded, keyed on the `dst`..`par` fingerprint.
pub fn on_mesh_rx_packet(pkt: &Packet, own_dev_addr: u16) {
    // Don't report messages that we sent
    if !started()
        || (pkt.ll_app().value.src == own_dev_addr && pkt.ll_app().value.dst != own_dev_addr)
    {
        return;
    }

    let data = addr_of!(pkt.ll_app().value) as *const u8;
    // Compute the CRC of important bits. This is from start of AppCmdValueT.dst (5) to end of AppCmdValueT.par (20)
    // SAFETY: (4) audited packed-wire view. `data` points at the live packet's
    // `AppCmdValue` field (20 bytes), and the 15-byte view is the `dst`..`par`
    // span of that wire layout (offsets 5..20), so it stays inside the
    // caller-owned packet. The offset is applied with `add`, not by casting the
    // address to an integer: a `u32` cast truncates the pointer on a 64-bit host
    // and reads through an address that is no longer inside the packet.
    let msg = unsafe { slice::from_raw_parts(data.add(5), 20 - 5) };

    // Check if this message is one we sent
    if UART_SENT_DEDUP.lock().iter().any(|c| c == msg) {
        // We did, nothing more to do here
        return;
    }

    let mut msg: UartData = UartData {
        len: UART_DATA_LEN as u32,
        data: [0; UART_DATA_LEN],
    };

    msg.data[MSG_TYPE_INDEX] = u8::from(UartMsg::MeshMessage);
    for i in 0..size_of::<AppCmdValue>() {
        // SAFETY: (4) audited packed-wire copy. `i < size_of::<AppCmdValue>()` (20) and
        // the destination offset `3 + i` stays below `UART_DATA_LEN` (44), so both the
        // `msg.data` store and the `data` load stay in bounds of their buffers.
        unsafe { *msg.data.as_mut_ptr().offset(3 + i as isize) = *data.add(i) };
    }

    let _ = send_message(&msg);
}

#[coverage(off)]
#[embassy_executor::task]
pub async fn uart_sender(
    tx_channel: &'static Channel<CsRawMutex, UartData, UART_TX_QUEUE_CAPACITY>,
) {
    // Stay parked until a host actually speaks on UART, so boot banners
    // queued into the UART TX channel cost nothing on a headless light.
    UART_STARTED_SIGNAL.wait().await;

    // Task locals: exactly one sender task ever touches them.
    let mut ack_counter: u8 = 0;
    let mut dropped_uart_tx_seen: u32 = 0;

    loop {
        report_queue_drops(&mut dropped_uart_tx_seen, tx_channel);

        let mut msg = wait_for_outgoing_message(tx_channel).await;
        prepare_outgoing_message(&mut msg, &mut ack_counter);
        send_with_retry(&msg, ack_counter).await;
    }
}

#[coverage(off)]
#[embassy_executor::task]
pub async fn uart_receiver() {
    let mut last_reported_rx_drops: u32 = 0;

    loop {
        let msg = wait_for_received_message().await;
        report_rx_drops(&mut last_reported_rx_drops);

        if let Some(sno) = handle_received_frame(&msg) {
            ack_msg(&msg, &sno).await;
        }
    }
}

/// Whether the UART bridge has been started by an incoming frame.
pub fn started() -> bool {
    UART_SENDER_STARTED.load(Ordering::Relaxed)
}

/// Configure the UART peripheral (GPIO, DMA buffers, 115200 baud).
pub fn init() {
    UART_HW.lock().init();
}

/// Clear any latched UART error flags.
///
/// The driver's status read-modify-write is an associated function, so the
/// guard is bound explicitly to keep it alive across the call: every
/// `UART_STATUS` access goes through the one [`UART_HW`] owner.
pub fn clear_error() {
    let _hw = UART_HW.lock();
    UartDriver::clear_error();
}

/// Queue a UART frame for the [`uart_sender`] task.
///
/// Returns `Err(())` when the transport rejected the frame: the queue was full
/// (and the registered ops counted the drop), or no ops have been registered yet
/// (nothing is counted — there is no transport to count against).
pub fn send_message(msg: &UartData) -> Result<(), ()> {
    if uart_tx_ops_push(msg) {
        Ok(())
    } else {
        Err(())
    }
}

/// Service a UART DMA interrupt.
///
/// Reads and clears the DMA RX/TX flags and copies any received frame out
/// under a short [`UART_HW`] critical section, **drops the guard**, then
/// signals [`UART_TX_DONE`] on TX complete and queues the raw frame for
/// [`uart_receiver`] — counting a drop if that channel is full.
///
/// No validation, logging or status reporting happens here: CRC checks, the
/// bridge start-up and drop logging all run in task context.
#[inline(always)]
pub fn take_rx_and_ack_tx_irq() {
    let (rx_frame, tx_done) = {
        let mut hw = UART_HW.lock();
        let irq_s = UartDriver::get_and_clear_irq_source();

        let rx_frame = if irq_s & UartIrqMask::Rx as u8 != 0 {
            Some(hw.rx_data_buf)
        } else {
            None
        };

        if irq_s & UartIrqMask::Tx as u8 != 0 {
            hw.clear_tx_busy_flag();
        }

        (rx_frame, irq_s & UartIrqMask::Tx as u8 != 0)
    };

    if tx_done {
        UART_TX_DONE.signal(());
    }

    if let Some(frame) = rx_frame {
        queue_rx_frame(frame);
    }
}

/// Queue one raw RX frame and count a drop when the channel is full.
///
/// `#[inline(never)]` on purpose: [`take_rx_and_ack_tx_irq`] is
/// `#[inline(always)]`, so anything inlined here is charged to
/// `handle_system_interrupts` — the frame the IRQ stack gate measures, and that
/// frame is on the deepest (timer) chain, which never runs the UART branch.
/// Keeping the channel internals out of line puts their ~100 B on the UART branch
/// only. Measured: 1436 B -> 1332 B IRQ max, margin 356 B -> 460 B.
#[inline(never)]
fn queue_rx_frame(frame: UartData) {
    if UART_RX_CHANNEL.try_send(frame).is_err() {
        count_rx_drop();
    }
}

/// Bumps [`UART_RX_DROPPED`] from interrupt context.
///
/// `fetch_add` needs a 32-bit atomic read-modify-write, which `thumbv6m`
/// (Cortex-M0) cannot do in hardware, so the load/modify/store runs masked.
/// `sdk::ble_app::queues::count_drop` does the same thing for the shared
/// queues, but it belongs to the SDK, which this module may not name.
fn count_rx_drop() {
    critical_section::with(|_| {
        UART_RX_DROPPED.store(
            UART_RX_DROPPED.load(Ordering::Relaxed).wrapping_add(1),
            Ordering::Relaxed,
        );
    });
}

/// Bumps one of the TX observability counters from task context.
///
/// Same TC32-safe idiom as [`count_rx_drop`]: `thumbv6m` has no 32-bit atomic
/// read-modify-write, so the load/modify/store runs inside a short masked
/// critical section instead of `fetch_add`.
fn count_tx_event(counter: &AtomicU32) {
    critical_section::with(|_| {
        counter.store(
            counter.load(Ordering::Relaxed).wrapping_add(1),
            Ordering::Relaxed,
        );
    });
}

/// Send a frame on the async path, waiting for the previous transfer to finish.
///
/// Holds [`UART_TX_LOCK`] for the whole call and [`UART_HW`] only for the
/// single non-`await` arm block. On the 100 ms deadline the transfer is
/// force-started and the function returns, so a lost TX-complete interrupt
/// degrades to one late frame instead of livelocking the sender.
pub async fn uart_send_async(msg: &UartData) {
    let _tx = UART_TX_LOCK.lock().await;

    #[cfg(not(test))]
    let deadline = Instant::now() + Duration::from_millis(100);
    // Host tests have no time driver; the test-mode wait below ignores it.
    #[cfg(test)]
    let deadline = Instant::MAX;

    loop {
        let started = {
            let mut hw = UART_HW.lock();
            if !hw.is_tx_busy() {
                // Arm the wake-up before transmitting so a TX-complete that
                // lands between the arm and the wait is still observed (the
                // signal latches).
                UART_TX_DONE.reset();
                hw.start_tx_unchecked(msg);
                true
            } else {
                false
            }
        };
        if started {
            return;
        }

        if wait_tx_done(deadline).await.is_err() {
            // 100 ms with no TX-complete: force-start DMA1 cleanly under the
            // critical section and return. Never livelock.
            let mut hw = UART_HW.lock();
            UART_TX_DONE.reset();
            hw.start_tx_unchecked(msg);
            return;
        }
    }
}

/// Blocking UART write for the panic path.
///
/// Delegates to [`UartDriver::send_panic`], the driver's DMA1-safe panic sink:
/// it never waits on a previous transfer, clears a stuck DMA1 channel before
/// arming the frame, and waits out the shift time without parking.
///
/// Must not touch the async queue or [`UART_TX_LOCK`]: a panic while sending
/// would re-enter `panic_fmt` through `try_send`, a call-graph cycle the static
/// stack gate rejects.
///
/// # Safety
///
/// Acquires [`UART_HW`] for the duration of the write, so it must never be
/// called from a context that already holds that guard. `lock` has no
/// production re-entrancy check and `panic = "abort"` never drops the outer
/// guard, so a re-entrant call would hand `UartDriver::send_panic` a second
/// `&mut UartDriver` aliasing the first (see `sdk::common::sync` assumption 3).
pub fn send_sync_panic(msg: &UartData) {
    UART_HW.lock().send_panic(msg);
}

/// Waits for the next UART ack counter, giving up at `deadline`.
///
/// Free function on purpose: the target/host timeout split lives in one place.
#[cfg(not(test))]
async fn wait_ack_or_timeout(deadline: Instant) -> Result<u8, ()> {
    UART_ACK_SIGNAL
        .wait()
        .with_deadline(deadline)
        .await
        .map_err(|_| ())
}

/// Host tests have no time driver, so a missing ack is simply "not ready yet".
#[cfg(test)]
fn wait_ack_or_timeout() -> Result<u8, ()> {
    UART_ACK_SIGNAL.try_take().ok_or(())
}

/// Waits for the TX-complete signal, giving up at `deadline`.
#[cfg(not(test))]
async fn wait_tx_done(deadline: Instant) -> Result<(), ()> {
    UART_TX_DONE
        .wait()
        .with_deadline(deadline)
        .await
        .map_err(|_| ())
}

/// Host tests have no time driver: a missing signal means "not done yet", so
/// the caller takes its timeout branch immediately instead of parking.
#[cfg(test)]
async fn wait_tx_done(_deadline: Instant) -> Result<(), ()> {
    UART_TX_DONE.try_take().ok_or(())
}

async fn ack_msg(msg: &UartData, sno: &[u8; 3]) {
    // If data[1] is 0xff, it means this message is an ack from the client
    if msg.data[1] == u8::from(UartMsg::Ack) {
        // Late when nothing is waiting for an ack, or when the ack answers a
        // frame other than the one in flight. `send_with_retry` owns clearing
        // `UART_AWAITING_ACK`; this path only counts.
        if !UART_AWAITING_ACK.load(Ordering::Relaxed)
            || msg.data[0] != UART_PENDING_COUNTER.load(Ordering::Relaxed)
        {
            count_tx_event(&UART_LATE_ACKS);
        }
        UART_LAST_ACK.store(msg.data[0], Ordering::Relaxed);
        UART_ACK_SIGNAL.signal(msg.data[0]);
        return;
    }

    let result = build_ack(msg, *sno);

    // Send the ack
    uart_send_async(&result).await;
}

/// Surface UART TX queue overflow, one log line per change.
///
/// The SDK cannot log — it must not reach the application's UART sink — so it
/// only counts drops, and the counter is read back through [`UartTxOps`].
/// Turning the delta into a line here keeps a sustained overload visible from
/// the host without a write per iteration.
///
/// Only reported once the backlog has drained past the halfway mark: a log
/// frame pushed into the very queue that overflowed can itself be dropped,
/// which would bump the counter and produce another line — a report storm that
/// keeps the queue saturated. Waiting for headroom means the line fits and the
/// diagnostic cannot amplify the condition it reports.
fn report_queue_drops(
    dropped_uart_tx_seen: &mut u32,
    tx_channel: &Channel<CsRawMutex, UartData, UART_TX_QUEUE_CAPACITY>,
) {
    let total = uart_tx_ops_dropped();
    if total == *dropped_uart_tx_seen {
        return;
    }
    if tx_channel.len() > tx_channel.capacity() / 2 {
        return;
    }

    uprintln!(
        "uart queue full: tx +{} (total {})",
        total.wrapping_sub(*dropped_uart_tx_seen),
        total
    );
    *dropped_uart_tx_seen = total;
}

/// Surface UART RX channel overflow, one log line per change (task context only).
///
/// Mirrors [`report_queue_drops`], minus its headroom guard: the line goes to
/// the sender's queue, which is not the queue that overflowed, so it cannot
/// amplify its own condition.
fn report_rx_drops(last_reported_rx_drops: &mut u32) {
    let total = UART_RX_DROPPED.load(Ordering::Relaxed);
    if total == *last_reported_rx_drops {
        return;
    }

    uprintln!(
        "uart rx queue full: +{} (total {})",
        total.wrapping_sub(*last_reported_rx_drops),
        total
    );
    *last_reported_rx_drops = total;
}

/// Validate and handle one frame in task context.
///
/// Returns the sno to ACK, or `None` for an invalid-CRC frame — invalid frames
/// are never ACKed and never start the bridge, exactly as before the RX path
/// moved out of the interrupt handler.
fn handle_received_frame(msg: &UartData) -> Option<[u8; 3]> {
    if !verify_crc(msg) {
        return None;
    }

    // The first valid frame is what starts the bridge: wake the parked
    // `uart_sender` and let the application enable the status reporting the
    // host expects (before the initial dump that enabling performs).
    if !UART_SENDER_STARTED.swap(true, Ordering::Relaxed) {
        UART_STARTED_SIGNAL.signal(());
        dispatch_uart_cmd(UartCommand::HostAttached);
    }

    match decode_command(msg) {
        Some(cmd) => {
            // Record what we are about to relay before the mesh can echo it
            // back through `on_mesh_rx_packet`.
            if let UartCommand::LightCtrl { .. } = cmd {
                record_sent_fingerprint(fingerprint(msg));
            }
            Some(dispatch_uart_cmd(cmd))
        }
        // Nothing to execute, but the frame was valid and is still acknowledged.
        None => Some([0; 3]),
    }
}

async fn wait_for_received_message() -> UartData {
    UART_RX_CHANNEL.receive().await
}

async fn wait_for_outgoing_message(
    tx_channel: &Channel<CsRawMutex, UartData, UART_TX_QUEUE_CAPACITY>,
) -> UartData {
    tx_channel.receive().await
}

fn record_sent_fingerprint(fingerprint: Fingerprint) {
    let mut sent = UART_SENT_DEDUP.lock();
    if sent.is_full() {
        sent.pop_front();
    }

    let push_result = sent.push_back(fingerprint);
    debug_assert!(push_result.is_ok());
    let _ = push_result;
}

fn prepare_outgoing_message(msg: &mut UartData, ack_counter: &mut u8) {
    record_sent_fingerprint(outgoing_fingerprint(msg));

    *ack_counter = ack_counter.wrapping_add(1);
    *msg = ack_retry_frame(msg, *ack_counter);
}

/// Transmit one frame and wait for its ack, at most [`MAX_TX_ATTEMPTS`] times.
///
/// The frame identity is published before each transmit so an ack that lands
/// before the wait is matched, not counted as late. A host that never acks
/// costs at most three attempts, after which the frame is counted in
/// [`UART_TX_DROPPED`] and abandoned — `uart_sender` then advances to the next
/// queued frame instead of wedging on this one.
///
/// The drop is logged once per unresponsive episode, not once per frame. The
/// log line is itself a frame on [`UART_TX_CHANNEL`], so it travels back
/// through this same function: a host that never acks would drop and log the
/// log, forever. [`UART_DROP_LOG_PENDING`] suppresses repeats until a matching
/// ack proves the host is listening again.
async fn send_with_retry(msg: &UartData, ack_counter: u8) {
    let mut attempt: u8 = 1;

    loop {
        // Publish the frame identity *before* transmitting: a fast ack that
        // arrives between the send and the wait must match this counter rather
        // than look late. `ack_msg` reads both, so the order is load-bearing.
        UART_PENDING_COUNTER.store(ack_counter, Ordering::Relaxed);
        UART_AWAITING_ACK.store(true, Ordering::Relaxed);

        // Arm the wake-up before transmitting so an ack that lands between
        // the send and the wait is still observed (the value is latched).
        UART_ACK_SIGNAL.reset();
        uart_send_async(msg).await;

        if UART_LAST_ACK.load(Ordering::Relaxed) == ack_counter {
            UART_AWAITING_ACK.store(false, Ordering::Relaxed);
            UART_DROP_LOG_PENDING.store(false, Ordering::Relaxed);
            return;
        }

        // One deadline for the whole wait: stale acks must not extend it.
        #[cfg(not(test))]
        let deadline = Instant::now() + Duration::from_millis(100);

        loop {
            #[cfg(not(test))]
            {
                match wait_ack_or_timeout(deadline).await {
                    Ok(ack) if ack == ack_counter => {
                        UART_AWAITING_ACK.store(false, Ordering::Relaxed);
                        UART_DROP_LOG_PENDING.store(false, Ordering::Relaxed);
                        return;
                    }
                    Ok(_) => continue,
                    Err(()) => break,
                }
            }
            #[cfg(test)]
            {
                match wait_ack_or_timeout() {
                    Ok(ack) if ack == ack_counter => {
                        UART_AWAITING_ACK.store(false, Ordering::Relaxed);
                        UART_DROP_LOG_PENDING.store(false, Ordering::Relaxed);
                        return;
                    }
                    Ok(_) => continue,
                    Err(()) => break,
                }
            }
        }

        // 100 ms with no matching ack.
        if attempt >= MAX_TX_ATTEMPTS {
            UART_AWAITING_ACK.store(false, Ordering::Relaxed);
            count_tx_event(&UART_TX_DROPPED);
            if !UART_DROP_LOG_PENDING.swap(true, Ordering::Relaxed) {
                uprintln!("uart tx drop: counter {} attempts {}", ack_counter, attempt);
            }
            return;
        }

        count_tx_event(&UART_TX_RETRIES);
        attempt += 1;
    }
}

/// Reset every piece of module state a host test can observe.
///
/// This is the replacement for the deleted singleton reset helper: host tests
/// share one process, so `App::default()` can no longer hand out a fresh
/// manager. `UART_TX_LOCK` needs no reset — it is only held inside an
/// `.await`-free critical window and is always released by dropping a guard.
/// The registered command handler is cleared, so a test that needs one
/// registers its own; the test-only TX ops are restored.
#[cfg(test)]
pub fn reset_for_test() {
    *UART_HW.lock() = UartDriver::default_const();
    UART_SENT_DEDUP.lock().clear();
    UART_SENDER_STARTED.store(false, Ordering::Relaxed);
    UART_LAST_ACK.store(0, Ordering::Relaxed);
    UART_RX_DROPPED.store(0, Ordering::Relaxed);
    UART_TX_RETRIES.store(0, Ordering::Relaxed);
    UART_TX_DROPPED.store(0, Ordering::Relaxed);
    UART_LATE_ACKS.store(0, Ordering::Relaxed);
    UART_PENDING_COUNTER.store(0, Ordering::Relaxed);
    UART_AWAITING_ACK.store(false, Ordering::Relaxed);
    UART_DROP_LOG_PENDING.store(false, Ordering::Relaxed);
    UART_STARTED_SIGNAL.reset();
    UART_ACK_SIGNAL.reset();
    UART_TX_DONE.reset();
    while UART_RX_CHANNEL.try_receive().is_ok() {}
    set_tx_ops(TEST_TX_OPS);
    *UART_COMMAND_HANDLER.lock() = None;
    tlsr8266_radio::queues::reset_for_test();
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use core::future::Future;
    use core::sync::atomic::AtomicUsize;
    use futures::executor::block_on;
    use futures::task::noop_waker;
    use mesh_protocol::crc::crc16;
    use mesh_protocol::packet_types::{PacketL2capHead, PacketLlApp};
    use std::sync::Mutex as StdMutex;
    use std::task::{Context, Poll};
    use tlsr8266_hal::mcu::clock::fake_clock;
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_radio::queues::UART_TX_CHANNEL;

    /// `reg_dma_rx_rdy0`: DMA/UART interrupt flags, write-1-to-clear.
    const REG_DMA_RX_RDY0: u32 = 0x526;

    /// `reg_dma_tx_rdy0`: write to trigger the DMA1 transfer.
    const REG_DMA_TX_RDY0: u32 = 0x524;
    /// Number of DMA1 transmit triggers, i.e. `start_tx_unchecked` arms.
    fn dma1_arms() -> usize {
        FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == REG_DMA_TX_RDY0 | tlsr8266_pac::REG_BASE_ADDR)
            .count()
    }

    /// DMA1 arm count observed by the ack injector when it latched the ack.
    static INJECTED_AT_ARMS: AtomicUsize = AtomicUsize::new(0);

    fn reset_fakes() {
        FakeMmio::reset();
        fake_clock::reset();
    }

    /// Create a test packet with the given parameters
    fn create_test_packet(
        sno: [u8; 3],
        src: u16,
        dst: u16,
        op: u8,
        vendor_id: u16,
        par: [u8; 10],
    ) -> Packet {
        Packet {
            ll_app: PacketLlApp {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: 0,
                    rf_len: 0,
                    l2cap_len: 0,
                    chan_id: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: AppCmdValue {
                    sno,
                    src,
                    dst,
                    op,
                    vendor_id,
                    par,
                },
                rsv: [0; 10],
            },
        }
    }

    /// Builds a `UartData` frame with a valid CRC, matching the pattern used by
    /// the `take_rx_and_ack_tx_irq`/`handle_received_frame` tests below.
    fn frame_with_valid_crc() -> UartData {
        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };

        for (idx, byte) in msg.data.iter_mut().take(CRC_RANGE.end).enumerate() {
            *byte = idx as u8;
        }

        compute_crc(&mut msg);
        msg
    }

    /// A light-control frame with a valid CRC and every field distinct, so a
    /// decode that reads the wrong offset cannot pass.
    fn light_ctrl_frame() -> UartData {
        let mut msg = empty_frame();
        msg.data[MSG_TYPE_INDEX] = u8::from(UartMsg::LightCtrl);
        msg.data[CTRL_DESTINATION_RANGE].copy_from_slice(&u16::to_le_bytes(0x1234));
        for (idx, byte) in msg.data[CTRL_PAYLOAD_RANGE].iter_mut().enumerate() {
            *byte = (idx as u8) + 1;
        }
        msg.data[CTRL_RETRANSMIT_INDEX] = 2;
        msg.data[CTRL_ACK_FLAG_INDEX] = 1;
        compute_crc(&mut msg);
        msg
    }

    fn empty_frame() -> UartData {
        UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        }
    }

    /// Commands handed to the test handler, in dispatch order.
    static DISPATCHED: StdMutex<heapless::Vec<UartCommand, 8>> =
        StdMutex::new(heapless::Vec::new());

    /// Test handler: records the command and answers with the contract `app::
    /// uart_command_hook` implements — a real sno for `LightCtrl`, zeros else.
    fn recording_handler(cmd: UartCommand) -> [u8; 3] {
        let _ = DISPATCHED.lock().unwrap().push(cmd);
        match cmd {
            UartCommand::LightCtrl { .. } => [4, 5, 6],
            _ => [0; 3],
        }
    }

    fn dispatched() -> heapless::Vec<UartCommand, 8> {
        DISPATCHED.lock().unwrap().clone()
    }

    fn reset_recorder() {
        DISPATCHED.lock().unwrap().clear();
    }

    /// Marks the bridge as started, so the next frame is not the one that
    /// dispatches `HostAttached`.
    fn start_bridge() {
        UART_SENDER_STARTED.store(true, Ordering::Relaxed);
    }

    #[test]
    fn test_wait_for_received_message_returns_message() {
        reset_for_test();
        let mut msg = UartData {
            len: 1,
            data: [0; UART_DATA_LEN],
        };
        msg.data[0] = 0xAA;
        let expected_len = msg.len;
        let expected_data = msg.data;
        UART_RX_CHANNEL.try_send(msg).unwrap();

        let result = block_on(wait_for_received_message());

        assert_eq!(result.len, expected_len);
        assert_eq!(result.data, expected_data);
    }

    #[test]
    fn test_wait_for_outgoing_message_returns_message() {
        reset_for_test();
        let mut msg = UartData {
            len: 2,
            data: [0; UART_DATA_LEN],
        };
        msg.data[0] = 0xAB;
        msg.data[1] = 0xCD;
        let expected_len = msg.len;
        let expected_data = msg.data;
        UART_TX_CHANNEL.try_send(msg).unwrap();

        let result = block_on(wait_for_outgoing_message(&UART_TX_CHANNEL));

        assert_eq!(result.len, expected_len);
        assert_eq!(result.data, expected_data);
    }

    #[test]
    fn test_wait_for_outgoing_message_waits_until_message_available() {
        reset_for_test();
        let future = wait_for_outgoing_message(&UART_TX_CHANNEL);
        futures::pin_mut!(future);

        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);

        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Pending));

        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };
        msg.data[0] = 0xEF;
        UART_TX_CHANNEL.try_send(msg).unwrap();

        let poll_result = future.as_mut().poll(&mut cx);
        assert!(
            matches!(poll_result, Poll::Ready(_)),
            "future should be ready after message enqueued"
        );
        let Poll::Ready(result) = poll_result else {
            unreachable!()
        };
        assert_eq!(result.data[0], 0xEF);
    }

    /// A light-control frame is dispatched to the registered handler and its
    /// `dst`..`par` fingerprint is recorded before the mesh can echo it back.
    #[test]
    fn test_handle_received_frame_dispatches_light_ctrl_and_records_fingerprint() {
        reset_for_test();
        reset_recorder();
        set_command_handler(recording_handler);
        start_bridge();

        let msg = light_ctrl_frame();

        let sno = handle_received_frame(&msg);

        assert_eq!(sno, Some([4, 5, 6]));
        assert_eq!(
            dispatched().as_slice(),
            &[UartCommand::LightCtrl {
                dst: 0x1234,
                payload: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13],
                retransmit: 2,
                ack: true,
            }]
        );

        let expected_fingerprint = fingerprint(&msg);
        let sent = UART_SENT_DEDUP.lock();
        assert_eq!(sent.len(), 1);
        assert_eq!(sent.front().unwrap(), &expected_fingerprint);
    }

    /// A frame whose command byte is not dispatched is still acknowledged, but
    /// nothing is executed and nothing is recorded.
    #[test]
    fn test_handle_received_frame_unknown_type_returns_zero_and_dispatches_nothing() {
        reset_for_test();
        reset_recorder();
        set_command_handler(recording_handler);
        start_bridge();

        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0x99; UART_DATA_LEN],
        };
        compute_crc(&mut msg);

        let sno = handle_received_frame(&msg);

        assert_eq!(sno, Some([0; 3]));
        assert!(dispatched().is_empty());
        assert!(UART_SENT_DEDUP.lock().is_empty());
    }

    /// A light-status request is dispatched as `EnableStatusReporting`, which is
    /// the command that re-runs the node-status dump.
    #[test]
    fn test_handle_received_frame_light_status_requests_reporting() {
        reset_for_test();
        reset_recorder();
        set_command_handler(recording_handler);
        start_bridge();

        let mut msg = empty_frame();
        msg.data[MSG_TYPE_INDEX] = u8::from(UartMsg::LightStatus);
        compute_crc(&mut msg);

        let sno = handle_received_frame(&msg);

        assert_eq!(sno, Some([0; 3]));
        assert_eq!(
            dispatched().as_slice(),
            &[UartCommand::EnableStatusReporting]
        );
    }

    #[test]
    fn test_prepare_outgoing_message_updates_counter_crc_and_fingerprint() {
        reset_for_test();
        let mut ack_counter: u8 = 0;

        let mut msg = empty_frame();

        for (idx, byte) in msg.data[OUTGOING_FINGERPRINT_RANGE].iter_mut().enumerate() {
            *byte = (idx as u8) ^ 0xAA;
        }
        prepare_outgoing_message(&mut msg, &mut ack_counter);

        assert_eq!(ack_counter, 1);
        assert_eq!(msg.data[0], 1);

        let expected_crc = crc16(&msg.data[CRC_RANGE]);
        assert_eq!(msg.data[42], (expected_crc & 0xff) as u8);
        assert_eq!(msg.data[43], ((expected_crc >> 8) & 0xff) as u8);

        let expected_fingerprint = outgoing_fingerprint(&msg);
        let sent = UART_SENT_DEDUP.lock();
        assert_eq!(sent.len(), 1);
        assert_eq!(sent.front().unwrap(), &expected_fingerprint);
    }

    /// An ack that landed before the wait ends the frame on the first attempt:
    /// no retransmit, no drop, and the in-flight flag is cleared again.
    #[test]
    fn test_send_with_retry_exits_when_ack_already_received() {
        reset_for_test();
        reset_fakes();
        UART_LAST_ACK.store(5, Ordering::Relaxed);

        let msg = empty_frame();

        block_on(send_with_retry(&msg, 5));

        assert_eq!(dma1_arms(), 1, "DMA1 must be triggered once");
        assert_eq!(
            UART_TX_RETRIES.load(Ordering::Relaxed),
            0,
            "a first-attempt ack must not count a retransmit"
        );
        assert_eq!(UART_TX_DROPPED.load(Ordering::Relaxed), 0);
        assert!(
            !UART_AWAITING_ACK.load(Ordering::Relaxed),
            "the in-flight flag must be cleared once the frame is acked"
        );
    }

    /// The retry loop re-arms DMA1 when the ack for its counter has not
    /// arrived, and exits once it has.
    ///
    /// The ack is delivered from a helper thread the moment the retransmission
    /// is visible in the register write log. That is the same point a stub used
    /// to inject it at (the second `write_reg_dma1_addr`), but observed through
    /// the real PAC path: the accessor is a real write, so the delivery has to
    /// come from outside the synchronous retry loop.
    #[test]
    fn test_send_with_retry_retransmits_until_ack_is_received() {
        reset_for_test();
        reset_fakes();
        UART_LAST_ACK.store(0, Ordering::Relaxed);
        INJECTED_AT_ARMS.store(0, Ordering::SeqCst);

        let injector = std::thread::spawn(|| {
            // The loop arms DMA1 within microseconds, so this normally returns
            // immediately. The bound exists so that a regression fails the
            // assertion below instead of leaving the retry loop spinning.
            for _ in 0..100_000 {
                if dma1_arms() >= 2 {
                    break;
                }
                std::thread::yield_now();
            }
            // Latch the matching ack unconditionally, so the loop can always
            // finish even when it never retransmitted.
            INJECTED_AT_ARMS.store(dma1_arms(), Ordering::SeqCst);
            UART_LAST_ACK.store(1, Ordering::Relaxed);
        });

        let msg = empty_frame();

        // The first wait finds no ack (host mode: `try_take` -> Err) and
        // retransmits; the injected ack ends the loop.
        block_on(send_with_retry(&msg, 1));
        injector.join().expect("ack injector must not panic");

        assert!(
            INJECTED_AT_ARMS.load(Ordering::SeqCst) >= 2,
            "DMA1 must be re-armed when the ack for the sent counter is missing"
        );
    }

    /// A host that never acks costs exactly [`MAX_TX_ATTEMPTS`] transmits, then
    /// the frame is dropped and the function returns so `uart_sender` can move
    /// on to the next queued frame.
    ///
    /// The host `wait_ack_or_timeout` takes the timeout branch immediately
    /// (there is no time driver), so an unbounded retry loop would hang here.
    #[test]
    fn test_send_with_retry_drops_after_max_attempts() {
        reset_for_test();
        reset_fakes();
        UART_LAST_ACK.store(0, Ordering::Relaxed);

        let msg = empty_frame();

        block_on(send_with_retry(&msg, 1));

        assert_eq!(
            dma1_arms(),
            MAX_TX_ATTEMPTS as usize,
            "one transmit plus one per retransmit, then stop"
        );
        assert_eq!(
            UART_TX_RETRIES.load(Ordering::Relaxed),
            u32::from(MAX_TX_ATTEMPTS - 1),
            "retries count retransmits only"
        );
        assert_eq!(
            UART_TX_DROPPED.load(Ordering::Relaxed),
            1,
            "an exhausted frame is counted once"
        );
        assert!(
            !UART_AWAITING_ACK.load(Ordering::Relaxed),
            "a dropped frame must not leave the in-flight flag set"
        );
        assert_eq!(
            UART_TX_CHANNEL.len(),
            1,
            "the drop must produce exactly one log line"
        );
    }

    /// A host that never acks must not amplify its own drop logs: the log line
    /// is itself a frame on `UART_TX_CHANNEL`, so it travels back through
    /// `send_with_retry` and would be dropped and logged again, forever.
    ///
    /// Two consecutive exhausted frames therefore count two drops but emit one
    /// log line.
    #[test]
    fn test_send_with_retry_logs_one_drop_per_unresponsive_episode() {
        reset_for_test();
        reset_fakes();
        UART_LAST_ACK.store(0, Ordering::Relaxed);

        let msg = empty_frame();

        block_on(send_with_retry(&msg, 1));
        block_on(send_with_retry(&msg, 2));

        assert_eq!(
            UART_TX_DROPPED.load(Ordering::Relaxed),
            2,
            "every exhausted frame is still counted"
        );
        assert_eq!(
            UART_TX_CHANNEL.len(),
            1,
            "the second drop in the same unresponsive episode must not log again"
        );
    }

    /// A matching ack proves the host is listening again, so the guard is
    /// cleared and the next unresponsive episode logs its own drop.
    #[test]
    fn test_send_with_retry_matching_ack_rearms_drop_log() {
        reset_for_test();
        reset_fakes();
        UART_LAST_ACK.store(0, Ordering::Relaxed);

        let msg = empty_frame();

        block_on(send_with_retry(&msg, 1));
        assert_eq!(
            UART_TX_CHANNEL.len(),
            1,
            "the first unresponsive episode logs once"
        );

        // Fast path: the ack for this counter is already latched.
        UART_LAST_ACK.store(2, Ordering::Relaxed);
        block_on(send_with_retry(&msg, 2));
        assert!(
            !UART_DROP_LOG_PENDING.load(Ordering::Relaxed),
            "a matching ack must re-arm the drop log"
        );

        // A fresh episode after the ack logs again.
        UART_LAST_ACK.store(0, Ordering::Relaxed);
        block_on(send_with_retry(&msg, 3));
        assert_eq!(
            UART_TX_CHANNEL.len(),
            2,
            "a new episode after an ack logs again"
        );
    }

    /// `reset_for_test()` clears the drop-log guard, so one test's unresponsive
    /// episode cannot silence the next test's drop log.
    #[test]
    fn test_reset_for_test_clears_drop_log_guard() {
        reset_for_test();
        UART_DROP_LOG_PENDING.store(true, Ordering::Relaxed);

        reset_for_test();

        assert!(
            !UART_DROP_LOG_PENDING.load(Ordering::Relaxed),
            "reset_for_test must clear the drop-log guard"
        );
    }

    /// An ack with no frame in flight is unsolicited: it is counted as late and
    /// still latches `UART_LAST_ACK` for the fast path.
    #[test]
    fn test_ack_msg_counts_unsolicited_ack_as_late() {
        reset_for_test();

        let mut ack = empty_frame();
        ack.data[0] = 3;
        ack.data[1] = u8::from(UartMsg::Ack);

        block_on(ack_msg(&ack, &[0, 0, 0]));

        assert_eq!(UART_LATE_ACKS.load(Ordering::Relaxed), 1);
        assert_eq!(UART_LAST_ACK.load(Ordering::Relaxed), 3);
        assert_eq!(UART_ACK_SIGNAL.try_take(), Some(3));
    }

    /// An ack for a counter other than the frame in flight is stale: it is
    /// counted as late even while a frame is awaiting its own ack.
    #[test]
    fn test_ack_msg_counts_stale_ack_as_late() {
        reset_for_test();
        UART_PENDING_COUNTER.store(7, Ordering::Relaxed);
        UART_AWAITING_ACK.store(true, Ordering::Relaxed);

        let mut ack = empty_frame();
        ack.data[0] = 6;
        ack.data[1] = u8::from(UartMsg::Ack);

        block_on(ack_msg(&ack, &[0, 0, 0]));

        assert_eq!(UART_LATE_ACKS.load(Ordering::Relaxed), 1);
        assert!(
            UART_AWAITING_ACK.load(Ordering::Relaxed),
            "ack_msg must not clear the in-flight flag; the retry loop owns it"
        );
    }

    /// An ack for the frame in flight is not late.
    #[test]
    fn test_ack_msg_matching_ack_is_not_late() {
        reset_for_test();
        UART_PENDING_COUNTER.store(7, Ordering::Relaxed);
        UART_AWAITING_ACK.store(true, Ordering::Relaxed);

        let mut ack = empty_frame();
        ack.data[0] = 7;
        ack.data[1] = u8::from(UartMsg::Ack);

        block_on(ack_msg(&ack, &[0, 0, 0]));

        assert_eq!(UART_LATE_ACKS.load(Ordering::Relaxed), 0);
        assert_eq!(UART_ACK_SIGNAL.try_take(), Some(7));
    }

    /// Tests that an incoming ack frame wakes a waiter parked on
    /// `UART_ACK_SIGNAL`.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state and pin `UART_ACK_SIGNAL.wait()`
    /// 2. Poll once with a noop waker: must be `Pending`
    /// 3. Feed an ack frame (`data[1] == 0xff`) through `ack_msg`
    /// 4. Poll again: must be `Ready` with the acked counter
    #[test]
    fn test_ack_signal_wakes_waiter_after_ack_msg() {
        reset_for_test();
        let future = UART_ACK_SIGNAL.wait();
        futures::pin_mut!(future);

        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);

        assert!(
            matches!(future.as_mut().poll(&mut cx), Poll::Pending),
            "ack_signal should start unsignalled"
        );

        let expected = 3u8;
        let mut ack = empty_frame();
        ack.data[0] = expected;
        ack.data[1] = u8::from(UartMsg::Ack);

        block_on(ack_msg(&ack, &[0, 0, 0]));

        let poll_result = future.as_mut().poll(&mut cx);
        assert!(
            matches!(poll_result, Poll::Ready(_)),
            "future should be ready after the ack frame is handled"
        );
        let Poll::Ready(result) = poll_result else {
            unreachable!()
        };
        assert_eq!(result, expected);
        assert_eq!(UART_LAST_ACK.load(Ordering::Relaxed), expected);
    }

    #[test]
    fn test_record_sent_fingerprint_evicts_oldest_when_full() {
        reset_for_test();

        {
            let mut sent = UART_SENT_DEDUP.lock();
            for value in 0..12u8 {
                sent.push_back([value; 15]).unwrap();
            }
        }

        let new_fingerprint = [0xAA; 15];

        record_sent_fingerprint(new_fingerprint);

        let sent = UART_SENT_DEDUP.lock();
        assert_eq!(sent.len(), 12);
        assert_eq!(sent.front().unwrap(), &[1u8; 15]);
        assert_eq!(sent.back().unwrap(), &new_fingerprint);
    }

    /// Tests the send_message function when the channel has space.
    ///
    /// This test verifies that send_message correctly adds a message to the send queue
    /// when there's space available.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state with an empty send channel
    /// 2. Create a test message
    /// 3. Call send_message with the test message
    /// 4. Verify the function returns Ok(()) (message accepted)
    #[test]
    fn test_send_message_with_space() {
        reset_for_test();

        let msg = empty_frame();

        let result = send_message(&msg);

        assert!(result.is_ok());
        assert_eq!(UART_TX_CHANNEL.len(), 1);
    }

    /// Tests the send_message function when the channel is full.
    ///
    /// This test verifies that send_message correctly rejects messages
    /// when the send channel is full.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state
    /// 2. Fill the send channel to capacity
    /// 3. Try to send one more message
    /// 4. Verify the function returns Err(()) (message rejected)
    #[test]
    fn test_send_message_when_full() {
        reset_for_test();

        let msg = empty_frame();

        let capacity = 12; // This matches UART_TX_QUEUE_CAPACITY
        for _ in 0..capacity {
            let result = send_message(&msg);
            assert!(result.is_ok());
        }

        let result = send_message(&msg);

        assert!(result.is_err());
        assert_eq!(UART_TX_CHANNEL.len(), capacity);
    }

    /// An unregistered transport drops the frame instead of routing it: the
    /// production static starts empty, so nothing is silently written.
    #[test]
    fn test_send_message_without_registered_ops_is_dropped() {
        reset_for_test();
        *UART_TX_OPS.lock() = None;

        assert_eq!(send_message(&empty_frame()), Err(()));
        assert_eq!(uart_tx_ops_dropped(), 0);
        assert_eq!(UART_TX_CHANNEL.len(), 0);
    }

    /// Tests that an invalid-CRC frame is rejected in task context.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state (sender not started)
    /// 2. Build a frame whose CRC trailer does not match its payload
    /// 3. Call `handle_received_frame`
    /// 4. Verify it returns `None`, signals nothing and dispatches nothing
    #[test]
    fn test_handle_received_frame_invalid_crc_returns_none() {
        reset_for_test();
        reset_recorder();
        set_command_handler(recording_handler);

        let mut msg = frame_with_valid_crc();
        msg.data[42] ^= 0xff;

        let sno = handle_received_frame(&msg);

        assert!(sno.is_none());
        assert!(!UART_STARTED_SIGNAL.signaled());
        assert!(!started());
        assert!(dispatched().is_empty());
    }

    /// Tests that the first valid frame starts the bridge exactly once, and that
    /// it dispatches `HostAttached` *before* the frame's own command so the
    /// application can enable status reporting before the initial node dump.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state and register the recording handler
    /// 2. Handle one valid frame: `Some(sno)`, start signal latched, started,
    ///    and the handler saw `HostAttached` then `LightCtrl`
    /// 3. Handle a second valid frame: the bridge start-up must not run again
    #[test]
    fn test_handle_received_frame_first_valid_starts_bridge_once() {
        reset_for_test();
        reset_recorder();
        set_command_handler(recording_handler);

        let first = light_ctrl_frame();
        assert!(handle_received_frame(&first).is_some());

        assert!(UART_STARTED_SIGNAL.signaled());
        assert!(started());

        let seq = dispatched();
        assert_eq!(seq.len(), 2);
        assert_eq!(seq[0], UartCommand::HostAttached);
        assert!(matches!(seq[1], UartCommand::LightCtrl { .. }));

        let second = light_ctrl_frame();
        assert!(handle_received_frame(&second).is_some());

        let seq = dispatched();
        assert_eq!(seq.len(), 3, "the bridge must start exactly once");
        assert!(matches!(seq[2], UartCommand::LightCtrl { .. }));
    }

    /// The ACK frame sent for a command carries the mesh sequence number the
    /// registered handler returned, at `SNO_RANGE` (`2..5`) — not zeros.
    #[test]
    fn test_ack_frame_carries_the_handler_sno_at_sno_range() {
        reset_for_test();
        reset_recorder();
        set_command_handler(recording_handler);
        start_bridge();

        let msg = light_ctrl_frame();

        let sno = handle_received_frame(&msg).expect("a valid frame is acknowledged");
        assert_eq!(sno, [4, 5, 6], "the handler's sno must be the ACK payload");

        let ack = build_ack(&msg, sno);

        assert_eq!(ack.data[SNO_RANGE], [4, 5, 6]);
        assert_eq!(ack.data[2], 4);
        assert_eq!(ack.data[4], 6);
        assert_eq!(ack.data[1], u8::from(UartMsg::Ack));
        assert!(verify_crc(&ack), "the ack must carry a valid CRC");
    }

    /// Tests that a fresh module state has not signalled the started signal.
    #[test]
    fn test_started_signal_not_signaled_before_first_frame() {
        reset_for_test();

        assert!(!UART_STARTED_SIGNAL.signaled());
    }

    /// Tests that an RX drop is surfaced once per change, from task context.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state; an unchanged counter must log nothing
    /// 2. Bump `UART_RX_DROPPED` twice and report: one or more frames land in
    ///    the TX channel and the local seen-total follows
    /// 3. Report again without a new drop: no additional frame is enqueued
    #[test]
    fn test_report_rx_drops_logs_one_line_per_change() {
        reset_for_test();
        let mut last_reported = 0;

        report_rx_drops(&mut last_reported);
        assert_eq!(UART_TX_CHANNEL.len(), 0);
        assert_eq!(last_reported, 0);

        UART_RX_DROPPED.store(2, Ordering::Relaxed);
        report_rx_drops(&mut last_reported);

        assert!(
            UART_TX_CHANNEL.len() >= 1,
            "a drop delta must produce a log frame"
        );
        assert_eq!(last_reported, 2);
        let logged = UART_TX_CHANNEL.len();

        report_rx_drops(&mut last_reported);
        assert_eq!(
            UART_TX_CHANNEL.len(),
            logged,
            "an unchanged counter must not log again"
        );
    }

    /// Tests the started function.
    #[test]
    fn test_started() {
        reset_for_test();

        assert!(!started());

        UART_SENDER_STARTED.store(true, Ordering::Relaxed);

        assert!(started());
    }

    /// Tests that `reset_for_test()` restores every observable module static.
    ///
    /// This is the contract `sdk/` test helpers rely on (it replaces the
    /// deleted singleton reset helper).
    #[test]
    fn test_reset_for_test_restores_defaults() {
        reset_for_test();

        // Dirty every static a test can reach.
        UART_SENT_DEDUP.lock().push_back([1u8; 15]).unwrap();
        UART_SENDER_STARTED.store(true, Ordering::Relaxed);
        UART_LAST_ACK.store(7, Ordering::Relaxed);
        UART_RX_DROPPED.store(3, Ordering::Relaxed);
        UART_TX_RETRIES.store(2, Ordering::Relaxed);
        UART_TX_DROPPED.store(1, Ordering::Relaxed);
        UART_LATE_ACKS.store(4, Ordering::Relaxed);
        UART_PENDING_COUNTER.store(9, Ordering::Relaxed);
        UART_AWAITING_ACK.store(true, Ordering::Relaxed);
        UART_DROP_LOG_PENDING.store(true, Ordering::Relaxed);
        UART_STARTED_SIGNAL.signal(());
        UART_ACK_SIGNAL.signal(1);
        UART_TX_DONE.signal(());
        UART_RX_CHANNEL.try_send(empty_frame()).unwrap();
        send_message(&empty_frame()).unwrap();
        set_command_handler(recording_handler);

        reset_for_test();

        assert!(!started());
        assert!(!UART_STARTED_SIGNAL.signaled());
        assert_eq!(UART_LAST_ACK.load(Ordering::Relaxed), 0);
        assert_eq!(UART_RX_DROPPED.load(Ordering::Relaxed), 0);
        assert_eq!(UART_TX_RETRIES.load(Ordering::Relaxed), 0);
        assert_eq!(UART_TX_DROPPED.load(Ordering::Relaxed), 0);
        assert_eq!(UART_LATE_ACKS.load(Ordering::Relaxed), 0);
        assert_eq!(UART_PENDING_COUNTER.load(Ordering::Relaxed), 0);
        assert!(!UART_AWAITING_ACK.load(Ordering::Relaxed));
        assert!(!UART_DROP_LOG_PENDING.load(Ordering::Relaxed));
        assert!(UART_ACK_SIGNAL.try_take().is_none());
        assert!(UART_TX_DONE.try_take().is_none());
        assert_eq!(UART_RX_CHANNEL.len(), 0);
        assert!(UART_SENT_DEDUP.lock().is_empty());
        assert_eq!(UART_TX_CHANNEL.len(), 0);
        assert!(!UART_HW.lock().is_tx_busy());
        assert_eq!(
            dispatch_uart_cmd(UartCommand::HostAttached),
            [0; 3],
            "the registered command handler must be cleared"
        );
    }

    /// Tests the on_mesh_rx_packet function when UART is not started.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state
    /// 2. Create a test packet
    /// 3. Ensure UART is not started
    /// 4. Call on_mesh_rx_packet with the test packet
    /// 5. Verify no message was sent to UART
    #[test]
    fn test_on_mesh_rx_packet_not_started() {
        reset_for_test();

        let packet = create_test_packet([1, 2, 3], 0x1234, 0x5678, 0x01, 0xABCD, [0; 10]);

        on_mesh_rx_packet(&packet, 0);

        assert_eq!(UART_TX_CHANNEL.len(), 0);
    }

    /// Tests the on_mesh_rx_packet function when message is from our device.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state
    /// 2. Create a test packet with our device address as source
    /// 3. Start the bridge
    /// 4. Call on_mesh_rx_packet with the test packet
    /// 5. Verify no message was sent to UART
    #[test]
    fn test_on_mesh_rx_packet_from_our_device() {
        reset_for_test();
        let own_dev_addr = 0x1234;

        let packet = create_test_packet([1, 2, 3], own_dev_addr, 0x5678, 0x01, 0xABCD, [0; 10]);

        start_bridge();

        on_mesh_rx_packet(&packet, own_dev_addr);

        assert_eq!(UART_TX_CHANNEL.len(), 0);
    }

    /// Tests the on_mesh_rx_packet function when message is a duplicate.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state
    /// 2. Create a test packet
    /// 3. Start the bridge
    /// 4. Add the message to the sent deque
    /// 5. Call on_mesh_rx_packet with the test packet
    /// 6. Verify no message was sent to UART
    #[test]
    fn test_on_mesh_rx_packet_duplicate_message() {
        reset_for_test();

        let packet = create_test_packet([1, 2, 3], 0x1234, 0x5678, 0x01, 0xABCD, [0; 10]);

        start_bridge();

        // Add the message to the sent deque (simulating we sent this message)
        // The on_mesh_rx_packet function extracts data from offset 5 (dst) for 15 bytes
        // This corresponds to: dst(2) + op(1) + vendor_id(2) + par(10) = 15 bytes
        let msg_data = [0x78, 0x56, 0x01, 0xCD, 0xAB, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        UART_SENT_DEDUP.lock().push_back(msg_data).unwrap();

        on_mesh_rx_packet(&packet, 0);

        assert_eq!(UART_TX_CHANNEL.len(), 0);
    }

    /// Tests the on_mesh_rx_packet function with a valid new message.
    ///
    /// This is also the 64-bit regression: the `AppCmdValue` view is taken with
    /// pointer arithmetic (`data.add(5)`), never by truncating the address to
    /// 32 bits, so every byte of the 20-byte wire struct arrives intact on a
    /// host whose pointers are wider than the target's.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state
    /// 2. Create a test packet with valid data
    /// 3. Start the bridge
    /// 4. Call on_mesh_rx_packet with the test packet
    /// 5. Verify a message was sent to UART with correct format
    #[test]
    fn test_on_mesh_rx_packet_valid_message() {
        reset_for_test();

        let packet = create_test_packet(
            [1, 2, 3],
            0x1234,
            0x5678,
            0x01,
            0xABCD,
            [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0, 0, 0, 0, 0],
        );

        start_bridge();

        on_mesh_rx_packet(&packet, 0);

        assert_eq!(UART_TX_CHANNEL.len(), 1);

        // Get the sent message
        let sent_msg = UART_TX_CHANNEL.try_receive().unwrap();

        // Verify message format
        assert_eq!(sent_msg.data[2], u8::from(UartMsg::MeshMessage));

        // Verify the AppCmdValue data was copied correctly
        // sno (3 bytes)
        assert_eq!(sent_msg.data[3], 1);
        assert_eq!(sent_msg.data[4], 2);
        assert_eq!(sent_msg.data[5], 3);

        // src (2 bytes) - little endian
        assert_eq!(sent_msg.data[6], 0x34);
        assert_eq!(sent_msg.data[7], 0x12);

        // dst (2 bytes) - little endian
        assert_eq!(sent_msg.data[8], 0x78);
        assert_eq!(sent_msg.data[9], 0x56);

        // op (1 byte)
        assert_eq!(sent_msg.data[10], 0x01);

        // vendor_id (2 bytes) - little endian
        assert_eq!(sent_msg.data[11], 0xCD);
        assert_eq!(sent_msg.data[12], 0xAB);

        // par (10 bytes)
        assert_eq!(sent_msg.data[13], 0xAA);
        assert_eq!(sent_msg.data[14], 0xBB);
        assert_eq!(sent_msg.data[15], 0xCC);
        assert_eq!(sent_msg.data[16], 0xDD);
        assert_eq!(sent_msg.data[17], 0xEE);
        assert_eq!(sent_msg.data[18], 0);
        assert_eq!(sent_msg.data[19], 0);
        assert_eq!(sent_msg.data[20], 0);
        assert_eq!(sent_msg.data[21], 0);
        assert_eq!(sent_msg.data[22], 0);
    }

    /// The dedupe key is the `AppCmdValue` `dst`..`par` span (offsets 5..20 of
    /// the wire struct), so a command recorded from the host suppresses exactly
    /// that echo and nothing else.
    #[test]
    fn test_on_mesh_rx_packet_dedupes_on_the_dst_to_par_span() {
        reset_for_test();
        start_bridge();

        let packet = create_test_packet(
            [9, 8, 7],
            0x0102,
            0x0304,
            0x05,
            0x0607,
            [0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19],
        );
        let value = packet.ll_app().value;
        let expected = [
            value.dst as u8,
            (value.dst >> 8) as u8,
            value.op,
            value.vendor_id as u8,
            (value.vendor_id >> 8) as u8,
            0x10,
            0x11,
            0x12,
            0x13,
            0x14,
            0x15,
            0x16,
            0x17,
            0x18,
            0x19,
        ];
        UART_SENT_DEDUP.lock().push_back(expected).unwrap();

        on_mesh_rx_packet(&packet, 0);

        assert_eq!(
            UART_TX_CHANNEL.len(),
            0,
            "the recorded dst..par span must suppress the echo"
        );
    }

    /// Tests the on_mesh_rx_packet function with different message types.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state
    /// 2. Create test packets with different op codes and parameters
    /// 3. Start the bridge
    /// 4. Call on_mesh_rx_packet with each test packet
    /// 5. Verify messages were sent to UART with correct data
    #[test]
    fn test_on_mesh_rx_packet_different_message_types() {
        reset_for_test();

        start_bridge();

        // Test case 1: Light control message
        let packet1 = create_test_packet(
            [10, 20, 30],
            0x1111,
            0x2222,
            0x02,
            0x1111,
            [0x01, 0x02, 0x03, 0x04, 0x05, 0, 0, 0, 0, 0],
        );

        on_mesh_rx_packet(&packet1, 0);

        // Test case 2: Status request message
        let packet2 = create_test_packet(
            [40, 50, 60],
            0x3333,
            0x4444,
            0x03,
            0x2222,
            [0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0, 0, 0, 0, 0],
        );

        on_mesh_rx_packet(&packet2, 0);

        assert_eq!(UART_TX_CHANNEL.len(), 2);

        // Check first message by popping and checking
        let msg1 = UART_TX_CHANNEL.try_receive().unwrap();
        assert_eq!(msg1.data[2], u8::from(UartMsg::MeshMessage));
        assert_eq!(msg1.data[10], 0x02); // op code

        // Check second message
        let msg2 = UART_TX_CHANNEL.try_receive().unwrap();
        assert_eq!(msg2.data[2], u8::from(UartMsg::MeshMessage));
        assert_eq!(msg2.data[10], 0x03); // op code
    }

    /// The idle `uart_send_async` path arms DMA1 exactly once and returns.
    #[test]
    fn test_uart_send_async_starts_when_idle() {
        reset_for_test();
        reset_fakes();

        let msg = empty_frame();

        block_on(uart_send_async(&msg));

        assert_eq!(dma1_arms(), 1, "DMA1 must be triggered once");
        assert!(UART_HW.lock().is_tx_busy());
    }

    /// A TX that never completes must not livelock the sender: after the
    /// deadline `uart_send_async` force-starts DMA1 and returns.
    ///
    /// The host `wait_tx_done` takes the timeout branch immediately (there is
    /// no time driver), so this test would hang forever if the loop could spin.
    #[test]
    fn test_uart_send_async_timeout_force_starts_dma1() {
        reset_for_test();
        reset_fakes();

        // Pretend the previous transfer is still in flight and its TX-complete
        // interrupt never arrives.
        UART_HW.lock().start_tx_unchecked(&empty_frame());
        assert!(UART_HW.lock().is_tx_busy());

        let msg = empty_frame();

        block_on(uart_send_async(&msg));

        // One arm for the stuck transfer, one forced arm by the timeout branch.
        assert_eq!(
            dma1_arms(),
            2,
            "DMA1 must be triggered once per retransmission"
        );
        assert!(UART_HW.lock().is_tx_busy());
    }

    /// A TX-complete interrupt clears the busy flag and wakes an async sender.
    #[test]
    fn test_take_rx_and_ack_tx_irq_tx_complete_clears_busy_and_signals_tx_done() {
        reset_for_test();
        reset_fakes();
        // `take_rx_and_ack_tx_irq` reads the DMA/UART flag register through the
        // real `get_and_clear_irq_source`, which writes the value back to clear
        // it.
        FakeMmio::set_reg8(REG_DMA_RX_RDY0, UartIrqMask::Tx as u8);

        UART_HW.lock().start_tx_unchecked(&empty_frame());
        assert!(UART_HW.lock().is_tx_busy());

        take_rx_and_ack_tx_irq();

        assert!(!UART_HW.lock().is_tx_busy());
        assert!(UART_TX_DONE.try_take().is_some());
        assert_eq!(
            FakeMmio::writes()
                .into_iter()
                .filter(|w| w.addr == REG_DMA_RX_RDY0 | tlsr8266_pac::REG_BASE_ADDR
                    && w.value == u32::from(UartIrqMask::Tx as u8))
                .count(),
            1,
            "the TX flag must be cleared exactly once"
        );
    }

    /// An RX-complete interrupt queues the **raw** frame: no CRC check, no
    /// bridge start-up and no command dispatch happen on the interrupt stack.
    ///
    /// # Algorithm
    ///
    /// 1. Reset; put a frame with a deliberately invalid CRC in the DMA buffer
    /// 2. Raise the RX flag and call `take_rx_and_ack_tx_irq`
    /// 3. The frame is in `UART_RX_CHANNEL` byte for byte, the bridge is still
    ///    unstarted and no command reached the handler
    #[test]
    fn test_take_rx_and_ack_tx_irq_pushes_raw_frame_without_crc_or_dispatch() {
        reset_for_test();
        reset_fakes();
        reset_recorder();
        set_command_handler(recording_handler);
        let mut frame = frame_with_valid_crc();
        frame.data[42] ^= 0xff;
        UART_HW.lock().rx_data_buf = frame;

        // `take_rx_and_ack_tx_irq` reads the DMA/UART flag register through the
        // real `get_and_clear_irq_source`, which writes the value back to clear
        // it.
        FakeMmio::set_reg8(REG_DMA_RX_RDY0, UartIrqMask::Rx as u8);

        take_rx_and_ack_tx_irq();

        let received = UART_RX_CHANNEL.try_receive().unwrap();
        assert_eq!(received.data, frame.data);
        assert!(!started());
        assert!(!UART_STARTED_SIGNAL.signaled());
        assert_eq!(
            FakeMmio::writes()
                .into_iter()
                .filter(|w| w.addr == REG_DMA_RX_RDY0 | tlsr8266_pac::REG_BASE_ADDR
                    && w.value == u32::from(UartIrqMask::Rx as u8))
                .count(),
            1,
            "the RX flag must be cleared exactly once"
        );
        assert!(dispatched().is_empty());
    }

    /// A full RX channel costs a counter bump in the ISR, not a log line.
    ///
    /// # Algorithm
    ///
    /// 1. Reset; fill `UART_RX_CHANNEL` to its 12-slot capacity
    /// 2. Raise the RX flag with a frame in the DMA buffer and call the ISR body
    /// 3. `UART_RX_DROPPED` is 1, the channel still holds 12 frames, and
    ///    `UART_TX_CHANNEL` is untouched: the drop line belongs to the receive
    ///    task, so no `uprintln!` is reachable from the IRQ
    #[test]
    fn test_take_rx_and_ack_tx_irq_counts_drop_when_channel_full() {
        reset_for_test();
        reset_fakes();
        let frame = frame_with_valid_crc();
        for _ in 0..12 {
            UART_RX_CHANNEL.try_send(frame).unwrap();
        }
        UART_HW.lock().rx_data_buf = frame;

        FakeMmio::set_reg8(REG_DMA_RX_RDY0, UartIrqMask::Rx as u8);

        take_rx_and_ack_tx_irq();

        assert_eq!(UART_RX_DROPPED.load(Ordering::Relaxed), 1);
        assert_eq!(UART_RX_CHANNEL.len(), 12);
        assert_eq!(
            UART_TX_CHANNEL.len(),
            0,
            "the ISR must not log; the drop line belongs to the receive task"
        );
    }

    /// The panic sink arms DMA1 synchronously and does not touch the queue.
    #[test]
    fn test_send_sync_panic_starts_dma1() {
        reset_for_test();
        reset_fakes();

        let msg = empty_frame();

        send_sync_panic(&msg);

        assert_eq!(dma1_arms(), 1, "DMA1 must be triggered once");
        assert!(UART_HW.lock().is_tx_busy());
        assert_eq!(UART_TX_CHANNEL.len(), 0);
    }
}
