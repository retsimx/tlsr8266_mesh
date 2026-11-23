use core::convert::{TryFrom, TryInto};
use core::mem::size_of;
use core::ops::Range;
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

use crate::mesh;
use crate::sdk::ble_app::light_ll::mesh_management::mesh_report_status_enable;
use crate::sdk::ble_app::queues::{self, UART_TX_CHANNEL, UART_TX_QUEUE_CAPACITY};
use crate::uprintln;
use heapless::Vec;
use mesh_protocol::packet_types::{AppCmdValue, Packet};
#[cfg(test)]
use mesh_protocol::uart::UART_CRC_OFFSET;
use mesh_protocol::uart::{compute_crc, verify_crc, UartData, UartMsg, UART_DATA_LEN};
use tlsr8266_hal::common::sync::CriticalSectionMutex;
use tlsr8266_hal::common::sync::SimplifyLS;
use tlsr8266_hal::drivers::uart::{UartDriver, UartIrqMask};

pub(crate) const MSG_TYPE_INDEX: usize = 2;
const CTRL_DESTINATION_RANGE: Range<usize> = 3..5;
const CTRL_PAYLOAD_RANGE: Range<usize> = 5..18;
const CTRL_FINGERPRINT_RANGE: Range<usize> = 3..18;
const CTRL_RETRANSMIT_INDEX: usize = 18;
const CTRL_ACK_FLAG_INDEX: usize = 19;
const OUTGOING_FINGERPRINT_RANGE: Range<usize> = 7..22;
/// Byte span the UART CRC-16 covers. Production delegates the check to
/// `mesh_protocol::uart::verify_crc`/`compute_crc`; the frame-building tests
/// below still need the span itself, and deriving it from the crate's trailer
/// offset keeps the two from drifting apart.
#[cfg(test)]
pub(crate) const CRC_RANGE: Range<usize> = 0..UART_CRC_OFFSET;

pub(crate) const SNO_RANGE: Range<usize> = 2..5;
const CTRL_PAYLOAD_LEN: usize = CTRL_PAYLOAD_RANGE.end - CTRL_PAYLOAD_RANGE.start;
const CTRL_FINGERPRINT_LEN: usize = CTRL_FINGERPRINT_RANGE.end - CTRL_FINGERPRINT_RANGE.start;

type Fingerprint = [u8; CTRL_FINGERPRINT_LEN];

/// Raw mutex backing the UART channels/signals: masks interrupts for the
/// (short, allocation-free) critical sections inside `embassy-sync`.
type CsRawMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use crate::sdk::ble_app::rf_drv_8266::DEVICE_ADDRESS;

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

// AppCmdValueT
#[cfg_attr(test, mry::mry)]
pub fn light_mesh_rx_cb(data: &Packet) {
    // Don't report messages that we sent
    if !started()
        || (data.ll_app().value.src == DEVICE_ADDRESS.get()
            && data.ll_app().value.dst != DEVICE_ADDRESS.get())
    {
        return;
    }

    let data = addr_of!(data.ll_app().value) as *const u8;
    // Compute the CRC of important bits. This is from start of AppCmdValueT.dst (5) to end of AppCmdValueT.par (20)
    // SAFETY: (4) audited packed-wire view. `data` points at the live packet's
    // `AppCmdValue` field (21 bytes), and the 15-byte view is the `dst`..`par`
    // span of that wire layout (offsets 5..20), so it stays inside the
    // caller-owned packet.
    let msg = unsafe { slice::from_raw_parts((data as u32 + 5) as *const u8, 20 - 5) };

    // Check if this message is one we sent
    if UART_SENT_DEDUP.lock().iter().any(|c| c == msg) {
        // We did, nothing more to do here
        return;
    }

    let mut msg: UartData = UartData {
        len: UART_DATA_LEN as u32,
        data: [0; UART_DATA_LEN],
    };

    msg.data[2] = u8::from(UartMsg::MeshMessage);
    for i in 0..size_of::<AppCmdValue>() {
        // SAFETY: (4) audited packed-wire copy. `i < size_of::<AppCmdValue>()` (21) and
        // the destination offset `3 + i` stays below `UART_DATA_LEN` (44), so both the
        // `msg.data` store and the `data` load stay in bounds of their buffers.
        unsafe { *msg.data.as_mut_ptr().offset(3 + i as isize) = *data.add(i) };
    }

    let _ = send_message(&msg);
}

#[coverage(off)]
#[embassy_executor::task]
pub async fn uart_sender() {
    // Stay parked until a host actually speaks on UART, so boot banners
    // queued into the UART TX channel cost nothing on a headless light.
    UART_STARTED_SIGNAL.wait().await;

    // Task locals: exactly one sender task ever touches them.
    let mut ack_counter: u8 = 0;
    let mut dropped_uart_tx_seen: u32 = 0;

    loop {
        report_queue_drops(&mut dropped_uart_tx_seen);

        let mut msg = wait_for_outgoing_message().await;
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
            ack_msg(&msg, &Vec::from_slice(&sno).unwrap()).await;
        }
    }
}

/// Whether the UART bridge has been started by an incoming frame.
pub fn started() -> bool {
    UART_SENDER_STARTED.load(Ordering::Relaxed)
}

pub fn enable_uart_status_reporting() {
    queues::set_uart_status_reporting_enabled(true);
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
/// Returns `Err(())` when the queue is full (the frame was dropped and
/// counted).
#[cfg_attr(test, mry::mry)]
pub fn send_message(msg: &UartData) -> Result<(), ()> {
    if queues::enqueue_uart_tx(msg) {
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
#[cfg_attr(test, mry::mry)]
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
        queues::count_drop(&UART_RX_DROPPED);
    }
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
/// Must not touch the async queue or [`UART_TX_LOCK`]: a panic while sending
/// would re-enter `panic_fmt` through `try_send`, a call-graph cycle the static
/// stack gate rejects.
///
/// # Safety
///
/// Acquires [`UART_HW`] for the duration of the write, so it must never be
/// called from a context that already holds that guard. `lock` has no
/// production re-entrancy check and `panic = "abort"` never drops the outer
/// guard, so a re-entrant call would hand `UartDriver::send` a second
/// `&mut UartDriver` aliasing the first (see `sdk::common::sync` assumption 3).
#[cfg_attr(test, mry::mry)]
pub fn send_sync_panic(msg: &UartData) {
    UART_HW.lock().send(msg);
}

/// Waits for the next UART ack counter, giving up at `deadline`.
///
/// Free function on purpose: `Signal` appears in no `mry`-mocked signature,
/// and the target/host timeout split lives in one place.
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

async fn ack_msg(msg: &UartData, sno: &Vec<u8, 3>) {
    // If data[1] is 0xff, it means this message is an ack from the client
    if msg.data[1] == u8::from(UartMsg::Ack) {
        UART_LAST_ACK.store(msg.data[0], Ordering::Relaxed);
        UART_ACK_SIGNAL.signal(msg.data[0]);
        return;
    }

    let mut result = UartData {
        len: UART_DATA_LEN as u32,
        data: [0; UART_DATA_LEN],
    };

    // Set the counter
    result.data[0] = msg.data[0];

    // Ack the counter
    result.data[1] = 0xff;

    // Set the sno
    result.data[SNO_RANGE].copy_from_slice(sno.as_slice());

    // Set the crc16
    compute_crc(&mut result);

    // Send the ack
    uart_send_async(&result).await;
}

/// Surface UART TX queue overflow, one log line per change.
///
/// The SDK cannot log — it must not reach the application's UART sink — so it
/// only counts drops. Turning a counter delta into a line here keeps a
/// sustained overload visible from the host without a write per iteration.
///
/// Only reported once the backlog has drained past the halfway mark: a log
/// frame pushed into the very queue that overflowed can itself be dropped,
/// which would bump the counter and produce another line — a report storm that
/// keeps the queue saturated. Waiting for headroom means the line fits and the
/// diagnostic cannot amplify the condition it reports.
fn report_queue_drops(dropped_uart_tx_seen: &mut u32) {
    let total = queues::dropped_uart_tx();
    if total == *dropped_uart_tx_seen {
        return;
    }
    if UART_TX_CHANNEL.len() > UART_TX_QUEUE_CAPACITY / 2 {
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
/// `UART_TX_CHANNEL`, which is not the queue that overflowed, so it cannot
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
    // `uart_sender` and enable the status reporting the host expects.
    if !UART_SENDER_STARTED.swap(true, Ordering::Relaxed) {
        UART_STARTED_SIGNAL.signal(());

        // Enable status reporting before calling mesh_report_status_enable so
        // that the initial dump fired inside that function can actually reach
        // the UART send path.
        enable_uart_status_reporting();
        mesh_report_status_enable(true);
    }

    Some(process_received_message(msg))
}

async fn wait_for_received_message() -> UartData {
    UART_RX_CHANNEL.receive().await
}

async fn wait_for_outgoing_message() -> UartData {
    UART_TX_CHANNEL.receive().await
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

fn process_received_message(msg: &UartData) -> [u8; 3] {
    match UartMsg::try_from(msg.data[MSG_TYPE_INDEX]) {
        Ok(UartMsg::LightCtrl) => handle_light_ctrl(msg),
        Ok(UartMsg::LightStatus) => {
            handle_light_status();
            [0; 3]
        }
        _ => [0; 3],
    }
}

fn handle_light_ctrl(msg: &UartData) -> [u8; 3] {
    let destination = u16::from_le_bytes(
        msg.data[CTRL_DESTINATION_RANGE]
            .try_into()
            .expect("destination bytes"),
    );

    let mut data = [0u8; CTRL_PAYLOAD_LEN];
    data.copy_from_slice(&msg.data[CTRL_PAYLOAD_RANGE]);

    let fingerprint: Fingerprint = msg.data[CTRL_FINGERPRINT_RANGE]
        .try_into()
        .expect("fingerprint slice");
    record_sent_fingerprint(fingerprint);

    mesh::send_mesh_message(
        &Vec::from_slice(&data).unwrap(),
        destination,
        msg.data[CTRL_RETRANSMIT_INDEX],
        msg.data[CTRL_ACK_FLAG_INDEX] != 0,
    )
}

fn handle_light_status() {
    mesh_report_status_enable(true);
    enable_uart_status_reporting();
}

fn prepare_outgoing_message(msg: &mut UartData, ack_counter: &mut u8) {
    let fingerprint: Fingerprint = msg.data[OUTGOING_FINGERPRINT_RANGE]
        .try_into()
        .expect("outgoing fingerprint slice");
    record_sent_fingerprint(fingerprint);

    *ack_counter = ack_counter.wrapping_add(1);
    msg.data[0] = *ack_counter;
    compute_crc(msg);
}

async fn send_with_retry(msg: &UartData, ack_counter: u8) {
    loop {
        // Arm the wake-up before transmitting so an ack that lands between
        // the send and the wait is still observed (the value is latched).
        UART_ACK_SIGNAL.reset();
        uart_send_async(msg).await;

        if UART_LAST_ACK.load(Ordering::Relaxed) == ack_counter {
            break;
        }

        // One deadline for the whole wait: stale acks must not extend it.
        #[cfg(not(test))]
        let deadline = Instant::now() + Duration::from_millis(100);

        loop {
            #[cfg(not(test))]
            {
                match wait_ack_or_timeout(deadline).await {
                    Ok(ack) if ack == ack_counter => return,
                    Ok(_) => continue,
                    Err(()) => break,
                }
            }
            #[cfg(test)]
            {
                match wait_ack_or_timeout() {
                    Ok(ack) if ack == ack_counter => return,
                    Ok(_) => continue,
                    Err(()) => break,
                }
            }
        }
    }
}

/// Reset every piece of module state a host test can observe.
///
/// This is the replacement for the deleted singleton reset helper: host tests
/// share one process, so `App::default()` can no longer hand out a fresh
/// manager. `UART_TX_LOCK` needs no reset — it is only held inside an
/// `.await`-free critical window and is always released by dropping a guard.
#[cfg(test)]
pub fn reset_for_test() {
    *UART_HW.lock() = UartDriver::default_const();
    UART_SENT_DEDUP.lock().clear();
    UART_SENDER_STARTED.store(false, Ordering::Relaxed);
    UART_LAST_ACK.store(0, Ordering::Relaxed);
    UART_RX_DROPPED.store(0, Ordering::Relaxed);
    UART_STARTED_SIGNAL.reset();
    UART_ACK_SIGNAL.reset();
    UART_TX_DONE.reset();
    while UART_RX_CHANNEL.try_receive().is_ok() {}
    queues::reset_for_test();
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::mesh::{mock_send_mesh_message, send_mesh_message};
    use crate::sdk::ble_app::light_ll::mesh_management::mock_mesh_report_status_enable;
    use core::future::Future;
    use futures::executor::block_on;
    use futures::task::noop_waker;
    use mesh_protocol::crc::crc16;
    use mesh_protocol::packet_types::{PacketL2capHead, PacketLlApp};
    use mry::Any;
    use std::task::{Context, Poll};
    use tlsr8266_hal::mcu::clock::fake_clock;
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_pac::{mock_write_reg_dma1_addr, write_reg_dma1_addr};

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

    fn empty_frame() -> UartData {
        UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        }
    }

    #[test]
    fn test_uart_msg_try_from() {
        let cases = [
            (UartMsg::LightCtrl, 0x02),
            (UartMsg::LightStatus, 0x03),
            (UartMsg::MeshMessage, 0x04),
            (UartMsg::PanicMessage, 0x05),
            (UartMsg::PrintMessage, 0x06),
            (UartMsg::Ack, 0xff),
        ];

        for (variant, byte) in cases.iter() {
            assert_eq!(u8::from(*variant), *byte);
            assert_eq!(UartMsg::try_from(*byte).unwrap(), *variant);
        }

        assert!(UartMsg::try_from(0x11).is_err());
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

        let result = block_on(wait_for_outgoing_message());

        assert_eq!(result.len, expected_len);
        assert_eq!(result.data, expected_data);
    }

    #[test]
    fn test_wait_for_outgoing_message_waits_until_message_available() {
        reset_for_test();
        let future = wait_for_outgoing_message();
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

    #[test]
    #[mry::lock(send_mesh_message)]
    fn test_process_received_message_light_ctrl_records_message_and_calls_mesh() {
        reset_for_test();

        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };

        msg.data[MSG_TYPE_INDEX] = u8::from(UartMsg::LightCtrl);
        let destination = u16::to_le_bytes(0x1234);
        msg.data[CTRL_DESTINATION_RANGE].copy_from_slice(&destination);
        msg.data[CTRL_RETRANSMIT_INDEX] = 2; // retransmit count
        msg.data[CTRL_ACK_FLAG_INDEX] = 1; // send ack
        for (idx, byte) in msg.data[CTRL_PAYLOAD_RANGE].iter_mut().enumerate() {
            *byte = (idx as u8) + 1;
        }
        mock_send_mesh_message(Any, Any, Any, Any).returns([1, 2, 3]);

        let sno = process_received_message(&msg);

        assert_eq!(sno, [1, 2, 3]);
        mock_send_mesh_message(Any, Any, Any, Any).assert_called(1);

        let expected_fingerprint: Fingerprint = msg.data[CTRL_FINGERPRINT_RANGE]
            .try_into()
            .expect("fingerprint slice");
        let sent = UART_SENT_DEDUP.lock();
        assert_eq!(sent.len(), 1);
        assert_eq!(sent.front().unwrap(), &expected_fingerprint);
    }

    #[test]
    fn test_process_received_message_unknown_type_returns_zero() {
        reset_for_test();
        let msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0x99; UART_DATA_LEN],
        };

        let sno = process_received_message(&msg);

        assert_eq!(sno, [0; 3]);
        assert!(UART_SENT_DEDUP.lock().is_empty());
    }

    #[test]
    #[mry::lock(mesh_report_status_enable)]
    fn test_process_received_message_light_status_enables_reporting() {
        reset_for_test();

        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };
        msg.data[MSG_TYPE_INDEX] = u8::from(UartMsg::LightStatus);

        mock_mesh_report_status_enable(true).returns(());

        let sno = process_received_message(&msg);

        assert_eq!(sno, [0, 0, 0]);
        mock_mesh_report_status_enable(true).assert_called(1);
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

        let expected_fingerprint: Fingerprint = msg.data[OUTGOING_FINGERPRINT_RANGE]
            .try_into()
            .expect("fingerprint slice");
        let sent = UART_SENT_DEDUP.lock();
        assert_eq!(sent.len(), 1);
        assert_eq!(sent.front().unwrap(), &expected_fingerprint);
    }

    #[test]
    fn test_send_with_retry_exits_when_ack_already_received() {
        reset_for_test();
        reset_fakes();
        UART_LAST_ACK.store(5, Ordering::Relaxed);

        let msg = empty_frame();

        block_on(send_with_retry(&msg, 5));

        assert_eq!(dma1_arms(), 1, "DMA1 must be triggered once");
    }

    #[test]
    #[mry::lock(write_reg_dma1_addr)]
    fn test_send_with_retry_retransmits_until_ack_is_received() {
        reset_for_test();
        reset_fakes();
        UART_LAST_ACK.store(0, Ordering::Relaxed);

        // The DMA1 address write is the per-transmission seam: inject the ack
        // when the second transmission is armed, so the retry loop exits.
        let sends = AtomicU8::new(0);
        mock_write_reg_dma1_addr(Any).returns_with(move |_addr: u16| {
            if sends.fetch_add(1, Ordering::SeqCst) + 1 == 2 {
                UART_LAST_ACK.store(1, Ordering::Relaxed);
            }
        });

        let msg = empty_frame();

        // The first wait finds no ack (host mode: `try_take` -> Err) and
        // retransmits; the second send marks the ack, so the loop exits.
        block_on(send_with_retry(&msg, 1));

        assert_eq!(
            dma1_arms(),
            2,
            "DMA1 must be triggered once per retransmission"
        );
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
        let sno = Vec::from_slice(&[0u8, 0, 0]).unwrap();

        block_on(ack_msg(&ack, &sno));

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
                sent.push_back([value; CTRL_FINGERPRINT_LEN]).unwrap();
            }
        }

        let new_fingerprint = [0xAA; CTRL_FINGERPRINT_LEN];

        record_sent_fingerprint(new_fingerprint);

        let sent = UART_SENT_DEDUP.lock();
        assert_eq!(sent.len(), 12);
        assert_eq!(sent.front().unwrap(), &[1u8; CTRL_FINGERPRINT_LEN]);
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

    /// Tests the compute_crc function.
    ///
    /// This test verifies that compute_crc correctly calculates and sets the
    /// CRC in the specified message.
    ///
    /// # Algorithm
    ///
    /// 1. Create a test message with known data
    /// 2. Call compute_crc on the message
    /// 3. Manually compute the expected CRC
    /// 4. Verify the CRC in the message matches the expected value
    #[test]
    fn test_compute_crc() {
        let mut msg = empty_frame();

        for i in 0..42 {
            msg.data[i] = i as u8;
        }

        compute_crc(&mut msg);

        let expected_crc = crc16(&msg.data[0..42]);
        let expected_crc_low = (expected_crc & 0xff) as u8;
        let expected_crc_high = ((expected_crc >> 8) & 0xff) as u8;

        assert_eq!(msg.data[42], expected_crc_low);
        assert_eq!(msg.data[43], expected_crc_high);
    }

    /// Tests that an invalid-CRC frame is rejected in task context.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state (sender not started)
    /// 2. Build a frame whose CRC trailer does not match its payload
    /// 3. Call `handle_received_frame`
    /// 4. Verify it returns `None`, signals nothing and reports no status
    #[test]
    #[mry::lock(mesh_report_status_enable)]
    fn test_handle_received_frame_invalid_crc_returns_none() {
        reset_for_test();

        let mut msg = frame_with_valid_crc();
        msg.data[42] ^= 0xff;

        let sno = handle_received_frame(&msg);

        assert!(sno.is_none());
        assert!(!UART_STARTED_SIGNAL.signaled());
        assert!(!started());
        mock_mesh_report_status_enable(true).assert_called(0);
    }

    /// Tests that the first valid frame starts the bridge exactly once, and that
    /// it enables UART status reporting *before* the initial status dump.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state and stub `mesh_report_status_enable` with a body
    ///    that asserts reporting is already enabled when the dump fires
    /// 2. Handle one valid frame: `Some(sno)`, start signal latched, started
    /// 3. Handle a second valid frame: bridge start-up must not run again
    #[test]
    #[mry::lock(mesh_report_status_enable)]
    fn test_handle_received_frame_first_valid_starts_bridge_once() {
        reset_for_test();
        mock_mesh_report_status_enable(true).returns_with(|_enable| {
            assert!(
                queues::uart_status_reporting_enabled(),
                "status reporting must be enabled before the initial node-status dump"
            );
        });

        let first = frame_with_valid_crc();
        assert!(handle_received_frame(&first).is_some());

        assert!(UART_STARTED_SIGNAL.signaled());
        assert!(started());
        mock_mesh_report_status_enable(true).assert_called(1);

        let second = frame_with_valid_crc();
        assert!(handle_received_frame(&second).is_some());

        mock_mesh_report_status_enable(true).assert_called(1);
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
        UART_SENT_DEDUP
            .lock()
            .push_back([1u8; CTRL_FINGERPRINT_LEN])
            .unwrap();
        UART_SENDER_STARTED.store(true, Ordering::Relaxed);
        UART_LAST_ACK.store(7, Ordering::Relaxed);
        UART_RX_DROPPED.store(3, Ordering::Relaxed);
        UART_STARTED_SIGNAL.signal(());
        UART_ACK_SIGNAL.signal(1);
        UART_TX_DONE.signal(());
        UART_RX_CHANNEL.try_send(empty_frame()).unwrap();
        send_message(&empty_frame()).unwrap();

        reset_for_test();

        assert!(!started());
        assert!(!UART_STARTED_SIGNAL.signaled());
        assert_eq!(UART_LAST_ACK.load(Ordering::Relaxed), 0);
        assert_eq!(UART_RX_DROPPED.load(Ordering::Relaxed), 0);
        assert!(UART_ACK_SIGNAL.try_take().is_none());
        assert!(UART_TX_DONE.try_take().is_none());
        assert_eq!(UART_RX_CHANNEL.len(), 0);
        assert!(UART_SENT_DEDUP.lock().is_empty());
        assert_eq!(UART_TX_CHANNEL.len(), 0);
        assert!(!UART_HW.lock().is_tx_busy());
    }

    /// Tests the light_mesh_rx_cb function when UART is not started.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state
    /// 2. Create a test packet
    /// 3. Ensure UART is not started
    /// 4. Call light_mesh_rx_cb with the test packet
    /// 5. Verify no message was sent to UART
    #[test]
    fn test_light_mesh_rx_cb_not_started() {
        reset_for_test();

        let packet = create_test_packet([1, 2, 3], 0x1234, 0x5678, 0x01, 0xABCD, [0; 10]);

        light_mesh_rx_cb(&packet);

        assert_eq!(UART_TX_CHANNEL.len(), 0);
    }

    /// Tests the light_mesh_rx_cb function when message is from our device.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state
    /// 2. Create a test packet with our device address as source
    /// 3. Start the bridge
    /// 4. Call light_mesh_rx_cb with the test packet
    /// 5. Verify no message was sent to UART
    #[test]
    fn test_light_mesh_rx_cb_from_our_device() {
        reset_for_test();
        DEVICE_ADDRESS.set(0x1234);

        let packet = create_test_packet(
            [1, 2, 3],
            DEVICE_ADDRESS.get(),
            0x5678,
            0x01,
            0xABCD,
            [0; 10],
        );

        UART_SENDER_STARTED.store(true, Ordering::Relaxed);

        light_mesh_rx_cb(&packet);

        assert_eq!(UART_TX_CHANNEL.len(), 0);
    }

    /// Tests the light_mesh_rx_cb function when message is a duplicate.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state
    /// 2. Create a test packet
    /// 3. Start the bridge
    /// 4. Add the message to the sent deque
    /// 5. Call light_mesh_rx_cb with the test packet
    /// 6. Verify no message was sent to UART
    #[test]
    fn test_light_mesh_rx_cb_duplicate_message() {
        reset_for_test();
        DEVICE_ADDRESS.set(0);

        let packet = create_test_packet([1, 2, 3], 0x1234, 0x5678, 0x01, 0xABCD, [0; 10]);

        UART_SENDER_STARTED.store(true, Ordering::Relaxed);

        // Add the message to the sent deque (simulating we sent this message)
        // The light_mesh_rx_cb function extracts data from offset 5 (dst) for 15 bytes
        // This corresponds to: dst(2) + op(1) + vendor_id(2) + par(10) = 15 bytes
        let msg_data = [0x78, 0x56, 0x01, 0xCD, 0xAB, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        UART_SENT_DEDUP.lock().push_back(msg_data).unwrap();

        light_mesh_rx_cb(&packet);

        assert_eq!(UART_TX_CHANNEL.len(), 0);
    }

    /// Tests the light_mesh_rx_cb function with a valid new message.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state
    /// 2. Create a test packet with valid data
    /// 3. Start the bridge
    /// 4. Call light_mesh_rx_cb with the test packet
    /// 5. Verify a message was sent to UART with correct format
    #[test]
    fn test_light_mesh_rx_cb_valid_message() {
        reset_for_test();
        DEVICE_ADDRESS.set(0);

        let packet = create_test_packet(
            [1, 2, 3],
            0x1234,
            0x5678,
            0x01,
            0xABCD,
            [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0, 0, 0, 0, 0],
        );

        UART_SENDER_STARTED.store(true, Ordering::Relaxed);

        light_mesh_rx_cb(&packet);

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

    /// Tests the light_mesh_rx_cb function with different message types.
    ///
    /// # Algorithm
    ///
    /// 1. Reset the module state
    /// 2. Create test packets with different op codes and parameters
    /// 3. Start the bridge
    /// 4. Call light_mesh_rx_cb with each test packet
    /// 5. Verify messages were sent to UART with correct data
    #[test]
    fn test_light_mesh_rx_cb_different_message_types() {
        reset_for_test();
        DEVICE_ADDRESS.set(0);

        UART_SENDER_STARTED.store(true, Ordering::Relaxed);

        // Test case 1: Light control message
        let packet1 = create_test_packet(
            [10, 20, 30],
            0x1111,
            0x2222,
            0x02,
            0x1111,
            [0x01, 0x02, 0x03, 0x04, 0x05, 0, 0, 0, 0, 0],
        );

        light_mesh_rx_cb(&packet1);

        // Test case 2: Status request message
        let packet2 = create_test_packet(
            [40, 50, 60],
            0x3333,
            0x4444,
            0x03,
            0x2222,
            [0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0, 0, 0, 0, 0],
        );

        light_mesh_rx_cb(&packet2);

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
    /// bridge start-up and no status dump happen on the interrupt stack.
    ///
    /// # Algorithm
    ///
    /// 1. Reset; put a frame with a deliberately invalid CRC in the DMA buffer
    /// 2. Raise the RX flag and call `take_rx_and_ack_tx_irq`
    /// 3. The frame is in `UART_RX_CHANNEL` byte for byte, the bridge is still
    ///    unstarted and `mesh_report_status_enable` was never called
    #[test]
    #[mry::lock(mesh_report_status_enable)]
    fn test_take_rx_and_ack_tx_irq_pushes_raw_frame_without_crc_or_status_enable() {
        reset_for_test();
        reset_fakes();
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
        mock_mesh_report_status_enable(true).assert_called(0);
    }

    /// A full RX channel costs a counter bump in the ISR, not a log line.
    ///
    /// # Algorithm
    ///
    /// 1. Reset; fill `UART_RX_CHANNEL` to its 12-slot capacity
    /// 2. Raise the RX flag with a frame in the DMA buffer and call the ISR body
    /// 3. `UART_RX_DROPPED` is 1, the channel still holds 12 frames, and
    ///    `UART_TX_CHANNEL` is untouched: the drop line belongs to the receive
    ///    task, so no `uprintln!`/`enqueue_uart_tx` is reachable from the IRQ
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
