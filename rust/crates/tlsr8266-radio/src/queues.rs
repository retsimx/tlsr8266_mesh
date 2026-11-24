//! Module-level bounded queues for the three ISR-reachable handoffs.
//!
//! # Why these live at module scope
//!
//! Mesh RX, mesh TX and UART TX are produced from interrupt-adjacent code (RF
//! IRQ, UART IRQ) and consumed by the async tasks. `sdk/stack_analysis.py`
//! follows **direct** call edges from `irq_handler` and scores an unknown callee
//! as 0 bytes, so routing these handoffs through a function pointer would
//! silently understate the interrupt stack depth. Direct calls to these statics
//! keep the edge in the traced call graph.
//!
//! RAM is unchanged by the move: the mesh and UART managers previously owned
//! private copies of the very same buffers, and those fields are gone.
//!
//! # Layer contract
//!
//! This module may only depend on the SDK itself, `core`, `critical_section`
//! and `embassy_sync`. It must never reach up into the
//! application layer — that is the whole point of the module.
//!
//! # Full-queue policy
//!
//! Every producer is a `try_` push: the SDK cannot block an interrupt on a full
//! queue, so a full queue drops the newest item and the helper returns `false`
//! so the caller can notice. The SDK has no logging sink of its own (the
//! application owns the UART log path and is not reachable from here), so
//! instead each rejected enqueue bumps a per-queue `AtomicU32`. The number of
//! drops is *observable* through [`dropped_mesh_rx`], [`dropped_mesh_tx`] and
//! [`dropped_uart_tx`]; the application tasks poll those counters and log the
//! delta, keeping the log sink on the app side of the layering boundary.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use heapless::Vec;

use mesh_protocol::packet_types::Packet;
use mesh_protocol::uart::UartData;
use tlsr8266_hal::common::sync::CriticalSectionMutex;

/// Raw mutex backing the channels and the wake signal: masks interrupts for the
/// (short, allocation-free) critical sections inside `embassy-sync`.
type CsRawMutex = CriticalSectionRawMutex;

/// Number of mesh packets the RX channel can hold.
pub const MESH_RX_QUEUE_CAPACITY: usize = 8;
/// Number of mesh packets the TX queue can hold.
pub const MESH_TX_QUEUE_CAPACITY: usize = 8;
/// Number of UART frames the TX channel can hold.
pub const UART_TX_QUEUE_CAPACITY: usize = 12;

/// One queued mesh transmission.
///
/// `delay` is an absolute deadline in `clock_time64()` ticks, not a relative
/// offset: the TX task samples the clock once per scan and compares against it.
#[derive(Copy, Clone)]
pub struct SendPkt {
    pub delay: u64,
    pub pkt: Packet,
    pub send_count: u8,
}

/// Mesh packets received from the RF IRQ, waiting for `rf_link_rc_data`.
pub static MESH_RX_CHANNEL: Channel<CsRawMutex, Packet, MESH_RX_QUEUE_CAPACITY> = Channel::new();

/// Mesh packets waiting to be transmitted, scanned for the earliest due packet.
pub static MESH_TX_QUEUE: CriticalSectionMutex<Vec<SendPkt, MESH_TX_QUEUE_CAPACITY>> =
    CriticalSectionMutex::new(Vec::new());

/// Latched wake-up for the mesh TX task.
///
/// Signalled by [`enqueue_mesh_tx`] (and by the retransmit re-queue) so the TX
/// task rescans immediately instead of waiting for its deadline.
pub static MESH_TX_WAKE: Signal<CsRawMutex, ()> = Signal::new();

/// UART frames waiting for the UART sender task.
pub static UART_TX_CHANNEL: Channel<CsRawMutex, UartData, UART_TX_QUEUE_CAPACITY> = Channel::new();

/// Whether UART status reporting (host attached) is enabled.
pub static UART_STATUS_REPORTING_ENABLED: AtomicBool = AtomicBool::new(false);

/// Mesh RX packets dropped because [`MESH_RX_CHANNEL`] was full.
static DROPPED_MESH_RX: AtomicU32 = AtomicU32::new(0);

/// Mesh TX packets dropped because [`MESH_TX_QUEUE`] was full.
static DROPPED_MESH_TX: AtomicU32 = AtomicU32::new(0);

/// UART frames dropped because [`UART_TX_CHANNEL`] was full.
static DROPPED_UART_TX: AtomicU32 = AtomicU32::new(0);

/// Bump one drop counter by one.
///
/// `fetch_add` requires 32-bit atomic read-modify-write, which `thumbv6m`
/// (Cortex-M0) does not provide; `tlsr8266-hal`'s `mcu/clock.rs` uses the same
/// load-modify-store form for that reason. Producers run in both task and
/// interrupt context, so the update is masked. `critical_section` nesting is
/// counted, so a producer that is already masked pays two register writes.
///
/// `uart`'s `UART_RX_DROPPED` counter cannot reach this helper (the SDK is
/// above the transport, not below it), so it keeps its own copy of the same
/// masked update.
pub(crate) fn count_drop(counter: &AtomicU32) {
    critical_section::with(|_| {
        counter.store(
            counter.load(Ordering::Relaxed).wrapping_add(1),
            Ordering::Relaxed,
        );
    });
}

/// Queue a mesh packet for the RX task.
///
/// Returns `false` when the channel is full and the packet was dropped.
pub fn enqueue_mesh_rx(pkt: &Packet) -> bool {
    if MESH_RX_CHANNEL.try_send(*pkt).is_ok() {
        true
    } else {
        count_drop(&DROPPED_MESH_RX);
        false
    }
}

/// Queue a mesh packet for transmission at `delay` (an absolute `clock_time64()`
/// tick) with `send_count` retransmissions remaining.
///
/// Returns `false` when the queue is full and the packet was dropped. On success
/// the TX task is woken.
///
/// The wake is signalled after the queue guard is released so no interrupt-mask
/// is held while touching the signal.
pub fn enqueue_mesh_tx(pkt: &Packet, delay: u64, send_count: u8) -> bool {
    let pushed = {
        let mut queue = MESH_TX_QUEUE.lock();
        queue
            .push(SendPkt {
                delay,
                pkt: *pkt,
                send_count,
            })
            .is_ok()
    };

    if pushed {
        MESH_TX_WAKE.signal(());
    } else {
        count_drop(&DROPPED_MESH_TX);
    }

    pushed
}

/// Queue a UART frame for the sender task.
///
/// Returns `false` when the channel is full and the frame was dropped.
pub fn enqueue_uart_tx(msg: &UartData) -> bool {
    if UART_TX_CHANNEL.try_send(*msg).is_ok() {
        true
    } else {
        count_drop(&DROPPED_UART_TX);
        false
    }
}

/// How many mesh RX packets have been dropped because the channel was full.
pub fn dropped_mesh_rx() -> u32 {
    DROPPED_MESH_RX.load(Ordering::Relaxed)
}

/// How many mesh TX packets have been dropped because the queue was full.
pub fn dropped_mesh_tx() -> u32 {
    DROPPED_MESH_TX.load(Ordering::Relaxed)
}

/// How many UART frames have been dropped because the channel was full.
pub fn dropped_uart_tx() -> u32 {
    DROPPED_UART_TX.load(Ordering::Relaxed)
}

/// Whether UART status reporting is currently enabled.
pub fn uart_status_reporting_enabled() -> bool {
    UART_STATUS_REPORTING_ENABLED.load(Ordering::Relaxed)
}

/// Enable or disable UART status reporting.
pub fn set_uart_status_reporting_enabled(enabled: bool) {
    UART_STATUS_REPORTING_ENABLED.store(enabled, Ordering::Relaxed);
}

/// Clear every queue, flag and drop counter.
///
/// Host tests run in one process, so global queues would otherwise leak state
/// from one test into the next. Every manager `default()` constructor calls
/// this on host builds; there is no production cost.
///
/// Gated on `target_os`, not `test`: the firmware crate compiles this crate as a
/// dependency and its fixture needs the same reset.
#[cfg(not(target_os = "none"))]
pub fn reset_for_test() {
    while MESH_RX_CHANNEL.try_receive().is_ok() {}
    while UART_TX_CHANNEL.try_receive().is_ok() {}
    MESH_TX_QUEUE.lock().clear();
    MESH_TX_WAKE.reset();
    UART_STATUS_REPORTING_ENABLED.store(false, Ordering::Relaxed);
    DROPPED_MESH_RX.store(0, Ordering::Relaxed);
    DROPPED_MESH_TX.store(0, Ordering::Relaxed);
    DROPPED_UART_TX.store(0, Ordering::Relaxed);
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use mesh_protocol::uart::UART_DATA_LEN;

    fn test_packet(src: u16, dst: u16, sno: u8) -> Packet {
        use mesh_protocol::packet_types::{PacketAttCmd, PacketAttValue, PacketL2capHead};

        Packet {
            att_cmd: PacketAttCmd {
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
                value: PacketAttValue {
                    sno: [sno, 0, 0],
                    src: src.to_le_bytes(),
                    dst: dst.to_le_bytes(),
                    val: [0; 23],
                },
            },
        }
    }

    fn test_uart_msg(tag: u8) -> UartData {
        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };
        msg.data[2] = tag;
        msg
    }

    #[test]
    fn test_enqueue_mesh_rx_round_trip() {
        reset_for_test();

        let pkt = test_packet(0x10, 0x20, 7);
        assert!(enqueue_mesh_rx(&pkt));

        let received = MESH_RX_CHANNEL.try_receive().unwrap();
        // `AppCmdValue` is packed, so read the fields into locals before
        // comparing (assert_eq! would otherwise take a misaligned reference).
        let sno = received.ll_app().value.sno;
        let src = received.ll_app().value.src;
        let dst = received.ll_app().value.dst;
        assert_eq!(sno, [7, 0, 0]);
        assert_eq!(src, 0x10);
        assert_eq!(dst, 0x20);
    }

    #[test]
    fn test_enqueue_mesh_rx_full_drops_and_returns_false() {
        reset_for_test();

        let pkt = test_packet(0x10, 0x20, 1);
        for _ in 0..MESH_RX_QUEUE_CAPACITY {
            assert!(enqueue_mesh_rx(&pkt));
        }
        assert!(!enqueue_mesh_rx(&pkt));
        assert_eq!(MESH_RX_CHANNEL.len(), MESH_RX_QUEUE_CAPACITY);
    }

    #[test]
    fn test_enqueue_mesh_tx_pushes_and_signals() {
        reset_for_test();

        let pkt = test_packet(0x10, 0x20, 3);
        assert!(enqueue_mesh_tx(&pkt, 1234, 2));
        assert!(MESH_TX_WAKE.signaled());

        let queued = MESH_TX_QUEUE.lock();
        assert_eq!(queued.len(), 1);
        assert_eq!(queued[0].delay, 1234);
        assert_eq!(queued[0].send_count, 2);
        let sno = queued[0].pkt.ll_app().value.sno;
        assert_eq!(sno, [3, 0, 0]);
    }

    #[test]
    fn test_enqueue_mesh_tx_full_drops_and_returns_false() {
        reset_for_test();

        let pkt = test_packet(0x10, 0x20, 1);
        for _ in 0..MESH_TX_QUEUE_CAPACITY {
            assert!(enqueue_mesh_tx(&pkt, 0, 0));
        }
        assert!(!enqueue_mesh_tx(&pkt, 0, 0));
        assert_eq!(MESH_TX_QUEUE.lock().len(), MESH_TX_QUEUE_CAPACITY);
    }

    #[test]
    fn test_enqueue_uart_tx_round_trip_and_full() {
        reset_for_test();

        let msg = test_uart_msg(0x07);
        for _ in 0..UART_TX_QUEUE_CAPACITY {
            assert!(enqueue_uart_tx(&msg));
        }
        assert!(!enqueue_uart_tx(&msg));

        let received = UART_TX_CHANNEL.try_receive().unwrap();
        assert_eq!(received.data[2], 0x07);
    }

    #[test]
    fn test_uart_status_reporting_flag() {
        reset_for_test();
        assert!(!uart_status_reporting_enabled());

        set_uart_status_reporting_enabled(true);
        assert!(uart_status_reporting_enabled());

        set_uart_status_reporting_enabled(false);
        assert!(!uart_status_reporting_enabled());
    }

    #[test]
    fn test_reset_for_test_clears_every_queue_and_flag() {
        let pkt = test_packet(0x10, 0x20, 1);
        let _ = enqueue_mesh_rx(&pkt);
        let _ = enqueue_mesh_tx(&pkt, 0, 0);
        let _ = enqueue_uart_tx(&test_uart_msg(0x03));
        set_uart_status_reporting_enabled(true);

        reset_for_test();

        assert_eq!(MESH_RX_CHANNEL.len(), 0);
        assert_eq!(MESH_TX_QUEUE.lock().len(), 0);
        assert_eq!(UART_TX_CHANNEL.len(), 0);
        assert!(!MESH_TX_WAKE.signaled());
        assert!(!uart_status_reporting_enabled());
    }

    #[test]
    fn test_drop_counters_count_full_queue_rejections() {
        reset_for_test();

        let pkt = test_packet(0x10, 0x20, 1);
        for _ in 0..MESH_RX_QUEUE_CAPACITY {
            assert!(enqueue_mesh_rx(&pkt));
        }
        for _ in 0..MESH_TX_QUEUE_CAPACITY {
            assert!(enqueue_mesh_tx(&pkt, 0, 0));
        }
        let msg = test_uart_msg(0x05);
        for _ in 0..UART_TX_QUEUE_CAPACITY {
            assert!(enqueue_uart_tx(&msg));
        }

        for _ in 0..5 {
            assert!(!enqueue_mesh_rx(&pkt));
            assert!(!enqueue_mesh_tx(&pkt, 0, 0));
            assert!(!enqueue_uart_tx(&msg));
        }

        assert_eq!(dropped_mesh_rx(), 5);
        assert_eq!(dropped_mesh_tx(), 5);
        assert_eq!(dropped_uart_tx(), 5);
    }

    #[test]
    fn test_reset_for_test_clears_drop_counters() {
        reset_for_test();

        let pkt = test_packet(0x10, 0x20, 1);
        for _ in 0..MESH_RX_QUEUE_CAPACITY {
            assert!(enqueue_mesh_rx(&pkt));
        }
        assert!(!enqueue_mesh_rx(&pkt));
        assert_eq!(dropped_mesh_rx(), 1);

        reset_for_test();

        assert_eq!(dropped_mesh_rx(), 0);
        assert_eq!(dropped_mesh_tx(), 0);
        assert_eq!(dropped_uart_tx(), 0);
    }
}
