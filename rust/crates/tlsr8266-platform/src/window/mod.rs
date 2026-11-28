//! One shared 768-byte packet window for BLE status responses and OTA staging.

use core::mem::size_of;
use core::sync::atomic::{AtomicU32, AtomicU8, AtomicUsize, Ordering};

use mesh_protocol::mesh_types::MESH_NODE_MAX_NUM;
use mesh_protocol::packet_types::{Packet, PacketAttData, PacketL2capHead};
use tlsr8266_hal::common::sync::{CriticalSectionMutex, SimplifyLS};
use tlsr8266_radio::StatusEnqueueOutcome;

pub const PACKET_WINDOW_SLOTS: usize = 16;

/// Bytes of ATT payload a staged OTA frame can carry.
///
/// One `PacketAttData` is a 10-byte L2CAP head, three address bytes and this
/// 23-byte payload, so a staged slot fits the window's existing `Packet` array
/// exactly.
pub const OTA_SLOT_DATA_MAX: usize = 23;

/// One OTA frame as the engine sees it: no `Packet`, no union view.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StagedOtaSlot {
    /// L2CAP length carried by the frame head; peer-controlled.
    pub l2cap_len: u16,
    /// ATT payload, zero-padded to [`OTA_SLOT_DATA_MAX`].
    pub data: [u8; OTA_SLOT_DATA_MAX],
}

impl StagedOtaSlot {
    pub const fn empty() -> Self {
        Self {
            l2cap_len: 0,
            data: [0; OTA_SLOT_DATA_MAX],
        }
    }
}

const _: [(); 48] = [(); size_of::<Packet>()];
const _: [(); 768] = [(); size_of::<[Packet; PACKET_WINDOW_SLOTS]>()];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PacketWindowMode {
    Idle = 0,
    Status = 1,
    Ota = 2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum OtaClaimant {
    Gatt = 1,
    Mesh = 2,
    Uart = 3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SessionId(pub u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OtaClaim {
    pub claimant: OtaClaimant,
    pub session: SessionId,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OtaBeginError {
    AlreadyClaimed,
    StatusAbortRejected,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct StatusRecord {
    adr: [u8; 1],
    alarm_id: u8,
}

pub struct PacketWindow<const N: usize> {
    packets: CriticalSectionMutex<[Packet; N]>,
    transition: CriticalSectionMutex<()>,
    mode: AtomicU8,
    read: AtomicUsize,
    write: AtomicUsize,
    claim: CriticalSectionMutex<Option<OtaClaim>>,
    next_session: AtomicU32,
    status_dropped: AtomicU32,
    ota_dropped: AtomicU32,
    record_index: AtomicUsize,
    status_records: CriticalSectionMutex<[StatusRecord; MESH_NODE_MAX_NUM]>,
}

const fn zero_packet() -> Packet {
    // Initialise the whole 36-byte `PacketAttData` view, not only the 10-byte
    // head: `read_staged_ota_slot` reads the payload back out of an empty slot,
    // and an empty slot must not expose uninitialised union bytes.
    Packet {
        att_data: PacketAttData {
            head: PacketL2capHead {
                dma_len: 0,
                _type: 0,
                rf_len: 0,
                l2cap_len: 0,
                chan_id: 0,
            },
            att: 0,
            hl: 0,
            hh: 0,
            dat: [0; OTA_SLOT_DATA_MAX],
        },
    }
}

impl<const N: usize> PacketWindow<N> {
    pub const fn new() -> Self {
        Self {
            packets: CriticalSectionMutex::new([zero_packet(); N]),
            transition: CriticalSectionMutex::new(()),
            mode: AtomicU8::new(PacketWindowMode::Idle as u8),
            read: AtomicUsize::new(0),
            write: AtomicUsize::new(0),
            claim: CriticalSectionMutex::new(None),
            next_session: AtomicU32::new(0),
            status_dropped: AtomicU32::new(0),
            ota_dropped: AtomicU32::new(0),
            record_index: AtomicUsize::new(0),
            status_records: CriticalSectionMutex::new(
                [StatusRecord {
                    adr: [0],
                    alarm_id: 0,
                }; MESH_NODE_MAX_NUM],
            ),
        }
    }

    /// Clear all packet-window data while the transition guard is held.
    fn reset_storage(&self) {
        let mut packets = self.packets.lock();
        packets.fill(zero_packet());
        self.read.store(0, Ordering::Relaxed);
        self.write.store(0, Ordering::Relaxed);
        self.record_index.store(0, Ordering::Relaxed);

        let mut records = self.status_records.lock();
        records.fill(StatusRecord {
            adr: [0],
            alarm_id: 0,
        });
    }

    pub fn start_status_window(&self) -> bool {
        let _transition = self.transition.lock();

        if self.mode.load(Ordering::Relaxed) == PacketWindowMode::Ota as u8 {
            return false;
        }

        self.reset_storage();
        self.mode
            .store(PacketWindowMode::Status as u8, Ordering::Relaxed);
        true
    }

    pub fn stop_status_window(&self) {
        let _transition = self.transition.lock();

        if self.mode.load(Ordering::Relaxed) == PacketWindowMode::Ota as u8 {
            return;
        }

        self.reset_storage();
        self.mode
            .store(PacketWindowMode::Idle as u8, Ordering::Relaxed);
    }

    pub fn enqueue_status_response(
        &self,
        source_addr: u8,
        packet: &Packet,
    ) -> StatusEnqueueOutcome {
        let _transition = self.transition.lock();

        if N == 0 || self.mode.load(Ordering::Relaxed) != PacketWindowMode::Status as u8 {
            return StatusEnqueueOutcome::Rejected;
        }

        let mut packets = self.packets.lock();
        if self.mode.load(Ordering::Relaxed) != PacketWindowMode::Status as u8 {
            return StatusEnqueueOutcome::Rejected;
        }

        let record_count = self.record_index.load(Ordering::Relaxed);
        if record_count != 0 {
            let records = self.status_records.lock();
            for record in records.iter().take(record_count) {
                if record.adr[0] == source_addr {
                    return StatusEnqueueOutcome::Duplicate;
                }
            }
        }

        let write = self.write.load(Ordering::Relaxed);
        let read = self.read.load(Ordering::Relaxed);
        if (write + 1) % N == read || record_count >= MESH_NODE_MAX_NUM {
            self.status_dropped.inc();
            return StatusEnqueueOutcome::Rejected;
        }

        {
            let mut records = self.status_records.lock();
            records[record_count].adr[0] = source_addr;
        }
        packets[write] = *packet;
        self.record_index.store(record_count + 1, Ordering::Relaxed);
        self.write.store((write + 1) % N, Ordering::Relaxed);
        StatusEnqueueOutcome::Added
    }

    pub fn peek_status_response(&self) -> Option<Packet> {
        let _transition = self.transition.lock();

        if self.mode.load(Ordering::Relaxed) != PacketWindowMode::Status as u8 {
            return None;
        }

        let packets = self.packets.lock();
        let read = self.read.load(Ordering::Relaxed);
        let write = self.write.load(Ordering::Relaxed);
        if read == write {
            None
        } else {
            Some(packets[read])
        }
    }

    pub fn advance_status_response(&self) {
        let _transition = self.transition.lock();

        if self.mode.load(Ordering::Relaxed) != PacketWindowMode::Status as u8 {
            return;
        }

        let _packets = self.packets.lock();
        let read = self.read.load(Ordering::Relaxed);
        let write = self.write.load(Ordering::Relaxed);
        if read != write {
            self.read.store((read + 1) % N, Ordering::Relaxed);
        }
    }

    pub fn begin_ota(
        &self,
        claimant: OtaClaimant,
        abort_status: impl FnOnce() -> bool,
    ) -> Result<OtaClaim, OtaBeginError> {
        let _transition = self.transition.lock();

        let mode = self.mode.load(Ordering::Relaxed);
        if mode == PacketWindowMode::Ota as u8 {
            return Err(OtaBeginError::AlreadyClaimed);
        }

        if mode == PacketWindowMode::Status as u8 && !abort_status() {
            return Err(OtaBeginError::StatusAbortRejected);
        }

        let mut claim = self.claim.lock();
        if claim.is_some() || self.mode.load(Ordering::Relaxed) == PacketWindowMode::Ota as u8 {
            return Err(OtaBeginError::AlreadyClaimed);
        }

        let mut session = self.next_session.get().wrapping_add(1);
        if session == 0 {
            session = 1;
        }
        self.next_session.set(session);

        self.reset_storage();

        let ota_claim = OtaClaim {
            claimant,
            session: SessionId(session),
        };
        *claim = Some(ota_claim);
        self.mode
            .store(PacketWindowMode::Ota as u8, Ordering::Relaxed);
        Ok(ota_claim)
    }

    pub fn stage_ota_packet(&self, idx: usize, pkt: &Packet) {
        let _transition = self.transition.lock();

        if self.mode.load(Ordering::Relaxed) != PacketWindowMode::Ota as u8 {
            self.ota_dropped.inc();
            return;
        }

        if N == 0 {
            self.ota_dropped.inc();
            return;
        }

        let mut packets = self.packets.lock();
        if self.mode.load(Ordering::Relaxed) != PacketWindowMode::Ota as u8 {
            self.ota_dropped.inc();
            return;
        }
        packets[idx % N] = *pkt;
    }

    pub fn read_staged_ota_packet(&self, idx: usize) -> Packet {
        let _transition = self.transition.lock();

        if idx >= N || self.mode.load(Ordering::Relaxed) != PacketWindowMode::Ota as u8 {
            return zero_packet();
        }

        let packets = self.packets.lock();
        if self.mode.load(Ordering::Relaxed) != PacketWindowMode::Ota as u8 {
            zero_packet()
        } else {
            packets[idx]
        }
    }

    /// Stage one OTA frame at `idx` (wrapped by `N`), mode-gated like
    /// [`Self::stage_ota_packet`].
    ///
    /// The frame is encoded into the window's *existing* `[Packet; N]` array:
    /// `PacketAttData` is exactly a 10-byte L2CAP head, three address bytes and
    /// the 23-byte [`OTA_SLOT_DATA_MAX`] payload, so a slot maps one-to-one onto
    /// a packet-sized cell. No second staging array exists and no `Packet`
    /// crosses into the OTA module.
    pub fn stage_ota_slot(&self, idx: usize, slot: &StagedOtaSlot) {
        let packet = Packet {
            att_data: PacketAttData {
                head: PacketL2capHead {
                    l2cap_len: slot.l2cap_len,
                    ..PacketL2capHead::default()
                },
                // The three address bytes stay zero, as the legacy staging did;
                // the payload length is implied by `l2cap_len`, not stored.
                dat: slot.data,
                ..PacketAttData::default()
            },
        };
        self.stage_ota_packet(idx, &packet);
    }

    /// Read one staged OTA frame back out at exactly `idx`.
    ///
    /// The window stores the wire frame, so the payload comes back padded to
    /// [`OTA_SLOT_DATA_MAX`] and the length is implied by `l2cap_len`.
    pub fn read_staged_ota_slot(&self, idx: usize) -> StagedOtaSlot {
        let packet = self.read_staged_ota_packet(idx);
        if idx >= N || self.mode.load(Ordering::Relaxed) != PacketWindowMode::Ota as u8 {
            return StagedOtaSlot::empty();
        }

        let frame = packet.att_data();
        StagedOtaSlot {
            l2cap_len: frame.head.l2cap_len,
            data: frame.dat,
        }
    }

    pub fn end_ota(&self) {
        let _transition = self.transition.lock();

        {
            let mut claim = self.claim.lock();
            *claim = None;
        }
        self.reset_storage();
        self.mode
            .store(PacketWindowMode::Idle as u8, Ordering::Relaxed);
    }

    pub fn active_ota_claim(&self) -> Option<OtaClaim> {
        let _transition = self.transition.lock();
        let claim = self.claim.lock();
        *claim
    }

    pub fn status_dropped_count(&self) -> u32 {
        self.status_dropped.get()
    }

    pub fn ota_dropped_count(&self) -> u32 {
        self.ota_dropped.get()
    }
}

impl<const N: usize> Default for PacketWindow<N> {
    fn default() -> Self {
        Self::new()
    }
}

static PACKET_WINDOW: PacketWindow<PACKET_WINDOW_SLOTS> = PacketWindow::new();

pub fn start_status_window() -> bool {
    PACKET_WINDOW.start_status_window()
}

pub fn stop_status_window() {
    PACKET_WINDOW.stop_status_window()
}

pub fn enqueue_status_response(source_addr: u8, packet: &Packet) -> StatusEnqueueOutcome {
    PACKET_WINDOW.enqueue_status_response(source_addr, packet)
}

pub fn peek_status_response() -> Option<Packet> {
    PACKET_WINDOW.peek_status_response()
}

pub fn advance_status_response() {
    PACKET_WINDOW.advance_status_response()
}

pub fn begin_ota(
    claimant: OtaClaimant,
    abort_status: impl FnOnce() -> bool,
) -> Result<OtaClaim, OtaBeginError> {
    PACKET_WINDOW.begin_ota(claimant, abort_status)
}

#[inline(never)]
pub fn stage_ota_slot(idx: usize, slot: &StagedOtaSlot) {
    PACKET_WINDOW.stage_ota_slot(idx, slot)
}

#[inline(never)]
pub fn read_staged_ota_slot(idx: usize) -> StagedOtaSlot {
    PACKET_WINDOW.read_staged_ota_slot(idx)
}

pub fn end_ota() {
    PACKET_WINDOW.end_ota()
}

pub fn active_ota_claim() -> Option<OtaClaim> {
    PACKET_WINDOW.active_ota_claim()
}

pub fn active_ota_claimant() -> Option<OtaClaimant> {
    PACKET_WINDOW.active_ota_claim().map(|claim| claim.claimant)
}

pub fn status_dropped_count() -> u32 {
    PACKET_WINDOW.status_dropped.get()
}

pub fn ota_dropped_count() -> u32 {
    PACKET_WINDOW.ota_dropped.get()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Mutex, MutexGuard};

    static SERIAL: Mutex<()> = Mutex::new(());

    fn serial() -> MutexGuard<'static, ()> {
        SERIAL.lock().unwrap_or_else(|error| error.into_inner())
    }

    fn packet(mark: u32) -> Packet {
        Packet {
            head: PacketL2capHead {
                dma_len: mark,
                ..PacketL2capHead::default()
            },
        }
    }

    fn mark(pkt: Packet) -> u32 {
        pkt.head().dma_len
    }

    fn reset_static() {
        end_ota();
        stop_status_window();
        start_status_window();
        stop_status_window();
    }

    #[test]
    fn status_start_stop_and_empty_queue() {
        let _serial = serial();
        reset_static();

        assert!(start_status_window());
        assert!(peek_status_response().is_none());
        assert_eq!(
            PACKET_WINDOW.mode.load(Ordering::Relaxed),
            PacketWindowMode::Status as u8
        );

        stop_status_window();
        assert_eq!(
            PACKET_WINDOW.mode.load(Ordering::Relaxed),
            PacketWindowMode::Idle as u8
        );
        assert!(peek_status_response().is_none());
    }

    #[test]
    fn status_enqueue_duplicate_and_peek_advance() {
        let _serial = serial();
        reset_static();
        assert!(start_status_window());

        assert_eq!(
            enqueue_status_response(7, &packet(101)),
            StatusEnqueueOutcome::Added
        );
        assert_eq!(
            enqueue_status_response(7, &packet(102)),
            StatusEnqueueOutcome::Duplicate
        );

        assert_eq!(peek_status_response().map(mark), Some(101));
        advance_status_response();
        assert!(peek_status_response().is_none());
    }

    #[test]
    fn status_ring_wraps_without_losing_cursors() {
        let _serial = serial();
        reset_static();
        assert!(start_status_window());

        for i in 0..15 {
            assert_eq!(
                enqueue_status_response(i as u8 + 1, &packet(i as u32 + 100)),
                StatusEnqueueOutcome::Added
            );
        }
        for i in 0..15 {
            assert_eq!(peek_status_response().map(mark), Some(i as u32 + 100));
            advance_status_response();
        }
        assert!(peek_status_response().is_none());

        assert_eq!(
            enqueue_status_response(100, &packet(999)),
            StatusEnqueueOutcome::Added
        );
        assert_eq!(peek_status_response().map(mark), Some(999));
    }

    #[test]
    fn status_full_counts_drops_and_rejects() {
        let _serial = serial();
        reset_static();
        assert!(start_status_window());

        for i in 0..15 {
            assert_eq!(
                enqueue_status_response(i as u8 + 1, &packet(i as u32)),
                StatusEnqueueOutcome::Added
            );
        }
        assert_eq!(
            enqueue_status_response(99, &packet(999)),
            StatusEnqueueOutcome::Rejected
        );
        assert_eq!(status_dropped_count(), 1);
    }

    #[test]
    fn status_enqueue_rejects_in_idle_and_ota_modes() {
        let _serial = serial();
        reset_static();

        assert_eq!(
            enqueue_status_response(1, &packet(1)),
            StatusEnqueueOutcome::Rejected
        );

        let claim = begin_ota(OtaClaimant::Gatt, || true).unwrap();
        assert_eq!(claim.claimant, OtaClaimant::Gatt);
        assert_eq!(
            enqueue_status_response(1, &packet(1)),
            StatusEnqueueOutcome::Rejected
        );
        end_ota();
    }

    #[test]
    fn ota_stage_read_wrap_and_inactive_rejection() {
        let _serial = serial();
        reset_static();
        let before = ota_dropped_count();

        PACKET_WINDOW.stage_ota_packet(0, &packet(10));
        assert_eq!(ota_dropped_count(), before + 1);

        let claim = begin_ota(OtaClaimant::Mesh, || true).unwrap();
        assert_eq!(claim.claimant, OtaClaimant::Mesh);
        PACKET_WINDOW.stage_ota_packet(17, &packet(17));
        assert_eq!(mark(PACKET_WINDOW.read_staged_ota_packet(1)), 17);
        assert_eq!(mark(PACKET_WINDOW.read_staged_ota_packet(16)), 0);

        end_ota();
        assert_eq!(mark(PACKET_WINDOW.read_staged_ota_packet(1)), 0);
    }

    #[test]
    fn ota_begin_aborts_status_or_rejects_unabortable_status() {
        let _serial = serial();
        reset_static();
        assert!(start_status_window());
        assert_eq!(
            enqueue_status_response(3, &packet(33)),
            StatusEnqueueOutcome::Added
        );

        assert_eq!(
            begin_ota(OtaClaimant::Gatt, || false),
            Err(OtaBeginError::StatusAbortRejected)
        );
        assert_eq!(
            PACKET_WINDOW.mode.load(Ordering::Relaxed),
            PacketWindowMode::Status as u8
        );
        assert_eq!(peek_status_response().map(mark), Some(33));

        let claim = begin_ota(OtaClaimant::Gatt, || true).unwrap();
        assert_eq!(claim.claimant, OtaClaimant::Gatt);
        assert!(peek_status_response().is_none());
        end_ota();
    }

    #[test]
    fn gatt_mesh_uart_claim_order_is_single_claimant() {
        let _serial = serial();
        reset_static();

        let gatt = begin_ota(OtaClaimant::Gatt, || true).unwrap();
        assert_eq!(active_ota_claimant(), Some(OtaClaimant::Gatt));
        PACKET_WINDOW.stage_ota_packet(0, &packet(0x1234));

        assert_eq!(
            begin_ota(OtaClaimant::Mesh, || true),
            Err(OtaBeginError::AlreadyClaimed)
        );
        assert_eq!(active_ota_claim(), Some(gatt));
        assert_eq!(mark(PACKET_WINDOW.read_staged_ota_packet(0)), 0x1234);

        end_ota();
        let mesh = begin_ota(OtaClaimant::Mesh, || true).unwrap();
        assert_eq!(active_ota_claimant(), Some(OtaClaimant::Mesh));
        assert_eq!(
            begin_ota(OtaClaimant::Gatt, || true),
            Err(OtaBeginError::AlreadyClaimed)
        );
        assert_eq!(active_ota_claim(), Some(mesh));

        assert_eq!(
            begin_ota(OtaClaimant::Uart, || true),
            Err(OtaBeginError::AlreadyClaimed)
        );
        assert_eq!(active_ota_claim(), Some(mesh));
        end_ota();

        let uart = begin_ota(OtaClaimant::Uart, || true).unwrap();
        assert_eq!(active_ota_claim(), Some(uart));
        assert_eq!(
            begin_ota(OtaClaimant::Gatt, || true),
            Err(OtaBeginError::AlreadyClaimed)
        );
        end_ota();
    }

    #[test]
    fn rejection_preserves_claim_session_mode_cursor_and_bytes() {
        let _serial = serial();
        reset_static();

        let claim = begin_ota(OtaClaimant::Gatt, || true).unwrap();
        PACKET_WINDOW.stage_ota_packet(4, &packet(444));
        PACKET_WINDOW.read.store(7, Ordering::Relaxed);
        PACKET_WINDOW.write.store(9, Ordering::Relaxed);

        assert_eq!(
            begin_ota(OtaClaimant::Mesh, || true),
            Err(OtaBeginError::AlreadyClaimed)
        );
        assert_eq!(active_ota_claim(), Some(claim));
        assert_eq!(
            PACKET_WINDOW.mode.load(Ordering::Relaxed),
            PacketWindowMode::Ota as u8
        );
        assert_eq!(PACKET_WINDOW.read.load(Ordering::Relaxed), 7);
        assert_eq!(PACKET_WINDOW.write.load(Ordering::Relaxed), 9);
        assert_eq!(mark(PACKET_WINDOW.read_staged_ota_packet(4)), 444);

        end_ota();
    }

    #[test]
    fn end_ota_is_reset_funnel() {
        let _serial = serial();
        reset_static();

        let first = begin_ota(OtaClaimant::Mesh, || true).unwrap();
        PACKET_WINDOW.stage_ota_packet(2, &packet(222));
        end_ota();
        assert_eq!(active_ota_claim(), None);
        assert_eq!(
            PACKET_WINDOW.mode.load(Ordering::Relaxed),
            PacketWindowMode::Idle as u8
        );
        assert_eq!(PACKET_WINDOW.read.load(Ordering::Relaxed), 0);
        assert_eq!(PACKET_WINDOW.write.load(Ordering::Relaxed), 0);
        assert_eq!(PACKET_WINDOW.record_index.load(Ordering::Relaxed), 0);
        assert_eq!(mark(PACKET_WINDOW.read_staged_ota_packet(2)), 0);

        let second = begin_ota(OtaClaimant::Gatt, || true).unwrap();
        assert_ne!(first.session, second.session);
        assert!(matches!(second.claimant, OtaClaimant::Gatt));
        end_ota();
    }

    #[test]
    fn status_start_and_ota_begin_share_one_transition_lock() {
        let _serial = serial();
        reset_static();

        assert!(start_status_window());
        let claim = begin_ota(OtaClaimant::Gatt, || true).unwrap();
        assert_eq!(claim.claimant, OtaClaimant::Gatt);
        assert_eq!(
            PACKET_WINDOW.mode.load(Ordering::Relaxed),
            PacketWindowMode::Ota as u8
        );
        assert!(!start_status_window());
        assert_eq!(active_ota_claim(), Some(claim));

        end_ota();
        assert!(start_status_window());
        stop_status_window();
    }

    #[test]
    fn stop_status_cannot_reset_active_ota_claim() {
        let _serial = serial();
        reset_static();

        let claim = begin_ota(OtaClaimant::Mesh, || true).unwrap();
        PACKET_WINDOW.stage_ota_packet(3, &packet(303));
        stop_status_window();

        assert_eq!(active_ota_claim(), Some(claim));
        assert_eq!(
            PACKET_WINDOW.mode.load(Ordering::Relaxed),
            PacketWindowMode::Ota as u8
        );
        assert_eq!(mark(PACKET_WINDOW.read_staged_ota_packet(3)), 303);
        end_ota();
    }

    #[test]
    fn repeated_start_begin_transition_sequence_is_consistent() {
        let _serial = serial();

        for _ in 0..32 {
            reset_static();
            assert!(start_status_window());
            let claim = begin_ota(OtaClaimant::Uart, || true).unwrap();

            assert_eq!(claim.claimant, OtaClaimant::Uart);
            assert_eq!(
                PACKET_WINDOW.mode.load(Ordering::Relaxed),
                PacketWindowMode::Ota as u8
            );
            assert_eq!(active_ota_claimant(), Some(OtaClaimant::Uart));
            end_ota();
            assert_eq!(
                PACKET_WINDOW.mode.load(Ordering::Relaxed),
                PacketWindowMode::Idle as u8
            );
        }
    }

    #[test]
    fn repeated_fake_cs_stage_access_is_consistent() {
        let _serial = serial();
        reset_static();
        assert!(begin_ota(OtaClaimant::Uart, || true).is_ok());

        for idx in 0..8 {
            PACKET_WINDOW.stage_ota_packet(idx, &packet(idx as u32 + 500));
        }

        for idx in 0..8 {
            assert_eq!(
                mark(PACKET_WINDOW.read_staged_ota_packet(idx)),
                idx as u32 + 500
            );
        }
        end_ota();
    }

    #[test]
    fn packet_window_instance_and_drop_counts_work() {
        let window = PacketWindow::<16>::new();
        assert!(window.start_status_window());
        assert_eq!(
            window.enqueue_status_response(1, &packet(42)),
            StatusEnqueueOutcome::Added
        );
        assert_eq!(window.peek_status_response().map(mark), Some(42));
        window.stop_status_window();
        assert_eq!(window.status_dropped_count(), 0);
        assert_eq!(window.ota_dropped_count(), 0);
    }

    /// A staged OTA slot round-trips through the window's packet array: the
    /// frame head and all 23 payload bytes come back byte-exact, and the index
    /// wraps at the window depth.
    #[test]
    fn ota_slot_round_trips_through_the_packet_array() {
        let _serial = serial();
        reset_static();
        assert!(begin_ota(OtaClaimant::Gatt, || true).is_ok());

        let mut slot = StagedOtaSlot::empty();
        slot.l2cap_len = 23;
        for (i, byte) in slot.data.iter_mut().enumerate() {
            *byte = i as u8 + 1;
        }

        stage_ota_slot(2, &slot);
        let read = read_staged_ota_slot(2);
        assert_eq!(read.l2cap_len, 23);
        assert_eq!(read.data, slot.data);

        stage_ota_slot(18, &slot);
        assert_eq!(read_staged_ota_slot(2).data, slot.data);

        end_ota();
        assert_eq!(read_staged_ota_slot(2), StagedOtaSlot::empty());
    }

    /// Staging a slot outside an OTA claim is dropped and counted, exactly like
    /// a raw packet.
    #[test]
    fn ota_slot_stage_outside_a_claim_is_dropped() {
        let _serial = serial();
        reset_static();
        let before = ota_dropped_count();

        stage_ota_slot(0, &StagedOtaSlot::empty());
        assert_eq!(ota_dropped_count(), before + 1);
        assert_eq!(read_staged_ota_slot(0), StagedOtaSlot::empty());

        let _ = begin_ota(OtaClaimant::Mesh, || true);
        stage_ota_slot(0, &StagedOtaSlot::empty());
        assert_eq!(ota_dropped_count(), before + 1);
        end_ota();
    }
}
