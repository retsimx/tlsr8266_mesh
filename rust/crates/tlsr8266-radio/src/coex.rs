//! Pure radio coexistence contract types.
//!
//! This module provides fixed-stack, Copy value types and a static ZST trait
//! for auxiliary RF and BLE link scheduling. It does not own or create a queue.

use mesh_protocol::packet_types::Packet;

/// BLE link identity and advertisement payloads used by coexistence implementations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LinkIdentity {
    pub mac: [u8; 6],
    pub device_address: u16,
    pub vendor_id: u16,
    pub adv_payload: [u8; 31],
    pub adv_len: u8,
    pub scan_rsp_payload: [u8; 31],
    pub scan_rsp_len: u8,
}

/// Fixed auxiliary RF parameters.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AuxiliaryRfPlan {
    pub access_code: u32,
    pub listen_channels: [u8; 4],
}

/// Source or ingress route for an auxiliary RF frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameIngressTag {
    RfValidatedAux,
    GattBridge,
}

/// A Copy auxiliary RF frame carrying an explicit ingress tag.
#[derive(Clone, Copy)]
pub struct RadioFrame {
    pub tag: FrameIngressTag,
    pub rf_validated: bool,
    pub packet: Packet,
}

/// Counts frames dropped after an auxiliary submission queue was full.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QueueFull {
    pub dropped_total: u32,
}

/// Receive context captured alongside an auxiliary RF frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RxContext {
    pub rssi: u8,
    pub channel: u8,
    pub rx_tick: u32,
}

/// Action to take after an auxiliary receive or listen event.
#[derive(Clone, Copy)]
pub enum RadioAction {
    Ignore,
    Continue,
    Transmit(Packet),
    ScheduleListen { delay_us: u32 },
}

/// Outcome of adding a packet to the shared status response window.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StatusEnqueueOutcome {
    Added,
    Duplicate,
    Rejected,
}

/// Link-layer lifecycle event.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinkEvent {
    Connected,
    Disconnected,
    Reset,
    AuthWindowExpired,
}

/// Action to take after a link-layer lifecycle event.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinkAction {
    None,
    ResumeAdvertising,
    Disconnect,
}

/// Timing context for one connection slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ConnectionSlot {
    pub connected_tick: u32,
    pub next_connect_tick: u32,
    pub interval_ticks: u32,
    pub tx_ready: bool,
    /// The current system tick, sampled once by the radio.
    ///
    /// The coexistence implementation needs an elapsed-time reference for its
    /// own supervision countdown, so the radio hands the tick over instead of
    /// letting the implementation read a register.
    pub current_tick: u32,
}

/// Action to take for a connection slot.
#[derive(Clone, Copy)]
pub enum ConnectionPlan {
    Continue,
    Transmit(Packet),
    Disconnect,
    Reschedule {
        delay_us: u32,
    },
    /// The implementation serviced this slot itself.
    ///
    /// The radio must reschedule the next connection event and return without
    /// running bridge or mesh work for this slot.
    SkipToNextEvent,
}

/// Timing context for one auxiliary listen slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ListenSlot {
    pub tick: u32,
    pub cycle: u32,
}

/// Timing context for one advertisement slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdvertisementSlot {
    pub tick: u32,
    pub cycle: u32,
}

/// One deferred ATT packet selected for transmission.
#[derive(Clone, Copy)]
pub struct AttTxPacket {
    pub packet: Packet,
}

/// Static radio coexistence contract implemented by ZST radio adapters.
pub trait RadioCoexistence: 'static {
    fn link_identity() -> LinkIdentity;

    fn auxiliary_rf() -> AuxiliaryRfPlan;

    fn on_aux_receive(frame: &RadioFrame, context: RxContext) -> RadioAction;

    fn submit_aux_frame(frame: RadioFrame) -> Result<(), QueueFull>;

    fn on_link_event(event: LinkEvent) -> LinkAction;

    fn on_connection_slot(slot: ConnectionSlot) -> ConnectionPlan;

    fn on_aux_listen_slot(slot: ListenSlot) -> RadioAction;

    fn on_post_advertisement(slot: AdvertisementSlot) -> RadioAction;

    fn poll_deferred_att() -> Option<AttTxPacket>;

    /// Whether auxiliary RF reception may run right now.
    ///
    /// The policy lives in the platform; the radio only asks whether it may
    /// service an auxiliary frame at this point.
    fn allow_aux_rx() -> bool;

    /// Optional override for the BLE receive window, in microseconds.
    ///
    /// The policy lives in the platform; the radio only asks how long to keep
    /// the receiver open, and `None` means "use the normal window".
    fn receive_window_override_us() -> Option<u32>;

    /// A bulk GATT write arrived on the writable attribute.
    ///
    /// The policy lives in the platform; the radio only hands over the ATT
    /// payload.
    fn on_bulk_gatt_write(l2cap_len: u16, payload: &[u8]);

    /// A bulk GATT read was issued for the readable attribute.
    ///
    /// The policy lives in the platform; the radio only reports that the read
    /// happened.
    fn on_bulk_gatt_read();

    fn start_status_window() -> bool;

    fn stop_status_window();

    fn enqueue_status_response(source_addr: u8, packet: &Packet) -> StatusEnqueueOutcome;

    fn peek_status_response() -> Option<Packet>;

    fn advance_status_response();
}

#[cfg(test)]
mod test_window {
    use super::*;
    use crate::types::{StatusRecord, MESH_NODE_MAX_NUM};
    use core::sync::atomic::{AtomicBool, AtomicU8, AtomicUsize};
    use mesh_protocol::packet_types::{PacketAttData, PacketL2capHead};
    use tlsr8266_hal::common::sync::CriticalSectionMutex;

    static TEST_STATUS_ACTIVE: AtomicBool = AtomicBool::new(false);
    static TEST_STATUS_READ: AtomicUsize = AtomicUsize::new(0);
    static TEST_STATUS_WRITE: AtomicUsize = AtomicUsize::new(0);
    static TEST_STATUS_RECORD_INDEX: AtomicUsize = AtomicUsize::new(0);
    /// The plan `on_connection_slot` reports, encoded as 0 = Continue,
    /// 1 = `SkipToNextEvent`, 2 = `Disconnect`, 3 = `Reschedule`.
    static TEST_CONNECTION_PLAN: AtomicU8 = AtomicU8::new(0);
    /// Number of `LinkEvent::Disconnected` notifications the radio delivered.
    static TEST_LINK_DISCONNECTS: AtomicUsize = AtomicUsize::new(0);
    /// What `allow_aux_rx` reports; defaults to "allowed".
    static TEST_ALLOW_AUX_RX: AtomicBool = AtomicBool::new(true);
    /// What `receive_window_override_us` reports; `0` means "no override".
    static TEST_RECEIVE_WINDOW_OVERRIDE_US: AtomicUsize = AtomicUsize::new(0);
    static TEST_STATUS_PACKETS: CriticalSectionMutex<[Packet; 16]> = CriticalSectionMutex::new(
        [Packet {
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
                dat: [0; 23],
            },
        }; 16],
    );
    static TEST_STATUS_RECORDS: CriticalSectionMutex<[StatusRecord; MESH_NODE_MAX_NUM]> =
        CriticalSectionMutex::new(
            [StatusRecord {
                adr: [0],
                alarm_id: 0,
            }; MESH_NODE_MAX_NUM],
        );

    pub(crate) struct TestRadioCoexistence;

    impl TestRadioCoexistence {
        pub(crate) fn reset() {
            TEST_STATUS_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
            TEST_STATUS_READ.store(0, core::sync::atomic::Ordering::Relaxed);
            TEST_STATUS_WRITE.store(0, core::sync::atomic::Ordering::Relaxed);
            TEST_STATUS_RECORD_INDEX.store(0, core::sync::atomic::Ordering::Relaxed);
            TEST_CONNECTION_PLAN.store(0, core::sync::atomic::Ordering::Relaxed);
            TEST_LINK_DISCONNECTS.store(0, core::sync::atomic::Ordering::Relaxed);
            TEST_ALLOW_AUX_RX.store(true, core::sync::atomic::Ordering::Relaxed);
            TEST_RECEIVE_WINDOW_OVERRIDE_US.store(0, core::sync::atomic::Ordering::Relaxed);
            TEST_STATUS_PACKETS.lock().fill(Packet {
                att_data: PacketAttData::default(),
            });
            TEST_STATUS_RECORDS.lock().fill(StatusRecord {
                adr: [0],
                alarm_id: 0,
            });
        }

        pub(crate) fn read_pointer() -> usize {
            TEST_STATUS_READ.load(core::sync::atomic::Ordering::Relaxed)
        }

        pub(crate) fn write_pointer() -> usize {
            TEST_STATUS_WRITE.load(core::sync::atomic::Ordering::Relaxed)
        }

        pub(crate) fn record_index() -> usize {
            TEST_STATUS_RECORD_INDEX.load(core::sync::atomic::Ordering::Relaxed)
        }

        pub(crate) fn record_address(index: usize) -> u8 {
            TEST_STATUS_RECORDS.lock()[index].adr[0]
        }

        pub(crate) fn set_read_pointer(index: usize) {
            TEST_STATUS_READ.store(index, core::sync::atomic::Ordering::Relaxed);
        }

        pub(crate) fn set_write_pointer(index: usize) {
            TEST_STATUS_WRITE.store(index, core::sync::atomic::Ordering::Relaxed);
        }

        pub(crate) fn set_record_index(index: usize) {
            TEST_STATUS_RECORD_INDEX.store(index, core::sync::atomic::Ordering::Relaxed);
        }

        pub(crate) fn packet(index: usize) -> Packet {
            TEST_STATUS_PACKETS.lock()[index]
        }

        /// How many link-layer disconnects the radio reported.
        pub(crate) fn link_disconnects() -> usize {
            TEST_LINK_DISCONNECTS.load(core::sync::atomic::Ordering::Relaxed)
        }

        /// Allow (or suspend) auxiliary RF reception.
        pub(crate) fn set_allow_aux_rx_for_test(allowed: bool) {
            TEST_ALLOW_AUX_RX.store(allowed, core::sync::atomic::Ordering::Relaxed);
        }

        /// Program the receive-window override the next slot reports, in µs;
        /// `0` restores "no override".
        pub(crate) fn set_receive_window_override_for_test(us: u32) {
            TEST_RECEIVE_WINDOW_OVERRIDE_US
                .store(us as usize, core::sync::atomic::Ordering::Relaxed);
        }

        /// Program the plan the next `on_connection_slot` reports.
        pub(crate) fn set_connection_plan_for_test(plan: ConnectionPlan) {
            let encoded = match plan {
                ConnectionPlan::Continue => 0,
                ConnectionPlan::SkipToNextEvent => 1,
                ConnectionPlan::Disconnect => 2,
                ConnectionPlan::Transmit(_) | ConnectionPlan::Reschedule { .. } => 3,
            };
            TEST_CONNECTION_PLAN.store(encoded, core::sync::atomic::Ordering::Relaxed);
        }
    }

    impl RadioCoexistence for TestRadioCoexistence {
        fn link_identity() -> LinkIdentity {
            LinkIdentity {
                mac: [0; 6],
                device_address: 0,
                vendor_id: 0,
                adv_payload: [0; 31],
                adv_len: 0,
                scan_rsp_payload: [0; 31],
                scan_rsp_len: 0,
            }
        }

        fn auxiliary_rf() -> AuxiliaryRfPlan {
            AuxiliaryRfPlan {
                access_code: 0,
                listen_channels: [37, 38, 39, 40],
            }
        }

        fn on_aux_receive(_frame: &RadioFrame, _context: RxContext) -> RadioAction {
            RadioAction::Continue
        }

        fn submit_aux_frame(_frame: RadioFrame) -> Result<(), QueueFull> {
            Ok(())
        }

        fn on_link_event(event: LinkEvent) -> LinkAction {
            if event == LinkEvent::Disconnected {
                TEST_LINK_DISCONNECTS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
            LinkAction::None
        }

        fn on_connection_slot(_slot: ConnectionSlot) -> ConnectionPlan {
            match TEST_CONNECTION_PLAN.load(core::sync::atomic::Ordering::Relaxed) {
                1 => ConnectionPlan::SkipToNextEvent,
                2 => ConnectionPlan::Disconnect,
                _ => ConnectionPlan::Continue,
            }
        }

        fn on_aux_listen_slot(_slot: ListenSlot) -> RadioAction {
            RadioAction::Ignore
        }

        fn on_post_advertisement(_slot: AdvertisementSlot) -> RadioAction {
            RadioAction::Continue
        }

        fn poll_deferred_att() -> Option<AttTxPacket> {
            None
        }

        fn allow_aux_rx() -> bool {
            TEST_ALLOW_AUX_RX.load(core::sync::atomic::Ordering::Relaxed)
        }

        fn receive_window_override_us() -> Option<u32> {
            match TEST_RECEIVE_WINDOW_OVERRIDE_US.load(core::sync::atomic::Ordering::Relaxed) {
                0 => None,
                us => Some(us as u32),
            }
        }

        fn on_bulk_gatt_write(_l2cap_len: u16, _payload: &[u8]) {}

        fn on_bulk_gatt_read() {}

        fn start_status_window() -> bool {
            TEST_STATUS_ACTIVE.store(true, core::sync::atomic::Ordering::Relaxed);
            TEST_STATUS_READ.store(0, core::sync::atomic::Ordering::Relaxed);
            TEST_STATUS_WRITE.store(0, core::sync::atomic::Ordering::Relaxed);
            TEST_STATUS_RECORD_INDEX.store(0, core::sync::atomic::Ordering::Relaxed);
            TEST_STATUS_PACKETS.lock().fill(Packet {
                att_data: PacketAttData::default(),
            });
            TEST_STATUS_RECORDS.lock().fill(StatusRecord {
                adr: [0],
                alarm_id: 0,
            });
            true
        }

        fn stop_status_window() {
            TEST_STATUS_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
            TEST_STATUS_READ.store(0, core::sync::atomic::Ordering::Relaxed);
            TEST_STATUS_WRITE.store(0, core::sync::atomic::Ordering::Relaxed);
            TEST_STATUS_RECORD_INDEX.store(0, core::sync::atomic::Ordering::Relaxed);
            TEST_STATUS_RECORDS.lock().fill(StatusRecord {
                adr: [0],
                alarm_id: 0,
            });
        }

        fn enqueue_status_response(source_addr: u8, packet: &Packet) -> StatusEnqueueOutcome {
            if !TEST_STATUS_ACTIVE.load(core::sync::atomic::Ordering::Relaxed) {
                return StatusEnqueueOutcome::Rejected;
            }

            let record_index = TEST_STATUS_RECORD_INDEX.load(core::sync::atomic::Ordering::Relaxed);
            if record_index != 0
                && TEST_STATUS_RECORDS
                    .lock()
                    .iter()
                    .take(record_index)
                    .any(|record| record.adr[0] == source_addr)
            {
                return StatusEnqueueOutcome::Duplicate;
            }

            let read = TEST_STATUS_READ.load(core::sync::atomic::Ordering::Relaxed);
            let write = TEST_STATUS_WRITE.load(core::sync::atomic::Ordering::Relaxed);
            if (write + 1) % 16 == read || record_index >= MESH_NODE_MAX_NUM {
                return StatusEnqueueOutcome::Rejected;
            }

            TEST_STATUS_RECORDS.lock()[record_index].adr[0] = source_addr;
            TEST_STATUS_RECORD_INDEX.store(record_index + 1, core::sync::atomic::Ordering::Relaxed);
            TEST_STATUS_PACKETS.lock()[write] = *packet;
            TEST_STATUS_WRITE.store((write + 1) % 16, core::sync::atomic::Ordering::Relaxed);
            StatusEnqueueOutcome::Added
        }

        fn peek_status_response() -> Option<Packet> {
            let read = TEST_STATUS_READ.load(core::sync::atomic::Ordering::Relaxed);
            let write = TEST_STATUS_WRITE.load(core::sync::atomic::Ordering::Relaxed);
            if read == write {
                None
            } else {
                Some(TEST_STATUS_PACKETS.lock()[read])
            }
        }

        fn advance_status_response() {
            let read = TEST_STATUS_READ.load(core::sync::atomic::Ordering::Relaxed);
            let write = TEST_STATUS_WRITE.load(core::sync::atomic::Ordering::Relaxed);
            if read != write {
                TEST_STATUS_READ.store((read + 1) % 16, core::sync::atomic::Ordering::Relaxed);
            }
        }
    }
}

#[cfg(test)]
pub(crate) use test_window::TestRadioCoexistence;

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_protocol::packet_types::PacketL2capHead;

    fn assert_copy<T: Copy>() {}

    fn packet() -> Packet {
        Packet {
            head: PacketL2capHead::default(),
        }
    }

    #[test]
    fn types_construct_and_remain_copy() {
        let identity = LinkIdentity {
            mac: [1, 2, 3, 4, 5, 6],
            device_address: 0x1234,
            vendor_id: 0x5678,
            adv_payload: [7; 31],
            adv_len: 31,
            scan_rsp_payload: [8; 31],
            scan_rsp_len: 31,
        };
        let identity_copy = identity;
        assert_eq!(identity_copy.mac, identity.mac);

        let auxiliary_rf = AuxiliaryRfPlan {
            access_code: 0x1234_5678,
            listen_channels: [37, 38, 39, 40],
        };
        assert_eq!(auxiliary_rf.listen_channels, [37, 38, 39, 40]);

        let frame = RadioFrame {
            tag: FrameIngressTag::RfValidatedAux,
            rf_validated: true,
            packet: packet(),
        };
        let frame_copy = frame;
        assert!(matches!(frame_copy.tag, FrameIngressTag::RfValidatedAux));
        assert!(frame_copy.rf_validated);

        assert_copy::<LinkIdentity>();
        assert_copy::<AuxiliaryRfPlan>();
        assert_copy::<FrameIngressTag>();
        assert_copy::<RadioFrame>();
    }

    #[test]
    fn bounded_context_types_construct_and_remain_copy() {
        let queue_full = QueueFull { dropped_total: 3 };
        let rx_context = RxContext {
            rssi: 194,
            channel: 39,
            rx_tick: 1234,
        };
        let connection_slot = ConnectionSlot {
            connected_tick: 100,
            next_connect_tick: 148,
            interval_ticks: 48,
            tx_ready: true,
            current_tick: 120,
        };
        let listen_slot = ListenSlot {
            tick: 200,
            cycle: 2,
        };
        let advertisement_slot = AdvertisementSlot {
            tick: 300,
            cycle: 3,
        };

        assert_copy::<QueueFull>();
        assert_copy::<RxContext>();
        assert_copy::<ConnectionSlot>();
        assert_copy::<ListenSlot>();
        assert_copy::<AdvertisementSlot>();
        assert_eq!(queue_full.dropped_total, 3);
        assert_eq!(rx_context.channel, 39);
        assert!(connection_slot.tx_ready);
        assert_eq!(connection_slot.current_tick, 120);
        assert_eq!(listen_slot.cycle, 2);
        assert_eq!(advertisement_slot.cycle, 3);
    }

    #[test]
    fn actions_plans_and_att_remain_copy() {
        assert_copy::<RadioAction>();
        assert_copy::<StatusEnqueueOutcome>();
        assert_copy::<LinkEvent>();
        assert_copy::<LinkAction>();
        assert_copy::<ConnectionPlan>();
        assert_copy::<AttTxPacket>();

        let actions = [
            RadioAction::Ignore,
            RadioAction::Continue,
            RadioAction::Transmit(packet()),
            RadioAction::ScheduleListen { delay_us: 250 },
        ];
        assert!(matches!(actions[2], RadioAction::Transmit(_)));

        let plans = [
            ConnectionPlan::Continue,
            ConnectionPlan::Transmit(packet()),
            ConnectionPlan::Disconnect,
            ConnectionPlan::Reschedule { delay_us: 500 },
            ConnectionPlan::SkipToNextEvent,
        ];
        assert!(matches!(plans[2], ConnectionPlan::Disconnect));
        assert!(matches!(plans[4], ConnectionPlan::SkipToNextEvent));

        let att = AttTxPacket { packet: packet() };
        let att_copy = att;
        assert!(matches!(att_copy.packet.head(), _));
    }

    struct CompatRadio;

    impl RadioCoexistence for CompatRadio {
        fn link_identity() -> LinkIdentity {
            LinkIdentity {
                mac: [0; 6],
                device_address: 0,
                vendor_id: 0,
                adv_payload: [0; 31],
                adv_len: 0,
                scan_rsp_payload: [0; 31],
                scan_rsp_len: 0,
            }
        }

        fn auxiliary_rf() -> AuxiliaryRfPlan {
            AuxiliaryRfPlan {
                access_code: 0,
                listen_channels: [37, 38, 39, 0],
            }
        }

        fn on_aux_receive(_frame: &RadioFrame, _context: RxContext) -> RadioAction {
            RadioAction::Continue
        }

        fn submit_aux_frame(_frame: RadioFrame) -> Result<(), QueueFull> {
            Ok(())
        }

        fn on_link_event(event: LinkEvent) -> LinkAction {
            match event {
                LinkEvent::Disconnected => LinkAction::ResumeAdvertising,
                _ => LinkAction::None,
            }
        }

        fn on_connection_slot(_slot: ConnectionSlot) -> ConnectionPlan {
            ConnectionPlan::Continue
        }

        fn on_aux_listen_slot(_slot: ListenSlot) -> RadioAction {
            RadioAction::Ignore
        }

        fn on_post_advertisement(_slot: AdvertisementSlot) -> RadioAction {
            RadioAction::Continue
        }

        fn poll_deferred_att() -> Option<AttTxPacket> {
            None
        }

        fn allow_aux_rx() -> bool {
            true
        }

        fn receive_window_override_us() -> Option<u32> {
            None
        }

        fn on_bulk_gatt_write(_l2cap_len: u16, _payload: &[u8]) {}

        fn on_bulk_gatt_read() {}

        fn start_status_window() -> bool {
            false
        }

        fn stop_status_window() {}

        fn enqueue_status_response(_source_addr: u8, _packet: &Packet) -> StatusEnqueueOutcome {
            StatusEnqueueOutcome::Rejected
        }

        fn peek_status_response() -> Option<Packet> {
            None
        }

        fn advance_status_response() {}
    }

    #[test]
    fn zst_trait_implementation_is_compatible() {
        let identity = CompatRadio::link_identity();
        assert_eq!(identity.device_address, 0);

        let frame = RadioFrame {
            tag: FrameIngressTag::GattBridge,
            rf_validated: false,
            packet: packet(),
        };
        let context = RxContext {
            rssi: 200,
            channel: 37,
            rx_tick: 999,
        };
        assert!(matches!(
            CompatRadio::on_aux_receive(&frame, context),
            RadioAction::Continue
        ));
        assert!(CompatRadio::submit_aux_frame(frame).is_ok());
        assert_eq!(
            CompatRadio::on_link_event(LinkEvent::Disconnected),
            LinkAction::ResumeAdvertising
        );
        assert!(matches!(
            CompatRadio::on_connection_slot(ConnectionSlot {
                connected_tick: 1,
                next_connect_tick: 2,
                interval_ticks: 1,
                tx_ready: false,
                current_tick: 2,
            }),
            ConnectionPlan::Continue
        ));
        assert!(matches!(
            CompatRadio::on_aux_listen_slot(ListenSlot { tick: 1, cycle: 1 }),
            RadioAction::Ignore
        ));
        assert!(matches!(
            CompatRadio::on_post_advertisement(AdvertisementSlot { tick: 1, cycle: 1 }),
            RadioAction::Continue
        ));
        assert!(CompatRadio::poll_deferred_att().is_none());
        assert!(
            CompatRadio::allow_aux_rx(),
            "the compatibility adapter keeps auxiliary RX enabled"
        );
        assert_eq!(
            CompatRadio::receive_window_override_us(),
            None,
            "the compatibility adapter keeps the normal receive window"
        );
        CompatRadio::on_bulk_gatt_write(7, &[0; 7]);
        CompatRadio::on_bulk_gatt_read();
    }
}
