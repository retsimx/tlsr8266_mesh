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
}

/// Action to take for a connection slot.
#[derive(Clone, Copy)]
pub enum ConnectionPlan {
    Continue,
    Transmit(Packet),
    Disconnect,
    Reschedule { delay_us: u32 },
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
}

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
        assert_eq!(listen_slot.cycle, 2);
        assert_eq!(advertisement_slot.cycle, 3);
    }

    #[test]
    fn actions_plans_and_att_remain_copy() {
        assert_copy::<RadioAction>();
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
        ];
        assert!(matches!(plans[2], ConnectionPlan::Disconnect));

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
    }
}
