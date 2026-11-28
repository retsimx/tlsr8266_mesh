//! Static platform adapter for the radio coexistence contract.

use core::marker::PhantomData;

use mesh_protocol::packet_types::Packet;
use tlsr8266_radio::{
    AdvertisementSlot, AttTxPacket, AuxiliaryRfPlan, ConnectionPlan, ConnectionSlot, LinkAction,
    LinkEvent, LinkIdentity, ListenSlot, QueueFull, RadioAction, RadioCoexistence, RadioFrame,
    RxContext, StatusEnqueueOutcome,
};

use crate::api::Product;

/// Zero-sized coexistence adapter. It carries no shadow queue or mutable state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlatformRadio<P: Product>(PhantomData<P>);

impl<P: Product> PlatformRadio<P> {
    pub const fn new() -> Self {
        Self(PhantomData)
    }
}

impl<P: Product> Default for PlatformRadio<P> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P: Product> RadioCoexistence for PlatformRadio<P> {
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
            listen_channels: [0; 4],
        }
    }

    fn on_aux_receive(_frame: &RadioFrame, _context: RxContext) -> RadioAction {
        RadioAction::Ignore
    }

    fn submit_aux_frame(_frame: RadioFrame) -> Result<(), QueueFull> {
        // F-5 is contract-only. Production ingress remains in MESH_RX_CHANNEL;
        // this placeholder neither creates nor updates any shadow queue.
        Ok(())
    }

    fn on_link_event(_event: LinkEvent) -> LinkAction {
        LinkAction::None
    }

    fn on_connection_slot(_slot: ConnectionSlot) -> ConnectionPlan {
        ConnectionPlan::Continue
    }

    fn on_aux_listen_slot(_slot: ListenSlot) -> RadioAction {
        RadioAction::Ignore
    }

    fn on_post_advertisement(_slot: AdvertisementSlot) -> RadioAction {
        RadioAction::Ignore
    }

    fn poll_deferred_att() -> Option<AttTxPacket> {
        None
    }

    fn start_status_window() -> bool {
        crate::window::start_status_window()
    }

    fn stop_status_window() {
        crate::window::stop_status_window();
    }

    fn enqueue_status_response(source_addr: u8, packet: &Packet) -> StatusEnqueueOutcome {
        crate::window::enqueue_status_response(source_addr, packet)
    }

    fn peek_status_response() -> Option<Packet> {
        crate::window::peek_status_response()
    }

    fn advance_status_response() {
        crate::window::advance_status_response();
    }
}
