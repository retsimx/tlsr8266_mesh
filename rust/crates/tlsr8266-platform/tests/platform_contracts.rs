//! Coverage for the platform contract types that nothing else exercised.
//!
//! `config.rs` and `coex.rs` were at 0% and 25% line coverage: the dummy-product
//! fixture asserted that the types exist and are zero-sized, but never called
//! them (epic #111 audit). A placeholder that no test ever runs is free to rot.

use mesh_protocol::packet_types::PacketL2capHead;
use tlsr8266_platform::api::{PlatformEvent, Product, RebootIndication};
use tlsr8266_platform::coex::PlatformRadio;
use tlsr8266_platform::config::{FirmwareIdentity, PlatformConfig, ProductConfig};
use tlsr8266_radio::{
    AttTxPacket, ConnectionPlan, ConnectionSlot, FrameIngressTag, LinkAction, LinkEvent,
    ListenSlot, RadioAction, RadioCoexistence, RadioFrame, RxContext,
};

/// Minimal product so `PlatformRadio<P>` can be named.
struct FixtureProduct;

impl Product for FixtureProduct {
    type GattId = core::convert::Infallible;

    const PROFILE: &'static [tlsr8266_platform::api::ProductAttribute<Self::GattId>] = &[];

    fn on_mesh(
        _request: tlsr8266_platform::api::MeshRequest<'_>,
        _response: &mut tlsr8266_platform::api::MeshResponse,
    ) -> tlsr8266_platform::api::ProductResult {
        tlsr8266_platform::api::ProductResult::Ignored
    }

    fn on_uart(
        _request: tlsr8266_platform::api::UartRequest<'_>,
        _response: &mut tlsr8266_platform::api::UartResponse,
    ) -> tlsr8266_platform::api::ProductResult {
        tlsr8266_platform::api::ProductResult::Ignored
    }

    fn on_gatt_read(
        id: Self::GattId,
        _request: tlsr8266_radio::GattReadRequest<'_>,
    ) -> tlsr8266_radio::GattReadOutcome {
        match id {}
    }

    fn on_gatt_write(
        id: Self::GattId,
        _request: tlsr8266_radio::GattWriteRequest<'_>,
    ) -> tlsr8266_radio::GattWriteOutcome {
        match id {}
    }

    fn on_status(
        _request: tlsr8266_platform::api::StatusRequest,
        _response: &mut tlsr8266_platform::api::StatusResponse,
    ) -> tlsr8266_platform::api::ProductResult {
        tlsr8266_platform::api::ProductResult::Ignored
    }

    fn on_event(_event: PlatformEvent) {}

    fn on_pre_reboot(_indication: RebootIndication) {}
}

fn packet() -> mesh_protocol::packet_types::Packet {
    mesh_protocol::packet_types::Packet {
        head: PacketL2capHead::default(),
    }
}

#[test]
fn configuration_types_round_trip_their_fields() {
    let platform = PlatformConfig::new(250, false, true);
    assert_eq!(platform.heartbeat_ticks, 250);
    assert!(!platform.mesh_rx_enabled);
    assert!(platform.uart_enabled);

    let default = PlatformConfig::default();
    assert!(default.mesh_rx_enabled && default.uart_enabled);
    assert!(default.heartbeat_ticks > 0);

    let product = ProductConfig::new(0x001f, 0x0211, 0x0002);
    assert_eq!(product.device_address, 0x001f);
    assert_eq!(product.vendor_id, 0x0211);
    assert_eq!(ProductConfig::default().vendor_id, 0);

    let identity = FirmwareIdentity::new(0x0211, 0, 3543);
    assert_eq!(
        identity.firmware_version.to_le_bytes(),
        [0xd7, 0x0d, 0x00, 0x00],
        "the mesh OTA start response stamps this little-endian"
    );
    assert_eq!(FirmwareIdentity::default().firmware_version, 0);
}

#[test]
fn coexistence_placeholders_report_their_documented_defaults() {
    // These are the F-5 placeholder answers. Pinning them means a later
    // implementation has to change a failing test rather than silently altering
    // what the radio is told.
    let identity = PlatformRadio::<FixtureProduct>::link_identity();
    assert_eq!(identity.device_address, 0);
    assert_eq!(identity.vendor_id, 0);
    assert_eq!(identity.adv_len, 0);
    assert_eq!(identity.scan_rsp_len, 0);

    let plan = PlatformRadio::<FixtureProduct>::auxiliary_rf();
    assert_eq!(plan.access_code, 0);
    assert_eq!(plan.listen_channels, [0; 4]);

    let frame = RadioFrame {
        tag: FrameIngressTag::GattBridge,
        rf_validated: false,
        packet: packet(),
    };
    assert!(matches!(
        PlatformRadio::<FixtureProduct>::on_aux_receive(
            &frame,
            RxContext {
                rssi: 100,
                channel: 37,
                rx_tick: 1,
            },
        ),
        RadioAction::Ignore
    ));
    assert!(
        PlatformRadio::<FixtureProduct>::submit_aux_frame(frame).is_ok(),
        "the placeholder accepts a frame without creating a shadow queue"
    );
    assert_eq!(
        PlatformRadio::<FixtureProduct>::on_link_event(LinkEvent::Connected),
        LinkAction::None
    );
    assert!(matches!(
        PlatformRadio::<FixtureProduct>::on_connection_slot(ConnectionSlot {
            connected_tick: 0,
            next_connect_tick: 0,
            interval_ticks: 0,
            tx_ready: false,
        }),
        ConnectionPlan::Continue
    ));
    assert!(matches!(
        PlatformRadio::<FixtureProduct>::on_aux_listen_slot(ListenSlot { tick: 0, cycle: 0 }),
        RadioAction::Ignore
    ));
    assert!(matches!(
        PlatformRadio::<FixtureProduct>::on_post_advertisement(tlsr8266_radio::AdvertisementSlot {
            tick: 0,
            cycle: 0
        }),
        RadioAction::Ignore
    ));
    assert!(PlatformRadio::<FixtureProduct>::poll_deferred_att().is_none());
}

#[test]
fn status_window_round_trips_through_the_radio_port() {
    // The status window is the one coexistence method with real behaviour at
    // this stage; the radio drives it through these five calls.
    assert!(
        PlatformRadio::<FixtureProduct>::start_status_window(),
        "starting an idle window must succeed"
    );
    let queued = packet();
    assert_eq!(
        PlatformRadio::<FixtureProduct>::enqueue_status_response(7, &queued),
        tlsr8266_radio::StatusEnqueueOutcome::Added
    );
    assert_eq!(
        PlatformRadio::<FixtureProduct>::enqueue_status_response(7, &queued),
        tlsr8266_radio::StatusEnqueueOutcome::Duplicate,
        "the same source must not be queued twice"
    );
    assert!(PlatformRadio::<FixtureProduct>::peek_status_response().is_some());
    PlatformRadio::<FixtureProduct>::advance_status_response();
    assert!(
        PlatformRadio::<FixtureProduct>::peek_status_response().is_none(),
        "advancing past the only entry drains the window"
    );
    PlatformRadio::<FixtureProduct>::stop_status_window();
}

#[test]
fn att_placeholder_is_unused_but_constructible() {
    // `AttTxPacket` is the deferred-reply carrier; the placeholder never has one.
    let none: Option<AttTxPacket> = PlatformRadio::<FixtureProduct>::poll_deferred_att();
    assert!(none.is_none());
}
