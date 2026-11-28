use std::sync::atomic::{AtomicU32, Ordering};

use mesh_protocol::packet_types::{Packet, PacketL2capHead};
use tlsr8266_platform::api::{
    MeshRequest, MeshResponse, Platform, PlatformEvent, Product, ProductResult, RebootIndication,
    StatusRequest, StatusResponse, UartRequest, UartResponse,
};
use tlsr8266_platform::coex::PlatformRadio;
use tlsr8266_platform::gatt::{AttrBinding, PlatformGatt};
use tlsr8266_platform::tasks;
use tlsr8266_radio::{AttrPermissions, AttrRole, AttrUuid, CompiledAttr, FrameIngressTag};
use tlsr8266_radio::{
    GattBackend, GattReadKind, GattReadOutcome, GattReadRequest, GattServer, GattWriteKind,
    GattWriteOutcome, GattWriteRequest, RadioCoexistence, RadioFrame,
};

static SWITCH_EVENT_COUNT: AtomicU32 = AtomicU32::new(0);
static SWITCH_REBOOT_COUNT: AtomicU32 = AtomicU32::new(0);
static SENSOR_EVENT_COUNT: AtomicU32 = AtomicU32::new(0);
static SENSOR_REBOOT_COUNT: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DummySwitchGattId {
    OnOff,
}

struct DummySwitch;

static DUMMY_SWITCH_PROFILE: [CompiledAttr<AttrBinding<DummySwitchGattId>>; 1] = [CompiledAttr {
    role: AttrRole::CharacteristicValue {
        permissions: AttrPermissions {
            read: true,
            write: true,
            write_without_rsp: true,
        },
    },
    uuid: AttrUuid::Uuid16(0x2a56),
    len: 1,
    max_len: 1,
    binding: AttrBinding::Product(DummySwitchGattId::OnOff),
}];

impl Product for DummySwitch {
    type GattId = DummySwitchGattId;
    const PROFILE: &'static [CompiledAttr<AttrBinding<Self::GattId>>] = &DUMMY_SWITCH_PROFILE;

    fn on_mesh(request: MeshRequest<'_>, response: &mut MeshResponse) -> ProductResult {
        if request.payload == b"switch" {
            response.payload[0..6].copy_from_slice(b"switch");
            response.len = 6;
            ProductResult::Reply
        } else {
            ProductResult::Ignored
        }
    }

    fn on_uart(_request: UartRequest<'_>, response: &mut UartResponse) -> ProductResult {
        response.payload[0] = 1;
        response.len = 1;
        ProductResult::Handled
    }

    fn on_gatt_read(
        id: Self::GattId,
        request: tlsr8266_radio::GattReadRequest<'_>,
    ) -> GattReadOutcome {
        assert_eq!(id, DummySwitchGattId::OnOff);
        if !request.out.is_empty() {
            request.out[0] = 1;
            GattReadOutcome::Value(1)
        } else {
            GattReadOutcome::Value(0)
        }
    }

    fn on_gatt_write(id: Self::GattId, _request: GattWriteRequest<'_>) -> GattWriteOutcome {
        assert_eq!(id, DummySwitchGattId::OnOff);
        GattWriteOutcome::Accepted
    }

    fn on_status(request: StatusRequest, response: &mut StatusResponse) -> ProductResult {
        response.payload[0..4].copy_from_slice(&request.tick.to_le_bytes());
        response.len = 4;
        ProductResult::Handled
    }

    fn on_event(event: PlatformEvent) {
        match event {
            PlatformEvent::Connected | PlatformEvent::OtaStarted => {
                SWITCH_EVENT_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            _ => {}
        }
    }

    fn on_pre_reboot(_indication: RebootIndication) {
        SWITCH_REBOOT_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DummySensorGattId {
    Temperature,
    Humidity,
}

struct DummySensor;

static DUMMY_SENSOR_PROFILE: [CompiledAttr<AttrBinding<DummySensorGattId>>; 2] = [
    CompiledAttr {
        role: AttrRole::CharacteristicValue {
            permissions: AttrPermissions {
                read: true,
                write: false,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid16(0x2a6e),
        len: 2,
        max_len: 2,
        binding: AttrBinding::Product(DummySensorGattId::Temperature),
    },
    CompiledAttr {
        role: AttrRole::CharacteristicValue {
            permissions: AttrPermissions {
                read: true,
                write: false,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid16(0x2a6f),
        len: 1,
        max_len: 1,
        binding: AttrBinding::Product(DummySensorGattId::Humidity),
    },
];

impl Product for DummySensor {
    type GattId = DummySensorGattId;
    const PROFILE: &'static [CompiledAttr<AttrBinding<Self::GattId>>] = &DUMMY_SENSOR_PROFILE;

    fn on_mesh(_request: MeshRequest<'_>, _response: &mut MeshResponse) -> ProductResult {
        ProductResult::Unsupported
    }

    fn on_uart(request: UartRequest<'_>, response: &mut UartResponse) -> ProductResult {
        if request.payload == b"sensor" {
            response.payload[0..6].copy_from_slice(b"sensor");
            response.len = 6;
            ProductResult::Reply
        } else {
            ProductResult::Ignored
        }
    }

    fn on_gatt_read(id: Self::GattId, request: GattReadRequest<'_>) -> GattReadOutcome {
        match id {
            DummySensorGattId::Temperature => {
                if request.out.len() >= 2 {
                    request.out[0..2].copy_from_slice(&21_i16.to_le_bytes());
                    GattReadOutcome::Value(2)
                } else {
                    GattReadOutcome::Value(0)
                }
            }
            DummySensorGattId::Humidity => {
                if !request.out.is_empty() {
                    request.out[0] = 45;
                    GattReadOutcome::Value(1)
                } else {
                    GattReadOutcome::Value(0)
                }
            }
        }
    }

    fn on_gatt_write(id: Self::GattId, _request: GattWriteRequest<'_>) -> GattWriteOutcome {
        match id {
            DummySensorGattId::Temperature | DummySensorGattId::Humidity => {
                GattWriteOutcome::Error(tlsr8266_radio::AttError::WriteNotPermitted)
            }
        }
    }

    fn on_status(request: StatusRequest, response: &mut StatusResponse) -> ProductResult {
        response.payload[0] = (request.tick & 0xff) as u8;
        response.len = 1;
        ProductResult::Handled
    }

    fn on_event(event: PlatformEvent) {
        match event {
            PlatformEvent::Connected | PlatformEvent::OtaFailed => {
                SENSOR_EVENT_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            _ => {}
        }
    }

    fn on_pre_reboot(_indication: RebootIndication) {
        SENSOR_REBOOT_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

static SWITCH_PLATFORM_PROFILE: [CompiledAttr<AttrBinding<DummySwitchGattId>>; 1] =
    [CompiledAttr {
        role: AttrRole::Descriptor {
            permissions: AttrPermissions {
                read: true,
                write: false,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid16(0x2902),
        len: 2,
        max_len: 2,
        binding: AttrBinding::Cccd,
    }];

static SENSOR_PLATFORM_PROFILE: [CompiledAttr<AttrBinding<DummySensorGattId>>; 1] =
    [CompiledAttr {
        role: AttrRole::Descriptor {
            permissions: AttrPermissions {
                read: true,
                write: false,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid16(0x2902),
        len: 2,
        max_len: 2,
        binding: AttrBinding::Cccd,
    }];

use tlsr8266_platform::define_platform_tasks;

define_platform_tasks!(dummy_switch_tasks, DummySwitch);
define_platform_tasks!(dummy_sensor_tasks, DummySensor);

fn packet() -> Packet {
    Packet {
        head: PacketL2capHead::default(),
    }
}

#[test]
fn dummy_products_dispatch_and_coexist_without_hook_swapping() {
    let mut mesh_response = MeshResponse::new();
    let mesh_result =
        Platform::<DummySwitch>::on_mesh(MeshRequest::new(b"switch"), &mut mesh_response);
    assert_eq!(mesh_result, ProductResult::Reply);
    assert_eq!(
        &mesh_response.payload[..mesh_response.len as usize],
        b"switch"
    );

    let mut uart_response = UartResponse::new();
    let uart_result =
        Platform::<DummySensor>::on_uart(UartRequest::new(b"sensor"), &mut uart_response);
    assert_eq!(uart_result, ProductResult::Reply);
    assert_eq!(
        &uart_response.payload[..uart_response.len as usize],
        b"sensor"
    );

    let mut status_response = StatusResponse::new();
    assert_eq!(
        Platform::<DummySwitch>::on_status(StatusRequest::new(0x1234_5678), &mut status_response,),
        ProductResult::Handled,
    );
    assert_eq!(&status_response.payload[..4], &0x1234_5678u32.to_le_bytes());

    Platform::<DummySwitch>::on_event(PlatformEvent::Connected);
    Platform::<DummySensor>::on_event(PlatformEvent::OtaFailed);
    Platform::<DummySwitch>::on_pre_reboot(RebootIndication::OtaSuccess);
    Platform::<DummySensor>::on_pre_reboot(RebootIndication::OtaFailure);
    assert_eq!(SWITCH_EVENT_COUNT.load(Ordering::Relaxed), 1);
    assert_eq!(SENSOR_EVENT_COUNT.load(Ordering::Relaxed), 1);
    assert_eq!(SWITCH_REBOOT_COUNT.load(Ordering::Relaxed), 1);
    assert_eq!(SENSOR_REBOOT_COUNT.load(Ordering::Relaxed), 1);
}

#[test]
fn independent_profiles_and_gatt_servers_coexist() {
    let switch_profile = PlatformGatt::<DummySwitch>::profile_view(&SWITCH_PLATFORM_PROFILE);
    let sensor_profile = PlatformGatt::<DummySensor>::profile_view(&SENSOR_PLATFORM_PROFILE);

    assert_eq!(switch_profile.platform.len(), 1);
    assert_eq!(switch_profile.product.len(), 1);
    assert_eq!(sensor_profile.platform.len(), 1);
    assert_eq!(sensor_profile.product.len(), 2);

    let _switch_server = GattServer::<PlatformGatt<DummySwitch>>::new(switch_profile);
    let _sensor_server = GattServer::<PlatformGatt<DummySensor>>::new(sensor_profile);

    let mut switch_read = [0u8; 4];
    let switch_outcome = PlatformGatt::<DummySwitch>::read(
        AttrBinding::Product(DummySwitchGattId::OnOff),
        GattReadRequest {
            handle: 2,
            kind: GattReadKind::Read,
            out: &mut switch_read,
        },
    );
    assert_eq!(switch_outcome, GattReadOutcome::Value(1));
    assert_eq!(switch_read[0], 1);

    let sensor_write = PlatformGatt::<DummySensor>::write(
        AttrBinding::Product(DummySensorGattId::Temperature),
        GattWriteRequest {
            handle: 2,
            kind: GattWriteKind::Request,
            l2cap_len: 0,
            payload: &[],
        },
    );
    assert_eq!(
        sensor_write,
        GattWriteOutcome::Error(tlsr8266_radio::AttError::WriteNotPermitted),
    );

    let cccd = PlatformGatt::<DummySwitch>::read(
        AttrBinding::Cccd,
        GattReadRequest {
            handle: 1,
            kind: GattReadKind::Read,
            out: &mut switch_read,
        },
    );
    assert_eq!(cccd, GattReadOutcome::Value(2));
}

#[test]
fn auxiliary_placeholder_has_no_shadow_queue() {
    let frame = RadioFrame {
        tag: FrameIngressTag::GattBridge,
        rf_validated: false,
        packet: packet(),
    };

    assert!(PlatformRadio::<DummySwitch>::submit_aux_frame(frame).is_ok());
    assert!(PlatformRadio::<DummySensor>::submit_aux_frame(frame).is_ok());

    assert!(matches!(
        PlatformRadio::<DummySwitch>::on_aux_receive(
            &frame,
            tlsr8266_radio::RxContext {
                rssi: 1,
                channel: 39,
                rx_tick: 100,
            },
        ),
        tlsr8266_radio::RadioAction::Ignore,
    ));
}

#[test]
fn task_macro_expands_concrete_wrappers_for_both_products() {
    let _ = core::any::type_name_of_val(&dummy_switch_tasks::mesh_rx);
    let _ = core::any::type_name_of_val(&dummy_switch_tasks::heartbeat);
    let _ = core::any::type_name_of_val(&dummy_sensor_tasks::mesh_rx);
    let _ = core::any::type_name_of_val(&dummy_sensor_tasks::heartbeat);

    let _ = core::any::type_name_of_val(&tasks::platform_mesh_rx_loop::<DummySwitch>);
    let _ = core::any::type_name_of_val(&tasks::platform_heartbeat_loop::<DummySensor>);
}
