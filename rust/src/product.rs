//! Minimal light product composition shell.

use core::convert::Infallible;

use mesh_protocol::mesh_types::OtaState;
use tlsr8266_platform::api::{
    MeshRequest, MeshResponse, PlatformEvent, Product, ProductResult, RebootIndication,
    StatusRequest, StatusResponse, UartRequest, UartResponse,
};
use tlsr8266_platform::{
    AttError, GattReadOutcome, GattReadRequest, GattWriteOutcome, GattWriteRequest,
};

pub struct LightProduct;

impl Product for LightProduct {
    type GattId = Infallible;

    const PROFILE: &'static [tlsr8266_platform::CompiledAttr<
        tlsr8266_platform::gatt::AttrBinding<Self::GattId>,
    >] = &[];

    fn on_mesh(request: MeshRequest<'_>, _response: &mut MeshResponse) -> ProductResult {
        if request.payload.len() < 17 {
            return ProductResult::Unsupported;
        }

        let op = request.payload[0] & 0x3f;
        let mut params = [0_u8; 16];
        params.copy_from_slice(&request.payload[1..17]);

        match crate::light::decode_mesh_command(op, &params) {
            crate::light::LightCommandAction::EnqueueLight(message) => {
                crate::light::send_light_message(message.cmd, message.params);
                ProductResult::Handled
            }
            _ => ProductResult::Unsupported,
        }
    }

    fn on_uart(_request: UartRequest<'_>, _response: &mut UartResponse) -> ProductResult {
        ProductResult::Unsupported
    }

    fn on_gatt_read(_id: Infallible, _request: GattReadRequest<'_>) -> GattReadOutcome {
        GattReadOutcome::Error(AttError::ReadNotPermitted)
    }

    fn on_gatt_write(_id: Infallible, _request: GattWriteRequest<'_>) -> GattWriteOutcome {
        GattWriteOutcome::Error(AttError::WriteNotPermitted)
    }

    fn on_status(_request: StatusRequest, response: &mut StatusResponse) -> ProductResult {
        let on = if crate::light::transitions::is_light_off() {
            0
        } else {
            1
        };
        response.payload[0] = on;
        response.payload[1] = 0xff;
        response.len = 2;
        ProductResult::Reply
    }

    fn on_event(_event: PlatformEvent) {}

    fn on_pre_reboot(indication: RebootIndication) {
        match indication {
            RebootIndication::OtaSuccess => {
                crate::ota::rf_link_slave_ota_finish_led_and_reboot(OtaState::Ok);
            }
            RebootIndication::OtaFailure => {
                crate::ota::rf_link_slave_ota_finish_led_and_reboot(OtaState::Error);
            }
            RebootIndication::UserReset => {}
        }
    }
}

pub type LightPlatform = tlsr8266_platform::api::Platform<LightProduct>;
pub type LightPlatformGatt = tlsr8266_platform::gatt::PlatformGatt<LightProduct>;
pub type LightPlatformRadio = tlsr8266_platform::coex::PlatformRadio<LightProduct>;

tlsr8266_platform::define_platform_tasks!(light_platform_tasks, LightProduct);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn profile_is_empty() {
        assert_eq!(LightProduct::PROFILE.len(), 0);
    }

    #[test]
    fn status_reports_existing_light_state() {
        crate::light::reset_for_test();
        let mut response = StatusResponse::new();
        let result = LightPlatform::on_status(StatusRequest::new(0), &mut response);

        assert_eq!(result, ProductResult::Reply);
        assert_eq!(response.len, 2);
        assert_eq!(&response.payload[..2], &[0, 0xff]);
    }

    #[test]
    fn mesh_shell_rejects_short_payload() {
        let mut response = MeshResponse::new();
        let result = LightPlatform::on_mesh(MeshRequest::new(&[0]), &mut response);

        assert_eq!(result, ProductResult::Unsupported);
    }

    #[test]
    fn uart_and_gatt_shell_are_unsupported() {
        let mut response = UartResponse::new();
        assert_eq!(
            LightPlatform::on_uart(UartRequest::new(&[0]), &mut response),
            ProductResult::Unsupported
        );

        assert_eq!(size_of::<LightPlatformGatt>(), 0);
        assert_eq!(size_of::<LightPlatformRadio>(), 0);
    }
}
