//! Minimal light product composition shell.

use core::convert::Infallible;

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

    /// Runs the bounded synchronous indication, and nothing else.
    ///
    /// The platform owns the staging-area erase, the ready-flag arming and the
    /// reset itself; this hook only lights the indicator while the product still
    /// has code in flash, so no lock is taken here and no flash access happens.
    fn on_pre_reboot(indication: RebootIndication) {
        match indication {
            RebootIndication::OtaSuccess => {
                tlsr8266_hal::pm::blink_ota_led_ok(crate::config::OTA_LED as u32);
            }
            RebootIndication::OtaFailure => blink_ota_led_error(crate::config::OTA_LED as u32),
            RebootIndication::UserReset => {}
        }
    }
}

/// Signals a failed transfer: 60 toggles of 100 ms on `pin`.
///
/// Runs with no lock held (there is nothing to lock) and feeds the watchdog
/// between toggles, so the indication never trips it.
fn blink_ota_led_error(pin: u32) {
    use tlsr8266_hal::mcu::clock::sleep_us;
    use tlsr8266_hal::mcu::gpio::{gpio_set_func, gpio_set_output_en, gpio_write, AS_GPIO};
    use tlsr8266_hal::mcu::watchdog::wd_clear;

    gpio_set_func(pin, AS_GPIO);
    gpio_set_output_en(pin, 1);
    let mut led_on = true;
    for _ in 0..60 {
        gpio_write(pin, u32::from(led_on));
        led_on = !led_on;
        wd_clear();
        sleep_us(100 * 1000);
    }
}

pub type LightPlatform = tlsr8266_platform::api::Platform<LightProduct>;
pub type LightPlatformGatt = tlsr8266_platform::gatt::PlatformGatt<LightProduct>;
pub type LightPlatformRadio = tlsr8266_platform::coex::PlatformRadio<LightProduct>;

tlsr8266_platform::define_platform_tasks!(light_platform_tasks, LightProduct);

#[cfg(test)]
mod tests {
    use super::*;
    use tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog;
    use tlsr8266_hal::mcu::clock::fake_clock;
    use tlsr8266_pac::fake_mmio::FakeMmio;

    /// `reg_gpio_gpio_func`: GPIO-function select register base.
    const REG_GPIO_GPIO_FUNC: u32 = 0x586;
    /// `reg_gpio_out`: output-data register base.
    const REG_GPIO_OUT: u32 = 0x583;

    /// The indicator pin, and its per-port register offset.
    fn led_pin() -> (u32, u32) {
        let pin = crate::config::OTA_LED as u32;
        (pin, (pin >> 8) << 3)
    }

    /// The HAL host seams are process-wide singletons.
    fn reset_fakes() {
        FakeMmio::reset();
        FakeAnalog::reset();
        fake_clock::reset();
    }

    /// The values written to the register at `offset`, oldest first.
    fn writes_to(offset: u32) -> Vec<u32> {
        FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == (offset | tlsr8266_pac::REG_BASE_ADDR))
            .map(|w| w.value)
            .collect()
    }

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

        assert_eq!(core::mem::size_of::<LightPlatformGatt>(), 0);
        assert_eq!(core::mem::size_of::<LightPlatformRadio>(), 0);
    }

    /// The failure indication is the bounded 60-toggle / 100 ms blink loop, and
    /// it must not touch flash or reboot the MCU (the platform owns both).
    #[test]
    fn ota_failure_indication_runs_the_bounded_blink_only() {
        reset_fakes();

        LightProduct::on_pre_reboot(RebootIndication::OtaFailure);

        let (_, port_offset) = led_pin();

        assert_eq!(
            fake_clock::sleeps(),
            vec![100 * 1000u32; 60],
            "the failure indication must be 60 toggles of 100 ms"
        );
        assert_eq!(
            writes_to(REG_GPIO_GPIO_FUNC + port_offset).len(),
            1,
            "the pin must be switched to GPIO function exactly once"
        );
        assert_eq!(
            writes_to(REG_GPIO_OUT + port_offset).len(),
            60,
            "every toggle must drive the pin"
        );
        assert!(
            writes_to(0x6f).is_empty(),
            "the product must never latch the reset itself"
        );
    }

    /// The success indication is the HAL's blink helper, not the error loop.
    #[test]
    fn ota_success_indication_uses_the_hal_blink() {
        reset_fakes();

        LightProduct::on_pre_reboot(RebootIndication::OtaSuccess);

        assert!(
            !fake_clock::sleeps().is_empty(),
            "the success blink still drives the indicator"
        );
        assert_ne!(
            fake_clock::sleeps(),
            vec![100 * 1000u32; 60],
            "the success indication is not the failure loop"
        );
        assert!(
            writes_to(0x6f).is_empty(),
            "the product must never latch the reset itself"
        );
    }

    /// A user reset carries no OTA indication at all.
    #[test]
    fn user_reset_indication_is_a_no_op() {
        reset_fakes();

        LightProduct::on_pre_reboot(RebootIndication::UserReset);

        assert!(FakeMmio::writes().is_empty(), "a user reset lights nothing");
        assert!(fake_clock::sleeps().is_empty());
    }
}
