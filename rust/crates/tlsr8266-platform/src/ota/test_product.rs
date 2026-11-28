//! Minimal non-light product for the OTA unit tests.
//!
//! The OTA engine is product-agnostic, so its unit tests need a `Product` that
//! does nothing and owns no profile. The integration test
//! (`tests/dummy_product.rs`) exercises the same lifecycle through two concrete
//! dummy products with observability.

use crate::api::{
    MeshRequest, MeshResponse, PlatformEvent, Product, ProductAttribute, ProductResult,
    RebootIndication, StatusRequest, StatusResponse, UartRequest, UartResponse,
};
use tlsr8266_radio::{GattReadOutcome, GattReadRequest, GattWriteOutcome, GattWriteRequest};

pub(crate) struct TestProduct;

impl Product for TestProduct {
    type GattId = ();

    const PROFILE: &'static [ProductAttribute<Self::GattId>] = &[];

    fn on_mesh(_request: MeshRequest<'_>, _response: &mut MeshResponse) -> ProductResult {
        ProductResult::Ignored
    }

    fn on_uart(_request: UartRequest<'_>, _response: &mut UartResponse) -> ProductResult {
        ProductResult::Ignored
    }

    fn on_gatt_read(_id: Self::GattId, _request: GattReadRequest<'_>) -> GattReadOutcome {
        GattReadOutcome::Value(0)
    }

    fn on_gatt_write(_id: Self::GattId, _request: GattWriteRequest<'_>) -> GattWriteOutcome {
        GattWriteOutcome::Accepted
    }

    fn on_status(_request: StatusRequest, _response: &mut StatusResponse) -> ProductResult {
        ProductResult::Ignored
    }

    fn on_event(_event: PlatformEvent) {}

    fn on_pre_reboot(_indication: RebootIndication) {}
}
