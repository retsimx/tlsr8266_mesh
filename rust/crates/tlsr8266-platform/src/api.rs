//! Static product dispatch contracts.

use core::marker::PhantomData;

use tlsr8266_radio::{GattReadOutcome, GattReadRequest, GattWriteOutcome, GattWriteRequest};

use crate::gatt::AttrBinding;

/// A compiled product attribute binding.
pub type ProductAttribute<Id> = tlsr8266_radio::CompiledAttr<AttrBinding<Id>>;

/// Fixed result of a product dispatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProductResult {
    Handled,
    Reply,
    Ignored,
    Unsupported,
}

/// Borrowed mesh dispatch input.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MeshRequest<'a> {
    pub payload: &'a [u8],
}

impl<'a> MeshRequest<'a> {
    pub const fn new(payload: &'a [u8]) -> Self {
        Self { payload }
    }
}

/// Fixed-size mesh dispatch response.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MeshResponse {
    pub payload: [u8; 32],
    pub len: u8,
}

impl MeshResponse {
    pub const fn new() -> Self {
        Self {
            payload: [0; 32],
            len: 0,
        }
    }
}

impl Default for MeshResponse {
    fn default() -> Self {
        Self::new()
    }
}

/// Borrowed UART dispatch input.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UartRequest<'a> {
    pub payload: &'a [u8],
}

impl<'a> UartRequest<'a> {
    pub const fn new(payload: &'a [u8]) -> Self {
        Self { payload }
    }
}

/// Fixed-size UART dispatch response.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UartResponse {
    pub payload: [u8; 32],
    pub len: u8,
}

impl UartResponse {
    pub const fn new() -> Self {
        Self {
            payload: [0; 32],
            len: 0,
        }
    }
}

impl Default for UartResponse {
    fn default() -> Self {
        Self::new()
    }
}

/// Status dispatch input.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StatusRequest {
    pub tick: u32,
}

impl StatusRequest {
    pub const fn new(tick: u32) -> Self {
        Self { tick }
    }
}

/// Fixed-size status dispatch response.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StatusResponse {
    pub payload: [u8; 32],
    pub len: u8,
}

impl StatusResponse {
    pub const fn new() -> Self {
        Self {
            payload: [0; 32],
            len: 0,
        }
    }
}

impl Default for StatusResponse {
    fn default() -> Self {
        Self::new()
    }
}

/// Product-visible platform lifecycle events.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlatformEvent {
    Connected,
    Disconnected,
    OtaStarted,
    OtaCompleted,
    OtaFailed,
    AuthWindowExpired,
}

/// Completion indication immediately before reboot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RebootIndication {
    OtaSuccess,
    OtaFailure,
    UserReset,
}

/// A product implemented as a static zero-sized type.
pub trait Product: 'static {
    type GattId: Copy + 'static;

    const PROFILE: &'static [ProductAttribute<Self::GattId>];

    fn on_mesh(request: MeshRequest<'_>, response: &mut MeshResponse) -> ProductResult;
    fn on_uart(request: UartRequest<'_>, response: &mut UartResponse) -> ProductResult;
    fn on_gatt_read(id: Self::GattId, request: GattReadRequest<'_>) -> GattReadOutcome;
    fn on_gatt_write(id: Self::GattId, request: GattWriteRequest<'_>) -> GattWriteOutcome;
    fn on_status(request: StatusRequest, response: &mut StatusResponse) -> ProductResult;
    fn on_event(event: PlatformEvent);
    fn on_pre_reboot(indication: RebootIndication);
}

/// Zero-sized monomorphized dispatch shell for one product.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Platform<P: Product>(PhantomData<P>);

impl<P: Product> Platform<P> {
    pub const fn new() -> Self {
        Self(PhantomData)
    }

    pub fn on_mesh(request: MeshRequest<'_>, response: &mut MeshResponse) -> ProductResult {
        P::on_mesh(request, response)
    }

    pub fn on_uart(request: UartRequest<'_>, response: &mut UartResponse) -> ProductResult {
        P::on_uart(request, response)
    }

    pub fn on_gatt_read(id: P::GattId, request: GattReadRequest<'_>) -> GattReadOutcome {
        P::on_gatt_read(id, request)
    }

    pub fn on_gatt_write(id: P::GattId, request: GattWriteRequest<'_>) -> GattWriteOutcome {
        P::on_gatt_write(id, request)
    }

    pub fn on_status(request: StatusRequest, response: &mut StatusResponse) -> ProductResult {
        P::on_status(request, response)
    }

    pub fn on_event(event: PlatformEvent) {
        P::on_event(event)
    }

    pub fn on_pre_reboot(indication: RebootIndication) {
        P::on_pre_reboot(indication)
    }
}

impl<P: Product> Default for Platform<P> {
    fn default() -> Self {
        Self::new()
    }
}
