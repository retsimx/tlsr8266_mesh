//! Static GATT profile binding and backend dispatch.

use core::marker::PhantomData;

use tlsr8266_hal::common::sync::CriticalSectionMutex;
use tlsr8266_radio::{
    AttError, CompiledAttr, GattBackend, GattReadOutcome, GattReadRequest, GattWriteKind,
    GattWriteOutcome, GattWriteRequest, ProfileView,
};

use crate::api::Product;

/// Stored Client Characteristic Configuration value.
///
/// A CCCD is the one platform attribute a central actually writes: it says
/// whether notifications are enabled. Discarding it (acknowledging the write and
/// reading back zeros) makes every subscription silently dead, so the value is
/// kept here, exactly as the deployed single-CCCD slot did. There is one CCCD in
/// the deployed profile, so one slot.
static CCCD_VALUE: CriticalSectionMutex<[u8; 2]> = CriticalSectionMutex::new([0, 0]);

/// Identity of a platform-owned GATT attribute.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlatformAttrId {
    FirmwareVersion,
    ManufacturerName,
    ModelNumber,
    DeviceName,
}

/// Binding carried by a compiled GATT attribute.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AttrBinding<Id: Copy + 'static> {
    Static,
    ConstValue(&'static [u8]),
    Cccd,
    Platform(PlatformAttrId),
    Product(Id),
}

/// Generic GATT backend that dispatches product bindings directly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlatformGatt<P: Product>(PhantomData<P>);

impl<P: Product> PlatformGatt<P> {
    pub const fn new() -> Self {
        Self(PhantomData)
    }

    pub const fn profile_view(
        platform_attrs: &'static [CompiledAttr<AttrBinding<P::GattId>>],
    ) -> ProfileView<AttrBinding<P::GattId>> {
        ProfileView {
            platform: platform_attrs,
            product: P::PROFILE,
        }
    }
}

impl<P: Product> Default for PlatformGatt<P> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P: Product> GattBackend for PlatformGatt<P> {
    type Binding = AttrBinding<P::GattId>;

    fn read(binding: Self::Binding, request: GattReadRequest<'_>) -> GattReadOutcome {
        match binding {
            AttrBinding::Static => GattReadOutcome::Value(0),
            AttrBinding::ConstValue(value) => {
                let len = value.len().min(request.out.len()) as u8;
                request.out[..len as usize].copy_from_slice(&value[..len as usize]);
                GattReadOutcome::Value(len)
            }
            AttrBinding::Cccd => {
                let value = *CCCD_VALUE.lock();
                let len = value.len().min(request.out.len());
                request.out[..len].copy_from_slice(&value[..len]);
                GattReadOutcome::Value(len as u8)
            }
            AttrBinding::Platform(id) => match id {
                PlatformAttrId::FirmwareVersion
                | PlatformAttrId::ManufacturerName
                | PlatformAttrId::ModelNumber
                | PlatformAttrId::DeviceName => GattReadOutcome::Value(0),
            },
            AttrBinding::Product(id) => P::on_gatt_read(id, request),
        }
    }

    fn write(binding: Self::Binding, request: GattWriteRequest<'_>) -> GattWriteOutcome {
        match binding {
            AttrBinding::Static | AttrBinding::ConstValue(_) => {
                GattWriteOutcome::Error(AttError::WriteNotPermitted)
            }
            AttrBinding::Cccd => {
                // The deployed backend rejected a CCCD write that did not carry
                // two value bytes; acknowledging it would leave the central
                // believing subscriptions were enabled.
                if request.l2cap_len < 3 || request.payload.len() < 2 {
                    return GattWriteOutcome::Error(AttError::InvalidAttributeValueLength);
                }
                CCCD_VALUE.lock().copy_from_slice(&request.payload[..2]);
                match request.kind {
                    GattWriteKind::Request => GattWriteOutcome::Accepted,
                    GattWriteKind::Command => GattWriteOutcome::NoWriteResponse,
                }
            }
            AttrBinding::Platform(id) => match id {
                PlatformAttrId::FirmwareVersion
                | PlatformAttrId::ManufacturerName
                | PlatformAttrId::ModelNumber
                | PlatformAttrId::DeviceName => {
                    GattWriteOutcome::Error(AttError::WriteNotPermitted)
                }
            },
            AttrBinding::Product(id) => P::on_gatt_write(id, request),
        }
    }
}
