use core::marker::PhantomData;
use core::sync::atomic::AtomicU8;

use crate::ble::pair::{pair_read_arm, pair_write_payload, PAIR_LOGIN_OK};
use crate::coex::RadioCoexistence;
use crate::facade::RadioConfig;
use crate::mesh_ll::node_status::{mesh_report_status_enable, mesh_report_status_enable_mask};
use crate::rf::drv_8266::rf_link_slave_data_write_payload;
use mesh_protocol::service::{
    TELINK_SPP_DATA_CLIENT2SERVER, TELINK_SPP_DATA_OTA, TELINK_SPP_DATA_PAIR,
    TELINK_SPP_DATA_SERVER2CLIENT, TELINK_SPP_UUID_SERVICE,
};
use tlsr8266_hal::common::sync::{CriticalSectionMutex, SimplifyLS};

pub type ValueLength = u8;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum AttError {
    AttributeNotFound = 0x0a,
    InvalidHandle = 0x01,
    ReadNotPermitted = 0x02,
    WriteNotPermitted = 0x03,
    InvalidAttributeValueLength = 0x0d,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GattReadKind {
    Read,
    ReadByType,
}

pub struct GattReadRequest<'a> {
    pub handle: u16,
    pub kind: GattReadKind,
    pub out: &'a mut [u8],
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GattReadOutcome {
    Value(ValueLength),
    Deferred,
    Error(AttError),
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GattWriteKind {
    Request,
    Command,
}

pub struct GattWriteRequest<'a> {
    pub handle: u16,
    pub kind: GattWriteKind,
    pub l2cap_len: u16,
    pub payload: &'a [u8],
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GattWriteOutcome {
    Accepted,
    NoWriteResponse,
    Error(AttError),
}

pub trait GattBackend {
    type Binding: Copy + 'static;

    fn read(binding: Self::Binding, request: GattReadRequest<'_>) -> GattReadOutcome;
    fn write(binding: Self::Binding, request: GattWriteRequest<'_>) -> GattWriteOutcome;
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AttrUuid {
    Uuid16(u16),
    Uuid128(&'static [u8; 16]),
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct AttrPermissions {
    pub read: bool,
    pub write: bool,
    pub write_without_rsp: bool,
}

impl AttrPermissions {
    pub const fn allows(&self, kind: GattWriteKind) -> bool {
        match kind {
            // The deployed firmware had a single "writable" notion: a Write
            // Request to an attribute that accepts write-without-response was
            // still executed and acknowledged. Narrowing this to the `write`
            // property would silently discard (and falsely ACK) a 0x12 write to
            // the OTA or status characteristic, which declare only
            // write-without-response. Migration must preserve the legacy
            // response, not tighten it.
            GattWriteKind::Request => self.write || self.write_without_rsp,
            GattWriteKind::Command => self.write_without_rsp,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AttrRole {
    PrimaryService { end_handle: u16 },
    CharacteristicDecl { properties: u8, value_handle: u16 },
    CharacteristicValue { permissions: AttrPermissions },
    Descriptor { permissions: AttrPermissions },
}

#[derive(Clone, Copy)]
pub struct CompiledAttr<B: 'static> {
    pub role: AttrRole,
    pub uuid: AttrUuid,
    pub len: u8,
    pub max_len: u8,
    pub binding: B,
}

#[derive(Clone, Copy)]
pub struct ProfileView<B: 'static> {
    pub platform: &'static [CompiledAttr<B>],
    pub product: &'static [CompiledAttr<B>],
}

impl<B: 'static> ProfileView<B> {
    pub fn total_handles(&self) -> usize {
        self.platform.len() + self.product.len()
    }

    pub fn get(&self, handle: u16) -> Option<&'static CompiledAttr<B>> {
        if handle == 0 {
            return None;
        }
        let index = handle as usize - 1;
        if index < self.platform.len() {
            Some(&self.platform[index])
        } else {
            let product_index = index - self.platform.len();
            self.product.get(product_index)
        }
    }
}

/** @addtogroup GATT_Characteristic_Property GATT characteristic properties
 * @{
 */
pub const CHAR_PROP_BROADCAST: u8 = 0x01;
// permit broadcasts of the Characteristic Value
pub const CHAR_PROP_READ: u8 = 0x02;
// permit reads of the Characteristic Value
pub const CHAR_PROP_WRITE_WITHOUT_RSP: u8 = 0x04;
// Permit writes of the Characteristic Value without response
pub const CHAR_PROP_WRITE: u8 = 0x08;
// Permit writes of the Characteristic Value with response
pub const CHAR_PROP_NOTIFY: u8 = 0x10;
// Permit notifications of a Characteristic Value without acknowledgement
pub const CHAR_PROP_INDICATE: u8 = 0x20;
// Permit indications of a Characteristic Value with acknowledgement
pub const CHAR_PROP_AUTHEN: u8 = 0x40;
// permit signed writes to the Characteristic Value
pub const CHAR_PROP_EXTENDED: u8 = 0x80;
// additional characteristic properties are defined
/** @} end of group GATT_Characteristic_Property */
/** @addtogroup GATT_CCCC_Bits Client CharacteristicConfiguration bits
 * @{
 */
pub const CLIENT_CHAR_CFG_NOTI: u16 = 0x0001;
// permit broadcasts of the Characteristic Value
pub const CLIENT_CHAR_CFG_IND: u16 = 0x0002;
// permit reads of the Characteristic Value
/** @} end of group GATT_CCCC_Bits */
/** @addtogroup GATT_Property_length GATT characteristic property length
 * @{
 */
pub const CHAR_PROP_SIZE: u8 = 1;
/** @} end of group GATT_Property_length */
/** @addtogroup GATT_Char_Cfg_Bit_length GATT characteristic configuration Bits length
 * @{
 */
pub const CHAR_CFG_BITS_SIZE: u8 = 2;
/** @} end of group GATT_Char_Cfg_Bit_length */
pub const GATT_UUID_PRIMARY_SERVICE: u16 = 0x2800; // Primary Service
pub const GATT_UUID_SECONDARY_SERVICE: u16 = 0x2801; // Secondary Service
pub const GATT_UUID_INCLUDE: u16 = 0x2802; // Include
pub const GATT_UUID_CHARACTER: u16 = 0x2803; // Characteristic
pub const GATT_UUID_CHAR_EXT_PROPS: u16 = 0x2900; // Characteristic Extended Properties
pub const GATT_UUID_CHAR_USER_DESC: u16 = 0x2901; // Characteristic User Description
pub const GATT_UUID_CLIENT_CHAR_CFG: u16 = 0x2902; // Client Characteristic Configuration
pub const GATT_UUID_SERVER_CHAR_CFG: u16 = 0x2903; // Server Characteristic Configuration
pub const GATT_UUID_CHAR_PRESENT_FORMAT: u16 = 0x2904; // Characteristic Present Format
pub const GATT_UUID_CHAR_AGG_FORMAT: u16 = 0x2905; // Characteristic Aggregate Format
pub const GATT_UUID_VALID_RANGE: u16 = 0x2906; // Valid Range
pub const GATT_UUID_EXT_REPORT_REF: u16 = 0x2907; // External Report Reference
pub const GATT_UUID_REPORT_REF: u16 = 0x2908; // Report Reference

pub const GATT_UUID_DEVICE_NAME: u16 = 0x2a00; // Report Reference

pub const CHARACTERISTIC_UUID_MANU_NAME_STRING: u16 = 0x2A29;
pub const CHARACTERISTIC_UUID_MODEL_NUM_STRING: u16 = 0x2A24;
pub const CHARACTERISTIC_UUID_SERIAL_NUM_STRING: u16 = 0x2A25;
pub const CHARACTERISTIC_UUID_HW_REVISION_STRING: u16 = 0x2A27;
pub const CHARACTERISTIC_UUID_FW_REVISION_STRING: u16 = 0x2A26;
pub const CHARACTERISTIC_UUID_SW_REVISION_STRING: u16 = 0x2A28;
pub const CHARACTERISTIC_UUID_SYSTEM_ID: u16 = 0x2A23;
pub const CHARACTERISTIC_UUID_IEEE_11073_CERT_LIST: u16 = 0x2A2A;
pub const CHARACTERISTIC_UUID_PNP_ID: u16 = 0x2A50;

pub const GAP_APPEARE_UNKNOWN: u16 = 0x0000;

static APPEARANCE_UIID: u16 = 0x2a01;
static MODEL_ID_VALUE: &[u8] = b"model id 123";

// ---- Config-driven GATT values --------------------------------------------
// The 29-entry table below stays immutable in `.rodata`, so the three values
// that `RadioConfig` drives live in these SRAM buffers instead of in the table
// itself. `init_gatt_profile` fills them and `LegacyGattBackend` determines
// the exact bytes and runtime lengths to serve.

/// Maximum device-name value: the ReadByType response value slot starts at
/// offset 3 of a 22-byte array, so 19 bytes is the largest that always fits.
const DEV_NAME_MAX: u8 = 19;
/// Firmware revision value: the little-endian build-version word.
const FW_REVISION_MAX: u8 = 4;
/// Maximum manufacturer-name value (the mesh network name is 16 bytes).
const MANU_NAME_MAX: u8 = 16;

/// Device-name value (GATT handle 3). Written only by `init_gatt_profile`.
static DEV_NAME_BUF: CriticalSectionMutex<[u8; DEV_NAME_MAX as usize]> =
    CriticalSectionMutex::new(*b"Telink tLight\0\0\0\0\0\0");
static DEV_NAME_LEN: AtomicU8 = AtomicU8::new(13);

/// Manufacturer-name value (GATT handle 11), i.e. the mesh network name.
static MANU_NAME_BUF: CriticalSectionMutex<[u8; MANU_NAME_MAX as usize]> =
    CriticalSectionMutex::new(*b"out_of_mesh\0\0\0\0\0");
static MANU_NAME_LEN: AtomicU8 = AtomicU8::new(11);

/// Firmware-revision value (GATT handle 9), the build-version word.
static FW_REVISION_BUF: CriticalSectionMutex<[u8; FW_REVISION_MAX as usize]> =
    CriticalSectionMutex::new([0; 4]);

//////////////////////// SPP /////////////////////////////////////////////////////
pub static TELINK_SPP_SERVICE_UUID: [u8; 16] = TELINK_SPP_UUID_SERVICE;
pub static TELINK_SPP_DATA_SERVER2CLIENT_UUID: [u8; 16] = TELINK_SPP_DATA_SERVER2CLIENT;
pub static TELINK_SPP_DATA_CLIENT2SERVICE_UUID: [u8; 16] = TELINK_SPP_DATA_CLIENT2SERVER;
pub static TELINK_SPP_DATA_OTA_UUID: [u8; 16] = TELINK_SPP_DATA_OTA;
pub static TELINK_SPP_DATA_PAIR_UUID: [u8; 16] = TELINK_SPP_DATA_PAIR;

// Spp data from Server to Client characteristic variables
static SPP_DATA_SERVER2CLIENT_PROP: u8 = CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_NOTIFY;

// Spp data from Client to Server characteristic variables
static SPP_DATA_CLIENT2SERVER_PROP: u8 =
    CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RSP;

// Spp data for OTA characteristic variables
static SPP_DATA_OTA_PROP: u8 = CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP;
static SPP_DATA_OTA_DATA: [u8; 20] = [0xe0; 20];

static SPP_DATA_SERVER2CLIENT_DATA: CriticalSectionMutex<[u8; 4]> =
    CriticalSectionMutex::new([0; 4]);
static STATUS_CCC: CriticalSectionMutex<[u8; 2]> = CriticalSectionMutex::new([0x01, 0x00]);

// Spp data for OTA characteristic variables
static SPP_DATA_PAIR_PROP: u8 = CHAR_PROP_READ | CHAR_PROP_WRITE;
static SPP_DATA_PAIR_DATA: [u8; 20] = [0xe0; 20];

/// Client → server SPP value. Owned here as a single canonical status buffer.
pub static SEND_TO_MASTER: CriticalSectionMutex<[u8; 16]> = CriticalSectionMutex::new([0; 16]);

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LegacyBinding {
    None,
    Const(&'static [u8]),
    DeviceName,
    FirmwareRevision,
    ManufacturerName,
    MeshStatus,
    StatusCccd,
    SppCommand,
    Ota,
    Pair,
}

static GAP_SERVICE_VALUE: [u8; 2] = [0x00, 0x18];
static DEV_INFO_SERVICE_VALUE: [u8; 2] = [0x0a, 0x18];
static APPEARANCE_VALUE: [u8; 2] = [0x00, 0x00];
static HW_REVISION_VALUE_BYTES: [u8; 4] = [0x22, 0x22, 0x22, 0x22];

pub const LEGACY_PROFILE_ATTRS: [CompiledAttr<LegacyBinding>; 28] = [
    CompiledAttr {
        role: AttrRole::PrimaryService { end_handle: 6 },
        uuid: AttrUuid::Uuid16(GATT_UUID_PRIMARY_SERVICE),
        len: 2,
        max_len: 2,
        binding: LegacyBinding::Const(&GAP_SERVICE_VALUE),
    },
    CompiledAttr {
        role: AttrRole::CharacteristicDecl {
            properties: CHAR_PROP_READ,
            value_handle: 3,
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_CHARACTER),
        len: 1,
        max_len: 1,
        binding: LegacyBinding::None,
    },
    CompiledAttr {
        role: AttrRole::CharacteristicValue {
            permissions: AttrPermissions {
                read: true,
                write: false,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_DEVICE_NAME),
        len: 13,
        max_len: 19,
        binding: LegacyBinding::DeviceName,
    },
    CompiledAttr {
        role: AttrRole::Descriptor {
            permissions: AttrPermissions {
                read: true,
                write: false,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_CHAR_USER_DESC),
        len: 7,
        max_len: 7,
        binding: LegacyBinding::Const(b"DevName"),
    },
    CompiledAttr {
        role: AttrRole::CharacteristicDecl {
            properties: CHAR_PROP_READ,
            value_handle: 6,
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_CHARACTER),
        len: 1,
        max_len: 1,
        binding: LegacyBinding::None,
    },
    CompiledAttr {
        role: AttrRole::CharacteristicValue {
            permissions: AttrPermissions {
                read: true,
                write: false,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid16(APPEARANCE_UIID),
        len: 2,
        max_len: 2,
        binding: LegacyBinding::Const(&APPEARANCE_VALUE),
    },
    CompiledAttr {
        role: AttrRole::PrimaryService { end_handle: 9 },
        uuid: AttrUuid::Uuid16(GATT_UUID_PRIMARY_SERVICE),
        len: 2,
        max_len: 2,
        binding: LegacyBinding::Const(&DEV_INFO_SERVICE_VALUE),
    },
    CompiledAttr {
        role: AttrRole::CharacteristicDecl {
            properties: CHAR_PROP_READ,
            value_handle: 9,
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_CHARACTER),
        len: 1,
        max_len: 1,
        binding: LegacyBinding::None,
    },
    CompiledAttr {
        role: AttrRole::CharacteristicValue {
            permissions: AttrPermissions {
                read: true,
                write: false,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid16(CHARACTERISTIC_UUID_FW_REVISION_STRING),
        len: 4,
        max_len: 4,
        binding: LegacyBinding::FirmwareRevision,
    },
    CompiledAttr {
        role: AttrRole::CharacteristicDecl {
            properties: CHAR_PROP_READ,
            value_handle: 11,
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_CHARACTER),
        len: 1,
        max_len: 1,
        binding: LegacyBinding::None,
    },
    CompiledAttr {
        role: AttrRole::CharacteristicValue {
            permissions: AttrPermissions {
                read: true,
                write: false,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid16(CHARACTERISTIC_UUID_MANU_NAME_STRING),
        len: 11,
        max_len: 16,
        binding: LegacyBinding::ManufacturerName,
    },
    CompiledAttr {
        role: AttrRole::CharacteristicDecl {
            properties: CHAR_PROP_READ,
            value_handle: 13,
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_CHARACTER),
        len: 1,
        max_len: 1,
        binding: LegacyBinding::None,
    },
    CompiledAttr {
        role: AttrRole::CharacteristicValue {
            permissions: AttrPermissions {
                read: true,
                write: false,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid16(CHARACTERISTIC_UUID_MODEL_NUM_STRING),
        len: 12,
        max_len: 12,
        binding: LegacyBinding::Const(MODEL_ID_VALUE),
    },
    CompiledAttr {
        role: AttrRole::CharacteristicDecl {
            properties: CHAR_PROP_READ,
            value_handle: 15,
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_CHARACTER),
        len: 1,
        max_len: 1,
        binding: LegacyBinding::None,
    },
    CompiledAttr {
        role: AttrRole::CharacteristicValue {
            permissions: AttrPermissions {
                read: true,
                write: false,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid16(CHARACTERISTIC_UUID_HW_REVISION_STRING),
        len: 4,
        max_len: 4,
        binding: LegacyBinding::Const(&HW_REVISION_VALUE_BYTES),
    },
    CompiledAttr {
        role: AttrRole::PrimaryService { end_handle: 28 },
        uuid: AttrUuid::Uuid16(GATT_UUID_PRIMARY_SERVICE),
        len: 16,
        max_len: 16,
        binding: LegacyBinding::Const(&TELINK_SPP_SERVICE_UUID),
    },
    CompiledAttr {
        role: AttrRole::CharacteristicDecl {
            properties: SPP_DATA_SERVER2CLIENT_PROP,
            value_handle: 18,
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_CHARACTER),
        len: 1,
        max_len: 1,
        binding: LegacyBinding::None,
    },
    CompiledAttr {
        role: AttrRole::CharacteristicValue {
            permissions: AttrPermissions {
                read: true,
                write: true,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid128(&TELINK_SPP_DATA_SERVER2CLIENT_UUID),
        len: 1,
        max_len: 1,
        binding: LegacyBinding::MeshStatus,
    },
    CompiledAttr {
        role: AttrRole::Descriptor {
            permissions: AttrPermissions {
                read: true,
                write: true,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_CLIENT_CHAR_CFG),
        len: 2,
        max_len: 2,
        binding: LegacyBinding::StatusCccd,
    },
    CompiledAttr {
        role: AttrRole::CharacteristicDecl {
            properties: SPP_DATA_CLIENT2SERVER_PROP,
            value_handle: 21,
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_CHARACTER),
        len: 1,
        max_len: 1,
        binding: LegacyBinding::None,
    },
    CompiledAttr {
        role: AttrRole::CharacteristicValue {
            permissions: AttrPermissions {
                read: true,
                write: true,
                write_without_rsp: true,
            },
        },
        uuid: AttrUuid::Uuid128(&TELINK_SPP_DATA_CLIENT2SERVICE_UUID),
        len: 16,
        max_len: 16,
        binding: LegacyBinding::SppCommand,
    },
    CompiledAttr {
        role: AttrRole::Descriptor {
            permissions: AttrPermissions {
                read: true,
                write: false,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_CHAR_USER_DESC),
        len: 7,
        max_len: 7,
        binding: LegacyBinding::Const(b"Command"),
    },
    CompiledAttr {
        role: AttrRole::CharacteristicDecl {
            properties: SPP_DATA_OTA_PROP,
            value_handle: 24,
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_CHARACTER),
        len: 1,
        max_len: 1,
        binding: LegacyBinding::None,
    },
    CompiledAttr {
        role: AttrRole::CharacteristicValue {
            permissions: AttrPermissions {
                read: true,
                write: false,
                write_without_rsp: true,
            },
        },
        uuid: AttrUuid::Uuid128(&TELINK_SPP_DATA_OTA_UUID),
        len: 16,
        max_len: 16,
        binding: LegacyBinding::Ota,
    },
    CompiledAttr {
        role: AttrRole::Descriptor {
            permissions: AttrPermissions {
                read: true,
                write: false,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_CHAR_USER_DESC),
        len: 3,
        max_len: 3,
        binding: LegacyBinding::Const(b"OTA"),
    },
    CompiledAttr {
        role: AttrRole::CharacteristicDecl {
            properties: SPP_DATA_PAIR_PROP,
            value_handle: 27,
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_CHARACTER),
        len: 1,
        max_len: 1,
        binding: LegacyBinding::None,
    },
    CompiledAttr {
        role: AttrRole::CharacteristicValue {
            permissions: AttrPermissions {
                read: true,
                write: true,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid128(&TELINK_SPP_DATA_PAIR_UUID),
        len: 16,
        max_len: 16,
        binding: LegacyBinding::Pair,
    },
    CompiledAttr {
        role: AttrRole::Descriptor {
            permissions: AttrPermissions {
                read: true,
                write: false,
                write_without_rsp: false,
            },
        },
        uuid: AttrUuid::Uuid16(GATT_UUID_CHAR_USER_DESC),
        len: 4,
        max_len: 4,
        binding: LegacyBinding::Const(b"Pair"),
    },
];

pub const LEGACY_PROFILE: ProfileView<LegacyBinding> = ProfileView {
    platform: &LEGACY_PROFILE_ATTRS,
    product: &[],
};

fn copy_read_value(out: &mut [u8], value: &[u8]) -> GattReadOutcome {
    let len = value.len().min(out.len()).min(u8::MAX as usize) as ValueLength;
    out[..len as usize].copy_from_slice(&value[..len as usize]);
    GattReadOutcome::Value(len)
}

pub fn mesh_status_write_bytes(l2cap_len: u16, payload: &[u8]) {
    if !PAIR_LOGIN_OK.get() {
        return;
    }

    let copy_len = payload.len().min(4);
    if copy_len > 0 {
        SPP_DATA_SERVER2CLIENT_DATA.lock()[..copy_len].copy_from_slice(&payload[..copy_len]);
    }

    if l2cap_len > 4 {
        let payload_len = l2cap_len.saturating_sub(3) as usize;
        let payload_len = payload_len.min(payload.len());
        if payload_len > 0 {
            mesh_report_status_enable_mask(&payload[..payload_len]);
        }
    } else if let Some(&enabled) = payload.first() {
        mesh_report_status_enable(enabled != 0);
    }
}

/// Legacy GATT profile backend.
///
/// The coexistence implementation owns every policy decision the legacy
/// bindings used to make here: the OTA characteristic only hands the ATT
/// payload over through [`RadioCoexistence::on_bulk_gatt_write`] /
/// [`RadioCoexistence::on_bulk_gatt_read`].
pub struct LegacyGattBackend<C: RadioCoexistence>(PhantomData<C>);

impl<C: RadioCoexistence> GattBackend for LegacyGattBackend<C> {
    type Binding = LegacyBinding;

    fn read(binding: Self::Binding, request: GattReadRequest<'_>) -> GattReadOutcome {
        match binding {
            LegacyBinding::None => GattReadOutcome::Error(AttError::ReadNotPermitted),
            LegacyBinding::Const(value) => copy_read_value(request.out, value),
            LegacyBinding::DeviceName => {
                let len = DEV_NAME_LEN.get().min(DEV_NAME_MAX);
                let buf = DEV_NAME_BUF.lock();
                copy_read_value(request.out, &buf[..len as usize])
            }
            LegacyBinding::FirmwareRevision => {
                copy_read_value(request.out, &FW_REVISION_BUF.lock()[..])
            }
            LegacyBinding::ManufacturerName => {
                let len = MANU_NAME_LEN.get().min(MANU_NAME_MAX);
                let buf = MANU_NAME_BUF.lock();
                copy_read_value(request.out, &buf[..len as usize])
            }
            LegacyBinding::MeshStatus => {
                let value = SPP_DATA_SERVER2CLIENT_DATA.lock();
                copy_read_value(request.out, &value[..1])
            }
            LegacyBinding::StatusCccd => {
                let value = STATUS_CCC.lock();
                copy_read_value(request.out, &value[..])
            }
            LegacyBinding::SppCommand => {
                let mut value = SEND_TO_MASTER.lock();
                let outcome = copy_read_value(request.out, &value[..]);
                if request.kind == GattReadKind::Read {
                    value.fill(0);
                }
                outcome
            }
            LegacyBinding::Ota => {
                let outcome = copy_read_value(request.out, &SPP_DATA_OTA_DATA[..16]);
                if request.kind == GattReadKind::Read {
                    // A read is the initiator asking for the terminate
                    // notifications. Whether that means anything is the
                    // platform's decision, not the radio's.
                    C::on_bulk_gatt_read();
                }
                outcome
            }
            LegacyBinding::Pair => match request.kind {
                GattReadKind::Read => {
                    pair_read_arm();
                    GattReadOutcome::Deferred
                }
                GattReadKind::ReadByType => copy_read_value(request.out, &SPP_DATA_PAIR_DATA[..16]),
            },
        }
    }

    fn write(binding: Self::Binding, request: GattWriteRequest<'_>) -> GattWriteOutcome {
        match binding {
            LegacyBinding::None => GattWriteOutcome::NoWriteResponse,
            LegacyBinding::Const(_)
            | LegacyBinding::DeviceName
            | LegacyBinding::FirmwareRevision
            | LegacyBinding::ManufacturerName => GattWriteOutcome::Accepted,
            LegacyBinding::MeshStatus => {
                mesh_status_write_bytes(request.l2cap_len, request.payload);
                if request.kind == GattWriteKind::Request {
                    GattWriteOutcome::Accepted
                } else {
                    GattWriteOutcome::NoWriteResponse
                }
            }
            LegacyBinding::StatusCccd => {
                if request.l2cap_len < 3 || request.payload.len() < 2 {
                    GattWriteOutcome::Error(AttError::InvalidAttributeValueLength)
                } else {
                    STATUS_CCC.lock().copy_from_slice(&request.payload[..2]);
                    if request.kind == GattWriteKind::Request {
                        GattWriteOutcome::Accepted
                    } else {
                        GattWriteOutcome::NoWriteResponse
                    }
                }
            }
            LegacyBinding::SppCommand => {
                rf_link_slave_data_write_payload(
                    match request.kind {
                        GattWriteKind::Request => 0x12,
                        GattWriteKind::Command => 0x52,
                    },
                    request.l2cap_len,
                    request.payload,
                );
                if request.kind == GattWriteKind::Request {
                    GattWriteOutcome::Accepted
                } else {
                    GattWriteOutcome::NoWriteResponse
                }
            }
            LegacyBinding::Ota => {
                // Hand the ATT payload straight over: no 48-byte `Packet` is
                // built on the ATT path any more, and no OTA policy lives here.
                C::on_bulk_gatt_write(request.l2cap_len, request.payload);
                if request.kind == GattWriteKind::Request {
                    GattWriteOutcome::Accepted
                } else {
                    GattWriteOutcome::NoWriteResponse
                }
            }
            LegacyBinding::Pair => {
                pair_write_payload(request.payload);
                if request.kind == GattWriteKind::Request {
                    GattWriteOutcome::Accepted
                } else {
                    GattWriteOutcome::NoWriteResponse
                }
            }
        }
    }
}

/// Copies the configuration-driven GATT values into their SRAM buffers.
///
/// Strings are clamped to the buffer size and zero-padded, so the served length
/// and the buffer contents can never disagree. Each lock is held for exactly
/// one copy, so no interrupt-masking lock outlives this call.
pub fn init_gatt_profile(cfg: &RadioConfig) {
    let device_name = &cfg.device_name[..cfg.device_name.len().min(DEV_NAME_MAX as usize)];
    {
        let mut buf = DEV_NAME_BUF.lock();
        buf.fill(0);
        buf[..device_name.len()].copy_from_slice(device_name);
    }
    DEV_NAME_LEN.set(device_name.len() as u8);

    let mesh_name_bytes = cfg.default_mesh_name.as_bytes();
    let mesh_name = &mesh_name_bytes[..mesh_name_bytes.len().min(MANU_NAME_MAX as usize)];
    {
        let mut buf = MANU_NAME_BUF.lock();
        buf.fill(0);
        buf[..mesh_name.len()].copy_from_slice(mesh_name);
    }
    MANU_NAME_LEN.set(mesh_name.len() as u8);

    FW_REVISION_BUF
        .lock()
        .copy_from_slice(&cfg.build_version.to_le_bytes());
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;

    #[test]
    fn legacy_profile_has_29_table_entries_and_28_valid_handles() {
        assert_eq!(LEGACY_PROFILE_ATTRS.len(), 28);
        assert_eq!(LEGACY_PROFILE.total_handles(), 28);
        assert!(LEGACY_PROFILE.get(1).is_some());
        assert!(LEGACY_PROFILE.get(28).is_some());
        assert!(LEGACY_PROFILE.get(29).is_none());

        let source = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../tests/fixtures/gatt_compat.txt"
        ));
        assert!(source.contains("gatt.table.count = 29"));
        assert!(source.contains("gatt.table.max_valid_handle = 28"));
        assert!(source.contains("gatt.att.mtu = 23"));
        assert!(source.contains("gatt.att.read_by_type_max_value_len = 19"));
        assert!(source.contains("gatt.att.read_rsp_max_value_len = 22"));
    }

    #[test]
    fn primary_service_profile_values_match_legacy_fixture() {
        assert_eq!(
            LEGACY_PROFILE_ATTRS[0].role,
            AttrRole::PrimaryService { end_handle: 6 }
        );
        assert_eq!(
            LEGACY_PROFILE_ATTRS[6].role,
            AttrRole::PrimaryService { end_handle: 9 }
        );
        assert_eq!(
            LEGACY_PROFILE_ATTRS[15].role,
            AttrRole::PrimaryService { end_handle: 28 }
        );

        let mut out = [0u8; 22];
        let result = LegacyGattBackend::<crate::coex::TestRadioCoexistence>::read(
            LegacyBinding::Const(&GAP_SERVICE_VALUE),
            GattReadRequest {
                handle: 1,
                kind: GattReadKind::ReadByType,
                out: &mut out,
            },
        );
        assert_eq!(result, GattReadOutcome::Value(2));
        assert_eq!(&out[..2], &[0x00, 0x18]);
    }

    #[test]
    fn spp_command_read_consumes_only_for_read_not_read_by_type() {
        SEND_TO_MASTER.lock().copy_from_slice(&[0x11; 16]);
        let mut first = [0u8; 22];
        let outcome = LegacyGattBackend::<crate::coex::TestRadioCoexistence>::read(
            LegacyBinding::SppCommand,
            GattReadRequest {
                handle: 21,
                kind: GattReadKind::ReadByType,
                out: &mut first,
            },
        );
        assert_eq!(outcome, GattReadOutcome::Value(16));
        assert_eq!(first[0], 0x11);

        let mut second = [0u8; 22];
        let outcome = LegacyGattBackend::<crate::coex::TestRadioCoexistence>::read(
            LegacyBinding::SppCommand,
            GattReadRequest {
                handle: 21,
                kind: GattReadKind::Read,
                out: &mut second,
            },
        );
        assert_eq!(outcome, GattReadOutcome::Value(16));
        assert_eq!(*SEND_TO_MASTER.lock(), [0; 16]);
    }

    #[test]
    fn write_request_reaches_a_write_without_rsp_attribute() {
        // Regression (epic #111 audit, F-4/#115): the typed permission gate must
        // not narrow the deployed behaviour. The OTA characteristic (handle 24)
        // declares only write-without-response, yet the deployed firmware
        // executed a 0x12 Write Request to it and answered 0x13. Requiring the
        // `write` property acknowledged the packet and then dropped it, so a
        // write-with-response client silently lost the whole OTA stream.
        let perms_of = |index: usize| match LEGACY_PROFILE_ATTRS[index].role {
            AttrRole::CharacteristicValue { permissions } => permissions,
            other => panic!(
                "handle {} is not a characteristic value: {other:?}",
                index + 1
            ),
        };

        let ota = perms_of(23); // handle 24
        assert!(
            ota.write_without_rsp && !ota.write,
            "handle 24 must stay write-without-response only"
        );
        assert!(
            ota.allows(GattWriteKind::Request),
            "a 0x12 Write Request to the OTA characteristic must be executed, \
             not ACKed and dropped"
        );
        assert!(ota.allows(GattWriteKind::Command));

        // The status characteristic (handle 18) declares `write`, so it was
        // never affected; pin it so a future edit cannot regress it silently.
        let status = perms_of(17); // handle 18
        assert!(status.allows(GattWriteKind::Request));

        // The gate must still refuse a genuinely read-only attribute for both
        // kinds, which is the case it exists for.
        let read_only = AttrPermissions {
            read: true,
            write: false,
            write_without_rsp: false,
        };
        assert!(!read_only.allows(GattWriteKind::Request));
        assert!(!read_only.allows(GattWriteKind::Command));
    }

    #[test]
    fn status_cccd_write_and_pair_read_are_independent() {
        STATUS_CCC.lock().copy_from_slice(&[0x01, 0x00]);
        let outcome = LegacyGattBackend::<crate::coex::TestRadioCoexistence>::write(
            LegacyBinding::StatusCccd,
            GattWriteRequest {
                handle: 19,
                kind: GattWriteKind::Request,
                l2cap_len: 5,
                payload: &[0x02, 0x00],
            },
        );
        assert_eq!(outcome, GattWriteOutcome::Accepted);
        assert_eq!(*STATUS_CCC.lock(), [0x02, 0x00]);

        let mut out = [0u8; 22];
        let outcome = LegacyGattBackend::<crate::coex::TestRadioCoexistence>::read(
            LegacyBinding::Pair,
            GattReadRequest {
                handle: 27,
                kind: GattReadKind::ReadByType,
                out: &mut out,
            },
        );
        assert_eq!(outcome, GattReadOutcome::Value(16));
    }

    #[test]
    fn read_only_and_const_bindings_do_not_mutate() {
        let mut out = [0u8; 22];
        let outcome = LegacyGattBackend::<crate::coex::TestRadioCoexistence>::read(
            LegacyBinding::DeviceName,
            GattReadRequest {
                handle: 3,
                kind: GattReadKind::Read,
                out: &mut out,
            },
        );
        assert!(matches!(outcome, GattReadOutcome::Value(_)));

        let outcome = LegacyGattBackend::<crate::coex::TestRadioCoexistence>::write(
            LegacyBinding::DeviceName,
            GattWriteRequest {
                handle: 3,
                kind: GattWriteKind::Request,
                l2cap_len: 5,
                payload: &[0xaa],
            },
        );
        assert_eq!(outcome, GattWriteOutcome::Accepted);
    }
}
