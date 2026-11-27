use core::ptr::{addr_of, null, null_mut};
use core::sync::atomic::AtomicU8;

use crate::ble::pair::{pair_read, pair_write, PAIR_LOGIN_OK};
use crate::facade::RadioConfig;
use crate::mesh_ll::node_status::{mesh_report_status_enable, mesh_report_status_enable_mask};
use crate::rf::drv_8266::rf_link_slave_data_write;
use mesh_protocol::packet_types::Packet;
use mesh_protocol::service::{
    SERVICE_UUID_DEVICE_INFORMATION, TELINK_SPP_DATA_CLIENT2SERVER, TELINK_SPP_DATA_OTA,
    TELINK_SPP_DATA_PAIR, TELINK_SPP_DATA_SERVER2CLIENT, TELINK_SPP_UUID_SERVICE,
};
use tlsr8266_hal::common::sync::{CriticalSectionMutex, SimplifyLS};

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

static PRIMARY_SERVICE_UUID: u16 = GATT_UUID_PRIMARY_SERVICE;
static CHARACTER_UUID: u16 = GATT_UUID_CHARACTER;

static GAP_SERVICE_UUID: u16 = 0x1800;
static DEV_NAME_UUID: u16 = GATT_UUID_DEVICE_NAME;
static APPEARANCE_UIID: u16 = 0x2a01;

static DEV_INFO_UUID: u16 = SERVICE_UUID_DEVICE_INFORMATION;
static CLIENT_CHARACTER_CFG_UUID: u16 = GATT_UUID_CLIENT_CHAR_CFG;

static FW_REVISION_CHAR_UUID: u16 = CHARACTERISTIC_UUID_FW_REVISION_STRING;
static MANU_NAME_STRING_CHAR_UUID: u16 = CHARACTERISTIC_UUID_MANU_NAME_STRING;
static MODEL_ID_CHAR_UUID: u16 = CHARACTERISTIC_UUID_MODEL_NUM_STRING;
static HW_REVISION_CHAR_UUID: u16 = CHARACTERISTIC_UUID_HW_REVISION_STRING;

// Device Name Characteristic Properties
static DEV_NAME_CHARACTER: u8 = CHAR_PROP_READ;

// Appearance Characteristic Properties
static APPEARANCE_CHARACTER: u8 = CHAR_PROP_READ;
static APPEARANCE: u16 = GAP_APPEARE_UNKNOWN;

static FW_REVISION_CHAR: u8 = CHAR_PROP_READ;

static MANU_NAME_STRING_CHAR: u8 = CHAR_PROP_READ;

static MODEL_ID_CHAR: u8 = CHAR_PROP_READ;
static MODEL_ID_VALUE: &[u8] = b"model id 123";

static HW_REVISION_CHAR: u8 = CHAR_PROP_READ;
static HW_REVISION_VALUE: u32 = 0x22222222;

// ---- Config-driven GATT values --------------------------------------------
// The 29-entry table below stays immutable in `.rodata`, so the three values
// that `RadioConfig` drives live in these SRAM buffers instead of in the table
// itself. `init_gatt_profile` fills them and `attr_effective_len` is the only
// way the read paths learn how many bytes to serve.

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
static STATUS_CCC: [u8; 2] = [0x01, 0x00];

// Spp data for OTA characteristic variables
static SPP_DATA_PAIR_PROP: u8 = CHAR_PROP_READ | CHAR_PROP_WRITE;
static SPP_DATA_PAIR_DATA: [u8; 20] = [0xe0; 20];

/// Client → server SPP value. Owned here rather than aliased through a second
/// `static`, so the GATT table's `p_attr_value` pointer has one canonical name.
pub static SEND_TO_MASTER: CriticalSectionMutex<[u8; 16]> = CriticalSectionMutex::new([0; 16]);

static USERDESC_UUID: u16 = GATT_UUID_CHAR_USER_DESC;

static SPP_COMMANDNAME: &[u8] = b"Command";
static SPP_OTANAME: &[u8] = b"OTA";
static SPP_PAIRNAME: &[u8] = b"Pair";
static SPP_DEVICENAME: &[u8] = b"DevName";

#[export_name = "tlsr8266_mesh_status_write"]
fn mesh_status_write(p: &Packet) {
    if !PAIR_LOGIN_OK.get() {
        return;
    }

    // Access packet data safely through att_val
    let pktdata = &p.att_val().value;

    // Copy first 4 bytes to SPP_DATA_SERVER2CLIENT_DATA
    SPP_DATA_SERVER2CLIENT_DATA
        .lock()
        .copy_from_slice(&pktdata[0..4]);

    // Route based on packet length
    let l2cap_len = p.head().l2cap_len as usize;
    if l2cap_len > 4 {
        // Long packet: pass selective mesh addresses
        mesh_report_status_enable_mask(&pktdata[0..l2cap_len - 3]);
    } else {
        // Short packet: just enable/disable based on first byte
        mesh_report_status_enable(pktdata[0] != 0);
    }
}

pub struct AttributeT {
    pub att_num: u8,
    pub uuid_len: u8,
    pub attr_len: u8,
    pub attr_max_len: u8,
    pub uuid: *const u8,
    pub p_attr_value: *mut u8,
    pub w: Option<fn(data: &Packet)>,
    pub r: Option<fn(data: &Packet)>,
}

#[macro_export]
macro_rules! attrdef {
    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr, $uuid:expr, $pAttrValue:expr, $w:expr, $r: expr) => {
        AttributeT {
            att_num: $attNum as u8,
            uuid_len: $uuidLen as u8,
            attr_len: $attrLen as u8,
            attr_max_len: $attrMaxLen as u8,
            uuid: addr_of!($uuid) as *const u8,
            p_attr_value: $pAttrValue.as_ptr() as *mut u8,
            w: $w,
            r: $r,
        }
    };

    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr, $uuid:expr, $pAttrValue:expr) => {
        AttributeT {
            att_num: $attNum as u8,
            uuid_len: $uuidLen as u8,
            attr_len: $attrLen as u8,
            attr_max_len: $attrMaxLen as u8,
            uuid: addr_of!($uuid) as *const u8,
            p_attr_value: $pAttrValue.as_ptr() as *mut u8,
            w: None,
            r: None,
        }
    };

    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr) => {
        AttributeT {
            att_num: $attNum,
            uuid_len: $uuidLen,
            attr_len: $attrLen,
            attr_max_len: $attrMaxLen,
            uuid: null(),
            p_attr_value: null_mut(),
            w: None,
            r: None,
        }
    };
}

#[macro_export]
macro_rules! attrdefu {
    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr, $uuid:expr, $pAttrValue:expr, $w:expr, $r: expr) => {
        AttributeT {
            att_num: $attNum as u8,
            uuid_len: $uuidLen as u8,
            attr_len: $attrLen as u8,
            attr_max_len: $attrMaxLen as u8,
            uuid: addr_of!($uuid) as *const u8,
            p_attr_value: addr_of!($pAttrValue) as *mut u8,
            w: $w,
            r: $r,
        }
    };

    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr, $uuid:expr, $pAttrValue:expr) => {
        AttributeT {
            att_num: $attNum as u8,
            uuid_len: $uuidLen as u8,
            attr_len: $attrLen as u8,
            attr_max_len: $attrMaxLen as u8,
            uuid: addr_of!($uuid) as *const u8,
            p_attr_value: addr_of!($pAttrValue) as *mut u8,
            w: None,
            r: None,
        }
    };

    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr) => {
        AttributeT {
            att_num: $attNum,
            uuid_len: $uuidLen,
            attr_len: $attrLen,
            attr_max_len: $attrMaxLen,
            uuid: null(),
            p_attr_value: null_mut(),
            w: None,
            r: None,
        }
    };
}

#[coverage(off)]
const fn size_of_val<T>(_: &T) -> usize {
    core::mem::size_of::<T>()
}

/// Number of entries in the GATT attribute table. Entry 0 carries the last
/// valid handle in its `att_num`, so the table length is the single source of
/// truth for that value.
const GATT_ATT_COUNT: usize = 29;

#[repr(C, align(4))]
struct GAttributesType([AttributeT; GATT_ATT_COUNT]);

// SAFETY: (4) audited `Sync` wrapper — the table itself is never mutated in
// production. Each entry points at immutable statics, at the
// `SPP_DATA_SERVER2CLIENT_DATA` / `SEND_TO_MASTER` / `DEV_NAME_BUF` /
// `MANU_NAME_BUF` / `FW_REVISION_BUF` `CriticalSectionMutex` cells (written only
// through their locks, or by `init_gatt_profile` before the radio starts), or at
// the pair/OTA scratch arrays written only from the ATT handler. Under
// `cfg(test)` the table lives in a `TestGAttributes` cell and the host test
// harness is its only writer.
unsafe impl Sync for GAttributesType {}

/// Single definition of the table, shared by the production and test statics so
/// their contents cannot drift apart.
const GATT_TABLE: GAttributesType = GAttributesType([
    attrdef!((GATT_ATT_COUNT - 1) as u8, 0, 0, 0),
    // gatt
    attrdefu!(6, 2, 2, 2, PRIMARY_SERVICE_UUID, GAP_SERVICE_UUID),
    attrdefu!(0, 2, 1, 1, CHARACTER_UUID, DEV_NAME_CHARACTER),
    attrdef!(
        0,
        2,
        DEV_NAME_MAX,
        DEV_NAME_MAX,
        DEV_NAME_UUID,
        DEV_NAME_BUF
    ),
    attrdef!(
        0,
        2,
        SPP_DEVICENAME.len(),
        SPP_DEVICENAME.len(),
        USERDESC_UUID,
        SPP_DEVICENAME
    ),
    attrdefu!(0, 2, 1, 1, CHARACTER_UUID, APPEARANCE_CHARACTER),
    attrdefu!(
        0,
        2,
        size_of_val(&APPEARANCE),
        size_of_val(&APPEARANCE),
        APPEARANCE_UIID,
        APPEARANCE
    ),
    // device info
    attrdefu!(9, 2, 2, 2, PRIMARY_SERVICE_UUID, DEV_INFO_UUID),
    attrdefu!(0, 2, 1, 1, CHARACTER_UUID, FW_REVISION_CHAR),
    attrdef!(
        0,
        2,
        FW_REVISION_MAX,
        FW_REVISION_MAX,
        FW_REVISION_CHAR_UUID,
        FW_REVISION_BUF
    ),
    attrdefu!(0, 2, 1, 1, CHARACTER_UUID, MANU_NAME_STRING_CHAR),
    attrdef!(
        0,
        2,
        MANU_NAME_MAX,
        MANU_NAME_MAX,
        MANU_NAME_STRING_CHAR_UUID,
        MANU_NAME_BUF
    ),
    attrdefu!(0, 2, 1, 1, CHARACTER_UUID, MODEL_ID_CHAR),
    attrdef!(
        0,
        2,
        MODEL_ID_VALUE.len(),
        MODEL_ID_VALUE.len(),
        MODEL_ID_CHAR_UUID,
        MODEL_ID_VALUE
    ),
    attrdefu!(0, 2, 1, 1, CHARACTER_UUID, HW_REVISION_CHAR),
    attrdefu!(
        0,
        2,
        size_of_val(&HW_REVISION_VALUE),
        size_of_val(&HW_REVISION_VALUE),
        HW_REVISION_CHAR_UUID,
        HW_REVISION_VALUE
    ),
    // spp u//0x10
    attrdef!(13, 2, 16, 16, PRIMARY_SERVICE_UUID, TELINK_SPP_SERVICE_UUID),
    attrdefu!(0, 2, 1, 1, CHARACTER_UUID, SPP_DATA_SERVER2CLIENT_PROP), //prop
    attrdef!(
        0,
        16,
        1,
        1,
        TELINK_SPP_DATA_SERVER2CLIENT_UUID,
        SPP_DATA_SERVER2CLIENT_DATA,
        Some(mesh_status_write),
        None
    ), //value
    attrdef!(
        0,
        2,
        STATUS_CCC.len(),
        STATUS_CCC.len(),
        CLIENT_CHARACTER_CFG_UUID,
        STATUS_CCC
    ), /*value, CCC is must for some third-party APP if there is notify or indication*/
    attrdefu!(0, 2, 1, 1, CHARACTER_UUID, SPP_DATA_CLIENT2SERVER_PROP), //prop
    attrdef!(
        0,
        16,
        16,
        16,
        TELINK_SPP_DATA_CLIENT2SERVICE_UUID,
        SEND_TO_MASTER,
        Some(rf_link_slave_data_write),
        None
    ), //value
    attrdef!(
        0,
        2,
        SPP_COMMANDNAME.len(),
        SPP_COMMANDNAME.len(),
        USERDESC_UUID,
        SPP_COMMANDNAME
    ),
    attrdefu!(0, 2, 1, 1, CHARACTER_UUID, SPP_DATA_OTA_PROP), //prop
    attrdef!(
        0,
        16,
        16,
        16,
        TELINK_SPP_DATA_OTA_UUID,
        SPP_DATA_OTA_DATA,
        Some(crate::callbacks::dispatch_ota_write),
        None
    ), //value
    attrdef!(
        0,
        2,
        SPP_OTANAME.len(),
        SPP_OTANAME.len(),
        USERDESC_UUID,
        SPP_OTANAME
    ),
    attrdefu!(0, 2, 1, 1, CHARACTER_UUID, SPP_DATA_PAIR_PROP), //prop
    attrdef!(
        0,
        16,
        16,
        16,
        TELINK_SPP_DATA_PAIR_UUID,
        SPP_DATA_PAIR_DATA,
        Some(pair_write),
        Some(pair_read)
    ), //value
    attrdef!(
        0,
        2,
        SPP_PAIRNAME.len(),
        SPP_PAIRNAME.len(),
        USERDESC_UUID,
        SPP_PAIRNAME
    ),
]);

// The table is immutable on target and a `UnsafeCell` on host builds, so tests in
// this crate *and* in the firmware crate can patch an entry temporarily. The gate
// is `target_os`, not `test`: the firmware compiles this crate as a dependency and
// never carries its `cfg(test)`.
#[cfg(target_os = "none")]
static G_ATTRIBUTES: GAttributesType = GATT_TABLE;

#[cfg(not(target_os = "none"))]
struct TestGAttributes(core::cell::UnsafeCell<GAttributesType>);

#[cfg(not(target_os = "none"))]
// SAFETY: host tests are the only writers and they serialise through the
// single `&mut` handed out by `get_g_attributes()`; production has no access to
// this type at all.
unsafe impl Sync for TestGAttributes {}

#[cfg(not(target_os = "none"))]
static G_ATTRIBUTES: TestGAttributes = TestGAttributes(core::cell::UnsafeCell::new(GATT_TABLE));

/// The GATT attribute table.
#[cfg(target_os = "none")]
pub fn get_g_attributes() -> &'static [AttributeT; GATT_ATT_COUNT] {
    &G_ATTRIBUTES.0
}

/// The GATT attribute table, writable for tests that temporarily patch entries.
#[cfg(not(target_os = "none"))]
pub fn get_g_attributes() -> &'static mut [AttributeT; GATT_ATT_COUNT] {
    // SAFETY: the cell is a `static`, so the pointer is non-null and aligned;
    // the returned lifetime is unconstrained because tests own the table for the
    // whole process and each call re-borrows the same cell without keeping the
    // previous borrow alive.
    unsafe { &mut (*G_ATTRIBUTES.0.get()).0 }
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

/// Length of `attr`'s value as it must be served over ATT.
///
/// Handles 3 (device name) and 11 (manufacturer name) carry a runtime length, so
/// their `attr_len` is only the buffer maximum; every other entry serves its
/// fixed `attr_len`.
pub fn attr_effective_len(handle: usize, attr: &AttributeT) -> u8 {
    match handle {
        3 => DEV_NAME_LEN.get().min(DEV_NAME_MAX),
        11 => MANU_NAME_LEN.get().min(MANU_NAME_MAX),
        _ => attr.attr_len,
    }
}

/// True for the SRAM-backed GATT values, which are read-only on the wire.
///
/// They live above `__RAM_START_ADDR`, so the ATT write path's ROM check does not
/// reject them; it consults this instead.
pub fn is_readonly_ram_attr(handle: usize) -> bool {
    matches!(handle, 3 | 9 | 11)
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../../tests/fixtures/rust_parser.rs"
    ));
    use crate::mesh_ll::node_status::{
        MESH_NODE_MASK, MESH_NODE_MAX, MESH_NODE_REPORT_ENABLE, MESH_NODE_ST,
    };
    use mesh_protocol::packet_types::PacketL2capHead;

    /// Seeds the module-level reporting statics so the two report paths are
    /// distinguishable by their real effects: `mesh_report_status_enable(true)`
    /// marks every known node in bulk (bits `0..MESH_NODE_MAX`), whereas
    /// `mesh_report_status_enable_mask(..)` marks only the addresses it lists.
    ///
    /// `node_addr_1` is the address of the second known node. Leave both
    /// addresses at 0 on the bulk-enable path so no UART status dump is queued.
    fn seed_reporting_state(node_addr_0: u8, node_addr_1: u8) {
        MESH_NODE_MAX.set(2);
        MESH_NODE_REPORT_ENABLE.set(false);
        for word in MESH_NODE_MASK.lock().iter_mut() {
            *word = 0;
        }
        let mut node_st = MESH_NODE_ST.lock();
        node_st[0].val.dev_adr = node_addr_0;
        node_st[1].val.dev_adr = node_addr_1;
    }

    /// Restores the reporting statics every test above perturbs, so the next
    /// test in the process starts from the boot-time state.
    fn reset_reporting_state() {
        MESH_NODE_MAX.set(0);
        MESH_NODE_REPORT_ENABLE.set(false);
        for word in MESH_NODE_MASK.lock().iter_mut() {
            *word = 0;
        }
        let mut node_st = MESH_NODE_ST.lock();
        node_st[0].val.dev_adr = 0;
        node_st[1].val.dev_adr = 0;
    }

    /// First reporting-mask word, i.e. the bits for nodes 0..31.
    fn mask_word0() -> u32 {
        MESH_NODE_MASK.lock()[0]
    }

    #[test]
    fn test_mesh_status_write_not_logged_in() {
        // Arrange - User not logged in. The packet would enable reporting and
        // mark node 0 (address 0x02) if either reporting path ran.
        PAIR_LOGIN_OK.set(false);
        seed_reporting_state(0x02, 0x00);
        SPP_DATA_SERVER2CLIENT_DATA.lock().copy_from_slice(&[0; 4]);

        let mut pkt = Packet {
            head: PacketL2capHead {
                dma_len: 0,
                _type: 0,
                rf_len: 0,
                l2cap_len: 0,
                chan_id: 0,
            },
        };

        // Set up test data
        pkt.att_val_mut().value[0] = 0x01;
        pkt.att_val_mut().value[1] = 0x02;
        pkt.att_val_mut().value[2] = 0x03;
        pkt.att_val_mut().value[3] = 0x04;

        // Act
        mesh_status_write(&pkt);

        // Assert - Should return immediately without processing: no reporting
        // flag, no marked node, and no SPP mirror write.
        assert!(
            !MESH_NODE_REPORT_ENABLE.get(),
            "the enable flag must not be touched without a login"
        );
        assert_eq!(mask_word0(), 0, "no node may be marked as reporting");
        assert_eq!(
            *SPP_DATA_SERVER2CLIENT_DATA.lock(),
            [0; 4],
            "the SPP mirror must not be written without a login"
        );

        reset_reporting_state();
    }

    #[test]
    fn test_mesh_status_write_logged_in_short_packet() {
        // Arrange - User logged in, short packet (l2cap_len <= 4)
        PAIR_LOGIN_OK.set(true);
        seed_reporting_state(0x00, 0x00);

        let mut pkt = Packet {
            head: PacketL2capHead {
                dma_len: 0,
                _type: 0,
                rf_len: 0,
                l2cap_len: 4, // Short packet: 4 - 3 = 1 byte, not > 4
                chan_id: 0,
            },
        };

        // Set up test data - first byte is enable flag
        pkt.att_val_mut().value[0] = 0x01; // Non-zero = enable
        pkt.att_val_mut().value[1] = 0xAA;
        pkt.att_val_mut().value[2] = 0xBB;
        pkt.att_val_mut().value[3] = 0xCC;

        // Act
        mesh_status_write(&pkt);

        // Assert - the bulk-enable path ran (pktdata[0] != 0): reporting is on and
        // both known nodes are marked. The selective path would have marked none,
        // because the packet carries no address list.
        assert!(MESH_NODE_REPORT_ENABLE.get());
        assert_eq!(mask_word0(), 0b11, "all known nodes are enabled");

        reset_reporting_state();
    }

    #[test]
    fn test_mesh_status_write_logged_in_short_packet_disable() {
        // Arrange - User logged in, short packet with disable flag
        PAIR_LOGIN_OK.set(true);
        seed_reporting_state(0x00, 0x00);
        {
            let mut node_mask = MESH_NODE_MASK.lock();
            node_mask[0] = 0b11;
        }
        MESH_NODE_REPORT_ENABLE.set(true);

        let mut pkt = Packet {
            head: PacketL2capHead {
                dma_len: 0,
                _type: 0,
                rf_len: 0,
                l2cap_len: 4, // Short packet
                chan_id: 0,
            },
        };

        // Set up test data - first byte is disable flag
        pkt.att_val_mut().value[0] = 0x00; // Zero = disable
        pkt.att_val_mut().value[1] = 0x11;
        pkt.att_val_mut().value[2] = 0x22;
        pkt.att_val_mut().value[3] = 0x33;

        // Act
        mesh_status_write(&pkt);

        // Assert - pktdata[0] == 0 turns reporting off and leaves the node mask
        // untouched (disabling never rewrites it; enabling does).
        assert!(!MESH_NODE_REPORT_ENABLE.get());
        assert_eq!(mask_word0(), 0b11, "disabling leaves the node mask alone");

        reset_reporting_state();
    }

    #[test]
    fn test_mesh_status_write_logged_in_long_packet() {
        // Arrange - User logged in, long packet (l2cap_len > 4)
        // This tests that when l2cap_len > 4, the selective mask path runs.
        PAIR_LOGIN_OK.set(true);
        // Node 1 is address 0x05, which appears in the packet's address list.
        seed_reporting_state(0x10, 0x05);

        let mut pkt = Packet {
            head: PacketL2capHead {
                dma_len: 0,
                _type: 0,
                rf_len: 0,
                l2cap_len: 10, // Long packet: 10 - 3 = 7 bytes
                chan_id: 0,
            },
        };

        // Set up test data - multiple bytes for selective reporting
        pkt.att_val_mut().value[0] = 0x01;
        pkt.att_val_mut().value[1] = 0x02;
        pkt.att_val_mut().value[2] = 0x03;
        pkt.att_val_mut().value[3] = 0x04;
        pkt.att_val_mut().value[4] = 0x05;
        pkt.att_val_mut().value[5] = 0x06;
        pkt.att_val_mut().value[6] = 0x07;

        // Act
        mesh_status_write(&pkt);

        // Assert - the selective path ran over the first 7 bytes: only the listed
        // node (index 1) is marked. The bulk path would have set 0b11.
        assert!(MESH_NODE_REPORT_ENABLE.get());
        assert_eq!(mask_word0(), 0b10, "only the listed node is marked");
        // Verify the first 4 bytes were copied
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[0], 0x01);
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[1], 0x02);
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[2], 0x03);
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[3], 0x04);

        reset_reporting_state();
    }

    #[test]
    fn test_mesh_status_write_logged_in_maximum_long_packet() {
        // Arrange - User logged in, maximum size packet with selective addresses
        PAIR_LOGIN_OK.set(true);
        // Node 1 is address 0x0B, which appears in the packet's address list;
        // node 0 is deliberately outside it.
        seed_reporting_state(0x20, 0x0B);

        let mut pkt = Packet {
            head: PacketL2capHead {
                dma_len: 0,
                _type: 0,
                rf_len: 0,
                l2cap_len: 20, // Long packet: 20 - 3 = 17 bytes of selective data
                chan_id: 0,
            },
        };

        // Set up test data with multiple selective addresses
        let test_data = [
            0xFF, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,
            0x0E, 0x0F, 0x10,
        ];
        pkt.att_val_mut().value[0..17].copy_from_slice(&test_data);

        // Act
        mesh_status_write(&pkt);

        // Assert - the selective path ran over the first 17 bytes and marked only
        // the listed node.
        assert!(MESH_NODE_REPORT_ENABLE.get());
        assert_eq!(mask_word0(), 0b10, "only the listed node is marked");
        // Verify the first 4 bytes were copied
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[0], 0xFF);
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[1], 0x01);
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[2], 0x02);
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[3], 0x03);

        reset_reporting_state();
    }

    #[test]
    fn test_mesh_status_write_copies_first_four_bytes() {
        // Arrange - Verify that first 4 bytes are copied to SPP_DATA_SERVER2CLIENT_DATA
        PAIR_LOGIN_OK.set(true);
        seed_reporting_state(0x00, 0x00);

        let mut pkt = Packet {
            head: PacketL2capHead {
                dma_len: 0,
                _type: 0,
                rf_len: 0,
                l2cap_len: 4,
                chan_id: 0,
            },
        };

        // Set specific values to verify they are copied
        pkt.att_val_mut().value[0] = 0x11;
        pkt.att_val_mut().value[1] = 0x22;
        pkt.att_val_mut().value[2] = 0x33;
        pkt.att_val_mut().value[3] = 0x44;

        // Act
        mesh_status_write(&pkt);

        // Assert - 0x11 != 0 so the bulk-enable path ran, and every known node is
        // marked.
        assert!(MESH_NODE_REPORT_ENABLE.get());
        assert_eq!(mask_word0(), 0b11, "all known nodes are enabled");
        // Verify the first 4 bytes were copied to SPP_DATA_SERVER2CLIENT_DATA
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[0], 0x11);
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[1], 0x22);
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[2], 0x33);
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[3], 0x44);

        reset_reporting_state();
    }

    #[test]
    fn test_mesh_status_write_boundary_condition() {
        // Arrange - Test boundary case where l2cap_len = 5 (just over threshold of 4)
        PAIR_LOGIN_OK.set(true);
        // Node 1 is address 0xBB, the single selective address in the packet.
        seed_reporting_state(0x10, 0xBB);

        let mut pkt = Packet {
            head: PacketL2capHead {
                dma_len: 0,
                _type: 0,
                rf_len: 0,
                l2cap_len: 5, // 5 - 3 = 2, which is > 0, triggers mask path
                chan_id: 0,
            },
        };

        pkt.att_val_mut().value[0] = 0xAA;
        pkt.att_val_mut().value[1] = 0xBB;
        pkt.att_val_mut().value[2] = 0xCC;
        pkt.att_val_mut().value[3] = 0xDD;

        // Act
        mesh_status_write(&pkt);

        // Assert - the selective path ran over the first 2 bytes (l2cap_len - 3)
        // and marked only the listed node. The bulk path would have set 0b11.
        assert!(MESH_NODE_REPORT_ENABLE.get());
        assert_eq!(mask_word0(), 0b10, "only the listed node is marked");
        // Verify the first 4 bytes were copied
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[0], 0xAA);
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[1], 0xBB);
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[2], 0xCC);
        assert_eq!(SPP_DATA_SERVER2CLIENT_DATA.lock()[3], 0xDD);

        reset_reporting_state();
    }

    // OTA write seam tests. The `OtaHooks` slot is process-wide, so these tests
    // must run with `--test-threads=1` and each one restores the host-test
    // default on exit.
    use core::sync::atomic::{AtomicUsize, Ordering};

    use crate::callbacks::{dispatch_ota_write, set_ota_hooks, OtaHooks, TEST_OTA_HOOKS};

    static OTA_WRITE_HITS: AtomicUsize = AtomicUsize::new(0);
    static OTA_WRITE_L2CAP_LEN: AtomicUsize = AtomicUsize::new(0);
    static OTA_WRITE_FIRST_WORD: AtomicUsize = AtomicUsize::new(0);
    static OTA_SNAPSHOT_HITS: AtomicUsize = AtomicUsize::new(0);

    /// Restores the host-test default hooks and clears the observation counters.
    fn reset_ota_write_slot() {
        set_ota_hooks(TEST_OTA_HOOKS);
        OTA_WRITE_HITS.store(0, Ordering::SeqCst);
        OTA_WRITE_L2CAP_LEN.store(0, Ordering::SeqCst);
        OTA_WRITE_FIRST_WORD.store(0, Ordering::SeqCst);
        OTA_SNAPSHOT_HITS.store(0, Ordering::SeqCst);
    }

    /// Test handler: records how often it ran and what it received.
    fn recording_ota_handler(p: &Packet) {
        OTA_WRITE_HITS.fetch_add(1, Ordering::SeqCst);
        OTA_WRITE_L2CAP_LEN.store(p.head().l2cap_len as usize, Ordering::SeqCst);
        let value = &p.att_val().value;
        OTA_WRITE_FIRST_WORD.store(
            u32::from_le_bytes([value[0], value[1], value[2], value[3]]) as usize,
            Ordering::SeqCst,
        );
    }

    /// Recorder for a neighbouring slot, so a swapped registration is visible.
    fn recording_ota_snapshot() -> mesh_protocol::mesh_types::OtaStatusSnapshot {
        OTA_SNAPSHOT_HITS.fetch_add(1, Ordering::SeqCst);
        mesh_protocol::mesh_types::OtaStatusSnapshot {
            in_progress: false,
            finish_status: mesh_protocol::mesh_types::OtaState::Continue,
            timeout_expired: false,
        }
    }

    /// The OTA characteristic's write callback must route the packet through the
    /// `OtaHooks` slot, with the packet contents intact.
    #[test]
    fn test_ota_characteristic_write_dispatches_to_registered_hook() {
        reset_ota_write_slot();

        let mut pkt = Packet {
            head: PacketL2capHead {
                dma_len: 0,
                _type: 0x1b,
                rf_len: 0x18,
                l2cap_len: 0x0014,
                chan_id: 0x0004,
            },
        };
        pkt.att_val_mut().value[0..4].copy_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);

        set_ota_hooks(OtaHooks {
            on_gatt_ota_write: recording_ota_handler,
            status_snapshot: recording_ota_snapshot,
            ..TEST_OTA_HOOKS
        });

        // Act: go through the same entry point the attribute table holds.
        dispatch_ota_write(&pkt);

        // Assert: exactly one delivery with the packet contents intact
        assert_eq!(OTA_WRITE_HITS.load(Ordering::SeqCst), 1);
        assert_eq!(OTA_WRITE_L2CAP_LEN.load(Ordering::SeqCst), 0x0014);
        assert_eq!(
            OTA_WRITE_FIRST_WORD.load(Ordering::SeqCst),
            u32::from_le_bytes([0xDE, 0xAD, 0xBE, 0xEF]) as usize
        );
        // Only the write slot fired.
        assert_eq!(OTA_SNAPSHOT_HITS.load(Ordering::SeqCst), 0);

        reset_ota_write_slot();
    }

    /// The host-test default must be safe to call without any registration
    /// setup: it reaches the real OTA subsystem instead of panicking.
    #[test]
    fn test_ota_write_test_default_is_safe() {
        reset_ota_write_slot();
        crate::callbacks::test_ota::reset_for_test();

        let mut pkt = Packet {
            head: PacketL2capHead {
                dma_len: 0,
                _type: 0x1b,
                rf_len: 0x18,
                l2cap_len: 0x0014,
                chan_id: 0x0004,
            },
        };
        pkt.att_val_mut().value[0..4].copy_from_slice(&[0x01, 0x02, 0x03, 0x04]);

        // Act - the default hooks are installed, so this must not panic.
        dispatch_ota_write(&pkt);

        // The mesh-blocked/finish-flag gates keep the real handler inert here.
        assert!(
            !crate::callbacks::test_ota::snapshot().in_progress,
            "a packet on an idle OTA must not claim the transfer"
        );

        crate::callbacks::test_ota::reset_for_test();
        reset_ota_write_slot();
    }

    // ---- Config-driven GATT values -----------------------------------------

    use crate::config::DEFAULT;
    use crate::facade::{radio_config, set_radio_config};
    use crate::gatt::attribute::{l2cap_att_handler, GattOp};
    use core::ptr::addr_of_mut;
    use mesh_protocol::packet_types::PacketL2capData;

    /// Restore the factory profile. The suite runs with `--test-threads=1`, and
    /// every test below leaves the process-wide configuration as it found it.
    fn restore_default_profile() {
        set_radio_config(DEFAULT);
        init_gatt_profile(&DEFAULT);
    }

    /// The three SRAM-backed GATT values, for byte-for-byte comparison.
    fn sram_values() -> ([u8; 19], [u8; 4], [u8; 16]) {
        (
            *DEV_NAME_BUF.lock(),
            *FW_REVISION_BUF.lock(),
            *MANU_NAME_BUF.lock(),
        )
    }

    /// Build an ATT write packet: `value[3]` is the opcode, `value[4..6]` the
    /// little-endian handle and `value[6..]` the value, matching the layout
    /// `l2cap_att_handler` reads out of the packed union. `handle1` is the byte
    /// the write path reads as its `>= 3` "handle" gate.
    fn write_packet(opcode: u8, handle: u16, payload: &[u8]) -> Packet {
        let mut packet = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: payload.len() as u16 + 3,
                chan_id: 4,
                opcode: 0,
                handle: 4,
                handle1: handle as u8,
                value: [0; 30],
            },
        };
        packet.l2cap_data_mut().value[1] = 4;
        packet.l2cap_data_mut().value[3] = opcode;
        packet.l2cap_data_mut().value[4] = handle as u8;
        packet.l2cap_data_mut().value[5] = (handle >> 8) as u8;
        packet.l2cap_data_mut().value[6..6 + payload.len()].copy_from_slice(payload);
        packet
    }

    /// Handle 21 remains bound to the shallow callback for paired Write Request
    /// and Write Command. The Write Response byte is preserved for the Request;
    /// the Command remains silent and the queued Packet is unchanged.
    #[test]
    fn test_handle21_paired_write_request_and_command_queue_only() {
        restore_default_profile();
        crate::queues::reset_for_test();
        PAIR_LOGIN_OK.set(true);

        assert!(
            get_g_attributes()[21].w.is_some(),
            "handle 21 must keep a write callback"
        );

        let request = l2cap_att_handler(&write_packet(
            GattOp::AttOpWriteReq as u8,
            21,
            &[0x10, 0x11, 0x12],
        ))
        .expect("a paired Write Request keeps its 0x13 response");
        assert_eq!(request.att_write_rsp().opcode, GattOp::AttOpWriteRsp as u8);
        assert_eq!(crate::queues::MESH_RX_CHANNEL.len(), 1);

        let queued = crate::queues::MESH_RX_CHANNEL
            .try_receive()
            .expect("paired Write Request must enqueue one Packet");
        let queued_chan_id = queued.head().chan_id;
        let queued_opcode = queued.att_write().opcode;
        let queued_handle = queued.att_write().handle;
        let queued_handle1 = queued.att_write().handle1;
        assert_eq!(queued_chan_id, 4);
        assert_eq!(queued_opcode, GattOp::AttOpWriteReq as u8);
        assert_eq!(queued_handle, 21);
        assert_eq!(queued_handle1, 0);

        let command = l2cap_att_handler(&write_packet(
            GattOp::AttOpWriteCmd as u8,
            21,
            &[0x20, 0x21, 0x22],
        ));
        assert!(command.is_none(), "a paired Write Command remains silent");
        assert_eq!(crate::queues::MESH_RX_CHANNEL.len(), 1);

        let queued = crate::queues::MESH_RX_CHANNEL
            .try_receive()
            .expect("paired Write Command must enqueue one Packet");
        let queued_chan_id = queued.head().chan_id;
        let queued_opcode = queued.att_write().opcode;
        let queued_handle = queued.att_write().handle;
        let queued_handle1 = queued.att_write().handle1;
        assert_eq!(queued_chan_id, 4);
        assert_eq!(queued_opcode, GattOp::AttOpWriteCmd as u8);
        assert_eq!(queued_handle, 21);
        assert_eq!(queued_handle1, 0);

        PAIR_LOGIN_OK.set(false);
        crate::queues::reset_for_test();
    }

    /// Unpaired handle-21 writes stay inert: the ATT response contract is not
    /// changed, but neither Write Request nor Write Command reaches the RX queue.
    #[test]
    fn test_handle21_unpaired_writes_remain_inert() {
        restore_default_profile();
        crate::queues::reset_for_test();
        PAIR_LOGIN_OK.set(false);

        let request = l2cap_att_handler(&write_packet(
            GattOp::AttOpWriteReq as u8,
            21,
            &[0x01, 0x02, 0x03],
        ))
        .expect("unpaired Write Request still receives the legacy 0x13 response");
        assert_eq!(request.att_write_rsp().opcode, GattOp::AttOpWriteRsp as u8);

        let command = l2cap_att_handler(&write_packet(
            GattOp::AttOpWriteCmd as u8,
            21,
            &[0x04, 0x05, 0x06],
        ));
        assert!(command.is_none(), "unpaired Write Command remains silent");
        assert_eq!(crate::queues::MESH_RX_CHANNEL.len(), 0);
        assert_eq!(crate::queues::dropped_mesh_rx(), 0);

        crate::queues::reset_for_test();
    }

    /// A 19-byte name, a distinct mesh name, a distinct build version and a
    /// shorter name: every config-driven GATT value follows `RadioConfig`.
    #[test]
    fn test_init_gatt_profile_serves_injected_config() {
        const NAME_19: &[u8] = b"0123456789ABCDEFGHI";
        const NAME_20: &[u8] = b"0123456789ABCDEFGHIJ";

        let mut cfg = radio_config();
        cfg.device_name = NAME_19;
        cfg.default_mesh_name = "mesh-42";
        cfg.build_version = 0xA1B2_C3D4;
        set_radio_config(cfg);
        init_gatt_profile(&cfg);

        let device = *DEV_NAME_BUF.lock();
        let manu = *MANU_NAME_BUF.lock();
        assert_eq!(DEV_NAME_LEN.get(), 19, "a 19-byte name is kept whole");
        assert_eq!(&device[..], NAME_19);
        assert_eq!(MANU_NAME_LEN.get(), 7);
        assert_eq!(&manu[..7], b"mesh-42");
        assert_eq!(&manu[7..], &[0u8; 9][..], "the tail is zeroed");
        assert_eq!(*FW_REVISION_BUF.lock(), 0xA1B2_C3D4u32.to_le_bytes());

        assert_eq!(attr_effective_len(3, &get_g_attributes()[3]), 19);
        assert_eq!(
            attr_effective_len(9, &get_g_attributes()[9]),
            4,
            "the version word is fixed"
        );
        assert_eq!(attr_effective_len(11, &get_g_attributes()[11]), 7);
        assert_eq!(
            attr_effective_len(4, &get_g_attributes()[4]),
            7,
            "other handles keep attr_len"
        );

        // A shorter name is zero-padded and served at its own length.
        let mut short = cfg;
        short.device_name = b"ab";
        short.default_mesh_name = "m";
        init_gatt_profile(&short);
        let device = *DEV_NAME_BUF.lock();
        assert_eq!(DEV_NAME_LEN.get(), 2);
        assert_eq!(&device[..2], b"ab");
        assert_eq!(&device[2..], &[0u8; 17][..]);
        assert_eq!(attr_effective_len(3, &get_g_attributes()[3]), 2);
        assert_eq!(attr_effective_len(11, &get_g_attributes()[11]), 1);

        // An over-long name is clamped to the buffer size.
        let mut long = cfg;
        long.device_name = NAME_20;
        init_gatt_profile(&long);
        assert_eq!(DEV_NAME_LEN.get(), 19);
        assert_eq!(&DEV_NAME_BUF.lock()[..], NAME_19);

        restore_default_profile();
        assert_eq!(DEV_NAME_LEN.get(), 13);
        assert_eq!(MANU_NAME_LEN.get(), 11);
        assert_eq!(*FW_REVISION_BUF.lock(), [0; 4]);
        assert_eq!(&DEV_NAME_BUF.lock()[..13], b"Telink tLight");
        assert_eq!(attr_effective_len(3, &get_g_attributes()[3]), 13);
        assert_eq!(attr_effective_len(11, &get_g_attributes()[11]), 11);
    }

    /// `AttOpWriteReq` and `AttOpWriteCmd` to handles 3, 9 and 11 must not change
    /// the config-driven values.
    #[test]
    fn test_write_requests_cannot_mutate_config_driven_gatt_values() {
        restore_default_profile();
        let before = sram_values();
        let payload = [0xEE; 16];

        for handle in [3u16, 9, 11] {
            for opcode in [GattOp::AttOpWriteReq as u8, GattOp::AttOpWriteCmd as u8] {
                let response = l2cap_att_handler(&write_packet(opcode, handle, &payload));
                assert!(
                    response.is_none(),
                    "handle {handle} rejects writes: its declaration is read-only"
                );
                assert_eq!(
                    sram_values(),
                    before,
                    "handle {handle} must keep its config-driven value"
                );
            }
        }
    }

    /// With the declaration's write bits patched on, `is_readonly_ram_attr` is
    /// the only gate left rejecting the write, so this reaches that branch.
    #[test]
    fn test_readonly_ram_guard_rejects_write_when_permission_is_patched() {
        restore_default_profile();
        let before = sram_values();
        let payload = [0xEE; 16];
        // Characteristic declarations sit at `handle - 1`; give them write bits.
        let mut writable_props: u8 = CHAR_PROP_READ | CHAR_PROP_WRITE;

        for handle in [3usize, 9, 11] {
            let declaration = handle - 1;
            let original_props = get_g_attributes()[declaration].p_attr_value;
            // SAFETY: (4) test-only patch of the table's own declaration entry;
            // the original pointer is restored before `writable_props` (a local)
            // goes out of scope.
            unsafe {
                (*(addr_of_mut!(get_g_attributes()[declaration]))).p_attr_value =
                    addr_of_mut!(writable_props);
            }

            let write_req = l2cap_att_handler(&write_packet(
                GattOp::AttOpWriteReq as u8,
                handle as u16,
                &payload,
            ));
            let write_cmd = l2cap_att_handler(&write_packet(
                GattOp::AttOpWriteCmd as u8,
                handle as u16,
                &payload,
            ));

            // SAFETY: (4) restore the declaration value pointer patched above.
            unsafe {
                (*(addr_of_mut!(get_g_attributes()[declaration]))).p_attr_value = original_props;
            }

            assert_eq!(
                write_req.map(|p| p.att_write_rsp().opcode),
                Some(GattOp::AttOpWriteRsp as u8),
                "handle {handle} gets a write response, so the RAM guard is what rejected it"
            );
            assert!(write_cmd.is_none(), "a write command has no response");
            assert_eq!(
                sram_values(),
                before,
                "handle {handle} must keep its config-driven value"
            );
        }
    }

    /// `STATUS_CCC` (handle 19) must stay writable: subscriptions depend on it.
    ///
    /// The host stand-in for `__RAM_START_ADDR` is `0`, so every host static —
    /// `STATUS_CCC` included — sits *above* it and the ROM check lets the copy
    /// proceed. The entry is still pointed at a local value buffer so the write
    /// lands in test-owned memory; that exercises the same handler decision
    /// (handle 19 is not in `is_readonly_ram_attr`, so the copy proceeds)
    /// without depending on the host's static layout.
    #[test]
    fn test_status_ccc_remains_writable() {
        let original_ptr = get_g_attributes()[19].p_attr_value;
        let mut ccc = STATUS_CCC;
        // SAFETY: (4) test-only patch of the table's own CCC entry; the pointer is
        // restored before the local buffer `ccc` goes out of scope.
        unsafe {
            (*(addr_of_mut!(get_g_attributes()[19]))).p_attr_value = addr_of_mut!(ccc) as *mut u8;
        }

        let response = l2cap_att_handler(&write_packet(
            GattOp::AttOpWriteReq as u8,
            19,
            &[0x02, 0x00],
        ));

        // SAFETY: (4) restore the CCC value pointer patched above.
        unsafe {
            (*(addr_of_mut!(get_g_attributes()[19]))).p_attr_value = original_ptr;
        }

        assert_eq!(
            response.map(|p| p.att_write_rsp().opcode),
            Some(GattOp::AttOpWriteRsp as u8)
        );
        assert_eq!(ccc, [0x02, 0x00], "handle 19 must accept the write");
        assert!(
            !is_readonly_ram_attr(19),
            "the CCC handle must not be treated as read-only RAM"
        );
        assert_eq!(STATUS_CCC, [0x01, 0x00], "the CCC static is left alone");
    }

    #[test]
    fn test_gatt_fixture_29_rows_and_config_invariants() {
        let mut fixture = parse_fixture(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../tests/fixtures/gatt_compat.txt"
        )))
        .expect("gatt fixture must parse");

        assert_eq!(
            fixture.u8("gatt.table.count").unwrap(),
            GATT_ATT_COUNT as u8
        );
        assert_eq!(fixture.u8("gatt.table.max_valid_handle").unwrap(), 28);
        assert_eq!(fixture.u8("gatt.att.mtu").unwrap(), 23);
        assert_eq!(
            fixture.u8("gatt.att.read_by_type_max_value_len").unwrap(),
            19
        );
        assert_eq!(fixture.u8("gatt.att.read_rsp_max_value_len").unwrap(), 22);

        restore_default_profile();
        // Reset test-owned scratch values used by fixture rows so the oracle
        // observes the boot/default bytes regardless of test execution order.
        SPP_DATA_SERVER2CLIENT_DATA.lock().copy_from_slice(&[0; 4]);
        SEND_TO_MASTER.lock().fill(0);

        // Re-seed the host-test table from the canonical GATT_TABLE. Previous
        // tests intentionally patch entries temporarily; this oracle must never
        // inherit any stale or dangling temporary pointer.
        let table = get_g_attributes();
        for (handle, attr) in table.iter_mut().enumerate().take(GATT_ATT_COUNT) {
            let canonical = &GATT_TABLE.0[handle];
            attr.att_num = canonical.att_num;
            attr.uuid_len = canonical.uuid_len;
            attr.attr_len = canonical.attr_len;
            attr.attr_max_len = canonical.attr_max_len;
            attr.uuid = canonical.uuid;
            attr.p_attr_value = canonical.p_attr_value;
            attr.w = canonical.w;
            attr.r = canonical.r;
        }

        for (handle, attr) in table.iter().enumerate().take(GATT_ATT_COUNT) {
            let row = format!("gatt.row.{handle:02}");
            let span = fixture.u8(&format!("{row}.span")).unwrap();
            let uuid_len = fixture.u8(&format!("{row}.uuid_len")).unwrap();
            let uuid = fixture.bytes(&format!("{row}.uuid")).unwrap();
            let default_len = fixture.u8(&format!("{row}.default_len")).unwrap();
            let max_len = fixture.u8(&format!("{row}.max_len")).unwrap();
            let behavior = fixture.str(&format!("{row}.behavior")).unwrap().to_owned();

            assert_eq!(attr.att_num, span, "row {handle} span");
            assert_eq!(attr.uuid_len, uuid_len, "row {handle} uuid_len");
            assert_eq!(attr.attr_max_len, max_len, "row {handle} max_len");

            if uuid_len == 0 {
                assert!(attr.uuid.is_null(), "sentinel UUID pointer");
            } else {
                assert!(
                    !attr.uuid.is_null(),
                    "row {handle} UUID pointer must be present"
                );

                match uuid_len {
                    2 => {
                        // SAFETY: the table statics own two UUID bytes and
                        // `read_unaligned` is valid even if the 16-bit value is
                        // stored in a deliberately untrusted test pointer.
                        let actual = unsafe { core::ptr::read_unaligned(attr.uuid.cast::<u16>()) };
                        assert_eq!(
                            actual.to_le_bytes(),
                            uuid.as_slice(),
                            "row {handle} 16-bit UUID bytes"
                        );
                    }
                    16 => {
                        assert_eq!(
                            attr.uuid.align_offset(core::mem::align_of::<u8>()),
                            0,
                            "row {handle} 128-bit UUID pointer must be aligned for u8"
                        );
                        // SAFETY: uuid_len == 16 is fixture-pinned, the table
                        // entry owns a public 16-byte UUID static, and the
                        // pointer is non-null/aligned for u8 before the slice is formed.
                        let actual =
                            unsafe { core::slice::from_raw_parts(attr.uuid, uuid_len as usize) };
                        assert_eq!(actual, uuid.as_slice(), "row {handle} 128-bit UUID bytes");
                    }
                    other => panic!("unexpected row {handle} uuid_len {other}"),
                }
            }

            assert_eq!(
                attr_effective_len(handle, attr),
                if handle == 3 || handle == 11 || handle == 9 {
                    default_len
                } else {
                    attr.attr_len
                },
                "row {handle} effective/default length"
            );

            match behavior.as_str() {
                "sentinel" => assert_eq!(handle, 0),
                "ro_static" | "ro_dynamic_config" | "ro_dynamic_build_version" => {
                    assert!(attr.w.is_none(), "row {handle} must remain read-only")
                }
                "status_report_write" => assert!(attr.w.is_some(), "row {handle} write callback"),
                "cccd_writable" => assert_eq!(attr.attr_len, 2),
                "command_bridge_write_consume_on_read" => {
                    assert!(attr.w.is_some(), "row {handle} write callback");
                    assert!(attr.r.is_none(), "read consumes SEND_TO_MASTER");
                }
                "ota_write_terminate_on_completed_read" => {
                    assert!(attr.w.is_some(), "row {handle} write callback");
                    assert!(
                        attr.r.is_none(),
                        "OTA read side effect is handled by the ATT read path"
                    );
                }
                "pair_write_deferred_read" => {
                    assert!(attr.w.is_some(), "row {handle} write callback");
                    assert!(attr.r.is_some(), "row {handle} pair read callback");
                }
                other => panic!("unexpected behavior class {other}"),
            }

            let value_key = format!("{row}.value");
            let value_text = fixture.str(&value_key).unwrap();
            if value_text == "none" {
                assert_eq!(handle, 0);
            } else if value_text == "build_version_u32_le" {
                assert_eq!(handle, 9);
                assert_eq!(default_len, 4);
                assert_eq!(
                    *FW_REVISION_BUF.lock(),
                    radio_config().build_version.to_le_bytes(),
                    "handle 9 must serve the live build-version LE bytes"
                );
            } else {
                let expected = fixture.bytes(&value_key).unwrap();
                if handle == 3 {
                    assert_eq!(DEV_NAME_LEN.get(), 13);
                    assert_eq!(&DEV_NAME_BUF.lock()[..13], expected.as_slice());
                } else if handle == 11 {
                    assert_eq!(MANU_NAME_LEN.get(), 11);
                    assert_eq!(&MANU_NAME_BUF.lock()[..11], expected.as_slice());
                } else if !attr.p_attr_value.is_null() {
                    assert_eq!(
                        expected.len(),
                        default_len as usize,
                        "row {handle} fixture value length"
                    );
                    assert_eq!(
                        attr.p_attr_value.align_offset(core::mem::align_of::<u8>()),
                        0,
                        "row {handle} value pointer must be aligned for u8"
                    );
                    // SAFETY: default_len is fixture-pinned and every non-dynamic
                    // row here points at immutable/static or test-reset scratch
                    // storage with at least default_len readable bytes.
                    let actual = unsafe {
                        core::slice::from_raw_parts(attr.p_attr_value, default_len as usize)
                    };
                    assert_eq!(
                        actual,
                        expected.as_slice(),
                        "row {handle} static value bytes"
                    );
                } else {
                    panic!("row {handle} has fixture value bytes but no value pointer");
                }
            }
        }

        let mut cfg = radio_config();
        cfg.build_version = 0x1234_5678;
        set_radio_config(cfg);
        init_gatt_profile(&cfg);
        assert_eq!(*FW_REVISION_BUF.lock(), 0x1234_5678u32.to_le_bytes());
        restore_default_profile();

        let readonly = fixture.bytes("att.readonly_handles").unwrap();
        assert_eq!(readonly, vec![0x03, 0x09, 0x11]);
        assert_eq!(fixture.u8("att.write_gate_max_exclusive").unwrap(), 3);
    }
}
