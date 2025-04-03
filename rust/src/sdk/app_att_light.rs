use core::cell::UnsafeCell;
use core::ptr::{addr_of, null, null_mut, slice_from_raw_parts_mut};
use core::slice;

use crate::config::{DEVICE_NAME, MESH_NAME};
use crate::ota::rf_link_slave_data_ota;
use crate::pub_mut;
use crate::sdk::ble_app::ble_ll_pair::{pair_read, pair_write};
use crate::sdk::ble_app::light_ll::{mesh_report_status_enable, mesh_report_status_enable_mask};
use crate::sdk::ble_app::rf_drv_8266::rf_link_slave_data_write;
use crate::sdk::light::*;
use crate::sdk::packet_types::{Packet, PacketAttValue};
use crate::sdk::service::{
    SERVICE_UUID_DEVICE_INFORMATION, TELINK_SPP_DATA_CLIENT2SERVER, TELINK_SPP_DATA_OTA,
    TELINK_SPP_DATA_PAIR, TELINK_SPP_DATA_SERVER2CLIENT, TELINK_SPP_UUID_SERVICE,
};
use crate::state::{*};
use crate::version::BUILD_VERSION;

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

pub const GATT_UUID_PRIMARY_SERVICE: u16 = 0x2800;      // Primary Service
pub const GATT_UUID_SECONDARY_SERVICE: u16 = 0x2801;    // Secondary Service
pub const GATT_UUID_INCLUDE: u16 = 0x2802;              // Include
pub const GATT_UUID_CHARACTER: u16 = 0x2803;            // Characteristic
pub const GATT_UUID_CHAR_EXT_PROPS: u16 = 0x2900;       // Characteristic Extended Properties
pub const GATT_UUID_CHAR_USER_DESC: u16 = 0x2901;       // Characteristic User Description
pub const GATT_UUID_CLIENT_CHAR_CFG: u16 = 0x2902;      // Client Characteristic Configuration
pub const GATT_UUID_SERVER_CHAR_CFG: u16 = 0x2903;      // Server Characteristic Configuration
pub const GATT_UUID_CHAR_PRESENT_FORMAT: u16 = 0x2904;  // Characteristic Present Format
pub const GATT_UUID_CHAR_AGG_FORMAT: u16 = 0x2905;      // Characteristic Aggregate Format
pub const GATT_UUID_VALID_RANGE: u16 = 0x2906;          // Valid Range
pub const GATT_UUID_EXT_REPORT_REF: u16 = 0x2907;       // External Report Reference
pub const GATT_UUID_REPORT_REF: u16 = 0x2908;           // Report Reference

pub const GATT_UUID_DEVICE_NAME: u16 = 0x2a00;          // Report Reference

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
static PERI_CONN_PARAM_UUID: u16 = 0x2a04;

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

// Peripheral Preferred Connection Parameters Characteristic Properties
static PERI_CONN_PARAM_CHAR: u8 = CHAR_PROP_READ;

static FW_REVISION_CHAR: u8 = CHAR_PROP_READ;
static FW_REVISION_VALUE: [u8; 4] = [
    (BUILD_VERSION & 0xff) as u8,
    ((BUILD_VERSION >> 8) & 0xff) as u8,
    ((BUILD_VERSION >> 16) & 0xff) as u8,
    ((BUILD_VERSION >> 24) & 0xff) as u8
];

static MANU_NAME_STRING_CHAR: u8 = CHAR_PROP_READ;
static MANU_NAME_STRING_VALUE: &[u8] = MESH_NAME.as_bytes();

static MODEL_ID_CHAR: u8 = CHAR_PROP_READ;
static MODEL_ID_VALUE: &[u8] = b"model id 123";

static HW_REVISION_CHAR: u8 = CHAR_PROP_READ;
static HW_REVISION_VALUE: u32 = 0x22222222;

#[repr(C, align(4))]
struct GapPeriConnectParamsT {
    /** Minimum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
    pub interval_min: u16,
    /** Maximum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
    pub interval_max: u16,
    /** Number of LL latency connection events (0x0000 - 0x03e8) */
    pub latency: u16,
    /** Connection Timeout (0x000A - 0x0C80 * 10 ms) */
    pub timeout: u16,
}

const PERI_CONN_PARAMETERS: GapPeriConnectParamsT = GapPeriConnectParamsT {
    interval_min: 20,
    interval_max: 40,
    latency: 0,
    timeout: 1000,
};

// note !!!DEVICE_NAME max 13 bytes
static BLE_G_DEV_NAME: &'static [u8] = DEVICE_NAME;

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

static mut SPP_DATA_SERVER2CLIENT_DATA: [u8; 4] = [0; 4];
static STATUS_CCC: [u8; 2] = [0x01, 0x00];

// Spp data for OTA characteristic variables
static SPP_DATA_PAIR_PROP: u8 = CHAR_PROP_READ | CHAR_PROP_WRITE;
static SPP_DATA_PAIR_DATA: [u8; 20] = [0xe0; 20];

pub static mut SEND_TO_MASTER: [u8; 16] = [0; 16];

static SPP_DATA_CLIENT2SERVER_DATA: &[u8; 16] = unsafe { &SEND_TO_MASTER };

static USERDESC_UUID: u16 = GATT_UUID_CHAR_USER_DESC;

static SPP_STATUSNAME: &[u8] = b"Status";
static SPP_COMMANDNAME: &[u8] = b"Command";
static SPP_OTANAME: &[u8] = b"OTA";
static SPP_PAIRNAME: &[u8] = b"Pair";
static SPP_DEVICENAME: &[u8] = b"DevName";

fn mesh_status_write(p: &Packet) -> bool {
    if !PAIR_LOGIN_OK.get() {
        return true;
    }
    unsafe {
        let pktdata = &*slice_from_raw_parts_mut(addr_of!(p.att_write().value) as *mut u8, size_of::<PacketAttValue>());

        SPP_DATA_SERVER2CLIENT_DATA.copy_from_slice( &pktdata[0..4]);
        if p.head().l2cap_len > (3 + 1) {
            mesh_report_status_enable_mask(&pktdata[0..p.head().l2cap_len as usize - 3]);
        } else {
            mesh_report_status_enable(pktdata[0] != 0);
        }
    }
    return true;
}

#[derive(PartialEq)]
pub struct AttributeT {
    pub att_num: u8,
    pub uuid_len: u8,
    pub attr_len: u8,
    pub attr_max_len: u8,
    pub uuid: *const u8,
    pub p_attr_value: *mut u8,
    pub w: Option<fn(data: &Packet) -> bool>,
    pub r: Option<fn(data: &Packet) -> bool>,
}

#[macro_export]
macro_rules! attrdef {
    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr, $uuid:expr, $pAttrValue:expr, $w:expr, $r: expr) => {
        AttributeT {
            att_num: $attNum as u8,
            uuid_len: $uuidLen as u8,
            attr_len: $attrLen as u8,
            attr_max_len: $attrMaxLen as u8,
            uuid: unsafe { addr_of!($uuid) as *const u8 },
            p_attr_value: unsafe { $pAttrValue.as_ptr() as *mut u8 },
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
            uuid: unsafe { addr_of!($uuid) as *const u8 },
            p_attr_value: unsafe { $pAttrValue.as_ptr() as *mut u8 },
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

const fn size_of_val<T>(_: &T) -> usize {
    core::mem::size_of::<T>()
}

pub_mut!(
    gAttributes,
    [AttributeT; 29],
    [
        attrdef!((unsafe { gAttributes.0.len() - 1 }) as u8, 0, 0, 0),
        // gatt
        attrdefu!(6, 2, 2, 2, PRIMARY_SERVICE_UUID, GAP_SERVICE_UUID),
        attrdefu!(0, 2, 1, 1, CHARACTER_UUID, DEV_NAME_CHARACTER),
        attrdef!(
            0,
            2,
            BLE_G_DEV_NAME.len(),
            BLE_G_DEV_NAME.len(),
            DEV_NAME_UUID,
            BLE_G_DEV_NAME
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
        attrdef!(0, 2, 4, 4, FW_REVISION_CHAR_UUID, FW_REVISION_VALUE),
        attrdefu!(0, 2, 1, 1, CHARACTER_UUID, MANU_NAME_STRING_CHAR),
        attrdef!(
            0,
            2,
            MANU_NAME_STRING_VALUE.len(),
            MANU_NAME_STRING_VALUE.len(),
            MANU_NAME_STRING_CHAR_UUID,
            MANU_NAME_STRING_VALUE
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
            SPP_DATA_CLIENT2SERVER_DATA,
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
            Some(rf_link_slave_data_ota),
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
    ]
);
