use core::cell::RefCell;
use core::ptr::{addr_of, null, null_mut};
use core::slice;

use crate::{pub_mut, pub_static};
use crate::config::{DEVICE_NAME, MESH_NAME};
use crate::ota::rf_link_slave_data_ota;
use crate::sdk::ble_app::ble_ll_pair::{pair_read, pair_write};
use crate::sdk::ble_app::light_ll::{mesh_report_status_enable, mesh_report_status_enable_mask};
use crate::sdk::ble_app::rf_drv_8266::rf_link_slave_data_write;
use crate::sdk::light::*;
use crate::sdk::service::{
    SERVICE_UUID_DEVICE_INFORMATION, TELINK_SPP_DATA_CLIENT2SERVER, TELINK_SPP_DATA_OTA,
    TELINK_SPP_DATA_PAIR, TELINK_SPP_DATA_SERVER2CLIENT, TELINK_SPP_UUID_SERVICE,
};
use crate::state::State;
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

static primaryServiceUUID: u16 = GATT_UUID_PRIMARY_SERVICE;
static characterUUID: u16 = GATT_UUID_CHARACTER;

static gapServiceUUID: u16 = 0x1800;
static devNameUUID: u16 = GATT_UUID_DEVICE_NAME;
static appearanceUIID: u16 = 0x2a01;
static periConnParamUUID: u16 = 0x2a04;

static devInfoUUID: u16 = SERVICE_UUID_DEVICE_INFORMATION;
static clientCharacterCfgUUID: u16 = GATT_UUID_CLIENT_CHAR_CFG;

static fwRevision_charUUID: u16 = CHARACTERISTIC_UUID_FW_REVISION_STRING;
static manuNameString_charUUID: u16 = CHARACTERISTIC_UUID_MANU_NAME_STRING;
static modelId_charUUID: u16 = CHARACTERISTIC_UUID_MODEL_NUM_STRING;
static hwRevision_charUUID: u16 = CHARACTERISTIC_UUID_HW_REVISION_STRING;

// Device Name Characteristic Properties
static devNameCharacter: u8 = CHAR_PROP_READ;

// Appearance Characteristic Properties
static appearanceCharacter: u8 = CHAR_PROP_READ;
static appearance: u16 = GAP_APPEARE_UNKNOWN;

// Peripheral Preferred Connection Parameters Characteristic Properties
static periConnParamChar: u8 = CHAR_PROP_READ;

static fwRevisionChar: u8 = CHAR_PROP_READ;
pub_mut!(fwRevision_value, [u8; 4],
    [
        (BUILD_VERSION & 0xff) as u8,
        ((BUILD_VERSION >> 8) & 0xff) as u8,
        ((BUILD_VERSION >> 16) & 0xff) as u8,
        ((BUILD_VERSION >> 24) & 0xff) as u8
    ]
);

static manuNameStringChar: u8 = CHAR_PROP_READ;
static manuNameString_value: &[u8] = MESH_NAME.as_bytes();

static modelIdChar: u8 = CHAR_PROP_READ;
static modelId_value: &[u8] = b"model id 123";

static hwRevisionChar: u8 = CHAR_PROP_READ;
static hwRevision_value: u32 = 0x22222222;

#[repr(C, align(4))]
struct gap_periConnectParams_t {
    /** Minimum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
    pub intervalMin: u16,
    /** Maximum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
    pub intervalMax: u16,
    /** Number of LL latency connection events (0x0000 - 0x03e8) */
    pub latency: u16,
    /** Connection Timeout (0x000A - 0x0C80 * 10 ms) */
    pub timeout: u16,
}

const periConnParameters: gap_periConnectParams_t = gap_periConnectParams_t {
    intervalMin: 20,
    intervalMax: 40,
    latency: 0,
    timeout: 1000,
};

// note !!!DEVICE_NAME max 13 bytes
pub_static!(ble_g_devName, &'static [u8], DEVICE_NAME);

//////////////////////// SPP /////////////////////////////////////////////////////
// These must be pub_mut
pub_mut!(TelinkSppServiceUUID, [u8; 16], TELINK_SPP_UUID_SERVICE);
pub_mut!(
    TelinkSppDataServer2ClientUUID,
    [u8; 16],
    TELINK_SPP_DATA_SERVER2CLIENT
);
pub_mut!(
    TelinkSppDataClient2ServiceUUID,
    [u8; 16],
    TELINK_SPP_DATA_CLIENT2SERVER
);
pub_mut!(TelinkSppDataOtaUUID, [u8; 16], TELINK_SPP_DATA_OTA);
pub_mut!(TelinkSppDataPairUUID, [u8; 16], TELINK_SPP_DATA_PAIR);

// Spp data from Server to Client characteristic variables
static SppDataServer2ClientProp: u8 = CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_NOTIFY;

// Spp data from Client to Server characteristic variables
static SppDataClient2ServerProp: u8 =
    CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RSP;

// Spp data for OTA characteristic variables
static SppDataOtaProp: u8 = CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP;
static SppDataOtaData: [u8; 20] = [0xe0; 20];

static mut SppDataServer2ClientData: [u8; 4] = [0; 4];
static status_ccc: [u8; 2] = [0x01, 0x00];

// Spp data for OTA characteristic variables
static SppDataPairProp: u8 = CHAR_PROP_READ | CHAR_PROP_WRITE;
static SppDataPairData: [u8; 20] = [0xe0; 20];

pub_mut!(send_to_master, [u8; 16], [0; 16]);

static SppDataClient2ServerData: &[u8; 16] = unsafe { &send_to_master.0 };

static userdesc_UUID: u16 = GATT_UUID_CHAR_USER_DESC;

static spp_Statusname: &[u8] = b"Status";
static spp_Commandname: &[u8] = b"Command";
static spp_otaname: &[u8] = b"OTA";
static spp_pairname: &[u8] = b"Pair";
static spp_devicename: &[u8] = b"DevName";

fn mesh_status_write(state: &RefCell<State>, p: *const PacketAttWrite) -> bool {
    if !*get_pair_login_ok() {
        return true;
    }
    unsafe {
        SppDataServer2ClientData.copy_from_slice(slice::from_raw_parts(addr_of!((*p).value) as *const u8, 4));
        if (*p).l2cap_len > (3 + 1) {
            mesh_report_status_enable_mask(state, addr_of!((*p).value) as *const u8, (*p).l2cap_len - 3);
        } else {
            mesh_report_status_enable(state, if (*p).value[0] != 0 {true} else {false});
        }
    }
    return true;
}

#[repr(C, align(4))]
pub struct attribute_t {
    pub attNum: u8,
    pub uuidLen: u8,
    pub attrLen: u8,
    pub attrMaxLen: u8,
    pub uuid: *const u8,
    pub pAttrValue: *mut u8,
    pub w: Option<fn(state: &RefCell<State>, data: *const PacketAttWrite) -> bool>,
    pub r: Option<fn(state: &RefCell<State>, data: *const PacketAttWrite) -> bool>,
}

#[macro_export]
macro_rules! attrdef {
    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr, $uuid:expr, $pAttrValue:expr, $w:expr, $r: expr) => {
        attribute_t {
            attNum: $attNum as u8,
            uuidLen: $uuidLen as u8,
            attrLen: $attrLen as u8,
            attrMaxLen: $attrMaxLen as u8,
            uuid: unsafe { addr_of!($uuid) as *const u8 },
            pAttrValue: unsafe { $pAttrValue.as_ptr() as *mut u8 },
            w: $w,
            r: $r,
        }
    };

    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr, $uuid:expr, $pAttrValue:expr) => {
        attribute_t {
            attNum: $attNum as u8,
            uuidLen: $uuidLen as u8,
            attrLen: $attrLen as u8,
            attrMaxLen: $attrMaxLen as u8,
            uuid: unsafe { addr_of!($uuid) as *const u8 },
            pAttrValue: unsafe { $pAttrValue.as_ptr() as *mut u8 },
            w: None,
            r: None,
        }
    };

    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr) => {
        attribute_t {
            attNum: $attNum,
            uuidLen: $uuidLen,
            attrLen: $attrLen,
            attrMaxLen: $attrMaxLen,
            uuid: null(),
            pAttrValue: null_mut(),
            w: None,
            r: None,
        }
    };
}

#[macro_export]
macro_rules! attrdefu {
    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr, $uuid:expr, $pAttrValue:expr, $w:expr, $r: expr) => {
        attribute_t {
            attNum: $attNum as u8,
            uuidLen: $uuidLen as u8,
            attrLen: $attrLen as u8,
            attrMaxLen: $attrMaxLen as u8,
            uuid: addr_of!($uuid) as *const u8,
            pAttrValue: addr_of!($pAttrValue) as *mut u8,
            w: $w,
            r: $r,
        }
    };

    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr, $uuid:expr, $pAttrValue:expr) => {
        attribute_t {
            attNum: $attNum as u8,
            uuidLen: $uuidLen as u8,
            attrLen: $attrLen as u8,
            attrMaxLen: $attrMaxLen as u8,
            uuid: addr_of!($uuid) as *const u8,
            pAttrValue: addr_of!($pAttrValue) as *mut u8,
            w: None,
            r: None,
        }
    };

    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr) => {
        attribute_t {
            attNum: $attNum,
            uuidLen: $uuidLen,
            attrLen: $attrLen,
            attrMaxLen: $attrMaxLen,
            uuid: null(),
            pAttrValue: null_mut(),
            w: None,
            r: None,
        }
    };
}

const fn size_of_val<T>(_: &T) -> usize {
    core::mem::size_of::<T>()
}

pub_mut!(
    gAttributes_def,
    [attribute_t; 29],
    [
        attrdef!((unsafe { gAttributes_def.0.len() - 1 }) as u8, 0, 0, 0),
        // gatt
        attrdefu!(6, 2, 2, 2, primaryServiceUUID, gapServiceUUID),
        attrdefu!(0, 2, 1, 1, characterUUID, devNameCharacter),
        attrdef!(
            0,
            2,
            ble_g_devName.0.len(),
            ble_g_devName.0.len(),
            devNameUUID,
            ble_g_devName.0
        ),
        attrdef!(
            0,
            2,
            spp_devicename.len(),
            spp_devicename.len(),
            userdesc_UUID,
            spp_devicename
        ),
        attrdefu!(0, 2, 1, 1, characterUUID, appearanceCharacter),
        attrdefu!(
            0,
            2,
            size_of_val(&appearance),
            size_of_val(&appearance),
            appearanceUIID,
            appearance
        ),
        // device info
        attrdefu!(9, 2, 2, 2, primaryServiceUUID, devInfoUUID),
        attrdefu!(0, 2, 1, 1, characterUUID, fwRevisionChar),
        attrdef!(0, 2, 4, 4, fwRevision_charUUID, fwRevision_value.0),
        attrdefu!(0, 2, 1, 1, characterUUID, manuNameStringChar),
        attrdef!(
            0,
            2,
            manuNameString_value.len(),
            manuNameString_value.len(),
            manuNameString_charUUID,
            manuNameString_value
        ),
        attrdefu!(0, 2, 1, 1, characterUUID, modelIdChar),
        attrdef!(
            0,
            2,
            modelId_value.len(),
            modelId_value.len(),
            modelId_charUUID,
            modelId_value
        ),
        attrdefu!(0, 2, 1, 1, characterUUID, hwRevisionChar),
        attrdefu!(
            0,
            2,
            size_of_val(&hwRevision_value),
            size_of_val(&hwRevision_value),
            hwRevision_charUUID,
            hwRevision_value
        ),
        // spp u//0x10
        attrdef!(13, 2, 16, 16, primaryServiceUUID, TelinkSppServiceUUID.0),
        attrdefu!(0, 2, 1, 1, characterUUID, SppDataServer2ClientProp), //prop
        attrdef!(
            0,
            16,
            1,
            1,
            TelinkSppDataServer2ClientUUID,
            SppDataServer2ClientData,
            Some(mesh_status_write),
            None
        ), //value
        attrdef!(
            0,
            2,
            status_ccc.len(),
            status_ccc.len(),
            clientCharacterCfgUUID,
            status_ccc
        ), /*value, CCC is must for some third-party APP if there is notify or indication*/
        attrdefu!(0, 2, 1, 1, characterUUID, SppDataClient2ServerProp), //prop
        attrdef!(
            0,
            16,
            16,
            16,
            TelinkSppDataClient2ServiceUUID,
            SppDataClient2ServerData,
            Some(rf_link_slave_data_write),
            None
        ), //value
        attrdef!(
            0,
            2,
            spp_Commandname.len(),
            spp_Commandname.len(),
            userdesc_UUID,
            spp_Commandname
        ),
        attrdefu!(0, 2, 1, 1, characterUUID, SppDataOtaProp), //prop
        attrdef!(
            0,
            16,
            16,
            16,
            TelinkSppDataOtaUUID,
            SppDataOtaData,
            Some(rf_link_slave_data_ota),
            None
        ), //value
        attrdef!(
            0,
            2,
            spp_otaname.len(),
            spp_otaname.len(),
            userdesc_UUID,
            spp_otaname
        ),
        attrdefu!(0, 2, 1, 1, characterUUID, SppDataPairProp), //prop
        attrdef!(
            0,
            16,
            16,
            16,
            TelinkSppDataPairUUID,
            SppDataPairData,
            Some(pair_write),
            Some(pair_read)
        ), //value
        attrdef!(
            0,
            2,
            spp_pairname.len(),
            spp_pairname.len(),
            userdesc_UUID,
            spp_pairname
        ),
    ]
);
