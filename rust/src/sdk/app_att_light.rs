use crate::config::{DEVICE_NAME, MESH_NAME};
use crate::sdk::light::*;
use crate::sdk::service::{
    SERVICE_UUID_DEVICE_INFORMATION, TELINK_SPP_DATA_CLIENT2SERVER, TELINK_SPP_DATA_OTA,
    TELINK_SPP_DATA_PAIR, TELINK_SPP_DATA_SERVER2CLIENT, TELINK_SPP_UUID_SERVICE,
};
use core::convert::AsRef;
use core::ffi::CStr;
use core::mem::transmute;
use core::ptr::{addr_of, null};
use crate::{pub_mut, pub_static};

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
pub_mut!(fwRevision_value, [u8; 4], [1, 2, 3, 4]);

static manuNameStringChar: u8 = CHAR_PROP_READ;
static manuNameString_value: &[u8] = MESH_NAME.as_bytes();

static modelIdChar: u8 = CHAR_PROP_READ;
static modelId_value: &CStr = unsafe { CStr::from_bytes_with_nul_unchecked(b"model id 123\0") };

static hwRevisionChar: u8 = CHAR_PROP_READ;
static hwRevision_value: u32 = 0x22222222;

#[repr(C, packed)]
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
pub_static!(ble_g_devName, &'static [u8], DEVICE_NAME.to_bytes());

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

static SppDataServer2ClientData: [u8; 4] = [0; 4];
static status_ccc: [u8; 2] = [0x01, 0x00];

// Spp data for OTA characteristic variables
static SppDataPairProp: u8 = CHAR_PROP_READ | CHAR_PROP_WRITE;
static SppDataPairData: [u8; 20] = [0xe0; 20];

pub_mut!(send_to_master, [u8; 16]);

static SppDataClient2ServerData: &[u8; 16] = unsafe { &send_to_master };

static userdesc_UUID: u16 = GATT_UUID_CHAR_USER_DESC;

static spp_Statusname: &CStr = unsafe { CStr::from_bytes_with_nul_unchecked(b"Status\0") };
static spp_Commandname: &CStr = unsafe { CStr::from_bytes_with_nul_unchecked(b"Command\0") };
static spp_otaname: &CStr = unsafe { CStr::from_bytes_with_nul_unchecked(b"OTA\0") };
static spp_pairname: &CStr = unsafe { CStr::from_bytes_with_nul_unchecked(b"Pair\0") };
static spp_devicename: &CStr = unsafe { CStr::from_bytes_with_nul_unchecked(b"DevName\0") };

unsafe extern "C" fn meshStatusWrite(pw: *const u8) -> u32 {
    if !*get_pair_login_ok() {
        return 1;
    }

    let p: *const rf_packet_att_write_t = pw as *const rf_packet_att_write_t;
    *transmute::<&[u8; 4], *mut u8>(&SppDataServer2ClientData) = (*p).value;
    if (*p).l2capLen > (3 + 1) {
        _mesh_report_status_enable_mask(&(*p).value, (*p).l2capLen - 3);
    } else {
        _mesh_report_status_enable((*p).value);
    }
    return 1;
}

#[repr(C, packed)]
pub struct attribute_t {
    pub attNum: u8,
    pub uuidLen: u8,
    pub attrLen: u8,
    pub attrMaxLen: u8,
    pub uuid: *const u8,
    pub pAttrValue: *const u8,
    pub w: *const u8,
    pub r: *const u8,
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
            pAttrValue: unsafe { $pAttrValue.as_ptr() as *const u8 },
            w: $w as *const u8,
            r: $r as *const u8,
        }
    };

    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr, $uuid:expr, $pAttrValue:expr) => {
        attribute_t {
            attNum: $attNum as u8,
            uuidLen: $uuidLen as u8,
            attrLen: $attrLen as u8,
            attrMaxLen: $attrMaxLen as u8,
            uuid: unsafe { addr_of!($uuid) as *const u8 },
            pAttrValue: unsafe { $pAttrValue.as_ptr() as *const u8 },
            w: null(),
            r: null(),
        }
    };

    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr) => {
        attribute_t {
            attNum: $attNum,
            uuidLen: $uuidLen,
            attrLen: $attrLen,
            attrMaxLen: $attrMaxLen,
            uuid: null(),
            pAttrValue: null(),
            w: null(),
            r: null(),
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
            pAttrValue: addr_of!($pAttrValue) as *const u8,
            w: $w as *const u8,
            r: $r as *const u8,
        }
    };

    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr, $uuid:expr, $pAttrValue:expr) => {
        attribute_t {
            attNum: $attNum as u8,
            uuidLen: $uuidLen as u8,
            attrLen: $attrLen as u8,
            attrMaxLen: $attrMaxLen as u8,
            uuid: addr_of!($uuid) as *const u8,
            pAttrValue: addr_of!($pAttrValue) as *const u8,
            w: null(),
            r: null(),
        }
    };

    ($attNum:expr, $uuidLen:expr, $attrLen:expr, $attrMaxLen:expr) => {
        attribute_t {
            attNum: $attNum,
            uuidLen: $uuidLen,
            attrLen: $attrLen,
            attrMaxLen: $attrMaxLen,
            uuid: null(),
            pAttrValue: null(),
            w: null(),
            r: null(),
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
        attrdef!((unsafe { gAttributes_def.len() - 1 }) as u8, 0, 0, 0),
        // gatt
        attrdefu!(6, 2, 2, 2, primaryServiceUUID, gapServiceUUID),
        attrdefu!(0, 2, 1, 1, characterUUID, devNameCharacter),
        attrdef!(
            0,
            2,
            ble_g_devName.len(),
            ble_g_devName.len(),
            devNameUUID,
            ble_g_devName
        ),
        attrdef!(
            0,
            2,
            spp_devicename.to_bytes().len(),
            spp_devicename.to_bytes().len(),
            userdesc_UUID,
            spp_devicename.to_bytes()
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
        attrdef!(0, 2, 4, 4, fwRevision_charUUID, fwRevision_value),
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
            modelId_value.to_bytes().len(),
            modelId_value.to_bytes().len(),
            modelId_charUUID,
            modelId_value.to_bytes()
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
        attrdef!(13, 2, 16, 16, primaryServiceUUID, TelinkSppServiceUUID),
        attrdefu!(0, 2, 1, 1, characterUUID, SppDataServer2ClientProp), //prop
        attrdef!(
            0,
            16,
            1,
            1,
            TelinkSppDataServer2ClientUUID,
            SppDataServer2ClientData,
            meshStatusWrite,
            0
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
            SppDataClient2ServerData
        ), //value
        attrdef!(
            0,
            2,
            spp_Commandname.to_bytes().len(),
            spp_Commandname.to_bytes().len(),
            userdesc_UUID,
            spp_Commandname.to_bytes()
        ),
        attrdefu!(0, 2, 1, 1, characterUUID, SppDataOtaProp), //prop
        attrdef!(0, 16, 16, 16, TelinkSppDataOtaUUID, SppDataOtaData), //value
        attrdef!(
            0,
            2,
            spp_otaname.to_bytes().len(),
            spp_otaname.to_bytes().len(),
            userdesc_UUID,
            spp_otaname.to_bytes()
        ),
        attrdefu!(0, 2, 1, 1, characterUUID, SppDataPairProp), //prop
        attrdef!(
            0,
            16,
            16,
            16,
            TelinkSppDataPairUUID,
            SppDataPairData,
            _pairWrite,
            _pairRead
        ), //value
        attrdef!(
            0,
            2,
            spp_pairname.to_bytes().len(),
            spp_pairname.to_bytes().len(),
            userdesc_UUID,
            spp_pairname.to_bytes()
        ),
    ]
);
