use crate::BIT;
use crate::sdk::factory_reset::CFG_ADR_MAC_512K_FLASH;

pub const PAIR_CONFIG_VALID_FLAG: u8 = 0xFA;
pub const RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS: u16 = 30;
pub const LOOP_INTERVAL_US: u64 = 10000;

// must be 255
pub const PMW_MAX_TICK_BASE: u16 = 255;

// 209: freq = 600.4Hz
// 125: freq = 1003.9Hz
// 41: freq = 3060.7Hz
// More detail about why this should be above 3kHz can be found in this document:
// https://bio-licht.org/02_resources/info_ieee-pem_2014-09_led-flickering.pdf
pub const PMW_MAX_TICK_MULTI: u16 = 41;

// must less or equal than (255*256)
pub const PMW_MAX_TICK: u16 = PMW_MAX_TICK_BASE * PMW_MAX_TICK_MULTI;

pub const BRIDGE_MAX_CNT: u32 = 8;
pub const IRQ_TIMER1_ENABLE: bool = true;

pub const ONLINE_STATUS_TIMEOUT: u32 = 3000;

pub const AUTH_TIME: u32 = 60;
pub const MAX_GROUP_NUM: u8 = 8;

pub const START_UP_FLAG: u32 = 0x544c4e4b;

//app use
pub const PAR_READ_MESH_PAIR_CONFIRM: u8 = 0x0a;

// op cmd 11xxxxxxzzzzzzzzzzzzzzzz z's=VENDOR_ID  xxxxxx=LGT_CMD_
pub const LGT_CMD_NOTIFY_MESH: u8 = 0x02;
//internal use
pub const LGT_CMD_MESH_PAIR: u8 = 0x09;
pub const LGT_CMD_MESH_CMD_NOTIFY: u8 = 0x0a;
pub const CMD_NOTIFY_MESH_PAIR_END: u8 = 0x00;

pub const LGT_CMD_LIGHT_ONOFF: u8 = 0x10;
//internal use
pub const LGT_CMD_LIGHT_GRP_RSP1: u8 = 0x14;
//get group rsp: 8groups low 1bytes
pub const LGT_CMD_LIGHT_GRP_RSP2: u8 = 0x15;
//get group rsp: front 4groups 2bytes
pub const LGT_CMD_LIGHT_GRP_RSP3: u8 = 0x16;
//get group rsp: behind 4groups 2bytes
pub const LGT_CMD_LIGHT_CONFIG_GRP: u8 = 0x17;
//add or del group
//internal use
pub const LGT_CMD_LIGHT_READ_STATUS: u8 = 0x1a;
//get status req
pub const LGT_CMD_LIGHT_STATUS: u8 = 0x1b;
//get status rsp
pub const LGT_CMD_LIGHT_GRP_REQ: u8 = 0x1d;
//get group req
pub const LGT_CMD_CONFIG_DEV_ADDR: u8 = 0x20;
//add device address
pub const DEV_ADDR_PAR_WITH_MAC: u8 = 0x01;
pub const LGT_CMD_DEV_ADDR_RSP: u8 = 0x21;
//rsp
pub const LGT_CMD_KICK_OUT: u8 = 0x23;

pub const LGT_CMD_START_OTA_REQ: u8 = 0x24;
pub const LGT_CMD_START_OTA_RSP: u8 = 0x25;
pub const LGT_CMD_OTA_DATA_REQ: u8 = 0x26;
pub const LGT_CMD_OTA_DATA_RSP: u8 = 0x27;
pub const LGT_CMD_END_OTA_REQ: u8 = 0x28;
pub const LGT_CMD_END_OTA_RSP: u8 = 0x29;

pub const LGT_CMD_USER_NOTIFY_REQ: u8 = 0x2a;
pub const LGT_CMD_USER_NOTIFY_RSP: u8 = 0x2b;

pub const LGT_CMD_SET_LIGHT: u8 = 0x30;
pub const LGT_CMD_SET_MAC_ADDR: u8 = 0x31;
pub const LGT_POWER_ON: u8 = 0x32;
pub const LGT_PANIC_MSG: u8 = 0x34;

// Sent to acknowledge a non-notify message
pub const LGT_CMD_LIGHT_ACK: u8 = 0x35;

pub const GET_STATUS: u8 = 0;
pub const GET_GROUP1: u8 = 1;
// return 8 GROUP_ADDRESS(low 1byte)
pub const GET_GROUP2: u8 = 2;
// return front 4 GROUP_ADDRESS
pub const GET_GROUP3: u8 = 3;
// return behind 4 GROUP_ADDRESS
pub const GET_DEV_ADDR: u8 = 4;
// return device address
pub const GET_USER_NOTIFY: u8 = 7;
// return user notify info
pub const CMD_START_OTA: u8 = 8;
pub const CMD_OTA_DATA: u8 = 9;
pub const CMD_END_OTA: u8 = 10;

pub const LGT_CMD_SET_MESH_INFO: u8 = 0xc5;
pub const LGT_CMD_SET_DEV_ADDR: u8 = 0xc6;
pub const LGT_CMD_DEL_PAIR: u8 = 0xc7;
pub const LGT_CMD_MESH_PAIR_TIMEOUT: u8 = 0xcf;

pub const LIGHT_OFF_PARAM: u8 = 0x00;
pub const LIGHT_ON_PARAM: u8 = 0x01;

pub const LIGHT_ADD_GRP_PARAM: u8 = 0x01;
pub const LIGHT_DEL_GRP_PARAM: u8 = 0x00;

pub const DEVICE_ADDR_MASK_DEFAULT: u16 = 0x7FFF;

pub const MESH_NODE_MAX_NUM: usize = 64;
pub const MESH_NODE_MASK_LEN: usize = ((MESH_NODE_MAX_NUM + 31) >> 5);

pub const LIGHT_RX_BUFF_COUNT: usize = 4;
pub const BUFF_RESPONSE_PACKET_COUNT: usize = 16;
pub const BLT_FIFO_TX_PACKET_COUNT: usize = 8;

pub const PKT_CMD_LEN: usize = 11;

pub enum LightOpType {
    OpType1 = 1,
    OpType2 = 2,
    OpType3 = 3,
}

#[derive(PartialEq, Clone, Copy)]
pub enum ePairState {
    PairSetted = 0,
    PairSetting = 1,
    PairSetMeshTxStart = 2,
    PairSetMeshTxDone = 3, // send notify req, get mesh nodes' ac
    PairSetMeshRxDone = 4, // received all mesh nodes' ac, send cmd to switch to new mesh
}

#[derive(PartialEq, Clone, Copy)]
#[repr(C)]
pub enum OtaState {
    Continue = 0,
    // must zero
    Ok = 1,
    Error = 2,
    MasterOtaRebootOnly = 3,
}

// recover status before software reboot
pub enum RecoverStatus {
    LightOff = BIT!(0),
    MeshOtaMaster100 = BIT!(1),
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct AdvPrivate {
    pub manufacture_id: u16,
    // must vendor id to follow spec
    pub mesh_product_uuid: u16,
    pub mac_address: u32, // low 4 byte
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct AdvRspPrivate {
    pub manufacture_id: u16,             // 0
    // must vendor id to follow spec
    pub mesh_product_uuid: u16,           // 2
    pub mac_address: u32,                // 4
    // low 4 byte
    pub product_uuid: u16,               // 8
    pub status: u8,                     // 10
    pub device_address: u16,             // 11
    pub rsv: [u8; 16],                  // 13
}

#[derive(Clone, Copy)]
#[repr(C, align(4))]
pub struct LightRxBuff {
    pub dma_len: u8,        // 0
    pub unk1: [u8; 3],      // 1
    pub rssi: u8,           // 4
    pub unk2: [u8; 3],      // 5
    pub rx_time: u32,       // 8
    pub sno: [u8; 3],       // 12
    pub unk3: [u8; 5],      // 15
    pub mac: [u8; 4],       // 20
    pub unk4: [u8; 40]      // 24
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct StatusRecord {
    pub adr: [u8; 1],
    // don't modify, use internal
    pub alarm_id: u8, // don't modify, use internal
}

#[inline(always)]
pub fn is_unicast_addr(p_addr: &[u8]) -> bool {
    return p_addr[1] & 0x80 == 0;
}

// flash mesh extend shit needed to link
pub const CFG_ADR_CALIBRATION_512K_FLASH: u32 = CFG_ADR_MAC_512K_FLASH + 0x10;
// don't change
pub const CFG_SECTOR_ADR_CALIBRATION_CODE: u32 = CFG_ADR_CALIBRATION_512K_FLASH;

pub const UPDATE_CONNECT_PARA_DELAY_MS: u32 = 1000;

pub const INTERVAL_THRESHOLD: u16 = 16;
pub const ONLINE_STATUS_COMP: u32 = 3;
pub const SLAVE_READ_STATUS_BUSY_TIMEOUT: u32 = 25000;

pub const ADV_INTERVAL2LISTEN_INTERVAL: u16 = 4;
pub const ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL: u16 = 8;

pub const SEND_MESH_STATUS_INTERVAL_MS: u32 = 200;

#[derive(PartialEq, Copy, Clone)]
pub enum IrqHandlerStatus {
    None,
    Adv,
    Bridge,
    Rx,
    Listen
}
