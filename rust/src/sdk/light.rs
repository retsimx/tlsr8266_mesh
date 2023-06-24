use core::mem;
use core::mem::size_of;
use core::ptr::{null, null_mut};

use heapless::Deque;

use crate::{BIT, const_assert, pub_mut};
use crate::mesh::mesh_node_st_val_t;
use crate::sdk::factory_reset::CFG_ADR_MAC_512K_FLASH;

pub const PAIR_CONFIG_VALID_FLAG: u8 = 0xFA;

// These are filled at startup from values in config.rs
pub_mut!(pair_config_mesh_name, [u8; 16], [0; 16]);
pub_mut!(pair_config_mesh_pwd, [u8; 16], [0; 16]);
pub_mut!(pair_config_mesh_ltk, [u8; 16], [0; 16]);

pub_mut!(security_enable, bool, false);
pub_mut!(not_need_login, bool, false);
pub_mut!(pair_login_ok, bool, false);
pub_mut!(pair_sk, [u8; 16], [0; 16]);
pub_mut!(pair_sk_copy, [u8; 16], [0; 16]);
pub_mut!(slave_first_connected_tick, u32, 0);

pub_mut!(device_address, u16, 0);
pub_mut!(device_node_sn, u8, 1);
pub_mut!(dev_grp_next_pos, u16, 0);

pub static MAX_RELAY_NUM: u8 = 3;

pub_mut!(group_address, [u16; MAX_GROUP_NUM as usize], [0; MAX_GROUP_NUM as usize]);

pub_mut!(adr_flash_cfg_idx, i32, 0);

pub_mut!(slave_link_connected, bool, false);

pub_mut!(slave_read_status_busy, u8, 0);
pub_mut!(rf_slave_ota_busy, bool, false);

pub_mut!(pair_setting_flag, PairState, PairState::PairSetted);
pub_mut!(pair_ac, u32, 0);

pub_mut!(pair_nn, [u8; 16], [0; 16]);
pub_mut!(pair_pass, [u8; 16], [0; 16]);
pub_mut!(pair_ltk, [u8; 16], [0; 16]);
pub_mut!(pair_ltk_mesh, [u8; 16], [0; 16]);

pub_mut!(cur_ota_flash_addr, u32, 0);
pub_mut!(rf_slave_ota_finished_flag, OtaState, OtaState::Continue);
pub_mut!(rf_slave_ota_terminate_flag, bool, false);
pub_mut!(mesh_node_max, u8, 0);

pub const RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS: u16 = 30;
pub_mut!(rf_slave_ota_timeout_s, u16, RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);
pub_mut!(set_mesh_info_expired_flag, bool, false);
pub_mut!(set_mesh_info_time, u32, 0);

pub const LOOP_INTERVAL_US: u16 = 10000;

pub_mut!(pair_ivm, [u8; 8], [0, 0, 0, 0, 1, 0, 0, 0]);

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

pub const GET_STATUS: u8 = 0;
pub const GET_GROUP1: u8 = 1;
// return 8 group_address(low 1byte)
pub const GET_GROUP2: u8 = 2;
// return front 4 group_address
pub const GET_GROUP3: u8 = 3;
// return behind 4 group_address
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

pub const BUFF_RESPONSE_PACKET_COUNT: usize = 48;
pub const BLT_FIFO_TX_PACKET_COUNT: usize = 8;

pub const PKT_CMD_LEN: usize = 11;

pub const MAX_RELAY_COUNT: usize = 2;
pub const LAST_RELAY_TIME: usize = 3;
pub const CURRENT_RELAY_COUNT: usize = 4;

pub enum LightOpType {
    OpType1 = 1,
    OpType2 = 2,
    OpType3 = 3,
}

#[derive(PartialEq, Clone, Copy)]
pub enum PairState {
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

#[repr(C, align(4))]
pub struct RfPacketAdvIndModuleT {
	pub dma_len: u32,       // 0    //won't be a fixed number as previous, should adjust with the mouse package number

	pub _type: u8,			// 4	//RA(1)_TA(1)_RFU(2)_TYPE(4)
	pub rf_len: u8,			// 5	//LEN(6)_RFU(2)
	pub adv_a: [u8; 6],		// 6	//adv address
	pub data: [u8; 31]		// 12	//0-31 byte
}

#[repr(C, packed)]
pub struct AdvPrivate {
    pub manufacture_id: u16,
    // must vendor id to follow spec
    pub mesh_product_uuid: u16,
    pub mac_address: u32, // low 4 byte
}

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

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct PacketAttValue {
    pub sno: [u8; 3],
    pub src: [u8; 2],
    pub dst: [u8; 2],
    pub val: [u8; 23], // op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
                       // get status req: params[0]=tick  mac-app[2-3]=src-mac1...
                       // get status rsp: mac-app[0]=ttc  mac-app[1]=hop-count
}

#[repr(C, align(4))]
pub struct PacketLlData {
	pub dma_len: u32,   // 0         //won't be a fixed number as previous, should adjust with the mouse package number
	pub _type: u8,		// 4		//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	pub rf_len: u8,		// 5		//LEN(5)_RFU(3)
    pub l2cap_low: u8,  // 6        // 0x17
	pub l2cap_high: u8,	// 7		// 0
	pub chanid: u16,	// 8			//0x04,
	pub att: u8,		// 10		//0x12 for master; 0x1b for slave// as ttl when relay
	pub hl: u8,			// 11		// assigned by master
	pub hh: u8,			// 12		//
	pub sno: u8,        // 13
	pub nid: u8,        // 14
	pub ttc: u8,        // 15
	pub group: u16,     // 16
	pub sid: [u16; 2],
	pub cmd: [u8; PKT_CMD_LEN]
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct AppCmdValue {
   pub sno: [u8; 3],    // 0    13
   pub src: u16,    // 3    16
   pub dst: u16,    // 5    18
   pub op: u8,          // 7    20
   pub vendor_id: u16,  // 8    21
   pub par: [u8; 10],   // 10   23
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct PacketLlApp {
	pub dma_len: u32,   // 0
	pub _type: u8,      // 4
	pub rf_len: u8,     // 5
	pub l2cap_len: u16,  // 6
	pub chan_id: u16,    // 8
	pub opcode: u8,     // 10
	pub handle: u8,     // 11
	pub handle1: u8,    // 12
	pub app_cmd_v: AppCmdValue, // 13
	pub rsv: [u8; 10],
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct PacketAttReadRsp {
	pub dma_len: u32,   // 0        //won't be a fixed number as previous, should adjust with the mouse package number
	pub _type: u8,	    // 4		//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	pub rf_len: u8,		// 5		//LEN(5)_RFU(3)
	pub l2cap_len: u16,  // 6
	pub chan_id: u16,    // 8
	pub opcode: u8,     // 10
	pub value: [u8; 22] // 11
}

#[repr(C, align(4))]
pub struct PacketLlInit {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,				//RA(1)_TA(1)_RFU(2)_TYPE(4): connect request PDU
    pub rf_len: u8,				//LEN(6)_RFU(2)
    pub scan_a: [u8; 6],		// scan address
    pub adv_a: [u8; 6],			// adv address
    pub aa: [u8; 4],			// access code
    pub crcinit: [u8; 3],
    pub wsize: u8,
    pub woffset: u16,
    pub interval: u16,
    pub latency: u16,
    pub timeout: u16,
    pub chm: [u8; 5],
    pub hop: u8,				//sca(3)_hop(5)
}

#[derive(Clone, Copy)]
#[repr(C, align(4))]
pub struct PacketL2capData {
    pub l2cap_len: u16,
    pub chan_id: u16,
    pub opcode: u8,
    pub handle: u8,
    pub handle1: u8,
    pub value: [u8; 30],
}

#[repr(C, align(4))]
pub struct PktL2capSigConnParaUpRsp {
    pub dma_len: u32,
    pub _type: u8,
    pub rf_len: u8,
    pub l2cap_len: u16,
    pub chan_id: u16,
    pub code: u8,
    pub id: u8,
    pub data_len: u16,
    pub result: u16,
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct PacketAttWrite {
    pub dma_len: u32,
    pub rtype: u8,
    pub rf_len: u8,
    pub l2cap_len: u16,
    pub chan_id: u16,
    pub opcode: u8,
    pub handle: u8,
    pub handle1: u8,
    pub value: [u8; 30], //sno[3],src[2],dst[2],op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
}

#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct PacketAttCmd {
    pub dma_len: u32,   // 0
    pub _type: u8,      // 4
    pub rf_len: u8,     // 5
    pub l2cap_len: u16,  // 6
    pub chan_id: u16,    // 8
    pub opcode: u8,     // 10
    pub handle: u8,     // 11
    pub handle1: u8,    // 12
    pub value: PacketAttValue //sno[3],src[2],dst[2],op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
}

impl From<&MeshPkt> for &PacketAttCmd {
    fn from(pkt: &MeshPkt) -> Self {
        unsafe { mem::transmute(pkt) }
    }
}

#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct PacketAttData {
    pub dma_len: u32, //won't be a fixed number as previous, should adjust with the mouse package number

    pub _type: u8,
    //RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
    pub rf_len: u8,
    //LEN(5)_RFU(3)
    pub l2cap: u16,
    //0x17
    pub chanid: u16, //0x04,

    pub att: u8,
    //0x12 for master; 0x1b for slave// as ttl when relay
    pub hl: u8,
    // assigned by master
    pub hh: u8, //

    pub dat: [u8; 23],
}

#[derive(Clone, Copy)]
#[repr(C, align(4))]
pub struct MeshPkt {
	pub dma_len: u32,           // 0
	pub _type: u8,              // 4
	pub rf_len: u8,             // 5
	pub l2cap_len: u16,          // 6
	pub chan_id: u16,            // 8
	pub src_tx: u16,            // 10
	pub handle1: u8,            // 12 for flag
    pub sno: [u8; 3],           // 13
    pub src_adr: u16,           // 16
    pub dst_adr: u16,           // 18
    pub op: u8,                 // 20
    pub vendor_id: u16,         // 21
    pub par: [u8; 10],          // 23
    pub internal_par1: [u8; 5], // 33
    pub ttl: u8,                // 38
    pub internal_par2: [u8; 4], // 39
    pub no_use: [u8; 4]         // 43 size must 48, when is set to be rf tx address.
}

const_assert!(mem::size_of::<MeshPkt>() == 48);

#[repr(C, align(4))]
pub struct PacketAttMtu {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub	_type: u8,				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
    pub rf_len: u8,				//LEN(5)_RFU(3)
    pub l2cap_len: u16,
    pub chan_id: u16,
    pub opcode: u8,
    pub mtu: [u8; 2]
}

#[repr(C, align(4))]
pub struct PacketAttErrRsp {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
    pub rf_len: u8,				//LEN(5)_RFU(3)
    pub l2cap_len: u16,
    pub chan_id: u16,
    pub opcode: u8,
    pub err_opcode: u8,
    pub err_handle: u16,
    pub err_reason: u8
}

#[repr(C, align(4))]
pub struct PacketLlDataRsp {
	pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number

	pub _type: u8,				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	pub rf_len: u8,				//LEN(5)_RFU(3)

	pub l2cap: u16,				//0x17
	pub chanid: u16,				//0x04,

	pub att: u8,				//0x12 for master; 0x1b for slave// as ttl when relay
	pub hl: u8,				// assigned by master
	pub hh: u8,				//
	pub init: u8,
	pub dat: [u8; 14]
}

#[repr(C, align(4))]
pub struct PacketL2capHead {
	pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
	pub	_type: u8,				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	pub rf_len: u8,				//LEN(6)_RFU(2)
	pub l2cap_len: u16,
	pub chan_id: u16
}

#[repr(C, align(4))]
pub struct PacketVersionInd {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,				//RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,				//LEN(6)_RFU(2)
    pub opcode: u8,
    pub main_ver: u8,
    pub vendor: u16,
    pub sub_ver: u16
}

#[repr(C, align(4))]
pub struct PacketFeatureRsp {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,				//RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,				//LEN(6)_RFU(2)
    pub opcode: u8,
    pub data: [u8; 8]
}

#[repr(C, align(4))]
pub struct PacketCtrlUnknown {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,				//RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,				//LEN(6)_RFU(2)
    pub opcode: u8,
    pub data: [u8; 1]
}

#[repr(C, align(4))]
pub struct PacketLlWriteRsp {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number

    pub	_type: u8,				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
    pub rf_len: u8,				//LEN(5)_RFU(3)

    pub	l2cap: u16,				//0x17
    pub	chanid: u16,				//0x04,

    pub	op: u16
}

#[repr(C, align(4))]
pub struct PacketAttWriteRsp {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub	_type: u8,				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
    pub rf_len: u8,				//LEN(5)_RFU(3)
    pub l2cap_len: u16,
    pub chan_id: u16,
    pub opcode: u8
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct StatusRecord {
    pub adr: [u8; 1],
    // don't modify, use internal
    pub alarm_id: u8, // don't modify, use internal
}

#[derive(Clone, Copy, Debug)]
#[repr(C, packed)]
pub struct PktBuf {
    pub op: u8,
    pub sno: [u8; 3],
    pub notify_ok_flag: bool,
    pub sno2: [u8; 2], // for passive
}

#[derive(Clone, Copy)]
#[repr(C, align(4))]
pub struct PacketScanRsp {
	pub dma_len: u32,       // 0     //won't be a fixed number as previous, should adjust with the mouse package number

	pub _type: u8,          // 4				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	pub rf_len: u8,			// 5	//LEN(6)_RFU(2)
	pub adv_a: [u8; 6],		// 6	//adv address
	pub data: [u8; 31]		// 12	//0-31 byte

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

#[inline(always)]
pub fn is_unicast_addr(p_addr: &[u8]) -> bool {
    return p_addr[1] & 0x80 == 0;
}

// Shit required for linking
/////////////// password encode sk initial  ///////////////////////////////////////////////////
pub_mut!(pair_config_pwd_encode_sk, [u8; 17], [0; 17]);
pub_mut!(ble_pair_st, u8, 0);
pub_mut!(pair_enc_enable, bool, false);
pub_mut!(pair_ivs, [u8; 8], [0; 8]);
pub_mut!(pair_work, [u8; 16], [0; 16]);
pub_mut!(pairRead_pending, bool, false);
pub_mut!(pkt_read_rsp, PacketAttReadRsp, PacketAttReadRsp {
    dma_len: 0x1d,
	_type: 2,
	rf_len: 0x1b,
	l2cap_len: 0x17,
	chan_id: 0x4,
	opcode: 0xb,
	value: [0; 22]
});

/////////////// adv par define ///////////////////////////////////////////////////
pub_mut!(adv_interval2listen_interval, u16, 4);
// unit: default is 40ms, setting by 40000 from rf_link_slave_init (40000);
pub_mut!(online_status_interval2listen_interval, u16, 8);
// unit: default is 40ms, setting by 40000 from rf_link_slave_init (40000);
pub_mut!(rf_slave_ota_busy_mesh, bool, false);

// flash mesh extend shit needed to link
pub const CFG_ADR_CALIBRATION_512K_FLASH: u32 = CFG_ADR_MAC_512K_FLASH + 0x10;
// don't change
pub const CFG_SECTOR_ADR_CALIBRATION_CODE: u32 = CFG_ADR_CALIBRATION_512K_FLASH;
pub_mut!(
    flash_sector_calibration,
    u32,
    CFG_SECTOR_ADR_CALIBRATION_CODE
);

pub_mut!(
    slave_status_record,
    [StatusRecord; MESH_NODE_MAX_NUM as usize],
    [StatusRecord {
        adr: [0],
        alarm_id: 0
    }; MESH_NODE_MAX_NUM as usize]
);
pub_mut!(
    slave_status_record_size,
    u16,
    size_of::<[StatusRecord; MESH_NODE_MAX_NUM as usize]>() as u16
);

pub_mut!(
    rc_pkt_buf,
    Deque<PktBuf, 5>,
    Deque::new()
);

pub_mut!(dev_address_next_pos, u16, 0);

pub_mut!(need_update_connect_para, bool, false);
pub const UPDATE_CONNECT_PARA_DELAY_MS: u32 = 1000;

pub_mut!(update_interval_user_max, u16, 0);
pub_mut!(update_interval_user_min, u16, 0);
pub_mut!(update_timeout_user, u32, 0);
pub const INTERVAL_THRESHOLD: u16 = 16;
pub_mut!(update_interval_flag, u16, 0);
pub_mut!(update_interval_time, bool, false);
pub const ONLINE_STATUS_COMP: u32 = 3;
pub_mut!(slave_data_valid, u32, 0);
pub_mut!(t_bridge_cmd, u32, 0);
pub_mut!(st_brige_no, u32, 0);
pub_mut!(app_cmd_time, u32, 0);
pub_mut!(mesh_user_cmd_idx, u8, 0);
pub_mut!(slave_tx_cmd_time, u32, 0);
pub_mut!(slave_status_buffer_wptr, usize, 0);
pub_mut!(slave_status_buffer_rptr, usize, 0);
pub_mut!(slave_stat_sno, [u8; 3], [0; 3]);
pub_mut!(slave_read_status_unicast_flag, u8, 0);
pub_mut!(mesh_node_report_enable, bool, false);
pub_mut!(slave_timing_adjust_enable, bool, false);
pub_mut!(slave_tick_brx, u32, 0);
pub_mut!(slave_window_offset, u32, 0);
pub_mut!(slave_instant, u16, 0);
pub_mut!(slave_status_tick, u8, 0);
pub_mut!(slave_link_cmd, u8, 0);
pub_mut!(rcv_pkt_ttc, u8, 0);
pub_mut!(org_ttl, u8, 0);
pub_mut!(slave_read_status_response, bool, false);
pub_mut!(slave_sno, [u8; 3], [0; 3]);
pub_mut!(slave_status_record_idx, usize, 0);
pub_mut!(notify_req_mask_idx, u8, 0);
pub_mut!(adv_flag, bool, true);
pub_mut!(online_st_flag, bool, true);
pub_mut!(slave_read_status_busy_time, u32, 0);
pub const SLAVE_READ_STATUS_BUSY_TIMEOUT: u32 = 25000;
pub_mut!(st_listen_no, u32, 0);

pub_mut!(pkt_light_adv_status, PacketAttWrite, PacketAttWrite {
    dma_len: 0x27,
    rtype: 2,
    rf_len: 0x25,
    l2cap_len: 0x21,
    chan_id: 0xffff,
    opcode: 0,
    handle: 0,
    handle1: 0,
    value: [0; 30]
});

pub_mut!(pkt_mesh, MeshPkt, MeshPkt {
    dma_len: 0,
    _type: 0,
    rf_len: 0,
    l2cap_len: 0,
    chan_id: 0,
    src_tx: 0,
    handle1: 0,
    sno: [0; 3],
    src_adr: 0,
    dst_adr: 0,
    op: 0,
    vendor_id: 0,
    par: [0; 10],
    internal_par1: [0; 5],
    ttl: 0,
    internal_par2: [0; 4],
    no_use: [0; 4]
});
pub_mut!(pkt_mesh_user_cmd_buf, MeshPkt, MeshPkt {
    dma_len: 0,
    _type: 0,
    rf_len: 0,
    l2cap_len: 0,
    chan_id: 0,
    src_tx: 0,
    handle1: 0,
    sno: [0; 3],
    src_adr: 0,
    dst_adr: 0,
    op: 0,
    vendor_id: 0,
    par: [0; 10],
    internal_par1: [0; 5],
    ttl: 0,
    internal_par2: [0; 4],
    no_use: [0; 4]
});
pub_mut!(pkt_init, PacketLlInit, PacketLlInit {
    dma_len: 0x24,
    _type: 0x5,
    rf_len: 0x22,
    scan_a: [0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5],
    adv_a: [0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5],
    aa: [0xaa, 0x55, 0x55, 0xaa],
    crcinit: [0x55, 0x55, 0x55],
    wsize: 2,
    woffset: 0x1f,
    interval: 0x20,
    latency: 0,
    timeout: 0x48,
    chm: [0xff, 0xff, 0xff, 0xff, 0x1f],
    hop: 0xac
});

#[derive(PartialEq, Copy, Clone)]
pub enum IrqHandlerStatus {
    None,
    Adv,
    Bridge,
    Rx,
    Listen
}