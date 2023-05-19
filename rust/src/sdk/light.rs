use crate::sdk::factory_reset::CFG_ADR_MAC_512K_FLASH;
use core::mem;
use core::mem::{size_of, size_of_val, MaybeUninit};
use core::ptr::{null, null_mut};
use crate::{const_concat};
use crate::{pub_mut, BIT};
use crate::config::PAIR_VALID_FLAG;
use crate::mesh::mesh_node_st_val_t;

pub const PAIR_CONFIG_VALID_FLAG: u8 = 0xFA;

// These are filled at startup from values in config.rs
pub_mut!(pair_config_mesh_name, [u8; 16], [0; 16]);
pub_mut!(pair_config_mesh_pwd, [u8; 16], [0; 16]);
pub_mut!(pair_config_mesh_ltk, [u8; 16], [0; 16]);

pub_mut!(p_adv_pri_data, *const ll_adv_private_t, null());
pub_mut!(p_adv_rsp_data, *const ll_adv_rsp_private_t, null_mut());
pub_mut!(adv_private_data_len, u8, 0);

pub_mut!(security_enable, bool, false);
pub_mut!(not_need_login, bool, false);
pub_mut!(pair_login_ok, bool, false);
pub_mut!(pair_sk, [u8; 16], [0; 16]);
pub_mut!(pair_sk_copy, [u8; 16], [0; 16]);
pub_mut!(slave_first_connected_tick, u32, 0);

pub_mut!(device_address, u16, 0);
pub_mut!(device_node_sn, u8, 1);
pub_mut!(dev_grp_next_pos, u16, 0);
pub_mut!(max_relay_num, u8, 3);

pub_mut!(group_address, [u16; MAX_GROUP_NUM as usize], [0; MAX_GROUP_NUM as usize]);

pub_mut!(slave_p_mac, *const u8, null());

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
pub_mut!(rf_slave_ota_finished_flag, OtaState, OtaState::CONTINUE);
pub_mut!(rf_slave_ota_terminate_flag, bool, false);
pub_mut!(mesh_node_max, u8, 0);

pub const RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS: u16 = 30;
pub_mut!(rf_slave_ota_timeout_s, u16, RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);
pub_mut!(set_mesh_info_expired_flag, bool, false);
pub_mut!(set_mesh_info_time, u32, 0);

pub const LOOP_INTERVAL_US: u16 = 10000;

pub_mut!(pair_ivm, [u8; 8], [0, 0, 0, 0, 1, 0, 0, 0]);

pub_mut!(p_cb_rx_from_mesh, Option<fn(p: &app_cmd_value_t)>, Option::None);
// todo: new_node might be bool
pub_mut!(p_mesh_node_status_callback, Option<fn(p: *const mesh_node_st_val_t, new_node: u8)>, None);


pub const PMW_MAX_TICK_BASE: u16 = 255;
// must 255
pub const PMW_MAX_TICK_MULTI: u16 = 209;
// 209: freq = 600.4Hz
pub const PMW_MAX_TICK_1: u16 = PMW_MAX_TICK_BASE * PMW_MAX_TICK_MULTI;
// must less or equal than (255*256)
pub const PMW_MAX_TICK: u16 = PMW_MAX_TICK_1;

pub const BRIDGE_MAX_CNT: u32 = 8;
pub const IRQ_TIMER1_ENABLE: bool = true;

pub const ONLINE_STATUS_TIMEOUT: u32 = 3000;

pub const AUTH_TIME: u32 = 60;
pub const MAX_GROUP_NUM: u8 = 8;

pub const START_UP_FLAG: u32 = 0x544c4e4b;

// op cmd 11xxxxxxzzzzzzzzzzzzzzzz z's=VENDOR_ID  xxxxxx=LGT_CMD_
pub const LGT_CMD_READ_SCENE: u8 = 0x00;
//
pub const LGT_CMD_SCENE_RSP: u8 = 0x01;
//
pub const LGT_CMD_NOTIFY_MESH: u8 = 0x02;
//use between mesh
pub const LGT_CMD_SW_DATA: u8 = 0x03;
pub const LGT_CMD_SW_RSP: u8 = 0x04;
pub const LGT_CMD_TIME_SYNC: u8 = 0x05;
//internal use
pub const LGT_CMD_MESH_OTA_DATA: u8 = 0x06;
//internal use
pub const CMD_OTA_FW_VERSION: u16 = 0xff00;
// must larger than 0xff00
pub const CMD_OTA_START: u16 = 0xff01;
pub const CMD_OTA_END: u16 = 0xff02;
pub const CMD_STOP_MESH_OTA: u16 = 0xfffe;
//app use
pub const CMD_START_MESH_OTA: u16 = 0xffff; //app use  	// use in rf_link_data_callback()

pub const LGT_CMD_MESH_OTA_READ: u8 = 0x07;
//internal use
pub const PAR_READ_VER: u8 = 0x00;
//internal use
pub const PAR_READ_END: u8 = 0x01;
//internal use
pub const PAR_READ_MAP: u8 = 0x02;
//internal use
pub const PAR_READ_CALI: u8 = 0x03;
//internal use
pub const PAR_READ_PROGRESS: u8 = 0x04;
//internal use
pub const PAR_APP_READ_ST: u8 = 0x05;
//app use: 0 : idle; 1: slave mode; 2: master mode
pub const PAR_APP_OTA_HCI_TYPE_SET: u8 = 0x06;
//app use
pub const PAR_READ_MESH_PAIR_CONFIRM: u8 = 0x0a;
pub const LGT_CMD_MESH_OTA_READ_RSP: u8 = 0x08;
//internal use
pub const LGT_CMD_MESH_PAIR: u8 = 0x09;
pub const LGT_CMD_MESH_CMD_NOTIFY: u8 = 0x0a;
pub const CMD_NOTIFY_MESH_PAIR_END: u8 = 0x00;
pub const LGT_CMD_FRIENDSHIP: u8 = 0x0b;
pub const CMD_CTL_POLL: u8 = 0x01;
pub const CMD_CTL_UPDATE: u8 = 0x02;
pub const CMD_CTL_REQUEST: u8 = 0x03;
pub const CMD_CTL_OFFER: u8 = 0x04;
pub const CMD_CTL_GROUP_REPORT: u8 = 0x05;
pub const CMD_CTL_GROUP_REPORT_ACK: u8 = 0x06;

pub const LGT_CMD_LIGHT_ONOFF: u8 = 0x10;
pub const LGT_CMD_LIGHT_ON: u8 = 0x10;
pub const LGT_CMD_LIGHT_OFF: u8 = 0x11;
//internal use
pub const LGT_CMD_LIGHT_SET: u8 = 0x12;
//set lumen
pub const LGT_CMD_SWITCH_CONFIG: u8 = 0x13;
//internal use
pub const LGT_CMD_LIGHT_GRP_RSP1: u8 = 0x14;
//get group rsp: 8groups low 1bytes
pub const LGT_CMD_LIGHT_GRP_RSP2: u8 = 0x15;
//get group rsp: front 4groups 2bytes
pub const LGT_CMD_LIGHT_GRP_RSP3: u8 = 0x16;
//get group rsp: behind 4groups 2bytes
pub const LGT_CMD_LIGHT_CONFIG_GRP: u8 = 0x17;
//add or del group
pub const LGT_CMD_LUM_UP: u8 = 0x18;
//internal use
pub const LGT_CMD_LUM_DOWN: u8 = 0x19;
//internal use
pub const LGT_CMD_LIGHT_READ_STATUS: u8 = 0x1a;
//get status req
pub const LGT_CMD_LIGHT_STATUS: u8 = 0x1b;
//get status rsp
pub const LGT_CMD_LIGHT_ONLINE: u8 = 0x1c;
//get online status req//internal use
pub const LGT_CMD_LIGHT_GRP_REQ: u8 = 0x1d;
//get group req
pub const LGT_CMD_LEFT_KEY: u8 = 0x1e;
//internal use
pub const LGT_CMD_RIGHT_KEY: u8 = 0x1f;
//internal use
pub const LGT_CMD_CONFIG_DEV_ADDR: u8 = 0x20;
//add device address
pub const DEV_ADDR_PAR_ADR: u8 = 0x00;
pub const DEV_ADDR_PAR_WITH_MAC: u8 = 0x01;
pub const LGT_CMD_DEV_ADDR_RSP: u8 = 0x21;
//rsp
pub const LGT_CMD_SET_RGB_VALUE: u8 = 0x22;
//params[0]:1=R 2=G 3=B 4=RGB params[1~3]:R/G/B value 0~255  params[0]:5=CT params[1]=value 0~100%
pub const LGT_CMD_KICK_OUT: u8 = 0x23;
//
pub const LGT_CMD_SET_TIME: u8 = 0x24;
//
pub const LGT_CMD_ALARM: u8 = 0x25;
//
pub const LGT_CMD_READ_ALARM: u8 = 0x26;
//
pub const LGT_CMD_ALARM_RSP: u8 = 0x27;
//
pub const LGT_CMD_GET_TIME: u8 = 0x28;
//
pub const LGT_CMD_TIME_RSP: u8 = 0x29;
//
pub const LGT_CMD_USER_NOTIFY_REQ: u8 = 0x2a;
//
pub const LGT_CMD_USER_NOTIFY_RSP: u8 = 0x2b;
//
pub const LGT_CMD_SET_SW_GRP: u8 = 0x2c;
pub const LGT_CMD_LIGHT_RC_SET_RGB: u8 = 0x2d;
pub const LGT_CMD_SET_SCENE: u8 = 0x2e;
pub const LGT_CMD_LOAD_SCENE: u8 = 0x2f;
pub const LGT_CMD_SET_LIGHT: u8 = 0x30;
pub const LGT_CMD_SET_MAC_ADDR: u8 = 0x31;
pub const LGT_POWER_ON: u8 = 0x32;
pub const LGT_CLEAR_LUM_STATE: u8 = 0x33;
pub const LGT_PANIC_MSG: u8 = 0x34;
pub const LGT_TRIGGER_PANIC: u8 = 0x35;

pub const GET_STATUS: u8 = 0;
pub const GET_GROUP1: u8 = 1;
// return 8 group_address(low 1byte)
pub const GET_GROUP2: u8 = 2;
// return front 4 group_address
pub const GET_GROUP3: u8 = 3;
// return behind 4 group_address
pub const GET_DEV_ADDR: u8 = 4;
// return device address
pub const GET_ALARM: u8 = 5;
// return alarm info
pub const GET_TIME: u8 = 6;
// return time info
pub const GET_USER_NOTIFY: u8 = 7;
// return user notify info
pub const GET_SCENE: u8 = 8;
//
pub const GET_MESH_OTA: u8 = 9; //

pub const LGT_CMD_FRIEND_SHIP_OK: u8 = 0xb0;
pub const LGT_CMD_FRIEND_SHIP_DISCONNECT: u8 = 0xb1;
pub const LGT_CMD_PAIRING_LIGHT_OFF: u8 = 0xc1;
pub const LGT_CMD_PAIRING_LIGHT_ON: u8 = 0xc2;
pub const LGT_CMD_PAIRING_LIGHT_SEL: u8 = 0xc3;
pub const LGT_CMD_PAIRING_LIGHT_CONFIRM: u8 = 0xc4;
pub const LGT_CMD_SET_MESH_INFO: u8 = 0xc5;
pub const LGT_CMD_SET_DEV_ADDR: u8 = 0xc6;
pub const LGT_CMD_DEL_PAIR: u8 = 0xc7;
pub const LGT_CMD_PAIRING_LIGHT_RESPONSE: u8 = 0xc9;
pub const LGT_CMD_PAIRING_LIGHT_OTA_EN: u8 = 0xce;
pub const LGT_CMD_MESH_PAIR_TIMEOUT: u8 = 0xcf;
pub const LGT_CMD_DUAL_MODE_MESH: u8 = 0xd0;

pub const LIGHT_OFF_PARAM: u8 = 0x00;
pub const LIGHT_ON_PARAM: u8 = 0x01;
pub const ON_OFF_NORMAL: u8 = 0x00;
pub const ON_OFF_FROM_OTA: u8 = 0x01;
pub const ON_OFF_FROM_PASSIVE_DEV: u8 = 0x02;
// sent from passive device
pub const ON_OFF_FROM_PASSIVE_DEV_ALT: u8 = 0x03; // tx command: alter from passive device command

pub const LIGHT_SYNC_REST_PARAM: u8 = 0x02;

pub const LIGHT_ADD_GRP_PARAM: u8 = 0x01;
pub const LIGHT_DEL_GRP_PARAM: u8 = 0x00;

pub const DEVICE_ADDR_MASK_DEFAULT: u16 = 0x7FFF;

pub const MESH_NODE_MAX_NUM: u16 = 64;

pub const PKT_CMD_LEN: usize = 11;

pub const MAX_RELAY_COUNT: usize = 2;
pub const LAST_RELAY_TIME: usize = 3;
pub const CURRENT_RELAY_COUNT: usize = 4;

pub enum LightOpType {
    op_type_1 = 1,
    op_type_2 = 2,
    op_type_3 = 3,
}

#[derive(PartialEq, Clone, Copy)]
#[allow(dead_code)]
pub enum PairState {
    PairSetted = 0,
    PairSetting = 1,
    PairSetMeshTxStart = 2,
    PairSetMeshTxDone = 3, // send notify req, get mesh nodes' ac
    PairSetMeshRxDone = 4, // received all mesh nodes' ac, send cmd to switch to new mesh
}

#[derive(PartialEq)]
pub enum APP_OTA_HCI_TYPE {
    GATT_ONLY = 0,
    MESH = 1,
}

#[derive(PartialEq, Clone, Copy)]
#[repr(C)]
pub enum OtaState {
    CONTINUE = 0,
    // must zero
    OK = 1,
    ERROR = 2,
    MASTER_OTA_REBOOT_ONLY = 3,
}

#[derive(PartialEq)]
#[repr(C)]
pub enum MeshOtaLed {
    OK,
    ERROR,
    STOP,
}

#[derive(PartialEq)]
pub enum CMD_TYPE {
    NORMAL = 0,
    PASSIVE_DEV = 1,
    PASSIVE_DEV_ALT = 2,
}

// recover status before software reboot
pub enum RecoverStatus {
    LightOff = BIT!(0),
    MeshOtaMaster100 = BIT!(1),
    // LowBattFlg = BIT!(2),
    // LOW_BATT_LOOP_FLG           = BIT(3),    // 0 means check by user_init, 1 means by main loop
}

#[repr(C, align(4))]
pub struct rf_packet_adv_ind_module_t {
	pub dma_len: u32,       // 0    //won't be a fixed number as previous, should adjust with the mouse package number

	pub _type: u8,			// 4	//RA(1)_TA(1)_RFU(2)_TYPE(4)
	pub rf_len: u8,			// 5	//LEN(6)_RFU(2)
	pub advA: [u8; 6],		// 6	//address
	pub data: [u8; 31]		// 12	//0-31 byte
}

#[repr(C, packed)]
pub struct ll_adv_private_t {
    pub ManufactureID: u16,
    // must vendor id to follow spec
    pub MeshProductUUID: u16,
    pub MacAddress: u32, // low 4 byte
}

#[repr(C, packed)]
pub struct ll_adv_rsp_private_t {
    pub ManufactureID: u16,             // 0
    // must vendor id to follow spec
    pub MeshProductUUID: u16,           // 2
    pub MacAddress: u32,                // 4
    // low 4 byte
    pub ProductUUID: u16,               // 8
    pub status: u8,                     // 10
    pub DeviceAddress: u16,             // 11
    pub rsv: [u8; 16],                  // 13
}

#[repr(C, packed)]
pub struct rf_packet_att_value_t {
    pub sno: [u8; 3],
    pub src: [u8; 2],
    pub dst: [u8; 2],
    pub val: [u8; 23], // op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
                       // get status req: params[0]=tick  mac-app[2-3]=src-mac1...
                       // get status rsp: mac-app[0]=ttc  mac-app[1]=hop-count
}

#[repr(C, align(4))]
pub struct rf_packet_ll_data_t {
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
pub struct app_cmd_value_t {
   pub sno: [u8; 3],    // 0    13
   pub src: u16,    // 3    16
   pub dst: u16,    // 5    18
   pub op: u8,          // 7    20
   pub vendor_id: u16,  // 8    21
   pub par: [u8; 10],   // 10   23
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct rf_packet_ll_app_t {
	pub dma_len: u32,   // 0
	pub _type: u8,      // 4
	pub rf_len: u8,     // 5
	pub l2capLen: u16,  // 6
	pub chanId: u16,    // 8
	pub opcode: u8,     // 10
	pub handle: u8,     // 11
	pub handle1: u8,    // 12
	pub app_cmd_v: app_cmd_value_t, // 13
	pub rsv: [u8; 10],
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct rf_packet_att_readRsp_t {
	pub dma_len: u32,   // 0        //won't be a fixed number as previous, should adjust with the mouse package number
	pub _type: u8,	    // 4		//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	pub rf_len: u8,		// 5		//LEN(5)_RFU(3)
	pub l2capLen: u16,  // 6
	pub chanId: u16,    // 8
	pub opcode: u8,     // 10
	pub value: [u8; 22] // 11
}

#[repr(C, align(4))]
pub struct rf_packet_ll_init_t {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,				//RA(1)_TA(1)_RFU(2)_TYPE(4): connect request PDU
    pub rf_len: u8,				//LEN(6)_RFU(2)
    pub scanA: [u8; 6],			//
    pub advA: [u8; 6],			//
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
pub struct ll_packet_l2cap_data_t {
    pub l2capLen: u16,
    pub chanId: u16,
    pub opcode: u8,
    pub handle: u8,
    pub handle1: u8,
    pub value: [u8; 30],
}

#[repr(C, align(4))]
pub struct rf_pkt_l2cap_sig_connParaUpRsp_t {
    pub dma_len: u32,
    pub _type: u8,
    pub rf_len: u8,
    pub l2capLen: u16,
    pub chanId: u16,
    pub code: u8,
    pub id: u8,
    pub dataLen: u16,
    pub result: u16,
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct rf_packet_att_write_t {
    pub dma_len: u32,
    pub rtype: u8,
    pub rf_len: u8,
    pub l2capLen: u16,
    pub chanId: u16,
    pub opcode: u8,
    pub handle: u8,
    pub handle1: u8,
    pub value: [u8; 30], //sno[3],src[2],dst[2],op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct rf_packet_att_cmd_t {
    pub dma_len: u32,   // 0
    pub _type: u8,      // 4
    pub rf_len: u8,     // 5
    pub l2capLen: u16,  // 6
    pub chanId: u16,    // 8
    pub opcode: u8,     // 10
    pub handle: u8,     // 11
    pub handle1: u8,    // 12
    pub value: [u8; 30], //sno[3],src[2],dst[2],op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct rf_packet_att_data_t {
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
pub struct mesh_pkt_t {
	pub dma_len: u32,           // 0
	pub _type: u8,              // 4
	pub rf_len: u8,             // 5
	pub l2capLen: u16,          // 6
	pub chanId: u16,            // 8
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

#[repr(C, align(4))]
pub struct rf_packet_att_mtu_t {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub	_type: u8,				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
    pub rf_len: u8,				//LEN(5)_RFU(3)
    pub	l2capLen: u16,
    pub	chanId: u16,
    pub opcode: u8,
    pub mtu: [u8; 2]
}

#[repr(C, align(4))]
pub struct rf_packet_att_errRsp_t {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
    pub rf_len: u8,				//LEN(5)_RFU(3)
    pub l2capLen: u16,
    pub chanId: u16,
    pub opcode: u8,
    pub errOpcode: u8,
    pub errHandle: u16,
    pub errReason: u8
}

#[repr(C, align(4))]
pub struct rf_packet_ll_data_rsp_t {
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
pub struct rf_packet_l2cap_head_t {
	pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
	pub	_type: u8,				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	pub rf_len: u8,				//LEN(6)_RFU(2)
	pub	l2capLen: u16,
	pub	chanId: u16
}

#[repr(C, align(4))]
pub struct rf_packet_version_ind_t {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,				//RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,				//LEN(6)_RFU(2)
    pub opcode: u8,
    pub mainVer: u8,
    pub vendor: u16,
    pub subVer: u16
}

#[repr(C, align(4))]
pub struct rf_packet_feature_rsp_t {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,				//RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,				//LEN(6)_RFU(2)
    pub opcode: u8,
    pub data: [u8; 8]
}

#[repr(C, align(4))]
pub struct rf_packet_ctrl_unknown_t {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,				//RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,				//LEN(6)_RFU(2)
    pub opcode: u8,
    pub data: [u8; 1]
}

#[repr(C, align(4))]
pub struct rf_packet_ll_write_rsp_t {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number

    pub	_type: u8,				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
    pub rf_len: u8,				//LEN(5)_RFU(3)

    pub	l2cap: u16,				//0x17
    pub	chanid: u16,				//0x04,

    pub	op: u16
}

#[repr(C, align(4))]
pub struct rf_packet_att_writeRsp_t {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub	_type: u8,				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
    pub rf_len: u8,				//LEN(5)_RFU(3)
    pub	l2capLen: u16,
    pub	chanId: u16,
    pub opcode: u8
}

#[repr(C, packed)]
pub struct mesh_ota_dev_info_t {
    pub dev_mode: u16,
    pub no_check_dev_mode_flag_rsv: u8,
    pub rsv: [u8; 4],
}

#[repr(C, packed)]
pub struct mesh_ota_pkt_start_command_t {
    pub version: [u8; 4],
    pub dev_info: mesh_ota_dev_info_t,
} // don't modify element in it

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct status_record_t {
    pub adr: [u8; 1],
    // don't modify, use internal
    pub alarm_id: u8, // don't modify, use internal
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct rc_pkt_buf_t {
    pub op: u8,
    pub sno: [u8; 3],
    pub notify_ok_flag: bool,
    pub sno2: [u8; 2], // for passive
}

#[derive(Clone, Copy)]
#[repr(C, align(4))]
pub struct rf_packet_scan_rsp_t {
	pub dma_len: u32,       // 0     //won't be a fixed number as previous, should adjust with the mouse package number

	pub _type: u8,          // 4				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	pub rf_len: u8,			// 5	//LEN(6)_RFU(2)
	pub advA: [u8; 6],		// 6	//address
	pub data: [u8; 31]		// 12	//0-31 byte

}

#[derive(Clone, Copy)]
#[repr(C, align(4))]
pub struct light_rx_buff_t {
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
pub_mut!(pkt_read_rsp, rf_packet_att_readRsp_t, rf_packet_att_readRsp_t {
    dma_len: 0x1d,
	_type: 2,
	rf_len: 0x1b,
	l2capLen: 0x17,
	chanId: 0x4,
	opcode: 0xb,
	value: [0; 22]
});

pub_mut!(pair_rands, [u8; 8], [0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7]);
pub_mut!(pair_randm, [u8; 8], [0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7]);

/////////////// adv par define ///////////////////////////////////////////////////
pub_mut!(adv_interval2listen_interval, u16, 4);
// unit: default is 40ms, setting by 40000 from rf_link_slave_init (40000);
pub_mut!(online_status_interval2listen_interval, u16, 8);
// unit: default is 40ms, setting by 40000 from rf_link_slave_init (40000);
pub_mut!(rf_slave_ota_busy_mesh_en, u8, 0);

//if 1 send one adv packet in each interrupt

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
    [status_record_t; MESH_NODE_MAX_NUM as usize],
    [status_record_t {
        adr: [0],
        alarm_id: 0
    }; MESH_NODE_MAX_NUM as usize]
);
pub_mut!(
    slave_status_record_size,
    u16,
    size_of::<[status_record_t; MESH_NODE_MAX_NUM as usize]>() as u16
);

pub const RC_PKT_BUF_MAX: u8 = 2;
pub_mut!(
    rc_pkt_buf,
    [rc_pkt_buf_t; RC_PKT_BUF_MAX as usize],
    [rc_pkt_buf_t {
        op: 0,
        sno: [0; 3],
        notify_ok_flag: false,
        sno2: [0; 2],
    }; RC_PKT_BUF_MAX as usize]
);

pub_mut!(dev_address_next_pos, u16, 0);

pub_mut!(need_update_connect_para, bool, false);
pub const UPDATE_CONNECT_PARA_DELAY_MS: u32 = 1000;

pub_mut!(update_interval_user_max, u16, 0);
pub_mut!(update_interval_user_min, u16, 0);
pub_mut!(update_timeout_user, u16, 0);
pub const INTERVAL_THRESHOLD: u16 = 16;
pub_mut!(update_interval_flag, u16, 0);
pub_mut!(update_interval_time, bool, false);
pub const ONLINE_STATUS_COMP: u32 = 3;
pub_mut!(slave_data_valid, u32, 0);
pub_mut!(t_bridge_cmd, u32, 0);
pub_mut!(st_brige_no, u32, 0);
pub_mut!(slave_sno_sending, [u8; 3], [0; 3]);
pub_mut!(app_cmd_time, u32, 0);
pub_mut!(mesh_user_cmd_idx, u8, 0);
pub_mut!(slave_tx_cmd_time, u32, 0);
pub_mut!(blt_tx_wptr, u8, 0);
pub_mut!(slave_status_buffer_wptr, u8, 0);
pub_mut!(slave_status_buffer_rptr, u8, 0);
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
pub_mut!(slave_status_record_idx, u16, 0);
pub_mut!(notify_req_mask_idx, u8, 0);
pub_mut!(adv_flag, bool, true);
pub_mut!(online_st_flag, bool, true);
pub_mut!(slave_read_status_busy_time, u32, 0);
pub const SLAVE_READ_STATUS_BUSY_TIMEOUT: u32 = 25000;
pub_mut!(st_listen_no, u32, 0);

pub_mut!(pkt_light_adv_status, rf_packet_att_write_t, rf_packet_att_write_t {
    dma_len: 0x27,
    rtype: 2,
    rf_len: 0x25,
    l2capLen: 0x21,
    chanId: 0xffff,
    opcode: 0,
    handle: 0,
    handle1: 0,
    value: [0; 30]
});

pub_mut!(pkt_mesh, mesh_pkt_t, mesh_pkt_t {
    dma_len: 0,
    _type: 0,
    rf_len: 0,
    l2capLen: 0,
    chanId: 0,
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
pub_mut!(pkt_mesh_user_cmd_buf, mesh_pkt_t, mesh_pkt_t {
    dma_len: 0,
    _type: 0,
    rf_len: 0,
    l2capLen: 0,
    chanId: 0,
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
pub_mut!(pkt_init, rf_packet_ll_init_t, rf_packet_ll_init_t {
    dma_len: 0x24,
    _type: 0x5,
    rf_len: 0x22,
    scanA: [0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5],
    advA: [0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5],
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
