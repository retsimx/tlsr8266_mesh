// Stuff from the libble library

use common::PAIR_STATE;
use MESH_PWD_ENCODE_SK;

extern "C" {
    pub fn light_set_tick_per_us(tick: u32);

    pub fn mesh_security_enable(en: bool);
    pub fn mesh_get_fw_version();

    pub fn register_mesh_ota_master_ui(p: fn(*const u8));

    pub fn setSppUUID(p_service_uuid: *const u8, p_data_s2c_uuid: *const u8, p_data_c2s_uuid: *const u8, p_data_ota_uuid: *const u8, p_data_pair_uuid: *const u8);

    pub fn vendor_id_init(vendor_id: u16);

    pub fn is_receive_ota_window() -> bool;
    pub fn rf_link_slave_proc();
    pub fn is_add_packet_buf_ready() -> bool;
    pub fn rf_link_add_tx_packet(p: *const u8) -> bool;  // return value: 1 success,  0 failed

    pub fn light_sw_reboot();

    pub fn pair_save_key();
    pub fn pair_load_key();
    pub fn encode_password(pd: *mut u8);
    pub fn decode_password(pd: *mut u8);
    pub fn access_code(p_name: *const u8, p_pw: *const u8) -> u32;

    pub fn mesh_ota_slave_set_response(params: *mut u8, rtype: u8) -> bool;
    pub fn mesh_ota_timeout_handle(op: u8, params: *const u8);
    pub fn mesh_ota_master_start_firmware_from_own();
    pub fn is_master_ota_st() -> bool;
    pub fn is_mesh_ota_slave_running() -> bool;
    pub fn mesh_ota_slave_reboot_delay();
    pub fn mesh_ota_slave_save_data(params: *const u8) -> bool;
    pub fn mesh_ota_master_cancle(reset_flag: u8, complete: bool);
    pub fn mesh_push_user_command(sno: u32, dst: u16, p: *const u8, len: u8) -> u32;
    pub fn mesh_node_init();

    pub fn irq_light_slave_handler();

    // param st is 2bytes = lumen% + rsv(0xFF)  // rf pkt : device_address+sn+lumen+rsv;
    pub fn ll_device_status_update(st_val_par: *const u8, len: u8);

    pub static mut user_data: [u8; 16];
    pub static mut user_data_len: u8;
    pub static mut pair_config_valid_flag: u8;

    pub static mut pair_config_mesh_name: [u8; 17];
    pub static mut pair_config_mesh_pwd: [u8; 17];
    pub static mut pair_config_mesh_ltk: [u8; 17];

    pub static mut p_adv_pri_data: *const ll_adv_private_t;
    pub static mut p_adv_rsp_data: *const ll_adv_rsp_private_t;
    pub static mut adv_private_data_len: u8;

    pub static mut online_status_timeout: u32;

    pub static mut security_enable: bool;
    pub static mut pair_login_ok: bool;
    pub static mut slave_first_connected_tick: u32;

    pub static device_address: u16;
    pub static max_relay_num: u8;

    pub static group_address: [u16; MAX_GROUP_NUM as usize];

    pub static mut slave_p_mac: *const u8;
    pub static mut get_mac_en: bool;

    pub static mut adr_flash_cfg_idx: u32;

    pub static mut slave_link_connected: bool;

    pub static mut pkt_light_notify: rf_packet_att_cmd_t;

    pub static mut rf_slave_ota_busy: bool;

    pub static mut pair_setting_flag: PAIR_STATE;
    pub static mut pair_nn: [u8; 16];
    pub static mut pair_pass: [u8; 16];
    pub static mut pair_ltk: [u8; 16];
    pub static mut pair_ltk_mesh: [u8; 16];
    pub static mut pair_ac: u32;

    pub static mut mesh_ota_master_100_flag: u8;
    pub static mut mesh_node_max: u8;
}

pub const PMW_MAX_TICK_BASE	 : u16 =           255;   // must 255
pub const PMW_MAX_TICK_MULTI	 : u16 =           209;   // 209: freq = 600.4Hz
pub const PMW_MAX_TICK_1		 : u16 =           PMW_MAX_TICK_BASE*PMW_MAX_TICK_MULTI;  // must less or equal than (255*256)
pub const PMW_MAX_TICK		     : u16 =           PMW_MAX_TICK_1;

pub const BRIDGE_MAX_CNT: u32 = 0;
pub const IRQ_TIMER1_ENABLE: bool = true;
pub const IRQ_TIME1_INTERVAL: u8 = 10;

pub const ONLINE_STATUS_TIMEOUT: u32 = 3000;

pub const AUTH_TIME: u32 = 60;
pub const MAX_GROUP_NUM: u8 = 8;


// op cmd 11xxxxxxzzzzzzzzzzzzzzzz z's=VENDOR_ID  xxxxxx=LGT_CMD_
pub const         LGT_CMD_READ_SCENE             :u8 = 0x00;//
pub const         LGT_CMD_SCENE_RSP              :u8 = 0x01;//
pub const         LGT_CMD_NOTIFY_MESH			   :u8 = 0x02;//use between mesh
pub const		    LGT_CMD_SW_DATA				   :u8 = 0x03;
pub const		    LGT_CMD_SW_RSP				   :u8 = 0x04;
pub const		    LGT_CMD_TIME_SYNC			:u8 =	0x05;//internal use
pub const		    LGT_CMD_MESH_OTA_DATA		:u8 =	0x06;//internal use
pub const             CMD_OTA_FW_VERSION		:u16 =			0xff00;          // must larger than 0xff00
pub const             CMD_OTA_START			:u16 =			0xff01;
pub const             CMD_OTA_END				:u16 =			0xff02;
pub const             CMD_STOP_MESH_OTA		:u16 =			0xfffe;//app use
pub const             CMD_START_MESH_OTA		:u16 =			0xffff;//app use  	// use in rf_link_data_callback()

pub const		    LGT_CMD_MESH_OTA_READ		:u8 =	0x07;//internal use
pub const		        PAR_READ_VER			   :u8 =     0x00;//internal use
pub const		        PAR_READ_END			   :u8 =     0x01;//internal use
pub const		        PAR_READ_MAP			   :u8 =     0x02;//internal use
pub const		        PAR_READ_CALI			   :u8 =     0x03;//internal use
pub const		        PAR_READ_PROGRESS		:u8 =	    0x04;//internal use
pub const		        PAR_APP_READ_ST			   :u8 =     0x05;//app use: 0 : idle; 1: slave mode; 2: master mode
pub const             PAR_APP_OTA_HCI_TYPE_SET:u8 =		0x06;//app use
pub const		        PAR_READ_MESH_PAIR_CONFIRM:u8 =		0x0a;
pub const		    LGT_CMD_MESH_OTA_READ_RSP	:u8 =	0x08;//internal use
pub const         LGT_CMD_MESH_PAIR              :u8 = 0x09;
pub const         LGT_CMD_MESH_CMD_NOTIFY       :u8 =	0x0a;
pub const             CMD_NOTIFY_MESH_PAIR_END:u8 =		0x00;
pub const         LGT_CMD_FRIENDSHIP       	   :u8 = 0x0b;
pub const             CMD_CTL_POLL		       :u8 =     0x01;
pub const             CMD_CTL_UPDATE		       :u8 =     0x02;
pub const             CMD_CTL_REQUEST		       :u8 =     0x03;
pub const             CMD_CTL_OFFER		       :u8 =     0x04;
pub const             CMD_CTL_GROUP_REPORT	:u8 =	    0x05;
pub const             CMD_CTL_GROUP_REPORT_ACK:u8 =		0x06;

pub const			LGT_CMD_LIGHT_ONOFF			:u8 =	0x10;
pub const			LGT_CMD_LIGHT_ON			:u8 =	0x10;
pub const			LGT_CMD_LIGHT_OFF			:u8 =	0x11;//internal use
pub const			LGT_CMD_LIGHT_SET			:u8 =	0x12;//set lumen
pub const			LGT_CMD_SWITCH_CONFIG	       :u8 = 0x13;//internal use
pub const         LGT_CMD_LIGHT_GRP_RSP1         :u8 = 0x14;//get group rsp: 8groups low 1bytes
pub const         LGT_CMD_LIGHT_GRP_RSP2         :u8 = 0x15;//get group rsp: front 4groups 2bytes
pub const         LGT_CMD_LIGHT_GRP_RSP3         :u8 = 0x16;//get group rsp: behind 4groups 2bytes
pub const 		LGT_CMD_LIGHT_CONFIG_GRP 	:u8 =	0x17;//add or del group
pub const			LGT_CMD_LUM_UP				:u8 =	0x18;//internal use
pub const			LGT_CMD_LUM_DOWN			:u8 =	0x19;//internal use
pub const 		LGT_CMD_LIGHT_READ_STATUS 	:u8 =	0x1a;//get status req
pub const 		LGT_CMD_LIGHT_STATUS 		:u8 =	0x1b;//get status rsp
pub const 		LGT_CMD_LIGHT_ONLINE 		:u8 =	0x1c;//get online status req//internal use
pub const         LGT_CMD_LIGHT_GRP_REQ          :u8 = 0x1d;//get group req
pub const			LGT_CMD_LEFT_KEY			:u8 =	0x1e;//internal use
pub const			LGT_CMD_RIGHT_KEY			:u8 =	0x1f;//internal use
pub const			LGT_CMD_CONFIG_DEV_ADDR        :u8 = 0x20;//add device address
pub const		        DEV_ADDR_PAR_ADR		:u8 =		0x00;
pub const		        DEV_ADDR_PAR_WITH_MAC	:u8 =		0x01;
pub const         LGT_CMD_DEV_ADDR_RSP           :u8 = 0x21;//rsp
pub const         LGT_CMD_SET_RGB_VALUE          :u8 = 0x22;//params[0]:1=R 2=G 3=B 4=RGB params[1~3]:R/G/B value 0~255  params[0]:5=CT params[1]=value 0~100%
pub const         LGT_CMD_KICK_OUT               :u8 = 0x23;//
pub const         LGT_CMD_SET_TIME               :u8 = 0x24;//
pub const         LGT_CMD_ALARM                  :u8 = 0x25;//
pub const         LGT_CMD_READ_ALARM             :u8 = 0x26;//
pub const         LGT_CMD_ALARM_RSP              :u8 = 0x27;//
pub const         LGT_CMD_GET_TIME               :u8 = 0x28;//
pub const         LGT_CMD_TIME_RSP               :u8 = 0x29;//
pub const         LGT_CMD_USER_NOTIFY_REQ        :u8 = 0x2a;//
pub const         LGT_CMD_USER_NOTIFY_RSP        :u8 = 0x2b;//
pub const 		LGT_CMD_SET_SW_GRP			:u8 =	0x2c;
pub const         LGT_CMD_LIGHT_RC_SET_RGB       :u8 = 0x2d;
pub const         LGT_CMD_SET_SCENE              :u8 = 0x2e;
pub const         LGT_CMD_LOAD_SCENE             :u8 = 0x2f;
pub const         LGT_CMD_SET_LIGHT              :u8 = 0x30;


pub const GET_STATUS      : u8 = 0;
pub const GET_GROUP1      : u8 = 1;// return 8 group_address(low 1byte)
pub const GET_GROUP2      : u8 = 2;// return front 4 group_address
pub const GET_GROUP3      : u8 = 3;// return behind 4 group_address
pub const GET_DEV_ADDR    : u8 = 4;// return device address
pub const GET_ALARM       : u8 = 5;// return alarm info
pub const GET_TIME        : u8 = 6;// return time info
pub const GET_USER_NOTIFY : u8 = 7;// return user notify info
pub const GET_SCENE		  : u8   = 8;	//
pub const GET_MESH_OTA	  : u8   = 9;	//


pub const	LGT_CMD_FRIEND_SHIP_OK		    : u8 = 0xb0;
pub const	LGT_CMD_FRIEND_SHIP_DISCONNECT  : u8 = 0xb1;
pub const	LGT_CMD_PAIRING_LIGHT_OFF		: u8 = 0xc1;
pub const	LGT_CMD_PAIRING_LIGHT_ON		: u8 = 0xc2;
pub const	LGT_CMD_PAIRING_LIGHT_SEL		: u8 = 0xc3;
pub const	LGT_CMD_PAIRING_LIGHT_CONFIRM	: u8 = 0xc4;
pub const	LGT_CMD_SET_MESH_INFO           : u8 = 0xc5;
pub const	LGT_CMD_SET_DEV_ADDR            : u8 = 0xc6;
pub const	LGT_CMD_DEL_PAIR                : u8 = 0xc7;
pub const	LGT_CMD_PAIRING_LIGHT_RESPONSE	: u8 = 0xc9;
pub const	LGT_CMD_PAIRING_LIGHT_OTA_EN	: u8 = 0xce;
pub const	LGT_CMD_MESH_PAIR_TIMEOUT		: u8 = 0xcf;
pub const	LGT_CMD_DUAL_MODE_MESH		    : u8 = 0xd0;

pub const     LIGHT_OFF_PARAM                 : u8 = 0x00;
pub const    LIGHT_ON_PARAM                  : u8 = 0x01;
pub const        ON_OFF_NORMAL               : u8 =     0x00;
pub const        ON_OFF_FROM_OTA             : u8 =     0x01;
pub const        ON_OFF_FROM_PASSIVE_DEV     : u8 =     0x02;    // sent from passive device
pub const        ON_OFF_FROM_PASSIVE_DEV_ALT : u8 =     0x03;    // tx command: alter from passive device command

pub const     LIGHT_SYNC_REST_PARAM           : u8 = 0x02;

pub const     LIGHT_ADD_GRP_PARAM             : u8 = 0x01;
pub const    LIGHT_DEL_GRP_PARAM             : u8 = 0x00;

pub enum LightOpType {
    op_type_1 = 1,
    op_type_2 = 2,
    op_type_3 = 3,
}

pub enum OtaState {
    OTA_STATE_CONTINU = 0,  // must zero
    OTA_STATE_OK = 1,
    OTA_STATE_ERROR = 2,
    OTA_STATE_MASTER_OTA_REBOOT_ONLY = 3,
}


#[repr(C, packed)]
pub struct ll_adv_private_t {
    pub ManufactureID: u16,  // must vendor id to follow spec
    pub MeshProductUUID: u16,
    pub MacAddress: u32// low 4 byte
}

#[repr(C, packed)]
pub struct ll_adv_rsp_private_t {
    pub ManufactureID: u16,  // must vendor id to follow spec
    pub MeshProductUUID: u16,
    pub MacAddress: u32,// low 4 byte
    pub ProductUUID: u16,
    pub status: u8,
    pub DeviceAddress: [u8; 2],
    pub rsv: [u8; 16]
}

#[repr(C, packed)]
pub struct rf_packet_att_value_t {
    pub sno: [u8; 3],
    pub src: [u8; 2],
    pub dst: [u8; 2],
    pub val: [u8; 23],// op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
    // get status req: params[0]=tick  mac-app[2-3]=src-mac1...
    // get status rsp: mac-app[0]=ttc  mac-app[1]=hop-count
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct ll_packet_l2cap_data_t {
	pub l2capLen: u16,
	pub chanId: u16,
	pub opcode: u8,
	pub handle: u8,
	pub handle1: u8,
	pub value: [u8; 30]
}

#[repr(C, packed)]
pub struct rf_packet_att_cmd_t {
	pub dma_len: u32,
	pub _type: u8,
	pub rf_len: u8,
	pub l2capLen: u16,
	pub chanId: u16,
	pub opcode: u8,
	pub handle: u8,
	pub handle1: u8,
	pub value: [u8; 30] //sno[3],src[2],dst[2],op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
}

#[inline(always)]
pub unsafe fn is_unicast_addr(p_addr: *const u8) -> bool
{
    return (*p_addr.offset(1) & 0x80) == 0;
}

// required callback fns
#[no_mangle]
fn gpio_irq_user_handle() {}

#[no_mangle]
fn gpio_risc0_user_handle() {}

#[no_mangle]
fn gpio_risc1_user_handle() {}

#[no_mangle]
fn gpio_risc2_user_handle() {}

// Shit required for linking
/////////////// password encode sk initial  ///////////////////////////////////////////////////
#[no_mangle]
pub static mut pair_config_pwd_encode_sk: [u8; 17] = [0; 17];
#[no_mangle]
static mut pair_config_pwd_encode_enable: u8 = 1;
#[no_mangle]
static mut auth_code: [u8; 4] = [0x01,0x02,0x03,0x04];
#[no_mangle]
static mut auth_code_en: u8 = 0;
#[no_mangle]
static mut tx_packet_bridge_random_en: u8 = 0;

/////////////// adv par define ///////////////////////////////////////////////////
#[no_mangle]
static mut adv_interval2listen_interval: u16 = 4; // unit: default is 40ms, setting by 40000 from rf_link_slave_init (40000);
#[no_mangle]
static mut online_status_interval2listen_interval: u16 = 8; // unit: default is 40ms, setting by 40000 from rf_link_slave_init (40000);
#[no_mangle]
static mut rf_slave_ota_busy_mesh_en: u8 = 0;

/////////////// for passive switch ///////////////////////////////////////////////
#[no_mangle]
static mut separate_ADVpkt: u8 = 0;				//if 1 send one adv packet in each interrupt
#[no_mangle]
static mut mesh_chn_amount: u8 = 4;				//amount of sys_chn_listen

// Scene shit needed to link
#[no_mangle]
static mut pkt_mesh_scene_rsp: [u8; 1] = [0];

#[no_mangle]
fn is_scene_poll_notify_busy() -> u32 {
    return 0;
}

#[no_mangle]
fn is_new_scene(p_buf: *const rf_packet_att_value_t, p: *const rf_packet_att_value_t) -> u32 {
    return 1;
}

// Alarm shit needed to link
#[no_mangle]
static mut pkt_mesh_alarm_rsp: [u8; 1] = [0];

#[no_mangle]
fn is_alarm_poll_notify_busy() -> u32 {
    return 0;
}

#[no_mangle]
fn is_new_alarm(p_buf: *const rf_packet_att_value_t, p: *const rf_packet_att_value_t) -> u32 {
    return 1;
}

#[no_mangle]
fn memcopy_rtc_hhmmss(out: u32)
{
}

#[no_mangle]
fn is_need_sync_time() -> bool {
    return false;
}

#[no_mangle]
fn rtc_set_time(rtc_set: u32) -> i32 {
    return -1;
}

#[no_mangle]
fn alarm_event_check(){
}

#[no_mangle]
fn mesh_send_alarm_time (){
}

// dual mode shit needed to link
#[no_mangle]
fn dual_mode_rx_sig_beacon_proc(p: *const u8, t: u32) -> u32 {return 0}

#[no_mangle]
fn get_gatt_adv_cnt() -> u8 {return 3}

#[no_mangle]
fn dual_mode_channel_ac_set_with_check_TLK(){}

#[no_mangle]
fn dual_mode_check_and_select_disconnect_cb(){}

#[no_mangle]
fn get_sig_mesh_adv() -> u32 {return 0}

#[no_mangle]
fn dual_mode_channel_ac_proc(connect_st: u32){}

#[no_mangle]
fn tlk_mesh_access_code_backup(ac: u32){}

#[no_mangle]
fn dual_mode_select(sdk_rf_mode: u32){}