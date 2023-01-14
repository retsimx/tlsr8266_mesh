// Stuff from the libble library

use std::mem;
use std::mem::{MaybeUninit, size_of, size_of_val};
use common::{MESH_NODE_MAX_NUM, PAIR_STATE};
use ::{no_mangle_fn, no_mangle_fn_def};
use pub_mut;
use sdk::factory_reset::CFG_ADR_MAC_512K_FLASH;


no_mangle_fn!(light_set_tick_per_us, tick: u32);

no_mangle_fn!(mesh_security_enable, en: bool);
no_mangle_fn!(mesh_get_fw_version);

no_mangle_fn!(register_mesh_ota_master_ui, p: fn(*const u8));

no_mangle_fn!(setSppUUID, p_service_uuid: *const u8, p_data_s2c_uuid: *const u8, p_data_c2s_uuid: *const u8, p_data_ota_uuid: *const u8, p_data_pair_uuid: *const u8);

no_mangle_fn!(vendor_id_init, vendor_id: u16);

no_mangle_fn!(is_receive_ota_window, bool);
no_mangle_fn!(rf_link_slave_proc);
no_mangle_fn!(is_add_packet_buf_ready, bool);
no_mangle_fn!(rf_link_add_tx_packet, bool, p: *const u8);  // return value: 1 success,  0 faile)d
no_mangle_fn!(rf_link_slave_read_status_stop);
no_mangle_fn!(rf_link_data_callback,p: *const u8);
no_mangle_fn!(light_sw_reboot);
no_mangle_fn!(rf_ota_save_data, OtaState, data: *const u8);

no_mangle_fn!(pairRead, u32, p: *const u8);
no_mangle_fn!(pairWrite, u32, p: *const u8);
no_mangle_fn!(pair_save_key);
no_mangle_fn!(pair_load_key);
no_mangle_fn!(encode_password, pd: *mut u8);
no_mangle_fn!(decode_password, pd: *mut u8);
no_mangle_fn!(access_code, u32, p_name: *const u8, p_pw: *const u8);

no_mangle_fn!(is_master_sending_ota_st, bool);
no_mangle_fn!(mesh_ota_slave_set_response, bool, params: *mut u8, rtype: u8);
no_mangle_fn!(mesh_ota_timeout_handle, op: u8, params: *const u8);
no_mangle_fn!(mesh_ota_master_start, adr: *const u8, len: u32, p_dev_info: *const mesh_ota_dev_info_t);
no_mangle_fn!(is_master_ota_st, bool);
no_mangle_fn!(is_mesh_ota_slave_running, bool);
no_mangle_fn!(mesh_ota_slave_reboot_delay);
no_mangle_fn!(mesh_ota_slave_save_data, bool, params: *const u8);
no_mangle_fn!(mesh_ota_master_cancle, reset_flag: u8, complete: bool);
no_mangle_fn!(mesh_ota_master_start_firmware_from_backup);
no_mangle_fn!(mesh_push_user_command, u32, sno: u32, dst: u16, p: *const u8, len: u8);
no_mangle_fn!(mesh_node_init);
no_mangle_fn!(mesh_report_status_enable, mask: u8);
no_mangle_fn!(mesh_report_status_enable_mask, val: *const u8, len: u16);

no_mangle_fn!(get_fw_version, ver: *const u8);

no_mangle_fn!(irq_light_slave_handler);

// param st is 2bytes = lumen% + rsv(0xFF)  // rf pkt : device_address+sn+lumen+rsv);
no_mangle_fn!(ll_device_status_update, st_val_par: *const u8, len: u8);

no_mangle_fn!(setup_ble_parameter_start, u32, delay: u16, interval_min: u16, interval_max: u16, timeout: u16);

no_mangle_fn!(rf_drv_init, mode: u32);

pub_mut!(pair_config_valid_flag, u8);
pub_mut!(pair_config_mesh_name, [u8; 17]);
pub_mut!(pair_config_mesh_pwd, [u8; 17]);
pub_mut!(pair_config_mesh_ltk, [u8; 17]);

pub_mut!(p_adv_pri_data, *const ll_adv_private_t);
pub_mut!(p_adv_rsp_data, *const ll_adv_rsp_private_t);
pub_mut!(adv_private_data_len, u8);

pub_mut!(online_status_timeout, u32);

pub_mut!(security_enable, bool);
pub_mut!(pair_login_ok, bool);
pub_mut!(slave_first_connected_tick, u32);

pub_mut!(device_address, u16);
pub_mut!(max_relay_num, u8);

pub_mut!(group_address, [u16; MAX_GROUP_NUM as usize]);

pub_mut!(slave_p_mac, *const u8);

pub_mut!(adr_flash_cfg_idx, u32);

pub_mut!(slave_link_connected, bool);

pub_mut!(pkt_light_notify, rf_packet_att_cmd_t);

pub_mut!(slave_read_status_busy, bool);
pub_mut!(rf_slave_ota_busy, bool);

pub_mut!(pair_setting_flag, PAIR_STATE);
pub_mut!(pair_nn, [u8; 16]);
pub_mut!(pair_pass, [u8; 16]);
pub_mut!(pair_ltk, [u8; 16]);
pub_mut!(pair_ltk_mesh, [u8; 16]);
pub_mut!(pair_ac, u32);

pub_mut!(cur_ota_flash_addr, u32);
pub_mut!(mesh_ota_master_100_flag, u8);
pub_mut!(rf_slave_ota_finished_flag, OtaState);
pub_mut!(rf_slave_ota_terminate_flag, bool);
pub_mut!(app_ota_hci_type, APP_OTA_HCI_TYPE);
pub_mut!(mesh_node_max, u8);

pub_mut!(rf_slave_ota_timeout_def_s, u16);
pub_mut!(rf_slave_ota_timeout_s, u16);

pub_mut!(tick_per_us, u32);

pub_mut!(user_data, [u8; 16]);
pub_mut!(user_data_len, u8);


pub const PMW_MAX_TICK_BASE: u16 = 255;
// must 255
pub const PMW_MAX_TICK_MULTI: u16 = 209;
// 209: freq = 600.4Hz
pub const PMW_MAX_TICK_1: u16 = PMW_MAX_TICK_BASE * PMW_MAX_TICK_MULTI;
// must less or equal than (255*256)
pub const PMW_MAX_TICK: u16 = PMW_MAX_TICK_1;

pub const BRIDGE_MAX_CNT: u32 = 0;
pub const IRQ_TIMER1_ENABLE: bool = true;
pub const IRQ_TIME1_INTERVAL: u8 = 10;

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
pub const CMD_START_MESH_OTA: u16 = 0xffff;//app use  	// use in rf_link_data_callback()

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
pub const GET_MESH_OTA: u8 = 9;    //


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
pub const ON_OFF_FROM_PASSIVE_DEV_ALT: u8 = 0x03;    // tx command: alter from passive device command

pub const LIGHT_SYNC_REST_PARAM: u8 = 0x02;

pub const LIGHT_ADD_GRP_PARAM: u8 = 0x01;
pub const LIGHT_DEL_GRP_PARAM: u8 = 0x00;

pub const DEVICE_ADDR_MASK_DEFAULT: u16 = 0x7FFF;

pub enum LightOpType {
    op_type_1 = 1,
    op_type_2 = 2,
    op_type_3 = 3,
}

#[derive(PartialEq)]
pub enum APP_OTA_HCI_TYPE {
    GATT_ONLY = 0,
    MESH = 1,
}

#[derive(PartialEq)]
#[derive(Clone, Copy)]
pub enum OtaState {
    CONTINUE = 0,
    // must zero
    OK = 1,
    ERROR = 2,
    MASTER_OTA_REBOOT_ONLY = 3,
}

#[derive(PartialEq)]
pub enum MESH_OTA_LED {
    OK,
    ERROR,
    STOP,
}

pub enum CMD_TYPE {
    NORMAL = 0,
    PASSIVE_DEV = 1,
    PASSIVE_DEV_ALT = 2,
}

#[repr(C, packed)]
pub struct ll_adv_private_t {
    pub ManufactureID: u16,
    // must vendor id to follow spec
    pub MeshProductUUID: u16,
    pub MacAddress: u32,// low 4 byte
}

#[repr(C, packed)]
pub struct ll_adv_rsp_private_t {
    pub ManufactureID: u16,
    // must vendor id to follow spec
    pub MeshProductUUID: u16,
    pub MacAddress: u32,
    // low 4 byte
    pub ProductUUID: u16,
    pub status: u8,
    pub DeviceAddress: [u8; 2],
    pub rsv: [u8; 16],
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
    pub value: [u8; 30],
}

#[repr(C, packed)]
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

#[repr(C, packed)]
pub struct rf_packet_att_write_t {
    dma_len: u32,
    pub rtype: u8,
    pub rf_len: u8,
    pub l2capLen: u16,
    pub chanId: u16,
    pub opcode: u8,
    pub handle: u8,
    pub handle1: u8,
    pub value: u8,//sno[3],src[2],dst[2],op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
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
    pub value: [u8; 30], //sno[3],src[2],dst[2],op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
}

#[repr(C, packed)]
pub struct rf_packet_att_data_t {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number

    pub _type: u8,
    //RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
    pub rf_len: u8,
    //LEN(5)_RFU(3)
    pub l2cap: u16,
    //0x17
    pub chanid: u16,                //0x04,

    pub att: u8,
    //0x12 for master; 0x1b for slave// as ttl when relay
    pub hl: u8,
    // assigned by master
    pub hh: u8,                    //

    pub dat: [u8; 23],
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
}  // don't modify element in it

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct status_record_t {
    adr: [u8; 1],
    // don't modify, use internal
    alarm_id: u8,    // don't modify, use internal
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct rc_pkt_buf_t {
    pub op: u8,
    pub sno: [u8; 3],
    pub notify_ok_flag: u8,
    pub sno2: [u8; 2],   // for passive
}

#[inline(always)]
pub unsafe fn is_unicast_addr(p_addr: *const u8) -> bool
{
    return (*p_addr.offset(1) & 0x80) == 0;
}

// required callback fns
no_mangle_fn_def!(gpio_irq_user_handle);
no_mangle_fn_def!(gpio_risc0_user_handle);
no_mangle_fn_def!(gpio_risc1_user_handle);
no_mangle_fn_def!(gpio_risc2_user_handle);

// Shit required for linking
/////////////// password encode sk initial  ///////////////////////////////////////////////////
pub_mut!(pair_config_pwd_encode_sk, [u8; 17], [0; 17]);
pub_mut!(pair_config_pwd_encode_enable, u8, 1);
pub_mut!(auth_code, [u8; 4], [0x01, 0x02, 0x03, 0x04]);
pub_mut!(auth_code_en, u8, 0);
pub_mut!(tx_packet_bridge_random_en, u8, 0);

/////////////// adv par define ///////////////////////////////////////////////////
pub_mut!(adv_interval2listen_interval, u16, 4);
// unit: default is 40ms, setting by 40000 from rf_link_slave_init (40000);
pub_mut!(online_status_interval2listen_interval, u16, 8);
// unit: default is 40ms, setting by 40000 from rf_link_slave_init (40000);
pub_mut!(rf_slave_ota_busy_mesh_en, u8, 0);

/////////////// for passive switch ///////////////////////////////////////////////
pub_mut!(separate_ADVpkt, u8, 0);
//if 1 send one adv packet in each interrupt
pub_mut!(mesh_chn_amount, u8, 4);                //amount of sys_chn_listen

// Scene shit needed to link
pub_mut!(pkt_mesh_scene_rsp, [u8; 1], [0]);

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
pub_mut!(pkt_mesh_alarm_rsp, [u8; 1], [0]);

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
{}

#[no_mangle]
fn is_need_sync_time() -> bool {
    return false;
}

#[no_mangle]
fn rtc_set_time(rtc_set: u32) -> i32 {
    return -1;
}

no_mangle_fn_def!(alarm_event_check);
no_mangle_fn_def!(mesh_send_alarm_time);

// dual mode shit needed to link
#[no_mangle]
fn dual_mode_rx_sig_beacon_proc(p: *const u8, t: u32) -> u32 { return 0; }

#[no_mangle]
fn get_gatt_adv_cnt() -> u8 { return 3; }

no_mangle_fn_def!(dual_mode_channel_ac_set_with_check_TLK);
no_mangle_fn_def!(dual_mode_check_and_select_disconnect_cb);

#[no_mangle]
fn get_sig_mesh_adv() -> u32 { return 0; }

#[no_mangle]
fn dual_mode_channel_ac_proc(connect_st: u32) {}

#[no_mangle]
fn tlk_mesh_access_code_backup(ac: u32) {}

#[no_mangle]
fn dual_mode_select(sdk_rf_mode: u32) {}

// flash mesh extend shit needed to link
pub const CFG_ADR_CALIBRATION_512K_FLASH: u32 = CFG_ADR_MAC_512K_FLASH + 0x10;
// don't change
pub const CFG_SECTOR_ADR_CALIBRATION_CODE: u32 = CFG_ADR_CALIBRATION_512K_FLASH;
pub_mut!(flash_sector_calibration, u32, CFG_SECTOR_ADR_CALIBRATION_CODE);

no_mangle_fn_def!(mesh_ota_start_unprotect_flash);

// common shit needed to link
#[no_mangle]
fn forced_single_cmd_in_ble_interval_handle(ph: *const u8) {}

no_mangle_fn_def!(mesh_node_keep_alive_other);

#[no_mangle]
fn rx_mesh_adv_message_cb(p: *const u8, mac_match: u32) {}

no_mangle_fn_def!(sensor_enter_deep_cb);

pub_mut!(sensor_enable, u8, 0);
pub_mut!(sensor_last_adv_sleep_time_us, u16, 1200);
// us
pub_mut!(adv_uuid_flag, u8, 0);
pub_mut!(adv_uuid, [u8; 4], [0x03, 0x02, 0xAB, 0xCD]);
pub_mut!(passive_en, u8, 0);

#[no_mangle]
fn get_command_type(p_att_value: *const u8) -> CMD_TYPE { return CMD_TYPE::NORMAL; }

// Note: par[8], par[9] of passive command have been used internal for sno2.
#[no_mangle]
fn set_command_type2alt(p_att_value: *const u8) {}

pub_mut!(cb_mesh_node_filter, u32, 0);

#[no_mangle]
fn proc_sig_mesh_to_telink_mesh() -> u8 { return 0; }

#[no_mangle]
fn cb_set_sub_addr_tx_cmd(src: *const u8, sub_adr: u16) {}

#[no_mangle]
fn rssi_online_status_pkt_cb(p_node_st: *const u8, rssi: u8, online_again: u32) {}

pub_mut!(p_cb_rx_from_mesh, u32, 0);

#[no_mangle]
fn is_bridge_task_busy() -> bool { return false; }

#[no_mangle]
fn mesh_ota_third_complete_cb(calibrate_flag: u32) {}

pub_mut!(mesh_ota_third_fw_flag, u8, 0);

#[no_mangle]
fn mesh_ota_set_start_par_user(p: *const mesh_ota_pkt_start_command_t) {}

pub_mut!(p_cb_pair_failed, u32, 0);
pub_mut!(p_cb_ble_slave_disconnect, u32, 0);

no_mangle_fn_def!(rf_link_slave_connect_callback);

pub_mut!(work_sleep_en, u8, 0);
pub_mut!(start2adv, u32, 0);
//us
pub_mut!(iBeaconInterval, u8, 0);
pub_mut!(beacon_with_mesh_adv, u8, 0);

// 0 means only send beacon adv pkt;  1 means send both of beacon pkt and mesh adv pkt
#[no_mangle]
fn pa_txrx(val: u8) {}

#[no_mangle]
fn pa_init(tx_pin_level: u8, rx_pin_level: u8) {}

pub_mut!(slave_status_record, [status_record_t; MESH_NODE_MAX_NUM as usize], [status_record_t { adr: [0], alarm_id: 0 }; MESH_NODE_MAX_NUM as usize]);
pub_mut!(slave_status_record_size, u16, size_of::<[status_record_t; MESH_NODE_MAX_NUM as usize]>() as u16);
pub_mut!(SW_Low_Power, u8, 0);
pub_mut!(SW_Low_Power_rsp_flag, u8, 0);
pub_mut!(mesh_ota_only_calibrate_type1, u8, 0);
const RC_PKT_BUF_MAX: u8 = 2;
pub_mut!(rc_pkt_buf, [rc_pkt_buf_t; RC_PKT_BUF_MAX as usize], [rc_pkt_buf_t {
    op: 0,
    sno: [0; 3],
    notify_ok_flag: 0,
    sno2: [0; 2],
}; RC_PKT_BUF_MAX as usize]);

pub_mut!(mesh_cmd_cache_num, u8, RC_PKT_BUF_MAX);
pub_mut!(device_address_mask, u16, DEVICE_ADDR_MASK_DEFAULT);

#[no_mangle]
fn fn_rx_push_to_cache(p: *const u8) {}