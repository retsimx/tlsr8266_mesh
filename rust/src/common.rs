use std::mem::{size_of, size_of_val};
use std::ptr::{copy_nonoverlapping};
use ::{BIT, flash_adr_pairing};
use common::RecoverStatus::FldLightOff;
use main_light::{CMD_DELAY, CMD_DELAY_MS, CMD_LEFT_DELAY_MS, device_status_update, get_buff_response, IRQ_TIMER1_CB_TIME, LED_LUM, LED_VAL, light_adjust_rgb_hw, LIGHT_OFF, light_onoff_normal, light_slave_tx_command, rf_link_light_event_callback};
use sdk::common::bit::ONES_32;
use sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};
use sdk::light::*;
use sdk::mcu::analog::{analog_read__attribute_ram_code, analog_write__attribute_ram_code};
use sdk::mcu::clock::{clock_time, clock_time_exceed, sleep_us};
use sdk::mcu::irq_i::{irq_disable, irq_restore};
use vendor_light::{get_adv_rsp_pri_data};
use std::ptr::addr_of;
use sdk::common::crc::crc16;
use sdk::mcu::register::{write_reg_rf_irq_status, FLD_RF_IRQ_MASK};
use sdk::rf_drv::rf_set_ble_access_code;
use ::{flash_adr_light_new_fw, VENDOR_ID};
use ::{OTA_LED};
use ota::FLASH_ADR_OTA_READY_FLAG;
use pub_mut;
use sdk::mcu::gpio::{AS_GPIO, gpio_set_func, gpio_set_output_en, gpio_write};
use sdk::mcu::watchdog::wd_clear;

#[no_mangle]
pub static mut mesh_pair_start_time: u32 = 0;
#[no_mangle]
pub static mut default_mesh_time: u32 = 0;
#[no_mangle]
pub static mut default_mesh_effect_delay_ref: u32 = 0;  /* When receive change to default mesh command, shall delay at least 500ms */
#[no_mangle]
pub static mut mesh_pair_start_notify_time: u32 = 0;
#[no_mangle]
pub static mut mesh_pair_retry_cnt: u8 = 0;
#[no_mangle]
pub static mut mesh_pair_notify_rsp_mask: [u8; 32] = [0; 32];
#[no_mangle]
pub static mut new_mesh_name: [u8; 16] = [0; 16];
#[no_mangle]
pub static mut new_mesh_pwd: [u8; 16] = [0; 16];
#[no_mangle]
pub static mut new_mesh_ltk: [u8; 16] = [0; 16];
#[no_mangle]
pub static mut default_mesh_time_ref: u32 = 0;
#[no_mangle]
static mut effect_new_mesh: u8 = 0;
#[no_mangle]
static mut effect_new_mesh_delay_time: u32 = 0;
#[no_mangle]
static mut mesh_pair_cmd_interval: u32 = 0;
#[no_mangle]
static mut mesh_pair_timeout: u32 = 0;
pub_mut!(mesh_pair_enable, bool, false);
#[no_mangle]
pub static mut mesh_pair_checksum: [u8; 8] = [0; 8];
#[no_mangle]
pub static mut mesh_pair_retry_max: u8 = 3;
pub_mut!(get_mac_en, bool, false);

// BEGIN SHIT LIGHT_LL HAX
#[no_mangle]
static mut mesh_node_mask: [u32; ((MESH_NODE_MAX_NUM + 31) >> 5) as usize] = [0; ((MESH_NODE_MAX_NUM + 31) >> 5) as usize];
#[no_mangle]
static mut mesh_node_max_num: u16 = MESH_NODE_MAX_NUM;
#[no_mangle]
static mut mesh_node_st_val_len: u8 = MESH_NODE_ST_VAL_LEN;
#[no_mangle]
static mut mesh_node_st_par_len: u8 = MESH_NODE_ST_PAR_LEN;
#[no_mangle]
static mut mesh_node_st_len: u8 = size_of::<mesh_node_st_t>() as u8;
// END SHIT LIGHT_LL HAX

static mut OTA_PKT_CNT: u16 = 0;
static mut OTA_RCV_LAST_IDX: u16 = 0xffff;
static mut FW_CHECK_VAL: u32 = 0;
static mut NEED_CHECK_TYPE: u8 = 0;//=1:crc val sum
static mut OTA_PKT_TOTAL: u16 = 0;
static mut SLAVE_OTA_DATA_CACHE_IDX: u8 = 0;
static mut RF_SLAVE_OTA_FINISHED_TIME: u32 = 0;
static mut TERMINATE_CNT: u8 = 0;

#[no_mangle]
static mut conn_update_successed: u8 = 0;
#[no_mangle]
static mut conn_update_cnt: u8 = 0;

const UPDATE_CONN_PARA_CNT: u8 = 4;
const CONN_PARA_DATA: [[u16; 3]; UPDATE_CONN_PARA_CNT as usize] = [[18, 18+16, 200], [16, 16+16, 200], [32, 32+16, 200], [48, 48+16, 200]];
const SYS_CHN_LISTEN_MESH: [u8; 4] = [2, 12, 23, 34];	//8, 30, 52, 74
#[no_mangle]
static mut sys_chn_listen: [u8; 4] = SYS_CHN_LISTEN_MESH;

pub static mut MESH_PAIR_TIME: u32 = 0;
pub static mut MESH_PAIR_STATE: MeshPairState = MeshPairState::MeshPairName1;
#[no_mangle]
pub static mut mesh_node_st: [mesh_node_st_t; MESH_NODE_MAX_NUM as usize] = [mesh_node_st_t{ tick: 0, val: mesh_node_st_val_t {
    dev_adr: 0,
    sn: 0,
    par: [0; MESH_NODE_ST_PAR_LEN as usize],
} }; MESH_NODE_MAX_NUM as usize];

#[derive(PartialEq)]
pub enum GatewayStatusT
{
    GatewayStatusNormal = 0,             /* Normal gateway role */
    // GatewayStatusNodeRole,              /* As node role, when pushed button */

    // GatewayStatusTempDefaltMesh,       /* In default mesh temporary */
    // GatewayStatusSwitchToDefaultMesh,
    // GatewayStatusScanUnprovDev,        /* Scanning unpair device status */
    GatewayStatusCfgUnproDev,          /* Only provision device */
    // GatewayStatusCfgCurNetwork,        /* Change current network's information */
    GatewayStatusAddConfiguredDevice,  /* Add configured device */
}

pub static mut GATEWAY_STATUS: GatewayStatusT = GatewayStatusT::GatewayStatusNormal;

pub const MESH_PAIR_CMD_INTERVAL: u32 = 500;

//unit: s
pub const MESH_PAIR_TIMEOUT: u32 = 10;
//unit: ms
pub const MESH_PAIR_NOTIFY_TIMEOUT: u32 = 2500;

pub const FW_SIZE_MAX_K : u32 =	128;
pub const ERASE_SECTORS_FOR_OTA : u32 = (FW_SIZE_MAX_K + 3) / 4;

pub const MESH_NODE_ST_VAL_LEN: u8 = 4;       // MIN: 4,   MAX: 10
pub const MESH_NODE_ST_PAR_LEN: u8 = MESH_NODE_ST_VAL_LEN - 2;

pub const REGA_LIGHT_OFF: u8 = 0x3a;

pub const MESH_NODE_MAX_NUM: u16 = 64;

pub static PKT_TERMINATE: [u8; 8] = [0x04, 0x00, 0x00, 0x00, 0x03, 0x02, 0x02, 0x13];

#[derive(Clone, Copy)]
#[repr(C, packed)]
struct mesh_node_st_val_t {
    pub dev_adr: u8,     // don't change include type
    pub sn: u8,          // don't change include type
    pub par: [u8; MESH_NODE_ST_PAR_LEN as usize]  //lumen-rsv,
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct mesh_node_st_t {
    pub tick: u16,       // don't change include type
    val: mesh_node_st_val_t
}

// recover status before software reboot
pub enum RecoverStatus {
	FldLightOff = BIT!(0),
	FldMeshOtaMaster100 = BIT!(1),
	// LowBattFlg = BIT!(2),
	// LOW_BATT_LOOP_FLG           = BIT(3),    // 0 means check by user_init, 1 means by main loop
}

#[derive(PartialEq)]
#[derive(Clone, Copy)]
pub enum PairState {
    PairSetted = 0,
    // PairSetting,
    PairSetMeshTxStart,
    PairSetMeshTxDone,// send notify req, get mesh nodes' ac
    PairSetMeshRxDone,// received all mesh nodes' ac, send cmd to switch to new mesh
}

pub fn is_ota_area_valid(adr: u32) -> bool {
	let mut buf : [u8; 4] = [0; 4];
	for i in 0..ERASE_SECTORS_FOR_OTA {
        flash_read_page(adr + i * 0x1000, 4, buf.as_mut_ptr());
    	let tmp = buf[0] as u32 | (buf[1] as u32) << 8 | (buf[2] as u32) << 16 | (buf[3] as u32) << 24;
    	if tmp != ONES_32 {
            return false;
    	}
    }
	return true;
}

pub fn erase_ota_data(adr: u32){
    for i in 0..ERASE_SECTORS_FOR_OTA {
        flash_erase_sector(adr + (ERASE_SECTORS_FOR_OTA - 1 - i) * 0x1000);
    }
}

pub fn mesh_pair_init() {
	unsafe {
		mesh_pair_enable = true;
		mesh_pair_cmd_interval = MESH_PAIR_CMD_INTERVAL;
		mesh_pair_timeout = MESH_PAIR_TIMEOUT;
	}
}

pub fn mesh_pair_proc_effect() {
	unsafe {
		if effect_new_mesh != 0 || (effect_new_mesh_delay_time != 0 && (clock_time_exceed(effect_new_mesh_delay_time, mesh_pair_cmd_interval * 1000))) {
			save_effect_new_mesh();
			effect_new_mesh = 0;
			effect_new_mesh_delay_time = 0;
		}
	}
}

pub fn mesh_ota_master_100_flag_check()
{
	let val = analog_read__attribute_ram_code(REGA_LIGHT_OFF);
	if val & RecoverStatus::FldMeshOtaMaster100 as u8 != 0 {
		unsafe { mesh_ota_master_100_flag = 1; }
		analog_write__attribute_ram_code(REGA_LIGHT_OFF, val & !(RecoverStatus::FldMeshOtaMaster100 as u8));
	}
}

#[no_mangle]
pub unsafe fn dev_addr_with_mac_flag(params: *const u8) -> bool
{
	return DEV_ADDR_PAR_WITH_MAC == *params.offset(2);
}

pub unsafe fn dev_addr_with_mac_rsp(params: *const u8, par_rsp: *mut u8) -> bool
{
	if dev_addr_with_mac_match(params) {
		*par_rsp.offset(0) = (device_address & 0xff) as u8;
		*par_rsp.offset(1) = ((device_address >> 8) & 0xff) as u8;

		copy_nonoverlapping(get_slave_p_mac_val(), par_rsp.offset( 2), 6);
		#[allow(unaligned_references)]

		copy_nonoverlapping(&get_adv_rsp_pri_data().ProductUUID, par_rsp.offset( 8) as *mut u16, 2);
		return true;
	}
	return false;
}

pub unsafe fn dev_addr_with_mac_match(params: *const u8) -> bool
{
	return if (*params.offset(0) == 0xff) && (*params.offset(1) == 0xff) {    // get
		get_mac_en
	} else {
		for i in 0..6 {
			if params.offset(4+i) != get_slave_p_mac_val().offset(i) {
				return false;
			}
		}
		return true;
	}
}

pub unsafe fn get_mesh_pair_checksum(idx: u8) -> u8 {
    let i = (idx % 8) as usize;
    return (new_mesh_name[i] ^ new_mesh_name[i+8]) ^ (new_mesh_pwd[i] ^ new_mesh_pwd[i+8]) ^ (new_mesh_ltk[i] ^ new_mesh_ltk[i+8]);
}

pub fn light_cmd_delayed_ms(data: u8) -> u16 {
	let ttc_prec = data >> 6;
	let ttc_val: u16 	= (data & 0x3F) as u16;
	return if ttc_prec == 0 { ttc_val } else {
		if ttc_prec == 1 { ttc_val << 2 } else {
			if ttc_prec == 2 { ttc_val << 4 } else {
				if ttc_prec == 3 { ttc_val << 8 } else { 0 }
			}
		}
	}
}

pub unsafe fn mesh_pair_proc_get_mac_flag(){
	get_mac_en = false; 	// set success
	if mesh_pair_enable {
		let mut data: [u8; 1] = [0];
		flash_write_page(flash_adr_pairing + adr_flash_cfg_idx + 1, 1, data.as_mut_ptr());
		if data[0] == 1 {get_mac_en = true} else {get_mac_en = false}
	}
}

#[repr(C, packed)]
pub struct light_step_t {
	pub time: u32,
	pub lum_temp: i32,
	pub lum_dst: i32,
	pub step: u16,
	pub step_mod: u16,
	pub remainder: u16,
	pub adjusting_flag: bool
}

static mut LIGHT_STEP: light_step_t = light_step_t {
	time: 0,
	lum_temp: 0,
	lum_dst: 0,
	step: 0,
	step_mod: 0,
	remainder: 0,
	adjusting_flag: false,
};

const LUM_UP: u8 = 0;
const LUM_DOWN: u8 = 1;

const LIGHT_ADJUST_TIME     : u16 = 100;   //unit: 10ms
const LIGHT_ADJUST_INTERVAL : u16 = 2;  // unit :10ms;     min:20ms

unsafe fn get_step(direction: u8){
    LIGHT_STEP.remainder = 0;       // reset
    if LUM_UP == direction {
        LIGHT_STEP.step = ((LIGHT_STEP.lum_dst - LIGHT_STEP.lum_temp)/((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32)) as u16;
        LIGHT_STEP.step_mod = ((((LIGHT_STEP.lum_dst - LIGHT_STEP.lum_temp)%((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32))*256)/((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32)) as u16;
    } else {
        LIGHT_STEP.step = ((LIGHT_STEP.lum_temp - LIGHT_STEP.lum_dst)/((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32)) as u16;
        LIGHT_STEP.step_mod = ((((LIGHT_STEP.lum_temp - LIGHT_STEP.lum_dst)%((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32))*256)/((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32)) as u16;
    }
}

pub unsafe fn light_onoff_step(on: bool){
    let mut set_flag= true;

    if on {
        if LIGHT_OFF {
            if !LIGHT_STEP.adjusting_flag {
                LIGHT_STEP.lum_temp = 0;
            }
            LIGHT_STEP.lum_dst = LED_LUM as i32;
            get_step(LUM_UP);
    	}else{
    	    set_flag = false;
    	    light_onoff_normal(true); // make sure on. unnecessary.
    	}
        LIGHT_OFF = false;
	}else{
        if LIGHT_OFF {
    	    set_flag = false;
    	    light_onoff_normal(false); // make sure off. unnecessary.
    	}else{
            if !LIGHT_STEP.adjusting_flag {
                LIGHT_STEP.lum_temp = LED_LUM as i32;
            }
            LIGHT_STEP.lum_dst = 0;
            get_step(LUM_DOWN);
    	}
        LIGHT_OFF = true;
	}

    LIGHT_STEP.adjusting_flag = set_flag;
    LIGHT_STEP.time = 0;
}

pub unsafe fn light_step_reset(target: u16) {
    let r = irq_disable();
    if !LIGHT_STEP.adjusting_flag && target == LED_LUM {
        light_adjust_rgb_hw(LED_VAL[0], LED_VAL[1], LED_VAL[2], target);
        irq_restore(r);
		return
    }

    if LIGHT_STEP.adjusting_flag {
        if (target as i32) < LIGHT_STEP.lum_temp {
            LIGHT_STEP.lum_dst = target as i32;
            get_step(LUM_DOWN);
        }

        if (target as i32) > LIGHT_STEP.lum_temp {
            LIGHT_STEP.lum_dst = target as i32;
            get_step(LUM_UP);
        }
    } else {
        if target < LED_LUM {
            LIGHT_STEP.lum_temp = LED_LUM as i32;
            LIGHT_STEP.lum_dst = target as i32;
            get_step(LUM_DOWN);
        }

        if target > LED_LUM {
            LIGHT_STEP.lum_temp = LED_LUM as i32;
            LIGHT_STEP.lum_dst = target as i32;
            get_step(LUM_UP);
        }
    }

    LIGHT_STEP.adjusting_flag = true;
    LIGHT_STEP.time = 0;
    LED_LUM = target;

    irq_restore(r);
}

unsafe fn get_next_lum(direction: u8){
    let temp = LIGHT_STEP.remainder as u32 + LIGHT_STEP.step_mod as u32;
    LIGHT_STEP.remainder = (temp & 0xffff) as u16;

    if LUM_UP == direction {
        LIGHT_STEP.lum_temp += LIGHT_STEP.step as i32;
        if temp >= 0x10000 {
            LIGHT_STEP.lum_temp += 1;
        }
        if LIGHT_STEP.lum_temp >= LIGHT_STEP.lum_dst {
            LIGHT_STEP.lum_temp = LIGHT_STEP.lum_dst;
            LIGHT_STEP.remainder = 0;
        }
    }else{
        LIGHT_STEP.lum_temp -= LIGHT_STEP.step as i32;
        if temp >= 0x10000 {
            LIGHT_STEP.lum_temp -= 1;
        }
        if LIGHT_STEP.lum_temp <= LIGHT_STEP.lum_dst {
            LIGHT_STEP.lum_temp = LIGHT_STEP.lum_dst;
            LIGHT_STEP.remainder = 0;
        }
    }
}

pub unsafe fn light_onoff_step_timer() {
    if LIGHT_STEP.adjusting_flag {
        if LIGHT_STEP.time == 0 {
            if LIGHT_STEP.lum_dst != LIGHT_STEP.lum_temp {
                if LIGHT_STEP.lum_temp < LIGHT_STEP.lum_dst {
                    get_next_lum(LUM_UP);
                }else{
                    get_next_lum(LUM_DOWN);
                }
                light_adjust_rgb_hw(LED_VAL[0], LED_VAL[1], LED_VAL[2], LIGHT_STEP.lum_temp as u16);
            }else{
                LIGHT_STEP.adjusting_flag = false;
            }
        }

        LIGHT_STEP.time += 1;
        if LIGHT_STEP.time >= LIGHT_ADJUST_INTERVAL as u32 {
            LIGHT_STEP.time = 0;
        }
    }
}

// recover status before software reboot
#[no_mangle]
unsafe fn light_sw_reboot_callback() {
    if rf_slave_ota_busy || _is_mesh_ota_slave_running() {	// rf_slave_ota_busy means mesh ota master busy also.
        analog_write__attribute_ram_code (REGA_LIGHT_OFF, if LIGHT_OFF { FldLightOff as u8} else {0});
    }
}

pub unsafe fn is_mesh_cmd_need_delay(p_cmd: *const u8, params: *const u8, ttc: u8) -> bool
{
	let delay_tmp = (*params.offset(1) as u16) | ((*params.offset(2) as u16) << 8);
	if delay_tmp != 0 {
		if CMD_LEFT_DELAY_MS != 0 {
			return true;
		}
		CMD_DELAY_MS = delay_tmp;
		if CMD_DELAY_MS != 0 && IRQ_TIMER1_CB_TIME == 0 {
			let cmd_delayed_ms = light_cmd_delayed_ms(ttc);
			if CMD_DELAY_MS > cmd_delayed_ms {
				copy_nonoverlapping(p_cmd, addr_of!(CMD_DELAY) as *mut u8, size_of::<ll_packet_l2cap_data_t>());
				CMD_LEFT_DELAY_MS = CMD_DELAY_MS - cmd_delayed_ms;
				IRQ_TIMER1_CB_TIME = clock_time();
				return true;
			}
		}
	}
	return false;
}

#[derive(Clone, Copy)]
#[derive(PartialEq)]
pub enum MeshPairState {
    MeshPairName1 = 0,
    MeshPairName2,
    MeshPairPwd1,
    MeshPairPwd2,
    MeshPairLtk1,
    MeshPairLtk2,
    MeshPairEffectDelay,
    MeshPairEffect,
    MeshPairDefaultMesh,
}

pub unsafe fn mesh_pair_cb(params: *const u8)
{
    if default_mesh_time_ref != 0 {
        // return;
        default_mesh_time_ref = clock_time() | 1;
    }
	let cmd = *params.offset(0);
    if cmd == MeshPairState::MeshPairName1 as u8 {
        mesh_pair_start_time = clock_time() | 1;
        copy_nonoverlapping(params.offset(1), new_mesh_name.as_mut_ptr(), 8);
    }else if cmd == MeshPairState::MeshPairName2 as u8 {
        copy_nonoverlapping(params.offset(1), new_mesh_name.as_mut_ptr().offset(8), 8);
    }else if cmd == MeshPairState::MeshPairPwd1 as u8 {
        copy_nonoverlapping(params.offset(1), new_mesh_pwd.as_mut_ptr(), 8);
    }else if cmd == MeshPairState::MeshPairPwd2 as u8 {
        copy_nonoverlapping(params.offset(1), new_mesh_pwd.as_mut_ptr().offset(8), 8);
    }else if cmd == MeshPairState::MeshPairLtk1 as u8 {
        copy_nonoverlapping(params.offset(1), new_mesh_ltk.as_mut_ptr(), 8);
    }else if cmd == MeshPairState::MeshPairLtk2 as u8 {
        copy_nonoverlapping(params.offset(1), new_mesh_ltk.as_mut_ptr().offset(8), 8);
    }else if cmd == MeshPairState::MeshPairEffectDelay as u8 {
        effect_new_mesh_delay_time = clock_time() | 1;
        if default_mesh_time_ref != 0 {
            /* Keep default_mesh_time_ref non-zero */
            default_mesh_time = mesh_pair_cmd_interval * 2;
        }
    }else if cmd == MeshPairState::MeshPairEffect as u8 {
        effect_new_mesh = 1;
    }
    else if cmd == MeshPairState::MeshPairDefaultMesh as u8 {
        default_mesh_effect_delay_ref = clock_time() | 1;
        default_mesh_time = *params.offset(1) as u32 * 1000;
    }
}

unsafe fn mesh_cmd_notify(op: u8, p: *const u8, len: u8, dev_adr: u16) -> i32
{
    let mut err = -1;
    if slave_link_connected && pair_login_ok {
        if len > 10 {   //max length of par is 10
            return -1;
        }

		let mut pkt_notify = rf_packet_att_cmd_t {
				dma_len: 0x1d,						// dma_len
				_type: 0x02,						// type
				rf_len: 0x1b,						// rf_len
            l2capLen: 0x17,						// u16
				chanId: 0x04,						// chanid
				opcode: 0x1b,						// notify
				handle: 0x12, handle1: 0x00, 				// status handler
				value: [0; 30]
		};

        pkt_notify.value[8] = (VENDOR_ID & 0xff) as u8;
        pkt_notify.value[8] = (VENDOR_ID >> 8) as u8;

        pkt_notify.value[3] = (dev_adr & 0xFF) as u8;
        pkt_notify.value[4] = (dev_adr >> 8) as u8;
		pkt_notify.value[7] = op | 0xc0;

        let p_par = pkt_notify.value.as_mut_ptr().offset(10);
        copy_nonoverlapping(p, p_par, len as usize);

        let r = irq_disable();
        if _is_add_packet_buf_ready() {
            if _rf_link_add_tx_packet (addr_of!(pkt_notify) as *const u8) {
                err = 0;
            }
        }
        irq_restore(r);
    }

    return err;
}

unsafe fn mesh_pair_complete_notify() -> i32
{
	let par = [CMD_NOTIFY_MESH_PAIR_END];
	return mesh_cmd_notify(LGT_CMD_MESH_CMD_NOTIFY, par.as_ptr(), par.len() as u8, device_address);
}

unsafe fn _safe_effect_new_mesh_finish() {
    new_mesh_name = [0; 16];
    new_mesh_pwd = [0; 16];
    new_mesh_ltk = [0; 16];
    mesh_pair_start_notify_time = 0;
    mesh_pair_retry_cnt = 0;
    mesh_pair_start_time = 0;
    mesh_pair_notify_rsp_mask = [0; 32];
    pair_setting_flag = PairState::PairSetted;
}

unsafe fn save_effect_new_mesh()
{
    if default_mesh_time_ref != 0 || get_mac_en
    {
		mesh_pair_complete_notify();
		sleep_us(1000);
        /* Switch to normal mesh */
        _pair_load_key();
        default_mesh_time_ref = 0;

        _mesh_node_init();
        device_status_update();
        _safe_effect_new_mesh_finish();
        return;
    }

    if effect_new_mesh == 0 {
        copy_nonoverlapping(new_mesh_name.as_ptr(), pair_nn.as_mut_ptr(), 16);
        copy_nonoverlapping(new_mesh_pwd.as_ptr(), pair_pass.as_mut_ptr(), 16);
        copy_nonoverlapping(new_mesh_ltk.as_ptr(), pair_ltk.as_mut_ptr(), 16);
    }
    else
    {
        copy_nonoverlapping(pair_ltk_mesh.as_ptr(), pair_ltk.as_mut_ptr(), 16);
    }

	mesh_pair_complete_notify();

    // make sure not receive legacy mesh data from now on
    let r = irq_disable();
    _pair_save_key();
    rf_set_ble_access_code (addr_of!(pair_ac) as *const u8);// use new access code at once.
    rf_link_light_event_callback(LGT_CMD_SET_MESH_INFO);	// clear online status :mesh_node_init()
	sleep_us (1000);
    write_reg_rf_irq_status(FLD_RF_IRQ_MASK::IRQ_RX as u16);		// clear current rx in buffer
    irq_restore(r);

    _safe_effect_new_mesh_finish();
}

#[no_mangle] // Required by light_ll rf_link_slave_add_status_ll
pub unsafe fn mesh_pair_notify_refresh(p: *const rf_packet_att_cmd_t) -> u8 {
    let mut same = true;
    for i in 0..8
    {
        if mesh_pair_checksum[i] != *(*p).value.as_ptr().offset(12 + i as isize) {
            same = false;
            break;
        }
    }

    if same {
        // mesh pair : success one device, clear the mask flag
        mesh_pair_notify_rsp_mask[((*p).value[10] / 8) as usize] &= !(BIT!((*p).value[10] % 8));
    }

    return 1;// if return 2, then the notify rsp will not report to master.
}

unsafe fn switch_to_default_mesh(delay_s: u8)
{
    default_mesh_time_ref = clock_time() | 1;
    default_mesh_time = delay_s as u32 * 1000;

    /* Only change AC and LTK */
    pair_ac = _access_code(get_pair_config_mesh_name().as_ptr(), get_pair_config_mesh_pwd().as_ptr());
    copy_nonoverlapping(get_pair_config_mesh_ltk().as_ptr(), pair_ltk.as_mut_ptr(), 16);
}

unsafe fn get_online_node_cnt() -> u8
{
    let mut cnt = 0;
	for i in 0..mesh_node_max {
	    if mesh_node_st[i as usize].tick != 0 {
	        cnt += 0;
	        if i > 0 {
	            mesh_pair_notify_rsp_mask[(mesh_node_st[i as usize].val.dev_adr / 8) as usize] |= BIT!(mesh_node_st[i as usize].val.dev_adr % 8);
	        }
	    }
	}

    if GATEWAY_STATUS == GatewayStatusT::GatewayStatusCfgUnproDev || GATEWAY_STATUS == GatewayStatusT::GatewayStatusAddConfiguredDevice
    {
        /* If provisioning device to network, shall at least return two device */
        if cnt < 2
        {
            return 2;
        }
    }

	return cnt;
}

#[no_mangle] // required by light_ll
unsafe fn mesh_pair_proc()
{
    let mut dst_addr = 0xFFFF;
    let mut op_para: [u8; 16] = [0; 16];

    if default_mesh_effect_delay_ref != 0 && clock_time_exceed(default_mesh_effect_delay_ref, MESH_PAIR_CMD_INTERVAL * 1000)
    {
        default_mesh_effect_delay_ref = 0;

        if default_mesh_time == 0x00
        {
            _pair_load_key();
            default_mesh_time_ref = 0;
        }
        else
        {
            switch_to_default_mesh((default_mesh_time / 1000) as u8);
            default_mesh_time_ref = clock_time() | 1;
        }
    }
    else if default_mesh_time_ref != 0 && clock_time_exceed(default_mesh_time_ref, default_mesh_time * 1000)
    {
        /* Switch to normal mesh */
        if default_mesh_time == 255000
        {
            default_mesh_time_ref = clock_time() | 1;
        }
        else
        {
            _pair_load_key();
            default_mesh_time_ref = 0;
        }
    }
    if mesh_pair_start_time != 0 && clock_time_exceed(mesh_pair_start_time, mesh_pair_timeout*1000*1000) {
        //mesh pair time out
        _pair_load_key();
        new_mesh_name = [0; 16];
        new_mesh_pwd = [0; 16];
        new_mesh_ltk = [0; 16];
        MESH_PAIR_STATE = MeshPairState::MeshPairName1;
        mesh_pair_start_notify_time = 0;
        mesh_pair_retry_cnt = 0;
        mesh_pair_start_time = 0;
        mesh_pair_notify_rsp_mask = [0; 32];
        pair_setting_flag = PairState::PairSetted;
        rf_link_light_event_callback(LGT_CMD_MESH_PAIR_TIMEOUT);
        return;
    }

    if pair_setting_flag == PairState::PairSetMeshTxStart && MESH_PAIR_STATE == MeshPairState::MeshPairName1 && get_online_node_cnt() == 1 {
        op_para[0] = LGT_CMD_MESH_PAIR;
        op_para[3] = MeshPairState::MeshPairEffect as u8;
        dst_addr = 0x0000;// there is noly one device in mesh,just effect itself.
        MESH_PAIR_STATE = MeshPairState::MeshPairName1;
        mesh_pair_start_notify_time = 0;
        mesh_pair_retry_cnt = 0;
        mesh_pair_start_time = 0;
        mesh_pair_notify_rsp_mask = [0; 32];
        pair_setting_flag = PairState::PairSetted;
    }else if pair_setting_flag as u8 >= PairState::PairSetMeshTxStart as u8 && clock_time_exceed(MESH_PAIR_TIME, mesh_pair_cmd_interval*1000) {
        MESH_PAIR_TIME = clock_time();
        if pair_setting_flag == PairState::PairSetMeshTxStart {
            op_para[0] = LGT_CMD_MESH_PAIR;
            op_para[3] = MESH_PAIR_STATE as u8;
            if MESH_PAIR_STATE == MeshPairState::MeshPairName1 {
                // send mesh name [0-7]
                copy_nonoverlapping(pair_nn.as_mut_ptr(), op_para.as_mut_ptr().offset(4), 8);
        		MESH_PAIR_STATE = MeshPairState::MeshPairName2;
            }else if MESH_PAIR_STATE == MeshPairState::MeshPairName2 {
                // send mesh name [8-15]
                copy_nonoverlapping(pair_nn.as_mut_ptr().offset(8), op_para.as_mut_ptr().offset(4), 8);
        		MESH_PAIR_STATE = MeshPairState::MeshPairPwd1;
            }else if MESH_PAIR_STATE == MeshPairState::MeshPairPwd1 {
                // send mesh pwd [0-7]
                copy_nonoverlapping(pair_pass.as_mut_ptr(), op_para.as_mut_ptr().offset(4), 8);
        		MESH_PAIR_STATE = MeshPairState::MeshPairPwd2;
            }else if MESH_PAIR_STATE == MeshPairState::MeshPairPwd2 {
                // send mesh pwd [8-15]
                copy_nonoverlapping(pair_pass.as_mut_ptr().offset(8), op_para.as_mut_ptr().offset(4), 8);
        		MESH_PAIR_STATE = MeshPairState::MeshPairLtk1;
            }else if MESH_PAIR_STATE == MeshPairState::MeshPairLtk1 {
                // send mesh ltk [0-7]
                copy_nonoverlapping(pair_ltk_mesh.as_mut_ptr(), op_para.as_mut_ptr().offset(4), 8);
        		MESH_PAIR_STATE = MeshPairState::MeshPairLtk2;
            }else if MESH_PAIR_STATE == MeshPairState::MeshPairLtk2 {
                // send mesh ltk [8-15]
                copy_nonoverlapping(pair_ltk_mesh.as_mut_ptr().offset(8), op_para.as_mut_ptr().offset(4), 8);
        		MESH_PAIR_STATE = MeshPairState::MeshPairName1;
        		pair_setting_flag = PairState::PairSetMeshTxDone;
            }else{
                MESH_PAIR_STATE = MeshPairState::MeshPairName1;
                mesh_pair_start_notify_time = 0;
                mesh_pair_retry_cnt = 0;
                mesh_pair_start_time = 0;
                mesh_pair_notify_rsp_mask = [0; 32];
        		pair_setting_flag = PairState::PairSetted;
        		return;
            }
        }else if pair_setting_flag == PairState::PairSetMeshTxDone {
            // get mesh nodes' confirm value
            //rf_link_slave_read_status_start();
            op_para[0] = LGT_CMD_MESH_OTA_READ;
            op_para[3] = 0x10;// bridge cnt
            op_para[4] = PAR_READ_MESH_PAIR_CONFIRM;
            pair_setting_flag = PairState::PairSetMeshRxDone;
            mesh_pair_start_notify_time = clock_time() | 0;
            for i in 0..8 {
                mesh_pair_checksum[i] = get_mesh_pair_checksum(i as u8);
            }
        }else if pair_setting_flag == PairState::PairSetMeshRxDone {
            let mut effect_flag = mesh_pair_notify_rsp_mask == [0; 32];
            if !effect_flag && clock_time_exceed(mesh_pair_start_time, MESH_PAIR_NOTIFY_TIMEOUT*1000) {
                if mesh_pair_retry_cnt < mesh_pair_retry_max {
                    mesh_pair_start_time = clock_time() | 1;
                    pair_setting_flag = PairState::PairSetMeshTxStart;
                    MESH_PAIR_STATE = MeshPairState::MeshPairName1;
                }else{
                    // retry timeout, effect or cancel?? effect now
                    effect_flag = true;
                }
                mesh_pair_retry_cnt += 1;
            }
            if effect_flag {
                //send cmd to switch to new mesh
                op_para[0] = LGT_CMD_MESH_PAIR;
                op_para[3] = MeshPairState::MeshPairEffectDelay as u8;
                MESH_PAIR_STATE = MeshPairState::MeshPairName1;
                mesh_pair_start_notify_time = 0;
                mesh_pair_retry_cnt = 0;
                mesh_pair_start_time = 0;
                mesh_pair_notify_rsp_mask = [0; 32];
                pair_setting_flag = PairState::PairSetted;
            }
        }
    }else{
        return;
    }

    light_slave_tx_command(op_para.as_mut_ptr(), dst_addr);
}

#[no_mangle] // required by light_ll
unsafe fn mesh_node_buf_init()
{
    [mesh_node_st_t{ tick: 0, val: mesh_node_st_val_t {
        dev_adr: 0,
        sn: 0,
        par: [0; MESH_NODE_ST_PAR_LEN as usize],
    } }; MESH_NODE_MAX_NUM as usize];

	device_status_update();
}

#[no_mangle]
pub unsafe fn rf_link_slave_data_ota(ph: *const u8) -> bool
{
    if rf_slave_ota_finished_flag != OtaState::CONTINUE {
        return true;
    }

    if !rf_slave_ota_busy {
        if !pair_login_ok || _is_master_sending_ota_st() || _is_mesh_ota_slave_running() {
            return true;
        }

        rf_slave_ota_busy = true;
        if slave_read_status_busy
        {
            _rf_link_slave_read_status_stop ();
        }
    }
	copy_nonoverlapping(ph, get_buff_response()[(SLAVE_OTA_DATA_CACHE_IDX %16) as usize].as_mut_ptr() as *mut u8, size_of::<rf_packet_att_data_t>());
    SLAVE_OTA_DATA_CACHE_IDX += 1;
	return true;
}

#[no_mangle]
unsafe fn get_ota_check_type(par: *const u8) -> u8
{
	if *par.offset(0) == 0x5D {
		return *par.offset(1);
	}
	return 0;
}

#[no_mangle]
unsafe fn rf_slave_ota_finished_flag_set(reset_flag: OtaState)
{
	rf_slave_ota_finished_flag = reset_flag;
	RF_SLAVE_OTA_FINISHED_TIME = clock_time();
}

pub unsafe fn rf_link_slave_data_ota_save() -> bool
{
    let mut reset_flag= OtaState::CONTINUE;
	for i in 0..SLAVE_OTA_DATA_CACHE_IDX {
		let p = get_buff_response()[i as usize].as_mut_ptr() as *const rf_packet_att_data_t;
		let n_data_len = (*p).l2cap - 7;

		if crc16((*p).dat.as_ptr(), n_data_len + 2) == (*p).dat[(n_data_len +2) as usize] as u16 | ((*p).dat[(n_data_len +3) as usize] as u16) << 8 {
			rf_slave_ota_timeout_s = rf_slave_ota_timeout_def_s;	// refresh timeout

			let cur_idx = (*p).dat[0] as u16 | ((*p).dat[1] as u16) << 8;
			if n_data_len == 0 {
				if (cur_idx == OTA_RCV_LAST_IDX +1) && (cur_idx == OTA_PKT_TOTAL) {
					// ota ok, save, reboot
					reset_flag = OtaState::OK;
				}else{
					// ota err
					cur_ota_flash_addr = 0;
                    OTA_PKT_CNT = 0;
                    OTA_RCV_LAST_IDX = 0;
					reset_flag = OtaState::ERROR;
				}
			}else{
				if cur_idx == 0 {
					// start ota
					if cur_ota_flash_addr != 0 {
					    // 0x10000 should be 0x00
						cur_ota_flash_addr = 0;
                        OTA_PKT_CNT = 0;
                        OTA_RCV_LAST_IDX = 0;
	                    reset_flag = OtaState::ERROR;
					}else{
					    NEED_CHECK_TYPE = get_ota_check_type(&(*p).dat[8]);
					    if NEED_CHECK_TYPE == 1 {
					    	FW_CHECK_VAL = ((*p).dat[(n_data_len +2) as usize] as u16 | ((*p).dat[(n_data_len +3) as usize] as u16) <<8) as u32;
					    }
	    				reset_flag = _rf_ota_save_data((*p).dat.as_ptr().offset(2));
					}
				}else if cur_idx == OTA_RCV_LAST_IDX + 1 {
					// ota fw check
					if cur_idx == 1 {
						OTA_PKT_TOTAL = (((((*p).dat[10] as u32) |( (((*p).dat[11] as u32) << 8) & 0xFF00) | ((((*p).dat[12] as u32) << 16) & 0xFF0000) | ((((*p).dat[13] as u32) << 24) & 0xFF000000)) + 15)/16) as u16;
						if OTA_PKT_TOTAL < 3 {
							// invalid fw
							cur_ota_flash_addr = 0;
                            OTA_PKT_CNT = 0;
                            OTA_RCV_LAST_IDX = 0;
							reset_flag = OtaState::ERROR;
						}else if NEED_CHECK_TYPE == 1 {
							FW_CHECK_VAL += ((*p).dat[(n_data_len +2) as usize] as u16 | ((*p).dat[(n_data_len +3) as usize] as u16) <<8) as u32;
						}
					}else if cur_idx < OTA_PKT_TOTAL - 1 && NEED_CHECK_TYPE == 1 {
						FW_CHECK_VAL += ((*p).dat[(n_data_len +2) as usize] as u16 | ((*p).dat[(n_data_len +3) as usize] as u16) <<8) as u32;
					}else if cur_idx == OTA_PKT_TOTAL - 1 && NEED_CHECK_TYPE == 1 {
						if FW_CHECK_VAL != (((*p).dat[2] as u32) |( (((*p).dat[3] as u32) << 8) & 0xFF00) | ((((*p).dat[4] as u32) << 16) & 0xFF0000) | ((((*p).dat[5] as u32) << 24) & 0xFF000000)) {
							cur_ota_flash_addr = 0;
                            OTA_PKT_CNT = 0;
                            OTA_RCV_LAST_IDX = 0;
							reset_flag = OtaState::ERROR;
						}
					}

					if cur_ota_flash_addr + 16 > (FW_SIZE_MAX_K * 1024) || reset_flag != OtaState::CONTINUE { // !is_valid_fw_len()
					    reset_flag = OtaState::ERROR;
				    }else{
					    reset_flag = _rf_ota_save_data(&(*p).dat[2]);
					}
				}else{
					// error, ota failed
					cur_ota_flash_addr = 0;
                    OTA_PKT_CNT = 0;
                    OTA_RCV_LAST_IDX = 0;
					reset_flag = OtaState::ERROR;
				}

				OTA_RCV_LAST_IDX = cur_idx;
			}
		}else{
			// error, ota failed
			cur_ota_flash_addr = 0;
            OTA_PKT_CNT = 0;
            OTA_RCV_LAST_IDX = 0;
		    reset_flag = OtaState::ERROR;
		}

		if reset_flag != OtaState::CONTINUE {
		    if rf_slave_ota_finished_flag == OtaState::CONTINUE {
		    	if (APP_OTA_HCI_TYPE::MESH == app_ota_hci_type)
		    	&& (reset_flag == OtaState::OK) {
		    		_mesh_ota_master_start_firmware_from_backup();
		    		rf_slave_ota_timeout_s = 0;	// stop gatt ota timeout check
		    		rf_slave_ota_busy = false;		// must
			    }else{
			    	rf_slave_ota_finished_flag_set(reset_flag);
			    }
		    }

			break;
		}
	}
	SLAVE_OTA_DATA_CACHE_IDX = 0;
	return true;
}

#[no_mangle]
unsafe fn rf_ota_set_flag()
{
    let fw_flag_telink = START_UP_FLAG;
    let mut flag_new = 0;
	flash_read_page(flash_adr_light_new_fw + 8, size_of_val(&flag_new) as u32, addr_of!(flag_new) as *mut u8);
	flag_new &= 0xffffff4b;
    if flag_new != fw_flag_telink {
        return ; // invalid flag
    }

	let mut flag0 = 0;
	flash_read_page(flash_adr_light_new_fw + 8, 1, addr_of!(flag0) as *mut u8);
	if 0x4b != flag0 {
	    flag0 = 0x4b;
	    flash_write_page(flash_adr_light_new_fw + 8, 1, addr_of!(flag0) as *mut u8);		//Set FW flag, make sure valid. because the firmware may be from 8267 by mesh ota
	}

	flash_erase_sector (FLASH_ADR_OTA_READY_FLAG);
	let flag: u32 = 0xa5;
	flash_write_page (FLASH_ADR_OTA_READY_FLAG, 4, addr_of!(flag) as *mut u8);
}

pub fn rf_led_ota_ok(){
	gpio_set_func(OTA_LED as u32, AS_GPIO);
	gpio_set_output_en(OTA_LED as u32, 1);
	let mut led_onoff = 1;
	for _ in 0..6
    {
		gpio_write(OTA_LED as u32, led_onoff);
		led_onoff = !led_onoff;
        wd_clear();
		sleep_us(1000*1000);
	}
}

#[no_mangle]
fn rf_led_ota_error(){
	gpio_set_func(OTA_LED as u32, AS_GPIO);
	gpio_set_output_en(OTA_LED as u32, 1);
	let mut led_onoff = 1;
	for _ in 0..60
	{
		gpio_write(OTA_LED as u32, led_onoff);
		led_onoff = !led_onoff;
        wd_clear();
		sleep_us(100*1000);
	}
}

#[no_mangle]
unsafe fn rf_link_slave_ota_finish_led_and_reboot(st: OtaState)
{
    if OtaState::ERROR == st {
        erase_ota_data(flash_adr_light_new_fw);
        rf_led_ota_error();
    }else if OtaState::OK == st {
        //rf_ota_save_data(0);
        rf_ota_set_flag ();
        rf_led_ota_ok();
    }else if OtaState::MASTER_OTA_REBOOT_ONLY == st {
    	// just reboot
    }
    irq_disable ();
    _light_sw_reboot();
}

#[no_mangle]
unsafe fn rf_link_slave_ota_finish_handle()		// poll when ota busy in bridge
{
	rf_link_slave_data_ota_save();

    if rf_slave_ota_finished_flag != OtaState::CONTINUE {
        let mut reboot_flag = false;
        if (0 == TERMINATE_CNT) && rf_slave_ota_terminate_flag {
            if _is_add_packet_buf_ready() {
                TERMINATE_CNT = 6;
                _rf_link_add_tx_packet (PKT_TERMINATE.as_ptr());
            }
        }

        if TERMINATE_CNT != 0 {
            TERMINATE_CNT -= 1;
            if TERMINATE_CNT == 0{
                reboot_flag = true;
            }
        }

        if !rf_slave_ota_terminate_flag && (clock_time() - RF_SLAVE_OTA_FINISHED_TIME) > 2000*1000 * tick_per_us {
            rf_slave_ota_terminate_flag = true;    // for ios: no last read command
        }

        if (clock_time() - RF_SLAVE_OTA_FINISHED_TIME) > 4000*1000 * tick_per_us {
            reboot_flag = true;
        }

        if reboot_flag {
            rf_link_slave_ota_finish_led_and_reboot(rf_slave_ota_finished_flag);
            // have been reboot
        }
    }
}

#[no_mangle]
fn mesh_ota_led_cb(_type: MESH_OTA_LED)
{
    if MESH_OTA_LED::OK == _type {
        rf_led_ota_ok();
    }else if MESH_OTA_LED::ERROR == _type{
        rf_led_ota_error();
    }else if MESH_OTA_LED::STOP == _type{
        rf_led_ota_error();
    }
}

// adr:0=flag 16=name 32=pwd 48=ltk
// p:data
// n:length
#[no_mangle]
unsafe fn save_pair_info(adr: u32, p: *const u8, n: u32)
{
	flash_write_page (flash_adr_pairing + adr_flash_cfg_idx + adr, n, p);
}

#[no_mangle]
fn get_ota_erase_sectors() -> u32
{
   return ERASE_SECTORS_FOR_OTA;
}

#[no_mangle]
fn is_valid_fw_len(fw_len: u32) -> bool
{
	return fw_len <= (FW_SIZE_MAX_K * 1024);
}

#[no_mangle]
fn get_fw_len(fw_adr: u32) -> u32
{
	let fw_len = 0;
	flash_read_page(fw_adr+0x18, 4, addr_of!(fw_len) as *mut u8);	// use flash read should be better
	return fw_len;
}

#[no_mangle]
unsafe fn mesh_ota_master_start_firmware(p_dev_info: *const mesh_ota_dev_info_t, new_fw_adr: u32)
{
	let fw_len = get_fw_len(new_fw_adr);
	if is_valid_fw_len(fw_len) {
		_mesh_ota_master_start(new_fw_adr as *const u8, fw_len, p_dev_info);
	}
}

#[no_mangle]
pub fn mesh_ota_master_start_firmware_from_own()
{
    let adr_fw = 0;
    let fw_len = get_fw_len(adr_fw);
	if is_valid_fw_len(fw_len) {
		let dev_info: mesh_ota_dev_info_t = mesh_ota_dev_info_t{
            dev_mode: 0x02, // LIGHT_MODE
            no_check_dev_mode_flag_rsv: 0,
            rsv: [0; 4],
        };
		_mesh_ota_master_start(adr_fw as *const u8, fw_len, addr_of!(dev_info));
	}
}

#[no_mangle]
unsafe fn mesh_ota_slave_need_ota(params: *const u8) -> bool
{
   let mut ret = true;
   let p = params.offset(2) as *const mesh_ota_pkt_start_command_t;

   if 0x02 == (*p).dev_info.dev_mode { // LIGHT_MODE
       let mut ver_myself: [u8; 4] = [0; 4];
       _get_fw_version(ver_myself.as_mut_ptr());
       if (*p).version[1] < ver_myself[1] {
           ret = false;
       }else if (*p).version[1] == ver_myself[1] {
           if (*p).version[3] <= ver_myself[3] {
               ret = false;
           }
       }
   }else{
       ret = false;
   }

   return ret;
}

#[no_mangle]
unsafe fn is_light_mode_match_check_fw(new_fw_dev_info: *const u8) -> bool
{
   let light_mode_self: u16 = 0x02; // LIGHT_MODE;
   return *(new_fw_dev_info as *const u16) == light_mode_self;
}

/************
*
* int setup_ble_parameter_start(u16 delay, u16 interval_min, u16 interval_max, u16 timeout);
*
* delay   :  unit: one ble interval
* interval_min,interval_max:  if all 0,will keep the system parameter for android but not ios.   unit: 1.25ms; must longer than 20ms.
* timeout:  if 0,will keep the system parameter.   unit: 10ms; must longer than 3second for steady connect.
*
* return 0 means setup parameters is valid.
* return -1 means parameter of interval is invalid.
* return -2 means parameter of timeout is invalid.
*
*
* void rf_link_slave_connect_callback()
* system will call this function when receive command of BLE connect request.
    IOS Note:
    20 ms <= interval_min
    interval_min + 20 ms <= interval_max <= 2second
    timeout <= 6second
*/
#[no_mangle]
unsafe fn update_ble_parameter_cb() {
   if conn_update_successed == 0 {
       _setup_ble_parameter_start(1, CONN_PARA_DATA[0][0], CONN_PARA_DATA[0][1], CONN_PARA_DATA[0][2]);  // interval 32: means 40ms;   timeout 200: means 2000ms
       conn_update_cnt += 1;
   }
}

#[no_mangle]
unsafe fn rf_update_conn_para(p: *const u8) -> u8
{
   let pp = p as *const rf_pkt_l2cap_sig_connParaUpRsp_t;
   let sig_conn_param_update_rsp: [u8; 9] = [0x0A, 0x06, 0x00, 0x05, 0x00, 0x13, 0x01, 0x02, 0x00];
    let mut equal = true;
    for i in 0..9
    {
        if *((&(*pp).rf_len) as *const u8).offset(i) != sig_conn_param_update_rsp[i as usize] {
            equal = false;
        }
    }

   if equal && (((*pp)._type & 0b11) == 2){//l2cap data pkt, start pkt
       if (*pp).result == 0x0000 {
           conn_update_cnt = 0;
           conn_update_successed = 1;
       }else if (*pp).result == 0x0001 {
           if conn_update_cnt >= UPDATE_CONN_PARA_CNT {
               conn_update_cnt = 0;
           }else{
               _setup_ble_parameter_start(1, CONN_PARA_DATA[conn_update_cnt as usize][0], CONN_PARA_DATA[conn_update_cnt as usize][1], CONN_PARA_DATA[conn_update_cnt as usize][2]);
               conn_update_cnt += 1;
           }
       }
   }

   return 0;
}

#[no_mangle]
unsafe fn light_slave_tx_command_callback (p: *const u8) {
    _rf_link_data_callback(p);
}
