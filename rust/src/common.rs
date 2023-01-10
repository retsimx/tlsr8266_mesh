use std::mem::size_of;
use std::ptr::{copy_nonoverlapping, write_bytes};
use ::{BIT, flash_adr_pairing};
use common::RECOVER_STATUS::FLD_LIGHT_OFF;
use main_light::{cmd_delay, cmd_delay_ms, cmd_left_delay_ms, device_status_update, irq_timer1_cb_time, led_lum, led_val, light_adjust_RGB_hw, light_off, light_onoff_normal, light_slave_tx_command, rf_link_light_event_callback};
use sdk::common::bit::ONES_32;
use sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};
use sdk::light::{access_code, adr_flash_cfg_idx, CMD_NOTIFY_MESH_PAIR_END, DEV_ADDR_PAR_WITH_MAC, device_address, get_mac_en, is_add_packet_buf_ready, is_mesh_ota_slave_running, LGT_CMD_MESH_CMD_NOTIFY, LGT_CMD_MESH_OTA_READ, LGT_CMD_MESH_PAIR, LGT_CMD_MESH_PAIR_TIMEOUT, LGT_CMD_SET_MESH_INFO, ll_packet_l2cap_data_t, mesh_node_init, mesh_node_max, mesh_ota_master_100_flag, pair_ac, pair_config_mesh_ltk, pair_config_mesh_name, pair_config_mesh_pwd, pair_load_key, pair_login_ok, pair_ltk, pair_ltk_mesh, pair_nn, pair_pass, pair_save_key, pair_setting_flag, PAR_READ_MESH_PAIR_CONFIRM, rf_link_add_tx_packet, rf_packet_att_cmd_t, rf_slave_ota_busy, slave_link_connected, slave_p_mac};
use sdk::mcu::analog::{analog_read__attribute_ram_code, analog_write__attribute_ram_code};
use sdk::mcu::clock::{clock_time, clock_time_exceed, sleep_us};
use sdk::mcu::irq_i::{irq_disable, irq_restore};
use vendor_light::adv_rsp_pri_data;
use std::ptr::addr_of;
use sdk::mcu::register::{write_reg_rf_irq_status, FLD_RF_IRQ_MASK};
use sdk::rf_drv::rf_set_ble_access_code;
use VENDOR_ID;

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
#[no_mangle]
pub static mut mesh_pair_enable: bool = false;
#[no_mangle]
pub static mut mesh_pair_checksum: [u8; 8] = [0; 8];
#[no_mangle]
pub static mut mesh_pair_retry_max: u8 = 3;

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

pub static mut mesh_pair_time: u32 = 0;
pub static mut mesh_pair_state: MESH_PAIR_STATE = MESH_PAIR_STATE::MESH_PAIR_NAME1;
#[no_mangle]
pub static mut mesh_node_st: [mesh_node_st_t; MESH_NODE_MAX_NUM as usize] = [mesh_node_st_t{ tick: 0, val: mesh_node_st_val_t {
    dev_adr: 0,
    sn: 0,
    par: [0; MESH_NODE_ST_PAR_LEN as usize],
} }; MESH_NODE_MAX_NUM as usize];

#[derive(PartialEq)]
pub enum gateway_status_t
{
    GATEWAY_STATUS_NORMAL = 0,             /* Normal gateway role */
    GATEWAY_STATUS_NODE_ROLE,              /* As node role, when pushed button */

    GATEWAY_STATUS_TEMP_DEFALT_MESH,       /* In default mesh temporary */
    GATEWAY_STATUS_SWITCH_TO_DEFAULT_MESH,
    GATEWAY_STATUS_SCAN_UNPROV_DEV,        /* Scanning unpair device status */
    GATEWAY_STATUS_CFG_UNPRO_DEV,          /* Only provision device */
    GATEWAY_STATUS_CFG_CUR_NETWORK,        /* Change current network's information */
    GATEWAY_STATUS_ADD_CONFIGURED_DEVICE,  /* Add configured device */
}

pub static mut gateway_status: gateway_status_t = gateway_status_t::GATEWAY_STATUS_NORMAL;

pub const MESH_PAIR_CMD_INTERVAL: u32 = 500;

//unit: s
pub const MESH_PAIR_TIMEOUT: u32 = 10;
//unit: ms
pub const MESH_PAIR_NOTIFY_TIMEOUT: u32 = 2500;

pub const FW_SIZE_MAX_K : u32 =	128;
pub const ERASE_SECTORS_FOR_OTA : u32 = (FW_SIZE_MAX_K + 3) / 4;

pub const MESH_NODE_ST_VAL_LEN: u8 = 4;       // MIN: 4,   MAX: 10
pub const MESH_NODE_ST_PAR_LEN: u8 = MESH_NODE_ST_VAL_LEN - 2;

pub const rega_light_off: u8 = 0x3a;

const MESH_NODE_MAX_NUM: u16 = 64;

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
pub enum RECOVER_STATUS {
	FLD_LIGHT_OFF				= BIT!(0),
	FLD_MESH_OTA_MASTER_100		= BIT!(1),
	LOW_BATT_FLG                = BIT!(2),
	// LOW_BATT_LOOP_FLG           = BIT(3),    // 0 means check by user_init, 1 means by main loop
}

#[derive(PartialEq)]
#[derive(Clone, Copy)]
pub enum PAIR_STATE {
    PAIR_SETTED = 0,
    PAIR_SETTING,
    PAIR_SET_MESH_TX_START,
    PAIR_SET_MESH_TX_DONE,// send notify req, get mesh nodes' ac
    PAIR_SET_MESH_RX_DONE,// received all mesh nodes' ac, send cmd to switch to new mesh
}

pub fn is_ota_area_valid(adr: u32) -> bool {
	let mut buf : [u8; 4] = [0; 4];
	for i in 0..ERASE_SECTORS_FOR_OTA {
    	flash_read_page(adr + i*0x1000, 4, buf.as_mut_ptr());
    	let tmp = buf[0] as u32 | (buf[1] as u32) << 8 | (buf[2] as u32) << 16 | (buf[3] as u32) << 24;
    	if tmp != ONES_32 {
            return false;
    	}
    }
	return true;
}

pub fn erase_ota_data(adr: u32){
    for i in 0..ERASE_SECTORS_FOR_OTA {
        flash_erase_sector(adr+(ERASE_SECTORS_FOR_OTA -1 - i)*0x1000);
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
	let val = analog_read__attribute_ram_code(rega_light_off);
	if val & RECOVER_STATUS::FLD_MESH_OTA_MASTER_100 as u8 != 0 {
		unsafe { mesh_ota_master_100_flag = 1; }
		analog_write__attribute_ram_code(rega_light_off, val & !(RECOVER_STATUS::FLD_MESH_OTA_MASTER_100 as u8));
	}
}

pub unsafe fn dev_addr_with_mac_flag(params: *const u8) -> bool
{
	return DEV_ADDR_PAR_WITH_MAC == *params.offset(2);
}

pub unsafe fn dev_addr_with_mac_rsp(params: *const u8, par_rsp: *mut u8) -> bool
{
	if dev_addr_with_mac_match(params) {
		*par_rsp.offset(0) = (device_address & 0xff) as u8;
		*par_rsp.offset(1) = ((device_address >> 8) & 0xff) as u8;

		copy_nonoverlapping(slave_p_mac, par_rsp.offset( 2), 6);
		#[allow(unaligned_references)]
		copy_nonoverlapping(&adv_rsp_pri_data.ProductUUID, par_rsp.offset( 8) as *mut u16, 2);
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
			if params.offset(4+i) != slave_p_mac.offset(i) {
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

static mut light_step: light_step_t = light_step_t {
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
    light_step.remainder = 0;       // reset
    if LUM_UP == direction {
        light_step.step = ((light_step.lum_dst - light_step.lum_temp)/((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32)) as u16;
        light_step.step_mod = ((((light_step.lum_dst - light_step.lum_temp)%((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32))*256)/((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32)) as u16;
    } else {
        light_step.step = ((light_step.lum_temp - light_step.lum_dst)/((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32)) as u16;
        light_step.step_mod = ((((light_step.lum_temp - light_step.lum_dst)%((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32))*256)/((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32)) as u16;
    }
}

pub unsafe fn light_onoff_step(on: bool){
    let mut set_flag= true;

    if on {
        if light_off {
            if !light_step.adjusting_flag {
                light_step.lum_temp = 0;
            }
            light_step.lum_dst = led_lum as i32;
            get_step(LUM_UP);
    	}else{
    	    set_flag = false;
    	    light_onoff_normal(true); // make sure on. unnecessary.
    	}
        light_off = false;
	}else{
        if light_off {
    	    set_flag = false;
    	    light_onoff_normal(false); // make sure off. unnecessary.
    	}else{
            if !light_step.adjusting_flag {
                light_step.lum_temp = led_lum as i32;
            }
            light_step.lum_dst = 0;
            get_step(LUM_DOWN);
    	}
        light_off = true;
	}

    light_step.adjusting_flag = set_flag;
    light_step.time = 0;
}

pub unsafe fn light_step_reset(target: u16) {
    let r = irq_disable();
    if !light_step.adjusting_flag && target == led_lum {
        light_adjust_RGB_hw(led_val[0], led_val[1], led_val[2], target);
        irq_restore(r);
		return
    }

    if light_step.adjusting_flag {
        if (target as i32) < light_step.lum_temp {
            light_step.lum_dst = target as i32;
            get_step(LUM_DOWN);
        }

        if (target as i32) > light_step.lum_temp {
            light_step.lum_dst = target as i32;
            get_step(LUM_UP);
        }
    } else {
        if target < led_lum {
            light_step.lum_temp = led_lum as i32;
            light_step.lum_dst = target as i32;
            get_step(LUM_DOWN);
        }

        if target > led_lum {
            light_step.lum_temp = led_lum as i32;
            light_step.lum_dst = target as i32;
            get_step(LUM_UP);
        }
    }

    light_step.adjusting_flag = true;
    light_step.time = 0;
    led_lum = target;

    irq_restore(r);
}

unsafe fn get_next_lum(direction: u8){
    let temp = light_step.remainder as u32 + light_step.step_mod as u32;
    light_step.remainder = (temp & 0xffff) as u16;

    if LUM_UP == direction {
        light_step.lum_temp += light_step.step as i32;
        if temp >= 0x10000 {
            light_step.lum_temp += 1;
        }
        if light_step.lum_temp >= light_step.lum_dst {
            light_step.lum_temp = light_step.lum_dst;
            light_step.remainder = 0;
        }
    }else{
        light_step.lum_temp -= light_step.step as i32;
        if temp >= 0x10000 {
            light_step.lum_temp -= 1;
        }
        if light_step.lum_temp <= light_step.lum_dst {
            light_step.lum_temp = light_step.lum_dst;
            light_step.remainder = 0;
        }
    }
}

pub unsafe fn light_onoff_step_timer() {
    if light_step.adjusting_flag {
        if light_step.time == 0 {
            if light_step.lum_dst != light_step.lum_temp {
                if light_step.lum_temp < light_step.lum_dst {
                    get_next_lum(LUM_UP);
                }else{
                    get_next_lum(LUM_DOWN);
                }
                light_adjust_RGB_hw(led_val[0], led_val[1], led_val[2], light_step.lum_temp as u16);
            }else{
                light_step.adjusting_flag = false;
            }
        }

        light_step.time += 1;
        if light_step.time >= LIGHT_ADJUST_INTERVAL as u32 {
            light_step.time = 0;
        }
    }
}

// recover status before software reboot
#[no_mangle]
unsafe fn light_sw_reboot_callback() {
    if rf_slave_ota_busy || is_mesh_ota_slave_running() {	// rf_slave_ota_busy means mesh ota master busy also.
        analog_write__attribute_ram_code (rega_light_off, if light_off {FLD_LIGHT_OFF as u8} else {0});
    }
}

pub unsafe fn is_mesh_cmd_need_delay(p_cmd: *const u8, params: *const u8, ttc: u8) -> bool
{
	let delay_tmp = (*params.offset(1) as u16) | ((*params.offset(2) as u16) << 8);
	if delay_tmp != 0 {
		if cmd_left_delay_ms != 0 {
			return true;
		}
		cmd_delay_ms = delay_tmp;
		if cmd_delay_ms != 0 && irq_timer1_cb_time == 0 {
			let cmd_delayed_ms = light_cmd_delayed_ms(ttc);
			if cmd_delay_ms > cmd_delayed_ms {
				copy_nonoverlapping(p_cmd, addr_of!(cmd_delay) as *mut u8, size_of::<ll_packet_l2cap_data_t>());
				cmd_left_delay_ms = cmd_delay_ms - cmd_delayed_ms;
				irq_timer1_cb_time = clock_time();
				return true;
			}
		}
	}
	return false;
}

#[derive(Clone, Copy)]
#[derive(PartialEq)]
pub enum MESH_PAIR_STATE {
    MESH_PAIR_NAME1 = 0,
    MESH_PAIR_NAME2,
    MESH_PAIR_PWD1,
    MESH_PAIR_PWD2,
    MESH_PAIR_LTK1,
    MESH_PAIR_LTK2,
    MESH_PAIR_EFFECT_DELAY,
    MESH_PAIR_EFFECT,
    MESH_PAIR_DEFAULT_MESH,
}

pub unsafe fn mesh_pair_cb(params: *const u8)
{
    if default_mesh_time_ref != 0 {
        // return;
        default_mesh_time_ref = clock_time() | 1;
    }
	let cmd = *params.offset(0);
    if cmd == MESH_PAIR_STATE::MESH_PAIR_NAME1 as u8 {
        mesh_pair_start_time = clock_time() | 1;
        copy_nonoverlapping(params.offset(1), new_mesh_name.as_mut_ptr(), 8);
    }else if cmd == MESH_PAIR_STATE::MESH_PAIR_NAME2 as u8 {
        copy_nonoverlapping(params.offset(1), new_mesh_name.as_mut_ptr().offset(8), 8);
    }else if cmd == MESH_PAIR_STATE::MESH_PAIR_PWD1 as u8 {
        copy_nonoverlapping(params.offset(1), new_mesh_pwd.as_mut_ptr(), 8);
    }else if cmd == MESH_PAIR_STATE::MESH_PAIR_PWD2 as u8 {
        copy_nonoverlapping(params.offset(1), new_mesh_pwd.as_mut_ptr().offset(8), 8);
    }else if cmd == MESH_PAIR_STATE::MESH_PAIR_LTK1 as u8 {
        copy_nonoverlapping(params.offset(1), new_mesh_ltk.as_mut_ptr(), 8);
    }else if cmd == MESH_PAIR_STATE::MESH_PAIR_LTK2 as u8 {
        copy_nonoverlapping(params.offset(1), new_mesh_ltk.as_mut_ptr().offset(8), 8);
    }else if cmd == MESH_PAIR_STATE::MESH_PAIR_EFFECT_DELAY as u8 {
        effect_new_mesh_delay_time = clock_time() | 1;
        if default_mesh_time_ref != 0 {
            /* Keep default_mesh_time_ref non-zero */
            default_mesh_time = mesh_pair_cmd_interval * 2;
        }
    }else if cmd == MESH_PAIR_STATE::MESH_PAIR_EFFECT as u8 {
        effect_new_mesh = 1;
    }
    else if cmd == MESH_PAIR_STATE::MESH_PAIR_DEFAULT_MESH as u8 {
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
        if is_add_packet_buf_ready() {
            if rf_link_add_tx_packet (addr_of!(pkt_notify) as *const u8) {
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
    pair_setting_flag = PAIR_STATE::PAIR_SETTED;
}

unsafe fn save_effect_new_mesh()
{
    if default_mesh_time_ref != 0 || get_mac_en
    {
		mesh_pair_complete_notify();
		sleep_us(1000);
        /* Switch to normal mesh */
        pair_load_key();
        default_mesh_time_ref = 0;

        mesh_node_init();
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
    pair_save_key();
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
    default_mesh_time = (delay_s as u32 * 1000);

    /* Only change AC and LTK */
    pair_ac = access_code(pair_config_mesh_name.as_ptr(), pair_config_mesh_pwd.as_ptr());
    copy_nonoverlapping(pair_config_mesh_ltk.as_ptr(), pair_ltk.as_mut_ptr(), 16);
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

    if gateway_status == gateway_status_t::GATEWAY_STATUS_CFG_UNPRO_DEV || gateway_status == gateway_status_t::GATEWAY_STATUS_ADD_CONFIGURED_DEVICE
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
            pair_load_key();
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
            pair_load_key();
            default_mesh_time_ref = 0;
        }
    }
    if mesh_pair_start_time != 0 && clock_time_exceed(mesh_pair_start_time, mesh_pair_timeout*1000*1000) {
        //mesh pair time out
        pair_load_key();
        new_mesh_name = [0; 16];
        new_mesh_pwd = [0; 16];
        new_mesh_ltk = [0; 16];
        mesh_pair_state = MESH_PAIR_STATE::MESH_PAIR_NAME1;
        mesh_pair_start_notify_time = 0;
        mesh_pair_retry_cnt = 0;
        mesh_pair_start_time = 0;
        mesh_pair_notify_rsp_mask = [0; 32];
        pair_setting_flag = PAIR_STATE::PAIR_SETTED;
        rf_link_light_event_callback(LGT_CMD_MESH_PAIR_TIMEOUT);
        return;
    }

    if pair_setting_flag == PAIR_STATE::PAIR_SET_MESH_TX_START && mesh_pair_state == MESH_PAIR_STATE::MESH_PAIR_NAME1 && get_online_node_cnt() == 1 {
        op_para[0] = LGT_CMD_MESH_PAIR;
        op_para[3] = MESH_PAIR_STATE::MESH_PAIR_EFFECT as u8;
        dst_addr = 0x0000;// there is noly one device in mesh,just effect itself.
        mesh_pair_state = MESH_PAIR_STATE::MESH_PAIR_NAME1;
        mesh_pair_start_notify_time = 0;
        mesh_pair_retry_cnt = 0;
        mesh_pair_start_time = 0;
        mesh_pair_notify_rsp_mask = [0; 32];
        pair_setting_flag = PAIR_STATE::PAIR_SETTED;
    }else if pair_setting_flag as u8 >= PAIR_STATE::PAIR_SET_MESH_TX_START as u8 && clock_time_exceed(mesh_pair_time, mesh_pair_cmd_interval*1000) {
        mesh_pair_time = clock_time();
        if pair_setting_flag == PAIR_STATE::PAIR_SET_MESH_TX_START {
            op_para[0] = LGT_CMD_MESH_PAIR;
            op_para[3] = mesh_pair_state as u8;
            if mesh_pair_state == MESH_PAIR_STATE::MESH_PAIR_NAME1 {
                // send mesh name [0-7]
                copy_nonoverlapping(pair_nn.as_mut_ptr(), op_para.as_mut_ptr().offset(4), 8);
        		mesh_pair_state = MESH_PAIR_STATE::MESH_PAIR_NAME2;
            }else if mesh_pair_state == MESH_PAIR_STATE::MESH_PAIR_NAME2 {
                // send mesh name [8-15]
                copy_nonoverlapping(pair_nn.as_mut_ptr().offset(8), op_para.as_mut_ptr().offset(4), 8);
        		mesh_pair_state = MESH_PAIR_STATE::MESH_PAIR_PWD1;
            }else if(mesh_pair_state == MESH_PAIR_STATE::MESH_PAIR_PWD1){
                // send mesh pwd [0-7]
                copy_nonoverlapping(pair_pass.as_mut_ptr(), op_para.as_mut_ptr().offset(4), 8);
        		mesh_pair_state = MESH_PAIR_STATE::MESH_PAIR_PWD2;
            }else if(mesh_pair_state == MESH_PAIR_STATE::MESH_PAIR_PWD2){
                // send mesh pwd [8-15]
                copy_nonoverlapping(pair_pass.as_mut_ptr().offset(8), op_para.as_mut_ptr().offset(4), 8);
        		mesh_pair_state = MESH_PAIR_STATE::MESH_PAIR_LTK1;
            }else if(mesh_pair_state == MESH_PAIR_STATE::MESH_PAIR_LTK1){
                // send mesh ltk [0-7]
                copy_nonoverlapping(pair_ltk_mesh.as_mut_ptr(), op_para.as_mut_ptr().offset(4), 8);
        		mesh_pair_state = MESH_PAIR_STATE::MESH_PAIR_LTK2;
            }else if(mesh_pair_state == MESH_PAIR_STATE::MESH_PAIR_LTK2){
                // send mesh ltk [8-15]
                copy_nonoverlapping(pair_ltk_mesh.as_mut_ptr().offset(8), op_para.as_mut_ptr().offset(4), 8);
        		mesh_pair_state = MESH_PAIR_STATE::MESH_PAIR_NAME1;
        		pair_setting_flag = PAIR_STATE::PAIR_SET_MESH_TX_DONE;
            }else{
                mesh_pair_state = MESH_PAIR_STATE::MESH_PAIR_NAME1;
                mesh_pair_start_notify_time = 0;
                mesh_pair_retry_cnt = 0;
                mesh_pair_start_time = 0;
                mesh_pair_notify_rsp_mask = [0; 32];
        		pair_setting_flag = PAIR_STATE::PAIR_SETTED;
        		return;
            }
        }else if pair_setting_flag == PAIR_STATE::PAIR_SET_MESH_TX_DONE {
            // get mesh nodes' confirm value
            //rf_link_slave_read_status_start();
            op_para[0] = LGT_CMD_MESH_OTA_READ;
            op_para[3] = 0x10;// bridge cnt
            op_para[4] = PAR_READ_MESH_PAIR_CONFIRM;
            pair_setting_flag = PAIR_STATE::PAIR_SET_MESH_RX_DONE;
            mesh_pair_start_notify_time = clock_time() | 0;
            for i in 0..8 {
                mesh_pair_checksum[i] = get_mesh_pair_checksum(i as u8);
            }
        }else if pair_setting_flag == PAIR_STATE::PAIR_SET_MESH_RX_DONE {
            let mut effect_flag = mesh_pair_notify_rsp_mask == [0; 32];
            if !effect_flag && clock_time_exceed(mesh_pair_start_time, MESH_PAIR_NOTIFY_TIMEOUT*1000) {
                if mesh_pair_retry_cnt < mesh_pair_retry_max {
                    mesh_pair_start_time = clock_time() | 1;
                    pair_setting_flag = PAIR_STATE::PAIR_SET_MESH_TX_START;
                    mesh_pair_state = MESH_PAIR_STATE::MESH_PAIR_NAME1;
                }else{
                    // retry timeout, effect or cancel?? effect now
                    effect_flag = true;
                }
                mesh_pair_retry_cnt += 1;
            }
            if effect_flag {
                //send cmd to switch to new mesh
                op_para[0] = LGT_CMD_MESH_PAIR;
                op_para[3] = MESH_PAIR_STATE::MESH_PAIR_EFFECT_DELAY as u8;
                mesh_pair_state = MESH_PAIR_STATE::MESH_PAIR_NAME1;
                mesh_pair_start_notify_time = 0;
                mesh_pair_retry_cnt = 0;
                mesh_pair_start_time = 0;
                mesh_pair_notify_rsp_mask = [0; 32];
                pair_setting_flag = PAIR_STATE::PAIR_SETTED;
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