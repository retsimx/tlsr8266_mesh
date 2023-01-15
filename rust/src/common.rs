use std::mem::{size_of};
use sdk::drivers::flash::{flash_write_page};
use sdk::light::*;
use sdk::mcu::analog::{analog_write__attribute_ram_code};
use sdk::mcu::clock::{clock_time, clock_time_exceed, sleep_us};
use sdk::mcu::irq_i::{irq_disable, irq_restore};
use std::ptr::addr_of;
use std::slice;
use ::{BIT};
use sdk::mcu::register::{write_reg_rf_irq_status, FLD_RF_IRQ_MASK};
use sdk::rf_drv::rf_set_ble_access_code;
use config::*;
use main_light::*;
use ::pub_mut;
use vendor_light::get_adv_rsp_pri_data;

pub_mut!(mesh_pair_start_time, u32, 0);
pub_mut!(default_mesh_time, u32, 0);
pub_mut!(default_mesh_effect_delay_ref, u32, 0);  /* When receive change to default mesh command, shall delay at least 500ms */
pub_mut!(mesh_pair_start_notify_time, u32, 0);
pub_mut!(mesh_pair_retry_cnt, u8, 0);
pub_mut!(mesh_pair_notify_rsp_mask, [u8; 32], [0; 32]);
pub_mut!(new_mesh_name, [u8; 16], [0; 16]);
pub_mut!(new_mesh_pwd, [u8; 16], [0; 16]);
pub_mut!(new_mesh_ltk, [u8; 16], [0; 16]);
pub_mut!(default_mesh_time_ref, u32, 0);
pub_mut!(effect_new_mesh, u8, 0);
pub_mut!(effect_new_mesh_delay_time, u32, 0);
pub_mut!(mesh_pair_cmd_interval, u32, 0);
pub_mut!(mesh_pair_timeout, u32, 0);
pub_mut!(mesh_pair_enable, bool, false);
pub_mut!(mesh_pair_checksum, [u8; 8], [0; 8]);
pub_mut!(mesh_pair_retry_max, u8, 3);
pub_mut!(get_mac_en, bool, false);

// BEGIN SHIT LIGHT_LL HAX
pub_mut!(mesh_node_mask, [u32; ((MESH_NODE_MAX_NUM + 31) >> 5) as usize], [0; ((MESH_NODE_MAX_NUM + 31) >> 5) as usize]);
pub_mut!(mesh_node_max_num, u16, MESH_NODE_MAX_NUM);
pub_mut!(mesh_node_st_val_len, u8, MESH_NODE_ST_VAL_LEN);
pub_mut!(mesh_node_st_par_len, u8, MESH_NODE_ST_PAR_LEN);
pub_mut!(mesh_node_st_len, u8, size_of::<mesh_node_st_t>() as u8);
// END SHIT LIGHT_LL HAX

pub_mut!(conn_update_successed, u8, 0);
pub_mut!(conn_update_cnt, u8, 0);

const UPDATE_CONN_PARA_CNT: u8 = 4;
const CONN_PARA_DATA: [[u16; 3]; UPDATE_CONN_PARA_CNT as usize] = [[18, 18+16, 200], [16, 16+16, 200], [32, 32+16, 200], [48, 48+16, 200]];
const SYS_CHN_LISTEN_MESH: [u8; 4] = [2, 12, 23, 34];	//8, 30, 52, 74
pub_mut!(sys_chn_listen, [u8; 4], SYS_CHN_LISTEN_MESH);

pub_mut!(mesh_pair_time, u32, 0);
pub_mut!(mesh_pair_state, MeshPairState, MeshPairState::MeshPairName1);
pub_mut!(mesh_node_st, [mesh_node_st_t; MESH_NODE_MAX_NUM as usize], [mesh_node_st_t{ tick: 0, val: mesh_node_st_val_t {
    dev_adr: 0,
    sn: 0,
    par: [0; MESH_NODE_ST_PAR_LEN as usize],
} }; MESH_NODE_MAX_NUM as usize]);

#[derive(PartialEq)]
#[allow(dead_code)]
pub enum GatewayStatus
{
    GatewayStatusNormal = 0,             /* Normal gateway role */
    GatewayStatusNodeRole,              /* As node role, when pushed button */

    GatewayStatusTempDefaltMesh,       /* In default mesh temporary */
    GatewayStatusSwitchToDefaultMesh,
    GatewayStatusScanUnprovDev,        /* Scanning unpair device status */
    GatewayStatusCfgUnproDev,          /* Only provision device */
    GatewayStatusCfgCurNetwork,        /* Change current network's information */
    GatewayStatusAddConfiguredDevice,  /* Add configured device */
}

pub_mut!(gateway_status, GatewayStatus, GatewayStatus::GatewayStatusNormal);

pub const MESH_PAIR_CMD_INTERVAL: u32 = 500;

//unit: s
pub const MESH_PAIR_TIMEOUT: u32 = 10;
//unit: ms
pub const MESH_PAIR_NOTIFY_TIMEOUT: u32 = 2500;

pub const MESH_NODE_ST_VAL_LEN: u8 = 4;       // MIN: 4,   MAX: 10
pub const MESH_NODE_ST_PAR_LEN: u8 = MESH_NODE_ST_VAL_LEN - 2;

pub const REGA_LIGHT_OFF: u8 = 0x3a;

pub const MESH_NODE_MAX_NUM: u16 = 64;

pub_mut!(pkt_terminate, [u8; 8], [0x04, 0x00, 0x00, 0x00, 0x03, 0x02, 0x02, 0x13]);

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

#[derive(PartialEq)]
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub enum PairState {
    PairSetted = 0,
    PairSetting,
    PairSetMeshTxStart,
    PairSetMeshTxDone,// send notify req, get mesh nodes' ac
    PairSetMeshRxDone,// received all mesh nodes' ac, send cmd to switch to new mesh
}

pub fn mesh_pair_init() {
    set_mesh_pair_enable(true);
    set_mesh_pair_cmd_interval(MESH_PAIR_CMD_INTERVAL);
    set_mesh_pair_timeout(MESH_PAIR_TIMEOUT);
}

pub fn mesh_pair_proc_effect() {
    if *get_effect_new_mesh() != 0 || (*get_effect_new_mesh_delay_time() != 0 && (clock_time_exceed(*get_effect_new_mesh_delay_time(), *get_mesh_pair_cmd_interval() * 1000))) {
        save_effect_new_mesh();
        set_effect_new_mesh(0);
        set_effect_new_mesh_delay_time(0);
    }
}

#[no_mangle] // Required by light_ll
pub fn dev_addr_with_mac_flag(params: *const u8) -> bool
{
	return DEV_ADDR_PAR_WITH_MAC == unsafe { *params.offset(2) };
}

pub fn dev_addr_with_mac_rsp(params: &[u8], par_rsp: &mut [u8]) -> bool
{
	if dev_addr_with_mac_match(params) {
		par_rsp[0] = (*get_device_address() & 0xff) as u8;
		par_rsp[1] = ((*get_device_address() >> 8) & 0xff) as u8;

        let slave_mac = unsafe { slice::from_raw_parts(*get_slave_p_mac(), 6) };
        par_rsp[0..6].copy_from_slice(slave_mac);
        par_rsp[8] = (get_adv_rsp_pri_data().ProductUUID & 0xff) as u8;
        par_rsp[9] = (get_adv_rsp_pri_data().ProductUUID >> 8 & 0xff) as u8;
		return true;
	}
	return false;
}

pub fn dev_addr_with_mac_match(params: &[u8]) -> bool
{
	return if params[0] == 0xff && params[1] == 0xff {    // get
		*get_get_mac_en()
	} else {
        let slave_mac = unsafe { slice::from_raw_parts(*get_slave_p_mac(), 6) };
		for i in 0..6 {
			if params[i] != slave_mac[i] {
				return false;
			}
		}
		return true;
	}
}

pub fn get_mesh_pair_checksum_fn(idx: u8) -> u8 {
    let i = (idx % 8) as usize;
    return (get_new_mesh_name()[i] ^ get_new_mesh_name()[i+8]) ^ (get_new_mesh_pwd()[i] ^ get_new_mesh_pwd()[i+8]) ^ (get_new_mesh_ltk()[i] ^ get_new_mesh_ltk()[i+8]);
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

pub fn mesh_pair_proc_get_mac_flag(){
	set_get_mac_en(false); 	// set success
	if *get_mesh_pair_enable() {
		let mut data: [u8; 1] = [0];
		flash_write_page(*get_flash_adr_pairing() + *get_adr_flash_cfg_idx() + 1, 1, data.as_mut_ptr());
		set_get_mac_en(if data[0] == 1 { true } else { false });
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

pub_mut!(light_step, light_step_t, light_step_t {
	time: 0,
	lum_temp: 0,
	lum_dst: 0,
	step: 0,
	step_mod: 0,
	remainder: 0,
	adjusting_flag: false,
});

const LUM_UP: u8 = 0;
const LUM_DOWN: u8 = 1;

const LIGHT_ADJUST_TIME     : u16 = 100;   //unit: 10ms
const LIGHT_ADJUST_INTERVAL : u16 = 2;  // unit :10ms;     min:20ms

fn get_step(direction: u8){
    get_light_step().remainder = 0;       // reset
    if LUM_UP == direction {
        get_light_step().step = ((get_light_step().lum_dst - get_light_step().lum_temp)/((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32)) as u16;
        get_light_step().step_mod = ((((get_light_step().lum_dst - get_light_step().lum_temp)%((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32))*256)/((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32)) as u16;
    } else {
        get_light_step().step = ((get_light_step().lum_temp - get_light_step().lum_dst)/((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32)) as u16;
        get_light_step().step_mod = ((((get_light_step().lum_temp - get_light_step().lum_dst)%((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32))*256)/((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32)) as u16;
    }
}

pub fn light_onoff_step(on: bool){
    let mut set_flag= true;

    if on {
        if *get_light_off() {
            if !get_light_step().adjusting_flag {
                get_light_step().lum_temp = 0;
            }
            get_light_step().lum_dst = *get_led_lum() as i32;
            get_step(LUM_UP);
    	}else{
    	    set_flag = false;
    	    light_onoff_normal(true); // make sure on. unnecessary.
    	}
        set_light_off(false);
	}else{
        if *get_light_off() {
    	    set_flag = false;
    	    light_onoff_normal(false); // make sure off. unnecessary.
    	}else{
            if !get_light_step().adjusting_flag {
                get_light_step().lum_temp = *get_led_lum() as i32;
            }
            get_light_step().lum_dst = 0;
            get_step(LUM_DOWN);
    	}
        set_light_off(true);
	}

    get_light_step().adjusting_flag = set_flag;
    get_light_step().time = 0;
}

pub fn light_step_reset(target: u16) {
    let r = irq_disable();
    if !get_light_step().adjusting_flag && target == *get_led_lum() {
        light_adjust_rgb_hw(get_led_val()[0], get_led_val()[1], get_led_val()[2], target);
        irq_restore(r);
		return
    }

    if get_light_step().adjusting_flag {
        if (target as i32) < get_light_step().lum_temp {
            get_light_step().lum_dst = target as i32;
            get_step(LUM_DOWN);
        }

        if (target as i32) > get_light_step().lum_temp {
            get_light_step().lum_dst = target as i32;
            get_step(LUM_UP);
        }
    } else {
        if target < *get_led_lum() {
            get_light_step().lum_temp = *get_led_lum() as i32;
            get_light_step().lum_dst = target as i32;
            get_step(LUM_DOWN);
        }

        if target > *get_led_lum() {
            get_light_step().lum_temp = *get_led_lum() as i32;
            get_light_step().lum_dst = target as i32;
            get_step(LUM_UP);
        }
    }

    get_light_step().adjusting_flag = true;
    get_light_step().time = 0;
    set_led_lum(target);

    irq_restore(r);
}

fn get_next_lum(direction: u8){
    let temp = get_light_step().remainder as u32 + get_light_step().step_mod as u32;
    get_light_step().remainder = (temp & 0xffff) as u16;

    if LUM_UP == direction {
        get_light_step().lum_temp += get_light_step().step as i32;
        if temp >= 0x10000 {
            get_light_step().lum_temp += 1;
        }
        if get_light_step().lum_temp >= get_light_step().lum_dst {
            get_light_step().lum_temp = get_light_step().lum_dst;
            get_light_step().remainder = 0;
        }
    }else{
        get_light_step().lum_temp -= get_light_step().step as i32;
        if temp >= 0x10000 {
            get_light_step().lum_temp -= 1;
        }
        if get_light_step().lum_temp <= get_light_step().lum_dst {
            get_light_step().lum_temp = get_light_step().lum_dst;
            get_light_step().remainder = 0;
        }
    }
}

pub fn light_onoff_step_timer() {
    if get_light_step().adjusting_flag {
        if get_light_step().time == 0 {
            if get_light_step().lum_dst != get_light_step().lum_temp {
                if get_light_step().lum_temp < get_light_step().lum_dst {
                    get_next_lum(LUM_UP);
                }else{
                    get_next_lum(LUM_DOWN);
                }
                light_adjust_rgb_hw(get_led_val()[0], get_led_val()[1], get_led_val()[2], get_light_step().lum_temp as u16);
            }else{
                get_light_step().adjusting_flag = false;
            }
        }

        get_light_step().time += 1;
        if get_light_step().time >= LIGHT_ADJUST_INTERVAL as u32 {
            get_light_step().time = 0;
        }
    }
}

// recover status before software reboot
#[no_mangle] // required by light_ll
fn light_sw_reboot_callback() {
    if *get_rf_slave_ota_busy() || _is_mesh_ota_slave_running() {	// rf_slave_ota_busy means mesh ota master busy also.
        analog_write__attribute_ram_code (REGA_LIGHT_OFF, if *get_light_off() {RecoverStatus::LightOff as u8} else {0});
    }
}

pub fn is_mesh_cmd_need_delay(p_cmd: *const ll_packet_l2cap_data_t, params: &[u8], ttc: u8) -> bool
{
	let delay_tmp = params[1] as u16 | (params[2] as u16) << 8;
	if delay_tmp != 0 {
		if *get_cmd_left_delay_ms() != 0 {
			return true;
		}
		set_cmd_delay_ms(delay_tmp);
		if *get_cmd_delay_ms() != 0 && *get_irq_timer1_cb_time() == 0 {
			let cmd_delayed_ms = light_cmd_delayed_ms(ttc);
			if *get_cmd_delay_ms() > cmd_delayed_ms {
                unsafe { *get_cmd_delay() = *p_cmd; }
				set_cmd_left_delay_ms(*get_cmd_delay_ms() - cmd_delayed_ms);
				set_irq_timer1_cb_time(clock_time());
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

pub fn mesh_pair_cb(params: &[u8])
{
    if *get_default_mesh_time_ref() != 0 {
        set_default_mesh_time_ref(clock_time() | 1);
    }
	let cmd = params[0];
    if cmd == MeshPairState::MeshPairName1 as u8 {
        set_mesh_pair_start_time(clock_time() | 1);
        get_new_mesh_name()[0..8].copy_from_slice(&params[1..9]);
    }else if cmd == MeshPairState::MeshPairName2 as u8 {
        get_new_mesh_name()[8..16].copy_from_slice(&params[1..9]);
    }else if cmd == MeshPairState::MeshPairPwd1 as u8 {
        get_new_mesh_pwd()[0..8].copy_from_slice(&params[1..9]);
    }else if cmd == MeshPairState::MeshPairPwd2 as u8 {
        get_new_mesh_pwd()[8..16].copy_from_slice(&params[1..9]);
    }else if cmd == MeshPairState::MeshPairLtk1 as u8 {
        get_new_mesh_ltk()[0..8].copy_from_slice(&params[1..9]);
    }else if cmd == MeshPairState::MeshPairLtk2 as u8 {
        get_new_mesh_ltk()[8..16].copy_from_slice(&params[1..9]);
    }else if cmd == MeshPairState::MeshPairEffectDelay as u8 {
        set_effect_new_mesh_delay_time(clock_time() | 1);
        if *get_default_mesh_time_ref() != 0 {
            /* Keep default_mesh_time_ref non-zero */
            set_default_mesh_time(*get_mesh_pair_cmd_interval() * 2);
        }
    }else if cmd == MeshPairState::MeshPairEffect as u8 {
        set_effect_new_mesh(1);
    }
    else if cmd == MeshPairState::MeshPairDefaultMesh as u8 {
        set_default_mesh_effect_delay_ref(clock_time() | 1);
        set_default_mesh_time(params[1] as u32 * 1000);
    }
}

fn mesh_cmd_notify(op: u8, p: &[u8], dev_adr: u16) -> i32
{
    let mut err = -1;
    if *get_slave_link_connected() && *get_pair_login_ok() {
        if p.len() > 10 {   //max length of par is 10
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

        pkt_notify.value[10..10+p.len()].copy_from_slice(&p[0..p.len()]);

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

fn mesh_pair_complete_notify() -> i32
{
	let par = [CMD_NOTIFY_MESH_PAIR_END];
	return mesh_cmd_notify(LGT_CMD_MESH_CMD_NOTIFY, &par, *get_device_address());
}

fn _safe_effect_new_mesh_finish() {
    set_new_mesh_name([0; 16]);
    set_new_mesh_pwd([0; 16]);
    set_new_mesh_ltk([0; 16]);
    set_mesh_pair_start_notify_time(0);
    set_mesh_pair_retry_cnt(0);
    set_mesh_pair_start_time(0);
    set_mesh_pair_notify_rsp_mask([0; 32]);
    set_pair_setting_flag(PairState::PairSetted);
}

fn save_effect_new_mesh()
{
    if *get_default_mesh_time_ref() != 0 || *get_get_mac_en()
    {
		mesh_pair_complete_notify();
		sleep_us(1000);
        /* Switch to normal mesh */
        _pair_load_key();
        set_default_mesh_time_ref(0);

        _mesh_node_init();
        device_status_update();
        _safe_effect_new_mesh_finish();
        return;
    }

    if *get_effect_new_mesh() == 0 {
        get_pair_nn()[0..16].copy_from_slice(&get_new_mesh_name()[0..16]);
        get_pair_pass()[0..16].copy_from_slice(&get_new_mesh_pwd()[0..16]);
        get_pair_ltk()[0..16].copy_from_slice(&get_new_mesh_ltk()[0..16]);
    }
    else
    {
        get_pair_ltk()[0..16].copy_from_slice(&get_pair_ltk_mesh()[0..16]);
    }

	mesh_pair_complete_notify();

    // make sure not receive legacy mesh data from now on
    let r = irq_disable();
    _pair_save_key();
    rf_set_ble_access_code (get_pair_ac_addr() as *const u8);// use new access code at once.
    rf_link_light_event_callback(LGT_CMD_SET_MESH_INFO);	// clear online status :mesh_node_init()
	sleep_us (1000);
    write_reg_rf_irq_status(FLD_RF_IRQ_MASK::IRQ_RX as u16);		// clear current rx in buffer
    irq_restore(r);

    _safe_effect_new_mesh_finish();
}

#[no_mangle] // Required by light_ll rf_link_slave_add_status_ll
pub fn mesh_pair_notify_refresh(p: *const rf_packet_att_cmd_t) -> u8 {
    let p = &(unsafe { *p });

    if get_mesh_pair_checksum()[0..8] == p.value[12..12+8] {
        // mesh pair : success one device, clear the mask flag
        get_mesh_pair_notify_rsp_mask()[(p.value[10] / 8) as usize] &= !(BIT!(p.value[10] % 8));
    }

    return 1;// if return 2, then the notify rsp will not report to master.
}

fn switch_to_default_mesh(delay_s: u8)
{
    set_default_mesh_time_ref(clock_time() | 1);
    set_default_mesh_time(delay_s as u32 * 1000);

    /* Only change AC and LTK */
    set_pair_ac(_access_code(get_pair_config_mesh_name().as_ptr(), get_pair_config_mesh_pwd().as_ptr()));
    get_pair_ltk()[0..16].copy_from_slice(&get_pair_config_mesh_ltk()[0..16]);
}

fn get_online_node_cnt() -> u8
{
    let mut cnt = 0;
	for i in 0..*get_mesh_node_max() {
	    if get_mesh_node_st()[i as usize].tick != 0 {
	        cnt += 0;
	        if i > 0 {
	            get_mesh_pair_notify_rsp_mask()[(get_mesh_node_st()[i as usize].val.dev_adr / 8) as usize] |= BIT!(get_mesh_node_st()[i as usize].val.dev_adr % 8);
	        }
	    }
	}

    if *get_gateway_status() == GatewayStatus::GatewayStatusCfgUnproDev || *get_gateway_status() == GatewayStatus::GatewayStatusAddConfiguredDevice
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
fn mesh_pair_proc()
{
    let mut dst_addr = 0xFFFF;
    let mut op_para: [u8; 16] = [0; 16];

    if *get_default_mesh_effect_delay_ref() != 0 && clock_time_exceed(*get_default_mesh_effect_delay_ref(), MESH_PAIR_CMD_INTERVAL * 1000)
    {
        set_default_mesh_effect_delay_ref(0);

        if *get_default_mesh_time() == 0x00
        {
            _pair_load_key();
            set_default_mesh_time_ref(0);
        }
        else
        {
            switch_to_default_mesh((*get_default_mesh_time() / 1000) as u8);
            set_default_mesh_time_ref(clock_time() | 1);
        }
    }
    else if *get_default_mesh_time_ref() != 0 && clock_time_exceed(*get_default_mesh_time_ref(), *get_default_mesh_time() * 1000)
    {
        /* Switch to normal mesh */
        if *get_default_mesh_time() == 255000
        {
            set_default_mesh_time_ref(clock_time() | 1);
        }
        else
        {
            _pair_load_key();
            set_default_mesh_time_ref(0);
        }
    }
    if *get_mesh_pair_start_time() != 0 && clock_time_exceed(*get_mesh_pair_start_time(), *get_mesh_pair_timeout()*1000*1000) {
        //mesh pair time out
        _pair_load_key();
        set_new_mesh_name([0; 16]);
        set_new_mesh_pwd([0; 16]);
        set_new_mesh_ltk([0; 16]);
        set_mesh_pair_state(MeshPairState::MeshPairName1);
        set_mesh_pair_start_notify_time(0);
        set_mesh_pair_retry_cnt(0);
        set_mesh_pair_start_time(0);
        set_mesh_pair_notify_rsp_mask([0; 32]);
        set_pair_setting_flag(PairState::PairSetted);
        rf_link_light_event_callback(LGT_CMD_MESH_PAIR_TIMEOUT);
        return;
    }

    if *get_pair_setting_flag() == PairState::PairSetMeshTxStart && *get_mesh_pair_state() == MeshPairState::MeshPairName1 && get_online_node_cnt() == 1 {
        op_para[0] = LGT_CMD_MESH_PAIR;
        op_para[3] = MeshPairState::MeshPairEffect as u8;
        dst_addr = 0x0000;// there is noly one device in mesh,just effect itself.
        set_mesh_pair_state(MeshPairState::MeshPairName1);
        set_mesh_pair_start_notify_time(0);
        set_mesh_pair_retry_cnt(0);
        set_mesh_pair_start_time(0);
        set_mesh_pair_notify_rsp_mask([0; 32]);
        set_pair_setting_flag(PairState::PairSetted);
    }else if *get_pair_setting_flag() as u8 >= PairState::PairSetMeshTxStart as u8 && clock_time_exceed(*get_mesh_pair_time(), *get_mesh_pair_cmd_interval()*1000) {
        set_mesh_pair_time(clock_time());
        if *get_pair_setting_flag() == PairState::PairSetMeshTxStart {
            op_para[0] = LGT_CMD_MESH_PAIR;
            op_para[3] = *get_mesh_pair_state() as u8;
            if *get_mesh_pair_state() == MeshPairState::MeshPairName1 {
                // send mesh name [0-7]
                op_para[4..4+8].copy_from_slice(&get_pair_nn()[0..8]);
        		set_mesh_pair_state(MeshPairState::MeshPairName2);
            }else if *get_mesh_pair_state() == MeshPairState::MeshPairName2 {
                // send mesh name [8-15]
                op_para[4..4+8].copy_from_slice(&get_pair_nn()[8..16]);
        		set_mesh_pair_state(MeshPairState::MeshPairPwd1);
            }else if *get_mesh_pair_state() == MeshPairState::MeshPairPwd1 {
                // send mesh pwd [0-7]
                op_para[4..4+8].copy_from_slice(&get_pair_pass()[0..8]);
        		set_mesh_pair_state(MeshPairState::MeshPairPwd2);
            }else if *get_mesh_pair_state() == MeshPairState::MeshPairPwd2 {
                // send mesh pwd [8-15]
                op_para[4..4+8].copy_from_slice(&get_pair_pass()[8..16]);
        		set_mesh_pair_state(MeshPairState::MeshPairLtk1);
            }else if *get_mesh_pair_state() == MeshPairState::MeshPairLtk1 {
                // send mesh ltk [0-7]
                op_para[4..4+8].copy_from_slice(&get_pair_ltk_mesh()[0..8]);
        		set_mesh_pair_state(MeshPairState::MeshPairLtk2);
            }else if *get_mesh_pair_state() == MeshPairState::MeshPairLtk2 {
                // send mesh ltk [8-15]
                op_para[4..4+8].copy_from_slice(&get_pair_ltk_mesh()[8..16]);
        		set_mesh_pair_state(MeshPairState::MeshPairName1);
        		set_pair_setting_flag(PairState::PairSetMeshTxDone);
            }else{
                set_mesh_pair_state(MeshPairState::MeshPairName1);
                set_mesh_pair_start_notify_time(0);
                set_mesh_pair_retry_cnt(0);
                set_mesh_pair_start_time(0);
                set_mesh_pair_notify_rsp_mask([0; 32]);
        		set_pair_setting_flag(PairState::PairSetted);
        		return;
            }
        }else if *get_pair_setting_flag() == PairState::PairSetMeshTxDone {
            // get mesh nodes' confirm value
            //rf_link_slave_read_status_start();
            op_para[0] = LGT_CMD_MESH_OTA_READ;
            op_para[3] = 0x10;// bridge cnt
            op_para[4] = PAR_READ_MESH_PAIR_CONFIRM;
            set_pair_setting_flag(PairState::PairSetMeshRxDone);
            set_mesh_pair_start_notify_time(clock_time() | 0);
            get_mesh_pair_checksum().iter_mut().enumerate().for_each(|(i, x) | {*x = get_mesh_pair_checksum_fn(i as u8)});
        }else if *get_pair_setting_flag() == PairState::PairSetMeshRxDone {
            let mut effect_flag = *get_mesh_pair_notify_rsp_mask() == [0; 32];
            if !effect_flag && clock_time_exceed(*get_mesh_pair_start_time(), MESH_PAIR_NOTIFY_TIMEOUT*1000) {
                if *get_mesh_pair_retry_cnt() < *get_mesh_pair_retry_max() {
                    set_mesh_pair_start_time(clock_time() | 1);
                    set_pair_setting_flag(PairState::PairSetMeshTxStart);
                    set_mesh_pair_state(MeshPairState::MeshPairName1);
                }else{
                    // retry timeout, effect or cancel?? effect now
                    effect_flag = true;
                }
                set_mesh_pair_retry_cnt(*get_mesh_pair_retry_cnt() + 1);
            }
            if effect_flag {
                //send cmd to switch to new mesh
                op_para[0] = LGT_CMD_MESH_PAIR;
                op_para[3] = MeshPairState::MeshPairEffectDelay as u8;
                set_mesh_pair_state(MeshPairState::MeshPairName1);
                set_mesh_pair_start_notify_time(0);
                set_mesh_pair_retry_cnt(0);
                set_mesh_pair_start_time(0);
                set_mesh_pair_notify_rsp_mask([0; 32]);
                set_pair_setting_flag(PairState::PairSetted);
            }
        }
    }else{
        return;
    }

    light_slave_tx_command(&op_para, dst_addr);
}

#[no_mangle] // required by light_ll
fn mesh_node_buf_init()
{
    [mesh_node_st_t{ tick: 0, val: mesh_node_st_val_t {
        dev_adr: 0,
        sn: 0,
        par: [0; MESH_NODE_ST_PAR_LEN as usize],
    } }; MESH_NODE_MAX_NUM as usize];

	device_status_update();
}

// adr:0=flag 16=name 32=pwd 48=ltk
// p:data
// n:length
#[no_mangle] // required by light_ll
fn save_pair_info(adr: u32, p: *const u8, n: u32)
{
	flash_write_page (*get_flash_adr_pairing() + *get_adr_flash_cfg_idx() + adr, n, p);
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
#[no_mangle] // required by light_ll
fn update_ble_parameter_cb() {
   if *get_conn_update_successed() == 0 {
       _setup_ble_parameter_start(1, CONN_PARA_DATA[0][0], CONN_PARA_DATA[0][1], CONN_PARA_DATA[0][2]);  // interval 32: means 40ms;   timeout 200: means 2000ms
       set_conn_update_cnt(*get_conn_update_cnt() + 1);
   }
}

#[no_mangle] // required by light_ll
fn rf_update_conn_para(p: *const u8) -> u8
{
   let pp = unsafe { &*(p as *const rf_pkt_l2cap_sig_connParaUpRsp_t) };
   let sig_conn_param_update_rsp: [u8; 9] = [0x0A, 0x06, 0x00, 0x05, 0x00, 0x13, 0x01, 0x02, 0x00];
    let mut equal = true;
    for i in 0..9
    {
        unsafe {
            if *((&pp.rf_len) as *const u8).offset(i) != sig_conn_param_update_rsp[i as usize] {
                equal = false;
            }
        }
    }

   if equal && (((*pp)._type & 0b11) == 2){//l2cap data pkt, start pkt
       if (*pp).result == 0x0000 {
           set_conn_update_cnt(0);
           set_conn_update_successed(1);
       }else if (*pp).result == 0x0001 {
           if *get_conn_update_cnt() >= UPDATE_CONN_PARA_CNT {
               set_conn_update_cnt(0);
           }else{
               _setup_ble_parameter_start(
                   1,
                   CONN_PARA_DATA[*get_conn_update_cnt() as usize][0],
                   CONN_PARA_DATA[*get_conn_update_cnt() as usize][1],
                   CONN_PARA_DATA[*get_conn_update_cnt() as usize][2]
               );
               set_conn_update_cnt(*get_conn_update_cnt() + 1);
           }
       }
   }

   return 0;
}

#[no_mangle] // required by light_ll
fn light_slave_tx_command_callback (p: *const u8) {
    _rf_link_data_callback(p);
}
