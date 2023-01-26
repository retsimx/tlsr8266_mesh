use crate::config::*;
use crate::main_light::*;
use crate::pub_mut;
use crate::sdk::drivers::flash::flash_write_page;
use crate::sdk::light::*;
use crate::sdk::mcu::analog::analog_write__attribute_ram_code;
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use std::ptr::addr_of;
use std::slice;
use crate::mesh::wrappers::get_get_mac_en;
use crate::vendor_light::get_adv_rsp_pri_data;



pub_mut!(conn_update_successed, u8, 0);
pub_mut!(conn_update_cnt, u8, 0);

const UPDATE_CONN_PARA_CNT: u8 = 4;
const CONN_PARA_DATA: [[u16; 3]; UPDATE_CONN_PARA_CNT as usize] = [
    [18, 18 + 16, 200],
    [16, 16 + 16, 200],
    [32, 32 + 16, 200],
    [48, 48 + 16, 200],
];
const SYS_CHN_LISTEN_MESH: [u8; 4] = [2, 12, 23, 34]; //8, 30, 52, 74
pub_mut!(sys_chn_listen, [u8; 4], SYS_CHN_LISTEN_MESH);

pub const REGA_LIGHT_OFF: u8 = 0x3a;

#[no_mangle] // Required by light_ll
pub fn dev_addr_with_mac_flag(params: *const u8) -> bool {
    return DEV_ADDR_PAR_WITH_MAC == unsafe { *params.offset(2) };
}

pub fn dev_addr_with_mac_rsp(params: &[u8], par_rsp: &mut [u8]) -> bool {
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

pub fn dev_addr_with_mac_match(params: &[u8]) -> bool {
    return if params[0] == 0xff && params[1] == 0xff {
        // get
        *get_get_mac_en()
    } else {
        let slave_mac = unsafe { slice::from_raw_parts(*get_slave_p_mac(), 6) };
        for i in 0..6 {
            if params[i] != slave_mac[i] {
                return false;
            }
        }
        return true;
    };
}

#[repr(C, packed)]
pub struct light_step_t {
    pub time: u32,
    pub lum_temp: i32,
    pub lum_dst: i32,
    pub step: u16,
    pub step_mod: u16,
    pub remainder: u16,
    pub adjusting_flag: bool,
}

pub_mut!(
    light_step,
    light_step_t,
    light_step_t {
        time: 0,
        lum_temp: 0,
        lum_dst: 0,
        step: 0,
        step_mod: 0,
        remainder: 0,
        adjusting_flag: false,
    }
);

const LUM_UP: u8 = 0;
const LUM_DOWN: u8 = 1;

const LIGHT_ADJUST_TIME: u16 = 100; //unit: 10ms
const LIGHT_ADJUST_INTERVAL: u16 = 2; // unit :10ms;     min:20ms

fn get_step(direction: u8) {
    get_light_step().remainder = 0; // reset
    if LUM_UP == direction {
        get_light_step().step = ((get_light_step().lum_dst - get_light_step().lum_temp)
            / ((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32))
            as u16;
        get_light_step().step_mod = ((((get_light_step().lum_dst - get_light_step().lum_temp)
            % ((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32))
            * 256)
            / ((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32))
            as u16;
    } else {
        get_light_step().step = ((get_light_step().lum_temp - get_light_step().lum_dst)
            / ((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32))
            as u16;
        get_light_step().step_mod = ((((get_light_step().lum_temp - get_light_step().lum_dst)
            % ((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32))
            * 256)
            / ((LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL) as i32))
            as u16;
    }
}

pub fn light_onoff_step(on: bool) {
    let mut set_flag = true;

    if on {
        if *get_light_off() {
            if !get_light_step().adjusting_flag {
                get_light_step().lum_temp = 0;
            }
            get_light_step().lum_dst = *get_led_lum() as i32;
            get_step(LUM_UP);
        } else {
            set_flag = false;
            light_onoff_normal(true); // make sure on. unnecessary.
        }
        set_light_off(false);
    } else {
        if *get_light_off() {
            set_flag = false;
            light_onoff_normal(false); // make sure off. unnecessary.
        } else {
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
    return;
    let r = irq_disable();
    if !get_light_step().adjusting_flag && target == *get_led_lum() {
        light_adjust_rgb_hw(get_led_val()[0], get_led_val()[1], get_led_val()[2], target);
        irq_restore(r);
        return;
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

fn get_next_lum(direction: u8) {
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
    } else {
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
    return;
    if get_light_step().adjusting_flag {
        if get_light_step().time == 0 {
            if get_light_step().lum_dst != get_light_step().lum_temp {
                if get_light_step().lum_temp < get_light_step().lum_dst {
                    get_next_lum(LUM_UP);
                } else {
                    get_next_lum(LUM_DOWN);
                }
                light_adjust_rgb_hw(
                    get_led_val()[0],
                    get_led_val()[1],
                    get_led_val()[2],
                    get_light_step().lum_temp as u16,
                );
            } else {
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
    if *get_rf_slave_ota_busy() || _is_mesh_ota_slave_running() {
        // rf_slave_ota_busy means mesh ota master busy also.
        analog_write__attribute_ram_code(
            REGA_LIGHT_OFF,
            if *get_light_off() {
                RecoverStatus::LightOff as u8
            } else {
                0
            },
        );
    }
}

// adr:0=flag 16=name 32=pwd 48=ltk
// p:data
// n:length
#[no_mangle] // required by light_ll
fn save_pair_info(adr: u32, p: *const u8, n: u32) {
    flash_write_page(
        *get_flash_adr_pairing() + *get_adr_flash_cfg_idx() + adr,
        n,
        p,
    );
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
        _setup_ble_parameter_start(
            1,
            CONN_PARA_DATA[0][0],
            CONN_PARA_DATA[0][1],
            CONN_PARA_DATA[0][2],
        ); // interval 32: means 40ms;   timeout 200: means 2000ms
        set_conn_update_cnt(*get_conn_update_cnt() + 1);
    }
}

#[no_mangle] // required by light_ll
fn rf_update_conn_para(p: *const u8) -> u8 {
    let pp = unsafe { &*(p as *const rf_pkt_l2cap_sig_connParaUpRsp_t) };
    let sig_conn_param_update_rsp: [u8; 9] = [0x0A, 0x06, 0x00, 0x05, 0x00, 0x13, 0x01, 0x02, 0x00];
    let mut equal = true;
    for i in 0..9 {
        unsafe {
            if *((&pp.rf_len) as *const u8).offset(i) != sig_conn_param_update_rsp[i as usize] {
                equal = false;
            }
        }
    }

    if equal && (((*pp)._type & 0b11) == 2) {
        //l2cap data pkt, start pkt
        if (*pp).result == 0x0000 {
            set_conn_update_cnt(0);
            set_conn_update_successed(1);
        } else if (*pp).result == 0x0001 {
            if *get_conn_update_cnt() >= UPDATE_CONN_PARA_CNT {
                set_conn_update_cnt(0);
            } else {
                _setup_ble_parameter_start(
                    1,
                    CONN_PARA_DATA[*get_conn_update_cnt() as usize][0],
                    CONN_PARA_DATA[*get_conn_update_cnt() as usize][1],
                    CONN_PARA_DATA[*get_conn_update_cnt() as usize][2],
                );
                set_conn_update_cnt(*get_conn_update_cnt() + 1);
            }
        }
    }

    return 0;
}


