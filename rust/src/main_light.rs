use crate::app;
use std::cmp::{max, min};
use std::convert::TryInto;
use std::mem::size_of;
use std::ptr::addr_of;
use crate::{pub_mut, BIT};

use crate::common::*;
use crate::config::*;
use crate::mesh::MESH_NODE_ST_PAR_LEN;
use crate::sdk::drivers::flash::{flash_erase_sector, flash_write_page};
use crate::sdk::drivers::pwm::{pwm_set_cmp, pwm_set_duty, pwm_start};
use crate::sdk::factory_reset::{factory_reset_cnt_check, factory_reset_handle, kick_out};
use crate::sdk::light::*;
use crate::sdk::mcu::analog::{analog_read__attribute_ram_code, analog_write__attribute_ram_code};
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed, CLOCK_SYS_CLOCK_1US, CLOCK_SYS_CLOCK_HZ, CLOCK_SYS_CLOCK_1S};
use crate::sdk::mcu::gpio::{gpio_set_func, AS_GPIO};
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use crate::sdk::mcu::register::{read_reg_irq_mask, read_reg_tmr_ctrl, write_reg_irq_mask, write_reg_tmr_ctrl, FLD_IRQ, FLD_TMR, write_reg_tmr0_tick, write_reg_tmr0_capt};
use crate::sdk::pm::usb_dp_pullup_en;
use crate::sdk::rf_drv::*;
use crate::sdk::service::*;
use crate::vendor_light::{get_adv_pri_data, get_adv_rsp_pri_data, vendor_set_adv_data};

pub const LED_INDICATE_VAL: u16 = 0xffff;
pub const LED_MASK: u8 = 0x07;
pub const LUM_SAVE_FLAG: u8 = 0xA5;

macro_rules! config_led_event {
    ($on:expr, $off:expr, $n:expr, $sel:expr) => {
        $on as u32 | ($off as u32) << 8 | ($n as u32) << 16 | ($sel as u32) << 24
    };
}

// pub const LED_EVENT_FLASH_4HZ_10S: u32 = config_led_event!(2,2,40,LED_MASK);
// pub const LED_EVENT_FLASH_STOP: u32 = config_led_event!(1,1,1,LED_MASK);
pub const LED_EVENT_FLASH_2HZ_2S: u32 = config_led_event!(4, 4, 4, LED_MASK);
// pub const LED_EVENT_FLASH_1HZ_1S: u32 = config_led_event!(8,8,1,LED_MASK);
// pub const LED_EVENT_FLASH_1HZ_2S: u32 = config_led_event!(8,8,2,LED_MASK);
// pub const LED_EVENT_FLASH_1HZ_3S: u32 = config_led_event!(8,8,3,LED_MASK);
pub const LED_EVENT_FLASH_1HZ_4S: u32 = config_led_event!(8, 8, 4, LED_MASK);
// pub const LED_EVENT_FLASH_4HZ: u32 = config_led_event!(2,2,0,LED_MASK);
// pub const LED_EVENT_FLASH_1HZ: u32 = config_led_event!(8,8,0,LED_MASK);
// pub const LED_EVENT_FLASH_4HZ_3T: u32 = config_led_event!(2,2,3,LED_MASK);
// pub const LED_EVENT_FLASH_1HZ_3T: u32 = config_led_event!(8,8,3,LED_MASK);
pub const LED_EVENT_FLASH_0P25HZ_1T: u32 = config_led_event!(4, 60, 1, LED_MASK);

pub_mut!(buff_response, [[u32; 9]; 48], [[0; 9]; 48]);
pub_mut!(adv_data, [u8; 3]);
pub_mut!(max_mesh_name_len, u8);

pub_mut!(led_event_pending, u32, 0);

pub_mut!(led_count, u32, 0);
pub_mut!(led_ton, u32, 0);
pub_mut!(led_toff, u32, 0);
pub_mut!(led_sel, u32, 0);
pub_mut!(led_tick, u32, 0);
pub_mut!(led_no, u32, 0);
pub_mut!(led_is_on, u32, 0);

pub_mut!(lum_changed_time, u32, 0);
pub_mut!(light_lum_addr, u32, 0);

pub_mut!(light_off, bool, true);
pub_mut!(led_lum, u16, 0xFFFF);
pub_mut!(
    led_val,
    [u16; 3],
    [
        MAX_LUM_BRIGHTNESS_VALUE,
        MAX_LUM_BRIGHTNESS_VALUE,
        MAX_LUM_BRIGHTNESS_VALUE
    ]
);

#[repr(C, packed)]
struct LumSaveT {
    save_falg: u8,
    lum: u16,
    ledval: [u16; 3],
}

fn calculate_lumen_map(val: u16) -> f32 {
    let percentage = (val as f32 / MAX_LUM_BRIGHTNESS_VALUE as f32) * 100.;
    // return keyframe::ease(keyframe::functions::EaseInOutCubic, 0., 0xffff as f32, percentage);
    return (-0.00539160 * libm::powf(percentage, 3.0))
        + (4.47709595 * libm::powf(percentage, 2.0))
        + (153.72442036 * percentage);
}

fn pwm_set_lum(id: u32, y: u32, pol: bool) {
    let lum = (y * PMW_MAX_TICK as u32) / MAX_LUM_BRIGHTNESS_VALUE as u32;

    pwm_set_cmp(id, if pol { PMW_MAX_TICK as u32 - lum } else { lum } as u16);
}

fn get_pwm_cmp(val: u16, lum: u16) -> u32 {
    let val_lumen_map = calculate_lumen_map(lum);

    return ((val as f32 * val_lumen_map) / MAX_LUM_BRIGHTNESS_VALUE as f32) as u32;
}

fn cfg_led_event(e: u32) {
    set_led_event_pending(e);
}

fn mesh_ota_master_led(_: *const u8) {
    if *get_led_count() == 0 && *get_led_event_pending() == 0 {
        cfg_led_event(LED_EVENT_FLASH_0P25HZ_1T);
    }
}

pub fn light_hw_timer1_config() {
    // Enable timer1 interrupts for controlling the lights
    // write_reg_irq_mask(read_reg_irq_mask() | FLD_IRQ::TMR1_EN as u32);
    // write_reg_tmr1_tick(0);
    // write_reg_tmr1_capt(CLOCK_SYS_CLOCK_1MS );
    //
    // write_reg_tmr_ctrl(read_reg_tmr_ctrl() | FLD_TMR::TMR1_EN as u32);

    // enable timer0 interrupt for tracking clock_time overflow
    write_reg_irq_mask(read_reg_irq_mask() | FLD_IRQ::TMR0_EN as u32);
    write_reg_tmr0_tick(0);
    write_reg_tmr0_capt(CLOCK_SYS_CLOCK_1S );

    write_reg_tmr_ctrl(read_reg_tmr_ctrl() | FLD_TMR::TMR0_EN as u32);
}

fn light_init_default() {
    let mut _max_mesh_name_len = 0;
    let len = (get_adv_data().len() + size_of::<ll_adv_private_t>() + 2) as u8;
    if len < 31 {
        _max_mesh_name_len = 31 - len - 2;
        set_max_mesh_name_len(if _max_mesh_name_len < 16 {
            _max_mesh_name_len
        } else {
            16
        });
    }

    // get fw version @flash 0x02,0x03,0x04,0x05
    _mesh_get_fw_version();

    //add the user_data after the adv_pri_data
    let user_const_data: [u8; 6] = [0x05, 0x02, 0x19, 0x00, 0x69, 0x69];
    get_user_data()[0..user_const_data.len()].clone_from_slice(&user_const_data);

    set_user_data_len(0); // disable add the userdata after the adv_pridata

    _light_set_tick_per_us(CLOCK_SYS_CLOCK_HZ / 1000000);

    set_pair_config_valid_flag(PAIR_VALID_FLAG);

    get_pair_config_mesh_name().iter_mut().for_each(|m| *m = 0);
    let len = min(MESH_NAME.len(), _max_mesh_name_len as usize);
    get_pair_config_mesh_name()[0..len].copy_from_slice(&MESH_NAME.as_bytes()[0..len]);

    get_pair_config_mesh_pwd().iter_mut().for_each(|m| *m = 0);
    let len = min(MESH_PWD.len(), 16);
    get_pair_config_mesh_pwd()[0..len].copy_from_slice(&MESH_PWD.as_bytes()[0..len]);

    get_pair_config_mesh_ltk().iter_mut().for_each(|m| *m = 0);
    get_pair_config_mesh_ltk()[0..16].copy_from_slice(&MESH_LTK[0..16]);

    _setSppUUID(
        TELINK_SPP_UUID_SERVICE.as_ptr(),
        TELINK_SPP_DATA_SERVER2CLIENT.as_ptr(),
        TELINK_SPP_DATA_CLIENT2SERVER.as_ptr(),
        TELINK_SPP_DATA_OTA.as_ptr(),
        TELINK_SPP_DATA_PAIR.as_ptr(),
    );

    set_p_adv_pri_data(get_adv_pri_data());
    set_adv_private_data_len(size_of::<ll_adv_private_t>() as u8);
    set_p_adv_rsp_data(get_adv_rsp_pri_data());

    _rf_link_slave_pairing_enable(1);
    _rf_set_power_level_index(RF_POWER::RF_POWER_8dBm as u32);
    _rf_link_slave_set_buffer(get_buff_response().as_mut_ptr(), 48);
    _rf_link_set_max_bridge(BRIDGE_MAX_CNT);

    _vendor_id_init(VENDOR_ID);

    usb_dp_pullup_en(true);

    light_hw_timer1_config();

    set_online_status_timeout(ONLINE_STATUS_TIMEOUT);

    app().mesh_manager.mesh_pair_init();
}

pub fn user_init() {
    // for app ota
    app().ota_manager.check_ota_area_startup();

    light_init_default();

    pwm_set_duty(PWMID_R, PMW_MAX_TICK, PMW_MAX_TICK);
    pwm_set_duty(PWMID_G, PMW_MAX_TICK, PMW_MAX_TICK);
    pwm_set_duty(PWMID_B, PMW_MAX_TICK, 0);

    //retrieve lumen value
    light_lum_retrieve();

    pwm_start(PWMID_R);
    pwm_start(PWMID_G);
    pwm_start(PWMID_B);

    gpio_set_func(PWM_R as u32, !AS_GPIO);
    gpio_set_func(PWM_G as u32, !AS_GPIO);
    gpio_set_func(PWM_B as u32, !AS_GPIO);

    _rf_link_slave_init(40000);

    factory_reset_handle();

    vendor_set_adv_data();

    device_status_update();
    _mesh_security_enable(true);

    _register_mesh_ota_master_ui(mesh_ota_master_led); //  mesh_ota_master_led() will be called when sending mesh ota data.
}

fn proc_led() {
    if *get_led_count() == 0 && *get_led_event_pending() == 0 {
        return; //led flash finished
    }

    if *get_led_event_pending() != 0 {
        // new event
        set_led_ton((*get_led_event_pending() & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US);
        set_led_toff(((*get_led_event_pending() >> 8) & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US);
        set_led_count((*get_led_event_pending() >> 16) & 0xff);
        set_led_sel(*get_led_event_pending() >> 24);

        set_led_event_pending(0);
        set_led_tick(clock_time() + 30000000 * CLOCK_SYS_CLOCK_1US);
        set_led_no(0);
        set_led_is_on(0);
    }

    if clock_time() - *get_led_tick()
        >= (if *get_led_is_on() != 0 {
            *get_led_ton()
        } else {
            *get_led_toff()
        })
    {
        set_led_tick(clock_time());
        let led_off = (*get_led_is_on() != 0 || *get_led_ton() == 0) && *get_led_toff() != 0;
        let led_on = *get_led_is_on() == 0 && *get_led_ton() != 0;

        set_led_is_on(!*get_led_is_on());
        if *get_led_is_on() != 0 {
            set_led_no(*get_led_no() + 1);
            if *get_led_no() - 1 == *get_led_count() {
                set_led_count(0);
                set_led_no(0);
                light_onoff_hw(!*get_light_off()); // should not report online status again
                return;
            }
        }

        if led_off || led_on {
            if *get_led_sel() & BIT!(0) != 0 {
                light_adjust_g(LED_INDICATE_VAL * led_on as u16, 0xffff);
            }
            if *get_led_sel() & BIT!(1) != 0 {
                light_adjust_b(LED_INDICATE_VAL * led_on as u16, 0xffff);
            }
            if *get_led_sel() & BIT!(2) != 0 {
                light_adjust_r(LED_INDICATE_VAL * led_on as u16, 0xffff);
            }
            if *get_led_sel() & BIT!(5) != 0 {}
        }
    }
}

fn light_auth_check() {
    if *get_security_enable()
        && !*get_pair_login_ok()
        && *get_slave_first_connected_tick() != 0
        && clock_time_exceed(*get_slave_first_connected_tick(), AUTH_TIME * 1000 * 1000)
    {
        //rf_link_slave_disconnect(); // must login in 60s after connected, if need
        set_slave_first_connected_tick(0);
    }
}

//erase flash
fn light_lum_erase() {
    set_light_lum_addr(*get_flash_adr_lum());
    flash_erase_sector(*get_flash_adr_lum());
    light_lum_store();
}

//save cur lum value, if disconnected for a while
fn light_lum_store() {
    if *get_light_lum_addr()
        >= (*get_flash_adr_lum() + FLASH_SECTOR_SIZE as u32 - size_of::<LumSaveT>() as u32)
    {
        light_lum_erase();
        return;
    }

    let lum_save = LumSaveT {
        save_falg: LUM_SAVE_FLAG,
        lum: *get_led_lum(),
        ledval: *get_led_val(),
    };

    flash_write_page(
        *get_light_lum_addr(),
        size_of::<LumSaveT>() as u32,
        addr_of!(lum_save) as *const u8,
    );
    set_light_lum_addr(*get_light_lum_addr() + size_of::<LumSaveT>() as u32);
}

//retrieve LUM : brightness or RGB/CT value
fn light_lum_retrieve() {
    let mut i = 0;
    while i < FLASH_SECTOR_SIZE {
        set_light_lum_addr(*get_flash_adr_lum() + i as u32);

        let lum_save = unsafe { &*(*get_light_lum_addr() as *const LumSaveT) };
        if LUM_SAVE_FLAG == (*lum_save).save_falg {
            set_led_lum(lum_save.lum);
            *get_led_val() = lum_save.ledval;
        } else if lum_save.save_falg == 0xFF {
            break;
        }

        i += size_of::<LumSaveT>() as u16
    }

    //effect
    light_adjust_rgb_hw(0, 0, 0, 0);

    app().ota_manager.mesh_ota_master_100_flag_check();

    let val = analog_read__attribute_ram_code(REGA_LIGHT_OFF);
    if val & RecoverStatus::LightOff as u8 != 0 {
        analog_write__attribute_ram_code(REGA_LIGHT_OFF, val & !(RecoverStatus::LightOff as u8));
        light_onoff(false);
    } else {
        light_onoff(true);
    }
}

fn light_user_func() {
    light_auth_check();

    factory_reset_cnt_check();

    // save current lum-val
    if *get_lum_changed_time() != 0 && clock_time_exceed(*get_lum_changed_time(), 5000 * 1000) {
        set_lum_changed_time(0);
        light_lum_store();
    }

    app().mesh_manager.mesh_pair_proc_effect();
}

pub fn main_loop() {
    if _is_receive_ota_window() {
        return;
    }

    light_user_func();
    _rf_link_slave_proc();
    proc_led();
}

/*@brief: This function is called in IRQ state, use IRQ stack.
**@param: ppp: is pointer to response
**@param: p_req: is pointer to request command*/
#[no_mangle] // required by light_ll
fn rf_link_response_callback(
    ppp: *mut rf_packet_att_value_t,
    p_req: *const rf_packet_att_value_t,
) -> bool {
    let ppp = unsafe { &mut (*ppp) };
    let p_req = unsafe { &(*p_req) };
    // mac-app[5] low 2 bytes used as ttc && hop-count
    let dst_unicast = is_unicast_addr(&p_req.dst);
    ppp.dst = p_req.src;
    ppp.src[0] = (*get_device_address() & 0xff) as u8;
    ppp.src[1] = ((*get_device_address() >> 8) & 0xff) as u8;

    let mut params: [u8; 10] = [0; 10];
    params[0..10].copy_from_slice(&ppp.val[3..10 + 3]);
    ppp.val[3..10 + 3].copy_from_slice(&[0; 10]);

    ppp.val[1] = (VENDOR_ID & 0xFF) as u8;
    ppp.val[2] = ((VENDOR_ID >> 8) & 0xff) as u8;

    ppp.val[18] = *get_max_relay_num();

    let mut idx = 0;
    if ppp.val[15] == GET_STATUS {
        ppp.val[0] = LGT_CMD_LIGHT_STATUS | 0xc0;
        for i in 0..3 {
            //params[0]
            ppp.val[i * 2 + 3] = if *get_light_off() {
                0
            } else {
                (get_led_val()[i] & 0xff) as u8
            };
            ppp.val[i * 2 + 4] = if *get_light_off() {
                0
            } else {
                ((get_led_val()[i] >> 8) & 0xff) as u8
            };
        }
    } else if ppp.val[15] == GET_GROUP1 {
        ppp.val[0] = LGT_CMD_LIGHT_GRP_RSP1 | 0xc0;
        for i in 0..MAX_GROUP_NUM as usize {
            ppp.val[i + 3] = 0xFF;
            if get_group_address()[i] != 0 {
                ppp.val[idx + 3] = get_group_address()[i] as u8;
                idx += 1;
            }
        }
    } else if ppp.val[15] == GET_GROUP2 {
        ppp.val[0] = LGT_CMD_LIGHT_GRP_RSP2 | 0xc0;
        for i in 0..MAX_GROUP_NUM as usize {
            ppp.val[i + 3] = 0xFF;
            if get_group_address()[i / 2] != 0 {
                ppp.val[idx + 3] = if (i % 2) != 0 {
                    (get_group_address()[i / 2] >> 8) as u8
                } else {
                    get_group_address()[i / 2] as u8
                };
                idx += 1;
            }
        }
    } else if ppp.val[15] == GET_GROUP3 {
        ppp.val[0] = LGT_CMD_LIGHT_GRP_RSP3 | 0xc0;
        for i in 0..MAX_GROUP_NUM as usize {
            ppp.val[i + 3] = 0xFF;
            if get_group_address()[4 + i / 2] != 0 {
                ppp.val[idx + 3] = if (i % 2) != 0 {
                    (get_group_address()[4 + i / 2] >> 8) as u8
                } else {
                    get_group_address()[4 + i / 2] as u8
                };
                idx += 1;
            }
        }
    } else if ppp.val[15] == GET_DEV_ADDR {
        ppp.val[0] = LGT_CMD_DEV_ADDR_RSP | 0xc0;
        if dev_addr_with_mac_flag(params.as_ptr()) {
            return dev_addr_with_mac_rsp(&params, &mut ppp.val);
        } else {
            ppp.val[3] = (*get_device_address() & 0xFF) as u8;
            ppp.val[4] = ((*get_device_address() >> 8) & 0xff) as u8;
        }
    } else if ppp.val[15] == GET_USER_NOTIFY {
        /*user can get parameters from APP.
        params[0] is relay times.
        params[1 -- 9] is parameters from APP if haved been set by user.

        dst_unicast == 1 means destination address is unicast address.
        */
        if dst_unicast {
            // params[0 -- 9] is valid
        } else {
            // only params[0 -- 4] is valid
        }

        ppp.val[0] = LGT_CMD_USER_NOTIFY_RSP | 0xc0;
        for i in 0..8 {
            //params[2]
            ppp.val[5 + i] = i as u8;
        }
        ppp.val[3] = (*get_device_address() & 0xFF) as u8;
        ppp.val[4] = ((*get_device_address() >> 8) & 0xff) as u8;
    } else if ppp.val[15] == GET_MESH_OTA {
        ppp.val[0] = LGT_CMD_MESH_OTA_READ_RSP | 0xc0;
        if params[1] == PAR_READ_MESH_PAIR_CONFIRM {
            for i in 0..8 {
                ppp.val[5 + i] = app().mesh_manager.get_mesh_pair_checksum_fn(i as u8);
            }
            ppp.val[3] = (*get_device_address() & 0xFF) as u8;
            ppp.val[4] = ((*get_device_address() >> 8) & 0xff) as u8;
            return true;
        }
        return _mesh_ota_slave_set_response(ppp.val[3..].as_mut_ptr(), params[1]);
    } else {
        return false;
    }

    return true;
}

/*@brief: This function is called in IRQ state, use IRQ stack.
*/
#[no_mangle] // required by light_ll
pub fn rf_link_data_callback(p: *const ll_packet_l2cap_data_t) {
    // p start from l2capLen of rf_packet_att_cmd_t
    let mut op_cmd: [u8; 8] = [0; 8];
    let op_cmd_len: u8 = 0;
    let mut params: [u8; 16] = [0; 16];
    let params_len: u8 = 0;
    let p = unsafe { &*p };
    let pp = unsafe { &*(p.value.as_ptr() as *const rf_packet_att_value_t) };
    _rf_link_get_op_para(
        p,
        op_cmd.as_mut_ptr(),
        &op_cmd_len,
        params.as_mut_ptr(),
        &params_len,
        1,
    );

    if op_cmd_len != LightOpType::op_type_3 as u8 {
        return;
    }

    // let vendor_id = (op_cmd[2] as u16) << 8 | op_cmd[1] as u16;
    let op = op_cmd[0] & 0x3F;

    _mesh_ota_timeout_handle(op, params.as_ptr());

    if op == LGT_CMD_LIGHT_ONOFF {
        app().light_manager.send_message(LGT_CMD_LIGHT_ONOFF, params);
    } else if op == LGT_CMD_LIGHT_CONFIG_GRP {
        let val = params[1] as u16 | ((params[2] as u16) << 8);
        if params[0] == LIGHT_DEL_GRP_PARAM {
            if _rf_link_del_group(val) {
                cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
            }
        } else if params[0] == LIGHT_ADD_GRP_PARAM {
            if _rf_link_add_group(val) {
                cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
            }
        }
    } else if op == LGT_CMD_CONFIG_DEV_ADDR {
        let val = params[0] as u16 | ((params[1] as u16) << 8);
        if !dev_addr_with_mac_flag(params.as_ptr()) || dev_addr_with_mac_match(&params) {
            if _rf_link_add_dev_addr(val) {
                app().mesh_manager.mesh_pair_proc_get_mac_flag();
            }
        }
    } else if op == LGT_CMD_SET_LIGHT {
        if params[8] & 0x1 != 0 {
            // Brightness
            let value = (params[1] as u16) << 8 | params[0] as u16;

            set_led_lum(value);

            if *get_light_off() {
                return;
            }

            app().light_manager.begin_transition(get_led_val()[0], get_led_val()[1], get_led_val()[2], value);
        }
        if params[8] & 0x2 != 0 {
            // Temperature
            let value = (params[3] as u16) << 8 | params[2] as u16;

            get_led_val()[0] = 0;
            get_led_val()[1] = MAX_LUM_BRIGHTNESS_VALUE - value;
            get_led_val()[2] = value;

            if *get_light_off() {
                return;
            }

            app().light_manager.begin_transition(get_led_val()[0], get_led_val()[1], get_led_val()[2], *get_led_lum());
        }

        set_lum_changed_time(clock_time());
    } else if op == LGT_CMD_KICK_OUT {
        irq_disable();
        kick_out((params[0] as u32).try_into().unwrap());
        _light_sw_reboot();
    } else if op == LGT_CMD_NOTIFY_MESH {
        light_notify(&pp.val[3..3 + 10], &pp.src);
    } else if op == LGT_CMD_MESH_OTA_DATA {
        let idx = (params[0] as u16) | ((params[1] as u16) << 8);
        if !_is_master_ota_st() {
            // no update firmware for itself
            if CMD_START_MESH_OTA == idx {
                app().ota_manager.mesh_ota_master_start_firmware_from_own();
                //cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
            } else if CMD_STOP_MESH_OTA == idx {
                if _is_mesh_ota_slave_running() {
                    // reboot to initial flash: should be delay to relay command.
                    _mesh_ota_slave_reboot_delay(); // reboot after 320ms
                }
            } else {
                _mesh_ota_slave_save_data(params.as_ptr());
            }
        } else {
            if CMD_STOP_MESH_OTA == idx {
                _mesh_ota_master_cancle(OtaState::MASTER_OTA_REBOOT_ONLY as u8, false);
                //cfg_led_event(LED_EVENT_FLASH_4HZ_3T);
            }
        }
    } else if op == LGT_CMD_MESH_PAIR {
        app().mesh_manager.mesh_pair_cb(&params);
    }
}

// p_cmd : cmd[3]+para[10]
// para    : dst
pub fn light_slave_tx_command(p_cmd: &[u8], para: u16) {
    let mut cmd_op_para: [u8; 16] = [0; 16];
    let cmd_sno = clock_time() + *get_device_address() as u32;

    cmd_op_para[0..13].copy_from_slice(&p_cmd[0..13]);

    cmd_op_para[0] |= 0xc0;
    cmd_op_para[1] = (VENDOR_ID & 0xFF) as u8;
    cmd_op_para[2] = (VENDOR_ID >> 8) as u8;

    let dst = para;
    _mesh_push_user_command(cmd_sno, dst, cmd_op_para.as_ptr(), 13);
}

fn light_notify(p: &[u8], p_src: &[u8]) -> i32 {
    let mut err = -1;
    if *get_slave_link_connected() && *get_pair_login_ok() {
        if p.len() > 10 {
            //max length of par is 10
            return -1;
        }

        get_pkt_light_notify().value[3] = p_src[0];
        get_pkt_light_notify().value[4] = p_src[1];

        // let valptr = get_pkt_light_notify().value.as_mut_ptr().offset(10);
        get_pkt_light_notify().value[10..10 + 10].copy_from_slice(&[0; 10]);
        get_pkt_light_notify().value[10..10 + 10].copy_from_slice(&p[0..p.len()]);

        let r = irq_disable();
        if _is_add_packet_buf_ready() {
            if !_rf_link_add_tx_packet(get_pkt_light_notify_addr() as *const u8) {
                err = 0;
            }
        }
        irq_restore(r);
    }

    return err;
}

#[no_mangle] // required by light_ll
pub fn rf_link_light_event_callback(status: u8) {
    if status == LGT_CMD_SET_MESH_INFO {
        _mesh_node_init();
        device_status_update();
        cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
    } else if status == LGT_CMD_SET_DEV_ADDR {
        _mesh_node_init();
        device_status_update();
        cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
    } else if status == LGT_CMD_DEL_PAIR {
        cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
    } else if status == LGT_CMD_MESH_PAIR_TIMEOUT {
        cfg_led_event(LED_EVENT_FLASH_2HZ_2S);
    }
}

#[no_mangle] // required by light_ll
fn irq_timer1() {}

// Counts any clock_time overflows
static mut CLOCK_TIME_UPPER: u32 = 0;
static mut LAST_CLOCK_TIME: u32 = 0;

unsafe fn check_clock_overflow() -> u32 {
    let time = clock_time();
    if clock_time() < LAST_CLOCK_TIME {
        // Overflow has occurred
        CLOCK_TIME_UPPER += 1;
    }

    LAST_CLOCK_TIME = time;

    time
}

#[no_mangle] // required by light_ll
// This timer is configured to run once per second to check if the internal clock has overflowed.
// this is a workaround in case there are no 'clock_time64' calls between overflows
fn irq_timer0() {
    unsafe { check_clock_overflow(); }
}

pub fn clock_time64() -> u64 {
    unsafe {
        let time = check_clock_overflow();
        return (CLOCK_TIME_UPPER as u64) << 32 | time as u64;
    }
}

#[no_mangle] // required by light_ll
#[allow(non_snake_case)]
fn irq_handler__attribute_ram_code() {
    _irq_light_slave_handler();
}

fn light_onoff_hw(on: bool) {
    app().light_manager.begin_transition(get_led_val()[0], get_led_val()[1], get_led_val()[2], match on {true => 0xffff, false => 0});
}

pub fn light_onoff(on: bool) {
    light_onoff_hw(on);

    device_status_update();
}

pub fn device_status_update() {
    // packet
    let mut st_val_par: [u8; MESH_NODE_ST_PAR_LEN as usize] = [0xff; MESH_NODE_ST_PAR_LEN as usize];

    let value = if *get_light_off() {
        0
    } else {
        max(*get_led_lum(), 1)
    };

    // led_lum should not be 0, because app will take it to be light off
    st_val_par[0] = (value & 0xff) as u8; //Note: bit7 of par[0] have been use internal for FLD_SYNCED
    st_val_par[1] = ((value >> 8) & 0xff) as u8; // rsv
                                                 // end

    _ll_device_status_update(st_val_par.as_ptr(), st_val_par.len() as u8);
}

fn light_adjust_r(val: u16, lum: u16) {
    pwm_set_lum(PWMID_R, get_pwm_cmp(val, lum), false);
}

fn light_adjust_g(val: u16, lum: u16) {
    pwm_set_lum(PWMID_G, get_pwm_cmp(val, lum), false);
}

fn light_adjust_b(val: u16, lum: u16) {
    pwm_set_lum(PWMID_B, get_pwm_cmp(val, lum), true);
}

pub fn light_adjust_rgb_hw(val_r: u16, val_g: u16, val_b: u16, lum: u16) {
    light_adjust_r(val_r, lum);
    light_adjust_g(val_g, lum);
    light_adjust_b(val_b, lum);
}
