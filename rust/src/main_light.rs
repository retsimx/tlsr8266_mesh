use core::cmp::min;
use core::convert::TryInto;
use core::mem::size_of;
use core::ops::DerefMut;
use core::ptr::addr_of;
use embassy_time::{Duration, Timer};

use fixed::types::I16F16;

use crate::app;
use crate::BIT;
use crate::common::*;
use crate::config::*;
use crate::sdk::ble_app::light_ll::{light_check_tick_per_us, mesh_push_user_command, rf_link_get_op_para, rf_link_slave_pairing_enable, rf_link_slave_proc, vendor_id_init};
use crate::sdk::ble_app::rf_drv_8266::{rf_link_slave_init, rf_set_power_level_index};
use crate::sdk::drivers::flash::{flash_erase_sector, flash_write_page};
use crate::sdk::drivers::pwm::{pwm_set_duty, pwm_start};
use crate::sdk::factory_reset::{factory_reset_cnt_check, factory_reset_handle, kick_out, KickoutReason};
use crate::sdk::light::*;
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1S, CLOCK_SYS_CLOCK_1US, clock_time, clock_time_exceed};
use crate::sdk::mcu::gpio::{AS_GPIO, gpio_set_func};
use crate::sdk::mcu::irq_i::irq_disable;
use crate::sdk::mcu::register::{FLD_IRQ, FLD_TMR, read_reg_irq_mask, read_reg_tmr_ctrl, write_reg_irq_mask, write_reg_tmr0_capt, write_reg_tmr0_tick, write_reg_tmr1_capt, write_reg_tmr_ctrl};
use crate::sdk::pm::{light_sw_reboot, usb_dp_pullup_en};
use crate::sdk::rf_drv::*;
use crate::state::{State, STATE};
use crate::vendor_light::vendor_set_adv_data;
use crate::version::BUILD_VERSION;

pub const LED_INDICATE_VAL: u16 = MAX_LUM_BRIGHTNESS_VALUE;
pub const LED_MASK: u8 = 0x07;

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
// pub const LED_EVENT_FLASH_0P25HZ_1T: u32 = config_led_event!(4, 60, 1, LED_MASK);

fn cfg_led_event(state: &mut State, e: u32) {
    state.led_event_pending = e;
}

pub fn light_hw_timer1_config() {
    // Enable timer1 interrupts for controlling light transitions
    write_reg_irq_mask(read_reg_irq_mask() | FLD_IRQ::TMR1_EN as u32);
    write_reg_tmr1_capt(CLOCK_SYS_CLOCK_1S / 50); // ~ 50hz

    // enable timer0 interrupt for tracking clock_time overflow
    write_reg_irq_mask(read_reg_irq_mask() | FLD_IRQ::TMR0_EN as u32);
    write_reg_tmr0_tick(0);
    write_reg_tmr0_capt(CLOCK_SYS_CLOCK_1S);

    write_reg_tmr_ctrl(read_reg_tmr_ctrl() | FLD_TMR::TMR0_EN as u32);
}

fn light_init_default(state: &mut State) {
    let mut _max_mesh_name_len = 0;
    let len = state.adv_data.len() + size_of::<AdvPrivate>() + 2;
    if len < 31 {
        _max_mesh_name_len = 31 - len - 2;
        state.max_mesh_name_len = if _max_mesh_name_len < 16 {
            _max_mesh_name_len
        } else {
            16
        };
    }

    light_check_tick_per_us(state, CLOCK_SYS_CLOCK_1US);

    state.pair_config_mesh_name.fill(0);
    let len = min(MESH_NAME.len(), _max_mesh_name_len as usize);
    state.pair_config_mesh_name[0..len].copy_from_slice(&MESH_NAME.as_bytes()[0..len]);

    state.pair_config_mesh_pwd.fill(0);
    let len = min(MESH_PWD.len(), 16);
    state.pair_config_mesh_pwd[0..len].copy_from_slice(&MESH_PWD.as_bytes()[0..len]);

    state.pair_config_mesh_ltk[0..16].copy_from_slice(&MESH_LTK[0..16]);

    rf_link_slave_pairing_enable(state, true);
    rf_set_power_level_index(RfPower::RfPower8dBm as u32);

    vendor_id_init(state, VENDOR_ID);

    usb_dp_pullup_en(true);

    light_hw_timer1_config();

    app().mesh_manager.mesh_pair_init(state);
}

pub fn user_init(state: &mut State) {
    // for app ota
    app().ota_manager.check_ota_area_startup();

    light_init_default(state);

    pwm_set_duty(PWMID_G, PMW_MAX_TICK, 0);
    pwm_set_duty(PWMID_B, PMW_MAX_TICK, PMW_MAX_TICK);

    pwm_start(PWMID_G);
    pwm_start(PWMID_B);

    gpio_set_func(PWM_G as u32, !AS_GPIO);
    gpio_set_func(PWM_B as u32, !AS_GPIO);

    //retrieve lumen value
    app().light_manager.light_lum_retrieve(state);

    rf_link_slave_init(state, 40000);

    factory_reset_handle(state);

    vendor_set_adv_data(state);

    app().light_manager.device_status_update(state);
    app().mesh_manager.mesh_security_enable(state, true);
}

fn proc_led(state: &mut State) {


    if state.led_count == 0 && state.led_event_pending == 0 {
        return; //led flash finished
    }

    if state.led_event_pending != 0 {
        // new event
        state.led_ton = (state.led_event_pending & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US;
        state.led_toff = ((state.led_event_pending >> 8) & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US;
        state.led_count = (state.led_event_pending >> 16) & 0xff;
        state.led_sel = state.led_event_pending >> 24;

        state.led_event_pending = 0;
        state.led_tick = clock_time() + 30000000 * CLOCK_SYS_CLOCK_1US;
        state.led_no = 0;
        state.led_is_on = 0;
    }

    if clock_time() - state.led_tick
        >= (if state.led_is_on != 0 {
        state.led_ton
    } else {
        state.led_toff
    })
    {
        state.led_tick = clock_time();
        let led_off = (state.led_is_on != 0 || state.led_ton == 0) && state.led_toff != 0;
        let led_on = state.led_is_on == 0 && state.led_ton != 0;

        state.led_is_on = !state.led_is_on;
        if state.led_is_on != 0 {
            state.led_no += 1;
            if state.led_no - 1 == state.led_count {
                state.led_count = 0;
                state.led_no = 0;
                app().light_manager.light_onoff_hw(!app().light_manager.is_light_off()); // should not report online status again
                return;
            }
        }

        if led_off || led_on {
            if state.led_sel & BIT!(0) != 0 {
                app().light_manager.light_adjust_cw(I16F16::from_num(LED_INDICATE_VAL * led_on as u16), I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE));
            }
            if state.led_sel & BIT!(1) != 0 {
                app().light_manager.light_adjust_ww(I16F16::from_num(LED_INDICATE_VAL * led_on as u16), I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE));
            }
            if state.led_sel & BIT!(5) != 0 {}
        }
    }
}

fn light_auth_check(state: &mut State) {
    if state.security_enable
        && !state.pair_login_ok
        && state.slave_first_connected_tick != 0
        && clock_time_exceed(state.slave_first_connected_tick, AUTH_TIME * 1000 * 1000)
    {
        //rf_link_slave_disconnect(); // must login in 60s after connected, if need
        state.slave_first_connected_tick = 0;
    }
}

fn light_user_func(state: &mut State) {
    app().light_manager.check_light_state_save();

    light_auth_check(state);
    factory_reset_cnt_check(state);
    app().mesh_manager.mesh_pair_proc_effect(state);
}

pub async fn main_loop() {
    Timer::after(Duration::from_micros(LOOP_INTERVAL_US)).await;

    STATE.lock(|state| {
        let mut binding = state.borrow_mut();
        let state = binding.deref_mut();

        light_user_func(state);
        rf_link_slave_proc(state);

        proc_led(state);
    });
}

/*@brief: This function is called in IRQ state, use IRQ stack.
**@param: ppp: is pointer to response
**@param: p_req: is pointer to request command
Called to handle messages that require a response to be returned
*/
pub fn rf_link_response_callback(
    state: &mut State,
    ppp: *mut PacketAttValue,
    p_req: *const PacketAttValue,
) -> bool {
    let ppp = unsafe { &mut (*ppp) };
    let p_req = unsafe { &(*p_req) };
    // mac-app[5] low 2 bytes used as ttc && hop-count
    let dst_unicast = is_unicast_addr(&p_req.dst);
    ppp.dst = p_req.src;
    ppp.src[0] = (state.device_address & 0xff) as u8;
    ppp.src[1] = ((state.device_address >> 8) & 0xff) as u8;

    let params = &p_req.val[3..13];
    ppp.val[3..10 + 3].fill(0);

    ppp.val[1] = (VENDOR_ID & 0xFF) as u8;
    ppp.val[2] = ((VENDOR_ID >> 8) & 0xff) as u8;

    ppp.val[18] = MAX_RELAY_NUM;

    let mut idx = 0;
    match ppp.val[15] {
        GET_STATUS => {
            ppp.val[0] = LGT_CMD_LIGHT_STATUS | 0xc0;

            let state = app().light_manager.get_current_light_state();
            ppp.val[3] = (state.cw.to_num::<u16>() & 0xff) as u8;
            ppp.val[4] = ((state.cw.to_num::<u16>() >> 8) & 0xff) as u8;
            ppp.val[5] = (state.ww.to_num::<u16>() & 0xff) as u8;
            ppp.val[6] = ((state.ww.to_num::<u16>() >> 8) & 0xff) as u8;
            ppp.val[7] = (state.brightness.to_num::<u16>() & 0xff) as u8;
            ppp.val[8] = ((state.brightness.to_num::<u16>() >> 8) & 0xff) as u8;
        }
        GET_GROUP1 => {
            ppp.val[0] = LGT_CMD_LIGHT_GRP_RSP1 | 0xc0;
            for i in 0..MAX_GROUP_NUM as usize {
                ppp.val[i + 3] = 0xFF;
                if state.group_address[i] != 0 {
                    ppp.val[idx + 3] = state.group_address[i] as u8;
                    idx += 1;
                }
            }
        }
        GET_GROUP2 => {
            ppp.val[0] = LGT_CMD_LIGHT_GRP_RSP2 | 0xc0;
            for i in 0..MAX_GROUP_NUM as usize {
                ppp.val[i + 3] = 0xFF;
                if state.group_address[i / 2] != 0 {
                    ppp.val[idx + 3] = if (i % 2) != 0 {
                        (state.group_address[i / 2] >> 8) as u8
                    } else {
                        state.group_address[i / 2] as u8
                    };
                    idx += 1;
                }
            }
        }
        GET_GROUP3 => {
            ppp.val[0] = LGT_CMD_LIGHT_GRP_RSP3 | 0xc0;
            for i in 0..MAX_GROUP_NUM as usize {
                ppp.val[i + 3] = 0xFF;
                if state.group_address[4 + i / 2] != 0 {
                    ppp.val[idx + 3] = if (i % 2) != 0 {
                        (state.group_address[4 + i / 2] >> 8) as u8
                    } else {
                        state.group_address[4 + i / 2] as u8
                    };
                    idx += 1;
                }
            }
        }
        GET_DEV_ADDR => {
            ppp.val[0] = LGT_CMD_DEV_ADDR_RSP | 0xc0;
            return dev_addr_with_mac_rsp(state, &mut ppp.val);
        }
        GET_USER_NOTIFY => {
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
            ppp.val[3] = (state.device_address & 0xFF) as u8;
            ppp.val[4] = ((state.device_address >> 8) & 0xff) as u8;
        },
        CMD_START_OTA => {
            ppp.val[0] = LGT_CMD_START_OTA_RSP | 0xc0;

            ppp.val[3] = BUILD_VERSION as u8;
            ppp.val[4] = (BUILD_VERSION >> 8) as u8;
            ppp.val[5] = (BUILD_VERSION >> 16) as u8;
            ppp.val[6] = (BUILD_VERSION >> 24) as u8;

            state.rf_slave_ota_busy_mesh = true;
        },
        CMD_OTA_DATA => {
            ppp.val[0] = LGT_CMD_OTA_DATA_RSP | 0xc0;

            let idx = app().ota_manager.rf_mesh_data_ota(state, &params, false);

            ppp.val[1] = idx as u8;
            ppp.val[2] = (idx >> 8) as u8;
        },
        CMD_END_OTA => {
            ppp.val[0] = LGT_CMD_END_OTA_RSP | 0xc0;

            let idx = app().ota_manager.rf_mesh_data_ota(state, &params, true);

            ppp.val[1] = idx as u8;
            ppp.val[2] = (idx >> 8) as u8;
        }
        _ => return false
    }

    return true;
}

/*@brief: This function is called in IRQ state, use IRQ stack.
Called to handle messages sent to us that don't require a response
*/
pub fn rf_link_data_callback(state: &mut State, p: *const PacketL2capData) {
    // p start from l2cap_len of RfPacketAttCmdT
    let mut op_cmd: [u8; 3] = [0; 3];
    let mut op_cmd_len: u8 = 0;
    let mut params: [u8; 16] = [0; 16];
    let mut params_len: u8 = 0;
    rf_link_get_op_para(
        unsafe { &*p },
        &mut op_cmd,
        &mut op_cmd_len,
        &mut params,
        &mut params_len,
        true,
    );

    if op_cmd_len != LightOpType::OpType3 as u8 {
        return;
    }

    // Verify that the vendor id is correct. This can catch unexpected corrupt messages
    let vendor_id = (op_cmd[2] as u16) << 8 | op_cmd[1] as u16;
    if vendor_id != VENDOR_ID {
        return;
    }

    let op = op_cmd[0] & 0x3F;

    match op {
        LGT_CMD_LIGHT_ONOFF => app().light_manager.send_message(LGT_CMD_LIGHT_ONOFF, params),
        LGT_CMD_LIGHT_CONFIG_GRP => {
            let val = params[1] as u16 | ((params[2] as u16) << 8);
            match params[0] {
                LIGHT_DEL_GRP_PARAM => {
                    if rf_link_del_group(state, val) {
                        cfg_led_event(state, LED_EVENT_FLASH_1HZ_4S);
                    }
                }
                LIGHT_ADD_GRP_PARAM => {
                    if rf_link_add_group(state, val) {
                        cfg_led_event(state, LED_EVENT_FLASH_1HZ_4S);
                    }
                }
                _ => ()
            }
        }
        LGT_CMD_CONFIG_DEV_ADDR => {
            let val = params[0] as u16 | ((params[1] as u16) << 8);
            if !dev_addr_with_mac_flag(params.as_ptr()) || dev_addr_with_mac_match(state, &params) {
                if rf_link_add_dev_addr(state, val) {
                    app().mesh_manager.mesh_pair_proc_get_mac_flag(state);
                }
            }
        }
        LGT_CMD_SET_LIGHT => app().light_manager.send_message(LGT_CMD_SET_LIGHT, params),
        LGT_CMD_SET_MAC_ADDR => {
            let mac = [params[0], params[1], params[2], params[3], params[4], params[5]];
            flash_erase_sector(FLASH_ADR_MAC);
            flash_write_page(FLASH_ADR_MAC, mac.len() as u32, addr_of!(mac) as *const u8);
            light_sw_reboot(state);
        }
        LGT_CMD_KICK_OUT => {
            irq_disable();
            let res = (params[0] as u32).try_into();
            if res.is_ok() {
                kick_out(state, res.unwrap());
            } else {
                kick_out(state, KickoutReason::OutOfMesh);
            }
            light_sw_reboot(state);
        }
        LGT_CMD_MESH_PAIR => app().mesh_manager.mesh_pair_cb(&params),
        _ => ()
    }
}

// p_cmd : cmd[3]+para[10]
// para    : dst
pub fn light_slave_tx_command(state: &mut State, p_cmd: &[u8], para: u16) -> bool {
    let mut cmd_op_para: [u8; 16] = [0; 16];
    let cmd_sno = clock_time() + state.device_address as u32;

    cmd_op_para[0..13].copy_from_slice(&p_cmd[0..13]);

    cmd_op_para[0] |= 0xc0;
    cmd_op_para[1] = (VENDOR_ID & 0xFF) as u8;
    cmd_op_para[2] = (VENDOR_ID >> 8) as u8;

    let dst = para;
    mesh_push_user_command(state, cmd_sno, dst, cmd_op_para.as_ptr(), 13)
}

pub fn rf_link_light_event_callback(state: &mut State, status: u8) {
    match status {
        LGT_CMD_SET_MESH_INFO => {
            mesh_node_init(state);
            app().light_manager.device_status_update(state);
            cfg_led_event(state, LED_EVENT_FLASH_1HZ_4S);
        }
        LGT_CMD_SET_DEV_ADDR => {
            mesh_node_init(state);
            app().light_manager.device_status_update(state);
            cfg_led_event(state, LED_EVENT_FLASH_1HZ_4S);
        }
        LGT_CMD_DEL_PAIR => cfg_led_event(state, LED_EVENT_FLASH_1HZ_4S),
        LGT_CMD_MESH_PAIR_TIMEOUT => cfg_led_event(state, LED_EVENT_FLASH_2HZ_2S),
        _ => ()
    }
}
