use core::cmp::min;
use core::convert::TryInto;
use core::mem::size_of;
use core::ptr::addr_of;

use embassy_time::{Duration, Timer};
use fixed::types::I16F16;

use crate::app;
use crate::BIT;
use crate::common::*;
use crate::config::*;
use crate::sdk::ble_app::light_ll::{light_check_tick_per_us, mesh_construct_packet, rf_link_get_op_para, rf_link_slave_pairing_enable, rf_link_slave_proc};
use crate::sdk::ble_app::rf_drv_8266::{rf_link_slave_init, rf_set_power_level_index};
use crate::sdk::drivers::flash::{flash_erase_sector, flash_write_page};
use crate::sdk::drivers::pwm::{pwm_set_duty, pwm_start};
use crate::sdk::factory_reset::{factory_reset_cnt_check, factory_reset_handle, kick_out, KickoutReason};
use crate::sdk::light::*;
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1S, CLOCK_SYS_CLOCK_1US, clock_time, clock_time_exceed};
use crate::sdk::mcu::gpio::{AS_GPIO, gpio_set_func};
use crate::sdk::mcu::irq_i::irq_disable;
use crate::sdk::mcu::register::{FLD_IRQ, FLD_TMR, read_reg_irq_mask, read_reg_tmr_ctrl, write_reg_irq_mask, write_reg_tmr0_capt, write_reg_tmr0_tick, write_reg_tmr1_capt, write_reg_tmr_ctrl};
use crate::sdk::packet_types::{Packet, PacketAttValue};
use crate::sdk::pm::{light_sw_reboot, usb_dp_pullup_en};
use crate::sdk::rf_drv::*;
use crate::state::{*};
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

fn cfg_led_event(e: u32) {
    LED_EVENT_PENDING.set(e);
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

fn light_init_default() {
    let mut max_mesh_name_len = 0;
    let len = ADV_DATA.lock().len() + size_of::<AdvPrivate>() + 2;
    if len < 31 {
        max_mesh_name_len = 31 - len - 2;
        MAX_MESH_NAME_LEN.set(
            if max_mesh_name_len < 16 {
                max_mesh_name_len
            } else {
                16
            }
        );
    }

    light_check_tick_per_us(CLOCK_SYS_CLOCK_1US);

    PAIR_CONFIG_MESH_NAME.lock().fill(0);
    let len = min(MESH_NAME.len(), max_mesh_name_len);
    PAIR_CONFIG_MESH_NAME.lock()[0..len].copy_from_slice(&MESH_NAME.as_bytes()[0..len]);

    PAIR_CONFIG_MESH_PWD.lock().fill(0);
    let len = min(MESH_PWD.len(), 16);
    PAIR_CONFIG_MESH_PWD.lock()[0..len].copy_from_slice(&MESH_PWD.as_bytes()[0..len]);

    PAIR_CONFIG_MESH_LTK.lock()[0..16].copy_from_slice(&MESH_LTK[0..16]);

    rf_link_slave_pairing_enable(true);
    rf_set_power_level_index(RfPower::RfPower8dBm as u32);

    usb_dp_pullup_en(true);

    light_hw_timer1_config();

    app().mesh_manager.mesh_pair_init();
}

pub fn user_init() {
    // for app ota
    app().ota_manager.check_ota_area_startup();

    light_init_default();

    pwm_set_duty(PWMID_G, PMW_MAX_TICK, 0);
    pwm_set_duty(PWMID_B, PMW_MAX_TICK, PMW_MAX_TICK);

    pwm_start(PWMID_G);
    pwm_start(PWMID_B);

    gpio_set_func(PWM_G as u32, !AS_GPIO);
    gpio_set_func(PWM_B as u32, !AS_GPIO);

    //retrieve lumen value
    app().light_manager.light_lum_retrieve();

    rf_link_slave_init(40000);

    factory_reset_handle();

    vendor_set_adv_data();

    app().light_manager.device_status_update();
    app().mesh_manager.mesh_security_enable(true);
}

fn proc_led() {
    if LED_COUNT.get() == 0 && LED_EVENT_PENDING.get() == 0 {
        return; //led flash finished
    }

    if LED_EVENT_PENDING.get() != 0 {
        // new event
        LED_TON.set((LED_EVENT_PENDING.get() & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US);
        LED_TOFF.set(((LED_EVENT_PENDING.get() >> 8) & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US);
        LED_COUNT.set((LED_EVENT_PENDING.get() >> 16) & 0xff);
        LED_SEL.set(LED_EVENT_PENDING.get() >> 24);

        LED_EVENT_PENDING.set(0);
        LED_TICK.set(clock_time() + 30000000 * CLOCK_SYS_CLOCK_1US);
        LED_NO.set(0);
        LED_IS_ON.set(0);
    }

    if clock_time() - LED_TICK.get()
        >= (if LED_IS_ON.get() != 0 {
        LED_TON.get()
    } else {
        LED_TOFF.get()
    })
    {
        LED_TICK.set(clock_time());
        let led_off = (LED_IS_ON.get() != 0 || LED_TON.get() == 0) && LED_TOFF.get() != 0;
        let led_on = LED_IS_ON.get() == 0 && LED_TON.get() != 0;

        LED_IS_ON.set(!LED_IS_ON.get());
        if LED_IS_ON.get() != 0 {
            LED_NO.inc();
            if LED_NO.get() - 1 == LED_COUNT.get() {
                LED_COUNT.set(0);
                LED_NO.set(0);
                app().light_manager.light_onoff_hw(!app().light_manager.is_light_off()); // should not report online status again
                return;
            }
        }

        if led_off || led_on {
            if LED_SEL.get() & BIT!(0) != 0 {
                app().light_manager.light_adjust_cw(I16F16::from_num(LED_INDICATE_VAL * led_on as u16), I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE));
            }
            if LED_SEL.get() & BIT!(1) != 0 {
                app().light_manager.light_adjust_ww(I16F16::from_num(LED_INDICATE_VAL * led_on as u16), I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE));
            }
            if LED_SEL.get() & BIT!(5) != 0 {}
        }
    }
}

fn light_auth_check() {
    if SECURITY_ENABLE.get()
        && !PAIR_LOGIN_OK.get()
        && SLAVE_FIRST_CONNECTED_TICK.get() != 0
        && clock_time_exceed(SLAVE_FIRST_CONNECTED_TICK.get(), AUTH_TIME * 1000 * 1000)
    {
        //rf_link_slave_disconnect(); // must login in 60s after connected, if need
        SLAVE_FIRST_CONNECTED_TICK.set(0);
    }
}

fn light_user_func() {
    app().light_manager.check_light_state_save();

    light_auth_check();
    factory_reset_cnt_check();
    app().mesh_manager.mesh_pair_proc_effect();
}

pub async fn main_loop() {
    Timer::after(Duration::from_micros(LOOP_INTERVAL_US)).await;

    light_user_func();
    rf_link_slave_proc();

    proc_led();
}

/*@brief: This function is called in IRQ state, use IRQ stack.
**@param: ppp: is pointer to response
**@param: p_req: is pointer to request command
Called to handle messages that require a response to be returned
*/
pub fn rf_link_response_callback(

    ppp: &mut PacketAttValue,
    p_req: &PacketAttValue,
) -> bool {
    // mac-app[5] low 2 bytes used as ttc && hop-count
    let dst_unicast = is_unicast_addr(&p_req.dst);
    ppp.dst = p_req.src;
    ppp.src[0] = (DEVICE_ADDRESS.get() & 0xff) as u8;
    ppp.src[1] = ((DEVICE_ADDRESS.get() >> 8) & 0xff) as u8;

    let params = &p_req.val[3..13];
    ppp.val[3..10 + 3].fill(0);

    ppp.val[1] = (VENDOR_ID & 0xFF) as u8;
    ppp.val[2] = ((VENDOR_ID >> 8) & 0xff) as u8;

    ppp.val[18] = MAX_RELAY_NUM;

    let group_address = GROUP_ADDRESS.lock();

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
                if group_address[i] != 0 {
                    ppp.val[idx + 3] = group_address[i] as u8;
                    idx += 1;
                }
            }
        }
        GET_GROUP2 => {
            ppp.val[0] = LGT_CMD_LIGHT_GRP_RSP2 | 0xc0;
            for i in 0..MAX_GROUP_NUM as usize {
                ppp.val[i + 3] = 0xFF;
                if group_address[i / 2] != 0 {
                    ppp.val[idx + 3] = if (i % 2) != 0 {
                        (group_address[i / 2] >> 8) as u8
                    } else {
                        group_address[i / 2] as u8
                    };
                    idx += 1;
                }
            }
        }
        GET_GROUP3 => {
            ppp.val[0] = LGT_CMD_LIGHT_GRP_RSP3 | 0xc0;
            for i in 0..MAX_GROUP_NUM as usize {
                ppp.val[i + 3] = 0xFF;
                if group_address[4 + i / 2] != 0 {
                    ppp.val[idx + 3] = if (i % 2) != 0 {
                        (group_address[4 + i / 2] >> 8) as u8
                    } else {
                        group_address[4 + i / 2] as u8
                    };
                    idx += 1;
                }
            }
        }
        GET_DEV_ADDR => {
            ppp.val[0] = LGT_CMD_DEV_ADDR_RSP | 0xc0;
            return dev_addr_with_mac_rsp(&mut ppp.val);
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
            ppp.val[3] = (DEVICE_ADDRESS.get() & 0xFF) as u8;
            ppp.val[4] = ((DEVICE_ADDRESS.get() >> 8) & 0xff) as u8;
        }
        CMD_START_OTA => {
            ppp.val[0] = LGT_CMD_START_OTA_RSP | 0xc0;

            ppp.val[3] = BUILD_VERSION as u8;
            ppp.val[4] = (BUILD_VERSION >> 8) as u8;
            ppp.val[5] = (BUILD_VERSION >> 16) as u8;
            ppp.val[6] = (BUILD_VERSION >> 24) as u8;

            RF_SLAVE_OTA_BUSY_MESH.set(true);
        }
        CMD_OTA_DATA => {
            ppp.val[0] = LGT_CMD_OTA_DATA_RSP | 0xc0;

            let idx = app().ota_manager.rf_mesh_data_ota(&params, false);

            ppp.val[1] = idx as u8;
            ppp.val[2] = (idx >> 8) as u8;
        }
        CMD_END_OTA => {
            ppp.val[0] = LGT_CMD_END_OTA_RSP | 0xc0;

            let idx = app().ota_manager.rf_mesh_data_ota(&params, true);

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
pub fn rf_link_data_callback(p: &Packet) {
    // p start from l2cap_len of RfPacketAttCmdT
    let mut op_cmd: [u8; 3] = [0; 3];
    let mut op_cmd_len: u8 = 0;
    let mut params: [u8; 16] = [0; 16];
    let mut params_len: u8 = 0;
    rf_link_get_op_para(
        p,
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
                    if rf_link_del_group(val) {
                        cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
                    }
                }
                LIGHT_ADD_GRP_PARAM => {
                    if rf_link_add_group(val) {
                        cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
                    }
                }
                _ => ()
            }
        }
        LGT_CMD_CONFIG_DEV_ADDR => {
            let val = params[0] as u16 | ((params[1] as u16) << 8);
            if !dev_addr_with_mac_flag(params.as_ptr()) || dev_addr_with_mac_match(&params) {
                if rf_link_add_dev_addr(val) {
                    app().mesh_manager.mesh_pair_proc_get_mac_flag();
                }
            }
        }
        LGT_CMD_SET_LIGHT => app().light_manager.send_message(LGT_CMD_SET_LIGHT, params),
        LGT_CMD_SET_MAC_ADDR => {
            let mac = [params[0], params[1], params[2], params[3], params[4], params[5]];
            flash_erase_sector(FLASH_ADR_MAC);
            flash_write_page(FLASH_ADR_MAC, mac.len() as u32, addr_of!(mac) as *const u8);
            light_sw_reboot();
        }
        LGT_CMD_KICK_OUT => {
            irq_disable();
            let res = (params[0] as u32).try_into();
            if res.is_ok() {
                kick_out(res.unwrap());
            } else {
                kick_out(KickoutReason::OutOfMesh);
            }
            light_sw_reboot();
        }
        LGT_CMD_MESH_PAIR => app().mesh_manager.mesh_pair_cb(&params),
        _ => ()
    }
}

// p_cmd : cmd[3]+para[10]
// para    : dst
pub fn light_slave_tx_command(p_cmd: &[u8], para: u16) -> Packet {
    let mut cmd_op_para: [u8; 13] = [0; 13];
    let cmd_sno = clock_time() + DEVICE_ADDRESS.get() as u32;

    cmd_op_para[0..13].copy_from_slice(&p_cmd[0..13]);

    cmd_op_para[0] |= 0xc0;
    cmd_op_para[1] = (VENDOR_ID & 0xFF) as u8;
    cmd_op_para[2] = (VENDOR_ID >> 8) as u8;

    let dst = para;
    mesh_construct_packet(cmd_sno, dst, &cmd_op_para)
}

pub fn rf_link_light_event_callback(status: u8) {
    match status {
        LGT_CMD_SET_MESH_INFO => {
            mesh_node_init();
            app().light_manager.device_status_update();
            cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
        }
        LGT_CMD_SET_DEV_ADDR => {
            mesh_node_init();
            app().light_manager.device_status_update();
            cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
        }
        LGT_CMD_DEL_PAIR => cfg_led_event(LED_EVENT_FLASH_1HZ_4S),
        LGT_CMD_MESH_PAIR_TIMEOUT => cfg_led_event(LED_EVENT_FLASH_2HZ_2S),
        _ => ()
    }
}
