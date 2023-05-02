use core::cmp::min;
use core::convert::TryInto;
use core::mem::size_of;
use core::ptr::{addr_of, addr_of_mut};

use fixed::types::I16F16;
use heapless::Deque;

use crate::{app, blinken};
use crate::{BIT, pub_mut};
use crate::common::*;
use crate::config::*;
use crate::mesh::wrappers::mesh_security_enable;
use crate::sdk::ble_app::ble_ll_attribute::setSppUUID;
use crate::sdk::ble_app::light_ll::{is_receive_ota_window, light_set_tick_per_us, mesh_push_user_command, register_mesh_ota_master_ui, rf_link_get_op_para, rf_link_set_max_bridge, rf_link_slave_pairing_enable, rf_link_slave_proc, rf_link_slave_set_buffer, vendor_id_init};
use crate::sdk::ble_app::rf_drv_8266::{get_adv_data, rf_link_slave_init, rf_set_power_level_index};
use crate::sdk::drivers::flash::{flash_erase_sector, flash_write_page};
use crate::sdk::drivers::pwm::{pwm_set_duty, pwm_start};
use crate::sdk::factory_reset::{factory_reset_cnt_check, factory_reset_handle, kick_out, KickoutReason};
use crate::sdk::light::*;
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1S, CLOCK_SYS_CLOCK_1US, CLOCK_SYS_CLOCK_HZ, clock_time, clock_time_exceed};
use crate::sdk::mcu::gpio::{AS_GPIO, gpio_set_func};
use crate::sdk::mcu::irq_i::irq_disable;
use crate::sdk::mcu::register::{FLD_IRQ, FLD_TMR, read_reg_irq_mask, read_reg_tmr_ctrl, write_reg_irq_mask, write_reg_tmr0_capt, write_reg_tmr0_tick, write_reg_tmr1_capt, write_reg_tmr_ctrl};
use crate::sdk::pm::{light_sw_reboot, usb_dp_pullup_en};
use crate::sdk::rf_drv::*;
use crate::sdk::service::*;
use crate::vendor_light::{get_adv_pri_data, get_adv_rsp_pri_data, vendor_set_adv_data};

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
pub const LED_EVENT_FLASH_0P25HZ_1T: u32 = config_led_event!(4, 60, 1, LED_MASK);

pub_mut!(buff_response, [rf_packet_att_data_t; 48], [rf_packet_att_data_t {
    dma_len: 0,
    _type: 0,
    rf_len: 0,
    l2cap: 0,
    chanid: 0,
    att: 0,
    hl: 0,
    hh: 0,
    dat: [0; 23]
}; 48]);
pub_mut!(max_mesh_name_len, u8, 16);

pub_mut!(led_event_pending, u32, 0);

pub_mut!(led_count, u32, 0);
pub_mut!(led_ton, u32, 0);
pub_mut!(led_toff, u32, 0);
pub_mut!(led_sel, u32, 0);
pub_mut!(led_tick, u32, 0);
pub_mut!(led_no, u32, 0);
pub_mut!(led_is_on, u32, 0);


fn cfg_led_event(e: u32) {
    set_led_event_pending(e);
}

fn mesh_ota_master_led(_: *const u8) {
    if *get_led_count() == 0 && *get_led_event_pending() == 0 {
        cfg_led_event(LED_EVENT_FLASH_0P25HZ_1T);
    }
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

    light_set_tick_per_us(CLOCK_SYS_CLOCK_HZ / 1000000);

    get_pair_config_mesh_name().iter_mut().for_each(|m| *m = 0);
    let len = min(MESH_NAME.len(), _max_mesh_name_len as usize);
    get_pair_config_mesh_name()[0..len].copy_from_slice(&MESH_NAME.as_bytes()[0..len]);

    get_pair_config_mesh_pwd().iter_mut().for_each(|m| *m = 0);
    let len = min(MESH_PWD.len(), 16);
    get_pair_config_mesh_pwd()[0..len].copy_from_slice(&MESH_PWD.as_bytes()[0..len]);

    get_pair_config_mesh_ltk().iter_mut().for_each(|m| *m = 0);
    get_pair_config_mesh_ltk()[0..16].copy_from_slice(&MESH_LTK[0..16]);

    setSppUUID(
        TELINK_SPP_UUID_SERVICE.as_ptr(),
        TELINK_SPP_DATA_SERVER2CLIENT.as_ptr(),
        TELINK_SPP_DATA_CLIENT2SERVER.as_ptr(),
        TELINK_SPP_DATA_OTA.as_ptr(),
        TELINK_SPP_DATA_PAIR.as_ptr(),
    );

    set_p_adv_pri_data(get_adv_pri_data());
    set_adv_private_data_len(size_of::<ll_adv_private_t>() as u8);
    set_p_adv_rsp_data(get_adv_rsp_pri_data());

    rf_link_slave_pairing_enable(true);
    rf_set_power_level_index(RF_POWER::RF_POWER_8dBm as u32);
    rf_link_slave_set_buffer(get_buff_response().as_mut_slice());
    rf_link_set_max_bridge(BRIDGE_MAX_CNT);

    vendor_id_init(VENDOR_ID);

    usb_dp_pullup_en(true);

    light_hw_timer1_config();

    set_online_status_timeout(ONLINE_STATUS_TIMEOUT);

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
    mesh_security_enable(true);

    register_mesh_ota_master_ui(mesh_ota_master_led); //  mesh_ota_master_led() will be called when sending mesh ota data.
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
                app().light_manager.light_onoff_hw(!app().light_manager.is_light_off()); // should not report online status again
                return;
            }
        }

        if led_off || led_on {
            if *get_led_sel() & BIT!(0) != 0 {
                app().light_manager.light_adjust_cw(I16F16::from_num(LED_INDICATE_VAL * led_on as u16), I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE));
            }
            if *get_led_sel() & BIT!(1) != 0 {
                app().light_manager.light_adjust_ww(I16F16::from_num(LED_INDICATE_VAL * led_on as u16), I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE));
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

fn light_user_func() {
    light_auth_check();

    factory_reset_cnt_check();

    app().light_manager.check_light_state_save();
    app().mesh_manager.mesh_pair_proc_effect();
}

static mut NOTIFY_QUEUE: Deque<[u8; 13], 2> = Deque::new();

pub fn main_loop() {
    if !is_receive_ota_window() {
        return;
    }

    light_user_func();
    rf_link_slave_proc();
    proc_led();

    unsafe {
        while !NOTIFY_QUEUE.is_empty() {
            let data = critical_section::with(|_| {
                NOTIFY_QUEUE.pop_front().unwrap()
            });

            app().mesh_manager.send_mesh_message(&data, 0xffff);
        }
    }
}

/*@brief: This function is called in IRQ state, use IRQ stack.
**@param: ppp: is pointer to response
**@param: p_req: is pointer to request command*/
pub fn rf_link_response_callback(
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
                if get_group_address()[i] != 0 {
                    ppp.val[idx + 3] = get_group_address()[i] as u8;
                    idx += 1;
                }
            }
        }
        GET_GROUP2 => {
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
        }
        GET_GROUP3 => {
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
            ppp.val[3] = (*get_device_address() & 0xFF) as u8;
            ppp.val[4] = ((*get_device_address() >> 8) & 0xff) as u8;
        }
        _ => return false
    }

    return true;
}

/*@brief: This function is called in IRQ state, use IRQ stack.
*/
pub fn rf_link_data_callback(p: *const ll_packet_l2cap_data_t) {
    // p start from l2capLen of rf_packet_att_cmd_t
    let mut op_cmd: [u8; 8] = [0; 8];
    let mut op_cmd_len: u8 = 0;
    let mut params: [u8; 16] = [0; 16];
    let mut params_len: u8 = 0;
    let p = unsafe { &*p };
    let pp = unsafe { &*(p.value.as_ptr() as *const rf_packet_att_value_t) };
    rf_link_get_op_para(
        p,
        &mut op_cmd,
        &mut op_cmd_len,
        &mut params,
        &mut params_len,
        true,
    );

    if op_cmd_len != LightOpType::op_type_3 as u8 {
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
            blinken();
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
            flash_erase_sector(*get_flash_adr_mac());
            flash_write_page(*get_flash_adr_mac(), mac.len() as u32, addr_of!(mac) as *const u8);
            light_sw_reboot();
        }
        LGT_CLEAR_LUM_STATE => flash_erase_sector(*get_flash_adr_lum()),
        // Clear the lum state
        LGT_TRIGGER_PANIC => panic!("She's dead jim"),
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
        LGT_CMD_NOTIFY_MESH => light_notify(&pp.val[3..3 + 10], &pp.src),
        LGT_CMD_MESH_PAIR => app().mesh_manager.mesh_pair_cb(&params),
        _ => ()
    }
}

// p_cmd : cmd[3]+para[10]
// para    : dst
pub fn light_slave_tx_command(p_cmd: &[u8], para: u16) -> bool {
    let mut cmd_op_para: [u8; 16] = [0; 16];
    let cmd_sno = clock_time() + *get_device_address() as u32;

    cmd_op_para[0..13].copy_from_slice(&p_cmd[0..13]);

    cmd_op_para[0] |= 0xc0;
    cmd_op_para[1] = (VENDOR_ID & 0xFF) as u8;
    cmd_op_para[2] = (VENDOR_ID >> 8) as u8;

    let dst = para;
    mesh_push_user_command(cmd_sno, dst, cmd_op_para.as_ptr(), 13)
}

fn light_notify(p: &[u8], p_src: &[u8]) {
    let mut pkt: rf_packet_att_value_t = rf_packet_att_value_t {
        sno: [0; 3],
        src: [0; 2],
        dst: [0; 2],
        val: [0; 23],
    };

    pkt.src[0] = p_src[0];
    pkt.src[1] = p_src[1];

    // let valptr = get_pkt_light_notify().value.as_mut_ptr().offset(10);
    pkt.val[3..3 + 20].copy_from_slice(&[0; 20]);
    pkt.val[3..3 + p.len()].copy_from_slice(&p[0..p.len()]);
    pkt.val[15] = p[p.len() - 1];

    rf_link_response_callback(&mut pkt, &pkt);

    unsafe {
        let mut data: [u8; 13] = [0; 13];
        data.copy_from_slice(&pkt.val[0..13]);
        if !NOTIFY_QUEUE.is_full() {
            NOTIFY_QUEUE.push_back(data).unwrap();
        }
    }
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
