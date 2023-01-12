use std::cmp::{max, min};
use std::convert::TryInto;
use std::mem::size_of;
use std::ptr::{addr_of, copy_nonoverlapping, write_bytes};
use std::slice;

use ::{flash_adr_light_new_fw, PAIR_VALID_FLAG};
use ::{MESH_NAME, MESH_PWD};
use ::{MESH_LTK, VENDOR_ID};
use ::{MAX_LUM_BRIGHTNESS_VALUE, PWMID_B};
use ::{PWMID_G, PWMID_R};
use ::{PWM_G, PWM_R};
use ::{BIT, PWM_B};
use ::{flash_adr_lum, FLASH_SECTOR_SIZE};
use common::*;
use pub_mut;
use sdk::drivers::flash::{flash_erase_sector, flash_write_page};
use sdk::drivers::pwm::{pwm_set_cmp, pwm_set_duty, pwm_start};
use sdk::factory_reset::{factory_reset_cnt_check, factory_reset_handle, kick_out};
use sdk::light::*;
use sdk::mcu::analog::{analog_read__attribute_ram_code, analog_write__attribute_ram_code};
use sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, CLOCK_SYS_CLOCK_HZ, clock_time, clock_time_exceed};
use sdk::mcu::gpio::{AS_GPIO, gpio_set_func};
use sdk::mcu::irq_i::{irq_disable, irq_restore};
use sdk::mcu::register::*;
use sdk::pm::usb_dp_pullup_en;
use sdk::rf_drv::*;
use sdk::service::*;
use vendor_light::{get_adv_pri_data, get_adv_rsp_pri_data, vendor_set_adv_data};

extern "C" {
    static mut advData: [u8; 3];
    static mut max_mesh_name_len: u8;
}

pub const LED_INDICATE_VAL: u16 = 0xffff;
pub const LED_MASK: u8 = 0x07;
pub const LUM_SAVE_FLAG: u8 = 0xA5;

macro_rules! config_led_event {
    ($on:expr, $off:expr, $n:expr, $sel:expr) => {
        $on as u32 | ($off as u32) << 8 | ($n as u32) << 16 | ($sel as u32) << 24
    }
}

// pub const LED_EVENT_FLASH_4HZ_10S: u32 = config_led_event!(2,2,40,LED_MASK);
// pub const LED_EVENT_FLASH_STOP: u32 = config_led_event!(1,1,1,LED_MASK);
pub const LED_EVENT_FLASH_2HZ_2S: u32 = config_led_event!(4,4,4,LED_MASK);
// pub const LED_EVENT_FLASH_1HZ_1S: u32 = config_led_event!(8,8,1,LED_MASK);
// pub const LED_EVENT_FLASH_1HZ_2S: u32 = config_led_event!(8,8,2,LED_MASK);
// pub const LED_EVENT_FLASH_1HZ_3S: u32 = config_led_event!(8,8,3,LED_MASK);
pub const LED_EVENT_FLASH_1HZ_4S: u32 = config_led_event!(8,8,4,LED_MASK);
// pub const LED_EVENT_FLASH_4HZ: u32 = config_led_event!(2,2,0,LED_MASK);
// pub const LED_EVENT_FLASH_1HZ: u32 = config_led_event!(8,8,0,LED_MASK);
// pub const LED_EVENT_FLASH_4HZ_3T: u32 = config_led_event!(2,2,3,LED_MASK);
// pub const LED_EVENT_FLASH_1HZ_3T: u32 = config_led_event!(8,8,3,LED_MASK);
pub const LED_EVENT_FLASH_0P25HZ_1T: u32 = config_led_event!(4,60,1,LED_MASK);


pub_mut!(buff_response, [[u32; 9]; 48], [[0; 9]; 48]);

#[no_mangle]
static mut led_event_pending: u32 = 0;

static mut LED_COUNT: u32 = 0;
static mut LED_TON: u32 = 0;
static mut LED_TOFF: u32 = 0;
static mut LED_SEL: u32 = 0;
static mut LED_TICK: u32 = 0;
static mut LED_NO: u32 = 0;
static mut LED_IS_ON: u32 = 0;

static mut LUM_CHANGED_TIME: u32 = 0;
static mut LIGHT_LUM_ADDR: u32 = 0;

pub static mut LIGHT_OFF: bool = true;
pub static mut LED_LUM: u16 = 0xFFFF;
pub static mut LED_VAL: [u16; 3] = [MAX_LUM_BRIGHTNESS_VALUE, MAX_LUM_BRIGHTNESS_VALUE, MAX_LUM_BRIGHTNESS_VALUE];
pub static mut CMD_LEFT_DELAY_MS: u16 = 0;
pub static mut CMD_DELAY_MS: u16 = 0;
pub static mut IRQ_TIMER1_CB_TIME: u32 = 0;
pub static mut CMD_DELAY: ll_packet_l2cap_data_t = ll_packet_l2cap_data_t {
    l2capLen: 0,
    chanId: 0,
    opcode: 0,
    handle: 0,
    handle1: 0,
    value: [0; 30],
};

#[repr(C, packed)]
struct LumSaveT {
    save_falg: u8,
    lum: u16,
    ledval: [u16; 3],
}

fn calculate_lumen_map(val: u16) -> f32 {
    let percentage = (val as f32 / MAX_LUM_BRIGHTNESS_VALUE as f32) * 100.;
    // return keyframe::ease(keyframe::functions::EaseInOutCubic, 0., 0xffff as f32, percentage);
    return (-0.00539160*libm::powf(percentage, 3.0))+(4.47709595*libm::powf(percentage, 2.0))+(153.72442036*percentage);
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
    unsafe { led_event_pending = e; }
}

fn mesh_ota_master_led(_: *const u8) {
    unsafe {
        if LED_COUNT == 0 && led_event_pending == 0 {
            cfg_led_event(LED_EVENT_FLASH_0P25HZ_1T);
        }
    }
}

pub fn light_hw_timer1_config() {
    //enable timer1 interrupt
    write_reg_irq_mask(read_reg_irq_mask() | FLD_IRQ::TMR1_EN as u32);
    write_reg_tmr1_tick(0);
    write_reg_tmr1_capt(CLOCK_SYS_CLOCK_1US * IRQ_TIME1_INTERVAL as u32 * 1000);

    write_reg_tmr_ctrl(read_reg_tmr_ctrl() | FLD_TMR::TMR1_EN as u32);
}

fn light_init_default() {
    // unsafe { rest_light_init(); }
    // return;
    let len = (unsafe { advData }.len() + size_of::<ll_adv_private_t>() + 2) as u8;
    if len >= 31 {
        // error
        unsafe { max_mesh_name_len = 0; }
    } else {
        unsafe {
            max_mesh_name_len = 31 - len - 2;
            max_mesh_name_len = if max_mesh_name_len < 16 { max_mesh_name_len } else { 16 };
        }
    }

    // get fw version @flash 0x02,0x03,0x04,0x05
    _mesh_get_fw_version();

    //add the user_data after the ADV_PRI_DATA
    let user_const_data: [u8; 6] = [0x05, 0x02, 0x19, 0x00, 0x69, 0x69];
    unsafe {
        get_user_data()[0..user_const_data.len()].clone_from_slice(&user_const_data);

        set_user_data_len(0); // disable add the userdata after the adv_pridata

        _light_set_tick_per_us(CLOCK_SYS_CLOCK_HZ / 1000000);

        pair_config_valid_flag = PAIR_VALID_FLAG;

        get_pair_config_mesh_name().iter_mut().for_each(|m| *m = 0);
        let len = min(MESH_NAME.len(), max_mesh_name_len as usize);
        get_pair_config_mesh_name_val()[0..len].copy_from_slice(&MESH_NAME.as_bytes()[0..len]);

        get_pair_config_mesh_pwd().iter_mut().for_each(|m| *m = 0);
        let len = min(MESH_PWD.len(), 16);
        get_pair_config_mesh_pwd_val()[0..len].copy_from_slice(&MESH_PWD.as_bytes()[0..len]);

        get_pair_config_mesh_ltk().iter_mut().for_each(|m| *m = 0);
        get_pair_config_mesh_ltk_val()[0..16].copy_from_slice(&MESH_LTK[0..16]);
    }

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

    unsafe { online_status_timeout = ONLINE_STATUS_TIMEOUT; }

    mesh_pair_init();
}

pub fn user_init()
{
    // for APP ota
    if !is_ota_area_valid(flash_adr_light_new_fw) {
        erase_ota_data(flash_adr_light_new_fw);
    }

    light_init_default();

    // unsafe { rest_user_init(); }
    // return;

    pwm_set_duty(PWMID_R, PMW_MAX_TICK, PMW_MAX_TICK);
    pwm_set_duty(PWMID_G, PMW_MAX_TICK, PMW_MAX_TICK);
    pwm_set_duty(PWMID_B, PMW_MAX_TICK, 0);

    //retrieve lumen value
    unsafe { light_lum_retrieve(); }

    pwm_start(PWMID_R);
    pwm_start(PWMID_G);
    pwm_start(PWMID_B);

    gpio_set_func(PWM_R as u32, !AS_GPIO);
    gpio_set_func(PWM_G as u32, !AS_GPIO);
    gpio_set_func(PWM_B as u32, !AS_GPIO);

    _rf_link_slave_init(40000);

    factory_reset_handle();

    vendor_set_adv_data();

    unsafe { device_status_update() };
    _mesh_security_enable(true);

    _register_mesh_ota_master_ui(mesh_ota_master_led);   //  mesh_ota_master_led() will be called when sending mesh ota data.
}

unsafe fn proc_led()
{
    if LED_COUNT == 0 && led_event_pending == 0 {
        return;  //led flash finished
    }

    if led_event_pending != 0
    {
        // new event
        LED_TON = (led_event_pending & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US;
        LED_TOFF = ((led_event_pending >> 8) & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US;
        LED_COUNT = (led_event_pending >> 16) & 0xff;
        LED_SEL = led_event_pending >> 24;

        led_event_pending = 0;
        LED_TICK = clock_time() + 30000000 * CLOCK_SYS_CLOCK_1US;
        LED_NO = 0;
        LED_IS_ON = 0;
    }

    if clock_time() - LED_TICK >= (if LED_IS_ON != 0 { LED_TON } else { LED_TOFF }) {
        LED_TICK = clock_time();
        let led_off = (LED_IS_ON != 0 || LED_TON == 0) && LED_TOFF != 0;
        let led_on = LED_IS_ON == 0 && LED_TON != 0;

        LED_IS_ON = !LED_IS_ON;
        if LED_IS_ON != 0
        {
            LED_NO += 1;
            if LED_NO - 1 == LED_COUNT
            {
                LED_COUNT = 0;
                LED_NO = 0;
                light_onoff_hw(!LIGHT_OFF); // should not report online status again
                return;
            }
        }

        if led_off || led_on {
            if LED_SEL & BIT!(0) != 0
            {
                light_adjust_g(LED_INDICATE_VAL * led_on as u16, 0xffff);
            }
            if LED_SEL & BIT!(1) != 0
            {
                light_adjust_b(LED_INDICATE_VAL * led_on as u16, 0xffff);
            }
            if LED_SEL & BIT!(2) != 0
            {
                light_adjust_r(LED_INDICATE_VAL * led_on as u16, 0xffff);
            }
            if LED_SEL & BIT!(5) != 0
            {}
        }
    }
}

fn light_auth_check() {
    unsafe {
        if security_enable && !pair_login_ok && slave_first_connected_tick != 0 && clock_time_exceed(slave_first_connected_tick, AUTH_TIME * 1000 * 1000) {
            //rf_link_slave_disconnect(); // must login in 60s after connected, if need
            slave_first_connected_tick = 0;
        }
    }
}

//erase flash
unsafe fn light_lum_erase() {
    LIGHT_LUM_ADDR = flash_adr_lum;
    flash_erase_sector(flash_adr_lum);
    light_lum_store();
}

//save cur lum value, if disconnected for a while
unsafe fn light_lum_store() {
    if LIGHT_LUM_ADDR >= (flash_adr_lum + FLASH_SECTOR_SIZE as u32 - size_of::<LumSaveT>() as u32) {
        light_lum_erase();
        return;
    }

    let mut lum_save = LumSaveT {
        save_falg: LUM_SAVE_FLAG,
        lum: LED_LUM,
        ledval: [0, 0, 0],
    };

    #[allow(unaligned_references)]
    copy_nonoverlapping(LED_VAL.as_ptr(), lum_save.ledval.as_mut_ptr(), LED_VAL.len());

    flash_write_page(LIGHT_LUM_ADDR, size_of::<LumSaveT>() as u32, addr_of!(lum_save) as *const u8);
    LIGHT_LUM_ADDR += size_of::<LumSaveT>() as u32;

    return;
}

//retrieve LUM : brightness or RGB/CT value
unsafe fn light_lum_retrieve() {
    let mut i = 0;
    while i < FLASH_SECTOR_SIZE
    {
        LIGHT_LUM_ADDR = flash_adr_lum + i as u32;

        let lum_save = LIGHT_LUM_ADDR as *const LumSaveT;
        if LUM_SAVE_FLAG == (*lum_save).save_falg {
            LED_LUM = (*lum_save).lum;
            #[allow(unaligned_references)]
            copy_nonoverlapping((*lum_save).ledval.as_ptr(), LED_VAL.as_mut_ptr(), LED_VAL.len());
        } else if (*lum_save).save_falg == 0xFF {
            break;
        }

        i += size_of::<LumSaveT>() as u16
    }

    //effect
    light_adjust_rgb_hw(0, 0, 0, 0);

    mesh_ota_master_100_flag_check();

    let val = analog_read__attribute_ram_code(REGA_LIGHT_OFF);
    if val & RecoverStatus::FldLightOff as u8 != 0 {
        analog_write__attribute_ram_code(REGA_LIGHT_OFF, val & !(RecoverStatus::FldLightOff as u8));
        light_onoff(false);
    } else {
        light_onoff(true);
    }
}

fn light_user_func() {
    light_auth_check();

    factory_reset_cnt_check();

    // save current lum-val
    unsafe {
        if LUM_CHANGED_TIME != 0 && clock_time_exceed(LUM_CHANGED_TIME, 5000 * 1000) {
            LUM_CHANGED_TIME = 0;
            light_lum_store();
        }
    }

    mesh_pair_proc_effect();
}

pub fn main_loop()
{
    if _is_receive_ota_window() {
        return;
    }

    light_user_func();
    _rf_link_slave_proc();
    unsafe { proc_led(); }
}

/*@brief: This function is called in IRQ state, use IRQ stack.
**@param: ppp: is pointer to response
**@param: p_req: is pointer to request command*/
#[no_mangle]
unsafe fn rf_link_response_callback(ppp: *mut rf_packet_att_value_t, p_req: *const rf_packet_att_value_t) -> bool
{
    // mac-APP[5] low 2 bytes used as ttc && hop-count
    let dst_unicast = is_unicast_addr((*p_req).dst.as_ptr());
    (*ppp).dst[0] = (*p_req).src[0];
    (*ppp).dst[1] = (*p_req).src[1];
    (*ppp).src[0] = (device_address & 0xff) as u8;
    (*ppp).src[1] = ((device_address >> 8) & 0xff) as u8;

    let mut params: [u8; 10] = [0; 10];
    copy_nonoverlapping((*ppp).val.as_ptr().offset(3), params.as_mut_ptr(), params.len()); // be same with p_req->val+3
    write_bytes((*ppp).val.as_mut_ptr().offset(3), 0, params.len());

    (*ppp).val[1] = (VENDOR_ID & 0xFF) as u8;
    (*ppp).val[2] = ((VENDOR_ID >> 8) & 0xff) as u8;

    (*ppp).val[18] = max_relay_num;

    let mut idx = 0;
    if (*ppp).val[15] == GET_STATUS {
        (*ppp).val[0] = LGT_CMD_LIGHT_STATUS | 0xc0;
        for i in 0..3 {//params[0]
            (*ppp).val[i * 2 + 3] = if LIGHT_OFF { 0 } else { (LED_VAL[i] & 0xff) as u8 };
            (*ppp).val[i * 2 + 4] = if LIGHT_OFF { 0 } else { ((LED_VAL[i] >> 8) & 0xff) as u8 };
        }
    } else if (*ppp).val[15] == GET_GROUP1 {
        (*ppp).val[0] = LGT_CMD_LIGHT_GRP_RSP1 | 0xc0;
        for i in 0..MAX_GROUP_NUM as usize {
            (*ppp).val[i + 3] = 0xFF;
            if group_address[i] != 0 {
                (*ppp).val[idx + 3] = group_address[i] as u8;
                idx += 1;
            }
        }
    } else if (*ppp).val[15] == GET_GROUP2 {
        (*ppp).val[0] = LGT_CMD_LIGHT_GRP_RSP2 | 0xc0;
        for i in 0..MAX_GROUP_NUM as usize {
            (*ppp).val[i + 3] = 0xFF;
            if group_address[i / 2] != 0 {
                (*ppp).val[idx + 3] = if (i % 2) != 0 { (group_address[i / 2] >> 8) as u8 } else { group_address[i / 2] as u8 };
                idx += 1;
            }
        }
    } else if (*ppp).val[15] == GET_GROUP3 {
        (*ppp).val[0] = LGT_CMD_LIGHT_GRP_RSP3 | 0xc0;
        for i in 0..MAX_GROUP_NUM as usize {
            (*ppp).val[i + 3] = 0xFF;
            if group_address[4 + i / 2] != 0 {
                (*ppp).val[idx + 3] = if (i % 2) != 0 { (group_address[4 + i / 2] >> 8) as u8 } else { group_address[4 + i / 2] as u8 };
                idx += 1;
            }
        }
    } else if (*ppp).val[15] == GET_DEV_ADDR {
        (*ppp).val[0] = LGT_CMD_DEV_ADDR_RSP | 0xc0;
        if dev_addr_with_mac_flag(params.as_ptr()) {
            return dev_addr_with_mac_rsp(params.as_ptr(), (*ppp).val.as_mut_ptr().offset(3));
        } else {
            (*ppp).val[3] = (device_address & 0xFF) as u8;
            (*ppp).val[4] = ((device_address >> 8) & 0xff) as u8;
        }
    } else if (*ppp).val[15] == GET_USER_NOTIFY {
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

        (*ppp).val[0] = LGT_CMD_USER_NOTIFY_RSP | 0xc0;
        for i in 0..8 {//params[2]
            (*ppp).val[5 + i] = i as u8;
        }
        (*ppp).val[3] = (device_address & 0xFF) as u8;
        (*ppp).val[4] = ((device_address >> 8) & 0xff) as u8;
    } else if (*ppp).val[15] == GET_MESH_OTA {
        (*ppp).val[0] = LGT_CMD_MESH_OTA_READ_RSP | 0xc0;
        if params[1] == PAR_READ_MESH_PAIR_CONFIRM {
            for i in 0..8 {
                (*ppp).val[5 + i] = get_mesh_pair_checksum(i as u8);
            }
            (*ppp).val[3] = (device_address & 0xFF) as u8;
            (*ppp).val[4] = ((device_address >> 8) & 0xff) as u8;
            return true;
        }
        return _mesh_ota_slave_set_response((*ppp).val.as_mut_ptr().offset(3), params[1]);
    } else {
        return false;
    }

    return true;
}

/*@brief: This function is called in IRQ state, use IRQ stack.
*/
#[no_mangle]
unsafe fn rf_link_data_callback(p: *const ll_packet_l2cap_data_t)
{
    // p start from l2capLen of rf_packet_att_cmd_t
    let mut op_cmd: [u8; 8] = [0; 8];
    let op_cmd_len: u8 = 0;
    let mut params: [u8; 16] = [0; 16];
    let params_len: u8 = 0;
    let pp = (*p).value.as_ptr() as *const rf_packet_att_value_t;
    _rf_link_get_op_para(p, op_cmd.as_mut_ptr(), &op_cmd_len, params.as_mut_ptr(), &params_len, 1);

    if op_cmd_len != LightOpType::op_type_3 as u8 {
        return;
    }

    // let vendor_id = (op_cmd[2] as u16) << 8 | op_cmd[1] as u16;
    let op = op_cmd[0] & 0x3F;

    if op == LGT_CMD_LIGHT_ONOFF {
        if CMD_LEFT_DELAY_MS != 0 {
            return;
        }
        CMD_DELAY_MS = params[1] as u16 | ((params[2] as u16) << 8);
        if CMD_DELAY_MS != 0 && IRQ_TIMER1_CB_TIME == 0 {
            let cmd_delayed_ms = light_cmd_delayed_ms((*pp).val[(op_cmd_len + params_len) as usize]);
            if CMD_DELAY_MS > cmd_delayed_ms {
                CMD_DELAY = (*p).clone();
                CMD_LEFT_DELAY_MS = CMD_DELAY_MS - cmd_delayed_ms;
                IRQ_TIMER1_CB_TIME = clock_time();
                return;
            }
        }
    }

    _mesh_ota_timeout_handle(op, params.as_ptr());

    if op == LGT_CMD_LIGHT_ONOFF {
        if params[0] == LIGHT_ON_PARAM {
            light_onoff(true);
        } else if params[0] == LIGHT_OFF_PARAM {
            if ON_OFF_FROM_OTA == params[3] { // only PWM off,
                light_step_reset(0);
            } else {
                light_onoff(false);
            }
        }
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
        if !dev_addr_with_mac_flag(params.as_ptr()) || dev_addr_with_mac_match(params.as_ptr()) {
            if _rf_link_add_dev_addr(val) {
                mesh_pair_proc_get_mac_flag();
            }
        }
    } else if op == LGT_CMD_SET_LIGHT
    {
        if params[8] & 0x1 != 0 {
            // Brightness
            let value = (params[1] as u16) << 8 | params[0] as u16;

            if LIGHT_OFF {
                LED_LUM = value;
                return;
            }
            light_step_reset(value);
        }
        if params[8] & 0x2 != 0 {
            // Temperature
            let value = (params[3] as u16) << 8 | params[2] as u16;

            LED_VAL[0] = 0;
            LED_VAL[1] = MAX_LUM_BRIGHTNESS_VALUE - value;
            LED_VAL[2] = value;

            if LIGHT_OFF {
                return;
            }

            light_step_reset(LED_LUM);
        }

        LUM_CHANGED_TIME = clock_time();
    } else if op == LGT_CMD_KICK_OUT
    {
        if is_mesh_cmd_need_delay(p as *const u8, params.as_ptr(), (*pp).val[(op_cmd_len + params_len) as usize]) {
            return;
        }
        irq_disable();
        kick_out((params[0] as u32).try_into().unwrap());
        _light_sw_reboot();
    } else if op == LGT_CMD_NOTIFY_MESH
    {
        light_notify((*pp).val.as_ptr().offset(3), 10, (*pp).src.as_ptr());
    } else if op == LGT_CMD_MESH_OTA_DATA
    {
        let idx = (params[0] as u16) | ((params[1] as u16) << 8);
        if !_is_master_ota_st() {  // no update firmware for itself
            if CMD_START_MESH_OTA == idx {
                mesh_ota_master_start_firmware_from_own();
                //cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
            } else if CMD_STOP_MESH_OTA == idx {
                if _is_mesh_ota_slave_running() {
                    // reboot to initial flash: should be delay to relay command.
                    _mesh_ota_slave_reboot_delay();  // reboot after 320ms
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
    } else if op == LGT_CMD_MESH_PAIR
    {
        mesh_pair_cb(params.as_ptr());
    }
}

// p_cmd : cmd[3]+para[10]
// para    : dst
unsafe fn light_slave_tx_command_ll(p_cmd: *const u8, para: u16) -> bool
{
    let mut cmd_op_para: [u8; 16] = [0; 16];
    let cmd_sno = clock_time() + device_address as u32;

    let slice = slice::from_raw_parts(p_cmd, 13);
    cmd_op_para[0..13].copy_from_slice(slice);

    cmd_op_para[0] = *p_cmd.offset(0) | 0xc0;
    cmd_op_para[1] = (VENDOR_ID & 0xFF) as u8;
    cmd_op_para[2] = (VENDOR_ID >> 8) as u8;

    let dst = para;
    _mesh_push_user_command(cmd_sno, dst, cmd_op_para.as_ptr(), 13);

    return true;
}

pub unsafe fn light_slave_tx_command(p_cmd: *const u8, para: u16) -> bool
{
    return light_slave_tx_command_ll(p_cmd, para);
}

unsafe fn light_notify(p: *const u8, len: u8, p_src: *const u8) -> i32 {
    let mut err = -1;
    if slave_link_connected && pair_login_ok {
        if len > 10 {   //max length of par is 10
            return -1;
        }

        pkt_light_notify.value[3] = *p_src.offset(0);
        pkt_light_notify.value[4] = *p_src.offset(1);

        let valptr = pkt_light_notify.value.as_mut_ptr().offset(10);

        write_bytes(valptr, 0, 10);
        copy_nonoverlapping(p, valptr, len as usize);

        let r = irq_disable();
        if _is_add_packet_buf_ready() {
            if !_rf_link_add_tx_packet(addr_of!(pkt_light_notify) as *const u8) {
                err = 0;
            }
        }
        irq_restore(r);
    }

    return err;
}

#[no_mangle]
pub unsafe fn rf_link_light_event_callback(status: u8)
{
    if status == LGT_CMD_SET_MESH_INFO
    {
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

#[no_mangle]
unsafe fn irq_timer1() {
    if IRQ_TIMER1_CB_TIME != 0 && clock_time_exceed(IRQ_TIMER1_CB_TIME, (CMD_LEFT_DELAY_MS * 1000) as u32) {
        CMD_LEFT_DELAY_MS = 0;
        rf_link_data_callback(&CMD_DELAY);
        CMD_DELAY_MS = 0;
        IRQ_TIMER1_CB_TIME = 0;
    }

    light_onoff_step_timer();
}

#[no_mangle]
fn irq_timer0() {}

#[no_mangle]
#[allow(non_snake_case)]
unsafe fn irq_handler__attribute_ram_code()
{
    _irq_light_slave_handler();
}

fn light_onoff_hw(on: bool) {
    unsafe { light_onoff_step(on); }
}

fn light_onoff(on: bool) {
    light_onoff_hw(on);

    unsafe { device_status_update(); }
}

#[no_mangle]
pub unsafe fn device_status_update() {
    // packet
    let mut st_val_par: [u8; MESH_NODE_ST_PAR_LEN as usize] = [0xff; MESH_NODE_ST_PAR_LEN as usize];

    let value = if LIGHT_OFF { 0 } else { max(LED_LUM, 1) };

    // LED_LUM should not be 0, because APP will take it to be light off
    st_val_par[0] = (value & 0xff) as u8;     //Note: bit7 of par[0] have been use internal for FLD_SYNCED
    st_val_par[1] = ((value >> 8) & 0xff) as u8;   // rsv
    // end

    _ll_device_status_update(st_val_par.as_ptr(), st_val_par.len() as u8);
}

#[no_mangle]
pub unsafe fn light_onoff_normal(on: bool) {
    if on {
        LIGHT_OFF = false;
        light_adjust_rgb_hw(LED_VAL[0], LED_VAL[1], LED_VAL[2], LED_LUM);
    } else {
        LIGHT_OFF = true;
        light_adjust_rgb_hw(0, 0, 0, 0);
    }
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

#[no_mangle]
pub fn light_adjust_rgb_hw(val_r: u16, val_g: u16, val_b: u16, lum: u16) {
    light_adjust_r(val_r, lum);
    light_adjust_g(val_g, lum);
    light_adjust_b(val_b, lum);
}