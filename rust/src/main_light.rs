use std::cmp::{max, min};
use std::convert::TryInto;
use std::mem::size_of;
use std::ops::Deref;
use std::ptr::{addr_of, addr_of_mut, copy_nonoverlapping, write_bytes};
use std::slice;

use ::{flash_adr_light_new_fw, PAIR_VALID_FLAG};
use ::{MESH_NAME, MESH_PWD};
use ::{MESH_LTK, VENDOR_ID};
use ::{MAX_LUM_BRIGHTNESS_VALUE, PWMID_B};
use ::{PWMID_G, PWMID_R};
use ::{PWM_G, PWM_R};
use ::{BIT, PWM_B};
use ::{flash_adr_lum, FLASH_SECTOR_SIZE};
use common::{dev_addr_with_mac_flag, dev_addr_with_mac_match, dev_addr_with_mac_rsp, erase_ota_data, get_mesh_pair_checksum, is_mesh_cmd_need_delay, is_ota_area_valid, light_cmd_delayed_ms, light_onoff_step, light_onoff_step_timer, light_step_reset, MESH_NODE_ST_PAR_LEN, mesh_ota_master_100_flag_check, mesh_pair_cb, mesh_pair_init, mesh_pair_proc_effect, mesh_pair_proc_get_mac_flag, RECOVER_STATUS, rega_light_off};
use sdk::drivers::flash::{flash_erase_sector, flash_write_page};
use sdk::drivers::pwm::{pwm_set_cmp, pwm_set_duty, pwm_start};
use sdk::factory_reset::{factory_reset_cnt_check, factory_reset_handle, kick_out};
use sdk::light::{adv_private_data_len, AUTH_TIME, BRIDGE_MAX_CNT, CMD_START_MESH_OTA, CMD_STOP_MESH_OTA, device_address, GET_DEV_ADDR, GET_GROUP1, GET_GROUP2, GET_GROUP3, GET_MESH_OTA, GET_STATUS, GET_USER_NOTIFY, group_address, irq_light_slave_handler, IRQ_TIME1_INTERVAL, is_add_packet_buf_ready, is_master_ota_st, is_mesh_ota_slave_running, is_receive_ota_window, is_unicast_addr, LGT_CMD_CONFIG_DEV_ADDR, LGT_CMD_DEL_PAIR, LGT_CMD_DEV_ADDR_RSP, LGT_CMD_KICK_OUT, LGT_CMD_LIGHT_CONFIG_GRP, LGT_CMD_LIGHT_GRP_RSP1, LGT_CMD_LIGHT_GRP_RSP2, LGT_CMD_LIGHT_GRP_RSP3, LGT_CMD_LIGHT_ONOFF, LGT_CMD_LIGHT_STATUS, LGT_CMD_MESH_OTA_DATA, LGT_CMD_MESH_OTA_READ_RSP, LGT_CMD_MESH_PAIR, LGT_CMD_MESH_PAIR_TIMEOUT, LGT_CMD_NOTIFY_MESH, LGT_CMD_SET_DEV_ADDR, LGT_CMD_SET_LIGHT, LGT_CMD_SET_MESH_INFO, LGT_CMD_USER_NOTIFY_RSP, LIGHT_ADD_GRP_PARAM, LIGHT_DEL_GRP_PARAM, LIGHT_OFF_PARAM, LIGHT_ON_PARAM, light_set_tick_per_us, light_sw_reboot, LightOpType, ll_adv_private_t, ll_adv_rsp_private_t, ll_device_status_update, ll_packet_l2cap_data_t, MAX_GROUP_NUM, max_relay_num, mesh_get_fw_version, mesh_node_init, mesh_ota_master_cancle, mesh_ota_master_start_firmware_from_own, mesh_ota_slave_reboot_delay, mesh_ota_slave_save_data, mesh_ota_slave_set_response, mesh_ota_timeout_handle, mesh_push_user_command, mesh_security_enable, ON_OFF_FROM_OTA, online_status_timeout, ONLINE_STATUS_TIMEOUT, OtaState, p_adv_pri_data, p_adv_rsp_data, pair_config_mesh_ltk, pair_config_mesh_name, pair_config_mesh_pwd, pair_config_valid_flag, pair_login_ok, PAR_READ_MESH_PAIR_CONFIRM, pkt_light_notify, PMW_MAX_TICK, register_mesh_ota_master_ui, rf_link_add_tx_packet, rf_link_slave_proc, rf_packet_att_value_t, security_enable, setSppUUID, slave_first_connected_tick, slave_link_connected, user_data, user_data_len, vendor_id_init};
use sdk::mcu::analog::{analog_read__attribute_ram_code, analog_write__attribute_ram_code};
use sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, CLOCK_SYS_CLOCK_HZ, clock_time, clock_time_exceed};
use sdk::mcu::gpio::{AS_GPIO, gpio_set_func};
use sdk::mcu::irq_i::{irq_disable, irq_restore};
use sdk::mcu::register::{FLD_IRQ, FLD_TMR, read_reg_irq_mask, read_reg_tmr_ctrl, write_reg_irq_mask, write_reg_tmr1_capt, write_reg_tmr1_tick, write_reg_tmr_ctrl};
use sdk::pm::usb_dp_pullup_en;
use sdk::rf_drv::{rf_link_add_dev_addr, rf_link_add_group, rf_link_del_group, rf_link_get_op_para, rf_link_set_max_bridge, rf_link_slave_init, rf_link_slave_pairing_enable, rf_link_slave_set_buffer, RF_POWER, rf_set_power_level_index};
use sdk::service::{TELINK_SPP_DATA_CLIENT2SERVER, TELINK_SPP_DATA_OTA, TELINK_SPP_DATA_PAIR, TELINK_SPP_DATA_SERVER2CLIENT, TELINK_SPP_UUID_SERVICE};
use vendor_light::{adv_pri_data, adv_rsp_pri_data, vendor_set_adv_data};

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

pub const LED_EVENT_FLASH_4HZ_10S: u32 = config_led_event!(2,2,40,LED_MASK);
pub const LED_EVENT_FLASH_STOP: u32 = config_led_event!(1,1,1,LED_MASK);
pub const LED_EVENT_FLASH_2HZ_2S: u32 = config_led_event!(4,4,4,LED_MASK);
pub const LED_EVENT_FLASH_1HZ_1S: u32 = config_led_event!(8,8,1,LED_MASK);
pub const LED_EVENT_FLASH_1HZ_2S: u32 = config_led_event!(8,8,2,LED_MASK);
pub const LED_EVENT_FLASH_1HZ_3S: u32 = config_led_event!(8,8,3,LED_MASK);
pub const LED_EVENT_FLASH_1HZ_4S: u32 = config_led_event!(8,8,4,LED_MASK);
pub const LED_EVENT_FLASH_4HZ: u32 = config_led_event!(2,2,0,LED_MASK);
pub const LED_EVENT_FLASH_1HZ: u32 = config_led_event!(8,8,0,LED_MASK);
pub const LED_EVENT_FLASH_4HZ_3T: u32 = config_led_event!(2,2,3,LED_MASK);
pub const LED_EVENT_FLASH_1HZ_3T: u32 = config_led_event!(8,8,3,LED_MASK);
pub const LED_EVENT_FLASH_0p25HZ_1T: u32 = config_led_event!(4,60,1,LED_MASK);


#[no_mangle]
pub static mut buff_response: [[u32; 9]; 48] = [[0; 9]; 48];

#[no_mangle]
static mut led_event_pending: u32 = 0;

static mut led_count: u32 = 0;
static mut led_ton: u32 = 0;
static mut led_toff: u32 = 0;
static mut led_sel: u32 = 0;
static mut led_tick: u32 = 0;
static mut led_no: u32 = 0;
static mut led_is_on: u32 = 0;

static mut lum_changed_time: u32 = 0;
static mut light_lum_addr: u32 = 0;

pub static mut light_off: bool = true;
pub static mut led_lum: u16 = 0xFFFF;
pub static mut led_val: [u16; 3] = [MAX_LUM_BRIGHTNESS_VALUE, MAX_LUM_BRIGHTNESS_VALUE, MAX_LUM_BRIGHTNESS_VALUE];
pub static mut cmd_left_delay_ms: u16 = 0;
pub static mut cmd_delay_ms: u16 = 0;
pub static mut irq_timer1_cb_time: u32 = 0;
pub static mut cmd_delay: ll_packet_l2cap_data_t = ll_packet_l2cap_data_t {
    l2capLen: 0,
    chanId: 0,
    opcode: 0,
    handle: 0,
    handle1: 0,
    value: [0; 30],
};

#[repr(C, packed)]
struct lum_save_t {
    save_falg: u8,
    lum: u16,
    ledval: [u16; 3],
}

fn calculate_lumen_map(val: u16) -> f32 {
    let percentage = (val as f32 / MAX_LUM_BRIGHTNESS_VALUE as f32) * 100.0;
    return (-0.00539160*micromath::F32Ext::powf(percentage, 3.0))+(4.47709595*micromath::F32Ext::powf(percentage, 2.0))+(153.72442036*percentage);
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
        if led_count == 0 && led_event_pending == 0 {
            cfg_led_event(LED_EVENT_FLASH_0p25HZ_1T);
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
    unsafe { mesh_get_fw_version(); }

    //add the user_data after the adv_pri_data
    let user_const_data: [u8; 6] = [0x05, 0x02, 0x19, 0x00, 0x69, 0x69];
    unsafe {
        user_data[0..user_const_data.len()].clone_from_slice(&user_const_data);

        user_data_len = 0; // disable add the userdata after the adv_pridata

        light_set_tick_per_us(CLOCK_SYS_CLOCK_HZ / 1000000);

        pair_config_valid_flag = PAIR_VALID_FLAG;

        pair_config_mesh_name.iter_mut().for_each(|m| *m = 0);
        let len = min(MESH_NAME.len(), max_mesh_name_len as usize);
        pair_config_mesh_name[0..len].copy_from_slice(&MESH_NAME.as_bytes()[0..len]);

        pair_config_mesh_pwd.iter_mut().for_each(|m| *m = 0);
        let len = min(MESH_PWD.len(), 16);
        pair_config_mesh_pwd[0..len].copy_from_slice(&MESH_PWD.as_bytes()[0..len]);

        pair_config_mesh_ltk.iter_mut().for_each(|m| *m = 0);
        pair_config_mesh_ltk[0..16].copy_from_slice(&MESH_LTK[0..16]);
    }

    unsafe {
        setSppUUID(
            TELINK_SPP_UUID_SERVICE.as_ptr(),
            TELINK_SPP_DATA_SERVER2CLIENT.as_ptr(),
            TELINK_SPP_DATA_CLIENT2SERVER.as_ptr(),
            TELINK_SPP_DATA_OTA.as_ptr(),
            TELINK_SPP_DATA_PAIR.as_ptr(),
        );
    }

    unsafe { p_adv_pri_data = &adv_pri_data; }
    unsafe { adv_private_data_len = size_of::<ll_adv_private_t>() as u8; }
    unsafe { p_adv_rsp_data = &adv_rsp_pri_data; }

    unsafe { rf_link_slave_pairing_enable(1); }

    unsafe { rf_set_power_level_index(RF_POWER::RF_POWER_8dBm as u32); }

    unsafe { rf_link_slave_set_buffer(buff_response.as_mut_ptr(), 48); }

    unsafe { rf_link_set_max_bridge(BRIDGE_MAX_CNT); }
    unsafe { vendor_id_init(VENDOR_ID); }

    usb_dp_pullup_en(true);

    light_hw_timer1_config();

    unsafe { online_status_timeout = ONLINE_STATUS_TIMEOUT; }

    mesh_pair_init();
}

pub fn user_init()
{
    // for app ota
    unsafe {
        if !is_ota_area_valid(flash_adr_light_new_fw) {
            erase_ota_data(flash_adr_light_new_fw);
        }
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

    unsafe { rf_link_slave_init(40000); }

    unsafe { factory_reset_handle(); }

    unsafe { vendor_set_adv_data(); }

    unsafe { device_status_update(); }
    unsafe { mesh_security_enable(true); }

    unsafe { register_mesh_ota_master_ui(mesh_ota_master_led); }   //  mesh_ota_master_led() will be called when sending mesh ota data.
}

unsafe fn proc_led()
{
    if led_count == 0 && led_event_pending == 0 {
        return;  //led flash finished
    }

    if led_event_pending != 0
    {
        // new event
        led_ton = (led_event_pending & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US;
        led_toff = ((led_event_pending >> 8) & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US;
        led_count = (led_event_pending >> 16) & 0xff;
        led_sel = led_event_pending >> 24;

        led_event_pending = 0;
        led_tick = clock_time() + 30000000 * CLOCK_SYS_CLOCK_1US;
        led_no = 0;
        led_is_on = 0;
    }

    if clock_time() - led_tick >= (if led_is_on != 0 { led_ton } else { led_toff }) {
        led_tick = clock_time();
        let led_off = (led_is_on != 0 || led_ton == 0) && led_toff != 0;
        let led_on = led_is_on == 0 && led_ton != 0;

        led_is_on = !led_is_on;
        if led_is_on != 0
        {
            led_no += 1;
            if led_no - 1 == led_count
            {
                led_count = 0;
                led_no = 0;
                light_onoff_hw(!light_off); // should not report online status again
                return;
            }
        }

        if led_off || led_on {
            if led_sel & BIT!(0) != 0
            {
                light_adjust_G(LED_INDICATE_VAL * led_on as u16, 0xffff);
            }
            if led_sel & BIT!(1) != 0
            {
                light_adjust_B(LED_INDICATE_VAL * led_on as u16, 0xffff);
            }
            if led_sel & BIT!(2) != 0
            {
                light_adjust_R(LED_INDICATE_VAL * led_on as u16, 0xffff);
            }
            if led_sel & BIT!(5) != 0
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
    light_lum_addr = flash_adr_lum;
    flash_erase_sector(flash_adr_lum);
    light_lum_store();
}

//save cur lum value, if disconnected for a while
unsafe fn light_lum_store() {
    if light_lum_addr >= (flash_adr_lum + FLASH_SECTOR_SIZE as u32 - size_of::<lum_save_t>() as u32) {
        light_lum_erase();
        return;
    }

    let mut lum_save = lum_save_t {
        save_falg: LUM_SAVE_FLAG,
        lum: led_lum,
        ledval: [0, 0, 0],
    };

    #[allow(unaligned_references)]
    copy_nonoverlapping(led_val.as_ptr(), lum_save.ledval.as_mut_ptr(), led_val.len());

    flash_write_page(light_lum_addr, size_of::<lum_save_t>() as u32, addr_of!(lum_save) as *const u8);
    light_lum_addr += size_of::<lum_save_t>() as u32;

    return;
}

//retrieve LUM : brightness or RGB/CT value
unsafe fn light_lum_retrieve() {
    let mut i = 0;
    while i < FLASH_SECTOR_SIZE
    {
        light_lum_addr = flash_adr_lum + i as u32;

        let lum_save = light_lum_addr as *const lum_save_t;
        if LUM_SAVE_FLAG == (*lum_save).save_falg {
            led_lum = (*lum_save).lum;
            #[allow(unaligned_references)]
            copy_nonoverlapping((*lum_save).ledval.as_ptr(), led_val.as_mut_ptr(), led_val.len());
        } else if (*lum_save).save_falg == 0xFF {
            break;
        }

        i += size_of::<lum_save_t>() as u16
    }

    //effect
    light_adjust_RGB_hw(0, 0, 0, 0);

    mesh_ota_master_100_flag_check();

    let val = analog_read__attribute_ram_code(rega_light_off);
    if val & RECOVER_STATUS::FLD_LIGHT_OFF as u8 != 0 {
        analog_write__attribute_ram_code(rega_light_off, val & !(RECOVER_STATUS::FLD_LIGHT_OFF as u8));
        light_onoff(false);
    } else {
        light_onoff(true);
    }
}

fn light_user_func() {
    light_auth_check();

    unsafe { factory_reset_cnt_check(); }

    // save current lum-val
    unsafe {
        if lum_changed_time != 0 && clock_time_exceed(lum_changed_time, 5000 * 1000) {
            lum_changed_time = 0;
            light_lum_store();
        }
    }

    mesh_pair_proc_effect();
}

pub fn main_loop()
{
    unsafe {
        if is_receive_ota_window() {
            return;
        }
    }

    light_user_func();

    unsafe { rf_link_slave_proc(); }

    unsafe { proc_led(); }
}

/*@brief: This function is called in IRQ state, use IRQ stack.
**@param: ppp: is pointer to response
**@param: p_req: is pointer to request command*/
#[no_mangle]
unsafe fn rf_link_response_callback(ppp: *mut rf_packet_att_value_t, p_req: *const rf_packet_att_value_t) -> bool
{
    // mac-app[5] low 2 bytes used as ttc && hop-count
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
            (*ppp).val[i * 2 + 3] = if light_off { 0 } else { (led_val[i] & 0xff) as u8 };
            (*ppp).val[i * 2 + 4] = if light_off { 0 } else { ((led_val[i] >> 8) & 0xff) as u8 };
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
        return mesh_ota_slave_set_response((*ppp).val.as_mut_ptr().offset(3), params[1]);
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
    rf_link_get_op_para(p, op_cmd.as_mut_ptr(), &op_cmd_len, params.as_mut_ptr(), &params_len, 1);

    if op_cmd_len != LightOpType::op_type_3 as u8 {
        return;
    }

    let vendor_id = (op_cmd[2] as u16) << 8 | op_cmd[1] as u16;
    let op = op_cmd[0] & 0x3F;

    if op == LGT_CMD_LIGHT_ONOFF {
        if cmd_left_delay_ms != 0 {
            return;
        }
        cmd_delay_ms = params[1] as u16 | ((params[2] as u16) << 8);
        if cmd_delay_ms != 0 && irq_timer1_cb_time == 0 {
            let cmd_delayed_ms = light_cmd_delayed_ms((*pp).val[(op_cmd_len + params_len) as usize]);
            if cmd_delay_ms > cmd_delayed_ms {
                cmd_delay = (*p).clone();
                cmd_left_delay_ms = cmd_delay_ms - cmd_delayed_ms;
                irq_timer1_cb_time = clock_time();
                return;
            }
        }
    }

    mesh_ota_timeout_handle(op, params.as_ptr());

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
            if rf_link_del_group(val) {
                cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
            }
        } else if params[0] == LIGHT_ADD_GRP_PARAM {
            if rf_link_add_group(val) {
                cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
            }
        }
    } else if op == LGT_CMD_CONFIG_DEV_ADDR {
        let val = params[0] as u16 | ((params[1] as u16) << 8);
        if !dev_addr_with_mac_flag(params.as_ptr()) || dev_addr_with_mac_match(params.as_ptr()) {
            if rf_link_add_dev_addr(val) {
                mesh_pair_proc_get_mac_flag();
            }
        }
    } else if op == LGT_CMD_SET_LIGHT
    {
        if params[8] & 0x1 != 0 {
            // Brightness
            let value = (params[1] as u16) << 8 | params[0] as u16;

            if light_off {
                led_lum = value;
                return;
            }
            light_step_reset(value);
        }
        if params[8] & 0x2 != 0 {
            // Temperature
            let value = (params[3] as u16) << 8 | params[2] as u16;

            led_val[0] = 0;
            led_val[1] = MAX_LUM_BRIGHTNESS_VALUE - value;
            led_val[2] = value;

            if light_off {
                return;
            }

            light_step_reset(led_lum);
        }

        lum_changed_time = clock_time();
    } else if op == LGT_CMD_KICK_OUT
    {
        if is_mesh_cmd_need_delay(p as *const u8, params.as_ptr(), (*pp).val[(op_cmd_len + params_len) as usize]) {
            return;
        }
        irq_disable();
        kick_out((params[0] as u32).try_into().unwrap());
        light_sw_reboot();
    } else if op == LGT_CMD_NOTIFY_MESH
    {
        light_notify((*pp).val.as_ptr().offset(3), 10, (*pp).src.as_ptr());
    } else if op == LGT_CMD_MESH_OTA_DATA
    {
        let idx = (params[0] as u16) | ((params[1] as u16) << 8);
        if !is_master_ota_st() {  // no update firmware for itself
            if CMD_START_MESH_OTA == idx {
                mesh_ota_master_start_firmware_from_own();
                //cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
            } else if CMD_START_MESH_OTA == idx {
                if is_mesh_ota_slave_running() {
                    // reboot to initial flash: should be delay to relay command.
                    mesh_ota_slave_reboot_delay();  // reboot after 320ms
                }
            } else {
                mesh_ota_slave_save_data(params.as_ptr());
            }
        } else {
            if CMD_STOP_MESH_OTA == idx {
                mesh_ota_master_cancle(OtaState::MASTER_OTA_REBOOT_ONLY as u8, false);
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
    mesh_push_user_command(cmd_sno, dst, cmd_op_para.as_ptr(), 13);

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
        if is_add_packet_buf_ready() {
            if !rf_link_add_tx_packet(addr_of!(pkt_light_notify) as *const u8) {
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
        mesh_node_init();
        device_status_update();
        cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
    } else if status == LGT_CMD_SET_DEV_ADDR {
        mesh_node_init();
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
    if irq_timer1_cb_time != 0 && clock_time_exceed(irq_timer1_cb_time, (cmd_left_delay_ms * 1000) as u32) {
        cmd_left_delay_ms = 0;
        rf_link_data_callback(&cmd_delay);
        cmd_delay_ms = 0;
        irq_timer1_cb_time = 0;
    }

    light_onoff_step_timer();
}

#[no_mangle]
fn irq_timer0() {}

#[no_mangle]
unsafe fn irq_handler__attribute_ram_code()
{
    irq_light_slave_handler();
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

    let value = if light_off { 0 } else { max(led_lum, 1) };

    // led_lum should not be 0, because app will take it to be light off
    st_val_par[0] = (value & 0xff) as u8;     //Note: bit7 of par[0] have been use internal for FLD_SYNCED
    st_val_par[1] = ((value >> 8) & 0xff) as u8;   // rsv
    // end

    ll_device_status_update(st_val_par.as_ptr(), st_val_par.len() as u8);
}

#[no_mangle]
pub unsafe fn light_onoff_normal(on: bool) {
    if on {
        light_off = false;
        light_adjust_RGB_hw(led_val[0], led_val[1], led_val[2], led_lum);
    } else {
        light_off = true;
        light_adjust_RGB_hw(0, 0, 0, 0);
    }
}

fn light_adjust_R(val: u16, lum: u16) {
    unsafe { pwm_set_lum(PWMID_R, get_pwm_cmp(val, lum), false); }
}

fn light_adjust_G(val: u16, lum: u16) {
    unsafe { pwm_set_lum(PWMID_G, get_pwm_cmp(val, lum), false); }
}

fn light_adjust_B(val: u16, lum: u16) {
    unsafe { pwm_set_lum(PWMID_B, get_pwm_cmp(val, lum), true); }
}

#[no_mangle]
pub fn light_adjust_RGB_hw(val_R: u16, val_G: u16, val_B: u16, lum: u16) {
    light_adjust_R(val_R, lum);
    light_adjust_G(val_G, lum);
    light_adjust_B(val_B, lum);
}