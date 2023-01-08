use std::mem::{size_of, size_of_val, transmute};
use std::ptr::{copy_nonoverlapping, write_bytes, addr_of};
use common::{erase_ota_data, is_ota_area_valid, mesh_ota_master_100_flag_check, mesh_pair_init, mesh_pair_proc_effect, RECOVER_STATUS, rega_light_off};
use ::{flash_adr_light_new_fw, PAIR_VALID_FLAG};
use ::{MESH_NAME, MESH_PWD};
use ::{MESH_LTK, VENDOR_ID};
use ::{MAX_LUM_BRIGHTNESS_VALUE, PWMID_B};
use ::{PWMID_G, PWMID_R};
use ::{PWM_G, PWM_R};
use ::{BIT, PWM_B};
use ::{flash_adr_lum, FLASH_SECTOR_SIZE};
use sdk::drivers::flash::{flash_erase_sector, flash_write_page};
use sdk::drivers::pwm::{pwm_set_cmp, pwm_set_duty, pwm_start};
use sdk::light::{adv_private_data_len, AUTH_TIME, BRIDGE_MAX_CNT, IRQ_TIME1_INTERVAL, is_receive_ota_window, light_set_tick_per_us, ll_adv_private_t, ll_adv_rsp_private_t, mesh_get_fw_version, mesh_security_enable, online_status_timeout, ONLINE_STATUS_TIMEOUT, p_adv_pri_data, p_adv_rsp_data, pair_config_mesh_ltk, pair_config_mesh_name, pair_config_mesh_pwd, pair_config_valid_flag, pair_login_ok, PMW_MAX_TICK, register_mesh_ota_master_ui, rf_link_slave_proc, security_enable, setSppUUID, slave_first_connected_tick, user_data, user_data_len, vendor_id_init};
use sdk::mcu::analog::{analog_read__attribute_ram_code, analog_write__attribute_ram_code};
use sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, CLOCK_SYS_CLOCK_HZ, clock_time, clock_time_exceed};
use sdk::mcu::gpio::{AS_GPIO, gpio_set_func};
use sdk::mcu::register::{FLD_IRQ, FLD_TMR, read_reg_irq_mask, write_reg_irq_mask, write_reg_tmr1_tick, write_reg_tmr1_capt, write_reg_tmr_ctrl, read_reg_tmr_ctrl};
use sdk::pm::usb_dp_pullup_en;
use sdk::rf_drv::{rf_link_set_max_bridge, rf_link_slave_init, rf_link_slave_pairing_enable, rf_link_slave_set_buffer, RF_POWER, rf_set_power_level_index};
use sdk::service::{TELINK_SPP_DATA_CLIENT2SERVER, TELINK_SPP_DATA_OTA, TELINK_SPP_DATA_PAIR, TELINK_SPP_DATA_SERVER2CLIENT, TELINK_SPP_UUID_SERVICE};

extern "C" {
    static advData: [u8; 3];
    static mut max_mesh_name_len: u8;
    static mut adv_pri_data: ll_adv_private_t;
    static mut adv_rsp_pri_data: ll_adv_rsp_private_t;


    // todo
    fn factory_reset_handle();
    fn factory_reset_cnt_check();
    fn vendor_set_adv_data();
    fn device_status_update();
    fn light_onoff_step(on: bool);
    fn light_adjust_R(val: u16, lum: u16);
    fn light_adjust_G(val: u16, lum: u16);
    fn light_adjust_B(val: u16, lum: u16);
    fn light_adjust_RGB_hw(val_R: u16, val_G: u16, val_B: u16, lum: u16);

    static mut light_off: u8;
    static mut lum_changed_time: u32;
    static mut light_lum_addr: u32;
    static mut led_lum: u16;
    static mut led_val: [u16; 3];
}

static LED_INDICATE_VAL: u16 = 0xffff;
static LED_MASK: u8 =							0x07;
static LUM_SAVE_FLAG: u8 = 0xA5;

macro_rules! config_led_event {
    ($on:expr, $off:expr, $n:expr, $sel:expr) => {
        $on as u32 | ($off as u32) << 8 | ($n as u32) << 16 | ($sel as u32) << 24
    }
}

static LED_EVENT_FLASH_4HZ_10S	: u32 =			config_led_event!(2,2,40,LED_MASK);
static LED_EVENT_FLASH_STOP		: u32 =		    config_led_event!(1,1,1,LED_MASK);
static LED_EVENT_FLASH_2HZ_2S		: u32 =		config_led_event!(4,4,4,LED_MASK);
static LED_EVENT_FLASH_1HZ_1S		: u32 =		config_led_event!(8,8,1,LED_MASK);
static LED_EVENT_FLASH_1HZ_2S		: u32 =		config_led_event!(8,8,2,LED_MASK);
static LED_EVENT_FLASH_1HZ_3S		: u32 =		config_led_event!(8,8,3,LED_MASK);
static LED_EVENT_FLASH_1HZ_4S		: u32 =		config_led_event!(8,8,4,LED_MASK);
static LED_EVENT_FLASH_4HZ		: u32 =			config_led_event!(2,2,0,LED_MASK);
static LED_EVENT_FLASH_1HZ		: u32 =			config_led_event!(8,8,0,LED_MASK);
static LED_EVENT_FLASH_4HZ_3T		: u32 =		config_led_event!(2,2,3,LED_MASK);
static LED_EVENT_FLASH_1HZ_3T		: u32 =		config_led_event!(8,8,3,LED_MASK);
static LED_EVENT_FLASH_0p25HZ_1T	: u32 =		config_led_event!(4,60,1,LED_MASK);


#[no_mangle]
static mut buff_response: [[u32; 9]; 48] = [[0; 9]; 48];

#[no_mangle]
static mut led_event_pending: u32 = 0;

static  mut led_count : u32 = 0;
static	mut led_ton : u32 = 0;
static	mut led_toff : u32 = 0;
static	mut led_sel: u32 = 0;
static	mut led_tick : u32 = 0;
static	mut led_no : u32 = 0;
static	mut led_is_on : u32 = 0;

#[repr(C, packed)]
struct lum_save_t {
    save_falg: u8,
    lum: u16,
    ledval: [u16; 3]
}

// fn calculate_lumen_map(val: u16) -> f32 {
//     let percentage = (val as f32 / MAX_LUM_BRIGHTNESS_VALUE as f32) * 100.0;
//     return (-0.00539160*micromath::F32Ext::powf(percentage, 3.0))+(4.47709595*micromath::F32Ext::powf(percentage, 2.0))+(153.72442036*percentage);
// }

fn pwm_set_lum (id: u32, y: u32, pol: bool)
{
    let lum = (y * PMW_MAX_TICK as u32) / MAX_LUM_BRIGHTNESS_VALUE as u32;

	pwm_set_cmp(id, if pol {PMW_MAX_TICK as u32 - lum} else {lum} as u16);
}

fn cfg_led_event(e: u32) {
    unsafe { led_event_pending = e; }
}

fn mesh_ota_master_led(_: *const u8)
{
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
            max_mesh_name_len = if max_mesh_name_len < 16 {max_mesh_name_len} else {16};
        }
	}

    // get fw version @flash 0x02,0x03,0x04,0x05
    unsafe { mesh_get_fw_version(); }

	//add the user_data after the adv_pri_data
	let user_const_data: [u8; 6] = [0x05,0x02,0x19,0x00,0x69,0x69];
    unsafe { copy_nonoverlapping(user_const_data.as_ptr(), user_data.as_mut_ptr(), user_const_data.len()); }
    unsafe { user_data_len = 0; } //disable add the userdata after the adv_pridata

    unsafe { light_set_tick_per_us(CLOCK_SYS_CLOCK_HZ / 1000000); }

    unsafe { pair_config_valid_flag = PAIR_VALID_FLAG; }

    unsafe { write_bytes(pair_config_mesh_name.as_mut_ptr(), 0, pair_config_mesh_name.len()); }
    unsafe { copy_nonoverlapping(MESH_NAME.as_ptr(), pair_config_mesh_name.as_mut_ptr(), if MESH_NAME.len() > max_mesh_name_len as usize { max_mesh_name_len } else { MESH_NAME.len() as u8 } as usize); }

    unsafe { write_bytes(pair_config_mesh_pwd.as_mut_ptr(), 0, pair_config_mesh_pwd.len()); }
    unsafe { copy_nonoverlapping(MESH_PWD.as_ptr(), pair_config_mesh_pwd.as_mut_ptr(), if MESH_PWD.len() > 16 as usize { 16 } else { MESH_PWD.len() as u8 } as usize); }

    unsafe { write_bytes(pair_config_mesh_ltk.as_mut_ptr(), 0, pair_config_mesh_ltk.len()); }
    unsafe { copy_nonoverlapping(MESH_LTK.as_ptr(), pair_config_mesh_ltk.as_mut_ptr(), if MESH_LTK.len() > 16 as usize { 16 } else { MESH_LTK.len() as u8 } as usize); }

    unsafe {
        setSppUUID(
        TELINK_SPP_UUID_SERVICE.as_ptr(),
        TELINK_SPP_DATA_SERVER2CLIENT.as_ptr(),
        TELINK_SPP_DATA_CLIENT2SERVER.as_ptr(),
        TELINK_SPP_DATA_OTA.as_ptr(),
        TELINK_SPP_DATA_PAIR.as_ptr()
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

	usb_dp_pullup_en (true);

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

    pwm_set_duty (PWMID_R, PMW_MAX_TICK, PMW_MAX_TICK);
    pwm_set_duty (PWMID_G, PMW_MAX_TICK, PMW_MAX_TICK);
    pwm_set_duty (PWMID_B, PMW_MAX_TICK, 0);

    //retrieve lumen value
    unsafe { light_lum_retrieve(); }

    pwm_start (PWMID_R);
    pwm_start (PWMID_G);
    pwm_start (PWMID_B);

    gpio_set_func (PWM_R as u32, !AS_GPIO);
    gpio_set_func (PWM_G as u32, !AS_GPIO);
    gpio_set_func (PWM_B as u32, !AS_GPIO);

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
		led_toff = ((led_event_pending>>8) & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US;
		led_count = (led_event_pending>>16) & 0xff;
		led_sel = led_event_pending>>24;

		led_event_pending = 0;
		led_tick = clock_time () + 30000000 * CLOCK_SYS_CLOCK_1US;
		led_no = 0;
		led_is_on = 0;
	}

    if clock_time() - led_tick >= (if led_is_on != 0 {led_ton} else {led_toff}) {
        led_tick = clock_time ();
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
                light_onoff_hw(light_off == 0); // should not report online status again
                return ;
            }
        }

        if  led_off || led_on   {
            if led_sel & BIT!(0) != 0
            {
                light_adjust_G (LED_INDICATE_VAL * led_on as u16, 0xffff);
            }
            if led_sel & BIT!(1) != 0
            {
                light_adjust_B (LED_INDICATE_VAL * led_on as u16, 0xffff);
            }
            if led_sel & BIT!(2) != 0
            {
                light_adjust_R (LED_INDICATE_VAL * led_on as u16, 0xffff);
            }
            if led_sel & BIT!(5) != 0
            {
            }
        }
    }
}
                                   
fn light_auth_check() {
    unsafe {
        if security_enable != 0 && pair_login_ok == 0 && slave_first_connected_tick != 0 && clock_time_exceed(slave_first_connected_tick, AUTH_TIME * 1000 * 1000) {
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
unsafe fn light_lum_store(){
	if light_lum_addr >= (flash_adr_lum + FLASH_SECTOR_SIZE as u32 - size_of::<lum_save_t>() as u32) {
		light_lum_erase();
		return;
	}

    let mut lum_save = lum_save_t {
        save_falg: LUM_SAVE_FLAG,
        lum: led_lum,
        ledval: [0, 0, 0]
    };

    #[allow(unaligned_references)]
    copy_nonoverlapping(led_val.as_ptr(), lum_save.ledval.as_mut_ptr(), led_val.len());

	flash_write_page(light_lum_addr, size_of::<lum_save_t>() as u32, addr_of!(lum_save) as *const u8);
	light_lum_addr += size_of::<lum_save_t>() as u32;

	return;
}

pub fn light_onoff_hw(on: bool) {
    unsafe { light_onoff_step(on); }
}

pub fn light_onoff(on: bool) {
    light_onoff_hw(on);

    unsafe { device_status_update(); }
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
		}else if (*lum_save).save_falg == 0xFF {
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
	}else{
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

	light_user_func ();

    unsafe { rf_link_slave_proc(); }

    unsafe { proc_led(); }
}