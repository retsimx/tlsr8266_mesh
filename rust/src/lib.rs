use std::ffi::CStr;
use std::io::Write;
use std::mem;
use std::mem::{MaybeUninit, transmute};
use std::ptr::addr_of;
use main_light::{main_loop, user_init};
use sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page, PAGE_SIZE};
use sdk::light::pair_config_pwd_encode_sk;
use sdk::mcu::dma::dma_init;
use sdk::mcu::gpio::{gpio_init, GPIO_PIN_TYPE};
use sdk::mcu::clock::clock_init;
use sdk::mcu::gpio::GPIO_PIN_TYPE::{GPIO_PA1, GPIO_PC0, GPIO_PC2, GPIO_PC4};
use sdk::mcu::irq_i::{irq_disable, irq_enable, irq_init};
use sdk::mcu::register::{write_reg_clk_en1, write_reg_pwdn_ctrl, write_reg_rst1, write_reg_system_tick_ctrl};
use sdk::mcu::watchdog::wd_clear;
use sdk::pm;

mod sdk;
mod main_light;
mod common;
mod vendor_light;
mod ota;
mod config;
mod app;
mod math_wrapper;

// General stuff
pub const PAIR_VALID_FLAG: u8 = 0xFA;
// 0~max_mesh_name_len bytes  (strlen(advData) + strlen(MESH_NAME) + sizeof(ll_adv_private_t))<=31
pub const MESH_NAME: &str = "out_of_mesh";
// max 16 bytes
pub const MESH_PWD: &str = "123";
// max 16 bytes
pub const MESH_PWD_ENCODE_SK: &str = "0123456789ABCDEF";
// max 16 bytes, random data from master
pub const MESH_LTK: [u8; 16] = [0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf];

pub const DEVICE_NAME: const_cstr::ConstCStr = const_cstr::const_cstr!("Telink tLight");

pub const OUT_OF_MESH: &str = "out_of_mesh";

pub const MAX_LUM_BRIGHTNESS_VALUE: u16 = 0xffff;

pub const VENDOR_ID: u16 = 0x0211;

// don't use GPIO_PB0(pwm5) of 8266, because its default status is ouput 1 as GPIO.
pub const PWM_R: GPIO_PIN_TYPE =     GPIO_PC4;
pub const PWM_G: GPIO_PIN_TYPE =     GPIO_PC0;
pub const PWM_B: GPIO_PIN_TYPE =     GPIO_PC2;
pub const PWM_W: GPIO_PIN_TYPE =     GPIO_PA1;

pub const PWMID_R: u32 =     2;
pub const PWMID_G: u32 =     0;
pub const PWMID_B: u32 =     1;
pub const PWMID_W: u32 =     3;

pub const OTA_LED: GPIO_PIN_TYPE = PWM_R;

pub const FLASH_SECTOR_SIZE: u16 = 4096;

extern "C" {
    fn rf_drv_init(mode: i32);
}

// 512K flash
#[no_mangle]
pub static flash_adr_mac: u32 = 0x76000;
#[no_mangle]
pub static flash_adr_pairing: u32 = 0x77000;
#[no_mangle]
pub static flash_adr_dev_grp_adr: u32 = 0x79000;
#[no_mangle]
pub static flash_adr_lum: u32 = 0x78000;
#[no_mangle]
pub static flash_adr_ota_master: u32 = 0x20000;
#[no_mangle]
pub static flash_adr_reset_cnt: u32 = 0x7A000;
#[no_mangle]
pub static flash_adr_alarm: u32 = 0x7B000;
#[no_mangle]
pub static flash_adr_scene: u32 = 0x7C000;
#[no_mangle]
pub static flash_adr_user_data: u32 = 0x7D000;
#[no_mangle]
pub static flash_adr_light_new_fw: u32 = 0x40000;

fn fill_from_str(mut bytes: &mut [u8], s: &[u8]) {
    bytes.write(s).unwrap();
}

const FLASH_ADR_OTA_READY_FLAG: u32 = 0x3F000;
const FLASH_OTA_READY_FLAG: u8 = 0xa5;

#[inline(never)]
unsafe fn handle_ota_update__attribute_ram_code() {
    // This function requires that *everything* be in ram
    if *(FLASH_ADR_OTA_READY_FLAG as *const u8) != FLASH_OTA_READY_FLAG {
        // Nothing to do
        return;
    }

    // Ota is ready to go.
    // First configure the system clock
    write_reg_rst1(0);
    write_reg_clk_en1(0xff);
    write_reg_system_tick_ctrl(0x01);
    irq_disable();

    let mut	buff: [u8; 256] = MaybeUninit::uninit().assume_init();

    flash_read_page (flash_adr_light_new_fw, PAGE_SIZE, buff.as_mut_ptr());
	let	n = *(buff.as_ptr().offset(0x18) as *const u32);
    let mut i = 0;
    while i < n
	{
		if (i & 0xfff) == 0
		{
			flash_erase_sector (i);
		}

		flash_read_page (flash_adr_light_new_fw + i, PAGE_SIZE, buff.as_mut_ptr());
		flash_write_page (i, PAGE_SIZE, buff.as_mut_ptr());

        i += PAGE_SIZE;
	}

	buff[0] = 0;

	flash_write_page (FLASH_ADR_OTA_READY_FLAG, 1, buff.as_mut_ptr());	//clear OTA flag

    // Force reboot
    write_reg_pwdn_ctrl(0x20);

    loop {}
}

#[no_mangle]
pub fn main_entrypoint() -> i32 {
    // Must happen first
    unsafe { handle_ota_update__attribute_ram_code(); }

    unsafe { pm::cpu_wakeup_init() }

    unsafe { fill_from_str(&mut pair_config_pwd_encode_sk, MESH_PWD_ENCODE_SK.as_bytes()); }

    clock_init();
    dma_init();
    gpio_init();
    irq_init();

    unsafe { rf_drv_init(0) }

    user_init();

    irq_enable();

    loop {
        wd_clear();
        main_loop();
    }
}
