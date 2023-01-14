use std::mem::{MaybeUninit};
use app::App;
use config::get_flash_adr_light_new_fw;
use sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page, PAGE_SIZE};
use sdk::mcu::irq_i::irq_disable;
use sdk::mcu::register::{write_reg_clk_en1, write_reg_pwdn_ctrl, write_reg_rst1, write_reg_system_tick_ctrl};

mod sdk;
mod main_light;
mod common;
mod vendor_light;
mod app;
mod config;

pub static APP: App = App::default();

const FLASH_ADR_OTA_READY_FLAG: u32 = 0x3F000;
const FLASH_OTA_READY_FLAG: u8 = 0xa5;

#[inline(never)]
#[allow(non_snake_case)]
unsafe fn handle_ota_update__attribute_ram_code() {
    // This function requires that *everything* be in ram
    if *(FLASH_ADR_OTA_READY_FLAG as *const u8) != FLASH_OTA_READY_FLAG {
        // Nothing to do
        return;
    }

    // First configure the system clock
    write_reg_rst1(0);
    write_reg_clk_en1(0xff);
    write_reg_system_tick_ctrl(0x01);
    irq_disable();

    let mut	buff: [u8; 256] = MaybeUninit::uninit().assume_init();

    flash_read_page (*get_flash_adr_light_new_fw(), PAGE_SIZE, buff.as_mut_ptr());
	let	n = *(buff.as_ptr().offset(0x18) as *const u32);
    let mut i = 0;
    while i < n
	{
		if (i & 0xfff) == 0
		{
			flash_erase_sector (i);
		}

		flash_read_page (*get_flash_adr_light_new_fw() + i, PAGE_SIZE, buff.as_mut_ptr());
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
pub unsafe fn main_entrypoint() {
    // Must happen first
    handle_ota_update__attribute_ram_code();

    // Configure the system
    APP.init();

    // Run the application
    APP.run();
}
