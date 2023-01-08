use BIT;
use sdk::common::bit::ONES_32;
use sdk::drivers::flash::{flash_erase_sector, flash_read_page};
use sdk::mcu::analog::{analog_read__attribute_ram_code, analog_write__attribute_ram_code};
use sdk::mcu::clock::clock_time_exceed;

extern "C" {
	pub static mut mesh_pair_enable: u8;
	pub static mut mesh_pair_cmd_interval: u32;
	pub static mut mesh_pair_timeout: u32;

	pub static mut mesh_ota_master_100_flag: u8;

	// todo
	static mut effect_new_mesh: u8;
	static mut effect_new_mesh_delay_time: u32;
	fn save_effect_new_mesh();
}

pub static MESH_PAIR_CMD_INTERVAL: u32 = 500;
//unit: s
pub static MESH_PAIR_TIMEOUT: u32 = 10;

pub static FW_SIZE_MAX_K : u32 =	128;
pub static ERASE_SECTORS_FOR_OTA : u32 = (FW_SIZE_MAX_K + 3) / 4;

pub const rega_light_off: u8 = 0x3a;

// recover status before software reboot
pub enum RECOVER_STATUS {
	FLD_LIGHT_OFF				= BIT!(0),
	FLD_MESH_OTA_MASTER_100		= BIT!(1),
	LOW_BATT_FLG                = BIT!(2),
	// LOW_BATT_LOOP_FLG           = BIT(3),    // 0 means check by user_init, 1 means by main loop
}

pub fn is_ota_area_valid(adr: u32) -> bool {
	let mut buf : [u8; 4] = [0; 4];
	for i in 0..ERASE_SECTORS_FOR_OTA {
    	flash_read_page(adr + i*0x1000, 4, buf.as_mut_ptr());
    	let tmp = buf[0] as u32 | (buf[1] as u32) << 8 | (buf[2] as u32) << 16 | (buf[3] as u32) << 24;
    	if tmp != ONES_32 {
            return false;
    	}
    }
	return true;
}

pub fn erase_ota_data(adr: u32){
    for i in 0..ERASE_SECTORS_FOR_OTA {
        flash_erase_sector(adr+(ERASE_SECTORS_FOR_OTA -1 - i)*0x1000);
    }
}

pub fn mesh_pair_init() {
	unsafe {
		mesh_pair_enable = 1;
		mesh_pair_cmd_interval = MESH_PAIR_CMD_INTERVAL;
		mesh_pair_timeout = MESH_PAIR_TIMEOUT;
	}
}

pub fn mesh_pair_proc_effect() {
	unsafe {
		if effect_new_mesh != 0 || (effect_new_mesh_delay_time != 0 && (clock_time_exceed(effect_new_mesh_delay_time, mesh_pair_cmd_interval * 1000))) {
			save_effect_new_mesh();
			effect_new_mesh = 0;
			effect_new_mesh_delay_time = 0;
		}
	}
}

pub fn mesh_ota_master_100_flag_check()
{
	let val = analog_read__attribute_ram_code(rega_light_off);
	if val & RECOVER_STATUS::FLD_MESH_OTA_MASTER_100 as u8 != 0 {
		unsafe { mesh_ota_master_100_flag = 1; }
		analog_write__attribute_ram_code(rega_light_off, val & !(RECOVER_STATUS::FLD_MESH_OTA_MASTER_100 as u8));
	}
}