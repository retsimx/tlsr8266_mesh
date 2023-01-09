use std::ptr::{copy_nonoverlapping};
use ::{BIT, flash_adr_pairing};
use sdk::common::bit::ONES_32;
use sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};
use sdk::light::{adr_flash_cfg_idx, DEV_ADDR_PAR_WITH_MAC, device_address, get_mac_en, slave_p_mac};
use sdk::mcu::analog::{analog_read__attribute_ram_code, analog_write__attribute_ram_code};
use sdk::mcu::clock::clock_time_exceed;
use vendor_light::adv_rsp_pri_data;

extern "C" {
	pub static mut mesh_pair_enable: bool;
	pub static mut mesh_pair_cmd_interval: u32;
	pub static mut mesh_pair_timeout: u32;

	pub static mut mesh_ota_master_100_flag: u8;

	// todo
	static mut effect_new_mesh: u8;
	static mut effect_new_mesh_delay_time: u32;
	pub static mut new_mesh_name: [u8; 16];
	pub static mut new_mesh_pwd: [u8; 16];
	pub static mut new_mesh_ltk: [u8; 16];
	fn save_effect_new_mesh();
}

pub const MESH_PAIR_CMD_INTERVAL: u32 = 500;
//unit: s
pub const MESH_PAIR_TIMEOUT: u32 = 10;

pub const FW_SIZE_MAX_K : u32 =	128;
pub const ERASE_SECTORS_FOR_OTA : u32 = (FW_SIZE_MAX_K + 3) / 4;

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
		mesh_pair_enable = true;
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

pub unsafe fn dev_addr_with_mac_flag(params: *const u8) -> bool
{
	return DEV_ADDR_PAR_WITH_MAC == *params.offset(2);
}

pub unsafe fn dev_addr_with_mac_rsp(params: *const u8, par_rsp: *mut u8) -> bool
{
	if dev_addr_with_mac_match(params) {
		*par_rsp.offset(0) = (device_address & 0xff) as u8;
		*par_rsp.offset(1) = ((device_address >> 8) & 0xff) as u8;

		copy_nonoverlapping(slave_p_mac, par_rsp.offset( 2), 6);
		#[allow(unaligned_references)]
		copy_nonoverlapping(&adv_rsp_pri_data.ProductUUID, par_rsp.offset( 8) as *mut u16, 2);
		return true;
	}
	return false;
}

pub unsafe fn dev_addr_with_mac_match(params: *const u8) -> bool
{
	return if (*params.offset(0) == 0xff) && (*params.offset(1) == 0xff) {    // get
		get_mac_en
	} else {
		for i in 0..6 {
			if params.offset(4+i) != slave_p_mac.offset(i) {
				return false;
			}
		}
		return true;
	}
}

pub unsafe fn get_mesh_pair_checksum(idx: u8) -> u8 {
    let i = (idx % 8) as usize;
    return (new_mesh_name[i] ^ new_mesh_name[i+8]) ^ (new_mesh_pwd[i] ^ new_mesh_pwd[i+8]) ^ (new_mesh_ltk[i] ^ new_mesh_ltk[i+8]);
}

pub fn light_cmd_delayed_ms(data: u8) -> u16 {
	let ttc_prec = data >> 6;
	let ttc_val: u16 	= (data & 0x3F) as u16;
	return if ttc_prec == 0 { ttc_val } else {
		if ttc_prec == 1 { ttc_val << 2 } else {
			if ttc_prec == 2 { ttc_val << 4 } else {
				if ttc_prec == 3 { ttc_val << 8 } else { 0 }
			}
		}
	}
}

pub unsafe fn mesh_pair_proc_get_mac_flag(){
	get_mac_en = false; 	// set success
	if mesh_pair_enable {
		let mut data: [u8; 1] = [0];
		flash_write_page(flash_adr_pairing + adr_flash_cfg_idx + 1, 1, data.as_mut_ptr());
		if data[0] == 1 {get_mac_en = true} else {get_mac_en = false}
	}
}