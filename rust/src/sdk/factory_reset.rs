use std::cmp::min;
use std::ptr::{copy_nonoverlapping, addr_of};
use ::{BIT, flash_adr_reset_cnt};
use ::{flash_adr_pairing, MESH_PWD};
use ::{OUT_OF_MESH, PAIR_VALID_FLAG};
use common::mesh_pair_enable;
use sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};
use sdk::light::{encode_password, light_sw_reboot, pair_config_mesh_ltk};
use sdk::mcu::clock::clock_time_exceed;
use sdk::mcu::irq_i::{irq_disable, irq_restore};

extern "C" {
	// todo
	fn rf_led_ota_ok();

	static mut pair_config_pwd_encode_enable: u8;
	static mut get_mac_en: u8;
}

const SERIALS_CNT: u8 = 5;   // must less than 7
const factory_reset_serials : [u8; (SERIALS_CNT * 2) as usize] = [
	0, 3,    // [0]:must 0
	0, 3,    // [2]:must 0
	0, 3,    // [4]:must 0
	3, 30,
	3, 30
];

const RESET_CNT_RECOUNT_FLAG: u8 =          0;
const RESET_FLAG: u8 =                      0x80;

const FLASH_ADR_RESET_CNT: u32 = 0x7A000;
const FLASH_ADR_PAR_MAX: u32 = 0x80000;
const CFG_ADR_MAC_512K_FLASH: u32 = 0x76000;
const CFG_SECTOR_ADR_MAC_CODE: u32 = CFG_ADR_MAC_512K_FLASH;

static mut adr_reset_cnt_idx : u32 = 0;
static mut reset_cnt: u8 = 0;
static mut clear_st: u8 = 3;
static mut reset_check_time: u32 = 0;

unsafe fn reset_cnt_clean()
{
	if adr_reset_cnt_idx < 3840
	{
		return;
	}
	flash_erase_sector (flash_adr_reset_cnt);
	adr_reset_cnt_idx = 0;
}

unsafe fn write_reset_cnt(cnt: u8)
{
	let data = [cnt];
	flash_write_page (flash_adr_reset_cnt + adr_reset_cnt_idx, 1, data.as_ptr());
}

unsafe fn clear_reset_cnt ()
{
    write_reset_cnt(RESET_CNT_RECOUNT_FLAG);
}

unsafe fn reset_cnt_get_idx()
{
	let pf = flash_adr_reset_cnt as *const u8;
	adr_reset_cnt_idx = 0;
	while adr_reset_cnt_idx < 4096
	{
	    let restcnt_bit = *pf.offset(adr_reset_cnt_idx as isize);
		if restcnt_bit != RESET_CNT_RECOUNT_FLAG    //end
		{
        	if ((!(BIT!(0)|BIT!(1)|BIT!(2)|BIT!(3))) as u8 == restcnt_bit)  // the fourth not valid
        	 || ((!(BIT!(0)|BIT!(1)|BIT!(2)|BIT!(3)|BIT!(4)|BIT!(5))) as u8 == restcnt_bit) {  // the fifth not valid
                clear_reset_cnt();
            } else {
			    break;
			}
		}
		adr_reset_cnt_idx += 1;
	}

    reset_cnt_clean();
}

unsafe fn get_reset_cnt_bit() -> u8
{
	if adr_reset_cnt_idx < 0
	{
	    reset_cnt_clean();
		return 0;
	}

	let mut data = [0];
	flash_read_page(flash_adr_reset_cnt + adr_reset_cnt_idx, 1, data.as_mut_ptr());
	reset_cnt = data[0];
	return reset_cnt;
}

unsafe fn increase_reset_cnt()
{
	let mut restcnt_bit = get_reset_cnt_bit();
	for i in 0..8 {
        if restcnt_bit & BIT!(i) != 0 {
            if i < 3 {
                reset_cnt = i;
            }else if i < 5 {
                reset_cnt = 3;
            }else if i < 7 {
                reset_cnt = 4;
            }

            restcnt_bit &= !(BIT!(i));
            write_reset_cnt(restcnt_bit);
            break;
        }
	}
}

pub unsafe fn factory_reset_handle()
{
    reset_cnt_get_idx();
    let restcnt_bit = get_reset_cnt_bit();
	if restcnt_bit == RESET_FLAG {
        irq_disable();
        factory_reset();
        rf_led_ota_ok();
	    light_sw_reboot();
	} else {
        increase_reset_cnt();
	}
}

pub unsafe fn factory_reset_cnt_check()
{
	if 0 == clear_st {
		return;
	}

	if 3 == clear_st {
        clear_st -= 1;
        reset_check_time = factory_reset_serials[reset_cnt as usize * 2] as u32;
    }

	if (2 == clear_st) && clock_time_exceed(0, reset_check_time*1000*1000) {
	    clear_st -= 1;
	    reset_check_time = factory_reset_serials[reset_cnt as usize * 2 + 1] as u32;
	    if 3 == reset_cnt || 4 == reset_cnt {
            increase_reset_cnt();
        }
	}

	if (1 == clear_st) && clock_time_exceed(0, reset_check_time*1000*1000) {
	    clear_st = 0;
        clear_reset_cnt();
	}
}

fn factory_reset() {
	let r = irq_disable();
	//clear_reset_cnt();
	for i in 1..((FLASH_ADR_PAR_MAX - CFG_SECTOR_ADR_MAC_CODE) / 4096)
	{
	    let adr = CFG_SECTOR_ADR_MAC_CODE + i*0x1000;
	    if FLASH_ADR_RESET_CNT != adr {
		    flash_erase_sector(adr);
		}
	}

    flash_erase_sector(FLASH_ADR_RESET_CNT); // at last should be better, when power off during factory reset erase.

    irq_restore(r);
}

#[derive(PartialEq)]
pub enum KICKOUT_REASON{
	OUT_OF_MESH = 0,
	DEFAULT_NAME,
	MODE_MAX,
}

#[no_mangle]
unsafe fn kick_out(par: KICKOUT_REASON) {
    factory_reset();

    if par == KICKOUT_REASON::OUT_OF_MESH {
        let mut buff: [u8; 16] = [0; 16];
        copy_nonoverlapping(pair_config_mesh_ltk.as_ptr(), buff.as_mut_ptr(), 16);
        flash_write_page (flash_adr_pairing + 48, 16, buff.as_mut_ptr());

		let mut buff: [u8; 16] = [0; 16];
		copy_nonoverlapping(MESH_PWD.as_ptr(), buff.as_mut_ptr(), min(MESH_PWD.len(), buff.len()));
	    encode_password(buff.as_mut_ptr());
        flash_write_page (flash_adr_pairing + 32, 16, buff.as_mut_ptr());

		let mut buff: [u8; 16] = [0; 16];
		copy_nonoverlapping(OUT_OF_MESH.as_ptr(), buff.as_mut_ptr(), min(OUT_OF_MESH.len(), buff.len()));
        flash_write_page (flash_adr_pairing + 16, 16, buff.as_mut_ptr());

		let mut buff: [u8; 16] = [0; 16];
        buff[0] = PAIR_VALID_FLAG;
        if pair_config_pwd_encode_enable != 0 {
            buff[15] = PAIR_VALID_FLAG;
        }

		if mesh_pair_enable != 0 {
	        get_mac_en = 1;
	        buff[1] = get_mac_en;
        }
        flash_write_page (flash_adr_pairing, 16, buff.as_mut_ptr());
    }

    rf_led_ota_ok();
}