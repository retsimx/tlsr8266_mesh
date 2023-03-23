use crate::config::{
    get_flash_adr_pairing, get_flash_adr_reset_cnt, MESH_PWD, OUT_OF_MESH, PAIR_VALID_FLAG,
};
use crate::mesh::wrappers::{get_mesh_pair_enable, set_get_mac_en};
use crate::sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};
use crate::sdk::light::{_encode_password, _light_sw_reboot, get_pair_config_mesh_ltk};
use crate::sdk::mcu::clock::clock_time_exceed;
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use core::cmp::min;
use core::convert::TryFrom;
use core::ptr::{addr_of, copy_nonoverlapping};
use crate::BIT;
use crate::{app, pub_mut};

pub_mut!(pair_config_pwd_encode_enable, u8);

const SERIALS_CNT: u8 = 5; // must less than 7
const FACTORY_RESET_SERIALS: [u8; (SERIALS_CNT * 2) as usize] = [
    0, 3, // [0]:must 0
    0, 3, // [2]:must 0
    0, 3, // [4]:must 0
    3, 30, 3, 30,
];

pub const RESET_CNT_RECOUNT_FLAG: u8 = 0;
pub const RESET_FLAG: u8 = 0x80;

pub const FLASH_ADR_RESET_CNT: u32 = 0x7A000;
pub const FLASH_ADR_PAR_MAX: u32 = 0x80000;
pub const CFG_ADR_MAC_512K_FLASH: u32 = 0x76000;
pub const CFG_SECTOR_ADR_MAC_CODE: u32 = CFG_ADR_MAC_512K_FLASH;

pub_mut!(adr_reset_cnt_idx, u32, 0);
pub_mut!(reset_cnt, u8, 0);
pub_mut!(clear_st, u8, 3);
pub_mut!(reset_check_time, u32, 0);

fn reset_cnt_clean() {
    if *get_adr_reset_cnt_idx() < 3840 {
        return;
    }
    flash_erase_sector(*get_flash_adr_reset_cnt());
    set_adr_reset_cnt_idx(0);
}

fn write_reset_cnt(cnt: u8) {
    let data = [cnt];
    flash_write_page(
        *get_flash_adr_reset_cnt() + *get_adr_reset_cnt_idx(),
        1,
        data.as_ptr(),
    );
}

fn clear_reset_cnt() {
    write_reset_cnt(RESET_CNT_RECOUNT_FLAG);
}

fn reset_cnt_get_idx() {
    let pf = *get_flash_adr_reset_cnt() as *const u8;
    set_adr_reset_cnt_idx(0);
    while *get_adr_reset_cnt_idx() < 4096 {
        let restcnt_bit = unsafe { *pf.offset(adr_reset_cnt_idx as isize) };
        if restcnt_bit != RESET_CNT_RECOUNT_FLAG
        //end
        {
            if ((!(BIT!(0)|BIT!(1)|BIT!(2)|BIT!(3))) as u8 == restcnt_bit)  // the fourth not valid
        	 || ((!(BIT!(0)|BIT!(1)|BIT!(2)|BIT!(3)|BIT!(4)|BIT!(5))) as u8 == restcnt_bit)
            {
                // the fifth not valid
                clear_reset_cnt();
            } else {
                break;
            }
        }
        set_adr_reset_cnt_idx(*get_adr_reset_cnt_idx() + 1);
    }

    reset_cnt_clean();
}

fn get_reset_cnt_bit() -> u8 {
    if *get_adr_reset_cnt_idx() < 0 {
        reset_cnt_clean();
        return 0;
    }

    let mut data = [0];
    flash_read_page(
        *get_flash_adr_reset_cnt() + *get_adr_reset_cnt_idx(),
        1,
        data.as_mut_ptr(),
    );
    set_reset_cnt(data[0]);
    return *get_reset_cnt();
}

fn increase_reset_cnt() {
    let mut restcnt_bit = get_reset_cnt_bit();
    for i in 0..8 {
        if restcnt_bit & BIT!(i) != 0 {
            if i < 3 {
                set_reset_cnt(i);
            } else if i < 5 {
                set_reset_cnt(3);
            } else if i < 7 {
                set_reset_cnt(4);
            }

            restcnt_bit &= !(BIT!(i));
            write_reset_cnt(restcnt_bit);
            break;
        }
    }
}

pub fn factory_reset_handle() {
    reset_cnt_get_idx();
    let restcnt_bit = get_reset_cnt_bit();
    if restcnt_bit == RESET_FLAG {
        irq_disable();
        factory_reset();
        app().ota_manager.rf_led_ota_ok();
        _light_sw_reboot();
    } else {
        increase_reset_cnt();
    }
}

pub fn factory_reset_cnt_check() {
    if *get_clear_st() == 0 {
        return;
    }

    if *get_clear_st() == 3 {
        set_clear_st(*get_clear_st() - 1);
        set_reset_check_time(FACTORY_RESET_SERIALS[*get_reset_cnt() as usize * 2] as u32);
    }

    if *get_clear_st() == 2 && clock_time_exceed(0, *get_reset_check_time() * 1000 * 1000) {
        set_clear_st(*get_clear_st() - 1);
        set_reset_check_time(FACTORY_RESET_SERIALS[*get_reset_cnt() as usize * 2 + 1] as u32);
        if *get_reset_cnt() == 3 || *get_reset_cnt() == 4 {
            increase_reset_cnt();
        }
    }

    if *get_clear_st() == 1 && clock_time_exceed(0, *get_reset_check_time() * 1000 * 1000) {
        set_clear_st(0);
        clear_reset_cnt();
    }
}

fn factory_reset() {
    let r = irq_disable();
    //clear_reset_cnt();
    for i in 1..((FLASH_ADR_PAR_MAX - CFG_SECTOR_ADR_MAC_CODE) / 4096) {
        let adr = CFG_SECTOR_ADR_MAC_CODE + i * 0x1000;
        if FLASH_ADR_RESET_CNT != adr {
            flash_erase_sector(adr);
        }
    }

    flash_erase_sector(FLASH_ADR_RESET_CNT); // at last should be better, when power off during factory reset erase.

    irq_restore(r);
}

#[derive(PartialEq)]
pub enum KickoutReason {
    OutOfMesh = 0,
    DefaultName,
    ModeMax,
}

impl TryFrom<u32> for KickoutReason {
    type Error = ();

    fn try_from(v: u32) -> Result<Self, Self::Error> {
        match v {
            x if x == KickoutReason::OutOfMesh as u32 => Ok(KickoutReason::OutOfMesh),
            x if x == KickoutReason::DefaultName as u32 => Ok(KickoutReason::DefaultName),
            x if x == KickoutReason::ModeMax as u32 => Ok(KickoutReason::ModeMax),
            _ => Err(()),
        }
    }
}

#[no_mangle]
pub fn kick_out(par: KickoutReason) {
    factory_reset();

    if par == KickoutReason::OutOfMesh {
        let pairing_addr = *get_flash_adr_pairing();
        let mut buff: [u8; 16] = [0; 16];
        buff[0..16].copy_from_slice(&get_pair_config_mesh_ltk()[0..16]);
        flash_write_page(pairing_addr + 48, 16, buff.as_mut_ptr());

        let mut buff: [u8; 16] = [0; 16];
        let len = min(MESH_PWD.len(), buff.len());
        buff[0..len].copy_from_slice(&MESH_PWD.as_bytes()[0..len]);
        _encode_password(buff.as_mut_ptr());
        flash_write_page(pairing_addr + 32, 16, buff.as_mut_ptr());

        let mut buff: [u8; 16] = [0; 16];
        let len = min(OUT_OF_MESH.len(), buff.len());
        buff[0..len].copy_from_slice(&OUT_OF_MESH.as_bytes()[0..len]);
        flash_write_page(pairing_addr + 16, 16, buff.as_mut_ptr());

        let mut buff: [u8; 16] = [0; 16];
        buff[0] = PAIR_VALID_FLAG;
        if *get_pair_config_pwd_encode_enable() != 0 {
            buff[15] = PAIR_VALID_FLAG;
        }

        if *get_mesh_pair_enable() {
            set_get_mac_en(true);
            buff[1] = 1;
        }
        flash_write_page(pairing_addr, 16, buff.as_mut_ptr());
    }

    app().ota_manager.rf_led_ota_ok();
}
