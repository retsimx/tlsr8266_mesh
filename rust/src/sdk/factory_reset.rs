use core::cmp::min;
use core::convert::TryFrom;

use crate::app;
use crate::BIT;
use crate::config::{FLASH_ADR_PAIRING, FLASH_ADR_RESET_CNT, MESH_PWD, OUT_OF_MESH, PAIR_VALID_FLAG};
use crate::sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};
use crate::sdk::mcu::clock::clock_time_exceed;
use crate::sdk::mcu::crypto::encode_password;
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use crate::sdk::pm::light_sw_reboot;
use crate::state::{*};

const SERIALS_CNT: u8 = 5; // must less than 7
const FACTORY_RESET_SERIALS: [u8; (SERIALS_CNT * 2) as usize] = [
    0, 3, // [0]:must 0
    0, 3, // [2]:must 0
    0, 3, // [4]:must 0
    3, 30, 3, 30,
];

pub const RESET_CNT_RECOUNT_FLAG: u8 = 0;
pub const RESET_FLAG: u8 = 0x80;

pub const FLASH_ADR_PAR_MAX: u32 = 0x80000;
pub const CFG_ADR_MAC_512K_FLASH: u32 = 0x76000;
pub const CFG_SECTOR_ADR_MAC_CODE: u32 = CFG_ADR_MAC_512K_FLASH;

fn reset_cnt_clean() {
    if ADR_RESET_CNT_IDX.get() < 3840 {
        return;
    }
    flash_erase_sector(FLASH_ADR_RESET_CNT);
    ADR_RESET_CNT_IDX.set(0);
}

fn write_reset_cnt(cnt: u8) {
    let data = [cnt];
    flash_write_page(
        FLASH_ADR_RESET_CNT + ADR_RESET_CNT_IDX.get(),
        1,
        data.as_ptr(),
    );
}

fn clear_reset_cnt() {
    write_reset_cnt(RESET_CNT_RECOUNT_FLAG);
}

fn reset_cnt_get_idx() {
    let pf = FLASH_ADR_RESET_CNT as *const u8;
    ADR_RESET_CNT_IDX.set(0);
    while ADR_RESET_CNT_IDX.get() < 4096 {
        let restcnt_bit = unsafe { *pf.offset(ADR_RESET_CNT_IDX.get() as isize) };
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
        ADR_RESET_CNT_IDX.inc();
    }

    reset_cnt_clean();
}

fn get_reset_cnt_bit() -> u8 {
    if ADR_RESET_CNT_IDX.get() < 0 {
        reset_cnt_clean();
        return 0;
    }

    let mut data = [0];
    flash_read_page(
        FLASH_ADR_RESET_CNT + ADR_RESET_CNT_IDX.get(),
        1,
        data.as_mut_ptr(),
    );
    RESET_CNT.set(data[0]);
    return RESET_CNT.get();
}

fn increase_reset_cnt() {
    let mut restcnt_bit = get_reset_cnt_bit();
    for i in 0..8 {
        if restcnt_bit & BIT!(i) != 0 {
            if i < 3 {
                RESET_CNT.set(i);
            } else if i < 5 {
                RESET_CNT.set(3);
            } else if i < 7 {
                RESET_CNT.set(4);
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
        light_sw_reboot();
    } else {
        increase_reset_cnt();
    }
}

pub fn factory_reset_cnt_check() {
    let mut increase_cnt = false;
    let mut clear_cnt = false;

    if CLEAR_ST.get() == 0 {
        return;
    }

    if CLEAR_ST.get() == 3 {
        CLEAR_ST.set(CLEAR_ST.get() - 1);
        RESET_CHECK_TIME.set(FACTORY_RESET_SERIALS[RESET_CNT.get() as usize * 2] as u32);
    }

    if CLEAR_ST.get() == 2 && clock_time_exceed(0, RESET_CHECK_TIME.get() * 1000 * 1000) {
        CLEAR_ST.set(CLEAR_ST.get() - 1);
       RESET_CHECK_TIME.set(FACTORY_RESET_SERIALS[RESET_CNT.get() as usize * 2 + 1] as u32);
        if RESET_CNT.get() == 3 || RESET_CNT.get() == 4 {
            increase_cnt = true;
        }
    }

    if CLEAR_ST.get() == 1 && clock_time_exceed(0, RESET_CHECK_TIME.get() * 1000 * 1000) {
        CLEAR_ST.set(0);
        clear_cnt = true;
    }

    if increase_cnt {
        increase_reset_cnt();
    }

    if clear_cnt {
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

pub fn kick_out(par: KickoutReason) {
    factory_reset();

    if par == KickoutReason::OutOfMesh {
        let pairing_addr = FLASH_ADR_PAIRING;
        let mut buff: [u8; 16] = *PAIR_CONFIG_MESH_LTK.lock().get_mut();
        flash_write_page(pairing_addr + 48, 16, buff.as_mut_ptr());

        let mut buff: [u8; 16] = [0; 16];
        let len = min(MESH_PWD.len(), buff.len());
        buff[0..len].copy_from_slice(&MESH_PWD.as_bytes()[0..len]);
        encode_password(&mut buff);
        flash_write_page(pairing_addr + 32, 16, buff.as_mut_ptr());

        let mut buff: [u8; 16] = [0; 16];
        let len = min(OUT_OF_MESH.len(), buff.len());
        buff[0..len].copy_from_slice(&OUT_OF_MESH.as_bytes()[0..len]);
        flash_write_page(pairing_addr + 16, 16, buff.as_mut_ptr());

        let mut buff: [u8; 16] = [0; 16];
        buff[0] = PAIR_VALID_FLAG;
        buff[15] = PAIR_VALID_FLAG;

        if MESH_PAIR_ENABLE.get() {
            GET_MAC_EN.set(true);
            buff[1] = 1;
        }
        flash_write_page(pairing_addr, 16, buff.as_mut_ptr());
    }

    app().ota_manager.rf_led_ota_ok();
}
