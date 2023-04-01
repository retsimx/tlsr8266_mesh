use crate::sdk::mcu::analog::analog_write;
use crate::sdk::mcu::clock::sleep_us;
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use crate::sdk::mcu::register::write_reg8;
use core::panic::PanicInfo;
use crate::blinken;
use crate::sdk::mcu::watchdog::wd_clear;

extern "C" {
    fn memcmp(s1: *const u8, s2: *const u8, count: usize) -> i32;
}

#[no_mangle]
fn _critical_section_1_0_acquire() -> u8 {
    irq_disable()
}

#[no_mangle]
fn _critical_section_1_0_release(state: u8) {
    unsafe { irq_restore(state); }
}

#[no_mangle] // required by light_ll
fn bcmp(s1: *const u8, s2: *const u8, count: usize) -> i32 {
    unsafe {
        return memcmp(s1, s2, count);
    }
}

pub const TCMD_UNDER_RD: u8 = 0x80;
pub const TCMD_UNDER_WR: u8 = 0x40;
pub const TCMD_UNDER_BOTH: u8 = 0xc0;
pub const TCMD_MASK: u8 = 0x3f;

pub const TCMD_WRITE: u8 = 0x3;
pub const TCMD_WAIT: u8 = 0x7;
pub const TCMD_WAREG: u8 = 0x8;

#[repr(C, packed)]
pub struct TBLCMDSET {
    pub adr: u16,
    pub dat: u8,
    pub cmd: u8,
}

#[no_mangle] // required by light_ll
pub fn LoadTblCmdSet(pt: *const TBLCMDSET, size: u32) -> u32 {
    let mut l = 0;

    while l < size {
        let ptr = unsafe { &(*pt.offset(l as isize)) };
        let cadr: u32 = 0x800000 | ptr.adr as u32;
        let cdat: u8 = ptr.dat;
        let mut ccmd: u8 = ptr.cmd;
        let cvld: u8 = ccmd & TCMD_UNDER_WR;
        ccmd &= TCMD_MASK;
        if cvld != 0 {
            if ccmd == TCMD_WRITE {
                write_reg8(cadr, cdat);
            } else if ccmd == TCMD_WAREG {
                analog_write(cadr as u8, cdat);
            } else if ccmd == TCMD_WAIT {
                sleep_us((ptr.adr as u32) * 256 + cdat as u32);
            }
        }
        l += 1;
    }
    return size;
}

#[panic_handler]
#[no_mangle]
pub fn panic(_info: &PanicInfo) -> ! {
    irq_disable();

    loop {
        blinken();
    }
}