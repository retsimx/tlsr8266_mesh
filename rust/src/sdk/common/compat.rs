use crate::sdk::mcu::analog::analog_write;
use crate::sdk::mcu::clock::sleep_us;
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use crate::sdk::mcu::register::write_reg8;

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


// Stores a *new* value into the atomic integer if the *ptr* value is the same as the *current* value.
// The return value is a result indicating whether the new value was written and containing the
// previous value. On success this value is guaranteed to be equal to current.
#[no_mangle]
unsafe fn __atomic_compare_exchange_4(ptr: *mut u32, expected: *mut u32, desired: u32, success_order: u32, failure_order: u32) -> bool {
    return if *ptr == *expected {
        *ptr = desired;
        true
    } else {
        *expected = *ptr;
        false
    }
}

#[no_mangle]
unsafe fn __atomic_load_4(ptr: *const u32, model: u32) -> u32 {
    *ptr
}

#[no_mangle]
unsafe fn __atomic_store_4(ptr: *mut u32, val: u32, model: u32) {
    *ptr = val;
}

#[no_mangle]
unsafe fn __atomic_fetch_and_4(ptr: *mut u32, val: u32, model: u32) -> u32 {
    let tmp = *ptr;
    *ptr = tmp & val;
    tmp
}

#[no_mangle]
unsafe fn __atomic_fetch_or_4(ptr: *mut u32, val: u32, model: u32) -> u32 {
    let tmp = *ptr;
    *ptr = tmp | val;
    tmp
}

#[no_mangle]
unsafe fn __atomic_exchange_4(dest: *mut u32, val: u32, model: u32) -> u32 {
    let tmp = *dest;
    *dest = val;
    tmp
}

// use 32-bit math to multiply 16-bit numbers and get overflow
#[inline(always)]
fn mul_16(a: u16, b: u16, overflow: &mut u16) -> u16 {
    let res = a as u32 * b as u32;

    *overflow = (res >> 16) as u16;

    res as u16
}

#[inline(always)]
fn sum_16(a: u16, b: u16, overflow: &mut u16) -> u16 {
    let res = a as u32 + b as u32;

    *overflow = (res >> 16) as u16;

    res as u16
}

#[inline(always)]
fn sum_5_16(a: u16, b: u16, c: u16, d: u16, e: u16, overflow: &mut u16) -> u16 {
    let res = a as u32 + b as u32 + c as u32 + d as u32 + e as u32;

    *overflow = (res >> 16) as u16;

    res as u16
}

#[no_mangle]
fn abort() {
    loop {}
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

pub const XTOA_UPPER: u16 = 0x0200;
pub fn ultoa(mut val: u32, mut string: heapless::String<43>, mut base: u16) -> heapless::String<43> {
    let mut buffer: heapless::String<43> = heapless::String::new();

    let mut upper = false;

    if base & XTOA_UPPER != 0 {
        upper = true;
        base &= !XTOA_UPPER;
    }

    loop {
        let mut v = val % base as u32;
        val = val / base as u32;
        if v <= 9 {
            v += '0' as u32;
        } else {
            if (upper) {
                v += 'A' as u32 - 10;
            } else {
                v += 'a' as u32 - 10;
            }
        }
        buffer.push(v as u8 as char);

        if val == 0 {
            break;
        }
    }

    buffer = buffer.chars().rev().collect::<heapless::String<43>>();

    string.push_str(&buffer.as_str()).unwrap();

    string
}

use core::panic::PanicInfo;

#[panic_handler]
#[no_mangle]
pub fn panic(_info: &PanicInfo) -> ! {
    loop { }
}
