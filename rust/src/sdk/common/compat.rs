use crate::sdk::mcu::analog::analog_write__attribute_ram_code;
use crate::sdk::mcu::clock::sleep_us;
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use crate::sdk::mcu::register::write_reg8;

extern "C" {
    fn memcmp(s1: *const u8, s2: *const u8, count: usize) -> i32;
}

static mut critical_section_state: u8 = 0;

#[no_mangle]
fn _critical_section_1_0_acquire() {
    unsafe { critical_section_state = irq_disable(); }
}

#[no_mangle]
fn _critical_section_1_0_release() {
    unsafe { irq_restore(critical_section_state); }
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
fn __muldi3(a: u64, b: u64) -> u64 {
    // extract 16 bits out of each
    let A: [u16; 4] = [
        a as u16, (a >> 16) as u16, (a >> 32) as u16, (a >> 48) as u16
    ];
    let B: [u16; 4] = [
        b as u16, (b >> 16) as u16, (b >> 32) as u16, (b >> 48) as u16
    ];

    // store results in a 4x5 table, rows by columns. 1 extra col for overflow logic
    let mut F: [[u16; 5]; 4] = [[0; 5]; 4];

    // start multiplying it out, the long way
    for row in 0..4 {
        for col in row..4 {
            // multiply, store remainder in next cell
            let mul_res = mul_16(A[col - row], B[row], &mut F[row][col + 1]);

            // add to current cell (= remainder from previous cell), keep track of remainder
            let mut rem = 0;
            F[row][col] = sum_16(F[row][col], mul_res, &mut rem);

            // add sum remainder to mul remainder to next cell, remainder now should always be 0
            F[row][col + 1] = sum_16(F[row][col + 1], rem, &mut rem);

            assert_eq!(rem, 0);
        }
    }

    // now sum the intermediate values into the result
    let mut R: [u16; 5] = [0; 5]; // 1 extra to simplify logic
    for col in 0..4 {
        R[col] = sum_5_16(R[col], F[0][col], F[1][col], F[2][col], F[3][col], &mut R[col + 1]);
    }

    // convert back into a 64-bit number
    R[0] as u64 + ((R[1] as u64) << 16) + ((R[2] as u64) << 32) + ((R[3] as u64) << 48)
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
                analog_write__attribute_ram_code(cadr as u8, cdat);
            } else if ccmd == TCMD_WAIT {
                sleep_us((ptr.adr as u32) * 256 + cdat as u32);
            }
        }
        l += 1;
    }
    return size;
}

#[cfg(test)]
mod tests {
    use crate::sdk::common::compat::__muldi3;

    #[test]
    fn test_muldi3() {
        let expected: u64 = 0x123456 * 0x123456789;
        assert_eq!(__muldi3(0x123456, 0x123456789), expected);
        assert_eq!(__muldi3(0x123456789, 0x123456), expected);

        let expected: u64 = 0x9876543210 * 0x50;
        assert_eq!(__muldi3(0x9876543210, 0x50), expected);
        assert_eq!(__muldi3(0x50, 0x9876543210), expected);
    }
}