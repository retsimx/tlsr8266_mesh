use sdk::mcu::analog::analog_write__attribute_ram_code;
use sdk::mcu::clock::sleep_us;
use sdk::mcu::register::write_reg8;

extern "C" {
    fn memcmp(s1: *const u8, s2: *const u8, count: usize) -> i32;
}

#[no_mangle]
fn bcmp (s1: *const u8, s2: *const u8, count: usize) -> i32
{
    unsafe { return memcmp(s1, s2, count); }
}

pub const TCMD_UNDER_RD		: u8 = 0x80;
pub const TCMD_UNDER_WR		: u8 = 0x40;
pub const TCMD_UNDER_BOTH	: u8 = 	0xc0;
pub const TCMD_MASK			: u8 = 0x3f;

pub const TCMD_WRITE		: u8 = 	0x3;
pub const TCMD_WAIT			: u8 = 0x7;
pub const TCMD_WAREG		: u8 = 	0x8;

#[repr(C, packed)]
pub struct TBLCMDSET {
	pub adr: u16,
	pub dat: u8,
	pub cmd: u8
}

#[no_mangle]
unsafe fn LoadTblCmdSet (pt: *const TBLCMDSET, size: u32) -> u32 {
	let mut l= 0;

	while l < size {
        let ptr = &(*pt.offset(l as isize));
		let cadr: u32 = 0x800000 | ptr.adr as u32;
		let cdat: u8 = ptr.dat;
		let mut ccmd: u8 = ptr.cmd;
		let cvld: u8 = ccmd & TCMD_UNDER_WR;
		ccmd &= TCMD_MASK;
		if cvld != 0 {
			if ccmd == TCMD_WRITE {
				write_reg8 (cadr, cdat);
			}
			else if ccmd == TCMD_WAREG {
                analog_write__attribute_ram_code(cadr as u8, cdat);
			}
			else if ccmd == TCMD_WAIT {
                sleep_us((ptr.adr as u32) * 256 + cdat as u32);
			}
		}
		l += 1;
	}
	return size;
}