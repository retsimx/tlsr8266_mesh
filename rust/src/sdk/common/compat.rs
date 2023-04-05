use core::cmp::min;
use core::{fmt, slice};
use core::fmt::Write;
use crate::sdk::mcu::analog::analog_write;
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed, sleep_us};
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use crate::sdk::mcu::register::write_reg8;
use core::panic::PanicInfo;
use core::ptr::{addr_of, addr_of_mut};
use crate::{app, blinken};
use crate::config::{get_flash_adr_panic_info, PANIC_VALID_FLAG};
use crate::embassy::yield_now::yield_now;
use crate::sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};
use crate::sdk::drivers::uart::{UART_DATA_LEN, uart_data_t};
use crate::sdk::light::{_light_sw_reboot, LGT_PANIC_MSG};
use crate::sdk::mcu::watchdog::wd_clear;
use crate::uart_manager::UartMsg;

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

/// A byte stream to the host (e.g., host's stdout or stderr).
#[derive(Clone, Copy)]
pub struct HostStream {
    pub length: usize,
    pub out_buffer: [u8; 256]
}

impl HostStream {
    /// Attempts to write an entire `buffer` into this sink
    pub fn write_all(&mut self, buffer: &[u8]) -> Result<(), ()> {
        self.out_buffer[self.length..self.length + buffer.len()].copy_from_slice(buffer);
        self.length += buffer.len();
        Ok(())
    }

    pub fn send(&self) {
        let mut buffer = unsafe { slice::from_raw_parts(self.out_buffer.as_ptr(), self.length) };
        while !buffer.is_empty() {
            let len = min(UART_DATA_LEN - 3, buffer.len());

            let mut msg = uart_data_t {
                len: UART_DATA_LEN as u32, data: [0; UART_DATA_LEN]
            };

            msg.data[2] = UartMsg::PanicMessage as u8;
            for i in 0..len {
                msg.data[3 + i] = buffer[i];
            }

            app().uart_manager.driver.uart_send(&msg);

            buffer = unsafe { slice::from_raw_parts(buffer.as_ptr().offset(len as isize), buffer.len() - len) }
        }
    }
}

impl Write for HostStream {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.write_all(s.as_bytes()).map_err(|_| fmt::Error)
    }
}

fn write_panic_info(info: &HostStream) {
    let panic_addr = *get_flash_adr_panic_info();

    // Clear the panic info sector for writing just in case
    flash_erase_sector(panic_addr);

    // Panic section looks as follows
    // valid_flag: u8   + 0
    // length: u16      + 1
    // data: [u8; 256]  + 3

    // Write the length first
    let length = info.length as u16;
    flash_write_page(panic_addr + 1, 2, addr_of!(length) as *const u8);

    // Then write the panic message
    flash_write_page(panic_addr + 3, length as u32, info.out_buffer.as_ptr());

    // Finally write the panic valid flag last
    let flag = PANIC_VALID_FLAG;
    flash_write_page(panic_addr, 1, addr_of!(flag));
}


async fn delay(ms: u32) {
    let t_timeout = clock_time();
    while !clock_time_exceed(t_timeout, ms*1000) {
        yield_now().await;
    }
}


pub async fn check_panic_info() {
    let panic_addr = *get_flash_adr_panic_info();

    let mut flag: u8 = 0;
    flash_read_page(panic_addr, 1, addr_of_mut!(flag));

    // If the panic info flag isn't set, there's nothing to do
    if flag != PANIC_VALID_FLAG {
        return
    }

    // There is panic info, send it in to the mesh

    // Read the length
    let mut length: u16 = 0;
    flash_read_page(panic_addr + 1, 2, addr_of_mut!(length) as *mut u8);

    // Read the message
    let mut message: [u8; 256] = [0; 256];
    flash_read_page(panic_addr + 3, length as u32, message.as_mut_ptr());

    let mut buffer = unsafe { slice::from_raw_parts(message.as_ptr(), length as usize) };
    while !buffer.is_empty() {
        // Wait a moment to send the next message
        delay(100).await;

        let len = min(10, buffer.len());

        let mut data = [0 as u8; 13];
        data[0] = LGT_PANIC_MSG;
        data[3..3+len].copy_from_slice(&buffer[0..len]);

        app().mesh_manager.send_mesh_message(&data, 0xffff);

        buffer = unsafe { slice::from_raw_parts(buffer.as_ptr().offset(len as isize), buffer.len() - len) }
    }

    // Wait a moment to send the next message
    delay(100).await;

    // Send an empty panic message to indicate the end of the message
    let mut data = [0 as u8; 13];
    data[0] = LGT_PANIC_MSG;

    app().mesh_manager.send_mesh_message(&data, 0xffff);

    // Finally clear the panic info
    flash_erase_sector(panic_addr);
}


#[panic_handler]
#[no_mangle]
pub fn panic(info: &PanicInfo) -> ! {
    irq_disable();

    let mut stream = HostStream { length: 0, out_buffer: [0; 256] };

    write!(stream, "{}", info).ok();
    stream.send();
    write_panic_info(&stream);

    _light_sw_reboot();

    loop {}
}