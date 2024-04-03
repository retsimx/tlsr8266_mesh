use core::{fmt, slice};
use core::cmp::min;
use core::fmt::Write;
use core::panic::PanicInfo;
use core::ptr::{addr_of, addr_of_mut};

use critical_section::RawRestoreState;

use crate::app;
use crate::config::{FLASH_ADR_PANIC_INFO, PANIC_VALID_FLAG};
use crate::embassy::yield_now::yield_now;
use crate::sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};
use crate::sdk::drivers::uart::{UART_DATA_LEN, UartDataT};
use crate::sdk::light::LGT_PANIC_MSG;
use crate::sdk::mcu::analog::analog_write;
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed, sleep_us};
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use crate::sdk::mcu::register::write_reg8;
use crate::sdk::pm::light_sw_reboot;
use crate::uart_manager::UartMsg;

struct TlsrCriticalSection;
critical_section::set_impl!(TlsrCriticalSection);

unsafe impl critical_section::Impl for TlsrCriticalSection {
    unsafe fn acquire() -> RawRestoreState {
        irq_disable()
    }

    unsafe fn release(state: RawRestoreState) {
        irq_restore(state)
    }
}

pub static mut uartstream: UartStream<128> = UartStream::new();

#[macro_export]
macro_rules! uprintln {
    ( $($arg:tt)* ) => {
        {
            use core::fmt::Write;
            use crate::sdk::common::compat::uartstream;

            critical_section::with(|_| {
                #[allow(unused_unsafe)]
                unsafe {
                    uartstream.length = 0;
                    core::writeln!(uartstream, $($arg)*).unwrap();
                    uartstream.send(false, true);
                }
            });
        }
    };
}

#[macro_export]
macro_rules! uprintln_fast {
    ( $($arg:tt)* ) => {
        {
            use core::fmt::Write;
            use crate::sdk::common::compat::uartstream;

            critical_section::with(|_| {
                #[allow(unused_unsafe)]
                unsafe {
                    uartstream.length = 0;
                    core::writeln!(uartstream, $($arg)*).unwrap();
                    uartstream.send(false, false);
                }
            });
        }
    };
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

pub fn load_tbl_cmd_set(pt: *const TBLCMDSET, size: u32) -> u32 {
    let mut l = 0;

    while l < size {
        let ptr = unsafe { &(*pt.offset(l as isize)) };
        let cadr: u32 = 0x800000 | ptr.adr as u32;
        let cdat: u8 = ptr.dat;
        let mut ccmd: u8 = ptr.cmd;
        let cvld: u8 = ccmd & TCMD_UNDER_WR;
        ccmd &= TCMD_MASK;
        if cvld != 0 {
            match ccmd {
                TCMD_WRITE => write_reg8(cadr, cdat),
                TCMD_WAREG => analog_write(cadr as u8, cdat),
                TCMD_WAIT => sleep_us((ptr.adr as u32) * 256 + cdat as u32),
                _ => ()
            }
        }
        l += 1;
    }
    return size;
}

/// A byte stream to the host (e.g., host's stdout or stderr).
#[derive(Clone, Copy)]
pub struct UartStream<const size: usize> {
    pub length: usize,
    pub out_buffer: [u8; size]
}

impl<const size: usize> UartStream<size> {
    pub const fn new() -> UartStream<size> {
        UartStream::<size> { length: 0, out_buffer: [0; size] }
    }

    /// Attempts to write an entire `buffer` into this sink
    pub fn write_all(&mut self, buffer: &[u8]) -> Result<(), ()> {
        self.out_buffer[self.length..self.length + buffer.len()].copy_from_slice(buffer);
        self.length += buffer.len();
        Ok(())
    }

    pub fn send(&self, is_panic: bool, doasync: bool) {
        let mut buffer = unsafe { slice::from_raw_parts(self.out_buffer.as_ptr(), self.length) };
        while !buffer.is_empty() {
            let len = min(UART_DATA_LEN - 3, buffer.len());

            let mut msg = UartDataT {
                len: UART_DATA_LEN as u32, data: [0; UART_DATA_LEN]
            };

            msg.data[2] = if is_panic {UartMsg::PanicMessage} else {UartMsg::PrintMessage} as u8;
            for i in 0..len {
                msg.data[3 + i] = buffer[i];
            }

            if doasync {
                app().uart_manager.send_message(&msg);
            } else {
                app().uart_manager.driver.uart_send(&msg);
            }

            buffer = unsafe { slice::from_raw_parts(buffer.as_ptr().offset(len as isize), buffer.len() - len) }
        }
    }
}

impl<const size: usize> Write for UartStream<size> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.write_all(s.as_bytes()).map_err(|_| fmt::Error)
    }
}

fn write_panic_info<const size: usize>(info: &UartStream<size>) {
    let panic_addr = FLASH_ADR_PANIC_INFO;

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
    let panic_addr = FLASH_ADR_PANIC_INFO;

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

        let mut data = [0u8; 13];
        data[0] = LGT_PANIC_MSG;
        data[3..3+len].copy_from_slice(&buffer[0..len]);

        app().mesh_manager.send_mesh_message(&data, 0xffff, 3, false);

        buffer = unsafe { slice::from_raw_parts(buffer.as_ptr().offset(len as isize), buffer.len() - len) }
    }

    // Wait a moment to send the next message
    delay(100).await;

    // Send an empty panic message to indicate the end of the message
    let mut data = [0u8; 13];
    data[0] = LGT_PANIC_MSG;

    app().mesh_manager.send_mesh_message(&data, 0xffff, 3, false);

    // Finally clear the panic info
    flash_erase_sector(panic_addr);
}

#[cfg(not(test))]
#[panic_handler]
pub fn panic(info: &PanicInfo) -> ! {
    irq_disable();

    let mut stream = UartStream::<256>::new();

    let _ = write!(stream, "{}", info);
    stream.send(true, false);
    write_panic_info(&stream);

    light_sw_reboot();

    loop {}
}

pub fn array4_to_int(data: &[u8]) -> u32 {
    data[0] as u32 | ((data[1] as u32) << 8) | ((data[2] as u32) << 16) | ((data[3] as u32) << 24)
}