use core::cmp::min;
use core::fmt::Write;
use core::panic::PanicInfo;
use core::ptr::{addr_of, addr_of_mut};
use core::{fmt, slice};

use crate::app;
use crate::config::{FLASH_ADR_PANIC_INFO, PANIC_VALID_FLAG};
use crate::embassy::yield_now::yield_now;
use crate::sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};
use crate::sdk::drivers::uart::{UartData, UART_DATA_LEN};
use crate::sdk::light::LGT_PANIC_MSG;
use crate::sdk::mcu::analog::analog_write;
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed, sleep_us};
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use crate::sdk::mcu::register::write_reg8;
use crate::sdk::pm::light_sw_reboot;
use crate::uart_manager::UartMsg;
use critical_section::RawRestoreState;
use heapless::Vec;

#[cfg(test)]
use mry::mry;

// UART packet structure constants
// Packet layout: [counter(1)][cmd_type(1)][cmd_code(1)][payload(39)][crc(2)] = 44 bytes total
const UART_PAYLOAD_OFFSET: usize = 3; // Payload starts at index 3
const UART_CRC_OFFSET: usize = 42; // CRC starts at index 42
const UART_PAYLOAD_MAX_SIZE: usize = UART_CRC_OFFSET - UART_PAYLOAD_OFFSET; // 39 bytes

struct TlsrCriticalSection;
critical_section::set_impl!(TlsrCriticalSection);

#[coverage(off)]
unsafe impl critical_section::Impl for TlsrCriticalSection {
    unsafe fn acquire() -> RawRestoreState {
        if !cfg!(test) {
            return irq_disable();
        };

        0
    }

    unsafe fn release(state: RawRestoreState) {
        if !cfg!(test) {
            irq_restore(state)
        }
    }
}

// Define a function to get a new UartStream rather than using a static mutable
#[inline]
pub fn get_uart_stream() -> UartStream<128> {
    UartStream::new()
}

// Helper function to print and send data to the uart
#[inline]
pub fn print_to_uart<F>(doasync: bool, f: F)
where
    F: FnOnce(&mut UartStream<128>) -> fmt::Result,
{
    critical_section::with(|_| {
        let mut stream = get_uart_stream();
        let _ = f(&mut stream);
        stream.send(false, doasync);
    });
}

#[macro_export]
macro_rules! uprintln {
    ( $($arg:tt)* ) => {
        {
            use core::fmt::Write;
            use crate::sdk::common::compat::print_to_uart;

            print_to_uart(true, |stream| {
                core::writeln!(stream, $($arg)*)
            });
        }
    };
}

#[macro_export]
macro_rules! uprintln_fast {
    ( $($arg:tt)* ) => {
        {
            use crate::sdk::common::compat::print_to_uart;

            print_to_uart(false, |stream| {
                core::writeln!(stream, $($arg)*)
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
#[derive(Clone)]
pub struct TBLCMDSET {
    pub adr: u16,
    pub dat: u8,
    pub cmd: u8,
}

#[cfg_attr(test, mry(skip_args(TBLCMDSET)))]
pub fn load_tbl_cmd_set(cmds: &[TBLCMDSET]) -> u32 {
    for cmd in cmds {
        let adr: u32 = cmd.adr as u32; // write_reg8 adds 0x800000 itself
        let dat: u8 = cmd.dat;
        let mut ccmd: u8 = cmd.cmd;
        let cvld: u8 = ccmd & TCMD_UNDER_WR;
        ccmd &= TCMD_MASK;

        if cvld != 0 {
            match ccmd {
                TCMD_WRITE => write_reg8(adr, dat),
                TCMD_WAREG => analog_write(adr as u8, dat),
                TCMD_WAIT => sleep_us((cmd.adr as u32) * 256 + dat as u32),
                _ => (),
            }
        }
    }

    cmds.len() as u32
}

/// A byte stream to the host (e.g., host's stdout or stderr).
#[derive(Clone, Copy)]
pub struct UartStream<const size: usize> {
    pub length: usize,
    pub out_buffer: [u8; size],
}

impl<const size: usize> UartStream<size> {
    pub const fn new() -> UartStream<size> {
        UartStream::<size> {
            length: 0,
            out_buffer: [0; size],
        }
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
            // Use the correct payload size to avoid overwriting CRC
            let len = min(UART_PAYLOAD_MAX_SIZE, buffer.len());

            let mut msg = UartData {
                len: UART_DATA_LEN as u32,
                data: [0; UART_DATA_LEN],
            };

            msg.data[2] = if is_panic {
                UartMsg::PanicMessage
            } else {
                UartMsg::PrintMessage
            } as u8;
            for i in 0..len {
                msg.data[UART_PAYLOAD_OFFSET + i] = buffer[i];
            }

            if doasync {
                app().uart_manager.send_message(&msg);
            } else {
                app().uart_manager.driver.send(&msg);
            }

            buffer = unsafe {
                slice::from_raw_parts(buffer.as_ptr().offset(len as isize), buffer.len() - len)
            }
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
    while !clock_time_exceed(t_timeout, ms * 1000) {
        yield_now().await;
    }
}

pub async fn check_panic_info() {
    let panic_addr = FLASH_ADR_PANIC_INFO;

    let mut flag: u8 = 0;
    flash_read_page(panic_addr, 1, addr_of_mut!(flag));

    // If the panic info flag isn't set, there's nothing to do
    if flag != PANIC_VALID_FLAG {
        return;
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
        data[3..3 + len].copy_from_slice(&buffer[0..len]);

        app()
            .mesh_manager
            .send_mesh_message(&Vec::from_slice(&data).unwrap(), 0xffff, 3, false);

        buffer = unsafe {
            slice::from_raw_parts(buffer.as_ptr().offset(len as isize), buffer.len() - len)
        }
    }

    // Wait a moment to send the next message
    delay(100).await;

    // Send an empty panic message to indicate the end of the message
    let mut data = [0u8; 13];
    data[0] = LGT_PANIC_MSG;

    app()
        .mesh_manager
        .send_mesh_message(&Vec::from_slice(&data).unwrap(), 0xffff, 3, false);

    // Finally clear the panic info
    flash_erase_sector(panic_addr);
}

#[cfg(target_os = "none")]
#[panic_handler]
pub fn panic(info: &PanicInfo) -> ! {
    critical_section::with(|_| {
        let mut stream = UartStream::<256>::new();

        let _ = write!(stream, "{}", info);
        stream.send(true, false);
        write_panic_info(&stream);

        light_sw_reboot();
    });

    loop {}
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::embassy::yield_now::mock_yield_now;
    use crate::sdk::drivers::flash::{
        mock_flash_erase_sector, mock_flash_read_page, mock_flash_write_page,
    };
    use crate::sdk::drivers::uart::UartDriver;
    use crate::sdk::mcu::analog::*;
    use crate::sdk::mcu::clock::*;
    use crate::sdk::mcu::clock::{mock_clock_time, mock_clock_time_exceed};
    use crate::sdk::mcu::register::*;
    use crate::uart_manager::UartManager;
    use crate::{app_mocker, mock_app_mocker, App};
    use mry::{send_wrapper::SendWrapper, Any};
    use std::sync::atomic::{AtomicU8, Ordering};

    #[test]
    fn test_load_tbl_cmd_set_empty() {
        let result = load_tbl_cmd_set(&[]);
        assert_eq!(result, 0);
    }

    #[test]
    #[mry::lock(write_reg8)]
    fn test_load_tbl_cmd_set_write_cmd() {
        // Create a test command set with TCMD_WRITE
        let cmd_set = [TBLCMDSET {
            adr: 0x1234,
            dat: 0x56,
            cmd: TCMD_WRITE | TCMD_UNDER_WR,
        }];

        // Mock write_reg8 function
        mock_write_reg8(0x1234, 0x56).returns(());

        let result = load_tbl_cmd_set(&cmd_set);

        // Assert the function returns the size of the command set
        assert_eq!(result, 1);

        // Verify that write_reg8 was called with the expected arguments
        mock_write_reg8(0x1234, 0x56).assert_called(1);
    }

    #[test]
    #[mry::lock(analog_write)]
    fn test_load_tbl_cmd_set_wareg_cmd() {
        // Create a test command set with TCMD_WAREG
        let cmd_set = [TBLCMDSET {
            adr: 0x78,
            dat: 0x9A,
            cmd: TCMD_WAREG | TCMD_UNDER_WR,
        }];

        // Mock analog_write function
        mock_analog_write(0x78, 0x9A).returns(());

        let result = load_tbl_cmd_set(&cmd_set);

        // Assert the function returns the size of the command set
        assert_eq!(result, 1);

        // Verify that analog_write was called with the expected arguments
        mock_analog_write(0x78, 0x9A).assert_called(1);
    }

    #[test]
    #[mry::lock(sleep_us)]
    fn test_load_tbl_cmd_set_wait_cmd() {
        // Create a test command set with TCMD_WAIT
        let cmd_set = [TBLCMDSET {
            adr: 0x02, // 0x02 * 256 = 512
            dat: 0x34, // + 0x34 = 52 = 564
            cmd: TCMD_WAIT | TCMD_UNDER_WR,
        }];

        // Mock sleep_us function
        mock_sleep_us(564).returns(());

        let result = load_tbl_cmd_set(&cmd_set);

        // Assert the function returns the size of the command set
        assert_eq!(result, 1);

        // Verify that sleep_us was called with the expected arguments
        mock_sleep_us(564).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg8)]
    #[mry::lock(analog_write)]
    #[mry::lock(sleep_us)]
    fn test_load_tbl_cmd_set_unsupported_cmd() {
        // Create a test command set with an unsupported command value
        let cmd_set = [TBLCMDSET {
            adr: 0x1234,
            dat: 0x56,
            cmd: 0x0F | TCMD_UNDER_WR, // Unsupported command
        }];

        let result = load_tbl_cmd_set(&cmd_set);

        // Assert the function returns the size of the command set
        assert_eq!(result, 1);

        // No mocked functions should be called
        mock_write_reg8(Any, Any).assert_called(0);
        mock_analog_write(Any, Any).assert_called(0);
        mock_sleep_us(Any).assert_called(0);
    }

    #[test]
    #[mry::lock(write_reg8)]
    #[mry::lock(analog_write)]
    #[mry::lock(sleep_us)]
    fn test_load_tbl_cmd_set_invalid_flag() {
        // Create a test command set with invalid cmd flag (no TCMD_UNDER_WR)
        let cmd_set = [TBLCMDSET {
            adr: 0x1234,
            dat: 0x56,
            cmd: TCMD_WRITE, // Missing TCMD_UNDER_WR flag
        }];

        let result = load_tbl_cmd_set(&cmd_set);

        // Assert the function returns the size of the command set
        assert_eq!(result, 1);

        // No mocked functions should be called
        mock_write_reg8(Any, Any).assert_called(0);
        mock_analog_write(Any, Any).assert_called(0);
        mock_sleep_us(Any).assert_called(0);
    }

    #[test]
    #[mry::lock(write_reg8)]
    #[mry::lock(analog_write)]
    #[mry::lock(sleep_us)]
    fn test_load_tbl_cmd_set_multiple_commands() {
        // Create a test command set with multiple commands
        let cmd_set = [
            TBLCMDSET {
                adr: 0x1234,
                dat: 0x56,
                cmd: TCMD_WRITE | TCMD_UNDER_WR,
            },
            TBLCMDSET {
                adr: 0x78,
                dat: 0x9A,
                cmd: TCMD_WAREG | TCMD_UNDER_WR,
            },
            TBLCMDSET {
                adr: 0x02,
                dat: 0x34,
                cmd: TCMD_WAIT | TCMD_UNDER_WR,
            },
            TBLCMDSET {
                adr: 0xABCD,
                dat: 0xEF,
                cmd: 0xFF, // Invalid cmd - should be ignored
            },
        ];

        // Mock all required functions
        mock_write_reg8(0x1234, 0x56).returns(());
        mock_analog_write(0x78, 0x9A).returns(());
        mock_sleep_us(564).returns(());

        let result = load_tbl_cmd_set(&cmd_set);

        // Assert the function returns the size of the command set
        assert_eq!(result, 4);

        // Verify all functions were called with the expected arguments
        mock_write_reg8(0x1234, 0x56).assert_called(1);
        mock_analog_write(0x78, 0x9A).assert_called(1);
        mock_sleep_us(564).assert_called(1);
    }

    #[test]
    #[mry::lock(app_mocker)]
    fn test_uart_stream_send_synchronous() {
        // Create a test stream with some data
        let mut stream = UartStream::<128>::new();
        let test_data = b"Hello UART";
        let _ = stream.write_all(test_data);

        // Create a UartDriver (mry makes it mockable)
        let mut driver = UartDriver::default();
        // Mock the send method to return true
        driver.mock_send(Any).returns(());

        // Create a UartManager with the mocked driver
        let mut uart_manager = UartManager::default();
        uart_manager.driver = driver;

        // Set up mock app with the uart_manager
        let mut app = crate::App::default();
        app.uart_manager = uart_manager;
        mock_app_mocker().returns(&mut app);

        // Call send with doasync = false (synchronous) to cover line 172
        stream.send(false, false);

        // Verify that the synchronous send was called
        app.uart_manager.driver.mock_send(Any).assert_called(1);
    }

    #[test]
    #[mry::lock(app_mocker)]
    fn test_uart_stream_send_panic_mode() {
        // Create a test stream with some data
        let mut stream = UartStream::<128>::new();
        let test_data = b"Panic message!";
        let _ = stream.write_all(test_data);

        // Create a UartDriver (mry makes it mockable)
        let mut driver = UartDriver::default();
        // Mock the send method to return true
        driver.mock_send(Any).returns(());

        // Create a UartManager with the mocked driver
        let mut uart_manager = UartManager::default();
        uart_manager.driver = driver;

        // Set up mock app with the uart_manager
        let mut app = crate::App::default();
        app.uart_manager = uart_manager;
        mock_app_mocker().returns(&mut app);

        // Call send with is_panic = true and doasync = false (synchronous panic mode) to cover line 170
        stream.send(true, false);

        // Verify that the synchronous send was called
        app.uart_manager.driver.mock_send(Any).assert_called(1);
    }

    #[test]
    #[mry::lock(flash_erase_sector, flash_write_page)]
    fn test_write_panic_info() {
        // Create a test stream with some panic data
        let mut stream = UartStream::<128>::new();
        let test_data = b"Panic occurred!";
        let _ = stream.write_all(test_data);

        // Mock flash operations
        mock_flash_erase_sector(FLASH_ADR_PANIC_INFO).returns(());
        mock_flash_write_page(FLASH_ADR_PANIC_INFO + 1, 2, Any).returns(());
        mock_flash_write_page(FLASH_ADR_PANIC_INFO + 3, test_data.len() as u32, Any).returns(());
        mock_flash_write_page(FLASH_ADR_PANIC_INFO, 1, Any).returns(());

        // Call write_panic_info
        write_panic_info(&stream);

        // Verify flash operations were called correctly
        mock_flash_erase_sector(FLASH_ADR_PANIC_INFO).assert_called(1);
        mock_flash_write_page(FLASH_ADR_PANIC_INFO + 1, 2, Any).assert_called(1);
        mock_flash_write_page(FLASH_ADR_PANIC_INFO + 3, test_data.len() as u32, Any)
            .assert_called(1);
        mock_flash_write_page(FLASH_ADR_PANIC_INFO, 1, Any).assert_called(1);
    }

    #[test]
    #[mry::lock(flash_read_page, flash_erase_sector, app_mocker)]
    fn test_check_panic_info_no_valid_flag() {
        // Mock reading invalid flag
        mock_flash_read_page(FLASH_ADR_PANIC_INFO, 1, Any).returns_with(
            |_, _, buf: SendWrapper<*mut u8>| {
                unsafe {
                    core::ptr::write_bytes(*buf, 0x00, 1); // Write invalid flag
                }
            },
        );

        // Create mock app (needed for the app() call in the function, but shouldn't be reached)
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);

        // Run check_panic_info (should return early)
        let future = check_panic_info();
        futures::executor::block_on(future);

        // Verify only the flag read was called
        mock_flash_read_page(FLASH_ADR_PANIC_INFO, 1, Any).assert_called(1);
        mock_flash_erase_sector(Any).assert_called(0); // Should not clear panic info
    }

    #[test]
    #[mry::lock(
        flash_read_page,
        flash_erase_sector,
        app_mocker,
        clock_time,
        clock_time_exceed,
        yield_now
    )]
    fn test_check_panic_info_with_valid_data() {
        let test_message = b"Test panic message";
        let message_len = test_message.len() as u16;

        // Mock reading valid flag
        mock_flash_read_page(FLASH_ADR_PANIC_INFO, 1, Any).returns_with(
            |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::write_bytes(*buf, PANIC_VALID_FLAG, 1);
            },
        );

        // Mock reading message length
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 1, 2, Any).returns_with(
            move |_, _, buf: SendWrapper<*mut u8>| unsafe {
                let len_bytes = message_len.to_le_bytes();
                core::ptr::copy_nonoverlapping(len_bytes.as_ptr(), *buf, 2);
            },
        );

        // Mock reading message data
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 3, message_len as u32, Any).returns_with(
            move |_, _, buf: SendWrapper<*mut u8>| unsafe {
                core::ptr::copy_nonoverlapping(test_message.as_ptr(), *buf, test_message.len());
            },
        );

        // Create mock app with mocked mesh_manager
        let mut app = App::default();
        app.mesh_manager
            .mock_send_mesh_message(Any, Any, Any, Any)
            .returns([1, 2, 3]);
        mock_app_mocker().returns(&mut app);

        // Mock delay function components
        mock_clock_time().returns(1000); // Initial time
        mock_clock_time_exceed(1000, 100 * 1000).returns(true); // Make delays complete immediately
        mock_yield_now().returns(());

        // Mock final erase
        mock_flash_erase_sector(FLASH_ADR_PANIC_INFO).returns(());

        // Run check_panic_info
        let future = check_panic_info();
        futures::executor::block_on(future);

        // Verify all operations were called
        mock_flash_read_page(FLASH_ADR_PANIC_INFO, 1, Any).assert_called(1); // Flag read
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 1, 2, Any).assert_called(1); // Length read
        mock_flash_read_page(FLASH_ADR_PANIC_INFO + 3, message_len as u32, Any).assert_called(1); // Data read
        mock_flash_erase_sector(FLASH_ADR_PANIC_INFO).assert_called(1); // Final erase

        // Verify mesh messages were sent (3: two data chunks + one empty terminator)
        app.mesh_manager
            .mock_send_mesh_message(Any, Any, Any, Any)
            .assert_called(3);
    }

    #[test]
    #[mry::lock(clock_time, clock_time_exceed, yield_now)]
    fn test_delay_function_waits_until_timeout() {
        let start_time = 1000;
        let delay_ms = 50; // 50ms delay = 50,000 microseconds
        let expected_timeout = start_time + delay_ms * 1000;

        // Mock initial clock time
        mock_clock_time().returns(start_time);

        // Track how many times clock_time_exceed is called
        let exceed_calls = AtomicU8::new(0);

        // Mock clock_time_exceed to return false for first 3 calls, then true
        mock_clock_time_exceed(Any, Any).returns_with(move |_, _| {
            let count = exceed_calls.fetch_add(1, Ordering::SeqCst);
            count >= 3 // Return true on the 4th call (after 3 yields)
        });

        // Mock yield_now to do nothing
        mock_yield_now().returns(());

        // Run the delay function
        futures::executor::block_on(delay(delay_ms));

        // Verify clock_time was called once to get start time
        mock_clock_time().assert_called(1);

        // Verify clock_time_exceed was called 4 times (3 false + 1 true)
        mock_clock_time_exceed(Any, Any).assert_called(4);

        // Verify yield_now was called 3 times (once for each loop iteration before timeout)
        mock_yield_now().assert_called(3);
    }
}
