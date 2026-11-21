//! Pure MCU command-table executor: writes register/analog values and delays
//! according to a static `TBLCMDSET` table. It touches nothing but the register,
//! analog and clock leaves of the HAL.

use crate::sdk::mcu::analog::analog_write;
use crate::sdk::mcu::clock::sleep_us;
use crate::sdk::mcu::register::write_reg8;

#[cfg(test)]
use mry::mry;

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

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::sdk::mcu::analog::mock_analog_write;
    use crate::sdk::mcu::clock::mock_sleep_us;
    use crate::sdk::mcu::register::mock_write_reg8;
    use mry::Any;

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
}
