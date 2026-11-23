//! Pure MCU command-table executor: writes register/analog values and delays
//! according to a static `TBLCMDSET` table. It touches nothing but the register,
//! analog and clock leaves of the HAL.

use crate::mcu::analog::analog_write;
use crate::mcu::clock::sleep_us;
use tlsr8266_pac::write_reg8;

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
    use crate::mcu::analog::fake_analog::FakeAnalog;
    use crate::mcu::clock::fake_clock;
    use tlsr8266_pac::fake_mmio::{FakeMmio, RegWrite};
    use tlsr8266_pac::REG_BASE_ADDR;

    /// A byte no command in this suite writes, used to prove "no analog write".
    const ANALOG_SENTINEL_ADDR: u8 = 0x78;
    const ANALOG_SENTINEL_VALUE: u8 = 0x5a;

    /// `FakeMmio`, `FakeAnalog` and `fake_clock` are process-wide singletons and
    /// the suite runs with `--test-threads=1`, so every test starts from zero.
    fn reset_fakes() {
        FakeMmio::reset();
        FakeAnalog::reset();
        fake_clock::reset();
    }

    #[test]
    fn test_load_tbl_cmd_set_empty() {
        reset_fakes();

        let result = load_tbl_cmd_set(&[]);

        assert_eq!(result, 0);
        assert!(FakeMmio::writes().is_empty(), "nothing may be written");
        assert!(fake_clock::sleeps().is_empty(), "nothing may be waited on");
    }

    #[test]
    fn test_load_tbl_cmd_set_write_cmd() {
        reset_fakes();

        // Create a test command set with TCMD_WRITE
        let cmd_set = [TBLCMDSET {
            adr: 0x1234,
            dat: 0x56,
            cmd: TCMD_WRITE | TCMD_UNDER_WR,
        }];

        let result = load_tbl_cmd_set(&cmd_set);

        // Assert the function returns the size of the command set
        assert_eq!(result, 1);

        // Verify the register was written with the expected value
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: 0x1234 | REG_BASE_ADDR,
                width: 1,
                value: 0x56,
            }]
        );
        assert_eq!(FakeMmio::reg8(0x1234), 0x56);
    }

    #[test]
    fn test_load_tbl_cmd_set_wareg_cmd() {
        reset_fakes();

        // Create a test command set with TCMD_WAREG
        let cmd_set = [TBLCMDSET {
            adr: ANALOG_SENTINEL_ADDR as u16,
            dat: 0x9A,
            cmd: TCMD_WAREG | TCMD_UNDER_WR,
        }];

        let result = load_tbl_cmd_set(&cmd_set);

        // Assert the function returns the size of the command set
        assert_eq!(result, 1);

        // Verify the analog register received the expected value
        assert_eq!(FakeAnalog::get(ANALOG_SENTINEL_ADDR), 0x9A);
        assert!(
            FakeMmio::writes().is_empty(),
            "an analog write must not touch the MMIO bank"
        );
    }

    #[test]
    fn test_load_tbl_cmd_set_wait_cmd() {
        reset_fakes();

        // Create a test command set with TCMD_WAIT
        let cmd_set = [TBLCMDSET {
            adr: 0x02, // 0x02 * 256 = 512
            dat: 0x34, // + 0x34 = 52 = 564
            cmd: TCMD_WAIT | TCMD_UNDER_WR,
        }];

        let result = load_tbl_cmd_set(&cmd_set);

        // Assert the function returns the size of the command set
        assert_eq!(result, 1);

        // Verify the wait duration reached the clock leaf
        assert_eq!(fake_clock::sleeps(), vec![564]);
    }

    #[test]
    fn test_load_tbl_cmd_set_unsupported_cmd() {
        reset_fakes();
        FakeAnalog::set(ANALOG_SENTINEL_ADDR, ANALOG_SENTINEL_VALUE);

        // Create a test command set with an unsupported command value
        let cmd_set = [TBLCMDSET {
            adr: 0x1234,
            dat: 0x56,
            cmd: 0x0F | TCMD_UNDER_WR, // Unsupported command
        }];

        let result = load_tbl_cmd_set(&cmd_set);

        // Assert the function returns the size of the command set
        assert_eq!(result, 1);

        // No side effect may be observed
        assert!(FakeMmio::writes().is_empty(), "no register write");
        assert_eq!(
            FakeAnalog::get(ANALOG_SENTINEL_ADDR),
            ANALOG_SENTINEL_VALUE,
            "no analog write"
        );
        assert!(fake_clock::sleeps().is_empty(), "no wait");
    }

    #[test]
    fn test_load_tbl_cmd_set_invalid_flag() {
        reset_fakes();
        FakeAnalog::set(ANALOG_SENTINEL_ADDR, ANALOG_SENTINEL_VALUE);

        // Create a test command set with invalid cmd flag (no TCMD_UNDER_WR)
        let cmd_set = [TBLCMDSET {
            adr: 0x1234,
            dat: 0x56,
            cmd: TCMD_WRITE, // Missing TCMD_UNDER_WR flag
        }];

        let result = load_tbl_cmd_set(&cmd_set);

        // Assert the function returns the size of the command set
        assert_eq!(result, 1);

        // No side effect may be observed
        assert!(FakeMmio::writes().is_empty(), "no register write");
        assert_eq!(
            FakeAnalog::get(ANALOG_SENTINEL_ADDR),
            ANALOG_SENTINEL_VALUE,
            "no analog write"
        );
        assert!(fake_clock::sleeps().is_empty(), "no wait");
    }

    #[test]
    fn test_load_tbl_cmd_set_multiple_commands() {
        reset_fakes();

        // Create a test command set with multiple commands
        let cmd_set = [
            TBLCMDSET {
                adr: 0x1234,
                dat: 0x56,
                cmd: TCMD_WRITE | TCMD_UNDER_WR,
            },
            TBLCMDSET {
                adr: ANALOG_SENTINEL_ADDR as u16,
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

        let result = load_tbl_cmd_set(&cmd_set);

        // Assert the function returns the size of the command set
        assert_eq!(result, 4);

        // Verify all three command kinds took effect exactly once
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: 0x1234 | REG_BASE_ADDR,
                width: 1,
                value: 0x56,
            }]
        );
        assert_eq!(FakeAnalog::get(ANALOG_SENTINEL_ADDR), 0x9A);
        assert_eq!(fake_clock::sleeps(), vec![564]);
    }
}
