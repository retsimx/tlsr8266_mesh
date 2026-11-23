//! Timer peripheral control for the TLSR8266 SoC.
//!
//! This module owns the register-width decision for the TMR0/TMR1 control
//! register, so callers state intent (`tmr1_start()`) instead of re-deriving
//! masks at every site.
//!
//! The control register is deliberately accessed as the 8-bit view
//! (`reg_tmr_ctrl8`, `0x620`): the 32-bit window at `0x620` ends at `0x623`,
//! which is the write-1-to-clear `reg_tmr_sta` register. A 32-bit
//! read-modify-write of the control word therefore retires timer status bits it
//! only meant to read. `TMR0_EN` (bit 0) and `TMR1_EN` (bit 3) both live in the
//! low byte, so the 8-bit accessor is sufficient.

use tlsr8266_pac::{
    read_reg_tmr_ctrl8, write_reg_tmr0_capt, write_reg_tmr0_tick, write_reg_tmr1_capt,
    write_reg_tmr1_tick, write_reg_tmr_ctrl8, write_reg_tmr_sta, FLD_TMR, FLD_TMR_STA,
};

/// Configures TMR0 and leaves it running.
///
/// # Algorithm
///
/// 1. Clear `TMR0_EN` in the 8-bit timer control register
/// 2. Reset the TMR0 tick counter to zero
/// 3. Program the TMR0 capture (period) value
/// 4. Set `TMR0_EN` in the 8-bit timer control register
///
/// # Parameters
///
/// * `capt_ticks` - Capture value in system clock ticks
///
/// # Notes
///
/// * The disable step is what makes the capture write take effect on a timer
///   that was already running
/// * Other bits of `reg_tmr_ctrl8` (TMR1/TMR2/watchdog fields) are preserved
pub fn tmr0_configure_and_enable(capt_ticks: u32) {
    write_reg_tmr_ctrl8(read_reg_tmr_ctrl8() & !(FLD_TMR::TMR0_EN.bits() as u8));
    write_reg_tmr0_tick(0);
    write_reg_tmr0_capt(capt_ticks);
    write_reg_tmr_ctrl8(read_reg_tmr_ctrl8() | FLD_TMR::TMR0_EN.bits() as u8);
}

/// Acknowledges the TMR0 interrupt by clearing its status bit.
///
/// # Notes
///
/// * `reg_tmr_sta` is write-1-to-clear, so the written value must contain only
///   the status bits being acknowledged
pub fn tmr0_ack_irq() {
    write_reg_tmr_sta(FLD_TMR_STA::TMR0.bits());
}

/// Programs the TMR1 capture (period) value.
///
/// # Parameters
///
/// * `capt_ticks` - Capture value in system clock ticks
///
/// # Notes
///
/// * Does not start or stop TMR1; see [`tmr1_start`] and [`tmr1_stop`]
pub fn tmr1_configure(capt_ticks: u32) {
    write_reg_tmr1_capt(capt_ticks);
}

/// Resets the TMR1 tick counter and enables TMR1.
///
/// # Algorithm
///
/// 1. Reset the TMR1 tick counter to zero
/// 2. Set `TMR1_EN` in the 8-bit timer control register
///
/// # Notes
///
/// * Other bits of `reg_tmr_ctrl8` are preserved
pub fn tmr1_start() {
    write_reg_tmr1_tick(0);
    write_reg_tmr_ctrl8(read_reg_tmr_ctrl8() | FLD_TMR::TMR1_EN.bits() as u8);
}

/// Disables TMR1 without touching its capture or tick value.
///
/// # Notes
///
/// * Only `TMR1_EN` is cleared; every other bit of `reg_tmr_ctrl8` is preserved
pub fn tmr1_stop() {
    write_reg_tmr_ctrl8(read_reg_tmr_ctrl8() & !(FLD_TMR::TMR1_EN.bits() as u8));
}

/// Acknowledges the TMR1 interrupt by clearing its status bit.
///
/// # Notes
///
/// * `reg_tmr_sta` is write-1-to-clear, so the written value must contain only
///   the status bits being acknowledged
pub fn tmr1_ack_irq() {
    write_reg_tmr_sta(FLD_TMR_STA::TMR1.bits());
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use tlsr8266_pac::fake_mmio::{FakeMmio, RegWrite};
    use tlsr8266_pac::REG_BASE_ADDR;

    /// The 8-bit timer control register, cleared to the documented state where
    /// TMR1 is already off but TMR0 is not.
    const TMR_CTRL8: u32 = 0x620;
    const TMR0_CAPT: u32 = 0x624;
    const TMR1_CAPT: u32 = 0x628;
    const TMR0_TICK: u32 = 0x630;
    const TMR1_TICK: u32 = 0x634;
    const TMR_STA: u32 = 0x623;

    /// Tests that tmr1_start zeroes the tick counter, sets TMR1_EN and leaves
    /// every other control bit alone.
    ///
    /// # Algorithm
    ///
    /// 1. Seed `reg_tmr_ctrl8` with TMR0 enabled and TMR1 disabled
    /// 2. Seed a pending TMR0 status bit in `reg_tmr_sta`
    /// 3. Call tmr1_start()
    /// 4. Verify the tick write, the control bits and the untouched status
    ///
    /// # Notes
    ///
    /// * The write-log assertion pins the control access to **8 bits**: a
    ///   32-bit read-modify-write would retire the seeded `reg_tmr_sta` bits
    #[test]
    fn test_tmr1_start_enables_tmr1_and_zeroes_tick() {
        FakeMmio::reset();
        FakeMmio::set_reg8(TMR_CTRL8, 0b0000_0001); // TMR0 on, TMR1 off
        FakeMmio::set_reg8(TMR_STA, FLD_TMR_STA::TMR0.bits()); // pending status
        FakeMmio::clear_writes();

        tmr1_start();

        assert_eq!(
            FakeMmio::reg8(TMR_CTRL8),
            0b0000_1001,
            "TMR1_EN (bit 3) must be set and TMR0_EN (bit 0) preserved"
        );
        assert_eq!(
            FakeMmio::reg32(TMR1_TICK),
            0,
            "tmr1_start must reset the TMR1 tick counter"
        );
        assert_eq!(
            FakeMmio::reg8(TMR_STA),
            FLD_TMR_STA::TMR0.bits(),
            "must not write the status byte; the write-width check below is the W1C proof"
        );
        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: TMR1_TICK | REG_BASE_ADDR,
                    width: 4,
                    value: 0,
                },
                RegWrite {
                    addr: TMR_CTRL8 | REG_BASE_ADDR,
                    width: 1,
                    value: 0b0000_1001,
                },
            ]
        );
    }

    /// Tests that tmr1_stop clears only TMR1_EN.
    ///
    /// # Notes
    ///
    /// * A control byte with most bits set proves the other timer, mode and
    ///   watchdog fields survive the read-modify-write
    #[test]
    fn test_tmr1_stop_clears_only_tmr1_enable() {
        FakeMmio::reset();
        FakeMmio::set_reg8(TMR_CTRL8, 0b1111_1011); // everything but TMR1_EN
        FakeMmio::clear_writes();

        tmr1_stop();

        assert_eq!(
            FakeMmio::reg8(TMR_CTRL8),
            0b1111_0011,
            "only TMR1_EN (bit 3) may change"
        );
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: TMR_CTRL8 | REG_BASE_ADDR,
                width: 1,
                value: 0b1111_0011,
            }],
            "tmr1_stop must not write a tick or capture register"
        );
    }

    /// Tests that tmr1_configure writes the capture register.
    #[test]
    fn test_tmr1_configure_writes_capture() {
        FakeMmio::reset();
        FakeMmio::clear_writes();

        tmr1_configure(0x1122_3344);

        assert_eq!(FakeMmio::reg32(TMR1_CAPT), 0x1122_3344);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: TMR1_CAPT | REG_BASE_ADDR,
                width: 4,
                value: 0x1122_3344,
            }]
        );
    }

    /// Tests that tmr0_configure_and_enable programs the tick and capture
    /// values and finishes with TMR0 enabled.
    ///
    /// # Notes
    ///
    /// * The seeded byte has TMR1 enabled, so the end state also proves the
    ///   other timer bit survives the read-modify-write
    #[test]
    fn test_tmr0_configure_and_enable_programs_timer() {
        const CAPT_TICKS: u32 = 32_000_000;

        FakeMmio::reset();
        FakeMmio::set_reg8(TMR_CTRL8, 0b0000_1000); // TMR1 on, TMR0 off
        FakeMmio::clear_writes();

        tmr0_configure_and_enable(CAPT_TICKS);

        assert_eq!(
            FakeMmio::reg8(TMR_CTRL8),
            0b0000_1001,
            "TMR0_EN (bit 0) must end set and TMR1_EN (bit 3) preserved"
        );
        assert_eq!(
            FakeMmio::reg32(TMR0_TICK),
            0,
            "the TMR0 tick counter must be reset"
        );
        assert_eq!(
            FakeMmio::reg32(TMR0_CAPT),
            CAPT_TICKS,
            "the TMR0 capture value must be programmed"
        );
    }

    /// Tests that tmr0_configure_and_enable disables TMR0 before writing the
    /// capture value and re-enables it afterwards.
    ///
    /// # Notes
    ///
    /// * The first control write must have bit 0 clear and the last one set;
    ///   the log proves the sequence instead of only the end state
    #[test]
    fn test_tmr0_configure_and_enable_disables_before_capture() {
        const CAPT_TICKS: u32 = 32_000_000;

        FakeMmio::reset();
        FakeMmio::set_reg8(TMR_CTRL8, 0b0000_1000); // TMR1 on, TMR0 off
        FakeMmio::clear_writes();

        tmr0_configure_and_enable(CAPT_TICKS);

        assert_eq!(
            FakeMmio::writes(),
            vec![
                // TMR0 disabled before the capture write
                RegWrite {
                    addr: TMR_CTRL8 | REG_BASE_ADDR,
                    width: 1,
                    value: 0b0000_1000,
                },
                RegWrite {
                    addr: TMR0_TICK | REG_BASE_ADDR,
                    width: 4,
                    value: 0,
                },
                RegWrite {
                    addr: TMR0_CAPT | REG_BASE_ADDR,
                    width: 4,
                    value: CAPT_TICKS,
                },
                // TMR0 enabled again
                RegWrite {
                    addr: TMR_CTRL8 | REG_BASE_ADDR,
                    width: 1,
                    value: 0b0000_1001,
                },
            ]
        );
    }

    /// Tests that each timer interrupt ack writes only its own status bit.
    ///
    /// # Notes
    ///
    /// * `reg_tmr_sta` is write-1-to-clear, so the two acks must not be
    ///   interchangeable: TMR0 writes 0x01 and TMR1 writes 0x02
    #[test]
    fn test_timer_acks_write_their_status_bits() {
        FakeMmio::reset();
        FakeMmio::clear_writes();

        tmr0_ack_irq();

        assert_eq!(FakeMmio::reg8(TMR_STA), FLD_TMR_STA::TMR0.bits());
        assert_eq!(FakeMmio::reg8(TMR_STA), 0x01);

        FakeMmio::reset();
        FakeMmio::clear_writes();

        tmr1_ack_irq();

        assert_eq!(FakeMmio::reg8(TMR_STA), FLD_TMR_STA::TMR1.bits());
        assert_eq!(FakeMmio::reg8(TMR_STA), 0x02);

        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: TMR_STA | REG_BASE_ADDR,
                width: 1,
                value: 0x02,
            }]
        );
    }
}
