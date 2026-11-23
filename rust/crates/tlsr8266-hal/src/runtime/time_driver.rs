use crate::mcu::clock::{clock_time64, CLOCK_SYS_CLOCK_1S};
use core::cell::RefCell;
use core::task::Waker;
use embassy_time_driver::Driver;
use tlsr8266_pac::{
    read_reg_tmr_ctrl8, write_reg_tmr0_capt, write_reg_tmr0_tick, write_reg_tmr_ctrl8,
    write_reg_tmr_sta, FLD_TMR, FLD_TMR_STA,
};

/// Custom time driver implementation for the embassy-time crate
///
/// This driver bridges the SDK's clock_time functionality with embassy's time system,
/// providing a monotonic 64-bit timestamp counter even though the underlying
/// hardware may have a 32-bit counter that wraps around. Expiring timers are
/// delivered by reprogramming the hardware `TMR0` to the earliest deadline.
struct MyDriver {
    queue: critical_section::Mutex<RefCell<embassy_time_queue_utils::Queue>>,
}

impl MyDriver {
    const fn new() -> Self {
        Self {
            queue: critical_section::Mutex::new(RefCell::new(
                embassy_time_queue_utils::Queue::new(),
            )),
        }
    }

    /// Arms `TMR0` to fire at `at`, returning `false` when that deadline has already passed.
    ///
    /// The register access goes through the 8-bit `reg_tmr_ctrl8` view of `0x620` only:
    /// the 32-bit `reg_tmr_ctrl` read-modify-write would also cover `0x623` (`reg_tmr_sta`),
    /// where writing a stale value back silently clears a pending `TMR1`/watchdog flag.
    fn arm_tmr0(&self, at: u64) -> bool {
        let now = clock_time64();
        if at <= now {
            return false;
        }

        // Cap the delta at one second so `clock_time64()` is called at least once per
        // second, which is what extends the 32-bit `reg_system_tick` past its 134 s wrap.
        let delta = (at - now).min(CLOCK_SYS_CLOCK_1S as u64) as u32;

        let tmr0_en = FLD_TMR::TMR0_EN.bits() as u8;
        let ctrl8 = read_reg_tmr_ctrl8();
        write_reg_tmr_ctrl8(ctrl8 & !tmr0_en);
        write_reg_tmr0_tick(0);
        write_reg_tmr0_capt(delta.max(32)); // >= 1 us at 32 MHz
        write_reg_tmr_sta(FLD_TMR_STA::TMR0.bits());
        write_reg_tmr_ctrl8(ctrl8 | tmr0_en);
        true
    }

    /// Re-arms `TMR0` for the earliest queued deadline, skipping deadlines that already
    /// passed while the previous one was being handled.
    ///
    /// The loop terminates because `next_expiration` returns `u64::MAX` once the queue is
    /// empty and [`arm_tmr0`](Self::arm_tmr0) accepts any deadline in the future.
    fn rearm(&self, queue: &mut embassy_time_queue_utils::Queue) {
        let mut next = queue.next_expiration(self.now());
        while !self.arm_tmr0(next) {
            next = queue.next_expiration(self.now());
        }
    }

    /// Delivers every timer that expired and re-arms `TMR0` for the next one.
    fn on_interrupt(&self) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            self.rearm(&mut queue);
        });
    }
}

// Register our driver with the embassy-time framework
embassy_time_driver::time_driver_impl!(static DRIVER: MyDriver = MyDriver::new());

impl Driver for MyDriver {
    /// Returns the current monotonic timestamp in ticks
    ///
    /// Uses clock_time64 to handle timer overflow and provide a continuous 64-bit counter
    fn now(&self) -> u64 {
        clock_time64()
    }

    /// Queues the waker for `at` and re-arms `TMR0` if that changed the next deadline.
    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            if queue.schedule_wake(at, waker) {
                self.rearm(&mut queue);
            }
        });
    }
}

/// Called from the `TMR0` interrupt to flush expired embassy timers and re-arm `TMR0`.
pub fn on_timer0_interrupt() {
    DRIVER.on_interrupt();
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::mcu::clock::fake_clock;
    use tlsr8266_pac::fake_mmio::{FakeMmio, RegWrite};
    use tlsr8266_pac::REG_BASE_ADDR;

    /// `reg_system_tick`, `reg_tmr_ctrl` (8-bit view), `reg_tmr0_tick`,
    /// `reg_tmr0_capt` and `reg_tmr_sta`.
    const SYSTEM_TICK: u32 = 0x740;
    const TMR_CTRL8: u32 = 0x620;
    const TMR0_TICK: u32 = 0x630;
    const TMR0_CAPT: u32 = 0x624;
    const TMR_STA: u32 = 0x623;

    /// `FakeMmio` and `fake_clock` are process-wide and the suite runs with
    /// `--test-threads=1`, so every test starts from zero.
    fn reset_fakes() {
        FakeMmio::reset();
        fake_clock::reset();
    }

    #[test]
    fn test_driver_now_returns_clock_time64() {
        // Arrange
        reset_fakes();
        let driver = MyDriver::new();
        let expected_time = 0x1234567890ABCDEF;
        fake_clock::set_ticks(expected_time);

        // Act
        let result = driver.now();

        // Assert
        assert_eq!(result, expected_time);
    }

    #[test]
    fn test_arm_tmr0_past_timestamp_returns_false_without_writes() {
        // Arrange
        reset_fakes();
        let driver = MyDriver::new();
        // `now` is 5000 ticks, the deadline is already behind it
        fake_clock::set_ticks(5000);

        // Act
        let armed = driver.arm_tmr0(4000);

        // Assert
        assert!(!armed);
        assert!(
            FakeMmio::writes().is_empty(),
            "a past deadline must not touch TMR0"
        );
    }

    #[test]
    fn test_arm_tmr0_near_future_clamps_delta_to_32_ticks() {
        // Arrange
        reset_fakes();
        let driver = MyDriver::new();
        fake_clock::set_ticks(1000);
        FakeMmio::set_reg8(TMR_CTRL8, 0x01); // TMR0 already enabled

        // Act: a 10 tick delta is below the 1 us floor
        let armed = driver.arm_tmr0(1010);

        // Assert: end state, not just the write order
        assert!(armed);
        assert_eq!(
            FakeMmio::reg32(TMR0_CAPT),
            32,
            "delta clamps up to 32 ticks"
        );
        assert_eq!(FakeMmio::reg32(TMR0_TICK), 0, "the tick counter restarts");
        assert_eq!(
            FakeMmio::reg8(TMR_STA),
            FLD_TMR_STA::TMR0.bits(),
            "the pending status bit is acknowledged"
        );
        assert_eq!(
            FakeMmio::reg8(TMR_CTRL8),
            0x01,
            "TMR0 is left enabled and every other control bit preserved"
        );
    }

    #[test]
    fn test_arm_tmr0_far_future_clamps_delta_to_one_second() {
        // Arrange
        reset_fakes();
        let driver = MyDriver::new();
        fake_clock::set_ticks(0);
        FakeMmio::set_reg8(TMR_CTRL8, 0x01); // TMR0 already enabled

        // Act: ten seconds out
        let armed = driver.arm_tmr0(CLOCK_SYS_CLOCK_1S as u64 * 10);

        // Assert: end state, not just the write order
        assert!(armed);
        assert_eq!(
            FakeMmio::reg32(TMR0_CAPT),
            CLOCK_SYS_CLOCK_1S,
            "delta clamps down to one second"
        );
        assert_eq!(FakeMmio::reg32(TMR0_TICK), 0, "the tick counter restarts");
        assert_eq!(FakeMmio::reg8(TMR_STA), FLD_TMR_STA::TMR0.bits());
        assert_eq!(FakeMmio::reg8(TMR_CTRL8), 0x01);
    }

    // Proof slices (design 022 §6.2): the same `arm_tmr0` behaviour asserted
    // above, but expressed against the ordered bytes written to the hardware.
    // The real accessor bodies run and route through the in-process `FakeMmio`
    // bank.

    #[test]
    fn test_arm_tmr0_near_future_via_fake_mmio() {
        // Arrange
        reset_fakes();
        FakeMmio::set_reg32(SYSTEM_TICK, 1000); // now == 1000
        FakeMmio::set_reg8(TMR_CTRL8, 0x01); // TMR0 already enabled
        FakeMmio::clear_writes();
        let driver = MyDriver::new();

        // Act
        let armed = driver.arm_tmr0(1010);

        // Assert
        assert!(armed);
        assert_eq!(
            FakeMmio::writes(),
            vec![
                // ctrl8 & !TMR0_EN
                RegWrite {
                    addr: TMR_CTRL8 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x00,
                },
                // tick = 0
                RegWrite {
                    addr: TMR0_TICK | REG_BASE_ADDR,
                    width: 4,
                    value: 0,
                },
                // delta 10 clamped up to 32
                RegWrite {
                    addr: TMR0_CAPT | REG_BASE_ADDR,
                    width: 4,
                    value: 32,
                },
                // status clear
                RegWrite {
                    addr: TMR_STA | REG_BASE_ADDR,
                    width: 1,
                    value: FLD_TMR_STA::TMR0.bits() as u32,
                },
                // ctrl8 | TMR0_EN
                RegWrite {
                    addr: TMR_CTRL8 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x01,
                },
            ]
        );
    }

    #[test]
    fn test_arm_tmr0_far_future_via_fake_mmio() {
        // Arrange
        reset_fakes();
        FakeMmio::set_reg32(SYSTEM_TICK, 0); // now == 0
        FakeMmio::set_reg8(TMR_CTRL8, 0x01); // TMR0 already enabled
        FakeMmio::clear_writes();
        let driver = MyDriver::new();

        // Act
        let armed = driver.arm_tmr0(CLOCK_SYS_CLOCK_1S as u64 * 10);

        // Assert
        assert!(armed);
        assert_eq!(
            FakeMmio::writes(),
            vec![
                // ctrl8 & !TMR0_EN
                RegWrite {
                    addr: TMR_CTRL8 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x00,
                },
                // tick = 0
                RegWrite {
                    addr: TMR0_TICK | REG_BASE_ADDR,
                    width: 4,
                    value: 0,
                },
                // delta clamped to one second
                RegWrite {
                    addr: TMR0_CAPT | REG_BASE_ADDR,
                    width: 4,
                    value: CLOCK_SYS_CLOCK_1S,
                },
                // status clear
                RegWrite {
                    addr: TMR_STA | REG_BASE_ADDR,
                    width: 1,
                    value: FLD_TMR_STA::TMR0.bits() as u32,
                },
                // ctrl8 | TMR0_EN
                RegWrite {
                    addr: TMR_CTRL8 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x01,
                },
            ]
        );
    }
}
