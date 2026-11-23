use crate::sdk::mcu::clock::{clock_time64, CLOCK_SYS_CLOCK_1S};
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
#[cfg_attr(test, mry::mry)]
pub fn on_timer0_interrupt() {
    DRIVER.on_interrupt();
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::sdk::mcu::clock::{mock_clock_time64, reset_clock_time64_for_test};
    use mry::{self, Any};
    use std::sync::Mutex;
    use tlsr8266_pac::fake_mmio::{FakeMmio, RegWrite};
    use tlsr8266_pac::{
        mock_read_reg_tmr_ctrl8, mock_write_reg_tmr0_capt, mock_write_reg_tmr0_tick,
        mock_write_reg_tmr_ctrl8, mock_write_reg_tmr_sta, REG_BASE_ADDR,
    };

    /// Tag used to identify a mocked register write, in call order.
    #[derive(Clone, Debug, PartialEq, Eq)]
    enum Tmr0Write {
        Ctrl8(u8),
        Tick(u32),
        Capt(u32),
        Sta(u8),
    }

    /// Shared log of `TMR0` register writes, filled by the mock closures.
    static WRITE_LOG: Mutex<Vec<Tmr0Write>> = Mutex::new(Vec::new());

    fn reset_write_log() {
        WRITE_LOG.lock().unwrap().clear();
    }

    fn write_log() -> Vec<Tmr0Write> {
        WRITE_LOG.lock().unwrap().clone()
    }

    #[test]
    #[mry::lock(clock_time64)]
    fn test_driver_now_returns_clock_time64() {
        // Arrange
        let driver = MyDriver::new();
        let expected_time = 0x1234567890ABCDEF;

        // Mock clock_time64 with the correct format
        mock_clock_time64().returns(expected_time);

        // Act
        let result = driver.now();

        // Assert
        assert_eq!(result, expected_time);
        mock_clock_time64().assert_called(1);
    }

    #[test]
    #[mry::lock(
        clock_time64,
        read_reg_tmr_ctrl8,
        write_reg_tmr_ctrl8,
        write_reg_tmr0_tick,
        write_reg_tmr0_capt,
        write_reg_tmr_sta
    )]
    fn test_arm_tmr0_past_timestamp_returns_false_without_writes() {
        // Arrange
        let driver = MyDriver::new();
        mock_clock_time64().returns(5000);

        // Act
        let armed = driver.arm_tmr0(4000);

        // Assert
        assert!(!armed);
        mock_clock_time64().assert_called(1);
        mock_read_reg_tmr_ctrl8().assert_called(0);
        mock_write_reg_tmr_ctrl8(Any).assert_called(0);
        mock_write_reg_tmr0_tick(Any).assert_called(0);
        mock_write_reg_tmr0_capt(Any).assert_called(0);
        mock_write_reg_tmr_sta(Any).assert_called(0);
    }

    #[test]
    #[mry::lock(
        clock_time64,
        read_reg_tmr_ctrl8,
        write_reg_tmr_ctrl8,
        write_reg_tmr0_tick,
        write_reg_tmr0_capt,
        write_reg_tmr_sta
    )]
    fn test_arm_tmr0_near_future_clamps_delta_to_32_ticks() {
        // Arrange
        let driver = MyDriver::new();
        reset_write_log();
        mock_clock_time64().returns(1000);
        mock_read_reg_tmr_ctrl8().returns(0x01);
        mock_write_reg_tmr_ctrl8(Any).returns_with(|value: u8| {
            WRITE_LOG.lock().unwrap().push(Tmr0Write::Ctrl8(value));
        });
        mock_write_reg_tmr0_tick(Any).returns_with(|value: u32| {
            WRITE_LOG.lock().unwrap().push(Tmr0Write::Tick(value));
        });
        mock_write_reg_tmr0_capt(Any).returns_with(|value: u32| {
            WRITE_LOG.lock().unwrap().push(Tmr0Write::Capt(value));
        });
        mock_write_reg_tmr_sta(Any).returns_with(|value: u8| {
            WRITE_LOG.lock().unwrap().push(Tmr0Write::Sta(value));
        });

        // Act
        let armed = driver.arm_tmr0(1010);

        // Assert
        assert!(armed);
        assert_eq!(
            write_log(),
            vec![
                Tmr0Write::Ctrl8(0x00),
                Tmr0Write::Tick(0),
                Tmr0Write::Capt(32),
                Tmr0Write::Sta(FLD_TMR_STA::TMR0.bits()),
                Tmr0Write::Ctrl8(0x01),
            ]
        );
    }

    #[test]
    #[mry::lock(
        clock_time64,
        read_reg_tmr_ctrl8,
        write_reg_tmr_ctrl8,
        write_reg_tmr0_tick,
        write_reg_tmr0_capt,
        write_reg_tmr_sta
    )]
    fn test_arm_tmr0_far_future_clamps_delta_to_one_second() {
        // Arrange
        let driver = MyDriver::new();
        reset_write_log();
        mock_clock_time64().returns(0);
        mock_read_reg_tmr_ctrl8().returns(0x01);
        mock_write_reg_tmr_ctrl8(Any).returns_with(|value: u8| {
            WRITE_LOG.lock().unwrap().push(Tmr0Write::Ctrl8(value));
        });
        mock_write_reg_tmr0_tick(Any).returns_with(|value: u32| {
            WRITE_LOG.lock().unwrap().push(Tmr0Write::Tick(value));
        });
        mock_write_reg_tmr0_capt(Any).returns_with(|value: u32| {
            WRITE_LOG.lock().unwrap().push(Tmr0Write::Capt(value));
        });
        mock_write_reg_tmr_sta(Any).returns_with(|value: u8| {
            WRITE_LOG.lock().unwrap().push(Tmr0Write::Sta(value));
        });

        // Act
        let armed = driver.arm_tmr0(CLOCK_SYS_CLOCK_1S as u64 * 10);

        // Assert
        assert!(armed);
        assert_eq!(
            write_log(),
            vec![
                Tmr0Write::Ctrl8(0x00),
                Tmr0Write::Tick(0),
                Tmr0Write::Capt(CLOCK_SYS_CLOCK_1S),
                Tmr0Write::Sta(FLD_TMR_STA::TMR0.bits()),
                Tmr0Write::Ctrl8(0x01),
            ]
        );
    }

    // Proof slices (design 022 §6.2): the same `arm_tmr0` behaviour asserted above,
    // but expressed against the bytes written to the hardware instead of against
    // mocked accessor calls. Deliberately carry no mry lock attribute, so the real
    // accessor bodies run and route through the in-process `FakeMmio` bank.

    #[test]
    fn test_arm_tmr0_near_future_via_fake_mmio() {
        // Arrange
        FakeMmio::reset();
        reset_clock_time64_for_test();
        FakeMmio::set_reg32(0x740, 1000); // reg_system_tick -> now == 1000
        FakeMmio::set_reg8(0x620, 0x01); // TMR0 already enabled
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
                    addr: 0x620 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x00,
                },
                // tick = 0
                RegWrite {
                    addr: 0x630 | REG_BASE_ADDR,
                    width: 4,
                    value: 0,
                },
                // delta 10 clamped up to 32
                RegWrite {
                    addr: 0x624 | REG_BASE_ADDR,
                    width: 4,
                    value: 32,
                },
                // status clear
                RegWrite {
                    addr: 0x623 | REG_BASE_ADDR,
                    width: 1,
                    value: FLD_TMR_STA::TMR0.bits() as u32,
                },
                // ctrl8 | TMR0_EN
                RegWrite {
                    addr: 0x620 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x01,
                },
            ]
        );
    }

    #[test]
    fn test_arm_tmr0_far_future_via_fake_mmio() {
        // Arrange
        FakeMmio::reset();
        reset_clock_time64_for_test();
        FakeMmio::set_reg32(0x740, 0); // reg_system_tick -> now == 0
        FakeMmio::set_reg8(0x620, 0x01); // TMR0 already enabled
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
                    addr: 0x620 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x00,
                },
                // tick = 0
                RegWrite {
                    addr: 0x630 | REG_BASE_ADDR,
                    width: 4,
                    value: 0,
                },
                // delta clamped to one second
                RegWrite {
                    addr: 0x624 | REG_BASE_ADDR,
                    width: 4,
                    value: CLOCK_SYS_CLOCK_1S,
                },
                // status clear
                RegWrite {
                    addr: 0x623 | REG_BASE_ADDR,
                    width: 1,
                    value: FLD_TMR_STA::TMR0.bits() as u32,
                },
                // ctrl8 | TMR0_EN
                RegWrite {
                    addr: 0x620 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x01,
                },
            ]
        );
    }
}
