use crate::sdk::mcu::clock::{clock_time, CLOCK_SYS_CLOCK_1S};
use crate::sdk::mcu::register::{
    read_reg_tmr_ctrl8, write_reg_tmr0_capt, write_reg_tmr0_tick, write_reg_tmr_ctrl8,
    write_reg_tmr_sta, FLD_TMR, FLD_TMR_STA,
};
use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Waker;
use embassy_time_driver::Driver;

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
static CLOCK_TIME_UPPER: AtomicU32 = AtomicU32::new(0);
#[cfg(test)]
static LAST_CLOCK_TIME: AtomicU32 = AtomicU32::new(0);

/// Provides a 64-bit monotonic clock based on the 32-bit clock_time()
///
/// This function extends the 32-bit hardware timer to a 64-bit logical timer by
/// tracking overflows of the underlying 32-bit counter. It uses atomic operations
/// to safely handle concurrent access without locks in the common case.
///
/// # Algorithm
/// 1. Inside a critical section (interrupts disabled), read the current hardware time
///    and the previously seen time atomically together
/// 2. If current time < last time, the 32-bit hardware counter has overflowed - increment
///    the upper 32 bits to extend the counter
/// 3. Update the last seen time
/// 4. Return a combined 64-bit timestamp (upper 32 bits | lower 32 bits)
///
/// # Why clock_time() must be called inside the critical section
///
/// Previously, clock_time() was called BEFORE entering the critical section. This created
/// a race condition: if a hardware interrupt fired between reading clock_time() and reading
/// LAST_CLOCK_TIME, and that interrupt handler itself called clock_time64() (e.g. to step
/// a light transition), it would update LAST_CLOCK_TIME to a newer value. When the
/// interrupted code resumed, LAST_CLOCK_TIME was now genuinely larger than the stale
/// current_time, so the code falsely detected a 32-bit overflow and incremented the upper
/// bits. At 16-24 MHz this adds ~180-268 seconds to Instant::now() instantaneously, which
/// blew past any in-progress transition timestamp and caused the light to snap to its final
/// state. By reading clock_time() inside the critical section, interrupts are physically
/// incapable of wedging themselves between the hardware read and the comparison.
#[cfg_attr(test, mry::mry)]
pub fn clock_time64() -> u64 {
    // When not testing, these static variables are defined here
    // When testing, they're defined at module level for test access
    #[cfg(not(test))]
    static CLOCK_TIME_UPPER: AtomicU32 = AtomicU32::new(0);
    #[cfg(not(test))]
    static LAST_CLOCK_TIME: AtomicU32 = AtomicU32::new(0);

    critical_section::with(|_| {
        // Read the hardware time inside the critical section so that no interrupt can update
        // LAST_CLOCK_TIME between this read and the comparison below.
        let current_time = clock_time();
        let last_time = LAST_CLOCK_TIME.load(Ordering::Relaxed);

        if current_time < last_time {
            // Overflow confirmed - increment upper bits.
            // Use load-modify-store instead of fetch_add since fetch_add might not be supported
            // on all platforms or with all atomics implementations.
            let upper = CLOCK_TIME_UPPER.load(Ordering::Relaxed);
            CLOCK_TIME_UPPER.store(upper + 1, Ordering::Relaxed);
        }

        // Always update the last seen time so subsequent calls can detect the next overflow.
        LAST_CLOCK_TIME.store(current_time, Ordering::Relaxed);

        // Combine upper and lower bits to form the 64-bit timestamp.
        // Upper 32 bits track the number of overflows.
        // Lower 32 bits are the current hardware timer value.
        (CLOCK_TIME_UPPER.load(Ordering::Relaxed) as u64) << 32 | current_time as u64
    })
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::sdk::mcu::clock::mock_clock_time;
    use crate::sdk::mcu::register::{
        mock_read_reg_tmr_ctrl8, mock_write_reg_tmr0_capt, mock_write_reg_tmr0_tick,
        mock_write_reg_tmr_ctrl8, mock_write_reg_tmr_sta,
    };
    use mry::{self, Any};
    use std::sync::Mutex;

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

    // Helper function to reset static variables for testing without using unsafe
    fn reset_static_vars() {
        CLOCK_TIME_UPPER.store(0, Ordering::Relaxed);
        LAST_CLOCK_TIME.store(0, Ordering::Relaxed);
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

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_no_overflow() {
        // Arrange
        reset_static_vars();

        // Set initial value
        let initial_value = 1000;
        LAST_CLOCK_TIME.store(initial_value, Ordering::Relaxed);

        // Simulate non-overflow case
        let next_value = initial_value + 100;

        // Mock clock_time with the correct format
        mock_clock_time().returns(next_value);

        // Act
        let result = clock_time64();

        // Assert
        assert_eq!(result, next_value as u64);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 0);
        assert_eq!(LAST_CLOCK_TIME.load(Ordering::Relaxed), next_value);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_with_overflow() {
        // Arrange
        reset_static_vars();

        // Set up overflow scenario
        let initial_value = 0xFFFFFF00_u32;
        LAST_CLOCK_TIME.store(initial_value, Ordering::Relaxed);

        // Simulate overflow: next value will be less than previous
        let next_value = 100_u32;

        // Mock clock_time with the correct format
        mock_clock_time().returns(next_value);

        // Act
        let result = clock_time64();

        // Assert
        // The upper 32 bits should be incremented to 1, and lower 32 bits should be our next value
        assert_eq!(result, (1_u64 << 32) | next_value as u64);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 1);
        assert_eq!(LAST_CLOCK_TIME.load(Ordering::Relaxed), next_value);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_normal_increment() {
        // Arrange
        reset_static_vars();
        LAST_CLOCK_TIME.store(1000, Ordering::Relaxed);
        mock_clock_time().returns(1500);

        // Act
        let result = clock_time64();

        // Assert
        assert_eq!(result, 1500);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 0);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_first_overflow() {
        // Arrange
        reset_static_vars();
        LAST_CLOCK_TIME.store(0xFFFFFF00, Ordering::Relaxed);
        mock_clock_time().returns(100);

        // Act
        let result = clock_time64();

        // Assert
        assert_eq!(result, (1_u64 << 32) | 100);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 1);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_after_overflow() {
        // Arrange
        reset_static_vars();
        LAST_CLOCK_TIME.store(100, Ordering::Relaxed);
        CLOCK_TIME_UPPER.store(1, Ordering::Relaxed);
        mock_clock_time().returns(200);

        // Act
        let result = clock_time64();

        // Assert
        assert_eq!(result, (1_u64 << 32) | 200);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 1);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_second_overflow() {
        // Arrange
        reset_static_vars();
        LAST_CLOCK_TIME.store(0xFFFFFF00, Ordering::Relaxed);
        CLOCK_TIME_UPPER.store(1, Ordering::Relaxed);
        mock_clock_time().returns(300);

        // Act
        let result = clock_time64();

        // Assert
        assert_eq!(result, (2_u64 << 32) | 300);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 2);
        mock_clock_time().assert_called(1);
    }
}
