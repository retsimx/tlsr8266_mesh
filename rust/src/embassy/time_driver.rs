use core::sync::atomic::{AtomicU32, Ordering};
use embassy_time::driver::{Driver, AlarmHandle};
use crate::sdk::mcu::clock::clock_time;

struct MyDriver {

}

embassy_time::time_driver_impl!(static DRIVER: MyDriver = MyDriver{});

impl Driver for MyDriver {
    fn now(&self) -> u64 {
        clock_time64() as u64
    }
    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        return Option::from(AlarmHandle::new(0));
    }
    fn set_alarm_callback(&self, _alarm: AlarmHandle, _callback: fn(*mut ()), _ctx: *mut ()) {

    }
    fn set_alarm(&self, _alarm: AlarmHandle, _timestamp: u64) -> bool {
        _timestamp > self.now()
    }
}

// Counts any clock_time overflows
pub fn clock_time64() -> u64 {
    static CLOCK_TIME_UPPER: AtomicU32 = AtomicU32::new(0);
    static LAST_CLOCK_TIME: AtomicU32 = AtomicU32::new(0);

    critical_section::with(|_| {
        let time = clock_time();
        if time < LAST_CLOCK_TIME.load(Ordering::Relaxed) {
            // Overflow has occurred
            CLOCK_TIME_UPPER.store(CLOCK_TIME_UPPER.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
        }

        LAST_CLOCK_TIME.store(time, Ordering::Relaxed);

        (CLOCK_TIME_UPPER.load(Ordering::Relaxed) as u64) << 32 | time as u64
    })
}
