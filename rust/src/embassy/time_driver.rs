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
static mut CLOCK_TIME_UPPER: u32 = 0;
static mut LAST_CLOCK_TIME: u32 = 0;

pub unsafe fn check_clock_overflow() -> u32 {
    critical_section::with(|_| {
        let time = clock_time();
        if time < LAST_CLOCK_TIME {
            // Overflow has occurred
            CLOCK_TIME_UPPER += 1;
        }

        LAST_CLOCK_TIME = time;

        time
    })
}

pub fn clock_time64() -> u64 {
    unsafe {
        let time = check_clock_overflow();
        return (CLOCK_TIME_UPPER as u64) << 32 | time as u64;
    }
}