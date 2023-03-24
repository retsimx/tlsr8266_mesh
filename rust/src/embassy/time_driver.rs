use embassy_time::driver::{Driver, AlarmHandle};
use crate::main_light::{clock_time64};

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