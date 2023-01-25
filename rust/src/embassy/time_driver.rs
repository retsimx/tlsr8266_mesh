use embassy_time::driver::{Driver, AlarmHandle};
use crate::main_light::TIME_COUNTER_10US;

struct MyDriver {

}

embassy_time::time_driver_impl!(static DRIVER: MyDriver = MyDriver{});

impl Driver for MyDriver {
    fn now(&self) -> u64 {
        unsafe { return TIME_COUNTER_10US; }
    }
    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        todo!()
    }
    fn set_alarm_callback(&self, alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
        todo!()
    }
    fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) -> bool {
        todo!()
    }
}