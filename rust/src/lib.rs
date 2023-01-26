#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use app::App;
use ota::OtaManager;
use crate::embassy::executor::Executor;
use crate::main_light::light_adjust_rgb_hw;
use crate::sdk::mcu::watchdog::wd_clear;

mod app;
mod common;
mod config;
mod main_light;
mod mesh;
mod ota;
mod sdk;
mod vendor_light;
mod embassy;

static mut APP: App = App::default();

pub fn app() -> &'static mut App {
    return unsafe { &mut APP };
}

unsafe fn __make_static<T>(t: &mut T) -> &'static mut T {
    core::mem::transmute(t)
}

#[embassy_executor::task]
pub async fn run(spawner: Spawner) {
    app().run(spawner).await;
}

#[no_mangle]
pub fn blinken() {
    let mut led_onoff = true;
    for idx in 0..6 {
        if idx % 2 == 0 {
            light_adjust_rgb_hw(0xffff, 0xffff, 0xffff, 0xffff);
        } else {
            light_adjust_rgb_hw(0, 0, 0, 0);
        }

        for _ in 0..1000000 {
            wd_clear();
        }

        led_onoff = !led_onoff;
    }

    light_adjust_rgb_hw(0, 0, 0, 0);
}

#[no_mangle]
pub fn main_entrypoint() {
    // Must happen first
    OtaManager::handle_ota_update__attribute_ram_code();

    // Configure the system
    app().init();

    // Run the application
    let mut executor = Executor::new();
    let executor = unsafe { __make_static(&mut executor) };
    executor.run(|spawner| {
        spawner.must_spawn(run(spawner));
    });
}
