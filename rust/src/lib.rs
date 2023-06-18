#![feature(type_alias_impl_trait)]
#![no_std]

use core::ptr::null_mut;
use embassy_executor::Spawner;
use app::App;
use ota::OtaManager;
use crate::config::{MAX_LUM_BRIGHTNESS_VALUE, PWM_B, PWM_G};
use crate::embassy::executor::Executor;
use crate::sdk::mcu::gpio::*;
use crate::sdk::mcu::watchdog::wd_clear;
use fixed::types::I16F16;
use crate::sdk::pm::cpu_wakeup_init;

mod app;
mod common;
mod config;
mod main_light;
mod mesh;
mod ota;
mod sdk;
mod vendor_light;
mod embassy;
mod light_manager;
mod uart_manager;
pub mod version;

static mut APP: App = App::default();
static mut SPAWNER: *const Spawner = null_mut();

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

fn set_pw1(on: bool) {
    gpio_write(PWM_G as u32, if on {1} else {0});
}

fn set_pw2(on: bool) {
    gpio_write(PWM_B as u32, if on {1} else {0});
}

#[no_mangle]
pub fn blinken_testboard() {
    for idx in 0..6 {
        if idx % 2 == 0 {
            set_pw1(true);
            set_pw2(true);
        } else {
            set_pw1(false);
            set_pw2(false);
        }

        for _ in 0..1000000 {
            wd_clear();
        }
    }

    set_pw1(false);
    set_pw2(false);
}

pub fn blinken() {
    for idx in 0..6 {
        if idx % 2 == 0 {
            app().light_manager.light_adjust_rgb_hw(I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE), I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE), I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE));
        } else {
            app().light_manager.light_adjust_rgb_hw(I16F16::from_num(0), I16F16::from_num(0), I16F16::from_num(0));
        }

        for _ in 0..1000000 {
            wd_clear();
        }
    }

    app().light_manager.light_adjust_rgb_hw(I16F16::from_num(0), I16F16::from_num(0), I16F16::from_num(0));
}

// no_mangle because this is referenced as an entrypoint from the assembler bootstrap
#[no_mangle]
pub extern "C" fn main_entrypoint() {
    // Must happen first
    OtaManager::handle_ota_update();

    // Get the CPU configured
    cpu_wakeup_init();

    // Run the application
    let mut executor = Executor::new();
    let executor = unsafe { __make_static(&mut executor) };
    executor.run(|spawner| {
        unsafe {
            SPAWNER = &spawner;
        }

        spawner.must_spawn(run(spawner));
    });
}
