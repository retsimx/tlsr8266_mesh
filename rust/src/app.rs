use std::io::Write;
use config::MESH_PWD_ENCODE_SK;
use main_light::{main_loop, user_init};
use sdk::light::{_rf_drv_init, get_pair_config_pwd_encode_sk};
use sdk::mcu::clock::clock_init;
use sdk::mcu::dma::dma_init;
use sdk::mcu::gpio::gpio_init;
use sdk::mcu::irq_i::{irq_enable, irq_init};
use sdk::mcu::watchdog::wd_clear;
use sdk::pm::cpu_wakeup_init;

fn fill_from_str(mut bytes: &mut [u8], s: &[u8]) {
    bytes.write(s).unwrap();
}

pub struct App {

}

impl App {
    pub const fn default() -> App {
        App {

        }
    }

    pub unsafe fn init(&self) {
        // Copy the password in to the pair config
        unsafe { fill_from_str(get_pair_config_pwd_encode_sk(), MESH_PWD_ENCODE_SK.as_bytes()); }

        // Init various subsystems
        cpu_wakeup_init();
        clock_init();
        dma_init();
        gpio_init();
        irq_init();
        _rf_drv_init(0);

        // Run our initialisation
        user_init();
    }

    pub unsafe fn run(&self) {
        // Ready to go, enable interrupts and run the main loop
        irq_enable();

        loop {
            wd_clear();
            main_loop();
        }
    }
}