use std::io::Write;
use config::MESH_PWD_ENCODE_SK;
use main_light::{main_loop, user_init};
use ota::OtaManager;
use sdk::light::{_rf_drv_init, get_pair_config_pwd_encode_sk};
use sdk::mcu::clock::clock_init;
use sdk::mcu::dma::dma_init;
use sdk::mcu::gpio::gpio_init;
use sdk::mcu::irq_i::{irq_enable, irq_init};
use sdk::mcu::watchdog::wd_clear;
use sdk::pm::cpu_wakeup_init;

pub struct App {
    pub ota_manager: OtaManager
}

impl App {
    pub const fn default() -> App {
        App {
            ota_manager: OtaManager::default()
        }
    }

    pub fn init(&self) {
        // Copy the password in to the pair config
        get_pair_config_pwd_encode_sk().as_mut_slice().write(MESH_PWD_ENCODE_SK.as_bytes()).unwrap();

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

    pub fn run(&self) {
        // Ready to go, enable interrupts and run the main loop
        irq_enable();

        loop {
            wd_clear();
            main_loop();
        }
    }
}