use crate::config::MESH_PWD_ENCODE_SK;
use crate::main_light::{main_loop, user_init};
use crate::mesh::MeshManager;
use crate::ota::OtaManager;
use crate::sdk::mcu::clock::clock_init;
use crate::sdk::mcu::dma::dma_init;
use crate::sdk::mcu::gpio::gpio_init;
use crate::sdk::mcu::irq_i::{irq_enable, irq_init};
use crate::sdk::mcu::watchdog::wd_clear;
use crate::sdk::pm::cpu_wakeup_init;
use std::io::Write;
use crate::sdk::light::*;

pub struct App {
    pub ota_manager: OtaManager,
    pub mesh_manager: MeshManager,
}

// #[embassy_executor::task]
// async fn my_task() {
//
// }

impl App {
    pub const fn default() -> App {
        App {
            ota_manager: OtaManager::default(),
            mesh_manager: MeshManager::default(),
        }
    }

    pub fn init(&self) {
        // Copy the password in to the pair config
        get_pair_config_pwd_encode_sk()
            .as_mut_slice()
            .write(MESH_PWD_ENCODE_SK.as_bytes())
            .unwrap();

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
