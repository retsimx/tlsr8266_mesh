use crate::config::MESH_PWD_ENCODE_SK;
use crate::main_light::{main_loop, user_init};
use crate::mesh::MeshManager;
use crate::ota::OtaManager;
use crate::sdk::mcu::clock::{clock_init};
use crate::sdk::mcu::dma::dma_init;
use crate::sdk::mcu::gpio::gpio_init;
use crate::sdk::mcu::irq_i::{irq_enable, irq_init};
use crate::sdk::mcu::watchdog::wd_clear;
use crate::sdk::pm::cpu_wakeup_init;
use std::io::Write;
use embassy_executor::Spawner;
use crate::{app};
use crate::embassy::yield_now::yield_now;
use crate::light_manager::LightManager;
use crate::sdk::drivers::uart::UartDriver;
use crate::sdk::light::*;
use crate::uart_manager::UartManager;

pub struct App {
    pub ota_manager: OtaManager,
    pub mesh_manager: MeshManager,
    pub light_manager: LightManager,
    pub uart_manager: UartManager
}

#[embassy_executor::task]
async fn light_manager(spawner: Spawner) {
    app().light_manager.run(spawner).await;
}

#[embassy_executor::task]
async fn uart_manager(spawner: Spawner) {
    app().uart_manager.run(spawner).await;
}

impl App {
    pub const fn default() -> App {
        App {
            ota_manager: OtaManager::default(),
            mesh_manager: MeshManager::default(),
            light_manager: LightManager::default(),
            uart_manager: UartManager::default()
        }
    }

    pub fn init(&mut self) {
        // Init various subsystems
        cpu_wakeup_init();

        // Copy the password in to the pair config
        get_pair_config_pwd_encode_sk()
            .as_mut_slice()
            .write(MESH_PWD_ENCODE_SK.as_bytes())
            .unwrap();

        clock_init();
        dma_init();
        gpio_init();
        irq_init();
        _rf_drv_init(0);

        // Get uart ready to go
        self.uart_manager.init();

        // Run our initialisation
        user_init();
    }

    pub async fn run(&mut self, spawner: Spawner) {
        // Ready to go, enable interrupts and run the main loop
        irq_enable();

        // Start the light manager
        spawner.spawn(light_manager(spawner)).unwrap();

        // Start the uart manager
        spawner.spawn(uart_manager(spawner)).unwrap();

        loop {
            wd_clear();
            main_loop();

            // Clear any uart errors if they've occurred
            UartDriver::uart_error_clr();

            // Let other tasks run
            yield_now().await;
        }
    }
}
