use core::ops::DerefMut;

use embassy_executor::Spawner;

use crate::{app, uprintln};
use crate::config::MESH_PWD_ENCODE_SK;
use crate::light_manager::LightManager;
use crate::main_light::{main_loop, user_init};
use crate::mesh::MeshManager;
use crate::ota::OtaManager;
use crate::sdk::ble_app::rf_drv_8266::rf_drv_init;
use crate::sdk::common::compat::check_panic_info;
use crate::sdk::drivers::uart::UartDriver;
use crate::sdk::light::*;
use crate::sdk::mcu::clock::clock_init;
use crate::sdk::mcu::dma::dma_init;
use crate::sdk::mcu::gpio::gpio_init;
use crate::sdk::mcu::irq_i::{irq_enable, irq_init};
use crate::sdk::mcu::watchdog::wd_clear;
use crate::state::{State, STATE};
use crate::uart_manager::UartManager;
use crate::version::BUILD_VERSION;

pub struct App {
    pub ota_manager: OtaManager,
    pub mesh_manager: MeshManager,
    pub light_manager: LightManager,
    pub uart_manager: UartManager
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

    pub fn init(&mut self, state: &mut State) {
        // Copy the password in to the pair config
        for i in 0..MESH_PWD_ENCODE_SK.len() {
            state.pair_config_pwd_encode_sk[i] = MESH_PWD_ENCODE_SK.as_bytes()[i];
        }

        unsafe { rf_drv_init(state, true); }

        // Run our initialisation
        user_init(state);
    }

    pub async fn run(&mut self, spawner: Spawner) {
        // Initialise various subsystems needed by uart
        clock_init();
        dma_init();
        gpio_init();
        irq_init();

        // Get uart ready to go early
        self.uart_manager.init();

        uprintln!("Booting FW version {}", BUILD_VERSION);

        // Configure the rest of the system
        STATE.lock(|state| {
            let mut binding = state.borrow_mut();
            let state = binding.deref_mut();

            self.init(state);
        });

        // Start the mesh packet sender
        #[embassy_executor::task]
        async fn mesh_packet_sender() {
            app().mesh_manager.send_mesh_msg_task().await;
        }

        spawner.spawn(mesh_packet_sender()).unwrap();

        // Start the mesh packet receive handler
        #[embassy_executor::task]
        async fn mesh_packet_rcv() {
            app().mesh_manager.rcv_mesh_msg_task().await;
        }

        spawner.spawn(mesh_packet_rcv()).unwrap();

        // Ready to go, enable interrupts and run the main loop
        irq_enable();

        // Start the light manager
        #[embassy_executor::task]
        async fn light_manager(spawner: Spawner) {
            app().light_manager.run(spawner).await;
        }

        spawner.spawn(light_manager(spawner)).unwrap();

        // Send a message to the network saying that we just booted up
        let mut data = [0 as u8; 13];
        data[0] = LGT_POWER_ON;

        STATE.lock(|state| {
            let mut binding = state.borrow_mut();
            let state = binding.deref_mut();

            app().mesh_manager.send_mesh_message(state, &data, 0xffff);
        });

        // Start the panic checker to see if there is information we need to send
        #[embassy_executor::task]
        async fn panic_check(_spawner: Spawner) {
            check_panic_info().await;
        }

        spawner.spawn(panic_check(spawner)).unwrap();

        loop {
            // Clear the watchdog timer
            wd_clear();

            // Run the main loop - this only runs once per LOOP_INTERVAL_US
            main_loop().await;

            // Clear any uart errors if they've occurred
            UartDriver::uart_error_clr();
        }
    }
}
