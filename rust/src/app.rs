#![coverage(off)]
use core::sync::atomic::Ordering;

use embassy_executor::Spawner;

use crate::config::MESH_PWD_ENCODE_SK;
use crate::diagnostics::{boot_init, record_breadcrumb, CRUMB_INIT_DONE, CRUMB_MAIN_LOOP};
use crate::light_manager::LightManager;
use crate::main_light::{main_loop, user_init};
use crate::mesh::MeshManager;
use crate::ota::OtaManager;
use crate::sdk::ble_app::rf_drv_8266::rf_drv_init;
use crate::sdk::common::compat::check_panic_info;
use crate::sdk::drivers::uart::UartDriver;
use crate::sdk::light::*;
use crate::sdk::mcu::clock::{clock_init, is_16mhz_xtal};
use crate::sdk::mcu::dma::dma_init;
use crate::sdk::mcu::gpio::gpio_init;
use crate::sdk::mcu::irq_i::{irq_enable, irq_init};
use crate::sdk::mcu::watchdog::wd_clear;
use crate::state::PAIR_CONFIG_PWD_ENCODE_SK;
use crate::uart_manager::{uart_receiver, uart_sender, UartManager};
use crate::version::BUILD_VERSION;
use crate::{app, uprintln};
use heapless::Vec;

#[cfg_attr(test, mry::mry)]
pub struct App {
    pub ota_manager: OtaManager,
    pub mesh_manager: MeshManager,
    pub light_manager: LightManager,
    pub uart_manager: UartManager,
}

#[cfg_attr(test, mry::mry(skip_fns(default_const, run)))]
impl App {
    #[cfg(not(test))]
    pub const fn default_const() -> App {
        App {
            ota_manager: OtaManager::default_const(),
            mesh_manager: MeshManager::default_const(),
            light_manager: LightManager::default_const(),
            uart_manager: UartManager::default_const(),
        }
    }

    #[cfg(test)]
    pub fn default() -> App {
        App {
            ota_manager: OtaManager::default(),
            mesh_manager: MeshManager::default(),
            light_manager: LightManager::default(),
            uart_manager: UartManager::default(),
            mry: Default::default(),
        }
    }

    pub fn init(&mut self) {
        // Copy the password in to the pair config
        PAIR_CONFIG_PWD_ENCODE_SK.lock()[0..MESH_PWD_ENCODE_SK.len()]
            .copy_from_slice(&MESH_PWD_ENCODE_SK.as_bytes()[0..MESH_PWD_ENCODE_SK.len()]);

        rf_drv_init(true);

        // Run our initialisation
        user_init();
    }

    pub async fn run(&mut self, spawner: Spawner) {
        // Initialise various subsystems needed by uart
        clock_init();
        dma_init();
        gpio_init();
        irq_init();

        // Get uart ready to go early
        self.uart_manager.init();

        // Report why the previous boot ended and mark this one as running
        boot_init();

        uprintln!(
            "Booting FW version {}, xtal: {}MHz",
            BUILD_VERSION,
            if is_16mhz_xtal() { 16 } else { 12 }
        );

        // Configure the rest of the system
        self.init();
        record_breadcrumb(CRUMB_INIT_DONE);

        // Start the mesh packet sender
        #[embassy_executor::task]
        async fn mesh_packet_sender() {
            app().mesh_manager.send_mesh_msg_task().await;
        }

        spawner.spawn(mesh_packet_sender().unwrap());

        // Start the mesh packet receive handler
        #[embassy_executor::task]
        async fn mesh_packet_rcv() {
            app().mesh_manager.rcv_mesh_msg_task().await;
        }

        spawner.spawn(mesh_packet_rcv().unwrap());

        // Start the UART tasks. `uart_sender` parks on `started_signal` until the
        // first valid host frame arrives, so a headless light never retries DMA TX
        // for the boot banners.
        spawner.spawn(uart_receiver().unwrap());
        spawner.spawn(uart_sender().unwrap());

        // Ready to go, enable interrupts and run the main loop
        irq_enable();
        record_breadcrumb(CRUMB_MAIN_LOOP);

        // Start the light manager
        #[embassy_executor::task]
        async fn light_manager(spawner: Spawner) {
            app().light_manager.run(spawner).await;
        }

        spawner.spawn(light_manager(spawner).unwrap());

        // Send a message to the network saying that we just booted up
        let mut data = [0_u8; 13];
        data[0] = LGT_POWER_ON;

        app()
            .mesh_manager
            .send_mesh_message(&Vec::from_slice(&data).unwrap(), 0xffff, 3, false);

        // Start the panic checker to see if there is information we need to send
        #[embassy_executor::task]
        async fn panic_check(_spawner: Spawner) {
            check_panic_info().await;
        }

        spawner.spawn(panic_check(spawner).unwrap());

        loop {
            // Prove to a host that the loop is still turning (`fetch_add` does
            // not exist on thumbv6m)
            let next = crate::HEARTBEAT.load(Ordering::Relaxed).wrapping_add(1);
            crate::HEARTBEAT.store(next, Ordering::Relaxed);

            // Clear the watchdog timer
            wd_clear();

            // Run the main loop - this only runs once per LOOP_INTERVAL_US
            main_loop().await;

            // Clear any uart errors if they've occurred
            UartDriver::clear_error();
        }
    }
}
