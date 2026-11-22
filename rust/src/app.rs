#![coverage(off)]
use core::sync::atomic::Ordering;

use embassy_executor::Spawner;

use crate::config::MESH_PWD_ENCODE_SK;
use crate::diagnostics::{
    boot_init, check_panic_info, record_breadcrumb, CRUMB_INIT_DONE, CRUMB_MAIN_LOOP,
};
use crate::main_light::{main_loop, user_init};
use crate::sdk::ble_app::callbacks::{set_ble_app_hooks, BleAppHooks};
use crate::sdk::ble_app::rf_drv_8266::rf_drv_init;
use crate::sdk::light::*;
use crate::sdk::mcu::clock::{clock_init, is_16mhz_xtal};
use crate::sdk::mcu::dma::dma_init;
use crate::sdk::mcu::gpio::gpio_init;
use crate::sdk::mcu::irq_i::{irq_enable, irq_init};
use crate::sdk::mcu::watchdog::wd_clear;
use crate::state::PAIR_CONFIG_PWD_ENCODE_SK;
use crate::uart_manager::{uart_receiver, uart_sender};
use crate::uprintln;
use crate::version::BUILD_VERSION;
use heapless::Vec;

/// The application's single root task.
///
/// Boot order is normative (design 018 §5): OTA recovery runs in
/// `main_entrypoint` before any clock or IRQ exists, then this task brings the
/// peripherals up, registers every SDK seam **before** `rf_drv_init(true)`
/// populates the radio, and only then spawns the per-owner tasks and enters the
/// main loop.
#[embassy_executor::task]
pub async fn main_task(spawner: Spawner) {
    // Initialise various subsystems needed by uart
    clock_init();
    dma_init();
    gpio_init();
    irq_init();

    // Get uart ready to go early
    crate::uart_manager::init();

    // Report why the previous boot ended and mark this one as running
    boot_init();

    uprintln!(
        "Booting FW version {}, xtal: {}MHz",
        BUILD_VERSION,
        if is_16mhz_xtal() { 16 } else { 12 }
    );

    // Wire the SDK's callback seams to the application before any RF or pairing
    // path can run, so neither slot is ever observed empty.
    crate::sdk::app_att_light::set_ota_write_handler(crate::ota::rf_link_slave_data_ota);
    crate::sdk::rf_drv::set_light_event_handler(crate::main_light::rf_link_light_event_callback);
    set_ble_app_hooks(BleAppHooks {
        on_mesh_pair_proc: mesh_pair_proc_hook,
        on_light_data: crate::main_light::rf_link_data_callback,
        on_light_response: crate::main_light::rf_link_response_callback,
        on_uart_mesh_rx: crate::uart_manager::light_mesh_rx_cb,
    });

    // Copy the password in to the pair config
    PAIR_CONFIG_PWD_ENCODE_SK.lock()[0..MESH_PWD_ENCODE_SK.len()]
        .copy_from_slice(&MESH_PWD_ENCODE_SK.as_bytes()[0..MESH_PWD_ENCODE_SK.len()]);

    rf_drv_init(true);

    // Run our initialisation; the lamp's write pointer lives here from now on
    let mut flash_store = user_init();
    record_breadcrumb(CRUMB_INIT_DONE);

    // Start the mesh packet sender / receive handler
    spawner.spawn(crate::mesh::mesh_packet_sender().unwrap());
    spawner.spawn(crate::mesh::mesh_packet_rcv().unwrap());

    // Start the UART tasks. `uart_sender` parks on `started_signal` until the
    // first valid host frame arrives, so a headless light never retries DMA TX
    // for the boot banners.
    spawner.spawn(uart_receiver().unwrap());
    spawner.spawn(uart_sender().unwrap());

    // Start the light command consumer
    spawner.spawn(light_manager().unwrap());

    // Start the panic checker to see if there is information we need to send
    spawner.spawn(panic_check().unwrap());

    // Ready to go, enable interrupts and run the main loop
    irq_enable();
    record_breadcrumb(CRUMB_MAIN_LOOP);

    // Send a message to the network saying that we just booted up
    let mut data = [0_u8; 13];
    data[0] = LGT_POWER_ON;

    crate::mesh::send_mesh_message(&Vec::from_slice(&data).unwrap(), 0xffff, 3, false);

    loop {
        // Prove to a host that the loop is still turning (`fetch_add` does
        // not exist on thumbv6m)
        let next = crate::HEARTBEAT.load(Ordering::Relaxed).wrapping_add(1);
        crate::HEARTBEAT.store(next, Ordering::Relaxed);

        // Clear the watchdog timer
        wd_clear();

        // Run the main loop - this only runs once per LOOP_INTERVAL_US
        main_loop(&mut flash_store).await;

        // Clear any uart errors if they've occurred
        crate::uart_manager::clear_error();
    }
}

/// Task wrapper for the light command queue consumer.
///
/// `light_manager_task` is a plain `async fn` so the queue owner stays testable
/// without an executor; this shim gives it a task slot.
#[embassy_executor::task]
async fn light_manager() {
    crate::light_manager::light_manager_task().await;
}

/// Task wrapper for the one-shot panic-info report.
#[embassy_executor::task]
async fn panic_check() {
    check_panic_info().await;
}

/// Adapter that gives the mesh pairing seam its own symbol in the linked ELF.
///
/// `mesh::mesh_pair_proc` and this wrapper have indistinguishable prologues, so
/// `sdk/stack_analysis.py`'s `INDIRECT_EDGES` matches the edge by symbol name.
/// Both this function and `mesh::mesh_pair_proc` are `#[inline(never)]` so the
/// symbol the analyser looks for actually survives `OPT_INLINE_THRESHOLD`.
#[inline(never)]
fn mesh_pair_proc_hook() {
    crate::mesh::mesh_pair_proc();
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    /// `main_task` is the composition root for the SDK callback seams, but it
    /// cannot be executed on the host: `rf_drv_init` and `user_init` are not
    /// `mry`-mockable and both touch hardware registers, and the slots they fill
    /// are private statics owned by `sdk::app_att_light`, `sdk::rf_drv` and
    /// `sdk::ble_app::callbacks`.
    ///
    /// What can be pinned here is the structural invariant that makes the wiring
    /// safe: every registration lives in `main_task`'s body and precedes
    /// `rf_drv_init(true)`, so an accidental reorder or deletion fails this test.
    /// That the slots are populated afterwards is proven on target by the HIL
    /// acceptance run (boot, pairing LED flash, OTA).
    #[test]
    fn test_init_registers_both_sdk_handlers_before_rf_drv_init() {
        let src = include_str!("app.rs");

        let task_start = src
            .find("pub async fn main_task(spawner: Spawner)")
            .expect("main_task must exist");
        // Search only the task body: the tail of the file includes this test,
        // whose own string literals would otherwise satisfy the needles below.
        let task_tail = &src[task_start..];
        let task_body = &task_tail[..task_tail
            .find("\n}\n")
            .expect("main_task must be a well-formed task body")];

        let ota = task_body
            .find("set_ota_write_handler(crate::ota::rf_link_slave_data_ota)")
            .expect("main_task must register the OTA write handler");
        let light = task_body
            .find("set_light_event_handler(")
            .expect("main_task must register the light event handler");
        let hooks = task_body
            .find("set_ble_app_hooks(BleAppHooks {")
            .expect("main_task must register the BLE app hooks");
        let rf_drv = task_body
            .find("rf_drv_init(true)")
            .expect("main_task must initialise the RF driver");

        assert!(
            ota < rf_drv && light < rf_drv && hooks < rf_drv,
            "every handler registration must precede rf_drv_init(true)"
        );
    }
}
