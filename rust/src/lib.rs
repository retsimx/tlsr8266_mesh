#![feature(coverage_attribute)]
#![cfg_attr(not(test), no_std)]

use core::sync::atomic::AtomicU32;
use tlsr8266_hal::pm::cpu_wakeup_init;
use tlsr8266_hal::runtime::executor::Executor;

mod app;
mod config;
mod diagnostics;
mod irq;
mod light_manager;
mod logging;
mod main_light;
mod mesh;
mod ota;
mod sdk;
#[cfg(target_os = "none")]
mod startup;
mod uart_manager;
mod vendor_light;
pub mod version;

/// Bit-manipulation macros live in the PAC crate; re-exported here so existing
/// `use crate::{BIT, ...}` paths keep resolving.
pub use tlsr8266_pac::{BIT, BIT_MASK_LEN, BIT_RNG};

#[cfg(test)]
mod wire_contract_tests;

/// Storage for the single executor driven by `main_entrypoint`.
static EXECUTOR: static_cell::StaticCell<Executor> = static_cell::StaticCell::new();

/// Bumped once per main-loop pass so a host can tell "alive" from "hung".
#[no_mangle]
pub static HEARTBEAT: AtomicU32 = AtomicU32::new(0);

// no_mangle because this is referenced as an entrypoint from the assembler bootstrap
#[no_mangle]
#[coverage(off)]
pub extern "C" fn main_entrypoint() {
    // Must happen first
    ota::handle_ota_update();

    // Get the CPU configured
    cpu_wakeup_init();

    // Run the application
    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(app::main_task(spawner).unwrap());
    });
}
