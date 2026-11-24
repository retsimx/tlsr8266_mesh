#![feature(coverage_attribute)]
#![cfg_attr(not(test), no_std)]

#[cfg(target_os = "none")]
use core::panic::PanicInfo;
use core::sync::atomic::AtomicU32;
use tlsr8266_hal::pm::cpu_wakeup_init;
use tlsr8266_hal::runtime::executor::Executor;

mod app;
mod config;
mod diagnostics;
mod factory_reset;
mod irq;
mod light;
mod logging;
mod mesh;
mod ota;
#[cfg(target_os = "none")]
mod startup;
mod uart;
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

/// Panic handler: mark the reset, persist the message, reboot.
///
/// This lives in the composition root because it is the only place that may
/// name `diagnostics`, `logging` and the HAL at once without creating a cycle.
///
/// This is the emergency path, so the ordering is the contract: the retained
/// sentinel and breadcrumb are written *before* any formatting, because
/// formatting the panic info into the 256-byte stream is work that can itself
/// fail and a double panic must not cost the next boot its breadcrumb. The
/// message then goes out on the blocking sink (never the async channel) and to
/// flash, and only then does the device reboot.
#[cfg(target_os = "none")]
#[panic_handler]
pub fn panic(info: &PanicInfo) -> ! {
    critical_section::with(|_| {
        // Mark the panic before anything that can itself fail.
        diagnostics::record_panic_sentinel();

        let mut stream = logging::UartStream::<256>::new();

        let _ = core::fmt::Write::write_fmt(&mut stream, format_args!("{}", info));

        // The sink is named directly (not the registered sinks) so the panic
        // path keeps a direct call edge to it. One RTT mirror happens inside
        // `send_blocking_with`; do not mirror again.
        stream.send_blocking_with(true, uart::send_sync_panic);
        diagnostics::write_panic_bytes(stream.as_bytes());

        tlsr8266_hal::pm::light_sw_reboot();
    });

    #[allow(clippy::empty_loop)]
    loop {}
}

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
