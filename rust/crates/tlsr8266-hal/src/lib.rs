//! TLSR8266 hardware abstraction layer and async runtime.
//!
//! `mcu` holds the on-chip peripherals (GPIO, clocks, timers, DMA, analog,
//! watchdog, crypto), `common` the shared bit and critical-section helpers,
//! `pm` the power-management sequences, and `runtime` the embassy executor and
//! time driver. `tlsr8266-pac` is the only register-level dependency, and the
//! fake-MMIO host fakes compile under `cfg(not(target_os = "none"))` so the
//! same code is testable on the target and on the host.

#![feature(coverage_attribute)]
#![cfg_attr(target_os = "none", no_std)]

pub use tlsr8266_pac::{BIT, BIT_MASK_LEN, BIT_RNG};

pub mod common;
pub mod mcu;
pub mod pm;
pub mod runtime;
