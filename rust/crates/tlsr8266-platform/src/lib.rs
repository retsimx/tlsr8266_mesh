#![cfg_attr(target_os = "none", no_std)]

pub mod api;
pub mod coex;
pub mod config;
pub mod gatt;
pub mod ota;
pub mod tasks;
pub mod window;

pub use tlsr8266_radio::{
    AttError, CompiledAttr, GattBackend, GattReadOutcome, GattReadRequest, GattServer,
    GattWriteOutcome, GattWriteRequest, ProfileView,
};
