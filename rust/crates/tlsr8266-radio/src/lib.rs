//! The TLSR8266 radio: BLE link layer, GATT server, AES pairing, RF driver and
//! mesh link layer.
//!
//! # Boundary
//!
//! This crate sits directly above the HAL in the `pac <- hal <- radio <-
//! firmware` DAG. It may name `core`, `critical-section`, its sibling protocol
//! crates and the HAL, and nothing else: **it must never name the application
//! layer** (`app`, `config`, `version`, `ota`, `uart`, `mesh`, `light`,
//! `diagnostics`, `logging`). Everything the firmware owes the radio arrives
//! through the [`callbacks`] and [`queues`] seams, installed once at boot by the
//! composition root.
//!
//! Module map (normative, issue #72 / design 032 §3):
//!
//! - [`ble`] — link-layer connection, pairing, status and flash configuration
//! - [`gatt`] — attribute server and the light service table
//! - [`mesh_ll`] — mesh node status, packet processing and advertisements
//! - [`rf`] — the RF driver, its register tables and the register-level `rf_drv`
//!   helpers
//! - [`irq`] — the interrupt-facing state machines
//! - [`facade`] — the surface the firmware drives the radio through
//!
//! # Why `allow(warnings)`
//!
//! Temporary by design: the extraction lands the move unchanged so the diff is
//! reviewable, and C-16 (#78) removes this attribute together with the dead code
//! and doc-link breakage the move leaves behind.

#![feature(coverage_attribute)]
#![cfg_attr(target_os = "none", no_std)]
#![allow(warnings)]

pub mod ble {
    pub mod channel_selection;
    pub mod connection;
    pub mod flash_config;
    pub mod pair;
    pub mod pairing;
    pub mod status;
}

pub mod gatt {
    pub mod attribute;
    pub mod service_table;
}

pub mod mesh_ll {
    pub mod adv;
    pub mod node_status;
    pub mod packet_processing;
}

pub mod rf {
    pub mod drv_8266;
    pub mod tables;
}

pub mod callbacks;
pub mod config;
pub mod facade;
pub mod irq;
pub mod queues;
pub mod types;

/// The radio surface the firmware names: identity, pairing, link state and the
/// boot-time initialisation entry points.
pub use facade::*;
