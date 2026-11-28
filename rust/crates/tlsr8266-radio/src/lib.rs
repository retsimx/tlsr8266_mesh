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

#![feature(coverage_attribute)]
#![cfg_attr(target_os = "none", no_std)]
// The project mandates `Result<_, ()>` for fallible hardware/protocol calls
// (docs/checklists/firmware.md); a richer error type would invent information
// these call sites do not have.
#![allow(clippy::result_unit_err)]

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
pub mod coex;
pub mod config;

pub use coex::{
    AdvertisementSlot, AttTxPacket, AuxiliaryRfPlan, ConnectionPlan, ConnectionSlot,
    FrameIngressTag, LinkAction, LinkEvent, LinkIdentity, ListenSlot, QueueFull, RadioAction,
    RadioCoexistence, RadioFrame, RxContext, StatusEnqueueOutcome,
};

pub use gatt::attribute::{l2cap_att_handler_with, AttRequest, AttResponse, GattOp, GattServer};
pub use gatt::service_table::{
    AttError, AttrPermissions, AttrRole, AttrUuid, CompiledAttr, GattBackend, GattReadKind,
    GattReadOutcome, GattReadRequest, GattWriteKind, GattWriteOutcome, GattWriteRequest,
    LegacyBinding, LegacyGattBackend, ProfileView, ValueLength, LEGACY_PROFILE,
    LEGACY_PROFILE_ATTRS,
};
pub mod facade;
pub mod irq;
pub mod queues;
pub mod types;

/// The radio surface the firmware names: identity, pairing, link state and the
/// boot-time initialisation entry points.
pub use facade::*;
