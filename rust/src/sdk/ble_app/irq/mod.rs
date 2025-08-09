//! # BLE Link Layer IRQ Handler Module Collection
//!
//! This module collection implements the Bluetooth Low Energy (BLE) link layer interrupt handling
//! for a TLSR8266-based mesh lighting system. The functionality has been decomposed into
//! focused, testable modules for better maintainability.
//!
//! ## Module Organization:
//! - `connection`: BLE connection parameter updates and state management
//! - `mesh`: Mesh networking, node status reporting, and discovery
//! - `advertisement`: BLE advertisement and scan response handling
//! - `packet_handling`: RF packet reception, validation, and processing
//! - `timing`: BLE timing, synchronization, and channel hopping
//! - `dispatcher`: Main interrupt dispatcher and context tracking
//! - `tests`: Comprehensive test suite for all functionality
//!
//! ## Key Responsibilities:
//! - BLE connection states, timing synchronization, mesh packet processing
//! - Advertisement and scan response handling, connection state management
//! - OTA updates, RF channel hopping and access code management
//!
//! ## BLE Specification Compliance:
//! This implementation follows BLE 4.0+ specifications for:
//! - Connection parameter update procedures (LL_CONNECTION_UPDATE_IND)
//! - Channel map update procedures (LL_CHANNEL_MAP_IND)
//! - Connection event timing and supervision timeouts
//! - Frequency hopping sequence management

pub mod connection;
pub mod mesh;
pub mod advertisement;
pub mod packet_handling;
pub mod timing;
pub mod dispatcher;



// Re-export the main interrupt handler and key functions for compatibility
pub use dispatcher::{irq_handler, IrqTracker};
pub use mesh::mesh_node_report_status;

// Re-export state management functions used by other modules
pub use connection::{handle_ble_connection_parameter_updates, cleanup_ble_disconnection, handle_ble_connected_state, process_queued_status_responses};
pub use mesh::{configure_rf_for_mesh_listening, handle_mesh_listening_state};
pub use advertisement::{get_ble_advertisement_channel_count, handle_ble_advertisement_state};
pub use packet_handling::{handle_rf_transmission_complete, handle_rf_packet_reception};
pub use timing::configure_ble_receive_state;
