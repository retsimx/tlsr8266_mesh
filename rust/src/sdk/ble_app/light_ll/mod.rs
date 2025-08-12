//! # Light Link Layer (light_ll) Module
//!
//! This module implements the core link layer functionality for a BLE mesh lighting system 
//! on the TLSR8266 chip. The light_ll module serves as the bridge between the low-level 
//! BLE radio hardware and the higher-level mesh networking protocol stack.
//!
//! ## Architecture Overview
//!
//! The module is organized into six main subsystems that work together to provide a complete
//! mesh networking solution:
//!
//! 1. **Connection Management**: Handles BLE peripheral connections, timing synchronization,
//!    and parameter updates
//! 2. **Mesh Management**: Manages mesh network topology, node discovery, and status propagation
//! 3. **Packet Processing**: Parses and routes incoming packets, handles retransmission and acknowledgments
//! 4. **Status Management**: Coordinates device status reporting and bridging between BLE and mesh
//! 5. **OTA Management**: Provides over-the-air firmware update capabilities
//! 6. **Pairing Management**: Handles device authentication and security key management
//!
//! ## Key Design Principles
//!
//! ### Multi-Protocol Support
//! The system operates simultaneously as:
//! - A BLE peripheral (for smartphone/gateway connections)
//! - A mesh node (for device-to-device communication)
//! - A bridge between these two networks
//!
//! ### Timing-Critical Operation
//! All mesh communication is precisely timed to ensure:
//! - Collision avoidance in the shared radio medium
//! - Reliable message delivery with minimal latency
//! - Power-efficient operation through synchronized sleep cycles
//!
//! ### Scalable Mesh Architecture
//! The mesh protocol supports:
//! - Up to 254 nodes per network (addresses 1-254, with 0 reserved for broadcast)
//! - Automatic relay and routing of messages
//! - Self-healing network topology
//! - Group addressing for synchronized control

/// Mesh network management functionality
///
/// Handles mesh node discovery, status tracking, and network topology maintenance.
/// Implements algorithms for reliable status propagation and automatic node timeout detection.
pub mod mesh_management;

/// BLE connection management functionality  
///
/// Manages BLE peripheral connections, including connection parameter negotiation,
/// timing synchronization, and adaptive parameter updates for optimal performance.
pub mod connection_management;

/// Packet processing and parsing functionality
///
/// Implements the core packet routing logic, including mesh relay algorithms,
/// duplicate detection, acknowledgment handling, and protocol parsing.
pub mod packet_processing;

/// Over-The-Air (OTA) update management
///
/// Provides secure firmware update capabilities over the mesh network,
/// with flash memory management and verification algorithms.
pub mod ota_management;

/// Device status and reporting management
///
/// Coordinates status reporting between BLE and mesh networks, implementing
/// bridge algorithms and response buffering for reliable data transfer.
pub mod status_management;

/// Pairing and security management
///
/// Handles device authentication, key management, and secure pairing protocols
/// for mesh network access control.
pub mod pairing_management;
