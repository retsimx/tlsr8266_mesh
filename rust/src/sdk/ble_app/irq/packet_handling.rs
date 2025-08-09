//! # Packet Reception and Processing Module
//!
//! This module handles RF packet reception, validation, processing, and buffer management
//! for the TLSR8266-based mesh lighting system.
//!
//! ## Key Responsibilities:
//! - RF packet reception interrupt handling
//! - Packet validation and integrity checking
//! - Buffer management and circular buffer operations
//! - BLE and mesh packet differentiation and processing
//! - Scan request and connection request handling

use core::ptr::addr_of;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::config::VENDOR_ID;
use crate::sdk::ble_app::ble_ll_pair::pair_dec_packet_mesh;
use crate::sdk::ble_app::light_ll::{*};
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::light::{LIGHT_RX_BUFF_COUNT, LightRxBuff, AdvRspPrivate};
use crate::sdk::mcu::clock::CLOCK_SYS_CLOCK_1US;
use crate::sdk::mcu::register::{*};
use crate::sdk::packet_types::{Packet, PacketScanRsp, ScanRspData};
use crate::state::{*};
use crate::{app};

/// Handles RF transmission complete interrupt.
///
/// This function is called when an RF transmission has completed.
/// It simply clears the RF interrupt status to acknowledge the transmission.
///
/// ## Interrupt Handling:
/// - Clears bit 2 of RF interrupt status (TX complete)
/// - Allows the system to proceed with next operations
/// - Minimal processing to maintain real-time performance
pub fn handle_rf_transmission_complete()
{
    // Clear RF interrupt status bit 2 (transmission complete)
    write_reg_rf_irq_status(2);
}

/// Handles RF packet reception interrupt.
///
/// This is the main packet reception handler that processes incoming RF packets.
/// It manages the circular receive buffer, validates packets, handles mesh and BLE
/// packets differently, and processes connection requests and scan requests.
///
/// ## Buffer Management:
/// - Uses circular buffer with atomic read/write pointers
/// - Handles buffer overflow and invalid packets gracefully
/// - Manages DMA addressing for next reception
///
/// ## Packet Processing:
/// - Validates packet integrity and format
/// - Distinguishes between mesh and BLE connection packets
/// - Handles advertisement responses and connection requests
/// - Processes mesh network packets and status updates
///
/// ## Real-time Performance:
/// - Marked `#[inline(always)]` for maximum performance
/// - Minimal processing in fast path
/// - Delegates complex processing to separate function
#[inline(always)]
pub fn handle_rf_packet_reception()
{
    // Static variable to track last received packet time (duplicate detection)
    static T_RX_LAST: AtomicU32 = AtomicU32::new(0);

    // Get current receive buffer index and advance write pointer
    let rx_index = LIGHT_RX_BUFFER_WRITE_POINTER.get();
    LIGHT_RX_BUFFER_WRITE_POINTER.set((rx_index + 1) % LIGHT_RX_BUFF_COUNT);

    // Check RF receive status - 0x0b indicates reception error
    if read_reg_rf_rx_status() == 0x0b {
        // Clear RF interrupt and return on error
        write_reg_rf_irq_status(1);
        return;
    }

    let mut dma_len = 0;
    let mut light_rx_buff = LIGHT_RX_BUFF.lock();

    // Configure DMA for next reception using the new write pointer
    write_reg_dma2_addr(addr_of!(light_rx_buff[LIGHT_RX_BUFFER_WRITE_POINTER.get()]) as u16);
    
    // Clear RF interrupt status to acknowledge reception
    write_reg_rf_irq_status(1);

    // Extract timing and length information from received packet
    let rx_time = light_rx_buff[rx_index].rx_time;
    dma_len = light_rx_buff[rx_index].dma_len;

    // Mark this buffer entry as processed by setting dma_len to 1
    light_rx_buff[rx_index].dma_len = 1;

    // Handle invalid or duplicate packets
    if dma_len == 1 {
        // Check if this is a duplicate packet (same timestamp)
        if T_RX_LAST.load(Ordering::Relaxed) == rx_time {
            // Duplicate detected: restart reception
            rf_stop_trx();
            rf_start_stx2rx(addr_of!(PKT_EMPTY) as u32, CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick());
            return;
        }

        // Invalid packet: ignore and continue
        return;
    }

    // Update last received packet timestamp
    T_RX_LAST.store(rx_time, Ordering::Relaxed);

    // Delegate complex packet processing to the slow path function
    // This separation keeps the interrupt handler fast path minimal while providing
    // comprehensive packet processing capabilities in the slow path
    process_received_packet_slow_path(rx_index, dma_len, &mut *light_rx_buff);
}

/// Complex packet processing function (marked inline(never) for code size optimization).
///
/// This function handles the detailed processing of received packets including:
/// - Packet validation and integrity checking
/// - BLE advertisement and connection request handling
/// - Mesh network packet processing and forwarding
/// - Connection timing and synchronization
/// - Scan response generation
#[inline(never)]
fn process_received_packet_slow_path(rx_index: usize, dma_len: u8, light_rx_buff: &mut [LightRxBuff; 4]) {
    let entry = &light_rx_buff[rx_index];
    let rx_time = entry.rx_time;

    // Comprehensive packet validation with multiple integrity checks:
    // 1. dma_len > 0xe (14): Minimum packet size for valid BLE/mesh packets
    // 2. dma_len == (entry.sno[1] & 0x3f) + 0x11: Length field consistency check
    //    - entry.sno[1] contains the payload length in lower 6 bits
    //    - Add 0x11 (17) for header overhead to get total expected length
    // 3. Status byte validation: Check RF status at end of packet
    //    - Address calculation: packet_start + packet_length + 3 bytes offset
    //    - Status mask 0x51 checked against expected value 0x40
    //    - This verifies successful RF reception without errors
    if dma_len > 0xe && dma_len == (entry.sno[1] & 0x3f) + 0x11 && unsafe { *((addr_of!(*entry) as u32 + dma_len as u32 + 3) as *const u8) } & 0x51 == 0x40 {
        // Cast receive buffer to packet structure for processing
        let packet = unsafe { &*(addr_of!(entry.rx_time) as *const Packet) };

        // Extract command type from packet header (lower 4 bits of first sequence byte)
        // Command types: 3 = scan request, 5 = connection request, others = data/control
        let cmd = entry.sno[0] & 0xf;
        
        // Store packet reception timestamp for timing calculations and debugging
        LAST_PACKET_RECEIVED_TIMESTAMP.set(rx_time);
        
        // Handle packets when in advertisement state
        if BLE_PERIPHERAL_LINK_STATE.get() == crate::sdk::light::BlePeripheralLinkState::Advertising {
            // Command 3: BLE Scan Request - Generate and send scan response
            if cmd == 3 {
                handle_scan_request(entry, rx_time);
                return;
            }

            // Command 5: BLE Connection Request - Initiate connection establishment
            if cmd == 5 {
                handle_connection_request(entry, packet, rx_time);
                return;
            }
        }

        // Process mesh network packets (when not in OTA mode and not in active BLE RX state)
        if !OTA_UPDATE_IN_PROGRESS.get() && BLE_PERIPHERAL_LINK_STATE.get() != crate::sdk::light::BlePeripheralLinkState::Receiving {
            handle_mesh_packet(packet, rx_time);
            return;
        }

        // Process BLE connection data packets
        handle_ble_connection_data(entry, packet, rx_time);
        return;
    }

    // Handle cleanup when in BLE receive state but packet was invalid
    if BLE_PERIPHERAL_LINK_STATE.get() == crate::sdk::light::BlePeripheralLinkState::Receiving {
        // Clear RF interrupt status and stop radio operations
        // This handles the case where we were expecting a BLE packet but received invalid data
        write_reg8(0x80050f, 0);
        rf_stop_trx();
    }
}

/// Handles BLE scan request packets and generates scan responses.
///
/// This function processes scan requests addressed to this device and generates
/// appropriate scan response packets containing device information.
fn handle_scan_request(entry: &LightRxBuff, rx_time: u32) {
    // Verify the scan request is addressed to this device by checking MAC address
    if entry.mac == MAC_ID.lock()[0..4] {
        // Stop current radio operations to prepare for response transmission
        rf_stop_trx();
        
        // Schedule the scan response transmission after the required interval
        // BLE_SCAN_RESPONSE_INTERVAL_US defines the BLE-mandated delay before responding
        write_reg_rf_sched_tick(rx_time + BLE_SCAN_RESPONSE_INTERVAL_US.get() * CLOCK_SYS_CLOCK_1US);
        
        // Configure RF for single transmission mode (0x85)
        write_reg_rf_mode_control(0x85);

        // Construct BLE scan response packet with device information
        let pkt_scan_rsp = Packet {
            scan_rsp: PacketScanRsp {
                dma_len: 0x27,                    // DMA transfer length (39 bytes)
                _type: 0x4,                       // BLE scan response packet type
                rf_len: 0x25,                     // RF payload length (37 bytes)
                adv_a: *MAC_ID.lock(),            // Advertiser's MAC address
                data: ScanRspData {
                    handle: 0xff1e,               // Mesh-specific handle identifier
                    data: AdvRspPrivate {
                        device_address: DEVICE_ADDRESS.get(),  // Device's mesh address
                        ..*ADV_RSP_PRI_DATA.lock()             // Additional device-specific data
                    }
                },
            }
        };

        // Configure DMA to transmit the scan response packet
        write_reg_dma3_addr(addr_of!(pkt_scan_rsp) as u16);
        
        // Schedule next interrupt for 1ms to return to normal operation
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 1000 + read_reg_system_tick_irq());
    }
}

/// Handles BLE connection request packets.
///
/// This function processes connection requests addressed to this device and
/// initiates the BLE connection establishment procedure.
fn handle_connection_request(entry: &LightRxBuff, packet: &Packet, rx_time: u32) {
    // Verify the connection request is addressed to this device
    if entry.mac == MAC_ID.lock()[0..4] {
        // Process the connection request and establish BLE connection
        // This handles connection parameter negotiation and state setup
        rf_link_slave_connect(packet, rx_time);
    }
}

/// Handles mesh network packet processing.
///
/// This function validates and processes mesh network packets, including
/// decryption, duplicate detection, and forwarding to the mesh manager.
fn handle_mesh_packet(packet: &Packet, rx_time: u32) {
    // Create a mutable copy of the packet for decryption and processing
    let mut packet = *packet;

    /// Validates mesh packet integrity and filters duplicates.
    ///
    /// This closure performs comprehensive packet validation including:
    /// - Timing validation for connected devices
    /// - Packet format validation (length, type, channel ID)
    /// - Mesh packet decryption and authentication
    /// - Duplicate packet detection
    /// - Command parsing and opcode extraction
    let mut pkt_valid = || {
        // For connected devices, validate timing to prevent stale packet processing
        if BLE_PERIPHERAL_CONNECTION_ACTIVE.get() {
            // Check if the interrupt timing is too far in the future (>1ms + large offset)
            // This prevents processing packets that arrived too late in the connection window
            if 0x3fffffffi32 < (read_reg_system_tick_irq() as i32 - read_reg_system_tick() as i32) - (CLOCK_SYS_CLOCK_1US * 1000) as i32 {
                return false;
            }
        }

        // Validate mesh packet format:
        // - rf_len must be 0x25 (37 bytes) - standard mesh packet size
        // - l2cap_len must be 0x21 (33 bytes) - L2CAP payload size
        // - type must be 2 (L2CAP data packet)
        // - chan_id must not be 0xeeff (reserved/invalid channel)
        // - packet must pass mesh decryption/authentication
        if packet.head().rf_len != 0x25 || packet.head().l2cap_len != 0x21 || packet.head()._type & 3 != 2 || packet.head().chan_id == 0xeeff || !pair_dec_packet_mesh(&mut packet) {
            return false;
        }

        // Special case: Node status advertisement packets (channel ID 0xffff)
        // These packets broadcast node status information across the mesh
        if packet.head().chan_id == 0xffff {
            return true;
        }

        // Parse the mesh command opcode and parameters from the decrypted packet
        let (success, op_cmd, op_cmd_len, params, params_len) = parse_ble_packet_op_params(&packet, true);
        if !success {
            return false;
        }

        // Extract the primary opcode from the command
        let mut op = 0;
        if op_cmd_len == 3 {
            // Use lower 6 bits of first command byte as opcode
            op = op_cmd[0] & 0x3f;
        }

        // Check for duplicate packets using opcode and packet content
        // This prevents processing the same mesh command multiple times
        if is_exist_in_rc_pkt_buf(op, &packet) {
            return false;
        }

        true
    };

    // If packet validation passes, forward it to the mesh manager for processing
    if pkt_valid() {
        app().mesh_manager.add_rcv_mesh_msg(&packet);
    }
}

/// Handles BLE connection data packets.
///
/// This function processes data packets received during an active BLE connection,
/// including timing adjustment and connection state management.
fn handle_ble_connection_data(entry: &LightRxBuff, packet: &Packet, rx_time: u32) {
    // Extract master sequence number from packet header for connection tracking
    let master_sn = ((entry.sno[2] as u16) * 0x100) | ((entry.sno[0] >> 3) & 1) as u16;
    
    // Check if this is a packet from the current connection session
    if LIGHT_CONN_SN_MASTER.get() == master_sn {
        // Same connection session: adjust timing based on packet arrival
        // This maintains synchronization with the master device
        rf_link_timing_adjust(rx_time);
    } else {
        // New connection session or first packet from master:
        // Update connection tracking and process the data
        
        // Store the new master sequence number for future comparison
        LIGHT_CONN_SN_MASTER.set(master_sn);
        
        // Update connection establishment timestamp
        SLAVE_CONNECTED_TICK.set(read_reg_system_tick());
        
        // Mark the device as connected to a BLE master
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);

        // Process the connection data packet (may contain connection parameters,
        // L2CAP data, or other BLE protocol information)
        rf_link_slave_data(packet, rx_time);
    }

    // Handle connection window size management for timing synchronization
    if SLAVE_WINDOW_SIZE.get() != 0 {
        // Check if connection parameter update timing is in progress
        if BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.get() {
            // Verify that the timing update window has passed
            // 0x40000001 is a large value check to handle timer wraparound
            if 0x40000001 > BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.get() - read_reg_system_tick() {
                return;
            }

            // Clear the timing update flag as the window has completed
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.set(false);
        }
        
        // Reset window size to 0 indicating we've received a packet in this window
        // This closes the reception window for this connection event
        SLAVE_WINDOW_SIZE.set(0);

        // Calculate next connection event timing:
        // Use packet arrival time + connection interval - 1.25ms (standard BLE offset)
        // This maintains precise timing for the next connection event
        SLAVE_NEXT_CONNECT_TICK.set(rx_time + SLAVE_LINK_INTERVAL.get() - CLOCK_SYS_CLOCK_1US * 1250);
    }
}
