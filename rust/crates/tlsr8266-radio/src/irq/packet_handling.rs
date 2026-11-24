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

use crate::ble::connection::{
    rf_link_slave_connect, rf_link_timing_adjust, BLE_PERIPHERAL_CONNECTION_ACTIVE,
    BLE_PERIPHERAL_LINK_STATE, BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG,
    BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME, LAST_PACKET_RECEIVED_TIMESTAMP,
    LIGHT_CONN_SN_MASTER, SLAVE_CONNECTED_TICK, SLAVE_LINK_INTERVAL, SLAVE_NEXT_CONNECT_TICK,
    SLAVE_WINDOW_SIZE, SLAVE_WINDOW_SIZE_UPDATE,
};
use crate::ble::pair::pair_dec_packet_mesh;
use crate::callbacks::dispatch_ota_snapshot;
use crate::mesh_ll::packet_processing::{
    is_exist_in_rc_pkt_buf, parse_ble_packet_op_params, rf_link_slave_data,
};
use crate::rf::drv_8266::*;
use crate::rf::drv_8266::{
    ADV_RSP_PRI_DATA, BLE_SCAN_RESPONSE_INTERVAL_US, DEVICE_ADDRESS, LIGHT_RX_BUFF,
    LIGHT_RX_BUFFER_WRITE_POINTER, MAC_ID, PKT_EMPTY,
};
use crate::types::{LightRxBuff, LIGHT_RX_BUFF_COUNT};
use mesh_protocol::packet_types::{AdvRspPrivate, Packet, PacketScanRsp, ScanRspData};
use tlsr8266_hal::common::sync::SimplifyLS;
use tlsr8266_hal::mcu::clock::CLOCK_SYS_CLOCK_1US;
use tlsr8266_pac::*;

/// Handles RF transmission complete interrupt.
///
/// This function is called when an RF transmission has completed.
/// It simply clears the RF interrupt status to acknowledge the transmission.
///
/// ## Interrupt Handling:
/// - Clears bit 2 of RF interrupt status (TX complete)
/// - Allows the system to proceed with next operations
/// - Minimal processing to maintain real-time performance
pub fn handle_rf_transmission_complete() {
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
pub fn handle_rf_packet_reception() {
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
            rf_start_stx2rx(
                addr_of!(PKT_EMPTY) as u32,
                CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick(),
            );
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
fn process_received_packet_slow_path(
    rx_index: usize,
    dma_len: u8,
    light_rx_buff: &mut [LightRxBuff; LIGHT_RX_BUFF_COUNT],
) {
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
    if dma_len > 0xe
        && dma_len == (entry.sno[1] & 0x3f) + 0x11
        // SAFETY: (4) audited RF-DMA view. `entry` is one `LightRxBuff` slot of the
        // contiguous RX DMA buffer; the radio appends the status byte three bytes past
        // the `dma_len` frame it just wrote, so the read stays inside that DMA region.
        // Provenance is preserved by `addr_of!().cast::<u8>().add(dma_len + 3)`.
        && unsafe { *addr_of!(*entry).cast::<u8>().add(dma_len as usize + 3) } & 0x51 == 0x40
    {
        // Cast receive buffer to packet structure for processing
        // SAFETY: (4) audited `Packet`-union view. `entry.rx_time` is the start of the
        // DMA payload the radio filled with a full `Packet` (validated above); the view
        // is packed and all-integer, so every bit pattern is valid, and it borrows the
        // caller's `light_rx_buff` slot for as long as `packet` lives.
        let packet = unsafe { &*(addr_of!(entry.rx_time) as *const Packet) };

        // Extract command type from packet header (lower 4 bits of first sequence byte)
        // Command types: 3 = scan request, 5 = connection request, others = data/control
        let cmd = entry.sno[0] & 0xf;

        // Store packet reception timestamp for timing calculations and debugging
        LAST_PACKET_RECEIVED_TIMESTAMP.set(rx_time);

        // Handle packets when in advertisement state
        if BLE_PERIPHERAL_LINK_STATE.get() == crate::types::BlePeripheralLinkState::Advertising {
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
        if !dispatch_ota_snapshot().in_progress
            && BLE_PERIPHERAL_LINK_STATE.get() != crate::types::BlePeripheralLinkState::Receiving
        {
            handle_mesh_packet(packet, rx_time);
            return;
        }

        // Process BLE connection data packets
        handle_ble_connection_data(entry, packet, rx_time);
        return;
    }

    // Handle cleanup when in BLE receive state but packet was invalid
    if BLE_PERIPHERAL_LINK_STATE.get() == crate::types::BlePeripheralLinkState::Receiving {
        // Clear DMA3 (RF TX) mode byte before stopping TRX
        // This handles the case where we were expecting a BLE packet but received invalid data
        write_reg_dma3_mode(0);
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
        write_reg_rf_sched_tick(
            rx_time + BLE_SCAN_RESPONSE_INTERVAL_US.get() * CLOCK_SYS_CLOCK_1US,
        );

        // Configure RF for single transmission mode (0x85)
        write_reg_rf_mode_control(0x85);

        // Construct BLE scan response packet with device information
        let pkt_scan_rsp = Packet {
            scan_rsp: PacketScanRsp {
                dma_len: 0x27,         // DMA transfer length (39 bytes)
                _type: 0x4,            // BLE scan response packet type
                rf_len: 0x25,          // RF payload length (37 bytes)
                adv_a: *MAC_ID.lock(), // Advertiser's MAC address
                data: ScanRspData {
                    handle: 0xff1e, // Mesh-specific handle identifier
                    data: AdvRspPrivate {
                        device_address: DEVICE_ADDRESS.get(), // Device's mesh address
                        ..*ADV_RSP_PRI_DATA.lock()            // Additional device-specific data
                    },
                },
            },
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
            if 0x3fffffffi32
                < (read_reg_system_tick_irq() as i32 - read_reg_system_tick() as i32)
                    - (CLOCK_SYS_CLOCK_1US * 1000) as i32
            {
                return false;
            }
        }

        // Validate mesh packet format:
        // - rf_len must be 0x25 (37 bytes) - standard mesh packet size
        // - l2cap_len must be 0x21 (33 bytes) - L2CAP payload size
        // - type must be 2 (L2CAP data packet)
        // - chan_id must not be 0xeeff (reserved/invalid channel)
        // - packet must pass mesh decryption/authentication
        if packet.head().rf_len != 0x25
            || packet.head().l2cap_len != 0x21
            || packet.head()._type & 3 != 2
            || packet.head().chan_id == 0xeeff
            || pair_dec_packet_mesh(&mut packet).is_err()
        {
            return false;
        }

        // Special case: Node status advertisement packets (channel ID 0xffff)
        // These packets broadcast node status information across the mesh
        if packet.head().chan_id == 0xffff {
            return true;
        }

        // Parse the mesh command opcode and parameters from the decrypted packet
        let (success, op_cmd, op_cmd_len, params, params_len) =
            parse_ble_packet_op_params(&packet, true);
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

    // If packet validation passes, hand it to the mesh RX queue for processing
    if pkt_valid() {
        crate::queues::enqueue_mesh_rx(&packet);
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
            if 0x40000001
                > BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.get() - read_reg_system_tick()
            {
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
        SLAVE_NEXT_CONNECT_TICK
            .set(rx_time + SLAVE_LINK_INTERVAL.get() - CLOCK_SYS_CLOCK_1US * 1250);
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use core::ptr::addr_of;

    use crate::ble::connection::{
        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED, BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP,
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME, BRIDGE_RECEIVE_TIMING_TICK,
        CONN_UPDATE_CNT,
    };
    use crate::ble::pair::SECURITY_ENABLE;
    use crate::mesh_ll::node_status::RC_PKT_BUF;
    use crate::queues::MESH_RX_CHANNEL;
    use crate::types::BlePeripheralLinkState;
    use mesh_protocol::packet_types::PktBuf;
    use tlsr8266_hal::mcu::clock::CLOCK_SYS_CLOCK_1US;
    use tlsr8266_pac::fake_mmio::FakeMmio;

    /// Datasheet offsets of the RF receive / DMA / timer registers this module touches.
    const REG_RF_RX_STATUS: u32 = 0x443;
    const REG_RF_IRQ_STATUS: u32 = 0xf20;
    const REG_DMA2_ADDR: u32 = 0x508;
    const REG_SYSTEM_TICK: u32 = 0x740;
    const REG_SYSTEM_TICK_IRQ: u32 = 0x744;

    /// Number of `write_reg_dma2_addr` (0x508) writes the run performed.
    fn dma2_addr_writes() -> usize {
        FakeMmio::writes()
            .iter()
            .filter(|w| w.addr == (REG_DMA2_ADDR | tlsr8266_pac::REG_BASE_ADDR))
            .count()
    }

    /// Seed the free-running system tick and its interrupt compare value.
    fn seed_system_ticks(tick: u32, irq_tick: u32) {
        FakeMmio::set_reg32(REG_SYSTEM_TICK, tick);
        FakeMmio::set_reg32(REG_SYSTEM_TICK_IRQ, irq_tick);
    }

    /// Helper function to reset global state to known values for test isolation.
    /// This ensures each test starts with a clean state.
    fn reset_global_state() {
        // The PAC/HAL host fakes are process-wide singletons: clear them so a
        // test never inherits another test's register bank, analog bank, clock
        // or AES transport.
        FakeMmio::reset();
        tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog::reset();
        tlsr8266_hal::mcu::clock::fake_clock::reset();
        tlsr8266_hal::mcu::crypto::fake_crypto::reset();

        // Global queues and the duplicate-detection cache outlive every test in
        // this process: clear them.
        crate::queues::reset_for_test();
        RC_PKT_BUF.lock().clear();

        LIGHT_RX_BUFFER_WRITE_POINTER.set(0);
        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Disconnected);
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(false);
        crate::ble::connection::rf_link_slave_pairing_enable(false);
        crate::callbacks::test_ota::reset_for_test();
        crate::callbacks::test_ota::set_in_progress_for_test(false);
        SECURITY_ENABLE.set(false);
        LAST_PACKET_RECEIVED_TIMESTAMP.set(0);
        LIGHT_CONN_SN_MASTER.set(0);
        SLAVE_CONNECTED_TICK.set(0);
        SLAVE_WINDOW_SIZE.set(0);
        SLAVE_LINK_INTERVAL.set(0x9c400); // Default value
        SLAVE_NEXT_CONNECT_TICK.set(0);
        BLE_SCAN_RESPONSE_INTERVAL_US.set(150); // Default BLE scan response interval
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(0);
        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(false);
        BRIDGE_RECEIVE_TIMING_TICK.set(0);
        CONN_UPDATE_CNT.set(0);
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.set(false);
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.set(0);
        SLAVE_WINDOW_SIZE_UPDATE.set(0);
        DEVICE_ADDRESS.set(0x1234);

        // Initialize device MAC address
        {
            let mut mac = MAC_ID.lock();
            *mac = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        }

        // Initialize advertisement response data
        {
            let mut adv_data = ADV_RSP_PRI_DATA.lock();
            *adv_data = AdvRspPrivate::default();
        }
    }

    /// Helper function to create a mock LightRxBuff with valid packet data.
    fn create_rx_buffer_entry(
        rx_time: u32,
        dma_len: u8,
        mac: [u8; 4],
        sno: [u8; 3],
    ) -> LightRxBuff {
        LightRxBuff {
            dma_len,
            unk1: [0; 3],
            rssi: 0,
            unk2: [0; 3],
            rx_time,
            sno,
            unk3: [0; 5],
            mac,
            unk4: [0; 40],
        }
    }

    /// Helper function to create a valid BLE packet structure.
    fn create_ble_packet() -> Packet {
        use mesh_protocol::packet_types::{PacketAttCmd, PacketAttValue, PacketL2capHead};
        Packet {
            att_cmd: PacketAttCmd {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: 0,
                    rf_len: 0x25,
                    l2cap_len: 0x21,
                    chan_id: 0x0004,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: PacketAttValue {
                    sno: [0; 3],
                    src: [0; 2],
                    dst: [0; 2],
                    val: [0; 23],
                },
            },
        }
    }

    /// A connection-data packet that reaches `rf_link_slave_data`'s channel-map
    /// branch: `type & 3 == 3` and `l2cap_len == 1`. That branch records
    /// `BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP = 1`, which is the observable
    /// that proves the payload really was handed to the BLE data path.
    fn create_channel_map_update_packet() -> Packet {
        use mesh_protocol::packet_types::{PacketAttCmd, PacketAttValue, PacketL2capHead};
        Packet {
            att_cmd: PacketAttCmd {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: 3,
                    rf_len: 0x25,
                    l2cap_len: 1,
                    chan_id: 0x0004,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: PacketAttValue {
                    sno: [0; 3],
                    src: [0; 2],
                    dst: [0; 2],
                    val: [0; 23],
                },
            },
        }
    }

    /// Helper function to create a valid mesh packet structure.
    ///
    /// `val[0]` carries a 3-byte opcode (`0xc0` has both opcode-length bits
    /// set), which is what `parse_ble_packet_op_params` needs to accept a
    /// 0x21-byte L2CAP payload.
    fn create_mesh_packet() -> Packet {
        use mesh_protocol::packet_types::{PacketAttCmd, PacketAttValue, PacketL2capHead};
        Packet {
            att_cmd: PacketAttCmd {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: 2,        // L2CAP data packet - (_type & 3) == 2
                    rf_len: 0x25,    // Required mesh packet size (37 bytes)
                    l2cap_len: 0x21, // Required L2CAP payload size (33 bytes)
                    chan_id: 0x0004, // Valid channel (not 0xeeff)
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: PacketAttValue {
                    sno: [0; 3],
                    src: [0; 2],
                    dst: [0; 2],
                    // Bits 7 and 6 set = 3-byte opcode, which is what the real
                    // parser needs for a 0x21-byte L2CAP payload to fit.
                    val: [
                        0xc0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    ],
                },
            },
        }
    }

    // ================================================================================
    // Tests for handle_rf_transmission_complete function
    // ================================================================================

    /// Tests that RF transmission complete handler clears the correct interrupt bit.
    ///
    /// This function should clear bit 2 of the RF interrupt status register
    /// to acknowledge the transmission completion.
    #[test]
    fn test_handle_rf_transmission_complete() {
        reset_global_state();

        // Execute function
        handle_rf_transmission_complete();

        // Verify RF interrupt status bit 2 is cleared (write-1-to-clear)
        assert_eq!(
            FakeMmio::reg16(REG_RF_IRQ_STATUS),
            2,
            "bit 2 of reg_rf_irq_status must be acknowledged"
        );
    }

    // ================================================================================
    // Tests for handle_rf_packet_reception function - Error Handling
    // ================================================================================

    /// Tests RF packet reception with RF error status.
    ///
    /// When RF receive status is 0x0b (error), the function should:
    /// - Clear RF interrupt status (bit 1)
    /// - Return early without processing packet
    /// - Not advance write pointer or set up DMA
    #[test]
    fn test_handle_rf_packet_reception_rf_error() {
        reset_global_state();

        // RF error status in the receive status register
        FakeMmio::set_reg8(REG_RF_RX_STATUS, 0x0b);

        // Setup initial state
        let initial_write_ptr = 1;
        LIGHT_RX_BUFFER_WRITE_POINTER.set(initial_write_ptr);

        // Execute function
        handle_rf_packet_reception();

        // Verify error handling
        assert_eq!(
            FakeMmio::reg16(REG_RF_IRQ_STATUS),
            1,
            "bit 1 of reg_rf_irq_status must be acknowledged"
        );

        // Verify write pointer was advanced (this happens before error check)
        assert_eq!(
            LIGHT_RX_BUFFER_WRITE_POINTER.get(),
            (initial_write_ptr + 1) % LIGHT_RX_BUFF_COUNT,
            "Write pointer should be advanced even on error"
        );
    }

    /// Tests RF packet reception with invalid packet (dma_len == 1).
    ///
    /// The second reception of the same timestamp is the duplicate case: the
    /// handler must stop the radio and restart a scheduled STX2RX instead of
    /// processing the frame again.
    #[test]
    fn test_handle_rf_packet_reception_invalid_packet_duplicate() {
        reset_global_state();

        // Valid RF status
        FakeMmio::set_reg8(REG_RF_RX_STATUS, 0x00);
        seed_system_ticks(10_000, 0);

        let test_timestamp = 5000u32;

        // First reception: a frame the fast path accepts as new (dma_len != 1),
        // which records the timestamp used by duplicate detection. Its header
        // fails the slow-path length self-check, so no handler runs.
        {
            let mut buff = LIGHT_RX_BUFF.lock();
            *buff = [create_rx_buffer_entry(0, 0, [0; 4], [0; 3]); LIGHT_RX_BUFF_COUNT];
            // The handler processes the slot the write pointer names (1), and
            // advances it afterwards.
            buff[1] = create_rx_buffer_entry(test_timestamp, 25, [0; 4], [0x05, 0x00, 0x00]);
        }
        LIGHT_RX_BUFFER_WRITE_POINTER.set(1);
        handle_rf_packet_reception();

        // The duplicate-detection writes are the subject of the second round.
        FakeMmio::clear_writes();

        // Second reception: the same timestamp again, and the entry is now
        // marked processed, which is exactly the duplicate signature.
        LIGHT_RX_BUFFER_WRITE_POINTER.set(1);
        handle_rf_packet_reception();

        // Verify the radio was restarted for another receive attempt.
        assert_eq!(
            read_reg_rf_sched_tick(),
            CLOCK_SYS_CLOCK_1US * 10 + 10_000,
            "the restart must be scheduled 10µs after the system tick"
        );
        assert_eq!(
            read_reg_rf_mode() & 0x04,
            0x04,
            "the restart must enable cmd_schedule mode"
        );
        assert_eq!(
            read_reg_rf_mode_control(),
            0x87,
            "the restart must switch the radio to stx2rx"
        );
        assert_ne!(
            read_reg_dma3_addr(),
            0,
            "the restart must point the TX DMA at the empty packet"
        );

        // Verify the interrupt is acknowledged.
        assert_eq!(
            FakeMmio::reg16(REG_RF_IRQ_STATUS),
            1,
            "bit 1 of reg_rf_irq_status must be acknowledged"
        );
    }

    /// Tests RF packet reception with invalid packet (dma_len == 1) but no duplicate.
    ///
    /// When a packet has dma_len == 1 but timestamp doesn't match last received,
    /// the function should simply return without restarting RF operations.
    #[test]
    fn test_handle_rf_packet_reception_invalid_packet_no_duplicate() {
        reset_global_state();

        // Valid RF status
        FakeMmio::set_reg8(REG_RF_RX_STATUS, 0x00);

        // Create buffer with invalid packet (dma_len == 1) with unique timestamp
        let mut light_rx_buff = [create_rx_buffer_entry(0, 0, [0; 4], [0; 3]); LIGHT_RX_BUFF_COUNT];
        light_rx_buff[0] = create_rx_buffer_entry(9999, 1, [0; 4], [0; 3]);

        LIGHT_RX_BUFFER_WRITE_POINTER.set(1); // Will use index 0

        {
            let mut buff = LIGHT_RX_BUFF.lock();
            *buff = light_rx_buff;
        }

        // Execute function
        handle_rf_packet_reception();

        // Verify basic operations
        assert_eq!(
            FakeMmio::reg16(REG_RF_IRQ_STATUS),
            1,
            "bit 1 of reg_rf_irq_status must be acknowledged"
        );
        assert_eq!(dma2_addr_writes(), 1, "one DMA2 re-arm expected");
        assert_eq!(
            read_reg_rf_mode_control(),
            0,
            "an invalid non-duplicate frame must not restart the radio"
        );
    }

    // ================================================================================
    // Tests for handle_rf_packet_reception function - Valid Packet Processing
    // ================================================================================

    /// Tests RF packet reception with valid packet that gets processed.
    ///
    /// When a valid packet is received (dma_len > 1), the function should:
    /// - Set up DMA for next reception  
    /// - Mark current buffer as processed (dma_len = 1)
    /// - Update last received timestamp
    /// - Call slow path processing function
    #[test]
    fn test_handle_rf_packet_reception_valid_packet() {
        reset_global_state();

        // Valid RF status
        FakeMmio::set_reg8(REG_RF_RX_STATUS, 0x00);

        // Create buffer with valid packet
        let test_timestamp = 7500u32;
        let test_dma_len = 25u8;
        let mut light_rx_buff = [create_rx_buffer_entry(0, 0, [0; 4], [0; 3]); LIGHT_RX_BUFF_COUNT];
        light_rx_buff[0] = create_rx_buffer_entry(
            test_timestamp,
            test_dma_len,
            [0x11, 0x22, 0x33, 0x44],
            [0x05, 0x15, 0x10],
        );

        LIGHT_RX_BUFFER_WRITE_POINTER.set(1); // Will process index 0

        {
            let mut buff = LIGHT_RX_BUFF.lock();
            *buff = light_rx_buff;
        }

        // Execute function
        handle_rf_packet_reception();

        // The fast path marks the processed slot before delegating. The write
        // pointer names slot 1, so that is the slot it consumes.
        {
            let buff = LIGHT_RX_BUFF.lock();
            assert_eq!(
                buff[1].dma_len, 1,
                "the received slot must be marked processed"
            );
            assert_eq!(buff[0].dma_len, test_dma_len, "other slots stay untouched");
        }
        assert_eq!(
            LAST_PACKET_RECEIVED_TIMESTAMP.get(),
            0,
            "a frame failing the length self-check must not be timestamped"
        );

        // Verify DMA setup and interrupt clearing
        assert_eq!(
            FakeMmio::reg16(REG_RF_IRQ_STATUS),
            1,
            "bit 1 of reg_rf_irq_status must be acknowledged"
        );
        assert_eq!(dma2_addr_writes(), 1, "one DMA2 re-arm expected");
    }

    /// Tests buffer wraparound behavior in packet reception.
    ///
    /// Verifies that the circular buffer write pointer wraps around correctly
    /// when it reaches the buffer size limit.
    #[test]
    fn test_handle_rf_packet_reception_buffer_wraparound() {
        reset_global_state();

        // Valid RF status
        FakeMmio::set_reg8(REG_RF_RX_STATUS, 0x00);

        // Setup initial state at buffer boundary
        let initial_ptr = LIGHT_RX_BUFF_COUNT - 1; // Last valid index
        LIGHT_RX_BUFFER_WRITE_POINTER.set(initial_ptr);

        // Execute function
        handle_rf_packet_reception();

        // Verify wraparound: (3 + 1) % 4 = 0
        assert_eq!(
            LIGHT_RX_BUFFER_WRITE_POINTER.get(),
            0,
            "Write pointer should wrap around to 0 when exceeding buffer size"
        );
    }

    // ================================================================================
    // Tests for handle_scan_request function
    // ================================================================================

    /// Tests scan request handling with matching MAC address.
    ///
    /// When a scan request is addressed to this device (MAC matches), should:
    /// - Stop current RF operations
    /// - Schedule scan response transmission
    /// - Configure RF for single transmission mode
    /// - Set up DMA for scan response packet
    /// - Schedule next interrupt
    #[test]
    fn test_handle_scan_request_matching_mac() {
        reset_global_state();

        seed_system_ticks(0, 20_000);

        // Setup matching MAC address
        let device_mac = [0x11, 0x22, 0x33, 0x44];
        {
            let mut mac = MAC_ID.lock();
            mac[0..4].copy_from_slice(&device_mac);
        }

        // Create scan request entry with matching MAC
        let rx_time = 15000u32;
        let entry = create_rx_buffer_entry(rx_time, 20, device_mac, [0x03, 0x15, 0x10]);

        // Execute function
        handle_scan_request(&entry, rx_time);

        // Verify scan response timing (rx_time + scan_interval * clock)
        let expected_sched_time =
            rx_time + BLE_SCAN_RESPONSE_INTERVAL_US.get() * CLOCK_SYS_CLOCK_1US;
        assert_eq!(
            read_reg_rf_sched_tick(),
            expected_sched_time,
            "the scan response must be scheduled one scan interval after reception"
        );

        // Verify RF mode configuration for single transmission
        assert_eq!(
            read_reg_rf_mode_control(),
            0x85,
            "the radio must be switched to single-transmission mode"
        );

        // Verify DMA setup for scan response packet
        assert_ne!(
            read_reg_dma3_addr(),
            0,
            "the scan response packet must be handed to the TX DMA"
        );

        // Verify next interrupt scheduling
        let expected_irq_time = CLOCK_SYS_CLOCK_1US * 1000 + 20_000;
        assert_eq!(
            read_reg_system_tick_irq(),
            expected_irq_time,
            "the next state tick must be armed 1ms later"
        );

        // The radio is stopped before the response is scheduled.
        let mode_control_writes: std::vec::Vec<u32> = FakeMmio::writes()
            .iter()
            .filter(|w| w.addr == (0xf00 | tlsr8266_pac::REG_BASE_ADDR))
            .map(|w| w.value)
            .collect();
        assert_eq!(
            mode_control_writes.as_slice(),
            [0x80, 0x85],
            "the radio must be stopped, then armed for the response"
        );
    }

    /// Tests scan request handling with non-matching MAC address.
    ///
    /// When a scan request is not addressed to this device (MAC doesn't match),
    /// the function should do nothing and return early.
    #[test]
    fn test_handle_scan_request_non_matching_mac() {
        reset_global_state();

        // Setup non-matching MAC addresses
        {
            let mut mac = MAC_ID.lock();
            mac[0..4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        }

        // Create scan request with different MAC
        let entry = create_rx_buffer_entry(15000, 20, [0xAA, 0xBB, 0xCC, 0xDD], [0x03, 0x15, 0x10]);

        // Execute function
        handle_scan_request(&entry, 15000);

        // Verify no RF operation is performed at all.
        assert!(
            FakeMmio::writes().is_empty(),
            "a scan request for another address must not touch the radio"
        );
    }

    // ================================================================================
    // Tests for handle_connection_request function
    // ================================================================================

    /// Tests connection request handling with matching MAC address.
    ///
    /// When a connection request is addressed to this device, should call
    /// rf_link_slave_connect to establish the BLE connection. The callee always
    /// resets its connection-update bookkeeping on entry, which is the
    /// observable that proves the call happened.
    #[test]
    fn test_handle_connection_request_matching_mac() {
        reset_global_state();

        // Setup matching MAC address
        let device_mac = [0x11, 0x22, 0x33, 0x44];
        {
            let mut mac = MAC_ID.lock();
            mac[0..4].copy_from_slice(&device_mac);
        }

        // Create connection request entry and packet
        let rx_time = 25000u32;
        let entry = create_rx_buffer_entry(rx_time, 30, device_mac, [0x05, 0x20, 0x15]);
        let packet = create_ble_packet();

        CONN_UPDATE_CNT.set(7);

        // Execute function
        handle_connection_request(&entry, &packet, rx_time);

        // Verify connection establishment is initiated
        assert_eq!(
            CONN_UPDATE_CNT.get(),
            0,
            "rf_link_slave_connect must have run its entry bookkeeping"
        );
    }

    /// Tests connection request handling with non-matching MAC address.
    ///
    /// When a connection request is not addressed to this device,
    /// the function should do nothing.
    #[test]
    fn test_handle_connection_request_non_matching_mac() {
        reset_global_state();

        // Setup non-matching MAC addresses
        {
            let mut mac = MAC_ID.lock();
            mac[0..4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        }

        // Create connection request with different MAC
        let entry = create_rx_buffer_entry(25000, 30, [0xAA, 0xBB, 0xCC, 0xDD], [0x05, 0x20, 0x15]);
        let packet = create_ble_packet();

        CONN_UPDATE_CNT.set(7);

        // Execute function
        handle_connection_request(&entry, &packet, 25000);

        // Verify no connection operations are performed
        assert_eq!(
            CONN_UPDATE_CNT.get(),
            7,
            "a connection request for another address must not touch connection state"
        );
    }

    // ================================================================================
    // Tests for handle_mesh_packet function
    // ================================================================================

    /// Tests mesh packet handling with valid packet that passes all checks.
    ///
    /// When a mesh packet is valid and passes validation, it should be handed to
    /// the mesh RX queue.
    #[test]
    fn test_handle_mesh_packet_valid_packet() {
        reset_global_state();

        // Connected device, with the interrupt time inside the connection window
        // so the timing gate admits the packet.
        seed_system_ticks(30_000, 50_000);
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);

        // Create a valid mesh packet
        let packet = create_mesh_packet();

        // Execute function
        handle_mesh_packet(&packet, 35000);

        // The valid packet reaches the mesh RX queue.
        assert_eq!(
            MESH_RX_CHANNEL.len(),
            1,
            "a valid mesh packet must be queued for the mesh task"
        );
    }

    /// Tests mesh packet handling with timing validation failure.
    ///
    /// When the connection is active but timing is invalid (too far in future),
    /// the packet should be rejected without further processing.
    #[test]
    fn test_handle_mesh_packet_timing_validation_failure() {
        reset_global_state();

        // Need: 0x3fffffffi32 < (irq_tick - sys_tick) - (1000 * clock)
        // 0x3fffffffi32 = 1073741823
        // With CLOCK_SYS_CLOCK_1US = 32: need (irq_tick - sys_tick) - 32000 > 1073741823
        // So need: irq_tick - sys_tick > 1073741823 + 32000 = 1073773823
        seed_system_ticks(100, 1_073_774_000);

        // Setup connection state for timing validation
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);

        // Create packet
        let packet = create_mesh_packet();

        // Execute function
        handle_mesh_packet(&packet, 35000);

        // The timing gate rejects the packet before it can reach the queue.
        assert_eq!(
            MESH_RX_CHANNEL.len(),
            0,
            "a packet outside the connection window must not be queued"
        );
    }

    /// Tests mesh packet handling with decryption failure.
    ///
    /// When packet decryption fails, the packet should be rejected.
    #[test]
    fn test_handle_mesh_packet_decryption_failure() {
        reset_global_state();

        // Enable security and leave the encrypted flag clear: the real
        // decryption path rejects the frame.
        SECURITY_ENABLE.set(true);
        let packet = create_mesh_packet();

        // Execute function
        handle_mesh_packet(&packet, 35000);

        // Verify the packet was rejected
        assert_eq!(
            MESH_RX_CHANNEL.len(),
            0,
            "an undecryptable packet must not be queued"
        );
    }

    /// Tests mesh packet handling with duplicate packet detection.
    ///
    /// When a packet is detected as duplicate, it should be rejected.
    #[test]
    fn test_handle_mesh_packet_duplicate_detection() {
        reset_global_state();

        let packet = create_mesh_packet();

        // Opcode 0 (`0xc0 & 0x3f`) with the packet's sequence number is already
        // in the duplicate-detection cache.
        RC_PKT_BUF.lock().push_front(PktBuf {
            op: 0,
            sno: [0; 3],
            notify_ok_flag: false,
        });

        // Execute function
        handle_mesh_packet(&packet, 35000);

        // Verify the duplicate was rejected
        assert_eq!(
            MESH_RX_CHANNEL.len(),
            0,
            "a duplicate mesh packet must not be queued"
        );
    }

    // ================================================================================
    // Tests for handle_ble_connection_data function
    // ================================================================================

    /// Tests BLE connection data handling with new connection session.
    ///
    /// When receiving data from a new master (different sequence number),
    /// should update connection tracking and process the data.
    #[test]
    fn test_handle_ble_connection_data_new_session() {
        reset_global_state();

        seed_system_ticks(45_000, 0);

        // Setup initial state
        LIGHT_CONN_SN_MASTER.set(100); // Different from what we'll receive

        // Create connection data entry with new master sequence number
        // Master SN = (sno[2] * 256) | ((sno[0] >> 3) & 1)
        // For sno = [0x10, 0x20, 0x01]: SN = (1 * 256) | ((0x10 >> 3) & 1) = 256 | (2 & 1) = 256 | 0 = 256
        let entry = create_rx_buffer_entry(40000, 25, [0x11, 0x22, 0x33, 0x44], [0x10, 0x20, 0x01]);
        let packet = create_channel_map_update_packet();
        let rx_time = 40000u32;

        // Execute function
        handle_ble_connection_data(&entry, &packet, rx_time);

        // Verify new session handling
        assert_eq!(
            LIGHT_CONN_SN_MASTER.get(),
            256,
            "Master sequence number should be updated"
        );
        assert_eq!(
            SLAVE_CONNECTED_TICK.get(),
            45000,
            "Connected timestamp should be updated"
        );
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_ACTIVE.get(),
            true,
            "Connection should be marked active"
        );

        // Verify the payload was handed to the BLE data processor: the channel
        // map update branch is observable through its timing marker.
        assert_eq!(
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(),
            1,
            "the channel-map payload must reach rf_link_slave_data"
        );
    }

    /// Tests BLE connection data handling with existing connection session.
    ///
    /// When receiving data from the current master (same sequence number),
    /// should only adjust timing without updating connection tracking.
    #[test]
    fn test_handle_ble_connection_data_existing_session() {
        reset_global_state();

        // Setup state for existing connection
        LIGHT_CONN_SN_MASTER.set(1);
        // Master SN = 1: sno[2] = 0 and sno[0] bit 3 set.
        let entry = create_rx_buffer_entry(42000, 25, [0x11, 0x22, 0x33, 0x44], [0x08, 0x20, 0x00]);
        let packet = create_ble_packet();
        let rx_time = 42000u32;

        // Enable the timing adjuster and make the packet arrive early, so the
        // real `rf_link_timing_adjust` advances the next connection tick.
        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(true);
        BRIDGE_RECEIVE_TIMING_TICK.set(rx_time - CLOCK_SYS_CLOCK_1US * 100);
        SLAVE_NEXT_CONNECT_TICK.set(500_000);

        // Execute function
        handle_ble_connection_data(&entry, &packet, rx_time);

        // Verify timing adjustment for existing session
        assert_eq!(
            SLAVE_NEXT_CONNECT_TICK.get(),
            500_000 - CLOCK_SYS_CLOCK_1US * 200,
            "an early packet must advance the next connection tick"
        );
        assert_eq!(
            BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.get(),
            false,
            "one adjustment per connection event only"
        );

        // Verify master SN remains unchanged
        assert_eq!(
            LIGHT_CONN_SN_MASTER.get(),
            1,
            "Master sequence number should remain unchanged"
        );
    }

    /// Tests BLE connection data handling with window size management.
    ///
    /// When a window size is set and various timing conditions are met,
    /// should manage connection windows and timing updates correctly.
    #[test]
    fn test_handle_ble_connection_data_window_management() {
        reset_global_state();

        seed_system_ticks(50_000, 0);

        // Setup state for window management
        LIGHT_CONN_SN_MASTER.set(1);
        SLAVE_WINDOW_SIZE.set(5000); // Non-zero window size
        SLAVE_LINK_INTERVAL.set(10000);
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.set(false);
        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(true);
        BRIDGE_RECEIVE_TIMING_TICK.set(48_000 - CLOCK_SYS_CLOCK_1US * 100);
        SLAVE_NEXT_CONNECT_TICK.set(500_000);

        // Create entry for existing session
        let entry = create_rx_buffer_entry(48000, 25, [0x11, 0x22, 0x33, 0x44], [0x08, 0x20, 0x00]);
        let packet = create_ble_packet();
        let rx_time = 48000u32;

        // Execute function
        handle_ble_connection_data(&entry, &packet, rx_time);

        // Verify window management
        assert_eq!(
            SLAVE_WINDOW_SIZE.get(),
            0,
            "Window size should be reset to 0"
        );

        // Verify next connection timing calculation. The timing adjust ran
        // first, so this also proves the window block recomputes the tick after
        // it.
        let expected_next_tick = rx_time + SLAVE_LINK_INTERVAL.get() - CLOCK_SYS_CLOCK_1US * 1250;
        assert_eq!(
            SLAVE_NEXT_CONNECT_TICK.get(),
            expected_next_tick,
            "Next connection tick should be calculated correctly"
        );
    }

    /// Tests BLE connection data handling with timing update in progress.
    ///
    /// When a timing update is in progress and conditions are met,
    /// should properly handle the timing update completion.
    #[test]
    fn test_handle_ble_connection_data_timing_update_in_progress() {
        reset_global_state();

        seed_system_ticks(55_000, 0);

        // Setup state for timing update scenario
        LIGHT_CONN_SN_MASTER.set(1);
        SLAVE_WINDOW_SIZE.set(3000);
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.set(true);
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.set(60000); // Future time
        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(true);
        BRIDGE_RECEIVE_TIMING_TICK.set(54_000 - CLOCK_SYS_CLOCK_1US * 100);
        SLAVE_NEXT_CONNECT_TICK.set(600_000);

        // Create entry for existing session
        let entry = create_rx_buffer_entry(54000, 25, [0x11, 0x22, 0x33, 0x44], [0x08, 0x20, 0x00]);
        let packet = create_ble_packet();
        let rx_time = 54000u32;

        // Execute function
        handle_ble_connection_data(&entry, &packet, rx_time);

        // The timing adjust ran for the existing session.
        assert_eq!(
            SLAVE_NEXT_CONNECT_TICK.get(),
            600_000 - CLOCK_SYS_CLOCK_1US * 200,
            "an early packet must advance the next connection tick"
        );

        // In this case, since the condition 0x40000001 > (ok_time - sys_tick) evaluates true,
        // the function should return early, so window size should remain unchanged
        assert_eq!(
            SLAVE_WINDOW_SIZE.get(),
            3000,
            "Window size should remain unchanged during timing update"
        );
    }

    // ================================================================================
    // Integration Tests for process_received_packet_slow_path
    // ================================================================================

    /// Tests the slow path packet processing with comprehensive packet validation.
    ///
    /// This test focuses on the packet validation logic and state transitions
    /// in the slow path processing function.
    #[test]
    fn test_process_received_packet_slow_path_invalid_packet_cleanup() {
        reset_global_state();

        // Setup state for BLE receiving mode (triggers cleanup on invalid packet)
        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Receiving);

        // Create an invalid packet that will fail validation
        // Validation requires: dma_len > 0xe AND dma_len == (sno[1] & 0x3f) + 0x11 AND status check
        let mut light_rx_buff = [create_rx_buffer_entry(0, 0, [0; 4], [0; 3]); LIGHT_RX_BUFF_COUNT];
        light_rx_buff[0] =
            create_rx_buffer_entry(60000, 10, [0x11, 0x22, 0x33, 0x44], [0x05, 0x15, 0x10]); // dma_len=10 < 0xe

        // Call the slow path function directly (normally called from handle_rf_packet_reception)
        process_received_packet_slow_path(0, 10, &mut light_rx_buff);

        // Verify cleanup operations for invalid packet in receiving state
        assert_eq!(
            read_reg_dma3_mode(),
            0,
            "the RF TX DMA mode byte must be cleared"
        );
        assert_eq!(
            read_reg_rf_mode_control(),
            0x80,
            "the radio must be stopped after a failed reception"
        );
    }

    /// Tests scan request detection in advertisement state.
    ///
    /// When in advertisement state and receiving a scan request (cmd=3),
    /// should call the scan request handler.
    #[test]
    fn test_process_received_packet_slow_path_scan_request_detection() {
        reset_global_state();

        // Setup state for advertisement mode
        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Advertising);

        // Create a packet that will pass validation and has cmd=3 (scan request)
        // Need: dma_len > 0xe, dma_len == (sno[1] & 0x3f) + 0x11, and status check
        // Let's use dma_len=32, so need sno[1] & 0x3f = 32 - 0x11 = 15 (0x0f)
        // For cmd=3, need sno[0] & 0xf = 3
        let mut light_rx_buff = [create_rx_buffer_entry(0, 0, [0; 4], [0; 3]); LIGHT_RX_BUFF_COUNT];
        light_rx_buff[0] =
            create_rx_buffer_entry(65000, 32, [0x11, 0x22, 0x33, 0x44], [0x03, 0x0f, 0x10]);

        // Execute the function. The synthetic buffer has no RF status byte after
        // the frame (`unk4` is zero), so validation fails and no handler runs.
        process_received_packet_slow_path(0, 32, &mut light_rx_buff);

        // The failed validation is visible in the untouched reception timestamp
        // and in a radio that was not reconfigured.
        assert_eq!(
            LAST_PACKET_RECEIVED_TIMESTAMP.get(),
            0,
            "a frame failing validation must not be timestamped"
        );
        assert!(
            FakeMmio::writes().is_empty(),
            "a frame failing validation must not touch the radio"
        );
    }

    // ================================================================================
    // Edge Case and Error Condition Tests
    // ================================================================================

    /// Tests buffer index boundary conditions.
    ///
    /// Verifies that the packet reception handles buffer boundaries correctly
    /// and doesn't cause out-of-bounds access.
    #[test]
    fn test_buffer_boundary_conditions() {
        reset_global_state();

        // Valid RF status
        FakeMmio::set_reg8(REG_RF_RX_STATUS, 0x00);

        // Test all buffer positions
        for i in 0..LIGHT_RX_BUFF_COUNT {
            LIGHT_RX_BUFFER_WRITE_POINTER.set(i);

            // Execute function
            handle_rf_packet_reception();

            // Verify write pointer advancement
            let expected_next = (i + 1) % LIGHT_RX_BUFF_COUNT;
            assert_eq!(
                LIGHT_RX_BUFFER_WRITE_POINTER.get(),
                expected_next,
                "Buffer pointer should advance correctly from position {}",
                i
            );
        }
    }

    /// Tests state consistency during various link states.
    ///
    /// No link state may make the slow path accept a frame that fails
    /// validation, so the mesh queue stays empty and the radio is untouched.
    #[test]
    fn test_link_state_consistency() {
        reset_global_state();

        let states = [
            BlePeripheralLinkState::Disconnected,
            BlePeripheralLinkState::Advertising,
            BlePeripheralLinkState::Connected,
            BlePeripheralLinkState::Receiving,
        ];

        for state in states.iter() {
            reset_global_state();
            BLE_PERIPHERAL_LINK_STATE.set(*state);

            let mut light_rx_buff =
                [create_rx_buffer_entry(0, 0, [0; 4], [0; 3]); LIGHT_RX_BUFF_COUNT];
            light_rx_buff[0] =
                create_rx_buffer_entry(70000, 10, [0x11, 0x22, 0x33, 0x44], [0x05, 0x15, 0x10]);

            process_received_packet_slow_path(0, 10, &mut light_rx_buff);

            assert_eq!(
                MESH_RX_CHANNEL.len(),
                0,
                "no link state may queue an invalid frame"
            );
            // Only the Receiving state performs radio cleanup; the others leave
            // the radio untouched.
            if *state == BlePeripheralLinkState::Receiving {
                assert_eq!(
                    read_reg_rf_mode_control(),
                    0x80,
                    "a failed reception must stop the radio in Receiving state"
                );
            } else {
                assert!(
                    FakeMmio::writes().is_empty(),
                    "a failed reception must not touch the radio in {:?}",
                    state
                );
            }
        }
    }

    /// Tests timestamp handling and overflow conditions.
    ///
    /// Verifies that timestamp comparisons and calculations handle
    /// overflow conditions correctly.
    #[test]
    fn test_timestamp_overflow_handling() {
        reset_global_state();

        // Valid RF status
        FakeMmio::set_reg8(REG_RF_RX_STATUS, 0x00);

        // Test with timestamp near overflow boundary
        let near_overflow_time = 0xFFFFFFF0u32;
        let mut light_rx_buff = [create_rx_buffer_entry(0, 0, [0; 4], [0; 3]); LIGHT_RX_BUFF_COUNT];
        light_rx_buff[0] = create_rx_buffer_entry(
            near_overflow_time,
            25,
            [0x11, 0x22, 0x33, 0x44],
            [0x05, 0x15, 0x10],
        );

        LIGHT_RX_BUFFER_WRITE_POINTER.set(1);

        {
            let mut buff = LIGHT_RX_BUFF.lock();
            *buff = light_rx_buff;
        }

        // Execute function
        handle_rf_packet_reception();

        // Verify function completes without issues
        assert_eq!(
            FakeMmio::reg16(REG_RF_IRQ_STATUS),
            1,
            "bit 1 of reg_rf_irq_status must be acknowledged"
        );

        // Test with zero timestamp
        light_rx_buff[1] =
            create_rx_buffer_entry(0, 25, [0x11, 0x22, 0x33, 0x44], [0x05, 0x15, 0x10]);
        LIGHT_RX_BUFFER_WRITE_POINTER.set(2);

        {
            let mut buff = LIGHT_RX_BUFF.lock();
            *buff = light_rx_buff;
        }

        handle_rf_packet_reception();

        // Verify function handles zero timestamp correctly: the ack is
        // write-1-to-clear, so both receptions leave the same value behind
        assert_eq!(
            FakeMmio::reg16(REG_RF_IRQ_STATUS),
            1,
            "each reception acknowledges bit 1"
        );
    }

    /// Number of writes of `value` to `reg_rf_mode_control` in the run log.
    fn rf_mode_control_writes(value: u8) -> usize {
        FakeMmio::writes()
            .iter()
            .filter(|w| w.addr == (0xf00 | tlsr8266_pac::REG_BASE_ADDR) && w.value == value as u32)
            .count()
    }

    /// Tests MAC address edge cases.
    ///
    /// Verifies that MAC address comparisons handle various edge cases
    /// including all zeros, all ones, and partial matches.
    #[test]
    fn test_mac_address_edge_cases() {
        reset_global_state();
        seed_system_ticks(0, 50_000);

        // Test with all-zero MAC
        {
            let mut mac = MAC_ID.lock();
            mac[0..4].copy_from_slice(&[0x00, 0x00, 0x00, 0x00]);
        }

        let entry_zero =
            create_rx_buffer_entry(75000, 20, [0x00, 0x00, 0x00, 0x00], [0x03, 0x15, 0x10]);
        handle_scan_request(&entry_zero, 75000);
        assert_eq!(rf_mode_control_writes(0x85), 1, "an all-zero MAC matches");

        // Test with all-ones MAC
        {
            let mut mac = MAC_ID.lock();
            mac[0..4].copy_from_slice(&[0xFF, 0xFF, 0xFF, 0xFF]);
        }

        let entry_ones =
            create_rx_buffer_entry(76000, 20, [0xFF, 0xFF, 0xFF, 0xFF], [0x03, 0x15, 0x10]);
        handle_scan_request(&entry_ones, 76000);
        assert_eq!(rf_mode_control_writes(0x85), 2, "an all-ones MAC matches");

        // Test with partial match (should not match)
        let entry_partial =
            create_rx_buffer_entry(77000, 20, [0xFF, 0xFF, 0xFF, 0xFE], [0x03, 0x15, 0x10]);
        handle_scan_request(&entry_partial, 77000);
        assert_eq!(
            rf_mode_control_writes(0x85),
            2,
            "a partial MAC match must not start a response"
        );
    }

    /// Tests the valid packet processing logic at line 145+ in process_received_packet_slow_path.
    ///
    /// This tests the actual logic inside the validation condition by creating a buffer
    /// structure that satisfies the validation requirements.
    #[test]
    fn test_process_received_packet_slow_path_valid_packet_logic() {
        reset_global_state();
        seed_system_ticks(0, 50_000);

        // Set state to advertising to trigger scan request path
        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Advertising);

        // Set up MAC to match for scan request handling
        {
            let mut mac = MAC_ID.lock();
            mac[0..4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        }

        // Create a buffer structure that will pass validation
        // Validation: dma_len > 0xe && dma_len == (sno[1] & 0x3f) + 0x11 && status check
        let test_dma_len = 25u8; // > 0xe ✓
        let sno1_for_dma = test_dma_len - 0x11; // 25 - 17 = 8 (0x08)
        let rx_time = 80000u32;

        // Create an extended buffer that includes space for the status byte
        let mut extended_buffer = [0u8; 128]; // Large enough buffer

        // Set up the LightRxBuff structure at the start
        let entry_ptr = extended_buffer.as_mut_ptr() as *mut LightRxBuff;
        unsafe {
            *entry_ptr = LightRxBuff {
                dma_len: test_dma_len,
                unk1: [0; 3],
                rssi: 0,
                unk2: [0; 3],
                rx_time,
                sno: [0x03, sno1_for_dma, 0x10], // cmd=3 (scan request), validation match
                unk3: [0; 5],
                mac: [0x11, 0x22, 0x33, 0x44], // Matching MAC for scan request
                unk4: [0; 40],
            };
        }

        // Set the status byte at the expected location: addr_of!(entry) + dma_len + 3
        // Status check: byte & 0x51 == 0x40, so we need a byte like 0x40
        let status_offset = test_dma_len as usize + 3;
        if status_offset < extended_buffer.len() {
            extended_buffer[status_offset] = 0x40; // Will pass: 0x40 & 0x51 = 0x40 ✓
        }

        // Create light_rx_buff array with our constructed entry
        let mut light_rx_buff = [create_rx_buffer_entry(0, 0, [0; 4], [0; 3]); LIGHT_RX_BUFF_COUNT];
        light_rx_buff[0] = unsafe { *entry_ptr };

        // Execute the function with our carefully constructed buffer
        process_received_packet_slow_path(0, test_dma_len, &mut light_rx_buff);

        // 1. Check that timestamp was stored
        assert_eq!(
            LAST_PACKET_RECEIVED_TIMESTAMP.get(),
            rx_time,
            "Timestamp should be updated when validation passes"
        );

        // 2. Since we're in advertising state with cmd=3, the scan request
        // handler must have armed a response.
        assert_eq!(
            read_reg_rf_sched_tick(),
            rx_time + BLE_SCAN_RESPONSE_INTERVAL_US.get() * CLOCK_SYS_CLOCK_1US,
            "the scan response must be scheduled after the received frame"
        );
        assert_eq!(
            read_reg_rf_mode_control(),
            0x85,
            "the radio must be switched to single-transmission mode"
        );
        assert_ne!(
            read_reg_dma3_addr(),
            0,
            "the scan response must be handed to the TX DMA"
        );
    }

    /// Tests the connection request path (line 165) in process_received_packet_slow_path.
    ///
    /// This tests the early return on line 165 when cmd=5 (connection request) in advertising state.
    #[test]
    fn test_process_received_packet_slow_path_connection_request_path() {
        reset_global_state();

        // Set state to advertising to trigger connection request path
        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Advertising);

        // Create a buffer structure for connection request (cmd=5)
        let test_dma_len = 25u8;
        let sno1_for_dma = test_dma_len - 0x11; // 25 - 17 = 8 (0x08)
        let rx_time = 85000u32;

        let mut extended_buffer = [0u8; 128];
        let entry_ptr = extended_buffer.as_mut_ptr() as *mut LightRxBuff;
        unsafe {
            *entry_ptr = LightRxBuff {
                dma_len: test_dma_len,
                unk1: [0; 3],
                rssi: 0,
                unk2: [0; 3],
                rx_time,
                sno: [0x05, sno1_for_dma, 0x10], // cmd=5 (connection request)
                unk3: [0; 5],
                mac: [0x11, 0x22, 0x33, 0x44],
                unk4: [0; 40],
            };
        }

        // Set status byte to pass validation
        let status_offset = test_dma_len as usize + 3;
        if status_offset < extended_buffer.len() {
            extended_buffer[status_offset] = 0x40;
        }

        let mut light_rx_buff = [create_rx_buffer_entry(0, 0, [0; 4], [0; 3]); LIGHT_RX_BUFF_COUNT];
        light_rx_buff[0] = unsafe { *entry_ptr };

        CONN_UPDATE_CNT.set(9);

        // Execute the function
        process_received_packet_slow_path(0, test_dma_len, &mut light_rx_buff);

        // Verify timestamp was stored
        assert_eq!(
            LAST_PACKET_RECEIVED_TIMESTAMP.get(),
            rx_time,
            "Timestamp should be updated when validation passes"
        );

        // Verify the connection request handler ran and returned early: the
        // connect routine resets its update counter before any validation, so
        // the mesh and BLE data paths were never reached.
        assert_eq!(
            CONN_UPDATE_CNT.get(),
            0,
            "the connection request must reach rf_link_slave_connect"
        );
        assert_eq!(
            LIGHT_CONN_SN_MASTER.get(),
            0,
            "the BLE data path must not run for a connection request"
        );
    }

    /// Tests the mesh packet path (line 172) in process_received_packet_slow_path.
    ///
    /// This tests the early return on line 172 when processing mesh packets.
    #[test]
    fn test_process_received_packet_slow_path_mesh_packet_path() {
        reset_global_state();

        // Set state to NOT advertising and NOT in OTA to trigger mesh path
        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Disconnected); // Not advertising
        crate::callbacks::test_ota::set_in_progress_for_test(false); // Not in OTA

        // Create a buffer structure for mesh packet (cmd != 3 and != 5).
        // The length self-check is `dma_len == (rf_len & 0x3f) + 0x11`, and a
        // mesh frame carries rf_len = 0x25, so the DMA length must be 0x36.
        let test_dma_len = 0x36u8;
        let sno1_for_dma = 0x25u8; // mesh rf_len
        let rx_time = 90000u32;

        let mut extended_buffer = [0u8; 128];
        let entry_ptr = extended_buffer.as_mut_ptr() as *mut LightRxBuff;
        unsafe {
            *entry_ptr = LightRxBuff {
                dma_len: test_dma_len,
                unk1: [0; 3],
                rssi: 0,
                unk2: [0; 3],
                rx_time,
                sno: [0x07, sno1_for_dma, 0x10], // cmd=7 (not scan/connection request)
                unk3: [0; 5],
                mac: [0x11, 0x22, 0x33, 0x44],
                unk4: [0; 40],
            };
        }

        // The bytes the radio wrote are the packet the mesh path inspects: make
        // them a well-formed, parseable mesh frame so acceptance is visible.
        // The `Packet` view starts at `rx_time`, so the frame's DMA-length word
        // is the reception timestamp of this synthetic entry.
        unsafe {
            let packet_ptr = addr_of!((*entry_ptr).rx_time) as *mut u8;
            let mut mesh = create_mesh_packet();
            mesh.head_mut().dma_len = rx_time;
            core::ptr::copy_nonoverlapping(
                addr_of!(mesh) as *const u8,
                packet_ptr,
                core::mem::size_of::<Packet>(),
            );
        }

        // Set status byte to pass validation (after the packet copy, which
        // overwrites the same stretch of the buffer).
        let status_offset = test_dma_len as usize + 3;
        if status_offset < extended_buffer.len() {
            extended_buffer[status_offset] = 0x40;
        }

        let mut light_rx_buff = [create_rx_buffer_entry(0, 0, [0; 4], [0; 3]); LIGHT_RX_BUFF_COUNT];
        light_rx_buff[0] = unsafe { *entry_ptr };
        // Execute the function
        process_received_packet_slow_path(0, test_dma_len, &mut light_rx_buff);

        // Verify timestamp was stored
        assert_eq!(
            LAST_PACKET_RECEIVED_TIMESTAMP.get(),
            rx_time,
            "Timestamp should be updated when validation passes"
        );

        // Verify the mesh packet handler accepted and queued the frame, and that
        // the BLE data path was not reached.
        assert_eq!(
            MESH_RX_CHANNEL.len(),
            1,
            "the mesh path must queue the accepted frame"
        );
        assert_eq!(
            LIGHT_CONN_SN_MASTER.get(),
            0,
            "the BLE data path must not run for a mesh packet"
        );
    }

    /// Tests the BLE connection data path (line 177) in process_received_packet_slow_path.
    ///
    /// This tests the fallthrough to line 176-177 for BLE connection data handling.
    #[test]
    fn test_process_received_packet_slow_path_ble_connection_data_path() {
        reset_global_state();

        // Set state to trigger BLE connection data path (line 176)
        // This happens when: NOT in advertising, AND (in OTA OR in Receiving state)
        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Receiving); // In receiving state
        crate::callbacks::test_ota::set_in_progress_for_test(false);

        // Create a buffer structure for BLE connection data
        let test_dma_len = 25u8;
        let sno1_for_dma = test_dma_len - 0x11; // 25 - 17 = 8 (0x08)
        let rx_time = 95000u32;

        let mut extended_buffer = [0u8; 128];
        let entry_ptr = extended_buffer.as_mut_ptr() as *mut LightRxBuff;
        unsafe {
            *entry_ptr = LightRxBuff {
                dma_len: test_dma_len,
                unk1: [0; 3],
                rssi: 0,
                unk2: [0; 3],
                rx_time,
                sno: [0x08, sno1_for_dma, 0x10], // cmd=8 (BLE data)
                unk3: [0; 5],
                mac: [0x11, 0x22, 0x33, 0x44],
                unk4: [0; 40],
            };
        }

        // Set status byte to pass validation
        let status_offset = test_dma_len as usize + 3;
        if status_offset < extended_buffer.len() {
            extended_buffer[status_offset] = 0x40;
        }

        let mut light_rx_buff = [create_rx_buffer_entry(0, 0, [0; 4], [0; 3]); LIGHT_RX_BUFF_COUNT];
        light_rx_buff[0] = unsafe { *entry_ptr };

        // Execute the function
        process_received_packet_slow_path(0, test_dma_len, &mut light_rx_buff);

        // Verify timestamp was stored
        assert_eq!(
            LAST_PACKET_RECEIVED_TIMESTAMP.get(),
            rx_time,
            "Timestamp should be updated when validation passes"
        );

        // Verify the BLE connection data handler ran: it adopts the new master
        // sequence number and marks the link active.
        assert_eq!(
            LIGHT_CONN_SN_MASTER.get(),
            ((0x10u16) * 0x100) | (((0x08u16 >> 3) & 1) as u16),
            "the BLE data handler must adopt the master sequence number"
        );
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_ACTIVE.get(),
            true,
            "the BLE data handler must mark the connection active"
        );
        assert_eq!(
            MESH_RX_CHANNEL.len(),
            0,
            "a BLE data packet must not be queued for the mesh task"
        );
    }

    /// Tests the node status advertisement special case (line 285) in handle_mesh_packet.
    ///
    /// This tests the early return when channel ID is 0xffff (node status packets).
    #[test]
    fn test_handle_mesh_packet_node_status_advertisement_line_285() {
        reset_global_state();

        // Set connection state for timing validation
        seed_system_ticks(30_000, 50_000);
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);

        // Create a mesh packet with channel ID 0xffff (node status)
        let mut packet = create_mesh_packet();
        unsafe {
            packet.head_mut().chan_id = 0xffff; // Special node status channel
        }

        let rx_time = 75000u32;

        // Execute the function - should hit the node-status early accept
        handle_mesh_packet(&packet, rx_time);

        // For node status packets (0xffff) the packet is queued for the mesh RX
        // task without opcode parsing.
        assert_eq!(crate::queues::MESH_RX_CHANNEL.len(), 1);
    }

    /// Tests the parse failure case (line 291) in handle_mesh_packet.
    ///
    /// This tests the early return when parse_ble_packet_op_params fails.
    #[test]
    fn test_handle_mesh_packet_parse_failure_line_291() {
        reset_global_state();

        // Set connection state for timing validation
        seed_system_ticks(30_000, 50_000);
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);

        // A 2-byte opcode (MSB set, 0x40 clear) is unsupported, so the real
        // parser rejects the frame.
        let mut packet = create_mesh_packet();
        unsafe {
            packet.att_write_mut().value.val[0] = 0x80;
        }

        let rx_time = 80000u32;

        // Execute the function
        handle_mesh_packet(&packet, rx_time);

        // Since validation returns false at the parse step, nothing is queued.
        assert_eq!(
            MESH_RX_CHANNEL.len(),
            0,
            "an unparseable mesh packet must not be queued"
        );
    }

    /// Tests the fallthrough to handle_ble_connection_data (line 193) in process_received_packet_slow_path.
    ///
    /// This tests the case where:
    /// - We pass packet validation
    /// - We enter the Advertising state check
    /// - We DON'T take the scan request branch (cmd != 3)
    /// - We DON'T take the connection request branch (cmd != 5)
    /// - We DON'T take the mesh packet branch because an OTA is in progress
    /// - We fall through to handle_ble_connection_data
    #[test]
    fn test_process_received_packet_slow_path_fallthrough_line_180() {
        reset_global_state();

        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Advertising);
        crate::callbacks::test_ota::set_in_progress_for_test(true); // In OTA mode to skip mesh handler

        // Create a buffer structure with cmd that is neither 3 nor 5
        let test_dma_len = 25u8;
        let sno1_for_dma = test_dma_len - 0x11; // 25 - 17 = 8 (0x08)
        let rx_time = 120000u32;

        let mut extended_buffer = [0u8; 128];
        let entry_ptr = extended_buffer.as_mut_ptr() as *mut LightRxBuff;
        unsafe {
            *entry_ptr = LightRxBuff {
                dma_len: test_dma_len,
                unk1: [0; 3],
                rssi: 0,
                unk2: [0; 3],
                rx_time,
                sno: [0x07, sno1_for_dma, 0x10], // cmd=7 (not 3 or 5)
                unk3: [0; 5],
                mac: [0x11, 0x22, 0x33, 0x44],
                unk4: [0; 40],
            };
        }

        // Set status byte to pass validation
        let status_offset = test_dma_len as usize + 3;
        if status_offset < extended_buffer.len() {
            extended_buffer[status_offset] = 0x40;
        }

        let mut light_rx_buff = [create_rx_buffer_entry(0, 0, [0; 4], [0; 3]); LIGHT_RX_BUFF_COUNT];
        light_rx_buff[0] = unsafe { *entry_ptr };

        // Execute the function
        process_received_packet_slow_path(0, test_dma_len, &mut light_rx_buff);

        // Only the BLE connection data handler ran: the other branches would
        // have queued a mesh packet or armed a response.
        assert_eq!(
            MESH_RX_CHANNEL.len(),
            0,
            "the mesh path must be skipped during OTA"
        );
        assert_eq!(
            read_reg_rf_mode_control(),
            0,
            "no scan/connection response may be armed"
        );
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_ACTIVE.get(),
            true,
            "the BLE data path must adopt the new session"
        );
    }
}
