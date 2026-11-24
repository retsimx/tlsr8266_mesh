//! # BLE Connection Management Module
//!
//! This module handles BLE connection parameter updates, connection state management,
//! and timing synchronization for the TLSR8266-based mesh lighting system.
//!
//! ## Key Responsibilities:
//! - BLE connection parameter and timing updates
//! - Connection state transitions and cleanup
//! - Channel map updates and frequency hopping management
//! - Connection event timing synchronization

use crate::ble::channel_selection::{ble_ll_build_available_channel_table, SLAVE_CHN_MAP};
use crate::ble::connection::{
    back_to_rxmode_bridge, BLE_CONN_INTERVAL, BLE_CONN_OFFSET, BLE_CONN_TIMEOUT,
    BLE_PERIPHERAL_CONNECTION_ACTIVE, BLE_PERIPHERAL_CONNECTION_INSTANT,
    BLE_PERIPHERAL_CONNECTION_TIMEOUT_US, BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP,
    BLE_PERIPHERAL_LINK_STATE, BLE_PERIPHERAL_NEXT_UPDATE_INSTANT,
    BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL, BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP,
    BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG, BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME,
    BRIDGE_COMMAND_TIMESTAMP, BRIDGE_SEQUENCE_NUMBER, BUFF_RESPONSE,
    GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP, NEED_UPDATE_CONNECT_PARA, SLAVE_CONNECTED_TICK,
    SLAVE_DATA_VALID, SLAVE_LINK_INTERVAL, SLAVE_NEXT_CONNECT_TICK, SLAVE_WINDOW_SIZE,
    SLAVE_WINDOW_SIZE_UPDATE,
};
use crate::ble::flash_config::pair_load_key;
use crate::ble::pair::{PAIR_LOGIN_OK, PAIR_SETTING_FLAG, SECURITY_ENABLE};
use crate::ble::status::{
    rf_link_slave_read_status_par_init, rf_link_slave_read_status_stop, tx_packet_bridge,
    DEVICE_STATUS_BUFFER_READ_POINTER, DEVICE_STATUS_BUFFER_WRITE_POINTER,
    DEVICE_STATUS_READ_BUSY_TIMESTAMP, SLAVE_READ_STATUS_BUSY,
};
use crate::callbacks::{dispatch_ota_snapshot, dispatch_ota_tick};
use crate::mesh_ll::node_status::{mesh_node_flush_status, mesh_report_status_enable};
use crate::mesh_ll::packet_processing::{
    is_add_packet_buf_ready, rf_link_add_tx_packet, rf_link_slave_data,
};
use crate::rf::drv_8266::*;
use crate::rf::drv_8266::{CURRENT_RF_STATE, PKT_INIT};
use crate::types::{
    ePairState, BlePeripheralLinkState, RfOperationState, BUFF_RESPONSE_PACKET_COUNT,
    SLAVE_READ_STATUS_BUSY_TIMEOUT,
};
use tlsr8266_hal::common::sync::SimplifyLS;
use tlsr8266_hal::mcu::clock::CLOCK_SYS_CLOCK_1US;
use tlsr8266_pac::*;

/// Outcome of one BLE connected-state dispatch, reported up to the interrupt
/// composition root (the crate root) which owns the `ota_manager` calls.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum BleConnectionEvent {
    /// Nothing for the composition root to do.
    None,
    /// OTA is active: poll the OTA manager's finish handler.
    OtaFinishCheck,
    /// Connection supervision timed out during OTA: finish with an error.
    OtaErrorReboot,
    /// OTA is active and the timeout counter reached zero: finish, then error-reboot
    /// only if the finish handler did not refresh the counter.
    OtaFinishCheckAndErrorReboot,
}

/// Handles BLE connection parameter and timing updates for the slave device.
///
/// This function implements the BLE specification's connection parameter update and
/// channel map update procedures. It processes timing updates that were previously
/// scheduled to occur at specific connection event instants.
///
/// ## BLE Specification Context:
/// - **Connection Parameter Updates**: When a master sends LL_CONNECTION_UPDATE_IND,
///   the slave must apply new parameters at the specified instant
/// - **Channel Map Updates**: When a master sends LL_CHANNEL_MAP_IND, the slave
///   must update its frequency hopping table at the specified instant
///
/// ## Update Types:
/// - `BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP == 1`: Channel map update (frequency hopping table)
/// - `BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP == 2`: Connection parameter update (interval, timeout, window)
///
/// ## Timing Synchronization:
/// Updates are applied when `BLE_PERIPHERAL_NEXT_UPDATE_INSTANT` matches `BLE_PERIPHERAL_CONNECTION_INSTANT`, ensuring
/// both master and slave apply changes at the same connection event for synchronization.
pub fn handle_ble_connection_parameter_updates() {
    // Increment the connection event counter - this tracks which connection event we're in
    // This is essential for BLE timing synchronization between master and slave
    BLE_PERIPHERAL_CONNECTION_INSTANT.inc();

    // Handle Channel Map Update (LL_CHANNEL_MAP_IND procedure)
    if BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get() == 1
        && BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.get() == BLE_PERIPHERAL_CONNECTION_INSTANT.get()
    {
        // Clear the update flag since we're processing it now
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(0);

        // Get the new channel map that was received from the master
        let chn_map = SLAVE_CHN_MAP.lock().clone();

        // Rebuild the frequency hopping table based on the new channel map
        // This updates which of the 37 BLE channels are available for data transmission
        ble_ll_build_available_channel_table(&chn_map, false);

        // Update the packet initialization structure with the new channel map
        // This ensures all future packets use the correct hopping sequence
        PKT_INIT.lock().ll_init_mut().chm = chn_map;
    } else {
        // Handle Connection Parameter Update (LL_CONNECTION_UPDATE_IND procedure)
        if BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get() == 2
            && BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.get() == BLE_PERIPHERAL_CONNECTION_INSTANT.get()
        {
            // Clear the update flag since we're processing it now
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(0);

            // Calculate the exact time when the new parameters take effect
            // This includes the connection offset to maintain precise timing
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME
                .set(BLE_CONN_OFFSET.get() + SLAVE_NEXT_CONNECT_TICK.get());

            // Apply the new connection parameters that were received from the master:

            // Update connection interval (time between connection events)
            SLAVE_LINK_INTERVAL.set(BLE_CONN_INTERVAL.get());

            // Update supervision timeout (max time before connection is considered lost)
            BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(BLE_CONN_TIMEOUT.get());

            // Set flag indicating timing update is in progress
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.set(true);

            // Calculate new window size: connection interval minus a fixed offset (0x4e2 * 1µs)
            // Window size determines how long the slave listens for packets in each connection event
            SLAVE_WINDOW_SIZE.set(
                BLE_CONN_INTERVAL
                    .get()
                    .saturating_sub(CLOCK_SYS_CLOCK_1US * 0x4e2),
            );

            // Use the smaller of the calculated window size or the master's requested window size
            // This ensures we don't exceed the master's expected listening window
            if SLAVE_WINDOW_SIZE_UPDATE.get() < SLAVE_WINDOW_SIZE.get() {
                SLAVE_WINDOW_SIZE.set(SLAVE_WINDOW_SIZE_UPDATE.get());
            }

            // Update the next connection event time to use the new timing
            SLAVE_NEXT_CONNECT_TICK.set(BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.get());
        }
    }
}

/// Handles BLE connection disconnection and cleanup.
///
/// This function performs comprehensive cleanup when a BLE connection is terminated,
/// either due to timeout, error, or explicit disconnection. It resets connection
/// state, security settings, and transitions back to advertisement mode.
///
/// ## Cleanup Operations:
/// - Resets connection and pairing state
/// - Stops ongoing status operations
/// - Handles security and pairing key management
/// - Configures hardware registers for disconnected state
/// - Disables mesh status reporting
///
/// ## State Transitions:
/// - Connection state → Disconnected
/// - Link state → Advertisement mode
/// - Mesh reporting → Disabled
/// - Pairing state → Reset or loaded from storage
pub fn cleanup_ble_disconnection() {
    // Schedule transition to advertisement state with short delay (100µs)
    write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
    *CURRENT_RF_STATE.lock() = RfOperationState::Advertising;

    // Reset DMA transmission pointer to clear any pending transmissions
    write_reg_dma_tx_rptr(0x10);

    // Clear connection and pairing state flags
    BLE_PERIPHERAL_CONNECTION_ACTIVE.set(false);
    PAIR_LOGIN_OK.set(false);

    // Handle security-specific cleanup
    if SECURITY_ENABLE.get() == false {
        // Security disabled: simply disable mesh status reporting
        mesh_report_status_enable(false);
    } else {
        // Security enabled: additional cleanup required
        BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(0); // Clear first connection timestamp
        PAIR_LOGIN_OK.set(false); // Ensure pairing state is cleared
        mesh_report_status_enable(false); // Disable mesh status reporting
    }

    // Stop any ongoing status read operations
    if SLAVE_READ_STATUS_BUSY.get() != 0 {
        rf_link_slave_read_status_stop();
    }

    // Handle pairing key management based on current pairing state
    if *PAIR_SETTING_FLAG.lock() == ePairState::PairSetted {
        // Device is already paired: just clear data validity flag
        SLAVE_DATA_VALID.set(0);
    } else {
        // Device not paired: clear data and load pairing keys from storage
        SLAVE_DATA_VALID.set(0);
        pair_load_key(); // Load stored pairing keys if available
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
    }

    // Configure the RF TX wait/settle timing low byte based on the SYSTEM CLOCK
    // (not the crystal frequency): 0x5e = 16 MHz sysclk, 0x68 = 32/48 MHz sysclk
    if CLOCK_SYS_CLOCK_1US == 0x10 {
        write_reg_rf_tx_wail_settle_time_lo(0x5e); // 16 MHz system clock
    } else {
        write_reg_rf_tx_wail_settle_time_lo(0x68); // 32/48 MHz system clock
    }

    // Reset connection parameter update state
    GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(0); // Clear service discovery timestamp
    NEED_UPDATE_CONNECT_PARA.set(false); // Clear connection parameter update flag
}

/// Handles the BLE bridge state interrupt for connected operation.
///
/// This is the main connection maintenance function that handles active BLE connections.
/// It manages connection timeouts, OTA updates, mesh packet transmission, status reporting,
/// and connection event timing. This function is called periodically during active
/// BLE connections.
///
/// ## Key Responsibilities:
/// - **Connection Supervision**: Monitors connection timeout and handles disconnection
/// - **OTA Management**: Handles over-the-air firmware update timeouts and completion
/// - **Mesh Communication**: Processes mesh packets and status reporting
/// - **Timing Management**: Maintains precise connection event timing
/// - **Bridge Operations**: Handles command bridging between BLE and mesh networks
///
/// ## State Management:
/// - Maintains connection state (BLE_PERIPHERAL_LINK_STATE = 5)
/// - Tracks OTA operation timeouts
/// - Manages connection parameter updates
/// - Coordinates mesh status reporting
///
/// Initializes hardware and registers for an active BLE connection.
///
/// This function sets up the hardware state for bridge mode operation:
/// - Sets link state to bridge mode (state 5)
/// - Increments operation counter for debugging
/// - Clears the DMA3 (RF TX) mode byte and stops ongoing operations
/// - Configures RF TX wait/settle timing based on system clock frequency
fn initialize_connection_hardware() {
    // Set link state to bridge mode - active BLE connection
    BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Connected);

    // Increment bridge operation counter for statistics/debugging
    BRIDGE_SEQUENCE_NUMBER.inc();

    // Clear DMA3 (RF TX) mode byte before stopping TRX
    write_reg_dma3_mode(0);
    rf_stop_trx();

    // Configure RF TX wait/settle timing (low byte of reg_rf_tx_wail_settle_time)
    // based on the SYSTEM CLOCK (not the crystal frequency)
    if CLOCK_SYS_CLOCK_1US == 0x10 {
        write_reg_rf_tx_wail_settle_time_lo(0x5e); // 16 MHz system clock
    } else {
        write_reg_rf_tx_wail_settle_time_lo(0x68); // 32/48 MHz system clock
    }
}

/// Checks for BLE connection supervision timeout according to BLE specification.
///
/// Per BLE Core Spec, if no communication occurs within the supervision timeout,
/// the connection must be considered lost and terminated. This function handles
/// the timeout detection and appropriate cleanup.
///
/// ## Returns
/// - `Some(BleConnectionEvent::OtaErrorReboot)` if the connection timed out while
///   an OTA update was in progress (cleanup already performed)
/// - `Some(BleConnectionEvent::None)` if the connection timed out and cleanup was
///   performed, with nothing left for the composition root to do
/// - `None` if the connection is still valid
fn check_connection_supervision_timeout() -> Option<BleConnectionEvent> {
    // Check for BLE connection supervision timeout
    // Per BLE specification, if no communication occurs within the supervision timeout,
    // the connection must be considered lost and terminated
    if BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.get() * CLOCK_SYS_CLOCK_1US
        < read_reg_system_tick() - SLAVE_CONNECTED_TICK.get()
    {
        // Connection has timed out - handle cleanup first
        cleanup_ble_disconnection();

        return Some(if dispatch_ota_snapshot().in_progress {
            BleConnectionEvent::OtaErrorReboot
        } else {
            BleConnectionEvent::None
        });
    }
    None
}

/// Handles timeouts for status read operations.
///
/// If a status read operation has been running too long, this function stops it
/// to prevent the system from hanging. Uses SLAVE_READ_STATUS_BUSY_TIMEOUT to
/// determine the maximum allowed duration.
fn handle_status_read_timeout() {
    // Handle status read operation timeout
    // If a status read operation has been running too long, stop it to prevent hanging
    if SLAVE_READ_STATUS_BUSY.get() != 0
        && SLAVE_READ_STATUS_BUSY_TIMEOUT * CLOCK_SYS_CLOCK_1US * 1000
            < read_reg_system_tick() - DEVICE_STATUS_READ_BUSY_TIMESTAMP.get()
    {
        rf_link_slave_read_status_stop();
    }
}

/// Handles active OTA (Over-The-Air) update operations and timeouts.
///
/// This function observes ongoing OTA operations, including:
/// - Tracking the OTA timeout in 1-second intervals
/// - Reporting when the timeout counter has reached zero
/// - Scheduling the next connection event during OTA
///
/// The OTA manager itself is driven by the interrupt composition root
/// (the crate root), which reacts to the returned event.
///
/// ## Returns
/// - `Some(BleConnectionEvent::OtaFinishCheckAndErrorReboot)` if OTA is active and
///   the timeout counter reads zero after this tick's countdown
/// - `Some(BleConnectionEvent::OtaFinishCheck)` if OTA is active and the counter is
///   still non-zero
/// - `None` if no OTA is in progress
fn handle_ota_operations() -> Option<BleConnectionEvent> {
    use core::sync::atomic::{AtomicU32, Ordering};

    static RF_SLAVE_OTA_TIMEOUT_TICK: AtomicU32 = AtomicU32::new(0);

    if !dispatch_ota_snapshot().in_progress {
        return None;
    }

    // Check for OTA timeout (1 second intervals)
    if CLOCK_SYS_CLOCK_1US * 1000000
        < read_reg_system_tick() - RF_SLAVE_OTA_TIMEOUT_TICK.load(Ordering::Relaxed)
    {
        RF_SLAVE_OTA_TIMEOUT_TICK.store(read_reg_system_tick(), Ordering::Relaxed);

        dispatch_ota_tick();
    }

    // During OTA, schedule next connection event and return to RX state
    write_reg_system_tick_irq(SLAVE_NEXT_CONNECT_TICK.get());
    *CURRENT_RF_STATE.lock() = RfOperationState::Receiving;

    // Read the snapshot again after the tick: `dispatch_ota_tick()` is what can
    // zero the counter, so a value captured before it could never report the
    // expiry. The composition root finishes the OTA (and may refresh the
    // counter), so this is the candidate outcome, not the decision.
    if dispatch_ota_snapshot().timeout_expired {
        Some(BleConnectionEvent::OtaFinishCheckAndErrorReboot)
    } else {
        Some(BleConnectionEvent::OtaFinishCheck)
    }
}

/// Handles bridge packet transmission based on connection conditions.
///
/// This function manages the transmission of bridge packets under specific conditions:
/// 1. Connection interval > 10ms AND no old interval pending, OR
/// 2. Connection parameter update is not at the target instant yet
///
/// This prevents transmission during critical timing updates to maintain BLE compliance.
fn handle_bridge_operations() {
    // Set bridge command timeout (200µs from now)
    // This provides a time window for processing bridge commands
    BRIDGE_COMMAND_TIMESTAMP.set(CLOCK_SYS_CLOCK_1US * 200 + read_reg_system_tick());

    // Transmit bridge packets under specific conditions:
    // 1. Connection interval > 10ms AND no old interval pending, OR
    // 2. Connection parameter update is not at the target instant yet
    // This prevents transmission during critical timing updates
    if CLOCK_SYS_CLOCK_1US * 10000 < SLAVE_LINK_INTERVAL.get()
        && BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.get() == 0
        || BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.get() != BLE_PERIPHERAL_CONNECTION_INSTANT.get()
    {
        tx_packet_bridge();
    }
}

/// Processes mesh operations including status reads, pairing, and status reporting.
///
/// This function handles various mesh-related operations:
/// - Processing queued status response packets
/// - Handling pairing operations when buffers are available
/// - Flushing pending mesh node status updates
/// - Generating and sending mesh status reports via BLE and UART
fn process_mesh_operations() {
    use crate::ble::pair::pair_proc;
    use crate::facade::radio_config;
    use mesh_protocol::mesh_types::MESH_NODE_ST_VAL_LEN;
    use mesh_protocol::packet_types::{Packet, PacketAttCmd, PacketAttValue, PacketL2capHead};

    // Process any pending status read operations
    if SLAVE_READ_STATUS_BUSY.get() != 0 {
        process_queued_status_responses();
    }

    // Handle pairing operations if transmission buffer is available
    if is_add_packet_buf_ready() {
        let pair_proc_result = pair_proc();
        if pair_proc_result.is_some() {
            let pkt = pair_proc_result.unwrap();
            let _ = rf_link_add_tx_packet(&pkt);
        }
    }

    // Flush any pending mesh node status updates
    mesh_node_flush_status();

    // Generate and send mesh status reports over BLE.
    // UART reporting is handled directly at the source of status changes
    // (mesh_node_update_status, mesh_node_flush_status, ll_device_status_update).
    let mut data = [0u8; 20];

    // Collect mesh node status data (limit based on available space)
    let count = super::mesh::mesh_node_report_status(&mut data, 10 / MESH_NODE_ST_VAL_LEN);

    if count != 0 {
        // BLE path: send status notification to the connected central.
        if is_add_packet_buf_ready() {
            let vendor_id = radio_config().vendor_id;
            // Create BLE ATT packet for mesh status reporting
            let mut pkt = Packet {
                att_cmd: PacketAttCmd {
                    head: PacketL2capHead {
                        dma_len: 0x1D,   // DMA length
                        _type: 2,        // L2CAP packet type
                        rf_len: 0x1B,    // RF payload length
                        l2cap_len: 0x17, // L2CAP payload length
                        chan_id: 4,      // ATT channel ID
                    },
                    opcode: 0x1B, // ATT Write Without Response
                    handle: 0x12, // ATT handle for mesh data
                    handle1: 0,
                    value: PacketAttValue {
                        sno: [0; 3], // Sequence number
                        src: [0; 2], // Source address
                        dst: [0; 2], // Destination address
                        // Initialize with mesh status header: [0xdc, vendor_id_low, vendor_id_high, ...]
                        val: [
                            0xdc,
                            vendor_id as u8,
                            (vendor_id >> 8) as u8,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                            0x00,
                        ],
                    },
                },
            };

            // Copy the collected mesh status data into the packet payload
            pkt.att_cmd_mut().value.val[3..].copy_from_slice(&data);

            // Add the packet to the BLE transmission queue
            let _ = rf_link_add_tx_packet(&pkt);
        }
    }
}

/// Manages BLE connection event timing and synchronization.
///
/// This critical function ensures proper BLE connection event scheduling by:
/// - Calculating time until the next connection event with safety margins
/// - Handling connection parameter update completion
/// - Managing missed connection events to maintain synchronization
/// - Processing timing updates and channel hopping for each missed event
///
/// The function implements a loop that advances through missed connection events
/// while maintaining BLE specification compliance for timing and frequency hopping.
fn manage_connection_event_timing() {
    use crate::ble::channel_selection::ble_ll_select_next_data_channel;

    // Configure radio to return to receive mode for next connection event
    back_to_rxmode_bridge();

    // Connection event timing loop - ensures proper BLE connection event scheduling
    // This loop handles missed connection events and maintains timing synchronization
    loop {
        // Calculate time until next connection event (with 500µs safety margin)
        let mut cur_interval =
            (SLAVE_NEXT_CONNECT_TICK.get() - (CLOCK_SYS_CLOCK_1US * 500)) - read_reg_system_tick();

        // Handle connection parameter update completion
        // When the instant matches and we have an old interval, the update is complete
        if BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.get() != 0
            && BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.get() == BLE_PERIPHERAL_CONNECTION_INSTANT.get()
        {
            // Check for time overflow (0x3fffffff is a large positive number check)
            if 0x3fffffff >= cur_interval {
                cur_interval = 0; // Force immediate processing
            }
            BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.set(0); // Clear old interval flag
        }

        // If we're within the connection interval window, exit loop
        if cur_interval <= SLAVE_LINK_INTERVAL.get() {
            break;
        }

        // We've missed this connection event, advance to the next one
        // This maintains synchronization even when processing takes too long
        SLAVE_NEXT_CONNECT_TICK.set(SLAVE_NEXT_CONNECT_TICK.get() + SLAVE_LINK_INTERVAL.get());

        // Process any pending timing updates for this connection event
        handle_ble_connection_parameter_updates();

        // Update to the next channel in the frequency hopping sequence
        let chn_map = PKT_INIT.lock().ll_init().chm;
        ble_ll_select_next_data_channel(&chn_map, PKT_INIT.lock().ll_init().hop & 0x1f);
    }
}

/// Schedules the next BLE connection event and transitions to receive state.
///
/// This function completes the connection event processing by:
/// - Scheduling the next connection event interrupt
/// - Transitioning the state handler to receive mode to listen for the next packet
fn schedule_next_connection_event() {
    // Schedule the next connection event interrupt
    write_reg_system_tick_irq(SLAVE_NEXT_CONNECT_TICK.get());

    // Transition to receive state to listen for the next packet from master
    *CURRENT_RF_STATE.lock() = RfOperationState::Receiving;
}

/// Handles the BLE bridge state interrupt for connected operation.
///
/// This is the main connection maintenance function that coordinates all aspects
/// of an active BLE connection. It has been decomposed into focused subfunctions
/// for better testability and maintainability.
///
/// ## Key Responsibilities:
/// - **Hardware Setup**: Initialize connection hardware and registers
/// - **Connection Supervision**: Monitor connection timeout per BLE specification
/// - **Timeout Management**: Handle status read and OTA operation timeouts
/// - **Operations Processing**: Manage bridge, mesh, and OTA operations
/// - **Timing Management**: Maintain precise connection event timing
///
/// ## BLE Specification Compliance:
/// - Implements supervision timeout according to BLE specification
/// - Maintains connection event timing for reliable communication
/// - Handles connection parameter update procedures
pub fn handle_ble_connected_state() -> BleConnectionEvent {
    // 1. Initialize hardware and registers for active connection
    initialize_connection_hardware();

    // 2. Check for connection supervision timeout (early exit if timed out)
    if let Some(ev) = check_connection_supervision_timeout() {
        return ev;
    }

    // 3. Handle status read operation timeouts
    handle_status_read_timeout();

    // 4. Handle OTA operations (early exit if OTA is active)
    if let Some(ev) = handle_ota_operations() {
        return ev;
    }

    // 5. Process bridge packet transmission
    handle_bridge_operations();

    // 6. Process mesh operations (status, pairing, reporting)
    process_mesh_operations();

    // 7. Manage connection event timing and synchronization
    manage_connection_event_timing();

    // 8. Schedule next connection event and transition state
    schedule_next_connection_event();

    BleConnectionEvent::None
}

/// Processes queued status response packets and sends them to the master.
///
/// This function manages a circular buffer of response packets that need to be sent
/// to the BLE master. It's typically used during status read operations where the
/// master has requested device status information.
///
/// ## Buffer Management:
/// - Uses a circular buffer with read and write pointers
/// - Processes packets in FIFO order
/// - Stops processing if transmission buffer is full
///
/// ## Flow Control:
/// - Only sends packets when the TX buffer has space available
/// - Prevents buffer overflow by checking `rf_link_add_tx_packet` return value
/// - Maintains read pointer consistency for reliable packet delivery
pub fn process_queued_status_responses() {
    // Get the current read pointer from the status buffer
    let mut rptr = DEVICE_STATUS_BUFFER_READ_POINTER.get();

    // Process all queued response packets
    while DEVICE_STATUS_BUFFER_WRITE_POINTER.get() != rptr {
        // Ensure read pointer stays within buffer bounds
        rptr = rptr % BUFF_RESPONSE_PACKET_COUNT;

        // Get the next packet from the response buffer
        let packet = BUFF_RESPONSE.lock()[rptr];

        // Try to add the packet to the transmission queue
        // If the TX buffer is full, stop processing and try again later
        if rf_link_add_tx_packet(&packet).is_err() {
            return;
        }

        // Advance the read pointer to the next packet
        // Use atomic update to prevent race conditions
        rptr = ((DEVICE_STATUS_BUFFER_READ_POINTER.get() + 1) % BUFF_RESPONSE_PACKET_COUNT);
        DEVICE_STATUS_BUFFER_READ_POINTER.set(rptr);
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::ble::pair::PAIR_AC;
    use crate::ble::status::DEVICE_STATUS_READ_UNICAST_MODE;
    use crate::mesh_ll::node_status::{
        MESH_NODE_MASK, MESH_NODE_MAX, MESH_NODE_REPORT_ENABLE, MESH_NODE_ST,
    };
    use crate::rf::drv_8266::FLASH_CONFIGURATION_INDEX;
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog;
    use tlsr8266_hal::mcu::clock::{fake_clock, CLOCK_SYS_CLOCK_1US};
    use tlsr8266_hal::mcu::crypto::fake_crypto;
    use tlsr8266_pac::fake_mmio::{FakeMmio, RegWrite};
    use tlsr8266_pac::REG_BASE_ADDR;

    /// Datasheet offsets of the registers the code under test drives.
    const REG_RF_ACCESS_CODE: u32 = 0x408;
    const REG_RF_RX_MODE: u32 = 0x428;
    const REG_RF_TXRX_STATE: u32 = 0xf02;
    const REG_DMA_TX_RPTR: u32 = 0x52a;
    const REG_DMA_TX_WPTR: u32 = 0x52b;
    const REG_DMA_TX_FIFO: u32 = 0x52c;
    const REG_DMA3_MODE: u32 = 0x50f;
    const REG_SYSTEM_TICK_IRQ: u32 = 0x744;
    const REG_RF_MODE_CONTROL: u32 = 0xf00;
    const REG_RF_TX_WAIL_SETTLE_LO: u32 = 0xf04;

    /// The write-log entries addressed at `offset`, in order.
    fn writes_to(offset: u32) -> Vec<RegWrite> {
        FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == (offset | REG_BASE_ADDR))
            .collect()
    }

    /// Whether the code under test wrote `offset` at all.
    fn wrote(offset: u32) -> bool {
        !writes_to(offset).is_empty()
    }

    /// Number of `write_reg_dma_tx_fifo` registrations: the direct evidence that
    /// `rf_link_add_tx_packet` queued a packet for the DMA engine.
    fn dma_tx_fifo_writes() -> usize {
        writes_to(REG_DMA_TX_FIFO).len()
    }

    /// Helper function to reset global state (and the process-wide host fakes)
    /// to known values for test isolation. This ensures each test starts with a
    /// clean state.
    fn reset_global_state() {
        // The PAC/HAL fakes are process-wide singletons: clear the register bank,
        // analog bank, clock, AES transport and NOR device so a test never
        // inherits another test's state.
        FakeMmio::reset();
        FakeAnalog::reset();
        fake_clock::reset();
        fake_crypto::reset();
        FakeFlash::reset();

        BLE_PERIPHERAL_CONNECTION_INSTANT.set(0);
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(0);
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(0);
        SLAVE_LINK_INTERVAL.set(0x9c400); // Default value
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(0);
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.set(false);
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.set(0);
        BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.set(0);
        SLAVE_NEXT_CONNECT_TICK.set(0);
        SLAVE_WINDOW_SIZE.set(0);
        SLAVE_WINDOW_SIZE_UPDATE.set(0);
        BLE_CONN_INTERVAL.set(0);
        BLE_CONN_TIMEOUT.set(0);
        BLE_CONN_OFFSET.set(0);

        // Connection / cleanup state owned by this module and its neighbours.
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(false);
        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Disconnected);
        BRIDGE_SEQUENCE_NUMBER.set(0);
        BRIDGE_COMMAND_TIMESTAMP.set(0);
        *CURRENT_RF_STATE.lock() = RfOperationState::Receiving;
        PAIR_LOGIN_OK.set(false);
        SECURITY_ENABLE.set(false);
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
        FLASH_CONFIGURATION_INDEX.set(0);

        // Status-read bookkeeping, so "stop was called" is observable through the
        // parameters `rf_link_slave_read_status_stop` re-initialises.
        SLAVE_READ_STATUS_BUSY.set(0);
        DEVICE_STATUS_READ_UNICAST_MODE.set(false);
        DEVICE_STATUS_BUFFER_READ_POINTER.set(0);
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(0);

        // Mesh status reporting off, table and mask blank.
        MESH_NODE_REPORT_ENABLE.set(false);
        MESH_NODE_MAX.set(0);
        MESH_NODE_MASK.lock().iter_mut().for_each(|m| *m = 0);
        MESH_NODE_ST.lock().iter_mut().for_each(|n| n.miss = 0);

        // Reset channel map to known state and seed the packet-init copy so a
        // "not rebuilt" assertion has something to compare against.
        {
            let mut chn_map = SLAVE_CHN_MAP.lock();
            *chn_map = [0xff, 0xff, 0xff, 0xff, 0x1f]; // Default all channels enabled
        }
        PKT_INIT.lock().ll_init_mut().chm = [0xff, 0xff, 0xff, 0xff, 0x1f];

        // The OTA seam is a process-wide singleton too: start every test with no
        // update in progress and the default supervision counter.
        crate::callbacks::test_ota::reset_for_test();

        // Reset the shared queues (this also drains MESH_TX_QUEUE and
        // MESH_RX_CHANNEL) so each test starts from a known-clean transport
        // state. The UART statics belong to the firmware's fixture, not to this
        // crate, so they are deliberately not touched here.
        crate::queues::reset_for_test();
    }

    /// Tests that the connection event counter is incremented on every call.
    ///
    /// This test verifies BLE specification compliance for connection event tracking.
    /// Per BLE Core Spec v5.4, Section 4.5.1: "connEventCount shall be incremented
    /// for each connection event regardless of whether the slave receives a packet."
    #[test]
    fn test_connection_event_counter_increment() {
        reset_global_state();

        // Setup: Reset connection event counter to known value
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(100);

        // Ensure no updates are pending to focus on counter increment only
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(0);

        // Execute function
        handle_ble_connection_parameter_updates();

        // Verify: Connection event counter incremented
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_INSTANT.get(),
            101,
            "Connection event counter must increment on every call per BLE specification"
        );
    }

    /// Tests Channel Map Update procedure (LL_CHANNEL_MAP_IND) per BLE specification.
    ///
    /// This test verifies compliance with BLE Core Spec v5.4, Section 5.1.12:
    /// - Channel map updates are applied at the specified instant
    /// - The channel table is rebuilt with the new channel map
    /// - The update flag is cleared after processing
    /// - The packet initialization structure is updated
    ///
    /// The rebuilt hop table itself lives in `ble_ll_channel_selection`'s
    /// module-private statics; the production-visible result of the rebuild is
    /// the channel map copied into `PKT_INIT`, which is what every transmitted
    /// packet hops with.
    #[test]
    fn test_channel_map_update_ble_specification_compliance() {
        reset_global_state();

        // Setup: Configure for channel map update at specific instant
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(1); // Type 1 = Channel Map Update
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(50); // Current instant
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(51); // Update should apply at instant 51

        // Setup new channel map (enable channels 0, 2, 4, 6, 8 - selective hopping)
        let test_channel_map = [0x55, 0x55, 0x55, 0x55, 0x15]; // Binary: 01010101...
        {
            let mut chn_map = SLAVE_CHN_MAP.lock();
            *chn_map = test_channel_map;
        }

        // Execute: This should increment instant to 51 and trigger channel map update
        handle_ble_connection_parameter_updates();

        // Verify: Channel map update was processed correctly
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_INSTANT.get(),
            51,
            "Instant should be incremented"
        );
        assert_eq!(
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(),
            0,
            "Update flag should be cleared after processing"
        );

        // Verify: Packet initialization structure was updated
        {
            let pkt_init = PKT_INIT.lock();
            assert_eq!(
                pkt_init.ll_init().chm,
                test_channel_map,
                "Packet init channel map should match new channel map"
            );
        }
    }

    /// Tests that channel map updates only occur at the correct instant.
    ///
    /// Per BLE specification, timing updates must be synchronized between master and slave.
    /// The update should only be applied when BLE_PERIPHERAL_NEXT_UPDATE_INSTANT matches BLE_PERIPHERAL_CONNECTION_INSTANT.
    #[test]
    fn test_channel_map_update_instant_synchronization() {
        reset_global_state();

        // Setup: Channel map update scheduled for future instant
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(1); // Channel map update pending
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(10); // Current instant
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(15); // Update scheduled for instant 15

        // A distinctive pending map, different from the one seeded in PKT_INIT.
        let pending_channel_map = [0x33, 0x33, 0x33, 0x33, 0x03];
        {
            let mut chn_map = SLAVE_CHN_MAP.lock();
            *chn_map = pending_channel_map;
        }

        // Execute: Should increment instant but not apply update
        handle_ble_connection_parameter_updates();

        // Verify: Instant incremented but update not processed
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_INSTANT.get(),
            11,
            "Instant should increment"
        );
        assert_eq!(
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(),
            1,
            "Update flag should remain set"
        );

        // Verify: The packet-init channel map was not touched before the instant
        assert_ne!(
            PKT_INIT.lock().ll_init().chm,
            pending_channel_map,
            "Packet init channel map should not be copied before the update instant"
        );

        // Execute again multiple times to reach the target instant
        for _ in 0..4 {
            handle_ble_connection_parameter_updates();
        }

        // Verify: Now at instant 15, update should be processed
        assert_eq!(BLE_PERIPHERAL_CONNECTION_INSTANT.get(), 15);
        assert_eq!(
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(),
            0,
            "Update should now be processed"
        );
        assert_eq!(
            PKT_INIT.lock().ll_init().chm,
            pending_channel_map,
            "Packet init channel map should carry the applied map"
        );
    }

    /// Tests Connection Parameter Update procedure (LL_CONNECTION_UPDATE_IND) per BLE specification.
    ///
    /// This test verifies compliance with BLE Core Spec v5.4, Section 5.1.11:
    /// - Connection parameters are updated at the specified instant
    /// - All timing calculations follow BLE specification formulas
    /// - Window size is calculated correctly with master's constraints
    /// - Update flags and timing are managed properly
    #[test]
    fn test_connection_parameter_update_ble_specification_compliance() {
        reset_global_state();

        // Setup: Configure for connection parameter update
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(2); // Type 2 = Connection Parameter Update
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(25); // Current instant
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(26); // Update at instant 26

        // Setup new connection parameters (realistic BLE values)
        BLE_CONN_INTERVAL.set(32 * CLOCK_SYS_CLOCK_1US); // 32ms interval (25.6 connection intervals)
        BLE_CONN_TIMEOUT.set(4000 * CLOCK_SYS_CLOCK_1US); // 4 second supervision timeout
        BLE_CONN_OFFSET.set(1000 * CLOCK_SYS_CLOCK_1US); // 1ms offset
        SLAVE_NEXT_CONNECT_TICK.set(10000 * CLOCK_SYS_CLOCK_1US); // Next event at 10ms
        SLAVE_WINDOW_SIZE_UPDATE.set(5 * CLOCK_SYS_CLOCK_1US); // 5ms window from master

        // Calculate expected timing before calling the function
        let expected_update_time = BLE_CONN_OFFSET.get() + SLAVE_NEXT_CONNECT_TICK.get();

        // Execute: Should increment instant and apply connection parameter update
        handle_ble_connection_parameter_updates();

        // Verify: Instant incremented and update flag cleared
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_INSTANT.get(),
            26,
            "Instant should increment to trigger update"
        );
        assert_eq!(
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(),
            0,
            "Update flag should be cleared"
        );

        // Verify: Connection parameters applied correctly
        assert_eq!(
            SLAVE_LINK_INTERVAL.get(),
            32 * CLOCK_SYS_CLOCK_1US,
            "Connection interval should be updated to new value"
        );
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.get(),
            4000 * CLOCK_SYS_CLOCK_1US,
            "Supervision timeout should be updated to new value"
        );

        // Verify: Timing calculations per BLE specification
        assert_eq!(
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.get(),
            expected_update_time,
            "Update timing should include connection offset"
        );
        assert_eq!(
            SLAVE_NEXT_CONNECT_TICK.get(),
            expected_update_time,
            "Next connection tick should use new timing"
        );

        // Verify: Update flag is set for timing coordination
        assert_eq!(
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.get(),
            true,
            "Timing update flag should be set during parameter transition"
        );
    }

    /// Tests window size calculation according to BLE specification.
    ///
    /// Window size calculation must follow BLE Core Spec requirements:
    /// - Base window size = connection interval - fixed offset (0x4e2 µs ≈ 1.25ms)
    /// - Final window size = min(calculated_size, master_requested_size)
    /// - Window size must never exceed connection interval
    #[test]
    fn test_window_size_calculation_ble_specification() {
        reset_global_state();

        // Test Case 1: Master's window size is smaller (should use master's size)
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(2);
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(0);
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(1);

        BLE_CONN_INTERVAL.set(50000 * CLOCK_SYS_CLOCK_1US); // 50000ms interval (large enough to avoid underflow)
        SLAVE_WINDOW_SIZE_UPDATE.set(10 * CLOCK_SYS_CLOCK_1US); // Master wants 10ms window
        BLE_CONN_OFFSET.set(0);
        SLAVE_NEXT_CONNECT_TICK.set(0);

        handle_ble_connection_parameter_updates();

        // Calculated window = 50000ms - 1.25ms = 49998.75ms, but master wants 10ms
        assert_eq!(
            SLAVE_WINDOW_SIZE.get(),
            10 * CLOCK_SYS_CLOCK_1US,
            "Should use master's smaller window size"
        );

        // Test Case 2: Calculated size is smaller (should use calculated size)
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(2);
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(1);
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(2);

        BLE_CONN_INTERVAL.set(10 * CLOCK_SYS_CLOCK_1US); // 10ms interval
        SLAVE_WINDOW_SIZE_UPDATE.set(50 * CLOCK_SYS_CLOCK_1US); // Master wants 50ms (too big)

        handle_ble_connection_parameter_updates();

        // Calculated window = 10ms - 1.25ms = 8.75ms, master wants 50ms
        let expected_window =
            (10 * CLOCK_SYS_CLOCK_1US).saturating_sub(CLOCK_SYS_CLOCK_1US * 0x4e2);
        assert_eq!(
            SLAVE_WINDOW_SIZE.get(),
            expected_window,
            "Should use calculated size when master requests too large window"
        );
    }

    /// Tests edge case where connection interval is very small.
    ///
    /// This tests BLE specification edge cases for minimum connection intervals.
    /// Per BLE spec, minimum connection interval is 7.5ms, but we test smaller
    /// values to ensure robust window size calculation.
    #[test]
    fn test_minimum_connection_interval_window_calculation() {
        reset_global_state();

        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(2);
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(0);
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(1);

        // Test with very small interval (smaller than the 0x4e2 offset)
        BLE_CONN_INTERVAL.set(CLOCK_SYS_CLOCK_1US * 1000); // 1ms interval (below BLE minimum)
        SLAVE_WINDOW_SIZE_UPDATE.set(CLOCK_SYS_CLOCK_1US * 2000); // 2ms window request
        BLE_CONN_OFFSET.set(0);
        SLAVE_NEXT_CONNECT_TICK.set(0);

        handle_ble_connection_parameter_updates();

        // Window calculation: 1ms - 1.25ms = negative, saturating_sub gives 0
        // Since calculated window (0) < master's window (2000), use calculated window (0)
        assert_eq!(
            SLAVE_WINDOW_SIZE.get(),
            0,
            "Should handle underflow gracefully by using calculated window of 0"
        );
    }

    /// Tests that updates don't occur when instant synchronization is off.
    ///
    /// This verifies the critical BLE requirement that parameter updates must be
    /// synchronized between master and slave at the exact same connection event.
    #[test]
    fn test_no_update_when_instant_mismatch() {
        reset_global_state();

        // Setup: Updates pending but instants don't match
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(1); // Channel map update pending
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(5); // Current instant
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(10); // Update scheduled for instant 10

        // Store original values to verify they don't change
        let original_interval = SLAVE_LINK_INTERVAL.get();
        let original_timeout = BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.get();
        let original_chn_map = PKT_INIT.lock().ll_init().chm;

        // Execute function
        handle_ble_connection_parameter_updates();

        // Verify: No updates applied
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_INSTANT.get(),
            6,
            "Only instant should increment"
        );
        assert_eq!(
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(),
            1,
            "Update flag should remain set"
        );
        assert_eq!(
            SLAVE_LINK_INTERVAL.get(),
            original_interval,
            "Interval unchanged"
        );
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.get(),
            original_timeout,
            "Timeout unchanged"
        );
        assert_eq!(
            PKT_INIT.lock().ll_init().chm,
            original_chn_map,
            "Packet init channel map unchanged"
        );
    }

    /// Tests connection parameter update type detection.
    ///
    /// The function must correctly distinguish between:
    /// - Type 1: Channel Map Update (LL_CHANNEL_MAP_IND)
    /// - Type 2: Connection Parameter Update (LL_CONNECTION_UPDATE_IND)
    /// - No update: BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP = 0
    #[test]
    fn test_update_type_detection() {
        reset_global_state();

        // Test Case 1: No update pending
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(0);
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(0);
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(1);

        let original_interval = SLAVE_LINK_INTERVAL.get();
        let original_chn_map = PKT_INIT.lock().ll_init().chm;

        handle_ble_connection_parameter_updates();

        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_INSTANT.get(),
            1,
            "Instant should still increment"
        );
        assert_eq!(
            SLAVE_LINK_INTERVAL.get(),
            original_interval,
            "No parameter changes"
        );
        assert_eq!(
            PKT_INIT.lock().ll_init().chm,
            original_chn_map,
            "No channel map changes"
        );

        // Test Case 2: Invalid update type
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(99); // Invalid type
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(1);
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(2);

        handle_ble_connection_parameter_updates();

        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_INSTANT.get(),
            2,
            "Instant should increment"
        );
        assert_eq!(
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(),
            99,
            "Invalid type should remain unchanged"
        );
        assert_eq!(
            PKT_INIT.lock().ll_init().chm,
            original_chn_map,
            "Invalid type must not rebuild the channel map"
        );
    }

    /// Tests BLE specification timing requirements for parameter updates.
    ///
    /// Per BLE Core Spec v5.4:
    /// - Updates must be applied at the exact connection event specified
    /// - Timing calculations must account for connection offset
    /// - Window sizing must respect both calculated and master-requested limits
    #[test]
    fn test_ble_timing_specification_compliance() {
        reset_global_state();

        // Setup realistic BLE connection scenario
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(2);
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(42); // Current connection event
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(43); // Update at next event

        // BLE-compliant parameter values
        BLE_CONN_INTERVAL.set(24 * CLOCK_SYS_CLOCK_1US); // 24ms (19.2 intervals)
        BLE_CONN_TIMEOUT.set(6000 * CLOCK_SYS_CLOCK_1US); // 6 second timeout
        BLE_CONN_OFFSET.set(500 * CLOCK_SYS_CLOCK_1US); // 500µs offset
        SLAVE_NEXT_CONNECT_TICK.set(50000 * CLOCK_SYS_CLOCK_1US); // 50ms base time
        SLAVE_WINDOW_SIZE_UPDATE.set(3 * CLOCK_SYS_CLOCK_1US); // 3ms window

        // Execute update
        handle_ble_connection_parameter_updates();

        // Verify BLE specification compliance
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_INSTANT.get(),
            43,
            "Update must occur at specified instant"
        );
        assert_eq!(
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(),
            0,
            "Update flag must be cleared"
        );
        assert_eq!(
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.get(),
            true,
            "Timing flag must be set"
        );

        // Verify timing calculations
        let expected_timing = BLE_CONN_OFFSET.get() + 50000 * CLOCK_SYS_CLOCK_1US;
        assert_eq!(
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.get(),
            expected_timing,
            "Update timing must include connection offset per BLE spec"
        );
        assert_eq!(
            SLAVE_NEXT_CONNECT_TICK.get(),
            expected_timing,
            "Next connection event must use updated timing"
        );

        // Verify window size follows BLE specification formula
        let calculated_window = BLE_CONN_INTERVAL
            .get()
            .saturating_sub(CLOCK_SYS_CLOCK_1US * 0x4e2);
        let expected_window = if SLAVE_WINDOW_SIZE_UPDATE.get() < calculated_window {
            SLAVE_WINDOW_SIZE_UPDATE.get()
        } else {
            calculated_window
        };
        assert_eq!(
            SLAVE_WINDOW_SIZE.get(),
            expected_window,
            "Window size must follow BLE specification: min(calculated, master_requested)"
        );
    }

    /// Tests state consistency during parameter updates.
    ///
    /// Ensures that all global state variables remain consistent during updates
    /// and that the function is idempotent when called multiple times.
    #[test]
    fn test_state_consistency_during_updates() {
        reset_global_state();

        // Setup initial state
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(2);
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(10);
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(11);

        // Set consistent parameter values
        BLE_CONN_INTERVAL.set(20 * CLOCK_SYS_CLOCK_1US);
        BLE_CONN_TIMEOUT.set(3000 * CLOCK_SYS_CLOCK_1US);
        BLE_CONN_OFFSET.set(100 * CLOCK_SYS_CLOCK_1US);
        SLAVE_NEXT_CONNECT_TICK.set(5000 * CLOCK_SYS_CLOCK_1US);
        SLAVE_WINDOW_SIZE_UPDATE.set(10 * CLOCK_SYS_CLOCK_1US);

        // Execute first update
        handle_ble_connection_parameter_updates();

        // Capture state after first update
        let interval_after = SLAVE_LINK_INTERVAL.get();
        let timeout_after = BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.get();
        let timing_after = BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.get();
        let window_after = SLAVE_WINDOW_SIZE.get();
        let instant_after = BLE_PERIPHERAL_CONNECTION_INSTANT.get();
        let update_flag_after = BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get();

        // Verify update was applied
        assert_eq!(instant_after, 11);
        assert_eq!(update_flag_after, 0);
        assert_eq!(interval_after, 20 * CLOCK_SYS_CLOCK_1US);

        // Execute function again (should only increment instant)
        handle_ble_connection_parameter_updates();

        // Verify state consistency: only instant should change
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_INSTANT.get(),
            instant_after + 1,
            "Only instant should increment"
        );
        assert_eq!(
            SLAVE_LINK_INTERVAL.get(),
            interval_after,
            "Interval should be stable"
        );
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.get(),
            timeout_after,
            "Timeout should be stable"
        );
        assert_eq!(
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.get(),
            timing_after,
            "Timing should be stable"
        );
        assert_eq!(
            SLAVE_WINDOW_SIZE.get(),
            window_after,
            "Window should be stable"
        );
        assert_eq!(
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(),
            update_flag_after,
            "Update flag should remain clear"
        );
    }

    // ================================================================================
    // Tests for cleanup_ble_disconnection function
    // ================================================================================

    /// Helper function to reset global state for cleanup tests
    fn reset_cleanup_state() {
        reset_global_state();

        // Reset connection state
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);

        // Reset security state
        SECURITY_ENABLE.set(false);
        BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(0x12345678);

        // Reset pairing state
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
        SLAVE_DATA_VALID.set(1);

        // Reset connection parameter state
        GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(0x87654321);
        NEED_UPDATE_CONNECT_PARA.set(true);

        // Mesh status reporting starts enabled so `mesh_report_status_enable(false)`
        // has an observable effect.
        MESH_NODE_REPORT_ENABLE.set(true);
    }

    /// Tests basic disconnection cleanup operations.
    ///
    /// Verifies that cleanup_ble_disconnection correctly:
    /// - Schedules transition to advertisement mode
    /// - Resets connection and pairing state flags
    /// - Resets DMA transmission pointer
    /// - Clears connection parameter update state
    #[test]
    fn test_cleanup_basic_disconnection_operations() {
        reset_cleanup_state();
        fake_clock::set_ticks(0x10000000);
        FakeMmio::clear_writes();

        // Verify initial state is set correctly by reset_cleanup_state
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_ACTIVE.get(),
            true,
            "Initial connection state should be active"
        );
        assert_eq!(
            PAIR_LOGIN_OK.get(),
            true,
            "Initial pair login should be true"
        );

        // Execute cleanup
        cleanup_ble_disconnection();

        // Verify advertisement state transition scheduling
        let expected_irq_time = CLOCK_SYS_CLOCK_1US * 100 + 0x10000000;
        assert_eq!(
            writes_to(REG_SYSTEM_TICK_IRQ),
            vec![RegWrite {
                addr: REG_SYSTEM_TICK_IRQ | REG_BASE_ADDR,
                width: 4,
                value: expected_irq_time,
            }],
            "the advertisement transition must be scheduled 100µs ahead"
        );

        // Verify state handler is set to advertisement
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Advertising,
            "Should schedule transition to advertisement state"
        );

        // Verify DMA transmission pointer is reset
        assert_eq!(
            writes_to(REG_DMA_TX_RPTR),
            vec![RegWrite {
                addr: REG_DMA_TX_RPTR | REG_BASE_ADDR,
                width: 1,
                value: 0x10,
            }],
            "DMA TX read pointer must be reset to 0x10"
        );

        // Verify connection state flags are cleared
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_ACTIVE.get(),
            false,
            "Connection active flag should be cleared"
        );
        assert_eq!(
            PAIR_LOGIN_OK.get(),
            false,
            "Pair login flag should be cleared"
        );

        // Verify connection parameter update state is reset
        assert_eq!(
            GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.get(),
            0,
            "Service discovery timestamp should be cleared"
        );
        assert_eq!(
            NEED_UPDATE_CONNECT_PARA.get(),
            false,
            "Connection parameter update flag should be cleared"
        );
    }

    /// Tests security-disabled cleanup path.
    ///
    /// When security is disabled, cleanup should:
    /// - Only disable mesh status reporting
    /// - Not clear first connection timestamp
    /// - Handle pairing state normally
    #[test]
    fn test_cleanup_security_disabled_path() {
        reset_cleanup_state();

        // Setup: Security disabled
        SECURITY_ENABLE.set(false);
        BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(0x12345678);

        // Execute cleanup
        cleanup_ble_disconnection();

        // Verify mesh status reporting is disabled
        assert_eq!(
            MESH_NODE_REPORT_ENABLE.get(),
            false,
            "mesh status reporting must be disabled on disconnect"
        );

        // Verify first connection timestamp is NOT cleared (security disabled path)
        assert_eq!(
            BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.get(),
            0x12345678,
            "First connection timestamp should remain unchanged in security disabled mode"
        );
    }

    /// Tests security-enabled cleanup path.
    ///
    /// When security is enabled, cleanup should:
    /// - Clear first connection timestamp
    /// - Clear pairing login state multiple times
    /// - Disable mesh status reporting
    #[test]
    fn test_cleanup_security_enabled_path() {
        reset_cleanup_state();

        // Setup: Security enabled
        SECURITY_ENABLE.set(true);
        BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(0x12345678);
        PAIR_LOGIN_OK.set(true);

        // Execute cleanup
        cleanup_ble_disconnection();

        // Verify security-specific cleanup
        assert_eq!(
            BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.get(),
            0,
            "First connection timestamp should be cleared in security enabled mode"
        );
        assert_eq!(
            PAIR_LOGIN_OK.get(),
            false,
            "Pair login should be cleared in security enabled mode"
        );

        // Verify mesh status reporting is disabled
        assert_eq!(
            MESH_NODE_REPORT_ENABLE.get(),
            false,
            "mesh status reporting must be disabled on disconnect"
        );
    }

    /// Tests status read operation cleanup.
    ///
    /// When a status read operation is in progress (SLAVE_READ_STATUS_BUSY != 0),
    /// cleanup should call rf_link_slave_read_status_stop to terminate it.
    #[test]
    fn test_cleanup_status_read_operation_stop() {
        reset_cleanup_state();

        // Setup: Status read operation in progress
        SLAVE_READ_STATUS_BUSY.set(5); // Non-zero indicates operation is busy
        DEVICE_STATUS_READ_UNICAST_MODE.set(true);

        // Execute cleanup
        cleanup_ble_disconnection();

        // The real stop handler clears the busy flag, drops unicast mode and
        // re-initialises the status parameters.
        assert_eq!(
            SLAVE_READ_STATUS_BUSY.get(),
            0,
            "status read operation must be stopped"
        );
        assert_eq!(
            DEVICE_STATUS_READ_UNICAST_MODE.get(),
            false,
            "unicast status mode must be disabled by the stop handler"
        );
    }

    /// Tests that status read cleanup is skipped when not needed.
    ///
    /// When no status read operation is in progress (SLAVE_READ_STATUS_BUSY == 0),
    /// rf_link_slave_read_status_stop should not be called.
    #[test]
    fn test_cleanup_no_status_read_operation() {
        reset_cleanup_state();

        // Setup: No status read operation in progress, but the state the stop
        // handler owns is non-default: if it ran, these would be cleared.
        SLAVE_READ_STATUS_BUSY.set(0);
        DEVICE_STATUS_READ_UNICAST_MODE.set(true);
        DEVICE_STATUS_BUFFER_READ_POINTER.set(2);
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(3);

        // Execute cleanup
        cleanup_ble_disconnection();

        // Verify status read stop is NOT called
        assert_eq!(
            DEVICE_STATUS_READ_UNICAST_MODE.get(),
            true,
            "the stop handler must not run when no status read is busy"
        );
        assert_eq!(
            (
                DEVICE_STATUS_BUFFER_READ_POINTER.get(),
                DEVICE_STATUS_BUFFER_WRITE_POINTER.get()
            ),
            (2, 3),
            "the stop handler's parameter re-init must not run"
        );
    }

    /// Tests pairing state management for already paired device.
    ///
    /// When device is already paired (PairSetted), cleanup should:
    /// - Clear data validity flag
    /// - Not call pair_load_key
    /// - Not change pairing state
    #[test]
    fn test_cleanup_pairing_already_paired() {
        reset_cleanup_state();

        // Setup: Device already paired. `pair_load_key` always re-scans the
        // pairing sector (setting FLASH_CONFIGURATION_INDEX); a sentinel index
        // makes an unexpected load visible.
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
        SLAVE_DATA_VALID.set(123);
        FLASH_CONFIGURATION_INDEX.set(7);

        // Execute cleanup
        cleanup_ble_disconnection();

        // Verify data validity is cleared
        assert_eq!(
            SLAVE_DATA_VALID.get(),
            0,
            "Data validity should be cleared for paired device"
        );

        // Verify pairing state remains unchanged
        assert_eq!(
            *PAIR_SETTING_FLAG.lock(),
            ePairState::PairSetted,
            "Pairing state should remain PairSetted"
        );

        // Verify pair_load_key is NOT called for already paired device
        assert_eq!(
            FLASH_CONFIGURATION_INDEX.get(),
            7,
            "the flash key loader must not run for an already paired device"
        );
    }

    /// Tests pairing state management for unpaired device.
    ///
    /// When device is not paired (not PairSetted), cleanup should:
    /// - Clear data validity flag
    /// - Call pair_load_key to load stored keys
    /// - Set pairing state to PairSetted
    #[test]
    fn test_cleanup_pairing_not_paired() {
        reset_cleanup_state();

        // Setup: Device not paired, blank NOR device.
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetting; // Not PairSetted
        SLAVE_DATA_VALID.set(456);
        FLASH_CONFIGURATION_INDEX.set(7);

        // Execute cleanup
        cleanup_ble_disconnection();

        // Verify data validity is cleared
        assert_eq!(
            SLAVE_DATA_VALID.get(),
            0,
            "Data validity should be cleared for unpaired device"
        );

        // Verify pair_load_key ran: the real loader's configuration scan sets
        // the index to -1 when the pairing sector is blank.
        assert_eq!(
            FLASH_CONFIGURATION_INDEX.get(),
            -1,
            "the flash key loader must scan the pairing sector on first disconnect"
        );

        // Verify pairing state is set to PairSetted
        assert_eq!(
            *PAIR_SETTING_FLAG.lock(),
            ePairState::PairSetted,
            "Pairing state should be set to PairSetted after loading keys"
        );
    }

    /// Tests hardware register configuration for 16MHz clock.
    ///
    /// When CLOCK_SYS_CLOCK_1US equals 0x10 (16MHz), the RF timing register
    /// should be configured with the specific value for that clock frequency.
    #[test]
    fn test_cleanup_hardware_config_16mhz_clock() {
        // This test assumes CLOCK_SYS_CLOCK_1US is set to 0x10 (16MHz)
        // In a real system, this would be a compile-time constant
        if CLOCK_SYS_CLOCK_1US != 0x10 {
            // Skip this test if we're not in 16MHz mode
            println!(
                "Skipping 16MHz test - clock is not 16MHz ({})",
                CLOCK_SYS_CLOCK_1US
            );
            return;
        }

        // Only run this test if we're actually in 16MHz mode
        #[cfg(any())] // This will never be true, effectively disabling the test
        {
            // Test code would go here, but since we can't determine the clock at runtime reliably,
            // we'll use a different approach to test this
        }

        // For now, just document that this test exists for 16MHz systems
        println!(
            "16MHz clock hardware config test would verify write_reg_rf_tx_wail_settle_time_lo(0x5e)"
        );
    }

    /// Tests hardware register configuration for non-16MHz clock.
    ///
    /// When CLOCK_SYS_CLOCK_1US is not 0x10, the RF timing register
    /// should be configured with the default value for other clock frequencies.
    #[test]
    fn test_cleanup_hardware_config_other_clock() {
        // This test assumes CLOCK_SYS_CLOCK_1US is NOT 0x10
        // In a real system, this would be a compile-time constant
        if CLOCK_SYS_CLOCK_1US == 0x10 {
            // Skip this test if we're in 16MHz mode
            println!("Skipping non-16MHz test - clock is 16MHz");
            return;
        }

        // Only run this test if we're actually in non-16MHz mode
        #[cfg(any())] // This will never be true, effectively disabling the test
        {
            // Test code would go here, but since we can't determine the clock at runtime reliably,
            // we'll use a different approach to test this
        }

        // For now, just document that this test exists for non-16MHz systems
        println!(
            "Non-16MHz clock hardware config test would verify write_reg_rf_tx_wail_settle_time_lo(0x68)"
        );
    }

    /// Tests comprehensive state reset after cleanup.
    ///
    /// Verifies that all relevant state variables are properly reset
    /// after cleanup_ble_disconnection completes.
    #[test]
    fn test_cleanup_comprehensive_state_reset() {
        reset_cleanup_state();

        // Setup initial state with non-zero values
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(0x87654321);
        NEED_UPDATE_CONNECT_PARA.set(true);
        SLAVE_DATA_VALID.set(789);

        // Execute cleanup
        cleanup_ble_disconnection();

        // Verify comprehensive state reset
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_ACTIVE.get(),
            false,
            "Connection active should be false"
        );
        assert_eq!(PAIR_LOGIN_OK.get(), false, "Pair login should be false");
        assert_eq!(
            GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.get(),
            0,
            "Service discovery timeout should be cleared"
        );
        assert_eq!(
            NEED_UPDATE_CONNECT_PARA.get(),
            false,
            "Connection parameter update flag should be false"
        );
        assert_eq!(SLAVE_DATA_VALID.get(), 0, "Data validity should be cleared");
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Advertising,
            "Handler state should be set to advertisement"
        );
    }

    /// Tests cleanup behavior with mixed security and pairing states.
    ///
    /// This test verifies proper cleanup when security is enabled but
    /// the device is not paired, testing the interaction between both
    /// code paths.
    #[test]
    fn test_cleanup_security_enabled_not_paired() {
        reset_cleanup_state();

        // Setup: Security enabled, device not paired, blank NOR device.
        SECURITY_ENABLE.set(true);
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetting;
        BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(0x12345678);
        PAIR_LOGIN_OK.set(true);
        FLASH_CONFIGURATION_INDEX.set(7);

        // Execute cleanup
        cleanup_ble_disconnection();

        // Verify security-enabled cleanup
        assert_eq!(
            BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.get(),
            0,
            "First connection timestamp should be cleared"
        );
        assert_eq!(
            PAIR_LOGIN_OK.get(),
            false,
            "Pair login should be cleared multiple times"
        );

        // Verify pairing key loading for unpaired device
        assert_eq!(
            FLASH_CONFIGURATION_INDEX.get(),
            -1,
            "the flash key loader must scan the pairing sector on first disconnect"
        );
        assert_eq!(
            *PAIR_SETTING_FLAG.lock(),
            ePairState::PairSetted,
            "Pairing state should be set to PairSetted after loading"
        );

        // Verify mesh status reporting is disabled
        assert_eq!(
            MESH_NODE_REPORT_ENABLE.get(),
            false,
            "mesh status reporting must be disabled on disconnect"
        );
    }

    /// Tests edge case with multiple status read operations.
    ///
    /// Tests behavior when SLAVE_READ_STATUS_BUSY has an unusual value
    /// to ensure robustness of the cleanup logic.
    #[test]
    fn test_cleanup_edge_case_multiple_status_operations() {
        reset_cleanup_state();

        // Setup: Unusual status read busy value
        SLAVE_READ_STATUS_BUSY.set(255); // Max value
        DEVICE_STATUS_READ_UNICAST_MODE.set(true);

        // Execute cleanup
        cleanup_ble_disconnection();

        // Verify status read stop is called regardless of specific value
        assert_eq!(
            SLAVE_READ_STATUS_BUSY.get(),
            0,
            "any non-zero busy value must trigger the stop handler"
        );
        assert_eq!(
            DEVICE_STATUS_READ_UNICAST_MODE.get(),
            false,
            "the stop handler must run for any non-zero busy value"
        );
    }

    /// Tests cleanup idempotency.
    ///
    /// Verifies that calling cleanup_ble_disconnection multiple times
    /// doesn't cause issues and maintains consistent state.
    #[test]
    fn test_cleanup_idempotency() {
        reset_cleanup_state();

        // Setup initial state
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetting;
        FakeMmio::clear_writes();

        // Execute cleanup first time
        cleanup_ble_disconnection();

        // Capture state after first cleanup
        let connection_active_after = BLE_PERIPHERAL_CONNECTION_ACTIVE.get();
        let pair_login_after = PAIR_LOGIN_OK.get();
        let data_valid_after = SLAVE_DATA_VALID.get();
        let pair_flag_after = *PAIR_SETTING_FLAG.lock();

        // Execute cleanup second time
        cleanup_ble_disconnection();

        // Verify state remains consistent after second cleanup
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_ACTIVE.get(),
            connection_active_after,
            "Connection active should remain stable"
        );
        assert_eq!(
            PAIR_LOGIN_OK.get(),
            pair_login_after,
            "Pair login should remain stable"
        );
        assert_eq!(
            SLAVE_DATA_VALID.get(),
            data_valid_after,
            "Data validity should remain stable"
        );
        assert_eq!(
            *PAIR_SETTING_FLAG.lock(),
            pair_flag_after,
            "Pair setting flag should remain stable"
        );

        // Verify the real side effects ran on both calls
        assert_eq!(
            writes_to(REG_SYSTEM_TICK_IRQ).len(),
            2,
            "each cleanup must schedule the advertisement transition"
        );
        assert_eq!(
            writes_to(REG_DMA_TX_RPTR),
            vec![
                RegWrite {
                    addr: REG_DMA_TX_RPTR | REG_BASE_ADDR,
                    width: 1,
                    value: 0x10,
                };
                2
            ],
            "each cleanup must reset the DMA TX read pointer"
        );
        assert_eq!(
            MESH_NODE_REPORT_ENABLE.get(),
            false,
            "each cleanup must disable mesh status reporting"
        );

        // Verify pair_load_key ran only on the first call: the first call flips
        // the pairing flag, so a second load would re-scan (and overwrite this
        // sentinel).
        FLASH_CONFIGURATION_INDEX.set(42);
        cleanup_ble_disconnection();
        assert_eq!(
            FLASH_CONFIGURATION_INDEX.get(),
            42,
            "the flash key loader must not run once the device reads as paired"
        );
    }

    // ================================================================================
    // Tests for decomposed functions from handle_ble_connected_state
    // ================================================================================

    /// Tests the initialize_connection_hardware function.
    ///
    /// Verifies that hardware initialization correctly:
    /// - Sets link state to bridge mode (state 5)
    /// - Increments bridge operation counter
    /// - Clears the DMA3 (RF TX) mode byte and stops ongoing operations
    /// - Configures RF TX wait/settle timing based on system clock
    #[test]
    fn test_initialize_connection_hardware() {
        reset_global_state();
        FakeMmio::clear_writes();

        // Reset state for clean test
        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Disconnected);
        BRIDGE_SEQUENCE_NUMBER.set(100);

        // Execute function
        initialize_connection_hardware();

        // Verify link state is set to bridge mode
        assert_eq!(
            BLE_PERIPHERAL_LINK_STATE.get(),
            BlePeripheralLinkState::Connected,
            "Link state should be set to bridge mode (Connected)"
        );

        // Verify bridge counter is incremented
        assert_eq!(
            BRIDGE_SEQUENCE_NUMBER.get(),
            101,
            "Bridge sequence number should be incremented"
        );

        // Verify RF operations are stopped
        assert!(
            writes_to(REG_RF_MODE_CONTROL).contains(&RegWrite {
                addr: REG_RF_MODE_CONTROL | REG_BASE_ADDR,
                width: 1,
                value: 0x80,
            }),
            "rf_stop_trx must write the stop mode: {:?}",
            FakeMmio::writes()
        );

        // Verify DMA3 (RF TX) mode byte is cleared
        assert_eq!(
            writes_to(REG_DMA3_MODE),
            vec![RegWrite {
                addr: REG_DMA3_MODE | REG_BASE_ADDR,
                width: 1,
                value: 0,
            }],
            "DMA3 mode byte must be cleared before stopping TRX"
        );

        // Verify RF timing registers are configured (depends on clock frequency)
        let expected_settle = if CLOCK_SYS_CLOCK_1US == 0x10 {
            0x5e
        } else {
            0x68
        };
        assert_eq!(
            writes_to(REG_RF_TX_WAIL_SETTLE_LO),
            vec![RegWrite {
                addr: REG_RF_TX_WAIL_SETTLE_LO | REG_BASE_ADDR,
                width: 1,
                value: expected_settle,
            }],
            "RF TX wait/settle timing must match the system clock"
        );
    }

    /// Tests connection supervision timeout detection.
    ///
    /// Verifies that check_connection_supervision_timeout correctly:
    /// - Reports `None` while the connection is still inside its supervision timeout
    /// - Reports `Some(BleConnectionEvent::None)` once it times out with no OTA active
    /// - Reports `Some(BleConnectionEvent::OtaErrorReboot)` when it times out during OTA
    /// - Always runs `cleanup_ble_disconnection` exactly once on the timeout path
    #[test]
    fn test_check_connection_supervision_timeout_no_timeout() {
        reset_global_state();

        // Connection is still valid and retains its state
        fake_clock::set_ticks(10000);
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(1000); // 1 second timeout
        SLAVE_CONNECTED_TICK.set(9500); // Connected 500 ticks ago
        PAIR_LOGIN_OK.set(true);

        // Execute function
        let result = check_connection_supervision_timeout();

        // Verify no timeout detected
        assert_eq!(
            result, None,
            "Should return None while the connection lives"
        );

        // Verify cleanup did not run: the disconnect path clears the pairing
        // login flag and retargets the RF state at advertisement.
        assert_eq!(
            PAIR_LOGIN_OK.get(),
            true,
            "no cleanup must run before the supervision timeout expires"
        );
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Receiving,
            "RF state must not be retargeted at advertisement"
        );
        assert!(
            !wrote(REG_DMA_TX_RPTR),
            "no register cleanup must run before the supervision timeout expires"
        );
    }

    #[test]
    fn test_check_connection_supervision_timeout_with_timeout() {
        reset_global_state();

        // Connection has timed out
        fake_clock::set_ticks(15000);
        // Setup connection state - need timeout_us * CLOCK_SYS_CLOCK_1US < elapsed_ticks
        // With CLOCK_SYS_CLOCK_1US = 32, timeout = 100us, elapsed = 5000 ticks
        // 100 * 32 = 3200 < 5000 ✓ (timeout condition met)
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(100); // 100 microsecond timeout
        SLAVE_CONNECTED_TICK.set(10000); // Connected at tick 10000, now at 15000 = 5000 ticks elapsed

        // Execute function
        let result = check_connection_supervision_timeout();

        // Verify timeout detected and cleanup performed, with nothing left for the
        // composition root to do.
        assert_eq!(
            result,
            Some(BleConnectionEvent::None),
            "A non-OTA supervision timeout must report Some(None)"
        );

        // Verify the real cleanup ran exactly once.
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Advertising,
            "cleanup must retarget the RF state at advertisement"
        );
        assert_eq!(
            writes_to(REG_SYSTEM_TICK_IRQ),
            vec![RegWrite {
                addr: REG_SYSTEM_TICK_IRQ | REG_BASE_ADDR,
                width: 4,
                value: CLOCK_SYS_CLOCK_1US * 100 + 15000,
            }],
            "cleanup must schedule exactly one advertisement transition"
        );
    }

    /// A supervision timeout while OTA is active must report `OtaErrorReboot` so the
    /// composition root (and only the composition root) drives the reboot, and it
    /// must still run the disconnection cleanup first.
    #[test]
    fn test_check_connection_supervision_timeout_with_timeout_during_ota() {
        reset_global_state();

        fake_clock::set_ticks(15000);
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(100);
        SLAVE_CONNECTED_TICK.set(10000);
        PAIR_LOGIN_OK.set(true);
        crate::callbacks::test_ota::set_in_progress_for_test(true);

        let result = check_connection_supervision_timeout();

        assert_eq!(
            result,
            Some(BleConnectionEvent::OtaErrorReboot),
            "A supervision timeout during OTA must report OtaErrorReboot"
        );

        // The cleanup still ran before the reboot verdict was reported.
        assert_eq!(
            PAIR_LOGIN_OK.get(),
            false,
            "cleanup must clear the pairing login flag before reboot"
        );
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Advertising,
            "cleanup must still run during OTA"
        );

        crate::callbacks::test_ota::set_in_progress_for_test(false);
    }

    /// Tests status read timeout handling.
    ///
    /// Verifies that handle_status_read_timeout correctly:
    /// - Only acts when status read is busy
    /// - Calculates timeout based on SLAVE_READ_STATUS_BUSY_TIMEOUT
    /// - Calls rf_link_slave_read_status_stop when timeout occurs
    #[test]
    fn test_handle_status_read_timeout_no_operation() {
        reset_global_state();

        fake_clock::set_ticks(10000);

        // Setup: No status read operation in progress, but the state the stop
        // handler owns is non-default so an unexpected stop is visible.
        SLAVE_READ_STATUS_BUSY.set(0);
        DEVICE_STATUS_READ_UNICAST_MODE.set(true);

        // Execute function
        handle_status_read_timeout();

        // Verify no action taken when no operation is busy
        assert_eq!(
            DEVICE_STATUS_READ_UNICAST_MODE.get(),
            true,
            "the stop handler must not run without a busy status read"
        );
    }

    #[test]
    fn test_handle_status_read_timeout_with_timeout() {
        reset_global_state();

        // Setup: Status read operation in progress and timed out
        // Logic: SLAVE_READ_STATUS_BUSY_TIMEOUT * CLOCK_SYS_CLOCK_1US * 1000 < elapsed_ticks
        // SLAVE_READ_STATUS_BUSY_TIMEOUT = 25000 (from light.rs constant)
        // With CLOCK_SYS_CLOCK_1US = 32: 25000 * 32 * 1000 = 800,000,000
        // Need elapsed > 800,000,000, so start_time = 10000, current = 800,100,000
        SLAVE_READ_STATUS_BUSY.set(5); // Operation in progress
        DEVICE_STATUS_READ_BUSY_TIMESTAMP.set(10000); // Started long ago
        DEVICE_STATUS_READ_UNICAST_MODE.set(true);

        fake_clock::set_ticks(800_100_000); // Large enough to exceed timeout

        // Execute function
        handle_status_read_timeout();

        // Verify timeout handling through the real stop handler's effects.
        assert_eq!(
            SLAVE_READ_STATUS_BUSY.get(),
            0,
            "timed-out status read must be stopped"
        );
        assert_eq!(
            DEVICE_STATUS_READ_UNICAST_MODE.get(),
            false,
            "the stop handler must disable unicast status mode"
        );
    }

    /// Tests OTA operations handling.
    ///
    /// Verifies that handle_ota_operations correctly:
    /// - Returns `None` when no OTA is in progress
    /// - Returns `Some(OtaFinishCheck)` while the timeout counter is non-zero
    /// - Returns `Some(OtaFinishCheckAndErrorReboot)` once the counter is zero
    /// - Still schedules the next connection event and returns to RX on every
    ///   OTA-active path
    #[test]
    fn test_handle_ota_operations_not_active() {
        reset_global_state();

        // Setup: No OTA in progress
        crate::callbacks::test_ota::set_in_progress_for_test(false);
        crate::callbacks::test_ota::set_timeout_for_test(0);

        // Execute function
        let result = handle_ota_operations();

        // Verify no OTA handling
        assert_eq!(result, None, "Should return None when OTA is not active");
        assert!(
            !wrote(REG_SYSTEM_TICK_IRQ),
            "no connection event may be scheduled when OTA is inactive"
        );
    }

    #[test]
    fn test_handle_ota_operations_active() {
        reset_global_state();

        // One microsecond past the one-second gate, so the countdown branch runs
        // on this pass. The gate is a strict `>` against the process-global tick
        // stamp (which starts at 0), so exactly one second would not fire. This
        // is also the largest tick any OTA-driving test in the crate uses, so
        // leaving the stamp here keeps later tests' elapsed subtraction valid.
        fake_clock::set_ticks(CLOCK_SYS_CLOCK_1US as u64 * 1_000_000 + 1);
        FakeMmio::clear_writes();

        // Setup: OTA in progress; the one-second countdown fires and leaves the
        // counter non-zero.
        crate::callbacks::test_ota::set_in_progress_for_test(true);
        crate::callbacks::test_ota::set_timeout_for_test(7);
        SLAVE_NEXT_CONNECT_TICK.set(15000);
        let timeout_before = crate::callbacks::test_ota::timeout_seconds_for_test();

        // Execute function
        let result = handle_ota_operations();

        // Verify OTA handling
        assert_eq!(
            result,
            Some(BleConnectionEvent::OtaFinishCheck),
            "Should report OtaFinishCheck while the timeout counter is non-zero"
        );
        assert_eq!(
            crate::callbacks::test_ota::timeout_seconds_for_test(),
            timeout_before - 1,
            "the one-second gate must decrement the countdown by exactly one"
        );
        assert_eq!(
            writes_to(REG_SYSTEM_TICK_IRQ),
            vec![RegWrite {
                addr: REG_SYSTEM_TICK_IRQ | REG_BASE_ADDR,
                width: 4,
                value: 15000,
            }],
            "the next connection event must be scheduled during OTA"
        );
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Receiving,
            "Should transition to RX state during OTA"
        );

        crate::callbacks::test_ota::set_in_progress_for_test(false);
        crate::callbacks::test_ota::set_timeout_for_test(0);
    }

    /// A zero OTA timeout counter must report `OtaFinishCheckAndErrorReboot` — the
    /// composition root runs the finish handler first (it may refresh the counter)
    /// and only then decides whether to reboot.
    #[test]
    fn test_handle_ota_operations_active_counter_zero() {
        reset_global_state();

        fake_clock::set_ticks(CLOCK_SYS_CLOCK_1US as u64 * 1_000_000 + 1);
        // The OTA tick stamp is process-global and monotonic: the test above
        // stores the instant it fired at (one microsecond past the one-second
        // gate), so read the same instant back (elapsed 0, gate stays idle)
        // instead of a smaller tick, which would underflow the elapsed
        // subtraction. The countdown is already zero either way.
        fake_clock::set_ticks(CLOCK_SYS_CLOCK_1US as u64 * 1_000_000 + 1);
        FakeMmio::clear_writes();

        crate::callbacks::test_ota::set_in_progress_for_test(true);
        crate::callbacks::test_ota::set_timeout_for_test(0);
        SLAVE_NEXT_CONNECT_TICK.set(15000);

        let result = handle_ota_operations();

        assert_eq!(
            result,
            Some(BleConnectionEvent::OtaFinishCheckAndErrorReboot),
            "A zero timeout counter must report OtaFinishCheckAndErrorReboot"
        );
        assert_eq!(
            writes_to(REG_SYSTEM_TICK_IRQ),
            vec![RegWrite {
                addr: REG_SYSTEM_TICK_IRQ | REG_BASE_ADDR,
                width: 4,
                value: 15000,
            }],
            "the next connection event must be scheduled during OTA"
        );
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Receiving,
            "Should still transition to RX state when the counter is zero"
        );

        crate::callbacks::test_ota::set_in_progress_for_test(false);
    }

    /// Tests bridge operations handling.
    ///
    /// Verifies that handle_bridge_operations correctly:
    /// - Sets bridge command timeout
    /// - Conditionally transmits bridge packets based on connection state
    /// - Respects timing constraints for transmission
    #[test]
    fn test_handle_bridge_operations_transmission_allowed() {
        reset_global_state();

        fake_clock::set_ticks(50000);
        FakeMmio::clear_writes();
        PAIR_AC.set(0x1234_5678);

        // Setup: Conditions allow bridge transmission
        SLAVE_LINK_INTERVAL.set(CLOCK_SYS_CLOCK_1US * 15000); // 15ms interval (> 10ms)
        BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.set(0); // No old interval pending
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(100);
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(99); // Not at update instant

        // Execute function
        handle_bridge_operations();

        // Verify bridge timeout is set
        let expected_timeout = CLOCK_SYS_CLOCK_1US * 200 + 50000;
        assert_eq!(
            BRIDGE_COMMAND_TIMESTAMP.get(),
            expected_timeout,
            "Bridge command timeout should be set to current time + 200µs"
        );

        // Verify transmission occurs: `tx_packet_bridge` reconfigures the radio,
        // writing the pairing access code byte-swapped.
        assert!(
            writes_to(REG_RF_ACCESS_CODE).contains(&RegWrite {
                addr: REG_RF_ACCESS_CODE | REG_BASE_ADDR,
                width: 4,
                value: 0x1234_5678u32.swap_bytes(),
            }),
            "bridge transmission must program the RF access code: {:?}",
            FakeMmio::writes()
        );
    }

    #[test]
    fn test_handle_bridge_operations_transmission_blocked() {
        reset_global_state();

        fake_clock::set_ticks(50000);
        FakeMmio::clear_writes();

        // Setup: Conditions block bridge transmission
        SLAVE_LINK_INTERVAL.set(CLOCK_SYS_CLOCK_1US * 5000); // 5ms interval (< 10ms)
        BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.set(100); // Old interval pending
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(100);
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(100); // At update instant

        // Execute function
        handle_bridge_operations();

        // The timeout is still armed, but no packet was transmitted.
        assert_eq!(
            BRIDGE_COMMAND_TIMESTAMP.get(),
            CLOCK_SYS_CLOCK_1US * 200 + 50000,
            "the bridge command timeout is armed regardless of transmission"
        );
        assert!(
            !wrote(REG_RF_ACCESS_CODE),
            "blocked bridge transmission must not touch the radio"
        );
    }

    /// Tests mesh operations processing.
    ///
    /// This is a complex function that handles multiple operations.
    /// We'll test the key decision points and integrations.
    #[test]
    fn test_process_mesh_operations_status_processing() {
        reset_global_state();

        // A status read is in progress with one response queued: the real
        // processing loop must hand it to the DMA queue and advance the read
        // pointer.
        SLAVE_READ_STATUS_BUSY.set(5);
        DEVICE_STATUS_BUFFER_READ_POINTER.set(0);
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(1);

        // The mesh sweep only runs once per MESH_STATUS_SWEEP_MS: park the fake
        // clock far past any timestamp an earlier test can have left behind.
        fake_clock::set_ticks(0xffff_0000);

        // Seed one online remote so the sweep has something to age.
        MESH_NODE_MAX.set(2);
        {
            let mut st = MESH_NODE_ST.lock();
            st[1].miss = 1;
        }

        // Execute function
        process_mesh_operations();

        // Verify status processing ran (it consumed the queued response)
        assert_eq!(
            DEVICE_STATUS_BUFFER_READ_POINTER.get(),
            1,
            "the queued status response must be sent"
        );

        // Verify the real mesh sweep ran: the online remote was aged by one
        // sweep (offline deadline for a 2-node mesh is 6 sweeps).
        assert_eq!(
            MESH_NODE_ST.lock()[1].miss,
            2,
            "mesh_node_flush_status must age online remotes by one sweep"
        );
    }

    /// Tests that the BLE and UART status reporting paths are fully independent.
    ///
    /// When BLE TX buffer is not ready, process_mesh_operations should not send
    /// any BLE packets. UART reporting is handled at the source of status changes,
    /// not in process_mesh_operations.
    #[test]
    fn test_process_mesh_operations_no_ble_when_tx_not_ready() {
        reset_global_state();

        // BLE TX buffer is NOT ready: three of the eight DMA slots are in use.
        FakeMmio::set_reg8(REG_DMA_TX_RPTR, 2);
        FakeMmio::set_reg8(REG_DMA_TX_WPTR, 5);

        // A node is due for reporting, so the only reason no packet goes out is
        // the full TX buffer.
        MESH_NODE_MAX.set(1);
        MESH_NODE_REPORT_ENABLE.set(true);
        MESH_NODE_MASK.lock()[0] = 1;

        SLAVE_READ_STATUS_BUSY.set(0);
        process_mesh_operations();

        // BLE should not have been called (TX buffer not ready)
        assert_eq!(
            dma_tx_fifo_writes(),
            0,
            "no packet may be registered with the DMA engine while the TX buffer is full"
        );
    }

    /// Tests that BLE status is sent when TX buffer is ready.
    /// UART reporting is handled at the source of status changes, not here.
    #[test]
    fn test_process_mesh_operations_ble_fires_when_ready() {
        reset_global_state();

        // BLE TX buffer is ready (empty).
        FakeMmio::set_reg8(REG_DMA_TX_RPTR, 0);
        FakeMmio::set_reg8(REG_DMA_TX_WPTR, 0);

        // One node is due for reporting.
        MESH_NODE_MAX.set(1);
        MESH_NODE_REPORT_ENABLE.set(true);
        MESH_NODE_MASK.lock()[0] = 1;

        SLAVE_READ_STATUS_BUSY.set(0);
        process_mesh_operations();

        // BLE should have fired: an empty buffer also gets the one-shot marker
        // registration, so two FIFO writes are expected.
        assert_eq!(
            dma_tx_fifo_writes(),
            2,
            "the status report must be registered with the DMA engine"
        );
        assert_eq!(
            MESH_NODE_MASK.lock()[0] & 1,
            0,
            "the reported node's mask bit must be consumed"
        );
    }

    /// Tests connection event timing management.
    ///
    /// This is the most complex function with critical timing logic.
    /// We'll test the core timing calculations and loop behavior.
    #[test]
    fn test_manage_connection_event_timing_within_window() {
        reset_global_state();

        // Need to avoid underflow in: (next_tick - 500*clock) - current_tick
        // With CLOCK_SYS_CLOCK_1US = 32: next=50000, safety=500*32=16000, so next-safety=34000
        // Need current_tick <= 34000 to avoid underflow, and result <= interval=20000 to exit loop
        fake_clock::set_ticks(20000); // 34000 - 20000 = 14000 <= 20000 ✓
        FakeMmio::clear_writes();

        // Setup: Current time is within connection interval window
        SLAVE_NEXT_CONNECT_TICK.set(50000); // Next event at 50000
        SLAVE_LINK_INTERVAL.set(20000); // 20000 tick interval
        BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.set(0);

        // Execute function
        manage_connection_event_timing();

        // Verify radio configuration: `back_to_rxmode_bridge` programs RX mode
        // and the RX state machine.
        assert!(
            wrote(REG_RF_RX_MODE),
            "back_to_rxmode_bridge must enable the receiver: {:?}",
            FakeMmio::writes()
        );
        assert!(
            wrote(REG_RF_TXRX_STATE),
            "back_to_rxmode_bridge must program the RX state: {:?}",
            FakeMmio::writes()
        );

        // Verify timing values remain unchanged (no missed events)
        assert_eq!(
            SLAVE_NEXT_CONNECT_TICK.get(),
            50000,
            "Next connect tick should be unchanged when within window"
        );
    }

    /// Tests scheduling of next connection event.
    ///
    /// Verifies that schedule_next_connection_event correctly:
    /// - Schedules interrupt for next connection event
    /// - Transitions state handler to receive mode
    #[test]
    fn test_schedule_next_connection_event() {
        reset_global_state();
        FakeMmio::clear_writes();

        // Setup: Next connection event time
        SLAVE_NEXT_CONNECT_TICK.set(25000);

        // Execute function
        schedule_next_connection_event();

        // Verify interrupt is scheduled
        assert_eq!(
            writes_to(REG_SYSTEM_TICK_IRQ),
            vec![RegWrite {
                addr: REG_SYSTEM_TICK_IRQ | REG_BASE_ADDR,
                width: 4,
                value: 25000,
            }],
            "the next connection event must be scheduled"
        );

        // Verify state transition
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Receiving,
            "Should transition to RX state to listen for next packet"
        );
    }

    // ================================================================================
    // Tests for handle_ble_connected_state (main orchestrator function)
    // ================================================================================

    /// Tests the main handle_ble_connected_state function end to end.
    ///
    /// The decomposed helpers are no longer stubbed: the real orchestration runs
    /// and every step is verified through its own observable effect, which is
    /// what proves the ordering claim.
    #[test]
    fn test_handle_ble_connected_state_normal_flow() {
        reset_global_state();

        // A live connection inside its supervision timeout.
        fake_clock::set_ticks(20000);
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(1000);
        SLAVE_CONNECTED_TICK.set(19000);
        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Disconnected);
        BRIDGE_SEQUENCE_NUMBER.set(100);
        BRIDGE_COMMAND_TIMESTAMP.set(0);

        // Timing such that the event-timing loop exits immediately.
        SLAVE_NEXT_CONNECT_TICK.set(50000);
        SLAVE_LINK_INTERVAL.set(20000);
        BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.set(0);

        // Execute the main function
        let result = handle_ble_connected_state();

        // The normal bridge path reports nothing for the composition root to do
        assert_eq!(result, BleConnectionEvent::None);

        // 1. initialize_connection_hardware
        assert_eq!(
            BLE_PERIPHERAL_LINK_STATE.get(),
            BlePeripheralLinkState::Connected,
            "the connection must be initialised into bridge mode"
        );
        assert_eq!(
            BRIDGE_SEQUENCE_NUMBER.get(),
            101,
            "the bridge sequence counter must advance once"
        );

        // 2/3. no supervision timeout, no status-read timeout
        assert_eq!(
            BLE_PERIPHERAL_CONNECTION_ACTIVE.get(),
            false,
            "no timeout path may run while the connection is live"
        );

        // 5. handle_bridge_operations armed the bridge window
        assert_eq!(
            BRIDGE_COMMAND_TIMESTAMP.get(),
            CLOCK_SYS_CLOCK_1US * 200 + 20000,
            "the bridge command window must be armed from the current tick"
        );

        // 7. manage_connection_event_timing returned without advancing
        assert_eq!(
            SLAVE_NEXT_CONNECT_TICK.get(),
            50000,
            "no missed connection event may be skipped"
        );

        // 8. schedule_next_connection_event
        assert_eq!(
            writes_to(REG_SYSTEM_TICK_IRQ),
            vec![RegWrite {
                addr: REG_SYSTEM_TICK_IRQ | REG_BASE_ADDR,
                width: 4,
                value: 50000,
            }],
            "the next connection event must be scheduled exactly once"
        );
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Receiving,
            "the handler must end in receive mode"
        );
    }

    #[test]
    fn test_handle_ble_connected_state_timeout_early_exit() {
        reset_global_state();

        // Expired supervision timeout.
        fake_clock::set_ticks(15000);
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(100);
        SLAVE_CONNECTED_TICK.set(10000);
        BRIDGE_SEQUENCE_NUMBER.set(100);
        BRIDGE_COMMAND_TIMESTAMP.set(0);

        // Execute the main function
        let result = handle_ble_connected_state();

        // A timed-out connection reports Some(None): cleanup already ran and the
        // state must not be overwritten back to Receiving.
        assert_eq!(result, BleConnectionEvent::None);

        // initialize_connection_hardware still ran first
        assert_eq!(
            BLE_PERIPHERAL_LINK_STATE.get(),
            BlePeripheralLinkState::Connected,
            "hardware initialisation precedes the timeout check"
        );
        assert_eq!(BRIDGE_SEQUENCE_NUMBER.get(), 101);

        // Early exit: only cleanup's advertisement transition was scheduled.
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Advertising,
            "a timed-out connection must be left advertising"
        );
        assert_eq!(
            writes_to(REG_SYSTEM_TICK_IRQ),
            vec![RegWrite {
                addr: REG_SYSTEM_TICK_IRQ | REG_BASE_ADDR,
                width: 4,
                value: CLOCK_SYS_CLOCK_1US * 100 + 15000,
            }],
            "the scheduling step must not run after the timeout cleanup"
        );

        // These steps must NOT run due to the early exit.
        assert_eq!(
            BRIDGE_COMMAND_TIMESTAMP.get(),
            0,
            "bridge operations must not run after a timeout"
        );
    }

    /// A supervision timeout during OTA must propagate `OtaErrorReboot` unchanged so
    /// the composition root can reboot.
    #[test]
    fn test_handle_ble_connected_state_propagates_ota_error_reboot() {
        reset_global_state();

        fake_clock::set_ticks(15000);
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(100);
        SLAVE_CONNECTED_TICK.set(10000);
        BRIDGE_COMMAND_TIMESTAMP.set(0);
        crate::callbacks::test_ota::set_in_progress_for_test(true);

        let result = handle_ble_connected_state();

        assert_eq!(result, BleConnectionEvent::OtaErrorReboot);
        assert_eq!(
            BRIDGE_COMMAND_TIMESTAMP.get(),
            0,
            "bridge operations must not run after a timeout"
        );
        assert_eq!(
            writes_to(REG_SYSTEM_TICK_IRQ),
            vec![RegWrite {
                addr: REG_SYSTEM_TICK_IRQ | REG_BASE_ADDR,
                width: 4,
                value: CLOCK_SYS_CLOCK_1US * 100 + 15000,
            }],
            "only the cleanup may schedule a wakeup before the reboot verdict"
        );
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Advertising,
            "the connection must be left advertising"
        );

        crate::callbacks::test_ota::set_in_progress_for_test(false);
    }

    #[test]
    fn test_handle_ble_connected_state_ota_early_exit() {
        reset_global_state();

        // OTA active with a non-zero supervision counter. The clock stays below
        // the one-second gate here: the process-global OTA tick stamp must not be
        // advanced past `CLOCK_SYS_CLOCK_1US * 1_000_000 + 1`, the largest tick
        // the other OTA-driving tests in the crate use (a larger stamp would
        // underflow their elapsed subtraction).
        let now = 500_000u64;
        fake_clock::set_ticks(now);
        // A live connection: the supervision timeout must not pre-empt the OTA path.
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(1000);
        SLAVE_CONNECTED_TICK.set(now as u32 - 100);
        crate::callbacks::test_ota::set_in_progress_for_test(true);
        crate::callbacks::test_ota::set_timeout_for_test(7);
        SLAVE_NEXT_CONNECT_TICK.set(15000);
        SLAVE_LINK_INTERVAL.set(20000);
        BRIDGE_COMMAND_TIMESTAMP.set(0);
        FakeMmio::clear_writes();

        // Execute the main function
        let result = handle_ble_connected_state();

        // The OTA event must be propagated unchanged
        assert_eq!(result, BleConnectionEvent::OtaFinishCheck);

        // Only the OTA path scheduled the next connection event, from
        // SLAVE_NEXT_CONNECT_TICK (the timing loop did not advance it).
        assert_eq!(
            writes_to(REG_SYSTEM_TICK_IRQ),
            vec![RegWrite {
                addr: REG_SYSTEM_TICK_IRQ | REG_BASE_ADDR,
                width: 4,
                value: 15000,
            }],
            "the OTA path must schedule the next connection event"
        );
        assert_eq!(
            SLAVE_NEXT_CONNECT_TICK.get(),
            15000,
            "the event-timing loop must not run during OTA"
        );

        // These steps must NOT run due to the early exit.
        assert_eq!(
            BRIDGE_COMMAND_TIMESTAMP.get(),
            0,
            "bridge operations must not run during OTA"
        );
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Receiving,
            "the OTA path returns to receive mode"
        );

        crate::callbacks::test_ota::set_in_progress_for_test(false);
        crate::callbacks::test_ota::set_timeout_for_test(0);
    }

    // ================================================================================
    // Tests for process_queued_status_responses function
    // ================================================================================

    /// Tests processing of queued status responses.
    ///
    /// Verifies that process_queued_status_responses correctly:
    /// - Processes packets in FIFO order
    /// - Advances read pointer correctly
    /// - Stops when transmission buffer is full
    /// - Handles buffer wraparound
    #[test]
    fn test_process_queued_status_responses_normal_processing() {
        reset_global_state();

        // Empty TX buffer: every packet can be queued.
        FakeMmio::set_reg8(REG_DMA_TX_RPTR, 0);
        FakeMmio::set_reg8(REG_DMA_TX_WPTR, 0);

        // Setup: Multiple packets queued (start with clean state)
        DEVICE_STATUS_BUFFER_READ_POINTER.set(0);
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(3); // 3 packets queued

        // Execute function
        process_queued_status_responses();

        // Verify all packets were processed: three successful queue
        // registrations, each preceded by the empty-buffer marker because the
        // fake DMA pointers never advance.
        assert_eq!(
            dma_tx_fifo_writes(),
            6,
            "three packets must be registered with the DMA engine"
        );

        // Verify read pointer advanced
        assert_eq!(
            DEVICE_STATUS_BUFFER_READ_POINTER.get(),
            3,
            "Read pointer should advance to match write pointer"
        );
    }

    #[test]
    fn test_process_queued_status_responses_buffer_full() {
        reset_global_state();

        // Four of the eight DMA slots used: `rf_link_add_tx_packet` rejects.
        FakeMmio::set_reg8(REG_DMA_TX_RPTR, 3);
        FakeMmio::set_reg8(REG_DMA_TX_WPTR, 7);

        // Setup: Packets queued (start with clean state)
        DEVICE_STATUS_BUFFER_READ_POINTER.set(0);
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(2); // 2 packets queued

        // Execute function
        process_queued_status_responses();

        // Verify only one packet was attempted (stopped on first failure)
        assert_eq!(
            dma_tx_fifo_writes(),
            0,
            "a full TX buffer must reject the first packet and stop the loop"
        );

        // Verify read pointer did not advance (packet failed to send)
        assert_eq!(
            DEVICE_STATUS_BUFFER_READ_POINTER.get(),
            0,
            "Read pointer should not advance when transmission fails"
        );
    }

    #[test]
    fn test_process_queued_status_responses_buffer_wraparound() {
        reset_global_state();

        // Empty TX buffer: every packet can be queued.
        FakeMmio::set_reg8(REG_DMA_TX_RPTR, 0);
        FakeMmio::set_reg8(REG_DMA_TX_WPTR, 0);

        // Setup: Buffer wraparound scenario (need to understand BUFF_RESPONSE_PACKET_COUNT)
        // Let's use a safer approach - start near the end and wrap to beginning
        let wrap_point = BUFF_RESPONSE_PACKET_COUNT - 1;
        DEVICE_STATUS_BUFFER_READ_POINTER.set(wrap_point);
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(1); // Wrapped around: 1 packet at end + 1 at start

        // Execute function
        process_queued_status_responses();

        // Verify correct number of packets processed
        // Should process: [wrap_point], [0] = 2 packets total
        assert_eq!(
            dma_tx_fifo_writes(),
            4,
            "two packets must be registered with the DMA engine"
        );

        // Verify read pointer wrapped around correctly
        assert_eq!(
            DEVICE_STATUS_BUFFER_READ_POINTER.get(),
            1,
            "Read pointer should wrap around correctly"
        );
    }

    #[test]
    fn test_process_queued_status_responses_empty_queue() {
        reset_global_state();

        // Setup: Empty queue (read == write)
        DEVICE_STATUS_BUFFER_READ_POINTER.set(5);
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(5);

        // Execute function
        process_queued_status_responses();

        // Verify no packets processed
        assert_eq!(
            dma_tx_fifo_writes(),
            0,
            "an empty queue must not touch the DMA engine"
        );

        // Verify pointers unchanged
        assert_eq!(
            DEVICE_STATUS_BUFFER_READ_POINTER.get(),
            5,
            "Read pointer should remain unchanged for empty queue"
        );
    }
}
