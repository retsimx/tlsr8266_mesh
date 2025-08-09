//! # BLE Link Layer IRQ Handler Module
//!
//! This module implements the Bluetooth Low Energy (BLE) link layer interrupt handling
//! for a TLSR8266-based mesh lighting system. It manages BLE connection states, timing
//! synchronization, mesh packet processing, and over-the-air (OTA) updates.
//!
//! ## Key Responsibilities:
//! - BLE connection parameter updates and timing synchronization
//! - Mesh network packet reception and transmission
//! - Advertisement and scan response handling
//! - Connection state management and timeout handling
//! - OTA update coordination
//! - RF channel hopping and access code management
//!
//! ## BLE Specification Compliance:
//! This implementation follows BLE 4.0+ specifications for:
//! - Connection parameter update procedures (LL_CONNECTION_UPDATE_IND)
//! - Channel map update procedures (LL_CHANNEL_MAP_IND)
//! - Connection event timing and supervision timeouts
//! - Frequency hopping sequence management

use core::ptr::{addr_of};
use core::slice;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::{app};
use crate::common::{pair_load_key, SYS_CHN_ADV, SYS_CHN_LISTEN};
use crate::config::VENDOR_ID;
use crate::embassy::time_driver::clock_time64;
use crate::mesh::MESH_NODE_ST_VAL_LEN;
use crate::sdk::ble_app::ble_ll_channel_selection::{ble_ll_build_available_channel_table, ble_ll_select_next_data_channel};
use crate::sdk::ble_app::ble_ll_pair::{pair_dec_packet_mesh, pair_proc};
use crate::sdk::ble_app::light_ll::{*};
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::light::{*};
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, sleep_us};
use crate::sdk::mcu::register::{*};
use crate::sdk::packet_types::{Packet, PacketAttCmd, PacketAttValue, PacketL2capHead, PacketScanRsp, ScanRspData};
use crate::state::{*};

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
fn handle_ble_connection_parameter_updates()
{
    // Increment the connection event counter - this tracks which connection event we're in
    // This is essential for BLE timing synchronization between master and slave
    BLE_PERIPHERAL_CONNECTION_INSTANT.inc();
    
    // Handle Channel Map Update (LL_CHANNEL_MAP_IND procedure)
    if BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get() == 1 && BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.get() == BLE_PERIPHERAL_CONNECTION_INSTANT.get() {
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
        if BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get() == 2 && BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.get() == BLE_PERIPHERAL_CONNECTION_INSTANT.get() {
            // Clear the update flag since we're processing it now
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(0);
            
            // Calculate the exact time when the new parameters take effect
            // This includes the connection offset to maintain precise timing
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.set(BLE_CONN_OFFSET.get() + SLAVE_NEXT_CONNECT_TICK.get());
            
            // Apply the new connection parameters that were received from the master:
            
            // Update connection interval (time between connection events)
            SLAVE_LINK_INTERVAL.set(BLE_CONN_INTERVAL.get());
            
            // Update supervision timeout (max time before connection is considered lost)
            BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(BLE_CONN_TIMEOUT.get());
            
            // Set flag indicating timing update is in progress
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.set(true);
            
            // Calculate new window size: connection interval minus a fixed offset (0x4e2 * 1µs)
            // Window size determines how long the slave listens for packets in each connection event
            SLAVE_WINDOW_SIZE.set(BLE_CONN_INTERVAL.get().saturating_sub(CLOCK_SYS_CLOCK_1US * 0x4e2));
            
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

/// Configures the RF transceiver to listen for mesh network packets.
///
/// This function sets up the radio to receive packets on the mesh listening channels.
/// It configures the BLE access code, CRC, and channel for mesh communication.
/// The device will listen on one of the predefined mesh channels in a round-robin fashion.
///
/// ## Mesh Network Context:
/// - Uses special mesh access codes for packet filtering
/// - Cycles through predefined listening channels to discover mesh traffic
/// - Sets the device into a listening state (BLE_PERIPHERAL_LINK_STATE = 4)
///
/// ## RF Configuration:
/// - **Access Code**: Uses pairing access code for mesh packet identification
/// - **CRC**: Uses advertisement CRC for mesh compatibility
/// - **Channel**: Rotates through SYS_CHN_LISTEN channels for mesh discovery
fn configure_rf_for_mesh_listening()
{
    // Stop any ongoing transmission or reception
    rf_stop_trx();

    // Set the BLE access code to the pairing access code
    // This allows the device to receive mesh pairing and discovery packets
    rf_set_ble_access_code(PAIR_AC.get());
    
    // Configure CRC for advertisement-style packets
    // Mesh packets use the same CRC format as BLE advertisements
    rf_set_ble_crc_adv();

    // Select the mesh listening channel in round-robin fashion
    // This cycles through predefined channels to discover mesh network activity
    rf_set_ble_channel(SYS_CHN_LISTEN[MESH_LISTEN_CYCLE_COUNT.get() as usize % SYS_CHN_LISTEN.len()]);
    
    // Enable receive mode to start listening for packets
    rf_set_rxmode();

    // Set the link state to indicate we're in mesh listening mode
    // State 4 = mesh listening state
    BLE_PERIPHERAL_LINK_STATE.set(4);
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
fn process_queued_status_responses()
{
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
        if !rf_link_add_tx_packet(&packet) {
            return;
        }

        // Advance the read pointer to the next packet
        // Use atomic update to prevent race conditions
        rptr = ((DEVICE_STATUS_BUFFER_READ_POINTER.get() + 1) % BUFF_RESPONSE_PACKET_COUNT);
        DEVICE_STATUS_BUFFER_READ_POINTER.set(rptr);
    }
}

/// Reports mesh node status information for transmission to the BLE master.
///
/// This function collects status information from mesh nodes that have pending updates
/// and formats them for transmission. It uses a bitmask to track which nodes have
/// status changes that need to be reported.
///
/// ## Parameters:
/// - `params`: Output buffer to store the formatted status data
/// - `len`: Maximum number of node statuses to include in this report
///
/// ## Returns:
/// Number of node statuses actually written to the params buffer
///
/// ## Mesh Network Context:
/// - Each mesh node has a status structure containing state information
/// - A bitmask tracks which nodes have pending status updates
/// - Status reports are sent in chunks to avoid packet size limits
/// - Offline nodes (tick == 0) have their sequence numbers cleared
///
/// ## Buffer Management:
/// - Uses atomic operations to prevent race conditions
/// - Clears processed status bits to prevent duplicate reports
/// - Handles partial reports when buffer space is limited
pub fn mesh_node_report_status(params: &mut [u8], len: usize) -> usize
{
    // Check if mesh node status reporting is enabled
    if !MESH_NODE_REPORT_ENABLE.get() {
        return 0;
    }

    let mut result = 0;

    // Clear the output buffer to ensure clean data
    params[0..MESH_NODE_ST_VAL_LEN * len].fill(0);

    // The algorithm:
    // 1. Iterate over each 32-bit value in the mask
    // 2. For each 32-bit value, check if it's 0 (no pending reports)
    // 3. Iterate over each bit in the 32-bit value and find any set bits
    // 4. Report the status of nodes at any set bits and clear the bit from the mask

    // Lock the mesh node data structures for atomic access
    let mut mesh_node_mask = MESH_NODE_MASK.lock();
    let mut mesh_node_st = MESH_NODE_ST.lock();

    // Process each mesh node to check for pending status reports
    mesh_node_st.iter().enumerate().for_each(|(idx, val)| {
        // Stop if we've filled the output buffer
        if result >= len {
            return;
        }

        // Calculate which 32-bit mask word and which bit within that word
        let mask_index = idx / 32;
        let mask_bit = idx % 32;
        let mask = mesh_node_mask[mask_index];

        // Check if this node has a pending status update
        if mask & (1 << mask_bit) != 0 {
            // Clear the bit from the mask so it isn't reported again
            // This prevents duplicate status reports
            mesh_node_mask[mask_index] = mask & !(1 << mask_bit);

            // Copy the node status value to the output parameters
            let params_idx = MESH_NODE_ST_VAL_LEN * result;
            params[params_idx..params_idx + MESH_NODE_ST_VAL_LEN].copy_from_slice(
                unsafe {
                    slice::from_raw_parts(
                        addr_of!(mesh_node_st[idx].val) as *const u8,
                        MESH_NODE_ST_VAL_LEN,
                    )
                }
            );

            // Special handling for offline devices:
            // If the tick is 0 (device offline), set the sequence number to 0
            // This indicates to the master that the device is not responding
            if mesh_node_st[idx].tick == 0 {
                params[params_idx + 1] = 0;
            }

            // Increment the result count
            // If we've exhausted the params buffer size, we'll stop processing
            // The next call to this function will send the next chunk of statuses
            result += 1;
        }
    });

    return result;
}

/// Handles the mesh listening state interrupt.
///
/// This function is called when the device is in listening mode, waiting to discover
/// mesh network activity or receive connection requests. It manages the transition
/// between listening, advertising, and online status reporting.
///
/// ## State Machine:
/// - **Listen**: Device scans for mesh traffic on predefined channels
/// - **Advertise**: Device broadcasts its availability for connections
/// - **Online Status**: Device reports its status to the mesh network
///
/// ## Timing Strategy:
/// - Uses intervals to balance power consumption and network responsiveness
/// - Randomizes advertisement timing to reduce collisions
/// - Prioritizes advertising when the flag is set
///
/// ## Mesh Discovery:
/// - Processes bridge commands that may have been received
/// - Rotates through listening channels for comprehensive coverage
/// - Handles periodic online status broadcasts
pub fn handle_mesh_listening_state()
{
    // Set the link state to listening mode (state 4)
    BLE_PERIPHERAL_LINK_STATE.set(4);

    // Stop any ongoing radio operations
    rf_stop_trx();

    // Process any pending bridge commands that may have been received
    // This handles mesh network control messages
    app_bridge_cmd_handle(read_reg_system_tick());

    // Configure the radio to listen for mesh packets
            configure_rf_for_mesh_listening();

    // Schedule the next listening interval
    write_reg_system_tick_irq(read_reg_system_tick() + MESH_LISTEN_INTERVAL_US.get());
    
    // If advertising is requested, switch to advertisement mode immediately
    if BLE_ADVERTISING_ENABLED.get() {
        // Set a shorter interval for advertisement timing (7ms)
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 7000 + read_reg_system_tick());
        *P_ST_HANDLER.lock() = IrqHandlerStatus::Adv;
        return;
    }
    
    // Handle periodic advertising and online status reporting
    if !MESH_DEVICE_ONLINE_STATUS.get() {
        // Increment the listen counter for interval calculations
        MESH_LISTEN_CYCLE_COUNT.inc();
        
        // Determine if we should advertise based on the listen interval ratio
        // This creates a pattern where advertising happens periodically during listening
        BLE_ADVERTISING_ENABLED.set(MESH_LISTEN_CYCLE_COUNT.get() % ADV_INTERVAL2LISTEN_INTERVAL as u32 != 0);
        
        // Determine if we should send online status based on the interval ratio
        // This ensures the device periodically reports its presence to the mesh
        MESH_DEVICE_ONLINE_STATUS.set(MESH_LISTEN_CYCLE_COUNT.get() % ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL as u32 != 0);
        
        // If advertising is now scheduled, add randomization to prevent collisions
        if BLE_ADVERTISING_ENABLED.get() {
            // Use XOR with random number to create pseudo-random timing offset
            // The 0x7fff mask limits the randomization to prevent excessive delays
            write_reg_system_tick_irq((((read_reg_system_tick() ^ read_reg_rnd_number() as u32) & 0x7fff) * CLOCK_SYS_CLOCK_1US + read_reg_system_tick_irq()));
            *P_ST_HANDLER.lock() = IrqHandlerStatus::Adv;
            return;
        }
        
        // If no online status is scheduled, continue listening
        if !MESH_DEVICE_ONLINE_STATUS.get() {
            return;
        }
    }

    // Default transition to advertisement state for online status or regular advertising
    *P_ST_HANDLER.lock() = IrqHandlerStatus::Adv;
}

/// Returns the number of GATT advertisement cycles to perform.
///
/// This function defines how many advertisement packets should be sent
/// during each advertisement sequence. The value 3 corresponds to advertising
/// on all three primary BLE advertisement channels (37, 38, 39).
///
/// ## BLE Advertisement Channels:
/// - Channel 37: 2402 MHz
/// - Channel 38: 2426 MHz  
/// - Channel 39: 2480 MHz
///
/// ## Returns:
/// Always returns 3 to ensure complete coverage of all advertisement channels
fn get_ble_advertisement_channel_count() -> u32
{
    return 3;
}

/// Handles the BLE advertisement state interrupt.
///
/// This function manages the BLE advertisement process, cycling through all three
/// advertisement channels and handling online status reporting. It implements
/// the standard BLE advertisement procedure with mesh-specific extensions.
///
/// ## Advertisement Sequence:
/// 1. Advertise on channel 37 (2402 MHz)
/// 2. Advertise on channel 38 (2426 MHz)  
/// 3. Advertise on channel 39 (2480 MHz)
/// 4. Send mesh online status (if scheduled)
/// 5. Return to listening mode
///
/// ## State Management:
/// - Uses static counter to track current advertisement channel
/// - Manages transition between advertisement and listening states
/// - Handles mesh online status reporting
///
/// ## Timing Control:
/// - Uses precise timing for advertisement intervals (1200µs = 0x4b0)
/// - Short delays for status reporting (100µs) vs listening (500µs)
/// - Enables STX2RX mode for potential scan response reception
pub fn handle_ble_advertisement_state()
{
    // Static counter to track which advertisement channel we're currently using
    static ST_PNO: AtomicU32 = AtomicU32::new(0);

    // Clear RF interrupt status and stop any ongoing transmission/reception
    write_reg8(0x80050f, 0);
    rf_stop_trx();

    // Set link state to advertisement mode (state 1)
    BLE_PERIPHERAL_LINK_STATE.set(1);

    // Check if advertisement sequence is complete or not requested
    if !BLE_ADVERTISING_ENABLED.get() || get_ble_advertisement_channel_count() <= ST_PNO.load(Ordering::Relaxed) {
        // Advertisement sequence is complete, clean up and transition
        
        // Clear advertisement flag and reset channel counter
        BLE_ADVERTISING_ENABLED.set(false);
        ST_PNO.store(0, Ordering::Relaxed);
        
        // Handle online status reporting if scheduled
        if MESH_DEVICE_ONLINE_STATUS.get() {
            // Send mesh online status packet to inform network of our presence
            mesh_send_online_status();
            // Short delay before returning to listening (100µs)
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
        } else {
            // No online status to send, use longer delay before listening (500µs)
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 500 + read_reg_system_tick());
        }
        
        // Clear online status flag since we've handled it
        MESH_DEVICE_ONLINE_STATUS.set(false);
        
        // Transition back to listening state
        *P_ST_HANDLER.lock() = IrqHandlerStatus::Listen;
        
        // Clear RF interrupt status
        write_reg_rf_irq_status(1);
    } else {
        // Continue advertisement sequence on next channel
        
        // Configure RF for BLE advertisement
        rf_set_ble_access_code_adv();  // Standard BLE advertisement access code
        rf_set_ble_crc_adv();          // Standard BLE advertisement CRC
        
        // Set the advertisement channel (cycling through 37, 38, 39)
        rf_set_ble_channel(SYS_CHN_ADV[ST_PNO.load(Ordering::Relaxed) as usize % 3]);
        
        // Schedule next interrupt for advertisement interval (1200µs)
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 0x4b0 + read_reg_system_tick());
        
        // Clear RF interrupt status
        write_reg_rf_irq_status(1);

        // Start transmission with scan response capability (if not on last channel)
        if ST_PNO.load(Ordering::Relaxed) < 3 {
            // Get the advertisement packet data
            let tmp = *PKT_ADV.lock();
            
            // Start STX2RX mode: transmit advertisement then switch to RX for scan responses
            // This allows the device to receive and respond to scan requests
            rf_start_stx2rx(addr_of!(tmp) as u32, CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick());
        }

        // Increment channel counter for next advertisement
        ST_PNO.store(ST_PNO.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
        
        // Check if we've completed all advertisement channels
        if get_ble_advertisement_channel_count() <= ST_PNO.load(Ordering::Relaxed) {
            // Reset counter and clear advertisement flag for next sequence
            ST_PNO.store(0, Ordering::Relaxed);
            BLE_ADVERTISING_ENABLED.set(false);
        }

        // Set next state to listening (will be processed after advertisement completes)
        *P_ST_HANDLER.lock() = IrqHandlerStatus::Listen;
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
fn cleanup_ble_disconnection() {
    // Schedule transition to advertisement state with short delay (100µs)
    write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
    *P_ST_HANDLER.lock() = IrqHandlerStatus::Adv;
    
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
        BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(0);  // Clear first connection timestamp
        PAIR_LOGIN_OK.set(false);           // Ensure pairing state is cleared
        mesh_report_status_enable(false);   // Disable mesh status reporting
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
        pair_load_key();  // Load stored pairing keys if available
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
    }
    
    // Configure hardware timing registers based on system clock
    // These values adjust RF timing parameters for the specific clock frequency
    if CLOCK_SYS_CLOCK_1US == 0x10 {
        write_reg8(0xf04, 0x5e);  // 16MHz clock timing
    } else {
        write_reg8(0xf04, 0x68);  // Other clock frequencies
    }

    // Reset connection parameter update state
    GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(0);      // Clear service discovery timestamp
    NEED_UPDATE_CONNECT_PARA.set(false);   // Clear connection parameter update flag
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
/// ## BLE Specification Compliance:
/// - Implements supervision timeout according to BLE specification
/// - Maintains connection event timing for reliable communication
/// - Handles connection parameter update procedures
pub fn handle_ble_connected_state()
{
    // Static variable to track OTA timeout intervals (1 second increments)
    static RF_SLAVE_OTA_TIMEOUT_TICK: AtomicU32 = AtomicU32::new(0);

    // Set link state to bridge mode (state 5) - active BLE connection
    BLE_PERIPHERAL_LINK_STATE.set(5);

    // Increment bridge operation counter for statistics/debugging
    BRIDGE_SEQUENCE_NUMBER.inc();
    
    // Clear RF status and stop any ongoing radio operations
    write_reg8(0x50f, 0);
    rf_stop_trx();

    // Configure RF timing registers based on system clock frequency
    // These settings optimize RF performance for the specific hardware clock
    if CLOCK_SYS_CLOCK_1US == 0x10 {
        write_reg8(0xf04, 0x5e);  // 16MHz clock: optimized RF timing
    } else {
        write_reg8(0xf04, 0x68);  // Other frequencies: standard RF timing
    }

    // Check for BLE connection supervision timeout
    // Per BLE specification, if no communication occurs within the supervision timeout,
    // the connection must be considered lost and terminated
    if BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.get() * CLOCK_SYS_CLOCK_1US < read_reg_system_tick() - SLAVE_CONNECTED_TICK.get() {
        // Connection has timed out - handle cleanup
        
        // If OTA is in progress, notify OTA manager of the error
        if OTA_UPDATE_IN_PROGRESS.get() {
            app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(OtaState::Error);
        }

        // Perform full BLE disconnection cleanup
        cleanup_ble_disconnection();

        return;
    }

    // Handle status read operation timeout
    // If a status read operation has been running too long, stop it to prevent hanging
    if SLAVE_READ_STATUS_BUSY.get() != 0 && SLAVE_READ_STATUS_BUSY_TIMEOUT * CLOCK_SYS_CLOCK_1US * 1000 < read_reg_system_tick() - DEVICE_STATUS_READ_BUSY_TIMESTAMP.get() {
        rf_link_slave_read_status_stop();
    }

    // Handle active OTA (Over-The-Air) update operations
    if OTA_UPDATE_IN_PROGRESS.get() {
        // Process any pending OTA operations
        app().ota_manager.rf_link_slave_ota_finish_handle();

        // Check for OTA timeout (1 second intervals)
        // This prevents OTA operations from hanging indefinitely
        if CLOCK_SYS_CLOCK_1US * 1000000 < read_reg_system_tick() - RF_SLAVE_OTA_TIMEOUT_TICK.load(Ordering::Relaxed) {
            // Update timeout tick for next second
            RF_SLAVE_OTA_TIMEOUT_TICK.store(read_reg_system_tick(), Ordering::Relaxed);
            
            // Decrement timeout counter if active
            if OTA_UPDATE_TIMEOUT_SECONDS.get() != 0 {
                OTA_UPDATE_TIMEOUT_SECONDS.dec();
                
                // If timeout reached zero, abort OTA with error
                if OTA_UPDATE_TIMEOUT_SECONDS.get() == 0 {
                    app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(OtaState::Error);
                }
            }
        }

        // During OTA, schedule next connection event and return to RX state
        write_reg_system_tick_irq(SLAVE_NEXT_CONNECT_TICK.get());
        *P_ST_HANDLER.lock() = IrqHandlerStatus::Rx;
        return;
    }

    // Set bridge command timeout (200µs from now)
    // This provides a time window for processing bridge commands
    BRIDGE_COMMAND_TIMESTAMP.set(CLOCK_SYS_CLOCK_1US * 200 + read_reg_system_tick());

    // Transmit bridge packets under specific conditions:
    // 1. Connection interval > 10ms AND no old interval pending, OR
    // 2. Connection parameter update is not at the target instant yet
    // This prevents transmission during critical timing updates
    if CLOCK_SYS_CLOCK_1US * 10000 < SLAVE_LINK_INTERVAL.get() && BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.get() == 0 || BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.get() != BLE_PERIPHERAL_CONNECTION_INSTANT.get() {
        tx_packet_bridge();
    }

    // Process any pending status read operations
    if SLAVE_READ_STATUS_BUSY.get() != 0 {
        process_queued_status_responses();
    }

    // Handle pairing operations if transmission buffer is available
    if is_add_packet_buf_ready() {
        let pair_proc_result = pair_proc();
        if pair_proc_result.is_some() {
            let pkt = pair_proc_result.unwrap();
            rf_link_add_tx_packet(&pkt);
        }
    }
    
    // Flush any pending mesh node status updates
    mesh_node_flush_status();
    
    // Generate and send mesh status reports if conditions are met:
    // - Transmission buffer is available
    // - UART manager is not started (to avoid conflicts)
    if is_add_packet_buf_ready() && !app().uart_manager.started() {
        let mut data = [0u8; 20];
        
        // Collect mesh node status data (limit based on available space)
        let count = mesh_node_report_status(&mut data, 10 / MESH_NODE_ST_VAL_LEN);
        
        if count != 0 {
            // Create BLE ATT packet for mesh status reporting
            let mut pkt = Packet {
                att_cmd: PacketAttCmd {
                    head: PacketL2capHead {
                        dma_len: 0x1D,        // DMA length
                        _type: 2,             // L2CAP packet type
                        rf_len: 0x1B,         // RF payload length
                        l2cap_len: 0x17,      // L2CAP payload length
                        chan_id: 4,           // ATT channel ID
                    },
                    opcode: 0x1B,             // ATT Write Without Response
                    handle: 0x12,             // ATT handle for mesh data
                    handle1: 0,
                    value: PacketAttValue {
                        sno: [0; 3],          // Sequence number
                        src: [0; 2],          // Source address
                        dst: [0; 2],          // Destination address
                        // Initialize with mesh status header: [0xdc, vendor_id_low, vendor_id_high, ...]
                        val: [0xdc, VENDOR_ID as u8, (VENDOR_ID >> 8) as u8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                    },
                }
            };

            // Copy the collected mesh status data into the packet payload
            pkt.att_cmd_mut().value.val[3..].copy_from_slice(&data);
            
            // Add the packet to the transmission queue
            rf_link_add_tx_packet(&pkt);

            // Also send status via UART if UART status reporting is enabled
            // This provides debugging/monitoring capability
            if app().uart_manager.uart_status_reporting_enabled() {
                use crate::sdk::drivers::uart::{UartData, UART_DATA_LEN};
                use crate::uart_manager::UartMsg;
                use crate::mesh::MESH_NODE_ST_VAL_LEN;

                let mut uart_msg = UartData {
                    len: UART_DATA_LEN as u32,
                    data: [0; UART_DATA_LEN],
                };
                uart_msg.data[2] = UartMsg::LightStatus as u8;
                uart_msg.data[3..3 + count * MESH_NODE_ST_VAL_LEN].copy_from_slice(&data[..count * MESH_NODE_ST_VAL_LEN]);
                let _ = app().uart_manager.send_message(&uart_msg);
            }
        }
    }

    // Configure radio to return to receive mode for next connection event
    back_to_rxmode_bridge();

    // Connection event timing loop - ensures proper BLE connection event scheduling
    // This loop handles missed connection events and maintains timing synchronization
    loop {
        // Calculate time until next connection event (with 500µs safety margin)
        let mut cur_interval = (SLAVE_NEXT_CONNECT_TICK.get() - (CLOCK_SYS_CLOCK_1US * 500)) - read_reg_system_tick();
        
        // Handle connection parameter update completion
        // When the instant matches and we have an old interval, the update is complete
        if BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.get() != 0 && BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.get() == BLE_PERIPHERAL_CONNECTION_INSTANT.get() {
            // Check for time overflow (0x3fffffff is a large positive number check)
            if 0x3fffffff >= cur_interval {
                cur_interval = 0;  // Force immediate processing
            }
            BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.set(0);  // Clear old interval flag
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

    // Schedule the next connection event interrupt
    write_reg_system_tick_irq(SLAVE_NEXT_CONNECT_TICK.get());
    
    // Transition to receive state to listen for the next packet from master
    *P_ST_HANDLER.lock() = IrqHandlerStatus::Rx;
}

/// Handles the BLE receive state interrupt.
///
/// This function configures the radio for receiving the next packet from the BLE master
/// during an active connection. It sets up the correct channel, access code, CRC, and
/// timing parameters for the upcoming connection event.
///
/// ## BLE Connection Event Flow:
/// 1. Configure RF for the next data channel (frequency hopping)
/// 2. Set connection-specific access code and CRC initialization
/// 3. Calculate timing for the next connection event
/// 4. Start buffered receive (BRX) mode
/// 5. Process any pending timing updates
///
/// ## Timing Management:
/// - Schedules next connection event based on connection interval
/// - Handles window size for slave listening duration
/// - Adjusts timing for OTA operations
/// - Enables timing adjustment mechanisms
///
/// ## Channel Hopping:
/// - Selects next channel using BLE frequency hopping algorithm
/// - Uses connection-specific channel map and hop increment
pub fn configure_ble_receive_state()
{
    // Clear RF interrupt status and stop current radio operations
    write_reg8(0x080050f, 0x80);
    rf_stop_trx();

    // Set link state to BLE receive mode (state 7)
    BLE_PERIPHERAL_LINK_STATE.set(7);

    // Configure RF timing: TX wait and settle time for optimal performance
    write_reg8(0x00800f04, 0x67);

    // Determine the next data channel using BLE frequency hopping
    let chn_map = PKT_INIT.lock().ll_init().chm;
    let next_chan = ble_ll_select_next_data_channel(&chn_map, PKT_INIT.lock().ll_init().hop & 0x1f) as u8;
    rf_set_ble_channel(next_chan);

    // Configure connection-specific access code for packet filtering
    // Access code is a 32-bit value that must match for packet reception
    let aa = PKT_INIT.lock().ll_init().aa;
    let crcinit = PKT_INIT.lock().ll_init().crcinit;
    
    // Reconstruct 32-bit access code from byte array (little-endian format)
    write_reg_rf_access_code(((aa[2] as u32) << 8) | ((aa[1] as u32) << 0x10) | (aa[3] as u32) | ((aa[0] as u32) << 0x18));
    
    // Configure CRC initialization value for packet integrity checking
    write_reg_rf_crc(((crcinit[1] as u32) << 8) | ((crcinit[2] as u32) << 0x10) | crcinit[0] as u32);

    // Calculate next connection event timing
    SLAVE_NEXT_CONNECT_TICK.set(SLAVE_LINK_INTERVAL.get() + read_reg_system_tick_irq());
    
    // Configure interrupt timing based on operational mode
    if OTA_UPDATE_IN_PROGRESS.get() == false {
        // Normal operation: use window size or minimum timing (0xed8 = ~3.8ms)
        if SLAVE_WINDOW_SIZE.get() == 0 || SLAVE_WINDOW_SIZE.get() <= CLOCK_SYS_CLOCK_1US * 0xed8 {
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 0xed8 + read_reg_system_tick());
        } else {
            write_reg_system_tick_irq(read_reg_system_tick() + SLAVE_WINDOW_SIZE.get());
        }
    } else {
        // OTA operation in progress: use longer timeout (10ms)
        write_reg_system_tick_irq(10000 * CLOCK_SYS_CLOCK_1US + read_reg_system_tick());
    }
    
    // Set next state to bridge mode for connection event processing
    *P_ST_HANDLER.lock() = IrqHandlerStatus::Bridge;
    
    // Configure buffered receive timing (start in 100µs)
    BRIDGE_RECEIVE_TIMING_TICK.set(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
    
    // Enable timing adjustment for connection synchronization
    BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(true);
    
    // Start buffered receive mode with empty packet buffer
    rf_start_brx(addr_of!(PKT_EMPTY) as u32, BRIDGE_RECEIVE_TIMING_TICK.get());
    
    // Short delay to ensure radio configuration is stable
    sleep_us(2);

    // Process any pending connection timing updates
    handle_ble_connection_parameter_updates();
}

/// Handles RF transmission complete interrupt.
///
/// This function is called when an RF transmission has completed.
/// It simply clears the RF interrupt status to acknowledge the transmission.
///
/// ## Interrupt Handling:
/// - Clears bit 2 of RF interrupt status (TX complete)
/// - Allows the system to proceed with next operations
/// - Minimal processing to maintain real-time performance
fn handle_rf_transmission_complete()
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
fn handle_rf_packet_reception()
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

    /// Complex packet processing function (marked inline(never) for code size optimization).
    ///
    /// This function handles the detailed processing of received packets including:
    /// - Packet validation and integrity checking
    /// - BLE advertisement and connection request handling
    /// - Mesh network packet processing and forwarding
    /// - Connection timing and synchronization
    /// - Scan response generation
    #[inline(never)]
    fn slow(rx_index: usize, dma_len: u8, light_rx_buff: &mut [LightRxBuff; 4]) {
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
            
            // Handle packets when in advertisement state (BLE_PERIPHERAL_LINK_STATE == 1)
            if BLE_PERIPHERAL_LINK_STATE.get() == 1 {
                // Command 3: BLE Scan Request - Generate and send scan response
                if cmd == 3 {
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

                        return;
                    }
                }

                // Command 5: BLE Connection Request - Initiate connection establishment
                if cmd == 5 {
                    // Verify the connection request is addressed to this device
                    if entry.mac == MAC_ID.lock()[0..4] {
                        // Process the connection request and establish BLE connection
                        // This handles connection parameter negotiation and state setup
                        rf_link_slave_connect(packet, rx_time);

                        return;
                    }
                }
            }

            // Process mesh network packets (when not in OTA mode and not in active BLE RX state)
            if !OTA_UPDATE_IN_PROGRESS.get() && BLE_PERIPHERAL_LINK_STATE.get() != 7 {
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

                return;
            }

            // Process BLE connection data packets
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
            return;
        }

        // Handle cleanup when in BLE receive state (state 7) but packet was invalid
        if BLE_PERIPHERAL_LINK_STATE.get() == 7 {
            // Clear RF interrupt status and stop radio operations
            // This handles the case where we were expecting a BLE packet but received invalid data
            write_reg8(0x80050f, 0);
            rf_stop_trx();
        }
    }

    // Delegate complex packet processing to the slow path function
    // This separation keeps the interrupt handler fast path minimal while providing
    // comprehensive packet processing capabilities in the slow path
    slow(rx_index, dma_len, &mut *light_rx_buff);
}

/// Global flag to track if we're currently executing within an interrupt context.
/// This is used by other parts of the system to determine execution context.
static IS_IRQ_MODE: AtomicBool = AtomicBool::new(false);

/// RAII guard for tracking interrupt execution context.
///
/// This structure automatically sets the global interrupt flag when created
/// and clears it when dropped, ensuring accurate tracking of interrupt context
/// even if the interrupt handler exits early or panics.
///
/// ## Usage:
/// ```rust
/// fn interrupt_handler() {
///     let _guard = IrqTracker::new(); // Sets IRQ flag
///     // ... interrupt handling code ...
/// } // IRQ flag automatically cleared when guard is dropped
/// ```
pub struct IrqTracker {}

impl IrqTracker {
    /// Creates a new interrupt tracker and sets the global IRQ flag.
    ///
    /// ## Returns:
    /// IrqTracker instance that will clear the flag when dropped
    #[inline(always)]
    pub fn new() -> Self {
        IS_IRQ_MODE.store(true, Ordering::Relaxed);
        return IrqTracker {}
    }

    /// Checks if code is currently executing within an interrupt context.
    ///
    /// ## Returns:
    /// `true` if currently in interrupt context, `false` otherwise
    #[inline(always)]
    pub fn in_irq() -> bool {
        IS_IRQ_MODE.load(Ordering::Relaxed)
    }
}

impl Drop for IrqTracker {
    /// Automatically clears the interrupt flag when the tracker is dropped.
    /// This ensures the flag is always cleared, even if the interrupt handler
    /// exits early or encounters an error.
    #[inline(always)]
    fn drop(&mut self) {
        IS_IRQ_MODE.store(false, Ordering::Relaxed);
    }
}

/// Main interrupt handler for the BLE mesh lighting system.
///
/// This is the primary interrupt entry point that handles all system interrupts
/// including RF events, system timers, and application timers. It's executed
/// directly by the hardware interrupt controller.
///
/// ## Critical Performance Requirements:
/// - `#[no_mangle]`: Prevents name mangling for assembly linkage
/// - `#[link_section = ".ram_code"]`: Places code in RAM for fastest execution
/// - `extern "C"`: Uses C calling convention for hardware compatibility
///
/// ## Interrupt Types Handled:
/// 1. **RF Interrupts**: TX complete, RX complete for BLE/mesh communication
/// 2. **System Timer**: BLE state machine transitions (Adv, Bridge, Rx, Listen)
/// 3. **Timer 0**: Clock overflow detection and OTA timeout management
/// 4. **Timer 1**: Light transition stepping for smooth dimming effects
/// 5. **UART**: Serial communication with external systems
///
/// ## Execution Flow:
/// 1. Set interrupt context tracking
/// 2. Handle high-priority RF interrupts (TX/RX)
/// 3. Process system timer and application interrupts
/// 4. Automatically clear interrupt context on exit
// no_mangle because this is referenced as an entrypoint from the assembler bootstrap
#[no_mangle]
#[link_section = ".ram_code"]
extern "C" fn irq_handler() {
    // Track interrupt context for system-wide interrupt awareness
    let _tracker = IrqTracker::new();

    // Handle RF (Radio Frequency) interrupts first (highest priority)
    let irq = read_reg_rf_irq_status();

    // Handle RF transmission complete interrupt
    if irq & FLD_RF_IRQ_MASK::IRQ_TX.bits() != 0 {
        handle_rf_transmission_complete();
    }

    // Handle RF reception complete interrupt
    if irq & FLD_RF_IRQ_MASK::IRQ_RX.bits() != 0 {
        handle_rf_packet_reception();
    }

    /// Handles lower-priority system and application interrupts.
    /// Marked `#[inline(never)]` to keep the main interrupt handler small and fast.
    #[inline(never)]
    fn slow() {
        // Get system interrupt sources
        let irq_source = read_reg_irq_src();
        
        // Handle system timer interrupt (BLE state machine)
        if irq_source & FLD_IRQ::SYSTEM_TIMER.bits() != 0 {
            // Clear the interrupt source
            write_reg_irq_src(FLD_IRQ::SYSTEM_TIMER.bits());
            
            // Get current BLE state and dispatch to appropriate handler
            let state = {
                *P_ST_HANDLER.lock()
            };
            match state {
                IrqHandlerStatus::Adv => handle_ble_advertisement_state(),        // Advertisement state
                IrqHandlerStatus::Bridge => handle_ble_connected_state(),   // Connected bridge state
                IrqHandlerStatus::Rx => configure_ble_receive_state(),      // BLE receive state
                IrqHandlerStatus::Listen => handle_mesh_listening_state(),   // Mesh listening state
                IrqHandlerStatus::None => {}                   // Idle state
            }
        }

        // Handle Timer 0: Clock overflow detection and OTA timeout management
        // This timer runs once per second to prevent clock overflow issues
        if irq_source & FLD_IRQ::TMR0_EN.bits() != 0 {
            write_reg_tmr_sta(FLD_TMR_STA::TMR0.bits());

            // Update 64-bit clock to handle 32-bit overflow
            clock_time64();

            // Handle mesh OTA timeout countdown
            if OTA_UPDATE_MESH_OPERATIONS_BLOCKED.get() {
                if OTA_UPDATE_TIMEOUT_SECONDS.get() != 0 {
                    OTA_UPDATE_TIMEOUT_SECONDS.dec();
                    if OTA_UPDATE_TIMEOUT_SECONDS.get() == 0 {
                        // OTA timeout reached: finish with current status
                        app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(*RF_SLAVE_OTA_FINISHED_FLAG.lock());
                    }
                }
            }
        }

        // Handle Timer 1: Light transition stepping for smooth dimming
        // This timer provides smooth light transitions by stepping through intermediate values
        if irq_source & FLD_IRQ::TMR1_EN.bits() != 0 {
            write_reg_tmr_sta(FLD_TMR_STA::TMR1.bits());

            app().light_manager.transition_step();
        }

        // Handle UART interrupts for external communication
        app().uart_manager.check_irq();
    }

    // Process lower-priority interrupts
    slow();
}

#[cfg(test)]
mod tests {
    use super::*;
    use mry::Any;
    
    // Import mock functions from their original modules
    use crate::sdk::ble_app::ble_ll_channel_selection::mock_ble_ll_build_available_channel_table;
    use crate::sdk::mcu::clock::CLOCK_SYS_CLOCK_1US;

    /// Helper function to reset global state to known values for test isolation.
    /// This ensures each test starts with a clean state.
    fn reset_global_state() {
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(0);
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(0);
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(0);
        SLAVE_LINK_INTERVAL.set(0x9c400); // Default value
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(0);
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.set(false);
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.set(0);
        SLAVE_NEXT_CONNECT_TICK.set(0);
        SLAVE_WINDOW_SIZE.set(0);
        SLAVE_WINDOW_SIZE_UPDATE.set(0);
        BLE_CONN_INTERVAL.set(0);
        BLE_CONN_TIMEOUT.set(0);
        BLE_CONN_OFFSET.set(0);
        
        // Reset channel map to known state
        {
            let mut chn_map = SLAVE_CHN_MAP.lock();
            *chn_map = [0xff, 0xff, 0xff, 0xff, 0x1f]; // Default all channels enabled
        }
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
        assert_eq!(BLE_PERIPHERAL_CONNECTION_INSTANT.get(), 101, 
            "Connection event counter must increment on every call per BLE specification");
    }

    /// Tests Channel Map Update procedure (LL_CHANNEL_MAP_IND) per BLE specification.
    ///
    /// This test verifies compliance with BLE Core Spec v5.4, Section 5.1.12:
    /// - Channel map updates are applied at the specified instant
    /// - The channel table is rebuilt with the new channel map
    /// - The update flag is cleared after processing
    /// - The packet initialization structure is updated
    #[test]
    #[mry::lock(ble_ll_build_available_channel_table)]
    fn test_channel_map_update_ble_specification_compliance() {
        reset_global_state();
        
        // Setup: Configure for channel map update at specific instant
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(1);        // Type 1 = Channel Map Update
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(50);             // Current instant
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(51);        // Update should apply at instant 51
        
        // Setup new channel map (enable channels 0, 2, 4, 6, 8 - selective hopping)
        let test_channel_map = [0x55, 0x55, 0x55, 0x55, 0x15]; // Binary: 01010101...
        {
            let mut chn_map = SLAVE_CHN_MAP.lock();
            *chn_map = test_channel_map;
        }
        
        // Setup mock for channel table building
        mock_ble_ll_build_available_channel_table(Any, Any).returns(());
        
        // Execute: This should increment instant to 51 and trigger channel map update
        handle_ble_connection_parameter_updates();
        
        // Verify: Channel map update was processed correctly
        assert_eq!(BLE_PERIPHERAL_CONNECTION_INSTANT.get(), 51, "Instant should be incremented");
        assert_eq!(BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(), 0, "Update flag should be cleared after processing");
        
        // Verify: Channel table was rebuilt with correct parameters
        mock_ble_ll_build_available_channel_table(test_channel_map.to_vec(), false)
            .assert_called(1);
        
        // Verify: Packet initialization structure was updated
        {
            let pkt_init = PKT_INIT.lock();
            assert_eq!(pkt_init.ll_init().chm, test_channel_map, 
                "Packet init channel map should match new channel map");
        }
    }

    /// Tests that channel map updates only occur at the correct instant.
    ///
    /// Per BLE specification, timing updates must be synchronized between master and slave.
    /// The update should only be applied when BLE_PERIPHERAL_NEXT_UPDATE_INSTANT matches BLE_PERIPHERAL_CONNECTION_INSTANT.
    #[test]
    #[mry::lock(ble_ll_build_available_channel_table)]
    fn test_channel_map_update_instant_synchronization() {
        reset_global_state();
        
        // Setup: Channel map update scheduled for future instant
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(1);        // Channel map update pending
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(10);             // Current instant
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(15);        // Update scheduled for instant 15
        
        // Setup mock (should not be called since instant doesn't match)
        mock_ble_ll_build_available_channel_table(Any, Any).returns(());
        
        // Execute: Should increment instant but not apply update
        handle_ble_connection_parameter_updates();
        
        // Verify: Instant incremented but update not processed
        assert_eq!(BLE_PERIPHERAL_CONNECTION_INSTANT.get(), 11, "Instant should increment");
        assert_eq!(BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(), 1, "Update flag should remain set");
        
        // Verify: Channel table rebuild was not called
        mock_ble_ll_build_available_channel_table(Any, Any).assert_called(0);
        
        // Execute again multiple times to reach the target instant
        for _ in 0..4 {
            handle_ble_connection_parameter_updates();
        }
        
        // Verify: Now at instant 15, update should be processed
        assert_eq!(BLE_PERIPHERAL_CONNECTION_INSTANT.get(), 15);
        assert_eq!(BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(), 0, "Update should now be processed");
        mock_ble_ll_build_available_channel_table(Any, Any).assert_called(1);
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
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(2);        // Type 2 = Connection Parameter Update
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(25);             // Current instant  
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(26);        // Update at instant 26
        
        // Setup new connection parameters (realistic BLE values)
        BLE_CONN_INTERVAL.set(32 * CLOCK_SYS_CLOCK_1US);    // 32ms interval (25.6 connection intervals)
        BLE_CONN_TIMEOUT.set(4000 * CLOCK_SYS_CLOCK_1US);   // 4 second supervision timeout
        BLE_CONN_OFFSET.set(1000 * CLOCK_SYS_CLOCK_1US);    // 1ms offset
        SLAVE_NEXT_CONNECT_TICK.set(10000 * CLOCK_SYS_CLOCK_1US); // Next event at 10ms
        SLAVE_WINDOW_SIZE_UPDATE.set(5 * CLOCK_SYS_CLOCK_1US);     // 5ms window from master
        
        // Calculate expected timing before calling the function
        let expected_update_time = BLE_CONN_OFFSET.get() + SLAVE_NEXT_CONNECT_TICK.get();
        
        // Execute: Should increment instant and apply connection parameter update
        handle_ble_connection_parameter_updates();
        
        // Verify: Instant incremented and update flag cleared
        assert_eq!(BLE_PERIPHERAL_CONNECTION_INSTANT.get(), 26, "Instant should increment to trigger update");
        assert_eq!(BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(), 0, "Update flag should be cleared");
        
        // Verify: Connection parameters applied correctly
        assert_eq!(SLAVE_LINK_INTERVAL.get(), 32 * CLOCK_SYS_CLOCK_1US,
            "Connection interval should be updated to new value");
        assert_eq!(BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.get(), 4000 * CLOCK_SYS_CLOCK_1US,
            "Supervision timeout should be updated to new value");
        
        // Verify: Timing calculations per BLE specification
        assert_eq!(BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.get(), expected_update_time,
            "Update timing should include connection offset");
        assert_eq!(SLAVE_NEXT_CONNECT_TICK.get(), expected_update_time,
            "Next connection tick should use new timing");
        
        // Verify: Update flag is set for timing coordination
        assert_eq!(BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.get(), true,
            "Timing update flag should be set during parameter transition");
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
        
        BLE_CONN_INTERVAL.set(50000 * CLOCK_SYS_CLOCK_1US);    // 50000ms interval (large enough to avoid underflow)
        SLAVE_WINDOW_SIZE_UPDATE.set(10 * CLOCK_SYS_CLOCK_1US); // Master wants 10ms window
        BLE_CONN_OFFSET.set(0);
        SLAVE_NEXT_CONNECT_TICK.set(0);
        
        handle_ble_connection_parameter_updates();
        
        // Calculated window = 50000ms - 1.25ms = 49998.75ms, but master wants 10ms
        assert_eq!(SLAVE_WINDOW_SIZE.get(), 10 * CLOCK_SYS_CLOCK_1US,
            "Should use master's smaller window size");
        
        // Test Case 2: Calculated size is smaller (should use calculated size)
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(2);
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(1);
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(2);
        
        BLE_CONN_INTERVAL.set(10 * CLOCK_SYS_CLOCK_1US);    // 10ms interval
        SLAVE_WINDOW_SIZE_UPDATE.set(50 * CLOCK_SYS_CLOCK_1US); // Master wants 50ms (too big)
        
        handle_ble_connection_parameter_updates();
        
        // Calculated window = 10ms - 1.25ms = 8.75ms, master wants 50ms
        let expected_window = (10 * CLOCK_SYS_CLOCK_1US).saturating_sub(CLOCK_SYS_CLOCK_1US * 0x4e2);
        assert_eq!(SLAVE_WINDOW_SIZE.get(), expected_window,
            "Should use calculated size when master requests too large window");
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
        BLE_CONN_INTERVAL.set(CLOCK_SYS_CLOCK_1US * 1000);  // 1ms interval (below BLE minimum)
        SLAVE_WINDOW_SIZE_UPDATE.set(CLOCK_SYS_CLOCK_1US * 2000); // 2ms window request
        BLE_CONN_OFFSET.set(0);
        SLAVE_NEXT_CONNECT_TICK.set(0);
        
        handle_ble_connection_parameter_updates();
        
        // Window calculation: 1ms - 1.25ms = negative, saturating_sub gives 0
        // Since calculated window (0) < master's window (2000), use calculated window (0)
        assert_eq!(SLAVE_WINDOW_SIZE.get(), 0,
            "Should handle underflow gracefully by using calculated window of 0");
    }

    /// Tests that updates don't occur when instant synchronization is off.
    ///
    /// This verifies the critical BLE requirement that parameter updates must be
    /// synchronized between master and slave at the exact same connection event.
    #[test]
    #[mry::lock(ble_ll_build_available_channel_table)]
    fn test_no_update_when_instant_mismatch() {
        reset_global_state();
        
        // Setup: Updates pending but instants don't match
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(1);        // Channel map update pending
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(5);              // Current instant
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(10);        // Update scheduled for instant 10
        
        // Store original values to verify they don't change
        let original_interval = SLAVE_LINK_INTERVAL.get();
        let original_timeout = BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.get();
        
        mock_ble_ll_build_available_channel_table(Any, Any).returns(());
        
        // Execute function
        handle_ble_connection_parameter_updates();
        
        // Verify: No updates applied
        assert_eq!(BLE_PERIPHERAL_CONNECTION_INSTANT.get(), 6, "Only instant should increment");
        assert_eq!(BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(), 1, "Update flag should remain set");
        assert_eq!(SLAVE_LINK_INTERVAL.get(), original_interval, "Interval unchanged");
        assert_eq!(BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.get(), original_timeout, "Timeout unchanged");
        
        // Verify: No channel operations called
        mock_ble_ll_build_available_channel_table(Any, Any).assert_called(0);
    }

    /// Tests connection parameter update type detection.
    ///
    /// The function must correctly distinguish between:
    /// - Type 1: Channel Map Update (LL_CHANNEL_MAP_IND)
    /// - Type 2: Connection Parameter Update (LL_CONNECTION_UPDATE_IND)
    /// - No update: BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP = 0
    #[test]
    #[mry::lock(ble_ll_build_available_channel_table)]
    fn test_update_type_detection() {
        reset_global_state();
        
        // Test Case 1: No update pending
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(0);
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(0);
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(1);
        
        let original_interval = SLAVE_LINK_INTERVAL.get();
        mock_ble_ll_build_available_channel_table(Any, Any).returns(());
        
        handle_ble_connection_parameter_updates();
        
        assert_eq!(BLE_PERIPHERAL_CONNECTION_INSTANT.get(), 1, "Instant should still increment");
        assert_eq!(SLAVE_LINK_INTERVAL.get(), original_interval, "No parameter changes");
        mock_ble_ll_build_available_channel_table(Any, Any).assert_called(0);
        
        // Test Case 2: Invalid update type
        BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(99);       // Invalid type
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(1);
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(2);
        
        handle_ble_connection_parameter_updates();
        
        assert_eq!(BLE_PERIPHERAL_CONNECTION_INSTANT.get(), 2, "Instant should increment");
        assert_eq!(BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(), 99, "Invalid type should remain unchanged");
        mock_ble_ll_build_available_channel_table(Any, Any).assert_called(0);
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
        BLE_PERIPHERAL_CONNECTION_INSTANT.set(42);             // Current connection event
        BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(43);        // Update at next event
        
        // BLE-compliant parameter values
        BLE_CONN_INTERVAL.set(24 * CLOCK_SYS_CLOCK_1US);    // 24ms (19.2 intervals)
        BLE_CONN_TIMEOUT.set(6000 * CLOCK_SYS_CLOCK_1US);   // 6 second timeout
        BLE_CONN_OFFSET.set(500 * CLOCK_SYS_CLOCK_1US);     // 500µs offset
        SLAVE_NEXT_CONNECT_TICK.set(50000 * CLOCK_SYS_CLOCK_1US); // 50ms base time
        SLAVE_WINDOW_SIZE_UPDATE.set(3 * CLOCK_SYS_CLOCK_1US);     // 3ms window
        
        // Record pre-update state
        let pre_update_flags = BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.get();
        
        // Execute update
        handle_ble_connection_parameter_updates();
        
        // Verify BLE specification compliance
        assert_eq!(BLE_PERIPHERAL_CONNECTION_INSTANT.get(), 43, "Update must occur at specified instant");
        assert_eq!(BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(), 0, "Update flag must be cleared");
        assert_eq!(BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.get(), true, "Timing flag must be set");
        
        // Verify timing calculations
        let expected_timing = BLE_CONN_OFFSET.get() + 50000 * CLOCK_SYS_CLOCK_1US;
        assert_eq!(BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.get(), expected_timing,
            "Update timing must include connection offset per BLE spec");
        assert_eq!(SLAVE_NEXT_CONNECT_TICK.get(), expected_timing,
            "Next connection event must use updated timing");
        
        // Verify window size follows BLE specification formula
        let calculated_window = BLE_CONN_INTERVAL.get().saturating_sub(CLOCK_SYS_CLOCK_1US * 0x4e2);
        let expected_window = if SLAVE_WINDOW_SIZE_UPDATE.get() < calculated_window {
            SLAVE_WINDOW_SIZE_UPDATE.get()
        } else {
            calculated_window
        };
        assert_eq!(SLAVE_WINDOW_SIZE.get(), expected_window,
            "Window size must follow BLE specification: min(calculated, master_requested)");
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
        assert_eq!(BLE_PERIPHERAL_CONNECTION_INSTANT.get(), instant_after + 1, "Only instant should increment");
        assert_eq!(SLAVE_LINK_INTERVAL.get(), interval_after, "Interval should be stable");
        assert_eq!(BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.get(), timeout_after, "Timeout should be stable");
        assert_eq!(BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.get(), timing_after, "Timing should be stable");
        assert_eq!(SLAVE_WINDOW_SIZE.get(), window_after, "Window should be stable");
        assert_eq!(BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.get(), update_flag_after, "Update flag should remain clear");
    }
}
