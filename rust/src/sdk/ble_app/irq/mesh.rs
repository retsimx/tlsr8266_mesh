//! # Mesh Networking Module
//!
//! This module handles mesh network communication, node status reporting,
//! and mesh listening operations for the TLSR8266-based mesh lighting system.
//!
//! ## Key Responsibilities:
//! - Mesh node status collection and reporting
//! - Mesh listening and discovery operations
//! - Mesh network state management
//! - Online status broadcasting

use core::ptr::addr_of;
use core::slice;

use crate::app;
use crate::common::SYS_CHN_LISTEN;
use crate::mesh::MESH_NODE_ST_VAL_LEN;
use crate::sdk::ble_app::light_ll::connection_management::back_to_rxmode_bridge;
use crate::sdk::ble_app::light_ll::mesh_management::{
    mesh_node_flush_status, mesh_report_status_enable_mask, mesh_send_online_status,
};
use crate::sdk::ble_app::light_ll::status_management::{app_bridge_cmd_handle, tx_packet_bridge};
use crate::sdk::ble_app::rf_drv_8266::*;
use crate::sdk::light::{
    RfOperationState, ADV_INTERVAL2LISTEN_INTERVAL, ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL,
};
use crate::sdk::mcu::clock::CLOCK_SYS_CLOCK_1US;
use crate::sdk::mcu::register::*;
use crate::state::*;

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
pub fn mesh_node_report_status(params: &mut [u8], len: usize) -> usize {
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
            params[params_idx..params_idx + MESH_NODE_ST_VAL_LEN]
                .copy_from_slice(bytemuck::bytes_of(&mesh_node_st[idx].val));

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
pub fn configure_rf_for_mesh_listening() {
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
    rf_set_ble_channel(
        SYS_CHN_LISTEN[MESH_LISTEN_CYCLE_COUNT.get() as usize % SYS_CHN_LISTEN.len()],
    );

    // Enable receive mode to start listening for packets
    rf_set_rxmode();

    // Set the link state to indicate we're in mesh listening mode
    // State 4 = mesh listening state
    BLE_PERIPHERAL_LINK_STATE.set(crate::sdk::light::BlePeripheralLinkState::Mesh);
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
#[cfg_attr(test, mry::mry)]
pub fn handle_mesh_listening_state() {
    // Set the link state to listening mode (state 4)
    BLE_PERIPHERAL_LINK_STATE.set(crate::sdk::light::BlePeripheralLinkState::Mesh);

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
        *CURRENT_RF_STATE.lock() = RfOperationState::Advertising;
        return;
    }

    // Handle periodic advertising and online status reporting
    if !MESH_DEVICE_ONLINE_STATUS.get() {
        // Increment the listen counter for interval calculations
        MESH_LISTEN_CYCLE_COUNT.inc();

        // Determine if we should advertise based on the listen interval ratio
        // This creates a pattern where advertising happens periodically during listening
        BLE_ADVERTISING_ENABLED
            .set(MESH_LISTEN_CYCLE_COUNT.get() % ADV_INTERVAL2LISTEN_INTERVAL as u32 != 0);

        // Determine if we should send online status based on the interval ratio
        // This ensures the device periodically reports its presence to the mesh
        MESH_DEVICE_ONLINE_STATUS.set(
            MESH_LISTEN_CYCLE_COUNT.get() % ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL as u32 != 0,
        );

        // If advertising is now scheduled, add randomization to prevent collisions
        if BLE_ADVERTISING_ENABLED.get() {
            // Use XOR with random number to create pseudo-random timing offset
            // The 0x7fff mask limits the randomization to prevent excessive delays
            write_reg_system_tick_irq(
                (((read_reg_system_tick() ^ read_reg_rnd_number() as u32) & 0x7fff)
                    * CLOCK_SYS_CLOCK_1US
                    + read_reg_system_tick_irq()),
            );
            *CURRENT_RF_STATE.lock() = RfOperationState::Advertising;
            return;
        }

        // If no online status is scheduled, continue listening
        if !MESH_DEVICE_ONLINE_STATUS.get() {
            return;
        }
    }

    // Default transition to advertisement state for online status or regular advertising
    *CURRENT_RF_STATE.lock() = RfOperationState::Advertising;
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::common::SYS_CHN_LISTEN;
    use crate::mesh::{MeshNodeStT, MeshNodeStValT, MESH_NODE_ST_PAR_LEN, MESH_NODE_ST_VAL_LEN};
    use crate::sdk::light::{
        BlePeripheralLinkState, RfOperationState, ADV_INTERVAL2LISTEN_INTERVAL,
        ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL,
    };
    use crate::sdk::mcu::clock::CLOCK_SYS_CLOCK_1US;
    use mry::Any;

    // Import mock functions for RF operations
    use crate::sdk::ble_app::rf_drv_8266::{
        mock_rf_set_ble_access_code, mock_rf_set_ble_channel, mock_rf_set_ble_crc_adv,
        mock_rf_set_rxmode, mock_rf_stop_trx,
    };

    // Import mock functions for register operations
    use crate::sdk::mcu::register::{
        mock_read_reg_rnd_number, mock_read_reg_system_tick, mock_write_reg_system_tick_irq,
    };

    // Import mock functions for app operations
    use crate::sdk::ble_app::light_ll::status_management::mock_app_bridge_cmd_handle;

    /// Helper function to reset global state to known values for test isolation.
    /// This ensures each test starts with a clean state.
    fn reset_global_state() {
        BLE_ADVERTISING_ENABLED.set(false);
        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Disconnected);
        MESH_DEVICE_ONLINE_STATUS.set(false);
        MESH_NODE_REPORT_ENABLE.set(false);
        MESH_LISTEN_CYCLE_COUNT.set(0);
        MESH_LISTEN_INTERVAL_US.set(100000); // 100ms default
        PAIR_AC.set(0x12345678);
        *CURRENT_RF_STATE.lock() = RfOperationState::MeshListening;

        // Reset mesh node structures
        let mut mesh_node_mask = MESH_NODE_MASK.lock();
        for i in 0..mesh_node_mask.len() {
            mesh_node_mask[i] = 0;
        }

        let mut mesh_node_st = MESH_NODE_ST.lock();
        for i in 0..mesh_node_st.len() {
            mesh_node_st[i] = MeshNodeStT {
                tick: 0,
                val: MeshNodeStValT {
                    dev_adr: 0,
                    sn: 0,
                    par: [0; MESH_NODE_ST_PAR_LEN],
                },
            };
        }
    }

    /// Helper function to set up a mock mesh node with status data
    fn setup_mock_mesh_node(
        idx: usize,
        dev_adr: u8,
        sn: u8,
        tick: u16,
        par: [u8; MESH_NODE_ST_PAR_LEN],
    ) {
        let mut mesh_node_mask = MESH_NODE_MASK.lock();
        let mut mesh_node_st = MESH_NODE_ST.lock();

        // Set the bit in the mask to indicate this node has pending status
        let mask_index = idx / 32;
        let mask_bit = idx % 32;
        mesh_node_mask[mask_index] |= 1 << mask_bit;

        // Set the node status data
        mesh_node_st[idx] = MeshNodeStT {
            tick,
            val: MeshNodeStValT { dev_adr, sn, par },
        };
    }

    // ================================================================================
    // Tests for mesh_node_report_status function
    // ================================================================================

    /// Tests mesh node status reporting when reporting is disabled.
    ///
    /// Verifies that when MESH_NODE_REPORT_ENABLE is false:
    /// - The function returns 0 immediately
    /// - No mesh node data is processed
    /// - The output buffer remains unchanged
    #[test]
    fn test_mesh_node_report_status_disabled() {
        // Reset state for clean test
        reset_global_state();

        // Setup: Reporting disabled
        MESH_NODE_REPORT_ENABLE.set(false);

        // Setup some mock nodes that would normally be reported
        setup_mock_mesh_node(0, 0x01, 0x10, 1000, [0xAA, 0xBB]);
        setup_mock_mesh_node(1, 0x02, 0x20, 2000, [0xCC, 0xDD]);

        // Prepare output buffer
        let mut params = [0xFF; 20]; // Initialize with non-zero to verify no changes

        // Execute function
        let result = mesh_node_report_status(&mut params, 2);

        // Verify no statuses were reported
        assert_eq!(result, 0, "Should return 0 when reporting is disabled");

        // Verify output buffer was not modified (still all 0xFF)
        assert_eq!(
            params, [0xFF; 20],
            "Output buffer should remain unchanged when reporting is disabled"
        );
    }

    /// Tests mesh node status reporting with no pending statuses.
    ///
    /// Verifies that when no nodes have pending status updates:
    /// - The function returns 0
    /// - The output buffer is cleared but no data is written
    #[test]
    fn test_mesh_node_report_status_no_pending() {
        // Reset state for clean test
        reset_global_state();

        // Setup: Reporting enabled but no pending statuses
        MESH_NODE_REPORT_ENABLE.set(true);

        // Prepare output buffer
        let mut params = [0xFF; 20];

        // Execute function
        let result = mesh_node_report_status(&mut params, 5);

        // Verify no statuses were reported
        assert_eq!(
            result, 0,
            "Should return 0 when no nodes have pending status"
        );

        // Verify output buffer was cleared (first MESH_NODE_ST_VAL_LEN * len bytes should be 0)
        let expected_clear_bytes = MESH_NODE_ST_VAL_LEN * 5;
        assert_eq!(
            &params[0..expected_clear_bytes],
            &vec![0u8; expected_clear_bytes][..],
            "Output buffer should be cleared when no statuses are pending"
        );
    }

    /// Tests mesh node status reporting with single online node.
    ///
    /// Verifies proper status reporting for one node:
    /// - Correct data is copied to output buffer
    /// - Mask bit is cleared after reporting
    /// - Function returns correct count
    #[test]
    fn test_mesh_node_report_status_single_online_node() {
        // Reset state for clean test
        reset_global_state();

        // Setup: Reporting enabled with one online node
        MESH_NODE_REPORT_ENABLE.set(true);
        setup_mock_mesh_node(0, 0x01, 0x10, 1500, [0xAA, 0xBB]); // tick > 0 = online

        // Prepare output buffer
        let mut params = [0xFF; 20];

        // Execute function
        let result = mesh_node_report_status(&mut params, 5);

        // Verify one status was reported
        assert_eq!(result, 1, "Should return 1 for one reported node");

        // Verify the reported data matches the node data
        assert_eq!(params[0], 0x01, "Device address should match");
        assert_eq!(params[1], 0x10, "Sequence number should match");
        assert_eq!(params[2], 0xAA, "Parameter 0 should match");
        assert_eq!(params[3], 0xBB, "Parameter 1 should match");

        // Verify the mask bit was cleared (no longer pending)
        let mesh_node_mask = MESH_NODE_MASK.lock();
        assert_eq!(
            mesh_node_mask[0] & 1,
            0,
            "Mask bit should be cleared after reporting"
        );
    }

    /// Tests mesh node status reporting with single offline node.
    ///
    /// Verifies special handling for offline nodes:
    /// - Sequence number is set to 0 for offline nodes (tick == 0)
    /// - Other data is copied correctly
    /// - Mask bit is cleared after reporting
    #[test]
    fn test_mesh_node_report_status_single_offline_node() {
        // Reset state for clean test
        reset_global_state();

        // Setup: Reporting enabled with one offline node
        MESH_NODE_REPORT_ENABLE.set(true);
        setup_mock_mesh_node(2, 0x03, 0x30, 0, [0xCC, 0xDD]); // tick == 0 = offline

        // Prepare output buffer
        let mut params = [0xFF; 20];

        // Execute function
        let result = mesh_node_report_status(&mut params, 5);

        // Verify one status was reported
        assert_eq!(result, 1, "Should return 1 for one reported node");

        // Verify the reported data - sequence number should be forced to 0 for offline nodes
        assert_eq!(params[0], 0x03, "Device address should match");
        assert_eq!(
            params[1], 0x00,
            "Sequence number should be 0 for offline node"
        );
        assert_eq!(params[2], 0xCC, "Parameter 0 should match");
        assert_eq!(params[3], 0xDD, "Parameter 1 should match");

        // Verify the mask bit was cleared
        let mesh_node_mask = MESH_NODE_MASK.lock();
        assert_eq!(
            mesh_node_mask[0] & (1 << 2),
            0,
            "Mask bit should be cleared after reporting"
        );
    }

    /// Tests mesh node status reporting with multiple nodes.
    ///
    /// Verifies batch processing of multiple node statuses:
    /// - Multiple nodes are processed in order
    /// - Each gets correct data in the output buffer
    /// - All mask bits are cleared
    /// - Function returns correct total count
    #[test]
    fn test_mesh_node_report_status_multiple_nodes() {
        // Reset state for clean test
        reset_global_state();

        // Setup: Reporting enabled with multiple nodes
        MESH_NODE_REPORT_ENABLE.set(true);
        setup_mock_mesh_node(0, 0x01, 0x10, 1000, [0xAA, 0xBB]);
        setup_mock_mesh_node(1, 0x02, 0x20, 2000, [0xCC, 0xDD]);
        setup_mock_mesh_node(3, 0x04, 0x40, 0, [0xEE, 0xFF]); // offline node

        // Prepare output buffer
        let mut params = [0x00; 20];

        // Execute function
        let result = mesh_node_report_status(&mut params, 5);

        // Verify three statuses were reported
        assert_eq!(result, 3, "Should return 3 for three reported nodes");

        // Verify first node data
        assert_eq!(params[0], 0x01, "First node device address should match");
        assert_eq!(params[1], 0x10, "First node sequence number should match");
        assert_eq!(params[2], 0xAA, "First node parameter 0 should match");
        assert_eq!(params[3], 0xBB, "First node parameter 1 should match");

        // Verify second node data
        assert_eq!(params[4], 0x02, "Second node device address should match");
        assert_eq!(params[5], 0x20, "Second node sequence number should match");
        assert_eq!(params[6], 0xCC, "Second node parameter 0 should match");
        assert_eq!(params[7], 0xDD, "Second node parameter 1 should match");

        // Verify third node data (offline node with sn forced to 0)
        assert_eq!(params[8], 0x04, "Third node device address should match");
        assert_eq!(
            params[9], 0x00,
            "Third node sequence number should be 0 for offline"
        );
        assert_eq!(params[10], 0xEE, "Third node parameter 0 should match");
        assert_eq!(params[11], 0xFF, "Third node parameter 1 should match");

        // Verify all mask bits were cleared
        let mesh_node_mask = MESH_NODE_MASK.lock();
        assert_eq!(
            mesh_node_mask[0] & 0b1011,
            0,
            "All reported node mask bits should be cleared"
        );
    }

    /// Tests mesh node status reporting with buffer size limit.
    ///
    /// Verifies that reporting respects buffer size limits:
    /// - Only processes up to the specified limit
    /// - Remaining nodes stay pending for next call
    /// - Function returns correct count of processed nodes
    #[test]
    fn test_mesh_node_report_status_buffer_limit() {
        // Reset state for clean test
        reset_global_state();

        // Setup: Reporting enabled with more nodes than buffer can hold
        MESH_NODE_REPORT_ENABLE.set(true);
        setup_mock_mesh_node(0, 0x01, 0x10, 1000, [0xAA, 0xBB]);
        setup_mock_mesh_node(1, 0x02, 0x20, 2000, [0xCC, 0xDD]);
        setup_mock_mesh_node(2, 0x03, 0x30, 3000, [0xEE, 0xFF]);

        // Prepare small output buffer (only room for 2 nodes)
        let mut params = [0x00; 8]; // 2 nodes * 4 bytes each

        // Execute function with limit of 2 nodes
        let result = mesh_node_report_status(&mut params, 2);

        // Verify only 2 statuses were reported
        assert_eq!(result, 2, "Should return 2 when limited by buffer size");

        // Verify first two nodes were processed
        assert_eq!(params[0], 0x01, "First node should be processed");
        assert_eq!(params[4], 0x02, "Second node should be processed");

        // Verify third node is still pending (mask bit not cleared)
        let mesh_node_mask = MESH_NODE_MASK.lock();
        assert_eq!(
            mesh_node_mask[0] & (1 << 2),
            1 << 2,
            "Third node should still be pending"
        );
        assert_eq!(
            mesh_node_mask[0] & 0b11,
            0,
            "First two nodes should be cleared"
        );
    }

    // ================================================================================
    // Tests for configure_rf_for_mesh_listening function
    // ================================================================================

    /// Tests RF configuration for mesh listening.
    ///
    /// Verifies that the RF transceiver is properly configured:
    /// - RF operations are stopped
    /// - Access code is set to pairing access code
    /// - CRC is configured for advertisements
    /// - Channel is set based on listen cycle count
    /// - Receive mode is enabled
    /// - Link state is set to mesh mode
    #[test]
    #[mry::lock(rf_stop_trx)]
    #[mry::lock(rf_set_ble_access_code)]
    #[mry::lock(rf_set_ble_crc_adv)]
    #[mry::lock(rf_set_ble_channel)]
    #[mry::lock(rf_set_rxmode)]
    fn test_configure_rf_for_mesh_listening() {
        // Setup mocks
        mock_rf_stop_trx().returns(());
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_rf_set_ble_channel(Any).returns(());
        mock_rf_set_rxmode().returns(());

        // Reset state for clean test
        reset_global_state();

        // Setup: Set specific pairing access code and listen cycle count
        PAIR_AC.set(0x12345678);
        MESH_LISTEN_CYCLE_COUNT.set(1); // Should select SYS_CHN_LISTEN[1]

        // Execute function
        configure_rf_for_mesh_listening();

        // Verify RF operations were called in correct sequence
        mock_rf_stop_trx().assert_called(1);
        mock_rf_set_ble_access_code(0x12345678).assert_called(1);
        mock_rf_set_ble_crc_adv().assert_called(1);
        mock_rf_set_ble_channel(SYS_CHN_LISTEN[1]).assert_called(1);
        mock_rf_set_rxmode().assert_called(1);

        // Verify link state was set to mesh mode
        assert_eq!(
            BLE_PERIPHERAL_LINK_STATE.get(),
            BlePeripheralLinkState::Mesh,
            "Link state should be set to mesh mode"
        );
    }

    /// Tests RF channel cycling for mesh listening.
    ///
    /// Verifies that the mesh listening channel cycles through all available channels:
    /// - Channel selection is based on cycle count modulo array length
    /// - All channels in SYS_CHN_LISTEN are used in rotation
    #[test]
    #[mry::lock(rf_stop_trx)]
    #[mry::lock(rf_set_ble_access_code)]
    #[mry::lock(rf_set_ble_crc_adv)]
    #[mry::lock(rf_set_ble_channel)]
    #[mry::lock(rf_set_rxmode)]
    fn test_configure_rf_for_mesh_listening_channel_cycling() {
        // Setup mocks
        mock_rf_stop_trx().returns(());
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_rf_set_ble_channel(Any).returns(());
        mock_rf_set_rxmode().returns(());

        // Reset state for clean test
        reset_global_state();

        // Test each channel in the cycle
        for i in 0..SYS_CHN_LISTEN.len() * 2 {
            // Test two full cycles
            MESH_LISTEN_CYCLE_COUNT.set(i as u32);
            let expected_channel = SYS_CHN_LISTEN[i % SYS_CHN_LISTEN.len()];

            configure_rf_for_mesh_listening();
        }

        // Verify that rf_set_ble_channel was called 8 times total (4 channels * 2 cycles)
        mock_rf_set_ble_channel(Any).assert_called(8);
    }

    // ================================================================================
    // Tests for handle_mesh_listening_state function
    // ================================================================================

    /// Tests mesh listening state with immediate advertising request.
    ///
    /// Verifies that when advertising is enabled:
    /// - Link state is set to mesh mode
    /// - RF operations are stopped
    /// - Bridge commands are processed
    /// - RF is configured for mesh listening
    /// - Listen interval is scheduled
    /// - Advertising takes priority with 7ms timing
    /// - State transitions to advertisement
    #[test]
    #[mry::lock(rf_stop_trx)]
    #[mry::lock(rf_set_ble_access_code)]
    #[mry::lock(rf_set_ble_crc_adv)]
    #[mry::lock(rf_set_ble_channel)]
    #[mry::lock(rf_set_rxmode)]
    #[mry::lock(read_reg_system_tick)]
    #[mry::lock(write_reg_system_tick_irq)]
    #[mry::lock(app_bridge_cmd_handle)]
    fn test_handle_mesh_listening_state_immediate_advertising() {
        // Setup mocks
        mock_rf_stop_trx().returns(());
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_rf_set_ble_channel(Any).returns(());
        mock_rf_set_rxmode().returns(());
        mock_read_reg_system_tick().returns(10000);
        mock_write_reg_system_tick_irq(Any).returns(());
        mock_app_bridge_cmd_handle(Any).returns(());

        // Reset state for clean test
        reset_global_state();

        // Setup: Advertising enabled
        BLE_ADVERTISING_ENABLED.set(true);
        MESH_LISTEN_INTERVAL_US.set(50000); // 50ms

        // Execute function
        handle_mesh_listening_state();

        // Verify initial setup was done
        assert_eq!(
            BLE_PERIPHERAL_LINK_STATE.get(),
            BlePeripheralLinkState::Mesh,
            "Link state should be set to mesh mode"
        );
        mock_rf_stop_trx().assert_called(2); // Called once in handle function, once in configure
        mock_app_bridge_cmd_handle(10000).assert_called(1);

        // Verify RF was configured for mesh listening
        mock_rf_set_ble_access_code(Any).assert_called(1);
        mock_rf_set_ble_crc_adv().assert_called(1);
        mock_rf_set_ble_channel(Any).assert_called(1);
        mock_rf_set_rxmode().assert_called(1);

        // Verify listen interval was scheduled
        mock_write_reg_system_tick_irq(10000 + 50000).assert_called(1);

        // Verify advertising timing (7ms) was scheduled
        let expected_adv_time = CLOCK_SYS_CLOCK_1US * 7000 + 10000;
        mock_write_reg_system_tick_irq(expected_adv_time).assert_called(1);

        // Verify state transitioned to advertisement
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Advertising,
            "State should transition to advertisement"
        );
    }

    /// Tests mesh listening state with periodic advertising cycle.
    ///
    /// Verifies the periodic advertising logic:
    /// - Listen cycle count is incremented
    /// - Advertising is enabled based on interval ratio
    /// - Online status is enabled based on interval ratio
    /// - Random timing is applied to prevent collisions
    /// - State transitions to advertisement when scheduled
    #[test]
    #[mry::lock(rf_stop_trx)]
    #[mry::lock(rf_set_ble_access_code)]
    #[mry::lock(rf_set_ble_crc_adv)]
    #[mry::lock(rf_set_ble_channel)]
    #[mry::lock(rf_set_rxmode)]
    #[mry::lock(read_reg_system_tick)]
    #[mry::lock(write_reg_system_tick_irq)]
    #[mry::lock(read_reg_rnd_number)]
    #[mry::lock(app_bridge_cmd_handle)]
    fn test_handle_mesh_listening_state_periodic_advertising() {
        // Setup mocks
        mock_rf_stop_trx().returns(());
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_rf_set_ble_channel(Any).returns(());
        mock_rf_set_rxmode().returns(());
        mock_read_reg_system_tick().returns(20000);
        mock_write_reg_system_tick_irq(Any).returns(());
        mock_read_reg_rnd_number().returns(0x1234);
        mock_app_bridge_cmd_handle(Any).returns(());

        // Reset state for clean test
        reset_global_state();

        // Setup: No immediate advertising, but conditions for periodic advertising
        BLE_ADVERTISING_ENABLED.set(false);
        MESH_DEVICE_ONLINE_STATUS.set(false);
        MESH_LISTEN_CYCLE_COUNT.set(ADV_INTERVAL2LISTEN_INTERVAL as u32 - 1); // Should trigger advertising

        // Execute function
        handle_mesh_listening_state();

        // Verify listen cycle count was incremented
        assert_eq!(
            MESH_LISTEN_CYCLE_COUNT.get(),
            ADV_INTERVAL2LISTEN_INTERVAL as u32,
            "Listen cycle count should be incremented"
        );

        // Verify advertising was enabled (cycle count % ADV_INTERVAL2LISTEN_INTERVAL != 0)
        assert_eq!(
            BLE_ADVERTISING_ENABLED.get(),
            false, // Should be false when count % interval == 0
            "Advertising state should be determined by interval ratio"
        );

        // In this case, advertising should not be enabled, so state should transition to Adv for online status
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Advertising,
            "State should transition to advertisement for online status"
        );
    }

    /// Tests mesh listening state when continuing to listen.
    ///
    /// Verifies the case where the device continues listening:
    /// - No advertising is scheduled
    /// - No online status is needed
    /// - State remains in listen mode
    /// - Function returns early without state change
    #[test]
    #[mry::lock(rf_stop_trx)]
    #[mry::lock(rf_set_ble_access_code)]
    #[mry::lock(rf_set_ble_crc_adv)]
    #[mry::lock(rf_set_ble_channel)]
    #[mry::lock(rf_set_rxmode)]
    #[mry::lock(read_reg_system_tick)]
    #[mry::lock(write_reg_system_tick_irq)]
    #[mry::lock(read_reg_system_tick_irq)]
    #[mry::lock(read_reg_rnd_number)]
    #[mry::lock(app_bridge_cmd_handle)]
    fn test_handle_mesh_listening_state_continue_listening() {
        // Setup mocks
        mock_rf_stop_trx().returns(());
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_rf_set_ble_channel(Any).returns(());
        mock_rf_set_rxmode().returns(());
        mock_read_reg_system_tick().returns(30000);
        mock_write_reg_system_tick_irq(Any).returns(());
        mock_read_reg_system_tick_irq().returns(50000);
        mock_read_reg_rnd_number().returns(0x1234);
        mock_app_bridge_cmd_handle(Any).returns(());

        // Reset state for clean test
        reset_global_state();

        // Setup: Conditions for continuing to listen
        BLE_ADVERTISING_ENABLED.set(false);
        MESH_DEVICE_ONLINE_STATUS.set(false);
        MESH_LISTEN_CYCLE_COUNT.set(1); // Will not trigger advertising or online status

        // Execute function
        handle_mesh_listening_state();

        // Verify listen cycle count was incremented
        assert_eq!(
            MESH_LISTEN_CYCLE_COUNT.get(),
            2,
            "Listen cycle count should be incremented"
        );

        // Verify advertising and online status determination
        let should_advertise = 2 % ADV_INTERVAL2LISTEN_INTERVAL as u32 != 0;
        let should_online_status = 2 % ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL as u32 != 0;

        assert_eq!(
            BLE_ADVERTISING_ENABLED.get(),
            should_advertise,
            "Advertising state should match interval calculation"
        );
        assert_eq!(
            MESH_DEVICE_ONLINE_STATUS.get(),
            should_online_status,
            "Online status should match interval calculation"
        );

        // If neither advertising nor online status is enabled, state should remain Listen
        if !should_advertise && !should_online_status {
            assert_eq!(
                *CURRENT_RF_STATE.lock(),
                RfOperationState::MeshListening,
                "State should remain Listen when no action is needed"
            );
        } else {
            assert_eq!(
                *CURRENT_RF_STATE.lock(),
                RfOperationState::Advertising,
                "State should transition to Adv when action is needed"
            );
        }
    }

    /// Tests mesh listening state timing calculations.
    ///
    /// Verifies correct timing calculations for various scenarios:
    /// - Listen interval scheduling
    /// - Advertisement timing (7ms)
    /// - Random offset calculations for collision avoidance
    #[test]
    #[mry::lock(rf_stop_trx)]
    #[mry::lock(rf_set_ble_access_code)]
    #[mry::lock(rf_set_ble_crc_adv)]
    #[mry::lock(rf_set_ble_channel)]
    #[mry::lock(rf_set_rxmode)]
    #[mry::lock(read_reg_system_tick)]
    #[mry::lock(write_reg_system_tick_irq)]
    #[mry::lock(read_reg_system_tick_irq)]
    #[mry::lock(read_reg_rnd_number)]
    #[mry::lock(app_bridge_cmd_handle)]
    fn test_handle_mesh_listening_state_timing_calculations() {
        // Setup mocks
        mock_rf_stop_trx().returns(());
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_rf_set_ble_channel(Any).returns(());
        mock_rf_set_rxmode().returns(());
        mock_read_reg_system_tick().returns(15000);
        mock_write_reg_system_tick_irq(Any).returns(());
        mock_read_reg_system_tick_irq().returns(60000);
        mock_read_reg_rnd_number().returns(0x5678);
        mock_app_bridge_cmd_handle(Any).returns(());

        // Reset state for clean test
        reset_global_state();

        // Setup: Custom listen interval
        MESH_LISTEN_INTERVAL_US.set(75000); // 75ms
        BLE_ADVERTISING_ENABLED.set(false);
        MESH_DEVICE_ONLINE_STATUS.set(false);
        MESH_LISTEN_CYCLE_COUNT.set(ADV_INTERVAL2LISTEN_INTERVAL as u32); // Will trigger advertising

        // Execute function
        handle_mesh_listening_state();

        // Since advertising should be enabled after incrementing, verify random timing
        if BLE_ADVERTISING_ENABLED.get() {
            // Random offset calculation: ((tick ^ rnd) & 0x7fff) * CLOCK_SYS_CLOCK_1US + tick_irq
            let random_offset = ((15000 ^ 0x5678) & 0x7fff) * CLOCK_SYS_CLOCK_1US;
            let expected_random_time = random_offset + 60000; // 60000 is the mock value for read_reg_system_tick_irq
            mock_write_reg_system_tick_irq(expected_random_time).assert_called(1);
        }
    }

    /// Tests the "continue listening" logic when no action is needed.
    ///
    /// Verifies that when neither advertising nor online status is scheduled:
    /// - The function returns early (continues listening)
    /// - The state remains as Listen
    /// - The cycle count is still incremented
    /// - No state transitions occur
    #[test]
    #[mry::lock(rf_stop_trx)]
    #[mry::lock(rf_set_ble_access_code)]
    #[mry::lock(rf_set_ble_crc_adv)]
    #[mry::lock(rf_set_ble_channel)]
    #[mry::lock(rf_set_rxmode)]
    #[mry::lock(read_reg_system_tick)]
    #[mry::lock(write_reg_system_tick_irq)]
    #[mry::lock(read_reg_system_tick_irq)]
    #[mry::lock(read_reg_rnd_number)]
    #[mry::lock(app_bridge_cmd_handle)]
    fn test_handle_mesh_listening_state_continue_listening_no_action_needed() {
        // Setup mocks
        mock_rf_stop_trx().returns(());
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_rf_set_ble_channel(Any).returns(());
        mock_rf_set_rxmode().returns(());
        mock_read_reg_system_tick().returns(40000);
        mock_write_reg_system_tick_irq(Any).returns(());
        mock_read_reg_system_tick_irq().returns(70000);
        mock_read_reg_rnd_number().returns(0x9abc);
        mock_app_bridge_cmd_handle(Any).returns(());

        // Reset state for clean test
        reset_global_state();

        // Setup: Conditions where no action is needed
        // Use cycle count that won't trigger advertising or online status
        // ADV_INTERVAL2LISTEN_INTERVAL = 4, ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL = 8
        // So cycle count 7 will not trigger either (7 % 4 != 0, 7 % 8 != 0)
        MESH_LISTEN_CYCLE_COUNT.set(7);
        BLE_ADVERTISING_ENABLED.set(false);
        MESH_DEVICE_ONLINE_STATUS.set(false);

        // Execute function
        handle_mesh_listening_state();

        // Verify listen cycle count was incremented
        assert_eq!(
            MESH_LISTEN_CYCLE_COUNT.get(),
            8,
            "Listen cycle count should be incremented from 7 to 8"
        );

        // Verify that after incrementing, neither advertising nor online status is enabled
        // 8 % 4 = 0, so advertising should be false
        // 8 % 8 = 0, so online status should be false
        assert_eq!(
            BLE_ADVERTISING_ENABLED.get(),
            false,
            "Advertising should not be enabled when cycle count % 4 == 0"
        );
        assert_eq!(
            MESH_DEVICE_ONLINE_STATUS.get(),
            false,
            "Online status should not be enabled when cycle count % 8 == 0"
        );

        // Most importantly: state should remain Listen (no transition to Adv)
        // This verifies the early return logic at line 226
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::MeshListening,
            "State should remain Listen when no action is needed (early return at line 226)"
        );

        // Verify that RF operations were called for setup, even when continuing to listen
        // rf_stop_trx is called twice: once in handle_mesh_listening_state and once in configure_rf_for_mesh_listening
        mock_rf_stop_trx().assert_called(2);
        mock_rf_set_ble_access_code(Any).assert_called(1);
        mock_rf_set_ble_crc_adv().assert_called(1);
        mock_rf_set_ble_channel(Any).assert_called(1);
        mock_rf_set_rxmode().assert_called(1);
    }

    /// Tests the flow when online status is already scheduled (line 203 if statement is false).
    ///
    /// Verifies that when MESH_DEVICE_ONLINE_STATUS is already true:
    /// - The if statement at line 203 is skipped entirely
    /// - No cycle count increment occurs
    /// - No advertising/online status calculations are performed
    /// - The function goes directly to the default transition to advertisement state
    /// - State transitions to RfOperationState::Advertising
    #[test]
    #[mry::lock(rf_stop_trx)]
    #[mry::lock(rf_set_ble_access_code)]
    #[mry::lock(rf_set_ble_crc_adv)]
    #[mry::lock(rf_set_ble_channel)]
    #[mry::lock(rf_set_rxmode)]
    #[mry::lock(read_reg_system_tick)]
    #[mry::lock(write_reg_system_tick_irq)]
    #[mry::lock(read_reg_system_tick_irq)]
    #[mry::lock(read_reg_rnd_number)]
    #[mry::lock(app_bridge_cmd_handle)]
    fn test_handle_mesh_listening_state_online_status_already_scheduled() {
        // Setup mocks
        mock_rf_stop_trx().returns(());
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_rf_set_ble_channel(Any).returns(());
        mock_rf_set_rxmode().returns(());
        mock_read_reg_system_tick().returns(45000);
        mock_write_reg_system_tick_irq(Any).returns(());
        mock_read_reg_system_tick_irq().returns(80000);
        mock_read_reg_rnd_number().returns(0xdef0);
        mock_app_bridge_cmd_handle(Any).returns(());

        // Reset state for clean test
        reset_global_state();

        // Setup: Online status is already scheduled (this is the key difference)
        // This will cause the if statement at line 203 to be false
        MESH_DEVICE_ONLINE_STATUS.set(true);
        BLE_ADVERTISING_ENABLED.set(false);
        MESH_LISTEN_CYCLE_COUNT.set(5); // Arbitrary value

        // Execute function
        handle_mesh_listening_state();

        // Verify that the cycle count was NOT incremented since the if statement was skipped
        assert_eq!(
            MESH_LISTEN_CYCLE_COUNT.get(),
            5,
            "Listen cycle count should NOT be incremented when online status is already scheduled"
        );

        // Verify that advertising and online status flags were NOT modified
        // They should remain as they were set before the function call
        assert_eq!(
            BLE_ADVERTISING_ENABLED.get(),
            false,
            "Advertising flag should remain unchanged when online status is already scheduled"
        );
        assert_eq!(
            MESH_DEVICE_ONLINE_STATUS.get(),
            true,
            "Online status flag should remain unchanged when already scheduled"
        );

        // Most importantly: state should transition to Adv (default path)
        // This verifies that the function skipped the if statement and went to line 230
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Advertising,
            "State should transition to Adv when online status is already scheduled (default path)"
        );

        // Verify that RF operations were still called for setup
        mock_rf_stop_trx().assert_called(2);
        mock_rf_set_ble_access_code(Any).assert_called(1);
        mock_rf_set_ble_crc_adv().assert_called(1);
        mock_rf_set_ble_channel(Any).assert_called(1);
        mock_rf_set_rxmode().assert_called(1);

        // Verify that the listen interval was scheduled (this always happens)
        mock_write_reg_system_tick_irq(45000 + 100000).assert_called(1); // 100000 is default from reset_global_state
    }
}
