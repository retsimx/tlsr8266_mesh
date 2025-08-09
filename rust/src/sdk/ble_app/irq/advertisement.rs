//! # BLE Advertisement Module
//!
//! This module handles BLE advertisement state management, scan response generation,
//! and advertisement channel cycling for the TLSR8266-based mesh lighting system.
//!
//! ## Key Responsibilities:
//! - BLE advertisement sequence management
//! - Scan response packet generation and transmission
//! - Advertisement channel cycling (37, 38, 39)
//! - State transitions between advertisement and listening modes

use core::ptr::addr_of;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::common::SYS_CHN_ADV;
use crate::sdk::ble_app::light_ll::mesh_send_online_status;
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::light::IrqHandlerStatus;
use crate::sdk::mcu::clock::CLOCK_SYS_CLOCK_1US;
use crate::sdk::mcu::register::{*};
use crate::state::{*};

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
pub fn get_ble_advertisement_channel_count() -> u32
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
    BLE_PERIPHERAL_LINK_STATE.set(crate::sdk::light::BlePeripheralLinkState::Advertising);

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
