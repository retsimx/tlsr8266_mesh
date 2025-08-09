//! # Timing and Synchronization Module
//!
//! This module handles BLE receive state configuration, channel hopping,
//! and timing calculations for the TLSR8266-based mesh lighting system.
//!
//! ## Key Responsibilities:
//! - BLE receive state configuration and timing
//! - Channel hopping and frequency selection
//! - Connection event timing management
//! - RF timing parameter configuration

use core::ptr::addr_of;

use crate::sdk::ble_app::ble_ll_channel_selection::ble_ll_select_next_data_channel;
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::light::IrqHandlerStatus;
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, sleep_us};
use crate::sdk::mcu::register::{*};
use crate::state::{*};

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
    BLE_PERIPHERAL_LINK_STATE.set(crate::sdk::light::BlePeripheralLinkState::Receiving);

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
    super::connection::handle_ble_connection_parameter_updates();
}
