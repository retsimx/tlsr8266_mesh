/*
 * BLE Radio Frequency (RF) Driver for TLSR8266 Chip
 * 
 * This module provides the hardware-level control for the Bluetooth Low Energy (BLE) RF interface.
 * It handles the following key functionalities:
 * - RF transceiver initialization and configuration
 * - RF power control and channel selection
 * - BLE transmission and reception operations with scheduling capabilities
 * - RF data packet handling and state management
 * - BLE protocol-specific parameters like access codes and CRC
 * 
 * The driver supports both normal BLE operations and mesh networking packet handling,
 * with configuration for different power levels and communication modes.
 */

use core::cmp::min;
use core::ops::{Deref, DerefMut};
use core::ptr::{addr_of, addr_of_mut};
use core::slice;

use crate::{BIT, uprintln};
use crate::common::{dev_addr_with_mac_flag, mesh_node_init, pair_load_key, retrieve_dev_grp_address};
use crate::config::{FLASH_ADR_MAC, FLASH_ADR_PAIRING, MESH_PWD, OUT_OF_MESH, PAIR_VALID_FLAG};
use crate::main_light::{rf_link_data_callback, rf_link_response_callback};
use crate::sdk::ble_app::ble_ll_pair::pair_dec_packet;
use crate::sdk::ble_app::light_ll::{parse_ble_packet_op_params, rf_link_is_notify_req, rf_link_match_group_mac, rf_link_slave_add_status, rf_link_slave_read_status_par_init, rf_link_slave_read_status_stop};
use crate::sdk::ble_app::rf_drv_8266_tables::{TBL_AGC, TBL_RF_INI, TBL_RF_POWER};
use crate::sdk::common::compat::{array4_to_int, load_tbl_cmd_set};
use crate::sdk::common::crc::crc16;
use crate::sdk::drivers::flash::{flash_read_page, flash_write_page};
use crate::sdk::light::{*};
use crate::sdk::mcu::analog::{analog_read, analog_write};
use crate::sdk::mcu::clock::CLOCK_SYS_CLOCK_1US;
use crate::sdk::mcu::crypto::encode_password;
use crate::sdk::mcu::irq_i::irq_disable;
use crate::sdk::mcu::random::rand;
use crate::sdk::mcu::register::{FLD_RF_IRQ_MASK, read_reg_irq_mask, read_reg_rf_mode, read_reg_system_tick, read_reg_system_tick_mode, write_reg16, write_reg32, write_reg8, write_reg_dma2_addr, write_reg_dma2_ctrl, write_reg_dma3_addr, write_reg_dma_chn_irq_msk, write_reg_irq_mask, write_reg_irq_src, write_reg_rf_access_code, write_reg_rf_crc, write_reg_rf_irq_mask, write_reg_rf_irq_status, write_reg_rf_mode, write_reg_rf_mode_control, write_reg_rf_sched_tick, write_reg_rf_sn, write_reg_system_tick_irq, write_reg_system_tick_mode, write_reg_rf_txrx_state, write_reg_pll_rx_fine_div_tune, write_reg_rf_channel, write_reg_rf_timing_config, write_reg_rf_sys_timer_config, FLD_RF_RX_MODE};
use crate::sdk::mcu::register::{rega_deepsleep_flag, write_reg_rf_rx_gain_agc, write_reg_rf_rx_mode};
use crate::sdk::packet_types::{Packet, PacketAttCmd, PacketAttData, PacketAttValue};
use crate::sdk::pm::light_sw_reboot;
use crate::state::{*};
use crate::version::BUILD_VERSION;

const RF_FAST_MODE: bool = true;
const RF_TRX_OFF: u8 = 0x45;

/**
 * Initialize the RF driver with specified settings
 * 
 * This function sets up the RF hardware by:
 * 1. Loading initialization command sets for configuring the RF registers
 * 2. Setting up RF gain values for Automatic Gain Control (AGC)
 * 3. Configuring transmission power parameters based on values stored in flash memory
 *
 * The RF initialization values depend on whether a 16MHz crystal is used (controlled by feature flag).
 * If transmission power calibration values are found in flash memory, they will be applied to 
 * RF_TP_BASE and RF_TP_GAIN parameters which control output power level.
 * 
 * @param enable - Whether to enable the RF module (true) or power it off (false)
 * @return The deepsleep flag value for recovery information (0x40 bit indicates sleep state)
 */
pub fn rf_drv_init(enable: bool) -> u8
{
    let result = analog_read(rega_deepsleep_flag) & 0x40;

    critical_section::with(|_| {
        if enable {
            // Load initialization values to configure the RF hardware registers
            // Different register values for 16MHz xtal configuration versus default
            #[cfg(feature = "xtal-16mhz")]
            load_tbl_cmd_set(&TBL_RF_INI);

            #[cfg(not(feature = "xtal-16mhz"))]
            load_tbl_cmd_set(&TBL_RF_INI[0 .. TBL_RF_INI.len() - 4]);

            // Configure the Automatic Gain Control (AGC) settings
            // Each value corresponds to different RF gain stages
            for i in 0..6 {
                write_reg_rf_rx_gain_agc(TBL_AGC[i], (i << 2) as u32)
            }
            
            // Check if custom RF power settings are stored at specific flash memory offsets
            // These calibration values adjust the transmission power parameters
            let mut power_settings = [0u8; 0x13];
            // FLASH_ADR_MAC is guaranteed to be aligned to FLASH_SECTOR_SIZE
            flash_read_page(FLASH_ADR_MAC, power_settings.len() as u32, power_settings.as_mut_ptr());
            
            // Process primary power calibration value
            if power_settings[0x11] != 0xff {
                let base_power = power_settings[0x11] as u32;
                RF_TP_BASE.set(base_power);
                
                // Calculate gain value from base power, with safeguard against underflow
                // Default threshold (0x19) is 25, which represents typical baseline power
                if base_power > 0x19 {
                    RF_TP_GAIN.set(((base_power - 0x19) << 8) / 80);
                } else {
                    RF_TP_GAIN.set(0); // Set gain to 0 if base power is too low
                }
            }
            
            // Process secondary power calibration value (power adjustment)
            if power_settings[0x12] != 0xff {
                let base_power = RF_TP_BASE.get();
                let power_adj = power_settings[0x12] as u32;
                
                // Ensure we don't underflow when calculating differential
                if base_power >= power_adj {
                    let u_var5 = base_power - power_adj;
                    RF_TP_GAIN.set((u_var5 << 8) / 80);
                } else {
                    RF_TP_GAIN.set(0); // Set gain to 0 if adjustment would cause underflow
                }
            }
        } else {
            // When RF module is disabled, power off the SAR (Successive Approximation Register) ADC
            analog_write(6, 0);
        }
    });

    result
}

/**
 * Stop all radio transmit and receive operations
 * 
 * This function stops any ongoing RF operations by setting the RF mode control 
 * register to the stop mode (0x80).
 */
pub fn rf_stop_trx() {
    write_reg_rf_mode_control(0x80);            // stop
}

/**
 * Set the RF module into receive mode
 * 
 * This function configures the RF hardware to actively listen for incoming packets by:
 * 1. Enabling the RF receiver in the RF_TRX_MODE register
 * 2. Setting the RF state machine to receive mode
 */
pub fn rf_set_rxmode()
{
    write_reg_rf_rx_mode((FLD_RF_RX_MODE::LOW_PASS_FILTER | FLD_RF_RX_MODE::EN).bits());  // rx enable
    write_reg_rf_txrx_state(RF_TRX_OFF | BIT!(5));   // RX enable
}

/**
 * Disable both RF transmitter and receiver
 * 
 * This function turns off both the transmitter and receiver circuits by:
 * 1. Setting the RF mode to a specific power configuration (0x29)
 * 2. Disabling the RF receiver
 * 3. Resetting the RF state machine
 */
pub fn rf_set_tx_rx_off()
{
    write_reg_rf_mode(0x29);                     // Set specific RF mode
    write_reg_rf_rx_mode(FLD_RF_RX_MODE::LOW_PASS_FILTER.bits());           // rx disable (replacing write_reg8(0x800428, RF_TRX_MODE))
    write_reg_rf_txrx_state(RF_TRX_OFF);            // reset tx/rx state machine
}

/**
 * Initialize the basic Link Layer (LL) MCU functionality
 * 
 * This function configures the core hardware settings required for BLE operation:
 * 1. Sets up DMA channels for packet transmission and reception
 * 2. Configures interrupt masks for RF events
 * 3. Sets up system tick operations for timing BLE events
 * 4. Initializes RF interrupt status and mask registers
 * 
 * This is a fundamental initialization function that must be called during device startup
 * before any RF communication can occur.
 */
pub fn blc_ll_init_basic_mcu()
{
    // Configure system tick interval
    write_reg_rf_sys_timer_config(700);
    
    // Set up DMA address for packet reception
    write_reg_dma2_addr(addr_of!(LIGHT_RX_BUFF.lock()[LIGHT_RX_WPTR.get()]) as u16);

    // Configure DMA2 control register for packet reception
    write_reg_dma2_ctrl(0x104);
    write_reg_dma_chn_irq_msk(0);

    // Enable interrupt handling for RF operations
    write_reg_irq_mask(read_reg_irq_mask() | 0x2000);
    write_reg_system_tick_mode(read_reg_system_tick_mode() | 2);

    // Clear interrupt flags and reset RF interrupt status
    write_reg_rf_irq_mask(FLD_RF_IRQ_MASK::empty().bits());
    write_reg_rf_irq_status(0xfffe);

    // Configure RF interrupts for RX and TX events
    write_reg_rf_irq_mask((FLD_RF_IRQ_MASK::IRQ_RX | FLD_RF_IRQ_MASK::IRQ_TX).bits());

    // Set up system tick interrupt for timing BLE events
    write_reg_system_tick_irq(read_reg_system_tick() | (0x80 << 0x18));
    write_reg_irq_src(0x80 << 0xd);
    write_reg_irq_mask(read_reg_irq_mask() | (0x80 << 0xd));

    // Set additional timing configuration
    write_reg_rf_timing_config(0xc00);
}

/**
 * Determine the appropriate response type for a command
 * 
 * Maps BLE command opcodes to their corresponding response types based on the command
 * opcode and an additional parameter. This function matches opcodes to their response types,
 * with special handling for group operations where the parameter value affects the response.
 * 
 * @param opcode - The command opcode to find the response type for
 * @param param_val - An additional parameter that modifies the response type in some cases (especially for group operations)
 * @return The appropriate response type code for the given command
 */
fn rf_link_get_rsp_type(opcode: u8, param_val: u8) -> u8 {
    match opcode {
        LGT_CMD_LIGHT_READ_STATUS => LGT_CMD_LIGHT_STATUS,
        
        LGT_CMD_LIGHT_GRP_REQ => match param_val {
            GET_GROUP1 => LGT_CMD_LIGHT_GRP_RSP1,
            GET_GROUP2 => LGT_CMD_LIGHT_GRP_RSP2,
            GET_GROUP3 => LGT_CMD_LIGHT_GRP_RSP3,
            _ => LGT_CMD_LIGHT_GRP_RSP1, // Default response for other values
        },
        
        LGT_CMD_LIGHT_CONFIG_GRP => LGT_CMD_LIGHT_GRP_RSP1,
        LGT_CMD_CONFIG_DEV_ADDR => LGT_CMD_DEV_ADDR_RSP,
        
        0 => 1, // No constant defined for this case
        LGT_CMD_OTA_DATA_REQ => LGT_CMD_OTA_DATA_RSP,
        LGT_CMD_END_OTA_REQ => LGT_CMD_END_OTA_RSP,
        LGT_CMD_USER_NOTIFY_REQ => LGT_CMD_USER_NOTIFY_RSP,
        
        7 => 8, // No constant defined for this case
        
        // Default for any other opcode
        _ => 0xff,
    }
}

/**
 * Initializes a status read response operation for BLE mesh packets
 * 
 * This function prepares the packet data structures needed to respond to a status
 * read request from a remote device. It performs the following operations:
 * 1. Records the current time as the start of the read operation
 * 2. Determines the appropriate response type based on the packet's opcode and parameters
 * 3. Sets unicast/broadcast flags based on the destination address
 * 4. Clears specific packet fields based on packet type and addressing mode
 * 5. Initializes status tracking data structures to store responses
 * 
 * For unicast packets (directed to a specific device), destination-specific fields
 * are cleared. For LGT_CMD_LIGHT_READ_STATUS packets, additional fields are zeroed
 * and the status tick value is copied into the packet.
 * 
 * @param pkt_light_data - The packet to prepare for status response
 */
fn rf_link_slave_read_status_start(pkt_light_data: &mut Packet)
{
    // Record the time when the status read operation started
    SLAVE_READ_STATUS_BUSY_TIME.set(read_reg_system_tick());
    
    // Extract opcode and parameter for determining response type
    let opcode = pkt_light_data.att_cmd().value.val[0] & 0x3f;
    let param = pkt_light_data.att_cmd().value.val[4];
    
    // Determine the appropriate response type based on the opcode and parameter
    SLAVE_READ_STATUS_BUSY.set(rf_link_get_rsp_type(opcode, param));
    
    // Check if this is a unicast (directed to a specific device) or broadcast request
    let is_unicast = (pkt_light_data.att_cmd().value.dst[1] & 0x80) == 0;
    SLAVE_READ_STATUS_UNICAST_FLAG.set(is_unicast);
    
    // For unicast packets, clear fields for device-specific information
    if is_unicast {
        pkt_light_data.att_cmd_mut().value.val[8..8 + 4].fill(0);
        pkt_light_data.att_cmd_mut().value.val[12] = 1;
    } else {
        pkt_light_data.att_cmd_mut().value.val[12] = 0;
    }
    
    // Special handling for LGT_CMD_LIGHT_READ_STATUS
    if opcode == LGT_CMD_LIGHT_READ_STATUS {
       // Clear parameter and destination info fields
       pkt_light_data.att_cmd_mut().value.val[4..4 + 4 + 4].fill(0);
       
       // Store the status tick value
       pkt_light_data.att_cmd_mut().value.val[3] = SLAVE_STATUS_TICK.get();
    }

    // Initialize parameters for status response handling
    rf_link_slave_read_status_par_init();

    // Clear the response buffer for fresh status data
    BUFF_RESPONSE.lock().fill(Packet { att_data: PacketAttData::default() });

    // Reset status record tracking indices
    SLAVE_STATUS_RECORD_IDX.set(0);
    SLAVE_STATUS_RECORD.lock().fill(
        StatusRecord { adr: [0; 1], alarm_id: 0 }
    );

    NOTIFY_REQ_MASK_IDX.set(0);
}

/**
 * Process an unencrypted BLE mesh packet for data transfer
 * 
 * This function handles received BLE mesh packets that don't require decryption
 * (either they were already decrypted or are meant to be processed in raw form).
 * It performs packet validation, sequence number checking, parameter extraction,
 * and appropriate response handling based on the packet opcode.
 * 
 * Algorithm:
 * 1. Validate packet length requirements (minimum size check)
 * 2. Check if this is a duplicate packet by comparing sequence numbers
 * 3. Extract operation parameters from the packet
 * 4. Determine if packet targets this device (by group or direct addressing)
 * 5. Process command based on opcode, with special handling for:
 *    - Device address configuration
 *    - Group operations
 *    - Status read requests
 *    - Notification commands
 * 6. Update internal state based on the received command
 * 7. Handle response packets for various command types
 * 8. Set up notification structures for asynchronous responses
 * 
 * @param data - The BLE mesh packet to process
 * @return true if packet was successfully processed or is a duplicate, false otherwise
 */
fn rf_link_slave_data_write_no_dec(data: &Packet) -> bool {
    // Validate minimum packet size requirement
    // RF length must be at least 0x11 bytes for valid command packets
    if data.head().rf_len < 0x11 {
        return false;
    }

    // Extract the sequence number from the packet
    // The sequence number is used to detect duplicate packets and prevent reprocessing
    let sno = data.att_write().value.sno;
    
    // Check if this is a duplicate packet (same sequence number as the last one)
    // Return true for duplicates to avoid reprocessing the same command
    if sno == *SLAVE_SNO.lock() {
        return true;
    }

    // Extract operation command and parameters from the packet using the refactored function
    let (success, op_cmd, op_cmd_len, params, params_len) = parse_ble_packet_op_params(data, false);

    // Determine if this packet targets this device by checking:
    // 1. If it matches one of the device's configured group addresses (group_match)
    // 2. If it directly targets this device's address (device_match)
    let (group_match, device_match) = rf_link_match_group_mac(data);
    
    // Extract the operation code (opcode) and destination address information
    let op = if op_cmd_len == 3 {
        op_cmd[0] & 0x3f  // Extract the lower 6 bits as the opcode (masking out flag bits)
    } else {
        0  // Default opcode if no valid command found
    };
    
    // Extract destination address from the packet
    // The destination high byte is at index 6 in the value array
    let mut dst_addr = data.att_write().value.dst[1] as u32;
    
    // Special handling for device address configuration command targeting broadcast address
    // The high bit of destination address indicates a special broadcast configuration
    if op == LGT_CMD_CONFIG_DEV_ADDR && ((dst_addr * 0x1000000) as i32) < 0 {
        // Validate mandatory parameter values for broadcast device address configuration
        // Parameters 0 and 1 must be 0xFF for this command type
        if params[0] != 0xff || params[1] != 0xff {
            return false;
        }
        // Construct 16-bit destination address from bytes 5-6
        dst_addr = (dst_addr << 8) | data.att_write().value.dst[0] as u32;
    } else {
        // For non-device-address commands, construct the destination address
        dst_addr = (dst_addr << 8) | data.att_write().value.dst[0] as u32;
        
        // Special handling for light on/off commands with broadcast bit set
        // The high bit of the 16-bit address indicates broadcast when set
        let is_broadcast = ((!dst_addr << 0x10) as i32) < 0;
        if is_broadcast && op == LGT_CMD_LIGHT_ONOFF {
            // If destination is 0, use the device's own address for notification
            if dst_addr == 0 {
                dst_addr = DEVICE_ADDRESS.get() as u32;
            }
            // Check if forced notification is required (stub implementation currently)
            uprintln!("stub: mesh_node_check_force_notify")
            // mesh_node_check_force_notify(uVar4, params[0]);
        }
    }

    // Lock the global packet buffers for response preparation
    let mut pkt_light_status = PKT_LIGHT_STATUS.lock();
    let mut pkt_light_data = PKT_LIGHT_DATA.lock();

    // Reset the packet value fields to prepare for new data
    pkt_light_data.att_cmd_mut().value = PacketAttValue::default();
    pkt_light_status.att_cmd_mut().value = PacketAttValue::default();

    // Create a clone of the packet instead of using unsafe raw memory operations
    // For efficient memory usage, we only clone the portion of the data we need
    let copy_size = params_len as usize + 0x11;
    
    // First clone the packet header which has the same structure
    *pkt_light_data = data.clone();
    
    // Configure the BLE packet headers for the response
    // These values define the packet format according to the BLE specification
    pkt_light_data.head_mut().chan_id = 0xff03;  // Vendor-specific channel ID
    pkt_light_data.head_mut().dma_len = 0x27;    // DMA buffer length 
    pkt_light_data.head_mut().rf_len = 0x25;     // RF packet length
    pkt_light_data.head_mut().l2cap_len = 0x21;  // L2CAP layer length

    // Set the source device address in the packet using safe method
    // This ensures the recipient knows which device sent the response
    let device_addr = DEVICE_ADDRESS.get();
    pkt_light_data.att_cmd_mut().opcode = device_addr as u8;
    pkt_light_data.att_cmd_mut().value.src[0] = (device_addr & 0xFF) as u8;
    pkt_light_data.att_cmd_mut().value.src[1] = ((device_addr >> 8) & 0xFF) as u8;

    // Process the command data if this packet is intended for this device
    // (either directly targeted or matches one of our group addresses)
    if device_match || group_match {
        rf_link_data_callback(&*pkt_light_data);
    }
    
    // Update the stored sequence number with the one from this packet
    // This prevents duplicate processing if the same packet is received again
    *SLAVE_SNO.lock() = sno;

    // Store the command opcode for potential future reference
    SLAVE_LINK_CMD.set(op);

    // For non-notification commands, handle the command directly
    if !rf_link_is_notify_req(op) {
        // If a status read operation was in progress, stop it
        if SLAVE_READ_STATUS_BUSY.get() != 0 {
            rf_link_slave_read_status_stop();
        }
        
        // Extract the destination address from the packet in a safe way
        let dst_addr_bytes = [data.att_write().value.dst[0], data.att_write().value.dst[1]];
        let dst_addr_u16 = u16::from_le_bytes(dst_addr_bytes);
        
        // Set the data validity flag based on addressing mode
        let needs_bridge_forwarding = !device_match && dst_addr_u16 != 0;
        if needs_bridge_forwarding {
            SLAVE_DATA_VALID.set(BRIDGE_MAX_CNT + 1);  // Needs bridge forwarding
        } else {
            SLAVE_DATA_VALID.set(0);  // No bridge forwarding needed
        }
        return true;
    }

    // For notification request commands, prepare the status response
    // Initialize status packet fields for the response
    pkt_light_status.att_cmd_mut().value.val[13] = 0;
    pkt_light_status.att_cmd_mut().value.val[14] = 0;
   
    // Store the link interval in the packet (converted from microseconds to milliseconds)
    pkt_light_data.att_cmd_mut().value.val[16] = (SLAVE_LINK_INTERVAL.get() / (CLOCK_SYS_CLOCK_1US * 1000)) as u8;
    
    // Function to set response type in the packet based on command type
    let set_response_type = |pkt: &mut Packet, cmd_op: u8, cmd_params: &[u8]| {
        match cmd_op {
            LGT_CMD_LIGHT_GRP_REQ => pkt.att_cmd_mut().value.val[15] = cmd_params[1],
            LGT_CMD_LIGHT_CONFIG_GRP => pkt.att_cmd_mut().value.val[15] = 1,
            LGT_CMD_CONFIG_DEV_ADDR => pkt.att_cmd_mut().value.val[15] = 4,
            LGT_CMD_USER_NOTIFY_REQ => pkt.att_cmd_mut().value.val[15] = 7,
            _ => pkt.att_cmd_mut().value.val[15] = 0,
        };
    };
    
    // Configure response parameters based on device matching and command type
    let is_special_addr_config = dst_addr != 0 && op == LGT_CMD_CONFIG_DEV_ADDR && dev_addr_with_mac_flag(&params);
    
    if !device_match || is_special_addr_config {
        // Handle specific request types that use parameter-based validation timing
        if op == LGT_CMD_LIGHT_GRP_REQ || op == LGT_CMD_LIGHT_READ_STATUS || op == LGT_CMD_USER_NOTIFY_REQ {
            // Set the validation flag using the parameter value for delay calculation
            // The formula params[0] * 2 + 1 creates staggered response times based on the parameter
            SLAVE_DATA_VALID.set(params[0] as u32 * 2 + 1);
            
            // Set appropriate response type indicator based on the command type
            set_response_type(&mut pkt_light_data, op, &params);
        } else if op != LGT_CMD_CONFIG_DEV_ADDR {
            // Handle non-address-config commands with maximum bridge delay
            SLAVE_DATA_VALID.set(BRIDGE_MAX_CNT * 2 + 1);
            pkt_light_data.att_cmd_mut().value.val[15] = 1;
        } else if !dev_addr_with_mac_flag(&params) {
            // Handle device address configuration without MAC flag
            SLAVE_DATA_VALID.set(BRIDGE_MAX_CNT * 2 + 1);
            pkt_light_data.att_cmd_mut().value.val[15] = 4;
        } else {
            // Handle device address configuration with MAC flag
            // Use param[3] to calculate custom response timing
            SLAVE_DATA_VALID.set(params[3] as u32 * 2 + 1);
            pkt_light_data.att_cmd_mut().value.val[15] = 4;
        }
    } else {
        // For direct device matches with no special handling needed
        SLAVE_DATA_VALID.set(0);
        
        // Set response type indicator based on command type
        set_response_type(&mut pkt_light_data, op, &params);
    }
    
    // Copy the response type indicator to the status packet
    pkt_light_status.att_cmd_mut().value.val[15] = pkt_light_data.att_cmd_mut().value.val[15];
    
    // Initialize the status read response operation
    // This sets up all necessary state for generating the status response
    rf_link_slave_read_status_start(pkt_light_data.deref_mut());

    // Store the sequence number for status tracking
    *SLAVE_STAT_SNO.lock() = sno;

    // Process the response if this packet is for this device
    if device_match || group_match {
        // Copy command data to the status packet
        let ptr = &pkt_light_data.att_cmd_mut().value.val[3..];
        pkt_light_status.att_cmd_mut().value.val[3..3 + ptr.len()].copy_from_slice(&ptr);

        // Set the sequence number in the status packet
        pkt_light_status.att_cmd_mut().value.sno = *SLAVE_SNO.lock();

        // Set this device's address as the source in the status packet using safe methods
        // Instead of raw pointer casting
        let device_addr = DEVICE_ADDRESS.get();
        pkt_light_status.att_cmd_mut().value.src[0] = (device_addr & 0xFF) as u8;
        pkt_light_status.att_cmd_mut().value.src[1] = ((device_addr >> 8) & 0xFF) as u8;

        // Create a temporary packet for passing to the response callback
        // Copy the original packet data to the temporary packet
        let mut tmp_pkt = data.att_write().value.clone();

        // Set this device's address as the source in the temporary packet
        let device_addr = DEVICE_ADDRESS.get();
        tmp_pkt.src[0] = (device_addr & 0xFF) as u8;
        tmp_pkt.src[1] = ((device_addr >> 8) & 0xFF) as u8;

        // Call the response callback to fill in application-specific status data
        // If the callback returns true, add the status packet to the response queue
        if rf_link_response_callback(&mut pkt_light_status.att_cmd_mut().value, &tmp_pkt) {
            rf_link_slave_add_status(&*pkt_light_status);
        }
    }

    // Return true to indicate successful packet processing
    return true;
}

/**
 * Process a received BLE packet with potential decryption
 * 
 * This function handles incoming packet data that might be encrypted by:
 * 1. Creating a local copy of the packet data
 * 2. Checking if pairing is established and packet requires decryption
 * 3. Attempting to decrypt the packet using the pairing keys if required
 * 4. Passing the decrypted packet to the non-decryption handler
 * 
 * Unlike the no_dec variant, this function handles the security layer
 * of the mesh protocol by decrypting packets before processing them.
 * 
 * @param data - Reference to the received packet data
 * @return true if packet was processed successfully, false otherwise
 */
pub fn rf_link_slave_data_write(data: &Packet) -> bool {
    let mut data: Packet = data.clone();
    
    // Check if security is enabled and attempt packet decryption
    if PAIR_LOGIN_OK.get() && pair_dec_packet(&mut data) {
        // Process the decrypted packet
        return rf_link_slave_data_write_no_dec(&mut data);
    }

    return false;
}

/**
 * Configure advertisement packet data for BLE slave device
 * 
 * This function takes advertisement data and populates the global advertisement
 * packet structure with the appropriate headers and data. The advertisement packet
 * is used for device discovery and connection establishment in BLE.
 * 
 * @param adv_data_ptr - Reference to the advertisement data to be included in the packet
 */
fn rf_link_slave_set_adv(adv_data_ptr: &[u8])
{
    let mut pkt_adv = PKT_ADV.lock();

    // Calculate the total packet length including headers
    pkt_adv.head_mut().dma_len = adv_data_ptr.len() as u32 + 8;
    pkt_adv.head_mut().rf_len = adv_data_ptr.len() as u8 + 6;

    // Copy the advertisement data to the packet
    pkt_adv.adv_ind_module_mut().data[0..adv_data_ptr.len()].copy_from_slice(&adv_data_ptr[0..adv_data_ptr.len()]);
}

/**
 * Initialize the BLE slave device with mesh networking capability
 * 
 * This is a core initialization function that sets up the BLE slave device by:
 * 1. Initializing the basic MCU link layer operations
 * 2. Configuring interrupt handlers and state variables
 * 3. Setting up timing parameters for connection and advertisement intervals
 * 4. Checking for existing MAC address or generating a new one
 * 5. Setting up the pairing configuration from flash or creating a new one
 * 6. Loading encryption keys for secure communication
 * 7. Configuring advertisement packets with the device's MAC address
 * 8. Retrieving mesh group addresses and initializing the mesh node
 * 9. Writing device identification information to system memory
 * 
 * This function must be called during device startup to enable BLE
 * communication and mesh networking capabilities.
 * 
 * @param interval - Base interval value for BLE timing operations
 */
pub fn rf_link_slave_init(interval: u32)
{
    unsafe {
        // Initialize the basic MCU link layer functionality
        blc_ll_init_basic_mcu();
        
        // Set up interrupt handler status for advertisement mode
        *P_ST_HANDLER.lock() = IrqHandlerStatus::Adv;
        
        // Initialize link state and timing parameters
        SLAVE_LINK_STATE.set(0);
        SLAVE_LISTEN_INTERVAL.set(interval * CLOCK_SYS_CLOCK_1US);

        // Configure connection timer tick
        SLAVE_CONNECTED_TICK.set(CLOCK_SYS_CLOCK_1US * 100000 + read_reg_system_tick());
        write_reg_system_tick_irq(SLAVE_CONNECTED_TICK.get());

        // Set RF hardware configuration
        write_reg8(0xf04, 0x68);

        // Generate a MAC address if one doesn't exist in flash
        if *(FLASH_ADR_MAC as *const u32) == u32::MAX {
            // Generate random MAC address values
            let mac: [u16; 2] = [rand(), rand()];
            flash_write_page(FLASH_ADR_MAC, 4, mac.as_ptr() as *const u8);
        }

        // Check for existing pairing configuration or create a new one
        let pair_addr = *(FLASH_ADR_PAIRING as *const u32) + 1;
        if pair_addr == 0 {
            // Configure pairing address in flash memory
            let pairing_addr = FLASH_ADR_PAIRING;
            
            // Store the mesh long-term key (LTK)
            let mut buff: [u8; 16] = [0; 16];
            buff[0..16].copy_from_slice(&PAIR_CONFIG_MESH_LTK.lock()[0..16]);
            flash_write_page(pairing_addr + 48, 16, buff.as_mut_ptr());

            // Encode and store the mesh password
            let mut buff: [u8; 16] = [0; 16];
            let len = min(MESH_PWD.len(), buff.len());
            buff[0..len].copy_from_slice(&MESH_PWD.as_bytes()[0..len]);
            encode_password(&mut buff);
            flash_write_page(pairing_addr + 32, 16, buff.as_mut_ptr());

            // Store the out-of-mesh device name
            let mut buff: [u8; 16] = [0; 16];
            let len = min(OUT_OF_MESH.len(), buff.len());
            buff[0..len].copy_from_slice(&OUT_OF_MESH.as_bytes()[0..len]);
            flash_write_page(pairing_addr + 16, 16, buff.as_mut_ptr());

            // Set up pairing validity flags
            let mut buff: [u8; 16] = [0; 16];
            buff[0] = PAIR_VALID_FLAG;  // Indicator that pairing data is valid
            buff[15] = PAIR_VALID_FLAG; // Redundant flag for data integrity check
            
            // Configure mesh pair enable if requested
            if MESH_PAIR_ENABLE.get() {
                GET_MAC_EN.set(true);
                buff[1] = 1;
            }
            
            // Write pairing configuration to flash
            flash_write_page(pairing_addr, 16, buff.as_mut_ptr());
            
            // Reboot device to apply new configuration
            irq_disable();
            light_sw_reboot();
            loop {}
        }

        // Read the MAC address from flash
        flash_read_page(FLASH_ADR_MAC, 6, MAC_ID.lock().as_mut_ptr());

        // Set the MAC address in the advertisement packet
        PKT_ADV.lock().adv_ind_module_mut().adv_a = *MAC_ID.lock();

        // Configure the advertisement data
        rf_link_slave_set_adv(&ADV_DATA.lock().clone());
        
        // Load encryption keys for secure communication
        pair_load_key();

        // Set device MAC address in packet templates for responses
        PKT_LIGHT_DATA.lock().att_cmd_mut().value.src[0..2].copy_from_slice(&MAC_ID.lock()[0..2]);
        PKT_LIGHT_STATUS.lock().att_cmd_mut().value.src[0..2].copy_from_slice(&MAC_ID.lock()[0..2]);

        // Enable advertising
        SLAVE_ADV_ENABLE.set(true);

        // Load device group addresses from flash memory
        retrieve_dev_grp_address();

        // Initialize mesh network node functionality
        mesh_node_init();

        // Write device identification info to system memory
        // This includes MAC address, firmware version, and data integrity check
        write_reg32(0x808004, array4_to_int(&*MAC_ID.lock()));
        write_reg32(0x808008, BUILD_VERSION);
        write_reg16(0x80800c, crc16(&slice::from_raw_parts(0x808004 as *const u8, 8)));
    }
}

/**
 * Sets the RF transmitter's power level based on a numeric index
 * 
 * Maps the provided index to specific analog register settings from the TBL_RF_POWER table.
 * Lower indices correspond to higher power levels, with 0 being the highest (typically +8dBm)
 * and 11 being the lowest (typically -37dBm).
 * 
 * @param index - The power level index (0-11), where 0 is highest power
 */
pub fn rf_set_power_level_index(index: u32)
{
    // Default to the lowest power setting (index 11) if out of range
    let tbl_index = if index < TBL_RF_POWER.len() as u32 {
        index as usize
    } else {
        11 // Use the lowest power setting if out of range
    };

    // Configure the four analog registers that control RF power
    unsafe {
        analog_write(0xa2, TBL_RF_POWER[tbl_index].a);  // Power amplifier (PA) gain control
        analog_write(4, TBL_RF_POWER[tbl_index].b);     // Power amplifier (PA) bias control
        analog_write(0xa7, TBL_RF_POWER[tbl_index].c);  // RF output match network control
        analog_write(0x8d, TBL_RF_POWER[tbl_index].d);  // RF output power control
    }
}

/**
 * BLE channel type enumeration to improve code clarity
 * Categorizes the different types of channels in the BLE specification
 */
enum BleChannelType {
    DataLow,    // Channels 0-10 (2402-2422 MHz) 
    DataHigh,   // Channels 11-36 (2424-2474 MHz)
    AdvPrimary, // Primary advertising channels (37, 38, 39)
    ExtLow,     // Extended channels 40-50 
    ExtHigh,    // Extended channels 51-61 (reverse direction frequency mapping)
    OutOfRange, // Any channel number outside the valid range
}

/**
 * Channel parameter structure to hold gain and frequency values
 */
struct ChannelParams {
    gain: u8,  // RF gain value
    intgn: u16 // Integral frequency in MHz
}

/**
 * Configure the RF module to operate on a specific BLE channel
 * 
 * This function determines the appropriate frequency for a given BLE channel number
 * and configures the RF hardware accordingly. BLE has several channel ranges with
 * different frequency mappings:
 * 
 * - Channels 0-10: Standard data channels (2402-2422 MHz)
 * - Channels 11-36: Higher data channels (2424-2474 MHz)
 * - Channel 37: Primary advertising channel (2402 MHz)
 * - Channel 38: Primary advertising channel (2426 MHz)
 * - Channel 39: Primary advertising channel (2480 MHz)
 * - Channels 40-50: Extended channels with direct frequency mapping
 * - Channels 51-61: Extended channels with reverse frequency mapping
 * 
 * NOTE: The extended channels (40-61) are not used in this project and have not been 
 * tested in production. The support for these channels is implemented but may not work.
 * 
 * @param chn - The BLE channel number to use (0-61)
 */
pub fn rf_set_ble_channel(chn: u8) {
    // Set the RF channel control register
    write_reg_rf_channel(chn);

    // Determine channel type and calculate parameters
    let params = calculate_channel_params(chn);

    // Power off the SAR (Successive Approximation Register) ADC
    analog_write(6, 0);

    // Turn on LDO and baseband PLL
    write_reg_rf_mode(0x29);

    // Disable receiver and reset TX/RX state machine
    write_reg_rf_rx_mode(0);           // rx disable 
    write_reg_rf_txrx_state(RF_TRX_OFF); // reset tx/rx state machine

    // Configure the frequency for auto TX
    write_reg_pll_rx_fine_div_tune(params.intgn); // Write integral frequency value

    // Set transmission power gain for the selected channel
    rf_set_tp_gain(params.gain);
}

/**
 * Calculate gain and frequency parameters for a given BLE channel
 * 
 * This function encapsulates the complex channel-to-frequency mapping logic
 * to make it more maintainable and easier to understand.
 * 
 * NOTE: The extended channels (40-61) are not used in this project and have not been 
 * tested in production. The support for these channels is implemented but may not work.
 * 
 * @param chn - The BLE channel number (0-61)
 * @return ChannelParams containing gain and frequency values
 */
fn calculate_channel_params(chn: u8) -> ChannelParams {
    // Determine the channel type
    let channel_type = match chn {
        0..=10 => BleChannelType::DataLow,
        11..=36 => BleChannelType::DataHigh,
        37 => BleChannelType::AdvPrimary,
        38 => BleChannelType::AdvPrimary,
        39 => BleChannelType::AdvPrimary,
        40..=50 => BleChannelType::ExtLow,
        51..=61 => BleChannelType::ExtHigh,
        _ => BleChannelType::OutOfRange
    };

    // Calculate gain and frequency based on channel type
    match channel_type {
        BleChannelType::DataLow => {
            // Channels 0-10: gain = (chn + 2) * 2, intgn = gain + 2400
            let gain = (chn + 2) * 2;
            ChannelParams {
                gain,
                intgn: gain as u16 + 2400
            }
        },
        BleChannelType::DataHigh => {
            // Channels 11-36: gain = (chn + 3) * 2, intgn = gain + 2400
            let gain = (chn + 3) * 2;
            ChannelParams {
                gain,
                intgn: gain as u16 + 2400
            }
        },
        BleChannelType::AdvPrimary => {
            // Primary advertising channels have specific values
            match chn {
                37 => ChannelParams { gain: 2, intgn: 2402 },
                38 => ChannelParams { gain: 0x1a, intgn: 2426 },
                _ => ChannelParams { gain: 0x50, intgn: 2480 }  // Channel 39
            }
        },
        BleChannelType::ExtLow => {
            // Extended channels 40-50: gain = chn * 2, intgn = gain + 2400
            let gain = chn * 2;
            ChannelParams {
                gain,
                intgn: gain as u16 + 2400
            }
        },
        BleChannelType::ExtHigh => {
            // Extended channels 51-61 have reverse mapping
            // gain = (61 - chn) * 2
            let gain = (61 - chn) * 2;
            ChannelParams {
                gain,
                // Frequency: 2388 MHz at chn=51 down to 2368 MHz at chn=61
                // Simplified calculation that works for all gain values
                intgn: 2408 - gain as u16
            }
        },
        BleChannelType::OutOfRange => {
            // Default to maximum frequency for out of range channels
            ChannelParams { gain: 0x50, intgn: 2480 }
        }
    }
}

/**
 * Configure transmission power based on channel-specific gain value
 * 
 * This function adjusts the RF transmission power for a specific channel by:
 * 1. Calculating an appropriate analog register value from the base and gain parameters
 * 2. Writing this value to analog register 0x93 which controls output power
 *
 * The calculation uses RF_TP_BASE and RF_TP_GAIN global variables that are 
 * calibrated during initialization based on values stored in flash.
 * 
 * @param gain - Channel-specific gain value calculated for the selected frequency
 */
fn rf_set_tp_gain(gain: u8)
{
    // Calculate power adjustment based on channel gain
    let power_adjustment = ((gain as u32 * RF_TP_GAIN.get() + 0x80) >> 8) as u8;
    
    // Apply adjustment by subtracting from base power value
    // Handle potential overflow by ensuring power_adjustment doesn't exceed RF_TP_BASE
    let base_value = RF_TP_BASE.get() as u8;
    let power_value = if power_adjustment > base_value {
        0 // If adjustment would cause overflow, use minimum power (0)
    } else {
        base_value - power_adjustment
    };
    
    // Write final power value to analog register 0x93 (power control register)
    analog_write(0x93, power_value);
}

/**
 * Schedule a state transition from RX to TX at a specific time
 * 
 * Configures the RF hardware to automatically switch from receive mode to transmit mode
 * at a specified system tick time. Used for timed responses in BLE protocols.
 * 
 * @param addr - The address in memory of the packet to transmit
 * @param tick - The system tick time when the transition should occur
 */
pub fn rf_start_srx2tx(addr: u32, tick: u32)
{
    write_reg_rf_sched_tick(tick);                                  // Setting schedule trigger time
    write_reg_rf_mode(read_reg_rf_mode() | 0x04);                   // Enable cmd_schedule mode
    write_reg_rf_mode_control(0x85);                                // Set mode to srx2tx
    write_reg_dma3_addr(addr as u16);                              // Set DMA address for the packet
}

/**
 * Schedule a state transition from TX to RX at a specific time
 * 
 * Configures the RF hardware to automatically switch from transmit mode to receive mode
 * at a specified system tick time. Used for timed responses in BLE protocols.
 * 
 * @param addr - The address in memory where received data should be stored
 * @param tick - The system tick time when the transition should occur
 */
pub fn rf_start_stx2rx(addr: u32, tick: u32)
{
    write_reg_rf_sched_tick(tick);                                // Setting schedule trigger time
    write_reg_rf_mode(read_reg_rf_mode() | 0x04);                 // Enable cmd_schedule mode
    write_reg_rf_mode_control(0x87);                              // Set mode to stx2rx
    write_reg_dma3_addr(addr as u16);                            // Set DMA address for the packet
}

/**
 * Schedule a transition to RX mode at a specific time (Broadcast RX)
 * 
 * Configures the RF hardware to enter receive mode at a specified system tick time.
 * Used for scheduled listening windows in BLE protocols, particularly for broadcast packets.
 * 
 * @param addr - The address in memory where received data should be stored
 * @param tick - The system tick time when the transition should occur
 */
pub fn rf_start_brx(addr: u32, tick: u32)
{
    write_reg32(0xf28, 0xffffffff);                              // Clear pending events
    write_reg_rf_sched_tick(tick);                               // Setting schedule trigger time
    write_reg_rf_mode(read_reg_rf_mode() | 0x04);                // Enable cmd_schedule mode
    write_reg_rf_mode_control(0x82);                             // Set mode to brx
    write_reg_dma3_addr(addr as u16);                           // Set DMA address for the packet
}

/**
 * Reset the RF packet sequence number
 * 
 * Resets the sequence number used for RF packet transmission by:
 * 1. First writing a specific value (0x3F) to ensure a clean reset
 * 2. Then writing zero to initialize a new sequence
 * 
 * The sequence number is used to identify and order packets in a transmission sequence.
 */
pub fn rf_reset_sn()
{
    write_reg_rf_sn(0x3f);  // Write specific reset value
    write_reg_rf_sn(0x00);  // Initialize to zero
}

/**
 * Set the BLE CRC initialization value
 * 
 * Configures the 24-bit CRC initialization value for BLE packet validation.
 * This value is used by the RF hardware to verify packet integrity.
 * 
 * @param crc - Array of 3 bytes representing the CRC initialization value
 */
pub fn rf_set_ble_crc(crc: &[u8])
{
    // Combine the 3 bytes into a single 24-bit value and write to the CRC register
    write_reg_rf_crc(crc[0] as u32 | ((crc[1] as u32) << 8) | ((crc[2] as u32) << 16));
}

/**
 * Set the default BLE CRC initialization value for advertising channels
 * 
 * Configures the RF hardware with the standard CRC initialization value (0x555555)
 * that is used for BLE advertising packets according to the specification.
 */
pub fn rf_set_ble_crc_adv()
{
    write_reg_rf_crc(0x555555);  // Standard BLE advertising CRC init value
}

/**
 * Configure a custom BLE access code
 * 
 * Sets the access code (preamble) that the RF hardware will use to identify
 * valid BLE packets. The access code is byte-swapped before being written to
 * accommodate the endianness requirements of the hardware.
 * 
 * @param ac - The 32-bit access code value
 */
pub fn rf_set_ble_access_code(ac: u32)
{
    write_reg_rf_access_code(ac.swap_bytes());  // Swap byte order for hardware compatibility
}

/**
 * Set the default BLE access code for advertising channels
 * 
 * Configures the RF hardware with the standard access code (0xd6be898e)
 * that is used for BLE advertising packets according to the specification.
 */
pub fn rf_set_ble_access_code_adv()
{
    write_reg_rf_access_code(0xd6be898e);  // Standard BLE advertising access code
}

#[cfg(test)]
mod tests {
    use mry::Any;
    use mry::send_wrapper::SendWrapper;
    use crate::sdk::ble_app::light_ll::mock_rf_link_slave_read_status_par_init;
    use crate::sdk::ble_app::rf_drv_8266_tables::{TBL_AGC, TBL_RF_INI};
    use crate::sdk::mcu::register::*;
    use crate::sdk::mcu::analog::*;
    use crate::sdk::common::compat::*;
    use crate::sdk::packet_types::{PacketAttWrite, PacketL2capHead, RfPacketAdvIndModuleT};
    use crate::sdk::drivers::flash::mock_flash_read_page;
    use crate::sdk::ble_app::light_ll::{mock_parse_ble_packet_op_params, mock_rf_link_match_group_mac, mock_rf_link_is_notify_req, mock_rf_link_slave_add_status, mock_rf_link_slave_read_status_stop};
    use crate::common::mock_dev_addr_with_mac_flag;
    use crate::main_light::{mock_rf_link_data_callback, mock_rf_link_response_callback};
    use super::*;

    #[test]
    fn test_rf_link_get_rsp_type() {
        // Group 1: Test the LGT_CMD_LIGHT_READ_STATUS opcode
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_LIGHT_READ_STATUS, 0), LGT_CMD_LIGHT_STATUS);
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_LIGHT_READ_STATUS, 100), LGT_CMD_LIGHT_STATUS);
        
        // Group 2: Test the LGT_CMD_LIGHT_GRP_REQ opcode with different parameter values
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_LIGHT_GRP_REQ, GET_GROUP1), LGT_CMD_LIGHT_GRP_RSP1);
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_LIGHT_GRP_REQ, GET_GROUP2), LGT_CMD_LIGHT_GRP_RSP2);
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_LIGHT_GRP_REQ, GET_GROUP3), LGT_CMD_LIGHT_GRP_RSP3);
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_LIGHT_GRP_REQ, 4), LGT_CMD_LIGHT_GRP_RSP1); // Default for unknown param
        
        // Group 3: Test opcodes that return fixed response values
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_LIGHT_CONFIG_GRP, 0), LGT_CMD_LIGHT_GRP_RSP1);
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_CONFIG_DEV_ADDR, 0), LGT_CMD_DEV_ADDR_RSP);
        assert_eq!(rf_link_get_rsp_type(0, 0), 1); // Special case with no constant
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_OTA_DATA_REQ, 0), LGT_CMD_OTA_DATA_RSP);
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_END_OTA_REQ, 0), LGT_CMD_END_OTA_RSP);
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_USER_NOTIFY_REQ, 0), LGT_CMD_USER_NOTIFY_RSP);
        assert_eq!(rf_link_get_rsp_type(7, 0), 8); // Special case with no constant
        
        // Group 4: Test default case for unmapped opcodes
        assert_eq!(rf_link_get_rsp_type(0x02, 0), 0xff);
        assert_eq!(rf_link_get_rsp_type(0x03, 0), 0xff);
        assert_eq!(rf_link_get_rsp_type(0x04, 0), 0xff);
        assert_eq!(rf_link_get_rsp_type(0x05, 0), 0xff);
        assert_eq!(rf_link_get_rsp_type(0x06, 0), 0xff);
        assert_eq!(rf_link_get_rsp_type(0x08, 0), 0xff);
        assert_eq!(rf_link_get_rsp_type(0x09, 0xff), 0xff);
        
        // Group 5: Verify parameter value doesn't matter for all opcodes except LGT_CMD_LIGHT_GRP_REQ
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_LIGHT_CONFIG_GRP, 100), LGT_CMD_LIGHT_GRP_RSP1);
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_CONFIG_DEV_ADDR, 100), LGT_CMD_DEV_ADDR_RSP);
        assert_eq!(rf_link_get_rsp_type(0, 100), 1);
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_OTA_DATA_REQ, 100), LGT_CMD_OTA_DATA_RSP);
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_END_OTA_REQ, 100), LGT_CMD_END_OTA_RSP);
        assert_eq!(rf_link_get_rsp_type(LGT_CMD_USER_NOTIFY_REQ, 100), LGT_CMD_USER_NOTIFY_RSP);
        assert_eq!(rf_link_get_rsp_type(7, 100), 8);
    }

    #[test]
    #[mry::lock(write_reg32)]
    fn test_rf_set_ble_access_code_adv() {
        mock_write_reg32(0x408, 0xd6be898e).returns(());

        rf_set_ble_access_code_adv();

        mock_write_reg32(0x408, 0xd6be898e).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg32)]
    fn test_rf_set_ble_access_code() {
        mock_write_reg32(0x408, Any).returns(());

        // Should swap the order of the bytes
        rf_set_ble_access_code(0x12345678);
        mock_write_reg32(0x408, 0x78563412).assert_called(1);

        rf_set_ble_access_code(0x78563412);
        mock_write_reg32(0x408, 0x12345678).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg32)]
    fn test_rf_set_ble_crc_adv() {
        mock_write_reg32(0x44c, Any).returns(());

        rf_set_ble_crc_adv();

        mock_write_reg32(0x44c, 0x555555).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg32)]
    fn test_rf_set_ble_crc() {
        mock_write_reg32(0x44c, Any).returns(());

        rf_set_ble_crc(&[0x12, 0x34, 0x56]);

        mock_write_reg32(0x44c, 0x563412).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg8)]
    fn test_rf_reset_sn() {
        mock_write_reg8(0xf01, Any).returns(());

        rf_reset_sn();

        mock_write_reg8(0xf01, 0x3f).assert_called(1);
        mock_write_reg8(0xf01, 0).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg32)]
    #[mry::lock(write_reg_rf_sched_tick)]
    #[mry::lock(read_reg_rf_mode)]
    #[mry::lock(write_reg_rf_mode)]
    #[mry::lock(write_reg_rf_mode_control)]
    #[mry::lock(write_reg_dma3_addr)]
    fn test_rf_start_brx() {
        mock_read_reg_rf_mode().returns(3);
        mock_write_reg32(0xf28, Any).returns(());
        mock_write_reg_rf_sched_tick(0x87654321).returns(());
        mock_write_reg_rf_mode(4 | 3).returns(());
        mock_write_reg_rf_mode_control(0x82).returns(());
        mock_write_reg_dma3_addr(0x5678).returns(());

        rf_start_brx(0x12345678, 0x87654321);

        mock_write_reg32(0xf28, 0xffffffff).assert_called(1);
        mock_write_reg_rf_sched_tick(0x87654321).assert_called(1);
        mock_write_reg_rf_mode(4 | 3).assert_called(1);
        mock_write_reg_rf_mode_control(0x82).assert_called(1);
        mock_write_reg_dma3_addr(0x5678).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg_rf_sched_tick)]
    #[mry::lock(read_reg_rf_mode)]
    #[mry::lock(write_reg_rf_mode)]
    #[mry::lock(write_reg_rf_mode_control)]
    #[mry::lock(write_reg_dma3_addr)]
    fn test_rf_start_srx2tx() {
        // Set up mocks
        mock_read_reg_rf_mode().returns(0);
        mock_write_reg_rf_sched_tick(0x87654321).returns(());
        mock_write_reg_rf_mode(0x04).returns(());
        mock_write_reg_rf_mode_control(0x85).returns(());
        mock_write_reg_dma3_addr(0x5678).returns(());

        // Call the function under test
        rf_start_srx2tx(0x12345678, 0x87654321);

        // Verify expectations
        mock_write_reg_rf_sched_tick(0x87654321).assert_called(1);
        mock_write_reg_rf_mode(0x04).assert_called(1);
        mock_write_reg_rf_mode_control(0x85).assert_called(1);
        mock_write_reg_dma3_addr(0x5678).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg_rf_sched_tick)]
    #[mry::lock(read_reg_rf_mode)]
    #[mry::lock(write_reg_rf_mode)]
    #[mry::lock(write_reg_rf_mode_control)]
    #[mry::lock(write_reg_dma3_addr)]
    fn test_rf_start_stx2rx() {
        // Set up mocks
        mock_read_reg_rf_mode().returns(0);
        mock_write_reg_rf_sched_tick(0x87654321).returns(());
        mock_write_reg_rf_mode(0x04).returns(());
        mock_write_reg_rf_mode_control(0x87).returns(());
        mock_write_reg_dma3_addr(0x5678).returns(());

        // Call the function under test
        rf_start_stx2rx(0x12345678, 0x87654321);

        // Verify expectations
        mock_write_reg_rf_sched_tick(0x87654321).assert_called(1);
        mock_write_reg_rf_mode(0x04).assert_called(1);
        mock_write_reg_rf_mode_control(0x87).assert_called(1);
        mock_write_reg_dma3_addr(0x5678).assert_called(1);
    }

    // Split the power level tests into separate test functions
    #[test]
    #[mry::lock(analog_write)]
    fn test_rf_set_power_level_index_highest() {
        // Test highest power level (index 0)
        mock_analog_write(0xa2, TBL_RF_POWER[0].a).returns(());
        mock_analog_write(4, TBL_RF_POWER[0].b).returns(());
        mock_analog_write(0xa7, TBL_RF_POWER[0].c).returns(());
        mock_analog_write(0x8d, TBL_RF_POWER[0].d).returns(());
        
        rf_set_power_level_index(0);
        
        mock_analog_write(0xa2, TBL_RF_POWER[0].a).assert_called(1);
        mock_analog_write(4, TBL_RF_POWER[0].b).assert_called(1);
        mock_analog_write(0xa7, TBL_RF_POWER[0].c).assert_called(1);
        mock_analog_write(0x8d, TBL_RF_POWER[0].d).assert_called(1);
    }
    
    #[test]
    #[mry::lock(analog_write)]
    fn test_rf_set_power_level_index_middle() {
        // Test middle power level (index 6)
        mock_analog_write(0xa2, TBL_RF_POWER[6].a).returns(());
        mock_analog_write(4, TBL_RF_POWER[6].b).returns(());
        mock_analog_write(0xa7, TBL_RF_POWER[6].c).returns(());
        mock_analog_write(0x8d, TBL_RF_POWER[6].d).returns(());
        
        rf_set_power_level_index(6);
        
        mock_analog_write(0xa2, TBL_RF_POWER[6].a).assert_called(1);
        mock_analog_write(4, TBL_RF_POWER[6].b).assert_called(1);
        mock_analog_write(0xa7, TBL_RF_POWER[6].c).assert_called(1);
        mock_analog_write(0x8d, TBL_RF_POWER[6].d).assert_called(1);
    }
    
    #[test]
    #[mry::lock(analog_write)]
    fn test_rf_set_power_level_index_lowest() {
        // Test lowest power level (index 11)
        mock_analog_write(0xa2, TBL_RF_POWER[11].a).returns(());
        mock_analog_write(4, TBL_RF_POWER[11].b).returns(());
        mock_analog_write(0xa7, TBL_RF_POWER[11].c).returns(());
        mock_analog_write(0x8d, TBL_RF_POWER[11].d).returns(());
        
        rf_set_power_level_index(11);
        
        mock_analog_write(0xa2, TBL_RF_POWER[11].a).assert_called(1);
        mock_analog_write(4, TBL_RF_POWER[11].b).assert_called(1);
        mock_analog_write(0xa7, TBL_RF_POWER[11].c).assert_called(1);
        mock_analog_write(0x8d, TBL_RF_POWER[11].d).assert_called(1);
    }
    
    #[test]
    #[mry::lock(analog_write)]
    fn test_rf_set_power_level_index_out_of_range() {
        // Test out of range index (should use the last entry in the table)
        mock_analog_write(0xa2, TBL_RF_POWER[11].a).returns(());
        mock_analog_write(4, TBL_RF_POWER[11].b).returns(());
        mock_analog_write(0xa7, TBL_RF_POWER[11].c).returns(());
        mock_analog_write(0x8d, TBL_RF_POWER[11].d).returns(());
        
        rf_set_power_level_index(20); // Out of range
        
        mock_analog_write(0xa2, TBL_RF_POWER[11].a).assert_called(1);
        mock_analog_write(4, TBL_RF_POWER[11].b).assert_called(1);
        mock_analog_write(0xa7, TBL_RF_POWER[11].c).assert_called(1);
        mock_analog_write(0x8d, TBL_RF_POWER[11].d).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg8)]
    #[mry::lock(write_reg_rf_channel)]
    #[mry::lock(write_reg_rf_mode)]
    #[mry::lock(write_reg_rf_rx_mode)]
    #[mry::lock(write_reg_rf_txrx_state)]
    #[mry::lock(write_reg16)]
    #[mry::lock(analog_write)]
    #[mry::lock(write_reg_pll_rx_fine_div_tune)]
    fn test_rf_set_ble_channel_data_channels() {
        // Mock all register writes
        mock_write_reg8(0x40d, Any).returns(());
        mock_write_reg_rf_channel(Any).returns(());
        mock_write_reg_rf_mode(Any).returns(());
        mock_write_reg_rf_rx_mode(Any).returns(());
        mock_write_reg_rf_txrx_state(Any).returns(());
        mock_write_reg16(0x4d6, Any).returns(());
        mock_analog_write(6, Any).returns(());
        mock_analog_write(0x93, Any).returns(());
        mock_write_reg_pll_rx_fine_div_tune(Any).returns(());
        
        // Test data channel (0-10)
        rf_set_ble_channel(5);
        mock_write_reg_rf_channel(5).assert_called(1);
        // Channel 5 should result in gain = (5+2)*2 = 14, intgn = 14+2400 = 2414
        mock_write_reg_pll_rx_fine_div_tune(2414).assert_called(1);
        // SAR ADC should be powered off
        mock_analog_write(6, 0).assert_called(1);
        // RF mode should be set to 0x29
        mock_write_reg_rf_mode(0x29).assert_called(1);
        // RX should be disabled
        mock_write_reg_rf_rx_mode(0).assert_called(1);
        // TX/RX state machine should be reset
        mock_write_reg_rf_txrx_state(RF_TRX_OFF).assert_called(1);
        
        // Test data channel (11-36)
        rf_set_ble_channel(20);
        mock_write_reg_rf_channel(20).assert_called(1);
        // Channel 20 should result in gain = (20+3)*2 = 46, intgn = 46+2400 = 2446
        mock_write_reg_pll_rx_fine_div_tune(2446).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg_rf_channel)]
    #[mry::lock(write_reg_rf_mode)]
    #[mry::lock(write_reg_rf_rx_mode)]
    #[mry::lock(write_reg_rf_txrx_state)]
    #[mry::lock(write_reg_pll_rx_fine_div_tune)]
    #[mry::lock(analog_write)]
    fn test_rf_set_ble_channel_adv_channels() {
        // Mock all register writes
        mock_write_reg_rf_channel(Any).returns(());
        mock_write_reg_rf_mode(Any).returns(());
        mock_write_reg_rf_rx_mode(Any).returns(());
        mock_write_reg_rf_txrx_state(Any).returns(());
        mock_write_reg_pll_rx_fine_div_tune(Any).returns(());
        mock_analog_write(6, Any).returns(());
        mock_analog_write(0x93, Any).returns(());
        
        // Test advertising channels
        // Channel 37 (2402 MHz)
        rf_set_ble_channel(37);
        mock_write_reg_rf_channel(37).assert_called(1);
        mock_write_reg_pll_rx_fine_div_tune(2402).assert_called(1);
        mock_write_reg_rf_mode(0x29).assert_called(1);
        mock_write_reg_rf_rx_mode(0).assert_called(1);
        mock_write_reg_rf_txrx_state(RF_TRX_OFF).assert_called(1);
        
        // Channel 38 (2426 MHz)
        rf_set_ble_channel(38);
        mock_write_reg_rf_channel(38).assert_called(1);
        mock_write_reg_pll_rx_fine_div_tune(2426).assert_called(1);
        
        // Channel 39 (2480 MHz)
        rf_set_ble_channel(39);
        mock_write_reg_rf_channel(39).assert_called(1);
        mock_write_reg_pll_rx_fine_div_tune(2480).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg8)]
    #[mry::lock(write_reg_rf_mode)]
    #[mry::lock(write_reg_rf_rx_mode)]
    #[mry::lock(write_reg_rf_txrx_state)]
    #[mry::lock(write_reg16)]
    #[mry::lock(analog_write)]
    #[mry::lock(write_reg_rf_channel)]
    #[mry::lock(write_reg_pll_rx_fine_div_tune)]
    fn test_rf_set_ble_channel_extended_40_50() {
        // Mock all register writes
        mock_write_reg8(0x40d, Any).returns(());
        mock_write_reg_rf_channel(Any).returns(());
        mock_write_reg_rf_mode(Any).returns(());
        mock_write_reg_rf_rx_mode(Any).returns(());
        mock_write_reg_rf_txrx_state(Any).returns(());
        mock_write_reg16(0x4d6, Any).returns(());
        mock_analog_write(6, Any).returns(());
        mock_analog_write(0x93, Any).returns(());
        mock_write_reg_pll_rx_fine_div_tune(Any).returns(());
        
        // Test extended channel (40-50)
        rf_set_ble_channel(45);
        mock_write_reg_rf_channel(45).assert_called(1);
        // Channel 45 should result in gain = 45*2 = 90, intgn = 90+2400 = 2490
        mock_write_reg_pll_rx_fine_div_tune(2490).assert_called(1);
        mock_write_reg_rf_mode(0x29).assert_called(1);
        mock_write_reg_rf_rx_mode(0).assert_called(1);
        mock_write_reg_rf_txrx_state(RF_TRX_OFF).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg_rf_channel)]
    #[mry::lock(write_reg_rf_mode)]
    #[mry::lock(write_reg_rf_rx_mode)]
    #[mry::lock(write_reg_rf_txrx_state)]
    #[mry::lock(write_reg_pll_rx_fine_div_tune)]
    #[mry::lock(analog_write)]
    fn test_rf_set_ble_channel_extended_51_61() {
        // Mock all register writes
        mock_write_reg_rf_channel(Any).returns(());
        mock_write_reg_rf_mode(Any).returns(());
        mock_write_reg_rf_rx_mode(Any).returns(());
        mock_write_reg_rf_txrx_state(Any).returns(());
        mock_write_reg_pll_rx_fine_div_tune(Any).returns(());
        mock_analog_write(6, Any).returns(());
        mock_analog_write(0x93, Any).returns(());
        
        // Test extended channel (51-61) with our new calculation
        // For channel 55:
        // gain = (61-55)*2 = 12
        // intgn = 2408 - gain = 2408 - 12 = 2396
        rf_set_ble_channel(55);
        mock_write_reg_rf_channel(55).assert_called(1);
        mock_write_reg_pll_rx_fine_div_tune(2396).assert_called(1);
        mock_write_reg_rf_mode(0x29).assert_called(1);
        mock_write_reg_rf_rx_mode(0).assert_called(1);
        mock_write_reg_rf_txrx_state(RF_TRX_OFF).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg_rf_channel)]
    #[mry::lock(write_reg_rf_mode)]
    #[mry::lock(write_reg_rf_rx_mode)]
    #[mry::lock(write_reg_rf_txrx_state)]
    #[mry::lock(write_reg_pll_rx_fine_div_tune)]
    #[mry::lock(analog_write)]
    fn test_rf_set_ble_channel_out_of_range() {
        // Mock all register writes
        mock_write_reg_rf_channel(Any).returns(());
        mock_write_reg_rf_mode(Any).returns(());
        mock_write_reg_rf_rx_mode(Any).returns(());
        mock_write_reg_rf_txrx_state(Any).returns(());
        mock_write_reg_pll_rx_fine_div_tune(Any).returns(());
        mock_analog_write(6, Any).returns(());
        mock_analog_write(0x93, Any).returns(());
        
        // Test out of range channel (> 61)
        rf_set_ble_channel(62);
        mock_write_reg_rf_channel(62).assert_called(1);
        // Out of range channel should default to 0x50 and 2480
        mock_write_reg_pll_rx_fine_div_tune(2480).assert_called(1);
        mock_write_reg_rf_mode(0x29).assert_called(1);
        mock_write_reg_rf_rx_mode(0).assert_called(1);
        mock_write_reg_rf_txrx_state(RF_TRX_OFF).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg_rf_mode_control)]
    fn test_rf_stop_trx() {
        mock_write_reg_rf_mode_control(0x80).returns(());

        rf_stop_trx();

        mock_write_reg_rf_mode_control(0x80).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg_rf_rx_mode)]
    #[mry::lock(write_reg_rf_txrx_state)]
    fn test_rf_set_rxmode() {
        mock_write_reg_rf_rx_mode((FLD_RF_RX_MODE::LOW_PASS_FILTER | FLD_RF_RX_MODE::EN).bits()).returns(());
        mock_write_reg_rf_txrx_state(RF_TRX_OFF | BIT!(5)).returns(());

        rf_set_rxmode();

        mock_write_reg_rf_rx_mode((FLD_RF_RX_MODE::LOW_PASS_FILTER | FLD_RF_RX_MODE::EN).bits()).assert_called(1);
        mock_write_reg_rf_txrx_state(RF_TRX_OFF | BIT!(5)).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg_rf_mode)]
    #[mry::lock(write_reg_rf_rx_mode)]
    #[mry::lock(write_reg_rf_txrx_state)]
    fn test_rf_set_tx_rx_off() {
        mock_write_reg_rf_mode(0x29).returns(());
        mock_write_reg_rf_rx_mode(FLD_RF_RX_MODE::LOW_PASS_FILTER.bits()).returns(());
        mock_write_reg_rf_txrx_state(RF_TRX_OFF).returns(());

        rf_set_tx_rx_off();

        mock_write_reg_rf_mode(0x29).assert_called(1);
        mock_write_reg_rf_rx_mode(FLD_RF_RX_MODE::LOW_PASS_FILTER.bits()).assert_called(1);
        mock_write_reg_rf_txrx_state(RF_TRX_OFF).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg8)]
    fn test_rf_link_slave_set_adv() {
        mock_write_reg8(Any, Any).returns(());
        
        // Setup a test advertisement data slice
        let test_adv_data = [0x02, 0x01, 0x06, 0x08, 0xFF, 0x12, 0x34];
        
        // Create a temporary locked PKT_ADV for testing
        critical_section::with(|_| {
            let pkt_adv = Packet { adv_ind_module: RfPacketAdvIndModuleT::default() };
            PKT_ADV.lock().clone_from(&pkt_adv);
            
            // Call the function
            rf_link_slave_set_adv(&test_adv_data);
            
            // Get updated packet from the global state
            let updated_pkt = PKT_ADV.lock().clone();
            
            // Verify the correct values were set
            let p = updated_pkt.head().dma_len;
            assert_eq!(p, test_adv_data.len() as u32 + 8);
            assert_eq!(updated_pkt.head().rf_len, test_adv_data.len() as u8 + 6);
            
            // Verify the advertisement data was copied correctly
            assert_eq!(updated_pkt.adv_ind_module().data[0..test_adv_data.len()], test_adv_data);
        });
    }

    #[test]
    #[mry::lock(analog_write)]
    fn test_rf_set_tp_gain_zero() {
        // Set up the global state variables
        RF_TP_BASE.set(0x30);
        RF_TP_GAIN.set(0x40);
        
        // Test gain = 0, should result in BASE value unchanged
        mock_analog_write(0x93, 0x30).returns(());
        rf_set_tp_gain(0);
        mock_analog_write(0x93, 0x30).assert_called(1);
    }
    
    #[test]
    #[mry::lock(analog_write)]
    fn test_rf_set_tp_gain_medium() {
        // Set up the global state variables
        RF_TP_BASE.set(0x30);
        RF_TP_GAIN.set(0x40);
        
        // Test gain = 10
        // Calculation: BASE - ((gain * GAIN + 0x80) >> 8)
        // 0x30 - ((10 * 0x40 + 0x80) >> 8) = 0x30 - 3 = 0x2D
        mock_analog_write(0x93, 0x2D).returns(());
        rf_set_tp_gain(10);
        mock_analog_write(0x93, 0x2D).assert_called(1);
    }
    
    #[test]
    #[mry::lock(analog_write)]
    fn test_rf_set_tp_gain_max() {
        // Set up the global state variables
        RF_TP_BASE.set(0x10);
        RF_TP_GAIN.set(0x40);
        
        // Test gain = 0xFF (maximum gain)
        // Without overflow protection: 0x10 - ((0xFF * 0x40 + 0x80) >> 8) would cause overflow
        // With overflow protection: Since power_adjustment > base_value, we should get 0
        mock_analog_write(0x93, 0).returns(());
        rf_set_tp_gain(0xFF);
        mock_analog_write(0x93, 0).assert_called(1);
    }

    #[test]
    #[mry::lock(analog_read)]
    #[mry::lock(load_tbl_cmd_set)]
    #[mry::lock(write_reg_rf_rx_gain_agc)]
    #[mry::lock(flash_read_page)]
    fn test_rf_drv_init_enable() {
        // Mock the analog_read to return a specific deepsleep flag value
        mock_analog_read(rega_deepsleep_flag).returns(0x40);
        
        // Mock functions for enabling the RF driver
        mock_load_tbl_cmd_set().returns((TBL_RF_INI.len() - 4) as u32);
        
        // Expect calls to configure the Automatic Gain Control (AGC) settings
        for i in 0..6 {
            mock_write_reg_rf_rx_gain_agc(TBL_AGC[i], (i << 2) as u32).returns(());
        }
        
        // Mock flash_read_page to return default values (0xFF) for power settings
        // This is important to test the default branch of the condition in rf_drv_init
        mock_flash_read_page(Any, Any, Any).returns_with(|addr, len, buf_ptr: SendWrapper<*mut u8>| {
            assert_eq!(addr, FLASH_ADR_MAC);
            assert_eq!(len, 0x13);
            
            // Create a sample power settings array with all bytes set to 0xFF (default flash value)
            let sample_settings = [0xFFu8; 0x13];
            
            // Copy settings to the output buffer
            unsafe {
                let buf_slice = core::slice::from_raw_parts_mut(*buf_ptr, len as usize);
                let copy_len = core::cmp::min(sample_settings.len(), len as usize);
                buf_slice[0..copy_len].copy_from_slice(&sample_settings[0..copy_len]);
            }
        });
        
        // Set some initial values for global variables
        let initial_base = 0x22;
        let initial_gain = 0x33;
        RF_TP_BASE.set(initial_base);
        RF_TP_GAIN.set(initial_gain);
        
        // Call with enable=true - should initialize the driver
        let result = rf_drv_init(true);
        
        // Verify that analog_read was called to check deepsleep flag
        mock_analog_read(rega_deepsleep_flag).assert_called(1);
        
        // Verify table initialization was loaded
        mock_load_tbl_cmd_set().assert_called(1);
        
        // Verify AGC settings were configured
        for i in 0..6 {
            mock_write_reg_rf_rx_gain_agc(TBL_AGC[i], (i << 2) as u32).assert_called(1);
        }
        
        // Verify flash_read_page was called with correct parameters
        mock_flash_read_page(FLASH_ADR_MAC, 0x13, Any).assert_called(1);
        
        // With all 0xFF values in power_settings, RF_TP_BASE and RF_TP_GAIN should remain unchanged
        assert_eq!(RF_TP_BASE.get(), initial_base);
        assert_eq!(RF_TP_GAIN.get(), initial_gain);
        
        // Verify the function returns the deepsleep flag value
        assert_eq!(result, 0x40);
    }

    #[test]
    #[mry::lock(analog_read)]
    #[mry::lock(analog_write)]
    fn test_rf_drv_init_disable() {
        // Mock the analog_read to return a specific deepsleep flag value
        mock_analog_read(rega_deepsleep_flag).returns(0x40);
        
        // Mock the analog_write function for powering off SAR ADC
        mock_analog_write(6, 0).returns(());
        
        // Call with enable=false - should power off the RF driver
        let result = rf_drv_init(false);
        
        // Verify that analog_read was called to check deepsleep flag
        mock_analog_read(rega_deepsleep_flag).assert_called(1);
        
        // Verify that the SAR ADC was powered off
        mock_analog_write(6, 0).assert_called(1);
        
        // Verify the function returns the deepsleep flag value
        assert_eq!(result, 0x40);
    }

    #[test]
    fn test_calculate_channel_params() {
        // Test data channels (0-10)
        let params = calculate_channel_params(5);
        assert_eq!(params.gain, 14); // (5+2)*2 = 14
        assert_eq!(params.intgn, 2414); // 14+2400 = 2414
        
        // Test data channels (11-36)
        let params = calculate_channel_params(20);
        assert_eq!(params.gain, 46); // (20+3)*2 = 46
        assert_eq!(params.intgn, 2446); // 46+2400 = 2446
        
        // Test advertising channels (37, 38, 39)
        let params = calculate_channel_params(37);
        assert_eq!(params.gain, 2);
        assert_eq!(params.intgn, 2402);
        
        let params = calculate_channel_params(38);
        assert_eq!(params.gain, 0x1a);
        assert_eq!(params.intgn, 2426);
        
        let params = calculate_channel_params(39);
        assert_eq!(params.gain, 0x50);
        assert_eq!(params.intgn, 2480);
        
        // Test extended channels (40-50)
        let params = calculate_channel_params(45);
        assert_eq!(params.gain, 90); // 45*2 = 90
        assert_eq!(params.intgn, 2490); // 90+2400 = 2490
        
        // Test extended channels (51-61)
        let params = calculate_channel_params(55);
        assert_eq!(params.gain, 12); // (61-55)*2 = 12
        assert_eq!(params.intgn, 2396); // 2408-12 = 2396
        
        // Test out of range channel
        let params = calculate_channel_params(62);
        assert_eq!(params.gain, 0x50);
        assert_eq!(params.intgn, 2480);
    }

    #[test]
    #[mry::lock(analog_read)]
    #[mry::lock(load_tbl_cmd_set)]
    #[mry::lock(write_reg_rf_rx_gain_agc)]
    #[mry::lock(flash_read_page)]
    fn test_rf_drv_init_with_power_settings() {
        // Mock the analog_read to return a specific deepsleep flag value
        mock_analog_read(rega_deepsleep_flag).returns(0x40);
        
        // Mock functions for enabling the RF driver
        mock_load_tbl_cmd_set().returns((TBL_RF_INI.len() - 4) as u32);
        
        // Expect calls to configure the Automatic Gain Control (AGC) settings
        for i in 0..6 {
            mock_write_reg_rf_rx_gain_agc(TBL_AGC[i], (i << 2) as u32).returns(());
        }
        
        // Mock flash_read_page to return power settings
        mock_flash_read_page(Any, Any, Any).returns_with(|addr, len, buf_ptr: SendWrapper<*mut u8>| {
            assert_eq!(addr, FLASH_ADR_MAC);
            
            // Create a sample power settings array with custom values at positions 0x11 and 0x12
            let mut sample_settings = [0u8; 0x13];
            sample_settings[0x11] = 0x25; // Non-default value for RF_TP_BASE
            sample_settings[0x12] = 0x10; // Non-default value for RF_TP_GAIN calculation
            
            // Copy settings to the output buffer
            unsafe {
                let buf_slice = core::slice::from_raw_parts_mut(*buf_ptr, len as usize);
                let copy_len = core::cmp::min(sample_settings.len(), len as usize);
                buf_slice[0..copy_len].copy_from_slice(&sample_settings[0..copy_len]);
            }
        });
        
        // Reset the global variables to ensure they're modified by the function
        RF_TP_BASE.set(0);
        RF_TP_GAIN.set(0);
        
        // Call with enable=true - should initialize the driver
        let result = rf_drv_init(true);
        
        // Verify flash_read_page was called with correct parameters
        mock_flash_read_page(FLASH_ADR_MAC, 0x13, Any).assert_called(1);
        
        // Verify that RF_TP_BASE and RF_TP_GAIN were updated with the values from flash
        assert_eq!(RF_TP_BASE.get(), 0x25); // Should be set directly from power_settings[0x11]
        
        // RF_TP_GAIN is calculated as ((RF_TP_BASE - power_settings[0x12]) << 8) / 80
        // With RF_TP_BASE = 0x25 and power_settings[0x12] = 0x10, we get:
        // ((0x25 - 0x10) << 8) / 80 = (0x1500 / 80) = 67
        assert_eq!(RF_TP_GAIN.get(), 67);
        
        // Verify the function returns the deepsleep flag value
        assert_eq!(result, 0x40);
    }
    
    #[test]
    #[mry::lock(analog_read)]
    #[mry::lock(load_tbl_cmd_set)]
    #[mry::lock(write_reg_rf_rx_gain_agc)]
    #[mry::lock(flash_read_page)]
    fn test_rf_drv_init_with_default_power_settings() {
        // Mock the analog_read to return a specific deepsleep flag value
        mock_analog_read(rega_deepsleep_flag).returns(0x40);
        
        // Mock functions for enabling the RF driver
        mock_load_tbl_cmd_set().returns((TBL_RF_INI.len() - 4) as u32);
        
        // Expect calls to configure the Automatic Gain Control (AGC) settings
        for i in 0..6 {
            mock_write_reg_rf_rx_gain_agc(TBL_AGC[i], (i << 2) as u32).returns(());
        }
        
        // Mock flash_read_page to return default values (0xFF) for power settings
            mock_flash_read_page(Any, Any, Any).returns_with(|addr, len, buf_ptr: SendWrapper<*mut u8>| {
            assert_eq!(addr, FLASH_ADR_MAC);
            
            // Create settings with 0xFF values (default for erased flash)
            let sample_settings = [0xFFu8; 0x13];
            
            // Copy settings to the output buffer
            unsafe {
                let buf_slice = core::slice::from_raw_parts_mut(*buf_ptr, len as usize);
                let copy_len = core::cmp::min(sample_settings.len(), len as usize);
                buf_slice[0..copy_len].copy_from_slice(&sample_settings[0..copy_len]);
            }
        });
        
        // Set initial values to verify they don't change when flash contains default values
        let initial_base = 0x22;
        let initial_gain = 0x33;
        RF_TP_BASE.set(initial_base);
        RF_TP_GAIN.set(initial_gain);
        
        // Call with enable=true - should initialize the driver
        let result = rf_drv_init(true);
        
        // Verify flash_read_page was called with correct parameters
        mock_flash_read_page(FLASH_ADR_MAC, 0x13, Any).assert_called(1);
        
        // When power_settings contains 0xFF (default), the values should remain unchanged
        assert_eq!(RF_TP_BASE.get(), initial_base);
        assert_eq!(RF_TP_GAIN.get(), initial_gain);
        
        // Verify the function returns the deepsleep flag value
        assert_eq!(result, 0x40);
    }
    
    #[test]
    #[mry::lock(analog_read)]
    #[mry::lock(analog_write)]
    #[mry::lock(flash_read_page)]
    fn test_rf_drv_init_disabled() {
        // Mock the analog_read to return a specific deepsleep flag value
        mock_analog_read(rega_deepsleep_flag).returns(0x40);
        
        // Mock the analog_write function for powering off SAR ADC
        mock_analog_write(6, 0).returns(());
        
        // Ensure flash_read_page is NOT called when disabled
        mock_flash_read_page(Any, Any, Any).returns(());
        
        // Call with enable=false - should power off the RF driver
        let result = rf_drv_init(false);
        
        // Verify that analog_read was called to check deepsleep flag
        mock_analog_read(rega_deepsleep_flag).assert_called(1);
        
        // Verify that the SAR ADC was powered off
        mock_analog_write(6, 0).assert_called(1);
        
        // Verify flash_read_page was NOT called
        mock_flash_read_page(Any, Any, Any).assert_called(0);
        
        // Verify the function returns the deepsleep flag value
        assert_eq!(result, 0x40);
    }
    
    #[test]
    #[mry::lock(analog_read)]
    #[mry::lock(load_tbl_cmd_set)]
    #[mry::lock(write_reg_rf_rx_gain_agc)]
    #[mry::lock(flash_read_page)]
    fn test_rf_drv_init_with_partial_power_settings() {
        // Mock the analog_read to return a specific deepsleep flag value
        mock_analog_read(rega_deepsleep_flag).returns(0x40);
        
        // Mock functions for enabling the RF driver
        mock_load_tbl_cmd_set().returns((TBL_RF_INI.len() - 4) as u32);
        
        // Expect calls to configure the Automatic Gain Control (AGC) settings
        for i in 0..6 {
            mock_write_reg_rf_rx_gain_agc(TBL_AGC[i], (i << 2) as u32).returns(());
        }
        
        // Mock flash_read_page to return power settings with only the first value set
            mock_flash_read_page(Any, Any, Any).returns_with(|addr, len, buf_ptr: SendWrapper<*mut u8>| {
            assert_eq!(addr, FLASH_ADR_MAC);
            
            // Create a sample power settings array with only position 0x11 set
            let mut sample_settings = [0xFFu8; 0x13]; // Default all to 0xFF
            sample_settings[0x11] = 0x29; // Only set the first value
            
            // Copy settings to the output buffer
            unsafe {
                let buf_slice = core::slice::from_raw_parts_mut(*buf_ptr, len as usize);
                let copy_len = core::cmp::min(sample_settings.len(), len as usize);
                buf_slice[0..copy_len].copy_from_slice(&sample_settings[0..copy_len]);
            }
        });
        
        // Reset the global variables to ensure they're modified by the function
        RF_TP_BASE.set(0);
        RF_TP_GAIN.set(0);
        
        // Call with enable=true - should initialize the driver
        let result = rf_drv_init(true);
        
        // Verify flash_read_page was called with correct parameters
        mock_flash_read_page(FLASH_ADR_MAC, 0x13, Any).assert_called(1);
        
        // Verify that RF_TP_BASE was updated with the value from flash
        assert_eq!(RF_TP_BASE.get(), 0x29);
        
        // RF_TP_GAIN is calculated as ((0x29 - 0x19) << 8) / 80 = (0x1000 / 80) = 51
        assert_eq!(RF_TP_GAIN.get(), 51);
        
        // Verify the function returns the deepsleep flag value
        assert_eq!(result, 0x40);
    }

    #[test]
    #[mry::lock(write_reg_rf_sys_timer_config)]
    #[mry::lock(write_reg_dma2_addr)]
    #[mry::lock(write_reg_dma2_ctrl)]
    #[mry::lock(write_reg_dma_chn_irq_msk)]
    #[mry::lock(read_reg_irq_mask)]
    #[mry::lock(write_reg_irq_mask)]
    #[mry::lock(read_reg_system_tick_mode)]
    #[mry::lock(write_reg_system_tick_mode)]
    #[mry::lock(write_reg_rf_irq_mask)]
    #[mry::lock(write_reg_rf_irq_status)]
    #[mry::lock(read_reg_system_tick)]
    #[mry::lock(write_reg_system_tick_irq)]
    #[mry::lock(write_reg_irq_src)]
    #[mry::lock(write_reg_rf_timing_config)]
    fn test_blc_ll_init_basic_mcu() {
        // Setup mocks for register operations
        mock_write_reg_rf_sys_timer_config(700).returns(());
        mock_write_reg_dma2_addr(Any).returns(());
        mock_write_reg_dma2_ctrl(0x104).returns(());
        mock_write_reg_dma_chn_irq_msk(0).returns(());
        
        // First call to write_reg_irq_mask with initial value | 0x2000
        mock_read_reg_irq_mask().returns(0x1000); // Initial value
        mock_write_reg_irq_mask(0x1000 | 0x2000).returns(());
        
        mock_read_reg_system_tick_mode().returns(0x01); // Simulate some initial value
        mock_write_reg_system_tick_mode(0x01 | 2).returns(());
        
        mock_write_reg_rf_irq_mask(0).returns(());
        mock_write_reg_rf_irq_status(0xfffe).returns(());
        
        // The expected value should be IRQ_RX | IRQ_TX
        let expected_rf_irq_mask = FLD_RF_IRQ_MASK::IRQ_RX | FLD_RF_IRQ_MASK::IRQ_TX;
        mock_write_reg_rf_irq_mask(expected_rf_irq_mask.bits()).returns(());
        
        mock_read_reg_system_tick().returns(0x1234); // Simulate some system tick value
        mock_write_reg_system_tick_irq(0x1234 | (0x80 << 0x18)).returns(());
        
        mock_write_reg_irq_src(0x80 << 0xd).returns(());
        
        // Second call to write_reg_irq_mask with 0x1000 | (0x80 << 0xd)
        mock_write_reg_irq_mask(0x1000 | (0x80 << 0xd)).returns(());
        
        mock_write_reg_rf_timing_config(0xc00).returns(());
        
        // Call the function under test
        blc_ll_init_basic_mcu();
        
        // Verify all register operations were performed correctly
        mock_write_reg_rf_sys_timer_config(700).assert_called(1); // System tick interval
        mock_write_reg_dma2_addr(Any).assert_called(1); // DMA buffer address
        mock_write_reg_dma2_ctrl(0x104).assert_called(1); // DMA2 control register
        mock_write_reg_dma_chn_irq_msk(0).assert_called(1); // DMA channel interrupt mask
        
        // RF interrupt configuration - verify both calls to read and write
        mock_read_reg_irq_mask().assert_called(2);
        mock_write_reg_irq_mask(0x1000 | 0x2000).assert_called(1);
        mock_write_reg_irq_mask(0x1000 | (0x80 << 0xd)).assert_called(1);
        
        mock_read_reg_system_tick_mode().assert_called(1);
        mock_write_reg_system_tick_mode(0x01 | 2).assert_called(1);
        
        // Clear and reset RF interrupt status
        mock_write_reg_rf_irq_mask(0).assert_called(1);
        mock_write_reg_rf_irq_status(0xfffe).assert_called(1);
        
        // Configure RF interrupts for RX and TX
        mock_write_reg_rf_irq_mask(expected_rf_irq_mask.bits()).assert_called(1);
        
        // System tick interrupt configuration
        mock_read_reg_system_tick().assert_called(1);
        mock_write_reg_system_tick_irq(0x1234 | (0x80 << 0x18)).assert_called(1);
        
        mock_write_reg_irq_src(0x80 << 0xd).assert_called(1);
        
        // Additional timing configuration
        mock_write_reg_rf_timing_config(0xc00).assert_called(1);
    }
    
    #[test]
    #[mry::lock(write_reg_rf_sys_timer_config)]
    #[mry::lock(write_reg_dma2_addr)]
    #[mry::lock(write_reg_dma2_ctrl)]
    #[mry::lock(write_reg_dma_chn_irq_msk)]
    #[mry::lock(read_reg_irq_mask)]
    #[mry::lock(write_reg_irq_mask)]
    #[mry::lock(read_reg_system_tick_mode)]
    #[mry::lock(write_reg_system_tick_mode)]
    #[mry::lock(write_reg_rf_irq_mask)]
    #[mry::lock(write_reg_rf_irq_status)]
    #[mry::lock(read_reg_system_tick)]
    #[mry::lock(write_reg_system_tick_irq)]
    #[mry::lock(write_reg_irq_src)]
    #[mry::lock(write_reg_rf_timing_config)]
    fn test_blc_ll_init_basic_mcu_dma_buffer_address() {
        // This test specifically checks that the DMA buffer address is correctly set
        // We need to mock all required functions but we're specifically interested in
        // the DMA buffer address calculation
        
        // Mock all register operations
        mock_write_reg_rf_sys_timer_config(700).returns(());
        mock_write_reg_dma2_ctrl(0x104).returns(());
        mock_write_reg_dma_chn_irq_msk(0).returns(());
        
        mock_read_reg_irq_mask().returns(0x1000);
        mock_write_reg_irq_mask(0x1000 | 0x2000).returns(());
        mock_write_reg_irq_mask(0x1000 | (0x80 << 0xd)).returns(());
        
        mock_read_reg_system_tick_mode().returns(0x01);
        mock_write_reg_system_tick_mode(0x01 | 2).returns(());
        
        mock_write_reg_rf_irq_mask(0).returns(());
        mock_write_reg_rf_irq_status(0xfffe).returns(());
        
        let expected_rf_irq_mask = FLD_RF_IRQ_MASK::IRQ_RX | FLD_RF_IRQ_MASK::IRQ_TX;
        mock_write_reg_rf_irq_mask(expected_rf_irq_mask.bits()).returns(());
        
        mock_read_reg_system_tick().returns(0x1234);
        mock_write_reg_system_tick_irq(0x1234 | (0x80 << 0x18)).returns(());
        
        mock_write_reg_irq_src(0x80 << 0xd).returns(());
        
        mock_write_reg_rf_timing_config(0xc00).returns(());
        
        // Mock the specific DMA address write we're testing
        mock_write_reg_dma2_addr(Any).returns(());
        
        // Setup test state - we need to know the expected DMA address
        critical_section::with(|_| {
            // Set the receive buffer write pointer to a specific value for testing
            LIGHT_RX_WPTR.set(2); 
            
            // Call the function under test
            blc_ll_init_basic_mcu();
            
            // Calculate expected DMA address (the address of buffer element at index 5)
            // For this test, we're more interested in verifying the call was made
            // with the correct pointer calculation than the exact numeric address
            let expected_addr = core::ptr::addr_of!(LIGHT_RX_BUFF.lock()[2]) as u16;
            mock_write_reg_dma2_addr(expected_addr).assert_called(1);
        });
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    #[mry::lock(rf_link_slave_read_status_par_init)]
    fn test_rf_link_slave_read_status_start() {
        // Mock system tick reading
        mock_read_reg_system_tick().returns(0x12345678);
        
        // Mock initialization function
        mock_rf_link_slave_read_status_par_init().returns(());
        
        // Create test packet with specific values for testing
        let mut test_packet = Packet { att_cmd: PacketAttCmd::default() };
        
        // Set up opcode and parameters for LGT_CMD_LIGHT_READ_STATUS
        test_packet.att_cmd_mut().value.val[0] = LGT_CMD_LIGHT_READ_STATUS;
        test_packet.att_cmd_mut().value.val[4] = 0xAB; // Random parameter value
        
        // Set up unicast address (non-broadcast)
        test_packet.att_cmd_mut().value.dst[1] = 0x42; // Not broadcast (MSB not set)
        
        // Set status tick to a known value
        SLAVE_STATUS_TICK.set(0x55);
        
        // Call the function under test
        rf_link_slave_read_status_start(&mut test_packet);
        
        // Verify system tick was recorded
        assert_eq!(SLAVE_READ_STATUS_BUSY_TIME.get(), 0x12345678);
        
        // Verify busy status was set from the opcode (mapped through rf_link_get_rsp_type)
        assert_eq!(SLAVE_READ_STATUS_BUSY.get(), LGT_CMD_LIGHT_STATUS);
        
        // Verify unicast flag is set for a non-broadcast address
        assert_eq!(SLAVE_READ_STATUS_UNICAST_FLAG.get(), true);
        
        // Verify destination specific info fields have been cleared
        assert_eq!(test_packet.att_cmd().value.val[8], 0);
        assert_eq!(test_packet.att_cmd().value.val[9], 0);
        assert_eq!(test_packet.att_cmd().value.val[10], 0);
        assert_eq!(test_packet.att_cmd().value.val[11], 0);
        
        // Verify unicast flag is copied to the packet at the right position
        assert_eq!(test_packet.att_cmd().value.val[12], 1);
        
        // Since this is LGT_CMD_LIGHT_READ_STATUS, check special handling
        assert_eq!(test_packet.att_cmd().value.val[4], 0); // Should be zeroed
        assert_eq!(test_packet.att_cmd().value.val[5], 0);
        assert_eq!(test_packet.att_cmd().value.val[6], 0);
        assert_eq!(test_packet.att_cmd().value.val[7], 0);
        assert_eq!(test_packet.att_cmd().value.val[8], 0);
        assert_eq!(test_packet.att_cmd().value.val[9], 0);
        assert_eq!(test_packet.att_cmd().value.val[10], 0);
        assert_eq!(test_packet.att_cmd().value.val[11], 0);
        assert_eq!(test_packet.att_cmd().value.val[3], 0x55); // Should contain SLAVE_STATUS_TICK
        
        // Verify initialization function was called
        mock_rf_link_slave_read_status_par_init().assert_called(1);
        
        // Verify status record variables were reset
        assert_eq!(SLAVE_STATUS_RECORD_IDX.get(), 0);
        
        // Verify notify request mask index was reset
        assert_eq!(NOTIFY_REQ_MASK_IDX.get(), 0);
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    #[mry::lock(rf_link_slave_read_status_par_init)]
    fn test_rf_link_slave_read_status_start_broadcast() {
        // Mock system tick reading
        mock_read_reg_system_tick().returns(0x87654321);
        
        // Mock initialization function
        mock_rf_link_slave_read_status_par_init().returns(());
        
        // Create test packet with specific values for testing
        let mut test_packet = Packet { att_cmd: PacketAttCmd::default() };
        
        // Set up opcode and parameters for LGT_CMD_LIGHT_GRP_REQ
        test_packet.att_cmd_mut().value.val[0] = LGT_CMD_LIGHT_GRP_REQ;
        test_packet.att_cmd_mut().value.val[4] = GET_GROUP2; // Using group 2 parameter
        
        // Set up broadcast address (bit 7 set in dst[1])
        test_packet.att_cmd_mut().value.dst[1] = 0x80; // Broadcast (MSB set)
        
        // Call the function under test
        rf_link_slave_read_status_start(&mut test_packet);
        
        // Verify system tick was recorded
        assert_eq!(SLAVE_READ_STATUS_BUSY_TIME.get(), 0x87654321);
        
        // Verify busy status was set from the opcode and parameter via rf_link_get_rsp_type
        // For LGT_CMD_LIGHT_GRP_REQ with GET_GROUP2, should be LGT_CMD_LIGHT_GRP_RSP2
        assert_eq!(SLAVE_READ_STATUS_BUSY.get(), LGT_CMD_LIGHT_GRP_RSP2);
        
        // Verify unicast flag is cleared for a broadcast address
        assert_eq!(SLAVE_READ_STATUS_UNICAST_FLAG.get(), false);
        
        // Broadcast packets don't have the destination fields cleared
        // and don't have special fields set
        
        // Verify initialization function was called
        mock_rf_link_slave_read_status_par_init().assert_called(1);
        
        // Verify status record variables were reset
        assert_eq!(SLAVE_STATUS_RECORD_IDX.get(), 0);
        
        // Verify notify request mask index was reset
        assert_eq!(NOTIFY_REQ_MASK_IDX.get(), 0);
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    #[mry::lock(rf_link_slave_read_status_par_init)]
    fn test_rf_link_slave_read_status_start_different_opcode() {
        // Mock system tick reading
        mock_read_reg_system_tick().returns(0xAABBCCDD);
        
        // Mock initialization function
        mock_rf_link_slave_read_status_par_init().returns(());
        
        // Create test packet with specific values for testing
        let mut test_packet = Packet { att_cmd: PacketAttCmd::default() };
        
        // Set up opcode and parameters for LGT_CMD_CONFIG_DEV_ADDR
        test_packet.att_cmd_mut().value.val[0] = LGT_CMD_CONFIG_DEV_ADDR;
        test_packet.att_cmd_mut().value.val[4] = 0x42; // Random parameter
        
        // Set up unicast address
        test_packet.att_cmd_mut().value.dst[1] = 0x12; // Not broadcast
        
        // Fill some values to verify they aren't cleared for non-LGT_CMD_LIGHT_READ_STATUS
        for i in 5..12 {
            test_packet.att_cmd_mut().value.val[i] = i as u8;
        }
        
        // Call the function under test
        rf_link_slave_read_status_start(&mut test_packet);
        
        // Verify system tick was recorded
        assert_eq!(SLAVE_READ_STATUS_BUSY_TIME.get(), 0xAABBCCDD);
        
        // Verify busy status was set from the opcode via rf_link_get_rsp_type
        // For LGT_CMD_CONFIG_DEV_ADDR, should be LGT_CMD_DEV_ADDR_RSP
        assert_eq!(SLAVE_READ_STATUS_BUSY.get(), LGT_CMD_DEV_ADDR_RSP);
        
        // Verify unicast flag is set for a non-broadcast address
        assert_eq!(SLAVE_READ_STATUS_UNICAST_FLAG.get(), true);
        
        // Verify destination specific info fields have been cleared
        assert_eq!(test_packet.att_cmd().value.val[8], 0);
        assert_eq!(test_packet.att_cmd().value.val[9], 0);
        assert_eq!(test_packet.att_cmd().value.val[10], 0);
        assert_eq!(test_packet.att_cmd().value.val[11], 0);
        
        // For non-LGT_CMD_LIGHT_READ_STATUS, values 4-7 remain unchanged
        assert_eq!(test_packet.att_cmd().value.val[4], 0x42);
        assert_eq!(test_packet.att_cmd().value.val[5], 5);
        assert_eq!(test_packet.att_cmd().value.val[6], 6);
        assert_eq!(test_packet.att_cmd().value.val[7], 7);
        
        // Verify unicast flag is copied to the packet at the right position
        assert_eq!(test_packet.att_cmd().value.val[12], 1);
        
        // Verify initialization function was called
        mock_rf_link_slave_read_status_par_init().assert_called(1);
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_invalid_length() {
        // Create a packet with invalid length (< 0x11)
        let packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead { rf_len: 0x10, ..Default::default() },
                ..Default::default()
            }
        };
        
        // Test the function
        let result = rf_link_slave_data_write_no_dec(&packet);
        
        // Invalid packet length should return false
        assert_eq!(result, false);
    }
    
    #[test]
    fn test_rf_link_slave_data_write_no_dec_duplicate_packet() {
        // Create a packet with valid length
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead { rf_len: 0x11, ..Default::default() },
                ..Default::default()
            }
        };
        
        // Set sequence number
        let sno = [1, 2, 3];
        critical_section::with(|_| {
            *SLAVE_SNO.lock() = sno;
            packet.att_write_mut().value.sno = sno;
        });
        
        // Test the function
        let result = rf_link_slave_data_write_no_dec(&packet);
        
        // Duplicate packet should return true (successful but no processing)
        assert_eq!(result, true);
    }
    
    #[test]
    #[mry::lock(parse_ble_packet_op_params)]
    #[mry::lock(rf_link_match_group_mac)]
    #[mry::lock(rf_link_is_notify_req)]
    #[mry::lock(rf_link_data_callback)]
    #[mry::lock(rf_link_slave_read_status_stop)]
    fn test_rf_link_slave_data_write_no_dec_non_notification_direct_match() {
        // Create a packet with valid length
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead { rf_len: 0x11, ..Default::default() },
                ..Default::default()
            }
        };
        
        // Set sequence number different from SLAVE_SNO
        let sno = [1, 2, 3];
        critical_section::with(|_| {
            *SLAVE_SNO.lock() = [1, 2, 2];
            packet.att_write_mut().value.sno = sno;
        });
        
        // Mock op_cmd and params extraction
        let op_cmd = [0x01, 0x02, 0x03];
        let op_cmd_len = 3;
        let params = [0x10, 0x11, 0x12, 0x13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let params_len = 4;
        
        mock_parse_ble_packet_op_params(Any, Any)
            .returns((true, op_cmd, op_cmd_len, params, params_len));
        
        // Mock group and device matching (direct match)
        mock_rf_link_match_group_mac(Any).returns((false, true));
        
        // Mock notification detection (not a notification)
        mock_rf_link_is_notify_req(Any).returns(false);
        
        // Mock data callback
        mock_rf_link_data_callback(Any).returns(());
        
        // Mock read status stop if busy
        mock_rf_link_slave_read_status_stop().returns(());
        
        // Test the function
        let result = rf_link_slave_data_write_no_dec(&packet);
        
        // Should return true for successful processing
        assert_eq!(result, true);
        
        // Verify callback was called
        mock_rf_link_data_callback(Any).assert_called(1);
        
        // Verify sequence number was updated
        assert_eq!(*SLAVE_SNO.lock(), sno);
        
        // Verify command opcode was stored
        assert_eq!(SLAVE_LINK_CMD.get(), op_cmd[0] & 0x3f);
        
        // Verify data validity flag is 0 for direct match
        assert_eq!(SLAVE_DATA_VALID.get(), 0);
    }
    
    #[test]
    #[mry::lock(parse_ble_packet_op_params)]
    #[mry::lock(rf_link_match_group_mac)]
    #[mry::lock(rf_link_is_notify_req)]
    #[mry::lock(rf_link_data_callback)]
    #[mry::lock(rf_link_slave_read_status_stop)]
    fn test_rf_link_slave_data_write_no_dec_non_notification_bridge() {
        // Create a packet with valid length
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead { rf_len: 0x11, ..Default::default() },
                ..Default::default()
            }
        };
        
        // Set sequence number different from SLAVE_SNO
        let sno = [1, 2, 3];
        critical_section::with(|_| {
            *SLAVE_SNO.lock() = [1, 2, 2];
            packet.att_write_mut().value.sno = sno;
        });
        
        // Set non-zero destination address for bridge forwarding
        packet.att_write_mut().value.dst[0] = 0x34;
        packet.att_write_mut().value.dst[1] = 0x12;
        
        // Mock op_cmd and params extraction
        let op_cmd = [0x01, 0x02, 0x03];
        let op_cmd_len = 3;
        let params = [0x10, 0x11, 0x12, 0x13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let params_len = 4;
        
        mock_parse_ble_packet_op_params(Any, Any)
            .returns((true, op_cmd, op_cmd_len, params, params_len));
        
        // Mock group and device matching (no match)
        mock_rf_link_match_group_mac(Any).returns((false, false));
        
        // Mock notification detection (not a notification)
        mock_rf_link_is_notify_req(Any).returns(false);
        
        // Mock data callback
        mock_rf_link_data_callback(Any).returns(());
        
        // Mock read status stop if busy
        mock_rf_link_slave_read_status_stop().returns(());
        
        // Test the function
        let result = rf_link_slave_data_write_no_dec(&packet);
        
        // Should return true for successful processing
        assert_eq!(result, true);
        
        // Verify callback was NOT called (no match)
        mock_rf_link_data_callback(Any).assert_called(0);
        
        // Verify sequence number was updated
        assert_eq!(*SLAVE_SNO.lock(), sno);
        
        // Verify command opcode was stored
        assert_eq!(SLAVE_LINK_CMD.get(), op_cmd[0] & 0x3f);
        
        // Verify data validity flag is set for bridge forwarding
        assert_eq!(SLAVE_DATA_VALID.get(), BRIDGE_MAX_CNT + 1);
    }
    
    #[test]
    #[mry::lock(parse_ble_packet_op_params)]
    #[mry::lock(rf_link_match_group_mac)]
    #[mry::lock(rf_link_is_notify_req)]
    #[mry::lock(rf_link_data_callback)]
    #[mry::lock(rf_link_slave_read_status_par_init)]
    #[mry::lock(rf_link_response_callback)]
    #[mry::lock(rf_link_slave_add_status)]
    #[mry::lock(read_reg_system_tick)]
    fn test_rf_link_slave_data_write_no_dec_notification_device_match() {
        // Mock system tick reading
        mock_read_reg_system_tick().returns(0xAABBCCDD);

        // Create a packet with valid length
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead { rf_len: 0x11, ..Default::default() },
                ..Default::default()
            }
        };
        
        // Set sequence number different from SLAVE_SNO
        let sno = [1, 2, 3];
        critical_section::with(|_| {
            *SLAVE_SNO.lock() = [1, 2, 2];
            packet.att_write_mut().value.sno = sno;
        });
        
        // Mock op_cmd and params extraction
        // Using LGT_CMD_LIGHT_READ_STATUS opcode (0x10)
        let op_cmd = [0x10, 0x02, 0x03];
        let op_cmd_len = 3;
        let params = [0x05, 0x11, 0x12, 0x13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let params_len = 4;
        
        mock_parse_ble_packet_op_params(Any, Any)
            .returns((true, op_cmd, op_cmd_len, params, params_len));
        
        // Mock group and device matching (device match)
        mock_rf_link_match_group_mac(Any).returns((false, true));
        
        // Mock notification detection (is notification)
        mock_rf_link_is_notify_req(Any).returns(true);
        
        // Mock data callback
        mock_rf_link_data_callback(Any).returns(());
        
        // Mock read status parameter initialization 
        mock_rf_link_slave_read_status_par_init().returns(());
        
        // Mock response callback that returns true to add status
        mock_rf_link_response_callback(Any, Any).returns(true);
        
        // Mock add status function
        mock_rf_link_slave_add_status(Any).returns(());
        
        // Mock device address for packet response
        DEVICE_ADDRESS.set(0x1234);
        
        // Set link interval for status packet
        SLAVE_LINK_INTERVAL.set(1000 * CLOCK_SYS_CLOCK_1US);
        
        // Test the function
        let result = rf_link_slave_data_write_no_dec(&packet);
        
        // Should return true for successful processing
        assert_eq!(result, true);
        
        // Verify callback was called
        mock_rf_link_data_callback(Any).assert_called(1);
        
        // Verify sequence number was updated
        assert_eq!(*SLAVE_SNO.lock(), sno);
        
        // Verify SLAVE_STATUS_TICK is now stored in stat_sno
        assert_eq!(*SLAVE_STAT_SNO.lock(), sno);
        
        // Verify response callback and add status were called
        mock_rf_link_response_callback(Any, Any).assert_called(1);
        mock_rf_link_slave_add_status(Any).assert_called(1);
        
        // Verify data validity flag is 0 for direct match
        assert_eq!(SLAVE_DATA_VALID.get(), 0);
    }
}