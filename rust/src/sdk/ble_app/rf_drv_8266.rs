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
use crate::sdk::ble_app::light_ll::{rf_link_get_op_para, rf_link_is_notify_req, rf_link_match_group_mac, rf_link_slave_add_status, rf_link_slave_read_status_par_init, rf_link_slave_read_status_stop};
use crate::sdk::common::compat::{array4_to_int, load_tbl_cmd_set, TBLCMDSET};
use crate::sdk::common::crc::crc16;
use crate::sdk::drivers::flash::{flash_read_page, flash_write_page};
use crate::sdk::light::{*};
use crate::sdk::mcu::analog::{analog_read, analog_write};
use crate::sdk::mcu::clock::CLOCK_SYS_CLOCK_1US;
use crate::sdk::mcu::crypto::encode_password;
use crate::sdk::mcu::irq_i::irq_disable;
use crate::sdk::mcu::random::rand;
use crate::sdk::mcu::register::{FLD_RF_IRQ_MASK, read_reg_irq_mask, read_reg_rf_mode, read_reg_system_tick, read_reg_system_tick_mode, write_reg16, write_reg32, write_reg8, write_reg_dma2_addr, write_reg_dma2_ctrl, write_reg_dma3_addr, write_reg_dma_chn_irq_msk, write_reg_irq_mask, write_reg_irq_src, write_reg_rf_access_code, write_reg_rf_crc, write_reg_rf_irq_mask, write_reg_rf_irq_status, write_reg_rf_mode, write_reg_rf_mode_control, write_reg_rf_sched_tick, write_reg_rf_sn, write_reg_system_tick_irq, write_reg_system_tick_mode};
use crate::sdk::mcu::register::{rega_deepsleep_flag, write_reg_rf_rx_gain_agc};
use crate::sdk::packet_types::{Packet, PacketAttCmd, PacketAttData, PacketAttValue};
use crate::sdk::pm::light_sw_reboot;
use crate::state::{*};
use crate::version::BUILD_VERSION;

const RF_FAST_MODE: bool = true;
const RF_TRX_MODE: u8 = 0x80;
const RF_TRX_OFF: u8 = 0x45;

static TBL_AGC: [u32; 7] = [0x30333231, 0x182C3C38, 0xC0C1C, 0x0, 0x1B150F0A, 0x322E2721, 0x3E38];

const TBL_RF_INI: [TBLCMDSET; 61] = [
    TBLCMDSET {
        adr: 0x5b4,
        dat: 0x02,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x474,
        dat: 0x08,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x85,
        dat: 0x00,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x80,
        dat: 0x61,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x06,
        dat: 0x00,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x8f,
        dat: 0x30,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x81,
        dat: 0xd8,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x8f,
        dat: 0x38,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x8b,
        dat: 0xe3,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x8e,
        dat: 0x6b,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x8d,
        dat: 0x67,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x402,
        dat: 0x26,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x9e,
        dat: 0xad,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0xa0,
        dat: 0x28,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0xa2,
        dat: 0x2c,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0xa3,
        dat: 0x10,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0xac,
        dat: 0xa7,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0xaa,
        dat: 0x2e,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x439,
        dat: 0x72,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x400,
        dat: 0x0b,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x42b,
        dat: 0xf3,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x43b,
        dat: 0xfc,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x74f,
        dat: 0x01,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0xf04,
        dat: 0x50,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0xf06,
        dat: 0x00,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0xf0c,
        dat: 0x50,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0xf10,
        dat: 0x00,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x400,
        dat: 0x0f,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x42b,
        dat: 0xf1,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x420,
        dat: 0x20,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x421,
        dat: 0x04,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x422,
        dat: 0x00,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x424,
        dat: 0x12,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x464,
        dat: 0x07,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x4cd,
        dat: 0x04,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x9e,
        dat: 0x56,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0xa3,
        dat: 0xf0,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0xaa,
        dat: 0x26,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x404,
        dat: 0xf5,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x408,
        dat: 0x8e,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x409,
        dat: 0x89,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x40a,
        dat: 0xbe,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x40b,
        dat: 0xd6,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x401,
        dat: 0x08,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x430,
        dat: 0x12,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x43d,
        dat: 0x71,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x402,
        dat: 0x24,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0xf04,
        dat: 0x68,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x42c,
        dat: 0x30,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x4ca,
        dat: 0x88,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x4cb,
        dat: 0x04,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x42d,
        dat: 0x33,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x433,
        dat: 0x00,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x434,
        dat: 0x01,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x43a,
        dat: 0x77,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x43e,
        dat: 0xc9,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x4cd,
        dat: 0x06,
        cmd: 0xc3,
    },

    // Enable these for BLE to work with 16mhz xtal
    TBLCMDSET {
        adr: 0x4eb,
        dat: 0x60,
        cmd: 0xc3,
    },
    TBLCMDSET {     // gauss filter sel: 16M
        adr: 0x99,
        dat: 0x31,
        cmd: 0xc8,
    },
    TBLCMDSET {     //enable rxadc clock
        adr: 0x82,
        dat: 0x34,
        cmd: 0xc8,
    },
    TBLCMDSET {     //reg_dc_mod (500K)
        adr: 0x9e,
        dat: 0x41,
        cmd: 0xc8,
    }
];

/**
 * Initialize the RF driver with specified settings
 * 
 * This function sets up the RF hardware by:
 * 1. Loading initialization command sets for configuring the RF registers
 * 2. Setting up RF gain values for Automatic Gain Control (AGC)
 * 3. Configuring transmission power parameters based on values stored in flash memory
 *
 * The RF initialization values depend on whether a 16MHz crystal is used (controlled by feature flag).
 * 
 * @param enable - Whether to enable the RF module (true) or power it off (false)
 * @return The deepsleep flag value for recovery information (0x40 bit indicates sleep state)
 */
pub fn rf_drv_init(enable: bool) -> u8
{
    let result = analog_read(rega_deepsleep_flag) & 0x40;

    // This function is not complete
    critical_section::with(|_| {
        if enable {
            // Load initialization values to configure the RF hardware registers
            // Different register values for 16MHz xtal configuration versus default
            #[cfg(feature = "xtal-16mhz")]
            load_tbl_cmd_set(TBL_RF_INI.as_ptr(), TBL_RF_INI.len() as u32);

            #[cfg(not(feature = "xtal-16mhz"))]
            load_tbl_cmd_set(TBL_RF_INI.as_ptr(), TBL_RF_INI.len() as u32 - 4);

            // Configure the Automatic Gain Control (AGC) settings
            // Each value corresponds to different RF gain stages
            for i in 0..6 {
                write_reg_rf_rx_gain_agc(TBL_AGC[i], (i << 2) as u32)
            }

            unsafe {
                // Check if custom RF power settings are stored at specific flash memory offsets
                // These calibration values adjust the transmission power parameters
                if *(FLASH_ADR_MAC as *const u8).offset(0x11) != 0xff {
                    let u_var5 = *(FLASH_ADR_MAC as *const u8).offset(0x11) as u32;
                    RF_TP_BASE.set(u_var5);
                    RF_TP_GAIN.set(((u_var5 - 0x19) << 8) / 80);
                }
                if *(FLASH_ADR_MAC as *const u8).offset(0x12) != 0xff {
                    let u_var5 = RF_TP_BASE.get() - *(FLASH_ADR_MAC as *const u8).offset(0x12) as u32;
                    RF_TP_GAIN.set((u_var5 << 8) / 80);
                }
            }
        } else {
            analog_write(6, 0);  // power off sar (Successive Approximation Register ADC)
        }
    });

    return result;
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
    write_reg8(0x800428, RF_TRX_MODE | BIT!(0));  // rx enable
    write_reg8(0x800f02, RF_TRX_OFF | BIT!(5));   // RX enable
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
    write_reg8(0x800428, RF_TRX_MODE);           // rx disable
    write_reg8(0x800f02, RF_TRX_OFF);            // reset tx/rx state machine
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
pub unsafe fn blc_ll_init_basic_mcu()
{
    // Configure system tick interval
    write_reg16(0xf0a, 700);
    
    // Set up DMA address for packet reception
    write_reg_dma2_addr(addr_of!(LIGHT_RX_BUFF.lock()[LIGHT_RX_WPTR.get()]) as u16);

    // Configure DMA2 control register for packet reception
    write_reg_dma2_ctrl(0x104);
    write_reg_dma_chn_irq_msk(0);

    // Enable interrupt handling for RF operations
    write_reg_irq_mask(read_reg_irq_mask() | 0x2000);
    write_reg_system_tick_mode(read_reg_system_tick_mode() | 2);

    // Clear interrupt flags and reset RF interrupt status
    write_reg16(0xf1c, 0);
    write_reg_rf_irq_status(0xfffe);

    // Configure RF interrupts for RX and TX events
    write_reg_rf_irq_mask(FLD_RF_IRQ_MASK::IRQ_RX as u16 | FLD_RF_IRQ_MASK::IRQ_TX as u16);

    // Set up system tick interrupt for timing BLE events
    write_reg_system_tick_irq(read_reg_system_tick() | (0x80 << 0x18));
    write_reg_irq_src(0x80 << 0xd);
    write_reg_irq_mask(read_reg_irq_mask() | (0x80 << 0xd));

    // Set additional timing configuration
    write_reg16(0xf2c, 0xc00);
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
 * Initialize and prepare for a status read response
 * 
 * This function prepares the BLE device for responding to status read requests by:
 * 1. Storing the current system tick for timeout tracking
 * 2. Determining the appropriate response type based on the request parameters
 * 3. Setting flags for unicast vs broadcast responses
 * 4. Initializing response buffer data structures
 * 5. Clearing any previous status records
 * 
 * Status reads are used in the mesh network to poll device state information.
 * 
 * @param pkt_light_data - Pointer to the packet data structure containing the request details
 */
fn rf_link_slave_read_status_start(pkt_light_data: &mut Packet)
{
    // Record the time when the status read operation started
    SLAVE_READ_STATUS_BUSY_TIME.set(read_reg_system_tick());
    
    // Determine the appropriate response type based on the opcode and parameter
    SLAVE_READ_STATUS_BUSY.set(rf_link_get_rsp_type(pkt_light_data.att_cmd_mut().value.val[0] & 0x3f, pkt_light_data.att_cmd_mut().value.val[4]));
    
    // Check if this is a unicast (directed to a specific device) or broadcast request
    SLAVE_READ_STATUS_UNICAST_FLAG.set(!pkt_light_data.att_cmd_mut().value.dst[1] >> 7);
    
    // For non-broadcast packets, clear fields for device-specific information
    if -1 < (!((pkt_light_data.att_cmd_mut().value.dst[1] as u32) << 0x18)) as i32 {
       pkt_light_data.att_cmd_mut().value.val[8..8 + 4].fill(0);
       pkt_light_data.att_cmd_mut().value.val[12] = SLAVE_READ_STATUS_UNICAST_FLAG.get();
    }
    
    // Special handling for opcode 0x1a (likely a specific status request type)
    if pkt_light_data.att_cmd_mut().value.val[0] & 0x3f == 0x1a {
       pkt_light_data.att_cmd_mut().value.val[4..8 + 4].fill(0);
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
 * Process a received BLE packet without performing decryption
 * 
 * This function handles incoming packet data by:
 * 1. Validating packet length and sequence number to prevent duplicate processing
 * 2. Extracting operation commands and parameters from the packet
 * 3. Determining if the packet is addressed to this device or its group
 * 4. Processing device address configuration and light control commands
 * 5. Setting up appropriate response packets with status information
 * 6. Triggering status read operations when notification requests are received
 * 
 * This is a core function in the mesh network that manages command processing
 * and state transitions based on received instructions.
 * 
 * @param data - The received packet data
 * @return true if the packet was processed successfully, false otherwise
 */
fn rf_link_slave_data_write_no_dec(data: &Packet) -> bool {
    // Validate packet length
    if data.head().rf_len < 0x11 {
        return false;
    }

    // Check sequence number to prevent duplicate packet processing
    let sno = &data.att_write().value[0..3];
    if sno == *SLAVE_SNO.lock() {
        return true;  // Already processed this packet - return success without reprocessing
    }

    // Extract operation command and parameters from the packet
    let mut op_cmd: [u8; 3] = [0; 3];
    let mut op_cmd_len: u8 = 0;
    let mut params: [u8; 16] = [0; 16];
    let mut params_len: u8 = 0;
    rf_link_get_op_para(
        data,
        &mut op_cmd,
        &mut op_cmd_len,
        &mut params,
        &mut params_len,
        false,
    );

    // Check if the packet matches this device's address or group
    let (group_match, device_match) = rf_link_match_group_mac(data);
    let op;
    let mut tmp = data.att_write().value[6] as u32;
    
    // Extract operation code from command
    if op_cmd_len == 3 {
        op = op_cmd[0] & 0x3f;
    } else {
        op = 0;
    }

    // Special handling for device address configuration command
    if op == LGT_CMD_CONFIG_DEV_ADDR && ((tmp * 0x1000000) as i32) < 0 {
        // Validate configuration parameters
        if params[0] != 0xff {
            return false;
        }
        if params[1] != 0xff {
            return false;
        }
        tmp = tmp << 8 | data.att_write().value[5] as u32;
    } else {
        // Process other commands
        tmp = tmp << 8 | data.att_write().value[5] as u32;
        // Special handling for ONOFF commands with specific addressing
        if ((!tmp << 0x10) as i32) < 0 && op == LGT_CMD_LIGHT_ONOFF {
            if tmp == 0 {
                tmp = DEVICE_ADDRESS.get() as u32;
            }
            // Check for forced notifications
            uprintln!("stub: mesh_node_check_force_notify")
            // mesh_node_check_force_notify(uVar4, params[0]);
        }
    }

    // Prepare response packet buffers
    let mut pkt_light_status = PKT_LIGHT_STATUS.lock();
    let mut pkt_light_data = PKT_LIGHT_DATA.lock();

    // Reset packet data structures for new response
    pkt_light_data.att_cmd_mut().value = PacketAttValue::default();
    pkt_light_status.att_cmd_mut().value = PacketAttValue::default();

    // Copy received packet data to response packet
    unsafe {
        slice::from_raw_parts_mut(
            addr_of_mut!(*pkt_light_data) as *mut u8,
            params_len as usize + 0x11,
        ).copy_from_slice(
            slice::from_raw_parts(
                addr_of!(*data) as *const u8,
                params_len as usize + 0x11,
            )
        )
    }

    // Set up header fields for response packet
   pkt_light_data.head_mut().chan_id = 0xff03;   // Channel ID for mesh network
   pkt_light_data.head_mut().dma_len = 0x27;     // Length for DMA transfer
   pkt_light_data.head_mut().rf_len = 0x25;      // RF packet length
   pkt_light_data.head_mut().l2cap_len = 0x21;   // L2CAP length field
   
    // Set this device's address in response packet
    unsafe { *(addr_of_mut!(pkt_light_data.att_cmd_mut().opcode) as *mut u16) = DEVICE_ADDRESS.get() };
    unsafe { *(addr_of_mut!(pkt_light_data.att_cmd_mut().value.src) as *mut u16) = DEVICE_ADDRESS.get() };

    // Process packet data if it matches this device or its group
    if device_match || group_match {
        rf_link_data_callback(&*pkt_light_data);
    }
    
    // Store sequence number to prevent duplicate processing
    SLAVE_SNO.lock().copy_from_slice(sno);

    // Record the command operation for reference
    SLAVE_LINK_CMD.set(op);

    // For non-notification requests, stop any ongoing status read operation
    if !rf_link_is_notify_req(op) {
        if SLAVE_READ_STATUS_BUSY.get() != 0 {
            rf_link_slave_read_status_stop();
        }
        
        // Set validity flag based on addressing
        if device_match == false && unsafe { *(addr_of!(data.att_write().value[5]) as *const u16) } != 0 {
            SLAVE_DATA_VALID.set(BRIDGE_MAX_CNT + 1);
        } else {
            SLAVE_DATA_VALID.set(0);
        }
        return true;
    }

    // Initialize status response fields
   pkt_light_status.att_cmd_mut().value.val[13] = 0;
   pkt_light_status.att_cmd_mut().value.val[14] = 0;
   
   // Set connection interval in response
   pkt_light_data.att_cmd_mut().value.val[16] = (SLAVE_LINK_INTERVAL.get() / (CLOCK_SYS_CLOCK_1US * 1000)) as u8;
    
    // Complex handling of different commands based on addressing and operation type
    // This section configures the response data flags and validity timing
    if device_match == false || (tmp != 0 && op == LGT_CMD_CONFIG_DEV_ADDR && dev_addr_with_mac_flag(&params)) {
        // Handle specific commands with parameters
        if op == LGT_CMD_LIGHT_GRP_REQ || op == LGT_CMD_LIGHT_READ_STATUS || op == LGT_CMD_USER_NOTIFY_REQ {
            // Set data validity timer based on parameter
            SLAVE_DATA_VALID.set(params[0] as u32 * 2 + 1);
            
            // Set response flags based on operation type
            if op == LGT_CMD_LIGHT_GRP_REQ {
               pkt_light_data.att_cmd_mut().value.val[15] = params[1];
            } else if op == LGT_CMD_LIGHT_CONFIG_GRP {
               pkt_light_data.att_cmd_mut().value.val[15] = 1;
            } else {
                // Additional flag setting for specific operations
                if op == LGT_CMD_CONFIG_DEV_ADDR {
                   pkt_light_data.att_cmd_mut().value.val[15] = 4;
                }
                if op == LGT_CMD_USER_NOTIFY_REQ {
                   pkt_light_data.att_cmd_mut().value.val[15] = 7;
                } else {
                   pkt_light_data.att_cmd_mut().value.val[15] = 0;
                }
            }
        } else if op != LGT_CMD_CONFIG_DEV_ADDR {
            // Process other operations
            if op != LGT_CMD_LIGHT_CONFIG_GRP {
                // Flag setting for various operation types
                if op == LGT_CMD_LIGHT_GRP_REQ {
                   pkt_light_data.att_cmd_mut().value.val[15] = params[1];
                } else if op == LGT_CMD_LIGHT_CONFIG_GRP {
                   pkt_light_data.att_cmd_mut().value.val[15] = 1;
                } else {
                    if op == LGT_CMD_CONFIG_DEV_ADDR {
                       pkt_light_data.att_cmd_mut().value.val[15] = 4;
                    }
                    if op == LGT_CMD_USER_NOTIFY_REQ {
                       pkt_light_data.att_cmd_mut().value.val[15] = 7;
                    } else {
                       pkt_light_data.att_cmd_mut().value.val[15] = 0;
                    }
                }
            }
            // Set validity for bridge operations
            SLAVE_DATA_VALID.set(BRIDGE_MAX_CNT * 2 + 1);
           pkt_light_data.att_cmd_mut().value.val[15] = 1;
        } else if dev_addr_with_mac_flag(&params) == false {
            // Handle device address configuration without MAC flag
            SLAVE_DATA_VALID.set(BRIDGE_MAX_CNT * 2 + 1);
           pkt_light_data.att_cmd_mut().value.val[15] = 4;
        } else {
            // Handle device address configuration with MAC flag
            SLAVE_DATA_VALID.set(params[3] as u32 * 2 + 1);
           pkt_light_data.att_cmd_mut().value.val[15] = 4;
        }
    } else {
        // Default handling for direct commands
        SLAVE_DATA_VALID.set(0);
        
        // Set response flags based on operation type
        if op == LGT_CMD_LIGHT_GRP_REQ {
           pkt_light_data.att_cmd_mut().value.val[15] = params[1];
        } else if op == LGT_CMD_LIGHT_CONFIG_GRP {
           pkt_light_data.att_cmd_mut().value.val[15] = 1;
        } else {
            if op == LGT_CMD_CONFIG_DEV_ADDR {
               pkt_light_data.att_cmd_mut().value.val[15] = 4;
            }
            if op == LGT_CMD_USER_NOTIFY_REQ {
               pkt_light_data.att_cmd_mut().value.val[15] = 7;
            } else {
               pkt_light_data.att_cmd_mut().value.val[15] = 0;
            }
        }
    }
    
    // Copy the flag to status packet
    pkt_light_status.att_cmd_mut().value.val[15] = pkt_light_data.att_cmd_mut().value.val[15];
    
    // Start status read response preparation
    rf_link_slave_read_status_start(pkt_light_data.deref_mut());

    // Store sequence number for status responses
    SLAVE_STAT_SNO.lock().copy_from_slice(sno);

    // Generate actual response for matched devices
    if device_match || group_match {
        // Copy packet data to status response
        let ptr = &pkt_light_data.att_cmd_mut().value.val[3..];
       pkt_light_status.att_cmd_mut().value.val[3..3 + ptr.len()].copy_from_slice(&ptr);

       pkt_light_status.att_cmd_mut().value.sno = *SLAVE_SNO.lock();

        unsafe { *(addr_of!(pkt_light_status.att_cmd_mut().value.src) as *mut u16) = DEVICE_ADDRESS.get() };

        // Create temporary packet for response callback
        let tmp_pkt: PacketAttValue = PacketAttValue::default();

        unsafe {
            slice::from_raw_parts_mut(
                addr_of!(tmp_pkt) as *mut u8,
                0x1e,
            ).copy_from_slice(&data.att_write().value);
        }

        unsafe {
            *(addr_of!(tmp_pkt.src) as *mut u16) = DEVICE_ADDRESS.get();
        }

        // Process response through callback and add to status if successful
        if rf_link_response_callback(&mut pkt_light_status.att_cmd_mut().value, &tmp_pkt) {
            rf_link_slave_add_status(&*pkt_light_status);
        }
    }

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
 * RF power control table structure
 * Contains analog register values to configure RF power settings
 * Each entry defines values for four different analog registers that together control transmission power
 */
#[repr(C, packed)]
struct TblRfPowerT {
    pub a: u8,  // Value for analog register 0xa2
    pub b: u8,  // Value for analog register 0x4
    pub c: u8,  // Value for analog register 0xa7
    pub d: u8,  // Value for analog register 0x8d
}

// Table of predefined RF power settings in descending order from highest to lowest power
// Each entry corresponds to a specific power level the transmitter can operate at
static TBL_RF_POWER: [TblRfPowerT; 12] = [
    TblRfPowerT {
        a: 0x25,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x0a,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x06,
        b: 0x74,
        c: 0x43,
        d: 0x61,
    },
    TblRfPowerT {
        a: 0x06,
        b: 0x64,
        c: 0xc2,
        d: 0x61,
    },
    TblRfPowerT {
        a: 0x06,
        b: 0x64,
        c: 0xc1,
        d: 0x61,
    },
    TblRfPowerT {
        a: 0x05,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x03,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x02,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x01,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x00,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x00,
        b: 0x64,
        c: 0x43,
        d: 0x61,
    },
    TblRfPowerT {
        a: 0x00,
        b: 0x64,
        c: 0xcb,
        d: 0x61,
    }
];

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
    write_reg8(0x40d, chn);

    // Determine channel type and calculate parameters
    let params = calculate_channel_params(chn);

    // Power off the SAR (Successive Approximation Register) ADC
    analog_write(6, 0);

    // Turn on LDO and baseband PLL
    write_reg_rf_mode(0x29);

    // Disable receiver and reset TX/RX state machine
    write_reg8(0x800428, RF_TRX_MODE);        // rx disable
    write_reg8(0x800f02, RF_TRX_OFF);         // reset tx/rx state machine

    // Configure the frequency for auto TX
    write_reg16(0x8004d6, params.intgn);     // Write integral frequency value

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
    // Calculate the target output power based on the channel gain and global calibration values
    // The output is subtracted from the base value to adjust for frequency-specific losses
    unsafe { analog_write(0x93, RF_TP_BASE.get() as u8 - ((gain as u32 * RF_TP_GAIN.get() + 0x80) >> 8) as u8); }
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

    use crate::sdk::mcu::register::*;
    use crate::sdk::mcu::analog::*;

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
    #[mry::lock(write_reg8)]
    #[mry::lock(write_reg16)]
    #[mry::lock(write_reg32)]
    #[mry::lock(read_reg8)]
    fn test_rf_start_brx() {
        mock_read_reg8(0xf16).returns(3);
        mock_write_reg32(0xf28, Any).returns(());
        mock_write_reg32(0xf18, Any).returns(());
        mock_write_reg8(0xf16, Any).returns(());
        mock_write_reg8(0xf00, Any).returns(());
        mock_write_reg16(0x50c, Any).returns(());

        rf_start_brx(0x12345678, 0x87654321);

        mock_write_reg32(0xf28, 0xffffffff).assert_called(1);
        mock_write_reg32(0xf18, 0x87654321).assert_called(1);
        mock_write_reg8(0xf16, 4 | 3).assert_called(1);
        mock_write_reg8(0xf00, 0x82).assert_called(1);
        mock_write_reg16(0x50c, 0x5678).assert_called(1);
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
    #[mry::lock(write_reg16)]
    #[mry::lock(write_reg_rf_mode)]
    #[mry::lock(analog_write)]
    fn test_rf_set_ble_channel_data_channels() {
        // Mock all register writes
        mock_write_reg8(0x40d, Any).returns(());
        mock_write_reg_rf_mode(Any).returns(());
        mock_write_reg8(0x800428, Any).returns(());
        mock_write_reg8(0x800f02, Any).returns(());
        mock_write_reg16(0x8004d6, Any).returns(());
        mock_analog_write(6, Any).returns(());
        mock_analog_write(0x93, Any).returns(());
        
        // Test data channel (0-10)
        rf_set_ble_channel(5);
        mock_write_reg8(0x40d, 5).assert_called(1);
        // Channel 5 should result in gain = (5+2)*2 = 14, intgn = 14+2400 = 2414
        mock_write_reg16(0x8004d6, 2414).assert_called(1);
        // SAR ADC should be powered off
        mock_analog_write(6, 0).assert_called(1);
        // RF mode should be set to 0x29
        mock_write_reg_rf_mode(0x29).assert_called(1);
        // RX should be disabled
        mock_write_reg8(0x800428, RF_TRX_MODE).assert_called(1);
        // TX/RX state machine should be reset
        mock_write_reg8(0x800f02, RF_TRX_OFF).assert_called(1);
        
        // Test data channel (11-36)
        rf_set_ble_channel(20);
        mock_write_reg8(0x40d, 20).assert_called(1);
        // Channel 20 should result in gain = (20+3)*2 = 46, intgn = 46+2400 = 2446
        mock_write_reg16(0x8004d6, 2446).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg8)]
    #[mry::lock(write_reg16)]
    #[mry::lock(write_reg_rf_mode)]
    #[mry::lock(analog_write)]
    fn test_rf_set_ble_channel_adv_channels() {
        // Mock all register writes
        mock_write_reg8(0x40d, Any).returns(());
        mock_write_reg_rf_mode(Any).returns(());
        mock_write_reg8(0x800428, Any).returns(());
        mock_write_reg8(0x800f02, Any).returns(());
        mock_write_reg16(0x8004d6, Any).returns(());
        mock_analog_write(6, Any).returns(());
        mock_analog_write(0x93, Any).returns(());
        
        // Test advertising channels
        // Channel 37 (2402 MHz)
        rf_set_ble_channel(37);
        mock_write_reg8(0x40d, 37).assert_called(1);
        mock_write_reg16(0x8004d6, 2402).assert_called(1);
        
        // Channel 38 (2426 MHz)
        rf_set_ble_channel(38);
        mock_write_reg8(0x40d, 38).assert_called(1);
        mock_write_reg16(0x8004d6, 2426).assert_called(1);
        
        // Channel 39 (2480 MHz)
        rf_set_ble_channel(39);
        mock_write_reg8(0x40d, 39).assert_called(1);
        mock_write_reg16(0x8004d6, 2480).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg8)]
    #[mry::lock(write_reg16)]
    #[mry::lock(write_reg_rf_mode)]
    #[mry::lock(analog_write)]
    fn test_rf_set_ble_channel_extended_40_50() {
        // Mock all register writes
        mock_write_reg8(0x40d, Any).returns(());
        mock_write_reg_rf_mode(Any).returns(());
        mock_write_reg8(0x800428, Any).returns(());
        mock_write_reg8(0x800f02, Any).returns(());
        mock_write_reg16(0x8004d6, Any).returns(());
        mock_analog_write(6, Any).returns(());
        mock_analog_write(0x93, Any).returns(());
        
        // Test extended channel (40-50)
        rf_set_ble_channel(45);
        mock_write_reg8(0x40d, 45).assert_called(1);
        // Channel 45 should result in gain = 45*2 = 90, intgn = 90+2400 = 2490
        mock_write_reg16(0x8004d6, 2490).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg8)]
    #[mry::lock(write_reg16)]
    #[mry::lock(write_reg_rf_mode)]
    #[mry::lock(analog_write)]
    fn test_rf_set_ble_channel_extended_51_61() {
        // Mock all register writes
        mock_write_reg8(0x40d, Any).returns(());
        mock_write_reg_rf_mode(Any).returns(());
        mock_write_reg8(0x800428, Any).returns(());
        mock_write_reg8(0x800f02, Any).returns(());
        mock_write_reg16(0x8004d6, Any).returns(());
        mock_analog_write(6, Any).returns(());
        mock_analog_write(0x93, Any).returns(());
        
        // Test extended channel (51-61) with our new calculation
        // For channel 55:
        // gain = (61-55)*2 = 12
        // intgn = 2408 - gain = 2408 - 12 = 2396
        rf_set_ble_channel(55);
        mock_write_reg8(0x40d, 55).assert_called(1);
        mock_write_reg16(0x8004d6, 2396).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg8)]
    #[mry::lock(write_reg16)]
    #[mry::lock(write_reg_rf_mode)]
    #[mry::lock(analog_write)]
    fn test_rf_set_ble_channel_out_of_range() {
        // Mock all register writes
        mock_write_reg8(0x40d, Any).returns(());
        mock_write_reg_rf_mode(Any).returns(());
        mock_write_reg8(0x800428, Any).returns(());
        mock_write_reg8(0x800f02, Any).returns(());
        mock_write_reg16(0x8004d6, Any).returns(());
        mock_analog_write(6, Any).returns(());
        mock_analog_write(0x93, Any).returns(());
        
        // Test out of range channel (> 61)
        rf_set_ble_channel(62);
        mock_write_reg8(0x40d, 62).assert_called(1);
        // Out of range channel should default to 0x50 and 2480
        mock_write_reg16(0x8004d6, 2480).assert_called(1);
    }
}