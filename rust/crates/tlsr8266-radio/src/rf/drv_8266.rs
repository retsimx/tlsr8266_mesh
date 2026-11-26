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

use core::mem::size_of;
use core::ops::DerefMut;
use core::ptr::addr_of;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU16, AtomicU32, AtomicUsize};

use crate::ble::connection::{
    BLE_PERIPHERAL_LINK_COMMAND, BLE_PERIPHERAL_LINK_STATE, BUFF_RESPONSE, SLAVE_CONNECTED_TICK,
    SLAVE_DATA_VALID, SLAVE_LINK_INTERVAL,
};
use crate::ble::flash_config::{dev_addr_with_mac_flag, pair_load_key, retrieve_dev_grp_address};
use crate::ble::pair::{pair_dec_packet, PAIR_AC, PAIR_LOGIN_OK};
use crate::ble::status::{
    rf_link_slave_read_status_par_init, rf_link_slave_read_status_stop,
    DEVICE_STATUS_READ_BUSY_TIMESTAMP, DEVICE_STATUS_READ_UNICAST_MODE, DEVICE_STATUS_RECORD_INDEX,
    DEVICE_STATUS_TICK_COUNTER, NOTIFICATION_REQUEST_MASK_INDEX, SLAVE_READ_STATUS_BUSY,
    SLAVE_STATUS_RECORD,
};
use crate::callbacks::{on_light_data, on_light_response};
use crate::config::radio_config;
use crate::mesh_ll::node_status::{
    mesh_node_table_init, rf_link_match_group_mac, GENERAL_MESSAGE_SEQUENCE_NUMBER,
    MESH_LISTEN_INTERVAL_US, PKT_LIGHT_DATA, PKT_LIGHT_STATUS, STATUS_MESSAGE_SEQUENCE_NUMBER,
};
use crate::mesh_ll::packet_processing::{
    parse_ble_packet_op_params, rf_link_is_notify_req, rf_link_slave_add_status,
};
use crate::rf::tables::{TBL_AGC, TBL_RF_INI, TBL_RF_POWER};
use crate::types::{
    LightRxBuff, RfOperationState, StatusRecord, BRIDGE_MAX_CNT, GET_DEV_ADDR, GET_GROUP1,
    GET_GROUP2, GET_GROUP3, GET_STATUS, GET_USER_NOTIFY, LGT_CMD_CONFIG_DEV_ADDR,
    LGT_CMD_DEV_ADDR_RSP, LGT_CMD_LIGHT_CONFIG_GRP, LGT_CMD_LIGHT_GRP_REQ, LGT_CMD_LIGHT_GRP_RSP1,
    LGT_CMD_LIGHT_GRP_RSP2, LGT_CMD_LIGHT_GRP_RSP3, LGT_CMD_LIGHT_ONOFF, LGT_CMD_LIGHT_READ_STATUS,
    LGT_CMD_LIGHT_STATUS, LGT_CMD_USER_NOTIFY_REQ, LGT_CMD_USER_NOTIFY_RSP, LIGHT_RX_BUFF_COUNT,
    MAX_GROUP_COUNT, PKT_VAL_MAC_APP_HOP_COUNT, PKT_VAL_MAC_APP_LINK_INTERVAL,
    PKT_VAL_MAC_APP_RESPONSE_TYPE, PKT_VAL_MAC_APP_TTC,
};
use mesh_protocol::const_assert;
use mesh_protocol::crc::crc16;
use mesh_protocol::mesh_types::{
    LGT_CMD_END_OTA_REQ, LGT_CMD_END_OTA_RSP, LGT_CMD_OTA_DATA_REQ, LGT_CMD_OTA_DATA_RSP,
};
use mesh_protocol::packet_types::{
    AdvPrivate, AdvRspPrivate, Packet, PacketAttData, PacketAttErrRsp, PacketAttValue,
    PacketCtrlUnknown, PacketL2capHead, PacketLlInit, RfPacketAdvIndModuleT,
};
use tlsr8266_hal::common::sync::{CriticalSectionMutex, SimplifyLS};
use tlsr8266_hal::drivers::flash::{flash_read_page, flash_write_page};
use tlsr8266_hal::mcu::analog::{analog_read, analog_write};
use tlsr8266_hal::mcu::clock::{is_16mhz_xtal, CLOCK_SYS_CLOCK_1US};
use tlsr8266_hal::mcu::cmd_table::{load_tbl_cmd_set, TBLCMDSET};
use tlsr8266_hal::mcu::random::rand;
use tlsr8266_pac::BIT;
use tlsr8266_pac::{
    read_reg_irq_mask, read_reg_rf_mode, read_reg_system_tick, read_reg_system_tick_mode,
    write_reg16, write_reg32, write_reg_dma2_addr, write_reg_dma2_ctrl, write_reg_dma3_addr,
    write_reg_dma_chn_irq_msk, write_reg_irq_mask, write_reg_irq_src,
    write_reg_pll_rx_fine_div_tune, write_reg_rf_access_code, write_reg_rf_channel,
    write_reg_rf_crc, write_reg_rf_event_clear, write_reg_rf_irq_mask, write_reg_rf_irq_status,
    write_reg_rf_mode, write_reg_rf_mode_control, write_reg_rf_sched_tick, write_reg_rf_sn,
    write_reg_rf_sys_timer_config, write_reg_rf_timing_config, write_reg_rf_tx_wail_settle_time_lo,
    write_reg_rf_txrx_state, write_reg_system_tick_irq, write_reg_system_tick_mode,
    FLD_RF_IRQ_MASK, FLD_RF_RX_MODE, REGA_RF_OUTPUT_MATCH, REGA_RF_OUTPUT_POWER, REGA_RF_PA_BIAS,
    REGA_RF_PA_GAIN, REGA_RF_TX_POWER, REGA_SAR_ADC_PWR,
};
use tlsr8266_pac::{rega_deepsleep_flag, write_reg_rf_rx_gain_agc, write_reg_rf_rx_mode};

const RF_TRX_OFF: u8 = 0x45;

// Firmware SRAM scratch offsets inside `__RAM_START_ADDR = 0x808000`, written by
// `rf_link_slave_init` and read back by the host tooling. These are RAM
// addresses, not PAC MMIO registers: they already carry the `REG_BASE_ADDR`
// bit, so the `write_reg*` accessors' base OR is a no-op for them. They stay
// local to this module.
const SRAM_IDENT_MAC: u32 = 0x808004;
const SRAM_IDENT_VERSION: u32 = 0x808008;
const SRAM_IDENT_CRC: u32 = 0x80800c;

/// Private advertising data structure used in BLE advertisements.
/// Contains manufacturer ID, mesh product UUID, and MAC address for device identification.
/// Broadcast during BLE advertising to allow discovery by mesh controllers.
///
/// The vendor id starts at the factory value and is overwritten by
/// [`apply_vendor_id`] when the injected configuration is installed.
///
/// Visibility is `pub(crate)`: the facade module is the only entry point from
/// outside this crate, and the crate boundary is the fence C-10 established.
pub(crate) static ADV_PRI_DATA: CriticalSectionMutex<AdvPrivate> =
    CriticalSectionMutex::new(AdvPrivate {
        manufacture_id: 0x0211,
        mesh_product_uuid: 0x0211,
        mac_address: 0,
    });

/// Private advertising response data sent in response to scan requests.
/// Extended version of advertising data including device status, product UUID,
/// device address, and additional reserved fields for future use.
pub(crate) static ADV_RSP_PRI_DATA: CriticalSectionMutex<AdvRspPrivate> =
    CriticalSectionMutex::new(AdvRspPrivate {
        manufacture_id: 0x0211,
        mesh_product_uuid: 0x0211,
        mac_address: 0,
        product_uuid: 0x1234,
        status: 0x01,
        device_address: 0,
        rsv: [0; 16],
    });

/// Propagate the configured vendor id into both private advertising payloads.
///
/// Called by `set_radio_config`; both the manufacturer id and the mesh product
/// UUID must carry the vendor id.
pub(crate) fn apply_vendor_id(vendor_id: u16) {
    {
        let mut adv = ADV_PRI_DATA.lock();
        adv.manufacture_id = vendor_id;
        adv.mesh_product_uuid = vendor_id;
    }
    {
        let mut adv = ADV_RSP_PRI_DATA.lock();
        adv.manufacture_id = vendor_id;
        adv.mesh_product_uuid = vendor_id;
    }
}

/// Current RF operation state indicating which mode is active.
/// Tracks whether the device is in advertising, connected, receiving, mesh listening, or idle state.
pub static CURRENT_RF_STATE: CriticalSectionMutex<RfOperationState> =
    CriticalSectionMutex::new(RfOperationState::Idle);

/// Group addresses that this device belongs to (up to 8 groups).
/// Used for group-based lighting control where multiple devices respond to the same command.
pub(crate) static GROUP_ADDRESS: CriticalSectionMutex<[u16; MAX_GROUP_COUNT as usize]> =
    CriticalSectionMutex::new([0; MAX_GROUP_COUNT as usize]);

/// Device MAC address used for network identification.
/// 6-byte IEEE MAC address uniquely identifying this device in the mesh network.
pub(crate) static MAC_ID: CriticalSectionMutex<[u8; 6]> = CriticalSectionMutex::new([0; 6]);

/// BLE advertising data payload.
/// Contains flags and service information broadcast during BLE advertising.
pub const ADV_DATA_INIT: [u8; 3] = [2, 1, 5];
pub(crate) static ADV_DATA: CriticalSectionMutex<[u8; ADV_DATA_INIT.len()]> =
    CriticalSectionMutex::new(ADV_DATA_INIT);
/// Advertising packet structure used for BLE advertisements.
/// Pre-configured packet with advertising data, device address, and payload.
/// Transmitted during advertising intervals to announce device presence.
pub static PKT_ADV: CriticalSectionMutex<Packet> = CriticalSectionMutex::new(Packet {
    adv_ind_module: RfPacketAdvIndModuleT {
        dma_len: 0x27,
        _type: 0,
        rf_len: 0x25,
        adv_a: [0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5],
        data: [0; 31],
    },
});

/// BLE connection initialization packet template.
/// Contains connection parameters, access address, channel map, and timing information.
/// Used when establishing BLE connections with central devices.
pub(crate) static PKT_INIT: CriticalSectionMutex<Packet> = CriticalSectionMutex::new(Packet {
    ll_init: PacketLlInit {
        dma_len: 0x24,
        _type: 0x5,
        rf_len: 0x22,
        scan_a: [0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5],
        adv_a: [0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5],
        aa: [0xaa, 0x55, 0x55, 0xaa],
        crcinit: [0x55, 0x55, 0x55],
        wsize: 2,
        woffset: 0x1f,
        interval: 0x20,
        latency: 0,
        timeout: 0x48,
        chm: [0xff, 0xff, 0xff, 0xff, 0x1f],
        hop: 0xac,
    },
});

/// Receive buffer array for incoming light control packets.
/// Contains 4 buffers storing received packet data including RSSI, timing, sequence numbers,
/// and MAC addresses. Used for processing incoming mesh communications.
pub(crate) static LIGHT_RX_BUFF: CriticalSectionMutex<[LightRxBuff; LIGHT_RX_BUFF_COUNT]> =
    CriticalSectionMutex::new(
        [LightRxBuff {
            dma_len: 0,
            unk1: [0; 3],
            rssi: 0,
            unk2: [0; 3],
            rx_time: 0,
            sno: [0; 3],
            unk3: [0; 5],
            mac: [0; 4],
            unk4: [0; 40],
        }; LIGHT_RX_BUFF_COUNT],
    );

/// Empty packet template used for BLE layer control.
/// Must be in RAM (.data section) because DMA cannot access flash memory.
/// Used for link layer control and empty data transmissions.
// This needs to be forced in to .data otherwise the DMA module will try to fetch from flash, which
// doesn't work
#[link_section = ".data"]
pub(crate) static PKT_EMPTY: Packet = Packet {
    head: PacketL2capHead {
        dma_len: 2,
        _type: 1,
        rf_len: 0,
        l2cap_len: 0,
        chan_id: 0,
    },
};

/// BLE connection termination packet.
/// Must be in RAM (.data section) for DMA access. Used to terminate active BLE connections
/// by sending the appropriate control packet with termination opcode.
#[link_section = ".data"]
pub(crate) static PKT_TERMINATE: Packet = Packet {
    ctrl_unknown: PacketCtrlUnknown {
        dma_len: 4,
        _type: 3,
        rf_len: 2,
        opcode: 2,
        data: [0x13],
    },
};

/// ATT error response packet template.
/// Must be in RAM (.data section) for DMA access. Used to send error responses
/// when ATT operations fail, containing error codes and handles.
#[link_section = ".data"]
pub(crate) static PKT_ERR_RSP: Packet = Packet {
    att_err_rsp: PacketAttErrRsp {
        head: PacketL2capHead {
            dma_len: 0x0b,
            _type: 0x02,
            rf_len: 0x09,
            l2cap_len: 0x05,
            chan_id: 0x04,
        },
        opcode: 0x01,
        err_opcode: 0,
        err_handle: 0,
        err_reason: 0x0a,
    },
};

/// Write pointer for the light receive buffer circular queue.
/// Points to the next available slot for storing incoming light control packets.
pub static LIGHT_RX_BUFFER_WRITE_POINTER: AtomicUsize = AtomicUsize::new(0);

/// Unique 16-bit address assigned to this device within the mesh network.
/// Used for addressing and routing messages to/from this specific device.
pub(crate) static DEVICE_ADDRESS: AtomicU16 = AtomicU16::new(0);

/// RF transmit power base setting controlling signal strength.
/// Hardware register value determining the baseline transmission power level.
static RF_TP_BASE: AtomicU32 = AtomicU32::new(0x1D);

/// RF transmit power gain adjustment for fine-tuning signal strength.
/// Additional gain control applied on top of the base power setting.
static RF_TP_GAIN: AtomicU32 = AtomicU32::new(0xC);

/// BLE scan response interval timing control.
/// Determines the delay between receiving scan requests and sending responses.
pub static BLE_SCAN_RESPONSE_INTERVAL_US: AtomicU32 = AtomicU32::new(0x92);

/// Flag indicating UUID configuration has been set.
/// Tracks whether device UUID has been properly configured.
pub(crate) static SET_UUID_FLAG: AtomicBool = AtomicBool::new(false);

/// Maximum allowed length for mesh network names.
/// Enforces string length limits for mesh configuration.
const ADV_PRIVATE_SIZE: usize = ADV_DATA_INIT.len() + size_of::<AdvPrivate>() + 2;
const_assert!(ADV_PRIVATE_SIZE < 31);

pub const MAX_MESH_NAME_LEN: usize = if 31 - ADV_PRIVATE_SIZE - 2 < 16 {
    31 - ADV_PRIVATE_SIZE - 2
} else {
    16
};

/// Flag indicating mesh device address configuration is pending MAC validation.
/// When true: Device address configuration requires MAC address validation before completion.
/// When false: MAC address validation completed successfully, device address is configured.
/// Used during mesh pairing to track the device address configuration state.
pub(crate) static MESH_DEVICE_ADDRESS_VALIDATION_PENDING: AtomicBool = AtomicBool::new(false);

/// Next position index for device group address allocation.
/// Points to the next available slot in the group address table.
pub(crate) static MESH_GROUP_ADDRESS_NEXT_POSITION: AtomicU16 = AtomicU16::new(0);

/// Flash configuration index for persistent storage operations.
/// Current position in flash memory for configuration data.
pub(crate) static FLASH_CONFIGURATION_INDEX: AtomicI32 = AtomicI32::new(0);

/// Next position for device address allocation in mesh network.
/// Used for assigning unique addresses to new mesh devices.
pub(crate) static MESH_DEVICE_ADDRESS_NEXT_POSITION: AtomicU16 = AtomicU16::new(0);

/// Flag controlling BLE advertising state.
/// When true, device actively advertises its presence for discovery.
pub(crate) static BLE_ADVERTISING_ENABLED: AtomicBool = AtomicBool::new(true);

/// Flag enabling BLE peripheral advertising functionality.
/// Controls whether the peripheral device can advertise for BLE connections.
pub static BLE_PERIPHERAL_ADVERTISING_ENABLED: AtomicBool = AtomicBool::new(false);

/**
 * Select the RF initialization table for the crystal detected at boot
 *
 * Returns the whole table for a 16MHz crystal, and the table without its last
 * four commands for the default 12MHz crystal. The final four commands
 * (0x4eb, analog 0x99, analog 0x82, analog 0x9e) retune the PLL calibration
 * enable, the gauss filter, the RX ADC clock and the DC modulator for a 16MHz
 * crystal; they are wrong values on a 12MHz part, so they stay unloaded there.
 *
 * The selection lives in this helper so it is a pure function a test can
 * assert on directly: `load_tbl_cmd_set` walks the table by writing registers,
 * so its effect is observable but the slice it was handed never is.
 *
 * @param is_16mhz - Whether a 16MHz crystal was detected at boot
 * @return The RF initialization command table to load
 */
fn rf_init_table(is_16mhz: bool) -> &'static [TBLCMDSET] {
    if is_16mhz {
        &TBL_RF_INI
    } else {
        &TBL_RF_INI[..TBL_RF_INI.len() - 4]
    }
}

/**
 * Initialize the RF driver with specified settings
 *
 * This function sets up the RF hardware by:
 * 1. Loading initialization command sets for configuring the RF registers
 * 2. Setting up RF gain values for Automatic Gain Control (AGC)
 * 3. Configuring transmission power parameters based on values stored in flash memory
 *
 * The RF initialization table is chosen from the crystal measured at boot: a 16MHz
 * crystal loads the full table, the default 12MHz crystal omits the four commands
 * that only apply to a 16MHz part (see rf_init_table).
 * If transmission power calibration values are found in flash memory, they will be applied to
 * RF_TP_BASE and RF_TP_GAIN parameters which control output power level.
 *
 * @param enable - Whether to enable the RF module (true) or power it off (false)
 * @return The deepsleep flag value for recovery information (0x40 bit indicates sleep state)
 */
#[coverage(off)]
pub fn rf_drv_init(enable: bool) -> u8 {
    let result = analog_read(rega_deepsleep_flag) & 0x40;
    // Snapshot the configuration before entering the critical section.
    let cfg = radio_config();

    critical_section::with(|_| {
        if enable {
            // Load initialization values to configure the RF hardware registers.
            // The table is selected from the crystal detected at boot, because the
            // final four entries only apply to a 16MHz crystal.
            load_tbl_cmd_set(rf_init_table(is_16mhz_xtal()));

            // Configure the Automatic Gain Control (AGC) settings
            // Each value corresponds to different RF gain stages
            for (i, &gain) in TBL_AGC.iter().take(6).enumerate() {
                write_reg_rf_rx_gain_agc(gain, (i << 2) as u32)
            }

            // Check if custom RF power settings are stored at specific flash memory offsets
            // These calibration values adjust the transmission power parameters
            //
            // Erased defaults: a failed read leaves no calibration values to apply,
            // rather than a zeroed (and plausible) set read from a dead bus.
            let mut power_settings = [0xFFu8; 0x13];
            // The MAC sector is guaranteed to be aligned to FLASH_SECTOR_SIZE
            let _ = flash_read_page(
                cfg.flash_adr_mac,
                power_settings.len() as u32,
                power_settings.as_mut_ptr(),
            );

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
            // When RF module is disabled, power off the SAR ADC (REGA_SAR_ADC_PWR).
            // Note: 0x06 is ambiguous — project docs say SAR ADC power, Telink
            // 8267/8258 headers call it the PLL bandgap power-down.
            analog_write(REGA_SAR_ADC_PWR, 0);
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
    write_reg_rf_mode_control(0x80); // stop
}

/**
 * Set the RF module into receive mode
 *
 * This function configures the RF hardware to actively listen for incoming packets by:
 * 1. Enabling the RF receiver in the RF_TRX_MODE register
 * 2. Setting the RF state machine to receive mode
 */
pub fn rf_set_rxmode() {
    write_reg_rf_rx_mode((FLD_RF_RX_MODE::LOW_PASS_FILTER | FLD_RF_RX_MODE::EN).bits()); // rx enable
    write_reg_rf_txrx_state(RF_TRX_OFF | BIT!(5)); // RX enable
}

/**
 * Disable both RF transmitter and receiver
 *
 * This function turns off both the transmitter and receiver circuits by:
 * 1. Setting the RF mode to a specific power configuration (0x29)
 * 2. Disabling the RF receiver
 * 3. Resetting the RF state machine
 */
pub fn rf_set_tx_rx_off() {
    // reg_rf_mode (0xf16): TX_EN_DLY_EN (bit0) | R_TX_EN_DLY=2 (bits7:4) | bit3 (0x08) undocumented
    write_reg_rf_mode(0x29);
    write_reg_rf_rx_mode(FLD_RF_RX_MODE::LOW_PASS_FILTER.bits()); // rx disable (replacing write_reg8(0x800428, RF_TRX_MODE))
    write_reg_rf_txrx_state(RF_TRX_OFF); // reset tx/rx state machine
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
pub fn blc_ll_init_basic_mcu() {
    // Configure system tick interval
    write_reg_rf_sys_timer_config(700);

    // Set up DMA address for packet reception
    write_reg_dma2_addr(addr_of!(LIGHT_RX_BUFF.lock()[LIGHT_RX_BUFFER_WRITE_POINTER.get()]) as u16);

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
 * Clear the RF receive interrupt status bit
 *
 * Acknowledges the RX event by writing only `IRQ_RX` to `reg_rf_irq_status`
 * (0xf20), which is write-1-to-clear. Callers use this after they have consumed
 * the packet sitting in the RX buffer, so a stale RX status cannot be mistaken
 * for a new packet.
 */
pub fn rf_clear_rx_irq() {
    write_reg_rf_irq_status(FLD_RF_IRQ_MASK::IRQ_RX.bits())
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
fn rf_link_slave_read_status_start(pkt_light_data: &mut Packet) {
    // Record the time when the status read operation started
    DEVICE_STATUS_READ_BUSY_TIMESTAMP.set(read_reg_system_tick());

    // Extract opcode and parameter for determining response type
    let opcode = pkt_light_data.att_cmd().value.val[0] & 0x3f;
    let param = pkt_light_data.att_cmd().value.val[4];

    // Determine the appropriate response type based on the opcode and parameter
    SLAVE_READ_STATUS_BUSY.set(rf_link_get_rsp_type(opcode, param));

    // Check if this is a unicast (directed to a specific device) or broadcast request
    let is_unicast = (pkt_light_data.att_cmd().value.dst[1] & 0x80) == 0;
    DEVICE_STATUS_READ_UNICAST_MODE.set(is_unicast);

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
        pkt_light_data.att_cmd_mut().value.val[3] = DEVICE_STATUS_TICK_COUNTER.get();
    }

    // Initialize parameters for status response handling
    rf_link_slave_read_status_par_init();

    // Clear the response buffer for fresh status data
    BUFF_RESPONSE.lock().fill(Packet {
        att_data: PacketAttData::default(),
    });

    // Reset status record tracking indices
    DEVICE_STATUS_RECORD_INDEX.set(0);
    SLAVE_STATUS_RECORD.lock().fill(StatusRecord {
        adr: [0; 1],
        alarm_id: 0,
    });

    NOTIFICATION_REQUEST_MASK_INDEX.set(0);
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
fn rf_link_slave_data_write_no_dec(data: &Packet) {
    // Validate minimum packet size requirement
    // RF length must be at least 0x11 bytes for valid command packets
    if data.head().rf_len < 0x11 {
        return;
    }

    // Extract the sequence number from the packet
    // The sequence number is used to detect duplicate packets and prevent reprocessing
    let sno = data.att_write().value.sno;

    // Check if this is a duplicate packet (same sequence number as the last one)
    // Return true for duplicates to avoid reprocessing the same command
    if sno == *GENERAL_MESSAGE_SEQUENCE_NUMBER.lock() {
        return;
    }

    // Extract operation command and parameters from the packet using the refactored function
    let (_success, op_cmd, op_cmd_len, params, _params_len) =
        parse_ble_packet_op_params(data, false);

    // Determine if this packet targets this device by checking:
    // 1. If it matches one of the device's configured group addresses (group_match)
    // 2. If it directly targets this device's address (device_match)
    let (group_match, device_match) = rf_link_match_group_mac(data);

    // Extract the operation code (opcode) and destination address information
    let op = if op_cmd_len == 3 {
        op_cmd[0] & 0x3f // Extract the lower 6 bits as the opcode (masking out flag bits)
    } else {
        0 // Default opcode if no valid command found
    };

    // Extract destination address from the packet
    // The destination high byte is at index 6 in the value array
    let mut dst_addr = data.att_write().value.dst[1] as u32;

    // Special handling for device address configuration command targeting broadcast address
    // The high bit of destination address indicates a special broadcast configuration
    if op == LGT_CMD_CONFIG_DEV_ADDR && (dst_addr & 0x80) != 0 {
        // Validate mandatory parameter values for broadcast device address configuration
        // Parameters 0 and 1 must be 0xFF for this command type
        if params[0] != 0xff || params[1] != 0xff {
            return;
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
    // First clone the packet header which has the same structure
    *pkt_light_data = *data;

    // Configure the BLE packet headers for the response
    // These values define the packet format according to the BLE specification
    pkt_light_data.head_mut().chan_id = 0xff03; // Vendor-specific channel ID
    pkt_light_data.head_mut().dma_len = 0x27; // DMA buffer length
    pkt_light_data.head_mut().rf_len = 0x25; // RF packet length
    pkt_light_data.head_mut().l2cap_len = 0x21; // L2CAP layer length

    // Set the source device address in the packet using safe method
    // This ensures the recipient knows which device sent the response
    let device_addr = DEVICE_ADDRESS.get();
    pkt_light_data.att_cmd_mut().opcode = device_addr as u8;
    pkt_light_data.att_cmd_mut().value.src[0] = (device_addr & 0xFF) as u8;
    pkt_light_data.att_cmd_mut().value.src[1] = ((device_addr >> 8) & 0xFF) as u8;

    // Process the command data if this packet is intended for this device
    // (either directly targeted or matches one of our group addresses)
    if device_match || group_match {
        on_light_data(&pkt_light_data);
    }

    // Update the stored sequence number with the one from this packet
    // This prevents duplicate processing if the same packet is received again
    *GENERAL_MESSAGE_SEQUENCE_NUMBER.lock() = sno;

    // Store the command opcode for potential future reference
    BLE_PERIPHERAL_LINK_COMMAND.set(op);

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
            SLAVE_DATA_VALID.set(BRIDGE_MAX_CNT + 1); // Needs bridge forwarding
        } else {
            SLAVE_DATA_VALID.set(0); // No bridge forwarding needed
        }
        return;
    }

    // For notification request commands, prepare the status response
    // Initialize status packet fields for the response
    pkt_light_status.att_cmd_mut().value.val[PKT_VAL_MAC_APP_TTC] = 0;
    pkt_light_status.att_cmd_mut().value.val[PKT_VAL_MAC_APP_HOP_COUNT] = 0;

    // Store the link interval in the packet (converted from microseconds to milliseconds)
    pkt_light_data.att_cmd_mut().value.val[PKT_VAL_MAC_APP_LINK_INTERVAL] =
        (SLAVE_LINK_INTERVAL.get() / (CLOCK_SYS_CLOCK_1US * 1000)) as u8;

    // Helper function to determine the appropriate response type for a command
    let get_response_type = |cmd_op: u8, cmd_params: &[u8]| -> u8 {
        match cmd_op {
            LGT_CMD_LIGHT_GRP_REQ => cmd_params[1],
            LGT_CMD_LIGHT_CONFIG_GRP => GET_GROUP1,
            LGT_CMD_CONFIG_DEV_ADDR => GET_DEV_ADDR,
            LGT_CMD_USER_NOTIFY_REQ => GET_USER_NOTIFY,
            _ => GET_STATUS,
        }
    };

    // Determine response timing and type based on device match and command characteristics
    let has_mac_flag = dev_addr_with_mac_flag(&params);
    let is_special_addr_config = dst_addr != 0 && op == LGT_CMD_CONFIG_DEV_ADDR && has_mac_flag;

    // Calculate timing delay and response type
    let (timing, response_type) = if device_match && !is_special_addr_config {
        // Direct device match: respond immediately with standard response type
        (0, get_response_type(op, &params))
    } else if matches!(
        op,
        LGT_CMD_LIGHT_GRP_REQ | LGT_CMD_LIGHT_READ_STATUS | LGT_CMD_USER_NOTIFY_REQ
    ) {
        // Commands with parameter-based timing: use params[0] for slot assignment
        (params[0] as u32 * 2 + 1, get_response_type(op, &params))
    } else if op == LGT_CMD_CONFIG_DEV_ADDR {
        // Device address configuration: timing depends on MAC flag presence
        if has_mac_flag {
            // MAC-based config uses params[3] for slot assignment
            (params[3] as u32 * 2 + 1, GET_DEV_ADDR)
        } else {
            // Standard address config uses maximum bridge delay
            (BRIDGE_MAX_CNT * 2 + 1, GET_DEV_ADDR)
        }
    } else {
        // All other non-direct commands: use maximum bridge delay
        (BRIDGE_MAX_CNT * 2 + 1, GET_GROUP1)
    };

    // Apply the calculated timing and response type
    SLAVE_DATA_VALID.set(timing);
    pkt_light_data.att_cmd_mut().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE] = response_type;
    pkt_light_status.att_cmd_mut().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE] = response_type;

    // Initialize the status read response operation
    // This sets up all necessary state for generating the status response
    rf_link_slave_read_status_start(pkt_light_data.deref_mut());

    // Store the sequence number for status tracking
    *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() = sno;

    // Process the response if this packet is for this device
    if device_match || group_match {
        // Copy command data to the status packet
        let ptr = &pkt_light_data.att_cmd_mut().value.val[3..];
        pkt_light_status.att_cmd_mut().value.val[3..3 + ptr.len()].copy_from_slice(ptr);

        // Set the sequence number in the status packet
        pkt_light_status.att_cmd_mut().value.sno = *GENERAL_MESSAGE_SEQUENCE_NUMBER.lock();

        // Set this device's address as the source in the status packet using safe methods
        // Instead of raw pointer casting
        let device_addr = DEVICE_ADDRESS.get();
        pkt_light_status.att_cmd_mut().value.src[0] = (device_addr & 0xFF) as u8;
        pkt_light_status.att_cmd_mut().value.src[1] = ((device_addr >> 8) & 0xFF) as u8;

        // Create a temporary packet for passing to the response callback
        // Copy the original packet data to the temporary packet
        let mut tmp_pkt = data.att_write().value;

        // Set this device's address as the source in the temporary packet
        let device_addr = DEVICE_ADDRESS.get();
        tmp_pkt.src[0] = (device_addr & 0xFF) as u8;
        tmp_pkt.src[1] = ((device_addr >> 8) & 0xFF) as u8;

        // Call the response callback to fill in application-specific status data
        // If the callback returns true, add the status packet to the response queue
        if on_light_response(&mut pkt_light_status.att_cmd_mut().value, &tmp_pkt).is_ok() {
            rf_link_slave_add_status(&pkt_light_status);
        }
    }

    // Return true to indicate successful packet processing
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
pub fn rf_link_slave_data_write(data: &Packet) {
    let mut data: Packet = *data;

    // Check if security is enabled and attempt packet decryption
    if PAIR_LOGIN_OK.get() && pair_dec_packet(&mut data).is_ok() {
        // Process the decrypted packet
        rf_link_slave_data_write_no_dec(&data);
    }
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
fn rf_link_slave_set_adv(adv_data_ptr: &[u8]) {
    let mut pkt_adv = PKT_ADV.lock();

    // Calculate the total packet length including headers
    pkt_adv.head_mut().dma_len = adv_data_ptr.len() as u32 + 8;
    pkt_adv.head_mut().rf_len = adv_data_ptr.len() as u8 + 6;

    // Copy the advertisement data to the packet
    pkt_adv.adv_ind_module_mut().data[0..adv_data_ptr.len()]
        .copy_from_slice(&adv_data_ptr[0..adv_data_ptr.len()]);
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
pub fn rf_link_slave_init(interval: u32) {
    // One snapshot for the whole init path; the struct is `Copy`, so no
    // configuration lock is held while the radio is set up.
    let cfg = radio_config();

    // Initialize the basic MCU link layer functionality
    blc_ll_init_basic_mcu();

    // Set up interrupt handler status for advertisement mode
    *CURRENT_RF_STATE.lock() = RfOperationState::Advertising;

    // Initialize link state and timing parameters
    BLE_PERIPHERAL_LINK_STATE.set(crate::types::BlePeripheralLinkState::Disconnected);
    MESH_LISTEN_INTERVAL_US.set(interval * CLOCK_SYS_CLOCK_1US);

    // Configure connection timer tick
    SLAVE_CONNECTED_TICK.set(CLOCK_SYS_CLOCK_1US * 100000 + read_reg_system_tick());
    write_reg_system_tick_irq(SLAVE_CONNECTED_TICK.get());

    // Set RF hardware configuration
    write_reg_rf_tx_wail_settle_time_lo(0x68);

    // Generate a MAC address if one doesn't exist in flash. A failed read is
    // treated like the blank slot: a zeroed buffer must not pass for a stored
    // address.
    let mut mac_check: [u32; 1] = [0];
    if flash_read_page(cfg.flash_adr_mac, 4, mac_check.as_mut_ptr() as *mut u8).is_err()
        || mac_check[0] == u32::MAX
    {
        // Generate random MAC address values. A failed program leaves the
        // flash erased, so the next boot takes this same branch again.
        let mac: [u16; 2] = [rand(), rand()];
        let _ = flash_write_page(cfg.flash_adr_mac, 4, mac.as_ptr() as *const u8);
    }

    // Read the MAC address from flash. A failed read leaves the previous (or
    // zeroed) MAC in place; there is no boot-time channel to report it.
    let _ = flash_read_page(cfg.flash_adr_mac, 6, MAC_ID.lock().as_mut_ptr());

    // Set the MAC address in the advertisement packet
    PKT_ADV.lock().adv_ind_module_mut().adv_a = *MAC_ID.lock();

    // Configure the advertisement data
    rf_link_slave_set_adv(&ADV_DATA.lock().clone());

    // Load encryption keys for secure communication
    pair_load_key();

    let access_code = PAIR_AC.get();

    // Immediately configure RF hardware with new access code
    // This ensures the hardware is ready for mesh listening before interrupts fire
    rf_set_ble_access_code(access_code);

    // Set device MAC address in packet templates for responses
    PKT_LIGHT_DATA.lock().att_cmd_mut().value.src[0..2].copy_from_slice(&MAC_ID.lock()[0..2]);
    PKT_LIGHT_STATUS.lock().att_cmd_mut().value.src[0..2].copy_from_slice(&MAC_ID.lock()[0..2]);

    // Enable advertising
    BLE_PERIPHERAL_ADVERTISING_ENABLED.set(true);

    // Load device group addresses from flash memory
    retrieve_dev_grp_address();

    // Initialize mesh network node functionality
    mesh_node_table_init();

    // Write device identification info to system memory
    // This includes MAC address, firmware version, and data integrity check
    let mac_id = MAC_ID.lock();
    let mac_as_u32 = u32::from_le_bytes([mac_id[0], mac_id[1], mac_id[2], mac_id[3]]);
    let build_version = cfg.build_version;
    write_reg32(SRAM_IDENT_MAC, mac_as_u32);
    write_reg32(SRAM_IDENT_VERSION, build_version);
    let crc_data = [
        (mac_as_u32 & 0xFF) as u8,
        ((mac_as_u32 >> 8) & 0xFF) as u8,
        ((mac_as_u32 >> 16) & 0xFF) as u8,
        ((mac_as_u32 >> 24) & 0xFF) as u8,
        (build_version & 0xFF) as u8,
        ((build_version >> 8) & 0xFF) as u8,
        ((build_version >> 16) & 0xFF) as u8,
        ((build_version >> 24) & 0xFF) as u8,
    ];
    write_reg16(SRAM_IDENT_CRC, crc16(&crc_data));
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
pub fn rf_set_power_level_index(index: u32) {
    // Default to the lowest power setting (index 11) if out of range
    let tbl_index = if index < TBL_RF_POWER.len() as u32 {
        index as usize
    } else {
        11 // Use the lowest power setting if out of range
    };

    // Configure the four analog registers that control RF power. `tbl_index` is
    // bounded by the `TBL_RF_POWER.len()` check above, so the table reads are in
    // bounds, and `analog_write` is a safe `fn`.
    analog_write(REGA_RF_PA_GAIN, TBL_RF_POWER[tbl_index].a); // PA gain control
    analog_write(REGA_RF_PA_BIAS, TBL_RF_POWER[tbl_index].b); // PA bias control
    analog_write(REGA_RF_OUTPUT_MATCH, TBL_RF_POWER[tbl_index].c); // RF output match network
    analog_write(REGA_RF_OUTPUT_POWER, TBL_RF_POWER[tbl_index].d); // RF output power
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
    gain: u8,   // RF gain value
    intgn: u16, // Integral frequency in MHz
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

    // Power off the SAR ADC (REGA_SAR_ADC_PWR). Note: 0x06 is ambiguous —
    // project docs say SAR ADC, Telink 8267/8258 headers say PLL bandgap.
    analog_write(REGA_SAR_ADC_PWR, 0);

    // Turn on LDO and baseband PLL
    // reg_rf_mode (0xf16): TX_EN_DLY_EN (bit0) | R_TX_EN_DLY=2 (bits7:4) | bit3 (0x08) undocumented
    write_reg_rf_mode(0x29);

    // Disable receiver and reset TX/RX state machine
    write_reg_rf_rx_mode(0); // rx disable
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
        _ => BleChannelType::OutOfRange,
    };

    // Calculate gain and frequency based on channel type
    match channel_type {
        BleChannelType::DataLow => {
            // Channels 0-10: gain = (chn + 2) * 2, intgn = gain + 2400
            let gain = (chn + 2) * 2;
            ChannelParams {
                gain,
                intgn: gain as u16 + 2400,
            }
        }
        BleChannelType::DataHigh => {
            // Channels 11-36: gain = (chn + 3) * 2, intgn = gain + 2400
            let gain = (chn + 3) * 2;
            ChannelParams {
                gain,
                intgn: gain as u16 + 2400,
            }
        }
        BleChannelType::AdvPrimary => {
            // Primary advertising channels have specific values
            match chn {
                37 => ChannelParams {
                    gain: 2,
                    intgn: 2402,
                },
                38 => ChannelParams {
                    gain: 0x1a,
                    intgn: 2426,
                },
                _ => ChannelParams {
                    gain: 0x50,
                    intgn: 2480,
                }, // Channel 39
            }
        }
        BleChannelType::ExtLow => {
            // Extended channels 40-50: gain = chn * 2, intgn = gain + 2400
            let gain = chn * 2;
            ChannelParams {
                gain,
                intgn: gain as u16 + 2400,
            }
        }
        BleChannelType::ExtHigh => {
            // Extended channels 51-61 have reverse mapping
            // gain = (61 - chn) * 2
            let gain = (61 - chn) * 2;
            ChannelParams {
                gain,
                // Frequency: 2388 MHz at chn=51 down to 2368 MHz at chn=61
                // Simplified calculation that works for all gain values
                intgn: 2408 - gain as u16,
            }
        }
        BleChannelType::OutOfRange => {
            // Default to maximum frequency for out of range channels
            ChannelParams {
                gain: 0x50,
                intgn: 2480,
            }
        }
    }
}

/**
 * Configure transmission power based on channel-specific gain value
 *
 * This function adjusts the RF transmission power for a specific channel by:
 * 1. Calculating an appropriate analog register value from the base and gain parameters
 * 2. Writing this value to analog register REGA_RF_TX_POWER (0x93), which controls
 *    output power
 *
 * The calculation uses RF_TP_BASE and RF_TP_GAIN global variables that are
 * calibrated during initialization based on values stored in flash.
 *
 * @param gain - Channel-specific gain value calculated for the selected frequency
 */
fn rf_set_tp_gain(gain: u8) {
    // Calculate power adjustment based on channel gain
    let power_adjustment = ((gain as u32 * RF_TP_GAIN.get() + 0x80) >> 8) as u8;

    // Apply adjustment by subtracting from base power value
    // Handle potential overflow by ensuring power_adjustment doesn't exceed RF_TP_BASE
    let base_value = RF_TP_BASE.get() as u8;
    let power_value = base_value.saturating_sub(power_adjustment);

    // Write final power value to analog register REGA_RF_TX_POWER (0x93, TX power control)
    analog_write(REGA_RF_TX_POWER, power_value);
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
pub fn rf_start_srx2tx(addr: u32, tick: u32) {
    write_reg_rf_sched_tick(tick); // Setting schedule trigger time
    write_reg_rf_mode(read_reg_rf_mode() | 0x04); // Enable cmd_schedule mode
    write_reg_rf_mode_control(0x85); // Set mode to srx2tx
    write_reg_dma3_addr(addr as u16); // Set DMA address for the packet
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
pub fn rf_start_stx2rx(addr: u32, tick: u32) {
    write_reg_rf_sched_tick(tick); // Setting schedule trigger time
    write_reg_rf_mode(read_reg_rf_mode() | 0x04); // Enable cmd_schedule mode
    write_reg_rf_mode_control(0x87); // Set mode to stx2rx
    write_reg_dma3_addr(addr as u16); // Set DMA address for the packet
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
pub fn rf_start_brx(addr: u32, tick: u32) {
    write_reg_rf_event_clear(0xffffffff); // Clear pending events
    write_reg_rf_sched_tick(tick); // Setting schedule trigger time
    write_reg_rf_mode(read_reg_rf_mode() | 0x04); // Enable cmd_schedule mode
    write_reg_rf_mode_control(0x82); // Set mode to brx
    write_reg_dma3_addr(addr as u16); // Set DMA address for the packet
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
pub fn rf_reset_sn() {
    write_reg_rf_sn(0x3f); // Write specific reset value
    write_reg_rf_sn(0x00); // Initialize to zero
}

/**
 * Set the BLE CRC initialization value
 *
 * Configures the 24-bit CRC initialization value for BLE packet validation.
 * This value is used by the RF hardware to verify packet integrity.
 *
 * @param crc - Array of 3 bytes representing the CRC initialization value
 */
pub fn rf_set_ble_crc(crc: &[u8]) {
    // Combine the 3 bytes into a single 24-bit value and write to the CRC register
    write_reg_rf_crc(crc[0] as u32 | ((crc[1] as u32) << 8) | ((crc[2] as u32) << 16));
}

/**
 * Set the default BLE CRC initialization value for advertising channels
 *
 * Configures the RF hardware with the standard CRC initialization value (0x555555)
 * that is used for BLE advertising packets according to the specification.
 */
pub fn rf_set_ble_crc_adv() {
    write_reg_rf_crc(0x555555); // Standard BLE advertising CRC init value
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
pub fn rf_set_ble_access_code(ac: u32) {
    write_reg_rf_access_code(ac.swap_bytes()); // Swap byte order for hardware compatibility
}

/**
 * Set the default BLE access code for advertising channels
 *
 * Configures the RF hardware with the standard access code (0xd6be898e)
 * that is used for BLE advertising packets according to the specification.
 */
pub fn rf_set_ble_access_code_adv() {
    write_reg_rf_access_code(0xd6be898e); // Standard BLE advertising access code
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::ble::pair::{pair_init, PAIR_STATE};
    use crate::ble::status::{
        DEVICE_STATUS_BUFFER_READ_POINTER, DEVICE_STATUS_BUFFER_WRITE_POINTER,
    };
    use crate::callbacks::{set_ble_app_hooks, BleAppHooks};
    use crate::config::set_radio_config;
    use crate::rf::tables::{TBL_AGC, TBL_RF_INI};
    use core::sync::atomic::Ordering;
    use mesh_protocol::mesh_types::DEV_ADDR_PAR_WITH_MAC;
    use mesh_protocol::packet_types::Packet;
    use mesh_protocol::packet_types::{
        PacketAttCmd, PacketAttWrite, PacketL2capHead, RfPacketAdvIndModuleT,
    };
    use tlsr8266_hal::common::sync::SimplifyLS;
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog;
    use tlsr8266_hal::mcu::clock::fake_clock;
    use tlsr8266_hal::mcu::crypto::aes_att_encryption_packet;
    use tlsr8266_pac::fake_mmio::{FakeMmio, RegWrite};
    use tlsr8266_pac::*;

    /// Reset the PAC/HAL host fakes so a test never inherits another test's
    /// register bank, analog bank, clock or AES transport (the suite runs
    /// `--test-threads=1`, so all of these are process-wide singletons).
    /// The NOR device is reset too: it stores what production code writes.
    fn reset_host_fakes() {
        FakeMmio::reset();
        FakeAnalog::reset();
        fake_clock::reset();
        tlsr8266_hal::mcu::crypto::fake_crypto::reset();
        FakeFlash::reset();
        install_noop_hooks();
    }

    /// Count of `on_light_data` dispatches recorded by the installed hook.
    static RECORDED_LIGHT_DATA: AtomicUsize = AtomicUsize::new(0);
    /// Count of `on_light_response` dispatches recorded by the installed hook.
    static RECORDED_LIGHT_RESPONSE: AtomicUsize = AtomicUsize::new(0);

    fn noop_mesh_pair_proc() {}
    fn noop_light_data(_pkt: &Packet) {}
    fn noop_light_response(_rsp: &mut PacketAttValue, _src: &PacketAttValue) -> Result<(), ()> {
        Err(())
    }
    fn noop_uart_mesh_rx(_pkt: &Packet) {}

    /// Records a `on_light_data` dispatch so a test can prove the SDK reached
    /// the application seam.
    fn record_light_data(_pkt: &Packet) {
        RECORDED_LIGHT_DATA.fetch_add(1, Ordering::Relaxed);
    }

    /// Records a `on_light_response` dispatch and answers `Ok(())`, which is the
    /// application behaviour that makes the SDK queue a status packet.
    fn record_light_response(_rsp: &mut PacketAttValue, _src: &PacketAttValue) -> Result<(), ()> {
        RECORDED_LIGHT_RESPONSE.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    /// Installs hooks that record the two seams these tests assert on.
    fn install_recording_hooks() {
        RECORDED_LIGHT_DATA.store(0, Ordering::Relaxed);
        RECORDED_LIGHT_RESPONSE.store(0, Ordering::Relaxed);
        set_ble_app_hooks(BleAppHooks {
            on_mesh_pair_proc: noop_mesh_pair_proc,
            on_light_data: record_light_data,
            on_light_response: record_light_response,
            on_uart_mesh_rx: noop_uart_mesh_rx,
        });
    }

    /// Restores the neutral hook set the rest of the suite assumes.
    fn install_noop_hooks() {
        set_ble_app_hooks(BleAppHooks {
            on_mesh_pair_proc: noop_mesh_pair_proc,
            on_light_data: noop_light_data,
            on_light_response: noop_light_response,
            on_uart_mesh_rx: noop_uart_mesh_rx,
        });
    }

    /// Sequence number the packet-handling fixture stores as "already seen".
    const OLD_SNO: [u8; 3] = [1, 2, 2];
    /// Sequence number the packets under test carry, so they are not duplicates.
    const NEW_SNO: [u8; 3] = [1, 2, 3];

    /// Builds an ATT-write packet the real `parse_ble_packet_op_params` decodes
    /// as a 3-byte opcode followed by `params`.
    ///
    /// `rf_len` is the minimum the handler accepts and `l2cap_len` is exactly
    /// header (10) + opcode (3) + parameters, so the parser reports
    /// `op_cmd_len == 3` and `params_len == params.len()`.
    fn decoded_packet(opcode: u8, params: &[u8]) -> Packet {
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead {
                    rf_len: 0x11,
                    l2cap_len: (10 + 3 + params.len()) as u16,
                    ..Default::default()
                },
                ..Default::default()
            },
        };
        packet.att_write_mut().value.val[0] = 0x80 | 0x40 | (opcode & 0x3f);
        packet.att_write_mut().value.val[1] = 0x02;
        packet.att_write_mut().value.val[2] = 0x03;
        packet.att_write_mut().value.val[3..3 + params.len()].copy_from_slice(params);
        packet.att_write_mut().value.sno = NEW_SNO;
        packet
    }

    /// Puts every radio/packet global reachable from `rf_link_slave_data_write`
    /// in the state the retired mocks assumed: a stale sequence number, no group
    /// membership, no address match, no in-flight status read.
    fn reset_packet_state() -> RecordingHooksGuard {
        reset_host_fakes();
        crate::facade::reset_radio_state_for_test();
        critical_section::with(|_| {
            *GENERAL_MESSAGE_SEQUENCE_NUMBER.lock() = OLD_SNO;
            *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() = [0; 3];
        });
        BLE_PERIPHERAL_LINK_COMMAND.set(0xff);
        SLAVE_DATA_VALID.set(0);
        DEVICE_STATUS_RECORD_INDEX.set(0);
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(0);
        DEVICE_STATUS_BUFFER_READ_POINTER.set(0);
        NOTIFICATION_REQUEST_MASK_INDEX.set(0);
        SLAVE_LINK_INTERVAL.set(1000 * CLOCK_SYS_CLOCK_1US);
        write_reg_system_tick(0xaabbccdd);
        install_recording_hooks();
        RecordingHooksGuard
    }

    /// Restores the neutral hook set when a test that installed recording hooks
    /// ends, including while unwinding from a failed assertion.
    struct RecordingHooksGuard;

    impl Drop for RecordingHooksGuard {
        fn drop(&mut self) {
            install_noop_hooks();
        }
    }

    /// Asserts the status path queued exactly one responder record.
    fn assert_status_recorded() {
        assert_eq!(
            DEVICE_STATUS_RECORD_INDEX.get(),
            1,
            "add_status must record one status responder"
        );
        assert_eq!(
            DEVICE_STATUS_BUFFER_WRITE_POINTER.get(),
            1,
            "add_status must advance the response buffer write pointer"
        );
    }

    /// A value the analog bank never legitimately holds in these tests, seeded
    /// before a write so the assertion proves the write happened.
    const ANA_UNWRITTEN: u8 = 0xff;

    /// `TBL_RF_INI[0]` is a `TCMD_WRITE` of 0x02 to 0x5b4, and `TBL_RF_INI[2]`
    /// is a `TCMD_WAREG` of 0x31 to analog 0x99 (16MHz-only). Both are the
    /// observable proof that `rf_drv_init` ran the command table.
    const RF_INI_FIRST_REG: u32 = 0x5b4;
    const RF_INI_FIRST_VALUE: u8 = 0x02;
    const RF_INI_16MHZ_ANA: u8 = 0x99;
    const RF_INI_16MHZ_ANA_VALUE: u8 = 0x31;

    /// `reg_rnd_number`, the hardware RNG word `rand()` mixes with the clock.
    const REG_RND_NUMBER: u32 = 0x448;

    /// Asserts the leading command of `TBL_RF_INI` was executed.
    fn assert_rf_init_table_applied() {
        assert_eq!(
            FakeMmio::reg8(RF_INI_FIRST_REG),
            RF_INI_FIRST_VALUE,
            "rf_drv_init must load the RF init command table"
        );
    }

    /// Asserts every AGC stage register carries its `TBL_AGC` value.
    ///
    /// `write_reg_rf_rx_gain_agc(value, i)` writes `value` at `0x480 + i`, and
    /// production shifts the index by two, so stage `i` lands at `0x480 + 4i`.
    fn assert_agc_applied() {
        for (i, &expected) in TBL_AGC.iter().take(6).enumerate() {
            assert_eq!(
                FakeMmio::reg32(0x480 + ((i << 2) as u32)),
                expected,
                "AGC stage {} must carry its table value",
                i
            );
        }
    }

    #[test]
    fn test_rf_link_get_rsp_type() {
        // Group 1: Test the LGT_CMD_LIGHT_READ_STATUS opcode
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_LIGHT_READ_STATUS, 0),
            LGT_CMD_LIGHT_STATUS
        );
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_LIGHT_READ_STATUS, 100),
            LGT_CMD_LIGHT_STATUS
        );

        // Group 2: Test the LGT_CMD_LIGHT_GRP_REQ opcode with different parameter values
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_LIGHT_GRP_REQ, GET_GROUP1),
            LGT_CMD_LIGHT_GRP_RSP1
        );
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_LIGHT_GRP_REQ, GET_GROUP2),
            LGT_CMD_LIGHT_GRP_RSP2
        );
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_LIGHT_GRP_REQ, GET_GROUP3),
            LGT_CMD_LIGHT_GRP_RSP3
        );
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_LIGHT_GRP_REQ, 4),
            LGT_CMD_LIGHT_GRP_RSP1
        ); // Default for unknown param

        // Group 3: Test opcodes that return fixed response values
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_LIGHT_CONFIG_GRP, 0),
            LGT_CMD_LIGHT_GRP_RSP1
        );
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_CONFIG_DEV_ADDR, 0),
            LGT_CMD_DEV_ADDR_RSP
        );
        assert_eq!(rf_link_get_rsp_type(0, 0), 1); // Special case with no constant
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_OTA_DATA_REQ, 0),
            LGT_CMD_OTA_DATA_RSP
        );
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_END_OTA_REQ, 0),
            LGT_CMD_END_OTA_RSP
        );
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_USER_NOTIFY_REQ, 0),
            LGT_CMD_USER_NOTIFY_RSP
        );
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
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_LIGHT_CONFIG_GRP, 100),
            LGT_CMD_LIGHT_GRP_RSP1
        );
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_CONFIG_DEV_ADDR, 100),
            LGT_CMD_DEV_ADDR_RSP
        );
        assert_eq!(rf_link_get_rsp_type(0, 100), 1);
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_OTA_DATA_REQ, 100),
            LGT_CMD_OTA_DATA_RSP
        );
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_END_OTA_REQ, 100),
            LGT_CMD_END_OTA_RSP
        );
        assert_eq!(
            rf_link_get_rsp_type(LGT_CMD_USER_NOTIFY_REQ, 100),
            LGT_CMD_USER_NOTIFY_RSP
        );
        assert_eq!(rf_link_get_rsp_type(7, 100), 8);
    }

    #[test]
    fn test_rf_set_ble_access_code_adv() {
        reset_host_fakes();

        rf_set_ble_access_code_adv();

        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: 0x408 | REG_BASE_ADDR,
                width: 4,
                value: 0xd6be898e,
            }],
            "the advertising access code must reach reg_rf_access_code"
        );
    }

    #[test]
    fn test_rf_set_ble_access_code() {
        reset_host_fakes();

        // Should swap the order of the bytes
        rf_set_ble_access_code(0x12345678);
        assert_eq!(register_writes(0x408), vec![0x78563412]);

        rf_set_ble_access_code(0x78563412);
        assert_eq!(register_writes(0x408), vec![0x78563412, 0x12345678]);
    }

    #[test]
    fn test_rf_set_ble_crc_adv() {
        reset_host_fakes();

        rf_set_ble_crc_adv();

        assert_eq!(register_writes(0x44c), vec![0x555555]);
    }

    #[test]
    fn test_rf_set_ble_crc() {
        reset_host_fakes();

        rf_set_ble_crc(&[0x12, 0x34, 0x56]);

        assert_eq!(
            register_writes(0x44c),
            vec![0x563412],
            "the CRC init value is little-endian in the 24-bit register"
        );
    }

    #[test]
    fn test_rf_reset_sn() {
        reset_host_fakes();

        rf_reset_sn();

        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: 0xf01 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x3f,
                },
                RegWrite {
                    addr: 0xf01 | REG_BASE_ADDR,
                    width: 1,
                    value: 0,
                },
            ],
            "the sequence number is cleared after the 0x3f reset value"
        );
    }

    #[test]
    fn test_rf_start_brx() {
        reset_host_fakes();
        // Registers the scheduling bit must be OR-ed into.
        write_reg_rf_mode(3);
        FakeMmio::clear_writes();

        rf_start_brx(0x12345678, 0x87654321);

        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: 0xf28 | REG_BASE_ADDR,
                    width: 4,
                    value: 0xffffffff,
                },
                RegWrite {
                    addr: 0xf18 | REG_BASE_ADDR,
                    width: 4,
                    value: 0x87654321,
                },
                RegWrite {
                    addr: 0xf16 | REG_BASE_ADDR,
                    width: 1,
                    value: 4 | 3,
                },
                RegWrite {
                    addr: 0xf00 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x82,
                },
                RegWrite {
                    addr: 0x50c | REG_BASE_ADDR,
                    width: 2,
                    value: 0x5678,
                },
            ],
            "brx clears events, schedules the tick, arms brx and points DMA3 at the buffer"
        );
    }

    #[test]
    fn test_rf_start_srx2tx() {
        reset_host_fakes();
        // The scheduling bit is OR-ed into the current RF mode.
        write_reg_rf_mode(0);
        FakeMmio::clear_writes();

        rf_start_srx2tx(0x12345678, 0x87654321);

        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: 0xf18 | REG_BASE_ADDR,
                    width: 4,
                    value: 0x87654321,
                },
                RegWrite {
                    addr: 0xf16 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x04,
                },
                RegWrite {
                    addr: 0xf00 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x85,
                },
                RegWrite {
                    addr: 0x50c | REG_BASE_ADDR,
                    width: 2,
                    value: 0x5678,
                },
            ],
            "srx2tx must arm the srx2tx mode control value"
        );
    }

    #[test]
    fn test_rf_start_stx2rx() {
        reset_host_fakes();
        write_reg_rf_mode(0);
        FakeMmio::clear_writes();

        rf_start_stx2rx(0x12345678, 0x87654321);

        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: 0xf18 | REG_BASE_ADDR,
                    width: 4,
                    value: 0x87654321,
                },
                RegWrite {
                    addr: 0xf16 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x04,
                },
                RegWrite {
                    addr: 0xf00 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x87,
                },
                RegWrite {
                    addr: 0x50c | REG_BASE_ADDR,
                    width: 2,
                    value: 0x5678,
                },
            ],
            "stx2rx must arm the stx2rx mode control value"
        );
    }

    /// Seeds the four analog registers `rf_set_power_level_index` writes with a
    /// value the table never holds, so a missing write cannot pass by luck.
    fn seed_power_registers() {
        for addr in [0xa2u8, 4, 0xa7, 0x8d] {
            FakeAnalog::set(addr, ANA_UNWRITTEN);
        }
    }

    /// Asserts the four analog registers hold `TBL_RF_POWER[index]`.
    fn assert_power_registers(index: usize) {
        assert_eq!(
            FakeAnalog::get(0xa2),
            TBL_RF_POWER[index].a,
            "0xa2 must carry the table's `a` byte"
        );
        assert_eq!(
            FakeAnalog::get(4),
            TBL_RF_POWER[index].b,
            "0x04 must carry the table's `b` byte"
        );
        assert_eq!(
            FakeAnalog::get(0xa7),
            TBL_RF_POWER[index].c,
            "0xa7 must carry the table's `c` byte"
        );
        assert_eq!(
            FakeAnalog::get(0x8d),
            TBL_RF_POWER[index].d,
            "0x8d must carry the table's `d` byte"
        );
    }

    // Split the power level tests into separate test functions
    #[test]
    fn test_rf_set_power_level_index_highest() {
        // Test highest power level (index 0)
        reset_host_fakes();
        seed_power_registers();

        rf_set_power_level_index(0);

        assert_power_registers(0);
    }

    #[test]
    fn test_rf_set_power_level_index_middle() {
        // Test middle power level (index 6)
        reset_host_fakes();
        seed_power_registers();

        rf_set_power_level_index(6);

        assert_power_registers(6);
    }

    #[test]
    fn test_rf_set_power_level_index_lowest() {
        // Test lowest power level (index 11)
        reset_host_fakes();
        seed_power_registers();

        rf_set_power_level_index(11);

        assert_power_registers(11);
    }

    #[test]
    fn test_rf_set_power_level_index_out_of_range() {
        // Test out of range index (should use the last entry in the table)
        reset_host_fakes();
        seed_power_registers();

        rf_set_power_level_index(20); // Out of range

        assert_power_registers(11);
    }

    #[test]
    fn test_rf_set_ble_channel_data_channels() {
        reset_host_fakes();
        FakeAnalog::set(REGA_SAR_ADC_PWR, ANA_UNWRITTEN);

        // Test data channel (0-10)
        rf_set_ble_channel(5);
        assert_eq!(register_writes(0x40d), vec![5]);
        // Channel 5 should result in gain = (5+2)*2 = 14, intgn = 14+2400 = 2414
        assert_eq!(register_writes(0x4d6), vec![2414]);
        // SAR ADC should be powered off: seeded to a value the write must clear
        assert_eq!(
            FakeAnalog::get(REGA_SAR_ADC_PWR),
            0,
            "the SAR ADC power register must be written to 0"
        );
        // RF mode should be set to 0x29
        assert_eq!(register_writes(0xf16), vec![0x29]);
        // RX should be disabled
        assert_eq!(register_writes(0x428), vec![0]);
        // TX/RX state machine should be reset
        assert_eq!(register_writes(0xf02), vec![RF_TRX_OFF as u32]);

        // Test data channel (11-36)
        rf_set_ble_channel(20);
        assert_eq!(register_writes(0x40d), vec![5, 20]);
        // Channel 20 should result in gain = (20+3)*2 = 46, intgn = 46+2400 = 2446
        assert_eq!(register_writes(0x4d6), vec![2414, 2446]);
    }

    #[test]
    fn test_rf_set_ble_channel_adv_channels() {
        reset_host_fakes();

        // Test advertising channels
        // Channel 37 (2402 MHz)
        rf_set_ble_channel(37);
        assert_eq!(register_writes(0x40d), vec![37]);
        assert_eq!(register_writes(0x4d6), vec![2402]);
        assert_eq!(register_writes(0xf16), vec![0x29]);
        assert_eq!(register_writes(0x428), vec![0]);
        assert_eq!(register_writes(0xf02), vec![RF_TRX_OFF as u32]);

        // Channel 38 (2426 MHz)
        rf_set_ble_channel(38);
        assert_eq!(register_writes(0x40d), vec![37, 38]);
        assert_eq!(register_writes(0x4d6), vec![2402, 2426]);

        // Channel 39 (2480 MHz)
        rf_set_ble_channel(39);
        assert_eq!(register_writes(0x40d), vec![37, 38, 39]);
        assert_eq!(register_writes(0x4d6), vec![2402, 2426, 2480]);
    }

    #[test]
    fn test_rf_set_ble_channel_extended_40_50() {
        reset_host_fakes();

        // Test extended channel (40-50)
        rf_set_ble_channel(45);
        assert_eq!(register_writes(0x40d), vec![45]);
        // Channel 45 should result in gain = 45*2 = 90, intgn = 90+2400 = 2490
        assert_eq!(register_writes(0x4d6), vec![2490]);
        assert_eq!(register_writes(0xf16), vec![0x29]);
        assert_eq!(register_writes(0x428), vec![0]);
        assert_eq!(register_writes(0xf02), vec![RF_TRX_OFF as u32]);
    }

    #[test]
    fn test_rf_set_ble_channel_extended_51_61() {
        reset_host_fakes();

        // Test extended channel (51-61) with our new calculation
        // For channel 55:
        // gain = (61-55)*2 = 12
        // intgn = 2408 - gain = 2408 - 12 = 2396
        rf_set_ble_channel(55);
        assert_eq!(register_writes(0x40d), vec![55]);
        assert_eq!(register_writes(0x4d6), vec![2396]);
        assert_eq!(register_writes(0xf16), vec![0x29]);
        assert_eq!(register_writes(0x428), vec![0]);
        assert_eq!(register_writes(0xf02), vec![RF_TRX_OFF as u32]);
    }

    #[test]
    fn test_rf_set_ble_channel_out_of_range() {
        reset_host_fakes();

        // Test out of range channel (> 61)
        rf_set_ble_channel(62);
        assert_eq!(register_writes(0x40d), vec![62]);
        // Out of range channel should default to 0x50 and 2480
        assert_eq!(register_writes(0x4d6), vec![2480]);
        assert_eq!(register_writes(0xf16), vec![0x29]);
        assert_eq!(register_writes(0x428), vec![0]);
        assert_eq!(register_writes(0xf02), vec![RF_TRX_OFF as u32]);
    }

    #[test]
    fn test_rf_stop_trx() {
        reset_host_fakes();

        rf_stop_trx();

        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: 0xf00 | REG_BASE_ADDR,
                width: 1,
                value: 0x80,
            }],
            "stop is the 0x80 mode-control value"
        );
    }

    #[test]
    fn test_rf_set_rxmode() {
        reset_host_fakes();

        rf_set_rxmode();

        assert_eq!(
            register_writes(0x428),
            vec![(FLD_RF_RX_MODE::LOW_PASS_FILTER | FLD_RF_RX_MODE::EN).bits() as u32]
        );
        assert_eq!(register_writes(0xf02), vec![(RF_TRX_OFF | BIT!(5)) as u32]);
    }

    #[test]
    fn test_rf_set_tx_rx_off() {
        reset_host_fakes();

        rf_set_tx_rx_off();

        assert_eq!(register_writes(0xf16), vec![0x29]);
        assert_eq!(
            register_writes(0x428),
            vec![FLD_RF_RX_MODE::LOW_PASS_FILTER.bits() as u32]
        );
        assert_eq!(register_writes(0xf02), vec![RF_TRX_OFF as u32]);
    }

    #[test]
    fn test_rf_link_slave_set_adv() {
        reset_host_fakes();

        // Setup a test advertisement data slice
        let test_adv_data = [0x02, 0x01, 0x06, 0x08, 0xFF, 0x12, 0x34];

        // Create a temporary locked PKT_ADV for testing
        critical_section::with(|_| {
            let pkt_adv = Packet {
                adv_ind_module: RfPacketAdvIndModuleT::default(),
            };
            PKT_ADV.lock().clone_from(&pkt_adv);

            // Call the function
            rf_link_slave_set_adv(&test_adv_data);

            // Get updated packet from the global state
            let updated_pkt = *PKT_ADV.lock();

            // Verify the correct values were set
            let p = updated_pkt.head().dma_len;
            assert_eq!(p, test_adv_data.len() as u32 + 8);
            assert_eq!(updated_pkt.head().rf_len, test_adv_data.len() as u8 + 6);

            // Verify the advertisement data was copied correctly
            assert_eq!(
                updated_pkt.adv_ind_module().data[0..test_adv_data.len()],
                test_adv_data
            );
        });
    }

    #[test]
    fn test_rf_set_tp_gain_zero() {
        reset_host_fakes();
        FakeAnalog::set(0x93, ANA_UNWRITTEN);

        // Set up the global state variables
        RF_TP_BASE.set(0x30);
        RF_TP_GAIN.set(0x40);

        // Test gain = 0, should result in BASE value unchanged
        rf_set_tp_gain(0);
        assert_eq!(FakeAnalog::get(0x93), 0x30, "gain 0 keeps the base value");
    }

    #[test]
    fn test_rf_set_tp_gain_medium() {
        reset_host_fakes();
        FakeAnalog::set(0x93, ANA_UNWRITTEN);

        // Set up the global state variables
        RF_TP_BASE.set(0x30);
        RF_TP_GAIN.set(0x40);

        // Test gain = 10
        // Calculation: BASE - ((gain * GAIN + 0x80) >> 8)
        // 0x30 - ((10 * 0x40 + 0x80) >> 8) = 0x30 - 3 = 0x2D
        rf_set_tp_gain(10);
        assert_eq!(FakeAnalog::get(0x93), 0x2D, "gain 10 scales to 0x2D");
    }

    #[test]
    fn test_rf_set_tp_gain_max() {
        reset_host_fakes();
        FakeAnalog::set(0x93, ANA_UNWRITTEN);

        // Set up the global state variables
        RF_TP_BASE.set(0x10);
        RF_TP_GAIN.set(0x40);

        // Test gain = 0xFF (maximum gain)
        // Without overflow protection: 0x10 - ((0xFF * 0x40 + 0x80) >> 8) would cause overflow
        // With overflow protection: Since power_adjustment > base_value, we should get 0
        rf_set_tp_gain(0xFF);
        assert_eq!(
            FakeAnalog::get(0x93),
            0,
            "an adjustment larger than the base must clamp to 0"
        );
    }

    /// Tests that rf_init_table selects the full 61-entry table or the 12MHz prefix.
    ///
    /// This test verifies that rf_init_table:
    /// - Returns all 61 commands when a 16MHz crystal was detected
    /// - Returns only the leading 57 commands on the default 12MHz crystal
    #[test]
    fn test_rf_init_table_lengths() {
        assert_eq!(
            rf_init_table(true).len(),
            61,
            "a 16MHz crystal should load the full table"
        );
        assert_eq!(
            rf_init_table(false).len(),
            57,
            "a 12MHz crystal should skip the four 16MHz-only commands"
        );
        assert_eq!(
            TBL_RF_INI.len(),
            61,
            "TBL_RF_INI size is part of the contract"
        );
    }

    /// Tests that the 12MHz table is a prefix of the 16MHz table.
    ///
    /// The two selections must differ only by the trailing entries: every command
    /// the 12MHz crystal loads must be byte-identical to the 16MHz command at the
    /// same index, so a 12MHz part never gets a different register value.
    #[test]
    fn test_rf_init_table_12mhz_is_prefix_of_16mhz() {
        let table_16mhz = rf_init_table(true);
        let table_12mhz = rf_init_table(false);

        for (index, (cmd_12mhz, cmd_16mhz)) in
            table_12mhz.iter().zip(table_16mhz.iter()).enumerate()
        {
            // Field access is copied out of the packed struct instead of
            // borrowed: a reference to a packed field is unaligned.
            let (adr_12mhz, dat_12mhz, cmd_12mhz) = (cmd_12mhz.adr, cmd_12mhz.dat, cmd_12mhz.cmd);
            let (adr_16mhz, dat_16mhz, cmd_16mhz) = (cmd_16mhz.adr, cmd_16mhz.dat, cmd_16mhz.cmd);

            assert_eq!(adr_12mhz, adr_16mhz, "adr differs at index {}", index);
            assert_eq!(dat_12mhz, dat_16mhz, "dat differs at index {}", index);
            assert_eq!(cmd_12mhz, cmd_16mhz, "cmd differs at index {}", index);
        }
    }

    /// Tests that the entries skipped on a 12MHz crystal are the four 16MHz-only commands.
    ///
    /// This pins which commands the runtime selection drops: the PLL calibration
    /// enable at 0x4eb and the analog gauss filter, RX ADC clock and DC modulator
    /// retunes at 0x99, 0x82 and 0x9e.
    #[test]
    fn test_rf_init_table_drops_16mhz_only_commands() {
        let table_16mhz = rf_init_table(true);
        let first_dropped = rf_init_table(false).len();

        // (adr, dat, cmd) of the four commands only a 16MHz crystal receives
        let expected = [
            (0x4ebu16, 0x60u8, 0xc3u8), // reg_pll_ctrl_a = CAL_DONE_EN | LOCK_EN
            (0x99, 0x31, 0xc8),         // gauss filter select, 16M
            (0x82, 0x34, 0xc8),         // enable RX ADC clock
            (0x9e, 0x41, 0xc8),         // DC modulator (dc_mod), 500K
        ];

        for (offset, (adr, dat, cmd)) in expected.iter().enumerate() {
            let entry = &table_16mhz[first_dropped + offset];
            // Copy the fields out: taking a reference to a packed field is unaligned.
            let (entry_adr, entry_dat, entry_cmd) = (entry.adr, entry.dat, entry.cmd);
            assert_eq!(entry_adr, *adr, "adr of dropped entry {}", offset);
            assert_eq!(entry_dat, *dat, "dat of dropped entry {}", offset);
            assert_eq!(entry_cmd, *cmd, "cmd of dropped entry {}", offset);
        }
    }

    #[test]
    fn test_rf_drv_init_enable() {
        reset_host_fakes();
        // Seed the deepsleep flag the driver reports back
        FakeAnalog::set(rega_deepsleep_flag, 0x40);

        // Blank flash: power settings read back as 0xFF (the default branch).

        // Set some initial values for global variables
        let initial_base = 0x22;
        let initial_gain = 0x33;
        RF_TP_BASE.set(initial_base);
        RF_TP_GAIN.set(initial_gain);

        // Call with enable=true - should initialize the driver
        let result = rf_drv_init(true);

        // Verify table initialization was loaded (observable as its writes)
        assert_rf_init_table_applied();

        // Verify AGC settings were configured
        assert_agc_applied();

        // With all 0xFF values in power_settings, RF_TP_BASE and RF_TP_GAIN should remain unchanged
        assert_eq!(RF_TP_BASE.get(), initial_base);
        assert_eq!(RF_TP_GAIN.get(), initial_gain);

        // Verify the function returns the deepsleep flag value
        assert_eq!(result, 0x40);
    }

    /// Tests that rf_drv_init consults the crystal detected at boot.
    ///
    /// `rf_drv_init` selects the init table from `is_16mhz_xtal()`. This test
    /// pins that contract: with a 16MHz crystal the four 16MHz-only commands
    /// run, which is visible as the analog gauss-filter write at 0x99.
    #[test]
    fn test_rf_drv_init_reads_detected_xtal_flag() {
        reset_host_fakes();
        // Deepsleep flag clear, and a 16MHz crystal detected at boot
        FakeAnalog::set(rega_deepsleep_flag, 0);
        fake_clock::set_xtal_16mhz(true);

        // Erased flash: no power calibration, so RF_TP_BASE / RF_TP_GAIN are
        // left alone.

        let result = rf_drv_init(true);

        // The detected crystal reaches the table selection: the 16MHz-only
        // analog command at 0x99 (TCMD_WAREG, dat 0x31) must have run.
        assert_rf_init_table_applied();
        assert_eq!(
            FakeAnalog::get(RF_INI_16MHZ_ANA),
            RF_INI_16MHZ_ANA_VALUE,
            "a detected 16MHz crystal must select the 16MHz-only table entries"
        );

        assert_eq!(result, 0);
    }

    #[test]
    fn test_rf_drv_init_disable() {
        reset_host_fakes();
        // Seed the deepsleep flag and the SAR ADC power register
        FakeAnalog::set(rega_deepsleep_flag, 0x40);
        FakeAnalog::set(6, ANA_UNWRITTEN);

        // Call with enable=false - should power off the RF driver
        let result = rf_drv_init(false);

        // Verify that the SAR ADC was powered off
        assert_eq!(FakeAnalog::get(6), 0, "SAR ADC power must be cleared");

        // Verify the function returns the deepsleep flag value (0x40 is not the
        // bank default, so this proves the flag was read)
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
    fn test_rf_drv_init_with_power_settings() {
        reset_host_fakes();
        let cfg = radio_config();
        // Seed the deepsleep flag the driver reports back
        FakeAnalog::set(rega_deepsleep_flag, 0x40);

        // Seed custom power calibration values at offsets 0x11 and 0x12 of the
        // MAC sector.
        FakeFlash::write_slice(cfg.flash_adr_mac + 0x11, &[0x25, 0x10]);

        // Reset the global variables to ensure they're modified by the function
        RF_TP_BASE.set(0);
        RF_TP_GAIN.set(0);

        // Call with enable=true - should initialize the driver
        let result = rf_drv_init(true);

        // Verify that RF_TP_BASE and RF_TP_GAIN were updated with the values from flash
        assert_eq!(RF_TP_BASE.get(), 0x25); // Should be set directly from power_settings[0x11]

        // RF_TP_GAIN is calculated as ((RF_TP_BASE - power_settings[0x12]) << 8) / 80
        // With RF_TP_BASE = 0x25 and power_settings[0x12] = 0x10, we get:
        // ((0x25 - 0x10) << 8) / 80 = (0x1500 / 80) = 67
        assert_eq!(RF_TP_GAIN.get(), 67);

        // The AGC bank is still configured on this branch.
        assert_agc_applied();

        // Verify the function returns the deepsleep flag value
        assert_eq!(result, 0x40);
    }

    #[test]
    fn test_rf_drv_init_with_default_power_settings() {
        reset_host_fakes();
        // Seed the deepsleep flag the driver reports back
        FakeAnalog::set(rega_deepsleep_flag, 0x40);

        // Blank flash: both calibration bytes read back as 0xFF.

        // Set initial values to verify they don't change when flash contains default values
        let initial_base = 0x22;
        let initial_gain = 0x33;
        RF_TP_BASE.set(initial_base);
        RF_TP_GAIN.set(initial_gain);

        // Call with enable=true - should initialize the driver
        let result = rf_drv_init(true);

        // When power_settings contains 0xFF (default), the values should remain unchanged
        assert_eq!(RF_TP_BASE.get(), initial_base);
        assert_eq!(RF_TP_GAIN.get(), initial_gain);

        assert_agc_applied();

        // Verify the function returns the deepsleep flag value
        assert_eq!(result, 0x40);
    }

    #[test]
    fn test_rf_drv_init_disabled() {
        reset_host_fakes();
        let cfg = radio_config();
        // Seed the deepsleep flag and the SAR ADC power register
        FakeAnalog::set(rega_deepsleep_flag, 0x40);
        FakeAnalog::set(6, ANA_UNWRITTEN);

        // Seed valid calibration values: if the disabled path read flash they
        // would be applied, so unchanged globals prove the read never happened.
        FakeFlash::write_slice(cfg.flash_adr_mac + 0x11, &[0x30, 0x05]);
        let initial_base = 0x22;
        let initial_gain = 0x33;
        RF_TP_BASE.set(initial_base);
        RF_TP_GAIN.set(initial_gain);

        // Call with enable=false - should power off the RF driver
        let result = rf_drv_init(false);

        // Verify that the SAR ADC was powered off
        assert_eq!(FakeAnalog::get(6), 0, "SAR ADC power must be cleared");

        // Verify the power calibration was NOT applied
        assert_eq!(RF_TP_BASE.get(), initial_base);
        assert_eq!(RF_TP_GAIN.get(), initial_gain);

        // Verify the function returns the deepsleep flag value
        assert_eq!(result, 0x40);
    }

    #[test]
    fn test_rf_drv_init_with_partial_power_settings() {
        reset_host_fakes();
        let cfg = radio_config();
        // Seed the deepsleep flag the driver reports back
        FakeAnalog::set(rega_deepsleep_flag, 0x40);

        // Only the first calibration byte is set; the second stays 0xFF.
        FakeFlash::write_slice(cfg.flash_adr_mac + 0x11, &[0x29]);

        // Reset the global variables to ensure they're modified by the function
        RF_TP_BASE.set(0);
        RF_TP_GAIN.set(0);

        // Call with enable=true - should initialize the driver
        let result = rf_drv_init(true);

        // Verify that RF_TP_BASE was updated with the value from flash
        assert_eq!(RF_TP_BASE.get(), 0x29);

        // RF_TP_GAIN is calculated as ((0x29 - 0x19) << 8) / 80 = (0x1000 / 80) = 51
        assert_eq!(RF_TP_GAIN.get(), 51);

        assert_agc_applied();

        // Verify the function returns the deepsleep flag value
        assert_eq!(result, 0x40);
    }

    /// Datasheet offsets of the registers `blc_ll_init_basic_mcu` configures.
    const REG_RF_SYS_TIMER_CONFIG: u32 = 0xf0a;
    const REG_DMA2_ADDR: u32 = 0x508;
    const REG_DMA2_CTRL: u32 = 0x50a;
    const REG_DMA_CHN_IRQ_MSK: u32 = 0x521;
    const REG_IRQ_MASK: u32 = 0x640;
    const REG_SYSTEM_TICK_MODE: u32 = 0x74c;
    const REG_RF_IRQ_MASK: u32 = 0xf1c;
    const REG_RF_IRQ_STATUS: u32 = 0xf20;
    const REG_SYSTEM_TICK: u32 = 0x740;
    const REG_SYSTEM_TICK_IRQ: u32 = 0x744;
    const REG_IRQ_SRC: u32 = 0x648;
    const REG_RF_TIMING_CONFIG: u32 = 0xf2c;

    /// Values written to `addr`, oldest first, as recorded by FakeMmio.
    fn register_writes(addr: u32) -> Vec<u32> {
        FakeMmio::writes()
            .iter()
            .filter(|w| w.addr == (addr | tlsr8266_pac::REG_BASE_ADDR))
            .map(|w| w.value)
            .collect()
    }

    #[test]
    fn test_blc_ll_init_basic_mcu() {
        reset_host_fakes();

        // Pre-existing hardware state the function must preserve while OR-ing
        // its own interrupt enables in.
        FakeMmio::set_reg32(REG_IRQ_MASK, 0x1000);
        FakeMmio::set_reg8(REG_SYSTEM_TICK_MODE, 0x01);
        FakeMmio::set_reg32(REG_SYSTEM_TICK, 0x1234);

        // Call the function under test
        blc_ll_init_basic_mcu();

        // Verify all register operations were performed correctly
        assert_eq!(
            FakeMmio::reg16(REG_RF_SYS_TIMER_CONFIG),
            700,
            "system tick interval"
        );
        assert_eq!(FakeMmio::reg16(REG_DMA2_CTRL), 0x104, "DMA2 control");
        assert_eq!(
            FakeMmio::reg8(REG_DMA_CHN_IRQ_MSK),
            0,
            "DMA channel IRQ mask"
        );

        // RF interrupt configuration: the mask is cleared, then the RX|TX bits
        // are set, and the second `reg_irq_mask` write carries the first one's
        // bits, which proves the read-modify-write ran twice.
        let expected_rf_irq_mask = (FLD_RF_IRQ_MASK::IRQ_RX | FLD_RF_IRQ_MASK::IRQ_TX).bits();
        assert_eq!(
            register_writes(REG_RF_IRQ_MASK),
            vec![0, expected_rf_irq_mask as u32],
            "the RF IRQ mask must be cleared and then enabled"
        );
        assert_eq!(FakeMmio::reg16(REG_RF_IRQ_STATUS), 0xfffe, "RF IRQ status");

        assert_eq!(
            register_writes(REG_IRQ_MASK),
            vec![0x1000 | 0x2000, 0x1000 | 0x2000 | (0x80 << 0xd)],
            "both read-modify-write passes must accumulate"
        );
        assert_eq!(
            FakeMmio::reg8(REG_SYSTEM_TICK_MODE),
            0x01 | 2,
            "system tick mode keeps its existing bits"
        );
        assert_eq!(
            FakeMmio::reg32(REG_SYSTEM_TICK_IRQ),
            0x1234 | (0x80 << 0x18),
            "system tick IRQ keeps the current tick"
        );
        assert_eq!(
            FakeMmio::reg32(REG_IRQ_SRC),
            0x80 << 0xd,
            "IRQ source select"
        );
        assert_eq!(FakeMmio::reg16(REG_RF_TIMING_CONFIG), 0xc00, "RF timing");
    }

    #[test]
    fn test_blc_ll_init_basic_mcu_dma_buffer_address() {
        // This test specifically checks that the DMA buffer address is correctly set
        reset_host_fakes();
        FakeMmio::set_reg32(REG_IRQ_MASK, 0x1000);
        FakeMmio::set_reg8(REG_SYSTEM_TICK_MODE, 0x01);
        FakeMmio::set_reg32(REG_SYSTEM_TICK, 0x1234);

        critical_section::with(|_| {
            // Set the receive buffer write pointer to a specific value for testing
            LIGHT_RX_BUFFER_WRITE_POINTER.set(2);

            // Call the function under test
            blc_ll_init_basic_mcu();

            // The DMA2 address must be the address of the buffer element the
            // write pointer selects.
            let expected_addr = core::ptr::addr_of!(LIGHT_RX_BUFF.lock()[2]) as u16;
            assert_eq!(
                FakeMmio::reg16(REG_DMA2_ADDR),
                expected_addr,
                "DMA2 must point at the selected RX buffer element"
            );
        });
    }

    #[test]
    fn test_rf_link_slave_read_status_start() {
        reset_host_fakes();
        // Seed the hardware tick the operation timestamps itself with
        write_reg_system_tick(0x12345678);

        // Create test packet with specific values for testing
        let mut test_packet = Packet {
            att_cmd: PacketAttCmd::default(),
        };

        // Set up opcode and parameters for LGT_CMD_LIGHT_READ_STATUS
        test_packet.att_cmd_mut().value.val[0] = LGT_CMD_LIGHT_READ_STATUS;
        test_packet.att_cmd_mut().value.val[4] = 0xAB; // Random parameter value

        // Set up unicast address (non-broadcast)
        test_packet.att_cmd_mut().value.dst[1] = 0x42; // Not broadcast (MSB not set)

        // Set status tick to a known value
        DEVICE_STATUS_TICK_COUNTER.set(0x55);

        // Dirty the state the parameter initialiser must reset
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(9);
        DEVICE_STATUS_BUFFER_READ_POINTER.set(9);
        *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() = [9, 9, 9];

        // Call the function under test
        rf_link_slave_read_status_start(&mut test_packet);

        // Verify system tick was recorded
        assert_eq!(DEVICE_STATUS_READ_BUSY_TIMESTAMP.get(), 0x12345678);

        // Verify busy status was set from the opcode (mapped through rf_link_get_rsp_type)
        assert_eq!(SLAVE_READ_STATUS_BUSY.get(), LGT_CMD_LIGHT_STATUS);

        // Verify unicast flag is set for a non-broadcast address
        assert!(DEVICE_STATUS_READ_UNICAST_MODE.get());

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
        assert_eq!(test_packet.att_cmd().value.val[3], 0x55); // Should contain DEVICE_STATUS_TICK_COUNTER

        // Verify the parameter initialiser reset the response buffer state
        assert_eq!(DEVICE_STATUS_BUFFER_WRITE_POINTER.get(), 0);
        assert_eq!(DEVICE_STATUS_BUFFER_READ_POINTER.get(), 0);
        assert_eq!(*STATUS_MESSAGE_SEQUENCE_NUMBER.lock(), [0; 3]);

        // Verify status record variables were reset
        assert_eq!(DEVICE_STATUS_RECORD_INDEX.get(), 0);

        // Verify notify request mask index was reset
        assert_eq!(NOTIFICATION_REQUEST_MASK_INDEX.get(), 0);
    }

    #[test]
    fn test_rf_link_slave_read_status_start_broadcast() {
        reset_host_fakes();
        // Seed the hardware tick the operation timestamps itself with
        write_reg_system_tick(0x87654321);

        // Create test packet with specific values for testing
        let mut test_packet = Packet {
            att_cmd: PacketAttCmd::default(),
        };

        // Set up opcode and parameters for LGT_CMD_LIGHT_GRP_REQ
        test_packet.att_cmd_mut().value.val[0] = LGT_CMD_LIGHT_GRP_REQ;
        test_packet.att_cmd_mut().value.val[4] = GET_GROUP2; // Using group 2 parameter

        // Set up broadcast address (bit 7 set in dst[1])
        test_packet.att_cmd_mut().value.dst[1] = 0x80; // Broadcast (MSB set)

        // Dirty the state the parameter initialiser must reset
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(9);
        DEVICE_STATUS_BUFFER_READ_POINTER.set(9);
        *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() = [9, 9, 9];

        // Call the function under test
        rf_link_slave_read_status_start(&mut test_packet);

        // Verify system tick was recorded
        assert_eq!(DEVICE_STATUS_READ_BUSY_TIMESTAMP.get(), 0x87654321);

        // Verify busy status was set from the opcode and parameter via rf_link_get_rsp_type
        // For LGT_CMD_LIGHT_GRP_REQ with GET_GROUP2, should be LGT_CMD_LIGHT_GRP_RSP2
        assert_eq!(SLAVE_READ_STATUS_BUSY.get(), LGT_CMD_LIGHT_GRP_RSP2);

        // Verify unicast flag is cleared for a broadcast address
        assert!(!DEVICE_STATUS_READ_UNICAST_MODE.get());

        // Broadcast packets don't have the destination fields cleared
        // and don't have special fields set

        // Verify the parameter initialiser reset the response buffer state
        assert_eq!(DEVICE_STATUS_BUFFER_WRITE_POINTER.get(), 0);
        assert_eq!(DEVICE_STATUS_BUFFER_READ_POINTER.get(), 0);
        assert_eq!(*STATUS_MESSAGE_SEQUENCE_NUMBER.lock(), [0; 3]);

        // Verify status record variables were reset
        assert_eq!(DEVICE_STATUS_RECORD_INDEX.get(), 0);

        // Verify notify request mask index was reset
        assert_eq!(NOTIFICATION_REQUEST_MASK_INDEX.get(), 0);
    }

    #[test]
    fn test_rf_link_slave_read_status_start_different_opcode() {
        reset_host_fakes();
        // Seed the hardware tick the operation timestamps itself with
        write_reg_system_tick(0xAABBCCDD);

        // Create test packet with specific values for testing
        let mut test_packet = Packet {
            att_cmd: PacketAttCmd::default(),
        };

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
        assert_eq!(DEVICE_STATUS_READ_BUSY_TIMESTAMP.get(), 0xAABBCCDD);

        // Verify busy status was set from the opcode via rf_link_get_rsp_type
        // For LGT_CMD_CONFIG_DEV_ADDR, should be LGT_CMD_DEV_ADDR_RSP
        assert_eq!(SLAVE_READ_STATUS_BUSY.get(), LGT_CMD_DEV_ADDR_RSP);

        // Verify unicast flag is set for a non-broadcast address
        assert!(DEVICE_STATUS_READ_UNICAST_MODE.get());

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

        // Verify the parameter initialiser ran (it zeroes the sequence number)
        assert_eq!(*STATUS_MESSAGE_SEQUENCE_NUMBER.lock(), [0; 3]);
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_invalid_length() {
        // Create a packet with invalid length (< 0x11)
        let packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead {
                    rf_len: 0x10,
                    ..Default::default()
                },
                ..Default::default()
            },
        };

        // Test the function
        rf_link_slave_data_write_no_dec(&packet);

        // Invalid packet length should return false
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_duplicate_packet() {
        // Create a packet with valid length
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead {
                    rf_len: 0x11,
                    ..Default::default()
                },
                ..Default::default()
            },
        };

        // Set sequence number
        let sno = [1, 2, 3];
        critical_section::with(|_| {
            *GENERAL_MESSAGE_SEQUENCE_NUMBER.lock() = sno;
            packet.att_write_mut().value.sno = sno;
        });

        // Test the function
        rf_link_slave_data_write_no_dec(&packet);

        // Duplicate packet should return true (successful but no processing)
    }

    /// Response type the handler wrote into the outgoing light-data packet.
    fn light_data_response_type() -> u8 {
        critical_section::with(|_| {
            PKT_LIGHT_DATA.lock().att_cmd().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE]
        })
    }

    /// Asserts the packet was dropped before the handler updated any state.
    fn assert_packet_not_processed() {
        assert_eq!(
            *GENERAL_MESSAGE_SEQUENCE_NUMBER.lock(),
            OLD_SNO,
            "a rejected packet must not become the last accepted sequence number"
        );
        assert_eq!(
            BLE_PERIPHERAL_LINK_COMMAND.get(),
            0xff,
            "a rejected packet must not publish a command opcode"
        );
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_non_notification_direct_match() {
        let _hooks = reset_packet_state();
        // dst == 0 is device-addressed and matches this device.
        let packet = decoded_packet(0x01, &[0x10, 0x11, 0x12, 0x13]);

        rf_link_slave_data_write_no_dec(&packet);

        assert_eq!(
            RECORDED_LIGHT_DATA.get(),
            1,
            "a device-addressed packet must reach on_light_data"
        );
        assert_eq!(*GENERAL_MESSAGE_SEQUENCE_NUMBER.lock(), NEW_SNO);
        assert_eq!(BLE_PERIPHERAL_LINK_COMMAND.get(), 0x01);
        assert_eq!(
            SLAVE_DATA_VALID.get(),
            0,
            "a device match needs no bridge forwarding"
        );
        assert_eq!(
            RECORDED_LIGHT_RESPONSE.get(),
            0,
            "a non-notification must not build a status response"
        );
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_non_notification_bridge() {
        let _hooks = reset_packet_state();
        let mut packet = decoded_packet(0x01, &[0x10, 0x11, 0x12, 0x13]);
        // Non-zero, in the device address space, but not this device.
        packet.att_write_mut().value.dst = 0x1234u16.to_le_bytes();

        rf_link_slave_data_write_no_dec(&packet);

        assert_eq!(
            RECORDED_LIGHT_DATA.get(),
            0,
            "a packet for another device must not reach on_light_data"
        );
        assert_eq!(*GENERAL_MESSAGE_SEQUENCE_NUMBER.lock(), NEW_SNO);
        assert_eq!(BLE_PERIPHERAL_LINK_COMMAND.get(), 0x01);
        assert_eq!(
            SLAVE_DATA_VALID.get(),
            BRIDGE_MAX_CNT + 1,
            "a foreign destination must be queued for bridge forwarding"
        );
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_invalid_op_cmd_len() {
        let _hooks = reset_packet_state();
        // A first opcode byte with 0x80 set and 0x40 clear is the unsupported
        // 2-byte form, so the real parser reports `op_cmd_len == 0`.
        let mut packet = decoded_packet(0, &[]);
        packet.att_write_mut().value.val[0] = 0x80;

        rf_link_slave_data_write_no_dec(&packet);

        assert_eq!(
            BLE_PERIPHERAL_LINK_COMMAND.get(),
            0,
            "an opcode that does not come in three bytes is stored as 0"
        );
        assert_eq!(
            *GENERAL_MESSAGE_SEQUENCE_NUMBER.lock(),
            NEW_SNO,
            "the packet is still accepted enough to advance the sequence"
        );
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_device_addr_config_invalid_params() {
        let _hooks = reset_packet_state();
        let mut packet = decoded_packet(
            LGT_CMD_CONFIG_DEV_ADDR,
            &[0xFE, 0xFF, 0x12, 0x13], // params[0] != 0xFF is invalid for broadcast
        );
        packet.att_write_mut().value.dst = [0x56, 0x80 | 0x34]; // broadcast bit set
        SLAVE_DATA_VALID.set(7);

        rf_link_slave_data_write_no_dec(&packet);

        assert_packet_not_processed();
        assert_eq!(
            RECORDED_LIGHT_DATA.get(),
            0,
            "an invalid broadcast device-address config is dropped before dispatch"
        );
        assert_eq!(
            SLAVE_DATA_VALID.get(),
            7,
            "a rejected packet must leave the bridge flag untouched"
        );
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_device_addr_config_valid_broadcast() {
        let _hooks = reset_packet_state();
        let mut packet = decoded_packet(
            LGT_CMD_CONFIG_DEV_ADDR,
            &[0xFF, 0xFF, 0x12, 0x13], // params[0,1] == 0xFF is valid for broadcast
        );
        packet.att_write_mut().value.dst = [0x56, 0x80 | 0x34]; // broadcast bit set

        rf_link_slave_data_write_no_dec(&packet);

        assert_eq!(*GENERAL_MESSAGE_SEQUENCE_NUMBER.lock(), NEW_SNO);
        assert_eq!(
            SLAVE_DATA_VALID.get(),
            BRIDGE_MAX_CNT * 2 + 1,
            "a broadcast address config without the MAC flag waits the maximum bridge delay"
        );
        assert_eq!(light_data_response_type(), GET_DEV_ADDR);
        assert_eq!(
            RECORDED_LIGHT_DATA.get(),
            0,
            "a broadcast config is not addressed to this device"
        );
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_status_read_busy_stop() {
        let _hooks = reset_packet_state();
        let packet = decoded_packet(0x01, &[0x10, 0x11, 0x12]);
        SLAVE_READ_STATUS_BUSY.set(1);
        DEVICE_STATUS_READ_UNICAST_MODE.set(true);

        rf_link_slave_data_write_no_dec(&packet);

        assert_eq!(
            SLAVE_READ_STATUS_BUSY.get(),
            0,
            "a non-notification must stop an in-flight status read"
        );
        assert!(
            !DEVICE_STATUS_READ_UNICAST_MODE.get(),
            "stopping the read must clear unicast mode"
        );
        assert_eq!(
            RECORDED_LIGHT_DATA.get(),
            1,
            "the packet still reaches on_light_data"
        );
        assert_eq!(*GENERAL_MESSAGE_SEQUENCE_NUMBER.lock(), NEW_SNO);
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_notification_light_grp_req() {
        let _hooks = reset_packet_state();
        let mut packet = decoded_packet(LGT_CMD_LIGHT_GRP_REQ, &[0x05, GET_GROUP2, 0x12, 0x13]);
        packet.att_write_mut().value.dst = 0x1234u16.to_le_bytes();

        rf_link_slave_data_write_no_dec(&packet);

        assert_eq!(
            SLAVE_DATA_VALID.get(),
            11,
            "a group request uses params[0] * 2 + 1 as its response slot"
        );
        assert_eq!(
            light_data_response_type(),
            GET_GROUP2,
            "a group request answers with the requested group"
        );
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_notification_config_dev_addr_with_mac_bridge() {
        let _hooks = reset_packet_state();
        let mut packet = decoded_packet(
            LGT_CMD_CONFIG_DEV_ADDR,
            &[0xFF, 0xFF, DEV_ADDR_PAR_WITH_MAC, 0x07],
        );
        packet.att_write_mut().value.dst = [0x56, 0x80 | 0x34]; // broadcast bit set

        rf_link_slave_data_write_no_dec(&packet);

        assert_eq!(
            SLAVE_DATA_VALID.get(),
            15,
            "with the MAC flag the address config uses params[3] * 2 + 1"
        );
        assert_eq!(
            light_data_response_type(),
            GET_DEV_ADDR,
            "a device address config answers with GET_DEV_ADDR"
        );
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_notification_light_config_grp_bridge() {
        let _hooks = reset_packet_state();
        let mut packet = decoded_packet(LGT_CMD_LIGHT_CONFIG_GRP, &[0x05, 0x11, 0x12, 0x13]);
        packet.att_write_mut().value.dst = 0x1234u16.to_le_bytes();

        rf_link_slave_data_write_no_dec(&packet);

        assert_eq!(
            SLAVE_DATA_VALID.get(),
            BRIDGE_MAX_CNT * 2 + 1,
            "a bridged group config waits the maximum bridge delay"
        );
        assert_eq!(
            light_data_response_type(),
            GET_GROUP1,
            "all other non-direct commands answer with GET_GROUP1"
        );
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_notification_light_config_grp() {
        let _hooks = reset_packet_state();
        crate::facade::set_device_address(0x1234);
        let mut packet = decoded_packet(LGT_CMD_LIGHT_CONFIG_GRP, &[0x05, 0x11, 0x12, 0x13]);
        packet.att_write_mut().value.dst = 0x1234u16.to_le_bytes();

        rf_link_slave_data_write_no_dec(&packet);

        assert_eq!(
            SLAVE_DATA_VALID.get(),
            0,
            "a direct device match answers immediately"
        );
        assert_eq!(
            light_data_response_type(),
            GET_GROUP1,
            "a group config answers with GET_GROUP1"
        );
        assert_eq!(RECORDED_LIGHT_DATA.get(), 1);
        assert_eq!(RECORDED_LIGHT_RESPONSE.get(), 1);
        assert_status_recorded();
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_notification_user_notify_req() {
        let _hooks = reset_packet_state();
        let mut packet = decoded_packet(LGT_CMD_USER_NOTIFY_REQ, &[0x05, 0x11, 0x12, 0x13]);
        packet.att_write_mut().value.dst = 0x1234u16.to_le_bytes();

        rf_link_slave_data_write_no_dec(&packet);

        assert_eq!(
            light_data_response_type(),
            GET_USER_NOTIFY,
            "a user notification request answers with GET_USER_NOTIFY"
        );
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_notification_config_dev_addr_device_match() {
        let _hooks = reset_packet_state();
        crate::facade::set_device_address(0x3456);
        let mut packet = decoded_packet(
            LGT_CMD_CONFIG_DEV_ADDR,
            &[0x10, 0x11, 0x12, 0x07], // params[2] != DEV_ADDR_PAR_WITH_MAC
        );
        packet.att_write_mut().value.dst = 0x3456u16.to_le_bytes();

        rf_link_slave_data_write_no_dec(&packet);

        assert_eq!(
            SLAVE_DATA_VALID.get(),
            0,
            "a direct device match answers immediately"
        );
        assert_eq!(
            light_data_response_type(),
            GET_DEV_ADDR,
            "a device address config answers with GET_DEV_ADDR"
        );
        assert_eq!(RECORDED_LIGHT_DATA.get(), 1);
        assert_status_recorded();
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_notification_device_match_new() {
        let _hooks = reset_packet_state();
        crate::facade::set_device_address(0x1234);
        let mut packet = decoded_packet(LGT_CMD_LIGHT_READ_STATUS, &[0x08, 0x11, 0x12, 0x13]);
        packet.att_write_mut().value.dst = 0x1234u16.to_le_bytes();

        rf_link_slave_data_write_no_dec(&packet);

        assert_eq!(
            SLAVE_DATA_VALID.get(),
            0,
            "a direct device match answers immediately"
        );
        assert_eq!(
            light_data_response_type(),
            GET_STATUS,
            "a status read answers with GET_STATUS"
        );
        assert_eq!(RECORDED_LIGHT_DATA.get(), 1);
        assert_status_recorded();
    }

    #[test]
    fn test_rf_link_slave_data_write_no_dec_notification_device_match() {
        let _hooks = reset_packet_state();
        crate::facade::set_device_address(0x1234);
        // A real notification opcode: the retired mock forced `is_notify_req`
        // true for LGT_CMD_LIGHT_ONOFF, which production never treats as a
        // notification. LGT_CMD_USER_NOTIFY_REQ reaches the same branch for real.
        let mut packet = decoded_packet(LGT_CMD_USER_NOTIFY_REQ, &[0x05, 0x11, 0x12, 0x13]);
        packet.att_write_mut().value.dst = 0x1234u16.to_le_bytes();

        rf_link_slave_data_write_no_dec(&packet);

        assert_eq!(
            RECORDED_LIGHT_DATA.get(),
            1,
            "a device-matched packet reaches on_light_data"
        );
        assert_eq!(*GENERAL_MESSAGE_SEQUENCE_NUMBER.lock(), NEW_SNO);
        assert_eq!(
            *STATUS_MESSAGE_SEQUENCE_NUMBER.lock(),
            NEW_SNO,
            "the notification path records the sequence number for the status reply"
        );
        assert_eq!(
            RECORDED_LIGHT_RESPONSE.get(),
            1,
            "the application response hook runs for a notification"
        );
        assert_status_recorded();
        assert_eq!(
            SLAVE_DATA_VALID.get(),
            0,
            "a direct device match answers immediately"
        );
    }

    #[test]
    fn test_rf_link_slave_data_write_pair_login_false() {
        // Arrange: pairing not established, so no decryption is attempted.
        let _hooks = reset_packet_state();
        let packet = decoded_packet(0x01, &[0x10, 0x11, 0x12, 0x13]);

        // Act
        rf_link_slave_data_write(&packet);

        // Assert: neither decryption nor the handler ran.
        assert_packet_not_processed();
        assert_eq!(RECORDED_LIGHT_DATA.get(), 0);
    }

    #[test]
    fn test_rf_link_slave_data_write_decryption_fails() {
        // Arrange: pairing is up and security is on, but the MIC is wrong.
        let _hooks = reset_packet_state();
        PAIR_LOGIN_OK.set(true);
        crate::facade::set_security_enabled(true);
        let mut packet = decoded_packet(0x01, &[0x10, 0x11, 0x12, 0x13]);
        // `src` is the expected MIC, and these bytes cannot be the AES tag for
        // the all-zero session key.
        packet.att_write_mut().value.src = [0xAA, 0xBB];

        // Act
        rf_link_slave_data_write(&packet);

        // Assert: a failed decryption drops the packet.
        assert_packet_not_processed();
        assert_eq!(RECORDED_LIGHT_DATA.get(), 0);
    }

    #[test]
    fn test_rf_link_slave_data_write_success() {
        // Arrange: pair the device, then seal a packet with the same session
        // key and IV that `pair_dec_packet` will use to open it.
        let _hooks = reset_packet_state();
        let key = [0x42u8; 16];
        PAIR_STATE.lock().pair_sk = key;

        crate::facade::set_mac_id([0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
        // `pair_init` seeds PAIR_IVM[0..4] from the MAC and `pair_dec_packet`
        // overwrites [5..8] with the packet sequence number. Byte 4 production
        // code never touches, so a sibling test may have left any value there;
        // seal the same plaintext for each possibility and let the device pick
        // the one whose IV it holds.
        pair_init();

        PAIR_LOGIN_OK.set(true);
        crate::facade::set_security_enabled(true);

        // Plaintext: dst 0 (device match) followed by a 3-byte opcode 0x02.
        let plaintext = [0x00u8, 0x00, 0x80 | 0x40 | 0x02, 0x02, 0x03];
        let mut accepted_iv_byte = None;
        for iv_byte in 0u8..=0xff {
            let ivm = [
                0x11, 0x22, 0x33, 0x44, iv_byte, NEW_SNO[0], NEW_SNO[1], NEW_SNO[2],
            ];
            let mut wire = plaintext;
            let mut mic = [0u8; 2];
            aes_att_encryption_packet(&key, &ivm, &mut mic, &mut wire);

            let mut packet = decoded_packet(0x02, &[]);
            packet.att_write_mut().value.sno = NEW_SNO;
            packet.att_write_mut().value.src = mic;
            packet.att_write_mut().value.dst = [wire[0], wire[1]];
            packet.att_write_mut().value.val[0..3].copy_from_slice(&wire[2..5]);

            // Each attempt must start from "nothing processed yet".
            *GENERAL_MESSAGE_SEQUENCE_NUMBER.lock() = OLD_SNO;
            BLE_PERIPHERAL_LINK_COMMAND.set(0xff);

            rf_link_slave_data_write(&packet);

            if BLE_PERIPHERAL_LINK_COMMAND.get() == 0x02 {
                accepted_iv_byte = Some(iv_byte);
                break;
            }
        }

        // Assert: decryption succeeded, so the plaintext reached the handler.
        assert!(
            accepted_iv_byte.is_some(),
            "a packet sealed with the session key must decrypt and reach rf_link_slave_data_write_no_dec"
        );
        assert_eq!(*GENERAL_MESSAGE_SEQUENCE_NUMBER.lock(), NEW_SNO);
        assert_eq!(
            RECORDED_LIGHT_DATA.get(),
            1,
            "the decrypted packet is device-addressed and reaches on_light_data"
        );
    }

    #[test]
    fn test_rf_link_slave_init_mac_not_set() {
        reset_host_fakes();
        crate::facade::reset_radio_state_for_test();
        let cfg = radio_config();

        // Arrange - MAC address not set (u32::MAX); a blank device reads 0xFFFFFFFF.
        let test_interval = 1000;

        crate::facade::set_mac_id([0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);

        // The seed the driver reports and the clock `rand()` mixes in.
        write_reg_system_tick(0x12345678);

        // The generated MAC is `[rand(), rand()]`, and `rand()` is
        // `clock_time() as u16 ^ read_reg_rnd_number()`; clock_time() is the
        // seeded system tick, the RNG register is seeded here.
        FakeMmio::set_reg16(REG_RND_NUMBER, 0x1234);
        let expected_rand = 0x1234u16 ^ (0x1234_5678u32 as u16);

        // Act
        rf_link_slave_init(test_interval);

        // Assert - the generated MAC really landed in flash.
        let expected_mac_bytes = [
            (expected_rand & 0xff) as u8,
            (expected_rand >> 8) as u8,
            (expected_rand & 0xff) as u8,
            (expected_rand >> 8) as u8,
        ];
        let mut written_mac = [0u8; 4];
        FakeFlash::read_slice(cfg.flash_adr_mac, &mut written_mac);
        assert_eq!(
            written_mac, expected_mac_bytes,
            "both rand() calls must feed the generated MAC"
        );

        // The subsequent 6-byte read takes the freshly written MAC, and the
        // trailing bytes are the still-blank 0xFF tail of the page.
        assert_eq!(
            *MAC_ID.lock(),
            [
                expected_mac_bytes[0],
                expected_mac_bytes[1],
                expected_mac_bytes[2],
                expected_mac_bytes[3],
                0xFF,
                0xFF,
            ],
            "the MAC read-back must consume what the driver just wrote"
        );

        // Calculate expected CRC for assertion
        let expected_mac_u32 = u32::from_le_bytes(expected_mac_bytes);
        let crc_data = [
            (expected_mac_u32 & 0xFF) as u8,
            ((expected_mac_u32 >> 8) & 0xFF) as u8,
            ((expected_mac_u32 >> 16) & 0xFF) as u8,
            ((expected_mac_u32 >> 24) & 0xFF) as u8,
            (cfg.build_version & 0xFF) as u8,
            ((cfg.build_version >> 8) & 0xFF) as u8,
            ((cfg.build_version >> 16) & 0xFF) as u8,
            ((cfg.build_version >> 24) & 0xFF) as u8,
        ];
        let expected_crc = crc16(&crc_data);
        assert_eq!(register_writes(0x808004), vec![expected_mac_u32]);
        assert_eq!(register_writes(0x808008), vec![cfg.build_version]);
        assert_eq!(register_writes(0x80800c), vec![expected_crc as u32]);

        // `rf_link_slave_set_adv` copied the advertising payload into PKT_ADV;
        // pairing then appends the mesh name and manufacturer data after it.
        assert_eq!(
            PKT_ADV.lock().adv_ind_module().data[..ADV_DATA_INIT.len()],
            ADV_DATA_INIT,
            "the advertisement must start with ADV_DATA"
        );
        assert!(
            PKT_ADV.lock().head().rf_len >= ADV_DATA_INIT.len() as u8 + 6,
            "the advertisement must carry at least the ADV_DATA payload"
        );

        // `retrieve_dev_grp_address` derived the default address from the MAC.
        assert_eq!(
            DEVICE_ADDRESS.get(),
            u16::from_le_bytes([expected_mac_bytes[0], expected_mac_bytes[1]]),
            "the address table load must fall back to the MAC prefix"
        );

        // `mesh_node_table_init` registered exactly this device.
        assert_eq!(
            crate::mesh_ll::node_status::MESH_NODE_MAX.get(),
            1,
            "the mesh node table must be re-initialised with self only"
        );

        // `pair_load_key` regenerated the access code and the radio took it.
        assert_eq!(
            FakeMmio::reg32(0x408),
            PAIR_AC.get().swap_bytes(),
            "the access code must be the one pairing just derived"
        );

        // `blc_ll_init_basic_mcu` ran for real.
        assert_eq!(FakeMmio::reg16(REG_RF_TIMING_CONFIG), 0xc00);
    }

    #[test]
    fn test_rf_link_slave_init_normal_initialization() {
        reset_host_fakes();
        crate::facade::reset_radio_state_for_test();
        let cfg = radio_config();

        // Arrange - MAC set, pairing already configured
        let test_interval = 2000;

        // Seed a real MAC in flash so the check reads a non-MAX word and the
        // 6-byte read returns the stored address.
        let stored_mac = [0x11u8, 0x22, 0x33, 0x44, 0x55, 0x66];
        FakeFlash::write_slice(cfg.flash_adr_mac, &stored_mac);

        // Set MAC_ID to known value for testing
        crate::facade::set_mac_id(stored_mac);

        // Seed the system tick the connection deadline is based on.
        write_reg_system_tick(0xAABBCCDD);

        // Register writes for device identification
        let expected_mac_u32 = 0x44332211u32; // From MAC_ID [0x11, 0x22, 0x33, 0x44]
                                              // CRC of the 8 bytes: [0x11, 0x22, 0x33, 0x44, cfg.build_version bytes...]
        let crc_data = [
            0x11,
            0x22,
            0x33,
            0x44,
            (cfg.build_version & 0xFF) as u8,
            ((cfg.build_version >> 8) & 0xFF) as u8,
            ((cfg.build_version >> 16) & 0xFF) as u8,
            ((cfg.build_version >> 24) & 0xFF) as u8,
        ];
        let expected_crc = crc16(&crc_data);

        // Act
        rf_link_slave_init(test_interval);

        // Assert - normal initialization path
        // The stored MAC was not replaced.
        let mut written_mac = [0u8; 6];
        FakeFlash::read_slice(cfg.flash_adr_mac, &mut written_mac);
        assert_eq!(
            written_mac, stored_mac,
            "an existing MAC must not be rewritten"
        );
        assert_eq!(*MAC_ID.lock(), stored_mac);

        // `blc_ll_init_basic_mcu` ran for real.
        assert_eq!(FakeMmio::reg16(REG_RF_TIMING_CONFIG), 0xc00);

        // The advertisement was rebuilt from ADV_DATA (pairing then appends the
        // mesh name and manufacturer data, so only the prefix is pinned here).
        assert_eq!(
            PKT_ADV.lock().adv_ind_module().data[..ADV_DATA_INIT.len()],
            ADV_DATA_INIT
        );

        // The access code comes from the pairing keys and reached the radio.
        assert_eq!(
            FakeMmio::reg32(0x408),
            PAIR_AC.get().swap_bytes(),
            "the access code must be the one pairing derived"
        );

        // `retrieve_dev_grp_address` used the MAC prefix when flash had none.
        assert_eq!(DEVICE_ADDRESS.get(), 0x2211);

        // `mesh_node_table_init` registered exactly this device.
        assert_eq!(crate::mesh_ll::node_status::MESH_NODE_MAX.get(), 1);

        assert_eq!(register_writes(0x808004), vec![expected_mac_u32]);
        assert_eq!(register_writes(0x808008), vec![cfg.build_version]);
        assert_eq!(register_writes(0x80800c), vec![expected_crc as u32]);

        // Verify advertising is enabled
        assert!(BLE_PERIPHERAL_ADVERTISING_ENABLED.get());
    }

    /// The version word at `0x808008` and the CRC over it must come from the
    /// injected `RadioConfig`, not from a compile-time constant.
    #[test]
    fn test_rf_link_slave_init_uses_injected_build_version() {
        reset_host_fakes();
        crate::facade::reset_radio_state_for_test();

        let original = radio_config();
        let mut injected = original;
        injected.build_version = 0x1234_5678;
        set_radio_config(injected);

        // A stored MAC keeps the driver on the read-only flash path.
        let stored_mac = [0x11u8, 0x22, 0x33, 0x44, 0x55, 0x66];
        FakeFlash::write_slice(injected.flash_adr_mac, &stored_mac);
        crate::facade::set_mac_id(stored_mac);
        write_reg_system_tick(0xAABBCCDD);

        rf_link_slave_init(2000);

        // Restore before asserting so a failure cannot leak the injected value
        // into the next test of this single-threaded suite.
        set_radio_config(original);

        let expected_crc = crc16(&[0x11, 0x22, 0x33, 0x44, 0x78, 0x56, 0x34, 0x12]);
        assert_eq!(register_writes(0x808004), vec![0x44332211]);
        assert_eq!(register_writes(0x808008), vec![0x1234_5678]);
        assert_eq!(register_writes(0x80800c), vec![expected_crc as u32]);
    }

    // Proof slice for the RF RX ack helper: the real accessor body runs and
    // routes through the in-process `FakeMmio` bank.

    /// Tests that rf_clear_rx_irq acknowledges only the RX status bit.
    ///
    /// # Notes
    ///
    /// * `reg_rf_irq_status` is 16 bits wide and write-1-to-clear, so the write
    ///   must be exactly `IRQ_RX` at width 2
    #[test]
    fn test_rf_clear_rx_irq_writes_rx_status_bit() {
        FakeMmio::reset();
        FakeMmio::clear_writes();

        rf_clear_rx_irq();

        assert_eq!(
            FakeMmio::reg16(0xf20),
            FLD_RF_IRQ_MASK::IRQ_RX.bits(),
            "only the RX status bit may be acknowledged"
        );
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: 0xf20 | REG_BASE_ADDR,
                width: 2,
                value: FLD_RF_IRQ_MASK::IRQ_RX.bits() as u32,
            }]
        );
    }
}
