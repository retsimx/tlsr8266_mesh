use core::sync::atomic::{
    AtomicBool, AtomicI32, AtomicU16, AtomicU32, AtomicU8, AtomicUsize, Ordering,
};

use heapless::Deque;

use crate::config::VENDOR_ID;
use crate::embassy::sync::mutex::CriticalSectionMutex;
use crate::mesh::{MeshNodeStT, MeshNodeStValT, MESH_NODE_ST_PAR_LEN};
use crate::sdk::light::*;
use crate::sdk::packet_types::*;

/// Response buffer for storing packets that need to be sent back to the BLE host.
/// Contains an array of 16 Packet structures used to queue ATT responses before transmission.
/// Protected by critical section mutex for thread-safe access in interrupt contexts.
pub static BUFF_RESPONSE: CriticalSectionMutex<[Packet; BUFF_RESPONSE_PACKET_COUNT]> =
    CriticalSectionMutex::new(
        [Packet {
            att_data: PacketAttData {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: 0,
                    rf_len: 0,
                    l2cap_len: 0,
                    chan_id: 0,
                },
                att: 0,
                hl: 0,
                hh: 0,
                dat: [0; 23],
            },
        }; BUFF_RESPONSE_PACKET_COUNT],
    );

/// Mesh node status table storing information about all known nodes in the mesh network.
/// Contains up to 64 entries with each node's device address, sequence number, and parameters.
/// Used to track online status, last communication timestamp, and node-specific data.
pub static MESH_NODE_ST: CriticalSectionMutex<[MeshNodeStT; MESH_NODE_MAX_NUM]> =
    CriticalSectionMutex::new(
        [MeshNodeStT {
            tick: 0,
            val: MeshNodeStValT {
                dev_adr: 0,
                sn: 0,
                par: [0; MESH_NODE_ST_PAR_LEN],
            },
        }; MESH_NODE_MAX_NUM],
    );

/// Private advertising data structure used in BLE advertisements.
/// Contains manufacturer ID, mesh product UUID, and MAC address for device identification.
/// Broadcast during BLE advertising to allow discovery by mesh controllers.
pub static ADV_PRI_DATA: CriticalSectionMutex<AdvPrivate> = CriticalSectionMutex::new(AdvPrivate {
    manufacture_id: VENDOR_ID,
    mesh_product_uuid: VENDOR_ID,
    mac_address: 0,
});

/// Private advertising response data sent in response to scan requests.
/// Extended version of advertising data including device status, product UUID,
/// device address, and additional reserved fields for future use.
pub static ADV_RSP_PRI_DATA: CriticalSectionMutex<AdvRspPrivate> =
    CriticalSectionMutex::new(AdvRspPrivate {
        manufacture_id: VENDOR_ID,
        mesh_product_uuid: VENDOR_ID,
        mac_address: 0,
        product_uuid: 0x1234,
        status: 0x01,
        device_address: 0,
        rsv: [0; 16],
    });

/// BLE Link Layer channel table for frequency hopping sequence.
/// Contains the channel map used for data channel selection in BLE connections.
/// Updated during connection parameter negotiations and channel map updates.
pub static BLE_LL_CHANNEL_TABLE: CriticalSectionMutex<[u8; 40]> =
    CriticalSectionMutex::new([0; 40]);

/// Channel map for slave device indicating which of the 37 BLE data channels are usable.
/// Each bit represents one channel; used for adaptive frequency hopping.
pub static SLAVE_CHN_MAP: CriticalSectionMutex<[u8; 5]> = CriticalSectionMutex::new([0; 5]);

/// Current RF operation state indicating which mode is active.
/// Tracks whether the device is in advertising, connected, receiving, mesh listening, or idle state.
pub static CURRENT_RF_STATE: CriticalSectionMutex<RfOperationState> =
    CriticalSectionMutex::new(RfOperationState::Idle);

/// Mesh network name used for pairing and access control.
/// 16-byte string identifier that devices must match to join the mesh network.
pub static PAIR_CONFIG_MESH_NAME: CriticalSectionMutex<[u8; 16]> =
    CriticalSectionMutex::new([0; 16]);

/// Mesh network password used for authentication during pairing.
/// 16-byte password that must be provided to join the mesh network.
pub static PAIR_CONFIG_MESH_PWD: CriticalSectionMutex<[u8; 16]> =
    CriticalSectionMutex::new([0; 16]);

/// Long-Term Key (LTK) for mesh network encryption.
/// 16-byte cryptographic key used to encrypt/decrypt mesh communications.
pub static PAIR_CONFIG_MESH_LTK: CriticalSectionMutex<[u8; 16]> =
    CriticalSectionMutex::new([0; 16]);

/// Group addresses that this device belongs to (up to 8 groups).
/// Used for group-based lighting control where multiple devices respond to the same command.
pub static GROUP_ADDRESS: CriticalSectionMutex<[u16; MAX_GROUP_COUNT as usize]> =
    CriticalSectionMutex::new([0; MAX_GROUP_COUNT as usize]);

/// Current state of the mesh pairing process.
/// Tracks progress through pairing sequence: idle, setting, transmitting, receiving, etc.
pub static PAIR_SETTING_FLAG: CriticalSectionMutex<ePairState> =
    CriticalSectionMutex::new(ePairState::PairSetted);

/// Over-The-Air (OTA) update completion status flag.
/// Indicates whether an OTA firmware update has finished successfully or with errors.
pub static RF_SLAVE_OTA_FINISHED_FLAG: CriticalSectionMutex<OtaState> =
    CriticalSectionMutex::new(OtaState::Continue);

/// Initialization Vector for Master (IVM) used in mesh encryption.
/// 8-byte nonce used with AES encryption for securing mesh communications from master.
pub static PAIR_IVM: CriticalSectionMutex<[u8; 8]> =
    CriticalSectionMutex::new([0, 0, 0, 0, 1, 0, 0, 0]);

/// Session key derived from mesh password for secure pairing.
/// 16-byte key used during the pairing process to encrypt configuration data.
pub static PAIR_CONFIG_PWD_ENCODE_SK: CriticalSectionMutex<[u8; 16]> =
    CriticalSectionMutex::new([0; 16]);

/// Initialization Vector for Slave (IVS) used in mesh encryption.
/// 8-byte nonce used with AES encryption for securing mesh communications from slave.
pub static PAIR_IVS: CriticalSectionMutex<[u8; 8]> = CriticalSectionMutex::new([0; 8]);
/// Status record table for tracking mesh node communications.
/// Records device addresses and alarm IDs for up to 64 nodes in the mesh network.
/// Used to track which devices need status updates or have pending alarms.
pub static SLAVE_STATUS_RECORD: CriticalSectionMutex<[StatusRecord; MESH_NODE_MAX_NUM]> =
    CriticalSectionMutex::new(
        [StatusRecord {
            adr: [0],
            alarm_id: 0,
        }; MESH_NODE_MAX_NUM],
    );
/// Remote control packet buffer storing received commands for processing.
/// Queue of up to 20 packet buffers containing operation codes and sequence numbers.
pub static RC_PKT_BUF: CriticalSectionMutex<Deque<PktBuf, 20>> =
    CriticalSectionMutex::new(Deque::new());

/// Sequence number for device status messages.
/// 3-byte counter used to ensure message ordering and detect duplicates in status reports.
pub static STATUS_MESSAGE_SEQUENCE_NUMBER: CriticalSectionMutex<[u8; 3]> =
    CriticalSectionMutex::new([0; 3]);

/// General sequence number for all device transmissions.
/// 3-byte counter used to track and order outgoing mesh messages from this device.
pub static GENERAL_MESSAGE_SEQUENCE_NUMBER: CriticalSectionMutex<[u8; 3]> =
    CriticalSectionMutex::new([0; 3]);

/// Device MAC address used for network identification.
/// 6-byte IEEE MAC address uniquely identifying this device in the mesh network.
pub static MAC_ID: CriticalSectionMutex<[u8; 6]> = CriticalSectionMutex::new([0; 6]);

/// BLE advertising data payload.
/// Contains flags and service information broadcast during BLE advertising.
pub static ADV_DATA: CriticalSectionMutex<[u8; 3]> = CriticalSectionMutex::new([2, 1, 5]);
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

/// Mesh packet template for light control commands.
/// Pre-structured packet used for sending lighting commands through the mesh network.
/// Contains source/destination addressing and command payload area.
pub static PKT_LIGHT_DATA: CriticalSectionMutex<Packet> = CriticalSectionMutex::new(Packet {
    att_cmd: PacketAttCmd {
        head: PacketL2capHead {
            dma_len: 0x27,
            _type: 2,
            rf_len: 0x25,
            l2cap_len: 0xCCDD,
            chan_id: 0,
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
});
/// Mesh packet template for light status responses.
/// Pre-structured packet used for reporting device status back through the mesh.
/// Contains device state information and addressing for status updates.
pub static PKT_LIGHT_STATUS: CriticalSectionMutex<Packet> = CriticalSectionMutex::new(Packet {
    att_cmd: PacketAttCmd {
        head: PacketL2capHead {
            dma_len: 0x27,
            _type: 2,
            rf_len: 0x25,
            l2cap_len: 0x21,
            chan_id: 0,
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
});

/// BLE connection initialization packet template.
/// Contains connection parameters, access address, channel map, and timing information.
/// Used when establishing BLE connections with central devices.
pub static PKT_INIT: CriticalSectionMutex<Packet> = CriticalSectionMutex::new(Packet {
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

pub struct PairStateData {
    pub pair_ltk: [u8; 16],
    pub pair_sk: [u8; 16],
    pub pair_work: [u8; 16],
    pub pair_nn: [u8; 16],
    pub pair_pass: [u8; 16],
    pub pair_ltk_mesh: [u8; 16],
    pub pair_sk_copy: [u8; 16],
    pub pair_rands: [u8; 8],
    pub pair_randm: [u8; 8],
}

/// Comprehensive pairing state data containing all cryptographic material.
/// Stores LTK, session keys, work buffers, network names, passwords, and random values
/// used during the mesh pairing and authentication process.
pub static PAIR_STATE: CriticalSectionMutex<PairStateData> =
    CriticalSectionMutex::new(PairStateData {
        pair_ltk: [0; 16],
        pair_sk: [0; 16],
        pair_work: [0; 16],
        pair_nn: [0; 16],
        pair_pass: [0; 16],
        pair_ltk_mesh: [0; 16],
        pair_sk_copy: [0; 16],
        pair_rands: [0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7],
        pair_randm: [0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7],
    });

/// Receive buffer array for incoming light control packets.
/// Contains 4 buffers storing received packet data including RSSI, timing, sequence numbers,
/// and MAC addresses. Used for processing incoming mesh communications.
pub static LIGHT_RX_BUFF: CriticalSectionMutex<[LightRxBuff; LIGHT_RX_BUFF_COUNT]> =
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
pub static PKT_EMPTY: Packet = Packet {
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
pub static PKT_TERMINATE: Packet = Packet {
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
pub static PKT_ERR_RSP: Packet = Packet {
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

/// Global flag indicating whether mesh pairing mode is currently active.
/// When true, the device accepts pairing commands and can join new mesh networks.
pub static MESH_PAIR_ENABLE: AtomicBool = AtomicBool::new(false);

/// Bitmask indicating which mesh nodes are active/online.
/// Each bit represents one node in the mesh network (up to 64 nodes).
pub static MESH_NODE_MASK: CriticalSectionMutex<[u32; MESH_NODE_MASK_LEN]> =
    CriticalSectionMutex::new([0; MESH_NODE_MASK_LEN]);

/// Current state of the BLE pairing finite state machine.
/// Tracks progress through pairing steps: idle, random exchange, key exchange, etc.
pub static BLE_PAIR_ST: AtomicPairState = AtomicPairState::new(PairState::Idle);

/// Flag indicating successful completion of pairing login process.
/// Set to true when device has been authenticated and can access mesh functions.
pub static PAIR_LOGIN_OK: AtomicBool = AtomicBool::new(false);

/// Flag indicating whether mesh encryption is currently enabled.
/// When true, all mesh communications are encrypted using the configured LTK.
pub static PAIR_ENC_ENABLE: AtomicBool = AtomicBool::new(false);

/// Write pointer for the light receive buffer circular queue.
/// Points to the next available slot for storing incoming light control packets.
pub static LIGHT_RX_BUFFER_WRITE_POINTER: AtomicUsize = AtomicUsize::new(0);

/// Unique 16-bit address assigned to this device within the mesh network.
/// Used for addressing and routing messages to/from this specific device.
pub static DEVICE_ADDRESS: AtomicU16 = AtomicU16::new(0);

/// Access code derived from mesh name and password for RF communication.
/// 32-bit value used to identify and filter mesh network traffic.
pub static PAIR_AC: AtomicU32 = AtomicU32::new(0);

/// Flag indicating a pairing read operation is waiting for response.
/// Used to track pending configuration read requests during pairing.
pub static PAIR_READ_PENDING: AtomicBool = AtomicBool::new(false);

/// RF transmit power base setting controlling signal strength.
/// Hardware register value determining the baseline transmission power level.
pub static RF_TP_BASE: AtomicU32 = AtomicU32::new(0x1D);

/// RF transmit power gain adjustment for fine-tuning signal strength.
/// Additional gain control applied on top of the base power setting.
pub static RF_TP_GAIN: AtomicU32 = AtomicU32::new(0xC);

/// BLE scan response interval timing control.
/// Determines the delay between receiving scan requests and sending responses.
pub static BLE_SCAN_RESPONSE_INTERVAL_US: AtomicU32 = AtomicU32::new(0x92);

/// Current state of the BLE connection as a peripheral device.
/// Tracks connection status, advertising state, and link layer operations.
pub static BLE_PERIPHERAL_LINK_STATE: crate::sdk::light::AtomicBlePeripheralLinkState =
    crate::sdk::light::AtomicBlePeripheralLinkState::new(
        crate::sdk::light::BlePeripheralLinkState::Disconnected,
    );

/// Timestamp of the last received packet for timeout detection.
/// Used to detect communication timeouts and trigger reconnection procedures.
pub static LAST_PACKET_RECEIVED_TIMESTAMP: AtomicU32 = AtomicU32::new(0);

/// Global security enable flag for mesh communications.
/// When true, all mesh packets are encrypted before transmission.
pub static SECURITY_ENABLE: AtomicBool = AtomicBool::new(false);

/// Counter for the number of mesh listen cycles performed.
/// Used for timing and synchronization of mesh listening periods.
pub static MESH_LISTEN_CYCLE_COUNT: AtomicU32 = AtomicU32::new(0);

/// Sequence number for light connection messages from master.
/// Tracks message ordering in master-slave light control communications.
pub static LIGHT_CONN_SN_MASTER: AtomicU16 = AtomicU16::new(0);

/// Timestamp when the slave connection was established.
/// Used for calculating connection duration and timeout detection.
pub static SLAVE_CONNECTED_TICK: AtomicU32 = AtomicU32::new(0);

/// Flag indicating whether a BLE slave connection is currently active.
/// Set to true when connected to a central device, false when disconnected.
pub static BLE_PERIPHERAL_CONNECTION_ACTIVE: AtomicBool = AtomicBool::new(false);

/// BLE connection interval in microseconds for slave connections.
/// Determines how frequently connection events occur between central and peripheral.
pub static SLAVE_LINK_INTERVAL: AtomicU32 = AtomicU32::new(0x9c400);

/// Current connection window size for slave timing.
/// Defines the time window during which connection events can occur.
pub static SLAVE_WINDOW_SIZE: AtomicU32 = AtomicU32::new(0);

/// Updated connection window size pending application.
/// New window size that will be applied after connection parameter update.
pub static SLAVE_WINDOW_SIZE_UPDATE: AtomicU32 = AtomicU32::new(0);

/// Scheduled timestamp for the next connection event.
/// Used for precise timing of BLE connection intervals.
pub static SLAVE_NEXT_CONNECT_TICK: AtomicU32 = AtomicU32::new(0);

/// Flag indicating OTA update is busy with mesh operations.
/// Prevents mesh interference during firmware update process.
pub static OTA_UPDATE_MESH_OPERATIONS_BLOCKED: AtomicBool = AtomicBool::new(false);

/// Flag indicating OTA update is currently in progress.
/// Blocks other RF operations during firmware update.
pub static OTA_UPDATE_IN_PROGRESS: AtomicBool = AtomicBool::new(false);

/// Current maximum number of active nodes in the mesh network.
/// Dynamically tracks the highest node count for optimization.
pub static MESH_NODE_MAX: AtomicU8 = AtomicU8::new(0);

/// Flag enabling automatic mesh node status reporting.
/// When true, device periodically reports status to mesh network.
pub static MESH_NODE_REPORT_ENABLE: AtomicBool = AtomicBool::new(false);

/// Flag indicating successful completion of connection parameter update.
/// Set when BLE connection parameters have been successfully negotiated.
pub static CONN_UPDATE_SUCCESSED: AtomicBool = AtomicBool::new(false);

/// Counter tracking connection parameter update attempts.
/// Used to limit retry attempts and detect update failures.
pub static CONN_UPDATE_CNT: AtomicUsize = AtomicUsize::new(0);

/// Flag indicating UUID configuration has been set.
/// Tracks whether device UUID has been properly configured.
pub static SET_UUID_FLAG: AtomicBool = AtomicBool::new(false);

/// Maximum allowed length for mesh network names.
/// Enforces string length limits for mesh configuration.
pub static MAX_MESH_NAME_LEN: AtomicUsize = AtomicUsize::new(16);
/// LED controller state managing all LED operations.
/// Encapsulates timing, patterns, and current state for LED control.
pub struct LedControllerState {
    /// Bitmask of pending LED events waiting to be processed
    pub event_pending: AtomicU32,
    /// Counter for LED blink cycles in current sequence
    pub blink_count: AtomicU32,
    /// LED on-time duration in microseconds for blink patterns
    pub on_duration_us: AtomicU32,
    /// LED off-time duration in microseconds for blink patterns
    pub off_duration_us: AtomicU32,
    /// LED selection mask indicating which LEDs are active
    pub led_selection_mask: AtomicU32,
    /// Current timestamp for LED timing control
    pub timing_tick: AtomicU32,
    /// LED pattern number or sequence identifier
    pub pattern_number: AtomicU32,
    /// Current LED state: 1 for on, 0 for off
    pub is_on: AtomicU32,
}

impl LedControllerState {
    pub const fn new() -> Self {
        Self {
            event_pending: AtomicU32::new(0),
            blink_count: AtomicU32::new(0),
            on_duration_us: AtomicU32::new(0),
            off_duration_us: AtomicU32::new(0),
            led_selection_mask: AtomicU32::new(0),
            timing_tick: AtomicU32::new(0),
            pattern_number: AtomicU32::new(0),
            is_on: AtomicU32::new(0),
        }
    }
}

/// Global LED controller state for device status indication.
pub static LED_CONTROLLER: LedControllerState = LedControllerState::new();

/// Flag indicating mesh device address configuration is pending MAC validation.
/// When true: Device address configuration requires MAC address validation before completion.
/// When false: MAC address validation completed successfully, device address is configured.
/// Used during mesh pairing to track the device address configuration state.
pub static MESH_DEVICE_ADDRESS_VALIDATION_PENDING: AtomicBool = AtomicBool::new(false);

/// Current BLE Link Layer channel number for communication.
/// Tracks which of the 37 BLE channels is currently in use.
pub static BLE_LL_CHANNEL_NUM: AtomicUsize = AtomicUsize::new(0);

/// Last unmapped channel in the BLE channel hopping sequence.
/// Used for channel selection algorithm and interference avoidance.
pub static BLE_LL_LAST_UNMAPPED_CH: AtomicUsize = AtomicUsize::new(0);
/// GATT service discovery timeout tracking timestamp.
/// Used to detect when GATT service discovery has taken too long.
pub static GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP: AtomicU32 = AtomicU32::new(0);

/// BLE peripheral connection timeout value in microseconds.
/// Maximum time to wait before considering the connection lost.
pub static BLE_PERIPHERAL_CONNECTION_TIMEOUT_US: AtomicU32 = AtomicU32::new(0);

/// Timestamp for BLE peripheral timing update procedures.
/// Tracks when connection parameter updates were initiated.
pub static BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP: AtomicU32 = AtomicU32::new(0);

/// Flag indicating timing update timestamp 2 is active.
/// Used for secondary timing update coordination.
pub static BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG: AtomicBool = AtomicBool::new(false);

/// Timestamp for secondary timing update procedure OK time.
/// Tracks when secondary timing updates were confirmed.
pub static BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME: AtomicU32 = AtomicU32::new(0);

/// Next instant value for BLE connection parameter updates.
/// BLE connection event number when new parameters take effect.
pub static BLE_PERIPHERAL_NEXT_UPDATE_INSTANT: AtomicU16 = AtomicU16::new(0);

/// Previous connection interval before parameter update.
/// Stored for rollback in case of parameter update failure.
pub static BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL: AtomicU32 = AtomicU32::new(0);
/// BLE connection supervision timeout in 10ms units.
/// Maximum time between successful communication before connection is dropped.
pub static BLE_CONN_TIMEOUT: AtomicU32 = AtomicU32::new(0);

/// BLE connection interval in 1.25ms units.
/// Time between consecutive connection events.
pub static BLE_CONN_INTERVAL: AtomicU32 = AtomicU32::new(0);

/// BLE connection offset timing adjustment.
/// Fine-tuning for connection event timing synchronization.
pub static BLE_CONN_OFFSET: AtomicU32 = AtomicU32::new(0);
/// Factory reset detection state manager.
/// Tracks reset counts and timing for factory reset sequence detection.
pub struct FactoryResetState {
    /// Flash address index for reset counter storage
    pub flash_address_index: AtomicU32,
    /// Number of consecutive resets for factory reset detection
    pub consecutive_reset_count: AtomicU8,
    /// Clear state counter for reset sequence tracking
    pub clear_state: AtomicU8,
    /// Timestamp for reset counter timeout checking
    pub timing_check_timestamp: AtomicU32,
}

impl FactoryResetState {
    pub const fn new() -> Self {
        Self {
            flash_address_index: AtomicU32::new(0),
            consecutive_reset_count: AtomicU8::new(0),
            clear_state: AtomicU8::new(3),
            timing_check_timestamp: AtomicU32::new(0),
        }
    }
}

/// Global factory reset detection state.
pub static FACTORY_RESET_STATE: FactoryResetState = FactoryResetState::new();

/// Timestamp of the first successful slave connection.
/// Used for connection establishment timing and statistics.
pub static BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP: AtomicU32 = AtomicU32::new(0);
/// Device node sequence number for mesh communication.
/// Incremental counter used for message sequencing and duplicate detection.
pub static DEVICE_NODE_SN: AtomicU8 = AtomicU8::new(1);

/// Next position index for device group address allocation.
/// Points to the next available slot in the group address table.
pub static MESH_GROUP_ADDRESS_NEXT_POSITION: AtomicU16 = AtomicU16::new(0);

/// Flash configuration index for persistent storage operations.
/// Current position in flash memory for configuration data.
pub static FLASH_CONFIGURATION_INDEX: AtomicI32 = AtomicI32::new(0);

/// Status indicating slave is busy reading mesh node states.
/// Prevents concurrent status reading operations.
pub static SLAVE_READ_STATUS_BUSY: AtomicU8 = AtomicU8::new(0);
/// Current flash address for OTA firmware update operations.
/// Points to the next location where firmware data will be written.
pub static OTA_UPDATE_CURRENT_FLASH_ADDRESS: AtomicU32 = AtomicU32::new(0);

/// Flag to terminate ongoing OTA update process.
/// Set to abort firmware update and return to normal operation.
pub static OTA_UPDATE_TERMINATION_REQUESTED: AtomicBool = AtomicBool::new(false);

/// OTA operation timeout in seconds (default 30 seconds).
/// Maximum time allowed for firmware update operations.
pub static OTA_UPDATE_TIMEOUT_SECONDS: AtomicU16 =
    AtomicU16::new(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);
/// Next position for device address allocation in mesh network.
/// Used for assigning unique addresses to new mesh devices.
pub static MESH_DEVICE_ADDRESS_NEXT_POSITION: AtomicU16 = AtomicU16::new(0);

/// Flag indicating connection parameters need to be updated.
/// Set when BLE connection parameters require modification.
pub static NEED_UPDATE_CONNECT_PARA: AtomicBool = AtomicBool::new(false);

/// User-requested maximum connection interval limit.
/// Upper bound for BLE connection interval negotiations.
pub static UPDATE_INTERVAL_USER_MAX: AtomicU16 = AtomicU16::new(0);

/// User-requested minimum connection interval limit.
/// Lower bound for BLE connection interval negotiations.
pub static UPDATE_INTERVAL_USER_MIN: AtomicU16 = AtomicU16::new(0);
/// Validation flag for slave data integrity.
/// Ensures slave data structures are in a consistent state.
pub static SLAVE_DATA_VALID: AtomicU32 = AtomicU32::new(0);

/// Timestamp for bridge command operations.
/// Used for timing bridge/relay operations in mesh network.
pub static BRIDGE_COMMAND_TIMESTAMP: AtomicU32 = AtomicU32::new(0);

/// Bridge sequence number for mesh relay operations.
/// Tracks the current bridge/relay state and operation count.
pub static BRIDGE_SEQUENCE_NUMBER: AtomicU32 = AtomicU32::new(0);
/// Write pointer for the device status buffer circular queue.
/// Points to the next position for storing incoming status data.
pub static DEVICE_STATUS_BUFFER_WRITE_POINTER: AtomicUsize = AtomicUsize::new(0);

/// Read pointer for the device status buffer circular queue.
/// Points to the next status entry to be processed.
pub static DEVICE_STATUS_BUFFER_READ_POINTER: AtomicUsize = AtomicUsize::new(0);

/// Flag indicating unicast status reading is active.
/// When true, status reads are directed to specific devices rather than broadcast.
pub static DEVICE_STATUS_READ_UNICAST_MODE: AtomicBool = AtomicBool::new(false);
/// Flag enabling automatic peripheral timing adjustment.
/// When true, peripheral device adjusts its timing to maintain synchronization.
pub static BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED: AtomicBool = AtomicBool::new(false);

/// Device timing tick for bridge receive operations.
/// Timestamp used for mesh bridge receive timing coordination.
pub static BRIDGE_RECEIVE_TIMING_TICK: AtomicU32 = AtomicU32::new(0);

/// Window offset for BLE peripheral connection timing.
/// Adjustment applied to connection window for timing synchronization.
pub static BLE_PERIPHERAL_WINDOW_OFFSET: AtomicU32 = AtomicU32::new(0);

/// Current instant value for BLE peripheral connection events.
/// BLE connection event counter for timing coordination.
pub static BLE_PERIPHERAL_CONNECTION_INSTANT: AtomicU16 = AtomicU16::new(0);

/// Status tick counter for device state machine.
/// Used for timing device status operations and state transitions.
pub static DEVICE_STATUS_TICK_COUNTER: AtomicU8 = AtomicU8::new(0);

/// Current command state for BLE peripheral link operations.
/// Tracks which command is being processed by the peripheral device.
pub static BLE_PERIPHERAL_LINK_COMMAND: AtomicU8 = AtomicU8::new(0);

/// Current index into the device status record table.
/// Points to the active entry in the status tracking array.
pub static DEVICE_STATUS_RECORD_INDEX: AtomicUsize = AtomicUsize::new(0);

/// Notification request mask index for tracking pending requests.
/// Used to track which devices have pending notification requests.
pub static NOTIFICATION_REQUEST_MASK_INDEX: AtomicU8 = AtomicU8::new(0);
/// Flag controlling BLE advertising state.
/// When true, device actively advertises its presence for discovery.
pub static BLE_ADVERTISING_ENABLED: AtomicBool = AtomicBool::new(true);

/// Flag indicating device online status in mesh network.
/// When true, device is considered active and reachable by other nodes.
pub static MESH_DEVICE_ONLINE_STATUS: AtomicBool = AtomicBool::new(true);

/// Timestamp when device became busy reading status information.
/// Used to detect and timeout stuck status reading operations.
pub static DEVICE_STATUS_READ_BUSY_TIMESTAMP: AtomicU32 = AtomicU32::new(0);

/// Listen interval for mesh operations in microseconds.
/// Defines how often the device listens for mesh network traffic.
pub static MESH_LISTEN_INTERVAL_US: AtomicU32 = AtomicU32::new(0);

/// Flag enabling BLE peripheral advertising functionality.
/// Controls whether the peripheral device can advertise for BLE connections.
pub static BLE_PERIPHERAL_ADVERTISING_ENABLED: AtomicBool = AtomicBool::new(false);

/// Flag enabling BLE peripheral connection acceptance.
/// When true, peripheral device accepts incoming BLE connection requests.
pub static BLE_PERIPHERAL_CONNECTION_ENABLED: AtomicBool = AtomicBool::new(false);

pub trait SimplifyLS<T> {
    fn get(&self) -> T;
    fn set(&self, val: T);
    fn inc(&self);
    fn dec(&self);
}

impl SimplifyLS<bool> for AtomicBool {
    fn get(&self) -> bool {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: bool) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) {
        panic!("Not valid for bool");
    }
    fn dec(&self) {
        panic!("Not valid for bool")
    }
}

impl SimplifyLS<u8> for AtomicU8 {
    fn get(&self) -> u8 {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: u8) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) {
        self.set(self.get().wrapping_add(1));
    }
    fn dec(&self) {
        self.set(self.get().wrapping_sub(1));
    }
}

impl SimplifyLS<u16> for AtomicU16 {
    fn get(&self) -> u16 {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: u16) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) {
        self.set(self.get().wrapping_add(1));
    }
    fn dec(&self) {
        self.set(self.get().wrapping_sub(1));
    }
}

impl SimplifyLS<u32> for AtomicU32 {
    fn get(&self) -> u32 {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: u32) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) {
        self.set(self.get().wrapping_add(1));
    }
    fn dec(&self) {
        self.set(self.get().wrapping_sub(1));
    }
}

impl SimplifyLS<usize> for AtomicUsize {
    fn get(&self) -> usize {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: usize) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) {
        self.set(self.get().wrapping_add(1));
    }
    fn dec(&self) {
        self.set(self.get().wrapping_sub(1));
    }
}

impl SimplifyLS<i32> for AtomicI32 {
    fn get(&self) -> i32 {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: i32) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) {
        self.set(self.get().wrapping_add(1));
    }
    fn dec(&self) {
        self.set(self.get().wrapping_sub(1));
    }
}

/// Represents the current state of the BLE pairing process
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PairState {
    Idle = 0x00,                  // Initial/idle state
    AwaitingRandom = 0x02,        // Waiting for random challenge
    ReceivingMeshName = 0x05,     // Receiving mesh name
    ReceivingMeshPassword = 0x06, // Receiving mesh password
    ReceivingMeshLtk = 0x07,      // Receiving long-term key
    RequestingLtk = 0x09,         // Requesting LTK
    ResetMesh = 0x0A,             // Reset mesh configuration
    DeletePairing = 0x0B,         // Deleting pairing
    RandomConfirmation = 0x0C,    // Random value confirmation
    SessionKeyExchange = 0x0D,    // Session key exchange
    Init = 0x0E,                  // Initialization state
    Completed = 0x0F,             // Pairing process completed
}

impl From<u8> for PairState {
    fn from(value: u8) -> Self {
        match value {
            0x00 => Self::Idle,
            0x02 => Self::AwaitingRandom,
            0x05 => Self::ReceivingMeshName,
            0x06 => Self::ReceivingMeshPassword,
            0x07 => Self::ReceivingMeshLtk,
            0x09 => Self::RequestingLtk,
            0x0A => Self::ResetMesh,
            0x0B => Self::DeletePairing,
            0x0C => Self::RandomConfirmation,
            0x0D => Self::SessionKeyExchange,
            0x0E => Self::Init,
            0x0F => Self::Completed,
            _ => Self::Idle, // Default to Idle for unrecognized states
        }
    }
}

// Replace the raw atomic with an atomic PairState wrapper
pub struct AtomicPairState(AtomicU8);

impl AtomicPairState {
    pub const fn new(state: PairState) -> Self {
        Self(AtomicU8::new(state as u8))
    }

    pub fn get(&self) -> PairState {
        PairState::from(self.0.load(Ordering::Relaxed))
    }

    pub fn set(&self, value: PairState) {
        self.0.store(value as u8, Ordering::Relaxed)
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use core::sync::atomic::{
        AtomicBool, AtomicI32, AtomicU16, AtomicU32, AtomicU8, AtomicUsize, Ordering,
    };

    #[test]
    fn test_led_controller_state_new_initializes_zero() {
        let state = LedControllerState::new();

        assert_eq!(state.event_pending.load(Ordering::Relaxed), 0);
        assert_eq!(state.blink_count.load(Ordering::Relaxed), 0);
        assert_eq!(state.on_duration_us.load(Ordering::Relaxed), 0);
        assert_eq!(state.off_duration_us.load(Ordering::Relaxed), 0);
        assert_eq!(state.led_selection_mask.load(Ordering::Relaxed), 0);
        assert_eq!(state.timing_tick.load(Ordering::Relaxed), 0);
        assert_eq!(state.pattern_number.load(Ordering::Relaxed), 0);
        assert_eq!(state.is_on.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_factory_reset_state_new_defaults() {
        let state = FactoryResetState::new();

        assert_eq!(state.flash_address_index.load(Ordering::Relaxed), 0);
        assert_eq!(state.consecutive_reset_count.load(Ordering::Relaxed), 0);
        assert_eq!(state.clear_state.load(Ordering::Relaxed), 3);
        assert_eq!(state.timing_check_timestamp.load(Ordering::Relaxed), 0);
    }

    #[test]
    #[should_panic(expected = "Not valid for bool")]
    fn test_simplifyls_bool_inc_panics() {
        use super::SimplifyLS;

        let flag = AtomicBool::new(false);
        flag.inc();
    }

    #[test]
    #[should_panic(expected = "Not valid for bool")]
    fn test_simplifyls_bool_dec_panics() {
        use super::SimplifyLS;

        let flag = AtomicBool::new(false);
        flag.dec();
    }

    #[test]
    fn test_simplifyls_u8_get_set_and_wraps() {
        use super::SimplifyLS;

        let value = AtomicU8::new(0);
        value.set(10);
        assert_eq!(value.get(), 10);

        value.set(u8::MAX);
        value.inc();
        assert_eq!(value.get(), 0);

        value.set(0);
        value.dec();
        assert_eq!(value.get(), u8::MAX);
    }

    #[test]
    fn test_simplifyls_u16_get_set_and_wraps() {
        use super::SimplifyLS;

        let value = AtomicU16::new(0);
        value.set(42);
        assert_eq!(value.get(), 42);

        value.set(u16::MAX);
        value.inc();
        assert_eq!(value.get(), 0);

        value.set(0);
        value.dec();
        assert_eq!(value.get(), u16::MAX);
    }

    #[test]
    fn test_simplifyls_u32_get_set_and_wraps() {
        use super::SimplifyLS;

        let value = AtomicU32::new(0);
        value.set(100);
        assert_eq!(value.get(), 100);

        value.set(u32::MAX);
        value.inc();
        assert_eq!(value.get(), 0);

        value.set(0);
        value.dec();
        assert_eq!(value.get(), u32::MAX);
    }

    #[test]
    fn test_simplifyls_usize_get_set_and_wraps() {
        use super::SimplifyLS;

        let value = AtomicUsize::new(0);
        value.set(7);
        assert_eq!(value.get(), 7);

        value.set(usize::MAX);
        value.inc();
        assert_eq!(value.get(), 0);

        value.set(0);
        value.dec();
        assert_eq!(value.get(), usize::MAX);
    }

    #[test]
    fn test_simplifyls_i32_get_set_and_wraps() {
        use super::SimplifyLS;

        let value = AtomicI32::new(0);
        value.set(-12);
        assert_eq!(value.get(), -12);

        value.set(i32::MAX);
        value.inc();
        assert_eq!(value.get(), i32::MIN);

        value.set(i32::MIN);
        value.dec();
        assert_eq!(value.get(), i32::MAX);
    }

    #[test]
    fn test_pair_state_from_known_values() {
        assert_eq!(PairState::from(0x0A), PairState::ResetMesh);
        assert_eq!(PairState::from(0xFF), PairState::Idle);
    }

    #[test]
    fn test_atomic_pair_state_get_set() {
        let state = AtomicPairState::new(PairState::Init);
        assert_eq!(state.get(), PairState::Init);

        state.set(PairState::Completed);
        assert_eq!(state.get(), PairState::Completed);
    }
}
