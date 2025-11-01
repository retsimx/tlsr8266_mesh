use crate::sdk::factory_reset::CFG_ADR_MAC_512K_FLASH;
use crate::BIT;

pub const PAIR_CONFIG_VALID_FLAG: u8 = 0xFA;
pub const RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS: u16 = 30;
pub const LOOP_INTERVAL_US: u64 = 10000;

// must be 255
pub const PMW_MAX_TICK_BASE: u16 = 255;

// 209: freq = 600.4Hz
// 125: freq = 1003.9Hz
// 41: freq = 3060.7Hz
// More detail about why this should be above 3kHz can be found in this document:
// https://bio-licht.org/02_resources/info_ieee-pem_2014-09_led-flickering.pdf
pub const PMW_MAX_TICK_MULTI: u16 = 41;

// must less or equal than (255*256)
pub const PMW_MAX_TICK: u16 = PMW_MAX_TICK_BASE * PMW_MAX_TICK_MULTI;

pub const BRIDGE_MAX_CNT: u32 = 8;
pub const IRQ_TIMER1_ENABLE: bool = true;

pub const ONLINE_STATUS_TIMEOUT: u32 = 3000;

pub const AUTH_TIME: u32 = 60;
pub const MAX_GROUP_COUNT: u8 = 8;

pub const START_UP_FLAG: u32 = 0x544c4e4b;

//app use
pub const PAR_READ_MESH_PAIR_CONFIRM: u8 = 0x0a;

// op cmd 11xxxxxxzzzzzzzzzzzzzzzz z's=VENDOR_ID  xxxxxx=LGT_CMD_
pub const LGT_CMD_NOTIFY_MESH: u8 = 0x02;
//internal use
pub const LGT_CMD_MESH_PAIR: u8 = 0x09;
pub const LGT_CMD_MESH_CMD_NOTIFY: u8 = 0x0a;
pub const CMD_NOTIFY_MESH_PAIR_END: u8 = 0x00;

pub const LGT_CMD_LIGHT_ONOFF: u8 = 0x10;
//internal use
pub const LGT_CMD_LIGHT_GRP_RSP1: u8 = 0x14;
//get group rsp: 8groups low 1bytes
pub const LGT_CMD_LIGHT_GRP_RSP2: u8 = 0x15;
//get group rsp: front 4groups 2bytes
pub const LGT_CMD_LIGHT_GRP_RSP3: u8 = 0x16;
//get group rsp: behind 4groups 2bytes
pub const LGT_CMD_LIGHT_CONFIG_GRP: u8 = 0x17;
//add or del group
//internal use
pub const LGT_CMD_LIGHT_READ_STATUS: u8 = 0x1a;
//get status req
pub const LGT_CMD_LIGHT_STATUS: u8 = 0x1b;
//get status rsp
pub const LGT_CMD_LIGHT_GRP_REQ: u8 = 0x1d;
//get group req
pub const LGT_CMD_CONFIG_DEV_ADDR: u8 = 0x20;
//add device address
pub const DEV_ADDR_PAR_WITH_MAC: u8 = 0x01;
pub const LGT_CMD_DEV_ADDR_RSP: u8 = 0x21;
//rsp
pub const LGT_CMD_KICK_OUT: u8 = 0x23;

pub const LGT_CMD_START_OTA_REQ: u8 = 0x24;
pub const LGT_CMD_START_OTA_RSP: u8 = 0x25;
pub const LGT_CMD_OTA_DATA_REQ: u8 = 0x26;
pub const LGT_CMD_OTA_DATA_RSP: u8 = 0x27;
pub const LGT_CMD_END_OTA_REQ: u8 = 0x28;
pub const LGT_CMD_END_OTA_RSP: u8 = 0x29;

pub const LGT_CMD_USER_NOTIFY_REQ: u8 = 0x2a;
pub const LGT_CMD_USER_NOTIFY_RSP: u8 = 0x2b;

pub const LGT_CMD_SET_LIGHT: u8 = 0x30;
pub const LGT_CMD_SET_MAC_ADDR: u8 = 0x31;
pub const LGT_POWER_ON: u8 = 0x32;
pub const LGT_PANIC_MSG: u8 = 0x34;

// Sent to acknowledge a non-notify message
pub const LGT_CMD_LIGHT_ACK: u8 = 0x35;

pub const GET_STATUS: u8 = 0;
pub const GET_GROUP1: u8 = 1;
// return 8 GROUP_ADDRESS(low 1byte)
pub const GET_GROUP2: u8 = 2;
// return front 4 GROUP_ADDRESS
pub const GET_GROUP3: u8 = 3;
// return behind 4 GROUP_ADDRESS
pub const GET_DEV_ADDR: u8 = 4;
// return device address
pub const GET_USER_NOTIFY: u8 = 7;
// return user notify info
pub const CMD_START_OTA: u8 = 8;
pub const CMD_OTA_DATA: u8 = 9;
pub const CMD_END_OTA: u8 = 10;

pub const LGT_CMD_SET_MESH_INFO: u8 = 0xc5;
pub const LGT_CMD_SET_DEV_ADDR: u8 = 0xc6;
pub const LGT_CMD_DEL_PAIR: u8 = 0xc7;
pub const LGT_CMD_MESH_PAIR_TIMEOUT: u8 = 0xcf;

pub const LIGHT_OFF_PARAM: u8 = 0x00;
pub const LIGHT_ON_PARAM: u8 = 0x01;

pub const LIGHT_ADD_GRP_PARAM: u8 = 0x01;
pub const LIGHT_DEL_GRP_PARAM: u8 = 0x00;

pub const DEVICE_ADDR_MASK_DEFAULT: u16 = 0x7FFF;

pub const MESH_NODE_MAX_NUM: usize = 64;
pub const MESH_NODE_MASK_LEN: usize = ((MESH_NODE_MAX_NUM + 31) >> 5);

pub const LIGHT_RX_BUFF_COUNT: usize = 4;
pub const BUFF_RESPONSE_PACKET_COUNT: usize = 16;
pub const BLT_FIFO_TX_PACKET_COUNT: usize = 8;

pub const PKT_CMD_LEN: usize = 11;

pub enum LightOpType {
    OpType1 = 1,
    OpType2 = 2,
    OpType3 = 3,
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub enum ePairState {
    PairSetted = 0,
    PairSetting = 1,
    PairSetMeshTxStart = 2,
    PairSetMeshTxDone = 3, // send notify req, get mesh nodes' ac
    PairSetMeshRxDone = 4, // received all mesh nodes' ac, send cmd to switch to new mesh
}

#[repr(C)]
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum OtaState {
    Continue = 0,
    // must zero
    Ok = 1,
    Error = 2,
    MasterOtaRebootOnly = 3,
}

// recover status before software reboot
pub enum RecoverStatus {
    LightOff = BIT!(0),
    MeshOtaMaster100 = BIT!(1),
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct AdvPrivate {
    pub manufacture_id: u16,
    // must vendor id to follow spec
    pub mesh_product_uuid: u16,
    pub mac_address: u32, // low 4 byte
}

#[derive(Clone, Copy, Default)]
#[repr(C, packed)]
pub struct AdvRspPrivate {
    pub manufacture_id: u16, // 0
    // must vendor id to follow spec
    pub mesh_product_uuid: u16, // 2
    pub mac_address: u32,       // 4
    // low 4 byte
    pub product_uuid: u16,   // 8
    pub status: u8,          // 10
    pub device_address: u16, // 11
    pub rsv: [u8; 16],       // 13
}

#[derive(Clone, Copy)]
#[repr(C, align(4))]
pub struct LightRxBuff {
    pub dma_len: u8,    // 0
    pub unk1: [u8; 3],  // 1
    pub rssi: u8,       // 4
    pub unk2: [u8; 3],  // 5
    pub rx_time: u32,   // 8
    pub sno: [u8; 3],   // 12
    pub unk3: [u8; 5],  // 15
    pub mac: [u8; 4],   // 20
    pub unk4: [u8; 40], // 24
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct StatusRecord {
    pub adr: [u8; 1],
    // don't modify, use internal
    pub alarm_id: u8, // don't modify, use internal
}

#[inline(always)]
pub fn is_unicast_addr(p_addr: &[u8]) -> bool {
    return p_addr[1] & 0x80 == 0;
}

// flash mesh extend shit needed to link
pub const CFG_ADR_CALIBRATION_512K_FLASH: u32 = CFG_ADR_MAC_512K_FLASH + 0x10;
// don't change
pub const CFG_SECTOR_ADR_CALIBRATION_CODE: u32 = CFG_ADR_CALIBRATION_512K_FLASH;

pub const UPDATE_CONNECT_PARA_DELAY_MS: u32 = 1000;

pub const INTERVAL_THRESHOLD: u16 = 16;
pub const ONLINE_STATUS_COMP: u32 = 3;
pub const SLAVE_READ_STATUS_BUSY_TIMEOUT: u32 = 25000;

pub const ADV_INTERVAL2LISTEN_INTERVAL: u16 = 4;
pub const ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL: u16 = 8;

pub const SEND_MESH_STATUS_INTERVAL_MS: u32 = 200;

// Internal parameter array indices for mesh packet processing
pub const INTERNAL_PAR_STATUS_DATA: usize = 0;
pub const INTERNAL_PAR_PACKET_FORMAT_MODE: usize = 1;
pub const INTERNAL_PAR_RETRANSMIT_COUNT: usize = 2;
pub const INTERNAL_PAR_SEND_ACK: usize = 3;

// Packet formatting modes for internal_par1[INTERNAL_PAR_PACKET_FORMAT_MODE]
// Note: Only format mode 0 is actually used - all packets initialize with internal_par1: [0; 5]
pub const PACKET_FORMAT_STANDARD_PARAMS: u8 = 0; // Copy specific parameter values to response (only format actually used)

/// BLE peripheral connection link states.
///
/// This enum represents the different operational states of the BLE peripheral
/// connection state machine. Each state corresponds to a specific mode of operation
/// in the BLE protocol stack and mesh networking system.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BlePeripheralLinkState {
    /// Device is disconnected and idle.
    ///
    /// In this state, the device is not actively communicating via BLE
    /// and may be in a low-power mode or preparing for other operations.
    Disconnected = 0,

    /// Device is actively advertising for BLE connections.
    ///
    /// The device broadcasts advertisement packets to allow central devices
    /// to discover and connect to it. Responds to scan requests and connection requests.
    Advertising = 1,

    /// Device is operating in mesh networking mode.
    ///
    /// Processing and forwarding mesh network packets, participating in
    /// the mesh topology, and handling mesh-specific protocol operations.
    Mesh = 4,

    /// Device has an active BLE connection (bridge mode).
    ///
    /// Connected to a central device and actively processing BLE connection
    /// events, handling data exchange, and maintaining connection timing.
    Connected = 5,

    /// Device is in BLE receive mode.
    ///
    /// Actively listening for incoming BLE packets from the connected central device.
    /// This state is used during connection event processing.
    Receiving = 7,
}

impl BlePeripheralLinkState {
    /// Creates a BlePeripheralLinkState from a raw u8 value.
    ///
    /// Returns None if the value doesn't correspond to a valid state.
    /// This is useful for safely converting from hardware register values.
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Disconnected),
            1 => Some(Self::Advertising),
            4 => Some(Self::Mesh),
            5 => Some(Self::Connected),
            7 => Some(Self::Receiving),
            _ => None,
        }
    }

    /// Returns the raw u8 value for this state.
    ///
    /// This is useful when interfacing with hardware registers or
    /// legacy code that expects numeric values.
    pub fn as_u8(self) -> u8 {
        self as u8
    }

    /// Returns true if the device is in a state where it can accept connections.
    pub fn can_accept_connections(self) -> bool {
        matches!(self, Self::Advertising)
    }

    /// Returns true if the device has an active BLE connection.
    pub fn is_connected(self) -> bool {
        matches!(self, Self::Connected | Self::Receiving)
    }

    /// Returns true if the device is actively processing mesh operations.
    pub fn is_mesh_active(self) -> bool {
        matches!(self, Self::Mesh)
    }
}

/// Atomic wrapper for BLE peripheral link state.
///
/// This provides atomic access to the BLE peripheral link state with
/// type-safe enum operations while maintaining the underlying atomic u8 storage.
pub struct AtomicBlePeripheralLinkState {
    inner: core::sync::atomic::AtomicU8,
}

impl AtomicBlePeripheralLinkState {
    /// Creates a new atomic BLE peripheral link state with the given initial value.
    pub const fn new(state: BlePeripheralLinkState) -> Self {
        Self {
            inner: core::sync::atomic::AtomicU8::new(state as u8),
        }
    }

    /// Loads the current state.
    pub fn get(&self) -> BlePeripheralLinkState {
        let value = self.inner.load(core::sync::atomic::Ordering::Relaxed);
        BlePeripheralLinkState::from_u8(value).unwrap_or_else(|| {
            // Log error or handle invalid state - for now, default to Disconnected
            BlePeripheralLinkState::Disconnected
        })
    }

    /// Stores a new state.
    pub fn set(&self, state: BlePeripheralLinkState) {
        self.inner
            .store(state as u8, core::sync::atomic::Ordering::Relaxed);
    }
}

#[derive(PartialEq, Copy, Clone, Debug)]
pub enum RfOperationState {
    Idle,
    Advertising,
    Connected,
    Receiving,
    MeshListening,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Tests BlePeripheralLinkState enum conversion methods.
    ///
    /// This test verifies that BlePeripheralLinkState can be correctly converted
    /// to and from raw u8 values, and that invalid values return None.
    ///
    /// # Algorithm
    ///
    /// 1. Test valid u8 values convert to correct enum variants
    /// 2. Test invalid u8 values return None
    /// 3. Test as_u8() returns correct raw values
    /// 4. Test round-trip conversion works correctly
    ///
    /// # Notes
    ///
    /// * All valid BLE peripheral link states should be convertible
    /// * Invalid values should be rejected gracefully
    #[test]
    fn test_ble_peripheral_link_state_from_u8() {
        // Test valid conversions
        assert_eq!(
            BlePeripheralLinkState::from_u8(0),
            Some(BlePeripheralLinkState::Disconnected)
        );
        assert_eq!(
            BlePeripheralLinkState::from_u8(1),
            Some(BlePeripheralLinkState::Advertising)
        );
        assert_eq!(
            BlePeripheralLinkState::from_u8(4),
            Some(BlePeripheralLinkState::Mesh)
        );
        assert_eq!(
            BlePeripheralLinkState::from_u8(5),
            Some(BlePeripheralLinkState::Connected)
        );
        assert_eq!(
            BlePeripheralLinkState::from_u8(7),
            Some(BlePeripheralLinkState::Receiving)
        );

        // Test invalid conversions
        assert_eq!(BlePeripheralLinkState::from_u8(2), None);
        assert_eq!(BlePeripheralLinkState::from_u8(3), None);
        assert_eq!(BlePeripheralLinkState::from_u8(6), None);
        assert_eq!(BlePeripheralLinkState::from_u8(8), None);
        assert_eq!(BlePeripheralLinkState::from_u8(255), None);
    }

    /// Tests BlePeripheralLinkState as_u8 conversion.
    ///
    /// This test verifies that each enum variant converts to the correct
    /// raw u8 value as defined by the BLE peripheral link state machine.
    ///
    /// # Algorithm
    ///
    /// 1. Test each enum variant converts to expected u8 value
    ///
    /// # Notes
    ///
    /// * Raw values must match hardware register expectations
    #[test]
    fn test_ble_peripheral_link_state_as_u8() {
        assert_eq!(BlePeripheralLinkState::Disconnected.as_u8(), 0);
        assert_eq!(BlePeripheralLinkState::Advertising.as_u8(), 1);
        assert_eq!(BlePeripheralLinkState::Mesh.as_u8(), 4);
        assert_eq!(BlePeripheralLinkState::Connected.as_u8(), 5);
        assert_eq!(BlePeripheralLinkState::Receiving.as_u8(), 7);
    }

    /// Tests BlePeripheralLinkState round-trip conversion.
    ///
    /// This test ensures that converting from enum to u8 and back
    /// preserves the original value.
    ///
    /// # Algorithm
    ///
    /// 1. Convert each enum variant to u8
    /// 2. Convert back from u8 to enum
    /// 3. Verify the result matches the original
    ///
    /// # Notes
    ///
    /// * Round-trip conversion must be lossless
    #[test]
    fn test_ble_peripheral_link_state_round_trip() {
        let states = [
            BlePeripheralLinkState::Disconnected,
            BlePeripheralLinkState::Advertising,
            BlePeripheralLinkState::Mesh,
            BlePeripheralLinkState::Connected,
            BlePeripheralLinkState::Receiving,
        ];

        for &state in &states {
            let raw = state.as_u8();
            let converted_back = BlePeripheralLinkState::from_u8(raw).unwrap();
            assert_eq!(
                state, converted_back,
                "Round-trip conversion failed for {:?}",
                state
            );
        }
    }

    /// Tests BlePeripheralLinkState::can_accept_connections method.
    ///
    /// This test verifies that the method correctly identifies which states
    /// allow the device to accept new BLE connections.
    ///
    /// # Algorithm
    ///
    /// 1. Test that Advertising state returns true
    /// 2. Test that all other states return false
    ///
    /// # Notes
    ///
    /// * Only Advertising state should accept connections
    #[test]
    fn test_ble_peripheral_link_state_can_accept_connections() {
        assert!(BlePeripheralLinkState::Advertising.can_accept_connections());

        assert!(!BlePeripheralLinkState::Disconnected.can_accept_connections());
        assert!(!BlePeripheralLinkState::Mesh.can_accept_connections());
        assert!(!BlePeripheralLinkState::Connected.can_accept_connections());
        assert!(!BlePeripheralLinkState::Receiving.can_accept_connections());
    }

    /// Tests BlePeripheralLinkState::is_connected method.
    ///
    /// This test verifies that the method correctly identifies which states
    /// represent an active BLE connection.
    ///
    /// # Algorithm
    ///
    /// 1. Test that Connected and Receiving states return true
    /// 2. Test that other states return false
    ///
    /// # Notes
    ///
    /// * Connected and Receiving represent active connections
    #[test]
    fn test_ble_peripheral_link_state_is_connected() {
        assert!(BlePeripheralLinkState::Connected.is_connected());
        assert!(BlePeripheralLinkState::Receiving.is_connected());

        assert!(!BlePeripheralLinkState::Disconnected.is_connected());
        assert!(!BlePeripheralLinkState::Advertising.is_connected());
        assert!(!BlePeripheralLinkState::Mesh.is_connected());
    }

    /// Tests BlePeripheralLinkState::is_mesh_active method.
    ///
    /// This test verifies that the method correctly identifies when
    /// mesh networking operations are active.
    ///
    /// # Algorithm
    ///
    /// 1. Test that Mesh state returns true
    /// 2. Test that all other states return false
    ///
    /// # Notes
    ///
    /// * Only Mesh state represents active mesh operations
    #[test]
    fn test_ble_peripheral_link_state_is_mesh_active() {
        assert!(BlePeripheralLinkState::Mesh.is_mesh_active());

        assert!(!BlePeripheralLinkState::Disconnected.is_mesh_active());
        assert!(!BlePeripheralLinkState::Advertising.is_mesh_active());
        assert!(!BlePeripheralLinkState::Connected.is_mesh_active());
        assert!(!BlePeripheralLinkState::Receiving.is_mesh_active());
    }

    /// Tests AtomicBlePeripheralLinkState creation and basic operations.
    ///
    /// This test verifies that the atomic wrapper can be created and
    /// basic get/set operations work correctly.
    ///
    /// # Algorithm
    ///
    /// 1. Create atomic state with initial value
    /// 2. Verify initial value can be read
    /// 3. Set new value and verify it can be read back
    /// 4. Test multiple state changes
    ///
    /// # Notes
    ///
    /// * Atomic operations must be thread-safe
    /// * State transitions should be preserved
    #[test]
    fn test_atomic_ble_peripheral_link_state() {
        // Test creation with initial state
        let atomic_state = AtomicBlePeripheralLinkState::new(BlePeripheralLinkState::Disconnected);
        assert_eq!(atomic_state.get(), BlePeripheralLinkState::Disconnected);

        // Test setting and getting different states
        atomic_state.set(BlePeripheralLinkState::Advertising);
        assert_eq!(atomic_state.get(), BlePeripheralLinkState::Advertising);

        atomic_state.set(BlePeripheralLinkState::Mesh);
        assert_eq!(atomic_state.get(), BlePeripheralLinkState::Mesh);

        atomic_state.set(BlePeripheralLinkState::Connected);
        assert_eq!(atomic_state.get(), BlePeripheralLinkState::Connected);

        atomic_state.set(BlePeripheralLinkState::Receiving);
        assert_eq!(atomic_state.get(), BlePeripheralLinkState::Receiving);
    }

    /// Tests AtomicBlePeripheralLinkState error handling for invalid states.
    ///
    /// This test verifies that the atomic wrapper gracefully handles
    /// corrupted or invalid internal state values by defaulting to Disconnected.
    ///
    /// # Algorithm
    ///
    /// 1. Create atomic state with valid initial value
    /// 2. Directly corrupt the internal atomic value with an invalid u8
    /// 3. Verify get() returns Disconnected as fallback
    ///
    /// # Notes
    ///
    /// * Invalid states should be handled gracefully
    /// * Default fallback prevents system instability
    #[test]
    fn test_atomic_ble_peripheral_link_state_invalid_value_handling() {
        // Create atomic state with valid initial value
        let atomic_state = AtomicBlePeripheralLinkState::new(BlePeripheralLinkState::Advertising);
        assert_eq!(atomic_state.get(), BlePeripheralLinkState::Advertising);

        // Directly corrupt the internal atomic value with an invalid u8 (255)
        // This simulates memory corruption or other invalid state scenarios
        atomic_state
            .inner
            .store(255, core::sync::atomic::Ordering::Relaxed);

        // Verify that get() gracefully handles the invalid value by returning Disconnected
        assert_eq!(atomic_state.get(), BlePeripheralLinkState::Disconnected);
    }

    /// Tests AdvRspPrivate Default trait implementation.
    ///
    /// This test verifies that the Default trait creates a properly
    /// initialized AdvRspPrivate structure with expected values.
    ///
    /// # Algorithm
    ///
    /// 1. Create default instance
    /// 2. Verify all fields are zero-initialized as expected
    ///
    /// # Notes
    ///
    /// * Default should provide a clean, zero-initialized state
    #[test]
    fn test_adv_rsp_private_default() {
        let default_adv = AdvRspPrivate::default();

        // Copy fields to local variables to avoid unaligned access
        let manufacture_id = default_adv.manufacture_id;
        let mesh_product_uuid = default_adv.mesh_product_uuid;
        let mac_address = default_adv.mac_address;
        let product_uuid = default_adv.product_uuid;
        let status = default_adv.status;
        let device_address = default_adv.device_address;
        let rsv = default_adv.rsv;

        assert_eq!(manufacture_id, 0);
        assert_eq!(mesh_product_uuid, 0);
        assert_eq!(mac_address, 0);
        assert_eq!(product_uuid, 0);
        assert_eq!(status, 0);
        assert_eq!(device_address, 0);
        assert_eq!(rsv, [0; 16]);
    }

    /// Tests the is_unicast_addr function.
    ///
    /// This test verifies that is_unicast_addr correctly identifies
    /// unicast vs group BLE addresses based on the MSB of the second byte.
    ///
    /// # Algorithm
    ///
    /// 1. Test unicast addresses (MSB of second byte = 0)
    /// 2. Test group addresses (MSB of second byte = 1)
    /// 3. Test edge cases
    ///
    /// # Notes
    ///
    /// * BLE unicast addresses have the MSB of the second byte clear
    /// * Group addresses have the MSB of the second byte set
    #[test]
    fn test_is_unicast_addr() {
        // Test unicast addresses (second byte MSB = 0)
        assert!(is_unicast_addr(&[0x01, 0x02, 0x03, 0x04, 0x05, 0x06]));
        assert!(is_unicast_addr(&[0xFF, 0x7F, 0x03, 0x04, 0x05, 0x06])); // 0x7F = 01111111
        assert!(is_unicast_addr(&[0x00, 0x00, 0x03, 0x04, 0x05, 0x06]));

        // Test group addresses (second byte MSB = 1)
        assert!(!is_unicast_addr(&[0x01, 0x82, 0x03, 0x04, 0x05, 0x06])); // 0x82 = 10000010
        assert!(!is_unicast_addr(&[0xFF, 0xFF, 0x03, 0x04, 0x05, 0x06])); // 0xFF = 11111111
        assert!(!is_unicast_addr(&[0x00, 0x80, 0x03, 0x04, 0x05, 0x06])); // 0x80 = 10000000

        // Test edge cases
        assert!(is_unicast_addr(&[0x00, 0x7F, 0x00, 0x00, 0x00, 0x00])); // Maximum unicast
        assert!(!is_unicast_addr(&[0x00, 0x80, 0x00, 0x00, 0x00, 0x00])); // Minimum group
    }

    /// Tests constant values are correctly defined.
    ///
    /// This test verifies that key constants used throughout the light
    /// module have expected values for proper system operation.
    ///
    /// # Algorithm
    ///
    /// 1. Test PWM-related constants
    /// 2. Test mesh networking constants
    /// 3. Test timeout and interval constants
    ///
    /// # Notes
    ///
    /// * Constants must match hardware and protocol requirements
    #[test]
    fn test_light_constants() {
        // PWM constants
        assert_eq!(PMW_MAX_TICK_BASE, 255);
        assert_eq!(PMW_MAX_TICK_MULTI, 41);
        assert_eq!(PMW_MAX_TICK, PMW_MAX_TICK_BASE * PMW_MAX_TICK_MULTI);

        // Mesh constants
        assert_eq!(MAX_GROUP_COUNT, 8);
        assert_eq!(MESH_NODE_MAX_NUM, 64);
        assert_eq!(MESH_NODE_MASK_LEN, ((MESH_NODE_MAX_NUM + 31) >> 5));

        // Timeout and interval constants
        assert_eq!(ONLINE_STATUS_TIMEOUT, 3000);
        assert_eq!(AUTH_TIME, 60);
        assert_eq!(LOOP_INTERVAL_US, 10000);
        assert_eq!(UPDATE_CONNECT_PARA_DELAY_MS, 1000);
        assert_eq!(SLAVE_READ_STATUS_BUSY_TIMEOUT, 25000);
        assert_eq!(SEND_MESH_STATUS_INTERVAL_MS, 200);

        // Buffer size constants
        assert_eq!(LIGHT_RX_BUFF_COUNT, 4);
        assert_eq!(BUFF_RESPONSE_PACKET_COUNT, 16);
        assert_eq!(BLT_FIFO_TX_PACKET_COUNT, 8);
        assert_eq!(PKT_CMD_LEN, 11);

        // Bridge constants
        assert_eq!(BRIDGE_MAX_CNT, 8);

        // OTA constants
        assert_eq!(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS, 30);
    }

    /// Tests that enum discriminant values are correct.
    ///
    /// This test verifies that enum variants have the expected underlying
    /// values as required by the BLE protocol and hardware interfaces.
    ///
    /// # Algorithm
    ///
    /// 1. Test BlePeripheralLinkState discriminant values
    /// 2. Test OtaState discriminant values
    /// 3. Test RecoverStatus bit flag values
    ///
    /// # Notes
    ///
    /// * Discriminant values must match protocol specifications
    #[test]
    fn test_enum_discriminants() {
        // BlePeripheralLinkState values
        assert_eq!(BlePeripheralLinkState::Disconnected as u8, 0);
        assert_eq!(BlePeripheralLinkState::Advertising as u8, 1);
        assert_eq!(BlePeripheralLinkState::Mesh as u8, 4);
        assert_eq!(BlePeripheralLinkState::Connected as u8, 5);
        assert_eq!(BlePeripheralLinkState::Receiving as u8, 7);

        // OtaState values
        assert_eq!(OtaState::Continue as u8, 0);
        assert_eq!(OtaState::Ok as u8, 1);
        assert_eq!(OtaState::Error as u8, 2);
        assert_eq!(OtaState::MasterOtaRebootOnly as u8, 3);

        // RecoverStatus bit flags
        assert_eq!(RecoverStatus::LightOff as u8, 1 << 0); // BIT!(0) = 1
        assert_eq!(RecoverStatus::MeshOtaMaster100 as u8, 1 << 1); // BIT!(1) = 2

        // ePairState values
        assert_eq!(ePairState::PairSetted as u8, 0);
        assert_eq!(ePairState::PairSetting as u8, 1);
        assert_eq!(ePairState::PairSetMeshTxStart as u8, 2);
        assert_eq!(ePairState::PairSetMeshTxDone as u8, 3);
        assert_eq!(ePairState::PairSetMeshRxDone as u8, 4);
    }
}
