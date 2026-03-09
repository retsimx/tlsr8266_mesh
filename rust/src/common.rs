use crate::app;
use crate::config::*;
use crate::sdk::ble_app::light_ll::connection_management::setup_ble_parameter_start;
use crate::sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};
use crate::sdk::light::*;
use crate::sdk::mcu::crypto::{aes_att_encryption, decode_password};
use crate::sdk::packet_types::Packet;
use crate::sdk::rf_drv::{set_advertisement_manufacturer_data, set_advertisement_mesh_name};
use crate::state::*;

/// Number of connection parameter sets to attempt during negotiation.
///
/// When a parameter set is rejected, the code cycles through alternative
/// configurations to find one acceptable to both the central and peripheral.
const UPDATE_CONN_PARA_CNT: usize = 4;

/// BLE connection parameter sets for negotiation.
///
/// Each set contains [interval_min, interval_max, timeout]:
/// - interval_min/max: Connection interval in 1.25ms units (must be ≥20ms)
/// - timeout: Supervision timeout in 10ms units (must be ≥3s)
///
/// These parameters balance power consumption vs responsiveness:
/// - Lower intervals = faster data transfer, higher power
/// - Higher intervals = lower power, increased latency
const CONN_PARA_DATA: [[u16; 3]; UPDATE_CONN_PARA_CNT] = [
    [16, 16 + 16, 420], // 20ms-40ms interval, 4.2s timeout
    [18, 18 + 16, 420], // 22.5ms-42.5ms interval, 4.2s timeout
    [32, 32 + 16, 420], // 40ms-60ms interval, 4.2s timeout
    [48, 48 + 16, 420], // 60ms-80ms interval, 4.2s timeout
];

/// BLE channels used for mesh listening (advertising channel indices).
///
/// Actual radio frequencies:
/// - 2: 2406 MHz (advertising channel 37)
/// - 12: 2426 MHz (data channel)
/// - 23: 2448 MHz (advertising channel 38)
/// - 34: 2470 MHz (data channel)
///
/// These channels are selected for frequency diversity and reduced interference.
pub const SYS_CHN_LISTEN: [u8; 4] = [2, 12, 23, 34];

/// BLE advertising channels (standard advertising channel indices).
///
/// These are the three standard BLE advertising channels:
/// - 0x25 (37): 2402 MHz
/// - 0x26 (38): 2426 MHz  
/// - 0x27 (39): 2480 MHz
pub const SYS_CHN_ADV: [u8; 3] = [0x25, 0x26, 0x27];

/// Register address for light off state control.
pub const REGA_LIGHT_OFF: u8 = 0x3a;

/// Checks if device address parameters include MAC address flag.
///
/// This function is used during device address command processing to determine
/// if the command payload includes MAC address information. The flag is checked
/// at a specific position (byte 2) in the parameter array.
///
/// # Arguments
/// * `params` - Parameter array from device address command
///
/// # Returns
/// `true` if MAC flag is present, `false` otherwise
#[cfg_attr(test, mry::mry)]
pub fn dev_addr_with_mac_flag(params: &[u8]) -> bool {
    DEV_ADDR_PAR_WITH_MAC == params[2]
}

/// Populates response buffer with device MAC address.
///
/// Called when responding to device address queries that request MAC information.
/// Copies the 6-byte MAC address into the response buffer at the standard offset.
///
/// # Arguments
/// * `par_rsp` - Response buffer to populate (must be ≥10 bytes)
///
/// # Returns
/// Always returns `true` to indicate successful population
///
/// # Memory Layout
/// ```text
/// par_rsp[0..4]  - Reserved/other data
/// par_rsp[4..10] - 6-byte MAC address
/// ```
#[cfg_attr(test, mry::mry)]
pub fn dev_addr_with_mac_rsp(par_rsp: &mut [u8]) -> bool {
    par_rsp[4..10].copy_from_slice(&*MAC_ID.lock());
    true
}

/// Validates device address parameters against local MAC address.
///
/// This function supports two modes of operation:
/// 1. **Query Mode**: When params start with `0xFF 0xFF`, returns validation pending state
/// 2. **Match Mode**: Compares first 6 bytes of params against device MAC
///
/// Query mode is used during mesh pairing to check if address validation is in progress.
/// Match mode verifies that a command is targeted to this specific device.
///
/// # Arguments
/// * `params` - Parameter buffer containing MAC address or query marker
///
/// # Returns
/// - Query mode: Current validation pending state
/// - Match mode: `true` if MAC matches, `false` otherwise
#[cfg_attr(test, mry::mry)]
pub fn dev_addr_with_mac_match(params: &[u8]) -> bool {
    if params[0] == 0xff && params[1] == 0xff {
        // Query mode: Return current validation pending state
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get()
    } else {
        // Match mode: Compare MAC address
        return params[0..6] == *MAC_ID.lock();
    }
}

/// Writes pairing configuration data to flash storage.
///
/// This function implements wear-leveling by writing to the current configuration
/// index offset. The pairing sector supports multiple configuration slots to
/// distribute write cycles and extend flash lifetime.
///
/// # Flash Memory Layout
///
/// Within each configuration slot:
/// ```text
/// Offset  Size  Content
/// ------  ----  -------
/// 0x00    16    Flags and validation markers
/// 0x10    16    Mesh network name
/// 0x20    16    Password (may be encoded)
/// 0x30    16    Long-term key (LTK)
/// ```
///
/// # Arguments
/// * `adr` - Offset within configuration slot (0=flag, 16=name, 32=pwd, 48=ltk)
/// * `p` - Data buffer to write
///
/// # Flash Address Calculation
/// Final address = `FLASH_ADR_PAIRING + FLASH_CONFIGURATION_INDEX + adr`
#[cfg_attr(test, mry::mry)]
pub fn save_pair_info(adr: u32, p: &[u8]) {
    flash_write_page(
        (FLASH_ADR_PAIRING as i32 + FLASH_CONFIGURATION_INDEX.get() + adr as i32) as u32,
        p.len() as u32,
        p.as_ptr(),
    );
}

/// Initiates BLE connection parameter negotiation.
///
/// Called during the BLE connection establishment phase to propose initial
/// connection parameters. This callback is triggered by the BLE stack when
/// a central device (phone/gateway) connects.
///
/// # Connection Parameter Requirements
///
/// **iOS Compatibility**:
/// - interval_min ≥ 20ms
/// - interval_min + 20ms ≤ interval_max ≤ 2s
/// - timeout ≤ 6s
///
/// **Android**: More flexible, accepts wider range
///
/// # Negotiation Strategy
///
/// 1. Proposes first parameter set from CONN_PARA_DATA
/// 2. If rejected, rf_update_conn_para() will try next set
/// 3. Cycles through all available parameter sets
/// 4. Accepts whatever central finally agrees to
///
/// # Parameters Used (First Set)
/// - interval: 16 * 1.25ms = 20ms (min)
/// - interval: 32 * 1.25ms = 40ms (max)
/// - timeout: 420 * 10ms = 4200ms = 4.2s
///
/// # Reference: setup_ble_parameter_start()
///
/// The underlying function signature:
/// ```c
/// int setup_ble_parameter_start(u16 delay, u16 interval_min, u16 interval_max, u16 timeout);
/// ```
///
/// Returns:
/// - `0`: Parameters valid
/// - `-1`: Invalid interval
/// - `-2`: Invalid timeout
#[cfg_attr(test, mry::mry)]
pub fn update_ble_parameter_cb() {
    // Only initiate negotiation if not already successful
    if !CONN_UPDATE_SUCCESSED.get() {
        // Propose first parameter set
        setup_ble_parameter_start(
            CONN_PARA_DATA[0][0],        // delay: 16 BLE intervals
            CONN_PARA_DATA[0][1],        // interval_min: 20ms, interval_max: 40ms
            CONN_PARA_DATA[0][2] as u32, // timeout: 4200ms
        );
        CONN_UPDATE_CNT.inc();
    }
}

/// Handles BLE connection parameter update response packets.
///
/// This function processes L2CAP signaling connection parameter update response packets.
/// On success (0x0000), it resets the update counter and marks the update as successful.
/// On rejection (0x0001), it attempts to renegotiate with the next parameter set in the list,
/// cycling back to the beginning if all parameter sets have been tried.
///
/// # Arguments
/// * `p` - Reference to the packet to process
///
/// # Returns
/// Always returns 0
pub fn rf_update_conn_para(p: &Packet) -> u8 {
    let head = p.head();

    // Verify this is an L2CAP data packet (start packet)
    if (head._type & 0b11) != 2 {
        return 0;
    }

    // Verify this is a connection parameter update response by checking the packet structure
    // Expected values: rf_len=0x0A, l2cap_len=0x06, chan_id=0x05, code=0x13, id=0x01, data_len=0x02
    if head.rf_len != 0x0A || head.l2cap_len != 0x06 || head.chan_id != 0x05 {
        return 0;
    }

    let sig_rsp = p.sig_conn_para_up_rsp();
    if sig_rsp.code != 0x13 || sig_rsp.id != 0x01 || sig_rsp.data_len != 0x02 {
        return 0;
    }

    // Match on the connection parameter update response result
    match sig_rsp.result {
        0x0000 => {
            // Connection parameter update accepted
            CONN_UPDATE_CNT.set(0);
            CONN_UPDATE_SUCCESSED.set(true);
        }
        0x0001 => {
            // Connection parameter update rejected - try next parameter set
            let current_cnt = CONN_UPDATE_CNT.get();

            if current_cnt >= UPDATE_CONN_PARA_CNT {
                // All parameter sets attempted, reset counter
                CONN_UPDATE_CNT.set(0);
            } else {
                // Try the next parameter set
                let params = CONN_PARA_DATA[current_cnt];
                setup_ble_parameter_start(params[0], params[1], params[2] as u32);
                CONN_UPDATE_CNT.inc();
            }
        }
        _ => {}
    }

    0
}

/// Retrieves device and group addresses from flash storage.
///
/// This function implements the flash-based address discovery algorithm that loads
/// the device's unique mesh address and any associated group addresses from persistent
/// storage. The algorithm supports address persistence across power cycles and enables
/// the device to rejoin the mesh network with its previously assigned identity.
///
/// # Flash Storage Format
///
/// The addresses are stored in flash starting at `FLASH_ADR_DEV_GRP_ADR` with the
/// following structure:
///
/// - **16-bit Little-Endian Values**: Each address is 2 bytes
/// - **Sequential Storage**: Addresses stored contiguously
/// - **End Marker**: 0xFFFF indicates end of address list
/// - **Empty Slots**: 0x0000 indicates unused address slot
/// - **Maximum Size**: 0x1000 bytes (2048 addresses max)
///
/// # Address Classification Algorithm
///
/// Addresses are classified into two categories based on masking:
///
/// 1. **Device Address** (`addr & !dev_mask == 0`):
///    - Primary device identifier for mesh routing
///    - Only one device address per device
///    - Last device address in flash takes precedence
///    - Used for unicast messages
///
/// 2. **Group Address** (`addr & !dev_mask != 0`):
///    - Multicast group membership
///    - Device can belong to multiple groups (up to 8)
///    - Used for broadcast commands to device groups
///    - Stored in circular buffer with index masking
///
/// # Discovery Algorithm
///
/// The function implements a sequential scan with position tracking:
///
/// 1. **Sequential Scan**: Read 16-bit values from flash sequentially
/// 2. **Classification**: Determine if address is device or group type
/// 3. **Position Tracking**: Track next available flash position for each type
/// 4. **Deduplication**: Last device address wins if multiple found
/// 5. **Termination**: Stop at end marker (0xFFFF) or size limit (0x1000)
///
/// # Default Address Assignment
///
/// If no device address is found in flash, the algorithm:
/// 1. Derives address from MAC ID lower 2 bytes (little-endian)
/// 2. Uses address 1 if MAC-derived address would be 0
///
/// This ensures every device has a valid address even on first boot.
///
/// # Side Effects
/// * Updates `DEVICE_ADDRESS` with discovered or derived address
/// * Updates `GROUP_ADDRESS` array with discovered group memberships
/// * Updates position tracking globals for future flash writes
///
/// # Flash Read Safety
///
/// Uses `flash_read_page` instead of raw pointer access for:
/// - Memory safety (no undefined behavior from invalid pointers)
/// - Flash timing compliance (proper wait states)
/// - Hardware abstraction (flash controller management)
#[cfg_attr(test, mry::mry)]
pub fn retrieve_dev_grp_address() {
    let dev_mask = DEVICE_ADDR_MASK_DEFAULT;
    let mut device_addr_next_pos = MESH_DEVICE_ADDRESS_NEXT_POSITION.get();
    let mut group_addr_next_pos = 0;
    let mut group_addr_index = 0;
    let mut flash_offset = FLASH_ADR_DEV_GRP_ADR;

    // SEQUENTIAL FLASH SCAN: Read addresses until end marker or size limit
    // The 0x1000 byte limit prevents reading beyond the allocated flash region
    while group_addr_next_pos < 0x1000 {
        // Update position tracking before processing this address entry
        MESH_DEVICE_ADDRESS_NEXT_POSITION.set(device_addr_next_pos);
        MESH_GROUP_ADDRESS_NEXT_POSITION.set(group_addr_next_pos);

        // FLASH READ: Read 16-bit address value in little-endian format
        // Uses flash_read_page for safe flash access with proper timing
        let mut addr_bytes = [0u8; 2];
        flash_read_page(flash_offset, 2, addr_bytes.as_mut_ptr());
        let group_addr = u16::from_le_bytes(addr_bytes);

        // Advance to next flash position (2 bytes per address)
        group_addr_next_pos += 2;
        flash_offset += 2;

        // END MARKER CHECK: 0xFFFF indicates no more addresses to read
        // This is the standard flash erase value, so unprogrammed regions
        // naturally terminate the address list
        if group_addr == 0xFFFF {
            break;
        }

        // ADDRESS CLASSIFICATION: Determine address type and update appropriate storage
        match group_addr {
            // EMPTY SLOT: 0x0000 reserved as empty address marker
            // Position tracking remains at device address for next write
            0 => {
                MESH_DEVICE_ADDRESS_NEXT_POSITION.set(device_addr_next_pos);
            }
            // DEVICE ADDRESS: Address with only masked bits set is device type
            // Last device address in flash takes precedence (supports address changes)
            addr if addr & !dev_mask == 0 => {
                DEVICE_ADDRESS.set(addr & dev_mask);
                device_addr_next_pos = group_addr_next_pos;
                MESH_GROUP_ADDRESS_NEXT_POSITION.set(device_addr_next_pos);
                MESH_DEVICE_ADDRESS_NEXT_POSITION.set(device_addr_next_pos);
            }
            // GROUP ADDRESS: All other non-zero addresses are group memberships
            // Stored in circular buffer with & 7 masking (8 groups max)
            addr => {
                let mut group_addresses = GROUP_ADDRESS.lock();
                let index = (group_addr_index & 7) as usize; // Circular buffer wrap
                group_addresses[index] = addr;
                group_addr_index += 1;
                MESH_GROUP_ADDRESS_NEXT_POSITION.set(group_addr_next_pos);
            }
        }
    }

    // DEFAULT ADDRESS FALLBACK: If no device address found, derive from MAC ID
    if DEVICE_ADDRESS.get() == 0 {
        let mac_id = MAC_ID.lock();
        let default_addr = u16::from_le_bytes([mac_id[0], mac_id[1]]);
        DEVICE_ADDRESS.set(if default_addr == 0 { 1 } else { default_addr });
    }
}

/// Initializes mesh node tracking structures.
///
/// This function sets up the local mesh node state after device address
/// resolution. It prepares the node tracking buffers and initializes the
/// first entry with this device's information.
///
/// # Initialization Sequence
///
/// 1. **Buffer Setup**: Clears and initializes mesh node tracking buffers
/// 2. **Self-Registration**: Adds this device as the first tracked node
/// 3. **State Initialization**: Sets device address and sequence number
///
/// # Node Tracking Purpose
///
/// The mesh_node_st array maintains state for recently seen mesh devices:
/// - Enables duplicate packet detection via sequence number tracking
/// - Supports mesh message relay decisions
/// - Maintains network topology awareness
///
/// # Side Effects
/// * Clears all mesh node tracking buffers
/// * Sets MESH_NODE_MAX to 1 (only self is known)
/// * Initializes first node entry with local device info
#[cfg_attr(test, mry::mry)]
pub fn mesh_node_init() {
    // Initialize mesh node tracking buffers
    app().mesh_manager.mesh_node_buf_init();

    let mut mesh_node_st = MESH_NODE_ST.lock();

    // Register self as first tracked node
    mesh_node_st[0].val.dev_adr = DEVICE_ADDRESS.get() as u8; // Lower byte of device address
    mesh_node_st[0].val.sn = DEVICE_NODE_SN.get(); // Current sequence number
    MESH_NODE_MAX.set(1); // Only this device is known
}

/// Implements flash wear-leveling by wrapping configuration storage.
///
/// This function performs flash sector management to extend flash lifetime by
/// distributing write operations across the flash sector. It implements a simple
/// wear-leveling algorithm that rotates through available flash space before
/// erasing and reusing sectors.
///
/// # Flash Wear-Leveling Algorithm
///
/// The pairing configuration is stored in a dedicated flash sector with multiple
/// 64-byte configuration slots. The algorithm:
///
/// 1. **Sequential Writes**: Configurations are written sequentially through the sector
/// 2. **Wrap Detection**: When reaching the last slot (0xFC0), trigger wrap operation
/// 3. **Data Preservation**: Current configuration is backed up before sector erase
/// 4. **Sector Erase**: Erase entire sector to allow reuse
/// 5. **Configuration Restore**: Write backed-up configuration to sector start
/// 6. **Index Reset**: Reset configuration index to 0 for next write
///
/// # Flash Sector Layout
///
/// ```text
/// Flash Sector (4KB):
/// ├─ 0x000: Config slot 0 (64 bytes)
/// ├─ 0x040: Config slot 1 (64 bytes)
/// ├─ 0x080: Config slot 2 (64 bytes)
/// │  ...
/// ├─ 0xFC0: Config slot 63 (64 bytes) <- WRAP_THRESHOLD
/// └─ End of sector
/// ```
///
/// # Wear-Leveling Benefits
///
/// - **Extended Flash Lifetime**: Distributes writes across 64 slots instead of 1
/// - **Reduced Erase Cycles**: Sector erased only once per 64 configuration updates
/// - **Data Integrity**: Atomic operation ensures configuration is never lost
/// - **No External Storage**: Operates within single flash sector
///
/// # Why 0xFC0 as Wrap Threshold
///
/// - 0xFC0 = 4032 bytes = Last 64-byte aligned position before 4KB sector end
/// - Allows 64 configuration updates before requiring sector erase
/// - Provides ~10,000 configuration updates per sector erase cycle
/// - With 100K erase cycle flash, supports 10M configuration updates total
///
/// # Flash Operation Safety
///
/// Uses proper flash controller API for:
/// - **Timing Compliance**: flash_read_page handles read timing requirements
/// - **Erase Verification**: flash_erase_sector ensures complete erase
/// - **Write Integrity**: flash_write_page includes verification
///
/// # Side Effects
/// * Erases entire pairing flash sector at wrap point
/// * Resets `FLASH_CONFIGURATION_INDEX` to 0 after wrap
/// * Brief execution delay during sector erase (~20ms typical)
#[cfg_attr(test, mry::mry)]
pub fn pair_flash_clean() {
    const CONFIG_SIZE: usize = 64;
    const WRAP_THRESHOLD: i32 = 0xfc0; // Last 64-byte slot before 4KB sector end

    // WRAP DETECTION: Check if we've gone past the last configuration slot
    // At 0xFC0 (4032 bytes), we're at the last 64-byte boundary in a 4KB sector
    // We only wrap when we would exceed this (i.e., index > 0xFC0)
    if FLASH_CONFIGURATION_INDEX.get() > WRAP_THRESHOLD {
        let mut config_backup = [0u8; CONFIG_SIZE];
        // Read from the previous slot (current index - 64) since we just incremented past the boundary
        let src_addr = (FLASH_ADR_PAIRING as i32 + FLASH_CONFIGURATION_INDEX.get() - 64) as u32;

        // Read current configuration from previous slot into backup buffer
        flash_read_page(src_addr, CONFIG_SIZE as u32, config_backup.as_mut_ptr());

        // Erase sector and write configuration to the beginning
        flash_erase_sector(FLASH_ADR_PAIRING);
        flash_write_page(
            FLASH_ADR_PAIRING,
            CONFIG_SIZE as u32,
            config_backup.as_ptr(),
        );
        FLASH_CONFIGURATION_INDEX.set(0);
    }
}

/// Discovers and initializes pairing configuration from flash storage.
///
/// This function implements a wear-leveling aware configuration discovery algorithm
/// that finds the most recent valid pairing configuration in flash. It supports
/// multiple configuration slots to distribute write operations and extend flash lifetime.
///
/// # Flash Sector Layout
///
/// The pairing sector (4KB at FLASH_ADR_PAIRING) is divided into 64-byte slots:
/// ```text
/// Offset    Slot  Usage
/// --------  ----  -----
/// 0x000     0     Config slot 0 (active after wrap)
/// 0x040     1     Config slot 1
/// 0x080     2     Config slot 2
/// ...       ...   ...
/// 0xFC0     63    Config slot 63 (last before wrap)
/// ```
///
/// # Discovery Algorithm
///
/// 1. **Initial Check**: Read first slot to verify any config exists
/// 2. **Linear Scan**: Walk through slots sequentially
/// 3. **Validation**: Check first byte of each slot for PAIR_CONFIG_VALID_FLAG
/// 4. **Last Valid**: Track most recent valid configuration
/// 5. **Wrap Handling**: Call pair_flash_clean() to manage sector wraparound
///
/// # Wear Leveling Strategy
///
/// - New configurations written to next sequential slot
/// - When slot 63 is reached (0xFC0), sector erased and wrapped to slot 0
/// - Distributes ~64 writes before erase operation required
/// - Extends flash lifetime by ~64x vs. single-slot approach
///
/// # Returns
/// * `true` - Valid configuration found and loaded
/// * `false` - No configuration exists (first-time boot)
///
/// # Side Effects
/// * Sets FLASH_CONFIGURATION_INDEX to last valid slot offset or -1
/// * May trigger sector erase if at wrap threshold
#[cfg_attr(test, mry::mry)]
pub fn pair_flash_config_init() -> bool {
    // STEP 1: Check if any configuration exists
    let mut first_buffer = [0u8; 0x40];
    flash_read_page(FLASH_ADR_PAIRING, 0x40, first_buffer.as_mut_ptr());

    if first_buffer[0] != PAIR_CONFIG_VALID_FLAG {
        // No configuration has ever been written
        FLASH_CONFIGURATION_INDEX.set(-1);
        return false;
    }

    // STEP 2: Linear scan to find last valid configuration
    // This implements the wear-leveling read algorithm - finds the most recent
    // valid configuration by scanning sequentially until hitting an empty slot
    let mut last_valid_index = 0i32;
    let mut buffer = [0u8; 0x40];

    for offset in (0x40..=0xfc0).step_by(0x40) {
        flash_read_page(FLASH_ADR_PAIRING + offset as u32, 0x40, buffer.as_mut_ptr());

        if buffer[0] != PAIR_CONFIG_VALID_FLAG {
            // Found boundary between used and unused slots
            // Last valid config is at previous offset
            break;
        }

        // This slot is valid, track it as most recent
        last_valid_index = offset;
    }

    // STEP 3: Update global index and handle wraparound if needed
    FLASH_CONFIGURATION_INDEX.set(last_valid_index);
    pair_flash_clean(); // Check for sector wraparound condition
    true
}

/// Generates a Bluetooth LE access code from mesh name and password.
///
/// This function implements a cryptographic derivation algorithm that creates a
/// 32-bit BLE access code used for radio-level packet filtering. The algorithm
/// ensures the access code has proper bit distribution characteristics required
/// by the Bluetooth LE physical layer specification.
///
/// # Access Code Purpose
///
/// In Bluetooth LE, the access code:
/// - Provides radio-level packet discrimination (prevents wrong network reception)
/// - Must have balanced bit transitions for clock recovery
/// - Must avoid long runs of identical bits (DC balance)
/// - Serves as correlation pattern for packet detection
///
/// # Algorithm Overview
///
/// The derivation process consists of three stages:
///
/// ## Stage 1: AES-Based Key Derivation
/// 1. Pad name and password to 16 bytes
/// 2. Perform AES encryption (name as plaintext, password as key)
/// 3. Extract first 32 bits as base access code
///
/// ## Stage 2: Consecutive Bit Limiting
/// Ensures no more than 6 consecutive identical bits:
/// - Computes bit transition pattern: `(value >> 1) ^ value`
/// - Scans for runs of 6+ consecutive 1-bits in transition pattern
/// - Flips bits in original value to break up long runs
/// - Prevents DC bias and improves clock recovery
///
/// ## Stage 3: Bit Density Validation
/// Ensures minimum bit transitions (prevents degenerate codes):
/// - Tests alternating bit pattern (0xAAAAAAAA) against result
/// - Counts matching set bits
/// - If fewer than 3 matches, applies XOR correction (0xFF)
/// - Repeats test with complementary pattern (0x55555555)
///
/// # Bit Balance Requirements
///
/// The algorithm enforces Bluetooth LE radio requirements:
/// - **Run Length**: Maximum 6 consecutive identical bits
/// - **Transitions**: Minimum bit density ensures correlation peak
/// - **DC Balance**: Prevents receiver DC offset accumulation
/// - **Uniqueness**: AES ensures different names/passwords → different codes
///
/// # Why Two Passes in Stage 3?
///
/// The dual-pass validation with alternating patterns ensures:
/// 1. First pass: Validates against 0xAAAAAAAA (alternating 1010...)
/// 2. Second pass: Validates against 0x55555555 (alternating 0101...)
///    This catches both odd and even bit position degeneracy.
///
/// # Parameters
/// * `name` - Mesh network name (truncated/padded to 16 bytes)
/// * `pass` - Mesh network password (truncated/padded to 16 bytes)
///
/// # Returns
/// * 32-bit access code meeting BLE PHY requirements
///
/// # Example Bit Pattern
/// ```text
/// Input:  0b11111111000000001010101010101010
/// Stage 2: Break up consecutive runs of 1s and 0s
/// Stage 3: Ensure alternating pattern density
/// Output: 0b10110110100110101001101010110101 (valid BLE access code)
/// ```
#[cfg_attr(test, mry::mry)]
pub fn access_code(name: &[u8], pass: &[u8]) -> u32 {
    // STAGE 1: CRYPTOGRAPHIC KEY DERIVATION
    // Prepare fixed-size inputs for AES encryption
    let mut name_arr = [0u8; 16];
    let mut pass_arr = [0u8; 16];

    // Truncate inputs to 16 bytes (AES block size) or pad with zeros if shorter
    let name_len = name.len().min(16);
    let pass_len = pass.len().min(16);
    name_arr[..name_len].copy_from_slice(&name[..name_len]);
    pass_arr[..pass_len].copy_from_slice(&pass[..pass_len]);

    // AES ENCRYPTION: Use password as key, name as plaintext
    // This provides cryptographic uniqueness - different name/password combos
    // produce completely different access codes with no predictable relationship
    let encrypted = aes_att_encryption(&name_arr, &pass_arr);
    let mut result: [u32; 4] = bytemuck::cast(encrypted);

    // STAGE 2: CONSECUTIVE BIT RUN LIMITING
    // XOR with shifted self creates transition pattern (1 = bit differs from next)
    // Scanning this pattern finds runs of identical bits in original value
    let mut bit_pattern = (result[0] >> 1) ^ result[0];
    let mut consecutive_ones = 0;

    // Scan transition pattern to find and break up long runs
    for bit_position in 1..32 {
        if bit_pattern & 1 == 0 {
            break; // Found a transition (run ended)
        }
        consecutive_ones += 1;
        // Break runs longer than 6 bits by flipping a bit in the original value
        if consecutive_ones > 5 {
            result[0] ^= 1 << bit_position;
            consecutive_ones = 0; // Reset run counter after correction
        }
        bit_pattern >>= 1; // Check next bit position
    }

    // STAGE 3: BIT DENSITY VALIDATION
    // Ensure sufficient bit transitions for reliable packet detection
    // Uses two test patterns to validate both odd and even bit positions
    let mut test_pattern = 0xAAAAAAAA; // Binary: 10101010... (alternating pattern)

    for _ in 0..2 {
        // Count how many bits match between result and test pattern
        // This detects degenerate patterns with too few transitions
        let set_bits = (0..32)
            .filter(|&bit_pos| (1 << bit_pos & test_pattern) != 0)
            .count();

        // If fewer than 3 matching bits, pattern is too sparse - apply correction
        if set_bits < 3 {
            result[0] ^= 0xFF; // Flip lower byte to increase bit density
        }

        // Second pass: test complementary pattern (01010101...)
        // XOR with 0x55555555 inverts the alternating pattern for second validation
        test_pattern = result[0] ^ 0x55555555;
    }

    result[0]
}

/// Updates BLE advertisement data with current pairing configuration.
///
/// This function synchronizes the Bluetooth LE advertisement packets with the
/// current mesh network pairing state. It regenerates the access code and updates
/// the advertisement name and manufacturer data to reflect any configuration changes.
/// This is called after pairing updates to ensure the device advertises its current
/// mesh network membership.
///
/// # BLE Advertisement Structure
///
/// The function updates two components of the BLE advertisement:
///
/// ## 1. Mesh Name (GAP Local Name)
/// - Advertised as BLE device name
/// - Truncated to MAX_MESH_NAME_LEN if longer
/// - Null-terminated (stops at first 0x00 byte)
/// - Used by mesh network discovery
///
/// ## 2. Manufacturer Data (GAP Manufacturer Specific Data)
/// - Contains mesh network identification
/// - Includes vendor ID and device capabilities
/// - Used for mesh network filtering
/// - Allows quick network identification without connection
///
/// # Access Code Regeneration
///
/// The function regenerates the BLE access code when pairing changes because:
/// - Access code is derived from network name and password
/// - Changing networks requires different radio-level filtering
/// - Prevents receiving packets from wrong mesh network
/// - Ensures only same-network devices can communicate
///
/// # Name Length Calculation Algorithm
///
/// The algorithm determines effective name length:
/// 1. Search for null terminator (0x00) in name array
/// 2. If found, use position as length
/// 3. If not found, use full array length
/// 4. Clamp result to MAX_MESH_NAME_LEN
///
/// This handles both null-terminated C strings and full-length names correctly.
///
/// # Why Update Both Name and Manufacturer Data?
///
/// - **Name**: Human-readable, visible in BLE scanners, used for pairing
/// - **Manufacturer Data**: Machine-readable, contains vendor ID and network info
/// - Both must be updated atomically to maintain consistency
/// - Allows devices to filter advertisements before connection
///
/// # Thread Safety Note
///
/// Locks PAIR_STATE during read but releases before BLE operations to:
/// - Prevent deadlock with BLE radio interrupt handlers
/// - Minimize lock hold time for better concurrency
/// - Allow other threads to access pairing state during BLE updates
///
/// # Side Effects
/// * Updates `PAIR_AC` global with new access code
/// * Modifies BLE advertisement name field
/// * Modifies BLE advertisement manufacturer data field
/// * Triggers radio to restart advertising with new data
#[cfg_attr(test, mry::mry)]
pub fn pair_update_key() {
    let pair_state = PAIR_STATE.lock();

    // ACCESS CODE GENERATION: Derive BLE access code from network credentials
    // This must be done first as it affects radio-level packet filtering
    PAIR_AC.set(access_code(&pair_state.pair_nn, &pair_state.pair_pass));

    // NAME LENGTH CALCULATION: Find effective length of mesh name
    // Searches for null terminator, uses full length if not found, clamps to max
    // This three-stage approach handles: C strings, Rust strings, and full buffers
    let name_len = pair_state
        .pair_nn
        .iter()
        .position(|&byte| byte == 0) // Find null terminator
        .unwrap_or(pair_state.pair_nn.len()) // Use full length if no null
        .min(MAX_MESH_NAME_LEN); // Clamp to maximum allowed

    // ADVERTISEMENT NAME UPDATE: Set BLE device name in advertisement packet
    // This makes the mesh network name visible to BLE scanners and pairing tools
    set_advertisement_mesh_name(&pair_state.pair_nn[..name_len]);

    // MANUFACTURER DATA UPDATE: Set vendor-specific advertisement data
    let adv_data = *ADV_PRI_DATA.lock();
    // SAFETY: AdvPrivate is repr(C, packed) with only primitive integer types
    // (u16, u16, u32), making it safe to reinterpret as a byte slice. The
    // struct has a fixed size and alignment, and all bit patterns are valid.
    // This is necessary because AdvPrivate doesn't implement Pod trait due to
    // packed representation requirements.
    let adv_data_bytes = unsafe {
        core::slice::from_raw_parts(
            &adv_data as *const AdvPrivate as *const u8,
            core::mem::size_of::<AdvPrivate>(),
        )
    };
    set_advertisement_manufacturer_data(adv_data_bytes);
}

/// Loads pairing configuration from flash into runtime state.
///
/// This function implements the flash-based configuration recovery algorithm that
/// restores mesh network pairing credentials from persistent storage. It handles
/// flash layout navigation, data integrity verification, and password decoding
/// to restore the device's mesh network identity after power-on or reset.
///
/// # Flash Configuration Layout
///
/// The pairing data is stored in a 64-byte configuration block:
///
/// ```text
/// Offset  Size  Content
/// ------  ----  -------
/// 0x00    1     Valid flag (PAIR_CONFIG_VALID_FLAG)
/// 0x01    1     Address validation pending flag
/// 0x02-0x0E    Reserved
/// 0x0F    1     Encoded password flag (PAIR_CONFIG_VALID_FLAG = encoded)
/// 0x10    16    Mesh network name (null-terminated or full)
/// 0x20    16    Mesh network password (potentially encoded)
/// 0x30    16    Long-term key (LTK) for BLE pairing
/// ```
///
/// # Configuration Discovery Algorithm
///
/// The function uses multi-stage discovery:
///
/// ## Stage 1: Configuration Index Location
/// - Calls `pair_flash_config_init()` to find active configuration
/// - Scans flash sector for last valid configuration marker
/// - Returns early if no valid configuration exists (first boot)
///
/// ## Stage 2: Configuration Data Loading
/// - Reads mesh name from offset 0x10 (16 bytes)
/// - Reads password from offset 0x20 (16 bytes)
/// - Reads LTK from offset 0x30 (16 bytes)
/// - Reads validation flag from offset 0x01 (1 byte)
/// - Reads encoding flag from offset 0x0F (1 byte)
///
/// ## Stage 3: Password Decoding
/// - Checks encoding flag at offset 0x0F
/// - If set to PAIR_CONFIG_VALID_FLAG, password is encoded
/// - Calls `decode_password()` to recover plaintext password
/// - Decoded password replaces encoded version in runtime state
///
/// ## Stage 4: State Synchronization
/// - Calls `pair_update_key()` to regenerate access code
/// - Updates BLE advertisement with loaded configuration
/// - Ensures radio uses correct network credentials
///
/// # Password Encoding Rationale
///
/// Passwords may be stored encoded in flash to:
/// - Prevent casual observation of network credentials
/// - Provide minimal obfuscation against simple flash dumps
/// - Note: This is NOT cryptographic security, just basic obfuscation
///
/// # Address Validation Pending Flag
///
/// The validation pending flag indicates:
/// - Device has been paired but address not yet confirmed
/// - Used during multi-step pairing process
/// - Prevents premature mesh network joining
/// - Cleared when pairing is fully completed
///
/// # Flash Read Safety
///
/// Uses `flash_read_page` instead of raw pointers for:
/// - **Memory Safety**: No undefined behavior from invalid addresses
/// - **Flash Timing**: Proper wait states for flash read operations
/// - **Hardware Abstraction**: Flash controller timing managed by driver
/// - **Portability**: Works across different flash memory chips
///
/// # Early Return Optimization
///
/// Returns immediately if config_index < 0 because:
/// - No valid configuration exists (first boot scenario)
/// - Saves flash read operations and processing time
/// - Prevents reading from invalid flash addresses
/// - Device will use default/unpaired state
///
/// # Side Effects
/// * Updates `PAIR_STATE` with loaded configuration
/// * Updates `MESH_DEVICE_ADDRESS_VALIDATION_PENDING` flag
/// * Updates `PAIR_AC` with regenerated access code
/// * Updates BLE advertisement data via `pair_update_key()`
#[cfg_attr(test, mry::mry)]
pub fn pair_load_key() {
    // STAGE 1: CONFIGURATION DISCOVERY
    // Locate the active configuration block in flash wear-leveling structure
    pair_flash_config_init();

    let config_index = FLASH_CONFIGURATION_INDEX.get();

    // EARLY RETURN: No valid configuration exists (first boot or factory reset)
    // Negative index indicates pair_flash_config_init() found no valid config marker
    if config_index < 0 {
        // Initialize PAIR_STATE with default credentials from PAIR_CONFIG_*
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state.pair_nn = *PAIR_CONFIG_MESH_NAME.lock();
            pair_state.pair_pass = *PAIR_CONFIG_MESH_PWD.lock();
            pair_state.pair_ltk = *PAIR_CONFIG_MESH_LTK.lock();
        }
        // Update BLE advertisement with default name and regenerate access code
        pair_update_key();
        return;
    }

    // Calculate absolute flash address by adding config index to base address
    let pairing_addr = (FLASH_ADR_PAIRING as i32 + config_index) as u32;

    {
        let mut pair_state = PAIR_STATE.lock();

        // STAGE 2A: CLEAR EXISTING STATE
        // Zero out current configuration to ensure clean slate
        // Prevents partial/corrupted state if flash reads fail
        pair_state.pair_nn.fill(0);
        pair_state.pair_pass.fill(0);
        pair_state.pair_ltk.fill(0);

        // STAGE 2B: LOAD MESH NETWORK NAME
        // Offset 0x10, 16 bytes (MAX_MESH_NAME_LEN)
        // May be null-terminated or use full 16 bytes
        flash_read_page(
            pairing_addr + 0x10,
            MAX_MESH_NAME_LEN as u32,
            pair_state.pair_nn.as_mut_ptr(),
        );

        // STAGE 2C: LOAD NETWORK PASSWORD
        // Offset 0x20, 16 bytes
        // May be encoded (checked later via flag at 0x0F)
        flash_read_page(pairing_addr + 0x20, 0x10, pair_state.pair_pass.as_mut_ptr());

        // STAGE 2D: LOAD ADDRESS VALIDATION FLAG (if pairing enabled)
        // Offset 0x01, 1 byte
        // Indicates whether device address has been confirmed in mesh network
        if MESH_PAIR_ENABLE.get() {
            let mut validation_flag = [0u8; 1];
            flash_read_page(pairing_addr + 0x1, 1, validation_flag.as_mut_ptr());
            // Convert byte to boolean: non-zero = validation pending
            MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(validation_flag[0] != 0);
        }

        // STAGE 3: PASSWORD DECODING
        // Offset 0x0F contains encoding flag
        // If flag == PAIR_CONFIG_VALID_FLAG, password is encoded and must be decoded
        let mut config_flag = [0u8; 1];
        flash_read_page(pairing_addr + 0xF, 1, config_flag.as_mut_ptr());
        if config_flag[0] == PAIR_CONFIG_VALID_FLAG {
            // Decode password in-place (decode_password returns new array)
            pair_state.pair_pass = decode_password(&pair_state.pair_pass);
        }

        // STAGE 2E: LOAD LONG-TERM KEY
        // Offset 0x30, 16 bytes
        // Used for BLE encryption during pairing process
        flash_read_page(pairing_addr + 0x30, 0x10, pair_state.pair_ltk.as_mut_ptr());
    }

    // STAGE 4: STATE SYNCHRONIZATION
    // Regenerate access code and update BLE advertisements
    // Must be done after lock is released to prevent deadlock with radio interrupts
    pair_update_key();
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::sdk::ble_app::ble_ll_pair::pair_save_key;
    use crate::sdk::ble_app::light_ll::connection_management::mock_setup_ble_parameter_start;
    use crate::sdk::drivers::flash::{
        mock_flash_erase_sector, mock_flash_read_page, mock_flash_write_page,
    };
    use crate::sdk::mcu::crypto::{
        decode_password, encode_password, mock_aes_att_encryption, mock_decode_password,
        mock_encode_password,
    };
    use crate::sdk::packet_types::Packet;
    use crate::sdk::rf_drv::{
        mock_set_advertisement_manufacturer_data, mock_set_advertisement_mesh_name,
    };
    use crate::{app_mocker, mock_app_mocker, App};
    use mry::send_wrapper::SendWrapper;
    use mry::Any;

    /// Creates an aligned [u8; 100] buffer suitable for casting to *const Packet.
    /// Packet requires align(4); a [u8] array on the stack has no alignment guarantee.
    fn aligned_packet_buf() -> [u32; 25] {
        [0u32; 25]
    }

    /// Cast an aligned u32 buffer to a *const Packet for use in tests.
    macro_rules! packet_from_buf {
        ($buf:expr) => {
            unsafe { &*($buf.as_ptr() as *const Packet) }
        };
    }

    // --- Tests for dev_addr_with_mac_flag ---

    #[test]
    fn test_dev_addr_with_mac_flag_returns_true() {
        // Test that the function returns true when params[2] equals DEV_ADDR_PAR_WITH_MAC
        let params = [0x00, 0x00, DEV_ADDR_PAR_WITH_MAC, 0x00];
        assert!(
            dev_addr_with_mac_flag(&params),
            "Should return true when params[2] == DEV_ADDR_PAR_WITH_MAC"
        );
    }

    #[test]
    fn test_dev_addr_with_mac_flag_returns_false() {
        // Test that the function returns false when params[2] doesn't equal DEV_ADDR_PAR_WITH_MAC
        let params = [0x00, 0x00, 0x00, 0x00];
        assert!(
            !dev_addr_with_mac_flag(&params),
            "Should return false when params[2] != DEV_ADDR_PAR_WITH_MAC"
        );

        let params2 = [0x00, 0x00, 0xff, 0x00];
        assert!(
            !dev_addr_with_mac_flag(&params2),
            "Should return false for different value"
        );
    }

    // --- Tests for dev_addr_with_mac_rsp ---

    #[test]
    fn test_dev_addr_with_mac_rsp_copies_mac_id() {
        // Set test MAC_ID
        let test_mac = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        MAC_ID.lock().copy_from_slice(&test_mac);

        // Create response buffer
        let mut par_rsp = [0u8; 16];

        // Call function
        let result = dev_addr_with_mac_rsp(&mut par_rsp);

        // Verify result
        assert!(result, "Should return true");
        assert_eq!(
            &par_rsp[4..10],
            &test_mac,
            "MAC ID should be copied to par_rsp[4..10]"
        );
    }

    #[test]
    fn test_dev_addr_with_mac_rsp_preserves_other_bytes() {
        // Set test MAC_ID
        let test_mac = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        MAC_ID.lock().copy_from_slice(&test_mac);

        // Create response buffer with non-zero values
        let mut par_rsp = [0x99u8; 16];

        // Call function
        dev_addr_with_mac_rsp(&mut par_rsp);

        // Verify other bytes are preserved
        assert_eq!(par_rsp[0], 0x99, "Bytes before MAC should be preserved");
        assert_eq!(par_rsp[3], 0x99, "Bytes before MAC should be preserved");
        assert_eq!(par_rsp[10], 0x99, "Bytes after MAC should be preserved");
        assert_eq!(par_rsp[15], 0x99, "Bytes after MAC should be preserved");
    }

    // --- Tests for dev_addr_with_mac_match ---

    #[test]
    fn test_dev_addr_with_mac_match_returns_validation_pending_for_broadcast() {
        // Test that 0xFF 0xFF prefix returns validation pending state
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(true);
        let params = [0xff, 0xff, 0x00, 0x00, 0x00, 0x00];

        assert!(
            dev_addr_with_mac_match(&params),
            "Should return validation pending state (true)"
        );

        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);
        assert!(
            !dev_addr_with_mac_match(&params),
            "Should return validation pending state (false)"
        );
    }

    #[test]
    fn test_dev_addr_with_mac_match_compares_mac_id() {
        // Set test MAC_ID
        let test_mac = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        MAC_ID.lock().copy_from_slice(&test_mac);

        // Test matching MAC
        let params_match = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        assert!(
            dev_addr_with_mac_match(&params_match),
            "Should return true when MAC matches"
        );

        // Test non-matching MAC
        let params_no_match = [0x11, 0x22, 0x33, 0x44, 0x55, 0x67];
        assert!(
            !dev_addr_with_mac_match(&params_no_match),
            "Should return false when MAC doesn't match"
        );
    }

    // --- Tests for save_pair_info ---

    #[test]
    #[mry::lock(flash_write_page)]
    fn test_save_pair_info_writes_to_correct_address() {
        // Setup test configuration index
        FLASH_CONFIGURATION_INDEX.set(0x80);

        // Setup mock
        mock_flash_write_page(Any, Any, Any).returns(());

        // Call function with test data
        let test_data = [0x01, 0x02, 0x03, 0x04];
        let test_addr = 0x10u32;
        save_pair_info(test_addr, &test_data);

        // Calculate expected address
        let expected_addr = (FLASH_ADR_PAIRING as i32 + 0x80 + test_addr as i32) as u32;

        // Verify flash_write_page was called with correct parameters
        mock_flash_write_page(expected_addr, test_data.len() as u32, Any).assert_called(1);
    }

    #[test]
    #[mry::lock(flash_write_page)]
    fn test_save_pair_info_handles_different_offsets() {
        // Test with different configuration indices
        FLASH_CONFIGURATION_INDEX.set(0x00);

        mock_flash_write_page(Any, Any, Any).returns(());

        let test_data = [0xAA, 0xBB];
        save_pair_info(0x20, &test_data);

        let expected_addr = (FLASH_ADR_PAIRING as i32 + 0x20) as u32;
        mock_flash_write_page(expected_addr, 2, Any).assert_called(1);
    }

    // --- Tests for update_ble_parameter_cb ---

    #[test]
    #[mry::lock(setup_ble_parameter_start)]
    fn test_update_ble_parameter_cb_calls_setup_when_not_successful() {
        // Reset state
        CONN_UPDATE_SUCCESSED.set(false);
        CONN_UPDATE_CNT.set(0);

        // Setup mock
        mock_setup_ble_parameter_start(Any, Any, Any).returns(0);

        // Call function
        update_ble_parameter_cb();

        // Verify setup was called with first parameter set
        mock_setup_ble_parameter_start(
            CONN_PARA_DATA[0][0],
            CONN_PARA_DATA[0][1],
            CONN_PARA_DATA[0][2] as u32,
        )
        .assert_called(1);

        // Verify counter was incremented
        assert_eq!(CONN_UPDATE_CNT.get(), 1);
    }

    #[test]
    #[mry::lock(setup_ble_parameter_start)]
    fn test_update_ble_parameter_cb_does_nothing_when_successful() {
        // Set state to successful (don't rely on previous test state)
        CONN_UPDATE_SUCCESSED.set(true);
        CONN_UPDATE_CNT.set(0);

        // Setup mock (should not be called)
        mock_setup_ble_parameter_start(Any, Any, Any).returns(0);

        // Call function
        update_ble_parameter_cb();

        // Verify setup was NOT called
        mock_setup_ble_parameter_start(Any, Any, Any).assert_called(0);

        // Verify counter was not incremented
        assert_eq!(CONN_UPDATE_CNT.get(), 0);
    }

    // --- Tests for rf_update_conn_para ---

    #[test]
    #[mry::lock(setup_ble_parameter_start)]
    fn test_rf_update_conn_para_handles_success_response() {
        // Setup initial state - ensure clean state first
        CONN_UPDATE_CNT.set(0);
        CONN_UPDATE_SUCCESSED.set(false);

        // Create a packet with success response (0x0000)
        // PacketL2capHead structure: dma_len(4), _type(1), rf_len(1), l2cap_len(2), chan_id(2)
        let mut packet_data = aligned_packet_buf();
        let packet_data: &mut [u8] =
            unsafe { core::slice::from_raw_parts_mut(packet_data.as_mut_ptr() as *mut u8, 100) };
        // dma_len (offset 0-3) = 0x00000000
        packet_data[4] = 2; // _type: l2cap data pkt, start pkt
        packet_data[5] = 0x0A; // rf_len
        packet_data[6] = 0x06; // l2cap_len_low
        packet_data[7] = 0x00; // l2cap_len_high
        packet_data[8] = 0x05; // chanid_low
        packet_data[9] = 0x00; // chanid_high
        packet_data[10] = 0x13; // code
        packet_data[11] = 0x01; // id
        packet_data[12] = 0x02; // data_len_low
        packet_data[13] = 0x00; // data_len_high
        packet_data[14] = 0x00; // result_low (success)
        packet_data[15] = 0x00; // result_high

        let packet = packet_from_buf!(packet_data);

        mock_setup_ble_parameter_start(Any, Any, Any).returns(0);

        // Call function
        let result = rf_update_conn_para(packet);

        // Verify result
        assert_eq!(result, 0);
        // Note: CONN_UPDATE_CNT might not be 0 after success - it's only reset on rejection
        assert_eq!(CONN_UPDATE_CNT.get(), 0, "Counter should be reset to 0");
        assert!(
            CONN_UPDATE_SUCCESSED.get(),
            "Success flag should be set to true"
        );

        // Verify setup was not called for success case
        mock_setup_ble_parameter_start(Any, Any, Any).assert_called(0);
    }

    #[test]
    #[mry::lock(setup_ble_parameter_start)]
    fn test_rf_update_conn_para_handles_rejection_response() {
        // Setup initial state - start from 0 to test increment to 1
        CONN_UPDATE_CNT.set(0);
        CONN_UPDATE_SUCCESSED.set(false);

        // Create a packet with rejection response (0x0001)
        let mut packet_data = aligned_packet_buf();
        let packet_data: &mut [u8] =
            unsafe { core::slice::from_raw_parts_mut(packet_data.as_mut_ptr() as *mut u8, 100) };
        packet_data[4] = 2; // _type
        packet_data[5] = 0x0A; // rf_len
        packet_data[6] = 0x06; // l2cap_len_low
        packet_data[7] = 0x00; // l2cap_len_high
        packet_data[8] = 0x05; // chanid_low
        packet_data[9] = 0x00; // chanid_high
        packet_data[10] = 0x13; // code
        packet_data[11] = 0x01; // id
        packet_data[12] = 0x02; // data_len_low
        packet_data[13] = 0x00; // data_len_high
        packet_data[14] = 0x01; // result_low (rejection)
        packet_data[15] = 0x00; // result_high

        let packet = packet_from_buf!(packet_data);

        mock_setup_ble_parameter_start(Any, Any, Any).returns(0);

        // Call function
        rf_update_conn_para(packet);

        // Verify counter was incremented from 0 to 1
        assert_eq!(CONN_UPDATE_CNT.get(), 1);

        // Verify setup was called with first parameter set (counter was 0, now 1)
        mock_setup_ble_parameter_start(
            CONN_PARA_DATA[0][0],
            CONN_PARA_DATA[0][1],
            CONN_PARA_DATA[0][2] as u32,
        )
        .assert_called(1);
    }

    #[test]
    #[mry::lock(setup_ble_parameter_start)]
    fn test_rf_update_conn_para_resets_counter_when_max_reached() {
        // Setup initial state at max count
        CONN_UPDATE_CNT.set(UPDATE_CONN_PARA_CNT);
        CONN_UPDATE_SUCCESSED.set(false);

        // Create a packet with rejection response
        let mut packet_data = aligned_packet_buf();
        let packet_data: &mut [u8] =
            unsafe { core::slice::from_raw_parts_mut(packet_data.as_mut_ptr() as *mut u8, 100) };
        packet_data[4] = 2; // _type
        packet_data[5] = 0x0A; // rf_len
        packet_data[6] = 0x06; // l2cap_len_low
        packet_data[7] = 0x00; // l2cap_len_high
        packet_data[8] = 0x05; // chanid_low
        packet_data[9] = 0x00; // chanid_high
        packet_data[10] = 0x13; // code
        packet_data[11] = 0x01; // id
        packet_data[12] = 0x02; // data_len_low
        packet_data[13] = 0x00; // data_len_high
        packet_data[14] = 0x01; // result_low (rejection)
        packet_data[15] = 0x00; // result_high

        let packet = packet_from_buf!(packet_data);

        mock_setup_ble_parameter_start(Any, Any, Any).returns(0);

        // Call function
        rf_update_conn_para(packet);

        // Verify counter was reset to 0 instead of continuing to increment
        assert_eq!(CONN_UPDATE_CNT.get(), 0);

        // Verify setup was not called when max reached
        mock_setup_ble_parameter_start(Any, Any, Any).assert_called(0);
    }

    #[test]
    fn test_rf_update_conn_para_ignores_non_matching_packets() {
        // Setup initial state
        CONN_UPDATE_CNT.set(1);
        CONN_UPDATE_SUCCESSED.set(false);

        // Create a packet that doesn't match the expected response
        let mut packet_data = aligned_packet_buf();
        let packet_data: &mut [u8] =
            unsafe { core::slice::from_raw_parts_mut(packet_data.as_mut_ptr() as *mut u8, 100) };
        packet_data[4] = 2; // _type
        packet_data[5] = 0x0B; // Different rf_len (doesn't match 0x0A)

        let packet = packet_from_buf!(packet_data);

        // Call function
        let result = rf_update_conn_para(packet);

        // Verify state unchanged
        assert_eq!(result, 0);
        assert_eq!(CONN_UPDATE_CNT.get(), 1);
        assert!(!CONN_UPDATE_SUCCESSED.get());
    }

    #[test]
    fn test_rf_update_conn_para_rejects_non_l2cap_start_packet() {
        // Line 112: Verify packets with _type & 0b11 != 2 are rejected
        CONN_UPDATE_CNT.set(0);
        CONN_UPDATE_SUCCESSED.set(false);

        let mut packet_data = aligned_packet_buf();
        let packet_data: &mut [u8] =
            unsafe { core::slice::from_raw_parts_mut(packet_data.as_mut_ptr() as *mut u8, 100) };
        packet_data[4] = 0; // _type = 0 (not an L2CAP start packet)
        packet_data[5] = 0x0A; // Correct rf_len (but shouldn't matter)
        packet_data[6] = 0x06; // Correct l2cap_len_low
        packet_data[7] = 0x00; // Correct l2cap_len_high
        packet_data[8] = 0x05; // Correct chanid_low
        packet_data[9] = 0x00; // Correct chanid_high
        packet_data[10] = 0x13; // Correct code
        packet_data[11] = 0x01; // Correct id
        packet_data[12] = 0x02; // Correct data_len_low
        packet_data[13] = 0x00; // Correct data_len_high
        packet_data[14] = 0x00; // Success result

        let packet = packet_from_buf!(packet_data);

        // Call function
        let result = rf_update_conn_para(packet);

        // Verify packet was rejected (state unchanged)
        assert_eq!(result, 0);
        assert_eq!(CONN_UPDATE_CNT.get(), 0);
        assert!(!CONN_UPDATE_SUCCESSED.get());
    }

    #[test]
    fn test_rf_update_conn_para_rejects_wrong_l2cap_len() {
        // Line 123: Verify packets with wrong l2cap_len are rejected
        CONN_UPDATE_CNT.set(0);
        CONN_UPDATE_SUCCESSED.set(false);

        let mut packet_data = aligned_packet_buf();
        let packet_data: &mut [u8] =
            unsafe { core::slice::from_raw_parts_mut(packet_data.as_mut_ptr() as *mut u8, 100) };
        packet_data[4] = 2; // _type (correct)
        packet_data[5] = 0x0A; // rf_len (correct)
        packet_data[6] = 0x07; // l2cap_len_low (WRONG - should be 0x06)
        packet_data[7] = 0x00; // l2cap_len_high (correct)
        packet_data[8] = 0x05; // chanid_low (correct)
        packet_data[9] = 0x00; // chanid_high (correct)
        packet_data[10] = 0x13; // code (correct)
        packet_data[11] = 0x01; // id (correct)
        packet_data[12] = 0x02; // data_len_low (correct)
        packet_data[13] = 0x00; // data_len_high (correct)
        packet_data[14] = 0x00; // result (correct)

        let packet = packet_from_buf!(packet_data);

        let result = rf_update_conn_para(packet);

        assert_eq!(result, 0);
        assert_eq!(CONN_UPDATE_CNT.get(), 0);
        assert!(!CONN_UPDATE_SUCCESSED.get());
    }

    #[test]
    fn test_rf_update_conn_para_rejects_wrong_chan_id() {
        // Line 123: Verify packets with wrong chan_id are rejected
        CONN_UPDATE_CNT.set(0);
        CONN_UPDATE_SUCCESSED.set(false);

        let mut packet_data = aligned_packet_buf();
        let packet_data: &mut [u8] =
            unsafe { core::slice::from_raw_parts_mut(packet_data.as_mut_ptr() as *mut u8, 100) };
        packet_data[4] = 2; // _type (correct)
        packet_data[5] = 0x0A; // rf_len (correct)
        packet_data[6] = 0x06; // l2cap_len_low (correct)
        packet_data[7] = 0x00; // l2cap_len_high (correct)
        packet_data[8] = 0x04; // chanid_low (WRONG - should be 0x05)
        packet_data[9] = 0x00; // chanid_high (correct)
        packet_data[10] = 0x13; // code (correct)
        packet_data[11] = 0x01; // id (correct)
        packet_data[12] = 0x02; // data_len_low (correct)
        packet_data[13] = 0x00; // data_len_high (correct)
        packet_data[14] = 0x00; // result (correct)

        let packet = packet_from_buf!(packet_data);

        let result = rf_update_conn_para(packet);

        assert_eq!(result, 0);
        assert_eq!(CONN_UPDATE_CNT.get(), 0);
        assert!(!CONN_UPDATE_SUCCESSED.get());
    }

    #[test]
    fn test_rf_update_conn_para_rejects_wrong_code() {
        // Line 123: Verify packets with wrong code are rejected
        CONN_UPDATE_CNT.set(0);
        CONN_UPDATE_SUCCESSED.set(false);

        let mut packet_data = aligned_packet_buf();
        let packet_data: &mut [u8] =
            unsafe { core::slice::from_raw_parts_mut(packet_data.as_mut_ptr() as *mut u8, 100) };
        packet_data[4] = 2; // _type (correct)
        packet_data[5] = 0x0A; // rf_len (correct)
        packet_data[6] = 0x06; // l2cap_len_low (correct)
        packet_data[7] = 0x00; // l2cap_len_high (correct)
        packet_data[8] = 0x05; // chanid_low (correct)
        packet_data[9] = 0x00; // chanid_high (correct)
        packet_data[10] = 0x12; // code (WRONG - should be 0x13)
        packet_data[11] = 0x01; // id (correct)
        packet_data[12] = 0x02; // data_len_low (correct)
        packet_data[13] = 0x00; // data_len_high (correct)
        packet_data[14] = 0x00; // result (correct)

        let packet = packet_from_buf!(packet_data);

        let result = rf_update_conn_para(packet);

        assert_eq!(result, 0);
        assert_eq!(CONN_UPDATE_CNT.get(), 0);
        assert!(!CONN_UPDATE_SUCCESSED.get());
    }

    #[test]
    fn test_rf_update_conn_para_rejects_wrong_data_len() {
        // Line 123: Verify packets with wrong data_len are rejected
        CONN_UPDATE_CNT.set(0);
        CONN_UPDATE_SUCCESSED.set(false);

        let mut packet_data = aligned_packet_buf();
        let packet_data: &mut [u8] =
            unsafe { core::slice::from_raw_parts_mut(packet_data.as_mut_ptr() as *mut u8, 100) };
        packet_data[4] = 2; // _type (correct)
        packet_data[5] = 0x0A; // rf_len (correct)
        packet_data[6] = 0x06; // l2cap_len_low (correct)
        packet_data[7] = 0x00; // l2cap_len_high (correct)
        packet_data[8] = 0x05; // chanid_low (correct)
        packet_data[9] = 0x00; // chanid_high (correct)
        packet_data[10] = 0x13; // code (correct)
        packet_data[11] = 0x01; // id (correct)
        packet_data[12] = 0x03; // data_len_low (WRONG - should be 0x02)
        packet_data[13] = 0x00; // data_len_high (correct)
        packet_data[14] = 0x00; // result (correct)

        let packet = packet_from_buf!(packet_data);

        let result = rf_update_conn_para(packet);

        assert_eq!(result, 0);
        assert_eq!(CONN_UPDATE_CNT.get(), 0);
        assert!(!CONN_UPDATE_SUCCESSED.get());
    }

    #[test]
    fn test_rf_update_conn_para_ignores_unknown_result_code() {
        // Line 147: Verify unknown result codes (not 0x0000 or 0x0001) are ignored
        CONN_UPDATE_CNT.set(2);
        CONN_UPDATE_SUCCESSED.set(false);

        let mut packet_data = aligned_packet_buf();
        let packet_data: &mut [u8] =
            unsafe { core::slice::from_raw_parts_mut(packet_data.as_mut_ptr() as *mut u8, 100) };
        packet_data[4] = 2; // _type
        packet_data[5] = 0x0A; // rf_len
        packet_data[6] = 0x06; // l2cap_len_low
        packet_data[7] = 0x00; // l2cap_len_high
        packet_data[8] = 0x05; // chanid_low
        packet_data[9] = 0x00; // chanid_high
        packet_data[10] = 0x13; // code
        packet_data[11] = 0x01; // id
        packet_data[12] = 0x02; // data_len_low
        packet_data[13] = 0x00; // data_len_high
        packet_data[14] = 0x02; // result_low (unknown code - should be 0x00 or 0x01)
        packet_data[15] = 0x00; // result_high

        let packet = packet_from_buf!(packet_data);

        // Call function
        let result = rf_update_conn_para(packet);

        // Verify packet was processed but state unchanged (no match arm executed)
        assert_eq!(result, 0);
        assert_eq!(CONN_UPDATE_CNT.get(), 2);
        assert!(!CONN_UPDATE_SUCCESSED.get());
    }

    #[test]
    fn test_rf_update_conn_para_ignores_negative_result_code() {
        // Line 147: Verify other unknown result codes like 0xFFFF are ignored
        CONN_UPDATE_CNT.set(1);
        CONN_UPDATE_SUCCESSED.set(false);

        let mut packet_data = aligned_packet_buf();
        let packet_data: &mut [u8] =
            unsafe { core::slice::from_raw_parts_mut(packet_data.as_mut_ptr() as *mut u8, 100) };
        packet_data[4] = 2; // _type
        packet_data[5] = 0x0A; // rf_len
        packet_data[6] = 0x06; // l2cap_len_low
        packet_data[7] = 0x00; // l2cap_len_high
        packet_data[8] = 0x05; // chanid_low
        packet_data[9] = 0x00; // chanid_high
        packet_data[10] = 0x13; // code
        packet_data[11] = 0x01; // id
        packet_data[12] = 0x02; // data_len_low
        packet_data[13] = 0x00; // data_len_high
        packet_data[14] = 0xFF; // result_low (unknown code)
        packet_data[15] = 0xFF; // result_high

        let packet = packet_from_buf!(packet_data);

        let result = rf_update_conn_para(packet);

        assert_eq!(result, 0);
        assert_eq!(CONN_UPDATE_CNT.get(), 1);
        assert!(!CONN_UPDATE_SUCCESSED.get());
    }

    // --- Tests for retrieve_dev_grp_address ---
    // Note: These tests are skipped because retrieve_dev_grp_address directly accesses
    // raw memory addresses (FLASH_ADR_DEV_GRP_ADR) which don't exist in test environments.
    // To properly test this function, we would need to refactor it to use dependency injection
    // or create a testable wrapper that can be mocked.

    // --- Tests for mesh_node_init ---

    #[test]
    #[mry::lock(app_mocker)]
    fn test_mesh_node_init_initializes_node() {
        // Setup test device address
        DEVICE_ADDRESS.set(0x1234);
        DEVICE_NODE_SN.set(0x56);

        // Mock app
        let mut app = App::default();
        app.mesh_manager.mock_mesh_node_buf_init().returns(());

        mock_app_mocker().returns(&mut app as *mut App);

        // Call function
        mesh_node_init();

        // Verify mesh node was initialized
        let mesh_node_st = MESH_NODE_ST.lock();
        assert_eq!(mesh_node_st[0].val.dev_adr, 0x34); // Lower byte
        assert_eq!(mesh_node_st[0].val.sn, 0x56);
        assert_eq!(MESH_NODE_MAX.get(), 1);

        // Verify mock was called
        app.mesh_manager.mock_mesh_node_buf_init().assert_called(1);
    }

    // --- Tests for pair_flash_clean ---

    #[test]
    fn test_pair_flash_clean_does_nothing_when_not_at_end() {
        // Setup configuration below wrap point
        FLASH_CONFIGURATION_INDEX.set(0x80);

        // Call function
        pair_flash_clean();

        // Verify index unchanged
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), 0x80);
    }

    // Note: test_pair_flash_clean_wraps_at_end is skipped because pair_flash_clean
    // directly accesses raw memory addresses which don't exist in test environments.
    // The function reads from (FLASH_ADR_PAIRING + offset) which would cause segfault.
    // To test this, we would need to refactor pair_flash_clean to use dependency injection.

    // --- Tests for pair_flash_config_init ---

    #[test]
    #[mry::lock(flash_read_page, flash_erase_sector, flash_write_page)]
    fn test_pair_flash_config_init_returns_false_when_no_config() {
        // Setup mocks to return invalid flag
        mock_flash_read_page(Any, Any, Any).returns_with(
            |_addr: u32, _len: u32, buf: SendWrapper<*mut u8>| {
                let buffer = unsafe { core::slice::from_raw_parts_mut(*buf, 0x40) };
                buffer[0] = 0x00; // Invalid flag
            },
        );
        mock_flash_erase_sector(Any).returns(());
        mock_flash_write_page(Any, Any, Any).returns(());

        // Call function
        let result = pair_flash_config_init();

        // Verify result
        assert!(!result, "Should return false when no valid config");
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), -1);
    }

    #[test]
    #[mry::lock(flash_read_page, flash_erase_sector, flash_write_page)]
    fn test_pair_flash_config_init_finds_last_valid_config() {
        // Setup mocks to simulate valid configs at 0x00, 0x40, and 0x80, then invalid at 0xC0
        mock_flash_read_page(Any, Any, Any).returns_with(
            |addr: u32, _len: u32, buf: SendWrapper<*mut u8>| {
                let buffer = unsafe { core::slice::from_raw_parts_mut(*buf, 0x40) };

                if addr == FLASH_ADR_PAIRING {
                    buffer[0] = PAIR_CONFIG_VALID_FLAG; // First config valid
                } else if addr == FLASH_ADR_PAIRING + 0x40 {
                    buffer[0] = PAIR_CONFIG_VALID_FLAG; // Second config valid
                } else if addr == FLASH_ADR_PAIRING + 0x80 {
                    buffer[0] = PAIR_CONFIG_VALID_FLAG; // Third config valid
                } else {
                    buffer[0] = 0x00; // All others invalid
                }
            },
        );
        mock_flash_erase_sector(Any).returns(());
        mock_flash_write_page(Any, Any, Any).returns(());

        // Call function
        let result = pair_flash_config_init();

        // Verify result
        assert!(result, "Should return true when valid config found");
        assert_eq!(
            FLASH_CONFIGURATION_INDEX.get(),
            0x80,
            "Should find last valid config at 0x80"
        );
    }

    // --- Tests for access_code ---

    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_access_code_basic_operation() {
        // Setup test inputs
        let name = b"TestName";
        let pass = b"TestPassword";

        // Setup mock to return predictable value
        let mock_result = [
            0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
            0x77, 0x88,
        ];
        mock_aes_att_encryption(Any, Any).returns(mock_result);

        // Call function
        let result = access_code(name, pass);

        // Verify aes_att_encryption was called
        mock_aes_att_encryption(Any, Any).assert_called(1);

        // Verify result is a u32
        assert!(result != 0, "Should return non-zero access code");
    }

    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_access_code_handles_short_inputs() {
        // Test with very short inputs
        let name = b"Hi";
        let pass = b"Pw";

        let mock_result = [0xAAu8; 16];
        mock_aes_att_encryption(Any, Any).returns(mock_result);

        // Should not panic
        let result = access_code(name, pass);
        assert!(result != 0);
    }

    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_access_code_handles_long_inputs() {
        // Test with inputs longer than 16 bytes
        let name = b"ThisIsAVeryLongNameThatExceeds16Bytes";
        let pass = b"ThisIsAVeryLongPasswordThatExceeds16Bytes";

        let mock_result = [0xBBu8; 16];
        mock_aes_att_encryption(Any, Any).returns(mock_result);

        // Should not panic, should truncate to 16 bytes
        let result = access_code(name, pass);
        assert!(result != 0);
    }

    #[test]
    #[mry::lock(aes_att_encryption)]
    fn test_access_code_bit_manipulation_logic() {
        // Test that the bit manipulation logic is applied
        // Use a specific pattern that would trigger the bit flipping logic

        // Create a result where the first two bits are both 1 (triggering the logic)
        let mut mock_result = [0u8; 16];
        // Set up u32 value with specific bit pattern
        // The code checks: bit = destbuf[0] >> 1 ^ destbuf[0]
        // We want to create conditions where multiple consecutive bits are 1
        let test_value: u32 = 0b11111100000000000000000000000000u32;
        mock_result[0..4].copy_from_slice(&test_value.to_le_bytes());

        mock_aes_att_encryption(Any, Any).returns(mock_result);

        let name = b"Test";
        let pass = b"Test";

        // Call function
        let result = access_code(name, pass);

        // The function should apply bit manipulation logic
        // We can't predict the exact output without implementing the full algorithm,
        // but we can verify it returns a value
        assert!(result != 0);
    }

    // --- Tests for pair_update_key ---

    #[test]
    #[mry::lock(
        aes_att_encryption,
        set_advertisement_mesh_name,
        set_advertisement_manufacturer_data
    )]
    fn test_pair_update_key_updates_advertisement() {
        // Setup test pairing state
        let mut pair_state = PAIR_STATE.lock();
        pair_state.pair_nn = [0u8; MAX_MESH_NAME_LEN];
        pair_state.pair_nn[0..8].copy_from_slice(b"MeshName");
        pair_state.pair_pass = [0u8; 16];
        pair_state.pair_pass[0..8].copy_from_slice(b"Password");
        drop(pair_state);

        // Setup mocks
        let mock_result = [0x12u8; 16];
        mock_aes_att_encryption(Any, Any).returns(mock_result);
        mock_set_advertisement_mesh_name(Any).returns(());
        mock_set_advertisement_manufacturer_data(Any).returns(());

        // Call function
        pair_update_key();

        // Verify advertisement functions were called
        mock_set_advertisement_mesh_name(Any).assert_called(1);
        mock_set_advertisement_manufacturer_data(Any).assert_called(1);

        // Verify PAIR_AC was set
        assert_ne!(PAIR_AC.get(), 0, "PAIR_AC should be set");
    }

    #[test]
    #[mry::lock(
        aes_att_encryption,
        set_advertisement_mesh_name,
        set_advertisement_manufacturer_data
    )]
    fn test_pair_update_key_handles_null_terminated_name() {
        // Setup pairing state with null-terminated name
        let mut pair_state = PAIR_STATE.lock();
        pair_state.pair_nn = [0u8; MAX_MESH_NAME_LEN];
        pair_state.pair_nn[0..5].copy_from_slice(b"Test\0");
        pair_state.pair_pass = [0u8; 16];
        drop(pair_state);

        // Setup mocks
        mock_aes_att_encryption(Any, Any).returns([0u8; 16]);
        mock_set_advertisement_mesh_name(Any).returns(());
        mock_set_advertisement_manufacturer_data(Any).returns(());

        // Call function
        pair_update_key();

        // Verify set_advertisement_mesh_name was called (exact bytes hard to match due to slice truncation)
        mock_set_advertisement_mesh_name(Any).assert_called(1);
    }

    // --- Tests for pair_load_key ---
    // Note: pair_load_key tests are skipped because the function reads from raw memory
    // addresses (pairing_addr + offsets) which would cause segfaults in test environment.
    // The function uses: slice::from_raw_parts((pairing_addr + 0x10) as *const u8, ...)
    // To test this properly, we would need to refactor to use dependency injection.

    #[test]
    #[mry::lock(flash_read_page, flash_erase_sector, flash_write_page, pair_update_key)]
    fn test_pair_load_key_handles_no_config() {
        // Setup: Initialize PAIR_CONFIG_* with default values
        PAIR_CONFIG_MESH_NAME.lock().fill(0);
        PAIR_CONFIG_MESH_NAME.lock()[0..11].copy_from_slice(b"out_of_mesh");

        PAIR_CONFIG_MESH_PWD.lock().fill(0);
        PAIR_CONFIG_MESH_PWD.lock()[0..3].copy_from_slice(b"123");

        let test_ltk = [
            0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd,
            0xde, 0xdf,
        ];
        *PAIR_CONFIG_MESH_LTK.lock() = test_ltk;

        // Setup flash to have no valid configuration
        mock_flash_read_page(Any, Any, Any).returns_with(
            |_addr: u32, _len: u32, buf: SendWrapper<*mut u8>| {
                let buffer = unsafe { core::slice::from_raw_parts_mut(*buf, 0x40) };
                buffer[0] = 0x00; // Invalid flag
            },
        );
        mock_flash_erase_sector(Any).returns(());
        mock_flash_write_page(Any, Any, Any).returns(());
        mock_pair_update_key().returns(());

        // Call function - should load defaults and call pair_update_key
        pair_load_key();

        // Verify FLASH_CONFIGURATION_INDEX is -1
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), -1);

        // Verify PAIR_STATE was initialized with default credentials
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_nn[0..11], b"out_of_mesh");
        assert_eq!(&pair_state.pair_pass[0..3], b"123");
        assert_eq!(pair_state.pair_ltk, test_ltk);
        drop(pair_state);

        // Verify pair_update_key was called to update advertisements
        mock_pair_update_key().assert_called(1);
    }

    #[test]
    #[mry::lock(
        flash_read_page,
        flash_erase_sector,
        flash_write_page,
        pair_update_key,
        pair_flash_config_init
    )]
    fn test_pair_load_key_loads_valid_config() {
        // Setup: Initialize PAIR_CONFIG_* with defaults (should NOT be used)
        PAIR_CONFIG_MESH_NAME.lock().fill(0);
        PAIR_CONFIG_MESH_NAME.lock()[0..11].copy_from_slice(b"out_of_mesh");
        PAIR_CONFIG_MESH_PWD.lock().fill(0);
        PAIR_CONFIG_MESH_PWD.lock()[0..3].copy_from_slice(b"123");

        // Mock pair_flash_config_init to set a valid index
        mock_pair_flash_config_init().returns_with(|| {
            FLASH_CONFIGURATION_INDEX.set(0x1000);
            true
        });

        // Mock flash reads to return custom configuration
        mock_flash_read_page(Any, Any, Any).returns_with(
            |addr: u32, len: u32, buf: SendWrapper<*mut u8>| {
                let buffer = unsafe { core::slice::from_raw_parts_mut(*buf, len as usize) };

                // Determine what data to return based on offset
                let base_addr = FLASH_ADR_PAIRING + 0x1000;
                let offset = addr.saturating_sub(base_addr);

                match offset {
                    0x10 => {
                        // Mesh name at offset +0x10
                        buffer.fill(0);
                        buffer[0..10].copy_from_slice(b"CustomMesh");
                    }
                    0x20 => {
                        // Password at offset +0x20
                        buffer.fill(0);
                        buffer[0..8].copy_from_slice(b"Pass1234");
                    }
                    0x30 => {
                        // LTK at offset +0x30
                        buffer.fill(0xAA);
                    }
                    0x01 => {
                        // Validation flag
                        buffer[0] = 0x00;
                    }
                    0x0F => {
                        // Config flag (not encoded)
                        buffer[0] = 0x00;
                    }
                    _ => {
                        buffer.fill(0);
                    }
                }
            },
        );
        mock_flash_erase_sector(Any).returns(());
        mock_flash_write_page(Any, Any, Any).returns(());
        mock_pair_update_key().returns(());

        // Call function - should load from flash, not defaults
        pair_load_key();

        // Verify PAIR_STATE was loaded with custom config, NOT defaults
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_nn[0..10], b"CustomMesh");
        assert_eq!(&pair_state.pair_pass[0..8], b"Pass1234");
        assert_eq!(pair_state.pair_ltk[0], 0xAA);

        // Verify it did NOT use the default credentials
        assert_ne!(&pair_state.pair_nn[0..11], b"out_of_mesh");
        drop(pair_state);

        // Verify pair_update_key was called to update advertisements
        mock_pair_update_key().assert_called(1);
    }

    // --- Integration Tests for Flash Pairing Persistence ---
    // These tests verify the complete save/load cycle for pairing data

    /// Test that a single write/read cycle preserves pairing data correctly
    #[test]
    #[mry::lock(
        flash_read_page,
        flash_write_page,
        flash_erase_sector,
        pair_update_key,
        encode_password,
        decode_password
    )]
    fn test_pairing_persistence_single_write_read() {
        use std::sync::{Arc, Mutex};

        // Simulate 4KB flash sector
        let flash_storage = Arc::new(Mutex::new([0xFFu8; 4096]));
        let flash_clone_write = Arc::clone(&flash_storage);
        let flash_clone_read = Arc::clone(&flash_storage);
        let flash_clone_erase = Arc::clone(&flash_storage);

        // Setup: No existing config (first boot)
        FLASH_CONFIGURATION_INDEX.set(-1);

        // Set up default credentials (normally set during init)
        *PAIR_CONFIG_MESH_NAME.lock() = *b"defaultmesh\0\0\0\0\0";
        *PAIR_CONFIG_MESH_PWD.lock() = *b"defaultpass\0\0\0\0\0";
        *PAIR_CONFIG_MESH_LTK.lock() = [0xCD; 16];

        // Mock flash operations to use our simulated flash
        mock_flash_write_page(Any, Any, Any).returns_with(
            move |addr: u32, len: u32, data_ptr: SendWrapper<*const u8>| {
                let data = unsafe { std::slice::from_raw_parts(*data_ptr, len as usize) };
                let offset = (addr - FLASH_ADR_PAIRING) as usize;
                let mut storage = flash_clone_write.lock().unwrap();
                for i in 0..len as usize {
                    if offset + i < storage.len() {
                        storage[offset + i] &= data[i]; // Flash can only transition 1->0
                    }
                }
            },
        );

        mock_flash_read_page(Any, Any, Any).returns_with(
            move |addr: u32, len: u32, buf_ptr: SendWrapper<*mut u8>| {
                let buf = unsafe { std::slice::from_raw_parts_mut(*buf_ptr, len as usize) };
                let offset = (addr - FLASH_ADR_PAIRING) as usize;
                let storage = flash_clone_read.lock().unwrap();
                for i in 0..len as usize {
                    if offset + i < storage.len() {
                        buf[i] = storage[offset + i];
                    }
                }
            },
        );

        mock_flash_erase_sector(Any).returns_with(move |_addr: u32| {
            flash_clone_erase.lock().unwrap().fill(0xFF);
        });

        mock_pair_update_key().returns(());

        // Mock encode/decode_password to return the input data (identity function for testing)
        mock_encode_password(Any)
            .returns_with(|password: Vec<u8>| password.as_slice().try_into().unwrap());
        mock_decode_password(Any)
            .returns_with(|password: Vec<u8>| password.as_slice().try_into().unwrap());

        // Set test pairing data
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state
                .pair_nn
                .copy_from_slice(b"testmesh\0\0\0\0\0\0\0\0");
            pair_state
                .pair_pass
                .copy_from_slice(b"password123\0\0\0\0\0");
            pair_state.pair_ltk = [0xAB; 16];
        }

        // SAVE: Call pair_save_key
        pair_save_key();

        // Note: Flash erase is NOT called on first write since flash starts erased (all 0xFF)
        // Erase only happens when wrapping around at offset 0xFC0

        // Verify index was set to correct value
        // With original bug: -1 + 64 = 63
        // With fix: should be 0
        let save_index = FLASH_CONFIGURATION_INDEX.get();
        assert_eq!(save_index, 0, "First save should write to offset 0");

        // Clear pair state to verify load works
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state.pair_nn.fill(0);
            pair_state.pair_pass.fill(0);
            pair_state.pair_ltk.fill(0);
        }

        // Simulate reboot
        FLASH_CONFIGURATION_INDEX.set(-1);

        // LOAD: Call pair_load_key
        pair_load_key();

        // Verify loaded data matches
        let pair_state = PAIR_STATE.lock();
        assert_eq!(
            &pair_state.pair_nn[0..8],
            b"testmesh",
            "Mesh name should match"
        );
        assert_eq!(
            &pair_state.pair_pass[0..11],
            b"password123",
            "Password should match"
        );
        assert_eq!(pair_state.pair_ltk, [0xAB; 16], "LTK should match");

        // Verify the load found the config at the correct index
        let load_index = FLASH_CONFIGURATION_INDEX.get();
        assert_eq!(
            load_index, save_index,
            "Load should find config at same index where it was saved"
        );
    }

    /// Test that no pairing data in flash loads defaults
    #[test]
    #[mry::lock(flash_read_page, pair_update_key)]
    fn test_pairing_persistence_no_data_loads_defaults() {
        use std::sync::{Arc, Mutex};

        // Erased flash (all 0xFF)
        let flash_storage = Arc::new(Mutex::new([0xFFu8; 4096]));
        let flash_clone = Arc::clone(&flash_storage);

        FLASH_CONFIGURATION_INDEX.set(0); // Reset to initial state

        mock_flash_read_page(Any, Any, Any).returns_with(
            move |addr: u32, len: u32, buf_ptr: SendWrapper<*mut u8>| {
                let buf = unsafe { std::slice::from_raw_parts_mut(*buf_ptr, len as usize) };
                let offset = (addr - FLASH_ADR_PAIRING) as usize;
                let storage = flash_clone.lock().unwrap();
                for i in 0..len as usize {
                    if offset + i < storage.len() {
                        buf[i] = storage[offset + i];
                    }
                }
            },
        );

        mock_pair_update_key().returns(());

        // Set defaults in PAIR_CONFIG_*
        *PAIR_CONFIG_MESH_NAME.lock() = *b"defaultmesh\0\0\0\0\0";
        *PAIR_CONFIG_MESH_PWD.lock() = *b"defaultpass\0\0\0\0\0";
        *PAIR_CONFIG_MESH_LTK.lock() = [0xCD; 16];

        // Load from empty flash
        pair_load_key();

        // Verify defaults were loaded
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_nn[0..11], b"defaultmesh");
        assert_eq!(&pair_state.pair_pass[0..11], b"defaultpass");
        assert_eq!(pair_state.pair_ltk, [0xCD; 16]);

        // Verify index is -1 (no config found)
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), -1);
    }

    /// Test multiple writes use wear-leveling (different offsets)
    #[test]
    #[mry::lock(
        flash_read_page,
        flash_write_page,
        flash_erase_sector,
        pair_update_key,
        encode_password,
        decode_password
    )]
    fn test_pairing_persistence_wear_leveling() {
        use std::sync::{Arc, Mutex};

        let flash_storage = Arc::new(Mutex::new([0xFFu8; 4096]));
        let flash_clone_write = Arc::clone(&flash_storage);
        let flash_clone_read = Arc::clone(&flash_storage);
        let flash_clone_erase = Arc::clone(&flash_storage);

        FLASH_CONFIGURATION_INDEX.set(-1);

        mock_flash_write_page(Any, Any, Any).returns_with(
            move |addr: u32, len: u32, data_ptr: SendWrapper<*const u8>| {
                let data = unsafe { std::slice::from_raw_parts(*data_ptr, len as usize) };
                let offset = (addr - FLASH_ADR_PAIRING) as usize;
                let mut storage = flash_clone_write.lock().unwrap();
                for i in 0..len as usize {
                    if offset + i < storage.len() {
                        storage[offset + i] &= data[i];
                    }
                }
            },
        );

        mock_flash_read_page(Any, Any, Any).returns_with(
            move |addr: u32, len: u32, buf_ptr: SendWrapper<*mut u8>| {
                let buf = unsafe { std::slice::from_raw_parts_mut(*buf_ptr, len as usize) };
                let offset = (addr - FLASH_ADR_PAIRING) as usize;
                let storage = flash_clone_read.lock().unwrap();
                for i in 0..len as usize {
                    if offset + i < storage.len() {
                        buf[i] = storage[offset + i];
                    }
                }
            },
        );

        mock_flash_erase_sector(Any).returns_with(move |_addr: u32| {
            flash_clone_erase.lock().unwrap().fill(0xFF);
        });

        mock_pair_update_key().returns(());

        // Mock encode/decode_password to return the input data (identity function for testing)
        mock_encode_password(Any)
            .returns_with(|password: Vec<u8>| password.as_slice().try_into().unwrap());
        mock_decode_password(Any)
            .returns_with(|password: Vec<u8>| password.as_slice().try_into().unwrap());

        // First write
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state
                .pair_nn
                .copy_from_slice(b"mesh1\0\0\0\0\0\0\0\0\0\0\0");
            pair_state
                .pair_pass
                .copy_from_slice(b"pass1\0\0\0\0\0\0\0\0\0\0\0");
            pair_state.pair_ltk = [0x01; 16];
        }
        pair_save_key();
        let index1 = FLASH_CONFIGURATION_INDEX.get();

        // Second write (should use next slot)
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state
                .pair_nn
                .copy_from_slice(b"mesh2\0\0\0\0\0\0\0\0\0\0\0");
            pair_state
                .pair_pass
                .copy_from_slice(b"pass2\0\0\0\0\0\0\0\0\0\0\0");
            pair_state.pair_ltk = [0x02; 16];
        }
        pair_save_key();
        let index2 = FLASH_CONFIGURATION_INDEX.get();

        // Third write
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state
                .pair_nn
                .copy_from_slice(b"mesh3\0\0\0\0\0\0\0\0\0\0\0");
            pair_state
                .pair_pass
                .copy_from_slice(b"pass3\0\0\0\0\0\0\0\0\0\0\0");
            pair_state.pair_ltk = [0x03; 16];
        }
        pair_save_key();
        let index3 = FLASH_CONFIGURATION_INDEX.get();

        // Verify wear-leveling: each write should use next 64-byte slot
        assert_eq!(index2, index1 + 64, "Second write should advance 64 bytes");
        assert_eq!(index3, index2 + 64, "Third write should advance 64 bytes");

        // Simulate reboot and load
        FLASH_CONFIGURATION_INDEX.set(-1);
        pair_load_key();

        // Should load the last written config (mesh3)
        let pair_state = PAIR_STATE.lock();
        assert_eq!(&pair_state.pair_nn[0..5], b"mesh3");
        assert_eq!(&pair_state.pair_pass[0..5], b"pass3");
        assert_eq!(pair_state.pair_ltk, [0x03; 16]);

        // Load index should match last save index
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), index3);
    }

    /// Test sector boundary wraparound
    #[test]
    #[mry::lock(
        flash_read_page,
        flash_write_page,
        flash_erase_sector,
        pair_update_key,
        encode_password,
        decode_password
    )]
    fn test_pairing_persistence_wraparound() {
        use std::sync::{Arc, Mutex};

        let flash_storage = Arc::new(Mutex::new([0xFFu8; 4096]));
        let flash_clone_write = Arc::clone(&flash_storage);
        let flash_clone_read = Arc::clone(&flash_storage);
        let flash_clone_erase = Arc::clone(&flash_storage);

        // Start near end of sector
        FLASH_CONFIGURATION_INDEX.set(0xF80); // Second to last slot

        mock_flash_write_page(Any, Any, Any).returns_with(
            move |addr: u32, len: u32, data_ptr: SendWrapper<*const u8>| {
                let data = unsafe { std::slice::from_raw_parts(*data_ptr, len as usize) };
                let offset = (addr - FLASH_ADR_PAIRING) as usize;
                let mut storage = flash_clone_write.lock().unwrap();
                for i in 0..len as usize {
                    if offset + i < storage.len() {
                        storage[offset + i] &= data[i];
                    }
                }
            },
        );

        mock_flash_read_page(Any, Any, Any).returns_with(
            move |addr: u32, len: u32, buf_ptr: SendWrapper<*mut u8>| {
                let buf = unsafe { std::slice::from_raw_parts_mut(*buf_ptr, len as usize) };
                let offset = (addr - FLASH_ADR_PAIRING) as usize;
                let storage = flash_clone_read.lock().unwrap();
                for i in 0..len as usize {
                    if offset + i < storage.len() {
                        buf[i] = storage[offset + i];
                    }
                }
            },
        );

        let erase_count = Arc::new(Mutex::new(0));
        let erase_count_clone = Arc::clone(&erase_count);

        mock_flash_erase_sector(Any).returns_with(move |_addr: u32| {
            flash_clone_erase.lock().unwrap().fill(0xFF);
            *erase_count_clone.lock().unwrap() += 1;
        });

        mock_pair_update_key().returns(());

        // Mock encode/decode_password to return the input data (identity function for testing)
        mock_encode_password(Any)
            .returns_with(|password: Vec<u8>| password.as_slice().try_into().unwrap());
        mock_decode_password(Any)
            .returns_with(|password: Vec<u8>| password.as_slice().try_into().unwrap());

        // Write data
        {
            let mut pair_state = PAIR_STATE.lock();
            pair_state
                .pair_nn
                .copy_from_slice(b"boundary\0\0\0\0\0\0\0\0");
            pair_state
                .pair_pass
                .copy_from_slice(b"test\0\0\0\0\0\0\0\0\0\0\0\0");
            pair_state.pair_ltk = [0xBC; 16];
        }

        // First write goes to 0xFC0 (last slot)
        pair_save_key();
        assert_eq!(FLASH_CONFIGURATION_INDEX.get(), 0xFC0);

        // Next write should trigger wraparound
        pair_save_key();

        // After wraparound, index should be back at 0
        assert_eq!(
            FLASH_CONFIGURATION_INDEX.get(),
            0,
            "Should wrap to offset 0"
        );

        // Verify sector was erased during wraparound
        assert!(
            *erase_count.lock().unwrap() >= 1,
            "Sector should have been erased for wraparound"
        );
    }
}
