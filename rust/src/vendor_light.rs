use core::mem::size_of;
use core::ptr::addr_of;
use core::slice;

use crate::sdk::light::AdvPrivate;
use crate::sdk::rf_drv::set_advertisement_manufacturer_data;
use crate::state::{ADV_PRI_DATA, ADV_RSP_PRI_DATA, MAC_ID};

/// Configures and sets the advertisement data for the BLE mesh light device.
///
/// This function performs two primary operations:
/// 1. Sets up the primary advertisement data (ADV_PRI_DATA) with device identification
///    information including the MAC address, and sends this data to the RF driver.
/// 2. Configures the scan response advertisement data (ADV_RSP_PRI_DATA) with additional
///    device information.
///
/// # Implementation Details
/// - Uses the global MAC_ID to identify this device in the mesh network
/// - Configures the device as a CCT (Color-Correlated Temperature) light (product_uuid = 0x02)
/// - Prepares reservation data in a sequential pattern (0..16)
///
/// # Hardware Context
/// This function interacts with the TLSR8266's BLE transceiver through the SDK layer
/// to establish the device's presence in the mesh network.
pub fn vendor_set_adv_data() {
    // Step 1: Configure and send primary advertisement data
    // --------------------------------------------------

    // Get the MAC address for device identification
    let mac_address = u32::from_le_bytes([
        MAC_ID.lock()[0],
        MAC_ID.lock()[1],
        MAC_ID.lock()[2],
        MAC_ID.lock()[3],
    ]);

    // Lock and configure the primary advertisement data structure
    let mut adv_pri_data = ADV_PRI_DATA.lock();
    adv_pri_data.mac_address = mac_address;

    // Create a temporary copy of the data before serialization
    let tmp = *adv_pri_data;

    // Serialize the AdvPrivate struct to bytes and send to the RF driver
    // This makes the device visible/discoverable in the mesh network
    let serialized_data =
        unsafe { slice::from_raw_parts(addr_of!(tmp) as *const u8, size_of::<AdvPrivate>()) };
    set_advertisement_manufacturer_data(serialized_data);

    // Step 2: Configure scan response advertisement data
    // --------------------------------------------------

    // Lock and configure the scan response advertisement data structure
    let mut adv_rsp_pri_data = ADV_RSP_PRI_DATA.lock();

    // Set product type to CCT (Color-Correlated Temperature) light mode (0x02)
    // This defines the light's capabilities in the mesh network
    // Other potential values include:
    // - RGB light (0x01)
    // - RGBW light (0x03)
    // - etc.
    adv_rsp_pri_data.product_uuid = 0x02;

    // Set the MAC address in the scan response data to match primary advertisement
    // This provides consistent device identification across all BLE communications
    adv_rsp_pri_data.mac_address = mac_address;

    // Initialize reservation data array with sequential values (0..16)
    // This space is used for device-specific data in the BLE protocol
    // The sequential pattern helps with debugging and verification
    for (index, value) in adv_rsp_pri_data.rsv.iter_mut().enumerate() {
        *value = index as u8;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdk::light::AdvRspPrivate;
    use crate::sdk::rf_drv::mock_set_advertisement_manufacturer_data;
    use core::cell::RefCell;
    use mry::Any;

    // Thread-local storage for test data
    thread_local! {
        static ACTUAL_SERIALIZED: RefCell<Vec<u8>> = const { RefCell::new(Vec::new()) };
    }

    #[test]
    #[mry::lock(set_advertisement_manufacturer_data)]
    fn test_vendor_set_adv_data() {
        // Setup test MAC_ID
        let test_mac = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
        // Calculate expected MAC integer using the real function
        let expected_mac_int =
            u32::from_le_bytes([test_mac[0], test_mac[1], test_mac[2], test_mac[3]]);

        // Create test instances
        let test_adv_pri_data = AdvPrivate {
            manufacture_id: 0x1122,
            mesh_product_uuid: 0x3344,
            mac_address: 0,
        };

        let test_adv_rsp_pri_data = AdvRspPrivate {
            manufacture_id: 0x1122,
            mesh_product_uuid: 0x3344,
            mac_address: 0,
            product_uuid: 0,
            status: 0,
            device_address: 0,
            rsv: [0; 16],
        };

        // Set temporary values for our global mutexes
        MAC_ID.lock().copy_from_slice(&test_mac);
        *ADV_PRI_DATA.lock() = test_adv_pri_data;
        *ADV_RSP_PRI_DATA.lock() = test_adv_rsp_pri_data;

        // Mock rf_link_slave_set_adv_private_data to verify it's called with correct data
        mock_set_advertisement_manufacturer_data(Any).returns_with(|data: Vec<u8>| {
            // Verify data contains the expected structs
            assert_eq!(data.len(), size_of::<AdvPrivate>());
        });

        // Call function under test
        vendor_set_adv_data();

        // Verify the data was modified correctly
        let updated_adv_pri_data = ADV_PRI_DATA.lock();
        let updated_adv_rsp_pri_data = ADV_RSP_PRI_DATA.lock();

        // Copy values to temporaries to prevent unaligned reads
        let pri_mac_address = updated_adv_pri_data.mac_address;
        let rsp_mac_address = updated_adv_rsp_pri_data.mac_address;
        let product_uuid = updated_adv_rsp_pri_data.product_uuid;

        // Verify primary advertisement data
        assert_eq!(
            pri_mac_address, expected_mac_int,
            "Primary advertisement MAC address should match expected value"
        );

        // Verify scan response data
        assert_eq!(
            rsp_mac_address, expected_mac_int,
            "Response advertisement MAC address should match expected value"
        );
        assert_eq!(
            product_uuid, 0x02,
            "Product UUID should be 0x02 for CCT light mode"
        );

        // Verify rsv array is set to 0..16
        for i in 0..16 {
            assert_eq!(
                updated_adv_rsp_pri_data.rsv[i], i as u8,
                "rsv[{}] should be {}",
                i, i
            );
        }

        // Verify mock was called the expected number of times
        mock_set_advertisement_manufacturer_data(Any).assert_called(1);
    }

    #[test]
    #[mry::lock(set_advertisement_manufacturer_data)]
    fn test_vendor_set_adv_data_with_zero_mac() {
        // Setup mock MAC_ID with zeros
        let test_mac = [0x00, 0x00, 0x00, 0x00, 0x05, 0x06];
        // Calculate expected MAC integer using the real function
        let expected_mac_int =
            u32::from_le_bytes([test_mac[0], test_mac[1], test_mac[2], test_mac[3]]);

        // Create test instances with values to be overwritten
        let test_adv_pri_data = AdvPrivate {
            manufacture_id: 0x1122,
            mesh_product_uuid: 0x3344,
            mac_address: 0xFFFFFFFF, // Different from expected to verify change
        };

        let test_adv_rsp_pri_data = AdvRspPrivate {
            manufacture_id: 0x1122,
            mesh_product_uuid: 0x3344,
            mac_address: 0xFFFFFFFF, // Different from expected to verify change
            product_uuid: 0xFF,      // Different from expected (0x02)
            status: 0,
            device_address: 0,
            rsv: [0xFF; 16], // Fill with non-zero values to verify change
        };

        // Set temporary values for our global mutexes
        MAC_ID.lock().copy_from_slice(&test_mac);
        *ADV_PRI_DATA.lock() = test_adv_pri_data;
        *ADV_RSP_PRI_DATA.lock() = test_adv_rsp_pri_data;

        // Mock function calls
        mock_set_advertisement_manufacturer_data(Any).returns(());

        // Call function under test
        vendor_set_adv_data();

        // Verify the data was modified correctly
        let updated_adv_pri_data = ADV_PRI_DATA.lock();
        let updated_adv_rsp_pri_data = ADV_RSP_PRI_DATA.lock();

        // Copy values to temporaries to prevent unaligned reads
        let pri_mac_address = updated_adv_pri_data.mac_address;
        let rsp_mac_address = updated_adv_rsp_pri_data.mac_address;
        let product_uuid = updated_adv_rsp_pri_data.product_uuid;

        // Verify MAC address was updated correctly even with zeroes
        assert_eq!(pri_mac_address, expected_mac_int);
        assert_eq!(rsp_mac_address, expected_mac_int);

        // Verify product UUID was set
        assert_eq!(product_uuid, 0x02);

        // Verify rsv array was overwritten with correct sequence
        for i in 0..16 {
            // Copy value to prevent unaligned read
            let rsv_value = updated_adv_rsp_pri_data.rsv[i];
            assert_eq!(rsv_value, i as u8);
        }
    }

    #[test]
    #[mry::lock(set_advertisement_manufacturer_data)]
    fn test_adv_data_serialization() {
        // This test verifies that the data is properly serialized
        // when passed to set_advertisement_manufacturer_data

        let test_mac = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        // Calculate expected MAC integer using the real function
        let expected_mac_int =
            u32::from_le_bytes([test_mac[0], test_mac[1], test_mac[2], test_mac[3]]);

        // Create a specific AdvPrivate instance we can verify
        let test_adv_pri_data = AdvPrivate {
            manufacture_id: 0xABCD,
            mesh_product_uuid: 0xEF01,
            mac_address: 0, // Will be set by the function
        };

        let expected_serialized = {
            let mut expected = test_adv_pri_data;
            expected.mac_address = expected_mac_int;

            // Convert to bytes
            let expected_bytes = unsafe {
                slice::from_raw_parts(addr_of!(expected) as *const u8, size_of::<AdvPrivate>())
            };
            expected_bytes.to_vec()
        };

        // Setup test environment
        MAC_ID.lock().copy_from_slice(&test_mac);
        *ADV_PRI_DATA.lock() = test_adv_pri_data;
        *ADV_RSP_PRI_DATA.lock() = AdvRspPrivate::default();

        // Clear the thread-local storage before test
        ACTUAL_SERIALIZED.with(|cell| {
            cell.borrow_mut().clear();
        });

        // Capture the actual data sent to the rf_link function using thread-local storage
        mock_set_advertisement_manufacturer_data(Any).returns_with(|data: Vec<u8>| {
            ACTUAL_SERIALIZED.with(|cell| {
                let mut actual = cell.borrow_mut();
                actual.clear();
                actual.extend_from_slice(&data);
            });
        });

        // Call function under test
        vendor_set_adv_data();

        // Get the captured data from thread-local storage
        let actual_serialized = ACTUAL_SERIALIZED.with(|cell| cell.borrow().clone());

        // Verify the serialized data
        assert_eq!(actual_serialized.len(), expected_serialized.len());
        assert_eq!(
            actual_serialized, expected_serialized,
            "Serialized data doesn't match expected bytes"
        );
    }
}
