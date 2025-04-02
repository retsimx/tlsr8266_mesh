use crate::state::{*};

/// Calculates and populates the BLE channel table based on the provided channel map.
///
/// # Algorithm
/// This function implements the Bluetooth Low Energy adaptive frequency hopping (AFH) mechanism
/// by creating a lookup table of available channels. The function:
/// 1. Scans through all 40 BLE channels (0-39)
/// 2. For each channel that is marked as used in the channel map, adds it to the channel table
/// 3. Keeps track of how many channels are available in BLE_LL_CHANNEL_NUM
///
/// # Parameters
/// * `channel` - A byte array representing the channel map where each bit indicates if a channel
///   is used (1) or unused (0). The byte array is structured as a bitmap.
/// * `reset` - If true, resets the last unmapped channel index to 0
///
/// # BLE Channel Structure
/// In BLE, there are 40 channels (0-39), with channels 37, 38, and 39 reserved for advertising.
/// This function handles data channels (0-36) which are used for communication after connection
/// establishment.
pub fn ble_ll_build_available_channel_table(channel: &[u8], reset: bool)
{
    // Reset the channel counter
    BLE_LL_CHANNEL_NUM.set(0);

    // Reset the last unmapped channel if requested
    if reset {
        BLE_LL_LAST_UNMAPPED_CH.set(0);
    }

    // Iterate through all possible channel IDs (0-39)
    // Channels 0-36 are data channels for connection events
    for chan_id in 0..37 {
        // Calculate byte index and bit position for this channel
        let byte_idx = chan_id >> 3;     // Divide by 8 to get byte index
        let bit_pos = chan_id & 0x07;    // Modulo 8 to get bit position within byte
        
        // Check if this channel is marked as used (bit value is 1) in the channel map
        // In BLE, a bit value of 1 means the channel is used, 0 means unused
        if (channel[byte_idx] & (1 << bit_pos)) != 0 {
            let chan_num = BLE_LL_CHANNEL_NUM.get();
            // Add this channel to our table of usable channels
            BLE_LL_CHANNEL_TABLE.lock()[chan_num] = chan_id as u8;

            // Increment the count of usable channels
            BLE_LL_CHANNEL_NUM.inc();
        }
    }
}

/// Determines the next BLE channel to use based on the channel map and hop value.
///
/// # Algorithm
/// This function implements the BLE channel selection algorithm:
/// 1. Calculates a new unmapped channel index based on the current index plus the hop value
/// 2. If the calculated channel is unused according to the channel map, remaps to a usable channel
/// 3. Returns the selected channel index
///
/// # Parameters
/// * `channel_map` - A byte array representing the channel map where each bit indicates if a 
///   channel is used (1) or unused (0)
/// * `hop` - The hop increment value used to calculate the next channel in the sequence
///
/// # Returns
/// The index of the next channel to use (0-39)
///
/// # BLE Channel Selection
/// This implements a modified version of the "Channel Selection Algorithm #1" from the BLE spec.
/// The algorithm ensures that if a channel is masked out in the channel_map, it gets remapped
/// to a valid channel from the channel table.
pub fn ble_ll_select_next_data_channel(channel_map: &[u8], hop: u8) -> u32
{
    // Calculate the next unmapped channel index
    // 0x25 = 37, so we're ensuring the result is within data channels (0-36)
    // This implements the hop portion of Channel Selection Algorithm #1
    let mut index = (BLE_LL_LAST_UNMAPPED_CH.get() + hop as usize) % 0x25;
    BLE_LL_LAST_UNMAPPED_CH.set(index);

    // Check if the selected channel is valid according to the channel map
    // In the BLE spec, a bit value of 1 means the channel is used, 0 means unused
    // Calculate byte index and bit position for this channel
    let byte_idx = index >> 3;      // Divide by 8 to get byte index
    let bit_pos = index & 0x07;     // Modulo 8 to get bit position within byte
    
    // Check if the channel is used (bit is set to 1)
    if (channel_map[byte_idx] & (1 << bit_pos)) == 0 {
        // If the channel is not valid (bit is 0), remap it to a valid one from the channel table
        // We use modulo to ensure we pick a valid channel within the available range
        // This is the remapping feature of Channel Selection Algorithm #1
        if BLE_LL_CHANNEL_NUM.get() > 0 {
            index = BLE_LL_CHANNEL_TABLE.lock()[(index % BLE_LL_CHANNEL_NUM.get())] as usize;
        }
    }

    index as u32
}

#[cfg(test)]
mod tests {
    use mry::Any;
    use super::*;

    #[test]
    fn test_ble_ll_build_available_channel_table_all_channels() {
        // Arrange
        let channel_map = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF];

        // Reset static variables
        BLE_LL_CHANNEL_NUM.set(0);
        BLE_LL_LAST_UNMAPPED_CH.set(0);
        let mut channel_table = [0u8; 40];
        BLE_LL_CHANNEL_TABLE.lock().copy_from_slice(&channel_table);

        // Act
        ble_ll_build_available_channel_table(&channel_map, true);

        // Assert
        assert_eq!(BLE_LL_LAST_UNMAPPED_CH.get(), 0);
        // Only data channels 0-36 should be populated (not advertising channels 37-39)
        assert_eq!(BLE_LL_CHANNEL_NUM.get(), 37);
        for i in 0..37 {
            assert_eq!(BLE_LL_CHANNEL_TABLE.lock()[i], i as u8);
        }
    }

    #[test]
    fn test_ble_ll_build_available_channel_table_no_channels() {
        // Arrange
        let channel_map = [0x00, 0x00, 0x00, 0x00, 0x00];

        // Reset static variables
        BLE_LL_CHANNEL_NUM.set(0);
        BLE_LL_LAST_UNMAPPED_CH.set(0);
        let mut channel_table = [0u8; 40];
        BLE_LL_CHANNEL_TABLE.lock().copy_from_slice(&channel_table);

        // Act
        ble_ll_build_available_channel_table(&channel_map, false);

        // Assert
        assert_eq!(BLE_LL_LAST_UNMAPPED_CH.get(), 0); // Not reset
        assert_eq!(BLE_LL_CHANNEL_NUM.get(), 0);
    }

    #[test]
    fn test_ble_ll_build_available_channel_table_selective_channels() {
        // Arrange
        let channel_map = [0xAA, 0xAA, 0xAA, 0xAA, 0xAA];

        // Reset static variables
        BLE_LL_CHANNEL_NUM.set(0);
        BLE_LL_LAST_UNMAPPED_CH.set(0);
        let mut channel_table = [0u8; 40];
        BLE_LL_CHANNEL_TABLE.lock().copy_from_slice(&channel_table);

        // Act
        ble_ll_build_available_channel_table(&channel_map, true);

        // Assert
        let mut expected_count = 0;
        for i in 0..37 { // Only check data channels 0-36
            if i % 2 == 1 { // Every other bit is set in 0xAA pattern
                assert_eq!(BLE_LL_CHANNEL_TABLE.lock()[expected_count], i as u8);
                expected_count += 1;
            }
        }
        assert_eq!(BLE_LL_CHANNEL_NUM.get(), 18); // Only 18 of the data channels match (not 20)
    }

    #[test]
    fn test_ble_ll_select_next_data_channel_direct_mapping() {
        // Arrange
        // Mock all necessary state
        let channel_map = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF];  // All channels enabled
        let hop = 5;
        
        // Reset static variables
        BLE_LL_LAST_UNMAPPED_CH.set(10);
        
        // Act
        let next_channel = ble_ll_select_next_data_channel(&channel_map, hop);
        
        // Assert
        // Check that the new channel was calculated correctly: (10 + 5) % 37 = 15
        assert_eq!(next_channel, 15);
        assert_eq!(BLE_LL_LAST_UNMAPPED_CH.get(), 15);
    }

    #[test]
    fn test_ble_ll_select_next_data_channel_remapped() {
        // Arrange
        // Create a channel map where channel 15 isn't available
        let mut channel_map = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        // Disable channel 15 by clearing its bit in the channel map
        // Channel 15 is in byte 1 (index 1), bit 7 (index & 0x07)
        channel_map[15 >> 3] &= !(1 << (15 & 0x07));
        
        // Reset static variables
        BLE_LL_LAST_UNMAPPED_CH.set(10);
        BLE_LL_CHANNEL_NUM.set(30);  // 30 valid channels
        
        // Mock BLE_LL_CHANNEL_TABLE to return a specific value for remapping
        let mut channel_table = [0u8; 40];
        channel_table[15 % 30] = 20;  // Remap to channel 20
        BLE_LL_CHANNEL_TABLE.lock().copy_from_slice(&channel_table);
        
        let hop = 5;
        
        // Act
        let next_channel = ble_ll_select_next_data_channel(&channel_map, hop);
        
        // Assert
        // Should be remapped to channel 20
        assert_eq!(next_channel, 20);
        assert_eq!(BLE_LL_LAST_UNMAPPED_CH.get(), 15);
    }

    #[test]
    fn test_ble_ll_select_next_data_channel_wrap_around() {
        // Arrange
        // Mock all necessary state
        let channel_map = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF];  // All channels enabled
        let hop = 10;
        
        // Reset static variables
        BLE_LL_LAST_UNMAPPED_CH.set(30);
        
        // Act
        let next_channel = ble_ll_select_next_data_channel(&channel_map, hop);
        
        // Assert
        // Check that the new channel wrapped around correctly: (30 + 10) % 37 = 3
        assert_eq!(next_channel, 3);
        assert_eq!(BLE_LL_LAST_UNMAPPED_CH.get(), 3);
    }

    #[test]
    fn test_ble_ll_channel_and_next_channel_integration() {
        // This test verifies the integration between the two functions
        // by first calculating the channel table and then using it for channel selection

        // Arrange
        // Create a channel map with selective channels enabled
        let channel_map = [0xF0, 0xF0, 0xF0, 0xF0, 0x00];  // Only certain channels available
        
        // Reset static variables
        BLE_LL_CHANNEL_NUM.set(0);
        BLE_LL_LAST_UNMAPPED_CH.set(5); // Start from channel 5
        let mut channel_table = [0u8; 40];
        BLE_LL_CHANNEL_TABLE.lock().copy_from_slice(&channel_table);

        // Act
        // First calculate the channel table
        ble_ll_build_available_channel_table(&channel_map, false);
        
        // Then get the next channel
        let next_channel = ble_ll_select_next_data_channel(&channel_map, 3);
        
        // Assert
        // Verify that we have the expected number of channels
        // With 0xF0 pattern in each byte, we should have 4 bits per byte enabled
        assert_eq!(BLE_LL_CHANNEL_NUM.get(), 16);
        
        // The next channel should be properly selected
        assert!(next_channel < 40);
    }
}
