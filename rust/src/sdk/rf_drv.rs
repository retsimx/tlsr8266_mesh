// Radio Frequency Driver for TLSR8266 Bluetooth Mesh
//
// This driver manages Bluetooth Low Energy (BLE) advertisement packets,
// device addressing, and group management for mesh networking.
// It handles flash storage of addresses, advertisement data manipulation,
// and radio frequency power levels.

use core::ptr::addr_of;
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::config::FLASH_ADR_DEV_GRP_ADR;
use crate::main_light::rf_link_light_event_callback;
use crate::sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};
use crate::sdk::light::{DEVICE_ADDR_MASK_DEFAULT, MAX_GROUP_COUNT};
use crate::state::{*};

/// Radio frequency power levels available for the BLE radio
/// Values are in dBm (decibels relative to 1 milliwatt)
pub enum RfPower {
    /// 8 dBm power level (highest power)
    Power8dBm = 0,
    /// 4 dBm power level
    Power4dBm = 1,
    /// 0 dBm power level (1 milliwatt)
    Power0dBm = 2,
    /// -4 dBm power level
    PowerNeg4dBm = 3,
    /// -10 dBm power level
    PowerNeg10dBm = 4,
    /// -14 dBm power level
    PowerNeg14dBm = 5,
    /// -20 dBm power level
    PowerNeg20dBm = 6,
    /// -24 dBm power level
    PowerNeg24dBm = 8,
    /// -28 dBm power level
    PowerNeg28dBm = 9,
    /// -30 dBm power level
    PowerNeg30dBm = 10,
    /// -37 dBm power level (lowest power)
    PowerNeg37dBm = 11,
    /// Radio completely powered off
    PowerOff = 16,
}

/// Constants for BLE advertisement packet structure
const ADV_TYPE_MESH_NAME: u8 = 9;     // BLE advertisement type for mesh name
const ADV_TYPE_MANUFACTURER: u8 = 0xff; // BLE advertisement type for manufacturer data
const BLE_HEADER_LEN: u8 = 6;         // BLE packet header length
const DMA_HEADER_EXTRA: u32 = 8;      // Additional DMA buffer length beyond RF packet
const MAX_ADV_DATA_LEN: u8 = 0x25;    // Maximum advertisement data length (37 bytes)

/// Helper structure to track advertisement data field location
#[derive(Debug)]
struct AdvFieldLocation {
    type_offset: usize,      // Offset to the length field
    adv_type_offset: usize,  // Offset to the type field  
    data_offset: usize,      // Offset to the data field
}

/// Helper function to check if advertisement data is empty
fn is_adv_data_empty(pkt: &crate::sdk::packet_types::Packet) -> bool {
    pkt.head().rf_len as i8 - (BLE_HEADER_LEN as i8) < 1
}

/// Iterator that yields advertisement data fields as (offset, length, type)
fn adv_fields_iter(data: &[u8], max_len: usize) -> impl Iterator<Item = (usize, u8, u8)> + '_ {
    (0..max_len)
        .scan(0, move |offset, _| {
            if *offset >= max_len {
                return None;
            }
            
            let field_len = data.get(*offset).copied().unwrap_or(0);
            if field_len == 0 {
                return None; // Malformed data
            }
            
            let field_type = data.get(*offset + 1).copied().unwrap_or(0);
            let current_offset = *offset;
            
            *offset += field_len as usize + 1; // Move to next field
            Some((current_offset, field_len, field_type))
        })
}

/// Helper function to find or allocate space for an advertisement field type
fn find_or_allocate_adv_field(pkt: &mut crate::sdk::packet_types::Packet, field_type: u8) -> AdvFieldLocation {
    if is_adv_data_empty(pkt) {
        return AdvFieldLocation {
            type_offset: 0,
            adv_type_offset: 1,
            data_offset: 2,
        };
    }

    let adv_data_len = (pkt.head().rf_len as i8 - BLE_HEADER_LEN as i8) as usize;
    let data = &pkt.adv_ind_module().data;
    
    // Use iterator to find existing field
    adv_fields_iter(data, adv_data_len)
        .find(|(_, _, type_val)| *type_val == field_type)
        .map(|(offset, _, _)| AdvFieldLocation {
            type_offset: offset,
            adv_type_offset: offset + 1,
            data_offset: offset + 2,
        })
        .unwrap_or_else(|| {
            // Field not found, calculate end position
            let type_offset = adv_fields_iter(data, adv_data_len)
                .last()
                .map(|(offset, len, _)| offset + len as usize + 1)
                .unwrap_or(0);
            
            AdvFieldLocation {
                type_offset,
                adv_type_offset: type_offset + 1,
                data_offset: type_offset + 2,
            }
        })
}

/// Helper function to update packet length fields
fn update_packet_lengths(pkt: &mut crate::sdk::packet_types::Packet, total_data_len: usize) {
    pkt.head_mut().rf_len = (total_data_len + BLE_HEADER_LEN as usize) as u8;
    pkt.head_mut().dma_len = (total_data_len + DMA_HEADER_EXTRA as usize) as u32;
}

/// Helper function to clear all groups from memory
fn clear_all_groups_from_memory() -> bool {
    critical_section::with(|_| {
        let mut groups = GROUP_ADDRESS.lock();
        let had_groups = groups.iter().any(|&addr| addr != 0);
        
        groups.iter_mut().for_each(|addr| *addr = 0);
        
        had_groups
    })
}

/// Helper function to clear specific group from memory
fn clear_specific_group_from_memory(group_id: u16) -> bool {
    critical_section::with(|_| {
        GROUP_ADDRESS.lock()
            .iter_mut()
            .find(|addr| **addr == group_id)
            .map(|addr| { *addr = 0; true })
            .unwrap_or(false)
    })
}

/// Helper function to delete groups from flash storage
fn delete_groups_from_flash(group_id: u16, delete_all: bool) -> bool {
    let initial_pos: i16 = MESH_GROUP_ADDRESS_NEXT_POSITION.get().try_into().unwrap();
    let zero_short = 0u16;
    
    // Create iterator over flash positions in reverse order (newest to oldest)
    let flash_positions = (0..initial_pos).step_by(2).rev();
    
    let mut groups_deleted = 0;
    let mut found_any = false;
    
    for pos in flash_positions {
        let flash_addr = FLASH_ADR_DEV_GRP_ADR + pos as u32;
        
        // Read group address from flash
        let group_address = {
            let mut bytes = [0u8; 2];
            unsafe {
                flash_read_page(flash_addr, 2, bytes.as_mut_ptr());
            }
            u16::from_le_bytes(bytes)
        };
        
        // Check if this is a group address (not device address)
        let is_group_address = group_address & !DEVICE_ADDR_MASK_DEFAULT != 0;
        
        if is_group_address {
            let should_delete = delete_all || group_address == group_id;
            
            if should_delete {
                unsafe {
                    flash_write_page(flash_addr, DEVICE_ADDR_SIZE, addr_of!(zero_short) as *const u8);
                }
                found_any = true;
                groups_deleted += 1;
                
                // Early termination conditions
                match (delete_all, groups_deleted) {
                    (true, count) if count > 7 => return true,  // Delete-all limit
                    (false, _) => return true,                   // Found specific group
                    _ => continue,
                }
            }
        }
    }
    
    found_any
}

/// Helper function to compact advertisement data by removing invalid manufacturer data
/// Returns the new end position for the compacted data
fn compact_adv_data_removing_invalid_manufacturer(pkt: &mut crate::sdk::packet_types::Packet) -> usize {
    if is_adv_data_empty(pkt) {
        return 0;
    }

    let adv_data_len = (pkt.head().rf_len as i8 - BLE_HEADER_LEN as i8) as usize;
    let data = &pkt.adv_ind_module().data;
    
    // Process fields in-place using functional iterator patterns
    let data_clone = pkt.adv_ind_module().data.clone();
    let mut write_pos = 0;
    
    // Use iterator to process non-manufacturer fields
    for (read_offset, field_len, _field_type) in adv_fields_iter(&data_clone, adv_data_len)
        .filter(|(_, _, field_type)| *field_type != ADV_TYPE_MANUFACTURER)
    {
        let field_size = field_len as usize + 1;
        
        // Copy field to compacted position
        pkt.adv_ind_module_mut().data[write_pos..write_pos + field_size]
            .copy_from_slice(&data_clone[read_offset..read_offset + field_size]);
        
        write_pos += field_size;
    }
    
    write_pos
}

/// Sets the mesh network name in BLE advertisement packets
/// 
/// This function manages the Complete Local Name field (BLE type 0x09) in advertisement
/// packets. It will either update an existing mesh name field or create a new one if
/// none exists, preserving other advertisement data.
///
/// # Arguments
/// * `name` - The mesh network name as a byte slice (typically UTF-8 encoded)
///
/// # Behavior
/// - Finds existing mesh name field or allocates space for a new one
/// - Updates packet length fields automatically
/// - Preserves other advertisement data structures
pub fn set_advertisement_mesh_name(name: &[u8])
{
    let mut pkt_adv = PKT_ADV.lock();

    // Find or allocate space for mesh name field
    let location = find_or_allocate_adv_field(&mut pkt_adv, ADV_TYPE_MESH_NAME);
    
    // Copy mesh name data to advertisement packet
    pkt_adv.adv_ind_module_mut().data[location.data_offset..location.data_offset + name.len()].copy_from_slice(name);

    // Set length and type fields
    pkt_adv.adv_ind_module_mut().data[location.type_offset] = name.len() as u8 + 1;
    pkt_adv.adv_ind_module_mut().data[location.adv_type_offset] = ADV_TYPE_MESH_NAME;
    
    // Update packet length fields
    let total_data_len = location.type_offset + name.len() + 2;
    update_packet_lengths(&mut pkt_adv, total_data_len);
}

/// Sets manufacturer-specific data in BLE advertisement packets
/// 
/// This function manages the Manufacturer Specific Data field (BLE type 0xFF) in 
/// advertisement packets. It removes any existing manufacturer data and replaces
/// it with the provided data, preserving all other advertisement fields.
///
/// # Arguments
/// * `data` - The manufacturer-specific payload data (without length/type headers)
///
/// # Behavior
/// - Removes any existing manufacturer data fields
/// - Compacts remaining advertisement data
/// - Appends new manufacturer data with proper BLE formatting
/// - Updates packet length fields automatically
///
/// # BLE Format
/// The data is formatted as: [Length][Type=0xFF][Data...]
#[cfg_attr(test, mry::mry)]
pub fn set_advertisement_manufacturer_data(data: &[u8])
{
    let mut pkt_adv = PKT_ADV.lock();
    
    // Compact existing data by removing invalid manufacturer data
    let compacted_len = compact_adv_data_removing_invalid_manufacturer(&mut pkt_adv);
    
    // Calculate offsets to match original algorithm exactly
    let type_offset = compacted_len;
    let adv_type_offset = compacted_len + 1;
    let data_offset = compacted_len + 2;
    
    // Copy manufacturer data to advertisement packet
    pkt_adv.adv_ind_module_mut().data[data_offset..data_offset + data.len()].copy_from_slice(data);

    // Set length and type fields
    pkt_adv.adv_ind_module_mut().data[type_offset] = data.len() as u8 + 1;
    pkt_adv.adv_ind_module_mut().data[adv_type_offset] = ADV_TYPE_MANUFACTURER;
    
    // Update packet length to match original calculation exactly
    // Original: total_len = data.len() + 2 + dest_offset, where dest_offset = data_offset
    let total_len = data.len() + 2 + data_offset;
    pkt_adv.head_mut().dma_len = total_len as u32 + DMA_HEADER_EXTRA;
    pkt_adv.head_mut().rf_len = total_len as u8 + BLE_HEADER_LEN;
}

/// Sets UUID data in BLE advertisement packets at a fixed position
/// 
/// This function inserts UUID data at byte position 3 in the advertisement packet,
/// which is a protocol-specific requirement for this mesh implementation. It handles
/// both first-time insertion (shifting existing data) and replacement of existing UUID data.
///
/// # Arguments
/// * `uuid_data` - The UUID payload data to insert
///
/// # Behavior
/// - First time: Inserts at position 3, shifts existing data rightward
/// - Subsequent calls: Replaces existing UUID data in-place
/// - Validates that insertion won't exceed maximum advertisement length
/// - Updates packet length fields for first insertion only
///
/// # Protocol Note
/// The fixed position 3 is required by the mesh networking protocol specification.
pub fn set_advertisement_uuid(uuid_data: &[u8])
{
    let mut pkt_adv = PKT_ADV.lock();
    let rf_len = pkt_adv.head().rf_len as usize;
    
    // Early return pattern with guard clause
    let would_exceed_max = uuid_data.len() as i8 > MAX_ADV_DATA_LEN as i8 - (rf_len as i8);
    if would_exceed_max {
        return;
    }
    
    const UUID_INSERT_POSITION: usize = 3;
    let mut temp_buffer = [0u8; 31];
    
    match SET_UUID_FLAG.get() {
        false => {
            // First time: insert and shift existing data  
            let existing_payload_len = if rf_len >= 9 { rf_len - 9 } else { 0 };
            
            // Functional approach to data manipulation
            let (save_range, insert_range, restore_range) = {
                let save_start = UUID_INSERT_POSITION;
                let save_end = save_start + existing_payload_len;
                let insert_start = UUID_INSERT_POSITION;
                let insert_end = insert_start + uuid_data.len();
                let restore_start = insert_end;
                let restore_end = restore_start + existing_payload_len;
                
                (save_start..save_end, insert_start..insert_end, restore_start..restore_end)
            };
            
            // Save existing data that will be shifted
            temp_buffer[0..existing_payload_len]
                .copy_from_slice(&pkt_adv.adv_ind_module().data[save_range]);
            
            // Insert new UUID data and restore shifted data
            pkt_adv.adv_ind_module_mut().data[insert_range].copy_from_slice(uuid_data);
            pkt_adv.adv_ind_module_mut().data[restore_range]
                .copy_from_slice(&temp_buffer[0..existing_payload_len]);

            SET_UUID_FLAG.set(true);
            
            // Update packet lengths
            let length_increase = uuid_data.len();
            pkt_adv.head_mut().rf_len += length_increase as u8;
            pkt_adv.head_mut().dma_len += length_increase as u32;
        },
        true => {
            // Replace existing UUID data in place
            let replace_range = UUID_INSERT_POSITION..UUID_INSERT_POSITION + uuid_data.len();
            pkt_adv.adv_ind_module_mut().data[replace_range].copy_from_slice(uuid_data);
        }
    }
}

/// Constants for flash memory management
const MAX_FLASH_ADDR_OFFSET: usize = 0xFFF;  // Maximum address offset for flash storage
const GROUP_ADDR_STORAGE_SIZE: u32 = 0x10;   // Size of group address storage (16 bytes)
const DEVICE_ADDR_SIZE: u32 = 2;             // Size of a device address (2 bytes)
const INITIAL_ADDR_OFFSET: usize = 0x12;     // Initial address offset for storage
const GROUP_DELETE_ALL: u16 = 0xFFFF;        // Special value to delete all groups
const EVENT_DEVICE_ADDR_CHANGED: u8 = 0xC6;  // Event code for address changed event

/// Compacts flash storage when address storage space is nearly exhausted
///
/// This function performs flash garbage collection when either the device address
/// or group address storage pointers approach their maximum values. It erases the
/// flash sector and rewrites all current addresses at the beginning, effectively
/// defragmenting the storage space.
///
/// # Behavior
/// - Checks if storage pointers have reached maximum allowed values
/// - Erases the entire flash sector if cleanup is needed
/// - Rewrites current group addresses to the beginning of the sector
/// - Rewrites current device address after group addresses
/// - Resets storage pointers to the beginning
///
/// # Flash Layout After Cleanup
/// ```
/// [Group Addresses: 16 bytes][Device Address: 2 bytes][Free Space...]
/// ```
pub fn compact_flash_storage()
{
    // Check if either address pointer has reached maximum allowed value
    if MAX_FLASH_ADDR_OFFSET < MESH_GROUP_ADDRESS_NEXT_POSITION.get() as usize || MAX_FLASH_ADDR_OFFSET < MESH_DEVICE_ADDRESS_NEXT_POSITION.get() as usize {
        // Erase the entire flash sector
        flash_erase_sector(FLASH_ADR_DEV_GRP_ADR);
        
        // Write current group addresses to flash
        flash_write_page(
            FLASH_ADR_DEV_GRP_ADR, 
            GROUP_ADDR_STORAGE_SIZE, 
            GROUP_ADDRESS.lock().as_ptr() as *const u8
        );
        
        // Write current device address to flash
        let device_address = DEVICE_ADDRESS.get();
        flash_write_page(
            FLASH_ADR_DEV_GRP_ADR + GROUP_ADDR_STORAGE_SIZE, 
            DEVICE_ADDR_SIZE, 
            addr_of!(device_address) as *const u8
        );
        
        // Reset position pointers
        MESH_DEVICE_ADDRESS_NEXT_POSITION.set(INITIAL_ADDR_OFFSET as u16);
        MESH_GROUP_ADDRESS_NEXT_POSITION.set(INITIAL_ADDR_OFFSET as u16);
    }
}

/// Adds a new device address to the mesh network
///
/// This function validates a device address and stores it both in memory and persistent
/// flash storage. It ensures the address conforms to the device address mask and differs
/// from the current address before updating. It also triggers system event callbacks.
///
/// # Arguments
/// * `dev_id` - The device address to set (must be non-zero and conform to device mask)
///
/// # Returns
/// * `true` if the device address was successfully added and stored
/// * `false` if validation failed or the address is already current
///
/// # Validation Rules
/// - Address must be non-zero
/// - Address must conform to `DEVICE_ADDR_MASK_DEFAULT`
/// - Address must differ from current device address
pub fn add_device_address(dev_id: u16) -> bool
{
    // Validate the device ID
    let mut result = false;
    
    // Device ID must be non-zero and conform to the device address mask
    // Also must be different from the current device address
    if dev_id != 0 && 
       dev_id & !DEVICE_ADDR_MASK_DEFAULT == 0 && 
       DEVICE_ADDRESS.get() != dev_id {
        
        // Clean up flash if needed
        compact_flash_storage();
        
        let flash_pos = MESH_GROUP_ADDRESS_NEXT_POSITION.get();
        
        // If flash has been previously used, clear previous address
        if flash_pos != 0 && MESH_DEVICE_ADDRESS_NEXT_POSITION.get() != 0 {
            // Write zero to previous address location 
            let zero_data = 0u16;
            flash_write_page(
                FLASH_ADR_DEV_GRP_ADR - 2 + MESH_DEVICE_ADDRESS_NEXT_POSITION.get() as u32, 
                DEVICE_ADDR_SIZE, 
                addr_of!(zero_data) as *const u8
            );
        }
        
        // Update the device address in memory and in flash
        DEVICE_ADDRESS.set(dev_id);
        flash_write_page(
            flash_pos as u32 + FLASH_ADR_DEV_GRP_ADR, 
            DEVICE_ADDR_SIZE, 
            addr_of!(dev_id) as *const u8
        );
        
        // Update position pointers
        // Convert to u16 after adding to ensure correct type
        MESH_GROUP_ADDRESS_NEXT_POSITION.set((flash_pos as usize + DEVICE_ADDR_SIZE as usize) as u16);
        MESH_DEVICE_ADDRESS_NEXT_POSITION.set(MESH_GROUP_ADDRESS_NEXT_POSITION.get());
        
        // Notify application of address change
        rf_link_light_event_callback(EVENT_DEVICE_ADDR_CHANGED);
        
        result = true;
    }
    
    return result;
}

/// Removes a group address from the mesh network
///
/// This function removes a specific group address, or all group addresses if GROUP_DELETE_ALL
/// from both memory and persistent flash storage. It processes addresses in reverse order
/// (newest to oldest) and limits bulk deletion to prevent excessive flash operations.
///
/// # Arguments
/// * `group_id` - The specific group address to remove, or `GROUP_DELETE_ALL` (0xFFFF) to remove all
///
/// # Returns
/// * `true` if any group address was successfully removed from memory or flash
/// * `false` if no matching addresses were found or storage was empty
///
/// # Behavior
/// - Clears matching addresses from in-memory group table
/// - Searches flash storage from newest to oldest entries
/// - For specific group: removes first matching entry and returns
/// - For delete-all: removes up to 8 group entries to prevent excessive flash wear
/// - Only processes entries identified as group addresses (not device addresses)
#[cfg_attr(test, mry::mry)]
pub fn remove_group(group_id: u16) -> bool
{
    let grp_next_pos: i16 = MESH_GROUP_ADDRESS_NEXT_POSITION.get().try_into().unwrap();
    
    // Early exit if no groups are stored
    if grp_next_pos == 0 {
        return false;
    }
    
    let is_delete_all = group_id == GROUP_DELETE_ALL;
    
    // Clear groups from memory
    let memory_cleared = if is_delete_all {
        clear_all_groups_from_memory()
    } else {
        clear_specific_group_from_memory(group_id)
    };
    
    // Clear groups from flash storage
    let flash_cleared = delete_groups_from_flash(group_id, is_delete_all);
    
    // Return true if either memory or flash was modified
    memory_cleared || flash_cleared
}

/// Adds a new group address to the mesh network with rotation policy
///
/// This function validates a group address and adds it to both memory and persistent
/// flash storage. When the group table is full, it applies a rotation policy to replace
/// the oldest group address, ensuring the device can always join new groups.
///
/// # Arguments
/// * `group_id` - The group address to add (must be in valid group address range)
///
/// # Returns
/// * `true` if the group address was successfully added or updated
/// * `false` if validation failed (invalid group address range)
///
/// # Validation
/// - Group ID must pass `is_valid_group_id()` check (>= 0x8000 and != 0xFFFF)
/// - This ensures the address is in the valid group address space (MSB set, but not the special delete-all value)
///
/// # Rotation Policy
/// - If group table is full (8 entries), replaces the oldest entry
/// - Tracks oldest position using internal static counter
/// - Ensures device can always join new groups without manual cleanup
///
/// # Storage
/// - Updates in-memory group table immediately
/// - Appends to flash storage log for persistence
/// - Triggers flash compaction if storage space is low
pub fn add_group(group_id: u16) -> bool
{
    // Track the oldest group position for rotation policy
    static OLDEST_POS: AtomicUsize = AtomicUsize::new(0xffffffff);
    
    // Validate group ID: must be in valid group address range (MSB set, but not delete-all value)
    if group_id >= 0x8000 && group_id != GROUP_DELETE_ALL {
        // Clean up flash if needed
        compact_flash_storage();
        
        // If this is the first group, add to first position
        if MESH_GROUP_ADDRESS_NEXT_POSITION.get() == 0 {
            GROUP_ADDRESS.lock()[0] = group_id;
        } else {
            // Check existing groups
            for index in 0..MAX_GROUP_COUNT as usize {
                // Return if group already exists
                if group_id == GROUP_ADDRESS.lock()[index] {
                    return false;
                }
                
                // Found empty slot, add group here
                if GROUP_ADDRESS.lock()[index] == 0 {
                    GROUP_ADDRESS.lock()[index] = group_id;
                    
                    // Write to flash
                    flash_write_page(
                        MESH_GROUP_ADDRESS_NEXT_POSITION.get() as u32 + FLASH_ADR_DEV_GRP_ADR, 
                        DEVICE_ADDR_SIZE, 
                        addr_of!(group_id) as *const u8
                    );
                    
                    // Update position pointer
                    MESH_GROUP_ADDRESS_NEXT_POSITION.set((MESH_GROUP_ADDRESS_NEXT_POSITION.get() as usize + DEVICE_ADDR_SIZE as usize) as u16);
                    return true;
                }
            }
            
            // No empty slot found, replace oldest group using rotation policy
            if OLDEST_POS.load(Ordering::Relaxed) == 0xffffffff {
                OLDEST_POS.store(0, Ordering::Relaxed);
            }
            
            // Remove the oldest group
            let oldest_index = OLDEST_POS.load(Ordering::Relaxed);
            remove_group(GROUP_ADDRESS.lock()[oldest_index]);
            
            // Add new group in its place
            GROUP_ADDRESS.lock()[oldest_index] = group_id;
            
            // Advance oldest position for next replacement
            OLDEST_POS.store((oldest_index + 1) % MAX_GROUP_COUNT as usize, Ordering::Relaxed);
        }
        
        // Write group address to flash
        flash_write_page(
            MESH_GROUP_ADDRESS_NEXT_POSITION.get() as u32 + FLASH_ADR_DEV_GRP_ADR, 
            DEVICE_ADDR_SIZE, 
            addr_of!(group_id) as *const u8
        );
        
        // Update position pointer
        MESH_GROUP_ADDRESS_NEXT_POSITION.set((MESH_GROUP_ADDRESS_NEXT_POSITION.get() as usize + DEVICE_ADDR_SIZE as usize) as u16);
        return true;
    }
    
    return false;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdk::drivers::flash::{mock_flash_erase_sector, mock_flash_read_page, mock_flash_write_page};
    use crate::main_light::mock_rf_link_light_event_callback;
    use super::mock_remove_group;
    use mry::send_wrapper::SendWrapper;
    
    /// Helper function to reset global state for clean test environment
    fn reset_test_state() {
        // Reset all global state variables
        DEVICE_ADDRESS.set(0);
        MESH_DEVICE_ADDRESS_NEXT_POSITION.set(0);
        MESH_GROUP_ADDRESS_NEXT_POSITION.set(0);
        SET_UUID_FLAG.set(false);
        
        // Clear group addresses
        critical_section::with(|_| {
            GROUP_ADDRESS.lock().iter_mut().for_each(|addr| *addr = 0);
        });
        
        // Reset PKT_ADV to a clean state
        critical_section::with(|_| {
            let mut pkt = PKT_ADV.lock();
            pkt.head_mut().dma_len = DMA_HEADER_EXTRA;
            pkt.head_mut().rf_len = BLE_HEADER_LEN;
            pkt.adv_ind_module_mut().data.iter_mut().for_each(|b| *b = 0);
        });
    }
    
    /// Helper function to create a test advertisement packet with specific data
    fn create_test_adv_packet(initial_data: &[u8]) {
        critical_section::with(|_| {
            let mut pkt = PKT_ADV.lock();
            pkt.head_mut().rf_len = BLE_HEADER_LEN + initial_data.len() as u8;
            pkt.head_mut().dma_len = DMA_HEADER_EXTRA + initial_data.len() as u32;
            pkt.adv_ind_module_mut().data[0..initial_data.len()].copy_from_slice(initial_data);
        });
    }

    /// Tests that `adv_fields_iter` stops iterating when a zero-length field is encountered.
    #[test]
    fn test_adv_fields_iter_early_termination_on_zero_length() {
        let data = [0u8; 5];
        let mut iter = adv_fields_iter(&data, data.len());
        assert!(iter.next().is_none());
    }
    
    /// Tests the RfPower enum values
    ///
    /// Verifies that RF power levels have the expected numeric values
    /// for proper hardware configuration.
    #[test]
    fn test_rf_power_enum_values() {
        assert_eq!(RfPower::Power8dBm as u8, 0);
        assert_eq!(RfPower::Power4dBm as u8, 1);
        assert_eq!(RfPower::Power0dBm as u8, 2);
        assert_eq!(RfPower::PowerNeg4dBm as u8, 3);
        assert_eq!(RfPower::PowerNeg10dBm as u8, 4);
        assert_eq!(RfPower::PowerNeg14dBm as u8, 5);
        assert_eq!(RfPower::PowerNeg20dBm as u8, 6);
        assert_eq!(RfPower::PowerNeg24dBm as u8, 8);
        assert_eq!(RfPower::PowerNeg28dBm as u8, 9);
        assert_eq!(RfPower::PowerNeg30dBm as u8, 10);
        assert_eq!(RfPower::PowerNeg37dBm as u8, 11);
        assert_eq!(RfPower::PowerOff as u8, 16);
    }
    
    /// Tests set_advertisement_mesh_name with empty advertisement data
    ///
    /// Verifies that the function correctly initializes a new Complete Local Name
    /// field when the advertisement data section is empty.
    #[test]
    fn test_set_adv_mesh_name_empty_data() {
        reset_test_state();
        
        let mesh_name = b"TestMesh";
        set_advertisement_mesh_name(mesh_name);
        
        critical_section::with(|_| {
            let pkt = PKT_ADV.lock();
            
            // Check packet length is updated correctly
            let rf_len = pkt.head().rf_len;
            let dma_len = pkt.head().dma_len;
            assert_eq!(rf_len, BLE_HEADER_LEN + mesh_name.len() as u8 + 2);
            assert_eq!(dma_len, DMA_HEADER_EXTRA + mesh_name.len() as u32 + 2);
            
            // Check AD structure format
            assert_eq!(pkt.adv_ind_module().data[0], mesh_name.len() as u8 + 1); // Length
            assert_eq!(pkt.adv_ind_module().data[1], ADV_TYPE_MESH_NAME); // Type
            
            // Check mesh name data
            assert_eq!(&pkt.adv_ind_module().data[2..2 + mesh_name.len()], mesh_name);
        });
    }
    
    /// Tests set_advertisement_mesh_name with existing mesh name
    ///
    /// Verifies that the function correctly replaces an existing Complete Local Name
    /// field with new data.
    #[test]
    fn test_set_adv_mesh_name_replace_existing() {
        reset_test_state();
        
        // Set up initial advertisement data with existing mesh name
        let initial_data = [0x05, ADV_TYPE_MESH_NAME, b'O', b'l', b'd', b'1'];
        create_test_adv_packet(&initial_data);
        
        let new_mesh_name = b"NewMesh";
        set_advertisement_mesh_name(new_mesh_name);
        
        critical_section::with(|_| {
            let pkt = PKT_ADV.lock();
            
            // Check packet length is updated correctly
            assert_eq!(pkt.head().rf_len, BLE_HEADER_LEN + new_mesh_name.len() as u8 + 2);
            
            // Check AD structure format
            assert_eq!(pkt.adv_ind_module().data[0], new_mesh_name.len() as u8 + 1); // Length
            assert_eq!(pkt.adv_ind_module().data[1], ADV_TYPE_MESH_NAME); // Type
            
            // Check new mesh name data
            assert_eq!(&pkt.adv_ind_module().data[2..2 + new_mesh_name.len()], new_mesh_name);
        });
    }
    
    /// Tests set_advertisement_mesh_name with existing non-mesh data
    ///
    /// Verifies that the function correctly appends a Complete Local Name field
    /// when other advertisement data exists but no mesh name is present.
    #[test]
    fn test_set_adv_mesh_name_with_other_data() {
        reset_test_state();
        
        // Set up initial advertisement data with manufacturer data
        let initial_data = [0x05, ADV_TYPE_MANUFACTURER, 0x01, 0x02, 0x03, 0x04];
        create_test_adv_packet(&initial_data);
        
        let mesh_name = b"Test";
        set_advertisement_mesh_name(mesh_name);
        
        critical_section::with(|_| {
            let pkt = PKT_ADV.lock();
            
            // Check that original data is preserved
            assert_eq!(pkt.adv_ind_module().data[0], 0x05);
            assert_eq!(pkt.adv_ind_module().data[1], ADV_TYPE_MANUFACTURER);
            
            // Check new mesh name is appended
            let mesh_start = 6; // After the initial data
            assert_eq!(pkt.adv_ind_module().data[mesh_start], mesh_name.len() as u8 + 1);
            assert_eq!(pkt.adv_ind_module().data[mesh_start + 1], ADV_TYPE_MESH_NAME);
            assert_eq!(&pkt.adv_ind_module().data[mesh_start + 2..mesh_start + 2 + mesh_name.len()], mesh_name);
        });
    }
    
    /// Tests rf_link_slave_set_adv_private_data with empty advertisement data
    ///
    /// Verifies that the function correctly initializes manufacturer-specific data
    /// when the advertisement data section is empty.
    #[test]
    fn test_set_adv_private_data_empty() {
        reset_test_state();
        
        let private_data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
        set_advertisement_manufacturer_data(&private_data);
        
        critical_section::with(|_| {
            let pkt = PKT_ADV.lock();
            
            // Check packet length is updated correctly
            let rf_len = pkt.head().rf_len;
            let dma_len = pkt.head().dma_len;
            assert_eq!(rf_len, BLE_HEADER_LEN + private_data.len() as u8 + 4); // +2 for header + 2 for dest_offset
            assert_eq!(dma_len, DMA_HEADER_EXTRA + private_data.len() as u32 + 4); // +2 for header + 2 for dest_offset
            
            // Check AD structure format
            assert_eq!(pkt.adv_ind_module().data[0], private_data.len() as u8 + 1); // Length
            assert_eq!(pkt.adv_ind_module().data[1], ADV_TYPE_MANUFACTURER); // Type
            
            // Check private data
            assert_eq!(&pkt.adv_ind_module().data[2..2 + private_data.len()], &private_data);
        });
    }
    
    /// Tests rf_link_slave_set_adv_private_data with existing manufacturer data
    ///
    /// Verifies that the function correctly replaces existing manufacturer data
    /// with the expected length (10 bytes total including type).
    #[test]
    fn test_set_adv_private_data_replace_existing() {
        reset_test_state();
        
        // Set up existing manufacturer data with correct length (9 data + 1 type = 10 total)
        let initial_data = [0x09, ADV_TYPE_MANUFACTURER, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22];
        create_test_adv_packet(&initial_data);
        
        let new_private_data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09];
        set_advertisement_manufacturer_data(&new_private_data);
        
        critical_section::with(|_| {
            let pkt = PKT_ADV.lock();
            
            // Check AD structure format
            assert_eq!(pkt.adv_ind_module().data[0], new_private_data.len() as u8 + 1); // Length
            assert_eq!(pkt.adv_ind_module().data[1], ADV_TYPE_MANUFACTURER); // Type
            
            // Check new private data
            assert_eq!(&pkt.adv_ind_module().data[2..2 + new_private_data.len()], &new_private_data);
        });
    }
    
    /// Tests rf_link_slave_set_adv_private_data preserving other data
    ///
    /// Verifies that the function preserves other advertisement data structures
    /// while updating only the manufacturer-specific data.
    #[test]
    fn test_set_adv_private_data_preserve_other() {
        reset_test_state();
        
        // Set up data with mesh name and wrong-length manufacturer data
        let initial_data = [
            0x05, ADV_TYPE_MESH_NAME, b'T', b'e', b's', b't',  // Mesh name
            0x03, ADV_TYPE_MANUFACTURER, 0xAA, 0xBB  // Short manufacturer data (will be filtered out)
        ];
        create_test_adv_packet(&initial_data);
        
        let private_data = [0x01, 0x02, 0x03, 0x04];
        set_advertisement_manufacturer_data(&private_data);
        
        critical_section::with(|_| {
            let pkt = PKT_ADV.lock();
            
            // Check that mesh name is preserved
            assert_eq!(pkt.adv_ind_module().data[0], 0x05);
            assert_eq!(pkt.adv_ind_module().data[1], ADV_TYPE_MESH_NAME);
            assert_eq!(&pkt.adv_ind_module().data[2..6], b"Test");
            
            // Check new manufacturer data is appended
            assert_eq!(pkt.adv_ind_module().data[6], private_data.len() as u8 + 1);
            assert_eq!(pkt.adv_ind_module().data[7], ADV_TYPE_MANUFACTURER);
            assert_eq!(&pkt.adv_ind_module().data[8..8 + private_data.len()], &private_data);
        });
    }
    
    /// Tests rf_link_slave_set_adv_uuid_data first time setting
    ///
    /// Verifies that UUID data is correctly inserted at position 3 and existing
    /// payload is shifted when UUID is set for the first time.
    #[test]
    fn test_set_adv_uuid_data_first_time() {
        reset_test_state();
        
        // Set up existing advertisement data
        let initial_data = [0x05, ADV_TYPE_MESH_NAME, b'T', b'e', b's', b't'];
        create_test_adv_packet(&initial_data);
        
        let uuid_data = [0x01, 0x02, 0x03, 0x04];
        set_advertisement_uuid(&uuid_data);
        
        critical_section::with(|_| {
            let pkt = PKT_ADV.lock();
            
            // Check UUID flag is set
            assert_eq!(SET_UUID_FLAG.get(), true);
            
            // Check packet length is updated
            assert_eq!(pkt.head().rf_len, BLE_HEADER_LEN + initial_data.len() as u8 + uuid_data.len() as u8);
            
            // Check that first 3 bytes are preserved
            assert_eq!(&pkt.adv_ind_module().data[0..3], &initial_data[0..3]);
            
            // Check UUID data is inserted at position 3
            assert_eq!(&pkt.adv_ind_module().data[3..3 + uuid_data.len()], &uuid_data);
            
            // Check that remaining data is shifted
            let remaining_start = 3 + uuid_data.len();
            assert_eq!(&pkt.adv_ind_module().data[remaining_start..remaining_start + 3], &initial_data[3..6]);
        });
    }
    
    /// Tests rf_link_slave_set_adv_uuid_data replacing existing UUID
    ///
    /// Verifies that existing UUID data is correctly replaced when UUID flag
    /// is already set.
    #[test]
    fn test_set_adv_uuid_data_replace_existing() {
        reset_test_state();
        SET_UUID_FLAG.set(true);
        
        // Set up data with existing UUID section
        let initial_data = [0x05, 0x09, 0x06, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];  // UUID at position 3-8
        create_test_adv_packet(&initial_data);
        
        let new_uuid_data = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        set_advertisement_uuid(&new_uuid_data);
        
        critical_section::with(|_| {
            let pkt = PKT_ADV.lock();
            
            // Check that first 3 bytes are preserved
            assert_eq!(&pkt.adv_ind_module().data[0..3], &initial_data[0..3]);
            
            // Check new UUID data replaces old
            assert_eq!(&pkt.adv_ind_module().data[3..3 + new_uuid_data.len()], &new_uuid_data);
        });
    }
    
    /// Tests rf_link_slave_set_adv_uuid_data exceeding maximum length
    ///
    /// Verifies that the function correctly handles cases where adding UUID data
    /// would exceed the maximum advertisement data length.
    #[test]
    fn test_set_adv_uuid_data_max_length_exceeded() {
        reset_test_state();
        
        // Create data that would exceed max length when UUID is added
        let large_data = [0x42; 28]; // Close to max length (31 - 3 = 28 to leave room for UUID header)
        create_test_adv_packet(&large_data);
        
        let uuid_data = [0x01, 0x02, 0x03, 0x04, 0x05];
        set_advertisement_uuid(&uuid_data);
        
        critical_section::with(|_| {
            let pkt = PKT_ADV.lock();
            
            // UUID flag should not be set
            assert_eq!(SET_UUID_FLAG.get(), false);
            
            // Data should remain unchanged
            assert_eq!(pkt.head().rf_len, BLE_HEADER_LEN + large_data.len() as u8);
            assert_eq!(&pkt.adv_ind_module().data[0..large_data.len()], &large_data);
        });
    }
    
    /// Tests dev_grp_flash_clean when cleanup is not needed
    ///
    /// Verifies that flash cleanup is skipped when position pointers
    /// are within acceptable limits.
    #[test]
    #[mry::lock(flash_erase_sector, flash_write_page)]
    fn test_dev_grp_flash_clean_no_cleanup_needed() {
        reset_test_state();
        
        // Set positions well within limits
        MESH_GROUP_ADDRESS_NEXT_POSITION.set(100);
        MESH_DEVICE_ADDRESS_NEXT_POSITION.set(200);
        
        // Mock flash operations - these should not be called
        mock_flash_erase_sector(mry::Any).returns(());
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).returns(());
        
        // Call function
        compact_flash_storage();
        
        // Verify positions unchanged (no cleanup occurred)
        assert_eq!(MESH_GROUP_ADDRESS_NEXT_POSITION.get(), 100);
        assert_eq!(MESH_DEVICE_ADDRESS_NEXT_POSITION.get(), 200);
        
        // Verify flash functions were not called
        mock_flash_erase_sector(mry::Any).assert_called(0);
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).assert_called(0);
    }
    
    /// Tests dev_grp_flash_clean when cleanup is needed
    ///
    /// Verifies that flash cleanup correctly erases sector and rewrites data
    /// when position pointers exceed maximum values.
    #[test]
    #[mry::lock(flash_erase_sector, flash_write_page)]
    fn test_dev_grp_flash_clean_cleanup_needed() {
        reset_test_state();
        
        // Set positions beyond limits to trigger cleanup
        MESH_GROUP_ADDRESS_NEXT_POSITION.set((MAX_FLASH_ADDR_OFFSET + 1) as u16);
        MESH_DEVICE_ADDRESS_NEXT_POSITION.set(100);
        
        // Set some test group addresses and device address
        critical_section::with(|_| {
            GROUP_ADDRESS.lock()[0] = 0x1234;
            GROUP_ADDRESS.lock()[1] = 0x5678;
        });
        DEVICE_ADDRESS.set(0xABCD);
        
        // Mock flash operations
        mock_flash_erase_sector(FLASH_ADR_DEV_GRP_ADR).returns(());
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).returns(());
        
        // Call function
        compact_flash_storage();
        
        // Verify positions are reset
        assert_eq!(MESH_GROUP_ADDRESS_NEXT_POSITION.get(), INITIAL_ADDR_OFFSET as u16);
        assert_eq!(MESH_DEVICE_ADDRESS_NEXT_POSITION.get(), INITIAL_ADDR_OFFSET as u16);
        
        // Verify flash operations were called
        mock_flash_erase_sector(FLASH_ADR_DEV_GRP_ADR).assert_called(1);
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).assert_called(2); // Group + device address
    }
    
    /// Tests rf_link_add_dev_addr with valid device ID
    ///
    /// Verifies that a valid device address is correctly added to both
    /// memory and flash storage.
    #[test]
    #[mry::lock(flash_write_page, rf_link_light_event_callback)]
    fn test_add_dev_addr_valid() {
        reset_test_state();
        
        // Mock flash operations
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).returns(());
        mock_rf_link_light_event_callback(mry::Any).returns(());
        
        let dev_id = 0x1234;
        let result = add_device_address(dev_id);
        
        // Verify function returns true
        assert_eq!(result, true);
        
        // Verify device address is set in memory
        assert_eq!(DEVICE_ADDRESS.get(), dev_id);
        
        // Verify position pointers are updated
        assert_eq!(MESH_GROUP_ADDRESS_NEXT_POSITION.get(), DEVICE_ADDR_SIZE as u16);
        assert_eq!(MESH_DEVICE_ADDRESS_NEXT_POSITION.get(), DEVICE_ADDR_SIZE as u16);
        
        // Verify flash write was called
        mock_flash_write_page(mry::Any, DEVICE_ADDR_SIZE, mry::Any).assert_called(1);
        // Verify callback was called
        mock_rf_link_light_event_callback(EVENT_DEVICE_ADDR_CHANGED).assert_called(1);
    }
    
    /// Tests rf_link_add_dev_addr with invalid device ID (zero)
    ///
    /// Verifies that zero device addresses are rejected.
    #[test]
    fn test_add_dev_addr_zero() {
        reset_test_state();
        
        let result = add_device_address(0);
        
        // Verify function returns false
        assert_eq!(result, false);
        
        // Verify device address remains unchanged
        assert_eq!(DEVICE_ADDRESS.get(), 0);
    }
    
    /// Tests rf_link_add_dev_addr with invalid device ID (mask violation)
    ///
    /// Verifies that device addresses violating the device address mask are rejected.
    #[test]
    fn test_add_dev_addr_mask_violation() {
        reset_test_state();
        
        // Device ID that violates DEVICE_ADDR_MASK_DEFAULT
        let invalid_dev_id = 0x8000; // This should violate the mask
        let result = add_device_address(invalid_dev_id);
        
        // Verify function returns false
        assert_eq!(result, false);
        
        // Verify device address remains unchanged
        assert_eq!(DEVICE_ADDRESS.get(), 0);
    }
    
    /// Tests rf_link_add_dev_addr with same device ID
    ///
    /// Verifies that setting the same device address is rejected.
    #[test]
    fn test_add_dev_addr_same_id() {
        reset_test_state();
        
        let dev_id = 0x1234;
        DEVICE_ADDRESS.set(dev_id);
        
        let result = add_device_address(dev_id);
        
        // Verify function returns false
        assert_eq!(result, false);
    }
    
    /// Tests rf_link_add_dev_addr replacing existing address
    ///
    /// Verifies that replacing an existing device address correctly
    /// clears the old address in flash.
    #[test]
    #[mry::lock(flash_write_page, rf_link_light_event_callback)]
    fn test_add_dev_addr_replace_existing() {
        reset_test_state();
        
        // Mock flash operations
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).returns(());
        mock_rf_link_light_event_callback(mry::Any).returns(());
        
        // Set up existing address
        DEVICE_ADDRESS.set(0x1111);
        MESH_GROUP_ADDRESS_NEXT_POSITION.set(10);
        MESH_DEVICE_ADDRESS_NEXT_POSITION.set(5);
        
        let new_dev_id = 0x2222;
        let result = add_device_address(new_dev_id);
        
        // Verify function returns true
        assert_eq!(result, true);
        
        // Verify new device address is set
        assert_eq!(DEVICE_ADDRESS.get(), new_dev_id);
        
        // Verify position pointers are updated
        assert_eq!(MESH_GROUP_ADDRESS_NEXT_POSITION.get(), 12); // 10 + DEVICE_ADDR_SIZE
        assert_eq!(MESH_DEVICE_ADDRESS_NEXT_POSITION.get(), 12);
        
        // Verify flash operations: clear old + write new
        mock_flash_write_page(mry::Any, DEVICE_ADDR_SIZE, mry::Any).assert_called(2);
        // Verify callback was called
        mock_rf_link_light_event_callback(EVENT_DEVICE_ADDR_CHANGED).assert_called(1);
    }
    
    /// Tests rf_link_del_group with no groups stored
    ///
    /// Verifies that the function correctly handles the case where
    /// no group addresses are stored.
    #[test]
    fn test_del_group_no_groups() {
        reset_test_state();
        
        let result = remove_group(0x1234);
        
        // Verify function returns false
        assert_eq!(result, false);
    }
    
    /// Tests rf_link_del_group deleting all groups
    ///
    /// Verifies that GROUP_DELETE_ALL correctly removes all group addresses
    /// from both memory and flash.
    #[test]
    #[mry::lock(flash_write_page, flash_read_page)]
    fn test_del_group_delete_all() {
        reset_test_state();
        
        // Mock flash operations
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).returns(());
        // Mock flash read to return group addresses (simulate valid group data in flash)
        mock_flash_read_page(mry::Any, mry::Any, mry::Any).returns_with(|_addr, _len, buf: SendWrapper<*mut u8>| {
            unsafe {
                // Write a group address pattern (high bit set to distinguish from device addresses)
                let group_bytes = 0x8001u16.to_le_bytes();
                core::ptr::copy_nonoverlapping(group_bytes.as_ptr(), *buf, 2);
            }
        });
        
        // Set up some group addresses
        critical_section::with(|_| {
            GROUP_ADDRESS.lock()[0] = 0x8001;
            GROUP_ADDRESS.lock()[1] = 0x8002;
            GROUP_ADDRESS.lock()[2] = 0x8003;
        });
        MESH_GROUP_ADDRESS_NEXT_POSITION.set(6); // 3 groups * 2 bytes each
        
        let result = remove_group(GROUP_DELETE_ALL);
        
        // Verify function returns true (flash operations find valid group addresses)
        assert_eq!(result, true);
        
        // Verify all groups are cleared in memory
        critical_section::with(|_| {
            for i in 0..3 {
                assert_eq!(GROUP_ADDRESS.lock()[i], 0);
            }
        });
    }
    
    /// Tests rf_link_del_group deleting specific group
    ///
    /// Verifies that a specific group address is correctly removed
    /// from both memory and flash.
    #[test]
    #[mry::lock(flash_write_page, flash_read_page)]
    fn test_del_group_specific() {
        reset_test_state();
        
        // Mock flash operations
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).returns(());
        // Mock flash read to return the target group address we want to delete
        mock_flash_read_page(mry::Any, mry::Any, mry::Any).returns_with(|_addr, _len, buf: SendWrapper<*mut u8>| {
            unsafe {
                // Write the target group address we're trying to delete
                let group_bytes = 0x8002u16.to_le_bytes();
                core::ptr::copy_nonoverlapping(group_bytes.as_ptr(), *buf, 2);
            }
        });
        
        // Set up group addresses
        critical_section::with(|_| {
            GROUP_ADDRESS.lock()[0] = 0x8001;
            GROUP_ADDRESS.lock()[1] = 0x8002;
            GROUP_ADDRESS.lock()[2] = 0x8003;
        });
        MESH_GROUP_ADDRESS_NEXT_POSITION.set(6);
        
        let result = remove_group(0x8002);
        
        // Verify function returns true (found and deleted the target group)
        assert_eq!(result, true);
        
        // Verify specific group is cleared (the one that matches)
        critical_section::with(|_| {
            // The function clears the first match it finds
            assert_eq!(GROUP_ADDRESS.lock()[1], 0);
        });
    }
    
    /// Tests rf_link_add_group with valid group ID
    ///
    /// Verifies that a valid group address is correctly added to both
    /// memory and flash storage.
    #[test]
    #[mry::lock(flash_write_page)]
    fn test_add_group_valid() {
        reset_test_state();
        
        // Mock flash operations
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).returns(());
        
        let group_id = 0x8001; // Valid group ID that passes validation
        let result = add_group(group_id);
        
        // Verify function returns true
        assert_eq!(result, true);
        
        // Verify group is added to memory
        critical_section::with(|_| {
            assert_eq!(GROUP_ADDRESS.lock()[0], group_id);
        });
        
        // Verify position pointer is updated
        assert_eq!(MESH_GROUP_ADDRESS_NEXT_POSITION.get(), DEVICE_ADDR_SIZE as u16);
        
        // Verify flash write was called
        mock_flash_write_page(mry::Any, DEVICE_ADDR_SIZE, mry::Any).assert_called(1);
    }
    
    /// Tests rf_link_add_group with invalid group ID
    ///
    /// Verifies that invalid group addresses (outside valid range) are rejected.
    #[test]
    fn test_add_group_invalid() {
        reset_test_state();
        
        // Invalid group ID (outside valid range)
        let invalid_group_id = 0x7FFF;
        let result = add_group(invalid_group_id);
        
        // Verify function returns false
        assert_eq!(result, false);
        
        // Verify no group is added
        critical_section::with(|_| {
            assert_eq!(GROUP_ADDRESS.lock()[0], 0);
        });
    }
    
    /// Tests rf_link_add_group with duplicate group ID
    ///
    /// Verifies that duplicate group addresses are rejected.
    #[test]
    #[mry::lock(flash_write_page)]
    fn test_add_group_duplicate() {
        reset_test_state();
        
        // Mock flash write for initial insertion
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).returns(());

        let group_id = 0x8001; // Valid group ID that should succeed initially

        // Add group first time
        assert_eq!(add_group(group_id), true);
        mock_flash_write_page(mry::Any, DEVICE_ADDR_SIZE, mry::Any).assert_called(1);
        
        // Try to add same group again
        let result = add_group(group_id);
        
        // Verify function returns false and no additional flash write occurs
        assert_eq!(result, false);
        mock_flash_write_page(mry::Any, DEVICE_ADDR_SIZE, mry::Any).assert_called(1);
    }
    
    /// Tests rf_link_add_group with full group slots using rotation policy
    ///
    /// Verifies that when all group slots are full, the oldest group
    /// is replaced using the rotation policy.
    #[test]
    #[mry::lock(flash_write_page, remove_group)]
    fn test_add_group_rotation_policy() {
        reset_test_state();
        
        // Mock flash operations
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).returns(());
        mock_remove_group(mry::Any).returns(true);
        
        // Fill all group slots with valid group IDs
        for i in 0..crate::sdk::light::MAX_GROUP_COUNT as u16 {
            critical_section::with(|_| {
                GROUP_ADDRESS.lock()[i as usize] = 0x8001 + i; // Valid group IDs
            });
        }
        MESH_GROUP_ADDRESS_NEXT_POSITION.set(10); // Non-zero to avoid first-group path
        
        let new_group_id = 0x8010; // Valid group ID
        let result = add_group(new_group_id);
        
        // Verify function returns true
        assert_eq!(result, true);
        
        // Verify oldest group is replaced (first one due to rotation)
        critical_section::with(|_| {
            assert_eq!(GROUP_ADDRESS.lock()[0], new_group_id);
        });
    }
    
    /// Tests rf_link_add_group with empty slot available
    ///
    /// Verifies that when empty slots are available, new groups
    /// are added to the first available slot.
    #[test]
    #[mry::lock(flash_write_page)]
    fn test_add_group_empty_slot() {
        reset_test_state();
        
        // Mock flash operations
        mock_flash_write_page(mry::Any, mry::Any, mry::Any).returns(());
        
        // Set up some existing groups with empty slots
        critical_section::with(|_| {
            GROUP_ADDRESS.lock()[0] = 0x8001; // Valid group ID
            GROUP_ADDRESS.lock()[1] = 0; // Empty slot
            GROUP_ADDRESS.lock()[2] = 0x8002; // Valid group ID
        });
        MESH_GROUP_ADDRESS_NEXT_POSITION.set(4); // Non-zero to avoid first-group path
        
        let new_group_id = 0x8003; // Valid group ID
        let result = add_group(new_group_id);
        
        // Verify function returns true
        assert_eq!(result, true);
        
        // Verify group is added to first empty slot
        critical_section::with(|_| {
            assert_eq!(GROUP_ADDRESS.lock()[1], new_group_id);
        });
        
        // Verify flash write was called
        mock_flash_write_page(mry::Any, DEVICE_ADDR_SIZE, mry::Any).assert_called(1);
    }
}

