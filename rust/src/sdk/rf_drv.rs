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
use crate::sdk::drivers::flash::{flash_erase_sector, flash_write_page};
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

/// Sets the mesh name in advertisement packets
/// 
/// The function locates the position of the Complete Local Name field (type 9)
/// in the advertisement packet, or creates it if not found, and updates it with
/// the provided name.
///
/// # Arguments
/// * `name` - The mesh name to be set in the advertisement packet
pub fn rf_link_slave_set_adv_mesh_name(name: &[u8])
{
    let mut data_offset = 0;
    let mut type_offset;
    let mut adv_type_offset;

    let mut pkt_adv = PKT_ADV.lock();

    if pkt_adv.head().rf_len as i8 - (BLE_HEADER_LEN as i8) < 1 {
        // Empty advertisement data section, initialize new structure
        adv_type_offset = 1;
        data_offset = 2;
        type_offset = 0;
    } else {
        // Search for existing Complete Local Name field (type 9)
        type_offset = 0;
        let mut found = false;
        
        loop {
            adv_type_offset = type_offset + 1;
            
            // Found existing Complete Local Name field
            if pkt_adv.adv_ind_module().data[type_offset + 1] == ADV_TYPE_MESH_NAME {
                data_offset = type_offset + 2;
                found = true;
                break;
            }
            
            // Skip to next AD structure
            type_offset = pkt_adv.adv_ind_module().data[type_offset] as usize + 1 + type_offset;
            
            // End of advertisement data reached
            if type_offset as i8 >= pkt_adv.head().rf_len as i8 - (BLE_HEADER_LEN as i8) {
                break;
            }
        }
        
        if !found {
            // No Complete Local Name field found, append at current position
            data_offset = type_offset + 2;
            adv_type_offset = type_offset + 1;
        }
    }

    // Copy mesh name data to advertisement packet
    pkt_adv.adv_ind_module_mut().data[data_offset..data_offset + name.len()].copy_from_slice(name);

    // Set length and type fields
    pkt_adv.adv_ind_module_mut().data[type_offset] = name.len() as u8 + 1;
    pkt_adv.adv_ind_module_mut().data[adv_type_offset] = ADV_TYPE_MESH_NAME;
    
    // Update packet length fields
    pkt_adv.head_mut().dma_len = (type_offset + name.len() + 2 + DMA_HEADER_EXTRA as usize) as u32;
    pkt_adv.head_mut().rf_len = (type_offset + name.len() + 2 + BLE_HEADER_LEN as usize) as u8;
}

/// Sets manufacturer-specific data in advertisement packets
/// 
/// The function locates the position for manufacturer-specific data (type 0xFF)
/// in the advertisement packet, preserving other fields, and updates it with
/// the provided data.
///
/// # Arguments
/// * `data` - The manufacturer-specific data to be set in the advertisement packet
pub fn rf_link_slave_set_adv_private_data(data: &[u8])
{
    let mut dest_offset: usize = 0;
    let mut type_offset: usize = 0;
    let mut adv_type_offset: usize = 0;

    let mut pkt_adv = PKT_ADV.lock();

    let rf_len = pkt_adv.head().rf_len;
    if rf_len as i8 - (BLE_HEADER_LEN as i8) < 1 {
        // Empty advertisement data section, initialize new structure
        adv_type_offset = 1;
        dest_offset = 2;
        type_offset = 0;
    } else {
        // Copy existing data while searching for manufacturer data
        dest_offset = 0;
        type_offset = 0;
        
        loop {
            let current_len = pkt_adv.adv_ind_module().data[type_offset] + 1;
            let adv_data = pkt_adv.adv_ind_module().data.clone();
            
            // Copy current AD structure to destination
            pkt_adv.adv_ind_module_mut().data[dest_offset..dest_offset + current_len as usize]
                .copy_from_slice(&adv_data[type_offset..type_offset + current_len as usize]);
            
            // Check if this is manufacturer-specific data field and has expected length
            if pkt_adv.adv_ind_module().data[type_offset + 1] == ADV_TYPE_MANUFACTURER {
                if current_len == 10 {
                    break;
                }
            } else {
                // Move destination offset for next AD structure
                dest_offset = dest_offset + current_len as usize;
            }
            
            // Move to next AD structure
            type_offset = type_offset + current_len as usize;
            
            // End of advertisement data reached
            if type_offset as i8 >= rf_len as i8 - (BLE_HEADER_LEN as i8) {
                break;
            }
        }
        
        dest_offset = dest_offset + 2;
        adv_type_offset = dest_offset - 1;
    }
    
    // Copy manufacturer data to advertisement packet
    pkt_adv.adv_ind_module_mut().data[dest_offset..dest_offset + data.len()].copy_from_slice(data);

    // Set length and type fields
    pkt_adv.adv_ind_module_mut().data[type_offset] = data.len() as u8 + 1;
    pkt_adv.adv_ind_module_mut().data[adv_type_offset] = ADV_TYPE_MANUFACTURER;
    
    // Update total length
    let total_len = data.len() + 2 + dest_offset;
    pkt_adv.head_mut().dma_len = total_len as u32 + DMA_HEADER_EXTRA;
    pkt_adv.head_mut().rf_len = total_len as u8 + BLE_HEADER_LEN;
}

/// Sets the UUID data in advertisement packets
/// 
/// This function inserts UUID data at a specific position in the advertisement packet,
/// preserving the structure of other data. It handles both initial and subsequent updates.
///
/// # Arguments
/// * `uuid_data` - The UUID data to be set in the advertisement packet
pub fn rf_link_slave_set_adv_uuid_data(uuid_data: &[u8])
{
    let mut pkt_adv = PKT_ADV.lock();
    let rf_len = pkt_adv.head().rf_len as usize;
    
    // Check if new UUID data would exceed maximum advertisement data length
    if uuid_data.len() as i8 <= MAX_ADV_DATA_LEN as i8 - (rf_len as i8) {
        let mut tmp_data = [0u8; 31]; // Temporary buffer for data rearrangement
        
        if SET_UUID_FLAG.get() == false {
            // First time setting UUID - insert at position 3 and shift existing data
            let payload_len = rf_len - 9;
            
            // Save existing payload
            tmp_data[0..payload_len].copy_from_slice(&pkt_adv.adv_ind_module().data[3..3 + payload_len]);

            // Insert UUID data at position 3
            pkt_adv.adv_ind_module_mut().data[3..3 + uuid_data.len()].copy_from_slice(uuid_data);
            
            // Restore saved payload after UUID
            pkt_adv.adv_ind_module_mut().data[3 + uuid_data.len()..3 + uuid_data.len() + payload_len]
                .copy_from_slice(&tmp_data[0..payload_len]);

            SET_UUID_FLAG.set(true);
            
            // Update packet length fields
            pkt_adv.head_mut().rf_len += uuid_data.len() as u8;
            pkt_adv.head_mut().dma_len += uuid_data.len() as u32;
        } else {
            // UUID already set - replace existing UUID
            let uuid_section_len = uuid_data.len() + 3;
            let remaining_len = (rf_len - BLE_HEADER_LEN as usize) - uuid_section_len;
            
            // Save beginning of the advertisement data including UUID headers
            tmp_data[0..uuid_section_len].copy_from_slice(&pkt_adv.adv_ind_module().data[0..uuid_section_len]);

            // Replace UUID data at position 3
            pkt_adv.adv_ind_module_mut().data[3..3 + uuid_data.len()].copy_from_slice(uuid_data);
            
            // Restore remaining data after UUID
            if remaining_len > 0 {
                pkt_adv.adv_ind_module_mut().data[3 + uuid_data.len()..3 + uuid_data.len() + remaining_len]
                    .copy_from_slice(&tmp_data[0..remaining_len]);
            }
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

/// Cleans up the flash storage for device/group addresses when needed
///
/// If the storage pointers have reached their limit, this function will erase
/// the flash sector and rewrite the current group addresses and device address
/// at the beginning of the sector, then reset the pointers.
pub fn dev_grp_flash_clean()
{
    // Check if either address pointer has reached maximum allowed value
    if MAX_FLASH_ADDR_OFFSET < DEV_GRP_NEXT_POS.get() as usize || MAX_FLASH_ADDR_OFFSET < DEV_ADDRESS_NEXT_POS.get() as usize {
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
        DEV_ADDRESS_NEXT_POS.set(INITIAL_ADDR_OFFSET as u16);
        DEV_GRP_NEXT_POS.set(INITIAL_ADDR_OFFSET as u16);
    }
}

/// Adds a new device address to the mesh network
///
/// This function validates the device address according to the device address mask,
/// stores it in flash memory, and updates the current device address.
///
/// # Arguments
/// * `dev_id` - The device address to add
///
/// # Returns
/// * `true` if the device address was successfully added, `false` otherwise
pub fn rf_link_add_dev_addr(dev_id: u16) -> bool
{
    // Validate the device ID
    let mut result = false;
    
    // Device ID must be non-zero and conform to the device address mask
    // Also must be different from the current device address
    if dev_id != 0 && 
       dev_id & !DEVICE_ADDR_MASK_DEFAULT == 0 && 
       DEVICE_ADDRESS.get() != dev_id {
        
        // Clean up flash if needed
        dev_grp_flash_clean();
        
        let flash_pos = DEV_GRP_NEXT_POS.get();
        
        // If flash has been previously used, clear previous address
        if flash_pos != 0 && DEV_ADDRESS_NEXT_POS.get() != 0 {
            // Write zero to previous address location 
            let zero_data = 0u16;
            flash_write_page(
                FLASH_ADR_DEV_GRP_ADR - 2 + DEV_ADDRESS_NEXT_POS.get() as u32, 
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
        DEV_GRP_NEXT_POS.set((flash_pos as usize + DEVICE_ADDR_SIZE as usize) as u16);
        DEV_ADDRESS_NEXT_POS.set(DEV_GRP_NEXT_POS.get());
        
        // Notify application of address change
        rf_link_light_event_callback(EVENT_DEVICE_ADDR_CHANGED);
        
        result = true;
    }
    
    return result;
}

/// Removes a group address from the mesh network
///
/// This function removes a specific group address, or all group addresses if GROUP_DELETE_ALL
/// is specified, from both memory and flash storage.
///
/// # Arguments
/// * `group_id` - The group address to remove, or GROUP_DELETE_ALL to remove all groups
///
/// # Returns
/// * `true` if any group address was successfully removed, `false` otherwise
pub fn rf_link_del_group(group_id: u16) -> bool
{
    let mut grp_next_pos: i16 = DEV_GRP_NEXT_POS.get().try_into().unwrap();
    let mut group_index = 0;
    let mut result = false;
    let mut delete_mode = 0;
    
    // Only proceed if we have any group addresses stored
    if grp_next_pos != 0 {
        // First, update in-memory group addresses
        let mut current_group: usize = 0;
        let mut found_any = 0;
        let mut should_break = false;
        
        // Check each in-memory group address (max 8 groups)
        while current_group != MAX_GROUP_COUNT as usize {
            delete_mode = found_any;
            
            // Handle special case: delete all groups
            if group_id == GROUP_DELETE_ALL {
                GROUP_ADDRESS.lock()[group_index] = 0;
                delete_mode = 1;
                current_group += 1;
                group_index += 1;
                found_any = 1;
                
                if current_group == MAX_GROUP_COUNT as usize {
                    should_break = true;
                    break;
                }
                
                delete_mode = found_any;
            }
            
            if should_break {
                break;
            }
            
            // Check for specific group match
            if GROUP_ADDRESS.lock()[group_index] == group_id {
                GROUP_ADDRESS.lock()[current_group] = 0;
                delete_mode = 2;  // Found specific group
                break;
            }
            
            current_group += 1;
            group_index += 1;
            found_any = delete_mode;
        }
        
        // Now update flash storage
        let zero_short = 0u16;
        let mut groups_deleted = 0;
        result = false;
        
        // Iterate through the flash storage in reverse order (newest to oldest)
        while grp_next_pos >= 0 {
            grp_next_pos -= 2;  // Move to previous group address
            
            let p_group_address = (FLASH_ADR_DEV_GRP_ADR + grp_next_pos as u32) as *const u16;
            unsafe {
                // Only process entries that are group addresses (not device addresses)
                if *p_group_address & !DEVICE_ADDR_MASK_DEFAULT != 0 {
                    // Delete all groups mode
                    if delete_mode == 1 {
                        flash_write_page(p_group_address as u32, DEVICE_ADDR_SIZE, addr_of!(zero_short) as *const u8);
                        result = true;
                        groups_deleted += 1;
                        
                        // Exit after deleting 8 groups
                        if groups_deleted > 7 {
                            return true;
                        }
                    } 
                    // Delete specific group mode
                    else if *p_group_address == group_id && delete_mode != 0 {
                        flash_write_page(p_group_address as u32, DEVICE_ADDR_SIZE, addr_of!(zero_short) as *const u8);
                        return true;
                    }
                }
            }
        }
    }
    
    return result;
}

/// Adds a new group address to the mesh network
///
/// This function validates and adds a group address to both memory and flash storage.
/// If all group slots are filled, it will apply a rotation policy to replace the oldest group.
///
/// # Arguments
/// * `group_id` - The group address to add
///
/// # Returns
/// * `true` if the group address was successfully added, `false` otherwise
pub fn rf_link_add_group(group_id: u16) -> bool
{
    // Track the oldest group position for rotation policy
    static OLDEST_POS: AtomicUsize = AtomicUsize::new(0xffffffff);
    
    // Validate group ID: must be in valid group address range
    if (group_id + 0x8000) < 0x7fff {
        // Clean up flash if needed
        dev_grp_flash_clean();
        
        // If this is the first group, add to first position
        if DEV_GRP_NEXT_POS.get() == 0 {
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
                        DEV_GRP_NEXT_POS.get() as u32 + FLASH_ADR_DEV_GRP_ADR, 
                        DEVICE_ADDR_SIZE, 
                        addr_of!(group_id) as *const u8
                    );
                    
                    // Update position pointer
                    DEV_GRP_NEXT_POS.set((DEV_GRP_NEXT_POS.get() as usize + DEVICE_ADDR_SIZE as usize) as u16);
                    return true;
                }
            }
            
            // No empty slot found, replace oldest group using rotation policy
            if OLDEST_POS.load(Ordering::Relaxed) == 0xffffffff {
                OLDEST_POS.store(0, Ordering::Relaxed);
            }
            
            // Remove the oldest group
            let oldest_index = OLDEST_POS.load(Ordering::Relaxed);
            rf_link_del_group(GROUP_ADDRESS.lock()[oldest_index]);
            
            // Add new group in its place
            GROUP_ADDRESS.lock()[oldest_index] = group_id;
            
            // Advance oldest position for next replacement
            OLDEST_POS.store((oldest_index + 1) % MAX_GROUP_COUNT as usize, Ordering::Relaxed);
        }
        
        // Write group address to flash
        flash_write_page(
            DEV_GRP_NEXT_POS.get() as u32 + FLASH_ADR_DEV_GRP_ADR, 
            DEVICE_ADDR_SIZE, 
            addr_of!(group_id) as *const u8
        );
        
        // Update position pointer
        DEV_GRP_NEXT_POS.set((DEV_GRP_NEXT_POS.get() as usize + DEVICE_ADDR_SIZE as usize) as u16);
        return true;
    }
    
    return false;
}

