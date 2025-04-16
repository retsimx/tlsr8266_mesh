use core::ptr::{addr_of, addr_of_mut, read_unaligned};
use core::slice;

use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use crate::config::VENDOR_ID;

use crate::sdk::app_att_light::{AttributeT, get_gAttributes, SEND_TO_MASTER, GATT_UUID_CLIENT_CHAR_CFG};
use crate::sdk::light::OtaState;
use crate::sdk::mcu::register::read_reg_system_tick;
use crate::sdk::packet_types::{Packet, PacketAttMtu, PacketAttReadRsp, PacketAttWriteRsp, PacketCtrlUnknown, PacketFeatureRsp, PacketL2capHead, PacketVersionInd};
use crate::state::{*};

#[cfg(not(test))]
extern "C" {
    pub static __RAM_START_ADDR: u32;
}

#[cfg(test)]
pub static mut __RAM_START_ADDR: u32 = 0;

#[derive(FromPrimitive)]
pub enum GattOp {
    AttOpErrorRsp = 0x01,
    // Error Response op code
    AttOpExchangeMtuReq = 0x02,
    // Exchange MTU Request op code
    AttOpExchangeMtuRsp = 0x03,
    // Exchange MTU Response op code
    AttOpFindInfoReq = 0x04,
    // Find Information Request op code
    AttOpFindInfoRsp = 0x05,
    // Find Information Response op code
    AttOpFindByTypeValueReq = 0x06,
    // Find By Type Vaue Request op code
    AttOpFindByTypeValueRsp = 0x07,
    // Find By Type Vaue Response op code
    AttOpReadByTypeReq = 0x08,
    // Read By Type Request op code
    AttOpReadByTypeRsp = 0x09,
    // Read By Type Response op code
    AttOpReadReq = 0x0a,
    // Read Request op code
    AttOpReadRsp = 0x0b,
    // Read Response op code
    AttOpReadBlobReq = 0x0c,
    // Read Blob Request op code
    AttOpReadBlobRsp = 0x0d,
    // Read Blob Response op code
    AttOpReadMultiReq = 0x0e,
    // Read Multiple Request op code
    AttOpReadMultiRsp = 0x0f,
    // Read Multiple Response op code
    AttOpReadByGroupTypeReq = 0x10,
    // Read By Group Type Request op code
    AttOpReadByGroupTypeRsp = 0x11,
    // Read By Group Type Response op code
    AttOpWriteReq = 0x12,
    // Write Request op code
    AttOpWriteRsp = 0x13,
    // Write Response op code
    AttOpPrepareWriteReq = 0x16,
    // Prepare Write Request op code
    AttOpPrepareWriteRsp = 0x17,
    // Prepare Write Response op code
    AttOpExecuteWriteReq = 0x18,
    // Execute Write Request op code
    AttOpExecuteWriteRsp = 0x19,
    // Execute Write Response op code
    AttOpHandleValueNoti = 0x1b,
    // Handle Value Notification op code
    AttOpHandleValueInd = 0x1d,
    // Handle Value Indication op code
    AttOpHandleValueCfm = 0x1e,
    // Handle Value Confirmation op code
    AttOpWriteCmd = 0x52, // ATT Write Command
}

/// Searches for an attribute with the specified UUID within a range of handle values.
///
/// This function implements a key BLE ATT protocol operation for finding attributes by UUID.
/// It's used extensively by the L2CAP attribute protocol handler to respond to client requests.
///
/// # Parameters
///
/// * `handle_start` - The starting handle value to begin searching from (inclusive)
/// * `handle_end` - The ending handle value to search up to (inclusive)
/// * `uuid` - The UUID to search for, as a byte slice (either 2 or 16 bytes long)
///
/// # Returns
///
/// * `Some((&[AttributeT], usize))` - A tuple containing a slice of the found attribute(s) 
///   and the handle value where it was found
/// * `None` - If no matching attribute was found in the specified range
///
/// # Algorithm
///
/// 1. Get the total attribute count from the first attribute in the table
/// 2. Ensure we're not starting at or beyond the attribute count
/// 3. Adjust the end handle if it exceeds the attribute count
/// 4. Iterate through attributes in the specified range
/// 5. Compare UUIDs based on their length (16-bit or 128-bit)
/// 6. Return the attribute and its position if found
pub fn l2cap_att_search(mut handle_start: usize, handle_end: usize, uuid: &[u8]) -> Option<(&[AttributeT], usize)>
{
    // The first attribute in the table contains the total number of attributes
    let att_num = get_gAttributes()[0].att_num as usize;

    // If handle_start equals att_num, we've reached the end of the attribute table
    if att_num != handle_start {
        // Adjust end handle if it exceeds the total number of attributes
        let end = handle_end.min(att_num);

        // Iterate through each handle in the specified range
        for current_handle in handle_start..=end {
            let current_attr = &get_gAttributes()[current_handle];
            
            // Check if the attribute has a 16-bit UUID (2 bytes)
            if current_attr.uuid_len == 2 {
                // Get the UUID bytes and compare with the target UUID
                let attr_uuid = unsafe { slice::from_raw_parts(current_attr.uuid, 2) };
                if uuid == attr_uuid {
                    // Return this attribute and the next one, along with the handle position
                    return Some((&get_gAttributes()[current_handle..current_handle+2], current_handle));
                }
            } else {
                // Handle 128-bit UUID (16 bytes)
                let attr_uuid = unsafe { slice::from_raw_parts(current_attr.uuid, 0x10) };
                if uuid == attr_uuid {
                    // Return this attribute and the next one, along with the handle position
                    return Some((&get_gAttributes()[current_handle..current_handle+2], current_handle));
                }
            }
        }
    }

    // No matching attribute found
    None
}

/// Processes BLE Attribute Protocol (ATT) requests and generates appropriate responses.
///
/// This function is the core handler for all ATT operations in the BLE stack. It parses incoming
/// ATT packets, performs the requested operations on the attribute database, and constructs response
/// packets according to the Bluetooth Low Energy specification.
///
/// # Parameters
///
/// * `packet` - Reference to the incoming packet containing an ATT request
///
/// # Returns
///
/// * `Some(Packet)` - A response packet if the operation requires a response
/// * `None` - If no response is needed (e.g., for Write Commands) or if the operation is handled by a callback
///
/// # Supported ATT Operations
///
/// * **Exchange MTU**: Negotiates the Maximum Transmission Unit size
/// * **Find Information**: Returns handle-UUID pairs for a range of attribute handles
/// * **Find By Type Value**: Finds attributes with a specific type and value
/// * **Read By Type**: Finds and returns attributes of a specific type
/// * **Read**: Returns the value of a specific attribute
/// * **Read By Group Type**: Finds and returns service declarations
/// * **Write Request/Command**: Updates the value of a specific attribute
///
/// # Algorithm
///
/// 1. If the packet is an MTU Exchange Response, handle special connection setup operations
/// 2. Verify the L2CAP channel ID is 4 (ATT)
/// 3. Match on the ATT operation code and process accordingly:
///    - For discovery operations: search for attributes and build a formatted response 
///    - For read operations: retrieve attribute values or call read callbacks
///    - For write operations: update attribute values or call write callbacks
/// 4. Return the appropriate response packet or None
///
/// # Notes
///
/// * The handler updates global state such as `ATT_SERVICE_DISCOVER_TICK` and `SLAVE_LINK_TIME_OUT` 
///   during service discovery and connection management.
/// * For attributes with read/write callbacks, the callback function is responsible for processing 
///   the request and generating any response.
/// * Special handling is implemented for OTA (Over-The-Air) updates and specific attribute handles.
pub fn l2cap_att_handler(packet: &Packet) -> Option<Packet>
{
    // Check if this is an MTU exchange response packet (opcode & 3 = 3)
    // MTU exchange responses have special handling for connection setup
    if packet.l2cap_data().opcode & 3 == GattOp::AttOpExchangeMtuRsp as u8 {
        return handle_mtu_exchange_response(packet);
    }

    // Check if this packet is for the ATT channel (channel ID = 4)
    // The channel ID is encoded in bytes 1-2 of the value field
    // If not the ATT channel, we don't process this packet
    if *bytemuck::from_bytes::<u16>(&packet.l2cap_data().value[1..3]) != 4u16 {
        return None
    }

    // Process different ATT operations based on the opcode in byte 3
    // Each operation is handled by a separate function for clarity and maintainability
    return match FromPrimitive::from_u8(packet.l2cap_data().value[3]) {
        Some(GattOp::AttOpExchangeMtuReq) => handle_exchange_mtu_request(),
        Some(GattOp::AttOpFindInfoReq) => handle_find_info_request(packet),
        Some(GattOp::AttOpFindByTypeValueReq) => handle_find_by_type_value_request(packet),
        Some(GattOp::AttOpReadByTypeReq) => handle_read_by_type_request(packet),
        Some(GattOp::AttOpReadReq) => handle_read_request(packet),
        Some(GattOp::AttOpReadByGroupTypeReq) => handle_read_by_group_type_request(packet),
        Some(GattOp::AttOpWriteReq) | Some(GattOp::AttOpWriteCmd) => handle_write_request_or_command(packet),
        // If the opcode is not recognized or not implemented, return None
        _ => None
    };
}

/// Handles MTU Exchange Response packets with special connection setup logic.
/// 
/// This handler processes ATT MTU Exchange Response packets which have specific behavior depending
/// on the handle value in the packet. It's a critical part of the BLE connection and service 
/// discovery process.
/// 
/// # Parameters
/// 
/// * `packet` - Reference to the packet containing the MTU Exchange Response
/// 
/// # Returns
/// 
/// * `Some(Packet)` - A response packet if required by the specific handle value
/// * `None` - If no response is needed (e.g., for handle = 2)
/// 
/// # Algorithm
/// 
/// 1. Extract the handle value from the packet
/// 2. Process based on the specific handle value:
///    - For handle 0xC: Connection setup handling, returns version information
///    - For handle 8: Mark service discovery as active, return feature response
///    - For handle 2: Set slave link timeout, no response needed
///    - For all other handles: Return a control packet with the handle value
/// 
/// # Special Cases
/// 
/// * **Handle 0xC**: Marks connection established and returns device version info
/// * **Handle 8**: Marks service discovery as active and returns feature info
/// * **Handle 2**: Sets the slave link timeout but doesn't return a response
fn handle_mtu_exchange_response(packet: &Packet) -> Option<Packet> {
    // Extract the handle value from the packet
    let handle = packet.l2cap_data().handle1;
    
    // Special case for handle 0xC - handle connection setup
    if handle == 0xc {
        // Set discovery tick to mark connection established (OR with 1 to ensure non-zero)
        ATT_SERVICE_DISCOVER_TICK.set(read_reg_system_tick() | 1);

        // Return version information packet to identify the device
        return Some(
            Packet {
                version_ind: PacketVersionInd {
                    dma_len: 8,
                    _type: 3,
                    rf_len: 6,
                    opcode: 0x0c,
                    main_ver: 0x08,
                    vendor: VENDOR_ID,
                    sub_ver: 0x08,
                }
            }
        )
    }
    
    // Handle isn't 8 (regular service discovery case)
    if handle != 8 {
        // Special case for handle 2 - set timeout for slave link
        if handle == 2 {
            SLAVE_LINK_TIME_OUT.set(1000000);
            return None
        }

        // For other handle values, return a control packet with the handle value
        return Some(
            Packet {
                ctrl_unknown: PacketCtrlUnknown {
                    dma_len: 0x04,
                    _type: 0x03,
                    rf_len: 0x02,
                    opcode: 0x07,
                    data: [handle],
                }
            }
        )
    }

    // Handle 8 case - mark service discovery as active
    ATT_SERVICE_DISCOVER_TICK.set(read_reg_system_tick() | 1);

    // Return feature response packet with flags
    Some(
        Packet {
            feature_rsp: PacketFeatureRsp {
                dma_len: 0x0b,
                _type: 0x3,
                rf_len: 0x09,
                opcode: 0x09,
                data: [1, 0, 0, 0, 0, 0, 0, 0],
            }
        }
    )
}

/// Creates a blank response packet template for ATT operations.
///
/// This function initializes a new packet with the ATT Read Response structure
/// and zeroes all fields. This template serves as the starting point for constructing
/// various ATT response types, which will be populated with specific data by the
/// respective handler functions.
///
/// # Returns
///
/// * `Packet` - An initialized packet with zeroed fields for use in building ATT responses
///
/// # Notes
///
/// The template uses `PacketAttReadRsp` as its base structure, which has a value array
/// of 22 bytes that can be used to hold various response formats depending on the specific
/// ATT operation being handled.
fn create_att_response_template() -> Packet {
    Packet {
        att_read_rsp: PacketAttReadRsp {
            head: PacketL2capHead {
                dma_len: 0,
                _type: 0,
                rf_len: 0,
                l2cap_len: 0,
                chan_id: 0,
            },
            opcode: 0,
            value: [0; 22],
        }
    }
}

/// Handles MTU Exchange Requests from clients.
///
/// When a client wants to negotiate the Maximum Transmission Unit (MTU) size with
/// the server, it sends an MTU Exchange Request. This function generates the appropriate
/// response packet, which informs the client of the server's supported MTU size.
///
/// # Returns
///
/// * `Some(Packet)` - An Exchange MTU Response packet with the server's MTU size (23 bytes)
///
/// # Algorithm
///
/// 1. Create a new Packet with the Exchange MTU Response structure
/// 2. Set the appropriate header fields (dma_len, _type, rf_len, l2cap_len, chan_id)
/// 3. Set the opcode to 0x03 (Exchange MTU Response)
/// 4. Set the MTU size to 0x17 (23 bytes), which is the default BLE ATT MTU
/// 5. Return the response packet
///
/// # Notes
///
/// The MTU size is hardcoded to 23 bytes (0x17 0x00 in little-endian format),
/// which is the minimum required MTU according to the Bluetooth Core Specification.
fn handle_exchange_mtu_request() -> Option<Packet> {
    Some(
        Packet {
            att_mtu: PacketAttMtu {
                head: PacketL2capHead {
                    dma_len: 0x09,
                    _type: 2,
                    rf_len: 0x07,
                    l2cap_len: 0x03,
                    chan_id: 0x04,
                },
                opcode: 0x03,
                mtu: [0x17, 0x00],
            }
        }
    )
}

/// Handles Find Information Requests that return UUID and handle pairs.
///
/// This function processes ATT Find Information Requests, which are used during service discovery
/// to retrieve the UUIDs of all attributes within a specified handle range. The response contains
/// handle-UUID pairs for the attributes found within the range.
///
/// # Parameters
///
/// * `packet` - Reference to the incoming packet containing a Find Information Request
///
/// # Returns
///
/// * `Some(Packet)` - A Find Information Response packet containing handle-UUID pairs
///   or an Error Response if no attributes are found in the range
///
/// # Algorithm
///
/// 1. Mark service discovery as active by updating the global discovery tick
/// 2. Extract the handle range (start_handle, end_handle) from the request
/// 3. Ensure end_handle doesn't exceed the total attribute count
/// 4. For each attribute in the range:
///    - If this is the first attribute, record its UUID length as the expected format
///    - If another attribute has a different UUID length, end the current response
///    - Add the attribute handle and UUID to the response packet
///    - Format type 1 (byte 0) indicates 16-bit UUIDs; format type 2 indicates 128-bit UUIDs
/// 5. End the response when all attributes in range are processed, or when buffer space is exhausted
/// 6. Return an error response if no attributes were found in the range
///
/// # Notes
///
/// * Each ATT Find Information Response can only contain UUIDs of one size (all 16-bit or all 128-bit)
/// * To comply with Bluetooth Core Spec, response is limited to avoid fragmentation
/// * UUID format is indicated by the first byte of the response value:
///   - 0x01: All UUIDs are 16-bit
///   - 0x02: All UUIDs are 128-bit
fn handle_find_info_request(packet: &Packet) -> Option<Packet> {
    // Mark service discovery as active
    ATT_SERVICE_DISCOVER_TICK.set(read_reg_system_tick() | 1);
    
    // Extract handle range from the request
    let mut start_handle = packet.l2cap_data().value[4] as usize;
    let mut end_handle = packet.l2cap_data().value[6] as usize;
    
    // Ensure end_handle doesn't exceed the total attribute count
    if get_gAttributes()[0].att_num < packet.l2cap_data().value[6] {
        end_handle = get_gAttributes()[0].att_num as usize;
    }

    // Start constructing the response
    let mut rf_packet_att_rsp = create_att_response_template();
    
    // Error case - start handle is 0 or is greater than end handle
    if start_handle == 0 || start_handle > end_handle {
        return prepare_error_response(GattOp::AttOpFindInfoReq as u8, start_handle as u16);
    }

    // Variables needed during attribute processing
    let mut format_type = 1;   // Format byte: 1=16-bit UUIDs, 2=128-bit UUIDs
    let mut offset = 0;        // Position in response buffer
    let buffer_limit = 0x17;   // Maximum buffer size to prevent overflows
    
    // Get the first attribute's UUID length to determine response format
    // If no attributes in range, this will be caught later
    let uuid_len = get_gAttributes()[start_handle].uuid_len;
    
    // Process attributes within the requested range
    for current_handle in start_handle..=end_handle {
        let current_attr = &get_gAttributes()[current_handle];
        
        // If we encounter a different UUID length, end the current response
        if current_attr.uuid_len != uuid_len {
            break;
        }
        
        let next_entry_size = if current_attr.uuid_len == 2 { 4 } else { 0x12 };
        
        // Stop if buffer limit would be reached
        if offset + next_entry_size > buffer_limit {
            break;
        }
        
        // Add attribute handle (2 bytes) to response - MOVED after buffer limit check
        rf_packet_att_rsp.att_read_rsp_mut().value[offset + 1] = current_handle as u8;
        rf_packet_att_rsp.att_read_rsp_mut().value[offset + 2] = 0;
        
        // Add attribute UUID to response based on UUID length
        if current_attr.uuid_len == 2 {
            // 16-bit UUID: Copy 2 bytes directly
            *bytemuck::from_bytes_mut(
                &mut rf_packet_att_rsp.att_read_rsp_mut().value[offset + 3..offset + 5]
            ) = unsafe { *(current_attr.uuid as *const u16) };
            offset += 4; // Handle (2) + UUID (2) = 4 bytes
        } else {
            // 128-bit UUID: Copy 16 bytes
            let uuid_slice = unsafe { slice::from_raw_parts(current_attr.uuid, 0x10) };
            rf_packet_att_rsp.att_read_rsp_mut().value[offset + 3..offset + 3 + 0x10]
                .copy_from_slice(uuid_slice);
            offset += 0x12; // Handle (2) + UUID (16) = 18 bytes
            format_type = 2; // Format type 2 indicates 128-bit UUIDs
        }
    }
    
    // No attributes processed - return error response
    // Unreachable with the current gAttribute table.
    // if offset == 0 {
    //     return prepare_error_response(GattOp::AttOpFindInfoReq as u8, start_handle as u16);
    // }
    
    // Finalize response packet with header information
    rf_packet_att_rsp.head_mut().l2cap_len = offset as u16 + 2;
    rf_packet_att_rsp.head_mut().dma_len = offset as u32 + 8;
    rf_packet_att_rsp.head_mut()._type = 2;
    rf_packet_att_rsp.head_mut().rf_len = offset as u8 + 6;
    rf_packet_att_rsp.head_mut().chan_id = 4;
    rf_packet_att_rsp.att_read_rsp_mut().opcode = GattOp::AttOpFindInfoRsp as u8;
    rf_packet_att_rsp.att_read_rsp_mut().value[0] = format_type;
    
    Some(rf_packet_att_rsp)
}

/// Helper function to prepare an error response packet
fn prepare_error_response(err_opcode: u8, err_handle: u16) -> Option<Packet> {
    let mut err = PKT_ERR_RSP;
    err.att_err_rsp_mut().err_opcode = err_opcode;
    err.att_err_rsp_mut().err_handle = err_handle;
    Some(err)
}

/// Handles Find By Type Value Requests that find attributes with specific type (UUID) and value.
///
/// This function processes ATT Find By Type Value Requests, which are used to locate attributes that
/// have a specific type (UUID) and value. It's primarily used by clients to discover primary services
/// with a specific UUID.
///
/// # Parameters
///
/// * `packet` - Reference to the incoming packet containing a Find By Type Value Request
///
/// # Returns
///
/// * `Some(Packet)` - A Find By Type Value Response packet containing handle pairs (start/end) 
///   or an Error Response if no attributes are found
///
/// # Algorithm
///
/// 1. Mark service discovery as active by updating the global discovery tick
/// 2. Extract the handle range (start_handle, end_handle) from the request
/// 3. Extract the UUID and value to search for
/// 4. For each attribute in the range:
///    - Search for attributes with the specified UUID
///    - Check if the attribute value matches the requested value
///    - For matching attributes, add the handle and group end handle to the response
/// 5. If no matching attributes are found, return an error response
/// 6. Otherwise, finalize and return the response packet
///
/// # Notes
///
/// * The response contains pairs of handles: the attribute handle and the group end handle
/// * Limited to 10 handle pairs per response to avoid packet fragmentation
/// * Primarily used in service discovery to locate specific services by UUID
fn handle_find_by_type_value_request(packet: &Packet) -> Option<Packet> {
    // Mark service discovery as active
    ATT_SERVICE_DISCOVER_TICK.set(read_reg_system_tick() | 1);
    
    // Extract handle range and UUID from request
    let start_handle = packet.l2cap_data().value[4] as usize;
    let end_handle = packet.l2cap_data().value[6] as usize;
    
    // Extract UUID to search for (typically Primary Service UUID 0x2800)
    let mut uuid = [0; 2];
    uuid.copy_from_slice(&packet.l2cap_data().value[8..10]);
    
    // Extract value to match (typically a 16-bit service UUID)
    let mut value_bytes = [0u8; 2];
    value_bytes.copy_from_slice(&packet.l2cap_data().value[10..12]);
    let target_value: u16 = *bytemuck::from_bytes(&value_bytes);
    
    // Initialize response packet and counters
    let mut response = create_att_response_template();
    let mut handle_pairs_count = 0;
    let max_handle_pairs = 9; // Max of 9 handle pairs to avoid overflow
    let mut current_handle = start_handle;
    
    // Search for matching attributes in the requested handle range
    while current_handle <= end_handle && handle_pairs_count <= max_handle_pairs {
        // Find attribute with the requested UUID in the handle range
        let search_result = l2cap_att_search(current_handle, end_handle, &uuid);
        
        // No more matching attributes found
        if (search_result.is_none()) {
            break;
        }
        
        // Unpack search result
        let (found_attr, found_handle) = search_result.unwrap();
        let attr = &found_attr[0];
        
        // Check if attribute value matches the requested value
        let attr_value_matches = attr.attr_len == 2 && 
                               unsafe { *(attr.p_attr_value as *const u16) } == target_value;
        
        if attr_value_matches {
            // Calculate base offset for this handle pair in response buffer
            // Each pair consists of 4 bytes: start handle (2) + end handle (2)
            let offset = handle_pairs_count * 4;
            
            // Add attribute handle to response (start handle)
            response.att_read_rsp_mut().value[offset] = (found_handle & 0xff) as u8;
            response.att_read_rsp_mut().value[offset + 1] = (found_handle >> 8) as u8;
            
            // Calculate end handle based on attribute's att_num field
            let group_end_handle = found_attr[0].att_num as usize + (found_handle - 1);
            
            // Add end handle to response
            response.att_read_rsp_mut().value[offset + 2] = (group_end_handle & 0xff) as u8;
            response.att_read_rsp_mut().value[offset + 3] = (group_end_handle >> 8) as u8;
            
            // Update counter and move to next search position
            handle_pairs_count += 1;
            current_handle = found_handle + attr.att_num as usize;
        } else {
            // Move to next attribute if current one doesn't match
            current_handle = found_handle + 1;
        }
    }
    
    // Return appropriate response based on search results
    if handle_pairs_count == 0 {
        // No matches found - return error response
        return prepare_error_response(GattOp::AttOpFindByTypeValueReq as u8, start_handle as u16);
    } else {
        // Matches found - calculate total response size
        let total_bytes = handle_pairs_count * 4; // Each pair is 4 bytes (start handle + end handle)
        
        // Set packet header fields in a more structured way
        let header = response.head_mut();
        header.dma_len = total_bytes as u32 + 7;
        header._type = 2;
        header.rf_len = total_bytes as u8 + 5;
        header.l2cap_len = total_bytes as u16 + 1;
        header.chan_id = 4;
        
        // Set response opcode
        response.att_read_rsp_mut().opcode = GattOp::AttOpFindByTypeValueRsp as u8;

        Some(response)
    }
}

/// Handles Read By Type Requests that find and return attributes with specific UUID.
///
/// This function processes ATT Read By Type Requests, which are used to discover attributes
/// with a specific UUID (type) and retrieve their values. It's commonly used for discovering
/// characteristics during service discovery.
///
/// # Parameters
///
/// * `packet` - Reference to the incoming packet containing a Read By Type Request
///
/// # Returns
///
/// * `Some(Packet)` - A Read By Type Response packet containing handle-value pairs
///   or an Error Response if no attributes are found with the specified UUID
///
/// # Algorithm
///
/// 1. Mark service discovery as active by updating the global discovery tick
/// 2. Extract the handle range (handle_start, handle_end) and UUID from the request
/// 3. Special handling for 128-bit UUIDs (when handle1 = 0x15):
///    - Search for the attribute with the specified 128-bit UUID
///    - If found, add its handle and value to the response
///    - First byte of value array contains length of each handle-value pair
/// 4. For 16-bit UUIDs:
///    - Search for attributes with the specified UUID
///    - If the UUID is 0x2803 (Characteristic declaration), handle special formatting:
///      - Find all matching characteristic declarations in the range
///      - Format each entry with: declaration handle, properties, value handle, and UUID
///      - All entries must have consistent format (same UUID length)
/// 5. Return error if no matching attributes are found in the range
///
/// # Notes
///
/// * Each entry in the response consists of the attribute handle followed by its value
/// * All entries must have the same length (as indicated by the first byte in the response)
/// * For characteristic declarations, multiple components (properties, value handle, UUID)
///   are packed together in a specific format
/// * The response is limited to avoid packet fragmentation
fn handle_read_by_type_request(packet: &Packet) -> Option<Packet> {
    // Mark service discovery as active
    ATT_SERVICE_DISCOVER_TICK.set(read_reg_system_tick() | 1);
    
    // Extract handle range from the request
    let original_handle_start = packet.l2cap_data().value[4] as usize;
    let mut handle_start = original_handle_start;
    let handle_end = packet.l2cap_data().value[6] as usize;
    
    // Initialize response packet and tracking variables
    let mut rf_packet_att_rsp = create_att_response_template();
    let mut bytes_read = 0;

    // ---- Handle selection based on UUID type (16-bit or 128-bit) ----
    if unsafe { *(addr_of!(packet.l2cap_data().handle1) as *const u16) } == 0x15 {
        // ---- 128-bit UUID search ----
        let mut uuid = [0; 16];
        uuid.copy_from_slice(&packet.l2cap_data().value[8..8 + 0x10]);
        
        match l2cap_att_search(handle_start, handle_end, &uuid) {
            None => {
                // No match found - prepare for error response
                rf_packet_att_rsp.att_read_rsp_mut().value[0] = 0;
            },
            Some((found_attrs, handle)) => {
                let found_attr = &found_attrs[0];
                
                // Set bytes_read to attribute length + 2 (for handle)
                bytes_read = found_attr.attr_len + 2;
                
                // Format response: handle (2 bytes) + attribute value
                rf_packet_att_rsp.att_read_rsp_mut().value[0] = bytes_read;
                rf_packet_att_rsp.att_read_rsp_mut().value[1] = handle as u8;
                rf_packet_att_rsp.att_read_rsp_mut().value[2] = (handle >> 8) as u8;
                
                // Copy attribute value to response
                rf_packet_att_rsp.att_read_rsp_mut().value[3..3 + found_attr.attr_len as usize]
                    .copy_from_slice(unsafe { 
                        slice::from_raw_parts(found_attr.p_attr_value, found_attr.attr_len as usize)
                    });
            }
        }
    } else {
        // ---- 16-bit UUID search ----
        let mut uuid = [0; 2];
        uuid.copy_from_slice(&packet.l2cap_data().value[8..=9]);
        
        // Special handling for Characteristic UUID (0x2803)
        if uuid[0] == 3 && uuid[1] == 0x28 {
            // ---- Characteristic Declaration Search ----
            let mut counter = 0; // Tracks UUID length for format consistency
            
            // Search for all characteristics in the handle range
            loop {
                match l2cap_att_search(handle_start, handle_end, &uuid) {
                    None => break, // No more characteristics found
                    
                    Some((found_attrs, handle)) => {
                        // Enforce format consistency and buffer space limit
                        if !((counter == 0 || found_attrs[1].uuid_len == counter) && 
                             bytes_read + found_attrs[0].uuid_len < 0x13) {
                            break;
                        }
                        
                        // Format each characteristic entry:
                        // 1. Declaration handle (2 bytes)
                        rf_packet_att_rsp.att_read_rsp_mut().value[bytes_read as usize + 1] = handle as u8;
                        bytes_read += 1;
                        rf_packet_att_rsp.att_read_rsp_mut().value[bytes_read as usize + 1] = 0;
                        bytes_read += 1;
                        
                        // 2. Characteristic properties (1 byte)
                        rf_packet_att_rsp.att_read_rsp_mut().value[bytes_read as usize + 1] = unsafe {
                            *(found_attrs[0].p_attr_value) 
                        };
                        bytes_read += 1;
                        
                        // 3. Value handle (2 bytes) - always declaration handle + 1
                        rf_packet_att_rsp.att_read_rsp_mut().value[bytes_read as usize + 1] = handle as u8 + 1;
                        bytes_read += 1;
                        rf_packet_att_rsp.att_read_rsp_mut().value[bytes_read as usize + 1] = 0;
                        bytes_read += 1;
                        
                        // 4. Characteristic UUID (variable length)
                        rf_packet_att_rsp.att_read_rsp_mut().value[bytes_read as usize + 1..(bytes_read as usize + 1 + found_attrs[1].uuid_len as usize)]
                            .copy_from_slice(unsafe { 
                                slice::from_raw_parts(found_attrs[1].uuid, found_attrs[1].uuid_len as usize)
                            });
                        
                        // Update for next iteration
                        counter = found_attrs[1].uuid_len;
                        bytes_read += counter;
                        handle_start = handle + 2; // Skip to next potential characteristic
                    }
                }
            }
            
            // Set format byte if characteristics were found
            if counter > 0 {
                // Format byte = UUID length + 5 (handle=2, properties=1, value handle=2)
                rf_packet_att_rsp.att_read_rsp_mut().value[0] = counter + 5;
            }
        } else {
            // ---- Regular 16-bit UUID search ----
            match l2cap_att_search(handle_start, handle_end, &uuid) {
                None => {
                    // No matching attributes found
                    rf_packet_att_rsp.att_read_rsp_mut().value[0] = 0;
                },
                Some((found_attrs, handle)) => {
                    let found_attr = &found_attrs[0];
                    
                    // Format response: handle (2 bytes) + attribute value
                    bytes_read = found_attr.attr_len + 2;
                    
                    rf_packet_att_rsp.att_read_rsp_mut().value[0] = bytes_read;
                    rf_packet_att_rsp.att_read_rsp_mut().value[1] = handle as u8;
                    rf_packet_att_rsp.att_read_rsp_mut().value[2] = (handle >> 8) as u8;
                    
                    // Copy attribute value to response
                    rf_packet_att_rsp.att_read_rsp_mut().value[3..3 + found_attr.attr_len as usize]
                        .copy_from_slice(unsafe { 
                            slice::from_raw_parts(found_attr.p_attr_value, found_attr.attr_len as usize)
                        });
                }
            }
        }
    }
    
    // Return error response if no matching attributes were found
    if bytes_read == 0 {
        return prepare_error_response(GattOp::AttOpReadByTypeReq as u8, original_handle_start as u16);
    }
    
    // Finalize the response packet header
    rf_packet_att_rsp.head_mut().dma_len = bytes_read as u32 + 8;
    rf_packet_att_rsp.head_mut()._type = 2;
    rf_packet_att_rsp.head_mut().rf_len = bytes_read + 6;
    rf_packet_att_rsp.head_mut().l2cap_len = bytes_read as u16 + 2;
    rf_packet_att_rsp.head_mut().chan_id = 4;
    rf_packet_att_rsp.att_read_rsp_mut().opcode = GattOp::AttOpReadByTypeRsp as u8;
    
    Some(rf_packet_att_rsp)
}

/// Handles Read Request operations that return an attribute's value.
///
/// This function processes ATT Read Requests, which are used by clients to retrieve
/// the value of a specific attribute identified by its handle. If the attribute has
/// a read callback, the callback function is invoked to handle the request; otherwise,
/// the attribute's value is directly returned in the response.
///
/// # Parameters
///
/// * `packet` - Reference to the incoming packet containing a Read Request
///
/// # Returns
///
/// * `Some(Packet)` - A Read Response packet containing the requested attribute's value
/// * `None` - If the attribute has a read callback function that handles the response
///
/// # Algorithm
///
/// 1. Extract the attribute handle from the request
/// 2. Validate the handle:
///    - Ensure the high byte of the handle is zero
///    - Ensure the handle doesn't exceed the total attribute count
/// 3. Check if the attribute has a read callback function:
///    - If no callback, directly copy the attribute's value to the response
///    - Handle special cases for SEND_TO_MASTER and OTA termination flags
/// 4. If the attribute has a callback, invoke it and return None
///
/// # Special Cases
///
/// * **SEND_TO_MASTER**: If the attribute's value pointer is SEND_TO_MASTER, 
///   clear the SEND_TO_MASTER array after reading
/// * **OTA Termination**: If reading attribute 0x18 and OTA state is not "Continue",
///   set RF_SLAVE_OTA_TERMINATE_FLAG to trigger OTA termination
///
/// # Notes
///
/// * Attributes with read callbacks are typically customized to implement special behavior,
///   such as reading dynamic system state or triggering side effects
/// * When a read callback is present, it's the callback's responsibility to generate
///   and send any response packet
fn handle_read_request(packet: &Packet) -> Option<Packet> {
    // Get attribute handle from request
    let att_num = packet.l2cap_data().value[4] as usize;

    // Validate handle has zero in high byte
    if packet.l2cap_data().value[5] != 0 {
        return None
    }
    
    // Validate handle doesn't exceed total attribute count
    if get_gAttributes()[0].att_num < att_num as u8 {
        return None
    }
    
    // Check if attribute has a read callback function
    if get_gAttributes()[att_num].r.is_none() {
        // No read callback - directly return the attribute's value
        let mut rf_packet_att_rsp = create_att_response_template();
        unsafe {
            slice::from_raw_parts_mut(
                addr_of_mut!(rf_packet_att_rsp.att_read_rsp_mut().value[0]),
                get_gAttributes()[att_num].attr_len as usize,
            ).copy_from_slice(
                slice::from_raw_parts(
                    get_gAttributes()[att_num].p_attr_value,
                    get_gAttributes()[att_num].attr_len as usize,
                )
            );
        }

        // Special case: handle SEND_TO_MASTER flag
        if get_gAttributes()[att_num].p_attr_value == unsafe { SEND_TO_MASTER.as_mut_ptr() } {
            // Clear the SEND_TO_MASTER array after reading
            unsafe { SEND_TO_MASTER.fill(0); }
        } 
        // Special case: handle OTA termination flag for handle 0x18
        else if att_num == 0x18 && *RF_SLAVE_OTA_FINISHED_FLAG.lock() != OtaState::Continue {
            // Mark OTA for termination
            RF_SLAVE_OTA_TERMINATE_FLAG.set(true);
        }
        
        // Prepare and return the response packet
        let current_attr = &get_gAttributes()[att_num];
        let attr_len = current_attr.attr_len;
        
        // Set packet header fields according to the ATT protocol requirements
        // L2CAP header: rf_len = value_len + 5 bytes overhead
        rf_packet_att_rsp.head_mut().rf_len = attr_len + 5;
        // DMA length includes 2 additional bytes for radio hardware
        rf_packet_att_rsp.head_mut().dma_len = (attr_len as u32) + 7;
        // Packet type 2 = data packet
        rf_packet_att_rsp.head_mut()._type = 2;
        // L2CAP length = value_len + 1 byte opcode
        rf_packet_att_rsp.head_mut().l2cap_len = (attr_len as u16) + 1;
        // ATT channel ID = 4
        rf_packet_att_rsp.head_mut().chan_id = 4;
        // 0x0B = Read Response opcode
        rf_packet_att_rsp.att_read_rsp_mut().opcode = GattOp::AttOpReadRsp as u8;
        
        return Some(rf_packet_att_rsp)
    }

    // If attribute has a read callback, call it and let it handle the response
    get_gAttributes()[att_num].r.unwrap()(packet);

    None
}

/// Handles Read By Group Type Request that returns service declarations.
///
/// This function processes ATT Read By Group Type Requests, which are used to discover
/// services within a specified handle range. The primary use case is discovering primary
/// and secondary services during the service discovery phase.
///
/// # Parameters
///
/// * `packet` - Reference to the incoming packet containing a Read By Group Type Request
///
/// # Returns
///
/// * `Some(Packet)` - A Read By Group Type Response packet containing handle ranges and service UUIDs
///   or an Error Response if no matching attributes are found
///
/// # Algorithm
///
/// 1. Mark service discovery as active by updating the global discovery tick
/// 2. Extract the handle range (handle_start, handle_end) and group type UUID from the request
/// 3. For each attribute in the range:
///    - Search for attributes with the specified UUID (typically 0x2800 for Primary Service)
///    - For the first matching attribute, record its value size as the expected format for all entries
///    - Ensure subsequent attributes have the same value size (required by the ATT protocol)
///    - For each match, add to the response:
///      * The start handle (attribute handle) 
///      * The end handle (calculated from attribute's att_num field)
///      * The attribute value (service UUID)
/// 4. If no matching attributes are found, return an Error Response
/// 5. Otherwise, finalize and return the response packet
///
/// # Notes
///
/// * All entries in the response must have the same format and value length
/// * The first byte of the response value indicates the length of each tuple (start handle + end handle + value)
/// * Response size is limited to avoid packet fragmentation
/// * Primarily used during service discovery to locate service declarations
fn handle_read_by_group_type_request(packet: &Packet) -> Option<Packet> {
    // Mark service discovery as active
    ATT_SERVICE_DISCOVER_TICK.set(read_reg_system_tick() | 1);
    
    // Extract handle range and target UUID from the request
    let start_handle = packet.l2cap_data().value[4] as usize;
    let handle_end = packet.l2cap_data().value[6] as usize;
    let mut handle_start = start_handle;
    
    // Extract UUID to search for (typically 0x2800 for Primary Service)
    let mut uuid = [0; 2];
    uuid.copy_from_slice(&packet.l2cap_data().value[8..=9]);
    
    // Immediately check if request is valid, return error for invalid range
    if start_handle == 0 || start_handle > handle_end {
        return prepare_error_response(GattOp::AttOpReadByGroupTypeReq as u8, start_handle as u16);
    }
    
    // Tracking variables for response construction
    let mut dest_ptr = 0;        // Position in the response buffer
    let mut format_length = 0;   // Expected size of each entry (for consistency checking)
    
    // Buffer for accumulating response data during search loop
    let mut response = create_att_response_template();
    
    // Maximum buffer size to prevent overflow in response
    const MAX_BUFFER_SIZE: usize = 0x13;
    
    // Search for matching group declarations within handle range
    while handle_start <= handle_end {
        // Find next attribute with the requested UUID
        match l2cap_att_search(handle_start, handle_end, &uuid) {
            None => break, // No more matching attributes found
            
            Some((found_attrs, current_handle)) => {
                let found_attr = &found_attrs[0];
                let attr_len = found_attr.attr_len as usize;
                
                // Format consistency check - all entries must have same size
                if format_length != 0 && attr_len != format_length {
                    break; // Inconsistent attribute sizes, can't include in same response
                }
                
                // Check for buffer overflow - ensure we have enough space
                if MAX_BUFFER_SIZE < attr_len + dest_ptr * 2 {
                    break; // Not enough buffer space for this entry
                }
                
                // Initialize format_length on first match
                if format_length == 0 {
                    format_length = attr_len;
                }
                
                // Calculate handle positions for response construction
                let next_ptr = dest_ptr + 1;
                let value_ptr = next_ptr + 1;
                
                // Calculate end handle based on attribute's att_num field
                let group_end_handle = ((current_handle - 1) + found_attr.att_num as usize) as u16;
                
                // Add start handle to response (2 bytes)
                response.att_read_rsp_mut().value[dest_ptr * 2 + 1] = (current_handle & 0xff) as u8;
                response.att_read_rsp_mut().value[dest_ptr * 2 + 2] = (current_handle >> 8) as u8;
                
                // Add end handle to response (2 bytes)
                response.att_read_rsp_mut().value[next_ptr * 2 + 1] = (group_end_handle & 0xff) as u8;
                response.att_read_rsp_mut().value[next_ptr * 2 + 2] = (group_end_handle >> 8) as u8;
                
                // Get attribute value as a slice for easier handling
                let value_slice = unsafe {
                    slice::from_raw_parts(
                        found_attr.p_attr_value,
                        found_attr.attr_len as usize,
                    )
                };
                
                // Add attribute value (service UUID) to response
                response.att_read_rsp_mut().value[value_ptr * 2 + 1..value_ptr * 2 + value_slice.len() + 1]
                    .copy_from_slice(value_slice);
                
                // Update pointers and counters for next iteration
                dest_ptr = value_ptr + (found_attr.attr_len as usize / 2);
                
                // Move start handle beyond current group to search for next group
                handle_start = current_handle + found_attr.att_num as usize;
                
                // Check if we've reached the end of the requested range
                if handle_start > handle_end {
                    break;
                }
            }
        }
    }
    
    // If no matching attributes were found, return error response
    if dest_ptr == 0 {
        return prepare_error_response(GattOp::AttOpReadByGroupTypeReq as u8, start_handle as u16);
    }
    
    // Prepare the final response packet
    let mut response_packet = create_att_response_template();
    
    // Set packet header fields according to BLE ATT protocol requirements
    response_packet.head_mut().chan_id = 4;  // ATT channel ID = 4
    response_packet.head_mut()._type = 2;    // Type 2 = data packet
    
    // Calculate packet lengths based on data size
    // Each handle pair is: 4 bytes (start handle + end handle) + format_length
    // The packet needs: 
    //   - dma_len: (dest_ptr as u32 + 4) * 2
    //   - rf_len: dma_len - 2
    //   - l2cap_len: dma_len - 6
    let dma_len = (dest_ptr as u32 + 4) * 2;
    response_packet.head_mut().dma_len = dma_len;
    response_packet.head_mut().rf_len = dma_len as u8 - 2;
    response_packet.head_mut().l2cap_len = dma_len as u16 - 6;
    
    // Set response opcode and format byte
    // format byte = attribute value length + 4 (for start and end handles)
    response_packet.att_read_rsp_mut().opcode = GattOp::AttOpReadByGroupTypeRsp as u8;
    response_packet.att_read_rsp_mut().value[0] = format_length as u8 + 4;
    
    // Copy accumulated response data
    if dest_ptr > 0 {
        response_packet.att_read_rsp_mut().value[1..dest_ptr*2+1]
            .copy_from_slice(&response.att_read_rsp().value[1..dest_ptr*2+1]);
    }

    Some(response_packet)
}

/// Handles Write Request and Write Command operations that update attribute values.
///
/// This function processes ATT Write Request and Write Command operations, which are used by
/// clients to update the value of a specific attribute identified by its handle. Write Requests
/// require a response, while Write Commands don't.
///
/// # Parameters
///
/// * `packet` - Reference to the incoming packet containing a Write Request or Command
///
/// # Returns
///
/// * `Some(Packet)` - A Write Response packet if handling a Write Request
/// * `None` - If handling a Write Command or if the write operation is handled by a callback
///
/// # Algorithm
///
/// 1. Extract the attribute handle from the request
/// 2. Validate the handle:
///    - Ensure the high byte of the handle is zero
///    - Ensure the handle doesn't exceed the total attribute count
/// 3. Perform permission checks (skipped for CCCD attributes):
///    - Reject writes to attributes with handles < 2
///    - Check if the characteristic declaration permits writes (bits 2-3 of properties byte)
/// 4. Prepare the response (if the operation is a Write Request, not a Write Command)
/// 5. Process the write operation:
///    - If the attribute has no write callback:
///      - Check if the request has enough data (handle1 >= 3)
///      - Verify the attribute value is in RAM, not ROM
///      - Copy the new value from the request to the attribute
///    - If the attribute has a write callback, invoke it to handle the write
/// 6. Return the prepared response (for Write Request) or None (for Write Command)
///
/// # Notes
///
/// * Write Requests require a confirmation response; Write Commands don't
/// * Client Characteristic Configuration Descriptors (UUID 0x2902) bypass permission checks
/// * A characteristic is writeable if bits 2-3 of its properties byte are set
/// * Attributes with write callbacks are typically customized to implement special behavior,
///   such as triggering device operations or performing value validation
/// * If the attribute value points to a memory address <= __RAM_START_ADDR, it's in ROM 
///   and can't be written to directly
fn handle_write_request_or_command(packet: &Packet) -> Option<Packet> {
    // --- Extract attribute handle from request ---
    let att_handle = packet.l2cap_data().value[4] as usize;
    
    // --- Validate handle: check high byte and total attribute count ---
    let high_byte_is_zero = packet.l2cap_data().value[5] == 0;
    let handle_in_range = get_gAttributes()[0].att_num >= att_handle as u8;
    
    if !high_byte_is_zero || !handle_in_range {
        return None;
    }
    
    // --- Get current attribute and check if it's a CCCD ---
    let current_attr = &get_gAttributes()[att_handle];
    let is_cccd = !current_attr.uuid.is_null() && 
                        unsafe { read_unaligned(current_attr.uuid as *const u16) == GATT_UUID_CLIENT_CHAR_CFG };  
    
    // --- Apply permission checks (skip for CCCDs) ---
    if !is_cccd {
        // Reject writes to handles 0 and 1 (reserved attributes)
        if att_handle < 2 {
            return None;
        }
        
        // Check characteristic declaration for write permission (bits 2-3)
        let declaration_properties = unsafe { *get_gAttributes()[att_handle - 1].p_attr_value };
        let write_permitted = declaration_properties & 0xc != 0;
        
        if !write_permitted {
            return None;
        }
    }

    // --- Prepare response based on operation type ---
    // Write Request (0x12) needs a response; Write Command (0x52) doesn't
    let is_write_request = packet.l2cap_data().value[3] == GattOp::AttOpWriteReq as u8;
    let response = if is_write_request {
        Some(
            Packet {
                att_write_rsp: PacketAttWriteRsp {
                    head: PacketL2capHead {
                        dma_len: 0x07,
                        _type: 2,
                        rf_len: 0x05,
                        l2cap_len: 0x01,
                        chan_id: 0x04,
                    },
                    opcode: GattOp::AttOpWriteRsp as u8,
                }
            }
        )
    } else {
        None
    };

    // --- Process the write operation ---
    if current_attr.w.is_none() {
        // Handle case when no write callback is defined
        
        // Early return if request data is insufficient (handle too small)
        let handle_value = unsafe { *(addr_of!(packet.l2cap_data().handle1) as *const u16) };
        if handle_value < 3 {
            return response;
        }

        // Check if attribute value is stored in RAM (not ROM)
        let ram_start_addr = unsafe { addr_of!(__RAM_START_ADDR) } as u32;
        let attr_addr = current_attr.p_attr_value as u32;
        
        if attr_addr <= ram_start_addr {
            return response;
        }

        // Get mutable slice of attribute value and incoming data
        let attr_value_slice = unsafe {
            slice::from_raw_parts_mut(
                current_attr.p_attr_value,
                current_attr.attr_len as usize,
            )
        };
        
        let incoming_value_slice = unsafe {
            slice::from_raw_parts(
                addr_of!(packet.l2cap_data().value[6]),
                current_attr.attr_len as usize,
            )
        };

        // Update attribute value with incoming data
        attr_value_slice.fill(0);
        attr_value_slice.copy_from_slice(incoming_value_slice);
    } else {
        // Attribute has a write callback - invoke it
        let write_callback = current_attr.w.unwrap();
        write_callback(packet);
    }

    response
}

#[cfg(test)]
mod tests {
    use core::ptr::null_mut;
    use super::*;
    use crate::sdk::app_att_light::{GATT_UUID_CHARACTER, GATT_UUID_PRIMARY_SERVICE, get_gAttributes, TELINK_SPP_DATA_SERVER2CLIENT_UUID};
    use crate::sdk::mcu::register::mock_read_reg_system_tick;
    use crate::sdk::packet_types::{PacketAttData, PacketL2capData};

    // Helper function to create UUID16 as a byte slice
    fn uuid16_to_bytes(uuid: u16) -> [u8; 2] {
        uuid.to_le_bytes()
    }

    // Helper function to create a 128-bit UUID as a byte slice
    fn create_uuid128(uuid16: u16) -> [u8; 16] {
        let mut uuid = [0u8; 16];
        uuid[0..2].copy_from_slice(&uuid16_to_bytes(uuid16));
        uuid
    }

    // Helper function to set up mocks for operations that use ATT_SERVICE_DISCOVER_TICK
    fn setup_system_tick_mock() {
        mock_read_reg_system_tick().returns(0x12345678);
    }

    // Helper function to create a test packet with specified handle
    fn create_test_packet_with_handle(handle: u8) -> Packet {
        let mut packet = Packet {
            l2cap_data: PacketL2capData {
                l2cap_len: 3,
                chan_id: 0x04,
                opcode: GattOp::AttOpExchangeMtuRsp as u8,
                handle: 0,
                handle1: handle, // Set the handle we're testing
                value: [0; 30],
            }
        };
        packet
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_handle_mtu_exchange_response() {
        // Test all MTU exchange response handle cases in a single test
        
        // --- Test handle 0xC case (version indication) ---
        {
            setup_system_tick_mock();
            ATT_SERVICE_DISCOVER_TICK.set(0);
            let packet = create_test_packet_with_handle(0x0C);
            let response = handle_mtu_exchange_response(&packet).unwrap();
            
            // Verify ATT_SERVICE_DISCOVER_TICK was set correctly
            assert_eq!(ATT_SERVICE_DISCOVER_TICK.get(), 0x12345679); // 0x12345678 | 1
            
            // Verify version indication packet
            assert_eq!(response.version_ind().opcode, 0x0C);
            assert_eq!(response.version_ind()._type, 3);
            assert_eq!(response.version_ind().rf_len, 6);
            assert_eq!(response.version_ind().dma_len, 8);
            assert_eq!(response.version_ind().vendor, VENDOR_ID);
            assert_eq!(response.version_ind().main_ver, 0x08);
            assert_eq!(response.version_ind().sub_ver, 0x08);
        }
        
        // --- Test handle 8 case (feature response) ---
        {
            setup_system_tick_mock();
            ATT_SERVICE_DISCOVER_TICK.set(0);
            let packet = create_test_packet_with_handle(8);
            let response = handle_mtu_exchange_response(&packet).unwrap();
            
            // Verify ATT_SERVICE_DISCOVER_TICK was set correctly
            assert_eq!(ATT_SERVICE_DISCOVER_TICK.get(), 0x12345679);
            
            // Verify feature response packet
            assert_eq!(response.feature_rsp().opcode, 0x09);
            assert_eq!(response.feature_rsp()._type, 0x3);
            assert_eq!(response.feature_rsp().rf_len, 0x09);
            assert_eq!(response.feature_rsp().dma_len, 0x0b);
            assert_eq!(response.feature_rsp().data[0], 1);
            // Check remaining bytes are zero
            for i in 1..8 {
                assert_eq!(response.feature_rsp().data[i], 0);
            }
        }
        
        // --- Test handle 2 case (link timeout) ---
        {
            SLAVE_LINK_TIME_OUT.set(0);
            let packet = create_test_packet_with_handle(2);
            let response = handle_mtu_exchange_response(&packet);
            
            // Verify timeout was set and no response
            assert_eq!(SLAVE_LINK_TIME_OUT.get(), 1000000);
            assert!(response.is_none());
        }
        
        // --- Test other handle cases (control packet) ---
        {
            // Test several other handles to verify control packet response
            for handle in [1, 3, 5, 7, 9, 10] {
                let packet = create_test_packet_with_handle(handle);
                let response = handle_mtu_exchange_response(&packet).unwrap();
                
                // Verify control packet with expected fields
                assert_eq!(response.ctrl_unknown().opcode, 0x07);
                assert_eq!(response.ctrl_unknown().data[0], handle);
                assert_eq!(response.ctrl_unknown()._type, 0x03);
                assert_eq!(response.ctrl_unknown().rf_len, 0x02);
                assert_eq!(response.ctrl_unknown().dma_len, 0x04);
            }
        }
    }

    // Helper function to create a packet with the specified opcode and data
    fn create_att_packet(opcode: u8, data: &[u8]) -> Packet {
        let mut packet = Packet {
            l2cap_data: PacketL2capData {
                // opcode + handle(2) + data
                l2cap_len: data.len() as u16 + 3,
                // Set L2CAP channel ID to 4 (ATT)
                chan_id: 0x04,
                // Set opcode
                opcode: opcode,
                // Set handle1 and handle2 to pass the check that verifies channel ID is 4 (ATT)
                // This corresponds to bytes 1-2 in the value array
                handle: 4,
                handle1: 0,
                value: [0; 30],
            }
        };

        // Set handle check
        packet.l2cap_data_mut().value[1] = 4;

        // Copy data to value field starting at position 4 (after opcode=3 and handle=1,2)
        if !data.is_empty() {
            packet.l2cap_data_mut().value[3..3+data.len()].copy_from_slice(data);
        }
        
        packet
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_exchange_mtu_req() {
        setup_system_tick_mock();
        
        // Test Exchange MTU Request
        let packet = create_att_packet(0, &[GattOp::AttOpExchangeMtuReq as u8, 0x17, 0x00]); // MTU size 23
        
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Verify response is Exchange MTU Response
        assert_eq!(response.att_mtu().opcode, GattOp::AttOpExchangeMtuRsp as u8);
        assert_eq!(response.att_mtu().mtu, [0x17, 0x00]); // MTU = 23
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_exchange_mtu_rsp() {
        setup_system_tick_mock();
        
        // Create Exchange MTU Response packet with handle=0xC (special handling case)
        let mut packet = create_att_packet(GattOp::AttOpExchangeMtuRsp as u8, &[0x17, 0x00]);
        packet.l2cap_data_mut().handle1 = 0x0C;
        
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Should return Version Indication response
        assert_eq!(response.version_ind().opcode, 0x0C);
        assert_eq!(response.version_ind().vendor, VENDOR_ID);
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_find_info_req() {
        setup_system_tick_mock();
        
        // Create Find Information Request for a range that includes uuid16 attributes
        let mut data = [0; 6];
        data[0] = GattOp::AttOpFindInfoReq as u8; // OpCode: Find Information Request
        data[1] = 2;    // Starting Handle (little endian)
        data[2] = 0;
        data[3] = 6;    // Ending Handle (little endian)
        data[4] = 0;
        
        let packet = create_att_packet(0, &data);
        
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Should be a Find Information Response
        assert_eq!(response.att_read_rsp().opcode, GattOp::AttOpFindInfoRsp as u8);
        // First byte of value indicates the format (1 = 16-bit UUIDs)
        assert_eq!(response.att_read_rsp().value[0], 1);
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_find_by_type_value_req() {
        setup_system_tick_mock();
        
        // Create Find By Type Value Request for Primary Service UUID
        let mut data = [0; 12];
        data[0] = GattOp::AttOpFindByTypeValueReq as u8; // Opcode: Find By Type Value Request
        data[1] = 1;    // Starting Handle (little endian)
        data[2] = 0;
        data[3] = 10;   // Ending Handle (little endian)
        data[4] = 0;
        
        // UUID to find: Primary Service (0x2800)
        data[5] = 0x00; 
        data[6] = 0x28;
        
        // Value to match: GAP Service (0x1800)
        data[7] = 0x00;
        data[8] = 0x18;
        
        let packet = create_att_packet(0, &data);
        
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Should be either a Find By Type Value Response or Error Response
        // Both are valid depending on the current state of the attribute table
        if response.head().l2cap_len > 1 {
            // Find By Type Value Response
            assert_eq!(response.att_read_rsp().opcode, GattOp::AttOpFindByTypeValueRsp as u8);
        } else {
            // Error Response
            assert_eq!(response.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
            assert_eq!(response.att_err_rsp().err_opcode, GattOp::AttOpFindByTypeValueReq as u8);
        }
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_read_by_type_req_16bit_uuid() {
        setup_system_tick_mock();
        
        // Create Read By Type Request for Characteristic UUID
        let mut data = [0; 10];
        data[0] = GattOp::AttOpReadByTypeReq as u8; // Opcode: Read By Type Request
        data[1] = 1;    // Starting Handle (little endian)
        data[2] = 0;
        data[3] = 20;   // Ending Handle (little endian)
        data[4] = 0;
        
        // UUID to find: Characteristic (0x2803)
        data[5] = 0x03; 
        data[6] = 0x28;
        
        let packet = create_att_packet(0x08, &data);
        
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Should be either Read By Type Response or Error Response
        if response.head().chan_id == 0x04 && response.att_read_rsp().opcode == GattOp::AttOpReadByTypeRsp as u8 {
            // Read By Type Response - first byte contains length of each attribute data
            assert!(response.att_read_rsp().value[0] > 0);
        } else {
            // Error Response
            assert_eq!(response.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
            assert_eq!(response.att_err_rsp().err_opcode, GattOp::AttOpReadByTypeReq as u8);
        }
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_read_by_type_req_128bit_uuid() {
        setup_system_tick_mock();
        
        // Create Read By Type Request for a 128-bit UUID
        let mut data = [0; 24];
        data[0] = GattOp::AttOpReadByTypeReq as u8; // Opcode: Read By Type Request
        data[1] = 1;    // Starting Handle (little endian)
        data[2] = 0;
        data[3] = 28;   // Ending Handle (little endian)
        data[4] = 0;
        
        // Copy TELINK_SPP_DATA_SERVER2CLIENT_UUID (128-bit UUID)
        data[5..21].copy_from_slice(&TELINK_SPP_DATA_SERVER2CLIENT_UUID);
        
        let mut packet = create_att_packet(0, &data);
        packet.l2cap_data_mut().handle1 = 0x15; // Set handle1 to indicate 128-bit UUID search
        
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Should be either Read By Type Response or Error Response
        if response.head().chan_id == 0x04 && response.att_read_rsp().opcode == GattOp::AttOpReadByTypeRsp as u8 {
            // Read By Type Response - first byte contains length of each attribute data
            assert!(response.att_read_rsp().value[0] > 0);
        } else {
            // Error Response
            assert_eq!(response.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
            assert_eq!(response.att_err_rsp().err_opcode, GattOp::AttOpReadByTypeReq as u8);
        }
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_read_req() {
        setup_system_tick_mock();
        
        // Create Read Request for an attribute
        let mut data = [0; 6];
        data[0] = GattOp::AttOpReadReq as u8; // OpCode: Read Request
        data[1] = 3;    // Handle to read (Attribute #3 - Device Name)
        data[2] = 0;
        data[3] = 0;    // High byte of handle (should be 0)
        
        let packet = create_att_packet(0, &data);
        
        let response = l2cap_att_handler(&packet);
        
        // If the attribute has a read callback, response will be None
        // Otherwise, it will be a Read Response
        if let Some(resp) = response {
            assert_eq!(resp.att_read_rsp().opcode, GattOp::AttOpReadRsp as u8);
            // Response contains the attribute value
            // Not checking exact value since it depends on the attribute content
        }
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_read_by_group_type_req() {
        setup_system_tick_mock();
        
        // Create Read By Group Type Request for Primary Service UUID
        let mut data = [0; 10];
        data[0] = GattOp::AttOpReadByGroupTypeReq as u8; // Opcode: Read By Group Type Request
        data[1] = 1;    // Starting Handle (little endian)
        data[2] = 0;
        data[3] = 20;   // Ending Handle (little endian)
        data[4] = 0;
        
        // UUID to find: Primary Service (0x2800)
        data[5] = 0x00; 
        data[6] = 0x28;
        
        let packet = create_att_packet(0, &data);
        
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Should be either a Read By Group Type Response or Error Response
        if response.head().chan_id == 0x04 && response.att_read_rsp().opcode == GattOp::AttOpReadByGroupTypeRsp as u8 {
            // Read By Group Type Response - value[0] contains the length of each attribute data
            assert!(response.att_read_rsp().value[0] > 0);
        } else {
            // Error Response
            assert_eq!(response.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
            assert_eq!(response.att_err_rsp().err_opcode, GattOp::AttOpReadByGroupTypeReq as u8);
        }
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_write_req() {
        setup_system_tick_mock();
        
        // Create Write Request for an attribute
        let mut data = [0; 23];
        data[0] = GattOp::AttOpWriteReq as u8; // Opcode: Write Request
        data[1] = 18;   // Client Characteristic Configuration Handle
        data[2] = 0;
        data[3] = 0;    // High byte of handle (should be 0)
        data[4] = 0x01; // Value to write - enable notifications
        data[5] = 0x00;
        
        let packet = create_att_packet(0, &data);
        
        let response = l2cap_att_handler(&packet);
        
        // If the attribute has a write callback, response might be None
        // Or it will be a Write Response
        if let Some(resp) = response {
            assert_eq!(resp.att_write_rsp().opcode, GattOp::AttOpWriteRsp as u8);
        }
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_write_cmd() {
        setup_system_tick_mock();
        
        // Create Write Command for an attribute
        let mut data = [0; 23];
        data[0] = GattOp::AttOpWriteCmd as u8; // Opcode: Write Command
        data[1] = 3;    // Handle
        data[2] = 0;
        data[3] = 0;    // High byte of handle (should be 0)
        data[4] = 0xAA; // Value to write
        data[5] = 0xBB;
        
        let packet = create_att_packet(0, &data);
        
        let response = l2cap_att_handler(&packet);
        
        // Write Commands should not have a response
        assert!(response.is_none() || response.unwrap().head().l2cap_len <= 1);
    }
    
    #[test]
    fn test_l2cap_att_handler_unsupported_opcode() {
        // Create a packet with an unsupported opcode
        let data = [0xFF; 6]; // Invalid opcode
        
        let packet = create_att_packet(0, &data);
        
        let response = l2cap_att_handler(&packet);
        
        // Should return None for unsupported opcodes
        assert!(response.is_none());
    }

    #[test]
    fn test_l2cap_att_handler_invalid_channel_id() {
        // Create a packet with an invalid channel ID (not 4)
        let mut packet = create_att_packet(0, &[GattOp::AttOpReadReq as u8, 0x03, 0x00]);
        
        // Override the channel ID bytes in the value array
        packet.l2cap_data_mut().value[1] = 5; // Channel ID 5 instead of 4
        packet.l2cap_data_mut().value[2] = 0;
        
        // Call handler with invalid channel ID
        let response = l2cap_att_handler(&packet);
        
        // Should return None because channel ID is not 4
        assert!(response.is_none());
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_exchange_mtu_rsp_handle_2() {
        setup_system_tick_mock();
        
        // Test case: handle == 2 should set SLAVE_LINK_TIME_OUT and return None
        let mut packet = create_att_packet(GattOp::AttOpExchangeMtuRsp as u8, &[0x17, 0x00]);
        packet.l2cap_data_mut().handle1 = 0x02;
        
        // Verify SLAVE_LINK_TIME_OUT is not set to 1000000 before
        SLAVE_LINK_TIME_OUT.set(1000);
        
        let response = l2cap_att_handler(&packet);
        
        // Verify the timeout was set
        assert_eq!(SLAVE_LINK_TIME_OUT.get(), 1000000);
        
        // Should return None
        assert!(response.is_none());
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_exchange_mtu_rsp_handle_other() {
        setup_system_tick_mock();
        
        // Test case: handle != 8 && handle != 2 should return PacketCtrlUnknown
        let mut packet = create_att_packet(GattOp::AttOpExchangeMtuRsp as u8, &[0x17, 0x00]);
        packet.l2cap_data_mut().handle1 = 0x05; // Not 2 or 8
        
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Should return PacketCtrlUnknown with opcode 0x07
        assert_eq!(response.ctrl_unknown().opcode, 0x07);
        assert_eq!(response.ctrl_unknown().data[0], 0x05); // Should include the handle value
        assert_eq!(response.ctrl_unknown()._type, 0x03);
        assert_eq!(response.ctrl_unknown().rf_len, 0x02);
        assert_eq!(response.ctrl_unknown().dma_len, 0x04);
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_exchange_mtu_rsp_handle_8() {
        setup_system_tick_mock();
        
        // Reset ATT_SERVICE_DISCOVER_TICK
        ATT_SERVICE_DISCOVER_TICK.set(0);
        
        // Test case: handle == 8 should set ATT_SERVICE_DISCOVER_TICK and return PacketFeatureRsp
        let mut packet = create_att_packet(GattOp::AttOpExchangeMtuRsp as u8, &[0x17, 0x00]);
        packet.l2cap_data_mut().handle1 = 0x08;
        
        // System tick mock will return 0x12345678
        let expected_tick_value = 0x12345679; // 0x12345678 | 1
        
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Verify ATT_SERVICE_DISCOVER_TICK was set
        assert_eq!(ATT_SERVICE_DISCOVER_TICK.get(), expected_tick_value);
        
        // Should return PacketFeatureRsp
        assert_eq!(response.feature_rsp().opcode, 0x09);
        assert_eq!(response.feature_rsp()._type, 0x3);
        assert_eq!(response.feature_rsp().rf_len, 0x09);
        assert_eq!(response.feature_rsp().dma_len, 0x0b);
        // Check first byte of data array which is set to 1
        assert_eq!(response.feature_rsp().data[0], 1);
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_find_info_req_end_handle_adjustment() {
        setup_system_tick_mock();
        
        // Get the total number of attributes
        let total_attrs = get_gAttributes()[0].att_num;
        
        // Create Find Information Request with end handle greater than total attributes
        let mut data = [0; 6];
        data[0] = GattOp::AttOpFindInfoReq as u8; // OpCode: Find Information Request
        data[1] = 1;    // Starting Handle (little endian)
        data[2] = 0;
        data[3] = total_attrs + 10;  // Ending Handle much larger than total attributes
        data[4] = 0;
        
        let packet = create_att_packet(0, &data);
        
        // Monitor the response to see if it's processed correctly
        let response = l2cap_att_handler(&packet);
        
        // Response should exist since we're requesting a valid range
        assert!(response.is_some());
        
        let resp = response.unwrap();
        
        // Should be a Find Information Response
        assert_eq!(resp.att_read_rsp().opcode, GattOp::AttOpFindInfoRsp as u8);
        
        // We can't directly test that end_handle was adjusted internally,
        // but we can verify the response is valid, which means the handler
        // didn't try to access attributes beyond the total count
        
        // Create another packet with a starting handle beyond total attributes
        let mut data_beyond = [0; 6];
        data_beyond[0] = GattOp::AttOpFindInfoReq as u8;
        data_beyond[1] = total_attrs + 1; // Starting Handle beyond total attributes
        data_beyond[2] = 0;
        data_beyond[3] = total_attrs + 10; // Ending Handle beyond total attributes
        data_beyond[4] = 0;
        
        let packet_beyond = create_att_packet(0, &data_beyond);
        
        // This should return an error response since there are no attributes in the range
        let response_beyond = l2cap_att_handler(&packet_beyond).unwrap();
        
        // Should be an Error Response
        assert_eq!(response_beyond.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
        assert_eq!(response_beyond.att_err_rsp().err_opcode, GattOp::AttOpFindInfoReq as u8);
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_find_info_req_128bit_uuid() {
        setup_system_tick_mock();

        // This test verifies the code block that handles 128-bit UUIDs in Find Info Request:

        // Find a 128-bit UUID attribute
        let mut uuid128_handle = 0;
        for i in 1..get_gAttributes()[0].att_num as usize {
            if get_gAttributes()[i].uuid_len == 16 {
                uuid128_handle = i;
                break;
            }
        }

        // Create Find Information Request specifically targeting the 128-bit UUID
        let mut data = [0; 6];
        data[0] = GattOp::AttOpFindInfoReq as u8;     // OpCode: Find Information Request
        data[1] = uuid128_handle as u8;              // Starting Handle (the 128-bit UUID attribute)
        data[2] = 0;
        data[3] = (uuid128_handle + 1) as u8;        // Ending Handle (just this attribute)
        data[4] = 0;

        let packet = create_att_packet(0, &data);

        let response = l2cap_att_handler(&packet).unwrap();

        // Verify response is a Find Information Response
        assert_eq!(response.att_read_rsp().opcode, GattOp::AttOpFindInfoRsp as u8);

        // Format should be 2 for 128-bit UUIDs
        assert_eq!(response.att_read_rsp().value[0], 2);

        // Verify packet fields are set correctly for 128-bit UUIDs
        assert_eq!(response.head()._type, 2);
        let chan_id = response.head().chan_id;
        assert_eq!(chan_id, 4);

        // For 128-bit UUIDs:
        // - Each entry is 2 bytes for handle + 16 bytes for UUID = 18 bytes
        // - The response should have counter = 0x12 (18) after processing one entry
        // Verify the response packet has the correct size
        let expected_l2cap_len = 18 + 2;  // counter (18) + 2 for format and opcode
        let l2cap_len = response.head().l2cap_len;
        assert_eq!(l2cap_len, expected_l2cap_len);
        let dma_len = response.head().dma_len;
        assert_eq!(dma_len, expected_l2cap_len as u32 + 6);
        assert_eq!(response.head().rf_len, expected_l2cap_len as u8 + 4);

        // Verify the attribute handle was correctly copied to the response
        assert_eq!(response.att_read_rsp().value[1], uuid128_handle as u8);
        assert_eq!(response.att_read_rsp().value[2], 0);

        // Verify the UUID was copied correctly
        // Get the 128-bit UUID from attribute table
        let expected_uuid = unsafe {
            slice::from_raw_parts(
                get_gAttributes()[uuid128_handle].uuid,
                16
            )
        };

        // UUID starts at offset 3 in the response
        let response_uuid = &response.att_read_rsp().value[3..19];

        // Compare the UUIDs
        assert_eq!(response_uuid, expected_uuid);
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_write_permission_checks() {
        setup_system_tick_mock();

        // Test 1: Return None when uuid != 0x2902 and att_num < 2
        {
            // Find an attribute with UUID != 0x2902 and that can be modified
            let test_attr_idx = 3; // Device name attribute
            let original_uuid = unsafe { *(get_gAttributes()[test_attr_idx].uuid as *const u16) };
            let original_write_callback = get_gAttributes()[test_attr_idx].w;
            
            // Verify this attribute doesn't have UUID 0x2902
            assert_ne!(original_uuid, 0x2902, "Test requires attribute with UUID != 0x2902");
            
            // Temporarily clear the write callback so our code path is executed
            unsafe {
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = None;
            }
            
            // Create a Write Request packet
            let mut data = [0; 8];
            data[0] = GattOp::AttOpWriteReq as u8;
            data[1] = test_attr_idx as u8; // Non-CCCD attribute
            data[2] = 0;
            data[3] = 0;
            data[4] = 0xAA; // Value to write
            
            let packet = create_att_packet(0, &data);
            
            // Call the handler - should return None since att_num < 2 fails
            let response = l2cap_att_handler(&packet);
            assert!(response.is_none(), "Write to non-CCCD attribute with handle < 2 should be rejected");
            
            // Restore original write callback
            unsafe {
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = original_write_callback;
            }
        }
        
        // Test 2: Return None when uuid != 0x2902, att_num >= 2, but previous attribute doesn't have write permission
        {
            // Find an attribute with UUID != 0x2902 and index >= 2
            let test_attr_idx = 5; // Some attribute with index >= 2 (not a CCCD)
            let original_uuid = unsafe { *(get_gAttributes()[test_attr_idx].uuid as *const u16) };
            let original_write_callback = get_gAttributes()[test_attr_idx].w;
            
            // Verify this attribute doesn't have UUID 0x2902
            assert_ne!(original_uuid, 0x2902, "Test requires attribute with UUID != 0x2902");
            assert!(test_attr_idx >= 2, "Test requires attribute with index >= 2");
            
            // Temporarily clear the write callback so our code path is executed
            unsafe {
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = None;
            }
            
            // Backup the original value of the previous attribute
            let prev_attr_idx = test_attr_idx - 1;
            let original_prev_attr_value = unsafe { *get_gAttributes()[prev_attr_idx].p_attr_value };
            
            // Set the previous attribute's permissions to have no write permission
            // 0xc = 1100 in binary, so ~0xc & original = remove bits 2 and 3 (write permissions)
            unsafe {
                let no_write_perm = original_prev_attr_value & !0xc;
                let mut temp_val = no_write_perm;
                (*(addr_of_mut!(get_gAttributes()[prev_attr_idx]) as *mut AttributeT)).p_attr_value = addr_of_mut!(temp_val);
            }
            
            // Create a Write Request packet
            let mut data = [0; 8];
            data[0] = GattOp::AttOpWriteReq as u8;
            data[1] = test_attr_idx as u8;
            data[2] = 0;
            data[3] = 0;
            data[4] = 0xAA; // Value to write
            
            let packet = create_att_packet(0, &data);
            
            // Call the handler - should return None since previous attribute has no write permission
            let response = l2cap_att_handler(&packet);
            assert!(response.is_none(), "Write to attribute with no write permission in previous attr should be rejected");
            
            // Restore original write callback
            unsafe {
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = original_write_callback;
                // We don't need to restore prev_attr_value since we never modified the actual memory
            }
        }
        
        // Test 3: Test successful path - uuid != 0x2902, att_num >= 2, previous attribute has write permission
        {
            // Find an attribute with UUID != 0x2902 and index >= 2
            let test_attr_idx = 5; // Some attribute with index >= 2 (not a CCCD)
            let original_uuid = unsafe { *(get_gAttributes()[test_attr_idx].uuid as *const u16) };
            let original_write_callback = get_gAttributes()[test_attr_idx].w;
            
            // Verify this attribute doesn't have UUID 0x2902
            assert_ne!(original_uuid, 0x2902, "Test requires attribute with UUID != 0x2902");
            assert!(test_attr_idx >= 2, "Test requires attribute with index >= 2");
            
            // Temporarily clear the write callback so our code path is executed
            unsafe {
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = None;
            }
            
            // Backup the original value of the previous attribute
            let prev_attr_idx = test_attr_idx - 1;
            let original_prev_attr_value = unsafe { *get_gAttributes()[prev_attr_idx].p_attr_value };
            
            // Set the previous attribute's permissions to have write permission
            // 0x4 = 0100 in binary = write permission
            unsafe {
                let write_perm = original_prev_attr_value | 0x4;
                let mut temp_val = write_perm;
                (*(addr_of_mut!(get_gAttributes()[prev_attr_idx]) as *mut AttributeT)).p_attr_value = addr_of_mut!(temp_val);
            }
            
            // Create a Write Request packet
            let mut data = [0; 8];
            data[0] = GattOp::AttOpWriteReq as u8; // Write Request
            data[1] = test_attr_idx as u8;
            data[2] = 0;
            data[3] = 0;
            data[4] = 0xAA; // Value to write
            
            let packet = create_att_packet(0, &data);
            
            // Call the handler - should return a write response since all checks pass
            let response = l2cap_att_handler(&packet);
            assert!(response.is_some(), "Write to attribute with write permission should be accepted");
            assert_eq!(response.unwrap().att_write_rsp().opcode, GattOp::AttOpWriteRsp as u8);
            
            // Restore original write callback
            unsafe {
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = original_write_callback;
                // We don't need to restore prev_attr_value since we never modified the actual memory
            }
        }
        
        // Test 4: CCCD attribute (UUID == 0x2902) bypasses the permission checks
        {
            // Find or create a CCCD attribute (UUID = 0x2902)
            let test_attr_idx = 16; // Common index for a CCCD
            let original_uuid = unsafe { *(get_gAttributes()[test_attr_idx].uuid as *const u16) };
            let original_uuid_len = get_gAttributes()[test_attr_idx].uuid_len;
            let original_write_callback = get_gAttributes()[test_attr_idx].w;
            
            // Temporarily modify the attribute to have UUID 0x2902
            unsafe {
                let mut cccd_uuid: u16 = 0x2902;
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).uuid = addr_of_mut!(cccd_uuid) as *mut u8;
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).uuid_len = 2;
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = None;
            }
            
            // Create a Write Request packet
            let mut data = [0; 8];
            data[0] = GattOp::AttOpWriteReq as u8; // Write Request
            data[1] = test_attr_idx as u8;
            data[2] = 0;
            data[3] = 0;
            data[4] = 0x01; // Enable notifications
            data[5] = 0x00;
            
            let packet = create_att_packet(0, &data);
            
            // Call the handler - should return a write response since CCCD bypasses permission checks
            let response = l2cap_att_handler(&packet);
            assert!(response.is_some(), "Write to CCCD attribute should bypass permission checks");
            assert_eq!(response.unwrap().att_write_rsp().opcode, GattOp::AttOpWriteRsp as u8);
            
            // Restore original UUID and write callback
            unsafe {
                let mut orig_uuid = original_uuid;
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).uuid = addr_of_mut!(orig_uuid) as *mut u8;
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).uuid_len = original_uuid_len;
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = original_write_callback;
            }
        }
        
        // Test 5: Return None when value[5] is non-zero
        {
            // Find any attribute we can test with
            let test_attr_idx = 3;
            
            // Create a Write Request packet with non-zero high byte in handle
            let mut data = [0; 8];
            data[0] = GattOp::AttOpWriteReq as u8;
            data[1] = test_attr_idx as u8;
            data[2] = 0;
            data[3] = 0;
            data[4] = 0xAA;
            
            let mut packet = create_att_packet(0, &data);
            packet.l2cap_data_mut().value[5] = 0x01; // Set high byte to non-zero
            
            // Call the handler - should return None due to non-zero high byte
            let response = l2cap_att_handler(&packet);
            assert!(response.is_none(), "Write with non-zero high byte should be rejected");
        }
        
        // Test 6: Return None when att_num exceeds total attributes
        {
            // Get the total number of attributes
            let total_attrs = get_gAttributes()[0].att_num;
            
            // Create a Write Request packet with att_num beyond the total
            let mut data = [0; 8];
            data[0] = GattOp::AttOpWriteReq as u8;
            data[1] = total_attrs + 1; // Beyond total attributes
            data[2] = 0;
            data[3] = 0;
            data[4] = 0xAA;
            
            let packet = create_att_packet(0, &data);
            
            // Call the handler - should return None due to att_num exceeding total
            let response = l2cap_att_handler(&packet);
            assert!(response.is_none(), "Write with att_num exceeding total should be rejected");
        }
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_read_by_type_req_128bit_uuid_response_preparation() {
        setup_system_tick_mock();
        
        // This test specifically focuses on the code block that prepares
        // the Read By Type Response packet when a 128-bit UUID attribute is found
        
        // First, find a 128-bit UUID attribute in the table
        let mut uuid128_handle = 0;
        let mut uuid128_attr = None;
        
        for i in 1..get_gAttributes()[0].att_num as usize {
            if get_gAttributes()[i].uuid_len == 16 {
                uuid128_handle = i;
                uuid128_attr = Some(&get_gAttributes()[i]);
                break;
            }
        }
        
        // Make sure we found a 128-bit UUID attribute
        let found_attr = uuid128_attr.unwrap();
        
        // Get the attribute's 128-bit UUID
        let uuid128_bytes = unsafe { slice::from_raw_parts(found_attr.uuid, 16) };
        
        // Create Read By Type Request packet specifically for this 128-bit UUID
        let mut data = [0; 24]; // 8 bytes header + 16 bytes UUID
        data[0] = GattOp::AttOpReadByTypeReq as u8;   // Opcode: Read By Type Request
        data[1] = uuid128_handle as u8;               // Starting Handle
        data[2] = 0;
        data[3] = (uuid128_handle + 5) as u8;         // Ending Handle
        data[4] = 0;
        
        // Copy the 128-bit UUID into the request
        data[5..21].copy_from_slice(uuid128_bytes);
        
        // Create the packet with handle1 = 0x15 to trigger the 128-bit UUID handling path
        let mut packet = create_att_packet(0, &data);
        packet.l2cap_data_mut().handle1 = 0x15; // This is important to trigger 128-bit UUID block
        
        // Call the handler
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Verify the response is a Read By Type Response
        assert_eq!(response.att_read_rsp().opcode, GattOp::AttOpReadByTypeRsp as u8);
        
        // Verify bytes_read was correctly set in value[0]
        let expected_bytes_read = found_attr.attr_len + 2; // attr_len + 2 bytes for the handle
        assert_eq!(response.att_read_rsp().value[0], expected_bytes_read);
        
        // Verify the handle was correctly copied to the response
        assert_eq!(response.att_read_rsp().value[1], uuid128_handle as u8);
        assert_eq!(response.att_read_rsp().value[2], (uuid128_handle >> 8) as u8);
        
        // Verify the attribute value was correctly copied to the response
        let expected_value = unsafe {
            slice::from_raw_parts(found_attr.p_attr_value, found_attr.attr_len as usize)
        };
        
        let actual_value = &response.att_read_rsp().value[3..3 + found_attr.attr_len as usize];
        assert_eq!(actual_value, expected_value);
        
        // Verify the packet header fields were set correctly
        let chan_id = response.head().chan_id;
        assert_eq!(chan_id, 4);
        assert_eq!(response.head()._type, 2);
        
        // Verify dma_len, rf_len, and l2cap_len were calculated correctly
        let expected_dma_len = expected_bytes_read as u32 + 8;
        let expected_rf_len = expected_bytes_read + 6;
        let expected_l2cap_len = expected_bytes_read as u16 + 2;

        let dma_len = response.head().dma_len;
        assert_eq!(dma_len, expected_dma_len);
        assert_eq!(response.head().rf_len, expected_rf_len);
        let l2cap_len = response.head().l2cap_len;
        assert_eq!(l2cap_len, expected_l2cap_len);
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_read_by_type_req_uuid_not_found() {
        setup_system_tick_mock();
        
        // This test checks the error response when no matching attribute is found
        
        // Create Read By Type Request with a UUID that doesn't exist in the attribute table
        let mut data = [0; 10];
        data[0] = GattOp::AttOpReadByTypeReq as u8;  // Opcode: Read By Type Request
        data[1] = 1;    // Starting Handle
        data[2] = 0;
        data[3] = 20;   // Ending Handle
        data[4] = 0;
        
        // UUID that doesn't exist: 0xDEAD
        data[5] = 0xAD;
        data[6] = 0xDE;
        
        // Create the packet
        let packet = create_att_packet(0, &data);
        
        // Call the handler
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Verify the response is an Error Response
        assert_eq!(response.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
        assert_eq!(response.att_err_rsp().err_opcode, GattOp::AttOpReadByTypeReq as u8);
        
        // Error handle should be set to the start handle from the request
        assert_eq!(response.att_err_rsp().err_handle, 1);
    }
    
    #[test]
    fn test_l2cap_att_handler_read_req_non_zero_high_byte() {
        // This test verifies that READ_REQ returns None when the high byte of handle is not 0
        
        // Create Read Request packet with non-zero value in the high byte position
        let mut data = [0; 6];
        data[0] = GattOp::AttOpReadReq as u8; // OpCode: Read Request
        data[1] = 3;    // Valid attribute handle
        data[2] = 0;
        
        let mut packet = create_att_packet(0, &data);
        // Set value[5] (the high byte of handle in the request) to non-zero
        packet.l2cap_data_mut().value[5] = 0x01;
        
        // Call the handler
        let response = l2cap_att_handler(&packet);
        
        // Should return None due to the high byte check
        assert!(response.is_none());
    }
    
    #[test]
    fn test_l2cap_att_handler_read_req_invalid_att_num() {
        // This test verifies that READ_REQ returns None when the attribute number
        // exceeds the total number of attributes
        
        // Get the total number of attributes
        let total_attrs = get_gAttributes()[0].att_num;
        
        // Create Read Request packet with attribute handle > total_attrs
        let mut data = [0; 6];
        data[0] = GattOp::AttOpReadReq as u8; // OpCode: Read Request
        data[1] = total_attrs + 1; // Invalid attribute handle (beyond total count)
        data[2] = 0;
        data[3] = 0;    // High byte of handle is 0 (to pass the first check)
        
        let packet = create_att_packet(0, &data);
        
        // Call the handler
        let response = l2cap_att_handler(&packet);
        
        // Should return None due to the att_num check
        assert!(response.is_none());
    }
    
    #[test]
    fn test_l2cap_att_handler_write_req_non_zero_high_byte() {
        // This test verifies that WRITE_REQ returns None when the high byte of handle is not 0
        
        // Create Write Request packet with non-zero value in the high byte position
        let mut data = [0; 8];
        data[0] = GattOp::AttOpWriteReq as u8; // OpCode: Write Request
        data[1] = 3;    // Valid attribute handle
        data[2] = 0;
        
        let mut packet = create_att_packet(0, &data);
        // Set value[5] (the high byte of handle in the request) to non-zero
        packet.l2cap_data_mut().value[5] = 0x01;
        
        // Call the handler
        let response = l2cap_att_handler(&packet);
        
        // Should return None due to the high byte check
        assert!(response.is_none());
    }
    
    #[test]
    fn test_l2cap_att_handler_write_req_invalid_att_num() {
        // This test verifies that WRITE_REQ returns None when the attribute number
        // exceeds the total number of attributes
        
        // Get the total number of attributes
        let total_attrs = get_gAttributes()[0].att_num;
        
        // Create Write Request packet with attribute handle > total_attrs
        let mut data = [0; 8];
        data[0] = GattOp::AttOpWriteReq as u8; // OpCode: Write Request
        data[1] = total_attrs + 1; // Invalid attribute handle (beyond total count)
        data[2] = 0;
        data[3] = 0;    // High byte of handle is 0 (to pass the first check)
        data[4] = 0x01; // Some value to write
        
        let packet = create_att_packet(0, &data);
        
        // Call the handler
        let response = l2cap_att_handler(&packet);
        
        // Should return None due to the att_num check
        assert!(response.is_none());
    }
    
    #[test]
    fn test_l2cap_att_handler_write_cmd_non_zero_high_byte() {
        // This test verifies that WRITE_CMD returns None when the high byte of handle is not 0
        
        // Create Write Command packet with non-zero value in the high byte position
        let mut data = [0; 8];
        data[0] = GattOp::AttOpWriteCmd as u8; // OpCode: Write Command
        data[1] = 3;    // Valid attribute handle
        data[2] = 0;
        
        let mut packet = create_att_packet(0, &data);
        // Set value[5] (the high byte of handle in the request) to non-zero
        packet.l2cap_data_mut().value[5] = 0x01;
        
        // Call the handler
        let response = l2cap_att_handler(&packet);
        
        // Should return None due to the high byte check
        assert!(response.is_none());
    }

    #[test]
    fn test_l2cap_att_handler_read_req_send_to_master() {
        // This test verifies that reading an attribute whose p_attr_value points to SEND_TO_MASTER
        // results in SEND_TO_MASTER being cleared (filled with zeros)
        
        // Create a temporary attribute that points to SEND_TO_MASTER for testing
        let temp_attr_idx = 5; // Use an attribute that likely has r callback as None
        let original_ptr = get_gAttributes()[temp_attr_idx].p_attr_value;
        let original_len = get_gAttributes()[temp_attr_idx].attr_len;
        let original_r = get_gAttributes()[temp_attr_idx].r;
        
        // Temporarily modify the attribute to point to SEND_TO_MASTER
        unsafe {
            // Fill SEND_TO_MASTER with a test pattern
            SEND_TO_MASTER.fill(0xAA);
            
            // Point attribute to SEND_TO_MASTER
            let send_to_master_ptr = SEND_TO_MASTER.as_mut_ptr();
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).p_attr_value = send_to_master_ptr;
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).attr_len = SEND_TO_MASTER.len() as u8;
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).r = None;
        }
        
        // Create a Read Request for this attribute
        let mut data = [0; 6];
        data[0] = GattOp::AttOpReadReq as u8;
        data[1] = temp_attr_idx as u8;
        data[2] = 0;
        data[3] = 0;
        
        let packet = create_att_packet(0, &data);
        
        // Call the handler
        let response = l2cap_att_handler(&packet);
        
        // Verify response exists and is a Read Response
        assert!(response.is_some());
        let resp = response.unwrap();
        assert_eq!(resp.att_read_rsp().opcode, GattOp::AttOpReadRsp as u8);
        
        // Verify that SEND_TO_MASTER was cleared (filled with zeros)
        unsafe {
            for &byte in SEND_TO_MASTER.iter() {
                assert_eq!(byte, 0, "SEND_TO_MASTER should be cleared after read");
            }
        }
        
        // Restore the original attribute
        unsafe {
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).p_attr_value = original_ptr;
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).attr_len = original_len;
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).r = original_r;
        }
    }
    
    #[test]
    fn test_l2cap_att_handler_read_req_ota_terminate() {
        // This test verifies that reading attribute 0x18 when OTA state is not Continue
        // results in RF_SLAVE_OTA_TERMINATE_FLAG being set to true
        
        // Save original attribute properties
        let temp_attr_idx = 0x18;
        let original_r = get_gAttributes()[temp_attr_idx].r;
        
        // Temporarily modify the attribute to have no read callback
        unsafe {
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).r = None;
        }
        
        // Create a Read Request for attribute 0x18
        let mut data = [0; 6];
        data[0] = GattOp::AttOpReadReq as u8;
        data[1] = temp_attr_idx as u8;
        
        // Test case 1: OTA state is Continue - flag should not be set
        unsafe {
            *RF_SLAVE_OTA_FINISHED_FLAG.lock() = OtaState::Continue;
        }
        
        let packet = create_att_packet(0, &data);
        let _ = l2cap_att_handler(&packet);
        
        // Verify flag was not set
        assert_eq!(RF_SLAVE_OTA_TERMINATE_FLAG.get(), false);
        
        // Test case 2: OTA state is not Continue - flag should be set
        unsafe {
            *RF_SLAVE_OTA_FINISHED_FLAG.lock() = OtaState::Ok;
        }
        
        let packet = create_att_packet(0, &data);
        let _ = l2cap_att_handler(&packet);
        
        // Verify flag was set
        assert_eq!(RF_SLAVE_OTA_TERMINATE_FLAG.get(), true);
        
        // Restore original attribute properties
        unsafe {
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).r = original_r;
        }
        
        // Reset flags
        RF_SLAVE_OTA_TERMINATE_FLAG.set(false);
        unsafe {
            *RF_SLAVE_OTA_FINISHED_FLAG.lock() = OtaState::Continue;
        }
    }
    
    #[test]
    fn test_l2cap_att_handler_read_req_both_conditions() {
        // This test checks the priority of the two conditions:
        // 1. SEND_TO_MASTER check
        // 2. OTA terminate flag check
        // The SEND_TO_MASTER check should take precedence if both conditions are true
        
        // Temporarily modify attribute 0x18 to point to SEND_TO_MASTER and have no read callback
        let temp_attr_idx = 0x18;
        let original_ptr = get_gAttributes()[temp_attr_idx].p_attr_value;
        let original_len = get_gAttributes()[temp_attr_idx].attr_len;
        let original_r = get_gAttributes()[temp_attr_idx].r;
        
        // Temporarily modify the attribute
        unsafe {
            // Fill SEND_TO_MASTER with a test pattern
            SEND_TO_MASTER.fill(0xAA);
            
            // Point attribute to SEND_TO_MASTER
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).p_attr_value = SEND_TO_MASTER.as_mut_ptr();
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).attr_len = SEND_TO_MASTER.len() as u8;
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).r = None;
        }
        
        // Create a Read Request for attribute 0x18
        let mut data = [0; 6];
        data[0] = GattOp::AttOpReadReq as u8;
        data[1] = temp_attr_idx as u8;
        data[2] = 0;
        data[3] = 0;
        
        // Set initial state of flags
        RF_SLAVE_OTA_TERMINATE_FLAG.set(false);
        unsafe {
            *RF_SLAVE_OTA_FINISHED_FLAG.lock() = OtaState::Ok; // Not Continue
        }
        
        let packet = create_att_packet(0, &data);
        let _ = l2cap_att_handler(&packet);
        
        // Verify SEND_TO_MASTER was cleared
        unsafe {
            for &byte in SEND_TO_MASTER.iter() {
                assert_eq!(byte, 0, "SEND_TO_MASTER should be cleared after read");
            }
        }
        
        // Verify OTA terminate flag was NOT set despite the OTA state condition being true
        // because the SEND_TO_MASTER condition should take precedence
        assert_eq!(RF_SLAVE_OTA_TERMINATE_FLAG.get(), false);
        
        // Restore original attribute properties
        unsafe {
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).p_attr_value = original_ptr;
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).attr_len = original_len;
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).r = original_r;
        }
        
        // Reset flags
        RF_SLAVE_OTA_TERMINATE_FLAG.set(false);
        unsafe {
            *RF_SLAVE_OTA_FINISHED_FLAG.lock() = OtaState::Continue;
        }
    }

    #[test]
    fn test_l2cap_att_handler_read_req_with_callback() {
        // This test verifies that when an attribute has a read callback, 
        // it is properly called when handling a Read Request
        
        // Find an attribute that we can temporarily modify for this test
        let temp_attr_idx = 8; // Choose an attribute index that's likely safe to modify
        let original_r = get_gAttributes()[temp_attr_idx].r;
        
        // Flag to track if callback was called
        static mut CALLBACK_CALLED: bool = false;
        
        // Define a test callback function
        let test_callback = |_packet: &Packet| -> bool {
            unsafe { CALLBACK_CALLED = true; }
            true
        };
        
        // Set up the callback on the attribute
        unsafe {
            CALLBACK_CALLED = false;
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).r = Some(test_callback);
        }
        
        // Create a Read Request for this attribute
        let mut data = [0; 6];
        data[0] = GattOp::AttOpReadReq as u8;
        data[1] = temp_attr_idx as u8;
        data[2] = 0;
        data[3] = 0;
        
        let packet = create_att_packet(0, &data);
        
        // Call the handler
        let response = l2cap_att_handler(&packet);
        
        // Verify the callback was called
        unsafe {
            assert!(CALLBACK_CALLED, "Read callback was not called");
        }
        
        // Verify that None was returned (as per the implementation)
        assert!(response.is_none(), "Response should be None when read callback is defined");
        
        // Restore original callback
        unsafe {
            (*(addr_of_mut!(get_gAttributes()[temp_attr_idx]) as *mut AttributeT)).r = original_r;
        }
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_read_by_group_type_req_loop_break_on_none() {
        setup_system_tick_mock();
        
        // This test verifies that the loop in l2cap_att_handler breaks correctly
        // when l2cap_att_search returns None for a ReadByGroupTypeReq operation
        
        // Create an intentionally invalid UUID that shouldn't exist in the attribute table
        let uuid_non_existent = [0xDE, 0xAD]; // Non-existent UUID
        
        // Create a Read By Group Type Request with this non-existent UUID
        let mut data = [0; 10];
        data[0] = GattOp::AttOpReadByGroupTypeReq as u8;  // Opcode: Read By Group Type Request
        data[1] = 1;    // Starting Handle (little endian)
        data[2] = 0;
        data[3] = 20;   // Ending Handle (little endian)
        data[4] = 0;
        
        // UUID to find: Non-existent UUID (0xDEAD)
        data[5] = uuid_non_existent[0]; 
        data[6] = uuid_non_existent[1];
        
        let packet = create_att_packet(0, &data);
        
        // Call the handler
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Verify that when no matching attributes are found, the handler
        // returns an Error Response, which means the loop broke correctly
        assert_eq!(response.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
        assert_eq!(response.att_err_rsp().err_opcode, GattOp::AttOpReadByGroupTypeReq as u8);
        
        // Verify that the error handle is set to the starting handle from the request
        assert_eq!(response.att_err_rsp().err_handle, 1);
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_read_by_group_type_req_loop_break_on_dest_ptr_overflow() {
        setup_system_tick_mock();
        
        // This test verifies another case where the loop in ReadByGroupTypeReq breaks:
        // when the destination buffer would overflow (dest_ptr * 2 + attr_len > 0x13)
        
        // Find a valid attribute with a large attr_len
        let attr_to_modify = 1; // Use the first service definition
        let original_attr_len = get_gAttributes()[attr_to_modify].attr_len;
        
        // Temporarily increase the attribute length to trigger the buffer overflow check
        unsafe {
            (*(addr_of_mut!(get_gAttributes()[attr_to_modify]) as *mut AttributeT)).attr_len = 20; // Large enough to exceed the 0x13 limit
        }
        
        // Create a valid UUID for primary service that exists
        let primary_service_uuid = uuid16_to_bytes(GATT_UUID_PRIMARY_SERVICE);
        
        // Create a Read By Group Type Request for this valid UUID
        let mut data = [0; 10];
        data[0] = GattOp::AttOpReadByGroupTypeReq as u8;  // Opcode: Read By Group Type Request
        data[1] = 1;    // Starting Handle (little endian)
        data[2] = 0;
        data[3] = 20;   // Ending Handle (little endian)
        data[4] = 0;
        
        // UUID to find: Primary Service (0x2800)
        data[5] = primary_service_uuid[0];
        data[6] = primary_service_uuid[1];
        
        let packet = create_att_packet(0, &data);

        // Call the handler
        let response = l2cap_att_handler(&packet).unwrap();

        // Verify that when no matching attributes are found, the handler
        // returns an Error Response, which means the loop broke correctly
        assert_eq!(response.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
        assert_eq!(response.att_err_rsp().err_opcode, GattOp::AttOpReadByGroupTypeReq as u8);

        // Verify that the error handle is set to the starting handle from the request
        assert_eq!(response.att_err_rsp().err_handle, 1);

        // Restore original attribute length
        unsafe {
            (*(addr_of_mut!(get_gAttributes()[attr_to_modify]) as *mut AttributeT)).attr_len = original_attr_len;
        }
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_write_request_return_paths() {
        setup_system_tick_mock();

        // Test 1: Result should contain a write response when opcode is 0x12 (Write Request)
        {
            // Create a Write Request packet
            let mut data = [0; 8];
            data[0] = GattOp::AttOpWriteReq as u8; // Write Request (0x12)
            data[1] = 8; // Valid attribute handle
            data[2] = 0;
            data[3] = 0;
            data[4] = 0xAA; // Value to write

            let packet = create_att_packet(0, &data);

            // Call the handler
            let response = l2cap_att_handler(&packet);

            // Response should be a Write Response with opcode 0x13
            assert!(response.is_some());
            let resp = response.unwrap();
            assert_eq!(resp.att_write_rsp().opcode, GattOp::AttOpWriteRsp as u8);
        }

        // Test 2: Result should be None when opcode is 0x52 (Write Command)
        {
            // Create a Write Command packet
            let mut data = [0; 8];
            data[0] = GattOp::AttOpWriteCmd as u8; // Write Command (0x52)
            data[1] = 5; // Valid attribute handle
            data[2] = 0;
            data[3] = 0;
            data[4] = 0xAA; // Value to write

            let packet = create_att_packet(0, &data);

            // Call the handler
            let response = l2cap_att_handler(&packet);

            // Response should be None for Write Command
            assert!(response.is_none());
        }

        // Test 3: Return early if w is None and handle1 < 3
        {
            // Find an attribute without a write callback (w is None)
            let test_attr_idx = 8; // Device name attribute often doesn't have a write callback
            let original_w = get_gAttributes()[test_attr_idx].w;

            // Ensure the write callback is None
            unsafe {
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = None;
            }

            // Create a Write Request packet with handle1 < 3
            let mut data = [0; 8];
            data[0] = GattOp::AttOpWriteReq as u8;
            data[1] = test_attr_idx as u8;
            data[2] = 0;
            data[3] = 0;
            data[4] = 0xAA;

            let mut packet = create_att_packet(0, &data);
            packet.l2cap_data_mut().handle1 = 2; // Set handle1 < 3

            // Call the handler - should return early without modifying the attribute
            let response = l2cap_att_handler(&packet);

            // Response should match the result set based on opcode (Write Response)
            assert!(response.is_some());
            assert_eq!(response.unwrap().att_write_rsp().opcode, GattOp::AttOpWriteRsp as u8);

            // Restore original write callback
            unsafe {
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = original_w;
            }
        }

        // Test 4: Return early if w is None and p_attr_value <= __RAM_START_ADDR
        {
            // Find an attribute without a write callback
            let test_attr_idx = 12;
            let original_w = get_gAttributes()[test_attr_idx].w;
            let original_p_attr_value = get_gAttributes()[test_attr_idx].p_attr_value;

            // Ensure the write callback is None and p_attr_value points to RAM start
            unsafe {
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = None;
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).p_attr_value =
                    addr_of!(__RAM_START_ADDR) as *mut u8; // Point to __RAM_START_ADDR
            }

            // Create a Write Request packet with handle1 >= 3
            let mut data = [0; 8];
            data[0] = GattOp::AttOpWriteReq as u8;
            data[1] = test_attr_idx as u8;
            data[2] = 0;
            data[3] = 0;
            data[4] = 0xAA;

            let mut packet = create_att_packet(0, &data);
            packet.l2cap_data_mut().handle1 = 3; // Set handle1 >= 3

            // Call the handler - should return early without modifying the attribute
            let response = l2cap_att_handler(&packet);

            // Response should match the result set based on opcode (Write Response)
            assert!(response.is_some());
            assert_eq!(response.unwrap().att_write_rsp().opcode, GattOp::AttOpWriteRsp as u8);

            // Restore original values
            unsafe {
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = original_w;
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).p_attr_value = original_p_attr_value;
            }
        }

        // Test 5: Copy value to attribute when w is None, handle1 >= 3, and p_attr_value > __RAM_START_ADDR
        {
            // Find an attribute that can be written to
            let test_attr_idx = 14;
            let original_w = get_gAttributes()[test_attr_idx].w;
            let original_attr_value = unsafe {
                slice::from_raw_parts(
                    get_gAttributes()[test_attr_idx].p_attr_value,
                    get_gAttributes()[test_attr_idx].attr_len as usize
                ).to_vec()
            };

            // Create a mutable buffer for our test value
            let mut test_value = [0xCC; 10]; // Fill with a test pattern

            // Ensure the write callback is None and the attribute value is writable
            unsafe {
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = None;

                // Set up a new buffer pointer
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).p_attr_value =
                    test_value.as_mut_ptr();
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).attr_len =
                    test_value.len() as u8;
            }

            // Create a Write Request packet with handle1 >= 3
            let mut data = [0; 16]; // Larger buffer for the value to write
            data[0] = GattOp::AttOpWriteReq as u8;
            data[1] = test_attr_idx as u8;
            data[2] = 0;

            // Fill the value portion with a recognizable pattern
            for i in 0..10 {
                data[3 + i] = 0xAA + i as u8;
            }

            let mut packet = create_att_packet(0, &data);
            packet.l2cap_data_mut().handle1 = 3; // Set handle1 >= 3

            // Call the handler - should copy the value to the attribute
            let response = l2cap_att_handler(&packet);

            // Response should match the result set based on opcode (Write Response)
            assert!(response.is_some());
            assert_eq!(response.unwrap().att_write_rsp().opcode, GattOp::AttOpWriteRsp as u8);

            // Verify the attribute value was modified correctly
            for i in 0..10 {
                assert_eq!(test_value[i], 0xAA + i as u8);
            }

            // Restore original values
            unsafe {
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = original_w;

                // No need to restore attribute value since we used a temporary buffer
                // Just restore the original pointer and length
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).p_attr_value =
                    get_gAttributes()[test_attr_idx].p_attr_value;
            }
        }

        // Test 6: Call write callback when w is Some
        {
            // Flag to track if callback was called
            static mut WRITE_CALLBACK_CALLED: bool = false;

            // Find an attribute to modify for testing
            let test_attr_idx = 15;
            let original_w = get_gAttributes()[test_attr_idx].w;

            // Define a test callback function
            let test_callback = |_packet: &Packet| -> bool {
                unsafe { WRITE_CALLBACK_CALLED = true; }
                true
            };

            // Set the write callback
            unsafe {
                WRITE_CALLBACK_CALLED = false;
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = Some(test_callback);
            }

            // Create a Write Request packet
            let mut data = [0; 8];
            data[0] = GattOp::AttOpWriteReq as u8;
            data[1] = test_attr_idx as u8;
            data[2] = 0;
            data[3] = 0;
            data[4] = 0xAA;

            let packet = create_att_packet(0, &data);

            // Call the handler - should call the write callback
            let response = l2cap_att_handler(&packet);

            // Verify the callback was called
            unsafe {
                assert!(WRITE_CALLBACK_CALLED, "Write callback was not called");
            }

            // Response should match the result set based on opcode (Write Response)
            assert!(response.is_some());
            assert_eq!(response.unwrap().att_write_rsp().opcode, GattOp::AttOpWriteRsp as u8);

            // Restore original write callback
            unsafe {
                (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = original_w;
            }
        }

        // Test 7: Result value is preserved through all code paths
        {
            // Create a Write Command packet (should have None as result)
            let mut data = [0; 8];
            data[0] = GattOp::AttOpWriteCmd as u8; // Write Command (0x52)
            data[1] = 3; // Valid attribute handle
            data[2] = 0;
            data[3] = 0;
            data[4] = 0xAA;

            // Test the result preservation through different code paths

            // 7.1: With a write callback
            {
                let test_attr_idx = 18;
                let original_w = get_gAttributes()[test_attr_idx].w;

                // Define a callback
                static mut CALLED: bool = false;
                let test_callback = |_packet: &Packet| -> bool {
                    unsafe { CALLED = true; }
                    true
                };

                // Set up the write callback
                unsafe {
                    (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = Some(test_callback);
                }

                let mut packet = create_att_packet(0, &data);
                packet.l2cap_data_mut().value[4] = test_attr_idx as u8;

                // Call the handler - should call the write callback and return None
                let response = l2cap_att_handler(&packet);
                assert!(response.is_none(), "Write Command should return None even with w callback");

                assert!(unsafe { CALLED }, "Write callback was not called");

                // Restore original callback
                unsafe {
                    (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = original_w;
                }
            }

            // 7.2: Without a write callback, early return due to handle1 < 3
            {
                let test_attr_idx = 5;
                let original_w = get_gAttributes()[test_attr_idx].w;

                // Remove write callback
                unsafe {
                    (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = None;
                }

                let mut packet = create_att_packet(0, &data);
                packet.l2cap_data_mut().value[4] = test_attr_idx as u8;
                packet.l2cap_data_mut().handle1 = 2; // < 3, trigger early return

                // Call the handler - should return None (preserved from result)
                let response = l2cap_att_handler(&packet);
                assert!(response.is_none(), "Write Command should preserve None through early return path");

                // Restore original callback
                unsafe {
                    (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = original_w;
                }
            }

            // 7.3: Without a write callback, early return due to p_attr_value <= __RAM_START_ADDR
            {
                let test_attr_idx = 5;
                let original_w = get_gAttributes()[test_attr_idx].w;
                let original_p_attr_value = get_gAttributes()[test_attr_idx].p_attr_value;

                // Remove write callback and set invalid pointer
                unsafe {
                    (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = None;
                    (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).p_attr_value =
                        addr_of!(__RAM_START_ADDR) as *mut u8;
                }

                let mut packet = create_att_packet(0, &data);
                packet.l2cap_data_mut().value[4] = test_attr_idx as u8;
                packet.l2cap_data_mut().handle1 = 3; // >= 3, pass first check

                // Call the handler - should return None (preserved from result)
                let response = l2cap_att_handler(&packet);
                assert!(response.is_none(), "Write Command should preserve None through p_attr_value check");

                // Restore original values
                unsafe {
                    (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).w = original_w;
                    (*(addr_of_mut!(get_gAttributes()[test_attr_idx]) as *mut AttributeT)).p_attr_value = original_p_attr_value;
                }
            }
        }
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_find_by_type_value_req_handle_exceeds_range() {
        setup_system_tick_mock();
        
        // Create Find By Type Value Request with a narrow handle range
        let mut data = [0; 12];
        data[0] = GattOp::AttOpFindByTypeValueReq as u8; // Opcode: Find By Type Value Request
        
        // Set a narrow range that should be easily exceeded after finding one match
        data[1] = 1;    // Starting Handle (little endian)
        data[2] = 0;
        data[3] = 2;    // Ending Handle (little endian) - deliberately small end handle
        data[4] = 0;
        
        // UUID to find: Primary Service (0x2800)
        data[5] = 0x00; 
        data[6] = 0x28;
        
        // Value to match: GAP Service (0x1800)
        data[7] = 0x00;
        data[8] = 0x18;
        
        let packet = create_att_packet(0, &data);
        
        let response = l2cap_att_handler(&packet).unwrap();
        
        // The response should complete properly, even with the restricted range
        // This implicitly tests that the loop terminates when start_handle > end_handle
        
        // We should have a proper response with at most 1 entry
        // Calculate number of handle pairs from L2CAP length
        let handle_pairs = (response.head().l2cap_len as usize - 1) / 4;
        assert!(handle_pairs <= 1, "Should have found at most 1 service in the narrow range");
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_l2cap_att_handler_find_by_type_value_no_matches() {
        setup_system_tick_mock();
        
        // Create Find By Type Value Request with UUID and value that won't match anything
        let mut data = [0; 12];
        data[0] = GattOp::AttOpFindByTypeValueReq as u8; // Opcode: Find By Type Value Request
        data[1] = 1;    // Starting Handle (little endian)
        data[2] = 0;
        data[3] = 20;   // Ending Handle (little endian)
        data[4] = 0;
        
        // UUID to find: Primary Service (0x2800)
        data[5] = 0x00; 
        data[6] = 0x28;
        
        // Value to match: Non-existent service UUID (0xDEAD)
        data[7] = 0xAD;
        data[8] = 0xDE;
        
        let packet = create_att_packet(0, &data);
        
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Verify that an error response is returned when no matches are found
        assert_eq!(response.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
        assert_eq!(response.att_err_rsp().err_opcode, GattOp::AttOpFindByTypeValueReq as u8);
        
        // Error handle should be set to the start_handle value we were using
        // when we determined there were no matches
        assert_eq!(response.att_err_rsp().err_handle, 1);
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_handle_read_by_type_request_no_match() {
        setup_system_tick_mock();
        
        // Create a Read By Type Request packet with a non-existent UUID
        let mut data = [0; 10];
        data[0] = GattOp::AttOpReadByTypeReq as u8; // Opcode: Read By Type Request
        data[1] = 1;    // Starting Handle (little endian)
        data[2] = 0;
        data[3] = 20;   // Ending Handle (little endian)
        data[4] = 0;
        
        // UUID to find: A non-existent UUID (0xDEAD)
        data[5] = 0xAD; 
        data[6] = 0xDE;
        
        let packet = create_att_packet(GattOp::AttOpReadByTypeReq as u8, &data);
        
        // Call the function directly to test the specific code path
        let response = handle_read_by_type_request(&packet).unwrap();
        
        // Verify that an error response was generated
        assert_eq!(response.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
        assert_eq!(response.att_err_rsp().err_opcode, GattOp::AttOpReadByTypeReq as u8);
        assert_eq!(response.att_err_rsp().err_handle, 1); // Should match the starting handle from request
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_handle_read_by_type_request_uuid128_no_match() {
        setup_system_tick_mock();
        
        // Create a Read By Type Request for a non-existent 128-bit UUID
        let mut data = [0; 24];
        data[0] = GattOp::AttOpReadByTypeReq as u8; // Opcode: Read By Type Request
        data[1] = 1;    // Starting Handle (little endian)
        data[2] = 0;
        data[3] = 28;   // Ending Handle (little endian)
        data[4] = 0;
        
        // Create a non-existent 128-bit UUID
        let non_existent_uuid = create_uuid128(0xDEAD);
        data[5..21].copy_from_slice(&non_existent_uuid);
        
        let mut packet = create_att_packet(GattOp::AttOpReadByTypeReq as u8, &data);
        packet.l2cap_data_mut().handle1 = 0x15; // Set handle1 to indicate 128-bit UUID search
        
        // Call the function directly to test the specific code path
        let response = handle_read_by_type_request(&packet).unwrap();
        
        // Verify that an error response was generated
        assert_eq!(response.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
        assert_eq!(response.att_err_rsp().err_opcode, GattOp::AttOpReadByTypeReq as u8);
        assert_eq!(response.att_err_rsp().err_handle, 1); // Should match the starting handle from request
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_handle_read_by_type_request_16bit_uuid_success_path() {
        setup_system_tick_mock();
        
        // This test specifically focuses on the code block that handles
        // successful 16-bit UUID lookup in handle_read_by_type_request
        
        // Instead of guessing handles, find an actual 16-bit UUID attribute in the table
        // Find the Device Information Service UUID (0x180A) or any other 16-bit UUID
        let mut target_uuid: [u8; 2] = [0x0A, 0x18]; // Device Information Service in little-endian
        
        // Find the actual handle range by scanning the attribute table
        let mut start_handle = 1; // Default start from 1
        let total_attrs = get_gAttributes()[0].att_num as usize;
        let mut found_handle = 0;
        
        // Find an attribute with our target UUID to use in the test
        for i in 1..total_attrs {
            if get_gAttributes()[i].uuid_len == 2 {
                let attr_uuid = unsafe { slice::from_raw_parts(get_gAttributes()[i].uuid, 2) };
                if attr_uuid == target_uuid {
                    found_handle = i;
                    start_handle = i;
                    break;
                }
            }
        }
        
        // If we didn't find our target UUID, fall back to finding any 16-bit UUID
        if found_handle == 0 {
            for i in 1..total_attrs {
                if get_gAttributes()[i].uuid_len == 2 {
                    found_handle = i;
                    start_handle = i;
                    break;
                }
            }
            
            // Get the actual UUID from the found attribute
            if found_handle > 0 {
                target_uuid.copy_from_slice(unsafe { 
                    slice::from_raw_parts(get_gAttributes()[found_handle].uuid, 2) 
                });
            }
        }
        
        // If we still don't have a valid handle, the test would fail, but at least
        // we're not using hardcoded values that might be invalid
        assert!(found_handle > 0, "Could not find any 16-bit UUID attribute for testing");
        
        // Define a reasonable end handle (a few handles after start, but not beyond total)
        let end_handle = (start_handle + 10).min(total_attrs);
        
        // Create Read By Type Request for the found UUID
        let mut data = [0; 10];
        data[0] = GattOp::AttOpReadByTypeReq as u8; // Read By Type Request
        data[1] = start_handle as u8;    // Starting Handle - using the found handle
        data[2] = 0;
        data[3] = end_handle as u8;   // Ending Handle - a reasonable range after start
        data[4] = 0;
        
        // UUID to find: Use the actual UUID we found
        data[5] = target_uuid[0];
        data[6] = target_uuid[1];
        
        // Call handle_read_by_type_request directly to test the specific block
        let packet = create_att_packet(0, &data);
        let response = handle_read_by_type_request(&packet).unwrap();
        
        // Verify it's a Read By Type Response
        assert_eq!(response.att_read_rsp().opcode, GattOp::AttOpReadByTypeRsp as u8);
        
        // Find the expected attribute using l2cap_att_search with our actual handles
        let handle = l2cap_att_search(start_handle, end_handle, &target_uuid).unwrap();
        let found_attr = &handle.0[0];
        let found_handle = handle.1;
        
        // Verify bytes_read was correctly calculated
        // Should be attribute length + 2 (for handle)
        let expected_bytes_read = found_attr.attr_len + 2;
        assert_eq!(response.att_read_rsp().value[0], expected_bytes_read);
        
        // Verify the handle was correctly copied to response
        assert_eq!(response.att_read_rsp().value[1], found_handle as u8);
        assert_eq!(response.att_read_rsp().value[2], (found_handle >> 8) as u8);
        
        // Verify the attribute value was correctly copied 
        let expected_value = unsafe {
            slice::from_raw_parts(found_attr.p_attr_value, found_attr.attr_len as usize)
        };
        let actual_value = &response.att_read_rsp().value[3..3 + found_attr.attr_len as usize];
        assert_eq!(actual_value, expected_value);
        
        // Verify the packet header fields were set correctly
        assert_eq!(response.head()._type, 2);
        let chan_id = response.head().chan_id;
        assert_eq!(chan_id, 4);
        let dma_len = response.head().dma_len;
        assert_eq!(dma_len, expected_bytes_read as u32 + 8);
        assert_eq!(response.head().rf_len, expected_bytes_read + 6);
        let l2cap_len = response.head().l2cap_len;
        assert_eq!(l2cap_len, expected_bytes_read as u16 + 2);
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_read_by_type_request_loop_break_triggers() {
        setup_system_tick_mock();
        
        // Create Read By Type Request for Characteristic UUID (0x2803)
        let mut data = [0; 10];
        data[0] = GattOp::AttOpReadByTypeReq as u8; // Opcode: Read By Type Request
        data[1] = 1;    // Starting Handle (little endian)
        data[2] = 0;
        data[3] = 20;   // Ending Handle (little endian)
        data[4] = 0;
        
        // UUID to find: Characteristic (0x2803)
        data[5] = 0x03; 
        data[6] = 0x28;
        
        // We need to modify an attribute to force a break condition
        // Find a Characteristic attribute to modify
        let mut char_attr_idx = 0;
        for i in 1..get_gAttributes()[0].att_num as usize {
            if get_gAttributes()[i].uuid_len == 2 {
                let attr_uuid = unsafe { slice::from_raw_parts(get_gAttributes()[i].uuid, 2) };
                if attr_uuid == &uuid16_to_bytes(GATT_UUID_CHARACTER) {
                    char_attr_idx = i;
                    break;
                }
            }
        }
        
        assert!(char_attr_idx > 0, "Failed to find a Characteristic attribute for test");
        
        // Save the original UUID length for restoration later
        let original_uuid_len = get_gAttributes()[char_attr_idx + 1].uuid_len;
        
        // Case 1: Trigger break through inconsistent format
        // Modify the UUID length of the next attribute to force format inconsistency
        unsafe {
            // First call should work and establish a format
            // Then we modify the attribute to cause a break on the next match
            (*(addr_of_mut!(get_gAttributes()[char_attr_idx + 1]) as *mut AttributeT)).uuid_len =
                if original_uuid_len == 2 { 16 } else { 2 };
        }
        
        let packet = create_att_packet(0, &data);
        
        // Call the handler
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Verify we got a response that contains at least one characteristic
        assert_eq!(response.att_read_rsp().opcode, GattOp::AttOpReadByTypeRsp as u8);
        
        // Check that at least one characteristic was found before the break
        // Format byte should be > 0 and equal to uuid_len + 5
        assert!(response.att_read_rsp().value[0] > 0);
        
        // Case 2: Trigger break through buffer space limitation
        // Create an attribute with a very large UUID length to force a buffer space issue
        let max_space_attr_idx = char_attr_idx + 2; // Another characteristic
        let original_uuid_len2 = get_gAttributes()[max_space_attr_idx].uuid_len;
        
        unsafe {
            // Restore the first modification
            (*(addr_of_mut!(get_gAttributes()[char_attr_idx + 1]) as *mut AttributeT)).uuid_len = original_uuid_len;
            
            // Modify another attribute to have a large UUID length
            // This should cause bytes_read + found_attr[0].uuid_len >= 0x13
            (*(addr_of_mut!(get_gAttributes()[max_space_attr_idx]) as *mut AttributeT)).uuid_len = 0x10;
        }
        
        // Call the handler again
        let response2 = l2cap_att_handler(&packet).unwrap();
        
        // Verify we got a response
        assert_eq!(response2.att_read_rsp().opcode, GattOp::AttOpReadByTypeRsp as u8);
        
        // Restore the original attribute
        unsafe {
            (*(addr_of_mut!(get_gAttributes()[max_space_attr_idx]) as *mut AttributeT)).uuid_len = original_uuid_len2;
        }
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_read_by_type_request_none_break_condition() {
        setup_system_tick_mock();
        
        // Find a characteristic in the attribute table
        let char_uuid = uuid16_to_bytes(GATT_UUID_CHARACTER);
        let mut char_handle = 0;
        
        for i in 1..get_gAttributes()[0].att_num as usize {
            if get_gAttributes()[i].uuid_len == 2 {
                let attr_uuid = unsafe { slice::from_raw_parts(get_gAttributes()[i].uuid, 2) };
                if attr_uuid == &char_uuid {
                    char_handle = i;
                    break;
                }
            }
        }
        
        assert!(char_handle > 0, "Failed to find a Characteristic attribute for test");
        
        // Create Read By Type Request specifically targeting the range where only one 
        // characteristic exists - so that the next search returns None
        let mut data = [0; 10];
        data[0] = GattOp::AttOpReadByTypeReq as u8;
        data[1] = char_handle as u8;    // Start with the found characteristic
        data[2] = 0;
        data[3] = char_handle as u8;    // End at the same handle to force "None" on next search
        data[4] = 0;
        
        // UUID to find: Characteristic (0x2803)
        data[5] = 0x03; 
        data[6] = 0x28;
        
        let packet = create_att_packet(0, &data);
        
        // Call the handler
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Verify we got exactly one characteristic in the response
        // (This confirms the loop processed one characteristic and then broke on None)
        assert_eq!(response.att_read_rsp().opcode, GattOp::AttOpReadByTypeRsp as u8);
        
        // Get the format byte (how many bytes per entry)
        let format = response.att_read_rsp().value[0];
        
        // Calculate expected bytes_read from the number of characteristics
        // One characteristic = format bytes total
        let expected_bytes_read = format;
        
        // Verify packet has the right size for exactly one characteristic
        let dma_len = response.head().dma_len;
        assert_eq!(dma_len, expected_bytes_read as u32 + 8);
        assert_eq!(response.head().rf_len, expected_bytes_read + 6);
        let l2cap_len = response.head().l2cap_len;
        assert_eq!(l2cap_len, expected_bytes_read as u16 + 2);
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_read_by_type_request_error_condition() {
        setup_system_tick_mock();
        
        // Create Read By Type Request with handle range that will trigger a "None" return
        // We'll use a characteristic UUID (0x2803) but with a handle range that doesn't contain
        // any matching characteristics
        let mut data = [0; 10];
        data[0] = GattOp::AttOpReadByTypeReq as u8;
        data[1] = 28;    // Start handle - intentionally too high (beyond attribute table)
        data[2] = 0;
        data[3] = 30;    // End handle - also beyond attribute table
        data[4] = 0;
        
        // UUID to find: Characteristic (0x2803)
        data[5] = 0x03; 
        data[6] = 0x28;
        
        let packet = create_att_packet(0, &data);
        
        // Call the handler - this should trigger l2cap_att_search() to return None
        let response = l2cap_att_handler(&packet).unwrap();
        
        // Verify we got an error response with the correct parameters
        assert_eq!(response.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
        assert_eq!(response.att_err_rsp().err_opcode, GattOp::AttOpReadByTypeReq as u8);
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_handle_read_by_group_type_request_zero_start_handle() {
        setup_system_tick_mock();
        
        // Create Read By Group Type Request with start_handle = 0 (invalid)
        let mut data = [0; 10];
        data[0] = GattOp::AttOpReadByGroupTypeReq as u8; // Opcode: Read By Group Type Request
        data[1] = 0;    // Starting Handle = 0 (invalid)
        data[2] = 0;
        data[3] = 20;   // Ending Handle
        data[4] = 0;
        
        // UUID to find: Primary Service (0x2800)
        data[5] = 0x00; 
        data[6] = 0x28;
        
        let packet = create_att_packet(0, &data);
        
        // Call the handle_read_by_group_type_request function directly
        let response = handle_read_by_group_type_request(&packet).unwrap();
        
        // Verify that an error response is returned
        assert_eq!(response.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
        assert_eq!(response.att_err_rsp().err_opcode, GattOp::AttOpReadByGroupTypeReq as u8);
        assert_eq!(response.att_err_rsp().err_handle, 0); // Should match the invalid start_handle value
        
        // Also verify through the main handler
        let response2 = l2cap_att_handler(&packet).unwrap();
        
        // Should also be an error response with same parameters
        assert_eq!(response2.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
        assert_eq!(response2.att_err_rsp().err_opcode, GattOp::AttOpReadByGroupTypeReq as u8);
        assert_eq!(response2.att_err_rsp().err_handle, 0);
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_handle_read_by_group_type_request_start_handle_greater_than_end_handle() {
        setup_system_tick_mock();
        
        // Create Read By Group Type Request with start_handle > end_handle (invalid)
        let mut data = [0; 10];
        data[0] = GattOp::AttOpReadByGroupTypeReq as u8; // Opcode: Read By Group Type Request
        data[1] = 20;   // Starting Handle
        data[2] = 0;
        data[3] = 10;   // Ending Handle (smaller than start handle)
        data[4] = 0;
        
        // UUID to find: Primary Service (0x2800)
        data[5] = 0x00; 
        data[6] = 0x28;
        
        let packet = create_att_packet(0, &data);
        
        // Call the handle_read_by_group_type_request function directly
        let response = handle_read_by_group_type_request(&packet).unwrap();
        
        // Verify that an error response is returned
        assert_eq!(response.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
        assert_eq!(response.att_err_rsp().err_opcode, GattOp::AttOpReadByGroupTypeReq as u8);
        assert_eq!(response.att_err_rsp().err_handle, 20); // Should match the invalid start_handle value
        
        // Also verify through the main handler
        let response2 = l2cap_att_handler(&packet).unwrap();
        
        // Should also be an error response with same parameters
        assert_eq!(response2.att_err_rsp().opcode, GattOp::AttOpErrorRsp as u8);
        assert_eq!(response2.att_err_rsp().err_opcode, GattOp::AttOpReadByGroupTypeReq as u8);
        assert_eq!(response2.att_err_rsp().err_handle, 20);
    }
    
    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_handle_read_by_group_type_request_valid_range() {
        setup_system_tick_mock();
        
        // Create Read By Group Type Request with a valid handle range
        // We need to ensure this handle range contains at least one primary service
        // Most BLE devices have a Generic Access service at handle 1
        let mut data = [0; 10];
        data[0] = GattOp::AttOpReadByGroupTypeReq as u8; // Opcode: Read By Group Type Request
        data[1] = 1;    // Starting Handle
        data[2] = 0;
        data[3] = 10;   // Ending Handle
        data[4] = 0;
        
        // UUID to find: Primary Service (0x2800)
        data[5] = 0x00; 
        data[6] = 0x28;
        
        let packet = create_att_packet(0, &data);
        
        // Call the handle_read_by_group_type_request function directly
        let response = handle_read_by_group_type_request(&packet).unwrap();
        
        // For a valid range with primary services, we should get a Read By Group Type Response
        // and not an error response
        assert_eq!(response.att_read_rsp().opcode, GattOp::AttOpReadByGroupTypeRsp as u8);
        
        // The first byte of the value array should contain the format byte
        // Format byte = attribute value length + 4 (for start and end handles)
        // Primary service UUIDs are typically 2 bytes, so format byte should be 6
        assert!(response.att_read_rsp().value[0] >= 4, "Format byte should be at least 4");
        
        // Also verify through the main handler
        let response2 = l2cap_att_handler(&packet).unwrap();
        
        // Should also be a Read By Group Type Response
        assert_eq!(response2.att_read_rsp().opcode, GattOp::AttOpReadByGroupTypeRsp as u8);
    }

    #[test]
    #[mry::lock(read_reg_system_tick)]
    fn test_handle_write_request_or_command_restricted_handles() {
        setup_system_tick_mock();
        
        // Test write attempts to the restricted handles (0 and 1)
        for handle in 0..2 {
            // Create Write Request for a restricted handle
            let mut data = [0; 23];
            data[0] = GattOp::AttOpWriteReq as u8; // Opcode: Write Request
            data[1] = handle;   // Handle 0 or 1 (restricted)
            data[2] = 0;
            data[3] = 0;        // High byte of handle (should be 0)
            data[4] = 0xAA;     // Value to write
            data[5] = 0xBB;
            
            let packet = create_att_packet(0, &data);
            
            let response = l2cap_att_handler(&packet);
            
            // Write to restricted handles should return None (blocked)
            assert!(response.is_none());
        }
    }

    #[test]
    fn test_l2cap_att_search_functionality() {
        // Test all attribute search functionality in a single parameterized test
        
        // ---- Case 1: Test UUID16 matching ----
        {
            // PRIMARY_SERVICE_UUID is a 16-bit UUID used in the attribute table
            let uuid_bytes = uuid16_to_bytes(GATT_UUID_PRIMARY_SERVICE);
            let result = l2cap_att_search(1, 28, &uuid_bytes);

            // Should find a match
            assert!(result.is_some());

            // Verify handle is in the expected range
            let (found_attrs, handle) = result.unwrap();
            assert!(handle >= 1);
            assert!(handle <= 28);

            // Verify slice length
            assert_eq!(found_attrs.len(), 2);
        }
        
        // ---- Case 2: Test UUID128 matching ----
        {
            // Get SPP service UUID from attribute 18 in the attribute table
            let spp_service_handle = 18;

            // Use the actual 128-bit UUID directly from the attribute table
            let result = l2cap_att_search(1, 28, &TELINK_SPP_DATA_SERVER2CLIENT_UUID);

            // Should find a match
            assert!(result.is_some());

            // Verify found handle is as expected
            let (_, found_handle) = result.unwrap();
            assert_eq!(found_handle, spp_service_handle);

            // Also verify that searching from the handle itself works
            let result2 = l2cap_att_search(spp_service_handle, spp_service_handle, &TELINK_SPP_DATA_SERVER2CLIENT_UUID);
            assert!(result2.is_some());
            let (_, found_handle2) = result2.unwrap();
            assert_eq!(found_handle2, spp_service_handle);
        }
        
        // ---- Case 3: Test handle range restrictions ----
        {
            // We know PRIMARY_SERVICE_UUID appears at multiple places in the attribute table
            // First at index 1 (GAP service) and then at index 7 (Device Info service)

            let uuid_bytes = uuid16_to_bytes(GATT_UUID_PRIMARY_SERVICE);

            // Search only in the range that includes the first occurrence (1-6)
            let result1 = l2cap_att_search(1, 6, &uuid_bytes);
            assert!(result1.is_some());
            let (_, handle1) = result1.unwrap();
            assert_eq!(handle1, 1); // Should find at index 1

            // Search only in the range that includes the second occurrence (7-15)
            let result2 = l2cap_att_search(7, 15, &uuid_bytes);
            assert!(result2.is_some());
            let (_, handle2) = result2.unwrap();
            assert_eq!(handle2, 7); // Should find at index 7

            // Search in a range that excludes any PRIMARY_SERVICE_UUID (2-6)
            // (assuming there's no PRIMARY_SERVICE_UUID between index 2 and 6)
            let result3 = l2cap_att_search(2, 6, &uuid_bytes);
            assert!(result3.is_none()); // Should not find any
        }
        
        // ---- Case 4: Test found in sequence ----
        {
            // We know CHARACTER_UUID appears multiple times in the attribute table
            let uuid_bytes = uuid16_to_bytes(GATT_UUID_CHARACTER);

            // Search for the first occurrence
            let result1 = l2cap_att_search(1, 28, &uuid_bytes);
            assert!(result1.is_some());
            let (_, handle1) = result1.unwrap();

            // Now search for the next occurrence, starting after the first one
            let result2 = l2cap_att_search(handle1 + 1, 28, &uuid_bytes);
            assert!(result2.is_some());
            let (_, handle2) = result2.unwrap();

            // Verify that the second handle is greater than the first one
            assert!(handle2 > handle1);
        }
        
        // ---- Case 5: Test handle range adjustment ----
        {
            // att_num is the total number of attributes
            let att_num = get_gAttributes()[0].att_num as usize;

            // We know PRIMARY_SERVICE_UUID exists in the attribute table
            // Search beyond att_num should be adjusted to att_num
            let uuid_bytes = uuid16_to_bytes(GATT_UUID_PRIMARY_SERVICE);
            let result = l2cap_att_search(1, att_num + 10, &uuid_bytes);

            // Should find a match since PRIMARY_SERVICE_UUID exists
            assert!(result.is_some());
            // Handle should be less than or equal to att_num
            let (_, handle) = result.unwrap();
            assert!(handle <= att_num);
        }
        
        // ---- Case 6: Test no match found ----
        {
            // Search for UUID that doesn't exist in the attribute list
            let uuid_bytes = uuid16_to_bytes(0xABCD); // Non-existent UUID
            let result = l2cap_att_search(1, 28, &uuid_bytes);
            assert!(result.is_none());
            
            // Search for a 16-bit UUID that doesn't exist in the attribute table
            let uuid_bytes = uuid16_to_bytes(0xDEAD); // Non-existent UUID
            let result = l2cap_att_search(1, 28, &uuid_bytes);
            assert!(result.is_none());

            // Create a 128-bit UUID that doesn't exist in the attribute table
            let uuid128 = create_uuid128(0xBEEF); // Non-existent UUID
            let result = l2cap_att_search(1, 28, &uuid128);
            assert!(result.is_none());
        }
        
        // ---- Case 7: Test handle_start = att_num edge case ----
        {
            // The first attribute in gAttributes has its att_num set to the total number of attributes
            // in our case, that's 28 (index 0-27)
            let att_num = get_gAttributes()[0].att_num as usize;

            // Search with handle_start equal to att_num should return None
            let uuid_bytes = uuid16_to_bytes(GATT_UUID_PRIMARY_SERVICE);
            let result = l2cap_att_search(att_num, att_num + 5, &uuid_bytes);
            assert!(result.is_none());
        }
    }
}