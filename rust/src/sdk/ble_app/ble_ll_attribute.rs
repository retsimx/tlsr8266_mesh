use core::ptr::{addr_of, addr_of_mut};
use core::slice;

use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use crate::config::VENDOR_ID;

use crate::sdk::app_att_light::{AttributeT, get_gAttributes, SEND_TO_MASTER};
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

pub fn l2cap_att_handler(packet: &Packet) -> Option<Packet>
{
    if packet.l2cap_data().opcode & 3 == GattOp::AttOpExchangeMtuRsp as u8 {
        let handle = packet.l2cap_data().handle1;
        if handle == 0xc {

            ATT_SERVICE_DISCOVER_TICK.set(read_reg_system_tick() | 1);

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
        if handle != 8 {
            if handle == 2 {
                SLAVE_LINK_TIME_OUT.set(1000000);
                return None
            }

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

        ATT_SERVICE_DISCOVER_TICK.set(read_reg_system_tick() | 1);

        return Some(
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

    if *bytemuck::from_bytes::<u16>(&packet.l2cap_data().value[1..3]) != 4u16 {
        return None
    }

    let mut rf_packet_att_rsp = Packet {
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
    };

    return match FromPrimitive::from_u8(packet.l2cap_data().value[3]) {
        Some(GattOp::AttOpExchangeMtuReq) => {
            return Some(
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
        Some(GattOp::AttOpFindInfoReq) => {
            ATT_SERVICE_DISCOVER_TICK.set(read_reg_system_tick() | 1);
            let mut start_handle = packet.l2cap_data().value[4] as usize;
            let mut end_handle = packet.l2cap_data().value[6] as usize;
            if get_gAttributes()[0].att_num < packet.l2cap_data().value[6] {
                end_handle = get_gAttributes()[0].att_num as usize;
            }
            if start_handle <= end_handle {
                let mut uuid_len = 0;
                let mut handle = 1;
                let mut counter = 0;
                let mut offset_adj = 4;
                loop {
                    if uuid_len == 0 {
                        uuid_len = get_gAttributes()[start_handle].uuid_len;
                    } else if get_gAttributes()[start_handle].uuid_len != uuid_len {
                        if counter == 0 {
                            break;
                        }
                        
                        rf_packet_att_rsp.head_mut().l2cap_len = counter + 2;
                        rf_packet_att_rsp.head_mut().dma_len = counter as u32 + 8;
                        rf_packet_att_rsp.head_mut()._type = 2;
                        rf_packet_att_rsp.head_mut().rf_len = counter as u8 + 6;
                        rf_packet_att_rsp.head_mut().chan_id = 4;
                        rf_packet_att_rsp.att_read_rsp_mut().opcode = 5;
                        rf_packet_att_rsp.att_read_rsp_mut().value[0] = handle;
                        return Some(rf_packet_att_rsp)
                    }
                    rf_packet_att_rsp.att_read_rsp_mut().value[counter as usize + 1] = start_handle as u8;
                    rf_packet_att_rsp.att_read_rsp_mut().value[counter as usize + 2] = 0;
                    if get_gAttributes()[start_handle].uuid_len == 2 {
                        *bytemuck::from_bytes_mut(&mut rf_packet_att_rsp.att_read_rsp_mut().value[counter as usize + 3..counter as usize + 5]) = unsafe { *(get_gAttributes()[start_handle].uuid as *const u16) };
                        counter = counter + 4;
                    } else {
                        rf_packet_att_rsp.att_read_rsp_mut().value[counter as usize + 3..counter as usize + 3 + 0x10].copy_from_slice(
                            unsafe {
                                slice::from_raw_parts(
                                    get_gAttributes()[start_handle].uuid,
                                    0x10,
                                )
                            }
                        );

                        counter += 0x12;
                        handle = 2;
                        offset_adj = 0x10;
                    }
                    start_handle = start_handle + 1;
                    if end_handle < start_handle || 0x17 < offset_adj + counter {
                        rf_packet_att_rsp.head_mut().l2cap_len = counter + 2;
                        rf_packet_att_rsp.head_mut().dma_len = counter as u32 + 8;
                        rf_packet_att_rsp.head_mut()._type = 2;
                        rf_packet_att_rsp.head_mut().rf_len = counter as u8 + 6;
                        rf_packet_att_rsp.head_mut().chan_id = 4;
                        rf_packet_att_rsp.att_read_rsp_mut().opcode = 5;
                        rf_packet_att_rsp.att_read_rsp_mut().value[0] = handle;
                        return Some(rf_packet_att_rsp)
                    }
                }
            }

            let mut err = PKT_ERR_RSP;
            err.att_err_rsp_mut().err_opcode = GattOp::AttOpFindInfoReq as u8;
            err.att_err_rsp_mut().err_handle = start_handle as u16;
            Some(err)
        }
        Some(GattOp::AttOpFindByTypeValueReq) => {
            ATT_SERVICE_DISCOVER_TICK.set(read_reg_system_tick() | 1);
            let mut start_handle = packet.l2cap_data().value[4] as usize;
            let end_handle = packet.l2cap_data().value[6] as usize;
            let mut uuid = [0; 2];
            uuid.copy_from_slice(&packet.l2cap_data().value[8..10]);
            let value: u16 = *bytemuck::from_bytes(&packet.l2cap_data().value[10..12]);
            let mut counter = 0;
            loop {
                let handle = l2cap_att_search(start_handle, end_handle, &uuid);
                if handle == None || 9 < counter
                {
                    break;
                }
                let found_attr = &handle.unwrap().0[0];
                let found_handle = handle.unwrap().1;
                if found_attr.attr_len == 2 && unsafe { *(found_attr.p_attr_value as *const u16) } == value {
                    rf_packet_att_rsp.att_read_rsp_mut().value[counter * 2] = (found_handle & 0xff) as u8;
                    rf_packet_att_rsp.att_read_rsp_mut().value[counter * 2 + 1] = (found_handle >> 8) as u8;
                    start_handle = counter + 1;
                    rf_packet_att_rsp.att_read_rsp_mut().value[start_handle * 2] = (found_attr.att_num as usize + (found_handle - 1) & 0xff) as u8;
                    rf_packet_att_rsp.att_read_rsp_mut().value[start_handle * 2 + 1] = (found_attr.att_num as usize + (found_handle - 1) >> 8) as u8;
                    counter = start_handle + 1;
                    start_handle = found_handle + found_attr.att_num as usize;
                } else {
                    start_handle = found_handle + 1;
                }

                if start_handle > packet.l2cap_data().value[6] as usize {
                    break;
                }
            }
            if counter == 0 {
                let mut err = PKT_ERR_RSP;
                err.att_err_rsp_mut().err_opcode = GattOp::AttOpFindByTypeValueReq as u8;
                err.att_err_rsp_mut().err_handle = start_handle as u16;
                Some(err)
            } else {
                let c2 = counter * 2;
                rf_packet_att_rsp.head_mut().dma_len = c2 as u32 + 7;
                rf_packet_att_rsp.head_mut()._type = 2;
                rf_packet_att_rsp.head_mut().rf_len = c2 as u8 + 5;
                rf_packet_att_rsp.head_mut().l2cap_len = c2 as u16 + 1;
                rf_packet_att_rsp.head_mut().chan_id = 4;
                rf_packet_att_rsp.att_read_rsp_mut().opcode = 7;

                Some(rf_packet_att_rsp)
            }
        }
        Some(GattOp::AttOpReadByTypeReq) => {
            ATT_SERVICE_DISCOVER_TICK.set(read_reg_system_tick() | 1);
            let mut handle_start = packet.l2cap_data().value[4] as usize;
            let handle_end = packet.l2cap_data().value[6] as usize;
            let mut bytes_read = 0;
            let mut uuid_len = 0;
            let mut found_handle = handle_end;
            if unsafe { *(addr_of!(packet.l2cap_data().handle1) as *const u16) } == 0x15 {
                let mut uuid = [0; 16];
                uuid.copy_from_slice(&packet.l2cap_data().value[8..8 + 0x10]);
                uuid_len = 0x10;
                let handle = l2cap_att_search(handle_start, handle_end, &uuid);
                if handle == None
                {
                    rf_packet_att_rsp.att_read_rsp_mut().value[0] = 0;
                    bytes_read = 0;
                } else {
                    let found_attr = &handle.unwrap().0[0];
                    if (*found_attr).uuid_len != uuid_len {
                        return None
                    }
                    found_handle = handle.unwrap().1;

                    bytes_read = found_attr.attr_len + 2;

                    rf_packet_att_rsp.att_read_rsp_mut().value[1] = found_handle as u8;
                    rf_packet_att_rsp.att_read_rsp_mut().value[2] = (found_handle >> 8) as u8;
                    rf_packet_att_rsp.att_read_rsp_mut().value[3..3 + found_attr.attr_len as usize].copy_from_slice(unsafe { slice::from_raw_parts((*found_attr).p_attr_value, (*found_attr).attr_len as usize) });
                    rf_packet_att_rsp.att_read_rsp_mut().value[0] = bytes_read;
                }
            } else {
                let mut uuid = [0; 2];
                uuid.copy_from_slice(&packet.l2cap_data().value[8..=9]);
                if uuid[0] != 3 || uuid[1] != 0x28 {
                    uuid_len = 2;
                    let handle = l2cap_att_search(handle_start, handle_end, &uuid);
                    if handle == None
                    {
                        rf_packet_att_rsp.att_read_rsp_mut().value[0] = 0;
                        bytes_read = 0;
                    } else {
                        let found_attr = &handle.unwrap().0[0];
                        if (*found_attr).uuid_len != uuid_len {
                            return None
                        }
                        found_handle = handle.unwrap().1;

                        bytes_read = (*found_attr).attr_len + 2;

                        rf_packet_att_rsp.att_read_rsp_mut().value[1] = found_handle as u8;
                        rf_packet_att_rsp.att_read_rsp_mut().value[2] = (found_handle >> 8) as u8;
                        rf_packet_att_rsp.att_read_rsp_mut().value[3..3 + found_attr.attr_len as usize].copy_from_slice(unsafe { slice::from_raw_parts((*found_attr).p_attr_value, (*found_attr).attr_len as usize) });
                        rf_packet_att_rsp.att_read_rsp_mut().value[0] = bytes_read;
                    }
                }

                let mut counter = 0;
                bytes_read = 0;

                loop {
                    let handle = l2cap_att_search(handle_start, handle_end, &uuid);
                    if handle == None
                    {
                        break;
                    }
                    let found_attr = handle.unwrap().0;
                    found_handle = handle.unwrap().1;

                    if !((counter == 0 || found_attr[1].uuid_len == counter) && bytes_read + found_attr[0].uuid_len < 0x13) {
                        break;
                    }

                    rf_packet_att_rsp.att_read_rsp_mut().value[bytes_read as usize + 1] = found_handle as u8;
                    bytes_read = bytes_read + 1;
                    rf_packet_att_rsp.att_read_rsp_mut().value[bytes_read as usize + 1] = 0;
                    bytes_read = bytes_read + 1;
                    rf_packet_att_rsp.att_read_rsp_mut().value[bytes_read as usize + 1] = unsafe { *(found_attr[0]).p_attr_value };
                    bytes_read = bytes_read + 1;
                    rf_packet_att_rsp.att_read_rsp_mut().value[bytes_read as usize + 1] = found_handle as u8 + 1;
                    bytes_read = bytes_read + 1;
                    handle_start = found_handle + 2;
                    rf_packet_att_rsp.att_read_rsp_mut().value[bytes_read as usize + 1] = 0;
                    bytes_read = bytes_read + 1;

                    rf_packet_att_rsp.att_read_rsp_mut().value[bytes_read as usize + 1..bytes_read as usize + 1 + (found_attr[1]).uuid_len as usize].copy_from_slice(
                        unsafe {
                            slice::from_raw_parts(
                                found_attr[1].uuid,
                                found_attr[1].uuid_len as usize,
                            )
                        }
                    );

                    counter = found_attr[1].uuid_len;
                    bytes_read = counter + bytes_read;
                }
                rf_packet_att_rsp.att_read_rsp_mut().value[0] = counter + 5;
            }
            if bytes_read == 0 {
                let mut err = PKT_ERR_RSP;
                err.att_err_rsp_mut().err_opcode = GattOp::AttOpReadByTypeReq as u8;
                err.att_err_rsp_mut().err_handle = found_handle as u16;
                Some(err)
            } else {
                rf_packet_att_rsp.head_mut().dma_len = bytes_read as u32 + 8;
                rf_packet_att_rsp.head_mut()._type = 2;
                rf_packet_att_rsp.head_mut().rf_len = bytes_read + 6;
                rf_packet_att_rsp.head_mut().l2cap_len = bytes_read as u16 + 2;
                rf_packet_att_rsp.head_mut().chan_id = 4;
                rf_packet_att_rsp.att_read_rsp_mut().opcode = 9;

                Some(rf_packet_att_rsp)
            }
        }
        Some(GattOp::AttOpReadReq) => {
            let att_num = packet.l2cap_data().value[4] as usize;

            if packet.l2cap_data().value[5] != 0 {
                return None
            }
            if get_gAttributes()[0].att_num < att_num as u8 {
                return None
            }
            if get_gAttributes()[att_num].r.is_none() {
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

                if get_gAttributes()[att_num].p_attr_value == unsafe { SEND_TO_MASTER.as_mut_ptr() } {
                    unsafe { SEND_TO_MASTER.fill(0); }
                } else if att_num == 0x18 && *RF_SLAVE_OTA_FINISHED_FLAG.lock() != OtaState::Continue {
                    RF_SLAVE_OTA_TERMINATE_FLAG.set(true);
                }
                rf_packet_att_rsp.head_mut().rf_len = get_gAttributes()[att_num].attr_len + 5;
                rf_packet_att_rsp.head_mut().dma_len = rf_packet_att_rsp.head_mut().rf_len as u32 + 2;
                rf_packet_att_rsp.head_mut()._type = 2;
                rf_packet_att_rsp.head_mut().l2cap_len = get_gAttributes()[att_num].attr_len as u16 + 1;
                rf_packet_att_rsp.head_mut().chan_id = 4;
                rf_packet_att_rsp.att_read_rsp_mut().opcode = 0xb;
                return Some(rf_packet_att_rsp)
            }

            get_gAttributes()[att_num].r.unwrap()(packet);

            None
        }
        Some(GattOp::AttOpReadByGroupTypeReq) => {
            ATT_SERVICE_DISCOVER_TICK.set(read_reg_system_tick() | 1);
            let mut handle_start = packet.l2cap_data().value[4] as usize;
            let handle_end = packet.l2cap_data().value[6] as usize;
            let mut uuid = [0; 2];
            uuid.copy_from_slice(&packet.l2cap_data().value[8..=9]);
            let mut dest_ptr = 0;
            let mut counter = 0;
            loop {
                let handle = l2cap_att_search(handle_start, handle_end, &uuid);
                if handle == None {
                    break;
                }

                let found_attr = &handle.unwrap().0[0];

                let mut attr_len = 0;
                if counter == 0 {
                    attr_len = found_attr.attr_len as usize;
                } else {
                    attr_len = found_attr.attr_len as usize;
                    if attr_len != counter {
                        break;
                    }
                }
                if 0x13 < attr_len + dest_ptr * 2 {
                    break;
                }
                counter = handle.unwrap().1;
                rf_packet_att_rsp.att_read_rsp_mut().value[dest_ptr * 2 + 1] = (counter & 0xff) as u8;
                rf_packet_att_rsp.att_read_rsp_mut().value[dest_ptr * 2 + 2] = (counter >> 8) as u8;

                handle_start = dest_ptr + 1;
                rf_packet_att_rsp.att_read_rsp_mut().value[(handle_start * 2) + 1] = (((counter - 1) + (*found_attr).att_num as usize) & 0xff) as u8;
                rf_packet_att_rsp.att_read_rsp_mut().value[(handle_start * 2) + 2] = (((counter - 1) + (*found_attr).att_num as usize) >> 8) as u8;

                handle_start = handle_start + 1;
                rf_packet_att_rsp.att_read_rsp_mut().value[(handle_start * 2) + 1..(handle_start * 2) + (*found_attr).attr_len as usize + 1].copy_from_slice(
                    unsafe {
                        slice::from_raw_parts(
                            found_attr.p_attr_value,
                            found_attr.attr_len as usize,
                        )
                    }
                );
                dest_ptr = handle_start + ((*found_attr).attr_len as usize / 2);
                counter = counter + (*found_attr).att_num as usize;
                handle_start = counter;
                counter = attr_len;
                if handle_start > handle_end {
                    break;
                }
            }
            if dest_ptr == 0 {
                let mut err = PKT_ERR_RSP;
                err.att_err_rsp_mut().err_opcode = GattOp::AttOpReadByGroupTypeReq as u8;
                err.att_err_rsp_mut().err_handle = packet.l2cap_data().value[4] as u16;
                Some(err)
            } else {
                rf_packet_att_rsp.head_mut().dma_len = (dest_ptr as u32 + 4) * 2;
                rf_packet_att_rsp.head_mut()._type = 2;
                rf_packet_att_rsp.head_mut().rf_len = rf_packet_att_rsp.head_mut().dma_len as u8 - 2;
                rf_packet_att_rsp.head_mut().l2cap_len = rf_packet_att_rsp.head_mut().dma_len as u16 - 6;
                rf_packet_att_rsp.head_mut().chan_id = 4;
                rf_packet_att_rsp.att_read_rsp_mut().opcode = 0x11;
                rf_packet_att_rsp.att_read_rsp_mut().value[0] = counter as u8 + 4;

                Some(rf_packet_att_rsp)
            }
        }
        Some(GattOp::AttOpWriteReq) | Some(GattOp::AttOpWriteCmd) => {
            let att_num = packet.l2cap_data().value[4] as usize;

            let mut result;
            if unsafe { *(get_gAttributes()[att_num].uuid as *const u16) } != 0x2902 {
                if att_num < 2 {
                    return None
                }
                if unsafe { *get_gAttributes()[att_num - 1].p_attr_value } & 0xc == 0 {
                    return None
                }
            }

            result = None;
            if packet.l2cap_data().value[5] != 0 {
                return result;
            }
            if (get_gAttributes()[0].att_num as usize) < att_num {
                return result;
            }
            if packet.l2cap_data().value[3] == 0x12 {
                result = Some(
                    Packet {
                        att_write_rsp: PacketAttWriteRsp {
                            head: PacketL2capHead {
                                dma_len: 0x07,
                                _type: 2,
                                rf_len: 0x05,
                                l2cap_len: 0x01,
                                chan_id: 0x04,
                            },
                            opcode: 0x13,
                        }
                    }
                )
            }
            if get_gAttributes()[att_num].w.is_none() {
                if unsafe { *(addr_of!(packet.l2cap_data().handle1) as *const u16) } < 3 {
                    return result;
                }

                if get_gAttributes()[att_num].p_attr_value as u32 <= unsafe { addr_of!(__RAM_START_ADDR) } as u32 {
                    return result;
                }

                let dest = unsafe {
                    slice::from_raw_parts_mut(
                        get_gAttributes()[att_num].p_attr_value,
                        get_gAttributes()[att_num].attr_len as usize,
                    )
                };

                dest.fill(0);
                dest.copy_from_slice(
                    unsafe {
                        slice::from_raw_parts(
                            addr_of!(packet.l2cap_data().value[6]),
                            get_gAttributes()[att_num].attr_len as usize,
                        )
                    }
                );

                return result;
            }

            get_gAttributes()[att_num].w.unwrap()(packet);

            result
        }
        _ => None
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdk::app_att_light::{GATT_UUID_CHARACTER, GATT_UUID_PRIMARY_SERVICE, get_gAttributes, TELINK_SPP_DATA_SERVER2CLIENT_UUID};

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

    #[test]
    fn test_l2cap_att_search_not_found() {
        // Search for UUID that doesn't exist in the attribute list
        let uuid_bytes = uuid16_to_bytes(0xABCD); // Non-existent UUID
        let result = l2cap_att_search(1, 28, &uuid_bytes);
        assert!(result.is_none());
    }

    #[test]
    fn test_l2cap_att_search_handle_start_equals_att_num() {
        // The first attribute in gAttributes has its att_num set to the total number of attributes
        // in our case, that's 28 (index 0-27)
        let att_num = get_gAttributes()[0].att_num as usize;

        // Search with handle_start equal to att_num should return None
        let uuid_bytes = uuid16_to_bytes(GATT_UUID_PRIMARY_SERVICE);
        let result = l2cap_att_search(att_num, att_num + 5, &uuid_bytes);
        assert!(result.is_none());
    }

    #[test]
    fn test_l2cap_att_search_handle_end_adjustment() {
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

    #[test]
    fn test_l2cap_att_search_uuid16_match() {
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

    #[test]
    fn test_l2cap_att_search_uuid128_match() {
        // Get SPP service UUID from attribute 18 in the attribute table
        // This is a 128-bit UUID (16 bytes)
        let spp_service_handle = 18;

        // Use the actual 128-bit UUID directly from the attribute table
        // TELINK_SPP_DATA_SERVER2CLIENT_UUID is defined in app_att_light.rs
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

    #[test]
    fn test_l2cap_att_search_handle_range() {
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

    #[test]
    fn test_l2cap_att_search_first_match() {
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

    #[test]
    fn test_l2cap_att_search_uuid16_no_match() {
        // Search for a 16-bit UUID that doesn't exist in the attribute table
        let uuid_bytes = uuid16_to_bytes(0xDEAD); // Non-existent UUID
        let result = l2cap_att_search(1, 28, &uuid_bytes);

        // Should not find a match
        assert!(result.is_none());
    }

    #[test]
    fn test_l2cap_att_search_uuid128_no_match() {
        // Create a 128-bit UUID that doesn't exist in the attribute table
        let uuid128 = create_uuid128(0xBEEF); // Non-existent UUID
        let result = l2cap_att_search(1, 28, &uuid128);

        // Should not find a match
        assert!(result.is_none());
    }
}