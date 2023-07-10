use core::mem::size_of_val;
use core::ptr::{addr_of, addr_of_mut, null};
use core::slice;

use num_derive::FromPrimitive;
use num_traits::FromPrimitive;

use crate::sdk::app_att_light::{AttributeT, get_gAttributes, SEND_TO_MASTER};
use crate::sdk::light::{OtaState, PacketAttWrite, PacketL2capData, PacketL2capHead};
use crate::sdk::mcu::register::read_reg_system_tick;
use crate::state::State;

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

pub unsafe fn l2cap_att_search(mut handle_start: usize, handle_end: usize, uuid: &[u8]) -> Option<(*const AttributeT, usize)>
{
    let att_num = get_gAttributes()[0].att_num as usize;

    if att_num != handle_start {
        let mut end = handle_end;
        if att_num < handle_end {
            end = att_num;
        }

        for handle_start in handle_start..=end {
            if get_gAttributes()[handle_start].uuid_len == 2 {
                if *(uuid.as_ptr() as *const u16) == *(get_gAttributes()[handle_start].uuid as *const u16) {
                    return Some((&get_gAttributes()[handle_start], handle_start));
                }
            } else {
                if uuid == slice::from_raw_parts(get_gAttributes()[handle_start].uuid, 0x10) {
                    return Some((&get_gAttributes()[handle_start], handle_start));
                }
            }
        }
    }

    return None;
}

pub unsafe fn l2cap_att_handler(state: &mut State, mut packet: *const PacketL2capData) -> (*const PacketL2capHead, usize)
{
    if (*packet).opcode & 3 == GattOp::AttOpExchangeMtuRsp as u8 {
        let handle = (*packet).handle1;
        if handle == 0xc {
            state.pkt_version_ind.rf_len = 6;
            state.att_service_discover_tick = read_reg_system_tick() | 1;
            return (addr_of!(state.pkt_version_ind) as *const PacketL2capHead, size_of_val(&state.pkt_version_ind));
        }
        if handle != 8 {
            if handle == 2 {
                state.slave_link_time_out = 1000000;
                return (null(), 0);
            }
            state.rf_pkt_unknown_response.data[0] = handle;
            return (addr_of!(state.rf_pkt_unknown_response) as *const PacketL2capHead, size_of_val(&state.rf_pkt_unknown_response));
        }
        state.pkt_feature_rsp.rf_len = 9;
        state.att_service_discover_tick = read_reg_system_tick() | 1;
        return (addr_of!(state.pkt_feature_rsp) as *const PacketL2capHead, size_of_val(&state.pkt_feature_rsp));
    }

    if *(addr_of!((*packet).value[1]) as *const u16) != 4 {
        return (null(), 0);
    }

    return match FromPrimitive::from_u8((*packet).value[3]) {
        Some(GattOp::AttOpExchangeMtuReq) => {
            return (addr_of!(state.pkt_mtu_rsp) as *const PacketL2capHead, size_of_val(&state.pkt_mtu_rsp));
        }
        Some(GattOp::AttOpFindInfoReq) => {
            state.att_service_discover_tick = read_reg_system_tick() | 1;
            let mut start_handle = (*packet).value[4] as usize;
            let mut end_handle = (*packet).value[6] as usize;
            if get_gAttributes()[0].att_num < (*packet).value[6] {
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

                        state.rf_packet_att_rsp.l2cap_len = counter + 2;
                        state.rf_packet_att_rsp.dma_len = counter as u32 + 8;
                        state.rf_packet_att_rsp._type = 2;
                        state.rf_packet_att_rsp.rf_len = counter as u8 + 6;
                        state.rf_packet_att_rsp.chan_id = 4;
                        state.rf_packet_att_rsp.opcode = 5;
                        state.rf_packet_att_rsp.value[0] = handle;
                        return (addr_of!(state.rf_packet_att_rsp) as *const PacketL2capHead, size_of_val(&state.rf_packet_att_rsp));
                    }
                    state.rf_packet_att_rsp.value[counter as usize + 1] = start_handle as u8;
                    state.rf_packet_att_rsp.value[counter as usize + 2] = 0;
                    if get_gAttributes()[start_handle].uuid_len == 2 {
                        *(addr_of!(state.rf_packet_att_rsp.value[counter as usize + 3]) as *mut u16) = *(get_gAttributes()[start_handle].uuid as *const u16);
                        counter = counter + 4;
                    } else {
                        state.rf_packet_att_rsp.value[counter as usize + 3..counter as usize + 3 + 0x10].copy_from_slice(
                            slice::from_raw_parts(
                                get_gAttributes()[start_handle].uuid as *const u8,
                                0x10,
                            )
                        );

                        counter += 0x12;
                        handle = 2;
                        offset_adj = 0x10;
                    }
                    start_handle = start_handle + 1;
                    if end_handle < start_handle || 0x17 < offset_adj + counter {
                        state.rf_packet_att_rsp.l2cap_len = counter + 2;
                        state.rf_packet_att_rsp.dma_len = counter as u32 + 8;
                        state.rf_packet_att_rsp._type = 2;
                        state.rf_packet_att_rsp.rf_len = counter as u8 + 6;
                        state.rf_packet_att_rsp.chan_id = 4;
                        state.rf_packet_att_rsp.opcode = 5;
                        state.rf_packet_att_rsp.value[0] = handle;
                        return (addr_of!(state.rf_packet_att_rsp) as *const PacketL2capHead, size_of_val(&state.rf_packet_att_rsp));
                    }
                }
            }
            state.pkt_err_rsp.err_opcode = 4;
            state.pkt_err_rsp.err_handle = start_handle as u16;
            (addr_of!(state.pkt_err_rsp) as *const PacketL2capHead, size_of_val(&state.pkt_err_rsp))
        }
        Some(GattOp::AttOpFindByTypeValueReq) => {
            state.att_service_discover_tick = read_reg_system_tick() | 1;
            let mut start_handle = (*packet).value[4] as usize;
            let end_handle = (*packet).value[6] as usize;
            let mut uuid = [0; 2];
            uuid.copy_from_slice(&(*packet).value[8..=9]);
            let value = *(addr_of!((*packet).value[10]) as *const u16);
            let mut counter = 0;
            loop {
                let handle = l2cap_att_search(start_handle, end_handle, &uuid);
                if handle == None || 9 < counter
                {
                    break;
                }
                let found_attr = handle.unwrap().0;
                let found_handle = handle.unwrap().1;
                if (*found_attr).attr_len == 2 && *((*found_attr).p_attr_value as *const u16) == value {
                    state.rf_packet_att_rsp.value[counter * 2] = (found_handle & 0xff) as u8;
                    state.rf_packet_att_rsp.value[counter * 2 + 1] = (found_handle >> 8) as u8;
                    start_handle = counter + 1;
                    state.rf_packet_att_rsp.value[start_handle * 2] = ((*found_attr).att_num as usize + (found_handle - 1) & 0xff) as u8;
                    state.rf_packet_att_rsp.value[start_handle * 2 + 1] = ((*found_attr).att_num as usize + (found_handle - 1) >> 8) as u8;
                    counter = start_handle + 1;
                    start_handle = found_handle + (*found_attr).att_num as usize;
                } else {
                    start_handle = found_handle + 1;
                }

                if start_handle > (*packet).value[6] as usize {
                    break;
                }
            }
            if counter == 0 {
                state.pkt_err_rsp.err_opcode = 6;
                state.pkt_err_rsp.err_handle = start_handle as u16;

                (addr_of!(state.pkt_err_rsp) as *const PacketL2capHead, size_of_val(&state.pkt_err_rsp))
            } else {
                let c2 = counter * 2;
                state.rf_packet_att_rsp.dma_len = c2 as u32 + 7;
                state.rf_packet_att_rsp._type = 2;
                state.rf_packet_att_rsp.rf_len = c2 as u8 + 5;
                state.rf_packet_att_rsp.l2cap_len = c2 as u16 + 1;
                state.rf_packet_att_rsp.chan_id = 4;
                state.rf_packet_att_rsp.opcode = 7;

                (addr_of!(state.rf_packet_att_rsp) as *const PacketL2capHead, size_of_val(&state.rf_packet_att_rsp))
            }
        }
        Some(GattOp::AttOpReadByTypeReq) => {
            state.att_service_discover_tick = read_reg_system_tick() | 1;
            let mut handle_start = (*packet).value[4] as usize;
            let handle_end = (*packet).value[6] as usize;
            let mut bytes_read = 0;
            let mut uuid_len = 0;
            let mut found_handle = handle_end;
            if *(addr_of!((*packet).handle1) as *const u16) == 0x15 {
                let mut uuid = [0; 16];
                uuid.copy_from_slice(&(*packet).value[8..8 + 0x10]);
                uuid_len = 0x10;
                let handle = l2cap_att_search(handle_start, handle_end, &uuid);
                if handle == None
                {
                    state.rf_packet_att_rsp.value[0] = 0;
                    bytes_read = 0;
                } else {
                    let found_attr = handle.unwrap().0;
                    if (*found_attr).uuid_len != uuid_len {
                        return (null(), 0);
                    }
                    found_handle = handle.unwrap().1;
                    state.rf_packet_att_rsp.value[1] = found_handle as u8;
                    state.rf_packet_att_rsp.value[2] = (found_handle >> 8) as u8;
                    state.rf_packet_att_rsp.value[3..3 + (*found_attr).attr_len as usize].copy_from_slice(slice::from_raw_parts((*found_attr).p_attr_value, (*found_attr).attr_len as usize));
                    bytes_read = (*found_attr).attr_len + 2;
                    state.rf_packet_att_rsp.value[0] = bytes_read;
                }
            } else {
                let mut uuid = [0; 2];
                uuid.copy_from_slice(&(*packet).value[8..=9]);
                if uuid[0] != 3 || uuid[1] != 0x28 {
                    uuid_len = 2;
                    let handle = l2cap_att_search(handle_start, handle_end, &uuid);
                    if handle == None
                    {
                        state.rf_packet_att_rsp.value[0] = 0;
                        bytes_read = 0;
                    } else {
                        let found_attr = handle.unwrap().0;
                        if (*found_attr).uuid_len != uuid_len {
                            return (null(), 0);
                        }
                        found_handle = handle.unwrap().1;
                        state.rf_packet_att_rsp.value[1] = found_handle as u8;
                        state.rf_packet_att_rsp.value[2] = (found_handle >> 8) as u8;
                        state.rf_packet_att_rsp.value[3..3 + (*found_attr).attr_len as usize].copy_from_slice(slice::from_raw_parts((*found_attr).p_attr_value, (*found_attr).attr_len as usize));
                        bytes_read = (*found_attr).attr_len + 2;
                        state.rf_packet_att_rsp.value[0] = bytes_read;
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

                    if !((counter == 0 || (*found_attr.offset(1)).uuid_len == counter) && bytes_read + (*found_attr).uuid_len < 0x13) {
                        break;
                    }

                    state.rf_packet_att_rsp.value[bytes_read as usize + 1] = found_handle as u8;
                    bytes_read = bytes_read + 1;
                    state.rf_packet_att_rsp.value[bytes_read as usize + 1] = 0;
                    bytes_read = bytes_read + 1;
                    state.rf_packet_att_rsp.value[bytes_read as usize + 1] = *(*found_attr).p_attr_value;
                    bytes_read = bytes_read + 1;
                    state.rf_packet_att_rsp.value[bytes_read as usize + 1] = found_handle as u8 + 1;
                    bytes_read = bytes_read + 1;
                    handle_start = found_handle + 2;
                    state.rf_packet_att_rsp.value[bytes_read as usize + 1] = 0;
                    bytes_read = bytes_read + 1;

                    state.rf_packet_att_rsp.value[bytes_read as usize + 1..bytes_read as usize + 1 + (*found_attr.offset(1)).uuid_len as usize].copy_from_slice(
                        slice::from_raw_parts(
                            (*found_attr.offset(1)).uuid,
                            (*found_attr.offset(1)).uuid_len as usize,
                        )
                    );

                    counter = (*found_attr.offset(1)).uuid_len;
                    bytes_read = counter + bytes_read;
                }
                state.rf_packet_att_rsp.value[0] = counter + 5;
            }
            if bytes_read == 0 {
                state.pkt_err_rsp.err_opcode = 8;
                state.pkt_err_rsp.err_handle = found_handle as u16;
                return (addr_of!(state.pkt_err_rsp) as *const PacketL2capHead, size_of_val(&state.pkt_err_rsp));
            } else {
                state.rf_packet_att_rsp.dma_len = bytes_read as u32 + 8;
                state.rf_packet_att_rsp._type = 2;
                state.rf_packet_att_rsp.rf_len = bytes_read + 6;
                state.rf_packet_att_rsp.l2cap_len = bytes_read as u16 + 2;
                state.rf_packet_att_rsp.chan_id = 4;
                state.rf_packet_att_rsp.opcode = 9;

                (addr_of!(state.rf_packet_att_rsp) as *const PacketL2capHead, size_of_val(&state.rf_packet_att_rsp))
            }
        }
        Some(GattOp::AttOpReadReq) => {
            let att_num = (*packet).value[4] as usize;

            if (*packet).value[5] != 0 {
                return (null(), 0);
            }
            if get_gAttributes()[0].att_num < att_num as u8 {
                return (null(), 0);
            }
            if get_gAttributes()[att_num].r.is_none() {
                slice::from_raw_parts_mut(
                    addr_of_mut!(state.rf_packet_att_rsp.value[0]) as *mut u8,
                    get_gAttributes()[att_num].attr_len as usize,
                ).copy_from_slice(
                    slice::from_raw_parts(
                        get_gAttributes()[att_num].p_attr_value,
                        get_gAttributes()[att_num].attr_len as usize,
                    )
                );

                if get_gAttributes()[att_num].p_attr_value == unsafe { SEND_TO_MASTER.as_mut_ptr() } {
                    unsafe { SEND_TO_MASTER.fill(0); }
                } else if att_num == 0x18 && state.rf_slave_ota_finished_flag != OtaState::Continue {
                    state.rf_slave_ota_terminate_flag = true;
                }
                state.rf_packet_att_rsp.rf_len = get_gAttributes()[att_num].attr_len + 5;
                state.rf_packet_att_rsp.dma_len = state.rf_packet_att_rsp.rf_len as u32 + 2;
                state.rf_packet_att_rsp._type = 2;
                state.rf_packet_att_rsp.l2cap_len = get_gAttributes()[att_num].attr_len as u16 + 1;
                state.rf_packet_att_rsp.chan_id = 4;
                state.rf_packet_att_rsp.opcode = 0xb;
                return (addr_of!(state.rf_packet_att_rsp) as *const PacketL2capHead, size_of_val(&state.rf_packet_att_rsp));
            }

            get_gAttributes()[att_num].r.unwrap()(state, unsafe { &mut *(packet as *mut PacketAttWrite) });

            (null(), 0)
        }
        Some(GattOp::AttOpReadByGroupTypeReq) => {
            state.att_service_discover_tick = read_reg_system_tick() | 1;
            let mut handle_start = (*packet).value[4] as usize;
            let handle_end = (*packet).value[6] as usize;
            let mut uuid = [0; 2];
            uuid.copy_from_slice(&(*packet).value[8..=9]);
            let mut dest_ptr = 0;
            let mut counter = 0;
            loop {
                let handle = l2cap_att_search(handle_start, handle_end, &uuid);
                if handle == None {
                    break;
                }

                let found_attr = handle.unwrap().0;

                let mut attr_len = 0;
                if counter == 0 {
                    attr_len = (*found_attr).attr_len as usize;
                } else {
                    attr_len = (*found_attr).attr_len as usize;
                    if attr_len != counter {
                        break;
                    }
                }
                if 0x13 < attr_len + dest_ptr * 2 {
                    break;
                }
                counter = handle.unwrap().1;
                state.rf_packet_att_rsp.value[dest_ptr * 2 + 1] = (counter & 0xff) as u8;
                state.rf_packet_att_rsp.value[dest_ptr * 2 + 2] = (counter >> 8) as u8;

                handle_start = dest_ptr + 1;
                state.rf_packet_att_rsp.value[(handle_start * 2) + 1] = (((counter - 1) + (*found_attr).att_num as usize) & 0xff) as u8;
                state.rf_packet_att_rsp.value[(handle_start * 2) + 2] = (((counter - 1) + (*found_attr).att_num as usize) >> 8) as u8;

                handle_start = handle_start + 1;
                state.rf_packet_att_rsp.value[(handle_start * 2) + 1..(handle_start * 2) + (*found_attr).attr_len as usize + 1].copy_from_slice(
                    slice::from_raw_parts(
                        (*found_attr).p_attr_value,
                        (*found_attr).attr_len as usize,
                    )
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
                state.pkt_err_rsp.err_opcode = 0x10;
                state.pkt_err_rsp.err_handle = (*packet).value[4] as u16;
                (addr_of!(state.pkt_err_rsp) as *const PacketL2capHead, size_of_val(&state.pkt_err_rsp))
            } else {
                state.rf_packet_att_rsp.dma_len = (dest_ptr as u32 + 4) * 2;
                state.rf_packet_att_rsp._type = 2;
                state.rf_packet_att_rsp.rf_len = state.rf_packet_att_rsp.dma_len as u8 - 2;
                state.rf_packet_att_rsp.l2cap_len = state.rf_packet_att_rsp.dma_len as u16 - 6;
                state.rf_packet_att_rsp.chan_id = 4;
                state.rf_packet_att_rsp.opcode = 0x11;
                state.rf_packet_att_rsp.value[0] = counter as u8 + 4;

                (addr_of!(state.rf_packet_att_rsp) as *const PacketL2capHead, size_of_val(&state.rf_packet_att_rsp))
            }
        }
        Some(GattOp::AttOpWriteReq) | Some(GattOp::AttOpWriteCmd) => {
            let att_num = (*packet).value[4] as usize;

            let mut result;
            if *(get_gAttributes()[att_num].uuid as *const u16) != 0x2902 {
                if att_num < 2 {
                    return (null(), 0);
                }
                if unsafe { *get_gAttributes()[att_num - 1].p_attr_value } & 0xc == 0 {
                    return (null(), 0);
                }
            }

            result = (null(), 0);
            if (*packet).value[5] != 0 {
                return result;
            }
            if (get_gAttributes()[0].att_num as usize) < att_num {
                return result;
            }
            if (*packet).value[3] == 0x12 {
                result = (addr_of!(state.pkt_write_rsp) as *const PacketL2capHead, size_of_val(&state.pkt_write_rsp));
            }
            if get_gAttributes()[att_num].w.is_none() {
                if *(addr_of!((*packet).handle1) as *const u16) < 3 {
                    return result;
                }

                if get_gAttributes()[att_num].p_attr_value as u32 <= addr_of!(__RAM_START_ADDR) as u32 {
                    return result;
                }

                let dest = slice::from_raw_parts_mut(
                    get_gAttributes()[att_num].p_attr_value,
                    get_gAttributes()[att_num].attr_len as usize,
                );

                dest.fill(0);
                dest.copy_from_slice(
                    slice::from_raw_parts(
                        addr_of!((*packet).value[6]),
                        get_gAttributes()[att_num].attr_len as usize,
                    )
                );

                return result;
            }

            get_gAttributes()[att_num].w.unwrap()(state, unsafe { &mut *(packet as *mut PacketAttWrite) });

            result
        }
        _ => (null(), 0),
    };
}
