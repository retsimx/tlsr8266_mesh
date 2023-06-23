use core::cell::{RefCell, RefMut};
use core::mem::size_of_val;
use core::ptr::{addr_of, addr_of_mut, null, null_mut};
use core::slice;

use num_derive::FromPrimitive;
use num_traits::FromPrimitive;

use crate::pub_mut;
use crate::config::VENDOR_ID;
use crate::sdk::app_att_light::{attribute_t, get_send_to_master, get_TelinkSppDataClient2ServiceUUID, get_TelinkSppDataOtaUUID, get_TelinkSppDataPairUUID, get_TelinkSppDataServer2ClientUUID, get_TelinkSppServiceUUID};
use crate::sdk::ble_app::rf_drv_8266::get_gAttributes;
use crate::sdk::light::{get_rf_slave_ota_finished_flag, OtaState, PacketAttErrRsp, PacketAttMtu, PacketAttReadRsp, PacketAttWrite, PacketAttWriteRsp, PacketCtrlUnknown, PacketFeatureRsp, PacketL2capData, PacketL2capHead, PacketVersionInd, set_rf_slave_ota_terminate_flag};
use crate::sdk::mcu::register::read_reg_system_tick;
use crate::state::State;

pub_mut!(pkt_version_ind, PacketVersionInd, PacketVersionInd {
    dma_len: 8,
    _type: 3,
    rf_len: 6,
    opcode: 0x0c,
    main_ver: 0x08,
    vendor: VENDOR_ID,
    sub_ver: 0x08
});
pub_mut!(rf_pkt_unknown_response, PacketCtrlUnknown, PacketCtrlUnknown {
    dma_len: 0x04,
    _type: 0x03,
    rf_len: 0x02,
    opcode: 0x07,
    data: [0]
});
pub_mut!(pkt_feature_rsp, PacketFeatureRsp, PacketFeatureRsp{
    dma_len: 0x0b,
    _type: 0x3,
    rf_len: 0x09,
    opcode: 0x09,
    data: [1, 0, 0, 0, 0, 0, 0, 0]
});
pub_mut!(pkt_mtu_rsp, PacketAttMtu, PacketAttMtu {
    dma_len: 0x09,
    _type: 2,
    rf_len: 0x07,
    l2cap_len: 0x03,
    chan_id: 0x04,
    opcode: 0x03,
    mtu: [0x17, 0x00]
});
pub_mut!(pkt_errRsp, PacketAttErrRsp, PacketAttErrRsp{
    dma_len: 0x0b,
    _type: 0x02,
    rf_len: 0x09,
    l2cap_len: 0x05,
    chan_id: 0x04,
    opcode: 0x01,
    err_opcode: 0,
    err_handle: 0,
    err_reason: 0x0a
});
pub_mut!(rf_packet_att_rsp, PacketAttReadRsp, PacketAttReadRsp{
    dma_len: 0,
    _type: 0,
    rf_len: 0,
    l2cap_len: 0,
    chan_id: 0,
    opcode: 0,
    value: [0; 22]
});
pub_mut!(pkt_writeRsp, PacketAttWriteRsp, PacketAttWriteRsp{
    dma_len: 0x07,
    _type: 2,
    rf_len: 0x05,
    l2cap_len: 0x01,
    chan_id: 0x04,
    opcode: 0x13
});
pub_mut!(att_service_discover_tick, u32, 0);
pub_mut!(slave_link_time_out, u32, 0);
#[cfg(not(test))]
extern "C" {
    pub static __RAM_START_ADDR: u32;
}

#[cfg(test)]
pub static mut __RAM_START_ADDR: u32 = 0;

#[derive(FromPrimitive)]
pub enum GattOp {
    AttOpErrorRsp = 0x01, // Error Response op code
    AttOpExchangeMtuReq = 0x02, // Exchange MTU Request op code
    AttOpExchangeMtuRsp = 0x03, // Exchange MTU Response op code
    AttOpFindInfoReq = 0x04, // Find Information Request op code
    AttOpFindInfoRsp = 0x05, // Find Information Response op code
    AttOpFindByTypeValueReq = 0x06, // Find By Type Vaue Request op code
    AttOpFindByTypeValueRsp = 0x07, // Find By Type Vaue Response op code
    AttOpReadByTypeReq = 0x08, // Read By Type Request op code
    AttOpReadByTypeRsp = 0x09, // Read By Type Response op code
    AttOpReadReq = 0x0a, // Read Request op code
    AttOpReadRsp = 0x0b, // Read Response op code
    AttOpReadBlobReq = 0x0c, // Read Blob Request op code
    AttOpReadBlobRsp = 0x0d, // Read Blob Response op code
    AttOpReadMultiReq = 0x0e, // Read Multiple Request op code
    AttOpReadMultiRsp = 0x0f, // Read Multiple Response op code
    AttOpReadByGroupTypeReq = 0x10, // Read By Group Type Request op code
    AttOpReadByGroupTypeRsp = 0x11, // Read By Group Type Response op code
    AttOpWriteReq = 0x12, // Write Request op code
    AttOpWriteRsp = 0x13, // Write Response op code
    AttOpPrepareWriteReq = 0x16, // Prepare Write Request op code
    AttOpPrepareWriteRsp = 0x17, // Prepare Write Response op code
    AttOpExecuteWriteReq = 0x18, // Execute Write Request op code
    AttOpExecuteWriteRsp = 0x19, // Execute Write Response op code
    AttOpHandleValueNoti = 0x1b, // Handle Value Notification op code
    AttOpHandleValueInd = 0x1d, // Handle Value Indication op code
    AttOpHandleValueCfm = 0x1e, // Handle Value Confirmation op code
    AttOpWriteCmd = 0x52, // ATT Write Command
}

pub unsafe fn l2cap_att_search(mut handle_start: u16, handle_end: u16, uuid: &[u8]) -> Option<(*const attribute_t, u16)>
{
    let att_num = (*(*get_gAttributes()).offset(0)).attNum as u16;

    if att_num != handle_start {
        let mut end = handle_end;
        if att_num < handle_end {
            end = att_num;
        }
        for handle_start in handle_start..=end {
            if (*(*get_gAttributes()).offset(handle_start as isize)).uuidLen == 2 {
                if *(uuid.as_ptr() as *const u16) == *((*(*get_gAttributes()).offset(handle_start as isize)).uuid as *const u16) {
                    return Some(((*get_gAttributes()).offset(handle_start as isize), handle_start));
                }
            } else {
                if uuid == slice::from_raw_parts(((*(*get_gAttributes()).offset(handle_start as isize)).uuid as *const u8), 0x10) {
                    return Some(((*get_gAttributes()).offset(handle_start as isize), handle_start));
                }
            }
        }
    }

    return None;
}

pub unsafe fn l2cap_att_handler(state: &RefCell<State>, mut packet: *const PacketL2capData) -> (*const PacketL2capHead, usize)
{
    if *get_gAttributes() == null_mut() {
        return (null(), 0);
    }

    if (*packet).opcode & 3 == GattOp::AttOpExchangeMtuRsp as u8 {
        let handle = (*packet).handle1;
        if handle == 0xc {
            (*get_pkt_version_ind()).rf_len = 6;
            set_att_service_discover_tick(read_reg_system_tick() | 1);
            return (get_pkt_version_ind_addr() as *const PacketL2capHead, size_of_val(&*get_pkt_version_ind_addr()));
        }
        if handle != 8 {
            if handle == 2 {
                set_slave_link_time_out(1000000);
                return (null(), 0);
            }
            (*get_rf_pkt_unknown_response()).data[0] = handle;
            return (get_rf_pkt_unknown_response_addr() as *const PacketL2capHead, size_of_val(&*get_rf_pkt_unknown_response()));
        }
        (*get_pkt_feature_rsp()).rf_len = 9;
        set_att_service_discover_tick(read_reg_system_tick() | 1);
        return (get_pkt_feature_rsp_addr() as *const PacketL2capHead, size_of_val(&*get_pkt_feature_rsp()));
    }

    if *(addr_of!((*packet).value[1]) as *const u16) != 4 {
        return (null(), 0);
    }

    return match FromPrimitive::from_u8((*packet).value[3]) {
        Some(GattOp::AttOpExchangeMtuReq) => {
            return (get_pkt_mtu_rsp_addr() as *const PacketL2capHead, size_of_val(&*get_pkt_mtu_rsp()));
        },
        Some(GattOp::AttOpFindInfoReq) => {
            set_att_service_discover_tick(read_reg_system_tick() | 1);
            let mut start_handle = (*packet).value[4];
            let mut end_handle = (*packet).value[6];
            if (*(*get_gAttributes())).attNum < (*packet).value[6] {
                end_handle = (*(*get_gAttributes())).attNum;
            }
            if start_handle <= end_handle {
                let mut uuid_len = 0;
                let mut handle = 1;
                let mut counter = 0;
                let mut offset_adj = 4;
                loop {
                    if uuid_len == 0 {
                        uuid_len = (*(*get_gAttributes()).offset(start_handle as isize)).uuidLen;
                    } else if (*(*get_gAttributes()).offset(start_handle as isize)).uuidLen != uuid_len {
                        if counter == 0 {
                            break;
                        }

                        (*get_rf_packet_att_rsp()).l2cap_len = counter + 2;
                        (*get_rf_packet_att_rsp()).dma_len = counter as u32 + 8;
                        (*get_rf_packet_att_rsp())._type = 2;
                        (*get_rf_packet_att_rsp()).rf_len = counter as u8 + 6;
                        (*get_rf_packet_att_rsp()).chan_id = 4;
                        (*get_rf_packet_att_rsp()).opcode = 5;
                        (*get_rf_packet_att_rsp()).value[0] = handle;
                        return (get_rf_packet_att_rsp_addr() as *const PacketL2capHead, size_of_val(&*get_rf_packet_att_rsp()));
                    }
                    (*get_rf_packet_att_rsp()).value[counter as usize + 1] = start_handle;
                    (*get_rf_packet_att_rsp()).value[counter as usize + 2] = 0;
                    if (*(*get_gAttributes()).offset(start_handle as isize)).uuidLen == 2 {
                        *(addr_of!((*get_rf_packet_att_rsp()).value[counter as usize + 3]) as *mut u16) = *((*(*get_gAttributes()).offset(start_handle as isize)).uuid as *const u16);
                        counter = counter + 4;
                    } else {
                        (*get_rf_packet_att_rsp()).value[counter as usize + 3..counter as usize + 3 + 0x10].copy_from_slice(
                            slice::from_raw_parts(
                                (*(*get_gAttributes()).offset(start_handle as isize)).uuid as *const u8,
                                    0x10
                            )
                        );

                        counter += 0x12;
                        handle = 2;
                        offset_adj = 0x10;
                    }
                    start_handle = start_handle + 1;
                    if end_handle < start_handle || 0x17 < offset_adj + counter {
                        (*get_rf_packet_att_rsp()).l2cap_len = counter + 2;
                        (*get_rf_packet_att_rsp()).dma_len = counter as u32 + 8;
                        (*get_rf_packet_att_rsp())._type = 2;
                        (*get_rf_packet_att_rsp()).rf_len = counter as u8 + 6;
                        (*get_rf_packet_att_rsp()).chan_id = 4;
                        (*get_rf_packet_att_rsp()).opcode = 5;
                        (*get_rf_packet_att_rsp()).value[0] = handle;
                        return (get_rf_packet_att_rsp_addr() as *const PacketL2capHead, size_of_val(&*get_rf_packet_att_rsp()));
                    }
                }
            }
            (*get_pkt_errRsp()).err_opcode = 4;
            (*get_pkt_errRsp()).err_handle = start_handle as u16;
            (get_pkt_errRsp_addr() as *const PacketL2capHead, size_of_val(&*get_pkt_errRsp()))
        },
        Some(GattOp::AttOpFindByTypeValueReq) => {
            set_att_service_discover_tick(read_reg_system_tick() | 1);
            let mut start_handle = (*packet).value[4] as u16;
            let end_handle = (*packet).value[6] as u16;
            let mut uuid = [0; 2];
            uuid.copy_from_slice(&(*packet).value[8..=9]);
            let value = *(addr_of!((*packet).value[10]) as *const u16);
            let mut counter = 0;
            loop {
                let handle = l2cap_att_search(start_handle as u16, end_handle as u16, &uuid);
                if handle == None || 9 < counter
                {
                    break;
                }
                let found_attr = handle.unwrap().0;
                let found_handle = handle.unwrap().1;
                if (*found_attr).attrLen == 2 && *((*found_attr).pAttrValue as *const u16) == value {
                    (*get_rf_packet_att_rsp()).value[counter * 2] = (found_handle & 0xff) as u8;
                    (*get_rf_packet_att_rsp()).value[counter * 2 + 1] = (found_handle >> 8) as u8;
                    start_handle = counter as u16 + 1;
                    (*get_rf_packet_att_rsp()).value[start_handle as usize * 2] = ((*found_attr).attNum as u16 + (found_handle - 1) & 0xff) as u8;
                    (*get_rf_packet_att_rsp()).value[start_handle as usize * 2 + 1] = ((*found_attr).attNum as u16 + (found_handle - 1) >> 8) as u8;
                    counter = start_handle as usize + 1;
                    start_handle = found_handle as u16 + (*found_attr).attNum as u16;
                } else {
                    start_handle = found_handle + 1;
                }

                if start_handle > (*packet).value[6] as u16 {
                    break;
                }
            }
            if counter == 0 {
                (*get_pkt_errRsp()).err_opcode = 6;
                (*get_pkt_errRsp()).err_handle = start_handle as u16;

                (get_pkt_errRsp_addr() as *const PacketL2capHead, size_of_val(&*get_pkt_errRsp()))
            } else {
                let iVar8 = counter * 2;
                (*get_rf_packet_att_rsp()).dma_len = iVar8 as u32 + 7;
                (*get_rf_packet_att_rsp())._type = 2;
                (*get_rf_packet_att_rsp()).rf_len = iVar8 as u8 + 5;
                (*get_rf_packet_att_rsp()).l2cap_len = iVar8 as u16 + 1;
                (*get_rf_packet_att_rsp()).chan_id = 4;
                (*get_rf_packet_att_rsp()).opcode = 7;

                (get_rf_packet_att_rsp_addr() as *const PacketL2capHead, size_of_val(&*get_rf_packet_att_rsp()))
            }
        },
        Some(GattOp::AttOpReadByTypeReq) => {
            set_att_service_discover_tick(read_reg_system_tick() | 1);
            let mut handle_start = (*packet).value[4] as u16;
            let handle_end = (*packet).value[6] as u16;
            let mut tick = 0;
            let mut bVar11 = 0;
            let mut found_handle = handle_end;
            if *(addr_of!((*packet).handle1) as *const u16) == 0x15 {
                let mut uuid = [0; 16];
                uuid.copy_from_slice(&(*packet).value[8..8+0x10]);
                bVar11 = 0x10;
                let handle = l2cap_att_search(handle_start, handle_end, &uuid);
                if handle == None
                {
                    (*get_rf_packet_att_rsp()).value[0] = 0;
                    tick = 0;
                } else {
                    let found_attr = handle.unwrap().0;
                    if (*found_attr).uuidLen != bVar11 {
                        return (null(), 0);
                    }
                    found_handle = handle.unwrap().1;
                    (*get_rf_packet_att_rsp()).value[1] = found_handle as u8;
                    (*get_rf_packet_att_rsp()).value[2] = (found_handle >> 8) as u8;
                    (*get_rf_packet_att_rsp()).value[3..3+(*found_attr).attrLen as usize].copy_from_slice(slice::from_raw_parts((*found_attr).pAttrValue, (*found_attr).attrLen as usize));
                    tick = (*found_attr).attrLen + 2;
                    (*get_rf_packet_att_rsp()).value[0] = tick;
                }
            } else {
                let mut uuid = [0; 2];
                uuid.copy_from_slice(&(*packet).value[8..=9]);
                if uuid[0] != 3 || uuid[1] != 0x28 {
                    bVar11 = 2;
                    let handle = l2cap_att_search(handle_start, handle_end, &uuid);
                    if handle == None
                    {
                        (*get_rf_packet_att_rsp()).value[0] = 0;
                        tick = 0;
                    } else {
                        let found_attr = handle.unwrap().0;
                        if (*found_attr).uuidLen != bVar11 {
                            return (null(), 0);
                        }
                        found_handle = handle.unwrap().1;
                        (*get_rf_packet_att_rsp()).value[1] = found_handle as u8;
                        (*get_rf_packet_att_rsp()).value[2] = (found_handle >> 8) as u8;
                        (*get_rf_packet_att_rsp()).value[3..3+(*found_attr).attrLen as usize].copy_from_slice(slice::from_raw_parts((*found_attr).pAttrValue, (*found_attr).attrLen as usize));
                        tick = (*found_attr).attrLen + 2;
                        (*get_rf_packet_att_rsp()).value[0] = tick;
                    }
                }

                let mut counter = 0;
                tick = 0;

                loop {
                    let handle = l2cap_att_search(handle_start, handle_end, &uuid);
                    if handle == None
                    {
                        break;
                    }
                    let found_attr = handle.unwrap().0;
                    found_handle = handle.unwrap().1;

                    if !((counter == 0 || (*found_attr.offset(1)).uuidLen == counter) && tick + (*found_attr).uuidLen < 0x13) {
                        break;
                    }

                    (*get_rf_packet_att_rsp()).value[tick as usize + 1] = found_handle as u8;
                    tick = tick + 1;
                    (*get_rf_packet_att_rsp()).value[tick as usize + 1] = 0;
                    tick = tick + 1;
                    (*get_rf_packet_att_rsp()).value[tick as usize + 1] = *(*found_attr).pAttrValue;
                    tick = tick + 1;
                    (*get_rf_packet_att_rsp()).value[tick as usize + 1] = found_handle as u8 + 1;
                    tick = tick + 1;
                    handle_start = found_handle + 2;
                    (*get_rf_packet_att_rsp()).value[tick as usize + 1] = 0;
                    tick = tick + 1;

                    (*get_rf_packet_att_rsp()).value[tick as usize + 1..tick as usize + 1 + (*found_attr.offset(1)).uuidLen as usize].copy_from_slice(
                        slice::from_raw_parts(
                        (*found_attr.offset(1)).uuid,
                        (*found_attr.offset(1)).uuidLen as usize
                        )
                    );

                    counter = (*found_attr.offset(1)).uuidLen;
                    tick = counter + tick;
                }
                (*get_rf_packet_att_rsp()).value[0] = counter + 5;
            }
            if tick == 0 {
                (*get_pkt_errRsp()).err_opcode = 8;
                (*get_pkt_errRsp()).err_handle = found_handle as u16;
                return (get_pkt_errRsp_addr() as *const PacketL2capHead, size_of_val(&*get_pkt_errRsp()));
            } else {
                (*get_rf_packet_att_rsp()).dma_len = tick as u32 + 8;
                (*get_rf_packet_att_rsp())._type = 2;
                (*get_rf_packet_att_rsp()).rf_len = tick as u8 + 6;
                (*get_rf_packet_att_rsp()).l2cap_len = tick as u16 + 2;
                (*get_rf_packet_att_rsp()).chan_id = 4;
                (*get_rf_packet_att_rsp()).opcode = 9;

                (get_rf_packet_att_rsp_addr() as *const PacketL2capHead, size_of_val(&*get_rf_packet_att_rsp()))
            }
        },
        Some(GattOp::AttOpReadReq) => {
            let tick = (*packet).value[4] as u32;
            if (*packet).value[5] != 0 {
                return (null(), 0);
            }
            if (*(*get_gAttributes())).attNum < tick as u8 {
                return (null(), 0);
            }
            if (*(*get_gAttributes()).offset(tick as isize)).r.is_none() {
                slice::from_raw_parts_mut(
                    addr_of_mut!((*get_rf_packet_att_rsp()).value[0]) as *mut u8,
                    (*(*get_gAttributes()).offset(tick as isize)).attrLen as usize,
                ).copy_from_slice(
                    slice::from_raw_parts(
                        (*(*get_gAttributes()).offset(tick as isize)).pAttrValue,
                        (*(*get_gAttributes()).offset(tick as isize)).attrLen as usize,
                    )
                );

                if (*(*get_gAttributes()).offset(tick as isize)).pAttrValue == (*get_send_to_master()).as_mut_ptr() {
                    (*get_send_to_master()).fill(0);
                } else if tick == 0x18 && *get_rf_slave_ota_finished_flag() != OtaState::Continue {
                    set_rf_slave_ota_terminate_flag(true);
                }
                (*get_rf_packet_att_rsp()).rf_len = (*(*get_gAttributes()).offset(tick as isize)).attrLen + 5;
                (*get_rf_packet_att_rsp()).dma_len = (*get_rf_packet_att_rsp()).rf_len as u32 + 2;
                (*get_rf_packet_att_rsp())._type = 2;
                (*get_rf_packet_att_rsp()).l2cap_len = (*(*get_gAttributes()).offset(tick as isize)).attrLen as u16 + 1;
                (*get_rf_packet_att_rsp()).chan_id = 4;
                (*get_rf_packet_att_rsp()).opcode = 0xb;
                return (get_rf_packet_att_rsp_addr() as *const PacketL2capHead, size_of_val(&*get_rf_packet_att_rsp()));
            }

            (*(*get_gAttributes()).offset(tick as isize)).r.unwrap()(state, packet as *const PacketAttWrite);
            // return get_rf_pkt_unknown_response_addr() as *const RfPacketL2capHeadT;
            (null(), 0)
        },
        Some(GattOp::AttOpReadByGroupTypeReq) => {
            set_att_service_discover_tick(read_reg_system_tick() | 1);
            let mut handle_start = (*packet).value[4] as u16;
            let handle_end = (*packet).value[6] as u16;
            let mut uuid = [0; 2];
            uuid.copy_from_slice(&(*packet).value[8..=9]);
            let mut dest_ptr = 0;
            let mut counter = 0u16;
            loop {
                let handle = l2cap_att_search(handle_start, handle_end, &uuid);
                if handle == None {
                    break;
                }

                let found_attr = handle.unwrap().0;

                let mut attr_len = 0;
                if counter == 0 {
                    attr_len = (*found_attr).attrLen;
                } else {
                    attr_len = (*found_attr).attrLen;
                    if attr_len as u16 != counter {
                        break;
                    }
                }
                if 0x13 < attr_len + dest_ptr as u8 * 2 {
                    break;
                }
                counter = handle.unwrap().1;
                (*get_rf_packet_att_rsp()).value[dest_ptr * 2 + 1] = (counter & 0xff) as u8;
                (*get_rf_packet_att_rsp()).value[dest_ptr * 2 + 2] = (counter >> 8) as u8;

                handle_start = dest_ptr as u16 + 1;
                (*get_rf_packet_att_rsp()).value[(handle_start * 2) as usize + 1] = (((counter - 1) + (*found_attr).attNum as u16) & 0xff) as u8;
                (*get_rf_packet_att_rsp()).value[(handle_start * 2) as usize + 2] = (((counter - 1) + (*found_attr).attNum as u16) >> 8) as u8;

                handle_start = handle_start + 1;
                (*get_rf_packet_att_rsp()).value[(handle_start as usize * 2) + 1..(handle_start as usize * 2) + (*found_attr).attrLen as usize + 1].copy_from_slice(
                    slice::from_raw_parts(
                        (*found_attr).pAttrValue,
                        (*found_attr).attrLen as usize,
                    )
                );
                dest_ptr = handle_start as usize + ((*found_attr).attrLen as usize / 2);
                counter = counter + (*found_attr).attNum as u16;
                handle_start = counter;
                counter = attr_len as u16;
                if handle_start > handle_end {
                    break;
                }
            }
            if dest_ptr == 0 {
                (*get_pkt_errRsp()).err_opcode = 0x10;
                (*get_pkt_errRsp()).err_handle = (*packet).value[4] as u16;
                (get_pkt_errRsp_addr() as *const PacketL2capHead, size_of_val(&*get_pkt_errRsp()))
            } else {
                (*get_rf_packet_att_rsp()).dma_len = (dest_ptr as u32 + 4) * 2;
                (*get_rf_packet_att_rsp())._type = 2;
                (*get_rf_packet_att_rsp()).rf_len = (*get_rf_packet_att_rsp()).dma_len as u8 - 2;
                (*get_rf_packet_att_rsp()).l2cap_len = (*get_rf_packet_att_rsp()).dma_len as u16 - 6;
                (*get_rf_packet_att_rsp()).chan_id = 4;
                (*get_rf_packet_att_rsp()).opcode = 0x11;
                (*get_rf_packet_att_rsp()).value[0] = counter as u8 + 4;

                (get_rf_packet_att_rsp_addr() as *const PacketL2capHead, size_of_val(&*get_rf_packet_att_rsp()))
            }
        },
        Some(GattOp::AttOpWriteReq) | Some(GattOp::AttOpWriteCmd) => {
            let tick = (*packet).value[4];
            if *((*(*get_gAttributes()).offset(tick as isize)).uuid as *const u16) != 0x2902 {
                if tick < 2 {
                    return (null(), 0);
                }
                if *(*(*get_gAttributes()).offset(tick as isize - 1)).pAttrValue & 0xc == 0 {
                    return (null(), 0);
                }
            }
            let mut result = (null(), 0);
            if (*packet).value[5] != 0 {
                return result;
            }
            if (*(*get_gAttributes())).attNum < tick {
                return result;
            }
            if (*packet).value[3] == 0x12 {
                result = (get_pkt_writeRsp_addr() as *const PacketL2capHead, size_of_val(&*get_pkt_writeRsp()));
            }
            if (*(*get_gAttributes()).offset(tick as isize)).w.is_none() {
                if *(addr_of!((*packet).handle1) as *const u16) < 3 {
                    return result;
                }

                if (*(*get_gAttributes()).offset(tick as isize)).pAttrValue as u32 <= addr_of!(__RAM_START_ADDR) as u32 {
                    return result;
                }

                let dest = slice::from_raw_parts_mut(
                    (*(*get_gAttributes()).offset(tick as isize)).pAttrValue,
                    (*(*get_gAttributes()).offset(tick as isize)).attrLen as usize,
                );

                dest.fill(0);
                dest.copy_from_slice(
                    slice::from_raw_parts(
                        addr_of!((*packet).value[6]),
                        (*(*get_gAttributes()).offset(tick as isize)).attrLen as usize,
                    )
                );

                return result;
            }

            (*(*get_gAttributes()).offset(tick as isize)).w.unwrap()(state, packet as *const PacketAttWrite);

            result
        },
        _ => (null(), 0),
    };
}

pub fn setSppUUID(service_uuid: *const u8, server2client_uuid: *const u8, client2service_uuid: *const u8, ota_uuid: *const u8, pair_uuid: *const u8)
{
    unsafe {
        (*get_TelinkSppServiceUUID()).copy_from_slice(slice::from_raw_parts(service_uuid, 0x10));
        (*get_TelinkSppDataServer2ClientUUID()).copy_from_slice(slice::from_raw_parts(server2client_uuid, 0x10));
        (*get_TelinkSppDataClient2ServiceUUID()).copy_from_slice(slice::from_raw_parts(client2service_uuid, 0x10));
        (*get_TelinkSppDataOtaUUID()).copy_from_slice(slice::from_raw_parts(ota_uuid, 0x10));
        (*get_TelinkSppDataPairUUID()).copy_from_slice(slice::from_raw_parts(pair_uuid, 0x10));
    }
}