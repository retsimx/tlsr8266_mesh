use core::cmp::min;
use core::ptr::{addr_of, addr_of_mut, slice_from_raw_parts_mut};
use core::slice;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use crate::{app, BIT};
use crate::common::rf_update_conn_para;
use crate::config::VENDOR_ID;
use crate::embassy::time_driver::clock_time64;
use crate::main_light::{rf_link_data_callback, rf_link_response_callback};
use crate::mesh::{MESH_NODE_ST_VAL_LEN, mesh_node_st_val_t};
use crate::sdk::ble_app::ble_ll_attribute::l2cap_att_handler;
use crate::sdk::ble_app::ble_ll_pair::{pair_enc_packet};
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::light::{*};
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, clock_time};
use crate::sdk::mcu::register::{*};
use crate::sdk::packet_types::{*};
use crate::state::{*};
use crate::uart_manager::light_mesh_rx_cb;

use super::mesh_management::{mesh_node_update_status, rf_link_match_group_mac};

/// Checks if a packet exists in the RC packet buffer
#[cfg_attr(test, mry::mry)]
pub fn is_exist_in_rc_pkt_buf(opcode: u8, cmd_pkt: &Packet) -> bool
{
    RC_PKT_BUF.lock().iter().any(|v| v.op == opcode && v.sno == cmd_pkt.att_cmd().value.sno)
}

/// Checks if an opcode is a notify response
fn rf_link_is_notify_rsp(opcode: u8) -> bool
{
    [
        LGT_CMD_LIGHT_GRP_RSP1,
        LGT_CMD_LIGHT_GRP_RSP2,
        LGT_CMD_LIGHT_GRP_RSP3,
        LGT_CMD_LIGHT_STATUS,
        LGT_CMD_DEV_ADDR_RSP,
        LGT_CMD_USER_NOTIFY_RSP,
        LGT_CMD_START_OTA_RSP,
        LGT_CMD_OTA_DATA_RSP
    ].contains(&opcode)
}

/// Pushes a packet to the RC packet buffer
fn rc_pkt_buf_push(opcode: u8, cmd_pkt: &Packet)
{
    let mut rc_pkt_buf = RC_PKT_BUF.lock();

    if rc_pkt_buf.is_full() {
        rc_pkt_buf.pop_back();
    }

    rc_pkt_buf.push_front(
        PktBuf {
            op: opcode,
            sno: cmd_pkt.att_cmd().value.sno,
            notify_ok_flag: false,
        }
    ).unwrap();
}

/// Checks if a request command is notify ok
fn req_cmd_is_notify_ok(opcode: u8, cmd_pkt: &Packet) -> bool
{
    RC_PKT_BUF.lock().iter().any(|pkt| {
        pkt.op == opcode && pkt.sno == cmd_pkt.att_cmd().value.sno && pkt.notify_ok_flag
    })
}

/// Sets the notify ok flag for a request command
fn req_cmd_set_notify_ok_flag(opcode: u8, cmd_pkt: &Packet)
{
    RC_PKT_BUF.lock().iter_mut().filter(
        |v| { v.op == opcode && v.sno == cmd_pkt.att_cmd().value.sno }
    ).for_each(
        |v| { v.notify_ok_flag = true }
    );
}

/// Checks if an opcode is a notify request
#[cfg_attr(test, mry::mry)]
pub fn rf_link_is_notify_req(value: u8) -> bool
{
    if !OTA_UPDATE_IN_PROGRESS.get() {
        return [
            LGT_CMD_LIGHT_READ_STATUS,
            LGT_CMD_LIGHT_GRP_REQ,
            LGT_CMD_CONFIG_DEV_ADDR,
            LGT_CMD_LIGHT_CONFIG_GRP,
            LGT_CMD_USER_NOTIFY_REQ,
            LGT_CMD_START_OTA_REQ,
            LGT_CMD_OTA_DATA_REQ,
            LGT_CMD_END_OTA_REQ
        ].contains(&value);
    }

    return false;
}

/// Handles mask notification for slave read status
fn rf_link_slave_notify_req_mask(adr: u8)
{
    if SLAVE_READ_STATUS_BUSY.get() != 0 && (DEVICE_ADDRESS.get() as u8 != adr || SLAVE_READ_STATUS_BUSY.get() == 0x21) {
        if !DEVICE_STATUS_READ_UNICAST_MODE.get() {
            if PKT_LIGHT_DATA.lock().att_cmd().value.val[8..0xd].iter().any(|v| *v == adr) {
                return;
            }

            PKT_LIGHT_DATA.lock().att_cmd_mut().value.val[(NOTIFICATION_REQUEST_MASK_INDEX.get() + 8) as usize] = adr;
            NOTIFICATION_REQUEST_MASK_INDEX.set((NOTIFICATION_REQUEST_MASK_INDEX.get() + 1) % 5);
        } else {
            SLAVE_DATA_VALID.set(0);
        }
    }
}

/// Adds slave status to response buffer
#[cfg_attr(test, mry::mry)]
pub fn rf_link_slave_add_status(packet: &Packet)
{
    let mut buf_response = BUFF_RESPONSE.lock();

    let mut result = false;

    if DEVICE_STATUS_RECORD_INDEX.get() != 0 {
        for st_rec in *SLAVE_STATUS_RECORD.lock() {
            if packet.mesh().src_adr as u8 == st_rec.adr[0] {
                rf_link_slave_notify_req_mask(packet.mesh().src_adr as u8);
                return;
            }
        }
    }

    result = false;
    if (DEVICE_STATUS_BUFFER_WRITE_POINTER.get() + 1) % BUFF_RESPONSE_PACKET_COUNT != DEVICE_STATUS_BUFFER_READ_POINTER.get() && DEVICE_STATUS_RECORD_INDEX.get() < MESH_NODE_MAX_NUM {
        SLAVE_STATUS_RECORD.lock()[DEVICE_STATUS_RECORD_INDEX.get()].adr[0] = packet.mesh().src_adr as u8;
        DEVICE_STATUS_RECORD_INDEX.inc();
        rf_link_slave_notify_req_mask(packet.mesh().src_adr as u8);

        // Update the buffer write pointer
        let index = DEVICE_STATUS_BUFFER_WRITE_POINTER.get();
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set((index + 1) % BUFF_RESPONSE_PACKET_COUNT);

        let st_ptr = &mut buf_response[index];
        st_ptr.head_mut().dma_len = 0x1d;
        st_ptr.head_mut()._type = 2;
        st_ptr.head_mut().rf_len = 0x1b;
        st_ptr.head_mut().l2cap_len = 0x17;
        st_ptr.head_mut().chan_id = 4;
        st_ptr.att_data_mut().att = 0x1b;
        st_ptr.att_data_mut().hl = 0x12;

        st_ptr.att_data_mut().dat[0..0x14].copy_from_slice(
            unsafe {
                slice::from_raw_parts(addr_of!(packet.mesh().sno) as *const u8, 0x14)
            }
        );

        if packet.mesh().internal_par1[1] == 0 {
            st_ptr.att_data_mut().dat[0x12] = packet.mesh().par[9];
            st_ptr.att_data_mut().dat[0x13] = packet.mesh().internal_par1[0];
        } else if packet.mesh().internal_par1[1] != 4 && packet.mesh().internal_par1[1] != 5 && packet.mesh().internal_par1[1] != 8 {
            if packet.mesh().internal_par1[1] == 6 {
                st_ptr.att_data_mut().dat[0x11..0x11 + 3].fill(0xff);
            } else if packet.mesh().internal_par1[1] != 7 && packet.mesh().internal_par1[1] != 9 {
                st_ptr.att_data_mut().dat[0x12..0x12 + 2].fill(0xff);
            }
        }
        st_ptr.att_data_mut().dat[3] = packet.mesh().src_adr as u8;
        st_ptr.att_data_mut().dat[4] = (packet.mesh().src_adr >> 8) as u8;
        result = true;
    }
}

/// Handles received RC (remote control) data packets
pub fn rf_link_rc_data(packet: &mut Packet) {
    let pktdata = unsafe { &*slice_from_raw_parts_mut(addr_of!(packet.att_write().value) as *mut u8, core::mem::size_of::<PacketAttValue>()) };

    // Check if this is a node update packet (pkt adv status)
    if packet.head().chan_id == 0xffff {
        if pktdata[24..28].iter().all(|v| *v == 0xa5) {
            mesh_node_update_status(unsafe { slice::from_raw_parts(addr_of!(packet.mesh().sno) as *const mesh_node_st_val_t, 0x1a / MESH_NODE_ST_VAL_LEN) });
        }

        return;
    }

    // Parse the opcode and parameters from the packet
    let (success, mut op_cmd, mut op_cmd_len, mut params, mut params_len) = parse_ble_packet_op_params(packet, true);
    if !success {
        return;
    }

    // Get the opcode
    let mut op = 0;
    if op_cmd_len == 3 {
        op = op_cmd[0] & 0x3f;
    }

    let not_slave_message = packet.att_cmd().value.sno != *GENERAL_MESSAGE_SEQUENCE_NUMBER.lock();

    // If we've already seen this packet, then there is nothing else to do
    if is_exist_in_rc_pkt_buf(op, packet) {
        return;
    }

    // Send this packet over UART
    light_mesh_rx_cb(packet);

    // Record the packet so we don't handle it again if we receive it again
    rc_pkt_buf_push(op, packet);

    // If the slave link is connected (Android) and the dest address is us, then we should forward
    // the packet on to the slave link if the opcode is a response opcode and the slave (Android) is waiting
    // for a response
    if rf_link_is_notify_rsp(op) && packet.mesh().dst_adr == DEVICE_ADDRESS.get() && BLE_PERIPHERAL_CONNECTION_ACTIVE.get() {
        if SLAVE_READ_STATUS_BUSY.get() != op || packet.att_cmd().value.sno != *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() {
            return;
        }
        rf_link_slave_add_status(packet);
        return;
    }

    let (group_match, device_match) = rf_link_match_group_mac(packet);
    if group_match || device_match {
        rf_link_data_callback(packet);

        // Don't worry about responding with an ack for a notify request.
        if !rf_link_is_notify_req(op) && packet.mesh().internal_par1[INTERNAL_PAR_SEND_ACK] != 0 {
            // Prepare and send the ack message
            let mut pkt_light_status = PKT_LIGHT_STATUS.lock();

            let cmd_sno = clock_time() + DEVICE_ADDRESS.get() as u32;
            pkt_light_status.att_cmd_mut().value.sno.copy_from_slice(unsafe {
                slice::from_raw_parts(addr_of!(cmd_sno) as *const u8, 3)
            });

            packet.mesh_mut().src_tx = DEVICE_ADDRESS.get();

            unsafe {
                // todo: I reckon this is probably incorrect, why is there a +1?
                let ptr = slice::from_raw_parts((addr_of!(packet.mesh().vendor_id) as u32 + 1) as *const u8, params_len as usize);
                pkt_light_status.att_cmd_mut().value.val[3..3 + ptr.len()].copy_from_slice(&ptr);
            }

            if not_slave_message || BLE_PERIPHERAL_LINK_COMMAND.get() != op {
                pkt_light_status.att_cmd_mut().value.src.copy_from_slice(unsafe { slice::from_raw_parts(addr_of!(packet.mesh().src_adr) as *const u8, 2) });

                pkt_light_status.att_cmd_mut().value.dst = packet.att_cmd().value.src;
                pkt_light_status.att_cmd_mut().value.src[0] = (DEVICE_ADDRESS.get() & 0xff) as u8;
                pkt_light_status.att_cmd_mut().value.src[1] = ((DEVICE_ADDRESS.get() >> 8) & 0xff) as u8;

                pkt_light_status.att_cmd_mut().value.val[0] = LGT_CMD_LIGHT_ACK | 0xc0;
                pkt_light_status.att_cmd_mut().value.val[1] = (VENDOR_ID & 0xFF) as u8;
                pkt_light_status.att_cmd_mut().value.val[2] = ((VENDOR_ID >> 8) & 0xff) as u8;

                pkt_light_status.att_cmd_mut().value.val[3..10 + 3].fill(0);
                pkt_light_status.att_cmd_mut().value.val[3] = op;
                pkt_light_status.att_cmd_mut().value.val[4..4 + 3].copy_from_slice(&packet.att_cmd().value.sno);

                pkt_light_status.head_mut()._type |= BIT!(7);

                app().mesh_manager.add_send_mesh_msg(&*pkt_light_status, 0, packet.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT]);
            }
        }
    }

    let mut slave_read_status_response = device_match;

    // Only handle notify requests once
    if rf_link_is_notify_req(op) {
        if req_cmd_is_notify_ok(op, packet) {
            slave_read_status_response = false;
        } else {
            req_cmd_set_notify_ok_flag(op, packet);
        }
    }

    packet.head_mut().dma_len = (packet.head().l2cap_len as u32 + 6) & 0xffffff;
    packet.head_mut().rf_len = packet.head().l2cap_len as u8 + 4;

    if not_slave_message || BLE_PERIPHERAL_LINK_COMMAND.get() != op {
        packet.mesh_mut().src_tx = DEVICE_ADDRESS.get();

        *GENERAL_MESSAGE_SEQUENCE_NUMBER.lock() = packet.att_cmd().value.sno;
        BLE_PERIPHERAL_LINK_COMMAND.set(op);
    }

    if rf_link_is_notify_req(op) && slave_read_status_response {
        let mut pkt_light_status = PKT_LIGHT_STATUS.lock();

        pkt_light_status.att_cmd_mut().value.sno = packet.att_cmd().value.sno;
        packet.mesh_mut().src_tx = DEVICE_ADDRESS.get();
        if op == LGT_CMD_LIGHT_READ_STATUS {
            pkt_light_status.att_cmd_mut().value.val[15] = GET_STATUS;
            pkt_light_status.att_cmd_mut().value.val[13] = packet.mesh().par[9];
            pkt_light_status.att_cmd_mut().value.val[14] = 0;
        } else if op == LGT_CMD_LIGHT_GRP_REQ {
            pkt_light_status.att_cmd_mut().value.val[15] = packet.mesh().internal_par1[1];
        } else if op == LGT_CMD_LIGHT_CONFIG_GRP {
            pkt_light_status.att_cmd_mut().value.val[15] = GET_GROUP1;
        } else if op == LGT_CMD_CONFIG_DEV_ADDR {
            pkt_light_status.att_cmd_mut().value.val[15] = GET_DEV_ADDR;
        } else if op == LGT_CMD_USER_NOTIFY_REQ {
            pkt_light_status.att_cmd_mut().value.val[15] = GET_USER_NOTIFY;
        } else if op == LGT_CMD_START_OTA_REQ {
            pkt_light_status.att_cmd_mut().value.val[15] = CMD_START_OTA;
        } else if op == LGT_CMD_OTA_DATA_REQ {
            pkt_light_status.att_cmd_mut().value.val[15] = CMD_OTA_DATA;
        } else if op == LGT_CMD_END_OTA_REQ {
            pkt_light_status.att_cmd_mut().value.val[15] = CMD_END_OTA;
        }

        unsafe {
            // todo: I reckon this is probably incorrect, why is there a +1?
            let ptr = slice::from_raw_parts((addr_of!(packet.mesh().vendor_id) as u32 + 1) as *const u8, params_len as usize);
            pkt_light_status.att_cmd_mut().value.val[3..3 + ptr.len()].copy_from_slice(&ptr);
        }

        if (not_slave_message || BLE_PERIPHERAL_LINK_COMMAND.get() != op) || params[1] != 0 {
            pkt_light_status.att_cmd_mut().value.src.copy_from_slice(unsafe { slice::from_raw_parts(addr_of!(packet.mesh().src_adr) as *const u8, 2) });

            if rf_link_response_callback(&mut pkt_light_status.att_cmd_mut().value, &packet.att_cmd().value) {
                pkt_light_status.head_mut()._type |= BIT!(7);
                
                app().mesh_manager.add_send_mesh_msg(&*pkt_light_status, 0, packet.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT]);
            }
        }
    }

    // Relay the message if it's not for us
    if !device_match
    {
        packet.head_mut()._type |= BIT!(7);

        let mut delay = 100;
        if !BLE_PERIPHERAL_CONNECTION_ACTIVE.get() {
            // Random delay to avoid congestion between 0us and 8ms
            delay = 8000 - (((read_reg_system_tick() as u16 ^ read_reg_rnd_number()) % 16) * 500);
        }

        app().mesh_manager.add_send_mesh_msg(packet, clock_time64() + (delay as u64 * CLOCK_SYS_CLOCK_1US as u64), packet.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT]);
    }
}

/// Handles slave data packets
#[cfg_attr(test, mry::mry)]
pub fn rf_link_slave_data(packet: &Packet, time: u32) -> bool {
    let rf_len: u8 = packet.head().rf_len;
    let chanid: u16 = packet.head().chan_id;

    if packet.head()._type as i32 * 0x1000000 >= -1 {
        if (packet.head()._type & 3) == 2 {
            if 6 < chanid {
                return false;
            }
        }

        if chanid == 5 {
            rf_update_conn_para(packet);
        }

        super::connection_management::rf_link_timing_adjust(time);
        if rf_len < 6 {
            if rf_len == 0 {
                return false;
            }
        } else {
            if packet.head()._type & 3 == 3 && packet.head().l2cap_len & 0xff == 1 {
                BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(1);
                BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(((packet.ll_data().sno as u16) << 8) | packet.ll_data().hh as u16);
                SLAVE_CHN_MAP.lock().iter_mut().enumerate().for_each(|(i, v)| {
                    *v = unsafe { *(addr_of!(packet.head().l2cap_len) as *const u8).offset(i as isize + 1) };
                });

                return true;
            }

            if rf_len == 0xc && packet.head()._type & 3 == 3 && packet.head().l2cap_len & 0xff == 0 {
                BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.set(SLAVE_LINK_INTERVAL.get());
                BLE_PERIPHERAL_NEXT_UPDATE_INSTANT.set(packet.ll_data().group);
                SLAVE_WINDOW_SIZE_UPDATE.set(((packet.head().l2cap_len >> 8) as u32 * 1250 + 1300) * CLOCK_SYS_CLOCK_1US);
                BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP.set(2);
                BLE_CONN_INTERVAL.set(CLOCK_SYS_CLOCK_1US * 1250 * unsafe { (*addr_of!(packet.ll_data().att) as *const u16) } as u32);
                BLE_CONN_OFFSET.set(packet.head().chan_id as u32 * CLOCK_SYS_CLOCK_1US * 1250);
                BLE_CONN_TIMEOUT.set(packet.ll_data().nid as u32 * 10000);
                return false;
            }
        }
        let res_pkt = l2cap_att_handler(packet);
        if res_pkt.is_some() {
            rf_link_add_tx_packet(&res_pkt.unwrap());
        }
    }

    return false;
}

/// Checks if the packet buffer is ready for new packets
#[cfg_attr(test, mry::mry)]
pub fn is_add_packet_buf_ready() -> bool
{
    return (read_reg_dma_tx_wptr() - read_reg_dma_tx_rptr() & 7) < 3;
}

/// Adds a transmission packet to the queue
#[cfg_attr(test, mry::mry)]
pub fn rf_link_add_tx_packet(packet: &Packet) -> bool
{
    use crate::embassy::sync::mutex::{CriticalSectionMutex, Mutex};
    
    static BLT_TX_FIFO: CriticalSectionMutex<[Packet; BLT_FIFO_TX_PACKET_COUNT]> = Mutex::new(
        [
            Packet {
                att_write: PacketAttWrite {
                    head: PacketL2capHead {
                        dma_len: 0,
                        _type: 0,
                        rf_len: 0,
                        l2cap_len: 0,
                        chan_id: 0,
                    },
                    opcode: 0,
                    handle: 0,
                    handle1: 0,
                    value: PacketAttValue {
                        sno: [0; 3],
                        src: [0; 2],
                        dst: [0; 2],
                        val: [0; 23]
                    },
                }
            };
            BLT_FIFO_TX_PACKET_COUNT
        ]
    );
    static BLT_TX_WPTR: AtomicUsize = AtomicUsize::new(0);

    let wptr = read_reg_dma_tx_wptr();
    let rptr = read_reg_dma_tx_rptr();
    let widx = (wptr - rptr) % BLT_FIFO_TX_PACKET_COUNT as u8;

    if widx < 4 {
        if widx == 0 {
            write_reg_dma_tx_fifo(addr_of!(PKT_EMPTY) as u16);
        }

        let index = BLT_TX_WPTR.get();
        BLT_TX_WPTR.set((index + 1) % BLT_FIFO_TX_PACKET_COUNT);

        let mut blt_tx_fifo = BLT_TX_FIFO.lock();

        blt_tx_fifo[index] = *packet;
        pair_enc_packet(&mut blt_tx_fifo[index]);

        write_reg_dma_tx_fifo(addr_of!(blt_tx_fifo[index]) as u16);
        return true;
    }
    return false;
}

/// Extracts operation codes and parameters from a BLE packet.
///
/// This function parses the given packet to extract operation codes and their associated parameters
/// based on the BLE packet format. It handles different operation code lengths (1, 2, or 3 bytes)
/// and processes parameters differently based on mesh/non-mesh mode.
///
/// # Arguments
///
/// * `packet` - A reference to the packet to parse
/// * `mesh_flag` - A boolean flag indicating whether the packet is a mesh packet (affects parameter handling)
///
/// # Returns
///
/// A tuple containing:
/// * `success` - Boolean indicating if parsing was successful
/// * `op_codes` - Array of operation code bytes (up to 3 bytes)
/// * `op_len` - Length of the operation code (1, 2, or 3 bytes)
/// * `parameters` - Array of parameter bytes (up to 16 bytes)
/// * `params_len` - Length of the extracted parameters
///
/// # Operation Code Format
///
/// The operation code length is determined by the MSB bits of the first byte:
/// * If MSB is clear (0), the op code is 1 byte
/// * If MSB is set (1) and bit 6 is clear (10), the op code is 2 bytes
/// * If MSB is set (1) and bit 6 is set (11), the op code is 3 bytes
///
/// # Parameter Handling
///
/// Parameters are extracted differently based on:
/// * The mesh_flag parameter
/// * The operation code (special handling for op code 6)
/// * Available packet length
///
#[cfg_attr(test, mry::mry)]
pub fn parse_ble_packet_op_params(packet: &Packet, mesh_flag: bool) -> (bool, [u8; 3], u8, [u8; 16], u8)
{
    // Initialize return values
    let mut op_codes = [0u8; 3];
    let mut op_len = 0u8;
    let mut parameters = [0u8; 16];
    let mut params_len = 0u8;

    // Access the value field directly through the att_cmd() accessor method
    let val = &packet.att_cmd().value.val;
    
    // Determine the operation command length based on the bit patterns
    let first_op_byte = val[0];
    if first_op_byte & 0x80 != 0 {  // MSB is set
        op_len = if first_op_byte & 0x40 != 0 { 3 } else { 2 };
    } else {  // MSB is clear
        op_len = 1;
    }
    
    // Copy the operation code bytes
    op_codes[0..op_len as usize].copy_from_slice(&val[0..op_len as usize]);

    // Special handling for opcode 6 (more parameters allowed + delta offset adjustment)
    let is_special_op = (first_op_byte & 0x3f) == 6;
    let max_param_len = if is_special_op { 0xf } else { 10 };
    let pkt_len_delta = if is_special_op { 5 } else { 0 };

    // Calculate total available data space for parameters
    let header_len: u16 = 10;
    let mut packet_data_len = packet.head().l2cap_len - header_len;
    
    // Adjust for mesh vs non-mesh packet structures
    if mesh_flag {
        packet_data_len = pkt_len_delta + packet_data_len - op_len as u16 - header_len;
    } else {
        packet_data_len = packet_data_len - op_len as u16;
    }

    params_len = packet_data_len as u8;

    // Check if parameters will fit within allowed limits
    let success = packet_data_len <= max_param_len;
    
    if success {
        // Copy parameter bytes from the packet
        let param_start = op_len as usize;
        let param_end = param_start + packet_data_len as usize;
        parameters[0..packet_data_len as usize].copy_from_slice(&val[param_start..param_end]);
    } else {
        // If parameters are too long, set the length to 0
        params_len = 0;
    }

    (success, op_codes, op_len, parameters, params_len)
}
