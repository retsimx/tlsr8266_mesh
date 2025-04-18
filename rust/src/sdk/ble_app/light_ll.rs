use core::cmp::min;
use core::ptr::{addr_of, addr_of_mut, slice_from_raw_parts_mut};
use core::slice;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use crate::{app, BIT, uprintln};
use crate::common::{rf_update_conn_para, SYS_CHN_LISTEN, update_ble_parameter_cb};
use crate::config::{FLASH_ADR_LIGHT_NEW_FW, VENDOR_ID};
use crate::embassy::sync::mutex::{CriticalSectionMutex, Mutex};
use crate::embassy::time_driver::clock_time64;
use crate::main_light::{rf_link_data_callback, rf_link_response_callback};
use crate::mesh::{MESH_NODE_ST_VAL_LEN, mesh_node_st_val_t};
use crate::sdk::ble_app::ble_ll_channel_selection::ble_ll_build_available_channel_table;
use crate::sdk::ble_app::ble_ll_attribute::l2cap_att_handler;
use crate::sdk::ble_app::ble_ll_pair::{pair_enc_packet, pair_init, pair_save_key, pair_set_key};
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::drivers::flash::{flash_read_page, flash_write_page};
use crate::sdk::light::{*};
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, clock_time, clock_time_exceed, sleep_us};
use crate::sdk::mcu::register::{*};
use crate::sdk::packet_types::{*};
use crate::state::{*};
use crate::uart_manager::light_mesh_rx_cb;

fn mesh_node_update_status(pkt: &[mesh_node_st_val_t]) -> u32
{
    let mut mesh_node_st = MESH_NODE_ST.lock();

    let mut src_index = 0;
    let mut result = 0xfffffffe;
    let tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;
    while src_index < pkt.len() && pkt[src_index].dev_adr != 0 {
        // todo: This needs to be removed when we figure out why dev address 1 is being introduced
        // todo: incorrectly
        if pkt[src_index].dev_adr == 1 {
            src_index += 1;
            continue;
        }

        if DEVICE_ADDRESS.get() as u8 != pkt[src_index].dev_adr {
            let mesh_node_max = MESH_NODE_MAX.get();
            let mut current_index = 1;
            let mut mesh_node = &mut mesh_node_st[current_index];
            if mesh_node_max >= 2 {
                if mesh_node.val.dev_adr != pkt[src_index].dev_adr {
                    for tidx in 1..MESH_NODE_MAX_NUM {
                        current_index = tidx;
                        mesh_node = &mut mesh_node_st[current_index];

                        if mesh_node_max <= tidx as u8 || pkt[src_index].dev_adr == mesh_node.val.dev_adr {
                            break;
                        }
                    }
                }
            }

            if MESH_NODE_MAX_NUM == current_index {
                return 1;
            }

            if mesh_node_max as usize == current_index {
                MESH_NODE_MAX.inc();

                mesh_node.val = pkt[src_index];
                mesh_node.tick = tick;

                MESH_NODE_MASK.lock()[mesh_node_max as usize >> 5] |= 1 << (mesh_node_max & 0x1f);

                result = mesh_node_max as u32;
            } else if current_index < mesh_node_max as usize {
                let sn_difference = pkt[src_index].sn - mesh_node.val.sn;
                let par_match = pkt[src_index].par == mesh_node.val.par;

                // todo: Why the divided by two here?
                let timeout = (ONLINE_STATUS_TIMEOUT * 1000) / 2;

                result = current_index as u32;
                if sn_difference - 2 < 0x3f || (sn_difference != 0 && (mesh_node.tick == 0 || (((timeout * CLOCK_SYS_CLOCK_1US) >> 0x10) as u16) < tick - mesh_node.tick)) {
                    mesh_node.val = pkt[src_index];

                    if !par_match || mesh_node.tick == 0 {
                        MESH_NODE_MASK.lock()[current_index >> 5] |= 1 << (current_index & 0x1f);
                    }

                    mesh_node.tick = tick;
                }
            }
        }

        src_index += 1;
    }
    return 1;
}

pub fn is_exist_in_rc_pkt_buf(opcode: u8, cmd_pkt: &Packet) -> bool
{
    RC_PKT_BUF.lock().iter().any(|v| v.op == opcode && v.sno == cmd_pkt.att_cmd().value.sno)
}

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

fn req_cmd_is_notify_ok(opcode: u8, cmd_pkt: &Packet) -> bool
{
    RC_PKT_BUF.lock().iter().any(|pkt| {
        pkt.op == opcode && pkt.sno == cmd_pkt.att_cmd().value.sno && pkt.notify_ok_flag
    })
}

fn req_cmd_set_notify_ok_flag(opcode: u8, cmd_pkt: &Packet)
{
    RC_PKT_BUF.lock().iter_mut().filter(
        |v| { v.op == opcode && v.sno == cmd_pkt.att_cmd().value.sno }
    ).for_each(
        |v| { v.notify_ok_flag = true }
    );
}

fn rf_link_slave_notify_req_mask(adr: u8)
{
    if SLAVE_READ_STATUS_BUSY.get() != 0 && (DEVICE_ADDRESS.get() as u8 != adr || SLAVE_READ_STATUS_BUSY.get() == 0x21) {
        if !SLAVE_READ_STATUS_UNICAST_FLAG.get() {
            if PKT_LIGHT_DATA.lock().att_cmd().value.val[8..0xd].iter().any(|v| *v == adr) {
                return;
            }

            PKT_LIGHT_DATA.lock().att_cmd_mut().value.val[(NOTIFY_REQ_MASK_IDX.get() + 8) as usize] = adr;
            NOTIFY_REQ_MASK_IDX.set((NOTIFY_REQ_MASK_IDX.get() + 1) % 5);
        } else {
            SLAVE_DATA_VALID.set(0);
        }
    }
}

#[cfg_attr(test, mry::mry)]
pub fn rf_link_slave_add_status(packet: &Packet)
{
    let mut buf_response = BUFF_RESPONSE.lock();

    let mut result = false;

    if SLAVE_STATUS_RECORD_IDX.get() != 0 {
        for st_rec in *SLAVE_STATUS_RECORD.lock() {
            if packet.mesh().src_adr as u8 == st_rec.adr[0] {
                rf_link_slave_notify_req_mask(packet.mesh().src_adr as u8);
                return;
            }
        }
    }

    result = false;
    if (SLAVE_STATUS_BUFFER_WPTR.get() + 1) % BUFF_RESPONSE_PACKET_COUNT != SLAVE_STATUS_BUFFER_RPTR.get() && SLAVE_STATUS_RECORD_IDX.get() < MESH_NODE_MAX_NUM {
        SLAVE_STATUS_RECORD.lock()[SLAVE_STATUS_RECORD_IDX.get()].adr[0] = packet.mesh().src_adr as u8;
        SLAVE_STATUS_RECORD_IDX.inc();
        rf_link_slave_notify_req_mask(packet.mesh().src_adr as u8);

        // Update the buffer write pointer
        let index = SLAVE_STATUS_BUFFER_WPTR.get();
        SLAVE_STATUS_BUFFER_WPTR.set((index + 1) % BUFF_RESPONSE_PACKET_COUNT);

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

pub fn rf_link_rc_data(packet: &mut Packet) {
    let pktdata = unsafe { &*slice_from_raw_parts_mut(addr_of!(packet.att_write().value) as *mut u8, size_of::<PacketAttValue>()) };

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

    let not_slave_message = packet.att_cmd().value.sno != *SLAVE_SNO.lock();

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
    if rf_link_is_notify_rsp(op) && packet.mesh().dst_adr == DEVICE_ADDRESS.get() && SLAVE_LINK_CONNECTED.get() {
        if SLAVE_READ_STATUS_BUSY.get() != op || packet.att_cmd().value.sno != *SLAVE_STAT_SNO.lock() {
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

            if not_slave_message || SLAVE_LINK_CMD.get() != op {
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

    if not_slave_message || SLAVE_LINK_CMD.get() != op {
        packet.mesh_mut().src_tx = DEVICE_ADDRESS.get();

        *SLAVE_SNO.lock() = packet.att_cmd().value.sno;
        SLAVE_LINK_CMD.set(op);
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

        if (not_slave_message || SLAVE_LINK_CMD.get() != op) || params[1] != 0 {
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
        if !SLAVE_LINK_CONNECTED.get() {
            // Random delay to avoid congestion between 0us and 8ms
            delay = 8000 - (((read_reg_system_tick() as u16 ^ read_reg_rnd_number()) % 16) * 500);
        }

        app().mesh_manager.add_send_mesh_msg(packet, clock_time64() + (delay as u64 * CLOCK_SYS_CLOCK_1US as u64), packet.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT]);
    }
}

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

        rf_link_timing_adjust(time);
        if rf_len < 6 {
            if rf_len == 0 {
                return false;
            }
        } else {
            if packet.head()._type & 3 == 3 && packet.head().l2cap_len & 0xff == 1 {
                SLAVE_TIMING_UPDATE.set(1);
                SLAVE_INSTANT_NEXT.set(((packet.ll_data().sno as u16) << 8) | packet.ll_data().hh as u16);
                SLAVE_CHN_MAP.lock().iter_mut().enumerate().for_each(|(i, v)| {
                    *v = unsafe { *(addr_of!(packet.head().l2cap_len) as *const u8).offset(i as isize + 1) };
                });

                return true;
            }

            if rf_len == 0xc && packet.head()._type & 3 == 3 && packet.head().l2cap_len & 0xff == 0 {
                SLAVE_INTERVAL_OLD.set(SLAVE_LINK_INTERVAL.get());
                SLAVE_INSTANT_NEXT.set(packet.ll_data().group);
                SLAVE_WINDOW_SIZE_UPDATE.set(((packet.head().l2cap_len >> 8) as u32 * 1250 + 1300) * CLOCK_SYS_CLOCK_1US);
                SLAVE_TIMING_UPDATE.set(2);
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

pub fn rf_link_timing_adjust(time: u32)
{
    if SLAVE_TIMING_ADJUST_ENABLE.get() {
        SLAVE_TIMING_ADJUST_ENABLE.set(false);
        if time - SLAVE_TICK_BRX.get() < CLOCK_SYS_CLOCK_1US * 700 {
            SLAVE_NEXT_CONNECT_TICK.set(SLAVE_NEXT_CONNECT_TICK.get() - CLOCK_SYS_CLOCK_1US * 200);
        } else if CLOCK_SYS_CLOCK_1US * 1100 < time - SLAVE_TICK_BRX.get() {
            SLAVE_NEXT_CONNECT_TICK.set(SLAVE_NEXT_CONNECT_TICK.get() + CLOCK_SYS_CLOCK_1US * 200);
        }
    }
}

fn check_par_con(packet: &Packet) -> bool
{
    if packet.ll_init().interval - 6 & 0xffff < 0xc7b && packet.ll_init().wsize != 0 && packet.ll_init().wsize < 9 && 9 < packet.ll_init().timeout && packet.ll_init().timeout < 0xc81 && packet.ll_init().woffset <= packet.ll_init().interval && packet.ll_init().hop != 0 &&
        packet.ll_init().chm.iter().any(|v| { *v != 0 }) {
        if packet.ll_init().latency == 0 {
            return false;
        }

        if packet.ll_init().latency as u32 <= ((packet.ll_init().interval as u32) << 3) / packet.ll_init().interval as u32 {
            return false;
        }
    }
    return true;
}

pub fn rf_link_slave_connect(packet: &Packet, time: u32) -> bool
{
    CONN_UPDATE_SUCCESSED.set(false);
    CONN_UPDATE_CNT.set(0);

    if SLAVE_CONNECTION_ENABLE.get() || packet.ll_init().scan_a == packet.ll_init().adv_a {
        if check_par_con(packet) == false {
            rf_stop_trx();

            SLAVE_WINDOW_OFFSET.set(CLOCK_SYS_CLOCK_1US * 1250 * (packet.ll_init().woffset as u32 + 1));

            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 1000 + read_reg_system_tick());
            write_reg_irq_src(0x100000);
            write_reg_system_tick_irq(time + SLAVE_WINDOW_OFFSET.get() + CLOCK_SYS_CLOCK_1US * (0 - if packet.ll_init().woffset == 0 { 500 } else { 700 }));
            if 0x80000000 < read_reg_system_tick_irq() - read_reg_system_tick() {
                write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick());
            }

            LIGHT_CONN_SN_MASTER.set(0x80);
            SLAVE_CONNECTED_TICK.set(read_reg_system_tick());
            if SECURITY_ENABLE.get() {
                SLAVE_FIRST_CONNECTED_TICK.set(SLAVE_CONNECTED_TICK.get());
            }

            SLAVE_STATUS_TICK.set(((packet.ll_init().interval as u32 * 5) >> 2) as u8);
            SLAVE_LINK_INTERVAL.set(packet.ll_init().interval as u32 * CLOCK_SYS_CLOCK_1US * 1250);
            SLAVE_WINDOW_SIZE.set((packet.ll_init().wsize as u32 * 1250 + 1100) * CLOCK_SYS_CLOCK_1US);

            let tmp = SLAVE_LINK_INTERVAL.get() - CLOCK_SYS_CLOCK_1US * 1250;
            if tmp <= SLAVE_WINDOW_SIZE.get() && SLAVE_WINDOW_SIZE.get() - tmp != 0 {
                SLAVE_WINDOW_SIZE.set(tmp);
            }

            SLAVE_LINK_TIME_OUT.set(packet.ll_init().timeout as u32 * 10000);
            PKT_INIT.lock().ll_init_mut().clone_from(&PacketLlInit {
                dma_len: 0x24,
                _type: 0x5,
                rf_len: 0x22,
                ..*packet.ll_init()
            });

            let chn_map = PKT_INIT.lock().ll_init().chm;
            ble_ll_build_available_channel_table(&chn_map, true);

            // rf_set_ble_crc(&(state.PKT_INIT()).crcinit);
            let crcinit = PKT_INIT.lock().ll_init().crcinit;
            write_reg_rf_crc(((crcinit[1] as u32) << 8) | ((crcinit[2] as u32) << 0x10) | crcinit[0] as u32);
            rf_reset_sn();

            SLAVE_INSTANT.set(0);
            SLAVE_TIMING_UPDATE2_FLAG.set(false);
            SLAVE_INTERVAL_OLD.set(0);
            SLAVE_TIMING_UPDATE2_OK_TIME.set(0);
            SLAVE_WINDOW_SIZE_UPDATE.set(0);

            pair_init();

            MESH_NODE_REPORT_ENABLE.set(false);

            MESH_NODE_MASK.lock().fill(0);

            *P_ST_HANDLER.lock() = IrqHandlerStatus::Rx;
            NEED_UPDATE_CONNECT_PARA.set(true);
            ATT_SERVICE_DISCOVER_TICK.set(read_reg_system_tick() | 1);

            write_reg8(0x00800f04, 0x67);  // tx wail & settle time

            return true;
        }
    }
    return false;
}

pub fn light_check_tick_per_us(ticks: u32)
{
    if ticks == 0x10 {
        T_SCAN_RSP_INTVL.set(0);
    } else if ticks == 0x20 || ticks != 0x30 {
        T_SCAN_RSP_INTVL.set(0x92);
    } else {
        T_SCAN_RSP_INTVL.set(0x93);
    }
}

pub fn rf_link_slave_pairing_enable(enable: bool)
{
    SLAVE_CONNECTION_ENABLE.set(enable);
    SLAVE_ADV_ENABLE.set(enable);
}

// param st is 2bytes = lumen% + rsv(0xFF)  // rf pkt : device_address+sn+lumen+rsv);
pub fn ll_device_status_update(val_par: &[u8])
{
    let mut mesh_node_st = MESH_NODE_ST.lock();

    mesh_node_st[0].val.par.copy_from_slice(val_par);
    mesh_node_st[0].tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;

    MESH_NODE_MASK.lock()[0] |= 1;
}

pub fn setup_ble_parameter_start(mut interval_min: u16, mut interval_max: u16, timeout: u32) -> u32
{
    let mut invalid = false;

    if interval_max | interval_min == 0 {
        UPDATE_INTERVAL_USER_MAX.set(SLAVE_LINK_INTERVAL.get() as u16);
        UPDATE_INTERVAL_USER_MIN.set(SLAVE_LINK_INTERVAL.get() as u16);
        interval_max = UPDATE_INTERVAL_USER_MAX.get();
        interval_min = UPDATE_INTERVAL_USER_MIN.get();
    } else {
        if interval_min < INTERVAL_THRESHOLD {
            invalid = true;
        }

        UPDATE_INTERVAL_USER_MIN.set(interval_min);
        UPDATE_INTERVAL_USER_MAX.set(interval_max);
    }

    if timeout < 100 {
        UPDATE_INTERVAL_USER_MAX.set(0);
        UPDATE_INTERVAL_USER_MIN.set(0);
        return 0xfffffffd;
    }

    if invalid == false {
        return 0;
    }

    UPDATE_INTERVAL_USER_MIN.set(0);
    UPDATE_INTERVAL_USER_MAX.set(0);
    return 0xfffffffe;
}

fn update_connect_para()
{
    if NEED_UPDATE_CONNECT_PARA.get() {
        if ATT_SERVICE_DISCOVER_TICK.get() != 0 {
            if UPDATE_CONNECT_PARA_DELAY_MS * CLOCK_SYS_CLOCK_1US * 1000 < read_reg_system_tick() - ATT_SERVICE_DISCOVER_TICK.get() {
                NEED_UPDATE_CONNECT_PARA.set(false);
                ATT_SERVICE_DISCOVER_TICK.set(0);

                update_ble_parameter_cb();
            }
        }
    }
}

pub fn mesh_node_flush_status()
{
    static TICK_NODE_REPORT: AtomicU32 = AtomicU32::new(0);

    // Only report status every 500ms
    if !clock_time_exceed(TICK_NODE_REPORT.load(Ordering::Relaxed), 500000) {
        return;
    }

    let tick = read_reg_system_tick();
    TICK_NODE_REPORT.store(tick, Ordering::Relaxed);

    let mut mesh_node_st = MESH_NODE_ST.lock();

    // Iterate over each mesh node and check if it's timed out
    for count in 1..MESH_NODE_MAX.get() as usize {
        if mesh_node_st[count].tick != 0 && (CLOCK_SYS_CLOCK_1US * ONLINE_STATUS_TIMEOUT * 1000) >> 0x10 < (tick >> 0x10 | 1) - mesh_node_st[count].tick as u32 {
            mesh_node_st[count].tick = 0;

            // Set the bit in the mask so that the status is reported (Since the device has changed to offline now)
            MESH_NODE_MASK.lock()[count >> 5] |= 1 << (count & 0x1f);
        }
    }
}

fn mesh_node_keep_alive()
{
    DEVICE_NODE_SN.inc();
    if DEVICE_NODE_SN.get() == 0 {
        DEVICE_NODE_SN.set(1);
    }

    let mut mesh_node_st = MESH_NODE_ST.lock();

    mesh_node_st[0].val.sn = DEVICE_NODE_SN.get();
    mesh_node_st[0].tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;
}

fn mesh_node_adv_status(p_data: &mut [u8]) -> u32
{
    static MESH_NODE_CUR: AtomicUsize = AtomicUsize::new(1);

    // Clear the result data
    p_data.fill(0);

    // Get the max number of elements the result data can hold
    let mut elems = p_data.len() / MESH_NODE_ST_VAL_LEN;
    if (MESH_NODE_MAX.get() as usize) < p_data.len() / MESH_NODE_ST_VAL_LEN {
        elems = MESH_NODE_MAX.get() as usize;
    }

    {
        let mut mesh_node_st = MESH_NODE_ST.lock();

        // Copy our status in to the result data first
        p_data[0..MESH_NODE_ST_VAL_LEN].copy_from_slice(
            unsafe {
                slice::from_raw_parts(
                    addr_of!(mesh_node_st[0].val) as *const u8,
                    MESH_NODE_ST_VAL_LEN,
                )
            }
        );
    }

    // Update our own record to keep our status record in sync
    mesh_node_keep_alive();

    let mut mesh_node_st = MESH_NODE_ST.lock();

    let max_node = MESH_NODE_MAX.get() as usize;
    let mut count = 1;

    let mut out_index = count;
    while out_index < elems && count < max_node {
        let mnc = MESH_NODE_CUR.load(Ordering::Relaxed);
        if mnc < max_node && mesh_node_st[mnc].tick != 0 {
            let ptr = MESH_NODE_ST_VAL_LEN * out_index;
            out_index = out_index + 1;
            p_data[ptr..ptr + MESH_NODE_ST_VAL_LEN].copy_from_slice(
                unsafe {
                    slice::from_raw_parts(
                        addr_of!(mesh_node_st[mnc].val) as *const u8,
                        MESH_NODE_ST_VAL_LEN,
                    )
                }
            );
        }

        MESH_NODE_CUR.store(mnc + 1, Ordering::Relaxed);

        if max_node <= MESH_NODE_CUR.load(Ordering::Relaxed) {
            MESH_NODE_CUR.store(1, Ordering::Relaxed);
        }

        count += 1;
    }

    return out_index as u32;
}

pub fn mesh_send_online_status()
{
    static ADV_ST_SN: AtomicU32 = AtomicU32::new(0);
    static LAST_STATUS_TIME: AtomicU32 = AtomicU32::new(0);

    if !clock_time_exceed(LAST_STATUS_TIME.get(), 1000 * SEND_MESH_STATUS_INTERVAL_MS) {
        return;
    }

    LAST_STATUS_TIME.set(clock_time());

    let mut pkt_light_adv_status = Packet {
        att_write: PacketAttWrite {
            head: PacketL2capHead {
                dma_len: 0x27,
                _type: 2,
                rf_len: 0x25,
                l2cap_len: 0x21,
                chan_id: 0xffff,
            },
            opcode: 0,
            handle: 0,
            handle1: 0,
            value: PacketAttValue::default(),
        }
    };

    let pktdata = unsafe { &mut *slice_from_raw_parts_mut(addr_of!(pkt_light_adv_status.att_write().value) as *mut u8, size_of::<PacketAttValue>()) };

    mesh_node_flush_status();
    mesh_node_adv_status(&mut pktdata[..24]);

    ADV_ST_SN.store(ADV_ST_SN.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
    unsafe {
        let val = ADV_ST_SN.load(Ordering::Relaxed);
        slice::from_raw_parts_mut(addr_of_mut!(pkt_light_adv_status.att_write_mut().opcode), 3).copy_from_slice(slice::from_raw_parts(addr_of!(val) as *const u8, 3))
    }

    pktdata[24..28].fill(0xa5);

    // todo: Perhaps this send count should be greater than 0 to improve reliability
    app().mesh_manager.add_send_mesh_msg(&pkt_light_adv_status, 0, 0);
}

pub fn back_to_rxmode_bridge()
{
    rf_set_tx_rx_off();
    rf_set_ble_access_code(PAIR_AC.get());
    rf_set_ble_crc_adv();
    rf_set_ble_channel(SYS_CHN_LISTEN[(ST_BRIGE_NO.get() as usize % SYS_CHN_LISTEN.len()) >> 1]);
    rf_set_rxmode();
}

#[cfg_attr(test, mry::mry)]
pub fn rf_link_is_notify_req(value: u8) -> bool
{
    if !RF_SLAVE_OTA_BUSY.get() {
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

pub fn app_bridge_cmd_handle(bridge_cmd_time: u32)
{
    let mut pkt_light_data = PKT_LIGHT_DATA.lock();

    if SLAVE_DATA_VALID.get() != 0 {
        SLAVE_DATA_VALID.dec();
        if SLAVE_DATA_VALID.get() == 0 {} else if SLAVE_READ_STATUS_BUSY.get() == 0 || SLAVE_DATA_VALID.get() as i32 > -1 {
            pkt_light_data.head_mut()._type |= BIT!(7);

            if rf_link_is_notify_req(pkt_light_data.att_cmd().value.val[0] & 0x3f) {
                let mut relay_time = min(
                    (((read_reg_system_tick() - bridge_cmd_time) / CLOCK_SYS_CLOCK_1US) + 500) >> 10,
                    0xff,
                );

                pkt_light_data.att_cmd_mut().value.val[17] = relay_time as u8;
            }

            app().mesh_manager.add_send_mesh_msg(&*pkt_light_data, 0, 0);
        }
    }
}

pub fn tx_packet_bridge()
{
    static TICK_BRIDGE_REPORT: AtomicU32 = AtomicU32::new(0);

    rf_set_tx_rx_off();

    sleep_us(100);

    rf_set_ble_access_code(PAIR_AC.get());
    rf_set_ble_crc_adv();

    let tick = read_reg_system_tick();
    if SLAVE_LISTEN_INTERVAL.get() * ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL as u32 - ONLINE_STATUS_COMP * CLOCK_SYS_CLOCK_1US * 1000 < tick - TICK_BRIDGE_REPORT.load(Ordering::Relaxed) {
        TICK_BRIDGE_REPORT.store(tick, Ordering::Relaxed);
        mesh_send_online_status();
    }
    app_bridge_cmd_handle(T_BRIDGE_CMD.get());
}

pub fn rf_link_slave_proc() {
    app().mesh_manager.mesh_pair_proc();
    update_connect_para();
}

pub fn is_add_packet_buf_ready() -> bool
{
    return (read_reg_dma_tx_wptr() - read_reg_dma_tx_rptr() & 7) < 3;
}

pub fn rf_link_add_tx_packet(packet: &Packet) -> bool
{
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

#[cfg_attr(test, mry::mry)]
pub fn rf_link_slave_read_status_par_init()
{
    SLAVE_STATUS_BUFFER_WPTR.set(0);
    SLAVE_STATUS_BUFFER_RPTR.set(0);
    *SLAVE_STAT_SNO.lock() = [0; 3];
}

#[cfg_attr(test, mry::mry)]
pub fn rf_link_slave_read_status_stop()
{
    SLAVE_READ_STATUS_BUSY.set(0);
    SLAVE_READ_STATUS_UNICAST_FLAG.set(false);
    SLAVE_DATA_VALID.set(0);

    rf_link_slave_read_status_par_init();
}

pub fn rf_ota_save_data(data: &[u8]) -> OtaState
{
    let addr = CUR_OTA_FLASH_ADDR.get() + FLASH_ADR_LIGHT_NEW_FW;
    flash_write_page(addr, data.len() as u32, data.as_ptr());

    let mut tmp = [0u8; 0x10];
    flash_read_page(addr, data.len() as u32, tmp.as_mut_ptr());

    if data == &tmp[..data.len()] {
        CUR_OTA_FLASH_ADDR.set(CUR_OTA_FLASH_ADDR.get() + data.len() as u32);
        return OtaState::Continue;
    } else {
        return OtaState::Error;
    }
}

#[cfg_attr(test, mry::mry)]
pub fn rf_link_match_group_mac(pkt: &Packet) -> (bool, bool)
{
    let mut group_match = false;
    let mut device_match = false;

    if pkt.ll_app().value.dst & !DEVICE_ADDR_MASK_DEFAULT != 0 {
        for addr in *GROUP_ADDRESS.lock() {
            if addr == pkt.ll_app().value.dst {
                group_match = true;
                break;
            }
        }
        if pkt.ll_app().value.dst == 0xffff {
            group_match = true;
        }
    } else {
        let addr = pkt.ll_app().value.dst & DEVICE_ADDR_MASK_DEFAULT;
        if addr == 0 || addr == DEVICE_ADDRESS.get() {
            device_match = true;
        }
    }

    (group_match, device_match)
}

pub fn mesh_construct_packet(sno: u32, dst: u16, cmd_op_para: &[u8], retransmit_count: u8, send_ack: bool) -> Packet
{
    assert!(cmd_op_para.len() > 2);
    assert!(cmd_op_para.len() <= 13);

    let device_address = DEVICE_ADDRESS.get();

    let mut pkt = MeshPkt {
        head: PacketL2capHead {
            dma_len: 0x27,
            _type: 2,
            rf_len: 0x25,
            l2cap_len: 0x21,
            chan_id: 0xff03,
        },
        src_tx: device_address,
        handle1: 0,
        sno: [0; 3],
        src_adr: device_address,
        dst_adr: dst,
        op: 0,
        vendor_id: 0,
        par: [0; 10],
        internal_par1: [0; 5],
        ttl: 0,
        internal_par2: [0; 4],
        no_use: [0; 4],
    };

    // Set send number
    pkt.sno[0] = sno as u8;
    pkt.sno[1] = (sno >> 8) as u8;
    pkt.sno[2] = (sno >> 16) as u8;

    // todo: replace this unsafe
    // Set opcode
    // pkt.op = cmd_op_para[0];

    unsafe {
        slice::from_raw_parts_mut(addr_of_mut!(pkt.op), cmd_op_para.len()).copy_from_slice(cmd_op_para)
    }

    pkt.internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT] = retransmit_count;
    pkt.internal_par1[INTERNAL_PAR_SEND_ACK] = if send_ack { 1 } else { 0 };

    Packet { mesh: pkt }
}

pub fn mesh_report_status_enable(enable: bool)
{
    let mut mesh_node_mask = MESH_NODE_MASK.lock();
    if enable {
        if MESH_NODE_MAX.get() >> 5 != 0 {
            mesh_node_mask.iter_mut().for_each(|v| { *v = 0xfffffffe });
        }

        if MESH_NODE_MAX.get() & 0x1f != 0 {
            mesh_node_mask[MESH_NODE_MAX.get() as usize >> 5] = (1 << (MESH_NODE_MAX.get() & 0x1f)) - 1;
        }
    }

    MESH_NODE_REPORT_ENABLE.set(enable);
}

pub fn mesh_report_status_enable_mask(data: &[u8])
{
    let mut mesh_node_mask = MESH_NODE_MASK.lock();
    let mut mesh_node_st = MESH_NODE_ST.lock();

    MESH_NODE_REPORT_ENABLE.set(data[0] != 0);
    if MESH_NODE_REPORT_ENABLE.get() && data.len() > 1 {
        for index in 1..data.len() {
            if MESH_NODE_MAX.get() != 0 {
                mesh_node_st.iter_mut().enumerate().for_each(|(i, v)| {
                    if data[index] == v.val.dev_adr {
                        mesh_node_mask[i >> 5] |= 1 << (i & 0x1f);
                    }
                });
            }
        }
    }
}

#[cfg_attr(test, mry::mry)]
pub fn rf_link_delete_pair()
{
    let mut key = [0u8; 16 * 3];

    key[0..0x10].copy_from_slice(&*PAIR_CONFIG_MESH_NAME.lock());
    key[0x10..0x20].copy_from_slice(&*PAIR_CONFIG_MESH_PWD.lock());

    pair_set_key(&key);
    pair_save_key();

    PAIR_LOGIN_OK.set(false);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdk::packet_types::*;
    use core::mem::size_of;

    #[test]
    fn test_rf_link_get_op_para_one_byte_op() {
        // Create a test packet with a 1-byte operation code
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead {
                    dma_len: 0x27,
                    _type: 2,
                    rf_len: 0x25,
                    l2cap_len: 20, // l2cap_len = 20 (10 + op_len + params_len)
                    chan_id: 0xff03,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: PacketAttValue::default(),
            }
        };

        // Set op code byte (not MSB set, so 1 byte op)
        packet.att_cmd_mut().value.val[0] = 0x42; // Some op code without MSB set
        
        // Set some parameter bytes
        packet.att_cmd_mut().value.val[1] = 0xA1;
        packet.att_cmd_mut().value.val[2] = 0xB2;
        packet.att_cmd_mut().value.val[3] = 0xC3;

        // Call the function
        let (success, op, op_len, params, params_len) = parse_ble_packet_op_params(&packet, false);

        // Verify results
        assert!(success);
        assert_eq!(op[0], 0x42);
        assert_eq!(op_len, 1);
        assert_eq!(params[0], 0xA1);
        assert_eq!(params[1], 0xB2);
        assert_eq!(params[2], 0xC3);
        assert_eq!(params_len, 9); // l2cap_len - 10 - op_len = 20 - 10 - 1 = 9
    }

    #[test]
    fn test_rf_link_get_op_para_two_byte_op() {
        // Create a test packet with a 2-byte operation code
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead {
                    dma_len: 0x27,
                    _type: 2,
                    rf_len: 0x25,
                    l2cap_len: 21, // l2cap_len = 21 (10 + op_len + params_len)
                    chan_id: 0xff03,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: PacketAttValue::default(),
            }
        };

        // Set op code bytes (MSB set but not 11 in top bits, so 2 byte op)
        packet.att_cmd_mut().value.val[0] = 0x82; // 10... in top bits
        packet.att_cmd_mut().value.val[1] = 0xAB;
        
        // Set some parameter bytes
        packet.att_cmd_mut().value.val[2] = 0xC1;
        packet.att_cmd_mut().value.val[3] = 0xD2;

        // Call the function
        let (success, op, op_len, params, params_len) = parse_ble_packet_op_params(&packet, false);

        // Verify results
        assert!(success);
        assert_eq!(op[0], 0x82);
        assert_eq!(op[1], 0xAB);
        assert_eq!(op_len, 2);
        assert_eq!(params[0], 0xC1);
        assert_eq!(params[1], 0xD2);
        assert_eq!(params_len, 9); // l2cap_len - 10 - op_len = 21 - 10 - 2 = 9
    }

    #[test]
    fn test_rf_link_get_op_para_three_byte_op() {
        // Create a test packet with a 3-byte operation code
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead {
                    dma_len: 0x27,
                    _type: 2,
                    rf_len: 0x25,
                    l2cap_len: 22, // l2cap_len = 22 (10 + op_len + params_len)
                    chan_id: 0xff03,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: PacketAttValue::default(),
            }
        };

        // Set op code bytes (MSB set with 11 in top bits, so 3 byte op)
        packet.att_cmd_mut().value.val[0] = 0xC5; // 11... in top bits
        packet.att_cmd_mut().value.val[1] = 0xAB;
        packet.att_cmd_mut().value.val[2] = 0xCD;
        
        // Set some parameter bytes
        packet.att_cmd_mut().value.val[3] = 0xE1;
        packet.att_cmd_mut().value.val[4] = 0xF2;

        // Call the function
        let (success, op, op_len, params, params_len) = parse_ble_packet_op_params(&packet, false);

        // Verify results
        assert!(success);
        assert_eq!(op[0], 0xC5);
        assert_eq!(op[1], 0xAB);
        assert_eq!(op[2], 0xCD);
        assert_eq!(op_len, 3);
        assert_eq!(params[0], 0xE1);
        assert_eq!(params[1], 0xF2);
        assert_eq!(params_len, 9); // l2cap_len - 10 - op_len = 22 - 10 - 3 = 9
    }

    #[test]
    fn test_rf_link_get_op_para_special_op_code_6() {
        // Create a test packet with op code that has 0x6 in the lower 6 bits
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead {
                    dma_len: 0x27,
                    _type: 2,
                    rf_len: 0x25,
                    l2cap_len: 30, // l2cap_len = 30 (enough for larger parameters)
                    chan_id: 0xff03,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: PacketAttValue::default(),
            }
        };

        // Set op code byte with 0x6 in the lower 6 bits
        packet.att_cmd_mut().value.val[0] = 0x06; // 0x06 & 0x3f = 0x06
        
        // Set parameter bytes
        for i in 0..15 {
            packet.att_cmd_mut().value.val[i + 1] = i as u8;
        }

        // Call the function with mesh_flag = true to test special handling
        let (success, op, op_len, params, params_len) = parse_ble_packet_op_params(&packet, true);

        // Verify results
        assert!(success);
        assert_eq!(op[0], 0x06);
        assert_eq!(op_len, 1);
        // Check that we got the expected parameters with delta of 5
        assert_eq!(params_len, 14); // With the special handling for op code 6 and mesh_flag=true
        for i in 0..14 {
            assert_eq!(params[i as usize], i as u8);
        }
    }

    #[test]
    fn test_rf_link_get_op_para_too_long_params() {
        // Create a test packet with parameters that are too long
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead {
                    dma_len: 0x27,
                    _type: 2,
                    rf_len: 0x25,
                    l2cap_len: 30, // l2cap_len = 30 (makes params too long)
                    chan_id: 0xff03,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: PacketAttValue::default(),
            }
        };

        // Set op code byte (not special case)
        packet.att_cmd_mut().value.val[0] = 0x01;
        
        // Set many parameter bytes
        for i in 0..20 {
            packet.att_cmd_mut().value.val[i + 1] = i as u8;
        }

        // Call the function
        let (success, op, op_len, params, params_len) = parse_ble_packet_op_params(&packet, false);

        // Verify results
        assert!(!success); // Should fail due to too-long parameters
        assert_eq!(op[0], 0x01);
        assert_eq!(op_len, 1);
        assert_eq!(params_len, 0); // params_len should be set to 0 on failure
    }

    #[test]
    fn test_rf_link_get_op_para_mesh_flag() {
        // Create a test packet with mesh_flag = true and mesh_flag = false
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead {
                    dma_len: 0x27,
                    _type: 2,
                    rf_len: 0x25,
                    l2cap_len: 21, // Reduced from 25 to ensure parameters don't exceed max length for non-mesh mode
                    chan_id: 0xff03,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: PacketAttValue::default(),
            }
        };

        // Set op code byte (1 byte op code)
        packet.att_cmd_mut().value.val[0] = 0x42; // Op code without MSB set
        
        // Set parameter bytes (reduced number of parameters to fit within limits)
        for i in 0..5 {
            packet.att_cmd_mut().value.val[i + 1] = 0xA0 + i as u8;
        }

        // Test with mesh_flag = false
        let (success_no_mesh, op_no_mesh, op_len_no_mesh, params_no_mesh, params_len_no_mesh) = 
            parse_ble_packet_op_params(&packet, false);

        // Test with mesh_flag = true
        let (success_mesh, op_mesh, op_len_mesh, params_mesh, params_len_mesh) = 
            parse_ble_packet_op_params(&packet, true);

        // Verify results are different between mesh and non-mesh
        assert!(success_no_mesh, "Non-mesh mode should succeed");
        assert!(success_mesh, "Mesh mode should succeed");
        assert_eq!(op_no_mesh, op_mesh); // Op code should be the same
        assert_eq!(op_len_no_mesh, op_len_mesh); // Op length should be the same
        assert_ne!(params_len_no_mesh, params_len_mesh); // Param lengths should differ
    }
}


