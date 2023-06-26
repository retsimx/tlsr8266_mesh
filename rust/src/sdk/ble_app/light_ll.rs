use core::cmp::min;
use core::mem::size_of;
use core::ptr::{addr_of, addr_of_mut, null};
use core::slice;
use core::sync::atomic::{AtomicU32, AtomicU8, AtomicUsize, Ordering};

use crate::{app, BIT};
use crate::common::{rf_update_conn_para, SYS_CHN_LISTEN, update_ble_parameter_cb};
use crate::config::FLASH_ADR_LIGHT_NEW_FW;
use crate::embassy::time_driver::clock_time64;
use crate::main_light::{rf_link_data_callback, rf_link_response_callback};
use crate::mesh::{MESH_NODE_ST_VAL_LEN, mesh_node_st_val_t};
use crate::sdk::ble_app::ble_ll_att::ble_ll_channel_table_calc;
use crate::sdk::ble_app::ble_ll_attribute::l2cap_att_handler;
use crate::sdk::ble_app::ble_ll_pair::{pair_dec_packet_mesh, pair_enc_packet, pair_init, pair_save_key, pair_set_key};
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::drivers::flash::{flash_read_page, flash_write_page};
use crate::sdk::light::{*};
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, clock_time_exceed, sleep_us};
use crate::sdk::mcu::register::{*};
use crate::state::{DEVICE_ADDRESS, PAIR_AC, SECURITY_ENABLE, SimplifyLS, State};
use crate::uart_manager::light_mesh_rx_cb;

fn mesh_node_update_status(state: &mut State, pkt: &[mesh_node_st_val_t]) -> u32
{
    let mut src_index = 0;
    let mut result = 0xfffffffe;
    let tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;
    while src_index < pkt.len() && pkt[src_index].dev_adr != 0 {
        if DEVICE_ADDRESS.get() as u8 != pkt[src_index].dev_adr {
            let mesh_node_max = state.mesh_node_max;
            let mut current_index = 1;
            let mut mesh_node_st = &mut state.mesh_node_st[current_index];
            if mesh_node_max >= 2 {
                if mesh_node_st.val.dev_adr != pkt[src_index].dev_adr {
                    for tidx in 1..MESH_NODE_MAX_NUM {
                        current_index = tidx;
                        mesh_node_st = &mut state.mesh_node_st[current_index];

                        if mesh_node_max <= tidx as u8 || pkt[src_index].dev_adr == mesh_node_st.val.dev_adr {
                            break;
                        }
                    }
                }
            }

            if MESH_NODE_MAX_NUM == current_index {
                return 1;
            }

            if mesh_node_max as usize == current_index {
                state.mesh_node_max += 1;

                mesh_node_st.val = pkt[src_index];
                mesh_node_st.tick = tick;

                state.mesh_node_mask[mesh_node_max as usize >> 5] |= 1 << (mesh_node_max & 0x1f);

                result = mesh_node_max as u32;
            } else if current_index < mesh_node_max as usize {
                let sn_difference = pkt[src_index].sn - mesh_node_st.val.sn;
                let par_match = pkt[src_index].par == mesh_node_st.val.par;

                // todo: Why the divided by two here?
                let timeout= (ONLINE_STATUS_TIMEOUT * 1000) / 2;

                result = current_index as u32;
                if sn_difference - 2 < 0x3f || (sn_difference != 0 && (mesh_node_st.tick == 0 || (((timeout * CLOCK_SYS_CLOCK_1US) >> 0x10) as u16) < tick - mesh_node_st.tick)) {
                    mesh_node_st.val = pkt[src_index];

                    if !par_match || mesh_node_st.tick == 0 {
                        state.mesh_node_mask[current_index >> 5] |= 1 << (current_index & 0x1f);
                    }

                    mesh_node_st.tick = tick;
                }
            }
        }

        src_index += 1;
    }
    return 1;
}

fn is_exist_in_rc_pkt_buf(state: &mut State, opcode: u8, cmd_pkt: &AppCmdValue) -> bool
{
    state.rc_pkt_buf.iter().any(|v| v.op == opcode && v.sno == cmd_pkt.sno)
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

fn rc_pkt_buf_push(state: &mut State, opcode: u8, cmd_pkt: &AppCmdValue)
{
    if rf_link_is_notify_rsp(opcode) || is_exist_in_rc_pkt_buf(state, opcode, cmd_pkt) {
        return;
    }

    if state.rc_pkt_buf.is_full() {
        state.rc_pkt_buf.pop_back();
    }

    state.rc_pkt_buf.push_front(
        PktBuf {
            op: opcode,
            sno: cmd_pkt.sno,
            notify_ok_flag: false,
            sno2: cmd_pkt.par[8..10].try_into().unwrap(),
        }
    ).unwrap();
}

fn req_cmd_is_notify_ok(state: &mut State, opcode: u8, cmd_pkt: &AppCmdValue) -> bool
{
    state.rc_pkt_buf.iter().any(|pkt| {
        pkt.op == opcode && pkt.sno == cmd_pkt.sno && pkt.notify_ok_flag
    })
}

fn req_cmd_set_notify_ok_flag(state: &mut State, opcode: u8, cmd_pkt: &AppCmdValue)
{
    state.rc_pkt_buf.iter_mut().filter(
        |v| { v.op == opcode && v.sno == cmd_pkt.sno }
    ).for_each(
        |v| { v.notify_ok_flag = true }
    );
}

pub fn copy_par_user_all(state: &mut State, params_len: u32, ptr: *const u8)
{
    state.pkt_light_status.value.val[3..3 + params_len as usize].copy_from_slice(
        unsafe {
            slice::from_raw_parts(ptr, params_len as usize)
        }
    );
}

fn rf_link_slave_notify_req_mask(state: &mut State, adr: u8)
{
    if state.slave_read_status_busy != 0 && (DEVICE_ADDRESS.get() as u8 != adr || state.slave_read_status_busy == 0x21) {
        if state.slave_read_status_unicast_flag == 0 {
            if state.pkt_light_data.value.val[8..0xd].iter().any(|v| *v == adr) {
                return;
            }

            state.pkt_light_data.value.val[(state.notify_req_mask_idx + 8) as usize] = adr;
            state.notify_req_mask_idx = (state.notify_req_mask_idx + 1) % 5;
        } else {
            state.slave_data_valid = 0;
        }
    }
}

pub fn rf_link_slave_add_status(state: &mut State, packet: &MeshPkt)
{
    let mut result = false;

    if app().mesh_manager.mesh_pair_notify_refresh(unsafe { &*(addr_of!(*packet) as *const PacketAttCmd) }) != 2 {
        if state.slave_status_record_idx != 0 {
            for st_rec in state.slave_status_record {
                if packet.src_adr as u8 == st_rec.adr[0] {
                    rf_link_slave_notify_req_mask(state, packet.src_adr as u8);
                    return;
                }
            }
        }

        result = false;
        if (state.slave_status_buffer_wptr + 1) % BUFF_RESPONSE_PACKET_COUNT != state.slave_status_buffer_rptr && state.slave_status_record_idx < MESH_NODE_MAX_NUM {
            state.slave_status_record[state.slave_status_record_idx].adr[0] = packet.src_adr as u8;
            state.slave_status_record_idx += 1;
            rf_link_slave_notify_req_mask(state, packet.src_adr as u8);

            // Update the buffer write pointer
            let index = state.slave_status_buffer_wptr;
            state.slave_status_buffer_wptr = (index + 1) % BUFF_RESPONSE_PACKET_COUNT;

            let st_ptr = &mut state.buff_response[index];
            st_ptr.dma_len = 0x1d;
            st_ptr._type = 2;
            st_ptr.rf_len = 0x1b;
            st_ptr.l2cap = 0x17;
            st_ptr.chanid = 4;
            st_ptr.att = 0x1b;
            st_ptr.hl = 0x12;

            st_ptr.dat[0..0x14].copy_from_slice(
                unsafe {
                    slice::from_raw_parts(addr_of!(packet.sno) as *const u8, 0x14)
                }
            );

            if packet.internal_par1[1] == 0 {
                st_ptr.dat[0x12] = packet.par[9];
                st_ptr.dat[0x13] = packet.internal_par1[0];
            } else if packet.internal_par1[1] != 4 && packet.internal_par1[1] != 5 && packet.internal_par1[1] != 8 {
                if packet.internal_par1[1] == 6 {
                    st_ptr.dat[0x11..0x11 + 3].fill(0xff);
                } else if packet.internal_par1[1] != 7 && packet.internal_par1[1] != 9 {
                    st_ptr.dat[0x12..0x12 + 2].fill(0xff);
                }
            }
            st_ptr.dat[3] = packet.src_adr as u8;
            st_ptr.dat[4] = (packet.src_adr >> 8) as u8;
            result = true;
        }
    }
}

pub fn rf_link_rc_data(state: &mut State, packet: &mut MeshPkt, needs_decode: bool) -> bool {
    if state.slave_link_connected {
        if 0x3fffffffi32 < (read_reg_system_tick_irq() as i32 - read_reg_system_tick() as i32) - (CLOCK_SYS_CLOCK_1US * 1000) as i32 {
            return false;
        }
    }

    if packet.rf_len != 0x25 || packet.l2cap_len != 0x21 || packet._type & 3 != 2 || packet.chan_id == 0xeeff || (needs_decode && unsafe { !pair_dec_packet_mesh(state, packet) }) {
        return false;
    }

    // Check if this is a node update packet (pkt adv status)
    if packet.chan_id == 0xffff {
        if MESH_NODE_ST_VAL_LEN == 4 {
            if packet.internal_par1[LAST_RELAY_TIME] != 0xa5 || packet.internal_par1[CURRENT_RELAY_COUNT] != 0xa5 {
                return false;
            }
        }

        // todo: ttl? Probably packet is not the right type in this check
        if packet.ttl != 0xa5 || packet.internal_par2[0] != 0xa5 {
            return false;
        }

        mesh_node_update_status(state, unsafe { slice::from_raw_parts(addr_of!(packet.sno) as *const mesh_node_st_val_t, 0x1a / MESH_NODE_ST_VAL_LEN) });
        return false;
    }

    // Don't do anything if this packet has been relayed too many times
    if MAX_RELAY_NUM + 6 < packet.internal_par1[CURRENT_RELAY_COUNT] {
        return false;
    }

    // Parse the opcode and parameters from the packet
    let mut op_cmd: [u8; 3] = [0; 3];
    let mut op_cmd_len: u8 = 0;
    let mut params: [u8; 16] = [0; 16];
    let mut params_len: u8 = 0;
    let l2cap_data = unsafe { &*(addr_of!(packet.l2cap_len) as *const PacketL2capData) };
    if !rf_link_get_op_para(l2cap_data, &mut op_cmd, &mut op_cmd_len, &mut params, &mut params_len, true) {
        return false;
    }

    // Get the opcode
    let mut op = 0;
    if op_cmd_len == 3 {
        op = op_cmd[0] & 0x3f;
    }

    let cmd_pkt = unsafe { &*(addr_of!(packet.sno) as *const AppCmdValue) };
    let no_match_slave_sno = cmd_pkt.sno != unsafe { slice::from_raw_parts(addr_of!(state.slave_sno) as *const u8, 3) };
    let pkt_exists_in_buf = is_exist_in_rc_pkt_buf(state, op, cmd_pkt);
    let pkt_valid = (no_match_slave_sno || state.slave_link_cmd != op) && !pkt_exists_in_buf;

    // If the slave link is connected (Android) and the dest address is us, then we should forward
    // the packet on to the slave link
    let mut should_notify = {
        packet.dst_adr == DEVICE_ADDRESS.get() && state.slave_link_connected
    };

    // See if we should call the message callback
    if pkt_valid || (rf_link_is_notify_rsp(op) && should_notify)
    {
        light_mesh_rx_cb(cmd_pkt);
    }

    // If we should notify, and the opcode is a response opcode and the slave (Android) is waiting
    // for a response, then send this response to the slave
    if rf_link_is_notify_rsp(op) && should_notify {
        if state.slave_read_status_busy != op || cmd_pkt.sno != state.slave_stat_sno {
            return false;
        }
        rf_link_slave_add_status(state, &packet);
        return false;
    }

    state.rcv_pkt_ttc = packet.par[9];

    let (group_match, device_match) = rf_link_match_group_mac(state, cmd_pkt);

    // Record the packet so we don't handle it again if we receive it again
    if group_match || device_match {
        if pkt_valid {
            rc_pkt_buf_push(state, op, cmd_pkt);
            rf_link_data_callback(state, l2cap_data);
        }
    }

    // Only handle notify requests once
    if rf_link_is_notify_req(state, op) {
        state.slave_read_status_response = device_match;

        if req_cmd_is_notify_ok(state, op, cmd_pkt) {
            state.slave_read_status_response = false;
        } else {
            req_cmd_set_notify_ok_flag(state, op, cmd_pkt);
        }
    }

    let relay_count = packet.internal_par1[CURRENT_RELAY_COUNT];

    packet.dma_len = (packet.l2cap_len as u32 + 6) & 0xffffff;
    packet.rf_len = packet.l2cap_len as u8 + 4;

    let mut ttl = relay_count;

    if no_match_slave_sno || state.slave_link_cmd != op {
        packet.src_tx = DEVICE_ADDRESS.get();

        state.slave_sno = cmd_pkt.sno;
        state.slave_link_cmd = op;
    } else {
        ttl = state.org_ttl;
    }

    state.org_ttl = ttl;

    if rf_link_is_notify_req(state, op) && state.slave_read_status_response {
        state.pkt_light_status.value.sno = cmd_pkt.sno;
        packet.src_tx = DEVICE_ADDRESS.get();
        if op == LGT_CMD_LIGHT_READ_STATUS {
            state.pkt_light_status.value.val[15] = GET_STATUS;
            if pkt_valid {
                state.pkt_light_status.value.val[13] = packet.par[9];
                state.pkt_light_status.value.val[18] = packet.internal_par1[CURRENT_RELAY_COUNT];
                if MAX_RELAY_NUM < packet.internal_par1[CURRENT_RELAY_COUNT] {
                    state.pkt_light_status.value.val[14] = 0;
                } else {
                    state.pkt_light_status.value.val[14] = MAX_RELAY_NUM - packet.internal_par1[CURRENT_RELAY_COUNT];
                }
            }
        } else if op == LGT_CMD_LIGHT_GRP_REQ {
            state.pkt_light_status.value.val[15] = packet.internal_par1[1];
        } else if op == LGT_CMD_LIGHT_CONFIG_GRP {
            state.pkt_light_status.value.val[15] = GET_GROUP1;
        } else if op == LGT_CMD_CONFIG_DEV_ADDR {
            state.pkt_light_status.value.val[15] = GET_DEV_ADDR;
        } else if op == LGT_CMD_USER_NOTIFY_REQ {
            state.pkt_light_status.value.val[15] = GET_USER_NOTIFY;
        } else if op == LGT_CMD_START_OTA_REQ {
            state.pkt_light_status.value.val[15] = CMD_START_OTA;
        } else if op == LGT_CMD_OTA_DATA_REQ {
            state.pkt_light_status.value.val[15] = CMD_OTA_DATA;
        } else if op == LGT_CMD_END_OTA_REQ {
            state.pkt_light_status.value.val[15] = CMD_END_OTA;
        }
        unsafe { copy_par_user_all(state, params_len as u32, (addr_of!(packet.vendor_id) as u32 + 1) as *const u8); }
        if (no_match_slave_sno || state.slave_link_cmd != op) || params[1] != 0 {
            state.pkt_light_status.value.src.copy_from_slice(unsafe { slice::from_raw_parts(addr_of!(packet.src_adr) as *const u8, 2) });

            let mut request_params: PacketAttValue = PacketAttValue::default();

            unsafe {
                slice::from_raw_parts_mut(addr_of_mut!(request_params) as *mut u8, size_of::<PacketAttValue>()).copy_from_slice(
                    slice::from_raw_parts(
                        addr_of!(*cmd_pkt) as *const u8,
                        size_of::<PacketAttValue>(),
                    )
                )
            }

            let mut result_data = state.pkt_light_status.value;
            if rf_link_response_callback(state, &mut result_data, &request_params) {
                state.pkt_light_status.value = result_data;
                state.pkt_light_status._type |= BIT!(7);

                app().mesh_manager.add_send_mesh_msg(unsafe { &*(addr_of!(state.pkt_light_status) as *const MeshPkt) }, 0);
            }
        }
    }

    // Relay the message if it's not for us
    if relay_count != 0 && state.org_ttl == relay_count && !device_match
    {
        if state.slave_read_status_busy == 0 || !rf_link_is_notify_rsp(op) {
            packet.internal_par1[CURRENT_RELAY_COUNT] = relay_count - 1;
        }

        packet._type |= BIT!(7);

        if op != LGT_CMD_LIGHT_STATUS {
            rf_link_proc_ttc(state.rcv_pkt_time, state.rcv_pkt_ttc, packet);
        }

        if rf_link_is_notify_req(state, op) {
            let relay_time = min(
                ((((read_reg_system_tick() - state.rcv_pkt_time) / CLOCK_SYS_CLOCK_1US) + 500) >> 10) + packet.internal_par1[LAST_RELAY_TIME] as u32,
                0xff
            );

            packet.internal_par1[LAST_RELAY_TIME] = relay_time as u8;
        }

        let mut delay = 100;
        if !state.slave_link_connected {
            delay = 8000 - (((read_reg_system_tick() as u16 ^ read_reg_rnd_number()) & 0xf) * 500);
        }

        app().mesh_manager.add_send_mesh_msg(&packet, clock_time64() + (delay as u64 * CLOCK_SYS_CLOCK_1US as u64));
    }

    return true;
}

pub fn rf_link_slave_data(state: &mut State, packet: &PacketLlData, time: u32) -> bool {
    let rf_len: u8 = packet.rf_len;
    let chanid: u16 = packet.chanid;

    if packet._type as i32 * 0x1000000 >= -1 {
        if (packet._type & 3) == 2 {
            if 6 < chanid {
                return false;
            }
        }

        if chanid == 5 {
            rf_update_conn_para(state, packet);
        }

        rf_link_timing_adjust(state, time);
        if rf_len < 6 {
            if rf_len == 0 {
                return false;
            }
        } else {
            if packet._type & 3 == 3 && packet.l2cap_low == 1 {
                state.slave_timing_update = 1;
                state.slave_instant_next = ((packet.sno as u16) << 8) | packet.hh as u16;
                state.slave_chn_map.iter_mut().enumerate().for_each(|(i, v)| {
                    *v = unsafe { *addr_of!(packet.l2cap_high).offset(i as isize) };
                });

                return true;
            }

            if rf_len == 0xc && packet._type & 3 == 3 && packet.l2cap_low == 0 {
                state.slave_interval_old = state.slave_link_interval;
                state.slave_instant_next = packet.group;
                state.slave_window_size_update = (packet.l2cap_high as u32 * 1250 + 1300) * CLOCK_SYS_CLOCK_1US;
                state.slave_timing_update = 2;
                state.ble_conn_interval = CLOCK_SYS_CLOCK_1US * 1250 * unsafe { (*addr_of!(packet.att) as *const u16) } as u32;
                state.ble_conn_offset = packet.chanid as u32 * CLOCK_SYS_CLOCK_1US * 1250;
                state.ble_conn_timeout = packet.nid as u32 * 10000;
                return false;
            }
        }
        let (res_pkt, len) = unsafe { l2cap_att_handler(state, addr_of!(*packet) as *const PacketL2capData) };
        if res_pkt != null() && !rf_link_add_tx_packet(state, unsafe { res_pkt as *const PacketAttCmd }, len) {

            state.add_tx_packet_rsp_failed += 1;
        }
        return false;
    }

    return false;
}

pub fn rf_link_timing_adjust(state: &mut State, time: u32)
{

    if state.slave_timing_adjust_enable {
        state.slave_timing_adjust_enable = false;
        if time - state.slave_tick_brx < CLOCK_SYS_CLOCK_1US * 700 {
            state.slave_next_connect_tick -= CLOCK_SYS_CLOCK_1US * 200;
        } else if CLOCK_SYS_CLOCK_1US * 1100 < time - state.slave_tick_brx {
            state.slave_next_connect_tick += CLOCK_SYS_CLOCK_1US * 200;
        }
    }
}

fn check_par_con(packet: &PacketLlInit) -> bool
{
    if packet.interval - 6 & 0xffff < 0xc7b && packet.wsize != 0 && packet.wsize < 9 && 9 < packet.timeout && packet.timeout < 0xc81 && packet.woffset <= packet.interval && packet.hop != 0 &&
        packet.chm.iter().any(|v| { *v != 0 }) {
        if packet.latency == 0 {
            return false;
        }

        if packet.latency as u32 <= ((packet.interval as u32) << 3) / packet.interval as u32 {
            return false;
        }
    }
    return true;
}

pub fn rf_link_slave_connect(state: &mut State, packet: &PacketLlInit, time: u32) -> bool
{
    state.conn_update_successed = false;
    state.conn_update_cnt = 0;

    if state.slave_connection_enable || packet.scan_a == packet.adv_a {
        if check_par_con(packet) == false {
            rf_stop_trx();

            state.slave_window_offset = CLOCK_SYS_CLOCK_1US * 1250 * (packet.woffset as u32 + 1);

            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 1000 + read_reg_system_tick());
            write_reg_irq_src(0x100000);
            write_reg_system_tick_irq(time + state.slave_window_offset + CLOCK_SYS_CLOCK_1US * (0 - if packet.woffset == 0 { 500 } else { 700 }));
            if 0x80000000 < read_reg_system_tick_irq() - read_reg_system_tick() {
                write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick());
            }

            state.light_conn_sn_master = 0x80;
            state.slave_connected_tick = read_reg_system_tick();
            if SECURITY_ENABLE.get() {
                state.slave_first_connected_tick = state.slave_connected_tick;
            }

            state.slave_status_tick = ((packet.interval as u32 * 5) >> 2) as u8;
            state.slave_link_interval = packet.interval as u32 * CLOCK_SYS_CLOCK_1US * 1250;
            state.slave_window_size = (packet.wsize as u32 * 1250 + 1100) * CLOCK_SYS_CLOCK_1US;

            let tmp = state.slave_link_interval - CLOCK_SYS_CLOCK_1US * 1250;
            if tmp <= state.slave_window_size && state.slave_window_size - tmp != 0 {
                state.slave_window_size = tmp;
            }

            state.slave_link_time_out = packet.timeout as u32 * 10000;
            unsafe {
                slice::from_raw_parts_mut(
                    addr_of_mut!(state.pkt_init.scan_a) as *mut u8,
                    0x22,
                ).copy_from_slice(
                    slice::from_raw_parts(
                        addr_of!(packet.scan_a) as *const u8,
                        0x22,
                    )
                )
            };

            let chn_map = state.pkt_init.chm;
            ble_ll_channel_table_calc(state, &chn_map, true);

            // rf_set_ble_crc(&(state.pkt_init()).crcinit);
            write_reg_rf_crc(((state.pkt_init.crcinit[1] as u32) << 8) | ((state.pkt_init.crcinit[2] as u32) << 0x10) | state.pkt_init.crcinit[0] as u32);
            rf_reset_sn();

            state.slave_instant = 0;
            state.slave_timing_update2_flag = 0;
            state.slave_interval_old = 0;
            state.slave_timing_update2_ok_time = 0;
            state.slave_window_size_update = 0;

            pair_init(state);

            state.mesh_node_report_enable = false;

            state.mesh_node_mask.fill(0);

            state.p_st_handler = IrqHandlerStatus::Rx;
            state.need_update_connect_para = true;
            state.att_service_discover_tick = read_reg_system_tick() | 1;

            write_reg8(0x00800f04, 0x67);  // tx wail & settle time

            return true;
        }
    }
    return false;
}

pub fn light_check_tick_per_us(state: &mut State, ticks: u32)
{


    if ticks == 0x10 {
        state.t_scan_rsp_intvl = 0;
    } else if ticks == 0x20 || ticks != 0x30 {
        state.t_scan_rsp_intvl = 0x92;
    } else {
        state.t_scan_rsp_intvl = 0x93;
    }
}

pub fn rf_link_slave_pairing_enable(state: &mut State, enable: bool)
{


    state.slave_connection_enable = enable;
    state.slave_adv_enable = enable;
}

pub fn vendor_id_init(state: &mut State, vendor_id: u16)
{


    state.pkt_light_report.value.val[1] = vendor_id as u8;
    state.pkt_light_report.value.val[2] = (vendor_id >> 8) as u8;

    state.g_vendor_id = vendor_id;
}

// param st is 2bytes = lumen% + rsv(0xFF)  // rf pkt : device_address+sn+lumen+rsv);
pub fn ll_device_status_update(state: &mut State, val_par: &[u8])
{
    state.mesh_node_st[0].val.par.copy_from_slice(val_par);
    state.mesh_node_st[0].tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;

    state.mesh_node_mask[0] |= 1;
}

pub fn setup_ble_parameter_start(state: &mut State, delay: u16, mut interval_min: u16, mut interval_max: u16, timeout: u32) -> u32
{
    let mut invalid = false;

    if interval_max | interval_min == 0 {
        state.update_interval_user_max = state.slave_link_interval as u16;
        state.update_interval_user_min = state.slave_link_interval as u16;
        interval_max = state.update_interval_user_max;
        interval_min = state.update_interval_user_min;
    } else {
        if interval_min < INTERVAL_THRESHOLD {
            invalid = true;
        }

        state.update_interval_user_min = interval_min;
        state.update_interval_user_max = interval_max;
    }

    if timeout == 0 {
        state.update_timeout_user = state.slave_link_time_out;
    } else {
        state.update_timeout_user = timeout;
        if timeout < 100 {
            state.update_interval_user_max = 0;
            state.update_interval_user_min = 0;
            state.update_timeout_user = 0;
            return 0xfffffffd;
        }
    }
    if invalid == false {
        state.update_interval_flag = delay;
        state.update_interval_time = true;
        return 0;
    }
    state.update_timeout_user = 0;
    state.update_interval_user_min = 0;
    state.update_interval_user_max = 0;
    return 0xfffffffe;
}

fn update_connect_para(state: &mut State)
{
    if state.need_update_connect_para {
        if state.att_service_discover_tick != 0 {
            if UPDATE_CONNECT_PARA_DELAY_MS * CLOCK_SYS_CLOCK_1US * 1000 < read_reg_system_tick() - state.att_service_discover_tick {
                state.need_update_connect_para = false;
                state.att_service_discover_tick = 0;

                update_ble_parameter_cb(state);
            }
        }
    }
}

fn set_mesh_info_time_handle(state: &mut State)
{
    if state.set_mesh_info_time != 0 && !state.set_mesh_info_expired_flag && state.set_mesh_info_time * CLOCK_SYS_CLOCK_1US * 1000000 < read_reg_system_tick() {
        state.set_mesh_info_expired_flag = true;
    }
}

pub fn mesh_node_flush_status(state: &mut State)
{
    static TICK_NODE_REPORT: AtomicU32 = AtomicU32::new(0);

    // Only report status every 500ms
    if !clock_time_exceed(TICK_NODE_REPORT.load(Ordering::Relaxed), 500000) {
        return;
    }

    let tick = read_reg_system_tick();
    TICK_NODE_REPORT.store(tick, Ordering::Relaxed);

    // Iterate over each mesh node and check if it's timed out
    for count in 1..state.mesh_node_max as usize {
        if state.mesh_node_st[count].tick != 0 && (CLOCK_SYS_CLOCK_1US * ONLINE_STATUS_TIMEOUT * 1000) >> 0x10 < (tick >> 0x10 | 1) - state.mesh_node_st[count].tick as u32 {
            state.mesh_node_st[count].tick = 0;

            // Set the bit in the mask so that the status is reported (Since the device has changed to offline now)
            state.mesh_node_mask[count >> 5] |= 1 << (count & 0x1f);
        }
    }
}

fn mesh_node_keep_alive(state: &mut State)
{
    state.device_node_sn = state.device_node_sn + 1;
    if state.device_node_sn == 0 {
        state.device_node_sn = 1;
    }

    state.mesh_node_st[0].val.sn = state.device_node_sn;
    state.mesh_node_st[0].tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;
}

fn mesh_node_adv_status(state: &mut State, p_data: &mut [u8]) -> u32
{
    static SEND_ONLINE_STATUS_CNT: AtomicU8 = AtomicU8::new(0xC8);
    static MESH_NODE_CUR: AtomicUsize = AtomicUsize::new(1);

    // Clear the result data
    p_data.fill(0);

    // Get the max number of elements the result data can hold
    let mut elems = p_data.len() / MESH_NODE_ST_VAL_LEN;
    if (state.mesh_node_max as usize) < p_data.len() / MESH_NODE_ST_VAL_LEN {
        elems = state.mesh_node_max as usize;
    }

    // Copy our status in to the result data first
    p_data[0..MESH_NODE_ST_VAL_LEN].copy_from_slice(
        unsafe {
            slice::from_raw_parts(
                addr_of!(state.mesh_node_st[0].val) as *const u8,
                MESH_NODE_ST_VAL_LEN,
            )
        }
    );

    // Update our own record to keep our status record in sync
    mesh_node_keep_alive(state);

    let max_node = state.mesh_node_max as usize;
    let mut count = 1;

    let mut out_index = count;
    while out_index < elems && count < max_node {
        let mnc = MESH_NODE_CUR.load(Ordering::Relaxed);
        if mnc < max_node && state.mesh_node_st[mnc].tick != 0 {
            let ptr = MESH_NODE_ST_VAL_LEN * out_index;
            out_index = out_index + 1;
            p_data[ptr..ptr + MESH_NODE_ST_VAL_LEN].copy_from_slice(
                unsafe {
                    slice::from_raw_parts(
                        addr_of!(state.mesh_node_st[mnc].val) as *const u8,
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

pub fn mesh_send_online_status(state: &mut State)
{
    static ADV_ST_SN: AtomicU32 = AtomicU32::new(0);

    mesh_node_flush_status(state);
    let mut result_data = [0; 26];
    mesh_node_adv_status(state, &mut result_data);
    state.pkt_light_adv_status.value[..26].copy_from_slice(&result_data);

    let mut tmp_pkt: PacketAttCmd = PacketAttCmd::default();

    ADV_ST_SN.store(ADV_ST_SN.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
    unsafe {
        let val = ADV_ST_SN.load(Ordering::Relaxed);
        slice::from_raw_parts_mut(addr_of_mut!(state.pkt_light_adv_status.opcode), 3).copy_from_slice(slice::from_raw_parts(addr_of!(val) as *const u8, 3))
    }

    if MESH_NODE_ST_VAL_LEN == 4 {
        state.pkt_light_adv_status.value[25] = 0xa5;
        state.pkt_light_adv_status.value[24] = 0xa5;
    }
    state.pkt_light_adv_status.value[27] = 0xa5;
    state.pkt_light_adv_status.value[26] = 0xa5;

    unsafe {
        slice::from_raw_parts_mut(addr_of_mut!(tmp_pkt) as *mut u8, size_of::<PacketAttWrite>()).copy_from_slice(
            slice::from_raw_parts(addr_of!(state.pkt_light_adv_status) as *const u8, size_of::<PacketAttWrite>())
        )
    }

    app().mesh_manager.add_send_mesh_msg(unsafe { &*(addr_of!(tmp_pkt) as *const MeshPkt) }, 0);
}

pub fn back_to_rxmode_bridge(state: &mut State)
{
    rf_set_tx_rx_off();
    rf_set_ble_access_code(PAIR_AC.get());
    rf_set_ble_crc_adv();
    rf_set_ble_channel(SYS_CHN_LISTEN[(state.st_brige_no as usize % SYS_CHN_LISTEN.len()) >> 1]);
    rf_set_rxmode();
}

pub fn rf_link_proc_ttc(cmd_time: u32, last_ttc: u8, src: &mut MeshPkt)
{
    let mut new_ttc_base = (read_reg_system_tick() - cmd_time) / CLOCK_SYS_CLOCK_1US;
    let mut ttc_divisor = last_ttc >> 6;
    let mut new_ttc = new_ttc_base >> 10;
    if ttc_divisor != 0 {
        new_ttc = new_ttc_base >> 0xc;
        if ttc_divisor != 1 {
            new_ttc = new_ttc_base >> 0xe;
            if ttc_divisor != 2 {
                new_ttc = 1;
                if ttc_divisor == 3 {
                    new_ttc = new_ttc_base >> 0x12;
                }
            }
        }
    }

    new_ttc = new_ttc + ((last_ttc as u32) & 0x3f);
    while 0x3e < new_ttc {
        if ttc_divisor == 0 || ttc_divisor == 1 {
            new_ttc = new_ttc >> 2;
        } else {
            if ttc_divisor != 2 {
                if ttc_divisor != 3 {
                    if ttc_divisor < 3 {
                        ttc_divisor = ttc_divisor + 1;
                    }
                    src.par[9] = (ttc_divisor << 6 | 1);
                }
                new_ttc = 0x3f;
                src.par[9] = (((new_ttc as u8) & 0x3f) | (ttc_divisor << 6));
            }
            new_ttc = new_ttc >> 4;
        }
        ttc_divisor = ttc_divisor + 1;
    }
    if new_ttc == 0 {
        src.par[9] = (ttc_divisor << 6 | 1);
    } else {
        src.par[9] = (((new_ttc as u8) & 0x3f) | (ttc_divisor << 6));
    }
}

pub fn rf_link_is_notify_req(state: &mut State, value: u8) -> bool
{
    if !state.rf_slave_ota_busy {
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

static mut PKT_MESH: MeshPkt = MeshPkt {
    dma_len: 0,
    _type: 0,
    rf_len: 0,
    l2cap_len: 0,
    chan_id: 0,
    src_tx: 0,
    handle1: 0,
    sno: [0; 3],
    src_adr: 0,
    dst_adr: 0,
    op: 0,
    vendor_id: 0,
    par: [0; 10],
    internal_par1: [0; 5],
    ttl: 0,
    internal_par2: [0; 4],
    no_use: [0; 4],
};

pub fn app_bridge_cmd_handle(state: &mut State, bridge_cmd_time: u32)
{
    if state.slave_data_valid != 0 {
        state.slave_data_valid -= 1;
        if state.slave_data_valid == 0 {
        } else if state.slave_read_status_busy == 0 || state.slave_data_valid as i32 > -1 {
            state.pkt_light_data._type |= BIT!(7);

            rf_link_proc_ttc(state.app_cmd_time, 0, unsafe { &mut *(addr_of_mut!(state.pkt_light_data.value.val) as *mut MeshPkt) });

            if rf_link_is_notify_req(state, state.pkt_light_data.value.val[0] & 0x3f) {
                let mut relay_time = min(
                    (((read_reg_system_tick() - bridge_cmd_time) / CLOCK_SYS_CLOCK_1US) + 500) >> 10,
                    0xff
                );

                state.pkt_light_data.value.val[17] = relay_time as u8;
            }

            app().mesh_manager.add_send_mesh_msg(unsafe { &*(addr_of!(state.pkt_light_data) as *const MeshPkt) }, 0);
        }
    }
}

pub fn tx_packet_bridge(state: &mut State)
{
    static TICK_BRIDGE_REPORT: AtomicU32 = AtomicU32::new(0);
    static LAST_ALARM_TIME_BRIDGE: AtomicU32 = AtomicU32::new(0);
    static LAST_CONN_IBEACON_TIME: AtomicU32 = AtomicU32::new(0);

    rf_set_tx_rx_off();

    sleep_us(100);

    rf_set_ble_access_code(PAIR_AC.get());
    rf_set_ble_crc_adv();

    let tick = read_reg_system_tick();
    if state.slave_listen_interval * state.online_status_interval2listen_interval as u32 - ONLINE_STATUS_COMP * CLOCK_SYS_CLOCK_1US * 1000 < tick - TICK_BRIDGE_REPORT.load(Ordering::Relaxed) {
        TICK_BRIDGE_REPORT.store(tick, Ordering::Relaxed);
        if CLOCK_SYS_CLOCK_1US * 30000000 < tick - LAST_ALARM_TIME_BRIDGE.load(Ordering::Relaxed) {
            LAST_ALARM_TIME_BRIDGE.store(tick, Ordering::Relaxed);
            // noop
        } else {
            mesh_send_online_status(state);
        }
    }
    app_bridge_cmd_handle(state, state.t_bridge_cmd);
}

pub fn rf_link_slave_proc(state: &mut State) {
    set_mesh_info_time_handle(state);
    app().mesh_manager.mesh_pair_proc(state);
    update_connect_para(state);
}

pub fn is_add_packet_buf_ready() -> bool
{
    return (read_reg_dma_tx_wptr() - read_reg_dma_tx_rptr() & 7) < 3;
}

pub fn rf_link_add_tx_packet(state: &mut State, packet: *const PacketAttCmd, size: usize) -> bool
{
    let wptr = read_reg_dma_tx_wptr();
    let rptr = read_reg_dma_tx_rptr();
    let widx = (wptr - rptr) & 7;

    if widx < 4 {
        if widx == 0 {
            write_reg_dma_tx_fifo(addr_of!(PKT_EMPTY) as u16);
        }

        let index = state.blt_tx_wptr;
        state.blt_tx_wptr = (index + 1) % BLT_FIFO_TX_PACKET_COUNT;

        {
            let mut dest = &mut state.blt_tx_fifo[index];

            dest.fill(0);
            dest[..size].copy_from_slice(
                unsafe {
                    slice::from_raw_parts(
                        packet as *const u8,
                        size,
                    )
                }
            );
        }

        pair_enc_packet(state, unsafe { &mut *(state.blt_tx_fifo[index].as_ptr() as *mut PacketLlApp) });

        write_reg_dma_tx_fifo(state.blt_tx_fifo[index].as_ptr() as u16);
        return true;
    }
    return false;
}

pub fn rf_link_slave_read_status_par_init(state: &mut State)
{


    state.slave_status_buffer_wptr = 0;
    state.slave_status_buffer_rptr = 0;
    state.slave_stat_sno = [0; 3];
}

pub fn rf_link_slave_read_status_stop(state: &mut State)
{
    {


        state.slave_read_status_busy = 0;
        state.slave_read_status_unicast_flag = 0;
        state.slave_data_valid = 0;
    }

    rf_link_slave_read_status_par_init(state);
}

pub fn rf_ota_save_data(state: &mut State, data: &[u8]) -> OtaState
{
    let addr = state.cur_ota_flash_addr + FLASH_ADR_LIGHT_NEW_FW;
    flash_write_page(addr, data.len() as u32, data.as_ptr());

    let mut tmp = [0u8; 0x10];
    flash_read_page(addr, data.len() as u32, tmp.as_mut_ptr());

    if data == &tmp[..data.len()] {
        state.cur_ota_flash_addr += data.len() as u32;
        return OtaState::Continue;
    } else {
        return OtaState::Error;
    }
}

pub fn rf_link_match_group_mac(state: &mut State, sno: *const AppCmdValue) -> (bool, bool)
{
    let mut group_match = false;
    let mut device_match = false;

    unsafe {
        if (*sno).dst & !DEVICE_ADDR_MASK_DEFAULT != 0 {
            for addr in state.group_address {
                if addr == (*sno).dst {
                    group_match = true;
                    break;
                }
            }
            if (*sno).dst == 0xffff {
                group_match = true;
            }
        } else {
            let addr = (*sno).dst & DEVICE_ADDR_MASK_DEFAULT;
            if addr == 0 || addr == DEVICE_ADDRESS.get() {
                device_match = true;
            }
        }
    }

    (group_match, device_match)
}

pub fn mesh_construct_packet(sno: u32, dst: u16, cmd_op_para: &[u8]) -> MeshPkt
{
    let max_relay = MAX_RELAY_NUM + 1;

    assert!(cmd_op_para.len() > 2);
    assert!(cmd_op_para.len() <= 13);

    let device_address = DEVICE_ADDRESS.get();

    let mut pkt = MeshPkt {
        dma_len: 0x27,
        _type: 2,
        rf_len: 0x25,
        l2cap_len: 0xCCDD,
        chan_id: 0,
        src_tx: 0,
        handle1: 0,
        sno: [0; 3],
        src_adr: 0,
        dst_adr: 0,
        op: 0,
        vendor_id: 0,
        par: [0; 10],
        internal_par1: [0; 5],
        ttl: 0,
        internal_par2: [0; 4],
        no_use: [0; 4],
    };

    pkt.dma_len = 0x27;
    pkt.l2cap_len = 0x21;
    pkt.rf_len = 0x25;

    pkt._type = 2;
    pkt.chan_id = 0xff03;
    pkt.src_tx = device_address;
    pkt.handle1 = 0;
    pkt.op = 0;
    pkt.vendor_id = 0;
    pkt.par.fill(0);
    pkt.internal_par1.fill(0);
    pkt.ttl = 0;
    pkt.internal_par2[0] = 0;
    pkt.sno.copy_from_slice(unsafe {
        slice::from_raw_parts(addr_of!(sno) as *const u8, 3)
    });
    pkt.src_adr = device_address;
    pkt.dst_adr = dst;
    unsafe {
        slice::from_raw_parts_mut(addr_of_mut!(pkt.op), cmd_op_para.len()).copy_from_slice(cmd_op_para)
    }

    pkt.internal_par1[CURRENT_RELAY_COUNT] = max_relay;

    pkt
}

pub fn mesh_report_status_enable(state: &mut State, enable: bool)
{

    if enable {
        if state.mesh_node_max >> 5 != 0 {
            state.mesh_node_mask.iter_mut().for_each(|v| { *v = 0xfffffffe });
        }

        if state.mesh_node_max & 0x1f != 0 {
            state.mesh_node_mask[state.mesh_node_max as usize >> 5] = (1 << (state.mesh_node_max & 0x1f)) - 1;
        }
    }

    state.mesh_node_report_enable = enable;
}

pub fn mesh_report_status_enable_mask(state: &mut State, data: *const u8, len: u16)
{
    state.mesh_node_report_enable = if unsafe { *data } != 0 { true } else { false };
    if state.mesh_node_report_enable && len > 1 {
        for index in 1..len {
            if state.mesh_node_max != 0 {
                state.mesh_node_st.iter_mut().enumerate().for_each(|(i, v)| {
                    if unsafe { *data.offset(index as isize) } == v.val.dev_adr {
                        state.mesh_node_mask[i >> 5] |= 1 << (i & 0x1f);
                    }
                });
            }
        }
    }
}

pub fn rf_link_delete_pair(state: &mut State)
{
    let mut key = [0u8; 32];

    key[0..0x10].copy_from_slice(&state.pair_config_mesh_name);
    key[0x10..0x20].copy_from_slice(&state.pair_config_mesh_pwd);

    pair_set_key(state, key.as_ptr());
    pair_save_key(state);

    if state.not_need_login == false {
        state.pair_login_ok = false;
    }
}

pub fn rf_link_get_op_para(packet: *const PacketL2capData, p_op: &mut [u8], p_op_len: &mut u8, p_para: &mut [u8], p_para_len: &mut u8, mesh_flag: bool) -> bool
{
    unsafe {
        if (((*packet).value[7] as u32 * 0x1000000) as i32) < 0 {
            if (*packet).value[7] >> 6 == 3 {
                *p_op_len = 3;
            } else {
                *p_op_len = 2;
            }
        } else {
            *p_op_len = 1;
        }
        p_op[0..*p_op_len as usize].copy_from_slice(&(*packet).value[7..7 + *p_op_len as usize]);
        let mut pkt_len = (*packet).l2cap_len - 10;
        let pkt_len_delta;
        let max_param_len;
        if p_op[0] & 0x3f == 6 {
            max_param_len = 0xf;
            pkt_len_delta = 5;
        } else {
            max_param_len = 10;
            pkt_len_delta = 0;
        }

        if mesh_flag {
            pkt_len = pkt_len_delta + pkt_len - *p_op_len as u16 - 10;
        } else {
            pkt_len = pkt_len - *p_op_len as u16;
        }

        *p_para_len = pkt_len as u8;

        if pkt_len <= max_param_len {
            p_para[0..pkt_len as usize].copy_from_slice(&(*packet).value[7 + *p_op_len as usize..7 + *p_op_len as usize + pkt_len as usize]);
        } else {
            *p_para_len = 0;
        }

        return pkt_len <= max_param_len;
    }
}


