use core::cmp::min;
use core::mem::{size_of, transmute};
use core::ptr::{addr_of, addr_of_mut, null, null_mut};
use core::slice;
use core::sync::atomic::{AtomicU32, AtomicU8, AtomicUsize, Ordering};

use crate::{app, BIT, pub_mut, uprintln};
use crate::common::{dev_addr_with_mac_flag, rf_update_conn_para, set_conn_update_cnt, set_conn_update_successed, SYS_CHN_LISTEN, update_ble_parameter_cb};
use crate::config::FLASH_ADR_LIGHT_NEW_FW;
use crate::main_light::{rf_link_data_callback, rf_link_response_callback};
use crate::mesh::{get_mesh_node_mask, get_mesh_node_st, get_mesh_pair_enable, MESH_NODE_ST_VAL_LEN, mesh_node_st_val_t};
use crate::sdk::ble_app::ble_ll_att::ble_ll_channel_table_calc;
use crate::sdk::ble_app::ble_ll_attribute::{get_att_service_discover_tick, get_slave_link_time_out, l2cap_att_handler, set_att_service_discover_tick, set_slave_link_time_out};
use crate::sdk::ble_app::ble_ll_pair::{pair_dec_packet_mesh, pair_enc_packet, pair_enc_packet_mesh, pair_init, pair_save_key, pair_set_key};
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::ble_app::shared_mem::get_blt_tx_fifo;
use crate::sdk::common::compat::array4_to_int;
use crate::sdk::drivers::flash::{flash_read_page, flash_write_page};
use crate::sdk::light::{*};
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, clock_time_exceed, sleep_us};
use crate::sdk::mcu::register::{*};
use crate::uart_manager::{get_pkt_user_cmd, get_pkt_user_cmd_addr, light_mesh_rx_cb};

pub_mut!(slave_timing_update, u32, 0);
pub_mut!(slave_instant_next, u16, 0);
pub_mut!(slave_chn_map, [u8; 5], [0; 5]);
pub_mut!(slave_interval_old, u32, 0);
pub_mut!(slave_link_interval, u32, 0x9c400);
pub_mut!(slave_window_size_update, u32, 0);
pub_mut!(ble_conn_timeout, u32, 0);
pub_mut!(ble_conn_interval, u32, 0);
pub_mut!(ble_conn_offset, u32, 0);
pub_mut!(add_tx_packet_rsp_failed, u32, 0);
pub_mut!(T_scan_rsp_intvl, u32, 0x92);
pub_mut!(p_slave_status_buffer, *mut PacketAttData, null_mut());
pub_mut!(slave_status_buffer_num, u8, 0);
pub_mut!(g_vendor_id, u16, 0x211);
pub_mut!(light_rcv_rssi, u8, 0);
pub_mut!(rcv_pkt_time, u32, 0);
pub_mut!(light_conn_sn_master, u16, 0);
pub_mut!(slave_window_size, u32, 0);
pub_mut!(slave_timing_update2_flag, u32, 0);
pub_mut!(slave_next_connect_tick, u32, 0);
pub_mut!(slave_timing_update2_ok_time, u32, 0);

#[derive(PartialEq)]
pub enum IrqHandlerStatus {
    None,
    Adv,
    Bridge,
    Rx,
    Listen
}
pub_mut!(p_st_handler, IrqHandlerStatus, IrqHandlerStatus::None);

pub_mut!(pkt_empty, [u8; 6], [02, 00, 00, 00, 01, 00]);

pub_mut!(pkt_light_report, PacketAttCmd, PacketAttCmd {
    dma_len: 0x1D,
    _type: 2,
    rf_len: 0x1B,
    l2cap_len: 0x17,
    chan_id: 4,
    opcode: 0x1B,
    handle: 0x12,
    handle1: 0,
    value: PacketAttValue {
        sno: [0; 3],
        src: [0; 2],
        dst: [0; 2],
        val: [0xdc, 0x11, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    }
});

fn mesh_node_update_status(pkt: &[mesh_node_st_val_t]) -> u32
{
    let mut src_index = 0;
    let mut result = 0xfffffffe;
    let tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;
    while src_index < pkt.len() && pkt[src_index].dev_adr != 0 {
        if *get_device_address() as u8 != pkt[src_index].dev_adr {
            let mesh_node_max = *get_mesh_node_max();
            let mut current_index = 1;
            let mut mesh_node_st = &mut (*get_mesh_node_st())[current_index];
            if mesh_node_max >= 2 {
                if mesh_node_st.val.dev_adr != pkt[src_index].dev_adr {
                    for tidx in 1..(*get_mesh_node_st()).len() {
                        current_index = tidx;
                        mesh_node_st = &mut (*get_mesh_node_st())[current_index];

                        if mesh_node_max <= tidx as u8 || pkt[src_index].dev_adr == mesh_node_st.val.dev_adr {
                            break;
                        }
                    }
                }
            }

            if MESH_NODE_MAX_NUM as usize == current_index {
                return 1;
            }

            if mesh_node_max as usize == current_index {
                set_mesh_node_max(*get_mesh_node_max() + 1);

                mesh_node_st.val = pkt[src_index];
                mesh_node_st.tick = tick;

                (*get_mesh_node_mask())[mesh_node_max as usize >> 5] |= 1 << (mesh_node_max & 0x1f);

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
                        (*get_mesh_node_mask())[current_index >> 5] |= 1 << (current_index & 0x1f);
                    }

                    mesh_node_st.tick = tick;
                }
            }
        }

        src_index += 1;
    }
    return 1;
}

fn is_exist_in_rc_pkt_buf(opcode: u8, cmd_pkt: &AppCmdValue) -> bool
{
    get_rc_pkt_buf().iter().any(|v| v.op == opcode && v.sno == cmd_pkt.sno)
}

fn rf_link_is_notify_rsp(opcode: u8) -> bool
{
    [
        LGT_CMD_LIGHT_GRP_RSP1,
        LGT_CMD_LIGHT_GRP_RSP2,
        LGT_CMD_LIGHT_GRP_RSP3,
        LGT_CMD_LIGHT_STATUS,
        LGT_CMD_DEV_ADDR_RSP,
        LGT_CMD_USER_NOTIFY_RSP
    ].contains(&opcode)
}

fn rc_pkt_buf_push(opcode: u8, cmd_pkt: &AppCmdValue)
{
    if rf_link_is_notify_rsp(opcode) || is_exist_in_rc_pkt_buf(opcode, cmd_pkt) {
        return;
    }

    if get_rc_pkt_buf().is_full() {
        get_rc_pkt_buf().pop_back();
    }

    get_rc_pkt_buf().push_front(
        PktBuf {
            op: opcode,
            sno: cmd_pkt.sno,
            notify_ok_flag: false,
            sno2: cmd_pkt.par[8..10].try_into().unwrap(),
        }
    ).unwrap();
}

fn req_cmd_is_notify_ok(opcode: u8, cmd_pkt: &AppCmdValue) -> bool
{
    get_rc_pkt_buf().iter().any(|pkt| {
        pkt.op == opcode && pkt.sno == cmd_pkt.sno && pkt.notify_ok_flag
    })
}

fn req_cmd_set_notify_ok_flag(opcode: u8, cmd_pkt: &AppCmdValue)
{
    get_rc_pkt_buf().iter_mut().filter(
        |v| { v.op == opcode && v.sno == cmd_pkt.sno }
    ).for_each(
        |v| { v.notify_ok_flag = true }
    );
}

pub fn copy_par_user_all(params_len: u32, ptr: *const u8)
{
    (*get_pkt_light_status()).value.val[3..3 + params_len as usize].copy_from_slice(
        unsafe {
            slice::from_raw_parts(ptr, params_len as usize)
        }
    );
}

fn rf_link_slave_notify_req_mask(adr: u8)
{
    if *get_slave_read_status_busy() != 0 && (*get_device_address() as u8 != adr || *get_slave_read_status_busy() == 0x21) {
        if *get_slave_read_status_unicast_flag() == 0 {
            if (*get_pkt_light_data()).value.val[8..0xd].iter().any(|v| *v == adr) {
                return;
            }

            (*get_pkt_light_data()).value.val[(*get_notify_req_mask_idx() + 8) as usize] = adr;
            set_notify_req_mask_idx((*get_notify_req_mask_idx() + 1) % 5);
        } else {
            set_slave_data_valid(0);
        }
    }
}

fn rf_link_slave_add_status_ll(packet: &MeshPkt) -> bool
{
    let mut result = false;

    if app().mesh_manager.mesh_pair_notify_refresh(unsafe { &*(addr_of!(*packet) as *const PacketAttCmd) }) != 2 {
        if *get_slave_status_record_idx() != 0 {
            for st_rec in *get_slave_status_record() {
                if packet.src_adr as u8 == st_rec.adr[0] {
                    rf_link_slave_notify_req_mask(packet.src_adr as u8);
                    // mesh_ota_master_read_rsp_handle(src, true);
                    return false;
                }
            }
        }

        result = false;
        if (*get_slave_status_buffer_wptr() + 1) % *get_slave_status_buffer_num() != *get_slave_status_buffer_rptr() && *get_slave_status_record_idx() < MESH_NODE_MAX_NUM {
            (*get_slave_status_record())[*get_slave_status_record_idx() as usize].adr[0] = packet.src_adr as u8;
            set_slave_status_record_idx(*get_slave_status_record_idx() + 1);
            rf_link_slave_notify_req_mask(packet.src_adr as u8);
            let st_ptr = unsafe { &mut *(*get_p_slave_status_buffer()).offset((*get_slave_status_buffer_wptr() % *get_slave_status_buffer_num()) as isize) };
            set_slave_status_buffer_wptr((*get_slave_status_buffer_wptr() + 1) % *get_slave_status_buffer_num());
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
    return result;
}

pub fn rf_link_slave_add_status(packet: &MeshPkt)
{
    critical_section::with(|_| {
        rf_link_slave_add_status_ll(packet);
    });
}

pub fn rf_link_rc_data(mut packet: MeshPkt) -> bool {
    if *get_slave_link_connected() {
        if 0x3fffffffi32 < (read_reg_system_tick_irq() as i32 - read_reg_system_tick() as i32) - (CLOCK_SYS_CLOCK_1US * 1000) as i32 {
            return false;
        }
    }

    if packet.rf_len != 0x25 || packet.l2cap_len != 0x21 || packet._type & 3 != 2 || packet.chan_id == 0xeeff || unsafe { !pair_dec_packet_mesh(&packet) } {
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

        mesh_node_update_status(unsafe { slice::from_raw_parts(addr_of!(packet.sno) as *const mesh_node_st_val_t, 0x1a / MESH_NODE_ST_VAL_LEN as usize) });
        return false;
    }

    // Don't do anything if this packet has been relayed too many times
    if *get_max_relay_num() + 6 < packet.internal_par1[CURRENT_RELAY_COUNT] {
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
    let no_match_slave_sno = cmd_pkt.sno != unsafe { slice::from_raw_parts(get_slave_sno_addr() as *const u8, 3) };
    let match_slave_sno_sending = cmd_pkt.sno == unsafe { slice::from_raw_parts(get_slave_sno_sending_addr() as *const u8, 3) };
    let pkt_exists_in_buf = is_exist_in_rc_pkt_buf(op, cmd_pkt);
    let pkt_valid = (no_match_slave_sno || *get_slave_link_cmd() != op) && !pkt_exists_in_buf;

    // If the slave link is connected (Android) and the dest address is us, then we should forward
    // the packet on to the slave link
    let mut should_notify = packet.dst_adr == *get_device_address() && *get_slave_link_connected();

    // See if we should call the message callback
    if pkt_valid || (rf_link_is_notify_rsp(op) && should_notify)
    {
        light_mesh_rx_cb(cmd_pkt);
    }

    let src_device_addr_match = packet.src_adr == *get_device_address();

    // If we should notify, and the opcode is a response opcode and the slave (Android) is waiting
    // for a response, then send this response to the slave
    if rf_link_is_notify_rsp(op) && should_notify {
        if *get_slave_read_status_busy() != op || cmd_pkt.sno != *get_slave_stat_sno() {
            return false;
        }
        rf_link_slave_add_status(&packet);
        return false;
    }

    if src_device_addr_match {
        if op != LGT_CMD_CONFIG_DEV_ADDR || !dev_addr_with_mac_flag(params.as_ptr()) || match_slave_sno_sending {
            return false;
        }
    }

    set_rcv_pkt_ttc(packet.par[9]);

    let (group_match, device_match) = rf_link_match_group_mac(cmd_pkt);

    // Record the packet so we don't handle it again if we receive it again
    if group_match != false || device_match != false {
        if pkt_valid {
            rc_pkt_buf_push(op, cmd_pkt);
            rf_link_data_callback(l2cap_data);
        }
    }

    if rf_link_is_notify_req(op) {
        set_slave_read_status_response(true);
        if group_match == false {
            set_slave_read_status_response(device_match);
        }
        if req_cmd_is_notify_ok(op, cmd_pkt) {
            set_slave_read_status_response(false);
        } else if packet.dst_adr & !DEVICE_ADDR_MASK_DEFAULT != 0 {
            for i in 0..5 {
                if packet.par[i + 4] == *get_device_address() as u8 {
                    req_cmd_set_notify_ok_flag(op, cmd_pkt);
                    set_slave_read_status_response(false);
                    break;
                }
            }
        }
    }

    let relay_count = packet.internal_par1[CURRENT_RELAY_COUNT];

    packet.dma_len = (packet.l2cap_len as u32 + 6) & 0xffffff;
    packet.rf_len = packet.l2cap_len as u8 + 4;

    let mut ttl = relay_count;

    if no_match_slave_sno || *get_slave_link_cmd() != op {
        packet.src_tx = *get_device_address();
        (*get_slave_sno())[0..3].copy_from_slice(&cmd_pkt.sno);

        set_slave_link_cmd(op);
    } else {
        ttl = *get_org_ttl();
    }

    set_org_ttl(ttl);

    // todo: Use rand register?
    let rand_1 = ((read_reg_system_tick() ^ array4_to_int(get_mac_id())) & 0xf) * 500;
    if rf_link_is_notify_req(op) && *get_slave_read_status_response() {
        (*get_pkt_light_status()).value.sno = cmd_pkt.sno;
        packet.src_tx = *get_device_address();
        if op == LGT_CMD_LIGHT_READ_STATUS {
            (*get_pkt_light_status()).value.val[15] = GET_STATUS;
            if pkt_valid {
                (*get_pkt_light_status()).value.val[13] = packet.par[9];
                (*get_pkt_light_status()).value.val[18] = packet.internal_par1[CURRENT_RELAY_COUNT];
                if *get_max_relay_num() < packet.internal_par1[CURRENT_RELAY_COUNT] {
                    (*get_pkt_light_status()).value.val[14] = 0;
                } else {
                    (*get_pkt_light_status()).value.val[14] = *get_max_relay_num() - packet.internal_par1[CURRENT_RELAY_COUNT];
                }
            }
        } else if op == LGT_CMD_LIGHT_GRP_REQ {
            (*get_pkt_light_status()).value.val[15] = packet.internal_par1[1];
        } else if op == LGT_CMD_LIGHT_CONFIG_GRP {
            (*get_pkt_light_status()).value.val[15] = GET_GROUP1;
        } else if op == LGT_CMD_CONFIG_DEV_ADDR {
            (*get_pkt_light_status()).value.val[15] = GET_DEV_ADDR;
        } else if op == LGT_CMD_USER_NOTIFY_REQ {
            (*get_pkt_light_status()).value.val[15] = GET_USER_NOTIFY;
        }
        unsafe { copy_par_user_all(params_len as u32, (addr_of!(packet.vendor_id) as u32 + 1) as *const u8); }
        if (no_match_slave_sno || *get_slave_link_cmd() != op) || params[1] != 0 {
            (*get_pkt_light_status()).value.src.copy_from_slice(unsafe { slice::from_raw_parts(addr_of!(packet.src_adr) as *const u8, 2) });

            let mut request_params: PacketAttValue = PacketAttValue {
                sno: [0; 3],
                src: [0; 2],
                dst: [0; 2],
                val: [0; 23],
            };

            unsafe {
                slice::from_raw_parts_mut(addr_of_mut!(request_params) as *mut u8, size_of::<PacketAttValue>()).copy_from_slice(
                    slice::from_raw_parts(
                        addr_of!(*cmd_pkt) as *const u8,
                        size_of::<PacketAttValue>(),
                    )
                )
            }

            if rf_link_response_callback(&mut get_pkt_light_status().value, &request_params) {
                (*get_pkt_light_status())._type |= BIT!(7);

                rf_send_stx_mesh(get_pkt_light_status());
            }
        }
    }

    if relay_count != 0 && *get_org_ttl() == relay_count && !device_match
    {
        if *get_slave_read_status_busy() == 0 || !rf_link_is_notify_rsp(op) {
            packet.internal_par1[CURRENT_RELAY_COUNT] = relay_count - 1;
        }

        // Relay the message if it's not for us
        let mut delay = 100;
        if !*get_slave_link_connected() {
            delay = 8000 - rand_1;
        }
        sleep_us(delay);

        packet._type |= BIT!(7);

        if op != LGT_CMD_LIGHT_STATUS {
            rf_link_proc_ttc(*get_rcv_pkt_time(), *get_rcv_pkt_ttc(), &mut packet);
        }

        if rf_link_is_notify_req(op) {
            let relay_time = min(
                ((((read_reg_system_tick() - *get_rcv_pkt_time()) / CLOCK_SYS_CLOCK_1US) + 500) >> 10) + packet.internal_par1[LAST_RELAY_TIME] as u32,
                0xff
            );

            packet.internal_par1[LAST_RELAY_TIME] = relay_time as u8;
        }

        rf_send_stx_mesh((&packet).into());
    }

    rf_set_rxmode();

    return true;
}

pub fn rf_send_stx_mesh(pkt: &PacketAttCmd) {
    rf_set_tx_rx_off();
    sleep_us(100);

    rf_set_ble_access_code(*get_pair_ac());
    rf_set_ble_crc_adv();

    // Pick a random start channel
    let start_chan_idx = (read_reg_system_tick() as u16 ^ read_reg_rnd_number()) & 3;

    // Send the packet on each channel
    for channel_index in start_chan_idx..start_chan_idx + 4 {
        rf_set_ble_channel(SYS_CHN_LISTEN[channel_index as usize & 3]);
        rf_start_stx_mesh(pkt, CLOCK_SYS_CLOCK_1US * 30 + read_reg_system_tick());
        sleep_us(700);
    }
}

pub fn rf_link_slave_data(packet: &PacketLlData, time: u32) -> bool {
    let rf_len: u8 = packet.rf_len;
    let chanid: u16 = packet.chanid;

    if (packet._type as i32 * 0x1000000) as i32 >= -1 {
        if (packet._type & 3) == 2 {
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
            if packet._type & 3 == 3 && packet.l2cap_low == 1 {
                set_slave_timing_update(1);
                set_slave_instant_next(((packet.sno as u16) << 8) | packet.hh as u16);
                (*get_slave_chn_map()).iter_mut().enumerate().for_each(|(i, v)| {
                    *v = unsafe { *(addr_of!(packet.l2cap_high) as *const u8).offset(i as isize) };
                });

                return true;
            }

            if rf_len == 0xc && packet._type & 3 == 3 && packet.l2cap_low == 0 {
                set_slave_interval_old(*get_slave_link_interval());
                set_slave_instant_next(packet.group);
                set_slave_window_size_update((packet.l2cap_high as u32 * 1250 + 1300) * CLOCK_SYS_CLOCK_1US);
                set_slave_timing_update(2);
                set_ble_conn_interval(CLOCK_SYS_CLOCK_1US * 1250 * unsafe { (*addr_of!(packet.att) as *const u16) } as u32);
                set_ble_conn_offset(packet.chanid as u32 * CLOCK_SYS_CLOCK_1US * 1250);
                set_ble_conn_timeout(packet.nid as u32 * 10000);
                return false;
            }
        }
        let (res_pkt, len) = unsafe { l2cap_att_handler(addr_of!(*packet) as *const PacketL2capData) };
        if res_pkt != null() && !rf_link_add_tx_packet(unsafe { res_pkt as *const PacketAttCmd }, len) {
            set_add_tx_packet_rsp_failed(*get_add_tx_packet_rsp_failed() + 1);
        }
        return false;
    }

    return false;
}

pub fn rf_link_timing_adjust(time: u32)
{
    if *get_slave_timing_adjust_enable() {
        set_slave_timing_adjust_enable(false);
        if time - *get_slave_tick_brx() < CLOCK_SYS_CLOCK_1US * 700 {
            set_slave_next_connect_tick(*get_slave_next_connect_tick() - CLOCK_SYS_CLOCK_1US * 200);
        } else if CLOCK_SYS_CLOCK_1US * 1100 < time - *get_slave_tick_brx() {
            set_slave_next_connect_tick(CLOCK_SYS_CLOCK_1US * 200 + *get_slave_next_connect_tick());
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

pub fn rf_link_slave_connect(packet: &PacketLlInit, time: u32) -> bool
{
    set_conn_update_successed(false);
    set_conn_update_cnt(0);
    if *get_slave_connection_enable() || packet.scan_a == packet.adv_a {
        if check_par_con(packet) == false {
            rf_stop_trx();

            set_slave_window_offset(CLOCK_SYS_CLOCK_1US * 1250 * (packet.woffset as u32 + 1));

            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 1000 + read_reg_system_tick());
            write_reg_irq_src(0x100000);
            write_reg_system_tick_irq(time + *get_slave_window_offset() + CLOCK_SYS_CLOCK_1US * (0 - if packet.woffset == 0 { 500 } else { 700 }));
            if 0x80000000 < read_reg_system_tick_irq() - read_reg_system_tick() {
                write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick());
            }

            set_light_conn_sn_master(0x80);
            set_slave_connected_tick(read_reg_system_tick());
            if *get_security_enable() != false {
                set_slave_first_connected_tick(*get_slave_connected_tick());
            }

            set_slave_status_tick(((packet.interval as u32 * 5) >> 2) as u8);
            set_slave_link_interval(packet.interval as u32 * CLOCK_SYS_CLOCK_1US * 1250);
            set_slave_window_size((packet.wsize as u32 * 1250 + 1100) * CLOCK_SYS_CLOCK_1US);

            let tmp = *get_slave_link_interval() - CLOCK_SYS_CLOCK_1US * 1250;
            if tmp <= *get_slave_window_size() && *get_slave_window_size() - tmp != 0 {
                set_slave_window_size(tmp);
            }

            set_slave_link_time_out(packet.timeout as u32 * 10000);
            unsafe {
                slice::from_raw_parts_mut(
                    addr_of_mut!((*get_pkt_init()).scan_a) as *mut u8,
                    0x22,
                ).copy_from_slice(
                    slice::from_raw_parts(
                        addr_of!(packet.scan_a) as *const u8,
                        0x22,
                    )
                )
            };

            ble_ll_channel_table_calc((*get_pkt_init()).chm.as_ptr(), true);

            // rf_set_ble_crc(&(*get_pkt_init()).crcinit);
            write_reg_rf_crc((((*get_pkt_init()).crcinit[1] as u32) << 8) | (((*get_pkt_init()).crcinit[2] as u32) << 0x10) | (*get_pkt_init()).crcinit[0] as u32);
            rf_reset_sn();

            set_slave_instant(0);
            set_slave_timing_update2_flag(0);
            set_slave_interval_old(0);
            set_slave_timing_update2_ok_time(0);
            set_slave_window_size_update(0);

            pair_init();

            set_mesh_node_report_enable(false);

            (*get_mesh_node_mask()).fill(0);

            set_p_st_handler(IrqHandlerStatus::Rx);
            set_need_update_connect_para(true);
            set_att_service_discover_tick(read_reg_system_tick() | 1);

            write_reg8(0x00800f04, 0x67);  // tx wail & settle time

            return true;
        }
    }
    return false;
}

pub fn light_check_tick_per_us(ticks: u32)
{
    if ticks == 0x10 {
        set_T_scan_rsp_intvl(0);
    } else if ticks == 0x20 || ticks != 0x30 {
        set_T_scan_rsp_intvl(0x92);
    } else {
        set_T_scan_rsp_intvl(0x93);
    }
}

pub fn rf_link_slave_pairing_enable(enable: bool)
{
    set_slave_connection_enable(enable);
    set_slave_adv_enable(enable);
}

pub fn rf_link_slave_set_buffer(addr: &mut [PacketAttData])
{
    set_p_slave_status_buffer((*addr).as_mut_ptr());
    set_slave_status_buffer_num(addr.len() as u8);
}

pub fn vendor_id_init(vendor_id: u16)
{
    (*get_pkt_light_report()).value.val[1] = vendor_id as u8;
    (*get_pkt_light_report()).value.val[2] = (vendor_id >> 8) as u8;

    set_g_vendor_id(vendor_id);
}

// param st is 2bytes = lumen% + rsv(0xFF)  // rf pkt : device_address+sn+lumen+rsv);
pub fn ll_device_status_update(val_par: &[u8])
{
    (*get_mesh_node_st())[0].val.par.copy_from_slice(val_par);
    (*get_mesh_node_st())[0].tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;

    (*get_mesh_node_mask())[0] |= 1;
}

pub fn is_receive_ota_window() -> bool
{
    static TICK_LOOP: AtomicU32 = AtomicU32::new(0);

    if LOOP_INTERVAL_US != 0 {
        if read_reg_system_tick() - TICK_LOOP.load(Ordering::Relaxed) <= LOOP_INTERVAL_US as u32 * CLOCK_SYS_CLOCK_1US {
            return true;
        }
        TICK_LOOP.store(read_reg_system_tick(), Ordering::Relaxed);
    }
    if *get_slave_link_state() == 1 && CLOCK_SYS_CLOCK_1US == 0x10 {
        return true;
    }

    let mut result = false;
    if *get_rf_slave_ota_busy() != false {
        if *get_slave_link_state() == 7 {
            result = 6 < *get_slave_link_state();
        }
    }

    return result;
}

pub fn setup_ble_parameter_start(delay: u16, mut interval_min: u16, mut interval_max: u16, timeout: u16) -> u32
{
    let mut invalid = false;

    if interval_max | interval_min == 0 {
        set_update_interval_user_max(*get_slave_link_interval() as u16);
        set_update_interval_user_min(*get_slave_link_interval() as u16);
        interval_max = *get_update_interval_user_max();
        interval_min = *get_update_interval_user_min();
    } else {
        if interval_min < INTERVAL_THRESHOLD {
            invalid = true;
        }

        set_update_interval_user_min(interval_min);
        set_update_interval_user_max(interval_max);
    }

    if timeout == 0 {
        set_update_timeout_user(*get_slave_link_time_out() as u16);
    } else {
        set_update_timeout_user(timeout);
        if timeout < 100 {
            set_update_interval_user_max(0);
            set_update_interval_user_min(0);
            set_update_timeout_user(0);
            return 0xfffffffd;
        }
    }
    if invalid == false {
        set_update_interval_flag(delay);
        set_update_interval_time(true);
        return 0;
    }
    set_update_timeout_user(0);
    set_update_interval_user_min(0);
    set_update_interval_user_max(0);
    return 0xfffffffe;
}

fn update_connect_para()
{
    if *get_need_update_connect_para() {
        if *get_att_service_discover_tick() != 0 {
            if UPDATE_CONNECT_PARA_DELAY_MS * CLOCK_SYS_CLOCK_1US * 1000 < read_reg_system_tick() - *get_att_service_discover_tick() {
                update_ble_parameter_cb();
                set_need_update_connect_para(false);
                set_att_service_discover_tick(0);
            }
        }
    }
}

fn set_mesh_info_time_handle()
{
    if *get_set_mesh_info_time() != 0 && !*get_set_mesh_info_expired_flag() && *get_set_mesh_info_time() * CLOCK_SYS_CLOCK_1US * 1000000 < read_reg_system_tick() {
        set_set_mesh_info_expired_flag(true);
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

    // Iterate over each mesh node and check if it's timed out
    for count in 1..*get_mesh_node_max() as usize {
        let p_node_st = &mut (*get_mesh_node_st())[count];
        if p_node_st.tick != 0 && (CLOCK_SYS_CLOCK_1US * ONLINE_STATUS_TIMEOUT * 1000) >> 0x10 < (tick >> 0x10 | 1) - p_node_st.tick as u32 {
            p_node_st.tick = 0;

            // Set the bit in the mask so that the status is reported (Since the device has changed to offline now)
            (*get_mesh_node_mask())[count >> 5] |= 1 << (count & 0x1f);
        }
    }
}

fn mesh_node_keep_alive()
{
    set_device_node_sn(*get_device_node_sn() + 1);
    if *get_device_node_sn() == 0 {
        set_device_node_sn(1);
    }

    (*get_mesh_node_st())[0].val.sn = *get_device_node_sn();
    (*get_mesh_node_st())[0].tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;
}

fn mesh_node_adv_status(p_data: &mut [u8]) -> u32
{
    static SEND_ONLINE_STATUS_CNT: AtomicU8 = AtomicU8::new(0xC8);
    static MESH_NODE_CUR: AtomicUsize = AtomicUsize::new(1);

    // Clear the result data
    p_data.fill(0);

    // Get the max number of elements the result data can hold
    let mut elems = p_data.len() / MESH_NODE_ST_VAL_LEN as usize;
    if (*get_mesh_node_max() as usize) < p_data.len() / MESH_NODE_ST_VAL_LEN as usize {
        elems = *get_mesh_node_max() as usize;
    }

    // Copy our status in to the result data first
    p_data[0..MESH_NODE_ST_VAL_LEN as usize].copy_from_slice(
        unsafe {
            slice::from_raw_parts(
                addr_of!((*get_mesh_node_st())[0].val) as *const u8,
                MESH_NODE_ST_VAL_LEN as usize,
            )
        }
    );

    // Update our own record to keep our status record in sync
    mesh_node_keep_alive();

    let max_node = *get_mesh_node_max() as usize;
    let mut count = 1;

    let mut out_index = count;
    while out_index < elems && count < max_node {
        let mnc = MESH_NODE_CUR.load(Ordering::Relaxed);
        if mnc < max_node && (*get_mesh_node_st())[mnc].tick != 0 {
            let ptr = MESH_NODE_ST_VAL_LEN as usize * out_index;
            out_index = out_index + 1;
            p_data[ptr..ptr + MESH_NODE_ST_VAL_LEN as usize].copy_from_slice(
                unsafe {
                    slice::from_raw_parts(
                        addr_of!((*get_mesh_node_st())[mnc].val) as *const u8,
                        MESH_NODE_ST_VAL_LEN as usize,
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

pub fn mesh_send_command(packet: *const PacketAttCmd, retransmit_count: u32)
{
    rf_set_tx_rx_off();
    rf_set_ble_access_code(*get_pair_ac());
    rf_set_ble_crc_adv();

    if retransmit_count != 0xffffffff {
        for _ in 0..retransmit_count+1 {
            for chn in SYS_CHN_LISTEN {
                rf_set_ble_channel(chn);
                rf_start_srx2tx(packet as u32, read_reg_system_tick() + CLOCK_SYS_CLOCK_1US * 30);
                sleep_us(600);
            }
        }
    }
}


pub fn mesh_send_online_status()
{
    static ADV_ST_SN: AtomicU32 = AtomicU32::new(0);

    mesh_node_flush_status();

    mesh_node_adv_status(&mut (*get_pkt_light_adv_status()).value[..26]);

    ADV_ST_SN.store(ADV_ST_SN.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
    unsafe {
        let val = ADV_ST_SN.load(Ordering::Relaxed);
        slice::from_raw_parts_mut((addr_of_mut!((*get_pkt_light_adv_status()).opcode) as *mut u8), 3).copy_from_slice(slice::from_raw_parts(addr_of!(val) as *const u8, 3))
    }

    if MESH_NODE_ST_VAL_LEN == 4 {
        (*get_pkt_light_adv_status()).value[25] = 0xa5;
        (*get_pkt_light_adv_status()).value[24] = 0xa5;
    }
    (*get_pkt_light_adv_status()).value[27] = 0xa5;
    (*get_pkt_light_adv_status()).value[26] = 0xa5;

    let mut tmp_pkt: PacketAttCmd = PacketAttCmd::default();

    unsafe {
        slice::from_raw_parts_mut(addr_of_mut!(tmp_pkt) as *mut u8, size_of::<PacketAttWrite>()).copy_from_slice(
            slice::from_raw_parts(get_pkt_light_adv_status_addr() as *const u8, size_of::<PacketAttWrite>())
        )
    }

    if *get_security_enable() {
        tmp_pkt._type |= BIT!(7);
        pair_enc_packet_mesh(addr_of_mut!(tmp_pkt) as *mut MeshPkt);
    }

    mesh_send_command(&tmp_pkt, 0);
}

pub fn back_to_rxmode_bridge()
{
    rf_set_tx_rx_off();
    rf_set_ble_access_code(*get_pair_ac());
    rf_set_ble_crc_adv();
    rf_set_ble_channel(SYS_CHN_LISTEN[(*get_st_brige_no() as usize % SYS_CHN_LISTEN.len()) >> 1]);
    rf_set_rxmode();
}

fn rf_link_proc_ttc(cmd_time: u32, last_ttc: u8, src: &mut MeshPkt)
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

pub fn rf_link_is_notify_req(value: u8) -> bool
{
    if !*get_rf_slave_ota_busy() {
        return [
            LGT_CMD_LIGHT_READ_STATUS,
            LGT_CMD_LIGHT_GRP_REQ,
            LGT_CMD_CONFIG_DEV_ADDR,
            LGT_CMD_LIGHT_CONFIG_GRP,
            LGT_CMD_USER_NOTIFY_REQ
        ].contains(&value);
    }

    return false;
}

pub fn rf_start_stx_mesh(packet: &PacketAttCmd, sched_tick: u32)
{
    get_pkt_mesh().clone_from(unsafe { transmute(packet) });

    (*get_pkt_mesh()).src_tx = *get_device_address();
    (*get_pkt_mesh()).handle1 = 0;

    if packet._type & BIT!(7) != 0 {
        pair_enc_packet_mesh(get_pkt_mesh());
    }

    rf_start_srx2tx(get_pkt_mesh_addr() as u32, sched_tick);
}

pub fn app_bridge_cmd_handle(bridge_cmd_time: u32)
{
    if *get_slave_data_valid() != 0 {
        set_slave_data_valid(*get_slave_data_valid() - 1);
        if *get_slave_data_valid() == 0 {
            set_slave_sno_sending([0, 0, 0]);
        } else if *get_slave_read_status_busy() == 0 || *get_slave_data_valid() as i32 > -1 {
            (*get_pkt_light_data())._type |= BIT!(7);

            rf_link_proc_ttc(*get_app_cmd_time(), 0, unsafe { &mut *(addr_of_mut!((*get_pkt_light_data()).value.val) as *mut MeshPkt) });

            if rf_link_is_notify_req((*get_pkt_light_data()).value.val[0] & 0x3f) {
                let mut relay_time = min(
                    (((read_reg_system_tick() - bridge_cmd_time) / CLOCK_SYS_CLOCK_1US) + 500) >> 10,
                    0xff
                );

                (*get_pkt_light_data()).value.val[17] = relay_time as u8;
            }

            for chn in 0..4 {
                rf_set_ble_channel(SYS_CHN_LISTEN[chn]);
                rf_start_stx_mesh(&*get_pkt_light_data(), CLOCK_SYS_CLOCK_1US * 0x1e + read_reg_system_tick());
                sleep_us(700);
            }
        }
    }
}

fn mesh_user_command_pkt_enc2buf()
{
    if !*get_security_enable() {
        get_pkt_mesh_user_cmd_buf().clone_from(unsafe { transmute(&*get_pkt_user_cmd()) });
    } else {
        (*get_pkt_user_cmd())._type |= BIT!(7);
        get_pkt_mesh_user_cmd_buf().clone_from(unsafe { transmute(&*get_pkt_user_cmd()) });
        pair_enc_packet_mesh(get_pkt_mesh_user_cmd_buf_addr());
    }
}

pub fn mesh_send_user_command() -> u8
{
    if *get_mesh_user_cmd_idx() == 0 {
        return 0;
    }
    set_mesh_user_cmd_idx(*get_mesh_user_cmd_idx() - 1);
    critical_section::with(|_| {
        rf_set_tx_rx_off();
        rf_set_ble_access_code(*get_pair_ac());
        rf_set_ble_crc_adv();

        let mut user_cmd = false;

        if ((((*get_pkt_user_cmd()).handle1 as u32) << 0x1e) as i32) < 0 {
            if ((((*get_pkt_user_cmd()).handle1 as u32) << 0x1e) as i32) < 0 {
                user_cmd = true;
            }

            mesh_user_command_pkt_enc2buf();
        }

        for chn in SYS_CHN_LISTEN {
            if !user_cmd {
                rf_link_proc_ttc(*get_slave_tx_cmd_time(), 0, unsafe { &mut *(get_pkt_user_cmd_addr() as *mut MeshPkt) });
                mesh_user_command_pkt_enc2buf();
            }
            rf_set_ble_channel(chn);
            rf_start_srx2tx(get_pkt_mesh_user_cmd_buf_addr() as u32, CLOCK_SYS_CLOCK_1US * 0x1e + read_reg_system_tick());
        }
    });
    *get_mesh_user_cmd_idx()
}

pub fn tx_packet_bridge()
{
    static TICK_BRIDGE_REPORT: AtomicU32 = AtomicU32::new(0);
    static LAST_ALARM_TIME_BRIDGE: AtomicU32 = AtomicU32::new(0);
    static LAST_CONN_IBEACON_TIME: AtomicU32 = AtomicU32::new(0);

    critical_section::with(|_| {
        rf_set_tx_rx_off();

        sleep_us(100);

        rf_set_ble_access_code(*get_pair_ac());
        rf_set_ble_crc_adv();

        let tick = read_reg_system_tick();
        if *get_slave_listen_interval() * *get_online_status_interval2listen_interval() as u32 - ONLINE_STATUS_COMP * CLOCK_SYS_CLOCK_1US * 1000 < tick - TICK_BRIDGE_REPORT.load(Ordering::Relaxed) {
            TICK_BRIDGE_REPORT.store(tick, Ordering::Relaxed);
            if CLOCK_SYS_CLOCK_1US * 30000000 < tick - LAST_ALARM_TIME_BRIDGE.load(Ordering::Relaxed) {
                LAST_ALARM_TIME_BRIDGE.store(tick, Ordering::Relaxed);
                // noop
            } else {
                mesh_send_online_status();
            }
        }
        app_bridge_cmd_handle(*get_t_bridge_cmd());

        if *get_slave_data_valid() == 0 {
            mesh_send_user_command();
        }
    });
}

pub fn rf_link_slave_proc() {
    set_mesh_info_time_handle();
    app().mesh_manager.mesh_pair_proc();
    update_connect_para();
}

pub fn is_add_packet_buf_ready() -> bool
{
    return (read_reg_dma_tx_wptr() - read_reg_dma_tx_rptr() & 7) < 3;
}

pub fn rf_link_add_tx_packet(packet: *const PacketAttCmd, size: usize) -> bool
{
    let wptr = read_reg_dma_tx_wptr();
    let rptr = read_reg_dma_tx_rptr();
    let widx = (wptr - rptr) & 7;

    if widx < 4 {
        if widx == 0 {
            write_reg_dma_tx_fifo((*get_pkt_empty()).as_ptr() as u16);
        }

        let idx = (*get_blt_tx_wptr() as usize & 7) * 48;
        let mut dest = &mut (*get_blt_tx_fifo())[idx..idx + size];

        dest.copy_from_slice(
            unsafe {
                slice::from_raw_parts(
                    packet as *const u8,
                    size,
                )
            }
        );

        if idx + size < 0x28 {
            (*get_blt_tx_fifo())[idx + size..idx + 0x28].fill(0);
        }

        set_blt_tx_wptr(*get_blt_tx_wptr() + 1);
        pair_enc_packet(unsafe { transmute(dest.as_mut_ptr()) });

        write_reg_dma_tx_fifo(dest.as_mut_ptr() as u32 as u16);
        return true;
    }
    return false;
}

pub fn rf_link_slave_read_status_par_init()
{
    set_slave_status_buffer_wptr(0);
    set_slave_status_buffer_rptr(0);
    set_slave_stat_sno([0; 3]);
}

pub fn rf_link_slave_read_status_stop()
{
    set_slave_read_status_busy(0);
    set_slave_read_status_unicast_flag(0);
    set_slave_data_valid(0);
    rf_link_slave_read_status_par_init();
}

pub fn rf_ota_save_data(data: *const u8) -> OtaState
{
    let addr = *get_cur_ota_flash_addr() + FLASH_ADR_LIGHT_NEW_FW;
    flash_write_page(addr, 0x10, data);

    let mut tmp = [0u8; 0x10];
    flash_read_page(addr, 0x10, tmp.as_mut_ptr());

    if unsafe { slice::from_raw_parts(data, 0x10) } == tmp {
        set_cur_ota_flash_addr(*get_cur_ota_flash_addr() + 0x10);
        return OtaState::Continue;
    } else {
        return OtaState::Error;
    }
}

pub fn rf_link_match_group_mac(sno: *const AppCmdValue) -> (bool, bool)
{
    let mut group_match = false;
    let mut device_match = false;

    unsafe {
        if (*sno).dst & !DEVICE_ADDR_MASK_DEFAULT != 0 {
            for addr in *get_group_address() {
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
            if addr == 0 || addr == *get_device_address() {
                device_match = true;
            }
        }
    }

    (group_match, device_match)
}

fn mesh_push_user_command_ll(sno: u32, dst: u16, cmd_op_para: *const u8, len: u8, max_relay: u8) -> bool
{
    if len > 2 {
        let mut pkt_len = 0xd;
        if unsafe { *cmd_op_para } & 0x3f == 6 {
            pkt_len = 0x12;
        }

        if len <= pkt_len {
            pkt_len = 0xd;
            if unsafe { *cmd_op_para } & 0x3f == 6 {
                pkt_len = len;
            }

            let cmd_op_para = unsafe { slice::from_raw_parts(cmd_op_para, pkt_len as usize) };

            critical_section::with(|_| {
                if cmd_op_para[0] & 0x3f == 6 {
                    set_mesh_user_cmd_idx(0x20);
                    if 0xff00 > cmd_op_para[4] as u16 * 0x100 + cmd_op_para[3] as u16 {
                        set_mesh_user_cmd_idx(4);
                    }
                } else {
                    set_mesh_user_cmd_idx(BRIDGE_MAX_CNT as u8);
                }

                (*get_pkt_user_cmd()).dma_len = 0x27;
                (*get_pkt_user_cmd()).l2cap_len = 0x21;
                (*get_pkt_user_cmd()).rf_len = 0x25;

                (*get_pkt_user_cmd())._type = 2;
                (*get_pkt_user_cmd()).chan_id = 0xff03;
                (*get_pkt_user_cmd()).src_tx = *get_device_address() as u16;
                (*get_pkt_user_cmd()).handle1 = 0;
                (*get_pkt_user_cmd()).op = 0;
                (*get_pkt_user_cmd()).vendor_id = 0;
                (*get_pkt_user_cmd()).par.fill(0);
                (*get_pkt_user_cmd()).internal_par1.fill(0);
                (*get_pkt_user_cmd()).ttl = 0;
                (*get_pkt_user_cmd()).internal_par2[0] = 0;
                (*get_pkt_user_cmd()).sno.copy_from_slice(unsafe {
                    slice::from_raw_parts(addr_of!(sno) as *const u8, 3)
                });
                (*get_pkt_user_cmd()).src_adr = *get_device_address();
                (*get_pkt_user_cmd()).dst_adr = dst;
                unsafe {
                    slice::from_raw_parts_mut(addr_of_mut!((*get_pkt_user_cmd()).op), pkt_len as usize).copy_from_slice(cmd_op_para)
                }

                (*get_pkt_user_cmd()).internal_par1[CURRENT_RELAY_COUNT] = max_relay;

                let (group_match, device_match) = rf_link_match_group_mac(addr_of_mut!((*get_pkt_user_cmd()).sno) as *const AppCmdValue);
                set_slave_tx_cmd_time(read_reg_system_tick());
                if group_match || device_match {
                    rf_link_data_callback(addr_of!((*get_pkt_user_cmd()).l2cap_len) as *const PacketL2capData);
                    if device_match {
                        set_mesh_user_cmd_idx(0);
                    }
                }
            });

            return true;
        }
    }
    return false;
}

pub fn mesh_push_user_command(sno: u32, dst: u16, cmd_op_para: *const u8, len: u8) -> bool
{
    mesh_push_user_command_ll(sno, dst, cmd_op_para, len, *get_max_relay_num() + 1)
}

pub fn mesh_report_status_enable(enable: bool)
{
    if enable {
        if *get_mesh_node_max() >> 5 != 0 {
            (*get_mesh_node_mask()).iter_mut().for_each(|v| { *v = 0xfffffffe });
        }

        if *get_mesh_node_max() & 0x1f != 0 {
            (*get_mesh_node_mask())[*get_mesh_node_max() as usize >> 5] = (1 << (*get_mesh_node_max() & 0x1f)) - 1;
        }
    }

    set_mesh_node_report_enable(enable);
}

pub fn mesh_report_status_enable_mask(data: *const u8, len: u16)
{
    set_mesh_node_report_enable(if unsafe { *data } != 0 { true } else { false });
    if *get_mesh_node_report_enable() && len > 1 {
        for index in 1..len {
            if *get_mesh_node_max() != 0 {
                (*get_mesh_node_st()).iter_mut().enumerate().for_each(|(i, v)| {
                    if unsafe { *data.offset(index as isize) } == v.val.dev_adr {
                        (*get_mesh_node_mask())[i >> 5] |= 1 << (i & 0x1f);
                    }
                });
            }
        }
    }
}

pub fn rf_link_delete_pair()
{
    let mut key = [0u8; 32];

    key[0..0x10].copy_from_slice(&*get_pair_config_mesh_name());
    key[0x10..0x20].copy_from_slice(&*get_pair_config_mesh_pwd());

    pair_set_key(key.as_ptr());
    pair_save_key();
    if *get_not_need_login() == false {
        set_pair_login_ok(false);
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


