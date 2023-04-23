use core::mem::{size_of, transmute};
use core::ptr::{addr_of, addr_of_mut, null, null_mut, slice_from_raw_parts, slice_from_raw_parts_mut};
use core::slice;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use crate::{app, blinken, no_mangle_fn, pub_mut};
use crate::common::{dev_addr_with_mac_flag, get_conn_update_cnt, get_conn_update_successed, get_mesh_node_ignore_addr, get_sys_chn_adv, get_sys_chn_listen, rf_update_conn_para, set_conn_update_cnt, set_conn_update_successed, update_ble_parameter_cb};
use crate::config::get_flash_adr_light_new_fw;
use crate::main_light::{irq_timer0, irq_timer1, rf_link_data_callback, rf_link_response_callback};
use crate::mesh::wrappers::{get_mesh_node_mask, get_mesh_node_st, get_mesh_node_st_val_len, get_mesh_pair_enable, light_slave_tx_command_callback};
use crate::sdk::ble_app::ble_ll_att::ble_ll_channelTable_calc;
use crate::sdk::ble_app::ble_ll_attribute::{get_att_service_discover_tick, get_slave_link_time_out, l2cap_att_handler, set_att_service_discover_tick, set_slave_link_time_out};
use crate::sdk::ble_app::ble_ll_pair::{pair_dec_packet_mesh, pair_enc_packet, pair_enc_packet_mesh, pair_save_key, pair_set_key, pairInit};
use crate::sdk::ble_app::rf_drv_8266::{get_FtoRX, get_light_rx_wptr, get_pkt_ibeacon_addr, get_pkt_light_data, get_pkt_light_status, get_pkt_light_status_addr, get_pkt_scan_rsp, get_pkt_scan_rsp_addr, get_slave_connected_tick, get_slave_connection_enable, get_slave_link_state, get_slave_listen_interval, irq_st_response, rf_reset_sn, rf_set_ble_access_code, rf_set_ble_access_code_adv, rf_set_ble_channel, rf_set_ble_crc, rf_set_ble_crc_adv, rf_set_rxmode, rf_set_tx_rx_off, rf_start_beacon, rf_start_srx2tx, rf_start_stx2rx, rf_stop_trx, set_FtoRX, set_light_rx_wptr, set_slave_adv_enable, set_slave_connected_tick, set_slave_connection_enable};
use crate::sdk::ble_app::shared_mem::{get_blt_tx_fifo, get_light_rx_buff};
use crate::sdk::drivers::flash::{flash_read_page, flash_write_page};
use crate::sdk::light::{rf_packet_l2cap_head_t, get_app_cmd_time, get_beacon_with_mesh_adv, get_command_type, get_device_address, get_device_address_addr, get_device_node_sn, get_gateway_en, get_gateway_tx_wptr, get_gateway_tx_rptr, get_iBeaconInterval, get_interval_th, get_loop_interval_us, get_lpn_retransmit_cnt, get_mesh_chn_amount, get_mesh_node_cur, get_mesh_node_max, get_mesh_send_online_status_flag, get_mesh_user_cmd_idx, get_need_update_connect_para, get_online_status_comp, get_online_status_interval2listen_interval, get_online_status_timeout, get_p_mesh_node_status_callback, get_pair_ac, get_passive_en, get_pkt_light_adv_status, get_pkt_light_adv_status_addr, get_pkt_mesh, get_pkt_mesh_addr, get_pkt_mesh_user_cmd_buf, get_pkt_mesh_user_cmd_buf_addr, get_rf_slave_ota_busy, get_rf_slave_ota_busy_mesh_en, get_security_enable, get_send_self_online_status_cycle, get_set_mesh_info_expired_flag, get_set_mesh_info_time, get_slave_data_valid, get_slave_p_mac, get_slave_read_status_busy, get_slave_tx_cmd_time, get_st_brige_no, get_sw_flag, get_SW_Low_Power, get_sw_no_pair, get_switch_rf_tx_once_time, get_sync_time_enable, get_synced_flag, get_t_bridge_cmd, get_tick_per_us, get_tx_packet_bridge_delay_us, get_tx_packet_bridge_random_en, get_tx_packet_bridge_tick, get_update_connect_para_delay_ms, get_update_interval_user_max, get_update_interval_user_min, ll_adv_rsp_private_t, mesh_pkt_t, rf_packet_adv_ind_module_t, rf_packet_att_cmd_t, rf_packet_ll_app_t, rf_packet_ll_data_t, rf_packet_ll_init_t, rf_packet_scan_rsp_t, set_device_node_sn, set_lpn_retransmit_cnt, set_mesh_node_cur, set_mesh_ota_master_ui_sending, set_mesh_user_cmd_idx, set_need_update_connect_para, set_set_mesh_info_expired_flag, set_slave_data_valid, set_slave_link_connected, set_slave_sno_sending, set_tick_per_us, set_tx_packet_bridge_delay_us, set_tx_packet_bridge_tick, set_update_ble_par_success_flag, set_update_interval_flag, set_update_interval_time, set_update_interval_user_max, set_update_interval_user_min, set_update_timeout_user, status_record_t, set_gateway_tx_wptr, get_gateway_security, get_blt_tx_wptr, set_blt_tx_wptr, get_mesh_notify_enc_enable, set_slave_status_buffer_wptr, set_slave_status_buffer_rptr, set_slave_stat_sno, set_slave_read_status_busy, set_slave_read_status_unicast_flag, OtaState, get_cur_ota_flash_addr, set_cur_ota_flash_addr, get_mesh_ota_master_st, get_max_relay_num, app_cmd_value_t, DEVICE_ADDR_MASK_DEFAULT, get_group_address, get_device_address_mask, set_slave_tx_cmd_time, ll_packet_l2cap_data_t, set_mesh_node_report_enable, get_mesh_node_report_enable, get_pair_config_mesh_name, get_pair_config_mesh_pwd, get_not_need_login, set_pair_login_ok, get_slave_timing_adjust_enable, get_slave_tick_brx, set_slave_timing_adjust_enable, get_slave_pairing_master_tick, get_slave_conn_delay, get_slave_window_offset, set_slave_window_offset, set_slave_link_sno, set_slave_first_connected_tick, set_slave_status_tick, get_slave_n6, set_slave_n6, get_pkt_init, set_slave_instant, set_ADV_Suspend, set_enc_disable, get_slave_link_connected, get_slave_link_sno_addr, get_p_cb_rx_from_mesh, get_slave_sno_sending_addr, get_rc_pkt_buf, get_mesh_cmd_cache_num, get_slave_link_cmd, set_rcv_pkt_ttc, set_slave_pairing_state, get_req_cmd_in_adv_type, CMD_TYPE, set_slave_read_status_response, get_mesh_ota_slave_st, get_slave_sno, set_slave_link_cmd, get_slave_read_status_response, rf_packet_att_value_t, get_pair_setting_flag, get_slave_stat_sno, get_slave_sno_addr, set_SW_Low_Power_rsp_flag, get_org_ttl, set_org_ttl, get_rcv_pkt_ttc, PairState, get_slave_sno_sending, get_pkt_need_relay};
use crate::sdk::mcu::clock::{clock_time, sleep_us};
use crate::sdk::mcu::register::{FLD_IRQ, FLD_RF_IRQ_MASK, read_reg_dma_tx_rptr, read_reg_dma_tx_wptr, read_reg_irq_src, read_reg_mcu_wakeup_mask, read_reg_rf_irq_status, read_reg_rf_rx_status, read_reg_rnd_number, read_reg_system_tick, read_reg_system_tick_irq, read_reg_tmr_ctrl8, write_reg32, write_reg8, write_reg_dma2_addr, write_reg_dma3_addr, write_reg_dma_tx_fifo, write_reg_irq_src, write_reg_mcu_wakeup_mask, write_reg_pwdn_ctrl, write_reg_rf_irq_status, write_reg_system_tick_irq, write_reg_tmr1_capt, write_reg_tmr1_tick, write_reg_tmr_ctrl8, write_reg_tmr_sta};
use crate::uart_manager::get_pkt_user_cmd;
use crate::vendor_light::{get_adv_rsp_pri_data, get_adv_rsp_pri_data_addr};

pub_mut!(slave_timing_update, u32);
pub_mut!(slave_instant_next, u16);
pub_mut!(slave_chn_map, [u8; 5]);
pub_mut!(slave_interval_old, u32);
pub_mut!(slave_link_interval, u32);
pub_mut!(slave_window_size_update, u32);
pub_mut!(ble_conn_timeout, u32);
pub_mut!(ble_conn_interval, u32);
pub_mut!(ble_conn_offset, u32);
pub_mut!(slave_ll_rsp, *const rf_packet_l2cap_head_t);
pub_mut!(add_tx_packet_rsp_failed, u32, 0);
pub_mut!(T_scan_rsp_intvl, u32); //, 0x92);
pub_mut!(p_slave_status_buffer, *mut [u32; 9]); //, null_mut());
pub_mut!(slave_status_buffer_num, u8); //, 0);
pub_mut!(bridge_max_cnt, u32); //, 8);
pub_mut!(g_vendor_id, u16); //, 0x211);

pub_mut!(pkt_light_notify, rf_packet_att_cmd_t);
//, rf_packet_att_cmd_t {
//     dma_len: 0x1D,
//     _type: 2,
//     rf_len: 0x1B,
//     l2capLen: 0x17,
//     chanId: 4,
//     opcode: 0x1B,
//     handle: 0x12,
//     handle1: 0,
//     value: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xea, 0x11, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
// });
pub_mut!(pkt_light_report, rf_packet_att_cmd_t);
//, rf_packet_att_cmd_t {
//     dma_len: 0x1D,
//     _type: 2,
//     rf_len: 0x1B,
//     l2capLen: 0x17,
//     chanId: 4,
//     opcode: 0x1B,
//     handle: 0x12,
//     handle1: 0,
//     value: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x11, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
// });

no_mangle_fn!(irq_st_ble_rx);

fn is_exist_in_rc_pkt_buf(opcode: u8, cmd_pkt: &app_cmd_value_t) -> bool
{
    for idx in 0..*get_mesh_cmd_cache_num() as usize {
        if (*get_rc_pkt_buf())[idx].op == opcode && (*get_rc_pkt_buf())[idx].sno == cmd_pkt.sno {
            return true;
        }
    }

    return false;
}

fn rf_link_is_notify_rsp(opcode: u8) -> bool
{
    if opcode == 0x14 || opcode == 0x1b || opcode == 0x16 || opcode == 0x15 || opcode == 1 || opcode == 0x27 || opcode == 0x21 || opcode == 0x29 {
        return true;
    } else {
        // todo May need to be !=
        return (opcode == 8) | (opcode == 0x2b);
    }
}

fn rc_pkt_buf_push(opcode: u8, cmd_pkt: &app_cmd_value_t)
{
    static RC_PKT_BUF_IDX: AtomicU8 = AtomicU8::new(0);

    if !rf_link_is_notify_rsp(opcode) {
        if !is_exist_in_rc_pkt_buf(opcode, cmd_pkt) {
            let pkt_idx = RC_PKT_BUF_IDX.load(Ordering::Relaxed) as usize;
            (*get_rc_pkt_buf())[pkt_idx].op = opcode;
            (*get_rc_pkt_buf())[pkt_idx].sno = cmd_pkt.sno;
            (*get_rc_pkt_buf())[pkt_idx].notify_ok_flag = false;
            (*get_rc_pkt_buf())[pkt_idx].sno2.copy_from_slice(&cmd_pkt.par[8..10]);
            RC_PKT_BUF_IDX.store(((pkt_idx + 1) % *get_mesh_cmd_cache_num() as usize) as u8, Ordering::Relaxed);
        }
    }
    return;
}

fn req_cmd_is_notify_ok(opcode: u8, cmd_pkt: &app_cmd_value_t) -> bool
{
    for idx in 0..*get_mesh_cmd_cache_num() as usize {
        if (*get_rc_pkt_buf())[idx].op == opcode && (*get_rc_pkt_buf())[idx].sno == cmd_pkt.sno {
            return (*get_rc_pkt_buf())[idx].notify_ok_flag;
        }
    }

    return false;
}

fn req_cmd_set_notify_ok_flag(opcode: u8, cmd_pkt: &app_cmd_value_t)
{
    (*get_rc_pkt_buf()).iter_mut().filter(
        |v| { v.op == opcode && v.sno == cmd_pkt.sno }
    ).for_each(
        |v| { v.notify_ok_flag = true }
    );
}

fn copy_par_User_All(params_len: u32, ptr: *const u8)
{
    (*get_pkt_light_status()).value[10..10 + params_len as usize].copy_from_slice(
        unsafe {
            slice::from_raw_parts(ptr, params_len as usize)
        }
    );
}

no_mangle_fn!(rf_link_slave_add_status_ll, packet: *const mesh_pkt_t);

fn rf_link_slave_add_status(packet: *const mesh_pkt_t)
{
    critical_section::with(|_| {
        _rf_link_slave_add_status_ll(packet);
    });
}

no_mangle_fn!(mesh_node_update_status, bool, pkt: *const rf_packet_att_value_t, len: u32);

// pub_mut!(cb_mesh_node_filter, u32, 0);
// no_mangle_fn!(rf_link_rc_data, bool, packet: &mut mesh_pkt_t);
#[inline(never)]
fn rf_link_rc_data(packet: &mut mesh_pkt_t) -> bool {
    if packet.rf_len != 0x25 || packet.l2capLen != 0x21 {
        if *get_sw_no_pair() == false {
            return false;
        }
        if packet.rf_len != 0x24 {
            return false;
        }
        if packet.l2capLen != 0x20 {
            return false;
        }
        if packet.chanId != 0xff04 {
            return false;
        }
        set_enc_disable(true);
    }

    if *get_slave_link_connected() {
        if 0x3fffffffi32 < (read_reg_system_tick_irq() as i32 - read_reg_system_tick() as i32) - (*get_tick_per_us() * 1000) as i32 {
            set_enc_disable(false);
            return false;
        }
    }

    if unsafe { !pair_dec_packet_mesh(packet) } {
        set_enc_disable(false);
        return false;
    }

    if packet._type & 3 != 2 {
        set_enc_disable(false);
        return false;
    }

    if packet.chanId == 0xeeff {
        set_enc_disable(false);
        return false;
    }

    if packet.chanId == 0xffff {
        if *get_mesh_node_st_val_len() == 4 {
            if packet.internal_par1[3] != 0xa5 || packet.internal_par1[4] != 0xa5 {
                set_enc_disable(false);
                return false;
            }
        }

        // todo: ttl? Probably packet is not the right type in this check
        if packet.ttl != 0xa5 || packet.internal_par2[0] != 0xa5 {
            set_enc_disable(false);
            return false;
        }

        _mesh_node_update_status(addr_of!(packet.sno) as *const rf_packet_att_value_t, 0x1a / *get_mesh_node_st_val_len() as u32);
        set_enc_disable(false);
        return false;
    }

    if *get_max_relay_num() + 6 < packet.internal_par1[4] {
        set_enc_disable(false);
        return false;
    }

    let mut op_cmd: [u8; 8] = [0; 8];
    let mut op_cmd_len: u8 = 0;
    let mut params: [u8; 16] = [0; 16];
    let mut params_len: u8 = 0;
    let l2cap_data = unsafe { &*(addr_of_mut!(packet.l2capLen) as *const ll_packet_l2cap_data_t) };
    if !rf_link_get_op_para(l2cap_data, &mut op_cmd, &mut op_cmd_len, &mut params, &mut params_len, true) {
        set_enc_disable(false);
        return false;
    }

    let mut op = 0;
    if op_cmd_len == 3 {
        op = op_cmd[0] & 0x3f;
    }

    let cmd_pkt = unsafe { &*(addr_of_mut!(packet.sno) as *const app_cmd_value_t) };
    let match_slave_sno = cmd_pkt.sno != unsafe { slice::from_raw_parts(get_slave_sno_addr() as *const u8, 3) };
    let match_slave_sno_sending = cmd_pkt.sno == unsafe { slice::from_raw_parts(get_slave_sno_sending_addr() as *const u8, 3) };
    let pkt_exists_in_buf = is_exist_in_rc_pkt_buf(op, cmd_pkt);
    if (*get_p_cb_rx_from_mesh()).is_some() && (match_slave_sno || op != *get_slave_link_cmd()) && !pkt_exists_in_buf
    {
        (*get_p_cb_rx_from_mesh()).unwrap()(cmd_pkt);
    }

    let src_device_addr_match = packet.src_adr == *get_device_address();

    let mut iVar12 = false;
    if packet.dst_adr == *get_device_address() {
        iVar12 = true;
        if *get_slave_link_connected() == false {
            let uVar20 = *get_req_cmd_in_adv_type();
            if uVar20 != 1 {
                iVar12 = false;
                if 1 < (*get_mesh_ota_master_st())[20] {
                    // todo: Might need to be !=
                    iVar12 = uVar20 - 2 == 0;
                }
            }
        }
    }

    let iStack_6c;
    let (mut result1, mut result2) = (false, false);
    if rf_link_is_notify_rsp(op) && iVar12 {
        if op != 8 || *get_mesh_pair_enable() == false || *get_pair_setting_flag() == PairState::PairSetted {
            if *get_slave_read_status_busy() != op || cmd_pkt.sno != *get_slave_stat_sno() {
                set_enc_disable(false);
                return false;
            }
        }
        rf_link_slave_add_status(packet);
        set_enc_disable(false);
        return false;
    } else {
        if src_device_addr_match && !*get_mesh_node_ignore_addr() {
            if op != 0x20 || dev_addr_with_mac_flag(params.as_ptr()) || match_slave_sno_sending {
                set_enc_disable(false);
                return false;
            }
        }
        set_slave_pairing_state(0);

        set_rcv_pkt_ttc(packet.par[9]);

        (result1, result2) = rf_link_match_group_mac(cmd_pkt);

        iStack_6c = (match_slave_sno || *get_slave_link_cmd() != op) && !pkt_exists_in_buf;

        if result1 != false || result2 != false {
            if iStack_6c {
                rc_pkt_buf_push(op, cmd_pkt);
                rf_link_data_callback(l2cap_data);
            }
        }

        if rf_link_is_notify_req(op) {
            set_slave_read_status_response(true);
            if result1 == false {
                // todo: May need to be !result2
                set_slave_read_status_response(result2);
            }
            let bVar8 = req_cmd_is_notify_ok(op, cmd_pkt);
            if bVar8 {
                set_slave_read_status_response(false);
                if op == 7 && (*get_mesh_ota_slave_st())[21] == 7 && (*get_mesh_ota_slave_st())[18] != 0 {
                    (*get_mesh_ota_slave_st())[19] = 0x20;
                }
            } else if (packet.dst_adr >> 8) & 0x80 != 0 {
                for i in 0..5 {
                    if packet.par[i + 4] == *get_device_address() as u8 {
                        req_cmd_set_notify_ok_flag(op, cmd_pkt);
                        set_slave_read_status_response(false);
                        break;
                    }
                }
            }
        }
    }
    let bVar2 = packet.internal_par1[4];
    let uVar3 = packet.l2capLen;
    let iVar7 = uVar3 + 6;

    packet.dma_len = (iVar7 as u32) & 0xffffff;
    packet.rf_len = uVar3 as u8 + 4;

    let mut bVar1 = bVar2;

    if match_slave_sno || *get_slave_link_cmd() != op {
        packet.src_tx = *get_device_address();
        (*get_slave_sno())[0..3].copy_from_slice(&cmd_pkt.sno);

        set_slave_link_cmd(op);
    } else {
        bVar1 = *get_org_ttl();
    }

    set_org_ttl(bVar1);

    let uVar10 = (read_reg_system_tick() ^ unsafe { *(*get_slave_p_mac()) as u32 }) & 0xf;
    let iVar7 = uVar10 * 500;
    if rf_link_is_notify_req(op) && *get_slave_read_status_response() {
        (*get_pkt_light_status()).value[0..3].copy_from_slice(&cmd_pkt.sno[0..3]);
        packet.src_tx = *get_device_address();

        if op == 0x1a {
            (*get_pkt_light_status()).value[22] = 0;
            if iStack_6c {
                (*get_pkt_light_status()).value[20] = packet.par[9];
                (*get_pkt_light_status()).value[25] = packet.internal_par1[4];
                if *get_max_relay_num() < packet.internal_par1[4] {
                    (*get_pkt_light_status()).value[21] = 0;
                } else {
                    (*get_pkt_light_status()).value[21] = *get_max_relay_num() - packet.internal_par1[4];
                }
            }
        } else if op == 0x1d {
            (*get_pkt_light_status()).value[22] = packet.internal_par1[1];
        } else if op == 0x17 {
            (*get_pkt_light_status()).value[22] = 1;
        } else if op == 0x20 {
            (*get_pkt_light_status()).value[22] = 4;
        } else if op == 0x26 {
            (*get_pkt_light_status()).value[22] = 5;
        } else if op == 0x28 {
            (*get_pkt_light_status()).value[22] = 6;
        } else if op == 0x2a {
            (*get_pkt_light_status()).value[22] = 7;
        } else if op == 7 {
            (*get_pkt_light_status()).value[22] = 9;
        } else if op == 0 {
            (*get_pkt_light_status()).value[22] = 8;
        }
        unsafe { copy_par_User_All(params_len as u32, (addr_of!(packet.vendor_id) as u32 + 1) as *const u8); }
        if (match_slave_sno || *get_slave_link_cmd() != op) || ((op != 0) && (op != 0x26) || params[1] != 0) {
            (*get_pkt_light_status()).value[3..3 + 2].copy_from_slice(unsafe { slice::from_raw_parts(addr_of!(packet.src_adr) as *const u8, 2) });

            let mut rStack_64: rf_packet_att_value_t = rf_packet_att_value_t {
                sno: [0; 3],
                src: [0; 2],
                dst: [0; 2],
                val: [0; 23],
            };

            unsafe {
                slice::from_raw_parts_mut(addr_of_mut!(rStack_64) as *mut u8, size_of::<rf_packet_att_value_t>()).copy_from_slice(
                    slice::from_raw_parts(
                        addr_of!(*cmd_pkt) as *const u8,
                        size_of::<rf_packet_att_value_t>(),
                    )
                )
            }

            if rf_link_response_callback(addr_of_mut!((*get_pkt_light_status()).value) as *mut rf_packet_att_value_t, &rStack_64) {
                if *get_SW_Low_Power() == false {
                    if *get_slave_link_connected() == false {
                        write_reg_irq_src(0x100000);
                        let iVar12 = ((unsafe { *(*get_slave_p_mac()) as u32 } ^ (read_reg_rnd_number() as u32 ^ read_reg_system_tick()) & 0xffff) & 0xf) * 700 + 4000;
                        let mut puVar9 = 0x3e80 + iVar12 + iVar7;
                        let bVar1 = packet.internal_par1[2];
                        if 0x1d < bVar1 {
                            let mut uVar11 = (((read_reg_system_tick() - *get_rcv_pkt_time()) / *get_tick_per_us()) + 500 >> 10) + packet.internal_par1[3] as u32;
                            if 0xff < uVar11 {
                                uVar11 = 0xff;
                            }
                            let uVar11 = bVar1 as u32 - uVar11;
                            if uVar11 < 0x32 {
                                puVar9 = uVar11 * 1000;
                                if result2 == false {
                                    if result1 != false && *get_max_relay_num() - packet.internal_par1[4] < 3 {
                                        puVar9 = puVar9 + iVar12 + iVar7;
                                    }
                                } else {
                                    puVar9 = puVar9 + 2000;
                                }
                            }
                        }
                        write_reg_system_tick_irq(puVar9 * *get_tick_per_us() + read_reg_system_tick());
                        set_p_st_handler(Some(irq_st_response));
                    } else {
                        rf_set_tx_rx_off();
                        sleep_us(100);
                        rf_set_ble_access_code(*get_pair_ac());
                        rf_set_ble_crc_adv();
                        for chn in *get_sys_chn_listen()
                        {
                            if *get_slave_link_connected() != false {
                                if 0x3fffffffi32 < (read_reg_system_tick_irq() as i32 - read_reg_system_tick() as i32) - (*get_tick_per_us() * 2000) as i32 {
                                    rf_stop_trx();
                                    set_enc_disable(false);
                                    return false;
                                }
                            }
                            rf_set_ble_channel(chn);
                            (*get_pkt_light_status())._type |= 0x7f;
                            rf_start_stx_mesh(get_pkt_light_status(), *get_tick_per_us() * 0x1e + read_reg_system_tick());
                            sleep_us(600);
                        }
                    }
                } else {
                    set_SW_Low_Power_rsp_flag(1);
                }
            }
        }
    }

    if *get_pkt_need_relay() && *get_SW_Low_Power() == false && bVar2 != 0 && *get_org_ttl() == bVar2
    {
        rf_set_tx_rx_off();
        sleep_us(100);
        rf_set_ble_access_code(*get_pair_ac());
        rf_set_ble_crc_adv();
        if *get_slave_read_status_busy() == 0 || !rf_link_is_notify_rsp(op) {
            packet.internal_par1[4] = bVar2 - 1;
        }
        if result2 == false {
            let mut iVar7 = 100;
            if *get_slave_link_connected() == false {
                iVar7 = uVar10 * 500 - 8000;
            }
            sleep_us(iVar7);

            op = packet.internal_par1[3];
            let tmp = (read_reg_system_tick() ^ unsafe { *(*get_slave_p_mac()) as u32 }) & 3;
            for uVar11 in tmp..tmp + 4 {
                if *get_slave_link_connected() != false {
                    if 0x3fffffffi32 < (read_reg_system_tick_irq() as i32 - read_reg_system_tick() as i32) - (*get_tick_per_us() * 2000) as i32 {
                        rf_stop_trx();
                        set_enc_disable(false);
                        return false;
                    }
                }
                rf_set_ble_channel((*get_sys_chn_listen())[uVar11 as usize & 3]);
                packet._type |= 0x7f;
                if op != 0x1b {
                    rf_link_proc_ttc(*get_rcv_pkt_time(), *get_rcv_pkt_ttc() as u32, addr_of_mut!(packet.par[9]));
                }
                if rf_link_is_notify_req(op) {
                    let mut uVar8 = (((read_reg_system_tick() - *get_rcv_pkt_time()) / *get_tick_per_us() + 500) >> 10) + op as u32;
                    if 0xff < uVar8 {
                        uVar8 = 0xff;
                    }
                    packet.internal_par1[3] = uVar8 as u8;
                }
                rf_start_stx_mesh(unsafe { &*(addr_of!(*packet) as *const rf_packet_att_cmd_t) }, *get_tick_per_us() * 0x1e + read_reg_system_tick());
                sleep_us(600);
            }
        }
        rf_set_rxmode();
    }
    set_enc_disable(false);
    return true;
}

pub unsafe fn rf_link_slave_data(packet: &rf_packet_ll_data_t, time: u32) -> bool {
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
                    *v = *(addr_of!(packet.l2cap_high) as *const u8).offset(i as isize);
                });

                return true;
            }

            if rf_len == 0xc && packet._type & 3 == 3 && packet.l2cap_low == 0 {
                set_slave_interval_old(*get_slave_link_interval());
                set_slave_instant_next(packet.group);
                set_slave_window_size_update((packet.l2cap_high as u32 * 1250 + 1300) * *get_tick_per_us());
                set_slave_timing_update(2);
                set_ble_conn_interval(*get_tick_per_us() * 1250 * (*addr_of!(packet.att) as *const u16) as u32);
                set_ble_conn_offset(packet.chanid as u32 * *get_tick_per_us() * 1250);
                set_ble_conn_timeout(packet.nid as u32 * 10000);
                return false;
            }
        }
        set_slave_ll_rsp(l2cap_att_handler(addr_of!(*packet) as *const ll_packet_l2cap_data_t));
        if *get_slave_ll_rsp() != null() && !rf_link_add_tx_packet(*get_slave_ll_rsp() as *const rf_packet_att_cmd_t) {
            set_add_tx_packet_rsp_failed(*get_add_tx_packet_rsp_failed() + 1);
        }
        return false;
    }

    return false;
}

pub_mut!(p_st_handler, Option<unsafe extern "C" fn()>);

#[link_section = ".ram_code"]
pub unsafe fn irq_light_slave_handler() {
    let irq = read_reg_rf_irq_status();
    if irq & FLD_RF_IRQ_MASK::IRQ_RX as u16 != 0 {
        irq_light_slave_rx();
    }

    if irq & FLD_RF_IRQ_MASK::IRQ_TX as u16 != 0 {
        irq_light_slave_tx();
    }

    let irq_source = read_reg_irq_src();
    if irq_source & FLD_IRQ::SYSTEM_TIMER as u32 != 0 {
        write_reg_irq_src(FLD_IRQ::SYSTEM_TIMER as u32);
        if p_st_handler.is_some() {
            p_st_handler.unwrap()();
        }
    }

    if irq_source & FLD_IRQ::TMR0_EN as u32 != 0 {
        irq_timer0();
        write_reg_irq_src(FLD_IRQ::TMR0_EN as u32);
    }
    if irq_source & FLD_IRQ::TMR1_EN as u32 != 0 {
        irq_timer1();
        write_reg_irq_src(FLD_IRQ::TMR1_EN as u32);
    }
}

// pub_mut!(light_rx_wptr, u32);
// pub_mut!(light_rx_buff, [u8; 256]);
pub_mut!(light_rcv_rssi, u8); //, 0);
// pub_mut!(t_rx_last, u32, 0);
pub_mut!(rcv_pkt_time, u32); //, 0);
// pub_mut!(slave_link_state, u32);
// pub_mut!(T_scan_rsp_intvl, u32);
// pub_mut!(pkt_scan_rsp, rf_packet_scan_rsp_t);
// pub_mut!(FtoRX, bool);
pub_mut!(light_conn_sn_master, u16); //, 0);
pub_mut!(disconnect_flag, bool); //, false);
// pub_mut!(slave_connected_tick, u32);
pub_mut!(slave_window_size, u32); //, 0);
pub_mut!(slave_timing_update2_flag, u32); //, 0);
pub_mut!(slave_next_connect_tick, u32); //, 0);
pub_mut!(slave_timing_update2_ok_time, u32); //, 0);
//
// no_mangle_fn!(rf_start_stx2rx, addr: u32, tick: u32);
// no_mangle_fn!(rf_link_rc_data, packet: *const u8, tick: u32);
//
// static pkt_empty: [u8; 6] = [02, 00, 00, 00, 01, 00];
pub_mut!(pkt_empty, [u8; 6]);

#[link_section = ".ram_code"]
fn irq_light_slave_tx()
{
    write_reg_rf_irq_status(2);
    if *get_FtoRX() {
        set_FtoRX(false);
    }
}

fn rf_link_timing_adjust(time: u32)
{
    if *get_slave_timing_adjust_enable() != 0 && !*get_gateway_en() {
        set_slave_timing_adjust_enable(0);
        if time - *get_slave_tick_brx() < *get_tick_per_us() * 700 {
            set_slave_next_connect_tick(*get_slave_next_connect_tick() - *get_tick_per_us() * 200);
        } else if *get_tick_per_us() * 1100 < time - *get_slave_tick_brx() {
            set_slave_next_connect_tick(*get_tick_per_us() * 200 + *get_slave_next_connect_tick());
        }
    }
}

fn check_par_con(packet: &rf_packet_ll_init_t) -> bool
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

fn rf_link_slave_connect(packet: &rf_packet_ll_init_t, time: u32) -> bool
{
    set_conn_update_successed(0);
    set_conn_update_cnt(0);
    if *get_slave_connection_enable() || packet.scanA == packet.advA || *get_slave_pairing_master_tick() != 0 {
        if check_par_con(packet) == false {
            rf_stop_trx();

            set_slave_window_offset(*get_tick_per_us() * 1250 * (packet.woffset as u32 + 1));

            write_reg_system_tick_irq(*get_tick_per_us() * 1000 + read_reg_system_tick());
            write_reg_irq_src(0x100000);
            write_reg_system_tick_irq(time + *get_slave_window_offset() + *get_tick_per_us() * (*get_slave_conn_delay() as u32 - if packet.woffset == 0 { 500 } else { 700 }));
            if 0x80000000 < read_reg_system_tick_irq() - read_reg_system_tick() {
                write_reg_system_tick_irq(*get_tick_per_us() * 10 + read_reg_system_tick());
            }

            set_light_conn_sn_master(0x80);
            set_slave_link_sno(0xfffffffe);
            set_slave_connected_tick(read_reg_system_tick());
            if *get_security_enable() != false {
                set_slave_first_connected_tick(*get_slave_connected_tick());
            }
            set_disconnect_flag(false);
            set_slave_status_tick(((packet.interval as u32 * 5) >> 2) as u8);
            set_slave_link_interval(packet.interval as u32 * *get_tick_per_us() * 1250);
            set_slave_window_size((packet.wsize as u32 * 1250 + 1100) * *get_tick_per_us());

            let tmp = *get_slave_link_interval() - *get_tick_per_us() * 1250;
            if tmp <= *get_slave_window_size() && *get_slave_window_size() - tmp != 0 {
                set_slave_window_size(tmp);
            }

            if packet.woffset == 0 {
                set_slave_n6(*get_slave_n6() + 1);
            }

            set_slave_link_time_out(packet.timeout as u32 * 10000);
            unsafe {
                slice::from_raw_parts_mut(
                    addr_of_mut!((*get_pkt_init()).scanA) as *mut u8,
                    0x22
                ).copy_from_slice(
                    slice::from_raw_parts(
                        addr_of!(packet.scanA) as *const u8,
                        0x22
                    )
                )
            };

            ble_ll_channelTable_calc((*get_pkt_init()).chm.as_ptr(), true);

            rf_set_ble_crc(&(*get_pkt_init()).crcinit);
            rf_reset_sn();

            set_slave_instant(0);
            set_slave_timing_update2_flag(0);
            set_slave_interval_old(0);
            set_slave_timing_update2_ok_time(0);
            set_slave_window_size_update(0);

            pairInit();

            set_mesh_node_report_enable(false);

            (*get_mesh_node_mask()).fill(0);

            set_p_st_handler(Some(irq_st_ble_rx));
            set_ADV_Suspend(0);
            set_need_update_connect_para(true);
            set_att_service_discover_tick(read_reg_system_tick() | 1);

            write_reg8(0x00800f04, 0x67);  // tx wail & settle time

            return true;
        }
    }
    return false;
}

#[link_section = ".ram_code"]
unsafe fn irq_light_slave_rx()
{
    static t_rx_last: AtomicU32 = AtomicU32::new(0);

    let rx_index = *get_light_rx_wptr() as usize;
    set_light_rx_wptr((*get_light_rx_wptr() + 1) & 3);

    let entry = &mut (*get_light_rx_buff())[rx_index];

    if read_reg_rf_rx_status() == 0x0b {
        write_reg_rf_irq_status(1);
        return;
    }

    write_reg_dma2_addr(addr_of!((*get_light_rx_buff())[*get_light_rx_wptr() as usize & 3]) as u16);

    write_reg_rf_irq_status(1);

    set_light_rcv_rssi(entry.rssi);

    let rx_time = entry.rx_time;
    let dma_len = entry.dma_len;
    if dma_len == 1 {
        if t_rx_last.load(Ordering::Relaxed) == rx_time {
            rf_stop_trx();
            rf_start_stx2rx(get_pkt_empty_addr() as u32, *get_tick_per_us() * 10 + read_reg_system_tick());
            return;
        }
    } else if dma_len > 0xe && dma_len == (entry.sno[1] & 0x3f) + 0x11 && *((addr_of!(*entry) as u32 + dma_len as u32 + 3) as *const u8) & 0x51 == 0x40 {
        let packet = addr_of!(entry.rx_time);
        set_rcv_pkt_time(rx_time);
        if entry.sno[0] & 0xf == 3 {
            if *get_slave_link_state() != 1 && *get_slave_link_state() != 7 {
                if *get_rf_slave_ota_busy() == false || *get_rf_slave_ota_busy_mesh_en() != 0 {
                    t_rx_last.store(rx_time, Ordering::Relaxed);
                    if *get_slave_link_state() - 4 < 2 {
                        rf_link_rc_data(&mut *(packet as *mut mesh_pkt_t));
                    }
                    entry.dma_len = 1;
                    return;
                }
            }
            if *get_slave_link_state() == 1 {
                // todo: Improve this check
                if entry.mac[0] == *(*get_slave_p_mac()).offset(0) &&
                    entry.mac[1] == *(*get_slave_p_mac()).offset(1) &&
                    entry.mac[2] == *(*get_slave_p_mac()).offset(2) &&
                    entry.mac[3] == *(*get_slave_p_mac()).offset(3) {
                    rf_stop_trx();
                    write_reg32(0x800f18, rx_time + *get_T_scan_rsp_intvl() * *get_tick_per_us());
                    write_reg8(0x800f00, 0x85);                        // single TX
                    t_rx_last.store(rx_time, Ordering::Relaxed);

                    (*get_adv_rsp_pri_data()).DeviceAddress = *get_device_address_addr();
                    (*get_pkt_scan_rsp()).data[0] = 0x1e;
                    (*get_pkt_scan_rsp()).data[1] = 0xff;
                    (*get_pkt_scan_rsp()).data[2..2 + size_of::<ll_adv_rsp_private_t>()].copy_from_slice(
                        slice::from_raw_parts(get_adv_rsp_pri_data_addr() as *const u8, size_of::<ll_adv_rsp_private_t>())
                    );
                    (*get_pkt_scan_rsp()).dma_len = 0x27;
                    (*get_pkt_scan_rsp()).rf_len = 0x25;

                    write_reg_dma3_addr(get_pkt_scan_rsp_addr() as u16);
                    write_reg_system_tick_irq(*get_tick_per_us() * 1000 + read_reg_system_tick_irq());

                    entry.dma_len = 1;
                    return;
                }

                if *get_rf_slave_ota_busy() == false || *get_rf_slave_ota_busy_mesh_en() != 0 {
                    t_rx_last.store(rx_time, Ordering::Relaxed);

                    if *get_slave_link_state() - 4 < 2 {
                        rf_link_rc_data(&mut *(packet as *mut mesh_pkt_t));
                    }

                    entry.dma_len = 1;
                    return;
                }
            }
        } else {
            if entry.sno[0] & 0xf == 5 && *get_slave_link_state() == 1 {
                // todo: improve
                if entry.mac[0] == *(*get_slave_p_mac()).offset(0) &&
                    entry.mac[1] == *(*get_slave_p_mac()).offset(1) &&
                    entry.mac[2] == *(*get_slave_p_mac()).offset(2) &&
                    entry.mac[3] == *(*get_slave_p_mac()).offset(3) {
                    t_rx_last.store(rx_time, Ordering::Relaxed);

                    rf_link_slave_connect(&*(packet as *const rf_packet_ll_init_t), rx_time);

                    entry.dma_len = 1;
                    return;
                }

                if *get_rf_slave_ota_busy() == false || *get_rf_slave_ota_busy_mesh_en() != 0 {
                    t_rx_last.store(rx_time, Ordering::Relaxed);

                    if *get_slave_link_state() - 4 < 2 {
                        rf_link_rc_data(&mut *(packet as *mut mesh_pkt_t));
                    }

                    entry.dma_len = 1;
                    return;
                }
            }

            if *get_slave_link_state() != 7 {
                if *get_rf_slave_ota_busy() == false || *get_rf_slave_ota_busy_mesh_en() != 0 {
                    t_rx_last.store(rx_time, Ordering::Relaxed);

                    if *get_slave_link_state() - 4 < 2 {
                        rf_link_rc_data(&mut *(packet as *mut mesh_pkt_t));
                    }

                    entry.dma_len = 1;
                    return;
                }
            }
        }

        t_rx_last.store(rx_time, Ordering::Relaxed);

        set_FtoRX(true);

        let master_sn = ((entry.sno[2] as u16) << 8) | ((entry.sno[0] >> 3) & 1) as u16;
        if *get_light_conn_sn_master() == master_sn {
            rf_link_timing_adjust(rx_time);
        } else {
            set_light_conn_sn_master(master_sn);
            if *get_disconnect_flag() == false {
                set_slave_connected_tick(read_reg_system_tick());
                set_slave_link_connected(true);
                rf_link_slave_data(&*(packet as *const rf_packet_ll_data_t), rx_time);
            }
        }
        if *get_slave_window_size() != 0 {
            if *get_slave_timing_update2_flag() != 0 {
                // todo: What the fuck?
                if 0x40000001 > *get_slave_timing_update2_ok_time() - read_reg_system_tick() {
                    entry.dma_len = 1;
                    return;
                }
                set_slave_timing_update2_flag(0);
            }
            set_slave_window_size(0);
            // todo: Maybe this - needs to be a +
            set_slave_next_connect_tick(*get_slave_link_interval() - *get_tick_per_us() * 1250 + rx_time);
        }
        entry.dma_len = 1;
        return;
    }

    t_rx_last.store(rx_time, Ordering::Relaxed);

    if *get_slave_link_state() == 7 {
        write_reg8(0x80050f, 0);
        rf_stop_trx();
    }

    entry.dma_len = 1;
}

pub fn light_set_tick_per_us(ticks: u32)
{
    set_tick_per_us(ticks);
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

pub fn rf_link_slave_set_buffer(addr: *mut [u32; 9], len: u8)
{
    set_p_slave_status_buffer(addr);
    if addr == null_mut() {
        set_slave_status_buffer_num(0);
    } else {
        set_slave_status_buffer_num(len);
    }
}

pub fn rf_link_set_max_bridge(count: u32)
{
    set_bridge_max_cnt(count);
}

pub fn vendor_id_init(vendor_id: u16)
{
    (*get_pkt_light_report()).value[8] = vendor_id as u8;
    (*get_pkt_light_notify()).value[8] = vendor_id as u8;

    (*get_pkt_light_report()).value[9] = (vendor_id >> 8) as u8;
    (*get_pkt_light_notify()).value[9] = (vendor_id >> 8) as u8;

    set_g_vendor_id(vendor_id);
}

// param st is 2bytes = lumen% + rsv(0xFF)  // rf pkt : device_address+sn+lumen+rsv);
pub fn ll_device_status_update(val_par: &[u8])
{
    (*get_mesh_node_st())[0].val.par.copy_from_slice(val_par);
    if *get_sync_time_enable() != false {
        if *get_synced_flag() == false {
            (*get_mesh_node_st())[0].val.par[0] = (*get_mesh_node_st())[0].val.par[0] | 0x7f;
        } else {
            (*get_mesh_node_st())[0].val.par[0] = (*get_mesh_node_st())[0].val.par[0] & 0x7f;
        }
    }
    (*get_mesh_node_mask())[0] = (*get_mesh_node_mask())[0] | 1;
    (*get_mesh_node_st())[0].tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;
}

pub fn is_receive_ota_window() -> bool
{
    static TICK_LOOP: AtomicU32 = AtomicU32::new(0);

    if *get_SW_Low_Power() == false {
        if *get_loop_interval_us() != 0 {
            if read_reg_system_tick() - TICK_LOOP.load(Ordering::Relaxed) <= *get_loop_interval_us() as u32 * *get_tick_per_us() {
                return true;
            }
            TICK_LOOP.store(read_reg_system_tick(), Ordering::Relaxed);
        }
        if *get_slave_link_state() == 1 && *get_tick_per_us() == 0x10 {
            return true;
        }
    }

    let mut result = false;
    if *get_rf_slave_ota_busy() != false {
        if *get_slave_link_state() == 7 {
            // todo: This may need to be == true?
            result = *get_gateway_en() == false && 6 < *get_slave_link_state();
        }
    }

    return result;
}

pub fn setup_ble_parameter_start(delay: u16, mut interval_min: u16, mut interval_max: u16, timeout: u16) -> u32
{
    let mut invalid = false;

    set_update_ble_par_success_flag(false);
    if interval_max | interval_min == 0 {
        set_update_interval_user_max(*get_slave_link_interval() as u16);
        set_update_interval_user_min(*get_slave_link_interval() as u16);
        interval_max = *get_update_interval_user_max();
        interval_min = *get_update_interval_user_min();
    } else {
        if interval_min < *get_interval_th() as u16 {
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
            set_update_ble_par_success_flag(false);
            set_update_interval_user_max(0);
            set_update_interval_user_min(0);
            set_update_timeout_user(0);
            return 0xfffffffd;
        }
    }
    if invalid == false {
        set_update_interval_flag(delay);
        set_update_ble_par_success_flag(false);
        set_update_interval_time(1);
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
            if *get_update_connect_para_delay_ms() * *get_tick_per_us() * 1000 < read_reg_system_tick() - *get_att_service_discover_tick() {
                update_ble_parameter_cb();
                set_need_update_connect_para(false);
                set_att_service_discover_tick(0);
            }
        }
    }
}

fn set_mesh_info_time_handle()
{
    if *get_set_mesh_info_time() != 0 && !*get_set_mesh_info_expired_flag() && *get_set_mesh_info_time() * *get_tick_per_us() * 1000000 < read_reg_system_tick() {
        set_set_mesh_info_expired_flag(true);
    }
}

fn tx_packet_bridge_poll()
{
    if *get_tx_packet_bridge_random_en() {
        if *get_tx_packet_bridge_tick() != 0 && *get_slave_link_state() == 5 && *get_tick_per_us() * *get_tx_packet_bridge_delay_us() < read_reg_system_tick() - *get_tx_packet_bridge_tick() {
            set_tx_packet_bridge_delay_us(0);
            set_tx_packet_bridge_tick(0);
            tx_packet_bridge();
        }
    }
}

fn mesh_node_flush_status()
{
    static TICK_NODE_REPORT: AtomicU32 = AtomicU32::new(0);

    // todo: wtf?
    if read_reg_system_tick() - TICK_NODE_REPORT.load(Ordering::Relaxed) + *get_tick_per_us() * (-500000 as i32 as u32) < 0x40000001 {
        TICK_NODE_REPORT.store(read_reg_system_tick(), Ordering::Relaxed);
        let tick = read_reg_system_tick();
        if *get_mesh_node_max() > 1 {
            for count in 1..*get_mesh_node_max() as usize {
                let p_node_st = &mut (*get_mesh_node_st())[count];
                if (*p_node_st).tick != 0 && *get_tick_per_us() * *get_online_status_timeout() * 1000 >> 0x10 < (tick >> 0x10 | 1) - (*p_node_st).tick as u32 {
                    (*p_node_st).tick = 0;

                    (*get_mesh_node_mask())[count >> 5] = 1 << (count & 0x1f) | (*get_mesh_node_mask())[count >> 5];
                    if *get_p_mesh_node_status_callback() != None {
                        let mut node_data = (*p_node_st).val.clone();
                        node_data.sn = 0;
                        (*get_p_mesh_node_status_callback()).unwrap()(&node_data, 0);
                    }
                }
            }
        }
    }
    return;
}

fn mesh_node_keep_alive()
{
    set_device_node_sn(*get_device_node_sn() + 1);
    if *get_device_node_sn() == 0 {
        set_device_node_sn(1);
    }

    (*get_mesh_node_st())[0].val.sn = *get_device_node_sn();
    let tick = read_reg_system_tick();
    (*get_mesh_node_st())[0].tick = (tick >> 16) as u16 | 1;
}

fn mesh_node_adv_status(p_data: &mut [u8]) -> u32
{
    static SEND_ONLINE_STATUS_CNT: AtomicU8 = AtomicU8::new(0xC8);

    let mut uVar2;

    p_data.fill(0);
    let mut elems = p_data.len() / *get_mesh_node_st_val_len() as usize;
    if (*get_mesh_node_max() as usize) < p_data.len() / *get_mesh_node_st_val_len() as usize {
        elems = *get_mesh_node_max() as usize;
    }

    uVar2 = elems;

    SEND_ONLINE_STATUS_CNT.store(SEND_ONLINE_STATUS_CNT.load(Ordering::Relaxed) + 1, Ordering::Relaxed);

    let mut uVar4 = 0;
    if *get_send_self_online_status_cycle() <= SEND_ONLINE_STATUS_CNT.load(Ordering::Relaxed) || *get_mesh_node_max() < 0x14 {
        SEND_ONLINE_STATUS_CNT.store(0, Ordering::Relaxed);
        p_data.copy_from_slice(
            unsafe {
                slice::from_raw_parts(
                    addr_of!((*get_mesh_node_st())[0].val.dev_adr),
                    *get_mesh_node_st_val_len() as usize,
                )
            }
        );
        mesh_node_keep_alive();
        uVar2 = *get_mesh_node_max() as usize;
        uVar4 = 1;
    }
    let mut uVar5 = uVar4;
    if uVar4 < uVar2 {
        loop {
            let mut uVar3 = *get_mesh_node_cur() as usize;
            if uVar3 < uVar2 && (*get_mesh_node_st())[uVar3].tick != 0 {
                let iVar1 = *get_mesh_node_st_val_len() as usize * uVar5;
                uVar5 = uVar5 + 1;
                p_data[iVar1..iVar1 + *get_mesh_node_st_val_len() as usize].copy_from_slice(
                    unsafe {
                        slice::from_raw_parts(
                            addr_of!((*get_mesh_node_st())[uVar3].val) as *const u8,
                            *get_mesh_node_st_val_len() as usize,
                        )
                    }
                );

                uVar3 = *get_mesh_node_cur() as usize;
            }
            set_mesh_node_cur(uVar3 as u8 + 1);
            uVar2 = *get_mesh_node_max() as usize;
            if uVar2 <= uVar3 + 1 & 0xff {
                set_mesh_node_cur(1);
            }
            uVar4 += 1;

            if uVar5 == elems || uVar4 >= uVar2 {
                break;
            }
        }
    }
    return uVar5 as u32;
}

fn blt_stall_mcu(ticks: u32)
{
    critical_section::with(|_| {
        write_reg_tmr1_tick(0);
        write_reg_tmr1_capt(ticks);
        write_reg_tmr_ctrl8(read_reg_tmr_ctrl8() | 8);
        write_reg_mcu_wakeup_mask(read_reg_mcu_wakeup_mask() | 2);
        write_reg_tmr_sta(2);
        write_reg_pwdn_ctrl(0x80);
        write_reg_tmr_sta(2);
        write_reg_tmr_ctrl8(read_reg_tmr_ctrl8() & 0xf7);
    });
}

pub fn mesh_send_command(packet: *const rf_packet_att_cmd_t, channel_index: u32, retransmit_count: u32)
{
    rf_set_tx_rx_off();
    rf_set_ble_access_code(*get_pair_ac());
    rf_set_ble_crc_adv();

    if retransmit_count != 0xffffffff {
        for transmit_count in 0..retransmit_count {
            if *get_mesh_chn_amount() != 0 {
                let mut chn_idx = channel_index;
                let mut chn_cnt = 0;
                if (channel_index == 0xff) {
                    chn_idx = 0;
                }
                loop {
                    rf_set_ble_channel((*get_sys_chn_listen())[chn_idx as usize & 3]);
                    rf_start_srx2tx(packet as u32, read_reg_system_tick() + *get_tick_per_us() * 0x1e);
                    if *get_switch_rf_tx_once_time() == 0 {
                        sleep_us(600);
                    } else {
                        blt_stall_mcu(*get_switch_rf_tx_once_time() * *get_tick_per_us());
                    }
                    if channel_index != 0xff {
                        if transmit_count < retransmit_count {
                            sleep_us(200);
                        }
                        break;
                    }
                    chn_idx = chn_cnt + 1;
                    chn_cnt = chn_idx;
                    if chn_idx >= *get_mesh_chn_amount() as u32 {
                        break;
                    }
                }
            }
        }
    }
}


fn mesh_send_online_status()
{
    static ADV_ST_SN: AtomicU32 = AtomicU32::new(0);

    if *get_mesh_send_online_status_flag() {
        mesh_node_flush_status();

        mesh_node_adv_status(&mut (*get_pkt_light_adv_status()).data[1..1 + 26]);

        ADV_ST_SN.store(ADV_ST_SN.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
        unsafe {
            let val = ADV_ST_SN.load(Ordering::Relaxed);
            slice::from_raw_parts_mut((addr_of_mut!((*get_pkt_light_adv_status()).advA) as *mut u8).offset(4), 3).copy_from_slice(slice::from_raw_parts(addr_of!(val) as *const u8, 3))
        }

        if *get_mesh_node_st_val_len() == 4 {
            (*get_pkt_light_adv_status()).data[26] = 0xa5;
            (*get_pkt_light_adv_status()).data[25] = 0xa5;
        }
        (*get_pkt_light_adv_status()).data[28] = 0xa5;
        (*get_pkt_light_adv_status()).data[27] = 0xa5;

        let mut rStack_40: rf_packet_att_cmd_t = rf_packet_att_cmd_t {
            dma_len: 0,
            _type: 0,
            rf_len: 0,
            l2capLen: 0,
            chanId: 0,
            opcode: 0,
            handle: 0,
            handle1: 0,
            value: [0; 30],
        };

        unsafe {
            slice::from_raw_parts_mut(addr_of_mut!(rStack_40) as *mut u8, size_of::<rf_packet_adv_ind_module_t>()).copy_from_slice(
                slice::from_raw_parts(get_pkt_light_adv_status_addr() as *const u8, size_of::<rf_packet_adv_ind_module_t>())
            )
        }

        if *get_security_enable() {
            rStack_40._type = rStack_40._type | 0x7f;
            pair_enc_packet_mesh(addr_of_mut!(rStack_40) as *mut mesh_pkt_t);
        }

        mesh_send_command(&rStack_40, 0xff, 0);
    }
}

fn back_to_rxmode_bridge()
{
    if !*get_SW_Low_Power() {
        rf_set_tx_rx_off();
        rf_set_ble_access_code(*get_pair_ac());
        rf_set_ble_crc_adv();
        rf_set_ble_channel((*get_sys_chn_listen())[*get_st_brige_no() as usize >> 1]);
        rf_set_rxmode();
    }
}

fn rf_link_proc_ttc(cmd_time: u32, unknown: u32, src: *mut u8)
{
    unsafe {
        if *src.offset(-0xd) & 0x3f != 6 {
            let mut uVar2 = (read_reg_system_tick() - cmd_time) / *get_tick_per_us();
            let mut uVar4 = (unknown << 0x18) >> 0x1e;
            let mut uVar3 = uVar2 >> 10;
            if uVar4 != 0 {
                uVar3 = uVar2 >> 0xc;
                if uVar4 != 1 {
                    uVar3 = uVar2 >> 0xe;
                    if uVar4 != 2 {
                        uVar3 = 1;
                        if uVar4 == 3 {
                            uVar3 = uVar2 >> 0x12;
                        }
                    }
                }
            }

            uVar3 = uVar3 + (unknown & 0x3f);
            while 0x3e < uVar3 {
                if uVar4 == 0 || uVar4 == 1 {
                    uVar3 = uVar3 >> 2;
                } else {
                    if uVar4 != 2 {
                        if uVar4 != 3 {
                            if uVar4 < 3 {
                                uVar4 = uVar4 + 1 & 0xff;
                            }
                            *src = (uVar4 << 6 | 1) as u8;
                        }
                        uVar3 = 0x3f;
                        *src = ((uVar3 & 0x3f) | (uVar4 << 6)) as u8;
                    }
                    uVar3 = uVar3 >> 4;
                }
                uVar4 = uVar4 + 1 & 0xff;
            }
            if uVar3 == 0 {
                *src = (uVar4 << 6 | 1) as u8;
            } else {
                *src = ((uVar3 & 0x3f) | (uVar4 << 6)) as u8;
            }
        }
    }
}

fn rf_link_is_notify_req(value: u8) -> bool
{
    if !*get_rf_slave_ota_busy() {
        if value == 0x1d || value == 0x1a || value == 0x20 || value == 0x17 || value == 0x26 || value == 0 || value == 0x2a || value == 0x28 {
            return true;
        } else {
            // todo: Maybe this is 6, or a ==
            return value != 7;
        }
    }

    return false;
}

fn rf_start_stx_mesh(packet: &rf_packet_att_cmd_t, sched_tick: u32)
{
    get_pkt_mesh().clone_from(unsafe { transmute(packet) });

    (*get_pkt_mesh()).src_tx = *get_device_address();
    (*get_pkt_mesh()).handle1 = 0;

    if (packet._type as i8) < 0 {
        pair_enc_packet_mesh(get_pkt_mesh());
    }

    rf_start_srx2tx(get_pkt_mesh_addr() as u32, read_reg_system_tick());
}

fn app_bridge_cmd_handle(bridge_cmd_time: u32)
{
    if *get_slave_data_valid() != 0 {
        set_slave_data_valid(*get_slave_data_valid() - 1);
        if *get_slave_data_valid() == 0 {
            set_slave_sno_sending(0);
        } else if *get_slave_read_status_busy() == 0 || *get_slave_data_valid() as i32 > -1 {
            for chn in 0..4 {
                rf_set_ble_channel((*get_sys_chn_listen())[chn]);
                (*get_pkt_light_data())._type |= 0x7f;
                rf_link_proc_ttc(*get_app_cmd_time(), 0, addr_of_mut!((*get_pkt_light_data()).value[0x14]));
                if rf_link_is_notify_req((*get_pkt_light_data()).value[7] & 0x3f) {
                    let mut uVar3 = (((read_reg_system_tick() - bridge_cmd_time) / *get_tick_per_us()) + 500) >> 10;
                    if uVar3 > 0xff {
                        uVar3 = 0xff;
                    }
                    (*get_pkt_light_data()).value[24] = uVar3 as u8;
                }
                rf_start_stx_mesh(&*get_pkt_light_data(), *get_tick_per_us() * 0x1e + read_reg_system_tick());
                sleep_us(700);
            }
        }
    }
}

fn mesh_user_command_pkt_enc2buf()
{
    if !*get_security_enable() || (*get_sw_no_pair() && (*get_pkt_user_cmd()).dma_len == 0x26) {
        get_pkt_mesh_user_cmd_buf().clone_from(unsafe { transmute(&*get_pkt_user_cmd()) });
    } else {
        (*get_pkt_user_cmd())._type |= 0x7f;
        get_pkt_mesh_user_cmd_buf().clone_from(unsafe { transmute(&*get_pkt_user_cmd()) });
        pair_enc_packet_mesh(get_pkt_mesh_user_cmd_buf_addr());
    }
}

fn mesh_send_user_command() -> u8
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

        if *get_SW_Low_Power() != false || ((((*get_pkt_user_cmd()).handle1 as u32) << 0x1e) as i32) < 0 {
            if *get_SW_Low_Power() != false {
                user_cmd = false;
            }

            if ((((*get_pkt_user_cmd()).handle1 as u32) << 0x1e) as i32) < 0 {
                user_cmd = true;
            }

            mesh_user_command_pkt_enc2buf();
        }
        let mut uVar9 = 0;
        let mut iVar8 = 0;
        let mut breakit = false;
        loop {
            loop {
                if !user_cmd {
                    rf_link_proc_ttc(*get_slave_tx_cmd_time(), 0, addr_of_mut!((*get_pkt_user_cmd()).par[9]));
                    mesh_user_command_pkt_enc2buf();
                }
                rf_set_ble_channel((*get_sys_chn_listen())[uVar9 & 3]);
                rf_start_srx2tx(get_pkt_mesh_user_cmd_buf_addr() as u32, *get_tick_per_us() * 0x1e + read_reg_system_tick());
                if *get_switch_rf_tx_once_time() != 0 {
                    break;
                }

                uVar9 = uVar9 + 1;
                iVar8 = ((*get_lpn_retransmit_cnt()) + 1) * 4;
                if iVar8 as usize - uVar9 == 0 || (iVar8 as usize) < uVar9 {
                    breakit = true;
                    break;
                }
            }

            if breakit {
                break;
            }

            blt_stall_mcu(*get_tick_per_us() * *get_switch_rf_tx_once_time());
            uVar9 = uVar9 + 1;
            iVar8 = ((*get_lpn_retransmit_cnt()) + 1) * 4;

            if iVar8 as usize - uVar9 == 0 && uVar9 > iVar8 as usize {
                break;
            }
        }

        if *get_mesh_user_cmd_idx() == 0 {
            set_lpn_retransmit_cnt(0);
        }
    });
    *get_mesh_user_cmd_idx()
}

fn mesh_send_beacon(beacon_pkt: *const rf_packet_adv_ind_module_t)
{
    rf_set_tx_rx_off();
    rf_set_ble_access_code_adv();
    rf_set_ble_crc_adv();

    for chn in *get_sys_chn_adv() {
        rf_set_ble_channel(chn);
        rf_start_beacon(beacon_pkt as u32, *get_tick_per_us() * 0x1e + read_reg_system_tick());

        sleep_us(600);
    }
}

fn tx_packet_bridge()
{
    static TICK_BRIDGE_REPORT: AtomicU32 = AtomicU32::new(0);
    static LAST_ALARM_TIME_BRIDGE: AtomicU32 = AtomicU32::new(0);
    static LAST_CONN_IBEACON_TIME: AtomicU32 = AtomicU32::new(0);

    critical_section::with(|_| {
        if !*get_SW_Low_Power() {
            rf_set_tx_rx_off();

            sleep_us(100);

            rf_set_ble_access_code(*get_pair_ac());
            rf_set_ble_crc_adv();

            let tick = read_reg_system_tick();
            if *get_slave_listen_interval() * *get_online_status_interval2listen_interval() as u32 - *get_online_status_comp() as u32 * *get_tick_per_us() * 1000 < tick - TICK_BRIDGE_REPORT.load(Ordering::Relaxed) {
                TICK_BRIDGE_REPORT.store(tick, Ordering::Relaxed);
                if *get_tick_per_us() * 30000000 < tick - LAST_ALARM_TIME_BRIDGE.load(Ordering::Relaxed) {
                    LAST_ALARM_TIME_BRIDGE.store(tick, Ordering::Relaxed);
                    // noop
                } else if *get_iBeaconInterval() == 0 || *get_beacon_with_mesh_adv() == 0 || tick - LAST_CONN_IBEACON_TIME.load(Ordering::Relaxed) <= *get_tick_per_us() * *get_iBeaconInterval() as u32 * 100000 {
                    mesh_send_online_status();
                } else {
                    mesh_send_beacon(get_pkt_ibeacon_addr());
                    LAST_CONN_IBEACON_TIME.store(tick, Ordering::Relaxed);
                }
            }
            app_bridge_cmd_handle(*get_t_bridge_cmd());
        }
        if *get_slave_data_valid() == 0 && !*get_sw_flag() {
            mesh_send_user_command();
        }
        if *get_tx_packet_bridge_random_en() {
            back_to_rxmode_bridge();
        }
    });
}

pub fn rf_link_slave_proc() {
    tx_packet_bridge_poll();
    set_mesh_info_time_handle();
    app().mesh_manager.mesh_pair_proc();
    update_connect_para();
}

pub fn is_add_packet_buf_ready() -> bool
{
    if *get_gateway_en() {
        return 1 - ((*get_gateway_tx_wptr() + 1) & 7) == *get_gateway_tx_rptr();
    } else {
        return (read_reg_dma_tx_wptr() - read_reg_dma_tx_rptr() & 7) < 3;
    }
}

pub fn rf_link_add_tx_packet(packet: *const rf_packet_att_cmd_t) -> bool
{
    if *get_gateway_en() {
        let wptr = *get_gateway_tx_wptr();
        let next_wptr = (*get_gateway_tx_wptr() + 1) & 7;
        if next_wptr != *get_gateway_tx_rptr() {
            set_gateway_tx_wptr(next_wptr);

            (*get_blt_tx_fifo())[(wptr as usize & 7) * 0x28..(wptr as usize & 7) * 0x28 + 0x28].copy_from_slice(
                unsafe {
                    slice::from_raw_parts(
                        packet as *const u8,
                        0x28,
                    )
                }
            );

            if *get_gateway_security() != false {
                pair_enc_packet(addr_of_mut!((*get_blt_tx_fifo())[wptr as usize & 7]) as *mut rf_packet_ll_app_t);
            }
            return true;
        }

        return false;
    } else {
        let wptr = read_reg_dma_tx_wptr();
        let rptr = read_reg_dma_tx_rptr();
        let widx = (wptr - rptr) & 7;

        if widx < 4 {
            if widx == 0 {
                write_reg_dma_tx_fifo(get_pkt_empty_addr() as u16);
            }

            let mut dest = &mut (*get_blt_tx_fifo())[(*get_blt_tx_wptr() as usize & 7) * 0x28..(*get_blt_tx_wptr() as usize & 7) * 0x28 + 0x28];
            dest.copy_from_slice(
                unsafe {
                    slice::from_raw_parts(
                        packet as *const u8,
                        0x28,
                    )
                }
            );

            set_blt_tx_wptr(*get_blt_tx_wptr() + 1);
            if *get_mesh_notify_enc_enable() != 0 {
                pair_enc_packet(unsafe { transmute(dest.as_mut_ptr()) });
            }
            write_reg_dma_tx_fifo(dest.as_mut_ptr() as u32 as u16);
            return true;
        }
        return false;
    }
}

fn rf_link_slave_read_status_par_init()
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
    let addr = *get_cur_ota_flash_addr() + *get_flash_adr_light_new_fw();
    flash_write_page(addr, 0x10, data);

    let mut tmp = [0u8; 0x10];
    flash_read_page(addr, 0x10, tmp.as_mut_ptr());

    if unsafe { slice::from_raw_parts(data, 0x10) } == tmp {
        set_cur_ota_flash_addr(*get_cur_ota_flash_addr() + 0x10);
        return OtaState::CONTINUE;
    } else {
        return OtaState::ERROR;
    }
}

pub fn is_master_sending_ota_st() -> bool
{
    return (*get_mesh_ota_master_st())[20] > 1;
}

pub fn register_mesh_ota_master_ui(cb: fn(*const u8))
{
    set_mesh_ota_master_ui_sending(Some(cb));
}

pub fn rf_link_match_group_mac(sno: *const app_cmd_value_t) -> (bool, bool)
{
    let mut group_match = false;
    let mut device_match = false;

    unsafe {
        if (*sno).dst & !*get_device_address_mask() != 0 {
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
            let addr = (*sno).dst & *get_device_address_mask();
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
                        if (*get_mesh_ota_master_st())[20] == 0x8 {
                            set_mesh_user_cmd_idx(8);
                        }
                    }
                } else {
                    set_mesh_user_cmd_idx(*get_bridge_max_cnt() as u8);
                }
                if *get_sw_no_pair() && *get_sw_flag() {
                    (*get_pkt_user_cmd()).dma_len = 0x26;
                    (*get_pkt_user_cmd()).l2capLen = 0x20;
                    (*get_pkt_user_cmd()).rf_len = 0x24;
                } else {
                    (*get_pkt_user_cmd()).dma_len = 0x27;
                    (*get_pkt_user_cmd()).l2capLen = 0x21;
                    (*get_pkt_user_cmd()).rf_len = 0x25;
                }
                (*get_pkt_user_cmd())._type = 2;
                (*get_pkt_user_cmd()).chanId = 0xff03;
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

                (*get_pkt_user_cmd()).internal_par1[2] = max_relay;

                let (group_match, device_match) = rf_link_match_group_mac(addr_of_mut!((*get_pkt_user_cmd()).sno) as *const app_cmd_value_t);
                set_slave_tx_cmd_time(read_reg_system_tick());
                if group_match || device_match {
                    light_slave_tx_command_callback(addr_of!((*get_pkt_user_cmd()).l2capLen) as *const ll_packet_l2cap_data_t);
                    if device_match && *get_mesh_node_ignore_addr() == false {
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
            if *get_mesh_node_max() & 0x1f != 0 {
                (*get_mesh_node_mask())[*get_mesh_node_max() as usize >> 5] = (1 << (*get_mesh_node_max() & 0x1f)) - 1;
            }
        }
    }

    set_mesh_node_report_enable(enable);
}

pub fn mesh_report_status_enable_mask(data: *const u8, len: u16)
{
    set_mesh_node_report_enable(if unsafe { *data } != 0 { true } else { false });
    if *get_mesh_node_report_enable() && len > 1 {
        for index in 1..len {
            if (*get_mesh_node_max() != 0) {
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

pub fn rf_link_get_op_para(packet: &ll_packet_l2cap_data_t, p_op: &mut [u8], p_op_len: &mut u8, p_para: &mut [u8], p_para_len: &mut u8, mesh_flag: bool) -> bool
{
    unsafe {
        if ((packet.value[7] as u32 * 0x1000000) as i32) < 0 {
            if packet.value[7] >> 6 == 3 {
                *p_op_len = 3;
            } else {
                *p_op_len = 2;
            }
        } else {
            *p_op_len = 1;
        }
        p_op[0..*p_op_len as usize].copy_from_slice(&packet.value[7..7 + *p_op_len as usize]);

        let mut pkt_len = packet.l2capLen - 10;
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
            p_para[0..pkt_len as usize].copy_from_slice(&packet.value[7 + *p_op_len as usize..7 + *p_op_len as usize + pkt_len as usize]);
        } else {
            *p_para_len = 0;
        }

        return pkt_len <= max_param_len;
    }
}


