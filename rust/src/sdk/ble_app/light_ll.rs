use core::mem::size_of;
use core::ptr::{addr_of, null, null_mut, slice_from_raw_parts};
use core::sync::atomic::{AtomicU32, Ordering};

use crate::{app, blinken, no_mangle_fn, pub_mut};
use crate::common::{get_conn_update_cnt, get_conn_update_successed, rf_update_conn_para, set_conn_update_cnt, update_ble_parameter_cb};
use crate::main_light::{irq_timer0, irq_timer1};
use crate::mesh::wrappers::{get_mesh_node_mask, get_mesh_node_st};
use crate::sdk::ble_app::ble_ll_attribute::{get_att_service_discover_tick, get_slave_link_time_out, set_att_service_discover_tick};
use crate::sdk::ble_app::rf_drv_8266::{get_slave_link_state, rf_stop_trx, set_slave_adv_enable, set_slave_connection_enable};
use crate::sdk::light::{_mesh_ota_timeout_handle, _rf_link_add_tx_packet, _rf_link_slave_connect, _rf_link_slave_data, get_device_address_addr, get_gateway_en, get_interval_th, get_loop_interval_us, get_need_update_connect_para, get_rf_slave_ota_busy, get_rf_slave_ota_busy_mesh_en, get_slave_p_mac, get_SW_Low_Power, get_sync_time_enable, get_synced_flag, get_tick_per_us, get_update_connect_para_delay_ms, get_update_interval_user_max, get_update_interval_user_min, ll_adv_rsp_private_t, rf_packet_att_cmd_t, rf_packet_ll_data_t, rf_packet_ll_init_t, rf_packet_scan_rsp_t, set_mesh_ota_master_ui_sending, set_need_update_connect_para, set_slave_link_connected, set_tick_per_us, set_update_ble_par_success_flag, set_update_interval_flag, set_update_interval_time, set_update_interval_user_max, set_update_interval_user_min, set_update_timeout_user};
use crate::sdk::mcu::clock::clock_time;
use crate::sdk::mcu::register::{FLD_IRQ, FLD_RF_IRQ_MASK, read_reg_irq_src, read_reg_rf_irq_status, read_reg_rf_rx_status, read_reg_system_tick, read_reg_system_tick_irq, write_reg32, write_reg8, write_reg_dma2_addr, write_reg_dma3_addr, write_reg_irq_src, write_reg_rf_irq_status, write_reg_system_tick_irq};
use crate::vendor_light::{get_adv_rsp_pri_data, get_adv_rsp_pri_data_addr};

no_mangle_fn!(rf_link_timing_adjust, time: u32);
pub_mut!(slave_timing_update, u32);
pub_mut!(slave_instant_next, u16);
pub_mut!(slave_chn_map, [u8; 5]);
pub_mut!(slave_interval_old, u32);
pub_mut!(slave_link_interval, u32);
pub_mut!(slave_window_size_update, u32);
pub_mut!(ble_conn_timeout, u32);
pub_mut!(ble_conn_interval, u32);
pub_mut!(ble_conn_offset, u32);
pub_mut!(slave_ll_rsp, u32);
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

no_mangle_fn!(l2cap_att_handler, u32, packet: *const rf_packet_ll_data_t);

// #[no_mangle]
// pub unsafe fn rf_link_slave_data(packet: *const rf_packet_ll_data_t, time: u32) -> bool {
//     let rf_len: u8 = (*packet).rf_len;
//     let chanid: u16 = (*packet).chanid;
//
//     if ((*packet)._type as i32 * 0x1000000) as i32 >= -1 {
//         if ((*packet)._type & 3) == 2 {
//             if 6 < chanid {
//                 return false;
//             }
//         }
//
//         if chanid == 5 {
//             rf_update_conn_para(packet as *const u8);
//         }
//
//         _rf_link_timing_adjust(time);
//         if rf_len < 6 {
//             if rf_len == 0 {
//                 return false;
//             }
//         } else {
//             if (*packet)._type & 3 == 3 && (*packet).l2cap_low == 1 {
//                 set_slave_timing_update(1);
//                 set_slave_instant_next((((*packet).sno as u16) << 8) | (*packet).hh as u16);
//                 for i in 0..5 {
//                     (*(get_slave_chn_map_addr() as *mut [u8; 5]))[i] = *(packet as *const u8).offset(7 + i as isize);
//                 }
//                 return true;
//             }
//
//             if rf_len == 0xc && (*packet)._type & 3 == 3 && (*packet).l2cap_low == 0 {
//                 set_slave_interval_old(*get_slave_link_interval());
//                 set_slave_instant_next((*packet).group);
//                 set_slave_window_size_update(((*packet).l2cap_high as u32 * 1250 + 1300) * *get_tick_per_us());
//                 set_slave_timing_update(2);
//                 set_ble_conn_timeout((*packet).nid as u32 * 10000);
//                 set_ble_conn_interval(*get_tick_per_us() * 1250 * (*packet).att as u32);
//                 set_ble_conn_offset((*packet).chanid as u32 * *get_tick_per_us() * 1250);
//                 return false;
//             }
//         }
//         set_slave_ll_rsp(_l2cap_att_handler(packet));
//         let addSuccess = _rf_link_add_tx_packet(*get_slave_ll_rsp() as *const u8);
//         if *get_slave_ll_rsp() != 0x0 && !addSuccess {
//             add_tx_packet_rsp_failed += 1;
//         }
//         return false;
//     }
//
//     return false;
// }

no_mangle_fn!(irq_light_slave_tx);
no_mangle_fn!(irq_light_slave_rx);
no_mangle_fn!(sleep_start);

pub_mut!(p_st_handler, Option<fn()>);

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
// pub_mut!(light_rcv_rssi, u8);
// pub_mut!(t_rx_last, u32, 0);
// pub_mut!(rcv_pkt_time, u32);
// pub_mut!(slave_link_state, u32);
// pub_mut!(T_scan_rsp_intvl, u32);
// pub_mut!(pkt_scan_rsp, rf_packet_scan_rsp_t);
// pub_mut!(FtoRX, bool);
// pub_mut!(light_conn_sn_master, u16);
// pub_mut!(disconnect_flag, bool);
// pub_mut!(slave_connected_tick, u32);
// pub_mut!(slave_window_size, u32);
// pub_mut!(slave_timing_update2_flag, u32);
// pub_mut!(slave_next_connect_tick, u32);
// pub_mut!(slave_timing_update2_ok_time, u32);
//
// no_mangle_fn!(rf_start_stx2rx, addr: u32, tick: u32);
// no_mangle_fn!(rf_link_rc_data, packet: *const u8, tick: u32);
//
// static pkt_empty: [u8; 6] = [02, 00, 00, 00, 01, 00];
//
// #[link_section = ".ram_code"]
// #[inline(never)]
// unsafe fn irq_light_slave_rx()
// {
//     light_rx_wptr = (light_rx_wptr + 1) & 3;
//     if read_reg_rf_rx_status() == 0x0b {
//         write_reg_rf_irq_status(1);
//         return;
//     }
//
//     let rx_index = (light_rx_wptr * 0x40) as usize;
//     write_reg_dma2_addr(light_rx_buff.as_ptr().offset(rx_index as isize * 0x40) as u16);
//     write_reg_rf_irq_status(1);
//     light_rcv_rssi = light_rx_buff[rx_index + 4];
//     let time = *(light_rx_buff.as_ptr().offset(rx_index as isize + 8) as *const u32);
//     let dma_len = light_rx_buff[rx_index];
//     light_rx_wptr = rx_index as u32;
//     if dma_len == 1 {
//         if t_rx_last == time {
//             rf_stop_trx();
//             rf_start_stx2rx(addr_of!(pkt_empty) as u32, *get_tick_per_us() * 10 + clock_time());
//             return;
//         }
//     } else if dma_len >= 0xf && dma_len == (light_rx_buff[rx_index + 0xd] & 0x3f) + 0x11 && light_rx_buff[rx_index + dma_len as usize + 3] & 0x51 == 0x40 {
//         let packet = light_rx_buff.as_ptr().offset(rx_index as isize + 8);
//         rcv_pkt_time = time;
//         if (light_rx_buff[rx_index + 0xc] & 0xf) == 3 {
//             if slave_link_state != 1 {
//                 if slave_link_state != 7 {
//                     if *get_rf_slave_ota_busy() == false || *get_rf_slave_ota_busy_mesh_en() != 0 {
//                         t_rx_last = time;
//                         if slave_link_state - 4 < 2 {
//                             rf_link_rc_data(packet, time);
//                         }
//                         light_rx_buff[rx_index] = 1;
//                         return;
//                     }
//                 }
//             } else {
//                 if light_rx_buff[rx_index + 0x14] == *(*get_slave_p_mac()).offset(0) &&
//                     light_rx_buff[rx_index + 0x15] == *(*get_slave_p_mac()).offset(1) &&
//                     light_rx_buff[rx_index + 0x16] == *(*get_slave_p_mac()).offset(2) &&
//                     light_rx_buff[rx_index + 0x17] == *(*get_slave_p_mac()).offset(3) {
//                     write_reg32(0x800f18, time + T_scan_rsp_intvl * *get_tick_per_us());
//                     write_reg8(0x800f00, 0x85);                        // single TX
//                     t_rx_last = time;
//
//                     get_adv_rsp_pri_data().DeviceAddress = *get_device_address_addr();
//                     get_adv_rsp_pri_data().rsv[0] = 0x1e;
//                     get_adv_rsp_pri_data().rsv[1] = 0xff;
//                     pkt_scan_rsp.data[2..].copy_from_slice(&*slice_from_raw_parts(get_adv_rsp_pri_data_addr() as *const u8, size_of::<ll_adv_rsp_private_t>()));
//                     pkt_scan_rsp.dma_len = 0x27;
//                     pkt_scan_rsp.rf_len = 0x25;
//                     write_reg_dma3_addr(addr_of!(pkt_scan_rsp) as u16);
//                     write_reg_system_tick_irq(*get_tick_per_us() * 1000 + read_reg_system_tick_irq());
//                     light_rx_buff[rx_index] = 1;
//                     return;
//                 }
//                 if *get_rf_slave_ota_busy() == false || *get_rf_slave_ota_busy_mesh_en() != 0 {
//                     t_rx_last = time;
//                     if slave_link_state - 4 < 2 {
//                         rf_link_rc_data(packet, time);
//                     }
//                     light_rx_buff[rx_index] = 1;
//                     return;
//                 }
//             }
//         } else {
//             if (light_rx_buff[rx_index + 0xc] & 0xf) == 5 && slave_link_state == 1 {
//                 if light_rx_buff[rx_index + 0x14] == *(*get_slave_p_mac()).offset(0) &&
//                     light_rx_buff[rx_index + 0x15] == *(*get_slave_p_mac()).offset(1) &&
//                     light_rx_buff[rx_index + 0x16] == *(*get_slave_p_mac()).offset(2) &&
//                     light_rx_buff[rx_index + 0x17] == *(*get_slave_p_mac()).offset(3) {
//                     t_rx_last = time;
//                     _rf_link_slave_connect(packet as *const rf_packet_ll_init_t, time);
//                     light_rx_buff[rx_index] = 1;
//                     return;
//                 }
//                 if *get_rf_slave_ota_busy() == false || *get_rf_slave_ota_busy_mesh_en() != 0 {
//                     t_rx_last = time;
//                     if slave_link_state - 4 < 2 {
//                         rf_link_rc_data(packet, time);
//                     }
//                     light_rx_buff[rx_index] = 1;
//                     return;
//                 }
//             }
//             if (slave_link_state != 7) {
//                 if *get_rf_slave_ota_busy() == false || *get_rf_slave_ota_busy_mesh_en() != 0 {
//                     t_rx_last = time;
//                     if slave_link_state - 4 < 2 {
//                         rf_link_rc_data(packet, time);
//                     }
//                     light_rx_buff[rx_index] = 1;
//                     return;
//                 }
//             }
//         }
//         t_rx_last = time;
//         FtoRX = true;
//         let master_sn = (light_rx_buff[rx_index + 0xe] as u16) << 8 | (light_rx_buff[rx_index + 0xc] >> 3 & 1) as u16;
//         if light_conn_sn_master == master_sn {
//             rf_link_timing_adjust(time);
//         } else {
//             light_conn_sn_master = master_sn;
//             if disconnect_flag == false {
//                 slave_connected_tick = clock_time();
//                 set_slave_link_connected(true);
//                 _rf_link_slave_data(packet as *const rf_packet_ll_data_t, time);
//             }
//         }
//         if (slave_window_size != 0) {
//             if (slave_timing_update2_flag != 0) {
//                 if slave_timing_update2_ok_time - clock_time() < 0x40000001 {
//                     light_rx_buff[rx_index] = 1;
//                     return;
//                 }
//                 slave_timing_update2_flag = 0;
//             }
//             slave_window_size = 0;
//             slave_next_connect_tick = (slave_link_interval as i32 + *get_tick_per_us() as i32 * -0x4e2 + time as i32) as u32;
//         }
//         light_rx_buff[rx_index] = 1;
//         return;
//     }
//     t_rx_last = time;
//     if slave_link_state == 7 {
//         write_reg8(0x80050f, 0);
//         rf_stop_trx();
//     }
// }
//
// // irq_light_slave_rx

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

pub fn register_mesh_ota_master_ui(cb: fn(*const u8))
{
    set_mesh_ota_master_ui_sending(Some(cb));
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

pub fn update_connect_para()
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

no_mangle_fn!(tx_packet_bridge_poll);
no_mangle_fn!(set_mesh_info_time_handle);
no_mangle_fn!(mesh_ota_slave_proc_poll);
no_mangle_fn!(mesh_ota_timeout_poll);

pub fn rf_link_slave_proc() {
    _tx_packet_bridge_poll();
    _set_mesh_info_time_handle();
    _mesh_ota_slave_proc_poll();
    _mesh_ota_timeout_poll();
    app().mesh_manager.mesh_pair_proc();
    update_connect_para();
}