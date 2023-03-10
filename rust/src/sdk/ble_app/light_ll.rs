use std::mem::size_of;
use std::ops::Deref;
use std::ptr::{addr_of, slice_from_raw_parts};
use crate::{no_mangle_fn, pub_mut};
use crate::common::rf_update_conn_para;
use crate::main_light::{irq_timer0, irq_timer1};
use crate::sdk::ble_app::rf_drv_8266::rf_stop_trx;
use crate::sdk::light::{_rf_link_add_tx_packet, _rf_link_slave_connect, _rf_link_slave_data, get_device_address_addr, get_rf_slave_ota_busy, get_rf_slave_ota_busy_mesh_en, get_slave_p_mac, get_tick_per_us, ll_adv_rsp_private_t, rf_packet_ll_data_t, rf_packet_ll_init_t, rf_packet_scan_rsp_t, set_slave_link_connected};
use crate::sdk::mcu::clock::clock_time;
use crate::sdk::mcu::register::{FLD_IRQ, FLD_RF_IRQ_MASK, read_reg_irq_src, read_reg_rf_irq_status, read_reg_rf_rx_status, read_reg_system_tick_irq, write_reg32, write_reg8, write_reg_dma2_addr, write_reg_dma3_addr, write_reg_irq_src, write_reg_rf_irq_status, write_reg_system_tick_irq};
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

