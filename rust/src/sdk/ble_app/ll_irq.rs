use core::mem::{size_of, size_of_val};
use core::ptr::{addr_of, null};
use core::slice;
use core::sync::atomic::{AtomicU32, Ordering};
use crate::common::{SYS_CHN_LISTEN, pair_load_key, SYS_CHN_ADV};
use crate::sdk::ble_app::ble_ll_attribute::{get_slave_link_time_out, set_att_service_discover_tick, set_slave_link_time_out};
use crate::sdk::ble_app::light_ll::{*};
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::light::{*};
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, clock_time, sleep_us};
use crate::sdk::mcu::register::{*};
use crate::{app, BIT, uprintln};
use crate::embassy::time_driver::check_clock_overflow;
use crate::mesh::{get_mesh_node_mask, get_mesh_node_st, MESH_NODE_ST_VAL_LEN};
use crate::sdk::ble_app::ble_ll_att::{ble_ll_channel_table_calc, ble_ll_conn_get_next_channel};
use crate::sdk::ble_app::ble_ll_pair::pair_proc;
use crate::sdk::ble_app::shared_mem::get_light_rx_buff;
use crate::vendor_light::{get_adv_rsp_pri_data, get_adv_rsp_pri_data_addr};

fn slave_timing_update_handle()
{
    set_slave_instant(*get_slave_instant() + 1);
    if *get_slave_timing_update() == 1 {
        if *get_slave_instant_next() == *get_slave_instant() {
            set_slave_timing_update(0);
            ble_ll_channel_table_calc((*get_slave_chn_map()).as_ptr(), false);
            (*get_pkt_init()).chm = *get_slave_chn_map();
        }
    } else if *get_slave_timing_update() == 2 && *get_slave_instant_next() == *get_slave_instant() {
        set_slave_timing_update(0);
        set_slave_timing_update2_ok_time(*get_ble_conn_offset() + *get_slave_next_connect_tick());
        set_slave_link_interval(*get_ble_conn_interval());
        set_slave_link_time_out(*get_ble_conn_timeout());
        set_slave_timing_update2_flag(1);
        set_slave_window_size(*get_ble_conn_interval() - CLOCK_SYS_CLOCK_1US * 0x4e2);
        if *get_slave_window_size_update() < *get_slave_window_size() {
            set_slave_window_size(*get_slave_window_size_update());
        }
        set_slave_next_connect_tick(*get_slave_timing_update2_ok_time());
    }
}

fn rf_set_rxmode_mesh_listen()
{
    rf_stop_trx();

    rf_set_ble_access_code(*get_pair_ac());
    rf_set_ble_crc_adv();

    rf_set_ble_channel(SYS_CHN_LISTEN[*get_st_listen_no() as usize & 3]);
    rf_set_rxmode();

    set_slave_link_state(4);
}

fn rf_link_slave_read_status_update()
{
    let mut rptr = *get_slave_status_buffer_rptr();
    if *get_slave_status_buffer_wptr() != rptr {
        loop {
            rptr = rptr % *get_slave_status_buffer_num();
            if !rf_link_add_tx_packet(unsafe { get_p_slave_status_buffer().offset((rptr & 0xff) as isize) } as *const PacketAttCmd, size_of::<PacketAttCmd>()) {
                return;
            }

            rptr = ((*get_slave_status_buffer_rptr() + 1) % *get_slave_status_buffer_num()) & 0xff;
            set_slave_status_buffer_rptr(rptr);
            if *get_slave_status_buffer_wptr() == rptr {
                break;
            }
        }
    }
}

fn mesh_node_report_status(params: &mut [u8], len: usize) -> usize
{
    if !*get_mesh_node_report_enable() {
        return 0;
    }

    let mut result = 0;

    params[0..MESH_NODE_ST_VAL_LEN as usize * len as usize].fill(0);

    // Iterate over each 32 bit value in the mask
    //      For each 32 bit value, check if it's 0
    //          If it's 0, then no lights need to be reported for this 32 bit value
    //      Iterate over each bit in the 32 bit value and find any set bits
    //      Report the status of the light at any set bits and clear the bit from the mask

    for mask_idx in 0..get_mesh_node_mask().len() {
        let mask_val = &mut (*get_mesh_node_mask())[mask_idx];
        // If this mask value is not 0 then it means there are unsent status updates for this value
        if *mask_val != 0 {
            // Iterate over each bit in the word and test to see if the status needs to be sent
            for bit in 0..32 {
                // Make sure we don't exceed the maximum number of nodes
                let mut current_index = bit + (mask_idx * 32);
                if current_index > MESH_NODE_MAX_NUM as usize {
                    return result;
                }

                // If this bit is set, then report the light status
                if *mask_val & (1 << bit) != 0 {
                    // Clear the bit from the mask so it isn't reported again
                    *mask_val = *mask_val & !(1 << bit);
                    // Copy the node status value to the params
                    let params_idx = MESH_NODE_ST_VAL_LEN as usize * result;
                    params[params_idx..params_idx + MESH_NODE_ST_VAL_LEN as usize].copy_from_slice(
                        unsafe {
                            slice::from_raw_parts(
                                addr_of!((*get_mesh_node_st())[current_index].val) as *const u8,
                                MESH_NODE_ST_VAL_LEN as usize,
                            )
                        }
                    );

                    // If the tick is 0 (Device offline), set the sn value to 0
                    if (*get_mesh_node_st())[current_index].tick == 0 {
                        params[params_idx + 1] = 0;
                    }

                    // Increment the result, and if we've exhausted the params size, return
                    // Next time this function is called it will send the next chunk of statuses
                    result += 1;

                    if result >= len {
                        return result;
                    }
                }
            }
        }
    }

    return result;
}

pub fn irq_st_response()
{
    rf_set_tx_rx_off();
    sleep_us(100);
    rf_set_ble_access_code(*get_pair_ac());

    rf_set_ble_crc_adv();
    let tmp = (unsafe { **get_slave_p_mac() as u32 } ^ read_reg_system_tick()) & 3;
    for uVar4 in tmp..tmp + 4 {
        rf_set_ble_channel(SYS_CHN_LISTEN[uVar4 as usize & 3]);
        (*get_pkt_light_status())._type |= BIT!(7);
        rf_start_stx_mesh(get_pkt_light_status(), CLOCK_SYS_CLOCK_1US * 30 + read_reg_system_tick());
        sleep_us(700);
    }
    write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
    set_p_st_handler(Some(irq_st_listen));
}

pub fn irq_st_listen()
{
    set_slave_link_state(4);
    rf_stop_trx();

    app_bridge_cmd_handle(read_reg_system_tick());
    mesh_send_user_command();

    rf_set_rxmode_mesh_listen();
    write_reg_system_tick_irq(read_reg_system_tick() + *get_slave_listen_interval());
    if *get_adv_flag() {
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 7000 + read_reg_system_tick());
        set_p_st_handler(Some(irq_st_adv));
        return;
    }
    if !*get_online_st_flag() {
        set_st_listen_no(*get_st_listen_no() + 1);
        set_adv_flag(*get_st_listen_no() % *get_adv_interval2listen_interval() as u32 != 0);
        set_online_st_flag(*get_st_listen_no() % *get_online_status_interval2listen_interval() as u32 != 0);
        if *get_adv_flag() {
            write_reg_system_tick_irq((((read_reg_system_tick() ^ read_reg_rnd_number() as u32) & 0x7fff) * CLOCK_SYS_CLOCK_1US + read_reg_system_tick_irq()));
            set_p_st_handler(Some(irq_st_adv));
            return;
        }
        if !*get_online_st_flag() {
            return;
        }
    }

    set_p_st_handler(Some(irq_st_adv));
}

fn get_gatt_adv_cnt() -> u32
{
    return 3;
}

pub fn irq_st_adv()
{
    static ST_PNO: AtomicU32 = AtomicU32::new(0);
    static LAST_IBEACON_TIME: AtomicU32 = AtomicU32::new(0);
    static LAST_ALARM_TIME: AtomicU32 = AtomicU32::new(0);
    static I_BEACON_CNT: AtomicU32 = AtomicU32::new(0);

    write_reg8(0x80050f, 0);
    rf_stop_trx();

    set_slave_link_state(1);

    if !*get_adv_flag() || get_gatt_adv_cnt() <= ST_PNO.load(Ordering::Relaxed) {
        set_adv_flag(false);
        ST_PNO.store(0, Ordering::Relaxed);
        if *get_online_st_flag() {
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 0xdac + read_reg_system_tick());
            if CLOCK_SYS_CLOCK_1US * 30000000 < read_reg_system_tick() - LAST_ALARM_TIME.load(Ordering::Relaxed) {
                LAST_ALARM_TIME.store(read_reg_system_tick(), Ordering::Relaxed);
            } else {
                mesh_send_online_status();
                write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
            }
        } else {
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 500 + read_reg_system_tick());
        }
        set_online_st_flag(false);
        set_p_st_handler(Some(irq_st_listen));
        write_reg_rf_irq_status(1);
    } else {
        rf_set_ble_access_code_adv();
        rf_set_ble_crc_adv();

        rf_set_ble_channel(SYS_CHN_ADV[ST_PNO.load(Ordering::Relaxed) as usize % 3]);
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 0x4b0 + read_reg_system_tick());
        write_reg_rf_irq_status(1);

        if ST_PNO.load(Ordering::Relaxed) < 3 {
            rf_start_stx2rx(get_pkt_adv_addr() as u32, CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick());
        }

        ST_PNO.store(ST_PNO.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
        if get_gatt_adv_cnt() <= ST_PNO.load(Ordering::Relaxed) {
            ST_PNO.store(0, Ordering::Relaxed);
            set_adv_flag(false);
        }

        set_p_st_handler(Some(irq_st_listen));
    }
}

pub fn irq_st_bridge()
{
    static RF_SLAVE_OTA_TIMEOUT_TICK: AtomicU32 = AtomicU32::new(0);

    set_slave_link_state(5);
    set_st_brige_no(*get_st_brige_no() + 1);
    write_reg8(0x50f, 0);
    rf_stop_trx();

    if CLOCK_SYS_CLOCK_1US == 0x10 {
        write_reg8(0xf04, 0x5e);
    } else {
        write_reg8(0xf04, 0x68);
    }

    if *get_slave_link_time_out() * CLOCK_SYS_CLOCK_1US < read_reg_system_tick() - *get_slave_connected_tick() {
        if *get_rf_slave_ota_busy() != false {
            app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(OtaState::Error);
        }
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
        set_p_st_handler(Some(irq_st_adv));
        write_reg_dma_tx_rptr(0x10);
        set_slave_link_connected(false);
        if *get_not_need_login() == false {
            set_pair_login_ok(false);
        }
        if *get_security_enable() == false {
            mesh_report_status_enable(false);
        } else {
            set_slave_first_connected_tick(0);
            set_pair_login_ok(false);
            mesh_report_status_enable(false);
        }
        if *get_slave_read_status_busy() != 0 {
            rf_link_slave_read_status_stop();
        }
        if *get_pair_setting_flag() == PairState::PairSetted {
            set_slave_data_valid(0);
        } else {
            set_slave_data_valid(0);
            pair_load_key();
            set_pair_setting_flag(PairState::PairSetted);
        }
        if CLOCK_SYS_CLOCK_1US == 0x10 {
            write_reg8(0xf04, 0x5e);
        } else {
            write_reg8(0xf04, 0x68);
        }

        set_att_service_discover_tick(0);
        set_need_update_connect_para(false);

        return;
    }

    if *get_slave_read_status_busy() != 0  && SLAVE_READ_STATUS_BUSY_TIMEOUT * CLOCK_SYS_CLOCK_1US * 1000 < read_reg_system_tick() - *get_slave_read_status_busy_time() {
        rf_link_slave_read_status_stop();
    }

    if *get_rf_slave_ota_busy() {
        app().ota_manager.rf_link_slave_ota_finish_handle();

        if CLOCK_SYS_CLOCK_1US * 1000000 < read_reg_system_tick() - RF_SLAVE_OTA_TIMEOUT_TICK.load(Ordering::Relaxed) {
            RF_SLAVE_OTA_TIMEOUT_TICK.store(read_reg_system_tick(), Ordering::Relaxed);
            let sot = *get_rf_slave_ota_timeout_s();
            if sot != 0 {
                set_rf_slave_ota_timeout_s(sot - 1);
                if sot - 1 == 0
                {
                    app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(OtaState::Error);
                }
            }
        }
        if *get_rf_slave_ota_busy_mesh_en() == 0 {
            write_reg_system_tick_irq(*get_slave_next_connect_tick());
            set_p_st_handler(Some(irq_st_ble_rx));
            return;
        }
    }

    set_t_bridge_cmd(CLOCK_SYS_CLOCK_1US * 200 + read_reg_system_tick());

    if CLOCK_SYS_CLOCK_1US * 10000 < *get_slave_link_interval() && (*get_slave_interval_old() == 0 || *get_slave_instant_next() != *get_slave_instant()) {
        tx_packet_bridge();
    }

    if *get_slave_read_status_busy() != 0 {
        rf_link_slave_read_status_update();
    }

    let intflag = *get_update_interval_flag();
    if intflag != 0 {
        if intflag - 1 == 0 && *get_update_interval_time() {
            let mut pkt: [u32; 6] = [
                0x12,
                0xc1002,
                0x1120005,
                0x270008,
                0x27,
                0xc8
            ];

            if *get_update_interval_user_min() != 0 || *get_update_timeout_user() != 0 {
                pkt[5] = *get_update_timeout_user() as u32;
                // todo: These might need to be flipped?
                pkt[3] = ((*get_update_interval_user_min() as u32) << 0x10) | 0x27;
                pkt[4] = *get_update_interval_user_max() as u32 | 0x270000;
            }

            if is_add_packet_buf_ready() == false {
                set_update_interval_flag(2);
            } else {
                rf_link_add_tx_packet(addr_of!(pkt) as *const PacketAttCmd, size_of::<u32>() * pkt.len());
                set_update_interval_time(false);
            }
        }
    }
    if is_add_packet_buf_ready() {
        let pair_proc_result = pair_proc();
        if pair_proc_result != null() {
            rf_link_add_tx_packet(pair_proc_result as *const PacketAttCmd, size_of::<PacketAttReadRsp>());
        }
    }
    mesh_node_flush_status();
    if is_add_packet_buf_ready() {
        if mesh_node_report_status(&mut (*get_pkt_light_report()).value[10..], 10 / MESH_NODE_ST_VAL_LEN as usize) != 0 {
            rf_link_add_tx_packet(get_pkt_light_report_addr(), size_of::<PacketAttCmd>());
        }
    }

    back_to_rxmode_bridge();

    let mut uVar5 = (*get_slave_next_connect_tick() - (CLOCK_SYS_CLOCK_1US * 500)) - read_reg_system_tick();
    loop {
        if *get_slave_interval_old() != 0 && *get_slave_instant_next() == *get_slave_instant() {
            uprintln!("maybe fixme 2");
            uVar5 &= if 0x3fffffff < uVar5 { *get_slave_instant() as u32 } else { 0 };
            set_slave_interval_old(0);
        }

        if uVar5 <= *get_slave_link_interval() {
            break;
        }

        set_slave_next_connect_tick(*get_slave_next_connect_tick() + *get_slave_link_interval());
        slave_timing_update_handle();
        ble_ll_conn_get_next_channel((*get_pkt_init()).chm.as_ptr(), (*get_pkt_init()).hop & 0x1f);
        uVar5 = (*get_slave_next_connect_tick() - (CLOCK_SYS_CLOCK_1US * 500)) - read_reg_system_tick();
    }

    write_reg_system_tick_irq(*get_slave_next_connect_tick());
    set_p_st_handler(Some(irq_st_ble_rx));
}

pub fn irq_st_ble_rx()
{
    write_reg8(0x080050f, 0x80);
    rf_stop_trx();

    set_slave_link_state(7);

    write_reg8(0x00800f04, 0x67);  // tx wail & settle time

    rf_set_ble_channel(ble_ll_conn_get_next_channel((*get_pkt_init()).chm.as_ptr(), (*get_pkt_init()).hop & 0x1f) as u8);
    // rf_set_ble_access_code(unsafe { *(addr_of!((*get_pkt_init()).aa) as *const u32) });
    // rf_set_ble_crc(&(*get_pkt_init()).crcinit);

    write_reg_rf_access_code((((*get_pkt_init()).aa[2] as u32) << 8) | (((*get_pkt_init()).aa[1] as u32) << 0x10) | ((*get_pkt_init()).aa[3] as u32) | (((*get_pkt_init()).aa[0] as u32) << 0x18));
    write_reg_rf_crc((((*get_pkt_init()).crcinit[1] as u32) << 8) | (((*get_pkt_init()).crcinit[2] as u32) << 0x10) | (*get_pkt_init()).crcinit[0] as u32);

    set_slave_next_connect_tick(*get_slave_link_interval() + read_reg_system_tick_irq());
    if *get_rf_slave_ota_busy() == false {
        if *get_slave_window_size() == 0 || *get_slave_window_size() <= CLOCK_SYS_CLOCK_1US * 0xed8 {
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 0xed8 + read_reg_system_tick());
        } else {
            write_reg_system_tick_irq(read_reg_system_tick() + *get_slave_window_size());
        }
    } else {
        let tmp;
        if *get_rf_slave_ota_busy_mesh_en() == 0 {
            tmp = 10000;
        } else {
            tmp = 6000;
        }
        write_reg_system_tick_irq(tmp * CLOCK_SYS_CLOCK_1US + read_reg_system_tick());
    }
    set_p_st_handler(Some(irq_st_bridge));
    set_slave_tick_brx(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
    set_slave_timing_adjust_enable(true);
    rf_start_brx((*get_pkt_empty()).as_ptr() as u32, *get_slave_tick_brx());

    sleep_us(2);
    slave_timing_update_handle();
}

#[link_section = ".ram_code"]
#[inline(never)]
fn irq_light_slave_tx()
{
    write_reg_rf_irq_status(2);
}

#[link_section = ".ram_code"]
unsafe fn irq_light_slave_rx()
{
    static T_RX_LAST: AtomicU32 = AtomicU32::new(0);

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
        if T_RX_LAST.load(Ordering::Relaxed) == rx_time {
            rf_stop_trx();
            rf_start_stx2rx((*get_pkt_empty()).as_ptr() as u32, CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick());
            return;
        }
    } else if dma_len > 0xe && dma_len == (entry.sno[1] & 0x3f) + 0x11 && *((addr_of!(*entry) as u32 + dma_len as u32 + 3) as *const u8) & 0x51 == 0x40 {
        let packet = addr_of!(entry.rx_time);
        set_rcv_pkt_time(rx_time);
        if entry.sno[0] & 0xf == 3 {
            if *get_slave_link_state() != 1 && *get_slave_link_state() != 7 {
                if !*get_rf_slave_ota_busy() || *get_rf_slave_ota_busy_mesh_en() != 0 {
                    T_RX_LAST.store(rx_time, Ordering::Relaxed);
                    if *get_slave_link_state() < 6 {
                        rf_link_rc_data(&mut *(packet as *mut MeshPkt));
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
                    write_reg_rf_sched_tick(rx_time + *get_T_scan_rsp_intvl() * CLOCK_SYS_CLOCK_1US);
                    write_reg_rf_mode_control(0x85);                        // single TX
                    T_RX_LAST.store(rx_time, Ordering::Relaxed);

                    (*get_adv_rsp_pri_data()).device_address = *get_device_address_addr();
                    (*get_pkt_scan_rsp()).data[0] = 0x1e;
                    (*get_pkt_scan_rsp()).data[1] = 0xff;
                    (*get_pkt_scan_rsp()).data[2..2 + size_of::<AdvRspPrivate>()].copy_from_slice(
                        slice::from_raw_parts(get_adv_rsp_pri_data_addr() as *const u8, size_of::<AdvRspPrivate>())
                    );
                    (*get_pkt_scan_rsp()).dma_len = 0x27;
                    (*get_pkt_scan_rsp()).rf_len = 0x25;

                    write_reg_dma3_addr(get_pkt_scan_rsp_addr() as u16);
                    write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 1000 + read_reg_system_tick_irq());

                    entry.dma_len = 1;
                    return;
                }

                if !*get_rf_slave_ota_busy() || *get_rf_slave_ota_busy_mesh_en() != 0 {
                    T_RX_LAST.store(rx_time, Ordering::Relaxed);

                    if *get_slave_link_state() < 6 {
                        rf_link_rc_data(&mut *(packet as *mut MeshPkt));
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
                    T_RX_LAST.store(rx_time, Ordering::Relaxed);

                    rf_link_slave_connect(&*(packet as *const PacketLlInit), rx_time);

                    entry.dma_len = 1;
                    return;
                }

                if !*get_rf_slave_ota_busy() || *get_rf_slave_ota_busy_mesh_en() != 0 {
                    T_RX_LAST.store(rx_time, Ordering::Relaxed);

                    if *get_slave_link_state() < 6 {
                        rf_link_rc_data(&mut *(packet as *mut MeshPkt));
                    }

                    entry.dma_len = 1;
                    return;
                }
            }

            if *get_slave_link_state() != 7 {
                if !*get_rf_slave_ota_busy() || *get_rf_slave_ota_busy_mesh_en() != 0 {
                    T_RX_LAST.store(rx_time, Ordering::Relaxed);

                    if *get_slave_link_state() < 6 {
                        rf_link_rc_data(&mut *(packet as *mut MeshPkt));
                    }

                    entry.dma_len = 1;
                    return;
                }
            }
        }

        T_RX_LAST.store(rx_time, Ordering::Relaxed);

        let master_sn = ((entry.sno[2] as u16) * 0x100) | ((entry.sno[0] >> 3) & 1) as u16;
        if *get_light_conn_sn_master() == master_sn {
            rf_link_timing_adjust(rx_time);
        } else {
            set_light_conn_sn_master(master_sn);
            set_slave_connected_tick(read_reg_system_tick());
            set_slave_link_connected(true);
            rf_link_slave_data(&*(packet as *const PacketLlData), rx_time);
        }
        if *get_slave_window_size() != 0 {
            if *get_slave_timing_update2_flag() != 0 {
                if 0x40000001 > *get_slave_timing_update2_ok_time() - read_reg_system_tick() {
                    entry.dma_len = 1;
                    return;
                }
                set_slave_timing_update2_flag(0);
            }
            set_slave_window_size(0);

            set_slave_next_connect_tick(rx_time + *get_slave_link_interval() - CLOCK_SYS_CLOCK_1US * 1250);
        }
        entry.dma_len = 1;
        return;
    }

    T_RX_LAST.store(rx_time, Ordering::Relaxed);

    if *get_slave_link_state() == 7 {
        write_reg8(0x80050f, 0);
        rf_stop_trx();
    }

    entry.dma_len = 1;
}

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
        if (*get_p_st_handler()).is_some() {
            (*get_p_st_handler()).unwrap()();
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

#[inline(never)]
fn irq_timer1() {
    app().light_manager.transition_step();
}

// This timer is configured to run once per second to check if the internal clock has overflowed.
// this is a workaround in case there are no 'clock_time64' calls between overflows
#[inline(never)]
fn irq_timer0() {
    unsafe { check_clock_overflow(); }
}

// no_mangle because this is referenced as an entrypoint from the assembler bootstrap
#[no_mangle]
#[link_section = ".ram_code"]
extern "C" fn irq_handler() {
    unsafe { irq_light_slave_handler(); }

    app().uart_manager.check_irq();
}
