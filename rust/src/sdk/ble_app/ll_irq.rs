use core::cell::RefCell;
use core::mem::size_of;
use core::ptr::{addr_of, null};
use core::slice;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::{app, uprintln_fast};
use crate::common::{pair_load_key, SYS_CHN_ADV, SYS_CHN_LISTEN};
use crate::embassy::time_driver::clock_time64;
use crate::mesh::{MESH_NODE_ST_VAL_LEN};
use crate::sdk::ble_app::ble_ll_att::{ble_ll_channel_table_calc, ble_ll_conn_get_next_channel};
use crate::sdk::ble_app::ble_ll_attribute::{get_slave_link_time_out, set_att_service_discover_tick, set_slave_link_time_out};
use crate::sdk::ble_app::ble_ll_pair::pair_proc;
use crate::sdk::ble_app::light_ll::{*};
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::light::{*};
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, sleep_us};
use crate::sdk::mcu::register::{*};
use crate::state::{State, STATE};

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

fn rf_link_slave_read_status_update(state: &RefCell<State>)
{
    let mut rptr = *get_slave_status_buffer_rptr();
    while *get_slave_status_buffer_wptr() != rptr {
        rptr = rptr % BUFF_RESPONSE_PACKET_COUNT;
        let packet = state.borrow().buff_response[rptr];
        if !rf_link_add_tx_packet(state, addr_of!(packet) as *const PacketAttCmd, size_of::<PacketAttCmd>()) {
            return;
        }

        rptr = ((*get_slave_status_buffer_rptr() + 1) % BUFF_RESPONSE_PACKET_COUNT);
        set_slave_status_buffer_rptr(rptr);
    }
}

pub fn mesh_node_report_status(state: &RefCell<State>, params: &mut [u8], len: usize) -> usize
{
    if !*get_mesh_node_report_enable() {
        return 0;
    }

    let mut result = 0;

    params[0..MESH_NODE_ST_VAL_LEN as usize * len].fill(0);

    // Iterate over each 32 bit value in the mask
    //      For each 32 bit value, check if it's 0
    //          If it's 0, then no lights need to be reported for this 32 bit value
    //      Iterate over each bit in the 32 bit value and find any set bits
    //      Report the status of the light at any set bits and clear the bit from the mask

    let mut state = state.borrow_mut();

    let mut new_mask = state.mesh_node_mask;

    state.mesh_node_st.iter().enumerate().for_each(|(idx, val)| {
        if result >= len {
            return;
        }

        // Get the mask and bit indexes
        let mask_index = idx / 32;
        let mask_bit = idx % 32;
        let mask = new_mask[mask_index];

        // If this bit is set, then report the light status
        if mask & (1 << mask_bit) != 0 {
            // Clear the bit from the mask so it isn't reported again
            new_mask[mask_index] = mask & !(1 << mask_bit);

            // Copy the node status value to the params
            let params_idx = MESH_NODE_ST_VAL_LEN as usize * result;
            params[params_idx..params_idx + MESH_NODE_ST_VAL_LEN as usize].copy_from_slice(
                unsafe {
                    slice::from_raw_parts(
                        addr_of!(state.mesh_node_st[idx].val) as *const u8,
                        MESH_NODE_ST_VAL_LEN as usize,
                    )
                }
            );

            // If the tick is 0 (Device offline), set the sn value to 0
            if state.mesh_node_st[idx].tick == 0 {
                params[params_idx + 1] = 0;
            }

            // Increment the result, and if we've exhausted the params size, don't process any more
            // nodes.
            // Next time this function is called it will send the next chunk of statuses
            result += 1;
        }
    });

    state.mesh_node_mask = new_mask;

    return result;
}

pub fn irq_st_listen(state: &RefCell<State>)
{
    set_slave_link_state(4);
    rf_stop_trx();

    app_bridge_cmd_handle(read_reg_system_tick());
    mesh_send_user_command(state);

    rf_set_rxmode_mesh_listen();
    write_reg_system_tick_irq(read_reg_system_tick() + *get_slave_listen_interval());
    if *get_adv_flag() {
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 7000 + read_reg_system_tick());
        set_p_st_handler(IrqHandlerStatus::Adv);
        return;
    }
    if !*get_online_st_flag() {
        set_st_listen_no(*get_st_listen_no() + 1);
        set_adv_flag(*get_st_listen_no() % *get_adv_interval2listen_interval() as u32 != 0);
        set_online_st_flag(*get_st_listen_no() % *get_online_status_interval2listen_interval() as u32 != 0);
        if *get_adv_flag() {
            write_reg_system_tick_irq((((read_reg_system_tick() ^ read_reg_rnd_number() as u32) & 0x7fff) * CLOCK_SYS_CLOCK_1US + read_reg_system_tick_irq()));
            set_p_st_handler(IrqHandlerStatus::Adv);
            return;
        }
        if !*get_online_st_flag() {
            return;
        }
    }

    set_p_st_handler(IrqHandlerStatus::Adv);
}

fn get_gatt_adv_cnt() -> u32
{
    return 3;
}

pub fn irq_st_adv(state: &RefCell<State>)
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
                mesh_send_online_status(state);
                write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
            }
        } else {
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 500 + read_reg_system_tick());
        }
        set_online_st_flag(false);
        set_p_st_handler(IrqHandlerStatus::Listen);
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

        set_p_st_handler(IrqHandlerStatus::Listen);
    }
}

pub fn irq_st_bridge(state: &RefCell<State>)
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
        if *get_rf_slave_ota_busy() {
            app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(OtaState::Error);
        }
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
        set_p_st_handler(IrqHandlerStatus::Adv);
        write_reg_dma_tx_rptr(0x10);
        set_slave_link_connected(false);
        if *get_not_need_login() == false {
            set_pair_login_ok(false);
        }
        if *get_security_enable() == false {
            mesh_report_status_enable(state, false);
        } else {
            set_slave_first_connected_tick(0);
            set_pair_login_ok(false);
            mesh_report_status_enable(state, false);
        }
        if *get_slave_read_status_busy() != 0 {
            rf_link_slave_read_status_stop();
        }
        if *get_pair_setting_flag() == PairState::PairSetted {
            set_slave_data_valid(0);
        } else {
            set_slave_data_valid(0);
            pair_load_key(state);
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

    if *get_slave_read_status_busy() != 0 && SLAVE_READ_STATUS_BUSY_TIMEOUT * CLOCK_SYS_CLOCK_1US * 1000 < read_reg_system_tick() - *get_slave_read_status_busy_time() {
        rf_link_slave_read_status_stop();
    }

    if *get_rf_slave_ota_busy() {
        app().ota_manager.rf_link_slave_ota_finish_handle(state);

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

        write_reg_system_tick_irq(*get_slave_next_connect_tick());
        set_p_st_handler(IrqHandlerStatus::Rx);
        return;
    }

    set_t_bridge_cmd(CLOCK_SYS_CLOCK_1US * 200 + read_reg_system_tick());

    if CLOCK_SYS_CLOCK_1US * 10000 < *get_slave_link_interval() && (*get_slave_interval_old() == 0 || *get_slave_instant_next() != *get_slave_instant()) {
        tx_packet_bridge(state);
    }

    if *get_slave_read_status_busy() != 0 {
        rf_link_slave_read_status_update(state);
    }

    if is_add_packet_buf_ready() {
        let pair_proc_result = pair_proc(state);
        if pair_proc_result != null() {
            rf_link_add_tx_packet(state, pair_proc_result as *const PacketAttCmd, size_of::<PacketAttReadRsp>());
        }
    }
    mesh_node_flush_status(state);
    if is_add_packet_buf_ready() && !app().uart_manager.started() {
        if mesh_node_report_status(state, &mut (*get_pkt_light_report()).value.val[3..], 10 / MESH_NODE_ST_VAL_LEN as usize) != 0 {
            rf_link_add_tx_packet(state, get_pkt_light_report_addr(), size_of::<PacketAttCmd>());
        }
    }

    back_to_rxmode_bridge();

    loop {
        let mut cur_interval = (*get_slave_next_connect_tick() - (CLOCK_SYS_CLOCK_1US * 500)) - read_reg_system_tick();
        if *get_slave_interval_old() != 0 && *get_slave_instant_next() == *get_slave_instant() {
            if 0x3fffffff >= cur_interval {
                cur_interval = 0;
            }
            set_slave_interval_old(0);
        }

        if cur_interval <= *get_slave_link_interval() {
            break;
        }

        set_slave_next_connect_tick(*get_slave_next_connect_tick() + *get_slave_link_interval());
        slave_timing_update_handle();
        ble_ll_conn_get_next_channel((*get_pkt_init()).chm.as_ptr(), (*get_pkt_init()).hop & 0x1f);
    }

    write_reg_system_tick_irq(*get_slave_next_connect_tick());
    set_p_st_handler(IrqHandlerStatus::Rx);
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
        write_reg_system_tick_irq(10000 * CLOCK_SYS_CLOCK_1US + read_reg_system_tick());
    }
    set_p_st_handler(IrqHandlerStatus::Bridge);
    set_slave_tick_brx(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
    set_slave_timing_adjust_enable(true);
    rf_start_brx(addr_of!(PKT_EMPTY) as u32, *get_slave_tick_brx());
    sleep_us(2);
    slave_timing_update_handle();
}

fn irq_light_slave_tx()
{
    write_reg_rf_irq_status(2);
}

fn irq_light_slave_rx(state: &RefCell<State>)
{
    static T_RX_LAST: AtomicU32 = AtomicU32::new(0);

    let rx_index;
    let rx_time;
    let dma_len;
    {
        let mut state_mut = state.borrow_mut();

        rx_index = state_mut.light_rx_wptr;
        state_mut.light_rx_wptr = (rx_index + 1) % state_mut.light_rx_buff.len();

        if read_reg_rf_rx_status() == 0x0b {
            write_reg_rf_irq_status(1);
            return;
        }

        let next_addr = addr_of!(state_mut.light_rx_buff[state_mut.light_rx_wptr]);

        let entry = &mut state_mut.light_rx_buff[rx_index];

        write_reg_dma2_addr(next_addr as u16);
        write_reg_rf_irq_status(1);

        set_light_rcv_rssi(entry.rssi);

        rx_time = entry.rx_time;
        dma_len = entry.dma_len;

        entry.dma_len = 1;

        if dma_len == 1 {
            if T_RX_LAST.load(Ordering::Relaxed) == rx_time {
                rf_stop_trx();
                rf_start_stx2rx(addr_of!(PKT_EMPTY) as u32, CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick());
                return;
            }

            return;
        }
    }

    let entry = state.borrow().light_rx_buff[rx_index];

    if dma_len > 0xe && dma_len == (entry.sno[1] & 0x3f) + 0x11 && unsafe { *((addr_of!(entry) as u32 + dma_len as u32 + 3) as *const u8) } & 0x51 == 0x40 {
        let packet = addr_of!(entry.rx_time);
        let mut return_fn = || {
            T_RX_LAST.store(rx_time, Ordering::Relaxed);
            if *get_slave_link_state() < 6 {
                rf_link_rc_data(state, unsafe { *(packet as *mut MeshPkt) }, true);
            }
            return;
        };

        set_rcv_pkt_time(rx_time);
        if entry.sno[0] & 0xf == 3 {
            if *get_slave_link_state() != 1 && *get_slave_link_state() != 7 {
                if !*get_rf_slave_ota_busy() {
                    return return_fn();
                }
            }
            if *get_slave_link_state() == 1 {
                if entry.mac == (*get_mac_id())[0..4] {
                    rf_stop_trx();
                    write_reg_rf_sched_tick(rx_time + *get_T_scan_rsp_intvl() * CLOCK_SYS_CLOCK_1US);
                    write_reg_rf_mode_control(0x85);                        // single TX
                    T_RX_LAST.store(rx_time, Ordering::Relaxed);

                    state.borrow_mut().adv_rsp_pri_data.device_address = unsafe { *get_device_address_addr() };
                    (*get_pkt_scan_rsp()).data[0] = 0x1e;
                    (*get_pkt_scan_rsp()).data[1] = 0xff;
                    (*get_pkt_scan_rsp()).data[2..2 + size_of::<AdvRspPrivate>()].copy_from_slice(
                        unsafe { slice::from_raw_parts(addr_of!(state.borrow().adv_rsp_pri_data) as *const u8, size_of::<AdvRspPrivate>()) }
                    );
                    (*get_pkt_scan_rsp()).dma_len = 0x27;
                    (*get_pkt_scan_rsp()).rf_len = 0x25;

                    write_reg_dma3_addr(get_pkt_scan_rsp_addr() as u16);
                    write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 1000 + read_reg_system_tick_irq());

                    return;
                }

                if !*get_rf_slave_ota_busy() {
                    return return_fn();
                }
            }
        } else {
            if entry.sno[0] & 0xf == 5 && *get_slave_link_state() == 1 {
                if entry.mac == (*get_mac_id())[0..4] {
                    T_RX_LAST.store(rx_time, Ordering::Relaxed);

                    rf_link_slave_connect(state, unsafe { &*(packet as *const PacketLlInit) }, rx_time);

                    return;
                }

                if !*get_rf_slave_ota_busy() {
                    return return_fn();
                }
            }

            if *get_slave_link_state() != 7 {
                if !*get_rf_slave_ota_busy() {
                    return return_fn();
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
            rf_link_slave_data(state, unsafe { &*(packet as *const PacketLlData) }, rx_time);
        }
        if *get_slave_window_size() != 0 {
            if *get_slave_timing_update2_flag() != 0 {
                if 0x40000001 > *get_slave_timing_update2_ok_time() - read_reg_system_tick() {
                    return;
                }
                set_slave_timing_update2_flag(0);
            }
            set_slave_window_size(0);

            set_slave_next_connect_tick(rx_time + *get_slave_link_interval() - CLOCK_SYS_CLOCK_1US * 1250);
        }
        return;
    }

    T_RX_LAST.store(rx_time, Ordering::Relaxed);

    if *get_slave_link_state() == 7 {
        write_reg8(0x80050f, 0);
        rf_stop_trx();
    }
}

static IS_IRQ_MODE: AtomicBool = AtomicBool::new(false);

pub struct IrqTracker {}

impl IrqTracker {
    pub fn new() -> Self {
        IS_IRQ_MODE.store(true, Ordering::Relaxed);
        return IrqTracker {}
    }

    pub fn in_irq() -> bool {
        IS_IRQ_MODE.load(Ordering::Relaxed)
    }
}

impl Drop for IrqTracker {
    fn drop(&mut self) {
        IS_IRQ_MODE.store(false, Ordering::Relaxed);
    }
}

// no_mangle because this is referenced as an entrypoint from the assembler bootstrap
#[no_mangle]
extern "C" fn irq_handler() {
    let _tracker = IrqTracker::new();

    STATE.lock(|state| {
        let irq = read_reg_rf_irq_status();
        if irq & FLD_RF_IRQ_MASK::IRQ_RX as u16 != 0 {
            irq_light_slave_rx(state);
        }

        if irq & FLD_RF_IRQ_MASK::IRQ_TX as u16 != 0 {
            irq_light_slave_tx();
        }

        let irq_source = read_reg_irq_src();
        if irq_source & FLD_IRQ::SYSTEM_TIMER as u32 != 0 {
            write_reg_irq_src(FLD_IRQ::SYSTEM_TIMER as u32);
            match *get_p_st_handler() {
                IrqHandlerStatus::Adv => irq_st_adv(state),
                IrqHandlerStatus::Bridge => irq_st_bridge(state),
                IrqHandlerStatus::Rx => irq_st_ble_rx(),
                IrqHandlerStatus::Listen => irq_st_listen(state),
                IrqHandlerStatus::None => {}
            }
        }

        // This timer is configured to run once per second to check if the internal clock has overflowed.
        // this is a workaround in case there are no 'clock_time64' calls between overflows
        if irq_source & FLD_IRQ::TMR0_EN as u32 != 0 {
            write_reg_tmr_sta(FLD_TMR_STA::TMR0 as u8);

            clock_time64();

            if *get_rf_slave_ota_busy_mesh() {
                let sot = *get_rf_slave_ota_timeout_s();
                if sot != 0 {
                    set_rf_slave_ota_timeout_s(sot - 1);
                    if sot - 1 == 0
                    {
                        app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(*get_rf_slave_ota_finished_flag());
                    }
                }
            }
        }

        // This timer triggers the transition step of the light so that the transition is smooth
        if irq_source & FLD_IRQ::TMR1_EN as u32 != 0 {
            write_reg_tmr_sta(FLD_TMR_STA::TMR1 as u8);

            app().light_manager.transition_step();
        }

        app().uart_manager.check_irq(state);
    });
}
