use core::mem::size_of;
use core::ops::DerefMut;
use core::ptr::{addr_of, null};
use core::slice;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::app;
use crate::common::{pair_load_key, SYS_CHN_ADV, SYS_CHN_LISTEN};
use crate::embassy::time_driver::clock_time64;
use crate::mesh::MESH_NODE_ST_VAL_LEN;
use crate::sdk::ble_app::ble_ll_att::{ble_ll_channel_table_calc, ble_ll_conn_get_next_channel};
use crate::sdk::ble_app::ble_ll_pair::pair_proc;
use crate::sdk::ble_app::light_ll::{*};
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::light::{*};
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, sleep_us};
use crate::sdk::mcu::register::{*};
use crate::state::{State, STATE};

fn slave_timing_update_handle(state: &mut State)
{
    state.slave_instant += 1;
    if state.slave_timing_update == 1 && state.slave_instant_next == state.slave_instant {
        state.slave_timing_update = 0;
        let chn_map = state.slave_chn_map.clone();
        ble_ll_channel_table_calc(state, &chn_map, false);
        state.pkt_init.chm = chn_map;
    } else {


        if state.slave_timing_update == 2 && state.slave_instant_next == state.slave_instant {
            state.slave_timing_update = 0;
            state.slave_timing_update2_ok_time = state.ble_conn_offset + state.slave_next_connect_tick;
            state.slave_link_interval = state.ble_conn_interval;
            state.slave_link_time_out = state.ble_conn_timeout;
            state.slave_timing_update2_flag = 1;
            state.slave_window_size = state.ble_conn_interval - CLOCK_SYS_CLOCK_1US * 0x4e2;
            if state.slave_window_size_update < state.slave_window_size {
                state.slave_window_size = state.slave_window_size_update;
            }
            state.slave_next_connect_tick = state.slave_timing_update2_ok_time;
        }
    }
}

fn rf_set_rxmode_mesh_listen(state: &mut State)
{
    rf_stop_trx();

    rf_set_ble_access_code(state.pair_ac);
    rf_set_ble_crc_adv();

    rf_set_ble_channel(state, SYS_CHN_LISTEN[state.st_listen_no as usize & 3]);
    rf_set_rxmode();

    state.slave_link_state = 4;
}

fn rf_link_slave_read_status_update(state: &mut State)
{
    let mut rptr = state.slave_status_buffer_rptr;
    while state.slave_status_buffer_wptr != rptr {
        rptr = rptr % BUFF_RESPONSE_PACKET_COUNT;
        let packet = state.buff_response[rptr];
        if !rf_link_add_tx_packet(state, addr_of!(packet) as *const PacketAttCmd, size_of::<PacketAttCmd>()) {
            return;
        }

        rptr = ((state.slave_status_buffer_rptr + 1) % BUFF_RESPONSE_PACKET_COUNT);
        state.slave_status_buffer_rptr = rptr;
    }
}

pub fn mesh_node_report_status(state: &mut State, params: &mut [u8], len: usize) -> usize
{


    if !state.mesh_node_report_enable {
        return 0;
    }

    let mut result = 0;

    params[0..MESH_NODE_ST_VAL_LEN as usize * len].fill(0);

    // Iterate over each 32 bit value in the mask
    //      For each 32 bit value, check if it's 0
    //          If it's 0, then no lights need to be reported for this 32 bit value
    //      Iterate over each bit in the 32 bit value and find any set bits
    //      Report the status of the light at any set bits and clear the bit from the mask

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

pub fn irq_st_listen(state: &mut State)
{
    state.slave_link_state = 4;
    rf_stop_trx();

    app_bridge_cmd_handle(state, read_reg_system_tick());
    mesh_send_user_command(state);

    rf_set_rxmode_mesh_listen(state);


    write_reg_system_tick_irq(read_reg_system_tick() + state.slave_listen_interval);
    if state.adv_flag {
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 7000 + read_reg_system_tick());
        state.p_st_handler = IrqHandlerStatus::Adv;
        return;
    }
    if !state.online_st_flag {
        state.st_listen_no += 1;
        state.adv_flag = state.st_listen_no % state.adv_interval2listen_interval as u32 != 0;
        state.online_st_flag = state.st_listen_no % state.online_status_interval2listen_interval as u32 != 0;
        if state.adv_flag {
            write_reg_system_tick_irq((((read_reg_system_tick() ^ read_reg_rnd_number() as u32) & 0x7fff) * CLOCK_SYS_CLOCK_1US + read_reg_system_tick_irq()));
            state.p_st_handler = IrqHandlerStatus::Adv;
            return;
        }
        if !state.online_st_flag {
            return;
        }
    }

    state.p_st_handler = IrqHandlerStatus::Adv;
}

fn get_gatt_adv_cnt() -> u32
{
    return 3;
}

pub fn irq_st_adv(state: &mut State)
{
    static ST_PNO: AtomicU32 = AtomicU32::new(0);
    static LAST_IBEACON_TIME: AtomicU32 = AtomicU32::new(0);
    static LAST_ALARM_TIME: AtomicU32 = AtomicU32::new(0);
    static I_BEACON_CNT: AtomicU32 = AtomicU32::new(0);

    write_reg8(0x80050f, 0);
    rf_stop_trx();

    state.slave_link_state = 1;

    if !state.adv_flag || get_gatt_adv_cnt() <= ST_PNO.load(Ordering::Relaxed) {
        state.adv_flag = false;
        ST_PNO.store(0, Ordering::Relaxed);
        if state.online_st_flag {
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
        state.online_st_flag = false;
        state.p_st_handler = IrqHandlerStatus::Listen;
        write_reg_rf_irq_status(1);
    } else {
        rf_set_ble_access_code_adv();
        rf_set_ble_crc_adv();

        rf_set_ble_channel(state, SYS_CHN_ADV[ST_PNO.load(Ordering::Relaxed) as usize % 3]);
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 0x4b0 + read_reg_system_tick());
        write_reg_rf_irq_status(1);

        if ST_PNO.load(Ordering::Relaxed) < 3 {
            rf_start_stx2rx(addr_of!(state.pkt_adv) as u32, CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick());
        }

        ST_PNO.store(ST_PNO.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
        if get_gatt_adv_cnt() <= ST_PNO.load(Ordering::Relaxed) {
            ST_PNO.store(0, Ordering::Relaxed);
            state.adv_flag = false;
        }

        state.p_st_handler = IrqHandlerStatus::Listen;
    }
}

pub fn irq_st_bridge(state: &mut State)
{
    static RF_SLAVE_OTA_TIMEOUT_TICK: AtomicU32 = AtomicU32::new(0);

    state.slave_link_state = 5;
    state.st_brige_no += 1;
    write_reg8(0x50f, 0);
    rf_stop_trx();

    if CLOCK_SYS_CLOCK_1US == 0x10 {
        write_reg8(0xf04, 0x5e);
    } else {
        write_reg8(0xf04, 0x68);
    }

    if state.slave_link_time_out * CLOCK_SYS_CLOCK_1US < read_reg_system_tick() - state.slave_connected_tick {
        if state.rf_slave_ota_busy {
            app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(state, OtaState::Error);
        }
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
        state.p_st_handler = IrqHandlerStatus::Adv;
        write_reg_dma_tx_rptr(0x10);
        state.slave_link_connected = false;
        if state.not_need_login == false {
            state.pair_login_ok = false;
        }
        if state.security_enable == false {
            mesh_report_status_enable(state, false);
        } else {
            state.slave_first_connected_tick = 0;
            state.pair_login_ok = false;
            mesh_report_status_enable(state, false);
        }
        if state.slave_read_status_busy != 0 {
            rf_link_slave_read_status_stop(state);
        }
        if state.pair_setting_flag == PairState::PairSetted {
            state.slave_data_valid = 0;
        } else {
            state.slave_data_valid = 0;
            pair_load_key(state);
            state.pair_setting_flag = PairState::PairSetted;
        }
        if CLOCK_SYS_CLOCK_1US == 0x10 {
            write_reg8(0xf04, 0x5e);
        } else {
            write_reg8(0xf04, 0x68);
        }

        state.att_service_discover_tick = 0;
        state.need_update_connect_para = false;

        return;
    }

    if state.slave_read_status_busy != 0 && SLAVE_READ_STATUS_BUSY_TIMEOUT * CLOCK_SYS_CLOCK_1US * 1000 < read_reg_system_tick() - state.slave_read_status_busy_time {
        rf_link_slave_read_status_stop(state);
    }

    if state.rf_slave_ota_busy {
        app().ota_manager.rf_link_slave_ota_finish_handle(state);

        if CLOCK_SYS_CLOCK_1US * 1000000 < read_reg_system_tick() - RF_SLAVE_OTA_TIMEOUT_TICK.load(Ordering::Relaxed) {
            RF_SLAVE_OTA_TIMEOUT_TICK.store(read_reg_system_tick(), Ordering::Relaxed);
            let sot = state.rf_slave_ota_timeout_s;
            if sot != 0 {
                state.rf_slave_ota_timeout_s = sot - 1;
                if sot - 1 == 0
                {
                    app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(state, OtaState::Error);
                }
            }
        }

        write_reg_system_tick_irq(state.slave_next_connect_tick);
        state.p_st_handler = IrqHandlerStatus::Rx;
        return;
    }

    state.t_bridge_cmd = CLOCK_SYS_CLOCK_1US * 200 + read_reg_system_tick();

    if CLOCK_SYS_CLOCK_1US * 10000 < state.slave_link_interval && state.slave_interval_old == 0 || state.slave_instant_next != state.slave_instant {
        tx_packet_bridge(state);
    }

    if state.slave_read_status_busy != 0 {
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
        let mut data = [0u8; 20];
        if mesh_node_report_status(state, &mut data, 10 / MESH_NODE_ST_VAL_LEN) != 0 {
            state.pkt_light_report.value.val[3..].copy_from_slice(&data);
            rf_link_add_tx_packet(state, &state.pkt_light_report, size_of::<PacketAttCmd>());
        }
    }

    back_to_rxmode_bridge(state);

    loop {
        {


            let mut cur_interval = (state.slave_next_connect_tick - (CLOCK_SYS_CLOCK_1US * 500)) - read_reg_system_tick();
            if state.slave_interval_old != 0 && state.slave_instant_next == state.slave_instant {
                if 0x3fffffff >= cur_interval {
                    cur_interval = 0;
                }
                state.slave_interval_old = 0;
            }

            if cur_interval <= state.slave_link_interval {
                break;
            }

            state.slave_next_connect_tick = state.slave_next_connect_tick + state.slave_link_interval;
        }
        slave_timing_update_handle(state);
        let chn_map = state.pkt_init.chm;
        ble_ll_conn_get_next_channel(state, &chn_map, state.pkt_init.hop & 0x1f);
    }


    write_reg_system_tick_irq(state.slave_next_connect_tick);
    state.p_st_handler = IrqHandlerStatus::Rx;
}

pub fn irq_st_ble_rx(state: &mut State)
{
    write_reg8(0x080050f, 0x80);
    rf_stop_trx();

    state.slave_link_state = 7;

    write_reg8(0x00800f04, 0x67);  // tx wail & settle time

    let chn_map = state.pkt_init.chm;
    let next_chan = ble_ll_conn_get_next_channel(state, &chn_map, state.pkt_init.hop & 0x1f) as u8;
    rf_set_ble_channel(state, next_chan);

    // rf_set_ble_access_code(unsafe { *(addr_of!((state.pkt_init()).aa) as *const u32) });
    // rf_set_ble_crc(&(state.pkt_init()).crcinit);
    write_reg_rf_access_code(((state.pkt_init.aa[2] as u32) << 8) | ((state.pkt_init.aa[1] as u32) << 0x10) | (state.pkt_init.aa[3] as u32) | ((state.pkt_init.aa[0] as u32) << 0x18));
    write_reg_rf_crc(((state.pkt_init.crcinit[1] as u32) << 8) | ((state.pkt_init.crcinit[2] as u32) << 0x10) | state.pkt_init.crcinit[0] as u32);

    state.slave_next_connect_tick = state.slave_link_interval + read_reg_system_tick_irq();
    if state.rf_slave_ota_busy == false {
        if state.slave_window_size == 0 || state.slave_window_size <= CLOCK_SYS_CLOCK_1US * 0xed8 {
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 0xed8 + read_reg_system_tick());
        } else {
            write_reg_system_tick_irq(read_reg_system_tick() + state.slave_window_size);
        }
    } else {
        write_reg_system_tick_irq(10000 * CLOCK_SYS_CLOCK_1US + read_reg_system_tick());
    }
    state.p_st_handler = IrqHandlerStatus::Bridge;
    state.slave_tick_brx = CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick();
    state.slave_timing_adjust_enable = true;
    rf_start_brx(addr_of!(PKT_EMPTY) as u32, state.slave_tick_brx);
    sleep_us(2);

    slave_timing_update_handle(state);
}

fn irq_light_slave_tx()
{
    write_reg_rf_irq_status(2);
}

fn irq_light_slave_rx(state: &mut State)
{
    static T_RX_LAST: AtomicU32 = AtomicU32::new(0);

    let rx_index;
    let rx_time;
    let dma_len;
    {
        rx_index = state.light_rx_wptr;
        state.light_rx_wptr = (rx_index + 1) % state.light_rx_buff.len();

        if read_reg_rf_rx_status() == 0x0b {
            write_reg_rf_irq_status(1);
            return;
        }

        let next_addr = addr_of!(state.light_rx_buff[state.light_rx_wptr]);

        let rssi;
        {
            let entry = &mut state.light_rx_buff[rx_index];

            write_reg_dma2_addr(next_addr as u16);
            write_reg_rf_irq_status(1);

            rx_time = entry.rx_time;
            dma_len = entry.dma_len;

            entry.dma_len = 1;

            rssi = entry.rssi;
        }

        state.light_rcv_rssi = rssi;

        if dma_len == 1 {
            if T_RX_LAST.load(Ordering::Relaxed) == rx_time {
                rf_stop_trx();
                rf_start_stx2rx(addr_of!(PKT_EMPTY) as u32, CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick());
                return;
            }

            return;
        }
    }

    let entry = state.light_rx_buff[rx_index];

    if dma_len > 0xe && dma_len == (entry.sno[1] & 0x3f) + 0x11 && unsafe { *((addr_of!(entry) as u32 + dma_len as u32 + 3) as *const u8) } & 0x51 == 0x40 {
        let packet = addr_of!(entry.rx_time);

        state.rcv_pkt_time = rx_time;
        if entry.sno[0] & 0xf == 3 {
            if state.slave_link_state != 1 && state.slave_link_state != 7 {
                if !state.rf_slave_ota_busy {
                    T_RX_LAST.store(rx_time, Ordering::Relaxed);
                    if state.slave_link_state < 6 {
                        rf_link_rc_data(state, unsafe { *(packet as *mut MeshPkt) }, true);
                    }
                    return;
                }
            }
            if state.slave_link_state == 1 {


                if entry.mac == state.mac_id[0..4] {
                    rf_stop_trx();
                    write_reg_rf_sched_tick(rx_time + state.t_scan_rsp_intvl * CLOCK_SYS_CLOCK_1US);
                    write_reg_rf_mode_control(0x85);                        // single TX
                    T_RX_LAST.store(rx_time, Ordering::Relaxed);

                    state.adv_rsp_pri_data.device_address = state.device_address;
                    state.pkt_scan_rsp.data[0] = 0x1e;
                    state.pkt_scan_rsp.data[1] = 0xff;
                    state.pkt_scan_rsp.data[2..2 + size_of::<AdvRspPrivate>()].copy_from_slice(
                        unsafe { slice::from_raw_parts(addr_of!(state.adv_rsp_pri_data) as *const u8, size_of::<AdvRspPrivate>()) }
                    );
                    state.pkt_scan_rsp.dma_len = 0x27;
                    state.pkt_scan_rsp.rf_len = 0x25;

                    write_reg_dma3_addr(addr_of!(state.pkt_scan_rsp) as u16);
                    write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 1000 + read_reg_system_tick_irq());

                    return;
                }

                if !state.rf_slave_ota_busy {
                    T_RX_LAST.store(rx_time, Ordering::Relaxed);
                    if state.slave_link_state < 6 {
                        rf_link_rc_data(state, unsafe { *(packet as *mut MeshPkt) }, true);
                    }
                    return;
                }
            }
        } else {
            if entry.sno[0] & 0xf == 5 && state.slave_link_state == 1 {
                if entry.mac == state.mac_id[0..4] {
                    T_RX_LAST.store(rx_time, Ordering::Relaxed);

                    rf_link_slave_connect(state, unsafe { &*(packet as *const PacketLlInit) }, rx_time);

                    return;
                }

                if !state.rf_slave_ota_busy {
                    T_RX_LAST.store(rx_time, Ordering::Relaxed);
                    if state.slave_link_state < 6 {
                        rf_link_rc_data(state, unsafe { *(packet as *mut MeshPkt) }, true);
                    }
                    return;
                }
            }

            if state.slave_link_state != 7 {
                if !state.rf_slave_ota_busy {
                    T_RX_LAST.store(rx_time, Ordering::Relaxed);
                    if state.slave_link_state < 6 {
                        rf_link_rc_data(state, unsafe { *(packet as *mut MeshPkt) }, true);
                    }
                    return;
                }
            }
        }

        T_RX_LAST.store(rx_time, Ordering::Relaxed);

        let master_sn = ((entry.sno[2] as u16) * 0x100) | ((entry.sno[0] >> 3) & 1) as u16;
        if state.light_conn_sn_master == master_sn {
            rf_link_timing_adjust(state, rx_time);
        } else {
            {

                state.light_conn_sn_master = master_sn;
                state.slave_connected_tick = read_reg_system_tick();
                state.slave_link_connected = true;
            }
            rf_link_slave_data(state, unsafe { &*(packet as *const PacketLlData) }, rx_time);
        }


        if state.slave_window_size != 0 {
            if state.slave_timing_update2_flag != 0 {
                if 0x40000001 > state.slave_timing_update2_ok_time - read_reg_system_tick() {
                    return;
                }
                state.slave_timing_update2_flag = 0;
            }
            state.slave_window_size = 0;

            state.slave_next_connect_tick = rx_time + state.slave_link_interval - CLOCK_SYS_CLOCK_1US * 1250;
        }
        return;
    }

    T_RX_LAST.store(rx_time, Ordering::Relaxed);

    if state.slave_link_state == 7 {
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
        let mut binding = state.borrow_mut();
        let state = binding.deref_mut();

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
            let status = state.p_st_handler;
            match status {
                IrqHandlerStatus::Adv => irq_st_adv(state),
                IrqHandlerStatus::Bridge => irq_st_bridge(state),
                IrqHandlerStatus::Rx => irq_st_ble_rx(state),
                IrqHandlerStatus::Listen => irq_st_listen(state),
                IrqHandlerStatus::None => {}
            }
        }

        // This timer is configured to run once per second to check if the internal clock has overflowed.
        // this is a workaround in case there are no 'clock_time64' calls between overflows
        if irq_source & FLD_IRQ::TMR0_EN as u32 != 0 {
            write_reg_tmr_sta(FLD_TMR_STA::TMR0 as u8);

            clock_time64();

            if state.rf_slave_ota_busy_mesh {
                let sot = state.rf_slave_ota_timeout_s;
                if sot != 0 {
                    state.rf_slave_ota_timeout_s = sot - 1;
                    if sot - 1 == 0
                    {
                        app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(state, state.rf_slave_ota_finished_flag);
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
