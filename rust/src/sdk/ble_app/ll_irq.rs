use core::cell::RefMut;
use core::mem::size_of;
use core::ops::DerefMut;
use core::ptr::{addr_of, null};
use core::slice;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::{app};
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
use crate::state::{*};

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
            SLAVE_TIMING_UPDATE2_OK_TIME.set(state.ble_conn_offset + SLAVE_NEXT_CONNECT_TICK.get());
            SLAVE_LINK_INTERVAL.set(state.ble_conn_interval);
            state.slave_link_time_out = state.ble_conn_timeout;
            SLAVE_TIMING_UPDATE2_FLAG.set(true);
            SLAVE_WINDOW_SIZE.set(state.ble_conn_interval - CLOCK_SYS_CLOCK_1US * 0x4e2);
            if SLAVE_WINDOW_SIZE_UPDATE.get() < SLAVE_WINDOW_SIZE.get() {
                SLAVE_WINDOW_SIZE.set(SLAVE_WINDOW_SIZE_UPDATE.get());
            }
            SLAVE_NEXT_CONNECT_TICK.set(SLAVE_TIMING_UPDATE2_OK_TIME.get());
        }
    }
}

fn rf_set_rxmode_mesh_listen()
{
    rf_stop_trx();

    rf_set_ble_access_code(PAIR_AC.get());
    rf_set_ble_crc_adv();

    rf_set_ble_channel(SYS_CHN_LISTEN[ST_LISTEN_NO.get() as usize % SYS_CHN_LISTEN.len()]);
    rf_set_rxmode();

    SLAVE_LINK_STATE.set(4);
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
    if !MESH_NODE_REPORT_ENABLE.get() {
        return 0;
    }

    let mut result = 0;

    params[0..MESH_NODE_ST_VAL_LEN * len].fill(0);

    // Iterate over each 32 bit value in the mask
    //      For each 32 bit value, check if it's 0
    //          If it's 0, then no lights need to be reported for this 32 bit value
    //      Iterate over each bit in the 32 bit value and find any set bits
    //      Report the status of the light at any set bits and clear the bit from the mask

    let binding = MESH_NODE_MASK.lock();
    let mut mesh_node_mask = binding.borrow_mut();

    state.mesh_node_st.iter().enumerate().for_each(|(idx, val)| {
        if result >= len {
            return;
        }

        // Get the mask and bit indexes
        let mask_index = idx / 32;
        let mask_bit = idx % 32;
        let mask = mesh_node_mask[mask_index];

        // If this bit is set, then report the light status
        if mask & (1 << mask_bit) != 0 {
            // Clear the bit from the mask so it isn't reported again
            mesh_node_mask[mask_index] = mask & !(1 << mask_bit);

            // Copy the node status value to the params
            let params_idx = MESH_NODE_ST_VAL_LEN * result;
            params[params_idx..params_idx + MESH_NODE_ST_VAL_LEN].copy_from_slice(
                unsafe {
                    slice::from_raw_parts(
                        addr_of!(state.mesh_node_st[idx].val) as *const u8,
                        MESH_NODE_ST_VAL_LEN,
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

    return result;
}

pub fn irq_st_listen(state: &mut State)
{
    SLAVE_LINK_STATE.set(4);

    rf_stop_trx();

    app_bridge_cmd_handle(state, read_reg_system_tick());

    rf_set_rxmode_mesh_listen();

    write_reg_system_tick_irq(read_reg_system_tick() + state.slave_listen_interval);
    if state.adv_flag {
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 7000 + read_reg_system_tick());
        state.p_st_handler = IrqHandlerStatus::Adv;
        return;
    }
    if !state.online_st_flag {
        ST_LISTEN_NO.set(ST_LISTEN_NO.get() + 1);
        state.adv_flag = ST_LISTEN_NO.get() % state.adv_interval2listen_interval as u32 != 0;
        state.online_st_flag = ST_LISTEN_NO.get() % state.online_status_interval2listen_interval as u32 != 0;
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

    write_reg8(0x80050f, 0);
    rf_stop_trx();

    SLAVE_LINK_STATE.set(1);

    if !state.adv_flag || get_gatt_adv_cnt() <= ST_PNO.load(Ordering::Relaxed) {
        state.adv_flag = false;
        ST_PNO.store(0, Ordering::Relaxed);
        if state.online_st_flag {
            mesh_send_online_status(state);
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
        } else {
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 500 + read_reg_system_tick());
        }
        state.online_st_flag = false;
        state.p_st_handler = IrqHandlerStatus::Listen;
        write_reg_rf_irq_status(1);
    } else {
        rf_set_ble_access_code_adv();
        rf_set_ble_crc_adv();

        rf_set_ble_channel(SYS_CHN_ADV[ST_PNO.load(Ordering::Relaxed) as usize % 3]);
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

    SLAVE_LINK_STATE.set(5);

    state.st_brige_no += 1;
    write_reg8(0x50f, 0);
    rf_stop_trx();

    if CLOCK_SYS_CLOCK_1US == 0x10 {
        write_reg8(0xf04, 0x5e);
    } else {
        write_reg8(0xf04, 0x68);
    }

    if state.slave_link_time_out * CLOCK_SYS_CLOCK_1US < read_reg_system_tick() - SLAVE_CONNECTED_TICK.get() {
        if RF_SLAVE_OTA_BUSY.get() {
            app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(OtaState::Error);
        }
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
        state.p_st_handler = IrqHandlerStatus::Adv;
        write_reg_dma_tx_rptr(0x10);
        SLAVE_LINK_CONNECTED.set(false);
        if state.not_need_login == false {
            PAIR_LOGIN_OK.set(false);
        }
        if SECURITY_ENABLE.get() == false {
            mesh_report_status_enable(false);
        } else {
            state.slave_first_connected_tick = 0;
            PAIR_LOGIN_OK.set(false);
            mesh_report_status_enable(false);
        }
        if state.slave_read_status_busy != 0 {
            rf_link_slave_read_status_stop(state);
        }
        if state.pair_setting_flag == ePairState::PairSetted {
            state.slave_data_valid = 0;
        } else {
            state.slave_data_valid = 0;
            pair_load_key(state);
            state.pair_setting_flag = ePairState::PairSetted;
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

    if RF_SLAVE_OTA_BUSY.get() {
        app().ota_manager.rf_link_slave_ota_finish_handle(state);

        if CLOCK_SYS_CLOCK_1US * 1000000 < read_reg_system_tick() - RF_SLAVE_OTA_TIMEOUT_TICK.load(Ordering::Relaxed) {
            RF_SLAVE_OTA_TIMEOUT_TICK.store(read_reg_system_tick(), Ordering::Relaxed);
            let sot = state.rf_slave_ota_timeout_s;
            if sot != 0 {
                state.rf_slave_ota_timeout_s = sot - 1;
                if sot - 1 == 0
                {
                    app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(OtaState::Error);
                }
            }
        }

        write_reg_system_tick_irq(SLAVE_NEXT_CONNECT_TICK.get());
        state.p_st_handler = IrqHandlerStatus::Rx;
        return;
    }

    state.t_bridge_cmd = CLOCK_SYS_CLOCK_1US * 200 + read_reg_system_tick();

    if CLOCK_SYS_CLOCK_1US * 10000 < SLAVE_LINK_INTERVAL.get() && state.slave_interval_old == 0 || state.slave_instant_next != state.slave_instant {
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
        let mut cur_interval = (SLAVE_NEXT_CONNECT_TICK.get() - (CLOCK_SYS_CLOCK_1US * 500)) - read_reg_system_tick();
        if state.slave_interval_old != 0 && state.slave_instant_next == state.slave_instant {
            if 0x3fffffff >= cur_interval {
                cur_interval = 0;
            }
            state.slave_interval_old = 0;
        }

        if cur_interval <= SLAVE_LINK_INTERVAL.get() {
            break;
        }

        SLAVE_NEXT_CONNECT_TICK.set(SLAVE_NEXT_CONNECT_TICK.get() + SLAVE_LINK_INTERVAL.get());

        slave_timing_update_handle(state);
        let chn_map = state.pkt_init.chm;
        ble_ll_conn_get_next_channel(state, &chn_map, state.pkt_init.hop & 0x1f);
    }

    write_reg_system_tick_irq(SLAVE_NEXT_CONNECT_TICK.get());
    state.p_st_handler = IrqHandlerStatus::Rx;
}

pub fn irq_st_ble_rx(state: &mut State)
{
    write_reg8(0x080050f, 0x80);
    rf_stop_trx();

    SLAVE_LINK_STATE.set(7);

    write_reg8(0x00800f04, 0x67);  // tx wail & settle time

    let chn_map = state.pkt_init.chm;
    let next_chan = ble_ll_conn_get_next_channel(state, &chn_map, state.pkt_init.hop & 0x1f) as u8;
    rf_set_ble_channel(next_chan);

    // rf_set_ble_access_code(unsafe { *(addr_of!((state.pkt_init()).aa) as *const u32) });
    // rf_set_ble_crc(&(state.pkt_init()).crcinit);
    write_reg_rf_access_code(((state.pkt_init.aa[2] as u32) << 8) | ((state.pkt_init.aa[1] as u32) << 0x10) | (state.pkt_init.aa[3] as u32) | ((state.pkt_init.aa[0] as u32) << 0x18));
    write_reg_rf_crc(((state.pkt_init.crcinit[1] as u32) << 8) | ((state.pkt_init.crcinit[2] as u32) << 0x10) | state.pkt_init.crcinit[0] as u32);

    SLAVE_NEXT_CONNECT_TICK.set(SLAVE_LINK_INTERVAL.get() + read_reg_system_tick_irq());
    if RF_SLAVE_OTA_BUSY.get() == false {
        if SLAVE_WINDOW_SIZE.get() == 0 || SLAVE_WINDOW_SIZE.get() <= CLOCK_SYS_CLOCK_1US * 0xed8 {
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 0xed8 + read_reg_system_tick());
        } else {
            write_reg_system_tick_irq(read_reg_system_tick() + SLAVE_WINDOW_SIZE.get());
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

#[inline(always)]
fn irq_light_slave_rx()
{
    static T_RX_LAST: AtomicU32 = AtomicU32::new(0);

    let rx_index = LIGHT_RX_WPTR.get();
    LIGHT_RX_WPTR.set((rx_index + 1) % LIGHT_RX_BUFF_COUNT);

    if read_reg_rf_rx_status() == 0x0b {
        write_reg_rf_irq_status(1);
        return;
    }

    let mut dma_len = 0;
    let binding = LIGHT_RX_BUFF.lock();
    let mut light_rx_buff = binding.borrow_mut();
    let mut light_rx_buff = light_rx_buff.deref_mut();

    write_reg_dma2_addr(addr_of!(light_rx_buff[LIGHT_RX_WPTR.get()]) as u16);
    write_reg_rf_irq_status(1);

    let rx_time = light_rx_buff[rx_index].rx_time;
    dma_len = light_rx_buff[rx_index].dma_len;

    light_rx_buff[rx_index].dma_len = 1;

    if dma_len == 1 {
        if T_RX_LAST.load(Ordering::Relaxed) == rx_time {
            rf_stop_trx();
            rf_start_stx2rx(addr_of!(PKT_EMPTY) as u32, CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick());
            return;
        }

        return;
    }

    T_RX_LAST.store(rx_time, Ordering::Relaxed);

    #[inline(never)]
    fn slow(rx_index: usize, dma_len: u8, light_rx_buff: &mut [LightRxBuff; 4]) {
        let binding = STATE.lock();
        let mut state = binding.borrow_mut();
        let mut state = state.deref_mut();

        let entry = &light_rx_buff[rx_index];
        let rx_time = entry.rx_time;

        if dma_len > 0xe && dma_len == (entry.sno[1] & 0x3f) + 0x11 && unsafe { *((addr_of!(*entry) as u32 + dma_len as u32 + 3) as *const u8) } & 0x51 == 0x40 {
            let packet = addr_of!(entry.rx_time);

            let cmd = entry.sno[0] & 0xf;
            RCV_PKT_TIME.set(rx_time);
            if SLAVE_LINK_STATE.get() == 1 {
                if cmd == 3 {
                    if entry.mac == state.mac_id[0..4] {
                        rf_stop_trx();
                        write_reg_rf_sched_tick(rx_time + T_SCAN_RSP_INTVL.get() * CLOCK_SYS_CLOCK_1US);
                        write_reg_rf_mode_control(0x85);                        // single TX

                        state.adv_rsp_pri_data.device_address = DEVICE_ADDRESS.get();
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
                }

                if cmd == 5 {
                    if entry.mac == state.mac_id[0..4] {
                        rf_link_slave_connect(state, unsafe { &*(packet as *const PacketLlInit) }, rx_time);

                        return;
                    }
                }
            }

            if !RF_SLAVE_OTA_BUSY.get() && SLAVE_LINK_STATE.get() != 7 {
                app().mesh_manager.add_rcv_mesh_msg(&unsafe { *(packet as *mut MeshPkt) }, true);

                rf_set_rxmode();
                return;
            }

            let master_sn = ((entry.sno[2] as u16) * 0x100) | ((entry.sno[0] >> 3) & 1) as u16;
            if LIGHT_CONN_SN_MASTER.get() == master_sn {
                rf_link_timing_adjust(state, rx_time);
            } else {
                LIGHT_CONN_SN_MASTER.set(master_sn);
                SLAVE_CONNECTED_TICK.set(read_reg_system_tick());
                SLAVE_LINK_CONNECTED.set(true);

                rf_link_slave_data(state, unsafe { &*(packet as *const PacketLlData) }, rx_time);
            }


            if SLAVE_WINDOW_SIZE.get() != 0 {
                if SLAVE_TIMING_UPDATE2_FLAG.get() {
                    if 0x40000001 > SLAVE_TIMING_UPDATE2_OK_TIME.get() - read_reg_system_tick() {
                        return;
                    }

                    SLAVE_TIMING_UPDATE2_FLAG.set(false);
                }
                SLAVE_WINDOW_SIZE.set(0);

                SLAVE_NEXT_CONNECT_TICK.set(rx_time + SLAVE_LINK_INTERVAL.get() - CLOCK_SYS_CLOCK_1US * 1250);
            }
            return;
        }

        if SLAVE_LINK_STATE.get() == 7 {
            write_reg8(0x80050f, 0);
            rf_stop_trx();
        }
    }

    slow(rx_index, dma_len, light_rx_buff);
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
#[link_section = ".ram_code"]
extern "C" fn irq_handler() {
    let _tracker = IrqTracker::new();

    let irq = read_reg_rf_irq_status();

    if irq & FLD_RF_IRQ_MASK::IRQ_TX as u16 != 0 {
        irq_light_slave_tx();
    }

    if irq & FLD_RF_IRQ_MASK::IRQ_RX as u16 != 0 {
        irq_light_slave_rx();
    }

    #[inline(never)]
    fn slow() {
        let binding = STATE.lock();
        let mut state = binding.borrow_mut();
        let mut state = state.deref_mut();

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

            if RF_SLAVE_OTA_BUSY_MESH.get() {
                let sot = state.rf_slave_ota_timeout_s;
                if sot != 0 {
                    state.rf_slave_ota_timeout_s = sot - 1;
                    if sot - 1 == 0
                    {
                        app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(state.rf_slave_ota_finished_flag);
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
    }

    slow();
}
