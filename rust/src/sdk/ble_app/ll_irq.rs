use core::ptr::{addr_of};
use core::slice;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::{app};
use crate::common::{pair_load_key, SYS_CHN_ADV, SYS_CHN_LISTEN};
use crate::config::VENDOR_ID;
use crate::embassy::time_driver::clock_time64;
use crate::mesh::MESH_NODE_ST_VAL_LEN;
use crate::sdk::ble_app::ble_ll_channel_selection::{ble_ll_build_available_channel_table, ble_ll_select_next_data_channel};
use crate::sdk::ble_app::ble_ll_pair::{pair_dec_packet_mesh, pair_proc};
use crate::sdk::ble_app::light_ll::{*};
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::light::{*};
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, sleep_us};
use crate::sdk::mcu::register::{*};
use crate::sdk::packet_types::{Packet, PacketAttCmd, PacketAttValue, PacketL2capHead, PacketScanRsp, ScanRspData};
use crate::state::{*};

fn slave_timing_update_handle()
{
    SLAVE_INSTANT.inc();
    if SLAVE_TIMING_UPDATE.get() == 1 && SLAVE_INSTANT_NEXT.get() == SLAVE_INSTANT.get() {
        SLAVE_TIMING_UPDATE.set(0);
        let chn_map = SLAVE_CHN_MAP.lock().clone();
        ble_ll_build_available_channel_table(&chn_map, false);
        PKT_INIT.lock().ll_init_mut().chm = chn_map;
    } else {
        if SLAVE_TIMING_UPDATE.get() == 2 && SLAVE_INSTANT_NEXT.get() == SLAVE_INSTANT.get() {
            SLAVE_TIMING_UPDATE.set(0);
            SLAVE_TIMING_UPDATE2_OK_TIME.set(BLE_CONN_OFFSET.get() + SLAVE_NEXT_CONNECT_TICK.get());
            SLAVE_LINK_INTERVAL.set(BLE_CONN_INTERVAL.get());
            SLAVE_LINK_TIME_OUT.set(BLE_CONN_TIMEOUT.get());
            SLAVE_TIMING_UPDATE2_FLAG.set(true);
            SLAVE_WINDOW_SIZE.set(BLE_CONN_INTERVAL.get() - CLOCK_SYS_CLOCK_1US * 0x4e2);
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

fn rf_link_slave_read_status_update()
{
    let mut rptr = SLAVE_STATUS_BUFFER_RPTR.get();
    while SLAVE_STATUS_BUFFER_WPTR.get() != rptr {
        rptr = rptr % BUFF_RESPONSE_PACKET_COUNT;
        let packet = BUFF_RESPONSE.lock()[rptr];
        if !rf_link_add_tx_packet(&packet) {
            return;
        }

        rptr = ((SLAVE_STATUS_BUFFER_RPTR.get() + 1) % BUFF_RESPONSE_PACKET_COUNT);
        SLAVE_STATUS_BUFFER_RPTR.set(rptr);
    }
}

pub fn mesh_node_report_status(params: &mut [u8], len: usize) -> usize
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

    let mut mesh_node_mask = MESH_NODE_MASK.lock();
    let mut mesh_node_st = MESH_NODE_ST.lock();

    mesh_node_st.iter().enumerate().for_each(|(idx, val)| {
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
                        addr_of!(mesh_node_st[idx].val) as *const u8,
                        MESH_NODE_ST_VAL_LEN,
                    )
                }
            );

            // If the tick is 0 (Device offline), set the sn value to 0
            if mesh_node_st[idx].tick == 0 {
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

pub fn irq_st_listen()
{
    SLAVE_LINK_STATE.set(4);

    rf_stop_trx();

    app_bridge_cmd_handle(read_reg_system_tick());

    rf_set_rxmode_mesh_listen();

    write_reg_system_tick_irq(read_reg_system_tick() + SLAVE_LISTEN_INTERVAL.get());
    if ADV_FLAG.get() {
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 7000 + read_reg_system_tick());
        *P_ST_HANDLER.lock() = IrqHandlerStatus::Adv;
        return;
    }
    if !ONLINE_ST_FLAG.get() {
        ST_LISTEN_NO.inc();
        ADV_FLAG.set(ST_LISTEN_NO.get() % ADV_INTERVAL2LISTEN_INTERVAL as u32 != 0);
        ONLINE_ST_FLAG.set(ST_LISTEN_NO.get() % ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL as u32 != 0);
        if ADV_FLAG.get() {
            write_reg_system_tick_irq((((read_reg_system_tick() ^ read_reg_rnd_number() as u32) & 0x7fff) * CLOCK_SYS_CLOCK_1US + read_reg_system_tick_irq()));
            *P_ST_HANDLER.lock() = IrqHandlerStatus::Adv;
            return;
        }
        if !ONLINE_ST_FLAG.get() {
            return;
        }
    }

    *P_ST_HANDLER.lock() = IrqHandlerStatus::Adv;
}

fn get_gatt_adv_cnt() -> u32
{
    return 3;
}

pub fn irq_st_adv()
{
    static ST_PNO: AtomicU32 = AtomicU32::new(0);

    write_reg8(0x80050f, 0);
    rf_stop_trx();

    SLAVE_LINK_STATE.set(1);

    if !ADV_FLAG.get() || get_gatt_adv_cnt() <= ST_PNO.load(Ordering::Relaxed) {
        ADV_FLAG.set(false);
        ST_PNO.store(0, Ordering::Relaxed);
        if ONLINE_ST_FLAG.get() {
            mesh_send_online_status();
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
        } else {
            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 500 + read_reg_system_tick());
        }
        ONLINE_ST_FLAG.set(false);
        *P_ST_HANDLER.lock() = IrqHandlerStatus::Listen;
        write_reg_rf_irq_status(1);
    } else {
        rf_set_ble_access_code_adv();
        rf_set_ble_crc_adv();

        rf_set_ble_channel(SYS_CHN_ADV[ST_PNO.load(Ordering::Relaxed) as usize % 3]);
        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 0x4b0 + read_reg_system_tick());
        write_reg_rf_irq_status(1);

        if ST_PNO.load(Ordering::Relaxed) < 3 {
            let tmp = *PKT_ADV.lock();
            rf_start_stx2rx(addr_of!(tmp) as u32, CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick());
        }

        ST_PNO.store(ST_PNO.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
        if get_gatt_adv_cnt() <= ST_PNO.load(Ordering::Relaxed) {
            ST_PNO.store(0, Ordering::Relaxed);
            ADV_FLAG.set(false);
        }

        *P_ST_HANDLER.lock() = IrqHandlerStatus::Listen;
    }
}

fn disconnect_ble() {
    write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
    *P_ST_HANDLER.lock() = IrqHandlerStatus::Adv;
    write_reg_dma_tx_rptr(0x10);
    SLAVE_LINK_CONNECTED.set(false);
    PAIR_LOGIN_OK.set(false);

    if SECURITY_ENABLE.get() == false {
        mesh_report_status_enable(false);
    } else {
        SLAVE_FIRST_CONNECTED_TICK.set(0);
        PAIR_LOGIN_OK.set(false);
        mesh_report_status_enable(false);
    }
    if SLAVE_READ_STATUS_BUSY.get() != 0 {
        rf_link_slave_read_status_stop();
    }
    if *PAIR_SETTING_FLAG.lock() == ePairState::PairSetted {
        SLAVE_DATA_VALID.set(0);
    } else {
        SLAVE_DATA_VALID.set(0);
        pair_load_key();
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
    }
    if CLOCK_SYS_CLOCK_1US == 0x10 {
        write_reg8(0xf04, 0x5e);
    } else {
        write_reg8(0xf04, 0x68);
    }

    ATT_SERVICE_DISCOVER_TICK.set(0);
    NEED_UPDATE_CONNECT_PARA.set(false);
}

pub fn irq_st_bridge()
{
    static RF_SLAVE_OTA_TIMEOUT_TICK: AtomicU32 = AtomicU32::new(0);

    SLAVE_LINK_STATE.set(5);

    ST_BRIGE_NO.inc();
    write_reg8(0x50f, 0);
    rf_stop_trx();

    if CLOCK_SYS_CLOCK_1US == 0x10 {
        write_reg8(0xf04, 0x5e);
    } else {
        write_reg8(0xf04, 0x68);
    }

    if SLAVE_LINK_TIME_OUT.get() * CLOCK_SYS_CLOCK_1US < read_reg_system_tick() - SLAVE_CONNECTED_TICK.get() {
        if RF_SLAVE_OTA_BUSY.get() {
            app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(OtaState::Error);
        }

        // Disconnect
        disconnect_ble();

        return;
    }

    if SLAVE_READ_STATUS_BUSY.get() != 0 && SLAVE_READ_STATUS_BUSY_TIMEOUT * CLOCK_SYS_CLOCK_1US * 1000 < read_reg_system_tick() - SLAVE_READ_STATUS_BUSY_TIME.get() {
        rf_link_slave_read_status_stop();
    }

    if RF_SLAVE_OTA_BUSY.get() {
        app().ota_manager.rf_link_slave_ota_finish_handle();

        if CLOCK_SYS_CLOCK_1US * 1000000 < read_reg_system_tick() - RF_SLAVE_OTA_TIMEOUT_TICK.load(Ordering::Relaxed) {
            RF_SLAVE_OTA_TIMEOUT_TICK.store(read_reg_system_tick(), Ordering::Relaxed);
            if RF_SLAVE_OTA_TIMEOUT_S.get() != 0 {
                RF_SLAVE_OTA_TIMEOUT_S.dec();
                if RF_SLAVE_OTA_TIMEOUT_S.get() == 0 {
                    app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(OtaState::Error);
                }
            }
        }

        write_reg_system_tick_irq(SLAVE_NEXT_CONNECT_TICK.get());
        *P_ST_HANDLER.lock() = IrqHandlerStatus::Rx;
        return;
    }

    T_BRIDGE_CMD.set(CLOCK_SYS_CLOCK_1US * 200 + read_reg_system_tick());

    if CLOCK_SYS_CLOCK_1US * 10000 < SLAVE_LINK_INTERVAL.get() && SLAVE_INTERVAL_OLD.get() == 0 || SLAVE_INSTANT_NEXT.get() != SLAVE_INSTANT.get() {
        tx_packet_bridge();
    }

    if SLAVE_READ_STATUS_BUSY.get() != 0 {
        rf_link_slave_read_status_update();
    }

    if is_add_packet_buf_ready() {
        let pair_proc_result = pair_proc();
        if pair_proc_result.is_some() {
            let pkt = pair_proc_result.unwrap();
            rf_link_add_tx_packet(&pkt);
        }
    }
    mesh_node_flush_status();
    if is_add_packet_buf_ready() && !app().uart_manager.started() {
        let mut data = [0u8; 20];
        if mesh_node_report_status(&mut data, 10 / MESH_NODE_ST_VAL_LEN) != 0 {
            let mut pkt = Packet {
                att_cmd: PacketAttCmd {
                    head: PacketL2capHead {
                        dma_len: 0x1D,
                        _type: 2,
                        rf_len: 0x1B,
                        l2cap_len: 0x17,
                        chan_id: 4,
                    },
                    opcode: 0x1B,
                    handle: 0x12,
                    handle1: 0,
                    value: PacketAttValue {
                        sno: [0; 3],
                        src: [0; 2],
                        dst: [0; 2],
                        val: [0xdc, VENDOR_ID as u8, (VENDOR_ID >> 8) as u8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                    },
                }
            };

            pkt.att_cmd_mut().value.val[3..].copy_from_slice(&data);
            rf_link_add_tx_packet(&pkt);
        }
    }

    back_to_rxmode_bridge();

    loop {
        let mut cur_interval = (SLAVE_NEXT_CONNECT_TICK.get() - (CLOCK_SYS_CLOCK_1US * 500)) - read_reg_system_tick();
        if SLAVE_INTERVAL_OLD.get() != 0 && SLAVE_INSTANT_NEXT.get() == SLAVE_INSTANT.get() {
            if 0x3fffffff >= cur_interval {
                cur_interval = 0;
            }
            SLAVE_INTERVAL_OLD.set(0);
        }

        if cur_interval <= SLAVE_LINK_INTERVAL.get() {
            break;
        }

        SLAVE_NEXT_CONNECT_TICK.set(SLAVE_NEXT_CONNECT_TICK.get() + SLAVE_LINK_INTERVAL.get());

        slave_timing_update_handle();
        let chn_map = PKT_INIT.lock().ll_init().chm;
        ble_ll_select_next_data_channel(&chn_map, PKT_INIT.lock().ll_init().hop & 0x1f);
    }

    write_reg_system_tick_irq(SLAVE_NEXT_CONNECT_TICK.get());
    *P_ST_HANDLER.lock() = IrqHandlerStatus::Rx;
}

pub fn irq_st_ble_rx()
{
    write_reg8(0x080050f, 0x80);
    rf_stop_trx();

    SLAVE_LINK_STATE.set(7);

    write_reg8(0x00800f04, 0x67);  // tx wail & settle time

    let chn_map = PKT_INIT.lock().ll_init().chm;
    let next_chan = ble_ll_select_next_data_channel(&chn_map, PKT_INIT.lock().ll_init().hop & 0x1f) as u8;
    rf_set_ble_channel(next_chan);

    // rf_set_ble_access_code(unsafe { *(addr_of!((state.PKT_INIT()).aa) as *const u32) });
    // rf_set_ble_crc(&(state.PKT_INIT()).crcinit);
    let aa = PKT_INIT.lock().ll_init().aa;
    let crcinit = PKT_INIT.lock().ll_init().crcinit;
    write_reg_rf_access_code(((aa[2] as u32) << 8) | ((aa[1] as u32) << 0x10) | (aa[3] as u32) | ((aa[0] as u32) << 0x18));
    write_reg_rf_crc(((crcinit[1] as u32) << 8) | ((crcinit[2] as u32) << 0x10) | crcinit[0] as u32);

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
    *P_ST_HANDLER.lock() = IrqHandlerStatus::Bridge;
    SLAVE_TICK_BRX.set(CLOCK_SYS_CLOCK_1US * 100 + read_reg_system_tick());
    SLAVE_TIMING_ADJUST_ENABLE.set(true);
    rf_start_brx(addr_of!(PKT_EMPTY) as u32, SLAVE_TICK_BRX.get());
    sleep_us(2);

    slave_timing_update_handle();
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
    let mut light_rx_buff = LIGHT_RX_BUFF.lock();

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
        let entry = &light_rx_buff[rx_index];
        let rx_time = entry.rx_time;

        if dma_len > 0xe && dma_len == (entry.sno[1] & 0x3f) + 0x11 && unsafe { *((addr_of!(*entry) as u32 + dma_len as u32 + 3) as *const u8) } & 0x51 == 0x40 {
            let packet = unsafe { &*(addr_of!(entry.rx_time) as *const Packet) };

            let cmd = entry.sno[0] & 0xf;
            RCV_PKT_TIME.set(rx_time);
            if SLAVE_LINK_STATE.get() == 1 {
                if cmd == 3 {
                    if entry.mac == MAC_ID.lock()[0..4] {
                        rf_stop_trx();
                        write_reg_rf_sched_tick(rx_time + T_SCAN_RSP_INTVL.get() * CLOCK_SYS_CLOCK_1US);
                        write_reg_rf_mode_control(0x85);                        // single TX

                        let pkt_scan_rsp = Packet {
                            scan_rsp: PacketScanRsp {
                                dma_len: 0x27,
                                _type: 0x4,
                                rf_len: 0x25,
                                adv_a: *MAC_ID.lock(),
                                data: ScanRspData {
                                    handle: 0xff1e,
                                    data: AdvRspPrivate {
                                        device_address: DEVICE_ADDRESS.get(),
                                        ..*ADV_RSP_PRI_DATA.lock()
                                    }
                                },
                            }
                        };

                        write_reg_dma3_addr(addr_of!(pkt_scan_rsp) as u16);
                        write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 1000 + read_reg_system_tick_irq());

                        return;
                    }
                }

                if cmd == 5 {
                    if entry.mac == MAC_ID.lock()[0..4] {
                        rf_link_slave_connect(packet, rx_time);

                        return;
                    }
                }
            }

            if !RF_SLAVE_OTA_BUSY.get() && SLAVE_LINK_STATE.get() != 7 {
                let mut packet = *packet;

                let mut pkt_valid = || {
                    if SLAVE_LINK_CONNECTED.get() {
                        if 0x3fffffffi32 < (read_reg_system_tick_irq() as i32 - read_reg_system_tick() as i32) - (CLOCK_SYS_CLOCK_1US * 1000) as i32 {
                            return false;
                        }
                    }

                    if packet.head().rf_len != 0x25 || packet.head().l2cap_len != 0x21 || packet.head()._type & 3 != 2 || packet.head().chan_id == 0xeeff || !pair_dec_packet_mesh(&mut packet) {
                        return false;
                    }

                    // Check if this is a node update packet (pkt adv status)
                    if packet.head().chan_id == 0xffff {
                        return true;
                    }

                    // Parse the opcode and parameters from the packet
                    let (success, op_cmd, op_cmd_len, params, params_len) = parse_ble_packet_op_params(&packet, true);
                    if !success {
                        return false;
                    }

                    // Get the opcode
                    let mut op = 0;
                    if op_cmd_len == 3 {
                        op = op_cmd[0] & 0x3f;
                    }

                    // If we've already seen this packet, then there is nothing else to do
                    if is_exist_in_rc_pkt_buf(op, &packet) {
                        return false;
                    }

                    true
                };

                if pkt_valid() {
                    app().mesh_manager.add_rcv_mesh_msg(&packet);
                }

                return;
            }

            let master_sn = ((entry.sno[2] as u16) * 0x100) | ((entry.sno[0] >> 3) & 1) as u16;
            if LIGHT_CONN_SN_MASTER.get() == master_sn {
                rf_link_timing_adjust(rx_time);
            } else {
                LIGHT_CONN_SN_MASTER.set(master_sn);
                SLAVE_CONNECTED_TICK.set(read_reg_system_tick());
                SLAVE_LINK_CONNECTED.set(true);

                rf_link_slave_data(packet, rx_time);
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

    slow(rx_index, dma_len, &mut *light_rx_buff);
}

static IS_IRQ_MODE: AtomicBool = AtomicBool::new(false);

pub struct IrqTracker {}

impl IrqTracker {
    #[inline(always)]
    pub fn new() -> Self {
        IS_IRQ_MODE.store(true, Ordering::Relaxed);
        return IrqTracker {}
    }

    #[inline(always)]
    pub fn in_irq() -> bool {
        IS_IRQ_MODE.load(Ordering::Relaxed)
    }
}

impl Drop for IrqTracker {
    #[inline(always)]
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

    if irq & FLD_RF_IRQ_MASK::IRQ_TX.bits() != 0 {
        irq_light_slave_tx();
    }

    if irq & FLD_RF_IRQ_MASK::IRQ_RX.bits() != 0 {
        irq_light_slave_rx();
    }

    #[inline(never)]
    fn slow() {
        let irq_source = read_reg_irq_src();
        if irq_source & FLD_IRQ::SYSTEM_TIMER.bits() != 0 {
            write_reg_irq_src(FLD_IRQ::SYSTEM_TIMER.bits());
            let state = {
                *P_ST_HANDLER.lock()
            };
            match state {
                IrqHandlerStatus::Adv => irq_st_adv(),
                IrqHandlerStatus::Bridge => irq_st_bridge(),
                IrqHandlerStatus::Rx => irq_st_ble_rx(),
                IrqHandlerStatus::Listen => irq_st_listen(),
                IrqHandlerStatus::None => {}
            }
        }

        // This timer is configured to run once per second to check if the internal clock has overflowed.
        // this is a workaround in case there are no 'clock_time64' calls between overflows
        if irq_source & FLD_IRQ::TMR0_EN.bits() != 0 {
            write_reg_tmr_sta(FLD_TMR_STA::TMR0.bits());

            clock_time64();

            if RF_SLAVE_OTA_BUSY_MESH.get() {
                if RF_SLAVE_OTA_TIMEOUT_S.get() != 0 {
                    RF_SLAVE_OTA_TIMEOUT_S.dec();
                    if RF_SLAVE_OTA_TIMEOUT_S.get() == 0 {
                        app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(*RF_SLAVE_OTA_FINISHED_FLAG.lock());
                    }
                }
            }
        }

        // This timer triggers the transition step of the light so that the transition is smooth
        if irq_source & FLD_IRQ::TMR1_EN.bits() != 0 {
            write_reg_tmr_sta(FLD_TMR_STA::TMR1.bits());

            app().light_manager.transition_step();
        }

        app().uart_manager.check_irq();
    }

    slow();
}
