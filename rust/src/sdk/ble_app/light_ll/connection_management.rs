use core::ptr::{addr_of};
use core::slice;

use crate::{app};
use crate::common::{rf_update_conn_para, SYS_CHN_LISTEN, update_ble_parameter_cb};
use crate::sdk::ble_app::ble_ll_channel_selection::ble_ll_build_available_channel_table;
use crate::sdk::ble_app::ble_ll_pair::{pair_init};
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::light::{*};
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US};
use crate::sdk::mcu::register::{*};
use crate::sdk::packet_types::{*};
use crate::state::{*};

/// Checks connection parameters for validity
fn check_par_con(packet: &Packet) -> bool
{
    if packet.ll_init().interval - 6 & 0xffff < 0xc7b && packet.ll_init().wsize != 0 && packet.ll_init().wsize < 9 && 9 < packet.ll_init().timeout && packet.ll_init().timeout < 0xc81 && packet.ll_init().woffset <= packet.ll_init().interval && packet.ll_init().hop != 0 &&
        packet.ll_init().chm.iter().any(|v| { *v != 0 }) {
        if packet.ll_init().latency == 0 {
            return false;
        }

        if packet.ll_init().latency as u32 <= ((packet.ll_init().interval as u32) << 3) / packet.ll_init().interval as u32 {
            return false;
        }
    }
    return true;
}

/// Handles slave connection setup
#[cfg_attr(test, mry::mry)]
pub fn rf_link_slave_connect(packet: &Packet, time: u32) -> bool
{
    CONN_UPDATE_SUCCESSED.set(false);
    CONN_UPDATE_CNT.set(0);

    if BLE_PERIPHERAL_CONNECTION_ENABLED.get() || packet.ll_init().scan_a == packet.ll_init().adv_a {
        if check_par_con(packet) == false {
            rf_stop_trx();

            BLE_PERIPHERAL_WINDOW_OFFSET.set(CLOCK_SYS_CLOCK_1US * 1250 * (packet.ll_init().woffset as u32 + 1));

            write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 1000 + read_reg_system_tick());
            write_reg_irq_src(0x100000);
            write_reg_system_tick_irq(time + BLE_PERIPHERAL_WINDOW_OFFSET.get() + CLOCK_SYS_CLOCK_1US * (0 - if packet.ll_init().woffset == 0 { 500 } else { 700 }));
            if 0x80000000 < read_reg_system_tick_irq() - read_reg_system_tick() {
                write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US * 10 + read_reg_system_tick());
            }

            LIGHT_CONN_SN_MASTER.set(0x80);
            SLAVE_CONNECTED_TICK.set(read_reg_system_tick());
            if SECURITY_ENABLE.get() {
                BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(SLAVE_CONNECTED_TICK.get());
            }

            DEVICE_STATUS_TICK_COUNTER.set(((packet.ll_init().interval as u32 * 5) >> 2) as u8);
            SLAVE_LINK_INTERVAL.set(packet.ll_init().interval as u32 * CLOCK_SYS_CLOCK_1US * 1250);
            SLAVE_WINDOW_SIZE.set((packet.ll_init().wsize as u32 * 1250 + 1100) * CLOCK_SYS_CLOCK_1US);

            let tmp = SLAVE_LINK_INTERVAL.get() - CLOCK_SYS_CLOCK_1US * 1250;
            if tmp <= SLAVE_WINDOW_SIZE.get() && SLAVE_WINDOW_SIZE.get() - tmp != 0 {
                SLAVE_WINDOW_SIZE.set(tmp);
            }

            BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(packet.ll_init().timeout as u32 * 10000);
            PKT_INIT.lock().ll_init_mut().clone_from(&PacketLlInit {
                dma_len: 0x24,
                _type: 0x5,
                rf_len: 0x22,
                ..*packet.ll_init()
            });

            let chn_map = PKT_INIT.lock().ll_init().chm;
            ble_ll_build_available_channel_table(&chn_map, true);

            // rf_set_ble_crc(&(state.PKT_INIT()).crcinit);
            let crcinit = PKT_INIT.lock().ll_init().crcinit;
            write_reg_rf_crc(((crcinit[1] as u32) << 8) | ((crcinit[2] as u32) << 0x10) | crcinit[0] as u32);
            rf_reset_sn();

            BLE_PERIPHERAL_CONNECTION_INSTANT.set(0);
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG.set(false);
            BLE_PERIPHERAL_PREVIOUS_CONNECTION_INTERVAL.set(0);
            BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_OK_TIME.set(0);
            SLAVE_WINDOW_SIZE_UPDATE.set(0);

            pair_init();

            MESH_NODE_REPORT_ENABLE.set(false);

            MESH_NODE_MASK.lock().fill(0);

            *CURRENT_RF_STATE.lock() = RfOperationState::Receiving;
            NEED_UPDATE_CONNECT_PARA.set(true);
            GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(read_reg_system_tick() | 1);

            write_reg8(0x00800f04, 0x67);  // tx wail & settle time

            return true;
        }
    }
    return false;
}

/// Handles timing adjustment for slave connections
#[cfg_attr(test, mry::mry)]
pub fn rf_link_timing_adjust(time: u32)
{
    if BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.get() {
        BLE_PERIPHERAL_TIMING_ADJUSTMENT_ENABLED.set(false);
        if time - BRIDGE_RECEIVE_TIMING_TICK.get() < CLOCK_SYS_CLOCK_1US * 700 {
            SLAVE_NEXT_CONNECT_TICK.set(SLAVE_NEXT_CONNECT_TICK.get() - CLOCK_SYS_CLOCK_1US * 200);
        } else if CLOCK_SYS_CLOCK_1US * 1100 < time - BRIDGE_RECEIVE_TIMING_TICK.get() {
            SLAVE_NEXT_CONNECT_TICK.set(SLAVE_NEXT_CONNECT_TICK.get() + CLOCK_SYS_CLOCK_1US * 200);
        }
    }
}

/// Enables or disables slave pairing
pub fn rf_link_slave_pairing_enable(enable: bool)
{
    BLE_PERIPHERAL_CONNECTION_ENABLED.set(enable);
    BLE_PERIPHERAL_ADVERTISING_ENABLED.set(enable);
}

/// Sets up BLE parameter updates
pub fn setup_ble_parameter_start(mut interval_min: u16, mut interval_max: u16, timeout: u32) -> u32
{
    let mut invalid = false;

    if interval_max | interval_min == 0 {
        UPDATE_INTERVAL_USER_MAX.set(SLAVE_LINK_INTERVAL.get() as u16);
        UPDATE_INTERVAL_USER_MIN.set(SLAVE_LINK_INTERVAL.get() as u16);
        interval_max = UPDATE_INTERVAL_USER_MAX.get();
        interval_min = UPDATE_INTERVAL_USER_MIN.get();
    } else {
        if interval_min < INTERVAL_THRESHOLD {
            invalid = true;
        }

        UPDATE_INTERVAL_USER_MIN.set(interval_min);
        UPDATE_INTERVAL_USER_MAX.set(interval_max);
    }

    if timeout < 100 {
        UPDATE_INTERVAL_USER_MAX.set(0);
        UPDATE_INTERVAL_USER_MIN.set(0);
        return 0xfffffffd;
    }

    if invalid == false {
        return 0;
    }

    UPDATE_INTERVAL_USER_MIN.set(0);
    UPDATE_INTERVAL_USER_MAX.set(0);
    return 0xfffffffe;
}

/// Updates connection parameters if needed
fn update_connect_para()
{
    if NEED_UPDATE_CONNECT_PARA.get() {
        if GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.get() != 0 {
            if UPDATE_CONNECT_PARA_DELAY_MS * CLOCK_SYS_CLOCK_1US * 1000 < read_reg_system_tick() - GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.get() {
                NEED_UPDATE_CONNECT_PARA.set(false);
                GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(0);

                update_ble_parameter_cb();
            }
        }
    }
}

/// Main slave processing function
pub fn rf_link_slave_proc() {
    app().mesh_manager.mesh_pair_proc();
    update_connect_para();
}

/// Checks system clock timing
pub fn light_check_tick_per_us(ticks: u32)
{
    if ticks == 0x10 {
        BLE_SCAN_RESPONSE_INTERVAL_US.set(0);
    } else if ticks == 0x20 || ticks != 0x30 {
        BLE_SCAN_RESPONSE_INTERVAL_US.set(0x92);
    } else {
        BLE_SCAN_RESPONSE_INTERVAL_US.set(0x93);
    }
}

/// Returns to receive mode for bridge operations
#[cfg_attr(test, mry::mry)]
pub fn back_to_rxmode_bridge()
{
    rf_set_tx_rx_off();
    rf_set_ble_access_code(PAIR_AC.get());
    rf_set_ble_crc_adv();
    rf_set_ble_channel(SYS_CHN_LISTEN[(BRIDGE_SEQUENCE_NUMBER.get() as usize % SYS_CHN_LISTEN.len()) >> 1]);
    rf_set_rxmode();
}
