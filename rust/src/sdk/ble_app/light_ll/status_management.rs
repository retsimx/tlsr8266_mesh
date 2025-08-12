use core::cmp::min;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::{app};
use crate::sdk::ble_app::rf_drv_8266::{*};
use crate::sdk::light::{*};
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, sleep_us};
use crate::sdk::mcu::register::{read_reg_system_tick, read_reg_rnd_number};
use crate::state::{*};

use super::mesh_management::mesh_send_online_status;
use super::packet_processing::{rf_link_is_notify_req};

/// Initializes parameters for slave read status
#[cfg_attr(test, mry::mry)]
pub fn rf_link_slave_read_status_par_init()
{
    DEVICE_STATUS_BUFFER_WRITE_POINTER.set(0);
    DEVICE_STATUS_BUFFER_READ_POINTER.set(0);
    *STATUS_MESSAGE_SEQUENCE_NUMBER.lock() = [0; 3];
}

/// Stops slave read status operation
#[cfg_attr(test, mry::mry)]
pub fn rf_link_slave_read_status_stop()
{
    SLAVE_READ_STATUS_BUSY.set(0);
    DEVICE_STATUS_READ_UNICAST_MODE.set(false);
    SLAVE_DATA_VALID.set(0);

    rf_link_slave_read_status_par_init();
}

/// Handles bridge command processing
#[cfg_attr(test, mry::mry)]
pub fn app_bridge_cmd_handle(bridge_cmd_time: u32)
{
    let mut pkt_light_data = PKT_LIGHT_DATA.lock();

    if SLAVE_DATA_VALID.get() != 0 {
        SLAVE_DATA_VALID.dec();
        if SLAVE_DATA_VALID.get() == 0 {} else if SLAVE_READ_STATUS_BUSY.get() == 0 || SLAVE_DATA_VALID.get() as i32 > -1 {
            pkt_light_data.head_mut()._type |= crate::BIT!(7);

            if rf_link_is_notify_req(pkt_light_data.att_cmd().value.val[0] & 0x3f) {
                let mut relay_time = min(
                    (((read_reg_system_tick() - bridge_cmd_time) / CLOCK_SYS_CLOCK_1US) + 500) >> 10,
                    0xff,
                );

                pkt_light_data.att_cmd_mut().value.val[17] = relay_time as u8;
            }

            app().mesh_manager.add_send_mesh_msg(&*pkt_light_data, 0, 0);
        }
    }
}

/// Transmits packets for bridge operations
#[cfg_attr(test, mry::mry)]
pub fn tx_packet_bridge()
{
    static TICK_BRIDGE_REPORT: AtomicU32 = AtomicU32::new(0);

    rf_set_tx_rx_off();

    sleep_us(100);

    rf_set_ble_access_code(PAIR_AC.get());
    rf_set_ble_crc_adv();

    let tick = read_reg_system_tick();
    if MESH_LISTEN_INTERVAL_US.get() * ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL as u32 - ONLINE_STATUS_COMP * CLOCK_SYS_CLOCK_1US * 1000 < tick - TICK_BRIDGE_REPORT.load(Ordering::Relaxed) {
        TICK_BRIDGE_REPORT.store(tick, Ordering::Relaxed);
        mesh_send_online_status();
    }
    app_bridge_cmd_handle(BRIDGE_COMMAND_TIMESTAMP.get());
}
