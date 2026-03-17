use core::cmp::min;
use core::ptr::addr_of;

#[cfg(not(test))]
use embassy_time::{Duration, Timer};
use fixed::types::I16F16;

use crate::app;
use crate::common::*;
use crate::config::*;
use crate::sdk::ble_app::light_ll::connection_management::{
    light_check_tick_per_us, rf_link_slave_pairing_enable, rf_link_slave_proc,
};
use crate::sdk::ble_app::light_ll::mesh_management::mesh_construct_packet;
use crate::sdk::ble_app::light_ll::packet_processing::parse_ble_packet_op_params;
use crate::sdk::ble_app::rf_drv_8266::{rf_link_slave_init, rf_set_power_level_index};
use crate::sdk::drivers::flash::{flash_erase_sector, flash_write_page};
use crate::sdk::drivers::pwm::{pwm_set_duty, pwm_start};
use crate::sdk::factory_reset::{
    factory_reset_cnt_check, factory_reset_handle, kick_out, KickoutReason,
};
use crate::sdk::light::*;
use crate::sdk::mcu::clock::{
    clock_time, clock_time_exceed, CLOCK_SYS_CLOCK_1S, CLOCK_SYS_CLOCK_1US,
};
use crate::sdk::mcu::gpio::{gpio_set_func, AS_GPIO};
use crate::sdk::mcu::irq_i::irq_disable;
use crate::sdk::mcu::register::{
    read_reg_irq_mask, read_reg_tmr_ctrl, write_reg_irq_mask, write_reg_tmr0_capt,
    write_reg_tmr0_tick, write_reg_tmr1_capt, write_reg_tmr_ctrl, FLD_IRQ, FLD_TMR,
};
use crate::sdk::packet_types::{Packet, PacketAttValue};
use crate::sdk::pm::{light_sw_reboot, usb_dp_pullup_en};
use crate::sdk::rf_drv::*;
use crate::state::*;
use crate::vendor_light::vendor_set_adv_data;
use crate::version::BUILD_VERSION;
use crate::{uprintln, BIT};
use heapless::Vec;

pub const LED_INDICATE_VAL: u16 = MAX_LUM_BRIGHTNESS_VALUE;
pub const LED_MASK: u8 = 0x07;

macro_rules! config_led_event {
    ($on:expr, $off:expr, $n:expr, $sel:expr) => {
        $on as u32 | ($off as u32) << 8 | ($n as u32) << 16 | ($sel as u32) << 24
    };
}

// pub const LED_EVENT_FLASH_4HZ_10S: u32 = config_led_event!(2,2,40,LED_MASK);
// pub const LED_EVENT_FLASH_STOP: u32 = config_led_event!(1,1,1,LED_MASK);
pub const LED_EVENT_FLASH_2HZ_2S: u32 = config_led_event!(4, 4, 4, LED_MASK);
// pub const LED_EVENT_FLASH_1HZ_1S: u32 = config_led_event!(8,8,1,LED_MASK);
// pub const LED_EVENT_FLASH_1HZ_2S: u32 = config_led_event!(8,8,2,LED_MASK);
// pub const LED_EVENT_FLASH_1HZ_3S: u32 = config_led_event!(8,8,3,LED_MASK);
pub const LED_EVENT_FLASH_1HZ_4S: u32 = config_led_event!(8, 8, 4, LED_MASK);
// pub const LED_EVENT_FLASH_4HZ: u32 = config_led_event!(2,2,0,LED_MASK);
// pub const LED_EVENT_FLASH_1HZ: u32 = config_led_event!(8,8,0,LED_MASK);
// pub const LED_EVENT_FLASH_4HZ_3T: u32 = config_led_event!(2,2,3,LED_MASK);
// pub const LED_EVENT_FLASH_1HZ_3T: u32 = config_led_event!(8,8,3,LED_MASK);
// pub const LED_EVENT_FLASH_0P25HZ_1T: u32 = config_led_event!(4, 60, 1, LED_MASK);

fn cfg_led_event(e: u32) {
    LED_CONTROLLER.event_pending.set(e);
}

#[cfg_attr(test, mry::mry)]
pub fn light_hw_timer_config() {
    // Enable timer1 interrupts for controlling light transitions
    write_reg_irq_mask(read_reg_irq_mask() | FLD_IRQ::TMR1_EN.bits());
    write_reg_tmr1_capt(CLOCK_SYS_CLOCK_1S / 120); // ~ 120hz

    // enable timer0 interrupt for tracking clock_time overflow
    write_reg_irq_mask(read_reg_irq_mask() | FLD_IRQ::TMR0_EN.bits());
    write_reg_tmr0_tick(0);
    write_reg_tmr0_capt(CLOCK_SYS_CLOCK_1S);

    write_reg_tmr_ctrl(read_reg_tmr_ctrl() | FLD_TMR::TMR0_EN.bits());
}

#[cfg_attr(test, mry::mry)]
fn light_init_default() {
    light_check_tick_per_us(CLOCK_SYS_CLOCK_1US);

    PAIR_CONFIG_MESH_NAME.lock().fill(0);
    let len = min(MESH_NAME.len(), MAX_MESH_NAME_LEN);
    PAIR_CONFIG_MESH_NAME.lock()[0..len].copy_from_slice(&MESH_NAME.as_bytes()[0..len]);

    PAIR_CONFIG_MESH_PWD.lock().fill(0);
    let len = min(MESH_PWD.len(), 16);
    PAIR_CONFIG_MESH_PWD.lock()[0..len].copy_from_slice(&MESH_PWD.as_bytes()[0..len]);

    PAIR_CONFIG_MESH_LTK.lock()[0..16].copy_from_slice(&MESH_LTK[0..16]);

    rf_link_slave_pairing_enable(true);
    rf_set_power_level_index(RfPower::Power8dBm as u32);

    usb_dp_pullup_en(true);

    light_hw_timer_config();

    app().mesh_manager.mesh_pair_init();
}

pub fn user_init() {
    // for app ota
    app().ota_manager.check_ota_area_startup();

    light_init_default();

    pwm_set_duty(PWMID_G, PMW_MAX_TICK, 0);
    pwm_set_duty(PWMID_B, PMW_MAX_TICK, PMW_MAX_TICK);

    pwm_start(PWMID_G);
    pwm_start(PWMID_B);

    gpio_set_func(PWM_G as u32, !AS_GPIO);
    gpio_set_func(PWM_B as u32, !AS_GPIO);

    //retrieve lumen value
    app().light_manager.light_lum_retrieve();

    rf_link_slave_init(40000);

    // Print MAC address now that it has been loaded from flash (reversed byte order)
    let mac = MAC_ID.lock();
    uprintln!(
        "MAC Address: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        mac[5],
        mac[4],
        mac[3],
        mac[2],
        mac[1],
        mac[0]
    );

    factory_reset_handle();

    vendor_set_adv_data();

    app().light_manager.device_status_update();
    app().mesh_manager.mesh_security_enable(true);
}

#[cfg_attr(test, mry::mry)]
fn proc_led() {
    if LED_CONTROLLER.blink_count.get() == 0 && LED_CONTROLLER.event_pending.get() == 0 {
        return; //led flash finished
    }

    if LED_CONTROLLER.event_pending.get() != 0 {
        // new event
        LED_CONTROLLER
            .on_duration_us
            .set((LED_CONTROLLER.event_pending.get() & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US);
        LED_CONTROLLER
            .off_duration_us
            .set(((LED_CONTROLLER.event_pending.get() >> 8) & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US);
        LED_CONTROLLER
            .blink_count
            .set((LED_CONTROLLER.event_pending.get() >> 16) & 0xff);
        LED_CONTROLLER
            .led_selection_mask
            .set(LED_CONTROLLER.event_pending.get() >> 24);

        LED_CONTROLLER.event_pending.set(0);
        LED_CONTROLLER.timing_tick.set(clock_time());
        LED_CONTROLLER.pattern_number.set(0);
        LED_CONTROLLER.is_on.set(0);
    }

    if clock_time() - LED_CONTROLLER.timing_tick.get()
        >= (if LED_CONTROLLER.is_on.get() != 0 {
            LED_CONTROLLER.on_duration_us.get()
        } else {
            LED_CONTROLLER.off_duration_us.get()
        })
    {
        LED_CONTROLLER.timing_tick.set(clock_time());
        let led_off = (LED_CONTROLLER.is_on.get() != 0 || LED_CONTROLLER.on_duration_us.get() == 0)
            && LED_CONTROLLER.off_duration_us.get() != 0;
        let led_on = LED_CONTROLLER.is_on.get() == 0 && LED_CONTROLLER.on_duration_us.get() != 0;

        LED_CONTROLLER
            .is_on
            .set(if LED_CONTROLLER.is_on.get() == 0 {
                1
            } else {
                0
            });
        if LED_CONTROLLER.is_on.get() != 0 {
            LED_CONTROLLER.pattern_number.inc();
            if LED_CONTROLLER.pattern_number.get() == LED_CONTROLLER.blink_count.get() {
                LED_CONTROLLER.blink_count.set(0);
                app()
                    .light_manager
                    .light_onoff_hw(!app().light_manager.is_light_off()); // should not report online status again
                return;
            }
        }

        if led_off || led_on {
            if LED_CONTROLLER.led_selection_mask.get() & BIT!(0) != 0 {
                app().light_manager.light_adjust_cw(
                    I16F16::from_num(LED_INDICATE_VAL * led_on as u16),
                    I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE),
                );
            }
            if LED_CONTROLLER.led_selection_mask.get() & BIT!(1) != 0 {
                app().light_manager.light_adjust_ww(
                    I16F16::from_num(LED_INDICATE_VAL * led_on as u16),
                    I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE),
                );
            }
            if LED_CONTROLLER.led_selection_mask.get() & BIT!(5) != 0 {}
        }
    }
}

fn light_auth_check() {
    if SECURITY_ENABLE.get()
        && !PAIR_LOGIN_OK.get()
        && BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.get() != 0
        && clock_time_exceed(
            BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.get(),
            AUTH_TIME * 1000 * 1000,
        )
    {
        //rf_link_slave_disconnect(); // must login in 60s after connected, if need
        BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(0);
    }
}

#[cfg_attr(test, mry::mry)]
fn light_user_func() {
    app().light_manager.check_light_state_save();

    light_auth_check();
    factory_reset_cnt_check();
    app().mesh_manager.mesh_pair_proc_effect();
}

pub async fn main_loop() {
    #[cfg(not(test))]
    Timer::after(Duration::from_micros(LOOP_INTERVAL_US)).await;

    light_user_func();
    rf_link_slave_proc();

    proc_led();
}

/*@brief: This function is called in IRQ state, use IRQ stack.
**@param: ppp: is pointer to response
**@param: p_req: is pointer to request command
Called to handle messages that require a response to be returned
*/
#[cfg_attr(test, mry::mry)]
pub fn rf_link_response_callback(ppp: &mut PacketAttValue, p_req: &PacketAttValue) -> bool {
    // mac-app[5] low 2 bytes used as ttc && hop-count
    // let dst_unicast = is_unicast_addr(&p_req.dst);
    ppp.dst = p_req.src;
    ppp.src[0] = (DEVICE_ADDRESS.get() & 0xff) as u8;
    ppp.src[1] = ((DEVICE_ADDRESS.get() >> 8) & 0xff) as u8;

    let params = &p_req.val[3..13];
    ppp.val[3..10 + 3].fill(0);

    ppp.val[1] = (VENDOR_ID & 0xFF) as u8;
    ppp.val[2] = ((VENDOR_ID >> 8) & 0xff) as u8;

    let group_address = GROUP_ADDRESS.lock();

    let mut idx = 0;
    match ppp.val[15] {
        GET_STATUS => {
            ppp.val[0] = LGT_CMD_LIGHT_STATUS | 0xc0;

            let state = app().light_manager.get_current_light_state();
            ppp.val[3] = (state.cw.to_num::<u16>() & 0xff) as u8;
            ppp.val[4] = ((state.cw.to_num::<u16>() >> 8) & 0xff) as u8;
            ppp.val[5] = (state.ww.to_num::<u16>() & 0xff) as u8;
            ppp.val[6] = ((state.ww.to_num::<u16>() >> 8) & 0xff) as u8;
            ppp.val[7] = (state.brightness.to_num::<u16>() & 0xff) as u8;
            ppp.val[8] = ((state.brightness.to_num::<u16>() >> 8) & 0xff) as u8;
        }
        GET_GROUP1 => {
            ppp.val[0] = LGT_CMD_LIGHT_GRP_RSP1 | 0xc0;
            for i in 0..MAX_GROUP_COUNT as usize {
                ppp.val[i + 3] = 0xFF;
                if group_address[i] != 0 {
                    ppp.val[idx + 3] = group_address[i] as u8;
                    idx += 1;
                }
            }
        }
        GET_GROUP2 => {
            ppp.val[0] = LGT_CMD_LIGHT_GRP_RSP2 | 0xc0;
            for i in 0..MAX_GROUP_COUNT as usize {
                ppp.val[i + 3] = 0xFF;
                if group_address[i / 2] != 0 {
                    ppp.val[idx + 3] = if (i % 2) != 0 {
                        (group_address[i / 2] >> 8) as u8
                    } else {
                        group_address[i / 2] as u8
                    };
                    idx += 1;
                }
            }
        }
        GET_GROUP3 => {
            ppp.val[0] = LGT_CMD_LIGHT_GRP_RSP3 | 0xc0;
            for i in 0..MAX_GROUP_COUNT as usize {
                ppp.val[i + 3] = 0xFF;
                if group_address[4 + i / 2] != 0 {
                    ppp.val[idx + 3] = if (i % 2) != 0 {
                        (group_address[4 + i / 2] >> 8) as u8
                    } else {
                        group_address[4 + i / 2] as u8
                    };
                    idx += 1;
                }
            }
        }
        GET_DEV_ADDR => {
            ppp.val[0] = LGT_CMD_DEV_ADDR_RSP | 0xc0;
            return dev_addr_with_mac_rsp(&mut ppp.val);
        }
        GET_USER_NOTIFY => {
            /*user can get parameters from APP.
            params[0] is relay times.
            params[1 -- 9] is parameters from APP if haved been set by user.

            dst_unicast == 1 means destination address is unicast address.
            */
            // if dst_unicast {
            //     // params[0 -- 9] is valid
            // } else {
            //     // only params[0 -- 4] is valid
            // }

            ppp.val[0] = LGT_CMD_USER_NOTIFY_RSP | 0xc0;
            for i in 0..8 {
                //params[2]
                ppp.val[5 + i] = i as u8;
            }
            ppp.val[3] = (DEVICE_ADDRESS.get() & 0xFF) as u8;
            ppp.val[4] = ((DEVICE_ADDRESS.get() >> 8) & 0xff) as u8;
        }
        CMD_START_OTA => {
            ppp.val[0] = LGT_CMD_START_OTA_RSP | 0xc0;

            ppp.val[3] = BUILD_VERSION as u8;
            ppp.val[4] = (BUILD_VERSION >> 8) as u8;
            ppp.val[5] = (BUILD_VERSION >> 16) as u8;
            ppp.val[6] = (BUILD_VERSION >> 24) as u8;

            OTA_UPDATE_MESH_OPERATIONS_BLOCKED.set(true);
        }
        CMD_OTA_DATA => {
            ppp.val[0] = LGT_CMD_OTA_DATA_RSP | 0xc0;

            let idx = app().ota_manager.rf_mesh_data_ota(params, false);

            ppp.val[1] = idx as u8;
            ppp.val[2] = (idx >> 8) as u8;
        }
        CMD_END_OTA => {
            ppp.val[0] = LGT_CMD_END_OTA_RSP | 0xc0;

            let idx = app().ota_manager.rf_mesh_data_ota(params, true);

            ppp.val[1] = idx as u8;
            ppp.val[2] = (idx >> 8) as u8;
        }
        _ => return false,
    }

    true
}

/*@brief: This function is called in IRQ state, use IRQ stack.
Called to handle messages sent to us that don't require a response
*/
#[cfg_attr(test, mry::mry)]
pub fn rf_link_data_callback(p: &Packet) {
    // p start from l2cap_len of RfPacketAttCmdT
    // Use the updated function to extract operation parameters
    let (_, op_cmd, op_cmd_len, params, _) = parse_ble_packet_op_params(p, true);
    if op_cmd_len != LightOpType::OpType3 as u8 {
        return;
    }

    // Verify that the vendor id is correct. This can catch unexpected corrupt messages
    let vendor_id = (op_cmd[2] as u16) << 8 | op_cmd[1] as u16;
    if vendor_id != VENDOR_ID {
        return;
    }

    let op = op_cmd[0] & 0x3F;

    match op {
        LGT_CMD_LIGHT_ONOFF => app()
            .light_manager
            .send_message(LGT_CMD_LIGHT_ONOFF, params),
        LGT_CMD_LIGHT_CONFIG_GRP => {
            let val = params[1] as u16 | ((params[2] as u16) << 8);
            match params[0] {
                LIGHT_DEL_GRP_PARAM if remove_group(val) => {
                    cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
                }
                LIGHT_ADD_GRP_PARAM if add_group(val) => {
                    cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
                }
                _ => (),
            }
        }
        LGT_CMD_CONFIG_DEV_ADDR => {
            let val = params[0] as u16 | ((params[1] as u16) << 8);
            if (!dev_addr_with_mac_flag(&params) || dev_addr_with_mac_match(&params))
                && add_device_address(val)
            {
                app()
                    .mesh_manager
                    .mesh_device_address_validation_completed();
            }
        }
        LGT_CMD_SET_LIGHT => app().light_manager.send_message(LGT_CMD_SET_LIGHT, params),
        LGT_CMD_SET_MAC_ADDR => {
            let mac = [
                params[0], params[1], params[2], params[3], params[4], params[5],
            ];
            flash_erase_sector(FLASH_ADR_MAC);
            flash_write_page(FLASH_ADR_MAC, mac.len() as u32, addr_of!(mac) as *const u8);
            light_sw_reboot();
        }
        LGT_CMD_KICK_OUT => {
            irq_disable();
            let res = (params[0] as u32).try_into();
            match res {
                Ok(res) => kick_out(res),
                Err(..) => kick_out(KickoutReason::OutOfMesh),
            }

            light_sw_reboot();
        }
        LGT_CMD_MESH_PAIR => app()
            .mesh_manager
            .mesh_pair_cb(&Vec::from_slice(&params[0..10]).unwrap()),
        _ => (),
    }
}

// p_cmd : cmd[3]+para[10]
// para    : dst
#[cfg_attr(test, mry::mry)]
pub fn light_slave_tx_command(
    p_cmd: &Vec<u8, 13>,
    para: u16,
    retransmit_count: u8,
    send_ack: bool,
) -> Packet {
    let mut cmd_op_para: [u8; 13] = [0; 13];
    let cmd_sno = clock_time() + DEVICE_ADDRESS.get() as u32;

    cmd_op_para[0..13].copy_from_slice(&p_cmd[0..13]);

    cmd_op_para[0] |= 0xc0;
    cmd_op_para[1] = (VENDOR_ID & 0xFF) as u8;
    cmd_op_para[2] = (VENDOR_ID >> 8) as u8;

    let dst = para;
    mesh_construct_packet(cmd_sno, dst, &cmd_op_para, retransmit_count, send_ack)
}

#[cfg_attr(test, mry::mry)]
pub fn rf_link_light_event_callback(status: u8) {
    match status {
        LGT_CMD_SET_MESH_INFO => {
            mesh_node_init();
            app().light_manager.device_status_update();
            cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
        }
        LGT_CMD_SET_DEV_ADDR => {
            mesh_node_init();
            app().light_manager.device_status_update();
            cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
        }
        LGT_CMD_DEL_PAIR => cfg_led_event(LED_EVENT_FLASH_1HZ_4S),
        LGT_CMD_MESH_PAIR_TIMEOUT => cfg_led_event(LED_EVENT_FLASH_2HZ_2S),
        _ => (),
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::app::App;
    use crate::common::{mock_dev_addr_with_mac_rsp, mock_mesh_node_init};
    use crate::sdk::ble_app::light_ll::connection_management::{
        mock_light_check_tick_per_us, mock_rf_link_slave_pairing_enable, mock_rf_link_slave_proc,
    };
    use crate::sdk::ble_app::light_ll::packet_processing::mock_parse_ble_packet_op_params;
    use crate::sdk::ble_app::rf_drv_8266::mock_rf_link_slave_init;
    use crate::sdk::ble_app::rf_drv_8266::mock_rf_set_power_level_index;
    use crate::sdk::drivers::flash::{mock_flash_erase_sector, mock_flash_write_page};
    use crate::sdk::drivers::pwm::{mock_pwm_set_duty, mock_pwm_start};
    use crate::sdk::factory_reset::mock_factory_reset_cnt_check;
    use crate::sdk::factory_reset::mock_factory_reset_handle;
    use crate::sdk::mcu::clock::{mock_clock_time, mock_clock_time_exceed, CLOCK_SYS_CLOCK_1US};
    use crate::sdk::mcu::gpio::mock_gpio_set_func;
    use crate::sdk::mcu::register::{
        mock_read_reg_irq_mask, mock_read_reg_tmr_ctrl, mock_write_reg_irq_mask,
        mock_write_reg_tmr0_capt, mock_write_reg_tmr0_tick, mock_write_reg_tmr1_capt,
        mock_write_reg_tmr_ctrl,
    };
    use crate::sdk::packet_types::{Packet, PacketAttValue, PacketL2capHead};
    use crate::sdk::pm::mock_light_sw_reboot;
    use crate::sdk::pm::mock_usb_dp_pullup_en;
    use crate::vendor_light::mock_vendor_set_adv_data;
    use crate::{app_mocker, mock_app_mocker};
    use futures::executor::block_on;
    use mry::Any;

    // --- Helper Functions ---

    /// Resets the LED_CONTROLLER to a clean state for testing
    fn reset_led_controller() {
        LED_CONTROLLER.event_pending.set(0);
        LED_CONTROLLER.blink_count.set(0);
        LED_CONTROLLER.on_duration_us.set(0);
        LED_CONTROLLER.off_duration_us.set(0);
        LED_CONTROLLER.led_selection_mask.set(0);
        LED_CONTROLLER.timing_tick.set(0);
        LED_CONTROLLER.pattern_number.set(0);
        LED_CONTROLLER.is_on.set(0);
    }

    /// Resets global state for testing
    fn reset_test_globals() {
        SECURITY_ENABLE.set(false);
        PAIR_LOGIN_OK.set(false);
        BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(0);
    }

    /// Creates a test PacketAttValue with default values
    fn create_test_packet_att_value() -> PacketAttValue {
        PacketAttValue {
            sno: [0; 3],
            src: [0x03, 0x04],
            dst: [0x01, 0x02],
            val: [0; 23],
        }
    }

    /// Creates a test Packet with default values
    fn create_test_packet() -> Packet {
        Packet {
            head: PacketL2capHead::default(),
        }
    }

    // --- Tests for cfg_led_event ---

    #[test]
    fn test_cfg_led_event_sets_pending_event() {
        // This test verifies that cfg_led_event correctly sets LED_CONTROLLER.event_pending

        // Setup: Clear any previous event
        LED_CONTROLLER.event_pending.set(0);

        // Execute: Set a new event
        let test_event = 0x12345678;
        cfg_led_event(test_event);

        // Verify: Event pending should match
        assert_eq!(
            LED_CONTROLLER.event_pending.get(),
            test_event,
            "Event pending should be set to the provided value"
        );
    }

    // --- Tests for light_hw_timer_config ---

    #[test]
    #[mry::lock(
        read_reg_irq_mask,
        write_reg_irq_mask,
        write_reg_tmr1_capt,
        write_reg_tmr0_tick,
        write_reg_tmr0_capt,
        read_reg_tmr_ctrl,
        write_reg_tmr_ctrl
    )]
    fn test_light_hw_timer_config_enables_timers() {
        // This test verifies that light_hw_timer_config properly enables TMR0 and TMR1 interrupts

        // Setup mocks
        let initial_irq_mask = 0x0000u32;
        let initial_tmr_ctrl = 0x0000u32;

        mock_read_reg_irq_mask().returns(initial_irq_mask);
        mock_read_reg_irq_mask().returns(initial_irq_mask); // Called twice
        mock_write_reg_irq_mask(Any).returns(());
        mock_write_reg_tmr1_capt(Any).returns(());
        mock_write_reg_tmr0_tick(Any).returns(());
        mock_write_reg_tmr0_capt(Any).returns(());
        mock_read_reg_tmr_ctrl().returns(initial_tmr_ctrl);
        mock_write_reg_tmr_ctrl(Any).returns(());

        // Execute
        light_hw_timer_config();

        // Verify TMR1 interrupt is enabled
        mock_write_reg_irq_mask(Any).assert_called(2);
        mock_write_reg_tmr1_capt(Any).assert_called(1);

        // Verify TMR0 is configured
        mock_write_reg_tmr0_tick(0).assert_called(1);
        mock_write_reg_tmr0_capt(Any).assert_called(1);

        // Verify TMR0 is enabled
        mock_write_reg_tmr_ctrl(Any).assert_called(1);
    }

    // --- Tests for proc_led ---

    #[test]
    fn test_proc_led_returns_early_when_no_event() {
        // Reset LED_CONTROLLER to clean state
        reset_led_controller();

        // Setup: No blink count and no pending event
        LED_CONTROLLER.blink_count.set(0);
        LED_CONTROLLER.event_pending.set(0);

        // Execute
        proc_led();

        // Verify: Nothing should happen (function returns early)
        assert_eq!(LED_CONTROLLER.blink_count.get(), 0);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_proc_led_initializes_new_event() {
        // Reset LED_CONTROLLER to clean state
        reset_led_controller();

        // Setup: Set a new event
        LED_CONTROLLER.event_pending.set(LED_EVENT_FLASH_2HZ_2S);
        LED_CONTROLLER.pattern_number.set(1);
        LED_CONTROLLER.blink_count.set(0);

        let test_time = 1000000u32;
        mock_clock_time().returns(test_time);

        // Execute
        proc_led();

        // Verify: Event was processed and state initialized
        assert_eq!(
            LED_CONTROLLER.event_pending.get(),
            0,
            "Event pending should be cleared"
        );
        assert_eq!(
            LED_CONTROLLER.blink_count.get(),
            4,
            "Blink count should be set from event"
        );
        assert_eq!(
            LED_CONTROLLER.led_selection_mask.get(),
            LED_MASK as u32,
            "LED mask should be set"
        );
        assert!(
            LED_CONTROLLER.on_duration_us.get() > 0,
            "On duration should be set"
        );
        assert!(
            LED_CONTROLLER.off_duration_us.get() > 0,
            "Off duration should be set"
        );
    }

    #[test]
    #[mry::lock(clock_time, app_mocker)]
    fn test_proc_led_toggles_led_state_on_timing() {
        // Reset LED_CONTROLLER to clean state
        reset_led_controller();

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        app.light_manager.mock_light_adjust_cw(Any, Any).returns(());
        app.light_manager.mock_light_adjust_ww(Any, Any).returns(());
        app.light_manager.mock_light_onoff_hw(Any).returns(());
        app.light_manager.mock_is_light_off().returns(false);

        // Set up LED controller with an active blink
        LED_CONTROLLER.blink_count.set(2);
        LED_CONTROLLER.event_pending.set(0);
        LED_CONTROLLER.on_duration_us.set(100 * CLOCK_SYS_CLOCK_1US);
        LED_CONTROLLER
            .off_duration_us
            .set(100 * CLOCK_SYS_CLOCK_1US);
        LED_CONTROLLER.led_selection_mask.set(LED_MASK as u32);
        LED_CONTROLLER.timing_tick.set(1000);
        LED_CONTROLLER.is_on.set(0);
        LED_CONTROLLER.pattern_number.set(0);

        // Simulate time passing enough to trigger LED on
        let new_time = LED_CONTROLLER.timing_tick.get() + LED_CONTROLLER.off_duration_us.get() + 1;
        mock_clock_time().returns(new_time);
        mock_clock_time().returns(new_time); // Called for timing check and update

        // Execute
        proc_led();

        // Verify: LED should turn on
        assert_eq!(LED_CONTROLLER.is_on.get(), 1, "LED should be on");
        app.light_manager
            .mock_light_adjust_cw(Any, Any)
            .assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time, app_mocker)]
    fn test_proc_led_continues_blink_sequence() {
        // Reset LED_CONTROLLER to clean state
        reset_led_controller();

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        app.light_manager.mock_light_adjust_cw(Any, Any).returns(());
        app.light_manager.mock_light_adjust_ww(Any, Any).returns(());
        app.light_manager.mock_light_onoff_hw(Any).returns(());
        app.light_manager.mock_is_light_off().returns(false);

        // Set up LED controller with a multi-blink sequence (blink_count = 3)
        // Pattern number starts at 0, so after increment it will be 1, which != 3
        LED_CONTROLLER.blink_count.set(3);
        LED_CONTROLLER.event_pending.set(0);
        LED_CONTROLLER.on_duration_us.set(100 * CLOCK_SYS_CLOCK_1US);
        LED_CONTROLLER
            .off_duration_us
            .set(100 * CLOCK_SYS_CLOCK_1US);
        LED_CONTROLLER.led_selection_mask.set(LED_MASK as u32);
        LED_CONTROLLER.timing_tick.set(0);
        LED_CONTROLLER.is_on.set(0);
        LED_CONTROLLER.pattern_number.set(0);

        // Simulate time passing enough to trigger LED on
        mock_clock_time().returns(100000);

        // Execute
        proc_led();

        // Verify: LED should turn on (line 160)
        assert_eq!(LED_CONTROLLER.is_on.get(), 1, "LED should be on");

        // Verify: Pattern number should be incremented (line 167)
        assert_eq!(
            LED_CONTROLLER.pattern_number.get(),
            1,
            "Pattern number should be incremented"
        );

        // Verify: Since pattern_number (1) != blink_count (3), it should fall through to line 180
        // and perform LED adjustment
        app.light_manager
            .mock_light_adjust_cw(Any, Any)
            .assert_called(1);

        // Verify: Blink sequence should continue (not complete)
        assert_eq!(
            LED_CONTROLLER.blink_count.get(),
            3,
            "Blink should not be complete"
        );
    }

    #[test]
    #[mry::lock(clock_time, app_mocker)]
    fn test_proc_led_completes_blink_sequence() {
        // Reset LED_CONTROLLER to clean state
        reset_led_controller();

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        app.light_manager.mock_light_onoff_hw(Any).returns(());
        app.light_manager.mock_is_light_off().returns(true);
        app.light_manager.mock_light_adjust_cw(Any, Any).returns(());
        app.light_manager.mock_light_adjust_ww(Any, Any).returns(());

        // Set up for last blink transition
        LED_CONTROLLER.blink_count.set(1);
        LED_CONTROLLER.event_pending.set(0);
        LED_CONTROLLER.on_duration_us.set(100 * CLOCK_SYS_CLOCK_1US);
        LED_CONTROLLER
            .off_duration_us
            .set(100 * CLOCK_SYS_CLOCK_1US);
        LED_CONTROLLER.led_selection_mask.set(LED_MASK as u32);
        LED_CONTROLLER.timing_tick.set(1000);
        LED_CONTROLLER.is_on.set(0);
        LED_CONTROLLER.pattern_number.set(0);

        // Simulate time passing
        let new_time = LED_CONTROLLER.timing_tick.get() + LED_CONTROLLER.off_duration_us.get() + 1;
        mock_clock_time().returns(new_time);
        mock_clock_time().returns(new_time);

        // Execute
        proc_led();

        // Verify: Pattern number incremented and blink completed
        assert_eq!(LED_CONTROLLER.pattern_number.get(), 1);
        assert_eq!(
            LED_CONTROLLER.blink_count.get(),
            0,
            "Blink should be complete"
        );
        app.light_manager
            .mock_light_onoff_hw(false)
            .assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time, app_mocker)]
    fn test_proc_led_adjusts_ww_light() {
        // Reset LED_CONTROLLER to clean state
        reset_led_controller();

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        app.light_manager.mock_light_adjust_cw(Any, Any).returns(());
        app.light_manager.mock_light_adjust_ww(Any, Any).returns(());
        app.light_manager.mock_light_onoff_hw(Any).returns(());
        app.light_manager.mock_is_light_off().returns(false);

        // Set up LED controller with BIT!(1) set (warm white)
        // Key: to reach the led_on branch, is_on must be 0, and on_duration_us must be non-zero
        // The led_on condition: is_on == 0 && on_duration_us != 0
        LED_CONTROLLER.blink_count.set(2);
        LED_CONTROLLER.event_pending.set(0);
        LED_CONTROLLER.on_duration_us.set(100 * CLOCK_SYS_CLOCK_1US);
        LED_CONTROLLER
            .off_duration_us
            .set(100 * CLOCK_SYS_CLOCK_1US);
        LED_CONTROLLER.led_selection_mask.set(0x02); // Only BIT!(1) set for warm white
        LED_CONTROLLER.timing_tick.set(1000);
        LED_CONTROLLER.is_on.set(0); // Start with is_on = 0 for led_on to be true
        LED_CONTROLLER.pattern_number.set(0);

        // Simulate time passing enough to trigger the LED state change
        // Timing condition: clock_time() - timing_tick >= (is_on != 0 ? on_duration : off_duration)
        // Since is_on=0, we need: clock_time() - 1000 >= off_duration
        let new_time = LED_CONTROLLER.timing_tick.get() + LED_CONTROLLER.off_duration_us.get() + 1;
        mock_clock_time().returns(new_time);
        mock_clock_time().returns(new_time); // Called again for timing_tick update

        // Execute
        proc_led();

        // Verify: light_adjust_ww should be called (covers line 188 with BIT!(1) check)
        // This happens because led_on = true (is_on was 0 and on_duration_us != 0)
        app.light_manager
            .mock_light_adjust_ww(Any, Any)
            .assert_called(1);

        // Verify: light_adjust_cw should NOT be called since BIT!(0) is not set
        app.light_manager
            .mock_light_adjust_cw(Any, Any)
            .assert_called(0);

        // Cleanup
        reset_led_controller();
    }

    #[test]
    #[mry::lock(clock_time, app_mocker)]
    fn test_proc_led_adjusts_led_bit5() {
        // Reset LED_CONTROLLER to clean state
        reset_led_controller();

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        app.light_manager.mock_light_adjust_cw(Any, Any).returns(());
        app.light_manager.mock_light_adjust_ww(Any, Any).returns(());
        app.light_manager.mock_light_onoff_hw(Any).returns(());
        app.light_manager.mock_is_light_off().returns(false);

        // Set up LED controller with BIT!(5) set (covers line 196)
        // Note: on_duration_us = 0 means led_on will be false and led_off will be true
        // We need led_off or led_on to be true to enter the adjustment section
        LED_CONTROLLER.blink_count.set(2);
        LED_CONTROLLER.event_pending.set(0);
        LED_CONTROLLER.on_duration_us.set(0); // on_duration = 0 makes led_off true
        LED_CONTROLLER
            .off_duration_us
            .set(100 * CLOCK_SYS_CLOCK_1US);
        LED_CONTROLLER.led_selection_mask.set(0x20); // Only BIT!(5) set
        LED_CONTROLLER.timing_tick.set(1000);
        LED_CONTROLLER.is_on.set(1); // Start with is_on = 1
        LED_CONTROLLER.pattern_number.set(0);

        // Simulate time passing enough to trigger the LED state change
        // Timing condition: clock_time() - timing_tick >= on_duration (since is_on=1)
        let new_time = LED_CONTROLLER.timing_tick.get()
            + LED_CONTROLLER.on_duration_us.get()
            + 100 * CLOCK_SYS_CLOCK_1US;
        mock_clock_time().returns(new_time);
        mock_clock_time().returns(new_time);

        // Execute
        proc_led();

        // Verify: Neither light_adjust_cw nor light_adjust_ww should be called
        // since only BIT!(5) is set (covers line 196 with the empty block)
        // led_off will be true: (is_on != 0 || on_duration == 0) && off_duration != 0
        // = (1 || true) && true = true
        // So we enter the if led_off || led_on block, but only BIT!(5) is handled (empty)
        app.light_manager
            .mock_light_adjust_cw(Any, Any)
            .assert_called(0);
        app.light_manager
            .mock_light_adjust_ww(Any, Any)
            .assert_called(0);

        // Cleanup
        reset_led_controller();
    }

    #[test]
    #[mry::lock(clock_time, app_mocker)]
    fn test_proc_led_skips_adjustment_when_both_durations_zero() {
        // Reset LED_CONTROLLER to clean state
        reset_led_controller();

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        app.light_manager.mock_light_adjust_cw(Any, Any).returns(());
        app.light_manager.mock_light_adjust_ww(Any, Any).returns(());
        app.light_manager.mock_light_onoff_hw(Any).returns(());
        app.light_manager.mock_is_light_off().returns(false);

        // Set up LED controller where both on_duration and off_duration are 0
        // This makes both led_on and led_off false, so the if condition at line 182 is false
        // and we skip all adjustment code (lines 184-196)
        LED_CONTROLLER.blink_count.set(2);
        LED_CONTROLLER.event_pending.set(0);
        LED_CONTROLLER.on_duration_us.set(0); // Both durations are 0
        LED_CONTROLLER.off_duration_us.set(0); // This makes led_off = (1 || 1) && 0 = false
        LED_CONTROLLER.led_selection_mask.set(0x07); // All bits set (doesn't matter)
        LED_CONTROLLER.timing_tick.set(1000);
        LED_CONTROLLER.is_on.set(1);
        LED_CONTROLLER.pattern_number.set(0);

        // Simulate time passing - timing condition will be true since 0 - 0 >= 0
        let new_time = LED_CONTROLLER.timing_tick.get() + 1;
        mock_clock_time().returns(new_time);
        mock_clock_time().returns(new_time);

        // Execute
        proc_led();

        // Verify: No light adjustments should be called because:
        // led_on = (is_on == 0 && on_duration != 0) = (1 == 0 && 0 != 0) = false
        // led_off = ((is_on != 0 || on_duration == 0) && off_duration != 0) = ((1 || 1) && 0) = false
        // So the if (led_off || led_on) condition at line 182 is false, skipping all adjustments
        app.light_manager
            .mock_light_adjust_cw(Any, Any)
            .assert_called(0);
        app.light_manager
            .mock_light_adjust_ww(Any, Any)
            .assert_called(0);
    }

    // --- Tests for light_auth_check ---

    #[test]
    #[mry::lock(clock_time_exceed)]
    fn test_light_auth_check_clears_timestamp_on_timeout() {
        // This test verifies authentication timeout when security is enabled

        // Setup: Reset global state first
        reset_test_globals();

        // Setup: Security enabled, not logged in, and timeout exceeded
        SECURITY_ENABLE.set(true);
        PAIR_LOGIN_OK.set(false);
        BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(1000000);

        mock_clock_time_exceed(Any, Any).returns(true);

        // Execute
        light_auth_check();

        // Verify: Timestamp should be cleared
        assert_eq!(
            BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.get(),
            0,
            "Timestamp should be cleared after timeout"
        );
    }

    #[test]
    fn test_light_auth_check_no_action_when_logged_in() {
        // This test verifies no action when already logged in

        // Setup: Reset global state first
        reset_test_globals();

        // Setup: Security enabled and logged in
        SECURITY_ENABLE.set(true);
        PAIR_LOGIN_OK.set(true);
        BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(1000000);

        // Execute
        light_auth_check();

        // Verify: Function should not crash (logged in means condition check short-circuits)
        // Since PAIR_LOGIN_OK is true, the whole condition is false and function returns early
        assert_eq!(
            BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.get(),
            1000000,
            "Timestamp should remain unchanged"
        );
    }

    #[test]
    fn test_light_auth_check_no_action_when_security_disabled() {
        // This test verifies no action when security is disabled

        // Setup: Reset global state first
        reset_test_globals();

        // Setup: Security disabled
        SECURITY_ENABLE.set(false);
        PAIR_LOGIN_OK.set(false);
        BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(1000000);

        // Execute
        light_auth_check();

        // Verify: Timestamp should remain unchanged
        assert_eq!(BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.get(), 1000000);
    }

    // --- Tests for rf_link_response_callback ---

    #[test]
    #[mry::lock(app_mocker)]
    fn test_rf_link_response_callback_get_status() {
        // This test verifies GET_STATUS response returns current light state

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);

        DEVICE_ADDRESS.set(0x1234);

        let mut request = create_test_packet_att_value();
        request.src = [0x56, 0x78];
        request.val[15] = GET_STATUS;

        let mut response = create_test_packet_att_value();

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(result, "Callback should return true for GET_STATUS");
        assert_eq!(
            response.dst, request.src,
            "Destination should be request source"
        );
        assert_eq!(response.val[0], LGT_CMD_LIGHT_STATUS | 0xc0)
    }

    #[test]
    fn test_rf_link_response_callback_get_group1() {
        // This test verifies GET_GROUP1 response returns group addresses

        // Setup
        DEVICE_ADDRESS.set(0x1234);
        let mut group_address = GROUP_ADDRESS.lock();
        group_address[0] = 0x11;
        group_address[1] = 0x22;
        group_address[2] = 0x33;
        for i in 3..MAX_GROUP_COUNT as usize {
            group_address[i] = 0;
        }
        drop(group_address);

        let mut request = create_test_packet_att_value();
        request.src = [0x56, 0x78];

        let mut response = create_test_packet_att_value();
        response.val[15] = GET_GROUP1;

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(result, "Callback should return true for GET_GROUP1");
        assert_eq!(response.val[0], LGT_CMD_LIGHT_GRP_RSP1 | 0xc0);
        assert_eq!(response.val[3], 0x11, "First group address");
        assert_eq!(response.val[4], 0x22, "Second group address");
        assert_eq!(response.val[5], 0x33, "Third group address");
    }

    #[test]
    fn test_rf_link_response_callback_get_group2() {
        // This test verifies GET_GROUP2 response returns group addresses (2-byte format)

        // Setup
        DEVICE_ADDRESS.set(0x1234);
        let mut group_address = GROUP_ADDRESS.lock();
        group_address[0] = 0xAABB;
        group_address[1] = 0xCCDD;
        for i in 2..MAX_GROUP_COUNT as usize {
            group_address[i] = 0;
        }
        drop(group_address);

        let mut request = create_test_packet_att_value();
        request.src = [0x56, 0x78];

        let mut response = create_test_packet_att_value();
        response.val[15] = GET_GROUP2;

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(result, "Callback should return true for GET_GROUP2");
        assert_eq!(response.val[0], LGT_CMD_LIGHT_GRP_RSP2 | 0xc0);
        assert_eq!(response.val[3], 0xBB, "First group low byte");
        assert_eq!(response.val[4], 0xAA, "First group high byte");
        assert_eq!(response.val[5], 0xDD, "Second group low byte");
        assert_eq!(response.val[6], 0xCC, "Second group high byte");
    }

    #[test]
    fn test_rf_link_response_callback_get_group3() {
        // This test verifies GET_GROUP3 response returns group addresses (last 4 groups)

        // Setup
        DEVICE_ADDRESS.set(0x1234);
        let mut group_address = GROUP_ADDRESS.lock();
        for i in 0..4 {
            group_address[i] = 0;
        }
        group_address[4] = 0x1122;
        group_address[5] = 0x3344;
        for i in 6..MAX_GROUP_COUNT as usize {
            group_address[i] = 0;
        }
        drop(group_address);

        let mut request = create_test_packet_att_value();
        request.src = [0x56, 0x78];

        let mut response = create_test_packet_att_value();
        response.val[15] = GET_GROUP3;

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(result, "Callback should return true for GET_GROUP3");
        assert_eq!(response.val[0], LGT_CMD_LIGHT_GRP_RSP3 | 0xc0);
        assert_eq!(response.val[3], 0x22, "Fifth group low byte");
        assert_eq!(response.val[4], 0x11, "Fifth group high byte");
    }

    #[test]
    #[mry::lock(dev_addr_with_mac_rsp)]
    fn test_rf_link_response_callback_get_dev_addr() {
        // This test verifies GET_DEV_ADDR response

        // Setup
        DEVICE_ADDRESS.set(0x1234);
        mock_dev_addr_with_mac_rsp(Any).returns(true);

        let mut request = create_test_packet_att_value();
        request.src = [0x56, 0x78];

        let mut response = create_test_packet_att_value();
        response.val[15] = GET_DEV_ADDR;

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(result, "Callback should return true for GET_DEV_ADDR");
        assert_eq!(response.val[0], LGT_CMD_DEV_ADDR_RSP | 0xc0);
        mock_dev_addr_with_mac_rsp(Any).assert_called(1);
    }

    #[test]
    fn test_rf_link_response_callback_get_user_notify() {
        // This test verifies GET_USER_NOTIFY response

        // Setup
        DEVICE_ADDRESS.set(0xABCD);

        let mut request = create_test_packet_att_value();
        request.src = [0x56, 0x78];

        let mut response = create_test_packet_att_value();
        response.val[15] = GET_USER_NOTIFY;

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(result, "Callback should return true for GET_USER_NOTIFY");
        assert_eq!(response.val[0], LGT_CMD_USER_NOTIFY_RSP | 0xc0);
        assert_eq!(response.val[3], 0xCD); // Device address low byte
        assert_eq!(response.val[4], 0xAB); // Device address high byte
                                           // Verify some response parameters
        for i in 0..8 {
            assert_eq!(response.val[5 + i], i as u8);
        }
    }

    #[test]
    #[mry::lock(app_mocker)]
    fn test_rf_link_response_callback_cmd_start_ota() {
        // This test verifies CMD_START_OTA response

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);

        DEVICE_ADDRESS.set(0x1234);
        OTA_UPDATE_MESH_OPERATIONS_BLOCKED.set(false);

        let mut request = create_test_packet_att_value();
        request.src = [0x56, 0x78];

        let mut response = create_test_packet_att_value();
        response.val[15] = CMD_START_OTA;

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(result, "Callback should return true for CMD_START_OTA");
        assert_eq!(response.val[0], LGT_CMD_START_OTA_RSP | 0xc0);
        assert_eq!(response.val[3], (BUILD_VERSION & 0xFF) as u8);
        assert_eq!(response.val[4], ((BUILD_VERSION >> 8) & 0xFF) as u8);
        assert_eq!(response.val[5], ((BUILD_VERSION >> 16) & 0xFF) as u8);
        assert_eq!(response.val[6], ((BUILD_VERSION >> 24) & 0xFF) as u8);
        assert!(
            OTA_UPDATE_MESH_OPERATIONS_BLOCKED.get(),
            "OTA operations should be blocked"
        );
    }

    #[test]
    #[mry::lock(app_mocker)]
    fn test_rf_link_response_callback_cmd_ota_data() {
        // This test verifies CMD_OTA_DATA response

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        app.ota_manager
            .mock_rf_mesh_data_ota(Any, Any)
            .returns(0x1234u16);

        DEVICE_ADDRESS.set(0x5678);

        let mut request = create_test_packet_att_value();
        request.src = [0xAA, 0xBB];

        let mut response = create_test_packet_att_value();
        response.val[15] = CMD_OTA_DATA;

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(result, "Callback should return true for CMD_OTA_DATA");
        assert_eq!(response.val[0], LGT_CMD_OTA_DATA_RSP | 0xc0);
        assert_eq!(response.val[1], 0x34); // idx low byte
        assert_eq!(response.val[2], 0x12); // idx high byte
        app.ota_manager
            .mock_rf_mesh_data_ota(Any, false)
            .assert_called(1);
    }

    #[test]
    #[mry::lock(app_mocker)]
    fn test_rf_link_response_callback_cmd_end_ota() {
        // This test verifies CMD_END_OTA response

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        app.ota_manager
            .mock_rf_mesh_data_ota(Any, Any)
            .returns(0xABCDu16);

        DEVICE_ADDRESS.set(0x5678);

        let mut request = create_test_packet_att_value();
        request.src = [0xCC, 0xDD];

        let mut response = create_test_packet_att_value();
        response.val[15] = CMD_END_OTA;

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(result, "Callback should return true for CMD_END_OTA");
        assert_eq!(response.val[0], LGT_CMD_END_OTA_RSP | 0xc0);
        assert_eq!(response.val[1], 0xCD); // idx low byte
        assert_eq!(response.val[2], 0xAB); // idx high byte
        app.ota_manager
            .mock_rf_mesh_data_ota(Any, true)
            .assert_called(1);
    }

    #[test]
    fn test_rf_link_response_callback_invalid_command() {
        // This test verifies that invalid commands return false

        // Setup
        DEVICE_ADDRESS.set(0x1234);

        let mut request = create_test_packet_att_value();

        let mut response = create_test_packet_att_value();
        response.val[15] = 0xFF; // Invalid command

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(!result, "Callback should return false for invalid command");
    }

    // --- Tests for rf_link_data_callback ---

    #[test]
    #[mry::lock(parse_ble_packet_op_params, app_mocker)]
    fn test_rf_link_data_callback_light_onoff() {
        // This test verifies LGT_CMD_LIGHT_ONOFF is handled correctly

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        app.light_manager.mock_send_message(Any, Any).returns(());

        let packet = create_test_packet();
        let op_cmd = [
            LGT_CMD_LIGHT_ONOFF,
            (VENDOR_ID & 0xFF) as u8,
            (VENDOR_ID >> 8) as u8,
        ];
        let params = [LIGHT_ON_PARAM, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        mock_parse_ble_packet_op_params(Any, Any).returns((
            false,
            op_cmd,
            LightOpType::OpType3 as u8,
            params,
            0,
        ));

        // Execute
        rf_link_data_callback(&packet);

        // Verify
        app.light_manager
            .mock_send_message(LGT_CMD_LIGHT_ONOFF, Any)
            .assert_called(1);
    }

    #[test]
    #[mry::lock(parse_ble_packet_op_params, remove_group)]
    fn test_rf_link_data_callback_light_config_grp_delete() {
        // This test verifies group deletion via LGT_CMD_LIGHT_CONFIG_GRP

        // Setup
        let packet = create_test_packet();
        let op_cmd = [
            LGT_CMD_LIGHT_CONFIG_GRP,
            (VENDOR_ID & 0xFF) as u8,
            (VENDOR_ID >> 8) as u8,
        ];
        let params = [
            LIGHT_DEL_GRP_PARAM,
            0x12,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];

        mock_parse_ble_packet_op_params(Any, Any).returns((
            false,
            op_cmd,
            LightOpType::OpType3 as u8,
            params,
            0,
        ));
        mock_remove_group(Any).returns(true);

        // Execute
        rf_link_data_callback(&packet);

        // Verify
        mock_remove_group(0x12).assert_called(1);
    }

    #[test]
    #[mry::lock(parse_ble_packet_op_params, remove_group)]
    fn test_rf_link_data_callback_light_config_grp_delete_no_flash() {
        // This test verifies group deletion that returns false doesn't trigger LED event

        // Setup
        let packet = create_test_packet();
        let op_cmd = [
            LGT_CMD_LIGHT_CONFIG_GRP,
            (VENDOR_ID & 0xFF) as u8,
            (VENDOR_ID >> 8) as u8,
        ];
        let params = [
            LIGHT_DEL_GRP_PARAM,
            0x12,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];

        mock_parse_ble_packet_op_params(Any, Any).returns((
            false,
            op_cmd,
            LightOpType::OpType3 as u8,
            params,
            0,
        ));
        mock_remove_group(Any).returns(false);

        // Clear LED event before test
        LED_CONTROLLER.event_pending.set(0);

        // Execute
        rf_link_data_callback(&packet);

        // Verify: No LED event should be set
        assert_eq!(LED_CONTROLLER.event_pending.get(), 0);

        // Cleanup
        LED_CONTROLLER.event_pending.set(0);
    }

    #[test]
    #[mry::lock(parse_ble_packet_op_params, add_group)]
    fn test_rf_link_data_callback_light_config_grp_add() {
        // This test verifies group addition via LGT_CMD_LIGHT_CONFIG_GRP

        // Setup
        let packet = create_test_packet();
        let op_cmd = [
            LGT_CMD_LIGHT_CONFIG_GRP,
            (VENDOR_ID & 0xFF) as u8,
            (VENDOR_ID >> 8) as u8,
        ];
        let params = [
            LIGHT_ADD_GRP_PARAM,
            0xCD,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];

        mock_parse_ble_packet_op_params(Any, Any).returns((
            false,
            op_cmd,
            LightOpType::OpType3 as u8,
            params,
            0,
        ));
        mock_add_group(Any).returns(true);

        // Execute
        rf_link_data_callback(&packet);

        // Verify
        mock_add_group(0xCD).assert_called(1);
    }

    #[test]
    #[mry::lock(
        parse_ble_packet_op_params,
        add_device_address,
        dev_addr_with_mac_flag,
        dev_addr_with_mac_match,
        app_mocker
    )]
    fn test_rf_link_data_callback_config_dev_addr() {
        // This test verifies device address configuration

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        app.mesh_manager
            .mock_mesh_device_address_validation_completed()
            .returns(());

        let packet = create_test_packet();
        let op_cmd = [
            LGT_CMD_CONFIG_DEV_ADDR,
            (VENDOR_ID & 0xFF) as u8,
            (VENDOR_ID >> 8) as u8,
        ];
        let params = [0x78, 0x56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        mock_parse_ble_packet_op_params(Any, Any).returns((
            false,
            op_cmd,
            LightOpType::OpType3 as u8,
            params,
            0,
        ));
        mock_dev_addr_with_mac_flag(Any).returns(false);
        mock_dev_addr_with_mac_match(Any).returns(true);
        mock_add_device_address(Any).returns(true);

        // Execute
        rf_link_data_callback(&packet);

        // Verify
        mock_add_device_address(0x5678).assert_called(1);
        app.mesh_manager
            .mock_mesh_device_address_validation_completed()
            .assert_called(1);
    }

    #[test]
    #[mry::lock(parse_ble_packet_op_params, app_mocker)]
    fn test_rf_link_data_callback_set_light() {
        // This test verifies LGT_CMD_SET_LIGHT is handled

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        app.light_manager.mock_send_message(Any, Any).returns(());

        let packet = create_test_packet();
        let op_cmd = [
            LGT_CMD_SET_LIGHT,
            (VENDOR_ID & 0xFF) as u8,
            (VENDOR_ID >> 8) as u8,
        ];
        let params = [0x10, 0x30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        mock_parse_ble_packet_op_params(Any, Any).returns((
            false,
            op_cmd,
            LightOpType::OpType3 as u8,
            params,
            0,
        ));

        // Execute
        rf_link_data_callback(&packet);

        // Verify
        app.light_manager
            .mock_send_message(LGT_CMD_SET_LIGHT, Any)
            .assert_called(1);
    }

    #[test]
    #[mry::lock(
        parse_ble_packet_op_params,
        flash_erase_sector,
        flash_write_page,
        light_sw_reboot
    )]
    fn test_rf_link_data_callback_set_mac_addr() {
        // This test verifies MAC address setting and reboot

        // Setup
        let packet = create_test_packet();
        let op_cmd = [
            LGT_CMD_SET_MAC_ADDR,
            (VENDOR_ID & 0xFF) as u8,
            (VENDOR_ID >> 8) as u8,
        ];
        let params = [0x11, 0x66, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        mock_parse_ble_packet_op_params(Any, Any).returns((
            false,
            op_cmd,
            LightOpType::OpType3 as u8,
            params,
            0,
        ));
        mock_flash_erase_sector(Any).returns(());
        mock_flash_write_page(Any, Any, Any).returns(());
        mock_light_sw_reboot().returns(());

        // Execute
        rf_link_data_callback(&packet);

        // Verify
        mock_flash_erase_sector(Any).assert_called(1);
        mock_flash_write_page(Any, Any, Any).assert_called(1);
        mock_light_sw_reboot().assert_called(1);
    }

    #[test]
    #[mry::lock(parse_ble_packet_op_params, app_mocker)]
    fn test_rf_link_data_callback_mesh_pair() {
        // This test verifies mesh pair command handling

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        app.mesh_manager.mock_mesh_pair_cb(Any).returns(());

        let packet = create_test_packet();
        let op_cmd = [
            LGT_CMD_MESH_PAIR,
            (VENDOR_ID & 0xFF) as u8,
            (VENDOR_ID >> 8) as u8,
        ];
        let params = [0x01, 0x0A, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        mock_parse_ble_packet_op_params(Any, Any).returns((
            false,
            op_cmd,
            LightOpType::OpType3 as u8,
            params,
            0,
        ));

        // Execute
        rf_link_data_callback(&packet);

        // Verify
        app.mesh_manager.mock_mesh_pair_cb(Any).assert_called(1);
    }

    #[test]
    #[mry::lock(parse_ble_packet_op_params)]
    fn test_rf_link_data_callback_wrong_vendor_id() {
        // This test verifies that packets with wrong vendor ID are ignored

        // Setup
        let packet = create_test_packet();
        let op_cmd = [LGT_CMD_LIGHT_ONOFF, 0xFF, 0xFF]; // Wrong vendor ID
        let params = [LIGHT_ON_PARAM, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        mock_parse_ble_packet_op_params(Any, Any).returns((
            false,
            op_cmd,
            LightOpType::OpType3 as u8,
            params,
            0,
        ));

        // Execute
        rf_link_data_callback(&packet);

        // Verify: No further processing occurs (test completes without errors)
    }

    #[test]
    #[mry::lock(parse_ble_packet_op_params)]
    fn test_rf_link_data_callback_wrong_op_cmd_len() {
        // This test verifies that packets with wrong op_cmd_len are ignored

        // Setup
        let packet = create_test_packet();
        let op_cmd = [
            LGT_CMD_LIGHT_ONOFF,
            (VENDOR_ID & 0xFF) as u8,
            (VENDOR_ID >> 8) as u8,
        ];
        let params = [LIGHT_ON_PARAM, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        mock_parse_ble_packet_op_params(Any, Any).returns((
            false,
            op_cmd,
            LightOpType::OpType2 as u8,
            params,
            0,
        )); // Wrong length

        // Execute
        rf_link_data_callback(&packet);

        // Verify: No further processing occurs (test completes without errors)
    }

    /// Integration test: verifies the full path from a UART LightCtrl payload, through
    /// `mesh_construct_packet` and `parse_ble_packet_op_params`, to `rf_link_data_callback`
    /// dispatching the light-on command.
    ///
    /// Guards against a regression where `mesh_construct_packet` was changed from a raw flat
    /// memory copy to explicit struct field assignments. `MeshPkt` is `repr(C, align(4))`, which
    /// inserts 1 byte of padding between `op` (offset 20) and `vendor_id` (offset 22). With
    /// struct field assignments, `vendor_id_lo` lands at offset 22 (the struct field) instead of
    /// offset 21 (what the `val[]` overlay reads). `parse_ble_packet_op_params` reads via `val[]`
    /// and sees `val[1] = padding = 0x00`, computing `vendor_id = 0x1100 ≠ VENDOR_ID`, causing
    /// `rf_link_data_callback` to silently discard the command on every node in the mesh.
    ///
    /// The fix is in construction: `mesh_construct_packet` uses a raw flat byte copy from
    /// `&pkt.op`, placing `vendor_id_lo` at offset 21 (padding slot) and `vendor_id_hi` at
    /// offset 22 — exactly where `val[1]` and `val[2]` read from.
    #[test]
    #[mry::lock(clock_time, app_mocker)]
    fn test_uart_light_ctrl_to_light_on_integration() {
        use crate::sdk::ble_app::light_ll::packet_processing::parse_ble_packet_op_params;
        use heapless::Vec;

        DEVICE_ADDRESS.set(0x1234);

        // Mock clock_time, which light_slave_tx_command uses to seed the mesh sequence number.
        mock_clock_time().returns(0x5000);

        // Step 1: Construct the 13-byte payload that handle_light_ctrl extracts from
        //         msg.data[CTRL_PAYLOAD_RANGE] (bytes 5..18 of a UART LightCtrl message).
        //         Layout: [op, vid_lo*, vid_hi*, par0..par9]
        //         (* vid bytes are overwritten by light_slave_tx_command with VENDOR_ID)
        let uart_payload: [u8; 13] = [
            LGT_CMD_LIGHT_ONOFF, // op = 0x10 (will become 0xd0 after |= 0xc0)
            0x00,                // vendor_id_lo placeholder — overwritten to VENDOR_ID lo
            0x00,                // vendor_id_hi placeholder — overwritten to VENDOR_ID hi
            LIGHT_ON_PARAM,      // par[0] = 0x01 (ON)
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];
        let destination: u16 = 0x1234; // Same as DEVICE_ADDRESS — targeting this device

        // Step 2: Build the packet as send_mesh_message does:
        //         light_slave_tx_command sets op |= 0xc0, fills VENDOR_ID bytes,
        //         then calls mesh_construct_packet which does a raw flat copy from &pkt.op.
        let payload_vec = Vec::<u8, 13>::from_slice(&uart_payload).unwrap();
        let packet = light_slave_tx_command(&payload_vec, destination, 0, false);

        // Step 3: Verify the wire-format bytes via the val[] overlay — the same view that
        //         parse_ble_packet_op_params uses. The raw copy writes:
        //           byte 20 (val[0]) = op | 0xc0
        //           byte 21 (val[1]) = VENDOR_ID & 0xFF = 0x11  ← vendor_id_lo in padding slot
        //           byte 22 (val[2]) = VENDOR_ID >> 8   = 0x02  ← vendor_id_hi
        //           byte 23 (val[3]) = LIGHT_ON_PARAM           ← par[0]
        //         If construction used struct fields instead, val[1] would be 0x00 (padding)
        //         and the vendor_id check in rf_link_data_callback would compute 0x1100 ≠ VENDOR_ID.
        let val = &packet.att_write().value.val;
        assert_eq!(val[0] & 0x3f, LGT_CMD_LIGHT_ONOFF, "val[0] must hold op");
        assert_eq!(
            val[1],
            (VENDOR_ID & 0xFF) as u8,
            "val[1] must hold vendor_id_lo (not padding)"
        );
        assert_eq!(
            val[2],
            (VENDOR_ID >> 8) as u8,
            "val[2] must hold vendor_id_hi"
        );
        assert_eq!(val[3], LIGHT_ON_PARAM, "val[3] must hold par[0]");

        // Step 4: Run the REAL parse_ble_packet_op_params (not mocked) end-to-end.
        let (success, op_cmd, op_len, params, _) = parse_ble_packet_op_params(&packet, true);
        assert!(success, "parse_ble_packet_op_params should succeed");
        assert_eq!(op_len, LightOpType::OpType3 as u8, "op_len must be 3");
        let parsed_vendor_id = (op_cmd[2] as u16) << 8 | op_cmd[1] as u16;
        assert_eq!(
            parsed_vendor_id, VENDOR_ID,
            "vendor_id reconstructed from op_cmd[1..3] must equal VENDOR_ID \
             (regression: struct field assignment leaves val[1]=padding=0x00 → vendor_id=0x1100)"
        );
        assert_eq!(
            op_cmd[0] & 0x3f,
            LGT_CMD_LIGHT_ONOFF,
            "op must be LGT_CMD_LIGHT_ONOFF"
        );
        assert_eq!(
            params[0], LIGHT_ON_PARAM,
            "params[0] must be LIGHT_ON_PARAM"
        );

        // Step 5: Run the REAL rf_link_data_callback (not mocked).
        //         With the bug: vendor_id check fails → early return → light never turns on.
        //         With the fix: vendor_id passes → LGT_CMD_LIGHT_ONOFF dispatched correctly.
        let mut app = App::default();
        app.light_manager.mock_send_message(Any, Any).returns(());
        mock_app_mocker().returns(&mut app);

        rf_link_data_callback(&packet);

        app.light_manager
            .mock_send_message(LGT_CMD_LIGHT_ONOFF, Any)
            .assert_called(1);
    }

    // --- Tests for rf_link_light_event_callback ---

    #[test]
    #[mry::lock(mesh_node_init, app_mocker)]
    fn test_rf_link_light_event_callback_set_mesh_info() {
        // This test verifies LGT_CMD_SET_MESH_INFO event handling

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        app.light_manager.mock_device_status_update().returns(());

        mock_mesh_node_init().returns(());

        // Clear LED event before test
        LED_CONTROLLER.event_pending.set(0);

        // Execute
        rf_link_light_event_callback(LGT_CMD_SET_MESH_INFO);

        // Verify
        mock_mesh_node_init().assert_called(1);
        app.light_manager
            .mock_device_status_update()
            .assert_called(1);
        assert_eq!(LED_CONTROLLER.event_pending.get(), LED_EVENT_FLASH_1HZ_4S);

        // Cleanup
        LED_CONTROLLER.event_pending.set(0);
    }

    #[test]
    #[mry::lock(mesh_node_init, app_mocker)]
    fn test_rf_link_light_event_callback_set_dev_addr() {
        // This test verifies LGT_CMD_SET_DEV_ADDR event handling

        // Setup
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);
        app.light_manager.mock_device_status_update().returns(());

        mock_mesh_node_init().returns(());

        // Clear LED event before test
        LED_CONTROLLER.event_pending.set(0);

        // Execute
        rf_link_light_event_callback(LGT_CMD_SET_DEV_ADDR);

        // Verify
        mock_mesh_node_init().assert_called(1);
        app.light_manager
            .mock_device_status_update()
            .assert_called(1);
        assert_eq!(LED_CONTROLLER.event_pending.get(), LED_EVENT_FLASH_1HZ_4S);

        // Cleanup
        LED_CONTROLLER.event_pending.set(0);
    }

    #[test]
    fn test_rf_link_light_event_callback_del_pair() {
        // This test verifies LGT_CMD_DEL_PAIR event handling

        // Clear LED event before test
        LED_CONTROLLER.event_pending.set(0);

        // Execute
        rf_link_light_event_callback(LGT_CMD_DEL_PAIR);

        // Verify
        assert_eq!(LED_CONTROLLER.event_pending.get(), LED_EVENT_FLASH_1HZ_4S);

        // Cleanup
        LED_CONTROLLER.event_pending.set(0);
    }

    #[test]
    fn test_rf_link_light_event_callback_mesh_pair_timeout() {
        // This test verifies LGT_CMD_MESH_PAIR_TIMEOUT event handling

        // Clear LED event before test
        LED_CONTROLLER.event_pending.set(0);

        // Execute
        rf_link_light_event_callback(LGT_CMD_MESH_PAIR_TIMEOUT);

        // Verify
        assert_eq!(LED_CONTROLLER.event_pending.get(), LED_EVENT_FLASH_2HZ_2S);

        // Cleanup
        LED_CONTROLLER.event_pending.set(0);
    }

    #[test]
    fn test_rf_link_light_event_callback_unknown_status() {
        // This test verifies that unknown status codes do nothing

        // Clear LED event before test
        LED_CONTROLLER.event_pending.set(0);

        // Execute
        rf_link_light_event_callback(0xFF);

        // Verify: LED event should remain 0
        assert_eq!(LED_CONTROLLER.event_pending.get(), 0);
    }

    // --- Tests for light_init_default ---

    #[test]
    #[mry::lock(
        light_check_tick_per_us,
        rf_link_slave_pairing_enable,
        rf_set_power_level_index,
        usb_dp_pullup_en,
        light_hw_timer_config,
        app_mocker
    )]
    fn test_light_init_default() {
        // This test verifies that light_init_default properly initializes mesh configuration

        // Setup mocks
        mock_light_check_tick_per_us(CLOCK_SYS_CLOCK_1US).returns(());
        mock_rf_link_slave_pairing_enable(Any).returns(());
        mock_rf_set_power_level_index(Any).returns(());
        mock_usb_dp_pullup_en(Any).returns(());
        mock_light_hw_timer_config().returns(());

        let mut app = App::default();
        app.mesh_manager.mock_mesh_pair_init().returns(());
        mock_app_mocker().returns(&mut app);

        // Setup initial state - simulate ADV_DATA with some length
        let mut adv_data = ADV_DATA.lock();
        adv_data[0] = 2;
        adv_data[1] = 1;
        adv_data[2] = 5; // Default values
        drop(adv_data);

        // Execute
        light_init_default();

        // Verify: light_check_tick_per_us was called with CLOCK_SYS_CLOCK_1US
        mock_light_check_tick_per_us(CLOCK_SYS_CLOCK_1US).assert_called(1);

        // Verify: rf_link_slave_pairing_enable was called with true
        mock_rf_link_slave_pairing_enable(true).assert_called(1);

        // Verify: rf_set_power_level_index was called with RfPower::Power8dBm
        mock_rf_set_power_level_index(RfPower::Power8dBm as u32).assert_called(1);

        // Verify: usb_dp_pullup_en was called with true
        mock_usb_dp_pullup_en(true).assert_called(1);

        // Verify: light_hw_timer_config was called
        mock_light_hw_timer_config().assert_called(1);

        // Verify: mesh_pair_init was called
        app.mesh_manager.mock_mesh_pair_init().assert_called(1);

        // Verify static state changes
        assert_eq!(MAX_MESH_NAME_LEN, MAX_MESH_NAME_LEN);

        // Verify mesh name was copied (first few bytes)
        let mesh_name = PAIR_CONFIG_MESH_NAME.lock();
        assert_eq!(mesh_name[0], MESH_NAME.as_bytes()[0]);
        assert_eq!(mesh_name[1], MESH_NAME.as_bytes()[1]);

        // Verify mesh password was copied
        let mesh_pwd = PAIR_CONFIG_MESH_PWD.lock();
        assert_eq!(mesh_pwd[0], MESH_PWD.as_bytes()[0]);
        assert_eq!(mesh_pwd[1], MESH_PWD.as_bytes()[1]);
        assert_eq!(mesh_pwd[2], MESH_PWD.as_bytes()[2]);

        // Verify mesh LTK was copied
        let mesh_ltk = PAIR_CONFIG_MESH_LTK.lock();
        assert_eq!(mesh_ltk[0], MESH_LTK[0]);
        assert_eq!(mesh_ltk[1], MESH_LTK[1]);
    }

    // --- Tests for user_init ---

    #[mry::mry]
    #[mry::lock(
        app_mocker,
        light_check_tick_per_us,
        rf_link_slave_pairing_enable,
        rf_set_power_level_index,
        usb_dp_pullup_en,
        light_hw_timer_config,
        pwm_set_duty,
        pwm_start,
        gpio_set_func,
        rf_link_slave_init,
        factory_reset_handle,
        vendor_set_adv_data
    )]
    #[test]
    fn test_user_init() {
        // Set up app mock
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);

        // Mock app methods
        app.ota_manager.mock_check_ota_area_startup().returns(());
        app.light_manager.mock_light_lum_retrieve().returns(());
        app.light_manager.mock_device_status_update().returns(());
        app.mesh_manager.mock_mesh_pair_init().returns(());
        app.mesh_manager.mock_mesh_security_enable(Any).returns(());

        // Mock functions that light_init_default calls
        mock_light_check_tick_per_us(CLOCK_SYS_CLOCK_1US).returns(());
        mock_rf_link_slave_pairing_enable(true).returns(());
        mock_rf_set_power_level_index(RfPower::Power8dBm as u32).returns(());
        mock_usb_dp_pullup_en(true).returns(());
        mock_light_hw_timer_config().returns(());

        // Mock functions that user_init calls directly
        mock_pwm_set_duty(0, 10455, 0).returns(());
        mock_pwm_set_duty(1, 10455, 10455).returns(());
        mock_pwm_start(0).returns(());
        mock_pwm_start(1).returns(());
        mock_gpio_set_func(513, 255).returns(());
        mock_gpio_set_func(516, 255).returns(());
        mock_rf_link_slave_init(40000).returns(());
        mock_factory_reset_handle().returns(());
        mock_vendor_set_adv_data().returns(());

        // Call user_init
        user_init();

        // Verify all mocks were called
        app.ota_manager
            .mock_check_ota_area_startup()
            .assert_called(1);
        mock_light_check_tick_per_us(CLOCK_SYS_CLOCK_1US).assert_called(1);
        mock_rf_link_slave_pairing_enable(true).assert_called(1);
        mock_rf_set_power_level_index(RfPower::Power8dBm as u32).assert_called(1);
        mock_usb_dp_pullup_en(true).assert_called(1);
        mock_light_hw_timer_config().assert_called(1);
        app.mesh_manager.mock_mesh_pair_init().assert_called(1);
        mock_pwm_set_duty(0, 10455, 0).assert_called(1);
        mock_pwm_set_duty(1, 10455, 10455).assert_called(1);
        mock_pwm_start(0).assert_called(1);
        mock_pwm_start(1).assert_called(1);
        mock_gpio_set_func(513, 255).assert_called(1);
        mock_gpio_set_func(516, 255).assert_called(1);
        app.light_manager.mock_light_lum_retrieve().assert_called(1);
        mock_rf_link_slave_init(40000).assert_called(1);
        mock_factory_reset_handle().assert_called(1);
        mock_vendor_set_adv_data().assert_called(1);
        app.light_manager
            .mock_device_status_update()
            .assert_called(1);
        app.mesh_manager
            .mock_mesh_security_enable(true)
            .assert_called(1);
    }

    // --- Tests for light_user_func ---

    #[test]
    #[mry::lock(app_mocker, clock_time_exceed, factory_reset_cnt_check)]
    fn test_light_user_func_structure() {
        // This test verifies that light_user_func is properly decorated for testing
        // and that it can be called with mocked app context

        // Setup - mock the app with all necessary sub-mocks
        let mut app = App::default();
        mock_app_mocker().returns(&mut app);

        // Mock the light_manager methods called by light_user_func and light_auth_check
        app.light_manager.mock_check_light_state_save().returns(());

        // Mock mesh_manager methods
        app.mesh_manager.mock_mesh_pair_proc_effect().returns(());

        // Mock clock_time_exceed to return false (no timeout)
        mock_clock_time_exceed(Any, Any).returns(false);

        // Mock factory_reset_cnt_check
        mock_factory_reset_cnt_check().returns(());

        // Execute - light_user_func calls:
        // 1. app().light_manager.check_light_state_save();
        // 2. light_auth_check(); (which checks global state)
        // 3. factory_reset_cnt_check(); (external function)
        // 4. app().mesh_manager.mesh_pair_proc_effect();
        light_user_func();

        // Verify the key app methods were called
        app.light_manager
            .mock_check_light_state_save()
            .assert_called(1);
        app.mesh_manager
            .mock_mesh_pair_proc_effect()
            .assert_called(1);
    }

    // --- Tests for main_loop ---

    #[test]
    #[mry::lock(light_user_func, rf_link_slave_proc, proc_led)]
    fn test_main_loop_structure() {
        // This test verifies that main_loop calls the expected functions

        // Mock the direct callees of main_loop
        mock_light_user_func().returns(());
        mock_rf_link_slave_proc().returns(());
        mock_proc_led().returns(());

        // Execute main_loop
        block_on(main_loop());

        // Verify the functions were called
        mock_light_user_func().assert_called(1);
        mock_rf_link_slave_proc().assert_called(1);
        mock_proc_led().assert_called(1);
    }
}
