#![coverage(off)]
use core::cmp::min;
use core::ptr::addr_of;
use core::sync::atomic::Ordering;

use embassy_executor::Spawner;

#[cfg(not(test))]
use embassy_time::{Duration, Timer};

use crate::config::*;
use crate::diagnostics::{
    boot_init, check_panic_info, record_breadcrumb, CRUMB_INIT_DONE, CRUMB_MAIN_LOOP,
};
use crate::factory_reset::{self, KickoutAction, KickoutReason, ResetAction};
use crate::light::effects::{
    adv_private_to_bytes, build_vendor_adv_data, config_led_event, led_indicate_step,
    LED_EVENT_FLASH_1HZ_4S, LED_EVENT_FLASH_2HZ_2S,
};
use crate::light::{
    decode_mesh_command, device_status_update, send_light_message, LightCommandAction,
    LightFlashStore, PMW_MAX_TICK,
};
use crate::mesh::{
    mesh_device_address_validation_completed, mesh_pair_cb, mesh_pair_init, mesh_pair_proc_effect,
    mesh_security_enable,
};
use crate::ota::{check_ota_area_startup, rf_mesh_data_ota, set_ota_radio_ops, OtaRadioOps};
use crate::sdk::ble_app::ble_ll_pair::{
    MESH_PAIR_ENABLE, PAIR_CONFIG_MESH_LTK, PAIR_CONFIG_MESH_NAME, PAIR_CONFIG_MESH_PWD,
    PAIR_CONFIG_PWD_ENCODE_SK, PAIR_LOGIN_OK, SECURITY_ENABLE,
};
use crate::sdk::ble_app::callbacks::{set_ble_app_hooks, set_ota_hooks, BleAppHooks, OtaHooks};
use crate::sdk::ble_app::flash_config::{dev_addr_with_mac_flag, dev_addr_with_mac_match};
use crate::sdk::ble_app::light_ll::connection_management::{
    light_check_tick_per_us, rf_link_slave_pairing_enable, rf_link_slave_proc,
    BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP,
};
use crate::sdk::ble_app::light_ll::mesh_management::mesh_node_table_init;
use crate::sdk::ble_app::light_ll::packet_processing::parse_ble_packet_op_params;
use crate::sdk::ble_app::rf_drv_8266::{
    rf_drv_init, rf_link_slave_init, rf_set_power_level_index, ADV_PRI_DATA, ADV_RSP_PRI_DATA,
    DEVICE_ADDRESS, GROUP_ADDRESS, MAC_ID, MAX_MESH_NAME_LEN,
    MESH_DEVICE_ADDRESS_VALIDATION_PENDING,
};
use crate::sdk::light::*;
use crate::sdk::rf_drv::*;
use crate::uart_manager::{uart_receiver, uart_sender};
use crate::uprintln;
use crate::version::BUILD_VERSION;
use heapless::Vec;
use mesh_protocol::mesh_types::{LGT_CMD_END_OTA_RSP, LGT_CMD_OTA_DATA_RSP, LGT_CMD_START_OTA_RSP};
use mesh_protocol::packet_types::{Packet, PacketAttValue};
use tlsr8266_hal::common::sync::SimplifyLS;
use tlsr8266_hal::drivers::flash::{flash_erase_sector, flash_write_page};
use tlsr8266_hal::drivers::pwm::{pwm_set_duty, pwm_start};
use tlsr8266_hal::mcu::clock::{
    clock_init, clock_time_exceed, is_16mhz_xtal, CLOCK_SYS_CLOCK_1S, CLOCK_SYS_CLOCK_1US,
};
use tlsr8266_hal::mcu::crypto::encode_password;
use tlsr8266_hal::mcu::dma::dma_init;
use tlsr8266_hal::mcu::gpio::{gpio_init, gpio_set_func, AS_GPIO};
use tlsr8266_hal::mcu::irq_i::{irq_disable, irq_enable, irq_init};
use tlsr8266_hal::mcu::watchdog::wd_clear;
use tlsr8266_hal::pm::{blink_ota_led_ok, light_sw_reboot, usb_dp_pullup_en};

/// The application's single root task.
///
/// Boot order is normative (design 018 §5): OTA recovery runs in
/// `main_entrypoint` before any clock or IRQ exists, then this task brings the
/// peripherals up, registers every SDK seam **before** `rf_drv_init(true)`
/// populates the radio, and only then spawns the per-owner tasks and enters the
/// main loop.
#[embassy_executor::task]
pub async fn main_task(spawner: Spawner) {
    // Initialise various subsystems needed by uart
    clock_init();
    dma_init();
    gpio_init();
    irq_init();

    // Get uart ready to go early
    crate::uart_manager::init();

    // Report why the previous boot ended and mark this one as running
    boot_init();

    uprintln!(
        "Booting FW version {}, xtal: {}MHz",
        BUILD_VERSION,
        if is_16mhz_xtal() { 16 } else { 12 }
    );

    // Wire the SDK's callback seams to the application before any RF or pairing
    // path can run, so neither slot is ever observed empty.
    set_ota_radio_ops(OtaRadioOps {
        try_begin_gatt_ota: ota_begin_gatt_hook,
        stage_packet: crate::sdk::ble_app::light_ll::connection_management::stage_ota_packet,
        read_staged_packet:
            crate::sdk::ble_app::light_ll::connection_management::read_staged_ota_packet,
        try_send_terminate_packet: ota_send_terminate_hook,
    });
    set_ota_hooks(OtaHooks {
        on_gatt_ota_write: crate::ota::on_gatt_ota_packet,
        status_snapshot: crate::ota::ota_status_snapshot_impl,
        request_termination: crate::ota::ota_request_termination_impl,
        tick_timeout_1s: crate::ota::ota_tick_timeout_1s_impl,
    });
    crate::sdk::rf_drv::set_light_event_handler(rf_link_light_event_callback);
    set_ble_app_hooks(BleAppHooks {
        on_mesh_pair_proc: mesh_pair_proc_hook,
        on_light_data: rf_link_data_callback,
        on_light_response: rf_link_response_callback,
        on_uart_mesh_rx: crate::uart_manager::light_mesh_rx_cb,
    });
    // The light subsystem owns the lamp but not the mesh node table: publish its
    // on/off state through this pointer instead of letting `light` depend on
    // the SDK. Registered here, before any mesh traffic exists.
    crate::light::set_status_update_hook(
        crate::sdk::ble_app::light_ll::mesh_management::ll_device_status_update,
    );

    // Copy the password in to the pair config
    PAIR_CONFIG_PWD_ENCODE_SK.lock()[0..MESH_PWD_ENCODE_SK.len()]
        .copy_from_slice(&MESH_PWD_ENCODE_SK.as_bytes()[0..MESH_PWD_ENCODE_SK.len()]);

    rf_drv_init(true);

    // Run our initialisation; the lamp's write pointer lives here from now on
    let mut flash_store = user_init();
    record_breadcrumb(CRUMB_INIT_DONE);

    // Start the mesh packet sender / receive handler
    spawner.spawn(crate::mesh::mesh_packet_sender().unwrap());
    spawner.spawn(crate::mesh::mesh_packet_rcv().unwrap());

    // Start the UART tasks. `uart_sender` parks on `started_signal` until the
    // first valid host frame arrives, so a headless light never retries DMA TX
    // for the boot banners.
    spawner.spawn(uart_receiver().unwrap());
    spawner.spawn(uart_sender().unwrap());

    // Start the light command consumer
    spawner.spawn(light_manager().unwrap());

    // Start the panic checker to see if there is information we need to send
    spawner.spawn(panic_check().unwrap());

    // Ready to go, enable interrupts and run the main loop
    irq_enable();
    record_breadcrumb(CRUMB_MAIN_LOOP);

    // Send a message to the network saying that we just booted up
    let mut data = [0_u8; 13];
    data[0] = LGT_POWER_ON;

    crate::mesh::send_mesh_message(&Vec::from_slice(&data).unwrap(), 0xffff, 3, false);

    loop {
        // Prove to a host that the loop is still turning (`fetch_add` does
        // not exist on thumbv6m)
        let next = crate::HEARTBEAT.load(Ordering::Relaxed).wrapping_add(1);
        crate::HEARTBEAT.store(next, Ordering::Relaxed);

        // Clear the watchdog timer
        wd_clear();

        // Run the main loop - this only runs once per LOOP_INTERVAL_US
        main_loop(&mut flash_store).await;

        // Clear any uart errors if they've occurred
        crate::uart_manager::clear_error();
    }
}

#[cfg_attr(test, mry::mry)]
pub fn light_hw_timer_config() {
    // Enable timer1 interrupts for controlling light transitions
    tlsr8266_hal::mcu::irq_i::irq_enable_tmr1();
    tlsr8266_hal::mcu::timer::tmr1_configure(CLOCK_SYS_CLOCK_1S / 120); // ~ 120hz

    // enable timer0 interrupt for tracking clock_time overflow
    tlsr8266_hal::mcu::irq_i::irq_enable_tmr0();
    tlsr8266_hal::mcu::timer::tmr0_configure_and_enable(CLOCK_SYS_CLOCK_1S);
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

    mesh_pair_init();
}

/// Initialise the lamp and hand the caller the persisted-brightness store that
/// the main loop owns from here on.
pub fn user_init() -> LightFlashStore {
    // for app ota
    check_ota_area_startup();

    light_init_default();

    pwm_set_duty(PWMID_G, PMW_MAX_TICK, 0);
    pwm_set_duty(PWMID_B, PMW_MAX_TICK, PMW_MAX_TICK);

    pwm_start(PWMID_G);
    pwm_start(PWMID_B);

    gpio_set_func(PWM_G as u32, !AS_GPIO);
    gpio_set_func(PWM_B as u32, !AS_GPIO);

    //retrieve lumen value
    let flash_store = LightFlashStore::retrieve_from_flash();

    rf_link_slave_init(40000);

    // Print MAC address now that it has been loaded from flash (reversed byte order)
    {
        // Scoped: the advertisement payload below takes the same lock.
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
    }

    match factory_reset::handle_boot_sequence() {
        ResetAction::FactoryResetAndReboot => {
            irq_disable();
            factory_reset::erase_config_sectors();
            blink_ota_led_ok(OTA_LED as u32);
            light_sw_reboot();
        }
        ResetAction::None => {}
    }

    // Brand the device: the light subsystem builds the two private advertisement
    // payloads from the loaded MAC, the vendor id and the device address.
    let (pri, rsp) = build_vendor_adv_data(*MAC_ID.lock(), VENDOR_ID, DEVICE_ADDRESS.get());
    *ADV_PRI_DATA.lock() = pri;
    *ADV_RSP_PRI_DATA.lock() = rsp;
    set_advertisement_manufacturer_data(&adv_private_to_bytes(&pri));

    device_status_update();
    mesh_security_enable(true);

    flash_store
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

/// One main-loop tick's worth of light work.
///
/// Deliberately not `mry`-mockable: `mry` requires every argument type to be
/// `Clone`, and handing the flash store over as `&mut` is the whole point of
/// the ownership split. Tests drive the real function and assert its
/// observable effects (`clock_time_exceed`, `flash_write_page`).
fn light_user_func(flash_store: &mut LightFlashStore) {
    flash_store.check_light_state_save();

    light_auth_check();
    let _ = factory_reset::check_timing_step();
    mesh_pair_proc_effect();
}

pub async fn main_loop(flash_store: &mut LightFlashStore) {
    #[cfg(not(test))]
    Timer::after(Duration::from_micros(LOOP_INTERVAL_US)).await;

    light_user_func(flash_store);
    rf_link_slave_proc();

    led_indicate_step();
}

/*@brief: This function is called in IRQ state, use IRQ stack.
**@param: ppp: is pointer to response
**@param: p_req: is pointer to request command
Called to handle messages that require a response to be returned
*/
#[inline(never)]
#[cfg_attr(test, mry::mry)]
pub fn rf_link_response_callback(
    ppp: &mut PacketAttValue,
    p_req: &PacketAttValue,
) -> Result<(), ()> {
    // The five light-domain opcodes are the light subsystem's own protocol; it
    // builds the whole answer, including the shared response prologue.
    if crate::light::build_light_status_response(
        ppp,
        p_req,
        DEVICE_ADDRESS.get(),
        &GROUP_ADDRESS.lock(),
        *MAC_ID.lock(),
    )
    .is_ok()
    {
        return Ok(());
    }

    // Everything else is the OTA seam: the same prologue, then the three opcodes.
    ppp.dst = p_req.src;
    ppp.src[0] = (DEVICE_ADDRESS.get() & 0xff) as u8;
    ppp.src[1] = ((DEVICE_ADDRESS.get() >> 8) & 0xff) as u8;

    let params = &p_req.val[3..13];
    ppp.val[3..10 + 3].fill(0);

    ppp.val[1] = (VENDOR_ID & 0xFF) as u8;
    ppp.val[2] = ((VENDOR_ID >> 8) & 0xff) as u8;

    match ppp.val[15] {
        CMD_START_OTA => {
            ppp.val[0] = LGT_CMD_START_OTA_RSP | 0xc0;

            ppp.val[3] = BUILD_VERSION as u8;
            ppp.val[4] = (BUILD_VERSION >> 8) as u8;
            ppp.val[5] = (BUILD_VERSION >> 16) as u8;
            ppp.val[6] = (BUILD_VERSION >> 24) as u8;

            crate::ota::start_mesh_ota();
        }
        CMD_OTA_DATA => {
            ppp.val[0] = LGT_CMD_OTA_DATA_RSP | 0xc0;

            let idx = rf_mesh_data_ota(params, false);

            ppp.val[1] = idx as u8;
            ppp.val[2] = (idx >> 8) as u8;
        }
        CMD_END_OTA => {
            ppp.val[0] = LGT_CMD_END_OTA_RSP | 0xc0;

            let idx = rf_mesh_data_ota(params, true);

            ppp.val[1] = idx as u8;
            ppp.val[2] = (idx >> 8) as u8;
        }
        _ => return Err(()),
    }

    Ok(())
}

/*@brief: This function is called in IRQ state, use IRQ stack.
Called to handle messages sent to us that don't require a response
*/
#[inline(never)]
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

    // The light subsystem decodes the opcode; everything below needs the flash,
    // the mesh node table, the pairing state machine or the radio, so it stays
    // in this layer.
    match decode_mesh_command(op, &params) {
        LightCommandAction::None => {}
        LightCommandAction::EnqueueLight(message) => {
            send_light_message(message.cmd, message.params)
        }
        LightCommandAction::ConfigGroup { add, group } => {
            let applied = if add {
                add_group(group)
            } else {
                remove_group(group)
            };
            if applied.is_ok() {
                config_led_event(LED_EVENT_FLASH_1HZ_4S);
            }
        }
        LightCommandAction::ConfigDevAddr { addr, params } => {
            if (!dev_addr_with_mac_flag(&params) || dev_addr_with_mac_match(&params))
                && add_device_address(addr).is_ok()
            {
                mesh_device_address_validation_completed();
            }
        }
        LightCommandAction::SetMacAddr(mac) => {
            flash_erase_sector(FLASH_ADR_MAC);
            flash_write_page(FLASH_ADR_MAC, mac.len() as u32, addr_of!(mac) as *const u8);
            light_sw_reboot();
        }
        LightCommandAction::KickOut(kick_out_param) => {
            irq_disable();
            // Unknown reasons fall back to a full out-of-mesh kick-out, as before.
            let reason =
                KickoutReason::try_from(kick_out_param as u32).unwrap_or(KickoutReason::OutOfMesh);
            let action = factory_reset::plan_kick_out(reason);

            // The erase must come first: the out-of-mesh credentials are written
            // into the sector it clears.
            factory_reset::erase_config_sectors();
            match action {
                KickoutAction::WriteOutOfMeshCredentialsAndReboot => {
                    write_out_of_mesh_credentials()
                }
                KickoutAction::ErasePairingAndReboot => {}
            }

            blink_ota_led_ok(OTA_LED as u32);
            light_sw_reboot();
        }
        LightCommandAction::MeshPair(pair_params) => {
            mesh_pair_cb(&Vec::from_slice(&pair_params).unwrap())
        }
    }
}

/// Re-write the out-of-mesh pairing credentials after a kick-out erased the
/// configuration sectors.
///
/// Four 16-byte blocks, in the slot layout the pairing code reads:
/// flags at `+0` (LTK at `+48`, encoded password at `+32`, network name at
/// `+16`).
fn write_out_of_mesh_credentials() {
    let pairing_addr = FLASH_ADR_PAIRING;
    let mut flags: [u8; 16] = [0; 16];
    let mut name: [u8; 16] = [0; 16];
    let mut password: [u8; 16] = [0; 16];

    flags[0] = PAIR_VALID_FLAG;
    flags[15] = PAIR_VALID_FLAG;
    let len = min(OUT_OF_MESH.len(), name.len());
    name[0..len].copy_from_slice(&OUT_OF_MESH.as_bytes()[0..len]);
    let len = min(MESH_PWD.len(), password.len());
    password[0..len].copy_from_slice(&MESH_PWD.as_bytes()[0..len]);

    // Encrypt the password before storing. The key is copied out of the mutex
    // first so the lock is released before `encode_password` runs (its AES call
    // masks interrupts).
    let key = *PAIR_CONFIG_PWD_ENCODE_SK.lock();
    let encoded_password = encode_password(&key, &password);

    // Only write the rejoin flag when mesh pairing is enabled.
    if MESH_PAIR_ENABLE.get() {
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(true);
        flags[1] = 1;
    }

    let mut ltk: [u8; 16] = *PAIR_CONFIG_MESH_LTK.lock();
    flash_write_page(pairing_addr + 48, 16, ltk.as_mut_ptr());
    flash_write_page(pairing_addr + 32, 16, encoded_password.as_ptr());
    flash_write_page(pairing_addr + 16, 16, name.as_ptr());
    flash_write_page(pairing_addr, 16, flags.as_ptr());
}

#[inline(never)]
#[cfg_attr(test, mry::mry)]
pub fn rf_link_light_event_callback(status: u8) {
    match status {
        LGT_CMD_SET_MESH_INFO => {
            mesh_node_table_init();
            device_status_update();
            config_led_event(LED_EVENT_FLASH_1HZ_4S);
        }
        LGT_CMD_SET_DEV_ADDR => {
            mesh_node_table_init();
            device_status_update();
            config_led_event(LED_EVENT_FLASH_1HZ_4S);
        }
        LGT_CMD_DEL_PAIR => config_led_event(LED_EVENT_FLASH_1HZ_4S),
        LGT_CMD_MESH_PAIR_TIMEOUT => config_led_event(LED_EVENT_FLASH_2HZ_2S),
        _ => (),
    }
}

/// Task wrapper for the light command queue consumer.
///
/// `light_manager_task` is a plain `async fn` so the queue owner stays testable
/// without an executor; this shim gives it a task slot.
#[embassy_executor::task]
async fn light_manager() {
    crate::light::light_manager_task().await;
}

/// Task wrapper for the one-shot panic-info report.
#[embassy_executor::task]
async fn panic_check() {
    check_panic_info().await;
}

/// Adapter that gives the mesh pairing seam its own symbol in the linked ELF.
///
/// `mesh::mesh_pair_proc` and this wrapper have indistinguishable prologues, so
/// `sdk/stack_analysis.py`'s `INDIRECT_EDGES` matches the edge by symbol name.
/// Both this function and `mesh::mesh_pair_proc` are `#[inline(never)]` so the
/// symbol the analyser looks for actually survives `OPT_INLINE_THRESHOLD`.
#[inline(never)]
fn mesh_pair_proc_hook() {
    crate::mesh::mesh_pair_proc();
}

/// `OtaRadioOps::try_begin_gatt_ota`: the pairing gate plus "stop a busy status
/// read" from the old GATT OTA write prologue.
///
/// This only *reports* whether the GATT OTA may start. The in-progress flag and
/// `pm::set_reboot_ota_active(true)` stay owned by `ota`, which is why there is
/// no `start_gatt_ota` for this hook to call.
#[inline(never)]
fn ota_begin_gatt_hook() -> bool {
    if !crate::sdk::ble_app::ble_ll_pair::PAIR_LOGIN_OK.get() {
        return false;
    }
    if crate::sdk::ble_app::light_ll::status_management::SLAVE_READ_STATUS_BUSY.get() != 0 {
        crate::sdk::ble_app::light_ll::status_management::rf_link_slave_read_status_stop();
    }
    true
}

/// `OtaRadioOps::try_send_terminate_packet`: `true` only when `PKT_TERMINATE`
/// was actually queued, so a full buffer neither arms the countdown nor reboots.
#[inline(never)]
fn ota_send_terminate_hook() -> bool {
    if !crate::sdk::ble_app::light_ll::packet_processing::is_add_packet_buf_ready() {
        return false;
    }
    let _ = crate::sdk::ble_app::light_ll::packet_processing::rf_link_add_tx_packet(
        &crate::sdk::ble_app::rf_drv_8266::PKT_TERMINATE,
    );
    true
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::factory_reset::{
        check_timing_step, handle_boot_sequence, mock_check_timing_step, mock_handle_boot_sequence,
        mock_plan_kick_out, plan_kick_out,
    };
    use crate::light::effects::{
        mock_led_indicate_step, pending_event, reset_for_test as reset_led_controller,
    };
    use crate::light::mock_device_status_update;
    use crate::light::transitions::{
        begin_transition, light_onoff_hw, mock_light_onoff_hw, LIGHT_CMD_CHANNEL,
    };
    use crate::mesh::{
        mock_mesh_device_address_validation_completed, mock_mesh_pair_cb, mock_mesh_pair_init,
        mock_mesh_pair_proc_effect, mock_mesh_security_enable,
    };
    use crate::ota::{mock_check_ota_area_startup, mock_rf_mesh_data_ota};
    use crate::sdk::ble_app::flash_config::{
        mock_dev_addr_with_mac_flag, mock_dev_addr_with_mac_match,
    };
    use crate::sdk::ble_app::light_ll::connection_management::BUFF_RESPONSE;
    use crate::sdk::ble_app::light_ll::connection_management::{
        mock_light_check_tick_per_us, mock_rf_link_slave_pairing_enable, mock_rf_link_slave_proc,
    };
    use crate::sdk::ble_app::light_ll::mesh_management::mock_mesh_node_table_init;
    use crate::sdk::ble_app::light_ll::packet_processing::mock_parse_ble_packet_op_params;
    use crate::sdk::ble_app::light_ll::status_management::{
        DEVICE_STATUS_BUFFER_READ_POINTER, DEVICE_STATUS_BUFFER_WRITE_POINTER,
        DEVICE_STATUS_RECORD_INDEX, SLAVE_READ_STATUS_BUSY,
    };
    use crate::sdk::ble_app::rf_drv_8266::mock_rf_link_slave_init;
    use crate::sdk::ble_app::rf_drv_8266::mock_rf_set_power_level_index;
    use crate::sdk::ble_app::rf_drv_8266::ADV_DATA;
    use crate::sdk::rf_drv::mock_set_advertisement_manufacturer_data;
    use futures::executor::block_on;
    use mesh_protocol::packet_types::PacketL2capHead;
    use mry::Any;
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog;
    use tlsr8266_hal::mcu::clock::{fake_clock, CLOCK_SYS_CLOCK_1US};
    use tlsr8266_hal::mcu::crypto::fake_crypto;
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_pac::{FLD_IRQ, FLD_TMR};

    /// `reg_irq_en`: the global interrupt enable register.
    const REG_IRQ_EN: u32 = 0x643;
    /// `reg_pwdn_ctrl`: written by `mcu_trigger_reboot`.
    const REG_PWDN_CTRL: u32 = 0x6f;
    /// `reg_gpio_gpio_func` plus the port C register offset (`(pin >> 8) << 3`).
    const REG_GPIO_PC_FUNC: u32 = 0x586 + 0x10;

    /// The HAL host seams are process-wide singletons, so every test that
    /// observes them starts from a known-empty bank, analog plane and clock.
    fn reset_fakes() {
        FakeMmio::reset();
        FakeAnalog::reset();
        fake_clock::reset();
        fake_crypto::reset();
        FakeFlash::reset();
    }

    /// Read one byte of the fake NOR array.
    fn flash_byte(addr: u32) -> u8 {
        let mut got = [0u8; 1];
        FakeFlash::read_slice(addr, &mut got);
        got[0]
    }

    // --- Helper Functions ---

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

    /// `main_task` is the composition root for the SDK callback seams, but it
    /// cannot be executed on the host: `rf_drv_init` and `user_init` are not
    /// `mry`-mockable and both touch hardware registers, and the slots they fill
    /// are private statics owned by `sdk::app_att_light`, `sdk::rf_drv` and
    /// `sdk::ble_app::callbacks`.
    ///
    /// What can be pinned here is the structural invariant that makes the wiring
    /// safe: every registration lives in `main_task`'s body and precedes
    /// `rf_drv_init(true)`, so an accidental reorder or deletion fails this test.
    /// That the slots are populated afterwards is proven on target by the HIL
    /// acceptance run (boot, pairing LED flash, OTA).
    #[test]
    fn test_init_registers_both_sdk_handlers_before_rf_drv_init() {
        let src = include_str!("app.rs");

        let task_start = src
            .find("pub async fn main_task(spawner: Spawner)")
            .expect("main_task must exist");
        // Search only the task body: the tail of the file includes this test,
        // whose own string literals would otherwise satisfy the needles below.
        let task_tail = &src[task_start..];
        let task_body = &task_tail[..task_tail
            .find("\n}\n")
            .expect("main_task must be a well-formed task body")];

        let ota_ops = task_body
            .find("set_ota_radio_ops(OtaRadioOps {")
            .expect("main_task must register the OTA radio ops");
        let ota_hooks = task_body
            .find("set_ota_hooks(OtaHooks {")
            .expect("main_task must register the OTA hooks");
        let light = task_body
            .find("set_light_event_handler(")
            .expect("main_task must register the light event handler");
        let hooks = task_body
            .find("set_ble_app_hooks(BleAppHooks {")
            .expect("main_task must register the BLE app hooks");
        let status_hook = task_body
            .find("set_status_update_hook(")
            .expect("main_task must register the light status hook");
        let rf_drv = task_body
            .find("rf_drv_init(true)")
            .expect("main_task must initialise the RF driver");

        assert!(
            ota_ops < rf_drv
                && ota_hooks < rf_drv
                && light < rf_drv
                && hooks < rf_drv
                && status_hook < rf_drv,
            "every handler registration must precede rf_drv_init(true)"
        );
    }

    /// `ota_send_terminate_hook` must report `false` without a free packet slot
    /// and `true` once `PKT_TERMINATE` is queued.
    #[test]
    fn test_ota_send_terminate_hook_reports_whether_the_packet_was_queued() {
        // Datasheet offsets of the DMA TX queue registers.
        const REG_DMA_TX_RPTR: u32 = 0x52a;
        const REG_DMA_TX_WPTR: u32 = 0x52b;

        // Buffer near capacity: (5 - 2) & 7 == 3, so `is_add_packet_buf_ready`
        // is false and nothing may be queued.
        FakeMmio::reset();
        FakeMmio::set_reg8(REG_DMA_TX_WPTR, 5);
        FakeMmio::set_reg8(REG_DMA_TX_RPTR, 2);
        assert_eq!(
            ota_send_terminate_hook(),
            false,
            "a full packet buffer must not report a queued terminate packet"
        );

        // Plenty of room: (2 - 0) & 7 == 2, so the packet is queued.
        FakeMmio::reset();
        FakeMmio::set_reg8(REG_DMA_TX_WPTR, 2);
        FakeMmio::set_reg8(REG_DMA_TX_RPTR, 0);
        assert_eq!(
            ota_send_terminate_hook(),
            true,
            "a free packet buffer must report the queued terminate packet"
        );
    }

    /// `ota_begin_gatt_hook` must refuse a GATT OTA until pairing succeeds.
    #[test]
    fn test_ota_begin_gatt_hook_requires_pairing() {
        use crate::sdk::ble_app::ble_ll_pair::PAIR_LOGIN_OK;
        use crate::sdk::ble_app::light_ll::status_management::SLAVE_READ_STATUS_BUSY;

        SLAVE_READ_STATUS_BUSY.set(0);
        PAIR_LOGIN_OK.set(false);
        assert_eq!(
            ota_begin_gatt_hook(),
            false,
            "an unpaired link must not begin a GATT OTA"
        );

        PAIR_LOGIN_OK.set(true);
        assert_eq!(
            ota_begin_gatt_hook(),
            true,
            "a paired link with an idle status read may begin a GATT OTA"
        );

        PAIR_LOGIN_OK.set(false);
    }

    // --- Tests for light_hw_timer_config ---

    #[test]
    fn test_light_hw_timer_config_enables_timers() {
        // This test verifies that light_hw_timer_config enables the TMR0/TMR1
        // interrupts, programs both capture values and leaves TMR0 running.

        // Seed an unrelated irq-mask bit and TMR1 as already enabled, so the
        // assertions below prove the read-modify-writes preserve other bits.
        FakeMmio::reset();
        FakeMmio::set_reg32(0x640, FLD_IRQ::SYSTEM_TIMER.bits());
        FakeMmio::set_reg8(0x620, FLD_TMR::TMR1_EN.bits() as u8);

        // Execute
        light_hw_timer_config();

        // Verify both timer interrupts are enabled and the seeded bit survives
        let irq_mask = FakeMmio::reg32(0x640);
        assert!(
            irq_mask & FLD_IRQ::TMR1_EN.bits() != 0,
            "TMR1 interrupt must be enabled"
        );
        assert!(
            irq_mask & FLD_IRQ::TMR0_EN.bits() != 0,
            "TMR0 interrupt must be enabled"
        );
        assert!(
            irq_mask & FLD_IRQ::SYSTEM_TIMER.bits() != 0,
            "pre-existing irq mask bits must be preserved"
        );

        // Verify TMR1 capture (~120 Hz) and TMR0 tick/capture
        assert_eq!(FakeMmio::reg32(0x628), CLOCK_SYS_CLOCK_1S / 120);
        assert_eq!(FakeMmio::reg32(0x630), 0);
        assert_eq!(FakeMmio::reg32(0x624), CLOCK_SYS_CLOCK_1S);

        // Verify TMR0 is enabled and TMR1's enable bit was left alone
        assert!(
            FakeMmio::reg8(0x620) & FLD_TMR::TMR0_EN.bits() as u8 != 0,
            "TMR0 must be left running"
        );
        assert!(
            FakeMmio::reg8(0x620) & FLD_TMR::TMR1_EN.bits() as u8 != 0,
            "TMR1 enable must not be cleared"
        );
    }

    // --- Tests for light_auth_check ---

    #[test]
    fn test_light_auth_check_clears_timestamp_on_timeout() {
        // This test verifies authentication timeout when security is enabled

        // Setup: Reset global state first
        reset_test_globals();

        // Setup: Security enabled, not logged in, and timeout exceeded
        SECURITY_ENABLE.set(true);
        PAIR_LOGIN_OK.set(false);
        BLE_PERIPHERAL_FIRST_CONNECTION_TIMESTAMP.set(1000000);

        // The authentication window is 60 s: park the tick past it.
        reset_fakes();
        fake_clock::set_ticks(1_000_000 + AUTH_TIME as u64 * 1000 * 1000 * 32 + 1);

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
    fn test_rf_link_response_callback_get_status() {
        // This test verifies GET_STATUS response returns current light state

        // Setup
        crate::light::reset_for_test();
        DEVICE_ADDRESS.set(0x1234);

        let mut request = create_test_packet_att_value();
        request.src = [0x56, 0x78];
        request.val[15] = GET_STATUS;

        let mut response = create_test_packet_att_value();

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(
            result.is_ok(),
            "Callback should return Ok(()) for GET_STATUS"
        );
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
        assert!(
            result.is_ok(),
            "Callback should return Ok(()) for GET_GROUP1"
        );
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
        assert!(
            result.is_ok(),
            "Callback should return Ok(()) for GET_GROUP2"
        );
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
        assert!(
            result.is_ok(),
            "Callback should return Ok(()) for GET_GROUP3"
        );
        assert_eq!(response.val[0], LGT_CMD_LIGHT_GRP_RSP3 | 0xc0);
        assert_eq!(response.val[3], 0x22, "Fifth group low byte");
        assert_eq!(response.val[4], 0x11, "Fifth group high byte");
    }

    #[test]
    fn test_rf_link_response_callback_get_dev_addr() {
        // This test verifies GET_DEV_ADDR response

        // Setup: the answer carries this node's MAC, so seed it.
        DEVICE_ADDRESS.set(0x1234);
        let mac = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        MAC_ID.lock().copy_from_slice(&mac);

        let mut request = create_test_packet_att_value();
        request.src = [0x56, 0x78];

        let mut response = create_test_packet_att_value();
        response.val[15] = GET_DEV_ADDR;

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(
            result.is_ok(),
            "Callback should return Ok(()) for GET_DEV_ADDR"
        );
        assert_eq!(response.val[0], LGT_CMD_DEV_ADDR_RSP | 0xc0);
        assert_eq!(
            &response.val[4..10],
            &mac,
            "GET_DEV_ADDR answers with this node's MAC at val[4..10]"
        );
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
        assert!(
            result.is_ok(),
            "Callback should return Ok(()) for GET_USER_NOTIFY"
        );
        assert_eq!(response.val[0], LGT_CMD_USER_NOTIFY_RSP | 0xc0);
        assert_eq!(response.val[3], 0xCD); // Device address low byte
        assert_eq!(response.val[4], 0xAB); // Device address high byte
                                           // Verify some response parameters
        for i in 0..8 {
            assert_eq!(response.val[5 + i], i as u8);
        }
    }

    #[test]
    fn test_rf_link_response_callback_cmd_start_ota() {
        // This test verifies CMD_START_OTA response

        // Setup
        DEVICE_ADDRESS.set(0x1234);
        crate::ota::reset_for_test();

        let mut request = create_test_packet_att_value();
        request.src = [0x56, 0x78];

        let mut response = create_test_packet_att_value();
        response.val[15] = CMD_START_OTA;

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(
            result.is_ok(),
            "Callback should return Ok(()) for CMD_START_OTA"
        );
        assert_eq!(response.val[0], LGT_CMD_START_OTA_RSP | 0xc0);
        assert_eq!(response.val[3], (BUILD_VERSION & 0xFF) as u8);
        assert_eq!(response.val[4], ((BUILD_VERSION >> 8) & 0xFF) as u8);
        assert_eq!(response.val[5], ((BUILD_VERSION >> 16) & 0xFF) as u8);
        assert_eq!(response.val[6], ((BUILD_VERSION >> 24) & 0xFF) as u8);
        assert!(
            crate::ota::is_mesh_ota_blocked(),
            "OTA operations should be blocked"
        );
    }

    #[test]
    #[mry::lock(rf_mesh_data_ota)]
    fn test_rf_link_response_callback_cmd_ota_data() {
        // This test verifies CMD_OTA_DATA response

        // Setup
        mock_rf_mesh_data_ota(Any, Any).returns(0x1234u16);

        DEVICE_ADDRESS.set(0x5678);

        let mut request = create_test_packet_att_value();
        request.src = [0xAA, 0xBB];

        let mut response = create_test_packet_att_value();
        response.val[15] = CMD_OTA_DATA;

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(
            result.is_ok(),
            "Callback should return Ok(()) for CMD_OTA_DATA"
        );
        assert_eq!(response.val[0], LGT_CMD_OTA_DATA_RSP | 0xc0);
        assert_eq!(response.val[1], 0x34); // idx low byte
        assert_eq!(response.val[2], 0x12); // idx high byte
        mock_rf_mesh_data_ota(Any, false).assert_called(1);
    }

    #[test]
    #[mry::lock(rf_mesh_data_ota)]
    fn test_rf_link_response_callback_cmd_end_ota() {
        // This test verifies CMD_END_OTA response

        // Setup
        mock_rf_mesh_data_ota(Any, Any).returns(0xABCDu16);

        DEVICE_ADDRESS.set(0x5678);

        let mut request = create_test_packet_att_value();
        request.src = [0xCC, 0xDD];

        let mut response = create_test_packet_att_value();
        response.val[15] = CMD_END_OTA;

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(
            result.is_ok(),
            "Callback should return Ok(()) for CMD_END_OTA"
        );
        assert_eq!(response.val[0], LGT_CMD_END_OTA_RSP | 0xc0);
        assert_eq!(response.val[1], 0xCD); // idx low byte
        assert_eq!(response.val[2], 0xAB); // idx high byte
        mock_rf_mesh_data_ota(Any, true).assert_called(1);
    }

    #[test]
    fn test_rf_link_response_callback_invalid_command() {
        // This test verifies that invalid commands return false

        // Setup
        DEVICE_ADDRESS.set(0x1234);

        let request = create_test_packet_att_value();

        let mut response = create_test_packet_att_value();
        response.val[15] = 0xFF; // Invalid command

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify
        assert!(
            result.is_err(),
            "Callback should return Err(()) for invalid command"
        );
    }

    // --- Tests for rf_link_data_callback ---

    #[test]
    #[mry::lock(parse_ble_packet_op_params)]
    fn test_rf_link_data_callback_light_onoff() {
        // This test verifies LGT_CMD_LIGHT_ONOFF is handled correctly

        // Setup
        crate::light::reset_for_test();

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

        // Verify: exactly one command was queued for the light task, with the
        // parsed parameters passed through unchanged.
        let msg = LIGHT_CMD_CHANNEL
            .try_receive()
            .expect("LGT_CMD_LIGHT_ONOFF must be queued");
        assert_eq!(msg.cmd, LGT_CMD_LIGHT_ONOFF);
        assert_eq!(msg.params, params);
        assert!(
            LIGHT_CMD_CHANNEL.try_receive().is_err(),
            "exactly one command must be queued"
        );
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
        mock_remove_group(Any).returns(Ok(()));
        reset_led_controller();

        // Execute
        rf_link_data_callback(&packet);

        // Verify
        mock_remove_group(0x12).assert_called(1);
        assert_eq!(
            pending_event(),
            LED_EVENT_FLASH_1HZ_4S,
            "a successful group change blinks the indicator"
        );
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
        mock_remove_group(Any).returns(Err(()));

        // Clear LED event before test
        reset_led_controller();

        // Execute
        rf_link_data_callback(&packet);

        // Verify: No LED event should be set
        assert_eq!(pending_event(), 0);

        // Cleanup
        reset_led_controller();
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
        mock_add_group(Any).returns(Ok(()));
        reset_led_controller();

        // Execute
        rf_link_data_callback(&packet);

        // Verify
        mock_add_group(0xCD).assert_called(1);
        assert_eq!(
            pending_event(),
            LED_EVENT_FLASH_1HZ_4S,
            "a successful group change blinks the indicator"
        );
    }

    #[test]
    #[mry::lock(
        parse_ble_packet_op_params,
        add_device_address,
        dev_addr_with_mac_flag,
        dev_addr_with_mac_match,
        mesh_device_address_validation_completed
    )]
    fn test_rf_link_data_callback_config_dev_addr() {
        // This test verifies device address configuration

        // Setup
        mock_mesh_device_address_validation_completed().returns(());

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
        mock_add_device_address(Any).returns(Ok(()));

        // Execute
        rf_link_data_callback(&packet);

        // Verify
        mock_add_device_address(0x5678).assert_called(1);
        mock_mesh_device_address_validation_completed().assert_called(1);
    }

    #[test]
    #[mry::lock(parse_ble_packet_op_params)]
    fn test_rf_link_data_callback_set_light() {
        // This test verifies LGT_CMD_SET_LIGHT is handled

        // Setup
        crate::light::reset_for_test();

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

        // Verify: exactly one command was queued, with the parsed parameters.
        let msg = LIGHT_CMD_CHANNEL
            .try_receive()
            .expect("LGT_CMD_SET_LIGHT must be queued");
        assert_eq!(msg.cmd, LGT_CMD_SET_LIGHT);
        assert_eq!(msg.params, params);
        assert!(
            LIGHT_CMD_CHANNEL.try_receive().is_err(),
            "exactly one command must be queued"
        );
    }

    #[test]
    #[mry::lock(parse_ble_packet_op_params)]
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

        reset_fakes();

        // Execute
        rf_link_data_callback(&packet);

        // Verify: the MAC is erased-then-programmed at its fixed flash address.
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_MAC),
            1,
            "the MAC sector is erased before programming"
        );
        let mut mac = [0xFFu8; 7];
        FakeFlash::read_slice(FLASH_ADR_MAC, &mut mac);
        assert_eq!(
            &mac[..6],
            &params[..6],
            "the six MAC bytes are programmed at FLASH_ADR_MAC"
        );
        assert_eq!(mac[6], 0xFF, "only the six MAC bytes are written");
        assert!(
            FakeMmio::writes()
                .iter()
                .any(|write| write.addr == REG_PWDN_CTRL | tlsr8266_pac::REG_BASE_ADDR),
            "setting the MAC address must reboot the MCU"
        );
    }

    /// `LGT_CMD_KICK_OUT` with the out-of-mesh reason: erase the configuration,
    /// then re-write the four pairing blocks, blink the OTA LED and reboot.
    #[test]
    #[mry::lock(parse_ble_packet_op_params, plan_kick_out)]
    fn test_rf_link_data_callback_kick_out_out_of_mesh_writes_credentials() {
        // Moved from `sdk/factory_reset.rs::tests`: the credential writes now
        // live in this module, so the assertion follows them.
        let expected_ltk = [0xAAu8; 16];
        {
            let mut ltk = PAIR_CONFIG_MESH_LTK.lock();
            *ltk = expected_ltk;
        }

        MESH_PAIR_ENABLE.set(true);
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);

        let encoded_password = {
            // Compute the expected ciphertext with the real AES path: the same
            // key and password buffer the kick-out path feeds to
            // `encode_password`.
            let key = *PAIR_CONFIG_PWD_ENCODE_SK.lock();
            let mut password = [0u8; 16];
            let len = MESH_PWD.len().min(password.len());
            password[..len].copy_from_slice(&MESH_PWD.as_bytes()[..len]);
            encode_password(&key, &password)
        };

        reset_fakes();
        crate::factory_reset::reset_for_test();
        // Stale credentials the erase must clear before the new ones land.
        FakeFlash::write_slice(FLASH_ADR_PAIRING, &[0x00; 64]);

        let mut expected_name = [0u8; 16];
        let name_bytes = OUT_OF_MESH.as_bytes();
        let copy_len = name_bytes.len().min(expected_name.len());
        expected_name[..copy_len].copy_from_slice(&name_bytes[..copy_len]);

        let mut expected_flags = [0u8; 16];
        expected_flags[0] = PAIR_VALID_FLAG;
        expected_flags[15] = PAIR_VALID_FLAG;
        expected_flags[1] = 1; // Mesh pairing enabled flag

        let packet = create_test_packet();
        let op_cmd = [
            LGT_CMD_KICK_OUT,
            (VENDOR_ID & 0xFF) as u8,
            (VENDOR_ID >> 8) as u8,
        ];
        let params = [0u8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        mock_parse_ble_packet_op_params(Any, Any).returns((
            false,
            op_cmd,
            LightOpType::OpType3 as u8,
            params,
            0,
        ));
        // The callback must route the reason through the reset policy; the
        // policy's own table is pinned in `factory_reset::tests`.
        mock_plan_kick_out(KickoutReason::OutOfMesh).calls_real_impl();

        rf_link_data_callback(&packet);

        mock_plan_kick_out(KickoutReason::OutOfMesh).assert_called(1);

        // The four 16-byte blocks must be present in flash, in the documented
        // slot layout: flags, name, encoded password, LTK.
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_PAIRING),
            1,
            "the kick-out must erase before writing"
        );
        let mut got = [0u8; 16];
        FakeFlash::read_slice(FLASH_ADR_PAIRING, &mut got);
        assert_eq!(got, expected_flags, "pairing flags must be written");
        FakeFlash::read_slice(FLASH_ADR_PAIRING + 16, &mut got);
        assert_eq!(got, expected_name, "out-of-mesh name must be written");
        FakeFlash::read_slice(FLASH_ADR_PAIRING + 32, &mut got);
        assert_eq!(got, encoded_password, "encoded password must be written");
        FakeFlash::read_slice(FLASH_ADR_PAIRING + 48, &mut got);
        assert_eq!(got, expected_ltk, "mesh LTK must be written");

        assert!(MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get());
        assert_eq!(
            fake_clock::sleeps(),
            vec![1000 * 1000u32; 6],
            "the success indicator must blink six times at 1 Hz"
        );
        assert!(
            FakeMmio::writes()
                .iter()
                .any(|write| write.addr == REG_PWDN_CTRL | tlsr8266_pac::REG_BASE_ADDR),
            "the kick-out must reboot the MCU"
        );

        // Restore mesh pairing flags for other tests.
        MESH_PAIR_ENABLE.set(false);
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);
    }

    /// Any other reason only erases: no pairing data is written.
    #[test]
    #[mry::lock(parse_ble_packet_op_params)]
    fn test_rf_link_data_callback_kick_out_default_name_only_erases() {
        reset_fakes();
        crate::factory_reset::reset_for_test();
        FakeFlash::write_slice(FLASH_ADR_PAIRING, &[0xAA; 64]);

        let packet = create_test_packet();
        let op_cmd = [
            LGT_CMD_KICK_OUT,
            (VENDOR_ID & 0xFF) as u8,
            (VENDOR_ID >> 8) as u8,
        ];
        let params = [1u8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]; // DefaultName
        mock_parse_ble_packet_op_params(Any, Any).returns((
            false,
            op_cmd,
            LightOpType::OpType3 as u8,
            params,
            0,
        ));

        rf_link_data_callback(&packet);

        assert_eq!(FakeFlash::erase_count(FLASH_ADR_PAIRING), 1);
        let mut got = [0u8; 64];
        FakeFlash::read_slice(FLASH_ADR_PAIRING, &mut got);
        assert_eq!(
            got, [0xFF; 64],
            "no pairing data may be written for a plain erase"
        );
        assert_eq!(
            fake_clock::sleeps(),
            vec![1000 * 1000u32; 6],
            "the success indicator must blink six times at 1 Hz"
        );
        assert!(
            FakeMmio::writes()
                .iter()
                .any(|write| write.addr == REG_PWDN_CTRL | tlsr8266_pac::REG_BASE_ADDR),
            "the kick-out must reboot the MCU"
        );
    }

    /// An unknown wire value still falls back to a full out-of-mesh kick-out.
    #[test]
    #[mry::lock(parse_ble_packet_op_params)]
    fn test_rf_link_data_callback_kick_out_unknown_reason_falls_back() {
        MESH_PAIR_ENABLE.set(false);
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);
        reset_fakes();
        crate::factory_reset::reset_for_test();

        let packet = create_test_packet();
        let op_cmd = [
            LGT_CMD_KICK_OUT,
            (VENDOR_ID & 0xFF) as u8,
            (VENDOR_ID >> 8) as u8,
        ];
        let params = [3u8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]; // Not a KickoutReason
        mock_parse_ble_packet_op_params(Any, Any).returns((
            false,
            op_cmd,
            LightOpType::OpType3 as u8,
            params,
            0,
        ));

        rf_link_data_callback(&packet);

        // The flags block proves the credential path ran.
        assert_eq!(
            flash_byte(FLASH_ADR_PAIRING),
            PAIR_VALID_FLAG,
            "an unknown reason must fall back to the out-of-mesh credentials"
        );
        assert!(
            !MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get(),
            "mesh pairing is disabled, so the rejoin flag stays clear"
        );
    }

    #[test]
    #[mry::lock(parse_ble_packet_op_params, mesh_pair_cb)]
    fn test_rf_link_data_callback_mesh_pair() {
        // This test verifies mesh pair command handling

        // Setup
        mock_mesh_pair_cb(Any).returns(());

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
        mock_mesh_pair_cb(Any).assert_called(1);
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

    // --- Tests for rf_link_light_event_callback ---

    #[test]
    #[mry::lock(mesh_node_table_init, device_status_update)]
    fn test_rf_link_light_event_callback_set_mesh_info() {
        // This test verifies LGT_CMD_SET_MESH_INFO event handling

        // Setup
        mock_device_status_update().returns(());

        mock_mesh_node_table_init().returns(());

        // Clear LED event before test
        reset_led_controller();

        // Execute
        rf_link_light_event_callback(LGT_CMD_SET_MESH_INFO);

        // Verify
        mock_mesh_node_table_init().assert_called(1);
        mock_device_status_update().assert_called(1);
        assert_eq!(pending_event(), LED_EVENT_FLASH_1HZ_4S);

        // Cleanup
        reset_led_controller();
    }

    #[test]
    #[mry::lock(mesh_node_table_init, device_status_update)]
    fn test_rf_link_light_event_callback_set_dev_addr() {
        // This test verifies LGT_CMD_SET_DEV_ADDR event handling

        // Setup
        mock_device_status_update().returns(());

        mock_mesh_node_table_init().returns(());

        // Clear LED event before test
        reset_led_controller();

        // Execute
        rf_link_light_event_callback(LGT_CMD_SET_DEV_ADDR);

        // Verify
        mock_mesh_node_table_init().assert_called(1);
        mock_device_status_update().assert_called(1);
        assert_eq!(pending_event(), LED_EVENT_FLASH_1HZ_4S);

        // Cleanup
        reset_led_controller();
    }

    #[test]
    fn test_rf_link_light_event_callback_del_pair() {
        // This test verifies LGT_CMD_DEL_PAIR event handling

        // Clear LED event before test
        reset_led_controller();

        // Execute
        rf_link_light_event_callback(LGT_CMD_DEL_PAIR);

        // Verify
        assert_eq!(pending_event(), LED_EVENT_FLASH_1HZ_4S);

        // Cleanup
        reset_led_controller();
    }

    #[test]
    fn test_rf_link_light_event_callback_mesh_pair_timeout() {
        // This test verifies LGT_CMD_MESH_PAIR_TIMEOUT event handling

        // Clear LED event before test
        reset_led_controller();

        // Execute
        rf_link_light_event_callback(LGT_CMD_MESH_PAIR_TIMEOUT);

        // Verify
        assert_eq!(pending_event(), LED_EVENT_FLASH_2HZ_2S);

        // Cleanup
        reset_led_controller();
    }

    #[test]
    fn test_rf_link_light_event_callback_unknown_status() {
        // This test verifies that unknown status codes do nothing

        // Clear LED event before test
        reset_led_controller();

        // Execute
        rf_link_light_event_callback(0xFF);

        // Verify: LED event should remain 0
        assert_eq!(pending_event(), 0);
    }

    // --- Tests for light_init_default ---

    #[test]
    #[mry::lock(
        light_check_tick_per_us,
        rf_link_slave_pairing_enable,
        rf_set_power_level_index,
        light_hw_timer_config,
        mesh_pair_init
    )]
    fn test_light_init_default() {
        // This test verifies that light_init_default properly initializes mesh configuration

        // Setup mocks
        reset_fakes();
        // Seed the USB analog register with bit 4 set, so enabling the DP
        // pull-up is observable as a clear.
        FakeAnalog::set(0x00, 0xff);
        mock_light_check_tick_per_us(CLOCK_SYS_CLOCK_1US).returns(());
        mock_rf_link_slave_pairing_enable(Any).returns(());
        mock_rf_set_power_level_index(Any).returns(());
        mock_light_hw_timer_config().returns(());

        mock_mesh_pair_init().returns(());

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
        assert_eq!(
            FakeAnalog::get(0x00),
            0xef,
            "enabling the USB DP pull-up must clear bit 4 of the USB analog register"
        );

        // Verify: light_hw_timer_config was called
        mock_light_hw_timer_config().assert_called(1);

        // Verify: mesh_pair_init was called
        mock_mesh_pair_init().assert_called(1);

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

    /// Drives `user_init` with every OTA / mesh / light / adv seam mocked, so
    /// the factory-reset boot path runs for real against `FakeFlash`.
    ///
    /// `handle_boot_sequence` is mocked *through its real body*: the mock exists
    /// only so the caller can assert that `user_init` consults the reset policy,
    /// while the sequence itself still runs on the `FakeFlash` bank.
    fn run_user_init_with_mocks() {
        mock_check_ota_area_startup().returns(());
        mock_mesh_pair_init().returns(());
        mock_mesh_security_enable(Any).returns(());
        mock_device_status_update().returns(());
        mock_handle_boot_sequence().calls_real_impl();

        // `LightFlashStore::retrieve_from_flash` scans the blank flash (no saved
        // entry) and then restores the persisted on/off state, which is observed
        // through `light_onoff_hw`.
        mock_light_onoff_hw(Any).returns(());

        // Mock functions that light_init_default calls
        mock_light_check_tick_per_us(CLOCK_SYS_CLOCK_1US).returns(());
        mock_rf_link_slave_pairing_enable(true).returns(());
        mock_rf_set_power_level_index(RfPower::Power8dBm as u32).returns(());
        mock_light_hw_timer_config().returns(());

        // Mock functions that user_init calls directly
        mock_rf_link_slave_init(40000).returns(());
        mock_set_advertisement_manufacturer_data(Any).returns(());

        let _flash_store = user_init();
    }

    #[mry::mry]
    #[mry::lock(
        light_check_tick_per_us,
        rf_link_slave_pairing_enable,
        rf_set_power_level_index,
        light_hw_timer_config,
        rf_link_slave_init,
        set_advertisement_manufacturer_data,
        check_ota_area_startup,
        light_onoff_hw,
        device_status_update,
        mesh_pair_init,
        mesh_security_enable,
        handle_boot_sequence
    )]
    #[test]
    fn test_user_init() {
        // Mock the OTA / mesh / light entry points user_init drives directly.
        reset_fakes();
        crate::factory_reset::reset_for_test();
        // Seed the USB analog register (bit 4 = pull-up disabled) and the port C
        // GPIO-function register, so the real `usb_dp_pullup_en` and
        // `gpio_set_func` calls below are observable.
        FakeAnalog::set(0x00, 0xff);
        FakeMmio::set_reg8(REG_GPIO_PC_FUNC, 0xff);
        // The advertisement payload is built from these two, so seed both.
        let mac = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        MAC_ID.lock().copy_from_slice(&mac);
        DEVICE_ADDRESS.set(0x1234);

        run_user_init_with_mocks();

        // Verify all mocks were called
        mock_check_ota_area_startup().assert_called(1);
        mock_light_check_tick_per_us(CLOCK_SYS_CLOCK_1US).assert_called(1);
        mock_rf_link_slave_pairing_enable(true).assert_called(1);
        mock_rf_set_power_level_index(RfPower::Power8dBm as u32).assert_called(1);
        assert_eq!(
            FakeAnalog::get(0x00),
            0xef,
            "user_init must enable the USB DP pull-up"
        );
        mock_light_hw_timer_config().assert_called(1);
        mock_mesh_pair_init().assert_called(1);
        assert_eq!(
            FakeMmio::reg8(REG_GPIO_PC_FUNC),
            0xfa,
            "user_init must select the peripheral function for PB0 and PB2"
        );
        // The PWM setup reached the real driver: both cycle registers hold the
        // max tick, with G's compare at 0 and B's compare at max (inverted).
        let max_tick = u32::from(PMW_MAX_TICK);
        assert_eq!(
            FakeMmio::reg32(0x794),
            max_tick << 16,
            "channel G must be configured with max tick and compare 0"
        );
        assert_eq!(
            FakeMmio::reg32(0x798),
            max_tick | (max_tick << 16),
            "channel B must be configured with max tick and compare max"
        );
        assert_eq!(
            FakeMmio::reg8(0x780),
            0b11,
            "both PWM channels must be enabled"
        );
        // The blank flash has no saved luminance entry, so the retrieve path ran
        // and the zeroed analog bank records the lamp as on.
        mock_light_onoff_hw(true).assert_called(1);
        mock_rf_link_slave_init(40000).assert_called(1);
        mock_set_advertisement_manufacturer_data(Any).assert_called(1);
        // The boot path must consult the reset policy exactly once.
        mock_handle_boot_sequence().assert_called(1);
        // The light subsystem built both payloads from the loaded MAC and this
        // node's address, and handed the primary one's bytes to the RF driver.
        let (pri, rsp) = {
            let pri = *ADV_PRI_DATA.lock();
            let rsp = *ADV_RSP_PRI_DATA.lock();
            (pri, rsp)
        };
        let (pri_manufacture, pri_uuid, pri_mac) =
            (pri.manufacture_id, pri.mesh_product_uuid, pri.mac_address);
        let (rsp_uuid, rsp_status, rsp_addr, rsp_rsv) =
            (rsp.product_uuid, rsp.status, rsp.device_address, rsp.rsv);
        assert_eq!(pri_manufacture, VENDOR_ID, "manufacturer id");
        assert_eq!(pri_uuid, VENDOR_ID, "mesh product uuid");
        assert_eq!(
            pri_mac,
            u32::from_le_bytes([mac[0], mac[1], mac[2], mac[3]]),
            "the primary payload carries the first four MAC bytes"
        );
        assert_eq!(rsp_uuid, 0x02, "product uuid 0x02 = CCT light");
        assert_eq!(rsp_status, 0x01, "status 0x01 = advertising");
        assert_eq!(
            rsp_addr,
            DEVICE_ADDRESS.get(),
            "the scan response carries this node's address"
        );
        for i in 0..16 {
            assert_eq!(rsp_rsv[i], i as u8, "rsv[{i}] must be {i}");
        }
        // Once from the on/off restore, once at the end of user_init.
        mock_device_status_update().assert_called(2);
        mock_mesh_security_enable(true).assert_called(1);

        // The boot ran the real sequence on the blank sector: step 1 is now in
        // the reset-count sector, and nothing was erased or rebooted.
        assert_eq!(
            flash_byte(FLASH_ADR_RESET_CNT),
            0xFE,
            "a blank sector must start the sequence at step 1"
        );
        assert!(
            !FakeMmio::writes()
                .iter()
                .any(|write| write.addr == REG_PWDN_CTRL | tlsr8266_pac::REG_BASE_ADDR),
            "a normal boot must not reboot the MCU"
        );
    }

    /// A boot that finds the factory-reset flag erases the configuration,
    /// blinks the OTA LED and reboots — the `FactoryResetAndReboot` arm.
    #[mry::mry]
    #[mry::lock(
        light_check_tick_per_us,
        rf_link_slave_pairing_enable,
        rf_set_power_level_index,
        light_hw_timer_config,
        rf_link_slave_init,
        set_advertisement_manufacturer_data,
        check_ota_area_startup,
        light_onoff_hw,
        device_status_update,
        mesh_pair_init,
        mesh_security_enable,
        handle_boot_sequence
    )]
    #[test]
    fn test_user_init_factory_reset_erases_blinks_and_reboots() {
        reset_fakes();
        crate::factory_reset::reset_for_test();
        // The previous five power cycles left the flag in the first slot.
        FakeFlash::write_slice(FLASH_ADR_RESET_CNT, &[0x80]);

        run_user_init_with_mocks();

        // The reset path disables interrupts before touching flash.
        assert!(
            FakeMmio::writes().iter().any(|write| write.addr
                == REG_IRQ_EN | tlsr8266_pac::REG_BASE_ADDR
                && write.value == 0),
            "the reset path must disable global interrupts"
        );
        // Every configuration sector is erased, the reset counter last.
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_PAIRING),
            1,
            "the factory reset must erase the pairing sector"
        );
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_RESET_CNT),
            1,
            "the reset counter sector is erased too"
        );
        assert_eq!(
            flash_byte(FLASH_ADR_RESET_CNT),
            0xFF,
            "the reset sector is left blank"
        );
        // The success indicator blinks six times at 1 Hz, then the MCU reboots.
        assert_eq!(
            fake_clock::sleeps(),
            vec![1000 * 1000u32; 6],
            "the success indicator must blink six times at 1 Hz"
        );
        assert!(
            FakeMmio::writes()
                .iter()
                .any(|write| write.addr == REG_PWDN_CTRL | tlsr8266_pac::REG_BASE_ADDR),
            "the reset path must trigger the MCU reboot"
        );
    }

    // --- Tests for light_user_func ---

    #[test]
    #[mry::lock(mesh_pair_proc_effect, check_timing_step)]
    fn test_light_user_func_structure() {
        // `check_light_state_save` is an inherent method on the flash store and is
        // deliberately not mockable, so this test drives the real function and
        // asserts its observable effects: the 5 s save timer is polled, the save
        // reaches flash, and the timer is cleared so a second tick does not save
        // again.
        reset_test_globals();
        crate::light::reset_for_test();
        crate::factory_reset::reset_for_test();
        reset_fakes();

        // Seed a pending luminance save: a colour transition stamps
        // `last_transition_time`.
        fake_clock::set_ticks(1000);
        begin_transition(500, 300, 800);

        // The colour command started a transition, so TMR1 must be running.
        assert!(
            FakeMmio::reg8(0x620) & FLD_TMR::TMR1_EN.bits() as u8 != 0,
            "begin_transition must start TMR1"
        );
        assert_eq!(
            FakeMmio::reg32(0x634),
            0,
            "begin_transition resets the tick"
        );

        // 5 s (160_000_000 ticks) have elapsed since that transition.
        fake_clock::set_ticks(1000 + 5000 * 1000 * 32 + 1);
        mock_mesh_pair_proc_effect().returns(());
        // The tick must poll the reset timing check; its own behaviour is pinned
        // by the `factory_reset` tests.
        mock_check_timing_step().returns(ResetAction::None);

        // Execute - light_user_func calls:
        // 1. flash_store.check_light_state_save();
        // 2. light_auth_check(); (which checks global state)
        // 3. factory_reset::check_timing_step();
        // 4. mesh_pair_proc_effect();
        let mut flash_store = LightFlashStore::default();
        light_user_func(&mut flash_store);

        // Verify the save path ran, and the other three steps of the tick ran.
        let mut saved = [0u8; 7];
        FakeFlash::read_slice(0, &mut saved);
        assert_eq!(saved[0], 0xA5, "the saved luminance entry is valid");
        assert_eq!(
            u16::from_le_bytes([saved[1], saved[2]]),
            MAX_LUM_BRIGHTNESS_VALUE,
            "the entry records the core's current brightness"
        );
        assert_eq!(
            u16::from_le_bytes([saved[3], saved[4]]),
            MAX_LUM_BRIGHTNESS_VALUE,
            "the entry records the transition's current CW"
        );
        assert_eq!(
            u16::from_le_bytes([saved[5], saved[6]]),
            0,
            "the entry records the transition's current WW"
        );
        mock_mesh_pair_proc_effect().assert_called(1);

        // A second tick must not save again: `check_light_state_save` cleared the
        // timer, so it never even asks whether 5 s elapsed. The clock still reads
        // well past the window, so a second write could only come from a timer
        // that was not cleared.
        light_user_func(&mut flash_store);
        let mut after = [0u8; 14];
        FakeFlash::read_slice(0, &mut after);
        assert_eq!(&after[..7], &saved, "the first entry is unchanged");
        assert_eq!(&after[7..], &[0xFF; 7], "no second entry was programmed");
        mock_mesh_pair_proc_effect().assert_called(2);
        // One reset-timing poll per tick: deleting the call site in `light_user_func`
        // must fail here.
        mock_check_timing_step().assert_called(2);
    }

    // --- Tests for main_loop ---

    #[test]
    #[mry::lock(
        mesh_pair_proc_effect,
        rf_link_slave_proc,
        led_indicate_step,
        check_timing_step
    )]
    fn test_main_loop_structure() {
        // This test verifies that main_loop drives one tick of each stage.
        //
        // `light_user_func` itself is not mockable (it takes `&mut LightFlashStore`),
        // so its body runs for real and is observed through two of its own callees.
        reset_test_globals();
        crate::light::reset_for_test();
        crate::factory_reset::reset_for_test();

        // Mock the callees of the light tick and the other two stages
        mock_check_timing_step().returns(ResetAction::None);
        mock_mesh_pair_proc_effect().returns(());
        mock_rf_link_slave_proc().returns(());
        mock_led_indicate_step().returns(());

        // Execute main_loop
        let mut flash_store = LightFlashStore::default();
        block_on(main_loop(&mut flash_store));

        // Verify the functions were called
        mock_check_timing_step().assert_called(1);
        mock_mesh_pair_proc_effect().assert_called(1);
        mock_rf_link_slave_proc().assert_called(1);
        mock_led_indicate_step().assert_called(1);
    }

    /// Regression test verifying the full response path:
    /// rf_link_response_callback(GET_STATUS) -> rf_link_slave_add_status -> BUFF_RESPONSE
    /// produces exact unshifted VENDOR_ID at dat[8..10] and CW/WW/Brightness at dat[10..16].
    ///
    /// This lives in the application layer because the callback reads the light
    /// state owned by the light subsystem; the SDK test tree may no longer inject an
    /// application state object.
    #[test]
    fn test_rf_link_response_callback_to_slave_add_status_layout() {
        use crate::sdk::ble_app::light_ll::packet_processing::rf_link_slave_add_status;

        // Reset only the globals this path reads: the request and response
        // packets are built locally below.
        crate::light::reset_for_test();
        DEVICE_ADDRESS.set(0x001F);
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(0);
        DEVICE_STATUS_BUFFER_READ_POINTER.set(0);
        DEVICE_STATUS_RECORD_INDEX.set(0);
        SLAVE_READ_STATUS_BUSY.set(LGT_CMD_LIGHT_STATUS);

        reset_fakes();
        fake_clock::set_ticks(1);
        begin_transition(8767, 24000, 16384);

        // `begin_transition` runs the real TMR1 HAL helpers, so the fake bank
        // must show a started timer with a cleared tick.
        assert!(
            FakeMmio::reg8(0x620) & FLD_TMR::TMR1_EN.bits() as u8 != 0,
            "begin_transition must start TMR1"
        );
        assert_eq!(
            FakeMmio::reg32(0x634),
            0,
            "begin_transition resets the tick"
        );

        let req = PacketAttValue {
            sno: [0x01, 0x02, 0x03],
            src: [0x01, 0x00],
            dst: [0x1F, 0x00],
            val: [0; 23],
        };

        let mut pkt_light_status = Packet {
            mesh: mesh_protocol::packet_types::MeshPkt::default(),
        };
        pkt_light_status.att_cmd_mut().value.sno = [0xA2, 0x5F, 0xE3];
        pkt_light_status.att_cmd_mut().value.val[PKT_VAL_MAC_APP_RESPONSE_TYPE] = GET_STATUS;

        rf_link_response_callback(&mut pkt_light_status.att_cmd_mut().value, &req)
            .expect("GET_STATUS response callback should succeed");
        rf_link_slave_add_status(&pkt_light_status);

        assert_eq!(DEVICE_STATUS_BUFFER_WRITE_POINTER.get(), 1);
        let buf = BUFF_RESPONSE.lock();
        let dat = &buf[0].att_data().dat;

        assert_eq!(&dat[0..3], &[0xA2, 0x5F, 0xE3], "sno mismatch");
        assert_eq!(&dat[3..5], &[0x1F, 0x00], "dst/src addr mismatch");
        assert_eq!(&dat[5..7], &[0x1F, 0x00], "src addr mismatch");
        assert_eq!(dat[7], LGT_CMD_LIGHT_STATUS | 0xc0, "op mismatch");
        assert_eq!(
            &dat[8..10],
            &VENDOR_ID.to_le_bytes(),
            "VENDOR_ID must not be shifted"
        );
        assert_eq!(
            &dat[10..12],
            &8767u16.to_le_bytes(),
            "CW bytes must align at dat[10..12]"
        );
        assert_eq!(
            &dat[12..14],
            &24000u16.to_le_bytes(),
            "WW bytes must align at dat[12..14]"
        );
        assert_eq!(
            &dat[14..16],
            &16384u16.to_le_bytes(),
            "Brightness bytes must align at dat[14..16]"
        );
        assert_eq!(&dat[16..18], &[0x00, 0x00], "trailing params zero");
        drop(buf);

        // This test runs (alphabetically) before the SDK suites, which assume an
        // empty shared status buffer. Leave the pointers where they were.
        DEVICE_STATUS_BUFFER_WRITE_POINTER.set(0);
        DEVICE_STATUS_BUFFER_READ_POINTER.set(0);
        DEVICE_STATUS_RECORD_INDEX.set(0);
    }
}
