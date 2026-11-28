#![coverage(off)]
use core::mem::size_of;
use core::ptr::addr_of;
use core::sync::atomic::Ordering;

use embassy_executor::{SpawnError, SpawnToken, Spawner};

#[cfg(not(test))]
use embassy_time::{Duration, Timer};

use crate::config::*;
use crate::diagnostics::{
    boot_init, clear_panic_report, read_panic_preview, read_panic_report, record_breadcrumb,
    BootResetReason, CRUMB_INIT_DONE, CRUMB_MAIN_LOOP,
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
    mesh_security_enable, set_mesh_event_hooks, MeshEventHooks,
};
use crate::ota::{check_ota_area_startup, rf_mesh_data_ota, set_ota_radio_ops, OtaRadioOps};
use crate::product::{LightPlatform, LightPlatformGatt, LightPlatformRadio};
use crate::uart::{self, uart_receiver, uart_sender, UartCommand, UartTxOps};
use crate::uprintln;
use crate::version::BUILD_VERSION;
use heapless::Vec;
use mesh_protocol::mesh_types::*;
use mesh_protocol::mesh_types::{LGT_CMD_END_OTA_RSP, LGT_CMD_OTA_DATA_RSP, LGT_CMD_START_OTA_RSP};
use mesh_protocol::packet_types::{Packet, PacketAttValue};
use mesh_protocol::uart::UartData;
use tlsr8266_hal::drivers::flash::{flash_erase_sector, flash_write_page};
use tlsr8266_hal::drivers::pwm::{pwm_set_duty, pwm_start};
use tlsr8266_hal::mcu::clock::{
    clock_init, is_16mhz_xtal, CLOCK_SYS_CLOCK_1S, CLOCK_SYS_CLOCK_1US,
};
use tlsr8266_hal::mcu::dma::dma_init;
use tlsr8266_hal::mcu::gpio::{gpio_init, gpio_set_func, AS_GPIO};
use tlsr8266_hal::mcu::irq_i::{irq_disable, irq_enable, irq_init};
use tlsr8266_hal::mcu::watchdog::wd_clear;
use tlsr8266_hal::pm::{blink_ota_led_ok, light_sw_reboot, usb_dp_pullup_en};
use tlsr8266_radio::ble::connection::{
    light_check_tick_per_us, rf_link_slave_pairing_enable, rf_link_slave_proc,
};
use tlsr8266_radio::ble::flash_config::{dev_addr_with_mac_flag, dev_addr_with_mac_match};
use tlsr8266_radio::callbacks::{set_ble_app_hooks, set_ota_hooks, BleAppHooks, OtaHooks};
use tlsr8266_radio::mesh_ll::adv::*;
use tlsr8266_radio::mesh_ll::node_status::mesh_node_table_init;
use tlsr8266_radio::mesh_ll::packet_processing::parse_ble_packet_op_params;
use tlsr8266_radio::rf::drv_8266::{rf_drv_init, rf_link_slave_init, rf_set_power_level_index};
use tlsr8266_radio::types::*;
use tlsr8266_radio::{
    abort_slave_read_status_if_busy, check_auth_timeout, device_address, group_addresses,
    is_paired_login, mac_id, radio_config, set_adv_private_data, try_send_terminate_packet,
    write_out_of_mesh_credentials,
};

// One shared packet window owns BLE status and OTA staging. Keep the OTA
// adapter bound, the platform window depth, and the 48-byte Packet layout in
// sync at compile time.
mesh_protocol::const_assert!(
    crate::ota::OTA_STAGING_SLOTS == tlsr8266_platform::window::PACKET_WINDOW_SLOTS
);
mesh_protocol::const_assert!(tlsr8266_platform::window::PACKET_WINDOW_SLOTS == 16);
mesh_protocol::const_assert!(size_of::<[Packet; 16]>() == 768);

/// The board configuration the radio layer runs with.
///
/// `sdk/` may not name `crate::config` or `crate::version` (design 031 / C-9), so
/// this composition root is the single place the two meet: `main_task` installs
/// the struct before the radio starts, and the default is only the host-test
/// fallback.
pub(crate) const RADIO_CONFIG: tlsr8266_radio::RadioConfig = tlsr8266_radio::RadioConfig {
    vendor_id: crate::config::VENDOR_ID,
    build_version: crate::version::BUILD_VERSION,
    device_name: crate::config::DEVICE_NAME,
    default_mesh_name: crate::config::MESH_NAME,
    default_mesh_pwd: crate::config::MESH_PWD,
    default_mesh_pwd_encode_sk: crate::config::MESH_PWD_ENCODE_SK,
    default_mesh_ltk: crate::config::MESH_LTK,
    out_of_mesh_name: crate::config::OUT_OF_MESH,
    pair_valid_flag: crate::config::PAIR_VALID_FLAG,
    flash_adr_mac: crate::config::FLASH_ADR_MAC,
    flash_adr_pairing: crate::config::FLASH_ADR_PAIRING,
    flash_adr_dev_grp_adr: crate::config::FLASH_ADR_DEV_GRP_ADR,
};

/// The application's single root task.
///
/// Boot order is normative (design 018 §5): OTA recovery runs in
/// `main_entrypoint` before any clock or IRQ exists, then this task brings the
/// peripherals up, registers every SDK seam **before** `rf_drv_init(true)`
/// populates the radio, and only then spawns the per-owner tasks and enters the
/// main loop.
#[embassy_executor::task]
pub async fn main_task(spawner: Spawner) {
    // Minimal compile-time product composition reference. Live callback and GATT
    // dispatch continue to use the existing application ownership below.
    let _light_platform = LightPlatform::new();
    let _light_profile = LightPlatformGatt::profile_view(&[]);
    let _light_rf_plan = <LightPlatformRadio as tlsr8266_radio::RadioCoexistence>::auxiliary_rf();

    // Initialise various subsystems needed by uart
    clock_init();
    dma_init();
    gpio_init();
    irq_init();

    // Get uart ready to go early
    uart::init();

    // Wire the transport's outbound edges to the SDK queue and the command
    // handler before any traffic (or log line) can occur. `uart` sits below the
    // SDK, so these adapters — not a compiled-in import — are the only edge from
    // a frame to a queue slot or a mesh send.
    uart::set_tx_ops(UartTxOps {
        enqueue_tx: enqueue_uart_tx_hook,
        dropped_tx: dropped_uart_tx_hook,
    });
    uart::set_command_handler(uart_command_hook);

    // Point the log sink at the UART transport before the first line. `logging`
    // sits below `uart`, so this adapter — not a compiled-in import — is the only
    // edge from a log line to the wire.
    crate::logging::set_log_sinks(crate::logging::LogSinks {
        async_tx: log_async_tx_hook,
    });

    // Report why the previous boot ended and mark this one as running
    let reason = boot_init();
    log_boot_reset_reason(reason);

    uprintln!(
        "Booting FW version {}, xtal: {}MHz",
        BUILD_VERSION,
        if is_16mhz_xtal() { 16 } else { 12 }
    );

    // Wire the SDK's callback seams to the application before any RF or pairing
    // path can run, so neither slot is ever observed empty.
    set_ota_radio_ops(OtaRadioOps {
        try_begin_gatt_ota: ota_begin_gatt_hook,
        stage_packet: tlsr8266_platform::window::stage_ota_packet,
        read_staged_packet: tlsr8266_platform::window::read_staged_ota_packet,
        try_send_terminate_packet: ota_send_terminate_hook,
    });
    set_ota_hooks(OtaHooks {
        on_gatt_ota_write: crate::ota::on_gatt_ota_packet,
        status_snapshot: crate::ota::ota_status_snapshot_impl,
        request_termination: crate::ota::ota_request_termination_impl,
        tick_timeout_1s: crate::ota::ota_tick_timeout_1s_impl,
    });
    tlsr8266_radio::mesh_ll::adv::set_light_event_handler(rf_link_light_event_callback);
    set_ble_app_hooks(BleAppHooks {
        on_mesh_pair_proc: mesh_pair_proc_hook,
        on_light_data: rf_link_data_callback,
        on_light_response: rf_link_response_callback,
        on_uart_mesh_rx: uart_mesh_rx_hook,
    });
    // The mesh subsystem owns the radio and the pairing state machine but not
    // the light, the LED or the UART bridge: those arrive as hooks, so `mesh`
    // never compiles in a dependency on a sibling. Registered before any mesh
    // traffic exists.
    set_mesh_event_hooks(MeshEventHooks {
        on_light_event: rf_link_light_event_callback,
        on_device_status_update: device_status_update,
        on_uart_mesh_rx: uart_mesh_rx_hook,
    });
    // The light subsystem owns the lamp but not the mesh node table: publish its
    // on/off state through this pointer instead of letting `light` depend on
    // the SDK. Registered here, before any mesh traffic exists.
    crate::light::set_status_update_hook(
        tlsr8266_radio::mesh_ll::node_status::ll_device_status_update,
    );

    // Install the board configuration and the GATT values it drives before the
    // radio starts: the first advertisement must already carry the vendor id and
    // the first GATT read the configured names and version.
    tlsr8266_radio::set_radio_config(RADIO_CONFIG);
    tlsr8266_radio::gatt::service_table::init_gatt_profile(&RADIO_CONFIG);

    rf_drv_init(true);

    // Run our initialisation; the lamp's write pointer lives here from now on
    let mut flash_store = user_init();
    record_breadcrumb(CRUMB_INIT_DONE);

    // Start the mesh packet sender / receive handler. Every boot spawn goes
    // through `spawn_boot_task`, which reports a spawn-graph wiring bug and
    // reboots instead of unwrapping into the panic handler.
    spawn_boot_task(
        spawner,
        "mesh_packet_sender",
        crate::mesh::mesh_packet_sender(),
    );
    spawn_boot_task(spawner, "mesh_packet_rcv", crate::mesh::mesh_packet_rcv());

    // Start the UART tasks. `uart_sender` parks on `started_signal` until the
    // first valid host frame arrives, so a headless light never retries DMA TX
    // for the boot banners.
    spawn_boot_task(spawner, "uart_receiver", uart_receiver());
    spawn_boot_task(
        spawner,
        "uart_sender",
        uart_sender(&tlsr8266_radio::queues::UART_TX_CHANNEL),
    );

    // Start the light command consumer
    spawn_boot_task(spawner, "light_manager", light_manager());

    // Start the panic checker to see if there is information we need to send
    spawn_boot_task(spawner, "panic_check", panic_check());

    // Ready to go, enable interrupts and run the main loop
    irq_enable();
    record_breadcrumb(CRUMB_MAIN_LOOP);

    // Send a message to the network saying that we just booted up
    let mut data = [0_u8; 13];
    data[0] = LGT_POWER_ON;

    // 13 bytes always fit; the `Err` arm is defence in depth, not a routine
    // branch, so it skips the broadcast silently.
    if let Ok(msg) = Vec::from_slice(&data) {
        crate::mesh::send_mesh_message(&msg, 0xffff, 3, false);
    }

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
        uart::clear_error();
    }
}

/// Spawns one task in the boot spawn graph.
///
/// A `#[embassy_executor::task]` constructor returns `Err` only when its static
/// slot is already taken — a wiring bug in `main_task`, not a runtime
/// condition. Name the failed spawn on the log channel and reboot, rather than
/// unwrapping into the panic handler.
fn spawn_boot_task<S>(spawner: Spawner, name: &str, task: Result<SpawnToken<S>, SpawnError>) {
    match task {
        Ok(token) => spawner.spawn(token),
        Err(_) => boot_spawn_abort(name),
    }
}

/// Reports a failed boot spawn and reboots. Never returns.
#[cold]
#[inline(never)]
fn boot_spawn_abort(name: &str) -> ! {
    uprintln!("[BOOT] spawn failed: {}; rebooting", name);
    light_sw_reboot();
    // On target `light_sw_reboot` never returns. Off target it does, so park
    // here rather than resuming boot with a task missing.
    loop {
        core::hint::spin_loop();
    }
}

/// Marker appended to a panic body that was clipped to fit the UART stream.
const PANIC_LOG_TRUNCATED: &str = "...";

/// Prints why the previous boot ended.
///
/// `#[inline(never)]`: the panic preview buffer and the formatting would
/// otherwise be carried in `main_task`'s future for the whole of boot.
#[inline(never)]
fn log_boot_reset_reason(reason: BootResetReason) {
    match reason {
        // No retained sentinel: the previous boot ended normally.
        BootResetReason::Clean => {}
        BootResetReason::Watchdog { breadcrumb } => {
            uprintln!("[WD_RESET] sentinel=0xa5 breadcrumb={}", breadcrumb);
        }
        BootResetReason::Panic { breadcrumb } => {
            // A record that is not trusted at all is reported as nothing rather
            // than as an empty message; a flash read failure is reported as an
            // explicit unreadable marker instead of a partial body.
            let preview = match read_panic_preview() {
                Ok(Some(preview)) => preview,
                Ok(None) => return,
                Err(()) => {
                    uprintln!(
                        "[PANIC] sentinel=0xe1 breadcrumb={} msg=<unreadable>",
                        breadcrumb
                    );
                    return;
                }
            };

            let body = core::str::from_utf8(&preview.buf[..preview.len]).unwrap_or("");
            let ellipsis = if preview.truncated {
                PANIC_LOG_TRUNCATED
            } else {
                ""
            };

            uprintln!(
                "[PANIC] sentinel=0xe1 breadcrumb={} msg={}{}",
                breadcrumb,
                body,
                ellipsis
            );
        }
    }
}

/// `logging::LogSinks::async_tx`: queue one log frame on the UART transport.
///
/// `#[inline(never)]` and no closure on purpose: `sdk/stack_analysis.py`'s
/// indirect-edge table binds `dispatch_log_async` to this symbol, and a closure
/// would mangle to `main_task::{{closure}}` and be rejected.
#[inline(never)]
fn log_async_tx_hook(msg: &UartData) -> bool {
    crate::uart::send_message(msg).is_ok()
}

/// `uart::UartTxOps::enqueue_tx`: one queue slot on the SDK's UART TX channel.
///
/// `#[inline(never)]` and no closure: `sdk/stack_analysis.py`'s indirect-edge
/// table binds `uart_tx_ops_push` to this symbol.
#[inline(never)]
fn enqueue_uart_tx_hook(msg: &UartData) -> bool {
    tlsr8266_radio::queues::enqueue_uart_tx(msg)
}

/// `uart::UartTxOps::dropped_tx`: how many frames the queue has dropped.
///
/// `#[inline(never)]`: the `INDIRECT_EDGES` key for `uart_tx_ops_dropped`.
#[inline(never)]
fn dropped_uart_tx_hook() -> u32 {
    tlsr8266_radio::queues::dropped_uart_tx()
}

/// `uart::set_command_handler`: execute one host command and return the mesh
/// sequence number its UART ACK frame must carry.
///
/// Every arm returns `[0; 3]` except a light command, whose ACK carries the
/// real mesh `sno` so the host can correlate the ack with the traffic it
/// caused. `HostAttached` enables status reporting *before* the initial node
/// dump that enabling performs, or that first dump is filtered out.
///
/// `#[inline(never)]` and no closure on purpose: `sdk/stack_analysis.py`'s
/// indirect-edge table binds `dispatch_uart_cmd` to this symbol.
#[inline(never)]
fn uart_command_hook(cmd: UartCommand) -> [u8; 3] {
    match cmd {
        UartCommand::LightCtrl {
            dst,
            payload,
            retransmit,
            ack,
        } => match Vec::from_slice(&payload) {
            Ok(msg) => crate::mesh::send_mesh_message(&msg, dst, retransmit, ack),
            // 13 bytes always fit; skipping the send keeps the ACK sno at zero.
            Err(_) => [0; 3],
        },
        UartCommand::EnableStatusReporting => {
            tlsr8266_radio::mesh_ll::node_status::mesh_report_status_enable(true);
            tlsr8266_radio::queues::set_uart_status_reporting_enabled(true);
            [0; 3]
        }
        UartCommand::HostAttached => {
            tlsr8266_radio::queues::set_uart_status_reporting_enabled(true);
            tlsr8266_radio::mesh_ll::node_status::mesh_report_status_enable(true);
            [0; 3]
        }
    }
}

/// `BleAppHooks::on_uart_mesh_rx` / `mesh`'s relay path: hand one received mesh
/// packet to the UART bridge with this device's address.
///
/// `#[inline(never)]`: the `INDIRECT_EDGES` key for the SDK's `on_uart_mesh_rx`
/// dispatcher. The address is read here, in the composition root, so `uart`
/// never imports the SDK.
#[inline(never)]
pub fn uart_mesh_rx_hook(pkt: &Packet) {
    crate::uart::on_mesh_rx_packet(pkt, device_address());
}

pub fn light_hw_timer_config() {
    // Enable timer1 interrupts for controlling light transitions
    tlsr8266_hal::mcu::irq_i::irq_enable_tmr1();
    tlsr8266_hal::mcu::timer::tmr1_configure(CLOCK_SYS_CLOCK_1S / 120); // ~ 120hz

    // enable timer0 interrupt for tracking clock_time overflow
    tlsr8266_hal::mcu::irq_i::irq_enable_tmr0();
    tlsr8266_hal::mcu::timer::tmr0_configure_and_enable(CLOCK_SYS_CLOCK_1S);
}

fn light_init_default() {
    light_check_tick_per_us(CLOCK_SYS_CLOCK_1US);

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
    let mac = mac_id();
    uprintln!(
        "MAC Address: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        mac[5],
        mac[4],
        mac[3],
        mac[2],
        mac[1],
        mac[0]
    );

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
    // payloads from the loaded MAC, the configured vendor id and the device
    // address, and the radio facade installs them together with the manufacturer
    // bytes the advertisement filter matches on.
    let (pri, rsp) = build_vendor_adv_data(mac_id(), radio_config().vendor_id, device_address());
    set_adv_private_data(pri, &adv_private_to_bytes(&pri), rsp);

    device_status_update();
    mesh_security_enable(true);

    flash_store
}

fn light_auth_check() {
    // The auth-window policy lives on the radio facade: with security on and no
    // login, the first-connection timestamp is cleared once `AUTH_TIME` has
    // elapsed. The connection itself is kept.
    check_auth_timeout(AUTH_TIME);
}

/// One main-loop tick's worth of light work.
///
/// Takes the flash store by `&mut`: handing it over by mutable reference, rather
/// than sharing a `Clone`, is the whole point of the ownership split between
/// `main_task` and the loop body. Tests drive the real function and assert its
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
pub fn rf_link_response_callback(
    ppp: &mut PacketAttValue,
    p_req: &PacketAttValue,
) -> Result<(), ()> {
    // The five light-domain opcodes are the light subsystem's own protocol; it
    // builds the whole answer, including the shared response prologue.
    if crate::light::build_light_status_response(
        ppp,
        p_req,
        device_address(),
        &group_addresses(),
        mac_id(),
    )
    .is_ok()
    {
        return Ok(());
    }

    // Everything else is the OTA seam: the same prologue, then the three opcodes.
    let address = device_address();
    ppp.dst = p_req.src;
    ppp.src[0] = (address & 0xff) as u8;
    ppp.src[1] = ((address >> 8) & 0xff) as u8;

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

            let _ = crate::ota::try_start_mesh_ota(|| {
                abort_slave_read_status_if_busy();
                true
            });
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
            // The reboot below happens either way, so a failed program is not
            // observable in this path; the MAC is regenerated on the next boot
            // when flash still reads erased.
            let _ = flash_erase_sector(FLASH_ADR_MAC);
            let _ = flash_write_page(FLASH_ADR_MAC, mac.len() as u32, addr_of!(mac) as *const u8);
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
            // The parameter array always fits; a failure would only mean the
            // pairing broadcast is skipped.
            if let Ok(params) = Vec::from_slice(&pair_params) {
                mesh_pair_cb(&params)
            }
        }
    }
}

#[inline(never)]
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
    report_panic_info().await;
}

/// Sends any stored panic message to the mesh, then clears the sector.
///
/// Chunked into 10-byte `LGT_PANIC_MSG` payloads with a 100 ms gap so the radio
/// keeps up, an empty `LGT_PANIC_MSG` terminates the report, and only then is
/// the sector erased. A plain `async fn` so the chunking stays testable without
/// an executor, exactly as `light_manager` above does.
async fn report_panic_info() {
    let mut buf = [0u8; 256];

    // Nothing to report when the flag is blank; an unreadable bus (`Err`) is
    // left alone so the record is retried after the next reset rather than
    // reported from a partial buffer.
    let Ok(Some(len)) = read_panic_report(&mut buf) else {
        return;
    };

    // There is panic info, send it in to the mesh.
    for chunk in buf[..len].chunks(10) {
        // Wait a moment to send the next message
        #[cfg(not(test))]
        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;

        let mut data = [0u8; 13];
        data[0] = LGT_PANIC_MSG;
        data[3..3 + chunk.len()].copy_from_slice(chunk);

        // 13 bytes always fit. The panic path must stay allocation-free and
        // log-free, so a failure just skips this chunk.
        if let Ok(msg) = Vec::from_slice(&data) {
            crate::mesh::send_mesh_message(&msg, 0xffff, 3, false);
        }
    }

    // Wait a moment to send the next message
    #[cfg(not(test))]
    embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;

    // Send an empty panic message to indicate the end of the message
    let mut data = [0u8; 13];
    data[0] = LGT_PANIC_MSG;

    // Same defence-in-depth skip as the chunk loop above.
    if let Ok(msg) = Vec::from_slice(&data) {
        crate::mesh::send_mesh_message(&msg, 0xffff, 3, false);
    }

    // Finally clear the panic info
    clear_panic_report();
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
#[export_name = "tlsr8266_ota_begin_gatt_hook"]
fn ota_begin_gatt_hook() -> bool {
    if !is_paired_login() {
        return false;
    }

    tlsr8266_platform::window::begin_ota(tlsr8266_platform::window::OtaClaimant::Gatt, || {
        abort_slave_read_status_if_busy();
        true
    })
    .is_ok()
}

/// `OtaRadioOps::try_send_terminate_packet`: `true` only when `PKT_TERMINATE`
/// was actually queued, so a full buffer neither arms the countdown nor reboots.
#[inline(never)]
fn ota_send_terminate_hook() -> bool {
    try_send_terminate_packet()
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::diagnostics::{CRUMB_PANIC, PANIC_LOG_BODY_MAX, PANIC_READ_MAX};
    use crate::light::effects::{pending_event, reset_for_test as reset_led_controller};
    use crate::light::status_probe;
    use crate::light::transitions::{begin_transition, LIGHT_CMD_CHANNEL};
    use crate::ota::reset_for_test as reset_ota;
    use crate::uart::reset_for_test;
    use futures::executor::block_on;
    use mesh_protocol::packet_types::PacketL2capHead;
    use mesh_protocol::uart::{UartData, UartMsg, UART_DATA_LEN};
    use tlsr8266_hal::drivers::flash::fake_flash::{FakeFlash, FakeFlashMode};
    use tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog;
    use tlsr8266_hal::mcu::clock::fake_clock;
    use tlsr8266_hal::mcu::crypto::{encode_password, fake_crypto};
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_pac::{FLD_IRQ, FLD_MASTER_SPI, FLD_TMR};
    use tlsr8266_radio::mesh_ll::node_status::{
        MESH_NODE_MAX, MESH_NODE_REPORT_ENABLE, MESH_NODE_ST,
    };
    use tlsr8266_radio::queues::{self, UART_TX_CHANNEL};
    use tlsr8266_radio::rf::drv_8266::BLE_PERIPHERAL_ADVERTISING_ENABLED;
    use tlsr8266_radio::rf::drv_8266::{BLE_SCAN_RESPONSE_INTERVAL_US, PKT_ADV};
    use tlsr8266_radio::{
        adv_private_data, dev_addr_validation_pending, first_connection_timestamp,
        flash_config_index, is_mesh_pair_enabled, is_security_enabled, reset_radio_state_for_test,
        set_dev_addr_validation_pending, set_device_address, set_first_connection_timestamp,
        set_flash_config_index, set_group_addresses, set_mac_id, set_mesh_pair_enabled,
        set_pair_config_mesh_credentials, set_paired_login, set_security_enabled,
        set_slave_read_status_busy,
    };

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

    /// Build the packet a data callback receives: the 3-byte opcode at
    /// `val[0..3]` and up to ten parameter bytes at `val[3..13]`.
    ///
    /// `l2cap_len` is the length `mesh_construct_packet` stamps, so the real
    /// parser derives exactly `params.len()` parameter bytes.
    fn mesh_packet(op: u8, vendor: u16, params: &[u8]) -> Packet {
        let mut pkt = create_test_packet();
        let write = pkt.att_write_mut();
        write.head.l2cap_len = 23 + params.len() as u16;
        write.value.val[0] = op | 0xc0;
        write.value.val[1] = (vendor & 0xff) as u8;
        write.value.val[2] = (vendor >> 8) as u8;
        write.value.val[3..3 + params.len()].copy_from_slice(params);
        pkt
    }

    /// Frames the async log sink recorded, in order.
    static LOG_FRAMES: std::sync::Mutex<std::vec::Vec<UartData>> =
        std::sync::Mutex::new(std::vec::Vec::new());

    /// Count of `BleAppHooks::on_mesh_pair_proc` dispatches.
    static MESH_PAIR_PROC_CALLS: core::sync::atomic::AtomicU32 =
        core::sync::atomic::AtomicU32::new(0);

    fn record_log_frame(frame: &UartData) -> bool {
        LOG_FRAMES.lock().unwrap().push(*frame);
        true
    }

    fn record_mesh_pair_proc() {
        MESH_PAIR_PROC_CALLS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }

    fn noop_light_data(_: &Packet) {}

    fn noop_light_response(_: &mut PacketAttValue, _: &PacketAttValue) -> Result<(), ()> {
        Err(())
    }

    fn noop_uart_mesh_rx(_: &Packet) {}

    /// Install the recording log sinks and drop every frame captured so far.
    fn install_log_recorder() {
        LOG_FRAMES.lock().unwrap().clear();
        crate::logging::set_log_sinks(crate::logging::LogSinks {
            async_tx: record_log_frame,
        });
    }

    /// Install the SDK hook set, with `on_mesh_pair_proc` counting.
    ///
    /// Starts from the radio's host-build defaults so a hook recorded by an
    /// earlier test cannot leak into this one; the recording set is installed
    /// immediately afterwards, so nothing here clobbers it.
    fn install_recording_hooks() {
        tlsr8266_radio::callbacks::reset_for_test();
        MESH_PAIR_PROC_CALLS.store(0, core::sync::atomic::Ordering::Relaxed);
        tlsr8266_radio::callbacks::set_ble_app_hooks(tlsr8266_radio::callbacks::BleAppHooks {
            on_mesh_pair_proc: record_mesh_pair_proc,
            on_light_data: noop_light_data,
            on_light_response: noop_light_response,
            on_uart_mesh_rx: noop_uart_mesh_rx,
        });
    }

    // --- Helper Functions ---

    /// Resets global state for testing
    fn reset_test_globals() {
        set_security_enabled(false);
        set_paired_login(false);
        set_first_connection_timestamp(0);
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
    /// cannot be executed on the host: `rf_drv_init` and `user_init` both touch
    /// hardware registers, and the slots they fill are private statics owned by
    /// `sdk::app_att_light`, `sdk::rf_drv` and `sdk::ble_app::callbacks`.
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
        let mesh_hooks = task_body
            .find("set_mesh_event_hooks(MeshEventHooks {")
            .expect("main_task must register the mesh event hooks");
        let uart_init = task_body
            .find("uart::init()")
            .expect("main_task must initialise the UART transport");
        let uart_tx_ops = task_body
            .find("uart::set_tx_ops(")
            .expect("main_task must register the UART tx ops");
        let uart_cmd = task_body
            .find("uart::set_command_handler(")
            .expect("main_task must register the UART command handler");
        let log_sinks = task_body
            .find("set_log_sinks(")
            .expect("main_task must register the log sinks");
        let banner = task_body
            .find("Booting FW version")
            .expect("main_task must print the boot banner");
        let rf_drv = task_body
            .find("rf_drv_init(true)")
            .expect("main_task must initialise the RF driver");
        let radio_config = task_body
            .find("set_radio_config(")
            .expect("main_task must install the radio configuration");
        let gatt_profile = task_body
            .find("init_gatt_profile(")
            .expect("main_task must initialise the GATT profile");

        assert!(
            uart_init < uart_tx_ops && uart_tx_ops < log_sinks && uart_cmd < banner,
            "UART must be initialised, its tx ops registered and its command \
             handler installed before the first log line, or the banner is dropped"
        );
        assert!(
            log_sinks < banner,
            "the log sinks must be registered before the first log line, or the \
             boot banner is dropped"
        );
        assert!(
            ota_ops < rf_drv
                && ota_hooks < rf_drv
                && light < rf_drv
                && hooks < rf_drv
                && status_hook < rf_drv
                && mesh_hooks < rf_drv,
            "every handler registration must precede rf_drv_init(true)"
        );
        assert!(
            radio_config < gatt_profile && gatt_profile < rf_drv,
            "the radio configuration must be installed and the config-driven GATT \
             values written before rf_drv_init(true)"
        );

        // `RADIO_CONFIG` is defined above `main_task`, so search only the head of
        // the file: this test's own literals would otherwise satisfy the needle.
        let config_head = &src[..task_start];
        let config_start = config_head
            .find("const RADIO_CONFIG:")
            .expect("app.rs must define RADIO_CONFIG");
        let config_tail = &config_head[config_start..];
        let config_body = &config_tail[..config_tail
            .find("};")
            .expect("RADIO_CONFIG must be a struct literal")];

        assert!(
            config_body.contains("build_version: crate::version::BUILD_VERSION"),
            "RADIO_CONFIG must carry the build version, or the target reports 0"
        );
    }

    /// `uart_command_hook` executes a light command through the mesh and returns
    /// the mesh sequence number the UART ACK frame carries.
    #[test]
    fn test_uart_command_hook_light_ctrl_returns_the_mesh_sno() {
        reset_for_test();
        set_device_address(0x1234);
        fake_clock::set_ticks(0x5000);

        let sno = uart_command_hook(UartCommand::LightCtrl {
            dst: 0x1234,
            payload: [7; 13],
            retransmit: 3,
            ack: true,
        });

        // `send_mesh_message` returns the packet's sequence number, which
        // `light_slave_tx_command` seeds with `clock_time() + device_address()`.
        assert_eq!(sno, [0x34, 0x62, 0x00]);
        let local = queues::MESH_RX_CHANNEL
            .try_receive()
            .expect("a device-addressed command is delivered locally");
        let local_dst = local.ll_app().value.dst;
        assert_eq!(local_dst, 0x1234);
        assert_eq!(
            queues::MESH_TX_QUEUE.lock().len(),
            0,
            "a device-addressed command has nowhere to be re-transmitted"
        );
        assert!(
            !queues::uart_status_reporting_enabled(),
            "a light command must not touch status reporting"
        );
        assert_eq!(
            UART_TX_CHANNEL.len(),
            0,
            "a light command must not emit a status frame"
        );
    }

    /// The `HostAttached` arm enables status reporting *before* the node dump
    /// that enabling performs, or the first dump is filtered out.
    #[test]
    fn test_uart_command_hook_host_attached_enables_reporting_before_the_dump() {
        reset_for_test();
        set_device_address(0x0034);
        mesh_node_table_init();
        // Start from "disabled", so the assertion below cannot be satisfied by
        // state an earlier test left behind.
        MESH_NODE_REPORT_ENABLE.store(false, Ordering::Relaxed);

        assert_eq!(uart_command_hook(UartCommand::HostAttached), [0; 3]);

        assert!(queues::uart_status_reporting_enabled());
        assert!(
            MESH_NODE_REPORT_ENABLE.load(core::sync::atomic::Ordering::Relaxed),
            "the SDK's mesh reporting is enabled too"
        );
        // The initial dump only reaches the host because the flag was set first.
        let frame = UART_TX_CHANNEL
            .try_receive()
            .expect("the node dump must reach the host");
        assert_eq!(frame.data[2], UartMsg::NodeStatus as u8);
        assert_eq!(frame.data[3], 0x34, "the dump reports this node");
        assert_eq!(frame.data[4], 1, "this node is online");
    }

    /// The status-refresh arm re-runs the dump and acknowledges with zeros.
    #[test]
    fn test_uart_command_hook_enable_status_reporting_reruns_the_dump() {
        reset_for_test();
        set_device_address(0x0034);
        mesh_node_table_init();
        MESH_NODE_REPORT_ENABLE.store(false, Ordering::Relaxed);

        assert_eq!(
            uart_command_hook(UartCommand::EnableStatusReporting),
            [0; 3]
        );

        assert!(queues::uart_status_reporting_enabled());
        assert!(
            MESH_NODE_REPORT_ENABLE.load(core::sync::atomic::Ordering::Relaxed),
            "the SDK call ran and enabled mesh reporting"
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
        assert!(
            !ota_send_terminate_hook(),
            "a full packet buffer must not report a queued terminate packet"
        );

        // Plenty of room: (2 - 0) & 7 == 2, so the packet is queued.
        FakeMmio::reset();
        FakeMmio::set_reg8(REG_DMA_TX_WPTR, 2);
        FakeMmio::set_reg8(REG_DMA_TX_RPTR, 0);
        assert!(
            ota_send_terminate_hook(),
            "a free packet buffer must report the queued terminate packet"
        );
    }

    /// `ota_begin_gatt_hook` must refuse a GATT OTA until pairing succeeds.
    #[test]
    fn test_ota_begin_gatt_hook_requires_pairing() {
        reset_ota();
        set_slave_read_status_busy(0);
        set_paired_login(false);
        assert!(
            !ota_begin_gatt_hook(),
            "an unpaired link must not begin a GATT OTA"
        );

        set_paired_login(true);
        assert!(
            ota_begin_gatt_hook(),
            "a paired link with an idle status read may begin a GATT OTA"
        );

        set_paired_login(false);
        tlsr8266_platform::window::end_ota();
        reset_ota();
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
        set_security_enabled(true);
        set_paired_login(false);
        set_first_connection_timestamp(1000000);

        // The authentication window is 60 s: park the tick past it.
        reset_fakes();
        fake_clock::set_ticks(1_000_000 + AUTH_TIME as u64 * 1000 * 1000 * 32 + 1);

        // Execute
        light_auth_check();

        // Verify: Timestamp should be cleared
        assert_eq!(
            first_connection_timestamp(),
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
        set_security_enabled(true);
        set_paired_login(true);
        set_first_connection_timestamp(1000000);

        // Execute
        light_auth_check();

        // Verify: Function should not crash (logged in means condition check short-circuits)
        // Since the host is already logged in, the whole condition is false and
        // the function returns early.
        assert_eq!(
            first_connection_timestamp(),
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
        set_security_enabled(false);
        set_paired_login(false);
        set_first_connection_timestamp(1000000);

        // Execute
        light_auth_check();

        // Verify: Timestamp should remain unchanged
        assert_eq!(first_connection_timestamp(), 1000000);
    }

    // --- Tests for rf_link_response_callback ---

    #[test]
    fn test_rf_link_response_callback_get_status() {
        // This test verifies GET_STATUS response returns current light state

        // Setup
        crate::light::reset_for_test();
        set_device_address(0x1234);

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
        set_device_address(0x1234);
        let mut group_addresses_seed = [0u16; MAX_GROUP_COUNT as usize];
        group_addresses_seed[0] = 0x11;
        group_addresses_seed[1] = 0x22;
        group_addresses_seed[2] = 0x33;
        set_group_addresses(group_addresses_seed);

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
        set_device_address(0x1234);
        let mut group_addresses_seed = [0u16; MAX_GROUP_COUNT as usize];
        group_addresses_seed[0] = 0xAABB;
        group_addresses_seed[1] = 0xCCDD;
        set_group_addresses(group_addresses_seed);

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
        set_device_address(0x1234);
        let mut group_addresses_seed = [0u16; MAX_GROUP_COUNT as usize];
        group_addresses_seed[4] = 0x1122;
        group_addresses_seed[5] = 0x3344;
        set_group_addresses(group_addresses_seed);

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
        set_device_address(0x1234);
        let mac = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        set_mac_id(mac);

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
        set_device_address(0xABCD);

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
        set_device_address(0x1234);
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

    /// The OTA data path runs for real: the index the response carries is the
    /// one the staging code read out of the request, and the block is accepted
    /// for staging.
    #[test]
    fn test_rf_link_response_callback_cmd_ota_data() {
        // Setup
        reset_ota();
        FakeFlash::reset();
        set_device_address(0x5678);
        assert!(crate::ota::try_start_mesh_ota(|| {
            abort_slave_read_status_if_busy();
            true
        }));

        let mut request = create_test_packet_att_value();
        request.src = [0xAA, 0xBB];
        // `rf_mesh_data_ota` reads the packet index from the first two params.
        request.val[3] = 0x34;
        request.val[4] = 0x12;
        request.val[5] = 0xA5; // first payload byte

        let mut response = create_test_packet_att_value();
        response.val[15] = CMD_OTA_DATA;

        // Execute
        let result = rf_link_response_callback(&mut response, &request);

        // Verify: the response carries the parsed index, and the block was
        // accepted (reporting still blocked, supervision timeout refreshed).
        assert!(
            result.is_ok(),
            "Callback should return Ok(()) for CMD_OTA_DATA"
        );
        assert_eq!(response.val[0], LGT_CMD_OTA_DATA_RSP | 0xc0);
        assert_eq!(response.val[1], 0x34); // idx low byte
        assert_eq!(response.val[2], 0x12); // idx high byte
        assert!(
            crate::ota::is_mesh_ota_blocked(),
            "the mesh OTA stays open while data arrives"
        );
        assert!(!crate::ota::is_timeout_expired());
    }

    /// The end-of-transfer path runs for real: the response carries the index
    /// the staging code parsed, and the staging area closes.
    #[test]
    fn test_rf_link_response_callback_cmd_end_ota() {
        // Setup
        reset_ota();
        FakeFlash::reset();
        set_device_address(0x5678);
        assert!(crate::ota::try_start_mesh_ota(|| {
            abort_slave_read_status_if_busy();
            true
        }));

        let mut request = create_test_packet_att_value();
        request.src = [0xCC, 0xDD];
        request.val[3] = 0xCD;
        request.val[4] = 0xAB;

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
    }

    #[test]
    fn test_rf_link_response_callback_invalid_command() {
        // This test verifies that invalid commands return false

        // Setup
        set_device_address(0x1234);

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

    /// `LGT_CMD_LIGHT_ONOFF` is decoded from the real wire bytes and reaches the
    /// light task with the parsed parameters.
    #[test]
    fn test_rf_link_data_callback_light_onoff() {
        crate::light::reset_for_test();

        let params = [LIGHT_ON_PARAM, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let packet = mesh_packet(LGT_CMD_LIGHT_ONOFF, VENDOR_ID, &params);

        rf_link_data_callback(&packet);

        let msg = LIGHT_CMD_CHANNEL
            .try_receive()
            .expect("LGT_CMD_LIGHT_ONOFF must be queued");
        assert_eq!(msg.cmd, LGT_CMD_LIGHT_ONOFF);
        let mut expected = [0u8; 16];
        expected[..params.len()].copy_from_slice(&params);
        assert_eq!(msg.params, expected);
        assert!(
            LIGHT_CMD_CHANNEL.try_receive().is_err(),
            "exactly one command must be queued"
        );
    }

    /// Group deletion removes the stored address and blinks the indicator.
    #[test]
    fn test_rf_link_data_callback_light_config_grp_delete() {
        reset_radio_state_for_test();
        reset_fakes();
        reset_led_controller();

        // Install the group first, so the delete has something to remove.
        assert!(add_group(0x8012).is_ok());
        assert!(group_addresses().contains(&0x8012));

        let params = [LIGHT_DEL_GRP_PARAM, 0x12, 0x80, 0, 0, 0, 0, 0, 0, 0];
        let packet = mesh_packet(LGT_CMD_LIGHT_CONFIG_GRP, VENDOR_ID, &params);

        rf_link_data_callback(&packet);

        assert!(
            !group_addresses().contains(&0x8012),
            "the stored group is removed"
        );
        assert_eq!(
            pending_event(),
            LED_EVENT_FLASH_1HZ_4S,
            "a successful group change blinks the indicator"
        );
    }

    /// Deleting a group that was never stored changes nothing and does not blink.
    #[test]
    fn test_rf_link_data_callback_light_config_grp_delete_no_flash() {
        reset_radio_state_for_test();
        reset_fakes();
        reset_led_controller();

        let params = [LIGHT_DEL_GRP_PARAM, 0x7E, 0x8F, 0, 0, 0, 0, 0, 0, 0];
        let packet = mesh_packet(LGT_CMD_LIGHT_CONFIG_GRP, VENDOR_ID, &params);

        rf_link_data_callback(&packet);

        assert_eq!(pending_event(), 0, "a failed delete raises no event");
        assert!(!group_addresses().contains(&0x8F7E));

        reset_led_controller();
    }

    /// Group addition stores the address in the table and the flash log.
    #[test]
    fn test_rf_link_data_callback_light_config_grp_add() {
        reset_radio_state_for_test();
        reset_fakes();
        reset_led_controller();

        let params = [LIGHT_ADD_GRP_PARAM, 0xCD, 0x80, 0, 0, 0, 0, 0, 0, 0];
        let packet = mesh_packet(LGT_CMD_LIGHT_CONFIG_GRP, VENDOR_ID, &params);

        rf_link_data_callback(&packet);

        assert!(
            group_addresses().contains(&0x80CD),
            "the new group is stored in the membership table"
        );
        assert_eq!(
            pending_event(),
            LED_EVENT_FLASH_1HZ_4S,
            "a successful group change blinks the indicator"
        );
    }

    /// Device address configuration accepts the new address, consumes the
    /// validation flag and clears the stored header byte.
    #[test]
    fn test_rf_link_data_callback_config_dev_addr() {
        reset_radio_state_for_test();
        reset_fakes();
        set_flash_config_index(0);
        set_dev_addr_validation_pending(true);
        set_mesh_pair_enabled(true);

        // `params[2] != DEV_ADDR_PAR_WITH_MAC`, so the MAC match is not required.
        let params = [0x78, 0x56, 0, 0, 0, 0, 0, 0, 0, 0];
        let packet = mesh_packet(LGT_CMD_CONFIG_DEV_ADDR, VENDOR_ID, &params);

        rf_link_data_callback(&packet);

        assert_eq!(device_address(), 0x5678, "the new address is adopted");
        assert!(
            !dev_addr_validation_pending(),
            "the validation flag is consumed"
        );
        assert_eq!(
            flash_byte(FLASH_ADR_PAIRING + 1),
            0x00,
            "byte [1] of the config header is programmed to zero"
        );

        set_mesh_pair_enabled(false);
    }

    /// `LGT_CMD_SET_LIGHT` is queued with its parameters.
    #[test]
    fn test_rf_link_data_callback_set_light() {
        crate::light::reset_for_test();

        let params = [0x10, 0x30, 0, 0, 0, 0, 0, 0, 0, 0];
        let packet = mesh_packet(LGT_CMD_SET_LIGHT, VENDOR_ID, &params);

        rf_link_data_callback(&packet);

        let msg = LIGHT_CMD_CHANNEL
            .try_receive()
            .expect("LGT_CMD_SET_LIGHT must be queued");
        assert_eq!(msg.cmd, LGT_CMD_SET_LIGHT);
        let mut expected = [0u8; 16];
        expected[..params.len()].copy_from_slice(&params);
        assert_eq!(msg.params, expected);
        assert!(
            LIGHT_CMD_CHANNEL.try_receive().is_err(),
            "exactly one command must be queued"
        );
    }

    /// MAC address configuration erases and reprograms the MAC sector, then
    /// reboots.
    #[test]
    fn test_rf_link_data_callback_set_mac_addr() {
        let params = [0x11, 0x66, 0, 0, 0, 0, 0, 0, 0, 0];
        let packet = mesh_packet(LGT_CMD_SET_MAC_ADDR, VENDOR_ID, &params);

        reset_fakes();

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
    fn test_rf_link_data_callback_kick_out_out_of_mesh_writes_credentials() {
        // Moved from `sdk/factory_reset.rs::tests`: the credential writes now
        // live in this module, so the assertion follows them.
        // Install the factory radio configuration first: the kick-out writer
        // reads the credential buffers `set_radio_config` seeds, not the
        // crate-root constants directly.
        reset_radio_state_for_test();
        let expected_ltk = [0xAAu8; 16];
        // Only the LTK is read by the kick-out writer; the name and password
        // buffers are re-seeded with the factory defaults so the global
        // credential state is left the way `seed_pair_config` installed it.
        let mut default_name = [0u8; 16];
        default_name[..OUT_OF_MESH.len()].copy_from_slice(OUT_OF_MESH.as_bytes());
        let mut default_pwd = [0u8; 16];
        default_pwd[..MESH_PWD.len()].copy_from_slice(MESH_PWD.as_bytes());
        set_pair_config_mesh_credentials(default_name, default_pwd, expected_ltk);

        set_mesh_pair_enabled(true);
        set_dev_addr_validation_pending(false);

        let encoded_password = {
            // Compute the expected ciphertext with the real AES path: the same
            // key and password buffer the kick-out path feeds to
            // `encode_password`.
            let mut key = [0u8; 16];
            let sk_len = MESH_PWD_ENCODE_SK.len().min(key.len());
            key[..sk_len].copy_from_slice(&MESH_PWD_ENCODE_SK.as_bytes()[..sk_len]);
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

        // `KickoutReason::OutOfMesh` is wire value 0: erase, then rewrite.
        let params = [0u8, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let packet = mesh_packet(LGT_CMD_KICK_OUT, VENDOR_ID, &params);

        rf_link_data_callback(&packet);

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

        assert!(dev_addr_validation_pending());
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
        set_mesh_pair_enabled(false);
        set_dev_addr_validation_pending(false);
    }

    /// Any other reason only erases: no pairing data is written.
    #[test]
    fn test_rf_link_data_callback_kick_out_default_name_only_erases() {
        reset_fakes();
        crate::factory_reset::reset_for_test();
        FakeFlash::write_slice(FLASH_ADR_PAIRING, &[0xAA; 64]);

        // `KickoutReason::DefaultName` is wire value 1.
        let params = [1u8, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let packet = mesh_packet(LGT_CMD_KICK_OUT, VENDOR_ID, &params);

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
    fn test_rf_link_data_callback_kick_out_unknown_reason_falls_back() {
        set_mesh_pair_enabled(false);
        set_dev_addr_validation_pending(false);
        reset_fakes();
        crate::factory_reset::reset_for_test();

        // 3 is not a `KickoutReason`.
        let params = [3u8, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let packet = mesh_packet(LGT_CMD_KICK_OUT, VENDOR_ID, &params);

        rf_link_data_callback(&packet);

        // The flags block proves the credential path ran.
        assert_eq!(
            flash_byte(FLASH_ADR_PAIRING),
            PAIR_VALID_FLAG,
            "an unknown reason must fall back to the out-of-mesh credentials"
        );
        assert!(
            !dev_addr_validation_pending(),
            "mesh pairing is disabled, so the rejoin flag stays clear"
        );
    }

    /// Mesh pairing chunks reach the pairing state machine with their bytes.
    #[test]
    fn test_rf_link_data_callback_mesh_pair() {
        crate::mesh::reset_for_test();

        // `MeshPairName1` (state byte 0) followed by the first eight name bytes.
        let params = [0x00, 0x0A, 0x0B, 0, 0, 0, 0, 0, 0, 0];
        let packet = mesh_packet(LGT_CMD_MESH_PAIR, VENDOR_ID, &params);

        rf_link_data_callback(&packet);

        // The machine accumulates outside flash, so finish the sequence with the
        // immediate-effect command and assert what it persisted.
        let name = crate::mesh::pair_fsm_view().new_mesh_name;
        assert_eq!(name[0], 0x0A, "the name chunk is accumulated");
        assert_eq!(name[1], 0x0B);
    }

    /// A packet from another vendor is ignored.
    #[test]
    fn test_rf_link_data_callback_wrong_vendor_id() {
        crate::light::reset_for_test();

        let params = [LIGHT_ON_PARAM, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let packet = mesh_packet(LGT_CMD_LIGHT_ONOFF, 0xFFFF, &params);

        rf_link_data_callback(&packet);

        assert!(
            LIGHT_CMD_CHANNEL.try_receive().is_err(),
            "a foreign vendor id must be dropped before dispatch"
        );
    }

    /// A 2-byte opcode is unsupported and the packet is ignored.
    #[test]
    fn test_rf_link_data_callback_wrong_op_cmd_len() {
        crate::light::reset_for_test();

        let params = [LIGHT_ON_PARAM, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let mut packet = mesh_packet(LGT_CMD_LIGHT_ONOFF, VENDOR_ID, &params);
        // Bit 7 set with bit 6 clear is the parser's "2-byte opcode" pattern,
        // which it rejects outright.
        packet.att_write_mut().value.val[0] = LGT_CMD_LIGHT_ONOFF | 0x80;

        rf_link_data_callback(&packet);

        assert!(
            LIGHT_CMD_CHANNEL.try_receive().is_err(),
            "an unsupported opcode length must be dropped"
        );
    }

    // --- Tests for rf_link_light_event_callback ---

    /// Rebuild the node table around this node and assert the callback did it.
    ///
    /// The node table is the SDK's real state, so a different device address
    /// written by `mesh_node_table_init` proves the reset ran; the light
    /// subsystem's status recorder proves the publish ran.
    fn assert_node_table_and_status_rebuilt(cmd: u8) {
        reset_radio_state_for_test();
        set_device_address(0x0034);
        // Leave the table describing a different node, so the rebuild shows.
        MESH_NODE_ST.lock()[0].val.dev_adr = 0;
        MESH_NODE_MAX.store(7, core::sync::atomic::Ordering::Relaxed);
        status_probe::reset();
        reset_led_controller();

        rf_link_light_event_callback(cmd);

        assert_eq!(
            MESH_NODE_ST.lock()[0].val.dev_adr,
            0x34,
            "mesh_node_table_init re-registers this node"
        );
        assert_eq!(
            MESH_NODE_MAX.load(core::sync::atomic::Ordering::Relaxed),
            1,
            "the rebuild leaves only this node known"
        );
        assert_eq!(
            status_probe::calls(),
            1,
            "the lamp status is republished once"
        );
        assert_eq!(pending_event(), LED_EVENT_FLASH_1HZ_4S);

        reset_led_controller();
    }

    #[test]
    fn test_rf_link_light_event_callback_set_mesh_info() {
        assert_node_table_and_status_rebuilt(LGT_CMD_SET_MESH_INFO);
    }

    #[test]
    fn test_rf_link_light_event_callback_set_dev_addr() {
        assert_node_table_and_status_rebuilt(LGT_CMD_SET_DEV_ADDR);
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
    fn test_light_init_default() {
        // Setup: the HAL host seams are process-wide singletons.
        reset_fakes();
        // Seed the USB analog register with bit 4 set, so enabling the DP
        // pull-up is observable as a clear.
        FakeAnalog::set(0x00, 0xff);
        // A sentinel interval proves the real calibration ran.
        BLE_SCAN_RESPONSE_INTERVAL_US.store(0x1234, Ordering::Relaxed);

        // Execute
        light_init_default();

        // Verify: the scan-response interval was calibrated for a 32-tick/us clock.
        assert_eq!(
            BLE_SCAN_RESPONSE_INTERVAL_US.load(Ordering::Relaxed),
            0x92,
            "light_check_tick_per_us must calibrate the scan-response interval"
        );
        // Advertising/pairing was enabled.
        assert!(
            BLE_PERIPHERAL_ADVERTISING_ENABLED.load(Ordering::Relaxed),
            "rf_link_slave_pairing_enable(true) must enable advertising"
        );
        // The RF power table row for `RfPower::Power8dBm` (index 0) reached the
        // analog plane: a=0x25, b=0x7c, c=0x67, d=0x67.
        assert_eq!(FakeAnalog::get(tlsr8266_pac::REGA_RF_PA_GAIN), 0x25);
        assert_eq!(FakeAnalog::get(tlsr8266_pac::REGA_RF_PA_BIAS), 0x7c);
        assert_eq!(FakeAnalog::get(tlsr8266_pac::REGA_RF_OUTPUT_MATCH), 0x67);
        assert_eq!(FakeAnalog::get(tlsr8266_pac::REGA_RF_OUTPUT_POWER), 0x67);
        // USB DP pull-up enabled.
        assert_eq!(
            FakeAnalog::get(0x00),
            0xef,
            "enabling the USB DP pull-up must clear bit 4 of the USB analog register"
        );
        // Both timers configured: TMR1 capture ~120 Hz, TMR0 tick.
        assert_eq!(FakeMmio::reg32(0x628), CLOCK_SYS_CLOCK_1S / 120);
        assert_eq!(FakeMmio::reg32(0x624), CLOCK_SYS_CLOCK_1S);
        assert!(
            FakeMmio::reg8(0x620) & FLD_TMR::TMR0_EN.bits() as u8 != 0,
            "TMR0 must be left running"
        );
        // Re-pairing armed with the machine's interval and timeout.
        assert!(is_mesh_pair_enabled());
        let view = crate::mesh::pair_fsm_view();
        let (interval, timeout) = (view.mesh_pair_cmd_interval, view.mesh_pair_timeout);
        assert_eq!(interval, crate::mesh::MESH_PAIR_CMD_INTERVAL);
        assert_eq!(timeout, crate::mesh::MESH_PAIR_TIMEOUT);
    }

    // --- Tests for user_init ---

    /// Run `user_init` and return the store it hands the main loop.
    ///
    /// Every step runs for real against the HAL fakes, so the assertions live in
    /// the tests rather than in a mock call log.
    fn run_user_init() -> LightFlashStore {
        user_init()
    }

    #[test]
    fn test_user_init() {
        // Reset the process-wide host seams and the reset subsystem.
        reset_fakes();
        crate::factory_reset::reset_for_test();
        status_probe::reset();
        // Seed the USB analog register (bit 4 = pull-up disabled) and the port C
        // GPIO-function register, so the real `usb_dp_pullup_en` and
        // `gpio_set_func` calls below are observable.
        FakeAnalog::set(0x00, 0xff);
        FakeMmio::set_reg8(REG_GPIO_PC_FUNC, 0xff);
        // The advertisement payload is built from these two, so seed both. The
        // MAC lives in flash: `rf_link_slave_init` loads it from there (and would
        // overwrite an in-RAM seed), so the NOR image is what matters.
        let mac = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        FakeFlash::write_slice(FLASH_ADR_MAC, &mac);
        set_mac_id(mac);
        set_device_address(0x1234);

        // A fresh lamp state: the on/off restore starts a transition, and a
        // stale one from an earlier test would underflow embassy's `Instant`.
        crate::light::reset_for_test();

        let _flash_store = run_user_init();

        // 1. `check_ota_area_startup`: the blank staging area is already valid.
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_LIGHT_NEW_FW),
            0,
            "a blank OTA staging area is valid and must not be erased"
        );
        // 2. `light_init_default`.
        assert_eq!(
            BLE_SCAN_RESPONSE_INTERVAL_US.load(Ordering::Relaxed),
            0x92,
            "light_check_tick_per_us calibrated the scan-response interval"
        );
        assert!(BLE_PERIPHERAL_ADVERTISING_ENABLED.load(Ordering::Relaxed));
        // `rf_set_power_level_index(Power8dBm)` reached the analog plane.
        assert_eq!(FakeAnalog::get(tlsr8266_pac::REGA_RF_PA_GAIN), 0x25);
        assert_eq!(
            FakeAnalog::get(0x00),
            0xef,
            "user_init must enable the USB DP pull-up"
        );
        assert_eq!(
            FakeMmio::reg32(0x628),
            CLOCK_SYS_CLOCK_1S / 120,
            "light_hw_timer_config programmed TMR1's capture"
        );
        assert!(is_mesh_pair_enabled(), "mesh_pair_init armed re-pairing");
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
        // and the zeroed analog bank records the lamp as on: restoring that state
        // starts the colour timer.
        assert!(
            FakeMmio::reg8(0x620) & FLD_TMR::TMR1_EN.bits() as u8 != 0,
            "restoring the lamp on must start the colour timer"
        );
        // `rf_link_slave_init` reloads the key, which recomputes and installs the
        // access code.
        assert!(
            FakeMmio::writes()
                .iter()
                .any(|write| write.addr == 0x408 | tlsr8266_pac::REG_BASE_ADDR),
            "rf_link_slave_init must install the mesh access code"
        );
        // The boot path consulted the reset policy: step 1 is now programmed.
        assert_eq!(
            flash_byte(FLASH_ADR_RESET_CNT),
            0xFE,
            "a blank sector must start the sequence at step 1"
        );
        // The light subsystem built both payloads from the loaded MAC and this
        // node's address, and the facade installed them together with the
        // primary payload's manufacturer bytes.
        let (pri, rsp) = adv_private_data();
        // The manufacturer block the radio facade installs is present in the
        // advertisement's AD payload.
        let adv_bytes = adv_private_to_bytes(&pri);
        {
            let pkt = PKT_ADV.lock();
            assert!(
                pkt.adv_ind_module()
                    .data
                    .windows(adv_bytes.len())
                    .any(|window| window == adv_bytes),
                "the manufacturer data must be installed into the advertisement"
            );
        }
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
            device_address(),
            "the scan response carries this node's address"
        );
        for (i, &byte) in rsp_rsv.iter().enumerate() {
            assert_eq!(byte, i as u8, "rsv[{i}] must be {i}");
        }
        // Once from the on/off restore, once at the end of user_init.
        assert_eq!(
            status_probe::calls(),
            2,
            "device_status_update runs from the restore and at the end of init"
        );
        assert!(is_security_enabled(), "user_init enables mesh encryption");

        // A normal boot must not have erased anything or rebooted.
        assert!(
            !FakeMmio::writes()
                .iter()
                .any(|write| write.addr == REG_PWDN_CTRL | tlsr8266_pac::REG_BASE_ADDR),
            "a normal boot must not reboot the MCU"
        );
    }

    /// A boot that finds the factory-reset flag erases the configuration,
    /// blinks the OTA LED and reboots — the `FactoryResetAndReboot` arm.
    #[test]
    fn test_user_init_factory_reset_erases_blinks_and_reboots() {
        reset_fakes();
        crate::factory_reset::reset_for_test();
        // The previous five power cycles left the flag in the first slot.
        FakeFlash::write_slice(FLASH_ADR_RESET_CNT, &[0x80]);
        crate::light::reset_for_test();

        run_user_init();

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
    fn test_light_user_func_structure() {
        // `check_light_state_save` is an inherent method on the flash store, so
        // this test drives the real function and asserts its observable effects:
        // the 5 s save timer is polled, the save reaches flash, and the timer is
        // cleared so a second tick does not save again.
        reset_test_globals();
        crate::light::reset_for_test();
        crate::factory_reset::reset_for_test();
        reset_fakes();

        // Put the factory-reset sequence in the middle of its five-step run: the
        // only state in which `check_timing_step` has an observable effect. Step
        // 3 is on the card (0xF8), so the public boot path advances it to step 4
        // (0xF0) and arms step 4's 3 s minimum window
        // (`POWER_CYCLE_TIMING[3].0`). Step 1 is not used instead because steps
        // 1-3 have no timing bit and so no write to observe.
        FakeFlash::write_slice(FLASH_ADR_RESET_CNT + 2, &[0xF8]);
        assert_eq!(
            crate::factory_reset::handle_boot_sequence(),
            crate::factory_reset::ResetAction::None
        );
        assert_eq!(
            flash_byte(FLASH_ADR_RESET_CNT + 2),
            0xF0,
            "the boot must advance the sequence to step 4"
        );

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
        // Arm the pairing state machine's immediate-effect path, so the real
        // `mesh_pair_proc_effect` has flash state to change.
        crate::mesh::set_pair_effect_new_mesh(1);

        // Execute - light_user_func calls:
        // 1. flash_store.check_light_state_save();
        // 2. light_auth_check(); (which checks global state)
        // 3. factory_reset::check_timing_step(); — the seeded sequence above
        //    makes it write the step-4 timing bit, asserted below;
        // 4. mesh_pair_proc_effect();
        let mut flash_store = LightFlashStore::default();
        light_user_func(&mut flash_store);

        // The tick polled `factory_reset::check_timing_step`, whose 3 s minimum
        // window has elapsed: the step-4 timing bit lands in flash (0xF0 ->
        // 0xD0). Deleting step 3 of `light_user_func` leaves the byte at 0xF0.
        assert_eq!(
            flash_byte(FLASH_ADR_RESET_CNT + 2),
            0xD0,
            "the tick must run factory_reset::check_timing_step"
        );

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
        assert_eq!(
            flash_config_index(),
            Some(0x40),
            "the tick ran mesh_pair_proc_effect, which persisted the credentials"
        );
        assert_eq!(
            crate::mesh::pair_fsm_view().effect_new_mesh,
            0,
            "the effect flag was consumed by the tick"
        );

        // A second tick must not save again: `check_light_state_save` cleared the
        // timer, so it never even asks whether 5 s elapsed. The clock still reads
        // well past the window, so a second write could only come from a timer
        // that was not cleared.
        light_user_func(&mut flash_store);
        let mut after = [0u8; 14];
        FakeFlash::read_slice(0, &mut after);
        assert_eq!(&after[..7], &saved, "the first entry is unchanged");
        assert_eq!(&after[7..], &[0xFF; 7], "no second entry was programmed");
        assert_eq!(
            flash_config_index(),
            Some(0x40),
            "the second tick must not save the pairing record again"
        );
    }

    // --- Tests for main_loop ---

    /// One `main_loop` tick drives each stage, observed through what the stages
    /// actually change: the luminance save (the light tick), the SDK's pairing
    /// callback (the radio service) and the LED controller (the blink stage).
    #[test]
    fn test_main_loop_structure() {
        reset_test_globals();
        crate::light::reset_for_test();
        crate::factory_reset::reset_for_test();
        reset_fakes();
        install_recording_hooks();

        // Seed a pending luminance save: a colour transition stamps
        // `last_transition_time`.
        fake_clock::set_ticks(1000);
        begin_transition(500, 300, 800);
        fake_clock::set_ticks(1000 + 5000 * 1000 * 32 + 1);

        // A pending blink the LED stage must consume.
        crate::light::effects::config_led_event(LED_EVENT_FLASH_1HZ_4S);
        assert_eq!(pending_event(), LED_EVENT_FLASH_1HZ_4S);

        // Execute main_loop
        let mut flash_store = LightFlashStore::default();
        block_on(main_loop(&mut flash_store));

        // The light tick saved the luminance entry.
        let mut saved = [0u8; 7];
        FakeFlash::read_slice(0, &mut saved);
        assert_eq!(saved[0], 0xA5, "the tick's save path ran");
        // The radio service reached the SDK's pairing callback.
        assert_eq!(
            MESH_PAIR_PROC_CALLS.load(Ordering::Relaxed),
            1,
            "rf_link_slave_proc must service the pairing seam once"
        );
        // The LED stage consumed the pending event.
        assert_eq!(
            pending_event(),
            0,
            "led_indicate_step must start the pending blink"
        );
        reset_led_controller();
    }

    /// Regression test verifying the full response path:
    /// rf_link_response_callback(GET_STATUS) -> rf_link_slave_add_status -> the
    /// shared staged-response buffer
    /// produces exact unshifted VENDOR_ID at dat[8..10] and CW/WW/Brightness at dat[10..16].
    ///
    /// This lives in the application layer because the callback reads the light
    /// state owned by the light subsystem; the SDK test tree may no longer inject an
    /// application state object.
    #[test]
    fn test_rf_link_response_callback_to_slave_add_status_layout() {
        use tlsr8266_radio::mesh_ll::packet_processing::rf_link_slave_add_status;

        // Reset only the globals this path reads: the request and response
        // packets are built locally below.
        crate::light::reset_for_test();
        set_device_address(0x001F);
        tlsr8266_platform::window::end_ota();
        assert!(
            <LightPlatformRadio as tlsr8266_radio::RadioCoexistence>::start_status_window(),
            "status window must start for the layout probe"
        );
        set_slave_read_status_busy(LGT_CMD_LIGHT_STATUS);

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
        rf_link_slave_add_status::<LightPlatformRadio>(&pkt_light_status);

        let response = tlsr8266_platform::window::peek_status_response()
            .expect("the status packet must be present in the shared window");
        let dat = &response.att_data().dat;

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

        // This test runs (alphabetically) before the SDK suites, which assume an
        // empty shared status buffer. Leave the window stopped.
        <LightPlatformRadio as tlsr8266_radio::RadioCoexistence>::stop_status_window();
    }

    // --- Boot report and panic-info delivery ---
    //
    // `log_boot_reset_reason` / `report_panic_info` are the `app` half of the
    // old `diagnostics::check_panic_info`: `diagnostics` now reports values and
    // this file decides how they reach the UART and the mesh.

    /// Payload bytes per UART packet (`UART_PAYLOAD_MAX_SIZE` in
    /// `mesh_protocol::uart`).
    const PAYLOAD_CHUNK: usize = 39;

    fn expected_print(bytes: &[u8]) -> UartData {
        let mut message = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };

        message.data[2] = UartMsg::PrintMessage as u8;
        message.data[3..3 + bytes.len()].copy_from_slice(bytes);

        message
    }

    /// Asserts the async log sink received exactly `text`, chunked the way
    /// `UartStream::send` packets it.
    fn assert_uart_print(text: &[u8]) {
        let frames = LOG_FRAMES.lock().unwrap();
        let expected: std::vec::Vec<UartData> =
            text.chunks(PAYLOAD_CHUNK).map(expected_print).collect();
        assert_eq!(
            frames.len(),
            expected.len(),
            "one frame per {PAYLOAD_CHUNK}-byte chunk"
        );
        for (got, want) in frames.iter().zip(expected.iter()) {
            assert_eq!(got, want);
        }
    }

    /// Clears the module statics and the HAL host seams, then installs the
    /// recording log sinks so a log line is captured instead of reaching the
    /// real 12-slot UART channel. `FakeAnalog` is a process-wide singleton, so
    /// every test here starts from a zeroed analog bank.
    fn reset_log_capture() {
        reset_for_test();
        FakeMmio::reset();
        FakeAnalog::reset();
        FakeFlash::reset();
        install_log_recorder();
    }

    /// Seed the panic-info sector with the layout `write_panic_bytes` produces:
    /// valid flag at `+0`, `len` at `+1`, `body` at `+3`.
    fn seed_panic_info(body: &[u8], len: u16) {
        FakeFlash::write_slice(FLASH_ADR_PANIC_INFO, &[PANIC_VALID_FLAG]);
        FakeFlash::write_slice(FLASH_ADR_PANIC_INFO + 1, &len.to_le_bytes());
        FakeFlash::write_slice(FLASH_ADR_PANIC_INFO + 3, body);
    }

    /// Restores the host default (the in-memory NOR device) even if the owning
    /// test panics, so a failure cannot strand later tests in `SpiRegisters`
    /// mode. The device is process-global, hence the guard.
    struct RestoreInMemory;

    impl Drop for RestoreInMemory {
        fn drop(&mut self) {
            FakeFlash::set_mode(FakeFlashMode::InMemory);
        }
    }

    /// A panic record that cannot be read from a dead bus is reported as
    /// explicitly unreadable, not as a partial body or as nothing at all.
    #[test]
    fn test_log_boot_reset_reason_panic_unreadable() {
        reset_log_capture();
        FakeFlash::set_mode(FakeFlashMode::SpiRegisters);
        let _restore = RestoreInMemory;

        // `reg_master_spi_ctrl`: the BUSY flag never clears.
        FakeMmio::on_read8(0x0d, |_| FLD_MASTER_SPI::BUSY.bits());

        log_boot_reset_reason(BootResetReason::Panic {
            breadcrumb: CRUMB_PANIC,
        });

        assert_uart_print(b"[PANIC] sentinel=0xe1 breadcrumb=255 msg=<unreadable>\n");
    }

    /// A clean previous boot prints nothing at all.
    #[test]
    fn test_log_boot_reset_reason_clean() {
        reset_log_capture();

        log_boot_reset_reason(BootResetReason::Clean);

        assert!(
            LOG_FRAMES.lock().unwrap().is_empty(),
            "no line may be logged"
        );
    }

    #[test]
    fn test_log_boot_reset_reason_wd_reset() {
        reset_log_capture();

        log_boot_reset_reason(BootResetReason::Watchdog {
            breadcrumb: CRUMB_MAIN_LOOP,
        });

        assert_uart_print(b"[WD_RESET] sentinel=0xa5 breadcrumb=3\n");
    }

    #[test]
    fn test_log_boot_reset_reason_panic_message() {
        reset_log_capture();

        let body = b"panic body";
        seed_panic_info(body, body.len() as u16);

        log_boot_reset_reason(BootResetReason::Panic {
            breadcrumb: CRUMB_PANIC,
        });

        assert_uart_print(b"[PANIC] sentinel=0xe1 breadcrumb=255 msg=panic body\n");
        // `report_panic_info()` owns erasing the sector, not the boot report.
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_PANIC_INFO),
            0,
            "the boot report must leave the panic sector intact"
        );
    }

    /// A panic body longer than the stream budget is clipped and marked, and the
    /// whole line still fits the 128-byte `UartStream` used by `uprintln!`.
    #[test]
    fn test_log_boot_reset_reason_panic_message_truncated() {
        reset_log_capture();

        let body = [b'x'; 200];
        seed_panic_info(&body, body.len() as u16);

        // `std::vec::Vec` explicitly: `heapless::Vec` is the crate-wide import.
        let mut expected = std::vec::Vec::new();
        expected.extend_from_slice(b"[PANIC] sentinel=0xe1 breadcrumb=255 msg=");
        expected.extend_from_slice(&body[..PANIC_LOG_BODY_MAX]);
        expected.extend_from_slice(b"...\n");

        // Worst case: crumb 255 and a full-length body. The stream is 128 bytes,
        // and the fixed prefix alone spans more than one 39-byte payload, so the
        // clipped line is packetised, never truncated by the stream itself.
        assert_eq!(expected.len(), 117);
        assert!(expected.len() <= 128);
        assert_eq!(&expected[expected.len() - 4..], b"...\n");
        assert_eq!(expected.len().div_ceil(PAYLOAD_CHUNK), 3);

        log_boot_reset_reason(BootResetReason::Panic {
            breadcrumb: CRUMB_PANIC,
        });

        assert_uart_print(&expected);
    }

    /// A message longer than the flash read window is marked with `...` even
    /// when the bytes that *were* read decode to exactly the 72-byte budget, so
    /// a cut can never look complete.
    #[test]
    fn test_log_boot_reset_reason_panic_message_marks_short_body_cut_by_read() {
        reset_log_capture();

        // 72 printable bytes followed by a byte that is invalid UTF-8, then more
        // data beyond the read window: the decoded prefix is exactly the budget.
        let mut body = [b'a'; 200];
        body[PANIC_LOG_BODY_MAX] = 0xFF;
        seed_panic_info(&body, body.len() as u16);

        let mut expected = std::vec::Vec::new();
        expected.extend_from_slice(b"[PANIC] sentinel=0xe1 breadcrumb=255 msg=");
        expected.extend_from_slice(&body[..PANIC_LOG_BODY_MAX]);
        expected.extend_from_slice(b"...\n");

        assert_eq!(expected.len(), 117);

        log_boot_reset_reason(BootResetReason::Panic {
            breadcrumb: CRUMB_PANIC,
        });

        assert_uart_print(&expected);
    }

    /// A message that fits the flash read window exactly but not the 72-byte
    /// stream budget is still marked, i.e. the clip alone can require `...`.
    #[test]
    fn test_log_boot_reset_reason_panic_message_marks_body_clipped_within_read() {
        reset_log_capture();

        let body = [b'w'; PANIC_READ_MAX];
        seed_panic_info(&body, body.len() as u16);

        let mut expected = std::vec::Vec::new();
        expected.extend_from_slice(b"[PANIC] sentinel=0xe1 breadcrumb=255 msg=");
        expected.extend_from_slice(&body[..PANIC_LOG_BODY_MAX]);
        expected.extend_from_slice(b"...\n");

        assert_eq!(expected.len(), 117);

        log_boot_reset_reason(BootResetReason::Panic {
            breadcrumb: CRUMB_PANIC,
        });

        assert_uart_print(&expected);
    }

    /// AC4: an invalid panic flag means the sector is not trusted at all, so the
    /// boot report stays silent.
    #[test]
    fn test_log_boot_reset_reason_panic_invalid_flag() {
        reset_log_capture();

        FakeFlash::write_slice(FLASH_ADR_PANIC_INFO, &[PANIC_VALID_FLAG ^ 0xFF]);

        log_boot_reset_reason(BootResetReason::Panic {
            breadcrumb: CRUMB_MAIN_LOOP,
        });

        // No line at all: the invalid flag short-circuits before the length read.
        assert!(
            LOG_FRAMES.lock().unwrap().is_empty(),
            "no line may be logged"
        );
    }

    /// AC4: a length beyond the 256-byte sector record is rejected before any
    /// body bytes are read, and nothing is logged.
    #[test]
    fn test_log_boot_reset_reason_panic_length_over_limit() {
        reset_log_capture();

        let len: u16 = 257;
        seed_panic_info(&[b'x'; 8], len);

        log_boot_reset_reason(BootResetReason::Panic {
            breadcrumb: CRUMB_MAIN_LOOP,
        });

        assert!(
            LOG_FRAMES.lock().unwrap().is_empty(),
            "no line may be logged"
        );
    }

    /// AC4: the bounded read can end inside a multi-byte character. The partial
    /// code point is dropped (`from_utf8` error clamp), never replaced and never
    /// logged raw, and the clipped line is marked with `...`.
    #[test]
    fn test_log_boot_reset_reason_panic_truncated_utf8() {
        reset_log_capture();

        // 75 ASCII bytes, then the two bytes of `é` straddling the 76-byte read
        // bound, then more data that is never read.
        let mut body = [b'a'; 200];
        body[PANIC_READ_MAX - 1] = 0xC3;
        body[PANIC_READ_MAX] = 0xA9;
        let len = body.len() as u16;

        // The clamp decodes 75 ASCII bytes, then `clip_to_uart_budget` keeps the
        // 72 the stream budget allows and the ellipsis marks the cut.
        let mut expected = std::vec::Vec::new();
        expected.extend_from_slice(b"[PANIC] sentinel=0xe1 breadcrumb=255 msg=");
        expected.extend_from_slice(&body[..PANIC_LOG_BODY_MAX]);
        expected.extend_from_slice(b"...\n");

        assert_eq!(body[PANIC_READ_MAX - 1], 0xC3);
        assert_eq!(expected.len(), 117);

        seed_panic_info(&body, len);

        log_boot_reset_reason(BootResetReason::Panic {
            breadcrumb: CRUMB_PANIC,
        });

        // Byte-exact: no replacement character (0xEF 0xBF 0xBD) and no stray
        // lead byte (0xC3) can appear in the logged line.
        assert_uart_print(&expected);
        assert!(!expected.windows(3).any(|w| w == [0xEF, 0xBF, 0xBD]));
        assert!(!expected.contains(&0xC3));
        assert_eq!(&expected[expected.len() - 5..], b"a...\n");
    }

    #[test]
    fn test_report_panic_info_no_valid_flag() {
        reset_for_test();
        // A blank sector reads 0xFF, which is not a valid panic flag.
        FakeFlash::reset();

        // Run the report (should return early)
        block_on(report_panic_info());

        // Nothing was reported, so the sector must not be cleared.
        assert_eq!(FakeFlash::erase_count(FLASH_ADR_PANIC_INFO), 0);
    }

    #[test]
    fn test_report_panic_info_with_valid_data() {
        reset_for_test();

        let test_message = b"Test panic message";
        let message_len = test_message.len() as u16;
        seed_panic_info(test_message, message_len);

        block_on(report_panic_info());

        // Three messages are queued for the mesh (two data chunks and one empty
        // terminator), each carrying one 10-byte panic fragment.
        let queued = queues::MESH_TX_QUEUE.lock();
        assert_eq!(queued.len(), 3, "two data chunks plus the terminator");
        let first = &queued[0].pkt.att_write().value.val;
        assert_eq!(first[0] & 0x3f, LGT_PANIC_MSG);
        assert_eq!(&first[3..13], &test_message[..10]);
        let second = &queued[1].pkt.att_write().value.val;
        assert_eq!(
            &second[3..3 + (test_message.len() - 10)],
            &test_message[10..]
        );
        let terminator = &queued[2].pkt.att_write().value.val;
        assert_eq!(terminator[0] & 0x3f, LGT_PANIC_MSG);
        assert_eq!(&terminator[3..13], &[0u8; 10]);
        drop(queued);

        // The report ends by erasing the sector, so the flag is back to blank.
        assert_eq!(FakeFlash::erase_count(FLASH_ADR_PANIC_INFO), 1);
        let mut flag = [0u8; 1];
        FakeFlash::read_slice(FLASH_ADR_PANIC_INFO, &mut flag);
        assert_eq!(flag[0], 0xFF, "the panic sector is cleared after reporting");
    }
}
