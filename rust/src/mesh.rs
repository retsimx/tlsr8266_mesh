use core::ptr::addr_of;

#[cfg(not(test))]
use embassy_time::{Duration, Instant, Timer, WithTimeout};
use heapless::Vec;

use crate::config::{FLASH_ADR_PAIRING, VENDOR_ID};
use crate::main_light::{light_slave_tx_command, rf_link_light_event_callback};
use crate::sdk::ble_app::ble_ll_pair::{
    pair_enc_packet_mesh, pair_save_key, MESH_PAIR_ENABLE, PAIR_AC, PAIR_CONFIG_MESH_LTK,
    PAIR_CONFIG_MESH_NAME, PAIR_CONFIG_MESH_PWD, PAIR_LOGIN_OK, PAIR_SETTING_FLAG, PAIR_STATE,
    SECURITY_ENABLE,
};
use crate::sdk::ble_app::flash_config::{access_code, pair_load_key, SYS_CHN_LISTEN};
use crate::sdk::ble_app::light_ll::connection_management::BLE_PERIPHERAL_CONNECTION_ACTIVE;
use crate::sdk::ble_app::light_ll::mesh_management::{
    mesh_node_table_init, rf_link_match_group_mac,
};
use crate::sdk::ble_app::light_ll::packet_processing::{
    is_add_packet_buf_ready, rf_link_add_tx_packet, rf_link_rc_data,
};
use crate::sdk::ble_app::queues::{self, SendPkt, MESH_RX_CHANNEL, MESH_TX_QUEUE, MESH_TX_WAKE};
use crate::sdk::ble_app::rf_drv_8266::{
    rf_set_ble_access_code, rf_set_ble_channel, rf_set_ble_crc_adv, rf_set_rxmode,
    rf_set_tx_rx_off, rf_start_srx2tx,
};
use crate::sdk::ble_app::rf_drv_8266::{
    DEVICE_ADDRESS, FLASH_CONFIGURATION_INDEX, MESH_DEVICE_ADDRESS_VALIDATION_PENDING,
};
use crate::sdk::common::sync::CriticalSectionMutex;
use crate::sdk::common::sync::SimplifyLS;
use crate::sdk::drivers::flash::flash_write_page;
use crate::sdk::light::*;
use crate::sdk::mcu::clock::{
    clock_time, clock_time64, clock_time_exceed, sleep_us, CLOCK_SYS_CLOCK_1US,
};
use crate::uart_manager::light_mesh_rx_cb;
use crate::{uprintln, BIT};
use mesh_protocol::packet_types::{Packet, PacketAttCmd, PacketAttValue, PacketL2capHead};

pub const MESH_PAIR_CMD_INTERVAL: u32 = 500;

//unit: s
pub const MESH_PAIR_TIMEOUT: u32 = 10;

/// Mesh pairing state machine commands for re-pairing devices in an existing mesh.
///
/// Credentials are transmitted in 6 messages (Name1/2, Pwd1/2, Ltk1/2), then
/// devices switch to the new mesh using MeshPairEffectDelay (delayed switch).
///
/// Note: MeshPairEffect (immediate switch) exists for receiving only - it's not
/// sent by this implementation as it was part of a broken single-device optimization.
#[derive(Clone, Copy, PartialEq, Debug)]
enum MeshPairState {
    MeshPairName1 = 0,
    MeshPairName2,
    MeshPairPwd1,
    MeshPairPwd2,
    MeshPairLtk1,
    MeshPairLtk2,
    MeshPairEffectDelay,
    MeshPairEffect, // Receive-only: immediate effect (not sent by this code)
    MeshPairDefaultMesh,
}

impl TryFrom<u8> for MeshPairState {
    type Error = ();

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == MeshPairState::MeshPairName1 as u8 => Ok(MeshPairState::MeshPairName1),
            x if x == MeshPairState::MeshPairName2 as u8 => Ok(MeshPairState::MeshPairName2),
            x if x == MeshPairState::MeshPairPwd1 as u8 => Ok(MeshPairState::MeshPairPwd1),
            x if x == MeshPairState::MeshPairPwd2 as u8 => Ok(MeshPairState::MeshPairPwd2),
            x if x == MeshPairState::MeshPairLtk1 as u8 => Ok(MeshPairState::MeshPairLtk1),
            x if x == MeshPairState::MeshPairLtk2 as u8 => Ok(MeshPairState::MeshPairLtk2),
            x if x == MeshPairState::MeshPairEffectDelay as u8 => {
                Ok(MeshPairState::MeshPairEffectDelay)
            }
            x if x == MeshPairState::MeshPairEffect as u8 => Ok(MeshPairState::MeshPairEffect),
            x if x == MeshPairState::MeshPairDefaultMesh as u8 => {
                Ok(MeshPairState::MeshPairDefaultMesh)
            }
            _ => Err(()),
        }
    }
}

/// Outcome of the send-buffer scan, computed inside the send critical section.
enum SendScan {
    /// Nothing is queued: park on the wake signal until a producer pushes.
    Empty,
    /// Packets are queued but none is due: park until the earliest due tick.
    Wait(u64),
    /// The first due packet, removed from the buffer.
    Ready(SendPkt),
}

/// Everything the mesh re-pairing state machine owns.
///
/// These 13 fields used to live on `MeshManager`, which meant every pairing
/// call reached them through an aliasable `&'static mut App`. They now have one
/// owner: [`MESH_PAIR`]. Every lock is short — no guard is ever held across a
/// flash read/write (`pair_load_key`, `pair_save_key`, `flash_write_page`) or
/// across a radio transmission.
pub struct MeshPairingState {
    mesh_pair_start_time: u32,
    default_mesh_time: u32,
    default_mesh_effect_delay_ref: u32,
    new_mesh_name: [u8; 16],
    new_mesh_pwd: [u8; 16],
    new_mesh_ltk: [u8; 16],
    default_mesh_time_ref: u32,
    effect_new_mesh: u8,
    effect_new_mesh_delay_time: u32,
    mesh_pair_cmd_interval: u32,
    mesh_pair_timeout: u32,
    mesh_pair_time: u32,
    mesh_pair_state: MeshPairState,
}

impl MeshPairingState {
    /// Const constructor so [`MESH_PAIR`] can be a `static` initialiser.
    pub const fn new() -> Self {
        Self {
            mesh_pair_start_time: 0,
            default_mesh_time: 0,
            default_mesh_effect_delay_ref: 0,
            new_mesh_name: [0; 16],
            new_mesh_pwd: [0; 16],
            new_mesh_ltk: [0; 16],
            default_mesh_time_ref: 0,
            effect_new_mesh: 0,
            effect_new_mesh_delay_time: 0,
            mesh_pair_cmd_interval: 0,
            mesh_pair_timeout: 0,
            mesh_pair_time: 0,
            mesh_pair_state: MeshPairState::MeshPairName1,
        }
    }
}

/// Sole owner of the mesh pairing state.
pub static MESH_PAIR: CriticalSectionMutex<MeshPairingState> =
    CriticalSectionMutex::new(MeshPairingState::new());

/// Restore the module-level pairing state and queues between host tests.
///
/// Host tests share one process, so a test that leaves `MESH_PAIR` or the mesh
/// queues dirty would leak into the next one.
#[cfg(test)]
pub fn reset_for_test() {
    *MESH_PAIR.lock() = MeshPairingState::new();
    queues::reset_for_test();
}

/// Send a message to the mesh and return the sequence number it was given.
///
/// `data`: 13 bytes of data usually in form `[op, vendor_id_hi, vendor_id_lo, params...]`
/// `destination`: the address of the destination
/// `retransmit_count`: how many times the packet should be rebroadcast at each node
/// `send_ack`: whether the destination device should send an ack after handling it
#[cfg_attr(test, mry::mry)]
pub fn send_mesh_message(
    data: &Vec<u8, 13>,
    destination: u16,
    retransmit_count: u8,
    send_ack: bool,
) -> [u8; 3] {
    let pkt = light_slave_tx_command(data, destination, retransmit_count, send_ack);
    let (group_match, device_match) = rf_link_match_group_mac(&pkt);
    if group_match || device_match {
        queues::enqueue_mesh_rx(&pkt);
        if !device_match {
            queues::enqueue_mesh_tx(
                &pkt,
                0,
                pkt.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
            );
        }
    } else {
        queues::enqueue_mesh_tx(
            &pkt,
            0,
            pkt.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
        );
    }

    pkt.mesh().sno
}

/// Enable the re-pairing state machine and arm its command interval and timeout.
#[cfg_attr(test, mry::mry)]
pub fn mesh_pair_init() {
    MESH_PAIR_ENABLE.set(true);
    let mut pair = MESH_PAIR.lock();
    pair.mesh_pair_cmd_interval = MESH_PAIR_CMD_INTERVAL;
    pair.mesh_pair_timeout = MESH_PAIR_TIMEOUT;
}

/// Apply a pending switch to the new mesh, if one is due.
///
/// The due check runs under a short [`MESH_PAIR`] lock; the save that follows
/// takes its own locks and copies the credentials out before it touches flash.
#[cfg_attr(test, mry::mry)]
pub fn mesh_pair_proc_effect() {
    let due = {
        let pair = MESH_PAIR.lock();
        pair.effect_new_mesh != 0
            || (pair.effect_new_mesh_delay_time != 0
                && clock_time_exceed(
                    pair.effect_new_mesh_delay_time,
                    pair.mesh_pair_cmd_interval * 1000,
                ))
    };

    if !due {
        return;
    }

    save_effect_new_mesh();

    let mut pair = MESH_PAIR.lock();
    pair.effect_new_mesh = 0;
    pair.effect_new_mesh_delay_time = 0;
}

/// Marks device address configuration as successfully validated.
/// Called when LGT_CMD_CONFIG_DEV_ADDR is processed and device address is accepted.
/// Sets the validation flag to false, allowing mesh credentials to be saved.
#[cfg_attr(test, mry::mry)]
pub fn mesh_device_address_validation_completed() {
    MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false); // validation completed successfully
    if MESH_PAIR_ENABLE.get() && FLASH_CONFIGURATION_INDEX.get() >= 0 {
        // Only write to flash if we have a valid configuration index
        // When FLASH_CONFIGURATION_INDEX is -1 (no config), we shouldn't write anything
        // Write to byte [1] of the header to clear the device address validation pending flag
        // The header is stored at FLASH_CONFIGURATION_INDEX + 0x00, and byte [1] holds the validation state
        let mut data: [u8; 1] = [0];
        flash_write_page(
            (FLASH_ADR_PAIRING as i32 + FLASH_CONFIGURATION_INDEX.get() + 1) as u32,
            1,
            data.as_mut_ptr(),
        );
    }
}

/// Feed one received mesh pairing command into the pairing state machine.
#[cfg_attr(test, mry::mry)]
pub fn mesh_pair_cb(params: &Vec<u8, 10>) {
    let mut pair = MESH_PAIR.lock();

    if pair.default_mesh_time_ref != 0 {
        pair.default_mesh_time_ref = clock_time() | 1;
    }
    let cmd = MeshPairState::try_from(params[0]);
    if cmd.is_err() {
        return;
    }

    match cmd.unwrap() {
        MeshPairState::MeshPairName1 => {
            pair.mesh_pair_start_time = clock_time() | 1;
            pair.new_mesh_name[0..8].copy_from_slice(&params[1..9]);
        }
        MeshPairState::MeshPairName2 => pair.new_mesh_name[8..16].copy_from_slice(&params[1..9]),
        MeshPairState::MeshPairPwd1 => pair.new_mesh_pwd[0..8].copy_from_slice(&params[1..9]),
        MeshPairState::MeshPairPwd2 => pair.new_mesh_pwd[8..16].copy_from_slice(&params[1..9]),
        MeshPairState::MeshPairLtk1 => pair.new_mesh_ltk[0..8].copy_from_slice(&params[1..9]),
        MeshPairState::MeshPairLtk2 => pair.new_mesh_ltk[8..16].copy_from_slice(&params[1..9]),
        MeshPairState::MeshPairEffectDelay => {
            pair.effect_new_mesh_delay_time = clock_time() | 1;
            if pair.default_mesh_time_ref != 0 {
                /* Keep default_mesh_time_ref non-zero */
                pair.default_mesh_time = pair.mesh_pair_cmd_interval * 2;
            }
        }
        MeshPairState::MeshPairEffect => pair.effect_new_mesh = 1,
        MeshPairState::MeshPairDefaultMesh => {
            pair.default_mesh_effect_delay_ref = clock_time() | 1;
            pair.default_mesh_time = params[1] as u32 * 1000;
        }
    }
}

fn mesh_cmd_notify(op: u8, p: &Vec<u8, 10>, dev_adr: u16) -> i32 {
    let mut err = -1;
    if BLE_PERIPHERAL_CONNECTION_ACTIVE.get() && PAIR_LOGIN_OK.get() {
        if p.len() > 10 {
            //max length of par is 10
            return -1;
        }

        let mut pkt_notify = PacketAttCmd {
            head: PacketL2capHead {
                dma_len: 0x1d,   // dma_len
                _type: 0x02,     // type
                rf_len: 0x1b,    // rf_len
                l2cap_len: 0x17, // u16
                chan_id: 0x04,   // chanid
            },
            opcode: 0x1b, // notify
            handle: 0x12,
            handle1: 0x00, // status handler
            value: PacketAttValue {
                sno: [op | 0xc0, (VENDOR_ID & 0xff) as u8, (VENDOR_ID >> 8) as u8],
                src: [(dev_adr & 0xFF) as u8, (dev_adr >> 8) as u8],
                dst: [0; 2],
                val: [0; 23],
            },
        };

        pkt_notify.value.val[3..3 + p.len()].copy_from_slice(&p[0..p.len()]);

        if is_add_packet_buf_ready()
            && rf_link_add_tx_packet(&Packet {
                att_cmd: pkt_notify,
            })
            .is_ok()
        {
            err = 0;
        }
    }

    err
}

fn mesh_pair_complete_notify() -> i32 {
    let par = [CMD_NOTIFY_MESH_PAIR_END];
    mesh_cmd_notify(
        LGT_CMD_MESH_CMD_NOTIFY,
        &Vec::from_slice(&par).unwrap(),
        DEVICE_ADDRESS.get(),
    )
}

/// Clear the pending credentials and mark pairing as settled.
fn safe_effect_new_mesh_finish() {
    {
        let mut pair = MESH_PAIR.lock();
        pair.new_mesh_name = [0; 16];
        pair.new_mesh_pwd = [0; 16];
        pair.new_mesh_ltk = [0; 16];
        pair.mesh_pair_start_time = 0;
    }
    *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
}

/// Persist the mesh credentials collected by the pairing state machine.
///
/// Every value the flash path needs is copied out of [`MESH_PAIR`] first, so no
/// guard is alive when `pair_load_key` / `pair_save_key` erase or write the
/// pairing sector.
fn save_effect_new_mesh() {
    let (default_mesh_active, use_pair_ltk_mesh) = {
        let pair = MESH_PAIR.lock();
        (pair.default_mesh_time_ref != 0, pair.effect_new_mesh != 0)
    };

    // Only skip saving if either:
    // 1. We're in a default mesh timeout period, OR
    // 2. Device address validation is still pending (not yet completed)
    if default_mesh_active || MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get() {
        mesh_pair_complete_notify();
        sleep_us(1000);
        /* Switch to normal mesh */
        pair_load_key(); // flash read: MESH_PAIR is not held
        MESH_PAIR.lock().default_mesh_time_ref = 0;

        mesh_node_table_init();
        crate::light_manager::device_status_update();
        safe_effect_new_mesh_finish();
        return;
    }

    // Copy the credentials out before the PAIR_STATE write and the flash save.
    let (new_mesh_name, new_mesh_pwd, new_mesh_ltk) = {
        let pair = MESH_PAIR.lock();
        (pair.new_mesh_name, pair.new_mesh_pwd, pair.new_mesh_ltk)
    };

    {
        let mut pair_state = PAIR_STATE.lock();

        if !use_pair_ltk_mesh {
            pair_state.pair_nn.copy_from_slice(&new_mesh_name);
            pair_state.pair_pass.copy_from_slice(&new_mesh_pwd);
            pair_state.pair_ltk.copy_from_slice(&new_mesh_ltk);
        } else {
            let pair_ltk_mesh = pair_state.pair_ltk_mesh;
            pair_state.pair_ltk.copy_from_slice(&pair_ltk_mesh);
        }
    }

    mesh_pair_complete_notify();

    // make sure not receive legacy mesh data from now on
    critical_section::with(|_| {
        pair_save_key(); // flash write: MESH_PAIR is not held
        rf_set_ble_access_code(PAIR_AC.get()); // use new access code at once.
        rf_link_light_event_callback(LGT_CMD_SET_MESH_INFO); // clear online status :mesh_node_table_init()
        sleep_us(1000);
        crate::sdk::ble_app::rf_drv_8266::rf_clear_rx_irq(); // clear current rx in buffer
    });

    safe_effect_new_mesh_finish();
}

/// Point the radio at the configured default mesh and arm its timeout.
fn switch_to_default_mesh(delay_s: u8) {
    {
        let mut pair = MESH_PAIR.lock();
        pair.default_mesh_time_ref = clock_time() | 1;
        pair.default_mesh_time = delay_s as u32 * 1000;
    }

    /* Only change AC and LTK */
    PAIR_AC.set(access_code(
        &*PAIR_CONFIG_MESH_NAME.lock(),
        &*PAIR_CONFIG_MESH_PWD.lock(),
    ));
    PAIR_STATE
        .lock()
        .pair_ltk
        .copy_from_slice(&*PAIR_CONFIG_MESH_LTK.lock());
}

impl MeshPairingState {
    /// First phase of the pairing timer: apply a default-mesh switch or cancel.
    ///
    /// Returns `(load_key, switch_delay_s)`. `load_key` reaches flash and
    /// `switch_delay_s` needs the radio statics, so both are deferred until the
    /// caller has dropped its [`MESH_PAIR`] guard.
    fn timer_step(&mut self) -> (bool, Option<u8>) {
        let mut load_key = false;
        let mut switch_delay = None;

        if self.default_mesh_effect_delay_ref != 0
            && clock_time_exceed(
                self.default_mesh_effect_delay_ref,
                MESH_PAIR_CMD_INTERVAL * 1000,
            )
        {
            self.default_mesh_effect_delay_ref = 0;

            if self.default_mesh_time == 0x00 {
                load_key = true;
                self.default_mesh_time_ref = 0;
            } else {
                switch_delay = Some((self.default_mesh_time / 1000) as u8);
            }
        } else if self.default_mesh_time_ref != 0
            && clock_time_exceed(self.default_mesh_time_ref, self.default_mesh_time * 1000)
        {
            /* Switch to normal mesh */
            if self.default_mesh_time == 255000 {
                self.default_mesh_time_ref = clock_time() | 1;
            } else {
                load_key = true;
                self.default_mesh_time_ref = 0;
            }
        }

        (load_key, switch_delay)
    }

    /// Second phase: has the pairing window expired?
    fn timed_out(&self) -> bool {
        self.mesh_pair_start_time != 0
            && clock_time_exceed(
                self.mesh_pair_start_time,
                self.mesh_pair_timeout * 1000 * 1000,
            )
    }

    /// Third phase: advance the re-pairing broadcast state machine.
    ///
    /// Returns the 16-byte parameter block to broadcast, or `None` when the
    /// command interval is not due or the machine has finished. Touches no
    /// flash and no radio; the caller transmits after dropping the guard.
    fn interval_tick(&mut self) -> Option<[u8; 16]> {
        let mut op_para: [u8; 16] = [0; 16];

        let flag = *PAIR_SETTING_FLAG.lock();
        if !(flag as u8 >= ePairState::PairSetMeshTxStart as u8
            && clock_time_exceed(self.mesh_pair_time, self.mesh_pair_cmd_interval * 1000))
        {
            return None;
        }

        self.mesh_pair_time = clock_time();
        if flag == ePairState::PairSetMeshTxStart {
            let pair_state = PAIR_STATE.lock();

            op_para[0] = LGT_CMD_MESH_PAIR;
            op_para[3] = self.mesh_pair_state as u8;
            match self.mesh_pair_state {
                MeshPairState::MeshPairName1 => {
                    // send mesh name [0-7]
                    op_para[4..4 + 8].copy_from_slice(&pair_state.pair_nn[0..8]);
                    self.mesh_pair_state = MeshPairState::MeshPairName2;
                }
                MeshPairState::MeshPairName2 => {
                    // send mesh name [8-15]
                    op_para[4..4 + 8].copy_from_slice(&pair_state.pair_nn[8..16]);
                    self.mesh_pair_state = MeshPairState::MeshPairPwd1;
                }
                MeshPairState::MeshPairPwd1 => {
                    // send mesh pwd [0-7]
                    op_para[4..4 + 8].copy_from_slice(&pair_state.pair_pass[0..8]);
                    self.mesh_pair_state = MeshPairState::MeshPairPwd2;
                }
                MeshPairState::MeshPairPwd2 => {
                    // send mesh pwd [8-15]
                    op_para[4..4 + 8].copy_from_slice(&pair_state.pair_pass[8..16]);
                    self.mesh_pair_state = MeshPairState::MeshPairLtk1;
                }
                MeshPairState::MeshPairLtk1 => {
                    // send mesh ltk [0-7]
                    op_para[4..4 + 8].copy_from_slice(&pair_state.pair_ltk_mesh[0..8]);
                    self.mesh_pair_state = MeshPairState::MeshPairLtk2;
                }
                MeshPairState::MeshPairLtk2 => {
                    // send mesh ltk [8-15]
                    op_para[4..4 + 8].copy_from_slice(&pair_state.pair_ltk_mesh[8..16]);
                    self.mesh_pair_state = MeshPairState::MeshPairName1;
                    *PAIR_SETTING_FLAG.lock() = ePairState::PairSetMeshTxDone;
                }
                _ => {
                    self.mesh_pair_state = MeshPairState::MeshPairName1;
                    self.mesh_pair_start_time = 0;
                    *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
                    return None;
                }
            }
        } else if *PAIR_SETTING_FLAG.lock() == ePairState::PairSetMeshTxDone {
            // get mesh nodes' confirm value
            op_para[0] = 0;
            op_para[3] = 0x10; // bridge cnt
            op_para[4] = PAR_READ_MESH_PAIR_CONFIRM;
            *PAIR_SETTING_FLAG.lock() = ePairState::PairSetMeshRxDone;
        } else if flag == ePairState::PairSetMeshRxDone {
            //send cmd to switch to new mesh
            op_para[0] = LGT_CMD_MESH_PAIR;
            op_para[3] = MeshPairState::MeshPairEffectDelay as u8;
            self.mesh_pair_state = MeshPairState::MeshPairName1;
            self.mesh_pair_start_time = 0;
            *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
        }

        Some(op_para)
    }
}

/// Periodic pairing state-machine service, called from the BLE-app callback seam.
///
/// `#[inline(never)]` is load-bearing: `app::mesh_pair_proc_hook` wraps this
/// function, and `sdk/stack_analysis.py`'s `INDIRECT_EDGES` matches the
/// `on_mesh_pair_proc` edge by this symbol name. Inlining it into the wrapper
/// would erase it from the linked ELF.
///
/// Each phase takes its own short [`MESH_PAIR`] lock. The two flash reads
/// (`pair_load_key`) and the broadcast itself run with the guard dropped.
#[inline(never)]
pub fn mesh_pair_proc() {
    let (load_key, switch_delay) = {
        let mut pair = MESH_PAIR.lock();
        pair.timer_step()
    };

    if let Some(delay_s) = switch_delay {
        switch_to_default_mesh(delay_s);
        // Keep the pre-split behaviour: the reference is refreshed once more
        // after the access code has been recomputed.
        MESH_PAIR.lock().default_mesh_time_ref = clock_time() | 1;
    }
    if load_key {
        pair_load_key();
    }

    let timed_out = {
        let pair = MESH_PAIR.lock();
        pair.timed_out()
    };
    if timed_out {
        //mesh pair time out
        pair_load_key();
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
        rf_link_light_event_callback(LGT_CMD_MESH_PAIR_TIMEOUT);
        return;
    }

    let op_para = {
        let mut pair = MESH_PAIR.lock();
        pair.interval_tick()
    };
    let Some(op_para) = op_para else {
        return;
    };

    let pkt = light_slave_tx_command(&Vec::from_slice(&op_para[..13]).unwrap(), 0xFFFF, 0, false);
    let (group_match, device_match) = rf_link_match_group_mac(&pkt);
    if group_match || device_match {
        queues::enqueue_mesh_rx(&pkt);
        if !device_match {
            queues::enqueue_mesh_tx(
                &pkt,
                0,
                pkt.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
            );
        }
    } else {
        queues::enqueue_mesh_tx(
            &pkt,
            0,
            pkt.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
        );
    }
}

/// Enable or disable mesh payload encryption.
#[cfg_attr(test, mry::mry)]
pub fn mesh_security_enable(enable: bool) {
    SECURITY_ENABLE.set(enable);
}

/// Surface mesh queue overflow, one log line per change.
///
/// The SDK cannot log — it must not reach the application's UART sink — so it
/// only counts drops. Turning a counter delta into a line here keeps a
/// sustained overload visible from RTT and the UART host without a write per
/// iteration. The "last reported" marks are owned by the sender task.
fn report_queue_drops(dropped_mesh_tx_seen: &mut u32, dropped_mesh_rx_seen: &mut u32) {
    let tx = queues::dropped_mesh_tx();
    if tx != *dropped_mesh_tx_seen {
        uprintln!(
            "mesh queue full: tx +{} (total {})",
            tx.wrapping_sub(*dropped_mesh_tx_seen),
            tx
        );
        *dropped_mesh_tx_seen = tx;
    }

    let rx = queues::dropped_mesh_rx();
    if rx != *dropped_mesh_rx_seen {
        uprintln!(
            "mesh queue full: rx +{} (total {})",
            rx.wrapping_sub(*dropped_mesh_rx_seen),
            rx
        );
        *dropped_mesh_rx_seen = rx;
    }
}

/// Process a single iteration of the mesh message sending task.
///
/// The scheduling scan is a three-state machine driven by one critical
/// section:
/// - [`SendScan::Empty`]: nothing queued; the buffer check happens before
///   `clock_time64()`, so host tests never touch MMIO. Park on
///   [`MESH_TX_WAKE`] until [`queues::enqueue_mesh_tx`] pushes a packet.
/// - [`SendScan::Wait`]: packets are queued but none is due yet; park on
///   [`MESH_TX_WAKE`] with a deadline of the earliest due tick.
/// - [`SendScan::Ready`]: the first due packet was removed and is
///   transmitted inline below.
///
/// A packet addressed to this device by this device is *report-only*: it is
/// handed straight to the UART bridge and neither transmitted nor
/// re-queued. Because that check runs here — in task context, after the pop
/// — the retransmit re-queue below can never loop a self-addressed packet.
///
/// The parked arms `return`, so the outer `loop` in [`mesh_packet_sender`]
/// re-enters and rescans instead of looping internally.
async fn mesh_packet_send_iteration(
    dropped_mesh_tx_seen: &mut u32,
    dropped_mesh_rx_seen: &mut u32,
) {
    report_queue_drops(dropped_mesh_tx_seen, dropped_mesh_rx_seen);

    let result = {
        // Reset before the scan: a push that lands after the scan leaves
        // the signal set, so the wake cannot be lost.
        MESH_TX_WAKE.reset();

        // Only the buffer scan runs with interrupts masked. `clock_time64`
        // nests one more critical section, exactly as before this queue
        // moved to module scope.
        let mut send_queue = MESH_TX_QUEUE.lock();

        if send_queue.is_empty() {
            SendScan::Empty
        } else {
            // Sample the clock exactly once for the whole scan.
            let now = clock_time64();

            match send_queue
                .iter()
                .enumerate()
                .find(|(_, elem)| now >= elem.delay)
            {
                Some((index, _)) => SendScan::Ready(send_queue.swap_remove(index)),
                None => {
                    let earliest = send_queue.iter().map(|e| e.delay).min().unwrap();
                    SendScan::Wait(earliest)
                }
            }
        }
    };

    let result = match result {
        SendScan::Empty => {
            MESH_TX_WAKE.wait().await;
            return;
        }
        SendScan::Wait(earliest) => {
            #[cfg(not(test))]
            let _ = MESH_TX_WAKE
                .wait()
                .with_deadline(Instant::from_ticks(earliest))
                .await;

            #[cfg(test)]
            {
                // Host tests have no time driver deadline; the wake signal
                // from `enqueue_mesh_tx` is the only exit.
                let _ = earliest;
                MESH_TX_WAKE.wait().await;
            }

            return;
        }
        SendScan::Ready(pkt) => pkt,
    };

    // Self-addressed packets are report-only: they exist so the host sees
    // locally generated commands, not so the radio repeats them.
    if result.pkt.ll_app().value.src == DEVICE_ADDRESS.get()
        && result.pkt.ll_app().value.dst == DEVICE_ADDRESS.get()
    {
        light_mesh_rx_cb(&result.pkt);
        return;
    }

    let mut pkt = result.pkt;

    pkt.mesh_mut().src_tx = DEVICE_ADDRESS.get();
    pkt.mesh_mut().handle1 = 0;

    // Encrypt the packet if required
    if SECURITY_ENABLE.get() {
        pkt.head_mut()._type |= BIT!(7);
        pair_enc_packet_mesh(&mut pkt);
    }

    // Pick a random start channel
    let start_chan_idx = crate::sdk::mcu::random::rand() as usize;

    // Send the packet on each channel
    for channel_index in start_chan_idx..start_chan_idx + SYS_CHN_LISTEN.len() {
        // This block needs to operate in a critical section
        critical_section::with(|_| {
            // Configure the BLE mode
            rf_set_tx_rx_off();
            rf_set_ble_access_code(PAIR_AC.get());
            rf_set_ble_crc_adv();

            rf_set_ble_channel(SYS_CHN_LISTEN[channel_index % SYS_CHN_LISTEN.len()]);
            rf_start_srx2tx(
                addr_of!(pkt) as u32,
                clock_time() + CLOCK_SYS_CLOCK_1US * 30,
            );
        });

        // Let the transmission on this channel finish before hopping.
        // Host tests have no time driver, so they skip the spacing delay.
        #[cfg(not(test))]
        Timer::after(Duration::from_micros(600)).await;
    }

    rf_set_rxmode();

    if result.send_count != 0 {
        // Random delay to avoid congestion between 0us and 8ms
        let delay = 8000 - ((crate::sdk::mcu::random::rand() % 16) * 500);

        queues::enqueue_mesh_tx(
            &result.pkt,
            clock_time64() + (delay as u64 * CLOCK_SYS_CLOCK_1US as u64),
            result.send_count - 1,
        );
    }
}

/// Mesh transmit task: owns [`MESH_TX_QUEUE`] and the drop-log bookkeeping.
#[coverage(off)]
#[embassy_executor::task]
pub async fn mesh_packet_sender() {
    let mut dropped_mesh_tx_seen: u32 = 0;
    let mut dropped_mesh_rx_seen: u32 = 0;

    #[cfg(not(test))]
    loop {
        mesh_packet_send_iteration(&mut dropped_mesh_tx_seen, &mut dropped_mesh_rx_seen).await;
    }

    #[cfg(test)]
    {
        // In test builds, just run one iteration to allow testing
        mesh_packet_send_iteration(&mut dropped_mesh_tx_seen, &mut dropped_mesh_rx_seen).await;
    }
}

/// Process a single iteration of the mesh message receiving task.
async fn mesh_packet_rcv_iteration() {
    let mut result = MESH_RX_CHANNEL.receive().await;

    rf_link_rc_data(&mut result);
}

/// Mesh receive task: hands [`MESH_RX_CHANNEL`] packets to the mesh decoder.
#[coverage(off)]
#[embassy_executor::task]
pub async fn mesh_packet_rcv() {
    #[cfg(not(test))]
    loop {
        mesh_packet_rcv_iteration().await;
    }

    #[cfg(test)]
    {
        // In test builds, just run one iteration to allow testing
        mesh_packet_rcv_iteration().await;
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::light_manager::{device_status_update, mock_device_status_update};
    use crate::sdk::ble_app::ble_ll_pair::mock_pair_save_key;
    use crate::sdk::ble_app::light_ll::mesh_management::mock_rf_link_match_group_mac;
    use crate::sdk::ble_app::light_ll::packet_processing::{
        mock_is_add_packet_buf_ready, mock_rf_link_add_tx_packet, mock_rf_link_rc_data,
    };
    use crate::sdk::ble_app::rf_drv_8266::{
        mock_rf_set_ble_access_code, mock_rf_set_ble_channel, mock_rf_set_ble_crc_adv,
        mock_rf_set_rxmode, mock_rf_set_tx_rx_off, mock_rf_start_srx2tx,
    };
    use crate::sdk::common::sync::CriticalSectionMutexGuard;
    use crate::sdk::drivers::flash::mock_flash_write_page;
    use crate::sdk::light::{ePairState, LGT_CMD_MESH_PAIR_TIMEOUT, LGT_CMD_SET_MESH_INFO};
    use crate::sdk::mcu::clock::{
        mock_clock_time, mock_clock_time64, mock_clock_time_exceed, mock_sleep_us,
    };
    use crate::sdk::mcu::random::{mock_rand, rand};
    use heapless::Vec;
    use mry::Any;
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_pac::FLD_RF_IRQ_MASK;

    use crate::main_light::{mock_light_slave_tx_command, mock_rf_link_light_event_callback};
    use crate::sdk::ble_app::flash_config::{mock_access_code, mock_pair_load_key};
    use crate::sdk::ble_app::light_ll::mesh_management::mock_mesh_node_table_init;
    use crate::uart_manager::mock_light_mesh_rx_cb;

    /// Borrow the pairing state for one field access.
    ///
    /// Every use is a single statement (`pair().field = value;` or an
    /// assertion), so the temporary guard is always released before the next
    /// call that locks [`MESH_PAIR`].
    fn pair() -> CriticalSectionMutexGuard<'static, MeshPairingState> {
        MESH_PAIR.lock()
    }

    /// Helper function to create a test packet with mesh data
    fn create_test_packet(src: u16, dst: u16, sno: [u8; 3]) -> Packet {
        Packet {
            att_cmd: PacketAttCmd {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: 0,
                    rf_len: 0,
                    l2cap_len: 0,
                    chan_id: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: PacketAttValue {
                    sno,
                    src: src.to_le_bytes(),
                    dst: dst.to_le_bytes(),
                    val: [0; 23],
                },
            },
        }
    }

    // --- TryFrom Tests ---

    /// Tests MeshPairState::try_from with all valid enum values
    #[test]
    fn test_mesh_pair_state_try_from_valid() {
        assert_eq!(
            MeshPairState::try_from(0).unwrap(),
            MeshPairState::MeshPairName1
        );
        assert_eq!(
            MeshPairState::try_from(1).unwrap(),
            MeshPairState::MeshPairName2
        );
        assert_eq!(
            MeshPairState::try_from(2).unwrap(),
            MeshPairState::MeshPairPwd1
        );
        assert_eq!(
            MeshPairState::try_from(3).unwrap(),
            MeshPairState::MeshPairPwd2
        );
        assert_eq!(
            MeshPairState::try_from(4).unwrap(),
            MeshPairState::MeshPairLtk1
        );
        assert_eq!(
            MeshPairState::try_from(5).unwrap(),
            MeshPairState::MeshPairLtk2
        );
        assert_eq!(
            MeshPairState::try_from(6).unwrap(),
            MeshPairState::MeshPairEffectDelay
        );
        assert_eq!(
            MeshPairState::try_from(7).unwrap(),
            MeshPairState::MeshPairEffect
        );
        assert_eq!(
            MeshPairState::try_from(8).unwrap(),
            MeshPairState::MeshPairDefaultMesh
        );
    }

    /// Tests MeshPairState::try_from with invalid value
    #[test]
    fn test_mesh_pair_state_try_from_invalid() {
        assert!(MeshPairState::try_from(9).is_err());
        assert!(MeshPairState::try_from(255).is_err());
    }

    // --- send_mesh_message Tests ---

    /// Tests send_mesh_message with group match
    #[test]
    #[mry::lock(light_slave_tx_command, rf_link_match_group_mac)]
    fn test_send_mesh_message_group_match() {
        // Setup
        reset_for_test();

        let test_data: Vec<u8, 13> = Vec::from_slice(&[1, 2, 3, 4, 5]).unwrap();
        let test_pkt = create_test_packet(0x1234, 0xFFFF, [10, 20, 30]);

        mock_light_slave_tx_command(Any, 0xFFFF, 3, true).returns(test_pkt);
        mock_rf_link_match_group_mac(Any).returns((true, false)); // group match, not device match

        // Execute
        let result = send_mesh_message(&test_data, 0xFFFF, 3, true);

        // Verify
        assert_eq!(result, [10, 20, 30]);
        mock_light_slave_tx_command(Any, 0xFFFF, 3, true).assert_called(1);
        mock_rf_link_match_group_mac(Any).assert_called(1);
    }

    /// Tests send_mesh_message with device match
    #[test]
    #[mry::lock(light_slave_tx_command, rf_link_match_group_mac)]
    fn test_send_mesh_message_device_match() {
        // Setup
        reset_for_test();

        let test_data: Vec<u8, 13> = Vec::from_slice(&[1, 2, 3]).unwrap();
        let test_pkt = create_test_packet(0x0001, 0x0001, [5, 6, 7]);

        mock_light_slave_tx_command(Any, 0x0001, 0, false).returns(test_pkt);
        mock_rf_link_match_group_mac(Any).returns((false, true)); // device match

        // Execute
        let result = send_mesh_message(&test_data, 0x0001, 0, false);

        // Verify - device match means no retransmission
        assert_eq!(result, [5, 6, 7]);
        mock_rf_link_match_group_mac(Any).assert_called(1);
    }

    /// Tests send_mesh_message with no match
    #[test]
    #[mry::lock(light_slave_tx_command, rf_link_match_group_mac)]
    fn test_send_mesh_message_no_match() {
        // Setup
        reset_for_test();

        let test_data: Vec<u8, 13> = Vec::from_slice(&[1, 2, 3]).unwrap();
        let test_pkt = create_test_packet(0x0002, 0x0003, [8, 9, 10]);

        mock_light_slave_tx_command(Any, 0x0003, 2, false).returns(test_pkt);
        mock_rf_link_match_group_mac(Any).returns((false, false)); // no match

        // Execute
        let result = send_mesh_message(&test_data, 0x0003, 2, false);

        // Verify
        assert_eq!(result, [8, 9, 10]);
        mock_rf_link_match_group_mac(Any).assert_called(1);
    }

    // --- mesh_pair_init Tests ---

    /// Tests mesh_pair_init sets proper values
    #[test]
    fn test_mesh_pair_init() {
        // Setup
        reset_for_test();
        MESH_PAIR_ENABLE.set(false);

        // Execute
        mesh_pair_init();

        // Verify
        assert!(MESH_PAIR_ENABLE.get());
        assert_eq!(pair().mesh_pair_cmd_interval, MESH_PAIR_CMD_INTERVAL);
        assert_eq!(pair().mesh_pair_timeout, MESH_PAIR_TIMEOUT);
    }

    // --- mesh_pair_proc_effect Tests ---

    /// Tests mesh_pair_proc_effect with effect_new_mesh set
    #[test]
    #[mry::lock(
        sleep_us,
        pair_save_key,
        rf_set_ble_access_code,
        rf_link_light_event_callback,
        is_add_packet_buf_ready,
        rf_link_add_tx_packet
    )]
    fn test_mesh_pair_proc_effect_with_effect_new_mesh() {
        // Setup
        reset_for_test();
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);
        MESH_PAIR_ENABLE.set(false);
        PAIR_AC.set(0x12345678);
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x0001);
        pair().effect_new_mesh = 1;

        mock_pair_save_key().returns(());
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_link_light_event_callback(LGT_CMD_SET_MESH_INFO).returns(());
        mock_sleep_us(1000).returns(());
        FakeMmio::set_reg16(0xf20, 0);
        mock_is_add_packet_buf_ready().returns(true);
        mock_rf_link_add_tx_packet(Any).returns(Ok(()));

        // Execute
        mesh_pair_proc_effect();

        // Verify
        assert_eq!(pair().effect_new_mesh, 0);
        assert_eq!(pair().effect_new_mesh_delay_time, 0);
        mock_pair_save_key().assert_called(1);
        assert_eq!(
            FakeMmio::reg16(0xf20),
            FLD_RF_IRQ_MASK::IRQ_RX.bits(),
            "the pending RX interrupt must be acknowledged during the mesh switch"
        );
    }

    /// Tests mesh_pair_proc_effect with delay time exceeded
    #[test]
    #[mry::lock(
        clock_time,
        clock_time_exceed,
        pair_load_key,
        pair_save_key,
        is_add_packet_buf_ready,
        rf_link_add_tx_packet,
        rf_set_ble_access_code,
        rf_link_light_event_callback
    )]
    fn test_mesh_pair_proc_effect_with_delay_exceeded() {
        // Setup
        reset_for_test();
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);
        MESH_PAIR_ENABLE.set(false);
        PAIR_AC.set(0x12345678);
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x0001);

        pair().effect_new_mesh = 0;
        pair().effect_new_mesh_delay_time = 1000;
        pair().mesh_pair_cmd_interval = 500;

        mock_clock_time().returns(0);
        mock_clock_time_exceed(Any, Any).returns(true); // For sleep_us
        mock_pair_load_key().returns(());
        mock_pair_save_key().returns(());
        mock_is_add_packet_buf_ready().returns(true);
        mock_rf_link_add_tx_packet(Any).returns(Ok(()));
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_link_light_event_callback(Any).returns(());
        FakeMmio::set_reg16(0xf20, 0);

        // Execute
        mesh_pair_proc_effect();

        // Verify
        assert_eq!(pair().effect_new_mesh_delay_time, 0);
        mock_clock_time_exceed(Any, Any).assert_called(2);
        assert_eq!(
            FakeMmio::reg16(0xf20),
            FLD_RF_IRQ_MASK::IRQ_RX.bits(),
            "the pending RX interrupt must be acknowledged during the mesh switch"
        );
    }

    /// Tests mesh_pair_proc_effect with no effect needed
    #[test]
    #[mry::lock(clock_time_exceed)]
    fn test_mesh_pair_proc_effect_no_effect() {
        // Setup
        reset_for_test();
        pair().effect_new_mesh = 0;
        pair().effect_new_mesh_delay_time = 1000;
        pair().mesh_pair_cmd_interval = 500;

        mock_clock_time_exceed(1000, 500 * 1000).returns(false);

        // Execute
        mesh_pair_proc_effect();

        // Verify - nothing should change
        assert_eq!(pair().effect_new_mesh_delay_time, 1000);
        mock_clock_time_exceed(1000, 500 * 1000).assert_called(1);
    }

    // --- mesh_device_address_validation_completed Tests ---

    /// Tests mesh_device_address_validation_completed with mesh pair enabled
    #[test]
    #[mry::lock(flash_write_page)]
    fn test_mesh_device_address_validation_completed_pair_enabled() {
        // Setup
        reset_for_test();
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(true);
        MESH_PAIR_ENABLE.set(true);
        FLASH_CONFIGURATION_INDEX.set(10);

        mock_flash_write_page((FLASH_ADR_PAIRING as i32 + 10 + 1) as u32, 1, Any).returns(());

        // Execute
        mesh_device_address_validation_completed();

        // Verify
        assert!(!MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get());
        mock_flash_write_page((FLASH_ADR_PAIRING as i32 + 10 + 1) as u32, 1, Any).assert_called(1);
    }

    /// Tests mesh_device_address_validation_completed without mesh pair
    #[test]
    fn test_mesh_device_address_validation_completed_pair_disabled() {
        // Setup
        reset_for_test();
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(true);
        MESH_PAIR_ENABLE.set(false);

        // Execute
        mesh_device_address_validation_completed();

        // Verify
        assert!(!MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get());
    }

    // --- mesh_pair_cb Tests ---

    /// Tests mesh_pair_cb with MeshPairName1
    #[test]
    #[mry::lock(clock_time)]
    fn test_mesh_pair_cb_name1() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]).unwrap();
        mock_clock_time().returns(1000);

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(pair().mesh_pair_start_time, 1001);
        assert_eq!(&pair().new_mesh_name[0..8], &[1, 2, 3, 4, 5, 6, 7, 8]);
    }

    /// Tests mesh_pair_cb with MeshPairName2
    #[test]
    fn test_mesh_pair_cb_name2() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[1, 11, 12, 13, 14, 15, 16, 17, 18]).unwrap();

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(
            &pair().new_mesh_name[8..16],
            &[11, 12, 13, 14, 15, 16, 17, 18]
        );
    }

    /// Tests mesh_pair_cb with MeshPairPwd1
    #[test]
    fn test_mesh_pair_cb_pwd1() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[2, 21, 22, 23, 24, 25, 26, 27, 28]).unwrap();

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(
            &pair().new_mesh_pwd[0..8],
            &[21, 22, 23, 24, 25, 26, 27, 28]
        );
    }

    /// Tests mesh_pair_cb with MeshPairPwd2
    #[test]
    fn test_mesh_pair_cb_pwd2() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[3, 31, 32, 33, 34, 35, 36, 37, 38]).unwrap();

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(
            &pair().new_mesh_pwd[8..16],
            &[31, 32, 33, 34, 35, 36, 37, 38]
        );
    }

    /// Tests mesh_pair_cb with MeshPairLtk1
    #[test]
    fn test_mesh_pair_cb_ltk1() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[4, 41, 42, 43, 44, 45, 46, 47, 48]).unwrap();

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(
            &pair().new_mesh_ltk[0..8],
            &[41, 42, 43, 44, 45, 46, 47, 48]
        );
    }

    /// Tests mesh_pair_cb with MeshPairLtk2
    #[test]
    fn test_mesh_pair_cb_ltk2() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[5, 51, 52, 53, 54, 55, 56, 57, 58]).unwrap();

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(
            &pair().new_mesh_ltk[8..16],
            &[51, 52, 53, 54, 55, 56, 57, 58]
        );
    }

    /// Tests mesh_pair_cb with MeshPairEffectDelay
    #[test]
    #[mry::lock(clock_time)]
    fn test_mesh_pair_cb_effect_delay() {
        // Setup
        reset_for_test();
        pair().default_mesh_time_ref = 500;
        pair().mesh_pair_cmd_interval = 250;
        let params: Vec<u8, 10> = Vec::from_slice(&[6, 0, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        mock_clock_time().returns(2000);

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(pair().effect_new_mesh_delay_time, 2001);
        assert_eq!(pair().default_mesh_time, 500); // 250 * 2
    }

    /// Tests mesh_pair_cb with MeshPairEffect
    #[test]
    fn test_mesh_pair_cb_effect() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[7, 0, 0, 0, 0, 0, 0, 0, 0]).unwrap();

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(pair().effect_new_mesh, 1);
    }

    /// Tests mesh_pair_cb with MeshPairDefaultMesh
    #[test]
    #[mry::lock(clock_time)]
    fn test_mesh_pair_cb_default_mesh() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[8, 5, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        mock_clock_time().returns(3000);

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(pair().default_mesh_effect_delay_ref, 3001);
        assert_eq!(pair().default_mesh_time, 5000); // 5 * 1000
    }

    /// Tests mesh_pair_cb with invalid command
    #[test]
    fn test_mesh_pair_cb_invalid_command() {
        // Setup
        reset_for_test();
        let params: Vec<u8, 10> = Vec::from_slice(&[99, 0, 0, 0, 0, 0, 0, 0, 0]).unwrap();

        // Execute - should return early without panic
        mesh_pair_cb(&params);

        // Verify - no changes should occur
        assert_eq!(pair().mesh_pair_start_time, 0);
    }

    /// Tests mesh_pair_cb updates default_mesh_time_ref when it's set
    #[test]
    #[mry::lock(clock_time)]
    fn test_mesh_pair_cb_updates_default_mesh_time_ref() {
        // Setup
        reset_for_test();
        pair().default_mesh_time_ref = 100;
        let params: Vec<u8, 10> = Vec::from_slice(&[1, 1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        mock_clock_time().returns(5000);

        // Execute
        mesh_pair_cb(&params);

        // Verify
        assert_eq!(pair().default_mesh_time_ref, 5001);
    }

    // --- mesh_cmd_notify Tests ---

    /// Tests mesh_cmd_notify when BLE is not connected
    #[test]
    fn test_mesh_cmd_notify_ble_not_connected() {
        // Setup
        reset_for_test();
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(false);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x1234);

        let params: Vec<u8, 10> = Vec::from_slice(&[1, 2, 3]).unwrap();

        // Execute
        let result = mesh_cmd_notify(0x10, &params, 0x1234);

        // Verify
        assert_eq!(result, -1);
    }

    /// Tests mesh_cmd_notify when pair login is not ok
    #[test]
    fn test_mesh_cmd_notify_pair_login_not_ok() {
        // Setup
        reset_for_test();
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(false);
        DEVICE_ADDRESS.set(0x1234);

        let params: Vec<u8, 10> = Vec::from_slice(&[1, 2, 3]).unwrap();

        // Execute
        let result = mesh_cmd_notify(0x10, &params, 0x1234);

        // Verify
        assert_eq!(result, -1);
    }

    /// Tests mesh_cmd_notify with params too long
    /// Note: Vec<u8, 10> can't actually exceed 10 elements, but this documents the behavior
    #[test]
    #[mry::lock(is_add_packet_buf_ready)]
    fn test_mesh_cmd_notify_params_too_long() {
        // Setup
        reset_for_test();
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x1234);

        let params: Vec<u8, 10> = Vec::from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]).unwrap();
        // The function checks p.len() > 10, which is impossible with Vec<u8, 10>
        // This test documents that the capacity constraint prevents overlong params

        mock_is_add_packet_buf_ready().returns(false);

        // Execute
        let _result = mesh_cmd_notify(0x10, &params, 0x1234);

        // With Vec<u8, 10>, the length can never exceed 10, so the early return is unreachable
    }

    /// Tests mesh_cmd_notify with successful notification
    #[test]
    #[mry::lock(is_add_packet_buf_ready, rf_link_add_tx_packet)]
    fn test_mesh_cmd_notify_success() {
        // Setup
        reset_for_test();
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x1234);

        let params: Vec<u8, 10> = Vec::from_slice(&[1, 2, 3]).unwrap();

        mock_is_add_packet_buf_ready().returns(true);
        mock_rf_link_add_tx_packet(Any).returns(Ok(()));

        // Execute
        let result = mesh_cmd_notify(0x10, &params, 0x1234);

        // Verify
        assert_eq!(result, 0);
        mock_is_add_packet_buf_ready().assert_called(1);
        mock_rf_link_add_tx_packet(Any).assert_called(1);
    }

    /// Tests mesh_cmd_notify when buffer is not ready
    #[test]
    #[mry::lock(is_add_packet_buf_ready)]
    fn test_mesh_cmd_notify_buffer_not_ready() {
        // Setup
        reset_for_test();
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x1234);

        let params: Vec<u8, 10> = Vec::from_slice(&[1, 2, 3]).unwrap();

        mock_is_add_packet_buf_ready().returns(false);

        // Execute
        let result = mesh_cmd_notify(0x10, &params, 0x1234);

        // Verify
        assert_eq!(result, -1);
        mock_is_add_packet_buf_ready().assert_called(1);
    }

    // --- mesh_pair_complete_notify Tests ---

    /// Tests mesh_pair_complete_notify
    #[test]
    #[mry::lock(is_add_packet_buf_ready, rf_link_add_tx_packet)]
    fn test_mesh_pair_complete_notify() {
        // Setup
        reset_for_test();
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x5678);

        mock_is_add_packet_buf_ready().returns(true);
        mock_rf_link_add_tx_packet(Any).returns(Ok(()));

        // Execute
        let result = mesh_pair_complete_notify();

        // Verify
        assert_eq!(result, 0);
        mock_is_add_packet_buf_ready().assert_called(1);
        mock_rf_link_add_tx_packet(Any).assert_called(1);
    }

    // --- safe_effect_new_mesh_finish Tests ---

    /// Tests safe_effect_new_mesh_finish clears state
    #[test]
    fn test_safe_effect_new_mesh_finish() {
        // Setup
        reset_for_test();
        pair().new_mesh_name = [1; 16];
        pair().new_mesh_pwd = [2; 16];
        pair().new_mesh_ltk = [3; 16];
        pair().mesh_pair_start_time = 1000;
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetting;

        // Execute
        safe_effect_new_mesh_finish();

        // Verify
        assert_eq!(pair().new_mesh_name, [0; 16]);
        assert_eq!(pair().new_mesh_pwd, [0; 16]);
        assert_eq!(pair().new_mesh_ltk, [0; 16]);
        assert_eq!(pair().mesh_pair_start_time, 0);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), ePairState::PairSetted);
    }

    // --- save_effect_new_mesh Tests ---

    /// Tests save_effect_new_mesh with default mesh time ref set
    #[test]
    #[mry::lock(
        is_add_packet_buf_ready,
        rf_link_add_tx_packet,
        sleep_us,
        pair_load_key,
        mesh_node_table_init,
        device_status_update
    )]
    fn test_save_effect_new_mesh_default_mesh_active() {
        // Setup
        reset_for_test();
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x0001);
        pair().default_mesh_time_ref = 1000; // Set to non-zero

        mock_is_add_packet_buf_ready().returns(true);
        mock_rf_link_add_tx_packet(Any).returns(Ok(()));
        mock_sleep_us(1000).returns(());
        mock_pair_load_key().returns(());
        mock_mesh_node_table_init().returns(());
        mock_device_status_update().returns(());

        // Execute
        save_effect_new_mesh();

        // Verify
        assert_eq!(pair().default_mesh_time_ref, 0);
        assert_eq!(pair().mesh_pair_start_time, 0);
        mock_pair_load_key().assert_called(1);
        mock_mesh_node_table_init().assert_called(1);
    }

    /// Tests save_effect_new_mesh with validation pending
    #[test]
    #[mry::lock(
        is_add_packet_buf_ready,
        rf_link_add_tx_packet,
        sleep_us,
        pair_load_key,
        mesh_node_table_init,
        device_status_update
    )]
    fn test_save_effect_new_mesh_validation_pending() {
        // Setup
        reset_for_test();
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(true);
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x0001);
        pair().default_mesh_time_ref = 0;

        mock_is_add_packet_buf_ready().returns(true);
        mock_rf_link_add_tx_packet(Any).returns(Ok(()));
        mock_sleep_us(1000).returns(());
        mock_pair_load_key().returns(());
        mock_mesh_node_table_init().returns(());
        mock_device_status_update().returns(());

        // Execute
        save_effect_new_mesh();

        // Verify
        mock_pair_load_key().assert_called(1);
        mock_mesh_node_table_init().assert_called(1);
    }

    /// Tests save_effect_new_mesh normal save path
    #[test]
    #[mry::lock(
        is_add_packet_buf_ready,
        rf_link_add_tx_packet,
        pair_save_key,
        rf_set_ble_access_code,
        rf_link_light_event_callback,
        sleep_us
    )]
    fn test_save_effect_new_mesh_normal_save() {
        // Setup
        reset_for_test();
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x0001);
        PAIR_AC.set(0xABCDEF12);

        pair().default_mesh_time_ref = 0;
        pair().effect_new_mesh = 0;
        pair().new_mesh_name = [1; 16];
        pair().new_mesh_pwd = [2; 16];
        pair().new_mesh_ltk = [3; 16];

        mock_is_add_packet_buf_ready().returns(true);
        mock_rf_link_add_tx_packet(Any).returns(Ok(()));
        mock_pair_save_key().returns(());
        mock_rf_set_ble_access_code(0xABCDEF12).returns(());
        mock_rf_link_light_event_callback(LGT_CMD_SET_MESH_INFO).returns(());
        mock_sleep_us(1000).returns(());
        FakeMmio::set_reg16(0xf20, 0);

        // Execute
        save_effect_new_mesh();

        // Verify
        assert_eq!(pair().mesh_pair_start_time, 0);
        mock_pair_save_key().assert_called(1);
        mock_rf_set_ble_access_code(0xABCDEF12).assert_called(1);
        mock_rf_link_light_event_callback(LGT_CMD_SET_MESH_INFO).assert_called(1);
        assert_eq!(
            FakeMmio::reg16(0xf20),
            FLD_RF_IRQ_MASK::IRQ_RX.bits(),
            "the pending RX interrupt must be acknowledged during the mesh switch"
        );
    }

    // --- switch_to_default_mesh Tests ---

    /// Tests switch_to_default_mesh
    #[test]
    #[mry::lock(clock_time, access_code)]
    fn test_switch_to_default_mesh() {
        // Setup
        reset_for_test();
        let test_name = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
        let test_pwd = [
            17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
        ];
        let test_ltk = [
            33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
        ];

        *PAIR_CONFIG_MESH_NAME.lock() = test_name;
        *PAIR_CONFIG_MESH_PWD.lock() = test_pwd;
        *PAIR_CONFIG_MESH_LTK.lock() = test_ltk;

        mock_clock_time().returns(5000);
        mock_access_code(Any, Any).returns(0xDEADBEEF);

        // Execute
        switch_to_default_mesh(10);

        // Verify
        assert_eq!(pair().default_mesh_time_ref, 5001);
        assert_eq!(pair().default_mesh_time, 10000);
        assert_eq!(PAIR_AC.get(), 0xDEADBEEF);
        assert_eq!(PAIR_STATE.lock().pair_ltk, test_ltk);
        mock_access_code(Any, Any).assert_called(1);
    }

    // --- mesh_pair_proc Tests ---

    /// Tests mesh_pair_proc with timeout
    #[test]
    #[mry::lock(clock_time_exceed, pair_load_key, rf_link_light_event_callback)]
    fn test_mesh_pair_proc_timeout() {
        // Setup
        reset_for_test();
        pair().mesh_pair_start_time = 1000;
        pair().mesh_pair_timeout = 10;
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetting;

        mock_clock_time_exceed(1000, 10 * 1000 * 1000).returns(true);
        mock_pair_load_key().returns(());
        mock_rf_link_light_event_callback(LGT_CMD_MESH_PAIR_TIMEOUT).returns(());

        // Execute
        mesh_pair_proc();

        // Verify
        assert_eq!(*PAIR_SETTING_FLAG.lock(), ePairState::PairSetted);
        mock_pair_load_key().assert_called(1);
        mock_rf_link_light_event_callback(LGT_CMD_MESH_PAIR_TIMEOUT).assert_called(1);
    }

    /// Tests mesh_pair_proc default mesh effect delay
    #[test]
    #[mry::lock(clock_time_exceed, pair_load_key)]
    fn test_mesh_pair_proc_default_mesh_effect_delay() {
        // Setup
        reset_for_test();
        pair().default_mesh_effect_delay_ref = 1000;
        pair().default_mesh_time = 0;
        pair().mesh_pair_start_time = 0;
        pair().default_mesh_time_ref = 0; // Ensure we don't skip the condition

        mock_clock_time_exceed(1000, MESH_PAIR_CMD_INTERVAL * 1000).returns(true);
        mock_pair_load_key().returns(());

        // Execute
        mesh_pair_proc();

        // Verify
        assert_eq!(pair().default_mesh_effect_delay_ref, 0);
        assert_eq!(pair().default_mesh_time_ref, 0);
        mock_pair_load_key().assert_called(1);
    }

    /// Tests mesh_pair_proc default mesh effect delay with switch
    #[test]
    #[mry::lock(clock_time_exceed, clock_time, access_code)]
    fn test_mesh_pair_proc_default_mesh_effect_delay_with_switch() {
        // Setup
        reset_for_test();
        pair().default_mesh_effect_delay_ref = 1000;
        pair().default_mesh_time = 5000;
        pair().mesh_pair_start_time = 0;

        mock_clock_time_exceed(1000, MESH_PAIR_CMD_INTERVAL * 1000).returns(true);
        mock_clock_time().returns(2000);
        mock_access_code(Any, Any).returns(0x11223344);

        // Execute
        mesh_pair_proc();

        // Verify
        assert_eq!(pair().default_mesh_effect_delay_ref, 0);
        assert_eq!(pair().default_mesh_time_ref, 2001);
        mock_access_code(Any, Any).assert_called(1);
    }

    /// Tests mesh_pair_proc default mesh timeout
    #[test]
    #[mry::lock(clock_time_exceed, pair_load_key)]
    fn test_mesh_pair_proc_default_mesh_timeout() {
        // Setup
        reset_for_test();
        pair().default_mesh_effect_delay_ref = 0;
        pair().default_mesh_time_ref = 1000;
        pair().default_mesh_time = 5000;
        pair().mesh_pair_start_time = 0;

        mock_clock_time_exceed(Any, Any).returns_with(|ref_time: u32, span: u32| -> bool {
            ref_time == 1000 && span == 5000 * 1000
        });
        mock_pair_load_key().returns(());

        // Execute
        mesh_pair_proc();

        // Verify
        assert_eq!(pair().default_mesh_time_ref, 0);
        mock_pair_load_key().assert_called(1);
    }

    /// Tests mesh_pair_proc default mesh timeout with 255s delay
    #[test]
    #[mry::lock(clock_time_exceed, clock_time)]
    fn test_mesh_pair_proc_default_mesh_timeout_255s() {
        // Setup
        reset_for_test();
        pair().default_mesh_effect_delay_ref = 0;
        pair().default_mesh_time_ref = 1000;
        pair().default_mesh_time = 255000;
        pair().mesh_pair_start_time = 0;

        mock_clock_time_exceed(Any, Any).returns_with(|ref_time: u32, span: u32| -> bool {
            ref_time == 1000 && span == 255000 * 1000
        });
        mock_clock_time().returns(3000);

        // Execute
        mesh_pair_proc();

        // Verify - should reset ref time, not clear it
        assert_eq!(pair().default_mesh_time_ref, 3001);
    }

    /// Tests mesh_pair_proc state machine - Name1
    #[test]
    #[mry::lock(
        clock_time,
        clock_time_exceed,
        light_slave_tx_command,
        rf_link_match_group_mac
    )]
    fn test_mesh_pair_proc_state_name1() {
        // Setup
        reset_for_test();
        pair().mesh_pair_state = MeshPairState::MeshPairName1;
        pair().mesh_pair_time = 1000;
        pair().mesh_pair_cmd_interval = 500;

        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetMeshTxStart;
        PAIR_STATE.lock().pair_nn = [1; 16];

        mock_clock_time_exceed(Any, Any)
            .returns_with(|ref_time: u32, _span: u32| -> bool { ref_time == 1000 });
        mock_clock_time().returns(2000);
        let test_pkt = create_test_packet(0x0001, 0xFFFF, [1, 2, 3]);
        mock_light_slave_tx_command(Any, Any, Any, Any).returns(test_pkt);
        mock_rf_link_match_group_mac(Any).returns((false, false));

        // Execute
        mesh_pair_proc();

        // Verify - should advance to Name2
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairName2);
    }

    /// Tests mesh_pair_proc state machine - transitions through all states
    #[test]
    #[mry::lock(
        clock_time,
        clock_time_exceed,
        light_slave_tx_command,
        rf_link_match_group_mac
    )]
    fn test_mesh_pair_proc_state_transitions() {
        // Setup
        reset_for_test();
        pair().mesh_pair_time = 1000;
        pair().mesh_pair_cmd_interval = 500;

        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetMeshTxStart;
        PAIR_STATE.lock().pair_nn = [1; 16];
        PAIR_STATE.lock().pair_pass = [2; 16];
        PAIR_STATE.lock().pair_ltk_mesh = [3; 16];

        mock_clock_time_exceed(Any, Any).returns(true); // For other clock_time_exceed calls
        mock_clock_time().returns(2000);
        let test_pkt = create_test_packet(0x0001, 0xFFFF, [1, 2, 3]);
        mock_light_slave_tx_command(Any, Any, Any, Any).returns(test_pkt);
        mock_rf_link_match_group_mac(Any).returns((false, false));

        // Test Name1 -> Name2
        pair().mesh_pair_state = MeshPairState::MeshPairName1;
        mesh_pair_proc();
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairName2);

        // Test Name2 -> Pwd1
        pair().mesh_pair_state = MeshPairState::MeshPairName2;
        mesh_pair_proc();
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairPwd1);

        // Test Pwd1 -> Pwd2
        pair().mesh_pair_state = MeshPairState::MeshPairPwd1;
        mesh_pair_proc();
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairPwd2);

        // Test Pwd2 -> Ltk1
        pair().mesh_pair_state = MeshPairState::MeshPairPwd2;
        mesh_pair_proc();
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairLtk1);

        // Test Ltk1 -> Ltk2
        pair().mesh_pair_state = MeshPairState::MeshPairLtk1;
        mesh_pair_proc();
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairLtk2);

        // Test Ltk2 -> Name1 and state change to TxDone
        pair().mesh_pair_state = MeshPairState::MeshPairLtk2;
        mesh_pair_proc();
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairName1);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), ePairState::PairSetMeshTxDone);
    }

    /// Tests mesh_pair_proc PairSetMeshTxDone state
    #[test]
    #[mry::lock(
        clock_time,
        clock_time_exceed,
        light_slave_tx_command,
        rf_link_match_group_mac
    )]
    fn test_mesh_pair_proc_tx_done_state() {
        // Setup
        reset_for_test();
        pair().mesh_pair_time = 1000;
        pair().mesh_pair_cmd_interval = 500;

        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetMeshTxDone;

        mock_clock_time_exceed(1000, 500 * 1000).returns(true);
        mock_clock_time().returns(2000);
        let test_pkt = create_test_packet(0x0001, 0xFFFF, [1, 2, 3]);
        mock_light_slave_tx_command(Any, Any, Any, Any).returns(test_pkt);
        mock_rf_link_match_group_mac(Any).returns((false, false));

        // Execute
        mesh_pair_proc();

        // Verify - should advance to RxDone state
        assert_eq!(*PAIR_SETTING_FLAG.lock(), ePairState::PairSetMeshRxDone);
    }

    /// Tests mesh_pair_proc PairSetMeshRxDone state
    #[test]
    #[mry::lock(
        clock_time,
        clock_time_exceed,
        light_slave_tx_command,
        rf_link_match_group_mac
    )]
    fn test_mesh_pair_proc_rx_done_state() {
        // Setup
        reset_for_test();
        pair().mesh_pair_time = 1000;
        pair().mesh_pair_cmd_interval = 500;
        pair().mesh_pair_timeout = 60; // Timeout in seconds - won't overflow when multiplied
        pair().mesh_pair_start_time = 5000;

        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetMeshRxDone;

        // Mock specific clock_time_exceed calls
        // Don't trigger timeout: clock_time_exceed(5000, 60 * 1000 * 1000)
        mock_clock_time_exceed(5000, 60 * 1000 * 1000).returns(false);
        // Trigger interval check: clock_time_exceed(1000, 500 * 1000)
        mock_clock_time_exceed(1000, 500 * 1000).returns(true);
        mock_clock_time().returns(6000);
        let test_pkt = create_test_packet(0x0001, 0xFFFF, [1, 2, 3]);
        mock_light_slave_tx_command(Any, Any, Any, Any).returns(test_pkt);
        mock_rf_link_match_group_mac(Any).returns((false, false));

        // Execute
        mesh_pair_proc();

        // Verify - should reset state and clear start time
        assert_eq!(pair().mesh_pair_state, MeshPairState::MeshPairName1);
        // mesh_pair_start_time should be cleared when transitioning from PairSetMeshRxDone
        assert_eq!(pair().mesh_pair_start_time, 0);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), ePairState::PairSetted);
    }

    // --- mesh_security_enable Tests ---

    /// Tests mesh_security_enable with true
    #[test]
    fn test_mesh_security_enable_true() {
        // Setup
        reset_for_test();
        SECURITY_ENABLE.set(false);

        // Execute
        mesh_security_enable(true);

        // Verify
        assert!(SECURITY_ENABLE.get());
    }

    /// Tests mesh_security_enable with false
    #[test]
    fn test_mesh_security_enable_false() {
        // Setup
        reset_for_test();
        SECURITY_ENABLE.set(true);

        // Execute
        mesh_security_enable(false);

        // Verify
        assert!(!SECURITY_ENABLE.get());
    }

    // --- queues::enqueue_mesh_tx Tests ---

    /// A self-addressed packet is report-only: the TX task pops it, hands it to
    /// the UART bridge and never transmits or re-queues it.
    #[test]
    #[mry::lock(
        light_mesh_rx_cb,
        clock_time64,
        rf_set_tx_rx_off,
        rf_set_ble_access_code,
        rf_set_ble_crc_adv,
        rf_set_ble_channel,
        rf_start_srx2tx,
        rf_set_rxmode
    )]
    fn test_send_mesh_msg_iteration_self_addressed_is_report_only() {
        use futures::executor::block_on;

        // Setup
        reset_for_test();
        DEVICE_ADDRESS.set(0x1234);
        SECURITY_ENABLE.set(false);
        let mut dropped_tx_seen = 0;
        let mut dropped_rx_seen = 0;
        let test_pkt = create_test_packet(0x1234, 0x1234, [1, 2, 3]);

        mock_light_mesh_rx_cb(Any).returns(());
        mock_clock_time64().returns(0);
        mock_rf_set_tx_rx_off().returns(());
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_rf_set_ble_channel(Any).returns(());
        mock_rf_start_srx2tx(Any, Any).returns(());
        mock_rf_set_rxmode().returns(());

        // Execute
        assert!(queues::enqueue_mesh_tx(&test_pkt, 0, 0));
        block_on(mesh_packet_send_iteration(
            &mut dropped_tx_seen,
            &mut dropped_rx_seen,
        ));

        // Verify - reported over UART, popped, not transmitted
        mock_light_mesh_rx_cb(Any).assert_called(1);
        assert_eq!(MESH_TX_QUEUE.lock().len(), 0);
        mock_rf_set_tx_rx_off().assert_called(0);
    }

    /// Tests queues::enqueue_mesh_tx normal add
    #[test]
    fn test_enqueue_mesh_tx_normal() {
        // Setup
        reset_for_test();
        DEVICE_ADDRESS.set(0x1234);
        let test_pkt = create_test_packet(0x5678, 0xABCD, [1, 2, 3]);

        // Execute
        assert!(queues::enqueue_mesh_tx(&test_pkt, 1000, 3));

        // Verify
        let send_queue = MESH_TX_QUEUE.lock();
        assert_eq!(send_queue.len(), 1);
        assert_eq!(send_queue[0].delay, 1000);
        assert_eq!(send_queue[0].send_count, 3);
    }

    /// Tests queues::enqueue_mesh_tx buffer full
    #[test]
    fn test_enqueue_mesh_tx_buffer_full() {
        // Setup
        reset_for_test();
        DEVICE_ADDRESS.set(0x1234);
        let test_pkt = create_test_packet(0x5678, 0xABCD, [1, 2, 3]);

        // Fill the buffer (capacity is 8)
        for i in 0..8 {
            let pkt = create_test_packet(i as u16, 0xFFFF, [i as u8, 0, 0]);
            assert!(queues::enqueue_mesh_tx(&pkt, i as u64, 0));
        }

        // Execute - try to add one more
        assert!(!queues::enqueue_mesh_tx(&test_pkt, 1000, 3));

        // Verify - buffer should still be at max capacity
        assert_eq!(MESH_TX_QUEUE.lock().len(), 8);
    }

    // --- queues::enqueue_mesh_rx Tests ---

    /// Tests queues::enqueue_mesh_rx normal add
    #[test]
    fn test_enqueue_mesh_rx_normal() {
        // Setup
        reset_for_test();
        let test_pkt = create_test_packet(0x1234, 0x5678, [1, 2, 3]);

        // Execute
        assert!(queues::enqueue_mesh_rx(&test_pkt));

        // Verify
        assert_eq!(MESH_RX_CHANNEL.len(), 1);
    }

    /// Tests queues::enqueue_mesh_rx buffer full
    #[test]
    fn test_enqueue_mesh_rx_buffer_full() {
        // Setup
        reset_for_test();

        // Fill the buffer (capacity is 8)
        for i in 0..8 {
            let pkt = create_test_packet(i as u16, 0xFFFF, [i as u8, 0, 0]);
            assert!(queues::enqueue_mesh_rx(&pkt));
        }

        // Execute - try to add one more
        let test_pkt = create_test_packet(0x9999, 0x8888, [9, 9, 9]);
        assert!(!queues::enqueue_mesh_rx(&test_pkt));

        // Verify - buffer should still be at max capacity
        assert_eq!(MESH_RX_CHANNEL.len(), 8);
    }

    /// Tests send_mesh_msg_iteration parks on the wake and wakes on a push
    #[test]
    fn test_send_mesh_msg_iteration_no_packets() {
        use core::future::Future;
        use core::task::{Context, Poll};
        use futures::task::noop_waker;

        // Setup
        reset_for_test();
        let mut dropped_tx_seen = 0;
        let mut dropped_rx_seen = 0;

        // The TX queue is empty by default, so the iteration parks on the wake
        let future = mesh_packet_send_iteration(&mut dropped_tx_seen, &mut dropped_rx_seen);
        futures::pin_mut!(future);

        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);

        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Pending));

        // Enqueue a ready packet; `enqueue_mesh_tx` signals the wake
        let test_pkt = create_test_packet(0x1234, 0x5678, [1, 2, 3]);
        assert!(queues::enqueue_mesh_tx(&test_pkt, 0, 0));
        assert_eq!(MESH_TX_QUEUE.lock().len(), 1);

        // The wake signal releases the parked iteration without sending; the
        // outer task loop re-enters and picks the packet up.
        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Ready(())));

        // Verify - the packet is still queued for the next iteration
        assert_eq!(MESH_TX_QUEUE.lock().len(), 1);
    }

    /// Tests send_mesh_msg_iteration with ready packet
    #[test]
    #[mry::lock(
        rand,
        clock_time,
        rf_set_tx_rx_off,
        rf_set_ble_access_code,
        rf_set_ble_crc_adv,
        rf_set_ble_channel,
        rf_start_srx2tx,
        rf_set_rxmode,
        clock_time64
    )]
    fn test_send_mesh_msg_iteration_with_ready_packet() {
        use futures::executor::block_on;

        // Setup
        reset_for_test();
        let mut dropped_tx_seen = 0;
        let mut dropped_rx_seen = 0;
        let test_pkt = create_test_packet(0x1234, 0x5678, [1, 2, 3]);

        // Ensure security is disabled to avoid encryption overflow
        SECURITY_ENABLE.set(false);

        // Add a packet that's ready to send (delay is in the past)
        assert!(queues::enqueue_mesh_tx(&test_pkt, 0, 0)); // delay=0 means ready to send immediately

        assert_eq!(MESH_TX_QUEUE.lock().len(), 1);

        // Mock the required functions. `rand()` used to be spelled
        // `system_tick ^ rnd_number`; the same 0x03E8 ^ 0x5555 value keeps the
        // channel selection and congestion delay identical.
        mock_rand().returns(1000 ^ 0x5555);
        mock_clock_time().returns(1000);
        mock_clock_time64().returns(1); // Return time > 0 so packet is ready
        mock_rf_set_tx_rx_off().returns(());
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_rf_set_ble_channel(Any).returns(());
        mock_rf_start_srx2tx(Any, Any).returns(());
        mock_rf_set_rxmode().returns(());

        // Execute
        block_on(mesh_packet_send_iteration(
            &mut dropped_tx_seen,
            &mut dropped_rx_seen,
        ));

        // Verify - packet should be processed and removed from buffer
        assert_eq!(MESH_TX_QUEUE.lock().len(), 0);
        mock_rf_set_tx_rx_off().assert_called(SYS_CHN_LISTEN.len());
        // The random start channel is drawn once, and the SRX2TX deadline is
        // sampled once per channel from `clock_time()`.
        mock_rand().assert_called(1);
        mock_clock_time().assert_called(SYS_CHN_LISTEN.len());
    }

    /// The congestion backoff re-queue must keep the pre-migration 0..8 ms
    /// spread: `8000 - (rand() % 16) * 500` microseconds past `clock_time64()`.
    ///
    /// `rand()` used to be spelled `system_tick as u16 ^ rnd_number`; this pins
    /// both the value and the single draw it takes per retransmission.
    #[test]
    #[mry::lock(
        rand,
        clock_time,
        rf_set_tx_rx_off,
        rf_set_ble_access_code,
        rf_set_ble_crc_adv,
        rf_set_ble_channel,
        rf_start_srx2tx,
        rf_set_rxmode,
        clock_time64
    )]
    fn test_send_mesh_msg_iteration_requeues_with_congestion_delay() {
        use futures::executor::block_on;

        // Setup
        reset_for_test();
        let mut dropped_tx_seen = 0;
        let mut dropped_rx_seen = 0;
        let test_pkt = create_test_packet(0x1234, 0x5678, [1, 2, 3]);

        SECURITY_ENABLE.set(false);

        // One retransmission left, due immediately.
        assert!(queues::enqueue_mesh_tx(&test_pkt, 0, 1));

        // 0x03E8 ^ 0x5555 = 0x56BD; 0x56BD % 16 == 13 -> 8000 - 13 * 500 = 1500 us.
        mock_rand().returns(1000 ^ 0x5555);
        mock_clock_time().returns(1000);
        mock_clock_time64().returns(1);
        mock_rf_set_tx_rx_off().returns(());
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_rf_set_ble_channel(Any).returns(());
        mock_rf_start_srx2tx(Any, Any).returns(());
        mock_rf_set_rxmode().returns(());

        // Execute
        block_on(mesh_packet_send_iteration(
            &mut dropped_tx_seen,
            &mut dropped_rx_seen,
        ));

        // Verify: transmitted once, then re-queued one step before the last send.
        mock_rf_set_tx_rx_off().assert_called(SYS_CHN_LISTEN.len());
        let send_queue = MESH_TX_QUEUE.lock();
        assert_eq!(send_queue.len(), 1, "the retransmission is re-queued");
        assert_eq!(send_queue[0].send_count, 0);
        assert_eq!(
            send_queue[0].delay,
            1 + 1500 * CLOCK_SYS_CLOCK_1US as u64,
            "the congestion delay must stay 8000 - (rand % 16) * 500 us"
        );
        drop(send_queue);
        mock_rand().assert_called(2);
    }

    /// Tests send_mesh_msg_iteration parks on the earliest queued deadline and
    /// wakes when a new packet is pushed.
    ///
    /// This is the `SendScan::Wait(earliest)` arm: the buffer is non-empty but
    /// nothing is due yet, so the only exit is the wake signal. Every mesh
    /// retransmission takes this path.
    #[test]
    #[mry::lock(clock_time64)]
    fn test_send_mesh_msg_iteration_parks_until_earliest_deadline() {
        use core::future::Future;
        use core::task::{Context, Poll};
        use futures::task::noop_waker;

        // Setup
        reset_for_test();
        let mut dropped_tx_seen = 0;
        let mut dropped_rx_seen = 0;

        // The scan samples the clock once (after the empty check).
        mock_clock_time64().returns(0);

        // Deadline far in the future: the buffer is not empty, but no packet is
        // due, so the scan must take `SendScan::Wait(earliest)`, not `Empty`.
        let far_pkt = create_test_packet(0x1234, 0x5678, [1, 2, 3]);
        assert!(queues::enqueue_mesh_tx(&far_pkt, u64::MAX / 2, 0));
        assert_eq!(MESH_TX_QUEUE.lock().len(), 1);

        let future = mesh_packet_send_iteration(&mut dropped_tx_seen, &mut dropped_rx_seen);
        futures::pin_mut!(future);

        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);

        // First poll: parked on the deadline wait.
        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Pending));

        // The not-yet-due packet must NOT have been consumed by the scan.
        assert_eq!(MESH_TX_QUEUE.lock().len(), 1);

        // A due packet signals the wake, which releases the parked wait.
        let ready_pkt = create_test_packet(0x9abc, 0xdef0, [4, 5, 6]);
        assert!(queues::enqueue_mesh_tx(&ready_pkt, 0, 0));
        assert_eq!(MESH_TX_QUEUE.lock().len(), 2);

        // Second poll: the wake completes the iteration without sending; the
        // outer task loop re-enters and rescans.
        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Ready(())));

        // Nothing was consumed by this iteration.
        assert_eq!(MESH_TX_QUEUE.lock().len(), 2);

        // The clock was sampled exactly once for the scan.
        mock_clock_time64().assert_called(1);
    }

    // --- rcv_mesh_msg_iteration Tests ---

    /// Tests rcv_mesh_msg_iteration parks on receive and wakes on a push
    #[test]
    #[mry::lock(rf_link_rc_data)]
    fn test_rcv_mesh_msg_iteration_no_packets() {
        use core::future::Future;
        use core::task::{Context, Poll};
        use futures::task::noop_waker;

        // Setup
        reset_for_test();
        mock_rf_link_rc_data(Any).returns(());

        // The RX channel is empty by default, so the iteration parks on receive()
        let future = mesh_packet_rcv_iteration();
        futures::pin_mut!(future);

        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);

        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Pending));

        // Enqueue a packet; Channel::try_send wakes the parked receiver
        let test_pkt = create_test_packet(0x1234, 0x5678, [1, 2, 3]);
        assert!(queues::enqueue_mesh_rx(&test_pkt));
        assert_eq!(MESH_RX_CHANNEL.len(), 1);

        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Ready(())));

        // Verify - the packet was received and handed to the mesh decoder
        assert_eq!(MESH_RX_CHANNEL.len(), 0);
        mock_rf_link_rc_data(Any).assert_called(1);
    }

    /// Tests rcv_mesh_msg_iteration with packet to receive
    #[test]
    #[mry::lock(rf_link_rc_data)]
    fn test_rcv_mesh_msg_iteration_with_packet() {
        use futures::executor::block_on;

        // Setup
        reset_for_test();
        let test_pkt = create_test_packet(0x1234, 0x5678, [1, 2, 3]);

        // Add a packet to receive
        assert!(queues::enqueue_mesh_rx(&test_pkt));
        assert_eq!(MESH_RX_CHANNEL.len(), 1);

        // Mock the rf_link_rc_data function
        mock_rf_link_rc_data(Any).returns(());

        // Execute
        block_on(mesh_packet_rcv_iteration());

        // Verify - packet should be processed and removed from buffer
        assert_eq!(MESH_RX_CHANNEL.len(), 0);
        mock_rf_link_rc_data(Any).assert_called(1);
    }
}
