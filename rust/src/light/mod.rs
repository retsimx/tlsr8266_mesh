//! The light subsystem: lamp state, PWM, persistence, LED indication and the
//! mesh command path.
//!
//! The dependency direction is strictly downward. This directory owns every
//! light-domain decision and may reach only `tlsr8266_hal`, `mesh_protocol`,
//! `crate::config` and the crates the lamp maths already needs (`fixed`,
//! `embassy_sync`, `embassy_time`, `const_format`). It never
//! names the SDK, the OTA subsystem, the UART bridge, the mesh manager, the
//! reset subsystem, diagnostics, logging or the PAC register crate, and every
//! line of it is plain safe code (`sdk/check_layering.py` enforces all of it).
//!
//! The one thing the subsystem cannot do itself is publish its on/off state into
//! the mesh node table: that routine (`ll_device_status_update`) belongs to the
//! SDK, which sits *above* this module. Instead of depending upward, the
//! composition root registers it once at boot through
//! [`set_status_update_hook`], so every report is an indirect call through a
//! pointer this module owns.

pub mod effects;
pub mod transitions;

use mesh_protocol::mesh_types::{
    GET_DEV_ADDR, GET_GROUP1, GET_GROUP2, GET_GROUP3, GET_STATUS, GET_USER_NOTIFY,
    LGT_CMD_CONFIG_DEV_ADDR, LGT_CMD_DEV_ADDR_RSP, LGT_CMD_KICK_OUT, LGT_CMD_LIGHT_CONFIG_GRP,
    LGT_CMD_LIGHT_GRP_RSP1, LGT_CMD_LIGHT_GRP_RSP2, LGT_CMD_LIGHT_GRP_RSP3, LGT_CMD_LIGHT_ONOFF,
    LGT_CMD_LIGHT_STATUS, LGT_CMD_MESH_PAIR, LGT_CMD_SET_LIGHT, LGT_CMD_SET_MAC_ADDR,
    LGT_CMD_USER_NOTIFY_RSP, LIGHT_ADD_GRP_PARAM, LIGHT_DEL_GRP_PARAM, MAX_GROUP_COUNT,
    MESH_NODE_ST_PAR_LEN,
};
use mesh_protocol::packet_types::PacketAttValue;
use tlsr8266_hal::common::sync::CriticalSectionMutex;

use crate::config::VENDOR_ID;
use transitions::{get_current_light_state, is_light_off, LIGHT_CMD_CHANNEL};

// The rest of the lamp API is reachable through `light::transitions`; only what
// the layers above actually name is flattened here.
#[cfg(test)]
pub use transitions::reset_for_test;
pub use transitions::{transition_step, LightFlashStore, Message};
// `wire_contract_tests` size-asserts the persisted record through this path.
#[cfg(test)]
pub(crate) use transitions::LuminanceRecord;

// must be 255
pub const PMW_MAX_TICK_BASE: u16 = 255;

// 209: freq = 600.4Hz
// 125: freq = 1003.9Hz
// 41: freq = 3060.7Hz
// More detail about why this should be above 3kHz can be found in this document:
// https://bio-licht.org/02_resources/info_ieee-pem_2014-09_led-flickering.pdf
pub const PMW_MAX_TICK_MULTI: u16 = 41;

// must less or equal than (255*256)
pub const PMW_MAX_TICK: u16 = PMW_MAX_TICK_BASE * PMW_MAX_TICK_MULTI;

/// The mesh node table update the light subsystem cannot perform itself.
type StatusUpdateHook = Option<fn(&[u8])>;

/// Registered once by the composition root before any mesh traffic exists, so
/// the slot is never observed empty in production.
static STATUS_UPDATE_HOOK: CriticalSectionMutex<StatusUpdateHook> = CriticalSectionMutex::new(None);

/// Install the routine that publishes a light status buffer to the mesh.
pub fn set_status_update_hook(cb: fn(&[u8])) {
    *STATUS_UPDATE_HOOK.lock() = Some(cb);
}

/// Hand a status buffer to the registered publisher, if any.
///
/// `#[inline(never)]` on purpose: the call is indirect, so the whole point of
/// having a symbol here is that `sdk/stack_analysis.py` can attribute the
/// publisher's frame to this dispatcher (see its `INDIRECT_EDGES`). The hook is
/// copied out of the mutex first: the critical section must not cover the
/// publisher, which touches mesh state and the UART bridge.
#[inline(never)]
fn notify_status_update(data: &[u8]) {
    let hook = *STATUS_UPDATE_HOOK.lock();
    if let Some(cb) = hook {
        cb(data);
    }
}

#[cfg(test)]
pub(crate) fn clear_status_update_hook() {
    *STATUS_UPDATE_HOOK.lock() = None;
}

/// Report the current on/off state to the mesh status buffer.
pub fn device_status_update() {
    // Snapshot the on/off state under the lock; the mesh notification below
    // must not run inside the critical section.
    let on = if is_light_off() { 0 } else { 1 };

    // packet
    let mut st_val_par: [u8; MESH_NODE_ST_PAR_LEN] = [0xff; MESH_NODE_ST_PAR_LEN];

    st_val_par[0] = on;
    st_val_par[1] = 0xff;

    notify_status_update(&st_val_par);
}

/// Queue a command for [`light_manager_task`]. Dropped if the queue is full.
pub fn send_light_message(cmd: u8, params: [u8; 16]) {
    let _ = LIGHT_CMD_CHANNEL.try_send(Message { cmd, params });
}

/// The one owner of the light command queue.
#[coverage(off)]
pub async fn light_manager_task() {
    loop {
        let msg = LIGHT_CMD_CHANNEL.receive().await;
        transitions::dispatch_message(msg);
    }
}

/// Build the response for one of the five light-domain request opcodes.
///
/// `GET_STATUS`, `GET_GROUP1..3`, `GET_DEV_ADDR` and `GET_USER_NOTIFY` are the
/// light subsystem's own protocol, so the whole answer is built here from
/// arguments instead of reaching into the SDK's globals. The response/request
/// prologue is the caller's historical one: destination is the requester,
/// source is this node, the vendor id sits at `val[1..3]` and `val[3..13]` starts
/// zeroed.
///
/// Returns `Err(())` **before touching `rsp`** when the opcode is not one of the
/// five: the OTA opcodes are the caller's, and it must still be able to build
/// their response unchanged.
pub fn build_light_status_response(
    rsp: &mut PacketAttValue,
    src: &PacketAttValue,
    dev_addr: u16,
    groups: &[u16; MAX_GROUP_COUNT as usize],
    mac_id: [u8; 6],
) -> Result<(), ()> {
    if !matches!(
        rsp.val[15],
        GET_STATUS | GET_GROUP1 | GET_GROUP2 | GET_GROUP3 | GET_DEV_ADDR | GET_USER_NOTIFY
    ) {
        return Err(());
    }

    rsp.dst = src.src;
    rsp.src[0] = (dev_addr & 0xff) as u8;
    rsp.src[1] = ((dev_addr >> 8) & 0xff) as u8;

    rsp.val[3..10 + 3].fill(0);

    rsp.val[1] = (VENDOR_ID & 0xFF) as u8;
    rsp.val[2] = ((VENDOR_ID >> 8) & 0xff) as u8;

    let mut idx = 0;
    match rsp.val[15] {
        GET_STATUS => {
            rsp.val[0] = LGT_CMD_LIGHT_STATUS | 0xc0;

            let state = get_current_light_state();
            rsp.val[3] = (state.cw.to_num::<u16>() & 0xff) as u8;
            rsp.val[4] = ((state.cw.to_num::<u16>() >> 8) & 0xff) as u8;
            rsp.val[5] = (state.ww.to_num::<u16>() & 0xff) as u8;
            rsp.val[6] = ((state.ww.to_num::<u16>() >> 8) & 0xff) as u8;
            rsp.val[7] = (state.brightness.to_num::<u16>() & 0xff) as u8;
            rsp.val[8] = ((state.brightness.to_num::<u16>() >> 8) & 0xff) as u8;
        }
        GET_GROUP1 => {
            rsp.val[0] = LGT_CMD_LIGHT_GRP_RSP1 | 0xc0;
            for i in 0..MAX_GROUP_COUNT as usize {
                rsp.val[i + 3] = 0xFF;
                if groups[i] != 0 {
                    rsp.val[idx + 3] = groups[i] as u8;
                    idx += 1;
                }
            }
        }
        GET_GROUP2 => {
            rsp.val[0] = LGT_CMD_LIGHT_GRP_RSP2 | 0xc0;
            for i in 0..MAX_GROUP_COUNT as usize {
                rsp.val[i + 3] = 0xFF;
                if groups[i / 2] != 0 {
                    rsp.val[idx + 3] = if (i % 2) != 0 {
                        (groups[i / 2] >> 8) as u8
                    } else {
                        groups[i / 2] as u8
                    };
                    idx += 1;
                }
            }
        }
        GET_GROUP3 => {
            rsp.val[0] = LGT_CMD_LIGHT_GRP_RSP3 | 0xc0;
            for i in 0..MAX_GROUP_COUNT as usize {
                rsp.val[i + 3] = 0xFF;
                if groups[4 + i / 2] != 0 {
                    rsp.val[idx + 3] = if (i % 2) != 0 {
                        (groups[4 + i / 2] >> 8) as u8
                    } else {
                        groups[4 + i / 2] as u8
                    };
                    idx += 1;
                }
            }
        }
        GET_DEV_ADDR => {
            rsp.val[0] = LGT_CMD_DEV_ADDR_RSP | 0xc0;
            rsp.val[4..10].copy_from_slice(&mac_id);
            return Ok(());
        }
        GET_USER_NOTIFY => {
            /*user can get parameters from APP.
            params[0] is relay times.
            params[1 -- 9] is parameters from APP if haved been set by user.

            dst_unicast == 1 means destination address is unicast address.
            */
            rsp.val[0] = LGT_CMD_USER_NOTIFY_RSP | 0xc0;
            for i in 0..8 {
                //params[2]
                rsp.val[5 + i] = i as u8;
            }
            rsp.val[3] = (dev_addr & 0xFF) as u8;
            rsp.val[4] = ((dev_addr >> 8) & 0xff) as u8;
        }
        _ => {}
    }

    Ok(())
}

/// What one decoded mesh opcode asks the caller to do.
///
/// The light subsystem decides *what* an opcode means; executing anything that
/// needs flash, the mesh node table, the pairing state machine or the radio is
/// the caller's job, which is why the effects on those subsystems are returned
/// as data instead of being performed here.
pub enum LightCommandAction {
    None,
    EnqueueLight(Message),
    ConfigGroup { add: bool, group: u16 },
    ConfigDevAddr { addr: u16, params: [u8; 16] },
    SetMacAddr([u8; 6]),
    KickOut(u8),
    MeshPair([u8; 10]),
}

/// Decode one light-domain opcode and its parameters.
pub fn decode_mesh_command(op: u8, params: &[u8; 16]) -> LightCommandAction {
    match op {
        LGT_CMD_LIGHT_ONOFF | LGT_CMD_SET_LIGHT => LightCommandAction::EnqueueLight(Message {
            cmd: op,
            params: *params,
        }),
        LGT_CMD_LIGHT_CONFIG_GRP => match params[0] {
            LIGHT_ADD_GRP_PARAM => LightCommandAction::ConfigGroup {
                add: true,
                group: u16::from_le_bytes([params[1], params[2]]),
            },
            LIGHT_DEL_GRP_PARAM => LightCommandAction::ConfigGroup {
                add: false,
                group: u16::from_le_bytes([params[1], params[2]]),
            },
            _ => LightCommandAction::None,
        },
        LGT_CMD_CONFIG_DEV_ADDR => LightCommandAction::ConfigDevAddr {
            addr: u16::from_le_bytes([params[0], params[1]]),
            params: *params,
        },
        LGT_CMD_SET_MAC_ADDR => {
            let mut mac = [0u8; 6];
            mac.copy_from_slice(&params[0..6]);
            LightCommandAction::SetMacAddr(mac)
        }
        LGT_CMD_KICK_OUT => LightCommandAction::KickOut(params[0]),
        LGT_CMD_MESH_PAIR => {
            let mut pair_params = [0u8; 10];
            pair_params.copy_from_slice(&params[0..10]);
            LightCommandAction::MeshPair(pair_params)
        }
        _ => LightCommandAction::None,
    }
}

#[cfg(test)]
pub(crate) mod status_probe {
    //! Host-test observer for [`super::set_status_update_hook`].
    //!
    //! The production hook is a plain `fn` pointer, so a test installs this
    //! recorder and reads back the last status buffer it was handed plus how many
    //! times it ran. The state is process-wide, so every test resets it first.

    use core::sync::atomic::{AtomicU32, Ordering};

    /// Last two status bytes packed as `byte0 << 8 | byte1`; `u32::MAX` = never.
    static LAST: AtomicU32 = AtomicU32::new(u32::MAX);
    static CALLS: AtomicU32 = AtomicU32::new(0);

    fn hook(data: &[u8]) {
        let byte0 = data.first().copied().unwrap_or(0xff) as u32;
        let byte1 = data.get(1).copied().unwrap_or(0xff) as u32;
        LAST.store(byte0 << 8 | byte1, Ordering::SeqCst);
        CALLS.fetch_add(1, Ordering::SeqCst);
    }

    /// Clear the recorder and install it as the status hook.
    pub(crate) fn reset() {
        LAST.store(u32::MAX, Ordering::SeqCst);
        CALLS.store(0, Ordering::SeqCst);
        super::set_status_update_hook(hook);
    }

    pub(crate) fn calls() -> u32 {
        CALLS.load(Ordering::SeqCst)
    }

    pub(crate) fn last() -> Option<[u8; 2]> {
        match LAST.load(Ordering::SeqCst) {
            u32::MAX => None,
            packed => Some([(packed >> 8) as u8, packed as u8]),
        }
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use mesh_protocol::mesh_types::LIGHT_ON_PARAM;
    use transitions::{begin_transition, light_onoff_hw};

    /// The four response fields, so a "nothing changed" check does not need
    /// `PacketAttValue` to implement `PartialEq`/`Debug`.
    type ResponseSnapshot = ([u8; 3], [u8; 2], [u8; 2], [u8; 23]);

    fn snapshot(response: &PacketAttValue) -> ResponseSnapshot {
        (response.sno, response.src, response.dst, response.val)
    }

    /// A request/response pair with the field values the response builder reads.
    fn packet_pair(opcode: u8) -> (PacketAttValue, PacketAttValue) {
        let request = PacketAttValue {
            sno: [0x01, 0x02, 0x03],
            src: [0x56, 0x78],
            dst: [0x1F, 0x00],
            val: [0; 23],
        };
        let mut response = PacketAttValue {
            sno: [0; 3],
            src: [0x03, 0x04],
            dst: [0x01, 0x02],
            val: [0; 23],
        };
        response.val[15] = opcode;
        (request, response)
    }

    const NO_GROUPS: [u16; MAX_GROUP_COUNT as usize] = [0; MAX_GROUP_COUNT as usize];

    // --- device_status_update ---

    #[test]
    fn test_device_status_update_reports_on() {
        reset_for_test();
        // Drive the lamp on through the real transition start, then report.
        light_onoff_hw(true);
        status_probe::reset();

        device_status_update();

        assert_eq!(status_probe::calls(), 1, "one status report per update");
        assert_eq!(
            status_probe::last(),
            Some([1, 0xff]),
            "a lit lamp reports on"
        );
    }

    #[test]
    fn test_device_status_update_reports_off() {
        reset_for_test();
        // The default core is already off (brightness_tr targets 0).
        status_probe::reset();

        device_status_update();

        assert_eq!(status_probe::calls(), 1, "one status report per update");
        assert_eq!(
            status_probe::last(),
            Some([0, 0xff]),
            "a dark lamp reports off"
        );
    }

    #[test]
    fn test_device_status_update_without_hook_is_a_no_op() {
        reset_for_test();
        status_probe::reset();
        clear_status_update_hook();

        // No publisher registered: the report must be dropped, not panic.
        device_status_update();

        assert_eq!(status_probe::calls(), 0, "no hook, no publish");
        assert_eq!(status_probe::last(), None);
    }

    // --- build_light_status_response ---

    #[test]
    fn test_build_light_status_response_get_status() {
        reset_for_test();
        begin_transition(8767, 24000, 16384);
        let (request, mut response) = packet_pair(GET_STATUS);

        let result =
            build_light_status_response(&mut response, &request, 0x1234, &NO_GROUPS, [0; 6]);

        assert!(result.is_ok(), "GET_STATUS is a light opcode");
        assert_eq!(response.dst, request.src, "destination is the requester");
        assert_eq!(response.src, [0x34, 0x12], "source is this node, LE");
        assert_eq!(
            response.val[1..3],
            VENDOR_ID.to_le_bytes(),
            "vendor id sits at val[1..3]"
        );
        assert_eq!(response.val[0], LGT_CMD_LIGHT_STATUS | 0xc0);
        assert_eq!(&response.val[3..5], &8767u16.to_le_bytes(), "cw");
        assert_eq!(&response.val[5..7], &24000u16.to_le_bytes(), "ww");
        assert_eq!(&response.val[7..9], &16384u16.to_le_bytes(), "brightness");
        assert_eq!(
            &response.val[9..13],
            &[0, 0, 0, 0],
            "the prologue leaves the trailing parameters zeroed"
        );
    }

    #[test]
    fn test_build_light_status_response_get_group1() {
        let (request, mut response) = packet_pair(GET_GROUP1);
        let mut groups = NO_GROUPS;
        groups[0] = 0x11;
        groups[1] = 0x22;
        groups[2] = 0x33;

        let result = build_light_status_response(&mut response, &request, 0x1234, &groups, [0; 6]);

        assert!(result.is_ok());
        assert_eq!(response.val[0], LGT_CMD_LIGHT_GRP_RSP1 | 0xc0);
        assert_eq!(response.val[3], 0x11, "first group address");
        assert_eq!(response.val[4], 0x22, "second group address");
        assert_eq!(response.val[5], 0x33, "third group address");
        // The remaining slots keep the "no such group" marker.
        assert_eq!(&response.val[6..11], &[0xFF; 5]);
    }

    #[test]
    fn test_build_light_status_response_get_group2() {
        let (request, mut response) = packet_pair(GET_GROUP2);
        let mut groups = NO_GROUPS;
        groups[0] = 0xAABB;
        groups[1] = 0xCCDD;

        let result = build_light_status_response(&mut response, &request, 0x1234, &groups, [0; 6]);

        assert!(result.is_ok());
        assert_eq!(response.val[0], LGT_CMD_LIGHT_GRP_RSP2 | 0xc0);
        assert_eq!(response.val[3], 0xBB, "first group low byte");
        assert_eq!(response.val[4], 0xAA, "first group high byte");
        assert_eq!(response.val[5], 0xDD, "second group low byte");
        assert_eq!(response.val[6], 0xCC, "second group high byte");
    }

    #[test]
    fn test_build_light_status_response_get_group3() {
        let (request, mut response) = packet_pair(GET_GROUP3);
        let mut groups = NO_GROUPS;
        groups[4] = 0x1122;
        groups[5] = 0x3344;

        let result = build_light_status_response(&mut response, &request, 0x1234, &groups, [0; 6]);

        assert!(result.is_ok());
        assert_eq!(response.val[0], LGT_CMD_LIGHT_GRP_RSP3 | 0xc0);
        assert_eq!(response.val[3], 0x22, "fifth group low byte");
        assert_eq!(response.val[4], 0x11, "fifth group high byte");
        assert_eq!(response.val[5], 0x44, "sixth group low byte");
        assert_eq!(response.val[6], 0x33, "sixth group high byte");
    }

    #[test]
    fn test_build_light_status_response_get_dev_addr() {
        let (request, mut response) = packet_pair(GET_DEV_ADDR);
        let mac_id = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];

        let result =
            build_light_status_response(&mut response, &request, 0x1234, &NO_GROUPS, mac_id);

        assert!(result.is_ok());
        assert_eq!(response.val[0], LGT_CMD_DEV_ADDR_RSP | 0xc0);
        assert_eq!(
            &response.val[4..10],
            &mac_id,
            "the device MAC lands at val[4..10]"
        );
        assert_eq!(&response.val[3..4], &[0], "val[3] stays zeroed");
    }

    #[test]
    fn test_build_light_status_response_get_user_notify() {
        let (request, mut response) = packet_pair(GET_USER_NOTIFY);

        let result =
            build_light_status_response(&mut response, &request, 0xABCD, &NO_GROUPS, [0; 6]);

        assert!(result.is_ok());
        assert_eq!(response.val[0], LGT_CMD_USER_NOTIFY_RSP | 0xc0);
        assert_eq!(response.val[3], 0xCD, "device address low byte");
        assert_eq!(response.val[4], 0xAB, "device address high byte");
        for i in 0..8 {
            assert_eq!(response.val[5 + i], i as u8, "user parameters are 0..8");
        }
    }

    #[test]
    fn test_build_light_status_response_rejects_unknown_opcode_without_mutating() {
        let (request, mut response) = packet_pair(0xFF);
        let before = snapshot(&response);

        let result =
            build_light_status_response(&mut response, &request, 0x1234, &NO_GROUPS, [0x11; 6]);

        assert!(result.is_err(), "an unknown opcode is not a light opcode");
        assert_eq!(
            snapshot(&response),
            before,
            "the response must be untouched when the opcode is rejected"
        );
    }

    #[test]
    fn test_build_light_status_response_leaves_ota_opcodes_alone() {
        use mesh_protocol::mesh_types::{CMD_END_OTA, CMD_OTA_DATA, CMD_START_OTA};
        // The three OTA opcodes belong to the caller: the builder must refuse
        // them without touching the response, or the caller's arm cannot run.
        for opcode in [CMD_START_OTA, CMD_OTA_DATA, CMD_END_OTA] {
            let (request, mut response) = packet_pair(opcode);
            let before = snapshot(&response);
            assert!(
                build_light_status_response(&mut response, &request, 0x1234, &NO_GROUPS, [0; 6])
                    .is_err(),
                "OTA opcode {opcode:#04x} must not be built here"
            );
            assert_eq!(
                snapshot(&response),
                before,
                "OTA opcode {opcode:#04x} must not mutate"
            );
        }
    }

    // --- decode_mesh_command ---

    #[test]
    fn test_decode_mesh_command_enqueues_onoff() {
        let params = [
            LIGHT_ON_PARAM,
            1,
            2,
            3,
            4,
            5,
            6,
            7,
            8,
            9,
            10,
            11,
            12,
            13,
            14,
            15,
        ];

        match decode_mesh_command(LGT_CMD_LIGHT_ONOFF, &params) {
            LightCommandAction::EnqueueLight(message) => {
                assert_eq!(message.cmd, LGT_CMD_LIGHT_ONOFF);
                assert_eq!(message.params, params);
            }
            _ => panic!("LGT_CMD_LIGHT_ONOFF must be queued for the light task"),
        }
    }

    #[test]
    fn test_decode_mesh_command_enqueues_set_light() {
        let params = [0x10, 0x30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        match decode_mesh_command(LGT_CMD_SET_LIGHT, &params) {
            LightCommandAction::EnqueueLight(message) => {
                assert_eq!(message.cmd, LGT_CMD_SET_LIGHT);
                assert_eq!(message.params, params);
            }
            _ => panic!("LGT_CMD_SET_LIGHT must be queued for the light task"),
        }
    }

    #[test]
    fn test_decode_mesh_command_config_group_add_and_delete() {
        let mut params = [0u8; 16];
        params[0] = LIGHT_ADD_GRP_PARAM;
        params[1] = 0x34;
        params[2] = 0x12;

        match decode_mesh_command(LGT_CMD_LIGHT_CONFIG_GRP, &params) {
            LightCommandAction::ConfigGroup { add, group } => {
                assert!(add, "LIGHT_ADD_GRP_PARAM adds");
                assert_eq!(group, 0x1234, "the group id is little-endian");
            }
            _ => panic!("a known group parameter must decode"),
        }

        params[0] = LIGHT_DEL_GRP_PARAM;
        match decode_mesh_command(LGT_CMD_LIGHT_CONFIG_GRP, &params) {
            LightCommandAction::ConfigGroup { add, group } => {
                assert!(!add, "LIGHT_DEL_GRP_PARAM removes");
                assert_eq!(group, 0x1234);
            }
            _ => panic!("a known group parameter must decode"),
        }
    }

    #[test]
    fn test_decode_mesh_command_config_group_unknown_parameter() {
        let mut params = [0u8; 16];
        params[0] = 0x7f;
        assert!(
            matches!(
                decode_mesh_command(LGT_CMD_LIGHT_CONFIG_GRP, &params),
                LightCommandAction::None
            ),
            "an unknown group action is ignored, not guessed"
        );
    }

    #[test]
    fn test_decode_mesh_command_config_dev_addr() {
        let params = [0x78, 0x56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        match decode_mesh_command(LGT_CMD_CONFIG_DEV_ADDR, &params) {
            LightCommandAction::ConfigDevAddr { addr, params: got } => {
                assert_eq!(addr, 0x5678, "the device address is little-endian");
                assert_eq!(got, params, "the caller re-checks the full parameter block");
            }
            _ => panic!("LGT_CMD_CONFIG_DEV_ADDR must decode"),
        }
    }

    #[test]
    fn test_decode_mesh_command_set_mac_addr() {
        let params = [
            0x11, 0x66, 0x22, 0x33, 0x44, 0x55, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ];

        match decode_mesh_command(LGT_CMD_SET_MAC_ADDR, &params) {
            LightCommandAction::SetMacAddr(mac) => {
                assert_eq!(mac, [0x11, 0x66, 0x22, 0x33, 0x44, 0x55]);
            }
            _ => panic!("LGT_CMD_SET_MAC_ADDR must decode"),
        }
    }

    #[test]
    fn test_decode_mesh_command_kick_out() {
        // Every wire reason is passed through unchanged: the policy lives in
        // `factory_reset::plan_kick_out`, not here.
        for reason in 0u8..3 {
            match decode_mesh_command(LGT_CMD_KICK_OUT, &[reason; 16]) {
                LightCommandAction::KickOut(got) => assert_eq!(got, reason),
                _ => panic!("LGT_CMD_KICK_OUT must decode"),
            }
        }
    }

    #[test]
    fn test_decode_mesh_command_mesh_pair() {
        let params = [
            0x01, 0x0A, 0x02, 0x0B, 0x03, 0x0C, 0x04, 0x0D, 0x05, 0x0E, 0, 0, 0, 0, 0, 0,
        ];

        match decode_mesh_command(LGT_CMD_MESH_PAIR, &params) {
            LightCommandAction::MeshPair(got) => {
                assert_eq!(
                    got,
                    [0x01, 0x0A, 0x02, 0x0B, 0x03, 0x0C, 0x04, 0x0D, 0x05, 0x0E]
                );
            }
            _ => panic!("LGT_CMD_MESH_PAIR must decode"),
        }
    }

    #[test]
    fn test_decode_mesh_command_unknown_op_is_none() {
        assert!(
            matches!(
                decode_mesh_command(0xFF, &[0; 16]),
                LightCommandAction::None
            ),
            "an unknown opcode is ignored"
        );
    }

    // --- PWM tick constants (moved from `sdk::light::tests`) ---

    #[test]
    fn test_pwm_tick_constants() {
        // PWM constants
        assert_eq!(PMW_MAX_TICK_BASE, 255);
        assert_eq!(PMW_MAX_TICK_MULTI, 41);
        assert_eq!(PMW_MAX_TICK, PMW_MAX_TICK_BASE * PMW_MAX_TICK_MULTI);
        // `pwm_set_lum` divides by `255 * 256` and the compare field is 16 bits,
        // so the tick count must stay inside both bounds.
        assert!(u32::from(PMW_MAX_TICK) <= 255 * 256);
    }
}
