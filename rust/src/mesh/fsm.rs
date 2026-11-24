//! The re-pairing state machine, with every hardware edge removed.
//!
//! Re-pairing a live mesh is a protocol, not a radio feature: one node
//! broadcasts the six credential chunks of a new network (Name1/2, Pwd1/2,
//! Ltk1/2), asks the other nodes to confirm, and then tells them to switch. The
//! transitions, the timers and the exact payload bytes all live here.
//!
//! Everything in this file is a function of the machine's own state and an
//! injected `now` tick. No clock is read, no register is touched, no global is
//! named: `mod.rs` samples the tick and executes the [`MeshPairAction`]s
//! this machine returns. That is what lets the whole protocol be tested with
//! zero mocks — every test below drives states, chunks and tick values directly.
//!
//! Time is compared with the same 32-bit wrapping rule the HAL's elapsed-time
//! predicate uses, reproduced in [`exceeded`] because calling that predicate
//! would put a clock read back inside the machine.

use mesh_protocol::mesh_types::{LGT_CMD_MESH_PAIR, PAR_READ_MESH_PAIR_CONFIRM};
use tlsr8266_hal::mcu::clock::CLOCK_SYS_CLOCK_1US;

/// The pairing flag is a plain SDK enum with no hardware behind it, so the
/// machine mirrors that type directly instead of inventing a second one plus
/// two conversions. `mod.rs` keeps the SDK static in sync with it.
pub use tlsr8266_radio::types::ePairState as PairSetState;

/// How often the re-pairing machine broadcasts one chunk, in milliseconds.
pub const MESH_PAIR_CMD_INTERVAL: u32 = 500;

/// How long a pairing window stays open after the first command, in seconds.
pub const MESH_PAIR_TIMEOUT: u32 = 10;

/// Mesh pairing state machine commands for re-pairing devices in an existing mesh.
///
/// Credentials are transmitted in 6 messages (Name1/2, Pwd1/2, Ltk1/2), then
/// devices switch to the new mesh using MeshPairEffectDelay (delayed switch).
///
/// Note: MeshPairEffect (immediate switch) exists for receiving only - it's not
/// sent by this implementation as it was part of a broken single-device optimization.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshPairState {
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
            0 => Ok(MeshPairState::MeshPairName1),
            1 => Ok(MeshPairState::MeshPairName2),
            2 => Ok(MeshPairState::MeshPairPwd1),
            3 => Ok(MeshPairState::MeshPairPwd2),
            4 => Ok(MeshPairState::MeshPairLtk1),
            5 => Ok(MeshPairState::MeshPairLtk2),
            6 => Ok(MeshPairState::MeshPairEffectDelay),
            7 => Ok(MeshPairState::MeshPairEffect),
            8 => Ok(MeshPairState::MeshPairDefaultMesh),
            _ => Err(()),
        }
    }
}

/// What the caller must do after one step of the machine.
///
/// Every arm names work the machine cannot do itself: flash (`ReloadSavedKey`,
/// `SaveAndApply`), the radio (`Broadcast`), the pairing flag and the LED/mesh
/// event (`TimedOut`), or the access code (`SwitchToDefaultMesh`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshPairAction {
    /// Nothing to do.
    None,
    /// Persist these credentials and switch to them.
    SaveAndApply {
        name: [u8; 16],
        pwd: [u8; 16],
        ltk: [u8; 16],
        use_pair_ltk: bool,
    },
    /// Fall back to the configured default mesh for `delay_s` seconds.
    SwitchToDefaultMesh { delay_s: u8 },
    /// Re-read the persisted key (`pair_load_key`).
    ReloadSavedKey,
    /// Transmit these 16 bytes as `LGT_CMD_MESH_PAIR` parameters.
    Broadcast([u8; 16]),
    /// The pairing window expired: settle the flag and fire the timeout event.
    TimedOut,
}

/// Has `span_us` microseconds elapsed since `reference`, measured at `now`?
///
/// This is the HAL's elapsed-time predicate with the current tick supplied by
/// the caller: the subtraction wraps, so the comparison keeps working across the
/// 32-bit tick roll-over.
fn exceeded(reference: u32, span_us: u32, now: u32) -> bool {
    now.wrapping_sub(reference) > span_us.wrapping_mul(CLOCK_SYS_CLOCK_1US)
}

/// Sole owner of the pairing state machine.
///
/// The fields are readable inside this module's parent so the moved integration
/// tests can assert them individually, exactly as they did when the state lived
/// on the old mesh manager; production code only ever mutates them through the
/// methods below.
pub struct MeshPairFsm {
    pub(super) mesh_pair_start_time: u32,
    pub(super) default_mesh_time: u32,
    pub(super) default_mesh_effect_delay_ref: u32,
    pub(super) new_mesh_name: [u8; 16],
    pub(super) new_mesh_pwd: [u8; 16],
    pub(super) new_mesh_ltk: [u8; 16],
    pub(super) default_mesh_time_ref: u32,
    pub(super) effect_new_mesh: u8,
    pub(super) effect_new_mesh_delay_time: u32,
    pub(super) mesh_pair_cmd_interval: u32,
    pub(super) mesh_pair_timeout: u32,
    pub(super) mesh_pair_time: u32,
    pub(super) mesh_pair_state: MeshPairState,
    /// The credentials this node broadcasts: the configured network the machine
    /// was seeded with (see [`MeshPairFsm::set_credentials`]).
    pub(super) mesh_name: [u8; 16],
    pub(super) mesh_pwd: [u8; 16],
    pub(super) mesh_ltk: [u8; 16],
    /// Local mirror of the SDK's pairing flag, refreshed by [`MeshPairFsm::sync_flag`].
    pair_setting_flag: PairSetState,
}

impl MeshPairFsm {
    /// Const constructor so the owner can be a `static` initialiser.
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
            mesh_name: [0; 16],
            mesh_pwd: [0; 16],
            mesh_ltk: [0; 16],
            pair_setting_flag: PairSetState::PairSetted,
        }
    }

    /// Seed the copies [`MeshPairFsm::interval_tick`] broadcasts from.
    pub fn set_credentials(&mut self, name: &[u8; 16], pwd: &[u8; 16], ltk: &[u8; 16]) {
        self.mesh_name = *name;
        self.mesh_pwd = *pwd;
        self.mesh_ltk = *ltk;
    }

    /// Take the current value of the SDK's pairing flag.
    pub fn sync_flag(&mut self, flag: PairSetState) {
        self.pair_setting_flag = flag;
    }

    /// The pairing flag after the last step, for the owner to write back.
    pub fn pair_setting_flag(&self) -> PairSetState {
        self.pair_setting_flag
    }

    /// First phase of the pairing timer: the default-mesh window.
    ///
    /// Returns `ReloadSavedKey` when the default mesh has timed out,
    /// `SwitchToDefaultMesh` when its delay just expired, and `None` otherwise.
    /// The caller executes the returned action *after* dropping its guard,
    /// because both arms reach flash or the radio statics.
    pub fn timer_step(&mut self, now: u32) -> MeshPairAction {
        let mut load_key = false;
        let mut switch_delay = None;

        if self.default_mesh_effect_delay_ref != 0
            && exceeded(
                self.default_mesh_effect_delay_ref,
                MESH_PAIR_CMD_INTERVAL * 1000,
                now,
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
            && exceeded(
                self.default_mesh_time_ref,
                self.default_mesh_time * 1000,
                now,
            )
        {
            /* Switch to normal mesh */
            if self.default_mesh_time == 255000 {
                self.default_mesh_time_ref = now | 1;
            } else {
                load_key = true;
                self.default_mesh_time_ref = 0;
            }
        }

        match switch_delay {
            Some(delay_s) => MeshPairAction::SwitchToDefaultMesh { delay_s },
            None if load_key => MeshPairAction::ReloadSavedKey,
            None => MeshPairAction::None,
        }
    }

    /// Has the pairing window expired?
    ///
    /// A zero start time means the window was never opened.
    pub fn timed_out(&self, now: u32) -> bool {
        self.mesh_pair_start_time != 0
            && exceeded(
                self.mesh_pair_start_time,
                self.mesh_pair_timeout * 1000 * 1000,
                now,
            )
    }

    /// Third phase: advance the re-pairing broadcast state machine.
    ///
    /// Returns the 16-byte parameter block to broadcast, or `None` when the
    /// command interval is not due or the machine has finished. Touches no
    /// flash and no radio; the caller transmits after dropping the guard.
    pub fn interval_tick(&mut self, now: u32) -> Option<MeshPairAction> {
        let mut op_para: [u8; 16] = [0; 16];

        let flag = self.pair_setting_flag;
        if !(flag as u8 >= PairSetState::PairSetMeshTxStart as u8
            && exceeded(self.mesh_pair_time, self.mesh_pair_cmd_interval * 1000, now))
        {
            return None;
        }

        self.mesh_pair_time = now;
        if flag == PairSetState::PairSetMeshTxStart {
            op_para[0] = LGT_CMD_MESH_PAIR;
            op_para[3] = self.mesh_pair_state as u8;
            match self.mesh_pair_state {
                MeshPairState::MeshPairName1 => {
                    // send mesh name [0-7]
                    op_para[4..4 + 8].copy_from_slice(&self.mesh_name[0..8]);
                    self.mesh_pair_state = MeshPairState::MeshPairName2;
                }
                MeshPairState::MeshPairName2 => {
                    // send mesh name [8-15]
                    op_para[4..4 + 8].copy_from_slice(&self.mesh_name[8..16]);
                    self.mesh_pair_state = MeshPairState::MeshPairPwd1;
                }
                MeshPairState::MeshPairPwd1 => {
                    // send mesh pwd [0-7]
                    op_para[4..4 + 8].copy_from_slice(&self.mesh_pwd[0..8]);
                    self.mesh_pair_state = MeshPairState::MeshPairPwd2;
                }
                MeshPairState::MeshPairPwd2 => {
                    // send mesh pwd [8-15]
                    op_para[4..4 + 8].copy_from_slice(&self.mesh_pwd[8..16]);
                    self.mesh_pair_state = MeshPairState::MeshPairLtk1;
                }
                MeshPairState::MeshPairLtk1 => {
                    // send mesh ltk [0-7]
                    op_para[4..4 + 8].copy_from_slice(&self.mesh_ltk[0..8]);
                    self.mesh_pair_state = MeshPairState::MeshPairLtk2;
                }
                MeshPairState::MeshPairLtk2 => {
                    // send mesh ltk [8-15]
                    op_para[4..4 + 8].copy_from_slice(&self.mesh_ltk[8..16]);
                    self.mesh_pair_state = MeshPairState::MeshPairName1;
                    self.pair_setting_flag = PairSetState::PairSetMeshTxDone;
                }
                _ => {
                    self.mesh_pair_state = MeshPairState::MeshPairName1;
                    self.mesh_pair_start_time = 0;
                    self.pair_setting_flag = PairSetState::PairSetted;
                    return None;
                }
            }
        } else if flag == PairSetState::PairSetMeshTxDone {
            // get mesh nodes' confirm value
            op_para[0] = 0;
            op_para[3] = 0x10; // bridge cnt
            op_para[4] = PAR_READ_MESH_PAIR_CONFIRM;
            self.pair_setting_flag = PairSetState::PairSetMeshRxDone;
        } else if flag == PairSetState::PairSetMeshRxDone {
            //send cmd to switch to new mesh
            op_para[0] = LGT_CMD_MESH_PAIR;
            op_para[3] = MeshPairState::MeshPairEffectDelay as u8;
            self.mesh_pair_state = MeshPairState::MeshPairName1;
            self.mesh_pair_start_time = 0;
            self.pair_setting_flag = PairSetState::PairSetted;
        }

        Some(MeshPairAction::Broadcast(op_para))
    }

    /// Buffer the first or second 8-byte half of the received network name.
    ///
    /// `state` is the wire command byte, so a chunk the mesh relay delivers
    /// twice rewrites the same half instead of corrupting the other one. The
    /// first half is also what opens the pairing window, which is why this arm
    /// takes the tick: `MeshPairName1` stamps `now | 1` as the window start.
    pub fn accumulate_name(
        &mut self,
        state: MeshPairState,
        chunk: &[u8; 8],
        now: u32,
    ) -> MeshPairAction {
        match state {
            MeshPairState::MeshPairName1 => {
                self.mesh_pair_start_time = now | 1;
                self.new_mesh_name[0..8].copy_from_slice(chunk);
            }
            MeshPairState::MeshPairName2 => self.new_mesh_name[8..16].copy_from_slice(chunk),
            _ => (),
        }
        MeshPairAction::None
    }

    /// Buffer the first or second 8-byte half of the received password.
    pub fn accumulate_pwd(&mut self, state: MeshPairState, chunk: &[u8; 8]) -> MeshPairAction {
        match state {
            MeshPairState::MeshPairPwd1 => self.new_mesh_pwd[0..8].copy_from_slice(chunk),
            MeshPairState::MeshPairPwd2 => self.new_mesh_pwd[8..16].copy_from_slice(chunk),
            _ => (),
        }
        MeshPairAction::None
    }

    /// Buffer the first or second 8-byte half of the received mesh LTK.
    pub fn accumulate_ltk(&mut self, state: MeshPairState, chunk: &[u8; 8]) -> MeshPairAction {
        match state {
            MeshPairState::MeshPairLtk1 => self.new_mesh_ltk[0..8].copy_from_slice(chunk),
            MeshPairState::MeshPairLtk2 => self.new_mesh_ltk[8..16].copy_from_slice(chunk),
            _ => (),
        }
        MeshPairAction::None
    }

    /// Arm the delayed switch: the credentials are saved once this window ends.
    pub fn accumulate_effect_delay(&mut self, now: u32) -> MeshPairAction {
        self.effect_new_mesh_delay_time = now | 1;
        if self.default_mesh_time_ref != 0 {
            /* Keep default_mesh_time_ref non-zero */
            self.default_mesh_time = self.mesh_pair_cmd_interval * 2;
        }
        MeshPairAction::None
    }

    /// Arm the immediate switch (receive-only command).
    pub fn accumulate_effect(&mut self) -> MeshPairAction {
        self.effect_new_mesh = 1;
        MeshPairAction::None
    }

    /// Start the default-mesh window requested by a received command.
    pub fn accumulate_default_mesh(&mut self, delay_s: u8, now: u32) -> MeshPairAction {
        self.default_mesh_effect_delay_ref = now | 1;
        self.default_mesh_time = delay_s as u32 * 1000;
        MeshPairAction::None
    }

    /// Is the pending credential switch due?
    ///
    /// The window is `mesh_pair_cmd_interval` after the delayed-effect command,
    /// which is what gives the tool a chance to call the switch off by asking
    /// for the default mesh instead.
    pub fn effect_action(&self, now: u32, validation_pending: bool) -> Option<MeshPairAction> {
        let due = self.effect_new_mesh != 0
            || (self.effect_new_mesh_delay_time != 0
                && exceeded(
                    self.effect_new_mesh_delay_time,
                    self.mesh_pair_cmd_interval * 1000,
                    now,
                ));
        if !due {
            return None;
        }

        // Only skip saving if either:
        // 1. We're in a default mesh timeout period, OR
        // 2. Device address validation is still pending (not yet completed)
        if self.default_mesh_time_ref != 0 || validation_pending {
            return Some(MeshPairAction::ReloadSavedKey);
        }

        Some(MeshPairAction::SaveAndApply {
            name: self.new_mesh_name,
            pwd: self.new_mesh_pwd,
            ltk: self.new_mesh_ltk,
            use_pair_ltk: self.effect_new_mesh != 0,
        })
    }
}

impl Default for MeshPairFsm {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;

    /// Ticks per broadcast interval at the HAL's 32 ticks/us.
    const INTERVAL_TICKS: u32 = MESH_PAIR_CMD_INTERVAL * 1000 * CLOCK_SYS_CLOCK_1US;
    /// Ticks per pairing-window second.
    const SECOND_TICKS: u32 = 1000 * 1000 * CLOCK_SYS_CLOCK_1US;

    const NAME: [u8; 16] = [
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e,
        0x1f,
    ];
    const PWD: [u8; 16] = [
        0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e,
        0x2f,
    ];
    const LTK: [u8; 16] = [
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e,
        0x3f,
    ];

    /// A machine armed the way `mesh_pair_init` arms it, seeded with [`NAME`],
    /// [`PWD`] and [`LTK`], with a broadcast due immediately.
    fn armed_fsm() -> MeshPairFsm {
        let mut fsm = MeshPairFsm::new();
        fsm.mesh_pair_cmd_interval = MESH_PAIR_CMD_INTERVAL;
        fsm.mesh_pair_timeout = MESH_PAIR_TIMEOUT;
        fsm.set_credentials(&NAME, &PWD, &LTK);
        fsm
    }

    fn chunk_of(source: &[u8; 16], second_half: bool) -> [u8; 8] {
        let mut chunk = [0u8; 8];
        let offset = if second_half { 8 } else { 0 };
        chunk.copy_from_slice(&source[offset..offset + 8]);
        chunk
    }

    // --- MeshPairState::try_from ---

    /// Tests MeshPairState::try_from with all valid enum values
    #[test]
    fn test_mesh_pair_state_try_from_valid() {
        let expected = [
            MeshPairState::MeshPairName1,
            MeshPairState::MeshPairName2,
            MeshPairState::MeshPairPwd1,
            MeshPairState::MeshPairPwd2,
            MeshPairState::MeshPairLtk1,
            MeshPairState::MeshPairLtk2,
            MeshPairState::MeshPairEffectDelay,
            MeshPairState::MeshPairEffect,
            MeshPairState::MeshPairDefaultMesh,
        ];
        for (value, state) in expected.iter().enumerate() {
            assert_eq!(MeshPairState::try_from(value as u8), Ok(*state));
            // The wire byte is the discriminant, so decoding must round-trip.
            assert_eq!(*state as u8, value as u8);
        }
    }

    /// Tests MeshPairState::try_from with invalid value
    #[test]
    fn test_mesh_pair_state_try_from_invalid() {
        assert!(MeshPairState::try_from(9).is_err());
        assert!(MeshPairState::try_from(255).is_err());
    }

    // --- credentials + flag mirror ---

    /// `set_credentials` and the flag mirror are the only inputs the broadcast
    /// and the gate read, so they must round-trip untouched.
    #[test]
    fn test_set_credentials_and_flag_mirror() {
        let mut fsm = armed_fsm();
        assert_eq!(fsm.pair_setting_flag(), PairSetState::PairSetted);

        fsm.sync_flag(PairSetState::PairSetMeshTxStart);
        assert_eq!(fsm.pair_setting_flag(), PairSetState::PairSetMeshTxStart);

        // A fresh machine knows no credentials at all.
        let empty = MeshPairFsm::new();
        assert_eq!(empty.mesh_name, [0; 16]);
        assert_eq!(empty.mesh_pwd, [0; 16]);
        assert_eq!(empty.mesh_ltk, [0; 16]);
    }

    // --- interval_tick: the six chunk broadcasts, then confirm, then switch ---

    /// The whole broadcast sequence: six credential chunks in order, the
    /// confirm request, and the switch command. Every frame's bytes are pinned.
    #[test]
    fn test_interval_tick_broadcasts_all_six_chunks_then_confirms_and_switches() {
        let mut fsm = armed_fsm();
        fsm.sync_flag(PairSetState::PairSetMeshTxStart);
        let mut now = 1_000 + INTERVAL_TICKS + 1;

        let chunks = [
            (MeshPairState::MeshPairName1, chunk_of(&NAME, false)),
            (MeshPairState::MeshPairName2, chunk_of(&NAME, true)),
            (MeshPairState::MeshPairPwd1, chunk_of(&PWD, false)),
            (MeshPairState::MeshPairPwd2, chunk_of(&PWD, true)),
            (MeshPairState::MeshPairLtk1, chunk_of(&LTK, false)),
            (MeshPairState::MeshPairLtk2, chunk_of(&LTK, true)),
        ];

        for (index, (state, chunk)) in chunks.iter().enumerate() {
            assert_eq!(fsm.mesh_pair_state, *state, "chunk {index} order");

            let action = fsm.interval_tick(now).expect("a chunk is due");
            let MeshPairAction::Broadcast(op_para) = action else {
                panic!("the broadcast arms only ever return Broadcast");
            };
            assert_eq!(op_para[0], LGT_CMD_MESH_PAIR, "chunk {index} opcode");
            assert_eq!(op_para[1], 0);
            assert_eq!(op_para[2], 0);
            assert_eq!(op_para[3], *state as u8, "chunk {index} state byte");
            assert_eq!(&op_para[4..12], chunk, "chunk {index} payload");
            assert_eq!(&op_para[12..16], &[0, 0, 0, 0], "chunk {index} tail");
            assert_eq!(fsm.mesh_pair_time, now, "the interval stamp advances");

            now += INTERVAL_TICKS + 1;
        }

        // Ltk2 hands the machine to the confirm phase.
        assert_eq!(
            fsm.mesh_pair_state,
            MeshPairState::MeshPairName1,
            "the chunk cursor wraps back to Name1"
        );
        assert_eq!(fsm.pair_setting_flag(), PairSetState::PairSetMeshTxDone);

        // Confirm request: bridge count in the state byte, PAR in the payload.
        let action = fsm.interval_tick(now).expect("the confirm is due");
        let MeshPairAction::Broadcast(op_para) = action else {
            panic!("the confirm arm returns Broadcast");
        };
        assert_eq!(op_para[0], 0, "the confirm frame carries no vendor opcode");
        assert_eq!(op_para[1], 0);
        assert_eq!(op_para[2], 0);
        assert_eq!(op_para[3], 0x10, "bridge count");
        assert_eq!(op_para[4], PAR_READ_MESH_PAIR_CONFIRM);
        assert_eq!(&op_para[5..16], &[0; 11]);
        assert_eq!(fsm.pair_setting_flag(), PairSetState::PairSetMeshRxDone);

        now += INTERVAL_TICKS + 1;

        // Switch to the new mesh at once.
        let action = fsm.interval_tick(now).expect("the switch is due");
        let MeshPairAction::Broadcast(op_para) = action else {
            panic!("the switch arm returns Broadcast");
        };
        assert_eq!(op_para[0], LGT_CMD_MESH_PAIR);
        assert_eq!(op_para[1], 0);
        assert_eq!(op_para[2], 0);
        assert_eq!(op_para[3], MeshPairState::MeshPairEffectDelay as u8);
        assert_eq!(&op_para[4..16], &[0; 12]);
        assert_eq!(fsm.pair_setting_flag(), PairSetState::PairSetted);
        assert_eq!(fsm.mesh_pair_state, MeshPairState::MeshPairName1);
        assert_eq!(fsm.mesh_pair_start_time, 0, "the switch closes the window");

        // And the machine is quiet again.
        assert_eq!(fsm.interval_tick(now + INTERVAL_TICKS + 1), None);
    }

    /// Nothing is due before the command interval elapses, and nothing is sent
    /// while the pairing flag is below `PairSetMeshTxStart`.
    #[test]
    fn test_interval_tick_waits_for_the_interval_and_the_flag() {
        let mut fsm = armed_fsm();
        fsm.mesh_pair_time = 1_000;

        // Flag below TxStart: never broadcasts, whatever the tick.
        fsm.sync_flag(PairSetState::PairSetted);
        assert_eq!(fsm.interval_tick(1_000 + INTERVAL_TICKS + 1), None);
        fsm.sync_flag(PairSetState::PairSetting);
        assert_eq!(fsm.interval_tick(1_000 + INTERVAL_TICKS + 1), None);
        assert_eq!(fsm.mesh_pair_time, 1_000, "a no-op leaves the stamp alone");

        // Armed, but the interval has not elapsed yet.
        fsm.sync_flag(PairSetState::PairSetMeshTxStart);
        assert_eq!(fsm.interval_tick(1_000 + INTERVAL_TICKS), None);
        assert_eq!(fsm.mesh_pair_time, 1_000);

        // One tick past the interval is due.
        assert!(fsm.interval_tick(1_000 + INTERVAL_TICKS + 1).is_some());
    }

    /// A state the transmit arms do not know cancels the sequence and settles
    /// the flag, which is how a device that only receives ending up armed.
    #[test]
    fn test_interval_tick_cancels_on_a_receive_only_state() {
        let mut fsm = armed_fsm();
        fsm.mesh_pair_start_time = 5_000;
        fsm.mesh_pair_time = 1_000;
        fsm.mesh_pair_state = MeshPairState::MeshPairEffect;
        fsm.sync_flag(PairSetState::PairSetMeshTxStart);

        assert_eq!(fsm.interval_tick(1_000 + INTERVAL_TICKS + 1), None);
        assert_eq!(fsm.mesh_pair_state, MeshPairState::MeshPairName1);
        assert_eq!(fsm.mesh_pair_start_time, 0);
        assert_eq!(fsm.pair_setting_flag(), PairSetState::PairSetted);
    }

    // --- timed_out ---

    /// The pairing window is 10 s wide, measured with the wrapping comparison.
    #[test]
    fn test_timed_out_before_at_and_after_the_window() {
        let mut fsm = armed_fsm();
        fsm.mesh_pair_start_time = 1_000;

        assert!(
            !fsm.timed_out(1_000),
            "at the reference nothing has elapsed"
        );
        assert!(
            !fsm.timed_out(1_000 + MESH_PAIR_TIMEOUT * SECOND_TICKS - 1),
            "one tick short of the window is still open"
        );
        assert!(
            !fsm.timed_out(1_000 + MESH_PAIR_TIMEOUT * SECOND_TICKS),
            "the comparison is strictly greater"
        );
        assert!(
            fsm.timed_out(1_000 + MESH_PAIR_TIMEOUT * SECOND_TICKS + 1),
            "one tick past the window has expired"
        );

        // Across the 32-bit roll-over the elapsed span still measures.
        fsm.mesh_pair_start_time = u32::MAX - 10;
        assert!(fsm.timed_out(10_000 + MESH_PAIR_TIMEOUT * SECOND_TICKS));
    }

    /// A closed window (zero start time) never times out.
    #[test]
    fn test_timed_out_is_false_without_a_window() {
        let fsm = armed_fsm();
        assert_eq!(fsm.mesh_pair_start_time, 0);
        assert!(!fsm.timed_out(u32::MAX));
    }

    // --- timer_step ---

    /// Nothing armed: nothing to do.
    #[test]
    fn test_timer_step_idle() {
        let mut fsm = armed_fsm();
        assert_eq!(fsm.timer_step(u32::MAX), MeshPairAction::None);
    }

    /// The default-mesh delay expiring with a zero delay reloads the saved key.
    #[test]
    fn test_timer_step_reloads_the_saved_key() {
        let mut fsm = armed_fsm();
        fsm.default_mesh_effect_delay_ref = 1_000;
        fsm.default_mesh_time = 0;

        // Not due yet.
        assert_eq!(fsm.timer_step(1_000 + INTERVAL_TICKS), MeshPairAction::None);
        assert_eq!(fsm.default_mesh_effect_delay_ref, 1_000);

        let now = 1_000 + INTERVAL_TICKS + 1;
        assert_eq!(fsm.timer_step(now), MeshPairAction::ReloadSavedKey);
        assert_eq!(fsm.default_mesh_effect_delay_ref, 0);
        assert_eq!(fsm.default_mesh_time_ref, 0);
    }

    /// The same delay with a non-zero default-mesh window switches to it.
    #[test]
    fn test_timer_step_switches_to_the_default_mesh() {
        let mut fsm = armed_fsm();
        fsm.default_mesh_effect_delay_ref = 1_000;
        fsm.default_mesh_time = 5_000;

        assert_eq!(
            fsm.timer_step(1_000 + INTERVAL_TICKS + 1),
            MeshPairAction::SwitchToDefaultMesh { delay_s: 5 }
        );
        assert_eq!(fsm.default_mesh_effect_delay_ref, 0);
    }

    /// A default-mesh window timing out reloads the saved key.
    #[test]
    fn test_timer_step_default_mesh_window_times_out() {
        let mut fsm = armed_fsm();
        fsm.default_mesh_time_ref = 1_000;
        fsm.default_mesh_time = 5_000;

        assert_eq!(
            fsm.timer_step(1_000 + 5_000 * 1000 * CLOCK_SYS_CLOCK_1US),
            MeshPairAction::None,
            "the comparison is strictly greater"
        );
        assert_eq!(
            fsm.timer_step(1_000 + 5_000 * 1000 * CLOCK_SYS_CLOCK_1US + 1),
            MeshPairAction::ReloadSavedKey
        );
        assert_eq!(fsm.default_mesh_time_ref, 0);
    }

    /// `255000` is the "keep waiting" marker: the window restarts at `now`
    /// instead of expiring, so a device stays on the default mesh indefinitely.
    #[test]
    fn test_timer_step_255s_window_refreshes_the_reference() {
        let mut fsm = armed_fsm();
        fsm.default_mesh_time_ref = 1_000;
        fsm.default_mesh_time = 255_000;

        // 255 s is longer than the documented `CLOCK_MAX_US`, so the tick span
        // wraps through 32 bits; the comparison handles that by construction.
        let now = 1_000u32
            .wrapping_add((255_000u32 * 1000).wrapping_mul(CLOCK_SYS_CLOCK_1US))
            .wrapping_add(1);
        assert_eq!(fsm.timer_step(now), MeshPairAction::None);
        assert_eq!(fsm.default_mesh_time_ref, now | 1, "refreshed, not cleared");
        assert_eq!(fsm.default_mesh_time, 255_000);
    }

    // --- effect_action ---

    /// Nothing armed and nothing pending: no action.
    #[test]
    fn test_effect_action_not_due() {
        let fsm = armed_fsm();
        assert_eq!(fsm.effect_action(0, false), None);
    }

    /// The delayed-effect window is `mesh_pair_cmd_interval` wide.
    #[test]
    fn test_effect_action_waits_for_the_delay_window() {
        let mut fsm = armed_fsm();
        fsm.accumulate_effect_delay(1_000);
        assert_eq!(fsm.effect_new_mesh_delay_time, 1_001, "the stamp is | 1");

        assert_eq!(fsm.effect_action(1_000 + INTERVAL_TICKS, false), None);
        assert!(fsm
            .effect_action(1_001 + INTERVAL_TICKS + 1, false)
            .is_some());
    }

    /// The immediate-effect command skips the window.
    #[test]
    fn test_effect_action_is_immediate_after_the_effect_command() {
        let mut fsm = armed_fsm();
        fsm.accumulate_effect();
        assert_eq!(fsm.effect_new_mesh, 1);
        assert_eq!(
            fsm.effect_action(0, false),
            Some(MeshPairAction::SaveAndApply {
                name: [0; 16],
                pwd: [0; 16],
                ltk: [0; 16],
                use_pair_ltk: true,
            }),
            "the immediate arm reuses the mesh LTK that is already paired"
        );
    }

    /// A default-mesh window and a pending address validation both mean "do not
    /// save the new credentials yet".
    #[test]
    fn test_effect_action_reloads_instead_of_saving() {
        let mut fsm = armed_fsm();
        fsm.accumulate_effect();

        fsm.default_mesh_time_ref = 1;
        assert_eq!(
            fsm.effect_action(0, false),
            Some(MeshPairAction::ReloadSavedKey)
        );
        fsm.default_mesh_time_ref = 0;

        assert_eq!(
            fsm.effect_action(0, true),
            Some(MeshPairAction::ReloadSavedKey)
        );
    }

    // --- accumulation: all nine wire commands, in order ---

    /// Every wire command in order: the six halves land in the right place,
    /// then the delayed-effect window opens and the machine hands out the exact
    /// payload the flash path must persist.
    #[test]
    fn test_all_nine_commands_in_order_end_in_save_and_apply() {
        let mut fsm = armed_fsm();
        let now = 4_000;

        assert_eq!(
            fsm.accumulate_name(MeshPairState::MeshPairName1, &chunk_of(&NAME, false), now),
            MeshPairAction::None
        );
        assert_eq!(
            fsm.accumulate_name(MeshPairState::MeshPairName2, &chunk_of(&NAME, true), now),
            MeshPairAction::None
        );
        assert_eq!(
            fsm.accumulate_pwd(MeshPairState::MeshPairPwd1, &chunk_of(&PWD, false)),
            MeshPairAction::None
        );
        assert_eq!(
            fsm.accumulate_pwd(MeshPairState::MeshPairPwd2, &chunk_of(&PWD, true)),
            MeshPairAction::None
        );
        assert_eq!(
            fsm.accumulate_ltk(MeshPairState::MeshPairLtk1, &chunk_of(&LTK, false)),
            MeshPairAction::None
        );
        assert_eq!(
            fsm.accumulate_ltk(MeshPairState::MeshPairLtk2, &chunk_of(&LTK, true)),
            MeshPairAction::None
        );

        // Receiving a chunk is not a save: nothing is due before the window.
        assert_eq!(fsm.effect_action(now, false), None);
        assert_eq!(fsm.new_mesh_name, NAME);
        assert_eq!(fsm.new_mesh_pwd, PWD);
        assert_eq!(fsm.new_mesh_ltk, LTK);

        // Command 7: delayed effect.
        fsm.accumulate_effect_delay(now);
        assert_eq!(fsm.effect_new_mesh_delay_time, now | 1);
        assert_eq!(fsm.effect_new_mesh, 0);

        assert!(fsm
            .effect_action(
                (now | 1) + MESH_PAIR_CMD_INTERVAL * 1000 * CLOCK_SYS_CLOCK_1US + 1,
                false
            )
            .is_some());

        // Command 8 is the receive-only immediate effect; command 9 the default
        // mesh window. Both only move their own timers.
        fsm.accumulate_effect();
        fsm.accumulate_default_mesh(0, now);
        assert_eq!(fsm.effect_new_mesh, 1);
        fsm.accumulate_default_mesh(5, now);
        assert_eq!(fsm.default_mesh_effect_delay_ref, now | 1);
        assert_eq!(fsm.default_mesh_time, 5_000);

        // With no default-mesh window the credentials are saved as received.
        fsm.default_mesh_time_ref = 0;
        assert_eq!(
            fsm.effect_action(now, false),
            Some(MeshPairAction::SaveAndApply {
                name: NAME,
                pwd: PWD,
                ltk: LTK,
                use_pair_ltk: true,
            })
        );

        // Clearing the immediate flag re-uses the received LTK instead.
        fsm.effect_new_mesh = 0;
        assert_eq!(
            fsm.effect_action(now, false),
            Some(MeshPairAction::SaveAndApply {
                name: NAME,
                pwd: PWD,
                ltk: LTK,
                use_pair_ltk: false,
            })
        );
    }

    /// A chunk the mesh relay delivers twice rewrites its own half only.
    #[test]
    fn test_accumulation_survives_a_duplicated_chunk() {
        let mut fsm = armed_fsm();
        let first = chunk_of(&NAME, false);
        let second = chunk_of(&NAME, true);

        fsm.accumulate_name(MeshPairState::MeshPairName1, &first, 1_000);
        fsm.accumulate_name(MeshPairState::MeshPairName2, &second, 1_000);
        // The relay repeats the second command.
        fsm.accumulate_name(MeshPairState::MeshPairName2, &second, 1_000);
        assert_eq!(fsm.new_mesh_name, NAME, "the first half was not clobbered");

        // And a repeated first command leaves the second half alone.
        fsm.accumulate_pwd(MeshPairState::MeshPairPwd1, &chunk_of(&PWD, false));
        fsm.accumulate_pwd(MeshPairState::MeshPairPwd1, &chunk_of(&PWD, false));
        assert_eq!(&fsm.new_mesh_pwd[0..8], &PWD[0..8]);
        assert_eq!(&fsm.new_mesh_pwd[8..16], &[0u8; 8]);
    }

    /// An accumulator handed a state that belongs to another credential leaves
    /// every buffer alone, and the const constructor and `Default` agree.
    #[test]
    fn test_mismatched_accumulator_states_are_ignored() {
        let mut fsm = MeshPairFsm::default();
        assert_eq!(
            fsm.pair_setting_flag(),
            MeshPairFsm::new().pair_setting_flag()
        );

        assert_eq!(
            fsm.accumulate_name(MeshPairState::MeshPairPwd1, &[0xFF; 8], 1),
            MeshPairAction::None
        );
        assert_eq!(
            fsm.accumulate_pwd(MeshPairState::MeshPairLtk1, &[0xFF; 8]),
            MeshPairAction::None
        );
        assert_eq!(
            fsm.accumulate_ltk(MeshPairState::MeshPairName1, &[0xFF; 8]),
            MeshPairAction::None
        );

        assert_eq!(fsm.new_mesh_name, [0; 16]);
        assert_eq!(fsm.new_mesh_pwd, [0; 16]);
        assert_eq!(fsm.new_mesh_ltk, [0; 16]);
        assert_eq!(
            fsm.mesh_pair_start_time, 0,
            "only the first name chunk opens the pairing window"
        );
    }

    /// A received `MeshPairDefaultMesh` while the default-mesh window is already
    /// open re-arms the switch with twice the command interval.
    #[test]
    fn test_effect_delay_extends_an_open_default_mesh_window() {
        let mut fsm = armed_fsm();
        fsm.default_mesh_time_ref = 1;
        fsm.default_mesh_time = 0;

        fsm.accumulate_effect_delay(2_000);
        assert_eq!(fsm.effect_new_mesh_delay_time, 2_001);
        assert_eq!(fsm.default_mesh_time, MESH_PAIR_CMD_INTERVAL * 2);

        // With the window closed the delay only stamps itself.
        let mut fsm = armed_fsm();
        fsm.accumulate_effect_delay(2_000);
        assert_eq!(fsm.default_mesh_time, 0);
    }
}
