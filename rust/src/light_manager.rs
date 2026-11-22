use core::cmp::min;
use core::mem::size_of;
use core::ptr::addr_of;

use const_format::formatcp;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant};
use fixed::types::{I16F16, U16F16};

use crate::config::{FLASH_ADR_LUM, FLASH_SECTOR_SIZE, MAX_LUM_BRIGHTNESS_VALUE, PWMID_B, PWMID_G};
use crate::mesh::MESH_NODE_ST_PAR_LEN;
use crate::sdk::ble_app::light_ll::mesh_management::ll_device_status_update;
use crate::sdk::common::sync::CriticalSectionMutex;
use crate::sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};
use crate::sdk::drivers::pwm::pwm_set_cmp;
use crate::sdk::light::{
    RecoverStatus, LGT_CMD_LIGHT_ONOFF, LGT_CMD_SET_LIGHT, LIGHT_OFF_PARAM, LIGHT_ON_PARAM,
    PMW_MAX_TICK,
};
use crate::sdk::mcu::analog::{analog_read, analog_write};
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed};
use crate::sdk::mcu::register::{
    read_reg_tmr_ctrl, write_reg_tmr1_tick, write_reg_tmr_ctrl, FLD_TMR,
};
use crate::sdk::pm::REGA_LIGHT_OFF;

const TRANSITION_TIME_MS: u64 = 1500;
const LIGHT_SAVE_VALID_FLAG: u8 = 0xA5;

/// Raw mutex backing the light command channel: masks interrupts for the
/// (short, allocation-free) critical section inside `embassy-sync`.
type CsRawMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

#[derive(Copy, Clone, Debug)]
pub struct Message {
    pub cmd: u8,
    pub params: [u8; 16],
}

#[derive(Copy, Clone, Debug)]
pub struct LightState {
    pub cw: I16F16,
    pub ww: I16F16,
    pub brightness: I16F16,
}

#[derive(Copy, Clone)]
#[repr(C, packed)]
pub(crate) struct LumSaveT {
    save_flag: u8,
    brightness: u16,
    cw: u16,
    ww: u16,
}

impl LightState {
    #[cfg(test)]
    pub fn default() -> Self {
        Self {
            cw: I16F16::lit(formatcp!("{}", MAX_LUM_BRIGHTNESS_VALUE)),
            ww: I16F16::lit(formatcp!("{}", 0u16)),
            brightness: I16F16::lit(formatcp!("{}", 0u16)),
        }
    }
}

fn ease_in_out(t: I16F16, b: I16F16, c: I16F16) -> I16F16 {
    static TWO: I16F16 = I16F16::lit(formatcp!("{}", 2u16));
    static D: I16F16 = I16F16::lit(formatcp!("{}", MAX_LUM_BRIGHTNESS_VALUE));

    let t = t / (D / TWO);
    if t < 1 {
        c / TWO * (t * t * t) + b
    } else {
        let t = t - TWO;
        c / TWO * (t * t * t + TWO) + b
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct Transition {
    from: I16F16,
    current: I16F16,
    to: I16F16,
    start: Instant,
    end: Instant,
}

impl Transition {
    const fn new(initial: I16F16) -> Self {
        Self {
            from: initial,
            current: initial,
            to: initial,
            start: Instant::from_ticks(0),
            end: Instant::from_ticks(0),
        }
    }

    fn begin_at(&mut self, start: Instant, to: I16F16) {
        // Same-target guard: if already heading to this target, don't restart the transition.
        if to == self.to {
            return;
        }
        self.from = self.current;
        self.start = start;
        self.end = start + Duration::from_millis(TRANSITION_TIME_MS);
        self.to = to;
    }

    fn step_at(&mut self, now: Instant) -> bool {
        if now >= self.end {
            self.current = self.to;
            return false;
        }
        let elapsed = (now - self.start).as_ticks();
        let total = (self.end - self.start).as_ticks();
        let t = I16F16::from_num(elapsed * MAX_LUM_BRIGHTNESS_VALUE as u64 / total);
        self.current = ease_in_out(t, self.from, self.to - self.from);
        true
    }

    /// Returns true if a transition was set up (end > start), regardless of
    /// whether it has completed. After completion current == to, so callers
    /// that use this to choose between `to` and `current` get the same value.
    fn is_active(&self) -> bool {
        self.end > self.start
    }
}

/// The lamp's transition state: the only owner of the five correlated fields
/// shared by the TMR1 ISR, the light task and the main loop.
///
/// Every field is updated together (a transition step touches all three
/// `Transition`s plus `brightness`), so they live behind one
/// [`CriticalSectionMutex`] instead of per-field atomics that would tear the
/// easing maths mid-step.
pub struct LightTransitionCore {
    brightness: u16,
    brightness_tr: Transition,
    cw_tr: Transition,
    ww_tr: Transition,
    last_transition_time: u32,
}

impl LightTransitionCore {
    /// Default (lights off, cool-white) core state. `const` so it can seed the
    /// [`LIGHT_HW`] static.
    const fn new() -> Self {
        Self {
            brightness: MAX_LUM_BRIGHTNESS_VALUE,
            brightness_tr: Transition::new(I16F16::lit(formatcp!("{}", 0u16))),
            cw_tr: Transition::new(I16F16::lit(formatcp!("{}", MAX_LUM_BRIGHTNESS_VALUE))),
            ww_tr: Transition::new(I16F16::lit(formatcp!("{}", 0u16))),
            last_transition_time: 0,
        }
    }

    fn handle_on_off(&mut self, on: u8) {
        self.light_onoff_inner(match on {
            LIGHT_ON_PARAM => true,
            LIGHT_OFF_PARAM => false,
            _ => true,
        });
    }

    fn handle_transition(&mut self, params: &[u8; 16]) {
        let mut brightness = self.brightness;
        let mut cw = self.cw_tr.current.to_num();
        let mut ww = self.ww_tr.current.to_num();

        if params[8] & 0x1 != 0 {
            // Brightness
            brightness = (params[1] as u16) << 8 | params[0] as u16;

            // If we're really changing the brightness, then we should save it soon
            if self.brightness != brightness {
                self.last_transition_time = clock_time();
            }

            self.brightness = brightness
        }

        if params[8] & 0x2 != 0 {
            // Temperature
            let value = (params[3] as u16) << 8 | params[2] as u16;

            cw = MAX_LUM_BRIGHTNESS_VALUE - value;
            ww = value;
        }

        if params[8] & 0x4 != 0 {
            // Temperature (independent CW/WW)
            cw = (params[3] as u16) << 8 | params[2] as u16;
            ww = (params[5] as u16) << 8 | params[4] as u16;
        }

        if self.is_light_off() {
            self.begin_transition_inner(cw, ww, 0);
        } else {
            self.begin_transition_inner(cw, ww, brightness);
        }
    }

    /// Start a transition to `(cw, ww, brightness)`.
    ///
    /// Must be called with [`LIGHT_HW`] already held: it calls
    /// `transition_step_inner` directly instead of the public
    /// `transition_step()`, which would re-lock the mutex and panic under the
    /// `cfg(test)` re-entrancy probe (issue #40).
    fn begin_transition_inner(&mut self, cw: u16, ww: u16, brightness: u16) {
        // Check if the cw or ww targets are changing and mark for save
        if self.cw_tr.to != cw || self.ww_tr.to != ww {
            self.last_transition_time = clock_time();
        }

        let now = Instant::now();
        self.brightness_tr
            .begin_at(now, I16F16::from_num(brightness));
        self.cw_tr.begin_at(now, I16F16::from_num(cw));
        self.ww_tr.begin_at(now, I16F16::from_num(ww));

        // Enable timer1
        write_reg_tmr1_tick(0);
        write_reg_tmr_ctrl(read_reg_tmr_ctrl() | FLD_TMR::TMR1_EN.bits());

        // Run a single transition now to avoid anything bugging out.
        self.transition_step_inner();

        // `begin_at` above made `brightness` the transition target, so the
        // on/off state is exactly `brightness == 0`.
        crate::sdk::pm::set_reboot_light_off_state(brightness == 0);
    }

    fn transition_step_inner(&mut self) {
        let now = Instant::now();
        let b = self.brightness_tr.step_at(now);
        let c = self.cw_tr.step_at(now);
        let w = self.ww_tr.step_at(now);

        if !b && !c && !w {
            write_reg_tmr_ctrl(read_reg_tmr_ctrl() & !FLD_TMR::TMR1_EN.bits());
        }

        light_adjust_rgb_hw_unlocked(
            self.cw_tr.current,
            self.ww_tr.current,
            self.brightness_tr.current,
        );
    }

    fn light_onoff_inner(&mut self, on: bool) {
        let target = if on { self.brightness } else { 0 };
        self.brightness_tr
            .begin_at(Instant::now(), I16F16::from_num(target));
        write_reg_tmr1_tick(0);
        write_reg_tmr_ctrl(read_reg_tmr_ctrl() | FLD_TMR::TMR1_EN.bits());
        self.transition_step_inner();
        // `begin_at` above made `target` the transition target, so the
        // on/off state is exactly `target == 0`.
        crate::sdk::pm::set_reboot_light_off_state(target == 0);
    }

    fn is_light_off(&self) -> bool {
        if self.brightness_tr.is_active() {
            self.brightness_tr.to == 0
        } else {
            self.brightness_tr.current == 0
        }
    }
}

/// Single owner of the lamp transition state.
pub static LIGHT_HW: CriticalSectionMutex<LightTransitionCore> =
    CriticalSectionMutex::new(LightTransitionCore::new());

/// Commands handed from the BLE stack / main loop to [`light_manager_task`].
pub static LIGHT_CMD_CHANNEL: Channel<CsRawMutex, Message, 10> = Channel::new();

fn calculate_lumen_map(val: I16F16) -> u32 {
    static COEFF1: U16F16 = U16F16::lit("5.2221");
    static COEFF2: U16F16 = U16F16::lit("130.5908");

    let percentage = U16F16::from_num(val) / MAX_LUM_BRIGHTNESS_VALUE as u32 * 100;
    ((COEFF1 * (percentage * percentage)) + COEFF2 * percentage).to_num()
}

fn pwm_set_lum(id: u32, y: u16, pol: bool) {
    let lum = (y as u32 * PMW_MAX_TICK as u32) / (255 * 256) as u32;

    pwm_set_cmp(id, if pol { PMW_MAX_TICK as u32 - lum } else { lum } as u16);
}

fn get_pwm_cmp(val: I16F16, lum: I16F16) -> u16 {
    let val_lumen_map = calculate_lumen_map(lum);

    ((val.to_num::<u32>() * val_lumen_map) / MAX_LUM_BRIGHTNESS_VALUE as u32) as u16
}

/// Set the cool-white PWM without taking [`LIGHT_HW`]; the caller holds it.
fn light_adjust_cw_unlocked(val: I16F16, lum: I16F16) {
    pwm_set_lum(PWMID_G, get_pwm_cmp(val, lum), false);
}

/// Set the warm-white PWM without taking [`LIGHT_HW`]; the caller holds it.
fn light_adjust_ww_unlocked(val: I16F16, lum: I16F16) {
    pwm_set_lum(PWMID_B, get_pwm_cmp(val, lum), true);
}

/// Drive both PWM channels without taking [`LIGHT_HW`]; the caller holds it.
fn light_adjust_rgb_hw_unlocked(val_cw: I16F16, val_ww: I16F16, lum: I16F16) {
    light_adjust_cw_unlocked(val_cw, lum);
    light_adjust_ww_unlocked(val_ww, lum);
}

/// Step every active transition and push the result to the PWM hardware.
#[cfg_attr(test, mry::mry)]
pub fn transition_step() {
    LIGHT_HW.lock().transition_step_inner();
}

/// Start a transition to `(cw, ww, brightness)`.
///
/// Only host tests drive this directly today; it stays part of the lock-once
/// API so a caller cannot reach `begin_transition_inner` without the lock.
#[allow(dead_code)]
pub fn begin_transition(cw: u16, ww: u16, brightness: u16) {
    LIGHT_HW.lock().begin_transition_inner(cw, ww, brightness);
}

/// Set the cool-white PWM duty. Locks [`LIGHT_HW`] once so the paired CW/WW
/// writes cannot interleave with a TMR1 transition step.
#[cfg_attr(test, mry::mry)]
pub fn light_adjust_cw(val: I16F16, lum: I16F16) {
    let _guard = LIGHT_HW.lock();
    light_adjust_cw_unlocked(val, lum);
}

/// Set the warm-white PWM duty. Locks [`LIGHT_HW`] once so the paired CW/WW
/// writes cannot interleave with a TMR1 transition step.
#[cfg_attr(test, mry::mry)]
pub fn light_adjust_ww(val: I16F16, lum: I16F16) {
    let _guard = LIGHT_HW.lock();
    light_adjust_ww_unlocked(val, lum);
}

/// Drive both PWM channels. Locks [`LIGHT_HW`] once for the pair.
///
/// The animation helpers that used this (`lib.rs::blinken`) are gone, so only
/// tests call it today; it stays part of the lock-once colour API.
#[allow(dead_code)]
pub fn light_adjust_rgb_hw(val_cw: I16F16, val_ww: I16F16, lum: I16F16) {
    let _guard = LIGHT_HW.lock();
    light_adjust_rgb_hw_unlocked(val_cw, val_ww, lum);
}

/// Turn the lamp hardware on or off (no mesh status report).
#[cfg_attr(test, mry::mry)]
pub fn light_onoff_hw(on: bool) {
    LIGHT_HW.lock().light_onoff_inner(on);
}

/// Turn the lamp on or off and report the new status to the mesh.
pub fn light_onoff(on: bool) {
    light_onoff_hw(on);
    device_status_update();
}

/// Whether the lamp is off, taking the pending transition target into account.
#[cfg_attr(test, mry::mry)]
pub fn is_light_off() -> bool {
    LIGHT_HW.lock().is_light_off()
}

/// Current (or target, while a transition is in flight) light state.
pub fn get_current_light_state() -> LightState {
    let core = LIGHT_HW.lock();
    LightState {
        cw: if core.cw_tr.is_active() {
            core.cw_tr.to
        } else {
            core.cw_tr.current
        },
        ww: if core.ww_tr.is_active() {
            core.ww_tr.to
        } else {
            core.ww_tr.current
        },
        brightness: if core.brightness_tr.is_active() {
            core.brightness_tr.to
        } else {
            core.brightness_tr.current
        },
    }
}

/// Report the current on/off state to the mesh status buffer.
#[cfg_attr(test, mry::mry)]
pub fn device_status_update() {
    // Snapshot the on/off state under the lock; the mesh notification below
    // must not run inside the critical section.
    let on = {
        let core = LIGHT_HW.lock();
        if core.is_light_off() {
            0
        } else {
            1
        }
    };

    // packet
    let mut st_val_par: [u8; MESH_NODE_ST_PAR_LEN] = [0xff; MESH_NODE_ST_PAR_LEN];

    st_val_par[0] = on;
    st_val_par[1] = 0xff;

    ll_device_status_update(&st_val_par);
}

/// Queue a command for [`light_manager_task`]. Dropped if the queue is full.
pub fn send_light_message(cmd: u8, params: [u8; 16]) {
    let _ = LIGHT_CMD_CHANNEL.try_send(Message { cmd, params });
}

/// Apply one queued command. Each arm takes [`LIGHT_HW`] once; the status
/// report is a separate lock-once call, so no guard is ever held twice.
fn dispatch_message(msg: Message) {
    match msg.cmd {
        LGT_CMD_LIGHT_ONOFF => {
            LIGHT_HW.lock().handle_on_off(msg.params[0]);
            device_status_update();
        }
        LGT_CMD_SET_LIGHT => LIGHT_HW.lock().handle_transition(&msg.params),
        _ => {}
    }
}

/// The one owner of the light command queue.
#[coverage(off)]
pub async fn light_manager_task() {
    loop {
        let msg = LIGHT_CMD_CHANNEL.receive().await;
        dispatch_message(msg);
    }
}

/// The lamp's persisted-brightness write pointer.
///
/// Owned by value by the main task and passed down as `&mut`; every flash call
/// happens outside [`LIGHT_HW`].
pub struct LightFlashStore {
    light_lum_addr: u32,
}

impl LightFlashStore {
    /// Scan the luminance sector for the last saved state, apply it and restore
    /// the lamp on/off state.
    ///
    /// All `flash_read_page` calls happen before/without the [`LIGHT_HW`] lock;
    /// the lock is taken exactly once, only to apply the loaded values.
    pub fn retrieve_from_flash() -> Self {
        // Read flash sector in chunks and scan for saved light state
        const ENTRIES_PER_CHUNK: usize = 10;
        let mut store = Self {
            light_lum_addr: FLASH_ADR_LUM,
        };
        let mut buffer = [LumSaveT {
            save_flag: 0xFF,
            brightness: 0,
            cw: 0,
            ww: 0,
        }; ENTRIES_PER_CHUNK];
        let mut loaded: Option<(u16, u16, u16)> = None;

        // Scan the entire sector in chunks
        'scan: for offset in
            (0..FLASH_SECTOR_SIZE).step_by(size_of::<LumSaveT>() * ENTRIES_PER_CHUNK)
        {
            let addr = FLASH_ADR_LUM + offset as u32;
            flash_read_page(
                addr,
                size_of::<LumSaveT>() as u32 * ENTRIES_PER_CHUNK as u32,
                buffer.as_mut_ptr() as *mut u8,
            );

            for (idx, entry) in buffer.iter().enumerate() {
                match entry.save_flag {
                    LIGHT_SAVE_VALID_FLAG => {
                        // Found valid saved state - remember it and update the
                        // address for the next write.
                        loaded = Some((
                            min(entry.brightness, MAX_LUM_BRIGHTNESS_VALUE),
                            min(entry.cw, MAX_LUM_BRIGHTNESS_VALUE),
                            min(entry.ww, MAX_LUM_BRIGHTNESS_VALUE),
                        ));
                        store.light_lum_addr = addr + (idx * size_of::<LumSaveT>()) as u32;
                    }
                    0xFF => {
                        // Reached end of written data - set address for next write
                        store.light_lum_addr = addr + (idx * size_of::<LumSaveT>()) as u32;
                        break 'scan;
                    }
                    _ => {
                        // Invalid entry, skip
                    }
                }
            }
        }

        if let Some((brightness, cw, ww)) = loaded {
            let mut core = LIGHT_HW.lock();
            core.brightness = brightness;
            let cw = I16F16::from_num(cw);
            let ww = I16F16::from_num(ww);
            // Set from/current/to all to saved value so colour starts
            // immediately at the saved colour with no fade on boot.
            core.cw_tr.from = cw;
            core.cw_tr.current = cw;
            core.cw_tr.to = cw;
            core.ww_tr.from = ww;
            core.ww_tr.current = ww;
            core.ww_tr.to = ww;
        }

        Self::restore_onoff_state();
        store
    }

    /// Restore the lamp on/off state recorded in analog memory.
    fn restore_onoff_state() {
        let val = analog_read(REGA_LIGHT_OFF);
        if val & RecoverStatus::LightOff as u8 != 0 {
            analog_write(REGA_LIGHT_OFF, val & !(RecoverStatus::LightOff as u8));
            light_onoff(false);
        } else {
            light_onoff(true);
        }
    }

    //erase flash
    fn light_lum_erase(&mut self) {
        self.light_lum_addr = FLASH_ADR_LUM;
        flash_erase_sector(FLASH_ADR_LUM);
    }

    //save cur lum value, if disconnected for a while
    pub fn light_state_save(&mut self) {
        if self.light_lum_addr
            >= (FLASH_ADR_LUM + FLASH_SECTOR_SIZE as u32 - size_of::<LumSaveT>() as u32)
        {
            self.light_lum_erase();
        }

        // Snapshot the values under a short lock; the flash write below must
        // not run inside a critical section.
        let (brightness, cw, ww): (u16, u16, u16) = {
            let core = LIGHT_HW.lock();
            (
                core.brightness,
                core.cw_tr.current.to_num(),
                core.ww_tr.current.to_num(),
            )
        };

        let lum_save = LumSaveT {
            save_flag: LIGHT_SAVE_VALID_FLAG,
            brightness,
            cw,
            ww,
        };

        flash_write_page(
            self.light_lum_addr,
            size_of::<LumSaveT>() as u32,
            addr_of!(lum_save) as *const u8,
        );

        self.light_lum_addr += size_of::<LumSaveT>() as u32;
    }

    pub fn check_light_state_save(&mut self) {
        // Save the light state if it's been more than 5 seconds since the last
        // transition. The timer is cleared under the lock; the guard is dropped
        // before any flash erase/write happens.
        let due = {
            let mut core = LIGHT_HW.lock();
            if core.last_transition_time != 0
                && clock_time_exceed(core.last_transition_time, 5000 * 1000)
            {
                core.last_transition_time = 0;
                true
            } else {
                false
            }
        };

        if due {
            self.light_state_save();
        }
    }
}

#[cfg(test)]
impl Default for LightFlashStore {
    fn default() -> Self {
        Self { light_lum_addr: 0 }
    }
}

/// Restore the global light state to its power-on defaults and drain the
/// command queue. Host tests are single-threaded, so this is enough isolation.
#[cfg(test)]
pub fn reset_for_test() {
    *LIGHT_HW.lock() = LightTransitionCore::new();
    while LIGHT_CMD_CHANNEL.try_receive().is_ok() {}
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::sdk::ble_app::light_ll::mesh_management::mock_ll_device_status_update;
    use crate::sdk::drivers::flash::{
        mock_flash_erase_sector, mock_flash_read_page, mock_flash_write_page,
    };
    use crate::sdk::drivers::pwm::mock_pwm_set_cmp;
    use crate::sdk::mcu::analog::{mock_analog_read, mock_analog_write};
    use crate::sdk::mcu::clock::{clock_time64, mock_clock_time64};
    use crate::sdk::mcu::clock::{mock_clock_time, mock_clock_time_exceed};
    use crate::sdk::mcu::register::{
        mock_read_reg_tmr_ctrl, mock_write_reg_tmr1_tick, mock_write_reg_tmr_ctrl,
    };
    use embassy_time::Instant;
    use mry::send_wrapper::SendWrapper;
    use mry::Any;

    // --- Helper Functions ---

    /// Run `f` with exclusive access to the global light transition core.
    ///
    /// The closure must only touch fields: calling a free function that locks
    /// `LIGHT_HW` from inside would trip the re-entrancy probe.
    fn with_core<R>(f: impl FnOnce(&mut LightTransitionCore) -> R) -> R {
        let mut core = LIGHT_HW.lock();
        f(&mut core)
    }

    /// Create test message for on/off command
    fn create_onoff_message(on: u8) -> Message {
        Message {
            cmd: LGT_CMD_LIGHT_ONOFF,
            params: [on, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        }
    }

    /// Create test message for light transition with brightness
    fn create_brightness_message(brightness: u16) -> Message {
        let brightness_bytes = brightness.to_le_bytes();
        Message {
            cmd: LGT_CMD_SET_LIGHT,
            params: [
                brightness_bytes[0],
                brightness_bytes[1],
                0,
                0,
                0,
                0,
                0,
                0,
                0x1, // Brightness flag
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            ],
        }
    }

    /// Create test message for temperature change
    fn create_temperature_message(temperature: u16) -> Message {
        let temp_bytes = temperature.to_le_bytes();
        Message {
            cmd: LGT_CMD_SET_LIGHT,
            params: [
                0,
                0,
                temp_bytes[0],
                temp_bytes[1],
                0,
                0,
                0,
                0,
                0x2, // Temperature flag
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            ],
        }
    }

    /// Create test message for independent CW/WW control
    #[allow(dead_code)]
    fn create_cw_ww_message(cw: u16, ww: u16) -> Message {
        let cw_bytes = cw.to_le_bytes();
        let ww_bytes = ww.to_le_bytes();
        Message {
            cmd: LGT_CMD_SET_LIGHT,
            params: [
                0,
                0,
                cw_bytes[0],
                cw_bytes[1],
                ww_bytes[0],
                ww_bytes[1],
                0,
                0,
                0x4, // Independent CW/WW flag
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            ],
        }
    }

    // --- Basic Initialization Tests ---

    #[test]
    fn test_light_manager_default() {
        // The global core comes up lights-off at cool white, the queue is empty
        // and the flash write pointer starts at zero.

        reset_for_test();

        assert!(LIGHT_CMD_CHANNEL.is_empty());
        with_core(|core| {
            assert_eq!(core.brightness, MAX_LUM_BRIGHTNESS_VALUE);
            assert_eq!(core.last_transition_time, 0);
            assert_eq!(core.brightness_tr.current.to_num::<u16>(), 0);
            assert_eq!(core.cw_tr.current.to_num::<u16>(), MAX_LUM_BRIGHTNESS_VALUE);
            assert_eq!(core.ww_tr.current.to_num::<u16>(), 0);
        });

        let store = LightFlashStore::default();
        assert_eq!(store.light_lum_addr, 0);
    }

    #[test]
    fn test_light_state_default() {
        // This test verifies LightState initializes with correct default values

        let state = LightState::default();

        assert_eq!(state.cw.to_num::<u16>(), MAX_LUM_BRIGHTNESS_VALUE);
        assert_eq!(state.ww.to_num::<u16>(), 0);
        assert_eq!(state.brightness.to_num::<u16>(), 0);
    }

    // --- Message Sending Tests ---

    #[test]
    fn test_send_message_adds_to_channel() {
        // This test verifies that send_light_message correctly adds messages to the channel

        reset_for_test();
        let test_params = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];

        send_light_message(LGT_CMD_LIGHT_ONOFF, test_params);

        assert_eq!(LIGHT_CMD_CHANNEL.len(), 1);
        let msg = LIGHT_CMD_CHANNEL.try_receive().unwrap();
        assert_eq!(msg.cmd, LGT_CMD_LIGHT_ONOFF);
        assert_eq!(msg.params, test_params);
    }

    #[test]
    fn test_send_message_does_not_overflow_channel() {
        // This test verifies that send_light_message doesn't add messages when channel is full

        reset_for_test();

        // Fill the channel (capacity is 10)
        for i in 0..10 {
            send_light_message(LGT_CMD_LIGHT_ONOFF, [i; 16]);
        }

        // Try to add one more - should be ignored
        send_light_message(LGT_CMD_LIGHT_ONOFF, [99; 16]);

        // All 10 retained messages are received in order and the 11th was dropped
        for i in 0..10 {
            assert_eq!(LIGHT_CMD_CHANNEL.try_receive().unwrap().params[0], i);
        }
        assert!(LIGHT_CMD_CHANNEL.try_receive().is_err());
    }

    // --- On/Off Handling Tests ---

    #[test]
    #[mry::lock(
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        ll_device_status_update,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_handle_on_off_turns_light_on() {
        // This test verifies that handle_on_off correctly processes LIGHT_ON_PARAM

        reset_for_test();
        with_core(|core| core.brightness = 500);

        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(0x08).returns(()); // Enable timer1
        mock_ll_device_status_update([1, 0xff]).returns(());
        // First transition_step call (at t=0, brightness=0)
        mock_pwm_set_cmp(0, 0).returns(()); // CW channel at brightness 0
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel (always 10455 for low brightness)
        mock_clock_time64().returns(1000);

        dispatch_message(create_onoff_message(LIGHT_ON_PARAM));

        // New state should target the saved brightness
        assert_eq!(with_core(|core| core.brightness_tr.to.to_num::<u16>()), 500);
    }

    #[test]
    #[mry::lock(
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        ll_device_status_update,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_handle_on_off_turns_light_off() {
        // This test verifies that handle_on_off correctly processes LIGHT_OFF_PARAM

        reset_for_test();
        with_core(|core| {
            core.brightness_tr.current = I16F16::from_num(1000);
            core.brightness_tr.to = I16F16::from_num(1000);
        });

        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(0x08).returns(()); // Enable timer1
        mock_ll_device_status_update([0, 0xff]).returns(()); // Light off status
                                                             // Initial transition_step at brightness=1000
        mock_pwm_set_cmp(0, 71).returns(()); // CW channel at brightness 1000
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel
        mock_clock_time64().returns(1000);

        dispatch_message(create_onoff_message(LIGHT_OFF_PARAM));

        // New state should target brightness 0
        assert_eq!(with_core(|core| core.brightness_tr.to.to_num::<u16>()), 0);
    }

    #[test]
    #[mry::lock(
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        ll_device_status_update,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_handle_on_off_invalid_param_defaults_to_on() {
        // This test verifies that handle_on_off treats invalid params as ON

        reset_for_test();
        with_core(|core| core.brightness = 300);

        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(0x08).returns(()); // Enable timer1
        mock_ll_device_status_update([1, 0xff]).returns(());
        // Initial transition_step at brightness=0
        mock_pwm_set_cmp(0, 0).returns(()); // CW channel at brightness 0
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel
        mock_clock_time64().returns(1000);

        dispatch_message(create_onoff_message(0x99)); // Invalid value

        assert_eq!(with_core(|core| core.brightness_tr.to.to_num::<u16>()), 300);
    }

    // --- Transition Handling Tests ---

    #[test]
    #[mry::lock(
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        clock_time,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_handle_transition_brightness_only() {
        // This test verifies that handle_transition correctly processes brightness changes

        reset_for_test();
        with_core(|core| {
            core.brightness = 100;
            core.brightness_tr.current = I16F16::from_num(100);
            core.brightness_tr.to = I16F16::from_num(100);
        });
        let params = [
            0x34, 0x12, // brightness = 0x1234
            0, 0, // temperature
            0, 0, // ww
            0, 0,   // reserved
            0x1, // brightness flag
            0, 0, 0, 0, 0, 0, 0,
        ];

        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(0x08).returns(()); // Enable timer1
        mock_clock_time().returns(1000);
        // Initial transition_step at brightness=100 (pwm_set_lum scales values down)
        mock_pwm_set_cmp(0, 6).returns(()); // CW channel at brightness 100
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel
        mock_clock_time64().returns(2000);

        LIGHT_HW.lock().handle_transition(&params);

        assert_eq!(with_core(|core| core.brightness), 0x1234);
        assert_eq!(
            with_core(|core| core.brightness_tr.to.to_num::<u16>()),
            0x1234
        );
        assert_eq!(with_core(|core| core.last_transition_time), 1000);
    }

    #[test]
    #[mry::lock(
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        pwm_set_cmp,
        clock_time,
        clock_time64
    )]
    fn test_handle_transition_temperature_only() {
        // This test verifies that handle_transition correctly processes temperature changes

        reset_for_test();
        with_core(|core| {
            core.brightness_tr.current = I16F16::from_num(500);
            core.brightness_tr.to = I16F16::from_num(500);
        });
        let params = [
            0, 0, // brightness
            0x78, 0x56, // temperature = 0x5678
            0, 0, // ww
            0, 0,   // reserved
            0x2, // temperature flag
            0, 0, 0, 0, 0, 0, 0,
        ];

        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(0x08).returns(()); // Enable timer1
                                                   // Initial transition_step at brightness=500, cw=MAX
        mock_pwm_set_cmp(0, 33).returns(()); // CW channel at brightness 500
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel
        mock_clock_time().returns(1000);
        mock_clock_time64().returns(2000);

        LIGHT_HW.lock().handle_transition(&params);

        // CW should be MAX - temperature, WW should be temperature
        assert_eq!(
            with_core(|core| core.cw_tr.to.to_num::<u16>()),
            MAX_LUM_BRIGHTNESS_VALUE - 0x5678
        );
        assert_eq!(with_core(|core| core.ww_tr.to.to_num::<u16>()), 0x5678);
    }

    #[test]
    #[mry::lock(
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        pwm_set_cmp,
        clock_time,
        clock_time64
    )]
    fn test_handle_transition_independent_cw_ww() {
        // This test verifies that handle_transition correctly processes independent CW/WW

        reset_for_test();
        with_core(|core| {
            core.brightness_tr.current = I16F16::from_num(500);
            core.brightness_tr.to = I16F16::from_num(500);
        });
        let params = [
            0, 0, // brightness
            0x34, 0x12, // cw = 0x1234
            0x78, 0x56, // ww = 0x5678
            0, 0,   // reserved
            0x4, // CW/WW flag
            0, 0, 0, 0, 0, 0, 0,
        ];

        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(0x08).returns(()); // Enable timer1
                                                   // Initial transition_step at brightness=500, cw=MAX
        mock_pwm_set_cmp(0, 33).returns(()); // CW channel at brightness 500
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel
        mock_clock_time().returns(1000);
        mock_clock_time64().returns(2000);

        LIGHT_HW.lock().handle_transition(&params);

        assert_eq!(with_core(|core| core.cw_tr.to.to_num::<u16>()), 0x1234);
        assert_eq!(with_core(|core| core.ww_tr.to.to_num::<u16>()), 0x5678);
    }

    #[test]
    #[mry::lock(
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        pwm_set_cmp,
        clock_time,
        clock_time64
    )]
    fn test_handle_transition_when_light_is_off_sets_brightness_zero() {
        // This test verifies that transitions when light is off keep brightness at 0

        reset_for_test();
        with_core(|core| core.brightness = 1000);
        // brightness_tr defaults to current=0, to=0 — light is off
        let params = [
            0x34, 0x12, // brightness
            0x78, 0x56, // temperature
            0, 0, // ww
            0, 0,   // reserved
            0x3, // brightness + temperature flags
            0, 0, 0, 0, 0, 0, 0,
        ];

        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(0x08).returns(()); // Enable timer1
        mock_pwm_set_cmp(0, 0).returns(()); // CW channel (light off)
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel (light off)
        mock_clock_time().returns(1000);
        mock_clock_time64().returns(2000);

        LIGHT_HW.lock().handle_transition(&params);

        // Brightness should be set to 0 even though 0x1234 was requested
        assert_eq!(with_core(|core| core.brightness_tr.to.to_num::<u16>()), 0);
    }

    // --- Easing Function Tests ---

    #[test]
    fn test_ease_in_out_at_start() {
        // This test verifies ease_in_out returns initial value at t=0

        let t = I16F16::from_num(0);
        let b = I16F16::from_num(100); // Start value
        let c = I16F16::from_num(900); // Change amount

        let result = ease_in_out(t, b, c);

        assert_eq!(result.to_num::<u16>(), 100);
    }

    #[test]
    fn test_ease_in_out_at_end() {
        // This test verifies ease_in_out returns final value at t=MAX

        let t = I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE);
        let b = I16F16::from_num(100);
        let c = I16F16::from_num(900);

        let result = ease_in_out(t, b, c);

        assert_eq!(result.to_num::<u16>(), 1000);
    }

    #[test]
    fn test_ease_in_out_at_midpoint() {
        // This test verifies ease_in_out produces smooth transition at midpoint

        let t = I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE / 2);
        let b = I16F16::from_num(0);
        let c = I16F16::from_num(1000);

        let result = ease_in_out(t, b, c);

        // At midpoint, should be roughly in the middle
        let result_val = result.to_num::<u16>();
        assert!(result_val > 400 && result_val < 600);
    }

    #[test]
    fn test_ease_in_out_negative_change() {
        // This test verifies ease_in_out works with negative change (decreasing)

        let t = I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE);
        let b = I16F16::from_num(1000);
        let c = I16F16::from_num(-500);

        let result = ease_in_out(t, b, c);

        assert_eq!(result.to_num::<u16>(), 500);
    }

    // --- Transition Step Tests ---

    #[test]
    #[mry::lock(read_reg_tmr_ctrl, write_reg_tmr_ctrl, pwm_set_cmp, clock_time64)]
    fn test_transition_step_completes_when_time_reached() {
        // This test verifies transition_step finalizes state when target time is reached

        reset_for_test();
        with_core(|core| {
            core.brightness_tr = Transition {
                from: I16F16::from_num(0),
                current: I16F16::from_num(0),
                to: I16F16::from_num(500),
                start: Instant::from_ticks(0),
                end: Instant::from_ticks(100),
            };
            core.cw_tr = Transition {
                from: I16F16::from_num(0),
                current: I16F16::from_num(0),
                to: I16F16::from_num(300),
                start: Instant::from_ticks(0),
                end: Instant::from_ticks(100),
            };
            core.ww_tr = Transition {
                from: I16F16::from_num(0),
                current: I16F16::from_num(0),
                to: I16F16::from_num(200),
                start: Instant::from_ticks(0),
                end: Instant::from_ticks(100),
            };
        });

        mock_clock_time64().returns(100);
        mock_read_reg_tmr_ctrl().returns(0x10);
        mock_write_reg_tmr_ctrl(0x10).returns(()); // Disable timer1 (0x10 & ~0x08 = 0x10)
        mock_pwm_set_cmp(0, 0).returns(()); // CW channel
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel

        transition_step();

        // Should match new state exactly
        assert_eq!(
            with_core(|core| core.brightness_tr.current.to_num::<u16>()),
            500
        );
        assert_eq!(with_core(|core| core.cw_tr.current.to_num::<u16>()), 300);
        assert_eq!(with_core(|core| core.ww_tr.current.to_num::<u16>()), 200);
        // Timer should be disabled
        mock_write_reg_tmr_ctrl(0x10).assert_called(1);
    }

    #[test]
    #[mry::lock(pwm_set_cmp, clock_time64)]
    fn test_transition_step_interpolates_during_transition() {
        // This test verifies transition_step correctly interpolates values mid-transition

        reset_for_test();
        with_core(|core| {
            core.brightness_tr = Transition {
                from: I16F16::from_num(0),
                current: I16F16::from_num(0),
                to: I16F16::from_num(1000),
                start: Instant::from_ticks(0),
                end: Instant::from_ticks(100),
            };
            core.cw_tr = Transition {
                from: I16F16::from_num(0),
                current: I16F16::from_num(0),
                to: I16F16::from_num(1000),
                start: Instant::from_ticks(0),
                end: Instant::from_ticks(100),
            };
            core.ww_tr = Transition {
                from: I16F16::from_num(0),
                current: I16F16::from_num(0),
                to: I16F16::from_num(1000),
                start: Instant::from_ticks(0),
                end: Instant::from_ticks(100),
            };
        });

        // At 50% progress
        mock_pwm_set_cmp(0, 0).returns(()); // CW channel
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel
        mock_clock_time64().returns(50);

        transition_step();

        // Should be partway through transition, not at extremes
        let brightness = with_core(|core| core.brightness_tr.current.to_num::<u16>());
        assert!(brightness > 100 && brightness < 900);
    }

    // --- Light State Tests ---

    #[test]
    #[mry::lock(clock_time64)]
    fn test_is_light_off_when_current_brightness_zero() {
        // This test verifies is_light_off returns false when transitioning to non-zero brightness

        mock_clock_time64().returns(1000);

        reset_for_test();
        // Active transition from 0 toward 500: is_active() = true, to = 500
        with_core(|core| {
            core.brightness_tr = Transition {
                from: I16F16::from_num(0),
                current: I16F16::from_num(0),
                to: I16F16::from_num(500),
                start: Instant::from_ticks(0),
                end: Instant::from_ticks(100),
            };
        });

        assert!(!is_light_off());
    }

    #[test]
    fn test_is_light_off_when_new_brightness_zero() {
        // This test verifies is_light_off returns true when new brightness is 0

        reset_for_test();
        // Active transition from 500 toward 0: is_active() = true, to = 0
        with_core(|core| {
            core.brightness_tr = Transition {
                from: I16F16::from_num(500),
                current: I16F16::from_num(500),
                to: I16F16::from_num(0),
                start: Instant::from_ticks(0),
                end: Instant::from_ticks(100),
            };
        });

        assert!(is_light_off());
    }

    #[test]
    fn test_is_light_off_when_brightness_nonzero() {
        // This test verifies is_light_off returns false when brightness is non-zero

        reset_for_test();
        // No active transition (start == end == 0), current = 500
        with_core(|core| {
            core.brightness_tr.current = I16F16::from_num(500);
            core.brightness_tr.to = I16F16::from_num(500);
        });

        assert!(!is_light_off());
    }

    // --- Get Current Light State Tests ---

    #[test]
    fn test_get_current_light_state_returns_new_when_newer() {
        // This test verifies get_current_light_state returns target when transition is active

        reset_for_test();
        // Active transition (end > start): report to
        with_core(|core| {
            core.brightness_tr = Transition {
                from: I16F16::from_num(0),
                current: I16F16::from_num(500),
                to: I16F16::from_num(1000),
                start: Instant::from_ticks(100),
                end: Instant::from_ticks(200),
            };
        });

        let state = get_current_light_state();

        assert_eq!(state.brightness.to_num::<u16>(), 1000);
    }

    #[test]
    fn test_get_current_light_state_returns_current_when_equal_timestamp() {
        // This test verifies get_current_light_state returns current when no transition is active

        reset_for_test();
        // No active transition (start == end): report current
        with_core(|core| {
            core.brightness_tr = Transition {
                from: I16F16::from_num(0),
                current: I16F16::from_num(500),
                to: I16F16::from_num(1000),
                start: Instant::from_ticks(100),
                end: Instant::from_ticks(100), // end == start → not active
            };
        });

        let state = get_current_light_state();

        assert_eq!(state.brightness.to_num::<u16>(), 500);
    }

    #[test]
    fn test_get_current_light_state_returns_current_when_older() {
        // This test verifies get_current_light_state returns current when idle (no transition set up)

        reset_for_test();
        // Default idle state (start == end == 0): report current
        with_core(|core| {
            core.brightness_tr.current = I16F16::from_num(500);
            core.brightness_tr.to = I16F16::from_num(1000);
        });

        let state = get_current_light_state();

        assert_eq!(state.brightness.to_num::<u16>(), 500);
    }

    // --- Flash Save/Retrieve Tests ---

    #[test]
    #[mry::lock(flash_erase_sector)]
    fn test_light_lum_erase_resets_address() {
        // This test verifies light_lum_erase resets address and erases sector

        reset_for_test();
        let mut store = LightFlashStore::default();
        store.light_lum_addr = 0x1234;

        mock_flash_erase_sector(FLASH_ADR_LUM).returns(());

        store.light_lum_erase();

        assert_eq!(store.light_lum_addr, FLASH_ADR_LUM);
        mock_flash_erase_sector(FLASH_ADR_LUM).assert_called(1);
    }

    #[test]
    #[mry::lock(flash_write_page)]
    fn test_light_state_save_writes_to_flash() {
        // This test verifies light_state_save writes current state to flash

        reset_for_test();
        let mut store = LightFlashStore::default();
        store.light_lum_addr = FLASH_ADR_LUM;
        with_core(|core| {
            core.brightness = 500;
            core.cw_tr.current = I16F16::from_num(300);
            core.ww_tr.current = I16F16::from_num(200);
        });

        mock_flash_write_page(FLASH_ADR_LUM, size_of::<LumSaveT>() as u32, Any).returns(());

        store.light_state_save();

        assert_eq!(
            store.light_lum_addr,
            FLASH_ADR_LUM + size_of::<LumSaveT>() as u32
        );
        mock_flash_write_page(FLASH_ADR_LUM, size_of::<LumSaveT>() as u32, Any).assert_called(1);
    }

    #[test]
    #[mry::lock(flash_erase_sector, flash_write_page)]
    fn test_light_state_save_erases_when_sector_full() {
        // This test verifies light_state_save erases sector when address reaches end

        reset_for_test();
        let mut store = LightFlashStore::default();
        store.light_lum_addr =
            FLASH_ADR_LUM + FLASH_SECTOR_SIZE as u32 - size_of::<LumSaveT>() as u32;
        with_core(|core| core.brightness = 500);

        mock_flash_erase_sector(FLASH_ADR_LUM).returns(());
        mock_flash_write_page(FLASH_ADR_LUM, size_of::<LumSaveT>() as u32, Any).returns(());

        store.light_state_save();

        assert_eq!(
            store.light_lum_addr,
            FLASH_ADR_LUM + size_of::<LumSaveT>() as u32
        );
        mock_flash_erase_sector(FLASH_ADR_LUM).assert_called(1);
    }

    #[test]
    #[mry::lock(
        flash_read_page,
        analog_read,
        analog_write,
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        ll_device_status_update,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_light_lum_retrieve_restores_state_and_turns_on() {
        // This test verifies retrieve_from_flash correctly restores saved state

        reset_for_test();

        // Create a mock flash buffer with saved light state
        let saved_brightness = 800u16;
        let saved_cw = 600u16;
        let saved_ww = 400u16;

        // Mock flash_read_page to return a buffer with valid saved data
        mock_flash_read_page(Any, Any, Any).returns_with(
            move |_addr: u32, len: u32, buf: SendWrapper<*mut u8>| {
                let mut buffer = vec![0xFFu8; len as usize];

                // Write a valid LumSaveT structure at the beginning
                if len >= size_of::<LumSaveT>() as u32 {
                    buffer[0] = LIGHT_SAVE_VALID_FLAG;
                    buffer[1] = (saved_brightness & 0xFF) as u8;
                    buffer[2] = ((saved_brightness >> 8) & 0xFF) as u8;
                    buffer[3] = (saved_cw & 0xFF) as u8;
                    buffer[4] = ((saved_cw >> 8) & 0xFF) as u8;
                    buffer[5] = (saved_ww & 0xFF) as u8;
                    buffer[6] = ((saved_ww >> 8) & 0xFF) as u8;

                    // Write 0xFF after the valid entry to signal end of data
                    if len >= size_of::<LumSaveT>() as u32 * 2 {
                        buffer[size_of::<LumSaveT>()] = 0xFF;
                    }
                }

                unsafe {
                    core::ptr::copy_nonoverlapping(buffer.as_ptr(), *buf, len as usize);
                }
            },
        );

        // Mock analog read to indicate light was NOT off
        mock_analog_read(REGA_LIGHT_OFF).returns(0x00);
        mock_analog_write(REGA_LIGHT_OFF, 0x00).returns(());
        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(0x08).returns(()); // Enable timer1
        mock_ll_device_status_update([1, 0xff]).returns(()); // Light on status
                                                             // Initial transition_step at brightness=0 (before restore)
        mock_pwm_set_cmp(0, 0).returns(()); // CW channel at brightness 0
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel
        mock_clock_time64().returns(1000);

        let store = LightFlashStore::retrieve_from_flash();

        // Verify the state was restored
        assert_eq!(with_core(|core| core.brightness), saved_brightness);
        assert_eq!(
            with_core(|core| core.cw_tr.current.to_num::<u16>()),
            saved_cw
        );
        assert_eq!(
            with_core(|core| core.ww_tr.current.to_num::<u16>()),
            saved_ww
        );
        // The next save lands right after the 0xFF terminator.
        assert_eq!(
            store.light_lum_addr,
            FLASH_ADR_LUM + size_of::<LumSaveT>() as u32
        );

        // The light_onoff should have been called
        mock_ll_device_status_update([1, 0xff]).assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time_exceed, flash_write_page)]
    fn test_check_light_state_save_saves_after_timeout() {
        // This test verifies check_light_state_save saves after 5 seconds

        reset_for_test();
        let mut store = LightFlashStore::default();
        with_core(|core| core.last_transition_time = 1000);

        mock_clock_time_exceed(1000, 5000 * 1000).returns(true);
        // flash_write_page(addr, size, pointer) - size is size_of::<LumSaveT>() = 7
        mock_flash_write_page(Any, 7, Any).returns(());

        store.check_light_state_save();

        assert_eq!(with_core(|core| core.last_transition_time), 0);
        mock_flash_write_page(Any, 7, Any).assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time_exceed)]
    fn test_check_light_state_save_does_not_save_before_timeout() {
        // This test verifies check_light_state_save doesn't save before 5 seconds

        reset_for_test();
        let mut store = LightFlashStore::default();
        with_core(|core| core.last_transition_time = 1000);

        mock_clock_time_exceed(1000, 5000 * 1000).returns(false);

        store.check_light_state_save();

        assert_eq!(with_core(|core| core.last_transition_time), 1000);
    }

    #[test]
    fn test_check_light_state_save_does_nothing_when_no_transition() {
        // This test verifies check_light_state_save does nothing when last_transition_time is 0

        reset_for_test();
        let mut store = LightFlashStore::default();
        with_core(|core| core.last_transition_time = 0);

        store.check_light_state_save();

        assert_eq!(with_core(|core| core.last_transition_time), 0);
    }

    // --- PWM Calculation Tests ---

    #[test]
    fn test_calculate_lumen_map_at_zero() {
        // This test verifies calculate_lumen_map returns 0 at 0% brightness

        let result = calculate_lumen_map(I16F16::from_num(0));

        assert_eq!(result, 0);
    }

    #[test]
    fn test_calculate_lumen_map_at_max() {
        // This test verifies calculate_lumen_map returns correct value at 100%

        let result = calculate_lumen_map(I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE));

        // At 100%: (5.2221 * 100^2) + 130.5908 * 100 = 52221 + 13059.08 = 65280.08
        assert!((65000..=66000).contains(&result));
    }

    #[test]
    fn test_calculate_lumen_map_at_fifty_percent() {
        // This test verifies calculate_lumen_map returns correct value at 50%

        let result = calculate_lumen_map(I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE / 2));

        // At 50%: (5.2221 * 50^2) + 130.5908 * 50 = 13055.25 + 6529.54 = 19584.79
        assert!((19000..=20500).contains(&result));
    }

    #[test]
    #[mry::lock(pwm_set_cmp)]
    fn test_light_adjust_cw_sets_correct_pwm() {
        // This test verifies light_adjust_cw correctly sets PWM value

        reset_for_test();
        let val = I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE);
        let lum = I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE);

        mock_pwm_set_cmp(PWMID_G, 10455).returns(()); // Max value, pol=false

        light_adjust_cw(val, lum);

        mock_pwm_set_cmp(PWMID_G, 10455).assert_called(1);
    }

    #[test]
    #[mry::lock(pwm_set_cmp)]
    fn test_light_adjust_ww_sets_correct_pwm() {
        // This test verifies light_adjust_ww correctly sets PWM value

        reset_for_test();
        let val = I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE);
        let lum = I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE);

        mock_pwm_set_cmp(PWMID_B, 0).returns(()); // Max inverted (pol=true)

        light_adjust_ww(val, lum);

        mock_pwm_set_cmp(PWMID_B, 0).assert_called(1);
    }

    #[test]
    #[mry::lock(pwm_set_cmp)]
    fn test_light_adjust_rgb_hw_calls_both_adjusters() {
        // This test verifies light_adjust_rgb_hw calls both CW and WW adjusters

        reset_for_test();
        let cw = I16F16::from_num(500);
        let ww = I16F16::from_num(300);
        let lum = I16F16::from_num(1000);

        mock_pwm_set_cmp(0, 0).returns(()); // CW channel (PWMID_G)
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel (PWMID_B)

        light_adjust_rgb_hw(cw, ww, lum);

        // Should be called once for each channel
        mock_pwm_set_cmp(0, 0).assert_called(1);
        mock_pwm_set_cmp(1, 10455).assert_called(1);
    }

    #[test]
    fn test_get_pwm_cmp_returns_scaled_value() {
        // This test verifies get_pwm_cmp correctly scales values

        let val = I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE);
        let lum = I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE);

        let result = get_pwm_cmp(val, lum);

        // At max brightness, lumen map returns ~65280, so result should be ~65280
        // (MAX * 65280) / MAX = 65280
        assert!(
            result >= 65000,
            "Result should be near 65280, got {}",
            result
        );
    }

    #[test]
    fn test_get_pwm_cmp_at_zero_brightness() {
        // This test verifies get_pwm_cmp returns 0 at 0 brightness

        let val = I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE);
        let lum = I16F16::from_num(0);

        let result = get_pwm_cmp(val, lum);

        assert_eq!(result, 0);
    }

    // --- Device Status Update Tests ---

    #[test]
    #[mry::lock(ll_device_status_update)]
    fn test_device_status_update_reports_on() {
        // This test verifies device_status_update reports correct status when light is on

        reset_for_test();
        with_core(|core| core.brightness_tr.current = I16F16::from_num(500));

        mock_ll_device_status_update([1, 0xff]).returns(()); // Status 0 = light on

        device_status_update();

        mock_ll_device_status_update([1, 0xff]).assert_called(1);
    }

    #[test]
    #[mry::lock(ll_device_status_update)]
    fn test_device_status_update_reports_off() {
        // This test verifies device_status_update reports correct status when light is off

        reset_for_test();
        // brightness_tr defaults to current=0, to=0 — light is already off

        mock_ll_device_status_update([0, 0xff]).returns(()); // Status 0 = light off

        device_status_update();

        mock_ll_device_status_update([0, 0xff]).assert_called(1);
    }

    // --- Begin Transition Tests ---

    #[test]
    #[mry::lock(
        clock_time,
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_begin_transition_updates_state() {
        // This test verifies begin_transition correctly sets up new transition

        reset_for_test();
        with_core(|core| {
            core.brightness_tr.current = I16F16::from_num(100);
            core.brightness_tr.to = I16F16::from_num(100);
            core.cw_tr.current = I16F16::from_num(100);
            core.cw_tr.to = I16F16::from_num(100);
            core.ww_tr.current = I16F16::from_num(100);
            core.ww_tr.to = I16F16::from_num(100);
        });

        mock_clock_time().returns(5000);
        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(0x08).returns(()); // Enable timer1
        mock_pwm_set_cmp(0, 0).returns(()); // CW channel
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel
        mock_clock_time64().returns(3000);

        begin_transition(500, 300, 800);

        assert_eq!(with_core(|core| core.cw_tr.to.to_num::<u16>()), 500);
        assert_eq!(with_core(|core| core.ww_tr.to.to_num::<u16>()), 300);
        assert_eq!(with_core(|core| core.brightness_tr.to.to_num::<u16>()), 800);
        assert_eq!(
            with_core(|core| core.brightness_tr.from.to_num::<u16>()),
            100
        );
        assert_eq!(with_core(|core| core.last_transition_time), 5000);
    }

    #[test]
    #[mry::lock(
        clock_time,
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_begin_transition_does_not_update_time_if_no_color_change() {
        // This test verifies begin_transition doesn't update time if only brightness changes

        reset_for_test();
        with_core(|core| {
            core.cw_tr.current = I16F16::from_num(500);
            core.cw_tr.to = I16F16::from_num(500);
            core.ww_tr.current = I16F16::from_num(300);
            core.ww_tr.to = I16F16::from_num(300);
            core.last_transition_time = 0;
        });

        mock_clock_time().returns(5000);
        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(Any).returns(());
        mock_pwm_set_cmp(Any, Any).returns(());
        mock_clock_time64().returns(3000);

        begin_transition(500, 300, 800); // Same CW/WW, different brightness

        assert_eq!(with_core(|core| core.last_transition_time), 0);
    }

    // --- Message Processing Tests ---

    #[test]
    #[mry::lock(
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        ll_device_status_update,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_process_message_handles_onoff() {
        // This test verifies process_message correctly handles on/off messages

        reset_for_test();
        with_core(|core| core.brightness = 500);

        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(0x08).returns(()); // Enable timer1
        mock_ll_device_status_update([1, 0xff]).returns(()); // Light on status
                                                             // Initial transition_step at brightness=0
        mock_pwm_set_cmp(0, 0).returns(()); // CW channel at brightness 0
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel
        mock_clock_time64().returns(1000);

        let msg = create_onoff_message(LIGHT_ON_PARAM);

        send_light_message(msg.cmd, msg.params);
        dispatch_message(LIGHT_CMD_CHANNEL.try_receive().unwrap());

        assert_eq!(with_core(|core| core.brightness_tr.to.to_num::<u16>()), 500);
    }

    #[test]
    #[mry::lock(
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        ll_device_status_update,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_process_message_impl_parks_until_send_message() {
        use core::future::Future;
        use core::task::{Context, Poll};
        use futures::task::noop_waker;

        // Setup
        reset_for_test();
        with_core(|core| core.brightness = 500);

        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(Any).returns(());
        mock_ll_device_status_update(Any).returns(());
        mock_pwm_set_cmp(Any, Any).returns(());
        mock_clock_time64().returns(1000);

        let future = light_manager_task();
        futures::pin_mut!(future);

        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);

        // The channel is empty on entry, so the task parks on receive().
        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Pending));
        assert!(LIGHT_CMD_CHANNEL.is_empty());

        // A message wakes the parked receive.
        let msg = create_onoff_message(LIGHT_ON_PARAM);
        send_light_message(msg.cmd, msg.params);

        // The task consumes and dispatches it, then parks on the empty channel
        // again, so the poll as a whole is still pending.
        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Pending));

        // The command took effect and the channel is drained.
        assert_eq!(with_core(|core| core.brightness_tr.to.to_num::<u16>()), 500);
        assert!(LIGHT_CMD_CHANNEL.is_empty());
    }

    #[test]
    #[mry::lock(
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        pwm_set_cmp,
        clock_time,
        clock_time64
    )]
    fn test_process_message_handles_transition() {
        // This test verifies process_message correctly handles transition messages

        reset_for_test();
        with_core(|core| {
            core.brightness_tr.current = I16F16::from_num(500);
            core.brightness_tr.to = I16F16::from_num(500);
        });

        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(0x08).returns(()); // Enable timer1
                                                   // Initial transition_step at brightness=500, cw=MAX
        mock_pwm_set_cmp(0, 33).returns(()); // CW channel at brightness 500
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel
        mock_clock_time().returns(1000);
        mock_clock_time64().returns(2000);

        let msg = create_brightness_message(1000);

        send_light_message(msg.cmd, msg.params);
        dispatch_message(LIGHT_CMD_CHANNEL.try_receive().unwrap());

        assert_eq!(with_core(|core| core.brightness), 1000);
        assert_eq!(
            with_core(|core| core.brightness_tr.to.to_num::<u16>()),
            1000
        );
    }

    #[test]
    fn test_process_message_ignores_unknown_command() {
        // This test verifies process_message ignores unknown command types

        reset_for_test();
        let old_brightness = with_core(|core| core.brightness);

        let msg = Message {
            cmd: 0xFF, // Unknown command
            params: [0; 16],
        };

        send_light_message(msg.cmd, msg.params);
        dispatch_message(LIGHT_CMD_CHANNEL.try_receive().unwrap());

        // State should be unchanged
        assert_eq!(with_core(|core| core.brightness), old_brightness);
    }

    // --- Integration Tests ---

    #[test]
    #[mry::lock(
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        ll_device_status_update,
        clock_time,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_full_transition_sequence() {
        // This test verifies transition state updates correctly

        reset_for_test();
        with_core(|core| core.brightness = 1000);
        // brightness_tr defaults to current=0 (light is off)

        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(0x08).returns(()); // Enable timer1
        mock_ll_device_status_update([1, 0xff]).returns(()); // Light on status
        mock_clock_time().returns(0);
        // Initial transition_step at brightness=0
        mock_pwm_set_cmp(0, 0).returns(()); // CW channel at brightness 0
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel
        mock_clock_time64().returns(1500);

        // Turn light on - should set up a transition
        light_onoff(true);
        assert_eq!(
            with_core(|core| core.brightness_tr.to.to_num::<u16>()),
            1000
        );
        assert_eq!(
            with_core(|core| core.brightness_tr.current.to_num::<u16>()),
            0
        );

        // Transition should be active (end > start)
        assert!(with_core(
            |core| core.brightness_tr.end > core.brightness_tr.start
        ));
    }

    #[test]
    #[mry::lock(
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        clock_time,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_color_temperature_adjustment() {
        // This test verifies adjusting color temperature while light is on

        reset_for_test();
        with_core(|core| {
            core.brightness_tr.current = I16F16::from_num(1000);
            core.brightness_tr.to = I16F16::from_num(1000);
            core.cw_tr.current = I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE);
            core.cw_tr.to = I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE);
            core.ww_tr.current = I16F16::from_num(0);
            core.ww_tr.to = I16F16::from_num(0);
        });

        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(0x08).returns(()); // Enable timer1
        mock_clock_time().returns(1000);
        // Initial transition_step at brightness=1000, cw=MAX
        mock_pwm_set_cmp(0, 71).returns(()); // CW channel at brightness 1000
        mock_pwm_set_cmp(1, 10455).returns(()); // WW channel
        mock_clock_time64().returns(2500);

        // Adjust to warm white (WW high, CW low)
        let params = create_temperature_message(MAX_LUM_BRIGHTNESS_VALUE - 100);
        LIGHT_HW.lock().handle_transition(&params.params);

        assert_eq!(with_core(|core| core.cw_tr.to.to_num::<u16>()), 100);
        assert_eq!(
            with_core(|core| core.ww_tr.to.to_num::<u16>()),
            MAX_LUM_BRIGHTNESS_VALUE - 100
        );
        assert_eq!(with_core(|core| core.last_transition_time), 1000);
    }

    // --- Get Current Light State (Active CW/WW) Tests ---

    #[test]
    fn test_get_current_light_state_cw_active_returns_to() {
        // cw_tr.is_active() == true path: must return cw_tr.to, not cw_tr.current
        reset_for_test();
        with_core(|core| {
            core.cw_tr = Transition {
                from: I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE),
                current: I16F16::from_num(30000),
                to: I16F16::from_num(100),
                start: Instant::from_ticks(1),
                end: Instant::from_ticks(200),
            };
        });

        let state = get_current_light_state();

        assert_eq!(state.cw.to_num::<u16>(), 100);
    }

    #[test]
    fn test_get_current_light_state_ww_active_returns_to() {
        // ww_tr.is_active() == true path: must return ww_tr.to, not ww_tr.current
        // Use 30000 — within I16F16 signed integer range (max 32767)
        reset_for_test();
        with_core(|core| {
            core.ww_tr = Transition {
                from: I16F16::from_num(0),
                current: I16F16::from_num(15000),
                to: I16F16::from_num(30000),
                start: Instant::from_ticks(1),
                end: Instant::from_ticks(200),
            };
        });

        let state = get_current_light_state();

        assert_eq!(state.ww.to_num::<u16>(), 30000);
    }

    // --- retrieve_from_flash: light-was-off + invalid-entry + full-sector-scan ---

    #[test]
    #[mry::lock(
        flash_read_page,
        analog_read,
        analog_write,
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        ll_device_status_update,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_light_lum_retrieve_light_was_off_turns_light_off() {
        // When the LightOff bit is set in analog memory, light_onoff(false) is called.
        reset_for_test();

        let saved_brightness = 800u16;
        let saved_cw = 600u16;
        let saved_ww = 400u16;

        mock_flash_read_page(Any, Any, Any).returns_with(
            move |_addr: u32, len: u32, buf: SendWrapper<*mut u8>| {
                let mut buffer = vec![0xFFu8; len as usize];
                if len as usize >= size_of::<LumSaveT>() {
                    buffer[0] = LIGHT_SAVE_VALID_FLAG;
                    buffer[1] = (saved_brightness & 0xFF) as u8;
                    buffer[2] = ((saved_brightness >> 8) & 0xFF) as u8;
                    buffer[3] = (saved_cw & 0xFF) as u8;
                    buffer[4] = ((saved_cw >> 8) & 0xFF) as u8;
                    buffer[5] = (saved_ww & 0xFF) as u8;
                    buffer[6] = ((saved_ww >> 8) & 0xFF) as u8;
                    // 0xFF after the first entry → triggers early return
                    if len as usize >= size_of::<LumSaveT>() * 2 {
                        buffer[size_of::<LumSaveT>()] = 0xFF;
                    }
                }
                unsafe { core::ptr::copy_nonoverlapping(buffer.as_ptr(), *buf, len as usize) };
            },
        );

        // Light WAS off at prior shutdown
        let light_off_bit = RecoverStatus::LightOff as u8;
        mock_analog_read(REGA_LIGHT_OFF).returns(light_off_bit);
        mock_analog_write(REGA_LIGHT_OFF, 0x00).returns(()); // clears the bit

        // light_onoff(false) → light_onoff_hw(false) → timer + pwm + status
        // Note: same-target guard fires (brightness_tr starts at 0, target is 0),
        // so transition_step immediately disables the timer (write_reg_tmr_ctrl(0x00)).
        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(Any).returns(());
        mock_ll_device_status_update([0, 0xff]).returns(());
        mock_pwm_set_cmp(0, 0).returns(());
        mock_pwm_set_cmp(1, 10455).returns(());
        mock_clock_time64().returns(1000);

        let store = LightFlashStore::retrieve_from_flash();

        assert_eq!(with_core(|core| core.brightness), saved_brightness);
        // brightness_tr targets 0 (light off)
        assert_eq!(with_core(|core| core.brightness_tr.to.to_num::<u16>()), 0);
        assert_eq!(
            store.light_lum_addr,
            FLASH_ADR_LUM + size_of::<LumSaveT>() as u32
        );
        mock_analog_write(REGA_LIGHT_OFF, 0x00).assert_called(1);
        mock_ll_device_status_update([0, 0xff]).assert_called(1);
    }

    #[test]
    #[mry::lock(
        flash_read_page,
        analog_read,
        analog_write,
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        ll_device_status_update,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_light_lum_retrieve_skips_invalid_flash_entry() {
        // An entry with a save_flag that is neither 0xA5 nor 0xFF is silently skipped.
        // The scan continues until a 0xFF entry is found.
        reset_for_test();

        let saved_cw = 1234u16;
        let saved_ww = 5678u16;
        let saved_brightness = 999u16;

        mock_flash_read_page(Any, Any, Any).returns_with(
            move |_addr: u32, len: u32, buf: SendWrapper<*mut u8>| {
                let mut buffer = vec![0xFFu8; len as usize];
                let entry_size = size_of::<LumSaveT>();
                // First entry: invalid flag byte (0x42)
                if len as usize >= entry_size {
                    buffer[0] = 0x42; // neither 0xA5 nor 0xFF
                    buffer[1] = 0;
                    buffer[2] = 0;
                    buffer[3] = 0;
                    buffer[4] = 0;
                    buffer[5] = 0;
                    buffer[6] = 0;
                }
                // Second entry: valid
                if len as usize >= entry_size * 2 {
                    buffer[entry_size] = LIGHT_SAVE_VALID_FLAG;
                    buffer[entry_size + 1] = (saved_brightness & 0xFF) as u8;
                    buffer[entry_size + 2] = ((saved_brightness >> 8) & 0xFF) as u8;
                    buffer[entry_size + 3] = (saved_cw & 0xFF) as u8;
                    buffer[entry_size + 4] = ((saved_cw >> 8) & 0xFF) as u8;
                    buffer[entry_size + 5] = (saved_ww & 0xFF) as u8;
                    buffer[entry_size + 6] = ((saved_ww >> 8) & 0xFF) as u8;
                }
                // Third entry: 0xFF → terminates scan
                // (buffer is already 0xFF-initialized, so this is implicit)
                unsafe { core::ptr::copy_nonoverlapping(buffer.as_ptr(), *buf, len as usize) };
            },
        );

        mock_analog_read(REGA_LIGHT_OFF).returns(0x00);
        mock_analog_write(REGA_LIGHT_OFF, 0x00).returns(());
        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(0x08).returns(());
        mock_ll_device_status_update([1, 0xff]).returns(());
        mock_pwm_set_cmp(0, 0).returns(());
        mock_pwm_set_cmp(1, 10455).returns(());
        mock_clock_time64().returns(1000);

        let store = LightFlashStore::retrieve_from_flash();

        // The valid entry (index 1) was loaded correctly despite the invalid one before it
        assert_eq!(with_core(|core| core.brightness), saved_brightness);
        assert_eq!(
            with_core(|core| core.cw_tr.current.to_num::<u16>()),
            saved_cw
        );
        assert_eq!(
            with_core(|core| core.ww_tr.current.to_num::<u16>()),
            saved_ww
        );
        // 0xFF terminator at index 2.
        assert_eq!(
            store.light_lum_addr,
            FLASH_ADR_LUM + 2 * size_of::<LumSaveT>() as u32
        );
    }

    #[test]
    #[mry::lock(
        flash_read_page,
        analog_read,
        analog_write,
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        ll_device_status_update,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_light_lum_retrieve_full_sector_uses_post_loop_restore() {
        // When the entire flash sector is full of valid entries (no 0xFF terminator),
        // the post-loop code restores the light state from analog memory.
        reset_for_test();

        let saved_brightness = 300u16;
        let saved_cw = 200u16;
        let saved_ww = 100u16;

        // Return only valid entries — no 0xFF anywhere
        mock_flash_read_page(Any, Any, Any).returns_with(
            move |_addr: u32, len: u32, buf: SendWrapper<*mut u8>| {
                let entry_size = size_of::<LumSaveT>();
                let num_entries = len as usize / entry_size;
                let mut buffer = vec![0u8; len as usize];
                for i in 0..num_entries {
                    let off = i * entry_size;
                    buffer[off] = LIGHT_SAVE_VALID_FLAG;
                    buffer[off + 1] = (saved_brightness & 0xFF) as u8;
                    buffer[off + 2] = ((saved_brightness >> 8) & 0xFF) as u8;
                    buffer[off + 3] = (saved_cw & 0xFF) as u8;
                    buffer[off + 4] = ((saved_cw >> 8) & 0xFF) as u8;
                    buffer[off + 5] = (saved_ww & 0xFF) as u8;
                    buffer[off + 6] = ((saved_ww >> 8) & 0xFF) as u8;
                }
                unsafe { core::ptr::copy_nonoverlapping(buffer.as_ptr(), *buf, len as usize) };
            },
        );

        // Light was on at prior shutdown
        mock_analog_read(REGA_LIGHT_OFF).returns(0x00);
        mock_analog_write(REGA_LIGHT_OFF, 0x00).returns(());
        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(0x08).returns(());
        mock_ll_device_status_update([1, 0xff]).returns(());
        mock_pwm_set_cmp(0, 0).returns(());
        mock_pwm_set_cmp(1, 10455).returns(());
        mock_clock_time64().returns(1000);

        let _store = LightFlashStore::retrieve_from_flash();

        // State from last valid entry in the full sector was restored
        assert_eq!(with_core(|core| core.brightness), saved_brightness);
        assert_eq!(
            with_core(|core| core.cw_tr.current.to_num::<u16>()),
            saved_cw
        );
        assert_eq!(
            with_core(|core| core.ww_tr.current.to_num::<u16>()),
            saved_ww
        );
        mock_ll_device_status_update([1, 0xff]).assert_called(1);
    }

    #[test]
    #[mry::lock(
        flash_read_page,
        analog_read,
        analog_write,
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        ll_device_status_update,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_light_lum_retrieve_full_sector_light_was_off() {
        // Full sector, no 0xFF terminator, AND the light was off at prior shutdown.
        // Exercises the post-loop RecoverStatus::LightOff == true branch.
        reset_for_test();

        let saved_brightness = 300u16;
        let saved_cw = 200u16;
        let saved_ww = 100u16;

        mock_flash_read_page(Any, Any, Any).returns_with(
            move |_addr: u32, len: u32, buf: SendWrapper<*mut u8>| {
                let entry_size = size_of::<LumSaveT>();
                let num_entries = len as usize / entry_size;
                let mut buffer = vec![0u8; len as usize];
                for i in 0..num_entries {
                    let off = i * entry_size;
                    buffer[off] = LIGHT_SAVE_VALID_FLAG;
                    buffer[off + 1] = (saved_brightness & 0xFF) as u8;
                    buffer[off + 2] = ((saved_brightness >> 8) & 0xFF) as u8;
                    buffer[off + 3] = (saved_cw & 0xFF) as u8;
                    buffer[off + 4] = ((saved_cw >> 8) & 0xFF) as u8;
                    buffer[off + 5] = (saved_ww & 0xFF) as u8;
                    buffer[off + 6] = ((saved_ww >> 8) & 0xFF) as u8;
                }
                unsafe { core::ptr::copy_nonoverlapping(buffer.as_ptr(), *buf, len as usize) };
            },
        );

        // Light WAS off at prior shutdown
        let light_off_bit = RecoverStatus::LightOff as u8;
        mock_analog_read(REGA_LIGHT_OFF).returns(light_off_bit);
        mock_analog_write(REGA_LIGHT_OFF, 0x00).returns(());
        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(Any).returns(());
        mock_ll_device_status_update([0, 0xff]).returns(());
        mock_pwm_set_cmp(0, 0).returns(());
        mock_pwm_set_cmp(1, 10455).returns(());
        mock_clock_time64().returns(1000);

        let _store = LightFlashStore::retrieve_from_flash();

        assert_eq!(with_core(|core| core.brightness), saved_brightness);
        assert_eq!(with_core(|core| core.brightness_tr.to.to_num::<u16>()), 0);
        mock_analog_write(REGA_LIGHT_OFF, 0x00).assert_called(1);
        mock_ll_device_status_update([0, 0xff]).assert_called(1);
    }

    // --- Independent Channel / Same-Target Guard Tests ---

    #[test]
    #[mry::lock(
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        pwm_set_cmp,
        clock_time64,
        clock_time
    )]
    fn test_same_target_guard_does_not_reset_transition() {
        // Verifies that begin_transition with the same brightness target mid-transition
        // does not reset the transition end time (same-target guard fires).

        reset_for_test();

        // Set up an in-progress brightness transition to 500.
        // Use a far-future end so Instant::now() won't exceed it during the test.
        with_core(|core| {
            core.brightness_tr.to = I16F16::from_num(500);
            core.brightness_tr.from = I16F16::from_num(0);
            core.brightness_tr.current = I16F16::from_num(250);
            core.brightness_tr.start = Instant::from_ticks(0);
            core.brightness_tr.end = Instant::from_ticks(100_000_000);
        });

        // cw/ww already at defaults (to = MAX and 0), pass the same so guard fires for them too
        mock_clock_time64().returns(50);
        mock_clock_time().returns(0);
        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(Any).returns(());
        mock_pwm_set_cmp(Any, Any).returns(());

        // begin_transition with same brightness target — guard should fire, end unchanged
        begin_transition(MAX_LUM_BRIGHTNESS_VALUE, 0, 500);

        assert_eq!(
            with_core(|core| core.brightness_tr.end),
            Instant::from_ticks(100_000_000)
        );
    }

    #[test]
    #[mry::lock(
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        pwm_set_cmp,
        clock_time64
    )]
    fn test_onoff_does_not_reset_colour_transition() {
        // Verifies that light_onoff_hw only starts a brightness transition and
        // does not disturb an in-progress colour transition.

        reset_for_test();

        // Set up an in-progress colour transition
        with_core(|core| {
            core.cw_tr.to = I16F16::from_num(100);
            core.cw_tr.from = I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE);
            core.cw_tr.current = I16F16::from_num(30000);
            core.cw_tr.start = Instant::from_ticks(0);
            core.cw_tr.end = Instant::from_ticks(100_000_000);
            core.brightness = 500;
        });

        mock_clock_time64().returns(50);
        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(Any).returns(());
        mock_pwm_set_cmp(Any, Any).returns(());

        // Turn light on — only brightness_tr should change
        light_onoff_hw(true);

        // cw_tr.end must be unchanged
        assert_eq!(
            with_core(|core| core.cw_tr.end),
            Instant::from_ticks(100_000_000)
        );
        // brightness_tr should now target 500
        assert_eq!(
            with_core(|core| core.brightness_tr.to),
            I16F16::from_num(500)
        );
    }

    #[test]
    #[mry::lock(
        read_reg_tmr_ctrl,
        write_reg_tmr1_tick,
        write_reg_tmr_ctrl,
        pwm_set_cmp,
        clock_time64,
        clock_time
    )]
    fn test_colour_command_does_not_reset_brightness_transition() {
        // Verifies that a colour-only change via begin_transition does not reset
        // an in-progress brightness transition (same-target guard fires for brightness).

        reset_for_test();

        // In-progress brightness transition to 1000
        with_core(|core| {
            core.brightness_tr.to = I16F16::from_num(1000);
            core.brightness_tr.from = I16F16::from_num(0);
            core.brightness_tr.current = I16F16::from_num(500);
            core.brightness_tr.start = Instant::from_ticks(0);
            core.brightness_tr.end = Instant::from_ticks(100_000_000);
            core.brightness = 1000;
        });

        mock_clock_time64().returns(50);
        mock_clock_time().returns(0);
        mock_read_reg_tmr_ctrl().returns(0x00);
        mock_write_reg_tmr1_tick(0).returns(());
        mock_write_reg_tmr_ctrl(Any).returns(());
        mock_pwm_set_cmp(Any, Any).returns(());

        // New colour, same brightness — guard fires for brightness
        begin_transition(100, 500, 1000);

        // brightness_tr.end unchanged (guard fired)
        assert_eq!(
            with_core(|core| core.brightness_tr.end),
            Instant::from_ticks(100_000_000)
        );
        // Colour was updated
        assert_eq!(with_core(|core| core.cw_tr.to), I16F16::from_num(100));
        assert_eq!(with_core(|core| core.ww_tr.to), I16F16::from_num(500));
    }
}
