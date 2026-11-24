//! LED status indication and the device-identity advertisement payload.
//!
//! Two independent jobs live here because both are pure value mapping:
//!
//! * the LED event machine, which blinks the lamp itself as a status indicator
//!   and therefore talks only to the lamp API in [`super::transitions`];
//! * [`build_vendor_adv_data`] / [`adv_private_to_bytes`], which turn the MAC,
//!   vendor id and device address into the two advertisement payloads. They take
//!   values and return values: no SDK, no globals, no pointer reinterpretation.

use core::sync::atomic::AtomicU32;

use fixed::types::I16F16;
use mesh_protocol::packet_types::{AdvPrivate, AdvRspPrivate};

use super::transitions::{is_light_off, light_adjust_cw, light_adjust_ww, light_onoff_hw};
use crate::config::MAX_LUM_BRIGHTNESS_VALUE;
use tlsr8266_hal::common::sync::SimplifyLS;
use tlsr8266_hal::mcu::clock::{clock_time, CLOCK_SYS_CLOCK_1US};
use tlsr8266_hal::BIT;

pub const LED_INDICATE_VAL: u16 = MAX_LUM_BRIGHTNESS_VALUE;
pub const LED_MASK: u8 = 0x07;

/// Pack one LED blink event into the `u32` the controller consumes.
///
/// layout: `on_ticks | off_ticks << 8 | blink_count << 16 | led_mask << 24`,
/// each interval counted in 64 ms units.
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

/// LED controller state managing all LED operations.
/// Encapsulates timing, patterns, and current state for LED control.
struct LedControllerState {
    /// Bitmask of pending LED events waiting to be processed
    event_pending: AtomicU32,
    /// Counter for LED blink cycles in current sequence
    blink_count: AtomicU32,
    /// LED on-time duration in microseconds for blink patterns
    on_duration_us: AtomicU32,
    /// LED off-time duration in microseconds for blink patterns
    off_duration_us: AtomicU32,
    /// LED selection mask indicating which LEDs are active
    led_selection_mask: AtomicU32,
    /// Current timestamp for LED timing control
    timing_tick: AtomicU32,
    /// LED pattern number or sequence identifier
    pattern_number: AtomicU32,
    /// Current LED state: 1 for on, 0 for off
    is_on: AtomicU32,
}

impl LedControllerState {
    const fn new() -> Self {
        Self {
            event_pending: AtomicU32::new(0),
            blink_count: AtomicU32::new(0),
            on_duration_us: AtomicU32::new(0),
            off_duration_us: AtomicU32::new(0),
            led_selection_mask: AtomicU32::new(0),
            timing_tick: AtomicU32::new(0),
            pattern_number: AtomicU32::new(0),
            is_on: AtomicU32::new(0),
        }
    }
}

/// Global LED controller state for device status indication.
static LED_CONTROLLER: LedControllerState = LedControllerState::new();

/// Queue one blink pattern. The next [`led_indicate_step`] seeds the controller
/// from it and clears the slot.
pub fn config_led_event(e: u32) {
    LED_CONTROLLER.event_pending.set(e);
}

/// Advance the LED indication machine by one main-loop pass.
///
/// Blinks the lamp itself (cool-white and/or warm-white, per the event's mask)
/// and restores the lamp's on/off state when the last blink completes.
pub fn led_indicate_step() {
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
                light_onoff_hw(!is_light_off()); // should not report online status again
                return;
            }
        }

        if led_off || led_on {
            if LED_CONTROLLER.led_selection_mask.get() & BIT!(0) != 0 {
                light_adjust_cw(
                    I16F16::from_num(LED_INDICATE_VAL * led_on as u16),
                    I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE),
                );
            }
            if LED_CONTROLLER.led_selection_mask.get() & BIT!(1) != 0 {
                light_adjust_ww(
                    I16F16::from_num(LED_INDICATE_VAL * led_on as u16),
                    I16F16::from_num(MAX_LUM_BRIGHTNESS_VALUE),
                );
            }
            if LED_CONTROLLER.led_selection_mask.get() & BIT!(5) != 0 {}
        }
    }
}

/// Build the two private advertisement payloads this device broadcasts.
///
/// Every field is initialised here rather than inherited from the statics'
/// initialisers, so the payload is a function of the arguments alone:
/// the vendor id fills the manufacturer and mesh-product fields, the first four
/// MAC bytes (little-endian) identify the node, `product_uuid` declares a CCT
/// light, the status byte is the advertising "present" marker, and the
/// reservation block is the sequential `0..16` pattern the protocol expects.
pub fn build_vendor_adv_data(
    mac_id: [u8; 6],
    vendor_id: u16,
    dev_addr: u16,
) -> (AdvPrivate, AdvRspPrivate) {
    let mac_address = u32::from_le_bytes([mac_id[0], mac_id[1], mac_id[2], mac_id[3]]);

    let pri = AdvPrivate {
        manufacture_id: vendor_id,
        mesh_product_uuid: vendor_id,
        mac_address,
    };

    let mut rsv = [0u8; 16];
    for (index, value) in rsv.iter_mut().enumerate() {
        *value = index as u8;
    }

    let rsp = AdvRspPrivate {
        manufacture_id: vendor_id,
        mesh_product_uuid: vendor_id,
        mac_address,
        product_uuid: 0x02,
        status: 0x01,
        device_address: dev_addr,
        rsv,
    };

    (pri, rsp)
}

/// Serialise the primary advertisement payload to its wire bytes.
///
/// `AdvPrivate` is `#[repr(C, packed)]` with integer fields only, so the eight
/// bytes are exactly the little-endian field encodings — no raw pointer view
/// (and therefore no pointer reinterpretation) is needed.
pub fn adv_private_to_bytes(adv: &AdvPrivate) -> [u8; 8] {
    let mut out = [0u8; 8];
    out[0..2].copy_from_slice(&adv.manufacture_id.to_le_bytes());
    out[2..4].copy_from_slice(&adv.mesh_product_uuid.to_le_bytes());
    out[4..8].copy_from_slice(&adv.mac_address.to_le_bytes());
    out
}

/// Clear the LED controller, so a test starts from a known-idle indicator.
#[cfg(test)]
pub fn reset_for_test() {
    let controller = &LED_CONTROLLER;
    controller.event_pending.set(0);
    controller.blink_count.set(0);
    controller.on_duration_us.set(0);
    controller.off_duration_us.set(0);
    controller.led_selection_mask.set(0);
    controller.timing_tick.set(0);
    controller.pattern_number.set(0);
    controller.is_on.set(0);
}

/// The queued LED event, if any. Host tests observe the callback's effect here.
#[cfg(test)]
pub fn pending_event() -> u32 {
    LED_CONTROLLER.event_pending.get()
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::config::{PWMID_B, PWMID_G};
    use crate::light::transitions::{is_light_off, reset_for_test as reset_light};
    use crate::light::PMW_MAX_TICK;
    use core::sync::atomic::Ordering;
    use tlsr8266_hal::mcu::clock::fake_clock;
    use tlsr8266_pac::fake_mmio::FakeMmio;

    /// `reg_pwm_cmp` base: `pwm_set_cmp(id, cmp)` writes a `u16` at `base + id*4`.
    const PWM_CMP_BASE: u32 = 0x794;

    /// The duty values written to PWM channel `id` during the test, in order.
    ///
    /// The lamp driver is the only observable of the indication step: it writes
    /// the compare register directly, so "the channel was driven" and "at what
    /// duty" are both visible in the write log.
    fn pwm_writes(id: u32) -> Vec<u32> {
        FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == PWM_CMP_BASE + id * 4 | tlsr8266_pac::REG_BASE_ADDR)
            .map(|w| w.value)
            .collect()
    }

    /// A lit LED drives cool white at full scale and warm white at the inverse
    /// end of its (inverted-polarity) range.
    const LED_LIT_CW: u32 = PMW_MAX_TICK as u32;
    const LED_LIT_WW: u32 = 0;

    /// The HAL host seams are process-wide singletons, so every test that
    /// observes them starts from a zeroed bank, clock and lamp state.
    fn reset_fakes() {
        FakeMmio::reset();
        fake_clock::reset();
        reset_light();
    }

    // --- Tests for config_led_event ---

    #[test]
    fn test_cfg_led_event_sets_pending_event() {
        // This test verifies that cfg_led_event correctly sets LED_CONTROLLER.event_pending

        // Setup: Clear any previous event
        LED_CONTROLLER.event_pending.set(0);

        // Execute: Set a new event
        let test_event = 0x12345678;
        config_led_event(test_event);

        // Verify: Event pending should match
        assert_eq!(
            LED_CONTROLLER.event_pending.get(),
            test_event,
            "Event pending should be set to the provided value"
        );
    }

    // --- Tests for led_indicate_step ---

    #[test]
    fn test_proc_led_returns_early_when_no_event() {
        // Reset LED_CONTROLLER to clean state
        reset_for_test();

        // Setup: No blink count and no pending event
        LED_CONTROLLER.blink_count.set(0);
        LED_CONTROLLER.event_pending.set(0);

        // Execute
        led_indicate_step();

        // Verify: Nothing should happen (function returns early)
        assert_eq!(LED_CONTROLLER.blink_count.get(), 0);
    }

    #[test]
    fn test_proc_led_initializes_new_event() {
        // Reset LED_CONTROLLER to clean state
        reset_for_test();

        // Setup: Set a new event
        LED_CONTROLLER.event_pending.set(LED_EVENT_FLASH_2HZ_2S);
        LED_CONTROLLER.pattern_number.set(1);
        LED_CONTROLLER.blink_count.set(0);

        let test_time = 1000000u32;
        fake_clock::set_ticks(test_time as u64);

        // Execute
        led_indicate_step();

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
    fn test_proc_led_toggles_led_state_on_timing() {
        // Reset LED_CONTROLLER to clean state
        reset_fakes();

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
        fake_clock::set_ticks(new_time as u64);

        // Execute
        led_indicate_step();

        // Verify: LED should turn on
        assert_eq!(LED_CONTROLLER.is_on.get(), 1, "LED should be on");
        // The mask selects both lamp channels, and the indication value is the
        // full-scale brightness, so each channel's compare register is driven
        // exactly once to the lit duty.
        assert_eq!(
            pwm_writes(PWMID_G),
            vec![LED_LIT_CW],
            "cool white must be driven lit exactly once"
        );
        assert_eq!(
            pwm_writes(PWMID_B),
            vec![LED_LIT_WW],
            "warm white must be driven lit exactly once"
        );
    }

    #[test]
    fn test_proc_led_continues_blink_sequence() {
        // Reset LED_CONTROLLER to clean state
        reset_fakes();

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
        fake_clock::set_ticks(100_000);

        // Execute
        led_indicate_step();

        // Verify: LED should turn on (line 160)
        assert_eq!(LED_CONTROLLER.is_on.get(), 1, "LED should be on");

        // Verify: Pattern number should be incremented (line 167)
        assert_eq!(
            LED_CONTROLLER.pattern_number.get(),
            1,
            "Pattern number should be incremented"
        );

        // Verify: Since pattern_number (1) != blink_count (3), it should fall through to line 180
        // and drive the lamp channels (the real driver's duty writes are the
        // observable that replaces the mocked adjust calls).
        assert_eq!(
            pwm_writes(PWMID_G),
            vec![LED_LIT_CW],
            "cool white must be driven lit exactly once"
        );

        // Verify: Blink sequence should continue (not complete)
        assert_eq!(
            LED_CONTROLLER.blink_count.get(),
            3,
            "Blink should not be complete"
        );
    }

    #[test]
    fn test_proc_led_completes_blink_sequence() {
        // Reset LED_CONTROLLER to clean state
        reset_fakes();

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

        // The lamp starts off (the transition core's boot state), so the
        // restore at the end of the sequence must keep it off.
        assert!(is_light_off(), "precondition: the lamp starts off");

        // Simulate time passing
        let new_time = LED_CONTROLLER.timing_tick.get() + LED_CONTROLLER.off_duration_us.get() + 1;
        fake_clock::set_ticks(new_time as u64);

        // Execute
        led_indicate_step();

        // Verify: Pattern number incremented and blink completed
        assert_eq!(LED_CONTROLLER.pattern_number.get(), 1);
        assert_eq!(
            LED_CONTROLLER.blink_count.get(),
            0,
            "Blink should be complete"
        );
        // The completion route restores the lamp's on/off state through the real
        // `light_onoff_hw(false)`: the brightness target stays zero, which the
        // driver turns into a zero cool-white duty and the warm-white channel's
        // inverted full duty.
        assert!(
            is_light_off(),
            "the completed indication must leave the lamp off"
        );
        assert_eq!(
            pwm_writes(PWMID_G).last().copied(),
            Some(0),
            "the restore must drive cool white to zero"
        );
        assert_eq!(
            pwm_writes(PWMID_B).last().copied(),
            Some(PMW_MAX_TICK as u32),
            "the restore must return warm white to the inverted zero duty"
        );
    }

    #[test]
    fn test_proc_led_adjusts_ww_light() {
        // Reset LED_CONTROLLER to clean state
        reset_fakes();

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
        fake_clock::set_ticks(new_time as u64);

        // Execute
        led_indicate_step();

        // Verify: the warm-white channel is driven lit (covers the BIT!(1)
        // branch) and cool white is never touched.
        // This happens because led_on = true (is_on was 0 and on_duration_us != 0)
        assert_eq!(
            pwm_writes(PWMID_B),
            vec![LED_LIT_WW],
            "warm white must be driven lit exactly once"
        );
        assert!(
            pwm_writes(PWMID_G).is_empty(),
            "cool white must not be touched when BIT!(0) is clear"
        );

        // Cleanup
        reset_for_test();
    }

    #[test]
    fn test_proc_led_adjusts_led_bit5() {
        // Reset LED_CONTROLLER to clean state
        reset_fakes();

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
        fake_clock::set_ticks(new_time as u64);

        // Execute
        led_indicate_step();

        // Verify: neither lamp channel may be driven, because the only selected
        // bit (BIT!(5)) has no lamp of its own.
        // led_off will be true: (is_on != 0 || on_duration == 0) && off_duration != 0
        // = (1 || true) && true = true
        // So we enter the if led_off || led_on block, but only BIT!(5) is handled (empty)
        assert!(
            pwm_writes(PWMID_G).is_empty(),
            "cool white must not be driven"
        );
        assert!(
            pwm_writes(PWMID_B).is_empty(),
            "warm white must not be driven"
        );

        // Cleanup
        reset_for_test();
    }

    #[test]
    fn test_proc_led_skips_adjustment_when_both_durations_zero() {
        // Reset LED_CONTROLLER to clean state
        reset_fakes();

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
        fake_clock::set_ticks(new_time as u64);

        // Execute
        led_indicate_step();

        // Verify: No lamp channel may be driven because:
        // led_on = (is_on == 0 && on_duration != 0) = (1 == 0 && 0 != 0) = false
        // led_off = ((is_on != 0 || on_duration == 0) && off_duration != 0) = ((1 || 1) && 0) = false
        // So the if (led_off || led_on) condition at line 182 is false, skipping all adjustments
        assert!(
            pwm_writes(PWMID_G).is_empty(),
            "cool white must not be driven"
        );
        assert!(
            pwm_writes(PWMID_B).is_empty(),
            "warm white must not be driven"
        );
    }

    #[test]
    fn test_led_controller_state_new_initializes_zero() {
        let state = LedControllerState::new();

        assert_eq!(state.event_pending.load(Ordering::Relaxed), 0);
        assert_eq!(state.blink_count.load(Ordering::Relaxed), 0);
        assert_eq!(state.on_duration_us.load(Ordering::Relaxed), 0);
        assert_eq!(state.off_duration_us.load(Ordering::Relaxed), 0);
        assert_eq!(state.led_selection_mask.load(Ordering::Relaxed), 0);
        assert_eq!(state.timing_tick.load(Ordering::Relaxed), 0);
        assert_eq!(state.pattern_number.load(Ordering::Relaxed), 0);
        assert_eq!(state.is_on.load(Ordering::Relaxed), 0);
    }

    // --- Tests for build_vendor_adv_data ---

    #[test]
    fn test_build_vendor_adv_data_fills_every_field() {
        let mac = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        let (pri, rsp) = build_vendor_adv_data(mac, 0x0211, 0x1F00);
        let expected_mac = u32::from_le_bytes([0x11, 0x22, 0x33, 0x44]);

        // Copy packed fields to locals before comparing: a reference to a packed
        // field is unaligned, so `assert_eq!` cannot take one directly.
        let (pri_manufacture, pri_uuid, pri_mac) =
            (pri.manufacture_id, pri.mesh_product_uuid, pri.mac_address);
        let (rsp_manufacture, rsp_uuid, rsp_mac) =
            (rsp.manufacture_id, rsp.mesh_product_uuid, rsp.mac_address);
        let (product_uuid, status, device_address, rsv) =
            (rsp.product_uuid, rsp.status, rsp.device_address, rsp.rsv);

        assert_eq!(pri_manufacture, 0x0211, "manufacturer id is the vendor id");
        assert_eq!(pri_uuid, 0x0211, "mesh product uuid is the vendor id");
        assert_eq!(pri_mac, expected_mac, "first four MAC bytes, little-endian");

        assert_eq!(rsp_manufacture, 0x0211, "manufacturer id is the vendor id");
        assert_eq!(rsp_uuid, 0x0211, "mesh product uuid is the vendor id");
        assert_eq!(rsp_mac, expected_mac, "same node id as the primary payload");
        assert_eq!(product_uuid, 0x02, "product uuid 0x02 = CCT light");
        assert_eq!(status, 0x01, "status 0x01 = advertising");
        assert_eq!(device_address, 0x1F00, "the mesh device address");
        for i in 0..16 {
            assert_eq!(rsv[i], i as u8, "rsv[{i}] must be {i}");
        }
    }

    #[test]
    fn test_build_vendor_adv_data_with_zero_mac_prefix() {
        // A zero MAC prefix must still overwrite whatever the caller holds.
        let (pri, rsp) = build_vendor_adv_data([0x00, 0x00, 0x00, 0x00, 0x05, 0x06], 0x0211, 0);

        let (pri_mac, rsp_mac, device_address, product_uuid) = (
            pri.mac_address,
            rsp.mac_address,
            rsp.device_address,
            rsp.product_uuid,
        );

        assert_eq!(pri_mac, 0);
        assert_eq!(rsp_mac, 0);
        assert_eq!(device_address, 0);
        assert_eq!(product_uuid, 0x02, "product uuid never comes from the MAC");
    }

    #[test]
    fn test_adv_private_to_bytes_is_little_endian_and_eight_bytes() {
        let pri = AdvPrivate {
            manufacture_id: 0xABCD,
            mesh_product_uuid: 0xEF01,
            mac_address: 0x44332211,
        };

        let bytes = adv_private_to_bytes(&pri);

        assert_eq!(bytes.len(), 8, "the primary payload is eight bytes on air");
        assert_eq!(
            bytes,
            [0xCD, 0xAB, 0x01, 0xEF, 0x11, 0x22, 0x33, 0x44],
            "each field is little-endian, in declaration order"
        );
    }

    #[test]
    fn test_adv_private_to_bytes_matches_the_built_payload() {
        // The serialiser and the builder must agree byte for byte: this is the
        // payload the RF driver copies into the advertising packet.
        let (pri, _rsp) = build_vendor_adv_data([0x01, 0x02, 0x03, 0x04, 0x05, 0x06], 0x0211, 0x1F);

        let bytes = adv_private_to_bytes(&pri);

        assert_eq!(
            bytes,
            [0x11, 0x02, 0x11, 0x02, 0x01, 0x02, 0x03, 0x04],
            "vendor id twice, then the four MAC bytes"
        );
    }
}
