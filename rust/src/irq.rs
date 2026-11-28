//! # Interrupt Composition Root
//!
//! This module is the hardware interrupt entry point for the TLSR8266-based
//! mesh lighting system and the place where the individual interrupt sources
//! are fanned out to their owners.
//!
//! ## Why this lives outside the SDK
//!
//! Fanning hardware interrupts out to BLE/RF, the Embassy time driver, the
//! light transition engine and the UART transport is composition-root work. It
//! belongs in this module rather than inside the BLE link-layer SDK, so the
//! SDK keeps only pure BLE/RF helpers (`tlsr8266_radio::irq`).
//!
//! ## Key Responsibilities:
//! - Main interrupt entry point and dispatch logic
//! - RF interrupt handling (TX/RX complete) via the BLE/RF helpers
//! - System timer, Embassy timer, light transition and UART interrupt dispatch
//! - Mesh OTA timeout countdown driven by `TMR0` elapsed time

use crate::product::{LightPlatformRadio, LightProduct};
use tlsr8266_hal::mcu::irq_i::read_irq_sources;
use tlsr8266_hal::mcu::timer::{tmr0_ack_irq, tmr1_ack_irq};

/// Main interrupt handler for the BLE mesh lighting system.
///
/// This is the primary interrupt entry point that handles all system interrupts
/// including RF events, system timers, and application timers. It's executed
/// directly by the hardware interrupt controller.
///
/// ## Critical Performance Requirements:
/// - `#[no_mangle]`: Prevents name mangling for assembly linkage
/// - `#[link_section = ".ram_code"]`: Places code in RAM for fastest execution
/// - `extern "C"`: Uses C calling convention for hardware compatibility
///
/// ## Interrupt Types Handled:
/// 1. **RF Interrupts**: TX complete, RX complete for BLE/mesh communication
/// 2. **System Timer**: BLE state machine transitions (Adv, Bridge, Rx, Listen)
/// 3. **Timer 0**: Embassy timer alarms, clock overflow detection, OTA timeouts
/// 4. **Timer 1**: Light transition stepping for smooth dimming effects
/// 5. **UART**: Serial communication with external systems
///
/// ## Execution Flow:
/// 1. Handle high-priority RF interrupts (TX/RX)
/// 2. Process system timer and application interrupts
// no_mangle because this is referenced as an entrypoint from the assembler bootstrap
#[no_mangle]
#[link_section = ".ram_code"]
pub extern "C" fn irq_handler() {
    #[cfg(all(target_os = "none", feature = "rtt"))]
    let isr_start = tlsr8266_hal::runtime::executor::note_isr_start();

    // Handle RF (Radio Frequency) interrupts first (highest priority)
    tlsr8266_radio::irq::handle_rf_irq::<LightPlatformRadio>();

    // Process lower-priority interrupts
    handle_system_interrupts();

    #[cfg(all(target_os = "none", feature = "rtt"))]
    tlsr8266_hal::runtime::executor::note_isr_ticks(isr_start);
}

/// Handles lower-priority system and application interrupts.
/// Marked `#[inline(never)]` to keep the main interrupt handler small and fast.
#[export_name = "tlsr8266_handle_system_interrupts"]
#[inline(never)]
fn handle_system_interrupts() {
    // Get system interrupt sources
    let irq_source = read_irq_sources();

    // Handle system timer interrupt (BLE state machine)
    if irq_source.is_system_timer() {
        handle_system_timer_irq();
    }

    // Handle Timer 0: embassy timer alarms, clock overflow detection and OTA
    // timeout management. `TMR0` fires whenever the earliest embassy deadline or
    // the one-second clock-overflow cap is reached, so the OTA countdown is gated
    // on elapsed time instead of on the interrupt rate.
    if irq_source.is_tmr0() {
        tmr0_ack_irq();

        // Deliver expired embassy timers and re-arm TMR0 for the next deadline.
        tlsr8266_hal::runtime::time_driver::on_timer0_interrupt();

        // Mesh OTA supervision countdown (also extends the 64-bit clock). The
        // platform owns the countdown and the completion funnel.
        if let Some(state) = tlsr8266_platform::ota::on_1s_tick::<LightProduct>() {
            tlsr8266_platform::ota::finish_and_reboot::<LightProduct>(state);
        }
    }

    // Handle Timer 1: Light transition stepping for smooth dimming
    // This timer provides smooth light transitions by stepping through intermediate values
    if irq_source.is_tmr1() {
        tmr1_ack_irq();

        crate::light::transition_step();
    }

    // Handle UART interrupts for external communication
    crate::uart::take_rx_and_ack_tx_irq();
}

/// Handles one `SYSTEM_TIMER` interrupt: the BLE state machine.
///
/// # Why this is out of line
///
/// `handle_system_interrupts` keeps **one** frame shared by every interrupt
/// branch, but only the timer branch runs the BLE state machine. Once
/// `handle_ble_system_timer_irq` (and, through it, the whole connection state
/// machine) is inlined into that frame, its locals are charged to the IRQ stack
/// gate on a branch the gate's maximum never walks: measured, they were 248 B of
/// the handler's 320 B frame. Keeping the dispatch out of line moves them onto
/// the only branch that uses them, which drops the deepest IRQ chain from
/// 1396 B to 1340 B (margin 396 B -> 452 B).
///
/// Only the stack layout changes: the same handlers run, in the same order, for
/// the same interrupt source.
#[inline(never)]
fn handle_system_timer_irq() {
    tlsr8266_radio::irq::handle_ble_system_timer_irq::<LightPlatformRadio>();
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::config::FLASH_ADR_LIGHT_NEW_FW;
    use mesh_protocol::mesh_types::{OtaState, START_UP_FLAG};
    use tlsr8266_hal::common::sync::SimplifyLS;
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog;
    use tlsr8266_hal::mcu::clock::{fake_clock, CLOCK_SYS_CLOCK_1S};
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_pac::{
        write_reg_irq_src, write_reg_rf_irq_status, FLD_IRQ, FLD_RF_IRQ_MASK, FLD_TMR_STA,
    };
    use tlsr8266_platform::ota::engine;
    use tlsr8266_radio::ble::connection::{
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US, SLAVE_CONNECTED_TICK, SLAVE_NEXT_CONNECT_TICK,
    };
    use tlsr8266_radio::rf::drv_8266::CURRENT_RF_STATE;
    use tlsr8266_radio::types::RfOperationState;

    /// Datasheet offsets of the registers the composition root drives or
    /// acknowledges.
    const REG_RF_IRQ_STATUS: u32 = 0xf20;
    const REG_IRQ_SRC: u32 = 0x648;
    const REG_TMR_STA: u32 = 0x623;
    /// `reg_system_tick_irq`: the next system-timer compare value.
    const REG_SYSTEM_TICK_IRQ: u32 = 0x744;
    /// `reg_pwdn_ctrl`: the MCU reset latch the platform reboot path writes.
    const REG_PWDN_CTRL: u32 = 0x6f;
    /// `FldPwdnCtrl::Reboot`: the bit that triggers the reset.
    const PWDN_CTRL_REBOOT: u32 = 0x20;
    /// `reg_pwm_cmp` base: the lamp driver's per-channel duty writes.
    const PWM_CMP_BASE: u32 = 0x794;
    /// `reg_dma2_addr`: the receive DMA pointer the packet handler re-points.
    const REG_DMA2_ADDR: u32 = 0x508;

    /// The platform's ready-flag sentinel address and value.
    const FLASH_ADR_OTA_READY_FLAG: u32 = 0x3f000;
    const FLASH_OTA_READY_FLAG: u8 = 0xa5;

    /// Timeout used by the OTA countdown tests; any non-zero value that cannot be
    /// decremented to zero in a single interrupt works.
    const TEST_OTA_TIMEOUT_SECONDS: u16 = 7;

    /// The values written to the register at `offset`, oldest first.
    fn writes_to(offset: u32) -> Vec<u32> {
        FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == (offset | tlsr8266_pac::REG_BASE_ADDR))
            .map(|w| w.value)
            .collect()
    }

    /// Restores every process-wide singleton between tests: the register bank,
    /// the analog plane, the clock, the NOR device, the platform OTA state and
    /// the radio/light/queue state.
    ///
    /// Tests run in one process with `--test-threads=1`, so process-wide
    /// resetting is safe.
    fn reset_irq_state() {
        FakeMmio::reset();
        FakeAnalog::reset();
        fake_clock::reset();
        FakeFlash::reset();
        engine::reset_for_test();
        // Start from the radio's host-build defaults, so a recording hook
        // installed by an earlier test cannot leak into this one.
        tlsr8266_radio::callbacks::reset_for_test();
        tlsr8266_radio::reset_radio_state_for_test();
        tlsr8266_radio::queues::reset_for_test();
        crate::light::transitions::reset_for_test();
        // The receive buffer pointer is process-global and not part of any reset
        // helper, so pin it before a test drives the RX path.
        tlsr8266_radio::rf::drv_8266::LIGHT_RX_BUFFER_WRITE_POINTER.set(0);
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(0);
        SLAVE_CONNECTED_TICK.set(0);
        SLAVE_NEXT_CONNECT_TICK.set(0);
        *CURRENT_RF_STATE.lock() = RfOperationState::Idle;
    }

    /// The reboot latch fired exactly once: the observable of the platform's
    /// `finish_and_reboot`.
    fn assert_reboot_latched() {
        assert_eq!(
            writes_to(REG_PWDN_CTRL),
            vec![PWDN_CTRL_REBOOT],
            "the finish path must latch exactly one MCU reset"
        );
    }

    /// The reboot latch never fired.
    fn assert_no_reboot() {
        assert!(
            writes_to(REG_PWDN_CTRL).is_empty(),
            "no finish path may latch a reboot here"
        );
    }

    /// One `SYSTEM_TIMER` interrupt dispatched through the real connected-state
    /// handler.
    fn dispatch_connected_system_timer() {
        FakeMmio::set_reg32(REG_IRQ_SRC, FLD_IRQ::SYSTEM_TIMER.bits());
        *CURRENT_RF_STATE.lock() = RfOperationState::Connected;
        handle_system_interrupts();
    }

    // ================================================================================
    // Composition root
    // ================================================================================

    /// `irq_handler` must fan one pass out to the RF helper *and* the system
    /// interrupt body.
    ///
    /// All three hardware sources are pending in the same pass: the RF helper
    /// acknowledges TX and then RX (and re-points the receive DMA), and the
    /// system body acknowledges TMR1 and steps the light engine. The four
    /// per-arm RF-dispatch tests that used to live here are deleted: after the
    /// RF helper moved to the radio crate its arms are covered by
    /// `sdk::ble_app::irq::dispatcher`'s own tests, and what is left for this
    /// module to prove is the composition-root fan-out itself.
    #[test]
    fn test_irq_handler_dispatches_rf_and_system_sources_in_one_pass() {
        reset_irq_state();
        write_reg_rf_irq_status(FLD_RF_IRQ_MASK::IRQ_TX.bits() | FLD_RF_IRQ_MASK::IRQ_RX.bits());
        write_reg_irq_src(FLD_IRQ::TMR1_EN.bits());
        FakeMmio::clear_writes();

        irq_handler();

        assert_eq!(
            writes_to(REG_RF_IRQ_STATUS),
            vec![2, 1],
            "the RF helper must acknowledge TX, then RX"
        );
        assert!(
            !writes_to(REG_DMA2_ADDR).is_empty(),
            "the RX handler must re-point the receive DMA"
        );
        assert_eq!(
            writes_to(REG_TMR_STA),
            vec![u32::from(FLD_TMR_STA::TMR1.bits())],
            "the system body must acknowledge the TMR1 interrupt"
        );
        assert!(
            !writes_to(PWM_CMP_BASE).is_empty(),
            "the TMR1 branch must step the light transition engine"
        );
    }

    /// With no source pending, `irq_handler` must stay silent: no RF
    /// acknowledgement, no timer status clear, no light step.
    ///
    /// This is the composition root's half of the "absent sources" guarantee —
    /// the RF helper's own version of it lives with the code in the radio
    /// crate's dispatcher tests.
    #[test]
    fn test_irq_handler_does_not_acknowledge_absent_sources() {
        reset_irq_state();
        FakeMmio::clear_writes();

        irq_handler();

        assert!(
            writes_to(REG_RF_IRQ_STATUS).is_empty(),
            "no RF interrupt may be acknowledged when none is pending"
        );
        assert!(
            writes_to(REG_IRQ_SRC).is_empty(),
            "no system interrupt source may be cleared when none is pending"
        );
        assert!(
            writes_to(REG_TMR_STA).is_empty(),
            "no timer status may be acknowledged when none is pending"
        );
        assert!(
            writes_to(PWM_CMP_BASE).is_empty(),
            "the light engine must not be stepped when TMR1 is not pending"
        );
    }

    // ================================================================================
    // Tests for handle_system_interrupts function - Timer 0 (Embassy alarms & OTA)
    // ================================================================================

    /// Tests that a `TMR0` interrupt flushes the embassy time driver exactly once.
    ///
    /// Acceptance: the dispatcher must hand `TMR0` interrupts to the dynamic embassy
    /// time driver, which delivers expired timers and re-arms the hardware alarm.
    #[test]
    fn test_tmr0_irq_flushes_embassy_timers() {
        reset_irq_state();
        FakeMmio::set_reg32(REG_IRQ_SRC, FLD_IRQ::TMR0_EN.bits());
        FakeMmio::set_reg8(REG_TMR_STA, 0);

        fake_clock::set_ticks(1000);

        handle_system_interrupts();

        // The real embassy hook re-arms TMR0 for the next deadline: the tick is
        // cleared and the capture is programmed with the one-second cap.
        let writes = FakeMmio::writes();
        assert!(
            writes
                .iter()
                .any(|w| w.addr == 0x630 | tlsr8266_pac::REG_BASE_ADDR),
            "on_timer0_interrupt must clear the TMR0 tick"
        );
        assert!(
            writes
                .iter()
                .any(|w| w.addr == 0x624 | tlsr8266_pac::REG_BASE_ADDR
                    && w.value == CLOCK_SYS_CLOCK_1S),
            "on_timer0_interrupt must re-arm TMR0 with the one-second cap"
        );
        assert_eq!(
            FakeMmio::reg8(REG_TMR_STA),
            FLD_TMR_STA::TMR0.bits(),
            "the TMR0 status bit must be acknowledged"
        );
    }

    /// Tests that the OTA tick is restamped while no mesh OTA is running.
    ///
    /// Acceptance: no countdown may happen outside a transfer, and restamping on
    /// every interrupt is what prevents an immediate off-by-one decrement on the
    /// first interrupt after a transfer begins.
    #[test]
    fn test_tmr0_irq_restamps_ota_tick_when_no_mesh_ota() {
        reset_irq_state();
        engine::set_timeout_for_test(TEST_OTA_TIMEOUT_SECONDS);
        FakeMmio::set_reg32(REG_IRQ_SRC, FLD_IRQ::TMR0_EN.bits());
        FakeMmio::set_reg8(REG_TMR_STA, 0);

        fake_clock::set_ticks(50_000);

        handle_system_interrupts();

        assert_eq!(
            engine::last_ota_timeout_tick_for_test(),
            50_000,
            "OTA tick should track the latest interrupt while no transfer runs"
        );
        assert_eq!(
            engine::timeout_seconds_for_test(),
            TEST_OTA_TIMEOUT_SECONDS,
            "an idle tick must not decrement the timeout"
        );
        assert_eq!(
            FakeMmio::reg8(REG_TMR_STA),
            FLD_TMR_STA::TMR0.bits(),
            "the TMR0 status bit must be acknowledged"
        );
    }

    /// Tests that the OTA countdown stays idle until a full second has elapsed.
    ///
    /// Acceptance: `TMR0` now fires at embassy alarm deadlines, so the countdown
    /// must be gated on elapsed clock ticks rather than on interrupt frequency.
    #[test]
    fn test_tmr0_irq_ota_elapsed_under_one_second_does_not_decrement() {
        reset_irq_state();
        engine::set_mesh_ota_blocked_for_test(true);
        engine::set_timeout_for_test(TEST_OTA_TIMEOUT_SECONDS);
        engine::set_last_ota_timeout_tick_for_test(1_000);
        FakeMmio::set_reg32(REG_IRQ_SRC, FLD_IRQ::TMR0_EN.bits());
        FakeMmio::set_reg8(REG_TMR_STA, 0);

        fake_clock::set_ticks(1_000 + CLOCK_SYS_CLOCK_1S as u64 - 1);

        handle_system_interrupts();

        assert_eq!(
            engine::last_ota_timeout_tick_for_test(),
            1_000,
            "OTA tick should not move before one second of clock ticks elapses"
        );
        assert_eq!(
            engine::timeout_seconds_for_test(),
            TEST_OTA_TIMEOUT_SECONDS,
            "OTA timeout must not decrement before one second elapses"
        );
        assert_eq!(
            FakeMmio::reg8(REG_TMR_STA),
            FLD_TMR_STA::TMR0.bits(),
            "the TMR0 status bit must be acknowledged"
        );
    }

    /// Tests that one second of elapsed clock ticks decrements the OTA timeout once.
    ///
    /// Acceptance: during a mesh transfer the timeout still counts real seconds,
    /// decrements by exactly one per second, and restamps the tick that gates the
    /// next decrement.
    #[test]
    fn test_tmr0_irq_ota_elapsed_one_second_decrements_once() {
        reset_irq_state();
        engine::set_mesh_ota_blocked_for_test(true);
        engine::set_timeout_for_test(TEST_OTA_TIMEOUT_SECONDS);
        engine::set_last_ota_timeout_tick_for_test(1_000);
        FakeMmio::set_reg32(REG_IRQ_SRC, FLD_IRQ::TMR0_EN.bits());
        FakeMmio::set_reg8(REG_TMR_STA, 0);

        let now = 1_000 + CLOCK_SYS_CLOCK_1S as u64;
        fake_clock::set_ticks(now);

        handle_system_interrupts();

        assert_eq!(
            engine::last_ota_timeout_tick_for_test(),
            now,
            "OTA tick should be restamped when the timeout decrements"
        );
        assert_eq!(
            engine::timeout_seconds_for_test(),
            TEST_OTA_TIMEOUT_SECONDS - 1,
            "exactly one second should be consumed by one interrupt"
        );
        assert_eq!(
            FakeMmio::reg8(REG_TMR_STA),
            FLD_TMR_STA::TMR0.bits(),
            "the TMR0 status bit must be acknowledged"
        );
        assert_no_reboot();
    }

    /// Tests that the OTA timeout drives the *real* completion funnel with the
    /// finish state the platform copied out of its own lock.
    ///
    /// Acceptance: the reboot runs with `OtaState::Ok` — the ready sentinel is
    /// written and no staging sector is erased — and the reset is latched
    /// exactly once. The state is no longer a stub argument, so the observable is
    /// the effect unique to each arm of the platform funnel.
    #[test]
    fn test_tmr0_irq_ota_timeout_finishes_ok_and_reboots() {
        reset_irq_state();
        engine::set_mesh_ota_blocked_for_test(true);
        engine::set_timeout_for_test(1);
        engine::set_last_ota_timeout_tick_for_test(1_000);
        engine::set_finished_flag_for_test(OtaState::Ok);
        // A staged image whose firmware flag `arm_ready_flag` accepts.
        FakeFlash::write_slice(FLASH_ADR_LIGHT_NEW_FW + 8, &START_UP_FLAG.to_le_bytes());
        FakeMmio::set_reg32(REG_IRQ_SRC, FLD_IRQ::TMR0_EN.bits());
        FakeMmio::set_reg8(REG_TMR_STA, 0);

        fake_clock::set_ticks(1_000 + CLOCK_SYS_CLOCK_1S as u64);

        handle_system_interrupts();

        let mut ready = [0u8; 1];
        FakeFlash::read_slice(FLASH_ADR_OTA_READY_FLAG, &mut ready);
        assert_eq!(
            ready[0], FLASH_OTA_READY_FLAG,
            "the Ok finish must arm the boot-time OTA ready sentinel"
        );
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_LIGHT_NEW_FW),
            0,
            "the Ok finish must not erase the staging area"
        );
        assert!(
            fake_clock::sleeps() != vec![100 * 1000u32; 60],
            "the success indication is the HAL's blink, not the failure loop"
        );
        assert_reboot_latched();
    }

    /// A mesh supervision timeout that reaches zero without a decision must
    /// follow the deployed finish funnel exactly.
    ///
    /// Legacy contract: `on_1s_tick` returned the finish flag as-is — `Continue`
    /// when no chunk had decided the outcome — and
    /// `rf_link_slave_ota_finish_led_and_reboot` rebooted unconditionally. So
    /// the `Continue` case does **no** staging-area erase, **no** LED
    /// indication, and still latches the reset. The dirty staging area is left
    /// for the next boot's `check_ota_area_startup` to clean, and the ready
    /// sentinel is never armed because no image was ever claimed staged.
    #[test]
    fn test_tmr0_irq_ota_timeout_without_a_decision_reboots_without_erasing() {
        reset_irq_state();
        engine::set_mesh_ota_blocked_for_test(true);
        engine::set_timeout_for_test(1);
        engine::set_last_ota_timeout_tick_for_test(1_000);
        FakeFlash::write_slice(FLASH_ADR_LIGHT_NEW_FW, &[0x11, 0x22]);
        FakeMmio::set_reg32(REG_IRQ_SRC, FLD_IRQ::TMR0_EN.bits());
        FakeMmio::set_reg8(REG_TMR_STA, 0);

        // The boot sentinel starts unarmed (erased to 0xff).
        let mut ready_before = [0u8; 1];
        FakeFlash::read_slice(FLASH_ADR_OTA_READY_FLAG, &mut ready_before);

        fake_clock::set_ticks(1_000 + CLOCK_SYS_CLOCK_1S as u64);

        handle_system_interrupts();

        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_LIGHT_NEW_FW),
            0,
            "a no-decision timeout must not erase the staging area"
        );
        // The staged bytes survive: the area is left dirty on purpose.
        let mut staged = [0u8; 2];
        FakeFlash::read_slice(FLASH_ADR_LIGHT_NEW_FW, &mut staged);
        assert_eq!(
            staged,
            [0x11, 0x22],
            "the no-decision timeout leaves the dirty staging area for the next boot to clean"
        );
        assert_ne!(
            fake_clock::sleeps(),
            vec![100 * 1000u32; 60],
            "a no-decision timeout must not run the product's failure indication"
        );
        let mut ready_after = [0u8; 1];
        FakeFlash::read_slice(FLASH_ADR_OTA_READY_FLAG, &mut ready_after);
        assert_eq!(
            ready_after, ready_before,
            "a no-decision timeout must not touch the boot sentinel"
        );
        assert_ne!(
            ready_after[0], FLASH_OTA_READY_FLAG,
            "a no-decision timeout must not claim a staged image"
        );
        assert_reboot_latched();
    }

    // ================================================================================
    // Tests for handle_system_interrupts function - System timer & Timer 1
    // ================================================================================

    /// Tests that a `SYSTEM_TIMER` interrupt dispatches to the BLE state helper,
    /// which clears the source exactly once.
    ///
    /// The helper runs for real (its `Idle` arm only clears the source), so this
    /// pins the wiring `SYSTEM_TIMER` → helper → register clear through the
    /// write log; a dropped call at the dispatch site would leave the source
    /// uncleared.
    #[test]
    fn test_system_timer_irq_clears_source_and_dispatches_ble_helper() {
        reset_irq_state();
        *CURRENT_RF_STATE.lock() = RfOperationState::Idle;

        FakeMmio::set_reg32(REG_IRQ_SRC, FLD_IRQ::SYSTEM_TIMER.bits());

        handle_system_interrupts();

        let clears = FakeMmio::writes()
            .into_iter()
            .filter(|write| {
                write.addr == REG_IRQ_SRC | tlsr8266_pac::REG_BASE_ADDR
                    && write.value == FLD_IRQ::SYSTEM_TIMER.bits()
            })
            .count();
        assert_eq!(
            clears, 1,
            "the SYSTEM_TIMER source must be cleared exactly once"
        );
    }

    /// A `SYSTEM_TIMER` interrupt on a connected link no longer reports an OTA
    /// event up to this composition root: the platform services the connection
    /// slot itself. What is left here is the connected-state handler running.
    #[test]
    fn test_system_timer_irq_runs_the_connected_state_handler() {
        reset_irq_state();
        // No supervision timeout, and the next event is still comfortably ahead
        // of the safety margin: the connected handler reaches its slot work.
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(10_000_000);
        SLAVE_CONNECTED_TICK.set(19_900);
        SLAVE_NEXT_CONNECT_TICK.set(50_000);
        fake_clock::set_ticks(20_000);

        dispatch_connected_system_timer();

        assert_eq!(
            writes_to(REG_SYSTEM_TICK_IRQ).last(),
            Some(&50_000),
            "the connected slot must schedule the next connection event last"
        );
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Receiving,
            "the connected slot must return the radio to RX"
        );
        assert_no_reboot();
    }

    /// Tests that a `TMR1` interrupt acknowledges the timer and steps the light
    /// transition engine exactly once.
    ///
    /// Acceptance: `TMR1` drives smooth dimming, so a missing acknowledge would
    /// re-enter the handler forever and a missing step would freeze transitions.
    /// The step is observed through the real engine's PWM duty writes.
    #[test]
    fn test_tmr1_irq_clears_status_and_steps_light_transition() {
        reset_irq_state();
        FakeMmio::set_reg32(REG_IRQ_SRC, FLD_IRQ::TMR1_EN.bits());
        FakeMmio::set_reg8(REG_TMR_STA, 0);

        handle_system_interrupts();

        assert_eq!(
            FakeMmio::reg8(REG_TMR_STA),
            FLD_TMR_STA::TMR1.bits(),
            "the TMR1 status bit must be acknowledged"
        );
        assert_eq!(
            writes_to(PWM_CMP_BASE).len(),
            1,
            "the light transition step must drive the cool-white duty exactly once"
        );
    }
}
