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
//! - OTA timeout countdown driven by `TMR0` elapsed time

use crate::product::LightPlatformRadio;
use mesh_protocol::mesh_types::OtaState;
use tlsr8266_hal::mcu::irq_i::read_irq_sources;
use tlsr8266_hal::mcu::timer::{tmr0_ack_irq, tmr1_ack_irq};
use tlsr8266_radio::irq::BleConnectionEvent;

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
    tlsr8266_radio::irq::handle_rf_irq();

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

        // Mesh OTA supervision countdown (also extends the 64-bit clock).
        if let Some(state) = crate::ota::on_1s_tick() {
            crate::ota::rf_link_slave_ota_finish_led_and_reboot(state);
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

/// Handles one `SYSTEM_TIMER` interrupt: the BLE state machine plus the OTA
/// finish arms its event reports.
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
    match tlsr8266_radio::irq::handle_ble_system_timer_irq::<LightPlatformRadio>() {
        BleConnectionEvent::None => {}
        BleConnectionEvent::OtaFinishCheck => {
            crate::ota::rf_link_slave_ota_finish_handle();
        }
        BleConnectionEvent::OtaErrorReboot => {
            crate::ota::rf_link_slave_ota_finish_led_and_reboot(OtaState::Error);
        }
        BleConnectionEvent::OtaFinishCheckAndErrorReboot => {
            // rf_link_slave_ota_finish_handle() -> rf_link_slave_data_ota_save() can
            // refresh the counter to 30s, so it MUST run before the zero check.
            crate::ota::rf_link_slave_ota_finish_handle();
            if crate::ota::is_timeout_expired() {
                crate::ota::rf_link_slave_ota_finish_led_and_reboot(OtaState::Error);
            }
        }
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::config::FLASH_ADR_LIGHT_NEW_FW;
    use crate::ota::flash_layout::{
        ERASE_SECTORS_FOR_OTA, FLASH_ADR_OTA_READY_FLAG, FLASH_OTA_READY_FLAG,
    };
    use mesh_protocol::crc::crc16;
    use mesh_protocol::mesh_types::{RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS, START_UP_FLAG};
    use mesh_protocol::packet_types::{Packet, PacketAttData};
    use tlsr8266_hal::common::sync::SimplifyLS;
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog;
    use tlsr8266_hal::mcu::clock::{fake_clock, CLOCK_SYS_CLOCK_1S};
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_pac::{
        write_reg_irq_src, write_reg_rf_irq_status, FLD_IRQ, FLD_RF_IRQ_MASK, FLD_TMR_STA,
    };
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
    const REG_SYSTEM_TICK_IRQ: u32 = 0x744;
    /// `reg_pwdn_ctrl`: the MCU reset latch `light_sw_reboot` writes.
    const REG_PWDN_CTRL: u32 = 0x6f;
    /// `FldPwdnCtrl::Reboot`: the bit that triggers the reset.
    const PWDN_CTRL_REBOOT: u32 = 0x20;
    /// `reg_pwm_cmp` base: the lamp driver's per-channel duty writes.
    const PWM_CMP_BASE: u32 = 0x794;
    /// `reg_dma2_addr`: the receive DMA pointer the packet handler re-points.
    const REG_DMA2_ADDR: u32 = 0x508;

    /// Timeout used by the OTA countdown tests; any non-zero value that cannot be
    /// decremented to zero in a single interrupt works.
    const TEST_OTA_TIMEOUT_SECONDS: u16 = 7;

    /// Tick used by the connected-state tests.
    ///
    /// Deliberately **below one second**: `handle_ota_operations` gates its
    /// countdown on a process-global tick stamp that is compared by plain
    /// subtraction, so staying under one second keeps this module from moving
    /// that stamp (and from underflowing it) and leaves the countdown to the
    /// `sdk::ble_app::irq::connection` tests that own it.
    const CONNECTED_TICK: u64 = 15_000;

    /// The values written to the register at `offset`, oldest first.
    fn writes_to(offset: u32) -> Vec<u32> {
        FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == (offset | tlsr8266_pac::REG_BASE_ADDR))
            .map(|w| w.value)
            .collect()
    }

    /// Restores every process-wide singleton between tests: the register bank,
    /// the analog plane, the clock, the NOR device, the OTA/radio/light/queue
    /// state and the RF operation state.
    ///
    /// Tests run in one process with `--test-threads=1`, so process-wide
    /// resetting is safe.
    fn reset_irq_state() {
        FakeMmio::reset();
        FakeAnalog::reset();
        fake_clock::reset();
        FakeFlash::reset();
        crate::ota::reset_for_test();
        // Start from the radio's host-build defaults (both hook slots and the
        // in-crate OTA stand-in), so a recording hook installed by an earlier
        // test cannot leak into this one. Called before the install below, which
        // is exactly the hook this fixture wants.
        tlsr8266_radio::callbacks::reset_for_test();
        // The radio's OTA seam must carry the *real* hooks: the radio is built as
        // a dependency here, so its own host-build default is the in-crate
        // stand-in, and these tests assert the composition root's reaction to the
        // real snapshot/tick.
        crate::ota::install_hooks_for_test();
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

    /// The reboot latch fired exactly once: the observable of
    /// `rf_link_slave_ota_finish_led_and_reboot`.
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
    ///
    /// The composition root reacts to the [`BleConnectionEvent`] that handler
    /// reports, and that report cannot be stubbed across the radio-crate
    /// boundary, so each OTA scenario drives the connection state that really
    /// produces the event.
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

    /// Tests that the OTA tick is restamped while OTA is inactive.
    ///
    /// Acceptance: no countdown may happen outside OTA, and restamping on every
    /// interrupt is what prevents an immediate off-by-one decrement on the first
    /// interrupt after OTA begins.
    #[test]
    fn test_tmr0_irq_restamps_ota_tick_when_ota_inactive() {
        reset_irq_state();
        crate::ota::set_timeout_for_test(TEST_OTA_TIMEOUT_SECONDS);
        FakeMmio::set_reg32(REG_IRQ_SRC, FLD_IRQ::TMR0_EN.bits());
        FakeMmio::set_reg8(REG_TMR_STA, 0);

        fake_clock::set_ticks(50_000);

        handle_system_interrupts();

        assert_eq!(
            crate::ota::last_ota_timeout_tick_for_test(),
            50_000,
            "OTA tick should track the latest interrupt while OTA is inactive"
        );
        assert_eq!(
            crate::ota::timeout_seconds_for_test(),
            TEST_OTA_TIMEOUT_SECONDS,
            "OTA inactive must not decrement the timeout"
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
        crate::ota::set_mesh_ota_blocked_for_test(true);
        crate::ota::set_timeout_for_test(TEST_OTA_TIMEOUT_SECONDS);
        crate::ota::set_last_ota_timeout_tick_for_test(1_000);
        FakeMmio::set_reg32(REG_IRQ_SRC, FLD_IRQ::TMR0_EN.bits());
        FakeMmio::set_reg8(REG_TMR_STA, 0);

        fake_clock::set_ticks(1_000 + CLOCK_SYS_CLOCK_1S as u64 - 1);

        handle_system_interrupts();

        assert_eq!(
            crate::ota::last_ota_timeout_tick_for_test(),
            1_000,
            "OTA tick should not move before one second of clock ticks elapses"
        );
        assert_eq!(
            crate::ota::timeout_seconds_for_test(),
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
    /// Acceptance: during OTA the timeout still counts real seconds, decrements by
    /// exactly one per second, and restamps the tick that gates the next decrement.
    #[test]
    fn test_tmr0_irq_ota_elapsed_one_second_decrements_once() {
        reset_irq_state();
        crate::ota::set_mesh_ota_blocked_for_test(true);
        crate::ota::set_timeout_for_test(TEST_OTA_TIMEOUT_SECONDS);
        crate::ota::set_last_ota_timeout_tick_for_test(1_000);
        FakeMmio::set_reg32(REG_IRQ_SRC, FLD_IRQ::TMR0_EN.bits());
        FakeMmio::set_reg8(REG_TMR_STA, 0);

        let now = 1_000 + CLOCK_SYS_CLOCK_1S as u64;
        fake_clock::set_ticks(now);

        handle_system_interrupts();

        assert_eq!(
            crate::ota::last_ota_timeout_tick_for_test(),
            now,
            "OTA tick should be restamped when the timeout decrements"
        );
        assert_eq!(
            crate::ota::timeout_seconds_for_test(),
            TEST_OTA_TIMEOUT_SECONDS - 1,
            "exactly one second should be consumed by one interrupt"
        );
        assert_eq!(
            FakeMmio::reg8(REG_TMR_STA),
            FLD_TMR_STA::TMR0.bits(),
            "the TMR0 status bit must be acknowledged"
        );
    }

    /// Tests that the OTA timeout drives the *real* reboot path with the finish
    /// state it copied out of the OTA lock.
    ///
    /// Acceptance: the reboot runs with `OtaState::Ok` — the ready sentinel is
    /// written and no staging sector is erased — and the reset is latched
    /// exactly once. The state is no longer a stub argument, so the observable
    /// is the effect unique to each arm of the real callee.
    ///
    /// The old mock body also re-read the flag to probe for a guard held across
    /// the call. The real `Ok` path (`rf_ota_set_flag`, the LED blink) never
    /// re-locks that mutex, so the probe cannot fire from here; the re-entrancy
    /// guarantee stays covered by `ota`'s `rf_link_slave_ota_finish_handle`
    /// tests, whose real callee does re-lock it.
    #[test]
    fn test_tmr0_irq_ota_timeout_finishes_ok_and_reboots() {
        reset_irq_state();
        crate::ota::set_mesh_ota_blocked_for_test(true);
        crate::ota::set_timeout_for_test(1);
        crate::ota::set_last_ota_timeout_tick_for_test(1_000);
        crate::ota::set_finished_flag_for_test(OtaState::Ok);
        // A staged image whose firmware flag `rf_ota_set_flag` accepts.
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

    /// `OtaFinishCheck` polls the finish handler and the poll itself decides not
    /// to reboot.
    ///
    /// The event comes from the real connected-state handler: an active OTA with
    /// a non-zero supervision counter reports `OtaFinishCheck`, and that arm also
    /// schedules the next connection event and moves the radio to RX — those two
    /// effects are what prove the event, since the event value itself cannot be
    /// stubbed across the radio-crate boundary. While the transfer flag is
    /// `Continue` the polled handler has no effect of its own (nothing staged,
    /// no terminate request), so the composition root's decision is pinned by
    /// the absence of a reboot, and `ota`'s tests cover the handler's arms.
    #[test]
    fn test_system_timer_irq_ota_finish_check_polls_without_rebooting() {
        reset_irq_state();
        crate::ota::set_in_progress_for_test(true);
        crate::ota::set_timeout_for_test(TEST_OTA_TIMEOUT_SECONDS);
        // No supervision timeout: the connected handler must reach the OTA arm.
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(10_000_000);
        SLAVE_NEXT_CONNECT_TICK.set(15_000);
        fake_clock::set_ticks(CONNECTED_TICK);

        dispatch_connected_system_timer();

        assert_eq!(
            writes_to(REG_SYSTEM_TICK_IRQ),
            vec![15_000],
            "the OTA-active arm must schedule the next connection event"
        );
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Receiving,
            "the OTA-active arm must return the radio to RX"
        );
        // The finish handler ran with the transfer still in flight: nothing to
        // reboot, and nothing erased.
        assert_no_reboot();
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_LIGHT_NEW_FW),
            0,
            "an in-flight OTA must not erase its staging area"
        );
    }

    /// `OtaErrorReboot` reboots with `OtaState::Error` without polling the finish
    /// handler first.
    ///
    /// The real supervision timeout during an active OTA produces the event; the
    /// `Error` arm of the reboot path erases the whole staging area and runs the
    /// sixty-step error blink, so both are pinned.
    #[test]
    fn test_system_timer_irq_ota_error_reboots_with_error_state() {
        reset_irq_state();
        crate::ota::set_in_progress_for_test(true);
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(100);
        SLAVE_CONNECTED_TICK.set(10_000);
        fake_clock::set_ticks(15_000);

        dispatch_connected_system_timer();

        // The timeout path ran its cleanup before reporting the verdict.
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Advertising,
            "the supervision timeout must run the disconnection cleanup"
        );
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_LIGHT_NEW_FW + (ERASE_SECTORS_FOR_OTA - 1) * 0x1000),
            1,
            "the Error finish must erase the last staging sector"
        );
        assert_eq!(
            fake_clock::sleeps(),
            vec![100 * 1000u32; 60],
            "the Error finish must run the sixty-step error blink"
        );
        assert_reboot_latched();
    }

    /// `OtaFinishCheckAndErrorReboot` must reboot when the finish handler leaves the
    /// timeout counter at zero.
    #[test]
    fn test_system_timer_irq_finish_check_and_zero_counter_reboots() {
        reset_irq_state();
        crate::ota::set_in_progress_for_test(true);
        crate::ota::set_timeout_for_test(0);
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(10_000_000);
        fake_clock::set_ticks(CONNECTED_TICK);

        dispatch_connected_system_timer();

        // Nothing was staged, so the finish handler could not refresh the
        // counter: the zero check must take the error-reboot arm.
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_LIGHT_NEW_FW),
            1,
            "the error reboot must erase the staging area"
        );
        assert_eq!(
            fake_clock::sleeps(),
            vec![100 * 1000u32; 60],
            "the error reboot must run the sixty-step error blink"
        );
        assert_reboot_latched();
    }

    /// `OtaFinishCheckAndErrorReboot` must NOT reboot when the finish handler
    /// refreshes the timeout counter (e.g. `rf_link_slave_data_ota_save()` sets it
    /// back to 30s), which is why the handler runs before the zero check.
    #[test]
    fn test_system_timer_irq_finish_check_refreshed_counter_skips_reboot() {
        reset_irq_state();
        // A real staged bridge-mode chunk: the finish handler saves it, and the
        // save path is what refreshes the supervision counter.
        crate::ota::set_gatt_ota_begin_for_test(true);
        let mut staged = PacketAttData::default();
        staged.head.l2cap_len = 7 + 16;
        staged.dat[0] = 0; // cur_idx 0: first data packet
        staged.dat[1] = 0;
        staged.dat[2..18].copy_from_slice(&[0x5A; 16]);
        let crc = crc16(&staged.dat[0..18]).to_le_bytes();
        staged.dat[18..20].copy_from_slice(&crc);
        crate::ota::on_gatt_ota_packet(&Packet { att_data: staged });

        // The countdown has run out, so the handler is the last chance to
        // refresh it.
        crate::ota::set_timeout_for_test(0);
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(10_000_000);
        fake_clock::set_ticks(CONNECTED_TICK);

        dispatch_connected_system_timer();

        assert_eq!(
            crate::ota::timeout_seconds_for_test(),
            RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS,
            "the finish handler must have refreshed the counter"
        );
        let mut saved = [0u8; 16];
        FakeFlash::read_slice(FLASH_ADR_LIGHT_NEW_FW, &mut saved);
        assert_eq!(
            saved, [0x5A; 16],
            "the staged chunk must have been saved to the staging area"
        );
        assert_no_reboot();
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_LIGHT_NEW_FW),
            0,
            "a refreshed counter must not trigger the error reboot"
        );
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
