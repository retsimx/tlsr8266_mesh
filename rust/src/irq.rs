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
//! SDK keeps only pure BLE/RF helpers (`crate::sdk::ble_app::irq`).
//!
//! ## Key Responsibilities:
//! - Main interrupt entry point and dispatch logic
//! - RF interrupt handling (TX/RX complete) via the BLE/RF helpers
//! - System timer, Embassy timer, light transition and UART interrupt dispatch
//! - OTA timeout countdown driven by `TMR0` elapsed time

use crate::sdk::ble_app::irq::BleConnectionEvent;
use mesh_protocol::mesh_types::OtaState;
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
    // Handle RF (Radio Frequency) interrupts first (highest priority)
    crate::sdk::ble_app::irq::handle_rf_irq();

    // Process lower-priority interrupts
    handle_system_interrupts();
}

/// Handles lower-priority system and application interrupts.
/// Marked `#[inline(never)]` to keep the main interrupt handler small and fast.
#[inline(never)]
#[cfg_attr(test, mry::mry)]
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
    crate::uart_manager::check_irq();
}

/// Handles one `SYSTEM_TIMER` interrupt: the BLE state machine plus the OTA
/// finish arms its event reports.
///
/// # Why this is out of line
///
/// `handle_system_interrupts` keeps **one** frame shared by every interrupt
/// branch, but the deepest chain (UART) never runs the BLE state machine. Once
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
    match crate::sdk::ble_app::irq::handle_ble_system_timer_irq() {
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
    use mry::Any;

    // Import mock functions from their original modules
    use crate::light::transitions::{mock_transition_step, transition_step};
    use crate::ota::{
        mock_rf_link_slave_ota_finish_handle, mock_rf_link_slave_ota_finish_led_and_reboot,
        rf_link_slave_ota_finish_handle, rf_link_slave_ota_finish_led_and_reboot,
    };
    use crate::sdk::ble_app::irq::dispatcher::{
        handle_ble_system_timer_irq, handle_rf_irq, mock_handle_ble_system_timer_irq,
        mock_handle_rf_irq,
    };
    use crate::sdk::ble_app::irq::packet_handling::{
        handle_rf_packet_reception, handle_rf_transmission_complete,
        mock_handle_rf_packet_reception, mock_handle_rf_transmission_complete,
    };
    use crate::sdk::ble_app::rf_drv_8266::CURRENT_RF_STATE;
    use crate::sdk::light::RfOperationState;
    use crate::uart_manager::{check_irq, mock_check_irq};
    use mesh_protocol::mesh_types::RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS;
    use tlsr8266_hal::mcu::clock::{fake_clock, CLOCK_SYS_CLOCK_1S};
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_pac::{
        mock_read_reg_rf_irq_status, mock_write_reg_irq_src, read_reg_rf_irq_status,
        write_reg_irq_src, FLD_IRQ, FLD_RF_IRQ_MASK, FLD_TMR_STA,
    };

    /// Timeout used by the OTA countdown tests; any non-zero value that cannot be
    /// decremented to zero in a single interrupt works.
    const TEST_OTA_TIMEOUT_SECONDS: u16 = 7;

    /// Restores the OTA subsystem and the HAL seams between tests.
    ///
    /// Tests run in one process with `--test-threads=1`, so process-wide resetting
    /// is safe.
    fn reset_ota_timeout_state() {
        crate::ota::reset_for_test();
        // The HAL host seams are process-wide singletons, so every TMR0 test
        // starts from a zeroed bank, tick count and write log.
        FakeMmio::reset();
        fake_clock::reset();
    }

    // ================================================================================
    // Tests for irq_handler function (main interrupt handler)
    // ================================================================================

    /// Tests that the entry point delegates to the RF helper and then to the
    /// system interrupt handler, exactly once each.
    ///
    /// This pins the composition-root boundary: `irq_handler` itself owns no
    /// dispatch logic, it only fans out to the BLE/RF helper and the system
    /// interrupt body.
    #[test]
    #[mry::lock(handle_rf_irq, handle_system_interrupts)]
    fn test_irq_handler_delegates_to_rf_and_system_helpers() {
        mock_handle_rf_irq().returns(());
        mock_handle_system_interrupts().returns(());

        irq_handler();

        mock_handle_rf_irq().assert_called(1);
        mock_handle_system_interrupts().assert_called(1);
    }

    /// Tests RF transmission complete interrupt handling.
    ///
    /// Verifies that when an RF transmission complete interrupt occurs,
    /// the transmission complete handler is called and the interrupt
    /// status is properly managed.
    #[test]
    #[mry::lock(
        read_reg_rf_irq_status,
        handle_rf_transmission_complete,
        handle_system_interrupts
    )]
    fn test_irq_handler_rf_transmission_complete() {
        // Mock RF transmission complete interrupt
        mock_read_reg_rf_irq_status().returns(FLD_RF_IRQ_MASK::IRQ_TX.bits());
        mock_handle_rf_transmission_complete().returns(());
        mock_handle_system_interrupts().returns(());

        // Execute the interrupt handler
        irq_handler();

        // Verify the transmission complete handler was called
        mock_handle_rf_transmission_complete().assert_called(1);
        // Verify system interrupts were processed
        mock_handle_system_interrupts().assert_called(1);
    }

    /// Tests RF reception complete interrupt handling.
    ///
    /// Verifies that when an RF reception complete interrupt occurs,
    /// the packet reception handler is called and the interrupt
    /// status is properly managed.
    #[test]
    #[mry::lock(
        read_reg_rf_irq_status,
        handle_rf_packet_reception,
        handle_system_interrupts
    )]
    fn test_irq_handler_rf_reception_complete() {
        // Mock RF reception complete interrupt
        mock_read_reg_rf_irq_status().returns(FLD_RF_IRQ_MASK::IRQ_RX.bits());
        mock_handle_rf_packet_reception().returns(());
        mock_handle_system_interrupts().returns(());

        // Execute the interrupt handler
        irq_handler();

        // Verify the packet reception handler was called
        mock_handle_rf_packet_reception().assert_called(1);
        // Verify system interrupts were processed
        mock_handle_system_interrupts().assert_called(1);
    }

    /// Tests handling of both RF transmission and reception interrupts.
    ///
    /// Verifies that when both RF interrupts occur simultaneously,
    /// both handlers are called in the correct order.
    #[test]
    #[mry::lock(
        read_reg_rf_irq_status,
        handle_rf_transmission_complete,
        handle_rf_packet_reception,
        handle_system_interrupts
    )]
    fn test_irq_handler_rf_both_interrupts() {
        // Mock both RF interrupts
        let both_interrupts = FLD_RF_IRQ_MASK::IRQ_TX.bits() | FLD_RF_IRQ_MASK::IRQ_RX.bits();
        mock_read_reg_rf_irq_status().returns(both_interrupts);
        mock_handle_rf_transmission_complete().returns(());
        mock_handle_rf_packet_reception().returns(());
        mock_handle_system_interrupts().returns(());

        // Execute the interrupt handler
        irq_handler();

        // Verify both handlers were called
        mock_handle_rf_transmission_complete().assert_called(1);
        mock_handle_rf_packet_reception().assert_called(1);
        // Verify system interrupts were processed
        mock_handle_system_interrupts().assert_called(1);
    }

    /// Tests handling when no RF interrupts are present.
    ///
    /// Verifies that when no RF interrupts are set, no RF handlers
    /// are called but system interrupts are still processed.
    #[test]
    #[mry::lock(
        read_reg_rf_irq_status,
        handle_rf_transmission_complete,
        handle_rf_packet_reception,
        handle_system_interrupts
    )]
    fn test_irq_handler_no_rf_interrupts() {
        // Mock no RF interrupts
        mock_read_reg_rf_irq_status().returns(0);
        mock_handle_rf_transmission_complete().returns(());
        mock_handle_rf_packet_reception().returns(());
        mock_handle_system_interrupts().returns(());

        // Execute the interrupt handler
        irq_handler();

        // Verify no RF handlers were called (since no RF interrupts were set)
        mock_handle_rf_transmission_complete().assert_called(0);
        mock_handle_rf_packet_reception().assert_called(0);
        // Verify system interrupts were still processed
        mock_handle_system_interrupts().assert_called(1);
    }

    // ================================================================================
    // Tests for handle_system_interrupts function - Timer 0 (Embassy alarms & OTA)
    // ================================================================================

    /// Tests that a `TMR0` interrupt flushes the embassy time driver exactly once.
    ///
    /// Acceptance: the dispatcher must hand `TMR0` interrupts to the dynamic embassy
    /// time driver, which delivers expired timers and re-arms the hardware alarm.
    #[test]
    #[mry::lock(check_irq)]
    fn test_tmr0_irq_flushes_embassy_timers() {
        reset_ota_timeout_state();
        FakeMmio::set_reg32(0x648, FLD_IRQ::TMR0_EN.bits());
        FakeMmio::set_reg8(0x623, 0);

        fake_clock::set_ticks(1000);

        mock_check_irq().returns(());

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
            FakeMmio::reg8(0x623),
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
    #[mry::lock(check_irq)]
    fn test_tmr0_irq_restamps_ota_tick_when_ota_inactive() {
        reset_ota_timeout_state();
        crate::ota::set_timeout_for_test(TEST_OTA_TIMEOUT_SECONDS);
        FakeMmio::set_reg32(0x648, FLD_IRQ::TMR0_EN.bits());
        FakeMmio::set_reg8(0x623, 0);

        fake_clock::set_ticks(50_000);

        mock_check_irq().returns(());

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
            FakeMmio::reg8(0x623),
            FLD_TMR_STA::TMR0.bits(),
            "the TMR0 status bit must be acknowledged"
        );
    }

    /// Tests that the OTA countdown stays idle until a full second has elapsed.
    ///
    /// Acceptance: `TMR0` now fires at embassy alarm deadlines, so the countdown
    /// must be gated on elapsed clock ticks rather than on interrupt frequency.
    #[test]
    #[mry::lock(check_irq)]
    fn test_tmr0_irq_ota_elapsed_under_one_second_does_not_decrement() {
        reset_ota_timeout_state();
        crate::ota::set_mesh_ota_blocked_for_test(true);
        crate::ota::set_timeout_for_test(TEST_OTA_TIMEOUT_SECONDS);
        crate::ota::set_last_ota_timeout_tick_for_test(1_000);
        FakeMmio::set_reg32(0x648, FLD_IRQ::TMR0_EN.bits());
        FakeMmio::set_reg8(0x623, 0);

        fake_clock::set_ticks(1_000 + CLOCK_SYS_CLOCK_1S as u64 - 1);

        mock_check_irq().returns(());

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
            FakeMmio::reg8(0x623),
            FLD_TMR_STA::TMR0.bits(),
            "the TMR0 status bit must be acknowledged"
        );
    }

    /// Tests that one second of elapsed clock ticks decrements the OTA timeout once.
    ///
    /// Acceptance: during OTA the timeout still counts real seconds, decrements by
    /// exactly one per second, and restamps the tick that gates the next decrement.
    #[test]
    #[mry::lock(check_irq)]
    fn test_tmr0_irq_ota_elapsed_one_second_decrements_once() {
        reset_ota_timeout_state();
        crate::ota::set_mesh_ota_blocked_for_test(true);
        crate::ota::set_timeout_for_test(TEST_OTA_TIMEOUT_SECONDS);
        crate::ota::set_last_ota_timeout_tick_for_test(1_000);
        FakeMmio::set_reg32(0x648, FLD_IRQ::TMR0_EN.bits());
        FakeMmio::set_reg8(0x623, 0);

        let now = 1_000 + CLOCK_SYS_CLOCK_1S as u64;
        fake_clock::set_ticks(now);

        mock_check_irq().returns(());

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
            FakeMmio::reg8(0x623),
            FLD_TMR_STA::TMR0.bits(),
            "the TMR0 status bit must be acknowledged"
        );
    }

    /// Tests that the OTA timeout copies the finish flag out of its lock before
    /// handing it to the reboot path.
    ///
    /// Acceptance: `rf_link_slave_ota_finish_led_and_reboot` erases the OTA staging
    /// area and then blocks for six seconds blinking the error LED, so a
    /// OTA-flag guard held across the call would mask interrupts for that whole
    /// window. The mock body re-reads the snapshot (which re-locks the flag); the
    /// `cfg(test)` re-entrancy probe turns that into a panic if the guard is
    /// still alive.
    #[test]
    #[mry::lock(rf_link_slave_ota_finish_led_and_reboot, check_irq)]
    fn test_tmr0_irq_ota_timeout_copies_finish_flag_before_reboot() {
        reset_ota_timeout_state();
        crate::ota::set_mesh_ota_blocked_for_test(true);
        crate::ota::set_timeout_for_test(1);
        crate::ota::set_last_ota_timeout_tick_for_test(1_000);
        crate::ota::set_finished_flag_for_test(OtaState::Ok);
        FakeMmio::set_reg32(0x648, FLD_IRQ::TMR0_EN.bits());
        FakeMmio::set_reg8(0x623, 0);

        fake_clock::set_ticks(1_000 + CLOCK_SYS_CLOCK_1S as u64);
        mock_rf_link_slave_ota_finish_led_and_reboot(Any).returns_with(|st: OtaState| {
            // Panics if the caller still holds the flag's critical section.
            let _probe = crate::ota::ota_status_snapshot_impl();
            assert_eq!(st, OtaState::Ok, "the reboot must see the copied flag");
        });
        mock_check_irq().returns(());

        handle_system_interrupts();

        mock_rf_link_slave_ota_finish_led_and_reboot(OtaState::Ok).assert_called(1);
        assert_eq!(
            FakeMmio::reg8(0x623),
            FLD_TMR_STA::TMR0.bits(),
            "the TMR0 status bit must be acknowledged"
        );
        crate::ota::set_finished_flag_for_test(OtaState::Continue);
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
    #[mry::lock(check_irq)]
    fn test_system_timer_irq_clears_source_and_dispatches_ble_helper() {
        *CURRENT_RF_STATE.lock() = RfOperationState::Idle;

        FakeMmio::reset();
        FakeMmio::set_reg32(0x648, FLD_IRQ::SYSTEM_TIMER.bits());

        mock_check_irq().returns(());

        handle_system_interrupts();

        let clears = FakeMmio::writes()
            .into_iter()
            .filter(|write| {
                write.addr == 0x648 | tlsr8266_pac::REG_BASE_ADDR
                    && write.value == FLD_IRQ::SYSTEM_TIMER.bits()
            })
            .count();
        assert_eq!(
            clears, 1,
            "the SYSTEM_TIMER source must be cleared exactly once"
        );
    }

    /// `OtaFinishCheck` only polls the OTA finish handler; nothing reboots.
    #[test]
    #[mry::lock(
        write_reg_irq_src,
        handle_ble_system_timer_irq,
        rf_link_slave_ota_finish_handle,
        rf_link_slave_ota_finish_led_and_reboot,
        check_irq
    )]
    fn test_system_timer_irq_ota_finish_check_only_polls() {
        FakeMmio::set_reg32(0x648, FLD_IRQ::SYSTEM_TIMER.bits());
        mock_write_reg_irq_src(Any).returns(());
        mock_handle_ble_system_timer_irq().returns(BleConnectionEvent::OtaFinishCheck);

        mock_rf_link_slave_ota_finish_handle().returns(());
        mock_rf_link_slave_ota_finish_led_and_reboot(Any).returns(());
        mock_check_irq().returns(());

        handle_system_interrupts();

        mock_rf_link_slave_ota_finish_handle().assert_called(1);
        mock_rf_link_slave_ota_finish_led_and_reboot(Any).assert_called(0);
    }

    /// `OtaErrorReboot` reboots with `OtaState::Error` without polling the finish
    /// handler first.
    #[test]
    #[mry::lock(
        write_reg_irq_src,
        handle_ble_system_timer_irq,
        rf_link_slave_ota_finish_handle,
        rf_link_slave_ota_finish_led_and_reboot,
        check_irq
    )]
    fn test_system_timer_irq_ota_error_reboots_with_error_state() {
        FakeMmio::set_reg32(0x648, FLD_IRQ::SYSTEM_TIMER.bits());
        mock_write_reg_irq_src(Any).returns(());
        mock_handle_ble_system_timer_irq().returns(BleConnectionEvent::OtaErrorReboot);

        mock_rf_link_slave_ota_finish_handle().returns(());
        mock_rf_link_slave_ota_finish_led_and_reboot(Any).returns(());
        mock_check_irq().returns(());

        handle_system_interrupts();

        mock_rf_link_slave_ota_finish_led_and_reboot(OtaState::Error).assert_called(1);
        mock_rf_link_slave_ota_finish_handle().assert_called(0);
    }

    /// `OtaFinishCheckAndErrorReboot` must reboot when the finish handler leaves the
    /// timeout counter at zero.
    #[test]
    #[mry::lock(
        write_reg_irq_src,
        handle_ble_system_timer_irq,
        rf_link_slave_ota_finish_handle,
        rf_link_slave_ota_finish_led_and_reboot,
        check_irq
    )]
    fn test_system_timer_irq_finish_check_and_zero_counter_reboots() {
        reset_ota_timeout_state();
        crate::ota::set_timeout_for_test(0);

        FakeMmio::set_reg32(0x648, FLD_IRQ::SYSTEM_TIMER.bits());
        mock_write_reg_irq_src(Any).returns(());
        mock_handle_ble_system_timer_irq()
            .returns(BleConnectionEvent::OtaFinishCheckAndErrorReboot);

        // The finish handler does not refresh the counter in this scenario.
        mock_rf_link_slave_ota_finish_handle().returns(());
        mock_rf_link_slave_ota_finish_led_and_reboot(Any).returns(());
        mock_check_irq().returns(());

        handle_system_interrupts();

        mock_rf_link_slave_ota_finish_handle().assert_called(1);
        mock_rf_link_slave_ota_finish_led_and_reboot(OtaState::Error).assert_called(1);

        crate::ota::set_timeout_for_test(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);
    }

    /// `OtaFinishCheckAndErrorReboot` must NOT reboot when the finish handler
    /// refreshes the timeout counter (e.g. `rf_link_slave_data_ota_save()` sets it
    /// back to 30s), which is why the handler runs before the zero check.
    #[test]
    #[mry::lock(
        write_reg_irq_src,
        handle_ble_system_timer_irq,
        rf_link_slave_ota_finish_handle,
        rf_link_slave_ota_finish_led_and_reboot,
        check_irq
    )]
    fn test_system_timer_irq_finish_check_refreshed_counter_skips_reboot() {
        reset_ota_timeout_state();
        crate::ota::set_timeout_for_test(0);

        FakeMmio::set_reg32(0x648, FLD_IRQ::SYSTEM_TIMER.bits());
        mock_write_reg_irq_src(Any).returns(());
        mock_handle_ble_system_timer_irq()
            .returns(BleConnectionEvent::OtaFinishCheckAndErrorReboot);

        // The finish handler refreshes the counter to the OTA default (30s).
        mock_rf_link_slave_ota_finish_handle().returns_with(|| {
            crate::ota::set_timeout_for_test(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);
        });
        mock_rf_link_slave_ota_finish_led_and_reboot(Any).returns(());
        mock_check_irq().returns(());

        handle_system_interrupts();

        mock_rf_link_slave_ota_finish_handle().assert_called(1);
        mock_rf_link_slave_ota_finish_led_and_reboot(Any).assert_called(0);
        assert_eq!(
            crate::ota::timeout_seconds_for_test(),
            RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS,
            "the finish handler must have refreshed the counter"
        );

        crate::ota::set_timeout_for_test(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);
    }

    /// Tests that a `TMR1` interrupt acknowledges the timer and steps the light
    /// transition engine exactly once.
    ///
    /// Acceptance: `TMR1` drives smooth dimming, so a missing acknowledge would
    /// re-enter the handler forever and a missing step would freeze transitions.
    #[test]
    #[mry::lock(transition_step, check_irq)]
    fn test_tmr1_irq_clears_status_and_steps_light_transition() {
        FakeMmio::set_reg32(0x648, FLD_IRQ::TMR1_EN.bits());
        FakeMmio::set_reg8(0x623, 0);

        mock_transition_step().returns(());
        mock_check_irq().returns(());

        handle_system_interrupts();

        assert_eq!(
            FakeMmio::reg8(0x623),
            FLD_TMR_STA::TMR1.bits(),
            "the TMR1 status bit must be acknowledged"
        );
        mock_transition_step().assert_called(1);
    }
}
