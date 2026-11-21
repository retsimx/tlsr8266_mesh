//! # Interrupt Composition Root
//!
//! This module is the hardware interrupt entry point for the TLSR8266-based
//! mesh lighting system and the place where the individual interrupt sources
//! are fanned out to their owners.
//!
//! ## Why this lives outside the SDK
//!
//! Fanning hardware interrupts out to BLE/RF, the Embassy time driver, the
//! light transition engine and the UART manager is composition-root work. It
//! belongs next to `app()` rather than inside the BLE link-layer SDK, so the
//! SDK keeps only pure BLE/RF helpers (`crate::sdk::ble_app::irq`).
//!
//! ## Key Responsibilities:
//! - Main interrupt entry point and dispatch logic
//! - RF interrupt handling (TX/RX complete) via the BLE/RF helpers
//! - System timer, Embassy timer, light transition and UART interrupt dispatch
//! - OTA timeout countdown driven by `TMR0` elapsed time

use crate::app;
use crate::embassy::sync::mutex::CriticalSectionMutex;
use crate::embassy::time_driver::clock_time64;
use crate::sdk::ble_app::irq::BleConnectionEvent;
use crate::sdk::light::OtaState;
use crate::sdk::mcu::clock::CLOCK_SYS_CLOCK_1S;
use crate::sdk::mcu::register::*;
use crate::state::*;

/// Timestamp of the last `TMR0` interrupt at which the OTA timeout was decremented.
///
/// `TMR0` is reprogrammed dynamically for embassy timers, so it no longer ticks at a
/// fixed 1 Hz. Stamping the tick here lets `OTA_UPDATE_TIMEOUT_SECONDS` keep counting
/// real seconds. Module-scoped (rather than function-local) so unit tests can reset it.
static LAST_OTA_TIMEOUT_TICK: CriticalSectionMutex<u64> = CriticalSectionMutex::new(0);

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
    let irq_source = read_reg_irq_src();

    // Handle system timer interrupt (BLE state machine)
    if irq_source & FLD_IRQ::SYSTEM_TIMER.bits() != 0 {
        match crate::sdk::ble_app::irq::handle_ble_system_timer_irq() {
            BleConnectionEvent::None => {}
            BleConnectionEvent::OtaFinishCheck => {
                app().ota_manager.rf_link_slave_ota_finish_handle();
            }
            BleConnectionEvent::OtaErrorReboot => {
                app()
                    .ota_manager
                    .rf_link_slave_ota_finish_led_and_reboot(OtaState::Error);
            }
            BleConnectionEvent::OtaFinishCheckAndErrorReboot => {
                // rf_link_slave_ota_finish_handle() -> rf_link_slave_data_ota_save() can
                // refresh the counter to 30s, so it MUST run before the zero check.
                app().ota_manager.rf_link_slave_ota_finish_handle();
                if OTA_UPDATE_TIMEOUT_SECONDS.get() == 0 {
                    app()
                        .ota_manager
                        .rf_link_slave_ota_finish_led_and_reboot(OtaState::Error);
                }
            }
        }
    }

    // Handle Timer 0: embassy timer alarms, clock overflow detection and OTA
    // timeout management. `TMR0` fires whenever the earliest embassy deadline or
    // the one-second clock-overflow cap is reached, so the OTA countdown below is
    // gated on elapsed time instead of on the interrupt rate.
    if irq_source & FLD_IRQ::TMR0_EN.bits() != 0 {
        write_reg_tmr_sta(FLD_TMR_STA::TMR0.bits());

        // Deliver expired embassy timers and re-arm TMR0 for the next deadline.
        crate::embassy::time_driver::on_timer0_interrupt();

        // Update 64-bit clock to handle 32-bit overflow
        let now = clock_time64();

        if !OTA_UPDATE_MESH_OPERATIONS_BLOCKED.get() {
            // Restamp while OTA is idle so the first interrupt after OTA begins
            // cannot decrement the timeout immediately.
            *LAST_OTA_TIMEOUT_TICK.lock() = now;
        } else {
            let mut last = LAST_OTA_TIMEOUT_TICK.lock();
            if now.saturating_sub(*last) >= CLOCK_SYS_CLOCK_1S as u64 {
                *last = now;
                drop(last);

                // Handle mesh OTA timeout countdown
                if OTA_UPDATE_TIMEOUT_SECONDS.get() != 0 {
                    OTA_UPDATE_TIMEOUT_SECONDS.dec();
                    if OTA_UPDATE_TIMEOUT_SECONDS.get() == 0 {
                        // OTA timeout reached: finish with current status
                        app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(
                            *RF_SLAVE_OTA_FINISHED_FLAG.lock(),
                        );
                    }
                }
            }
        }
    }

    // Handle Timer 1: Light transition stepping for smooth dimming
    // This timer provides smooth light transitions by stepping through intermediate values
    if irq_source & FLD_IRQ::TMR1_EN.bits() != 0 {
        write_reg_tmr_sta(FLD_TMR_STA::TMR1.bits());

        app().light_manager.transition_step();
    }

    // Handle UART interrupts for external communication
    app().uart_manager.check_irq();
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use mry::Any;

    // Import mock functions from their original modules
    use crate::embassy::time_driver::{
        mock_clock_time64, mock_on_timer0_interrupt, on_timer0_interrupt,
    };
    use crate::sdk::ble_app::irq::dispatcher::{
        handle_ble_system_timer_irq, handle_rf_irq, mock_handle_ble_system_timer_irq,
        mock_handle_rf_irq,
    };
    use crate::sdk::ble_app::irq::packet_handling::{
        handle_rf_packet_reception, handle_rf_transmission_complete,
        mock_handle_rf_packet_reception, mock_handle_rf_transmission_complete,
    };
    use crate::sdk::light::{RfOperationState, RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS};
    use crate::sdk::mcu::register::{
        mock_read_reg_irq_src, mock_read_reg_rf_irq_status, mock_write_reg_irq_src,
        mock_write_reg_tmr_sta,
    };
    use crate::state::{OTA_UPDATE_MESH_OPERATIONS_BLOCKED, OTA_UPDATE_TIMEOUT_SECONDS};
    use crate::{app::App, app_mocker, mock_app_mocker};

    /// Timeout used by the OTA countdown tests; any non-zero value that cannot be
    /// decremented to zero in a single interrupt works.
    const TEST_OTA_TIMEOUT_SECONDS: u16 = 7;

    /// Restores the module-scoped tick and the OTA statics between tests.
    ///
    /// Tests run in one process with `--test-threads=1`, so process-wide resetting
    /// is safe.
    fn reset_ota_timeout_state() {
        *LAST_OTA_TIMEOUT_TICK.lock() = 0;
        OTA_UPDATE_MESH_OPERATIONS_BLOCKED.set(false);
        OTA_UPDATE_TIMEOUT_SECONDS.set(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);
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
    #[mry::lock(
        read_reg_irq_src,
        write_reg_tmr_sta,
        clock_time64,
        on_timer0_interrupt,
        app_mocker
    )]
    fn test_tmr0_irq_flushes_embassy_timers() {
        reset_ota_timeout_state();

        mock_read_reg_irq_src().returns(FLD_IRQ::TMR0_EN.bits());
        mock_write_reg_tmr_sta(Any).returns(());
        mock_clock_time64().returns(1000);
        mock_on_timer0_interrupt().returns(());

        let mut app = App::default();
        app.uart_manager.mock_check_irq().returns(());
        mock_app_mocker().returns(&mut app);

        handle_system_interrupts();

        mock_on_timer0_interrupt().assert_called(1);
        mock_write_reg_tmr_sta(Any).assert_called(1);
    }

    /// Tests that the OTA tick is restamped while OTA is inactive.
    ///
    /// Acceptance: no countdown may happen outside OTA, and restamping on every
    /// interrupt is what prevents an immediate off-by-one decrement on the first
    /// interrupt after OTA begins.
    #[test]
    #[mry::lock(
        read_reg_irq_src,
        write_reg_tmr_sta,
        clock_time64,
        on_timer0_interrupt,
        app_mocker
    )]
    fn test_tmr0_irq_restamps_ota_tick_when_ota_inactive() {
        reset_ota_timeout_state();
        OTA_UPDATE_TIMEOUT_SECONDS.set(TEST_OTA_TIMEOUT_SECONDS);

        mock_read_reg_irq_src().returns(FLD_IRQ::TMR0_EN.bits());
        mock_write_reg_tmr_sta(Any).returns(());
        mock_clock_time64().returns(50_000);
        mock_on_timer0_interrupt().returns(());

        let mut app = App::default();
        app.uart_manager.mock_check_irq().returns(());
        mock_app_mocker().returns(&mut app);

        handle_system_interrupts();

        assert_eq!(
            *LAST_OTA_TIMEOUT_TICK.lock(),
            50_000,
            "OTA tick should track the latest interrupt while OTA is inactive"
        );
        assert_eq!(
            OTA_UPDATE_TIMEOUT_SECONDS.get(),
            TEST_OTA_TIMEOUT_SECONDS,
            "OTA inactive must not decrement the timeout"
        );
    }

    /// Tests that the OTA countdown stays idle until a full second has elapsed.
    ///
    /// Acceptance: `TMR0` now fires at embassy alarm deadlines, so the countdown
    /// must be gated on elapsed clock ticks rather than on interrupt frequency.
    #[test]
    #[mry::lock(
        read_reg_irq_src,
        write_reg_tmr_sta,
        clock_time64,
        on_timer0_interrupt,
        app_mocker
    )]
    fn test_tmr0_irq_ota_elapsed_under_one_second_does_not_decrement() {
        reset_ota_timeout_state();
        OTA_UPDATE_MESH_OPERATIONS_BLOCKED.set(true);
        OTA_UPDATE_TIMEOUT_SECONDS.set(TEST_OTA_TIMEOUT_SECONDS);
        *LAST_OTA_TIMEOUT_TICK.lock() = 1_000;

        mock_read_reg_irq_src().returns(FLD_IRQ::TMR0_EN.bits());
        mock_write_reg_tmr_sta(Any).returns(());
        mock_clock_time64().returns(1_000 + CLOCK_SYS_CLOCK_1S as u64 - 1);
        mock_on_timer0_interrupt().returns(());

        let mut app = App::default();
        app.uart_manager.mock_check_irq().returns(());
        mock_app_mocker().returns(&mut app);

        handle_system_interrupts();

        assert_eq!(
            *LAST_OTA_TIMEOUT_TICK.lock(),
            1_000,
            "OTA tick should not move before one second of clock ticks elapses"
        );
        assert_eq!(
            OTA_UPDATE_TIMEOUT_SECONDS.get(),
            TEST_OTA_TIMEOUT_SECONDS,
            "OTA timeout must not decrement before one second elapses"
        );
    }

    /// Tests that one second of elapsed clock ticks decrements the OTA timeout once.
    ///
    /// Acceptance: during OTA the timeout still counts real seconds, decrements by
    /// exactly one per second, and restamps the tick that gates the next decrement.
    #[test]
    #[mry::lock(
        read_reg_irq_src,
        write_reg_tmr_sta,
        clock_time64,
        on_timer0_interrupt,
        app_mocker
    )]
    fn test_tmr0_irq_ota_elapsed_one_second_decrements_once() {
        reset_ota_timeout_state();
        OTA_UPDATE_MESH_OPERATIONS_BLOCKED.set(true);
        OTA_UPDATE_TIMEOUT_SECONDS.set(TEST_OTA_TIMEOUT_SECONDS);
        *LAST_OTA_TIMEOUT_TICK.lock() = 1_000;

        let now = 1_000 + CLOCK_SYS_CLOCK_1S as u64;
        mock_read_reg_irq_src().returns(FLD_IRQ::TMR0_EN.bits());
        mock_write_reg_tmr_sta(Any).returns(());
        mock_clock_time64().returns(now);
        mock_on_timer0_interrupt().returns(());

        let mut app = App::default();
        app.uart_manager.mock_check_irq().returns(());
        mock_app_mocker().returns(&mut app);

        handle_system_interrupts();

        assert_eq!(
            *LAST_OTA_TIMEOUT_TICK.lock(),
            now,
            "OTA tick should be restamped when the timeout decrements"
        );
        assert_eq!(
            OTA_UPDATE_TIMEOUT_SECONDS.get(),
            TEST_OTA_TIMEOUT_SECONDS - 1,
            "exactly one second should be consumed by one interrupt"
        );
    }

    // ================================================================================
    // Tests for handle_system_interrupts function - System timer & Timer 1
    // ================================================================================

    /// Tests that a `SYSTEM_TIMER` interrupt dispatches to the BLE state helper,
    /// which clears the source exactly once.
    ///
    /// The helper is mocked with `calls_real_impl`, so the real helper runs while
    /// the call itself is tracked: this pins the wiring `SYSTEM_TIMER` → helper →
    /// register clear, which a dropped call at the dispatch site would break. The
    /// RF operation state is pinned to `Idle` so no BLE state handler runs.
    #[test]
    #[mry::lock(
        read_reg_irq_src,
        write_reg_irq_src,
        handle_ble_system_timer_irq,
        app_mocker
    )]
    fn test_system_timer_irq_clears_source_and_dispatches_ble_helper() {
        *CURRENT_RF_STATE.lock() = RfOperationState::Idle;

        mock_read_reg_irq_src().returns(FLD_IRQ::SYSTEM_TIMER.bits());
        mock_write_reg_irq_src(Any).returns(());
        mock_handle_ble_system_timer_irq().calls_real_impl();

        let mut app = App::default();
        app.uart_manager.mock_check_irq().returns(());
        mock_app_mocker().returns(&mut app);

        handle_system_interrupts();

        mock_write_reg_irq_src(FLD_IRQ::SYSTEM_TIMER.bits()).assert_called(1);
        mock_handle_ble_system_timer_irq().assert_called(1);
    }

    /// `OtaFinishCheck` only polls the OTA finish handler; nothing reboots.
    #[test]
    #[mry::lock(
        read_reg_irq_src,
        write_reg_irq_src,
        handle_ble_system_timer_irq,
        app_mocker
    )]
    fn test_system_timer_irq_ota_finish_check_only_polls() {
        mock_read_reg_irq_src().returns(FLD_IRQ::SYSTEM_TIMER.bits());
        mock_write_reg_irq_src(Any).returns(());
        mock_handle_ble_system_timer_irq().returns(BleConnectionEvent::OtaFinishCheck);

        let mut app = App::default();
        app.ota_manager
            .mock_rf_link_slave_ota_finish_handle()
            .returns(());
        app.ota_manager
            .mock_rf_link_slave_ota_finish_led_and_reboot(Any)
            .returns(());
        app.uart_manager.mock_check_irq().returns(());
        mock_app_mocker().returns(&mut app);

        handle_system_interrupts();

        app.ota_manager
            .mock_rf_link_slave_ota_finish_handle()
            .assert_called(1);
        app.ota_manager
            .mock_rf_link_slave_ota_finish_led_and_reboot(Any)
            .assert_called(0);
    }

    /// `OtaErrorReboot` reboots with `OtaState::Error` without polling the finish
    /// handler first.
    #[test]
    #[mry::lock(
        read_reg_irq_src,
        write_reg_irq_src,
        handle_ble_system_timer_irq,
        app_mocker
    )]
    fn test_system_timer_irq_ota_error_reboots_with_error_state() {
        mock_read_reg_irq_src().returns(FLD_IRQ::SYSTEM_TIMER.bits());
        mock_write_reg_irq_src(Any).returns(());
        mock_handle_ble_system_timer_irq().returns(BleConnectionEvent::OtaErrorReboot);

        let mut app = App::default();
        app.ota_manager
            .mock_rf_link_slave_ota_finish_handle()
            .returns(());
        app.ota_manager
            .mock_rf_link_slave_ota_finish_led_and_reboot(Any)
            .returns(());
        app.uart_manager.mock_check_irq().returns(());
        mock_app_mocker().returns(&mut app);

        handle_system_interrupts();

        app.ota_manager
            .mock_rf_link_slave_ota_finish_led_and_reboot(OtaState::Error)
            .assert_called(1);
        app.ota_manager
            .mock_rf_link_slave_ota_finish_handle()
            .assert_called(0);
    }

    /// `OtaFinishCheckAndErrorReboot` must reboot when the finish handler leaves the
    /// timeout counter at zero.
    #[test]
    #[mry::lock(
        read_reg_irq_src,
        write_reg_irq_src,
        handle_ble_system_timer_irq,
        app_mocker
    )]
    fn test_system_timer_irq_finish_check_and_zero_counter_reboots() {
        reset_ota_timeout_state();
        OTA_UPDATE_TIMEOUT_SECONDS.set(0);

        mock_read_reg_irq_src().returns(FLD_IRQ::SYSTEM_TIMER.bits());
        mock_write_reg_irq_src(Any).returns(());
        mock_handle_ble_system_timer_irq()
            .returns(BleConnectionEvent::OtaFinishCheckAndErrorReboot);

        let mut app = App::default();
        // The finish handler does not refresh the counter in this scenario.
        app.ota_manager
            .mock_rf_link_slave_ota_finish_handle()
            .returns(());
        app.ota_manager
            .mock_rf_link_slave_ota_finish_led_and_reboot(Any)
            .returns(());
        app.uart_manager.mock_check_irq().returns(());
        mock_app_mocker().returns(&mut app);

        handle_system_interrupts();

        app.ota_manager
            .mock_rf_link_slave_ota_finish_handle()
            .assert_called(1);
        app.ota_manager
            .mock_rf_link_slave_ota_finish_led_and_reboot(OtaState::Error)
            .assert_called(1);

        OTA_UPDATE_TIMEOUT_SECONDS.set(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);
    }

    /// `OtaFinishCheckAndErrorReboot` must NOT reboot when the finish handler
    /// refreshes the timeout counter (e.g. `rf_link_slave_data_ota_save()` sets it
    /// back to 30s), which is why the handler runs before the zero check.
    #[test]
    #[mry::lock(
        read_reg_irq_src,
        write_reg_irq_src,
        handle_ble_system_timer_irq,
        app_mocker
    )]
    fn test_system_timer_irq_finish_check_refreshed_counter_skips_reboot() {
        reset_ota_timeout_state();
        OTA_UPDATE_TIMEOUT_SECONDS.set(0);

        mock_read_reg_irq_src().returns(FLD_IRQ::SYSTEM_TIMER.bits());
        mock_write_reg_irq_src(Any).returns(());
        mock_handle_ble_system_timer_irq()
            .returns(BleConnectionEvent::OtaFinishCheckAndErrorReboot);

        let mut app = App::default();
        // The finish handler refreshes the counter to the OTA default (30s).
        app.ota_manager
            .mock_rf_link_slave_ota_finish_handle()
            .returns_with(|| {
                OTA_UPDATE_TIMEOUT_SECONDS.set(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);
            });
        app.ota_manager
            .mock_rf_link_slave_ota_finish_led_and_reboot(Any)
            .returns(());
        app.uart_manager.mock_check_irq().returns(());
        mock_app_mocker().returns(&mut app);

        handle_system_interrupts();

        app.ota_manager
            .mock_rf_link_slave_ota_finish_handle()
            .assert_called(1);
        app.ota_manager
            .mock_rf_link_slave_ota_finish_led_and_reboot(Any)
            .assert_called(0);
        assert_eq!(
            OTA_UPDATE_TIMEOUT_SECONDS.get(),
            RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS,
            "the finish handler must have refreshed the counter"
        );

        OTA_UPDATE_TIMEOUT_SECONDS.set(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);
    }

    /// Tests that a `TMR1` interrupt acknowledges the timer and steps the light
    /// transition engine exactly once.
    ///
    /// Acceptance: `TMR1` drives smooth dimming, so a missing acknowledge would
    /// re-enter the handler forever and a missing step would freeze transitions.
    #[test]
    #[mry::lock(read_reg_irq_src, write_reg_tmr_sta, app_mocker)]
    fn test_tmr1_irq_clears_status_and_steps_light_transition() {
        mock_read_reg_irq_src().returns(FLD_IRQ::TMR1_EN.bits());
        mock_write_reg_tmr_sta(Any).returns(());

        let mut app = App::default();
        app.light_manager.mock_transition_step().returns(());
        app.uart_manager.mock_check_irq().returns(());
        mock_app_mocker().returns(&mut app);

        handle_system_interrupts();

        mock_write_reg_tmr_sta(FLD_TMR_STA::TMR1.bits()).assert_called(1);
        app.light_manager.mock_transition_step().assert_called(1);
    }
}
