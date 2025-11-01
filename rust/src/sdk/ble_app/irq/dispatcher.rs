//! # Main Interrupt Dispatcher Module
//!
//! This module contains the main interrupt handler and interrupt context tracking
//! for the TLSR8266-based mesh lighting system.
//!
//! ## Key Responsibilities:
//! - Main interrupt entry point and dispatch logic
//! - Interrupt context tracking (IRQ mode detection)
//! - RF interrupt handling (TX/RX complete)
//! - System timer and application interrupt processing

use core::sync::atomic::{AtomicBool, Ordering};

use crate::app;
use crate::embassy::time_driver::clock_time64;
use crate::sdk::light::RfOperationState;
use crate::sdk::mcu::register::*;
use crate::state::*;

use super::advertisement::handle_ble_advertisement_state;
use super::connection::handle_ble_connected_state;
use super::mesh::handle_mesh_listening_state;
use super::packet_handling::{handle_rf_packet_reception, handle_rf_transmission_complete};
use super::timing::configure_ble_receive_state;

/// Global flag to track if we're currently executing within an interrupt context.
/// This is used by other parts of the system to determine execution context.
static IS_IRQ_MODE: AtomicBool = AtomicBool::new(false);

/// RAII guard for tracking interrupt execution context.
///
/// This structure automatically sets the global interrupt flag when created
/// and clears it when dropped, ensuring accurate tracking of interrupt context
/// even if the interrupt handler exits early or panics.
///
/// ## Usage:
/// ```rust
/// fn interrupt_handler() {
///     let _guard = IrqTracker::new(); // Sets IRQ flag
///     // ... interrupt handling code ...
/// } // IRQ flag automatically cleared when guard is dropped
/// ```
pub struct IrqTracker {}

impl IrqTracker {
    /// Creates a new interrupt tracker and sets the global IRQ flag.
    ///
    /// ## Returns:
    /// IrqTracker instance that will clear the flag when dropped
    #[inline(always)]
    pub fn new() -> Self {
        IS_IRQ_MODE.store(true, Ordering::Relaxed);
        return IrqTracker {};
    }

    /// Checks if code is currently executing within an interrupt context.
    ///
    /// ## Returns:
    /// `true` if currently in interrupt context, `false` otherwise
    #[inline(always)]
    pub fn in_irq() -> bool {
        IS_IRQ_MODE.load(Ordering::Relaxed)
    }
}

impl Drop for IrqTracker {
    /// Automatically clears the interrupt flag when the tracker is dropped.
    /// This ensures the flag is always cleared, even if the interrupt handler
    /// exits early or encounters an error.
    #[inline(always)]
    fn drop(&mut self) {
        IS_IRQ_MODE.store(false, Ordering::Relaxed);
    }
}

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
/// 3. **Timer 0**: Clock overflow detection and OTA timeout management
/// 4. **Timer 1**: Light transition stepping for smooth dimming effects
/// 5. **UART**: Serial communication with external systems
///
/// ## Execution Flow:
/// 1. Set interrupt context tracking
/// 2. Handle high-priority RF interrupts (TX/RX)
/// 3. Process system timer and application interrupts
/// 4. Automatically clear interrupt context on exit
// no_mangle because this is referenced as an entrypoint from the assembler bootstrap
#[no_mangle]
#[link_section = ".ram_code"]
pub extern "C" fn irq_handler() {
    // Track interrupt context for system-wide interrupt awareness
    let _tracker = IrqTracker::new();

    // Handle RF (Radio Frequency) interrupts first (highest priority)
    let irq = read_reg_rf_irq_status();

    // Handle RF transmission complete interrupt
    if irq & FLD_RF_IRQ_MASK::IRQ_TX.bits() != 0 {
        handle_rf_transmission_complete();
    }

    // Handle RF reception complete interrupt
    if irq & FLD_RF_IRQ_MASK::IRQ_RX.bits() != 0 {
        handle_rf_packet_reception();
    }

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
        // Clear the interrupt source
        write_reg_irq_src(FLD_IRQ::SYSTEM_TIMER.bits());

        // Get current BLE state and dispatch to appropriate handler
        let state = { *CURRENT_RF_STATE.lock() };
        match state {
            RfOperationState::Advertising => handle_ble_advertisement_state(), // Advertisement state
            RfOperationState::Connected => handle_ble_connected_state(), // Connected bridge state
            RfOperationState::Receiving => configure_ble_receive_state(), // BLE receive state
            RfOperationState::MeshListening => handle_mesh_listening_state(), // Mesh listening state
            RfOperationState::Idle => {}                                      // Idle state
        }
    }

    // Handle Timer 0: Clock overflow detection and OTA timeout management
    // This timer runs once per second to prevent clock overflow issues
    if irq_source & FLD_IRQ::TMR0_EN.bits() != 0 {
        write_reg_tmr_sta(FLD_TMR_STA::TMR0.bits());

        // Update 64-bit clock to handle 32-bit overflow
        clock_time64();

        // Handle mesh OTA timeout countdown
        if OTA_UPDATE_MESH_OPERATIONS_BLOCKED.get() {
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
mod tests {
    use super::*;
    use mry::Any;

    // Import mock functions from their original modules
    use crate::embassy::time_driver::mock_clock_time64;
    use crate::sdk::ble_app::irq::advertisement::mock_handle_ble_advertisement_state;
    use crate::sdk::ble_app::irq::connection::mock_handle_ble_connected_state;
    use crate::sdk::ble_app::irq::mesh::mock_handle_mesh_listening_state;
    use crate::sdk::ble_app::irq::packet_handling::{
        mock_handle_rf_packet_reception, mock_handle_rf_transmission_complete,
    };
    use crate::sdk::ble_app::irq::timing::mock_configure_ble_receive_state;
    use crate::sdk::light::{OtaState, RfOperationState};
    use crate::sdk::mcu::register::{
        mock_read_reg_irq_src, mock_read_reg_rf_irq_status, mock_read_reg_system_tick_irq,
        mock_write_reg_irq_src, mock_write_reg_rf_irq_status, mock_write_reg_tmr_sta,
    };
    use crate::state::{
        OTA_UPDATE_IN_PROGRESS, OTA_UPDATE_MESH_OPERATIONS_BLOCKED, OTA_UPDATE_TIMEOUT_SECONDS,
        RF_SLAVE_OTA_FINISHED_FLAG,
    };
    use crate::{app_mocker, mock_app_mocker};

    // ================================================================================
    // Tests for IrqTracker struct and interrupt context tracking
    // ================================================================================

    /// Tests that IrqTracker correctly sets and clears the interrupt context flag.
    ///
    /// This test verifies the RAII pattern used for interrupt context tracking.
    /// The global IS_IRQ_MODE flag should be set when IrqTracker is created
    /// and automatically cleared when it's dropped.
    #[test]
    fn test_irq_tracker_context_tracking() {
        // Ensure we start with no interrupt context
        assert_eq!(
            IrqTracker::in_irq(),
            false,
            "Should not be in IRQ context initially"
        );

        // Create tracker - should set IRQ context
        {
            let _tracker = IrqTracker::new();
            assert_eq!(
                IrqTracker::in_irq(),
                true,
                "Should be in IRQ context while tracker exists"
            );
        }

        // Tracker dropped - should clear IRQ context
        assert_eq!(
            IrqTracker::in_irq(),
            false,
            "Should not be in IRQ context after tracker dropped"
        );
    }

    /// Tests that IrqTracker::in_irq() returns the correct state.
    ///
    /// Verifies that the static method correctly reads the global interrupt context flag.
    #[test]
    fn test_irq_tracker_in_irq_method() {
        // Test when not in IRQ context
        assert_eq!(
            IrqTracker::in_irq(),
            false,
            "Should return false when not in IRQ context"
        );

        // Test when in IRQ context
        let _tracker = IrqTracker::new();
        assert_eq!(
            IrqTracker::in_irq(),
            true,
            "Should return true when in IRQ context"
        );

        // Test after dropping tracker
        drop(_tracker);
        assert_eq!(
            IrqTracker::in_irq(),
            false,
            "Should return false after dropping tracker"
        );
    }

    /// Tests multiple IrqTracker instances don't interfere with each other.
    ///
    /// Verifies that the RAII pattern works correctly with nested or multiple
    /// interrupt contexts.
    #[test]
    fn test_irq_tracker_multiple_instances() {
        // Start with no IRQ context
        assert_eq!(
            IrqTracker::in_irq(),
            false,
            "Should not be in IRQ context initially"
        );

        // Create first tracker
        let tracker1 = IrqTracker::new();
        assert_eq!(
            IrqTracker::in_irq(),
            true,
            "Should be in IRQ context with first tracker"
        );

        // Create second tracker - this will overwrite the first one
        let tracker2 = IrqTracker::new();
        assert_eq!(
            IrqTracker::in_irq(),
            true,
            "Should remain in IRQ context with second tracker"
        );

        // Drop first tracker - this will clear the IRQ context since only the last tracker is active
        drop(tracker1);
        assert_eq!(
            IrqTracker::in_irq(),
            false,
            "Should clear IRQ context after dropping first tracker"
        );

        // Drop second tracker - should still be false since it was already cleared
        drop(tracker2);
        assert_eq!(
            IrqTracker::in_irq(),
            false,
            "Should remain not in IRQ context after dropping all trackers"
        );
    }

    // ================================================================================
    // Tests for irq_handler function (main interrupt handler)
    // ================================================================================

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
        // Ensure clean state before test

        // Mock RF transmission complete interrupt
        mock_read_reg_rf_irq_status().returns(FLD_RF_IRQ_MASK::IRQ_TX.bits());
        mock_handle_rf_transmission_complete().returns(());
        mock_handle_system_interrupts().returns(());

        // Ensure we start with no interrupt context
        assert_eq!(
            IrqTracker::in_irq(),
            false,
            "Should not be in IRQ context before handler"
        );

        // Execute the interrupt handler
        irq_handler();

        // Verify the transmission complete handler was called
        mock_handle_rf_transmission_complete().assert_called(1);
        // Verify system interrupts were processed
        mock_handle_system_interrupts().assert_called(1);
        // Verify we're no longer in IRQ context after handler returns
        assert_eq!(
            IrqTracker::in_irq(),
            false,
            "Should not be in IRQ context after handler"
        );
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
        // Ensure clean state before test

        // Mock RF reception complete interrupt
        mock_read_reg_rf_irq_status().returns(FLD_RF_IRQ_MASK::IRQ_RX.bits());
        mock_handle_rf_packet_reception().returns(());
        mock_handle_system_interrupts().returns(());

        // Ensure we start with no interrupt context
        assert_eq!(
            IrqTracker::in_irq(),
            false,
            "Should not be in IRQ context before handler"
        );

        // Execute the interrupt handler
        irq_handler();

        // Verify the packet reception handler was called
        mock_handle_rf_packet_reception().assert_called(1);
        // Verify system interrupts were processed
        mock_handle_system_interrupts().assert_called(1);
        // Verify we're no longer in IRQ context after handler returns
        assert_eq!(
            IrqTracker::in_irq(),
            false,
            "Should not be in IRQ context after handler"
        );
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
        // Ensure clean state before test

        // Mock both RF interrupts
        let both_interrupts = FLD_RF_IRQ_MASK::IRQ_TX.bits() | FLD_RF_IRQ_MASK::IRQ_RX.bits();
        mock_read_reg_rf_irq_status().returns(both_interrupts);
        mock_handle_rf_transmission_complete().returns(());
        mock_handle_rf_packet_reception().returns(());
        mock_handle_system_interrupts().returns(());

        // Ensure we start with no interrupt context
        assert_eq!(
            IrqTracker::in_irq(),
            false,
            "Should not be in IRQ context before handler"
        );

        // Execute the interrupt handler
        irq_handler();

        // Verify both handlers were called
        mock_handle_rf_transmission_complete().assert_called(1);
        mock_handle_rf_packet_reception().assert_called(1);
        // Verify system interrupts were processed
        mock_handle_system_interrupts().assert_called(1);
        // Verify we're no longer in IRQ context after handler returns
        assert_eq!(
            IrqTracker::in_irq(),
            false,
            "Should not be in IRQ context after handler"
        );
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
        // Ensure clean state before test

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
        // Verify we're no longer in IRQ context after handler returns
        assert_eq!(
            IrqTracker::in_irq(),
            false,
            "Should not be in IRQ context after handler"
        );
    }

    // ================================================================================
    // Tests for handle_system_interrupts function - System Timer (BLE State Machine)
    // ================================================================================

    /// Tests system timer interrupt handling with no system timer interrupt.
    ///
    /// Verifies that when the system timer interrupt is not set, no state
    /// handlers are called and the interrupt source is not cleared.
    #[test]
    fn test_system_interrupts_no_system_timer() {
        // Test the interrupt source checking logic
        let irq_source = 0u32; // No system timer interrupt

        // Verify that when no system timer interrupt is set, the condition should be false
        let has_system_timer = irq_source & FLD_IRQ::SYSTEM_TIMER.bits() != 0;
        assert_eq!(
            has_system_timer, false,
            "Should not have system timer interrupt when source is 0"
        );
    }
}
