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

use crate::embassy::time_driver::clock_time64;
use crate::sdk::light::RfOperationState;
use crate::sdk::mcu::register::{*};
use crate::state::{*};
use crate::{app};

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
        return IrqTracker {}
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
fn handle_system_interrupts() {
    // Get system interrupt sources
    let irq_source = read_reg_irq_src();
    
    // Handle system timer interrupt (BLE state machine)
    if irq_source & FLD_IRQ::SYSTEM_TIMER.bits() != 0 {
        // Clear the interrupt source
        write_reg_irq_src(FLD_IRQ::SYSTEM_TIMER.bits());
        
        // Get current BLE state and dispatch to appropriate handler
        let state = {
            *CURRENT_RF_STATE.lock()
        };
        match state {
            RfOperationState::Advertising => handle_ble_advertisement_state(),        // Advertisement state
            RfOperationState::Connected => handle_ble_connected_state(),   // Connected bridge state
            RfOperationState::Receiving => configure_ble_receive_state(),      // BLE receive state
            RfOperationState::MeshListening => handle_mesh_listening_state(),   // Mesh listening state
            RfOperationState::Idle => {}                   // Idle state
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
                    app().ota_manager.rf_link_slave_ota_finish_led_and_reboot(*RF_SLAVE_OTA_FINISHED_FLAG.lock());
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
