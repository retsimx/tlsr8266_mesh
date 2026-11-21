//! # BLE/RF Interrupt Helpers Module
//!
//! This module contains the pure BLE/RF interrupt helpers for the
//! TLSR8266-based mesh lighting system. The hardware interrupt entry point
//! (`crate::irq::irq_handler`) calls these helpers; composing them with the
//! Embassy time driver, light transitions and the UART manager is
//! composition-root work that lives in `crate::irq`.
//!
//! ## Key Responsibilities:
//! - RF interrupt handling (TX/RX complete)
//! - BLE system timer state dispatch

use crate::sdk::light::RfOperationState;
use crate::sdk::mcu::register::*;
use crate::state::*;

use super::advertisement::handle_ble_advertisement_state;
use super::connection::handle_ble_connected_state;
use super::mesh::handle_mesh_listening_state;
use super::packet_handling::{handle_rf_packet_reception, handle_rf_transmission_complete};
use super::timing::configure_ble_receive_state;

/// Handles RF (Radio Frequency) interrupts.
///
/// Reads the RF interrupt status register and dispatches to the TX-complete and
/// RX-complete handlers. This is the highest-priority part of the hardware
/// interrupt path.
///
/// ## Interrupt Types Handled:
/// 1. **TX complete**: BLE/mesh transmission finished
/// 2. **RX complete**: BLE/mesh packet received and ready for processing
///
/// `#[inline(always)]` is mandatory: the whole RF path must be inlined into
/// `crate::irq::irq_handler` so it stays inside the `.ram_code` section (no XIP
/// flash jump during radio operations) and adds no extra interrupt stack frame.
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn handle_rf_irq() {
    let irq = read_reg_rf_irq_status();

    // Handle RF transmission complete interrupt
    if irq & FLD_RF_IRQ_MASK::IRQ_TX.bits() != 0 {
        handle_rf_transmission_complete();
    }

    // Handle RF reception complete interrupt
    if irq & FLD_RF_IRQ_MASK::IRQ_RX.bits() != 0 {
        handle_rf_packet_reception();
    }
}

/// Handles the BLE system timer interrupt (BLE state machine).
///
/// Clears the `SYSTEM_TIMER` interrupt source, then dispatches to the handler
/// that matches the current RF operation state.
///
/// ## States Handled:
/// - **Advertising**: advertisement and scan response handling
/// - **Connected**: connection bridge handling
/// - **Receiving**: BLE receive reconfiguration
/// - **MeshListening**: mesh packet listening
/// - **Idle**: nothing to do
///
/// `#[inline(always)]` is mandatory so this helper adds no extra interrupt stack
/// frame. It inlines into `crate::irq::handle_system_interrupts`, which is
/// `#[inline(never)]` and therefore stays in `.text` (only `irq_handler` itself
/// is `.ram_code`) — the same placement the dispatch had before the move.
#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn handle_ble_system_timer_irq() {
    // Clear the interrupt source
    write_reg_irq_src(FLD_IRQ::SYSTEM_TIMER.bits());

    // Get current BLE state and dispatch to appropriate handler
    let state = { *CURRENT_RF_STATE.lock() };
    match state {
        RfOperationState::Advertising => handle_ble_advertisement_state(), // Advertisement state
        RfOperationState::Connected => handle_ble_connected_state(),       // Connected bridge state
        RfOperationState::Receiving => configure_ble_receive_state(),      // BLE receive state
        RfOperationState::MeshListening => handle_mesh_listening_state(),  // Mesh listening state
        RfOperationState::Idle => {}                                       // Idle state
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use mry::Any;

    // Import mock functions from their original modules
    use crate::sdk::ble_app::irq::advertisement::mock_handle_ble_advertisement_state;
    use crate::sdk::ble_app::irq::connection::mock_handle_ble_connected_state;
    use crate::sdk::ble_app::irq::mesh::mock_handle_mesh_listening_state;
    use crate::sdk::ble_app::irq::packet_handling::{
        mock_handle_rf_packet_reception, mock_handle_rf_transmission_complete,
    };
    use crate::sdk::ble_app::irq::timing::mock_configure_ble_receive_state;
    use crate::sdk::mcu::register::{mock_read_reg_rf_irq_status, mock_write_reg_irq_src};

    /// Restores the process-wide RF operation state between tests.
    fn reset_rf_state() {
        *CURRENT_RF_STATE.lock() = RfOperationState::Idle;
    }

    // ================================================================================
    // Tests for handle_rf_irq
    // ================================================================================

    /// Tests that an RF transmission complete interrupt dispatches to the
    /// transmission complete handler.
    #[test]
    #[mry::lock(read_reg_rf_irq_status, handle_rf_transmission_complete)]
    fn test_handle_rf_irq_transmission_complete() {
        mock_read_reg_rf_irq_status().returns(FLD_RF_IRQ_MASK::IRQ_TX.bits());
        mock_handle_rf_transmission_complete().returns(());

        handle_rf_irq();

        mock_handle_rf_transmission_complete().assert_called(1);
    }

    /// Tests that an RF reception complete interrupt dispatches to the packet
    /// reception handler.
    #[test]
    #[mry::lock(read_reg_rf_irq_status, handle_rf_packet_reception)]
    fn test_handle_rf_irq_reception_complete() {
        mock_read_reg_rf_irq_status().returns(FLD_RF_IRQ_MASK::IRQ_RX.bits());
        mock_handle_rf_packet_reception().returns(());

        handle_rf_irq();

        mock_handle_rf_packet_reception().assert_called(1);
    }

    /// Tests that simultaneous TX and RX interrupts dispatch to both handlers.
    #[test]
    #[mry::lock(
        read_reg_rf_irq_status,
        handle_rf_transmission_complete,
        handle_rf_packet_reception
    )]
    fn test_handle_rf_irq_both_interrupts() {
        let both_interrupts = FLD_RF_IRQ_MASK::IRQ_TX.bits() | FLD_RF_IRQ_MASK::IRQ_RX.bits();
        mock_read_reg_rf_irq_status().returns(both_interrupts);
        mock_handle_rf_transmission_complete().returns(());
        mock_handle_rf_packet_reception().returns(());

        handle_rf_irq();

        mock_handle_rf_transmission_complete().assert_called(1);
        mock_handle_rf_packet_reception().assert_called(1);
    }

    /// Tests that no RF interrupt bits means no dispatch.
    #[test]
    #[mry::lock(
        read_reg_rf_irq_status,
        handle_rf_transmission_complete,
        handle_rf_packet_reception
    )]
    fn test_handle_rf_irq_no_interrupts() {
        mock_read_reg_rf_irq_status().returns(0);
        mock_handle_rf_transmission_complete().returns(());
        mock_handle_rf_packet_reception().returns(());

        handle_rf_irq();

        mock_handle_rf_transmission_complete().assert_called(0);
        mock_handle_rf_packet_reception().assert_called(0);
    }

    // ================================================================================
    // Tests for handle_ble_system_timer_irq
    // ================================================================================

    /// Tests that the system timer source is cleared before dispatching, and that
    /// the advertising state handler runs.
    #[test]
    #[mry::lock(write_reg_irq_src, handle_ble_advertisement_state)]
    fn test_handle_ble_system_timer_irq_clears_source_and_dispatches_advertising() {
        reset_rf_state();
        *CURRENT_RF_STATE.lock() = RfOperationState::Advertising;

        mock_write_reg_irq_src(Any).returns(());
        mock_handle_ble_advertisement_state().returns(());

        handle_ble_system_timer_irq();

        mock_write_reg_irq_src(Any).assert_called(1);
        mock_handle_ble_advertisement_state().assert_called(1);
    }

    /// Tests dispatch to the connected state handler.
    #[test]
    #[mry::lock(write_reg_irq_src, handle_ble_connected_state)]
    fn test_handle_ble_system_timer_irq_dispatches_connected() {
        reset_rf_state();
        *CURRENT_RF_STATE.lock() = RfOperationState::Connected;

        mock_write_reg_irq_src(Any).returns(());
        mock_handle_ble_connected_state().returns(());

        handle_ble_system_timer_irq();

        mock_handle_ble_connected_state().assert_called(1);
    }

    /// Tests dispatch to the receiving and mesh-listening state handlers.
    #[test]
    #[mry::lock(
        write_reg_irq_src,
        configure_ble_receive_state,
        handle_mesh_listening_state
    )]
    fn test_handle_ble_system_timer_irq_dispatches_receiving_and_mesh_listening() {
        reset_rf_state();
        mock_write_reg_irq_src(Any).returns(());
        mock_configure_ble_receive_state().returns(());
        mock_handle_mesh_listening_state().returns(());

        *CURRENT_RF_STATE.lock() = RfOperationState::Receiving;
        handle_ble_system_timer_irq();
        mock_configure_ble_receive_state().assert_called(1);

        *CURRENT_RF_STATE.lock() = RfOperationState::MeshListening;
        handle_ble_system_timer_irq();
        mock_handle_mesh_listening_state().assert_called(1);
    }

    /// Tests that the idle state clears the source but runs no state handler.
    #[test]
    #[mry::lock(
        write_reg_irq_src,
        handle_ble_advertisement_state,
        handle_ble_connected_state,
        configure_ble_receive_state,
        handle_mesh_listening_state
    )]
    fn test_handle_ble_system_timer_irq_idle_runs_no_state_handler() {
        reset_rf_state();

        mock_write_reg_irq_src(Any).returns(());
        mock_handle_ble_advertisement_state().returns(());
        mock_handle_ble_connected_state().returns(());
        mock_configure_ble_receive_state().returns(());
        mock_handle_mesh_listening_state().returns(());

        handle_ble_system_timer_irq();

        mock_write_reg_irq_src(Any).assert_called(1);
        mock_handle_ble_advertisement_state().assert_called(0);
        mock_handle_ble_connected_state().assert_called(0);
        mock_configure_ble_receive_state().assert_called(0);
        mock_handle_mesh_listening_state().assert_called(0);
    }
}
