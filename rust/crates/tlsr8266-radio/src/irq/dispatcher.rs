//! # BLE/RF Interrupt Helpers Module
//!
//! This module contains the pure BLE/RF interrupt helpers for the
//! TLSR8266-based mesh lighting system. The hardware interrupt entry point
//! (the crate root's `irq_handler`) calls these helpers; composing them with
//! the Embassy time driver, light transitions and the UART manager is
//! composition-root work that lives in the crate root.
//!
//! ## Key Responsibilities:
//! - RF interrupt handling (TX/RX complete)
//! - BLE system timer state dispatch

use crate::coex::RadioCoexistence;
use crate::rf::drv_8266::CURRENT_RF_STATE;
use crate::types::RfOperationState;
use tlsr8266_pac::*;

use super::advertisement::handle_ble_advertisement_state;
use super::connection::{handle_ble_connected_state, BleConnectionEvent};
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
/// the crate root's `irq_handler` so it stays inside the `.ram_code` section
/// (no XIP flash jump during radio operations) and adds no extra interrupt
/// stack frame.
#[inline(always)]
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
/// frame. It inlines into the crate root's `handle_system_interrupts`, which is
/// `#[inline(never)]` and therefore stays in `.text` (only `irq_handler` itself
/// is `.ram_code`) — the same placement the dispatch had before the move.
///
/// Returns the [`BleConnectionEvent`] reported by the connected-state handler so
/// the interrupt composition root can own the `ota_manager` calls. Every other
/// state reports [`BleConnectionEvent::None`].
#[inline(always)]
pub fn handle_ble_system_timer_irq<C: RadioCoexistence>() -> BleConnectionEvent {
    // Clear the interrupt source
    write_reg_irq_src(FLD_IRQ::SYSTEM_TIMER.bits());

    // Get current BLE state and dispatch to appropriate handler
    let state = { *CURRENT_RF_STATE.lock() };
    match state {
        RfOperationState::Advertising => {
            handle_ble_advertisement_state(); // Advertisement state
            BleConnectionEvent::None
        }
        RfOperationState::Connected => handle_ble_connected_state::<C>(), // Connected bridge state
        RfOperationState::Receiving => {
            configure_ble_receive_state(); // BLE receive state
            BleConnectionEvent::None
        }
        RfOperationState::MeshListening => {
            handle_mesh_listening_state(); // Mesh listening state
            BleConnectionEvent::None
        }
        RfOperationState::Idle => BleConnectionEvent::None, // Idle state
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;

    use crate::ble::connection::{
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US, BLE_PERIPHERAL_LINK_STATE, SLAVE_CONNECTED_TICK,
    };
    use crate::mesh_ll::node_status::MESH_DEVICE_ONLINE_STATUS;
    use crate::rf::drv_8266::{BLE_ADVERTISING_ENABLED, LIGHT_RX_BUFFER_WRITE_POINTER};
    use crate::types::BlePeripheralLinkState;
    use tlsr8266_hal::common::sync::SimplifyLS;
    use tlsr8266_hal::mcu::clock::CLOCK_SYS_CLOCK_1US;
    use tlsr8266_pac::fake_mmio::FakeMmio;

    /// Datasheet offsets of the interrupt registers this module touches.
    const REG_RF_IRQ_STATUS: u32 = 0xf20;
    const REG_RF_RX_STATUS: u32 = 0x443;
    const REG_IRQ_SRC: u32 = 0x648;

    /// Every value written to `reg_rf_irq_status`, oldest first. The RF
    /// handlers acknowledge their interrupt source with a write-1-to-clear, so
    /// the logged values are the dispatch evidence.
    fn rf_irq_ack_writes() -> Vec<u32> {
        FakeMmio::writes()
            .iter()
            .filter(|w| w.addr == (REG_RF_IRQ_STATUS | tlsr8266_pac::REG_BASE_ADDR))
            .map(|w| w.value)
            .collect()
    }

    /// Restores the process-wide RF operation state between tests, and clears
    /// the PAC/HAL host fakes so a test never inherits another test's register
    /// bank (the suite runs `--test-threads=1`, so these are singletons).
    fn reset_rf_state() {
        FakeMmio::reset();
        tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog::reset();
        tlsr8266_hal::mcu::clock::fake_clock::reset();
        tlsr8266_hal::mcu::crypto::fake_crypto::reset();
        crate::queues::reset_for_test();
        crate::callbacks::test_ota::reset_for_test();
        BLE_ADVERTISING_ENABLED.set(false);
        MESH_DEVICE_ONLINE_STATUS.set(false);
        LIGHT_RX_BUFFER_WRITE_POINTER.set(0);
        BLE_PERIPHERAL_LINK_STATE.set(BlePeripheralLinkState::Disconnected);
        // Keep the supervision timeout from interrupting the connected-state
        // handler in tests that drive that arm.
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(10_000_000);
        SLAVE_CONNECTED_TICK.set(0);
        *CURRENT_RF_STATE.lock() = RfOperationState::Idle;
    }

    // ================================================================================
    // Tests for handle_rf_irq
    // ================================================================================

    /// Tests that an RF transmission complete interrupt dispatches to the
    /// transmission complete handler.
    #[test]
    fn test_handle_rf_irq_transmission_complete() {
        reset_rf_state();
        FakeMmio::set_reg16(REG_RF_IRQ_STATUS, FLD_RF_IRQ_MASK::IRQ_TX.bits());

        handle_rf_irq();

        assert_eq!(
            rf_irq_ack_writes(),
            vec![2],
            "the TX handler must acknowledge bit 2 (RX stays untouched)"
        );
    }

    /// Tests that an RF reception complete interrupt dispatches to the packet
    /// reception handler.
    #[test]
    fn test_handle_rf_irq_reception_complete() {
        reset_rf_state();
        FakeMmio::set_reg16(REG_RF_IRQ_STATUS, FLD_RF_IRQ_MASK::IRQ_RX.bits());
        // RX error status: the reception handler acknowledges and returns
        // before it needs a real DMA frame in the buffer.
        FakeMmio::set_reg8(REG_RF_RX_STATUS, 0x0b);

        handle_rf_irq();

        assert_eq!(
            rf_irq_ack_writes(),
            vec![1],
            "the RX handler must acknowledge bit 1 (TX stays untouched)"
        );
    }

    /// Tests that simultaneous TX and RX interrupts dispatch to both handlers.
    #[test]
    fn test_handle_rf_irq_both_interrupts() {
        reset_rf_state();
        let both_interrupts = FLD_RF_IRQ_MASK::IRQ_TX.bits() | FLD_RF_IRQ_MASK::IRQ_RX.bits();
        FakeMmio::set_reg16(REG_RF_IRQ_STATUS, both_interrupts);
        FakeMmio::set_reg8(REG_RF_RX_STATUS, 0x0b);

        handle_rf_irq();

        assert_eq!(
            rf_irq_ack_writes(),
            vec![2, 1],
            "TX then RX must each acknowledge their own bit"
        );
    }

    /// Tests that no RF interrupt bits means no dispatch.
    #[test]
    fn test_handle_rf_irq_no_interrupts() {
        reset_rf_state();
        FakeMmio::set_reg16(REG_RF_IRQ_STATUS, 0);

        handle_rf_irq();

        assert!(
            rf_irq_ack_writes().is_empty(),
            "no interrupt bit means neither handler may run"
        );
    }

    // ================================================================================
    // Tests for handle_ble_system_timer_irq
    // ================================================================================

    /// Tests that the system timer source is cleared before dispatching, and that
    /// the advertising state handler runs and reports nothing to the composition root.
    #[test]
    fn test_handle_ble_system_timer_irq_clears_source_and_dispatches_advertising() {
        reset_rf_state();
        *CURRENT_RF_STATE.lock() = RfOperationState::Advertising;
        FakeMmio::set_reg32(0x740, 10_000);

        let event = handle_ble_system_timer_irq::<crate::coex::TestRadioCoexistence>();

        assert_eq!(event, BleConnectionEvent::None);
        assert_eq!(
            FakeMmio::reg32(REG_IRQ_SRC),
            FLD_IRQ::SYSTEM_TIMER.bits(),
            "the SYSTEM_TIMER source must be cleared before dispatch"
        );
        // The advertising handler's completion branch: arm the listen timer
        // (500µs, no online status queued) and return to mesh listening.
        assert_eq!(
            read_reg_system_tick_irq(),
            CLOCK_SYS_CLOCK_1US * 500 + 10_000,
            "the advertising handler must arm the next state timer"
        );
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::MeshListening,
            "the advertising handler must hand over to mesh listening"
        );
        assert_eq!(
            rf_irq_ack_writes(),
            vec![1],
            "the advertising handler acknowledges its RX interrupt"
        );
    }

    /// Tests dispatch to the connected state handler and that its event is
    /// propagated unchanged to the caller.
    #[test]
    fn test_handle_ble_system_timer_irq_dispatches_connected() {
        reset_rf_state();
        *CURRENT_RF_STATE.lock() = RfOperationState::Connected;

        // The connected handler reports `OtaErrorReboot` when the supervision
        // timeout expires while an OTA is in progress. Drive the real clock,
        // timeout and OTA state through their seams instead of stubbing the
        // handler.
        FakeMmio::set_reg32(0x740, 100_000);
        BLE_PERIPHERAL_CONNECTION_TIMEOUT_US.set(0);
        SLAVE_CONNECTED_TICK.set(0);
        crate::callbacks::test_ota::set_in_progress_for_test(true);

        let event = handle_ble_system_timer_irq::<crate::coex::TestRadioCoexistence>();

        assert_eq!(
            event,
            BleConnectionEvent::OtaErrorReboot,
            "the Connected arm must propagate the connection event unchanged"
        );
        // The handler body really ran: it configures the link for an active
        // connection before it checks the timeout.
        assert_eq!(
            crate::ble::connection::BLE_PERIPHERAL_LINK_STATE.get(),
            BlePeripheralLinkState::Connected,
            "the connected handler must configure the active link state"
        );
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Advertising,
            "a timed-out connection must fall back to advertising"
        );
    }

    /// Tests dispatch to the receiving and mesh-listening state handlers.
    /// Neither of them reports anything to the composition root.
    #[test]
    fn test_handle_ble_system_timer_irq_dispatches_receiving_and_mesh_listening() {
        reset_rf_state();

        *CURRENT_RF_STATE.lock() = RfOperationState::Receiving;
        let receiving_event = handle_ble_system_timer_irq::<crate::coex::TestRadioCoexistence>();
        assert_eq!(receiving_event, BleConnectionEvent::None);
        // The receive handler reconfigures the radio for the next connection
        // event and hands over to the connected state.
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Connected,
            "the receive handler must hand over to the connected state"
        );

        *CURRENT_RF_STATE.lock() = RfOperationState::MeshListening;
        let mesh_event = handle_ble_system_timer_irq::<crate::coex::TestRadioCoexistence>();
        assert_eq!(mesh_event, BleConnectionEvent::None);
        assert_eq!(
            crate::ble::connection::BLE_PERIPHERAL_LINK_STATE.get(),
            BlePeripheralLinkState::Mesh,
            "the mesh-listening handler must configure the mesh link state"
        );
    }

    /// Tests that the idle state clears the source, runs no state handler and
    /// reports nothing to the composition root.
    #[test]
    fn test_handle_ble_system_timer_irq_idle_runs_no_state_handler() {
        reset_rf_state();

        let event = handle_ble_system_timer_irq::<crate::coex::TestRadioCoexistence>();

        assert_eq!(event, BleConnectionEvent::None);
        assert_eq!(
            FakeMmio::reg32(REG_IRQ_SRC),
            FLD_IRQ::SYSTEM_TIMER.bits(),
            "the SYSTEM_TIMER source must be cleared before dispatch"
        );
        assert_eq!(
            crate::ble::connection::BLE_PERIPHERAL_LINK_STATE.get(),
            BlePeripheralLinkState::Disconnected,
            "no state handler may reconfigure the link"
        );
        assert!(
            rf_irq_ack_writes().is_empty(),
            "no state handler may acknowledge an RF interrupt"
        );
        assert_eq!(
            *CURRENT_RF_STATE.lock(),
            RfOperationState::Idle,
            "the idle arm must not schedule a state transition"
        );
    }
}
