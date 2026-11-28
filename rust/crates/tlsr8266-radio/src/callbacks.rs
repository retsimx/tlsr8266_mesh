//! The four non-queue seams by which the SDK asks the application to act.
//!
//! Mesh RX, mesh TX and UART TX are handed to the application through the
//! module-level queues in [`super::queues`], because those paths are reachable
//! from the interrupt handler where a direct call edge must stay visible to
//! `sdk/stack_analysis.py`. The four callbacks here are different: they are
//! called from long-lived BLE/mesh processing, are registered exactly once at
//! boot, and are always installed for the same `App`. They therefore live in one
//! `Copy` struct behind one lock: [`BleAppHooks`] carries the mesh/light
//! surface.
//!
//! # Layer contract
//!
//! This module may only depend on the SDK itself, `core` and
//! `critical_section`. It must never reach up into the application layer: the
//! whole point of the seam is that the SDK does not know that `App` exists.
//!
//! # Unset-hook policy
//!
//! Registration happens in `app::main_task` before `rf_drv_init(true)`, so no
//! production path can observe an empty slot. If a future change breaks that
//! ordering the dispatchers **panic** with a fixed message rather than silently
//! dropping mesh traffic — a light that stops answering with no diagnostic is
//! much harder to debug than a panic report.
//!
//! The host-test build is the one exception: its slot starts populated (with
//! `TEST_NOOP_HOOKS`) so that tests which do not care about a seam need no
//! setup, and so an unregistered seam cannot turn an unrelated unit test into a
//! panic. Tests that *do* care install their own hooks with
//! `set_ble_app_hooks` (see the tests at the bottom).

use core::cell::Cell;

use critical_section::Mutex;

use mesh_protocol::packet_types::{Packet, PacketAttValue};

/// Application entry points the SDK may invoke.
///
/// Every slot is a plain `fn` pointer so the whole struct is `Copy` and can be
/// read out of the critical section in one go.
#[derive(Copy, Clone)]
pub struct BleAppHooks {
    /// Advance the mesh pairing state machine (called from the BLE slave loop).
    pub on_mesh_pair_proc: fn(),
    /// Deliver a mesh packet addressed to this device.
    pub on_light_data: fn(&Packet),
    /// Fill in an application response for a status request.
    pub on_light_response: fn(&mut PacketAttValue, &PacketAttValue) -> Result<(), ()>,
    /// Forward a received mesh packet to the host UART bridge.
    pub on_uart_mesh_rx: fn(&Packet),
}

/// Host builds start every seam installed and inert.
///
/// The gate is `target_os`, not `test`: the firmware crate compiles the radio as
/// a *dependency*, which never carries the radio's own `cfg(test)`, and its test
/// fixture needs the same populated slots this crate's unit tests get.
#[cfg(not(target_os = "none"))]
static BLE_APP_HOOKS: Mutex<Cell<Option<BleAppHooks>>> =
    Mutex::new(Cell::new(Some(TEST_NOOP_HOOKS)));

#[cfg(target_os = "none")]
static BLE_APP_HOOKS: Mutex<Cell<Option<BleAppHooks>>> = Mutex::new(Cell::new(None));

/// Registers the application entry points.
///
/// Called once from `app::main_task`, before `rf_drv_init(true)`, so no RF or
/// pairing code can run against an empty slot.
///
/// `#[inline(never)]`: see [`on_light_data`] for why every function in this
/// module is kept out of line.
#[inline(never)]
pub fn set_ble_app_hooks(hooks: BleAppHooks) {
    critical_section::with(|cs| BLE_APP_HOOKS.borrow(cs).set(Some(hooks)));
}

/// Reads the registered hooks, or panics if registration never happened.
///
/// Shared by all four dispatchers and kept out of line so the ELF carries a
/// single copy of the lock-read instead of four.
#[inline(never)]
fn hooks_or_panic() -> BleAppHooks {
    match critical_section::with(|cs| BLE_APP_HOOKS.borrow(cs).get()) {
        Some(hooks) => hooks,
        None => panic!("BLE app hooks are not registered"),
    }
}

/// Runs the mesh pairing state machine hook.
///
/// `#[inline(never)]` on every dispatcher in this module is load-bearing, not
/// cosmetic: `OPT_INLINE_THRESHOLD=50` would otherwise fold a one-instruction
/// dispatcher into its caller and erase the symbol from the ELF, and
/// `sdk/stack_analysis.py`'s `INDIRECT_EDGES` matches the call edge on that
/// symbol.
#[inline(never)]
pub fn on_mesh_pair_proc() {
    let hooks = hooks_or_panic();
    (hooks.on_mesh_pair_proc)();
}

/// Delivers a received mesh packet to the application hook.
///
/// `#[inline(never)]`: see [`on_mesh_pair_proc`].
#[inline(never)]
pub fn on_light_data(pkt: &Packet) {
    let hooks = hooks_or_panic();
    (hooks.on_light_data)(pkt);
}

/// Fills an application response for a status request through the hook.
///
/// `#[inline(never)]`: see [`on_mesh_pair_proc`].
#[inline(never)]
pub fn on_light_response(rsp: &mut PacketAttValue, src: &PacketAttValue) -> Result<(), ()> {
    let hooks = hooks_or_panic();
    (hooks.on_light_response)(rsp, src)
}

/// Forwards a received mesh packet to the host UART bridge through the hook.
///
/// `#[inline(never)]`: see [`on_mesh_pair_proc`].
#[inline(never)]
pub fn on_uart_mesh_rx(pkt: &Packet) {
    let hooks = hooks_or_panic();
    (hooks.on_uart_mesh_rx)(pkt);
}

#[cfg(not(target_os = "none"))]
fn noop_mesh_pair_proc() {}

#[cfg(not(target_os = "none"))]
fn noop_light_data(_: &Packet) {}

#[cfg(not(target_os = "none"))]
fn noop_light_response(_: &mut PacketAttValue, _: &PacketAttValue) -> Result<(), ()> {
    // "No application response" is the only safe no-op: returning `Ok(())`
    // would make the SDK queue a response packet the application never built.
    Err(())
}

#[cfg(not(target_os = "none"))]
fn noop_uart_mesh_rx(_: &Packet) {}

/// Host-build default: every seam is installed but does nothing.
#[cfg(not(target_os = "none"))]
pub(crate) const TEST_NOOP_HOOKS: BleAppHooks = BleAppHooks {
    on_mesh_pair_proc: noop_mesh_pair_proc,
    on_light_data: noop_light_data,
    on_light_response: noop_light_response,
    on_uart_mesh_rx: noop_uart_mesh_rx,
};

/// Restore the hook slot to its host-build default.
///
/// Called by the fixtures so a test that installed recording hooks cannot leak
/// them into the next one.
#[cfg(not(target_os = "none"))]
pub fn reset_for_test() {
    set_ble_app_hooks(TEST_NOOP_HOOKS);
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicUsize, Ordering};

    static PAIR_CALLS: AtomicUsize = AtomicUsize::new(0);
    static DATA_CALLS: AtomicUsize = AtomicUsize::new(0);
    static RESPONSE_CALLS: AtomicUsize = AtomicUsize::new(0);
    static UART_CALLS: AtomicUsize = AtomicUsize::new(0);

    fn record_pair() {
        PAIR_CALLS.fetch_add(1, Ordering::Relaxed);
    }

    fn record_data(_: &Packet) {
        DATA_CALLS.fetch_add(1, Ordering::Relaxed);
    }

    fn record_response(_: &mut PacketAttValue, _: &PacketAttValue) -> Result<(), ()> {
        RESPONSE_CALLS.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    fn record_uart(_: &Packet) {
        UART_CALLS.fetch_add(1, Ordering::Relaxed);
    }

    fn test_packet() -> Packet {
        use mesh_protocol::packet_types::{PacketAttCmd, PacketL2capHead};

        Packet {
            att_cmd: PacketAttCmd {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: 0,
                    rf_len: 0,
                    l2cap_len: 0,
                    chan_id: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: PacketAttValue::default(),
            },
        }
    }

    /// The four dispatchers must reach the function pointers that
    /// `set_ble_app_hooks` stored, and must report the hook's own result.
    #[test]
    fn test_set_ble_app_hooks_dispatches_to_registered_functions() {
        set_ble_app_hooks(BleAppHooks {
            on_mesh_pair_proc: record_pair,
            on_light_data: record_data,
            on_light_response: record_response,
            on_uart_mesh_rx: record_uart,
        });

        let packet = test_packet();
        let mut rsp = PacketAttValue::default();
        let src = PacketAttValue::default();

        on_mesh_pair_proc();
        on_light_data(&packet);
        on_uart_mesh_rx(&packet);
        assert_eq!(
            on_light_response(&mut rsp, &src),
            Ok(()),
            "the dispatcher must return the hook's result"
        );

        assert_eq!(PAIR_CALLS.load(Ordering::Relaxed), 1);
        assert_eq!(DATA_CALLS.load(Ordering::Relaxed), 1);
        assert_eq!(RESPONSE_CALLS.load(Ordering::Relaxed), 1);
        assert_eq!(UART_CALLS.load(Ordering::Relaxed), 1);

        // Restore the host-test default so later tests see the documented no-op
        // behaviour rather than these recording hooks.
        set_ble_app_hooks(TEST_NOOP_HOOKS);
    }

    /// The host-test default must be a quiet no-op and must not claim a
    /// response was produced.
    #[test]
    fn test_test_default_hooks_are_noops() {
        let packet = test_packet();
        let mut rsp = PacketAttValue::default();
        let src = PacketAttValue::default();

        on_mesh_pair_proc();
        on_light_data(&packet);
        on_uart_mesh_rx(&packet);
        assert_eq!(on_light_response(&mut rsp, &src), Err(()));
    }
}
