//! The eight non-queue seams by which the SDK asks the application to act.
//!
//! Mesh RX, mesh TX and UART TX are handed to the application through the
//! module-level queues in [`super::queues`], because those paths are reachable
//! from the interrupt handler where a direct call edge must stay visible to
//! `sdk/stack_analysis.py`. The eight callbacks here are different: they are
//! called from long-lived BLE/mesh processing, are registered exactly once at
//! boot, and are always installed for the same `App`. They therefore live in
//! `Copy` structs behind one lock each instead of eight independent slots:
//! [`BleAppHooks`] carries the mesh/light surface, [`OtaHooks`] the OTA one.
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
//! The host-test build is the one exception: its slots start populated (with
//! `TEST_NOOP_HOOKS` and `TEST_OTA_HOOKS`) so that tests which do not care about
//! a seam need no setup, and so an unregistered seam cannot turn an unrelated
//! unit test into a panic. Tests that *do* care mock the dispatcher itself (see
//! `#[cfg_attr(test, mry::mry)]` below).

use core::cell::Cell;

use critical_section::Mutex;

use mesh_protocol::mesh_types::OtaStatusSnapshot;
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

#[cfg(not(test))]
static BLE_APP_HOOKS: Mutex<Cell<Option<BleAppHooks>>> = Mutex::new(Cell::new(None));

#[cfg(test)]
static BLE_APP_HOOKS: Mutex<Cell<Option<BleAppHooks>>> =
    Mutex::new(Cell::new(Some(TEST_NOOP_HOOKS)));

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
#[cfg_attr(test, mry::mry)]
#[inline(never)]
pub fn on_mesh_pair_proc() {
    let hooks = hooks_or_panic();
    (hooks.on_mesh_pair_proc)();
}

/// Delivers a received mesh packet to the application hook.
///
/// `#[inline(never)]`: see [`on_mesh_pair_proc`].
#[cfg_attr(test, mry::mry)]
#[inline(never)]
pub fn on_light_data(pkt: &Packet) {
    let hooks = hooks_or_panic();
    (hooks.on_light_data)(pkt);
}

/// Fills an application response for a status request through the hook.
///
/// `#[inline(never)]`: see [`on_mesh_pair_proc`].
#[cfg_attr(test, mry::mry)]
#[inline(never)]
pub fn on_light_response(rsp: &mut PacketAttValue, src: &PacketAttValue) -> Result<(), ()> {
    let hooks = hooks_or_panic();
    (hooks.on_light_response)(rsp, src)
}

/// Forwards a received mesh packet to the host UART bridge through the hook.
///
/// `#[inline(never)]`: see [`on_mesh_pair_proc`].
#[cfg_attr(test, mry::mry)]
#[inline(never)]
pub fn on_uart_mesh_rx(pkt: &Packet) {
    let hooks = hooks_or_panic();
    (hooks.on_uart_mesh_rx)(pkt);
}

#[cfg(test)]
fn noop_mesh_pair_proc() {}

#[cfg(test)]
fn noop_light_data(_: &Packet) {}

#[cfg(test)]
fn noop_light_response(_: &mut PacketAttValue, _: &PacketAttValue) -> Result<(), ()> {
    // "No application response" is the only safe no-op: returning `Ok(())`
    // would make the SDK queue a response packet the application never built.
    Err(())
}

#[cfg(test)]
fn noop_uart_mesh_rx(_: &Packet) {}

/// Host-test default: every seam is installed but does nothing.
#[cfg(test)]
const TEST_NOOP_HOOKS: BleAppHooks = BleAppHooks {
    on_mesh_pair_proc: noop_mesh_pair_proc,
    on_light_data: noop_light_data,
    on_light_response: noop_light_response,
    on_uart_mesh_rx: noop_uart_mesh_rx,
};

/// OTA entry points the SDK may invoke.
///
/// Every slot is a plain `fn` pointer so the whole struct is `Copy` and can be
/// read out of the critical section in one go. Ownership runs SDK → `ota`:
/// `ota` implements all four, this module only calls them, and `app::main_task`
/// installs them before `rf_drv_init(true)`. A dispatcher that ran first would
/// panic in `ota_hooks_or_panic` — a silently dropped OTA write is unrecoverable.
///
/// There is deliberately **one** status getter returning the whole
/// [`OtaStatusSnapshot`] rather than one getter per flag: four identical-shaped
/// `fn() -> bool` getters are exactly what `mergefunc` folds into a single
/// ambiguous symbol, and `sdk/stack_analysis.py` binds the indirect edge to that
/// symbol.
#[derive(Copy, Clone)]
pub struct OtaHooks {
    /// Deliver one `TELINK_SPP_DATA_OTA_UUID` write.
    pub on_gatt_ota_write: fn(&Packet),
    /// Read the OTA flags as one snapshot.
    pub status_snapshot: fn() -> OtaStatusSnapshot,
    /// Ask the finish path to send its terminate notifications.
    pub request_termination: fn(),
    /// Advance the OTA supervision timeout by one second; `true` when it expired.
    pub tick_timeout_1s: fn() -> bool,
}

#[cfg(not(test))]
static OTA_HOOKS: Mutex<Cell<Option<OtaHooks>>> = Mutex::new(Cell::new(None));

#[cfg(test)]
static OTA_HOOKS: Mutex<Cell<Option<OtaHooks>>> = Mutex::new(Cell::new(Some(TEST_OTA_HOOKS)));

/// Registers the OTA entry points.
///
/// Called once from `app::main_task`, before `rf_drv_init(true)`, next to
/// [`set_ble_app_hooks`].
///
/// `#[inline(never)]`: see [`on_mesh_pair_proc`] for why every function in this
/// module is kept out of line.
#[inline(never)]
pub fn set_ota_hooks(hooks: OtaHooks) {
    critical_section::with(|cs| OTA_HOOKS.borrow(cs).set(Some(hooks)));
}

/// Reads the registered OTA hooks, or panics if registration never happened.
///
/// Shared by all four OTA dispatchers and kept out of line so the ELF carries a
/// single copy of the lock-read instead of four.
#[inline(never)]
fn ota_hooks_or_panic() -> OtaHooks {
    match critical_section::with(|cs| OTA_HOOKS.borrow(cs).get()) {
        Some(hooks) => hooks,
        None => panic!("OTA hooks are not registered"),
    }
}

/// Delivers one GATT OTA write through the hook.
///
/// `#[inline(never)]`: see [`on_mesh_pair_proc`].
#[cfg_attr(test, mry::mry)]
#[inline(never)]
pub fn dispatch_ota_write(p: &Packet) {
    let hooks = ota_hooks_or_panic();
    (hooks.on_gatt_ota_write)(p);
}

/// Reads the OTA flags through the hook.
///
/// `#[inline(never)]`: see [`on_mesh_pair_proc`].
#[cfg_attr(test, mry::mry)]
#[inline(never)]
pub fn dispatch_ota_snapshot() -> OtaStatusSnapshot {
    let hooks = ota_hooks_or_panic();
    (hooks.status_snapshot)()
}

/// Asks the OTA finish path for its terminate notifications through the hook.
///
/// `#[inline(never)]`: see [`on_mesh_pair_proc`].
#[cfg_attr(test, mry::mry)]
#[inline(never)]
pub fn dispatch_ota_terminate() {
    let hooks = ota_hooks_or_panic();
    (hooks.request_termination)();
}

/// Advances the OTA supervision timeout through the hook.
///
/// `#[inline(never)]`: see [`on_mesh_pair_proc`].
#[cfg_attr(test, mry::mry)]
#[inline(never)]
pub fn dispatch_ota_tick() -> bool {
    let hooks = ota_hooks_or_panic();
    (hooks.tick_timeout_1s)()
}

/// Host-test default: the **real** `ota` implementations.
///
/// About twenty existing ble_app tests flip the OTA flags and then assert
/// OTA-gated branches. A no-op default would silently change what those tests
/// verify, so the slot starts wired to the real subsystem instead. `ota` is
/// below `sdk` in the layering graph, and this item is `#[cfg(test)]`, so the
/// layering gate (which strips test code) still sees a clean SDK file.
#[cfg(test)]
pub(crate) const TEST_OTA_HOOKS: OtaHooks = OtaHooks {
    on_gatt_ota_write: crate::ota::on_gatt_ota_packet,
    status_snapshot: crate::ota::ota_status_snapshot_impl,
    request_termination: crate::ota::ota_request_termination_impl,
    tick_timeout_1s: crate::ota::ota_tick_timeout_1s_impl,
};

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

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

    // ================================================================================
    // OTA hook seam
    // ================================================================================

    static OTA_WRITE_CALLS: AtomicUsize = AtomicUsize::new(0);
    static OTA_SNAPSHOT_CALLS: AtomicUsize = AtomicUsize::new(0);
    static OTA_TERMINATE_CALLS: AtomicUsize = AtomicUsize::new(0);
    static OTA_TICK_CALLS: AtomicUsize = AtomicUsize::new(0);
    /// Value the snapshot recorder reports; the dispatcher must return it
    /// verbatim so the four slots stay independently observable.
    static OTA_SNAPSHOT_IN_PROGRESS: AtomicBool = AtomicBool::new(false);
    /// Value the tick recorder reports.
    static OTA_TICK_RESULT: AtomicBool = AtomicBool::new(false);

    fn reset_ota_recorders() {
        OTA_WRITE_CALLS.store(0, Ordering::SeqCst);
        OTA_SNAPSHOT_CALLS.store(0, Ordering::SeqCst);
        OTA_TERMINATE_CALLS.store(0, Ordering::SeqCst);
        OTA_TICK_CALLS.store(0, Ordering::SeqCst);
        OTA_SNAPSHOT_IN_PROGRESS.store(false, Ordering::SeqCst);
        OTA_TICK_RESULT.store(false, Ordering::SeqCst);
    }

    fn record_ota_write(_: &Packet) {
        OTA_WRITE_CALLS.fetch_add(1, Ordering::SeqCst);
    }

    fn record_ota_snapshot() -> OtaStatusSnapshot {
        OTA_SNAPSHOT_CALLS.fetch_add(1, Ordering::SeqCst);
        OtaStatusSnapshot {
            in_progress: OTA_SNAPSHOT_IN_PROGRESS.load(Ordering::SeqCst),
            finish_status: mesh_protocol::mesh_types::OtaState::Continue,
            timeout_expired: false,
        }
    }

    fn record_ota_terminate() {
        OTA_TERMINATE_CALLS.fetch_add(1, Ordering::SeqCst);
    }

    fn record_ota_tick() -> bool {
        OTA_TICK_CALLS.fetch_add(1, Ordering::SeqCst);
        OTA_TICK_RESULT.load(Ordering::SeqCst)
    }

    /// Each OTA dispatcher must reach its own registered slot and report that
    /// slot's result.
    #[test]
    fn test_set_ota_hooks_dispatches_to_registered_functions() {
        reset_ota_recorders();
        set_ota_hooks(OtaHooks {
            on_gatt_ota_write: record_ota_write,
            status_snapshot: record_ota_snapshot,
            request_termination: record_ota_terminate,
            tick_timeout_1s: record_ota_tick,
        });

        let packet = test_packet();
        dispatch_ota_write(&packet);
        assert_eq!(
            dispatch_ota_snapshot().finish_status,
            mesh_protocol::mesh_types::OtaState::Continue
        );
        dispatch_ota_terminate();
        assert_eq!(dispatch_ota_tick(), false);

        assert_eq!(OTA_WRITE_CALLS.load(Ordering::SeqCst), 1);
        assert_eq!(OTA_SNAPSHOT_CALLS.load(Ordering::SeqCst), 1);
        assert_eq!(OTA_TERMINATE_CALLS.load(Ordering::SeqCst), 1);
        assert_eq!(OTA_TICK_CALLS.load(Ordering::SeqCst), 1);

        // The dispatchers must not substitute one slot for another.
        OTA_SNAPSHOT_IN_PROGRESS.store(true, Ordering::SeqCst);
        OTA_TICK_RESULT.store(true, Ordering::SeqCst);
        assert_eq!(dispatch_ota_snapshot().in_progress, true);
        assert_eq!(dispatch_ota_tick(), true);

        set_ota_hooks(TEST_OTA_HOOKS);
    }

    /// Calling one dispatcher must reach exactly one slot; the four slots are
    /// independent, so a folded or swapped slot fails here.
    #[test]
    fn test_ota_hook_slots_are_independent() {
        reset_ota_recorders();
        set_ota_hooks(OtaHooks {
            on_gatt_ota_write: record_ota_write,
            status_snapshot: record_ota_snapshot,
            request_termination: record_ota_terminate,
            tick_timeout_1s: record_ota_tick,
        });

        dispatch_ota_write(&test_packet());
        assert_eq!(
            (
                OTA_WRITE_CALLS.load(Ordering::SeqCst),
                OTA_SNAPSHOT_CALLS.load(Ordering::SeqCst),
                OTA_TERMINATE_CALLS.load(Ordering::SeqCst),
                OTA_TICK_CALLS.load(Ordering::SeqCst),
            ),
            (1, 0, 0, 0)
        );

        dispatch_ota_snapshot();
        assert_eq!(
            (
                OTA_WRITE_CALLS.load(Ordering::SeqCst),
                OTA_SNAPSHOT_CALLS.load(Ordering::SeqCst),
                OTA_TERMINATE_CALLS.load(Ordering::SeqCst),
                OTA_TICK_CALLS.load(Ordering::SeqCst),
            ),
            (1, 1, 0, 0)
        );

        dispatch_ota_terminate();
        assert_eq!(
            (
                OTA_WRITE_CALLS.load(Ordering::SeqCst),
                OTA_SNAPSHOT_CALLS.load(Ordering::SeqCst),
                OTA_TERMINATE_CALLS.load(Ordering::SeqCst),
                OTA_TICK_CALLS.load(Ordering::SeqCst),
            ),
            (1, 1, 1, 0)
        );

        dispatch_ota_tick();
        assert_eq!(
            (
                OTA_WRITE_CALLS.load(Ordering::SeqCst),
                OTA_SNAPSHOT_CALLS.load(Ordering::SeqCst),
                OTA_TERMINATE_CALLS.load(Ordering::SeqCst),
                OTA_TICK_CALLS.load(Ordering::SeqCst),
            ),
            (1, 1, 1, 1)
        );

        set_ota_hooks(TEST_OTA_HOOKS);
    }

    /// The host-test default must really reach the `ota` subsystem: a flag set
    /// through one dispatcher is visible through another, and a GATT write goes
    /// through the real handler.
    #[test]
    fn test_test_ota_hooks_default_reaches_ota() {
        crate::ota::reset_for_test();
        set_ota_hooks(TEST_OTA_HOOKS);

        // The terminate slot reaches `ota_request_termination_impl`.
        assert_eq!(crate::ota::is_termination_requested(), false);
        dispatch_ota_terminate();
        assert_eq!(crate::ota::is_termination_requested(), true);

        // The tick slot reaches `ota_tick_timeout_1s_impl`: a zero counter is
        // already expired and must not underflow.
        crate::ota::set_timeout_for_test(0);
        assert_eq!(dispatch_ota_tick(), true);

        // The snapshot slot reaches `ota_status_snapshot_impl`.
        crate::ota::set_in_progress_for_test(true);
        assert_eq!(dispatch_ota_snapshot().in_progress, true);

        // The write slot reaches `on_gatt_ota_packet`, which claims the OTA when
        // the begin gate allows it.
        crate::ota::set_in_progress_for_test(false);
        crate::ota::set_gatt_ota_begin_for_test(true);
        dispatch_ota_write(&test_packet());
        assert_eq!(dispatch_ota_snapshot().in_progress, true);

        crate::ota::reset_for_test();
    }
}
