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
//! unit test into a panic. Tests that *do* care install their own hooks with
//! `set_ble_app_hooks` / `set_ota_hooks` (see the tests at the bottom).

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

/// Host builds start every seam installed, wired to the in-crate OTA stand-in.
///
/// See [`BLE_APP_HOOKS`] for why the gate is `target_os`.
#[cfg(not(target_os = "none"))]
static OTA_HOOKS: Mutex<Cell<Option<OtaHooks>>> = Mutex::new(Cell::new(Some(TEST_OTA_HOOKS)));

#[cfg(target_os = "none")]
static OTA_HOOKS: Mutex<Cell<Option<OtaHooks>>> = Mutex::new(Cell::new(None));

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
#[inline(never)]
pub fn dispatch_ota_write(p: &Packet) {
    let hooks = ota_hooks_or_panic();
    (hooks.on_gatt_ota_write)(p);
}

/// Reads the OTA flags through the hook.
///
/// `#[inline(never)]`: see [`on_mesh_pair_proc`].
#[inline(never)]
pub fn dispatch_ota_snapshot() -> OtaStatusSnapshot {
    let hooks = ota_hooks_or_panic();
    (hooks.status_snapshot)()
}

/// Asks the OTA finish path for its terminate notifications through the hook.
///
/// `#[inline(never)]`: see [`on_mesh_pair_proc`].
#[inline(never)]
pub fn dispatch_ota_terminate() {
    let hooks = ota_hooks_or_panic();
    (hooks.request_termination)();
}

/// Advances the OTA supervision timeout through the hook.
///
/// `#[inline(never)]`: see [`on_mesh_pair_proc`].
#[inline(never)]
pub fn dispatch_ota_tick() -> bool {
    let hooks = ota_hooks_or_panic();
    (hooks.tick_timeout_1s)()
}

/// Host-build default: the crate's own OTA stand-in, [`test_ota`].
///
/// About thirty moved unit tests flip OTA flags and then assert OTA-gated
/// branches, so the default has to answer the four hook contracts instead of
/// being inert — an inert default would silently weaken what those tests
/// verify (design 032 §6.2). It is nevertheless *neutral*: it lives in this
/// crate, records nothing outside it, and never names the application's `ota`
/// subsystem, which a dependency crate cannot do at all. The firmware's own
/// tests install the real hooks with [`set_ota_hooks`] in their fixture.
#[cfg(not(target_os = "none"))]
pub(crate) const TEST_OTA_HOOKS: OtaHooks = test_ota::HOOKS;

/// Restore both hook slots to their host-build defaults and clear the OTA
/// stand-in.
///
/// Called by the fixtures so a test that installed recording hooks cannot leak
/// them into the next one.
#[cfg(not(target_os = "none"))]
pub fn reset_for_test() {
    test_ota::reset_for_test();
    set_ble_app_hooks(TEST_NOOP_HOOKS);
    set_ota_hooks(TEST_OTA_HOOKS);
}

/// The in-crate OTA stand-in behind [`TEST_OTA_HOOKS`] on host builds.
///
/// It owns exactly the flags the moved unit tests drive — in progress, the
/// supervision countdown, the terminate request, the transfer-completion state
/// and the GATT-begin gate — and nothing else: no staging buffer, no flash, no
/// reboot. Tests reach it through the same `*_for_test` names the application's
/// OTA module exposes, so the conversion of the old imports was a path
/// substitution rather than a rewrite of what each test asserts.
#[cfg(not(target_os = "none"))]
pub mod test_ota {
    use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU8, AtomicUsize, Ordering};

    use mesh_protocol::mesh_types::{
        OtaState, OtaStatusSnapshot, RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS,
    };
    use mesh_protocol::packet_types::Packet;

    use super::OtaHooks;

    static IN_PROGRESS: AtomicBool = AtomicBool::new(false);
    static TIMEOUT_SECONDS: AtomicU16 = AtomicU16::new(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);
    static TERMINATION_REQUESTED: AtomicBool = AtomicBool::new(false);
    static FINISHED_FLAG: AtomicU8 = AtomicU8::new(OtaState::Continue as u8);
    /// Stands in for the firmware's "may a GATT OTA start?" gate.
    static GATT_OTA_BEGIN: AtomicBool = AtomicBool::new(false);
    /// Number of writes that reached the hook, so a test can prove the write
    /// slot is wired without the staging machinery.
    static WRITES: AtomicUsize = AtomicUsize::new(0);

    /// The hook set that reads and writes this stand-in.
    pub const HOOKS: OtaHooks = OtaHooks {
        on_gatt_ota_write: write,
        status_snapshot: snapshot,
        request_termination,
        tick_timeout_1s,
    };

    fn finished_flag() -> OtaState {
        match FINISHED_FLAG.load(Ordering::Relaxed) {
            0 => OtaState::Continue,
            1 => OtaState::Ok,
            2 => OtaState::Error,
            _ => OtaState::MasterOtaRebootOnly,
        }
    }

    /// The projection the link layer reads.
    pub fn snapshot() -> OtaStatusSnapshot {
        OtaStatusSnapshot {
            in_progress: IN_PROGRESS.load(Ordering::Relaxed),
            finish_status: finished_flag(),
            timeout_expired: TIMEOUT_SECONDS.load(Ordering::Relaxed) == 0,
        }
    }

    /// One GATT OTA write.
    ///
    /// Mirrors the application handler's gates: nothing is accepted once the
    /// transfer has left `Continue`, and a transfer starts only when the begin
    /// gate allows it.
    pub fn write(_: &Packet) {
        WRITES.fetch_add(1, Ordering::Relaxed);
        if finished_flag() != OtaState::Continue {
            return;
        }
        if !IN_PROGRESS.load(Ordering::Relaxed) {
            if !GATT_OTA_BEGIN.load(Ordering::Relaxed) {
                return;
            }
            IN_PROGRESS.store(true, Ordering::Relaxed);
        }
    }

    /// Ask the finish path to notify the peer.
    pub fn request_termination() {
        TERMINATION_REQUESTED.store(true, Ordering::Relaxed);
    }

    /// One second of supervision timeout; `true` when the counter reads zero.
    pub fn tick_timeout_1s() -> bool {
        let seconds = TIMEOUT_SECONDS.load(Ordering::Relaxed);
        if seconds != 0 {
            TIMEOUT_SECONDS.store(seconds - 1, Ordering::Relaxed);
        }
        TIMEOUT_SECONDS.load(Ordering::Relaxed) == 0
    }

    /// How many writes the hook has seen.
    pub fn write_count() -> usize {
        WRITES.load(Ordering::Relaxed)
    }

    /// Restore every flag to its boot value and re-install [`HOOKS`].
    pub fn reset_for_test() {
        IN_PROGRESS.store(false, Ordering::Relaxed);
        TIMEOUT_SECONDS.store(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS, Ordering::Relaxed);
        TERMINATION_REQUESTED.store(false, Ordering::Relaxed);
        FINISHED_FLAG.store(OtaState::Continue as u8, Ordering::Relaxed);
        GATT_OTA_BEGIN.store(false, Ordering::Relaxed);
        WRITES.store(0, Ordering::Relaxed);
        super::set_ota_hooks(HOOKS);
    }

    /// Overwrite the transfer-completion flag.
    pub fn set_finished_flag_for_test(state: OtaState) {
        FINISHED_FLAG.store(state as u8, Ordering::Relaxed);
    }

    /// Overwrite the supervision timeout, in seconds.
    pub fn set_timeout_for_test(seconds: u16) {
        TIMEOUT_SECONDS.store(seconds, Ordering::Relaxed);
    }

    /// Read the supervision timeout back, in seconds.
    pub fn timeout_seconds_for_test() -> u16 {
        TIMEOUT_SECONDS.load(Ordering::Relaxed)
    }

    /// Overwrite the BLE-OTA in-progress flag.
    pub fn set_in_progress_for_test(in_progress: bool) {
        IN_PROGRESS.store(in_progress, Ordering::Relaxed);
    }

    /// Did the finish path get asked to notify the peer?
    pub fn is_termination_requested() -> bool {
        TERMINATION_REQUESTED.load(Ordering::Relaxed)
    }

    /// Overwrite the terminate-request flag.
    pub fn set_termination_requested_for_test(requested: bool) {
        TERMINATION_REQUESTED.store(requested, Ordering::Relaxed);
    }

    /// Allow (or forbid) a GATT OTA to begin.
    pub fn set_gatt_ota_begin_for_test(ok: bool) {
        GATT_OTA_BEGIN.store(ok, Ordering::Relaxed);
    }
}

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
        assert!(!dispatch_ota_tick());

        assert_eq!(OTA_WRITE_CALLS.load(Ordering::SeqCst), 1);
        assert_eq!(OTA_SNAPSHOT_CALLS.load(Ordering::SeqCst), 1);
        assert_eq!(OTA_TERMINATE_CALLS.load(Ordering::SeqCst), 1);
        assert_eq!(OTA_TICK_CALLS.load(Ordering::SeqCst), 1);

        // The dispatchers must not substitute one slot for another.
        OTA_SNAPSHOT_IN_PROGRESS.store(true, Ordering::SeqCst);
        OTA_TICK_RESULT.store(true, Ordering::SeqCst);
        assert!(dispatch_ota_snapshot().in_progress);
        assert!(dispatch_ota_tick());

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
        crate::callbacks::test_ota::reset_for_test();
        set_ota_hooks(TEST_OTA_HOOKS);

        // The terminate slot reaches `ota_request_termination_impl`.
        assert!(!crate::callbacks::test_ota::is_termination_requested());
        dispatch_ota_terminate();
        assert!(crate::callbacks::test_ota::is_termination_requested());

        // The tick slot reaches `ota_tick_timeout_1s_impl`: a zero counter is
        // already expired and must not underflow.
        crate::callbacks::test_ota::set_timeout_for_test(0);
        assert!(dispatch_ota_tick());

        // The snapshot slot reaches `ota_status_snapshot_impl`.
        crate::callbacks::test_ota::set_in_progress_for_test(true);
        assert!(dispatch_ota_snapshot().in_progress);

        // The write slot reaches `on_gatt_ota_packet`, which claims the OTA when
        // the begin gate allows it.
        crate::callbacks::test_ota::set_in_progress_for_test(false);
        crate::callbacks::test_ota::set_gatt_ota_begin_for_test(true);
        dispatch_ota_write(&test_packet());
        assert!(dispatch_ota_snapshot().in_progress);

        crate::callbacks::test_ota::reset_for_test();
    }
}
