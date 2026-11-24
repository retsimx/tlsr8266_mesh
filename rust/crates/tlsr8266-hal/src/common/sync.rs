//! Nest-counted critical-section mutex built only on the external
//! `critical_section` crate.
//!
//! # Why a single global depth counter
//!
//! `critical_section::acquire`/`release` must be paired 1:1 and properly nested
//! (see the crate's safety contract). Storing a restore state inside every mutex
//! instance cannot satisfy that: a nested or out-of-order drop would restore an
//! interrupt state captured at the wrong nesting level. Instead one global
//! `CS_DEPTH` counts held guards, and one `CS_SAVED` slot remembers the restore
//! state of the outermost acquire. Interrupts are therefore re-enabled exactly
//! once, when the last guard in the system is dropped, with the state that was
//! captured when interrupts were first disabled.
//!
//! # Mandatory invariant
//!
//! Never return a [`CriticalSectionMutexGuard`] out of a
//! `critical_section::with` closure: guard lifetimes must stay strictly nested
//! inside any enclosing critical-section scope. A guard outliving its enclosing
//! scope would let `CS_DEPTH` reach zero while the outer critical section is
//! still held, and the restore state would then be released too early.
//!
//! # Assumptions this safety model relies on
//!
//! 1. **Every preemption source on this part is maskable by
//!    `critical_section::acquire`** (a global interrupt disable). `enter_cs`
//!    reads `CS_DEPTH` *before* calling `acquire`, so an ISR can run in that
//!    window; correctness requires that such an ISR cannot run at all once a
//!    guard is held at depth > 0. If some source could preempt a
//!    depth-≥ 1 context, `enter_cs` would observe a non-zero depth and hand out
//!    exclusion-free access while interrupts are nominally "masked". This is
//!    why the embassy executor deliberately keeps the permanently
//!    latched USB 250 µs interrupt out of the wake mask: a permanently pending,
//!    unmaskable-by-`acquire` source would violate this assumption. Do not add
//!    such a source without revisiting the depth counter.
//! 2. **Any interrupt that takes a lock restores `CS_DEPTH` to its entry value
//!    before returning.** An ISR that acquired at depth 0 must release back to
//!    depth 0 (interrupts stay disabled for the ISR's whole body, which is the
//!    normal `critical_section`-style scope). Guards must not be leaked: an ISR
//!    that latched a guard across its `ret` would leave a depth level behind,
//!    so the interrupted thread would resume believing interrupts were already
//!    disabled, and the eventual `release` would enable them at the wrong
//!    point. In short: ISRs must not leak guards.
//! 3. **The panic path must never be reachable while a guard on the same mutex
//!    is held.** [`CriticalSectionMutex::lock`]'s re-entrancy assert exists only
//!    off-target (`not(target_os = "none")`); on the device a second `lock()` on
//!    the same mutex from the same context succeeds and hands out a second
//!    `&mut T`. With
//!    `panic = "abort"` there is no unwinding, so the outer guard is never
//!    dropped either. A panic raised inside a critical section whose handler
//!    then re-locks that mutex would therefore alias. The UART panic sink
//!    (`uart_manager::send_sync_panic`) takes `UART_HW`, so no guard region on
//!    `UART_HW` may contain a panic source: no indexing, no `unwrap`, no
//!    formatting.

use core::cell::UnsafeCell;
use core::ops::{Deref, DerefMut};
#[cfg(any(target_os = "none", test))]
use core::sync::atomic::compiler_fence;
use core::sync::atomic::{
    AtomicBool, AtomicI32, AtomicU16, AtomicU32, AtomicU8, AtomicUsize, Ordering,
};

#[cfg(any(target_os = "none", test))]
struct SavedState(UnsafeCell<critical_section::RestoreState>);

#[cfg(any(target_os = "none", test))]
// SAFETY: (4) `Sync` wrapper. Only touched while global interrupts are disabled
// by the outermost acquire path, so no ISR can observe it concurrently.
unsafe impl Sync for SavedState {}

/// Number of currently held critical-section levels.
///
/// Compiled on every target so there is exactly **one** depth counter in the
/// process. On device [`enter_cs`]/[`exit_cs`] own it; on host builds the
/// `critical_section` platform impl (`mcu::irq_i::TlsrCriticalSection`) owns it
/// through [`host_enter_cs`]/[`host_exit_cs`]. Host mutex guards are *not*
/// counted — the host `lock` uses its re-entrancy flag instead of a critical
/// section, so a host `cs_depth()` reports critical-section nesting only.
static CS_DEPTH: AtomicU8 = AtomicU8::new(0);
/// Restore state captured by the outermost `critical_section::acquire`.
#[cfg(any(target_os = "none", test))]
static CS_SAVED: SavedState =
    SavedState(UnsafeCell::new(critical_section::RestoreState::invalid()));

/// Current number of held critical-section levels.
///
/// Exists so any host-build test — in this crate or in a crate above it — can
/// prove that a code region it drives holds no critical section: the depth must
/// read the same value across a callback that must not run with interrupts
/// disabled. Host-only seam; the device build never compiles it, and on device
/// the depth is not readable at all.
///
/// Read it against a captured baseline rather than a literal `0`: a test that
/// panics inside a `critical_section::with` scope leaks a level for the rest of
/// the process, so only the *delta* is meaningful.
#[cfg(not(target_os = "none"))]
pub fn cs_depth() -> u8 {
    CS_DEPTH.load(Ordering::Relaxed)
}

/// Account one host critical-section level, mirroring [`enter_cs`]'s counting.
///
/// The host `critical_section` platform impl calls this from `acquire`, so host
/// units see the same counter the device counts in [`enter_cs`]. The
/// `critical_section` crate guarantees 1:1 nesting, so a plain increment is
/// correct; no restore state is involved because host builds touch no hardware.
#[cfg(not(target_os = "none"))]
#[inline]
pub(crate) fn host_enter_cs() {
    CS_DEPTH.fetch_add(1, Ordering::Relaxed);
}

/// Account one host critical-section level leaving, mirroring [`exit_cs`].
#[cfg(not(target_os = "none"))]
#[inline]
pub(crate) fn host_exit_cs() {
    CS_DEPTH.fetch_sub(1, Ordering::Relaxed);
}

/// Enter one nesting level, disabling interrupts only at the outermost level.
#[cfg(any(target_os = "none", test))]
#[inline]
fn enter_cs(acquire: impl FnOnce() -> critical_section::RestoreState) {
    // Read the depth *before* acquiring so a nested lock never touches the
    // restore-state slot or wastes an interrupt-disable.
    let depth = CS_DEPTH.load(Ordering::Relaxed);
    if depth == 0 {
        let state = acquire();
        // SAFETY: (4) `Sync`-wrapper interior. Interrupts are disabled from here
        // until the matching `exit_cs` release, so only this context sees it.
        unsafe { *CS_SAVED.0.get() = state };
    }
    CS_DEPTH.store(depth + 1, Ordering::Relaxed);
    compiler_fence(Ordering::Acquire);
}

/// Leave one nesting level, restoring interrupts only at the outermost level.
#[cfg(any(target_os = "none", test))]
#[inline]
fn exit_cs(release: impl FnOnce(critical_section::RestoreState)) {
    compiler_fence(Ordering::Release);
    // Every guard took a level in `enter_cs`, so this cannot underflow.
    let remaining = CS_DEPTH.load(Ordering::Relaxed) - 1;
    CS_DEPTH.store(remaining, Ordering::Relaxed);
    if remaining == 0 {
        // SAFETY: (4) `Sync`-wrapper interior. Written by the outermost
        // `enter_cs` before this level existed.
        let saved = unsafe { *CS_SAVED.0.get() };
        release(saved);
    }
}

/// A mutex that keeps interrupts disabled while its guard is alive.
///
/// See the [module docs](self) for the safety model.
#[cfg_attr(target_os = "none", repr(transparent))]
pub struct CriticalSectionMutex<T: ?Sized> {
    #[cfg(not(target_os = "none"))]
    locked: AtomicBool,
    data: UnsafeCell<T>,
}

// SAFETY: (4) `Send` wrapper. `T: Send` bounds moving the protected value between
// execution contexts, and every access to `data` is serialized by the
// interrupt-disabled critical section.
unsafe impl<T: ?Sized + Send> Send for CriticalSectionMutex<T> {}
// SAFETY: (4) `Sync` wrapper. Sharing is sound because every access to `data`
// goes through a guard that holds the interrupt-disabled critical section (or,
// off-target, the re-entrancy probe), so no two contexts can alias it.
unsafe impl<T: ?Sized + Send> Sync for CriticalSectionMutex<T> {}

impl<T> CriticalSectionMutex<T> {
    /// Create a new mutex in an unlocked state. Usable in `const` context.
    pub const fn new(t: T) -> Self {
        Self {
            #[cfg(not(target_os = "none"))]
            locked: AtomicBool::new(false),
            data: UnsafeCell::new(t),
        }
    }

    /// Raw pointer to the protected value, for handing a stable address to code
    /// that cannot take a guard (e.g. the GATT attribute table's value slots).
    ///
    /// The returned pointer carries exactly the same aliasing obligations as
    /// [`UnsafeCell::get`]: dereferencing it is `unsafe`, and any access through
    /// it must not overlap a live [`CriticalSectionMutexGuard`] on this mutex.
    pub const fn as_ptr(&self) -> *mut T {
        self.data.get()
    }
}

impl<T: ?Sized> CriticalSectionMutex<T> {
    /// Acquire the mutex, disabling interrupts until the guard is dropped.
    ///
    /// # Panics
    ///
    /// Under off-target builds only, panics if the same mutex is already locked
    /// in this thread; the device path has no re-entrancy check.
    #[inline(always)]
    pub fn lock(&self) -> CriticalSectionMutexGuard<'_, T> {
        #[cfg(not(target_os = "none"))]
        assert!(
            !self.locked.swap(true, Ordering::Relaxed),
            "re-entrant lock on same CriticalSectionMutex"
        );
        #[cfg(target_os = "none")]
        // SAFETY: (2) `critical_section::acquire` contract. `enter_cs` calls this only
        // at depth 0 and stores the returned state in `CS_SAVED` for the matching
        // `release`, so acquire/release stay 1:1 and properly nested.
        enter_cs(|| unsafe { critical_section::acquire() });
        CriticalSectionMutexGuard { lock: self }
    }
}

/// RAII guard returned by [`CriticalSectionMutex::lock`].
pub struct CriticalSectionMutexGuard<'a, T: ?Sized> {
    lock: &'a CriticalSectionMutex<T>,
}

impl<T: ?Sized> Deref for CriticalSectionMutexGuard<'_, T> {
    type Target = T;

    #[inline(always)]
    fn deref(&self) -> &T {
        // SAFETY: (4) `Sync`-wrapper interior. The guard holds the lock, so no
        // other context can access `data`.
        unsafe { &*self.lock.data.get() }
    }
}

impl<T: ?Sized> DerefMut for CriticalSectionMutexGuard<'_, T> {
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: (4) `Sync`-wrapper interior. `&mut self` proves this guard is
        // the only accessor.
        unsafe { &mut *self.lock.data.get() }
    }
}

impl<T: ?Sized> Drop for CriticalSectionMutexGuard<'_, T> {
    #[inline(always)]
    fn drop(&mut self) {
        #[cfg(not(target_os = "none"))]
        self.lock.locked.store(false, Ordering::Relaxed);
        #[cfg(target_os = "none")]
        // SAFETY: (2) `critical_section::release` contract. `exit_cs` decrements the
        // depth first and reaches this only at the outermost level, restoring exactly
        // the state captured by the matching `acquire`.
        exit_cs(|state| unsafe { critical_section::release(state) });
    }
}

pub trait SimplifyLS<T> {
    fn get(&self) -> T;
    fn set(&self, val: T);
    fn inc(&self);
    fn dec(&self);
}

impl SimplifyLS<bool> for AtomicBool {
    fn get(&self) -> bool {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: bool) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) {
        panic!("Not valid for bool");
    }
    fn dec(&self) {
        panic!("Not valid for bool")
    }
}

impl SimplifyLS<u8> for AtomicU8 {
    fn get(&self) -> u8 {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: u8) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) {
        self.set(self.get().wrapping_add(1));
    }
    fn dec(&self) {
        self.set(self.get().wrapping_sub(1));
    }
}

impl SimplifyLS<u16> for AtomicU16 {
    fn get(&self) -> u16 {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: u16) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) {
        self.set(self.get().wrapping_add(1));
    }
    fn dec(&self) {
        self.set(self.get().wrapping_sub(1));
    }
}

impl SimplifyLS<u32> for AtomicU32 {
    fn get(&self) -> u32 {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: u32) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) {
        self.set(self.get().wrapping_add(1));
    }
    fn dec(&self) {
        self.set(self.get().wrapping_sub(1));
    }
}

impl SimplifyLS<usize> for AtomicUsize {
    fn get(&self) -> usize {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: usize) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) {
        self.set(self.get().wrapping_add(1));
    }
    fn dec(&self) {
        self.set(self.get().wrapping_sub(1));
    }
}

impl SimplifyLS<i32> for AtomicI32 {
    fn get(&self) -> i32 {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: i32) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) {
        self.set(self.get().wrapping_add(1));
    }
    fn dec(&self) {
        self.set(self.get().wrapping_sub(1));
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use core::cell::{Cell, RefCell};
    use std::sync::Mutex as StdMutex;

    /// `CS_DEPTH`/`CS_SAVED` are process-global, so the tests that drive the
    /// helpers directly must not overlap with each other.
    static SERIAL: StdMutex<()> = StdMutex::new(());

    fn serial() -> std::sync::MutexGuard<'static, ()> {
        SERIAL.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// `cs_depth` is the exposed view of the global guard depth: zero outside a
    /// critical section, one inside it, and zero again after the outermost exit.
    #[test]
    fn cs_depth_tracks_the_outermost_critical_section() {
        let _serial = serial();
        assert_eq!(cs_depth(), 0, "no guard is held at test start");

        enter_cs(|| critical_section::RestoreState::invalid());
        assert_eq!(cs_depth(), 1, "one held guard must read as depth 1");

        exit_cs(|_| {});
        assert_eq!(cs_depth(), 0, "the released guard must return to depth 0");
    }

    /// The platform impl (`mcu::irq_i::TlsrCriticalSection`) must feed the same
    /// counter `enter_cs` feeds, so a plain `critical_section::with` is visible
    /// to `cs_depth`: one level above the baseline inside the scope, and the
    /// baseline again once it returns.
    #[test]
    fn critical_section_with_is_visible_to_cs_depth() {
        let _serial = serial();
        let base = cs_depth();
        assert_eq!(base, 0, "no critical section is held at test start");

        let inside = critical_section::with(|_| cs_depth());
        assert_eq!(
            inside,
            base + 1,
            "a `critical_section::with` scope must read as one level deep"
        );
        assert_eq!(
            cs_depth(),
            base,
            "the depth must return to the baseline once the scope exits"
        );
    }

    #[test]
    fn nested_lock_acquires_once_and_releases_once() {
        let _serial = serial();
        let acquires = Cell::new(0);
        let releases = Cell::new(0);

        enter_cs(|| {
            acquires.set(acquires.get() + 1);
            critical_section::RestoreState::invalid()
        });
        // Inner lock: the acquire closure is not expected to run.
        enter_cs(|| {
            acquires.set(acquires.get() + 1);
            critical_section::RestoreState::invalid()
        });
        assert_eq!(
            acquires.get(),
            1,
            "inner lock reuses the outer critical section"
        );
        assert_eq!(releases.get(), 0);

        exit_cs(|_| releases.set(releases.get() + 1));
        assert_eq!(
            releases.get(),
            0,
            "inner unlock must not restore interrupts"
        );

        exit_cs(|_| releases.set(releases.get() + 1));
        assert_eq!(
            releases.get(),
            1,
            "outermost unlock restores interrupts once"
        );
    }

    #[test]
    fn out_of_order_drop_of_two_mutexes_releases_at_depth_zero() {
        let _serial = serial();
        let events = RefCell::new(Vec::new());

        // First mutex's lock.
        enter_cs(|| {
            events.borrow_mut().push("acquire");
            critical_section::RestoreState::invalid()
        });
        // Second (distinct) mutex's lock, nested while the first is held.
        enter_cs(|| {
            events.borrow_mut().push("second-acquire");
            critical_section::RestoreState::invalid()
        });
        assert_eq!(*events.borrow(), ["acquire"]);

        // Drop the older guard first: depth 2 -> 1 must not restore interrupts.
        exit_cs(|_| events.borrow_mut().push("premature-release"));
        assert_eq!(*events.borrow(), ["acquire"]);

        // Dropping the remaining guard reaches depth 0 and releases the state
        // saved by the outermost acquire.
        exit_cs(|_| events.borrow_mut().push("release"));
        assert_eq!(*events.borrow(), ["acquire", "release"]);
    }

    #[test]
    #[should_panic(expected = "re-entrant lock on same CriticalSectionMutex")]
    fn locking_the_same_mutex_twice_panics() {
        let mutex = CriticalSectionMutex::new(0u8);
        let _outer = mutex.lock();
        let _inner = mutex.lock();
    }

    #[test]
    fn guard_deref_and_deref_mut_round_trip() {
        let mutex = CriticalSectionMutex::new(41u32);

        {
            let mut guard = mutex.lock();
            assert_eq!(*guard, 41);
            *guard += 1;
        }

        // The guard's Drop must have released the mutex for this second lock.
        assert_eq!(*mutex.lock(), 42);
    }

    #[test]
    #[should_panic(expected = "Not valid for bool")]
    fn test_simplifyls_bool_inc_panics() {
        use super::SimplifyLS;

        let flag = AtomicBool::new(false);
        flag.inc();
    }

    #[test]
    #[should_panic(expected = "Not valid for bool")]
    fn test_simplifyls_bool_dec_panics() {
        use super::SimplifyLS;

        let flag = AtomicBool::new(false);
        flag.dec();
    }

    #[test]
    fn test_simplifyls_u8_get_set_and_wraps() {
        use super::SimplifyLS;

        let value = AtomicU8::new(0);
        value.set(10);
        assert_eq!(value.get(), 10);

        value.set(u8::MAX);
        value.inc();
        assert_eq!(value.get(), 0);

        value.set(0);
        value.dec();
        assert_eq!(value.get(), u8::MAX);
    }

    #[test]
    fn test_simplifyls_u16_get_set_and_wraps() {
        use super::SimplifyLS;

        let value = AtomicU16::new(0);
        value.set(42);
        assert_eq!(value.get(), 42);

        value.set(u16::MAX);
        value.inc();
        assert_eq!(value.get(), 0);

        value.set(0);
        value.dec();
        assert_eq!(value.get(), u16::MAX);
    }

    #[test]
    fn test_simplifyls_u32_get_set_and_wraps() {
        use super::SimplifyLS;

        let value = AtomicU32::new(0);
        value.set(100);
        assert_eq!(value.get(), 100);

        value.set(u32::MAX);
        value.inc();
        assert_eq!(value.get(), 0);

        value.set(0);
        value.dec();
        assert_eq!(value.get(), u32::MAX);
    }

    #[test]
    fn test_simplifyls_usize_get_set_and_wraps() {
        use super::SimplifyLS;

        let value = AtomicUsize::new(0);
        value.set(7);
        assert_eq!(value.get(), 7);

        value.set(usize::MAX);
        value.inc();
        assert_eq!(value.get(), 0);

        value.set(0);
        value.dec();
        assert_eq!(value.get(), usize::MAX);
    }

    #[test]
    fn test_simplifyls_i32_get_set_and_wraps() {
        use super::SimplifyLS;

        let value = AtomicI32::new(0);
        value.set(-12);
        assert_eq!(value.get(), -12);

        value.set(i32::MAX);
        value.inc();
        assert_eq!(value.get(), i32::MIN);

        value.set(i32::MIN);
        value.dec();
        assert_eq!(value.get(), i32::MAX);
    }
}
