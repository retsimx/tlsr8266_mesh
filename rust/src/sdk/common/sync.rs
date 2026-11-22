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

use core::cell::UnsafeCell;
use core::ops::{Deref, DerefMut};
#[cfg(test)]
use core::sync::atomic::AtomicBool;
use core::sync::atomic::{compiler_fence, AtomicU8, Ordering};

struct SavedState(UnsafeCell<critical_section::RestoreState>);

// SAFETY: only touched while global interrupts are disabled by the acquire path
// of the outermost critical section, so no ISR can observe it concurrently.
unsafe impl Sync for SavedState {}

/// Number of currently held mutex guards.
static CS_DEPTH: AtomicU8 = AtomicU8::new(0);
/// Restore state captured by the outermost `critical_section::acquire`.
static CS_SAVED: SavedState =
    SavedState(UnsafeCell::new(critical_section::RestoreState::invalid()));

/// Enter one nesting level, disabling interrupts only at the outermost level.
#[inline]
fn enter_cs(acquire: impl FnOnce() -> critical_section::RestoreState) {
    // Read the depth *before* acquiring so a nested lock never touches the
    // restore-state slot or wastes an interrupt-disable.
    let depth = CS_DEPTH.load(Ordering::Relaxed);
    if depth == 0 {
        let state = acquire();
        // SAFETY: interrupts are disabled from here until the matching release
        // in `exit_cs`, so this cell is only reachable from the current context.
        unsafe { *CS_SAVED.0.get() = state };
    }
    CS_DEPTH.store(depth + 1, Ordering::Relaxed);
    compiler_fence(Ordering::Acquire);
}

/// Leave one nesting level, restoring interrupts only at the outermost level.
#[inline]
fn exit_cs(release: impl FnOnce(critical_section::RestoreState)) {
    compiler_fence(Ordering::Release);
    // Every guard took a level in `enter_cs`, so this cannot underflow.
    let remaining = CS_DEPTH.load(Ordering::Relaxed) - 1;
    CS_DEPTH.store(remaining, Ordering::Relaxed);
    if remaining == 0 {
        // SAFETY: written by the outermost `enter_cs` before this level existed.
        let saved = unsafe { *CS_SAVED.0.get() };
        release(saved);
    }
}

/// A mutex that keeps interrupts disabled while its guard is alive.
///
/// See the [module docs](self) for the safety model.
#[cfg_attr(not(test), repr(transparent))]
pub struct CriticalSectionMutex<T: ?Sized> {
    #[cfg(test)]
    locked: AtomicBool,
    data: UnsafeCell<T>,
}

// SAFETY: exclusive access to `data` is guaranteed by the interrupt-disabled
// critical section (and, under `cfg(test)`, by the re-entrancy probe).
unsafe impl<T: ?Sized + Send> Send for CriticalSectionMutex<T> {}
unsafe impl<T: ?Sized + Send> Sync for CriticalSectionMutex<T> {}

impl<T> CriticalSectionMutex<T> {
    /// Create a new mutex in an unlocked state. Usable in `const` context.
    pub const fn new(t: T) -> Self {
        Self {
            #[cfg(test)]
            locked: AtomicBool::new(false),
            data: UnsafeCell::new(t),
        }
    }
}

impl<T: ?Sized> CriticalSectionMutex<T> {
    /// Acquire the mutex, disabling interrupts until the guard is dropped.
    ///
    /// # Panics
    ///
    /// Under `cfg(test)` only, panics if the same mutex is already locked in
    /// this thread; the production path has no re-entrancy check.
    #[inline(always)]
    pub fn lock(&self) -> CriticalSectionMutexGuard<'_, T> {
        #[cfg(test)]
        assert!(
            !self.locked.swap(true, Ordering::Relaxed),
            "re-entrant lock on same CriticalSectionMutex"
        );
        #[cfg(not(test))]
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
        // SAFETY: the guard holds the lock, so no other context can access `data`.
        unsafe { &*self.lock.data.get() }
    }
}

impl<T: ?Sized> DerefMut for CriticalSectionMutexGuard<'_, T> {
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: `&mut self` proves this guard is the only accessor.
        unsafe { &mut *self.lock.data.get() }
    }
}

impl<T: ?Sized> Drop for CriticalSectionMutexGuard<'_, T> {
    #[inline(always)]
    fn drop(&mut self) {
        #[cfg(test)]
        self.lock.locked.store(false, Ordering::Relaxed);
        #[cfg(not(test))]
        exit_cs(|state| unsafe { critical_section::release(state) });
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
}
