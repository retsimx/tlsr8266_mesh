#![allow(dead_code)]

use core::cell::UnsafeCell;
use crate::sdk::ble_app::ll_irq::IrqTracker;

/// A "mutex" that only allows borrowing from thread mode.
///
/// # Safety
///
/// **This Mutex is only safe on single-core systems.**
///
/// On multi-core systems, a `ThreadModeMutex` **is not sufficient** to ensure exclusive access.
pub struct ThreadModeMutex<T: ?Sized> {
    inner: UnsafeCell<T>,
}

// NOTE: ThreadModeMutex only allows borrowing from one execution context ever: thread mode.
// Therefore it cannot be used to send non-sendable stuff between execution contexts, so it can
// be Send+Sync even if T is not Send (unlike CriticalSectionMutex)
unsafe impl<T: ?Sized> Sync for ThreadModeMutex<T> {}
unsafe impl<T: ?Sized> Send for ThreadModeMutex<T> {}

impl<T> ThreadModeMutex<T> {
    /// Creates a new mutex
    pub const fn new(value: T) -> Self {
        ThreadModeMutex {
            inner: UnsafeCell::new(value),
        }
    }
}

impl<T: ?Sized> ThreadModeMutex<T> {
    /// Lock the `ThreadModeMutex`, granting access to the data.
    ///
    /// # Panics
    ///
    /// This will panic if not currently running in thread mode.
    pub fn lock<R>(&self, f: impl FnOnce(&T) -> R) -> R {
        f(self.borrow())
    }

    /// Borrows the data
    ///
    /// # Panics
    ///
    /// This will panic if not currently running in thread mode.
    pub fn borrow(&self) -> &T {
        assert!(
            in_thread_mode(),
            "ThreadModeMutex can only be borrowed from thread mode."
        );
        unsafe { &*self.inner.get() }
    }
}

impl<T: ?Sized> Drop for ThreadModeMutex<T> {
    fn drop(&mut self) {
        // Only allow dropping from thread mode. Dropping calls drop on the inner `T`, so
        // `drop` needs the same guarantees as `lock`. `ThreadModeMutex<T>` is Send even if
        // T isn't, so without this check a user could create a ThreadModeMutex in thread mode,
        // send it to interrupt context and drop it there, which would "send" a T even if T is not Send.
        assert!(
            in_thread_mode(),
            "ThreadModeMutex can only be dropped from thread mode."
        );

        // Drop of the inner `T` happens after this.
    }
}


/// A "mutex" that only allows borrowing from irq mode.
///
/// # Safety
///
/// **This Mutex is only safe on single-core systems.**
///
/// On multi-core systems, a `IrqModeMutex` **is not sufficient** to ensure exclusive access.
pub struct IrqModeMutex<T: ?Sized> {
    inner: UnsafeCell<T>,
}

// NOTE: IrqModeMutex only allows borrowing from one execution context ever: irq mode.
// Therefore it cannot be used to send non-sendable stuff between execution contexts, so it can
// be Send+Sync even if T is not Send (unlike CriticalSectionMutex)
unsafe impl<T: ?Sized> Sync for IrqModeMutex<T> {}
unsafe impl<T: ?Sized> Send for IrqModeMutex<T> {}

impl<T> IrqModeMutex<T> {
    /// Creates a new mutex
    pub const fn new(value: T) -> Self {
        IrqModeMutex {
            inner: UnsafeCell::new(value),
        }
    }
}

impl<T: ?Sized> IrqModeMutex<T> {
    /// Lock the `IrqModeMutex`, granting access to the data.
    ///
    /// # Panics
    ///
    /// This will panic if not currently running in irq mode.
    pub fn lock<R>(&self, f: impl FnOnce(&T) -> R) -> R {
        f(self.borrow())
    }

    /// Borrows the data
    ///
    /// # Panics
    ///
    /// This will panic if not currently running in irq mode.
    pub fn borrow(&self) -> &T {
        assert!(
            !in_thread_mode(),
            "IrqModeMutex can only be borrowed from irq mode."
        );
        unsafe { &*self.inner.get() }
    }
}

impl<T: ?Sized> Drop for IrqModeMutex<T> {
    fn drop(&mut self) {
        // Only allow dropping from irq mode. Dropping calls drop on the inner `T`, so
        // `drop` needs the same guarantees as `lock`. `IrqModeMutex<T>` is Send even if
        // T isn't, so without this check a user could create a IrqModeMutex in irq mode,
        // send it to interrupt context and drop it there, which would "send" a T even if T is not Send.
        assert!(
            !in_thread_mode(),
            "IrqModeMutex can only be dropped from irq mode."
        );

        // Drop of the inner `T` happens after this.
    }
}

pub(crate) fn in_thread_mode() -> bool {
    return !IrqTracker::in_irq();
}
