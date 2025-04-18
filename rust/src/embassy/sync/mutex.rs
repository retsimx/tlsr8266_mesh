#![allow(dead_code)]

use core::cell::UnsafeCell;
use core::fmt;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicU8, Ordering};
use crate::sdk::ble_app::ll_irq::IrqTracker;

use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};

/// NB: Poisoning is not required for the TLSR8266 because if a panic occurs while the lock is held
/// then the entire chip will reboot anyway. So using a mutex after a panic is impossible. Due to
/// this the poisoning code has been removed to reduce code size and execution overheads


/// Raw mutex trait.
///
/// This mutex is "raw", which means it does not actually contain the protected data, it
/// just implements the mutex mechanism. For most uses you should use [`super::Mutex`] instead,
/// which is generic over a RawMutex and contains the protected data.
///
/// Note that, unlike other mutexes, implementations only guarantee no
/// concurrent access from other threads: concurrent access from the current
/// thread is allwed. For example, it's possible to lock the same mutex multiple times reentrantly.
///
/// Therefore, locking a `RawMutex` is only enough to guarantee safe shared (`&`) access
/// to the data, it is not enough to guarantee exclusive (`&mut`) access.
///
/// # Safety
///
/// RawMutex implementations must ensure that, while locked, no other thread can lock
/// the RawMutex concurrently.
///
/// Unsafe code is allowed to rely on this fact, so incorrect implementations will cause undefined behavior.
pub unsafe trait RawMutex {
    /// Create a new `RawMutex` instance.
    ///
    /// This is a const instead of a method to allow creating instances in const context.
    const INIT: Self;

    /// Lock this `RawMutex`.
    fn lock(&self);

    /// Unlock this `RawMutex`.
    fn unlock(&self);
}

pub struct CriticalSectionRawMutex {
    _phantom: PhantomData<()>,
    _restore_state: AtomicU8
}

unsafe impl Send for CriticalSectionRawMutex {}
unsafe impl Sync for CriticalSectionRawMutex {}

impl CriticalSectionRawMutex {
    /// Create a new `CriticalSectionRawMutex`.
    pub const fn new() -> Self {
        Self { _phantom: PhantomData, _restore_state: AtomicU8::new(0) }
    }
}

unsafe impl RawMutex for CriticalSectionRawMutex {
    #[allow(clippy::declare_interior_mutable_const)]
    const INIT: Self = Self::new();

    #[inline(always)]
    fn lock(&self) {
        if !cfg!(test) {
            self._restore_state.store(irq_disable(), Ordering::Relaxed);
        }
    }

    #[inline(always)]
    fn unlock(&self) {
        if !cfg!(test) {
            irq_restore(self._restore_state.load(Ordering::Relaxed));
        }
    }
}


/// A "mutex" that only allows borrowing from thread mode.
///
/// # Safety
///
/// **This Mutex is only safe on single-core systems.**
///
/// On multi-core systems, a `ThreadModeRawMutex` **is not sufficient** to ensure exclusive access.
pub struct ThreadModeRawMutex {
    _phantom: PhantomData<()>,
}

unsafe impl Send for ThreadModeRawMutex {}
unsafe impl Sync for ThreadModeRawMutex {}

impl ThreadModeRawMutex {
    /// Create a new `ThreadModeRawMutex`.
    pub const fn new() -> Self {
        Self { _phantom: PhantomData }
    }
}

unsafe impl RawMutex for ThreadModeRawMutex {
    const INIT: Self = Self::new();

    #[inline(always)]
    fn lock(&self) {
        assert!(!in_irq_mode(), "ThreadModeMutex can only be locked from thread mode.");
    }

    #[inline(always)]
    fn unlock(&self) {
        // Only allow dropping from thread mode. Dropping calls drop on the inner `T`, so
        // `drop` needs the same guarantees as `lock`. `ThreadModeMutex<T>` is Send even if
        // T isn't, so without this check a user could create a ThreadModeMutex in thread mode,
        // send it to interrupt context and drop it there, which would "send" a T even if T is not Send.
        assert!(
            !in_irq_mode(),
            "ThreadModeMutex can only be dropped from thread mode."
        );

        // Drop of the inner `T` happens after this.
    }
}

impl Drop for ThreadModeRawMutex {
    #[inline(always)]
    fn drop(&mut self) {
        self.unlock();
    }
}

/// A "mutex" that only allows borrowing from Irq mode.
///
/// # Safety
///
/// **This Mutex is only safe on single-core systems.**
///
/// On multi-core systems, a `IrqModeRawMutex` **is not sufficient** to ensure exclusive access.
pub struct IrqModeRawMutex {
    _phantom: PhantomData<()>,
}

unsafe impl Send for IrqModeRawMutex {}
unsafe impl Sync for IrqModeRawMutex {}

impl IrqModeRawMutex {
    /// Create a new `IrqModeRawMutex`.
    pub const fn new() -> Self {
        Self { _phantom: PhantomData }
    }
}

unsafe impl RawMutex for IrqModeRawMutex {
    const INIT: Self = Self::new();

    #[inline(always)]
    fn lock(&self) {
        assert!(!in_irq_mode(), "IrqModeMutex can only be locked from Irq mode.");
    }

    #[inline(always)]
    fn unlock(&self) {
        // Only allow dropping from Irq mode. Dropping calls drop on the inner `T`, so
        // `drop` needs the same guarantees as `lock`. `IrqModeMutex<T>` is Send even if
        // T isn't, so without this check a user could create a IrqModeMutex in Irq mode,
        // send it to interrupt context and drop it there, which would "send" a T even if T is not Send.
        assert!(
            !in_irq_mode(),
            "IrqModeMutex can only be dropped from Irq mode."
        );

        // Drop of the inner `T` happens after this.
    }
}

impl Drop for IrqModeRawMutex {
    #[inline(always)]
    fn drop(&mut self) {
        self.unlock();
    }
}

#[inline(always)]
fn in_irq_mode() -> bool {
    IrqTracker::in_irq()
}

/// A mutual exclusion primitive useful for protecting shared data
///
/// This mutex will block threads waiting for the lock to become available. The
/// mutex can be created via a [`new`] constructor. Each mutex has a type parameter
/// which represents the data that it is protecting. The data can only be accessed
/// through the RAII guards returned from [`lock`] and [`try_lock`], which
/// guarantees that the data is only ever accessed when the mutex is locked.
///
/// # Poisoning
///
/// The mutexes in this module implement a strategy called "poisoning" where a
/// mutex is considered poisoned whenever a thread panics while holding the
/// mutex. Once a mutex is poisoned, all other threads are unable to access the
/// data by default as it is likely tainted (some invariant is not being
/// upheld).
///
/// For a mutex, this means that the [`lock`] and [`try_lock`] methods return a
/// [`Result`] which indicates whether a mutex has been poisoned or not. Most
/// usage of a mutex will simply [`unwrap()`] these results, propagating panics
/// among threads to ensure that a possibly invalid invariant is not witnessed.
///
/// A poisoned mutex, however, does not prevent all access to the underlying
/// data. The [`PoisonError`] type has an [`into_inner`] method which will return
/// the guard that would have otherwise been returned on a successful lock. This
/// allows access to the data, despite the lock being poisoned.
///
/// [`new`]: Self::new
/// [`lock`]: Self::lock
/// [`try_lock`]: Self::try_lock
/// [`unwrap()`]: Result::unwrap
/// [`PoisonError`]: super::PoisonError
/// [`into_inner`]: super::PoisonError::into_inner
///
/// # Examples
///
/// ```
/// use std::sync::{Arc, Mutex};
/// use std::thread;
/// use std::sync::mpsc::channel;
///
/// const N: usize = 10;
///
/// // Spawn a few threads to increment a shared variable (non-atomically), and
/// // let the main thread know once all increments are done.
/// //
/// // Here we're using an Arc to share memory among threads, and the data inside
/// // the Arc is protected with a mutex.
/// let data = Arc::new(Mutex::new(0));
///
/// let (tx, rx) = channel();
/// for _ in 0..N {
///     let (data, tx) = (Arc::clone(&data), tx.clone());
///     thread::spawn(move || {
///         // The shared state can only be accessed once the lock is held.
///         // Our non-atomic increment is safe because we're the only thread
///         // which can access the shared state when the lock is held.
///         let mut data = data.lock();
///         *data += 1;
///         if *data == N {
///             tx.send(()).unwrap();
///         }
///         // the lock is unlocked here when `data` goes out of scope.
///     });
/// }
///
/// rx.recv().unwrap();
/// ```
///
/// To recover from a poisoned mutex:
///
/// ```
/// use std::sync::{Arc, Mutex};
/// use std::thread;
///
/// let lock = Arc::new(Mutex::new(0_u32));
/// let lock2 = Arc::clone(&lock);
///
/// let _ = thread::spawn(move || -> () {
///     // This thread will acquire the mutex first, unwrapping the result of
///     // `lock` because the lock has not been poisoned.
///     let _guard = lock2.lock();
///
///     // This panic while holding the lock (`_guard` is in scope) will poison
///     // the mutex.
///     panic!();
/// }).join();
///
/// // The lock is poisoned by this point, but the returned result can be
/// // pattern matched on to return the underlying guard on both branches.
/// let mut guard = match lock.lock() {
///     Ok(guard) => guard,
///     Err(poisoned) => poisoned.into_inner(),
/// };
///
/// *guard += 1;
/// ```
///
/// To unlock a mutex guard sooner than the end of the enclosing scope,
/// either create an inner scope or drop the guard manually.
///
/// ```
/// use std::sync::{Arc, Mutex};
/// use std::thread;
///
/// const N: usize = 3;
///
/// let data_mutex = Arc::new(Mutex::new(vec![1, 2, 3, 4]));
/// let res_mutex = Arc::new(Mutex::new(0));
///
/// let mut threads = Vec::with_capacity(N);
/// (0..N).for_each(|_| {
///     let data_mutex_clone = Arc::clone(&data_mutex);
///     let res_mutex_clone = Arc::clone(&res_mutex);
///
///     threads.push(thread::spawn(move || {
///         // Here we use a block to limit the lifetime of the lock guard.
///         let result = {
///             let mut data = data_mutex_clone.lock();
///             // This is the result of some important and long-ish work.
///             let result = data.iter().fold(0, |acc, x| acc + x * 2);
///             data.push(result);
///             result
///             // The mutex guard gets dropped here, together with any other values
///             // created in the critical section.
///         };
///         // The guard created here is a temporary dropped at the end of the statement, i.e.
///         // the lock would not remain being held even if the thread did some additional work.
///         *res_mutex_clone.lock() += result;
///     }));
/// });
///
/// let mut data = data_mutex.lock();
/// // This is the result of some important and long-ish work.
/// let result = data.iter().fold(0, |acc, x| acc + x * 2);
/// data.push(result);
/// // We drop the `data` explicitly because it's not necessary anymore and the
/// // thread still has work to do. This allow other threads to start working on
/// // the data immediately, without waiting for the rest of the unrelated work
/// // to be done here.
/// //
/// // It's even more important here than in the threads because we `.join` the
/// // threads after that. If we had not dropped the mutex guard, a thread could
/// // be waiting forever for it, causing a deadlock.
/// // As in the threads, a block could have been used instead of calling the
/// // `drop` function.
/// drop(data);
/// // Here the mutex guard is not assigned to a variable and so, even if the
/// // scope does not end after this line, the mutex is still released: there is
/// // no deadlock.
/// *res_mutex.lock() += result;
///
/// threads.into_iter().for_each(|thread| {
///     thread
///         .join()
///         .expect("The thread creating or execution failed !")
/// });
///
/// assert_eq!(*res_mutex.lock(), 800);
/// ```
///

pub struct Mutex<M, T: ?Sized> {
    inner: M,
    //poison: Flag,
    data: UnsafeCell<T>,
}

// these are the only places where `T: Send` matters; all other
// functionality works fine on a single thread.
unsafe impl<M, T: ?Sized + Send> Send for Mutex<M, T> {}
unsafe impl<M, T: ?Sized + Send> Sync for Mutex<M, T> {}

/// An RAII implementation of a "scoped lock" of a mutex. When this structure is
/// dropped (falls out of scope), the lock will be unlocked.
///
/// The data protected by the mutex can be accessed through this guard via its
/// [`Deref`] and [`DerefMut`] implementations.
///
/// This structure is created by the [`lock`] and [`try_lock`] methods on
/// [`Mutex`].
///
/// [`lock`]: Mutex::lock
/// [`try_lock`]: Mutex::try_lock
pub struct MutexGuard<'a, M: RawMutex, T: ?Sized + 'a> {
    lock: &'a Mutex<M, T>
}

// impl<T: ?Sized> !Send for MutexGuard<'_, T> {}
unsafe impl<M: RawMutex, T: ?Sized + Sync> Sync for MutexGuard<'_, M, T> {}

impl<M: RawMutex, T> Mutex<M, T> {
    /// Creates a new mutex in an unlocked state ready for use.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::sync::Mutex;
    ///
    /// let mutex = Mutex::new(0);
    /// ```
    pub const fn new(t: T) -> Mutex<M, T> {
        Mutex { inner: M::INIT, data: UnsafeCell::new(t) }
    }
}

impl<M: RawMutex, T: ?Sized> Mutex<M, T> {
    /// Acquires a mutex, blocking the current thread until it is able to do so.
    ///
    /// This function will block the local thread until it is available to acquire
    /// the mutex. Upon returning, the thread is the only thread with the lock
    /// held. An RAII guard is returned to allow scoped unlock of the lock. When
    /// the guard goes out of scope, the mutex will be unlocked.
    ///
    /// The exact behavior on locking a mutex in the thread which already holds
    /// the lock is left unspecified. However, this function will not return on
    /// the second call (it might panic or deadlock, for example).
    ///
    /// # Errors
    ///
    /// If another user of this mutex panicked while holding the mutex, then
    /// this call will return an error once the mutex is acquired.
    ///
    /// # Panics
    ///
    /// This function might panic when called if the lock is already held by
    /// the current thread.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::sync::{Arc, Mutex};
    /// use std::thread;
    ///
    /// let mutex = Arc::new(Mutex::new(0));
    /// let c_mutex = Arc::clone(&mutex);
    ///
    /// thread::spawn(move || {
    ///     *c_mutex.lock() = 10;
    /// }).join().expect("thread::spawn failed");
    /// assert_eq!(*mutex.lock(), 10);
    /// ```
    #[inline(always)]
    pub fn lock(&self) -> MutexGuard<'_, M, T> {
        unsafe {
            self.inner.lock();
            MutexGuard::new(self)
        }
    }

    /// Immediately drops the guard, and consequently unlocks the mutex.
    ///
    /// This function is equivalent to calling [`drop`] on the guard but is more self-documenting.
    /// Alternately, the guard will be automatically dropped when it goes out of scope.
    ///
    /// ```
    /// #![feature(mutex_unlock)]
    ///
    /// use std::sync::Mutex;
    /// let mutex = Mutex::new(0);
    ///
    /// let mut guard = mutex.lock();
    /// *guard += 20;
    /// Mutex::unlock(guard);
    /// ```
    #[inline(always)]
    pub fn unlock(guard: MutexGuard<'_, M, T>) {
        drop(guard);
    }

    /// Consumes this mutex, returning the underlying data.
    ///
    /// # Errors
    ///
    /// If another user of this mutex panicked while holding the mutex, then
    /// this call will return an error instead.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::sync::Mutex;
    ///
    /// let mutex = Mutex::new(0);
    /// assert_eq!(mutex.into_inner().unwrap(), 0);
    /// ```
    pub fn into_inner(self) -> T
        where
            T: Sized,
    {
        self.data.into_inner()
    }

    /// Returns a mutable reference to the underlying data.
    ///
    /// Since this call borrows the `Mutex` mutably, no actual locking needs to
    /// take place -- the mutable borrow statically guarantees no locks exist.
    ///
    /// # Errors
    ///
    /// If another user of this mutex panicked while holding the mutex, then
    /// this call will return an error instead.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::sync::Mutex;
    ///
    /// let mut mutex = Mutex::new(0);
    /// *mutex.get_mut().unwrap() = 10;
    /// assert_eq!(*mutex.lock(), 10);
    /// ```
    pub fn get_mut(&mut self) -> &mut T {
        self.data.get_mut()
    }
}

impl<M: RawMutex, T> From<T> for Mutex<M, T> {
    /// Creates a new mutex in an unlocked state ready for use.
    /// This is equivalent to [`Mutex::new`].
    fn from(t: T) -> Self {
        Mutex::new(t)
    }
}

impl<M: RawMutex, T: ?Sized + Default> Default for Mutex<M, T> {
    /// Creates a `Mutex<T>`, with the `Default` value for T.
    fn default() -> Mutex<M, T> {
        Mutex::new(Default::default())
    }
}

impl<'mutex, M: RawMutex, T: ?Sized> MutexGuard<'mutex, M, T> {
    unsafe fn new(lock: &'mutex Mutex<M, T>) -> MutexGuard<'mutex, M, T> {
        MutexGuard { lock }
    }
}

impl<M: RawMutex, T: ?Sized> Deref for MutexGuard<'_, M, T> {
    type Target = T;

    fn deref(&self) -> &T {
        unsafe { &*self.lock.data.get() }
    }
}

impl<M: RawMutex, T: ?Sized> DerefMut for MutexGuard<'_, M, T> {
    fn deref_mut(&mut self) -> &mut T {
        unsafe { &mut *self.lock.data.get() }
    }
}

impl<M: RawMutex, T: ?Sized> Drop for MutexGuard<'_, M, T> {
    #[inline]
    fn drop(&mut self) {
        self.lock.inner.unlock();
    }
}

impl<M: RawMutex, T: ?Sized + fmt::Debug> fmt::Debug for MutexGuard<'_, M, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(&**self, f)
    }
}

impl<M: RawMutex, T: ?Sized + fmt::Display> fmt::Display for MutexGuard<'_, M, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        (**self).fmt(f)
    }
}

pub fn guard_lock<'a, M: RawMutex, T: ?Sized>(guard: &MutexGuard<'a, M, T>) -> &'a M {
    &guard.lock.inner
}

pub type CriticalSectionMutex<T> = Mutex<CriticalSectionRawMutex, T>;
pub type ThreadModeMutex<T> = Mutex<ThreadModeRawMutex, T>;
pub type IrqModeMutex<T> = Mutex<IrqModeRawMutex, T>;