use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

/// Cooperatively gives up a timeslice to the task scheduler.
///
/// Calling this function will move the currently executing future to the back
/// of the execution queue, making room for other futures to execute. This is
/// especially useful after running CPU-intensive operations inside a future.
///
/// # Example
/// ```
/// async fn cpu_intensive_task() {
///     // Some CPU intensive work
///     // ...
///     
///     // Yield to let other tasks run
///     yield_now().await;
///     
///     // Continue with more work
/// }
/// ```
///
/// # How It Works
/// This function creates a `YieldNow` future that will yield control back to
/// the executor on its first poll. When polled again, it completes.
///
/// See also [`task::spawn_blocking`].
///
/// [`task::spawn_blocking`]: fn.spawn_blocking.html
#[inline]
#[cfg_attr(test, mry::mry)]
pub async fn yield_now() {
    // Create and await a YieldNow future with initial state set to false
    YieldNow(false).await
}

/// A future that yields control back to the executor once.
///
/// This struct is used internally by `yield_now()` to implement the yielding behavior.
/// The boolean field tracks whether the future has already yielded once.
struct YieldNow(bool);

impl Future for YieldNow {
    type Output = ();

    /// Implements the poll method for the YieldNow future.
    ///
    /// # Algorithm
    /// 1. On first poll (when self.0 is false):
    ///    - Set self.0 to true to track that we've yielded once
    ///    - Wake the waker to reschedule this future
    ///    - Return Poll::Pending to yield control
    /// 2. On second poll (when self.0 is true):
    ///    - Return Poll::Ready(()) to complete the future
    ///
    /// The futures executor is implemented as a FIFO queue, so all this future
    /// does is re-schedule the future back to the end of the queue, giving room
    /// for other futures to progress.
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        if !self.0 {
            // First poll: we need to yield
            self.0 = true;  // Mark that we've yielded once
            cx.waker().wake_by_ref();  // Reschedule this future
            Poll::Pending  // Yield control back to the executor
        } else {
            // Second poll: we've already yielded, so complete the future
            Poll::Ready(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::task::{RawWaker, RawWakerVTable, Waker};
    
    // Mock waker implementation for testing
    static VTABLE: RawWakerVTable = RawWakerVTable::new(
        |_| RawWaker::new(core::ptr::null(), &VTABLE),
        |_| {},
        |_| {},
        |_| {},
    );
    
    fn create_waker() -> Waker {
        // Track if the waker was called
        unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) }
    }
    
    #[test]
    fn test_yield_now_future() {
        // Create the future
        let mut yield_now_fut = YieldNow(false);
        
        // Create a waker and context
        let waker = create_waker();
        let mut cx = Context::from_waker(&waker);
        
        // First poll should return Pending
        let mut pinned = Pin::new(&mut yield_now_fut);
        assert!(matches!(pinned.as_mut().poll(&mut cx), Poll::Pending));
        
        // Get a reference to the future to check its state after polling
        // (avoiding the borrow conflict)
        let yield_now_after_poll = Pin::into_inner(pinned.as_mut());
        assert_eq!(yield_now_after_poll.0, true); // Internal state should be changed
        
        // Second poll should return Ready
        assert!(matches!(pinned.poll(&mut cx), Poll::Ready(())));
    }
    
    #[test]
    fn test_yield_now_already_yielded() {
        // Create the future with state already set to true (already yielded)
        let mut yield_now_fut = YieldNow(true);
        
        // Create a waker and context
        let waker = create_waker();
        let mut cx = Context::from_waker(&waker);
        
        // First poll should return Ready immediately since we've already yielded
        let pinned = Pin::new(&mut yield_now_fut);
        assert!(matches!(pinned.poll(&mut cx), Poll::Ready(())));
    }
    
    #[test]
    fn test_yield_now_with_mock_executor() {
        use core::cell::Cell;
        use std::rc::Rc;
        
        // Create a counter to track waker calls
        let wake_counter = Rc::new(Cell::new(0));
        
        // Custom VTABLE for our counter waker
        fn clone_waker(data: *const ()) -> RawWaker {
            let counter = unsafe { Rc::from_raw(data as *const Cell<u32>) };
            let counter_clone = counter.clone();
            std::mem::forget(counter); // Don't drop the original
            RawWaker::new(Rc::into_raw(counter_clone) as *const (), &COUNTER_VTABLE)
        }
        
        fn wake(data: *const ()) {
            let counter = unsafe { Rc::from_raw(data as *const Cell<u32>) };
            counter.set(counter.get() + 1);
            drop(counter); // Actually drop it this time
        }
        
        fn wake_by_ref(data: *const ()) {
            let counter = unsafe { Rc::from_raw(data as *const Cell<u32>) };
            counter.set(counter.get() + 1);
            std::mem::forget(counter); // Don't drop the original
        }
        
        fn drop_waker(data: *const ()) {
            drop(unsafe { Rc::from_raw(data as *const Cell<u32>) });
        }
        
        static COUNTER_VTABLE: RawWakerVTable = RawWakerVTable::new(
            clone_waker,
            wake,
            wake_by_ref,
            drop_waker,
        );
        
        // Create a waker that will increment our counter
        let waker = unsafe {
            Waker::from_raw(RawWaker::new(
                Rc::into_raw(wake_counter.clone()) as *const (),
                &COUNTER_VTABLE,
            ))
        };
        let mut cx = Context::from_waker(&waker);
        
        // Run a basic "executor" that polls the future until it completes
        let mut future = YieldNow(false);
        
        // First poll - should return Pending
        let mut pinned = Pin::new(&mut future);
        assert!(matches!(pinned.as_mut().poll(&mut cx), Poll::Pending));
        
        // The waker should have been called via wake_by_ref
        assert!(wake_counter.get() > 0, "Waker was not called (counter is still 0)");
        
        // Second poll - should return Ready
        assert!(matches!(pinned.poll(&mut cx), Poll::Ready(())));
    }
}