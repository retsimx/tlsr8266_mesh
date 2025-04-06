use core::sync::atomic::{AtomicU32, Ordering};
use embassy_time::driver::{Driver, AlarmHandle};
use crate::sdk::mcu::clock::clock_time;

/// Custom time driver implementation for the embassy-time crate
/// 
/// This driver bridges the SDK's clock_time functionality with embassy's time system,
/// providing a monotonic 64-bit timestamp counter even though the underlying
/// hardware may have a 32-bit counter that wraps around.
struct MyDriver {
    // No fields needed as we use global state for time tracking
}

// Register our driver with the embassy-time framework
embassy_time::time_driver_impl!(static DRIVER: MyDriver = MyDriver{});

impl Driver for MyDriver {
    /// Returns the current monotonic timestamp in ticks
    /// 
    /// Uses clock_time64 to handle timer overflow and provide a continuous 64-bit counter
    fn now(&self) -> u64 {
        clock_time64()
    }

    /// Allocates a new alarm handle
    /// 
    /// Currently only supports a single alarm with ID 0
    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        Option::from(AlarmHandle::new(0))
    }

    /// Sets the callback function for an alarm
    /// 
    /// Currently a no-op implementation as alarms are not fully supported
    fn set_alarm_callback(&self, _alarm: AlarmHandle, _callback: fn(*mut ()), _ctx: *mut ()) {
        // No-op: Alarm callbacks not implemented in this driver
    }

    /// Sets an alarm to trigger at the specified timestamp
    /// 
    /// Returns true only if the timestamp is in the future
    /// Note: This is a minimal implementation that doesn't actually schedule alarms
    fn set_alarm(&self, _alarm: AlarmHandle, _timestamp: u64) -> bool {
        _timestamp > self.now()
    }
}

#[cfg(test)]
static CLOCK_TIME_UPPER: AtomicU32 = AtomicU32::new(0);
#[cfg(test)]
static LAST_CLOCK_TIME: AtomicU32 = AtomicU32::new(0);

/// Provides a 64-bit monotonic clock based on the 32-bit clock_time()
///
/// This function extends the 32-bit hardware timer to a 64-bit logical timer by
/// tracking overflows of the underlying 32-bit counter. It uses atomic operations
/// to safely handle concurrent access without locks in the common case.
/// 
/// # Algorithm
/// 1. Read current hardware time and previously seen time
/// 2. If current time < last time, we may have an overflow
/// 3. Use a critical section to safely update the upper 32 bits if overflow confirmed
/// 4. Return a combined 64-bit timestamp (upper 32 bits | lower 32 bits)
#[cfg_attr(test, mry::mry)]
pub fn clock_time64() -> u64 {
    // When not testing, these static variables are defined here
    // When testing, they're defined at module level for test access
    #[cfg(not(test))]
    static CLOCK_TIME_UPPER: AtomicU32 = AtomicU32::new(0);
    #[cfg(not(test))]
    static LAST_CLOCK_TIME: AtomicU32 = AtomicU32::new(0);

    // Get current hardware time
    let current_time = clock_time();
    let last_time = LAST_CLOCK_TIME.load(Ordering::Relaxed);
    
    // Only enter critical section if we suspect an overflow
    // (current time is less than last seen time)
    if current_time < last_time {
        critical_section::with(|_| {
            // Re-check within critical section to avoid race conditions
            // This prevents multiple threads from incrementing the upper bits
            let last_time_cs = LAST_CLOCK_TIME.load(Ordering::Relaxed);
            if current_time < last_time_cs {
                // Overflow confirmed - increment upper bits
                // Use load-modify-store instead of fetch_add since fetch_add might not be supported
                // on all platforms or with all atomics implementations
                let upper = CLOCK_TIME_UPPER.load(Ordering::Relaxed);
                CLOCK_TIME_UPPER.store(upper + 1, Ordering::Relaxed);
            }
            
            // Update last time seen within the critical section
            LAST_CLOCK_TIME.store(current_time, Ordering::Relaxed);
        });
    } else {
        // Normal case (no overflow) - just update the last seen time
        // This fast path avoids the critical section in most calls
        LAST_CLOCK_TIME.store(current_time, Ordering::Relaxed);
    }
    
    // Combine upper and lower bits to form the 64-bit timestamp
    // Upper 32 bits track number of overflows
    // Lower 32 bits are the current hardware timer value
    (CLOCK_TIME_UPPER.load(Ordering::Relaxed) as u64) << 32 | current_time as u64
}

#[cfg(test)]
mod tests {
    use super::*;
    use mry::{self};
    use crate::sdk::mcu::clock::mock_clock_time;

    // Helper function to reset static variables for testing without using unsafe
    fn reset_static_vars() {
        CLOCK_TIME_UPPER.store(0, Ordering::Relaxed);
        LAST_CLOCK_TIME.store(0, Ordering::Relaxed);
    }

    #[test]
    #[mry::lock(clock_time64)]
    fn test_driver_now_returns_clock_time64() {
        // Arrange
        let driver = MyDriver {};
        let expected_time = 0x1234567890ABCDEF;
        
        // Mock clock_time64 with the correct format
        mock_clock_time64().returns(expected_time);
        
        // Act
        let result = driver.now();
        
        // Assert
        assert_eq!(result, expected_time);
        mock_clock_time64().assert_called(1);
    }

    #[test]
    fn test_driver_allocate_alarm() {
        // Arrange
        let driver = MyDriver {};
        
        // Act
        let result = unsafe { driver.allocate_alarm() };
        
        // Assert
        assert!(result.is_some());
        assert_eq!(result.unwrap().id(), 0);
    }

    #[test]
    fn test_driver_set_alarm_callback() {
        // Arrange
        let driver = MyDriver {};
        let alarm_handle = unsafe { AlarmHandle::new(0) };
        let callback: fn(*mut ()) = |_| {};
        let context: *mut () = core::ptr::null_mut();
        
        // Act & Assert
        // The function is a no-op, so we just verify it doesn't panic
        driver.set_alarm_callback(alarm_handle, callback, context);
    }

    #[test]
    #[mry::lock(clock_time)]
    #[mry::lock(clock_time64)]
    fn test_driver_set_alarm_future_timestamp() {
        // Arrange
        let driver = MyDriver {};
        let alarm_handle = unsafe { AlarmHandle::new(0) };
        
        // Need to mock both clock_time and clock_time64 to ensure consistent behavior
        mock_clock_time().returns(1000);
        mock_clock_time64().returns(1000); // Ensure now() returns this value
        
        // Act
        let result = driver.set_alarm(alarm_handle, 2000);
        
        // Assert
        assert!(result);
        mock_clock_time64().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_driver_set_alarm_past_timestamp() {
        // Arrange
        let driver = MyDriver {};
        let alarm_handle = unsafe { AlarmHandle::new(0) };
        
        // Mock clock_time with the correct format
        mock_clock_time().returns(2000);
        
        // Act - Set an alarm for the past
        let result = driver.set_alarm(alarm_handle, 1000);
        
        // Assert - Should return false for past timestamps
        assert!(!result);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_no_overflow() {
        // Arrange
        reset_static_vars();
        
        // Set initial value 
        let initial_value = 1000;
        LAST_CLOCK_TIME.store(initial_value, Ordering::Relaxed);
        
        // Simulate non-overflow case
        let next_value = initial_value + 100;
        
        // Mock clock_time with the correct format
        mock_clock_time().returns(next_value);
        
        // Act
        let result = clock_time64();
        
        // Assert
        assert_eq!(result, next_value as u64);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 0);
        assert_eq!(LAST_CLOCK_TIME.load(Ordering::Relaxed), next_value);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_with_overflow() {
        // Arrange
        reset_static_vars();
        
        // Set up overflow scenario
        let initial_value = 0xFFFFFF00_u32;
        LAST_CLOCK_TIME.store(initial_value, Ordering::Relaxed);
        
        // Simulate overflow: next value will be less than previous
        let next_value = 100_u32;
        
        // Mock clock_time with the correct format
        mock_clock_time().returns(next_value);
        
        // Act
        let result = clock_time64();
        
        // Assert
        // The upper 32 bits should be incremented to 1, and lower 32 bits should be our next value
        assert_eq!(result, (1_u64 << 32) | next_value as u64);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 1);
        assert_eq!(LAST_CLOCK_TIME.load(Ordering::Relaxed), next_value);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_normal_increment() {
        // Arrange
        reset_static_vars();
        LAST_CLOCK_TIME.store(1000, Ordering::Relaxed);
        mock_clock_time().returns(1500);
        
        // Act
        let result = clock_time64();
        
        // Assert
        assert_eq!(result, 1500);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 0);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_first_overflow() {
        // Arrange
        reset_static_vars();
        LAST_CLOCK_TIME.store(0xFFFFFF00, Ordering::Relaxed);
        mock_clock_time().returns(100);
        
        // Act
        let result = clock_time64();
        
        // Assert
        assert_eq!(result, (1_u64 << 32) | 100);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 1);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_after_overflow() {
        // Arrange
        reset_static_vars();
        LAST_CLOCK_TIME.store(100, Ordering::Relaxed);
        CLOCK_TIME_UPPER.store(1, Ordering::Relaxed);
        mock_clock_time().returns(200);
        
        // Act
        let result = clock_time64();
        
        // Assert
        assert_eq!(result, (1_u64 << 32) | 200);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 1);
        mock_clock_time().assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time)]
    fn test_clock_time64_second_overflow() {
        // Arrange
        reset_static_vars();
        LAST_CLOCK_TIME.store(0xFFFFFF00, Ordering::Relaxed);
        CLOCK_TIME_UPPER.store(1, Ordering::Relaxed);
        mock_clock_time().returns(300);
        
        // Act
        let result = clock_time64();
        
        // Assert
        assert_eq!(result, (2_u64 << 32) | 300);
        assert_eq!(CLOCK_TIME_UPPER.load(Ordering::Relaxed), 2);
        mock_clock_time().assert_called(1);
    }
}
