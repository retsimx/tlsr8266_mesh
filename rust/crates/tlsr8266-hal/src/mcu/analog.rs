//! Analog register interface for TLSR8266 SoC.
//!
//! This module provides low-level functions for reading from and writing to
//! analog control registers on the Telink TLSR8266F512 SoC. These operations
//! are critical for configuring various hardware components including
//! RF transceiver parameters, power management, and clock settings.

use tlsr8266_pac::{
    read_reg_ana_ctrl, read_reg_ana_data, write_reg_ana_addr, write_reg_ana_ctrl,
    write_reg_ana_data, FLD_ANA,
};

/// Maximum number of busy-flag polls [`analog_wait`] performs before giving up.
pub const ANALOG_WAIT_MAX_ITERS: u32 = 10_000;

/// Waits until analog register operations are complete.
///
/// # Details
///
/// This function polls the analog control register's busy flag until it
/// clears, indicating that any ongoing analog register access operation
/// has completed. This is a blocking operation and must be called between
/// analog register accesses to ensure proper timing.
///
/// # Returns
///
/// * `true` if the busy flag cleared within [`ANALOG_WAIT_MAX_ITERS`] polls
/// * `false` if the poll budget ran out, i.e. the analog bus is stuck
///
/// # Notes
///
/// * This function is marked inline(always) for performance since it's
///   frequently called in time-critical sections
/// * The budget bounds the spin: a stuck bus costs at most the poll budget
///   instead of hanging the firmware
#[inline(always)]
pub fn analog_wait() -> bool {
    // Poll the busy bit in the analog control register until it clears.
    for _ in 0..ANALOG_WAIT_MAX_ITERS {
        if read_reg_ana_ctrl() & (FLD_ANA::BUSY.bits()) == 0 {
            return true;
        }
    }
    false
}

/// Runs the SoC's read sequence on the `0xb8`/`0xb9`/`0xba` analog bus and
/// returns the byte the data register presented.
///
/// The sequence itself is unchanged from the pre-extraction code; it is only
/// factored out so the device path and the host `MmioRegisters` fake drive the
/// bus identically. `inline(always)` keeps it inside `analog_read`'s
/// `.ram_code` section on device — a flash-resident copy would change the
/// flash-access timing this sequence must not depend on.
///
/// If the busy flag never clears, the operation is aborted (control cleared)
/// and the byte reads as `0`.
#[inline(always)]
fn read_registers(addr: u8) -> u8 {
    // 1. Set the address register to the target analog register
    write_reg_ana_addr(addr);

    // 2. Write control register to start read operation
    //    START bit initiates the operation
    //    RSV bit must be set during reads
    write_reg_ana_ctrl((FLD_ANA::START | FLD_ANA::RSV).bits());
    //   Can't use one line setting "reg_ana_ctrl32 = ((FLD_ANA_START | FLD_ANA_RSV) << 16) | addr;"
    //   This will fail because of time sequence and more over size is bigger

    // 3. Wait for the analog operation to complete. A stuck bus reports `false`
    //    and the caller below returns the zero byte instead of hanging.
    let completed = analog_wait();

    // 4. Read the data from the data register
    let data = if completed { read_reg_ana_data() } else { 0 };

    // 5. Clear control register to finish operation (also the abort path)
    write_reg_ana_ctrl(0); // finish

    data
}

/// Runs the SoC's write sequence on the `0xb8`/`0xb9`/`0xba` analog bus.
///
/// See [`read_registers`] for why this is `inline(always)`.
#[inline(always)]
fn write_registers(addr: u8, v: u8) {
    // 1. Set the address register to the target analog register
    write_reg_ana_addr(addr);

    // 2. Write the data to the data register
    write_reg_ana_data(v);

    // 3. Write control register to start write operation
    //    START bit initiates the operation
    //    RW bit must be set during writes (1=write, 0=read)
    write_reg_ana_ctrl((FLD_ANA::START | FLD_ANA::RW).bits());

    //	 Can't use one line setting "reg_ana_ctrl32 = ((FLD_ANA_START | FLD_ANA_RW) << 16) | (v << 8) | addr;"
    //   This will fail because of time sequence and more over size is bigger

    // 4. Wait for the analog operation to complete. A stuck bus cannot be
    //    retried here, so the control clear below still terminates it.
    let _ = analog_wait();

    // 5. Clear control register to finish operation
    write_reg_ana_ctrl(0); // finish
}

/// Reads a byte from the specified analog register address.
///
/// # Parameters
///
/// * `addr` - The 8-bit address of the analog register to read
///
/// # Returns
///
/// * The 8-bit value read from the specified analog register
///
/// # Notes
///
/// * This function is placed in RAM (.ram_code section) for reliable execution
///   regardless of flash access timing
/// * Uses critical_section to ensure atomicity of the complete read operation
/// * The analog read sequence follows strict timing requirements of the hardware
///   and cannot be simplified without risking unreliable operation
/// * Host builds dispatch through [`fake_analog`]: `InMemory` (default) serves
///   the read from the 256-byte bank with no MMIO and no busy polling, while
///   `MmioRegisters` runs the real bus sequence above against `FakeMmio`
#[inline(never)]
#[link_section = ".ram_code"]
pub fn analog_read(addr: u8) -> u8 {
    #[cfg(not(target_os = "none"))]
    {
        fake_analog::read(addr)
    }
    #[cfg(target_os = "none")]
    {
        critical_section::with(|_| read_registers(addr))
    }
}

/// Writes a byte to the specified analog register address.
///
/// # Parameters
///
/// * `addr` - The 8-bit address of the analog register to write
/// * `v` - The 8-bit value to write to the register
///
/// # Notes
///
/// * This function is placed in RAM (.ram_code section) for reliable execution
///   regardless of flash access timing
/// * Uses critical_section to ensure atomicity of the complete write operation
/// * The analog write sequence follows strict timing requirements of the hardware
///   and cannot be simplified without risking unreliable operation
/// * The TLSR8266's analog registers control critical hardware functions including
///   RF parameters, power management, and clock settings
/// * Host builds dispatch through [`fake_analog`] exactly like [`analog_read`]
#[inline(never)]
#[link_section = ".ram_code"]
pub fn analog_write(addr: u8, v: u8) {
    #[cfg(not(target_os = "none"))]
    {
        fake_analog::write(addr, v)
    }
    #[cfg(target_os = "none")]
    {
        critical_section::with(|_| write_registers(addr, v));
    }
}

/// Host-side analog register stand-in (`cfg(not(target_os = "none"))`).
///
/// The SoC multiplexes 256 analog addresses through three MMIO registers
/// (`0xb8` address, `0xb9` data, `0xba` control), so a flat `FakeMmio` byte map
/// can neither hold two analog addresses at once nor replay an ordered torn
/// read. This seam therefore owns a 256-byte bank plus per-address read queues,
/// and offers `MmioRegisters` when a test is about the bus protocol itself.
#[cfg(not(target_os = "none"))]
pub mod fake_analog {
    use std::collections::{BTreeMap, VecDeque};
    use std::sync::{Mutex, MutexGuard};

    /// How host [`super::analog_read`] / [`super::analog_write`] reach the
    /// analog plane.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum FakeAnalogMode {
        /// 256-byte in-memory analog register bank (default on host).
        InMemory,
        /// Drives the real 0xb8/0xb9/0xba MMIO bus sequence against FakeMmio.
        MmioRegisters,
    }

    struct State {
        bank: [u8; 256],
        reads: BTreeMap<u8, VecDeque<u8>>,
        mode: FakeAnalogMode,
    }

    static STATE: Mutex<State> = Mutex::new(State {
        bank: [0u8; 256],
        reads: BTreeMap::new(),
        mode: FakeAnalogMode::InMemory,
    });

    /// Recover the inner state from a poisoned lock, so one panicking test
    /// cannot poison the rest of a `--test-threads=1` run (the `FakeMmio`
    /// convention).
    fn lock() -> MutexGuard<'static, State> {
        STATE.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// The 256-address analog register bank. Process-wide, like `FakeMmio`, so
    /// these are plain functions grouped under a name for readable call sites.
    pub struct FakeAnalog;

    impl FakeAnalog {
        /// Zero the bank, drop every queued read and restore `InMemory`.
        pub fn reset() {
            let mut state = lock();
            state.bank = [0u8; 256];
            state.reads.clear();
            state.mode = FakeAnalogMode::InMemory;
        }

        /// Select the host transport.
        pub fn set_mode(mode: FakeAnalogMode) {
            lock().mode = mode;
        }

        /// Pops a queued read for `addr` first, else returns the bank byte.
        ///
        /// Popping is what reproduces a torn high byte: queue `hi1` and `hi2`
        /// for `ANA_32K_COUNTER_HI`, and consecutive reads observe each in turn
        /// with the `LO` read in between.
        pub fn get(addr: u8) -> u8 {
            let mut state = lock();
            let queued = match state.reads.get_mut(&addr) {
                Some(queue) => queue.pop_front(),
                None => None,
            };
            queued.unwrap_or(state.bank[addr as usize])
        }

        /// Store a byte in the bank. Does not touch any queue.
        pub fn set(addr: u8, val: u8) {
            lock().bank[addr as usize] = val;
        }

        /// Queue reads for `addr`, FIFO: `values[0]` is returned by the next
        /// read of `addr`, `values[1]` by the one after, and so on.
        pub fn push_reads(addr: u8, values: &[u8]) {
            lock()
                .reads
                .entry(addr)
                .or_default()
                .extend(values.iter().copied());
        }
    }

    fn mode() -> FakeAnalogMode {
        lock().mode
    }

    /// Host `analog_read` body: the bank, or the real bus sequence.
    pub(crate) fn read(addr: u8) -> u8 {
        match mode() {
            FakeAnalogMode::InMemory => FakeAnalog::get(addr),
            FakeAnalogMode::MmioRegisters => super::read_registers(addr),
        }
    }

    /// Host `analog_write` body: the bank, or the real bus sequence.
    pub(crate) fn write(addr: u8, val: u8) {
        match mode() {
            FakeAnalogMode::InMemory => FakeAnalog::set(addr, val),
            FakeAnalogMode::MmioRegisters => super::write_registers(addr, val),
        }
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::mcu::analog::fake_analog::{FakeAnalog, FakeAnalogMode};
    use std::sync::atomic::{AtomicUsize, Ordering};
    use tlsr8266_pac::fake_mmio::{FakeMmio, RegWrite};
    use tlsr8266_pac::REG_BASE_ADDR;

    /// The three registers of the analog bus.
    const ANA_ADDR: u32 = 0xb8;
    const ANA_DATA: u32 = 0xb9;
    const ANA_CTRL: u32 = 0xba;

    /// `FakeMmio` and `FakeAnalog` are process-wide and the suite runs with
    /// `--test-threads=1`, so every test starts from zero and from `InMemory`.
    fn reset_fakes() {
        FakeMmio::reset();
        FakeAnalog::reset();
    }

    /// Values written to `addr`, oldest first, in the order the analog bus would
    /// have seen them.
    fn values_written_to(addr: u32) -> Vec<u32> {
        FakeMmio::writes()
            .into_iter()
            .filter(|write| write.addr == addr | REG_BASE_ADDR)
            .map(|write| write.value)
            .collect()
    }

    /// Tests analog_wait function for proper busy bit polling behavior.
    ///
    /// This test verifies that the analog_wait function correctly:
    /// - Polls the analog control register's busy flag
    /// - Exits when the busy flag clears
    /// - Reports completion to the caller
    ///
    /// # Algorithm
    ///
    /// 1. Script `reg_ana_ctrl` to report busy on its first two reads, then clear
    /// 2. Call analog_wait function
    /// 3. Read the control register once more and verify the busy flag is clear
    ///
    /// # Notes
    ///
    /// * The trailing read is the poll-count proof: exiting early would leave the
    ///   second scripted (busy) sample in place and fail the assertion
    #[test]
    fn test_analog_wait() {
        reset_fakes();

        // Busy for the first two polls, then idle
        FakeMmio::on_read8(ANA_CTRL, |call_idx| {
            if call_idx < 2 {
                FLD_ANA::BUSY.bits()
            } else {
                0
            }
        });

        // Call the function being tested
        assert!(analog_wait(), "a clearing busy flag must report success");

        // Verify the function polled past every busy sample
        assert_eq!(
            read_reg_ana_ctrl(),
            0,
            "analog_wait must poll until the busy flag clears"
        );
    }

    /// Reads of `reg_ana_ctrl` observed by [`always_busy_analog`].
    static ANA_CTRL_POLLS: AtomicUsize = AtomicUsize::new(0);

    /// Reports an analog bus that never finishes its operation.
    fn always_busy_analog(_call_idx: usize) -> u8 {
        ANA_CTRL_POLLS.fetch_add(1, Ordering::Relaxed);
        FLD_ANA::BUSY.bits()
    }

    /// A stuck busy flag exhausts the poll budget instead of spinning forever.
    ///
    /// The poll count is the proof: exactly one read per budgeted iteration,
    /// and the function reports failure instead of hanging.
    #[test]
    fn test_analog_wait_times_out() {
        reset_fakes();
        ANA_CTRL_POLLS.store(0, Ordering::Relaxed);
        FakeMmio::on_read8(ANA_CTRL, always_busy_analog);

        assert!(
            !analog_wait(),
            "a busy flag that never clears must report a timeout"
        );
        assert_eq!(
            ANA_CTRL_POLLS.load(Ordering::Relaxed),
            ANALOG_WAIT_MAX_ITERS as usize,
            "the busy wait must stop after ANALOG_WAIT_MAX_ITERS polls"
        );
    }

    /// A stuck analog bus makes `analog_read` return `0` and still terminate the
    /// operation by clearing the control register.
    #[test]
    fn test_analog_read_stuck_bus_returns_zero() {
        reset_fakes();
        FakeAnalog::set_mode(FakeAnalogMode::MmioRegisters);
        ANA_CTRL_POLLS.store(0, Ordering::Relaxed);
        FakeMmio::on_read8(ANA_CTRL, always_busy_analog);
        // The data register must not be read: the operation never completed.
        FakeMmio::set_reg8(ANA_DATA, 0xAB);

        assert_eq!(analog_read(0x42), 0, "a timed-out read must return zero");

        // The address and the start command reached the bus, and the last write
        // cleared the control register to finish the aborted operation.
        assert_eq!(
            values_written_to(ANA_CTRL),
            vec![u32::from((FLD_ANA::START | FLD_ANA::RSV).bits()), 0],
            "a timed-out read must still clear the control register"
        );
        assert_eq!(
            FakeMmio::reg8(ANA_CTRL),
            0,
            "the control register must end cleared"
        );
    }

    /// Tests analog_read function for proper register read sequence.
    ///
    /// This test verifies that the analog_read function correctly:
    /// - Sets the analog address register
    /// - Sets proper control bits for read operation
    /// - Waits for operation completion
    /// - Retrieves data from data register
    /// - Clears control register after operation
    /// - Returns correct data value
    ///
    /// # Algorithm
    ///
    /// 1. Select `MmioRegisters` so the real bus sequence runs against `FakeMmio`
    /// 2. Script the data register to present the expected byte
    /// 3. Call analog_read with test address
    /// 4. Verify the ordered register writes and the returned data value
    ///
    /// # Notes
    ///
    /// * Tests the complete analog register read sequence
    /// * Verifies critical timing sequence and register operations
    #[test]
    fn test_analog_read() {
        reset_fakes();
        FakeAnalog::set_mode(FakeAnalogMode::MmioRegisters);

        // Setup test data
        let test_addr: u8 = 0x42;
        let expected_data: u8 = 0xAB;
        FakeMmio::set_reg8(ANA_DATA, expected_data);

        // Call the function being tested
        let result = analog_read(test_addr);

        // Verify the address was set, the read was started and then finished
        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: ANA_ADDR | REG_BASE_ADDR,
                    width: 1,
                    value: test_addr as u32,
                },
                RegWrite {
                    addr: ANA_CTRL | REG_BASE_ADDR,
                    width: 1,
                    value: (FLD_ANA::START | FLD_ANA::RSV).bits() as u32,
                },
                RegWrite {
                    addr: ANA_CTRL | REG_BASE_ADDR,
                    width: 1,
                    value: 0,
                },
            ]
        );

        // Verify correct data was returned
        assert_eq!(result, expected_data);
    }

    /// Tests analog_write function for proper register write sequence.
    ///
    /// This test verifies that the analog_write function correctly:
    /// - Sets the analog address register
    /// - Sets the analog data register with the value to write
    /// - Sets proper control bits for write operation
    /// - Waits for operation completion
    /// - Clears control register after operation
    ///
    /// # Algorithm
    ///
    /// 1. Select `MmioRegisters` so the real bus sequence runs against `FakeMmio`
    /// 2. Call analog_write with test address and data value
    /// 3. Verify the ordered register writes
    ///
    /// # Notes
    ///
    /// * Tests the complete analog register write sequence
    /// * Checks that control bits are set correctly for write operation
    #[test]
    fn test_analog_write() {
        reset_fakes();
        FakeAnalog::set_mode(FakeAnalogMode::MmioRegisters);

        // Setup test data
        let test_addr: u8 = 0x42;
        let test_value: u8 = 0xCD;

        // Call the function being tested
        analog_write(test_addr, test_value);

        // Verify the address, data and control writes happened in bus order
        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: ANA_ADDR | REG_BASE_ADDR,
                    width: 1,
                    value: test_addr as u32,
                },
                RegWrite {
                    addr: ANA_DATA | REG_BASE_ADDR,
                    width: 1,
                    value: test_value as u32,
                },
                RegWrite {
                    addr: ANA_CTRL | REG_BASE_ADDR,
                    width: 1,
                    value: (FLD_ANA::START | FLD_ANA::RW).bits() as u32,
                },
                RegWrite {
                    addr: ANA_CTRL | REG_BASE_ADDR,
                    width: 1,
                    value: 0,
                },
            ]
        );
    }

    /// Tests analog_read function behavior through the bus sequence.
    ///
    /// This test verifies that analog_read correctly:
    /// - Runs the same read sequence as `test_analog_read` for another address
    /// - Returns the byte the data register presented
    ///
    /// # Notes
    ///
    /// * Host builds have no `critical_section` body to exercise (the device
    ///   path is `cfg(target_os = "none")`), so this asserts the address and
    ///   control write order, not atomicity
    #[test]
    fn test_analog_read_bus_write_order() {
        reset_fakes();
        FakeAnalog::set_mode(FakeAnalogMode::MmioRegisters);

        // Setup test data
        let test_addr: u8 = 0x55;
        let expected_data: u8 = 0x99;
        FakeMmio::set_reg8(ANA_DATA, expected_data);

        // Call the function being tested
        let result = analog_read(test_addr);

        // Verify correct result
        assert_eq!(result, expected_data);

        // Verify the address reached the bus before the data was read back
        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: ANA_ADDR | REG_BASE_ADDR,
                    width: 1,
                    value: test_addr as u32,
                },
                RegWrite {
                    addr: ANA_CTRL | REG_BASE_ADDR,
                    width: 1,
                    value: (FLD_ANA::START | FLD_ANA::RSV).bits() as u32,
                },
                RegWrite {
                    addr: ANA_CTRL | REG_BASE_ADDR,
                    width: 1,
                    value: 0,
                },
            ]
        );
    }

    /// Tests analog_write function behavior through the bus sequence.
    ///
    /// This test verifies that analog_write correctly:
    /// - Runs the same write sequence as `test_analog_write` for another address
    /// - Leaves the analog plane holding the written byte
    ///
    /// # Notes
    ///
    /// * Host builds have no `critical_section` body to exercise (the device
    ///   path is `cfg(target_os = "none")`), so this asserts the data and
    ///   control write order, not atomicity
    #[test]
    fn test_analog_write_bus_write_order() {
        reset_fakes();
        FakeAnalog::set_mode(FakeAnalogMode::MmioRegisters);

        // Setup test data
        let test_addr: u8 = 0x66;
        let test_value: u8 = 0x88;

        // Call the function being tested
        analog_write(test_addr, test_value);

        // Verify the data byte reached the bus
        assert_eq!(FakeMmio::reg8(ANA_DATA), test_value);
        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: ANA_ADDR | REG_BASE_ADDR,
                    width: 1,
                    value: test_addr as u32,
                },
                RegWrite {
                    addr: ANA_DATA | REG_BASE_ADDR,
                    width: 1,
                    value: test_value as u32,
                },
                RegWrite {
                    addr: ANA_CTRL | REG_BASE_ADDR,
                    width: 1,
                    value: (FLD_ANA::START | FLD_ANA::RW).bits() as u32,
                },
                RegWrite {
                    addr: ANA_CTRL | REG_BASE_ADDR,
                    width: 1,
                    value: 0,
                },
            ]
        );
    }
}
