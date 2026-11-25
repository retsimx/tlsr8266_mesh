use core::marker::PhantomData;
use core::ptr;
#[cfg(feature = "rtt")]
use core::sync::atomic::AtomicU32;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_executor::{raw, Spawner};

#[cfg(all(target_os = "none", feature = "rtt"))]
use crate::mcu::clock::CLOCK_SYS_CLOCK_1US;
#[cfg(target_os = "none")]
use crate::mcu::irq_i::{irq_disable, irq_restore};
#[cfg(target_os = "none")]
use tlsr8266_pac::{
    read_reg_irq_en, read_reg_irq_mask, write_reg_mcu_wakeup_mask, write_reg_pwdn_ctrl, FldPwdnCtrl,
};
#[cfg(all(target_os = "none", feature = "rtt"))]
use tlsr8266_pac::{
    read_reg_irq_src, read_reg_irq_src3, read_reg_rf_irq_status, read_reg_system_tick,
};

/// TC32 Executor
pub struct Executor {
    inner: raw::Executor,
    not_send: PhantomData<*mut ()>,
}

/// Set by [`__pender`] from any context (task or interrupt) when the executor has work.
static WORK_PENDING: AtomicBool = AtomicBool::new(false);

/// Host-tool contract: `probe-tlsr read --elf ... EXECUTOR_POLLS` resolves this
/// exact symbol name, so it may not be renamed.
///
/// Counts `Executor::run` loop iterations: a busy-polling executor grows this
/// without bound while `EXECUTOR_STALLS` stays flat.
#[cfg(feature = "rtt")]
#[no_mangle]
pub static EXECUTOR_POLLS: AtomicU32 = AtomicU32::new(0);

/// Host-tool contract: `probe-tlsr read --elf ... EXECUTOR_STALLS` resolves this
/// exact symbol name, so it may not be renamed.
///
/// Counts entries into the `0x6f = 0x80` CPU idle stall, i.e. iterations where
/// the executor actually parked instead of immediately re-polling.
#[cfg(feature = "rtt")]
#[no_mangle]
pub static EXECUTOR_STALLS: AtomicU32 = AtomicU32::new(0);

/// Host-tool contract: `probe-tlsr read --elf ... EXECUTOR_LOST_WAKES` resolves
/// this exact symbol name, so it may not be renamed.
///
/// Counts parks that returned with no enabled IRQ source latched: the executor
/// parked in `0x6f = 0x80` while `WORK_PENDING` was false, yet `reg_irq_src`
/// showed none of the wake sources it had enabled.
#[cfg(feature = "rtt")]
#[no_mangle]
pub static EXECUTOR_LOST_WAKES: AtomicU32 = AtomicU32::new(0);

/// One captured lost-wake park, for diagnosing [`EXECUTOR_LOST_WAKES`] on real
/// silicon.
///
/// Host-tool contract: `probe-tlsr read --elf ... EXECUTOR_PARK_RING <bytes>`
/// reads these raw, so the field order and width are part of the contract.
/// Each field is 4 bytes, little-endian, in declaration order.
#[cfg(feature = "rtt")]
#[repr(C)]
pub struct ExecutorParkSample {
    /// `reg_irq_src` sampled at the lost-wake check.
    pub irq_src: AtomicU32,
    /// `reg_irq_src3` sampled at the lost-wake check.
    pub irq_src3: AtomicU32,
    /// `reg_rf_irq_status` sampled at the lost-wake check.
    pub rf_irq_status: AtomicU32,
    /// The `wakeup_mask` the check compared against.
    pub wakeup_mask: AtomicU32,
    /// `reg_system_tick` immediately before entering the sleep.
    pub tick_before: AtomicU32,
    /// `reg_system_tick` immediately after the sleep returned.
    pub tick_after: AtomicU32,
}

#[cfg(feature = "rtt")]
impl ExecutorParkSample {
    const fn new() -> Self {
        Self {
            irq_src: AtomicU32::new(0),
            irq_src3: AtomicU32::new(0),
            rf_irq_status: AtomicU32::new(0),
            wakeup_mask: AtomicU32::new(0),
            tick_before: AtomicU32::new(0),
            tick_after: AtomicU32::new(0),
        }
    }
}

/// Number of retained [`ExecutorParkSample`] entries.
#[cfg(feature = "rtt")]
pub const EXECUTOR_PARK_RING_LEN: usize = 16;

/// Ring of the most recent lost-wake parks; the oldest entry is overwritten.
///
/// Host-tool contract: `probe-tlsr read --elf ... EXECUTOR_PARK_RING 384`
/// returns `EXECUTOR_PARK_RING_LEN` samples of 6 x u32 each.
#[cfg(feature = "rtt")]
#[no_mangle]
pub static EXECUTOR_PARK_RING: [ExecutorParkSample; EXECUTOR_PARK_RING_LEN] =
    [const { ExecutorParkSample::new() }; EXECUTOR_PARK_RING_LEN];

/// Total lost-wake parks captured into [`EXECUTOR_PARK_RING`].
///
/// Host-tool contract: `probe-tlsr read --elf ... EXECUTOR_PARK_RING_IDX`.
#[cfg(feature = "rtt")]
#[no_mangle]
pub static EXECUTOR_PARK_RING_IDX: AtomicU32 = AtomicU32::new(0);

/// Accumulated microseconds spent inside [`raw::Executor::poll`], i.e. the
/// executor's busy time.
///
/// Together with [`EXECUTOR_LOOP_US`], [`EXECUTOR_PARK_PATH_US`] and
/// [`EXECUTOR_ISR_US`] this gives the exact CPU duty cycle without perturbing
/// the target:
/// `sleep = LOOP_US - BUSY_US - PARK_PATH_US - ISR_US`.
///
/// Busy time is measured around `poll()` rather than around the sleep stall:
/// the `reg_pwdn_ctrl` write is posted, so the instructions after it run before
/// the core actually suspends. Bracketing the stall therefore measures the
/// pre-suspend window (~1 us), not the park. Bracketing `poll()` avoids that.
///
/// Accumulated in microseconds, not ticks, so a u32 lasts ~71 minutes instead
/// of ~134 seconds. Both counters start at zero on boot and must be read before
/// either wraps.
#[cfg(feature = "rtt")]
#[no_mangle]
pub static EXECUTOR_BUSY_US: AtomicU32 = AtomicU32::new(0);

/// Accumulated microseconds of executor loop time (busy + parked), the
/// denominator of the CPU-usage ratio.
///
/// Host-tool contract: `probe-tlsr read --elf ... EXECUTOR_LOOP_US`.
#[cfg(feature = "rtt")]
#[no_mangle]
pub static EXECUTOR_LOOP_US: AtomicU32 = AtomicU32::new(0);

/// Accumulated microseconds of executor bookkeeping between `poll()` and the
/// next loop iteration (irq disable/restore, wakeup-mask write, stall write,
/// settle read, `irq_src` read, checks).
///
/// This bracket spans the suspend as well, so it is **not** pure bookkeeping:
/// `PARK_PATH_US = PRE_SUSPEND_US + sleep + post-wake bookkeeping + ISR`.
///
/// Host-tool contract: `probe-tlsr read --elf ... EXECUTOR_PARK_PATH_US`.
#[cfg(feature = "rtt")]
#[no_mangle]
pub static EXECUTOR_PARK_PATH_US: AtomicU32 = AtomicU32::new(0);

/// Accumulated microseconds of executor bookkeeping from the end of `poll()`
/// up to the sleep write, i.e. the pre-suspend overhead that is certainly not
/// sleep (`irq_disable`, wakeup-mask write, stall count).
///
/// Host-tool contract: `probe-tlsr read --elf ... EXECUTOR_PARK_PRE_US`.
#[cfg(feature = "rtt")]
#[no_mangle]
pub static EXECUTOR_PARK_PRE_US: AtomicU32 = AtomicU32::new(0);

/// Accumulated microseconds spent inside the firmware's `irq_handler`.
///
/// ISR time is real CPU time that happens outside `poll()`, so the exact sleep
/// figure is
/// `sleep = EXECUTOR_LOOP_US - EXECUTOR_BUSY_US - EXECUTOR_PARK_PATH_US - EXECUTOR_ISR_US`.
///
/// Host-tool contract: `probe-tlsr read --elf ... EXECUTOR_ISR_US`.
#[cfg(feature = "rtt")]
#[no_mangle]
pub static EXECUTOR_ISR_US: AtomicU32 = AtomicU32::new(0);

/// Sample the system tick on interrupt-handler entry.
///
/// Paired with [`note_isr_ticks`]; kept in the HAL so the firmware's
/// `irq_handler` never reaches the PAC directly (the layering gate forbids it).
#[cfg(all(target_os = "none", feature = "rtt"))]
pub fn note_isr_start() -> u32 {
    read_reg_system_tick()
}

/// Record one interrupt handler's execution time, in microseconds.
///
/// Called from the firmware's `irq_handler` with the tick sampled on entry, so
/// the ISR's own cost can be subtracted from the parked residual. Assumes
/// interrupts do not nest (the TLSR8266 runs a single IRQ priority), otherwise
/// the inner handler's time would be counted twice.
#[cfg(all(target_os = "none", feature = "rtt"))]
pub fn note_isr_ticks(start: u32) {
    let end = read_reg_system_tick();
    EXECUTOR_ISR_US.store(
        EXECUTOR_ISR_US
            .load(Ordering::Relaxed)
            .wrapping_add(end.wrapping_sub(start) / CLOCK_SYS_CLOCK_1US),
        Ordering::Relaxed,
    );
}

/// `reg_system_tick` at the previous loop start, used to measure the loop
/// period.
#[cfg(feature = "rtt")]
static EXECUTOR_PREV_LOOP_TICK: AtomicU32 = AtomicU32::new(0);

/// Tells the executor that a task has been woken and needs polling.
///
/// The executor calls this from any context, so it must not poll. It only records the
/// request; [`Executor::run`] clears the flag before polling and re-checks it before
/// entering the CPU idle stall.
#[export_name = "__pender"]
fn __pender(_context: *mut ()) {
    WORK_PENDING.store(true, Ordering::Relaxed);
}

impl Executor {
    /// Create a new Executor.
    pub fn new() -> Self {
        Self {
            // use Signal_Work_Thread_Mode as substitute for local interrupt register
            inner: raw::Executor::new(ptr::null_mut()),
            not_send: PhantomData,
        }
    }

    /// Run the executor.
    ///
    /// The `init` closure is called with a [`Spawner`] that spawns tasks on
    /// this executor. Use it to spawn the initial task(s). After `init` returns,
    /// the executor starts running the tasks.
    ///
    /// To spawn more tasks later, you may keep copies of the [`Spawner`] (it is `Copy`),
    /// for example by passing it as an argument to the initial tasks.
    ///
    /// This function requires `&'static mut self`. This means you have to store the
    /// Executor instance in a place where it'll live forever and grants you mutable
    /// access. There's a few ways to do this:
    ///
    /// - a [StaticCell](https://docs.rs/static_cell/latest/static_cell/) (safe)
    /// - a `static mut` (unsafe)
    /// - a local variable in a function you know never returns (like `fn main() -> !`), upgrading its lifetime with `transmute`. (unsafe)
    ///
    /// This function never returns.
    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        init(self.inner.spawner());

        loop {
            // CPU-duty accounting: bracket `poll()` with tick reads. See
            // `EXECUTOR_BUSY_US` for why the sleep stall is not bracketed.
            #[cfg(all(target_os = "none", feature = "rtt"))]
            let loop_start = read_reg_system_tick();

            // TC32 has no read-modify-write atomics, so count with load + store.
            #[cfg(feature = "rtt")]
            EXECUTOR_POLLS.store(
                EXECUTOR_POLLS.load(Ordering::Relaxed).wrapping_add(1),
                Ordering::Relaxed,
            );

            WORK_PENDING.store(false, Ordering::Relaxed);
            // SAFETY: (3) `raw::Executor::poll` contract. `self` is the `&'static mut
            // Executor` handed to `run`, which never returns and never hands the reference
            // out again, so exactly one context polls this executor.
            unsafe {
                self.inner.poll();
            }

            #[cfg(all(target_os = "none", feature = "rtt"))]
            let poll_end = read_reg_system_tick();

            #[cfg(all(target_os = "none", feature = "rtt"))]
            {
                EXECUTOR_BUSY_US.store(
                    EXECUTOR_BUSY_US
                        .load(Ordering::Relaxed)
                        .wrapping_add(poll_end.wrapping_sub(loop_start) / CLOCK_SYS_CLOCK_1US),
                    Ordering::Relaxed,
                );
                let prev = EXECUTOR_PREV_LOOP_TICK.load(Ordering::Relaxed);
                EXECUTOR_PREV_LOOP_TICK.store(loop_start, Ordering::Relaxed);
                if prev != 0 {
                    EXECUTOR_LOOP_US.store(
                        EXECUTOR_LOOP_US
                            .load(Ordering::Relaxed)
                            .wrapping_add(loop_start.wrapping_sub(prev) / CLOCK_SYS_CLOCK_1US),
                        Ordering::Relaxed,
                    );
                }
            }

            #[cfg(target_os = "none")]
            {
                let prev = irq_disable();
                if !WORK_PENDING.load(Ordering::Relaxed) {
                    // Wake on exactly the sources we enable: `& 0x00ff_ffff` drops the
                    // `FLD_IRQ::EN` field (bits 24-31), and sourcing bit 16 from
                    // `reg_irq_mask` keeps the permanently-latched USB 250 us interrupt
                    // out of the wake mask because that mask never enables it.
                    let wakeup_mask = read_reg_irq_mask() & 0x00ff_ffff;
                    write_reg_mcu_wakeup_mask(wakeup_mask);
                    // Only the path that actually parks counts as a stall.
                    #[cfg(feature = "rtt")]
                    EXECUTOR_STALLS.store(
                        EXECUTOR_STALLS.load(Ordering::Relaxed).wrapping_add(1),
                        Ordering::Relaxed,
                    );
                    #[cfg(feature = "rtt")]
                    let tick_before = read_reg_system_tick();
                    #[cfg(all(target_os = "none", feature = "rtt"))]
                    EXECUTOR_PARK_PRE_US.store(
                        EXECUTOR_PARK_PRE_US
                            .load(Ordering::Relaxed)
                            .wrapping_add(tick_before.wrapping_sub(poll_end) / CLOCK_SYS_CLOCK_1US),
                        Ordering::Relaxed,
                    );
                    write_reg_pwdn_ctrl(FldPwdnCtrl::Sleep.bits()); // 0x80
                    #[cfg(feature = "rtt")]
                    {
                        // Settle read: the wake source's `reg_irq_src` latch is not
                        // visible on the first register access after the sleep
                        // returns. Sampling it immediately reported a false lost
                        // wake on ~19% of parks on the 12 MHz bench board; one
                        // prior register read makes the latch visible and the
                        // counter reads zero. A genuinely sourceless park still
                        // fails the check, so this does not mask a real lost wake.
                        let tick_after = read_reg_system_tick();
                        // Sample while IRQs are still masked: `WORK_PENDING` was false above,
                        // so a zero `reg_irq_src & wakeup_mask` means this park had no enabled
                        // wake source latched, i.e. a lost wake.
                        let irq_src = read_reg_irq_src();
                        if (irq_src & wakeup_mask) == 0 {
                            EXECUTOR_LOST_WAKES.store(
                                EXECUTOR_LOST_WAKES.load(Ordering::Relaxed).wrapping_add(1),
                                Ordering::Relaxed,
                            );
                            // Retain the exact state so the real wake source can be
                            // identified from the bench without re-instrumenting.
                            let slot = EXECUTOR_PARK_RING_IDX.load(Ordering::Relaxed) as usize
                                % EXECUTOR_PARK_RING_LEN;
                            let sample = &EXECUTOR_PARK_RING[slot];
                            sample.irq_src.store(irq_src, Ordering::Relaxed);
                            sample
                                .irq_src3
                                .store(u32::from(read_reg_irq_src3()), Ordering::Relaxed);
                            sample
                                .rf_irq_status
                                .store(u32::from(read_reg_rf_irq_status()), Ordering::Relaxed);
                            sample.wakeup_mask.store(wakeup_mask, Ordering::Relaxed);
                            sample.tick_before.store(tick_before, Ordering::Relaxed);
                            sample.tick_after.store(tick_after, Ordering::Relaxed);
                            EXECUTOR_PARK_RING_IDX.store(
                                EXECUTOR_PARK_RING_IDX
                                    .load(Ordering::Relaxed)
                                    .wrapping_add(1),
                                Ordering::Relaxed,
                            );
                        }
                    }
                }
                irq_restore(prev);
                // Flush bus write so any pending IRQ vectors before next iteration can re-mask.
                let _ = read_reg_irq_en();
                // Close the active-time bracket: everything from `poll_end` to
                // here is executor bookkeeping, not sleep.
                #[cfg(all(target_os = "none", feature = "rtt"))]
                {
                    let park_end = read_reg_system_tick();
                    EXECUTOR_PARK_PATH_US.store(
                        EXECUTOR_PARK_PATH_US
                            .load(Ordering::Relaxed)
                            .wrapping_add(park_end.wrapping_sub(poll_end) / CLOCK_SYS_CLOCK_1US),
                        Ordering::Relaxed,
                    );
                }
            }
        }
    }

    // todo: For when we make IRQ's async
    // pub fn init_spawner(&'static self, init: impl FnOnce(Spawner)) {
    //     init(self.inner.spawner());
    // }
    //
    // pub fn poll(&'static self) {
    //     unsafe {
    //         self.inner.poll();
    //     }
    // }
}

impl Default for Executor {
    fn default() -> Self {
        Self::new()
    }
}
