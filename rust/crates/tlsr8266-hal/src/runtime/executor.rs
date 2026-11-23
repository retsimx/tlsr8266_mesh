use core::marker::PhantomData;
use core::ptr;
#[cfg(feature = "rtt")]
use core::sync::atomic::AtomicU32;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_executor::{raw, Spawner};

#[cfg(target_os = "none")]
use crate::mcu::irq_i::{irq_disable, irq_restore};
#[cfg(target_os = "none")]
use tlsr8266_pac::{
    read_reg_irq_en, read_reg_irq_mask, write_reg_mcu_wakeup_mask, write_reg_pwdn_ctrl, FldPwdnCtrl,
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

            #[cfg(target_os = "none")]
            {
                let prev = irq_disable();
                if !WORK_PENDING.load(Ordering::Relaxed) {
                    // Wake on exactly the sources we enable: `& 0x00ff_ffff` drops the
                    // `FLD_IRQ::EN` field (bits 24-31), and sourcing bit 16 from
                    // `reg_irq_mask` keeps the permanently-latched USB 250 us interrupt
                    // out of the wake mask because that mask never enables it.
                    write_reg_mcu_wakeup_mask(read_reg_irq_mask() & 0x00ff_ffff);
                    // Only the path that actually parks counts as a stall.
                    #[cfg(feature = "rtt")]
                    EXECUTOR_STALLS.store(
                        EXECUTOR_STALLS.load(Ordering::Relaxed).wrapping_add(1),
                        Ordering::Relaxed,
                    );
                    write_reg_pwdn_ctrl(FldPwdnCtrl::Sleep.bits()); // 0x80
                }
                irq_restore(prev);
                // Flush bus write so any pending IRQ vectors before next iteration can re-mask.
                let _ = read_reg_irq_en();
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
