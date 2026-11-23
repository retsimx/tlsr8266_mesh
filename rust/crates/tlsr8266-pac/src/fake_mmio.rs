//! In-process register bank used by the `read_reg*` / `write_reg*` accessors on
//! host builds (`cfg(not(target_os = "none"))`) instead of touching silicon.
//!
//! Storage is keyed **per byte** (`addr | crate::REG_BASE_ADDR`), so a 32-bit
//! register and the 8-bit view of one of its bytes are literally the same byte
//! and overlapping silicon registers alias for free. A `BTreeMap` instead of a
//! fixed-size array is what removes the address-bounds panic class: arbitrary
//! addresses such as `0x1234` or `0x808004` are just sparse keys.
//!
//! All state lives behind one `Mutex`, and every lock recovers from poisoning,
//! so a single panicking test cannot break the rest of a `--test-threads=1` run.

use crate::REG_BASE_ADDR;
use std::collections::BTreeMap;
use std::sync::{Mutex, MutexGuard};
use std::vec::Vec;

type Hook8 = fn(usize) -> u8;
type Hook16 = fn(usize) -> u16;
type Hook32 = fn(usize) -> u32;

/// One logged write: normalized address, access width, zero-extended value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RegWrite {
    /// Normalized address, i.e. `addr | REG_BASE_ADDR`.
    pub addr: u32,
    /// Access width in bytes: 1, 2 or 4.
    pub width: u8,
    /// Written value, zero-extended to 32 bits.
    pub value: u32,
}

struct FakeMmioState {
    regs: BTreeMap<u32, u8>,
    writes: Vec<RegWrite>,
    read_counts: BTreeMap<u32, usize>,
    hooks8: BTreeMap<u32, Hook8>,
    hooks16: BTreeMap<u32, Hook16>,
    hooks32: BTreeMap<u32, Hook32>,
}

impl FakeMmioState {
    const fn new() -> Self {
        Self {
            regs: BTreeMap::new(),
            writes: Vec::new(),
            read_counts: BTreeMap::new(),
            hooks8: BTreeMap::new(),
            hooks16: BTreeMap::new(),
            hooks32: BTreeMap::new(),
        }
    }
}

static STATE: Mutex<FakeMmioState> = Mutex::new(FakeMmioState::new());

/// Lock the bank, recovering the inner state if an earlier test panicked
/// while holding the lock.
fn lock() -> MutexGuard<'static, FakeMmioState> {
    STATE.lock().unwrap_or_else(|e| e.into_inner())
}

/// Normalize a datasheet offset into the memory-mapped window. `|` (not `+`)
/// keeps every documented user of `read_reg*`/`write_reg*` — including the
/// already-normalized `0x808004` form — on a single key.
fn norm(addr: u32) -> u32 {
    addr | REG_BASE_ADDR
}

/// Unwritten bytes read as 0, like zeroed SRAM-backed registers.
fn read_byte(state: &FakeMmioState, addr: u32) -> u8 {
    state.regs.get(&addr).copied().unwrap_or(0)
}

fn write_byte(state: &mut FakeMmioState, addr: u32, v: u8) {
    state.regs.insert(addr, v);
}

/// 8-bit read used by `read_reg8`. Runs a registered hook if there is one.
pub fn read_u8(addr: u32) -> u8 {
    let a = norm(addr);
    let (hook, call_idx, stored) = {
        let mut state = lock();
        let call_idx = *state.read_counts.get(&a).unwrap_or(&0);
        state.read_counts.insert(a, call_idx + 1);
        let hook = state.hooks8.get(&a).copied();
        (hook, call_idx, read_byte(&state, a))
    };
    // The guard is dropped above, so a hook may touch the bank re-entrantly.
    match hook {
        Some(hook) => hook(call_idx),
        None => stored,
    }
}

/// 16-bit little-endian read used by `read_reg16`.
pub fn read_u16(addr: u32) -> u16 {
    let a = norm(addr);
    let (hook, call_idx, stored) = {
        let mut state = lock();
        let call_idx = *state.read_counts.get(&a).unwrap_or(&0);
        state.read_counts.insert(a, call_idx + 1);
        let hook = state.hooks16.get(&a).copied();
        let stored = u16::from_le_bytes([read_byte(&state, a), read_byte(&state, a + 1)]);
        (hook, call_idx, stored)
    };
    match hook {
        Some(hook) => hook(call_idx),
        None => stored,
    }
}

/// 32-bit little-endian read used by `read_reg32`.
pub fn read_u32(addr: u32) -> u32 {
    let a = norm(addr);
    let (hook, call_idx, stored) = {
        let mut state = lock();
        let call_idx = *state.read_counts.get(&a).unwrap_or(&0);
        state.read_counts.insert(a, call_idx + 1);
        let hook = state.hooks32.get(&a).copied();
        let stored = u32::from_le_bytes([
            read_byte(&state, a),
            read_byte(&state, a + 1),
            read_byte(&state, a + 2),
            read_byte(&state, a + 3),
        ]);
        (hook, call_idx, stored)
    };
    match hook {
        Some(hook) => hook(call_idx),
        None => stored,
    }
}

/// 8-bit write used by `write_reg8`. Logs exactly one `RegWrite`.
pub fn write_u8(addr: u32, v: u8) {
    let a = norm(addr);
    let mut state = lock();
    write_byte(&mut state, a, v);
    state.writes.push(RegWrite {
        addr: a,
        width: 1,
        value: v as u32,
    });
}

/// 16-bit little-endian write used by `write_reg16`. Logs exactly one `RegWrite`.
pub fn write_u16(addr: u32, v: u16) {
    let a = norm(addr);
    let mut state = lock();
    for (i, byte) in v.to_le_bytes().iter().enumerate() {
        write_byte(&mut state, a + i as u32, *byte);
    }
    state.writes.push(RegWrite {
        addr: a,
        width: 2,
        value: v as u32,
    });
}

/// 32-bit little-endian write used by `write_reg32`. Logs exactly one `RegWrite`.
pub fn write_u32(addr: u32, v: u32) {
    let a = norm(addr);
    let mut state = lock();
    for (i, byte) in v.to_le_bytes().iter().enumerate() {
        write_byte(&mut state, a + i as u32, *byte);
    }
    state.writes.push(RegWrite {
        addr: a,
        width: 4,
        value: v,
    });
}

/// Handle to the global fake register bank. The bank is process-wide, so
/// these are plain functions grouped under a name for readable call sites.
pub struct FakeMmio;

impl FakeMmio {
    /// Clear the byte map, the write log, the read counters and all hooks.
    pub fn reset() {
        let mut state = lock();
        state.regs.clear();
        state.writes.clear();
        state.read_counts.clear();
        state.hooks8.clear();
        state.hooks16.clear();
        state.hooks32.clear();
    }

    /// Seed a byte directly. Seeding is test setup, not a register access, so
    /// it neither appends to the write log nor consults a read hook.
    pub fn set_reg8(addr: u32, v: u8) {
        let mut state = lock();
        write_byte(&mut state, norm(addr), v);
    }

    /// Seed two little-endian bytes directly (no write-log entry).
    pub fn set_reg16(addr: u32, v: u16) {
        let a = norm(addr);
        let mut state = lock();
        for (i, byte) in v.to_le_bytes().iter().enumerate() {
            write_byte(&mut state, a + i as u32, *byte);
        }
    }

    /// Seed four little-endian bytes directly (no write-log entry).
    pub fn set_reg32(addr: u32, v: u32) {
        let a = norm(addr);
        let mut state = lock();
        for (i, byte) in v.to_le_bytes().iter().enumerate() {
            write_byte(&mut state, a + i as u32, *byte);
        }
    }

    /// Peek one byte of the bank. Deliberately bypasses read hooks and the
    /// read counters: this is the raw inspection API, so a read hook may
    /// call it from inside itself without recursing or deadlocking.
    pub fn reg8(addr: u32) -> u8 {
        let state = lock();
        read_byte(&state, norm(addr))
    }

    /// Peek two little-endian bytes of the bank (no hooks, no counters).
    pub fn reg16(addr: u32) -> u16 {
        let a = norm(addr);
        let state = lock();
        u16::from_le_bytes([read_byte(&state, a), read_byte(&state, a + 1)])
    }

    /// Peek four little-endian bytes of the bank (no hooks, no counters).
    pub fn reg32(addr: u32) -> u32 {
        let a = norm(addr);
        let state = lock();
        u32::from_le_bytes([
            read_byte(&state, a),
            read_byte(&state, a + 1),
            read_byte(&state, a + 2),
            read_byte(&state, a + 3),
        ])
    }

    /// The ordered write log (`write_reg{8,16,32}` calls, oldest first).
    pub fn writes() -> Vec<RegWrite> {
        lock().writes.clone()
    }

    /// Drop the write-log entries, keeping the byte map intact.
    pub fn clear_writes() {
        lock().writes.clear();
    }

    /// Script 8-bit reads of `addr`: the hook receives the per-address call
    /// index (0 on the first read, 1 on the second, ...).
    pub fn on_read8(addr: u32, hook: fn(usize) -> u8) {
        lock().hooks8.insert(norm(addr), hook);
    }

    /// Script 16-bit reads of `addr`; see [`FakeMmio::on_read8`].
    pub fn on_read16(addr: u32, hook: fn(usize) -> u16) {
        lock().hooks16.insert(norm(addr), hook);
    }

    /// Script 32-bit reads of `addr`; see [`FakeMmio::on_read8`].
    pub fn on_read32(addr: u32, hook: fn(usize) -> u32) {
        lock().hooks32.insert(norm(addr), hook);
    }

    /// Panic while holding the bank lock, so the next access has to recover
    /// from a poisoned mutex. Exists only to prove the poisoning policy.
    #[doc(hidden)]
    pub fn poison_bank_for_test() {
        let _guard = lock();
        panic!("intentional panic while holding the FakeMmio lock");
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::{FakeMmio, RegWrite};
    use crate::{read_reg16, read_reg32, read_reg8, write_reg16, write_reg32, write_reg8};

    #[test]
    fn test_fake_mmio_byte_aliasing_32_and_8() {
        FakeMmio::reset();

        FakeMmio::set_reg32(0x620, 0xAABB_CCDD);
        assert_eq!(FakeMmio::reg8(0x620), 0xDD);
        assert_eq!(FakeMmio::reg8(0x621), 0xCC);
        assert_eq!(FakeMmio::reg8(0x622), 0xBB);
        assert_eq!(FakeMmio::reg8(0x623), 0xAA);
        assert_eq!(FakeMmio::reg16(0x620), 0xCCDD);
        assert_eq!(FakeMmio::reg32(0x620), 0xAABB_CCDD);

        // An 8-bit write into the top byte shows up in the 32-bit view.
        FakeMmio::set_reg8(0x623, 0x11);
        assert_eq!(FakeMmio::reg32(0x620) & 0xFF00_0000, 0x1100_0000);
        assert_eq!(FakeMmio::reg32(0x620), 0x11BB_CCDD);

        // A 16-bit write is visible through both neighbours.
        FakeMmio::set_reg16(0x620, 0x1234);
        assert_eq!(FakeMmio::reg8(0x620), 0x34);
        assert_eq!(FakeMmio::reg8(0x621), 0x12);

        // The same aliasing holds through the real accessors.
        FakeMmio::reset();
        write_reg32(0x620, 0xAABB_CCDD);
        assert_eq!(read_reg8(0x620), 0xDD);
        assert_eq!(read_reg8(0x623), 0xAA);
        write_reg8(0x623, 0x11);
        assert_eq!(read_reg32(0x620), 0x11BB_CCDD);
    }

    #[test]
    fn test_fake_mmio_high_unaligned_address() {
        FakeMmio::reset();

        FakeMmio::set_reg8(0x1234, 0x5A);
        assert_eq!(FakeMmio::reg8(0x1234), 0x5A);
        assert_eq!(read_reg8(0x1234), 0x5A);

        // Command-table addresses are already normalized, so `addr | REG_BASE_ADDR`
        // must leave them alone rather than shifting them into a new key.
        FakeMmio::set_reg32(0x808004, 0x0102_0304);
        assert_eq!(FakeMmio::reg8(0x808007), 0x01);
        assert_eq!(read_reg32(0x808004), 0x0102_0304);
        assert_eq!(FakeMmio::reg32(0x800000 | 0x808004), 0x0102_0304);
    }

    #[test]
    fn test_fake_mmio_write_log() {
        FakeMmio::reset();

        write_reg8(0x620, 1);
        write_reg16(0x622, 0x1234);
        write_reg32(0x630, 0xDEAD_BEEF);

        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: 0x800620,
                    width: 1,
                    value: 0x0000_0001,
                },
                RegWrite {
                    addr: 0x800622,
                    width: 2,
                    value: 0x0000_1234,
                },
                RegWrite {
                    addr: 0x800630,
                    width: 4,
                    value: 0xDEAD_BEEF,
                },
            ]
        );

        FakeMmio::clear_writes();
        assert!(FakeMmio::writes().is_empty());
        // Clearing the log does not disturb the register contents.
        assert_eq!(FakeMmio::reg32(0x630), 0xDEAD_BEEF);
        assert_eq!(FakeMmio::reg16(0x622), 0x1234);
    }

    #[test]
    fn test_fake_mmio_reentrant_read_hook() {
        FakeMmio::reset();
        FakeMmio::on_read8(0x700, |call_idx| {
            // The hook runs with no lock held, so peeking at the bank is safe.
            let _ = FakeMmio::reg8(0x700);
            call_idx as u8
        });

        assert_eq!(read_reg8(0x700), 0);
        assert_eq!(read_reg8(0x700), 1);
        assert_eq!(super::read_u8(0x700), 2);
    }

    #[test]
    fn test_fake_mmio_reentrant_wide_read_hooks() {
        FakeMmio::reset();
        FakeMmio::set_reg16(0x710, 0x1234);
        FakeMmio::set_reg32(0x720, 0xDEAD_BEEF);

        // Both hooks re-enter the bank before returning a value derived from the
        // call index, so a hook invoked while the bank lock were still held would
        // deadlock here instead of returning.
        FakeMmio::on_read16(0x710, |call_idx| {
            let _ = FakeMmio::reg16(0x720);
            let _ = FakeMmio::reg8(0x710);
            0xAA00 | call_idx as u16
        });
        FakeMmio::on_read32(0x720, |call_idx| {
            let _ = FakeMmio::reg32(0x720);
            let _ = FakeMmio::reg16(0x710);
            0xBB00_0000 | call_idx as u32
        });

        // The hook wins over the seeded bytes and advances once per read...
        assert_eq!(read_reg16(0x710), 0xAA00);
        assert_eq!(read_reg16(0x710), 0xAA01);
        assert_eq!(read_reg32(0x720), 0xBB00_0000);
        assert_eq!(read_reg32(0x720), 0xBB00_0001);
        // ...while the bank itself keeps the seeded bytes untouched.
        assert_eq!(FakeMmio::reg16(0x710), 0x1234);
        assert_eq!(FakeMmio::reg32(0x720), 0xDEAD_BEEF);

        // `reset` drops the hooks, so the stored bytes are observed again.
        FakeMmio::reset();
        FakeMmio::set_reg16(0x710, 0x1234);
        FakeMmio::set_reg32(0x720, 0xDEAD_BEEF);
        assert_eq!(read_reg16(0x710), 0x1234);
        assert_eq!(read_reg32(0x720), 0xDEAD_BEEF);
    }

    #[test]
    fn test_fake_mmio_poison_recovery() {
        FakeMmio::reset();
        FakeMmio::set_reg8(0x40, 0x5A);

        let panicked = std::panic::catch_unwind(FakeMmio::poison_bank_for_test).is_err();
        assert!(
            panicked,
            "the poisoning helper must panic while holding the lock"
        );

        // std now considers the mutex poisoned; the accessors recover the inner
        // state instead of unwrapping the poisoning error.
        FakeMmio::set_reg8(0x41, 0xA5);
        assert_eq!(FakeMmio::reg8(0x40), 0x5A);
        assert_eq!(FakeMmio::reg8(0x41), 0xA5);
        assert_eq!(read_reg8(0x40), 0x5A);
    }
}
