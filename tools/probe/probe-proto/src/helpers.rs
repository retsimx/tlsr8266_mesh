//! Pure protocol math: address validation, RTT spans, breakpoints, flash bounds.
//!
//! Every function here is total over its input domain and uses widened
//! (`u32`/`u64`) intermediates, so debug builds cannot panic on overflow. The
//! crate shares these predicates between `probe-pico` and `probe-tlsr` so both
//! ends of the wire reject the same values.

use crate::msg::ProtoError;
use crate::{FLASH_SECTOR_SIZE, FLASH_SIZE};

/// Last address of the SWire low register window (`0x0000..=0x0fff`).
const SWIRE_LOW_END: u32 = 0x0fff;
/// First address of the SWire SRAM window (`0x8000..=0xbfff`).
const SWIRE_SRAM_START: u32 = 0x8000;
/// Last address of the SWire SRAM window.
const SWIRE_SRAM_END: u32 = 0xbfff;

/// Largest target address a breakpoint compare can hold (24 bits).
const BREAKPOINT_MAX_ADDR: u32 = 0x00ff_ffff;
/// SWire register of breakpoint slot 0; slots are 4 bytes apart.
const BREAKPOINT_REG_BASE: u16 = 0x0610;
/// Bit 24 of a breakpoint register value enables the compare.
const BREAKPOINT_ENABLE: u32 = 0x0100_0000;

/// Requires the RP2040 RTT ring to be exactly 256 bytes.
const RTT_CAP: u16 = 256;

/// Returns `true` when `addr..addr + len` lies wholly inside mapped SWire
/// space (`0x0000..=0x0fff` or `0x8000..=0xbfff`).
///
/// The span is inclusive of `addr + len - 1`; a zero `len` and a span that
/// runs past `0xffff` are both rejected.
pub fn is_valid_swire_range(addr: u16, len: u16) -> bool {
    if len == 0 {
        return false;
    }
    let start = u32::from(addr);
    // `start + len - 1` cannot overflow `u32`: both terms are 16-bit.
    let end = start + u32::from(len) - 1;
    if end > u32::from(u16::MAX) {
        return false;
    }
    (start <= SWIRE_LOW_END && end <= SWIRE_LOW_END)
        || (start >= SWIRE_SRAM_START && end <= SWIRE_SRAM_END)
}

/// Splits the pending RTT bytes into up to two ring spans, returning
/// `[(ring_offset, len); 2]` (unused span is `(0, 0)`) and the new read index.
///
/// `cap` must be `256` and `wr`/`rd` must be inside the ring; anything
/// else is [`ProtoError::InvalidArg`]. Wraparound is expressed by reporting the
/// tail of the ring first and the span that starts at offset `0` second.
pub fn rtt_read_spans(cap: u16, wr: u16, rd: u16) -> Result<([(u16, u16); 2], u16), ProtoError> {
    if cap != RTT_CAP || wr >= cap || rd >= cap {
        return Err(ProtoError::InvalidArg);
    }
    let avail = wr.wrapping_sub(rd) % cap;
    if avail == 0 {
        return Ok(([(0, 0), (0, 0)], rd));
    }
    let first = core::cmp::min(avail, cap - rd);
    let second = if avail > first {
        (0, avail - first)
    } else {
        (0, 0)
    };
    // `rd < cap` and `avail < cap`, so the sum stays well inside `u16`.
    let new_rd = (rd + avail) % cap;
    Ok(([(rd, first), second], new_rd))
}

/// Maps a breakpoint `slot` and `addr` to its `(register, value)` pair;
/// `addr == 0` clears the slot.
///
/// `slot` must be `0..=3` and `addr` must fit in 24 bits; otherwise
/// [`ProtoError::InvalidArg`]. A non-zero `addr` gets the enable bit
/// (`0x0100_0000`) set in the value.
pub fn breakpoint_reg_and_val(slot: u8, addr: u32) -> Result<(u16, u32), ProtoError> {
    if slot >= 4 || addr > BREAKPOINT_MAX_ADDR {
        return Err(ProtoError::InvalidArg);
    }
    let reg = BREAKPOINT_REG_BASE + u16::from(slot) * 4;
    let val = if addr == 0 {
        0
    } else {
        (addr & BREAKPOINT_MAX_ADDR) | BREAKPOINT_ENABLE
    };
    Ok((reg, val))
}

/// Returns `true` when `addr..addr + len` lies wholly inside the flash.
///
/// A zero `len` is rejected, and the end address must be strictly below
/// [`FLASH_SIZE`]; the sum is computed in `u64` so `addr == u32::MAX` cannot
/// overflow.
pub fn flash_range_valid(addr: u32, len: u16) -> bool {
    len > 0 && (u64::from(addr) + u64::from(len) - 1) < u64::from(FLASH_SIZE)
}

/// Returns `true` when `addr` is 4 KB sector aligned.
///
/// This checks alignment only; callers must also require `addr < FLASH_SIZE`.
pub fn flash_sector_aligned(addr: u32) -> bool {
    addr.is_multiple_of(FLASH_SECTOR_SIZE)
}

/// Returns `true` when `addr..addr + len` stays inside one 256 B flash page.
///
/// `len` must be `1..=256` and both ends must share the same page index; the
/// end address is computed in `u64` so any `u32` input is safe.
///
/// This checks page containment only; callers must also require
/// `addr < FLASH_SIZE`.
pub fn flash_page_valid(addr: u32, len: u16) -> bool {
    if !(1..=256).contains(&len) {
        return false;
    }
    let end = u64::from(addr) + u64::from(len) - 1;
    let page_mask = !0xff_u64;
    (u64::from(addr) & page_mask) == (end & page_mask)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn swire_accepts_both_mapped_windows() {
        assert!(is_valid_swire_range(0x0000, 1));
        assert!(is_valid_swire_range(0x0fff, 1));
        assert!(is_valid_swire_range(0x8000, 1));
        assert!(is_valid_swire_range(0xbfff, 1));
        // Exactly the 16 KB SRAM window.
        assert!(is_valid_swire_range(0x8000, 0x4000));
    }

    #[test]
    fn swire_rejects_dead_zones() {
        assert!(!is_valid_swire_range(0x1000, 1));
        assert!(!is_valid_swire_range(0x7fff, 1));
        assert!(!is_valid_swire_range(0xc000, 1));
        assert!(!is_valid_swire_range(0xffff, 1));
    }

    #[test]
    fn swire_rejects_zero_len_and_overflow() {
        assert!(!is_valid_swire_range(0x0000, 0));
        assert!(!is_valid_swire_range(0xffff, 0xffff));
        // Runs one byte past the SRAM window.
        assert!(!is_valid_swire_range(0xbfff, 2));
    }

    #[test]
    fn rtt_spans_empty_when_wr_equals_rd() {
        let (spans, new_rd) = rtt_read_spans(256, 0, 0).unwrap();
        assert_eq!(spans, [(0, 0), (0, 0)]);
        assert_eq!(new_rd, 0);

        let (spans, new_rd) = rtt_read_spans(256, 42, 42).unwrap();
        assert_eq!(spans, [(0, 0), (0, 0)]);
        assert_eq!(new_rd, 42);
    }

    #[test]
    fn rtt_spans_contiguous() {
        let (spans, new_rd) = rtt_read_spans(256, 10, 0).unwrap();
        assert_eq!(spans, [(0, 10), (0, 0)]);
        assert_eq!(new_rd, 10);
    }

    #[test]
    fn rtt_spans_wraparound() {
        let (spans, new_rd) = rtt_read_spans(256, 10, 250).unwrap();
        assert_eq!(spans, [(250, 6), (0, 10)]);
        assert_eq!(new_rd, 10);
    }

    #[test]
    fn rtt_spans_read_index_near_end() {
        let (spans, new_rd) = rtt_read_spans(256, 0, 255).unwrap();
        assert_eq!(spans, [(255, 1), (0, 0)]);
        assert_eq!(new_rd, 0);
    }

    #[test]
    fn rtt_spans_reject_bad_arguments() {
        assert_eq!(rtt_read_spans(256, 256, 0), Err(ProtoError::InvalidArg));
        assert_eq!(rtt_read_spans(256, 0, 256), Err(ProtoError::InvalidArg));
        assert_eq!(rtt_read_spans(128, 10, 0), Err(ProtoError::InvalidArg));
    }

    #[test]
    fn breakpoint_slot_registers() {
        for slot in 0..4u8 {
            let (reg, _) = breakpoint_reg_and_val(slot, 0x1234).unwrap();
            assert_eq!(reg, 0x0610 + u16::from(slot) * 4);
        }
        assert_eq!(breakpoint_reg_and_val(0, 0x1234).unwrap().0, 0x0610);
        assert_eq!(breakpoint_reg_and_val(3, 0x1234).unwrap().0, 0x061c);
    }

    #[test]
    fn breakpoint_values() {
        // `addr == 0` clears the slot.
        assert_eq!(breakpoint_reg_and_val(0, 0).unwrap(), (0x0610, 0));
        assert_eq!(
            breakpoint_reg_and_val(0, 0x1234).unwrap(),
            (0x0610, 0x0100_1234)
        );
        assert_eq!(
            breakpoint_reg_and_val(0, 0x00ff_ffff).unwrap(),
            (0x0610, 0x01ff_ffff)
        );
    }

    #[test]
    fn breakpoint_rejects_bad_arguments() {
        assert_eq!(
            breakpoint_reg_and_val(0, 0x0100_0000),
            Err(ProtoError::InvalidArg)
        );
        assert_eq!(
            breakpoint_reg_and_val(4, 0x1234),
            Err(ProtoError::InvalidArg)
        );
    }

    #[test]
    fn flash_range_bounds() {
        assert!(flash_range_valid(0, 1));
        assert!(flash_range_valid(0x7ffff, 1));
        assert!(!flash_range_valid(0x80000, 1));
        assert!(!flash_range_valid(0x7ffff, 2));
        assert!(!flash_range_valid(0, 0));
    }

    #[test]
    fn flash_sector_alignment() {
        assert!(flash_sector_aligned(0));
        assert!(flash_sector_aligned(4096));
        assert!(!flash_sector_aligned(100));
    }

    #[test]
    fn flash_page_containment() {
        assert!(flash_page_valid(0, 256));
        assert!(!flash_page_valid(0xf0, 32));
        assert!(flash_page_valid(0xf0, 16));
        assert!(!flash_page_valid(0, 257));
        assert!(!flash_page_valid(0, 0));
    }
}
