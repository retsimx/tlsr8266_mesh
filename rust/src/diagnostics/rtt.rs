//! Fixed-layout byte ring shared with the SWire host tool.

use core::sync::atomic::{AtomicU16, Ordering};

/// Capacity of the data area; ring indices are taken modulo this.
pub const RTT_CAP: usize = 256;
const FLAG_OVERFLOW: u16 = 1;

/// Control block layout, consumed by `probe-tlsr rtt`:
/// `magic` +0, `cap` +8, `wr` +10, `rd` +12, `flags` +14, `buf` +16.
///
/// `magic`, `cap`, `flags` and `buf` are read by the host tool rather than by
/// firmware code, so the compiler sees them as write-only.
#[repr(C)]
pub struct RttControlBlock {
    magic: [u8; 8],
    cap: u16,
    wr: AtomicU16,
    /// Advanced by the host as it drains; firmware only ever reads it.
    rd: AtomicU16,
    /// Bit 0 = overflow has occurred. Sticky: firmware only sets it and the
    /// set is a plain read-modify-write, so the host must treat it read-only.
    flags: u16,
    buf: [u8; RTT_CAP],
}

impl RttControlBlock {
    /// Fully zero-initialised, so the static lives in `.bss` and costs no
    /// flash image bytes. `cap == 0` means "not initialised yet" to a reader.
    pub const fn zeroed() -> Self {
        Self {
            magic: [0; 8],
            cap: 0,
            wr: AtomicU16::new(0),
            rd: AtomicU16::new(0),
            flags: 0,
            buf: [0; RTT_CAP],
        }
    }

    /// Advertises the control block to the host. Runs a few ms into boot,
    /// long before anything could be streaming.
    pub fn init(&mut self) {
        self.magic = *b"TLSR_RTT";
        self.cap = RTT_CAP as u16;
    }

    /// Copies as many bytes as fit into the ring, then drops the rest.
    ///
    /// Never blocks, never panics, never grows: a full ring sets the overflow
    /// flag and the tail of `bytes` is lost. Corrupt indices (a host writing
    /// `rd` out of range, or uninitialised state) are treated as 0. `rd` is
    /// sampled once per call, so a host that drains mid-write can trip a
    /// spurious overflow flag — staleness only ever costs log bytes, it can
    /// never make the writer overwrite unread data.
    pub fn write(&mut self, bytes: &[u8]) {
        let mut wr = self.wr.load(Ordering::Relaxed) as usize;
        if wr >= RTT_CAP {
            wr = 0;
        }

        let mut rd = self.rd.load(Ordering::Relaxed) as usize;
        if rd >= RTT_CAP {
            rd = 0;
        }

        for &byte in bytes {
            let next = (wr + 1) % RTT_CAP;

            // The host has not consumed the slot we would overwrite.
            if next == rd {
                self.flags |= FLAG_OVERFLOW;
                break;
            }

            // SAFETY: (5) panic/RTT call graph. `wr` is a valid index: it is
            // reduced modulo `RTT_CAP` above and a corrupt value is reset to
            // 0, so `wr < RTT_CAP`. A checked store would put
            // `panic_bounds_check` on the panic handler's call graph and the
            // static stack gate rejects it.
            unsafe { *self.buf.get_unchecked_mut(wr) = byte };
            wr = next;
            self.wr.store(wr as u16, Ordering::Relaxed);
        }
    }
}

/// Host-tool contract: this exact symbol name and layout are consumed by
/// `probe-tlsr rtt`, so it may not be renamed.
#[no_mangle]
#[allow(non_upper_case_globals)]
pub static _TLSR_RTT: critical_section::Mutex<core::cell::UnsafeCell<RttControlBlock>> =
    critical_section::Mutex::new(core::cell::UnsafeCell::new(RttControlBlock::zeroed()));

// The host tool reads the control block straight from the symbol address, so
// the mutex wrapper must not add a tag byte or any padding in front of it.
const _: () = assert!(
    core::mem::size_of::<critical_section::Mutex<core::cell::UnsafeCell<RttControlBlock>>>()
        == core::mem::size_of::<RttControlBlock>()
);
// `probe-tlsr rtt` hard-codes the sampled size of the ring.
const _: () = assert!(core::mem::size_of::<RttControlBlock>() == 272);

#[cfg(feature = "rtt")]
pub fn rtt_init() {
    critical_section::with(|cs| {
        // SAFETY: (4) audited `Sync`-wrapped static. `borrow` yields the
        // control-block cell for exactly this critical section: the cell is a
        // `static` (never null, aligned for `RttControlBlock`) and interrupts
        // are masked for the whole closure, so no other context can hold a
        // reference to it. The dereference cannot be made safe because the
        // mutex deliberately exposes no `&mut T`.
        let control_block = unsafe { &mut *_TLSR_RTT.borrow(cs).get() };
        control_block.init();
    });
}

#[cfg(feature = "rtt")]
pub fn rtt_write_bytes(bytes: &[u8]) {
    critical_section::with(|cs| {
        // SAFETY: (4) as in `rtt_init`: the critical section makes this the
        // only accessor of the `static` cell for the duration of the call.
        let control_block = unsafe { &mut *_TLSR_RTT.borrow(cs).get() };
        control_block.write(bytes);
    });
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use core::mem::{offset_of, size_of};

    use super::*;

    #[test]
    fn test_rtt_control_block_layout() {
        assert_eq!(size_of::<RttControlBlock>(), 272);
        assert_eq!(offset_of!(RttControlBlock, magic), 0);
        assert_eq!(offset_of!(RttControlBlock, cap), 8);
        assert_eq!(offset_of!(RttControlBlock, wr), 10);
        assert_eq!(offset_of!(RttControlBlock, rd), 12);
        assert_eq!(offset_of!(RttControlBlock, flags), 14);
        assert_eq!(offset_of!(RttControlBlock, buf), 16);
    }

    #[test]
    fn test_rtt_zeroed_is_all_zero() {
        let control_block = RttControlBlock::zeroed();

        assert_eq!(control_block.cap, 0);
        assert_eq!(control_block.wr.load(Ordering::Relaxed), 0);
        assert_eq!(control_block.rd.load(Ordering::Relaxed), 0);
        assert_eq!(control_block.flags, 0);
        assert!(control_block.buf.iter().all(|&byte| byte == 0));
    }

    #[test]
    fn test_rtt_write_and_drain() {
        let mut control_block = RttControlBlock::zeroed();
        control_block.init();

        control_block.write(b"hello");

        assert_eq!(&control_block.magic, b"TLSR_RTT");
        assert_eq!(control_block.cap, RTT_CAP as u16);
        assert_eq!(control_block.wr.load(Ordering::Relaxed), 5);
        assert_eq!(control_block.rd.load(Ordering::Relaxed), 0);
        assert_eq!(&control_block.buf[..5], b"hello");
        assert_eq!(control_block.flags, 0);
    }

    #[test]
    fn test_rtt_wraparound() {
        let mut control_block = RttControlBlock::zeroed();
        control_block.init();

        // The host drained everything up to 200, so both indices point at 200.
        control_block.wr.store(200, Ordering::Relaxed);
        control_block.rd.store(200, Ordering::Relaxed);

        let data: [u8; 100] = core::array::from_fn(|i| i as u8);
        control_block.write(&data);

        assert_eq!(control_block.wr.load(Ordering::Relaxed), 44);
        assert_eq!(control_block.flags, 0);
        assert_eq!(control_block.buf[255], data[55]);
        for i in 0..44 {
            assert_eq!(control_block.buf[i], data[56 + i]);
        }
    }

    #[test]
    fn test_rtt_overflow_drops_tail() {
        let mut control_block = RttControlBlock::zeroed();
        control_block.init();

        let data: [u8; 300] = core::array::from_fn(|i| i as u8);
        control_block.write(&data);

        // One slot is always kept free to tell "full" from "empty".
        assert_eq!(control_block.wr.load(Ordering::Relaxed), 255);
        assert_eq!(control_block.flags & FLAG_OVERFLOW, FLAG_OVERFLOW);
        assert_eq!(&control_block.buf[..255], &data[..255]);
    }

    #[test]
    fn test_rtt_corrupt_indices_treated_as_empty() {
        let mut control_block = RttControlBlock::zeroed();
        control_block.init();

        control_block.wr.store(0xFFFF, Ordering::Relaxed);
        control_block.rd.store(0xFFFF, Ordering::Relaxed);

        control_block.write(b"x");

        assert_eq!(control_block.wr.load(Ordering::Relaxed), 1);
        assert_eq!(control_block.buf[0], b'x');
    }
}
