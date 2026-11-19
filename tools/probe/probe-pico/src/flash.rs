//! MSPI flash driver and RTT ring drain for the TLSR8266.
//!
//! `0x000d` is the MSPI chip-select latch, `0x000c` the data port and `0x00b3`
//! the FIFO/RAM mode switch. A half-programmed MSPI session wedges the next
//! flash transaction, so every flash method has exactly one `restore_flash()`
//! site that runs before the result is returned.

use embassy_time::{Duration, Instant};
use probe_proto::{MAX_WRITE_PAYLOAD, ProtoError, crc16_ccitt, rtt_read_spans};

use crate::swire::SwireBus;

/// Erase waits for the WIP bit to clear for at most this long.
const ERASE_TIMEOUT: Duration = Duration::from_millis(500);
/// Page program waits for the WIP bit to clear for at most this long.
const WRITE_TIMEOUT: Duration = Duration::from_millis(50);

impl<'d> SwireBus<'d> {
    /// Leave the MSPI block in RAM mode with CSN released.
    ///
    /// Both writes are attempted even when the first one fails: a wedged
    /// session still has to see the CSN release attempt.
    fn restore_flash(&mut self) -> Result<(), ProtoError> {
        let mode = self.write_mem(0x00b3, &[0x00]);
        let csn = self.write_mem(0x000d, &[0x01]);
        mode?;
        csn
    }

    /// Write one MSPI command, address or read-strobe byte per SWS transaction.
    ///
    /// SWS auto-increment is *address*-level, not register-level: in normal mode
    /// (`0x00b3 = 0x00`) a multi-byte write to `0x000c` puts the second byte on
    /// `0x000d` (the CSN latch) and the rest on `0x000e`/`0x000f`. Every opcode,
    /// address and read-strobe byte therefore needs its own
    /// `0x5a 0x00 0x0c 0x00 <byte>` transaction, exactly as
    /// `remote.py::read_flash`/`write_flash_sector` do. Only the bulk payload may
    /// be one multi-byte write, and only after `0x00b3 = 0x80` selects FIFO
    /// mode; that is the one remaining direct `0x000c` write.
    fn flash_cmd(&mut self, cmd: &[u8]) -> Result<(), ProtoError> {
        for &byte in cmd {
            self.write_mem(0x000c, &[byte])?;
        }
        Ok(())
    }

    /// Read one byte from the MSPI data port (`0x000c = 0xff`, then read).
    fn read_flash_byte(&mut self) -> Result<u8, ProtoError> {
        self.flash_cmd(&[0xff])?;
        let mut byte = [0u8; 1];
        self.read_mem(0x000c, &mut byte)?;
        Ok(byte[0])
    }

    /// Poll the flash WIP bit (status `0x05`, bit 0) until it clears.
    fn wait_wip(&mut self, timeout: Duration) -> Result<(), ProtoError> {
        let polled = (|| {
            let deadline = Instant::now() + timeout;
            loop {
                self.write_mem(0x000d, &[0x00])?;
                self.flash_cmd(&[0x05])?;
                let status = self.read_flash_byte()?;
                self.write_mem(0x000d, &[0x01])?;
                // Bit 0 is WIP: the array is still programming/erasing.
                if status & 1 == 0 {
                    return Ok(());
                }
                if Instant::now() >= deadline {
                    return Err(ProtoError::FlashBusyTimeout);
                }
            }
        })();
        let restored = self.restore_flash();
        polled.and(restored)
    }

    /// Read the flash JEDEC ID (`0x9f`) through the MSPI FIFO.
    pub fn flash_jedec_id(&mut self) -> Result<[u8; 3], ProtoError> {
        let id = (|| {
            self.write_mem(0x000d, &[0x00])?;
            self.flash_cmd(&[0x9f])?;
            self.write_mem(0x00b3, &[0x80])?;
            let mut id = [0u8; 3];
            for slot in id.iter_mut() {
                *slot = self.read_flash_byte()?;
            }
            Ok(id)
        })();
        // Single restore site: no flash method returns before this point.
        let restored = self.restore_flash();
        id.and_then(|id| restored.map(|()| id))
    }

    /// Read `buf.len()` bytes starting at flash address `addr` (`0x03`).
    pub fn flash_read(&mut self, addr: u32, buf: &mut [u8]) -> Result<usize, ProtoError> {
        let read = (|| {
            self.write_mem(0x000d, &[0x00])?;
            self.flash_cmd(&[0x03, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8])?;
            self.write_mem(0x00b3, &[0x80])?;
            for slot in buf.iter_mut() {
                *slot = self.read_flash_byte()?;
            }
            Ok(buf.len())
        })();
        // Single restore site: no flash method returns before this point.
        let restored = self.restore_flash();
        read.and_then(|len| restored.map(|()| len))
    }

    /// Erase the 4 KiB sector containing `addr` (`0x20` + 24-bit address).
    pub fn flash_erase_sector(&mut self, addr: u32) -> Result<(), ProtoError> {
        let issued = (|| {
            // Write enable; it is consumed by the next command.
            self.write_mem(0x000d, &[0x00])?;
            self.flash_cmd(&[0x06])?;
            self.write_mem(0x000d, &[0x01])?;

            self.write_mem(0x000d, &[0x00])?;
            self.flash_cmd(&[0x20, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8])?;
            self.write_mem(0x000d, &[0x01])
        })();
        // Single restore site: no flash method returns before this point.
        let restored = self.restore_flash();
        issued.and(restored)?;
        self.wait_wip(ERASE_TIMEOUT)
    }

    /// Program one page (`0x02` + 24-bit address + data), then verify it.
    pub fn flash_write_page(&mut self, addr: u32, data: &[u8]) -> Result<(), ProtoError> {
        let programmed = (|| {
            // A page is at most `MAX_WRITE_PAYLOAD` bytes; the read-back buffer
            // below is fixed, so anything larger is rejected before any I/O.
            if data.len() > MAX_WRITE_PAYLOAD {
                return Err(ProtoError::InvalidArg);
            }

            // Write enable; it is consumed by the next command.
            self.write_mem(0x000d, &[0x00])?;
            self.flash_cmd(&[0x06])?;
            self.write_mem(0x000d, &[0x01])?;

            self.write_mem(0x000d, &[0x00])?;
            self.flash_cmd(&[0x02, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8])?;
            self.write_mem(0x00b3, &[0x80])?;
            // Bulk payload only: FIFO mode makes repeated `0x000c` writes land on
            // the same address.
            self.write_mem(0x000c, data)
        })();
        // Single restore site: no flash method returns before this point.
        let restored = self.restore_flash();
        programmed.and(restored)?;
        self.wait_wip(WRITE_TIMEOUT)?;

        let mut readback = [0u8; MAX_WRITE_PAYLOAD];
        let readback = &mut readback[..data.len()];
        self.flash_read(addr, readback)?;
        if crc16_ccitt(readback) != crc16_ccitt(data) {
            return Err(ProtoError::FlashVerifyFailed);
        }
        Ok(())
    }

    /// Drain the target's `_TLSR_RTT` ring and advance its read index.
    ///
    /// The 16-byte header is `magic` (`+0..+8`), `cap` (`+8..+10`), `wr`
    /// (`+10..+12`), `rd` (`+12..+14`) and `flags` (`+14..+16`), all `u16`
    /// little-endian, followed by the ring at `+16`. `rd` is only written after
    /// every span read succeeded, so a failed poll never loses bytes.
    pub fn rtt_poll(&mut self, ctrl_addr: u16, out: &mut [u8]) -> Result<usize, ProtoError> {
        let mut header = [0u8; 16];
        self.read_mem(ctrl_addr, &mut header)?;
        if header[..8] != *b"TLSR_RTT" {
            return Err(ProtoError::InvalidArg);
        }

        let cap = u16::from_le_bytes([header[8], header[9]]);
        let wr = u16::from_le_bytes([header[10], header[11]]);
        let rd = u16::from_le_bytes([header[12], header[13]]);
        let (spans, new_rd) = rtt_read_spans(cap, wr, rd)?;

        // Reject a too-small output buffer before touching the ring, so a
        // failure can never leave `rd` partially advanced.
        let total = usize::from(spans[0].1) + usize::from(spans[1].1);
        if total > out.len() {
            return Err(ProtoError::InvalidArg);
        }

        let base = u32::from(ctrl_addr) + 16;
        let mut written = 0;
        for &(offset, len) in &spans {
            if len == 0 {
                continue;
            }
            let addr = base + u32::from(offset);
            let addr = u16::try_from(addr).map_err(|_| ProtoError::InvalidArg)?;
            let end = written + usize::from(len);
            self.read_mem(addr, &mut out[written..end])?;
            written = end;
        }

        let rd_addr = u32::from(ctrl_addr) + 12;
        let rd_addr = u16::try_from(rd_addr).map_err(|_| ProtoError::InvalidArg)?;
        self.write_mem(rd_addr, &new_rd.to_le_bytes())?;
        Ok(written)
    }
}
