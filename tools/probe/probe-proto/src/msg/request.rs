//! Host-to-probe requests, tags `0x01..=0x0f`.

use heapless::Vec;

use super::{expect_len, CpuMode, ProtoError, MAX_WRITE_PAYLOAD};
use crate::helpers::{
    flash_page_valid, flash_range_valid, flash_sector_aligned, is_valid_swire_range,
};
use crate::FLASH_SIZE;

/// Host-to-probe requests, tags `0x01..=0x0f`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Request {
    /// Tag `0x01`: liveness check, empty body.
    Ping,
    /// Tag `0x02`: hold the core in reset for `hold_ms` before releasing it.
    Reset {
        /// Reset hold time in milliseconds; must be non-zero.
        hold_ms: u16,
        /// Whether the core stays stopped after the hold.
        stop_cpu: bool,
    },
    /// Tag `0x03`: activate the SWire interface with clock divider `div`.
    SwsActivate {
        /// Clock divider; must be non-zero.
        div: u8,
    },
    /// Tag `0x04`: read `len` bytes of SWire address space at `addr`.
    ReadMem {
        /// First address to read.
        addr: u16,
        /// Byte count, `1..=1024`.
        len: u16,
    },
    /// Tag `0x05`: write `data` to SWire address space at `addr`.
    WriteMem {
        /// First address to write.
        addr: u16,
        /// Bytes to write, `1..=256`.
        data: Vec<u8, MAX_WRITE_PAYLOAD>,
    },
    /// Tag `0x06`: read the analog register at `addr`.
    ReadAnalog {
        /// Analog register address.
        addr: u8,
    },
    /// Tag `0x07`: write `val` to the analog register at `addr`.
    WriteAnalog {
        /// Analog register address.
        addr: u8,
        /// Value to write.
        val: u8,
    },
    /// Tag `0x08`: read the SPI NOR JEDEC ID.
    FlashJedecId,
    /// Tag `0x09`: read `len` flash bytes at `addr`.
    FlashRead {
        /// Flash byte address.
        addr: u32,
        /// Byte count, `1..=1024`.
        len: u16,
    },
    /// Tag `0x0a`: erase the 4 KB flash sector containing `addr`.
    FlashEraseSector {
        /// Sector-aligned flash byte address.
        addr: u32,
    },
    /// Tag `0x0b`: program one flash page with `data`.
    FlashWritePage {
        /// Flash byte address.
        addr: u32,
        /// Bytes to program, `1..=256`, within a single 256 B page.
        data: Vec<u8, MAX_WRITE_PAYLOAD>,
    },
    /// Tag `0x0c`: control the target core.
    CpuCtrl {
        /// Requested CPU mode.
        mode: CpuMode,
    },
    /// Tag `0x0d`: read the SWire register block.
    ReadRegs,
    /// Tag `0x0e`: install (`addr != 0`) or clear (`addr == 0`) a breakpoint.
    SetBreakpoint {
        /// Breakpoint slot, `0..=3`.
        slot: u8,
        /// 24-bit target address, or `0` to clear the slot.
        addr: u32,
    },
    /// Tag `0x0f`: poll the 272 B RTT window starting at `ctrl_addr`.
    RttPoll {
        /// RTT control block address.
        ctrl_addr: u16,
    },
}

impl Request {
    /// Wire tag of this request.
    pub fn tag(&self) -> u8 {
        match self {
            Self::Ping => 0x01,
            Self::Reset { .. } => 0x02,
            Self::SwsActivate { .. } => 0x03,
            Self::ReadMem { .. } => 0x04,
            Self::WriteMem { .. } => 0x05,
            Self::ReadAnalog { .. } => 0x06,
            Self::WriteAnalog { .. } => 0x07,
            Self::FlashJedecId => 0x08,
            Self::FlashRead { .. } => 0x09,
            Self::FlashEraseSector { .. } => 0x0a,
            Self::FlashWritePage { .. } => 0x0b,
            Self::CpuCtrl { .. } => 0x0c,
            Self::ReadRegs => 0x0d,
            Self::SetBreakpoint { .. } => 0x0e,
            Self::RttPoll { .. } => 0x0f,
        }
    }

    /// Encoded body length of this request, tag and CRC excluded.
    fn body_len(&self) -> usize {
        match self {
            Self::Ping | Self::FlashJedecId | Self::ReadRegs => 0,
            Self::Reset { .. } => 3,
            Self::SwsActivate { .. } => 1,
            Self::ReadMem { .. } => 4,
            Self::WriteMem { data, .. } => 2 + data.len(),
            Self::ReadAnalog { .. } => 1,
            Self::WriteAnalog { .. } => 2,
            Self::FlashRead { .. } => 6,
            Self::FlashEraseSector { .. } => 4,
            Self::FlashWritePage { data, .. } => 4 + data.len(),
            Self::CpuCtrl { .. } => 1,
            Self::SetBreakpoint { .. } => 5,
            Self::RttPoll { .. } => 2,
        }
    }

    /// Encodes the body (no tag, no CRC) into `out` and returns its length.
    ///
    /// Validates first, so an invalid request never writes into `out`. Returns
    /// [`ProtoError::InvalidArg`] if `out` is too small.
    pub fn encode_body(&self, out: &mut [u8]) -> Result<usize, ProtoError> {
        validate_request(self)?;
        let total = self.body_len();
        if out.len() < total {
            return Err(ProtoError::InvalidArg);
        }
        match self {
            Self::Ping | Self::FlashJedecId | Self::ReadRegs => {}
            Self::Reset { hold_ms, stop_cpu } => {
                out[..2].copy_from_slice(&hold_ms.to_le_bytes());
                out[2] = u8::from(*stop_cpu);
            }
            Self::SwsActivate { div } => out[0] = *div,
            Self::ReadMem { addr, len } => {
                out[..2].copy_from_slice(&addr.to_le_bytes());
                out[2..4].copy_from_slice(&len.to_le_bytes());
            }
            Self::WriteMem { addr, data } => {
                out[..2].copy_from_slice(&addr.to_le_bytes());
                out[2..total].copy_from_slice(data.as_slice());
            }
            Self::ReadAnalog { addr } => out[0] = *addr,
            Self::WriteAnalog { addr, val } => {
                out[0] = *addr;
                out[1] = *val;
            }
            Self::FlashRead { addr, len } => {
                out[..4].copy_from_slice(&addr.to_le_bytes());
                out[4..6].copy_from_slice(&len.to_le_bytes());
            }
            Self::FlashEraseSector { addr } => out[..4].copy_from_slice(&addr.to_le_bytes()),
            Self::FlashWritePage { addr, data } => {
                out[..4].copy_from_slice(&addr.to_le_bytes());
                out[4..total].copy_from_slice(data.as_slice());
            }
            Self::CpuCtrl { mode } => out[0] = mode.code(),
            Self::SetBreakpoint { slot, addr } => {
                out[0] = *slot;
                out[1..5].copy_from_slice(&addr.to_le_bytes());
            }
            Self::RttPoll { ctrl_addr } => out[..2].copy_from_slice(&ctrl_addr.to_le_bytes()),
        }
        Ok(total)
    }

    /// Decodes the exact body bytes of a request carrying `tag`.
    ///
    /// Trailing or missing bytes are [`ProtoError::InvalidFrame`]; argument
    /// faults keep their encode-side mapping.
    pub fn decode_body(tag: u8, body: &[u8]) -> Result<Self, ProtoError> {
        let request = match tag {
            0x01..=0x07 => decode_swire_body(tag, body)?,
            0x08..=0x0b => decode_flash_body(tag, body)?,
            0x0c..=0x0f => decode_control_body(tag, body)?,
            _ => return Err(ProtoError::InvalidFrame),
        };
        validate_request(&request)?;
        Ok(request)
    }
}

/// Decodes tags `0x01..=0x07`, the SWire-facing requests.
fn decode_swire_body(tag: u8, body: &[u8]) -> Result<Request, ProtoError> {
    let request = match tag {
        0x01 => {
            expect_len(body, 0)?;
            Request::Ping
        }
        0x02 => decode_reset(body)?,
        0x03 => {
            expect_len(body, 1)?;
            Request::SwsActivate { div: body[0] }
        }
        0x04 => {
            expect_len(body, 4)?;
            Request::ReadMem {
                addr: u16::from_le_bytes([body[0], body[1]]),
                len: u16::from_le_bytes([body[2], body[3]]),
            }
        }
        0x05 => decode_write_mem(body)?,
        0x06 => {
            expect_len(body, 1)?;
            Request::ReadAnalog { addr: body[0] }
        }
        0x07 => {
            expect_len(body, 2)?;
            Request::WriteAnalog {
                addr: body[0],
                val: body[1],
            }
        }
        _ => return Err(ProtoError::InvalidFrame),
    };
    Ok(request)
}

/// Decodes tags `0x08..=0x0b`, the SPI flash requests.
fn decode_flash_body(tag: u8, body: &[u8]) -> Result<Request, ProtoError> {
    let request = match tag {
        0x08 => {
            expect_len(body, 0)?;
            Request::FlashJedecId
        }
        0x09 => {
            expect_len(body, 6)?;
            Request::FlashRead {
                addr: u32::from_le_bytes([body[0], body[1], body[2], body[3]]),
                len: u16::from_le_bytes([body[4], body[5]]),
            }
        }
        0x0a => {
            expect_len(body, 4)?;
            Request::FlashEraseSector {
                addr: u32::from_le_bytes([body[0], body[1], body[2], body[3]]),
            }
        }
        0x0b => decode_flash_write_page(body)?,
        _ => return Err(ProtoError::InvalidFrame),
    };
    Ok(request)
}

/// Decodes tags `0x0c..=0x0f`, the core, register and RTT requests.
fn decode_control_body(tag: u8, body: &[u8]) -> Result<Request, ProtoError> {
    let request = match tag {
        0x0c => {
            expect_len(body, 1)?;
            Request::CpuCtrl {
                mode: CpuMode::from_code(body[0]).ok_or(ProtoError::InvalidFrame)?,
            }
        }
        0x0d => {
            expect_len(body, 0)?;
            Request::ReadRegs
        }
        0x0e => {
            expect_len(body, 5)?;
            Request::SetBreakpoint {
                slot: body[0],
                addr: u32::from_le_bytes([body[1], body[2], body[3], body[4]]),
            }
        }
        0x0f => {
            expect_len(body, 2)?;
            Request::RttPoll {
                ctrl_addr: u16::from_le_bytes([body[0], body[1]]),
            }
        }
        _ => return Err(ProtoError::InvalidFrame),
    };
    Ok(request)
}

/// Decodes a `Reset` body: `hold_ms: u16`, then `stop_cpu: u8` (`0` or `1`).
fn decode_reset(body: &[u8]) -> Result<Request, ProtoError> {
    expect_len(body, 3)?;
    let stop_cpu = match body[2] {
        0 => false,
        1 => true,
        _ => return Err(ProtoError::InvalidFrame),
    };
    Ok(Request::Reset {
        hold_ms: u16::from_le_bytes([body[0], body[1]]),
        stop_cpu,
    })
}

/// Decodes a `WriteMem` body: `addr: u16` plus the payload.
fn decode_write_mem(body: &[u8]) -> Result<Request, ProtoError> {
    if body.len() < 2 || body.len() > 2 + MAX_WRITE_PAYLOAD {
        return Err(ProtoError::InvalidFrame);
    }
    let mut data = Vec::new();
    data.extend_from_slice(&body[2..])
        .map_err(|_| ProtoError::InvalidFrame)?;
    Ok(Request::WriteMem {
        addr: u16::from_le_bytes([body[0], body[1]]),
        data,
    })
}

/// Decodes a `FlashWritePage` body: `addr: u32` plus the payload.
fn decode_flash_write_page(body: &[u8]) -> Result<Request, ProtoError> {
    if body.len() < 4 || body.len() > 4 + MAX_WRITE_PAYLOAD {
        return Err(ProtoError::InvalidFrame);
    }
    let mut data = Vec::new();
    data.extend_from_slice(&body[4..])
        .map_err(|_| ProtoError::InvalidFrame)?;
    Ok(Request::FlashWritePage {
        addr: u32::from_le_bytes([body[0], body[1], body[2], body[3]]),
        data,
    })
}

/// Validates a request's arguments, mapping span faults to
/// [`ProtoError::InvalidAddrRange`] and every other fault to
/// [`ProtoError::InvalidArg`].
fn validate_request(request: &Request) -> Result<(), ProtoError> {
    match request {
        Request::Ping
        | Request::ReadAnalog { .. }
        | Request::WriteAnalog { .. }
        | Request::FlashJedecId
        | Request::ReadRegs
        | Request::CpuCtrl { .. } => Ok(()),
        Request::Reset { hold_ms, .. } => {
            if *hold_ms == 0 {
                return Err(ProtoError::InvalidArg);
            }
            Ok(())
        }
        Request::SwsActivate { div } => {
            if *div == 0 {
                return Err(ProtoError::InvalidArg);
            }
            Ok(())
        }
        Request::ReadMem { addr, len } => validate_read_mem(*addr, *len),
        Request::WriteMem { addr, data } => validate_write_mem(*addr, data.len()),
        Request::FlashRead { addr, len } => validate_flash_read(*addr, *len),
        Request::FlashEraseSector { addr } => validate_flash_erase(*addr),
        Request::FlashWritePage { addr, data } => validate_flash_write_page(*addr, data.len()),
        Request::SetBreakpoint { slot, addr } => validate_breakpoint(*slot, *addr),
        Request::RttPoll { ctrl_addr } => validate_rtt_poll(*ctrl_addr),
    }
}

/// `ReadMem`: a `1..=1024` byte span wholly inside mapped SWire space.
fn validate_read_mem(addr: u16, len: u16) -> Result<(), ProtoError> {
    if !(1..=1024).contains(&len) {
        return Err(ProtoError::InvalidArg);
    }
    if !is_valid_swire_range(addr, len) {
        return Err(ProtoError::InvalidAddrRange);
    }
    Ok(())
}

/// `WriteMem`: a non-empty payload wholly inside mapped SWire space.
fn validate_write_mem(addr: u16, len: usize) -> Result<(), ProtoError> {
    if len == 0 {
        return Err(ProtoError::InvalidArg);
    }
    if !is_valid_swire_range(addr, len as u16) {
        return Err(ProtoError::InvalidAddrRange);
    }
    Ok(())
}

/// `FlashRead`: a `1..=1024` byte span wholly inside flash.
fn validate_flash_read(addr: u32, len: u16) -> Result<(), ProtoError> {
    if !(1..=1024).contains(&len) {
        return Err(ProtoError::InvalidArg);
    }
    if !flash_range_valid(addr, len) {
        return Err(ProtoError::InvalidAddrRange);
    }
    Ok(())
}

/// `FlashEraseSector`: in-flash, then sector aligned, in that order.
fn validate_flash_erase(addr: u32) -> Result<(), ProtoError> {
    if addr >= FLASH_SIZE {
        return Err(ProtoError::InvalidAddrRange);
    }
    if !flash_sector_aligned(addr) {
        return Err(ProtoError::InvalidArg);
    }
    Ok(())
}

/// `FlashWritePage`: non-empty, in-flash, then inside one page, in that order.
fn validate_flash_write_page(addr: u32, len: usize) -> Result<(), ProtoError> {
    if len == 0 {
        return Err(ProtoError::InvalidArg);
    }
    if addr >= FLASH_SIZE {
        return Err(ProtoError::InvalidAddrRange);
    }
    if !flash_page_valid(addr, len as u16) {
        return Err(ProtoError::InvalidArg);
    }
    Ok(())
}

/// `SetBreakpoint`: slot `0..=3` and an address that fits in 24 bits.
fn validate_breakpoint(slot: u8, addr: u32) -> Result<(), ProtoError> {
    if slot >= 4 || addr > 0x00ff_ffff {
        return Err(ProtoError::InvalidArg);
    }
    Ok(())
}

/// `RttPoll`: the control block must sit in the SWire SRAM window.
fn validate_rtt_poll(ctrl_addr: u16) -> Result<(), ProtoError> {
    if ctrl_addr < 0x8000 || !is_valid_swire_range(ctrl_addr, 272) {
        return Err(ProtoError::InvalidAddrRange);
    }
    Ok(())
}
