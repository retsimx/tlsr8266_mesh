#![cfg_attr(not(test), no_std)]

//! Shared wire protocol for the TLSR8266 SWire probe.
//!
//! This crate is the single definition of the USB-CDC protocol spoken between
//! the RP2040 probe firmware (`probe-pico`) and the host CLI (`probe-tlsr`):
//!
//! ```text
//! COBS([seq][tag][body][crc16_le]) 0x00
//! ```
//!
//! - `seq` is a caller-assigned sequence byte, echoed by the peer.
//! - `tag` selects the request or response variant (see [`msg`]).
//! - `body` is variant-specific, little-endian for multi-byte fields.
//! - `crc16_le` is CRC16-CCITT (poly `0x1021`, init `0xFFFF`, no reflection,
//!   no final XOR) over `seq || tag || body`, appended little-endian.
//! - The frame is COBS-encoded and terminated by a single `0x00` delimiter.
//!
//! Limits: [`MAX_DECODED_FRAME`] (1044) bytes after COBS decoding,
//! [`MAX_WIRE_FRAME`] (1050) bytes on the wire including the delimiter.
//!
//! ## Stack budget
//!
//! The frame codec keeps its working buffers on the stack: `encode_frame`
//! builds a 1040-byte body buffer plus a 1044-byte decoded frame, and
//! `decode_frame` builds a 1044-byte frame buffer. Measured on
//! `thumbv6m-none-eabi`, that costs roughly 2.1 KB for `encode_frame` and
//! 3.4 KB for `decode_frame` in release builds; debug builds can exceed 4 KB
//! for `decode_frame`. `probe-pico` (#20) must budget that much scratch space
//! per task (or hand the codec a caller-provided buffer) before calling these
//! from a small RP2040 stack.
//!
//! The crate is `no_std` in every non-test build so it can be linked into
//! Cortex-M0 firmware; the host test harness (`cfg(test)`) links `std`, which
//! is why the `no_std` attribute is conditional rather than unconditional.

/// Largest request/response body, in bytes.
pub const MAX_BODY: usize = 1040;

/// Largest COBS-decoded frame: `seq + tag + body + crc16_le`.
pub const MAX_DECODED_FRAME: usize = 1044;

/// Largest frame on the wire, including the `0x00` delimiter.
pub const MAX_WIRE_FRAME: usize = 1050;

/// Protocol version reported in `Pong.proto_ver`.
pub const PROTOCOL_VERSION: u8 = 1;

/// SPI NOR flash size in bytes (512 KB).
pub const FLASH_SIZE: u32 = 0x0008_0000;

/// MSPI page-program size in bytes.
pub const FLASH_PAGE_SIZE: u32 = 256;

/// MSPI sector-erase size in bytes.
pub const FLASH_SECTOR_SIZE: u32 = 4096;

/// MSPI opcode that reads the JEDEC ID.
pub const MSPI_CMD_READ_JEDEC_ID: u8 = 0x9f;

pub mod cobs;
pub mod crc;
pub mod frame;
pub mod helpers;
pub mod msg;

pub use cobs::{cobs_decode, cobs_encode};
pub use crc::crc16_ccitt;
pub use frame::{decode_frame, encode_frame, Direction, Message};
pub use helpers::{
    breakpoint_reg_and_val, flash_page_valid, flash_range_valid, flash_sector_aligned,
    is_valid_swire_range, rtt_read_spans,
};
pub use msg::{CpuMode, ProtoError, Request, Response, MAX_DATA_PAYLOAD, MAX_WRITE_PAYLOAD};
