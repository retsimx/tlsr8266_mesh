//! Request/response messages, their tag space and body codecs.
//!
//! Bodies are little-endian for multi-byte fields and never carry the tag or the
//! CRC: `frame` owns those. Encoding and decoding both validate, so `probe-pico`
//! and `probe-tlsr` reject the same values.
//!
//! Requests live in `request.rs` and responses in `response.rs`; this module
//! owns the shared vocabulary (error codes, CPU modes, payload limits) and the
//! body length check both directions use.

mod request;
mod response;

pub use request::Request;
pub use response::Response;

/// Maximum `WriteMem` / `FlashWritePage` payload, in bytes.
pub const MAX_WRITE_PAYLOAD: usize = 256;

/// Maximum `Data` response payload, in bytes.
pub const MAX_DATA_PAYLOAD: usize = 1024;

/// CPU control mode carried by [`Request::CpuCtrl`] (wire values `0..=3`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CpuMode {
    /// `0`: stop/reset the core (`0x0602 = 0x05`).
    Stop = 0,
    /// `1`: terminal stall (`0x0602 = 0x06`); not resumable, recover with
    /// [`CpuMode::SoftReboot`].
    Stall = 1,
    /// `2`: run the core (`0x0602 = 0x00`).
    Run = 2,
    /// `3`: soft reboot via `0x006f = 0x20`.
    SoftReboot = 3,
}

impl CpuMode {
    /// Wire value of this mode.
    pub const fn code(self) -> u8 {
        self as u8
    }

    /// Maps a wire value to a mode, or `None` if it is not `0..=3`.
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Stop),
            1 => Some(Self::Stall),
            2 => Some(Self::Run),
            3 => Some(Self::SoftReboot),
            _ => None,
        }
    }
}

/// Protocol error codes, shared by the wire (`Error` responses) and this crate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ProtoError {
    /// Structurally invalid frame.
    InvalidFrame = 1,
    /// CRC mismatch.
    BadCrc = 2,
    /// Address or span outside a mapped region.
    InvalidAddrRange = 3,
    /// Any other invalid argument.
    InvalidArg = 4,
    /// SWire transaction timed out.
    SwsTimeout = 5,
    /// Flash read-back did not match what was written.
    FlashVerifyFailed = 6,
    /// Flash stayed busy past the timeout.
    FlashBusyTimeout = 7,
}

impl ProtoError {
    /// Wire code of this error.
    pub const fn code(self) -> u8 {
        self as u8
    }

    /// Maps a wire code to an error; unknown codes become
    /// [`ProtoError::InvalidFrame`].
    pub const fn from_code(code: u8) -> Self {
        match code {
            1 => Self::InvalidFrame,
            2 => Self::BadCrc,
            3 => Self::InvalidAddrRange,
            4 => Self::InvalidArg,
            5 => Self::SwsTimeout,
            6 => Self::FlashVerifyFailed,
            7 => Self::FlashBusyTimeout,
            _ => Self::InvalidFrame,
        }
    }
}

/// Requires `body` to be exactly `expected` bytes long.
fn expect_len(body: &[u8], expected: usize) -> Result<(), ProtoError> {
    if body.len() == expected {
        Ok(())
    } else {
        Err(ProtoError::InvalidFrame)
    }
}

#[cfg(test)]
mod tests;
