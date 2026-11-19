//! Command handlers for `probe-tlsr`.
//!
//! Each handler owns one request sequence and returns a typed [`Output`]. The
//! handlers take the transport directly so the protocol sequences are testable
//! with the scripted mock. The dispatch and the request/reply helpers live
//! here; the commands themselves are grouped by what they touch: the core
//! ([`control`]), SWire memory ([`mem`]), image programming ([`flash`]) and the
//! raw flash primitives ([`flash_raw`]).

mod control;
mod flash;
mod flash_raw;
mod mem;

#[cfg(test)]
mod test_support;

use probe_proto::{Request, Response};

use crate::output::Output;
use crate::transport::ProbeTransport;
use crate::{CliError, Command, Ctx};

/// SWire clock divider used by every command that talks to the target.
const SWS_DIV: u8 = 3;

/// SWire address of the CPU status byte.
const CPU_STATUS_ADDR: u16 = 0x0602;

/// Reset hold time used by `reset` and the flash path.
const RESET_HOLD_MS: u16 = 2;

/// Flash offset of the 12-byte firmware header.
const FLASH_HEADER_ADDR: u32 = 0x00;

/// Length of the firmware header.
const FLASH_HEADER_LEN: u16 = 12;

/// KNLT magic stored little-endian at header bytes `8..12`.
const KNLT_MAGIC: u32 = 0x544c_4e4b;

/// Bit 31 of the header version word selects the 16 MHz crystal.
const XTAL_BIT: u32 = 1 << 31;

/// Runs one parsed command.
pub fn run(cmd: Command, ctx: &mut Ctx) -> Result<Output, CliError> {
    match cmd {
        Command::Info { halt } => control::info(ctx.transport.as_mut(), halt),
        Command::Flash { path, force, reset } => {
            flash::flash(ctx.transport.as_mut(), &path, force, reset)
        }
        Command::FlashRead { addr, len, out } => {
            flash_raw::flash_read(ctx.transport.as_mut(), &addr, &len, out.as_deref())
        }
        Command::FlashErase { addr, len } => {
            flash_raw::flash_erase(ctx.transport.as_mut(), &addr, &len)
        }
        Command::FlashWrite { addr, file } => {
            flash_raw::flash_write(ctx.transport.as_mut(), &addr, &file)
        }
        Command::Read {
            addr_or_symbol,
            len,
        } => mem::read(
            ctx.transport.as_mut(),
            ctx.elf.as_ref(),
            &addr_or_symbol,
            len.as_deref(),
        ),
        Command::Write {
            addr_or_symbol,
            hex,
        } => mem::write(
            ctx.transport.as_mut(),
            ctx.elf.as_ref(),
            &addr_or_symbol,
            &hex,
        ),
        Command::AnalogRead { addr } => control::analog_read(ctx.transport.as_mut(), &addr),
        Command::AnalogWrite { addr, val } => {
            control::analog_write(ctx.transport.as_mut(), &addr, &val)
        }
        Command::Reset { stop, soft } => control::reset(ctx.transport.as_mut(), stop, soft),
        Command::Halt => control::halt(ctx.transport.as_mut()),
    }
}

fn sws_activate(transport: &mut dyn ProbeTransport) -> Result<(), CliError> {
    expect_ack(transport, Request::SwsActivate { div: SWS_DIV })
}

fn read_mem(transport: &mut dyn ProbeTransport, addr: u16, len: u16) -> Result<Vec<u8>, CliError> {
    match transport.exchange(&Request::ReadMem { addr, len })? {
        Response::Data(data) => expect_len(data.as_slice(), len).map(|()| data.as_slice().to_vec()),
        other => Err(unexpected("Data", &other)),
    }
}

fn read_flash(
    transport: &mut dyn ProbeTransport,
    addr: u32,
    len: u16,
) -> Result<Vec<u8>, CliError> {
    match transport.exchange(&Request::FlashRead { addr, len })? {
        Response::Data(data) => expect_len(data.as_slice(), len).map(|()| data.as_slice().to_vec()),
        other => Err(unexpected("Data", &other)),
    }
}

/// Requires an `Ack`; any other variant is a device fault.
fn expect_ack(transport: &mut dyn ProbeTransport, req: Request) -> Result<(), CliError> {
    match transport.exchange(&req)? {
        Response::Ack => Ok(()),
        other => Err(unexpected("Ack", &other)),
    }
}

/// Rejects a reply that is not the variant the request documents.
fn unexpected(expected: &str, response: &Response) -> CliError {
    CliError::Device(format!("expected {expected} response, got {response:?}"))
}

/// Requires the reply body to be exactly the requested length.
fn expect_len(data: &[u8], want: u16) -> Result<(), CliError> {
    let want = usize::from(want);
    match data.len() {
        got if got == want => Ok(()),
        got if got < want => Err(CliError::Device(format!(
            "short read: got {got}, want {want}"
        ))),
        got => Err(CliError::Device(format!(
            "long read: got {got}, want {want}"
        ))),
    }
}

/// Formats bytes as contiguous lower-case hex, no separators.
fn hex_string(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for &byte in bytes {
        out.push(char::from(HEX[usize::from(byte >> 4)]));
        out.push(char::from(HEX[usize::from(byte & 0x0f)]));
    }
    out
}
