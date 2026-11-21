//! Command handlers for `probe-tlsr`.
//!
//! Each handler owns one request sequence and returns a typed [`Output`]. The
//! handlers take the transport directly so the protocol sequences are testable
//! with the scripted mock. The dispatch and the request/reply helpers live
//! here; the commands themselves are grouped by what they touch: the core
//! ([`control`]), SWire memory ([`mem`]), image programming ([`flash`]), the
//! raw flash primitives ([`flash_raw`]) and crash inspection ([`diag`]).

mod control;
pub(crate) mod diag;
mod flash;
mod flash_raw;
mod mem;

#[cfg(test)]
mod test_support;

#[cfg(test)]
mod tests;

use std::io::Write;

use probe_proto::{CpuMode, Request, Response};

use crate::output::Output;
use crate::transport::ProbeTransport;
use crate::{CliError, Command, Ctx};

/// SWire clock divider used by every command that talks to the target.
const SWS_DIV: u8 = 3;

/// SWire address of the CPU status byte.
const CPU_STATUS_ADDR: u16 = 0x0602;

/// SWire address of the Timer 2 watchdog control byte.
const WATCHDOG_CTRL_ADDR: u16 = 0x0622;

/// `TMR_WD_EN`, bit 7 of [`WATCHDOG_CTRL_ADDR`].
const TMR_WD_EN: u8 = 0x80;

/// Analog register holding the retained reset sentinel.
const RESET_SENTINEL_ADDR: u8 = 0x3b;

/// Sentinel value written before an intentional probe-initiated reboot.
const SENTINEL_CLEAN_REBOOT: u8 = 0x00;

/// Sentinel value left by the panic handler; never overwritten by a reset.
const SENTINEL_PANIC: u8 = 0xe1;

/// Reset hold time used by `reset` and the flash path.
const RESET_HOLD_MS: u16 = 2;

/// Flash offset of the 12-byte firmware header.
const FLASH_HEADER_ADDR: u32 = 0x00;

/// Length of the firmware header.
const FLASH_HEADER_LEN: u16 = 12;

/// Byte offset of the firmware version word in the 12-byte header.
///
/// The reset branch is 2 bytes at address 0, the version `.word` follows
/// immediately (2..6), and `KNLT` starts at 8..12.
const HEADER_VERSION_OFFSET: usize = 2;

/// KNLT magic stored little-endian at header bytes `8..12`.
const KNLT_MAGIC: u32 = 0x544c_4e4b;

/// Runs one parsed command, returning the output to render.
///
/// `rtt` and `trap` are the self-rendering exceptions: `rtt` writes its lines
/// as they arrive and `trap` writes its one-shot hit capture before it clears
/// the slot, so both return `None` — there is no object left to print after
/// them.
pub fn run(cmd: Command, ctx: &mut Ctx, out: &mut dyn Write) -> Result<Option<Output>, CliError> {
    let output = match cmd {
        Command::Info { halt } => control::info(ctx.transport.as_mut(), halt)?,
        Command::Flash { path, reset } => flash::flash(ctx.transport.as_mut(), &path, reset)?,
        Command::FlashRead { addr, len, out } => {
            flash_raw::flash_read(ctx.transport.as_mut(), &addr, &len, out.as_deref())?
        }
        Command::FlashErase { addr, len } => {
            flash_raw::flash_erase(ctx.transport.as_mut(), &addr, &len)?
        }
        Command::FlashWrite { addr, file } => {
            flash_raw::flash_write(ctx.transport.as_mut(), &addr, &file)?
        }
        Command::Read {
            addr_or_symbol,
            len,
        } => mem::read(
            ctx.transport.as_mut(),
            ctx.elf.as_ref(),
            &addr_or_symbol,
            len.as_deref(),
        )?,
        Command::Write {
            addr_or_symbol,
            hex,
        } => mem::write(
            ctx.transport.as_mut(),
            ctx.elf.as_ref(),
            &addr_or_symbol,
            &hex,
        )?,
        Command::AnalogRead { addr } => control::analog_read(ctx.transport.as_mut(), &addr)?,
        Command::AnalogWrite { addr, val } => {
            control::analog_write(ctx.transport.as_mut(), &addr, &val)?
        }
        Command::Reset { stop, soft } => control::reset(ctx.transport.as_mut(), stop, soft)?,
        Command::Inspect => diag::inspect(ctx)?,
        Command::Rtt { until, watchdog_ms } => {
            diag::rtt::run(ctx, out, until.as_deref(), watchdog_ms)?;
            return Ok(None);
        }
        Command::Trap {
            addr_or_symbol,
            slot,
        } => {
            diag::trap::run(ctx, out, &addr_or_symbol, slot)?;
            return Ok(None);
        }
        Command::Halt => control::halt(ctx.transport.as_mut())?,
    };
    Ok(Some(output))
}

/// Stalls the core, then clears `TMR_WD_EN` so Timer 2 cannot reset the chip
/// while it is stalled.
///
/// The stall is terminal until `reset`; every caller leaves the core stopped.
///
/// A core that is already stopped (`0x05`) or stalled (`0x06`) is left alone.
/// Writing `0x06` while `[0x0602] == 0x05` is not a no-op on this silicon: it
/// releases the reset hold and starts the core, which then races any MSPI
/// read. A stopped core is already safe for register reads and flash access,
/// so the only thing left to do is hold the watchdog.
pub(super) fn stall_core(transport: &mut dyn ProbeTransport) -> Result<(), CliError> {
    let status = read_mem(transport, CPU_STATUS_ADDR, 1)?[0];
    if !matches!(status, 0x05 | 0x06) {
        expect_ack(
            transport,
            Request::CpuCtrl {
                mode: CpuMode::Stall,
            },
        )?;
    }
    pause_watchdog(transport)
}

/// Clears `TMR_WD_EN` (bit 7 of `0x0622`) without touching `0x0602`.
///
/// Timer 2 keeps counting after `CpuCtrl::Stop`/`Reset` and during a stall, so
/// a long erase/program/read must disarm it or the chip resets mid-operation.
pub(super) fn pause_watchdog(transport: &mut dyn ProbeTransport) -> Result<(), CliError> {
    let watchdog_ctrl = read_mem(transport, WATCHDOG_CTRL_ADDR, 1)?[0] & !TMR_WD_EN;
    expect_ack(
        transport,
        Request::WriteMem {
            addr: WATCHDOG_CTRL_ADDR,
            data: [watchdog_ctrl].into_iter().collect(),
        },
    )
}

/// Best-effort "this reset is intentional" marker for the next boot.
///
/// The analog sentinel survives a soft reboot (and a watchdog reset), so a
/// probe-initiated soft reboot without this would log `[WD_RESET]`. A panic
/// sentinel is kept: the next boot must still report the panic. Returns whether
/// SWire was activated, so the caller can skip a second `SwsActivate`; any
/// failure is ignored because a target that cannot answer SWire must still be
/// resettable.
pub(super) fn mark_clean_reboot(transport: &mut dyn ProbeTransport) -> bool {
    if sws_activate(transport).is_err() {
        return false;
    }
    let Ok(sentinel) = read_analog_value(transport, RESET_SENTINEL_ADDR) else {
        return true;
    };
    if sentinel != SENTINEL_PANIC {
        let _ = expect_ack(
            transport,
            Request::WriteAnalog {
                addr: RESET_SENTINEL_ADDR,
                val: SENTINEL_CLEAN_REBOOT,
            },
        );
    }
    true
}

/// Reads one analog register; the caller has already activated SWire.
pub(super) fn read_analog_value(
    transport: &mut dyn ProbeTransport,
    addr: u8,
) -> Result<u8, CliError> {
    match transport.exchange(&Request::ReadAnalog { addr })? {
        Response::Byte(value) => Ok(value),
        other => Err(unexpected("Byte", &other)),
    }
}

/// Activates SWire and accepts either reply the probe firmware sends.
///
/// `probe-pico` answers `SwsActivate` with the chip id it read back
/// (`Response::ChipId`), not `Response::Ack`; both mean the bus is up. Any
/// other variant is a device fault.
fn sws_activate(transport: &mut dyn ProbeTransport) -> Result<(), CliError> {
    match transport.exchange(&Request::SwsActivate { div: SWS_DIV })? {
        Response::Ack | Response::ChipId { .. } => Ok(()),
        other => Err(unexpected("Ack or ChipId", &other)),
    }
}

fn read_mem(transport: &mut dyn ProbeTransport, addr: u16, len: u16) -> Result<Vec<u8>, CliError> {
    match transport.exchange(&Request::ReadMem { addr, len })? {
        Response::Data(data) => expect_len(data.as_slice(), len).map(|()| data.as_slice().to_vec()),
        other => Err(unexpected("Data", &other)),
    }
}

/// Reads one little-endian `u32` from SWire memory.
///
/// Shared by every register/word poll (`info`'s PC, `rtt`'s heartbeat, `trap`'s
/// breakpoint control word), so the decode cannot drift between them.
fn read_word(transport: &mut dyn ProbeTransport, addr: u16) -> Result<u32, CliError> {
    // `read_mem` already rejected any reply that is not exactly 4 bytes.
    let bytes = read_mem(transport, addr, 4)?;
    Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
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
