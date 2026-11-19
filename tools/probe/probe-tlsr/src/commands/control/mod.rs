//! `info`, `halt`, `reset` and the `analog-*` pair.
//!
//! `info` samples the core before deciding whether to touch MSPI, and only
//! `halt`/`info --halt` clear the Timer 2 watchdog. The `analog-*` commands
//! never reset, stop or stall the core.

use probe_proto::{CpuMode, Request, Response};

use super::{
    expect_ack, read_flash, read_mem, sws_activate, unexpected, CPU_STATUS_ADDR, FLASH_HEADER_ADDR,
    FLASH_HEADER_LEN, KNLT_MAGIC, RESET_HOLD_MS, XTAL_BIT,
};
use crate::output::{FlashInfo, Output};
use crate::transport::ProbeTransport;
use crate::CliError;

/// SWire address of the core program counter.
const PC_ADDR: u16 = 0x06bc;

/// SWire address of the Timer 2 watchdog control byte.
const WATCHDOG_CTRL_ADDR: u16 = 0x0622;

/// `TMR_WD_EN`, bit 7 of [`WATCHDOG_CTRL_ADDR`].
const TMR_WD_EN: u8 = 0x80;

/// `info`: PC and CPU status, plus the flash header when the core is not
/// running (or `--halt` stops it first). A live target gets no MSPI traffic.
pub(super) fn info(transport: &mut dyn ProbeTransport, halt: bool) -> Result<Output, CliError> {
    sws_activate(transport)?;

    let pc_bytes = read_mem(transport, PC_ADDR, 4)?;
    let pc = u32::from_le_bytes([pc_bytes[0], pc_bytes[1], pc_bytes[2], pc_bytes[3]]);

    let cpu_status = read_mem(transport, CPU_STATUS_ADDR, 1)?[0];

    let flash_info = if halt || matches!(cpu_status, 0x05 | 0x06) {
        if halt {
            halt_core(transport)?;
        }
        Some(read_flash_info(transport)?)
    } else {
        None
    };

    Ok(Output::Info {
        pc,
        cpu_status,
        cpu_state: cpu_state(cpu_status),
        flash_info,
    })
}

/// Reads the JEDEC ID and firmware header, then decodes the header.
fn read_flash_info(transport: &mut dyn ProbeTransport) -> Result<FlashInfo, CliError> {
    let jedec = match transport.exchange(&Request::FlashJedecId)? {
        Response::JedecId(id) => id,
        other => return Err(unexpected("JedecId", &other)),
    };

    let header = read_flash(transport, FLASH_HEADER_ADDR, FLASH_HEADER_LEN)?;
    let version = u32::from_le_bytes([header[4], header[5], header[6], header[7]]);
    let magic = u32::from_le_bytes([header[8], header[9], header[10], header[11]]);

    Ok(FlashInfo {
        jedec_id: format!("{:02x}{:02x}{:02x}", jedec[0], jedec[1], jedec[2]),
        xtal_mhz: if version & XTAL_BIT == 0 { 12 } else { 16 },
        build_version: version & !XTAL_BIT,
        magic_ok: magic == KNLT_MAGIC,
    })
}

/// `halt`: stall the core and disable the watchdog.
///
/// The stall is terminal until `reset`.
pub(super) fn halt(transport: &mut dyn ProbeTransport) -> Result<Output, CliError> {
    sws_activate(transport)?;
    halt_core(transport)?;
    Ok(Output::Halt)
}

/// Stalls the core, then clears `TMR_WD_EN` so Timer 2 cannot reset the chip
/// while it is stalled.
fn halt_core(transport: &mut dyn ProbeTransport) -> Result<(), CliError> {
    expect_ack(
        transport,
        Request::CpuCtrl {
            mode: CpuMode::Stall,
        },
    )?;

    let watchdog_ctrl = read_mem(transport, WATCHDOG_CTRL_ADDR, 1)?[0] & !TMR_WD_EN;
    expect_ack(
        transport,
        Request::WriteMem {
            addr: WATCHDOG_CTRL_ADDR,
            data: [watchdog_ctrl].into_iter().collect(),
        },
    )
}

/// `reset`: hard reset (optionally leaving the core stopped) or a SWire soft
/// reboot.
pub(super) fn reset(
    transport: &mut dyn ProbeTransport,
    stop: bool,
    soft: bool,
) -> Result<Output, CliError> {
    if soft {
        sws_activate(transport)?;
        expect_ack(
            transport,
            Request::CpuCtrl {
                mode: CpuMode::SoftReboot,
            },
        )?;
    } else {
        expect_ack(
            transport,
            Request::Reset {
                hold_ms: RESET_HOLD_MS,
                stop_cpu: stop,
            },
        )?;
    }
    Ok(Output::Reset { stop, soft })
}

/// `analog-read`: read one analog register.
pub(super) fn analog_read(
    transport: &mut dyn ProbeTransport,
    addr: &str,
) -> Result<Output, CliError> {
    let addr = parse_byte(addr).map_err(|message| CliError::InvalidAddress { message })?;

    sws_activate(transport)?;
    let value = match transport.exchange(&Request::ReadAnalog { addr })? {
        Response::Byte(value) => value,
        other => return Err(unexpected("Byte", &other)),
    };

    Ok(Output::AnalogRead { addr, value })
}

/// `analog-write`: write one analog register.
pub(super) fn analog_write(
    transport: &mut dyn ProbeTransport,
    addr: &str,
    val: &str,
) -> Result<Output, CliError> {
    let addr = parse_byte(addr).map_err(|message| CliError::InvalidAddress { message })?;
    let value = parse_byte(val).map_err(CliError::Device)?;

    sws_activate(transport)?;
    expect_ack(transport, Request::WriteAnalog { addr, val: value })?;

    Ok(Output::AnalogWrite { addr, value })
}

/// Maps the CPU status byte to its documented state name.
fn cpu_state(status: u8) -> &'static str {
    match status {
        0x00 | 0x04 => "running",
        0x05 => "stopped",
        0x06 => "stalled",
        _ => "unknown",
    }
}

/// Parses a byte written as `0x..` hex or decimal.
fn parse_byte(text: &str) -> Result<u8, String> {
    let text = text.trim();
    let parsed = match text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        Some(hex) => u8::from_str_radix(hex, 16),
        None => text.parse(),
    };
    parsed.map_err(|_| format!("'{text}' is not a byte (0x00..=0xff)"))
}

#[cfg(test)]
mod tests;
