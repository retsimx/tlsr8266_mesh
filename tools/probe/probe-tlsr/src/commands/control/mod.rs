//! `info`, `halt`, `reset` and the `analog-*` pair.
//!
//! `info` samples the core before deciding whether to touch MSPI, and only
//! `halt`/`info --halt` clear the Timer 2 watchdog. The `analog-*` commands
//! never reset, stop or stall the core.

use probe_proto::{CpuMode, Request, Response};

use super::{
    expect_ack, mark_clean_reboot, read_analog_value, read_flash, read_mem, read_word, stall_core,
    sws_activate, unexpected, CPU_STATUS_ADDR, FLASH_HEADER_ADDR, FLASH_HEADER_LEN,
    HEADER_VERSION_OFFSET, KNLT_MAGIC, RESET_HOLD_MS,
};
use crate::output::{FlashInfo, Output};
use crate::transport::ProbeTransport;
use crate::CliError;

/// SWire address of the core program counter.
const PC_ADDR: u16 = 0x06bc;

/// `info`: PC and CPU status, plus the flash header when the core is not
/// running (or `--halt` stops it first). A live target gets no MSPI traffic.
pub(super) fn info(transport: &mut dyn ProbeTransport, halt: bool) -> Result<Output, CliError> {
    sws_activate(transport)?;

    let pc = read_word(transport, PC_ADDR)?;

    let cpu_status = read_mem(transport, CPU_STATUS_ADDR, 1)?[0];

    let flash_info = if halt || matches!(cpu_status, 0x05 | 0x06) {
        if halt {
            stall_core(transport)?;
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
    let version = u32::from_le_bytes(
        header[HEADER_VERSION_OFFSET..HEADER_VERSION_OFFSET + 4]
            .try_into()
            .expect("header is exactly FLASH_HEADER_LEN bytes"),
    );
    let magic = u32::from_le_bytes([header[8], header[9], header[10], header[11]]);

    Ok(FlashInfo {
        jedec_id: format!("{:02x}{:02x}{:02x}", jedec[0], jedec[1], jedec[2]),
        build_version: version & 0x7fff_ffff,
        magic_ok: magic == KNLT_MAGIC,
    })
}

/// `halt`: stall the core and disable the watchdog.
///
/// The stall is terminal until `reset`.
pub(super) fn halt(transport: &mut dyn ProbeTransport) -> Result<Output, CliError> {
    sws_activate(transport)?;
    stall_core(transport)?;
    Ok(Output::Halt)
}

/// `reset`: hard reset (optionally leaving the core stopped) or a SWire soft
/// reboot.
///
/// A soft reboot (`reset --soft`) writes the clean sentinel before rebooting,
/// because the analog register retains the previous `Running` value and the
/// next boot would otherwise report a watchdog reset. A pending `Panic`
/// sentinel is preserved so the panic is still reported. A hard `reset` sends
/// no SWire write: the reset pulse clears the analog registers in hardware.
pub(super) fn reset(
    transport: &mut dyn ProbeTransport,
    stop: bool,
    soft: bool,
) -> Result<Output, CliError> {
    if soft {
        if !mark_clean_reboot(transport) {
            sws_activate(transport)?;
        }
        expect_ack(
            transport,
            Request::CpuCtrl {
                mode: CpuMode::SoftReboot,
            },
        )?;
    } else {
        // A GP14 hard reset clears the analog sentinel itself, so there is
        // nothing to mark and no added latency on the recovery path.
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
    let value = read_analog_value(transport, addr)?;

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
        0x00 | 0x04 | 0x08 => "running",
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
