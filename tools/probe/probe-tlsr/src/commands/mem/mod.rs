//! `read` and `write`: SWire memory access by address or symbol.
//!
//! Both commands issue `SwsActivate` as their only control traffic; they never
//! reset, stop or stall the core. The whole requested span is validated
//! against one mapped window before the first request, and chunks are capped
//! so no request crosses a window boundary.

use probe_proto::{is_valid_swire_range, Request, MAX_DATA_PAYLOAD, MAX_WRITE_PAYLOAD};

use super::{expect_ack, hex_string, read_mem, sws_activate};
use crate::elf::{self, AddrSpec, ElfImage};
use crate::output::Output;
use crate::transport::ProbeTransport;
use crate::CliError;

/// `read` length when neither the command line nor the symbol provides one.
const DEFAULT_READ_LEN: u16 = 4;

/// Last address of the low SWire window (`0x0000..=0x0fff`).
const LOW_WINDOW_END: u32 = 0x0fff;

/// First address of the SRAM/register SWire window (`0x8000..=0xbfff`).
const HIGH_WINDOW_START: u32 = 0x8000;

/// Last address of the SRAM/register SWire window.
const HIGH_WINDOW_END: u32 = 0xbfff;

/// `read`: read SWire memory from a wire/physical address or a symbol.
pub(super) fn read(
    transport: &mut dyn ProbeTransport,
    elf: Option<&ElfImage>,
    spec: &str,
    len: Option<&str>,
) -> Result<Output, CliError> {
    let spec = resolve_addr(elf, spec)?;
    read_span(transport, &spec, len)
}

/// Reads `len` bytes (or the symbol size, or [`DEFAULT_READ_LEN`]) at `spec`.
fn read_span(
    transport: &mut dyn ProbeTransport,
    spec: &AddrSpec,
    len: Option<&str>,
) -> Result<Output, CliError> {
    let (wire, symbol_size) = elf::to_swire(spec)?;
    let len = match len {
        Some(text) => parse_len(text)?,
        None => match symbol_size {
            Some(size) => u16::try_from(size).map_err(|_| CliError::InvalidAddress {
                message: format!("symbol size {size} does not fit a 16-bit span"),
            })?,
            None => DEFAULT_READ_LEN,
        },
    };
    let window_end = validate_span(wire, len)?;

    sws_activate(transport)?;

    let end = u32::from(wire) + u32::from(len);
    let mut data = Vec::with_capacity(usize::from(len));
    let mut cursor = u32::from(wire);
    while cursor < end {
        let chunk = chunk_len(cursor, end, MAX_DATA_PAYLOAD, window_end);
        data.extend_from_slice(&read_mem(transport, cursor as u16, chunk as u16)?);
        cursor += chunk;
    }

    Ok(Output::Read {
        addr: wire,
        data_hex: hex_string(&data),
    })
}

/// `write`: write hex bytes to SWire memory at a wire/physical address or a
/// symbol.
///
/// A symbol with a known `st_size` bounds the payload. Like `read`, the only
/// control traffic is `SwsActivate`.
pub(super) fn write(
    transport: &mut dyn ProbeTransport,
    elf: Option<&ElfImage>,
    spec: &str,
    hex: &str,
) -> Result<Output, CliError> {
    let spec = resolve_addr(elf, spec)?;
    write_span(transport, &spec, hex)
}

/// Writes the payload parsed from `hex` at `spec`.
fn write_span(
    transport: &mut dyn ProbeTransport,
    spec: &AddrSpec,
    hex: &str,
) -> Result<Output, CliError> {
    let (wire, symbol_size) = elf::to_swire(spec)?;
    let data = parse_hex_bytes(hex)?;

    if let Some(size) = symbol_size {
        if data.len() > size as usize {
            return Err(CliError::Device(format!(
                "payload is {} bytes but the symbol holds {size}",
                data.len()
            )));
        }
    }

    let len = u16::try_from(data.len()).map_err(|_| CliError::InvalidAddress {
        message: format!("{} bytes do not fit the SWire windows", data.len()),
    })?;
    let window_end = validate_span(wire, len)?;

    sws_activate(transport)?;

    let end = u32::from(wire) + u32::from(len);
    let mut cursor = u32::from(wire);
    while cursor < end {
        let chunk = chunk_len(cursor, end, MAX_WRITE_PAYLOAD, window_end) as usize;
        let start = (cursor - u32::from(wire)) as usize;
        expect_ack(
            transport,
            Request::WriteMem {
                addr: cursor as u16,
                data: data[start..start + chunk].iter().copied().collect(),
            },
        )?;
        cursor += chunk as u32;
    }

    Ok(Output::Write {
        addr: wire,
        len: data.len(),
    })
}

/// Resolves `spec` with the ELF image when one was given.
///
/// Without `--elf` there is no symbol table, so only the numeric forms `0x...`
/// and decimal are accepted; anything else needs an ELF.
pub(super) fn resolve_addr(elf: Option<&ElfImage>, spec: &str) -> Result<AddrSpec, CliError> {
    if let Some(image) = elf {
        return image.parse_addr(spec);
    }
    if let Some(hex) = spec.strip_prefix("0x").or_else(|| spec.strip_prefix("0X")) {
        return u64::from_str_radix(hex, 16)
            .map(AddrSpec::Wire)
            .map_err(|_| CliError::InvalidAddress {
                message: format!("`{spec}` is not a hex address"),
            });
    }
    spec.parse::<u64>()
        .map(AddrSpec::Numeric)
        .map_err(|_| CliError::MissingElf)
}

/// Parses a byte count written as `0x..` hex or decimal.
fn parse_len(text: &str) -> Result<u16, CliError> {
    let text = text.trim();
    let parsed = match text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        Some(hex) => u16::from_str_radix(hex, 16),
        None => text.parse(),
    };
    parsed.map_err(|_| CliError::Device(format!("`{text}` is not a byte count")))
}

/// Parses hex bytes, tolerating whitespace, `_` separators and a `0x` prefix
/// on every group (`"0xde 0xad_be ef"`).
fn parse_hex_bytes(text: &str) -> Result<Vec<u8>, CliError> {
    let mut nibbles = Vec::new();
    for group in text.split(|ch: char| ch.is_whitespace() || ch == '_') {
        let group = group
            .strip_prefix("0x")
            .or_else(|| group.strip_prefix("0X"))
            .unwrap_or(group);
        if group.is_empty() {
            continue;
        }
        for ch in group.chars() {
            let nibble = ch
                .to_digit(16)
                .ok_or_else(|| CliError::Device(format!("`{group}` is not hexadecimal")))?;
            nibbles.push(nibble as u8);
        }
    }

    if nibbles.is_empty() {
        return Err(CliError::Device("write payload is empty".to_owned()));
    }
    if !nibbles.len().is_multiple_of(2) {
        return Err(CliError::Device(
            "write payload has an odd number of hex digits".to_owned(),
        ));
    }

    Ok(nibbles
        .as_chunks::<2>()
        .0
        .iter()
        .map(|pair| (pair[0] << 4) | pair[1])
        .collect())
}

/// Last address of the mapped window containing `addr`, if any.
fn window_end(addr: u16) -> Option<u32> {
    let addr = u32::from(addr);
    if addr <= LOW_WINDOW_END {
        Some(LOW_WINDOW_END)
    } else if (HIGH_WINDOW_START..=HIGH_WINDOW_END).contains(&addr) {
        Some(HIGH_WINDOW_END)
    } else {
        None
    }
}

/// Requires the whole `wire..wire + len` span to sit in one mapped window and
/// returns that window's last address.
fn validate_span(wire: u16, len: u16) -> Result<u32, CliError> {
    let window_end = window_end(wire).ok_or_else(|| CliError::InvalidAddress {
        message: format!("0x{wire:04x} is outside the SWire windows"),
    })?;
    if !is_valid_swire_range(wire, len) {
        return Err(CliError::InvalidAddress {
            message: format!("span 0x{wire:04x}+{len} is empty or crosses a SWire window boundary"),
        });
    }
    Ok(window_end)
}

/// Next chunk size: the request cap, the remaining span or the window end,
/// whichever is smallest.
fn chunk_len(cursor: u32, end: u32, cap: usize, window_end: u32) -> u32 {
    (end - cursor).min(cap as u32).min(window_end + 1 - cursor)
}

#[cfg(test)]
mod tests;
