//! Human rendering: `key=value` lines, hex dumps and the inspect block.
//!
//! JSON goes through `serde` on [`Output`] itself; this module only owns the
//! text form, so `--json` and human mode cannot drift apart in shape.

use std::io::Write;

use super::inspect::inspect_text;
use super::Output;
use crate::CliError;

/// Writes the command result to `out`: one JSON object or a human block.
pub fn print_output(output: &Output, json: bool, out: &mut dyn Write) -> Result<(), CliError> {
    let text = if json {
        serde_json::to_string(output).map_err(|err| CliError::Io(format!("json: {err}")))?
    } else {
        human_output(output)
    };
    writeln!(out, "{text}").map_err(|err| CliError::Io(format!("stdout: {err}")))
}

/// Human rendering: `key=value` lines for `info`, a hex dump for memory reads,
/// one confirmation line for everything else.
pub(super) fn human_output(output: &Output) -> String {
    match output {
        Output::Info {
            pc,
            cpu_status,
            cpu_state,
            flash_info,
        } => {
            let mut text =
                format!("pc=0x{pc:08x} cpu_status=0x{cpu_status:02x} cpu_state={cpu_state}");
            if let Some(flash) = flash_info {
                text.push_str(&format!(
                    "\nflash jedec_id={} build_version={} magic_ok={}",
                    flash.jedec_id, flash.build_version, flash.magic_ok
                ));
            }
            text
        }
        Output::Flash {
            path,
            bytes,
            sectors_erased,
            pages_written,
            ..
        } => format!(
            "flashed {bytes} bytes from {path}: {sectors_erased} sectors, {pages_written} pages"
        ),
        Output::FlashRead {
            addr,
            len,
            out,
            data_hex,
        } => match (out, data_hex) {
            (Some(out), _) => format!("read {len} bytes at 0x{addr:06x} into {out}"),
            (None, Some(hex)) => hex_dump(*addr, hex, 6),
            (None, None) => format!("read {len} bytes at 0x{addr:06x}"),
        },
        Output::FlashErase { addr, len, sectors } => {
            format!("erased {sectors} sectors covering 0x{addr:06x}+0x{len:x}")
        }
        Output::FlashWrite { addr, len, pages } => {
            format!("wrote {len} bytes at 0x{addr:06x} in {pages} pages")
        }
        Output::Read { addr, data_hex } => hex_dump(u32::from(*addr), data_hex, 4),
        Output::Write { addr, len } => format!("wrote {len} bytes at 0x{addr:04x}"),
        Output::AnalogRead { addr, value } => format!("analog 0x{addr:02x} = 0x{value:02x}"),
        Output::AnalogWrite { addr, value } => format!("analog 0x{addr:02x} <- 0x{value:02x}"),
        Output::Reset { stop, soft } => format!("reset (stop={stop} soft={soft})"),
        // Human-only, `inspect`-only: `trap` reuses `inspect_text` but always
        // soft-reboots on a hit, and the pinned JSON shape must not change.
        Output::Inspect { report } => format!(
            "{}\ncore stalled by inspect — run `probe-tlsr reset --soft` to restart it",
            inspect_text(report)
        ),
        Output::Trap {
            addr,
            slot,
            inspect,
            ..
        } => format!(
            "trap hit slot={slot} addr=0x{addr:06x}\n{}",
            inspect_text(inspect)
        ),
        Output::RttLine { line } => line.clone(),
        Output::Halt => "halted".to_owned(),
    }
}

/// Renders contiguous lower-case hex as 16 bytes per line, each prefixed with
/// its byte address in `width` hex digits.
fn hex_dump(base: u32, hex: &str, width: usize) -> String {
    let mut text = String::new();
    for (line, chunk) in hex.as_bytes().chunks(32).enumerate() {
        if line > 0 {
            text.push('\n');
        }
        let addr = base + (line * 16) as u32;
        text.push_str(&format!("0x{addr:0width$x}:", width = width));
        for pair in chunk.chunks(2) {
            text.push(' ');
            text.push_str(std::str::from_utf8(pair).unwrap_or("??"));
        }
    }
    text
}
