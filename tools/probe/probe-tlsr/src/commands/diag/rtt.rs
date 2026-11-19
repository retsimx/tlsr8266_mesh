//! `rtt`: stream the target's RTT ring, watching for a panic or a stuck loop.
//!
//! The probe advances the target's `rd` index itself, so the host only polls the
//! control block and appends whatever arrives to a line buffer; it never writes
//! through the ring. Every poll is followed by a `HEARTBEAT` read: a value that
//! stops changing means the main loop is no longer turning, which is the only
//! evidence of a hang the target offers short of a watchdog reset.
//!
//! A panic line or a frozen heartbeat stalls the core through `inspect`, writes
//! the full report to stdout *before* the failure object on stderr, and returns
//! [`CliError::TargetPanic`] (exit 3) or [`CliError::HeartbeatStalled`]
//! (exit 2). The stall is terminal: the documented recovery is
//! `probe-tlsr reset --soft`.

use std::io::Write;
use std::path::Path;
use std::time::{Duration, Instant};

use probe_proto::{Request, Response};

use super::inspect_report;
use crate::commands::{read_word, sws_activate, unexpected};
use crate::elf::{to_swire, AddrSpec, ElfImage};
use crate::output::{print_output, Output};
use crate::transport::ProbeTransport;
use crate::{CliError, Ctx};

/// `.symtab` name of the RTT control block.
pub(crate) const RTT_SYMBOL: &str = "_TLSR_RTT";

/// `.symtab` name of the main-loop heartbeat counter.
pub(crate) const HEARTBEAT_SYMBOL: &str = "HEARTBEAT";

/// Delay between one drain/heartbeat pass and the next.
const RTT_POLL_MS: u64 = 50;

/// Longest line buffered before it is flushed with no newline.
const RTT_LINE_MAX: usize = 4096;

/// Default `--watchdog-ms`: a heartbeat that stops moving for this long is a
/// hang.
pub(crate) const DEFAULT_WATCHDOG_MS: u64 = 1000;

/// Substrings that mark a firmware panic line.
const PANIC_MARKERS: [&str; 2] = ["[PANIC]", "panicked at"];

/// One stream's options plus the addresses resolved from the ELF.
///
/// The fields are `pub(super)` so the diagnostics tests can drive the loop
/// without building a whole [`Ctx`]; production code builds this from the CLI.
pub(super) struct Config<'a> {
    /// SWire address of the RTT control block.
    pub(super) ctrl_addr: u16,
    /// SWire address of the 4-byte heartbeat counter.
    pub(super) heartbeat: u16,
    /// Stop after the first emitted line containing this substring.
    pub(super) until: Option<&'a str>,
    /// Fail once the heartbeat is unchanged for this many milliseconds; `0`
    /// disables the watchdog.
    pub(super) watchdog_ms: u64,
    /// Raw `--timeout-ms`; `None` or `0` means no stream deadline.
    pub(super) timeout_ms: Option<u64>,
    /// Emit one JSON object per line instead of the raw text.
    pub(super) json: bool,
}

/// `rtt`: resolve the symbols, then stream at the production cadence.
pub(crate) fn run(
    ctx: &mut Ctx,
    out: &mut dyn Write,
    until: Option<&str>,
    watchdog_ms: u64,
) -> Result<(), CliError> {
    let config = config(ctx, until, watchdog_ms)?;
    stream(
        ctx.transport.as_mut(),
        out,
        ctx.elf.as_ref(),
        ctx.elf_path.as_deref(),
        &config,
        Duration::from_millis(RTT_POLL_MS),
    )
}

/// Resolves `_TLSR_RTT` and `HEARTBEAT` before any request is sent.
///
/// Both symbols are required: without the control block there is nothing to
/// read, and without the heartbeat the watchdog cannot work. A symbol missing
/// from `.symtab` is a misconfigured `--elf`, not a degradation.
///
/// A missing `--elf` names both symbols and the flag: the shared
/// [`CliError::MissingElf`] text describes a user symbol lookup, which is not
/// what `rtt` does.
fn config<'a>(ctx: &Ctx, until: Option<&'a str>, watchdog_ms: u64) -> Result<Config<'a>, CliError> {
    let elf = ctx.elf.as_ref().ok_or_else(|| {
        CliError::Elf(format!(
            "rtt needs --elf <path>: `{RTT_SYMBOL}` and `{HEARTBEAT_SYMBOL}` are resolved from its .symtab"
        ))
    })?;
    Ok(Config {
        ctrl_addr: symbol_wire(elf, RTT_SYMBOL)?,
        heartbeat: symbol_wire(elf, HEARTBEAT_SYMBOL)?,
        until,
        watchdog_ms,
        timeout_ms: ctx.timeout_ms,
        json: ctx.json,
    })
}

/// Resolves one required `.symtab` symbol to its SWire address.
pub(super) fn symbol_wire(elf: &ElfImage, name: &str) -> Result<u16, CliError> {
    let (value, size) = elf.lookup_symbol(name).ok_or_else(|| {
        CliError::Elf(format!(
            "symbol `{name}` is missing from the ELF .symtab; rtt cannot work without it"
        ))
    })?;
    to_swire(&AddrSpec::Symbol { value, size }).map(|(wire, _)| wire)
}

/// Streams until `--until` matches, a fault is reported, or the deadline ends.
///
/// `poll_interval` is a parameter so tests can drive a 1 ms cadence; production
/// code uses [`RTT_POLL_MS`].
pub(super) fn stream(
    transport: &mut dyn ProbeTransport,
    out: &mut dyn Write,
    elf: Option<&ElfImage>,
    elf_path: Option<&Path>,
    config: &Config<'_>,
    poll_interval: Duration,
) -> Result<(), CliError> {
    // An unrepresentable instant means "no deadline" rather than a panic: the
    // value comes straight from `--timeout-ms`.
    let deadline = config.timeout_ms.filter(|ms| *ms > 0).and_then(|ms| {
        Instant::now()
            .checked_add(Duration::from_millis(ms))
            .map(|at| Deadline { at, ms })
    });
    sws_activate(transport)?;

    let mut line = Vec::with_capacity(RTT_LINE_MAX);
    let mut heartbeat: Option<u32> = None;
    let mut beat_at = Instant::now();

    loop {
        // 1. Drain the ring and emit every complete line.
        drain(transport, config.ctrl_addr, &mut line)?;
        while let Some(end) = line.iter().position(|byte| *byte == b'\n') {
            let text = take_line(&mut line, end + 1);
            if emit_line(transport, out, elf, elf_path, config, &text)? {
                return Ok(());
            }
        }
        // A line that never ends must not grow without bound; flush it as a line
        // of its own (the tail of a real line beats an unbounded buffer).
        if line.len() >= RTT_LINE_MAX {
            let end = line.len();
            let text = take_line(&mut line, end);
            if emit_line(transport, out, elf, elf_path, config, &text)? {
                return Ok(());
            }
        }

        // 2. The heartbeat: unchanged for long enough means the loop is dead.
        let beat = read_word(transport, config.heartbeat)?;
        if heartbeat == Some(beat) {
            if config.watchdog_ms > 0
                && beat_at.elapsed() >= Duration::from_millis(config.watchdog_ms)
            {
                let error = CliError::HeartbeatStalled {
                    last: beat,
                    watchdog_ms: config.watchdog_ms,
                };
                return Err(fault(transport, out, elf, elf_path, config, error));
            }
        } else {
            heartbeat = Some(beat);
            beat_at = Instant::now();
        }

        // 3. Deadline, then sleep so expiry is accurate.
        let Some(deadline) = deadline else {
            std::thread::sleep(poll_interval);
            continue;
        };
        let remaining = deadline.remaining();
        if remaining.is_zero() {
            // A tail with no newline is still the last thing the target said,
            // so it is emitted before the deadline is reported. It goes through
            // the same per-line policy as a complete line: an `--until` match
            // or a panic marker wins over the bare timeout.
            if !line.is_empty() {
                let end = line.len();
                let text = take_line(&mut line, end);
                if emit_line(transport, out, elf, elf_path, config, &text)? {
                    return Ok(());
                }
            }
            return Err(CliError::Timeout(format!(
                "rtt: stream deadline of {} ms expired; no --until match",
                deadline.ms
            )));
        }
        std::thread::sleep(remaining.min(poll_interval));
    }
}

/// One `RttPoll`: the probe advances `rd` and returns the drained bytes.
fn drain(
    transport: &mut dyn ProbeTransport,
    ctrl_addr: u16,
    line: &mut Vec<u8>,
) -> Result<(), CliError> {
    match transport.exchange(&Request::RttPoll { ctrl_addr })? {
        Response::Data(data) => {
            line.extend_from_slice(&data);
            Ok(())
        }
        other => Err(unexpected("Data", &other)),
    }
}

/// Emits one line, then applies `--until` and the panic markers in that order.
///
/// Returns `true` when `--until` matched and the stream is done.
fn emit_line(
    transport: &mut dyn ProbeTransport,
    out: &mut dyn Write,
    elf: Option<&ElfImage>,
    elf_path: Option<&Path>,
    config: &Config<'_>,
    text: &str,
) -> Result<bool, CliError> {
    emit(out, config.json, text)?;
    if config.until.is_some_and(|needle| text.contains(needle)) {
        return Ok(true);
    }
    if PANIC_MARKERS.iter().any(|&marker| text.contains(marker)) {
        let error = CliError::TargetPanic {
            line: text.to_owned(),
        };
        return Err(fault(transport, out, elf, elf_path, config, error));
    }
    Ok(false)
}

/// Writes one line through the shared renderer, so `--json` and human mode
/// agree with the output table.
fn emit(out: &mut dyn Write, json: bool, line: &str) -> Result<(), CliError> {
    let output = Output::RttLine {
        line: line.to_owned(),
    };
    print_output(&output, json, out)
}

/// Stalls the core, writes the inspect report, then returns `error`.
///
/// A failing inspect is returned instead: the probe, not the target, is then
/// the thing that broke, and a silently missing report would hide it.
fn fault(
    transport: &mut dyn ProbeTransport,
    out: &mut dyn Write,
    elf: Option<&ElfImage>,
    elf_path: Option<&Path>,
    config: &Config<'_>,
    error: CliError,
) -> CliError {
    let report = match inspect_report(transport, elf, elf_path) {
        Ok(report) => report,
        Err(inspect_error) => return inspect_error,
    };
    let output = Output::Inspect {
        report: Box::new(report),
    };
    match print_output(&output, config.json, out) {
        Ok(()) => error,
        Err(write_error) => write_error,
    }
}

/// Removes and decodes the first `end` bytes, dropping one trailing newline.
///
/// A `\r` before the `\n` is stripped too, so CRLF output stays readable.
fn take_line(line: &mut Vec<u8>, end: usize) -> String {
    let mut bytes: Vec<u8> = line.drain(..end).collect();
    if bytes.last() == Some(&b'\n') {
        bytes.pop();
    }
    if bytes.last() == Some(&b'\r') {
        bytes.pop();
    }
    String::from_utf8_lossy(&bytes).into_owned()
}

/// A `--timeout-ms` stream deadline and the value it was set to.
#[derive(Clone, Copy)]
struct Deadline {
    /// Instant the stream gives up.
    at: Instant,
    /// `--timeout-ms` as given, for the error message.
    ms: u64,
}

impl Deadline {
    /// Time left before the deadline; zero once it has passed.
    fn remaining(&self) -> Duration {
        self.at.saturating_duration_since(Instant::now())
    }
}
