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

use probe_proto::{ProtoError, Request, Response};

use super::inspect_report;
use crate::commands::{read_mem, read_word, sws_activate, unexpected};
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

/// Offset of the `flags` word inside `_TLSR_RTT`; bit 0 is sticky overflow.
const RTT_FLAGS_OFFSET: u16 = 14;

/// Sticky bit 0 of the RTT `flags` word.
const RTT_FLAG_OVERFLOW: u16 = 1;

/// One-line warning emitted when the target dropped log bytes before attach.
const RTT_OVERFLOW_WARNING: &str =
    "[probe] target RTT ring overflowed before this stream; some log lines were lost";

/// How long a probe/target that is rebooting may fail polls before the stream
/// gives up.
///
/// A panic or watchdog reset leaves `_TLSR_RTT` uninitialised for the first few
/// milliseconds of the next boot, and the probe answers `InvalidArg` until the
/// firmware writes the magic. Treating that as fatal would lose the `[PANIC]`
/// line that the reboot was about to produce.
const RTT_READY_GRACE_MS: u64 = 2000;

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
    report_start_overflow(transport, out, config)?;

    let mut line = Vec::with_capacity(RTT_LINE_MAX);
    let mut heartbeat: Option<u32> = None;
    let mut beat_at = Instant::now();
    // Start of the current run of transient poll failures (target rebooting).
    let mut not_ready_since: Option<Instant> = None;

    loop {
        // 0. Deadline first, so a reboot grace period cannot outlive it and a
        // final unterminated line is still reported.
        if let Some(deadline) = deadline {
            if deadline.remaining().is_zero() {
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
        }

        // 1. Drain the ring and emit every complete line. A target that is
        // mid-reboot answers `InvalidArg` until `rtt_init()` writes the magic;
        // retry for a bounded grace period instead of losing the panic report.
        match drain(transport, config.ctrl_addr, &mut line) {
            Ok(()) => recovered(&mut not_ready_since, &mut heartbeat, &mut beat_at),
            Err(error) if transient(&error) => {
                if not_ready_since.is_none() {
                    // The target rebooted mid-line; pre- and post-reboot bytes
                    // must not be concatenated into one garbled line.
                    line.clear();
                }
                if !transient_pause(deadline, poll_interval, &mut not_ready_since) {
                    return Err(error);
                }
                continue;
            }
            Err(error) => return Err(error),
        }
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
        let beat = match read_word(transport, config.heartbeat) {
            Ok(beat) => {
                recovered(&mut not_ready_since, &mut heartbeat, &mut beat_at);
                beat
            }
            Err(error) if transient(&error) => {
                if not_ready_since.is_none() {
                    line.clear();
                }
                if !transient_pause(deadline, poll_interval, &mut not_ready_since) {
                    return Err(error);
                }
                continue;
            }
            Err(error) => return Err(error),
        };
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

        // 3. Sleep at most the remaining time so expiry is accurate.
        let sleep_for = deadline.map_or(poll_interval, |at| at.remaining().min(poll_interval));
        std::thread::sleep(sleep_for);
    }
}

/// Whether a poll failure is the target rebooting rather than a real fault.
pub(super) fn transient(error: &CliError) -> bool {
    matches!(
        error,
        CliError::Protocol(ProtoError::InvalidArg | ProtoError::SwsTimeout)
            | CliError::Transport(_)
    )
}

/// Emits one warning when the target's sticky overflow flag is already set.
///
/// The flag is sticky and survives until the target reboots, so checking once
/// on attach catches the common case: firmware logged more than the 256 B ring
/// before the host connected. A failure to read it is ignored; the log stream
/// itself is the feature, not this diagnostic.
fn report_start_overflow(
    transport: &mut dyn ProbeTransport,
    out: &mut dyn Write,
    config: &Config<'_>,
) -> Result<(), CliError> {
    let Some(addr) = config.ctrl_addr.checked_add(RTT_FLAGS_OFFSET) else {
        return Ok(());
    };
    let Ok(flags) = read_mem(transport, addr, 2) else {
        return Ok(());
    };
    if u16::from_le_bytes([flags[0], flags[1]]) & RTT_FLAG_OVERFLOW != 0 {
        emit(out, config.json, RTT_OVERFLOW_WARNING)?;
    }
    Ok(())
}

/// Bounds one uninterrupted run of transient failures to
/// [`RTT_READY_GRACE_MS`]. A success ends the run, so a target that alternates
/// success and failure is allowed to keep making progress until its deadline.
pub(super) fn within_grace(since: &mut Option<Instant>) -> bool {
    let start = *since.get_or_insert_with(Instant::now);
    start.elapsed() < Duration::from_millis(RTT_READY_GRACE_MS)
}

/// Sleeps one poll while a transient run is inside its grace window; `false`
/// means the run exceeded it and the caller must report the original error.
fn transient_pause(
    deadline: Option<Deadline>,
    poll_interval: Duration,
    since: &mut Option<Instant>,
) -> bool {
    if !within_grace(since) {
        return false;
    }
    let sleep_for = deadline.map_or(poll_interval, |at| at.remaining().min(poll_interval));
    std::thread::sleep(sleep_for);
    true
}

/// Re-arms the heartbeat baseline after a transient run.
///
/// A reboot resets `HEARTBEAT`; comparing a post-reboot value against the
/// pre-reboot baseline would report a healthy boot as a frozen loop.
fn recovered(since: &mut Option<Instant>, heartbeat: &mut Option<u32>, beat_at: &mut Instant) {
    if since.take().is_some() {
        *heartbeat = None;
        *beat_at = Instant::now();
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
