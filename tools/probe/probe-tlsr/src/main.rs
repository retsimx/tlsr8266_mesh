//! `probe-tlsr`: host CLI for the TLSR8266 SWire probe.
//!
//! The tool talks to a USB-CDC probe over the `probe-proto` wire format and
//! exposes the core debugging and flash primitives as subcommands. The CLI
//! types and the failure type live in [`cli`], the wire link in [`transport`],
//! the ELF helpers in [`elf`], the command handlers in [`commands`], and the
//! result/failure rendering in [`output`] and [`error`]; this file owns
//! transport construction, the command context and the process exit code.
//!
//! Transport construction happens before command-argument validation, so on a
//! host with no probe a bad argument reports a transport error first.
//!
//! # Output contract
//!
//! `--json` prints exactly one object per command on stdout, tagged with the
//! literal CLI spelling (`"info"`, `"flash-read"`, `"analog-read"`, `"halt"`,
//! `"inspect"`, …). `rtt` is the one streaming command: it prints one
//! `{"command":"rtt","line":"…"}` object per log line. Failures print one
//! object to **stderr** and leave stdout empty:
//! ```text
//! {"command":"error","exit":1,"error":"Protocol","proto_error":5,"message":"probe error: SwsTimeout"}
//! ```
//! `proto_error` is the numeric wire code from `probe-proto`, or `null` when
//! the failure is not a probe-reported protocol error. Human mode prints
//! `key=value` lines for `info`, a 16-byte-per-line hex dump for `read` and
//! `flash-read`, and one confirmation line for everything else.
//!
//! The one exception to "stdout stays empty on failure" is `rtt`: a panic line
//! or a frozen heartbeat first writes the full `inspect` report to stdout, then
//! reports the fault on stderr, so the crash is not lost behind the exit code.
//! `trap` likewise writes its hit report from the handler, before it clears the
//! slot and reboots, because a hit is a one-shot capture.
//!
//! # Core state
//!
//! `flash` without `--reset` leaves the core stopped, and an aborted flash
//! (a failed erase/page write) also leaves it stopped; run `probe-tlsr reset`
//! to start it again. The raw flash commands leave the core stopped too. `trap`
//! soft-reboots the core after a hit, and clears the armed slot before failing
//! on its wait deadline.
//!
//! # Exit codes
//! - `0` — success, including an `rtt --until` match and a `trap` hit.
//! - `1` — runtime failure (transport, protocol, ELF, I/O, address or device),
//!   plus the `rtt` stream and `trap` wait deadlines ([`CliError::Timeout`]).
//! - `2` — both the clap usage error and a frozen `rtt` `HEARTBEAT`
//!   ([`CliError::HeartbeatStalled`]); the overlap is fixed by the issue
//!   contract.
//! - `3` — a `rtt` panic-marker line ([`CliError::TargetPanic`]); the code is
//!   fixed by the issue contract.
//!
//! `--timeout-ms` is absent by default: the transport then falls back to
//! [`DEFAULT_TIMEOUT_MS`], so every existing command keeps its 5000 ms
//! behaviour. For `rtt` and `trap` the same flag is the *stream* or *wait*
//! deadline, where absent or `0` means no deadline at all.

mod cli;
mod commands;
mod elf;
mod error;
mod output;
mod transport;

pub use cli::{Cli, CliError, Command};

use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::Parser;

use output::print_output;
use transport::ProbeTransport;

/// Response timeout used when `--timeout-ms` is absent.
pub const DEFAULT_TIMEOUT_MS: u64 = 5000;

/// Everything a command handler needs beyond its own arguments.
pub struct Ctx {
    /// The probe link, real or scripted.
    pub transport: Box<dyn ProbeTransport>,
    /// The parsed `--elf` image, if one was given.
    pub elf: Option<elf::ElfImage>,
    /// Path of the `--elf` image, kept so the disassembler can re-read it.
    pub elf_path: Option<PathBuf>,
    /// Whether the result should be rendered as JSON.
    pub json: bool,
    /// Raw `--timeout-ms`; the streaming commands treat `None` or `0` as no
    /// deadline, while the transport falls back to [`DEFAULT_TIMEOUT_MS`].
    pub timeout_ms: Option<u64>,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let json = cli.json;
    let mut out = std::io::stdout();
    let mut err = std::io::stderr();
    let result = run(cli, &mut out);
    ExitCode::from(emit(result, json, &mut err))
}

/// Executes one command, writing its result to `out`.
///
/// Command failures return before anything is written, so stdout stays empty
/// on the error path; [`emit`] reports them on stderr. `rtt` and `trap` are the
/// deliberate exceptions: their report is written by the handler before it
/// fails, so a one-shot capture survives the failure.
fn run(cli: Cli, out: &mut dyn Write) -> Result<(), CliError> {
    let elf = match &cli.elf {
        Some(path) => Some(elf::ElfImage::parse(&read_file(path)?)?),
        None => None,
    };
    // `--timeout-ms 0` means "no stream deadline", which must not also turn the
    // per-response timeout into an instant timeout: the transport keeps its
    // 5000 ms fallback either way.
    let timeout_ms = cli
        .timeout_ms
        .filter(|ms| *ms > 0)
        .unwrap_or(DEFAULT_TIMEOUT_MS);
    let transport = build_transport(cli.port.as_deref(), timeout_ms)?;
    let mut ctx = Ctx {
        transport,
        elf,
        elf_path: cli.elf.clone(),
        json: cli.json,
        timeout_ms: cli.timeout_ms,
    };
    if let Some(output) = commands::run(cli.command, &mut ctx, &mut *out)? {
        print_output(&output, ctx.json, out)?;
    }
    Ok(())
}

/// Reports a failure on `err` (always stderr in `main`) and maps the result to
/// the process exit code.
fn emit(result: Result<(), CliError>, json: bool, err: &mut dyn Write) -> u8 {
    match result {
        Ok(()) => 0,
        Err(error) => {
            error::report_error(&error, json, err);
            error.exit_code()
        }
    }
}

fn read_file(path: &Path) -> Result<Vec<u8>, CliError> {
    fs::read(path).map_err(|err| CliError::Io(format!("{}: {err}", path.display())))
}

#[cfg(not(test))]
fn build_transport(
    port: Option<&Path>,
    timeout_ms: u64,
) -> Result<Box<dyn ProbeTransport>, CliError> {
    Ok(Box::new(transport::SerialTransport::open(
        port, timeout_ms,
    )?))
}

// Tests never talk to hardware; command tests build their own `MockTransport`.
#[cfg(test)]
fn build_transport(
    _port: Option<&Path>,
    _timeout_ms: u64,
) -> Result<Box<dyn ProbeTransport>, CliError> {
    Ok(Box::new(transport::MockTransport::new(Vec::new())))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    #[test]
    fn failure_reports_on_stderr_and_leaves_stdout_empty() {
        let cli = Cli {
            port: None,
            elf: None,
            json: true,
            timeout_ms: Some(1000),
            command: Command::Halt,
        };
        let mut out = Vec::new();
        let mut err = Vec::new();

        // The test transport scripts no responses, so `halt` fails.
        let code = emit(run(cli, &mut out), true, &mut err);

        assert_eq!(code, 1);
        assert!(out.is_empty(), "failure must not write to stdout");
        let text = String::from_utf8(err).expect("error report is utf-8");
        let value: Value = serde_json::from_str(text.trim_end()).expect("stderr is one object");
        assert_eq!(value["command"], "error");
        assert_eq!(value["exit"], 1);
        assert_eq!(value["error"], "Device");
        assert!(value["message"].is_string());
    }
}
