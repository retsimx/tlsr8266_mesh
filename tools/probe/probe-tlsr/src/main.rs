//! `probe-tlsr`: host CLI for the TLSR8266 SWire probe.
//!
//! The tool talks to a USB-CDC probe over the `probe-proto` wire format and
//! exposes the core debugging and flash primitives as subcommands. The wire
//! link lives in [`transport`], the ELF helpers in [`elf`], the command
//! handlers in [`commands`], and the result/failure rendering in [`output`]
//! and [`error`]; this file owns argument parsing, the error type, transport
//! construction and the process exit code.
//!
//! Transport construction happens before command-argument validation, so on a
//! host with no probe a bad argument reports a transport error first.
//!
//! # Output contract
//!
//! `--json` prints exactly one object per command on stdout, tagged with the
//! literal CLI spelling (`"info"`, `"flash-read"`, `"analog-read"`, `"halt"`,
//! …). Failures print one object to **stderr** and leave stdout empty:
//! ```text
//! {"command":"error","exit":1,"error":"Protocol","proto_error":5,"message":"probe error: SwsTimeout"}
//! ```
//! `proto_error` is the numeric wire code from `probe-proto`, or `null` when
//! the failure is not a probe-reported protocol error. Human mode prints
//! `key=value` lines for `info`, a 16-byte-per-line hex dump for `read` and
//! `flash-read`, and one confirmation line for everything else.
//!
//! # Core state
//!
//! `flash` without `--reset` leaves the core stopped, and an aborted flash
//! (crystal mismatch or a failed erase/page write) also leaves it stopped; run
//! `probe-tlsr reset` to start it again. The raw flash commands leave the core
//! stopped too.
//!
//! # Exit codes
//! - `0` — success.
//! - `1` — runtime failure (transport, protocol, ELF, I/O, address or device).
//! - `2` — command-line usage error, reported by clap itself.
//! - `3` — crystal mismatch; the image was not erased or written.

mod commands;
mod elf;
mod error;
mod output;
mod transport;

use std::fmt;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{Parser, Subcommand};
use probe_proto::ProtoError;

use output::print_output;
use transport::ProbeTransport;

/// Options shared by every subcommand.
#[derive(Debug, Parser)]
#[command(
    name = "probe-tlsr",
    version,
    about = "TLSR8266 SWire probe host CLI",
    after_help = "The probe port is opened before arguments are validated, so a missing probe is\n\
                  reported before a bad argument. `flash` without --reset and `halt` leave the\n\
                  core stopped; run `reset` to restart it."
)]
pub struct Cli {
    /// Serial port of the probe; auto-detected when omitted.
    #[arg(long, global = true, value_name = "PATH")]
    pub port: Option<PathBuf>,
    /// ELF image used for symbol lookups.
    #[arg(long, global = true, value_name = "PATH")]
    pub elf: Option<PathBuf>,
    /// Print one JSON object per command on stdout.
    #[arg(long, global = true)]
    pub json: bool,
    /// Response timeout in milliseconds.
    #[arg(long, global = true, value_name = "MS", default_value_t = 5000)]
    pub timeout_ms: u64,
    #[command(subcommand)]
    pub command: Command,
}

/// Probe commands; the clap names are the CLI spellings.
#[derive(Debug, Subcommand)]
pub enum Command {
    /// Report PC, CPU status and, when stopped, the firmware header.
    Info {
        /// Halt the core before reading the flash header.
        #[arg(long)]
        halt: bool,
    },
    /// Write an ELF or flat binary image to flash.
    Flash {
        /// Image to write.
        path: PathBuf,
        /// Skip the crystal-mismatch guard.
        #[arg(long)]
        force: bool,
        /// Reset the target after writing (without it the core stays stopped).
        #[arg(long)]
        reset: bool,
    },
    /// Read a flash span, optionally saving it to a file.
    #[command(name = "flash-read")]
    FlashRead {
        /// Flash byte address (hex or decimal).
        addr: String,
        /// Byte count.
        len: String,
        /// Write the raw bytes to this file.
        #[arg(long, value_name = "FILE")]
        out: Option<PathBuf>,
    },
    /// Erase the 4 KB sectors covering a flash span.
    #[command(name = "flash-erase")]
    FlashErase {
        /// Flash byte address (hex or decimal).
        addr: String,
        /// Byte count.
        len: String,
    },
    /// Program a file at a flash address.
    #[command(name = "flash-write")]
    FlashWrite {
        /// Flash byte address (hex or decimal).
        addr: String,
        /// File to write.
        file: PathBuf,
    },
    /// Read SWire memory by address or symbol.
    Read {
        /// Address (`0x...` or decimal) or symbol name.
        addr_or_symbol: String,
        /// Byte count; defaults to the symbol size, else 4.
        len: Option<String>,
    },
    /// Write hex bytes to SWire memory by address or symbol.
    Write {
        /// Address (`0x...` or decimal) or symbol name.
        addr_or_symbol: String,
        /// Hex bytes; whitespace, `0x` and `_` are tolerated.
        hex: String,
    },
    /// Read an analog register.
    #[command(name = "analog-read")]
    AnalogRead {
        /// Analog register address.
        addr: String,
    },
    /// Write an analog register.
    #[command(name = "analog-write")]
    AnalogWrite {
        /// Analog register address.
        addr: String,
        /// Value to write.
        val: String,
    },
    /// Reset the target core.
    Reset {
        /// Leave the core stopped after the reset.
        #[arg(long, conflicts_with = "soft")]
        stop: bool,
        /// Soft-reboot through the SWire interface.
        #[arg(long)]
        soft: bool,
    },
    /// Stall the core and disable the watchdog; only `reset` restarts it.
    Halt,
}

/// Every failure a command can report, plus its exit code.
#[derive(Debug, PartialEq, Eq)]
pub enum CliError {
    Transport(String),
    Protocol(ProtoError),
    Elf(String),
    Io(String),
    InvalidAddress { message: String },
    Device(String),
    CrystalMismatch { target_mhz: u8, image_mhz: u8 },
    MissingElf,
}

impl CliError {
    /// Process exit code: `3` for the one safety-relevant abort, else `1`.
    ///
    /// Usage errors never reach this: clap exits with `2` while parsing.
    pub fn exit_code(&self) -> u8 {
        match self {
            Self::CrystalMismatch { .. } => 3,
            _ => 1,
        }
    }

    /// Stable error kind used by the JSON error object.
    pub fn kind(&self) -> &'static str {
        match self {
            Self::Transport(_) => "Transport",
            Self::Protocol(_) => "Protocol",
            Self::Elf(_) => "Elf",
            Self::Io(_) => "Io",
            Self::InvalidAddress { .. } => "InvalidAddress",
            Self::Device(_) => "Device",
            Self::CrystalMismatch { .. } => "CrystalMismatch",
            Self::MissingElf => "MissingElf",
        }
    }

    /// Wire code of the underlying protocol error, when there is one.
    pub fn proto_error(&self) -> Option<u8> {
        match self {
            Self::Protocol(err) => Some(err.code()),
            _ => None,
        }
    }
}

impl fmt::Display for CliError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Transport(message) => write!(f, "transport: {message}"),
            Self::Protocol(err) => write!(f, "probe error: {err:?}"),
            Self::Elf(message) => write!(f, "elf: {message}"),
            Self::Io(message) => write!(f, "io: {message}"),
            Self::InvalidAddress { message } => write!(f, "invalid address: {message}"),
            Self::Device(message) => f.write_str(message),
            Self::CrystalMismatch {
                target_mhz,
                image_mhz,
            } => write!(
                f,
                "crystal mismatch: target {target_mhz} MHz, image {image_mhz} MHz (use --force to flash anyway); target left stopped — run `probe-tlsr reset`"
            ),
            Self::MissingElf => f.write_str("symbol lookup needs --elf <path>"),
        }
    }
}

impl std::error::Error for CliError {}

/// Everything a command handler needs beyond its own arguments.
pub struct Ctx {
    /// The probe link, real or scripted.
    pub transport: Box<dyn ProbeTransport>,
    /// The parsed `--elf` image, if one was given.
    pub elf: Option<elf::ElfImage>,
    /// Whether the result should be rendered as JSON.
    pub json: bool,
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
/// on the error path; [`emit`] reports them on stderr.
fn run(cli: Cli, out: &mut dyn Write) -> Result<(), CliError> {
    let elf = match &cli.elf {
        Some(path) => Some(elf::ElfImage::parse(&read_file(path)?)?),
        None => None,
    };
    let transport = build_transport(cli.port.as_deref(), cli.timeout_ms)?;
    let mut ctx = Ctx {
        transport,
        elf,
        json: cli.json,
    };
    let output = commands::run(cli.command, &mut ctx)?;
    print_output(&output, ctx.json, out)
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
    use crate::error::error_text;
    use serde_json::{json, Value};
    use std::collections::BTreeSet;

    /// Every `CliError` variant, for the kind, exit-code and error-shape tables.
    fn all_errors() -> Vec<CliError> {
        vec![
            CliError::Transport("link down".to_owned()),
            CliError::Protocol(ProtoError::SwsTimeout),
            CliError::Elf("bad header".to_owned()),
            CliError::Io("missing file".to_owned()),
            CliError::InvalidAddress {
                message: "0x1000".to_owned(),
            },
            CliError::Device("short read".to_owned()),
            CliError::CrystalMismatch {
                target_mhz: 12,
                image_mhz: 16,
            },
            CliError::MissingElf,
        ]
    }

    #[test]
    fn exit_codes_are_three_for_mismatch_and_one_for_runtime_failures() {
        assert_eq!(
            CliError::CrystalMismatch {
                target_mhz: 12,
                image_mhz: 16,
            }
            .exit_code(),
            3
        );
        for error in all_errors() {
            let expected = if matches!(error, CliError::CrystalMismatch { .. }) {
                3
            } else {
                1
            };
            assert_eq!(error.exit_code(), expected, "{error:?}");
        }
    }

    #[test]
    fn error_kinds_are_stable_and_codes_survive_the_mapping() {
        let kinds = [
            ("Transport", CliError::Transport(String::new())),
            ("Elf", CliError::Elf(String::new())),
            ("Io", CliError::Io(String::new())),
            (
                "InvalidAddress",
                CliError::InvalidAddress {
                    message: String::new(),
                },
            ),
            ("Device", CliError::Device(String::new())),
            (
                "CrystalMismatch",
                CliError::CrystalMismatch {
                    target_mhz: 12,
                    image_mhz: 16,
                },
            ),
            ("MissingElf", CliError::MissingElf),
        ];
        for (kind, error) in kinds {
            assert_eq!(error.kind(), kind, "{error:?}");
            assert_eq!(error.proto_error(), None, "{error:?}");
        }

        // `Response::Error` keeps its wire code all the way to `proto_error`.
        let codes = [
            ProtoError::InvalidFrame,
            ProtoError::BadCrc,
            ProtoError::InvalidAddrRange,
            ProtoError::InvalidArg,
            ProtoError::SwsTimeout,
            ProtoError::FlashVerifyFailed,
            ProtoError::FlashBusyTimeout,
        ];
        for code in codes {
            let error = CliError::Protocol(code);
            assert_eq!(error.kind(), "Protocol");
            assert_eq!(error.proto_error(), Some(code.code()), "{code:?}");
        }
    }

    #[test]
    fn error_object_has_the_documented_shape_and_message() {
        let mismatch = CliError::CrystalMismatch {
            target_mhz: 12,
            image_mhz: 16,
        };
        assert_eq!(
            error_text(&mismatch, true),
            concat!(
                r#"{"command":"error","exit":3,"error":"CrystalMismatch","proto_error":null,"#,
                r#""message":"crystal mismatch: target 12 MHz, image 16 MHz (use --force to flash anyway); target left stopped — run `probe-tlsr reset`"}"#
            )
        );

        let protocol = CliError::Protocol(ProtoError::SwsTimeout);
        assert_eq!(
            error_text(&protocol, true),
            concat!(
                r#"{"command":"error","exit":1,"error":"Protocol","proto_error":5,"#,
                r#""message":"probe error: SwsTimeout"}"#
            )
        );
        assert_eq!(
            error_text(&protocol, false),
            "error: probe error: SwsTimeout"
        );

        // Every failure renders exactly the five documented keys.
        for error in all_errors() {
            let text = error_text(&error, true);
            let value: Value = serde_json::from_str(&text).expect("error object is json");
            let object = value.as_object().expect("error object is a map");
            let keys: BTreeSet<&str> = object.keys().map(String::as_str).collect();
            assert_eq!(
                keys,
                ["command", "error", "exit", "message", "proto_error"]
                    .into_iter()
                    .collect::<BTreeSet<_>>(),
                "{error:?}"
            );
            assert_eq!(value["command"], "error");
            assert_eq!(value["exit"], error.exit_code());
            assert_eq!(value["error"], error.kind());
            assert_eq!(value["proto_error"], json!(error.proto_error()));
        }
    }

    #[test]
    fn failure_reports_on_stderr_and_leaves_stdout_empty() {
        let cli = Cli {
            port: None,
            elf: None,
            json: true,
            timeout_ms: 1000,
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

    #[test]
    fn global_flags_work_before_and_after_the_subcommand() {
        let after = Cli::try_parse_from([
            "probe-tlsr",
            "read",
            "0x808000",
            "--json",
            "--timeout-ms",
            "250",
        ])
        .expect("global flags are accepted after the subcommand");
        assert!(after.json);
        assert_eq!(after.timeout_ms, 250);

        let before =
            Cli::try_parse_from(["probe-tlsr", "--json", "--port", "/dev/ttyACM1", "halt"])
                .expect("global flags are accepted before the subcommand");
        assert!(before.json);
        assert_eq!(before.timeout_ms, 5000);
        assert_eq!(before.port.as_deref(), Some(Path::new("/dev/ttyACM1")));
    }

    #[test]
    fn reset_stop_and_soft_conflict() {
        assert!(Cli::try_parse_from(["probe-tlsr", "reset", "--stop", "--soft"]).is_err());
        assert!(Cli::try_parse_from(["probe-tlsr", "reset", "--stop"]).is_ok());
        assert!(Cli::try_parse_from(["probe-tlsr", "reset", "--soft"]).is_ok());
    }
}
