//! The command line and the failure type.
//!
//! Clap derives the parser from [`Cli`] and [`Command`]; the enum variant names
//! are the CLI spellings. [`CliError`] is every failure a command can report,
//! together with the exit code and the stable JSON kind it maps to.
//!
//! `--timeout-ms` has two readings, and the help says so: the transport falls
//! back to [`DEFAULT_TIMEOUT_MS`](crate::DEFAULT_TIMEOUT_MS) when the flag is
//! absent, while `rtt` and `trap` treat absent or `0` as *no deadline at all*
//! (a 5 s default would kill every log stream).

use std::fmt;
use std::path::PathBuf;

use clap::{Parser, Subcommand};
use probe_proto::ProtoError;

use crate::commands::diag::rtt::DEFAULT_WATCHDOG_MS;

/// Options shared by every subcommand.
#[derive(Debug, Parser)]
#[command(
    name = "probe-tlsr",
    version,
    about = "TLSR8266 SWire probe host CLI",
    after_help = "The probe port is opened before arguments are validated, so a missing probe is\n\
                  reported before a bad argument. `flash` without --reset and `halt` leave the\n\
                  core stopped; `inspect` and an `rtt` fault leave it stalled. Run `reset` to\n\
                  restart it (use `reset --soft` after a stall). Exit 2 is both a usage error\n\
                  and a frozen `rtt` heartbeat; exit 3 is an `rtt` panic line."
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
    /// Per-response timeout in milliseconds; absent means the transport default.
    ///
    /// The transport falls back to 5000 ms, so every command that waits for one
    /// reply keeps that cadence. `rtt` and `trap` also read this flag as their
    /// stream or wait deadline, where absent or `0` means no deadline.
    #[arg(long, global = true, value_name = "MS")]
    pub timeout_ms: Option<u64>,
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
    /// Stall the core and dump registers, stacks, reset cause and panic body.
    ///
    /// The core is left stalled; run `reset --soft` to restart it.
    Inspect,
    /// Stream the target's RTT log, failing on a panic line or a frozen heartbeat.
    ///
    /// Requires `--elf`: `_TLSR_RTT` and `HEARTBEAT` are resolved from its
    /// `.symtab`, and the command fails before any request without it.
    ///
    /// The stream ends on an `--until` match, a fault, or the `--timeout-ms`
    /// deadline; an absent or `0` `--timeout-ms` means no deadline.
    Rtt {
        /// Stop after the first emitted line containing this substring.
        #[arg(long, value_name = "SUBSTR")]
        until: Option<String>,
        /// Fail when `HEARTBEAT` is unchanged for this long; 0 disables it.
        #[arg(long, value_name = "MS", default_value_t = DEFAULT_WATCHDOG_MS)]
        watchdog_ms: u64,
    },
    /// Stall the core and disable the watchdog; only `reset` restarts it.
    Halt,
    /// Arm a hardware breakpoint, wait for it, then report and soft-reboot.
    ///
    /// The wait ends on the hit or the `--timeout-ms` deadline; an absent or `0`
    /// `--timeout-ms` waits forever. The armed slot is cleared before any
    /// failure, and a hit always soft-reboots the core.
    Trap {
        /// Address (`0x...` or decimal) or symbol name.
        addr_or_symbol: String,
        /// Breakpoint slot to arm.
        #[arg(
            long,
            value_name = "SLOT",
            default_value_t = 0,
            value_parser = clap::value_parser!(u8).range(0..=3)
        )]
        slot: u8,
    },
}

/// Every failure a command can report, plus its exit code.
#[derive(Debug, PartialEq, Eq)]
pub enum CliError {
    Transport(String),
    Protocol(ProtoError),
    Elf(String),
    Io(String),
    InvalidAddress {
        message: String,
    },
    Device(String),
    MissingElf,
    /// A wait deadline expired before the stream or trap condition was met.
    Timeout(String),
    /// `rtt` saw the same `HEARTBEAT` value for the whole watchdog window.
    HeartbeatStalled {
        last: u32,
        watchdog_ms: u64,
    },
    /// `rtt` saw a firmware panic marker on the log stream.
    TargetPanic {
        line: String,
    },
}

impl CliError {
    /// Process exit code: `2` for a frozen heartbeat, `3` for a `rtt` panic
    /// line, else `1`.
    ///
    /// Usage errors never reach this: clap exits with `2` while parsing, which
    /// is why `2` is deliberately shared with [`Self::HeartbeatStalled`].
    pub fn exit_code(&self) -> u8 {
        match self {
            Self::HeartbeatStalled { .. } => 2,
            Self::TargetPanic { .. } => 3,
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
            Self::MissingElf => "MissingElf",
            Self::Timeout(_) => "Timeout",
            Self::HeartbeatStalled { .. } => "HeartbeatStalled",
            Self::TargetPanic { .. } => "TargetPanic",
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
            Self::MissingElf => f.write_str("symbol lookup needs --elf <path>"),
            Self::Timeout(message) => f.write_str(message),
            Self::HeartbeatStalled { last, watchdog_ms } => write!(
                f,
                "rtt: heartbeat stuck at 0x{last:08x} for {watchdog_ms} ms; target left stalled — run `probe-tlsr reset --soft`"
            ),
            Self::TargetPanic { line } => write!(
                f,
                "rtt: panic line \"{line}\"; target left stalled — run `probe-tlsr reset --soft`"
            ),
        }
    }
}

impl std::error::Error for CliError {}

#[cfg(test)]
mod tests;
