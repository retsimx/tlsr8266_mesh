//! Contract pins for the CLI: exit codes, error kinds, the JSON error object
//! and the `--help` text a user actually reads.

use super::*;
use crate::error::error_text;
use serde_json::{json, Value};
use std::collections::BTreeSet;

/// Every `CliError` variant with the kind and exit code the contract pins
/// for it. The error-shape test walks the same table, so a new variant
/// cannot ship without a kind or an exit code.
fn error_cases() -> Vec<(CliError, &'static str, u8)> {
    vec![
        (CliError::Transport("link down".to_owned()), "Transport", 1),
        (CliError::Protocol(ProtoError::SwsTimeout), "Protocol", 1),
        (CliError::Elf("bad header".to_owned()), "Elf", 1),
        (CliError::Io("missing file".to_owned()), "Io", 1),
        (
            CliError::InvalidAddress {
                message: "0x1000".to_owned(),
            },
            "InvalidAddress",
            1,
        ),
        (CliError::Device("short read".to_owned()), "Device", 1),
        (
            CliError::CrystalMismatch {
                target_mhz: 12,
                image_mhz: 16,
            },
            "CrystalMismatch",
            3,
        ),
        (CliError::MissingElf, "MissingElf", 1),
        (
            CliError::Timeout("rtt: stream deadline of 5 ms expired".to_owned()),
            "Timeout",
            1,
        ),
        (
            CliError::HeartbeatStalled {
                last: 0x2a,
                watchdog_ms: 1000,
            },
            "HeartbeatStalled",
            2,
        ),
        (
            CliError::TargetPanic {
                line: "[PANIC] boom".to_owned(),
            },
            "TargetPanic",
            3,
        ),
    ]
}

/// Every `CliError` variant, for the error-shape table.
fn all_errors() -> Vec<CliError> {
    error_cases()
        .into_iter()
        .map(|(error, _, _)| error)
        .collect()
}

/// Rendered `--help` text, with clap's line wrapping undone so assertions
/// can match whole sentences.
fn help(args: &[&str]) -> String {
    let err = Cli::try_parse_from(args).expect_err("--help is a display error");
    assert_eq!(err.kind(), clap::error::ErrorKind::DisplayHelp, "{args:?}");
    err.to_string()
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
}

#[test]
fn exit_codes_are_pinned_per_variant() {
    for (error, _, expected) in error_cases() {
        assert_eq!(error.exit_code(), expected, "{error:?}");
    }
}

#[test]
fn error_kinds_are_stable_and_codes_survive_the_mapping() {
    for (error, kind, _) in error_cases() {
        assert_eq!(error.kind(), kind, "{error:?}");
        if kind != "Protocol" {
            assert_eq!(error.proto_error(), None, "{error:?}");
        }
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
fn fault_messages_name_the_observation_and_the_recovery() {
    let stalled = CliError::HeartbeatStalled {
        last: 0x2a,
        watchdog_ms: 1000,
    };
    assert_eq!(
        stalled.to_string(),
        "rtt: heartbeat stuck at 0x0000002a for 1000 ms; target left stalled — run `probe-tlsr reset --soft`"
    );

    let panic = CliError::TargetPanic {
        line: "[PANIC] boom".to_owned(),
    };
    assert_eq!(
        panic.to_string(),
        "rtt: panic line \"[PANIC] boom\"; target left stalled — run `probe-tlsr reset --soft`"
    );

    let timeout = CliError::Timeout("rtt: stream deadline of 5 ms expired".to_owned());
    assert_eq!(timeout.to_string(), "rtt: stream deadline of 5 ms expired");
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
    assert_eq!(after.timeout_ms, Some(250));

    let before = Cli::try_parse_from(["probe-tlsr", "--json", "--port", "/dev/ttyACM1", "halt"])
        .expect("global flags are accepted before the subcommand");
    assert!(before.json);
    assert_eq!(
        before.timeout_ms, None,
        "--timeout-ms has no default; the transport supplies 5000 ms"
    );
    assert_eq!(
        before.port.as_deref(),
        Some(std::path::Path::new("/dev/ttyACM1"))
    );
}

#[test]
fn reset_stop_and_soft_conflict() {
    assert!(Cli::try_parse_from(["probe-tlsr", "reset", "--stop", "--soft"]).is_err());
    assert!(Cli::try_parse_from(["probe-tlsr", "reset", "--stop"]).is_ok());
    assert!(Cli::try_parse_from(["probe-tlsr", "reset", "--soft"]).is_ok());
}

#[test]
fn rtt_defaults_and_flags_parse() {
    let cli = Cli::try_parse_from(["probe-tlsr", "rtt", "--elf", "app.elf"])
        .expect("rtt parses without its options");
    let Command::Rtt { until, watchdog_ms } = cli.command else {
        panic!("expected rtt, got {:?}", cli.command);
    };
    assert_eq!(until, None);
    assert_eq!(
        watchdog_ms, DEFAULT_WATCHDOG_MS,
        "--watchdog-ms defaults to the documented 1000 ms"
    );

    let cli = Cli::try_parse_from([
        "probe-tlsr",
        "rtt",
        "--until",
        "boot ok",
        "--watchdog-ms",
        "0",
    ])
    .expect("rtt options parse");
    let Command::Rtt { until, watchdog_ms } = cli.command else {
        panic!("expected rtt, got {:?}", cli.command);
    };
    assert_eq!(until.as_deref(), Some("boot ok"));
    assert_eq!(watchdog_ms, 0, "0 disables the watchdog");
}

#[test]
fn trap_slot_defaults_to_zero_and_is_range_checked() {
    let cli = Cli::try_parse_from(["probe-tlsr", "trap", "main"])
        .expect("trap parses without its options");
    let Command::Trap {
        addr_or_symbol,
        slot,
    } = cli.command
    else {
        panic!("expected trap, got {:?}", cli.command);
    };
    assert_eq!(addr_or_symbol, "main");
    assert_eq!(slot, 0, "--slot defaults to the documented slot 0");
    assert_eq!(
        cli.timeout_ms, None,
        "absent --timeout-ms means the wait has no deadline"
    );

    let cli = Cli::try_parse_from(["probe-tlsr", "trap", "0x104", "--slot", "3"])
        .expect("slot 3 is in range");
    let Command::Trap { slot, .. } = cli.command else {
        panic!("expected trap, got {:?}", cli.command);
    };
    assert_eq!(slot, 3);

    // Out of range is a usage error, reported before any probe traffic.
    let err = Cli::try_parse_from(["probe-tlsr", "trap", "0x104", "--slot", "4"])
        .expect_err("slot 4 is out of range");
    assert_eq!(err.kind(), clap::error::ErrorKind::ValueValidation);
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn help_states_the_two_timeout_readings() {
    // The global flag explains the transport fallback and the deadline
    // reading; `--help` is the only text a user reads.
    let top = help(&["probe-tlsr", "--help"]);
    assert!(top.contains("absent means the transport default"), "{top}");
    assert!(top.contains("stream or wait deadline"), "{top}");
    assert!(top.contains("absent or `0` means no deadline"), "{top}");

    for subcommand in ["rtt", "trap"] {
        let text = help(&["probe-tlsr", subcommand, "--help"]);
        assert!(
            text.contains("means no deadline") || text.contains("waits forever"),
            "{text}"
        );
    }
}

#[test]
fn rtt_help_says_the_elf_is_required() {
    // `rtt` derives `_TLSR_RTT` and `HEARTBEAT` from `.symtab`; without
    // `--elf` it fails before any request, so the flag is not optional here.
    let text = help(&["probe-tlsr", "rtt", "--help"]);
    assert!(text.contains("Requires `--elf`"), "{text}");
    assert!(
        text.contains("`_TLSR_RTT` and `HEARTBEAT` are resolved from its `.symtab`"),
        "{text}"
    );
}

#[test]
fn help_names_every_command_that_leaves_the_core_stopped() {
    let top = help(&["probe-tlsr", "--help"]);
    assert!(
        top.contains("`inspect` and an `rtt` fault leave it stalled"),
        "{top}"
    );
    assert!(top.contains("`reset --soft` after a stall"), "{top}");
}
