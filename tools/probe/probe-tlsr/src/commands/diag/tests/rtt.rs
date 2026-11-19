//! Behavioural tests for `rtt` over the scripted transport.
//!
//! The loop runs at a 1 ms cadence here (production uses 50 ms), so the
//! watchdog and deadline paths stay fast. Fault tests assert that the stream's
//! last requests are exactly the `inspect` sequence, which is what proves the
//! core was stalled and reported before the error was returned.

use std::time::Duration;

use probe_proto::{CpuMode, Request, Response};
use serde_json::Value;

use super::fixture::*;
use crate::commands::diag::rtt::{
    stream, symbol_wire, Config, DEFAULT_WATCHDOG_MS, HEARTBEAT_SYMBOL, RTT_SYMBOL,
};
use crate::commands::test_support::*;
use crate::transport::MockTransport;
use crate::{CliError, Ctx};

/// A 1 ms stream over the fixture ELF with every optional behaviour off.
fn config<'a>(until: Option<&'a str>) -> Config<'a> {
    Config {
        ctrl_addr: RTT_WIRE,
        heartbeat: HEARTBEAT_WIRE,
        until,
        watchdog_ms: 0,
        timeout_ms: None,
        json: false,
    }
}

/// The replies one `inspect` fault report needs, after the rtt polls.
fn inspect_replies() -> Vec<Response> {
    stall_prefix()
        .into_iter()
        .chain([
            regs_data(regs(0x0080_bfc0, 0, 0), 0, shadow()),
            words_data(&[0u32; 16]),
        ])
        .chain(tail(0x00, 0x00))
        .collect()
}

#[test]
fn the_required_symbols_map_onto_wire_addresses() {
    let elf = rtt_image();
    assert_eq!(symbol_wire(&elf, RTT_SYMBOL), Ok(RTT_WIRE));
    assert_eq!(symbol_wire(&elf, HEARTBEAT_SYMBOL), Ok(HEARTBEAT_WIRE));
}

#[test]
fn until_match_exits_zero_after_emitting_the_lines() {
    let elf = rtt_image();
    let mut mock = MockTransport::new([Response::Ack, data(*b"boot\nready\n")]);
    let mut out = Vec::new();

    stream(
        &mut mock,
        &mut out,
        Some(&elf),
        None,
        &config(Some("ready")),
        Duration::from_millis(1),
    )
    .expect("an --until match is success");

    assert_eq!(out, b"boot\nready\n");
    assert_eq!(
        mock.requests,
        [
            sws_activate_req(),
            Request::RttPoll {
                ctrl_addr: RTT_WIRE
            }
        ],
        "nothing is polled after the match"
    );
}

#[test]
fn until_wins_over_a_panic_marker_on_the_same_line() {
    let elf = rtt_image();
    let mut mock = MockTransport::new([Response::Ack, data(*b"[PANIC] boom\n")]);
    let mut out = Vec::new();

    stream(
        &mut mock,
        &mut out,
        Some(&elf),
        None,
        &config(Some("[PANIC]")),
        Duration::from_millis(1),
    )
    .expect("--until is checked before the panic markers");

    assert_eq!(out, b"[PANIC] boom\n");
    assert!(
        !mock
            .requests
            .iter()
            .any(|request| matches!(request, Request::CpuCtrl { .. })),
        "a match must not stall the core"
    );
}

#[test]
fn a_panic_line_emits_the_report_then_exits_three() {
    let elf = rtt_image();
    let mut responses = vec![Response::Ack, data(*b"boot\n[PANIC] boom\n")];
    responses.extend(inspect_replies());
    let mut mock = MockTransport::new(responses);
    let mut out = Vec::new();

    let err = stream(
        &mut mock,
        &mut out,
        Some(&elf),
        None,
        &Config {
            json: true,
            ..config(None)
        },
        Duration::from_millis(1),
    )
    .expect_err("a panic marker is a fault");

    assert_eq!(err.exit_code(), 3);
    let CliError::TargetPanic { line } = &err else {
        panic!("expected TargetPanic, got {err:?}");
    };
    assert_eq!(line, "[PANIC] boom");

    // Line, line, then the inspect report: stdout keeps the crash, stderr gets
    // the failure.
    let text = String::from_utf8(out).expect("output is utf-8");
    let values: Vec<Value> = text
        .lines()
        .map(|line| serde_json::from_str(line).expect("each output line is one object"))
        .collect();
    assert_eq!(values.len(), 3, "{text}");
    assert_eq!(values[0]["command"], "rtt");
    assert_eq!(values[0]["line"], "boot");
    assert_eq!(values[1]["command"], "rtt");
    assert_eq!(values[1]["line"], "[PANIC] boom");
    assert_eq!(values[2]["command"], "inspect");

    let mut expected = vec![
        sws_activate_req(),
        Request::RttPoll {
            ctrl_addr: RTT_WIRE,
        },
    ];
    expected.extend(inspect_requests());
    assert_eq!(mock.requests, expected);
}

#[test]
fn a_frozen_heartbeat_emits_the_report_then_exits_two() {
    let elf = rtt_image();

    // Two passes: the second heartbeat read is at least a 2 ms sleep after the
    // baseline one, so the 1 ms watchdog trips there deterministically.
    let mut responses = vec![
        Response::Ack,
        data(Vec::new()),
        data(0x2a_u32.to_le_bytes()),
        data(Vec::new()),
        data(0x2a_u32.to_le_bytes()),
    ];
    responses.extend(inspect_replies());
    let mut mock = MockTransport::new(responses);
    let mut out = Vec::new();

    let err = stream(
        &mut mock,
        &mut out,
        Some(&elf),
        None,
        &Config {
            watchdog_ms: 1,
            ..config(None)
        },
        Duration::from_millis(2),
    )
    .expect_err("a frozen heartbeat is a fault");

    assert_eq!(err.exit_code(), 2, "{err:?}");
    assert_eq!(
        err,
        CliError::HeartbeatStalled {
            last: 0x2a,
            watchdog_ms: 1,
        }
    );
    assert!(!out.is_empty(), "the report is written before the error");

    let stall = mock
        .requests
        .iter()
        .position(|request| {
            matches!(
                request,
                Request::CpuCtrl {
                    mode: CpuMode::Stall
                }
            )
        })
        .expect("the fault stalls the core");
    assert_eq!(
        &mock.requests[stall..],
        &inspect_requests()[1..],
        "the inspect sequence must follow the stall"
    );
}

#[test]
fn the_stream_deadline_expires_as_a_timeout() {
    let elf = rtt_image();

    // A moving heartbeat, so only the deadline can end the stream.
    let mut responses = vec![Response::Ack];
    for beat in 0..40u32 {
        responses.push(data(Vec::new()));
        responses.push(data(beat.to_le_bytes()));
    }
    let mut mock = MockTransport::new(responses);
    let mut out = Vec::new();

    let err = stream(
        &mut mock,
        &mut out,
        Some(&elf),
        None,
        &Config {
            timeout_ms: Some(1),
            ..config(None)
        },
        Duration::from_millis(1),
    )
    .expect_err("a 1 ms deadline must expire");

    assert_eq!(err.exit_code(), 1);
    assert!(matches!(err, CliError::Timeout(_)), "{err:?}");
    assert!(out.is_empty(), "a deadline writes no lines");
}

/// A 1 ms deadline over a stream whose first poll leaves an unterminated line.
fn stream_with_one_partial_line(
    until: Option<&str>,
    out: &mut Vec<u8>,
) -> (MockTransport, Result<(), CliError>) {
    let mut responses = vec![Response::Ack];
    for beat in 0..40u32 {
        responses.push(data(if beat == 0 {
            b"tail".to_vec()
        } else {
            Vec::new()
        }));
        responses.push(data(beat.to_le_bytes()));
    }
    let mut mock = MockTransport::new(responses);
    let err = stream(
        &mut mock,
        out,
        Some(&rtt_image()),
        None,
        &Config {
            timeout_ms: Some(1),
            ..config(until)
        },
        Duration::from_millis(1),
    );
    (mock, err)
}

#[test]
fn a_partial_line_is_flushed_before_the_deadline_error() {
    let mut out = Vec::new();
    let (_, err) = stream_with_one_partial_line(None, &mut out);

    assert!(matches!(err, Err(CliError::Timeout(_))), "{err:?}");
    assert_eq!(
        out, b"tail\n",
        "the buffered tail is emitted instead of being dropped"
    );
}

#[test]
fn a_partial_line_matching_until_still_exits_zero() {
    let mut out = Vec::new();
    let (_, err) = stream_with_one_partial_line(Some("tail"), &mut out);

    err.expect("the flushed tail matches --until");
    assert_eq!(out, b"tail\n");
}

#[test]
fn missing_elf_is_rejected_before_any_request() {
    // No scripted replies: any request at all would fail as a `Device` error
    // instead of `MissingElf`, so the variant alone proves nothing was sent.
    let mut ctx = Ctx {
        transport: Box::new(MockTransport::new([])),
        elf: None,
        elf_path: None,
        json: false,
        timeout_ms: None,
    };
    let mut out = Vec::new();

    let err = crate::commands::diag::rtt::run(&mut ctx, &mut out, None, DEFAULT_WATCHDOG_MS)
        .expect_err("rtt needs --elf");

    // `read`/`write` still report `MissingElf` for a bare symbol; `rtt` names
    // the two symbols it derives and the flag that carries them.
    let CliError::Elf(message) = &err else {
        panic!("expected an Elf error, got {err:?}");
    };
    assert!(message.contains("--elf"), "{message}");
    assert!(message.contains(RTT_SYMBOL), "{message}");
    assert!(message.contains(HEARTBEAT_SYMBOL), "{message}");
    assert!(out.is_empty());
}

#[test]
fn a_line_split_across_two_polls_is_reassembled() {
    let elf = rtt_image();
    let mut mock = MockTransport::new([
        Response::Ack,
        data(*b"hel"),
        data(1u32.to_le_bytes()),
        data(*b"lo\n"),
    ]);
    let mut out = Vec::new();

    stream(
        &mut mock,
        &mut out,
        Some(&elf),
        None,
        &config(Some("hello")),
        Duration::from_millis(1),
    )
    .expect("the reassembled line matches");

    assert_eq!(out, b"hello\n");
    assert_eq!(
        mock.requests,
        [
            sws_activate_req(),
            Request::RttPoll {
                ctrl_addr: RTT_WIRE
            },
            read_mem_req(HEARTBEAT_WIRE, 4),
            Request::RttPoll {
                ctrl_addr: RTT_WIRE
            },
        ]
    );
}

#[test]
fn carriage_returns_are_stripped_even_with_no_deadline() {
    let elf = rtt_image();
    let mut mock = MockTransport::new([Response::Ack, data(*b"boot\r\n")]);
    let mut out = Vec::new();

    // `--watchdog-ms 0` and `--timeout-ms 0` both mean "no limit".
    stream(
        &mut mock,
        &mut out,
        Some(&elf),
        None,
        &Config {
            timeout_ms: Some(0),
            ..config(Some("boot"))
        },
        Duration::from_millis(1),
    )
    .expect("the stripped line matches");

    assert_eq!(out, b"boot\n");
}

#[test]
fn a_missing_symbol_is_an_elf_error() {
    let without_rtt = rtt_image_without(RTT_SYMBOL);
    let err = symbol_wire(&without_rtt, RTT_SYMBOL).expect_err("`_TLSR_RTT` is required");
    assert!(matches!(err, CliError::Elf(_)), "{err:?}");
    assert!(err.to_string().contains(RTT_SYMBOL), "{err}");

    let without_beat = rtt_image_without(HEARTBEAT_SYMBOL);
    let err = symbol_wire(&without_beat, HEARTBEAT_SYMBOL).expect_err("`HEARTBEAT` is required");
    assert!(matches!(err, CliError::Elf(_)), "{err:?}");
    assert!(err.to_string().contains(HEARTBEAT_SYMBOL), "{err}");
}

#[test]
fn the_dispatch_streams_and_returns_no_output() {
    // `commands::run` must not print a second object after the lines `rtt`
    // already wrote, so the dispatch returns `None`.
    let mut ctx = Ctx {
        transport: Box::new(MockTransport::new([Response::Ack, data(*b"boot\n")])),
        elf: Some(rtt_image()),
        elf_path: None,
        json: true,
        timeout_ms: None,
    };
    let mut out = Vec::new();

    let output = crate::commands::run(
        crate::Command::Rtt {
            until: Some("boot".to_owned()),
            watchdog_ms: 0,
        },
        &mut ctx,
        &mut out,
    )
    .expect("rtt runs");

    assert_eq!(output, None, "rtt renders its own lines");
    assert_eq!(
        String::from_utf8(out).expect("output is utf-8"),
        "{\"command\":\"rtt\",\"line\":\"boot\"}\n"
    );
}
