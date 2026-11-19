//! Behavioural tests for `trap` over the scripted transport.
//!
//! Every test pins the exact request sequence, so a reordered request fails
//! here rather than on the probe. Waits run at a 1 ms cadence (production uses
//! 50 ms), and the timeout tests use an interval longer than the deadline so
//! the wait polls exactly once before it gives up.
//!
//! The report is rendered by the handler, so the hit tests read it back from
//! the output buffer: that is what proves the capture reaches stdout before the
//! slot is cleared and the core rebooted.

use std::time::Duration;

use probe_proto::{ProtoError, Request, Response};
use serde_json::Value;

use super::fixture::*;
use crate::commands::diag::trap::{code_addr, run, wait, Capture, BREAKPOINT_BASE, BREAKPOINT_HIT};
use crate::commands::test_support::*;
use crate::transport::MockTransport;
use crate::{CliError, Command, Ctx};

/// The slot word the probe reads back after a hit: bit 31 set on top of the
/// armed `addr | 0x0100_0000` value.
fn hit_word(addr: u32) -> Response {
    data((BREAKPOINT_HIT | 0x0100_0000 | addr).to_le_bytes())
}

/// Every reply one hit needs: activate, arm, the hit poll, the inspect capture,
/// the slot clear and the soft reboot.
fn hit_replies(addr: u32) -> Vec<Response> {
    let mut replies = vec![Response::Ack, Response::Ack, hit_word(addr)];
    replies.extend(stall_prefix());
    replies.push(regs_data(regs(0x0080_bfc0, 0, 0x104), 0, shadow()));
    replies.push(words_data(&[0u32; 16]));
    replies.extend(tail(0x00, 0x2a));
    replies.push(Response::Ack); // clear the slot
    replies.push(Response::Ack); // soft reboot
    replies
}

/// The whole hit sequence: arm, poll `poll_reg`, capture, clear, reboot.
fn hit_requests(addr: u32, slot: u8, poll_reg: u16) -> Vec<Request> {
    let mut requests = vec![
        sws_activate_req(),
        Request::SetBreakpoint { slot, addr },
        read_mem_req(poll_reg, 4),
    ];
    requests.extend(inspect_requests());
    requests.push(Request::SetBreakpoint { slot, addr: 0 });
    requests.push(Request::CpuCtrl {
        mode: probe_proto::CpuMode::SoftReboot,
    });
    requests
}

/// A capture that renders one JSON object into `out`.
fn capture(out: &mut Vec<u8>) -> Capture<'_> {
    Capture {
        out,
        json: true,
        elf: None,
        elf_path: None,
    }
}

/// Parses the one object the handler rendered.
fn rendered(out: &[u8]) -> Value {
    serde_json::from_slice(out).expect("the handler writes one JSON object")
}

#[test]
fn a_hit_emits_the_capture_before_it_clears_the_slot() {
    let mut mock = MockTransport::new(hit_replies(0x100));
    let mut out = Vec::new();

    wait(
        &mut mock,
        &mut capture(&mut out),
        0x100,
        0,
        None,
        Duration::from_millis(1),
    )
    .expect("the hit succeeds");

    // `--json` stays one object, and it carries the whole capture.
    let value = rendered(&out);
    assert_eq!(value["command"], "trap");
    assert_eq!(value["addr"], 0x100);
    assert_eq!(value["slot"], 0);
    assert_eq!(value["hit"], true);
    assert_eq!(value["inspect"]["sentinel"], 0x00);
    assert_eq!(value["inspect"]["breadcrumb"], 0x2a);
    assert_eq!(mock.requests, hit_requests(0x100, 0, 0x0610));
}

#[test]
fn a_failing_clear_still_emits_the_capture() {
    // The clear fails after the capture: the one-shot dump must already be on
    // stdout, and the error must say the slot state.
    let mut replies = hit_replies(0x100);
    let clear = replies.len() - 2;
    replies[clear] = Response::Error(ProtoError::SwsTimeout);
    let mut mock = MockTransport::new(replies);
    let mut out = Vec::new();

    let err = wait(
        &mut mock,
        &mut capture(&mut out),
        0x100,
        0,
        None,
        Duration::from_millis(1),
    )
    .expect_err("a failed clear is reported");

    let value = rendered(&out);
    assert_eq!(value["command"], "trap");
    assert_eq!(value["inspect"]["breadcrumb"], 0x2a);

    let message = err.to_string();
    assert!(
        message.contains("clearing the breakpoint slot 0 failed"),
        "{message}"
    );
    assert!(message.contains("left stopped"), "{message}");
    assert!(
        !mock.requests.iter().any(|request| matches!(
            request,
            Request::CpuCtrl {
                mode: probe_proto::CpuMode::SoftReboot
            }
        )),
        "no reboot after a failed clear: {:?}",
        mock.requests
    );
}

#[test]
fn a_failing_capture_clears_the_slot_and_says_so() {
    // The capture stops at `ReadRegs`: the slot is still armed and the core is
    // stalled, so the clear is sent and the error names both facts.
    let mut replies = vec![Response::Ack, Response::Ack, hit_word(0x100)];
    replies.extend(stall_prefix());
    replies.push(Response::Error(ProtoError::SwsTimeout)); // ReadRegs fails
    replies.push(Response::Ack); // the unconditional clear
    let mut mock = MockTransport::new(replies);
    let mut out = Vec::new();

    let err = wait(
        &mut mock,
        &mut capture(&mut out),
        0x100,
        0,
        None,
        Duration::from_millis(1),
    )
    .expect_err("a failed capture is an error");

    let message = err.to_string();
    assert!(message.contains("the capture failed"), "{message}");
    assert!(message.contains("slot 0 is cleared"), "{message}");
    assert!(message.contains("left stopped"), "{message}");
    assert!(out.is_empty(), "a failed capture renders nothing");
    assert_eq!(
        mock.requests.last(),
        Some(&Request::SetBreakpoint { slot: 0, addr: 0 }),
        "an armed slot must never be left behind: {:?}",
        mock.requests
    );
}

#[test]
fn slot_two_polls_its_own_control_word() {
    let mut mock = MockTransport::new(hit_replies(0x104));
    let mut out = Vec::new();

    wait(
        &mut mock,
        &mut capture(&mut out),
        0x104,
        2,
        None,
        Duration::from_millis(1),
    )
    .expect("the hit succeeds");

    assert_eq!(mock.requests, hit_requests(0x104, 2, 0x0618));
    assert_eq!(
        BREAKPOINT_BASE + 2 * 4,
        0x0618,
        "the fixture registers agree"
    );
    assert_eq!(rendered(&out)["slot"], 2);
}

#[test]
fn a_deadline_clears_the_slot_before_it_times_out() {
    let mut mock = MockTransport::new([
        Response::Ack,
        Response::Ack,
        data(0u32.to_le_bytes()), // no bit 31: the core is still running
        Response::Ack,            // the unconditional clear
    ]);
    let mut out = Vec::new();

    // The interval is longer than the deadline, so exactly one poll happens
    // before the deadline is checked; the test costs about a millisecond.
    let err = wait(
        &mut mock,
        &mut capture(&mut out),
        0x100,
        0,
        Some(1),
        Duration::from_millis(10),
    )
    .expect_err("a missed breakpoint is a timeout");

    assert_eq!(err.exit_code(), 1);
    let CliError::Timeout(message) = &err else {
        panic!("expected Timeout, got {err:?}");
    };
    for needle in ["slot 0", "0x000100", "1 ms", "slot is cleared"] {
        assert!(message.contains(needle), "{message}");
    }
    assert!(out.is_empty(), "a miss renders nothing");

    assert_eq!(
        mock.requests,
        [
            sws_activate_req(),
            Request::SetBreakpoint {
                slot: 0,
                addr: 0x100
            },
            read_mem_req(0x0610, 4),
            Request::SetBreakpoint { slot: 0, addr: 0 },
        ],
        "the clear is sent before the timeout is reported"
    );
}

#[test]
fn a_failing_clear_keeps_the_timeout_and_names_both_facts() {
    let mut mock = MockTransport::new([
        Response::Ack,
        Response::Ack,
        data(0u32.to_le_bytes()),
        Response::Error(ProtoError::SwsTimeout),
    ]);
    let mut out = Vec::new();

    let err = wait(
        &mut mock,
        &mut capture(&mut out),
        0x100,
        0,
        Some(1),
        Duration::from_millis(10),
    )
    .expect_err("a missed breakpoint is a timeout");

    // The deadline, not the write failure, is what the operator needs first.
    let CliError::Timeout(message) = &err else {
        panic!("expected Timeout, got {err:?}");
    };
    assert!(message.contains("was not hit within 1 ms"), "{message}");
    assert!(message.contains("clearing it also failed"), "{message}");
    assert_eq!(
        mock.requests.last(),
        Some(&Request::SetBreakpoint { slot: 0, addr: 0 })
    );
}

#[test]
fn a_failing_poll_still_clears_the_slot() {
    // A probe-reported error: the clear is still sent, and the read fault wins.
    let mut mock = MockTransport::new([
        Response::Ack,
        Response::Ack,
        Response::Error(ProtoError::SwsTimeout),
        Response::Ack,
    ]);
    let mut out = Vec::new();

    let err = wait(
        &mut mock,
        &mut capture(&mut out),
        0x100,
        0,
        None,
        Duration::from_millis(1),
    )
    .expect_err("a read fault ends the wait");

    assert_eq!(err, CliError::Protocol(ProtoError::SwsTimeout));
    assert_eq!(
        mock.requests,
        [
            sws_activate_req(),
            Request::SetBreakpoint {
                slot: 0,
                addr: 0x100
            },
            read_mem_req(0x0610, 4),
            Request::SetBreakpoint { slot: 0, addr: 0 },
        ]
    );

    // A read with no reply left is a `Device` fault; the clear is sent anyway.
    let mut mock = MockTransport::new([Response::Ack, Response::Ack]);
    let mut out = Vec::new();

    let err = wait(
        &mut mock,
        &mut capture(&mut out),
        0x100,
        0,
        None,
        Duration::from_millis(1),
    )
    .expect_err("a truncated reply ends the wait");

    assert!(matches!(err, CliError::Device(_)), "{err:?}");
    assert_eq!(
        mock.requests.last(),
        Some(&Request::SetBreakpoint { slot: 0, addr: 0 }),
        "an armed slot must never be left behind: {:?}",
        mock.requests
    );
}

#[test]
fn zero_and_wide_addresses_are_rejected_before_any_request() {
    for spec in ["0", "0x0", "0x01000000"] {
        // No scripted reply: any request at all would surface as `Device`
        // instead of `InvalidAddress`, so the variant proves nothing was sent.
        let mut ctx = test_ctx(Vec::<Response>::new());
        let mut out = Vec::new();

        let err = run(&mut ctx, &mut out, spec, 0).expect_err("the address is not a code address");

        assert!(
            matches!(err, CliError::InvalidAddress { .. }),
            "{spec}: {err:?}"
        );
        assert!(err.to_string().contains("24-bit"), "{spec}: {err}");
        assert!(out.is_empty());
    }

    // The reported value is what the operator typed, not a wire address.
    assert_eq!(
        code_addr(None, "0x01000000"),
        Err(CliError::InvalidAddress {
            message: "0x1000000 is wider than the 24-bit code address space (0x1..=0x00ffffff)"
                .to_owned(),
        })
    );
}

#[test]
fn a_symbol_resolves_to_its_raw_code_address() {
    let elf = image(("irq_spare", 0x0080_b000, 0x700));

    // `sym` is a `.text` symbol at physical 0x100. `to_swire` would refuse it
    // (its windows describe MMIO and SRAM), so the raw value proves the SWire
    // mapping is skipped instead of applied.
    assert_eq!(code_addr(Some(&elf), "sym"), Ok(0x100));
    assert_eq!(code_addr(Some(&elf), "0x104"), Ok(0x104));
    assert_eq!(code_addr(Some(&elf), "260"), Ok(260));
    // Only symbols carry the Thumb bit; a typed number is literal.
    assert_eq!(code_addr(Some(&elf), "0x105"), Ok(0x105));
    assert_eq!(code_addr(Some(&elf), "261"), Ok(261));

    // Without `--elf` there is no `.symtab`, so only numbers can resolve.
    assert_eq!(code_addr(None, "sym"), Err(CliError::MissingElf));
}

#[test]
fn a_thumb_symbol_is_armed_at_its_even_address() {
    // ELF `st_value`s carry the Thumb bit (`0x5af5`) while the hardware
    // comparator matches the even PC (`0x5af4`), so the symbol must be armed
    // and reported at the even address.
    let elf = text_image(("proc_led", 0x5af5, 0x1fc));
    let addr = code_addr(Some(&elf), "proc_led").expect("the symbol resolves");
    assert_eq!(addr, 0x5af4);

    let mut mock = MockTransport::new(hit_replies(addr));
    let mut out = Vec::new();

    wait(
        &mut mock,
        &mut capture(&mut out),
        addr,
        0,
        None,
        Duration::from_millis(1),
    )
    .expect("the hit succeeds");

    assert_eq!(mock.requests, hit_requests(0x5af4, 0, 0x0610));
    assert_eq!(rendered(&out)["addr"], 0x5af4);
}

#[test]
fn run_dispatches_to_trap_and_prints_one_object() {
    let mut ctx = Ctx {
        elf: Some(image(("irq_spare", 0x0080_b000, 0x700))),
        json: true,
        ..test_ctx(hit_replies(0x100))
    };
    let mut out = Vec::new();

    let output = crate::commands::run(
        Command::Trap {
            addr_or_symbol: "sym".to_owned(),
            slot: 1,
        },
        &mut ctx,
        &mut out,
    )
    .expect("trap runs");

    assert_eq!(output, None, "trap renders its own object");
    let text = String::from_utf8(out).expect("output is utf-8");
    assert_eq!(text.lines().count(), 1, "{text}");
    let value: Value = serde_json::from_str(text.trim_end()).expect("one object");
    assert_eq!(value["command"], "trap");
    assert_eq!(value["addr"], 0x100);
    assert_eq!(value["slot"], 1);
    assert_eq!(value["hit"], true);
}
