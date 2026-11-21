//! Behavioural tests for `inspect` over the scripted transport.
//!
//! Every test asserts the exact request sequence, so a reordered or extra
//! request fails here rather than on the probe.

use std::ffi::OsStr;
use std::path::Path;

use super::*;
use crate::commands::run;
use crate::commands::test_support::*;
use crate::transport::MockTransport;
use crate::Command;

mod fixture;
mod rtt;
mod trap;

use fixture::*;

#[test]
fn register_block_must_be_exactly_128_bytes() {
    for (len, message) in [
        (127, "short read: got 127, want 128"),
        (129, "long read: got 129, want 128"),
    ] {
        let mut responses = stall_prefix().to_vec();
        responses.push(data(vec![0u8; len]));
        let err = run_inspect(responses, None).expect_err("a wrong-sized register block must fail");
        assert_eq!(err, CliError::Device(message.to_owned()), "{len} bytes");
    }
}

#[test]
fn decodes_registers_psr_and_shadow_and_reads_the_stack_tail() {
    let mut mock = MockTransport::new(
        stall_prefix()
            .into_iter()
            .chain([
                regs_data(regs(0x0080_bfc0, 0x20, 0x104), 0x0100_0000, shadow()),
                words_data(&[0u32; 16]),
            ])
            .chain(tail(0x00, 0x2a)),
    );

    let report = inspect_report(&mut mock, None, None).expect("inspect succeeds");

    let expected = Regs {
        r0: 0,
        r1: 1,
        r2: 2,
        r3: 3,
        r4: 4,
        r5: 5,
        r6: 6,
        r7: 7,
        r8: 8,
        r9: 9,
        r10: 10,
        r11: 11,
        r12: 12,
        sp: 0x0080_bfc0,
        lr: 0x20,
        pc: 0x104,
        psr: 0x0100_0000,
    };
    assert_eq!(report.regs, expected);
    assert_eq!(report.shadow, shadow());
    assert_eq!(report.sentinel, 0x00);
    assert_eq!(report.sentinel_state, "CleanReboot");
    assert_eq!(report.breadcrumb, 0x2a);
    assert_eq!(report.panic, None);
    assert_eq!(report.irq_stack, None);
    assert_eq!(
        report.stack,
        StackDump {
            addr: 0x0080_bfc0,
            len: 64,
            words: vec![0; 16],
        }
    );

    assert_stall_requests(&mock);
    assert_eq!(
        &mock.requests[5..],
        [
            Request::ReadRegs,
            read_mem_req(0xbfc0, 64),
            Request::ReadAnalog {
                addr: RESET_SENTINEL_ADDR,
            },
            Request::ReadAnalog {
                addr: BREADCRUMB_ADDR,
            },
            read_flash_req(PANIC_SLOT_ADDR, 3),
        ]
    );
}

#[test]
fn sp_at_the_stack_top_reads_no_stack() {
    let mut mock = MockTransport::new(
        stall_prefix()
            .into_iter()
            .chain([regs_data(regs(0x0080_c000, 0, 0), 0, shadow())])
            .chain(tail(0xa5, 3)),
    );

    let report = inspect_report(&mut mock, None, None).expect("inspect succeeds");

    assert_eq!(
        report.stack,
        StackDump {
            addr: 0x0080_c000,
            len: 0,
            words: Vec::new(),
        }
    );
    let reads = mock
        .requests
        .iter()
        .filter(|request| matches!(request, Request::ReadMem { .. }))
        .count();
    assert_eq!(
        reads, 2,
        "only the status and watchdog reads may happen: {:?}",
        mock.requests
    );
}

#[test]
fn sp_below_the_floor_clamps_to_the_floor() {
    let mut mock = MockTransport::new(
        stall_prefix()
            .into_iter()
            .chain([
                regs_data(regs(0x0080_1000, 0, 0), 0, shadow()),
                words_data(&[0u32; 32]),
            ])
            .chain(tail(0x00, 0)),
    );

    let report = inspect_report(&mut mock, None, None).expect("inspect succeeds");

    assert_eq!(report.stack.addr, MAIN_STACK_FLOOR);
    assert_eq!(report.stack.len, STACK_SPAN);
    assert_eq!(report.stack.words.len(), 32);
    assert_eq!(mock.requests[6], read_mem_req(0x8000, 128));
}

#[test]
fn irq_stack_reads_the_top_128_bytes_of_the_symbol() {
    let elf = image(("irq_stk", 0x0080_af00, 0x700));
    let mut mock = MockTransport::new(
        stall_prefix()
            .into_iter()
            .chain([
                regs_data(regs(0x0080_c000, 0, 0), 0, shadow()),
                words_data(&[0x11; 32]),
            ])
            .chain(tail(0x00, 0)),
    );

    let report = inspect_report(&mut mock, Some(&elf), None).expect("inspect succeeds");

    assert_eq!(
        report.irq_stack,
        Some(StackDump {
            addr: 0x0080_b580,
            len: 128,
            words: vec![0x11; 32],
        })
    );
    assert_eq!(mock.requests[6], read_mem_req(0xb580, 128));
}

#[test]
fn zero_sized_irq_stack_uses_the_documented_default() {
    let elf = image(("irq_stk", 0x0080_b000, 0));
    let mut mock = MockTransport::new(
        stall_prefix()
            .into_iter()
            .chain([
                regs_data(regs(0x0080_c000, 0, 0), 0, shadow()),
                words_data(&[0u32; 32]),
            ])
            .chain(tail(0x00, 0)),
    );

    let report = inspect_report(&mut mock, Some(&elf), None).expect("inspect succeeds");

    let irq = report.irq_stack.expect("irq_stk resolves");
    assert_eq!(irq.addr, 0x0080_b000 + IRQ_STK_DEFAULT - STACK_SPAN);
    assert_eq!(irq.len, STACK_SPAN);
    assert_eq!(
        mock.requests[6],
        read_mem_req(
            (0x0080_b000 + IRQ_STK_DEFAULT - STACK_SPAN - SRAM_WINDOW_BASE) as u16,
            128
        )
    );
}

#[test]
fn absent_irq_stk_symbol_reads_nothing() {
    let elf = no_irq_image();
    let mut mock = MockTransport::new(
        stall_prefix()
            .into_iter()
            .chain([regs_data(regs(0x0080_c000, 0, 0), 0, shadow())])
            .chain(tail(0x00, 0)),
    );

    let report = inspect_report(&mut mock, Some(&elf), None).expect("inspect succeeds");

    assert_eq!(report.irq_stack, None);
    let reads = mock
        .requests
        .iter()
        .filter(|request| matches!(request, Request::ReadMem { .. }))
        .count();
    assert_eq!(
        reads, 2,
        "only the status and watchdog reads may happen: {:?}",
        mock.requests
    );
}

#[test]
fn sentinel_states_are_named() {
    assert_eq!(sentinel_state(0x00), "CleanReboot");
    assert_eq!(sentinel_state(0xA5), "Running");
    assert_eq!(sentinel_state(0xE1), "Panic");
    assert_eq!(sentinel_state(0x42), "Unknown");
}

#[test]
fn panic_record_decodes_the_flag_length_and_body() {
    let mut responses = stall_prefix()
        .into_iter()
        .chain([
            regs_data(regs(0x0080_c000, 0, 0), 0, shadow()),
            Response::Byte(0xe1),
            Response::Byte(0xff),
            data([0xde, 5, 0]),
            data(*b"crash"),
        ])
        .collect::<Vec<_>>();

    let mut mock = MockTransport::new(responses.clone());
    let report = inspect_report(&mut mock, None, None).expect("inspect succeeds");
    assert_eq!(
        report.panic,
        Some(PanicBody {
            len: 5,
            message: "crash".to_owned(),
        })
    );
    assert_eq!(report.sentinel_state, "Panic");
    assert_eq!(
        mock.requests.last(),
        Some(&read_flash_req(PANIC_SLOT_ADDR + 3, 5))
    );

    // The body is decoded lossily, never rejected for invalid UTF-8.
    responses.pop();
    responses.pop();
    responses.push(data([0xde, 2, 0]));
    responses.push(data([0xff, b'a']));
    let mut mock = MockTransport::new(responses);
    let report = inspect_report(&mut mock, None, None).expect("inspect succeeds");
    assert_eq!(
        report.panic.map(|panic| panic.message),
        Some("\u{fffd}a".to_owned())
    );
}

#[test]
fn invalid_panic_headers_yield_null_without_a_body_read() {
    for header in [[0x00, 5, 0], [0xde, 0, 0], [0xde, 1, 1]] {
        let mut mock = MockTransport::new(stall_prefix().into_iter().chain([
            regs_data(regs(0x0080_c000, 0, 0), 0, shadow()),
            Response::Byte(0x00),
            Response::Byte(0x00),
            data(header),
        ]));

        let report = inspect_report(&mut mock, None, None).expect("inspect succeeds");

        assert_eq!(report.panic, None, "header {header:?}");
        assert_eq!(
            mock.requests.last(),
            Some(&read_flash_req(PANIC_SLOT_ADDR, 3))
        );
    }
}

#[test]
fn symbolizes_pc_lr_and_stack_words_and_omits_non_code_words() {
    let elf = no_irq_image();
    let mut words = [0u32; 16];
    words[..3].copy_from_slice(&[0x100, 0x104, 0xdead_beef]);
    let mut mock = MockTransport::new(
        stall_prefix()
            .into_iter()
            .chain([
                regs_data(regs(0x0080_bfc0, 0x100, 0x104), 0, shadow()),
                words_data(&words),
            ])
            .chain(tail(0x00, 0)),
    );

    let report = inspect_report(&mut mock, Some(&elf), None).expect("inspect succeeds");

    assert_eq!(report.pc_symbol.as_deref(), Some("sym+0x4"));
    assert_eq!(report.lr_symbol.as_deref(), Some("sym"));
    assert_eq!(
        report.stack_symbols,
        [
            StackSymbol {
                addr: 0x0080_bfc0,
                symbol: "sym".to_owned(),
            },
            StackSymbol {
                addr: 0x0080_bfc4,
                symbol: "sym+0x4".to_owned(),
            },
        ]
    );
}

#[test]
fn inspect_succeeds_without_a_usable_disassembler() {
    let elf = no_irq_image();
    let mut mock = MockTransport::new(
        stall_prefix()
            .into_iter()
            .chain([regs_data(regs(0x0080_c000, 0, 0), 0, shadow())])
            .chain(tail(0x00, 0)),
    );

    // A path no objdump can open: the spawn fails or the tool exits non-zero.
    let missing = Path::new("/nonexistent/probe-tlsr/app.elf");
    let report = inspect_report(&mut mock, Some(&elf), Some(missing)).expect("inspect succeeds");

    assert_eq!(report.disassembly, None);
}

#[test]
fn objdump_path_prefers_llvm_then_legacy_on_path() {
    let dir = TempDir::new("objdump-path");
    let on_path = dir.path().join(OBJDUMP_NAMES[0]);
    std::fs::write(&on_path, b"").expect("tool fixture writes");
    // The legacy tool is a fallback, never preferred over llvm-objdump.
    std::fs::write(dir.path().join(OBJDUMP_NAMES[1]), b"").expect("fallback writes");

    assert_eq!(
        objdump_path(Path::new("/nonexistent"), Some(dir.path().as_os_str())),
        Some(on_path)
    );
    assert_eq!(
        objdump_path(
            Path::new("/nonexistent"),
            Some(OsStr::new("/nonexistent/bin"))
        ),
        None
    );
    assert_eq!(objdump_path(Path::new("/nonexistent"), None), None);
}

#[test]
fn disassemble_runs_the_pc_window_and_degrades_on_failure() {
    let dir = TempDir::new("disassemble");
    let elf = dir.path().join("app.elf");
    std::fs::write(&elf, b"elf").expect("elf fixture writes");

    let tool = script(dir.path(), "echo-objdump", "#!/bin/sh\necho \"$@\"\n");
    assert_eq!(
        disassemble(&tool, &elf, 0x104),
        Some(format!(
            "-d --start-address=0xe4 --stop-address=0x124 {}",
            elf.display()
        ))
    );
    assert_eq!(
        disassemble(&tool, &elf, 0),
        Some(format!(
            "-d --start-address=0x0 --stop-address=0x20 {}",
            elf.display()
        ))
    );

    let llvm = script(dir.path(), "llvm-objdump", "#!/bin/sh\necho \"$@\"\n");
    assert_eq!(
        disassemble(&llvm, &elf, 0x104),
        Some(format!(
            "-d --triple=thumbv6m-none-eabi --mattr=+tc32 \
             --start-address=0xe4 --stop-address=0x124 {}",
            elf.display()
        ))
    );

    let failing = script(dir.path(), "fail-objdump", "#!/bin/sh\nexit 3\n");
    assert_eq!(disassemble(&failing, &elf, 0x104), None);
    assert_eq!(
        disassemble(Path::new("/nonexistent/tc32-elf-objdump"), &elf, 0x104),
        None
    );
}

#[test]
fn run_dispatches_to_inspect() {
    let mut ctx = test_ctx(
        stall_prefix()
            .into_iter()
            .chain([
                regs_data(regs(0x0080_bfc0, 0, 0), 0, shadow()),
                words_data(&[0u32; 16]),
            ])
            .chain(tail(0xa5, 7)),
    );

    let mut out = Vec::new();
    let output = run(Command::Inspect, &mut ctx, &mut out)
        .expect("inspect runs")
        .expect("inspect returns an output");

    let Output::Inspect { report } = output else {
        panic!("expected an inspect output, got {output:?}");
    };
    assert_eq!(report.stack.len, 64);
    assert_eq!(report.sentinel, 0xa5);
    assert_eq!(report.sentinel_state, "Running");
    assert_eq!(report.breadcrumb, 7);
}
