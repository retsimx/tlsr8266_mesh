//! Human-rendering pins: one block per success, byte for byte.

use super::fixture::{empty_report, flash_info, inspect_report};
use crate::output::human::human_output;
use crate::output::inspect::inspect_text;
use crate::output::Output;

#[test]
fn human_rtt_lines_are_the_raw_line() {
    let output = Output::RttLine {
        line: "boot ok".to_owned(),
    };
    assert_eq!(human_output(&output), "boot ok");
}

#[test]
fn human_trap_prints_the_hit_line_then_the_inspect_block() {
    let output = Output::Trap {
        addr: 0x104,
        slot: 2,
        hit: true,
        inspect: Box::new(inspect_report()),
    };

    let text = human_output(&output);
    let (first, rest) = text.split_once('\n').expect("hit line then the report");
    assert_eq!(first, "trap hit slot=2 addr=0x000104");
    // Reused renderer, not a copy: the block is byte-identical to `inspect`.
    assert_eq!(rest, inspect_text(&inspect_report()));
}

#[test]
fn human_info_prints_key_value_lines() {
    let running = Output::Info {
        pc: 0x0001_10f0,
        cpu_status: 0x00,
        cpu_state: "running",
        flash_info: None,
    };
    assert_eq!(
        human_output(&running),
        "pc=0x000110f0 cpu_status=0x00 cpu_state=running"
    );

    let stopped = Output::Info {
        pc: 0,
        cpu_status: 0x05,
        cpu_state: "stopped",
        flash_info: Some(flash_info()),
    };
    assert_eq!(
        human_output(&stopped),
        concat!(
            "pc=0x00000000 cpu_status=0x05 cpu_state=stopped\n",
            "flash jedec_id=ef4018 build_version=3539 magic_ok=true"
        )
    );
}

#[test]
fn human_memory_reads_are_sixteen_byte_hex_dumps() {
    let read = Output::Read {
        addr: 0x8000,
        data_hex: "00112233445566778899aabbccddeeff0102".to_owned(),
    };
    assert_eq!(
        human_output(&read),
        concat!(
            "0x8000: 00 11 22 33 44 55 66 77 88 99 aa bb cc dd ee ff\n",
            "0x8010: 01 02"
        )
    );

    let dump = Output::FlashRead {
        addr: 0x20,
        len: 2,
        out: None,
        data_hex: Some("aabb".to_owned()),
    };
    assert_eq!(human_output(&dump), "0x000020: aa bb");

    let backup = Output::FlashRead {
        addr: 0x20,
        len: 2,
        out: Some("backup.bin".to_owned()),
        data_hex: None,
    };
    assert_eq!(
        human_output(&backup),
        "read 2 bytes at 0x000020 into backup.bin"
    );
}

#[test]
fn human_control_commands_are_single_lines() {
    assert_eq!(human_output(&Output::Halt), "halted");
    assert_eq!(
        human_output(&Output::Reset {
            stop: false,
            soft: true,
        }),
        "reset (stop=false soft=true)"
    );
    assert_eq!(
        human_output(&Output::AnalogRead {
            addr: 0x0c,
            value: 0x42,
        }),
        "analog 0x0c = 0x42"
    );
    assert_eq!(
        human_output(&Output::AnalogWrite {
            addr: 0x0c,
            value: 0xff,
        }),
        "analog 0x0c <- 0xff"
    );
    assert_eq!(
        human_output(&Output::Write {
            addr: 0x8000,
            len: 4,
        }),
        "wrote 4 bytes at 0x8000"
    );
    assert_eq!(
        human_output(&Output::FlashErase {
            addr: 0x812,
            len: 0x2000,
            sectors: 3,
        }),
        "erased 3 sectors covering 0x000812+0x2000"
    );
    assert_eq!(
        human_output(&Output::FlashWrite {
            addr: 0x80f0,
            len: 600,
            pages: 4,
        }),
        "wrote 600 bytes at 0x0080f0 in 4 pages"
    );
}

#[test]
fn human_inspect_prints_registers_stacks_and_panic() {
    let output = Output::Inspect {
        report: Box::new(inspect_report()),
    };
    assert_eq!(
        human_output(&output),
        concat!(
            "pc=0x00000104 sym+0x4\n",
            "lr=0x00000020 sym\n",
            "sp=0x0080bfc0\n",
            "psr=0x01000000\n",
            "r0=0x00000000\n",
            "r1=0x00000001\n",
            "r2=0x00000002\n",
            "r3=0x00000003\n",
            "r4=0x00000004\n",
            "r5=0x00000005\n",
            "r6=0x00000006\n",
            "r7=0x00000007\n",
            "r8=0x00000008\n",
            "r9=0x00000009\n",
            "r10=0x0000000a\n",
            "r11=0x0000000b\n",
            "r12=0x0000000c\n",
            "shadow=0x00000011 0x00000000 0x00000000 0x00000000 ",
            "0x00000000 0x00000000 0x00000000 0x00000000 ",
            "0x00000000 0x00000000 0x00000000 0x00000000 ",
            "0x00000000 0x00000022\n",
            "stack 0x0080bfc0 len=8:\n",
            "0x0080bfc0: 00000020 deadbeef\n",
            "irq_stack 0x0080b580 len=4:\n",
            "0x0080b580: 00000104\n",
            "stack_symbols:\n",
            "0x0080bfc0 sym\n",
            "0x0080b580 sym+0x4\n",
            "reset sentinel=0xa5 (Running) breadcrumb=3\n",
            "panic len=5 message=crash\n",
            "disassembly:\n",
            "00000104 <sym>:\n",
            " 104:\tpush\t{r4, lr}\n",
            "core stalled by inspect — run `probe-tlsr reset --soft` to restart it"
        )
    );
}

#[test]
fn human_inspect_omits_everything_that_is_absent() {
    let output = Output::Inspect {
        report: Box::new(empty_report()),
    };
    assert_eq!(
        human_output(&output),
        concat!(
            "pc=0x00000000\n",
            "lr=0x00000000\n",
            "sp=0x0080c000\n",
            "psr=0x00000000\n",
            "r0=0x00000000\n",
            "r1=0x00000000\n",
            "r2=0x00000000\n",
            "r3=0x00000000\n",
            "r4=0x00000000\n",
            "r5=0x00000000\n",
            "r6=0x00000000\n",
            "r7=0x00000000\n",
            "r8=0x00000000\n",
            "r9=0x00000000\n",
            "r10=0x00000000\n",
            "r11=0x00000000\n",
            "r12=0x00000000\n",
            "shadow=0x00000000 0x00000000 0x00000000 0x00000000 ",
            "0x00000000 0x00000000 0x00000000 0x00000000 ",
            "0x00000000 0x00000000 0x00000000 0x00000000 ",
            "0x00000000 0x00000000\n",
            "stack 0x0080c000 len=0 (empty)\n",
            "reset sentinel=0x00 (CleanReboot) breadcrumb=0\n",
            "core stalled by inspect — run `probe-tlsr reset --soft` to restart it"
        )
    );
}
