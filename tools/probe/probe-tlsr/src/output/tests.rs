//! Pins for the JSON tags and shapes.
//!
//! `output_cases` is the contract table: one representative value per variant
//! and key combination, compared against its exact serialized form.

mod fixture;
mod human;

use super::{print_output, Output};
use fixture::*;
use serde_json::{json, Value};
use std::collections::BTreeSet;

fn serialized(output: &Output) -> Value {
    serde_json::to_value(output).expect("output serializes")
}

/// One representative value per `Output` variant and key combination, plus
/// the exact JSON it must serialize to. Equality with this map pins the
/// literal `command` tag, the key set and every value.
fn output_cases() -> [(Output, Value); 18] {
    [
        (
            Output::Info {
                pc: 0x1_10f0,
                cpu_status: 0x05,
                cpu_state: "stopped",
                flash_info: Some(flash_info()),
            },
            json!({
                "command": "info", "pc": 69872, "cpu_status": 5,
                "cpu_state": "stopped",
                "flash_info": {
                    "jedec_id": "ef4018",
                    "build_version": 3539, "magic_ok": true
                }
            }),
        ),
        (
            Output::Info {
                pc: 0,
                cpu_status: 0,
                cpu_state: "running",
                flash_info: None,
            },
            json!({
                "command": "info", "pc": 0, "cpu_status": 0,
                "cpu_state": "running", "flash_info": null
            }),
        ),
        (
            Output::Flash {
                path: "build/app.bin".to_owned(),
                bytes: 78996,
                sectors_erased: 20,
                pages_written: 309,
            },
            json!({
                "command": "flash", "path": "build/app.bin", "bytes": 78996,
                "sectors_erased": 20, "pages_written": 309
            }),
        ),
        (
            Output::Flash {
                path: "raw.bin".to_owned(),
                bytes: 4,
                sectors_erased: 1,
                pages_written: 1,
            },
            json!({
                "command": "flash", "path": "raw.bin", "bytes": 4,
                "sectors_erased": 1, "pages_written": 1
            }),
        ),
        (
            Output::FlashRead {
                addr: 0,
                len: 12,
                out: Some("backup.bin".to_owned()),
                data_hex: None,
            },
            json!({
                "command": "flash-read", "addr": 0, "len": 12,
                "out": "backup.bin", "data_hex": null
            }),
        ),
        (
            Output::FlashRead {
                addr: 0x20,
                len: 2,
                out: None,
                data_hex: Some("aabb".to_owned()),
            },
            json!({
                "command": "flash-read", "addr": 32, "len": 2,
                "out": null, "data_hex": "aabb"
            }),
        ),
        (
            Output::FlashErase {
                addr: 0x1000,
                len: 0x2000,
                sectors: 2,
            },
            json!({"command": "flash-erase", "addr": 4096, "len": 8192, "sectors": 2}),
        ),
        (
            Output::FlashWrite {
                addr: 0x8000,
                len: 256,
                pages: 1,
            },
            json!({"command": "flash-write", "addr": 32768, "len": 256, "pages": 1}),
        ),
        (
            Output::Read {
                addr: 0x8000,
                data_hex: "deadbeef".to_owned(),
            },
            json!({"command": "read", "addr": 32768, "data_hex": "deadbeef"}),
        ),
        (
            Output::Write {
                addr: 0x8000,
                len: 4,
            },
            json!({"command": "write", "addr": 32768, "len": 4}),
        ),
        (
            Output::AnalogRead {
                addr: 0x0c,
                value: 0x42,
            },
            json!({"command": "analog-read", "addr": 12, "value": 66}),
        ),
        (
            Output::AnalogWrite {
                addr: 0x0c,
                value: 0xff,
            },
            json!({"command": "analog-write", "addr": 12, "value": 255}),
        ),
        (
            Output::Reset {
                stop: false,
                soft: true,
            },
            json!({"command": "reset", "stop": false, "soft": true}),
        ),
        (
            Output::Inspect {
                report: Box::new(inspect_report()),
            },
            json!({
                "command": "inspect",
                "regs": {
                    "r0": 0, "r1": 1, "r2": 2, "r3": 3, "r4": 4, "r5": 5,
                    "r6": 6, "r7": 7, "r8": 8, "r9": 9, "r10": 10, "r11": 11,
                    "r12": 12, "sp": 0x0080_bfc0, "lr": 0x20, "pc": 0x104,
                    "psr": 0x0100_0000
                },
                "shadow": [
                    0x11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x22
                ],
                "stack": {
                    "addr": 0x0080_bfc0, "len": 8,
                    "words": [0x20, 0xdead_beefu32]
                },
                "irq_stack": {
                    "addr": 0x0080_b580, "len": 4, "words": [0x104]
                },
                "sentinel": 0xa5,
                "sentinel_state": "Running",
                "breadcrumb": 3,
                "panic": {"len": 5, "message": "crash"},
                "pc_symbol": "sym+0x4",
                "lr_symbol": "sym",
                "stack_symbols": [
                    {"addr": 0x0080_bfc0, "symbol": "sym"},
                    {"addr": 0x0080_b580, "symbol": "sym+0x4"}
                ],
                "disassembly": "00000104 <sym>:\n 104:\tpush\t{r4, lr}"
            }),
        ),
        (
            Output::Inspect {
                report: Box::new(empty_report()),
            },
            json!({
                "command": "inspect",
                "regs": {
                    "r0": 0, "r1": 0, "r2": 0, "r3": 0, "r4": 0, "r5": 0,
                    "r6": 0, "r7": 0, "r8": 0, "r9": 0, "r10": 0, "r11": 0,
                    "r12": 0, "sp": 0x0080_c000, "lr": 0, "pc": 0, "psr": 0
                },
                "shadow": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                "stack": {"addr": 0x0080_c000, "len": 0, "words": []},
                "irq_stack": null,
                "sentinel": 0,
                "sentinel_state": "CleanReboot",
                "breadcrumb": 0,
                "panic": null,
                "pc_symbol": null,
                "lr_symbol": null,
                "stack_symbols": [],
                "disassembly": null
            }),
        ),
        (Output::Halt, json!({"command": "halt"})),
        (
            Output::RttLine {
                line: "boot ok".to_owned(),
            },
            json!({"command": "rtt", "line": "boot ok"}),
        ),
        (
            Output::Trap {
                addr: 0x104,
                slot: 2,
                hit: true,
                inspect: Box::new(inspect_report()),
            },
            json!({
                "command": "trap",
                "addr": 0x104,
                "slot": 2,
                "hit": true,
                // Nested, and with no inner `command` key: the trap object
                // is the only thing tagged here.
                "inspect": {
                    "regs": {
                        "r0": 0, "r1": 1, "r2": 2, "r3": 3, "r4": 4, "r5": 5,
                        "r6": 6, "r7": 7, "r8": 8, "r9": 9, "r10": 10, "r11": 11,
                        "r12": 12, "sp": 0x0080_bfc0, "lr": 0x20, "pc": 0x104,
                        "psr": 0x0100_0000
                    },
                    "shadow": [
                        0x11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x22
                    ],
                    "stack": {
                        "addr": 0x0080_bfc0, "len": 8,
                        "words": [0x20, 0xdead_beefu32]
                    },
                    "irq_stack": {
                        "addr": 0x0080_b580, "len": 4, "words": [0x104]
                    },
                    "sentinel": 0xa5,
                    "sentinel_state": "Running",
                    "breadcrumb": 3,
                    "panic": {"len": 5, "message": "crash"},
                    "pc_symbol": "sym+0x4",
                    "lr_symbol": "sym",
                    "stack_symbols": [
                        {"addr": 0x0080_bfc0, "symbol": "sym"},
                        {"addr": 0x0080_b580, "symbol": "sym+0x4"}
                    ],
                    "disassembly": "00000104 <sym>:\n 104:\tpush\t{r4, lr}"
                }
            }),
        ),
    ]
}

fn strings(tags: &[&str]) -> BTreeSet<String> {
    tags.iter().map(|tag| (*tag).to_owned()).collect()
}

#[test]
fn output_variants_serialize_with_literal_tags() {
    for (output, expected) in output_cases() {
        assert_eq!(serialized(&output), expected, "{output:?}");
    }
}

#[test]
fn every_output_variant_has_a_literal_tag() {
    let mut tags = BTreeSet::new();
    for (output, expected) in output_cases() {
        let value = serialized(&output);
        assert_eq!(value["command"], expected["command"], "{output:?}");
        tags.insert(
            value["command"]
                .as_str()
                .expect("tag is a string")
                .to_owned(),
        );
    }

    let expected = strings(&[
        "analog-read",
        "analog-write",
        "flash",
        "flash-erase",
        "flash-read",
        "flash-write",
        "halt",
        "info",
        "inspect",
        "read",
        "reset",
        "write",
        "rtt",
        "trap",
    ]);
    assert_eq!(tags, expected);
}

#[test]
fn json_success_writes_one_object_to_stdout() {
    let mut out = Vec::new();
    print_output(&Output::Halt, true, &mut out).expect("halt serializes");

    assert_eq!(
        String::from_utf8(out).expect("utf-8"),
        "{\"command\":\"halt\"}\n"
    );
}
