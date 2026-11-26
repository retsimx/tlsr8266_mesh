//! `wire` namespace: firmware-owned packed struct sizes, the OTA UART
//! sub-command opcodes, and the public pairing defaults.
//!
//! The rest of the UART frame layout and the pairing opcode/state
//! discriminants are `mesh-protocol`-owned and asserted in that crate.

use super::Parser;
use crate::config::{MESH_LTK, MESH_NAME, MESH_PWD, OUT_OF_MESH};
use core::mem::size_of;
use mesh_protocol::mesh_types::{
    LGT_CMD_END_OTA_REQ, LGT_CMD_OTA_DATA_REQ, LGT_CMD_OTA_DATA_RSP, LGT_CMD_START_OTA_REQ,
    LGT_CMD_START_OTA_RSP,
};

/// `(fixture key, expected size)` for the packed structs the firmware owns.
///
/// Every protocol struct now lives in `mesh-protocol` and is size-asserted
/// there; the light manager's persisted luminance record is firmware-owned.
fn struct_size_expectations() -> Vec<(&'static str, usize)> {
    vec![("size.LumSaveT", size_of::<crate::light::LuminanceRecord>())]
}

pub(super) fn read_struct_sizes(p: &mut Parser) {
    for (key, _) in struct_size_expectations() {
        let _ = p.u32(key);
    }
}

fn check_struct_sizes(p: &mut Parser) {
    for (key, expected) in struct_size_expectations() {
        assert_eq!(
            p.u32(key) as usize,
            expected,
            "struct size mismatch for `{key}`"
        );
    }
}

/// `(fixture key, expected opcode)` for the OTA sub-commands the firmware's
/// light-command layer dispatches. The `uart.op.*` command bytes belong to the
/// `UartMsg` enum in `mesh-protocol`.
fn uart_opcode_expectations() -> Vec<(&'static str, u8)> {
    vec![
        ("uart.ota.start", LGT_CMD_START_OTA_REQ),
        ("uart.ota.start_resp", LGT_CMD_START_OTA_RSP),
        ("uart.ota.data", LGT_CMD_OTA_DATA_REQ),
        ("uart.ota.data_resp", LGT_CMD_OTA_DATA_RSP),
        ("uart.ota.end", LGT_CMD_END_OTA_REQ),
    ]
}

pub(super) fn read_uart_opcodes(p: &mut Parser) {
    for (key, _) in uart_opcode_expectations() {
        let _ = p.u8(key);
    }
}

fn check_uart_opcodes(p: &mut Parser) {
    for (key, expected) in uart_opcode_expectations() {
        assert_eq!(p.u8(key), expected, "`{key}` mismatch");
    }
}

/// The public pairing defaults, read without asserting.
struct PairingDefaults {
    mesh_name: String,
    password: String,
    ltk: Vec<u8>,
}

fn read_pairing_defaults(p: &mut Parser) -> PairingDefaults {
    PairingDefaults {
        mesh_name: p.str("pair.default.mesh_name"),
        password: p.str("pair.default.password"),
        ltk: p.bytes("pair.default.ltk"),
    }
}

pub(super) fn read_pairing_constants(p: &mut Parser) {
    let _ = read_pairing_defaults(p);
}

fn check_pairing_constants(p: &mut Parser) {
    let defaults = read_pairing_defaults(p);
    assert_eq!(
        MESH_NAME, defaults.mesh_name,
        "`pair.default.mesh_name` mismatch"
    );
    assert_eq!(
        OUT_OF_MESH, defaults.mesh_name,
        "`pair.default.mesh_name` must be the out-of-mesh default"
    );
    assert_eq!(
        MESH_PWD, defaults.password,
        "`pair.default.password` mismatch"
    );
    assert_eq!(
        defaults.ltk,
        MESH_LTK.as_slice(),
        "`pair.default.ltk` mismatch"
    );
}

#[test]
fn fixture_struct_sizes() {
    check_struct_sizes(&mut Parser::new());
}

#[test]
fn fixture_uart_opcodes() {
    check_uart_opcodes(&mut Parser::new());
}

#[test]
fn fixture_pairing_constants() {
    check_pairing_constants(&mut Parser::new());
}
