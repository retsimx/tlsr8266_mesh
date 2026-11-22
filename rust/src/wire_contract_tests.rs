//! Cross-boundary wire-contract golden tests (R-2, issue #38).
//!
//! Every vector in `tests/wire_vectors.txt` that the firmware owns is asserted
//! here against the real production types and primitives. The Python host
//! tooling asserts the same fixture from `utilities/meshutils/tests/`; both
//! suites reading one committed file is what pins the shared protocol.
//!
//! The fixture also contains keys that only the Python suite owns (OTA packet
//! framing and the generated version mask/blob); those are listed in
//! [`PYTHON_ONLY_KEYS`] and are deliberately not consumed here.

use crate::config::{FLASH_ADR_LIGHT_NEW_FW, MESH_LTK, MESH_NAME, MESH_PWD, OUT_OF_MESH};
use crate::logging::{UART_CRC_OFFSET, UART_PAYLOAD_OFFSET};
use crate::ota::{FLASH_ADR_OTA_READY_FLAG, FLASH_OTA_READY_FLAG};
use crate::sdk::ble_app::ble_ll_pair::{
    PairState, PAIR_OP_DELETE_PAIRING, PAIR_OP_EXCHANGE_RANDOM, PAIR_OP_GET_MESH_LTK,
    PAIR_OP_RESET_MESH, PAIR_OP_SET_MESH_LTK, PAIR_OP_SET_MESH_NAME, PAIR_OP_SET_MESH_PASSWORD,
    PAIR_OP_VERIFY_CREDENTIALS,
};
use crate::sdk::common::crc::crc16;
use crate::sdk::drivers::uart::UART_DATA_LEN;
use crate::sdk::light::{
    LGT_CMD_END_OTA_REQ, LGT_CMD_OTA_DATA_REQ, LGT_CMD_OTA_DATA_RSP, LGT_CMD_START_OTA_REQ,
    LGT_CMD_START_OTA_RSP,
};
use crate::sdk::mcu::crypto::{
    aes_att_decryption_packet, aes_att_encryption, aes_att_encryption_packet, aes_ll_encryption,
    mock_aes_att_encryption, mock_aes_ll_encryption, AesDirection,
};
use crate::sdk::packet_types::{
    AppCmdValue, MeshPkt, Packet, PacketAttCmd, PacketAttData, PacketAttReadRsp, PacketAttValue,
    PacketL2capData, PacketL2capHead, PacketLlApp, PacketLlData, PacketLlInit,
    RfPacketAdvIndModuleT,
};
use crate::sdk::service::{
    TELINK_SPP_DATA_CLIENT2SERVER, TELINK_SPP_DATA_OTA, TELINK_SPP_DATA_PAIR,
    TELINK_SPP_DATA_SERVER2CLIENT,
};
use crate::uart_manager::{UartMsg, CRC_RANGE, MSG_TYPE_INDEX, SNO_RANGE};
use core::mem::size_of;
use mry::Any;

const FIXTURE: &str = include_str!("../../tests/wire_vectors.txt");

/// Fixture keys owned by the Python suite only: the host-only `ENABLE_UART`
/// opcode, the generated OTA version mask/blob, the OTA packet framing built by
/// the Python `PacketParser` subclasses, and the host-derived session key.
const PYTHON_ONLY_KEYS: &[&str] = &[
    "uart.op.enable_uart",
    "ota.version_mask",
    "ota.blob",
    "ota.ble.packet_size",
    "ota.ble.chunk_size",
    "ota.ble.packet.0",
    "ota.ble.packet.1",
    "ota.ble.packet.2",
    "ota.ble.packet.3",
    "ota.uart.packet_size",
    "ota.uart.chunk_size",
    "ota.uart.packet.0",
    "ota.uart.packet.1",
    "ota.uart.packet.2",
    "ota.uart.packet.3",
    "ota.uart.packet.4",
    "ota.uart.packet.5",
    "ota.uart.packet.6",
    "ota.uart.packet.7",
    "pair.session_key.shared_key",
    "pair.session_key.credentials",
    "pair.session_key.response",
    "pair.session_key.session_key",
    "pair.session_key.response_invalid",
];

// --- fixture parsing ------------------------------------------------------

/// Parses the fixture into ordered `(key, value)` pairs, rejecting malformed
/// lines and duplicate keys.
fn parse_fixture(source: &str) -> Vec<(String, String)> {
    let mut vectors: Vec<(String, String)> = Vec::new();

    for line in source.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        let (key, value) = match line.split_once('=') {
            Some((key, value)) => (key.trim(), value.trim()),
            None => panic!("fixture line has no `=`: {line:?}"),
        };
        if key.is_empty() || value.is_empty() {
            panic!("fixture line has an empty key or value: {line:?}");
        }
        if vectors.iter().any(|(existing, _)| existing == key) {
            panic!("fixture key `{key}` is duplicated");
        }

        vectors.push((key.to_owned(), value.to_owned()));
    }

    vectors
}

fn parse_u32(key: &str, value: &str) -> u32 {
    let parsed = match value.strip_prefix("0x") {
        Some(hex) => u32::from_str_radix(hex, 16),
        None => value.parse::<u32>(),
    };

    match parsed {
        Ok(parsed) => parsed,
        Err(_) => panic!("fixture key `{key}` has a malformed integer: {value:?}"),
    }
}

fn parse_bytes(key: &str, value: &str) -> Vec<u8> {
    let mut bytes = Vec::new();
    for token in value.split_whitespace() {
        if token.len() != 2 {
            panic!("fixture key `{key}` has a malformed byte: {token:?}");
        }

        match u8::from_str_radix(token, 16) {
            Ok(byte) => bytes.push(byte),
            Err(_) => panic!("fixture key `{key}` has a malformed byte: {token:?}"),
        }
    }

    if bytes.is_empty() {
        panic!("fixture key `{key}` has no bytes");
    }

    bytes
}

fn exact<const N: usize>(key: &str, bytes: Vec<u8>) -> [u8; N] {
    match <[u8; N]>::try_from(bytes.as_slice()) {
        Ok(array) => array,
        Err(_) => panic!("fixture key `{key}` must hold exactly {N} bytes"),
    }
}

fn hex_uuid(p: &mut Parser, key: &str) -> [u8; 16] {
    let text = p.str(key);
    if text.len() != 32 {
        panic!("fixture key `{key}` must be 32 hex characters, got {text:?}");
    }

    let mut bytes = [0u8; 16];
    for (index, byte) in bytes.iter_mut().enumerate() {
        let pair = &text[index * 2..index * 2 + 2];
        *byte = match u8::from_str_radix(pair, 16) {
            Ok(byte) => byte,
            Err(_) => panic!("fixture key `{key}` has malformed hex {pair:?}"),
        };
    }

    // The fixture stores canonical display order; the firmware constants are
    // LSB-first.
    bytes.reverse();
    bytes
}

/// Fixture accessor that records which keys were consumed, so
/// [`fixture_has_no_orphan_vectors`] can prove nothing in the file is dead.
struct Parser {
    vectors: Vec<(String, String)>,
    consumed: Vec<String>,
}

impl Parser {
    fn new() -> Self {
        Self {
            vectors: parse_fixture(FIXTURE),
            consumed: Vec::new(),
        }
    }

    fn contains(&self, key: &str) -> bool {
        self.vectors.iter().any(|(candidate, _)| candidate == key)
    }

    fn consumed_keys(&self) -> &[String] {
        &self.consumed
    }

    fn take(&mut self, key: &str) -> String {
        if self.consumed.iter().any(|candidate| candidate == key) {
            panic!("fixture key `{key}` consumed twice");
        }

        let value = self
            .vectors
            .iter()
            .find(|(candidate, _)| candidate == key)
            .map(|(_, value)| value.clone());

        match value {
            Some(value) => {
                self.consumed.push(key.to_owned());
                value
            }
            None => panic!("fixture is missing key `{key}`"),
        }
    }

    fn str(&mut self, key: &str) -> String {
        self.take(key)
    }

    fn u32(&mut self, key: &str) -> u32 {
        parse_u32(key, &self.take(key))
    }

    fn u8(&mut self, key: &str) -> u8 {
        let value = self.u32(key);
        match u8::try_from(value) {
            Ok(value) => value,
            Err(_) => panic!("fixture key `{key}` does not fit in a u8: {value:#x}"),
        }
    }

    fn bytes(&mut self, key: &str) -> Vec<u8> {
        parse_bytes(key, &self.take(key))
    }
}

// --- namespace readers and checks -----------------------------------------
//
// Every namespace is split in two: a `read_*` function that only consumes keys
// (returning the fixture values where a check needs them) and a `check_*`
// function that compares those values against the production code. The
// accounting test calls only the readers, so a value mismatch can fail only the
// test that owns it, while both halves draw on one key list and cannot drift.

/// `(fixture key, expected size)` for every packed struct the firmware owns.
fn struct_size_expectations() -> Vec<(&'static str, usize)> {
    vec![
        (
            "size.RfPacketAdvIndModuleT",
            size_of::<RfPacketAdvIndModuleT>(),
        ),
        ("size.PacketAttValue", size_of::<PacketAttValue>()),
        ("size.PacketL2capHead", size_of::<PacketL2capHead>()),
        ("size.PacketL2capData", size_of::<PacketL2capData>()),
        ("size.PacketLlData", size_of::<PacketLlData>()),
        ("size.AppCmdValue", size_of::<AppCmdValue>()),
        ("size.PacketLlApp", size_of::<PacketLlApp>()),
        ("size.PacketAttReadRsp", size_of::<PacketAttReadRsp>()),
        ("size.PacketLlInit", size_of::<PacketLlInit>()),
        ("size.PacketAttData", size_of::<PacketAttData>()),
        ("size.PacketAttCmd", size_of::<PacketAttCmd>()),
        ("size.MeshPkt", size_of::<MeshPkt>()),
        ("size.Packet", size_of::<Packet>()),
        ("size.LumSaveT", size_of::<crate::light_manager::LumSaveT>()),
        (
            "size.UartData",
            size_of::<crate::sdk::drivers::uart::UartData>(),
        ),
    ]
}

fn read_struct_sizes(p: &mut Parser) {
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

/// `(input, expected)` for every `crc.vN` vector.
fn read_crc16(p: &mut Parser) -> Vec<(Vec<u8>, u32)> {
    (1..=5)
        .map(|version| {
            (
                p.bytes(&format!("crc.v{version}.input")),
                p.u32(&format!("crc.v{version}.expect")),
            )
        })
        .collect()
}

fn check_crc16(p: &mut Parser) {
    for (index, (input, expected)) in read_crc16(p).into_iter().enumerate() {
        let version = index + 1;
        let expected = match u16::try_from(expected) {
            Ok(expected) => expected,
            Err(_) => {
                panic!("fixture key `crc.v{version}.expect` does not fit in a u16: {expected:#x}")
            }
        };

        assert_eq!(
            crc16(&input),
            expected,
            "crc16 mismatch for `crc.v{version}.expect`"
        );
    }
}

/// `(fixture key, expected opcode)` for the UART commands and OTA requests.
fn uart_opcode_expectations() -> Vec<(&'static str, u8)> {
    vec![
        ("uart.op.light_ctrl", u8::from(UartMsg::LightCtrl)),
        ("uart.op.light_status", u8::from(UartMsg::LightStatus)),
        ("uart.op.mesh_message", u8::from(UartMsg::MeshMessage)),
        ("uart.op.panic_message", u8::from(UartMsg::PanicMessage)),
        ("uart.op.print_message", u8::from(UartMsg::PrintMessage)),
        ("uart.op.node_status", u8::from(UartMsg::NodeStatus)),
        ("uart.op.ack", u8::from(UartMsg::Ack)),
        ("uart.ota.start", LGT_CMD_START_OTA_REQ),
        ("uart.ota.start_resp", LGT_CMD_START_OTA_RSP),
        ("uart.ota.data", LGT_CMD_OTA_DATA_REQ),
        ("uart.ota.data_resp", LGT_CMD_OTA_DATA_RSP),
        ("uart.ota.end", LGT_CMD_END_OTA_REQ),
    ]
}

fn read_uart_opcodes(p: &mut Parser) {
    for (key, _) in uart_opcode_expectations() {
        let _ = p.u8(key);
    }
}

fn check_uart_opcodes(p: &mut Parser) {
    for (key, expected) in uart_opcode_expectations() {
        assert_eq!(p.u8(key), expected, "`{key}` mismatch");
    }
}

struct UartLayout {
    packet_length: usize,
    payload_offset: usize,
    crc_offset: usize,
    payload_len: usize,
}

fn read_uart_layout(p: &mut Parser) -> UartLayout {
    UartLayout {
        packet_length: p.u32("uart.layout.packet_length") as usize,
        payload_offset: p.u32("uart.layout.payload_offset") as usize,
        crc_offset: p.u32("uart.layout.crc_offset") as usize,
        payload_len: p.u32("uart.layout.payload_len") as usize,
    }
}

/// Writes the CRC-16-MODBUS over the covered span, little-endian, at the
/// fixture's CRC offset.
fn seal_frame(layout: &UartLayout, mut frame: [u8; UART_DATA_LEN]) -> [u8; UART_DATA_LEN] {
    let crc = crc16(&frame[0..layout.crc_offset]);
    frame[layout.crc_offset] = (crc & 0xff) as u8;
    frame[layout.crc_offset + 1] = ((crc >> 8) & 0xff) as u8;
    frame
}

/// `counter | byte1 | cmd | payload(39) | crc16 LE`.
fn data_frame(
    layout: &UartLayout,
    counter: u8,
    byte1: u8,
    cmd: u8,
    payload: &[u8],
) -> [u8; UART_DATA_LEN] {
    let mut frame = [0u8; UART_DATA_LEN];
    frame[0] = counter;
    frame[1] = byte1;
    frame[2] = cmd;
    frame[layout.payload_offset..layout.payload_offset + payload.len()].copy_from_slice(payload);
    seal_frame(layout, frame)
}

/// Host ACK: `counter | 0xff | sno(3) | zeros | crc16 LE`; no command byte.
fn ack_frame(layout: &UartLayout, counter: u8, byte1: u8, sno: &[u8]) -> [u8; UART_DATA_LEN] {
    let mut frame = [0u8; UART_DATA_LEN];
    frame[0] = counter;
    frame[1] = byte1;
    frame[2..2 + sno.len()].copy_from_slice(sno);
    seal_frame(layout, frame)
}

const FRAME_NAMESPACES: [&str; 3] = ["light_ctrl", "light_status", "node_status"];

/// One `uart.frame.<namespace>` vector: the fields plus the expected bytes.
struct UartFrameVector {
    namespace: &'static str,
    counter: u8,
    byte1: u8,
    cmd: u8,
    payload: Vec<u8>,
    expected: Vec<u8>,
}

/// The `uart.layout.*` and `uart.frame.*` values, read without asserting.
struct UartFrameVectors {
    layout: UartLayout,
    data: Vec<UartFrameVector>,
    ack_counter: u8,
    ack_byte1: u8,
    ack_sno: Vec<u8>,
    ack_expected: Vec<u8>,
}

fn read_uart_frames(p: &mut Parser) -> UartFrameVectors {
    let layout = read_uart_layout(p);

    let data = FRAME_NAMESPACES
        .iter()
        .map(|&namespace| UartFrameVector {
            namespace,
            counter: p.u8(&format!("uart.frame.{namespace}.counter")),
            byte1: p.u8(&format!("uart.frame.{namespace}.byte1")),
            cmd: p.u8(&format!("uart.frame.{namespace}.cmd")),
            payload: p.bytes(&format!("uart.frame.{namespace}.payload")),
            expected: p.bytes(&format!("uart.frame.{namespace}.bytes")),
        })
        .collect();

    UartFrameVectors {
        layout,
        data,
        ack_counter: p.u8("uart.frame.ack.counter"),
        ack_byte1: p.u8("uart.frame.ack.byte1"),
        ack_sno: p.bytes("uart.frame.ack.sno"),
        ack_expected: p.bytes("uart.frame.ack.bytes"),
    }
}

/// Binds the fixture's `uart.layout.*` keys and the ACK `sno` placement to the
/// production constants the firmware actually parses with.
///
/// Without this the frame builders above would be self-consistent but blind:
/// they take every offset from the fixture, so changing `UART_PAYLOAD_OFFSET`,
/// `UART_CRC_OFFSET`, `CRC_RANGE`, `MSG_TYPE_INDEX` or `SNO_RANGE` in the
/// firmware would keep `fixture_uart_frames` green while the two sides drifted.
fn check_uart_layout_bindings(layout: &UartLayout, ack_sno: &[u8]) {
    assert_eq!(
        UART_PAYLOAD_OFFSET as u32, layout.payload_offset as u32,
        "`uart.layout.payload_offset` must equal UART_PAYLOAD_OFFSET"
    );
    assert_eq!(
        UART_CRC_OFFSET as u32, layout.crc_offset as u32,
        "`uart.layout.crc_offset` must equal UART_CRC_OFFSET"
    );
    assert_eq!(
        (UART_CRC_OFFSET - UART_PAYLOAD_OFFSET) as u32,
        layout.payload_len as u32,
        "`uart.layout.payload_len` must equal UART_CRC_OFFSET - UART_PAYLOAD_OFFSET"
    );

    // The command byte sits immediately before the payload, and the CRC covers
    // everything before it.
    assert_eq!(MSG_TYPE_INDEX, 2, "MSG_TYPE_INDEX must stay the third byte");
    assert_eq!(
        MSG_TYPE_INDEX as u32,
        layout.payload_offset as u32 - 1,
        "the command byte must sit immediately before the payload"
    );
    assert_eq!(
        CRC_RANGE,
        0..layout.crc_offset,
        "CRC_RANGE must cover exactly the pre-CRC span"
    );

    // The fixture's ACK `sno` is all zeros, so this pins the field's width and
    // placement rather than its content.
    assert_eq!(
        SNO_RANGE,
        2..5,
        "SNO_RANGE must stay the three bytes at 2..5"
    );
    assert_eq!(
        ack_sno.len(),
        SNO_RANGE.end - SNO_RANGE.start,
        "`uart.frame.ack.sno` must fill SNO_RANGE exactly"
    );
}

fn check_uart_frames(p: &mut Parser) {
    let vectors = read_uart_frames(p);
    let layout = &vectors.layout;

    assert_eq!(
        layout.packet_length, UART_DATA_LEN,
        "`uart.layout.packet_length` must equal UART_DATA_LEN"
    );
    assert_eq!(
        layout.payload_offset + layout.payload_len,
        layout.crc_offset,
        "payload must end where the CRC begins"
    );
    assert_eq!(
        layout.crc_offset + 2,
        layout.packet_length,
        "the two CRC bytes must end the packet"
    );
    check_uart_layout_bindings(layout, &vectors.ack_sno);

    for vector in &vectors.data {
        let namespace = vector.namespace;
        assert_eq!(
            vector.payload.len(),
            layout.payload_len,
            "`uart.frame.{namespace}.payload` must be {} bytes",
            layout.payload_len
        );
        let frame = data_frame(
            layout,
            vector.counter,
            vector.byte1,
            vector.cmd,
            &vector.payload,
        );
        assert_eq!(
            frame.as_slice(),
            vector.expected.as_slice(),
            "`uart.frame.{namespace}.bytes` mismatch"
        );
    }

    assert_eq!(
        vectors.ack_byte1,
        u8::from(UartMsg::Ack),
        "an ACK frame must set byte 1 to the ACK opcode"
    );
    let frame = ack_frame(
        layout,
        vectors.ack_counter,
        vectors.ack_byte1,
        &vectors.ack_sno,
    );
    assert_eq!(
        frame.as_slice(),
        vectors.ack_expected.as_slice(),
        "`uart.frame.ack.bytes` mismatch"
    );
}

/// `(fixture key, expected firmware UUID)` in canonical display order.
fn gatt_uuid_expectations() -> Vec<(&'static str, [u8; 16])> {
    vec![
        ("uuid.server2client", TELINK_SPP_DATA_SERVER2CLIENT),
        ("uuid.client2server", TELINK_SPP_DATA_CLIENT2SERVER),
        ("uuid.ota", TELINK_SPP_DATA_OTA),
        ("uuid.pair", TELINK_SPP_DATA_PAIR),
    ]
}

fn read_gatt_uuids(p: &mut Parser) {
    for (key, _) in gatt_uuid_expectations() {
        let _ = hex_uuid(p, key);
    }
}

fn check_gatt_uuids(p: &mut Parser) {
    for (key, expected) in gatt_uuid_expectations() {
        assert_eq!(hex_uuid(p, key), expected, "`{key}` mismatch");
    }
}

/// `(fixture key, expected opcode)` for the pairing commands.
fn pairing_opcode_expectations() -> Vec<(&'static str, u8)> {
    vec![
        ("pair.op.exchange_random", PAIR_OP_EXCHANGE_RANDOM),
        ("pair.op.set_mesh_name", PAIR_OP_SET_MESH_NAME),
        ("pair.op.set_mesh_password", PAIR_OP_SET_MESH_PASSWORD),
        ("pair.op.set_mesh_ltk", PAIR_OP_SET_MESH_LTK),
        ("pair.op.get_mesh_ltk", PAIR_OP_GET_MESH_LTK),
        ("pair.op.verify_credentials", PAIR_OP_VERIFY_CREDENTIALS),
        ("pair.op.reset_mesh", PAIR_OP_RESET_MESH),
        ("pair.op.delete_pairing", PAIR_OP_DELETE_PAIRING),
    ]
}

/// `(fixture key, expected discriminant)` for the pairing states.
fn pairing_state_expectations() -> Vec<(&'static str, u8)> {
    vec![
        ("pair.state.mesh_effect", PairState::ReceivingMeshLtk as u8),
        ("pair.state.requesting_ltk", PairState::RequestingLtk as u8),
        ("pair.state.pairing_complete", PairState::Completed as u8),
    ]
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

fn read_pairing_constants(p: &mut Parser) {
    for (key, _) in pairing_opcode_expectations() {
        let _ = p.u8(key);
    }
    for (key, _) in pairing_state_expectations() {
        let _ = p.u8(key);
    }
    let _ = read_pairing_defaults(p);
}

fn check_pairing_constants(p: &mut Parser) {
    for (key, expected) in pairing_opcode_expectations() {
        assert_eq!(p.u8(key), expected, "`{key}` mismatch");
    }
    for (key, expected) in pairing_state_expectations() {
        assert_eq!(p.u8(key), expected, "`{key}` mismatch");
    }

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

struct AesBlock {
    key: [u8; 16],
    plain: [u8; 16],
    cipher: [u8; 16],
}

/// `aes.block.N` key/plain/cipher triples, read without asserting.
fn read_aes_blocks(p: &mut Parser) -> Vec<AesBlock> {
    let mut blocks = Vec::new();
    let mut index = 1;

    while p.contains(&format!("aes.block.{index}.key")) {
        let key = exact(
            &format!("aes.block.{index}.key"),
            p.bytes(&format!("aes.block.{index}.key")),
        );
        let plain = exact(
            &format!("aes.block.{index}.plain"),
            p.bytes(&format!("aes.block.{index}.plain")),
        );
        let cipher = exact(
            &format!("aes.block.{index}.cipher"),
            p.bytes(&format!("aes.block.{index}.cipher")),
        );

        blocks.push(AesBlock { key, plain, cipher });
        index += 1;
    }

    blocks
}

struct AesPacket {
    key: [u8; 16],
    ivm: [u8; 8],
    plain: Vec<u8>,
    cipher: Vec<u8>,
    mic: [u8; 2],
}

/// `aes.packet.N` authenticated-packet vectors, read without asserting.
fn read_aes_packets(p: &mut Parser) -> Vec<AesPacket> {
    let mut packets = Vec::new();
    let mut index = 1;

    while p.contains(&format!("aes.packet.{index}.key")) {
        let key = exact(
            &format!("aes.packet.{index}.key"),
            p.bytes(&format!("aes.packet.{index}.key")),
        );
        let ivm = exact(
            &format!("aes.packet.{index}.ivm"),
            p.bytes(&format!("aes.packet.{index}.ivm")),
        );
        let plain = p.bytes(&format!("aes.packet.{index}.plain"));
        let cipher = p.bytes(&format!("aes.packet.{index}.cipher"));
        let mic = exact(
            &format!("aes.packet.{index}.mic"),
            p.bytes(&format!("aes.packet.{index}.mic")),
        );

        packets.push(AesPacket {
            key,
            ivm,
            plain,
            cipher,
            mic,
        });
        index += 1;
    }

    packets
}

/// The `ota.header` vector plus the two flash-layout constants, read without
/// asserting.
struct FirmwareHeader {
    header: Vec<u8>,
    reset_opcode: Vec<u8>,
    ramcode_word: Vec<u8>,
    irq_opcode: Vec<u8>,
    ready_flag_address: u32,
    ready_flag: u8,
    staging_address: u32,
}

fn read_firmware_header(p: &mut Parser) -> FirmwareHeader {
    FirmwareHeader {
        header: p.bytes("ota.header"),
        reset_opcode: p.bytes("ota.header.reset_opcode"),
        ramcode_word: p.bytes("ota.header.ramcode_word"),
        irq_opcode: p.bytes("ota.header.irq_opcode"),
        ready_flag_address: p.u32("ota.flash_adr_ota_ready_flag"),
        ready_flag: p.u8("ota.flash_ota_ready_flag"),
        staging_address: p.u32("ota.flash_adr_light_new_fw"),
    }
}

fn check_firmware_header(p: &mut Parser) {
    let vectors = read_firmware_header(p);
    let header = &vectors.header;

    assert_eq!(
        header.len(),
        32,
        "`ota.header` must be the first 32 bytes of the image"
    );

    assert_eq!(
        &header[0..2],
        vectors.reset_opcode.as_slice(),
        "`ota.header` must start with `ota.header.reset_opcode`"
    );

    let version = u32::from_le_bytes([header[2], header[3], header[4], header[5]]);
    assert_eq!(
        version & 0x8000_0000,
        0,
        "the header version word must have bit 31 clear"
    );
    assert_eq!(
        crate::version::BUILD_VERSION & 0x8000_0000,
        0,
        "the live BUILD_VERSION must have bit 31 clear"
    );

    assert_eq!(
        &header[6..8],
        &[0u8, 0u8],
        "header[6..8] must be the .org 0x8 gap"
    );
    assert_eq!(
        &header[8..12],
        &[0x4b, 0x4e, 0x4c, 0x54],
        "header magic 0x544c4e4b must land on flash as `KNLT`"
    );

    assert_eq!(
        &header[12..16],
        vectors.ramcode_word.as_slice(),
        "`ota.header` must carry `ota.header.ramcode_word`"
    );
    assert_eq!(
        &header[16..18],
        vectors.irq_opcode.as_slice(),
        "`ota.header` must carry `ota.header.irq_opcode`"
    );
    assert_eq!(
        &header[18..24],
        &[0u8; 6],
        "header[18..24] must be the .org 0x18 gap"
    );

    let bin_size = u32::from_le_bytes([header[24], header[25], header[26], header[27]]);
    assert!(
        bin_size > 0,
        "header[24..28] must carry a non-zero image size"
    );
    assert_eq!(
        bin_size % 4,
        0,
        "the image size word must stay word-aligned"
    );

    assert_eq!(
        FLASH_ADR_OTA_READY_FLAG, vectors.ready_flag_address,
        "`ota.flash_adr_ota_ready_flag` mismatch"
    );
    assert_eq!(
        FLASH_OTA_READY_FLAG, vectors.ready_flag,
        "`ota.flash_ota_ready_flag` mismatch"
    );
    assert_eq!(
        FLASH_ADR_LIGHT_NEW_FW, vectors.staging_address,
        "`ota.flash_adr_light_new_fw` mismatch"
    );
}

/// Consumes every Rust-owned key without asserting anything, so
/// [`fixture_has_no_orphan_vectors`] fails only for accounting problems.
fn read_all(p: &mut Parser) {
    read_struct_sizes(p);
    let _ = read_crc16(p);
    read_uart_opcodes(p);
    let _ = read_uart_frames(p);
    read_gatt_uuids(p);
    read_pairing_constants(p);
    let _ = read_aes_blocks(p);
    let _ = read_aes_packets(p);
    let _ = read_firmware_header(p);
}

// --- tests ----------------------------------------------------------------

#[test]
fn fixture_struct_sizes() {
    check_struct_sizes(&mut Parser::new());
}

#[test]
fn fixture_crc16() {
    check_crc16(&mut Parser::new());
}

#[test]
fn fixture_uart_opcodes() {
    check_uart_opcodes(&mut Parser::new());
}

#[test]
fn fixture_uart_frames() {
    check_uart_frames(&mut Parser::new());
}

#[test]
fn fixture_gatt_uuids() {
    check_gatt_uuids(&mut Parser::new());
}

#[test]
fn fixture_pairing_constants() {
    check_pairing_constants(&mut Parser::new());
}

/// Pins `aes_att_encryption`'s outer byte-reversal convention and its
/// `(key, plain)` -> `aes_ll_encryption` wiring.
///
/// This is a convention test, not a value test: the mock returns the very
/// fixture cipher byte-reversed, so it cannot prove that the block table's
/// `cipher` values are right. Value parity for that table comes from
/// [`fixture_aes_packets`] (the committed table drives the real CTR+CBC-MAC
/// path) and from Python's `AesBlockTests.test_round_trip`.
#[test]
#[mry::lock(aes_att_encryption, aes_ll_encryption)]
fn fixture_aes_blocks() {
    let blocks = read_aes_blocks(&mut Parser::new());
    assert!(
        !blocks.is_empty(),
        "the fixture has no `aes.block.*` vectors"
    );

    // Partial mock: `aes_att_encryption` runs for real, and its inner
    // `aes_ll_encryption` is stubbed with the byte-reversed fixture output,
    // so only the outer reversal convention is exercised.
    mock_aes_att_encryption(Any, Any).calls_real_impl();

    for block in &blocks {
        let mut stubbed = block.cipher;
        stubbed.reverse();
        mock_aes_ll_encryption(block.key, block.plain, AesDirection::Encrypt).returns(stubbed);
    }

    for block in &blocks {
        assert_eq!(
            aes_att_encryption(&block.key, &block.plain),
            block.cipher,
            "aes_att_encryption mismatch for fixture block {:02x?}",
            block.plain
        );
    }
}

/// Value parity for the AES block table and the authenticated-packet path: the
/// committed `(key, plain) -> cipher` set drives the real CTR+CBC-MAC code.
#[test]
#[mry::lock(aes_att_encryption)]
fn fixture_aes_packets() {
    let mut p = Parser::new();
    let blocks = read_aes_blocks(&mut p);
    let packets = read_aes_packets(&mut p);
    assert!(
        !blocks.is_empty(),
        "the fixture has no `aes.block.*` vectors"
    );
    assert!(
        !packets.is_empty(),
        "the fixture has no `aes.packet.*` vectors"
    );

    // The committed block table is the complete (key, plain) -> cipher set that
    // the packet CTR+CBC-MAC path feeds to the AES primitive.
    for block in &blocks {
        mock_aes_att_encryption(block.key, block.plain).returns(block.cipher);
    }
    // A missing pair falls through to a zero block, which fails the assertions
    // below loudly instead of silently weakening the test.
    mock_aes_att_encryption(Any, Any).returns([0u8; 16]);

    for packet in &packets {
        let mut ciphertext = packet.plain.clone();
        let mut mic = [0u8; 2];
        aes_att_encryption_packet(&packet.key, &packet.ivm, &mut mic, &mut ciphertext);
        assert_eq!(ciphertext, packet.cipher, "CTR ciphertext mismatch");
        assert_eq!(mic, packet.mic, "CBC-MAC MIC mismatch");

        let mut recovered = packet.cipher.clone();
        assert_eq!(
            aes_att_decryption_packet(&packet.key, &packet.ivm, &packet.mic, &mut recovered),
            Ok(()),
            "decryption rejected a valid MIC"
        );
        assert_eq!(recovered, packet.plain, "decrypted plaintext mismatch");

        let mut corrupted_mic = packet.mic;
        corrupted_mic[0] ^= 0xff;
        let mut ignored = packet.cipher.clone();
        assert_eq!(
            aes_att_decryption_packet(&packet.key, &packet.ivm, &corrupted_mic, &mut ignored),
            Err(()),
            "decryption accepted a corrupted MIC"
        );
    }
}

#[test]
fn fixture_firmware_header() {
    check_firmware_header(&mut Parser::new());
}

#[test]
fn fixture_has_no_orphan_vectors() {
    let mut p = Parser::new();
    read_all(&mut p);

    let all_keys = parse_fixture(FIXTURE);
    let consumed = p.consumed_keys();

    for (key, _) in &all_keys {
        if PYTHON_ONLY_KEYS.contains(&key.as_str()) {
            continue;
        }

        assert!(
            consumed.iter().any(|candidate| candidate == key),
            "fixture key `{key}` is never consumed by the Rust suite"
        );
    }

    for key in consumed {
        assert!(
            !PYTHON_ONLY_KEYS.contains(&key.as_str()),
            "the Rust suite consumed Python-only key `{key}`"
        );
    }

    assert_eq!(
        consumed.len(),
        all_keys.len() - PYTHON_ONLY_KEYS.len(),
        "consumed-key count does not match the fixture minus the Python-only allowlist"
    );
}
