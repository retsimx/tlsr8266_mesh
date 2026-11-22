//! Cross-boundary wire-contract golden tests (R-2, issue #38).
//!
//! Every vector in `tests/wire_vectors.txt` that the firmware owns is asserted
//! here against the real production types and primitives. The Python host
//! tooling asserts the same fixture from `utilities/meshutils/tests/`; both
//! suites reading one committed file is what pins the shared protocol.
//!
//! The protocol half of the fixture (packed struct sizes, CRC-16-MODBUS, UART
//! frame layout/frames/opcodes, GATT UUIDs, pairing opcodes and states) is
//! asserted by the `mesh-protocol` crate (`rust/crates/mesh-protocol/src/lib.rs`),
//! which now owns those definitions. What remains here is firmware-owned: the
//! light/Ota binding, the OTA sub-commands and flash layout, the pairing
//! defaults, and the AES vectors driven through the firmware's `mry`-mocked
//! crypto path.
//!
//! The fixture also contains keys that only the Python suite owns (OTA packet
//! framing and the generated version mask/blob); those are listed in
//! [`PYTHON_ONLY_KEYS`] and are deliberately not consumed here.

use crate::config::{FLASH_ADR_LIGHT_NEW_FW, MESH_LTK, MESH_NAME, MESH_PWD, OUT_OF_MESH};
use crate::ota::{FLASH_ADR_OTA_READY_FLAG, FLASH_OTA_READY_FLAG};
use crate::sdk::light::{
    LGT_CMD_END_OTA_REQ, LGT_CMD_OTA_DATA_REQ, LGT_CMD_OTA_DATA_RSP, LGT_CMD_START_OTA_REQ,
    LGT_CMD_START_OTA_RSP,
};
use crate::sdk::mcu::crypto::{
    aes_att_decryption_packet, aes_att_encryption, aes_att_encryption_packet, aes_ll_encryption,
    mock_aes_att_encryption, mock_aes_ll_encryption, AesDirection,
};
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

/// Fixture keys asserted by the `mesh-protocol` crate, which owns the packed
/// wire structs, the CRC-16-MODBUS table, the UART frame layout, the GATT UUIDs
/// and the pairing opcodes/states. They stay in the committed fixture and are
/// consumed there, so the orphan check below must not demand them here as well.
const MESH_PROTOCOL_KEYS: &[&str] = &[
    // `size.*` for every packed struct the crate now defines.
    "size.RfPacketAdvIndModuleT",
    "size.PacketAttValue",
    "size.PacketL2capHead",
    "size.PacketL2capData",
    "size.PacketLlData",
    "size.AppCmdValue",
    "size.PacketLlApp",
    "size.PacketAttReadRsp",
    "size.PacketLlInit",
    "size.PacketAttData",
    "size.PacketAttCmd",
    "size.MeshPkt",
    "size.Packet",
    "size.UartData",
    // CRC-16-MODBUS vectors.
    "crc.v1.input",
    "crc.v1.expect",
    "crc.v2.input",
    "crc.v2.expect",
    "crc.v3.input",
    "crc.v3.expect",
    "crc.v4.input",
    "crc.v4.expect",
    "crc.v5.input",
    "crc.v5.expect",
    // UART frame layout.
    "uart.layout.packet_length",
    "uart.layout.payload_offset",
    "uart.layout.crc_offset",
    "uart.layout.payload_len",
    // UART command opcodes (the `uart.ota.*` sub-commands stay firmware-side).
    "uart.op.light_ctrl",
    "uart.op.light_status",
    "uart.op.mesh_message",
    "uart.op.panic_message",
    "uart.op.print_message",
    "uart.op.node_status",
    "uart.op.ack",
    // Full 44-byte frames and the host ACK.
    "uart.frame.light_ctrl.counter",
    "uart.frame.light_ctrl.byte1",
    "uart.frame.light_ctrl.cmd",
    "uart.frame.light_ctrl.payload",
    "uart.frame.light_ctrl.bytes",
    "uart.frame.light_status.counter",
    "uart.frame.light_status.byte1",
    "uart.frame.light_status.cmd",
    "uart.frame.light_status.payload",
    "uart.frame.light_status.bytes",
    "uart.frame.node_status.counter",
    "uart.frame.node_status.byte1",
    "uart.frame.node_status.cmd",
    "uart.frame.node_status.payload",
    "uart.frame.node_status.bytes",
    "uart.frame.ack.counter",
    "uart.frame.ack.byte1",
    "uart.frame.ack.sno",
    "uart.frame.ack.bytes",
    // GATT characteristic UUIDs.
    "uuid.server2client",
    "uuid.client2server",
    "uuid.ota",
    "uuid.pair",
    // Pairing opcodes and state discriminants.
    "pair.op.exchange_random",
    "pair.op.set_mesh_name",
    "pair.op.set_mesh_password",
    "pair.op.set_mesh_ltk",
    "pair.op.get_mesh_ltk",
    "pair.op.verify_credentials",
    "pair.op.reset_mesh",
    "pair.op.delete_pairing",
    "pair.state.mesh_effect",
    "pair.state.requesting_ltk",
    "pair.state.pairing_complete",
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

/// `(fixture key, expected size)` for the packed structs the firmware owns.
///
/// Every protocol struct now lives in `mesh-protocol` and is size-asserted
/// there; the light manager's persisted luminance record is firmware-owned.
fn struct_size_expectations() -> Vec<(&'static str, usize)> {
    vec![("size.LumSaveT", size_of::<crate::light_manager::LumSaveT>())]
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
    read_uart_opcodes(p);
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
fn fixture_uart_opcodes() {
    check_uart_opcodes(&mut Parser::new());
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
        if PYTHON_ONLY_KEYS.contains(&key.as_str()) || MESH_PROTOCOL_KEYS.contains(&key.as_str()) {
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
        assert!(
            !MESH_PROTOCOL_KEYS.contains(&key.as_str()),
            "the firmware suite consumed `mesh-protocol`-owned key `{key}`"
        );
    }

    assert_eq!(
        consumed.len(),
        all_keys.len() - PYTHON_ONLY_KEYS.len() - MESH_PROTOCOL_KEYS.len(),
        "consumed-key count does not match the fixture minus the Python-only and mesh-protocol allowlists"
    );
}
