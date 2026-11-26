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
//! defaults, and the AES vectors driven through the real crypto path (the HAL's
//! `SoftwareAes` on host, hardware AES on device).
//!
//! The fixture also contains keys that only the Python suite owns (OTA packet
//! framing and the generated version mask/blob); those are listed in
//! [`PYTHON_ONLY_KEYS`] and are deliberately not consumed here.
//!
//! This root owns the fixture parser and the no-orphan accounting test. The
//! assertions are grouped by namespace: [`wire`] (packed sizes, UART opcodes,
//! pairing defaults), [`crypto`] (AES blocks/packets and command encryption)
//! and [`firmware`] (OTA header and flash layout).

mod crypto;
mod firmware;
mod wire;

const FIXTURE: &str = include_str!("../../../tests/wire_vectors.txt");

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

// --- namespace readers ----------------------------------------------------
//
// Every namespace is split in two: a `read_*` function that only consumes keys
// (returning the fixture values where a check needs them) and a `check_*`
// function that compares those values against the production code. The
// accounting test calls only the readers, so a value mismatch can fail only the
// test that owns it, while both halves draw on one key list and cannot drift.

/// Consumes every Rust-owned key without asserting anything, so
/// [`fixture_has_no_orphan_vectors`] fails only for accounting problems.
fn read_all(p: &mut Parser) {
    wire::read_struct_sizes(p);
    wire::read_uart_opcodes(p);
    wire::read_pairing_constants(p);
    let _ = crypto::read_aes_blocks(p);
    let _ = crypto::read_aes_packets(p);
    let _ = crypto::read_cmd_actions(p);
    let _ = firmware::read_firmware_header(p);
}

// --- tests ----------------------------------------------------------------

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
