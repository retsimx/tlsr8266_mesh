//! `mesh-protocol` — the hardware-independent wire protocol of the TLSR8266
//! mesh firmware.
//!
//! Everything in this crate is a byte-level contract shared with the BLE stack,
//! the UART host bridge and the Python tooling in `utilities/`. There is no
//! MMIO, no async runtime and no test-mocking framework here: only packed
//! structs, CRC, frame layout and protocol constants, so the boundary between
//! "protocol" and "hardware" is mechanical instead of conventional.

#![cfg_attr(not(test), no_std)]

#[macro_export]
macro_rules! const_assert {
    ($($tt:tt)*) => {
        const _: () = assert!($($tt)*);
    }
}

pub mod crc;
pub mod mesh_types;
pub mod packet_types;
pub mod service;
pub mod uart;

#[cfg(test)]
mod fixture_tests {
    use crate::crc::crc16;
    use crate::mesh_types::{
        PairState, PAIR_OP_DELETE_PAIRING, PAIR_OP_EXCHANGE_RANDOM, PAIR_OP_GET_MESH_LTK,
        PAIR_OP_RESET_MESH, PAIR_OP_SET_MESH_LTK, PAIR_OP_SET_MESH_NAME, PAIR_OP_SET_MESH_PASSWORD,
        PAIR_OP_VERIFY_CREDENTIALS,
    };
    use crate::packet_types::{
        AppCmdValue, MeshPkt, Packet, PacketAttCmd, PacketAttData, PacketAttReadRsp,
        PacketAttValue, PacketL2capData, PacketL2capHead, PacketLlApp, PacketLlData, PacketLlInit,
        RfPacketAdvIndModuleT, PKT_CMD_LEN,
    };
    use crate::service::{
        TELINK_SPP_DATA_CLIENT2SERVER, TELINK_SPP_DATA_OTA, TELINK_SPP_DATA_PAIR,
        TELINK_SPP_DATA_SERVER2CLIENT,
    };
    use crate::uart::{
        compute_crc, verify_crc, UartData, UartMsg, UART_CRC_OFFSET, UART_DATA_LEN,
        UART_PAYLOAD_MAX_SIZE, UART_PAYLOAD_OFFSET,
    };
    use core::mem::size_of;

    /// Golden vectors committed at the repository root and asserted from both
    /// this crate and the Python host tooling.
    const FIXTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../../tests/wire_vectors.txt"
    ));

    // --- fixture parsing --------------------------------------------------

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

    /// Reads a canonical-display-order UUID and returns the LSB-first form the
    /// crate constants use.
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

        // The fixture stores canonical display order; the crate constants are
        // LSB-first.
        bytes.reverse();
        bytes
    }

    /// Fixture accessor that records which keys were consumed, so a value
    /// mismatch can fail only the test that owns it.
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

    // --- wire struct sizes ------------------------------------------------

    /// `(fixture key, expected size)` for every packed struct this crate owns.
    ///
    /// `size.LumSaveT` and the OTA header keys are firmware-owned and stay in
    /// the firmware's own contract suite.
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
            ("size.UartData", size_of::<UartData>()),
        ]
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

    // --- CRC-16-MODBUS ----------------------------------------------------

    fn check_crc16(p: &mut Parser) {
        for version in 1..=5 {
            let input = p.bytes(&format!("crc.v{version}.input"));
            let expected = p.u32(&format!("crc.v{version}.expect"));
            let expected = match u16::try_from(expected) {
                Ok(expected) => expected,
                Err(_) => panic!(
                    "fixture key `crc.v{version}.expect` does not fit in a u16: {expected:#x}"
                ),
            };

            assert_eq!(
                crc16(&input),
                expected,
                "crc16 mismatch for `crc.v{version}.expect`"
            );
        }
    }

    // --- UART frame layout and opcodes ------------------------------------

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

    /// `counter | byte1 | cmd | payload(39) | crc16 LE`, sealed with the
    /// production `compute_crc`.
    fn data_frame(counter: u8, byte1: u8, cmd: u8, payload: &[u8]) -> UartData {
        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };
        msg.data[0] = counter;
        msg.data[1] = byte1;
        msg.data[2] = cmd;
        msg.data[UART_PAYLOAD_OFFSET..UART_PAYLOAD_OFFSET + payload.len()].copy_from_slice(payload);
        compute_crc(&mut msg);
        msg
    }

    /// Host ACK: `counter | 0xff | sno(3) | zeros | crc16 LE`; no command byte.
    fn ack_frame(counter: u8, byte1: u8, sno: &[u8]) -> UartData {
        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };
        msg.data[0] = counter;
        msg.data[1] = byte1;
        msg.data[2..2 + sno.len()].copy_from_slice(sno);
        compute_crc(&mut msg);
        msg
    }

    const FRAME_NAMESPACES: [&str; 3] = ["light_ctrl", "light_status", "node_status"];

    fn check_uart_frames(p: &mut Parser) {
        let layout = read_uart_layout(p);

        assert_eq!(
            layout.packet_length, UART_DATA_LEN,
            "`uart.layout.packet_length` must equal UART_DATA_LEN"
        );
        assert_eq!(
            layout.payload_offset, UART_PAYLOAD_OFFSET,
            "`uart.layout.payload_offset` must equal UART_PAYLOAD_OFFSET"
        );
        assert_eq!(
            layout.crc_offset, UART_CRC_OFFSET,
            "`uart.layout.crc_offset` must equal UART_CRC_OFFSET"
        );
        assert_eq!(
            layout.payload_len, UART_PAYLOAD_MAX_SIZE,
            "`uart.layout.payload_len` must equal UART_PAYLOAD_MAX_SIZE"
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
        // The command byte sits immediately before the payload.
        assert_eq!(
            layout.payload_offset - 1,
            2,
            "the command byte must stay the third frame byte"
        );

        for namespace in FRAME_NAMESPACES {
            let counter = p.u8(&format!("uart.frame.{namespace}.counter"));
            let byte1 = p.u8(&format!("uart.frame.{namespace}.byte1"));
            let cmd = p.u8(&format!("uart.frame.{namespace}.cmd"));
            let payload = p.bytes(&format!("uart.frame.{namespace}.payload"));
            let expected = p.bytes(&format!("uart.frame.{namespace}.bytes"));

            assert_eq!(
                payload.len(),
                layout.payload_len,
                "`uart.frame.{namespace}.payload` must be {} bytes",
                layout.payload_len
            );

            let frame = data_frame(counter, byte1, cmd, &payload);
            assert_eq!(
                frame.data.as_slice(),
                expected.as_slice(),
                "`uart.frame.{namespace}.bytes` mismatch"
            );
            assert!(
                verify_crc(&frame),
                "`uart.frame.{namespace}.bytes` must pass verify_crc"
            );
        }

        let ack_counter = p.u8("uart.frame.ack.counter");
        let ack_byte1 = p.u8("uart.frame.ack.byte1");
        let ack_sno = p.bytes("uart.frame.ack.sno");
        let ack_expected = p.bytes("uart.frame.ack.bytes");

        assert_eq!(
            ack_byte1,
            u8::from(UartMsg::Ack),
            "an ACK frame must set byte 1 to the ACK opcode"
        );

        let frame = ack_frame(ack_counter, ack_byte1, &ack_sno);
        assert_eq!(
            frame.data.as_slice(),
            ack_expected.as_slice(),
            "`uart.frame.ack.bytes` mismatch"
        );
        assert!(
            verify_crc(&frame),
            "`uart.frame.ack.bytes` must pass verify_crc"
        );
    }

    /// `(fixture key, expected opcode)` for the UART command bytes.
    ///
    /// The `uart.ota.*` sub-commands belong to the firmware's light-command
    /// layer (`LGT_CMD_*`) and are not asserted here.
    fn uart_opcode_expectations() -> Vec<(&'static str, u8)> {
        vec![
            ("uart.op.light_ctrl", u8::from(UartMsg::LightCtrl)),
            ("uart.op.light_status", u8::from(UartMsg::LightStatus)),
            ("uart.op.mesh_message", u8::from(UartMsg::MeshMessage)),
            ("uart.op.panic_message", u8::from(UartMsg::PanicMessage)),
            ("uart.op.print_message", u8::from(UartMsg::PrintMessage)),
            ("uart.op.node_status", u8::from(UartMsg::NodeStatus)),
            ("uart.op.ack", u8::from(UartMsg::Ack)),
        ]
    }

    fn check_uart_opcodes(p: &mut Parser) {
        for (key, expected) in uart_opcode_expectations() {
            assert_eq!(p.u8(key), expected, "`{key}` mismatch");
        }
    }

    // --- GATT UUIDs -------------------------------------------------------

    /// `(fixture key, expected crate UUID)` in LSB-first order.
    fn gatt_uuid_expectations() -> Vec<(&'static str, [u8; 16])> {
        vec![
            ("uuid.server2client", TELINK_SPP_DATA_SERVER2CLIENT),
            ("uuid.client2server", TELINK_SPP_DATA_CLIENT2SERVER),
            ("uuid.ota", TELINK_SPP_DATA_OTA),
            ("uuid.pair", TELINK_SPP_DATA_PAIR),
        ]
    }

    fn check_gatt_uuids(p: &mut Parser) {
        for (key, expected) in gatt_uuid_expectations() {
            assert_eq!(hex_uuid(p, key), expected, "`{key}` mismatch");
        }
    }

    // --- pairing opcodes and states ---------------------------------------

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

    fn check_pairing_constants(p: &mut Parser) {
        for (key, expected) in pairing_opcode_expectations() {
            assert_eq!(p.u8(key), expected, "`{key}` mismatch");
        }
        for (key, expected) in pairing_state_expectations() {
            assert_eq!(p.u8(key), expected, "`{key}` mismatch");
        }
    }

    // --- tests ------------------------------------------------------------

    /// Number of fixture keys this crate asserts, mirrored as
    /// `MESH_PROTOCOL_KEYS` in `rust/src/wire_contract_tests/mod.rs`.
    const CRATE_FIXTURE_KEY_COUNT: usize = 69;

    #[test]
    fn fixture_struct_sizes() {
        check_struct_sizes(&mut Parser::new());
    }

    /// `PKT_CMD_LEN` owns no fixture vector; `size.PacketLlData` pins it only
    /// transitively (its `cmd` field is `[u8; PKT_CMD_LEN]`), so assert the
    /// documented command-byte length directly.
    #[test]
    fn pkt_cmd_len_is_the_documented_length() {
        assert_eq!(PKT_CMD_LEN, 11);
    }

    /// The firmware's orphan check trusts its `MESH_PROTOCOL_KEYS` allow-list to
    /// name every key this crate consumes. This census makes crate-side drift
    /// fail here instead of silently skipping a key: dropping an assertion
    /// lowers the count, adding one raises it, and either way the allow-list
    /// must be updated in the same change.
    #[test]
    fn fixture_key_census_matches_the_firmware_mirror() {
        let mut p = Parser::new();
        check_struct_sizes(&mut p);
        check_crc16(&mut p);
        check_uart_opcodes(&mut p);
        check_uart_frames(&mut p);
        check_gatt_uuids(&mut p);
        check_pairing_constants(&mut p);

        assert_eq!(
            p.consumed.len(),
            CRATE_FIXTURE_KEY_COUNT,
            "the crate consumes a different number of fixture keys than the firmware's \
             MESH_PROTOCOL_KEYS allow-list names"
        );
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
}
