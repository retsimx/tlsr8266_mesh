#!/usr/bin/env python3
"""Wire namespace: CRC, UART framing and opcodes, GATT UUIDs, pairing constants.

Part of the Python half of the golden-vector suite; see
``wire_contract_fixture`` for the shared fixture loader and how to run it.
"""

import unittest

import mesh_common
import uart_constants

from wire_contract_fixture import FIXTURE


def build_uart_frame(counter, byte1, cmd=None, payload=b"", sno=None):
    """Rebuild a 44-byte host UART frame from its fields.

    Normal frames are ``counter | byte1 | cmd | payload(39)`` with a
    little-endian CRC-16-MODBUS at 42..44. Host ACKs instead write ``sno`` at
    2..5 and leave the rest of the payload zeroed.
    """
    frame = bytearray(uart_constants.PACKET_LENGTH)
    frame[0] = counter
    frame[1] = byte1
    if sno is not None:
        frame[2:5] = sno
    else:
        frame[2] = cmd
        offset = uart_constants.PAYLOAD_OFFSET
        frame[offset:offset + len(payload)] = payload
    crc = mesh_common.crc16(frame, uart_constants.PACKET_LENGTH - 2)
    frame[uart_constants.CRC_OFFSET] = crc & 0xFF
    frame[uart_constants.CRC_OFFSET + 1] = (crc >> 8) & 0xFF
    return bytes(frame)


class Crc16Tests(unittest.TestCase):
    """CRC-16-MODBUS (poly 0xA001, init 0xFFFF) over the shared vectors."""

    def test_vectors(self):
        for number in range(1, 6):
            with self.subTest(vector=number):
                data = FIXTURE.as_bytes(f"crc.v{number}.input")
                expected = FIXTURE.as_u32(f"crc.v{number}.expect")
                # crc16() defaults to len(packet) - 2, which is only right for
                # a frame that already carries its CRC. Always pass the length.
                self.assertEqual(mesh_common.crc16(data, len(data)), expected)


class UartConstantsTests(unittest.TestCase):
    """UART frame layout and command/OTA opcodes."""

    def test_layout(self):
        self.assertEqual(
            uart_constants.PACKET_LENGTH,
            FIXTURE.as_u32("uart.layout.packet_length"),
        )
        self.assertEqual(
            uart_constants.PAYLOAD_OFFSET,
            FIXTURE.as_u32("uart.layout.payload_offset"),
        )
        self.assertEqual(
            uart_constants.CRC_OFFSET,
            FIXTURE.as_u32("uart.layout.crc_offset"),
        )
        # payload_len is the CRC-covered payload span: 44 - 3 - 2 CRC bytes.
        self.assertEqual(
            FIXTURE.as_u32("uart.layout.payload_len"),
            uart_constants.PACKET_LENGTH - uart_constants.PAYLOAD_OFFSET - 2,
        )

    def test_command_codes(self):
        expected = (
            (uart_constants.LIGHT_CTRL, "uart.op.light_ctrl"),
            (uart_constants.LIGHT_STATUS, "uart.op.light_status"),
            (uart_constants.MESH_MESSAGE, "uart.op.mesh_message"),
            (uart_constants.PANIC_MESSAGE, "uart.op.panic_message"),
            (uart_constants.PRINT_MESSAGE, "uart.op.print_message"),
            (uart_constants.NODE_STATUS, "uart.op.node_status"),
            (uart_constants.ACK, "uart.op.ack"),
            # Host-only: the firmware never dispatches ENABLE_UART.
            (uart_constants.ENABLE_UART, "uart.op.enable_uart"),
        )
        for constant, key in expected:
            with self.subTest(key=key):
                self.assertEqual(constant, FIXTURE.as_u8(key))

    def test_ota_codes(self):
        expected = (
            (uart_constants.OTA_START, "uart.ota.start"),
            (uart_constants.OTA_START_RESP, "uart.ota.start_resp"),
            (uart_constants.OTA_DATA, "uart.ota.data"),
            (uart_constants.OTA_DATA_RESP, "uart.ota.data_resp"),
            (uart_constants.OTA_END, "uart.ota.end"),
        )
        for constant, key in expected:
            with self.subTest(key=key):
                self.assertEqual(constant, FIXTURE.as_u8(key))


class UartFrameTests(unittest.TestCase):
    """Full 44-byte UART frames rebuilt from their fields."""

    NORMAL_FRAMES = ("light_ctrl", "light_status", "node_status")

    def test_normal_frames(self):
        for name in self.NORMAL_FRAMES:
            prefix = f"uart.frame.{name}"
            with self.subTest(frame=name):
                frame = build_uart_frame(
                    FIXTURE.as_u8(f"{prefix}.counter"),
                    FIXTURE.as_u8(f"{prefix}.byte1"),
                    FIXTURE.as_u8(f"{prefix}.cmd"),
                    FIXTURE.as_bytes(f"{prefix}.payload"),
                )
                self.assertEqual(frame, FIXTURE.as_bytes(f"{prefix}.bytes"))

    def test_ack_frame(self):
        counter = FIXTURE.as_u8("uart.frame.ack.counter")
        byte1 = FIXTURE.as_u8("uart.frame.ack.byte1")
        sno = FIXTURE.as_bytes("uart.frame.ack.sno")
        # The receiver keys ACKs off frame[1] and echoes frame[0]; sno is unused.
        self.assertEqual(byte1, uart_constants.ACK)
        self.assertEqual(len(sno), 3)
        frame = build_uart_frame(counter, byte1, sno=sno)
        self.assertEqual(frame, FIXTURE.as_bytes("uart.frame.ack.bytes"))


class GattUuidTests(unittest.TestCase):
    """GATT characteristic UUIDs in canonical display order."""

    def test_characteristic_uuids(self):
        expected = (
            ("uuid.server2client", mesh_common.notify_characteristic_uuid),
            ("uuid.client2server", mesh_common.command_characteristic_uuid),
            ("uuid.ota", mesh_common.ota_characteristic_uuid),
            ("uuid.pair", mesh_common.pair_characteristic_uuid),
        )
        for key, module_uuid in expected:
            with self.subTest(key=key):
                canonical = FIXTURE.as_str(key)
                self.assertEqual(len(canonical), 32)
                # Same 16 bytes, one dashed (module) and one bare (fixture).
                self.assertEqual(len(module_uuid), 36)
                self.assertEqual(module_uuid.replace("-", ""), canonical)


class PairingTests(unittest.TestCase):
    """Pairing opcodes, state discriminants and public defaults."""

    def test_opcodes(self):
        expected = (
            ("pair.op.exchange_random", mesh_common.PAIR_OP_EXCHANGE_RANDOM),
            ("pair.op.set_mesh_name", mesh_common.PAIR_OP_SET_MESH_NAME),
            ("pair.op.set_mesh_password", mesh_common.PAIR_OP_SET_MESH_PASSWORD),
            ("pair.op.set_mesh_ltk", mesh_common.PAIR_OP_SET_MESH_LTK),
            ("pair.op.get_mesh_ltk", mesh_common.PAIR_OP_GET_MESH_LTK),
            ("pair.op.verify_credentials", mesh_common.PAIR_OP_VERIFY_CREDENTIALS),
        )
        for key, constant in expected:
            with self.subTest(key=key):
                self.assertEqual(constant, FIXTURE.as_u8(key))

    def test_states(self):
        expected = (
            ("pair.state.mesh_effect", mesh_common.PAIR_STATE_MESH_EFFECT),
            ("pair.state.requesting_ltk", mesh_common.PAIR_STATE_REQUESTING_LTK),
            ("pair.state.pairing_complete", mesh_common.PAIR_STATE_PAIRING_COMPLETE),
        )
        for key, constant in expected:
            with self.subTest(key=key):
                self.assertEqual(constant, FIXTURE.as_u8(key))

    def test_defaults(self):
        self.assertEqual(
            mesh_common.DEFAULT_MESH_NAME,
            FIXTURE.as_str("pair.default.mesh_name"),
        )
        # "123" is a string even though it parses as a decimal integer.
        self.assertEqual(
            mesh_common.DEFAULT_MESH_PASSWORD,
            FIXTURE.as_str("pair.default.password"),
        )
        ltk = FIXTURE.as_bytes("pair.default.ltk")
        self.assertEqual(len(ltk), mesh_common.KEY_SIZE)
        self.assertEqual(bytes(mesh_common.DEFAULT_LTK), ltk)


if __name__ == "__main__":
    unittest.main()
