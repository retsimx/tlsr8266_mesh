#!/usr/bin/env python3
"""Unit tests for the shared UART frame decoding in ``uart_protocol``.

The wire layout itself is asserted against the golden fixture by
``test_wire_contracts``; these tests pin the decoder behaviour both monitor
tools rely on.
"""

import unittest

import uart_constants as uc
import uart_protocol as proto


class FrameTests(unittest.TestCase):
    def test_build_packet_round_trip(self):
        frame = proto.build_packet(uc.ENABLE_UART)

        self.assertEqual(len(frame), uc.PACKET_LENGTH)
        self.assertEqual(frame[2], uc.ENABLE_UART)
        self.assertTrue(proto.check_crc(frame).ok)
        self.assertEqual(
            proto.frame_payload(frame),
            bytes(uc.CRC_OFFSET - uc.PAYLOAD_OFFSET),
        )

    def test_build_ack_round_trip(self):
        frame = proto.build_ack(0x5A)
        check = proto.check_crc(frame)

        self.assertEqual(frame[0], 0x5A)
        self.assertEqual(frame[1], uc.ACK)
        self.assertTrue(check.ok)
        self.assertEqual(check.expected, check.actual)

    def test_corrupt_frame_fails_crc(self):
        frame = proto.build_packet(uc.PRINT_MESSAGE)
        frame[5] ^= 0xFF

        self.assertFalse(proto.check_crc(frame).ok)


class NodeStatusTests(unittest.TestCase):
    def test_decodes_contiguous_entries(self):
        payload = bytes([0x1F, 1, 1, 0x05, 0, 1])
        payload += bytes(uc.CRC_OFFSET - uc.PAYLOAD_OFFSET - len(payload))

        self.assertEqual(
            list(proto.decode_node_status(payload)),
            [(0x1F, 1, 1), (0x05, 0, 1)],
        )

    def test_node_zero_is_a_valid_address(self):
        payload = bytes([0x00, 1, 1, 0x05, 0, 1])

        self.assertEqual(
            list(proto.decode_node_status(payload)),
            [(0x00, 1, 1), (0x05, 0, 1)],
        )

    def test_unused_entry_does_not_truncate(self):
        payload = bytes([0x00, 0, 0, 0x05, 1, 1])

        self.assertEqual(
            list(proto.decode_node_status(payload)), [(0x05, 1, 1)]
        )

    def test_partial_trailing_entry_ignored(self):
        payload = bytes([0x1F, 1, 1, 0x05, 1])

        self.assertEqual(
            list(proto.decode_node_status(payload)), [(0x1F, 1, 1)]
        )


class MeshMessageTests(unittest.TestCase):
    def test_decodes_fields(self):
        payload = bytearray(uc.CRC_OFFSET - uc.PAYLOAD_OFFSET)
        payload[0:3] = b"\x01\x02\x03"
        payload[3:5] = (0x1234).to_bytes(2, "little")
        payload[5:7] = (0x00FF).to_bytes(2, "little")
        payload[7] = 0xD0
        payload[8:10] = (0x0211).to_bytes(2, "little")
        payload[10:12] = b"\xAA\xBB"

        msg = proto.decode_mesh_message(bytes(payload))

        self.assertEqual(
            (msg.src, msg.dst, msg.op, msg.vendor),
            (0x1234, 0x00FF, 0xD0, 0x0211),
        )
        self.assertEqual(len(msg.params), 10)
        self.assertEqual(msg.params[:2], b"\xAA\xBB")

    def test_short_payload_returns_none(self):
        self.assertIsNone(proto.decode_mesh_message(b"\x00" * 9))


class TimestampTests(unittest.TestCase):
    def test_millisecond_format(self):
        self.assertRegex(proto.timestamp(), r"^\d{2}:\d{2}:\d{2}\.\d{3}$")


if __name__ == "__main__":
    unittest.main()
