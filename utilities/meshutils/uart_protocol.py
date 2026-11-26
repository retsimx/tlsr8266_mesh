#!/usr/bin/env python3
"""
Shared frame and payload decoding for the TLSR8266 UART protocol.

Every UART frame is the same 44-byte layout defined in ``uart_constants``:

  byte[0]      counter
  byte[1]      command type (ACK = 0xFF)
  byte[2]      command code
  byte[3-41]   payload
  byte[42-43]  CRC16 over bytes 0-41, little-endian

The helpers here decapsulate a frame and decode the payload fields that more
than one host tool cares about.  Each tool keeps its own presentation: this
module never prints.

Used by ``uart_debug_monitor.py`` and ``uart_mesh_status.py``.
"""

import datetime
import struct
from collections import namedtuple

from mesh_common import crc16
from uart_constants import ACK, CRC_OFFSET, PACKET_LENGTH, PAYLOAD_OFFSET

NODE_STATUS_ENTRY_LEN = 3  # [node_id, online, on_off]

CrcCheck = namedtuple("CrcCheck", "ok expected actual")
MeshMessage = namedtuple("MeshMessage", "src dst op vendor params")


def timestamp() -> str:
    """Return the ``HH:MM:SS.mmm`` prefix used by the monitor tools."""
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def check_crc(frame) -> CrcCheck:
    """Compare a frame's stored CRC16 with the one computed from its bytes."""
    expected = crc16(frame, PACKET_LENGTH - 2)
    actual = frame[CRC_OFFSET] | (frame[CRC_OFFSET + 1] << 8)
    return CrcCheck(expected == actual, expected, actual)


def frame_payload(frame) -> bytes:
    """Return the payload region of a frame."""
    return bytes(frame[PAYLOAD_OFFSET:CRC_OFFSET])


def build_packet(cmd_code: int, payload: bytes = b"") -> bytearray:
    """Build a 44-byte frame with a correct CRC."""
    pkt = bytearray(PACKET_LENGTH)
    pkt[0] = 0
    pkt[1] = 0
    pkt[2] = cmd_code
    pkt[PAYLOAD_OFFSET:PAYLOAD_OFFSET + len(payload)] = payload
    crc = crc16(pkt, PACKET_LENGTH - 2)
    pkt[CRC_OFFSET] = crc & 0xFF
    pkt[CRC_OFFSET + 1] = (crc >> 8) & 0xFF
    return pkt


def build_ack(counter: int) -> bytearray:
    """Build the ACK frame for a received counter byte."""
    pkt = bytearray(PACKET_LENGTH)
    pkt[0] = counter
    pkt[1] = ACK
    crc = crc16(pkt, PACKET_LENGTH - 2)
    pkt[CRC_OFFSET] = crc & 0xFF
    pkt[CRC_OFFSET + 1] = (crc >> 8) & 0xFF
    return pkt


def decode_mesh_message(payload: bytes):
    """Decode a MESH_MESSAGE payload, or return ``None`` when it is too short.

    Payload layout (AppCmdValue): sno[0:3], src[3:5], dst[5:7], op[7],
    vendor[8:10], par[10:20]; multi-byte fields are little-endian.
    """
    if len(payload) < 10:
        return None
    return MeshMessage(
        src=struct.unpack_from("<H", payload, 3)[0],
        dst=struct.unpack_from("<H", payload, 5)[0],
        op=payload[7],
        vendor=struct.unpack_from("<H", payload, 8)[0],
        params=bytes(payload[10:20] if len(payload) >= 20 else payload[10:]),
    )


def decode_node_status(payload: bytes):
    """Yield ``(node_id, online, on_off)`` triples from a NODE_STATUS payload.

    Decodes every whole entry in the payload.  Node 0 is a valid address, so a
    zero node id is not a terminator; an all-zero entry is an unused slot and
    is skipped.
    """
    for i in range(
        0, len(payload) - NODE_STATUS_ENTRY_LEN + 1, NODE_STATUS_ENTRY_LEN
    ):
        node_id, online, on_off = payload[i:i + NODE_STATUS_ENTRY_LEN]
        if node_id == 0 and online == 0 and on_off == 0:
            continue
        yield (node_id, online, on_off)
