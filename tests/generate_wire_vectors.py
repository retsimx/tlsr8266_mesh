#!/usr/bin/env python3
"""Regenerate ``tests/wire_vectors.txt``, the committed cross-boundary contract.

Run from the repository root, with the host-tooling dependencies installed:

    pip install -r utilities/meshutils/requirements.txt
    PYTHONPATH=utilities/meshutils python3 tests/generate_wire_vectors.py

It writes ``tests/wire_vectors.txt`` next to itself (never relative to the
current directory) and prints a short key census. It is deterministic: two runs
produce byte-identical output, which is what makes the fixture's sha256
reviewable.

That output IS the contract. Both suites -- ``rust/crates/mesh-protocol/src/lib.rs``
(``fixture_tests``, for the protocol half) and ``rust/src/wire_contract_tests/``
(the firmware-owned remainder), plus the host tooling in
``utilities/meshutils/tests/`` -- assert it byte for byte, so:

  * never hand-edit ``tests/wire_vectors.txt``;
  * change this script, re-run it, then run both suites and review the diff.

Every value that has a production implementation is produced by that
implementation rather than re-typed here:

  * mesh_common.crc16                  -> CRC vectors + UART frame CRCs
  * mesh_common.aes_att_encryption     -> AES-128 block parity table
  * mesh_common.CommandAction          -> 20-byte encrypted command vectors
  * flash_fw.BLEPacketParser           -> 20-byte BLE OTA packets
  * flash_fw_uart.UARTPacketParser     -> 10-byte UART OTA packets

Two derivations are spelled out longhand so they can act as an oracle, and are
then cross-checked against the production entry point:

  * the AES packet vectors transliterate ``crypto.rs::
    aes_att_encryption_packet`` and must round-trip through
    ``mesh_common.decrypt_notification``;
  * the session-key vector composes ``mesh_common.encrypt_data`` per the pairing
    protocol and must be accepted by ``mesh_common.process_session_key``.
    ``process_session_key`` reads the module global ``mesh_common.shared_key``,
    so this script sets it explicitly and restores it -- it is never left at the
    random value Python assigns at import time.
"""

from pathlib import Path

import mesh_common
import uart_constants
from flash_fw import BLEPacketParser
from flash_fw_uart import UARTPacketParser

# ---------------------------------------------------------------------------
# value formatting helpers (fixture format is normative)
# ---------------------------------------------------------------------------

def hexval(value):
    return "0x%02x" % value if value <= 0xFF else "0x%x" % value


def bytesval(data):
    return " ".join("%02x" % b for b in data)


def crc16(data):
    """CRC-16-MODBUS via the production Python implementation."""
    return mesh_common.crc16(list(data), len(data))


# ---------------------------------------------------------------------------
# 1. UART frames  (44 B: counter | byte1 | cmd | payload[39] | crc LE)
# ---------------------------------------------------------------------------

UART_PACKET_LENGTH = uart_constants.PACKET_LENGTH
UART_PAYLOAD_OFFSET = uart_constants.PAYLOAD_OFFSET
UART_CRC_OFFSET = uart_constants.CRC_OFFSET
UART_PAYLOAD_LEN = UART_CRC_OFFSET - UART_PAYLOAD_OFFSET  # 39


def uart_frame(counter, byte1, cmd, payload):
    """Build a 44-byte UART frame exactly the way flash_fw_uart.py does."""
    assert len(payload) == UART_PAYLOAD_LEN, len(payload)
    packet = [0] * UART_PACKET_LENGTH
    packet[0] = counter & 0xFF
    packet[1] = byte1 & 0xFF
    packet[2] = cmd & 0xFF
    packet[UART_PAYLOAD_OFFSET:UART_PAYLOAD_OFFSET + UART_PAYLOAD_LEN] = payload
    crc = mesh_common.crc16(packet, UART_PACKET_LENGTH - 2)
    packet[UART_CRC_OFFSET] = crc & 0xFF
    packet[UART_CRC_OFFSET + 1] = (crc >> 8) & 0xFF
    return bytes(packet)


# --- light_ctrl: host -> device OTA_DATA to mesh node 5 -------------------
# payload: dest LE(2) | OTA_DATA(0x26) | flags(0) | 0 | index LE(2) | chunk(8)
#          | retransmit(1, frame[18]) | ack-flag(1, frame[19]) | unused zeros
LIGHT_CTRL_COUNTER = 0x2A
light_ctrl_payload = (
    [0x05, 0x00]                      # target mesh address 5 (LE)
    + [uart_constants.OTA_DATA, 0x00, 0x00]   # sub-command 0x26 + two pad bytes
    + [0x04, 0x00]                    # OTA packet index 4 (LE)
    + [0x9A, 0x3C, 0x5E, 0x71, 0x0B, 0xD4, 0xF6, 0x28]  # 8-byte chunk
    + [0x00, 0x00]                    # frame[18] retransmit, frame[19] ack-flag
    + [0x00] * 22                     # unused payload tail (zero-filled)
)
assert len(light_ctrl_payload) == 39, len(light_ctrl_payload)
light_ctrl_frame = uart_frame(
    LIGHT_CTRL_COUNTER, 0x00, uart_constants.LIGHT_CTRL, light_ctrl_payload
)

# --- light_status: device -> host OTA_DATA_RESP for node 5 ----------------
# Response prefix offsets are dictated by flash_fw_uart.get_response():
#   frame[6]   -> src low byte      (payload[3])
#   frame[10]  -> op & 0x3F         (payload[7])
#   frame[11:] -> acked packet index (payload[8:])
LIGHT_STATUS_COUNTER = 0x5C
light_status_payload = (
    [0x01, 0x02, 0x03]                # sno
    + [0x05, 0x00]                    # src mesh address 5 (LE)
    + [0x00, 0x00]                    # dst mesh address 0 (LE)
    + [0xC0 | uart_constants.OTA_DATA_RESP]  # op 0xE7 (0x27 | 0xC0)
    + [0x11, 0x02]                    # vendor id 0x0211 (LE)
    + [0x04, 0x00]                    # acked OTA packet index 4 (LE)
    + [0x00] * 27                     # unused payload tail (zero-filled)
)
assert len(light_status_payload) == 39, len(light_status_payload)
light_status_frame = uart_frame(
    LIGHT_STATUS_COUNTER, 0x00, uart_constants.LIGHT_STATUS, light_status_payload
)

# --- node_status: device -> host, 13 x [node_id, online, on_off] ----------
NODE_STATUS_COUNTER = 0x7E
node_entries = [
    (0x01, 1, 1), (0x02, 1, 0), (0x05, 0, 0), (0x07, 1, 1), (0x0B, 0, 0),
    (0x0C, 1, 0), (0x11, 0, 0), (0x13, 1, 1), (0x15, 1, 0), (0x17, 1, 1),
    (0x19, 0, 0), (0x1B, 1, 1), (0x1F, 1, 0),
]
assert len(node_entries) == 13
node_status_payload = [b for entry in node_entries for b in entry]
assert len(node_status_payload) == 39, len(node_status_payload)
node_status_frame = uart_frame(
    NODE_STATUS_COUNTER, 0x00, uart_constants.NODE_STATUS, node_status_payload
)

# --- ack: host -> device reply, byte1 = 0xFF, sno zeroed ------------------
# Mirrors flash_fw_uart.recv_thread(): only frame[0] and frame[1] are set.
ACK_COUNTER = 0xB3
ack_sno = [0x00, 0x00, 0x00]
ack_frame = uart_frame(ACK_COUNTER, uart_constants.ACK, 0x00, [0x00] * 39)
assert ack_frame[0] == ACK_COUNTER
assert ack_frame[2:5] == bytes(ack_sno)

# ---------------------------------------------------------------------------
# 2. CRC vectors
# ---------------------------------------------------------------------------

crc_v1_input = bytes([0x00])
crc_v2_input = b"123456789"
crc_v3_input = light_ctrl_frame[0:42]
crc_v4_input = light_status_frame[0:42]
crc_v5_input = node_status_frame[0:42]

# ---------------------------------------------------------------------------
# 3. Session-key derivation (mesh_common.process_session_key)
# ---------------------------------------------------------------------------
# Synthetic values, not captured pairing traffic. shared_key is the client
# random (pair_randm); rands is the server random (pair_rands) carried in the
# first 8 bytes of the response payload.
SESSION_SHARED_KEY = bytes.fromhex("0102030405060708")
SESSION_CREDENTIALS = bytes.fromhex("00112233445566778899aabbccddeeff")
SESSION_RANDS = bytes.fromhex("a0a1a2a3a4a5a6a7")
SESSION_OPCODE = mesh_common.PAIR_OP_VERIFY_CREDENTIALS

assert len(SESSION_SHARED_KEY) == mesh_common.RANDOM_CHALLENGE_SIZE
assert len(SESSION_CREDENTIALS) == mesh_common.KEY_SIZE
assert len(SESSION_RANDS) == mesh_common.RANDOM_CHALLENGE_SIZE

# The server's expected challenge: rands(8) || reverse(encrypted[8:16]).
session_key16 = SESSION_RANDS + bytes(mesh_common.KEY_SIZE - len(SESSION_RANDS))
session_encrypted = mesh_common.encrypt_data(session_key16, SESSION_CREDENTIALS)
session_result = bytes(SESSION_RANDS) + bytes(reversed(session_encrypted[8:16]))
session_response = bytes([SESSION_OPCODE]) + session_result
assert len(session_response) == mesh_common.KEY_SIZE + 1, len(session_response)

# The derived key: reverse(encrypt_data(credentials, shared_key || rands)).
session_key = bytes(
    reversed(
        mesh_common.encrypt_data(
            SESSION_CREDENTIALS, SESSION_SHARED_KEY + SESSION_RANDS
        )
    )
)
assert len(session_key) == mesh_common.KEY_SIZE, len(session_key)

# One byte of the server result flipped: the verification must reject it.
session_response_invalid = bytearray(session_response)
session_response_invalid[5] ^= 0x01
assert bytes(session_response_invalid) != session_response

# Cross-check both against the production entry point. It reads the module
# global `shared_key`, so set it explicitly and restore it afterwards.
_saved_shared_key = mesh_common.shared_key
mesh_common.shared_key = SESSION_SHARED_KEY
try:
    assert (
        bytes(
            mesh_common.process_session_key(session_response, SESSION_CREDENTIALS)
        )
        == session_key
    ), "process_session_key disagrees with the derived session key"
    try:
        mesh_common.process_session_key(
            bytes(session_response_invalid), SESSION_CREDENTIALS
        )
    except Exception:
        pass
    else:
        raise AssertionError("process_session_key accepted a corrupted response")
finally:
    mesh_common.shared_key = _saved_shared_key

# ---------------------------------------------------------------------------
# 4. AES: Python mirror of Rust aes_att_encryption_packet
# ---------------------------------------------------------------------------
# crypto.rs: aes_att_encryption_packet()
#   Phase 1 (auth, plaintext):  auth_block = ivm[0:8] | len | 0*7
#                               auth = E(key, auth_block)
#                               for i: auth[i & 15] ^= plain[i]; after a full
#                                      block or the last byte: auth = E(key, auth)
#   Phase 2: mic = auth[0:mic_len]
#   Phase 3 (CTR): counter_block = 0 | ivm[0:8] | 0*7
#                  ks = E(key, counter_block)
#                  for i: at each block start (after the first) bump
#                         counter_block[0] and re-encrypt; then
#                         cipher[i] = plain[i] ^ ks[i & 15]
# E() is Rust aes_att_encryption, mirrored by mesh_common.aes_att_encryption.

def rust_encrypt_packet(key, ivm, plain, mic_len, block):
    """Literal transliteration of crypto.rs aes_att_encryption_packet()."""
    assert len(key) == 16 and len(ivm) == 8
    if len(plain) == 0:
        auth_block = bytes(ivm) + bytes([0]) + bytes(7)
        auth = block(key, auth_block)
        return bytes(auth[0:mic_len]), b""

    auth_block = bytes(ivm) + bytes([len(plain)]) + bytes(7)
    auth = block(key, auth_block)
    for i in range(len(plain)):
        auth = bytearray(auth)
        auth[i & 0xF] ^= plain[i]
        if (i & 0xF) == 0xF or i == len(plain) - 1:
            auth = block(key, bytes(auth))
    mic = bytes(auth[0:mic_len])

    counter_block = bytearray(16)
    counter_block[1:9] = ivm
    ks = block(key, bytes(counter_block))
    out = bytearray(plain)
    for i in range(len(plain)):
        if (i & 0xF) == 0 and i > 0:
            counter_block[0] = (counter_block[0] + 1) & 0xFF
            ks = block(key, bytes(counter_block))
        out[i] ^= ks[i & 0xF]
    return mic, bytes(out)


table = {}       # (key, plain) -> cipher, insertion ordered
call_log = []    # ordered list of (key, plain), duplicates preserved


def recording_block(key, plain):
    key = bytes(key)
    plain = bytes(plain)
    cipher = bytes(mesh_common.aes_att_encryption(key, plain))
    if (key, plain) in table:
        assert table[(key, plain)] == cipher, "AES is not a function"
    else:
        table[(key, plain)] = cipher
    call_log.append((key, plain))
    return cipher


AES_MIC_LEN = 2

# packet 1: 16-byte plaintext (single full block)
pkt1_key = bytes.fromhex("2b7e151628aed2a6abf7158809cf4f3c")
pkt1_ivm = bytes.fromhex("d4e5f6" "a1b2c3" "2500")   # mac[0:3] | sno | src
pkt1_plain = bytes(range(0x10, 0x20))                # 16 B
assert len(pkt1_ivm) == 8
calls_before = len(call_log)
mic1, cipher1 = rust_encrypt_packet(
    pkt1_key, pkt1_ivm, pkt1_plain, AES_MIC_LEN, recording_block
)
pkt1_calls = len(call_log) - calls_before

# packet 2: 24-byte plaintext (one full block + one partial block)
pkt2_key = bytes.fromhex("8e73b0f7da0e6452c810f32b809079e5")
pkt2_ivm = bytes.fromhex("0a1b2c" "4d5e6f" "3700")
pkt2_plain = bytes(range(0x20, 0x38))                # 24 B
calls_before = len(call_log)
mic2, cipher2 = rust_encrypt_packet(
    pkt2_key, pkt2_ivm, pkt2_plain, AES_MIC_LEN, recording_block
)
pkt2_calls = len(call_log) - calls_before

packets = [
    (1, pkt1_key, pkt1_ivm, pkt1_plain, cipher1, mic1),
    (2, pkt2_key, pkt2_ivm, pkt2_plain, cipher2, mic2),
]

# Strict replay: every AES call must be answerable from the emitted table alone.
def table_block(key, plain):
    key = bytes(key)
    plain = bytes(plain)
    if (key, plain) not in table:
        raise AssertionError(
            "packet algorithm used a block that is not in the table: key=%s plain=%s"
            % (key.hex(), plain.hex())
        )
    return table[(key, plain)]


for idx, key, ivm, plain, cipher, mic in packets:
    mic_rt, cipher_rt = rust_encrypt_packet(key, ivm, plain, AES_MIC_LEN, table_block)
    assert (mic_rt, cipher_rt) == (mic, cipher), "replay mismatch for packet %d" % idx

# Round-trip through the real Python decrypt path.
def mac_fwd_from_ivm(ivm):
    return b"\x00\x00\x00" + bytes(reversed(ivm[0:3]))


for idx, key, ivm, plain, cipher, mic in packets:
    sno = ivm[3:6]
    src = ivm[6:8]
    data = bytes(sno) + bytes(src) + bytes(mic) + bytes(cipher)
    assert (
        mesh_common.decrypt_notification(data, key, mac_fwd_from_ivm(ivm)) == plain
    ), "decrypt_notification round-trip failed for packet %d" % idx
    corrupted = bytearray(data)
    corrupted[5] ^= 0x01
    assert (
        mesh_common.decrypt_notification(
            bytes(corrupted), key, mac_fwd_from_ivm(ivm)
        )
        is None
    ), "corrupted MIC was accepted for packet %d" % idx

block_keys = list(table.keys())  # insertion order == first use

# ---------------------------------------------------------------------------
# 4b. CommandAction command encryption (mesh_common.CommandAction)
# ---------------------------------------------------------------------------
# Synthetic vectors, not captured traffic. A 20-byte command is
#   plain_cmd = sno(3) | mac_slot(2) | payload(15)
# and CommandAction.generate_data() mutates it in place: it writes the 2-byte
# MAC into command[3:5] and CTR-encrypts command[5:20]. The emitted
# encrypted_cmd is that production output; plain_cmd is the pre-call copy, so
# the fixture carries both sides of the mapping. Vector 1 is all-zero
# plaintext; vector 2 carries a non-zero sno and payload so the MAC and CTR
# paths are not exercised only on zeros.
CMD_ACTION_SESSION_KEY = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
CMD_ACTION_NONCE = bytes.fromhex("a1b2c3d4e5f60718")
assert len(CMD_ACTION_SESSION_KEY) == mesh_common.KEY_SIZE
assert len(CMD_ACTION_NONCE) == 8


def cmd_action_vector(sno, mac_slot, payload):
    """Encrypt one 20-byte command through the production entry point."""
    assert len(sno) == 3 and len(mac_slot) == 2 and len(payload) == 15
    plain_cmd = bytes(sno) + bytes(mac_slot) + bytes(payload)
    encrypted = mesh_common.CommandAction(
        mesh_common.BaseCommandAction(session_key=list(CMD_ACTION_SESSION_KEY))
    ).generate_data(
        list(CMD_ACTION_SESSION_KEY), list(CMD_ACTION_NONCE), bytearray(plain_cmd)
    )
    assert len(encrypted) == 20, len(encrypted)
    return plain_cmd, bytes(encrypted)


cmd_action_vectors = [
    (1, *cmd_action_vector(bytes(3), bytes(2), bytes(15))),
    (2, *cmd_action_vector(
        bytes([0x01, 0x02, 0x03]),
        bytes([0x04, 0x05]),
        bytes(range(0x10, 0x1F)),
    )),
]

# ---------------------------------------------------------------------------
# 5. OTA header + synthetic blob + packet parsers
# ---------------------------------------------------------------------------

BUILD_VERSION = 3543  # rust/src/version.rs at authoring time; bit 31 clear
BIN_SIZE = 64

RESET_OPCODE = bytes.fromhex("dead")     # SYNTHETIC placeholder (TC32 `tj` unknown)
IRQ_OPCODE = bytes.fromhex("beef")       # SYNTHETIC placeholder (TC32 `tj` unknown)
RAMCODE_WORD = bytes.fromhex("20018800") # .word 0x00880120 at 0x0C
MAGIC_BYTES = bytes.fromhex("4b4e4c54")  # .word 0x544c4e4b, little-endian => "KNLT"

ota_header = (
    RESET_OPCODE
    + (BUILD_VERSION & 0x7FFFFFFF).to_bytes(4, "little")
    + b"\x00\x00"
    + MAGIC_BYTES
    + RAMCODE_WORD
    + IRQ_OPCODE
    + b"\x00" * 6
    + BIN_SIZE.to_bytes(4, "little")
    + b"\x00" * 4
)
assert len(ota_header) == 32, len(ota_header)
assert ota_header[2:6] == (BUILD_VERSION & 0x7FFFFFFF).to_bytes(4, "little")
assert ota_header[8:12] == b"KNLT"

# digits of pi, purely deterministic filler so a byte flip anywhere is visible
PI_BYTES = bytes([
    0x31, 0x41, 0x59, 0x26, 0x53, 0x58, 0x97, 0x93,
    0x23, 0x84, 0x62, 0x64, 0x33, 0x83, 0x27, 0x95,
    0x02, 0x88, 0x41, 0x97, 0x16, 0x93, 0x99, 0x37,
    0x51, 0x05, 0x82, 0x09, 0x74, 0x94, 0x45, 0x92,
])
assert len(PI_BYTES) == 32
ota_blob = ota_header + PI_BYTES
assert len(ota_blob) == 64

ble_parser = BLEPacketParser(ota_blob)
ble_packets = []
while ble_parser.has_next_packet():
    ble_packets.append(bytes(ble_parser.get_next_packet()))
assert len(ble_packets) == 4, len(ble_packets)

uart_parser = UARTPacketParser(ota_blob)
uart_packets = []
while uart_parser.has_next_packet():
    uart_packets.append(bytes(uart_parser.get_next_packet()))
assert len(uart_packets) == 8, len(uart_packets)

for i, p in enumerate(ble_packets):
    assert len(p) == 20, len(p)
    assert p[0] | (p[1] << 8) == i
    assert p[18] | (p[19] << 8) == crc16(p[0:18])
    assert p[2:18] == ota_blob[i * 16:(i + 1) * 16]
for i, p in enumerate(uart_packets):
    assert len(p) == 10, len(p)
    assert p[0] | (p[1] << 8) == i
    assert p[2:10] == ota_blob[i * 8:(i + 1) * 8]

# ---------------------------------------------------------------------------
# 6. Emit the fixture
# ---------------------------------------------------------------------------

SIZE_VECTORS = [
    ("RfPacketAdvIndModuleT", 43),
    ("PacketAttValue", 30),
    ("PacketL2capHead", 10),
    ("PacketL2capData", 37),
    ("PacketLlData", 33),
    ("AppCmdValue", 20),
    ("PacketLlApp", 43),
    ("PacketAttReadRsp", 33),
    ("PacketLlInit", 40),
    ("PacketAttData", 36),
    ("PacketAttCmd", 43),
    ("MeshPkt", 48),
    ("Packet", 48),
    ("LumSaveT", 7),
    ("UartData", 48),
]

out = []
w = out.append

w("# wire_vectors.txt -- cross-boundary golden vectors for the TLSR8266 mesh")
w("# firmware (Rust, rust/src/wire_contract_tests/ and")
w("# rust/crates/mesh-protocol/src/lib.rs) and the host tooling")
w("# (Python, utilities/meshutils/tests/test_wire_contract_*.py).")
w("#")
w("# Format (normative):")
w("#   * one `key = value` per line; blank lines and lines starting with `#`")
w("#     are ignored; keys are dot-namespaced and MUST be unique;")
w("#   * a value is a decimal integer, a 0x-prefixed hex integer, a")
w("#     space-separated list of lowercase 2-hex-digit bytes, or a bare string;")
w("#   * the accessor chosen at the parse site decides the kind, so a value that")
w("#     looks numeric (pair.default.password = 123) is read as a string there.")
w("#     Single-token values are therefore ambiguous by design: '00' is one")
w("#     byte for crc.v1.input, and size.AppCmdValue = 20 is decimal twenty.")
w("#     Each suite must classify via the accessor it calls, never by shape.")
w("#   * uuid.* values are 32-character bare strings (canonical display order),")
w("#     not byte arrays, even though every character is a hex digit.")
w("#")
w("# Generated by tests/generate_wire_vectors.py, which imports the real Python")
w("# tooling (mesh_common.crc16, mesh_common.aes_att_encryption,")
w("# mesh_common.decrypt_notification, mesh_common.process_session_key,")
w("# BLEPacketParser, UARTPacketParser). Never hand-edit this file:")
w("# change the generator, re-run it, then re-run both suites.")
w("#")
w("# SYNTHETIC values are called out with an explicit `# SYNTHETIC` comment and")
w("# mirrored as their own keys so neither suite has to pretend they are facts.")
w("")

w("# --- struct sizes asserted in Rust (mem::size_of) -------------------------")
w("# Python has no equivalent structs, so these are Rust-only vectors.")
for name, size in SIZE_VECTORS:
    w("size.%s = %d" % (name, size))
w("")

w("# --- CRC-16-MODBUS (poly 0xA001, init 0xFFFF), asserted by both suites ----")
w("# v1: single zero byte. v2: the canonical ASCII check string.")
w("# v3/v4/v5: the first 42 bytes of uart.frame.light_ctrl / .light_status /")
w("# .node_status below (i.e. the CRC-covered span, excluding the CRC itself).")
w("crc.v1.input = %s" % bytesval(crc_v1_input))
w("crc.v1.expect = %s" % hexval(crc16(crc_v1_input)))
w("crc.v2.input = %s" % bytesval(crc_v2_input))
w("crc.v2.expect = %s" % hexval(crc16(crc_v2_input)))
w("crc.v3.input = %s" % bytesval(crc_v3_input))
w("crc.v3.expect = %s" % hexval(crc16(crc_v3_input)))
w("crc.v4.input = %s" % bytesval(crc_v4_input))
w("crc.v4.expect = %s" % hexval(crc16(crc_v4_input)))
w("crc.v5.input = %s" % bytesval(crc_v5_input))
w("crc.v5.expect = %s" % hexval(crc16(crc_v5_input)))
w("")

w("# --- UART frame layout: 44 B, payload 39 B at offset 3, CRC LE at 42 -----")
w("uart.layout.packet_length = %d" % UART_PACKET_LENGTH)
w("uart.layout.payload_offset = %d" % UART_PAYLOAD_OFFSET)
w("uart.layout.crc_offset = %d" % UART_CRC_OFFSET)
w("uart.layout.payload_len = %d" % UART_PAYLOAD_LEN)
w("")

w("# --- UART command codes (uart_constants.py / UartMsg in uart_manager.rs) --")
w("uart.op.light_ctrl = %s" % hexval(uart_constants.LIGHT_CTRL))
w("uart.op.light_status = %s" % hexval(uart_constants.LIGHT_STATUS))
w("uart.op.mesh_message = %s" % hexval(uart_constants.MESH_MESSAGE))
w("uart.op.panic_message = %s" % hexval(uart_constants.PANIC_MESSAGE))
w("uart.op.print_message = %s" % hexval(uart_constants.PRINT_MESSAGE))
w("uart.op.node_status = %s" % hexval(uart_constants.NODE_STATUS))
w("uart.op.ack = %s" % hexval(uart_constants.ACK))
w("# Python-only: the host sends ENABLE_UART, the firmware never dispatches it")
w("# (its UartMsg variant is commented out), so only the Python suite asserts it.")
w("uart.op.enable_uart = %s" % hexval(uart_constants.ENABLE_UART))
w("")

w("# --- UART OTA sub-command codes (uart_constants.py / LGT_CMD_* in light.rs)")
w("uart.ota.start = %s" % hexval(uart_constants.OTA_START))
w("uart.ota.start_resp = %s" % hexval(uart_constants.OTA_START_RESP))
w("uart.ota.data = %s" % hexval(uart_constants.OTA_DATA))
w("uart.ota.data_resp = %s" % hexval(uart_constants.OTA_DATA_RESP))
w("uart.ota.end = %s" % hexval(uart_constants.OTA_END))
w("")


def emit_frame(name, counter, byte1, cmd, payload, frame):
    w("# %s: bytes = counter | byte1 | cmd | payload(39) | crc16 LE at 42..44" % name)
    w("uart.frame.%s.counter = %s" % (name, hexval(counter)))
    w("uart.frame.%s.byte1 = %s" % (name, hexval(byte1)))
    w("uart.frame.%s.cmd = %s" % (name, hexval(cmd)))
    w("uart.frame.%s.payload = %s" % (name, bytesval(payload)))
    w("uart.frame.%s.bytes = %s" % (name, bytesval(frame)))
    w("")


w("# --- Full 44-byte UART frames, rebuildable from the keys above -----------")
w("# Unused payload bytes are zero because both builders zero-initialise the")
w("# 44-byte buffer (Python send_cmd, Rust UartData{data:[0; 44]}).")
emit_frame("light_ctrl", LIGHT_CTRL_COUNTER, 0x00, uart_constants.LIGHT_CTRL,
           light_ctrl_payload, light_ctrl_frame)
w("# light_status carries an OTA_DATA_RESP: flash_fw_uart.get_response() reads")
w("# frame[6] (src low), frame[10] (op & 0x3F) and frame[11:] (acked index).")
emit_frame("light_status", LIGHT_STATUS_COUNTER, 0x00, uart_constants.LIGHT_STATUS,
           light_status_payload, light_status_frame)
w("# node_status: 13 x [node_id, online, on_off] triples = exactly 39 bytes.")
emit_frame("node_status", NODE_STATUS_COUNTER, 0x00, uart_constants.NODE_STATUS,
           node_status_payload, node_status_frame)

w("# ack: host reply. Only frame[0] and frame[1] are written (flash_fw_uart")
w("# recv_thread); sno (frame[2:5]) stays zeroed, rest of the payload is zero.")
w("uart.frame.ack.counter = %s" % hexval(ACK_COUNTER))
w("uart.frame.ack.byte1 = %s" % hexval(uart_constants.ACK))
w("uart.frame.ack.sno = %s" % bytesval(ack_sno))
w("uart.frame.ack.bytes = %s" % bytesval(ack_frame))
w("")

w("# --- GATT characteristic UUIDs, canonical display order -----------------")
w("# Rust stores the byte-reversed form in TELINK_SPP_DATA_*; reverse(fixture)")
w("# must equal it. Python's *_characteristic_uuid strings are dashed forms.")
w("uuid.server2client = 000102030405060708090a0b0c0d1911")
w("uuid.client2server = 000102030405060708090a0b0c0d1912")
w("uuid.ota = 000102030405060708090a0b0c0d1913")
w("uuid.pair = 000102030405060708090a0b0c0d1914")
w("")

w("# --- Pairing opcodes (PAIR_OP_* in ble_ll_pair.rs / mesh_common.py) -------")
w("pair.op.exchange_random = %s" % hexval(mesh_common.PAIR_OP_EXCHANGE_RANDOM))
w("pair.op.set_mesh_name = %s" % hexval(mesh_common.PAIR_OP_SET_MESH_NAME))
w("pair.op.set_mesh_password = %s" % hexval(mesh_common.PAIR_OP_SET_MESH_PASSWORD))
w("pair.op.set_mesh_ltk = %s" % hexval(mesh_common.PAIR_OP_SET_MESH_LTK))
w("pair.op.get_mesh_ltk = %s" % hexval(mesh_common.PAIR_OP_GET_MESH_LTK))
w("pair.op.verify_credentials = %s" % hexval(mesh_common.PAIR_OP_VERIFY_CREDENTIALS))
w("# Rust-only: neither opcode exists in mesh_common.py, so only the Rust suite")
w("# asserts them (declared allowlist for the Python orphan-key check).")
w("pair.op.reset_mesh = 0x0a")
w("pair.op.delete_pairing = 0x0e")
w("")

w("# --- Pairing state discriminants (PairState in mesh-protocol/src/mesh_types.rs) ---")
w("pair.state.mesh_effect = %s" % hexval(mesh_common.PAIR_STATE_MESH_EFFECT))
w("pair.state.requesting_ltk = %s" % hexval(mesh_common.PAIR_STATE_REQUESTING_LTK))
w("pair.state.pairing_complete = %s" % hexval(mesh_common.PAIR_STATE_PAIRING_COMPLETE))
w("")

w("# --- Pairing defaults (config.rs MESH_NAME/MESH_PWD/MESH_LTK) ------------")
w("# Public, documented defaults, not real credentials. '123' is a string here")
w("# even though it parses as a decimal integer, hence the as_str accessor.")
w("pair.default.mesh_name = %s" % mesh_common.DEFAULT_MESH_NAME)
w("pair.default.password = %s" % mesh_common.DEFAULT_MESH_PASSWORD)
w("pair.default.ltk = %s" % bytesval(mesh_common.DEFAULT_LTK))
w("")

w("# --- Session-key derivation (mesh_common.process_session_key) -----------")
w("# Python-only: the firmware derives the same key during pairing, but there")
w("# is no firmware struct or constant to pin, so the Rust suite skips these")
w("# (declared PYTHON_ONLY_KEYS allowlist). Synthetic, not captured traffic.")
w("# With rands = response[1:9] (the server random, pair_rands) and")
w("#   encrypted = encrypt_data(rands padded to 16, credentials)")
w("# the response is")
w("#   opcode | rands(8) || reverse(encrypted[8:16])")
w("# and the session key is")
w("#   reverse(encrypt_data(credentials, shared_key(8) || rands(8)))")
w("# shared_key is the client random (pair_randm), a module global in")
w("# mesh_common, hence the save/set/restore in the Python test.")
w("pair.session_key.shared_key = %s" % bytesval(SESSION_SHARED_KEY))
w("pair.session_key.credentials = %s" % bytesval(SESSION_CREDENTIALS))
w("pair.session_key.response = %s" % bytesval(session_response))
w("pair.session_key.session_key = %s" % bytesval(session_key))
w("# response_invalid: one byte of the response's server result flipped, so")
w("# process_session_key must raise instead of deriving a key.")
w("pair.session_key.response_invalid = %s" % bytesval(session_response_invalid))
w("")

w("# --- AES-128 block parity table ----------------------------------------")
w("# Every distinct (key, plaintext) pair that aes_att_encryption_packet()")
w("# feeds to the AES primitive for the aes.packet.* vectors below, mapped to")
w("# the ciphertext returned by aes_att_encryption (Rust reverses key+input,")
w("# AES-ECB encrypts, reverses the output). Entries are numbered 1..N in")
w("# first-use order. The Rust suite mocks")
w("# aes_att_encryption from exactly these entries, so the set is closed:")
w("# a missing pair falls through to a loud zero-block stub.")
for i, (key, plain) in enumerate(block_keys, start=1):
    w("aes.block.%d.key = %s" % (i, bytesval(key)))
    w("aes.block.%d.plain = %s" % (i, bytesval(plain)))
    w("aes.block.%d.cipher = %s" % (i, bytesval(table[(key, plain)])))
w("")

w("# --- AES-128 authenticated packets (CTR + CBC-MAC, 2-byte MIC) ----------")
w("# cipher/mic are produced by aes_att_encryption_packet in crypto.rs and")
w("# verified by Rust aes_att_decryption_packet and Python")
w("# mesh_common.decrypt_notification. Wire form is")
w("#   data = sno(3) | src(2) | mic(2) | cipher,  ivm[3:6] = sno, ivm[6:8] = src")
w("# and the Python entry point wants mac_fwd = 00 00 00 | reverse(ivm[0:3]).")
w("# That derivation is only used to reach decrypt_notification; it does NOT")
w("# verify aes_att_encryption's byte-reversal convention (the Rust tests do).")
for idx, key, ivm, plain, cipher, mic in packets:
    note = "16-byte plaintext, one full block" if idx == 1 else \
           "24-byte plaintext, one full block + partial block"
    w("# packet %d: %s; synthetic payload, not captured traffic" % (idx, note))
    w("aes.packet.%d.key = %s" % (idx, bytesval(key)))
    w("aes.packet.%d.ivm = %s" % (idx, bytesval(ivm)))
    w("aes.packet.%d.plain = %s" % (idx, bytesval(plain)))
    w("aes.packet.%d.cipher = %s" % (idx, bytesval(cipher)))
    w("aes.packet.%d.mic = %s" % (idx, bytesval(mic)))
w("")

w("# --- CommandAction command encryption (mesh_common.CommandAction) -------")
w("# SYNTHETIC vectors, not captured traffic. Wire form is")
w("#   plain_cmd     = sno(3) | mac_slot(2) | payload(15)")
w("#   encrypted_cmd = sno(3) | mac(2)    | cipher(15)")
w("# encrypted_cmd is produced by the production CommandAction.generate_data,")
w("# which overwrites mac_slot with the 2-byte MAC and CTR-encrypts the 15-byte")
w("# payload; plain_cmd is the pre-call copy. Rust reproduces the 20 bytes with")
w("# aes_att_encryption_packet and round-trips them with aes_att_decryption_packet.")
for idx, plain_cmd, encrypted_cmd in cmd_action_vectors:
    note = "all-zero plaintext" if idx == 1 else "non-zero sno and payload"
    w("# vector %d: %s" % (idx, note))
    w("cmd_action.%d.session_key = %s" % (idx, bytesval(CMD_ACTION_SESSION_KEY)))
    w("cmd_action.%d.nonce = %s" % (idx, bytesval(CMD_ACTION_NONCE)))
    w("cmd_action.%d.plain_cmd = %s" % (idx, bytesval(plain_cmd)))
    w("cmd_action.%d.encrypted_cmd = %s" % (idx, bytesval(encrypted_cmd)))
w("")

w("# --- Firmware header (first 32 bytes of the image, rust/src/startup.rs) --")
w("# [0:2]   reset branch opcode   -- SYNTHETIC, see ota.header.reset_opcode")
w("# [2:6]   BUILD_VERSION u32 LE, bit 31 clear (rust/src/version.rs)")
w("# [6:8]   zero (the .org 0x8 gap)")
w("# [8:12]  .word 0x544c4e4b little-endian => 4b 4e 4c 54, i.e. 'KNLT' on flash")
w("# [12:16] .word 0x00880120 (RAM-code size word), see ota.header.ramcode_word")
w("# [16:18] IRQ branch opcode     -- SYNTHETIC, see ota.header.irq_opcode")
w("# [18:24] zero (.org 0x18 gap)")
w("# [24:28] _bin_size_ u32 LE (matches ota.blob length)")
w("# [28:32] zero placeholder: a real image carries FW_DEV_INFO (0x00000004);")
w("#         neither tool reads this word, so it is not asserted anywhere.")
w("ota.header = %s" % bytesval(ota_header))
w("")
w("# SYNTHETIC: the TC32 `tj` encoding for the two vector-table branches is not")
w("# known here, so these are clearly-fake sentinels (de ad / be ef). Both")
w("# suites assert them byte-for-byte against ota.header without treating them")
w("# as device facts. Replace with real opcodes if the encoding is confirmed.")
w("ota.header.reset_opcode = %s" % bytesval(RESET_OPCODE))
w("ota.header.ramcode_word = %s" % bytesval(RAMCODE_WORD))
w("ota.header.irq_opcode = %s" % bytesval(IRQ_OPCODE))
w("")

w("# --- OTA constants ------------------------------------------------------")
w("# 0x7fffffff masks the legacy bit 31 that pre-unification 16 MHz builds set.")
w("ota.version_mask = %s" % hexval(mesh_common.VERSION_MASK))
w("# rust/src/ota.rs FLASH_ADR_OTA_READY_FLAG / FLASH_OTA_READY_FLAG;")
w("# rust/src/config.rs FLASH_ADR_LIGHT_NEW_FW.")
w("ota.flash_adr_ota_ready_flag = 0x3f000")
w("ota.flash_ota_ready_flag = 0xa5")
w("ota.flash_adr_light_new_fw = 0x40000")
w("")

w("# --- Synthetic 64-byte firmware blob -----------------------------------")
w("# First 32 bytes are ota.header; the remaining 32 are the leading digits of")
w("# pi as bytes (0x31 0x41 0x59 ...) -- deterministic filler, not real code.")
w("ota.blob = %s" % bytesval(ota_blob))
w("")

w("# --- OTA transport packets over ota.blob -------------------------------")
w("# Produced by the real parsers, not reimplemented:")
w("#   BLEPacketParser  -> 20 B: index u16 LE | 16 B chunk | crc16 LE over 0..18")
w("#   UARTPacketParser -> 10 B: index u16 LE | 8 B chunk (no per-chunk CRC)")
w("# 64 bytes divides evenly, so no 0xFF padding occurs in either family.")
w("ota.ble.packet_size = 20")
w("ota.ble.chunk_size = 16")
for i, p in enumerate(ble_packets):
    w("ota.ble.packet.%d = %s" % (i, bytesval(p)))
w("")
w("ota.uart.packet_size = 10")
w("ota.uart.chunk_size = 8")
for i, p in enumerate(uart_packets):
    w("ota.uart.packet.%d = %s" % (i, bytesval(p)))
w("")

fixture = "\n".join(out) + "\n"

out_path = Path(__file__).resolve().parent / "wire_vectors.txt"
with open(out_path, "w", encoding="utf-8", newline="\n") as f:
    f.write(fixture)

key_count = sum(
    1 for line in fixture.splitlines() if line.strip() and not line.startswith("#")
)
print("wrote %s" % out_path)
print("keys: %d" % key_count)
print("aes block entries: %d" % len(block_keys))
print("aes primitive calls: %d (packet 1: %d, packet 2: %d)"
      % (len(call_log), pkt1_calls, pkt2_calls))
