#!/usr/bin/env python3
"""Cross-boundary wire-contract tests for the TLSR8266 host tooling.

The Python half of the golden-vector suite described in
``docs/plans/designs/010-r2-wire-contract-golden-tests.md``. It parses the
committed fixture ``tests/wire_vectors.txt`` -- the same file the Rust firmware
suite reads -- and asserts every value the host tools share with the firmware:
CRC-16-MODBUS, UART framing and opcodes, GATT UUIDs, pairing constants,
AES-128 hardware-parity crypto, and the 32-byte firmware header.

The fixture is the contract. If a test here fails, the protocol drifted; fix
the code (or regenerate the fixture deliberately), never the assertion.

Run from the repository root:

    PYTHONPATH=utilities/meshutils python3 -m unittest discover \\
        -s utilities/meshutils/tests -v

Import-time note: ``flash_fw`` needs ``bleak`` and ``flash_fw_uart`` needs
``pyserial``, so those distributions must be installed even though no test
opens a BLE or serial connection.
"""

import re
import unittest
from pathlib import Path

import mesh_common
import uart_constants
import flash_fw
import flash_fw_uart

FIXTURE_PATH = Path(__file__).resolve().parents[3] / "tests" / "wire_vectors.txt"

# Keys asserted only by the Rust suite: Python has no struct layout (``size.*``)
# and no equivalent of the two firmware-only pairing opcodes.
RUST_ONLY_KEY_PREFIXES = ("size.",)
RUST_ONLY_KEYS = frozenset(
    {
        "pair.op.reset_mesh",
        "pair.op.delete_pairing",
    }
)

_HEX_BYTE = re.compile(r"[0-9a-f]{2}")


class FixtureError(Exception):
    """The shared fixture is malformed, or a test asked for a missing key."""


def _read_fixture(path):
    """Parse ``key = value`` lines into a dict, rejecting duplicates."""
    values = {}
    for lineno, raw in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        key, separator, value = line.partition("=")
        if not separator:
            raise FixtureError(f"{path}:{lineno}: expected 'key = value', got {raw!r}")
        key = key.strip()
        value = value.strip()
        if not key:
            raise FixtureError(f"{path}:{lineno}: empty key in {raw!r}")
        if key in values:
            raise FixtureError(f"{path}:{lineno}: duplicate key {key!r}")
        values[key] = value
    return values


# Keys read through an accessor so far. Module-level rather than per-instance so
# the orphan guard sees every read regardless of which ``Fixture`` object did it.
CONSUMED_KEYS = set()


class Fixture:
    """Read-only fixture view; every accessor records the key it read."""

    def __init__(self, path):
        self.path = path
        self._values = _read_fixture(path)

    def as_str(self, key):
        """Return the raw value as a string."""
        return self._lookup(key)

    def as_u8(self, key):
        """Return the value as an unsigned 8-bit integer."""
        return self._as_int(key, 0xFF)

    def as_u32(self, key):
        """Return the value as an unsigned 32-bit integer."""
        return self._as_int(key, 0xFFFFFFFF)

    def as_bytes(self, key):
        """Return the value as bytes parsed from space-separated hex pairs."""
        text = self._lookup(key)
        tokens = text.split()
        if not tokens:
            raise FixtureError(f"{key}: expected hex bytes, got an empty value")
        value = bytearray()
        for token in tokens:
            if not _HEX_BYTE.fullmatch(token):
                raise FixtureError(
                    f"{key}: {token!r} is not a lowercase 2-hex-digit byte"
                )
            value.append(int(token, 16))
        return bytes(value)

    def keys(self):
        """All keys present in the fixture."""
        return frozenset(self._values)

    def _as_int(self, key, maximum):
        text = self._lookup(key)
        try:
            base = 16 if text.lower().startswith("0x") else 10
            value = int(text, base)
        except ValueError:
            raise FixtureError(f"{key}: expected an integer, got {text!r}") from None
        if not 0 <= value <= maximum:
            raise FixtureError(f"{key}: {value} does not fit in 0..{maximum}")
        return value

    def _lookup(self, key):
        try:
            value = self._values[key]
        except KeyError:
            raise FixtureError(f"{key}: no such fixture key") from None
        CONSUMED_KEYS.add(key)
        return value


FIXTURE = Fixture(FIXTURE_PATH)


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


def aes_packet_vector(number):
    """Return the derived fields of the ``aes.packet.N`` vector.

    ``mac_fwd``/``sno``/``src`` are reconstructed the way the firmware builds
    them: the notification's IV is ``mac[0:3] | sno(3) | src(2)`` and
    ``decrypt_notification`` wants the MAC in display order.
    """
    ivm = FIXTURE.as_bytes(f"aes.packet.{number}.ivm")
    if len(ivm) != 8:
        raise FixtureError(f"aes.packet.{number}.ivm: expected 8 bytes, got {len(ivm)}")
    return {
        "key": FIXTURE.as_bytes(f"aes.packet.{number}.key"),
        "plain": FIXTURE.as_bytes(f"aes.packet.{number}.plain"),
        "cipher": FIXTURE.as_bytes(f"aes.packet.{number}.cipher"),
        "mic": FIXTURE.as_bytes(f"aes.packet.{number}.mic"),
        "sno": ivm[3:6],
        "src": ivm[6:8],
        "mac_fwd": b"\x00\x00\x00" + bytes(reversed(ivm[0:3])),
    }


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


class SessionKeyTests(unittest.TestCase):
    """``process_session_key`` checks the server response, then derives the key.

    ``mesh_common.process_session_key`` reads the **module global**
    ``shared_key`` (the client random, ``pair_randm``) rather than taking it as
    an argument. This test therefore saves the import-time value, substitutes
    the fixture's, and restores it in ``tearDown`` *and* in a ``try``/``finally``
    around the calls: a failed assertion must not leak the substituted global
    into every later test in the same process.
    """

    def setUp(self):
        self._saved_shared_key = mesh_common.shared_key

    def tearDown(self):
        mesh_common.shared_key = self._saved_shared_key

    def test_derivation(self):
        shared_key = FIXTURE.as_bytes("pair.session_key.shared_key")
        credentials = FIXTURE.as_bytes("pair.session_key.credentials")
        response = FIXTURE.as_bytes("pair.session_key.response")
        expected = FIXTURE.as_bytes("pair.session_key.session_key")
        invalid = FIXTURE.as_bytes("pair.session_key.response_invalid")

        self.assertEqual(len(shared_key), mesh_common.RANDOM_CHALLENGE_SIZE)
        self.assertEqual(len(credentials), mesh_common.KEY_SIZE)
        self.assertEqual(len(response), mesh_common.KEY_SIZE + 1)
        self.assertEqual(len(expected), mesh_common.KEY_SIZE)
        self.assertEqual(len(invalid), len(response))

        mesh_common.shared_key = shared_key
        try:
            self.assertEqual(
                bytes(
                    mesh_common.process_session_key(response, credentials)
                ),
                expected,
            )
            # A flipped byte in the server result must be rejected, not derived.
            with self.assertRaises(Exception):
                mesh_common.process_session_key(invalid, credentials)
        finally:
            mesh_common.shared_key = self._saved_shared_key


class AesBlockTests(unittest.TestCase):
    """AES-128 hardware-parity block encryption and its inverse."""

    def test_round_trip(self):
        for number in range(1, 9):
            key = FIXTURE.as_bytes(f"aes.block.{number}.key")
            plain = FIXTURE.as_bytes(f"aes.block.{number}.plain")
            cipher = FIXTURE.as_bytes(f"aes.block.{number}.cipher")
            with self.subTest(block=number):
                self.assertEqual(len(key), mesh_common.KEY_SIZE)
                self.assertEqual(len(plain), mesh_common.KEY_SIZE)
                self.assertEqual(len(cipher), mesh_common.KEY_SIZE)
                self.assertEqual(mesh_common.aes_att_encryption(key, plain), cipher)
                self.assertEqual(mesh_common.aes_att_decryption(key, cipher), plain)


class AesPacketTests(unittest.TestCase):
    """Authenticated notifications: CTR decryption plus CBC-MAC verification."""

    def test_decrypt_notification(self):
        for number in (1, 2):
            vector = aes_packet_vector(number)
            with self.subTest(packet=number):
                self.assertEqual(len(vector["mic"]), 2)
                data = (
                    vector["sno"]
                    + vector["src"]
                    + vector["mic"]
                    + vector["cipher"]
                )
                self.assertEqual(
                    mesh_common.decrypt_notification(
                        data, vector["key"], vector["mac_fwd"]
                    ),
                    vector["plain"],
                )

    def test_corrupted_mic_is_rejected(self):
        for number in (1, 2):
            vector = aes_packet_vector(number)
            bad_mic = bytearray(vector["mic"])
            bad_mic[0] ^= 0x01
            data = vector["sno"] + vector["src"] + bytes(bad_mic) + vector["cipher"]
            with self.subTest(packet=number):
                self.assertIsNone(
                    mesh_common.decrypt_notification(
                        data, vector["key"], vector["mac_fwd"]
                    )
                )

    def test_corrupted_ciphertext_is_rejected(self):
        for number in (1, 2):
            vector = aes_packet_vector(number)
            bad_cipher = bytearray(vector["cipher"])
            bad_cipher[0] ^= 0x01
            data = vector["sno"] + vector["src"] + vector["mic"] + bytes(bad_cipher)
            with self.subTest(packet=number):
                self.assertIsNone(
                    mesh_common.decrypt_notification(
                        data, vector["key"], vector["mac_fwd"]
                    )
                )


class FirmwareHeaderTests(unittest.TestCase):
    """The 32-byte image header every OTA image starts with."""

    def test_header_layout(self):
        header = FIXTURE.as_bytes("ota.header")
        reset_opcode = FIXTURE.as_bytes("ota.header.reset_opcode")
        ramcode_word = FIXTURE.as_bytes("ota.header.ramcode_word")
        irq_opcode = FIXTURE.as_bytes("ota.header.irq_opcode")

        self.assertEqual(len(header), 32)
        self.assertEqual(len(reset_opcode), 2)
        self.assertEqual(len(ramcode_word), 4)
        self.assertEqual(len(irq_opcode), 2)

        # [0:2] branch to __reset.
        self.assertEqual(header[0:2], reset_opcode)
        # [2:6] BUILD_VERSION, u32 LE, legacy bit 31 ignored by both tools.
        version = int.from_bytes(header[2:6], "little")
        self.assertEqual(version & 0x80000000, 0)
        # [6:8] alignment gap.
        self.assertEqual(header[6:8], b"\x00\x00")
        # [8:12] `.word 0x544c4e4b` in flash order reads as KNLT, not TLNK.
        self.assertEqual(header[8:12], b"KNLT")
        self.assertEqual(header[12:16], ramcode_word)
        self.assertEqual(header[16:18], irq_opcode)
        # [18:24] alignment gap.
        self.assertEqual(header[18:24], b"\x00" * 6)
        # [24:28] _bin_size_, the payload length that follows the header.
        blob = FIXTURE.as_bytes("ota.blob")
        self.assertEqual(int.from_bytes(header[24:28], "little"), len(blob))

    def test_blob_starts_with_header(self):
        blob = FIXTURE.as_bytes("ota.blob")
        self.assertEqual(len(blob), 64)
        self.assertEqual(blob[:32], FIXTURE.as_bytes("ota.header"))


class OtaConstantsTests(unittest.TestCase):
    """OTA version masking and the flash layout it is applied to."""

    def test_version_mask(self):
        self.assertEqual(
            mesh_common.VERSION_MASK,
            FIXTURE.as_u32("ota.version_mask"),
        )

    def test_flash_layout(self):
        # rust/src/ota/mod.rs:133-135 and rust/src/config.rs:46. The host tools do not
        # name these addresses, but they are shared layout: the ready flag sits
        # in the 4 KiB sector directly below the staging area.
        ready_flag_address = FIXTURE.as_u32("ota.flash_adr_ota_ready_flag")
        ready_flag = FIXTURE.as_u8("ota.flash_ota_ready_flag")
        staging_address = FIXTURE.as_u32("ota.flash_adr_light_new_fw")

        self.assertEqual(ready_flag_address % 0x1000, 0)
        self.assertEqual(staging_address % 0x1000, 0)
        self.assertEqual(staging_address - ready_flag_address, 0x1000)
        # A deliberate sentinel: blank (0x00) or erased (0xFF) flash must never
        # read as "OTA image ready".
        self.assertNotIn(ready_flag, (0x00, 0xFF))


class BlePacketParserTests(unittest.TestCase):
    """20-byte BLE OTA packets over the shared firmware blob."""

    def test_packet_vectors(self):
        blob = FIXTURE.as_bytes("ota.blob")
        parser = flash_fw.BLEPacketParser(blob)

        packet_size = FIXTURE.as_u32("ota.ble.packet_size")
        chunk_size = FIXTURE.as_u32("ota.ble.chunk_size")
        self.assertEqual(packet_size, 20)
        self.assertEqual(chunk_size, 16)
        # The parser splits on the chunk size and headers each chunk with the
        # 2-byte index plus the 2-byte CRC.
        self.assertEqual(parser.packet_size, chunk_size)
        self.assertEqual(flash_fw.PACKET_SIZE, chunk_size)
        self.assertEqual(parser.total, len(blob) // chunk_size)

        for index in range(4):
            with self.subTest(packet=index):
                packet = parser.get_packet(index)
                self.assertEqual(len(packet), packet_size)
                self.assertEqual(
                    bytes(packet),
                    FIXTURE.as_bytes(f"ota.ble.packet.{index}"),
                )


class UartPacketParserTests(unittest.TestCase):
    """10-byte UART OTA packets over the same firmware blob."""

    def test_packet_vectors(self):
        blob = FIXTURE.as_bytes("ota.blob")
        parser = flash_fw_uart.UARTPacketParser(blob)

        packet_size = FIXTURE.as_u32("ota.uart.packet_size")
        chunk_size = FIXTURE.as_u32("ota.uart.chunk_size")
        self.assertEqual(packet_size, 10)
        self.assertEqual(chunk_size, 8)
        self.assertEqual(parser.packet_size, chunk_size)
        self.assertEqual(uart_constants.PACKET_SIZE, chunk_size)
        self.assertEqual(parser.total, len(blob) // chunk_size)

        for index in range(8):
            with self.subTest(packet=index):
                packet = parser.get_packet(index)
                self.assertEqual(len(packet), packet_size)
                self.assertEqual(
                    bytes(packet),
                    FIXTURE.as_bytes(f"ota.uart.packet.{index}"),
                )


class ZzOrphanVectorTests(unittest.TestCase):
    """Every shared fixture key must be load-bearing for the Python suite.

    The accounting is :data:`CONSUMED_KEYS`, a module-level set every accessor
    populates, so the guard does not depend on one ``Fixture`` instance. The
    ``Zz`` prefix still keeps this class last in unittest's alphabetical order,
    but the check no longer *requires* that order: run on its own, nothing has
    been consumed yet and the guard skips instead of reporting every key as an
    orphan.
    """

    def test_no_orphan_vectors(self):
        if not CONSUMED_KEYS:
            self.skipTest("no fixture keys consumed yet; run the whole module")

        all_keys = FIXTURE.keys()
        rust_only = {
            key for key in all_keys if key.startswith(RUST_ONLY_KEY_PREFIXES)
        } | RUST_ONLY_KEYS
        shared = all_keys - rust_only

        self.assertEqual(len(rust_only), 17)
        self.assertEqual(sorted(shared - CONSUMED_KEYS), [])
        self.assertEqual(sorted(CONSUMED_KEYS - shared), [])
        self.assertEqual(len(shared), 126)


if __name__ == "__main__":
    unittest.main()
