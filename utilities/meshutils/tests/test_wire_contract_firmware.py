#!/usr/bin/env python3
"""Firmware namespace: image header, OTA flash layout and OTA packet framing.

Part of the Python half of the golden-vector suite; see
``wire_contract_fixture`` for the shared fixture loader and how to run it.
"""

import unittest

import flash_fw
import flash_fw_uart
import mesh_common
import uart_constants

from wire_contract_fixture import FIXTURE


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


if __name__ == "__main__":
    unittest.main()
