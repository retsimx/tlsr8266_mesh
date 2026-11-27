#!/usr/bin/env python3
"""Python cross-consumption tests for Issue #114 compatibility fixtures."""

import asyncio
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import flash_fw
import flash_fw_uart
import mesh_common
import mesh_node
import uart_constants

from wire_contract_fixture import Fixture

REPO_ROOT = Path(__file__).resolve().parents[3]
FIXTURES = REPO_ROOT / "tests" / "fixtures"
GATT = Fixture(FIXTURES / "gatt_compat.txt")
MESH_UART = Fixture(FIXTURES / "mesh_uart_compat.txt")
OTA_FLASH_ADV = Fixture(FIXTURES / "ota_flash_adv_compat.txt")


def _canonical_128_uuid(raw):
    bare = bytes(reversed(raw)).hex()
    return "{}-{}-{}-{}-{}".format(
        bare[0:8], bare[8:12], bare[12:16], bare[16:20], bare[20:32]
    )


class GattFixtureTests(unittest.TestCase):
    def test_table_metadata_and_att_constants(self):
        self.assertEqual(GATT.as_u8("gatt.table.count"), 29)
        self.assertEqual(GATT.as_u8("gatt.table.max_valid_handle"), 28)
        self.assertEqual(GATT.as_u8("gatt.att.mtu"), 23)
        self.assertEqual(GATT.as_u8("gatt.att.read_by_type_max_value_len"), 19)
        self.assertEqual(GATT.as_u8("gatt.att.read_rsp_max_value_len"), 22)
        self.assertEqual(GATT.as_u8("gatt.att.error_opcode"), 0x01)
        self.assertEqual(GATT.as_u8("gatt.att.error_reason"), 0x0A)
        self.assertEqual(int(GATT.as_str("gatt.att.write_rsp_opcode"), 16), 0x13)
        self.assertEqual(GATT.as_u8("gatt.att.write_rsp_l2cap_len"), 1)

    def test_span_partition_and_all_rows(self):
        self.assertEqual(GATT.as_u8("gatt.row.00.span"), 28)

        spans = [(1, GATT.as_u8("gatt.row.01.span")),
                 (7, GATT.as_u8("gatt.row.07.span")),
                 (16, GATT.as_u8("gatt.row.16.span"))]
        cursor = 1
        for start, span in spans:
            self.assertEqual(start, cursor)
            cursor += span
        self.assertEqual(cursor, 29)

        for handle in range(29):
            prefix = f"gatt.row.{handle:02d}"
            GATT.as_u8(f"{prefix}.span")
            GATT.as_u8(f"{prefix}.uuid_len")
            GATT.as_str(f"{prefix}.uuid")
            GATT.as_u8(f"{prefix}.default_len")
            GATT.as_u8(f"{prefix}.max_len")
            GATT.as_str(f"{prefix}.value")
            GATT.as_str(f"{prefix}.behavior")

        self.assertEqual(GATT.as_bytes("gatt.row.03.value"), b"Telink tLight")
        self.assertEqual(GATT.as_bytes("gatt.row.11.value"), b"out_of_mesh")
        self.assertEqual(GATT.as_bytes("gatt.row.13.value"), b"model id 123")
        self.assertEqual(GATT.as_bytes("gatt.row.19.value"), bytes((0x01, 0x00)))
        self.assertEqual(GATT.as_bytes("gatt.row.21.value"), bytes(16))
        self.assertEqual(GATT.as_bytes("gatt.row.24.value"), b"\xe0" * 16)
        self.assertEqual(GATT.as_bytes("gatt.row.27.value"), b"\xe0" * 16)

    def test_characteristic_uuid_contract(self):
        uuids = {
            18: mesh_common.notify_characteristic_uuid,
            21: mesh_common.command_characteristic_uuid,
            24: mesh_common.ota_characteristic_uuid,
            27: mesh_common.pair_characteristic_uuid,
        }
        for handle, module_uuid in uuids.items():
            raw = GATT.as_bytes(f"gatt.row.{handle:02d}.uuid")
            self.assertEqual(_canonical_128_uuid(raw), module_uuid)

        firmware_row = GATT.as_bytes("gatt.row.09.uuid")
        self.assertEqual(firmware_row, bytes((0x26, 0x2A)))

    def test_att_opcodes_and_outcomes_are_consumed(self):
        for key in sorted(GATT.keys()):
            if key.startswith("att.op.") or key.startswith("att.case."):
                GATT.as_str(key)
        self.assertEqual(int(GATT.as_str("att.op.read_rsp"), 16), 0x0B)
        self.assertEqual(int(GATT.as_str("att.op.write_rsp"), 16), 0x13)
        self.assertEqual(int(GATT.as_str("att.op.write_cmd"), 16), 0x52)


class MeshUartFixtureTests(unittest.TestCase):
    def test_uart_layout_and_opcodes(self):
        self.assertEqual(uart_constants.PACKET_LENGTH, MESH_UART.as_u8("uart.packet_length"))
        self.assertEqual(uart_constants.PAYLOAD_OFFSET, MESH_UART.as_u8("uart.payload_offset"))
        self.assertEqual(uart_constants.CRC_OFFSET, MESH_UART.as_u8("uart.crc_offset"))
        self.assertEqual(uart_constants.LIGHT_CTRL, MESH_UART.as_u8("uart.op.light_ctrl"))
        self.assertEqual(uart_constants.LIGHT_STATUS, MESH_UART.as_u8("uart.op.light_status"))
        self.assertEqual(uart_constants.NODE_STATUS, MESH_UART.as_u8("uart.op.node_status"))
        self.assertEqual(uart_constants.ACK, MESH_UART.as_u8("uart.op.ack"))

        self.assertEqual(MESH_UART.as_u8("uart.light_ctrl.payload_len"), 13)
        self.assertEqual(MESH_UART.as_u8("uart.light_ctrl.dst_start"), 3)
        self.assertEqual(MESH_UART.as_u8("uart.light_ctrl.dst_end"), 5)
        self.assertEqual(MESH_UART.as_u8("uart.light_ctrl.payload_start"), 5)
        self.assertEqual(MESH_UART.as_u8("uart.light_ctrl.payload_end"), 18)
        self.assertEqual(MESH_UART.as_u8("uart.light_ctrl.retransmit_offset"), 18)
        self.assertEqual(MESH_UART.as_u8("uart.light_ctrl.ack_offset"), 19)
        self.assertEqual(MESH_UART.as_u8("uart.light_ctrl.fingerprint_start"), 3)
        self.assertEqual(MESH_UART.as_u8("uart.light_ctrl.fingerprint_end"), 18)
        self.assertEqual(MESH_UART.as_u8("uart.light_ctrl.ack_sno_start"), 2)
        self.assertEqual(MESH_UART.as_u8("uart.light_ctrl.ack_sno_end"), 5)

    def test_status_enable_and_node_status_projection(self):
        self.assertTrue(MESH_UART.as_bool("uart.light_status.platform_enable"))
        self.assertTrue(MESH_UART.as_bool("uart.light_status.zero_sno_ack"))
        self.assertTrue(MESH_UART.as_bool("uart.light_status.enable_mesh_reporting"))
        self.assertTrue(MESH_UART.as_bool("uart.light_status.enable_uart_reporting"))
        self.assertTrue(MESH_UART.as_bool("uart.node_status.on_off_from_par0"))
        self.assertEqual(MESH_UART.as_u8("uart.node_status.off_value"), 0)
        self.assertEqual(MESH_UART.as_u8("uart.node_status.on_value"), 1)
        self.assertEqual(MESH_UART.as_u8("uart.node_status_entry_len"), 3)
        self.assertEqual(MESH_UART.as_u8("uart.node_status_max_entries"), 13)

    def test_mesh_status_geometry_and_opcode_constants(self):
        self.assertEqual(MESH_UART.as_u8("mesh.status.node_val_len"), 4)
        self.assertEqual(MESH_UART.as_u8("mesh.status.node_par_len"), 2)
        self.assertEqual(MESH_UART.as_u8("mesh.status.value_len"), 24)
        self.assertEqual(MESH_UART.as_u8("mesh.status.records_per_packet"), 5)
        self.assertEqual(MESH_UART.as_u8("mesh.status.node_max_num"), 64)

        expected = (
            ("mesh.opcode.product.light_onoff", mesh_node.LGT_CMD_LIGHT_ONOFF),
            ("mesh.opcode.product.set_light", mesh_node.LGT_CMD_SET_LIGHT),
            ("mesh.opcode.product.light_read_status", mesh_node.LGT_CMD_LIGHT_READ_STATUS),
            ("mesh.opcode.product.light_status", mesh_node.LGT_CMD_LIGHT_STATUS),
            ("mesh.opcode.admin.grp_rsp1", mesh_node.LGT_CMD_LIGHT_GRP_RSP1),
            ("mesh.opcode.admin.config_grp", mesh_node.LGT_CMD_LIGHT_CONFIG_GRP),
            ("mesh.opcode.admin.grp_req", mesh_node.LGT_CMD_LIGHT_GRP_REQ),
            ("mesh.opcode.admin.config_dev_addr", mesh_node.LGT_CMD_CONFIG_DEV_ADDR),
            ("mesh.opcode.admin.kick_out", mesh_node.LGT_CMD_KICK_OUT),
            ("mesh.opcode.admin.set_mac_addr", mesh_node.LGT_CMD_SET_MAC_ADDR),
        )
        for key, value in expected:
            self.assertEqual(int(MESH_UART.as_str(key), 16), value)

        ota = (
            ("mesh.opcode.admin.ota_start_req", uart_constants.OTA_START),
            ("mesh.opcode.admin.ota_start_rsp", uart_constants.OTA_START_RESP),
            ("mesh.opcode.admin.ota_data_req", uart_constants.OTA_DATA),
            ("mesh.opcode.admin.ota_data_rsp", uart_constants.OTA_DATA_RESP),
            ("mesh.opcode.admin.ota_end_req", 0x28),
        )
        for key, value in ota:
            self.assertEqual(int(MESH_UART.as_str(key), 16), value)

    def test_shared_mesh_parameters_and_routing_facts(self):
        self.assertEqual(mesh_node.LIGHT_ADD_GRP_PARAM, MESH_UART.as_u8("mesh.light_add_grp_param"))
        self.assertEqual(mesh_node.LIGHT_DEL_GRP_PARAM, MESH_UART.as_u8("mesh.light_del_grp_param"))
        self.assertEqual(mesh_node.MAX_LUM_BRIGHTNESS_VALUE, MESH_UART.as_u16("mesh.max_lum_brightness_value"))
        self.assertEqual(int(MESH_UART.as_str("mesh.vendor_id"), 16), 0x0211)
        source = (REPO_ROOT / "utilities" / "meshutils" / "mesh_node.py").read_text(encoding="utf-8")
        self.assertIn("vendor_id=0x0211", source)

        for key in sorted(MESH_UART.keys()):
            if key.startswith("mesh.notify_req.") or key.startswith("mesh.notify_rsp."):
                MESH_UART.as_bool(key)
            elif key.startswith("mesh.request_response."):
                MESH_UART.as_u8(key)

        self.assertEqual(MESH_UART.as_str("mesh.spp_handle_21.ingress"), "mesh_routing")
        self.assertEqual(MESH_UART.as_str("mesh.uart_02.ingress"), "mesh_routing")
        self.assertEqual(MESH_UART.as_str("mesh.route.foreign_unicast"), "queue_tx_only")


class OtaFlashAdvertisingFixtureTests(unittest.TestCase):
    def test_ota_constants_and_packet_sizes(self):
        self.assertEqual(mesh_common.VERSION_MASK, OTA_FLASH_ADV.as_u32("ota.version_mask"))
        self.assertEqual(flash_fw.PACKET_SIZE, OTA_FLASH_ADV.as_u8("ota.ble_chunk_bytes"))
        self.assertEqual(uart_constants.PACKET_SIZE, OTA_FLASH_ADV.as_u8("ota.mesh_chunk_bytes"))
        self.assertEqual(OTA_FLASH_ADV.as_u32("ota.flash_adr_ota_ready_flag"), 0x3F000)
        self.assertEqual(OTA_FLASH_ADV.as_u8("ota.flash_ota_ready_flag"), 0xA5)
        self.assertEqual(OTA_FLASH_ADV.as_u32("ota.flash_adr_light_new_fw"), 0x40000)
        self.assertEqual(OTA_FLASH_ADV.as_u8("ota.erase_sectors_for_ota"), 32)
        self.assertEqual(OTA_FLASH_ADV.as_u32("ota.startup_flag"), 0x544C4E4B)
        self.assertEqual(OTA_FLASH_ADV.as_u8("ota.header_bin_size_offset"), 0x18)
        self.assertEqual(OTA_FLASH_ADV.as_u8("ota.timeout_default_s"), 30)
        self.assertEqual(OTA_FLASH_ADV.as_u8("ota.timeout_post_finish_s"), 4)
        self.assertEqual(OTA_FLASH_ADV.as_u8("ota.ble_packet_bytes"), 20)
        self.assertEqual(OTA_FLASH_ADV.as_u8("ota.mesh_packet_bytes"), 10)
        self.assertEqual(OTA_FLASH_ADV.as_u8("ota.downgrade_refusal_exit_code"), 2)

    def test_packet_window_and_crc_identity(self):
        slots = OTA_FLASH_ADV.as_u8("ota.packet_window.staging_slots")
        packet_size = OTA_FLASH_ADV.as_u8("ota.packet_window.packet_size")
        window = OTA_FLASH_ADV.as_u16("ota.packet_window.window_bytes")
        self.assertEqual((slots, packet_size, window), (16, 48, 768))
        self.assertTrue(OTA_FLASH_ADV.as_bool("ota.packet_window.shared_buffer"))
        self.assertEqual(OTA_FLASH_ADV.as_u8("ota.packet_window.extra_sram_bytes"), 0)

        golden = OTA_FLASH_ADV.as_bytes("sram.ident.golden_crc_input")
        expected = int(OTA_FLASH_ADV.as_str("sram.ident.golden_crc"), 16)
        self.assertEqual(mesh_common.crc16(list(golden), len(golden)), expected)

    def test_flash_map_exact_coverage_and_reset_order(self):
        sector_size = OTA_FLASH_ADV.as_u32("flash.sector_size")
        total = OTA_FLASH_ADV.as_u32("flash.total_size")
        count = OTA_FLASH_ADV.as_u8("flash.region_count")
        self.assertEqual(sector_size, 0x1000)
        self.assertEqual(total, 0x80000)
        self.assertEqual(count, 14)

        cursor = 0
        sectors = 0
        reset_last = OTA_FLASH_ADV.as_u32("flash.reset_erase_last_addr")
        for index in range(count):
            prefix = f"flash.region.{index:02d}"
            start = OTA_FLASH_ADV.as_u32(f"{prefix}.start")
            end = OTA_FLASH_ADV.as_u32(f"{prefix}.end")
            region_sectors = OTA_FLASH_ADV.as_u8(f"{prefix}.sectors")
            OTA_FLASH_ADV.as_str(f"{prefix}.name")
            OTA_FLASH_ADV.as_str(f"{prefix}.owner")
            OTA_FLASH_ADV.as_str(f"{prefix}.reset_policy")
            self.assertEqual(start, cursor)
            self.assertEqual(start % sector_size, 0)
            self.assertEqual(end % sector_size, 0)
            self.assertEqual(region_sectors, (end - start) // sector_size)
            cursor = end
            sectors += region_sectors
        self.assertEqual(cursor, total)
        self.assertEqual(sectors, total // sector_size)

        reset_count = OTA_FLASH_ADV.as_u32("flash.region.08.start")
        self.assertEqual(reset_count, reset_last)
        self.assertTrue(OTA_FLASH_ADV.as_bool("factory_reset.reset_count_erased_last"))

    def test_advertising_and_sram_identity(self):
        self.assertEqual(OTA_FLASH_ADV.as_bytes("advertising.init_flags"), bytes((0x02, 0x01, 0x05)))
        self.assertEqual(int(OTA_FLASH_ADV.as_str("advertising.vendor_id"), 16), 0x0211)
        self.assertEqual(mesh_common.DEFAULT_MESH_NAME, OTA_FLASH_ADV.as_str("advertising.out_of_mesh.name"))
        self.assertEqual(OTA_FLASH_ADV.as_bytes("advertising.out_of_mesh.primary_tlv")[:3], bytes((0x02, 0x01, 0x05)))
        self.assertEqual(OTA_FLASH_ADV.as_u8("advertising.out_of_mesh.primary_wire_len"), 28)
        self.assertEqual(OTA_FLASH_ADV.as_u8("advertising.out_of_mesh.rf_len"), 34)
        self.assertEqual(OTA_FLASH_ADV.as_u8("advertising.out_of_mesh.dma_len"), 36)

        self.assertEqual(OTA_FLASH_ADV.as_u32("sram.ident.mac_addr"), 0x808004)
        self.assertEqual(OTA_FLASH_ADV.as_u32("sram.ident.version_addr"), 0x808008)
        self.assertEqual(OTA_FLASH_ADV.as_u32("sram.ident.crc_addr"), 0x80800C)


class DowngradeGuardTests(unittest.TestCase):
    class _BleClient:
        def __init__(self, mac, dev_ver):
            self.mac = mac
            self.dev_ver = dev_ver

        async def __aenter__(self):
            return self

        async def __aexit__(self, exc_type, exc, tb):
            return False

        async def read_gatt_char(self, uuid):
            return struct.pack("I", self.dev_ver)

        async def write_gatt_char(self, uuid, data):
            return None

    def _run_main(self, dev_ver, fw_ver, force=False):
        blob = bytearray(64)
        struct.pack_into("I", blob, 2, fw_ver)
        with tempfile.NamedTemporaryFile(suffix=".bin") as firmware:
            firmware.write(blob)
            firmware.flush()
            argv = [
                "flash_fw.py", "--fw_file", firmware.name,
                "--mac", "11:22:33:44:55:66",
                "--mesh_name", "test", "--mesh_password", "test",
            ]
            if force:
                argv.append("--force")

            client = lambda mac: self._BleClient(mac, dev_ver)
            with mock.patch.object(sys, "argv", argv), \
                 mock.patch.object(flash_fw, "BleakClient", client), \
                 mock.patch.object(flash_fw, "authenticate", new_callable=mock.AsyncMock, return_value=b"key"):
                return asyncio.run(flash_fw.main())

    def test_masked_equal_or_older_refuses_with_exit_code_two(self):
        self.assertEqual(self._run_main(0x80000005, 0x00000005), 2)
        self.assertEqual(self._run_main(0x00000005, 0x00000005), 2)
        self.assertEqual(self._run_main(0x80000006, 0x00000005), 2)

    def test_newer_firmware_or_force_is_accepted(self):
        self.assertEqual(self._run_main(0x00000004, 0x00000005), 0)
        self.assertEqual(self._run_main(0x00000005, 0x00000004, force=True), 0)


class ZzCompatFixtureOrphanTests(unittest.TestCase):
    def test_no_orphan_domain_fixture_keys(self):
        for fixture in (GATT, MESH_UART, OTA_FLASH_ADV):
            for key in fixture.keys():
                fixture.as_str(key)
            self.assertEqual(fixture.orphan_keys(), frozenset(), (fixture.path, sorted(fixture.orphan_keys())))


if __name__ == "__main__":
    unittest.main()
