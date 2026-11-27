#!/usr/bin/env python3
"""SDK and cross-language compatibility fixture validation."""

import json
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "sdk"))

import stack_analysis  # noqa: E402

FIXTURES = REPO_ROOT / "tests" / "fixtures"


class FixtureError(Exception):
    pass


def read_fixture(path):
    values = {}
    for lineno, raw in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        key, separator, value = line.partition("=")
        if not separator:
            raise FixtureError(f"{path}:{lineno}: expected 'key = value'")
        key = key.strip()
        value = value.strip()
        if not key:
            raise FixtureError(f"{path}:{lineno}: empty key")
        if key in values:
            raise FixtureError(f"{path}:{lineno}: duplicate key {key}")
        values[key] = value
    return values


def as_int(values, key, maximum=None):
    text = values[key]
    base = 16 if text.lower().startswith("0x") else 10
    value = int(text, base)
    if maximum is not None and not 0 <= value <= maximum:
        raise FixtureError(f"{key}: {value} out of range")
    return value


def as_bytes(values, key):
    text = values[key]
    if text == "none":
        return b""
    return bytes(int(token, 16) for token in text.split())


class BaselineFixtureTests(unittest.TestCase):
    def test_strict_baseline_schema_and_safety_ceilings(self):
        required_keys = {
            "elf_bytes", "bin_bytes", "sections", "free_main_sram",
            "main_max", "irq_max", "main_margin", "irq_margin",
            "parsed_functions", "indirect_functions", "allowed_indirect",
            "legacy_edges_resolved", "gatt_edges_resolved",
            "runtime_allowed_indirect", "mode", "strict",
        }
        required_sections = {".text", ".rodata", ".data", ".bss", ".ram_code"}

        for name in ("baseline_default.json", "baseline_rtt.json"):
            with self.subTest(baseline=name):
                path = FIXTURES / name
                baseline = json.loads(path.read_text(encoding="utf-8"))
                self.assertEqual(set(baseline), required_keys)
                self.assertEqual(baseline["mode"], "strict")
                self.assertIs(baseline["strict"], True)
                self.assertLessEqual(baseline["bin_bytes"], 131072)
                self.assertLessEqual(baseline["irq_max"], 1392)
                self.assertGreaterEqual(baseline["main_margin"], 400)
                self.assertGreaterEqual(baseline["irq_margin"], 400)
                self.assertEqual(baseline["gatt_edges_resolved"], 5)
                sections = baseline["sections"]
                self.assertTrue(required_sections.issubset(sections))
                for section in required_sections:
                    self.assertIsInstance(sections[section], int)
                    self.assertGreaterEqual(sections[section], 0)


class DomainFixtureStructuralTests(unittest.TestCase):
    def test_gatt_span_partition_and_row_completeness(self):
        gatt = read_fixture(FIXTURES / "gatt_compat.txt")
        self.assertEqual(as_int(gatt, "gatt.table.count", 0xFF), 29)
        self.assertEqual(as_int(gatt, "gatt.table.max_valid_handle", 0xFF), 28)
        self.assertEqual(as_int(gatt, "gatt.att.mtu", 0xFF), 23)

        for handle in range(29):
            prefix = f"gatt.row.{handle:02d}"
            for suffix in ("span", "uuid_len", "uuid", "default_len", "max_len", "value", "behavior"):
                self.assertIn(f"{prefix}.{suffix}", gatt)

        span_starts = []
        for handle, span_key in ((1, "gatt.row.01.span"),
                                 (7, "gatt.row.07.span"),
                                 (16, "gatt.row.16.span")):
            span_starts.append((handle, as_int(gatt, span_key, 0xFF)))
        cursor = 1
        for start, span in span_starts:
            self.assertEqual(start, cursor)
            cursor += span
        self.assertEqual(cursor, 29)

        readonly = {int(part, 10) for part in gatt["att.readonly_handles"].split()}
        self.assertEqual(readonly, {3, 9, 11})

    def test_mesh_uart_fixture_is_structurally_complete(self):
        mesh = read_fixture(FIXTURES / "mesh_uart_compat.txt")
        for key in (
            "uart.packet_length", "uart.payload_offset", "uart.crc_offset",
            "uart.light_ctrl.payload_start", "uart.light_ctrl.payload_end",
            "uart.light_ctrl.retransmit_offset", "uart.light_ctrl.ack_offset",
            "mesh.status.node_val_len", "mesh.status.node_par_len",
            "mesh.status.value_len", "mesh.status.records_per_packet",
            "mesh.vendor_id", "mesh.spp_handle_21.ingress", "mesh.uart_02.ingress",
        ):
            self.assertIn(key, mesh)
        self.assertEqual(as_int(mesh, "uart.packet_length", 0xFF), 44)
        self.assertEqual(as_int(mesh, "mesh.status.node_val_len", 0xFF), 4)
        self.assertEqual(as_int(mesh, "mesh.status.node_par_len", 0xFF), 2)
        self.assertEqual(int(mesh["mesh.vendor_id"], 16), 0x0211)

    def test_flash_map_single_coverage_and_reset_last(self):
        ota = read_fixture(FIXTURES / "ota_flash_adv_compat.txt")
        sector_size = as_int(ota, "flash.sector_size")
        total_size = as_int(ota, "flash.total_size")
        region_count = as_int(ota, "flash.region_count", 0xFF)
        self.assertEqual(sector_size, 0x1000)
        self.assertEqual(total_size, 0x80000)
        self.assertEqual(region_count, 14)

        cursor = 0
        total_sectors = 0
        erased_regions = []
        for index in range(region_count):
            prefix = f"flash.region.{index:02d}"
            start = as_int(ota, f"{prefix}.start")
            end = as_int(ota, f"{prefix}.end")
            sectors = as_int(ota, f"{prefix}.sectors", 0xFF)
            policy = ota[f"{prefix}.reset_policy"]
            self.assertEqual(start, cursor)
            self.assertEqual(start % sector_size, 0)
            self.assertEqual(end % sector_size, 0)
            self.assertEqual(sectors, (end - start) // sector_size)
            cursor = end
            total_sectors += sectors
            if policy == "reset_erase_last":
                erased_regions.append(start)

        self.assertEqual(cursor, total_size)
        self.assertEqual(total_sectors, total_size // sector_size)
        self.assertEqual(erased_regions, [as_int(ota, "flash.reset_erase_last_addr")])

    def test_no_forbidden_implementation_names_in_txt_fixtures(self):
        forbidden = (
            "crate::", "tlsr8266_mesh", "tlsr8266_radio", "tlsr8266_hal",
            "mesh-protocol", ".rs:", ".rs)", "MESH_TX_QUEUE", "MESH_RX_CHANNEL",
            "SEND_TO_MASTER", "RC_PKT_BUF", "rf_link", "on_gatt", "on_light_data",
            "gatt_table", "attribute_table", "C_TABLE", "c_table",
        )
        for path in FIXTURES.glob("*.txt"):
            text = path.read_text(encoding="ascii")
            for name in forbidden:
                self.assertNotIn(name, text, f"{path.name}: forbidden implementation name {name}")


class HilFixtureSchemaTests(unittest.TestCase):
    def test_hil_matrix_schema_when_present(self):
        path = FIXTURES / "hil_matrix.json"
        if not path.exists():
            self.skipTest("HIL matrix not present")
        matrix = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(matrix["schema_version"], 1)
        self.assertIs(matrix["secret_free"], True)
        self.assertEqual(set(matrix["images"]), {"rtt", "base", "next"})
        for profile in ("rtt", "base", "next"):
            image = matrix["images"][profile]
            self.assertIn("elf", image)
            self.assertIn("bin", image)

        bench = matrix["bench"]
        self.assertEqual(bench["host"], "root@10.0.0.150")
        self.assertEqual(bench["artifacts_dir"], "/root/hil-accept/artifacts")
        self.assertEqual(bench["runner"], "/root/hil-accept/run.sh")

        preflight = matrix["preflight"]
        self.assertEqual(preflight["dut_mesh_addr_range"], [1, 63])
        self.assertIs(preflight["allow_silent_fallback"], False)
        self.assertIs(preflight["same_artifacts_across_crystals"], True)
        self.assertIs(preflight["no_env_read"], True)

        self.assertEqual(matrix["stages"], [0, 1, 2, 3, 4, 5])
        self.assertEqual(set(matrix["stage_assertions"]), {str(stage) for stage in range(6)})
        for profile, crystal, stages in (
            ("12mhz", 12, [0, 1, 2, 3, 4, 5]),
            ("16mhz", 16, [0, 1, 2, 3, 4]),
        ):
            self.assertEqual(matrix["profiles"][profile]["crystal_mhz"], crystal)
            if profile == "12mhz":
                self.assertEqual(matrix["profiles"][profile]["stages"], stages)
            else:
                self.assertEqual(matrix["profiles"][profile]["stages"], stages)


if __name__ == "__main__":
    unittest.main()
