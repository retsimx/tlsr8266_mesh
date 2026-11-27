"""Unit tests for the stdlib HIL acceptance driver."""

from __future__ import annotations

import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "sdk"))

import hil_acceptance  # noqa: E402


class TestHilMatrix(unittest.TestCase):
    def test_matrix_is_secret_free_and_covers_profiles_and_stages(self) -> None:
        matrix = hil_acceptance.load_matrix()
        self.assertTrue(matrix["secret_free"])
        self.assertEqual(matrix["stages"], [0, 1, 2, 3, 4, 5])
        self.assertEqual(
            set(matrix["images"]),
            {"rtt", "base", "next"},
        )
        for name in ("rtt", "base", "next"):
            self.assertEqual(matrix["images"][name]["elf"], f"{name}.elf")
            self.assertEqual(matrix["images"][name]["bin"], f"{name}.bin")

        self.assertEqual(matrix["bench"]["host"], "root@10.0.0.150")
        self.assertEqual(matrix["bench"]["artifacts_dir"], "/root/hil-accept/artifacts")
        self.assertEqual(matrix["bench"]["runner"], "/root/hil-accept/run.sh")
        self.assertEqual(matrix["preflight"]["dut_mesh_addr_range"], [1, 63])
        self.assertFalse(matrix["preflight"]["allow_silent_fallback"])

        self.assertEqual(matrix["profiles"]["12mhz"]["stages"], [0, 1, 2, 3, 4, 5])
        self.assertEqual(matrix["profiles"]["16mhz"]["timing_sensitive_stages"], [0, 1, 2, 3, 4])

        serialized = json.dumps(matrix)
        self.assertNotIn(".env", serialized)
        self.assertNotIn("AA:BB", serialized)


class TestHilCli(unittest.TestCase):
    def test_parse_args(self) -> None:
        args = hil_acceptance.parse_args([
            "--profile", "16mhz",
            "--stages", "0,2,5",
            "--toolchain-bin", "/tmp/toolchain",
            "--skip-build",
            "--dry-run",
        ])
        self.assertEqual(args.profile, "16mhz")
        self.assertEqual(args.stages, "0,2,5")
        self.assertEqual(args.toolchain_bin, "/tmp/toolchain")
        self.assertTrue(args.skip_build)
        self.assertTrue(args.dry_run)

    def test_parse_stages(self) -> None:
        self.assertEqual(hil_acceptance.parse_stages("0,2,5", [0, 1]), [0, 2, 5])
        self.assertEqual(hil_acceptance.parse_stages(None, [0, 1, 2]), [0, 1, 2])
        with self.assertRaises(hil_acceptance.HilError):
            hil_acceptance.parse_stages("1,1,6", [0])

    def test_dry_run_does_not_build_or_invoke_subprocesses(self) -> None:
        buf = io.StringIO()
        with (
            mock.patch.object(hil_acceptance.subprocess, "run") as run,
            contextlib.redirect_stdout(buf),
        ):
            code = hil_acceptance.main([
                "--profile", "12mhz",
                "--stages", "0,5",
                "--skip-build",
                "--dry-run",
            ])

        self.assertEqual(code, 0)
        run.assert_not_called()
        out = buf.getvalue()
        self.assertIn("DRY-RUN", out)
        self.assertIn("scp", out)
        self.assertIn("ssh", out)

    def test_skip_build_uses_named_prebuilt_images(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            build_dir = root / "_build" / "hil"
            build_dir.mkdir(parents=True)
            for name in ("rtt", "base", "next"):
                (build_dir / f"{name}.elf").write_bytes(b"ELF")
                (build_dir / f"{name}.bin").write_bytes(b"BIN")

            staged = {
                name: hil_acceptance.find_prebuilt(root, name)
                for name in ("rtt", "base", "next")
            }

        for name in ("rtt", "base", "next"):
            self.assertEqual(staged[name]["elf"].name, f"{name}.elf")
            self.assertEqual(staged[name]["bin"].name, f"{name}.bin")

    def test_full_run_invokes_runner_without_stage(self) -> None:
        bench = {
            "host": "root@10.0.0.150",
            "artifacts_dir": "/root/hil-accept/artifacts",
            "runner": "/root/hil-accept/run.sh",
        }
        with mock.patch.object(hil_acceptance, "run", return_value=None) as run:
            hil_acceptance.invoke_runner(bench, [0, 1, 2, 3, 4, 5], dry_run=False)
        run.assert_called_once_with(
            ["ssh", "root@10.0.0.150", "/root/hil-accept/run.sh"],
            dry_run=False,
        )

    def test_subset_invokes_one_stage_at_a_time(self) -> None:
        bench = {
            "host": "root@10.0.0.150",
            "artifacts_dir": "/root/hil-accept/artifacts",
            "runner": "/root/hil-accept/run.sh",
        }
        with mock.patch.object(hil_acceptance, "run", return_value=None) as run:
            hil_acceptance.invoke_runner(bench, [0, 4], dry_run=False)
        self.assertEqual(run.call_count, 2)
        self.assertIn(
            ["ssh", "root@10.0.0.150", "/root/hil-accept/run.sh", "--stage", "0"],
            [call.args[0] for call in run.call_args_list],
        )


if __name__ == "__main__":
    unittest.main()
