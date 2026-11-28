"""Linked-ELF checks for the single shared OTA/status packet window."""

from __future__ import annotations

import os
import shutil
import subprocess
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
ELF_PATH = REPO_ROOT / "_build" / "lightblemesh"


class TestPacketWindowElf(unittest.TestCase):
    def test_linked_window_is_one_platform_static(self) -> None:
        if not ELF_PATH.is_file():
            self.skipTest("firmware ELF has not been built")

        candidates = []
        if env_nm := os.environ.get("LLVM_NM"):
            candidates.append(Path(env_nm))
        candidates.extend(
            [
                REPO_ROOT.parent / "llvm" / "build-native" / "bin" / "llvm-nm",
                REPO_ROOT.parent / "llvm" / "build" / "bin" / "llvm-nm",
            ]
        )
        if path_nm := shutil.which("llvm-nm"):
            candidates.append(Path(path_nm))

        nm = next((path for path in candidates if path.is_file() or shutil.which(str(path))), None)
        if nm is None:
            self.skipTest("llvm-nm is unavailable")

        proc = subprocess.run(
            [str(nm), "-S", "--demangle", str(ELF_PATH)],
            check=True,
            capture_output=True,
            text=True,
        )

        window_symbols = []
        legacy_symbols = []
        for line in proc.stdout.splitlines():
            fields = line.split()
            if len(fields) < 4:
                continue
            try:
                size = int(fields[1], 16)
            except ValueError:
                continue
            name = " ".join(fields[3:])
            if name.strip().endswith("::PACKET_WINDOW"):
                window_symbols.append((name, size))
            if "BUFF_RESPONSE" in name:
                legacy_symbols.append((name, size))

        self.assertFalse(legacy_symbols, f"legacy response buffer symbols remain: {legacy_symbols}")
        self.assertEqual(
            len(window_symbols),
            1,
            f"expected exactly one PACKET_WINDOW symbol, found {window_symbols}",
        )
        self.assertGreaterEqual(window_symbols[0][1], 768, window_symbols)


if __name__ == "__main__":
    unittest.main()
