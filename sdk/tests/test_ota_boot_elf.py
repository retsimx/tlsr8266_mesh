"""Linked-ELF checks for the RAM-resident `.ram_code` OTA boot copier.

F-7 (#118) moved the boot copier into `tlsr8266-platform::ota::boot` behind
`#[export_name = "tlsr8266_ota_boot_copy"]` and made the whole boot chain
`.ram_code`, because flash code cannot run once the copy loop erases sector 0.

These assertions only run when the firmware has been built.  They reuse the
pure-Python parsers in `sdk/stack_analysis.py` (`objdump`, `read_symbols`,
`read_elf_sections`, `parse_functions`) rather than shelling out again, and the
reachability verdict comes from the *same* `check_ram_reachability` gate the
build enforces — never a re-implementation.
"""

from __future__ import annotations

import os
import shutil
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
ELF_PATH = REPO_ROOT / "_build" / "lightblemesh"

sys.path.insert(0, str(REPO_ROOT / "sdk"))

import stack_analysis  # noqa: E402  (sys.path set up above)

# Every symbol the boot path must own, and all of them must be `.ram_code`.
RAM_RESIDENT_SYMBOLS = (
    "handle_ota_update",
    "tlsr8266_ota_boot_copy",
    "reboot_and_retry_copy",
    "ram_sw_reboot",
)

# The seams F-7 deleted.  Any of them coming back means radio (or the product
# wrapper) grew an OTA fn-pointer hop again, which the stack model cannot see.
RETIRED_OTA_SYMBOLS = (
    "dispatch_ota_write",
    "dispatch_ota_snapshot",
    "dispatch_ota_terminate",
    "dispatch_ota_tick",
    "OtaHooks",
    "OtaRadioOps",
    "tlsr8266_ota_begin_gatt_hook",
    "stage_ota_packet",
)


def _llvm_tool(name: str, env_key: str) -> Path | None:
    """Resolve an LLVM tool: explicit env var, then the repo builds, then PATH."""
    explicit = os.environ.get(env_key)
    if explicit:
        candidate = Path(explicit)
        return candidate if candidate.is_file() else None
    for candidate in (
        REPO_ROOT.parent / "llvm" / "build-native" / "bin" / name,
        REPO_ROOT.parent / "llvm" / "build" / "bin" / name,
    ):
        if candidate.is_file():
            return candidate
    found = shutil.which(name)
    return Path(found) if found else None


class TestOtaBootElf(unittest.TestCase):
    funcs: dict[str, stack_analysis.Func]
    sections: dict[str, tuple[int, int]]
    symbols: dict[str, int]

    @classmethod
    def setUpClass(cls) -> None:
        if not ELF_PATH.is_file():
            raise unittest.SkipTest("firmware ELF has not been built")

        missing = [
            name
            for name, env_key in (("llvm-nm", "LLVM_NM"), ("llvm-objdump", "LLVM_OBJDUMP"))
            if _llvm_tool(name, env_key) is None
        ]
        if missing:
            raise unittest.SkipTest(f"LLVM tools unavailable: {', '.join(missing)}")

        try:
            disassembly = stack_analysis.objdump(str(ELF_PATH))
        except SystemExit as exc:  # the tool exists but cannot decode TC32
            raise unittest.SkipTest(f"llvm-objdump unusable: {exc}") from exc

        cls.funcs = stack_analysis.parse_functions(disassembly)
        if not cls.funcs:
            raise unittest.SkipTest("llvm-objdump produced no functions")

        cls.sections = stack_analysis.read_elf_sections(str(ELF_PATH))
        if not cls.sections:
            raise unittest.SkipTest("ELF section headers could not be read")

        stack_analysis.classify_function_sections(cls.funcs, cls.sections)
        cls.symbols = stack_analysis.read_symbols(str(ELF_PATH))

    def test_boot_copier_chain_is_ram_resident(self) -> None:
        for pattern in RAM_RESIDENT_SYMBOLS:
            matches = sorted(name for name in self.funcs if pattern in name)
            self.assertEqual(len(matches), 1, f"{pattern}: matched {matches}")
            func = self.funcs[matches[0]]
            self.assertTrue(
                func.section.startswith(".ram_code"),
                f"{matches[0]} lives in {func.section or '<unknown>'}, not .ram_code",
            )

    def test_retired_ota_symbols_are_gone(self) -> None:
        present = set(self.funcs) | set(self.symbols)
        for pattern in RETIRED_OTA_SYMBOLS:
            offending = sorted(name for name in present if pattern in name)
            self.assertEqual(
                offending,
                [],
                f"retired OTA symbol {pattern!r} is still linked: {offending}",
            )

    def test_ram_reachability_gate_reports_zero_violations(self) -> None:
        violations = stack_analysis.check_ram_reachability(
            self.funcs, self.sections, self.symbols
        )
        self.assertEqual(violations, [])
        self.assertEqual(
            stack_analysis.validate_ram_reachability(
                self.funcs, self.sections, self.symbols
            ),
            [],
        )


if __name__ == "__main__":
    unittest.main()
