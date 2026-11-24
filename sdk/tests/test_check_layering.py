"""Unit tests for the layering and universal ``unsafe`` gates.

Run from the repository root::

    python3 -m unittest discover -s sdk/tests -v

Every test is hermetic: synthetic trees live in a temporary directory and the
only real trees touched are ``rust/src/`` and the workspace crates (read-only),
by the handful of tests that assert the shipped trees are clean.
"""

from __future__ import annotations

import contextlib
import io
import os
import re
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "sdk"))

import check_layering  # noqa: E402  (sys.path set up above)

SRC_ROOT = REPO_ROOT / "rust" / "src"
CRATE_ROOT = REPO_ROOT / "rust" / "crates" / "mesh-protocol" / "src"
PAC_ROOT = REPO_ROOT / "rust" / "crates" / "tlsr8266-pac" / "src"
HAL_ROOT = REPO_ROOT / "rust" / "crates" / "tlsr8266-hal" / "src"
RADIO_ROOT = REPO_ROOT / "rust" / "crates" / "tlsr8266-radio" / "src"


def _write(path: Path, content: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    return path


def _lines(*lines: str) -> str:
    return "\n".join(lines) + "\n"


class _TempTree(unittest.TestCase):
    """A disposable ``sdk/`` root for one test."""

    def setUp(self) -> None:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name) / "sdk"
        self.root.mkdir()

    def write(self, relative: str, content: str) -> Path:
        return _write(self.root / relative, content)

    def run_gate(self) -> tuple[int, str, str]:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(["--root", str(self.root)])
        return code, out.getvalue(), err.getvalue()

    def assert_clean(self) -> None:
        code, out, err = self.run_gate()
        self.assertEqual(code, 0, out + err)
        self.assertIn("0 violations", out)

    def assert_violations(self, *expected: str) -> str:
        code, out, err = self.run_gate()
        self.assertEqual(code, 1, out + err)
        for text in expected:
            self.assertIn(text, out)
        return out


class TestRealTrees(unittest.TestCase):
    """The refactor is complete: every shipped tree must pass every pass.

    C-10 (#72) deleted ``rust/src/sdk``, so the real-tree cover is now the
    composition root plus all four workspace crates — the exact set the
    no-argument (CI) run audits.  The layering pass has no shipped tree left to
    read; the hermetic ``_TempTree`` / ``_TempSrcTree`` suites are its cover.
    """

    def test_default_src_root_is_the_repo_src(self) -> None:
        self.assertEqual(
            Path(check_layering.DEFAULT_SRC_ROOT).resolve(), SRC_ROOT.resolve()
        )

    def test_default_crate_root_is_the_workspace_crate(self) -> None:
        self.assertEqual(
            Path(check_layering.DEFAULT_CRATE_ROOT).resolve(), CRATE_ROOT.resolve()
        )

    def test_default_crate_roots_cover_every_workspace_crate(self) -> None:
        # C-4 added the HAL, and C-10 (#72) added the extracted radio crate: the
        # no-argument run audits the composition root plus these four, and a new
        # crate must be added here in the same change that creates it.
        roots = [Path(item).resolve() for item in check_layering.DEFAULT_CRATE_ROOTS]
        self.assertEqual(
            roots,
            [
                CRATE_ROOT.resolve(),
                PAC_ROOT.resolve(),
                HAL_ROOT.resolve(),
                RADIO_ROOT.resolve(),
            ],
        )

    def test_real_trees_are_clean(self) -> None:
        # The no-argument run is what CI executes: the composition root and all
        # four workspace crates, zero violations.
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main([])
        self.assertEqual(code, 0, out.getvalue() + err.getvalue())
        self.assertEqual(out.getvalue().count("0 violations"), 5)
        self.assertEqual(err.getvalue(), "")
        # Every tree is named in the summary, relative to the repo, so a root
        # silently dropped from the default list fails here.
        for root in (SRC_ROOT, CRATE_ROOT, PAC_ROOT, HAL_ROOT, RADIO_ROOT):
            self.assertIn(str(root.relative_to(REPO_ROOT)), out.getvalue())

    def test_real_tree_reports_every_pass(self) -> None:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main([])
        self.assertEqual(code, 0, out.getvalue() + err.getvalue())
        self.assertIn("rust src audit OK", out.getvalue())
        self.assertIn("mesh-protocol audit OK", out.getvalue())
        self.assertIn("tlsr8266-pac audit OK", out.getvalue())
        self.assertIn("tlsr8266-hal audit OK", out.getvalue())
        self.assertIn("tlsr8266-radio audit OK", out.getvalue())
        # C-10 (#72) deleted the layering root, so the default run has no
        # layering pass and must not pretend it ran one.
        self.assertNotIn("sdk layering OK", out.getvalue())
        self.assertEqual(len(out.getvalue().splitlines()), 5)

    def test_every_real_tree_passes_the_universal_rules(self) -> None:
        # Direct audit of each tree, independent of the CLI summary: the radio
        # crate is audited exactly like the other three, only unbudgeted.
        floors = {
            SRC_ROOT: 15,
            CRATE_ROOT: 5,
            PAC_ROOT: 2,
            HAL_ROOT: 20,
            RADIO_ROOT: 20,
        }
        for root, floor in floors.items():
            with self.subTest(root=str(root.relative_to(REPO_ROOT))):
                paths = check_layering.rust_files(str(root))
                self.assertGreaterEqual(len(paths), floor, f"{root} is too small")
                violations, _total = check_layering.audit_universal(
                    paths, str(root), enforce_budget=False
                )
                self.assertEqual(violations, [])

    def test_radio_crate_carries_the_injected_config_seam(self) -> None:
        # C-9 (#71) replaced the radio layer's reads of the application's root
        # `config`/`version` modules with an injected `RadioConfig`.  C-10 (#72)
        # moves that seam with the code, and in a separate crate it is the only
        # way to reach board constants at all: `crate::config` is the crate's
        # own module, and the application's `crate::version` cannot resolve.
        config = (RADIO_ROOT / "config.rs").read_text(encoding="utf-8")
        self.assertIn("pub struct RadioConfig", config)
        self.assertIn("pub fn set_radio_config", config)
        for path in check_layering.rust_files(str(RADIO_ROOT)):
            text = Path(path).read_text(encoding="utf-8")
            self.assertIsNone(
                re.search(r"crate::version\b", text),
                f"{path} still names the application's version module",
            )

    def test_production_unsafe_ceiling_equals_the_measured_count(self) -> None:
        # The ratchet may be lowered, never raised: raising it without removing
        # `unsafe` sites leaves the measured count below the ceiling and fails
        # this assertion, and so does lowering it without the removals.
        # C-10 (#72) ratcheted 60 -> 16 when the 44 RF/BLE sites moved into
        # `tlsr8266-radio`, which is audited but not budgeted.
        self.assertEqual(check_layering.MAX_PRODUCTION_UNSAFE, 16)
        violations, total = check_layering.audit_universal(
            check_layering.rust_files(str(SRC_ROOT)), str(SRC_ROOT)
        )
        self.assertEqual(violations, [])
        self.assertEqual(total, check_layering.MAX_PRODUCTION_UNSAFE)

    def test_real_ota_subtree_is_pure(self) -> None:
        # C-6: the shipped OTA subsystem names no layer above it, and the gate
        # now says so mechanically.  The unsafe budget is not the point here, so
        # it is not enforced on this subtree.
        ota_root = SRC_ROOT / "ota"
        paths = check_layering.rust_files(str(ota_root))
        self.assertGreaterEqual(len(paths), 2)  # mod.rs + fsm.rs
        violations, _total = check_layering.audit_universal(
            paths, "rust/src/ota", enforce_budget=False, ota_root=str(ota_root)
        )
        self.assertEqual(violations, [])

    def test_real_light_subtree_is_pure(self) -> None:
        # C-7: the light subsystem owns the lamp and names no layer above it —
        # the mesh status publish goes through the hook `app.rs` registers.
        light_root = SRC_ROOT / "light"
        paths = check_layering.rust_files(str(light_root))
        self.assertGreaterEqual(len(paths), 3)  # mod.rs + transitions.rs + effects.rs
        violations, _total = check_layering.audit_universal(
            paths,
            "rust/src/light",
            enforce_budget=False,
            subsystem_roots={"light": str(light_root)},
        )
        self.assertEqual(violations, [])

    def test_real_factory_reset_subtree_is_pure(self) -> None:
        # C-7: the reset subsystem may not name a single layer above it, and the
        # purity table covers it exactly like `ota`.
        reset_root = SRC_ROOT / "factory_reset"
        paths = check_layering.rust_files(str(reset_root))
        self.assertGreaterEqual(len(paths), 2)  # mod.rs + fsm.rs
        violations, _total = check_layering.audit_universal(
            paths,
            "rust/src/factory_reset",
            enforce_budget=False,
            subsystem_roots={"factory_reset": str(reset_root)},
        )
        self.assertEqual(violations, [])

    def test_real_diagnostics_subtree_is_pure(self) -> None:
        # C-8: `diagnostics` is the leaf of the application layer — the panic
        # sector, the breadcrumbs and the RTT mirror, and nothing above it.
        diagnostics_root = SRC_ROOT / "diagnostics"
        paths = check_layering.rust_files(str(diagnostics_root))
        self.assertGreaterEqual(len(paths), 2)  # mod.rs + rtt.rs
        violations, _total = check_layering.audit_universal(
            paths,
            "rust/src/diagnostics",
            enforce_budget=False,
            subsystem_roots={"diagnostics": str(diagnostics_root)},
        )
        self.assertEqual(violations, [])

    def test_real_logging_module_is_pure(self) -> None:
        # C-8: `logging` is a single module file, so its row is matched by exact
        # path. Its only legal `crate::` edges are `diagnostics::rtt` (the RTT
        # mirror) and itself.
        logging_file = SRC_ROOT / "logging.rs"
        self.assertTrue(logging_file.is_file())
        violations, _total = check_layering.audit_universal(
            [str(logging_file)],
            "rust/src/logging.rs",
            enforce_budget=False,
            subsystem_roots={"logging": str(logging_file)},
        )
        self.assertEqual(violations, [])

    def test_real_uart_subtree_is_pure(self) -> None:
        # C-8: the UART bridge owns framing, ACK/retry and its two tasks, and
        # reaches `crate::logging` / the HAL; it names neither `mesh` nor `sdk`.
        uart_root = SRC_ROOT / "uart"
        paths = check_layering.rust_files(str(uart_root))
        self.assertGreaterEqual(len(paths), 2)  # mod.rs + protocol.rs
        violations, _total = check_layering.audit_universal(
            paths,
            "rust/src/uart",
            enforce_budget=False,
            subsystem_roots={"uart": str(uart_root)},
        )
        self.assertEqual(violations, [])

    def test_real_mesh_subtree_is_pure(self) -> None:
        # C-8: `mesh` owns the radio and the pairing state machine but may not
        # name the UART bridge or the lamp; both arrive through hooks. `sdk` is
        # legal here (the node table and the radio driver live there).
        mesh_root = SRC_ROOT / "mesh"
        paths = check_layering.rust_files(str(mesh_root))
        self.assertGreaterEqual(len(paths), 2)  # mod.rs + fsm.rs
        violations, _total = check_layering.audit_universal(
            paths,
            "rust/src/mesh",
            enforce_budget=False,
            subsystem_roots={"mesh": str(mesh_root)},
        )
        self.assertEqual(violations, [])


class _TempSrcTree(unittest.TestCase):
    """A disposable ``rust/src``, SDK root and mesh-protocol root per test.

    The gate runs every pass here: ``--root`` keeps the layering pass on the
    fake SDK tree, ``--src-root`` points the firmware universal pass at the fake
    source tree and ``--crate-root`` at a fake workspace-crate tree, so no real
    tree is ever read.
    """

    def setUp(self) -> None:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.src_root = Path(tmp.name) / "src"
        self.root = self.src_root / "sdk"
        self.root.mkdir(parents=True)
        self.crate_root = Path(tmp.name) / "mesh-protocol" / "src"
        self.crate_root.mkdir(parents=True)
        _write(self.root / "clean.rs", _lines("pub fn f() {}"))
        _write(self.src_root / "clean.rs", _lines("pub fn f() {}"))
        _write(self.crate_root / "lib.rs", _lines("pub fn f() {}"))

    def write_src(self, relative: str, content: str) -> Path:
        return _write(self.src_root / relative, content)

    def write_crate(self, relative: str, content: str) -> Path:
        return _write(self.crate_root / relative, content)

    def write(self, relative: str, content: str) -> Path:
        return _write(self.root / relative, content)

    def run_gate(self) -> tuple[int, str, str]:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(
                [
                    "--root",
                    str(self.root),
                    "--src-root",
                    str(self.src_root),
                    "--crate-root",
                    str(self.crate_root),
                ]
            )
        return code, out.getvalue(), err.getvalue()

    def assert_clean(self) -> None:
        code, out, err = self.run_gate()
        self.assertEqual(code, 0, out + err)
        self.assertIn("rust src audit OK", out)
        self.assertIn("mesh-protocol audit OK", out)

    def assert_violations(self, *expected: str) -> str:
        code, out, err = self.run_gate()
        self.assertEqual(code, 1, out + err)
        for text in expected:
            self.assertIn(text, out)
        return out


class TestViolationFormat(_TempTree):
    def test_format_is_file_line_rule_violation(self) -> None:
        path = self.write("plain.rs", _lines("fn a() {}", "use crate::app::App;"))
        out = self.assert_violations("upward-dep")
        self.assertEqual(len(out.splitlines()), 1)
        self.assertEqual(out.splitlines()[0], f"{path}:2: upward-dep: crate::app")

    def test_one_line_per_violation_in_file_order(self) -> None:
        self.write("b.rs", _lines("use crate::mesh::is_online;", "uprintln!(\"x\");"))
        self.write("a.rs", _lines("use crate::logging::log;"))
        out = self.assert_violations("upward-dep: crate::logging")
        self.assertEqual(
            [line.split(": ", 1)[1] for line in out.splitlines()],
            ["upward-dep: crate::logging", "upward-dep: crate::mesh", "logging: uprintln!"],
        )
        self.assertLess(out.index("a.rs"), out.index("b.rs"))


class TestGlobalBans(_TempTree):
    def test_upward_app_import_fails(self) -> None:
        self.write("x.rs", _lines("use crate::app::App;"))
        self.assert_violations("1: upward-dep: crate::app")

    def test_upward_common_import_fails(self) -> None:
        self.write("x.rs", _lines("use crate::common::foo;"))
        self.assert_violations("upward-dep: crate::common")

    def test_uprintln_macro_fails(self) -> None:
        self.write("x.rs", _lines("pub fn f() {", '    uprintln!("x");', "}"))
        self.assert_violations('2: logging: uprintln!')

    def test_uprint_macro_fails(self) -> None:
        self.write("x.rs", _lines('pub fn f() { uprint!("x"); }'))
        self.assert_violations("logging: uprint!")

    def test_grouped_crate_import_fails(self) -> None:
        self.write("x.rs", _lines("use crate::{app, BIT};"))
        self.assert_violations("1: upward-dep: crate::app")

    def test_every_banned_module_is_rejected(self) -> None:
        # `UPWARD_MODULES` (reported as `upward-dep`) plus the two crate-root
        # constant modules C-9 banned (reported as `sdk-root-dep`).  Both sets
        # are exercised in one loop because the assertion is on the reported
        # `crate::<module>`, which is what a reviewer reads either way.
        banned = [
            "app",
            "common",
            "config",
            "factory_reset",
            "light",
            "main_light",
            "mesh",
            "light_manager",
            "uart",
            "uart_manager",
            "ota",
            "diagnostics",
            "logging",
            "irq",
            "embassy",
            "version",
        ]
        for module in banned:
            with self.subTest(module=module):
                _write(self.root / f"{module}.rs", _lines(f"use crate::{module}::thing;"))
        out = self.assert_violations(*[f"crate::{m}" for m in banned])
        self.assertEqual(len(out.splitlines()), len(banned))

    def test_sdk_and_root_macro_imports_are_legal(self) -> None:
        # `crate::sdk` siblings and the root macros stay legal; only the
        # application layer and the root constant modules are banned.
        self.write(
            "x.rs",
            _lines(
                "use crate::sdk::common::sync::CriticalSectionMutex;",
                "use crate::BIT;",
            ),
        )
        self.assert_clean()

    def test_config_root_import_is_rejected(self) -> None:
        # C-9 (#71): `crate::config` was the legitimate root constant module and
        # is now the one upward dependency that keeps `sdk/` from being
        # extracted, so the gate rejects it inside the SDK tree.
        path = self.write(
            "x.rs",
            _lines("pub fn f() -> u16 {", "    crate::config::VENDOR_ID", "}"),
        )
        out = self.assert_violations("sdk-root-dep: crate::config")
        self.assertEqual(len(out.splitlines()), 1)
        self.assertEqual(
            out.splitlines()[0], f"{path}:2: sdk-root-dep: crate::config"
        )

    def test_version_root_import_is_rejected(self) -> None:
        # `crate::version::BUILD_VERSION` is the other half of the same ban: the
        # extracted radio crate receives the version word through `RadioConfig`.
        path = self.write("x.rs", _lines("use crate::version::BUILD_VERSION;"))
        out = self.assert_violations("sdk-root-dep: crate::version")
        self.assertEqual(len(out.splitlines()), 1)
        self.assertEqual(
            out.splitlines()[0], f"{path}:1: sdk-root-dep: crate::version"
        )

    def test_root_import_inside_cfg_test_is_exempt(self) -> None:
        # The rule reads `production_code()`, so a test-only import is not
        # flagged here.  The issue's criterion is a literal grep, which is
        # stricter and covers tests too: both are enforced, at different scopes.
        self.write(
            "x.rs",
            _lines(
                "pub fn f() {}",
                "",
                "#[cfg(test)]",
                "mod tests {",
                "    use crate::config::VENDOR_ID;",
                "    use crate::version::BUILD_VERSION;",
                "}",
            ),
        )
        self.assert_clean()

    def test_identifiers_that_merely_contain_a_banned_name_are_legal(self) -> None:
        self.write(
            "x.rs",
            _lines(
                "use crate::sdk::app_att_light::APP_ATT_LIGHT;",
                "use crate::mesh_protocol::ping;",
                "pub fn f(p: &Packet) {",
                "    let _ = p.ll_app();",
                "    app_att_light::run();",
                "    crate::mesh_protocol::ping();",
                "}",
            ),
        )
        self.assert_clean()


class TestSdkRootBanScope(_TempSrcTree):
    """`sdk-root-dep` is scoped to the extracted subtree, not the whole tree.

    The rule lives in the layering pass, which only ever reads ``--root``; the
    shipped subtree it was written for (``rust/src/sdk``) was deleted by C-10
    (#72), so these fixtures are its only caller.  `crate::config` is the
    legitimate root constant module for the application, so a non-SDK file that
    names it must stay clean — otherwise the ban would be an app-wide refactor
    instead of a seam.
    """

    def test_crate_config_is_legal_outside_the_sdk_tree(self) -> None:
        self.write_src("app.rs", _lines("use crate::config::VENDOR_ID;"))
        self.write("ble_app/clean.rs", _lines("pub fn f() {}"))
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(
                [
                    "--root",
                    str(self.root),
                    "--src-root",
                    str(self.src_root),
                    "--crate-root",
                    str(self.crate_root),
                ]
            )
        self.assertEqual(code, 0, out.getvalue() + err.getvalue())
        self.assertNotIn("sdk-root-dep", out.getvalue())

    def test_the_same_import_inside_the_sdk_tree_is_rejected(self) -> None:
        # Same file content, SDK path: the only difference is the subtree.
        path = self.write_src("sdk/ble_app/config_user.rs", _lines("use crate::config::VENDOR_ID;"))
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(
                [
                    "--root",
                    str(self.root),
                    "--src-root",
                    str(self.src_root),
                    "--crate-root",
                    str(self.crate_root),
                ]
            )
        self.assertEqual(code, 1, out.getvalue() + err.getvalue())
        self.assertIn(f"{path}:1: sdk-root-dep: crate::config", out.getvalue())


class TestLayerRules(_TempTree):
    def test_common_may_not_import_mcu(self) -> None:
        self.write("common/clock_user.rs", _lines("use crate::sdk::mcu::clock::clock_time;"))
        self.assert_violations(
            "1: layer-common: crate::sdk::mcu::clock::clock_time"
        )

    def test_mcu_may_not_import_a_sibling_module(self) -> None:
        # `mcu` may only name `common`/`mcu` under `crate::sdk`; a sibling
        # module (the old `drivers/` layer, now `rf_drv` at the SDK root) is an
        # upward jump out of the layer.
        self.write("mcu/clock_user.rs", _lines("use crate::sdk::rf_drv::init;"))
        self.assert_violations("1: layer-mcu: crate::sdk::rf_drv::init")

    def test_layer_bottom_up_imports_are_legal(self) -> None:
        self.write(
            "common/sync_user.rs",
            _lines("use crate::sdk::common::sync::CriticalSectionMutex;"),
        )
        self.write(
            "mcu/clock_user.rs",
            _lines(
                "use crate::sdk::common::sync::CriticalSectionMutex;",
                "use crate::sdk::mcu::register::read_reg8;",
            ),
        )
        self.assert_clean()

    def test_root_macros_are_legal_imports(self) -> None:
        self.write("common/bits.rs", _lines("use crate::{BIT, MASK_VAL};"))
        self.write("mcu/bits.rs", _lines("use crate::{regrw_idx, BIT_LOW_BIT};"))
        self.write("mcu/more_bits.rs", _lines("use crate::{BM_CLR, BM_SET, CLR_FLD};"))
        self.assert_clean()

    def test_new_external_crate_in_a_layer_is_rejected(self) -> None:
        self.write("common/dep.rs", _lines("use embassy_sync::signal::Signal;"))
        self.assert_violations("1: layer-common: external crate `embassy_sync`")

    def test_layer_rules_do_not_apply_outside_the_layer_directories(self) -> None:
        # `ble_app` never was a layer directory, so a sibling `crate::sdk` import
        # that would fail from `mcu`/`common` is clean here.
        self.write("ble_app/sibling.rs", _lines("use crate::sdk::rf_drv::init;"))
        self.assert_clean()

    def test_multi_line_use_reports_the_use_line(self) -> None:
        self.write(
            "common/user.rs",
            _lines(
                "use crate::sdk::mcu::register::{",
                "    read_reg8, write_reg8,",
                "};",
            ),
        )
        self.assert_violations("1: layer-common: crate::sdk::mcu::register::read_reg8")

    def test_tlsr8266_hal_is_legal_in_mcu(self) -> None:
        # C-4: the extracted HAL crate is what the remaining `mcu` modules
        # consume, so naming it is not an inversion there.
        self.write("mcu/hal_user.rs", _lines("use tlsr8266_hal::mcu::clock::clock_time;"))
        self.assert_clean()

    def test_tlsr8266_hal_is_rejected_in_common(self) -> None:
        # `sdk/common` no longer exists in the firmware tree, and the allowance
        # was not widened: only `mcu` may name the HAL.
        self.write(
            "common/hal_user.rs",
            _lines("use tlsr8266_hal::common::sync::SimplifyLS;"),
        )
        self.assert_violations("1: layer-common: external crate `tlsr8266_hal`")


class TestPmLeafRule(_TempTree):
    """`sdk/pm.rs` sits at the SDK root but is a leaf: it follows `mcu` rules.

    C-4 moves the module into `tlsr8266-hal`, so it may not reach sideways into
    `sdk/ble_app` or `sdk/light`, which a root-level file would otherwise be
    free to do.
    """

    def test_pm_importing_ble_app_fails(self) -> None:
        path = self.write(
            "pm.rs",
            _lines("use crate::sdk::ble_app::callbacks::dispatch_ota_write;"),
        )
        out = self.assert_violations(
            "1: layer-mcu: "
            "crate::sdk::ble_app::callbacks::dispatch_ota_write"
        )
        self.assertEqual(len(out.splitlines()), 1)
        self.assertEqual(
            out.splitlines()[0],
            f"{path}:1: layer-mcu: "
            "crate::sdk::ble_app::callbacks::dispatch_ota_write",
        )

    def test_pm_importing_light_fails(self) -> None:
        path = self.write("pm.rs", _lines("use crate::sdk::light::RecoverStatus;"))
        out = self.assert_violations("1: layer-mcu: crate::sdk::light::RecoverStatus")
        self.assertEqual(len(out.splitlines()), 1)
        self.assertEqual(
            out.splitlines()[0],
            f"{path}:1: layer-mcu: crate::sdk::light::RecoverStatus",
        )

    def test_pm_importing_common_mcu_and_pac_is_clean(self) -> None:
        self.write(
            "pm.rs",
            _lines(
                "use core::sync::atomic::{AtomicBool, Ordering};",
                "use crate::sdk::common::sync::SimplifyLS;",
                "use crate::sdk::mcu::clock::clock_time;",
                "use crate::BIT;",
                "use critical_section;",
                "use tlsr8266_pac::write_reg_pwdn_ctrl;",
            ),
        )
        self.assert_clean()

    def test_leaf_rule_is_scoped_to_pm_rs(self) -> None:
        # Control: another root-level `sdk/*.rs` file keeps its current
        # behaviour (no layer), so the same import is legal there.  The branch
        # is an exact file match, not "every SDK-root file is a leaf".
        self.write("factory_reset.rs", _lines("use crate::sdk::light::RecoverStatus;"))
        self.assert_clean()
        self.assertEqual(
            check_layering._layer_of(str(self.root / "pm.rs"), str(self.root)), "mcu"
        )
        self.assertIsNone(
            check_layering._layer_of(str(self.root / "factory_reset.rs"), str(self.root))
        )


class TestMeshProtocolExternalCrate(_TempTree):
    """`mesh_protocol` is a pure workspace crate, legal in every SDK layer."""

    def test_mesh_protocol_imports_are_legal_in_every_layer(self) -> None:
        self.write("common/crc_user.rs", _lines("use mesh_protocol::crc::crc16;"))
        self.write(
            "mcu/mesh_user.rs", _lines("use mesh_protocol::packet_types::Packet;")
        )
        self.assert_clean()

    def test_mesh_protocol_multiline_grouped_import_is_legal(self) -> None:
        self.write(
            "mcu/mesh_user.rs",
            _lines(
                "use mesh_protocol::{",
                "    packet_types::{Packet, PacketL2capHead},",
                "    uart::UartData,",
                "};",
            ),
        )
        self.assert_clean()

    def test_another_external_crate_is_still_rejected_in_mcu(self) -> None:
        # The allowance names one crate; it does not open the whole namespace.
        self.write("mcu/x.rs", _lines("use serde::Serialize;"))
        self.assert_violations("1: layer-mcu: external crate `serde`")

    def test_mesh_protocol_does_not_unlock_app_layer_paths(self) -> None:
        self.write("mcu/x.rs", _lines("use crate::app::App;"))
        self.assert_violations("upward-dep: crate::app")


class TestTestCodeStripping(_TempTree):
    """The parser must strip only `#[cfg(test)]` and never truncate a file."""

    def test_inline_cfg_test_item_above_a_violation_is_still_caught(self) -> None:
        self.write(
            "trap.rs",
            _lines(
                "use crate::state::*;",          # 1
                "",                               # 2
                "#[cfg(test)]",                   # 3
                "pub static mut SCRATCH: u32 = 0;",  # 4
                "",                               # 5
                "pub fn production() {}",         # 6
                "",                               # 7
                "use crate::app::App;",           # 8
            ),
        )
        out = self.assert_violations("8: upward-dep: crate::app")

    def test_inline_cfg_test_brace_item_above_a_violation_is_still_caught(self) -> None:
        self.write(
            "trap.rs",
            _lines(
                "#[cfg(test)]",
                "fn helper() {",
                "    let x = 0;",
                "}",
                "",
                "pub fn production() { let _ = crate::uart_manager::x; }",
            ),
        )
        self.assert_violations("6: upward-dep: crate::uart_manager")

    def test_cfg_test_module_is_exempt(self) -> None:
        self.write(
            "clean.rs",
            _lines(
                "pub fn production() {}",
                "",
                "#[cfg(test)]",
                "mod tests {",
                "    use crate::app::App;",
                '    const S: &str = "uprintln!(x)";',
                "}",
            ),
        )
        self.assert_clean()

    def test_cfg_test_module_with_unbalanced_brace_in_comment_and_string(self) -> None:
        self.write(
            "clean.rs",
            _lines(
                "pub fn production() {}",
                "",
                "#[cfg(test)]",
                "mod tests {",
                "    // a brace that is not code: {",
                '    const S: &str = "}";',
                "    use crate::app::App;",
                "}",
                "",
                "pub fn still_production() { let _ = crate::mesh::is_online(0); }",
            ),
        )
        out = self.assert_violations("10: upward-dep: crate::mesh")
        self.assertEqual(len(out.splitlines()), 1)
        self.assertNotIn("crate::app", out)

    def test_production_code_preserves_line_numbers_and_later_code(self) -> None:
        text = _lines(
            "#[cfg(test)]",
            "mod tests {",
            "    use crate::app::App;",
            "}",
            "",
            "pub fn later() { let _ = crate::ota::x; }",
        )
        code = check_layering.production_code(text)
        self.assertEqual(len(code.splitlines()), len(text.splitlines()))
        self.assertNotIn("crate::app", code)
        self.assertIn("crate::ota", code)

    def test_cfg_not_test_and_cfg_attr_are_production_code(self) -> None:
        self.write(
            "trap.rs",
            _lines(
                "#[cfg(not(test))]",
                "pub fn real() {}",
                "",
                "#[cfg_attr(test, mry::mry)]",
                "pub fn mocked() {}",
                "",
                "pub fn bad() { let _ = crate::light_manager::x; }",
            ),
        )
        self.assert_violations("7: upward-dep: crate::light_manager")

    def test_cfg_test_on_a_struct_field_does_not_mask_the_next_item(self) -> None:
        # H-1: the field attribute used to run to the next brace block, hiding
        # the production item that followed it.
        self.write(
            "trap.rs",
            _lines(
                "pub struct S {",
                "    #[cfg(test)]",
                "    locked: u8,",
                "}",
                "",
                "pub fn production() { let _ = crate::app::App; }",
            ),
        )
        self.assert_violations("6: upward-dep: crate::app")

    def test_cfg_test_on_a_statement_does_not_mask_the_rest_of_the_body(self) -> None:
        # Same root cause, statement shape (`uart_manager::uart_send_async`).
        self.write(
            "trap.rs",
            _lines(
                "pub fn f() {",
                "    #[cfg(test)]",
                "    let scratch = 0;",
                "    let _ = crate::ota::plan();",
                "    loop {",
                "        break;",
                "    }",
                "}",
            ),
        )
        self.assert_violations("4: upward-dep: crate::ota")


class TestTestStrippingExtent(_TempTree):
    """The stripper's extent: an item is removed whole, a field/statement unit
    only up to its own `,`/`;` (the H-1 root cause)."""

    def _masked(self, *lines: str) -> str:
        return check_layering.production_code(_lines(*lines))

    def test_item_extent_removes_the_whole_item(self) -> None:
        code = self._masked(
            "#[cfg(test)]",
            "mod tests {",
            "    pub static mut SCRATCH: u8 = 0;",
            "}",
            "pub fn production() {}",
        )
        self.assertNotIn("SCRATCH", code)
        self.assertIn("pub fn production", code)

    def test_field_extent_stops_at_the_comma_and_keeps_array_types_whole(self) -> None:
        code = self._masked(
            "pub struct S {",
            "    #[cfg(test)]",
            "    scratch: [u8; 4],",
            "    data: u8,",
            "}",
        )
        self.assertNotIn("scratch", code)
        self.assertIn("data: u8", code)

    def test_statement_extent_stops_at_the_semicolon(self) -> None:
        code = self._masked(
            "pub fn f() {",
            "    #[cfg(test)]",
            "    let scratch = 0;",
            "    let kept = 1;",
            "}",
        )
        self.assertNotIn("scratch", code)
        self.assertIn("let kept = 1;", code)

    def test_block_bodied_annotation_stops_at_its_own_brace_group(self) -> None:
        code = self._masked(
            "pub fn f() {",
            "    #[cfg(test)]",
            "    unsafe { probe() }",
            "    unsafe { production() }",
            "}",
        )
        self.assertNotIn("probe", code)
        self.assertIn("unsafe { production() }", code)


class TestCommentAndStringPolicy(_TempTree):
    """Comments and strings are prose, not dependencies: they never count."""

    def test_banned_path_in_line_comment_is_ignored(self) -> None:
        self.write("x.rs", _lines("// historically: use crate::app::App;"))
        self.assert_clean()

    def test_banned_path_in_block_comment_is_ignored(self) -> None:
        self.write("x.rs", _lines("/* crate::mesh::is_online */", "pub fn f() {}"))
        self.assert_clean()

    def test_banned_path_in_string_literal_is_ignored(self) -> None:
        self.write("x.rs", _lines('pub const S: &str = "crate::app";'))
        self.assert_clean()

    def test_unbalanced_brace_in_comment_does_not_break_the_parser(self) -> None:
        self.write(
            "x.rs",
            _lines(
                "// if (flag) {",
                "pub fn f() { let _ = crate::app::App::default(); }",
            ),
        )
        self.assert_violations("2: upward-dep: crate::app")


class TestHistoricalTrapFiles(_TempTree):
    """The historical trap *shapes*, inlined as synthetic fixtures.

    These used to read and inject into copies of
    ``rust/src/sdk/ble_app/*.rs`` — a tree C-10 (#72) deleted.  Each test now
    writes the same shape from scratch: a violation injected after the exact
    construct that once defeated the mask, in a file whose directory depth and
    layer are the same as the original's.
    """

    def test_cfg_test_static_mut_above_production_does_not_truncate(self) -> None:
        # Shape of the deleted `ble_app/ble_ll_attribute.rs`: an inline
        # `#[cfg(test)] pub static mut` sitting between production items.  The
        # stripper used to run from that attribute to the next brace block and
        # hide every production line that followed it.
        path = self.write(
            "ble_app/ble_ll_attribute.rs",
            _lines(
                "use tlsr8266_pac::read_reg_system_tick;",       # 1
                "",                                               # 2
                "#[cfg(test)]",                                   # 3
                "pub static mut __RAM_START_ADDR: u32 = 0;",      # 4
                "",                                               # 5
                "pub fn production() {}",                         # 6
                "",                                               # 7
                "use crate::app::App;",                           # 8
            ),
        )
        out = self.assert_violations(f"{path}:8: upward-dep: crate::app")
        self.assertEqual(len(out.splitlines()), 1)

    def test_fn_local_import_is_production(self) -> None:
        # Shape of the deleted `ble_app/light_ll/packet_processing.rs`: the
        # violation is a `use` *inside* a `#[cfg_attr(test, mry::mry)]`
        # function, so a stripper that treated the attribute as test code lost
        # the whole body.
        path = self.write(
            "ble_app/light_ll/packet_processing.rs",
            _lines(
                "#[cfg_attr(test, mry::mry)]",                        # 1
                "pub fn rf_link_add_tx_packet(packet: &Packet) -> Result<(), ()> {",  # 2
                "    use crate::mesh::MESH_NODE_ST_VAL_LEN;",         # 3
                "    let _ = packet;",                                # 4
                "    Ok(())",                                         # 5
                "}",                                                  # 6
            ),
        )
        self.assert_violations(f"{path}:3: upward-dep: crate::mesh")

    def test_irq_fn_local_import_is_production(self) -> None:
        # Shape of the deleted `ble_app/irq/connection.rs`: a module-level
        # `use` inside a fn body, two directories below the root.
        path = self.write(
            "ble_app/irq/connection.rs",
            _lines(
                "#[cfg_attr(test, mry::mry)]",                        # 1
                "fn process_mesh_operations() {",                     # 2
                "    use crate::uart_manager::UartMsg;",              # 3
                "    let _ = core::mem::size_of::<UartMsg>();",       # 4
                "}",                                                  # 5
            ),
        )
        self.assert_violations(f"{path}:3: upward-dep: crate::uart_manager")

    def test_config_root_import_is_rejected_next_to_a_real_pac_import(self) -> None:
        # C-9 (#71): the ten real import sites were spread across eight files of
        # the extracted subtree, so the trap sits in a file that legitimately
        # names the PAC and keeps its own `#[cfg(test)]` copy of the same path.
        path = self.write(
            "ble_app/ble_ll_attribute.rs",
            _lines(
                "use tlsr8266_pac::read_reg_system_tick;",            # 1
                "use crate::config::VENDOR_ID;",                      # 2
                "",                                                   # 3
                "#[cfg(test)]",                                       # 4
                "mod tests {",                                        # 5
                "    use crate::config::VENDOR_ID;",                  # 6
                "}",                                                  # 7
            ),
        )
        out = self.assert_violations(f"{path}:2: sdk-root-dep: crate::config")
        # The test-only copy is exempt: the rule reads production code only.
        self.assertEqual(len(out.splitlines()), 1)


class TestPathBypasses(_TempTree):
    """Relative and aliased paths must not be able to dodge the crate:: rules."""

    def test_super_chain_from_the_sdk_root_is_rejected(self) -> None:
        self.write("x.rs", _lines("use super::super::app::app;"))
        out = self.assert_violations("1: super-escape")
        self.assertIn("chain of 2, depth 0", out)

    def test_super_chain_that_leaves_the_sdk_from_a_subdirectory_is_rejected(self) -> None:
        self.write("ble_app/light_ll/x.rs", _lines("use super::super::super::app::app;"))
        self.assert_violations("1: super-escape")

    def test_super_chain_that_stays_inside_the_sdk_is_legal(self) -> None:
        self.write(
            "ble_app/light_ll/status.rs",
            _lines(
                "use super::mesh_management::mesh_send_online_status;",
                "pub fn f() { super::super::queues::enqueue_mesh_rx(&PKT); }",
            ),
        )
        self.write("ble_app/irq/timing.rs", _lines("use super::mesh::handle_mesh_listening_state;"))
        # A brace group composes with its prefix, but sibling chains do not
        # stack: two hops, not four, so this stays legal at depth 2.
        self.write(
            "ble_app/light_ll/grouped.rs",
            _lines("use super::{super::queues, super::mesh_management};"),
        )
        self.assert_clean()

    def test_grouped_super_chain_that_escapes_is_rejected(self) -> None:
        self.write("x.rs", _lines("use super::{super::app::app};"))
        self.assert_violations("super-escape: super::2 past the audited root (chain of 2, depth 0)")

    def test_crate_alias_is_rejected(self) -> None:
        self.write(
            "x.rs",
            _lines("use crate as krate;", "pub fn f() { let _ = krate::app::app(); }"),
        )
        self.assert_violations("1: crate-alias: crate as krate")

    def test_grouped_crate_alias_is_rejected(self) -> None:
        self.write("x.rs", _lines("use crate::{self as krate};"))
        self.assert_violations("1: crate-alias: crate::{self as krate}")

    def test_extern_crate_self_alias_is_rejected(self) -> None:
        # `extern crate self as krate;` aliases the crate root like
        # `use crate as krate;` but is not a `use` statement.  The declaration
        # alone is the violation: it is what lets `krate::app::..` dodge the
        # path rules, whether or not this file happens to use it.
        self.write("x.rs", _lines("extern crate self as krate;"))
        out = self.assert_violations("1: crate-alias: extern crate self as krate")
        self.assertEqual(len(out.splitlines()), 1)

    def test_extern_crate_self_alias_with_qualified_use_is_rejected(self) -> None:
        self.write(
            "x.rs",
            _lines(
                "extern crate self as krate;",
                "pub fn f() { krate::app::app(); }",
            ),
        )
        self.assert_violations("1: crate-alias: extern crate self as krate")

    def test_plain_extern_crate_declarations_are_legal(self) -> None:
        # Control for the matcher above: the general `extern crate <name>;`
        # form is not a crate-root alias.  `extern crate core;` compiles in a
        # `#![no_std]` crate with no real dependency (verified with rustc); it
        # can only warn as an unused extern crate.
        self.write(
            "x.rs",
            _lines(
                "extern crate core;",
                "pub fn f() { core::mem::size_of::<u32>(); }",
            ),
        )
        self.assert_clean()
        for statement in ("extern crate core;", "extern crate std;", "extern crate alloc;"):
            self.assertIsNone(
                check_layering._EXTERN_CRATE_SELF_RE.search(statement), statement
            )


class TestCli(_TempTree):
    def test_missing_root_exits_2(self) -> None:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(["--root", str(self.root / "absent")])
        self.assertEqual(code, 2)
        self.assertIn("not a directory", err.getvalue())

    def test_empty_root_exits_2(self) -> None:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(["--root", str(self.root)])
        self.assertEqual(code, 2)
        self.assertIn("no .rs files", err.getvalue())

    def test_root_is_resolved_from_any_working_directory(self) -> None:
        self.write("x.rs", _lines("use crate::app::App;"))
        elsewhere = self.root.parent / "elsewhere"
        elsewhere.mkdir()
        out, err = io.StringIO(), io.StringIO()
        with contextlib.chdir(elsewhere):
            with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                code = check_layering.main(["--root", "../sdk"])
        self.assertEqual(code, 1, out.getvalue() + err.getvalue())
        self.assertTrue(out.getvalue().startswith(str(self.root / "x.rs") + ":1: "))
        self.assertTrue(re.search(r":1: upward-dep: crate::app$", out.getvalue()))

    def test_stray_non_rust_files_are_ignored(self) -> None:
        self.write("notes.txt", "use crate::app::App;\n")
        self.write("clean.rs", _lines("use crate::sdk::light::RecoverStatus;"))
        self.assert_clean()

    def test_explicit_root_skips_the_universal_pass(self) -> None:
        # Hermeticity: `--root` alone must never read the real `rust/src`.
        self.write("x.rs", _lines("pub fn f() {}"))
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(["--root", str(self.root)])
        self.assertEqual(code, 0, out.getvalue() + err.getvalue())
        self.assertNotIn("rust src audit", out.getvalue())
        self.assertEqual(len(out.getvalue().splitlines()), 1)


class TestOtaPurityRule(_TempSrcTree):
    """`rust/src/ota/**` production code may not name the layers above it.

    `ota` owns every OTA static and routine, so the seam is the only legal
    direction: the subsystem may reach `crate::config`, `mesh_protocol`,
    `tlsr8266_hal` and its own `fsm`, and nothing above it.  The rule reads
    production code only, because `ota`'s own tests legitimately use
    `tlsr8266_pac::fake_mmio::FakeMmio`.
    """

    def test_sdk_import_is_rejected(self) -> None:
        path = self.write_src(
            "ota/mod.rs",
            _lines("use crate::sdk::ble_app::callbacks::set_ota_hooks;"),
        )
        out = self.assert_violations("1: ota-upward-dep: crate::sdk")
        self.assertEqual(out.splitlines()[0], f"{path}:1: ota-upward-dep: crate::sdk")

    def test_qualified_app_path_is_rejected(self) -> None:
        # Not an import: the rule reads every `crate::` reference, so a
        # fully-qualified call cannot dodge it.
        self.write_src(
            "ota/mod.rs",
            _lines("pub fn f() { crate::light_manager::set_brightness(1); }"),
        )
        self.assert_violations("1: ota-upward-dep: crate::light_manager")

    def test_tlsr8266_pac_import_is_rejected(self) -> None:
        self.write_src("ota/mod.rs", _lines("use tlsr8266_pac::fake_mmio::FakeMmio;"))
        self.assert_violations("1: ota-forbidden-crate: tlsr8266_pac")

    def test_every_forbidden_module_is_rejected(self) -> None:
        # The set is the whole application layer, not just `sdk`: `ota` must not
        # name any of them, and `crate::state` is banned twice over.
        banned = sorted(check_layering.OTA_FORBIDDEN_MODULES - {"state"})
        for module in banned:
            with self.subTest(module=module):
                _write(
                    self.src_root / "ota" / f"{module}.rs",
                    _lines(f"use crate::{module}::thing;"),
                )
        out = self.assert_violations(*[f"crate::{m}" for m in banned])
        self.assertEqual(len(out.splitlines()), len(banned))

    def test_downward_and_internal_imports_are_accepted(self) -> None:
        self.write_src(
            "ota/mod.rs",
            _lines(
                "use core::sync::atomic::Ordering;",
                "use critical_section;",
                "use mesh_protocol::mesh_types::OtaState;",
                "use tlsr8266_hal::flash::read_flash;",
                "use crate::config::VENDOR_ID;",
                "use crate::ota::fsm::OtaChunkDecision;",
                "use super::fsm::evaluate_chunk;",
            ),
        )
        self.assert_clean()

    def test_forbidden_import_inside_cfg_test_is_accepted(self) -> None:
        # A raw text grep would fail this file: the fake MMIO lives in a test
        # module, and a test may legitimately stand the layers in.
        self.write_src(
            "ota/mod.rs",
            _lines(
                "pub fn production() {}",
                "",
                "#[cfg(test)]",
                "mod tests {",
                "    use crate::sdk::ble_app::callbacks::set_ota_hooks;",
                "    use tlsr8266_pac::fake_mmio::FakeMmio;",
                "}",
            ),
        )
        self.assert_clean()

    def test_rule_is_scoped_to_the_ota_directory(self) -> None:
        # Control: the same import one directory up is production code but not
        # `ota` code, so the purity rule leaves it to the layering rules.
        self.write_src(
            "sdk/ble_app/callbacks.rs",
            _lines("use tlsr8266_pac::write_reg8;"),
        )
        self.assert_clean()


class TestFactoryResetPurityRule(_TempSrcTree):
    """`rust/src/factory_reset/**` production code may not name the layers above.

    C-7 turned C-6's single `ota-purity` rule into the `SUBSYSTEM_PURITY` table;
    this class pins the new `factory_reset` row.  The reset subsystem owns boot
    detection and the configuration erase, and reaches downward only:
    `tlsr8266_hal`, `crate::config`, `core`, `critical_section` and its own
    `fsm`.  Everything above it — including its sibling `light` — is forbidden.
    """

    def test_sdk_import_is_rejected(self) -> None:
        path = self.write_src(
            "factory_reset/mod.rs",
            _lines("use crate::sdk::ble_app::ble_ll_pair::PAIR_CONFIG_MESH_LTK;"),
        )
        out = self.assert_violations("1: factory_reset-upward-dep: crate::sdk")
        self.assertEqual(
            out.splitlines()[0], f"{path}:1: factory_reset-upward-dep: crate::sdk"
        )

    def test_qualified_light_path_is_rejected(self) -> None:
        # `light` is a sibling subsystem, not a dependency: a fully-qualified
        # call must not dodge the rule.
        self.write_src(
            "factory_reset/mod.rs",
            _lines("pub fn f() { crate::light::device_status_update(); }"),
        )
        self.assert_violations("1: factory_reset-upward-dep: crate::light")

    def test_tlsr8266_pac_import_is_rejected(self) -> None:
        self.write_src(
            "factory_reset/mod.rs", _lines("use tlsr8266_pac::fake_mmio::FakeMmio;")
        )
        self.assert_violations("1: factory_reset-forbidden-crate: tlsr8266_pac")

    def test_every_forbidden_module_is_rejected(self) -> None:
        banned = sorted(check_layering.FACTORY_RESET_FORBIDDEN_MODULES - {"state"})
        for module in banned:
            with self.subTest(module=module):
                _write(
                    self.src_root / "factory_reset" / f"{module}.rs",
                    _lines(f"use crate::{module}::thing;"),
                )
        out = self.assert_violations(*[f"crate::{m}" for m in banned])
        self.assertEqual(len(out.splitlines()), len(banned))

    def test_downward_and_internal_imports_are_accepted(self) -> None:
        self.write_src(
            "factory_reset/mod.rs",
            _lines(
                "use core::sync::atomic::AtomicU32;",
                "use critical_section;",
                "use tlsr8266_hal::drivers::flash::flash_read_page;",
                "use crate::config::FLASH_ADR_RESET_CNT;",
                "mod fsm;",
                "use fsm::ResetStep;",
                "use super::fsm::boot_step_value;",
            ),
        )
        self.assert_clean()

    def test_forbidden_import_inside_cfg_test_is_accepted(self) -> None:
        # The subsystem's own tests seed `FakeFlash`, which lives in the HAL, but
        # they may also stand the PAC in: only production code is audited.
        self.write_src(
            "factory_reset/mod.rs",
            _lines(
                "pub fn production() {}",
                "",
                "#[cfg(test)]",
                "mod tests {",
                "    use crate::sdk::ble_app::ble_ll_pair::PAIR_CONFIG_MESH_LTK;",
                "    use tlsr8266_pac::fake_mmio::FakeMmio;",
                "}",
            ),
        )
        self.assert_clean()

    def test_rule_is_scoped_to_the_factory_reset_directory(self) -> None:
        # Control: the same import one directory up is production code but not
        # `factory_reset` code, so the purity rule leaves it to the layering
        # rules (and `crate::sdk` there is legal).
        self.write_src(
            "sdk/ble_app/callbacks.rs",
            _lines("use crate::sdk::ble_app::light_ll::packet_processing::*;"),
        )
        self.assert_clean()


class TestLightPurityRule(_TempSrcTree):
    """`rust/src/light/**` production code may not name the layers above it.

    The light subsystem owns the lamp state, PWM, persistence, LED indication and
    the light-domain wire protocol, and reaches downward only: `tlsr8266_hal`,
    `mesh_protocol`, `crate::config` and the lamp's own maths/queue crates.
    Everything above it is forbidden, `factory_reset` included: the two
    subsystems are siblings, not dependencies.  Tests are exempt, because a
    subsystem's own tests stand the HAL and PAC fakes in for hardware.
    """

    def test_sdk_import_is_rejected(self) -> None:
        path = self.write_src(
            "light/mod.rs",
            _lines("use crate::sdk::ble_app::light_ll::mesh_management::ll_device_status_update;"),
        )
        out = self.assert_violations("1: light-upward-dep: crate::sdk")
        self.assertEqual(
            out.splitlines()[0], f"{path}:1: light-upward-dep: crate::sdk"
        )

    def test_qualified_sibling_path_is_rejected(self) -> None:
        # `factory_reset` is a sibling subsystem: a fully-qualified call must not
        # dodge the rule.
        self.write_src(
            "light/transitions.rs",
            _lines("pub fn f() { crate::factory_reset::erase_config_sectors(); }"),
        )
        self.assert_violations("1: light-upward-dep: crate::factory_reset")

    def test_tlsr8266_pac_import_is_rejected(self) -> None:
        self.write_src(
            "light/transitions.rs", _lines("use tlsr8266_pac::fake_mmio::FakeMmio;")
        )
        self.assert_violations("1: light-forbidden-crate: tlsr8266_pac")

    def test_every_forbidden_module_is_rejected(self) -> None:
        banned = sorted(check_layering.LIGHT_FORBIDDEN_MODULES - {"state"})
        for module in banned:
            with self.subTest(module=module):
                _write(
                    self.src_root / "light" / f"{module}.rs",
                    _lines(f"use crate::{module}::thing;"),
                )
        out = self.assert_violations(*[f"crate::{m}" for m in banned])
        self.assertEqual(len(out.splitlines()), len(banned))

    def test_downward_and_internal_imports_are_accepted(self) -> None:
        self.write_src(
            "light/mod.rs",
            _lines(
                "use core::cmp::min;",
                "use critical_section;",
                "use fixed::types::I16F16;",
                "use mesh_protocol::mesh_types::LGT_CMD_LIGHT_ONOFF;",
                "use tlsr8266_hal::drivers::pwm::pwm_set_cmp;",
                "use crate::config::VENDOR_ID;",
                "use super::transitions::LIGHT_HW;",
                "use transitions::dispatch_message;",
            ),
        )
        self.assert_clean()

    def test_forbidden_import_inside_cfg_test_is_accepted(self) -> None:
        # The subsystem's own tests install the HAL's host fakes; only
        # production code is audited.
        self.write_src(
            "light/effects.rs",
            _lines(
                "pub fn production() {}",
                "",
                "#[cfg(test)]",
                "mod tests {",
                "    use crate::sdk::rf_drv::mock_set_advertisement_manufacturer_data;",
                "    use tlsr8266_pac::fake_mmio::FakeMmio;",
                "}",
            ),
        )
        self.assert_clean()

    def test_rule_is_scoped_to_the_light_directory(self) -> None:
        # Control: the same import one directory up is production code but not
        # `light` code, so the purity rule leaves it to the layering rules.
        self.write_src(
            "main_light.rs",
            _lines("use crate::sdk::rf_drv::set_advertisement_manufacturer_data;"),
        )
        self.assert_clean()


class TestSubsystemPurityTable(_TempSrcTree):
    """The per-directory purity table is the single source of every row."""

    def test_table_has_every_application_subsystem(self) -> None:
        # C-6 shipped `ota`, C-7 added `factory_reset` and `light`, and C-8
        # completed the application layer with these four.
        self.assertEqual(
            set(check_layering.SUBSYSTEM_PURITY),
            {
                "ota",
                "factory_reset",
                "light",
                "diagnostics",
                "logging",
                "uart",
                "mesh",
            },
        )

    def test_ota_row_still_matches_the_c6_constants(self) -> None:
        # The generalisation must not quietly widen or narrow C-6's rule.
        modules, crates = check_layering.SUBSYSTEM_PURITY["ota"]
        self.assertEqual(modules, check_layering.OTA_FORBIDDEN_MODULES)
        self.assertEqual(crates, check_layering.OTA_FORBIDDEN_CRATES)

    def test_ota_helper_delegates_to_the_table(self) -> None:
        # `audit_ota_purity_source` is C-6's public helper; it now reads the row.
        violations = check_layering.audit_ota_purity_source(
            _lines("use crate::sdk::ble_app::x;"), "ota/mod.rs"
        )
        self.assertEqual(
            [(item.rule, item.violation) for item in violations],
            [("ota-upward-dep", "crate::sdk")],
        )

    def test_no_row_forbids_its_own_name(self) -> None:
        # A row lists siblings and layers *above* the subsystem. Adding the
        # subsystem's own name would ban its own intra-module paths.
        for name, (modules, _crates) in check_layering.SUBSYSTEM_PURITY.items():
            with self.subTest(row=name):
                self.assertNotIn(name, modules)

    def test_uart_is_a_retired_name_fence_everywhere_but_its_own_row(self) -> None:
        # C-8 renamed the bridge to `crate::uart`; the deleted spelling stays
        # banned in every other row so it cannot come back by import. The bridge
        # itself is the one row that omits it, because `uart` is its own name.
        for name, (modules, _crates) in check_layering.SUBSYSTEM_PURITY.items():
            if name == "uart":
                continue
            with self.subTest(row=name):
                self.assertIn("uart_manager", modules)

    def test_logging_row_resolves_to_the_module_file(self) -> None:
        # `logging` is a single file, not a directory: the override keeps one
        # table, and the match accepts an exact file path.
        roots = check_layering.subsystem_roots("/src")
        self.assertEqual(roots["logging"], os.path.join("/src", "logging.rs"))
        self.assertEqual(roots["uart"], os.path.join("/src", "uart"))
        self.assertEqual(roots["mesh"], os.path.join("/src", "mesh"))
        self.assertEqual(
            check_layering._subsystem_of("/src/logging.rs", roots), "logging"
        )
        self.assertEqual(
            check_layering._subsystem_of("/src/uart/mod.rs", roots), "uart"
        )
        self.assertIsNone(check_layering._subsystem_of("/src/app.rs", roots))


class TestDiagnosticsPurityRule(_TempSrcTree):
    """`rust/src/diagnostics/**` production code may not name the layers above.

    `diagnostics` is the leaf of the application layer, so the whole rest of the
    application is forbidden — `logging` included, because the sink is what
    writes a panic body and the direction is `logging -> diagnostics::rtt`.
    """

    def test_sibling_import_is_rejected(self) -> None:
        path = self.write_src(
            "diagnostics/mod.rs", _lines("use crate::logging::UartStream;")
        )
        out = self.assert_violations("1: diagnostics-upward-dep: crate::logging")
        self.assertEqual(
            out.splitlines()[0], f"{path}:1: diagnostics-upward-dep: crate::logging"
        )

    def test_qualified_uart_path_is_rejected(self) -> None:
        # A fully-qualified call is not an import, and must not dodge the rule.
        self.write_src(
            "diagnostics/rtt.rs",
            _lines("pub fn f() { crate::uart::init(); }"),
        )
        self.assert_violations("1: diagnostics-upward-dep: crate::uart")

    def test_tlsr8266_pac_import_is_rejected(self) -> None:
        self.write_src(
            "diagnostics/rtt.rs", _lines("use tlsr8266_pac::fake_mmio::FakeMmio;")
        )
        self.assert_violations("1: diagnostics-forbidden-crate: tlsr8266_pac")

    def test_every_forbidden_module_is_rejected(self) -> None:
        banned = sorted(check_layering.DIAGNOSTICS_FORBIDDEN_MODULES - {"state"})
        for module in banned:
            with self.subTest(module=module):
                _write(
                    self.src_root / "diagnostics" / f"{module}.rs",
                    _lines(f"use crate::{module}::thing;"),
                )
        out = self.assert_violations(*[f"crate::{m}" for m in banned])
        self.assertEqual(len(out.splitlines()), len(banned))

    def test_downward_and_internal_imports_are_accepted(self) -> None:
        self.write_src(
            "diagnostics/mod.rs",
            _lines(
                "use core::sync::atomic::Ordering;",
                "use critical_section;",
                "use tlsr8266_hal::drivers::flash::flash_read_page;",
                "use crate::config::PANIC_LOG_BODY_MAX;",
                "use super::rtt::rtt_write_bytes;",
                "mod rtt;",
            ),
        )
        self.assert_clean()

    def test_forbidden_import_inside_cfg_test_is_accepted(self) -> None:
        self.write_src(
            "diagnostics/mod.rs",
            _lines(
                "pub fn production() {}",
                "",
                "#[cfg(test)]",
                "mod tests {",
                "    use crate::mesh::reset_for_test;",
                "    use tlsr8266_pac::fake_mmio::FakeMmio;",
                "}",
            ),
        )
        self.assert_clean()


class TestLoggingPurityRule(_TempSrcTree):
    """`rust/src/logging.rs` production code may not name its siblings.

    `logging` sits above `diagnostics` and below `uart`, so the transport and
    every subsystem are forbidden while `crate::diagnostics` is legal. The row
    is keyed on the module file itself, not a directory.
    """

    def test_uart_import_is_rejected(self) -> None:
        path = self.write_src("logging.rs", _lines("use crate::uart::send_message;"))
        out = self.assert_violations("1: logging-upward-dep: crate::uart")
        self.assertEqual(
            out.splitlines()[0], f"{path}:1: logging-upward-dep: crate::uart"
        )

    def test_diagnostics_import_is_accepted(self) -> None:
        self.write_src(
            "logging.rs",
            _lines(
                "use crate::diagnostics::rtt::rtt_write_bytes;",
                "use mesh_protocol::uart::UartData;",
                "use critical_section;",
            ),
        )
        self.assert_clean()

    def test_every_forbidden_module_is_rejected(self) -> None:
        banned = sorted(check_layering.LOGGING_FORBIDDEN_MODULES - {"state"})
        for module in banned:
            with self.subTest(module=module):
                _write(
                    self.src_root / "logging.rs",
                    _lines(f"use crate::{module}::thing;"),
                )
                out = self.assert_violations(f"logging-upward-dep: crate::{module}")
                self.assertEqual(len(out.splitlines()), 1)
        self.assertEqual(len(banned), 7)

    def test_tlsr8266_pac_import_is_rejected(self) -> None:
        self.write_src(
            "logging.rs", _lines("use tlsr8266_pac::write_reg8;")
        )
        self.assert_violations("1: logging-forbidden-crate: tlsr8266_pac")

    def test_rule_does_not_apply_to_an_app_root_file(self) -> None:
        # Control: the override is an exact path, not "every root-level file".
        self.write_src(
            "app.rs", _lines("use crate::uart::send_sync_panic;")
        )
        self.assert_clean()


class TestUartPurityRule(_TempSrcTree):
    """`rust/src/uart/**` production code may not name `mesh`, `sdk` or a peer.

    The bridge owns framing, ACK/retry and the two embassy tasks, and reaches
    `crate::logging` (for `uprintln!`) and the HAL. The mesh relay and the host
    command execution arrive through `MeshEventHooks` / `UartCommandHandler`, so
    `crate::mesh` and `crate::sdk` must stay out.
    """

    def test_mesh_import_is_rejected(self) -> None:
        path = self.write_src(
            "uart/mod.rs", _lines("use crate::mesh::send_mesh_message;")
        )
        out = self.assert_violations("1: uart-upward-dep: crate::mesh")
        self.assertEqual(
            out.splitlines()[0], f"{path}:1: uart-upward-dep: crate::mesh"
        )

    def test_sdk_import_is_rejected(self) -> None:
        self.write_src(
            "uart/protocol.rs",
            _lines("use crate::sdk::ble_app::queues::enqueue_uart_tx;"),
        )
        self.assert_violations("1: uart-upward-dep: crate::sdk")

    def test_tlsr8266_pac_import_is_rejected(self) -> None:
        self.write_src(
            "uart/mod.rs", _lines("use tlsr8266_pac::fake_mmio::FakeMmio;")
        )
        self.assert_violations("1: uart-forbidden-crate: tlsr8266_pac")

    def test_every_forbidden_module_is_rejected(self) -> None:
        banned = sorted(check_layering.UART_FORBIDDEN_MODULES - {"state"})
        for module in banned:
            with self.subTest(module=module):
                _write(
                    self.src_root / "uart" / f"{module}.rs",
                    _lines(f"use crate::{module}::thing;"),
                )
        out = self.assert_violations(*[f"crate::{m}" for m in banned])
        self.assertEqual(len(out.splitlines()), len(banned))

    def test_logging_and_the_hal_are_accepted(self) -> None:
        # `uart` sits *above* `logging`, so naming it (for `uprintln!`) and the
        # HAL it drives is the legal downward direction.
        self.write_src(
            "uart/mod.rs",
            _lines(
                "use core::fmt::Write;",
                "use critical_section;",
                "use mesh_protocol::uart::UartData;",
                "use tlsr8266_hal::drivers::uart::UartDriver;",
                "use crate::logging::UartStream;",
                "use protocol::decode_command;",
            ),
        )
        self.assert_clean()


class TestMeshPurityRule(_TempSrcTree):
    """`rust/src/mesh/**` production code may not name `uart`, the lamp or `light`.

    In the synthetic tree this fixture builds, `mesh` is the one subsystem
    allowed `crate::sdk` — it owns the radio, the node table and `ble_app`'s
    queues — and `crate::config` / `crate::logging` like everything else. The
    row stays armed for that fixture; the shipped tree names `tlsr8266_radio`
    instead (C-10 (#72) deleted `rust/src/sdk/`, and `git grep crate::sdk --
    rust/` is empty). `app` (the composition root) and `irq` (its ISR layer)
    are forbidden as well, so the module doc's "it never names `app`" is a
    checked claim. The four application edges go through `MeshEventHooks`, which
    is what keeps this row possible.
    """

    def test_uart_import_is_rejected(self) -> None:
        path = self.write_src(
            "mesh/mod.rs", _lines("use crate::uart::on_mesh_rx_packet;")
        )
        out = self.assert_violations("1: mesh-upward-dep: crate::uart")
        self.assertEqual(
            out.splitlines()[0], f"{path}:1: mesh-upward-dep: crate::uart"
        )

    def test_light_import_is_rejected(self) -> None:
        self.write_src(
            "mesh/fsm.rs", _lines("pub fn f() { crate::light::device_status_update(); }")
        )
        self.assert_violations("1: mesh-upward-dep: crate::light")

    def test_tlsr8266_pac_import_is_rejected(self) -> None:
        self.write_src(
            "mesh/mod.rs", _lines("use tlsr8266_pac::fake_mmio::FakeMmio;")
        )
        self.assert_violations("1: mesh-forbidden-crate: tlsr8266_pac")

    def test_every_forbidden_module_is_rejected(self) -> None:
        banned = sorted(check_layering.MESH_FORBIDDEN_MODULES - {"state"})
        for module in banned:
            with self.subTest(module=module):
                _write(
                    self.src_root / "mesh" / f"{module}.rs",
                    _lines(f"use crate::{module}::thing;"),
                )
        out = self.assert_violations(*[f"crate::{m}" for m in banned])
        self.assertEqual(len(out.splitlines()), len(banned))

    def test_composition_root_and_the_isr_layer_are_rejected(self) -> None:
        # L5: the module doc claims `mesh` never names `app`, and `irq` sits
        # above it. Both names must be in the row, or the claim is a comment
        # that lets `mesh -> app` back in silently.
        self.write_src(
            "mesh/mod.rs",
            _lines(
                "pub fn f() { crate::app::rf_link_data_callback(); }",
                "pub fn g() { crate::irq::handler(); }",
            ),
        )
        self.assert_violations(
            "1: mesh-upward-dep: crate::app", "2: mesh-upward-dep: crate::irq"
        )
        self.assertLessEqual(
            {"app", "irq"}, set(check_layering.MESH_FORBIDDEN_MODULES)
        )

    def test_sdk_config_and_logging_are_accepted(self) -> None:
        # The legal downward direction: the SDK (radio + node table), `config`,
        # the HAL, the wire crate and `uprintln!`.
        self.write_src(
            "mesh/mod.rs",
            _lines(
                "use critical_section;",
                "use mesh_protocol::packet_types::Packet;",
                "use tlsr8266_hal::mcu::clock::clock_time;",
                "use crate::config::VENDOR_ID;",
                "use crate::sdk::ble_app::queues::enqueue_mesh_tx;",
                "pub fn f() { uprintln!(\"mesh\"); }",
            ),
        )
        self.assert_clean()

    def test_forbidden_import_inside_cfg_test_is_accepted(self) -> None:
        # The subsystem's own tests use the lamp and the PAC fakes as oracles.
        self.write_src(
            "mesh/mod.rs",
            _lines(
                "pub fn production() {}",
                "",
                "#[cfg(test)]",
                "mod tests {",
                "    use crate::light::transitions::LIGHT_CMD_CHANNEL;",
                "    use tlsr8266_pac::fake_mmio::FakeMmio;",
                "}",
            ),
        )
        self.assert_clean()


class TestStateRemovedRule(_TempSrcTree):
    """`crate::state` is gone: any reference fails, tests included."""

    def test_crate_state_reference_fails(self) -> None:
        path = self.write_src("x.rs", _lines("use crate::state::PairState;"))
        out = self.assert_violations("1: state-removed: crate::state")
        self.assertEqual(out.splitlines()[0], f"{path}:1: state-removed: crate::state")

    def test_whitespace_around_the_path_separator_fails(self) -> None:
        # `::` is whitespace-insensitive to the lexer.  A crate alias needs no
        # rule: `c::state` is a compile error once the module is deleted.
        self.write_src("x.rs", _lines("use crate :: state :: X;"))
        self.assert_violations("1: state-removed: crate::state")

    def test_crate_state_inside_cfg_test_also_fails(self) -> None:
        # The rule reads the raw file: test modules import the deleted module
        # too, so the test mask must not hide it.
        self.write_src(
            "x.rs",
            _lines(
                "pub fn f() {}",
                "",
                "#[cfg(test)]",
                "mod tests {",
                "    use crate::state::PAIR_LOGIN_OK;",
                "}",
            ),
        )
        self.assert_violations("5: state-removed: crate::state")

    def test_other_root_modules_are_legal(self) -> None:
        self.write_src(
            "x.rs",
            _lines("use crate::config::VENDOR_ID;", "use crate::BIT;"),
        )
        self.assert_clean()


class TestStaticMutRule(_TempSrcTree):
    """Production `static mut` items are banned; references are not items."""

    def test_static_mut_item_fails(self) -> None:
        self.write_src("x.rs", _lines("static mut X: u8 = 0;"))
        self.assert_violations("1: static-mut: static mut X")

    def test_pub_crate_static_mut_item_fails(self) -> None:
        self.write_src("x.rs", _lines("pub(crate) static mut X: u8 = 0;"))
        self.assert_violations("1: static-mut: static mut X")

    def test_static_mut_reference_forms_are_legal(self) -> None:
        # Both patterns exist in the tree: the #44 executor receiver and the
        # attribute-table accessor.
        self.write_src(
            "executor.rs",
            _lines(
                "pub fn run(&'static mut self, init: u32) {",
                "    let _ = init;",
                "}",
            ),
        )
        self.write_src(
            "attribute.rs",
            _lines(
                "pub fn get_g_attributes() -> &'static mut [AttributeT; 29] {",
                "    todo!()",
                "}",
            ),
        )
        self.assert_clean()

    def test_cfg_test_static_mut_is_ignored(self) -> None:
        self.write_src(
            "x.rs",
            _lines(
                "pub fn production() {}",
                "",
                "#[cfg(test)]",
                "pub static mut SCRATCH: u32 = 0;",
            ),
        )
        self.assert_clean()

    def test_struct_field_cfg_test_does_not_hide_a_following_static_mut(self) -> None:
        # H-1: the field attribute used to mask everything up to the next brace
        # block, so a production `static mut` in that position was invisible.
        self.write_src(
            "x.rs",
            _lines(
                "pub struct S {",
                "    #[cfg(test)]",
                "    locked: u8,",
                "}",
                "",
                "pub static mut HIDDEN: u8 = 0;",
            ),
        )
        self.assert_violations("6: static-mut: static mut HIDDEN")

    def test_inline_attribute_static_mut_fails(self) -> None:
        # M-4: an attribute on the item's own line used to defeat the anchor.
        self.write_src(
            "x.rs",
            _lines('#[link_section = ".data"] pub static mut X: u8 = 0;'),
        )
        self.assert_violations("1: static-mut: static mut X")

    def test_static_mut_inside_a_macro_body_fails(self) -> None:
        # M-6: a single-line macro body used to defeat the line anchor.
        self.write_src(
            "x.rs",
            _lines("macro_rules! m { () => { pub static mut X: u8 = 0; } }"),
        )
        self.assert_violations("1: static-mut: static mut X")

    def test_cfg_test_field_inside_a_test_module_is_ignored(self) -> None:
        # No false positive: a field/statement attribute *inside* a test module
        # is test code like the module itself.
        self.write_src(
            "x.rs",
            _lines(
                "pub fn production() {}",
                "",
                "#[cfg(test)]",
                "mod tests {",
                "    struct S {",
                "        #[cfg(test)]",
                "        scratch: u8,",
                "    }",
                "    fn helper() {",
                "        #[cfg(test)]",
                "        static mut SCRATCH: u32 = 0;",
                "    }",
                "}",
            ),
        )
        self.assert_clean()


class TestUndocumentedUnsafeRule(_TempSrcTree):
    """Every production `unsafe` needs `// SAFETY:` directly above it."""

    def test_documented_sites_pass(self) -> None:
        self.write_src(
            "x.rs",
            _lines(
                "// SAFETY: caller guarantees `ptr` points at one valid u8.",
                "pub unsafe fn read(ptr: *const u8) -> u8 {",
                "    // SAFETY: same contract as the caller.",
                "    unsafe { *ptr }",
                "}",
            ),
        )
        self.assert_clean()

    def test_undocumented_unsafe_fails(self) -> None:
        self.write_src("x.rs", _lines("pub unsafe fn f() {}"))
        self.assert_violations(
            "1: undocumented-unsafe: no `// SAFETY:` comment directly above"
        )

    def test_cfg_test_safety_comment_does_not_launder_production_unsafe(self) -> None:
        self.write_src(
            "x.rs",
            _lines(
                "#[cfg(test)]",
                "mod tests {",
                "    // SAFETY: test-only justification.",
                "    unsafe fn helper() {}",
                "}",
                "",
                "pub unsafe fn production() {}",
            ),
        )
        out = self.assert_violations("7: undocumented-unsafe")
        self.assertEqual(len(out.splitlines()), 1)

    def test_unsafe_inside_cfg_test_is_ignored(self) -> None:
        self.write_src(
            "x.rs",
            _lines(
                "pub fn production() {}",
                "",
                "#[cfg(test)]",
                "mod tests {",
                "    unsafe fn helper() {}",
                "}",
            ),
        )
        self.assert_clean()

    def test_safety_comment_above_a_blank_line_does_not_count(self) -> None:
        self.write_src(
            "x.rs",
            _lines("// SAFETY: stale note.", "", "pub unsafe fn f() {}"),
        )
        self.assert_violations("3: undocumented-unsafe")

    def test_struct_field_cfg_test_does_not_hide_a_following_unsafe(self) -> None:
        # H-1, undocumented-unsafe half of the same defect.
        self.write_src(
            "x.rs",
            _lines(
                "pub struct S {",
                "    #[cfg(test)]",
                "    locked: u8,",
                "}",
                "",
                "pub unsafe fn hidden() {}",
            ),
        )
        self.assert_violations("6: undocumented-unsafe")

    def test_cfg_test_statement_does_not_hide_a_following_unsafe(self) -> None:
        self.write_src(
            "x.rs",
            _lines(
                "pub fn f() {",
                "    #[cfg(test)]",
                "    let deadline = 0u32;",
                "    unsafe {}",
                "}",
            ),
        )
        self.assert_violations("4: undocumented-unsafe")

    def test_cfg_test_field_before_documented_unsafe_impls_is_clean(self) -> None:
        # The exact shape shipped in `sdk/common/sync.rs`: the attribute is on a
        # field, and both production `unsafe impl`s must stay visible *and*
        # documented, so the real tree stays clean with the corrected count.
        self.write_src(
            "x.rs",
            _lines(
                "pub struct CriticalSectionMutex<T: ?Sized> {",
                "    #[cfg(test)]",
                "    locked: AtomicBool,",
                "    data: UnsafeCell<T>,",
                "}",
                "",
                "// SAFETY: (4) `Send` wrapper.",
                "unsafe impl<T: ?Sized + Send> Send for CriticalSectionMutex<T> {}",
                "// SAFETY: (4) `Sync` wrapper.",
                "unsafe impl<T: ?Sized + Send> Sync for CriticalSectionMutex<T> {}",
            ),
        )
        self.assert_clean()


class TestUnsafeBudgetRule(_TempSrcTree):
    """The production `unsafe` count is a one-way ratchet."""

    def _documented_sites(self, count: int) -> str:
        blocks = [
            f"// SAFETY: generated probe {index}.\nunsafe fn probe_{index}() {{}}"
            for index in range(count)
        ]
        return "\n".join(blocks) + "\n"

    def test_at_the_ceiling_passes(self) -> None:
        ceiling = check_layering.MAX_PRODUCTION_UNSAFE
        self.write_src("budget.rs", self._documented_sites(ceiling))
        self.assert_clean()

    def test_exceeding_the_ceiling_fails(self) -> None:
        ceiling = check_layering.MAX_PRODUCTION_UNSAFE
        self.write_src("budget.rs", self._documented_sites(ceiling + 1))
        out = self.assert_violations("unsafe-budget")
        self.assertIn(f"exceeds ceiling {ceiling}", out)
        self.assertNotIn("undocumented-unsafe", out)


class TestHostOnlyUnsafeBudget(_TempSrcTree):
    """`#[cfg(not(target_os = "none"))]` code never reaches the firmware image.

    It is excluded from the numeric `unsafe-budget` only.  The SAFETY and
    `static mut` rules keep applying: host-only code is still real Rust in the
    audited tree and still compiled for host tests.
    """

    def _audit_source(self, *lines: str) -> tuple[list, int]:
        path = self.write_src("x.rs", _lines(*lines))
        return check_layering.audit_universal_source(
            path.read_text(encoding="utf-8"), str(path)
        )

    def test_host_only_item_is_excluded_from_the_budget(self) -> None:
        violations, total = self._audit_source(
            '#[cfg(not(target_os = "none"))]',
            "// SAFETY: (1) host-only probe.",
            "unsafe fn host_probe() {}",
            "// SAFETY: (1) production probe.",
            "unsafe fn production_probe() {}",
        )
        self.assertEqual(violations, [])
        self.assertEqual(total, 1)  # only the production site is charged

    def test_host_only_unsafe_without_safety_still_fails(self) -> None:
        violations, total = self._audit_source(
            '#[cfg(not(target_os = "none"))]',
            "unsafe fn host_probe() {}",
        )
        self.assertEqual(
            [(item.rule, item.line) for item in violations],
            [("undocumented-unsafe", 2)],
        )
        self.assertEqual(total, 0)  # excluded from the budget, not from SAFETY

    def test_host_only_unsafe_with_safety_raises_nothing(self) -> None:
        violations, total = self._audit_source(
            '#[cfg(not(target_os = "none"))]',
            "// SAFETY: (1) host-only probe.",
            "unsafe fn host_probe() {}",
        )
        self.assertEqual(violations, [])
        self.assertEqual(total, 0)

    def test_host_only_static_mut_is_still_reported(self) -> None:
        violations, total = self._audit_source(
            '#[cfg(not(target_os = "none"))]',
            "static mut HOST_SCRATCH: u8 = 0;",
        )
        self.assertEqual(
            [(item.rule, item.line, item.violation) for item in violations],
            [("static-mut", 2, "static mut HOST_SCRATCH")],
        )
        self.assertEqual(total, 0)

    def test_production_unsafe_is_still_counted(self) -> None:
        # Positive control for the exclusion above: nothing but ordinary
        # production code here, and it must move the budget.
        violations, total = self._audit_source(
            "// SAFETY: (1) production probe.",
            "unsafe fn production_probe() {}",
            "",
            "// SAFETY: (1) second production probe.",
            "unsafe fn production_probe_two() {}",
        )
        self.assertEqual(violations, [])
        self.assertEqual(total, 2)

    def test_unsafe_impl_and_nested_unsafe_inside_a_host_only_item(self) -> None:
        violations, total = self._audit_source(
            '#[cfg(not(target_os = "none"))]',
            "mod host {",
            "    // SAFETY: (4) audited wrapper.",
            "    unsafe impl Send for Probe {}",
            "    // SAFETY: (1) raw pointer probe.",
            "    pub unsafe fn probe(p: *const u8) -> u8 {",
            "        // SAFETY: (2) same contract as the caller.",
            "        unsafe { *p }",
            "    }",
            "}",
            "// SAFETY: (1) production probe.",
            "unsafe fn production_probe() {}",
        )
        self.assertEqual(violations, [])
        self.assertEqual(total, 1)

    def test_annotation_shaped_host_only_block_stops_at_its_own_braces(self) -> None:
        # The host-only attribute here annotates a bare block, not an item, so
        # the extent is `_annotation_extent`'s job: stopping one block early
        # would hide the target branch below it from the budget.
        violations, total = self._audit_source(
            "pub fn probe(v: u8) -> u8 {",
            '    #[cfg(not(target_os = "none"))]',
            "    {",
            "        // SAFETY: (1) host-only branch.",
            "        unsafe { host_read(v) }",
            "    }",
            '    #[cfg(target_os = "none")]',
            "    {",
            "        // SAFETY: (1) target branch.",
            "        unsafe { read(v) }",
            "    }",
            "}",
        )
        self.assertEqual(violations, [])
        self.assertEqual(total, 1)  # the target branch is still firmware code

    def test_host_only_attribute_text_in_a_comment_does_not_strip_code(self) -> None:
        # The attribute is matched on the raw text because `mask_source` blanks
        # the `"none"` literal; the mask guard is what keeps a copy of it in a
        # comment from swallowing the production `unsafe` below.
        violations, total = self._audit_source(
            '// historically: #[cfg(not(target_os = "none"))]',
            "",
            "// SAFETY: (1) production probe.",
            "unsafe fn production_probe() {}",
        )
        self.assertEqual(violations, [])
        self.assertEqual(total, 1)

    def test_host_only_sites_are_not_charged_to_the_firmware_ceiling(self) -> None:
        # Gate-level proof: more documented host-only `unsafe` sites than the
        # whole firmware ceiling must still pass, because none of them ship.
        ceiling = check_layering.MAX_PRODUCTION_UNSAFE
        self.write_src(
            "host.rs",
            "\n".join(
                '#[cfg(not(target_os = "none"))]\n'
                f"// SAFETY: (1) generated host-only probe {index}.\n"
                f"unsafe fn host_probe_{index}() {{}}"
                for index in range(ceiling + 1)
            )
            + "\n",
        )
        self.assert_clean()


class TestMeshProtocolCrateUniversalRules(_TempSrcTree):
    """The workspace crate is audited like the firmware for `unsafe` invariants."""

    def test_static_mut_inside_the_crate_tree_fails(self) -> None:
        path = self.write_crate(
            "packet_types.rs", _lines("pub static mut SCRATCH: u8 = 0;")
        )
        out = self.assert_violations("1: static-mut: static mut SCRATCH")
        self.assertEqual(out.splitlines()[0], f"{path}:1: static-mut: static mut SCRATCH")

    def test_undocumented_unsafe_inside_the_crate_tree_fails(self) -> None:
        # The shipped `Packet`-union view shape, with the SAFETY comment lost.
        self.write_crate(
            "packet_types.rs",
            _lines(
                "impl Packet {",
                "    pub fn head(&self) -> &[u8; 10] {",
                "        unsafe { &self.head }",
                "    }",
                "}",
            ),
        )
        self.assert_violations("3: undocumented-unsafe")

    def test_audited_unsafe_impl_inside_the_crate_tree_passes(self) -> None:
        # The real crate ships documented `Pod`/`Zeroable` impls; they must pass.
        self.write_crate(
            "mesh_types.rs",
            _lines(
                "// SAFETY: (4) audited packed-wire view: all-zero bytes are valid.",
                "unsafe impl Zeroable for MeshNodeStValT {}",
            ),
        )
        self.assert_clean()

    def test_crate_unsafe_is_not_charged_to_the_firmware_budget(self) -> None:
        # The ratchet is measured over rust/src only: a crate tree carrying more
        # unsafe sites than the whole firmware ceiling must still pass.
        ceiling = check_layering.MAX_PRODUCTION_UNSAFE
        self.write_crate(
            "crc.rs",
            _lines(
                *(
                    f"// SAFETY: generated probe {index}.\nunsafe fn probe_{index}() {{}}"
                    for index in range(ceiling + 1)
                )
            ),
        )
        self.assert_clean()

    def test_crate_failure_is_labelled_with_its_own_root(self) -> None:
        self.write_crate("uart.rs", _lines("pub static mut SCRATCH: u8 = 0;"))
        code, out, err = self.run_gate()
        self.assertEqual(code, 1, out + err)
        self.assertIn("mesh-protocol audit FAILED: 1 violation(s)", err)
        self.assertNotIn("rust src audit FAILED", err)

    def test_clean_run_reports_both_universal_roots(self) -> None:
        code, out, err = self.run_gate()
        self.assertEqual(code, 0, out + err)
        self.assertIn(f"under {self.src_root}", out)
        self.assertIn(f"under {self.crate_root}", out)


class TestUniversalPassCli(_TempSrcTree):
    """Each pass reports independently; either failure exits 1."""

    def test_universal_only_failure_exits_1(self) -> None:
        self.write_src("x.rs", _lines("static mut GLOBAL: u8 = 0;"))
        code, out, err = self.run_gate()
        self.assertEqual(code, 1, out + err)
        self.assertIn("static-mut", out)
        self.assertNotIn("upward-dep", out)
        self.assertIn("rust src audit FAILED", err)
        self.assertNotIn("sdk layering FAILED", err)

    def test_layering_only_failure_exits_1(self) -> None:
        self.write("x.rs", _lines("use crate::app::App;"))
        code, out, err = self.run_gate()
        self.assertEqual(code, 1, out + err)
        self.assertIn("upward-dep: crate::app", out)
        self.assertIn("sdk layering FAILED", err)
        self.assertNotIn("static-mut", out)
        self.assertNotIn("undocumented-unsafe", out)

    def test_all_passes_clean_prints_three_summaries(self) -> None:
        code, out, err = self.run_gate()
        self.assertEqual(code, 0, out + err)
        self.assertEqual(
            [line.split(":")[0] for line in out.splitlines()],
            ["sdk layering OK", "rust src audit OK", "mesh-protocol audit OK"],
        )


class TestRepeatableCrateRoot(_TempSrcTree):
    """`--crate-root` may be repeated, and an explicit value audits only itself."""

    def test_both_explicit_crate_roots_are_audited(self) -> None:
        second = self.crate_root.parent.parent / "second-crate" / "src"
        _write(second / "lib.rs", _lines("pub static mut OTHER: u8 = 0;"))
        self.write_crate("lib.rs", _lines("pub static mut SCRATCH: u8 = 0;"))
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(
                [
                    "--root",
                    str(self.root),
                    "--src-root",
                    str(self.src_root),
                    "--crate-root",
                    str(self.crate_root),
                    "--crate-root",
                    str(second),
                ]
            )
        self.assertEqual(code, 1, out.getvalue() + err.getvalue())
        self.assertIn("static-mut: static mut SCRATCH", out.getvalue())
        self.assertIn("static-mut: static mut OTHER", out.getvalue())
        self.assertIn("mesh-protocol audit FAILED: 1 violation(s)", err.getvalue())
        self.assertIn("second-crate audit FAILED: 1 violation(s)", err.getvalue())

    def test_crate_root_alone_audits_only_that_crate(self) -> None:
        # The repeatable flag replaces the default list: no implicit firmware
        # pass and no implicit workspace crate is read along with it.
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(["--crate-root", str(self.crate_root)])
        self.assertEqual(code, 0, out.getvalue() + err.getvalue())
        self.assertIn(f"under {self.crate_root}", out.getvalue())
        self.assertNotIn("rust src audit", out.getvalue())
        self.assertNotIn("tlsr8266-pac audit", out.getvalue())
        self.assertNotIn("tlsr8266-hal audit", out.getvalue())


if __name__ == "__main__":
    unittest.main()
