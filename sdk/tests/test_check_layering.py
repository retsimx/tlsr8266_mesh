"""Unit tests for the ``rust/src/sdk/`` layering gate.

Run from the repository root::

    python3 -m unittest discover -s sdk/tests -v

Every test is hermetic: synthetic trees live in a temporary directory and the
only real tree touched is ``rust/src/sdk/`` (read-only).
"""

from __future__ import annotations

import contextlib
import io
import re
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "sdk"))

import check_layering  # noqa: E402  (sys.path set up above)

SDK_ROOT = REPO_ROOT / "rust" / "src" / "sdk"


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


class TestRealSdkTree(unittest.TestCase):
    """The refactor is complete: the shipped tree must pass."""

    def test_default_root_is_the_repo_sdk(self) -> None:
        self.assertEqual(Path(check_layering.DEFAULT_ROOT).resolve(), SDK_ROOT.resolve())

    def test_real_sdk_tree_is_clean(self) -> None:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main([])
        self.assertEqual(code, 0, out.getvalue() + err.getvalue())
        self.assertIn("0 violations", out.getvalue())
        self.assertGreaterEqual(len(check_layering.rust_files(str(SDK_ROOT))), 40)


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
        banned = [
            "app",
            "common",
            "main_light",
            "mesh",
            "light_manager",
            "uart_manager",
            "ota",
            "diagnostics",
            "logging",
            "irq",
            "embassy",
        ]
        for module in banned:
            with self.subTest(module=module):
                _write(self.root / f"{module}.rs", _lines(f"use crate::{module}::thing;"))
        out = self.assert_violations(*[f"crate::{m}" for m in banned])
        self.assertEqual(len(out.splitlines()), len(banned))

    def test_deferred_modules_are_legal(self) -> None:
        self.write(
            "x.rs",
            _lines(
                "use crate::state::PAIR_LOGIN_OK;",
                "use crate::config::VENDOR_ID;",
                "use crate::sdk::common::sync::CriticalSectionMutex;",
                "use crate::BIT;",
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


class TestLayerRules(_TempTree):
    def test_common_may_not_import_mcu(self) -> None:
        self.write("common/clock_user.rs", _lines("use crate::sdk::mcu::clock::clock_time;"))
        self.assert_violations(
            "1: layer-common: crate::sdk::mcu::clock::clock_time"
        )

    def test_mcu_may_not_import_drivers(self) -> None:
        self.write("mcu/clock_user.rs", _lines("use crate::sdk::drivers::uart::uart_init;"))
        self.assert_violations("1: layer-mcu: crate::sdk::drivers::uart::uart_init")

    def test_drivers_may_not_import_app_layer(self) -> None:
        self.write("drivers/uart_user.rs", _lines("use crate::mesh::is_online;"))
        out = self.assert_violations("layer-drivers", "upward-dep: crate::mesh")
        self.assertEqual(len(out.splitlines()), 2)

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
        self.write(
            "drivers/uart_user.rs",
            _lines(
                "use crate::sdk::common::sync::CriticalSectionMutex;",
                "use crate::sdk::mcu::register::read_reg8;",
                "use crate::sdk::drivers::spi::spi_write;",
                "use embassy_time::Instant;",
            ),
        )
        self.assert_clean()

    def test_root_macros_are_legal_imports(self) -> None:
        self.write("common/bits.rs", _lines("use crate::{BIT, MASK_VAL};"))
        self.write("mcu/bits.rs", _lines("use crate::{regrw_idx, BIT_LOW_BIT};"))
        self.write("drivers/bits.rs", _lines("use crate::{BM_CLR, BM_SET, CLR_FLD};"))
        self.assert_clean()

    def test_new_external_crate_in_a_layer_is_rejected(self) -> None:
        self.write("common/dep.rs", _lines("use embassy_sync::signal::Signal;"))
        self.assert_violations("1: layer-common: external crate `embassy_sync`")

    def test_layer_rules_do_not_apply_outside_the_layer_directories(self) -> None:
        self.write("ble_app/sibling.rs", _lines("use crate::sdk::drivers::uart::uart_init;"))
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
    """Re-inject the historical violations into copies of the three trap files."""

    def _copy_and_inject(self, relative: str, anchor: str, injected: str) -> tuple[str, int]:
        source = (SDK_ROOT / relative).read_text(encoding="utf-8")
        self.assertIn(anchor, source, f"anchor drifted in {relative}")
        text = source.replace(anchor, anchor + injected, 1)
        # Keep the original path: the layering gate derives both the layer and
        # the legal `super::` climb from the file's position in the tree.
        self.write(relative, text)
        injected_line = next(
            number
            for number, line in enumerate(text.splitlines(), start=1)
            if injected.strip() in line
        )
        return text, injected_line

    def test_ble_ll_attribute_inline_cfg_test_static_does_not_truncate(self) -> None:
        relative = "ble_app/ble_ll_attribute.rs"
        source = (SDK_ROOT / relative).read_text(encoding="utf-8")
        self.write(relative, source)
        self.assert_clean()  # the unmodified copy is clean too

        anchor = "#[cfg(test)]\npub static mut __RAM_START_ADDR: u32 = 0;"
        _, line = self._copy_and_inject(relative, anchor, "\n\nuse crate::app::App;")
        self.assert_violations(f"{line}: upward-dep: crate::app")

    def test_packet_processing_fn_local_import_is_production(self) -> None:
        relative = "ble_app/light_ll/packet_processing.rs"
        source = (SDK_ROOT / relative).read_text(encoding="utf-8")
        self.write(relative, source)
        self.assert_clean()

        anchor = (
            "#[cfg_attr(test, mry::mry)]\n"
            "pub fn rf_link_add_tx_packet(packet: &Packet) -> Result<(), ()> {"
        )
        _, line = self._copy_and_inject(
            relative, anchor, "\n    use crate::mesh::MESH_NODE_ST_VAL_LEN;"
        )
        self.assert_violations(f"{line}: upward-dep: crate::mesh")

    def test_connection_fn_local_import_is_production(self) -> None:
        relative = "ble_app/irq/connection.rs"
        source = (SDK_ROOT / relative).read_text(encoding="utf-8")
        self.write(relative, source)
        self.assert_clean()

        anchor = "#[cfg_attr(test, mry::mry)]\nfn process_mesh_operations() {"
        _, line = self._copy_and_inject(
            relative, anchor, "\n    use crate::uart_manager::UartMsg;"
        )
        self.assert_violations(f"{line}: upward-dep: crate::uart_manager")


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
        self.assert_violations("super-escape: super::2 past rust/src/sdk (chain of 2, depth 0)")

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
        self.write("clean.rs", _lines("use crate::state::PAIR_LOGIN_OK;"))
        self.assert_clean()


if __name__ == "__main__":
    unittest.main()
