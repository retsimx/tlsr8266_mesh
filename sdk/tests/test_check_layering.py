"""Unit tests for the layering and universal ``unsafe`` gates.

Run from the repository root::

    python3 -m unittest discover -s sdk/tests -v

Every test is hermetic: synthetic trees live in a temporary directory and the
only real tree touched is ``rust/src/`` (read-only), by the handful of tests
that assert the shipped tree is clean.
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
SRC_ROOT = REPO_ROOT / "rust" / "src"


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

    def test_default_src_root_is_the_repo_src(self) -> None:
        self.assertEqual(
            Path(check_layering.DEFAULT_SRC_ROOT).resolve(), SRC_ROOT.resolve()
        )

    def test_real_sdk_tree_is_clean(self) -> None:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main([])
        self.assertEqual(code, 0, out.getvalue() + err.getvalue())
        self.assertIn("0 violations", out.getvalue())
        self.assertGreaterEqual(len(check_layering.rust_files(str(SDK_ROOT))), 40)

    def test_real_tree_reports_both_passes(self) -> None:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main([])
        self.assertEqual(code, 0, out.getvalue() + err.getvalue())
        self.assertIn("sdk layering OK", out.getvalue())
        self.assertIn("rust src audit OK", out.getvalue())

    def test_production_unsafe_ceiling_equals_the_measured_count(self) -> None:
        # The ratchet may be lowered, never raised: raising it without removing
        # `unsafe` sites leaves the measured count below the ceiling and fails
        # this assertion, and so does lowering it without the removals.
        violations, total = check_layering.audit_universal(
            check_layering.rust_files(str(SRC_ROOT)), str(SRC_ROOT)
        )
        self.assertEqual(violations, [])
        self.assertEqual(total, check_layering.MAX_PRODUCTION_UNSAFE)


class _TempSrcTree(unittest.TestCase):
    """A disposable ``rust/src`` plus a disposable SDK root for one test.

    The gate runs both passes here: ``--root`` keeps the layering pass on the
    fake SDK tree and ``--src-root`` points the universal pass at the fake
    source tree, so the real ``rust/src`` is never read.
    """

    def setUp(self) -> None:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.src_root = Path(tmp.name) / "src"
        self.root = self.src_root / "sdk"
        self.root.mkdir(parents=True)
        _write(self.root / "clean.rs", _lines("pub fn f() {}"))
        _write(self.src_root / "clean.rs", _lines("pub fn f() {}"))

    def write_src(self, relative: str, content: str) -> Path:
        return _write(self.src_root / relative, content)

    def write(self, relative: str, content: str) -> Path:
        return _write(self.root / relative, content)

    def run_gate(self) -> tuple[int, str, str]:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(
                ["--root", str(self.root), "--src-root", str(self.src_root)]
            )
        return code, out.getvalue(), err.getvalue()

    def assert_clean(self) -> None:
        code, out, err = self.run_gate()
        self.assertEqual(code, 0, out + err)
        self.assertIn("rust src audit OK", out)

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

    def test_config_and_sdk_imports_are_legal(self) -> None:
        # `crate::state` used to be legal here; the universal `state-removed`
        # rule now bans it outright (see TestStateRemovedRule).
        self.write(
            "x.rs",
            _lines(
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
        self.write("clean.rs", _lines("use crate::config::VENDOR_ID;"))
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

    def test_both_passes_clean_prints_two_summaries(self) -> None:
        code, out, err = self.run_gate()
        self.assertEqual(code, 0, out + err)
        self.assertEqual(
            [line.split(":")[0] for line in out.splitlines()],
            ["sdk layering OK", "rust src audit OK"],
        )


if __name__ == "__main__":
    unittest.main()
