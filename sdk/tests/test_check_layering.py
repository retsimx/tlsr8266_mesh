"""Unit tests for the Cargo DAG, subsystem isolation and ``unsafe`` gates.

Run from the repository root::

    python3 -m unittest discover -s sdk/tests -v

Every test is hermetic: synthetic trees live in a temporary directory and
synthetic Cargo metadata is a dict, so neither cargo nor the network is needed.
The only real trees touched are ``rust/src/`` and the workspace crates
(read-only), by the tests that assert the shipped trees are clean and that the
per-crate ``unsafe`` ceilings equal their measured counts.
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
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "sdk"))

import check_layering  # noqa: E402  (sys.path set up above)

SRC_ROOT = REPO_ROOT / "rust" / "src"
CRATE_ROOT = REPO_ROOT / "rust" / "crates" / "mesh-protocol" / "src"
PAC_ROOT = REPO_ROOT / "rust" / "crates" / "tlsr8266-pac" / "src"
HAL_ROOT = REPO_ROOT / "rust" / "crates" / "tlsr8266-hal" / "src"
RADIO_ROOT = REPO_ROOT / "rust" / "crates" / "tlsr8266-radio" / "src"
PLATFORM_ROOT = REPO_ROOT / "rust" / "crates" / "tlsr8266-platform" / "src"

# The six shipping roots and the Cargo package each one belongs to.
SHIPPING_ROOTS = {
    SRC_ROOT: "tlsr8266_mesh",
    CRATE_ROOT: "mesh-protocol",
    PAC_ROOT: "tlsr8266-pac",
    HAL_ROOT: "tlsr8266-hal",
    RADIO_ROOT: "tlsr8266-radio",
    PLATFORM_ROOT: "tlsr8266-platform",
}


def _write(path: Path, content: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    return path


def _lines(*lines: str) -> str:
    return "\n".join(lines) + "\n"


def _cargo_dependency(name: str, kind: str | None = None) -> dict:
    dependency = {"name": name, "source": None, "req": "*", "kind": kind}
    if kind is None:
        dependency["path"] = f"/repo/rust/crates/{name}"
    return dependency


def _cargo_package(name: str, dependencies: list[dict]) -> dict:
    return {
        "name": name,
        "version": "0.1.0",
        "id": f"path+file:///repo/rust#{name}@0.1.0",
        "dependencies": dependencies,
    }


def _shipping_metadata() -> dict:
    """``cargo metadata --no-deps`` for the real workspace, as a dict."""
    return {
        "packages": [
            _cargo_package(
                "tlsr8266_mesh",
                [
                    _cargo_dependency("mesh-protocol"),
                    _cargo_dependency("tlsr8266-pac"),
                    _cargo_dependency("tlsr8266-hal"),
                    _cargo_dependency("tlsr8266-radio"),
                    _cargo_dependency("tlsr8266-platform"),
                    _cargo_dependency("embassy-time"),
                    _cargo_dependency("critical-section"),
                    _cargo_dependency("futures", "dev"),
                ],
            ),
            _cargo_package(
                "mesh-protocol", [_cargo_dependency("bytemuck")]
            ),
            _cargo_package(
                "tlsr8266-pac",
                [_cargo_dependency("bitflags"), _cargo_dependency("paste")],
            ),
            _cargo_package(
                "tlsr8266-hal",
                [
                    _cargo_dependency("tlsr8266-pac"),
                    _cargo_dependency("mesh-protocol"),
                    _cargo_dependency("critical-section"),
                    _cargo_dependency("embassy-executor"),
                ],
            ),
            _cargo_package(
                "tlsr8266-radio",
                [
                    _cargo_dependency("tlsr8266-pac"),
                    _cargo_dependency("tlsr8266-hal"),
                    _cargo_dependency("mesh-protocol"),
                    _cargo_dependency("heapless"),
                ],
            ),
            _cargo_package(
                "tlsr8266-platform",
                [
                    _cargo_dependency("tlsr8266-hal"),
                    _cargo_dependency("tlsr8266-radio"),
                    _cargo_dependency("mesh-protocol"),
                    _cargo_dependency("critical-section"),
                    _cargo_dependency("embassy-executor"),
                    _cargo_dependency("heapless"),
                ],
            ),
        ],
        "workspace_members": [
            f"path+file:///repo/rust#{name}@0.1.0"
            for name in check_layering.CARGO_DAG
        ],
        "version": 1,
    }


def _metadata_with(package: str, *dependencies: dict) -> dict:
    """The shipping metadata with ``dependencies`` added to ``package``."""
    metadata = _shipping_metadata()
    for entry in metadata["packages"]:
        if entry["name"] == package:
            entry["dependencies"].extend(dependencies)
    return metadata


def _without_package(name: str) -> dict:
    metadata = _shipping_metadata()
    metadata["packages"] = [
        entry for entry in metadata["packages"] if entry["name"] != name
    ]
    return metadata


class TestRealTrees(unittest.TestCase):
    """Every shipped tree must pass every pass, and the CI run must say so."""

    @classmethod
    def setUpClass(cls) -> None:
        # The no-argument run audits ~80 files in six roots, so the two tests
        # that assert on its output share one run rather than paying twice.
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            cls.exit_code = check_layering.main([])
        cls.stdout = out.getvalue()
        cls.stderr = err.getvalue()

    def test_default_src_root_is_the_repo_src(self) -> None:
        self.assertEqual(
            Path(check_layering.DEFAULT_SRC_ROOT).resolve(), SRC_ROOT.resolve()
        )

    def test_default_crate_root_is_the_workspace_crate(self) -> None:
        self.assertEqual(
            Path(check_layering.DEFAULT_CRATE_ROOT).resolve(), CRATE_ROOT.resolve()
        )

    def test_default_crate_roots_cover_every_workspace_crate(self) -> None:
        # C-4 added the HAL, C-10 (#72) the radio crate, and F-5 the platform
        # crate: the no-argument run audits the composition root plus these five.
        roots = [Path(item).resolve() for item in check_layering.DEFAULT_CRATE_ROOTS]
        self.assertEqual(
            roots,
            [
                CRATE_ROOT.resolve(),
                PAC_ROOT.resolve(),
                HAL_ROOT.resolve(),
                RADIO_ROOT.resolve(),
                PLATFORM_ROOT.resolve(),
            ],
        )

    def test_real_trees_are_clean(self) -> None:
        # The no-argument run is what CI executes: the Cargo DAG, the
        # composition root and all five workspace crates, zero violations.
        self.assertEqual(self.exit_code, 0, self.stdout + self.stderr)
        self.assertEqual(self.stdout.count("0 violations"), 7)
        self.assertEqual(self.stderr, "")
        # Every tree is named in the summary, relative to the repo, so a root
        # silently dropped from the default list fails here.
        for root in SHIPPING_ROOTS:
            self.assertIn(str(root.relative_to(REPO_ROOT)), self.stdout)

    def test_real_tree_reports_every_pass(self) -> None:
        lines = self.stdout.splitlines()
        self.assertEqual(len(lines), 7)
        self.assertTrue(lines[0].startswith("cargo dag OK"), lines[0])
        self.assertIn("rust src audit OK", lines[1])
        self.assertIn("mesh-protocol audit OK", self.stdout)
        self.assertIn("tlsr8266-pac audit OK", self.stdout)
        self.assertIn("tlsr8266-hal audit OK", self.stdout)
        self.assertIn("tlsr8266-radio audit OK", self.stdout)
        self.assertIn("tlsr8266-platform audit OK", self.stdout)

    def test_every_real_tree_passes_the_universal_rules(self) -> None:
        # Direct audit of each tree, independent of the CLI summary.
        floors = {
            SRC_ROOT: 15,
            CRATE_ROOT: 5,
            PAC_ROOT: 2,
            HAL_ROOT: 20,
            RADIO_ROOT: 20,
            PLATFORM_ROOT: 5,
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
        # `config`/`version` modules with an injected `RadioConfig`; in a
        # separate crate the application's `crate::version` cannot resolve.
        config = (RADIO_ROOT / "config.rs").read_text(encoding="utf-8")
        self.assertIn("pub struct RadioConfig", config)
        self.assertIn("pub fn set_radio_config", config)
        for path in check_layering.rust_files(str(RADIO_ROOT)):
            text = Path(path).read_text(encoding="utf-8")
            self.assertIsNone(
                re.search(r"crate::version\b", text),
                f"{path} still names the application's version module",
            )

    def test_ratchet_matches_shipping_tree(self) -> None:
        # The one-way half of the ratchet: removing `unsafe` must lower the map,
        # because strict equality with the measured count is asserted here.  The
        # tool itself only fails on an excess, so an improvement mid-refactor
        # does not block the developer — this assertion is what makes the
        # ceiling follow the code down.
        self.assertEqual(
            set(check_layering.MAX_PRODUCTION_UNSAFE_BY_CRATE), set(SHIPPING_ROOTS.values())
        )
        for root, name in SHIPPING_ROOTS.items():
            with self.subTest(crate=name):
                paths = check_layering.rust_files(str(root))
                violations, total = check_layering.audit_universal(
                    paths, str(root), enforce_budget=False
                )
                self.assertEqual(violations, [])
                self.assertEqual(
                    total,
                    check_layering.MAX_PRODUCTION_UNSAFE_BY_CRATE[name],
                    f"{name}: measured {total} production `unsafe` sites; the "
                    "map must equal the measured count",
                )

    def test_root_crate_names_maps_the_firmware_root(self) -> None:
        # `rust/src` is the firmware crate even though its directory is not the
        # package name; every crate root takes its manifest directory's name.
        self.assertEqual(
            check_layering.ROOT_CRATE_NAMES,
            {os.path.normpath(str(SRC_ROOT)): "tlsr8266_mesh"},
        )
        for root, name in SHIPPING_ROOTS.items():
            with self.subTest(crate=name):
                self.assertEqual(check_layering._crate_name(str(root)), name)

    def test_every_shipping_subsystem_is_pure(self) -> None:
        # One row per vertical subsystem: the shipped production code names no
        # layer above it.  The fixtures below pin each rule's behaviour; this
        # asserts the real tree, which is what makes the table a ratchet rather
        # than a migration.
        roots = check_layering.subsystem_roots(str(SRC_ROOT))
        purity = check_layering.subsystem_roots(str(SRC_ROOT))
        for name, subsystem_root in roots.items():
            with self.subTest(subsystem=name):
                # `logging` is a single module file, not a directory.
                paths = (
                    [subsystem_root]
                    if os.path.isfile(subsystem_root)
                    else check_layering.rust_files(subsystem_root)
                )
                self.assertGreaterEqual(len(paths), 1, f"{name} has no files")
                violations, _total = check_layering.audit_universal(
                    paths,
                    str(SRC_ROOT),
                    enforce_budget=False,
                    purity_roots=purity,
                )
                self.assertEqual(violations, [])

    def test_shipping_additional_rules_are_clean(self) -> None:
        # `app-pac` and `logging-macro` are live rules on the real tree, so they
        # are asserted directly here rather than only through the CLI summary
        # (which cannot tell which rule was clean).
        for path in ("app.rs", "irq.rs"):
            text = (SRC_ROOT / path).read_text(encoding="utf-8")
            self.assertIsNone(
                check_layering._PAC_REF_RE.search(
                    check_layering.production_code(text)
                ),
                f"{path} references the PAC in production code",
            )
        for path in check_layering.rust_files(str(PLATFORM_ROOT)):
            text = Path(path).read_text(encoding="utf-8")
            self.assertEqual(
                check_layering.audit_platform_purity_source(text, path),
                [],
                f"{path} contains a forbidden platform term",
            )
        for subsystem in check_layering.LOGGING_MACRO_FORBIDDEN_SUBSYSTEMS:
            root = SRC_ROOT / subsystem
            paths = (
                [str(root)]
                if root.is_file()
                else check_layering.rust_files(str(root))
            )
            for path in paths:
                text = Path(path).read_text(encoding="utf-8")
                self.assertIsNone(
                    check_layering._LOGGING_RE.search(
                        check_layering.production_code(text)
                    ),
                    f"{path} logs from a leaf subsystem",
                )


class _TempSrcTree(unittest.TestCase):
    """A disposable firmware root and workspace crate root per test.

    The gate runs in hermetic mode here: ``--src-root``/``--crate-root`` are
    explicit, so the Cargo DAG pass and the per-crate ratchet are skipped and no
    real tree is read.  A test that names only one of the two roots exercises
    the default for the other, which is what pins the banner's shape.
    """

    # What a hermetic run that names both roots announces on stdout before any
    # summary or violation.  A `--crate-root`-only run names one more skipped
    # pass, which `test_crate_root_alone_audits_only_that_crate` pins.
    HERMETIC_NOTICE = "hermetic mode: cargo DAG and unsafe ratchet skipped"

    def setUp(self) -> None:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = Path(tmp.name)
        self.src_root = self.tmp / "src"
        self.src_root.mkdir(parents=True)
        self.crate_root = self.tmp / "mesh-protocol" / "src"
        self.crate_root.mkdir(parents=True)
        _write(self.src_root / "clean.rs", _lines("pub fn f() {}"))
        _write(self.crate_root / "lib.rs", _lines("pub fn f() {}"))

    def write_src(self, relative: str, content: str) -> Path:
        return _write(self.src_root / relative, content)

    def write_crate(self, relative: str, content: str) -> Path:
        return _write(self.crate_root / relative, content)

    def run_gate(self) -> tuple[int, str, str]:
        """Run the hermetic gate, checking and removing its one-line banner.

        Every test here names a root, so every run is hermetic and says so.
        Asserting the banner here is what keeps it from disappearing — a silent
        hermetic run must fail the whole class — and stripping it lets the
        tests below read the output exactly like the bare CI gate's.
        """
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(
                [
                    "--src-root",
                    str(self.src_root),
                    "--crate-root",
                    str(self.crate_root),
                ]
            )
        stdout = out.getvalue()
        self.assertTrue(stdout.startswith(self.HERMETIC_NOTICE + "\n"), stdout)
        return code, stdout.split("\n", 1)[1], err.getvalue()

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


class TestViolationFormat(_TempSrcTree):
    def test_format_is_file_line_rule_violation(self) -> None:
        path = self.write_src(
            "ota/plain.rs", _lines("fn a() {}", "use crate::app::App;")
        )
        out = self.assert_violations("ota-upward-dep")
        self.assertEqual(len(out.splitlines()), 1)
        self.assertEqual(out.splitlines()[0], f"{path}:2: ota-upward-dep: crate::app")

    def test_one_line_per_violation_in_file_order(self) -> None:
        self.write_src("ota/b.rs", _lines("use crate::mesh::is_online;", 'uprintln!("x");'))
        self.write_src("ota/a.rs", _lines("use crate::app::App;"))
        out = self.assert_violations("ota-upward-dep: crate::app")
        self.assertEqual(
            [line.split(": ", 1)[1] for line in out.splitlines()],
            [
                "ota-upward-dep: crate::app",
                "ota-upward-dep: crate::mesh",
                "logging-macro: uprintln!",
            ],
        )
        self.assertLess(out.index("ota/a.rs"), out.index("ota/b.rs"))


class TestTestCodeStripping(_TempSrcTree):
    """The parser must strip only `#[cfg(test)]` and never truncate a file."""

    def test_inline_cfg_test_item_above_a_violation_is_still_caught(self) -> None:
        path = self.write_src(
            "ota/trap.rs",
            _lines(
                "use crate::config::VENDOR_ID;",      # 1
                "",                                    # 2
                "#[cfg(test)]",                        # 3
                "pub static mut SCRATCH: u32 = 0;",    # 4
                "",                                    # 5
                "pub fn production() {}",              # 6
                "",                                    # 7
                "use crate::app::App;",                # 8
            ),
        )
        out = self.assert_violations(f"{path}:8: ota-upward-dep: crate::app")
        self.assertEqual(len(out.splitlines()), 1)  # no static-mut from the mask

    def test_inline_cfg_test_brace_item_above_a_violation_is_still_caught(self) -> None:
        self.write_src(
            "ota/trap.rs",
            _lines(
                "#[cfg(test)]",
                "fn helper() {",
                "    let x = 0;",
                "}",
                "",
                "pub fn production() { let _ = crate::logging::UartStream; }",
            ),
        )
        self.assert_violations("6: ota-upward-dep: crate::logging")

    def test_cfg_test_module_is_exempt(self) -> None:
        self.write_src(
            "ota/clean.rs",
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
        self.write_src(
            "ota/clean.rs",
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
        out = self.assert_violations("10: ota-upward-dep: crate::mesh")
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
        self.write_src(
            "ota/trap.rs",
            _lines(
                "#[cfg(not(test))]",
                "pub fn real() {}",
                "",
                "#[cfg_attr(test, mry::mry)]",
                "pub fn mocked() {}",
                "",
                "pub fn bad() { let _ = crate::light::VENDOR_ID; }",
            ),
        )
        self.assert_violations("7: ota-upward-dep: crate::light")

    def test_cfg_test_on_a_struct_field_does_not_mask_the_next_item(self) -> None:
        # H-1: the field attribute used to run to the next brace block, hiding
        # the production item that followed it.
        self.write_src(
            "ota/trap.rs",
            _lines(
                "pub struct S {",
                "    #[cfg(test)]",
                "    locked: u8,",
                "}",
                "",
                "pub fn production() { let _ = crate::app::App; }",
            ),
        )
        self.assert_violations("6: ota-upward-dep: crate::app")

    def test_cfg_test_on_a_statement_does_not_mask_the_rest_of_the_body(self) -> None:
        # Same root cause, statement shape.
        self.write_src(
            "ota/trap.rs",
            _lines(
                "pub fn f() {",
                "    #[cfg(test)]",
                "    let scratch = 0;",
                "    let _ = crate::uart::send_message();",
                "    loop {",
                "        break;",
                "    }",
                "}",
            ),
        )
        self.assert_violations("4: ota-upward-dep: crate::uart")


class TestTestStrippingExtent(unittest.TestCase):
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


class TestCommentAndStringPolicy(_TempSrcTree):
    """Comments and strings are prose, not dependencies: they never count."""

    def test_banned_path_in_line_comment_is_ignored(self) -> None:
        self.write_src("ota/x.rs", _lines("// historically: use crate::app::App;"))
        self.assert_clean()

    def test_banned_path_in_block_comment_is_ignored(self) -> None:
        self.write_src("ota/x.rs", _lines("/* crate::mesh::is_online */", "pub fn f() {}"))
        self.assert_clean()

    def test_banned_path_in_string_literal_is_ignored(self) -> None:
        self.write_src("ota/x.rs", _lines('pub const S: &str = "crate::app";'))
        self.assert_clean()

    def test_unbalanced_brace_in_comment_does_not_break_the_parser(self) -> None:
        self.write_src(
            "ota/x.rs",
            _lines(
                "// if (flag) {",
                "pub fn f() { let _ = crate::app::App::default(); }",
            ),
        )
        self.assert_violations("2: ota-upward-dep: crate::app")


class TestPathBypasses(unittest.TestCase):
    """Relative and aliased paths must not be able to dodge the `crate::` rules."""

    def _audit(self, *lines: str, module: tuple[str, ...] = ()) -> list:
        violations, _total = check_layering.audit_universal_source(
            _lines(*lines), "x.rs", module=module
        )
        return violations

    def test_super_chain_at_the_root_is_rejected(self) -> None:
        violations = self._audit("use super::super::app::app;", module=())
        self.assertEqual(
            [(item.rule, item.violation) for item in violations],
            [
                (
                    "super-escape",
                    "super::2 past the audited root (chain of 2, depth 0)",
                )
            ],
        )

    def test_super_chain_deeper_than_the_module_depth_is_rejected(self) -> None:
        violations = self._audit(
            "use super::super::super::app::app;", module=("ota", "fsm")
        )
        self.assertEqual([item.rule for item in violations], ["super-escape"])

    def test_super_chain_inside_the_audited_root_is_legal(self) -> None:
        self.assertEqual(
            self._audit(
                "use super::mesh_management::mesh_send_online_status;",
                "pub fn f() { super::super::queues::enqueue_mesh_rx(&PKT); }",
                module=("radio", "ble"),
            ),
            [],
        )

    def test_sibling_super_chains_do_not_stack(self) -> None:
        # `super::{super::a, super::b}` is two hops, not three: the deepest
        # sibling wins, so it fits inside a two-segment module path.
        self.assertEqual(
            self._audit(
                "use super::{super::queues, super::mesh_management};",
                module=("radio", "ble"),
            ),
            [],
        )

    def test_grouped_super_chain_that_escapes_is_rejected(self) -> None:
        # The outer chain composes with the group (two hops) and the inner
        # `super::` is a chain of its own (one hop); at the crate root both
        # escape.
        violations = self._audit("use super::{super::app::app};", module=())
        self.assertEqual(
            [item.rule for item in violations], ["super-escape", "super-escape"]
        )
        self.assertIn(
            "super::2 past the audited root (chain of 2, depth 0)",
            [item.violation for item in violations],
        )

    def test_crate_alias_is_rejected(self) -> None:
        violations = self._audit(
            "use crate as krate;", "pub fn f() { let _ = krate::app::app(); }"
        )
        self.assertEqual(
            [(item.line, item.rule, item.violation) for item in violations],
            [(1, "crate-alias", "crate as krate")],
        )

    def test_grouped_crate_alias_is_rejected(self) -> None:
        violations = self._audit("use crate::{self as krate};")
        self.assertEqual(
            [(item.rule, item.violation) for item in violations],
            [("crate-alias", "crate::{self as krate}")],
        )

    def test_extern_crate_self_alias_is_rejected(self) -> None:
        # `extern crate self as krate;` aliases the crate root like
        # `use crate as krate;` but is not a `use` statement.  The declaration
        # alone is the violation: it is what lets `krate::app::..` dodge the
        # path rules, whether or not this file happens to use it.
        violations = self._audit("extern crate self as krate;")
        self.assertEqual(
            [(item.line, item.rule, item.violation) for item in violations],
            [(1, "crate-alias", "extern crate self as krate")],
        )

    def test_extern_crate_self_alias_with_qualified_use_is_rejected(self) -> None:
        violations = self._audit(
            "extern crate self as krate;",
            "pub fn f() { krate::app::app(); }",
        )
        self.assertEqual(
            [item.violation for item in violations],
            ["extern crate self as krate"],
        )

    def test_plain_extern_crate_declarations_are_legal(self) -> None:
        # Control for the matcher above: the general `extern crate <name>;`
        # form is not a crate-root alias.  `extern crate core;` compiles in a
        # `#![no_std]` crate with no real dependency (verified with rustc); it
        # can only warn as an unused extern crate.
        self.assertEqual(
            self._audit(
                "extern crate core;",
                "pub fn f() { core::mem::size_of::<u32>(); }",
            ),
            [],
        )
        for statement in ("extern crate core;", "extern crate std;", "extern crate alloc;"):
            self.assertIsNone(
                check_layering._EXTERN_CRATE_SELF_RE.search(statement), statement
            )

    def test_root_crate_file_may_use_one_super(self) -> None:
        # Module depth, not directory depth: a file directly under the root is
        # one `mod` below the crate root, so one `super::` stays inside the
        # audited tree.  This is the shape `tlsr8266-radio/src/callbacks.rs`
        # ships.
        self.assertEqual(
            self._audit(
                "pub mod test_ota { use super::OtaHooks; }", module=("callbacks",)
            ),
            [],
        )


class TestModuleSegments(unittest.TestCase):
    """`_module_segments` is the one file -> module mapping both rules share."""

    def test_crate_root_files_are_the_empty_path(self) -> None:
        for name in ("lib.rs", "main.rs"):
            with self.subTest(name=name):
                self.assertEqual(check_layering._module_segments(name, "."), ())

    def test_a_mod_file_is_its_directory(self) -> None:
        self.assertEqual(
            check_layering._module_segments(os.path.join("light", "mod.rs"), "."),
            ("light",),
        )
        self.assertEqual(
            check_layering._module_segments(
                os.path.join("light", "sub", "mod.rs"), "."
            ),
            ("light", "sub"),
        )

    def test_any_other_file_is_one_mod_deeper_than_its_sibling(self) -> None:
        # This is the M-2 fix: `ota/fsm.rs` may spend two `super::` hops, the
        # same count as `ota/mod.rs`'s one hop plus the `fsm` child.
        self.assertEqual(
            check_layering._module_segments(os.path.join("ota", "fsm.rs"), "."),
            ("ota", "fsm"),
        )
        self.assertEqual(
            check_layering._module_segments(
                os.path.join("ota", "sub", "chunk.rs"), "."
            ),
            ("ota", "sub", "chunk"),
        )
        self.assertEqual(
            check_layering._module_segments("app.rs", "."), ("app",)
        )


class TestCli(_TempSrcTree):
    def test_missing_src_root_exits_2(self) -> None:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(
                [
                    "--src-root",
                    str(self.tmp / "absent"),
                    "--crate-root",
                    str(self.crate_root),
                ]
            )
        self.assertEqual(code, 2)
        self.assertIn("not a directory", err.getvalue())

    def test_empty_src_root_exits_2(self) -> None:
        empty = self.tmp / "empty"
        empty.mkdir()
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(
                ["--src-root", str(empty), "--crate-root", str(self.crate_root)]
            )
        self.assertEqual(code, 2)
        self.assertIn("no .rs files", err.getvalue())

    def test_src_root_is_resolved_from_any_working_directory(self) -> None:
        self.write_src("ota/x.rs", _lines("use crate::app::App;"))
        elsewhere = self.tmp / "elsewhere"
        elsewhere.mkdir()
        out, err = io.StringIO(), io.StringIO()
        with contextlib.chdir(elsewhere):
            with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                code = check_layering.main(
                    ["--src-root", "../src", "--crate-root", "../mesh-protocol/src"]
                )
        self.assertEqual(code, 1, out.getvalue() + err.getvalue())
        # The hermetic banner is one line on stdout; the violations follow it.
        self.assertTrue(out.getvalue().startswith(self.HERMETIC_NOTICE + "\n"))
        self.assertTrue(
            out.getvalue().startswith(
                f"{self.HERMETIC_NOTICE}\n{self.src_root / 'ota' / 'x.rs'}:1: "
            )
        )
        self.assertTrue(re.search(r":1: ota-upward-dep: crate::app$", out.getvalue()))

    def test_stray_non_rust_files_are_ignored(self) -> None:
        self.write_src("notes.txt", "use crate::app::App;\n")
        self.assert_clean()

    def test_explicit_src_root_skips_the_cargo_dag_pass(self) -> None:
        # Hermeticity: a fixture tree has no Cargo.toml, so the DAG pass and the
        # ratchet must not run against the real workspace.
        code, out, err = self.run_gate()
        self.assertEqual(code, 0, out + err)
        self.assertNotIn("cargo dag OK", out)
        self.assertNotIn("unsafe-budget", out)

    def test_hermetic_run_announces_the_skipped_passes(self) -> None:
        # `run_gate` checks and strips the banner for every other test, so the
        # text is pinned here through a raw `main` call.  A copy-pasted
        # `--src-root rust/src` must not look like the full gate.
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(
                [
                    "--src-root",
                    str(self.src_root),
                    "--crate-root",
                    str(self.crate_root),
                ]
            )
        self.assertEqual(code, 0, out.getvalue() + err.getvalue())
        self.assertEqual(
            out.getvalue().splitlines()[0],
            "hermetic mode: cargo DAG and unsafe ratchet skipped",
        )
        self.assertNotIn("cargo dag OK", out.getvalue())
        # Both roots were named, so the firmware subsystem pass did run: the
        # banner above must not claim otherwise.
        self.assertIn("rust src audit OK", out.getvalue())

    def test_src_root_only_banner_names_the_two_skipped_passes(self) -> None:
        # `--src-root` alone still runs the firmware pass, so the banner names
        # only the DAG and the ratchet, and a subsystem violation is reported.
        self.write_src("ota/mod.rs", _lines("use crate::app::App;"))
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(["--src-root", str(self.src_root)])
        self.assertEqual(code, 1, out.getvalue() + err.getvalue())
        lines = out.getvalue().splitlines()
        self.assertEqual(
            lines[0], "hermetic mode: cargo DAG and unsafe ratchet skipped"
        )
        self.assertIn("ota-upward-dep: crate::app", lines[1])


class TestCargoDag(unittest.TestCase):
    """`audit_cargo_dag` is a pure function over `cargo metadata` JSON."""

    def test_real_shaped_metadata_passes(self) -> None:
        self.assertEqual(check_layering.audit_cargo_dag(_shipping_metadata()), [])

    def test_every_dag_violation_uses_the_cargo_dag_rule(self) -> None:
        metadata = _metadata_with(
            "tlsr8266-pac", _cargo_dependency("tlsr8266-hal")
        )
        violations = check_layering.audit_cargo_dag(metadata)
        self.assertTrue(violations)
        self.assertEqual({item.rule for item in violations}, {"cargo-dag"})
        self.assertEqual(
            {item.path for item in violations},
            {check_layering.CARGO_MANIFEST_LABEL},
        )
        # An edge is a manifest property, so it has no source line and must not
        # render a placeholder one.
        for item in violations:
            self.assertNotIn(":0:", item.render())
            self.assertTrue(
                item.render().startswith(
                    f"{check_layering.CARGO_MANIFEST_LABEL}: cargo-dag: "
                ),
                item.render(),
            )

    def test_illegal_workspace_edge_fails(self) -> None:
        # The PAC is the register leaf: it may not depend on the HAL above it.
        violations = check_layering.audit_cargo_dag(
            _metadata_with("tlsr8266-pac", _cargo_dependency("tlsr8266-hal"))
        )
        self.assertIn("tlsr8266-hal", " ".join(item.violation for item in violations))

    def test_platform_crate_may_not_reach_the_pac_or_firmware(self) -> None:
        pac_violations = check_layering.audit_cargo_dag(
            _metadata_with("tlsr8266-platform", _cargo_dependency("tlsr8266-pac"))
        )
        self.assertTrue(
            any("tlsr8266-pac" in item.violation for item in pac_violations),
            [item.violation for item in pac_violations],
        )
        firmware_violations = check_layering.audit_cargo_dag(
            _metadata_with("tlsr8266-platform", _cargo_dependency("tlsr8266_mesh"))
        )
        self.assertTrue(
            any("firmware crate" in item.violation for item in firmware_violations),
            [item.violation for item in firmware_violations],
        )

    def test_library_crate_depending_on_the_firmware_crate_fails(self) -> None:
        violations = check_layering.audit_cargo_dag(
            _metadata_with("tlsr8266-pac", _cargo_dependency("tlsr8266_mesh"))
        )
        # The edge also closes a cycle, so more than one message is expected;
        # the firmware-crate one is the invariant this test pins.
        self.assertTrue(
            any("firmware crate" in item.violation for item in violations),
            [item.violation for item in violations],
        )

    def test_mry_fails_in_every_dependency_kind(self) -> None:
        for kind in (None, "dev", "build"):
            with self.subTest(kind=kind):
                violations = check_layering.audit_cargo_dag(
                    _metadata_with("tlsr8266-hal", _cargo_dependency("mry", kind))
                )
                self.assertTrue(
                    any("banned in every crate" in item.violation for item in violations),
                    [item.violation for item in violations],
                )

    def test_unlisted_external_dependency_fails(self) -> None:
        violations = check_layering.audit_cargo_dag(
            _metadata_with("tlsr8266-pac", _cargo_dependency("serde"))
        )
        self.assertTrue(
            any("serde" in item.violation for item in violations),
            [item.violation for item in violations],
        )

    def test_unlisted_dev_dependency_fails(self) -> None:
        violations = check_layering.audit_cargo_dag(
            _metadata_with("mesh-protocol", _cargo_dependency("criterion", "dev"))
        )
        self.assertTrue(
            any("dev-dependency" in item.violation for item in violations),
            [item.violation for item in violations],
        )

    def test_unlisted_build_dependency_fails_as_non_dev(self) -> None:
        # Build-dependencies are external non-dev deps: a row with a non-dev
        # ceiling must list them, and a row with `None` has no external ceiling.
        violations = check_layering.audit_cargo_dag(
            _metadata_with("mesh-protocol", _cargo_dependency("cc", "build"))
        )
        self.assertEqual(len(violations), 1)
        self.assertIn("as a dependency", violations[0].violation)

    def test_missing_workspace_member_fails(self) -> None:
        violations = check_layering.audit_cargo_dag(_without_package("tlsr8266-hal"))
        self.assertTrue(
            any("is missing from the workspace" in item.violation for item in violations),
            [item.violation for item in violations],
        )

    def test_extra_workspace_member_fails(self) -> None:
        metadata = _shipping_metadata()
        metadata["packages"].append(_cargo_package("tlsr8266-extra", []))
        violations = check_layering.audit_cargo_dag(metadata)
        self.assertTrue(
            any("is not in CARGO_DAG" in item.violation for item in violations),
            [item.violation for item in violations],
        )

    def test_cyclic_workspace_graph_fails(self) -> None:
        metadata = _metadata_with(
            "mesh-protocol", _cargo_dependency("tlsr8266-hal")
        )
        violations = check_layering.audit_cargo_dag(metadata)
        self.assertTrue(
            any("cycle" in item.violation for item in violations),
            [item.violation for item in violations],
        )

    def test_empty_metadata_reports_every_missing_member(self) -> None:
        violations = check_layering.audit_cargo_dag({})
        self.assertEqual(len(violations), len(check_layering.CARGO_DAG))
        self.assertTrue(all("missing" in item.violation for item in violations))

    def test_cargo_failure_exits_2_and_says_why(self) -> None:
        # Fail closed: a DAG pass that cannot run is exit 2, never a silent OK.
        out, err = io.StringIO(), io.StringIO()
        with mock.patch.object(
            check_layering,
            "load_cargo_metadata",
            side_effect=check_layering.GateError("cargo metadata failed (exit 127)"),
        ):
            with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                code = check_layering.main([])
        self.assertEqual(code, 2, out.getvalue() + err.getvalue())
        self.assertIn("cargo metadata failed (exit 127)", err.getvalue())
        self.assertNotIn("cargo dag OK", out.getvalue())

    def test_dag_violation_fails_the_no_argument_run(self) -> None:
        metadata = _metadata_with("tlsr8266-pac", _cargo_dependency("mry"))
        out, err = io.StringIO(), io.StringIO()
        with mock.patch.object(
            check_layering, "load_cargo_metadata", return_value=metadata
        ):
            with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                code = check_layering.main([])
        self.assertEqual(code, 1, out.getvalue() + err.getvalue())
        self.assertIn("cargo-dag", out.getvalue())
        self.assertIn("cargo dag FAILED: 1 violation(s)", err.getvalue())

    def test_the_live_metadata_matches_the_table(self) -> None:
        # Integration with the real tool: the shipping workspace validates, and
        # its members are exactly CARGO_DAG's keys.
        metadata = check_layering.load_cargo_metadata(str(REPO_ROOT))
        self.assertEqual(
            {package["name"] for package in metadata["packages"]},
            set(check_layering.CARGO_DAG),
        )
        self.assertEqual(check_layering.audit_cargo_dag(metadata), [])


class TestPlatformPurityRule(unittest.TestCase):
    """The platform crate stays product-neutral and platform-only."""

    def test_forbidden_terms_are_rejected(self) -> None:
        text = _lines(
            "use tlsr8266_pac::write_reg8;",
            "use tlsr8266_mesh::light::decode;",
            "pub struct LightProduct;",
            "pub const LGT_CMD_SET_LIGHT: u8 = 1;",
        )
        violations = check_layering.audit_platform_purity_source(text, "rust/crates/tlsr8266-platform/src/lib.rs")
        self.assertTrue(violations)
        self.assertTrue(all(item.rule == "platform-purity" for item in violations))

    def test_pure_contract_code_is_clean(self) -> None:
        text = _lines(
            "pub trait Product: 'static {}",
            "pub struct Platform<P>(core::marker::PhantomData<P>);",
        )
        violations = check_layering.audit_platform_purity_source(text, "rust/crates/tlsr8266-platform/src/api.rs")
        self.assertEqual(violations, [])


class TestOtaPurityRule(_TempSrcTree):
    """`rust/src/ota/**` production code may not name the layers above it."""

    def test_sibling_import_is_rejected(self) -> None:
        path = self.write_src(
            "ota/mod.rs", _lines("use crate::app::set_ota_hooks;")
        )
        out = self.assert_violations("1: ota-upward-dep: crate::app")
        self.assertEqual(out.splitlines()[0], f"{path}:1: ota-upward-dep: crate::app")

    def test_qualified_light_path_is_rejected(self) -> None:
        # Not an import: the rule reads every `crate::` reference, so a
        # fully-qualified call cannot dodge it.
        self.write_src(
            "ota/mod.rs",
            _lines("pub fn f() { crate::light::set_brightness(1); }"),
        )
        self.assert_violations("1: ota-upward-dep: crate::light")

    def test_tlsr8266_pac_import_is_rejected(self) -> None:
        self.write_src("ota/mod.rs", _lines("use tlsr8266_pac::fake_mmio::FakeMmio;"))
        self.assert_violations("1: ota-forbidden-crate: tlsr8266_pac")

    def test_tlsr8266_radio_import_is_rejected(self) -> None:
        self.write_src("ota/mod.rs", _lines("use tlsr8266_radio::rf::init;"))
        self.assert_violations("1: ota-forbidden-crate: tlsr8266_radio")

    def test_a_pac_call_without_an_import_is_rejected(self) -> None:
        # The row bans the crate root, not the `use` statement: a fully
        # qualified call reaches the same registers.
        self.write_src(
            "ota/mod.rs",
            _lines("pub fn poke() { tlsr8266_pac::write_reg8(0x8000, 1); }"),
        )
        self.assert_violations("1: ota-forbidden-crate: tlsr8266_pac")

    def test_a_use_inside_a_block_is_rejected(self) -> None:
        # `_IMPORT_RE` is line-anchored, so a `use` after `{` used to be
        # invisible; the reference scan does not care where the name sits.
        self.write_src(
            "ota/mod.rs",
            _lines("pub fn poke() { use tlsr8266_pac::gpio; gpio::x(); }"),
        )
        self.assert_violations("1: ota-forbidden-crate: tlsr8266_pac")

    def test_two_supers_from_a_child_module_reach_the_crate_root(self) -> None:
        # `ota/fsm.rs` is `ota::fsm`, so two hops still land on the crate root;
        # this is legal Rust the gate used to reject.
        self.write_src("ota/fsm.rs", _lines("use super::super::config::VENDOR_ID;"))
        self.assert_clean()

    def test_every_forbidden_module_is_rejected(self) -> None:
        banned = sorted(check_layering.SUBSYSTEM_PURITY["ota"][0])
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
                "    use crate::app::set_ota_hooks;",
                "    use tlsr8266_pac::fake_mmio::FakeMmio;",
                "}",
            ),
        )
        self.assert_clean()

    def test_logging_macro_is_rejected(self) -> None:
        self.write_src(
            "ota/fsm.rs",
            _lines('pub fn f() { uprintln!("ota"); }'),
        )
        self.assert_violations("1: logging-macro: uprintln!")

    def test_a_nested_module_still_belongs_to_the_row(self) -> None:
        # A subdirectory must not let a module escape its subsystem's row.
        self.write_src(
            "ota/sub/chunk.rs", _lines("use crate::app::set_ota_hooks;")
        )
        self.assert_violations("1: ota-upward-dep: crate::app")

    def test_rule_is_scoped_to_the_ota_directory(self) -> None:
        # Control: the same import one directory up is production code but not
        # `ota` code, so the purity and logging rules leave it alone.
        self.write_src("config.rs", _lines("use tlsr8266_pac::write_reg8;"))
        self.assert_clean()


class TestFactoryResetPurityRule(_TempSrcTree):
    """`rust/src/factory_reset/**` production code may not name the layers above."""

    def test_sibling_import_is_rejected(self) -> None:
        path = self.write_src(
            "factory_reset/mod.rs",
            _lines("use crate::app::reboot_into_pairing;"),
        )
        out = self.assert_violations("1: factory_reset-upward-dep: crate::app")
        self.assertEqual(
            out.splitlines()[0], f"{path}:1: factory_reset-upward-dep: crate::app"
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
        banned = sorted(check_layering.SUBSYSTEM_PURITY["factory_reset"][0])
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
        self.write_src(
            "factory_reset/mod.rs",
            _lines(
                "pub fn production() {}",
                "",
                "#[cfg(test)]",
                "mod tests {",
                "    use crate::app::reboot_into_pairing;",
                "    use tlsr8266_pac::fake_mmio::FakeMmio;",
                "}",
            ),
        )
        self.assert_clean()


class TestLightPurityRule(_TempSrcTree):
    """`rust/src/light/**` production code may not name the layers above it."""

    def test_uart_import_is_rejected(self) -> None:
        path = self.write_src(
            "light/mod.rs", _lines("use crate::uart::send_message;")
        )
        out = self.assert_violations("1: light-upward-dep: crate::uart")
        self.assertEqual(
            out.splitlines()[0], f"{path}:1: light-upward-dep: crate::uart"
        )

    def test_super_import_is_the_same_upward_dep(self) -> None:
        # `super::uart` from `light/mod.rs` *is* `crate::uart`, and the report
        # names the resolved module, not the spelling that reached it.
        self.write_src("light/mod.rs", _lines("use super::uart::started;"))
        out = self.assert_violations("1: light-upward-dep: crate::uart")
        self.assertNotIn("super::", out)

    def test_super_call_without_a_use_is_the_same_upward_dep(self) -> None:
        # The relative spelling is an expression path here, not an import: the
        # resolution reads every `super::` chain, so `super::uart::send()`
        # cannot dodge the rule by skipping the `use`.
        path = self.write_src(
            "light/mod.rs", _lines("pub fn f() { super::uart::send(); }")
        )
        out = self.assert_violations("1: light-upward-dep: crate::uart")
        self.assertEqual(
            out.splitlines(), [f"{path}:1: light-upward-dep: crate::uart"]
        )

    def test_super_use_and_super_call_report_each_occurrence_once(self) -> None:
        # Both spellings resolve to the same name, and each occurrence gets
        # exactly one violation: never a duplicate, never a silent second form.
        path = self.write_src(
            "light/mod.rs",
            _lines(
                "use super::uart::started;",
                "pub fn f() { super::uart::send(); }",
            ),
        )
        out = self.assert_violations("1: light-upward-dep", "2: light-upward-dep")
        self.assertEqual(
            out.splitlines(),
            [
                f"{path}:1: light-upward-dep: crate::uart",
                f"{path}:2: light-upward-dep: crate::uart",
            ],
        )

    def test_a_super_chain_inside_a_use_group_is_reported_once(self) -> None:
        # `iter_imports` composes the brace group and `super-escape` judges the
        # composed chain; the expression-path scan must stay out of the use
        # tree and not re-read the same chain as a second finding.
        self.write_src(
            "light/mod.rs",
            _lines("use super::{", "    super::uart::started,", "};"),
        )
        out = self.assert_violations("super-escape")
        self.assertEqual(len(out.splitlines()), 1)

    def test_inline_mod_super_chain_is_judged_by_the_file_scope(self) -> None:
        # Known limitation, pinned: the `super` model is per file, so a chain
        # inside an inline `mod` is resolved against `light/mod.rs` rather than
        # `light::inner`.  It fails closed (reported, never silent), and no
        # shipped subsystem has a production inline `mod`.
        self.write_src(
            "light/mod.rs",
            _lines("mod inner { pub fn f() { super::uart::send(); } }"),
        )
        self.assert_violations("1: light-upward-dep: crate::uart")

    def test_crate_root_glob_is_rejected(self) -> None:
        # `use crate::*;` hands the whole root to the subsystem, so even a
        # qualified `uart::send()` becomes reachable without a path the row can
        # see.
        self.write_src(
            "light/mod.rs",
            _lines("use crate::*;", "pub fn f() { uart::send_message(); }"),
        )
        self.assert_violations("1: light-glob-import: crate::*")

    def test_super_glob_that_reaches_the_crate_root_is_rejected(self) -> None:
        # One hop from `light/mod.rs` lands on the crate root, so this is the
        # same glob as `use crate::*;` spelled relatively.
        self.write_src("light/mod.rs", _lines("use super::*;"))
        self.assert_violations("1: light-glob-import: crate::*")

    def test_glob_of_a_deeper_module_is_accepted(self) -> None:
        # A glob of the subsystem's own module reaches nothing above it.
        self.write_src(
            "light/mod.rs",
            _lines("use crate::light::*;", "use super::transitions::*;"),
        )
        self.assert_clean()

    def test_leading_colons_pac_path_is_rejected(self) -> None:
        # The absolute-path form is not a `use crate::` import and used to slip
        # past the import-only scan.
        self.write_src("light/mod.rs", _lines("use ::tlsr8266_pac::BIT;"))
        self.assert_violations("1: light-forbidden-crate: tlsr8266_pac")

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

    def test_tlsr8266_radio_import_is_rejected(self) -> None:
        self.write_src("light/transitions.rs", _lines("use tlsr8266_radio::rf::init;"))
        self.assert_violations("1: light-forbidden-crate: tlsr8266_radio")

    def test_every_forbidden_module_is_rejected(self) -> None:
        banned = sorted(check_layering.SUBSYSTEM_PURITY["light"][0])
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

    def test_logging_is_allowed_because_the_row_omits_it(self) -> None:
        # The lamp publishes state through a hook, but `crate::logging` is not
        # in the light row, so naming it is not an inversion.
        self.write_src(
            "light/effects.rs",
            _lines("use crate::logging::UartStream;", 'pub fn f() { uprintln!("l"); }'),
        )
        self.assert_clean()

    def test_forbidden_import_inside_cfg_test_is_accepted(self) -> None:
        self.write_src(
            "light/effects.rs",
            _lines(
                "pub fn production() {}",
                "",
                "#[cfg(test)]",
                "mod tests {",
                "    use crate::uart::reset_for_test;",
                "    use tlsr8266_pac::fake_mmio::FakeMmio;",
                "}",
            ),
        )
        self.assert_clean()


class TestDiagnosticsPurityRule(_TempSrcTree):
    """`rust/src/diagnostics/**` production code may not name the layers above."""

    def test_logging_import_is_rejected(self) -> None:
        path = self.write_src(
            "diagnostics/mod.rs", _lines("use crate::logging::UartStream;")
        )
        out = self.assert_violations("1: diagnostics-upward-dep: crate::logging")
        self.assertEqual(
            out.splitlines()[0], f"{path}:1: diagnostics-upward-dep: crate::logging"
        )

    def test_super_logging_import_is_rejected(self) -> None:
        # `diagnostics/mod.rs` sits directly under the crate root, so
        # `super::logging` is the sibling it may not name.
        self.write_src(
            "diagnostics/mod.rs", _lines("use super::logging::LOG_LEVEL_STATE;")
        )
        self.assert_violations("1: diagnostics-upward-dep: crate::logging")

    def test_qualified_uart_path_is_rejected(self) -> None:
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
        banned = sorted(check_layering.SUBSYSTEM_PURITY["diagnostics"][0])
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

    def test_logging_macro_is_rejected(self) -> None:
        self.write_src(
            "diagnostics/rtt.rs",
            _lines('pub fn f() { uprint!("d"); }'),
        )
        self.assert_violations("1: logging-macro: uprint!")

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
    """`rust/src/logging.rs` production code may not name its siblings."""

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
        banned = sorted(check_layering.SUBSYSTEM_PURITY["logging"][0])
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
        self.write_src("logging.rs", _lines("use tlsr8266_pac::write_reg8;"))
        self.assert_violations("1: logging-forbidden-crate: tlsr8266_pac")

    def test_rule_does_not_apply_to_an_app_root_file(self) -> None:
        # Control: the override is an exact path, not "every root-level file".
        self.write_src("config.rs", _lines("use crate::uart::send_sync_panic;"))
        self.assert_clean()


class TestUartPurityRule(_TempSrcTree):
    """`rust/src/uart/**` production code may not name `mesh` or a peer."""

    def test_mesh_import_is_rejected(self) -> None:
        path = self.write_src(
            "uart/mod.rs", _lines("use crate::mesh::send_mesh_message;")
        )
        out = self.assert_violations("1: uart-upward-dep: crate::mesh")
        self.assertEqual(
            out.splitlines()[0], f"{path}:1: uart-upward-dep: crate::mesh"
        )

    def test_light_import_is_rejected(self) -> None:
        self.write_src(
            "uart/protocol.rs",
            _lines("use crate::light::transitions::LIGHT_HW;"),
        )
        self.assert_violations("1: uart-upward-dep: crate::light")

    def test_super_mesh_import_is_rejected(self) -> None:
        self.write_src("uart/mod.rs", _lines("use super::mesh::is_online;"))
        self.assert_violations("1: uart-upward-dep: crate::mesh")

    def test_crate_root_glob_is_rejected(self) -> None:
        self.write_src(
            "uart/mod.rs",
            _lines("use crate::*;", "pub fn f() { mesh::is_online(); }"),
        )
        self.assert_violations("1: uart-glob-import: crate::*")

    def test_crate_root_glob_inside_a_function_body_is_rejected(self) -> None:
        # A `use` may begin a statement, not only a line: a block-local glob is
        # the same crate-root glob and used to be invisible to the import scan.
        self.write_src(
            "uart/mod.rs",
            _lines("pub fn f() { use crate::*; mesh::is_online(); }"),
        )
        self.assert_violations("1: uart-glob-import: crate::*")

    def test_crate_root_glob_after_a_statement_is_rejected(self) -> None:
        # The second statement position: after a `;`, so the glob starts
        # mid-line rather than right after the block's `{`.
        self.write_src(
            "uart/mod.rs",
            _lines("pub fn f() { let _ = 1; use super::*; }"),
        )
        self.assert_violations("1: uart-glob-import: crate::*")

    def test_glob_of_a_deeper_module_inside_a_function_body_is_accepted(
        self,
    ) -> None:
        # A glob of the subsystem's own subtree reaches nothing above it, so the
        # wider statement detection must not turn every block-local glob red.
        self.write_src(
            "uart/mod.rs",
            _lines(
                "pub fn f() {",
                "    use crate::uart::protocol::*;",
                "    use super::protocol::*;",
                "}",
            ),
        )
        self.assert_clean()

    def test_tlsr8266_pac_import_is_rejected(self) -> None:
        self.write_src("uart/mod.rs", _lines("use tlsr8266_pac::fake_mmio::FakeMmio;"))
        self.assert_violations("1: uart-forbidden-crate: tlsr8266_pac")

    def test_every_forbidden_module_is_rejected(self) -> None:
        banned = sorted(check_layering.SUBSYSTEM_PURITY["uart"][0])
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
                "pub fn f() { uprintln!(\"uart\"); }",
                "use protocol::decode_command;",
            ),
        )
        self.assert_clean()


class TestMeshPurityRule(_TempSrcTree):
    """`rust/src/mesh/**` production code may not name `uart` or the lamp."""

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
        self.write_src("mesh/mod.rs", _lines("use tlsr8266_pac::fake_mmio::FakeMmio;"))
        self.assert_violations("1: mesh-forbidden-crate: tlsr8266_pac")

    def test_tlsr8266_radio_is_allowed(self) -> None:
        # `mesh` owns the node table and drives the radio glue: it is the only
        # row whose forbidden-crate set omits `tlsr8266_radio`.
        self.write_src(
            "mesh/mod.rs",
            _lines(
                "use tlsr8266_radio::facade::ble_init;",
                "use tlsr8266_hal::mcu::clock::clock_time;",
                "use crate::logging::UartStream;",
                "pub fn f() { uprintln!(\"mesh\"); }",
            ),
        )
        self.assert_clean()

    def test_super_logging_call_is_allowed(self) -> None:
        # Control for the expression-path scan: `logging` is absent from the
        # mesh row, so `super::logging::x()` in `mesh/mod.rs` is `crate::logging`
        # and stays legal in either spelling.
        self.write_src("mesh/mod.rs", _lines("pub fn f() { super::logging::x(); }"))
        self.assert_clean()

    def test_every_forbidden_module_is_rejected(self) -> None:
        banned = sorted(check_layering.SUBSYSTEM_PURITY["mesh"][0])
        for module in banned:
            with self.subTest(module=module):
                _write(
                    self.src_root / "mesh" / f"{module}.rs",
                    _lines(f"use crate::{module}::thing;"),
                )
        out = self.assert_violations(*[f"crate::{m}" for m in banned])
        self.assertEqual(len(out.splitlines()), len(banned))

    def test_composition_root_and_the_isr_layer_are_rejected(self) -> None:
        # The subsystem docs claim `mesh` never names `app`; `irq` sits above
        # it.  Both names must be in the row, or the claim is a comment.
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
            {"app", "irq"}, set(check_layering.SUBSYSTEM_PURITY["mesh"][0])
        )

    def test_forbidden_import_inside_cfg_test_is_accepted(self) -> None:
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


class TestSubsystemPurityTable(unittest.TestCase):
    """The per-subsystem purity table is the single source of every row."""

    def test_table_is_exactly_the_issue_sets(self) -> None:
        # The normative table, pinned literally: widening or narrowing a row is
        # a deliberate act that has to change this test too.
        expected = {
            "diagnostics": (
                {"logging", "factory_reset", "ota", "light", "uart", "mesh", "app", "irq"},
                {"tlsr8266_pac", "tlsr8266_radio"},
            ),
            "logging": (
                {"factory_reset", "ota", "light", "uart", "mesh", "app", "irq"},
                {"tlsr8266_pac", "tlsr8266_radio"},
            ),
            "factory_reset": (
                {"diagnostics", "logging", "ota", "light", "uart", "mesh", "app", "irq"},
                {"tlsr8266_pac", "tlsr8266_radio"},
            ),
            "ota": (
                {
                    "diagnostics",
                    "logging",
                    "factory_reset",
                    "light",
                    "uart",
                    "mesh",
                    "app",
                    "irq",
                },
                {"tlsr8266_pac", "tlsr8266_radio"},
            ),
            "light": (
                {"diagnostics", "factory_reset", "ota", "uart", "mesh", "app", "irq"},
                {"tlsr8266_pac", "tlsr8266_radio"},
            ),
            "uart": (
                {"diagnostics", "factory_reset", "ota", "light", "mesh", "app", "irq"},
                {"tlsr8266_pac", "tlsr8266_radio"},
            ),
            "mesh": (
                {"diagnostics", "factory_reset", "ota", "light", "uart", "app", "irq"},
                {"tlsr8266_pac"},
            ),
        }
        actual = {
            name: (set(modules), set(crates))
            for name, (modules, crates) in check_layering.SUBSYSTEM_PURITY.items()
        }
        self.assertEqual(actual, expected)

    def test_no_row_forbids_its_own_name(self) -> None:
        # A row lists siblings and layers *above* the subsystem.  Adding the
        # subsystem's own name would ban its own intra-module paths.
        for name, (modules, _crates) in check_layering.SUBSYSTEM_PURITY.items():
            with self.subTest(row=name):
                self.assertNotIn(name, modules)

    def test_every_row_bans_the_pac(self) -> None:
        for name, (_modules, crates) in check_layering.SUBSYSTEM_PURITY.items():
            with self.subTest(row=name):
                self.assertIn("tlsr8266_pac", crates)

    def test_only_the_mesh_row_allows_the_radio_crate(self) -> None:
        allowed = {
            name
            for name, (_modules, crates) in check_layering.SUBSYSTEM_PURITY.items()
            if "tlsr8266_radio" not in crates
        }
        self.assertEqual(allowed, {"mesh"})

    def test_logging_is_allowed_in_the_rows_that_omit_it(self) -> None:
        allowed = {
            name
            for name, (modules, _crates) in check_layering.SUBSYSTEM_PURITY.items()
            if "logging" not in modules
        }
        # A row never names itself, so `logging` is expected too.
        self.assertEqual(allowed, {"logging", "light", "uart", "mesh"})

    def test_logging_macro_bans_only_the_leaf_subsystems(self) -> None:
        self.assertEqual(
            check_layering.LOGGING_MACRO_FORBIDDEN_SUBSYSTEMS,
            frozenset({"diagnostics", "factory_reset", "ota"}),
        )
        for subsystem in check_layering.LOGGING_MACRO_FORBIDDEN_SUBSYSTEMS:
            self.assertIn(subsystem, check_layering.SUBSYSTEM_PURITY)

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


class TestAppPacRule(_TempSrcTree):
    """The composition root and its ISR layer may not reference the PAC."""

    def test_app_reference_is_rejected(self) -> None:
        path = self.write_src(
            "app.rs",
            _lines("pub fn f() { tlsr8266_pac::write_reg8(0x408, 1); }"),
        )
        out = self.assert_violations("1: app-pac: tlsr8266_pac")
        self.assertEqual(out.splitlines()[0], f"{path}:1: app-pac: tlsr8266_pac")

    def test_irq_import_is_rejected(self) -> None:
        self.write_src(
            "irq.rs", _lines("use tlsr8266_pac::REG_IRQ_SRC;")
        )
        self.assert_violations("1: app-pac: tlsr8266_pac")

    def test_a_reference_that_is_not_a_use_is_still_rejected(self) -> None:
        # The rule is a reference check: a fully-qualified constant expression
        # must not slip past it.
        self.write_src(
            "irq.rs",
            _lines("pub fn f() { let _ = 0x630 | tlsr8266_pac::REG_BASE_ADDR; }"),
        )
        self.assert_violations("1: app-pac: tlsr8266_pac")

    def test_the_rule_is_scoped_to_app_and_irq(self) -> None:
        self.write_src("config.rs", _lines("use tlsr8266_pac::REG_BASE_ADDR;"))
        self.assert_clean()

    def test_test_only_pac_use_is_exempt(self) -> None:
        # `app.rs`'s own tests stand `FakeMmio` in for the register block.
        self.write_src(
            "app.rs",
            _lines(
                "pub fn production() {}",
                "",
                "#[cfg(test)]",
                "mod tests {",
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

    def test_static_mut_split_across_lines_fails(self) -> None:
        # Whitespace is not a token boundary: `static`, `mut` and the name may
        # each start on a new line, and `cargo fmt` does not run over
        # `#[rustfmt::skip]` code.
        self.write_src("a.rs", _lines("pub static", "    mut X: u32 = 0;"))
        self.write_src("b.rs", _lines("pub static mut", "    Y: u32 = 0;"))
        out = self.assert_violations("1: static-mut: static mut X")
        self.assertIn("b.rs:1: static-mut: static mut Y", out)
        # The reported snippet is normalised to one line even though the item
        # is not: a violation line must stay readable in a CI log.
        self.assertEqual(len(out.splitlines()), 2, out)

    def test_static_mut_reference_forms_are_legal(self) -> None:
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
        # The shape the HAL's `sync.rs` once shipped: the attribute is on a
        # field, and both production `unsafe impl`s must stay visible *and*
        # documented.
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


class TestUnsafeRatchet(unittest.TestCase):
    """The per-crate production `unsafe` ceiling is a one-way ratchet."""

    def _documented_sites(self, count: int) -> str:
        blocks = [
            f"// SAFETY: generated probe {index}.\nunsafe fn probe_{index}() {{}}"
            for index in range(count)
        ]
        return "\n".join(blocks) + "\n"

    def test_the_map_covers_the_whole_workspace(self) -> None:
        self.assertEqual(
            check_layering.MAX_PRODUCTION_UNSAFE_BY_CRATE,
            {
                "tlsr8266_mesh": 14,
                "tlsr8266-pac": 6,
                "tlsr8266-hal": 16,
                "tlsr8266-radio": 22,
                "tlsr8266-platform": 2,
                "mesh-protocol": 45,
            },
        )

    def test_at_the_ceiling_passes(self) -> None:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name) / "src"
        root.mkdir(parents=True)
        ceiling = check_layering.MAX_PRODUCTION_UNSAFE_BY_CRATE["tlsr8266_mesh"]
        _write(root / "budget.rs", self._documented_sites(ceiling))
        violations, total = check_layering.audit_universal(
            check_layering.rust_files(str(root)),
            str(root),
            crate_name="tlsr8266_mesh",
            enforce_budget=True,
        )
        self.assertEqual(violations, [])
        self.assertEqual(total, ceiling)

    def test_exceeding_the_ceiling_fails(self) -> None:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name) / "src"
        root.mkdir(parents=True)
        ceiling = check_layering.MAX_PRODUCTION_UNSAFE_BY_CRATE["tlsr8266-pac"]
        _write(root / "budget.rs", self._documented_sites(ceiling + 1))
        violations, total = check_layering.audit_universal(
            check_layering.rust_files(str(root)),
            str(root),
            crate_name="tlsr8266-pac",
            enforce_budget=True,
        )
        self.assertEqual(total, ceiling + 1)
        self.assertEqual(
            [(item.rule, item.line) for item in violations],
            [("unsafe-budget", 0)],
        )
        message = violations[0].violation
        self.assertIn(f"{ceiling + 1} production unsafe tokens", message)
        self.assertIn(f"ceiling {ceiling} for tlsr8266-pac", message)
        # The advice must be right for an *added* site, which is the only way
        # this violation can fire: the one-way framing belongs in the map's
        # docstring, not in a message read in the opposite direction.
        self.assertIn(
            "remove unsafe, or justify and ratchet the map deliberately", message
        )
        self.assertNotIn("never raise it", message)
        # A budget belongs to the whole tree, not to a source line: the report
        # must not send a reader looking for line 0.
        rendered = violations[0].render()
        self.assertNotIn(":0:", rendered)
        self.assertIn(": unsafe-budget: ", rendered)

    def test_budget_is_off_for_an_explicit_root(self) -> None:
        # Hermetic fixtures must not be charged to a shipping ceiling: the CLI
        # passes `enforce_budget=False` whenever a root was named explicitly.
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name) / "src"
        root.mkdir(parents=True)
        _write(root / "budget.rs", self._documented_sites(100))
        violations, total = check_layering.audit_universal(
            check_layering.rust_files(str(root)), str(root), enforce_budget=False
        )
        self.assertEqual(violations, [])
        self.assertEqual(total, 100)

    def test_an_unknown_crate_with_a_budget_is_an_error(self) -> None:
        # Fail closed: a root without a ceiling must not be silently unbudgeted.
        with self.assertRaises(check_layering.GateError):
            check_layering.audit_universal(
                [], "/tmp/whatever/src", crate_name="no-such-crate", enforce_budget=True
            )


class TestHostOnlyUnsafeBudget(_TempSrcTree):
    """`#[cfg(not(target_os = "none"))]` code never reaches the firmware image.

    It is excluded from the numeric `unsafe-budget` only.  The SAFETY and
    `static mut` rules keep applying: host-only code is still real Rust in the
    audited tree and still compiled for host tests.
    """

    def _audit_file(self, *lines: str) -> tuple[list, int]:
        path = self.write_src("x.rs", _lines(*lines))
        return check_layering.audit_universal_source(
            path.read_text(encoding="utf-8"), str(path), module=("x",)
        )

    def test_host_only_item_is_excluded_from_the_budget(self) -> None:
        violations, total = self._audit_file(
            '#[cfg(not(target_os = "none"))]',
            "// SAFETY: (1) host-only probe.",
            "unsafe fn host_probe() {}",
            "// SAFETY: (1) production probe.",
            "unsafe fn production_probe() {}",
        )
        self.assertEqual(violations, [])
        self.assertEqual(total, 1)  # only the production site is charged

    def test_host_only_unsafe_without_safety_still_fails(self) -> None:
        violations, total = self._audit_file(
            '#[cfg(not(target_os = "none"))]',
            "unsafe fn host_probe() {}",
        )
        self.assertEqual(
            [(item.rule, item.line) for item in violations],
            [("undocumented-unsafe", 2)],
        )
        self.assertEqual(total, 0)  # excluded from the budget, not from SAFETY

    def test_host_only_unsafe_with_safety_raises_nothing(self) -> None:
        violations, total = self._audit_file(
            '#[cfg(not(target_os = "none"))]',
            "// SAFETY: (1) host-only probe.",
            "unsafe fn host_probe() {}",
        )
        self.assertEqual(violations, [])
        self.assertEqual(total, 0)

    def test_host_only_static_mut_is_still_reported(self) -> None:
        violations, total = self._audit_file(
            '#[cfg(not(target_os = "none"))]',
            "static mut HOST_SCRATCH: u8 = 0;",
        )
        self.assertEqual(
            [(item.rule, item.line, item.violation) for item in violations],
            [("static-mut", 2, "static mut HOST_SCRATCH")],
        )
        self.assertEqual(total, 0)

    def test_production_unsafe_is_still_counted(self) -> None:
        # Positive control for the exclusion above.
        violations, total = self._audit_file(
            "// SAFETY: (1) production probe.",
            "unsafe fn production_probe() {}",
            "",
            "// SAFETY: (1) second production probe.",
            "unsafe fn production_probe_two() {}",
        )
        self.assertEqual(violations, [])
        self.assertEqual(total, 2)

    def test_unsafe_impl_and_nested_unsafe_inside_a_host_only_item(self) -> None:
        violations, total = self._audit_file(
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
        violations, total = self._audit_file(
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
        violations, total = self._audit_file(
            '// historically: #[cfg(not(target_os = "none"))]',
            "",
            "// SAFETY: (1) production probe.",
            "unsafe fn production_probe() {}",
        )
        self.assertEqual(violations, [])
        self.assertEqual(total, 1)

    def test_host_only_sites_are_not_charged_to_the_crate_ceiling(self) -> None:
        # Gate-level proof: more documented host-only `unsafe` sites than the
        # whole firmware ceiling must still pass, because none of them ship.
        ceiling = check_layering.MAX_PRODUCTION_UNSAFE_BY_CRATE["tlsr8266_mesh"]
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
        violations, total = check_layering.audit_universal(
            check_layering.rust_files(str(self.src_root)),
            str(self.src_root),
            crate_name="tlsr8266_mesh",
            enforce_budget=True,
        )
        self.assertEqual(violations, [])
        self.assertEqual(total, 0)


class TestWorkspaceCrateUniversalRules(_TempSrcTree):
    """A workspace crate is audited like the firmware for `unsafe` invariants."""

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

    def test_crate_ceiling_is_its_own(self) -> None:
        # The PA C ceiling is 6, not the firmware's 16: `enforce_budget` on a
        # crate root reads that crate's row.
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        crate = Path(tmp.name) / "tlsr8266-pac" / "src"
        crate.mkdir(parents=True)
        ceiling = check_layering.MAX_PRODUCTION_UNSAFE_BY_CRATE["tlsr8266-pac"]
        _write(
            crate / "lib.rs",
            "\n".join(
                f"// SAFETY: probe {index}.\nunsafe fn probe_{index}() {{}}"
                for index in range(ceiling + 1)
            )
            + "\n",
        )
        violations, total = check_layering.audit_universal(
            check_layering.rust_files(str(crate)),
            str(crate),
            enforce_budget=True,
        )
        self.assertEqual(total, ceiling + 1)
        self.assertEqual([item.rule for item in violations], ["unsafe-budget"])

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
    """Each universal root reports independently; a failure exits 1."""

    def test_universal_failure_exits_1(self) -> None:
        self.write_src("x.rs", _lines("static mut GLOBAL: u8 = 0;"))
        code, out, err = self.run_gate()
        self.assertEqual(code, 1, out + err)
        self.assertIn("static-mut", out)
        self.assertIn("rust src audit FAILED", err)

    def test_all_passes_clean_prints_one_summary_per_root(self) -> None:
        code, out, err = self.run_gate()
        self.assertEqual(code, 0, out + err)
        self.assertEqual(
            [line.split(":")[0] for line in out.splitlines()],
            ["rust src audit OK", "mesh-protocol audit OK"],
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
        # pass and no implicit workspace crate is read along with it.  The
        # banner names that third skipped pass too, so a copy-pasted invocation
        # cannot look green with the purity rows unexecuted.
        self.write_src("ota/mod.rs", _lines("use crate::app::App;"))
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = check_layering.main(["--crate-root", str(self.crate_root)])
        self.assertEqual(code, 0, out.getvalue() + err.getvalue())
        self.assertIn(f"under {self.crate_root}", out.getvalue())
        self.assertNotIn("rust src audit", out.getvalue())
        self.assertNotIn("tlsr8266-pac audit", out.getvalue())
        self.assertNotIn("tlsr8266-hal audit", out.getvalue())
        self.assertNotIn("cargo dag OK", out.getvalue())
        # The planted firmware violation proves the skipped pass really is off.
        self.assertNotIn("ota-upward-dep", out.getvalue())
        self.assertEqual(
            out.getvalue().splitlines()[0],
            "hermetic mode: cargo DAG, unsafe ratchet and rust/src subsystem "
            "pass skipped",
        )


if __name__ == "__main__":
    unittest.main()
