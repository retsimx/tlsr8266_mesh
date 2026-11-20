"""Unit tests for the deterministic, incremental TC32 build driver.

Run from the repository root::

    python3 -m unittest discover -s sdk/tests -v

The suite is pure stdlib and never invokes cargo, LLVM, or the TC32 toolchain:
every subprocess goes through ``build_firmware.run_command``, which is patched.
"""

from __future__ import annotations

import contextlib
import hashlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "sdk"))

import build_firmware  # noqa: E402  (sys.path set up above)

LIBGCC_REL = "toolchain/tc32/lib/gcc/tc32-elf/4.5.1.tc32-elf-1.5/libgcc.a"
DEPS_REL = "rust/target/thumbv6m-none-eabi/release/deps"
BUILD_REL = "rust/target/thumbv6m-none-eabi/release/build"
CACHE_HIT_MESSAGE = "Build manifest unchanged for 'default'; skipping LLVM/TC32 link."

RUSTC_VV_SAMPLE = """\
rustc 1.102.0-nightly (6905905bb 2026-11-19)
binary: rustc
commit-hash: 6905905bb7727fd0f9223f5fdc9ad4e62236d5a2
commit-date: 2026-11-19
host: x86_64-unknown-linux-gnu
release: 1.102.0-nightly
LLVM version: 23.1.6
"""

LLC_VERSION_SAMPLE = """\
LLVM (http://llvm.org/):
  LLVM version 23.1.7
  Optimized build with assertions.
  Default target:
  Host CPU: znver3
"""

FAKE_TOOLCHAIN = {
    "rustc_release": "1.102.0-nightly",
    "rustc_commit": "6905905bb7727fd0f9223f5fdc9ad4e62236d5a2",
    "rustc_llvm": "23.1.6",
    "rustc_llvm_major": 23,
    "fork_llvm": "23.1.7",
    "fork_llvm_major": 23,
}

# The pre-#37 flag literals, pinned here *on purpose*: the pipeline-argv test is
# the CI guard for the normative "flags and object order must not change"
# clause, so it must not read them back from the module under test.
OPT_SAFE_LITERAL = (
    "sroa,mem2reg,instcombine<no-verify-fixpoint>,simplifycfg,dce,"
    "loop-simplify,lcssa,loop-rotate,loop-mssa(licm),"
    "instcombine<no-verify-fixpoint>,simplifycfg,adce,dse"
)
OPT_PASSES_LITERAL = (
    f"ipsccp,deadargelim,function({OPT_SAFE_LITERAL}),always-inline,cgscc(inline),"
    f"ipsccp,deadargelim,function({OPT_SAFE_LITERAL})"
)


def _write(path: Path, content: str | bytes = b"x\n") -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(content.encode() if isinstance(content, str) else content)
    return target


def _executable(path: Path) -> Path:
    tool = _write(path, "#!/bin/sh\nexit 0\n")
    tool.chmod(0o755)
    return tool


def _artifact(*filenames: Path | str, kind: tuple[str, ...] = ("lib",)) -> str:
    return json.dumps(
        {
            "reason": "compiler-artifact",
            "target": {"kind": list(kind)},
            "filenames": [str(name) for name in filenames],
        }
    )


class _TempCase(unittest.TestCase):
    """A disposable directory per test."""

    def setUp(self) -> None:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)


class _RepoCase(_TempCase):
    """A fake repository with every file the manifest hashes."""

    def setUp(self) -> None:
        super().setUp()
        self.repo = self.root / "repo"
        self.out = self.repo / "_build"
        # rustc emits IR as `<crate>-<hash>.ll`; the rlib carries the `lib` prefix.
        self.ll_a = _write(self.repo / DEPS_REL / "alpha-11111111.ll", "alpha\n")
        self.ll_b = _write(self.repo / DEPS_REL / "beta-22222222.ll", "beta\n")
        self.startup_s = _write(self.repo / "sdk/cstartup_8266.S", ".section .text\n")
        self.version_in = _write(
            self.repo / "sdk/version.in", ".equ BUILD_VERSION,42\n"
        )
        self.version_h = _write(self.repo / "sdk/version.h", "#define BUILD_VERSION 42\n")
        self.version_rs = _write(
            self.repo / "rust/src/version.rs", "pub static BUILD_VERSION: u32 = 42;\n"
        )
        self.boot_link = _write(self.repo / "sdk/boot.link", "ENTRY(reset)\n")
        self.libgcc = _write(self.repo / LIBGCC_REL, b"!<arch>\n")
        self.tools = {
            "llc": _executable(self.repo / "tools/llc"),
            "llvm_link": _executable(self.repo / "tools/llvm-link"),
            "opt": _executable(self.repo / "tools/opt"),
            "as": _executable(self.repo / "toolchain/tc32/bin/tc32-elf-as"),
            "ld": _executable(self.repo / "toolchain/tc32/bin/tc32-elf-ld"),
            "objcopy": _executable(self.repo / "toolchain/tc32/bin/tc32-elf-objcopy"),
        }
        self.startup_inputs = [self.repo / name for name in build_firmware.STARTUP_INPUTS]
        self.startup_obj, self.startup_manifest = self._install_startup()

    def _install_startup(self) -> tuple[Path, Path]:
        """Create/replace the startup object through the driver's own cache path.

        The fake compiler writes a digest of the startup inputs as the object
        bytes, so a changed input produces a changed object, exactly as a real
        recompile would.

        Note: this runs with the real ``REPO_ROOT`` because ``_ensure_startup_object``
        is given ``self.repo`` explicitly, so the recorded input paths are
        absolute. A test that compares manifests against a freshly computed one
        must therefore reinstall the cache under a patched ``REPO_ROOT`` (see
        ``test_cache_hit_skips_pipeline``); otherwise it sees a cache *miss*.
        """

        def fake_compiler(cmd, **_kwargs):
            digest = hashlib.sha256(
                b"".join(path.read_bytes() for path in self.startup_inputs)
            ).hexdigest()
            argv = list(cmd)
            _write(Path(argv[argv.index("-o") + 1]), digest.encode())
            return subprocess.CompletedProcess(argv, 0)

        with mock.patch.object(build_firmware, "run_command", side_effect=fake_compiler):
            obj = build_firmware._ensure_startup_object(self.out, self.repo)
        return obj, self.out / "asm" / "cstartup_8266.manifest.json"

    def compute(
        self,
        *,
        features: tuple[str, ...] = (),
        ll_paths: list[Path] | None = None,
        tools: dict[str, Path] | None = None,
        toolchain: dict[str, object] | None = None,
    ) -> dict:
        return build_firmware.compute_manifest(
            list(features),
            [self.ll_a, self.ll_b] if ll_paths is None else ll_paths,
            self.startup_obj,
            self.boot_link,
            self.libgcc,
            self.tools if tools is None else tools,
            toolchain=toolchain,
        )


class TestFeatureSelection(unittest.TestCase):
    def test_parse_features_and_slug(self) -> None:
        cases = [
            (None, [], "default"),
            ("", [], "default"),
            ("   \t\n", [], "default"),
            ("rtt", ["rtt"], "rtt"),
            ("b, a", ["a", "b"], "a-b"),
            (" rtt , rtt,  foo\n\tbar ", ["bar", "foo", "rtt"], "bar-foo-rtt"),
            ("rtt,rtt", ["rtt"], "rtt"),
        ]
        for raw, expected_features, expected_slug in cases:
            with self.subTest(raw=raw):
                features = build_firmware.parse_features(raw)
                self.assertEqual(features, expected_features)
                self.assertEqual(build_firmware.feature_slug(features), expected_slug)


class TestDotenvAndToolResolution(_TempCase):
    def test_parse_dotenv_syntax(self) -> None:
        dotenv = _write(
            self.root / ".env",
            "\n".join(
                [
                    "# a comment",
                    "",
                    "   ",
                    "export LLC=/opt/llc",
                    'OPT="/opt/opt with space"',
                    "LLVM_LINK='/opt/llvm-link'",
                    "EQUATION=a=b=c",
                    "   export   SPACED = value   ",
                    'MIXED="/unbalanced\'',
                    "EMPTY=",
                    "export",
                ]
            ),
        )
        self.assertEqual(
            build_firmware.parse_dotenv(dotenv),
            {
                "LLC": "/opt/llc",
                "OPT": "/opt/opt with space",
                "LLVM_LINK": "/opt/llvm-link",
                "EQUATION": "a=b=c",
                "SPACED": "value",
                "MIXED": "\"/unbalanced'",
                "EMPTY": "",
            },
        )

    def test_parse_dotenv_missing_file(self) -> None:
        self.assertEqual(build_firmware.parse_dotenv(self.root / "absent.env"), {})

    def test_parse_dotenv_never_overwrites_environment(self) -> None:
        dotenv = _write(self.root / ".env", "LLC=/from/dotenv\n")
        with mock.patch.dict(os.environ, {"LLC": "/from/env"}):
            values = build_firmware.parse_dotenv(dotenv)
            self.assertEqual(os.environ["LLC"], "/from/env")
        self.assertEqual(values["LLC"], "/from/dotenv")

    def test_resolve_tool_precedence(self) -> None:
        repo = self.root / "repo"
        default = "../llvm/build/bin/llc"
        with mock.patch.dict(os.environ, {"LLC": "env/llc"}):
            self.assertEqual(
                build_firmware.resolve_tool("cli/llc", "LLC", {"LLC": "dotenv/llc"}, default, repo),
                repo / "cli/llc",
            )
            self.assertEqual(
                build_firmware.resolve_tool("", "LLC", {"LLC": "dotenv/llc"}, default, repo),
                repo / "env/llc",
            )
            self.assertEqual(
                build_firmware.resolve_tool(
                    None, "BUILD_FIRMWARE_TEST_UNSET", {"BUILD_FIRMWARE_TEST_UNSET": "dotenv/llc"}, default, repo
                ),
                repo / "dotenv/llc",
            )
            self.assertEqual(
                build_firmware.resolve_tool(None, "BUILD_FIRMWARE_TEST_UNSET", {}, default, repo),
                repo / default,
            )
        self.assertEqual(
            build_firmware.resolve_tool("/abs/tool", "LLC", {}, default, repo),
            Path("/abs/tool"),
        )

    def test_resolve_tool_anchors_at_repo_root_not_cwd(self) -> None:
        repo = self.root / "repo"
        elsewhere = self.root / "elsewhere"
        elsewhere.mkdir()
        with contextlib.chdir(elsewhere):
            self.assertEqual(
                build_firmware.resolve_tool(
                    None, "BUILD_FIRMWARE_TEST_UNSET", {"BUILD_FIRMWARE_TEST_UNSET": "tools/opt"}, "ignored", repo
                ),
                repo / "tools/opt",
            )
        self.assertEqual(
            build_firmware.resolve_tool(None, "BUILD_FIRMWARE_TEST_UNSET", {}, "../llvm/build/bin/opt", repo),
            repo / "../llvm/build/bin/opt",
        )


class TestArtifactResolution(_RepoCase):
    def test_parse_cargo_artifacts_filters_messages(self) -> None:
        deps = self.repo / DEPS_REL
        wanted = deps / "libwanted-12345678.rlib"
        lines = [
            "   Compiling wanted v0.1.0",
            "{not json at all",
            json.dumps({"reason": "build-finished", "success": True}),
            _artifact(deps / "libbuildscript-87654321.rlib", kind=("custom-build",)),
            _artifact(deps / "libmacro-11112222.rlib", kind=("proc-macro",)),
            _artifact(Path("/home/user/.cargo/registry/other/libhost-33334444.rlib")),
            _artifact(wanted, deps / "libwanted-12345678.so"),
        ]
        self.assertEqual(build_firmware.parse_cargo_artifacts(lines), [wanted])
        self.assertEqual(build_firmware.parse_cargo_artifacts("\n".join(lines)), [wanted])

    def test_resolve_ll_paths_selects_own_ir_and_ignores_stale(self) -> None:
        deps = self.repo / DEPS_REL
        alpha_rlib = _write(deps / "libalpha-11111111.rlib", "alpha rlib")
        beta_rlib = _write(deps / "libbeta-22222222.rlib", "beta rlib")
        alpha_ll = self.ll_a
        beta_ll = self.ll_b
        stale = _write(deps / "stale-deadbeef.ll", "stale")
        macro_rlib = deps / "libmacro-99999999.rlib"  # must be skipped, has no .ll
        self.assertTrue(stale.is_file())

        stdout = "\n".join(
            [
                _artifact(alpha_rlib),
                _artifact(macro_rlib, kind=("proc-macro",)),
                _artifact(beta_rlib),
                _artifact(alpha_rlib),  # duplicate report -> deduplicated result
            ]
        )
        resolved = build_firmware.resolve_ll_paths(
            build_firmware.parse_cargo_artifacts(stdout)
        )
        self.assertEqual(resolved, sorted({alpha_ll, beta_ll}))
        self.assertNotIn(stale, resolved)

    def test_resolve_ll_paths_missing_ir_names_the_rlib(self) -> None:
        rlib = self.repo / DEPS_REL / "libgamma-44444444.rlib"
        with self.assertRaises(build_firmware.BuildError) as caught:
            build_firmware.resolve_ll_paths([rlib])
        self.assertIn(str(rlib), str(caught.exception))
        self.assertIn("gamma-44444444.ll", str(caught.exception))

    def test_root_rlib_hardlink_inode_match(self) -> None:
        deps = self.repo / DEPS_REL
        release = self.repo / "rust/target/thumbv6m-none-eabi/release"
        hashed = _write(deps / "libroot-aaaa1111.rlib", "root rlib payload")
        expected_ll = _write(deps / "root-aaaa1111.ll", "root ir")
        root = release / "libroot.rlib"
        root.parent.mkdir(parents=True, exist_ok=True)
        os.link(hashed, root)

        self.assertEqual(os.stat(root).st_ino, os.stat(hashed).st_ino)
        self.assertEqual(
            build_firmware.resolve_ll_paths([root]), [expected_ll]
        )

    def test_root_rlib_non_hardlink_fallback(self) -> None:
        deps = self.repo / DEPS_REL
        release = self.repo / "rust/target/thumbv6m-none-eabi/release"
        content = b"identical root rlib payload"
        older = _write(deps / "libroot-aaaa1111.rlib", content)
        newer = _write(deps / "libroot-bbbb2222.rlib", content)
        older_ll = _write(deps / "root-aaaa1111.ll", "older ir")
        newer_ll = _write(deps / "root-bbbb2222.ll", "newer ir")
        root = release / "libroot.rlib"
        root.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(older, root)

        # Different inodes, so only the (st_size, sha256) fallback can match.
        self.assertNotEqual(os.stat(root).st_ino, os.stat(older).st_ino)
        self.assertEqual(os.stat(root).st_size, os.stat(older).st_size)
        self.assertEqual(
            hashlib.sha256(root.read_bytes()).hexdigest(),
            hashlib.sha256(older.read_bytes()).hexdigest(),
        )

        with self.subTest("newest mtime wins"):
            os.utime(older, ns=(1_600_000_000_000_000_000, 1_600_000_000_000_000_000))
            os.utime(newer, ns=(1_600_000_100_000_000_000, 1_600_000_100_000_000_000))
            self.assertEqual(build_firmware.resolve_ll_paths([root]), [newer_ll])

        with self.subTest("identical mtimes break lexically"):
            same = 1_600_000_200_000_000_000
            os.utime(older, ns=(same, same))
            os.utime(newer, ns=(same, same))
            self.assertEqual(build_firmware.resolve_ll_paths([root]), [older_ll])

    def test_resolve_ll_paths_cargo_1_100_build_out_layout(self) -> None:
        build_root = self.repo / BUILD_REL
        core_out = build_root / "core" / "2f8de896c3184f8b" / "out"
        builtins_out = build_root / "compiler_builtins" / "67c4781c90c5b76c" / "out"
        app_out = build_root / "tlsr8266_mesh" / "aeee5356951bbd14" / "out"

        core_rlib = _write(core_out / "libcore-2f8de896c3184f8b.rlib", "core")
        core_ll = _write(core_out / "core-2f8de896c3184f8b.ll", "core ir")
        builtins_rlib = _write(
            builtins_out / "libcompiler_builtins-67c4781c90c5b76c.rlib", "cb"
        )
        builtins_ll = _write(
            builtins_out / "compiler_builtins-67c4781c90c5b76c.ll", "cb ir"
        )
        app_rlib = _write(app_out / "libtlsr8266_mesh-aeee5356951bbd14.rlib", "app")
        app_ll = _write(app_out / "tlsr8266_mesh-aeee5356951bbd14.ll", "app ir")

        stale_build = _write(
            build_root / "tlsr8266_mesh" / "00000000deadbeef" / "out" / "tlsr8266_mesh-00000000deadbeef.ll",
            "stale",
        )
        stale_deps = _write(self.repo / DEPS_REL / "tlsr8266_mesh-deadbeef.ll", "stale")

        stdout = "\n".join(
            [
                _artifact(core_rlib, core_out / "libcore-2f8de896c3184f8b.rmeta"),
                _artifact(builtins_rlib),
                _artifact(app_rlib),
            ]
        )
        resolved = build_firmware.resolve_ll_paths(
            build_firmware.parse_cargo_artifacts(stdout)
        )
        self.assertEqual(resolved, sorted([core_ll, builtins_ll, app_ll]))
        self.assertNotIn(stale_build, resolved)
        self.assertNotIn(stale_deps, resolved)

    def test_root_rlib_matches_cargo_1_100_build_out_twin(self) -> None:
        release = self.repo / "rust/target/thumbv6m-none-eabi/release"
        out_dir = self.repo / BUILD_REL / "root" / "aeee5356951bbd14" / "out"
        hashed = _write(out_dir / "libroot-aeee5356951bbd14.rlib", "root payload")
        expected_ll = _write(out_dir / "root-aeee5356951bbd14.ll", "root ir")

        with self.subTest("hardlink twin in build/<crate>/<hash>/out"):
            root = release / "libroot.rlib"
            root.parent.mkdir(parents=True, exist_ok=True)
            os.link(hashed, root)
            self.assertEqual(build_firmware.resolve_ll_paths([root]), [expected_ll])
            root.unlink()

        with self.subTest("copied twin in build/<crate>/<hash>/out"):
            root = release / "libroot.rlib"
            shutil.copyfile(hashed, root)
            self.assertEqual(build_firmware.resolve_ll_paths([root]), [expected_ll])

        with self.subTest("ignores prefix-colliding sibling crate in build/<crate>/<hash>/out"):
            sibling_out = self.repo / BUILD_REL / "root-sibling" / "bbbb2222cccc3333" / "out"
            _write(sibling_out / "libroot-sibling-bbbb2222cccc3333.rlib", "root payload")
            _write(sibling_out / "root-sibling-bbbb2222cccc3333.ll", "sibling ir")
            hashed.unlink()
            with self.assertRaises(build_firmware.BuildError) as caught:
                build_firmware.resolve_ll_paths([root])
            self.assertEqual(caught.exception.exit_code, 1)


class TestManifestAndCache(_RepoCase):
    def _assert_edit_invalidates(self, baseline: dict, label: str, edit, restore) -> None:
        with self.subTest(label):
            edit()
            self.assertFalse(
                build_firmware.manifests_equal(baseline, self.compute()), label
            )
            restore()
            self.assertTrue(
                build_firmware.manifests_equal(baseline, self.compute()), label
            )

    def _assert_startup_edit_invalidates(self, baseline: dict, path: Path) -> None:
        original = path.read_bytes()

        def edit() -> None:
            path.write_bytes(original + b"\n/* edited */\n")
            self.assertTrue(
                build_firmware.startup_needs_rebuild(
                    self.startup_obj, self.startup_manifest, self.startup_inputs
                )
            )
            self.startup_obj, self.startup_manifest = self._install_startup()

        def restore() -> None:
            path.write_bytes(original)
            self.startup_obj, self.startup_manifest = self._install_startup()

        self._assert_edit_invalidates(baseline, path.name, edit, restore)

    def test_manifest_is_stable_then_invalidated_per_input(self) -> None:
        baseline = self.compute()
        self.assertTrue(build_firmware.manifests_equal(baseline, self.compute()))

        original_ll = self.ll_a.read_bytes()
        self._assert_edit_invalidates(
            baseline,
            ".ll content",
            lambda: self.ll_a.write_bytes(original_ll + b"\n; edited\n"),
            lambda: self.ll_a.write_bytes(original_ll),
        )

        self._assert_startup_edit_invalidates(baseline, self.startup_s)
        self._assert_startup_edit_invalidates(baseline, self.version_h)

        original_link = self.boot_link.read_bytes()
        self._assert_edit_invalidates(
            baseline,
            "boot.link content",
            lambda: self.boot_link.write_bytes(original_link + b"/* edited */\n"),
            lambda: self.boot_link.write_bytes(original_link),
        )

        with self.subTest("OPT_PASSES"):
            changed = build_firmware.OPT_PASSES + ",tailcallelim"
            with mock.patch.object(build_firmware, "OPT_PASSES", changed):
                self.assertFalse(build_firmware.manifests_equal(baseline, self.compute()))

        with self.subTest("feature list"):
            self.assertFalse(
                build_firmware.manifests_equal(baseline, self.compute(features=("rtt",)))
            )

        with self.subTest("tool mtime_ns"):
            llc = self.tools["llc"]
            stat = os.stat(llc)
            os.utime(llc, ns=(stat.st_atime_ns, stat.st_mtime_ns + 1_000_000_000))
            self.assertFalse(build_firmware.manifests_equal(baseline, self.compute()))

        with self.subTest("tool size"):
            opt = self.tools["opt"]
            opt.write_bytes(opt.read_bytes() + b"# grow\n")
            self.assertFalse(build_firmware.manifests_equal(baseline, self.compute()))

    def test_startup_needs_rebuild(self) -> None:
        fresh_obj = self.out / "asm" / "absent.o"
        self.assertTrue(
            build_firmware.startup_needs_rebuild(
                fresh_obj, self.out / "asm" / "absent.json", self.startup_inputs
            )
        )
        self.assertFalse(
            build_firmware.startup_needs_rebuild(
                self.startup_obj, self.startup_manifest, self.startup_inputs
            )
        )
        _write(self.version_h, "#define BUILD_VERSION 43\n")
        self.assertTrue(
            build_firmware.startup_needs_rebuild(
                self.startup_obj, self.startup_manifest, self.startup_inputs
            )
        )

    def test_cache_hit_skips_pipeline(self) -> None:
        stage = self.out / "default"
        _write(stage / "merged.o", b"untouched object")
        _write(stage / "lightblemesh", b"elf image")
        _write(stage / "lightblemesh.bin", b"binary image")
        args = build_firmware._build_parser().parse_args(
            [
                "--llc",
                str(self.tools["llc"]),
                "--llvm-link",
                str(self.tools["llvm_link"]),
                "--opt",
                str(self.tools["opt"]),
            ]
        )

        with mock.patch.object(build_firmware, "REPO_ROOT", self.repo):
            self.startup_obj, self.startup_manifest = self._install_startup()
            build_firmware._write_json(
                stage / "manifest.json", self.compute(toolchain=FAKE_TOOLCHAIN)
            )
            with mock.patch.object(
                build_firmware, "probe_toolchain", return_value=FAKE_TOOLCHAIN
            ), mock.patch.object(
                build_firmware, "_cargo_build", return_value=[self.ll_a, self.ll_b]
            ), mock.patch.object(build_firmware, "run_command") as runner:
                printed = io.StringIO()
                with contextlib.redirect_stdout(printed):
                    self.assertEqual(build_firmware._run_build(args), 0)

        self.assertIn(CACHE_HIT_MESSAGE, printed.getvalue())
        self.assertIn("Toolchain: rustc 1.102.0-nightly", printed.getvalue())
        runner.assert_not_called()
        self.assertEqual((stage / "merged.o").read_bytes(), b"untouched object")
        self.assertEqual((self.out / "lightblemesh").read_bytes(), b"elf image")
        self.assertEqual((self.out / "lightblemesh.bin").read_bytes(), b"binary image")


class TestPipelineCommands(_TempCase):
    """Pin the normative argv of the six-step pipeline and the startup compile.

    The flags and the link object order are a contract shared with the recorded
    byte-identical image; this is the only CI-visible guard on them, so every
    element below is asserted literally rather than read back from the module.
    """

    def test_stage_paths_and_argv_are_frozen(self) -> None:
        stage = self.root / "_build" / "default"
        tools = {name: Path(f"/tools/{name}") for name in build_firmware.TOOL_ORDER}
        ll_paths = [Path("/ir/alpha-11111111.ll"), Path("/ir/beta-22222222.ll")]
        startup_obj = Path("/repo/_build/asm/cstartup_8266.o")
        libgcc = Path("/repo/toolchain/libgcc.a")

        self.assertEqual(
            build_firmware._stage_paths(stage),
            {
                "bc": str(stage / "merged.bc"),
                "opt_bc": str(stage / "merged_opt.bc"),
                "asm": str(stage / "merged.s"),
                "obj": str(stage / "merged.o"),
                "elf": str(stage / "lightblemesh"),
                "bin": str(stage / "lightblemesh.bin"),
            },
        )

        commands = build_firmware._pipeline_commands(
            stage, ll_paths, startup_obj, libgcc, tools
        )

        self.assertEqual(
            [label for label, _ in commands],
            ["llvm-link", "opt", "llc", "as", "ld", "objcopy"],
        )
        self.assertEqual(
            [argv for _, argv in commands],
            [
                [
                    "/tools/llvm_link",
                    "/ir/alpha-11111111.ll",
                    "/ir/beta-22222222.ll",
                    "-o",
                    str(stage / "merged.bc"),
                ],
                [
                    "/tools/opt",
                    f"-passes={OPT_PASSES_LITERAL}",
                    "-inline-threshold=50",
                    str(stage / "merged.bc"),
                    "-o",
                    str(stage / "merged_opt.bc"),
                ],
                [
                    "/tools/llc",
                    "-march=thumb",
                    "-mcpu=arm9",
                    "-mattr=+soft-float,-v6",
                    str(stage / "merged_opt.bc"),
                    "-o",
                    str(stage / "merged.s"),
                ],
                [
                    "/tools/as",
                    "-o",
                    str(stage / "merged.o"),
                    str(stage / "merged.s"),
                ],
                [
                    "/tools/ld",
                    "--gc-sections",
                    "-T",
                    "./sdk/boot.link",
                    "-o",
                    str(stage / "lightblemesh"),
                    str(stage / "merged.o"),
                    str(startup_obj),
                    str(libgcc),
                ],
                [
                    "/tools/objcopy",
                    "-O",
                    "binary",
                    str(stage / "lightblemesh"),
                    str(stage / "lightblemesh.bin"),
                ],
            ],
        )
        with self.subTest("link order: merged.o, startup object, libgcc.a"):
            self.assertEqual(
                commands[4][1][-3:],
                [str(stage / "merged.o"), str(startup_obj), str(libgcc)],
            )

    def test_startup_compile_argv_is_frozen(self) -> None:
        repo = self.root / "repo"
        obj = repo / "_build/asm/cstartup_8266.o"
        self.assertEqual(
            build_firmware._startup_command(repo, obj),
            [
                str(repo / "toolchain/tc32/bin/tc32-elf-gcc"),
                "-B",
                str(repo / "toolchain/tc32/bin/tc32-elf-"),
                "-c",
                "-O2",
                "-fshort-wchar",
                "-fms-extensions",
                "-finline-small-functions",
                "-fpack-struct",
                "-fshort-enums",
                "-Wall",
                "-std=gnu99",
                "-DMCU_STARTUP_8266",
                "-D__PROJECT_LIGHT_8266__=1",
                "-DPROVISIONING_ENABLE",
                "-I",
                "./sdk/",
                "-ffunction-sections",
                "-fdata-sections",
                str(repo / "sdk/cstartup_8266.S"),
                "-o",
                str(obj),
            ],
        )


class TestVersionSync(_RepoCase):
    def test_check_version_sync_returns_matching_version(self) -> None:
        self.assertEqual(build_firmware.check_version_sync(self.repo), 42)

    def test_check_version_sync_rejects_mismatch(self) -> None:
        _write(self.version_rs, "pub static BUILD_VERSION: u32 = 43;\n")
        with self.assertRaises(build_firmware.BuildError) as caught:
            build_firmware.check_version_sync(self.repo)
        message = str(caught.exception)
        self.assertIn("42", message)
        self.assertIn("43", message)
        self.assertIn(str(self.version_in), message)
        self.assertIn(str(self.version_rs), message)
        self.assertEqual(caught.exception.exit_code, 2)

    def test_check_version_sync_rejects_missing_file(self) -> None:
        for path in (self.version_in, self.version_rs):
            with self.subTest(missing=path.name):
                path.unlink()
                with self.assertRaises(build_firmware.BuildError) as caught:
                    build_firmware.check_version_sync(self.repo)
                self.assertIn(str(path), str(caught.exception))
                self.assertEqual(caught.exception.exit_code, 2)
                _write(path, ".equ BUILD_VERSION,42\n" if path == self.version_in else "pub static BUILD_VERSION: u32 = 42;\n")


def _compiler_message(rendered: str | None, level: str = "warning") -> str:
    return json.dumps(
        {
            "reason": "compiler-message",
            "message": {"level": level, "rendered": rendered},
        }
    )


class TestCargoDiagnostics(_RepoCase):
    def test_render_cargo_messages_writes_rendered_diagnostics(self) -> None:
        deps = self.repo / DEPS_REL
        lines = [
            "   Compiling app v0.1.0",
            "{not json",
            _artifact(deps / "libalpha-11111111.rlib"),
            _compiler_message(None),
            _compiler_message("   \n"),
            _compiler_message("warning: unused variable\n --> src/lib.rs:3:9\n"),
            _compiler_message("error[E0425]: cannot find value", level="error"),
        ]
        stderr = io.StringIO()
        count = build_firmware.render_cargo_messages(lines, stream=stderr)
        self.assertEqual(count, 2)
        self.assertEqual(
            stderr.getvalue(),
            "warning: unused variable\n --> src/lib.rs:3:9\n"
            "error[E0425]: cannot find value\n",
        )

    def test_cargo_build_renders_warnings_on_success(self) -> None:
        deps = self.repo / DEPS_REL
        alpha_rlib = _write(deps / "libalpha-11111111.rlib", "alpha rlib")
        stdout = "\n".join(
            [
                _compiler_message("warning: sample warning\n"),
                _artifact(alpha_rlib),
                json.dumps({"reason": "build-finished", "success": True}),
            ]
        )
        proc = subprocess.CompletedProcess(["cargo"], 0, stdout=stdout)
        stderr = io.StringIO()
        with mock.patch.object(
            build_firmware, "run_command", return_value=proc
        ), contextlib.redirect_stderr(stderr):
            ll_paths = build_firmware._cargo_build(self.repo / "rust/Cargo.toml", [])

        self.assertEqual(ll_paths, [self.ll_a])
        self.assertEqual(stderr.getvalue(), "warning: sample warning\n")

    def test_cargo_build_renders_errors_on_failure_before_raising(self) -> None:
        stdout = "\n".join(
            [
                _compiler_message("warning: earlier warning\n"),
                _compiler_message(
                    "error[E0308]: mismatched types\n --> src/lib.rs:10:5\n",
                    level="error",
                ),
                json.dumps({"reason": "build-finished", "success": False}),
            ]
        )
        proc = subprocess.CompletedProcess(["cargo"], 101, stdout=stdout)
        stderr = io.StringIO()
        with mock.patch.object(
            build_firmware, "run_command", return_value=proc
        ), contextlib.redirect_stderr(stderr):
            with self.assertRaises(build_firmware.BuildError) as caught:
                build_firmware._cargo_build(self.repo / "rust/Cargo.toml", [])

        self.assertEqual(caught.exception.exit_code, 1)
        self.assertIn("cargo build failed with exit code 101", str(caught.exception))
        self.assertIn("warning: earlier warning", stderr.getvalue())
        self.assertIn("error[E0308]: mismatched types", stderr.getvalue())


class TestToolchainProvenance(_RepoCase):
    def test_parse_rustc_version_and_llc_version(self) -> None:
        rustc = build_firmware.parse_rustc_version(RUSTC_VV_SAMPLE)
        self.assertEqual(
            rustc,
            {
                "release": "1.102.0-nightly",
                "commit": "6905905bb7727fd0f9223f5fdc9ad4e62236d5a2",
                "llvm_version": "23.1.6",
                "llvm_major": 23,
            },
        )
        llc = build_firmware.parse_llc_version(LLC_VERSION_SAMPLE)
        self.assertEqual(llc, {"llvm_version": "23.1.7", "llvm_major": 23})

        with self.assertRaises(build_firmware.BuildError) as bad_rustc:
            build_firmware.parse_rustc_version("rustc 1.0.0\n")
        self.assertEqual(bad_rustc.exception.exit_code, 2)

        with self.assertRaises(build_firmware.BuildError) as bad_llc:
            build_firmware.parse_llc_version("no version line\n")
        self.assertEqual(bad_llc.exception.exit_code, 2)

    def test_probe_toolchain_accepts_older_or_equal_llvm(self) -> None:
        rust_dir = self.repo / "rust"
        calls: list[tuple[list[str], Path | None]] = []

        def fake_run(cmd, *, cwd=None, capture=False, env=None):
            argv = [str(part) for part in cmd]
            calls.append((argv, cwd))
            if argv[0] == "rustc":
                return subprocess.CompletedProcess(argv, 0, stdout=RUSTC_VV_SAMPLE)
            return subprocess.CompletedProcess(argv, 0, stdout=LLC_VERSION_SAMPLE)

        with mock.patch.object(build_firmware, "run_command", side_effect=fake_run):
            info = build_firmware.probe_toolchain(rust_dir, self.tools["llc"])

        self.assertEqual(info, FAKE_TOOLCHAIN)
        self.assertEqual(
            calls,
            [
                (["rustc", "-vV"], rust_dir),
                ([str(self.tools["llc"]), "--version"], None),
            ],
        )

        older_rustc = RUSTC_VV_SAMPLE.replace("23.1.6", "22.1.0")
        with mock.patch.object(
            build_firmware,
            "run_command",
            side_effect=[
                subprocess.CompletedProcess(["rustc"], 0, stdout=older_rustc),
                subprocess.CompletedProcess(["llc"], 0, stdout=LLC_VERSION_SAMPLE),
            ],
        ):
            older_info = build_firmware.probe_toolchain(rust_dir, self.tools["llc"])
        self.assertEqual(older_info["rustc_llvm_major"], 22)
        self.assertEqual(older_info["fork_llvm_major"], 23)

    def test_probe_toolchain_rejects_newer_rustc_llvm(self) -> None:
        newer_rustc = RUSTC_VV_SAMPLE.replace("23.1.6", "24.0.0")
        with mock.patch.object(
            build_firmware,
            "run_command",
            side_effect=[
                subprocess.CompletedProcess(["rustc"], 0, stdout=newer_rustc),
                subprocess.CompletedProcess(["llc"], 0, stdout=LLC_VERSION_SAMPLE),
            ],
        ):
            with self.assertRaises(build_firmware.BuildError) as caught:
                build_firmware.probe_toolchain(self.repo / "rust", self.tools["llc"])

        self.assertEqual(caught.exception.exit_code, 2)
        message = str(caught.exception)
        self.assertIn("24.0.0", message)
        self.assertIn("23.1.7", message)
        self.assertIn("retsimx/llvm-project", message)
        self.assertIn("rust/rust-toolchain.toml", message)

    def test_probe_toolchain_failure_raises_exit_2(self) -> None:
        with mock.patch.object(
            build_firmware,
            "run_command",
            side_effect=FileNotFoundError("rustc not found"),
        ):
            with self.assertRaises(build_firmware.BuildError) as caught:
                build_firmware.probe_toolchain(self.repo / "rust", self.tools["llc"])
        self.assertEqual(caught.exception.exit_code, 2)

        with mock.patch.object(
            build_firmware,
            "run_command",
            return_value=subprocess.CompletedProcess(["rustc"], 1, stdout=""),
        ):
            with self.assertRaises(build_firmware.BuildError) as caught:
                build_firmware.probe_toolchain(self.repo / "rust", self.tools["llc"])
        self.assertEqual(caught.exception.exit_code, 2)

    def test_manifest_invalidated_when_toolchain_changes(self) -> None:
        baseline = self.compute(toolchain=FAKE_TOOLCHAIN)
        for key, replacement in (
            ("rustc_release", "1.103.0-nightly"),
            ("rustc_commit", "deadbeef00000000000000000000000000000000"),
            ("rustc_llvm", "23.1.7"),
            ("fork_llvm", "23.1.8"),
        ):
            with self.subTest(changed=key):
                mutated = dict(FAKE_TOOLCHAIN)
                mutated[key] = replacement
                self.assertFalse(
                    build_firmware.manifests_equal(
                        baseline, self.compute(toolchain=mutated)
                    )
                )

    def test_run_build_probes_toolchain_prints_summary_and_hits_cache(self) -> None:
        stage = self.out / "default"
        _write(stage / "lightblemesh", b"elf")
        _write(stage / "lightblemesh.bin", b"bin")
        args = build_firmware._build_parser().parse_args(
            [
                "--llc",
                str(self.tools["llc"]),
                "--llvm-link",
                str(self.tools["llvm_link"]),
                "--opt",
                str(self.tools["opt"]),
            ]
        )
        seen: list[tuple[list[str], Path | None]] = []

        def fake_run(cmd, *, cwd=None, capture=False, env=None):
            argv = [str(part) for part in cmd]
            seen.append((argv, cwd))
            if argv == ["rustc", "-vV"]:
                return subprocess.CompletedProcess(argv, 0, stdout=RUSTC_VV_SAMPLE)
            if argv == [str(self.tools["llc"]), "--version"]:
                return subprocess.CompletedProcess(argv, 0, stdout=LLC_VERSION_SAMPLE)
            raise AssertionError(f"unexpected pipeline command: {argv}")

        with mock.patch.object(build_firmware, "REPO_ROOT", self.repo):
            self.startup_obj, self.startup_manifest = self._install_startup()
            build_firmware._write_json(
                stage / "manifest.json", self.compute(toolchain=FAKE_TOOLCHAIN)
            )
            with mock.patch.object(
                build_firmware, "_cargo_build", return_value=[self.ll_a, self.ll_b]
            ), mock.patch.object(
                build_firmware, "run_command", side_effect=fake_run
            ):
                printed = io.StringIO()
                with contextlib.redirect_stdout(printed):
                    self.assertEqual(build_firmware._run_build(args), 0)

        out = printed.getvalue()
        self.assertIn(
            "Toolchain: rustc 1.102.0-nightly (6905905bb, LLVM 23.1.6) -> llc LLVM 23.1.7",
            out,
        )
        self.assertIn(CACHE_HIT_MESSAGE, out)
        self.assertEqual(
            seen,
            [
                (["rustc", "-vV"], self.repo / "rust"),
                ([str(self.tools["llc"]), "--version"], None),
            ],
        )


if __name__ == "__main__":
    unittest.main()
