#!/usr/bin/env python3
"""Deterministic, incremental LLVM (TC32) build driver for the TLSR8266 firmware.

This replaces the IR/link half of the ``Makefile`` with a pure-LLVM pipeline:
Cargo emits LLVM IR, then ``llvm-link`` -> ``opt`` -> ``llc`` -> ``ld.lld`` ->
``llvm-objcopy`` produce the image. The vendored ``tc32-elf-*`` toolchain and the
text-assembly startup path are gone; reset vectors and startup code are emitted
from Rust (``rust/src/startup.rs``, ``global_asm!``) into an ordinary IR module.

The driver exists to fix two defects that made no-op and feature-switching
builds unsafe:

1. **Stale ``.ll`` glob.** ``llvm-link .../deps/*.ll`` linked every IR module ever
   emitted, so a Cargo metadata/feature change left old hashes beside the new
   ones and ``llvm-link`` failed with ``symbol multiply defined!`` until the
   tree was cleaned. This driver reads Cargo's ``--message-format=json`` stream
   and maps each reported ``.rlib`` to *its own* ``.ll`` (a hashed dependency
   crate, or the root crate's hardlink/``(size, sha256)`` twin under ``deps/``
   or ``build/*/*/out/``), so a stray ``stale-deadbeef.ll`` is never selected.
2. **Unconditional relink.** ``_build/lightblemesh`` was ``.PHONY``, so a no-op
   ``make`` still ran cargo plus the whole pipeline (~7.6 s wasted). A
   deterministic manifest of the exact inputs (IR file hashes, linker script,
   tool identity, flag strings, Cargo features) is compared before the pipeline;
   on a match the five commands are skipped and ``merged.o`` is left untouched.

Caching model
-------------
* ``_build/<feature_slug>/manifest.json`` records everything that can change
  the linked image. A cache hit requires parsed-JSON equality with the freshly
  computed manifest *and* the presence of both ``lightblemesh`` and
  ``lightblemesh.bin``. Anything else (edited source, rebuilt LLVM tool, changed
  flag string, different feature selection) misses and re-runs the pipeline.
* Feature selections are isolated per slug (``default``, ``rtt``, ...) so each
  keeps its own intermediates, while Cargo uses the single ``rust/target/``
  directory (run from ``rust/`` so rustup finds ``rust-toolchain.toml``); the
  exact-IR selection makes the stale-hash hazard impossible without a second
  target dir.
* ``rust/src/version.rs`` is the single source of truth for ``BUILD_VERSION``:
  the driver only reads it (``make bump-version`` stays the only mutator).

All subprocesses go through :func:`run_command`, and every relative path is
anchored at the repository root, so the driver is cwd-independent.

Usage:
    python3 sdk/build_firmware.py [--features <csv_or_space_list>]
        [--manifest-path rust/Cargo.toml] [--llc <path>] [--llvm-link <path>]
        [--opt <path>] [--ld <path>] [--objcopy <path>] [--out-dir _build]

LLVM tools resolve from ``--llc``/``--llvm-link``/``--opt``/``--ld``/
``--objcopy``, then ``LLC``/``LLVM_LINK``/``OPT``/``LLD``/``LLVM_OBJCOPY`` in
the environment, then repo-root ``.env`` (only keys not already in the
environment), then ``../llvm/build-native/bin/``, then ``../llvm/build/bin/``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import TextIO

REPO_ROOT = Path(__file__).resolve().parent.parent

SCHEMA_VERSION = 4
TARGET = "lightblemesh"
DEFAULT_MANIFEST = "rust/Cargo.toml"
DEFAULT_OUT_DIR = "_build"

# Pipeline flags. `OPT_PASSES`/`LLC_FLAGS`/`LLD_FLAGS` are recorded verbatim in
# the manifest, so a flag edit invalidates the cache.
OPT_INLINE_THRESHOLD = 50
# Size-first whole-program pipeline. `rustc` is built with
# `opt-level = "s"`/`codegen-units = 1`, so functions carry optsize/minisize
# attributes that `default<O2>` honours; `mergefunc` folds identical Rust
# monomorphisations and `constmerge`/`globaldce` drop duplicate constants and
# unreachable globals. There is no separate linker LTO step: every crate
# (including `core`) is linked into this one module before it is optimised.
OPT_PASSES = "internalize,default<O2>,mergefunc,constmerge,globaldce"

# The internalize public-API contract. Exact entries cover ABI, startup, IRQ,
# diagnostics/RTT, and runtime/libcall names that must survive whole-program
# optimization. Regex entries are deliberately narrow: they resolve only the
# emitted compiler-builtin, critical-section, and Rust panic/runtime families.
OPT_PRESERVE_ROOTS = (
    "__start",
    "__reset",
    "__irq",
    "irq_handler",
    "main_entrypoint",
    "startup_init",
    "HEARTBEAT",
    "irq_stk",
    "_TLSR_RTT",
    "__pender",
    "_critical_section_1_0_acquire",
    "_critical_section_1_0_release",
    "panic",
    "panic_fmt",
    "panic_bounds_check",
    "panic_cannot_unwind",
    "rust_begin_unwind",
    "rust_eh_personality",
    "_Unwind_Resume",
    "__rust_start_panic",
    "abort",
    "memcpy",
    "memmove",
    "memset",
    "memcmp",
    # Stable namespaced ABI/runtime anchors. These replace Rust-v0 mangled-name
    # fragments and are exported by the firmware source with #[export_name = ...].
    "tlsr8266_l2cap_att_handler",
    "tlsr8266_rf_link_slave_data_write",
    "tlsr8266_dispatch_ota_write",
    "tlsr8266_pair_write",
    "tlsr8266_enqueue_mesh_rx",
    "tlsr8266_enqueue_mesh_tx",
    "tlsr8266_handle_system_interrupts",
)
OPT_PRESERVE_ROOT_REGEXES = (
    r"^__aeabi_[A-Za-z0-9_]+$",
    r"^_critical_section_[0-9]+_[0-9]+_(?:acquire|release)$",
    r"^_R(?:N|I)v.*4core9panicking(?:[0-9]+)(?:panic_fmt|panic_bounds_check|panic_cannot_unwind)$",
    r"^_R(?:N|I)v.*(?:[0-9]+)?rust_(?:begin_unwind|eh_personality)$",
    r"^(?:rust_begin_unwind|rust_eh_personality|__rust_start_panic)$",
    r"^_Unwind_[A-Za-z0-9_]+$",
)
OPT_PRESERVE_API_NAME = "opt_preserve_api.txt"

LLC_FLAGS = "-march=thumb -mcpu=arm9 -mattr=+soft-float,-v6,+tc32"
LLD_FLAGS = "--tc32 --gc-sections -T ./sdk/boot.link"

VERSION_RS_REL = "rust/src/version.rs"
CARGO_MANIFEST_REL = "rust/Cargo.toml"
RUST_TOOLCHAIN_REL = "rust/rust-toolchain.toml"
CARGO_LOCK_REL = "rust/Cargo.lock"
WORKSPACE_CRATES_REL = "rust/crates"
LINKER_SCRIPT_REL = "sdk/boot.link"

# Ordered fallback candidates: `build-native` is the tree that ships `ld.lld`
# on this host, so it is probed before the plain `build` tree.
DEFAULT_TOOLS = {
    "llc": ("../llvm/build-native/bin/llc", "../llvm/build/bin/llc"),
    "llvm_link": ("../llvm/build-native/bin/llvm-link", "../llvm/build/bin/llvm-link"),
    "opt": ("../llvm/build-native/bin/opt", "../llvm/build/bin/opt"),
    "ld": ("../llvm/build-native/bin/ld.lld", "../llvm/build/bin/ld.lld"),
    "objcopy": (
        "../llvm/build-native/bin/llvm-objcopy",
        "../llvm/build/bin/llvm-objcopy",
    ),
}
TOOL_ENV_KEYS = {
    "llc": "LLC",
    "llvm_link": "LLVM_LINK",
    "opt": "OPT",
    "ld": "LLD",
    "objcopy": "LLVM_OBJCOPY",
}
TOOL_FLAGS = {
    "llc": "--llc",
    "llvm_link": "--llvm-link",
    "opt": "--opt",
    "ld": "--ld",
    "objcopy": "--objcopy",
}
TOOL_ORDER = ("llc", "llvm_link", "opt", "ld", "objcopy")
LLVM_TOOLS = ("llc", "llvm_link", "opt")
LINK_TOOLS = ("ld", "objcopy")

VERSION_RS_RE = re.compile(
    r"pub\s+(?:const|static)\s+BUILD_VERSION\s*:\s*u32\s*=\s*(\d+)\s*;"
)
# rustc emits IR as `<crate>-<metadata-hash>.ll` (no `lib` prefix), while the
# rlib that carries it is `lib<crate>-<hash>.rlib`.
HASHED_RLIB_RE = re.compile(r"^lib(?P<name>.+)-(?P<hash>[0-9a-f]{8,})\.rlib$")
PLAIN_RLIB_RE = re.compile(r"^lib(?P<name>.+)\.rlib$")
SKIP_ARTIFACT_KINDS = frozenset({"proc-macro", "custom-build"})

RUSTC_RELEASE_RE = re.compile(r"^release:\s*(\S+)\s*$", re.MULTILINE)
RUSTC_COMMIT_RE = re.compile(r"^commit-hash:\s*(\S+)\s*$", re.MULTILINE)
RUSTC_LLVM_RE = re.compile(
    r"^LLVM version:\s*(?P<version>(?P<major>\d+)(?:\.\d+)*\S*)\s*$",
    re.MULTILINE,
)
LLC_LLVM_RE = re.compile(
    r"^\s*LLVM version\s+(?P<version>(?P<major>\d+)(?:\.\d+)*\S*)\s*$",
    re.MULTILINE,
)


class BuildError(RuntimeError):
    """A user-actionable build failure.

    ``exit_code`` is 2 for usage/configuration problems (missing tool, version
    mismatch) and 1 for a failed build step.
    """

    def __init__(self, message: str, exit_code: int = 1) -> None:
        super().__init__(message)
        self.exit_code = exit_code


# ---------------------------------------------------------------------------
# Feature selection
# ---------------------------------------------------------------------------


def parse_features(raw: str | None) -> list[str]:
    """Split a comma/space separated feature list into a sorted, unique list."""
    tokens = re.split(r"[\s,]+", (raw or "").strip())
    return sorted({token for token in tokens if token})


def feature_slug(features: Sequence[str]) -> str:
    """Return the per-feature output directory name (``default`` when empty)."""
    return "-".join(features) if features else "default"


# ---------------------------------------------------------------------------
# Tool resolution
# ---------------------------------------------------------------------------


def parse_dotenv(path: Path) -> dict[str, str]:
    """Parse a minimal ``KEY=VALUE`` dotenv file.

    Blank lines and ``#`` comments are ignored, one leading ``export `` is
    stripped, values are split on the first ``=`` and surrounding single or
    double quotes are removed. Nothing is written to ``os.environ``.
    """
    dotenv_path = Path(path)
    if not dotenv_path.is_file():
        return {}

    values: dict[str, str] = {}
    for raw_line in dotenv_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :].lstrip()
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        if key:
            values[key] = value
    return values


def _anchor(value: str, repo_root: Path) -> Path:
    """Anchor a flag/env/default path at the repository root when relative."""
    path = Path(value).expanduser()
    return path if path.is_absolute() else Path(repo_root) / path


def _is_executable(path: Path) -> bool:
    """Whether ``path`` is a regular file this process may execute.

    One predicate for both ends of resolution: the default-candidate probe and
    the fail-closed check must agree on what "usable tool" means.
    """
    return path.is_file() and os.access(path, os.X_OK)


def resolve_tool(
    cli_value: str | None,
    env_key: str,
    dotenv: dict[str, str],
    defaults: Sequence[str],
    repo_root: Path,
) -> Path:
    """Resolve a tool path: CLI flag > environment > ``.env`` > ordered defaults.

    ``defaults`` is an ordered candidate list (``build-native`` before
    ``build``); the first candidate that exists and is executable wins. When
    none exists the first candidate is returned so :func:`_require_tool` fails
    closed and reports every path that was checked.
    """
    for candidate in (cli_value, os.environ.get(env_key), dotenv.get(env_key)):
        value = (candidate or "").strip()
        if value:
            return _anchor(value, repo_root)
    anchored = tuple(_anchor(default, repo_root) for default in defaults)
    for path in anchored:
        if _is_executable(path):
            return path
    return anchored[0]


def _require_tool(
    path: Path,
    flag: str,
    env_key: str,
    checked: Sequence[Path] = (),
) -> Path:
    """Fail loudly when a resolved tool is absent or not executable."""
    tool = Path(path)
    if not _is_executable(tool):
        tried = tuple(checked) or (tool,)
        raise BuildError(
            f"{flag}: tool is missing or not executable: "
            f"{', '.join(str(candidate) for candidate in tried)} "
            f"(cwd: {Path.cwd()}); pass {flag} <path> or set {env_key}",
            exit_code=2,
        )
    return tool


# ---------------------------------------------------------------------------
# Version gate (read-only)
# ---------------------------------------------------------------------------


def _read_version(path: Path, pattern: re.Pattern[str], expected: str) -> int:
    source = Path(path)
    try:
        text = source.read_text(encoding="utf-8")
    except OSError as exc:
        raise BuildError(
            f"cannot read version file {source}: {exc}; expected {expected}",
            exit_code=2,
        ) from exc
    match = pattern.search(text)
    if match is None:
        raise BuildError(
            f"no BUILD_VERSION in {source}; expected {expected}",
            exit_code=2,
        )
    return int(match.group(1))


def read_build_version(repo_root: Path) -> int:
    """Read ``BUILD_VERSION`` from ``rust/src/version.rs``.

    That file is the single source of truth: there is no second version file to
    cross-check. Raises :class:`BuildError` when it is missing or unparsable, and
    the driver never rewrites it.
    """
    return _read_version(
        Path(repo_root) / VERSION_RS_REL,
        VERSION_RS_RE,
        "pub const BUILD_VERSION: u32 = <N>;",
    )


# ---------------------------------------------------------------------------
# Toolchain provenance & LLVM version gate
# ---------------------------------------------------------------------------


def parse_rustc_version(output: str) -> dict[str, object]:
    """Parse ``rustc -vV`` output into release, commit, and LLVM version."""
    release_match = RUSTC_RELEASE_RE.search(output)
    commit_match = RUSTC_COMMIT_RE.search(output)
    llvm_match = RUSTC_LLVM_RE.search(output)
    if release_match is None or commit_match is None or llvm_match is None:
        raise BuildError(
            "cannot parse `rustc -vV` output; expected release, commit-hash, "
            "and LLVM version lines",
            exit_code=2,
        )
    return {
        "release": release_match.group(1),
        "commit": commit_match.group(1),
        "llvm_version": llvm_match.group("version"),
        "llvm_major": int(llvm_match.group("major")),
    }


def parse_llc_version(output: str) -> dict[str, object]:
    """Parse ``llc --version`` output into LLVM version and major integer."""
    llvm_match = LLC_LLVM_RE.search(output)
    if llvm_match is None:
        raise BuildError(
            "cannot parse `llc --version` output; expected `LLVM version X.Y.Z`",
            exit_code=2,
        )
    return {
        "llvm_version": llvm_match.group("version"),
        "llvm_major": int(llvm_match.group("major")),
    }


def _probe_stdout(
    cmd: Sequence[str | Path], *, cwd: Path | None = None, what: str
) -> str:
    try:
        proc = run_command(cmd, cwd=cwd, capture=True)
    except OSError as exc:
        raise BuildError(f"cannot execute {what}: {exc}", exit_code=2) from exc
    if proc.returncode != 0:
        raise BuildError(
            f"{what} failed with exit code {proc.returncode}",
            exit_code=2,
        )
    return proc.stdout or ""


def _probe_tc32_feature(llc_path: Path) -> None:
    """Fail if the resolved llc does not actually implement ``+tc32``.

    An older or stock llc accepts the flag with a warning on stderr and exits 0,
    then emits *unscrambled* Thumb bytes: a silently unbootable image. Refuse
    to build in that case.
    """
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        src = Path(tmp) / "tc32.ll"
        obj = Path(tmp) / "tc32.o"
        src.write_text("define void @f() { ret void }\n", encoding="utf-8")
        try:
            proc = run_command(
                [
                    str(llc_path),
                    "-march=thumb",
                    "-mcpu=arm9",
                    "-mattr=+soft-float,-v6,+tc32",
                    "-filetype=obj",
                    str(src),
                    "-o",
                    str(obj),
                ],
                capture=True,
                capture_stderr=True,
            )
        except OSError as exc:
            raise BuildError(f"cannot execute {llc_path}: {exc}", exit_code=2) from exc
    stderr = proc.stderr or ""
    if proc.returncode != 0 or "not a recognized feature" in stderr:
        raise BuildError(
            f"{llc_path} does not support the +tc32 subtarget feature; "
            "point --llc/LLC at the TC32 LLVM fork (llc --version should report "
            "this fork's build)",
            exit_code=2,
        )


def probe_toolchain(rust_dir: Path, llc_path: Path) -> dict[str, object]:
    """Probe ``rustc -vV`` and ``llc --version`` and enforce LLVM compatibility.

    ``rustc`` runs inside ``rust_dir`` so ``rustup`` honours
    ``rust/rust-toolchain.toml``. Raises :class:`BuildError` with exit code 2
    if ``rustc`` targets a newer LLVM major version than the ``tc32`` fork.
    """
    rustc_info = parse_rustc_version(
        _probe_stdout(["rustc", "-vV"], cwd=Path(rust_dir), what="rustc -vV")
    )
    llc_info = parse_llc_version(
        _probe_stdout([str(llc_path), "--version"], what=f"{llc_path} --version")
    )
    _probe_tc32_feature(llc_path)
    rustc_major = int(rustc_info["llvm_major"])
    fork_major = int(llc_info["llvm_major"])
    if rustc_major > fork_major:
        raise BuildError(
            f"LLVM version mismatch: rustc {rustc_info['release']} emits LLVM "
            f"{rustc_info['llvm_version']} IR (major {rustc_major}), which is "
            f"newer than the llc TC32 fork LLVM {llc_info['llvm_version']} "
            f"(major {fork_major}); rebase retsimx/llvm-project (tc32 branch) "
            "or pin rust/rust-toolchain.toml to an earlier nightly",
            exit_code=2,
        )
    return {
        "rustc_release": rustc_info["release"],
        "rustc_commit": rustc_info["commit"],
        "rustc_llvm": rustc_info["llvm_version"],
        "rustc_llvm_major": rustc_major,
        "fork_llvm": llc_info["llvm_version"],
        "fork_llvm_major": fork_major,
    }


# ---------------------------------------------------------------------------
# Cargo diagnostics & artifacts -> exact .ll paths
# ---------------------------------------------------------------------------


def render_cargo_messages(
    stdout_lines: Iterable[str] | str, *, stream: TextIO | None = None
) -> int:
    """Render Cargo JSON ``compiler-message`` diagnostics to ``stream``.

    Returns the number of non-empty rendered messages written. Non-JSON lines
    and other Cargo JSON reasons are ignored.
    """
    lines = stdout_lines.splitlines() if isinstance(stdout_lines, str) else stdout_lines
    target = sys.stderr if stream is None else stream
    rendered_count = 0
    for line in lines:
        stripped = line.strip()
        if not stripped.startswith("{"):
            continue
        try:
            record = json.loads(stripped)
        except json.JSONDecodeError:
            continue
        if record.get("reason") != "compiler-message":
            continue
        message = record.get("message")
        if not isinstance(message, dict):
            continue
        rendered = message.get("rendered")
        if isinstance(rendered, str) and rendered.strip():
            target.write(rendered if rendered.endswith("\n") else f"{rendered}\n")
            rendered_count += 1
    return rendered_count


def parse_cargo_artifacts(stdout_lines: Iterable[str] | str) -> list[Path]:
    """Return the target ``.rlib`` artifacts reported by Cargo's JSON stream.

    Only ``compiler-artifact`` messages are considered, build-script and
    proc-macro targets are skipped, and only ``.rlib`` files emitted for
    ``thumbv6m-none-eabi`` are kept.
    """
    lines = stdout_lines.splitlines() if isinstance(stdout_lines, str) else stdout_lines
    rlibs: list[Path] = []
    for line in lines:
        stripped = line.strip()
        if not stripped.startswith("{"):
            continue
        try:
            message = json.loads(stripped)
        except json.JSONDecodeError:
            continue
        if message.get("reason") != "compiler-artifact":
            continue
        target = message.get("target") or {}
        if SKIP_ARTIFACT_KINDS & set(target.get("kind") or ()):
            continue
        for filename in message.get("filenames") or ():
            if filename.endswith(".rlib") and "thumbv6m-none-eabi" in filename:
                rlibs.append(Path(filename))
    return rlibs


def _require_ll(rlib: Path, ll_path: Path) -> Path:
    """Return ``ll_path`` when it exists, naming its ``.rlib`` when it does not."""
    if not ll_path.is_file():
        raise BuildError(
            f"no LLVM IR module for {rlib}: expected {ll_path}; "
            "run `cargo build` with RUSTFLAGS='--emit=llvm-ir'",
            exit_code=1,
        )
    return ll_path


def _ll_for_hashed_rlib(rlib: Path) -> Path:
    """Map a hashed ``lib<name>-<hash>.rlib`` to its sibling ``<name>-<hash>.ll``."""
    match = HASHED_RLIB_RE.match(rlib.name)
    assert match is not None  # caller checked
    return rlib.parent / f"{match.group('name')}-{match.group('hash')}.ll"


def _match_inode(target: Path, candidates: Sequence[Path]) -> Path | None:
    """Find the hashed sibling rlib that is the same inode as the root rlib.

    Cargo hardlinks ``release/lib<name>.rlib`` to its hashed sibling on
    filesystems that support hardlinks; that is the strongest identity match.
    """
    target_stat = target.stat()
    for candidate in candidates:
        candidate_stat = candidate.stat()
        if (
            candidate_stat.st_dev == target_stat.st_dev
            and candidate_stat.st_ino == target_stat.st_ino
        ):
            return candidate
    return None


def _match_content(target: Path, candidates: Sequence[Path]) -> Path | None:
    """Fallback identity match by ``(st_size, sha256)`` for copied rlibs.

    Ties are broken deterministically: newest ``st_mtime_ns`` first, then the
    lexically smallest path, so the same tree always resolves the same ``.ll``.
    """
    target_stat = target.stat()
    target_digest = _sha256(target)
    matches = [
        candidate
        for candidate in candidates
        if candidate.stat().st_size == target_stat.st_size
        and _sha256(candidate) == target_digest
    ]
    if not matches:
        return None
    matches.sort(key=lambda path: (-path.stat().st_mtime_ns, path.as_posix()))
    return matches[0]


def _root_ll_path(rlib: Path, name: str) -> Path:
    """Map an unhashed ``release/lib<name>.rlib`` to its hashed IR module."""
    deps_dir = rlib.parent / "deps"
    build_dir = rlib.parent / "build"
    candidates = sorted(
        path
        for path in {
            *deps_dir.glob(f"lib{name}-*.rlib"),
            *build_dir.glob(f"*/*/out/lib{name}-*.rlib"),
        }
        if (matched := HASHED_RLIB_RE.match(path.name)) is not None
        and matched.group("name") == name
    )
    if not candidates:
        raise BuildError(
            f"cannot resolve the IR module for {rlib}: no lib{name}-*.rlib "
            f"candidate under {deps_dir} or {build_dir}; run `cargo clean` and rebuild",
            exit_code=1,
        )
    match = _match_inode(rlib, candidates) or _match_content(rlib, candidates)
    if match is None:
        raise BuildError(
            f"cannot resolve the IR module for {rlib}: no hashed deps/ or build/ "
            "rlib matches it by inode or by (size, sha256); run `cargo clean` "
            "and rebuild",
            exit_code=1,
        )
    return _ll_for_hashed_rlib(match)


def resolve_ll_paths(rlibs: Sequence[Path]) -> list[Path]:
    """Map each reported rlib to its exact ``.ll``, sorted and deduplicated."""
    resolved: list[Path] = []
    for raw_rlib in rlibs:
        rlib = Path(raw_rlib)
        hashed = HASHED_RLIB_RE.match(rlib.name)
        if hashed is not None:
            resolved.append(_require_ll(rlib, _ll_for_hashed_rlib(rlib)))
            continue
        plain = PLAIN_RLIB_RE.match(rlib.name)
        if plain is None:
            raise BuildError(f"unrecognised rlib name: {rlib}", exit_code=1)
        resolved.append(_require_ll(rlib, _root_ll_path(rlib, plain.group("name"))))
    return sorted(set(resolved))


# ---------------------------------------------------------------------------
# Manifest
# ---------------------------------------------------------------------------


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with open(path, "rb") as handle:
            for chunk in iter(lambda: handle.read(1 << 16), b""):
                digest.update(chunk)
    except OSError as exc:
        raise BuildError(f"cannot hash {path}: {exc}", exit_code=2) from exc
    return digest.hexdigest()


def _display_path(path: Path) -> str:
    """Represent a path relative to the repo root when possible."""
    source = Path(path)
    try:
        return source.resolve().relative_to(REPO_ROOT).as_posix()
    except (OSError, ValueError):
        return source.as_posix()


def _input_entries(inputs: Sequence[Path]) -> list[dict[str, str]]:
    """Map each input to its repo-relative path and SHA-256."""
    return [{"path": _display_path(path), "sha256": _sha256(path)} for path in inputs]


def _read_json(path: Path) -> dict | None:
    """Parsed JSON object, or ``None`` when absent, corrupt or not an object."""
    try:
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _write_json(path: Path, payload: dict) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _tool_entry(path: Path) -> dict[str, object]:
    tool = Path(path)
    try:
        stat = tool.stat()
    except OSError as exc:
        raise BuildError(f"tool not found: {tool} ({exc})", exit_code=2) from exc
    return {"path": str(tool), "mtime_ns": stat.st_mtime_ns, "size": stat.st_size}


LLVM_SYMBOL_RE = re.compile(r'@(?:"([^"]+)"|([A-Za-z0-9_.$-]+))')


def resolve_preserve_roots(ll_paths: Sequence[Path]) -> list[str]:
    """Resolve the exact roots and narrow emitted-name patterns against linked IR."""
    resolved = set(OPT_PRESERVE_ROOTS)
    linked_names: set[str] = set()
    for ll_path in ll_paths:
        source = Path(ll_path)
        try:
            text = source.read_text(encoding="utf-8")
        except OSError as exc:
            raise BuildError(
                f"cannot read linked IR for preserve-root resolution: {source}: {exc}",
                exit_code=2,
            ) from exc
        for match in LLVM_SYMBOL_RE.finditer(text):
            name = match.group(1) or match.group(2)
            if name:
                linked_names.add(name)

    compiled = tuple(re.compile(pattern) for pattern in OPT_PRESERVE_ROOT_REGEXES)
    for name in linked_names:
        if any(pattern.search(name) is not None for pattern in compiled):
            resolved.add(name)
    return sorted(resolved)


def render_opt_public_api(resolved_roots: Sequence[str]) -> str:
    """Render a one-symbol-per-line LLVM opt public-API file."""
    return "".join(f"{name}\n" for name in sorted(set(resolved_roots)))


def write_opt_public_api_file(stage_dir: Path, ll_paths: Sequence[Path]) -> Path:
    """Generate the per-feature file consumed by llvm opt's internalize pass."""
    destination = Path(stage_dir) / OPT_PRESERVE_API_NAME
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        render_opt_public_api(resolve_preserve_roots(ll_paths)), encoding="utf-8"
    )
    return destination


def _flags_block(ll_paths: Sequence[Path] = ()) -> dict[str, object]:
    """The pipeline/preserve contract strings that participate in cache invalidation."""
    resolved = resolve_preserve_roots(ll_paths)
    rendered_api = render_opt_public_api(resolved)
    return {
        "opt_passes": OPT_PASSES,
        "opt_inline_threshold": OPT_INLINE_THRESHOLD,
        "preserve_roots": sorted(set(OPT_PRESERVE_ROOTS)),
        "preserve_regexes": list(OPT_PRESERVE_ROOT_REGEXES),
        "resolved_preserve_roots": resolved,
        "resolved_preserve_roots_sha256": hashlib.sha256(
            rendered_api.encode("utf-8")
        ).hexdigest(),
        "llc_flags": LLC_FLAGS,
        "ldflags": LLD_FLAGS,
    }


def compute_manifest(
    features: Sequence[str],
    ll_paths: Sequence[Path],
    version: int,
    build_inputs: Sequence[Path],
    linker_script: Path,
    tools: dict[str, Path],
    toolchain: dict[str, object] | None = None,
) -> dict:
    """Build the deterministic manifest that guards the LLVM pipeline."""
    missing = [name for name in TOOL_ORDER if name not in tools]
    if missing:
        raise BuildError(f"manifest is missing tool entries: {missing}", exit_code=2)

    ll_inputs = _input_entries(ll_paths)
    ll_inputs.sort(key=lambda entry: entry["path"])
    return {
        "schema_version": SCHEMA_VERSION,
        "features": list(features),
        "build_version": version,
        "build_inputs": _input_entries(build_inputs),
        "ll_inputs": ll_inputs,
        "linker_script_sha256": _sha256(linker_script),
        "tools": {name: _tool_entry(tools[name]) for name in TOOL_ORDER},
        "toolchain": dict(toolchain) if toolchain is not None else {},
        "flags": _flags_block(ll_paths),
    }


def manifests_equal(first: dict | None, second: dict | None) -> bool:
    """Cache-hit predicate: parsed-JSON equality, never a partial/absent match."""
    return isinstance(first, dict) and isinstance(second, dict) and first == second


# ---------------------------------------------------------------------------
# Output sync
# ---------------------------------------------------------------------------


def sync_outputs(src: Path, dst: Path) -> None:
    """Copy ``src`` over ``dst`` via ``copy2`` only when the content differs."""
    source, destination = Path(src), Path(dst)
    if destination.is_file() and _sha256(destination) == _sha256(source):
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


# ---------------------------------------------------------------------------
# Command execution
# ---------------------------------------------------------------------------


def run_command(
    cmd: Sequence[str | Path],
    *,
    cwd: Path | None = None,
    capture: bool = False,
    capture_stderr: bool = False,
    env: dict[str, str] | None = None,
) -> subprocess.CompletedProcess:
    """Run one argv list (never a shell string).

    Every subprocess in this module goes through this function so the test
    suite can patch a single symbol.
    """
    return subprocess.run(
        [str(part) for part in cmd],
        cwd=None if cwd is None else str(cwd),
        env=env,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.PIPE if capture_stderr else None,
        text=True,
        check=False,
    )


def _run_step(
    cmd: Sequence[str | Path],
    *,
    cwd: Path,
    env: dict[str, str] | None = None,
    capture: bool = False,
    what: str = "command",
) -> subprocess.CompletedProcess:
    """Run one build step, raising :class:`BuildError` on a non-zero exit."""
    proc = run_command(cmd, cwd=cwd, capture=capture, env=env)
    if proc.returncode != 0:
        rendered = " ".join(str(part) for part in cmd)
        raise BuildError(
            f"{what} failed with exit code {proc.returncode}: {rendered}",
            exit_code=1,
        )
    return proc


# ---------------------------------------------------------------------------
# Build stages
# ---------------------------------------------------------------------------


def _cargo_build(manifest_path: Path, features: Sequence[str]) -> list[Path]:
    """Run Cargo from the manifest's directory and resolve the exact IR inputs."""
    cmd = [
        "cargo",
        "build",
        "--package",
        "tlsr8266_mesh",
        "--lib",
        "--release",
        "-Z",
        "build-std=core",
        "--target",
        "thumbv6m-none-eabi",
        "--message-format=json",
    ]
    if features:
        cmd += ["--features", ",".join(features)]
    env = dict(os.environ)
    env.pop("CARGO_ENCODED_RUSTFLAGS", None)
    env["RUSTFLAGS"] = "--emit=llvm-ir -C target-feature=+tc32"
    proc = run_command(
        cmd,
        cwd=Path(manifest_path).parent,
        capture=True,
        env=env,
    )
    render_cargo_messages(proc.stdout or "")
    if proc.returncode != 0:
        rendered = " ".join(str(part) for part in cmd)
        raise BuildError(
            f"cargo build failed with exit code {proc.returncode}: {rendered}",
            exit_code=1,
        )
    rlibs = parse_cargo_artifacts(proc.stdout or "")
    return resolve_ll_paths(rlibs)


def _stage_paths(stage_dir: Path) -> dict[str, str]:
    """The per-feature intermediates and outputs, as argv-ready strings."""
    stage = Path(stage_dir)
    return {
        "bc": str(stage / "merged.bc"),
        "opt_bc": str(stage / "merged_opt.bc"),
        "obj": str(stage / "merged.o"),
        "elf": str(stage / TARGET),
        "bin": str(stage / f"{TARGET}.bin"),
        "preserve_api": str(stage / OPT_PRESERVE_API_NAME),
    }


def _pipeline_commands(
    stage_dir: Path,
    ll_paths: Sequence[Path],
    tools: dict[str, Path],
    version: int,
) -> list[tuple[str, list[str]]]:
    """The five pure-LLVM commands that turn IR into the flashable image."""
    paths = _stage_paths(stage_dir)
    return [
        (
            "llvm-link",
            [str(tools["llvm_link"]), *(str(p) for p in ll_paths), "-o", paths["bc"]],
        ),
        (
            "opt",
            [
                str(tools["opt"]),
                f"-passes={OPT_PASSES}",
                f"-inline-threshold={OPT_INLINE_THRESHOLD}",
                f"-internalize-public-api-file={paths['preserve_api']}",
                paths["bc"],
                "-o",
                paths["opt_bc"],
            ],
        ),
        (
            "llc",
            [
                str(tools["llc"]),
                *LLC_FLAGS.split(),
                "--filetype=obj",
                "--function-sections",
                "--data-sections",
                paths["opt_bc"],
                "-o",
                paths["obj"],
            ],
        ),
        (
            "ld",
            [
                str(tools["ld"]),
                *LLD_FLAGS.split(),
                f"--defsym=BUILD_VERSION={version}",
                "-o",
                paths["elf"],
                paths["obj"],
            ],
        ),
        (
            "objcopy",
            [str(tools["objcopy"]), "-O", "binary", paths["elf"], paths["bin"]],
        ),
    ]


# ---------------------------------------------------------------------------
# Orchestration
# ---------------------------------------------------------------------------


def _resolve_tools(
    args: argparse.Namespace, dotenv: dict[str, str], repo_root: Path
) -> dict[str, Path]:
    """Resolve every pipeline tool: CLI flag > environment > ``.env`` > default."""
    resolved: dict[str, Path] = {}
    for name in (*LLVM_TOOLS, *LINK_TOOLS):
        # Anchor once: this tuple is both what `resolve_tool` probes and what
        # the failure message names, so the two can never drift apart.
        candidates = tuple(
            _anchor(default, repo_root) for default in DEFAULT_TOOLS[name]
        )
        path = resolve_tool(
            getattr(args, name),
            TOOL_ENV_KEYS[name],
            dotenv,
            candidates,
            repo_root,
        )
        resolved[name] = _require_tool(
            path,
            TOOL_FLAGS[name],
            TOOL_ENV_KEYS[name],
            candidates if path in candidates else (),
        )
    return resolved


def _build_inputs(repo_root: Path) -> list[Path]:
    """The repo files whose content guards the LLVM pipeline cache.

    The root manifest and the toolchain pin always participate. The lockfile and
    the per-crate manifests are optional so a fresh checkout that has not
    resolved dependencies (or a workspace with no member crates) still builds.
    """
    root = Path(repo_root)
    inputs = [root / CARGO_MANIFEST_REL, root / RUST_TOOLCHAIN_REL]
    lockfile = root / CARGO_LOCK_REL
    if lockfile.is_file():
        inputs.append(lockfile)
    crates_dir = root / WORKSPACE_CRATES_REL
    if crates_dir.is_dir():
        inputs.extend(sorted(crates_dir.glob("*/Cargo.toml")))
    return inputs


def _run_build(args: argparse.Namespace) -> int:
    repo_root = REPO_ROOT
    out_root = _anchor(args.out_dir, repo_root)
    manifest_path = _anchor(args.manifest_path, repo_root)
    features = parse_features(args.features)
    slug = feature_slug(features)

    build_version = read_build_version(repo_root)

    dotenv = {
        key: value
        for key, value in parse_dotenv(repo_root / ".env").items()
        if key not in os.environ
    }
    tools = _resolve_tools(args, dotenv, repo_root)

    toolchain = probe_toolchain(manifest_path.parent, tools["llc"])
    commit_short = str(toolchain["rustc_commit"])[:9]
    print(
        f"Toolchain: rustc {toolchain['rustc_release']} "
        f"({commit_short}, LLVM {toolchain['rustc_llvm']}) -> "
        f"llc LLVM {toolchain['fork_llvm']}"
    )

    ll_paths = _cargo_build(manifest_path, features)

    linker_script = repo_root / LINKER_SCRIPT_REL
    build_inputs = _build_inputs(repo_root)
    stage_dir = out_root / slug
    stage = _stage_paths(stage_dir)
    write_opt_public_api_file(stage_dir, ll_paths)

    manifest = compute_manifest(
        features,
        ll_paths,
        build_version,
        build_inputs,
        linker_script,
        tools,
        toolchain=toolchain,
    )
    target = Path(stage["elf"])
    target_bin = Path(stage["bin"])
    cache_hit = (
        manifests_equal(_read_json(stage_dir / "manifest.json"), manifest)
        and target.is_file()
        and target_bin.is_file()
    )
    if cache_hit:
        print(f"Build manifest unchanged for '{slug}'; skipping LLVM/TC32 link.")
    else:
        stage_dir.mkdir(parents=True, exist_ok=True)
        for label, cmd in _pipeline_commands(stage_dir, ll_paths, tools, build_version):
            _run_step(cmd, cwd=repo_root, what=label)
        _write_json(stage_dir / "manifest.json", manifest)

    sync_outputs(target, out_root / TARGET)
    sync_outputs(target_bin, out_root / f"{TARGET}.bin")
    return 0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="build_firmware.py",
        description=(
            "Deterministic, incremental LLVM/TC32 build driver for the "
            "TLSR8266 firmware. Relative paths are anchored at the repository root."
        ),
    )
    parser.add_argument(
        "--features",
        default="",
        metavar="LIST",
        help="Cargo features as a comma/space separated list (normalised, "
        "sorted; default: none).",
    )
    parser.add_argument(
        "--manifest-path",
        default=DEFAULT_MANIFEST,
        metavar="PATH",
        help=f"Cargo manifest to build (default: {DEFAULT_MANIFEST}).",
    )
    parser.add_argument(
        "--llc",
        default="",
        metavar="PATH",
        help="llc binary (default: $LLC, .env LLC, then "
        "../llvm/build-native/bin/llc, then ../llvm/build/bin/llc).",
    )
    parser.add_argument(
        "--llvm-link",
        default="",
        metavar="PATH",
        help="llvm-link binary (default: $LLVM_LINK, .env LLVM_LINK, then "
        "../llvm/build-native/bin/llvm-link, then ../llvm/build/bin/llvm-link).",
    )
    parser.add_argument(
        "--opt",
        default="",
        metavar="PATH",
        help="opt binary (default: $OPT, .env OPT, then "
        "../llvm/build-native/bin/opt, then ../llvm/build/bin/opt).",
    )
    parser.add_argument(
        "--ld",
        default="",
        metavar="PATH",
        help="ld.lld linker (default: $LLD, .env LLD, then "
        "../llvm/build-native/bin/ld.lld, then ../llvm/build/bin/ld.lld).",
    )
    parser.add_argument(
        "--objcopy",
        default="",
        metavar="PATH",
        help="llvm-objcopy binary (default: $LLVM_OBJCOPY, .env LLVM_OBJCOPY, "
        "then ../llvm/build-native/bin/llvm-objcopy, then "
        "../llvm/build/bin/llvm-objcopy).",
    )
    parser.add_argument(
        "--out-dir",
        default=DEFAULT_OUT_DIR,
        metavar="DIR",
        help=f"output root (default: {DEFAULT_OUT_DIR}).",
    )
    return parser


def main(argv: list[str]) -> int:
    """Run the driver; return a process exit code (2 = usage, 1 = build)."""
    args = _build_parser().parse_args(argv)
    try:
        return _run_build(args)
    except BuildError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return exc.exit_code


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
