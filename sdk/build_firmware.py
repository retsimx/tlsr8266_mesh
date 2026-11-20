#!/usr/bin/env python3
"""Deterministic, incremental TC32 (LLVM IR) build driver for the TLSR8266 firmware.

This replaces the IR/link half of the ``Makefile`` and fixes the three defects
that made no-op and feature-switching builds unsafe:

1. **Stale ``.ll`` glob.** ``llvm-link .../deps/*.ll`` linked every IR module ever
   emitted, so a Cargo metadata/feature change left old hashes beside the new
   ones and ``llvm-link`` failed with ``symbol multiply defined!`` until the
   tree was cleaned. This driver reads Cargo's ``--message-format=json`` stream
   and maps each reported ``.rlib`` to *its own* ``.ll`` (a hashed dependency
   crate, or the root crate's hardlink/``(size, sha256)`` twin under ``deps/``
   or ``build/*/*/out/``), so a stray ``stale-deadbeef.ll`` is never selected.
2. **Unconditional relink.** ``_build/lightblemesh`` was ``.PHONY``, so a no-op
   ``make`` still ran cargo, ``llvm-link``, ``opt``, ``llc``, ``as``, ``ld`` and
   ``objcopy`` (~7.6 s wasted). A deterministic manifest of the exact inputs (IR
   file hashes, startup object, linker script, libgcc, tool identity, flag
   strings) is compared before the pipeline; on a match the six commands are
   skipped and ``merged.o`` is left untouched.
3. **Dirty tree on build.** The startup-object recipe unconditionally ran
   ``sdk/increment_version.sh``, mutating tracked ``sdk/version.in`` and
   ``rust/src/version.rs``. This driver only *reads* the two version files,
   checks that their ``BUILD_VERSION`` integers agree, and rebuilds
   ``cstartup_8266.o`` from a SHA-256 manifest. ``make bump-version`` remains the
   only mutator.

Caching model
-------------
* ``_build/asm/cstartup_8266.o`` is rebuilt only when it or
  ``_build/asm/cstartup_8266.manifest.json`` is missing, or when the SHA-256 of
  ``sdk/cstartup_8266.S``, ``sdk/version.in`` or ``sdk/version.h`` changed.
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

All subprocesses go through :func:`run_command`, and every relative path is
anchored at the repository root, so the driver is cwd-independent.

Usage:
    python3 sdk/build_firmware.py [--features <csv_or_space_list>]
        [--manifest-path rust/Cargo.toml] [--llc <path>] [--llvm-link <path>]
        [--opt <path>] [--out-dir _build]

LLVM tools resolve from ``--llc``/``--llvm-link``/``--opt``, then ``LLC``/
``LLVM_LINK``/``OPT`` in the environment, then repo-root ``.env`` (only keys not
already in the environment), then ``../llvm/build/bin/``.
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

SCHEMA_VERSION = 1
TARGET = "lightblemesh"
DEFAULT_MANIFEST = "rust/Cargo.toml"
DEFAULT_OUT_DIR = "_build"

# Exact flags from the pre-driver Makefile. `OPT_PASSES`/`LLC_FLAGS`/`LDFLAGS`
# are recorded verbatim in the manifest, so a flag edit invalidates the cache.
OPT_INLINE_THRESHOLD = 50
OPT_SAFE = (
    "sroa,mem2reg,instcombine<no-verify-fixpoint>,simplifycfg,dce,"
    "loop-simplify,lcssa,loop-rotate,loop-mssa(licm),"
    "instcombine<no-verify-fixpoint>,simplifycfg,adce,dse"
)
OPT_PASSES = (
    f"ipsccp,deadargelim,function({OPT_SAFE}),always-inline,cgscc(inline),"
    f"ipsccp,deadargelim,function({OPT_SAFE})"
)
LLC_FLAGS = "-march=thumb -mcpu=arm9 -mattr=+soft-float,-v6"
LDFLAGS = "--gc-sections -T ./sdk/boot.link"

CCFLAGS = [
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
]

STARTUP_INPUTS = ("sdk/cstartup_8266.S", "sdk/version.in", "sdk/version.h")
VERSION_IN_REL = "sdk/version.in"
VERSION_RS_REL = "rust/src/version.rs"
LINKER_SCRIPT_REL = "sdk/boot.link"
LIBGCC_REL = "toolchain/tc32/lib/gcc/tc32-elf/4.5.1.tc32-elf-1.5/libgcc.a"
TC32_BIN_REL = "toolchain/tc32/bin"

DEFAULT_TOOLS = {
    "llc": "../llvm/build/bin/llc",
    "llvm_link": "../llvm/build/bin/llvm-link",
    "opt": "../llvm/build/bin/opt",
}
TOOL_ENV_KEYS = {"llc": "LLC", "llvm_link": "LLVM_LINK", "opt": "OPT"}
TOOL_FLAGS = {"llc": "--llc", "llvm_link": "--llvm-link", "opt": "--opt"}
TOOL_ORDER = ("llc", "llvm_link", "opt", "as", "ld", "objcopy")
LLVM_TOOLS = ("llc", "llvm_link", "opt")
TC32_TOOLS = ("as", "ld", "objcopy")

VERSION_IN_RE = re.compile(r"^\s*\.equ\s+BUILD_VERSION\s*,\s*(\d+)\s*$", re.MULTILINE)
VERSION_RS_RE = re.compile(r"pub\s+static\s+BUILD_VERSION\s*:\s*u32\s*=\s*(\d+)\s*;")
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


def resolve_tool(
    cli_value: str | None,
    env_key: str,
    dotenv: dict[str, str],
    default_rel: str,
    repo_root: Path,
) -> Path:
    """Resolve a tool path: CLI flag > environment > ``.env`` > repo default."""
    for candidate in (cli_value, os.environ.get(env_key), dotenv.get(env_key)):
        value = (candidate or "").strip()
        if value:
            return _anchor(value, repo_root)
    return _anchor(default_rel, repo_root)


def _require_tool(path: Path, flag: str, env_key: str) -> Path:
    """Fail loudly when a resolved tool is absent or not executable."""
    tool = Path(path)
    if not tool.is_file() or not os.access(tool, os.X_OK):
        raise BuildError(
            f"{flag}: tool is missing or not executable: {tool} "
            f"(cwd: {Path.cwd()}); pass {flag} <path> or set {env_key}",
            exit_code=2,
        )
    return tool


def _tc32_tool(repo_root: Path, name: str) -> Path:
    return Path(repo_root) / TC32_BIN_REL / f"tc32-elf-{name}"


# ---------------------------------------------------------------------------
# Version sync (read-only)
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


def read_version_in(path: Path) -> int:
    """Read ``.equ BUILD_VERSION,<N>`` from ``sdk/version.in``."""
    return _read_version(Path(path), VERSION_IN_RE, ".equ BUILD_VERSION,<N>")


def read_version_rs(path: Path) -> int:
    """Read ``pub static BUILD_VERSION: u32 = <N>;`` from ``rust/src/version.rs``."""
    return _read_version(
        Path(path), VERSION_RS_RE, "pub static BUILD_VERSION: u32 = <N>;"
    )


def check_version_sync(repo_root: Path) -> int:
    """Return ``BUILD_VERSION`` after proving both tracked files agree.

    Raises :class:`BuildError` when either file is missing or unparsable, or
    when the two integers differ. The driver never rewrites either file.
    """
    root = Path(repo_root)
    version_in = root / VERSION_IN_REL
    version_rs = root / VERSION_RS_REL
    from_in = read_version_in(version_in)
    from_rs = read_version_rs(version_rs)
    if from_in != from_rs:
        raise BuildError(
            f"BUILD_VERSION mismatch: {version_in} has {from_in} but "
            f"{version_rs} has {from_rs}; run `make bump-version` "
            "(sdk/increment_version.sh) to sync them",
            exit_code=2,
        )
    return from_in


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


def _startup_manifest(inputs: Sequence[Path]) -> dict:
    """The startup-object cache payload.

    Written and re-read through this one shape, so the writer and the
    ``startup_needs_rebuild`` predicate can never disagree about the schema.
    """
    return {"inputs": _input_entries(inputs)}


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


def _flags_block() -> dict[str, object]:
    """The flag strings that participate in cache invalidation."""
    return {
        "opt_passes": OPT_PASSES,
        "opt_inline_threshold": OPT_INLINE_THRESHOLD,
        "llc_flags": LLC_FLAGS,
        "ldflags": LDFLAGS,
    }


def compute_manifest(
    features: Sequence[str],
    ll_paths: Sequence[Path],
    startup_obj: Path,
    linker_script: Path,
    libgcc: Path,
    tools: dict[str, Path],
    toolchain: dict[str, object] | None = None,
) -> dict:
    """Build the deterministic manifest that guards the LLVM/TC32 pipeline."""
    missing = [name for name in TOOL_ORDER if name not in tools]
    if missing:
        raise BuildError(f"manifest is missing tool entries: {missing}", exit_code=2)

    ll_inputs = _input_entries(ll_paths)
    ll_inputs.sort(key=lambda entry: entry["path"])
    return {
        "schema_version": SCHEMA_VERSION,
        "features": list(features),
        "ll_inputs": ll_inputs,
        "startup_obj_sha256": _sha256(startup_obj),
        "linker_script_sha256": _sha256(linker_script),
        "libgcc_sha256": _sha256(libgcc),
        "tools": {name: _tool_entry(tools[name]) for name in TOOL_ORDER},
        "toolchain": dict(toolchain) if toolchain is not None else {},
        "flags": _flags_block(),
    }


def manifests_equal(first: dict | None, second: dict | None) -> bool:
    """Cache-hit predicate: parsed-JSON equality, never a partial/absent match."""
    return isinstance(first, dict) and isinstance(second, dict) and first == second


def startup_needs_rebuild(
    obj_path: Path,
    manifest_path: Path,
    inputs: Sequence[Path],
) -> bool:
    """True when the startup object is missing or any input hash changed."""
    obj = Path(obj_path)
    if not obj.is_file():
        return True
    recorded = _read_json(manifest_path)
    if recorded is None:
        return True
    return not manifests_equal(recorded, _startup_manifest(inputs))


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
        stderr=None,
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


def _startup_command(repo_root: Path, obj_path: Path) -> list[str]:
    """The pre-driver startup compile, unchanged in flags and order."""
    return [
        str(_tc32_tool(repo_root, "gcc")),
        "-B",
        str(Path(repo_root) / TC32_BIN_REL / "tc32-elf-"),
        "-c",
        *CCFLAGS,
        str(Path(repo_root) / STARTUP_INPUTS[0]),
        "-o",
        str(Path(obj_path)),
    ]


def _ensure_startup_object(out_root: Path, repo_root: Path) -> Path:
    asm_dir = Path(out_root) / "asm"
    obj = asm_dir / "cstartup_8266.o"
    manifest = asm_dir / "cstartup_8266.manifest.json"
    inputs = [Path(repo_root) / name for name in STARTUP_INPUTS]
    if startup_needs_rebuild(obj, manifest, inputs):
        asm_dir.mkdir(parents=True, exist_ok=True)
        _run_step(
            _startup_command(repo_root, obj),
            cwd=repo_root,
            what="startup compile",
        )
        _write_json(manifest, _startup_manifest(inputs))
    return obj


def _cargo_build(manifest_path: Path, features: Sequence[str]) -> list[Path]:
    """Run Cargo from the manifest's directory and resolve the exact IR inputs."""
    cmd = [
        "cargo",
        "build",
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
    env["RUSTFLAGS"] = "--emit=llvm-ir"
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
        "asm": str(stage / "merged.s"),
        "obj": str(stage / "merged.o"),
        "elf": str(stage / TARGET),
        "bin": str(stage / f"{TARGET}.bin"),
    }


def _pipeline_commands(
    stage_dir: Path,
    ll_paths: Sequence[Path],
    startup_obj: Path,
    libgcc: Path,
    tools: dict[str, Path],
) -> list[tuple[str, list[str]]]:
    """The six LLVM/TC32 commands, with the pre-driver flags and object order."""
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
                paths["opt_bc"],
                "-o",
                paths["asm"],
            ],
        ),
        ("as", [str(tools["as"]), "-o", paths["obj"], paths["asm"]]),
        (
            "ld",
            [
                str(tools["ld"]),
                *LDFLAGS.split(),
                "-o",
                paths["elf"],
                paths["obj"],
                str(startup_obj),
                str(libgcc),
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


def _resolve_llvm_tools(
    args: argparse.Namespace, dotenv: dict[str, str], repo_root: Path
) -> dict[str, Path]:
    resolved: dict[str, Path] = {}
    for name in LLVM_TOOLS:
        path = resolve_tool(
            getattr(args, name),
            TOOL_ENV_KEYS[name],
            dotenv,
            DEFAULT_TOOLS[name],
            repo_root,
        )
        resolved[name] = _require_tool(path, TOOL_FLAGS[name], TOOL_ENV_KEYS[name])
    return resolved


def _fixed_tools(repo_root: Path) -> dict[str, Path]:
    """Resolve and validate the vendored ``tc32-elf-{as,ld,objcopy}``."""
    tools: dict[str, Path] = {}
    for name in TC32_TOOLS:
        path = _tc32_tool(repo_root, name)
        tools[name] = _require_tool(path, f"tc32-elf-{name}", "PATH")
    return tools


def _run_build(args: argparse.Namespace) -> int:
    repo_root = REPO_ROOT
    out_root = _anchor(args.out_dir, repo_root)
    manifest_path = _anchor(args.manifest_path, repo_root)
    features = parse_features(args.features)
    slug = feature_slug(features)

    check_version_sync(repo_root)

    dotenv = {
        key: value
        for key, value in parse_dotenv(repo_root / ".env").items()
        if key not in os.environ
    }
    tools = _resolve_llvm_tools(args, dotenv, repo_root)
    tools.update(_fixed_tools(repo_root))

    toolchain = probe_toolchain(manifest_path.parent, tools["llc"])
    commit_short = str(toolchain["rustc_commit"])[:9]
    print(
        f"Toolchain: rustc {toolchain['rustc_release']} "
        f"({commit_short}, LLVM {toolchain['rustc_llvm']}) -> "
        f"llc LLVM {toolchain['fork_llvm']}"
    )

    startup_obj = _ensure_startup_object(out_root, repo_root)
    ll_paths = _cargo_build(manifest_path, features)

    linker_script = repo_root / LINKER_SCRIPT_REL
    libgcc = repo_root / LIBGCC_REL
    manifest = compute_manifest(
        features,
        ll_paths,
        startup_obj,
        linker_script,
        libgcc,
        tools,
        toolchain=toolchain,
    )

    stage_dir = out_root / slug
    stage = _stage_paths(stage_dir)
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
        for label, cmd in _pipeline_commands(
            stage_dir, ll_paths, startup_obj, libgcc, tools
        ):
            _run_step(cmd, cwd=repo_root, what=label)
        _write_json(stage_dir / "manifest.json", manifest)

    sync_outputs(target, out_root / TARGET)
    sync_outputs(target_bin, out_root / f"{TARGET}.bin")
    return 0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="build_firmware.py",
        description=(
            "Deterministic, incremental TC32/LLVM IR build driver for the "
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
        help="llc binary (default: $LLC, .env LLC, then ../llvm/build/bin/llc).",
    )
    parser.add_argument(
        "--llvm-link",
        default="",
        metavar="PATH",
        help="llvm-link binary (default: $LLVM_LINK, .env LLVM_LINK, then "
        "../llvm/build/bin/llvm-link).",
    )
    parser.add_argument(
        "--opt",
        default="",
        metavar="PATH",
        help="opt binary (default: $OPT, .env OPT, then ../llvm/build/bin/opt).",
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
