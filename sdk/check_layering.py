#!/usr/bin/env python3
"""Mechanical gates for the firmware workspace: Cargo DAG, subsystem isolation
and the per-crate ``unsafe`` ratchet.

``python3 sdk/check_layering.py`` — the no-argument form CI runs — executes
three independent passes over the shipping tree.

**Pass 1 — Cargo DAG.**  ``cargo metadata --manifest-path rust/Cargo.toml
--no-deps --format-version 1`` is validated against ``CARGO_DAG``: the five
workspace members, the workspace edges each crate may have, the external crates
each may depend on, the global ``mry`` ban (normal, dev *and* build), and
acyclicity.  The validator, ``audit_cargo_dag``, is a pure function over the
parsed JSON so the tests feed synthetic metadata and never invoke cargo;
``load_cargo_metadata`` is the only part that runs the tool.  A missing cargo,
a non-zero exit or unparseable JSON is **exit 2**: the gate fails closed rather
than reporting a DAG it never checked.

**Pass 2 — universal rules** over the composition root (``rust/src``) and every
``rust/crates/*/src`` tree.  One permanent rule per line of defence won by R-9
(#45):

* ``state-removed`` — any ``crate::state`` reference is a violation.  This rule
  scans the *whole raw file*, deliberately without the ``#[cfg(test)]`` mask:
  test modules import the deleted module too, and the acceptance criterion is
  literally ``rg 'crate::state' rust/src/`` returning nothing.  Whitespace
  around ``::`` is irrelevant to the Rust lexer and is matched.
* ``static-mut`` — a production ``static mut`` *item*.  The pattern is matched
  anywhere in production code, so an inline attribute or a one-line
  ``macro_rules!`` body does not hide it, while references such as
  ``fn run(&'static mut self, ...)`` and ``-> &'static mut [T; N]`` are
  rejected by the lookbehind (the preceding ``'`` is a lifetime).
* ``undocumented-unsafe`` — every production ``unsafe`` token must have
  ``SAFETY:`` in the contiguous ``//`` comment run directly above it.  The
  token is selected in ``production_code()`` (so test modules are excluded),
  but ``production_code()`` *blanks comments* — the raw ``text`` is therefore
  what the comment run is read from, while the ``#[cfg(test)]``-stripped mask
  proves each comment line is production code and not a test-only comment
  laundering a production site.

Path-level rules on every audited root (the first two used to guard only the
deleted ``rust/src/sdk/`` subtree):

* ``crate-alias`` — ``use crate as k;``, ``use crate::{self as k};`` and
  ``extern crate self as k;`` are violations.  Renaming the crate root defeats
  every ``crate::`` rule that follows, so the alias is banned outright.
* ``super-escape`` — a ``super::`` chain may not climb past the crate root.
  ``_module_segments`` is the single place that maps a file to its
  crate-relative module path, and a chain of *n* ``super`` hops from a file
  whose path has *n* segments lands exactly on the crate root:
  ``rust/src/light/mod.rs`` is ``light``, ``rust/src/ota/fsm.rs`` is
  ``ota::fsm``, ``rust/src/app.rs`` is ``app`` and ``lib.rs``/``main.rs`` is
  the empty path.  A chain one hop longer leaves the audited tree.  Module
  depth, not directory depth, is what the compiler resolves, so the rule
  matches Rust.
* ``app-pac`` — ``rust/src/app.rs`` and ``rust/src/irq.rs`` (the composition
  root and its ISR layer) may not *reference* ``tlsr8266_pac`` at all, in any
  production expression, not merely in a ``use``.  Registers arrive through the
  HAL, so a raw PAC path in those two files is a layering inversion.

**Pass 2 also carries the subsystem isolation table.**  ``SUBSYSTEM_PURITY``
maps each vertical subsystem of ``rust/src`` to the ``crate::`` modules and
external crate roots its production code may never name.  A row lists the
siblings and layers *above* its subsystem, never its own name, so the DAG is
enforced from both ends.  Each file under ``rust/src/<name>/`` (``logging`` is a
single module file, declared in ``SUBSYSTEM_ROOT_OVERRIDES``) gets that row's
``<name>-upward-dep`` and ``<name>-forbidden-crate`` rules.  A subsystem that is
*not* listed keeps the edge legal — ``light``, ``uart`` and ``mesh`` may all
name ``crate::logging`` because ``logging`` is absent from their rows, and
``mesh`` is the only row allowed ``tlsr8266_radio``.

``logging-macro`` bans ``uprintln!`` / ``uprint!`` in the leaf subsystems
``diagnostics``, ``factory_reset`` and ``ota``; the transport (``uart``), the
lamp, the node table and the composition root may log.  Like every isolation
rule it reads ``production_code()``, so ``#[cfg(test)]`` code stays exempt.

Every row rule resolves real module paths, so a spelling cannot dodge it.  A
``super::`` chain that lands on the crate root is the same inversion as
``crate::``: ``use super::uart::send;`` in ``light/mod.rs`` is
``crate::uart`` and is reported as ``light-upward-dep: crate::uart``, not
silently accepted.  So is the expression path ``super::uart::send()`` — the
resolution reads every ``super::`` chain in production code, not only the ones
inside a ``use``, and both spellings name the same resolved module (so a
repeat collapses instead of double-reporting).  ``use crate::*`` — and a
``use super::*`` whose hops reach the crate root — is ``<name>-glob-import`` in
every row, because the glob hands the whole crate root to the subsystem.  A
glob of a deeper module (``use crate::light::*`` inside ``light/``) stays
legal; it reaches no layer above the subsystem.  Forbidden crate roots are
matched as *references* over production code, exactly like ``app-pac``, so a
qualified ``tlsr8266_pac::write_reg8(..)`` call and ``use ::tlsr8266_pac::…``
are caught too, not only a canonical ``use``.

**Pass 3 — per-crate ``unsafe`` ratchet.**  ``MAX_PRODUCTION_UNSAFE_BY_CRATE``
holds one ceiling per workspace member, and the budget is enforced for **all
five** default roots, not just the firmware: a crate can no longer acquire
``unsafe`` freely.  The tool reports ``unsafe-budget`` only when a root
*exceeds* its ceiling, so removing ``unsafe`` never fails the tool; the
one-way half is a test that asserts ``measured == ceiling`` for every shipping
root, which is what forces the map down when sites disappear.  Code gated by
``#[cfg(not(target_os = "none"))]`` never reaches the firmware image, so it is
excluded from the count — but it is *not* exempt from ``static-mut`` or
``undocumented-unsafe``.  ``ROOT_CRATE_NAMES`` maps the firmware source root to
its Cargo package name (``rust/src`` is ``tlsr8266_mesh``); crate roots use
their manifest directory name.

**Known limitation.**  The gate is textual.  It sees the tokens and paths the
source *writes*, so a ``macro_rules!`` or proc-macro whose expansion names a
forbidden path is invisible to it: ``mry``-style derives, ``bitflags!`` bodies
and ``paste!``-generated identifiers are checked at the invocation site only.
No shipped macro expands to a forbidden edge today (the DAG pass and the
compiler cover the rest), but a new proc-macro dependency must be reviewed by
hand.  The scan is blind to re-export indirection for the same reason: a future
``pub use tlsr8266_pac as pac;`` in a permitted crate would hand a subsystem a
PAC path this gate does not know, so re-exporting a banned crate is a review
item, not a checked rule.

The ``super`` model is per *file*, not per lexical ``mod``: a ``super`` chain
inside an inline ``mod`` block is resolved against the file's own module path,
which is where the compiler starts only for a chain at the file's top level.
So ``mod inner { super::uart::x(); }`` in ``rust/src/light/mod.rs`` is read as
``crate::uart`` although the compiler resolves it to ``light::uart``.  Resolving
it properly needs a real module-tree walk (nested ``mod`` names, ``#[path]``
attributes), which is more machinery than this tree justifies: every inline
``mod`` in shipped subsystem code is ``#[cfg(test)]`` and stripped before the
scan, so the gap is latent, and it fails closed — a mislabelled or overshooting
chain is still reported, never silently accepted.
``test_inline_mod_super_chain_is_judged_by_the_file_scope`` pins that verdict.

Usage::

    python3 sdk/check_layering.py [--src-root PATH] [--crate-root PATH ...]

``--crate-root`` is repeatable.  When omitted, every workspace crate root is
audited; when given, exactly the named paths are.  Any explicit
``--src-root``/``--crate-root`` marks the run as hermetic: the Cargo DAG pass
and the per-crate ratchet are skipped, because a fixture tree has no
``Cargo.toml`` and must not be charged to a shipping ceiling.  A
``--crate-root`` without ``--src-root`` also drops the firmware subsystem pass,
because ``--crate-root`` then names exactly the trees to audit.  A hermetic run
names every pass it skipped on one stdout line
(``hermetic mode: ... skipped``), so a copy-pasted invocation cannot be
mistaken for the full gate.  The no-argument form audits the DAG, the
composition root and all four library crates.

Exit codes: ``0`` clean, ``1`` one or more violations in any pass, ``2``
usage/IO error, a root that was explicitly named but cannot be used, or a Cargo
DAG pass that could not be produced.  Violations are printed one per line as
``file:line: rule: violation``; a rule-level violation (a manifest dependency
edge or a per-tree budget) belongs to no source line and renders as
``path: rule: violation``.

The test stripper removes only ``#[cfg(test)]`` items and ``#[cfg(test)] mod
tests { ... }`` blocks, using brace balancing, so production code sitting
*below* an inline ``#[cfg(test)]`` item is still audited.  An attribute that is
not on an item (a struct field, a match arm, a statement) removes only its own
``;``/``,``-terminated unit, so the production item that follows it stays
visible.  Only strict ``#[cfg(test)]`` gates are stripped;
``#[cfg(not(test))]`` and ``#[cfg_attr(test, ...)]`` are production code.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from typing import Iterator, NamedTuple

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
DEFAULT_SRC_ROOT = os.path.join(REPO_ROOT, "rust", "src")
DEFAULT_CRATE_ROOT = os.path.join(REPO_ROOT, "rust", "crates", "mesh-protocol", "src")
# Every workspace crate audited by the universal pass when `--crate-root` is
# omitted.  A new crate is added here in the same change that creates it.
DEFAULT_CRATE_ROOTS = (
    DEFAULT_CRATE_ROOT,
    os.path.join(REPO_ROOT, "rust", "crates", "tlsr8266-pac", "src"),
    os.path.join(REPO_ROOT, "rust", "crates", "tlsr8266-hal", "src"),
    # C-10 (#72): the radio layer, extracted from the `rust/src/sdk/` tree that
    # C-10 then deleted.
    os.path.join(REPO_ROOT, "rust", "crates", "tlsr8266-radio", "src"),
)

# The firmware Cargo package.  Its source root is `rust/src`, so the package
# name cannot be read off the directory: this map is the one place that knows
# `rust/src` belongs to `tlsr8266_mesh`.  Every other audited root is
# `rust/crates/<package>/src` and takes its name from the manifest directory.
FIRMWARE_CRATE = "tlsr8266_mesh"
ROOT_CRATE_NAMES = {os.path.normpath(DEFAULT_SRC_ROOT): FIRMWARE_CRATE}

# ---------------------------------------------------------------------------
# Cargo DAG: (allowed workspace deps, allowed external non-dev, allowed dev).
# ---------------------------------------------------------------------------
# `None` is "no ceiling", never "no check": `WORKSPACE_FORBIDDEN_EXTERNAL` still
# applies, and the firmware crate may never appear as a dependency of a library
# crate.  A dependency whose name is a workspace member is a workspace edge;
# everything else is external.  Build-dependencies count as non-dev.
#
# Why each row is what it is:
#
# * `mesh-protocol` is the pure wire crate: `bytemuck` only, no firmware edge.
# * `tlsr8266-pac` is the register/bit-macro leaf: `bitflags` + `paste`.
# * `tlsr8266-hal` builds on the PAC and the wire crate; its external deps
#   (`critical-section`, `embassy-*`) are the async/MMIO scaffolding it owns.
# * `tlsr8266-radio` (C-10) owns the BLE/RF layer and sits above the HAL: it may
#   name the PAC, the HAL and the wire crate, and the embassy/heapless/bytemuck
#   stack it already carried inside the firmware.
# * `tlsr8266_mesh` is the composition root: it may depend on every workspace
#   crate, and nothing may depend on it.
CARGO_DAG = {
    "mesh-protocol": (frozenset(), frozenset({"bytemuck"}), frozenset()),
    "tlsr8266-pac": (frozenset(), frozenset({"bitflags", "paste"}), frozenset()),
    "tlsr8266-hal": (frozenset({"tlsr8266-pac", "mesh-protocol"}), None, None),
    "tlsr8266-radio": (
        frozenset({"tlsr8266-pac", "tlsr8266-hal", "mesh-protocol"}),
        None,
        None,
    ),
    "tlsr8266_mesh": (
        frozenset({"tlsr8266-pac", "tlsr8266-hal", "tlsr8266-radio", "mesh-protocol"}),
        None,
        None,
    ),
}
# `mry` was the vendored mocking crate C-10 (#72) retired.  It is banned in
# every crate and every dependency kind, so it cannot come back by dev-dep.
WORKSPACE_FORBIDDEN_EXTERNAL = frozenset({"mry"})
# The manifest path used in DAG violation reports.
CARGO_MANIFEST_LABEL = os.path.join("rust", "Cargo.toml")

# ---------------------------------------------------------------------------
# Modules every vertical subsystem of `rust/src/` must not name.
# ---------------------------------------------------------------------------
# One row per subsystem directly under `rust/src/`: the `crate::` modules its
# production code may never name, and the external crate roots it may never
# import.  A row lists the siblings and layers *above* the subsystem, never its
# own name, so the DAG is enforced from both ends (`uart` forbids `mesh` and
# `mesh` forbids `uart`).  An edge a row does *not* list stays legal —
# `light`, `uart` and `mesh` are allowed `crate::logging` exactly because
# `logging` is absent from their rows.
#
# Every row bans `tlsr8266_pac`: peripheral registers arrive through
# `tlsr8266_hal`, never by poking the PAC from a subsystem.  `tlsr8266_radio`
# is banned everywhere except `mesh`, which owns the node table and drives the
# radio glue.  `app` and `irq` are the composition root and its ISR layer, so
# they are forbidden in every row — the subsystem docs' "it never names `app`"
# is a checked claim, not a comment.
SUBSYSTEM_PURITY = {
    # The leaf of the application layer: the panic sector, the boot breadcrumbs
    # and the RTT mirror.  The sink is what writes the panic body, so the
    # direction is `logging -> diagnostics::rtt`, never back.
    "diagnostics": (
        frozenset(
            {"logging", "factory_reset", "ota", "light", "uart", "mesh", "app", "irq"}
        ),
        frozenset({"tlsr8266_pac", "tlsr8266_radio"}),
    ),
    # `logging` sits above `diagnostics` and below `uart`.  The transport is
    # reached through the `LogSinks` fn-pointer seam `app.rs` registers, which
    # is what keeps this row possible.
    "logging": (
        frozenset({"factory_reset", "ota", "light", "uart", "mesh", "app", "irq"}),
        frozenset({"tlsr8266_pac", "tlsr8266_radio"}),
    ),
    # Boot-time reset detection and the configuration erase.  It returns
    # `ResetAction`/`KickoutAction` instead of calling the layer above, so the
    # application side of the reset lives in the caller.
    "factory_reset": (
        frozenset(
            {
                "diagnostics",
                "logging",
                "ota",
                "light",
                "uart",
                "mesh",
                "app",
                "irq",
            }
        ),
        frozenset({"tlsr8266_pac", "tlsr8266_radio"}),
    ),
    # The OTA subsystem owns all OTA state and logic and reaches downward only.
    "ota": (
        frozenset(
            {
                "diagnostics",
                "logging",
                "factory_reset",
                "light",
                "uart",
                "mesh",
                "app",
                "irq",
            }
        ),
        frozenset({"tlsr8266_pac", "tlsr8266_radio"}),
    ),
    # The lamp: PWM, persistence, LED indication and the light-domain wire
    # protocol.  It publishes its status through a hook `app.rs` registers, so
    # it never needs the node table.
    "light": (
        frozenset({"diagnostics", "factory_reset", "ota", "uart", "mesh", "app", "irq"}),
        frozenset({"tlsr8266_pac", "tlsr8266_radio"}),
    ),
    # The UART bridge: framing, ACK/retry and its two embassy tasks.  The mesh
    # relay and host command execution arrive through seams, so `mesh` and
    # `app` stay out.
    "uart": (
        frozenset({"diagnostics", "factory_reset", "ota", "light", "mesh", "app", "irq"}),
        frozenset({"tlsr8266_pac", "tlsr8266_radio"}),
    ),
    # The node table, pairing state machine and radio glue.  The lamp status,
    # light events and the UART relay arrive through `MeshEventHooks`, so only
    # this row may name `tlsr8266_radio`.
    "mesh": (
        frozenset({"diagnostics", "factory_reset", "ota", "light", "uart", "app", "irq"}),
        frozenset({"tlsr8266_pac"}),
    ),
}

# A subsystem is normally a directory of the same name, but `logging` is a
# single module file.  This override keeps one table: `subsystem_roots` resolves
# each row's root as `rust/src/<name>` unless the name appears here, and the
# purity match accepts an exact file path as well as a directory.
SUBSYSTEM_ROOT_OVERRIDES = {"logging": "logging.rs"}

LOGGING_MACROS = ("uprintln", "uprint")
# The leaf subsystems are not allowed to log: they sit below the transport, and
# `uart`, `light`, `mesh` and the composition root are where output is emitted.
LOGGING_MACRO_FORBIDDEN_SUBSYSTEMS = frozenset(
    {"diagnostics", "factory_reset", "ota"}
)

# The composition root and its ISR layer: registers arrive through the HAL, so
# these two files may not reference the PAC at all.
COMPOSITION_ROOT_FILES = frozenset({"app.rs", "irq.rs"})

# One-way ratchet, per crate: lower a ceiling when `unsafe` sites disappear,
# never raise it.  Every number was measured by running this gate on the tree
# that ships it, never by hand-counting:
#
#     python3 -c 'import sys; sys.path.insert(0, "sdk"); \
#       import check_layering as c; \
#       print(c.audit_universal(c.rust_files("rust/src"), "rust/src")[1])'
#
# The history is why the firmware ceiling is small: R-9 (#45) measured 132 sites
# in `rust/src`; R-10 (#46) moved 45 into `mesh-protocol`, C-2 (#64) 6 into
# `tlsr8266-pac`, C-4/C-5 (#66/#67) 17 into `tlsr8266-hal`, C-7 (#69) and
# C-8 (#70) removed 4 more, and C-10 (#72) moved the 44 RF/BLE sites into
# `tlsr8266-radio`: 132 - 45 - 6 - 17 - 4 - 44 = 16.
#
# `sdk/tests` asserts `measured == ceiling` for all five roots.  The tool only
# fails on an *excess*, so an improvement never blocks a developer; that
# equality assertion is what forces the map down with the code.
MAX_PRODUCTION_UNSAFE_BY_CRATE = {
    "tlsr8266_mesh": 16,
    "tlsr8266-pac": 6,
    "tlsr8266-hal": 16,
    "tlsr8266-radio": 44,
    "mesh-protocol": 45,
}

# Root-module file names: the only files for which "directly under the root"
# means "the audited root's own module".
_CRATE_ROOT_FILES = frozenset({"lib.rs", "main.rs"})

_CFG_TEST_RE = re.compile(r"#\s*\[\s*cfg\s*\(\s*test\s*\)\s*\]")
# `#[cfg(not(target_os = "none"))]` marks code that only builds for the host.
# `mask_source` blanks the `"none"` literal, so this pattern can never match the
# masked `code`; it is matched against the raw `text` instead (see
# `strip_host_only_code`).
_CFG_HOST_RE = re.compile(
    r'#\s*\[\s*cfg\s*\(\s*not\s*\(\s*target_os\s*=\s*"none"\s*\)\s*\)\s*\]'
)
_LOGGING_RE = re.compile(
    r"(?<![A-Za-z0-9_])(?:" + "|".join(LOGGING_MACROS) + r")\s*!"
)
_CRATE_REF_RE = re.compile(r"crate\s*::")
# A `use` statement can start a line, or follow a `{` (a block or `mod` body) or
# a `;` (mid-block).  Rustfmt puts a function-local `use` on its own line, but
# the rules must not depend on formatting: `pub fn f() { use crate::*; }` is the
# same crate-root glob as the top-level spelling.
_IMPORT_RE = re.compile(
    r"(?:^[ \t]*|(?<=[{;])[ \t\n]*)"
    r"(?:pub(?:[ \t]*\([^)\n]*\))?[ \t]+)?use[ \t\n]+",
    re.MULTILINE,
)
# `extern crate self as krate;` aliases the crate root exactly like
# `use crate as krate;`, but it is not a `use` statement, so `iter_imports`
# never visits it.  `extern crate <name>;` (core, std, a dependency) must not
# match: only the `self as` form is a crate-root rename.
_EXTERN_CRATE_SELF_RE = re.compile(
    r"(?<![A-Za-z0-9_])extern\s+crate\s+self\s+as\s+(?P<alias>[A-Za-z_][A-Za-z0-9_]*)"
)
# One or more `super::` hops, e.g. `super::super::app`.  `\s*` is allowed so a
# chain that wraps across lines is still counted once.
_SUPER_CHAIN_RE = re.compile(r"(?:\bsuper\s*::\s*)+")
_SUPER_KEYWORD_RE = re.compile(r"\bsuper\b")
# The register-definition leaf crate.  `app.rs`/`irq.rs` may not *reference* it:
# the rule is a scan of production code, not only of `use` statements.
_PAC_REF_RE = re.compile(r"\btlsr8266_pac\b")
# One reference matcher per [`SUBSYSTEM_PURITY`] row for the crate roots it
# bans.  Matched as references, not `use` statements, so a qualified call and a
# leading-`::` absolute path are caught exactly like the `app-pac` rule.  The
# row's crate names do not overlap, so `group(0)` names the crate that matched.
SUBSYSTEM_FORBIDDEN_CRATE_RE = {
    name: re.compile(
        r"\b(?:" + "|".join(re.escape(root) for root in sorted(crates)) + r")\b"
    )
    for name, (_modules, crates) in SUBSYSTEM_PURITY.items()
}

# --- universal pass (R-9) -------------------------------------------------
# `state-removed` runs on the raw file text: the test mask would hide the
# `#[cfg(test)]` import sites, and those are exactly what must also be gone.
# The Rust lexer ignores whitespace around `::`, so `use crate :: state::X;` is
# the same reference and `\s*` is required.  Crate-alias evasion
# (`use crate as c; c::state::X`) needs no rule of its own and none is built:
# the module no longer exists, so `c::state` is a hard compile error and can
# never reach the tree.
_STATE_REF_RE = re.compile(r"crate\s*::\s*state")
# Matched anywhere in production code, so an attribute sharing the line
# (`#[link_section = ".data"] pub static mut X`) and a single-line
# `macro_rules!` body are both caught, and `\s` lets `static` and `mut` (or
# `mut` and the name) sit on different lines — whitespace is not a token
# boundary the Rust lexer respects.  It must never match a *reference*: in
# `&'static mut self` and `-> &'static mut [T; N]` the character before
# `static` is the `'` of the lifetime, which the lookbehind rejects.
_STATIC_MUT_RE = re.compile(r"(?<![&'\w])static\s+mut\s+[A-Za-z_]\w*")
_UNSAFE_RE = re.compile(r"\bunsafe\b")


class GateError(RuntimeError):
    """A requested pass could not be run; ``main`` turns this into exit 2."""


class Violation(NamedTuple):
    path: str
    line: int
    rule: str
    violation: str

    def render(self) -> str:
        # A rule-level violation belongs to no source line: a manifest edge and
        # a per-tree `unsafe` budget carry `line = 0` because the path is a
        # manifest or a directory.  Printing `:0:` would send a reader looking
        # for a line that does not exist, so those render without a line.
        if self.line < 1:
            return f"{self.path}: {self.rule}: {self.violation}"
        return f"{self.path}:{self.line}: {self.rule}: {self.violation}"


# ---------------------------------------------------------------------------
# Cargo DAG: load metadata (the only subprocess) and validate it purely.
# ---------------------------------------------------------------------------


def load_cargo_metadata(repo_root: str) -> dict:
    """Run ``cargo metadata`` for the workspace at ``repo_root/rust``.

    ``cwd`` is the repository root and not ``rust/``, so the pinned toolchain in
    ``rust/rust-toolchain.toml`` is never resolved and a CI job without that
    channel installed does not have to download it.  ``--no-deps`` keeps the
    call offline and lists exactly the workspace members.

    Raises [`GateError`] if cargo cannot be run, exits non-zero or emits
    something that is not a JSON object: the DAG pass fails closed.
    """
    manifest = os.path.join(repo_root, "rust", "Cargo.toml")
    command = [
        "cargo",
        "metadata",
        "--manifest-path",
        manifest,
        "--no-deps",
        "--format-version",
        "1",
    ]
    try:
        completed = subprocess.run(
            command,
            cwd=repo_root,
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError as exc:
        raise GateError(f"cannot run cargo metadata: {exc}") from exc
    if completed.returncode != 0:
        details = (completed.stderr or completed.stdout).strip().splitlines()
        tail = details[-1] if details else "no output"
        raise GateError(
            f"cargo metadata failed (exit {completed.returncode}): {tail}"
        )
    try:
        metadata = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        raise GateError(f"cargo metadata emitted unparseable JSON: {exc}") from exc
    if not isinstance(metadata, dict):
        raise GateError("cargo metadata emitted a non-object JSON document")
    return metadata


def _dependency_kind(dependency: dict) -> str | None:
    """``"dev"``, ``"build"`` or ``None`` (a normal dependency)."""
    kind = dependency.get("kind")
    return kind if kind in ("dev", "build") else None


def audit_cargo_dag(metadata: dict) -> list[Violation]:
    """Validate ``cargo metadata --no-deps`` JSON against [`CARGO_DAG`].

    Pure: no cargo, no filesystem.  Every violation carries the rule name
    ``cargo-dag`` and the manifest as its path, because a dependency edge is a
    property of the manifest rather than of a source line.

    Rules: the workspace members are exactly the [`CARGO_DAG`] keys; a
    dependency whose name is a member must be on that crate's allowed workspace
    list (and the firmware crate may never be one of them); every other
    dependency must be on the external/dev allowlist, or is a violation when the
    row has a ceiling; ``mry`` is a violation in every crate and every kind; and
    the workspace edge graph is acyclic.
    """
    violations: list[Violation] = []
    reported: set[str] = set()

    def report(message: str) -> None:
        if message in reported:
            return
        reported.add(message)
        violations.append(
            Violation(CARGO_MANIFEST_LABEL, 0, "cargo-dag", message)
        )

    packages = {
        package.get("name"): package
        for package in metadata.get("packages", ())
        if isinstance(package, dict) and package.get("name")
    }
    members = set(packages)

    for name in sorted(members - set(CARGO_DAG)):
        report(f"workspace member `{name}` is not in CARGO_DAG")
    for name in sorted(set(CARGO_DAG) - members):
        report(f"workspace member `{name}` is missing from the workspace")

    edges: dict[str, set[str]] = {name: set() for name in members}
    for name in sorted(members & set(CARGO_DAG)):
        allowed_workspace, allowed_external, allowed_dev = CARGO_DAG[name]
        for dependency in packages[name].get("dependencies", ()):
            if not isinstance(dependency, dict):
                continue
            dependency_name = dependency.get("name")
            if not dependency_name:
                continue
            kind = _dependency_kind(dependency)
            if dependency_name in members:
                edges[name].add(dependency_name)
                if dependency_name == FIRMWARE_CRATE:
                    report(
                        f"`{name}` may not depend on the firmware crate "
                        f"`{FIRMWARE_CRATE}`"
                    )
                elif dependency_name not in allowed_workspace:
                    report(
                        f"`{name}` has a workspace edge to `{dependency_name}` "
                        "that CARGO_DAG does not allow"
                    )
                continue
            if dependency_name in WORKSPACE_FORBIDDEN_EXTERNAL:
                report(
                    f"`{name}` depends on `{dependency_name}` "
                    f"({kind or 'normal'}): banned in every crate"
                )
                continue
            ceiling = allowed_dev if kind == "dev" else allowed_external
            if ceiling is not None and dependency_name not in ceiling:
                bucket = "a dev-dependency" if kind == "dev" else "a dependency"
                report(f"`{name}` may not have `{dependency_name}` as {bucket}")

    finished: set[str] = set()
    visiting: list[str] = []

    def walk(node: str) -> None:
        if node in finished:
            return
        if node in visiting:
            cycle = visiting[visiting.index(node) :] + [node]
            report("workspace dependency cycle: " + " -> ".join(cycle))
            return
        visiting.append(node)
        for child in sorted(edges.get(node, ())):
            walk(child)
        visiting.pop()
        finished.add(node)

    for node in sorted(edges):
        walk(node)

    violations.sort(key=lambda item: item.violation)
    return violations


# ---------------------------------------------------------------------------
# Lexing: decide which characters are code (not string/char/comment).
# ---------------------------------------------------------------------------


def _is_ident_start(char: str) -> bool:
    return char.isalpha() or char == "_"


def _is_ident_continue(char: str) -> bool:
    return char.isalnum() or char == "_"


def _raw_string_start(text: str, index: int) -> tuple[int, int] | None:
    """Return ``(quote_index, hash_count)`` for a raw string starting at index."""
    if index > 0 and _is_ident_continue(text[index - 1]):
        return None
    if text[index] not in ("r", "b"):
        return None
    cursor = index
    if text[cursor] == "b":
        if text.startswith('b"', cursor):
            return None  # a plain byte string; handled by the quote branch
        if cursor + 1 >= len(text) or text[cursor + 1] != "r":
            return None
        cursor += 1
    if text.startswith('r"', cursor):
        return cursor + 1, 0
    cursor += 1
    hashes = 0
    while cursor < len(text) and text[cursor] == "#":
        hashes += 1
        cursor += 1
    if hashes and cursor < len(text) and text[cursor] == '"':
        return cursor, hashes
    return None


def mask_source(text: str) -> bytearray:
    """Return a per-character mask: 1 for code, 0 for comments/strings/chars.

    A ``{`` or ``}`` inside a comment, string literal, or char literal is not
    code and must not disturb brace balancing.  Rust raw strings are handled
    (``r"..."``, ``r#"..."#``, ``br#"..."#``).
    """
    length = len(text)
    mask = bytearray(b"\x01") * length
    index = 0
    while index < length:
        char = text[index]

        if char == "/" and index + 1 < length:
            nxt = text[index + 1]
            if nxt == "/":
                stop = text.find("\n", index)
                if stop < 0:
                    stop = length
                mask[index:stop] = b"\x00" * (stop - index)
                index = stop
                continue
            if nxt == "*":
                depth = 0
                cursor = index
                while cursor < length:
                    if text.startswith("/*", cursor):
                        depth += 1
                        mask[cursor : cursor + 2] = b"\x00\x00"
                        cursor += 2
                    elif text.startswith("*/", cursor):
                        depth -= 1
                        mask[cursor : cursor + 2] = b"\x00\x00"
                        cursor += 2
                        if depth == 0:
                            break
                    else:
                        mask[cursor] = 0
                        cursor += 1
                index = cursor
                continue

        raw = _raw_string_start(text, index)
        if raw is not None:
            quote, hashes = raw
            terminator = '"' + "#" * hashes
            stop = text.find(terminator, quote + 1)
            stop = length if stop < 0 else stop + len(terminator)
            mask[index:stop] = b"\x00" * (stop - index)
            index = stop
            continue

        if char == '"':
            cursor = index + 1
            while cursor < length:
                if text[cursor] == "\\":
                    cursor += 2
                    continue
                if text[cursor] == '"':
                    cursor += 1
                    break
                cursor += 1
            mask[index:cursor] = b"\x00" * (cursor - index)
            index = cursor
            continue

        if char == "'":
            # Char literal vs lifetime.  A lifetime is `'ident` not followed by
            # another quote; everything else is a char literal to blank.
            nxt = text[index + 1] if index + 1 < length else ""
            if nxt == "\\":
                cursor = index + 2
                while cursor < length and text[cursor] != "'":
                    cursor += 1
                cursor = min(cursor + 1, length)
                mask[index:cursor] = b"\x00" * (cursor - index)
                index = cursor
                continue
            if nxt and _is_ident_start(nxt):
                cursor = index + 1
                while cursor < length and _is_ident_continue(text[cursor]):
                    cursor += 1
                if cursor >= length or text[cursor] != "'":
                    index = cursor  # lifetime, keep as code
                    continue
            cursor = index + 1
            while cursor < length:
                if text[cursor] == "\\":
                    cursor += 2
                    continue
                if text[cursor] == "'":
                    cursor += 1
                    break
                cursor += 1
            mask[index:cursor] = b"\x00" * (cursor - index)
            index = cursor
            continue

        index += 1
    return mask


def blank_copy(text: str, mask: bytearray) -> str:
    """Render ``text`` with every non-code character replaced by a space.

    Newlines are preserved so regexes still see line structure.
    """
    return "".join(
        char if char == "\n" or mask[index] else " "
        for index, char in enumerate(text)
    )


def _matching(code: str, index: int, open_char: str, close_char: str) -> int:
    depth = 0
    for cursor in range(index, len(code)):
        char = code[cursor]
        if char == open_char:
            depth += 1
        elif char == close_char:
            depth -= 1
            if depth == 0:
                return cursor
    return -1


def _skip_space(code: str, index: int) -> int:
    while index < len(code) and code[index].isspace():
        index += 1
    return index


def _ident_at(code: str, index: int) -> tuple[str, int]:
    """The identifier at ``index`` (``""`` if none) and the index after it."""
    if index >= len(code) or not _is_ident_start(code[index]):
        return "", index
    cursor = index
    while cursor < len(code) and _is_ident_continue(code[cursor]):
        cursor += 1
    return code[index:cursor], cursor


def _skip_attributes(code: str, index: int) -> int | None:
    """Skip whitespace and a run of attributes; ``None`` if one is unbalanced."""
    length = len(code)
    while True:
        index = _skip_space(code, index)
        if index >= length or code[index] != "#":
            return index
        cursor = _skip_space(code, index + 1)
        if cursor < length and code[cursor] == "!":
            cursor = _skip_space(code, cursor + 1)
        if cursor >= length or code[cursor] != "[":
            return index
        close = _matching(code, cursor, "[", "]")
        if close < 0:
            return None
        index = close + 1


# ---------------------------------------------------------------------------
# Test stripping: remove only `#[cfg(test)]` items, never truncate the file.
# ---------------------------------------------------------------------------


# Tokens that may precede an item keyword without changing what is annotated.
# `pub` is handled separately because of `pub(crate)` / `pub(in path)`.
_ITEM_PREFIXES = frozenset({"unsafe", "async", "default"})
# The keywords that start an *item*.  Only these can be annotated by
# `#[cfg(test)]` and removed wholesale.
_ITEM_KEYWORDS = frozenset(
    {
        "fn",
        "mod",
        "struct",
        "enum",
        "union",
        "trait",
        "impl",
        "static",
        "const",
        "use",
        "type",
        "extern",
        "macro_rules",
    }
)


def _starts_item(code: str, index: int) -> bool:
    """Whether the token after the attributes at ``index`` starts an item.

    ``#[cfg(test)]`` on an item removes the whole item.  On a struct field, a
    match arm or an expression statement it annotates something *inside* an
    item, and its extent must stop at that unit instead — never at whatever
    brace block happens to come next.
    """
    length = len(code)
    cursor = _skip_attributes(code, index)
    if cursor is None:
        return False
    while cursor < length:
        cursor = _skip_space(code, cursor)
        name, cursor = _ident_at(code, cursor)
        if name == "pub":
            after = _skip_space(code, cursor)
            if after < length and code[after] == "(":
                close = _matching(code, after, "(", ")")
                if close < 0:
                    return False
                cursor = close + 1
            continue
        if name in _ITEM_PREFIXES:
            continue
        return name in _ITEM_KEYWORDS
    return False


def _annotation_extent(code: str, index: int) -> int | None:
    """End index (exclusive) of the unit a non-item ``#[cfg(test)]`` annotates.

    A struct field ends at its ``,`` and a statement at its ``;``.  Brackets
    and parentheses nest, so the ``;`` of ``[u8; 4]`` does not terminate, and a
    block-bodied statement is consumed as one unit.  The scan never runs past
    the enclosing block's ``}``: over-stripping is the failure mode this exists
    to prevent.
    """
    length = len(code)
    depth = 0
    cursor = index
    while cursor < length:
        char = code[cursor]
        if char in "([":
            depth += 1
        elif char in ")]":
            if depth == 0:
                return cursor
            depth -= 1
        elif char == "{":
            if depth == 0:
                close = _matching(code, cursor, "{", "}")
                if close < 0:
                    return None
                after = _skip_space(code, close + 1)
                if after < length and code[after] in ";,":
                    return after + 1
                return close + 1
            depth += 1
        elif char == "}":
            if depth == 0:
                return cursor
            depth -= 1
        elif depth == 0 and char in ";,":
            return cursor + 1
        cursor += 1
    return None


def _item_extent(code: str, index: int) -> int | None:
    """End index (exclusive) of the item starting at ``index``.

    Attributes and whitespace are skipped, then the first top-level ``{`` is
    balanced to its ``}``; an item without a body ends at its top-level ``;``.
    """
    length = len(code)
    index = _skip_attributes(code, index)
    if index is None:
        return None
    if index >= length:
        return None

    depth = 0
    cursor = index
    while cursor < length:
        char = code[cursor]
        if char in "([":
            depth += 1
        elif char in ")]":
            if depth:
                depth -= 1
        elif depth == 0 and char == "{":
            close = _matching(code, cursor, "{", "}")
            return None if close < 0 else close + 1
        elif depth == 0 and char == ";":
            return cursor + 1
        cursor += 1
    return None


def strip_test_code(text: str, mask: bytearray) -> bytearray:
    """Zero the mask over every ``#[cfg(test)]`` item / module.

    The extent depends on what the attribute annotates: an item is removed
    whole, while a field or statement (``#[cfg(test)]`` *inside* an item) is
    removed only up to its own ``,``/``;``.
    """
    code = blank_copy(text, mask)
    stripped = bytearray(mask)
    for match in _CFG_TEST_RE.finditer(code):
        if _starts_item(code, match.end()):
            end = _item_extent(code, match.end())
        else:
            end = _annotation_extent(code, match.end())
        if end is not None:
            stripped[match.start() : end] = b"\x00" * (end - match.start())
    return stripped


def production_code(text: str) -> str:
    """The file with test-only code blanked out, keeping line numbers intact."""
    return blank_copy(text, strip_test_code(text, mask_source(text)))


def strip_host_only_code(text: str, stripped: bytearray) -> bytearray:
    """Zero the mask over every ``#[cfg(not(target_os = "none"))]`` span.

    Host-only code never reaches the firmware image, so it is excluded from the
    ``unsafe-budget`` count.  It is *not* exempt from ``static-mut`` or
    ``undocumented-unsafe``: those rules keep running on ``stripped``, so the
    returned mask is used for the numeric budget only.

    The attribute is matched against the raw ``text``: ``mask_source`` blanks
    the ``"none"`` string literal, so a regex over the masked ``code`` can never
    match it.  The code/comment mask then rejects the same characters sitting in
    a comment or a string literal.  Extents are computed on the test-stripped
    ``code`` — offsets are identical, ``blank_copy`` keeps the length — so an
    item is removed whole while a field, statement or bare block annotation
    stops at its own unit.
    """
    code = blank_copy(text, stripped)
    mask = mask_source(text)
    target_stripped = bytearray(stripped)
    for match in _CFG_HOST_RE.finditer(text):
        if mask[match.start()] != 1:
            continue  # the attribute text is inside a comment or a string
        if _starts_item(code, match.end()):
            end = _item_extent(code, match.end())
        else:
            end = _annotation_extent(code, match.end())
        if end is not None:
            target_stripped[match.start() : end] = b"\x00" * (end - match.start())
    return target_stripped


# ---------------------------------------------------------------------------
# Path extraction.
# ---------------------------------------------------------------------------


def _parse_use_tree(
    code: str, index: int, prefix: tuple[str, ...], first: int | None
) -> Iterator[tuple[int, tuple[str, ...]]]:
    """Yield ``(first_segment_index, leading_segments)`` for a use-tree path."""
    length = len(code)
    index = _skip_space(code, index)
    if index < length and code[index] == "{":
        close = _matching(code, index, "{", "}")
        if close < 0:
            return
        depth = 0
        part = index + 1
        cursor = index + 1
        while cursor < close:
            char = code[cursor]
            if char in "{[(":
                depth += 1
            elif char in "}])":
                depth -= 1
            elif char == "," and depth == 0:
                # Inherit the path's first-segment index so a multi-line use
                # tree is reported on the line where the path starts.
                yield from _parse_use_tree(code, part, prefix, first)
                part = cursor + 1
            cursor += 1
        yield from _parse_use_tree(code, part, prefix, first)
        return
    if index >= length or not _is_ident_start(code[index]):
        if index < length and code[index] == "*":
            # A glob is the last stop of a path.  Recording it as a `*` segment
            # is what lets a rule tell `use crate::*` (the whole crate root)
            # from `use crate::light::*` (one deeper module).
            yield (index if first is None else first), prefix + ("*",)
        return
    cursor = index
    while cursor < length and _is_ident_continue(code[cursor]):
        cursor += 1
    head = prefix + (code[index:cursor],)
    if first is None:
        first = index
    after = _skip_space(code, cursor)
    if code[after : after + 2] == "::":
        yield from _parse_use_tree(code, after + 2, head, first)
    else:
        yield first, head


def iter_crate_references(code: str) -> Iterator[tuple[int, tuple[str, ...]]]:
    """Every ``crate::`` path in production code, as leading segments."""
    for match in _CRATE_REF_RE.finditer(code):
        yield from _parse_use_tree(code, match.end(), (), None)


def iter_imports(code: str) -> Iterator[tuple[int, tuple[str, ...]]]:
    """Every ``use`` statement's path, as leading segments.

    A statement is found wherever one can begin — a line start, after ``{`` or
    after ``;`` — so a function-local ``use`` is read exactly like its
    top-level spelling.
    """
    for match in _IMPORT_RE.finditer(code):
        yield from _parse_use_tree(code, match.end(), (), None)


def use_statement_spans(code: str) -> list[tuple[int, int]]:
    """``(start, end)`` of every ``use`` statement in ``code``.

    A ``super::`` chain inside a use tree is resolved by [`iter_imports`],
    which composes the brace groups and reports the whole path once.  A caller
    that also reads ``super::`` chains as *expression* paths uses these spans
    to stay out of the use tree's way.  An extent that cannot be measured
    stops at the ``use`` keyword, so a malformed statement can repeat a
    finding but never hide one.
    """
    spans: list[tuple[int, int]] = []
    for match in _IMPORT_RE.finditer(code):
        end = _item_extent(code, match.end())
        spans.append((match.start(), match.end() if end is None else end))
    return spans


# ---------------------------------------------------------------------------
# Rules.
# ---------------------------------------------------------------------------


def _line_of(text: str, index: int) -> int:
    return text.count("\n", 0, index) + 1


def _super_chain_hops(code: str, match: re.Match) -> int:
    """Hops in one ``super::`` chain, including a following brace group.

    ``use super::{super::app}`` resolves like ``super::super::app``: a brace
    group composes with the prefix, so its deepest chain adds to the outer one.
    Sibling chains in one group do not stack (the deepest wins), so
    ``super::{super::a, super::b}`` counts two hops, not three.
    """
    hops = len(_SUPER_KEYWORD_RE.findall(match.group(0)))
    after = _skip_space(code, match.end())
    if code[after : after + 1] == "{":
        close = _matching(code, after, "{", "}")
        if close > 0:
            inner = code[after + 1 : close]
            hops += max(
                (_super_chain_hops(inner, nested) for nested in _SUPER_CHAIN_RE.finditer(inner)),
                default=0,
            )
    return hops


class _Collector:
    def __init__(self, path: str, text: str) -> None:
        self.path = path
        self.text = text
        self.found: list[Violation] = []
        self._seen: set[tuple[int, str, str]] = set()

    def add(self, index: int, rule: str, violation: str) -> None:
        line = _line_of(self.text, index)
        key = (line, rule, violation)
        if key in self._seen:
            return
        self._seen.add(key)
        self.found.append(Violation(self.path, line, rule, violation))


def _statement_of(text: str, index: int) -> str:
    """The ``use`` statement starting at ``index``, on one line."""
    stop = text.find("\n", index)
    stop = len(text) if stop < 0 else stop
    return text[index:stop].strip().rstrip(";")


# ---------------------------------------------------------------------------
# Subsystem purity: a `rust/src/<subsystem>/**` file must not name the layers
# above it.  The table lives in `SUBSYSTEM_PURITY`.
# ---------------------------------------------------------------------------


def _root_module_of(head: tuple[str, ...], module: tuple[str, ...]) -> str | None:
    """The crate-root segment a path lands on, if it lands there exactly.

    ``head`` is a path's leading segments — an import path's, or a bare
    ``super::`` chain rebuilt as ``("super", ...)`` plus its next segment —
    and ``module`` is the file's own module path (empty for the crate root
    itself).  ``crate::uart::send`` starts at the root, so the root segment is
    ``uart``; ``super::uart::send`` in ``light/mod.rs`` pops the one module the
    file sits below and resolves to the same ``uart``.  A chain that climbs
    past the root returns ``None`` — ``super-escape`` already reports it, and
    double-reporting one path helps nobody — and so does an internal path
    (``super::fsm`` from ``ota/mod.rs``, ``self::x``): it never names a
    crate-root module.  A glob is the segment ``*``, which is how a caller
    tells a crate-root glob from a deeper one.
    """
    hops = 0
    while hops < len(head) and head[hops] == "super":
        hops += 1
    if hops == 0:
        return head[1] if head[:1] == ("crate",) and len(head) > 1 else None
    if hops != len(module) or len(head) <= hops:
        return None
    return head[hops]


def audit_subsystem_purity_source(
    text: str,
    path: str,
    directory: str,
    module: tuple[str, ...],
    code: str | None = None,
) -> list[Violation]:
    """Audit one subsystem file for upward and forbidden names.

    ``directory`` is the key of its row in [`SUBSYSTEM_PURITY`]; the reported
    rules are ``<directory>-upward-dep``, ``<directory>-forbidden-crate`` and
    ``<directory>-glob-import``.  ``module`` is the file's crate-relative module
    path (see [`_module_segments`]), so a ``super::`` chain that lands on the
    crate root names a real root module and is judged as one.

    Runs on ``production_code()``, so the subsystem's own ``#[cfg(test)]``
    modules are exempt: they legitimately use
    ``tlsr8266_pac::fake_mmio::FakeMmio`` to stand in for the flash/radio
    register block, and a raw text grep would fail a correct tree.  Production
    code is what the "zero imports" criterion is about, and that is what this
    reads — the same stripping that makes the ratchet a production count.

    ``crate::`` references are read from the whole production text (so a
    qualified ``crate::light::..`` path is caught wherever it appears), and so
    are the forbidden external crate roots: a qualified
    ``tlsr8266_pac::write_reg8(..)`` call and a leading-``::`` absolute path are
    the same inversion as a canonical ``use``.  ``use`` statements supply the
    relative spellings — ``super::uart`` in ``light/mod.rs`` is
    ``crate::uart`` — and the crate-root globs, which `_root_module_of` marks
    with a ``*`` segment; a bare ``super::`` chain outside a ``use`` (an
    expression path such as ``super::uart::send()``) goes through the same
    resolution.  ``code`` is the already-masked ``production_code(text)`` when
    the universal pass has it; passing it keeps a subsystem file masked once
    rather than twice.
    """
    forbidden_modules = SUBSYSTEM_PURITY[directory][0]
    if code is None:
        code = production_code(text)
    collector = _Collector(path, text)

    def report_root(index: int, name: str | None) -> None:
        """Report one path that resolves to a crate-root segment the row bans.

        All three scans below agree on what a resolved root segment means, so
        the rule name and the violation text are written once.
        """
        if name in forbidden_modules:
            collector.add(index, f"{directory}-upward-dep", f"crate::{name}")

    for index, head in iter_crate_references(code):
        report_root(index, head[0])
    for index, head in iter_imports(code):
        # Only a use tree can spell the crate-root glob (`crate::light::*` in a
        # row is an ordinary module path), so the glob rule lives here and the
        # scans below treat `*` as an ordinary segment that no row bans.
        root = _root_module_of(head, module)
        if root == "*":
            collector.add(index, f"{directory}-glob-import", "crate::*")
        else:
            report_root(index, root)
    # `super::` roots an *expression* path too: `super::uart::send()` is the
    # same `crate::uart` edge as `use super::uart::send;` and must be reported
    # as one.  Chains inside a `use` tree are left to `iter_imports`, which
    # composes brace groups and reports the whole path at the path's start;
    # both spellings resolve through `_root_module_of` to the identical name,
    # so a repeat lands on the collector's `(line, rule, violation)` key and
    # collapses.  A chain that climbs past the crate root resolves to `None`
    # here because `super-escape` already reports it.
    use_spans = use_statement_spans(code)
    for match in _SUPER_CHAIN_RE.finditer(code):
        if any(start <= match.start() < end for start, end in use_spans):
            continue
        name, _after = _ident_at(code, _skip_space(code, match.end()))
        head = ("super",) * _super_chain_hops(code, match) + (name,)
        report_root(match.start(), _root_module_of(head, module))
    for match in SUBSYSTEM_FORBIDDEN_CRATE_RE[directory].finditer(code):
        collector.add(match.start(), f"{directory}-forbidden-crate", match.group(0))
    collector.found.sort(key=lambda item: (item.line, item.rule, item.violation))
    return collector.found


def subsystem_roots(src_root: str) -> dict[str, str]:
    """The real path of every [`SUBSYSTEM_PURITY`] row under ``src_root``.

    A row is normally the directory of the same name; ``logging`` overrides it
    with its single module file, which is why the returned value may name a file
    rather than a directory.  Keeping the resolution here means `main` and the
    tests can never disagree about what a row covers.
    """
    return {
        name: os.path.join(src_root, SUBSYSTEM_ROOT_OVERRIDES.get(name, name))
        for name in SUBSYSTEM_PURITY
    }


def _subsystem_of(path: str, roots: dict[str, str]) -> str | None:
    """The subsystem row that owns ``path``, if any.

    A row's root is the directory holding its files, or — for a subsystem that
    is a single module file (``logging.rs``) — that exact file.  A file in a
    subdirectory of a row's root belongs to that row too, so a nested module
    cannot escape the isolation rule by adding a directory level.  Both shapes
    are matched, so the table needs no second lookup path.
    """
    normalized = os.path.normpath(path)
    for name, root in roots.items():
        root = os.path.normpath(root)
        if normalized == root or normalized.startswith(root + os.sep):
            return name
    return None


# ---------------------------------------------------------------------------
# Universal rules (R-9): they apply to every audited root.
# ---------------------------------------------------------------------------


def _line_spans(text: str) -> list[tuple[int, int]]:
    """``(start, end)`` offsets of each line; ``end`` excludes the newline."""
    spans: list[tuple[int, int]] = []
    start = 0
    for index, char in enumerate(text):
        if char == "\n":
            spans.append((start, index))
            start = index + 1
    spans.append((start, len(text)))
    return spans


def _newline_is_live(stripped: bytearray, text: str, end: int) -> bool:
    """Whether the line ending at ``end`` survives test-code stripping.

    ``strip_test_code`` zeroes every byte of a ``#[cfg(test)]`` span, newline
    included, while a production comment keeps a live newline (its ``//`` text
    is masked by ``mask_source``, but the newline itself is code).  The final
    line of a file has no newline; fall back to its last character.
    """
    if end < len(text):
        return bool(stripped[end])
    return end > 0 and bool(stripped[end - 1])


def _documented_above(text: str, stripped: bytearray, line: int) -> bool:
    """Whether the contiguous ``//`` run directly above ``line`` says SAFETY:.

    ``line`` is the 1-based line of an ``unsafe`` token.  Every line of the run
    must be live production text: a ``// SAFETY:`` inside a ``#[cfg(test)]``
    module must not launder a production site that follows it.
    """
    # `split("\n")` and `_line_spans` agree with `_line_of`, which counts "\n".
    lines = text.split("\n")
    spans = _line_spans(text)
    documented = False
    cursor = line - 1
    while cursor >= 1:
        raw_line = lines[cursor - 1]
        if not raw_line.lstrip().startswith("//"):
            break
        _start, end = spans[cursor - 1]
        if not _newline_is_live(stripped, text, end):
            return False
        if "SAFETY:" in raw_line:
            documented = True
        cursor -= 1
    return documented


def audit_universal_source(
    text: str,
    path: str,
    module: tuple[str, ...],
    subsystem: str | None = None,
    composition_root: bool = False,
) -> tuple[list[Violation], int]:
    """Apply every universal rule to one file.

    ``module`` is the file's crate-relative module path (see
    [`_module_segments`]): its length bounds how far a ``super::`` chain may
    climb before it leaves the audited root, and it resolves a chain that lands
    *on* the root to the crate-root module it names.  ``subsystem`` names the
    [`SUBSYSTEM_PURITY`] row the file belongs to (a directory name such as
    ``"ota"``); such a file also gets that row's purity rules and, for the leaf
    rows, ``logging-macro``.  ``composition_root`` marks ``app.rs``/``irq.rs``,
    which may not reference the PAC at all.

    Returns the violations and the number of production ``unsafe`` tokens, so
    the caller can total the budget across the tree.
    """
    collector = _Collector(path, text)

    # state-removed: raw text on purpose (see the module docstring).
    for match in _STATE_REF_RE.finditer(text):
        collector.add(match.start(), "state-removed", "crate::state")

    stripped = strip_test_code(text, mask_source(text))
    code = blank_copy(text, stripped)

    # `use crate as krate;` (and the group form `use crate::{self as krate}`)
    # renames the crate root, which defeats every `crate::` rule below.
    for index, head in iter_imports(code):
        if head == ("crate",) or head[:2] == ("crate", "self"):
            collector.add(index, "crate-alias", _statement_of(text, index))

    # `extern crate self as krate;` renames the crate root the same way but is
    # not a `use` statement.  Flag the declaration itself, used or not: the
    # alias is what allows `krate::app::...` to dodge the path rules.
    for match in _EXTERN_CRATE_SELF_RE.finditer(code):
        collector.add(
            match.start(),
            "crate-alias",
            f"extern crate self as {match.group('alias')}",
        )

    # `super::` climbs the module tree.  The gate works on file paths, so a
    # chain deeper than the file's own module path must cross the audited root
    # and land above it — exactly the inversion this gate exists to stop.
    for match in _SUPER_CHAIN_RE.finditer(code):
        climbs = _super_chain_hops(code, match)
        if climbs > len(module):
            collector.add(
                match.start(),
                "super-escape",
                f"super::{climbs - len(module)} past the audited root "
                f"(chain of {climbs}, depth {len(module)})",
            )

    # `static`/`mut`/name may each start on a new line, and the reported text
    # stays one line: the source snippet is normalised, never embedded verbatim.
    for match in _STATIC_MUT_RE.finditer(code):
        collector.add(
            match.start(), "static-mut", " ".join(match.group(0).split())
        )

    # `undocumented-unsafe` reads `code`: a host-only `unsafe` is still real
    # Rust in this tree and still needs its `// SAFETY:` comment.
    for match in _UNSAFE_RE.finditer(code):
        line = _line_of(text, match.start())
        if not _documented_above(text, stripped, line):
            collector.add(
                match.start(),
                "undocumented-unsafe",
                "no `// SAFETY:` comment directly above",
            )

    # The budget alone ignores host-only code: it cannot reach the firmware.
    target_code = blank_copy(text, strip_host_only_code(text, stripped))
    unsafe_total = len(_UNSAFE_RE.findall(target_code))

    # The composition root reaches hardware exclusively through the HAL: any
    # PAC reference here (expression, path or import) is a layering inversion.
    if composition_root:
        for match in _PAC_REF_RE.finditer(code):
            collector.add(match.start(), "app-pac", "tlsr8266_pac")

    if subsystem is not None:
        collector.found.extend(
            audit_subsystem_purity_source(text, path, subsystem, module, code)
        )
        if subsystem in LOGGING_MACRO_FORBIDDEN_SUBSYSTEMS:
            for match in _LOGGING_RE.finditer(code):
                collector.add(
                    match.start(), "logging-macro", match.group(0).replace(" ", "")
                )

    collector.found.sort(key=lambda item: (item.line, item.rule, item.violation))
    return collector.found, unsafe_total


def _module_segments(path: str, root: str) -> tuple[str, ...]:
    """Crate-relative module path of ``path`` within the audited root.

    This is the one place that maps a file to the ``mod`` items the compiler
    would build from it, and both ``super::`` rules read it:

    * ``lib.rs`` / ``main.rs`` -> ``()`` — the crate root itself.
    * ``<dir>/mod.rs`` -> ``(<dir>, ...)`` — the directory's module.
    * ``<dir>/<name>.rs`` -> ``(<dir>, ..., <name>)`` — a child of ``mod.rs``.

    So ``rust/src/light/mod.rs`` is ``("light",)``, ``rust/src/ota/fsm.rs`` is
    ``("ota", "fsm")``, ``rust/src/app.rs`` is ``("app",)`` and
    ``tlsr8266-radio/src/callbacks.rs`` is ``("callbacks",)``.  The length is
    how many ``super::`` hops still land on the crate root; one more leaves the
    audited tree.  Module depth, not directory depth, is what the compiler
    resolves: a non-``mod.rs`` file is one ``mod`` deeper than its ``mod.rs``
    sibling, which is why ``super::super`` from ``ota/fsm.rs`` is legal.
    """
    parts = os.path.relpath(path, root).split(os.sep)
    *directories, filename = parts
    if not directories and filename in _CRATE_ROOT_FILES:
        return ()
    stem = filename.removesuffix(".rs")
    if directories and stem == "mod":
        return tuple(directories)
    return (*directories, stem)


def _is_composition_root(path: str, root: str) -> bool:
    """Whether ``path`` is ``app.rs``/``irq.rs`` directly under ``root``."""
    if os.path.basename(path) not in COMPOSITION_ROOT_FILES:
        return False
    directory = os.path.normpath(os.path.abspath(os.path.dirname(path)))
    return directory == os.path.normpath(os.path.abspath(root))


def _crate_name(root: str) -> str:
    """The Cargo package that owns an audited source root.

    ``rust/src`` belongs to ``tlsr8266_mesh`` and is the one root the directory
    name cannot answer, so it is resolved through [`ROOT_CRATE_NAMES`]; every
    other root is ``rust/crates/<package>/src`` and takes the manifest
    directory's name.
    """
    normalized = os.path.normpath(os.path.abspath(root))
    if normalized in ROOT_CRATE_NAMES:
        return ROOT_CRATE_NAMES[normalized]
    return os.path.basename(os.path.dirname(normalized))


def audit_universal(
    paths: list[str],
    root: str,
    *,
    crate_name: str | None = None,
    enforce_budget: bool = False,
    purity_roots: dict[str, str] | None = None,
) -> tuple[list[Violation], int]:
    """Audit every path; return violations and the tree-wide ``unsafe`` total.

    ``root`` is the audited tree root: it fixes each file's module depth and
    names the composition-root files, so a subset of paths (a subsystem, say)
    is still measured against the tree it lives in.

    ``crate_name`` selects the ceiling in [`MAX_PRODUCTION_UNSAFE_BY_CRATE`] and
    defaults to the package that owns ``root``; ``enforce_budget`` reports
    ``unsafe-budget`` only when the total *exceeds* that ceiling, so removing
    ``unsafe`` never fails the tool.  An unknown crate name with
    ``enforce_budget`` is a [`GateError`]: a root without a ceiling must not be
    silently unbudgeted.  Explicit ``--src-root``/``--crate-root`` runs pass
    ``False``, which is what keeps hermetic fixtures from being charged to a
    shipping ceiling.

    ``purity_roots`` maps a [`SUBSYSTEM_PURITY`] row name to the directory — or,
    for a single-file subsystem such as ``logging``, the exact file — that row
    covers; every file it owns also gets that subsystem's rules.  ``None``
    disables them, which is what the workspace crate trees want (they have no
    subsystems).
    """
    name = crate_name if crate_name is not None else _crate_name(root)
    ceiling = MAX_PRODUCTION_UNSAFE_BY_CRATE.get(name)
    if enforce_budget and ceiling is None:
        raise GateError(
            f"no `unsafe` ceiling for crate `{name}`: "
            "add it to MAX_PRODUCTION_UNSAFE_BY_CRATE"
        )

    violations: list[Violation] = []
    unsafe_total = 0
    roots = dict(purity_roots or {})
    for path in paths:
        with open(path, "r", encoding="utf-8") as handle:
            text = handle.read()
        found, count = audit_universal_source(
            text,
            _display_path(path),
            _module_segments(path, root),
            subsystem=_subsystem_of(path, roots),
            composition_root=_is_composition_root(path, root),
        )
        violations.extend(found)
        unsafe_total += count

    if enforce_budget and ceiling is not None and unsafe_total > ceiling:
        violations.append(
            Violation(
                _display_path(root),
                0,
                "unsafe-budget",
                f"{unsafe_total} production unsafe tokens exceeds ceiling "
                f"{ceiling} for {name} (remove unsafe, or justify and ratchet "
                "the map deliberately)",
            )
        )

    violations.sort(key=lambda item: (item.line, item.rule, item.violation))
    return violations, unsafe_total


def _display_path(path: str) -> str:
    try:
        relative = os.path.relpath(path, REPO_ROOT)
    except ValueError:  # different drive on Windows
        return path
    return path if relative.startswith(os.pardir) else relative


def rust_files(root: str) -> list[str]:
    paths: list[str] = []
    for directory, _subdirs, filenames in os.walk(root):
        for filename in filenames:
            if filename.endswith(".rs"):
                paths.append(os.path.join(directory, filename))
    return sorted(paths)


def _build_parser() -> argparse.ArgumentParser:
    default_crates = ", ".join(
        os.path.relpath(item, REPO_ROOT) for item in DEFAULT_CRATE_ROOTS
    )
    parser = argparse.ArgumentParser(
        description=(
            "Audit the Cargo workspace DAG, the firmware subsystem isolation and "
            "the per-crate `unsafe` ratchet over rust/src/ plus every "
            "rust/crates/*/src/ workspace crate."
        )
    )
    parser.add_argument(
        "--src-root",
        default=None,
        help=(
            "firmware source root for the universal pass "
            f"(default: {os.path.relpath(DEFAULT_SRC_ROOT, REPO_ROOT)}); naming "
            "it explicitly skips the Cargo DAG pass and the `unsafe` ratchet"
        ),
    )
    parser.add_argument(
        "--crate-root",
        action="append",
        default=None,
        help=(
            "workspace crate source root for the universal pass; repeatable "
            f"(default: every workspace crate, currently {default_crates}; "
            "explicit values audit only those paths and skip the Cargo DAG pass, "
            "the `unsafe` ratchet and — unless `--src-root` is also given — the "
            "firmware subsystem pass)"
        ),
    )
    return parser


def _resolve(value: str, flag: str) -> str | None:
    root = os.path.abspath(os.path.expanduser(value))
    if not os.path.isdir(root):
        print(f"error: {flag} {root} is not a directory", file=sys.stderr)
        return None
    return root


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    src_root_given = args.src_root is not None
    crate_roots_given = args.crate_root is not None
    # An explicit source root marks a hermetic fixture: its tree has no
    # `Cargo.toml`, and its `unsafe` count must not be charged to a shipping
    # ceiling.  Only the no-argument (CI) form runs the full gate.
    hermetic = src_root_given or crate_roots_given
    # An explicit `--crate-root` names exactly the trees to audit: the default
    # list is a convenience for the no-argument run, never an implicit extra
    # root, and the firmware pass then runs only when `--src-root` asks for it.
    run_src = src_root_given or not crate_roots_given

    dag_violations: list[Violation] = []
    if not hermetic:
        try:
            dag_violations = audit_cargo_dag(load_cargo_metadata(REPO_ROOT))
        except GateError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2

    src_root = ""
    src_paths: list[str] = []
    if run_src:
        resolved_src = _resolve(
            args.src_root if src_root_given else DEFAULT_SRC_ROOT,
            "--src-root",
        )
        if resolved_src is None:
            return 2
        src_root = resolved_src
        src_paths = rust_files(src_root)
        if not src_paths:
            print(f"error: no .rs files under {src_root}", file=sys.stderr)
            return 2

    # `(crate name, source root, .rs files)` per audited workspace crate.
    crate_audits: list[tuple[str, str, list[str]]] = []
    for value in args.crate_root if crate_roots_given else DEFAULT_CRATE_ROOTS:
        resolved_crate = _resolve(value, "--crate-root")
        if resolved_crate is None:
            return 2
        crate_paths = rust_files(resolved_crate)
        if not crate_paths:
            print(f"error: no .rs files under {resolved_crate}", file=sys.stderr)
            return 2
        crate_audits.append(
            (_crate_name(resolved_crate), resolved_crate, crate_paths)
        )

    src_violations: list[Violation] = []
    crate_violations: list[Violation] = []
    # `(label, display root, file count, violations)` per crate, so each root
    # reports itself instead of being folded into one anonymous total.
    crate_results: list[tuple[str, str, int, list[Violation]]] = []
    if hermetic:
        # A named root skips both the DAG pass and the ratchet; a `--crate-root`
        # without `--src-root` skips the firmware pass as well.  Name exactly
        # what was skipped: a copy-pasted invocation must not look like the full
        # gate.
        skipped = ["cargo DAG", "unsafe ratchet"]
        if not run_src:
            skipped.append("rust/src subsystem pass")
        print(f"hermetic mode: {', '.join(skipped[:-1])} and {skipped[-1]} skipped")
    try:
        if run_src:
            src_violations, _unsafe_total = audit_universal(
                src_paths,
                src_root,
                enforce_budget=not hermetic,
                purity_roots=subsystem_roots(src_root),
            )
        for label, crate_root, crate_paths in crate_audits:
            found, _crate_unsafe_total = audit_universal(
                crate_paths, crate_root, enforce_budget=not hermetic
            )
            crate_violations.extend(found)
            crate_results.append(
                (label, _display_path(crate_root), len(crate_paths), found)
            )
    except (OSError, UnicodeDecodeError, GateError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if not dag_violations and not src_violations and not crate_violations:
        if not hermetic:
            print(
                f"cargo dag OK: {len(CARGO_DAG)} workspace crates checked, "
                "0 violations"
            )
        if run_src:
            print(
                f"rust src audit OK: {len(src_paths)} files scanned under "
                f"{_display_path(src_root)}, 0 violations"
            )
        for label, display, count, _found in crate_results:
            print(
                f"{label} audit OK: {count} files scanned under "
                f"{display}, 0 violations"
            )
        return 0

    for violation in sorted(
        dag_violations + src_violations + crate_violations,
        key=lambda item: (item.path, item.line, item.rule, item.violation),
    ):
        print(violation.render())
    if dag_violations:
        print(
            f"cargo dag FAILED: {len(dag_violations)} violation(s) in "
            f"{len(CARGO_DAG)} workspace crates",
            file=sys.stderr,
        )
    if src_violations:
        print(
            f"rust src audit FAILED: {len(src_violations)} violation(s) "
            f"in {len(src_paths)} files",
            file=sys.stderr,
        )
    for label, _display, count, found in crate_results:
        if found:
            print(
                f"{label} audit FAILED: {len(found)} violation(s) "
                f"in {count} files",
                file=sys.stderr,
            )
    return 1


if __name__ == "__main__":
    sys.exit(main())
