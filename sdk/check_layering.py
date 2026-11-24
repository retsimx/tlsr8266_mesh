#!/usr/bin/env python3
"""Layering and ``unsafe`` gates for the firmware tree and its workspace crates.

The script runs two independent passes.

**Pass 1 — subtree layering**, only when ``--root`` is passed explicitly.

C-10 (#72) deleted ``rust/src/sdk/``: its files now live in
``rust/crates/tlsr8266-radio/``, which the universal pass audits like every
other workspace crate.  The layering pass therefore has no default root and
never runs in the no-argument (CI) form — the hermetic temp-tree tests are what
exercise it — and a directory that is merely absent can no longer produce
exit 2.

The layer it was written for was a from-scratch vendor layer: it must not reach
up into the application modules (``app``, ``mesh``, ``light_manager``, ...).
This gate strips *test-only* code from every ``.rs`` file and then audits what
is left:

* every ``.rs`` file may not reference the application layer at all, and may
  not call the application logging macros ``uprintln!`` / ``uprint!``;
* ``sdk-root-dep`` — production code may not name ``crate::config`` or
  ``crate::version`` (``SDK_FORBIDDEN_ROOTS``).  ``crate::config`` *was* the
  legitimate root constant module, and that is exactly why it had to be banned:
  the layer read board/product configuration straight out of the application
  root, so the radio could not be lifted out.  C-9 (#71) replaced those reads
  with an injected ``RadioConfig`` (``sdk/ble_app/config.rs``, now
  ``rust/crates/tlsr8266-radio/src/config.rs``), and this rule is what kept the
  seam honest until C-10 (#72) deleted ``rust/src/sdk/``: it was the difference
  between an extractable subtree and one that silently re-acquired an upward
  dependency before the move.  The rule reads ``production_code()``, so a
  ``#[cfg(test)]`` import is exempt; the issue's literal
  ``rg 'crate::(config|version)' rust/src/sdk/`` criterion — the tree C-10
  (#72) deleted — is what covered tests, and the two are meant to be satisfied
  together.  ``crate::config`` stays legal everywhere else in ``rust/src/``:
  the ban is scoped to the subtree handed to ``--root``;
* ``use crate as <ident>;`` and ``extern crate self as <ident>;`` are rejected:
  renaming the crate root would defeat every ``crate::`` rule below;
* a ``super::`` chain deeper than the file's directory depth is rejected: it
  leaves the audited root and lands in the tree above, which is the inversion
  this gate exists to catch (the ``rust/src/sdk/`` tree it was written for was
  deleted by C-10 (#72); the temp-tree fixtures still pin the rule);
* ``sdk/common/`` may only import ``crate::sdk::common``, root macros, and a
  small set of external crates;
* ``sdk/mcu/`` additionally may import ``crate::sdk::mcu``;
* no other ``crate::sdk`` sibling is nameable from either layer.  Before C-5
  there was a third layer directory, ``sdk/drivers/``, allowed ``common``,
  ``mcu`` and ``drivers``; the peripheral drivers (``flash``, ``pwm``, ``spi``,
  ``uart``) now live in ``tlsr8266-hal``.  C-4 and C-5 left only ``ble_app``
  under that root, which never was a layer, and C-10 (#72) then deleted
  ``rust/src/sdk/`` outright, so no layer directory ships today.  The two layer
  rules stay armed for any future subtree handed to ``--root``;
* ``sdk/pm.rs`` is a leaf even though it sits at that subtree's root, so it
  obeys the ``mcu`` rule set: ``crate::sdk::common`` plus ``crate::sdk::mcu``,
  root macros and the ``mcu`` external crates, and nothing else.  It is the
  module C-4 moved into ``tlsr8266-hal``, so it must not reach upward into
  ``sdk/ble_app`` or ``sdk/light``; the rule makes that mechanically enforced
  instead of review-enforced.

``mesh_protocol`` (the pure workspace crate in ``rust/crates/mesh-protocol/``)
is an *external* crate to every SDK layer: it is a sibling workspace member
with no firmware dependencies, so importing it is never an inversion.  It is
listed in ``LAYER_EXTERNAL_CRATES`` next to ``core`` and ``critical_section``.
``tlsr8266_pac`` (``rust/crates/tlsr8266-pac/``) is the same kind of leaf
crate — register definitions and bit macros — but only ``mcu`` may name it:
``sdk/common`` sits below the MCU and has no register access.

**Pass 2 — universal rules** over the composition root and every workspace
crate root: ``--src-root`` (default ``rust/src``, the firmware) and the
repeatable ``--crate-root`` (default ``rust/crates/mesh-protocol/src``,
``rust/crates/tlsr8266-pac/src``, ``rust/crates/tlsr8266-hal/src`` and
``rust/crates/tlsr8266-radio/src``).  All of them are audited for zero
production ``static mut`` and full ``// SAFETY:`` coverage; the ``unsafe``
budget ratchet below is measured over the firmware root alone, so adding crate
``unsafe`` sites can neither hide behind nor widen the firmware ceiling.  That
is exactly why C-10 (#72) could move 44 documented radio sites into the new
crate without re-cutting the ratchet around them.

Four permanent rules, one per line of defence won by R-9 (#45):

* ``state-removed`` — any ``crate::state`` reference is a violation.  This one
  rule scans the *whole raw file*, deliberately without the ``#[cfg(test)]``
  mask: test modules import the deleted module too, and the acceptance
  criterion is literally ``rg 'crate::state' rust/src/`` returning nothing.
  Whitespace around ``::`` is irrelevant to the Rust lexer and is matched;
  a crate alias cannot dodge it because ``c::state`` does not compile once the
  module is deleted.
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
* ``unsafe-budget`` — the total production ``unsafe`` token count over the
  whole source root may not exceed ``MAX_PRODUCTION_UNSAFE``.  Code gated by
  ``#[cfg(not(target_os = "none"))]`` is host-only: it never reaches the
  firmware image, so it is excluded from this count.  It is *not* exempt from
  the two rules above — a host-only ``unsafe`` still needs ``// SAFETY:`` and a
  host-only ``static mut`` is still an item.  Only the numeric budget ignores
  it; see ``strip_host_only_code``.

One path-scoped family of rules rides in the same pass:

* ``<subsystem>-purity`` — files directly under a subsystem directory of the
  firmware (``rust/src/<name>/**`` where ``<name>`` is a row of
  ``SUBSYSTEM_PURITY``) own their domain, so production code there may not name
  the ``crate::`` modules in that row nor import the external crates in it, and
  ``crate::config``, ``mesh_protocol``, ``tlsr8266_hal``, ``critical_section``
  and the subsystem's own ``fsm`` stay legal (a subsystem sits in
  ``rust/src/<name>/``, below the application layer and outside the deleted
  ``rust/src/sdk/`` subtree, so the SDK-only ``sdk-root-dep`` ban does not
  reach it).  Two rule names are
  reported per row: ``<name>-upward-dep`` (a forbidden ``crate::`` module) and
  ``<name>-forbidden-crate`` (a forbidden external crate).  The rules read
  ``production_code()`` only: a subsystem's own tests use
  ``tlsr8266_pac::fake_mmio::FakeMmio`` and must not be flagged, which is why a
  raw text grep would be wrong.  C-6 (#68) shipped the ``ota`` row; C-7 (#69)
  turned it into the table and added ``factory_reset``, whose production code
  may not name ``crate::sdk``, ``crate::light``, ``crate::ota``,
  ``crate::uart_manager``, ``crate::mesh``, ``crate::diagnostics``,
  ``crate::logging``, the deleted ``crate::state`` or ``tlsr8266_pac``, and
  whose sibling ``light`` row forbids ``crate::sdk``, ``crate::ota``,
  ``crate::uart_manager``, ``crate::mesh``, ``crate::factory_reset``,
  ``crate::diagnostics``, ``crate::logging``, ``crate::state`` and
  ``tlsr8266_pac`` (the lamp publishes its status through a hook the
  application registers, so it never needs the node table).

  C-8 (#70) completed the application layer with ``diagnostics``, ``logging``,
  ``uart`` and ``mesh``.  A row lists the siblings and layers *above* its
  subsystem, never its own name, so the DAG is enforced from both ends:
  ``uart`` forbids ``crate::mesh`` and ``mesh`` forbids ``crate::uart``.
  ``crate::uart`` is the live name of the UART bridge; ``crate::uart_manager``
  stays in the rows as a retired-name fence, so resurrecting the deleted module
  fails the same rule.  A subsystem that is a single file rather than a
  directory (``logging.rs``) declares its root in
  ``SUBSYSTEM_ROOT_OVERRIDES``: the match accepts an exact file path as well as
  a directory, so one table still carries every row.

``MAX_PRODUCTION_UNSAFE`` is a **one-way ratchet**: it may be lowered as
``unsafe`` sites are removed, never raised.  The value below was produced by
running this exact rule on the finished tree, not by hand-counting tokens.
It is applied to ``--src-root`` only: the workspace crates are audited for
rule violations but are not budgeted.

Usage::

    python3 sdk/check_layering.py [--src-root rust/src] [--crate-root PATH ...]
    python3 sdk/check_layering.py --root PATH [--src-root rust/src]

``--crate-root`` is repeatable.  When it is omitted every workspace crate root
is audited; when it is given, exactly the named paths are, so a caller never
gets an implicit extra tree.  ``--root`` is never defaulted: passing it runs the
layering pass (the temp-tree unit tests do), and passing it without
``--src-root`` or ``--crate-root`` runs only that pass.  ``--crate-root`` alone
runs only those crate audits.  The no-argument form (CI) audits the composition
root plus all four workspace crates and no layering pass.

Exit codes: ``0`` clean, ``1`` one or more violations in either pass, ``2``
usage/IO error or a root that was *explicitly* named but cannot be used.  A
default root that is absent cannot reach exit 2: the no-argument run has no
layering root to resolve.  Violations are printed one per line as
``file:line: rule: violation``.

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
import os
import re
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

# One-way ratchet: lower this when `unsafe` sites disappear, never raise it.
# Measured by running the `unsafe-budget` rule on the finished R-9 tree, so the
# number is reproducible from the gate itself rather than hand-counted:
#
#     python3 -c 'import sys; sys.path.insert(0, "sdk"); \
#       import check_layering as c; \
#       print(c.audit_universal(c.rust_files("rust/src"), "rust/src")[1])'
#
# 132 is the count *with* `#[cfg(test)]` on a struct field stripped correctly;
# the earlier 131 came from the stripper running past that field and masking the
# production `unsafe impl Send` in `sdk/common/sync.rs`.
#
# R-10 (#46) moved the pure wire crate out of the firmware tree: 45 of those
# documented sites now live in `rust/crates/mesh-protocol/`, which the gate
# audits for rule violations but does not budget, so the firmware ceiling drops
# with them: 132 - 45 = 87.
#
# C-2 (#64) moved the six volatile MMIO sites into `rust/crates/tlsr8266-pac/`,
# audited the same way and likewise unbudgeted: 87 - 6 = 81.
#
# C-4 (#66) moved the HAL layer into `rust/crates/tlsr8266-hal/`: `sync.rs` 9 +
# `irq_i.rs` 3 + `executor.rs` 1 = 13 documented sites leave the firmware tree
# (`pm.rs` contributed 0), so the ceiling follows them down: 81 - 13 = 68.
#
# C-5 (#67) moved `flash.rs` out of the firmware tree: three documented sites
# move with it, and the dead `core::slice::from_raw_parts` binding at the old
# `flash.rs:455` is deleted in the same change, so 4 leave the firmware tree:
# 68 - 4 = 64. (The HAL crate now holds 16 audited sites in total, of which
# 3 came from `flash.rs`; the firmware ceiling does not count the HAL.)
#
# C-7 (#69) replaced `sdk/factory_reset.rs`'s `read_flash_byte` (a
# `core::ptr::read_volatile` over the reset-count sector) with the HAL's safe
# `flash_read_page`, so that documented site leaves the firmware tree:
# 64 - 1 = 63.
#
# C-7 (#69) also deleted `vendor_light.rs`'s `slice::from_raw_parts` view of the
# primary advertisement payload: `light::effects::adv_private_to_bytes` encodes
# each field with `to_le_bytes()` instead, so one more documented site leaves
# the firmware tree: 63 - 1 = 62.
#
# C-8 (#70) moved `diagnostics::check_panic_info` to `app.rs` and replaced its
# two `slice::from_raw_parts` views of the panic report buffer with safe
# `chunks(10)`, so both documented sites leave the firmware tree: 62 - 2 = 60.
#
# C-10 (#72) moved the radio layer out of the firmware tree into
# `rust/crates/tlsr8266-radio/`: 44 RF/BLE sites (`gatt` 18 + `ble` 17 +
# `mesh_ll` 5 + `irq` 2 + `rf` 2) go with it, and the crate is audited for rule
# violations but not budgeted, exactly like `mesh-protocol`.  The ceiling
# follows them down: 60 - 44 = 16.  The remaining 16 are `startup` 4,
# `logging` 5, `diagnostics` 3, `ota` 2 and `uart` 2, measured with the command
# above after the deletion.
MAX_PRODUCTION_UNSAFE = 16

# Application-layer modules the extracted subtree must never name.  That subtree
# was `rust/src/sdk/` until C-10 (#72) deleted it — its files now live in
# `rust/crates/tlsr8266-radio/` — but the set stays armed for the layering pass,
# which is the regression cover the temp-tree tests still exercise.
# `crate::config` is deliberately absent from this set, but it is *not* legal
# either: C-9 (#71) banned `crate::config` and `crate::version` with
# `SDK_FORBIDDEN_ROOTS` below, because both were the application root leaking
# into the layer C-10 (#72) extracted as `crates/tlsr8266-radio`.  It stays out
# of this set because `upward-dep` and `sdk-root-dep` report different reasons
# (a layer inversion versus a root constant dependency) and the diagnostics
# should stay distinct.
# `crate::state` is absent here too, but it is likewise *not* legal: the
# universal `state-removed` rule bans it outright, tests included.  C-7 (#69)
# added `factory_reset` and `light`, which moved out of `sdk/` (and out of the
# old `light_manager` module) into app-root subsystems.  `light_manager` and
# `main_light` stay listed as retired-name fences: nothing may name them again.
# C-8 (#70) added `uart`: the UART bridge is now `rust/src/uart/`, and
# `uart_manager` stays in the set as the fence for its deleted spelling.
UPWARD_MODULES = frozenset(
    {
        "app",
        "common",
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
    }
)

# Crate-root modules the extracted subtree's production code must never name.
# That subtree was `rust/src/sdk/` until C-10 (#72) deleted it.  These are not
# *above* the subtree the way `UPWARD_MODULES` are — they are the shared
# root-level constant modules, and `crate::config` was the legitimate way for a
# module to read the board/product constants.  That legitimacy was the problem:
# the radio layer read `crate::config::*` and `crate::version::BUILD_VERSION`
# directly, so it could not be extracted into its own crate.  C-9 (#71) replaced
# those reads with the injected `RadioConfig` in `sdk/ble_app/config.rs`, and
# this ban kept the seam from regressing one import at a time until the move
# landed.  The scope is the pass this rule runs in — files under `--root` only —
# so `crate::config` stays legal everywhere else in `rust/src/` by construction.
SDK_FORBIDDEN_ROOTS = frozenset({"config", "version"})

# Modules `rust/src/ota/**` production code must never name.  `ota` is the OTA
# subsystem and owns all OTA state and logic, so it may reach downward only:
# `crate::config`, `mesh_protocol`, `tlsr8266_hal`, `critical_section`, and its
# own `ota::fsm` (internal, reachable as `super::fsm` or `crate::ota::fsm`, so it
# is not in this set).  Everything above `ota` is forbidden: the SDK, the light
# and mesh managers, the UART bridge (`crate::uart`, and its retired
# `crate::uart_manager` spelling), diagnostics, logging and the deleted `state`
# module.  Together with `OTA_FORBIDDEN_CRATES` this turns the issue's "zero
# imports" criterion into a checked rule instead of a review note.
OTA_FORBIDDEN_MODULES = frozenset(
    {
        "sdk",
        "light_manager",
        "main_light",
        "uart",
        "uart_manager",
        "mesh",
        "diagnostics",
        "logging",
        "state",
    }
)
# `tlsr8266_pac` is the register-definition leaf: `ota` drives flash and the
# radio only through `tlsr8266_hal`, never by poking registers itself.
OTA_FORBIDDEN_CRATES = frozenset({"tlsr8266_pac"})

# Modules `rust/src/factory_reset/**` production code must never name.  The
# subsystem owns boot-time reset detection and the configuration erase, and it
# reaches downward only: `tlsr8266_hal` (flash, clock), `crate::config`, `core`
# and the bare `critical_section` crate.  It returns `ResetAction`/
# `KickoutAction` instead of calling the layer above, so the whole application
# side of the reset (interrupts off, LED blink, reboot, credential writes) lives
# in the caller and is testable without it.  `crate::light` is forbidden too:
# `light` is a sibling subsystem, not a dependency, and so is the UART bridge
# (`crate::uart`, plus the retired `crate::uart_manager` spelling).
FACTORY_RESET_FORBIDDEN_MODULES = frozenset(
    {
        "sdk",
        "light",
        "ota",
        "uart",
        "uart_manager",
        "mesh",
        "diagnostics",
        "logging",
        "state",
    }
)
# `tlsr8266_pac` is forbidden for the same reason as in `ota`: the reset path
# drives flash through `tlsr8266_hal` only.
FACTORY_RESET_FORBIDDEN_CRATES = frozenset({"tlsr8266_pac"})

# Modules `rust/src/light/**` production code must never name.  The light
# subsystem owns the lamp, its PWM, its persistence, the LED indicator and the
# light-domain wire protocol, and it reaches downward only: `tlsr8266_hal`,
# `mesh_protocol`, `crate::config` and the lamp's own maths/queue crates.  Every
# layer above it is forbidden — including `factory_reset` and the UART bridge
# (`crate::uart`, plus its retired `crate::uart_manager` spelling), which are
# sibling subsystems and not dependencies — and so is the PAC: the lamp drives
# PWM, flash and timers through the HAL.
LIGHT_FORBIDDEN_MODULES = frozenset(
    {
        "sdk",
        "ota",
        "uart",
        "uart_manager",
        "mesh",
        "factory_reset",
        "diagnostics",
        "logging",
        "state",
    }
)
LIGHT_FORBIDDEN_CRATES = frozenset({"tlsr8266_pac"})

# Modules `rust/src/diagnostics/**` production code must never name.  It is the
# leaf of the application layer: the panic sector, the boot breadcrumbs and the
# RTT mirror, reaching downward only to `tlsr8266_hal`, `mesh_protocol` and
# `crate::config`.  Everything above it is forbidden, `logging` included: the
# sink is what writes the panic body, so the direction is
# `logging -> diagnostics::rtt`, never back.
DIAGNOSTICS_FORBIDDEN_MODULES = frozenset(
    {
        "sdk",
        "uart",
        "uart_manager",
        "mesh",
        "light",
        "ota",
        "factory_reset",
        "logging",
        "state",
    }
)
# `tlsr8266_pac` is forbidden for the same reason as everywhere else: the panic
# sector is written through the HAL's flash driver, not by poking registers.
DIAGNOSTICS_FORBIDDEN_CRATES = frozenset({"tlsr8266_pac"})

# Modules `rust/src/logging.rs` production code must never name.  `logging` sits
# above `diagnostics` (it mirrors a frame to RTT) and below `uart` (which calls
# `uprintln!`), so the UART transport, `mesh`, `light` and every subsystem are
# forbidden while `crate::diagnostics` is legal.  The transport is reached
# through the `LogSinks` fn-pointer seam `app.rs` registers — that is what keeps
# this row possible.
LOGGING_FORBIDDEN_MODULES = frozenset(
    {
        "sdk",
        "uart",
        "uart_manager",
        "mesh",
        "light",
        "ota",
        "factory_reset",
        "state",
    }
)
LOGGING_FORBIDDEN_CRATES = frozenset({"tlsr8266_pac"})

# Modules `rust/src/uart/**` production code must never name.  The bridge owns
# the framing, the ACK/retry bookkeeping and the two embassy tasks, and reaches
# downward only: `tlsr8266_hal`, `mesh_protocol`, `crate::logging` (for
# `uprintln!`) and its own `uart::protocol`.  The mesh relay and the host command
# execution arrive through the `UartTxOps` / `UartCommandHandler` seams, so
# `crate::mesh` and `crate::sdk` never appear here.
UART_FORBIDDEN_MODULES = frozenset(
    {
        "sdk",
        "mesh",
        "light",
        "ota",
        "factory_reset",
        "diagnostics",
        "state",
    }
)
UART_FORBIDDEN_CRATES = frozenset({"tlsr8266_pac"})

# Modules `rust/src/mesh/**` production code must never name.  `mesh` is allowed
# `crate::sdk` (`ble_app`'s queues, `light`/`ePairState`, `rf_drv_8266`) because
# it owns the radio and the node table, and `crate::config` / `crate::logging`
# like every other subsystem; it may not name the UART bridge, the lamp, the OTA
# or reset subsystems, or diagnostics.  `app` and `irq` are the composition root
# and the ISR layer above it, so they are forbidden too — the module doc's "it
# never names `app`" is a checked claim, not a comment.  The lamp status, the
# light events and the UART relay all arrive through `MeshEventHooks`.
MESH_FORBIDDEN_MODULES = frozenset(
    {
        "app",
        "irq",
        "uart",
        "uart_manager",
        "light",
        "ota",
        "factory_reset",
        "diagnostics",
        "state",
    }
)
MESH_FORBIDDEN_CRATES = frozenset({"tlsr8266_pac"})

# One row per subsystem directly under `rust/src/`: the `crate::` modules its
# production code must never name, and the external crates it must never import.
# The rule names are derived from the key, `<dir>-upward-dep` and
# `<dir>-forbidden-crate`, so `ota` keeps its C-6 spelling and a new subsystem
# only needs a row here.  `crate::config`, `mesh_protocol`, `tlsr8266_hal`,
# `critical_section` and the subsystem's own `fsm` stay legal everywhere.
SUBSYSTEM_PURITY = {
    "ota": (OTA_FORBIDDEN_MODULES, OTA_FORBIDDEN_CRATES),
    "factory_reset": (
        FACTORY_RESET_FORBIDDEN_MODULES,
        FACTORY_RESET_FORBIDDEN_CRATES,
    ),
    "light": (LIGHT_FORBIDDEN_MODULES, LIGHT_FORBIDDEN_CRATES),
    "diagnostics": (DIAGNOSTICS_FORBIDDEN_MODULES, DIAGNOSTICS_FORBIDDEN_CRATES),
    "logging": (LOGGING_FORBIDDEN_MODULES, LOGGING_FORBIDDEN_CRATES),
    "uart": (UART_FORBIDDEN_MODULES, UART_FORBIDDEN_CRATES),
    "mesh": (MESH_FORBIDDEN_MODULES, MESH_FORBIDDEN_CRATES),
}

# A subsystem is normally a directory of the same name, but `logging` is a
# single module file.  This override keeps one table: `main` resolves each row's
# root as `rust/src/<name>` unless the name appears here, and the purity match
# accepts an exact file path as well as a directory.
SUBSYSTEM_ROOT_OVERRIDES = {"logging": "logging.rs"}

LOGGING_MACROS = ("uprintln", "uprint")

# Root macros exported from `crate` (see `tlsr8266-hal`'s common/bit.rs, plus
# the bit macros re-exported from `tlsr8266_pac` in rust/src/lib.rs). Importing a
# macro is never a layering violation.
ROOT_MACROS = frozenset(
    {
        "BIT",
        "BIT_LOW_BIT",
        "BIT_MASK_LEN",
        "BIT_RNG",
        "BM_CLR",
        "BM_MASK_VAL",
        "BM_SET",
        "BM_SET_MASK_FLD",
        "CLR_FLD",
        "MASK_VAL",
        "SET_FLD",
        "SET_FLD_V",
        "regrw",
        "regrw_copy",
        "regrw_idx",
    }
)

# The `crate::sdk::<layer>` each layer may import.  Every layer may import the
# layers below it; nothing may import a layer above it.  `common` and `mcu` are
# the only layer directories the gate knows: C-4 moved them, and C-5 moved
# `drivers`, into `tlsr8266-hal`, and C-10 (#72) then deleted the `rust/src/sdk/`
# tree they came from, so no layer directory ships today.  The table stays armed
# for any future subtree handed to `--root`.
LAYER_SDK_IMPORTS = {
    "common": frozenset({"common"}),
    "mcu": frozenset({"common", "mcu"}),
}

# External crates each layer may import.  Adding a dependency here is a
# deliberate act: it widens what the bottom of the SDK is allowed to assume.
# `mesh_protocol` is the pure wire-format workspace crate: it has no firmware,
# MMIO or app dependencies, so every layer may name it.  `tlsr8266_pac` is the
# register/bit-macro leaf crate: it is MMIO, so `common` — below the MCU — may
# not name it.  `tlsr8266_hal` (C-4) is the extracted HAL layer: the remaining
# `mcu` modules consume it; `sdk/common` no longer exists in the firmware tree,
# so `common` stays exactly as it was.  The `drivers` row left with C-5, when
# the peripheral drivers themselves moved into `tlsr8266-hal`.
LAYER_EXTERNAL_CRATES = {
    "common": frozenset({"core", "critical_section", "mesh_protocol"}),
    "mcu": frozenset(
        {
            "core",
            "critical_section",
            "bitflags",
            "mesh_protocol",
            "tlsr8266_pac",
            "tlsr8266_hal",
        }
    ),
}

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
_IMPORT_RE = re.compile(
    r"(?m)^[ \t]*(?:pub(?:[ \t]*\([^)\n]*\))?[ \t]+)?use[ \t\n]+"
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
# `macro_rules!` body are both caught.  It must never match a *reference*:
# in `&'static mut self` and `-> &'static mut [T; N]` the character before
# `static` is the `'` of the lifetime, which the lookbehind rejects.
_STATIC_MUT_RE = re.compile(r"(?<![&'\w])static[ \t]+mut[ \t]+[A-Za-z_]\w*")
_UNSAFE_RE = re.compile(r"\bunsafe\b")


class Violation(NamedTuple):
    path: str
    line: int
    rule: str
    violation: str

    def render(self) -> str:
        return f"{self.path}:{self.line}: {self.rule}: {self.violation}"


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
    """Every ``use`` statement's path, as leading segments."""
    for match in _IMPORT_RE.finditer(code):
        yield from _parse_use_tree(code, match.end(), (), None)


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


def audit_source(
    text: str, path: str, layer: str | None, depth: int | None = None
) -> list[Violation]:
    """Audit one Rust source file.

    ``layer`` is ``common``/``mcu``, or ``None`` outside a layer directory.
    ``depth`` is how many directories the file sits below the audited root
    (``0`` for the root itself), which bounds how far a ``super::`` chain may
    climb before it leaves that subtree.  ``None`` skips that bound.
    """
    code = production_code(text)
    collector = _Collector(path, text)

    imports = list(iter_imports(code))

    # `use crate as krate;` (and the group form `use crate::{self as krate}`)
    # renames the crate root, which defeats every `crate::` rule below.
    for index, head in imports:
        if head == ("crate",) or head[:2] == ("crate", "self"):
            collector.add(index, "crate-alias", _statement_of(text, index))

    # `extern crate self as krate;` renames the crate root the same way but is
    # not a `use` statement.  Flag the declaration itself, used or not: the
    # alias is what allows `krate::app::...` to dodge the rules below.
    for match in _EXTERN_CRATE_SELF_RE.finditer(code):
        collector.add(
            match.start(),
            "crate-alias",
            f"extern crate self as {match.group('alias')}",
        )

    # `super::` climbs the module tree.  The gate works on file paths, so a
    # chain deeper than the file's directory depth must cross the audited root
    # and land above it — exactly the inversion this gate exists to stop.
    if depth is not None:
        for match in _SUPER_CHAIN_RE.finditer(code):
            climbs = _super_chain_hops(code, match)
            if climbs > depth:
                collector.add(
                    match.start(),
                    "super-escape",
                    f"super::{climbs - depth} past the audited root "
                    f"(chain of {climbs}, depth {depth})",
                )

    for index, head in iter_crate_references(code):
        module = head[0]
        if module in UPWARD_MODULES:
            collector.add(index, "upward-dep", f"crate::{module}")
        if module in SDK_FORBIDDEN_ROOTS:
            collector.add(index, "sdk-root-dep", f"crate::{module}")
        if layer is not None:
            if module == "sdk":
                allowed = len(head) > 1 and head[1] in LAYER_SDK_IMPORTS[layer]
            else:
                allowed = module in ROOT_MACROS
            if not allowed:
                collector.add(index, f"layer-{layer}", "crate::" + "::".join(head))

    for match in _LOGGING_RE.finditer(code):
        collector.add(match.start(), "logging", match.group(0).replace(" ", ""))

    if layer is not None:
        external = LAYER_EXTERNAL_CRATES[layer]
        for index, head in imports:
            root = head[0]
            if root in ("crate", "self", "super"):
                continue
            if root not in external:
                collector.add(
                    index, f"layer-{layer}", f"external crate `{root}`"
                )

    collector.found.sort(key=lambda item: (item.line, item.rule, item.violation))
    return collector.found


def _statement_of(text: str, index: int) -> str:
    """The ``use`` statement starting at ``index``, on one line."""
    stop = text.find("\n", index)
    stop = len(text) if stop < 0 else stop
    return text[index:stop].strip().rstrip(";")


# ---------------------------------------------------------------------------
# Subsystem purity: a `rust/src/<subsystem>/**` file must not name the layers
# above it.  The table lives in `SUBSYSTEM_PURITY`.
# ---------------------------------------------------------------------------


def audit_subsystem_purity_source(
    text: str, path: str, directory: str
) -> list[Violation]:
    """Audit one subsystem file for upward and forbidden names.

    ``directory`` is the key of its row in [`SUBSYSTEM_PURITY`]; the reported
    rules are ``<directory>-upward-dep`` and ``<directory>-forbidden-crate``.

    Runs on ``production_code()``, so the subsystem's own ``#[cfg(test)]``
    modules are exempt: they legitimately use
    ``tlsr8266_pac::fake_mmio::FakeMmio`` to stand in for the flash/radio
    register block, and a raw text grep would fail a correct tree.  Production
    code is what the issue's "zero imports" criterion is about, and that is what
    this reads — the same stripping that makes ``MAX_PRODUCTION_UNSAFE`` a
    production count.

    ``crate::`` references are read from the whole production text (so a
    qualified ``crate::sdk::..`` path is caught wherever it appears), while the
    forbidden external crate is read from ``use`` statements, matching how
    ``LAYER_EXTERNAL_CRATES`` has always worked.
    """
    forbidden_modules, forbidden_crates = SUBSYSTEM_PURITY[directory]
    code = production_code(text)
    collector = _Collector(path, text)
    for index, head in iter_crate_references(code):
        module = head[0]
        if module in forbidden_modules:
            collector.add(index, f"{directory}-upward-dep", f"crate::{module}")
    for index, head in iter_imports(code):
        root = head[0]
        if root in forbidden_crates:
            collector.add(index, f"{directory}-forbidden-crate", root)
    collector.found.sort(key=lambda item: (item.line, item.rule, item.violation))
    return collector.found


def audit_ota_purity_source(text: str, path: str) -> list[Violation]:
    """The ``ota`` row of [`SUBSYSTEM_PURITY`] (C-6's original rule)."""
    return audit_subsystem_purity_source(text, path, "ota")


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
    is a single module file (``logging.rs``) — that exact file.  Both shapes are
    matched, so the table needs no second lookup path.
    """
    normalized = os.path.normpath(path)
    directory = os.path.normpath(os.path.dirname(path))
    for name, root in roots.items():
        root = os.path.normpath(root)
        if directory == root or normalized == root:
            return name
    return None


# ---------------------------------------------------------------------------
# Universal rules (R-9): they apply to every file under `--src-root`.
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
    text: str, path: str, subsystem: str | None = None
) -> tuple[list[Violation], int]:
    """Apply the universal rules to one file.

    ``subsystem`` names the [`SUBSYSTEM_PURITY`] row the file belongs to (a
    directory name such as ``"ota"``); such a file also gets that subsystem's
    purity rule: it may not name the layers above it.

    Returns the violations and the number of production ``unsafe`` tokens, so
    the caller can total the budget across the tree.
    """
    collector = _Collector(path, text)

    # state-removed: raw text on purpose (see the module docstring).
    for match in _STATE_REF_RE.finditer(text):
        collector.add(match.start(), "state-removed", "crate::state")

    stripped = strip_test_code(text, mask_source(text))
    code = blank_copy(text, stripped)

    for match in _STATIC_MUT_RE.finditer(code):
        collector.add(match.start(), "static-mut", match.group(0).strip())

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

    if subsystem is not None:
        collector.found.extend(
            audit_subsystem_purity_source(text, path, subsystem)
        )

    return collector.found, unsafe_total


def audit_universal(
    paths: list[str],
    label: str,
    enforce_budget: bool = True,
    ota_root: str | None = None,
    subsystem_roots: dict[str, str] | None = None,
) -> tuple[list[Violation], int]:
    """Audit every path; return violations and the tree-wide ``unsafe`` total.

    ``enforce_budget`` is asserted for the firmware root only: the ratchet was
    measured there, and the ``mesh-protocol`` crate is a separate tree whose
    ``unsafe`` sites are bounded by the rule itself, not by a ceiling.

    ``subsystem_roots`` maps a [`SUBSYSTEM_PURITY`] row name to the directory —
    or, for a single-file subsystem such as ``logging``, the exact file — that
    row covers; every file it owns also gets that subsystem's purity rule.
    ``ota_root`` is C-6's spelling of the ``"ota"`` entry and is kept so existing
    callers do not have to change.  ``None`` on both disables the purity rules,
    which is what the workspace crate trees want (they have no subsystems).
    """
    violations: list[Violation] = []
    unsafe_total = 0
    roots = dict(subsystem_roots or {})
    if ota_root is not None:
        roots["ota"] = ota_root
    for path in paths:
        with open(path, "r", encoding="utf-8") as handle:
            text = handle.read()
        found, count = audit_universal_source(
            text, _display_path(path), subsystem=_subsystem_of(path, roots)
        )
        violations.extend(found)
        unsafe_total += count

    if enforce_budget and unsafe_total > MAX_PRODUCTION_UNSAFE:
        violations.append(
            Violation(
                label,
                0,
                "unsafe-budget",
                f"{unsafe_total} production unsafe tokens exceeds ceiling "
                f"{MAX_PRODUCTION_UNSAFE} (lower the ceiling, never raise it)",
            )
        )

    violations.sort(key=lambda item: (item.line, item.rule, item.violation))
    return violations, unsafe_total


def _layer_of(path: str, root: str) -> str | None:
    """The layer directory of ``path``: ``common``, ``mcu``, or ``None``.

    Only those two directory names are layers.  ``drivers`` was the third until
    C-5 moved the peripheral drivers into ``tlsr8266-hal``; C-4 and C-5 left
    only ``ble_app`` under the tree this gate used to default to, which never
    was a layer, and C-10 (#72) deleted that tree (``rust/src/sdk/``) outright.
    The lookup stays for any future subtree handed to ``--root``.  ``pm.rs``
    alone sits at that root and is a leaf held to the ``mcu`` rules.
    """
    parts = os.path.relpath(path, root).split(os.sep)
    if parts == ["pm.rs"]:
        return "mcu"          # sdk/pm.rs is a leaf: common + mcu only
    if len(parts) > 1 and parts[0] in LAYER_SDK_IMPORTS:
        return parts[0]
    return None


def _depth_of(path: str, root: str) -> int:
    """Directories between ``path`` and the SDK root (0 for the root itself)."""
    return len(os.path.relpath(path, root).split(os.sep)) - 1


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


def check_file(path: str, root: str) -> list[Violation]:
    with open(path, "r", encoding="utf-8") as handle:
        text = handle.read()
    return audit_source(
        text, _display_path(path), _layer_of(path, root), _depth_of(path, root)
    )


def _build_parser() -> argparse.ArgumentParser:
    default_crates = ", ".join(
        os.path.relpath(item, REPO_ROOT) for item in DEFAULT_CRATE_ROOTS
    )
    parser = argparse.ArgumentParser(
        description=(
            "Audit an explicitly named subtree for layering invariants, and "
            "rust/src/ plus every rust/crates/*/src/ workspace crate for "
            "`unsafe` invariants."
        )
    )
    parser.add_argument(
        "--root",
        default=None,
        help=(
            "subtree source root for the layering pass; never defaulted, so the "
            "pass runs only when this flag is given (the hermetic temp-tree "
            "tests pass it)"
        ),
    )
    parser.add_argument(
        "--src-root",
        default=None,
        help=(
            "firmware source root for the universal pass "
            f"(default: {os.path.relpath(DEFAULT_SRC_ROOT, REPO_ROOT)}; "
            "skipped when --root is given without --src-root/--crate-root)"
        ),
    )
    parser.add_argument(
        "--crate-root",
        action="append",
        default=None,
        help=(
            "workspace crate source root for the universal pass; repeatable "
            f"(default: every workspace crate, currently {default_crates}; "
            "explicit values audit only those paths)"
        ),
    )
    return parser


def _resolve(value: str, flag: str) -> str | None:
    root = os.path.abspath(os.path.expanduser(value))
    if not os.path.isdir(root):
        print(f"error: {flag} {root} is not a directory", file=sys.stderr)
        return None
    return root


def _crate_label(root: str) -> str:
    """The summary label for a crate root: ``rust/crates/<name>/src`` -> name."""
    return os.path.basename(os.path.dirname(root))


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    root_given = args.root is not None
    crate_roots_given = args.crate_root is not None
    # The layering pass runs only for an explicitly named `--root`: its default
    # root (`rust/src/sdk/`) was deleted by C-10 (#72), so a no-argument run must
    # never resolve it.  Resolving only what was asked for is what keeps an
    # absent directory from exiting 2, and it keeps the temp-tree unit tests
    # hermetic.
    #
    # An explicit `--crate-root` names exactly the trees to audit: the default
    # list is a convenience for the no-argument run, never an implicit extra
    # root, and the firmware pass then runs only when `--src-root` asks for it.
    run_src = args.src_root is not None or (not root_given and not crate_roots_given)
    run_universal = run_src or crate_roots_given or not root_given

    paths: list[str] = []
    root = ""
    if root_given:
        resolved_root = _resolve(args.root, "--root")
        if resolved_root is None:
            return 2
        root = resolved_root
        paths = rust_files(root)
        if not paths:
            print(f"error: no .rs files under {root}", file=sys.stderr)
            return 2

    src_root = ""
    src_paths: list[str] = []
    # `(label, source root, .rs files)` per audited workspace crate.
    crate_audits: list[tuple[str, str, list[str]]] = []
    if run_universal:
        if run_src:
            resolved_src = _resolve(
                args.src_root if args.src_root is not None else DEFAULT_SRC_ROOT,
                "--src-root",
            )
            if resolved_src is None:
                return 2
            src_root = resolved_src
            src_paths = rust_files(src_root)
            if not src_paths:
                print(f"error: no .rs files under {src_root}", file=sys.stderr)
                return 2

        crate_values = (
            args.crate_root if crate_roots_given else list(DEFAULT_CRATE_ROOTS)
        )
        for value in crate_values:
            resolved_crate = _resolve(value, "--crate-root")
            if resolved_crate is None:
                return 2
            crate_paths = rust_files(resolved_crate)
            if not crate_paths:
                print(f"error: no .rs files under {resolved_crate}", file=sys.stderr)
                return 2
            crate_audits.append(
                (_crate_label(resolved_crate), resolved_crate, crate_paths)
            )

    violations: list[Violation] = []
    universal: list[Violation] = []
    crate_universal: list[Violation] = []
    # `(label, display root, file count, violations)` per crate, so each root
    # reports itself instead of being folded into one anonymous total.
    crate_results: list[tuple[str, str, int, list[Violation]]] = []
    try:
        for path in paths:
            violations.extend(check_file(path, root))
        if run_universal:
            if run_src:
                universal, _unsafe_total = audit_universal(
                    src_paths,
                    _display_path(src_root),
                    subsystem_roots=subsystem_roots(src_root),
                )
            for label, crate_root, crate_paths in crate_audits:
                display = _display_path(crate_root)
                found, _crate_unsafe_total = audit_universal(
                    crate_paths, display, enforce_budget=False
                )
                crate_universal.extend(found)
                crate_results.append((label, display, len(crate_paths), found))
    except (OSError, UnicodeDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if not violations and not universal and not crate_universal:
        if root_given:
            print(f"sdk layering OK: {len(paths)} files scanned, 0 violations")
        if run_universal:
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
        violations + universal + crate_universal,
        key=lambda item: (item.path, item.line, item.rule, item.violation),
    ):
        print(violation.render())
    if violations:
        print(
            f"sdk layering FAILED: {len(violations)} violation(s) in {len(paths)} files",
            file=sys.stderr,
        )
    if universal:
        print(
            f"rust src audit FAILED: {len(universal)} violation(s) "
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
