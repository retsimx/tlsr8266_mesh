#!/usr/bin/env python3
"""Layering and ``unsafe`` gates for ``rust/src/``.

The script runs two independent passes.

**Pass 1 — SDK layering** over ``--root`` (default ``rust/src/sdk``).

The SDK is a from-scratch vendor layer: it must not reach up into the
application modules (``app``, ``mesh``, ``light_manager``, ...).  This gate
strips *test-only* code from every ``.rs`` file and then audits what is left:

* every ``.rs`` file may not reference the application layer at all, and may
  not call the application logging macros ``uprintln!`` / ``uprint!``;
* ``use crate as <ident>;`` and ``extern crate self as <ident>;`` are rejected:
  renaming the crate root would defeat every ``crate::`` rule below;
* a ``super::`` chain deeper than the file's directory depth is rejected: it
  leaves ``rust/src/sdk/`` and lands in ``rust/src/``, which is the inversion
  this gate exists to catch;
* ``sdk/common/`` may only import ``crate::sdk::common``, root macros, and a
  small set of external crates;
* ``sdk/mcu/`` additionally may import ``crate::sdk::mcu``;
* no other ``crate::sdk`` sibling is nameable from either layer.  Before C-5
  there was a third layer directory, ``sdk/drivers/``, allowed ``common``,
  ``mcu`` and ``drivers``; the peripheral drivers (``flash``, ``pwm``, ``spi``,
  ``uart``) now live in ``tlsr8266-hal``, so no layer directory remains under
  ``rust/src/sdk/`` besides ``ble_app``, which is not one.  The two layer rules
  stay armed for any future ``rust/src/sdk/<layer>`` path;
* ``sdk/pm.rs`` is a leaf even though it sits at the SDK root, so it obeys the
  ``mcu`` rule set: ``crate::sdk::common`` plus ``crate::sdk::mcu``, root macros
  and the ``mcu`` external crates, and nothing else.  It is the module the HAL
  extraction moves into ``tlsr8266-hal`` (C-4), so it must not reach upward into
  ``sdk/ble_app`` or ``sdk/light``; the rule makes that mechanically enforced
  instead of review-enforced.

``mesh_protocol`` (the pure workspace crate in ``rust/crates/mesh-protocol/``)
is an *external* crate to every SDK layer: it is a sibling workspace member
with no firmware dependencies, so importing it is never an inversion.  It is
listed in ``LAYER_EXTERNAL_CRATES`` next to ``core`` and ``critical_section``.
``tlsr8266_pac`` (``rust/crates/tlsr8266-pac/``) is the same kind of leaf
crate — register definitions and bit macros — but only ``mcu`` may name it:
``sdk/common`` sits below the MCU and has no register access.

**Pass 2 — universal rules** over the firmware root and every workspace crate
root: ``--src-root`` (default ``rust/src``, the firmware) and the repeatable
``--crate-root`` (default ``rust/crates/mesh-protocol/src``,
``rust/crates/tlsr8266-pac/src`` and ``rust/crates/tlsr8266-hal/src``).  All of
them are audited for zero production ``static mut`` and full ``// SAFETY:``
coverage; the ``unsafe`` budget ratchet below is measured over the firmware
root alone, so adding crate ``unsafe`` sites can neither hide behind nor widen
the firmware ceiling.

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

``MAX_PRODUCTION_UNSAFE`` is a **one-way ratchet**: it may be lowered as
``unsafe`` sites are removed, never raised.  The value below was produced by
running this exact rule on the finished tree, not by hand-counting tokens.
It is applied to ``--src-root`` only: the workspace crates are audited for
rule violations but are not budgeted.

Usage::

    python3 sdk/check_layering.py [--root rust/src/sdk] [--src-root rust/src]
                                  [--crate-root PATH ...]

``--crate-root`` is repeatable.  When it is omitted every workspace crate root
is audited; when it is given, exactly the named paths are, so a caller never
gets an implicit extra tree.  Passing ``--root`` explicitly without
``--src-root`` or ``--crate-root`` runs only the layering pass, and
``--crate-root`` alone runs only those crate audits: that is what keeps the
temp-tree unit tests hermetic.  The no-argument form (CI) runs every pass.

Exit codes: ``0`` clean, ``1`` one or more violations in either pass, ``2``
usage/IO error.  Violations are printed one per line as
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
DEFAULT_ROOT = os.path.join(REPO_ROOT, "rust", "src", "sdk")
DEFAULT_SRC_ROOT = os.path.join(REPO_ROOT, "rust", "src")
DEFAULT_CRATE_ROOT = os.path.join(REPO_ROOT, "rust", "crates", "mesh-protocol", "src")
# Every workspace crate audited by the universal pass when `--crate-root` is
# omitted.  A new crate is added here in the same change that creates it.
DEFAULT_CRATE_ROOTS = (
    DEFAULT_CRATE_ROOT,
    os.path.join(REPO_ROOT, "rust", "crates", "tlsr8266-pac", "src"),
    os.path.join(REPO_ROOT, "rust", "crates", "tlsr8266-hal", "src"),
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
MAX_PRODUCTION_UNSAFE = 64

# Application-layer modules `rust/src/sdk/` must never name.  `crate::config`
# is deliberately absent: it is the legitimate root-level constant module.
# `crate::state` is absent here too, but it is *not* legal: the universal
# `state-removed` rule bans it outright, tests included.
UPWARD_MODULES = frozenset(
    {
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
    }
)

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
# `drivers`, into `tlsr8266-hal`, so nothing under `rust/src/sdk/` is a layer
# now besides `ble_app` (which never was one).  The table stays armed for any
# future `rust/src/sdk/<layer>` path.
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
    ``depth`` is how many directories the file sits below the SDK root (``0``
    for ``rust/src/sdk/`` itself), which bounds how far a ``super::`` chain may
    climb before it leaves the SDK subtree.  ``None`` skips that bound.
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
    # chain deeper than the file's directory depth must cross `rust/src/sdk/`
    # and land in `rust/src/` — exactly the inversion this gate exists to stop.
    if depth is not None:
        for match in _SUPER_CHAIN_RE.finditer(code):
            climbs = _super_chain_hops(code, match)
            if climbs > depth:
                collector.add(
                    match.start(),
                    "super-escape",
                    f"super::{climbs - depth} past rust/src/sdk (chain of {climbs}, depth {depth})",
                )

    for index, head in iter_crate_references(code):
        module = head[0]
        if module in UPWARD_MODULES:
            collector.add(index, "upward-dep", f"crate::{module}")
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


def audit_universal_source(text: str, path: str) -> tuple[list[Violation], int]:
    """Apply the universal rules to one file.

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

    return collector.found, unsafe_total


def audit_universal(
    paths: list[str], label: str, enforce_budget: bool = True
) -> tuple[list[Violation], int]:
    """Audit every path; return violations and the tree-wide ``unsafe`` total.

    ``enforce_budget`` is asserted for the firmware root only: the ratchet was
    measured there, and the ``mesh-protocol`` crate is a separate tree whose
    ``unsafe`` sites are bounded by the rule itself, not by a ceiling.
    """
    violations: list[Violation] = []
    unsafe_total = 0
    for path in paths:
        with open(path, "r", encoding="utf-8") as handle:
            text = handle.read()
        found, count = audit_universal_source(text, _display_path(path))
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
    C-5 moved the peripheral drivers into ``tlsr8266-hal``; after C-4 and C-5 no
    layer directory remains under ``rust/src/sdk/`` besides ``ble_app``, which
    is not one.  The lookup stays for any future ``rust/src/sdk/<layer>`` path.
    ``pm.rs`` alone sits at the SDK root and is a leaf held to the ``mcu`` rules.
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
            "Audit rust/src/ for layering invariants and rust/src/ plus every "
            "rust/crates/*/src/ workspace crate for `unsafe` invariants."
        )
    )
    parser.add_argument(
        "--root",
        default=None,
        help=(
            "SDK source root for the layering pass "
            f"(default: {os.path.relpath(DEFAULT_ROOT, REPO_ROOT)})"
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


def _resolve(value: str | None, default: str, flag: str) -> str | None:
    root = os.path.abspath(os.path.expanduser(value if value is not None else default))
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
    # An explicit `--crate-root` names exactly the trees to audit: the default
    # list is a convenience for the no-argument run, never an implicit extra
    # root, and the firmware pass then runs only when `--src-root` asks for it.
    # An explicit `--root` on its own keeps the temp-tree unit tests hermetic:
    # they exercise the layering pass only.
    run_src = args.src_root is not None or (not root_given and not crate_roots_given)
    run_universal = run_src or crate_roots_given or not root_given

    root = _resolve(args.root, DEFAULT_ROOT, "--root")
    if root is None:
        return 2
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
            resolved_src = _resolve(args.src_root, DEFAULT_SRC_ROOT, "--src-root")
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
            resolved_crate = _resolve(value, value, "--crate-root")
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
                    src_paths, _display_path(src_root)
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
