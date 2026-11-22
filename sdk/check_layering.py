#!/usr/bin/env python3
"""Layering gate for ``rust/src/sdk/``.

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
* ``sdk/drivers/`` additionally may import ``crate::sdk::drivers``.

Usage::

    python3 sdk/check_layering.py [--root rust/src/sdk]

Exit codes: ``0`` clean, ``1`` one or more violations, ``2`` usage/IO error.
Violations are printed one per line as ``file:line: rule: violation``.

The test stripper removes only ``#[cfg(test)]`` items and ``#[cfg(test)] mod
tests { ... }`` blocks, using brace balancing, so production code sitting
*below* an inline ``#[cfg(test)]`` item is still audited.  Only strict
``#[cfg(test)]`` gates are stripped; ``#[cfg(not(test))]`` and
``#[cfg_attr(test, ...)]`` are production code.
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

# Application-layer modules `rust/src/sdk/` must never name.  `crate::state`
# and `crate::config` are deliberately absent: they are deferred to #45 and
# stay legal for now.
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

# Root macros exported from `crate` (see sdk/common/bit.rs and
# sdk/mcu/register.rs).  Importing a macro is never a layering violation.
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
# layers below it; nothing may import a layer above it.
LAYER_SDK_IMPORTS = {
    "common": frozenset({"common"}),
    "mcu": frozenset({"common", "mcu"}),
    "drivers": frozenset({"common", "mcu", "drivers"}),
}

# External crates each layer may import.  Adding a dependency here is a
# deliberate act: it widens what the bottom of the SDK is allowed to assume.
LAYER_EXTERNAL_CRATES = {
    "common": frozenset({"core", "critical_section"}),
    "mcu": frozenset({"core", "critical_section", "bitflags"}),
    "drivers": frozenset(
        {"core", "critical_section", "bitflags", "embassy_sync", "embassy_time"}
    ),
}

_CFG_TEST_RE = re.compile(r"#\s*\[\s*cfg\s*\(\s*test\s*\)\s*\]")
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


# ---------------------------------------------------------------------------
# Test stripping: remove only `#[cfg(test)]` items, never truncate the file.
# ---------------------------------------------------------------------------


def _item_extent(code: str, index: int) -> int | None:
    """End index (exclusive) of the item starting at ``index``.

    Attributes and whitespace are skipped, then the first top-level ``{`` is
    balanced to its ``}``; an item without a body ends at its top-level ``;``.
    """
    length = len(code)
    while True:
        index = _skip_space(code, index)
        if index < length and code[index] == "#":
            cursor = _skip_space(code, index + 1)
            if cursor < length and code[cursor] == "!":
                cursor = _skip_space(code, cursor + 1)
            if cursor < length and code[cursor] == "[":
                close = _matching(code, cursor, "[", "]")
                if close < 0:
                    return None
                index = close + 1
                continue
        break
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
    """Zero the mask over every ``#[cfg(test)]`` item / module."""
    code = blank_copy(text, mask)
    stripped = bytearray(mask)
    for match in _CFG_TEST_RE.finditer(code):
        end = _item_extent(code, match.end())
        if end is not None:
            stripped[match.start() : end] = b"\x00" * (end - match.start())
    return stripped


def production_code(text: str) -> str:
    """The file with test-only code blanked out, keeping line numbers intact."""
    return blank_copy(text, strip_test_code(text, mask_source(text)))


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

    ``layer`` is ``common``/``mcu``/``drivers``.  ``depth`` is how many
    directories the file sits below the SDK root (``0`` for ``rust/src/sdk/``
    itself), which bounds how far a ``super::`` chain may climb before it
    leaves the SDK subtree.  ``None`` skips that bound.
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


def _layer_of(path: str, root: str) -> str | None:
    parts = os.path.relpath(path, root).split(os.sep)
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
    parser = argparse.ArgumentParser(
        description="Audit rust/src/sdk/ for upward (SDK -> app) dependencies."
    )
    parser.add_argument(
        "--root",
        default=DEFAULT_ROOT,
        help=f"SDK source root to audit (default: {DEFAULT_ROOT})",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    root = os.path.abspath(os.path.expanduser(args.root))
    if not os.path.isdir(root):
        print(f"error: --root {root} is not a directory", file=sys.stderr)
        return 2
    paths = rust_files(root)
    if not paths:
        print(f"error: no .rs files under {root}", file=sys.stderr)
        return 2

    violations: list[Violation] = []
    try:
        for path in paths:
            violations.extend(check_file(path, root))
    except (OSError, UnicodeDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    violations.sort(key=lambda item: (item.path, item.line, item.rule, item.violation))

    if not violations:
        print(f"sdk layering OK: {len(paths)} files scanned, 0 violations")
        return 0

    for violation in violations:
        print(violation.render())
    print(
        f"sdk layering FAILED: {len(violations)} violation(s) in {len(paths)} files",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
