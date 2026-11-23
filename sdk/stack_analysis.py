#!/usr/bin/env python3
"""Static stack-depth analyzer for the TLSR8266 (TC32) firmware ELF.

Walks the disassembly produced by ``llvm-objdump -d``, derives each function's
stack frame from its prologue (``push {..}`` + ``sub sp, #N``), builds the
directed call graph from ``bl``/``b``, and reports the deepest root-to-leaf
stack path for the main stack and the IRQ stack separately. The legacy TC32
spellings (``tpush``, ``tpop``, ``tsub sp``, ``tadd sp``, ``tjl``, ``tj``,
``tjex``) are accepted too, so both disassemblers feed the same model.

This exists to evidence the RAM budget: the main stack grows down from the top
of RAM into the free region above ``_end_bss_`` (see ``sdk/boot.link`` and
``rust/src/startup.rs``), so the deepest reachable chain is the number the RAM
gate must fit. The IRQ stack (``irq_stk``, 1792 B) is separate.

Model and caveats (reported alongside the result):
  * Conservative frame model: a callee's whole frame is charged on top of the
    caller's whole frame, ignoring call-site sp offsets. This is an upper bound.
  * Tail calls (``b <sym>`` / ``tj <sym>`` to a known function) do not grow the
    stack; the callee replaces the caller's frame.
  * Interrupt nesting on the IRQ stack is not modelled (assumes non-nested);
    the ``__irq`` register-save preamble is added as a fixed constant.
  * Indirect dispatch (``fn`` pointers, ``bx rN`` / legacy ``tjex``) has no
    ``bl`` edge to walk, so the known dispatchers are declared explicitly in
    ``INDIRECT_EDGES`` (caller substring -> callee substrings, resolved against
    the linked symbols). That covers every fn-pointer seam: the app callback
    seam, the OTA ``OtaHooks`` / ``OtaRadioOps`` seam, and the light subsystem's
    status-update hook (``light::notify_status_update``). An indirect dispatch
    pushes a return address, so it is modelled as a *normal* call, never a tail
    call. Every entry must resolve to exactly one function; otherwise the gate
    fails rather than silently scoring the missing callee as 0 B (which is what
    ``longest_path`` does with an unknown name).

Usage:
    python3 sdk/stack_analysis.py [--elf _build/lightblemesh] [--top 15]

It also acts as a build gate: it derives the main/IRQ stack budgets from the
ELF's linker symbols and exits non-zero if either margin drops below
``--min-margin`` (default 400 B, >= the largest observed frame) or if a call
cycle is detected. Wire it into the build/CI with ``make stack-check``.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from dataclasses import dataclass, field

# __irq saves r0-r3 + lr, then r12 and the saved status, before calling
# irq_handler on irq_stk. 5 + 1 + 1 registers * 4 bytes = 28 bytes. r4-r11 are
# callee-saved, so the AAPCS handler preserves them itself.
IRQ_PREAMBLE_BYTES = 28

HEADER_RE = re.compile(r"^([0-9a-f]+) <(.*)>:\s*$")
# Accept both llvm-objdump's standard ARM Thumb spellings and the legacy TC32
# `t*` spellings. `\s+` after the mnemonic keeps `bl` from matching `blx`, and
# the branch target may carry llvm-objdump's `0x` prefix.
CALL_RE = re.compile(r"\b(?:tjl|bl)\s+(?:0x)?[0-9a-f]+\s+<([^>]+)>")
TAIL_RE = re.compile(r"\b(?:tj|b)\s+(?:0x)?[0-9a-f]+\s+<([^>]+)>")
PUSH_RE = re.compile(r"\b(?:tpush|push)\s*\{([^}]*)\}")
POP_RE = re.compile(r"\b(?:tpop|pop)\s*\{([^}]*)\}")
_IMM = r"(0x[0-9a-fA-F]+|\d+)"
SUB_RE = re.compile(rf"\b(?:tsub(?:cs)?|sub)\s+sp,\s*#{_IMM}")
ADD_RE = re.compile(rf"\b(?:tadd(?:cs)?|add)\s+sp,\s*#{_IMM}")
# `bx lr` is a return, not an indirect branch; `bx rN` and legacy `tjex` are.
INDIRECT_RE = re.compile(r"\btjex\b|\bbx\s+(?!lr\b)\w")

# Dispatchers that reach the application through a function pointer, so the
# disassembly contains only a `bx rN` and no `bl` edge to the real callee.
# Keyed by a *caller* substring, valued by *callee* substrings; both are matched
# against the legacy-mangled symbol names objdump puts in `<...>` operands.
# Substrings rather than full names because the mangling hash changes whenever
# the crate is rebuilt, but a stale/ambiguous entry is a hard gate failure (see
# ``apply_indirect_edges``) instead of a silently missing edge.
#
# The OTA seam is the same shape and therefore needs the same entries: ``OtaHooks``
# (``sdk/ble_app/callbacks.rs``) and ``OtaRadioOps`` (``ota/mod.rs``) are plain
# structs of ``fn`` pointers registered at boot, so each of the eight seam calls
# compiles to a ``bx rN`` and its real callee — the OTA write path, the flash
# save loop, the ``BUFF_RESPONSE`` staging shims — would otherwise be scored as
# 0 B, hiding an OTA frame from both stack maxima.
INDIRECT_EDGES: dict[str, tuple[str, ...]] = {
    "emit_light_event": ("rf_link_light_event_callback",),
    # Deliberately the hook, not ``mesh::mesh_pair_proc``: the dispatcher calls
    # ``app::mesh_pair_proc_hook``, which reaches the free pairing function with
    # a real `bl`. The substring ``mesh_pair_proc`` alone is ambiguous (it also
    # matches ``mesh_pair_proc_effect`` and the hook itself), so it would invent
    # a false edge.
    "on_mesh_pair_proc": ("mesh_pair_proc_hook",),
    "on_light_data": ("rf_link_data_callback",),
    "on_light_response": ("rf_link_response_callback",),
    "on_uart_mesh_rx": ("light_mesh_rx_cb",),
    # The light subsystem publishes its own on/off state through a hook `app.rs`
    # registers at boot, so `device_status_update` reaches the SDK's node-table
    # writer with a `bx rN`, not a `bl`.  The dispatcher is IRQ-reachable:
    # `rf_link_light_event_callback -> device_status_update ->
    # notify_status_update`, which is on the deepest IRQ path today.  The
    # dispatcher itself is `#[inline(never)]`, so the pattern survives on the
    # optimised image.
    "notify_status_update": ("ll_device_status_update",),
    # The four OtaHooks dispatchers.  ``stage_ota_packet`` is deliberately not a
    # substring of ``read_staged_ota_packet`` (the latter has ``staged_``), which
    # is why the two provider names differ in that one letter: a shared stem
    # would make one of the two entries ambiguous.
    "dispatch_ota_write": ("on_gatt_ota_packet",),
    "dispatch_ota_snapshot": ("ota_status_snapshot_impl",),
    "dispatch_ota_terminate": ("ota_request_termination_impl",),
    "dispatch_ota_tick": ("ota_tick_timeout_1s_impl",),
    # The four OtaRadioOps dispatchers: `ota` reaches staging/RF upward only
    # through these slots, never with a direct `bl`.
    "radio_stage_pkt": ("stage_ota_packet",),
    "radio_read_pkt": ("read_staged_ota_packet",),
    "radio_terminate_tx": ("ota_send_terminate_hook",),
    "radio_begin_gatt_ota": ("ota_begin_gatt_hook",),
}


@dataclass
class Func:
    addr: int
    frame: int = 0
    calls: list[tuple[str, bool]] = field(default_factory=list)  # (name, is_tail)
    size: int = 0
    indirect: bool = False  # contains an indirect branch (bx rN / legacy tjex)


def reg_count(spec: str) -> int:
    """Count registers in a tpush/tpop list, expanding ``r0-r7`` ranges."""
    total = 0
    for part in spec.split(","):
        part = part.strip().lower()
        if not part:
            continue
        m = re.fullmatch(r"r(\d+)-r(\d+)", part)
        if m:
            lo, hi = int(m.group(1)), int(m.group(2))
            total += abs(hi - lo) + 1
        elif re.fullmatch(r"r\d+|lr|pc", part):
            total += 1
        else:
            # Unknown token: assume one 32-bit slot to stay conservative.
            total += 1
    return total


def parse_functions(disasm: str) -> dict[str, Func]:
    funcs: dict[str, Func] = {}
    current: Func | None = None
    name: str | None = None
    sp_off = 0
    max_off = 0

    for line in disasm.splitlines():
        header = HEADER_RE.match(line)
        if header:
            if current is not None and name is not None:
                current.frame = max_off
                funcs[name] = current
            current = Func(addr=int(header.group(1), 16))
            name = header.group(2)
            sp_off = 0
            max_off = 0
            continue

        if current is None:
            continue

        push = PUSH_RE.search(line)
        if push:
            sp_off += reg_count(push.group(1)) * 4
            max_off = max(max_off, sp_off)

        sub = SUB_RE.search(line)
        if sub:
            sp_off += int(sub.group(1), 0)
            max_off = max(max_off, sp_off)

        pop = POP_RE.search(line)
        if pop:
            sp_off = max(0, sp_off - reg_count(pop.group(1)) * 4)

        add = ADD_RE.search(line)
        if add:
            sp_off = max(0, sp_off - int(add.group(1), 0))

        call = CALL_RE.search(line)
        if call:
            current.calls.append((call.group(1), False))
            continue

        tail = TAIL_RE.search(line)
        if tail:
            current.calls.append((tail.group(1), True))
            continue

        if INDIRECT_RE.search(line):
            current.indirect = True

    if current is not None and name is not None:
        current.frame = max_off
        funcs[name] = current

    return funcs


class IndirectEdgeError(Exception):
    """A configured ``INDIRECT_EDGES`` entry did not resolve to one function."""


def _resolve_one(funcs: dict[str, Func], pattern: str, role: str, key: str) -> str:
    """Resolve a substring to exactly one parsed function, or raise.

    Zero matches means the table is stale (the dispatcher was inlined, renamed
    or dropped) and more than one means the pattern is ambiguous. Both are hard
    errors: ``longest_path`` scores an unknown callee as 0 B, so a silently
    dropped edge would understate the stack depth this gate exists to bound.
    """
    matches = sorted(name for name in funcs if pattern in name)
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise IndirectEdgeError(
            f"INDIRECT_EDGES[{key!r}]: {role} {pattern!r} matches no parsed "
            f"function (of {len(funcs)}); the table is stale - was the "
            f"dispatcher inlined or renamed?"
        )
    raise IndirectEdgeError(
        f"INDIRECT_EDGES[{key!r}]: {role} {pattern!r} is ambiguous, it matches "
        f"{len(matches)} functions: {matches}"
    )


def apply_indirect_edges(
    funcs: dict[str, Func], edges: dict[str, tuple[str, ...]] | None = None
) -> list[tuple[str, str]]:
    """Add ``INDIRECT_EDGES`` dispatch edges to ``Func.calls``.

    Must run after :func:`parse_functions` and before :func:`all_roots` /
    :func:`longest_path`, so roots, depth and cycle detection all see the same
    graph. Edges are appended as normal calls ``(callee, False)`` because an
    indirect dispatch pushes a return address; marking them tail would drop the
    dispatcher frame.

    Returns the resolved ``(caller, callee)`` pairs so the caller can report the
    symbols the table actually bound to. Raises :class:`IndirectEdgeError` if any
    entry does not resolve to exactly one function; nothing is mutated in that
    case.
    """
    table = INDIRECT_EDGES if edges is None else edges
    resolved: list[tuple[str, str]] = []
    for caller_pattern, callee_patterns in table.items():
        caller = _resolve_one(funcs, caller_pattern, "caller", caller_pattern)
        for callee_pattern in callee_patterns:
            callee = _resolve_one(funcs, callee_pattern, "callee", caller_pattern)
            resolved.append((caller, callee))

    # Mutate only once every entry resolved, so a rejected table leaves the call
    # graph untouched and the failure is reported instead of half-applied.
    for caller, callee in resolved:
        funcs[caller].calls.append((callee, False))
    return resolved


def longest_path(
    funcs: dict[str, Func],
    root: str,
    exclude: tuple[str, ...] = (),
) -> tuple[int, list[str], list[str]]:
    """Return (max_bytes, deepest chain, cycle list) for a root.

    A tail callee replaces the caller's frame, so its depth is used directly;
    a normal callee is charged after the caller's frame. Functions whose name
    contains any ``exclude`` substring are treated as leaves (frame counted,
    callees ignored) so panic/unwind paths can be reported separately.
    """
    memo: dict[str, tuple[int, list[str]]] = {}
    in_progress: set[str] = set()
    cycles: list[str] = []

    def visit(name: str) -> tuple[int, list[str]]:
        if name in memo:
            return memo[name]
        if name in in_progress:
            cycles.append(name)
            return (0, [name])

        in_progress.add(name)
        node = funcs.get(name)
        if node is None:
            in_progress.discard(name)
            memo[name] = (0, [name])
            return memo[name]

        best = node.frame
        best_chain = [name]
        if not any(pat in name for pat in exclude):
            for callee, is_tail in node.calls:
                sub_bytes, sub_chain = visit(callee)
                candidate = sub_bytes if is_tail else node.frame + sub_bytes
                if candidate > best:
                    best = candidate
                    best_chain = [name] + sub_chain

        in_progress.discard(name)
        memo[name] = (best, best_chain)
        return memo[name]

    total, chain = visit(root)
    return total, chain, cycles


def _llvm_tool(name: str, env_key: str, *defaults: str) -> str:
    """Resolve an LLVM tool: explicit env var, then the repo build, then PATH."""
    from os import environ
    from pathlib import Path
    from shutil import which

    explicit = environ.get(env_key)
    if explicit:
        return explicit
    for candidate in defaults:
        if Path(candidate).is_file():
            return candidate
    found = which(name)
    if found:
        return found
    return defaults[0]


def objdump(elf: str) -> str:
    tool = _llvm_tool(
        "llvm-objdump",
        "LLVM_OBJDUMP",
        "../llvm/build/bin/llvm-objdump",
        "../llvm/build-native/bin/llvm-objdump",
    )
    try:
        proc = subprocess.run(
            [tool, "-d", "--mattr=+tc32", elf],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError as exc:
        sys.exit(
            f"stack gate: cannot run {tool} ({exc}); "
            "set LLVM_OBJDUMP to the LLVM fork's llvm-objdump"
        )
    if proc.returncode != 0:
        sys.exit(f"objdump failed: {proc.stderr.strip() or proc.returncode}")
    # Fail closed: a disassembler without the TC32 feature warns on stderr and
    # then decodes the permuted bytes as gibberish, which would silently shrink
    # every measured frame. Refuse to report a margin we cannot trust.
    if proc.stderr.strip():
        first = proc.stderr.strip().splitlines()[0]
        sys.exit(
            f"stack gate: {tool} cannot decode TC32 ({first}); "
            "set LLVM_OBJDUMP to the LLVM fork's llvm-objdump"
        )
    if proc.stdout.count("\n") and proc.stdout.count("<") < 20:
        sys.exit(
            f"stack gate: {tool} produced an implausible disassembly; "
            "set LLVM_OBJDUMP to the LLVM fork's llvm-objdump"
        )
    return proc.stdout


def report(label: str, total: int, chain: list[str], funcs: dict[str, Func]) -> None:
    print(f"\n{label}: max {total} bytes")
    print(f"  deepest chain ({len(chain)} frames):")
    for depth, name in enumerate(chain):
        frame = funcs[name].frame if name in funcs else 0
        print(f"    {frame:4d} B  {name}")


def all_roots(funcs: dict[str, Func]) -> list[str]:
    """Functions never targeted by a direct/tail call.

    Embassy invokes each task's ``poll`` through a function pointer (``bx rN``),
    so those poll bodies have no direct caller and must be treated as roots of
    the main stack. Any genuinely dead function also lands here; since the
    linker garbage-collects unreferenced code, address-taken functions are the
    realistic set and taking their maximum is a valid upper bound.
    """
    targeted: set[str] = set()
    for f in funcs.values():
        for callee, _ in f.calls:
            if callee in funcs:
                targeted.add(callee)
    return [name for name in funcs if name not in targeted]


def executor_overhead(funcs: dict[str, Func]) -> int:
    """Frames of main_entrypoint -> Executor::run -> Executor::poll.

    These sit above every task poll body, which is reached indirectly. Chains
    rooted at ``main_entrypoint`` already include them; task roots do not.
    """
    total = 0
    for name, f in funcs.items():
        if (
            name == "main_entrypoint"
            or "Executor3run" in name
            or "Executor4poll" in name
        ):
            total += f.frame
    return total


def max_over_roots(
    funcs: dict[str, Func], exclude: tuple[str, ...] = ()
) -> tuple[int, list[str], str, list[str]]:
    overhead = executor_overhead(funcs)
    best = (0, [], "", [])
    cycles: list[str] = []
    for root in all_roots(funcs):
        depth, chain, cyc = longest_path(funcs, root, exclude)
        cycles += cyc
        # Indirectly-dispatched task poll bodies need the executor frames.
        if "TaskStorage" in root:
            depth += overhead
        if depth > best[0]:
            best = (depth, chain, root, cyc)
    return best[0], best[1], best[2], cycles


def read_symbols(elf: str) -> dict[str, int]:
    tool = _llvm_tool(
        "llvm-nm",
        "LLVM_NM",
        "../llvm/build/bin/llvm-nm",
        "../llvm/build-native/bin/llvm-nm",
    )
    try:
        proc = subprocess.run(
            [tool, elf], capture_output=True, text=True, check=False
        )
    except OSError as exc:
        sys.exit(
            f"stack gate: cannot run {tool} ({exc}); "
            "set LLVM_NM to the LLVM fork's llvm-nm"
        )
    syms: dict[str, int] = {}
    for line in proc.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 3:
            try:
                syms[parts[2]] = int(parts[0], 16)
            except ValueError:
                pass
    return syms


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--elf", default="_build/lightblemesh")
    ap.add_argument("--top", type=int, default=15)
    ap.add_argument(
        "--min-margin",
        type=int,
        default=400,
        help="required free bytes per stack (>= largest observed frame)",
    )
    ap.add_argument(
        "--exclude-panic",
        action="store_true",
        help="also report depth ignoring panic/unwind callees",
    )
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    funcs = parse_functions(objdump(args.elf))
    if not funcs:
        sys.exit("no functions parsed; is the ELF built?")

    print(f"parsed {len(funcs)} functions from {args.elf}")
    indirect = sum(1 for f in funcs.values() if f.indirect)
    print(f"functions with indirect (bx/tjex) branches: {indirect}")

    # Must precede every root/depth query: an unmapped dispatcher would score
    # its callback as 0 B and silently understate the margin.
    try:
        edges = apply_indirect_edges(funcs)
    except IndirectEdgeError as exc:
        print(f"\nFAIL: {exc}")
        return 1
    print(f"indirect dispatch edges ({len(edges)}):")
    for caller, callee in edges:
        print(f"  {caller} -> {callee}")

    # Root the main stack at every in-degree-0 function, not __start: the
    # __start block contains the hardware vector table (tj __irq / tj __reset),
    # and embassy dispatches task polls indirectly.
    main_max, chain, root, cycles = max_over_roots(funcs)
    report(
        f"MAIN STACK (deepest of {len(all_roots(funcs))} roots; root {root})",
        main_max,
        chain,
        funcs,
    )

    if args.exclude_panic:
        calm, calm_chain, croot, _ = max_over_roots(funcs, ("panic", "unwind"))
        report(
            f"MAIN STACK excluding panic/unwind (root {croot})",
            calm,
            calm_chain,
            funcs,
        )

    irq_max = 0
    if "irq_handler" in funcs:
        irq_bytes, irq_chain, irq_cycles = longest_path(funcs, "irq_handler")
        irq_max = irq_bytes + IRQ_PREAMBLE_BYTES
        report(
            f"IRQ STACK (root irq_handler, +{IRQ_PREAMBLE_BYTES} B __irq preamble)",
            irq_max,
            irq_chain,
            funcs,
        )
        cycles += irq_cycles
        if args.exclude_panic:
            calm, calm_chain, _ = longest_path(
                funcs, "irq_handler", ("panic", "unwind")
            )
            report(
                "IRQ STACK excluding panic/unwind",
                calm + IRQ_PREAMBLE_BYTES,
                calm_chain,
                funcs,
            )

    print(f"\ntop {args.top} functions by frame size:")
    for f in sorted(funcs.values(), key=lambda x: x.frame, reverse=True)[: args.top]:
        print(f"  {f.frame:5d} B  @0x{f.addr:06x}")

    # ---- budget gate ----------------------------------------------------
    syms = read_symbols(args.elf)
    needed = ["__RAM_START_ADDR", "__RAM_SIZE_MAX", "_end_bss_", "IRQ_STK_SIZE"]
    missing = [n for n in needed if n not in syms]
    if missing:
        print(f"\nGATE ERROR: missing linker symbols: {missing}")
        return 2

    ram_top = syms["__RAM_START_ADDR"] + syms["__RAM_SIZE_MAX"]
    free_main = ram_top - syms["_end_bss_"]
    irq_budget = syms["IRQ_STK_SIZE"]
    main_margin = free_main - main_max
    irq_margin = irq_budget - irq_max

    print("\nRAM GATE")
    print(f"  main: free {free_main} B - max {main_max} B = margin {main_margin} B")
    print(f"  irq:  size {irq_budget} B - max {irq_max} B = margin {irq_margin} B")
    print(f"  required margin: {args.min_margin} B")

    ok = True
    if cycles:
        print(f"  FAIL: recursion detected via {sorted(set(cycles))}")
        ok = False
    if main_margin < args.min_margin:
        print("  FAIL: main stack margin below minimum")
        ok = False
    if irq_margin < args.min_margin:
        print("  FAIL: IRQ stack margin below minimum")
        ok = False

    print("  PASS" if ok else "  FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
