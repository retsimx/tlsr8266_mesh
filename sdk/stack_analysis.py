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
    the linked symbols). That covers every fn-pointer seam in the firmware: the
    app callback seam, the OTA ``OtaHooks`` / ``OtaRadioOps`` seam, the light
    subsystem's status-update hook (``light::notify_status_update``), and the
    ``mesh`` / ``logging`` / ``uart`` seams C-8 (#70) introduced
    (``MeshEventHooks``, ``LogSinks``, ``UartTxOps`` and the UART command
    handler). C-10 (#72) moved the app callback dispatchers and the OTA radio
    providers into ``tlsr8266-radio``, so those symbols now start with the
    radio crate's prefix; the patterns below did not change, because they are
    matched against the linked symbol names rather than module paths. An
    indirect dispatch pushes a return address, so it is modelled as a *normal*
    call, never a tail call. Every entry must resolve to exactly one function;
    otherwise the gate fails rather than silently scoring the missing callee as
    0 B (which is what ``longest_path`` does with an unknown name).

Usage:
    python3 sdk/stack_analysis.py [--elf _build/lightblemesh] [--top 15]

It also acts as a build gate: it derives the main/IRQ stack budgets from the
ELF's linker symbols and exits non-zero if either margin drops below
``--min-margin`` (default 400 B, >= the largest observed frame) or if a call
cycle is detected. Wire it into the build/CI with ``make stack-check``.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field

# __irq saves r0-r3 + lr, then r12 and the saved status, before calling
# irq_handler on irq_stk. 5 + 1 + 1 registers * 4 bytes = 28 bytes. r4-r11 are
# callee-saved, so the AAPCS handler preserves them itself.
IRQ_PREAMBLE_BYTES = 28

HEADER_RE = re.compile(r"^([0-9a-fA-F]+) <(.*)>:\s*$")
# Accept both llvm-objdump's standard ARM Thumb spellings and the legacy TC32
# `t*` spellings. The optional address before `<symbol>` covers Thumb/TC32
# disassemblers that emit either `bl 0x1234 <sym>` or `bl <sym>`.
CALL_RE = re.compile(
    r"\b(?:tjl|bl|blx)\b\s+(?:(?:0x)?[0-9a-fA-F]+\s+)?<([^>]+)>"
)
TAIL_RE = re.compile(
    r"\b(?:tj|b)\b\s+(?:(?:0x)?[0-9a-fA-F]+\s+)?<([^>]+)>"
)
PUSH_RE = re.compile(r"\b(?:tpush|push)\s*\{([^}]*)\}")
POP_RE = re.compile(r"\b(?:tpop|pop)\s*\{([^}]*)\}")
_IMM = r"(0x[0-9a-fA-F]+|\d+)"
SUB_RE = re.compile(rf"\b(?:tsub(?:cs)?|sub)\s+sp,\s*#{_IMM}")
ADD_RE = re.compile(rf"\b(?:tadd(?:cs)?|add)\s+sp,\s*#{_IMM}")
# `bx lr`/`bx r14`/`bx pc` are returns or CPU special cases, not indirect
# dispatches. Thumb/TC32 `bx rN`, emitted `blx rN`/numeric targets, and legacy
# `tjex` are indirect unless a symbolic `bl/blx` target was parsed above.
INDIRECT_RE = re.compile(
    r"\btjex\b(?:\s+\S+)?"
    r"|\bbx\b\s+(?!lr\b|r14\b|pc\b)\S+"
    r"|\bblx\b\s+(?!lr\b|r14\b|pc\b)(?:r\d+\b|sp\b|ip\b|"
    r"(?:0x)?[0-9a-fA-F]+\b(?!\s+<))"
)
INSTRUCTION_ADDRESS_RE = re.compile(r"^\s*([0-9a-fA-F]+):")

# Dispatchers that reach the application through a function pointer, so the
# disassembly contains only a `bx rN` and no `bl` edge to the real callee.
# Keyed by a *caller* substring, valued by *callee* substrings; both are matched
# against source-qualified/demangled names produced by ``llvm-objdump --demangle``.
# Legacy TC32 disassembler spellings are accepted, but strict resolve logic uses
# stable source-qualified names and namespaced ABI exports rather than Rust-v0
# crate hashes. Substrings rather than full names
# because the crate disambiguator changes whenever the crate is rebuilt, but a
# stale/ambiguous entry is a hard gate failure (see ``apply_indirect_edges``)
# instead of a silently missing edge.
#
# The OTA seam is the same shape and therefore needs the same entries:
# ``OtaHooks`` (``callbacks.rs`` in the radio crate) and ``OtaRadioOps``
# (``ota/mod.rs``) are plain structs of ``fn`` pointers registered at boot, so
# each of the eight seam calls compiles to a ``bx rN`` and its real callee — the
# OTA write path, the flash save loop, the ``BUFF_RESPONSE`` staging shims —
# would otherwise be scored as 0 B, hiding an OTA frame from both stack maxima.
#
# C-8 (#70) added the last four subsystems' seams, so every fn-pointer slot in
# the firmware is modelled: ``MeshEventHooks`` (`mesh`), ``LogSinks``
# (`logging`), ``UartTxOps`` + the command handler (`uart`). The last block
# below is those entries, in registration order.
INDIRECT_EDGES: dict[str, tuple[str, ...]] = {
    "emit_light_event": ("rf_link_light_event_callback",),
    "on_mesh_pair_proc": ("mesh_pair_proc_hook",),
    "on_light_data": ("rf_link_data_callback",),
    "on_light_response": ("rf_link_response_callback",),
    "on_uart_mesh_rx": ("uart_mesh_rx_hook",),
    "notify_status_update": ("ll_device_status_update",),
    "dispatch_ota_write": ("on_gatt_ota_packet",),
    "dispatch_ota_snapshot": ("ota_status_snapshot_impl",),
    "dispatch_ota_terminate": ("ota_request_termination_impl",),
    "dispatch_ota_tick": ("ota_tick_timeout_1s_impl",),
    "radio_stage_pkt": ("stage_ota_packet",),
    "radio_read_pkt": ("read_staged_ota_packet",),
    "radio_terminate_tx": ("ota_send_terminate_hook",),
    "radio_begin_gatt_ota": ("ota_begin_gatt_hook",),
    "dispatch_mesh_light_event": ("rf_link_light_event_callback",),
    "publish_light_status": ("tlsr8266_mesh::light::device_status_update",),
    "dispatch_uart_mesh_rx": ("uart_mesh_rx_hook",),
    "dispatch_uart_cmd": ("uart_command_hook",),
    "uart_tx_ops_push": ("enqueue_uart_tx_hook",),
    "uart_tx_ops_dropped": ("dropped_uart_tx_hook",),
    "dispatch_log_async": ("log_async_tx_hook",),
}

# Strict-only model of the five real callback slots in the GATT attribute table.
# This is deliberately separate from INDIRECT_EDGES: normal/make stack-check
# remains the staged 21-edge legacy graph, while --strict exposes the complete
# GATT-inclusive path and will fail if that path violates the RAM margin.
STRICT_GATT_EDGES: dict[str, tuple[str, ...]] = {
    "l2cap_att_handler": (
        "mesh_status_write",
        "rf_link_slave_data_write",
        "dispatch_ota_write",
        "pair_write",
        "pair_read",
    ),
}
# The GATT table callback dispatches occur in these two narrow ATT helpers in
# the current firmware.  They are known GATT dispatch roots, not project callback
# targets; strict classification does not use them to hide arbitrary callbacks.
STRICT_GATT_HELPER_PATTERNS: tuple[str, ...] = (
    "handle_write_request_or_command",
    "handle_read_request",
)

# Narrow runtime allowlist used only by strict indirect classification.
# Treatment semantics:
#   leaf       - the runtime branch/target is stack-transparent or has no project frame;
#   normal_call - the indirect branch is a normal call and the function frame is charged;
#   isolated  - the runtime preamble owns an isolated/IRQ frame with a fixed treatment.
CORE_RUNTIME_ALLOWLIST: dict[str, str] = {
    "__pender": "leaf",
    "__irq": "isolated",
    "embassy_executor::": "normal_call",
    "embassy_sync::": "normal_call",
    "embassy_time::": "leaf",
    "embassy_time_queue_utils::": "leaf",
    "critical_section::": "normal_call",
    "tlsr8266_hal::runtime::executor::Executor": "normal_call",
    "tlsr8266_hal::runtime::executor::Executor::run": "normal_call",
    "tlsr8266_hal::runtime::executor::Executor::poll": "normal_call",
    "core::fmt::": "normal_call",
    "core::cell::": "normal_call",
    "core::panicking::": "normal_call",
    "compiler_builtins::": "leaf",
    "rust_begin_unwind": "normal_call",
    "rust_eh_personality": "normal_call",
    "__aeabi_": "leaf",
}
# Narrow project-indirect seams that are known live callers, but are not core or
# Embassy runtime callbacks. These use stable namespaced export names rather than
# Rust-v0 mangled hashes or crate disambiguators.
STRICT_PROJECT_INDIRECT_ALLOWLIST: dict[str, str] = {
    "tlsr8266_enqueue_mesh_rx": "normal_call",
    "tlsr8266_enqueue_mesh_tx": "normal_call",
    "tlsr8266_handle_system_interrupts": "normal_call",
}


@dataclass
class Func:
    addr: int
    frame: int = 0
    calls: list[tuple[str, bool]] = field(default_factory=list)  # (name, is_tail)
    size: int = 0
    indirect: bool = False  # contains an indirect branch (bx rN / blx/tjex)
    section: str = ""  # .ram_code, .text, .rodata, or another ELF section
    indirect_insts: list[tuple[int, str, str]] = field(default_factory=list)

    @property
    def indirect_sites(self) -> list[tuple[int, str, str]]:
        """Backward-friendly name for sites (address, mnemonic, operand)."""
        return self.indirect_insts


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

        indirect = INDIRECT_RE.search(line)
        if indirect:
            current.indirect = True
            addr_match = INSTRUCTION_ADDRESS_RE.match(line)
            inst_addr = int(addr_match.group(1), 16) if addr_match else current.addr

            mnemonic_match = re.search(r"\b(tjex|bx|blx)\b", line, re.IGNORECASE)
            mnemonic = mnemonic_match.group(1).lower() if mnemonic_match else "indirect"
            operand = ""
            if mnemonic_match:
                remainder = line[mnemonic_match.end():].strip()
                if remainder:
                    operand = remainder.split()[0].strip("(),")

            current.indirect_insts.append((inst_addr, mnemonic, operand))

    if current is not None and name is not None:
        current.frame = max_off
        funcs[name] = current

    return funcs


class IndirectEdgeError(Exception):
    """A configured indirect-edge entry did not resolve to one function."""


class StrictIndirectBranchError(Exception):
    """A strict-mode project indirect branch was unknown or ambiguous."""


class RamReachabilityError(Exception):
    """A .ram_code caller reaches .text/.rodata during its reachable graph."""


def _resolve_one(
    funcs: dict[str, Func],
    pattern: str,
    role: str,
    key: str,
    table_name: str = "INDIRECT_EDGES",
) -> str:
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
            f"{table_name}[{key!r}]: {role} {pattern!r} matches no parsed "
            f"function (of {len(funcs)}); the table is stale - was the "
            f"dispatcher inlined or renamed?"
        )
    candidates: list[str] = []
    for name in matches:
        func = funcs[name]
        if role == "caller" and func.indirect_insts:
            sites = ", ".join(
                f"0x{inst_addr:08x}" for inst_addr, _mnemonic, _operand in func.indirect_insts
            )
            candidates.append(f"{name}@{sites}")
        elif role == "caller" and func.indirect:
            candidates.append(f"{name}@0x{func.addr:08x}")
        else:
            candidates.append(name)
    raise IndirectEdgeError(
        f"{table_name}[{key!r}]: {role} {pattern!r} is ambiguous, it matches "
        f"{len(matches)} functions: {candidates}"
    )


def _resolve_indirect_table(
    funcs: dict[str, Func],
    table: dict[str, tuple[str, ...]],
    table_name: str = "INDIRECT_EDGES",
) -> list[tuple[str, str]]:
    """Resolve one indirect-edge table without mutating the call graph."""
    resolved: list[tuple[str, str]] = []
    for caller_pattern, callee_patterns in table.items():
        caller = _resolve_one(
            funcs,
            caller_pattern,
            "caller",
            caller_pattern,
            table_name=table_name,
        )
        for callee_pattern in callee_patterns:
            callee = _resolve_one(
                funcs,
                callee_pattern,
                "callee",
                caller_pattern,
                table_name=table_name,
            )
            resolved.append((caller, callee))
    return resolved


def _append_indirect_edges(
    funcs: dict[str, Func], resolved: list[tuple[str, str]]
) -> None:
    for caller, callee in resolved:
        funcs[caller].calls.append((callee, False))


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
    table_name = "INDIRECT_EDGES"
    resolved = _resolve_indirect_table(funcs, table, table_name)

    # Mutate only once every entry resolved, so a rejected table leaves the call
    # graph untouched and the failure is reported instead of half-applied.
    _append_indirect_edges(funcs, resolved)
    return resolved


def apply_strict_indirect_edges(
    funcs: dict[str, Func],
) -> tuple[list[tuple[str, str]], list[tuple[str, str]]]:
    """Resolve and append legacy plus strict GATT edges in one transaction.

    Strict mode keeps the 21 legacy mappings and then applies the separate GATT
    table.  Resolution is completed for both tables before any graph mutation,
    so stale or ambiguous GATT roots leave the graph untouched on failure.
    """
    legacy_resolved = _resolve_indirect_table(
        funcs, INDIRECT_EDGES, "INDIRECT_EDGES"
    )
    gatt_resolved = _resolve_indirect_table(
        funcs, STRICT_GATT_EDGES, "STRICT_GATT_EDGES"
    )

    _append_indirect_edges(funcs, legacy_resolved + gatt_resolved)
    return legacy_resolved, gatt_resolved


def runtime_allowlist_treatment(name: str) -> str | None:
    """Return narrow core/Embassy treatment, or None for project callbacks."""
    # With llvm-objdump --demangle, runtime symbols are source-qualified and
    # contain no Rust-v0 crate hash. Match only stable module/prefix families;
    # project callbacks remain outside this table and fail closed unless modelled.
    for pattern, treatment in CORE_RUNTIME_ALLOWLIST.items():
        if pattern in name:
            return treatment
    return None


def strict_project_indirect_treatment(name: str) -> str | None:
    """Return treatment for a narrow, source-reviewed project indirect seam."""
    for pattern, treatment in STRICT_PROJECT_INDIRECT_ALLOWLIST.items():
        if pattern in name:
            return treatment
    return None


def strict_indirect_treatment(name: str) -> str | None:
    """Combine narrow core/Embassy and reviewed project indirect treatments."""
    return runtime_allowlist_treatment(name) or strict_project_indirect_treatment(name)


def collect_strict_indirect_failures(
    funcs: dict[str, Func],
    resolved_legacy: list[tuple[str, str]] = (),
    resolved_gatt: list[tuple[str, str]] = (),
) -> tuple[list[str], list[str], list[str]]:
    """Return (failures, allowed-runtime names, classified caller names)."""
    classified = {
        caller for caller, _callee in list(resolved_legacy) + list(resolved_gatt)
    }
    classified.update(
        callee for _caller, callee in list(resolved_legacy) + list(resolved_gatt)
    )
    for name in funcs:
        if any(pattern in name for pattern in STRICT_GATT_HELPER_PATTERNS):
            classified.add(name)
    failures: list[str] = []
    allowed: list[str] = []

    for name, func in sorted(funcs.items(), key=lambda item: item[0]):
        if not func.indirect:
            continue
        if name in classified:
            continue

        treatment = strict_indirect_treatment(name)
        if treatment:
            allowed.append(name)
            continue

        sites = func.indirect_insts
        if not sites:
            sites = [(func.addr, "unknown", "")]
        for inst_addr, mnemonic, operand in sites:
            operand_text = f" {operand}" if operand else ""
            failures.append(
                f"strict unknown indirect project branch: full symbol {name!r}, "
                f"instruction 0x{inst_addr:08x} ({mnemonic}{operand_text}); "
                "it is neither a legacy/GATT modeled edge nor a narrow "
                "core/Embassy runtime allowlist entry"
            )

    return failures, allowed, sorted(classified)


def validate_strict_indirect_branches(
    funcs: dict[str, Func],
    resolved_legacy: list[tuple[str, str]] = (),
    resolved_gatt: list[tuple[str, str]] = (),
) -> list[str]:
    """Fail closed on project indirect branches, returning allowed runtime names."""
    failures, allowed, _classified = collect_strict_indirect_failures(
        funcs, resolved_legacy, resolved_gatt
    )
    if failures:
        raise StrictIndirectBranchError("\n".join(f"FAIL: {error}" for error in failures))
    return allowed


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
    """Resolve an LLVM tool: explicit env var, then the repo builds in order, then PATH.

    ``../llvm/build-native/bin/`` is probed before ``../llvm/build/bin/`` (same
    order as ``build_firmware.py``): the ``build/`` tree on this host is a
    generic LLVM without the TC32 backend.
    """
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
        "../llvm/build-native/bin/llvm-objdump",
        "../llvm/build/bin/llvm-objdump",
    )
    try:
        proc = subprocess.run(
            [tool, "-d", "--demangle", "--mattr=+tc32", elf],
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
            or "Executor::run" in name
            or "Executor::poll" in name
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
        "../llvm/build-native/bin/llvm-nm",
        "../llvm/build/bin/llvm-nm",
    )
    try:
        proc = subprocess.run(
            [tool, "--demangle", elf], capture_output=True, text=True, check=False
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


def _parse_hex(value: str) -> int:
    return int(value.strip(), 16)


def parse_elf_sections(readelf_text: str) -> dict[str, tuple[int, int]]:
    """Parse ELF section names, VMA starts and byte sizes from readelf text.

    The format accepted here is enough for llvm-readelf/ GNU readelf ``-S --wide``
    output and for a small synthetic ``objdump -h``/section fixture.  It returns
    ``{section_name: (start_address, size)}`` so function addresses can be
    classified without adding a dependency.
    """
    sections: dict[str, tuple[int, int]] = {}

    readelf_re = re.compile(
        r"^\s*\[\s*\d+\]\s+(\.[^\s]+)\s+\S+\s+"
        r"([0-9a-fA-F]+)\s+[0-9a-fA-F]+\s+([0-9a-fA-F]+)"
    )
    objdump_re = re.compile(
        r"^\s*\d+\s+(\.[^\s]+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)"
    )
    simple_re = re.compile(
        r"^\s*(\.[A-Za-z0-9_.-]+)\s+(?:[^\s]+\s+)?"
        r"(?:0x)?([0-9a-fA-F]+)\s+(?:0x)?([0-9a-fA-F]+)\s*$"
    )

    for line in readelf_text.splitlines():
        match = readelf_re.match(line)
        if match:
            name = match.group(1)
            sections[name] = (_parse_hex(match.group(2)), _parse_hex(match.group(3)))
            continue

        match = objdump_re.match(line)
        if match:
            name = match.group(1)
            sections[name] = (_parse_hex(match.group(3)), _parse_hex(match.group(2)))
            continue

        match = simple_re.match(line)
        if match:
            name = match.group(1)
            sections[name] = (_parse_hex(match.group(2)), _parse_hex(match.group(3)))

    return sections


def read_elf_sections(elf: str) -> dict[str, tuple[int, int]]:
    """Read linked ELF section ranges, using llvm-readelf when available."""
    tool = _llvm_tool(
        "llvm-readelf",
        "LLVM_READELF",
        "../llvm/build-native/bin/llvm-readelf",
        "../llvm/build/bin/llvm-readelf",
        "readelf",
    )
    try:
        proc = subprocess.run(
            [tool, "-S", "--wide", elf],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        return {}
    if proc.returncode == 0:
        sections = parse_elf_sections(proc.stdout)
        if sections:
            return sections

    # llvm-objdump -h is a standard fallback and is sufficient for the sections
    # this gate needs.  It also keeps synthetic and host tests independent of
    # whether llvm-readelf happens to be on PATH.
    objdump_tool = _llvm_tool(
        "llvm-objdump",
        "LLVM_OBJDUMP",
        "../llvm/build-native/bin/llvm-objdump",
        "../llvm/build/bin/llvm-objdump",
    )
    try:
        proc = subprocess.run(
            [objdump_tool, "-h", elf],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        return {}
    if proc.returncode != 0:
        return {}
    return parse_elf_sections(proc.stdout)


def _normalize_section_map(
    sections: dict[str, tuple[int, int] | dict[str, int] | int] | None,
) -> dict[str, tuple[int, int]]:
    """Normalize section fixtures and parsed ELF ranges to (address, size)."""
    normalized: dict[str, tuple[int, int]] = {}
    if not sections:
        return normalized
    for name, value in sections.items():
        section_name = name if name.startswith(".") else f".{name}"
        if isinstance(value, tuple) and len(value) == 2:
            normalized[section_name] = (int(value[0]), int(value[1]))
        elif isinstance(value, dict):
            address = value.get(
                "address", value.get("vma", value.get("addr", value.get("start", 0)))
            )
            size = value.get("size", 0)
            normalized[section_name] = (int(address), int(size))
        elif isinstance(value, int):
            normalized[section_name] = (0, int(value))
    return normalized


def classify_function_sections(
    funcs: dict[str, Func],
    sections: dict[str, tuple[int, int] | dict[str, int] | int] | None,
) -> None:
    """Assign every parsed function to the smallest ELF section containing it."""
    normalized = _normalize_section_map(sections)
    for func in funcs.values():
        matches: list[tuple[str, int, int]] = []
        for name, (start, size) in normalized.items():
            if size <= 0:
                continue
            if start <= func.addr < start + size:
                matches.append((name, start, size))
        if matches:
            func.section = min(matches, key=lambda item: item[2])[0]


BOOT_OTA_ROOT_PATTERN = "handle_ota_update"
RAM_REACHABILITY_ROOT_PATTERN = BOOT_OTA_ROOT_PATTERN


def _is_ram_section(name: str) -> bool:
    return name == ".ram_code" or name.startswith(".ram_code.")


def _is_flash_or_rodata_section(name: str) -> bool:
    return (
        name == ".text"
        or name.startswith(".text.")
        or name == ".rodata"
        or name.startswith(".rodata.")
        or name == ".init"
        or name.startswith(".init.")
    )


def check_ram_reachability(
    funcs: dict[str, Func],
    sections: dict[str, tuple[int, int] | dict[str, int] | int] | None = None,
    symbols: dict[str, int] | None = None,
    root_pattern: str | tuple[str, ...] | list[str] = RAM_REACHABILITY_ROOT_PATTERN,
) -> list[tuple[str, str, str]]:
    """Return flash/rodata targets reached by RAM callers below a boot OTA root.

    The root pattern is a regex-friendly substring pattern.  String and sequence
    forms both support mangled ELF names (``handle_ota_update`` remains contained
    in the real Rust-v0 symbol).  Only the rooted call graph is walked; unrelated
    IRQ/main RAM callers are deliberately outside this boot-erase safety check.
    """
    # Backward-friendly positional form: a string/sequence in the legacy
    # ``sections`` slot is accepted as the root pattern.
    if isinstance(sections, (str, tuple, list)):
        if root_pattern == RAM_REACHABILITY_ROOT_PATTERN:
            root_pattern = sections
        sections = None

    if sections:
        classify_function_sections(funcs, sections)

    normalized = _normalize_section_map(sections)
    patterns = (
        (root_pattern,)
        if isinstance(root_pattern, str)
        else tuple(str(pattern) for pattern in root_pattern)
    )

    roots: set[str] = set()
    for name in funcs:
        for pattern in patterns:
            if not pattern:
                continue
            try:
                matched = re.search(pattern, name) is not None
            except re.error:
                matched = pattern in name
            if matched:
                roots.add(name)
                break

    visited: set[str] = set()
    violations: list[tuple[str, str, str]] = []

    def callee_section(name: str, callee: Func | None) -> str:
        section = callee.section if callee is not None else ""
        if section:
            return section
        if symbols is not None and name in symbols:
            callee_addr = symbols[name]
            matches: list[tuple[str, int, int]] = []
            for section_name, (start, size) in normalized.items():
                if size > 0 and start <= callee_addr < start + size:
                    matches.append((section_name, start, size))
            if matches:
                return min(matches, key=lambda item: item[2])[0]
        return ""

    def is_boot_reboot_leaf(name: str) -> bool:
        return (
            name == "reboot"
            or name == "light_sw_reboot"
            or "light_sw_reboot" in name
            or name == "mcu_trigger_reboot"
            or "mcu_trigger_reboot" in name
        )

    def visit(caller_name: str) -> None:
        if caller_name in visited:
            return
        visited.add(caller_name)
        caller = funcs.get(caller_name)
        if caller is None or not _is_ram_section(caller.section):
            return

        for callee_name, _is_tail in caller.calls:
            callee = funcs.get(callee_name)
            section = callee_section(callee_name, callee)

            if section and _is_flash_or_rodata_section(section):
                if section == ".text" and is_boot_reboot_leaf(callee_name):
                    continue
                violation = (caller_name, callee_name, section)
                if violation not in violations:
                    violations.append(violation)

            if callee is not None and _is_ram_section(callee.section):
                visit(callee_name)

    for root in sorted(roots):
        visit(root)

    return violations


def validate_ram_reachability(
    funcs: dict[str, Func],
    sections: dict[str, tuple[int, int] | dict[str, int] | int] | None = None,
    symbols: dict[str, int] | None = None,
    root_pattern: str | tuple[str, ...] | list[str] = RAM_REACHABILITY_ROOT_PATTERN,
) -> list[tuple[str, str, str]]:
    """Raise a fail-closed RAM reachability error if a rooted violation exists."""
    violations = check_ram_reachability(funcs, sections, symbols, root_pattern)
    if violations:
        messages = [
            f".ram_code caller {caller!r} reaches flash/readonly target "
            f"{callee!r} in {section}"
            for caller, callee, section in violations
        ]
        raise RamReachabilityError("\n".join(messages))
    return violations


def infer_bin_path(elf: str) -> str:
    if elf.endswith(".elf"):
        return elf[:-4] + ".bin"
    if elf.endswith(".bin"):
        return elf
    return elf + ".bin"


def read_image_bytes(elf: str) -> tuple[int, int]:
    elf_bytes = 0
    bin_bytes = 0
    try:
        elf_bytes = os.path.getsize(elf)
    except OSError:
        pass
    try:
        bin_bytes = os.path.getsize(infer_bin_path(elf))
    except OSError:
        pass
    return elf_bytes, bin_bytes


def build_baseline(
    elf: str,
    funcs: dict[str, Func],
    syms: dict[str, int],
    main_max: int,
    irq_max: int,
    sections: dict[str, tuple[int, int] | dict[str, int] | int] | None,
    legacy_edges: list[tuple[str, str]],
    gatt_edges: list[tuple[str, str]] | None = None,
    runtime_allowed: list[str] | None = None,
    strict: bool = False,
) -> dict[str, int | str | dict[str, int]]:
    """Build the optional machine-readable stack/image baseline record."""
    ram_top = syms["__RAM_START_ADDR"] + syms["__RAM_SIZE_MAX"]
    free_main = ram_top - syms["_end_bss_"]
    irq_budget = syms["IRQ_STK_SIZE"]

    elf_bytes, bin_bytes = read_image_bytes(elf)
    normalized = _normalize_section_map(sections)
    section_sizes = {
        name: size
        for name, (_start, size) in normalized.items()
    }
    for name in (".text", ".rodata", ".data", ".bss", ".ram_code"):
        section_sizes.setdefault(name, 0)

    gatt_edges = gatt_edges or []
    runtime_allowed = runtime_allowed or []
    allowed_count = len(legacy_edges) + len(gatt_edges) + len(runtime_allowed)

    return {
        "elf_bytes": elf_bytes,
        "bin_bytes": bin_bytes,
        "sections": section_sizes,
        "free_main_sram": free_main,
        "main_max": main_max,
        "irq_max": irq_max,
        "main_margin": free_main - main_max,
        "irq_margin": irq_budget - irq_max,
        "parsed_functions": len(funcs),
        "indirect_functions": sum(1 for func in funcs.values() if func.indirect),
        "allowed_indirect": allowed_count,
        "legacy_edges_resolved": len(legacy_edges),
        "gatt_edges_resolved": len(gatt_edges),
        "runtime_allowed_indirect": len(runtime_allowed),
        "mode": "strict" if strict else "legacy",
        "strict": strict,
    }


def write_baseline_json(destination: str, baseline: dict) -> None:
    text = json.dumps(baseline, indent=2, sort_keys=True)
    if destination == "-":
        print(text)
        return
    parent = os.path.dirname(os.path.abspath(destination))
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(destination, "w", encoding="utf-8") as handle:
        handle.write(text)
        handle.write("\n")
    print(f"baseline JSON written to {destination}")


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
    ap.add_argument(
        "--strict",
        action="store_true",
        help="fail closed on unknown project indirect branches and model strict GATT callbacks",
    )
    ap.add_argument(
        "--baseline-json",
        "--baseline",
        dest="baseline_json",
        nargs="?",
        const="-",
        metavar="PATH",
        default=None,
        help="write a JSON stack/image baseline (use '-' or omit PATH for stdout)",
    )
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    funcs = parse_functions(objdump(args.elf))
    if not funcs:
        sys.exit("no functions parsed; is the ELF built?")

    print(f"parsed {len(funcs)} functions from {args.elf}")
    indirect = sum(1 for f in funcs.values() if f.indirect)
    print(f"functions with indirect (bx/blx/tjex) branches: {indirect}")

    legacy_edges: list[tuple[str, str]] = []
    gatt_edges: list[tuple[str, str]] = []
    runtime_allowed: list[str] = []

    # Must precede every root/depth query: an unmapped dispatcher would score
    # its callback as 0 B and silently understate the margin.
    if args.strict:
        print("strict GATT model targets (5):")
        print("  mesh_status_write")
        print("  rf_link_slave_data_write")
        print("  dispatch_ota_write")
        print("  pair_write")
        print("  pair_read")
        try:
            legacy_edges, gatt_edges = apply_strict_indirect_edges(funcs)
        except IndirectEdgeError as exc:
            print(f"\nFAIL: {exc}")
            return 1

        print(f"legacy indirect dispatch edges ({len(legacy_edges)}):")
        for caller, callee in legacy_edges:
            print(f"  {caller} -> {callee}")
        print(f"strict GATT dispatch edges ({len(gatt_edges)}):")
        for caller, callee in gatt_edges:
            print(f"  {caller} -> {callee}")

        try:
            runtime_allowed = validate_strict_indirect_branches(
                funcs, legacy_edges, gatt_edges
            )
        except StrictIndirectBranchError as exc:
            print(f"\nFAIL: {exc}")
            return 1

        if runtime_allowed:
            print("strict known/core and Embassy indirect allowlist:")
            for name in runtime_allowed:
                treatment = strict_indirect_treatment(name) or "unknown"
                print(f"  {name} -> {treatment} (modeled stack treatment)")
    else:
        try:
            legacy_edges = apply_indirect_edges(funcs)
        except IndirectEdgeError as exc:
            print(f"\nFAIL: {exc}")
            return 1
        print(f"indirect dispatch edges ({len(legacy_edges)}):")
        for caller, callee in legacy_edges:
            print(f"  {caller} -> {callee}")

    sections = read_elf_sections(args.elf)
    classify_function_sections(funcs, sections)

    # Read RAM/linker symbols before the depth report so a RAM-resident caller
    # cannot be checked against missing linker state or silently score a flash
    # target as a normal caller.
    syms = read_symbols(args.elf)
    needed = ["__RAM_START_ADDR", "__RAM_SIZE_MAX", "_end_bss_", "IRQ_STK_SIZE"]
    missing = [n for n in needed if n not in syms]
    if missing:
        print(f"\nGATE ERROR: missing linker symbols: {missing}")
        return 2

    try:
        ram_violations = validate_ram_reachability(
            funcs,
            sections,
            syms,
            root_pattern=BOOT_OTA_ROOT_PATTERN,
        )
    except RamReachabilityError as exc:
        print(f"\nFAIL: RAM reachability: {exc}")
        return 1
    if sections:
        print(
            "RAM reachability: ELF sections classified "
            f"({len(sections)} sections, {len(ram_violations)} violations)"
        )

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

    if args.strict and gatt_edges:
        print("\nSTRICT GATT-inclusive path model:")
        print("  modeled targets: mesh_status_write, rf_link_slave_data_write, "
              "dispatch_ota_write, pair_write, pair_read")
        if main_max == 1684 or irq_max >= 1684:
            print("  failing GATT-inclusive path fixture: 1,684 B-style (1684 B)")

    print(f"\ntop {args.top} functions by frame size:")
    for f in sorted(funcs.values(), key=lambda x: x.frame, reverse=True)[: args.top]:
        print(f"  {f.frame:5d} B  @0x{f.addr:06x}")

    # ---- budget gate ----------------------------------------------------
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
    if args.strict and gatt_edges:
        print("  FAIL: strict GATT-inclusive path violates the fail-close policy")
        ok = False
    if main_margin < args.min_margin:
        print("  FAIL: main stack margin below minimum")
        ok = False
    if irq_margin < args.min_margin:
        print("  FAIL: IRQ stack margin below minimum")
        ok = False

    print("  PASS" if ok else "  FAIL")

    if args.baseline_json is not None:
        baseline = build_baseline(
            args.elf,
            funcs,
            syms,
            main_max,
            irq_max,
            sections,
            legacy_edges,
            gatt_edges,
            runtime_allowed,
            args.strict,
        )
        write_baseline_json(args.baseline_json, baseline)

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
