#!/usr/bin/env python3
"""Static stack-depth analyzer for the TLSR8266 (TC32) firmware ELF.

Walks the disassembly produced by ``tc32-elf-objdump -d``, derives each
function's stack frame from its prologue (``tpush {..}`` + ``tsub sp, #N``),
builds the directed call graph from ``tjl``/``tj``, and reports the deepest
root-to-leaf stack path for the main stack and the IRQ stack separately.

This exists to evidence the RAM budget: the main stack grows down from the top
of RAM into the free region above ``_end_bss_`` (see ``sdk/boot.link`` and
``sdk/cstartup_8266.S``), so the deepest reachable chain is the number the RAM
gate must fit. The IRQ stack (``irq_stk``, 2048 B) is separate.

Model and caveats (reported alongside the result):
  * Conservative frame model: a callee's whole frame is charged on top of the
    caller's whole frame, ignoring call-site sp offsets. This is an upper bound.
  * Tail calls (``tj <sym>`` to a known function) do not grow the stack; the
    callee replaces the caller's frame.
  * Interrupt nesting on the IRQ stack is not modelled (assumes non-nested);
    the ``__irq`` register-save preamble is added as a fixed constant.
  * The firmware contains no indirect ``tloadr pc/r15`` calls, so ``tjl`` edges
    capture the whole call graph. If that changes this script must be revisited.

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

# __irq saves r14, r0-r7, then r0-r5 before calling irq_handler on irq_stk.
# 1 + 8 + 6 registers * 4 bytes = 60 bytes.
IRQ_PREAMBLE_BYTES = 60

HEADER_RE = re.compile(r"^([0-9a-f]+) <(.*)>:\s*$")
CALL_RE = re.compile(r"\btjl\s+[0-9a-f]+\s+<([^>]+)>")
TAIL_RE = re.compile(r"\btj\s+[0-9a-f]+\s+<([^>]+)>")
PUSH_RE = re.compile(r"\btpush\s*\{([^}]*)\}")
POP_RE = re.compile(r"\btpop\s*\{([^}]*)\}")
SUB_RE = re.compile(r"\btsub(?:cs)?\s+sp,\s*#(\d+)")
ADD_RE = re.compile(r"\btadd(?:cs)?\s+sp,\s*#(\d+)")


@dataclass
class Func:
    addr: int
    frame: int = 0
    calls: list[tuple[str, bool]] = field(default_factory=list)  # (name, is_tail)
    size: int = 0
    indirect: bool = False  # contains a tjex (register/indirect branch)


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
            sp_off += int(sub.group(1))
            max_off = max(max_off, sp_off)

        pop = POP_RE.search(line)
        if pop:
            sp_off = max(0, sp_off - reg_count(pop.group(1)) * 4)

        add = ADD_RE.search(line)
        if add:
            sp_off = max(0, sp_off - int(add.group(1)))

        call = CALL_RE.search(line)
        if call:
            current.calls.append((call.group(1), False))
            continue

        tail = TAIL_RE.search(line)
        if tail:
            current.calls.append((tail.group(1), True))
            continue

        if re.search(r"\btjex\b", line):
            current.indirect = True

    if current is not None and name is not None:
        current.frame = max_off
        funcs[name] = current

    return funcs


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


def objdump(elf: str) -> str:
    from shutil import which

    tool = which("tc32-elf-objdump") or "./toolchain/tc32/bin/tc32-elf-objdump"
    proc = subprocess.run(
        [tool, "-d", elf], capture_output=True, text=True, check=False
    )
    if proc.returncode != 0:
        sys.exit(f"objdump failed: {proc.stderr.strip() or proc.returncode}")
    return proc.stdout


def report(label: str, total: int, chain: list[str], funcs: dict[str, Func]) -> None:
    print(f"\n{label}: max {total} bytes")
    print(f"  deepest chain ({len(chain)} frames):")
    for depth, name in enumerate(chain):
        frame = funcs[name].frame if name in funcs else 0
        print(f"    {frame:4d} B  {name}")


def all_roots(funcs: dict[str, Func]) -> list[str]:
    """Functions never targeted by a direct/tail call.

    Embassy invokes each task's ``poll`` through a function pointer (``tjex``),
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
    from shutil import which

    tool = which("tc32-elf-nm") or "./toolchain/tc32/bin/tc32-elf-nm"
    proc = subprocess.run([tool, elf], capture_output=True, text=True, check=False)
    syms: dict[str, int] = {}
    for line in proc.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 3:
            try:
                syms[parts[2]] = int(parts[0], 16)
            except ValueError:
                pass
    return syms


def main() -> int:
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
    args = ap.parse_args()

    funcs = parse_functions(objdump(args.elf))
    if not funcs:
        sys.exit("no functions parsed; is the ELF built?")

    print(f"parsed {len(funcs)} functions from {args.elf}")
    indirect = sum(1 for f in funcs.values() if f.indirect)
    print(f"functions with indirect (tjex) branches: {indirect}")

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
