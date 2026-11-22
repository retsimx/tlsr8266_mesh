"""Unit tests for the static stack-depth analyzer and its gate.

Run from the repository root::

    python3 -m unittest discover -s sdk/tests -v

The suite is pure stdlib and never invokes LLVM, the linker, or a real ELF:
``parse_functions`` is a pure ``str -> dict`` transformer, so each test feeds it
synthetic ``llvm-objdump`` text. The only two I/O boundaries (``objdump`` and
``read_symbols``) are patched for the ``main()`` tests.
"""

from __future__ import annotations

import contextlib
import io
import sys
import unittest
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "sdk"))

import stack_analysis  # noqa: E402  (sys.path set up above)

# Linker symbols that make the RAM gate's arithmetic exact: 4096 B of free main
# stack above _end_bss_ and the firmware's real 1792 B (0x700) IRQ stack.
SYMS = {
    "__RAM_START_ADDR": 0x808000,
    "__RAM_SIZE_MAX": 16384,
    "_end_bss_": 0x808000 + 16384 - 4096,
    "IRQ_STK_SIZE": 1792,
}


def _fn(
    name: str,
    frame: int,
    calls: tuple[tuple[str, bool], ...] = (),
    *,
    indirect: bool = False,
) -> str:
    """Render one synthetic disassembly block with an exact ``frame``-byte frame.

    A dispatcher is written as an indirect branch with *no* ``bl`` edge to its
    real callee, which is what the linked firmware looks like.
    """
    lines = [f"0000 <{name}>:", "\t0: 6580  \tpush\t{r7, lr}"]
    rest = frame - 8
    if rest > 0:
        lines.append(f"\t2: 6084  \tsub\tsp, #0x{rest:x}")
    for callee, is_tail in calls:
        # `b <sym>` is a tail call, `bl <sym>` a normal one; both spellings are
        # what the TC32 disassembler actually emits.
        mnemonic = "b" if is_tail else "bl"
        lines.append(f"\t4: 9000  \t{mnemonic}\t0x0000 <{callee}>")
    if indirect:
        lines.append("\t8: 4708  \tbx\tr1")
    lines.append("\t6: 6d80  \tpop\t{r7, pc}")
    return "\n".join(lines)


def _text(*blocks: str) -> str:
    return "\n".join(blocks) + "\n"


# A miniature version of the real call graph: a root reaches the light-data
# dispatcher directly, the dispatcher reaches the app callback only through a
# `bx rN`, and that callback reaches the light-event dispatcher, which in turn
# reaches the app's light-event handler only through another `bx rN`. The two
# `mesh_pair_proc_*` lookalikes are present because the table's
# ``mesh_pair_proc_hook`` entry must not be confused with them.
FIRMWARE_SHAPED = _text(
    _fn("main_entrypoint", 40, (("rf_drv_826624rf_link_slave_data_write", False),)),
    _fn(
        "rf_drv_826624rf_link_slave_data_write",
        184,
        (("callbacks13on_light_data", False),),
    ),
    _fn("callbacks13on_light_data", 24, indirect=True),
    _fn(
        "main_light21rf_link_data_callback",
        128,
        (("rf_drv16emit_light_event", False),),
    ),
    _fn("rf_drv16emit_light_event", 8, indirect=True),
    _fn("main_light28rf_link_light_event_callback", 200),
    _fn("callbacks17on_mesh_pair_proc", 24, indirect=True),
    _fn(
        "app19mesh_pair_proc_hook",
        8,
        (("MeshManager14mesh_pair_proc", False),),
    ),
    _fn("MeshManager14mesh_pair_proc", 180),
    _fn("MeshManager21mesh_pair_proc_effect", 60),
    _fn("callbacks17on_light_response", 24, indirect=True),
    _fn("main_light25rf_link_response_callback", 96),
    _fn("callbacks15on_uart_mesh_rx", 24, indirect=True),
    _fn("uart_manager16light_mesh_rx_cb", 72),
    _fn("irq_handler", 24, (("handle_system_interrupts", False),)),
    _fn(
        "handle_system_interrupts",
        272,
        (("rf_drv16emit_light_event", False),),
    ),
)


def _parse(text: str = FIRMWARE_SHAPED) -> dict[str, stack_analysis.Func]:
    funcs = stack_analysis.parse_functions(text)
    assert funcs, "synthetic disassembly parsed to nothing"
    return funcs


def _run_main(text: str, argv: list[str] | None = None) -> tuple[int, str]:
    """Run ``main`` against synthetic disassembly and capture its stdout."""
    args = ["--elf", "synthetic.elf", "--top", "3"] if argv is None else argv
    buf = io.StringIO()
    with (
        mock.patch.object(stack_analysis, "objdump", return_value=text),
        mock.patch.object(stack_analysis, "read_symbols", return_value=SYMS),
        contextlib.redirect_stdout(buf),
    ):
        code = stack_analysis.main(args)
    return code, buf.getvalue()


class TestParseFunctions(unittest.TestCase):
    def test_parses_frames_and_direct_edges(self) -> None:
        funcs = _parse()
        self.assertEqual(funcs["rf_drv_826624rf_link_slave_data_write"].frame, 184)
        self.assertIn(
            ("callbacks13on_light_data", False),
            funcs["rf_drv_826624rf_link_slave_data_write"].calls,
        )
        self.assertEqual(funcs["irq_handler"].frame, 24)

    def test_indirect_dispatch_produces_no_edge(self) -> None:
        """The whole reason INDIRECT_EDGES exists: `bx rN` is a dead end."""
        funcs = _parse()
        dispatcher = funcs["callbacks13on_light_data"]
        self.assertTrue(dispatcher.indirect)
        self.assertEqual(dispatcher.calls, [])

    def test_tail_call_is_marked_tail(self) -> None:
        funcs = stack_analysis.parse_functions(
            _text(
                _fn("caller", 64, (("__aeabi_memcpy", True),)),
                _fn("__aeabi_memcpy", 8),
            )
        )
        self.assertEqual(funcs["caller"].calls, [("__aeabi_memcpy", True)])


class TestApplyIndirectEdges(unittest.TestCase):
    def test_adds_normal_call_edge(self) -> None:
        funcs = _parse()
        resolved = stack_analysis.apply_indirect_edges(funcs)
        self.assertIn(
            ("main_light21rf_link_data_callback", False),
            funcs["callbacks13on_light_data"].calls,
        )
        # An indirect dispatch pushes a return address: never a tail call.
        for caller, callee in resolved:
            self.assertIn((callee, False), funcs[caller].calls)

    def test_resolved_pairs_name_the_real_symbols(self) -> None:
        funcs = _parse()
        resolved = stack_analysis.apply_indirect_edges(funcs)
        self.assertEqual(
            resolved,
            [
                (
                    "rf_drv16emit_light_event",
                    "main_light28rf_link_light_event_callback",
                ),
                ("callbacks17on_mesh_pair_proc", "app19mesh_pair_proc_hook"),
                ("callbacks13on_light_data", "main_light21rf_link_data_callback"),
                (
                    "callbacks17on_light_response",
                    "main_light25rf_link_response_callback",
                ),
                ("callbacks15on_uart_mesh_rx", "uart_manager16light_mesh_rx_cb"),
            ],
        )
        # The hook is the intermediate frame; the manager method keeps its own
        # direct `bl` edge and must not be wired twice.
        self.assertEqual(
            funcs["app19mesh_pair_proc_hook"].calls,
            [("MeshManager14mesh_pair_proc", False)],
        )
        self.assertEqual(funcs["MeshManager14mesh_pair_proc"].calls, [])

    def test_caller_with_zero_matches_is_rejected(self) -> None:
        funcs = _parse()
        edges = {"no_such_dispatcher": ("main_light21rf_link_data_callback",)}
        with self.assertRaises(stack_analysis.IndirectEdgeError) as ctx:
            stack_analysis.apply_indirect_edges(funcs, edges)
        message = str(ctx.exception)
        self.assertIn("no_such_dispatcher", message)
        self.assertIn("matches no parsed function", message)
        self.assertIn("caller", message)

    def test_callee_with_zero_matches_is_rejected(self) -> None:
        funcs = _parse()
        edges = {"on_light_data": ("no_such_callback",)}
        with self.assertRaises(stack_analysis.IndirectEdgeError) as ctx:
            stack_analysis.apply_indirect_edges(funcs, edges)
        message = str(ctx.exception)
        self.assertIn("on_light_data", message)
        self.assertIn("no_such_callback", message)
        self.assertIn("matches no parsed function", message)
        self.assertIn("callee", message)

    def test_ambiguous_substring_is_rejected(self) -> None:
        """`mesh_pair_proc` alone matches the hook, the method and the effect."""
        funcs = _parse()
        edges = {"on_mesh_pair_proc": ("mesh_pair_proc",)}
        with self.assertRaises(stack_analysis.IndirectEdgeError) as ctx:
            stack_analysis.apply_indirect_edges(funcs, edges)
        message = str(ctx.exception)
        self.assertIn("ambiguous", message)
        # Every candidate must be listed so the fix is obvious.
        self.assertIn("app19mesh_pair_proc_hook", message)
        self.assertIn("MeshManager14mesh_pair_proc", message)
        self.assertIn("MeshManager21mesh_pair_proc_effect", message)

    def test_hook_substring_disambiguates(self) -> None:
        funcs = _parse()
        resolved = stack_analysis.apply_indirect_edges(
            funcs, {"on_mesh_pair_proc": ("mesh_pair_proc_hook",)}
        )
        self.assertEqual(
            resolved, [("callbacks17on_mesh_pair_proc", "app19mesh_pair_proc_hook")]
        )

    def test_rejected_table_leaves_the_graph_untouched(self) -> None:
        funcs = _parse()
        before = {name: list(f.calls) for name, f in funcs.items()}
        edges = {
            "on_light_data": ("main_light21rf_link_data_callback",),
            "on_uart_mesh_rx": ("no_such_callback",),
        }
        with self.assertRaises(stack_analysis.IndirectEdgeError):
            stack_analysis.apply_indirect_edges(funcs, edges)
        self.assertEqual({name: f.calls for name, f in funcs.items()}, before)

    def test_table_covers_the_five_dispatchers(self) -> None:
        self.assertEqual(
            sorted(stack_analysis.INDIRECT_EDGES),
            [
                "emit_light_event",
                "on_light_data",
                "on_light_response",
                "on_mesh_pair_proc",
                "on_uart_mesh_rx",
            ],
        )
        for caller, callees in stack_analysis.INDIRECT_EDGES.items():
            self.assertEqual(len(callees), 1, caller)


class TestIndirectEdgesFeedDepth(unittest.TestCase):
    def test_depth_increases_once_edges_are_applied(self) -> None:
        funcs = _parse()
        before, _, _ = stack_analysis.longest_path(funcs, "main_entrypoint")
        stack_analysis.apply_indirect_edges(funcs)
        after, chain, cycles = stack_analysis.longest_path(
            funcs, "main_entrypoint"
        )
        # The dispatch is a normal call chain: the whole callback subtree
        # (128 + 8 + 200) that used to be scored as 0 B now counts.
        self.assertEqual(after - before, 336)
        self.assertEqual(cycles, [])
        self.assertIn("main_light28rf_link_light_event_callback", chain)

    def test_irq_chain_reaches_through_the_dispatcher(self) -> None:
        funcs = _parse()
        stack_analysis.apply_indirect_edges(funcs)
        depth, chain, _ = stack_analysis.longest_path(funcs, "irq_handler")
        self.assertEqual(
            depth, 24 + 272 + 8 + 200, "irq_handler -> isr -> emit -> callback"
        )
        self.assertIn("main_light28rf_link_light_event_callback", chain)

    def test_roots_no_longer_contain_indirect_only_callees(self) -> None:
        funcs = _parse()
        roots = set(stack_analysis.all_roots(funcs))
        self.assertIn("main_light21rf_link_data_callback", roots)
        self.assertIn("main_light28rf_link_light_event_callback", roots)
        stack_analysis.apply_indirect_edges(funcs)
        roots = set(stack_analysis.all_roots(funcs))
        self.assertNotIn("main_light21rf_link_data_callback", roots)
        self.assertNotIn("main_light28rf_link_light_event_callback", roots)
        self.assertNotIn("app19mesh_pair_proc_hook", roots)
        self.assertIn("main_entrypoint", roots)

    def test_introducing_a_cycle_fails_the_gate(self) -> None:
        funcs = stack_analysis.parse_functions(
            _text(
                _fn("caller", 32, (("callbacks13on_light_data", False),)),
                _fn("callbacks13on_light_data", 24, indirect=True),
                _fn(
                    "main_light21rf_link_data_callback",
                    64,
                    (("caller", False),),
                ),
            )
        )
        stack_analysis.apply_indirect_edges(
            funcs, {"on_light_data": ("rf_link_data_callback",)}
        )
        _, _, cycles = stack_analysis.longest_path(funcs, "caller")
        self.assertTrue(cycles, "a bad edge that closes a loop must be reported")


class TestMain(unittest.TestCase):
    def test_pass_reports_resolved_edges_and_margins(self) -> None:
        code, out = _run_main(FIRMWARE_SHAPED)
        self.assertEqual(code, 0, out)
        self.assertIn("indirect dispatch edges (5):", out)
        self.assertIn(
            "callbacks15on_uart_mesh_rx -> uart_manager16light_mesh_rx_cb", out
        )
        self.assertIn("main: free 4096 B - max 584 B = margin 3512 B", out)
        self.assertIn("irq:  size 1792 B - max 532 B = margin 1260 B", out)
        self.assertIn("PASS", out)

    def test_stale_caller_table_fails_closed(self) -> None:
        # Drop the UART dispatcher: on_uart_mesh_rx now matches nothing.
        text = FIRMWARE_SHAPED.replace(
            _fn("callbacks15on_uart_mesh_rx", 24, indirect=True) + "\n", ""
        )
        self.assertNotIn("callbacks15on_uart_mesh_rx", text)
        code, out = _run_main(text)
        self.assertEqual(code, 1, out)
        self.assertIn("FAIL: INDIRECT_EDGES['on_uart_mesh_rx']", out)
        self.assertIn("matches no parsed function", out)
        # Fail closed before any depth number is reported as trustworthy.
        self.assertNotIn("RAM GATE", out)

    def test_margin_below_minimum_still_fails(self) -> None:
        code, out = _run_main(
            FIRMWARE_SHAPED, ["--elf", "synthetic.elf", "--top", "0",
                              "--min-margin", "4000"]
        )
        self.assertEqual(code, 1, out)
        self.assertIn("FAIL: main stack margin below minimum", out)

    def test_parse_args_accepts_an_explicit_argv(self) -> None:
        args = stack_analysis.parse_args(
            ["--elf", "x.elf", "--top", "7", "--min-margin", "512",
             "--exclude-panic"]
        )
        self.assertEqual(
            (args.elf, args.top, args.min_margin, args.exclude_panic),
            ("x.elf", 7, 512, True),
        )
        self.assertFalse(stack_analysis.parse_args([]).exclude_panic)


if __name__ == "__main__":
    unittest.main()
