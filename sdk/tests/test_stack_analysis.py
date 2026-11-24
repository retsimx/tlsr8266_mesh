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
#
# The light subsystem's status publish is modelled as the real firmware shape:
# ``rf_link_light_event_callback`` calls ``light::device_status_update`` directly,
# which calls the ``notify_status_update`` dispatcher, which reaches the SDK's
# ``ll_device_status_update`` only through a ``bx rN``.
#
# The OTA seam is modelled the same way: one synthetic pair per ``INDIRECT_EDGES``
# entry — the OTA write dispatcher 24 B / 40 B (the real shape: a GATT write
# lands in ``dispatch_ota_write``, which reaches the OTA packet handler), the
# snapshot hook 24 B / 16 B, the two flag-only hooks 24 B / 8 B, and the four
# radio providers 8 B / 16 B. ``callbacks21dispatch_ota_write`` is a *direct*
# callee of the GATT write entry, because that is the real firmware shape; every
# other OTA dispatcher stays a leaf, so the maxima do not move.
#
# The C-8 seams are the same shape and use the real symbol namespaces: `mesh`'s
# three ``MeshEventHooks`` dispatchers (``mesh25dispatch_mesh_light_event``,
# ``mesh20publish_light_status`` — renamed from ``dispatch_device_status_update``
# so the caller pattern is not a substring of its callee — and
# ``mesh21dispatch_uart_mesh_rx``), `logging`'s async sink, and `uart`'s command
# handler plus its two ``UartTxOps`` slots. Every one is a leaf dispatcher whose
# callee is an `app` hook, except ``light20device_status_update``, which the
# fixture already carries as a direct callee above so the two light-event call
# sites converge on one subtree.
FIRMWARE_SHAPED = _text(
    _fn(
        "main_entrypoint",
        40,
        (("rf_drv_826624rf_link_slave_data_write", False),),
    ),
    _fn(
        "rf_drv_826624rf_link_slave_data_write",
        184,
        (
            ("callbacks13on_light_data", False),
            ("callbacks21dispatch_ota_write", False),
        ),
    ),
    _fn("callbacks13on_light_data", 24, indirect=True),
    _fn(
        "app21rf_link_data_callback",
        128,
        (("rf_drv16emit_light_event", False),),
    ),
    _fn("rf_drv16emit_light_event", 8, indirect=True),
    _fn(
        "app28rf_link_light_event_callback",
        200,
        (("light20device_status_update", False),),
    ),
    # The light subsystem's status publish: a real `bl` into the hook seam, then
    # an indirect dispatch to the SDK's node-table writer.
    _fn(
        "light20device_status_update",
        24,
        (("light20notify_status_update", False),),
    ),
    _fn("light20notify_status_update", 8, indirect=True),
    _fn("mesh_management23ll_device_status_update", 40),
    _fn("callbacks17on_mesh_pair_proc", 24, indirect=True),
    _fn(
        "app19mesh_pair_proc_hook",
        8,
        (("MeshManager14mesh_pair_proc", False),),
    ),
    _fn("MeshManager14mesh_pair_proc", 180),
    _fn("MeshManager21mesh_pair_proc_effect", 60),
    _fn("callbacks17on_light_response", 24, indirect=True),
    _fn("app25rf_link_response_callback", 96),
    _fn("callbacks15on_uart_mesh_rx", 24, indirect=True),
    _fn("app17uart_mesh_rx_hook", 72),
    # OtaHooks: the four dispatchers in sdk/ble_app/callbacks.rs.
    _fn("callbacks21dispatch_ota_write", 24, indirect=True),
    _fn("ota21on_gatt_ota_packet", 40),
    _fn("callbacks24dispatch_ota_snapshot", 24, indirect=True),
    _fn("ota26ota_status_snapshot_impl", 16),
    _fn("callbacks25dispatch_ota_terminate", 24, indirect=True),
    _fn("ota31ota_request_termination_impl", 8),
    _fn("callbacks19dispatch_ota_tick", 24, indirect=True),
    _fn("ota27ota_tick_timeout_1s_impl", 8),
    # OtaRadioOps: the four providers in ota/mod.rs.
    _fn("ota17radio_stage_pkt", 8, indirect=True),
    _fn("light_ll22stage_ota_packet", 16),
    _fn("ota16radio_read_pkt", 8, indirect=True),
    _fn("light_ll27read_staged_ota_packet", 16),
    _fn("ota19radio_terminate_tx", 8, indirect=True),
    _fn("app24ota_send_terminate_hook", 16),
    _fn("ota22radio_begin_gatt_ota", 8, indirect=True),
    _fn("app21ota_begin_gatt_hook", 16),
    # C-8: mesh's three MeshEventHooks dispatchers. The light-event slot reaches
    # the same app callback the SDK's emit seam does.
    _fn("mesh25dispatch_mesh_light_event", 24, indirect=True),
    _fn("mesh20publish_light_status", 24, indirect=True),
    _fn("mesh21dispatch_uart_mesh_rx", 24, indirect=True),
    # C-8: uart's command handler and its two UartTxOps slots.
    _fn("uart17dispatch_uart_cmd", 24, indirect=True),
    _fn("app17uart_command_hook", 24),
    _fn("uart16uart_tx_ops_push", 8, indirect=True),
    _fn("app20enqueue_uart_tx_hook", 8),
    _fn("uart19uart_tx_ops_dropped", 8, indirect=True),
    _fn("app20dropped_uart_tx_hook", 8),
    # C-8: logging's async sink.
    _fn("logging18dispatch_log_async", 24, indirect=True),
    _fn("app17log_async_tx_hook", 8),
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
            ("app21rf_link_data_callback", False),
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
                    "app28rf_link_light_event_callback",
                ),
                ("callbacks17on_mesh_pair_proc", "app19mesh_pair_proc_hook"),
                ("callbacks13on_light_data", "app21rf_link_data_callback"),
                (
                    "callbacks17on_light_response",
                    "app25rf_link_response_callback",
                ),
                ("callbacks15on_uart_mesh_rx", "app17uart_mesh_rx_hook"),
                (
                    "light20notify_status_update",
                    "mesh_management23ll_device_status_update",
                ),
                # The OTA seam, in table order: four OtaHooks dispatchers...
                ("callbacks21dispatch_ota_write", "ota21on_gatt_ota_packet"),
                (
                    "callbacks24dispatch_ota_snapshot",
                    "ota26ota_status_snapshot_impl",
                ),
                (
                    "callbacks25dispatch_ota_terminate",
                    "ota31ota_request_termination_impl",
                ),
                (
                    "callbacks19dispatch_ota_tick",
                    "ota27ota_tick_timeout_1s_impl",
                ),
                # ...then the four OtaRadioOps providers.
                ("ota17radio_stage_pkt", "light_ll22stage_ota_packet"),
                ("ota16radio_read_pkt", "light_ll27read_staged_ota_packet"),
                ("ota19radio_terminate_tx", "app24ota_send_terminate_hook"),
                ("ota22radio_begin_gatt_ota", "app21ota_begin_gatt_hook"),
                # ...then the C-8 subsystem seams.
                (
                    "mesh25dispatch_mesh_light_event",
                    "app28rf_link_light_event_callback",
                ),
                (
                    "mesh20publish_light_status",
                    "light20device_status_update",
                ),
                ("mesh21dispatch_uart_mesh_rx", "app17uart_mesh_rx_hook"),
                ("uart17dispatch_uart_cmd", "app17uart_command_hook"),
                ("uart16uart_tx_ops_push", "app20enqueue_uart_tx_hook"),
                ("uart19uart_tx_ops_dropped", "app20dropped_uart_tx_hook"),
                ("logging18dispatch_log_async", "app17log_async_tx_hook"),
            ],
        )
        # The hook is the intermediate frame; the manager method keeps its own
        # direct `bl` edge and must not be wired twice.
        self.assertEqual(
            funcs["app19mesh_pair_proc_hook"].calls,
            [("MeshManager14mesh_pair_proc", False)],
        )
        self.assertEqual(funcs["MeshManager14mesh_pair_proc"].calls, [])
        # Two callee patterns are deliberately shared by two call sites: the
        # app light-event callback (SDK emit + mesh dispatch) and the UART relay
        # hook (SDK bridge + mesh relay). Each pair still resolves to one symbol.
        callees = [
            callee
            for _caller, callees in stack_analysis.INDIRECT_EDGES.items()
            for callee in callees
        ]
        self.assertEqual(
            sorted({c for c in callees if callees.count(c) > 1}),
            ["rf_link_light_event_callback", "uart_mesh_rx_hook"],
        )

    def test_caller_with_zero_matches_is_rejected(self) -> None:
        funcs = _parse()
        edges = {"no_such_dispatcher": ("app21rf_link_data_callback",)}
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
            "on_light_data": ("app21rf_link_data_callback",),
            "on_uart_mesh_rx": ("no_such_callback",),
        }
        with self.assertRaises(stack_analysis.IndirectEdgeError):
            stack_analysis.apply_indirect_edges(funcs, edges)
        self.assertEqual({name: f.calls for name, f in funcs.items()}, before)

    def test_table_covers_every_dispatcher(self) -> None:
        self.assertEqual(
            sorted(stack_analysis.INDIRECT_EDGES),
            [
                "dispatch_log_async",
                "dispatch_mesh_light_event",
                "dispatch_ota_snapshot",
                "dispatch_ota_terminate",
                "dispatch_ota_tick",
                "dispatch_ota_write",
                "dispatch_uart_cmd",
                "dispatch_uart_mesh_rx",
                "emit_light_event",
                "notify_status_update",
                "on_light_data",
                "on_light_response",
                "on_mesh_pair_proc",
                "on_uart_mesh_rx",
                "publish_light_status",
                "radio_begin_gatt_ota",
                "radio_read_pkt",
                "radio_stage_pkt",
                "radio_terminate_tx",
                "uart_tx_ops_dropped",
                "uart_tx_ops_push",
            ],
        )
        for caller, callees in stack_analysis.INDIRECT_EDGES.items():
            self.assertEqual(len(callees), 1, caller)

    def test_no_pattern_is_a_substring_of_another(self) -> None:
        """`_resolve_one` fails closed on ambiguity, so the table must stay sharp.

        Every caller and callee pattern is a substring searched over the whole
        symbol table: if one pattern were contained in another, whichever
        symbol matched both would resolve as ambiguous and the gate would fail
        on a correct image.  `stage_ota_packet` vs `read_staged_ota_packet` is
        the pair this guards — the extra `d` is what keeps them distinct — and
        `publish_light_status` vs `light20device_status_update` is the C-8 pair:
        the dispatcher was renamed precisely so its own name is not a substring
        of its callee's.
        """
        patterns = list(stack_analysis.INDIRECT_EDGES)
        for callees in stack_analysis.INDIRECT_EDGES.values():
            patterns.extend(callees)
        self.assertEqual(len(patterns), 42)
        # Caller patterns name distinct dispatchers, so they must be unique; a
        # *callee* pattern may legitimately be shared by two call sites, and the
        # pair test below is over the distinct set.
        self.assertEqual(
            len(set(stack_analysis.INDIRECT_EDGES)),
            len(stack_analysis.INDIRECT_EDGES),
        )
        for pattern in set(patterns):
            for other in set(patterns):
                if pattern == other:
                    continue
                self.assertNotIn(
                    pattern,
                    other,
                    f"{pattern!r} is a substring of {other!r}: _resolve_one "
                    "would be ambiguous",
                )
        self.assertNotIn("stage_ota_packet", "read_staged_ota_packet")
        self.assertNotIn(
            "publish_light_status", "light20device_status_update"
        )
        # The plain name would be ambiguous (see the next test), so the table
        # must not carry it.
        self.assertNotIn("device_status_update", patterns)

    def test_plain_device_status_update_is_ambiguous(self) -> None:
        """Why the mesh row names its callee `light20device_status_update`.

        `device_status_update` alone matches three symbols on the real image:
        the mesh dispatcher (renamed away), the lamp's own publisher and the
        SDK's node-table writer one layer below. `_resolve_one` must reject it
        rather than pick one.
        """
        funcs = _parse()
        with self.assertRaises(stack_analysis.IndirectEdgeError) as ctx:
            stack_analysis.apply_indirect_edges(
                funcs, {"publish_light_status": ("device_status_update",)}
            )
        message = str(ctx.exception)
        self.assertIn("ambiguous", message)
        self.assertIn("light20device_status_update", message)
        self.assertIn(
            "mesh_management23ll_device_status_update", message
        )


class TestIndirectEdgesFeedDepth(unittest.TestCase):
    def test_depth_increases_once_edges_are_applied(self) -> None:
        funcs = _parse()
        before, _, _ = stack_analysis.longest_path(funcs, "main_entrypoint")
        stack_analysis.apply_indirect_edges(funcs)
        after, chain, cycles = stack_analysis.longest_path(
            funcs, "main_entrypoint"
        )
        # The dispatch is a normal call chain: the whole callback subtree
        # (128 + 200) plus the status publish it triggers (24 + 8 + 40) — all of
        # it used to be scored as 0 B — now counts.
        self.assertEqual(after - before, 408)
        self.assertEqual(cycles, [])
        self.assertIn("app28rf_link_light_event_callback", chain)
        # The OTA branch of the same GATT write entry is real but shorter
        # (24 + 40), so it is wired in without moving the maximum.
        ota_depth, ota_chain, _ = stack_analysis.longest_path(
            funcs, "callbacks21dispatch_ota_write"
        )
        self.assertEqual(ota_depth, 24 + 40, "dispatch -> on_gatt_ota_packet")
        self.assertEqual(
            ota_chain,
            ["callbacks21dispatch_ota_write", "ota21on_gatt_ota_packet"],
        )
        # The light-data branch (24 + 128 + 8 + 200) stays the deepest callee of
        # the GATT write entry, so wiring the OTA branch in moves no maximum.
        self.assertLess(ota_depth, 24 + 128 + 8 + 200)

    def test_irq_chain_reaches_through_the_dispatcher(self) -> None:
        funcs = _parse()
        stack_analysis.apply_indirect_edges(funcs)
        depth, chain, _ = stack_analysis.longest_path(funcs, "irq_handler")
        self.assertEqual(
            depth,
            24 + 272 + 8 + 200 + 24 + 8 + 40,
            "irq_handler -> isr -> emit -> callback -> device_status_update -> "
            "notify_status_update -> ll_device_status_update",
        )
        self.assertIn("app28rf_link_light_event_callback", chain)
        self.assertIn("mesh_management23ll_device_status_update", chain)
        # No OTA dispatcher is reachable from the IRQ root: the seam's frames
        # only enter the IRQ stack through the handlers that call them.
        self.assertNotIn("callbacks21dispatch_ota_write", chain)

    def test_roots_no_longer_contain_indirect_only_callees(self) -> None:
        funcs = _parse()
        roots = set(stack_analysis.all_roots(funcs))
        self.assertIn("app21rf_link_data_callback", roots)
        self.assertIn("app28rf_link_light_event_callback", roots)
        self.assertIn("app17uart_mesh_rx_hook", roots)
        self.assertIn("app17uart_command_hook", roots)
        self.assertIn("app17log_async_tx_hook", roots)
        # The OTA callees are roots only because nothing `bl`s them yet.
        self.assertIn("ota21on_gatt_ota_packet", roots)
        self.assertIn("app24ota_send_terminate_hook", roots)
        stack_analysis.apply_indirect_edges(funcs)
        roots = set(stack_analysis.all_roots(funcs))
        self.assertNotIn("app21rf_link_data_callback", roots)
        self.assertNotIn("app28rf_link_light_event_callback", roots)
        self.assertNotIn("app19mesh_pair_proc_hook", roots)
        self.assertNotIn("app17uart_mesh_rx_hook", roots)
        self.assertNotIn("app17uart_command_hook", roots)
        self.assertNotIn("app20enqueue_uart_tx_hook", roots)
        self.assertNotIn("app20dropped_uart_tx_hook", roots)
        self.assertNotIn("app17log_async_tx_hook", roots)
        self.assertNotIn("ota21on_gatt_ota_packet", roots)
        self.assertNotIn("app24ota_send_terminate_hook", roots)
        self.assertIn("main_entrypoint", roots)

    def test_introducing_a_cycle_fails_the_gate(self) -> None:
        funcs = stack_analysis.parse_functions(
            _text(
                _fn("caller", 32, (("callbacks13on_light_data", False),)),
                _fn("callbacks13on_light_data", 24, indirect=True),
                _fn(
                    "app21rf_link_data_callback",
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
        self.assertIn("indirect dispatch edges (21):", out)
        self.assertIn(
            "light20notify_status_update -> mesh_management23ll_device_status_update",
            out,
        )
        self.assertIn(
            "callbacks15on_uart_mesh_rx -> app17uart_mesh_rx_hook", out
        )
        # The OTA seam must be reported, not silently resolved.
        self.assertIn(
            "callbacks21dispatch_ota_write -> ota21on_gatt_ota_packet", out
        )
        self.assertIn("ota17radio_stage_pkt -> light_ll22stage_ota_packet", out)
        # So must every C-8 seam, including the renamed status dispatcher.
        self.assertIn(
            "mesh20publish_light_status -> light20device_status_update", out
        )
        self.assertIn("uart17dispatch_uart_cmd -> app17uart_command_hook", out)
        self.assertIn(
            "uart16uart_tx_ops_push -> app20enqueue_uart_tx_hook", out
        )
        self.assertIn(
            "logging18dispatch_log_async -> app17log_async_tx_hook", out
        )
        self.assertIn("main: free 4096 B - max 656 B = margin 3440 B", out)
        self.assertIn("irq:  size 1792 B - max 604 B = margin 1188 B", out)
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

    def test_missing_ota_callee_fails_closed(self) -> None:
        # A stale OTA entry is the failure this table exists to catch: without
        # on_gatt_ota_packet the OTA write subtree would score as 0 B and the
        # gate would under-report both margins instead of failing.
        text = FIRMWARE_SHAPED.replace(_fn("ota21on_gatt_ota_packet", 40) + "\n", "")
        self.assertNotIn("ota21on_gatt_ota_packet", text)
        # The dispatcher survives; only its callee is gone.
        self.assertIn("callbacks21dispatch_ota_write", text)
        code, out = _run_main(text)
        self.assertEqual(code, 1, out)
        self.assertIn("FAIL: INDIRECT_EDGES['dispatch_ota_write']", out)
        self.assertIn("matches no parsed function", out)
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
