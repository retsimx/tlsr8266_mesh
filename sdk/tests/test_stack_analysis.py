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


# The linked image's crate prefixes, spelled the way `llvm-nm` prints them after
# C-10 (#72).  The radio modules moved into `tlsr8266_radio`, and several paths
# changed with them (`rf_drv.rs` -> `rf::drv_8266`, `light_ll::mesh_management`
# -> `mesh_ll::node_status`, the OTA staging shims -> `ble::connection`), so the
# fixture below uses the paths the linked image actually has.  The
# `_RN...<hash>` prefix objdump prints is omitted on purpose: the crate
# disambiguator changes on every rebuild, which is exactly why ``INDIRECT_EDGES``
# matches substrings rather than whole names.
RADIO = "tlsr8266_radio"
RADIO_CALLBACKS = RADIO + "9callbacks"
MESH = "tlsr8266_mesh"

# A miniature version of the real call graph: a root reaches the light-data
# dispatcher directly, the dispatcher reaches the app callback only through a
# `bx rN`, and that callback reaches the light-event dispatcher, which in turn
# reaches the app's light-event handler only through another `bx rN`. The
# `mesh_pair_proc` lookalikes are present because the table's
# ``mesh_pair_proc_hook`` entry must not be confused with them.
#
# The light subsystem's status publish is modelled as the real firmware shape:
# ``rf_link_light_event_callback`` calls ``light::device_status_update`` directly,
# which calls the ``notify_status_update`` dispatcher, which reaches the radio
# crate's ``ll_device_status_update`` only through a ``bx rN``.
#
# The OTA seam is modelled the same way: one synthetic pair per ``INDIRECT_EDGES``
# entry — the OTA write dispatcher 24 B / 40 B (the real shape: a GATT write
# lands in ``dispatch_ota_write``, which reaches the OTA packet handler), the
# snapshot hook 24 B / 16 B, the two flag-only hooks 24 B / 8 B, and the four
# radio providers 8 B / 16 B. ``dispatch_ota_write`` is a *direct* callee of the
# GATT write entry, because that is the real firmware shape; every other OTA
# dispatcher stays a leaf, so the maxima do not move.
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
        ((f"{RADIO}2rf8drv_826624rf_link_slave_data_write", False),),
    ),
    _fn(
        f"{RADIO}2rf8drv_826624rf_link_slave_data_write",
        184,
        (
            (f"{RADIO_CALLBACKS}13on_light_data", False),
            (f"{RADIO_CALLBACKS}18dispatch_ota_write", False),
        ),
    ),
    _fn(f"{RADIO_CALLBACKS}13on_light_data", 24, indirect=True),
    _fn(
        f"{MESH}3app21rf_link_data_callback",
        128,
        ((f"{RADIO}7mesh_ll3adv16emit_light_event", False),),
    ),
    _fn(f"{RADIO}7mesh_ll3adv16emit_light_event", 8, indirect=True),
    _fn(
        f"{MESH}3app28rf_link_light_event_callback",
        200,
        ((f"{MESH}5light20device_status_update", False),),
    ),
    # The light subsystem's status publish: a real `bl` into the hook seam, then
    # an indirect dispatch to the radio crate's node-table writer.
    _fn(
        f"{MESH}5light20device_status_update",
        24,
        ((f"{MESH}5light20notify_status_update", False),),
    ),
    _fn(f"{MESH}5light20notify_status_update", 8, indirect=True),
    _fn(f"{RADIO}7mesh_ll11node_status23ll_device_status_update", 40),
    _fn(f"{RADIO_CALLBACKS}17on_mesh_pair_proc", 24, indirect=True),
    _fn(
        f"{MESH}3app19mesh_pair_proc_hook",
        8,
        ((f"{MESH}4mesh14mesh_pair_proc", False),),
    ),
    _fn(f"{MESH}4mesh14mesh_pair_proc", 180),
    _fn(f"{RADIO_CALLBACKS}17on_light_response", 24, indirect=True),
    _fn(f"{MESH}3app25rf_link_response_callback", 96),
    _fn(f"{RADIO_CALLBACKS}15on_uart_mesh_rx", 24, indirect=True),
    _fn(f"{MESH}3app17uart_mesh_rx_hook", 72),
    # OtaHooks: the four dispatchers in the radio crate's `callbacks.rs`.
    _fn(f"{RADIO_CALLBACKS}18dispatch_ota_write", 24, indirect=True),
    _fn(f"{MESH}3ota18on_gatt_ota_packet", 40),
    _fn(f"{RADIO_CALLBACKS}21dispatch_ota_snapshot", 24, indirect=True),
    _fn(f"{MESH}3ota24ota_status_snapshot_impl", 16),
    _fn(f"{RADIO_CALLBACKS}22dispatch_ota_terminate", 24, indirect=True),
    _fn(f"{MESH}3ota28ota_request_termination_impl", 8),
    _fn(f"{RADIO_CALLBACKS}17dispatch_ota_tick", 24, indirect=True),
    _fn(f"{MESH}3ota24ota_tick_timeout_1s_impl", 8),
    # OtaRadioOps: the four providers in ota/mod.rs, into the radio crate.
    _fn(f"{MESH}3ota15radio_stage_pkt", 8, indirect=True),
    _fn(f"{RADIO}3ble10connection16stage_ota_packet", 16),
    _fn(f"{MESH}3ota14radio_read_pkt", 8, indirect=True),
    _fn(f"{RADIO}3ble10connection22read_staged_ota_packet", 16),
    _fn(f"{MESH}3ota18radio_terminate_tx", 8, indirect=True),
    _fn(f"{MESH}3app23ota_send_terminate_hook", 16),
    _fn(f"{MESH}3ota20radio_begin_gatt_ota", 8, indirect=True),
    _fn(f"{MESH}3app19ota_begin_gatt_hook", 16),
    # C-8: mesh's three MeshEventHooks dispatchers. The light-event slot reaches
    # the same app callback the radio crate's emit seam does.
    _fn(f"{MESH}4mesh25dispatch_mesh_light_event", 24, indirect=True),
    _fn(f"{MESH}4mesh20publish_light_status", 24, indirect=True),
    _fn(f"{MESH}4mesh21dispatch_uart_mesh_rx", 24, indirect=True),
    # C-8: uart's command handler and its two UartTxOps slots.
    _fn(f"{MESH}4uart17dispatch_uart_cmd", 24, indirect=True),
    _fn(f"{MESH}3app17uart_command_hook", 24),
    _fn(f"{MESH}4uart16uart_tx_ops_push", 8, indirect=True),
    _fn(f"{MESH}3app20enqueue_uart_tx_hook", 8),
    _fn(f"{MESH}4uart19uart_tx_ops_dropped", 8, indirect=True),
    _fn(f"{MESH}3app20dropped_uart_tx_hook", 8),
    # C-8: logging's async sink.
    _fn(f"{MESH}7logging18dispatch_log_async", 24, indirect=True),
    _fn(f"{MESH}3app17log_async_tx_hook", 8),
    _fn(
        "irq_handler",
        24,
        ((f"{MESH}3irq24handle_system_interrupts", False),),
    ),
    _fn(
        f"{MESH}3irq24handle_system_interrupts",
        272,
        ((f"{RADIO}7mesh_ll3adv16emit_light_event", False),),
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
        writer = f"{RADIO}2rf8drv_826624rf_link_slave_data_write"
        self.assertEqual(funcs[writer].frame, 184)
        self.assertIn(
            (f"{RADIO_CALLBACKS}13on_light_data", False),
            funcs[writer].calls,
        )
        self.assertEqual(funcs["irq_handler"].frame, 24)

    def test_indirect_dispatch_produces_no_edge(self) -> None:
        """The whole reason INDIRECT_EDGES exists: `bx rN` is a dead end."""
        funcs = _parse()
        dispatcher = funcs[f"{RADIO_CALLBACKS}13on_light_data"]
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
            (f"{MESH}3app21rf_link_data_callback", False),
            funcs[f"{RADIO_CALLBACKS}13on_light_data"].calls,
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
                    f"{RADIO}7mesh_ll3adv16emit_light_event",
                    f"{MESH}3app28rf_link_light_event_callback",
                ),
                (
                    f"{RADIO_CALLBACKS}17on_mesh_pair_proc",
                    f"{MESH}3app19mesh_pair_proc_hook",
                ),
                (
                    f"{RADIO_CALLBACKS}13on_light_data",
                    f"{MESH}3app21rf_link_data_callback",
                ),
                (
                    f"{RADIO_CALLBACKS}17on_light_response",
                    f"{MESH}3app25rf_link_response_callback",
                ),
                (
                    f"{RADIO_CALLBACKS}15on_uart_mesh_rx",
                    f"{MESH}3app17uart_mesh_rx_hook",
                ),
                (
                    f"{MESH}5light20notify_status_update",
                    f"{RADIO}7mesh_ll11node_status23ll_device_status_update",
                ),
                # The OTA seam, in table order: four OtaHooks dispatchers...
                (
                    f"{RADIO_CALLBACKS}18dispatch_ota_write",
                    f"{MESH}3ota18on_gatt_ota_packet",
                ),
                (
                    f"{RADIO_CALLBACKS}21dispatch_ota_snapshot",
                    f"{MESH}3ota24ota_status_snapshot_impl",
                ),
                (
                    f"{RADIO_CALLBACKS}22dispatch_ota_terminate",
                    f"{MESH}3ota28ota_request_termination_impl",
                ),
                (
                    f"{RADIO_CALLBACKS}17dispatch_ota_tick",
                    f"{MESH}3ota24ota_tick_timeout_1s_impl",
                ),
                # ...then the four OtaRadioOps providers.
                (
                    f"{MESH}3ota15radio_stage_pkt",
                    f"{RADIO}3ble10connection16stage_ota_packet",
                ),
                (
                    f"{MESH}3ota14radio_read_pkt",
                    f"{RADIO}3ble10connection22read_staged_ota_packet",
                ),
                (
                    f"{MESH}3ota18radio_terminate_tx",
                    f"{MESH}3app23ota_send_terminate_hook",
                ),
                (
                    f"{MESH}3ota20radio_begin_gatt_ota",
                    f"{MESH}3app19ota_begin_gatt_hook",
                ),
                # ...then the C-8 subsystem seams.
                (
                    f"{MESH}4mesh25dispatch_mesh_light_event",
                    f"{MESH}3app28rf_link_light_event_callback",
                ),
                (
                    f"{MESH}4mesh20publish_light_status",
                    f"{MESH}5light20device_status_update",
                ),
                (
                    f"{MESH}4mesh21dispatch_uart_mesh_rx",
                    f"{MESH}3app17uart_mesh_rx_hook",
                ),
                (
                    f"{MESH}4uart17dispatch_uart_cmd",
                    f"{MESH}3app17uart_command_hook",
                ),
                (
                    f"{MESH}4uart16uart_tx_ops_push",
                    f"{MESH}3app20enqueue_uart_tx_hook",
                ),
                (
                    f"{MESH}4uart19uart_tx_ops_dropped",
                    f"{MESH}3app20dropped_uart_tx_hook",
                ),
                (
                    f"{MESH}7logging18dispatch_log_async",
                    f"{MESH}3app17log_async_tx_hook",
                ),
            ],
        )
        # The hook is the intermediate frame; the mesh method keeps its own
        # direct `bl` edge and must not be wired twice.
        self.assertEqual(
            funcs[f"{MESH}3app19mesh_pair_proc_hook"].calls,
            [(f"{MESH}4mesh14mesh_pair_proc", False)],
        )
        self.assertEqual(funcs[f"{MESH}4mesh14mesh_pair_proc"].calls, [])
        # Two callee patterns are deliberately shared by two call sites: the
        # app light-event callback (radio emit + mesh dispatch) and the UART
        # relay hook (radio bridge + mesh relay). Each pair still resolves to
        # one symbol.
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
        edges = {"no_such_dispatcher": (f"{MESH}3app21rf_link_data_callback",)}
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
        """`mesh_pair_proc` alone matches the dispatcher, the hook and the method.

        On the real image the dispatcher's own name is `on_mesh_pair_proc`, so a
        bare `mesh_pair_proc` callee pattern is ambiguous three ways — which is
        why the table names the hook.
        """
        funcs = _parse()
        edges = {"on_mesh_pair_proc": ("mesh_pair_proc",)}
        with self.assertRaises(stack_analysis.IndirectEdgeError) as ctx:
            stack_analysis.apply_indirect_edges(funcs, edges)
        message = str(ctx.exception)
        self.assertIn("ambiguous", message)
        # Every candidate must be listed so the fix is obvious.
        self.assertIn(f"{RADIO_CALLBACKS}17on_mesh_pair_proc", message)
        self.assertIn(f"{MESH}3app19mesh_pair_proc_hook", message)
        self.assertIn(f"{MESH}4mesh14mesh_pair_proc", message)

    def test_hook_substring_disambiguates(self) -> None:
        funcs = _parse()
        resolved = stack_analysis.apply_indirect_edges(
            funcs, {"on_mesh_pair_proc": ("mesh_pair_proc_hook",)}
        )
        self.assertEqual(
            resolved,
            [
                (
                    f"{RADIO_CALLBACKS}17on_mesh_pair_proc",
                    f"{MESH}3app19mesh_pair_proc_hook",
                )
            ],
        )

    def test_rejected_table_leaves_the_graph_untouched(self) -> None:
        funcs = _parse()
        before = {name: list(f.calls) for name, f in funcs.items()}
        edges = {
            "on_light_data": (f"{MESH}3app21rf_link_data_callback",),
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

        `device_status_update` alone matches two symbols on the real image: the
        lamp's own publisher and the radio crate's node-table writer one layer
        below (`mesh_ll::node_status`). `_resolve_one` must reject it rather
        than pick one.
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
            "mesh_ll11node_status23ll_device_status_update", message
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
        self.assertIn(f"{MESH}3app28rf_link_light_event_callback", chain)
        # The OTA branch of the same GATT write entry is real but shorter
        # (24 + 40), so it is wired in without moving the maximum.
        ota_depth, ota_chain, _ = stack_analysis.longest_path(
            funcs, f"{RADIO_CALLBACKS}18dispatch_ota_write"
        )
        self.assertEqual(ota_depth, 24 + 40, "dispatch -> on_gatt_ota_packet")
        self.assertEqual(
            ota_chain,
            [
                f"{RADIO_CALLBACKS}18dispatch_ota_write",
                f"{MESH}3ota18on_gatt_ota_packet",
            ],
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
        self.assertIn(f"{MESH}3app28rf_link_light_event_callback", chain)
        self.assertIn(
            f"{RADIO}7mesh_ll11node_status23ll_device_status_update", chain
        )
        # No OTA dispatcher is reachable from the IRQ root: the seam's frames
        # only enter the IRQ stack through the handlers that call them.
        self.assertNotIn(f"{RADIO_CALLBACKS}18dispatch_ota_write", chain)

    def test_roots_no_longer_contain_indirect_only_callees(self) -> None:
        funcs = _parse()
        roots = set(stack_analysis.all_roots(funcs))
        self.assertIn(f"{MESH}3app21rf_link_data_callback", roots)
        self.assertIn(f"{MESH}3app28rf_link_light_event_callback", roots)
        self.assertIn(f"{MESH}3app17uart_mesh_rx_hook", roots)
        self.assertIn(f"{MESH}3app17uart_command_hook", roots)
        self.assertIn(f"{MESH}3app17log_async_tx_hook", roots)
        # The OTA callees are roots only because nothing `bl`s them yet.
        self.assertIn(f"{MESH}3ota18on_gatt_ota_packet", roots)
        self.assertIn(f"{MESH}3app23ota_send_terminate_hook", roots)
        stack_analysis.apply_indirect_edges(funcs)
        roots = set(stack_analysis.all_roots(funcs))
        self.assertNotIn(f"{MESH}3app21rf_link_data_callback", roots)
        self.assertNotIn(f"{MESH}3app28rf_link_light_event_callback", roots)
        self.assertNotIn(f"{MESH}3app19mesh_pair_proc_hook", roots)
        self.assertNotIn(f"{MESH}3app17uart_mesh_rx_hook", roots)
        self.assertNotIn(f"{MESH}3app17uart_command_hook", roots)
        self.assertNotIn(f"{MESH}3app20enqueue_uart_tx_hook", roots)
        self.assertNotIn(f"{MESH}3app20dropped_uart_tx_hook", roots)
        self.assertNotIn(f"{MESH}3app17log_async_tx_hook", roots)
        self.assertNotIn(f"{MESH}3ota18on_gatt_ota_packet", roots)
        self.assertNotIn(f"{MESH}3app23ota_send_terminate_hook", roots)
        self.assertIn("main_entrypoint", roots)

    def test_introducing_a_cycle_fails_the_gate(self) -> None:
        funcs = stack_analysis.parse_functions(
            _text(
                _fn("caller", 32, ((f"{RADIO_CALLBACKS}13on_light_data", False),)),
                _fn(f"{RADIO_CALLBACKS}13on_light_data", 24, indirect=True),
                _fn(
                    f"{MESH}3app21rf_link_data_callback",
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
            f"{MESH}5light20notify_status_update -> "
            f"{RADIO}7mesh_ll11node_status23ll_device_status_update",
            out,
        )
        self.assertIn(
            f"{RADIO_CALLBACKS}15on_uart_mesh_rx -> "
            f"{MESH}3app17uart_mesh_rx_hook",
            out,
        )
        # The OTA seam must be reported, not silently resolved.
        self.assertIn(
            f"{RADIO_CALLBACKS}18dispatch_ota_write -> "
            f"{MESH}3ota18on_gatt_ota_packet",
            out,
        )
        self.assertIn(
            f"{MESH}3ota15radio_stage_pkt -> "
            f"{RADIO}3ble10connection16stage_ota_packet",
            out,
        )
        # So must every C-8 seam, including the renamed status dispatcher.
        self.assertIn(
            f"{MESH}4mesh20publish_light_status -> "
            f"{MESH}5light20device_status_update",
            out,
        )
        self.assertIn(
            f"{MESH}4uart17dispatch_uart_cmd -> "
            f"{MESH}3app17uart_command_hook",
            out,
        )
        self.assertIn(
            f"{MESH}4uart16uart_tx_ops_push -> "
            f"{MESH}3app20enqueue_uart_tx_hook",
            out,
        )
        self.assertIn(
            f"{MESH}7logging18dispatch_log_async -> "
            f"{MESH}3app17log_async_tx_hook",
            out,
        )
        self.assertIn("main: free 4096 B - max 656 B = margin 3440 B", out)
        self.assertIn("irq:  size 1792 B - max 604 B = margin 1188 B", out)
        self.assertIn("PASS", out)

    def test_stale_caller_table_fails_closed(self) -> None:
        # Drop the UART dispatcher: on_uart_mesh_rx now matches nothing.
        dropped = f"{RADIO_CALLBACKS}15on_uart_mesh_rx"
        text = FIRMWARE_SHAPED.replace(
            _fn(dropped, 24, indirect=True) + "\n", ""
        )
        self.assertNotIn(dropped, text)
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
        callee = f"{MESH}3ota18on_gatt_ota_packet"
        text = FIRMWARE_SHAPED.replace(_fn(callee, 40) + "\n", "")
        self.assertNotIn(callee, text)
        # The dispatcher survives; only its callee is gone.
        self.assertIn(f"{RADIO_CALLBACKS}18dispatch_ota_write", text)
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
