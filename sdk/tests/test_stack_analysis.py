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
import json
import sys
import tempfile
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
# callee is an `app` hook, except ``tlsr8266_mesh::light::device_status_update``, which the
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
        (("tlsr8266_mesh::light::device_status_update", False),),
    ),
    # The light subsystem's status publish: a real `bl` into the hook seam, then
    # an indirect dispatch to the radio crate's node-table writer.
    _fn(
        "tlsr8266_mesh::light::device_status_update",
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


# Strict GATT fixture.  The ATT handler is the only GATT root and it fans out
# exclusively to the five real callback symbols below.  The deepest main chain is
# 16 + 24 + 1644 = 1684 B, deliberately below the fixture's 2083 B free main
# SRAM budget so it fails the 400 B margin policy.
STRICT_GATT_FIXTURE = FIRMWARE_SHAPED.replace(
    _fn(
        "main_entrypoint",
        40,
        ((f"{RADIO}2rf8drv_826624rf_link_slave_data_write", False),),
    ) + "\n",
    _fn("main_entrypoint", 16, (("l2cap_att_handler", False),)) + "\n",
).replace(
    _fn(
        "irq_handler",
        24,
        ((f"{MESH}3irq24handle_system_interrupts", False),),
    ) + "\n",
    _fn("irq_handler", 24, (("l2cap_att_handler", False),)) + "\n",
).replace(
    _fn(
        f"{RADIO}2rf8drv_826624rf_link_slave_data_write",
        184,
        (
            (f"{RADIO_CALLBACKS}13on_light_data", False),
            (f"{RADIO_CALLBACKS}18dispatch_ota_write", False),
        ),
    ) + "\n",
    "",
) + _text(
    _fn("l2cap_att_handler", 24, indirect=True),
    _fn("mesh_status_write", 1644),
    _fn("rf_link_slave_data_write", 184),
    _fn("pair_write", 96),
    _fn("pair_read", 64),
)

GATT_SYMS = {
    "__RAM_START_ADDR": 0x808000,
    "__RAM_SIZE_MAX": 16384,
    "_end_bss_": 0x808000 + 16384 - 2083,
    "IRQ_STK_SIZE": 1792,
}


def _parse(text: str = FIRMWARE_SHAPED) -> dict[str, stack_analysis.Func]:
    funcs = stack_analysis.parse_functions(text)
    assert funcs, "synthetic disassembly parsed to nothing"
    return funcs


def _run_main(
    text: str,
    argv: list[str] | None = None,
    syms: dict[str, int] | None = None,
) -> tuple[int, str]:
    """Run ``main`` against synthetic disassembly and capture its stdout."""
    args = ["--elf", "synthetic.elf", "--top", "3"] if argv is None else argv
    buf = io.StringIO()
    syms = SYMS if syms is None else syms
    with (
        mock.patch.object(stack_analysis, "objdump", return_value=text),
        mock.patch.object(stack_analysis, "read_symbols", return_value=syms),
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

    def test_symbolic_blx_is_a_normal_call_and_register_blx_is_indirect(self) -> None:
        funcs = stack_analysis.parse_functions(
            _text(
                "00001000 <direct_caller>:",
                "\t1000:  blx 0x00002000 <direct_target>",
                "00002000 <direct_target>:",
                "\t2000:  4770  bx lr",
                "00003000 <indirect_caller>:",
                "\t3000:  4780  blx r3",
                "00004000 <tjex_caller>:",
                "\t4000:  tjex r5",
            )
        )
        self.assertEqual(
            funcs["direct_caller"].calls,
            [("direct_target", False)],
        )
        self.assertTrue(funcs["indirect_caller"].indirect)
        self.assertTrue(funcs["tjex_caller"].indirect)
        self.assertEqual(
            funcs["indirect_caller"].indirect_insts,
            [(0x3000, "blx", "r3")],
        )
        self.assertEqual(
            funcs["tjex_caller"].indirect_insts,
            [(0x4000, "tjex", "r5")],
        )

    def test_bx_lr_is_not_an_indirect_dispatch(self) -> None:
        funcs = stack_analysis.parse_functions(
            _text(
                "00001000 <returner>:",
                "\t1000:  bx lr",
                "00001004 <cpu_returner>:",
                "\t1004:  bx r14",
            )
        )
        self.assertFalse(funcs["returner"].indirect)
        self.assertFalse(funcs["cpu_returner"].indirect)


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
                    "tlsr8266_mesh::light::device_status_update",
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
        `publish_light_status` vs `tlsr8266_mesh::light::device_status_update` is the C-8 pair:
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
            "publish_light_status", "tlsr8266_mesh::light::device_status_update"
        )
        # The plain name would be ambiguous (see the next test), so the table
        # must not carry it.
        self.assertNotIn("device_status_update", patterns)

    def test_plain_device_status_update_is_ambiguous(self) -> None:
        """Why the mesh row names its callee `tlsr8266_mesh::light::device_status_update`.

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
        self.assertIn("tlsr8266_mesh::light::device_status_update", message)
        self.assertIn(
            "mesh_ll11node_status23ll_device_status_update", message
        )


class TestStrictIndirectContract(unittest.TestCase):
    def test_strict_gatt_table_is_exactly_the_five_real_callbacks(self) -> None:
        self.assertEqual(
            stack_analysis.STRICT_GATT_EDGES,
            {
                "l2cap_att_handler": (
                    "mesh_status_write",
                    "rf_link_slave_data_write",
                    "dispatch_ota_write",
                    "pair_write",
                    "pair_read",
                ),
            },
        )

    def test_runtime_allowlist_models_narrow_stack_treatment(self) -> None:
        self.assertEqual(stack_analysis.runtime_allowlist_treatment("__pender"), "leaf")
        self.assertEqual(
            stack_analysis.runtime_allowlist_treatment(
                "embassy_executor::raw::Executor::poll"
            ),
            "normal_call",
        )
        self.assertEqual(
            stack_analysis.runtime_allowlist_treatment(
                "embassy_sync::blocking_mutex::raw::RawMutex::lock"
            ),
            "normal_call",
        )
        self.assertEqual(
            stack_analysis.runtime_allowlist_treatment("core::fmt::write"),
            "normal_call",
        )
        self.assertEqual(
            stack_analysis.runtime_allowlist_treatment(
                "core::cell::BorrowMutError::fmt"
            ),
            "normal_call",
        )
        self.assertEqual(
            stack_analysis.runtime_allowlist_treatment(
                "tlsr8266_hal::runtime::executor::Executor::run"
            ),
            "normal_call",
        )
        self.assertEqual(
            stack_analysis.runtime_allowlist_treatment(
                "<tlsr8266_hal::runtime::executor::Executor>::run::<main_entrypoint>"
            ),
            "normal_call",
        )
        self.assertEqual(
            stack_analysis.runtime_allowlist_treatment(
                "<embassy_time_queue_utils::Queue>::next_expiration"
            ),
            "leaf",
        )
        self.assertIsNone(
            stack_analysis.runtime_allowlist_treatment("project_callback")
        )
        # Rust-v0 mangled names must not be the strict contract. Demangled ELF
        # names and stable export names are the only accepted live forms here.
        self.assertIsNone(
            stack_analysis.runtime_allowlist_treatment(
                "_RNvNtCsgudHjhsztDx_4core3fmt5write"
            )
        )
        self.assertEqual(
            stack_analysis.strict_project_indirect_treatment(
                "tlsr8266_enqueue_mesh_rx"
            ),
            "normal_call",
        )
        self.assertEqual(
            stack_analysis.strict_project_indirect_treatment(
                "tlsr8266_enqueue_mesh_tx"
            ),
            "normal_call",
        )
        self.assertEqual(
            stack_analysis.strict_project_indirect_treatment(
                "tlsr8266_handle_system_interrupts"
            ),
            "normal_call",
        )

        funcs = stack_analysis.parse_functions(
            _text(
                _fn("__pender", 16, indirect=True),
                _fn("project_callback", 32, indirect=True),
            )
        )
        failures, allowed, _classified = (
            stack_analysis.collect_strict_indirect_failures(funcs)
        )
        self.assertEqual(allowed, ["__pender"])
        self.assertEqual(len(failures), 1)
        self.assertIn("project_callback", failures[0])
        self.assertIn("0x00000008", failures[0])

    def test_resolved_legacy_callee_is_known_strict_target(self) -> None:
        funcs = stack_analysis.parse_functions(
            _text(_fn("rf_link_data_callback", 32, indirect=True))
        )
        failures, allowed, classified = stack_analysis.collect_strict_indirect_failures(
            funcs,
            resolved_legacy=[("on_light_data", "rf_link_data_callback")],
        )
        self.assertEqual(failures, [])
        self.assertEqual(allowed, [])
        self.assertIn("rf_link_data_callback", classified)

    def test_unknown_indirect_project_branch_fails_with_symbol_and_address(self) -> None:
        text = STRICT_GATT_FIXTURE + "\n" + _fn(
            "project_unknown_dispatcher", 32, indirect=True
        )
        code, out = _run_main(
            text,
            ["--elf", "synthetic.elf", "--top", "3"],
            GATT_SYMS,
        )
        self.assertEqual(code, 1, out)
        self.assertIn("project_unknown_dispatcher", out)
        self.assertIn("instruction 0x00000008", out)
        self.assertIn("strict unknown indirect project branch", out)

    def test_ambiguous_gatt_root_is_rejected_before_mutation(self) -> None:
        funcs = _parse(STRICT_GATT_FIXTURE)
        funcs.update(
            stack_analysis.parse_functions(
                _text(_fn("l2cap_att_handler_other", 24, indirect=True))
            )
        )
        before = {name: list(func.calls) for name, func in funcs.items()}
        with self.assertRaises(stack_analysis.IndirectEdgeError) as ctx:
            stack_analysis.apply_strict_indirect_edges(funcs)
        message = str(ctx.exception)
        self.assertIn("STRICT_GATT_EDGES", message)
        self.assertIn("l2cap_att_handler", message)
        self.assertIn("ambiguous", message)
        self.assertEqual({name: func.calls for name, func in funcs.items()}, before)

    def test_direct_default_strict_gatt_reports_five_targets_and_margin_failure(self) -> None:
        code, out = _run_main(
            STRICT_GATT_FIXTURE,
            ["--elf", "synthetic.elf", "--top", "3",
             "--min-margin", "400"],
            GATT_SYMS,
        )
        self.assertEqual(code, 1, out)
        self.assertIn("strict GATT dispatch edges (5):", out)
        self.assertIn("l2cap_att_handler -> mesh_status_write", out)
        self.assertIn("l2cap_att_handler -> rf_link_slave_data_write", out)
        self.assertIn("dispatch_ota_write", out)
        self.assertIn("l2cap_att_handler -> pair_write", out)
        self.assertIn("l2cap_att_handler -> pair_read", out)
        self.assertIn("FAIL: main stack margin below minimum", out)
        self.assertNotIn("strict GATT-inclusive path violates", out)

    def test_legacy_mode_does_not_add_gatt_edges(self) -> None:
        code, out = _run_main(
            STRICT_GATT_FIXTURE,
            ["--elf", "synthetic.elf", "--top", "3", "--legacy"],
        )
        self.assertEqual(code, 0, out)
        self.assertIn("indirect dispatch edges (21):", out)
        self.assertNotIn("strict GATT dispatch edges", out)


class TestRamReachability(unittest.TestCase):
    def test_elf_section_parser_classifies_ram_text_and_rodata_ranges(self) -> None:
        sections = stack_analysis.parse_elf_sections(
            "  [ 1] .ram_code      PROGBITS        00000200 00000000 00000040 00  AX  0  0  4\n"
            "  [ 2] .text          PROGBITS        00000300 00000040 00000080 00  AX  0  0  4\n"
            "  [ 3] .rodata        PROGBITS        00000400 000000c0 00000020 00   A  0  0  4\n"
        )
        self.assertEqual(sections[".ram_code"], (0x200, 0x40))
        self.assertEqual(sections[".text"], (0x300, 0x80))
        self.assertEqual(sections[".rodata"], (0x400, 0x20))

    def test_boot_ota_ram_chain_is_safe_through_direct_and_tail_edges(self) -> None:
        funcs = {
            "handle_ota_update": stack_analysis.Func(
                0x200,
                frame=32,
                calls=[("reboot_and_retry_copy", False)],
                section=".ram_code",
            ),
            "reboot_and_retry_copy": stack_analysis.Func(
                0x210,
                frame=16,
                calls=[("reboot", True)],
                section=".ram_code",
            ),
            "reboot": stack_analysis.Func(0x220, frame=8, section=".ram_code"),
        }
        self.assertEqual(
            stack_analysis.check_ram_reachability(
                funcs, root_pattern="handle_ota_update"
            ),
            [],
        )

    def test_boot_ota_chain_to_flash_resident_reboot_leaf_is_safe(self) -> None:
        funcs = {
            "handle_ota_update": stack_analysis.Func(
                0x200,
                frame=32,
                calls=[("reboot_and_retry_copy", False)],
                section=".ram_code",
            ),
            "reboot_and_retry_copy": stack_analysis.Func(
                0x210,
                frame=16,
                calls=[("light_sw_reboot", False)],
                section=".ram_code",
            ),
            "light_sw_reboot": stack_analysis.Func(
                0x300,
                frame=8,
                section=".text",
            ),
        }
        self.assertEqual(
            stack_analysis.check_ram_reachability(
                funcs, root_pattern="handle_ota_update"
            ),
            [],
        )

    def test_unrelated_irq_ram_flash_edge_is_outside_boot_ota_root(self) -> None:
        funcs = {
            "handle_ota_update": stack_analysis.Func(
                0x200,
                frame=32,
                calls=[],
                section=".ram_code",
            ),
            "irq_handler": stack_analysis.Func(
                0x334,
                frame=24,
                calls=[("handle_system_interrupts", False)],
                section=".ram_code",
            ),
            "handle_system_interrupts": stack_analysis.Func(
                0x300,
                frame=272,
                section=".text",
            ),
        }
        self.assertEqual(
            stack_analysis.check_ram_reachability(
                funcs, root_pattern="handle_ota_update"
            ),
            [],
        )

    def test_boot_ota_approved_indirect_chain_is_safe_when_targets_are_ram(self) -> None:
        funcs = {
            "handle_ota_update": stack_analysis.Func(
                0x200,
                frame=32,
                calls=[("reboot_and_retry_copy", False)],
                section=".ram_code",
            ),
            "reboot_and_retry_copy": stack_analysis.Func(
                0x210,
                frame=16,
                calls=[("reboot", False)],
                section=".ram_code",
                indirect=True,
            ),
            "reboot": stack_analysis.Func(0x220, frame=8, section=".ram_code"),
        }
        self.assertEqual(
            stack_analysis.check_ram_reachability(
                funcs, root_pattern="handle_ota_update"
            ),
            [],
        )

    def test_flash_resident_wrapper_or_callee_fails_ram_reachability(self) -> None:
        funcs = {
            "ram_wrapper": stack_analysis.Func(
                0x200,
                frame=32,
                calls=[("flash_callee", False)],
                section=".ram_code",
            ),
            "flash_callee": stack_analysis.Func(
                0x300,
                frame=64,
                section=".text",
            ),
        }
        violations = stack_analysis.check_ram_reachability(
            funcs, root_pattern="ram_wrapper"
        )
        self.assertEqual(violations, [("ram_wrapper", "flash_callee", ".text")])
        with self.assertRaises(stack_analysis.RamReachabilityError) as ctx:
            stack_analysis.validate_ram_reachability(
                funcs, root_pattern="ram_wrapper"
            )
        self.assertIn("ram_wrapper", str(ctx.exception))
        self.assertIn("flash_callee", str(ctx.exception))
        self.assertIn(".text", str(ctx.exception))


class TestBaselineJson(unittest.TestCase):
    def test_legacy_baseline_reports_image_sections_and_stack_counts(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            elf = Path(tmp) / "firmware.elf"
            bin_path = Path(tmp) / "firmware.bin"
            out_path = Path(tmp) / "baseline.json"
            elf.write_bytes(b"ELF")
            bin_path.write_bytes(b"BIN")

            sections = {
                ".text": (0x0, 0x1000),
                ".rodata": (0x1000, 0x400),
                ".ram_code": (0x1400, 0x400),
            }
            buf = io.StringIO()
            with (
                mock.patch.object(stack_analysis, "objdump", return_value=FIRMWARE_SHAPED),
                mock.patch.object(stack_analysis, "read_symbols", return_value=SYMS),
                mock.patch.object(stack_analysis, "read_elf_sections", return_value=sections),
                contextlib.redirect_stdout(buf),
            ):
                code = stack_analysis.main([
                    "--elf", str(elf), "--top", "3", "--legacy",
                    "--baseline-json", str(out_path),
                ])

            self.assertEqual(code, 0, buf.getvalue())
            with out_path.open("r", encoding="utf-8") as handle:
                baseline = json.load(handle)

            self.assertEqual(baseline["elf_bytes"], 3)
            self.assertEqual(baseline["bin_bytes"], 3)
            self.assertEqual(baseline["sections"][".text"], 0x1000)
            self.assertEqual(baseline["free_main_sram"], 4096)
            self.assertEqual(baseline["main_max"], 656)
            self.assertEqual(baseline["irq_max"], 604)
            self.assertEqual(baseline["main_margin"], 3440)
            self.assertEqual(baseline["irq_margin"], 1188)
            self.assertGreater(baseline["parsed_functions"], 0)
            self.assertGreater(baseline["indirect_functions"], 0)
            self.assertEqual(baseline["allowed_indirect"], 21)
            self.assertEqual(baseline["mode"], "legacy")
            self.assertFalse(baseline["strict"])

    def test_direct_default_strict_baseline_reports_gatt_edges(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            elf = Path(tmp) / "firmware.elf"
            bin_path = Path(tmp) / "firmware.bin"
            out_path = Path(tmp) / "baseline.json"
            elf.write_bytes(b"ELF")
            bin_path.write_bytes(b"BIN")

            sections = {
                ".text": (0x0, 0x1000),
                ".rodata": (0x1000, 0x400),
                ".ram_code": (0x1400, 0x400),
            }
            strict_pass_fixture = STRICT_GATT_FIXTURE.replace(
                _fn("mesh_status_write", 1644),
                _fn("mesh_status_write", 64),
            )
            buf = io.StringIO()
            with (
                mock.patch.object(stack_analysis, "objdump", return_value=strict_pass_fixture),
                mock.patch.object(stack_analysis, "read_symbols", return_value=SYMS),
                mock.patch.object(stack_analysis, "read_elf_sections", return_value=sections),
                contextlib.redirect_stdout(buf),
            ):
                code = stack_analysis.main([
                    "--elf", str(elf), "--top", "3",
                    "--baseline-json", str(out_path),
                ])

            self.assertEqual(code, 0, buf.getvalue())
            with out_path.open("r", encoding="utf-8") as handle:
                baseline = json.load(handle)

            self.assertEqual(baseline["mode"], "strict")
            self.assertTrue(baseline["strict"])
            self.assertEqual(baseline["legacy_edges_resolved"], 21)
            self.assertEqual(baseline["gatt_edges_resolved"], 5)


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
    def test_legacy_pass_reports_resolved_edges_and_margins(self) -> None:
        code, out = _run_main(
            FIRMWARE_SHAPED,
            ["--elf", "synthetic.elf", "--top", "3", "--legacy"],
        )
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
            "tlsr8266_mesh::light::device_status_update",
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
        code, out = _run_main(
            text,
            ["--elf", "synthetic.elf", "--top", "3", "--legacy"],
        )
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
        code, out = _run_main(
            text,
            ["--elf", "synthetic.elf", "--top", "3", "--legacy"],
        )
        self.assertEqual(code, 1, out)
        self.assertIn("FAIL: INDIRECT_EDGES['dispatch_ota_write']", out)
        self.assertIn("matches no parsed function", out)
        self.assertNotIn("RAM GATE", out)

    def test_margin_below_minimum_still_fails(self) -> None:
        code, out = _run_main(
            FIRMWARE_SHAPED, ["--elf", "synthetic.elf", "--top", "0",
                              "--min-margin", "4000", "--legacy"]
        )
        self.assertEqual(code, 1, out)
        self.assertIn("FAIL: main stack margin below minimum", out)

    def test_parse_args_accepts_explicit_modes_and_defaults_to_strict(self) -> None:
        args = stack_analysis.parse_args(
            ["--elf", "x.elf", "--top", "7", "--min-margin", "512",
             "--exclude-panic", "--strict", "--baseline-json", "base.json"]
        )
        self.assertEqual(
            (args.elf, args.top, args.min_margin, args.exclude_panic),
            ("x.elf", 7, 512, True),
        )
        self.assertTrue(args.strict)
        self.assertFalse(args.legacy)
        self.assertEqual(args.baseline_json, "base.json")

        default_args = stack_analysis.parse_args([])
        self.assertTrue(default_args.strict)
        self.assertFalse(default_args.legacy)
        self.assertFalse(default_args.exclude_panic)

        legacy_args = stack_analysis.parse_args(["--legacy"])
        self.assertFalse(legacy_args.strict)
        self.assertTrue(legacy_args.legacy)

        with self.assertRaises(SystemExit):
            stack_analysis.parse_args(["--strict", "--legacy"])


if __name__ == "__main__":
    unittest.main()
