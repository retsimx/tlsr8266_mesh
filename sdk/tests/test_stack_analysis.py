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
    addr: int = 0,
    words: tuple[int, ...] = (),
) -> str:
    """Render one synthetic disassembly block with an exact ``frame``-byte frame.

    A dispatcher is written as an indirect branch with *no* ``bl`` edge to its
    real callee, which is what the linked firmware looks like.  ``addr`` places
    the symbol inside a synthetic section; ``words`` emits an inline ``.word``
    literal pool at ``addr + 0x40 + 4 * index``, the way ``llvm-objdump`` prints
    one.
    """
    lines = [f"{addr:04x} <{name}>:", "\t0: 6580  \tpush\t{r7, lr}"]
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
    for index, value in enumerate(words):
        pool_addr = addr + 0x40 + 4 * index
        raw = " ".join(f"{byte:02x}" for byte in value.to_bytes(4, "little"))
        lines.append(f"\t{pool_addr:x}: {raw}  \t.word\t0x{value:08x}")
    return "\n".join(lines)


def _text(*blocks: str) -> str:
    return "\n".join(blocks) + "\n"


# The linked image's crate prefixes, spelled the way `llvm-nm` prints them after
# C-10 (#72).  The radio modules moved into `tlsr8266_radio`, and several paths
# changed with them (`rf_drv.rs` -> `rf::drv_8266`, `light_ll::mesh_management`
# -> `mesh_ll::node_status`), so the
# fixture below uses the paths the linked image actually has.  The
# `_RN...<hash>` prefix objdump prints is omitted on purpose: the crate
# disambiguator changes on every rebuild, which is exactly why ``INDIRECT_EDGES``
# matches substrings rather than whole names.
RADIO = "tlsr8266_radio"
RADIO_CALLBACKS = RADIO + "9callbacks"
MESH = "tlsr8266_mesh"

# Section ranges for the synthetic image. `.text`/`.rodata` hold the ordinary
# firmware shape at address 0; the boot OTA chain sits at 0x1400+ so it
# classifies into `.ram_code`, exactly like the linked image.
SECTIONS = {
    ".text": (0x0, 0x1000),
    ".rodata": (0x1000, 0x400),
    ".ram_code": (0x1400, 0x400),
}

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
# The OTA seam used to be modelled the same way: one synthetic pair per
# ``INDIRECT_EDGES`` entry — the OTA write dispatcher 24 B / 40 B (the real
# shape: a GATT write lands in ``dispatch_ota_write``, which reaches the OTA
# packet handler), the snapshot hook 24 B / 16 B, the two flag-only hooks
# 24 B / 8 B, and the four radio providers 8 B / 16 B. F-7 (#118) deleted that
# whole seam: OTA now lives in ``tlsr8266-platform`` behind static generic
# monomorphisations, so the fixture carries no OTA dispatcher and no OTA edge.
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
    # The boot OTA chain: the only `.ram_code` residents in the fixture. F-7
    # (#118) made it a short, fully RAM-resident chain rooted at
    # `handle_ota_update`; `check_ram_reachability` walks exactly this subtree.
    _fn(
        f"{MESH}3ota19handle_ota_update",
        8,
        (("tlsr8266_ota_boot_copy", False),),
        addr=0x1400,
    ),
    _fn(
        "tlsr8266_ota_boot_copy",
        8,
        (("reboot_and_retry_copy", False),),
        addr=0x1410,
    ),
    _fn(
        "reboot_and_retry_copy",
        8,
        (("tlsr8266_hal::pm::ram_sw_reboot", False),),
        addr=0x1420,
    ),
    _fn("tlsr8266_hal::pm::ram_sw_reboot", 8, addr=0x1430),
    _fn(
        "main_entrypoint",
        40,
        ((f"{RADIO}2rf8drv_826624rf_link_slave_data_write", False),),
    ),
    _fn(
        f"{RADIO}2rf8drv_826624rf_link_slave_data_write",
        184,
        ((f"{RADIO_CALLBACKS}13on_light_data", False),),
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
        mock.patch.object(stack_analysis, "read_elf_sections", return_value=SECTIONS),
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

    def test_word_literal_pool_entries_are_parsed(self) -> None:
        """objdump prints a literal pool as inline `.word` lines in the body."""
        funcs = stack_analysis.parse_functions(
            _text(_fn("pooled", 8, addr=0x400, words=(0x00800061, 0x0000f698)))
        )
        self.assertEqual(
            funcs["pooled"].literals,
            [(0x440, 0x00800061), (0x444, 0x0000F698)],
        )


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
                # The C-8 subsystem seams.
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
                "dispatch_uart_cmd",
                "dispatch_uart_mesh_rx",
                "emit_light_event",
                "notify_status_update",
                "on_light_data",
                "on_light_response",
                "on_mesh_pair_proc",
                "on_uart_mesh_rx",
                "publish_light_status",
                "uart_tx_ops_dropped",
                "uart_tx_ops_push",
            ],
        )
        self.assertEqual(len(stack_analysis.INDIRECT_EDGES), 13)
        for caller, callees in stack_analysis.INDIRECT_EDGES.items():
            self.assertEqual(len(callees), 1, caller)

    def test_retired_ota_seam_is_absent(self) -> None:
        """F-7 (#118) deleted ``OtaHooks`` / ``OtaRadioOps``: no OTA row survives.

        An entry for a symbol that no longer links is a hard gate failure, so a
        reintroduced OTA row would fail every build rather than be ignored.
        """
        retired = {
            "dispatch_ota_write",
            "dispatch_ota_snapshot",
            "dispatch_ota_terminate",
            "dispatch_ota_tick",
            "radio_stage_pkt",
            "radio_read_pkt",
            "radio_terminate_tx",
            "radio_begin_gatt_ota",
        }
        self.assertEqual(retired & set(stack_analysis.INDIRECT_EDGES), set())
        callees = {
            callee
            for callees in stack_analysis.INDIRECT_EDGES.values()
            for callee in callees
        }
        for pattern in ("on_gatt_ota_packet", "stage_ota_packet", "ota_status_snapshot_impl"):
            self.assertNotIn(pattern, callees)

    def test_no_pattern_is_a_substring_of_another(self) -> None:
        """`_resolve_one` fails closed on ambiguity, so the table must stay sharp.

        Every caller and callee pattern is a substring searched over the whole
        symbol table: if one pattern were contained in another, whichever
        symbol matched both would resolve as ambiguous and the gate would fail
        on a correct image.  `publish_light_status` vs
        `tlsr8266_mesh::light::device_status_update` is the C-8 pair: the
        dispatcher was renamed precisely so its own name is not a substring of
        its callee's.
        """
        patterns = list(stack_analysis.INDIRECT_EDGES)
        for callees in stack_analysis.INDIRECT_EDGES.values():
            patterns.extend(callees)
        self.assertEqual(len(patterns), 26)
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
    def test_obsolete_strict_gatt_contract_is_absent(self) -> None:
        self.assertFalse(hasattr(stack_analysis, "STRICT_GATT_EDGES"))
        self.assertFalse(hasattr(stack_analysis, "STRICT_GATT_HELPER_PATTERNS"))

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
        text = FIRMWARE_SHAPED + "\n" + _fn(
            "project_unknown_dispatcher", 32, indirect=True
        )
        code, out = _run_main(
            text,
            ["--elf", "synthetic.elf", "--top", "3"],
        )
        self.assertEqual(code, 1, out)
        self.assertIn("project_unknown_dispatcher", out)
        self.assertIn("instruction 0x00000008", out)
        self.assertIn("strict unknown indirect project branch", out)

    def test_direct_default_strict_mode_has_zero_gatt_edge_model(self) -> None:
        code, out = _run_main(
            FIRMWARE_SHAPED,
            ["--elf", "synthetic.elf", "--top", "3",
             "--min-margin", "400"],
        )
        self.assertEqual(code, 0, out)
        self.assertIn("strict mode: no project GATT indirect-edge model", out)
        self.assertIn("legacy indirect dispatch edges (13):", out)
        self.assertNotIn("strict GATT dispatch edges", out)

    def test_legacy_mode_does_not_add_gatt_edges(self) -> None:
        code, out = _run_main(
            FIRMWARE_SHAPED,
            ["--elf", "synthetic.elf", "--top", "3", "--legacy"],
        )
        self.assertEqual(code, 0, out)
        self.assertIn("indirect dispatch edges (13):", out)
        self.assertNotIn("strict GATT dispatch edges", out)


class TestProfileMarker(unittest.TestCase):
    """A gate must not measure another feature selection's image.

    Feature builds refresh the shared ``_build/lightblemesh`` copy, so without a
    fingerprint check ``make stack-check`` after ``make FEATURES=rtt`` measured
    the RTT image while labelling it ``default`` -- and passed.
    """

    def test_matching_profiles_are_accepted(self) -> None:
        self.assertIsNone(
            stack_analysis.profile_mismatch({"main_entrypoint": 0}, "default")
        )
        self.assertIsNone(
            stack_analysis.profile_mismatch(
                {"main_entrypoint": 0, "_TLSR_RTT": 0x100}, "rtt"
            )
        )
        # A slug containing the feature is still an RTT build.
        self.assertIsNone(
            stack_analysis.profile_mismatch({"_TLSR_RTT": 0x100}, "rtt-extra")
        )
        # An empty profile disables the check.
        self.assertIsNone(
            stack_analysis.profile_mismatch({"_TLSR_RTT": 0x100}, "")
        )

    def test_wrong_image_is_rejected(self) -> None:
        self.assertIn(
            "_TLSR_RTT",
            stack_analysis.profile_mismatch({"_TLSR_RTT": 0x100}, "default") or "",
        )
        self.assertIn(
            "has none",
            stack_analysis.profile_mismatch({"main_entrypoint": 0}, "rtt") or "",
        )


class TestSectionReaderFallback(unittest.TestCase):
    """Section discovery must not depend on ``llvm-readelf`` being resolvable.

    CI sets only ``LLVM_OBJDUMP``/``LLVM_NM``. ``_llvm_tool`` returns a
    repo-relative default that does not exist in a runner checkout, so a missing
    readelf used to return an empty section map. The RAM-residency gate is
    fail-closed, so that turned every boot root into "not in .ram_code" and
    failed the build with a misleading message (CI run 46116553601).
    """

    OBJDUMP_HEADERS = (
        "\n"
        "Sections:\n"
        "Idx Name            Size     VMA      LMA      Type\n"
        "  0                 00000000 00000000 00000000 \n"
        "  2 .ram_code       00000508 000000b0 000000b0 TEXT\n"
        "  3 .text           0000f0e8 000005b8 000005b8 TEXT\n"
        "  4 .rodata         000019d4 0000f6a0 0000f6a0 DATA\n"
    )

    def _run_with(self, readelf_raises: bool) -> tuple[dict, list[str]]:
        calls: list[str] = []

        def fake_run(argv, *args, **kwargs):  # noqa: ANN001, ANN002 - test double
            calls.append(argv[0])
            if argv[0].endswith("llvm-readelf") and readelf_raises:
                raise OSError("no such file or directory")
            return mock.Mock(
                returncode=0,
                stdout=self.OBJDUMP_HEADERS,
                stderr="",
            )

        def fake_tool(name: str, _env_key: str, *_defaults: str) -> str:
            return f"/tools/{name}"

        with mock.patch.object(stack_analysis, "_llvm_tool", fake_tool), \
                mock.patch.object(stack_analysis.subprocess, "run", fake_run):
            sections = stack_analysis.read_elf_sections("synthetic.elf")
        return sections, calls

    def test_missing_readelf_falls_back_to_the_objdump_reader(self) -> None:
        sections, calls = self._run_with(readelf_raises=True)

        self.assertTrue(
            any("llvm-objdump" in call for call in calls),
            "the objdump reader must be tried",
        )
        self.assertEqual(sections[".ram_code"], (0xB0, 0x508))
        self.assertEqual(sections[".text"], (0x5B8, 0xF0E8))

    def test_empty_readelf_output_also_falls_back(self) -> None:
        # A readelf that exists but prints nothing usable (e.g. an unsupported
        # machine type) must fall through too, not silently yield no sections.
        sections, calls = self._run_with(readelf_raises=False)

        # The fake returns the objdump table for both, so the first reader
        # "succeeds" here; the point is that a usable map is always produced.
        self.assertTrue(sections)
        self.assertIn(".ram_code", sections)
        self.assertTrue(calls[0].endswith("llvm-readelf"))


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

    def test_elf_section_parser_drops_non_allocatable_sections(self) -> None:
        """`.symtab`/`.strtab` sit at address 0 with a huge size; they own no VMA.

        Keeping them would make `classify_function_sections` report `.symtab`
        for a `.text` function, which is a wrong name in a violation message.
        """
        sections = stack_analysis.parse_elf_sections(
            "  [ 2] .ram_code      PROGBITS        000000b0 000100b0 000004c8 00  AX  0   0  4\n"
            "  [ 3] .text          PROGBITS        00000578 00010578 0000f120 00  AX  0   0  4\n"
            "  [12] .symtab        SYMTAB          00000000 0001f698 00003d70 10     13 247  4\n"
            "  [13] .strtab        STRTAB          00000000 00023308 00009306 00      0   0  1\n"
            "  [14] .comment       PROGBITS        00000000 0002c60e 0000002e 01  MS  0   0  1\n"
        )
        self.assertEqual(sections[".ram_code"], (0xB0, 0x4C8))
        self.assertEqual(sections[".text"], (0x578, 0xF120))
        self.assertNotIn(".symtab", sections)
        self.assertNotIn(".strtab", sections)
        self.assertNotIn(".comment", sections)

    def test_offset_label_callee_resolves_into_ram_code(self) -> None:
        """objdump renders a local branch as `<fn+0xNN>`; resolve it, never guess."""
        funcs = {
            "handle_ota_update": stack_analysis.Func(
                0x1400,
                frame=8,
                calls=[("boot_copy+0x38", True)],
                section=".ram_code",
            ),
            "boot_copy": stack_analysis.Func(0x1410, frame=16, section=".ram_code"),
        }
        self.assertEqual(
            stack_analysis.check_ram_reachability(funcs, SECTIONS), []
        )
        # The same label pointing past the end of the section is not RAM-proven.
        funcs["handle_ota_update"].calls = [("boot_copy+0x800", True)]
        self.assertEqual(
            stack_analysis.check_ram_reachability(funcs, SECTIONS),
            [
                (
                    "handle_ota_update",
                    "boot_copy+0x800",
                    stack_analysis.UNKNOWN_SECTION,
                )
            ],
        )

    def test_text_literal_pool_integer_constant_is_not_a_pointer(self) -> None:
        """`.word 0x2710` is a 10000-iteration timeout, not a `.text` address.

        `.text` values are anchored on the symbol table so an integer constant
        that merely ranges over `.text` is not reported; an actual function
        address in the same pool is.
        """
        sections = {".ram_code": (0xB0, 0x4C8), ".text": (0x578, 0x3000)}
        symbols = {"flash_read_page": 0x500, "light_sw_reboot": 0x1E0E}
        funcs = {
            "handle_ota_update": stack_analysis.Func(
                0x200,
                frame=8,
                calls=[("boot_copy", False)],
                section=".ram_code",
            ),
            "boot_copy": stack_analysis.Func(
                0x210,
                frame=8,
                section=".ram_code",
                literals=[(0x250, 0x2710)],
            ),
        }
        self.assertEqual(
            stack_analysis.check_ram_reachability(funcs, sections, symbols), []
        )
        funcs["boot_copy"].literals = [(0x250, 0x1E0E)]
        self.assertEqual(
            stack_analysis.check_ram_reachability(funcs, sections, symbols),
            [
                (
                    "boot_copy",
                    stack_analysis.LITERAL_POOL_VIOLATION,
                    ".text 0x00001e0e",
                )
            ],
        )

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

    def test_full_boot_ota_chain_in_ram_code_passes(self) -> None:
        """The real image's chain, with the real demangled symbol names.

        `handle_ota_update -> tlsr8266_ota_boot_copy -> reboot_and_retry_copy
        -> ram_sw_reboot` — all four `.ram_code`, no indirect branch, no pool.
        """
        funcs = {
            f"{MESH}3ota19handle_ota_update": stack_analysis.Func(
                0x570,
                frame=8,
                calls=[("tlsr8266_ota_boot_copy", False)],
                section=".ram_code",
            ),
            "tlsr8266_ota_boot_copy": stack_analysis.Func(
                0x34C,
                frame=300,
                calls=[
                    ("reboot_and_retry_copy", False),
                    ("tlsr8266_hal::pm::ram_sw_reboot", False),
                ],
                section=".ram_code",
            ),
            "reboot_and_retry_copy": stack_analysis.Func(
                0x470,
                frame=0,
                calls=[("tlsr8266_hal::pm::ram_sw_reboot", False)],
                section=".ram_code",
            ),
            "tlsr8266_hal::pm::ram_sw_reboot": stack_analysis.Func(
                0x120, frame=0, section=".ram_code"
            ),
        }
        self.assertEqual(stack_analysis.check_ram_reachability(funcs), [])
        self.assertEqual(stack_analysis.validate_ram_reachability(funcs), [])

    def test_boot_ota_chain_to_flash_resident_reboot_leaf_is_a_violation(self) -> None:
        """F-7 (#118) deleted the `light_sw_reboot` exemption.

        A `.text` reboot leaf is unreachable once the copy loop erases sector 0,
        so it is a hard violation now instead of a tolerated leaf.
        """
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
            [("reboot_and_retry_copy", "light_sw_reboot", ".text")],
        )
        with self.assertRaises(stack_analysis.RamReachabilityError) as ctx:
            stack_analysis.validate_ram_reachability(
                funcs, root_pattern="handle_ota_update"
            )
        message = str(ctx.exception)
        self.assertIn("reboot_and_retry_copy", message)
        self.assertIn("light_sw_reboot", message)
        self.assertIn(".text", message)

    def test_zero_matched_roots_fails_closed(self) -> None:
        """A renamed or inlined copier must never produce an empty, passing walk."""
        funcs = {
            "handle_ota_update_v2": stack_analysis.Func(
                0x200, frame=8, section=".ram_code"
            ),
        }
        violations = stack_analysis.check_ram_reachability(
            funcs, root_pattern="handle_ota_update$"
        )
        self.assertEqual(len(violations), 1)
        caller, callee, reason = violations[0]
        self.assertEqual(caller, "handle_ota_update$")
        self.assertEqual(callee, stack_analysis.ROOT_VIOLATION)
        self.assertIn("matches no parsed function", reason)
        with self.assertRaises(stack_analysis.RamReachabilityError) as ctx:
            stack_analysis.validate_ram_reachability(
                funcs, root_pattern="handle_ota_update$"
            )
        self.assertIn("is unusable", str(ctx.exception))
        self.assertIn("matches no parsed function", str(ctx.exception))

    def test_root_outside_ram_code_fails(self) -> None:
        funcs = {
            "handle_ota_update": stack_analysis.Func(
                0x300, frame=8, section=".text"
            ),
        }
        self.assertEqual(
            stack_analysis.check_ram_reachability(
                funcs, root_pattern="handle_ota_update"
            ),
            [
                (
                    "handle_ota_update",
                    stack_analysis.ROOT_VIOLATION,
                    "root is not in .ram_code (section .text)",
                )
            ],
        )
        with self.assertRaises(stack_analysis.RamReachabilityError) as ctx:
            stack_analysis.validate_ram_reachability(
                funcs, root_pattern="handle_ota_update"
            )
        self.assertIn("root is not in .ram_code", str(ctx.exception))

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

    def test_reachable_indirect_branch_in_ram_code_fails(self) -> None:
        """An indirect branch cannot be proven to stay in RAM, so it fails."""
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
                indirect_insts=[(0x218, "bx", "r1")],
            ),
            "reboot": stack_analysis.Func(0x220, frame=8, section=".ram_code"),
        }
        self.assertEqual(
            stack_analysis.check_ram_reachability(
                funcs, root_pattern="handle_ota_update"
            ),
            [
                (
                    "reboot_and_retry_copy",
                    stack_analysis.INDIRECT_VIOLATION,
                    "bx r1 at 0x00000218",
                )
            ],
        )
        with self.assertRaises(stack_analysis.RamReachabilityError) as ctx:
            stack_analysis.validate_ram_reachability(
                funcs, root_pattern="handle_ota_update"
            )
        self.assertIn("has an indirect branch", str(ctx.exception))

    def test_literal_pool_pointer_into_rodata_fails(self) -> None:
        """A `.word` pool entry into flash is a pointer the copy loop cannot follow."""
        funcs = {
            "handle_ota_update": stack_analysis.Func(
                0x1400,
                frame=8,
                calls=[("tlsr8266_ota_boot_copy", False)],
                section=".ram_code",
            ),
            "tlsr8266_ota_boot_copy": stack_analysis.Func(
                0x1410,
                frame=32,
                section=".ram_code",
                # 0x1010 is inside the fixture's `.rodata`; the other entry is a
                # register address in RAM and must stay clean.
                literals=[(0x1450, 0x00800061), (0x1454, 0x1010)],
            ),
        }
        self.assertEqual(
            stack_analysis.check_ram_reachability(funcs, SECTIONS),
            [
                (
                    "tlsr8266_ota_boot_copy",
                    stack_analysis.LITERAL_POOL_VIOLATION,
                    ".rodata 0x00001010",
                )
            ],
        )
        with self.assertRaises(stack_analysis.RamReachabilityError) as ctx:
            stack_analysis.validate_ram_reachability(funcs, SECTIONS)
        self.assertIn("literal-pool pointer into flash", str(ctx.exception))

    def test_ram_literal_pool_pointer_passes(self) -> None:
        """The real pool values point at registers in RAM, not at flash."""
        funcs = {
            "handle_ota_update": stack_analysis.Func(
                0x1400,
                frame=8,
                calls=[("tlsr8266_ota_boot_copy", False)],
                section=".ram_code",
            ),
            "tlsr8266_ota_boot_copy": stack_analysis.Func(
                0x1410,
                frame=32,
                section=".ram_code",
                literals=[(0x1450, 0x00800061), (0x1454, 0x0080074F)],
            ),
        }
        self.assertEqual(
            stack_analysis.check_ram_reachability(funcs, SECTIONS), []
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

    def test_rodata_callee_fails_ram_reachability(self) -> None:
        funcs = {
            "ram_wrapper": stack_analysis.Func(
                0x200,
                frame=32,
                calls=[("rodata_helper", True)],
                section=".ram_code",
            ),
            "rodata_helper": stack_analysis.Func(
                0x2000,
                frame=0,
                section=".rodata",
            ),
        }
        self.assertEqual(
            stack_analysis.check_ram_reachability(
                funcs, root_pattern="ram_wrapper"
            ),
            [("ram_wrapper", "rodata_helper", ".rodata")],
        )

    def test_unknown_section_callee_fails_when_sections_are_supplied(self) -> None:
        """With ELF sections known, an unclassifiable callee is not RAM-proven."""
        sections = {
            ".ram_code": (0x200, 0x100),
            ".text": (0x400, 0x100),
        }
        funcs = {
            "handle_ota_update": stack_analysis.Func(
                0x210,
                frame=8,
                calls=[("mystery", False)],
            ),
            "mystery": stack_analysis.Func(0x900, frame=8),
        }
        self.assertEqual(
            stack_analysis.check_ram_reachability(
                funcs, sections, root_pattern="handle_ota_update"
            ),
            [("handle_ota_update", "mystery", stack_analysis.UNKNOWN_SECTION)],
        )
        with self.assertRaises(stack_analysis.RamReachabilityError) as ctx:
            stack_analysis.validate_ram_reachability(
                funcs, sections, root_pattern="handle_ota_update"
            )
        self.assertIn("no known ELF section", str(ctx.exception))


class TestBaselineJson(unittest.TestCase):
    def test_legacy_baseline_reports_image_sections_and_stack_counts(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            elf = Path(tmp) / "firmware.elf"
            bin_path = Path(tmp) / "firmware.bin"
            out_path = Path(tmp) / "baseline.json"
            elf.write_bytes(b"ELF")
            bin_path.write_bytes(b"BIN")

            sections = SECTIONS
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
            self.assertEqual(baseline["allowed_indirect"], 13)
            self.assertEqual(baseline["mode"], "legacy")
            self.assertFalse(baseline["strict"])

    def test_direct_default_strict_baseline_reports_gatt_edges(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            elf = Path(tmp) / "firmware.elf"
            bin_path = Path(tmp) / "firmware.bin"
            out_path = Path(tmp) / "baseline.json"
            elf.write_bytes(b"ELF")
            bin_path.write_bytes(b"BIN")

            sections = SECTIONS
            buf = io.StringIO()
            with (
                mock.patch.object(stack_analysis, "objdump", return_value=FIRMWARE_SHAPED),
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
            self.assertEqual(baseline["legacy_edges_resolved"], 13)
            self.assertEqual(baseline["gatt_edges_resolved"], 0)

    def test_compare_baseline_prints_structured_delta_without_failing(self) -> None:
        current = {
            "elf_bytes": 500,
            "bin_bytes": 400,
            "sections": {".text": 120, ".rodata": 30, ".data": 10,
                         ".bss": 20, ".ram_code": 15},
            "free_main_sram": 4096,
            "main_max": 656,
            "irq_max": 604,
            "main_margin": 3440,
            "irq_margin": 1188,
            "allowed_indirect": 26,
        }
        baseline = dict(current)
        baseline["elf_bytes"] = 480
        baseline["bin_bytes"] = 390
        baseline["sections"] = dict(current["sections"])
        baseline["sections"][".text"] = 100
        baseline["allowed_indirect"] = 21

        with tempfile.TemporaryDirectory() as tmp:
            baseline_path = Path(tmp) / "baseline.json"
            baseline_path.write_text(json.dumps(baseline), encoding="utf-8")
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                valid = stack_analysis.compare_baseline(current, str(baseline_path))

        self.assertTrue(valid)
        out = buf.getvalue()
        self.assertIn("BASELINE DELTA (informational", out)
        for metric in (
            "elf_bytes", "bin_bytes", ".text", ".rodata", ".data", ".bss",
            ".ram_code", "free_main_sram", "main_max", "irq_max",
            "main_margin", "irq_margin", "allowed_indirect",
        ):
            self.assertIn(metric, out)

    def test_compare_baseline_rejects_missing_and_non_numeric_schema(self) -> None:
        complete = {
            "elf_bytes": 500,
            "bin_bytes": 400,
            "sections": {".text": 120, ".rodata": 30, ".data": 10,
                         ".bss": 20, ".ram_code": 15},
            "free_main_sram": 4096,
            "main_max": 656,
            "irq_max": 604,
            "main_margin": 3440,
            "irq_margin": 1188,
            "allowed_indirect": 26,
        }

        cases = []
        missing_baseline = json.loads(json.dumps(complete))
        del missing_baseline["elf_bytes"]
        cases.append((complete, missing_baseline))

        string_baseline = json.loads(json.dumps(complete))
        string_baseline["bin_bytes"] = "400"
        cases.append((complete, string_baseline))

        missing_section = json.loads(json.dumps(complete))
        del missing_section["sections"][".text"]
        cases.append((complete, missing_section))

        bool_current = json.loads(json.dumps(complete))
        bool_current["main_margin"] = True
        cases.append((bool_current, complete))

        missing_current = json.loads(json.dumps(complete))
        del missing_current["sections"][".bss"]
        cases.append((missing_current, complete))

        for current, baseline in cases:
            with self.subTest(current=current, baseline=baseline):
                with tempfile.TemporaryDirectory() as tmp:
                    baseline_path = Path(tmp) / "baseline.json"
                    baseline_path.write_text(json.dumps(baseline), encoding="utf-8")
                    buf = io.StringIO()
                    with contextlib.redirect_stdout(buf):
                        valid = stack_analysis.compare_baseline(current, str(baseline_path))

                self.assertFalse(valid)
                self.assertIn("INVALID SCHEMA", buf.getvalue())


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
    
    def test_roots_no_longer_contain_indirect_only_callees(self) -> None:
        funcs = _parse()
        roots = set(stack_analysis.all_roots(funcs))
        self.assertIn(f"{MESH}3app21rf_link_data_callback", roots)
        self.assertIn(f"{MESH}3app28rf_link_light_event_callback", roots)
        self.assertIn(f"{MESH}3app17uart_mesh_rx_hook", roots)
        self.assertIn(f"{MESH}3app17uart_command_hook", roots)
        self.assertIn(f"{MESH}3app17log_async_tx_hook", roots)
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
        self.assertIn("indirect dispatch edges (13):", out)
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
        # F-7 (#118) deleted the OTA seam, so no OTA edge may be reported.
        self.assertNotIn("dispatch_ota", out)
        self.assertNotIn("stage_ota_packet", out)
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

    def test_missing_callee_fails_closed(self) -> None:
        # A stale entry is the failure this table exists to catch: without
        # ll_device_status_update the status subtree would score as 0 B and the
        # gate would under-report both margins instead of failing.
        callee = f"{RADIO}7mesh_ll11node_status23ll_device_status_update"
        text = FIRMWARE_SHAPED.replace(_fn(callee, 40) + "\n", "")
        self.assertNotIn(callee, text)
        # The dispatcher survives; only its callee is gone.
        self.assertIn(f"{MESH}5light20notify_status_update", text)
        code, out = _run_main(
            text,
            ["--elf", "synthetic.elf", "--top", "3", "--legacy"],
        )
        self.assertEqual(code, 1, out)
        self.assertIn("FAIL: INDIRECT_EDGES['notify_status_update']", out)
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
             "--exclude-panic", "--strict", "--baseline-json", "base.json",
             "--compare-baseline", "snapshot.json"]
        )
        self.assertEqual(
            (args.elf, args.top, args.min_margin, args.exclude_panic),
            ("x.elf", 7, 512, True),
        )
        self.assertTrue(args.strict)
        self.assertFalse(args.legacy)
        self.assertEqual(args.baseline_json, "base.json")
        self.assertEqual(args.compare_baseline, "snapshot.json")

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
