# `picow_swire` — retired MicroPython SWire client

The MicroPython SWire client (`client.py`, `remote.py`, `__init__.py`) and its
pinned requirements (`requirements.in`, `requirements.txt`) were removed in
issue #24. SWire work now uses `tools/probe/`: the `probe-pico` RP2040 firmware
plus the `probe-tlsr` host CLI. See [docs/swire.md](../../docs/swire.md) for the
user guide.

The two binaries kept here are original OEM firmware dumps, retained for
recovery: `TYBT1_original_fw.bin` (TYBT1) and `e104.bin` (E104-BT05-TB).
