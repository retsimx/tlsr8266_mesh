#!/usr/bin/env python3
"""Shared loader for the committed wire-contract golden fixture.

The Python half of the golden-vector suite described in
``docs/plans/designs/010-r2-wire-contract-golden-tests.md``. It parses the
committed fixture ``tests/wire_vectors.txt`` -- the same file the Rust firmware
suite reads -- for the ``test_wire_contract_*`` namespace modules:

- ``test_wire_contract_wire.py``: CRC-16-MODBUS, UART framing/opcodes, GATT
  UUIDs and pairing constants.
- ``test_wire_contract_crypto.py``: session-key derivation, AES-128
  hardware-parity crypto and the 20-byte command vectors.
- ``test_wire_contract_firmware.py``: the 32-byte firmware header, OTA flash
  layout and the BLE/UART OTA packet framing.

``test_wire_contracts.py`` holds the orphan guard that proves every shared
fixture key is load-bearing.

The fixture is the contract. If a test fails, the protocol drifted; fix the
code (or regenerate the fixture deliberately), never the assertion.

Run from the repository root:

    PYTHONPATH=utilities/meshutils python3 -m unittest discover \\
        -s utilities/meshutils/tests -v

The filename does not match unittest's ``test*.py`` pattern, so this module is
never collected as a test module itself.

Import-time note: ``flash_fw`` needs ``bleak`` and ``flash_fw_uart`` needs
``pyserial``, so those distributions must be installed even though no test
opens a BLE or serial connection.
"""

import re
from pathlib import Path

FIXTURE_PATH = Path(__file__).resolve().parents[3] / "tests" / "wire_vectors.txt"

# Keys asserted only by the Rust suite: Python has no struct layout (``size.*``)
# and no equivalent of the two firmware-only pairing opcodes.
RUST_ONLY_KEY_PREFIXES = ("size.",)
RUST_ONLY_KEYS = frozenset(
    {
        "pair.op.reset_mesh",
        "pair.op.delete_pairing",
    }
)

_HEX_BYTE = re.compile(r"[0-9a-f]{2}")


class FixtureError(Exception):
    """The shared fixture is malformed, or a test asked for a missing key."""


def _read_fixture(path):
    """Parse ``key = value`` lines into a dict, rejecting duplicates."""
    values = {}
    for lineno, raw in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        key, separator, value = line.partition("=")
        if not separator:
            raise FixtureError(f"{path}:{lineno}: expected 'key = value', got {raw!r}")
        key = key.strip()
        value = value.strip()
        if not key:
            raise FixtureError(f"{path}:{lineno}: empty key in {raw!r}")
        if key in values:
            raise FixtureError(f"{path}:{lineno}: duplicate key {key!r}")
        values[key] = value
    return values


# Keys read through an accessor so far. Module-level rather than per-instance so
# the orphan guard sees every read regardless of which ``Fixture`` object did it.
CONSUMED_KEYS = set()


class Fixture:
    """Read-only fixture view; every accessor records the key it read."""

    def __init__(self, path):
        self.path = path
        self._values = _read_fixture(path)

    def as_str(self, key):
        """Return the raw value as a string."""
        return self._lookup(key)

    def as_u8(self, key):
        """Return the value as an unsigned 8-bit integer."""
        return self._as_int(key, 0xFF)

    def as_u32(self, key):
        """Return the value as an unsigned 32-bit integer."""
        return self._as_int(key, 0xFFFFFFFF)

    def as_bytes(self, key):
        """Return the value as bytes parsed from space-separated hex pairs."""
        text = self._lookup(key)
        tokens = text.split()
        if not tokens:
            raise FixtureError(f"{key}: expected hex bytes, got an empty value")
        value = bytearray()
        for token in tokens:
            if not _HEX_BYTE.fullmatch(token):
                raise FixtureError(
                    f"{key}: {token!r} is not a lowercase 2-hex-digit byte"
                )
            value.append(int(token, 16))
        return bytes(value)

    def keys(self):
        """All keys present in the fixture."""
        return frozenset(self._values)

    def _as_int(self, key, maximum):
        text = self._lookup(key)
        try:
            base = 16 if text.lower().startswith("0x") else 10
            value = int(text, base)
        except ValueError:
            raise FixtureError(f"{key}: expected an integer, got {text!r}") from None
        if not 0 <= value <= maximum:
            raise FixtureError(f"{key}: {value} does not fit in 0..{maximum}")
        return value

    def _lookup(self, key):
        try:
            value = self._values[key]
        except KeyError:
            raise FixtureError(f"{key}: no such fixture key") from None
        CONSUMED_KEYS.add(key)
        return value


FIXTURE = Fixture(FIXTURE_PATH)
