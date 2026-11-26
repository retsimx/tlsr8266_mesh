#!/usr/bin/env python3
"""Fixture accounting: every shared key must be load-bearing for the suite.

The Python half of the golden-vector suite is split by namespace:

- ``test_wire_contract_wire.py`` (CRC, UART framing/opcodes, GATT, pairing),
- ``test_wire_contract_crypto.py`` (session key, AES blocks/packets, commands),
- ``test_wire_contract_firmware.py`` (image header, OTA flash layout, packets),

all reading the same fixture through ``wire_contract_fixture``. This module
holds only the orphan guard, and its filename sorts after the three namespace
modules, so unittest discovery consumes every namespace before the guard runs.
Run on its own, nothing has been consumed yet and the guard skips instead of
reporting every key as an orphan.
"""

import unittest

from wire_contract_fixture import (
    CONSUMED_KEYS,
    FIXTURE,
    RUST_ONLY_KEY_PREFIXES,
    RUST_ONLY_KEYS,
)


class ZzOrphanVectorTests(unittest.TestCase):
    """Every shared fixture key must be load-bearing for the Python suite.

    The accounting is :data:`CONSUMED_KEYS`, a module-level set every accessor
    populates, so the guard does not depend on one ``Fixture`` instance. The
    ``Zz`` prefix still keeps this class last within this module, but the check
    no longer *requires* that order: run on its own, nothing has been consumed
    yet and the guard skips instead of reporting every key as an orphan.
    """

    def test_no_orphan_vectors(self):
        if not CONSUMED_KEYS:
            self.skipTest("no fixture keys consumed yet; run the whole suite")

        all_keys = FIXTURE.keys()
        rust_only = {
            key for key in all_keys if key.startswith(RUST_ONLY_KEY_PREFIXES)
        } | RUST_ONLY_KEYS
        shared = all_keys - rust_only

        self.assertEqual(len(rust_only), 17)
        self.assertEqual(sorted(shared - CONSUMED_KEYS), [])
        self.assertEqual(sorted(CONSUMED_KEYS - shared), [])
        self.assertEqual(len(shared), 134)


if __name__ == "__main__":
    unittest.main()
