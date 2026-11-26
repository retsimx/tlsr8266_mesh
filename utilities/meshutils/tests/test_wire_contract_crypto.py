#!/usr/bin/env python3
"""Crypto namespace: session-key derivation, AES blocks/packets, command vectors.

Part of the Python half of the golden-vector suite; see
``wire_contract_fixture`` for the shared fixture loader and how to run it.
"""

import unittest

import mesh_common

from wire_contract_fixture import FIXTURE, FixtureError


def aes_packet_vector(number):
    """Return the derived fields of the ``aes.packet.N`` vector.

    ``mac_fwd``/``sno``/``src`` are reconstructed the way the firmware builds
    them: the notification's IV is ``mac[0:3] | sno(3) | src(2)`` and
    ``decrypt_notification`` wants the MAC in display order.
    """
    ivm = FIXTURE.as_bytes(f"aes.packet.{number}.ivm")
    if len(ivm) != 8:
        raise FixtureError(f"aes.packet.{number}.ivm: expected 8 bytes, got {len(ivm)}")
    return {
        "key": FIXTURE.as_bytes(f"aes.packet.{number}.key"),
        "plain": FIXTURE.as_bytes(f"aes.packet.{number}.plain"),
        "cipher": FIXTURE.as_bytes(f"aes.packet.{number}.cipher"),
        "mic": FIXTURE.as_bytes(f"aes.packet.{number}.mic"),
        "sno": ivm[3:6],
        "src": ivm[6:8],
        "mac_fwd": b"\x00\x00\x00" + bytes(reversed(ivm[0:3])),
    }


def cmd_action_vector(number):
    """Return the fields of the ``cmd_action.N`` command-encryption vector."""
    session_key = FIXTURE.as_bytes(f"cmd_action.{number}.session_key")
    nonce = FIXTURE.as_bytes(f"cmd_action.{number}.nonce")
    plain_cmd = FIXTURE.as_bytes(f"cmd_action.{number}.plain_cmd")
    encrypted_cmd = FIXTURE.as_bytes(f"cmd_action.{number}.encrypted_cmd")
    if len(session_key) != mesh_common.KEY_SIZE:
        raise FixtureError(
            f"cmd_action.{number}.session_key: expected {mesh_common.KEY_SIZE} bytes"
        )
    if len(nonce) != 8:
        raise FixtureError(f"cmd_action.{number}.nonce: expected 8 bytes")
    if len(plain_cmd) != 20 or len(encrypted_cmd) != 20:
        raise FixtureError(f"cmd_action.{number}: expected 20-byte commands")
    return {
        "session_key": session_key,
        "nonce": nonce,
        "plain_cmd": plain_cmd,
        "encrypted_cmd": encrypted_cmd,
    }


class SessionKeyTests(unittest.TestCase):
    """``process_session_key`` checks the server response, then derives the key.

    ``mesh_common.process_session_key`` reads the **module global**
    ``shared_key`` (the client random, ``pair_randm``) rather than taking it as
    an argument. This test therefore saves the import-time value, substitutes
    the fixture's, and restores it in ``tearDown`` *and* in a ``try``/``finally``
    around the calls: a failed assertion must not leak the substituted global
    into every later test in the same process.
    """

    def setUp(self):
        self._saved_shared_key = mesh_common.shared_key

    def tearDown(self):
        mesh_common.shared_key = self._saved_shared_key

    def test_derivation(self):
        shared_key = FIXTURE.as_bytes("pair.session_key.shared_key")
        credentials = FIXTURE.as_bytes("pair.session_key.credentials")
        response = FIXTURE.as_bytes("pair.session_key.response")
        expected = FIXTURE.as_bytes("pair.session_key.session_key")
        invalid = FIXTURE.as_bytes("pair.session_key.response_invalid")

        self.assertEqual(len(shared_key), mesh_common.RANDOM_CHALLENGE_SIZE)
        self.assertEqual(len(credentials), mesh_common.KEY_SIZE)
        self.assertEqual(len(response), mesh_common.KEY_SIZE + 1)
        self.assertEqual(len(expected), mesh_common.KEY_SIZE)
        self.assertEqual(len(invalid), len(response))

        mesh_common.shared_key = shared_key
        try:
            self.assertEqual(
                bytes(
                    mesh_common.process_session_key(response, credentials)
                ),
                expected,
            )
            # A flipped byte in the server result must be rejected, not derived.
            with self.assertRaises(Exception):
                mesh_common.process_session_key(invalid, credentials)
        finally:
            mesh_common.shared_key = self._saved_shared_key


class AesBlockTests(unittest.TestCase):
    """AES-128 hardware-parity block encryption and its inverse."""

    def test_round_trip(self):
        for number in range(1, 9):
            key = FIXTURE.as_bytes(f"aes.block.{number}.key")
            plain = FIXTURE.as_bytes(f"aes.block.{number}.plain")
            cipher = FIXTURE.as_bytes(f"aes.block.{number}.cipher")
            with self.subTest(block=number):
                self.assertEqual(len(key), mesh_common.KEY_SIZE)
                self.assertEqual(len(plain), mesh_common.KEY_SIZE)
                self.assertEqual(len(cipher), mesh_common.KEY_SIZE)
                self.assertEqual(mesh_common.aes_att_encryption(key, plain), cipher)
                self.assertEqual(mesh_common.aes_att_decryption(key, cipher), plain)


class AesPacketTests(unittest.TestCase):
    """Authenticated notifications: CTR decryption plus CBC-MAC verification."""

    def test_decrypt_notification(self):
        for number in (1, 2):
            vector = aes_packet_vector(number)
            with self.subTest(packet=number):
                self.assertEqual(len(vector["mic"]), 2)
                data = (
                    vector["sno"]
                    + vector["src"]
                    + vector["mic"]
                    + vector["cipher"]
                )
                self.assertEqual(
                    mesh_common.decrypt_notification(
                        data, vector["key"], vector["mac_fwd"]
                    ),
                    vector["plain"],
                )

    def test_corrupted_mic_is_rejected(self):
        for number in (1, 2):
            vector = aes_packet_vector(number)
            bad_mic = bytearray(vector["mic"])
            bad_mic[0] ^= 0x01
            data = vector["sno"] + vector["src"] + bytes(bad_mic) + vector["cipher"]
            with self.subTest(packet=number):
                self.assertIsNone(
                    mesh_common.decrypt_notification(
                        data, vector["key"], vector["mac_fwd"]
                    )
                )

    def test_corrupted_ciphertext_is_rejected(self):
        for number in (1, 2):
            vector = aes_packet_vector(number)
            bad_cipher = bytearray(vector["cipher"])
            bad_cipher[0] ^= 0x01
            data = vector["sno"] + vector["src"] + vector["mic"] + bytes(bad_cipher)
            with self.subTest(packet=number):
                self.assertIsNone(
                    mesh_common.decrypt_notification(
                        data, vector["key"], vector["mac_fwd"]
                    )
                )


class CommandActionTests(unittest.TestCase):
    """``CommandAction.generate_data`` over the 20-byte command vectors.

    The fixture's ``encrypted_cmd`` is the production output for the paired
    ``plain_cmd``: ``generate_data`` writes the 2-byte MAC into bytes 3..5 and
    CTR-encrypts the 15-byte payload in place, so a fresh copy must reproduce it
    byte for byte.
    """

    def test_generate_data(self):
        for number in (1, 2):
            vector = cmd_action_vector(number)
            with self.subTest(vector=number):
                command = mesh_common.CommandAction(
                    mesh_common.BaseCommandAction(
                        session_key=list(vector["session_key"]),
                    )
                ).generate_data(
                    list(vector["session_key"]),
                    list(vector["nonce"]),
                    bytearray(vector["plain_cmd"]),
                )
                self.assertEqual(bytes(command), vector["encrypted_cmd"])


if __name__ == "__main__":
    unittest.main()
