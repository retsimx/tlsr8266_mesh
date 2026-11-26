#!/usr/bin/env python3
"""Tests for mesh credential resolution (CLI flag > environment > .env > default).

These cover the credential plumbing only: no BLE connection is opened and no
wire format is touched. The CLIs are imported to prove their credential flags
are optional, which is what lets the environment supply them.
"""

import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import mesh_add
import mesh_node
from mesh_common import (
    DEFAULT_MESH_NAME,
    DEFAULT_MESH_PASSWORD,
    DOTENV_PATH,
    MESH_LTK_ENV,
    MESH_NAME_ENV,
    MESH_PASSWORD_ENV,
    parse_dotenv,
    resolve_mesh_credential,
)


class ParseDotenvTests(unittest.TestCase):
    def test_dotenv_path_is_repository_root(self):
        self.assertEqual(DOTENV_PATH.name, ".env")
        self.assertTrue((DOTENV_PATH.parent / "utilities" / "meshutils").is_dir())

    def _parse(self, text):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / ".env"
            path.write_text(text, encoding="utf-8")
            return parse_dotenv(path)

    def test_parses_export_quotes_and_comments(self):
        values = self._parse(
            "# credentials\n"
            "\n"
            "export MESH_NAME=homelab\n"
            'MESH_PASSWORD="p@ss=word"\n'
            "MESH_LTK='00112233445566778899aabbccddeeff'\n"
            "malformed line\n"
        )
        self.assertEqual(values["MESH_NAME"], "homelab")
        # Only the first '=' splits the pair.
        self.assertEqual(values["MESH_PASSWORD"], "p@ss=word")
        self.assertEqual(values["MESH_LTK"], "00112233445566778899aabbccddeeff")
        self.assertNotIn("malformed line", values)

    def test_missing_file_is_empty(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            self.assertEqual(parse_dotenv(Path(tmpdir) / "absent.env"), {})


class ResolveCredentialTests(unittest.TestCase):
    env_key = "MESH_TD11_TEST_VALUE"

    def _clear(self):
        patcher = mock.patch.dict(os.environ, {}, clear=False)
        patcher.start()
        self.addCleanup(patcher.stop)
        os.environ.pop(self.env_key, None)

    def test_cli_flag_beats_environment_and_dotenv(self):
        self._clear()
        with mock.patch.dict(os.environ, {self.env_key: "from-env"}):
            self.assertEqual(
                resolve_mesh_credential("from-flag", self.env_key, {self.env_key: "from-dotenv"}),
                "from-flag",
            )

    def test_environment_beats_dotenv(self):
        self._clear()
        with mock.patch.dict(os.environ, {self.env_key: "from-env"}):
            self.assertEqual(
                resolve_mesh_credential(None, self.env_key, {self.env_key: "from-dotenv"}),
                "from-env",
            )

    def test_dotenv_used_when_environment_unset(self):
        self._clear()
        self.assertEqual(
            resolve_mesh_credential(None, self.env_key, {self.env_key: "from-dotenv"}),
            "from-dotenv",
        )

    def test_default_used_when_nothing_set(self):
        self._clear()
        self.assertEqual(resolve_mesh_credential(None, self.env_key, {}, "fallback"), "fallback")

    def test_empty_values_do_not_win(self):
        self._clear()
        with mock.patch.dict(os.environ, {self.env_key: "   "}):
            self.assertEqual(
                resolve_mesh_credential("", self.env_key, {self.env_key: "from-dotenv"}),
                "from-dotenv",
            )

    def test_values_are_stripped(self):
        self._clear()
        with mock.patch.dict(os.environ, {self.env_key: "  spaced  \n"}):
            self.assertEqual(resolve_mesh_credential(None, self.env_key, {}), "spaced")


class MeshAddCliTests(unittest.TestCase):
    def test_credentials_are_optional_flags(self):
        args = mesh_add.build_arg_parser().parse_args(["--mesh_address", "5"])
        self.assertIsNone(args.mesh_name)
        self.assertIsNone(args.mesh_password)
        self.assertIsNone(args.mesh_ltk)

    def test_nothing_set_keeps_existing_defaults(self):
        with mock.patch.dict(os.environ, {}, clear=False):
            for key in (MESH_NAME_ENV, MESH_PASSWORD_ENV, MESH_LTK_ENV):
                os.environ.pop(key, None)
            args = mesh_add.build_arg_parser().parse_args(["--mesh_address", "5"])
            self.assertEqual(
                resolve_mesh_credential(args.mesh_name, MESH_NAME_ENV, {}, DEFAULT_MESH_NAME),
                DEFAULT_MESH_NAME,
            )
            self.assertEqual(
                resolve_mesh_credential(args.mesh_password, MESH_PASSWORD_ENV, {}, DEFAULT_MESH_PASSWORD),
                DEFAULT_MESH_PASSWORD,
            )
            self.assertIsNone(resolve_mesh_credential(args.mesh_ltk, MESH_LTK_ENV, {}))

    def test_environment_supplies_omitted_flags(self):
        args = mesh_add.build_arg_parser().parse_args(["--mesh_address", "5"])
        env = {
            MESH_NAME_ENV: "envmesh",
            MESH_PASSWORD_ENV: "envpass",
            MESH_LTK_ENV: "00112233445566778899aabbccddeeff",
        }
        with mock.patch.dict(os.environ, env):
            self.assertEqual(resolve_mesh_credential(args.mesh_name, MESH_NAME_ENV, {}), "envmesh")
            self.assertEqual(resolve_mesh_credential(args.mesh_password, MESH_PASSWORD_ENV, {}), "envpass")
            self.assertEqual(
                resolve_mesh_credential(args.mesh_ltk, MESH_LTK_ENV, {}),
                "00112233445566778899aabbccddeeff",
            )


class MeshNodeCliTests(unittest.TestCase):
    def test_credentials_are_optional_flags(self):
        args = mesh_node.build_arg_parser().parse_args(
            ["--address", "12:34:56:78:9A:1D", "--get-status"]
        )
        self.assertIsNone(args.mesh_name)
        self.assertIsNone(args.mesh_password)

    def test_environment_supplies_omitted_flags(self):
        args = mesh_node.build_arg_parser().parse_args(
            ["--address", "12:34:56:78:9A:1D", "--get-status"]
        )
        env = {MESH_NAME_ENV: "envmesh", MESH_PASSWORD_ENV: "envpass"}
        with mock.patch.dict(os.environ, env):
            self.assertEqual(resolve_mesh_credential(args.mesh_name, MESH_NAME_ENV, {}), "envmesh")
            self.assertEqual(resolve_mesh_credential(args.mesh_password, MESH_PASSWORD_ENV, {}), "envpass")

    def test_flags_still_win_over_environment(self):
        args = mesh_node.build_arg_parser().parse_args(
            ["--address", "12:34:56:78:9A:1D", "--mesh-name", "flagmesh", "--get-status"]
        )
        with mock.patch.dict(os.environ, {MESH_NAME_ENV: "envmesh"}):
            self.assertEqual(resolve_mesh_credential(args.mesh_name, MESH_NAME_ENV, {}), "flagmesh")


if __name__ == "__main__":
    unittest.main()
