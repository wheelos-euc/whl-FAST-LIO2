#!/usr/bin/env python3

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]


class RuntimeEnvTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.root = Path(self.temp_dir.name)
        self.script = self.root / "scripts/env.sh"
        self.script.parent.mkdir()
        shutil.copyfile(ROOT_DIR / "scripts/env.sh", self.script)
        self.config = self.root / "modules/fast_lio2/conf/cyber.pb.conf"
        self.config.parent.mkdir(parents=True)
        self.config.write_text("transport_conf {}\n")
        self.bin_dir = self.root / "cached-bin"
        self.core_bin = self.bin_dir / "external/wheelos_core+/cyber"
        self.tools = [
            self.core_bin / "mainboard/mainboard",
            self.core_bin / "tools/cyber_recorder/cyber_recorder",
        ]
        for tool in self.tools:
            tool.parent.mkdir(parents=True)
            tool.write_text("#!/bin/sh\nexit 0\n")
            tool.chmod(0o755)
        self.env = {
            **os.environ,
            "FAST_LIO2_BAZEL_BIN": str(self.bin_dir),
            "CYBER_IP": "127.0.0.2",
            "CYBER_DOMAIN_ID": "123",
            "PATH": "/usr/bin:/bin",
        }
        self.env.pop("FAST_LIO2_CYBER_PATH", None)

    def source(self, command: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["bash", "-c", command, "bash", str(self.script)],
            env=self.env,
            text=True,
            capture_output=True,
            check=False,
        )

    def test_rejects_execution_instead_of_sourcing(self) -> None:
        result = subprocess.run(
            ["bash", str(self.script)], text=True, capture_output=True, check=False
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("Source this file", result.stderr)

    def test_missing_tool_does_not_change_runtime_environment(self) -> None:
        self.tools[1].unlink()
        result = self.source(
            'export APOLLO_ROOT_DIR=unchanged; source "$1"; '
            'status=$?; printf "%s\\n" "$APOLLO_ROOT_DIR"; exit "$status"'
        )
        self.assertEqual(result.returncode, 1)
        self.assertIn("Core tool is missing", result.stderr)
        self.assertEqual(result.stdout.strip(), "unchanged")

    def test_missing_configuration_is_rejected(self) -> None:
        self.config.unlink()
        result = self.source('source "$1"')
        self.assertEqual(result.returncode, 1)
        self.assertIn("Cyber configuration is missing", result.stderr)

    def test_sources_tools_idempotently_and_preserves_domain(self) -> None:
        result = self.source(
            'source "$1" && source "$1" && '
            'printf "%s\\n" "$APOLLO_ROOT_DIR" "$CYBER_PATH" '
            '"$CYBER_IP" "$CYBER_DOMAIN_ID" "$PATH"; '
            "command -v mainboard; command -v cyber_recorder"
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        lines = result.stdout.splitlines()
        self.assertEqual(
            lines[:4],
            [str(self.root), str(self.config.parent.parent), "127.0.0.2", "123"],
        )
        for tool in self.tools:
            self.assertEqual(lines[4].split(":").count(str(tool.parent)), 1)
        self.assertEqual(lines[5:], [str(tool) for tool in self.tools])


if __name__ == "__main__":
    unittest.main()
