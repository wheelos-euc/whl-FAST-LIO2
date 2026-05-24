#!/usr/bin/env python3

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from tools import prepare_apollo_host_run


class PrepareApolloHostRunTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)
        self.apollo_root = self.root / "apollo-base"
        (self.apollo_root / "modules/fast_lio2/conf").mkdir(parents=True)
        (self.apollo_root / "modules/fast_lio2/dag").mkdir(parents=True)
        (self.apollo_root / "modules/common/data").mkdir(parents=True)
        (self.apollo_root / ".cache/bazel/hash/execroot/_main/bazel-out/k8-fastbuild/bin/modules/fast_lio2").mkdir(
            parents=True
        )
        (self.apollo_root / ".cache/bazel/hash/execroot/_main/bazel-out/k8-fastbuild/bin/cyber/mainboard").mkdir(
            parents=True
        )
        (
            self.apollo_root
            / ".cache/bazel/hash/execroot/_main/bazel-out/k8-fastbuild/bin/cyber/tools/cyber_recorder"
        ).mkdir(parents=True)

        (self.apollo_root / "modules/fast_lio2/conf/fast_lio2.pb.txt").write_text(
            'pointcloud_topic: "/apollo/sensor/velodyne64/compensator/PointCloud2"\n'
            'max_pending_pointcloud_frames: 2\n'
            'max_pending_imu_messages: 20000\n'
            'result_path: "/apollo/data/fast_lio2/Initialization_result.txt"\n',
            encoding="utf-8",
        )
        (self.apollo_root / "modules/fast_lio2/dag/fast_lio2.dag").write_text(
            'module_config {\n'
            '  module_library : "/apollo/bazel-bin/modules/fast_lio2/libfast_lio2_component.so"\n'
            '  components {\n'
            '    class_name : "FastLio2Component"\n'
            '    config {\n'
            '      name : "fast_lio2"\n'
            '      config_file_path : "/apollo/modules/fast_lio2/conf/fast_lio2.pb.txt"\n'
            '      flag_file_path: "/apollo/modules/common/data/global_flagfile.txt"\n'
            "    }\n"
            "  }\n"
            "}\n",
            encoding="utf-8",
        )
        (self.apollo_root / "modules/common/data/global_flagfile.txt").write_text(
            "# flags\n", encoding="utf-8"
        )
        (
            self.apollo_root
            / ".cache/bazel/hash/execroot/_main/bazel-out/k8-fastbuild/bin/modules/fast_lio2/libfast_lio2_component.so"
        ).write_text("", encoding="utf-8")
        (
            self.apollo_root
            / ".cache/bazel/hash/execroot/_main/bazel-out/k8-fastbuild/bin/cyber/mainboard/mainboard"
        ).write_text("", encoding="utf-8")
        (
            self.apollo_root
            / ".cache/bazel/hash/execroot/_main/bazel-out/k8-fastbuild/bin/cyber/tools/cyber_recorder/cyber_recorder"
        ).write_text("", encoding="utf-8")
        os.chmod(
            self.apollo_root
            / ".cache/bazel/hash/execroot/_main/bazel-out/k8-fastbuild/bin/cyber/mainboard/mainboard",
            0o755,
        )
        os.chmod(
            self.apollo_root
            / ".cache/bazel/hash/execroot/_main/bazel-out/k8-fastbuild/bin/cyber/tools/cyber_recorder/cyber_recorder",
            0o755,
        )

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def write_record_path(self) -> Path:
        record_path = self.root / "sensor_rgb.record"
        record_path.write_text("", encoding="utf-8")
        return record_path

    def test_generates_host_assets_with_detected_binaries(self) -> None:
        output_dir = self.root / "out"
        record_path = self.write_record_path()
        with mock.patch(
            "sys.argv",
            [
                "prepare_apollo_host_run.py",
                "--apollo_root",
                str(self.apollo_root),
                "--output_dir",
                str(output_dir),
                "--record_path",
                str(record_path),
            ],
        ):
            prepare_apollo_host_run.main()

        config_text = (output_dir / "fast_lio2.host.pb.txt").read_text(
            encoding="utf-8"
        )
        self.assertIn("max_pending_pointcloud_frames: 256", config_text)
        self.assertIn("max_pending_imu_messages: 20000", config_text)
        self.assertIn(
            f'result_path: "{(output_dir / "Initialization_result.txt").resolve()}"',
            config_text,
        )

        dag_text = (output_dir / "fast_lio2.host.dag").read_text(encoding="utf-8")
        self.assertIn("libfast_lio2_component.so", dag_text)
        self.assertIn(str((output_dir / "fast_lio2.host.pb.txt").resolve()), dag_text)
        self.assertIn(
            str(
                (
                    self.apollo_root / "modules/common/data/global_flagfile.txt"
                ).resolve()
            ),
            dag_text,
        )

        summary = json.loads(
            (output_dir / "host_run_summary.json").read_text(encoding="utf-8")
        )
        self.assertEqual(summary["play_rate"], 0.1)
        self.assertEqual(summary["max_pending_pointcloud_frames"], 256)
        self.assertEqual(summary["max_pending_imu_messages"], 20000)
        self.assertEqual(
            summary["result_path"],
            str((output_dir / "Initialization_result.txt").resolve()),
        )
        self.assertEqual(
            summary["cyber_recorder_play_command"][-3:],
            [str(record_path.resolve()), "-r", "0.1"],
        )
        self.assertTrue(summary["mainboard_command"][0].endswith("/mainboard"))

    def test_explicit_override_requires_existing_files(self) -> None:
        output_dir = self.root / "out-explicit"
        with mock.patch(
            "sys.argv",
            [
                "prepare_apollo_host_run.py",
                "--apollo_root",
                str(self.apollo_root),
                "--output_dir",
                str(output_dir),
                "--module_library",
                str(self.root / "missing.so"),
            ],
        ):
            with self.assertRaises(FileNotFoundError):
                prepare_apollo_host_run.main()

    def test_record_path_is_normalized_and_queue_values_are_clamped(self) -> None:
        output_dir = self.root / "out-clamped"
        record_path = self.write_record_path()
        with mock.patch(
            "sys.argv",
            [
                "prepare_apollo_host_run.py",
                "--apollo_root",
                str(self.apollo_root),
                "--output_dir",
                str(output_dir),
                "--record_path",
                str(record_path),
                "--max_pending_pointcloud_frames",
                "0",
                "--max_pending_imu_messages",
                "1",
            ],
        ):
            prepare_apollo_host_run.main()

        config_text = (output_dir / "fast_lio2.host.pb.txt").read_text(
            encoding="utf-8"
        )
        self.assertIn("max_pending_pointcloud_frames: 1", config_text)
        self.assertIn("max_pending_imu_messages: 1000", config_text)
        summary = json.loads(
            (output_dir / "host_run_summary.json").read_text(encoding="utf-8")
        )
        self.assertEqual(summary["max_pending_pointcloud_frames"], 1)
        self.assertEqual(summary["max_pending_imu_messages"], 1000)
        self.assertEqual(
            summary["cyber_recorder_play_command"][2], str(record_path.resolve())
        )

    def test_require_single_path_rejects_ambiguous_artifacts(self) -> None:
        older = self.root / "older-fastbuild"
        newer = self.root / "newer-opt"
        older.write_text("", encoding="utf-8")
        newer.write_text("", encoding="utf-8")
        os.utime(older, (1, 1))
        os.utime(newer, (2, 2))

        with self.assertRaisesRegex(ValueError, "explicit override flag"):
            prepare_apollo_host_run.require_single_path([older, newer], "candidate")

    def test_prepare_assets_does_not_require_cyber_recorder_without_record_path(self) -> None:
        recorder = (
            self.apollo_root
            / ".cache/bazel/hash/execroot/_main/bazel-out/k8-fastbuild/bin/cyber/tools/cyber_recorder/cyber_recorder"
        )
        recorder.unlink()
        output_dir = self.root / "out-no-recorder"
        with mock.patch(
            "sys.argv",
            [
                "prepare_apollo_host_run.py",
                "--apollo_root",
                str(self.apollo_root),
                "--output_dir",
                str(output_dir),
            ],
        ):
            prepare_apollo_host_run.main()

        summary = json.loads(
            (output_dir / "host_run_summary.json").read_text(encoding="utf-8")
        )
        self.assertEqual(summary["cyber_recorder_binary"], "")
        self.assertNotIn("cyber_recorder_play_command", summary)

    def test_relative_result_path_stays_under_output_dir(self) -> None:
        output_dir = self.root / "out-relative-result"
        record_path = self.write_record_path()
        with mock.patch(
            "sys.argv",
            [
                "prepare_apollo_host_run.py",
                "--apollo_root",
                str(self.apollo_root),
                "--output_dir",
                str(output_dir),
                "--record_path",
                str(record_path),
                "--result_path",
                "custom/Initialization_result.txt",
            ],
        ):
            prepare_apollo_host_run.main()

        summary = json.loads(
            (output_dir / "host_run_summary.json").read_text(encoding="utf-8")
        )
        self.assertEqual(
            summary["result_path"],
            str((output_dir / "custom/Initialization_result.txt").resolve()),
        )

    def test_home_relative_result_path_expands_before_absolute_check(self) -> None:
        output_dir = self.root / "out-home-relative-result"
        record_path = self.write_record_path()
        home_relative = "~/custom/Initialization_result.txt"
        expected_result_path = str(
            Path(home_relative).expanduser().resolve()
        )
        with mock.patch(
            "sys.argv",
            [
                "prepare_apollo_host_run.py",
                "--apollo_root",
                str(self.apollo_root),
                "--output_dir",
                str(output_dir),
                "--record_path",
                str(record_path),
                "--result_path",
                home_relative,
            ],
        ):
            prepare_apollo_host_run.main()

        summary = json.loads(
            (output_dir / "host_run_summary.json").read_text(encoding="utf-8")
        )
        self.assertEqual(summary["result_path"], expected_result_path)

    def test_relative_result_path_cannot_escape_output_dir(self) -> None:
        output_dir = self.root / "out-relative-escape"
        record_path = self.write_record_path()
        with mock.patch(
            "sys.argv",
            [
                "prepare_apollo_host_run.py",
                "--apollo_root",
                str(self.apollo_root),
                "--output_dir",
                str(output_dir),
                "--record_path",
                str(record_path),
                "--result_path",
                "../escaped/Initialization_result.txt",
            ],
        ):
            with self.assertRaisesRegex(ValueError, "must stay within --output_dir"):
                prepare_apollo_host_run.main()


if __name__ == "__main__":
    unittest.main()
