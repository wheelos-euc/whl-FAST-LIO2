#!/usr/bin/env python3

"""Generate host-mode FAST-LIO2 runtime assets for an Apollo workspace."""

from __future__ import annotations

import argparse
import json
import os
import re
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate host-mode FAST-LIO2 dag/pbtxt assets for Apollo replay."
    )
    parser.add_argument("--apollo_root", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--record_path", default="")
    parser.add_argument("--module_library", default="")
    parser.add_argument("--mainboard_binary", default="")
    parser.add_argument("--cyber_recorder_binary", default="")
    parser.add_argument("--play_rate", type=float, default=0.1)
    parser.add_argument("--max_pending_pointcloud_frames", type=int, default=256)
    parser.add_argument("--max_pending_imu_messages", type=int, default=20000)
    parser.add_argument("--result_path", default="")
    return parser.parse_args()


def require_single_path(candidates: list[Path], label: str) -> Path:
    if not candidates:
        raise FileNotFoundError(f"unable to locate {label}")
    if len(candidates) == 1:
        return candidates[0]
    joined = "\n".join(str(path) for path in candidates)
    raise ValueError(
        f"multiple {label} candidates found; pass the explicit override flag for this artifact:\n{joined}"
    )


def detect_bazel_output(apollo_root: Path, relative_pattern: str, label: str) -> Path:
    candidates = sorted(apollo_root.glob(relative_pattern))
    return require_single_path(candidates, label)


def require_existing_file(
    path: Path, label: str, *, must_be_executable: bool = False
) -> Path:
    resolved = path.expanduser().resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"{label} not found: {resolved}")
    if not resolved.is_file():
        raise FileNotFoundError(f"{label} is not a file: {resolved}")
    if must_be_executable and not os.access(resolved, os.X_OK):
        raise PermissionError(f"{label} is not executable: {resolved}")
    return resolved


def format_pbtxt_value(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return f'"{value}"'
    return str(value)


def replace_pbtxt_field(text: str, field_name: str, value: object) -> str:
    pattern = re.compile(rf"(^\s*{re.escape(field_name)}\s*:\s*).*$", re.MULTILINE)
    if pattern.search(text):
        return pattern.sub(
            lambda match: f"{match.group(1)}{format_pbtxt_value(value)}",
            text,
            count=1,
        )
    suffix = "" if text.endswith("\n") else "\n"
    return f"{text}{suffix}{field_name}: {format_pbtxt_value(value)}\n"


def rewrite_dag_paths(
    dag_text: str, module_library: Path, config_path: Path, flag_file_path: Path
) -> str:
    replacements = {
        "module_library": str(module_library),
        "config_file_path": str(config_path),
        "flag_file_path": str(flag_file_path),
    }
    rewritten = dag_text
    for field_name, value in replacements.items():
        pattern = re.compile(rf"(^\s*{re.escape(field_name)}\s*:\s*).*$", re.MULTILINE)
        if not pattern.search(rewritten):
            raise ValueError(f"DAG template missing required field: {field_name}")
        rewritten = pattern.sub(
            lambda match, path=value: f'{match.group(1)}"{path}"',
            rewritten,
            count=1,
        )
    return rewritten


def build_summary(
    *,
    apollo_root: Path,
    output_dir: Path,
    dag_path: Path,
    config_path: Path,
    module_library: Path,
    mainboard_binary: Path,
    cyber_recorder_binary: Path | None,
    record_path: Path | None,
    play_rate: float,
    max_pending_pointcloud_frames: int,
    max_pending_imu_messages: int,
    result_path: Path,
) -> dict[str, object]:
    summary = {
        "apollo_root": str(apollo_root),
        "generated_dag": str(dag_path),
        "generated_config": str(config_path),
        "module_library": str(module_library),
        "mainboard_binary": str(mainboard_binary),
        "cyber_recorder_binary": (
            str(cyber_recorder_binary) if cyber_recorder_binary is not None else ""
        ),
        "play_rate": play_rate,
        "max_pending_pointcloud_frames": max_pending_pointcloud_frames,
        "max_pending_imu_messages": max_pending_imu_messages,
        "result_path": str(result_path),
        "mainboard_command": [
            str(mainboard_binary),
            "-d",
            str(dag_path),
        ],
    }
    if record_path is not None:
        summary["cyber_recorder_play_command"] = [
            str(cyber_recorder_binary),
            "play",
            str(record_path),
            "-r",
            str(play_rate),
        ]
    return summary


def normalize_runtime_queue(pointcloud_frames: int, imu_messages: int) -> tuple[int, int]:
    return max(1, pointcloud_frames), max(1000, imu_messages)


def main() -> None:
    args = parse_args()
    apollo_root = Path(args.apollo_root).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    config_template = apollo_root / "modules" / "fast_lio2" / "conf" / "fast_lio2.pb.txt"
    dag_template = apollo_root / "modules" / "fast_lio2" / "dag" / "fast_lio2.dag"
    flag_file_path = apollo_root / "modules" / "common" / "data" / "global_flagfile.txt"
    config_template = require_existing_file(config_template, "FAST-LIO2 config template")
    dag_template = require_existing_file(dag_template, "FAST-LIO2 DAG template")
    flag_file_path = require_existing_file(flag_file_path, "Apollo global flagfile")

    module_library = (
        require_existing_file(Path(args.module_library), "FAST-LIO2 shared library")
        if args.module_library
        else detect_bazel_output(
            apollo_root,
            ".cache/bazel/*/execroot/_main/bazel-out/*/bin/modules/fast_lio2/libfast_lio2_component.so",
            "FAST-LIO2 shared library",
        )
    )
    mainboard_binary = (
        require_existing_file(
            Path(args.mainboard_binary),
            "cyber mainboard binary",
            must_be_executable=True,
        )
        if args.mainboard_binary
        else require_existing_file(
            detect_bazel_output(
                apollo_root,
                ".cache/bazel/*/execroot/_main/bazel-out/*/bin/cyber/mainboard/mainboard",
                "cyber mainboard binary",
            ),
            "cyber mainboard binary",
            must_be_executable=True,
        )
    )

    generated_config = output_dir / "fast_lio2.host.pb.txt"
    generated_dag = output_dir / "fast_lio2.host.dag"
    generated_summary = output_dir / "host_run_summary.json"
    record_path = (
        require_existing_file(Path(args.record_path), "Apollo record path")
        if args.record_path
        else None
    )
    cyber_recorder_binary = None
    if record_path is not None:
        cyber_recorder_binary = (
            require_existing_file(
                Path(args.cyber_recorder_binary),
                "cyber_recorder binary",
                must_be_executable=True,
            )
            if args.cyber_recorder_binary
            else require_existing_file(
                detect_bazel_output(
                    apollo_root,
                    ".cache/bazel/*/execroot/_main/bazel-out/*/bin/cyber/tools/cyber_recorder/cyber_recorder",
                    "cyber_recorder binary",
                ),
                "cyber_recorder binary",
                must_be_executable=True,
            )
        )
    if args.result_path:
        requested_result_path = Path(args.result_path).expanduser()
        if requested_result_path.is_absolute():
            result_path = requested_result_path.resolve()
        else:
            result_path = (output_dir / requested_result_path).resolve()
            if not result_path.is_relative_to(output_dir):
                raise ValueError(
                    "--result_path must stay within --output_dir when a relative path is used"
                )
    else:
        result_path = output_dir / "Initialization_result.txt"
    max_pending_pointcloud_frames, max_pending_imu_messages = normalize_runtime_queue(
        args.max_pending_pointcloud_frames,
        args.max_pending_imu_messages,
    )

    config_text = config_template.read_text(encoding="utf-8")
    config_text = replace_pbtxt_field(
        config_text, "max_pending_pointcloud_frames", max_pending_pointcloud_frames
    )
    config_text = replace_pbtxt_field(
        config_text, "max_pending_imu_messages", max_pending_imu_messages
    )
    config_text = replace_pbtxt_field(config_text, "result_path", str(result_path))
    generated_config.write_text(config_text, encoding="utf-8")

    dag_text = dag_template.read_text(encoding="utf-8")
    dag_text = rewrite_dag_paths(
        dag_text, module_library, generated_config, flag_file_path.resolve()
    )
    generated_dag.write_text(dag_text, encoding="utf-8")

    summary = build_summary(
        apollo_root=apollo_root,
        output_dir=output_dir,
        dag_path=generated_dag,
        config_path=generated_config,
        module_library=module_library,
        mainboard_binary=mainboard_binary,
        cyber_recorder_binary=cyber_recorder_binary,
        record_path=record_path,
        play_rate=args.play_rate,
        max_pending_pointcloud_frames=max_pending_pointcloud_frames,
        max_pending_imu_messages=max_pending_imu_messages,
        result_path=result_path,
    )
    generated_summary.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
