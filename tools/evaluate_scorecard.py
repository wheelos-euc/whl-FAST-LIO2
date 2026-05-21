#!/usr/bin/env python3

"""Evaluate FAST-LIO2 baseline metrics against production gates."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any


DEFAULT_THRESHOLDS = {
    "min_lidar_frame_count": 30,
    "min_lidar_rate_hz": 5.0,
    "min_imu_rate_hz": 95.0,
    "min_point_time_span_sec": 0.01,
    "min_sensor_time_overlap_ratio": 0.95,
    "max_ate_rmse_m": 0.50,
    "max_ate_max_m": 1.50,
    "min_path_length_ratio": 0.85,
    "max_path_length_ratio": 1.15,
    "min_effective_feature_count": 500,
    "min_map_point_count": 4000,
    "max_mean_residual": 0.08,
    "max_latency_p95_ms": 200.0,
    "min_tile_coverage_ratio": 0.90,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a FAST-LIO2 pass/fail scorecard from metric JSON."
    )
    parser.add_argument("--metrics_json", required=True)
    parser.add_argument("--thresholds_json", default="")
    parser.add_argument("--output_json", required=True)
    parser.add_argument(
        "--allow_warnings",
        action="store_true",
        help="Exit 0 when only warning gates fail. Fail gates always return 1.",
    )
    return parser.parse_args()


def load_json(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle)


def get_number(root: dict[str, Any], dotted_key: str, default: float = 0.0) -> float:
    value: Any = root
    for part in dotted_key.split("."):
        if not isinstance(value, dict) or part not in value:
            return default
        value = value[part]
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def get_bool(root: dict[str, Any], dotted_key: str, default: bool = False) -> bool:
    value: Any = root
    for part in dotted_key.split("."):
        if not isinstance(value, dict) or part not in value:
            return default
        value = value[part]
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "y", "ok", "pass")
    return bool(value)


def min_gate(
    gates: list[dict[str, Any]],
    metric_id: str,
    severity: str,
    observed: float,
    threshold: float,
    message: str,
) -> None:
    gates.append(
        {
            "id": metric_id,
            "severity": severity,
            "status": "PASS" if observed >= threshold else severity,
            "observed": observed,
            "threshold": threshold,
            "operator": ">=",
            "message": message,
        }
    )


def max_gate(
    gates: list[dict[str, Any]],
    metric_id: str,
    severity: str,
    observed: float,
    threshold: float,
    message: str,
) -> None:
    gates.append(
        {
            "id": metric_id,
            "severity": severity,
            "status": "PASS" if observed <= threshold else severity,
            "observed": observed,
            "threshold": threshold,
            "operator": "<=",
            "message": message,
        }
    )


def bool_gate(
    gates: list[dict[str, Any]],
    metric_id: str,
    severity: str,
    observed: bool,
    message: str,
) -> None:
    gates.append(
        {
            "id": metric_id,
            "severity": severity,
            "status": "PASS" if observed else severity,
            "observed": observed,
            "threshold": True,
            "operator": "==",
            "message": message,
        }
    )


def build_scorecard(metrics: dict[str, Any], thresholds: dict[str, Any]) -> dict[str, Any]:
    th = dict(DEFAULT_THRESHOLDS)
    th.update(thresholds)
    gates: list[dict[str, Any]] = []

    min_gate(
        gates,
        "input.lidar_frame_count",
        "FAIL",
        get_number(metrics, "input.lidar_frame_count"),
        th["min_lidar_frame_count"],
        "LiDAR frame count must cover a useful segment.",
    )
    min_gate(
        gates,
        "input.lidar_rate_hz",
        "FAIL",
        get_number(metrics, "input.lidar_rate_hz"),
        th["min_lidar_rate_hz"],
        "LiDAR rate must support LIO.",
    )
    min_gate(
        gates,
        "input.imu_rate_hz",
        "FAIL",
        get_number(metrics, "input.imu_rate_hz"),
        th["min_imu_rate_hz"],
        "IMU rate must support deskew and propagation.",
    )
    min_gate(
        gates,
        "input.point_time_span_sec",
        "FAIL",
        get_number(metrics, "input.point_time_span_sec"),
        th["min_point_time_span_sec"],
        "LiDAR point timestamps must not be constant.",
    )
    bool_gate(
        gates,
        "input.has_lidar_to_imu_extrinsic",
        "FAIL",
        get_bool(metrics, "input.has_lidar_to_imu_extrinsic"),
        "LiDAR-to-IMU extrinsic is required.",
    )
    bool_gate(
        gates,
        "input.timestamps_monotonic",
        "FAIL",
        get_bool(metrics, "input.timestamps_monotonic"),
        "Sensor timestamps must be monotonic.",
    )
    min_gate(
        gates,
        "input.sensor_time_overlap_ratio",
        "FAIL",
        get_number(metrics, "input.sensor_time_overlap_ratio"),
        th["min_sensor_time_overlap_ratio"],
        "Sensor time windows must overlap.",
    )

    bool_gate(
        gates,
        "runtime.tracking_ok",
        "FAIL",
        get_bool(metrics, "runtime.tracking_ok"),
        "FAST-LIO2 tracking must remain OK.",
    )
    bool_gate(
        gates,
        "runtime.reject_ok",
        "FAIL",
        get_bool(metrics, "runtime.reject_ok"),
        "FAST-LIO2 reject reason must remain OK.",
    )
    max_gate(
        gates,
        "runtime.dropped_lidar_frames",
        "WARN",
        get_number(metrics, "runtime.dropped_lidar_frames"),
        0.0,
        "Dropped LiDAR frames indicate processing lag.",
    )
    max_gate(
        gates,
        "runtime.dropped_imu_messages",
        "FAIL",
        get_number(metrics, "runtime.dropped_imu_messages"),
        0.0,
        "Dropped IMU messages can break deskew.",
    )
    min_gate(
        gates,
        "runtime.effective_feature_count",
        "FAIL",
        get_number(metrics, "runtime.effective_feature_count"),
        th["min_effective_feature_count"],
        "Feature count must support stable scan matching.",
    )
    min_gate(
        gates,
        "runtime.map_point_count",
        "FAIL",
        get_number(metrics, "runtime.map_point_count"),
        th["min_map_point_count"],
        "Map must accumulate enough points.",
    )
    max_gate(
        gates,
        "runtime.mean_point_to_plane_residual",
        "FAIL",
        get_number(metrics, "runtime.mean_point_to_plane_residual"),
        th["max_mean_residual"],
        "Mean point-to-plane residual is too high.",
    )
    max_gate(
        gates,
        "runtime.latency_p95_ms",
        "WARN",
        get_number(metrics, "runtime.latency_p95_ms"),
        th["max_latency_p95_ms"],
        "Processing latency exceeds real-time budget.",
    )

    min_gate(
        gates,
        "trajectory.matched_pose_count",
        "FAIL",
        get_number(metrics, "trajectory.matched_pose_count"),
        20.0,
        "Need enough matched poses for trajectory scoring.",
    )
    max_gate(
        gates,
        "trajectory.ate_rmse_m",
        "FAIL",
        get_number(metrics, "trajectory.ate_rmse_m"),
        th["max_ate_rmse_m"],
        "Aligned trajectory RMSE regressed.",
    )
    max_gate(
        gates,
        "trajectory.ate_max_m",
        "WARN",
        get_number(metrics, "trajectory.ate_max_m"),
        th["max_ate_max_m"],
        "Aligned trajectory max error is high.",
    )
    min_gate(
        gates,
        "trajectory.path_length_ratio.min",
        "FAIL",
        get_number(metrics, "trajectory.path_length_ratio"),
        th["min_path_length_ratio"],
        "Estimated path is too short versus reference.",
    )
    max_gate(
        gates,
        "trajectory.path_length_ratio.max",
        "FAIL",
        get_number(metrics, "trajectory.path_length_ratio"),
        th["max_path_length_ratio"],
        "Estimated path is too long versus reference.",
    )

    bool_gate(
        gates,
        "map.optimized_frame_dataset_valid",
        "FAIL",
        get_bool(metrics, "map.optimized_frame_dataset_valid"),
        "optimized_frame_dataset must satisfy renderer schema.",
    )
    bool_gate(
        gates,
        "map.tile_bundle_valid",
        "WARN",
        get_bool(metrics, "map.tile_bundle_valid"),
        "tile_bundle should be generated.",
    )
    bool_gate(
        gates,
        "map.localization_map_package_valid",
        "WARN",
        get_bool(metrics, "map.localization_map_package_valid"),
        "localization_map_package should be generated.",
    )
    bool_gate(
        gates,
        "map.map_products_consistent",
        "FAIL",
        get_bool(metrics, "map.map_products_consistent"),
        "Tile and localization maps must share source dataset, map frame, and map origin.",
    )
    min_gate(
        gates,
        "map.tile_coverage_ratio",
        "WARN",
        get_number(metrics, "map.tile_coverage_ratio"),
        th["min_tile_coverage_ratio"],
        "Tile coverage should preserve useful map extent.",
    )

    failed = [gate for gate in gates if gate["status"] == "FAIL"]
    warnings = [gate for gate in gates if gate["status"] == "WARN"]
    return {
        "schema_version": "1.0.0",
        "dataset_id": metrics.get("dataset_id", "unknown"),
        "passed": not failed,
        "fail_count": len(failed),
        "warning_count": len(warnings),
        "thresholds": th,
        "metrics": metrics,
        "gates": gates,
    }


def main() -> int:
    args = parse_args()
    metrics = load_json(args.metrics_json)
    thresholds = load_json(args.thresholds_json) if args.thresholds_json else {}
    scorecard = build_scorecard(metrics, thresholds)

    output_path = Path(args.output_json)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(scorecard, indent=2, ensure_ascii=False), encoding="utf-8")

    if scorecard["passed"]:
        print(
            f"PASS {scorecard['dataset_id']}: "
            f"{scorecard['warning_count']} warning gate(s)"
        )
        return 0
    print(
        f"FAIL {scorecard['dataset_id']}: "
        f"{scorecard['fail_count']} fail gate(s), "
        f"{scorecard['warning_count']} warning gate(s)",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
