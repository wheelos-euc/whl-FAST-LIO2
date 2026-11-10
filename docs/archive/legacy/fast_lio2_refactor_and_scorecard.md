# WHL FAST-LIO2 refactor and continuous metric system

Date: 2026-05-21

## 1. Refactor target

The production target is no longer a ROS node or an Apollo-only wrapper. The target is a layered mapping/localization stack:

```text
fast_lio2/common        data types and frame conventions
fast_lio2/core          framework-independent FAST-LIO2 runtime boundary
fast_lio2/evaluation    scorecard gates and metric model
fast_lio2/map_infra     optimized_frame_dataset and localization-map contracts
fast_lio2/pipeline      offline mapping pipeline request/artifact layout
apollo-base adapter     Cyber message conversion, config, topics, replay
map-infra adapter       tile_pipeline optimizer backend bridge
```

The current Apollo migration remains the validated runtime implementation. New `whl-FAST-LIO2` code defines the extraction boundary so algorithm code can move out of Apollo safely without changing public contracts.

## 2. Core boundaries

### 2.1 Core input contract

The core accepts only canonical sensor data:

```text
ImuSample:
  timestamp_sec
  angular_velocity_radps
  linear_acceleration_mps2

PointCloudFrame:
  scan_start_sec
  scan_end_sec
  frame_id
  points[x,y,z,intensity,relative_time_sec,ring]

ExtrinsicLidarToImu:
  p_imu = R_li * p_lidar + t_li
```

This explicitly removes ROS message, Apollo protobuf, Cyber reader/writer, filesystem, and record playback dependencies from the algorithm boundary.

### 2.2 Core output contract

The core returns:

```text
Pose
OptimizedPointCloudFrame
ProcessingStats
```

These map directly to:

- Apollo `/apollo/localization/fast_lio2/pose`
- Apollo `/apollo/localization/fast_lio2/cloud_registered`
- Apollo `/apollo/localization/fast_lio2/metrics`
- map-infra `optimized_frame_dataset`

## 3. Metric system

The metric model is implemented in:

```text
fast_lio2/evaluation/scorecard.h
fast_lio2/evaluation/scorecard.cc
tools/evaluate_scorecard.py
tests/scorecard_contract_smoke.cc
```

It has four layers:

| Layer | Purpose |
| --- | --- |
| InputDataMetrics | reject bad records before blaming the algorithm |
| RuntimeMetrics | detect tracking, feature, residual, latency, and drop regressions |
| TrajectoryMetrics | compare against Apollo localization or external reference |
| MapQualityMetrics | verify renderer/localization-map products |

### 3.1 Required gates

Current default gates:

```text
lidar_frame_count >= 30
lidar_rate_hz >= 5
imu_rate_hz >= 95
point_time_span_sec >= 0.01
sensor_time_overlap_ratio >= 0.95
has_lidar_to_imu_extrinsic == true
timestamps_monotonic == true
tracking_ok == true
reject_ok == true
dropped_imu_messages == 0
effective_feature_count >= 500
map_point_count >= 4000
mean_residual <= 0.08
trajectory_rmse <= 0.50 m
path_length_ratio in [0.85, 1.15]
optimized_frame_dataset_valid == true
```

Warnings:

```text
dropped_lidar_frames > 0
latency_p95_ms > 200
trajectory_max_error > 1.5 m
tile/localization-map products missing
```

Warnings do not block algorithm correctness, but they block production readiness.

### 3.2 Regression workflow

Every baseline run should produce:

```text
diagnostics/input_audit.json
diagnostics/fast_lio2_metrics.csv
trajectory/fast_lio2_pose.csv
trajectory/reference_pose.csv
optimized_frame_dataset/
tile_bundle/
localization_map_package/
diagnostics/scorecard.json
```

The scorecard should be the CI/regression entry point. A change is acceptable only if:

1. all fail gates pass;
2. warning gates are triaged;
3. trajectory/map metrics do not regress against the stored baseline;
4. generated map artifacts match the declared schema.

Current checked-in scorecard inputs:

```text
tests/scorecard_metrics_sensor_rgb.json
tests/scorecard_metrics_zhongji.json
```

Interpretation:

- `sensor_rgb` is a correctness baseline and still carries runtime warnings for frame drops / latency;
- `zhongji` is the preferred clean short-window regression sample.

CLI usage:

```bash
python3 tools/evaluate_scorecard.py \
  --metrics_json tests/scorecard_metrics_zhongji.json \
  --output_json /tmp/fast_lio2_scorecard.json
```

Expected behavior:

```text
exit 0: all FAIL gates pass
exit 1: at least one FAIL gate fails
```

## 4. Current baselines

| Dataset | Status | Key result |
| --- | --- | --- |
| `sensor_rgb.record` | valid baseline | SE2 RMSE 0.237 m |
| `中集/2025年10月9日...` | valid short dynamic baseline | SE2 RMSE 0.124 m |

The Zhongji baseline passes the current scorecard gates if tile/localization-map products are marked as generated. Without those products it passes algorithm correctness but remains a WARN-level map-product run.

## 5. Refactor completion definition

The refactor is complete when:

1. Apollo adapter calls `whl::fast_lio2::FastLio2Core` instead of calibration-module internals.
2. The real FAST-LIO2 implementation is inside `fast_lio2/core`.
3. `fast_lio2/evaluation` emits scorecards from records automatically.
4. `fast_lio2/map_infra` writes `optimized_frame_dataset`.
5. `map-infra` renderer consumes that dataset without knowing the producer.
6. Localization map builder and tile renderer consume the same `optimized_frame_dataset` and publish matching `source_dataset_fingerprint`, `map_frame`, and `map_origin_id`.

## 6. Industry-practice review

The chosen architecture follows common production mapping practice:

- keep SLAM frontend independent of middleware;
- make record playback and publishing adapter-level responsibilities;
- use versioned persistent handoff contracts instead of ad hoc PCD files;
- score input quality separately from algorithm quality;
- keep renderer source-agnostic;
- produce localization maps from the same optimized keyframes used for tile rendering;
- reject builds where tile and localization products do not share the same source fingerprint, map frame, and map origin;
- retain the old optimizer until FAST-LIO2 passes the same contract and scorecard.

The critical risk is not whether FAST-LIO2 can generate a visually good local map. The risk is silent regressions caused by topic/extrinsic/timestamp changes. The scorecard gates are designed to catch those regressions before map production.
