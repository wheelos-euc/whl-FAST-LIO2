# FAST-LIO2 evaluation, baseline, mapping, and localization playbook

## 1. Evaluation philosophy

FAST-LIO2 should be evaluated in four independent dimensions:

1. Input validity: are LiDAR, IMU, timestamps, and extrinsics physically correct?
2. Runtime health: does the Cyber component process data without hidden failure?
3. Odometry and map quality: does the relative trajectory and map geometry make sense?
4. Production readiness: can it run in real time and provide map-frame localization with health monitoring?

Passing only dimension 2 means "the migration runs". It does not mean the result is good enough for autonomous driving.

## 2. Input validation checklist

### LiDAR checks

Required:

- PointCloud channel exists and has enough frames.
- Frame rate is stable and consistent with sensor type.
- Point count is stable.
- Per-point timestamps are non-zero and non-constant.
- `measurement_time` or header time is monotonic.
- Point frame has a valid `/tf_static` path to the IMU frame.

Good thresholds:

```text
LiDAR frame rate: >= 5 Hz, ideally 10 Hz or sensor nominal rate
point timestamp span: close to scan period, e.g. 0.09-0.11 s for 10 Hz spinning LiDAR
valid points after filtering: > 1000, preferably > 3000 for scan matching
```

Bad signs:

- `per_point_span_sec = 0.0`: deskew cannot be evaluated; use only for smoke tests.
- Timestamp jumps backward.
- Many frames rejected as "invalid or constant per-point timestamps".
- Point cloud already filtered to sparse non-ground only: useful for perception, usually bad for mapping.

### IMU checks

Required:

- IMU rate >= 100 Hz.
- Timestamp monotonic and overlaps LiDAR time range.
- Acceleration norm near gravity during static or low-dynamic periods.
- Gyro norm reasonable and finite.
- IMU coordinate frame matches the LiDAR-to-IMU extrinsic frame.

Good thresholds:

```text
IMU rate: >= 100 Hz
static/low-dynamic acceleration norm: 9.6-10.1 m/s^2 usually acceptable
gyro norm when static: near 0 rad/s
drop count: 0
```

Apollo raw GNSS IMU documents acceleration as Forward/Left/Up. FAST-LIO2 does not require a named vehicle frame; it requires that the IMU measurements and extrinsic use the same IMU coordinate definition.

### Extrinsic checks

FAST-LIO2 core expects:

```text
p_I = R_LI * p_L + T_LI
```

Use `/tf_static` this way:

- If TF is `imu -> lidar`, use it directly as LiDAR-to-IMU.
- If TF is `lidar -> imu`, invert it.
- If TF is chained, compose from IMU parent to LiDAR child.

Self-checks:

1. Quaternion norm is close to 1.
2. Translation is physically plausible.
3. A forward-moving vehicle should produce a forward-like trajectory after simple yaw/translation alignment.
4. The map should not look doubled, twisted, or mirrored.
5. Scan residual should not grow immediately after motion starts.

## 3. Candidate datasets found under `/mnt/synology`

The records below have LiDAR and IMU channels. Suitability depends on motion and timestamp quality.

| Dataset | Channels / rates | TF | Motion sampled | Suitability |
| --- | --- | --- | ---: | --- |
| `/mnt/synology/apollo/sensor_rgb.record` | Velodyne64 ~10 Hz, raw GNSS IMU ~187 Hz | `novatel -> velodyne64` | 12.2 m in first 5 s, 46 m effective baseline | Best current baseline |
| `/mnt/synology/中集/2025年10月9日 进入车厅/20251009183623.record.00000` | fusion LiDAR 10 Hz, corrected IMU 100 Hz | multi-hop `imu -> left_front -> ...` | 14.4 m over full sample | Good candidate after composing LiDAR extrinsic |
| `/mnt/synology/中集/12-22-perception-testing-data/20251222165739.record.00000` | fusion LiDAR 10 Hz, corrected IMU 100 Hz | multi-hop | 1.7 m in sampled window | Input check / low-motion baseline only |
| `/mnt/synology/中集/2025-12-29/obstacle/20251229155810.record.00000` | fusion LiDAR 10 Hz, corrected IMU 100 Hz | multi-hop | 11.5 m in sampled window | Good candidate after composing extrinsic |
| `/mnt/synology/软通/2026-05-06/20260429080453.record.00000` | lslidar 10 Hz, corrected IMU 100 Hz | `imu -> lslidar_main` | near-static | Static/extrinsic check only |
| `/mnt/synology/中铁/problem/2026-04-22-03-20-43/20260422032044.record.00000` | seyond front 10 Hz, corrected IMU 100 Hz | `imu -> lidar_front` | 5.7 m sampled | Not good for FAST-LIO2 deskew because point timestamp span was 0 |
| `/mnt/synology/road_test/39/20260515072115.record.00000` | perception non-ground cloud 10 Hz, corrected IMU 100 Hz | multi-hop | near-static | Not good for mapping: sparse non-ground cloud and timestamp span 0 |
| `/mnt/synology/中铁/zhongtie.record` | lidar128 + corrected IMU | no `/tf_static`, no reference pose | unknown | Smoke/stress only until extrinsic/reference are provided |

Conclusion: there are multiple records that meet the channel requirement, but the best next precision baselines are:

1. `sensor_rgb.record`: already validated.
2. `中集/2025年10月9日...`: enough motion, full TF chain, LiDAR/IMU/reference present.
3. `中集/2025-12-29/obstacle/20251229155810.record.00000`: enough motion and full TF chain.

For multi-LiDAR fusion topics, first decide whether FAST-LIO2 should use:

- one physical LiDAR with direct composed extrinsic, recommended for algorithm validation, or
- fused multi-LiDAR cloud, useful for mapping but needs a clearly defined fused cloud frame and extrinsic to IMU.

## 4. Current baseline and how to judge quality

Current baseline:

```text
record: /mnt/synology/apollo/sensor_rgb.record
output: /tmp/fast_lio2_baseline_sensor_rgb.record.00000
extrinsic: novatel -> velodyne64 from /tf_static
```

Result:

```text
metrics: 58
pose: 38
odometry: 38
cloud_registered: 45
map: 33
relative trajectory RMSE after SE2 alignment: 0.237 m
mean error: 0.209 m
max error: 0.653 m
FAST-LIO2 path: 46.621 m
reference path: 46.163 m
```

Health:

```text
tracking_status: OK
reject_reason: OK
effective features: 3050 final, mean 3465.9
map points: 33203 final
mean point-to-plane residual: 0.0237419 final
IMU drops: 0
```

Known problem:

```text
processing latency p50: ~982 ms
pointcloud drops: 78
```

Interpretation:

- Correctness baseline: pass.
- Relative odometry/map quality on this short segment: pass.
- Real-time production: fail until optimized.
- Absolute Apollo map-frame localization: not implemented yet; output is local FAST-LIO2 frame.

## 5. How to detect algorithm problems

### Symptom: trajectory rotated, mirrored, or diverges fast

Likely causes:

- LiDAR-to-IMU transform inverted.
- IMU axis convention mismatch.
- Wrong LiDAR frame selected for a fused cloud.
- Timestamp mismatch between LiDAR and IMU.

Checks:

- Compose `/tf_static` and confirm `p_I = R_LI p_L + T_LI`.
- Compare FAST-LIO2 trajectory to reference after SE2 alignment.
- Run a short straight segment: lateral drift should be small.

### Symptom: map is fuzzy, doubled, or has ghosting

Likely causes:

- Missing or constant point timestamps, bad deskew.
- Incorrect time offset between IMU and LiDAR.
- Dynamic objects dominating the scene.
- Too aggressive downsampling or wrong scan period.

Checks:

- Per-point timestamp span close to scan period.
- Residual does not spike after turns.
- Map cloud visual inspection around poles, curbs, walls.
- Compare same area from repeated passes.

### Symptom: residual high but trajectory seems smooth

Likely causes:

- Wrong voxel size or feature selection.
- Sparse/non-ground/perception cloud instead of raw/compensated LiDAR.
- Degenerate scene, e.g. corridor, open yard, repeated containers.

Checks:

- `effective_feature_count`.
- `scan_match_inlier_ratio`.
- Map point count growth.
- Compare raw LiDAR vs fused/perception cloud.

### Symptom: output pauses or skips frames

Likely causes:

- Processing slower than playback/sensor rate.
- Queue too small.
- CPU overloaded.

Checks:

- `last_processing_latency_ms`.
- `dropped_pointcloud_frames`.
- Replay at lower rate; if quality improves, performance is the bottleneck.

## 6. Map accuracy evaluation

FAST-LIO2 map quality should be evaluated at three levels.

### 6.1 Internal consistency

Metrics:

- point-to-plane residual distribution,
- inlier ratio,
- effective feature count,
- local map point count,
- repeated structure sharpness,
- thickness of planar walls/ground,
- overlap consistency between repeated passes.

Good signs:

- walls/curbs/poles are sharp,
- no obvious double edges,
- residual stable,
- map density not exploding,
- loop revisit aligns without visible offset.

### 6.2 Trajectory accuracy

Metrics:

- ATE/RMSE against reference pose after SE2/SE3 alignment,
- RPE over 10 m / 50 m / 100 m windows,
- yaw drift per 100 m,
- height drift,
- closed-loop error if route returns to start.

Current `sensor_rgb` baseline:

```text
SE2 RMSE: 0.237 m
path scale consistency: 46.621 m vs 46.163 m
```

### 6.3 Production map usability

For autonomous driving, a map is not just a point cloud. It needs:

- stable global frame,
- versioned map package,
- georeferencing to Apollo ENU/map frame,
- repeatability across days,
- localization success rate,
- semantic/HD-map alignment if used by planning.

Suggested gates:

```text
short baseline ATE after alignment: < 0.5 m
closed-loop drift: < 0.5-1.0% of path length for local map acceptance
localization success on built map: > 99% frames on clean routes
localization lateral error to HD map/reference: project-specific, typically decimeter-level for production
real-time latency: p95 below LiDAR period for online use
```

## 7. How to build a map with FAST-LIO2

Recommended mapping pipeline:

1. Select record with valid LiDAR, IMU, `/tf_static`, and enough motion.
2. Extract/compose LiDAR-to-IMU extrinsic.
3. Run FAST-LIO2 in offline mapping mode.
4. Record:
   - local pose,
   - odometry,
   - registered clouds,
   - accumulated map cloud,
   - metrics.
5. Save map package:
   - `map.pcd`,
   - `trajectory.txt`,
   - `extrinsics.pb.txt`,
   - `map_origin.pb.txt`,
   - `metrics.json`,
   - source record metadata.
6. Post-process:
   - voxel downsample,
   - remove dynamic objects,
   - crop ROI,
   - optionally split into tiles,
   - align to global map/ENU using GNSS/reference pose/control points.
7. Validate with ATE/RPE and visual map checks.

Important: FAST-LIO2 alone does not solve long-session loop closure. For large production maps, add pose graph optimization/loop closure or use repeated GNSS/reference constraints.

## 8. How to localize with a FAST-LIO2-built map

FAST-LIO2 is primarily LiDAR-inertial odometry and mapping. To localize on a prebuilt map, industry practice is to separate:

```text
high-rate local odometry: IMU + LiDAR odometry
low-rate/global correction: scan-to-map matching against prior map
fusion: EKF/ESKF/MSF combines odometry, map matching, GNSS/INS
```

Recommended architecture:

```text
Sensors
  LiDAR + IMU
    |
Fast-LIO2 odometry
  local frame pose at high rate
    |
Scan-to-map matcher
  NDT / ICP / GICP / point-to-plane against prior map
    |
Fusion filter
  map-frame localization + health
    |
Apollo /apollo/localization/pose
```

Implementation options in Apollo:

1. Use FAST-LIO2 to build a dense local/global point cloud map.
2. Convert the map to an Apollo localization map format or NDT voxel map.
3. Reuse Apollo NDT/MSF localization modules where possible.
4. Or add a new `FastLio2LocalizationComponent`:
   - load `map.pcd` or tiled map,
   - initialize from GNSS/reference pose,
   - run FAST-LIO2 local odometry,
   - periodically register current scan/local submap to prior map,
   - estimate `T_map_lio`,
   - publish map-frame `LocalizationEstimate`.

### Required localization states

The localization system should maintain:

```text
T_map_lio      global alignment from FAST-LIO2 local frame to map frame
T_lio_imu      high-rate FAST-LIO2 odometry state
IMU bias        estimated or inherited from LIO
quality state   OK / degraded / lost
```

Published pose:

```text
T_map_imu = T_map_lio * T_lio_imu
```

### Relocalization

If localization is lost:

1. Use GNSS/INS coarse pose, route prior, or last known pose.
2. Crop nearby map tiles.
3. Align current scan/submap with NDT/ICP/GICP.
4. Accept only if fitness, covariance, and consistency gates pass.
5. Reset `T_map_lio` and continue odometry.

## 9. Industry best practices

1. Do not use local odometry directly as global localization.
2. Keep mapping and localization modes separate.
3. Version every map with source records, calibration, code commit, config, and evaluation report.
4. Treat extrinsic and time offset as first-class map metadata.
5. Monitor quality online: residual, feature count, covariance, inlier ratio, map matching fitness, and drift to GNSS/reference.
6. Build maps offline with slower/high-quality settings; localize online with bounded real-time settings.
7. Use repeated routes and cross-day data to validate production robustness.
8. Add fallback: GNSS/INS, wheel odometry, dead reckoning, or degraded mode when LiDAR localization loses confidence.

## 10. Recommended next engineering milestones

1. Add `tools/extract_tf_static.py` to compose IMU-to-LiDAR transforms for any record.
2. Add `tools/evaluate_fast_lio2_record.py` to automate input validation and output scoring.
3. Add mapping mode that writes a complete map package.
4. Add localization mode that loads a map and publishes Apollo map-frame localization.
5. Optimize real-time performance and make frame drop a hard failure in online mode.
6. Promote `sensor_rgb`, `zhongji_20251009`, and `zhongji_obstacle_155810` into regression datasets.
