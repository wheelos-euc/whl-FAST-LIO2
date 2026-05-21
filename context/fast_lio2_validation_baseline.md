# FAST-LIO2 Apollo Cyber validation and baseline

## 1. What counts as "works"

The migration is considered usable only when all three layers pass:

| Layer | Check | Pass condition |
| --- | --- | --- |
| Input compatibility | Apollo record has LiDAR, IMU, per-point timestamps and usable LiDAR-to-IMU extrinsic | LiDAR >= 5 Hz, IMU >= 100 Hz, acceleration norm near 9.8 m/s^2, point cloud has non-constant point timestamps, `/tf_static` contains IMU parent -> LiDAR child transform |
| Runtime health | Cyber component consumes data and publishes all FAST-LIO2 outputs | `/apollo/localization/fast_lio2/metrics`, `/pose`, `/odometry`, `/cloud_registered`, `/map` have messages; no crash; drops and reject reason are visible |
| Quality baseline | FAST-LIO2 trajectory is compared with reference localization | Relative trajectory after SE2 alignment has bounded RMSE; residual/features/map size are stable; processing latency is tracked |

This separates "component can run" from "odometry/map quality is acceptable". A run that only publishes a few messages is not enough.

## 2. Current best baseline dataset

`/mnt/synology/apollo/sensor_rgb.record` is the best immediate baseline because it contains all required FAST-LIO2 inputs and a reference localization trajectory:

| Channel | Type | Count | Observed rate / content |
| --- | --- | ---: | --- |
| `/apollo/sensor/velodyne64/compensator/PointCloud2` | `apollo.drivers.PointCloud` | 587 | ~10 Hz, ~101k points/frame, per-point span ~0.100 s |
| `/apollo/sensor/gnss/imu` | `apollo.drivers.gnss.Imu` | 11630 | ~187-193 Hz in sampled window |
| `/tf_static` | `apollo.transform.TransformStampeds` | 1 | `novatel -> velodyne64` extrinsic |
| `/apollo/localization/pose` | `apollo.localization.LocalizationEstimate` | 5856 | ~100 Hz reference pose |

Extracted `/tf_static`:

```text
novatel -> velodyne64:
  translation: (-0.0581, 1.4593, 1.2496)
  quaternion xyzw: (0.027487, -0.032230, 0.706574, 0.706370)
```

Apollo TF follows the parent-frame to child-frame convention. Since `novatel` is the IMU/GNSS frame and `velodyne64` is the LiDAR frame, this is the LiDAR-to-IMU transform expected by the FAST-LIO2 core (`R_LI`, `T_LI`: LiDAR frame L to IMU frame I). Therefore the values are used directly in `modules/fast_lio2/conf/fast_lio2.pb.txt`.

IMU sanity for this record:

```text
acc norm mean/median: 9.9029 / 9.8967 m/s^2
gyro norm mean/median: 0.02988 / 0.02024 rad/s
first sample acc=(-0.1728,-0.8645,9.7569), gyro=(-0.00055,0.00204,0.00156)
```

This is consistent with a valid gravity-bearing IMU stream. FAST-LIO2 can use it for local odometry/mapping. The output frame remains a local FAST-LIO2 frame unless explicitly aligned to Apollo map/ENU.

## 3. Other candidate records checked

| Record | FAST-LIO2 inputs | Extrinsic | Reference pose | Baseline suitability |
| --- | --- | --- | --- | --- |
| `/mnt/synology/apollo/sensor_rgb.record` | Velodyne64 + raw GNSS IMU | Yes: `novatel -> velodyne64` | Yes | Best current baseline |
| `/mnt/synology/中集/2026-5-6-标定/20260506031248.record.00000` | Vanjee LiDAR + corrected/raw IMU | Yes, multi-LiDAR chain | Yes | Good for calibration/static checks; sampled motion was only ~0.57 m, weak for odometry quality |
| `/mnt/synology/中集/2025-12-29/lidar/20251229095550.record.00000` | Fusion LiDAR + IMU | No `/tf_static` | Yes | Needs external calibration source before quality baseline |
| `/mnt/synology/北电科/20250827183214.record.00000` | LiDAR + corrected/raw IMU | Yes: `imu -> lidar_left` chain | Yes | Good for input tests; sampled motion was nearly static |
| `/mnt/synology/中铁/zhongtie.record` | LiDAR128 + corrected IMU | No `/tf_static` in record | No reference pose | Useful for smoke/stress only, not quality baseline |

## 4. Baseline run command

Use the real Bazel output directory, not the `bazel-bin` symlink, because this workspace may switch between multiple Bazel caches:

```bash
cd /home/humble/01code/apollo-base
export CYBER_PATH=/home/humble/01code/apollo-base/cyber
BIN=$(bazel info bazel-bin 2>/dev/null | tail -1)

"$BIN/cyber/mainboard/mainboard" \
  -d /home/humble/.copilot/session-state/ad3a4042-6200-4412-b09a-3923aebaa96c/files/fast_lio2_local.dag

"$BIN/cyber/tools/cyber_recorder/cyber_recorder" play \
  -f /mnt/synology/apollo/sensor_rgb.record \
  -c /apollo/sensor/velodyne64/compensator/PointCloud2 \
  -c /apollo/sensor/gnss/imu \
  -r 0.3 -s 0
```

Record output channels:

```bash
"$BIN/cyber/tools/cyber_recorder/cyber_recorder" record \
  -o /tmp/fast_lio2_baseline_sensor_rgb.record \
  -c /apollo/localization/fast_lio2/metrics \
  -c /apollo/localization/fast_lio2/pose \
  -c /apollo/localization/fast_lio2/odometry \
  -c /apollo/localization/fast_lio2/cloud_registered \
  -c /apollo/localization/fast_lio2/map
```

## 5. Current baseline result

### Baseline A: Apollo `sensor_rgb.record`

Output record:

```text
/tmp/fast_lio2_baseline_sensor_rgb.record.00000
duration: 40.740518 s
```

Published outputs:

| Output channel | Count |
| --- | ---: |
| `/apollo/localization/fast_lio2/metrics` | 58 |
| `/apollo/localization/fast_lio2/pose` | 38 |
| `/apollo/localization/fast_lio2/odometry` | 38 |
| `/apollo/localization/fast_lio2/cloud_registered` | 45 |
| `/apollo/localization/fast_lio2/map` | 33 |

Final metrics:

```text
lidar received: 117
imu received: 2268
synced frames processed: 38
dropped pointcloud frames: 78
dropped imu messages: 0
stage: DATA_ACCUMULATION
tracking: OK
reject_reason: OK
map points: 33203
effective features: 3050
mean point-to-plane residual: 0.0237419
```

Aggregate metrics:

```text
processing latency ms: mean 655.184, p50 981.594, max 1063.024
scan residual: mean 0.019707, p50 0.019334, max 0.026644
effective features: mean 3465.9, min 2943, max 3641
registered cloud: 45 frames, ~5060 points/frame
map cloud: 33 frames, max 32634 points
FAST-LIO2 local pose path: 38 poses, 11.306 s, 46.621 m
```

Trajectory quality against `/apollo/localization/pose`:

```text
matched pose pairs: 38 / 38
SE2 alignment yaw: 160.79 deg
relative trajectory RMSE: 0.237 m
mean error: 0.209 m
max error: 0.653 m
FAST-LIO2 path length: 46.621 m
reference path length: 46.163 m
```

Interpretation:

- The LiDAR-IMU odometry and map-building path is working.
- The relative trajectory is close to Apollo reference after SE2 alignment.
- The large alignment yaw is expected because this FAST-LIO2 output is currently a local odometry/map frame, not globally initialized to Apollo map/ENU.
- Runtime is not yet real-time at this playback/configuration: latency p50 is ~982 ms and many pointcloud frames were dropped due to slower processing. This is acceptable for a correctness baseline but not production real-time.
- The checked-in `tests/scorecard_metrics_sensor_rgb.json` should be interpreted as a correctness baseline with warning-level offline performance evidence, not as a clean real-time baseline.

## 6. Current pass/fail judgment

| Requirement | Current status |
| --- | --- |
| Can subscribe Apollo LiDAR + IMU | Pass |
| Can use `/tf_static` LiDAR-to-IMU extrinsic | Pass for `sensor_rgb.record` |
| IMU data is numerically valid | Pass |
| Point clouds have per-point timestamps for deskew | Pass |
| Publishes pose and odometry | Pass |
| Publishes registered cloud and map cloud | Pass |
| Relative trajectory matches reference | Pass for current baseline, RMSE 0.237 m after SE2 alignment |
| Absolute Apollo map-frame localization | Not yet; output is local FAST-LIO2 frame |
| Production real-time performance | Not yet; optimize/downsample/threading before full-speed use |

## 7. Regression gates for future changes

Every future migration or tuning change should keep these gates:

1. Build: `bazel build //modules/fast_lio2:libfast_lio2_component.so` passes.
2. Input gate: selected record has LiDAR, IMU, point timestamps, and LiDAR-to-IMU extrinsic.
3. Runtime gate: output record contains metrics, pose, odometry, registered cloud, and map.
4. Health gate: final metrics have `tracking_status: OK`, `reject_reason: OK`, `dropped_imu_messages == 0`, `effective_feature_count > 1000`, `map_point_count > 10000`.
5. Quality gate on `sensor_rgb.record`: after SE2 alignment to `/apollo/localization/pose`, RMSE should stay near the current 0.237 m baseline and not exceed 0.5 m without explanation.
6. Performance gate for production: processing latency p95 must be below the LiDAR frame period or the component must explicitly support offline mapping mode. Current baseline does not pass this gate.

## 8. Remaining production work

The current baseline is sufficient to validate the migration and to build local FAST-LIO2 maps. For automatic-driving production use, the next blocking items are:

1. Add global initialization/alignment if `/apollo/localization/fast_lio2/pose` is expected in Apollo map/ENU rather than local odometry frame.
2. Optimize real-time performance: reduce frame drops, tune downsampling/ROI, and profile the scan matching path.
3. Run long dynamic records with known extrinsics and reference pose; the current high-quality baseline is only ~11 s of effective FAST-LIO2 pose output.
4. Add an automated evaluator so the baseline checks above are run with one command and fail on regression.

## 9. Additional baseline: Zhongji 2025-10-09

This run validates that the Apollo FAST-LIO2 migration works beyond the original Apollo demo record.

Input:

```text
record: /mnt/synology/中集/2025年10月9日 进入车厅/20251009183623.record.00000
LiDAR: /apollo/sensor/vanjeelidar/left_front/PointCloud2
IMU: /apollo/sensor/gnss/corrected_imu
reference: /apollo/localization/pose
extrinsic from /tf_static: imu -> left_front
  translation: (-1.055, 2.665, 0.320)
  quaternion xyzw: (0.0, 0.0, 0.923880, 0.382680)
```

Input sanity:

```text
LiDAR: 360 frames, ~10.01 Hz, 57600 points/frame, point-time span ~0.100 s
Corrected IMU: 3493 samples, ~100 Hz
Reference pose: 369 samples, ~10 Hz
Reference path length: 14.434 m over the record
```

Output:

```text
/tmp/fast_lio2_baseline_zhongji_20251009.record.00000
/tmp/fast_lio2_baseline_zhongji_20251009.record.00001
```

Published outputs:

| Output channel | Count |
| --- | ---: |
| `/apollo/localization/fast_lio2/metrics` | 1146 |
| `/apollo/localization/fast_lio2/pose` | 1137 |
| `/apollo/localization/fast_lio2/odometry` | 1137 |
| `/apollo/localization/fast_lio2/cloud_registered` | 1137 |
| `/apollo/localization/fast_lio2/map` | 59 |

Final metrics:

```text
lidar received: 170
imu received: 1606
synced frames processed: 166
dropped pointcloud frames: 3
dropped imu messages: 0
stage: DATA_ACCUMULATION
tracking: OK
reject_reason: OK
map points: 4681
effective features: 534
mean point-to-plane residual: 0.018361
```

Aggregate metrics:

```text
processing latency ms: mean 26.529, p50 0.001, max 1646.613
scan residual: mean 0.018802, p50 0.018328, max 0.025420
effective features: mean 572.8, min 510, max 659
registered cloud: 1137 frames, ~1762 points/frame
map cloud: 59 frames, max 4672 points
FAST-LIO2 local pose path: 1137 poses, 16.088 s, 7.048 m
```

Trajectory quality against `/apollo/localization/pose`, sampled to approximately LiDAR rate before alignment:

```text
matched pairs: 159
SE2 alignment yaw: 38.113 deg
relative trajectory RMSE: 0.124 m
mean error: 0.105 m
max error: 0.309 m
FAST-LIO2 path length: 7.048 m
reference path length: 6.490 m
```

Interpretation:

- This is a second valid correctness/local-mapping baseline.
- It has much lower trajectory RMSE than `sensor_rgb` on the effective output segment.
- It uses fewer effective features than `sensor_rgb`, so it is useful as a lower-feature Vanjee baseline.
- Output pose frequency is higher than LiDAR rate because the current wrapper publishes state snapshots whenever the worker processes new buffered data; future evaluator should downsample to LiDAR timestamps for trajectory scoring.
- The run did not cover the whole record because offline processing still lags playback; use it as a validated short dynamic segment, not a full production run.
- The checked-in `tests/scorecard_metrics_zhongji.json` represents the preferred clean short-window regression sample.
