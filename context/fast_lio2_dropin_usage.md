# FAST-LIO2 drop-in Apollo usage

Date: 2026-05-21

## 1. What to copy

Copy this directory into Apollo:

```text
whl-FAST-LIO2/modules/fast_lio2
  -> apollo-base/modules/fast_lio2
```

The copied module is self-contained with respect to FAST-LIO2 runtime code:

- `component/`
- `adapter/`
- `core/`
- `proto/`
- `conf/`
- `dag/`
- `launch/`

It does **not** depend on:

```text
modules/calibration/lidar_imu_calibration/*
```

## 2. Build command

```bash
cd /path/to/apollo-base
bazel build //modules/fast_lio2:libfast_lio2_component.so
```

Validated on 2026-05-21:

- build target: `//modules/fast_lio2:libfast_lio2_component.so`
- result: success

## 3. Start command

Standard Apollo runtime assets:

- `modules/fast_lio2/dag/fast_lio2.dag`
- `modules/fast_lio2/conf/fast_lio2.pb.txt`
- `modules/fast_lio2/launch/fast_lio2.launch`

In a normal Apollo `/apollo` environment, launch with the usual mainboard /
launch tooling.

Local smoke validation was completed by starting mainboard with a local DAG
rewrite and observing:

```text
FAST-LIO2 component started. pointcloud_topic=... imu_topic=...
```

For host-mode replay outside a Docker `/apollo` mount, generate rewritten DAG /
config assets with:

```bash
# first list the Bazel-cache candidates you want to use
find /path/to/apollo-base/.cache/bazel -path '*/execroot/_main/bazel-out/*/bin/modules/fast_lio2/libfast_lio2_component.so' -type f
find /path/to/apollo-base/.cache/bazel -path '*/execroot/_main/bazel-out/*/bin/cyber/mainboard/mainboard' -type f
find /path/to/apollo-base/.cache/bazel -path '*/execroot/_main/bazel-out/*/bin/cyber/tools/cyber_recorder/cyber_recorder' -type f

python3 tools/prepare_apollo_host_run.py \
  --apollo_root /path/to/apollo-base \
  --output_dir /tmp/fastlio_runs/sensor_rgb_host \
  --record_path /mnt/synology/apollo/sensor_rgb.record \
  --module_library /abs/path/to/libfast_lio2_component.so \
  --mainboard_binary /abs/path/to/mainboard \
  --cyber_recorder_binary /abs/path/to/cyber_recorder
```

This rewrites:

- `module_library`
- `config_file_path`
- `flag_file_path`
- `result_path`

and raises `max_pending_pointcloud_frames` from the module default (`2`) to an
offline-validation default (`256`).

If the Bazel cache contains multiple matching binaries or libraries, the script
requires these explicit override paths instead of auto-selecting one candidate.

## 4. Required inputs for mapping

To use the module for mapping, provide:

1. one LiDAR topic with valid per-point timestamps
2. one IMU topic
3. LiDAR-to-IMU extrinsic:
   - `p_imu = R_li * p_lidar + t_li`
4. config values:
   - `point_filter_num`
   - `blind`
   - `filter_size_surf`
   - `filter_size_map`
   - `max_iteration`
   - `timestamp_mode`

## 5. What the module publishes

Runtime outputs:

- `/apollo/localization/fast_lio2/pose`
- `/apollo/localization/fast_lio2/odometry`
- `/apollo/localization/fast_lio2/metrics`
- `/apollo/localization/fast_lio2/cloud_registered`
- `/apollo/localization/fast_lio2/map`

These provide:

- local FAST-LIO2 pose
- odometry
- registered scan
- local accumulated map
- residual / feature / latency / drop diagnostics

## 6. What mapping should hand off downstream

The runtime outputs are not the final offline delivery by themselves.

The canonical mapping handoff should be:

```text
optimized_frame_dataset/
  frames.csv
  pcd/frame_*.pcd
```

Then derive:

```text
optimized_frame_dataset
  -> tile_bundle
  -> localization_map_package
  -> map_product_bundle/manifest.json
```

## 7. How to judge whether the result is good

Minimum acceptance:

1. input valid:
   - LiDAR timestamps non-constant
   - IMU rate sufficient
   - extrinsic direction correct
2. runtime healthy:
   - tracking status OK
   - reject reason OK
   - dropped IMU messages = 0
3. trajectory stable:
   - RMSE / RPE within baseline
4. map geometry good:
   - low residual
   - no doubled walls / ghosting
5. map products consistent:
   - same source fingerprint / frame / origin
