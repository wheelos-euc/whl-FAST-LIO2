# FAST-LIO2 Apollo Cyber migration plan

## 1. Baseline analysis

### Current repository
- `/home/humble/01code/whl-FAST-LIO2` currently contains only repository metadata, `README.md`, `LICENSE`, `.gitignore`, and `context/`; no FAST-LIO2 source code is present yet.
- The target migration therefore needs to introduce a Cyber-native FAST-LIO2 module rather than only patch existing code.

### Reference repository
- `/home/humble/01code/apollo-base` is a Bazel-based Apollo workspace with Cyber runtime, common message protos, launch/dag conventions, and existing SLAM/calibration modules.
- Relevant Apollo message types:
  - LiDAR input: `apollo.drivers.PointCloud` from `modules/common_msgs/sensor_msgs/pointcloud.proto`, often consumed through `cyber/adaptive/AdaptivePointCloudReader`.
  - IMU input: `apollo.localization.CorrectedImu` from `modules/common_msgs/localization_msgs/imu.proto`; raw GNSS IMU also exists as `apollo.drivers.gnss.Imu`.
  - Output pose: `apollo.localization.LocalizationEstimate`.
  - Optional odometry/debug output: `apollo.localization.Odometry` under `modules/slam_localization/proto/odometry.proto`, and registered/map clouds as `apollo.drivers.PointCloud`.
- Relevant Apollo component patterns:
  - `modules/slam_localization/*_component.{h,cc}` show Cyber component registration, config loading, writers/readers, dag/launch layout.
  - `modules/calibration/lidar_imu_calibration` already contains Cyber adapters for Apollo point cloud + CorrectedImu and a FAST-LIO2-like `ikd-Tree`/IMU processing core with ROS compatibility shims.

### ROS to Apollo migration surface
- Replace ROS node handles/subscribers/publishers with `apollo::cyber::Component<>`, `CreateReader`, `CreateWriter`, dag and launch files.
- Replace ROS messages:
  - `sensor_msgs::PointCloud2` / Livox custom messages -> `apollo::drivers::PointCloud` or `AdaptivePointCloudReader` view.
  - `sensor_msgs::Imu` -> `apollo.localization.CorrectedImu` adapter.
  - `nav_msgs::Odometry`, `geometry_msgs::PoseStamped`, `tf` -> `apollo.localization.LocalizationEstimate`, `apollo.localization.Odometry`, optional Apollo transform broadcaster.
- Replace ROS time with Apollo message `header.timestamp_sec`, `measurement_time`, or receive time when source timestamps are invalid.
- Replace ROS params with proto config (`fast_lio2_conf.proto` + `.pb.txt`).
- Replace catkin/CMake with Bazel `BUILD`.

## 2. Migration target design

### Module layout
Create a self-contained Apollo module:

```text
modules/fast_lio2/
  BUILD
  adapter/
    pointcloud_adapter.{h,cc}
    corrected_imu_adapter.{h,cc}
  component/
    fast_lio2_component.{h,cc}
  core/
    fast_lio2_core.{h,cc}
    ros_compat.h
    include/...
  conf/
    fast_lio2.pb.txt
  dag/
    fast_lio2.dag
  launch/
    fast_lio2.launch
  proto/
    BUILD
    fast_lio2_conf.proto
    fast_lio2_metrics.proto
```

### Reuse strategy
- Use official FAST-LIO2 algorithm structure: IMU propagation, scan undistortion, ikd-tree local map, iterated point-to-plane update, incremental map insertion.
- Reuse Apollo-compatible implementations already present in `modules/calibration/lidar_imu_calibration` where possible:
  - `ros_compat.h`
  - `common_lib.h`, `so3_math.h`, `scope_timer.hpp`, `ikd_Tree`
  - Apollo point cloud and CorrectedImu conversion ideas
- Do not depend on ROS headers, ROS launch, `rosbag`, or catkin.

### Runtime behavior
- A single `FastLio2Component` subscribes to LiDAR and IMU, synchronizes one LiDAR frame with the IMU window, runs FAST-LIO2 mapping/odometry, and publishes:
  - `/apollo/localization/fast_lio2/pose` as `LocalizationEstimate`
  - `/apollo/localization/fast_lio2/odometry` as `Odometry`
  - optional `/apollo/localization/fast_lio2/cloud_registered`
  - optional `/apollo/localization/fast_lio2/map`
  - `/apollo/localization/fast_lio2/metrics` for iteration metrics
- Known LiDAR-to-IMU extrinsics come from config first. If missing, search `/mnt/synology` for calibration outputs and derive config values from those files.

## 3. Validation and metric loop

### Iteration 1: migration/build
1. Add module code, proto config, BUILD, dag, launch.
2. Build in `/home/humble/01code/apollo-base` with Bazel.
3. Metric gate:
   - module and proto targets compile;
   - shared object exists under `bazel-bin/modules/fast_lio2/`;
   - no ROS include or symbol remains in the new Apollo-facing component.

### Iteration 2: input compatibility
1. Inspect `/mnt/synology/apollo/sensor_rgb.record` and `/mnt/synology/apollo/demo_3.5.record` channel/type metadata.
2. Select actual LiDAR and IMU channels.
3. Run mainboard + record playback with the new dag.
4. Metric gate:
   - component starts;
   - LiDAR frames and IMU messages are counted;
   - synchronized LiDAR+IMU packets are produced;
   - frame drop/rejection reason is visible if synchronization fails.

### Iteration 3: odometry output
1. Run the FAST-LIO2 update over Apollo record data.
2. Publish pose/odometry/cloud output.
3. Metric gate:
   - pose count > 0;
   - output timestamps are monotonic;
   - effective feature count, residual, processing latency and map size are published;
   - no crash during record playback.

### Iteration 4: quality and regression
1. Compare trajectory stability against available localization or GNSS channels if present.
2. Tune `blind`, voxel sizes, max iterations, timestamp mode, and extrinsics.
3. Metric gate:
   - mean processing latency below LiDAR frame period when possible;
   - rejected frame rate reported and minimized;
   - drift/trajectory sanity checked where reference localization exists.

## 4. Immediate implementation sequence

1. Create the Apollo module skeleton and proto config/metrics.
2. Implement Apollo adapters for point cloud and CorrectedImu.
3. Port FAST-LIO2 core using Apollo-compatible ikd-tree and IMU processing.
4. Add component queueing, synchronization, publishing, dag and launch.
5. Build in apollo-base, fix compile issues.
6. Inspect and play records from `/mnt/synology/apollo`, adjust channel config and extrinsics.
7. Iterate with metrics until the component starts, consumes data, and publishes odometry.

## 5. Iteration log

### Iteration 1: migration/build
- Added `modules/fast_lio2` in `/home/humble/01code/apollo-base` with Cyber component, proto config/metrics, Bazel BUILD, dag and launch.
- Reused Apollo-compatible FAST-LIO2/ikd-tree pieces from `modules/calibration/lidar_imu_calibration` and added a state snapshot API for publishing pose, odometry and metrics.
- Build target passed: `bazel build //modules/fast_lio2:libfast_lio2_component.so`.

### Iteration 2: Apollo record compatibility
- Inspected `/mnt/synology/apollo/demo_3.5.record`: it has GNSS/Localization but no LiDAR point cloud.
- Inspected `/mnt/synology/apollo/sensor_rgb.record`: usable channels are:
  - `/apollo/sensor/velodyne64/compensator/PointCloud2` (`apollo.drivers.PointCloud`)
  - `/apollo/sensor/gnss/imu` (`apollo.drivers.gnss.Imu`)
- Added raw GNSS IMU support (`RAW_GNSS_IMU`) in addition to `CorrectedImu`.
- Validation config now targets the Velodyne64 + raw GNSS IMU channels.

### Iteration 3: runtime validation
- Mainboard starts with the FAST-LIO2 dag using a local dag path and repository config.
- Fixed hardcoded `/apollo/data/lidar_imu_calibration/Log` startup failure by deriving the log directory from the configured result path.
- Replayed `/mnt/synology/apollo/sensor_rgb.record` with LiDAR + IMU channels and recorded FAST-LIO2 outputs.
- Output validation record `/tmp/fast_lio2_validation.record.00000` contains:
  - 12 `FastLio2Metrics` messages
  - 6 `LocalizationEstimate` messages
  - 6 `Odometry` messages
- Current validation uses lidar-only bootstrap because no matching LiDAR-to-IMU extrinsic for this 2018 Velodyne64 record was found under `/mnt/synology/apollo`; unrelated calibration tables exist elsewhere under `/mnt/synology/青岛大地物流`.
