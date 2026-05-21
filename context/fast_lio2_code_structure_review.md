# FAST-LIO2 Apollo Cyber code structure review

## 1. Current implementation structure

The migration currently lives in `/home/humble/01code/apollo-base`, while `/home/humble/01code/whl-FAST-LIO2` stores the migration context documents.

Current Apollo module:

```text
modules/fast_lio2/
  BUILD
  component/
    fast_lio2_component.h
    fast_lio2_component.cc
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

The FAST-LIO2 algorithmic core is not fully copied into `modules/fast_lio2` yet. It is reused from the existing Apollo-compatible calibration module:

```text
modules/calibration/lidar_imu_calibration/
  adapter/
    pointcloud_view_adapter.{h,cc}
    corrected_imu_adapter.{h,cc}
  core/
    lidar_imu_calibration_core.{h,cc}
    imu_processing.h
    ros_compat.h
    include/
      common_lib.h
      so3_math.h
      ikd-Tree/...
      LI_init/...
```

This is why the current FAST-LIO2 component is mostly a Cyber wrapper:

1. Load proto config.
2. Convert Apollo `PointCloud`, `CorrectedImu`, or raw GNSS `Imu` into the calibration core format.
3. Feed the reused FAST-LIO2-like core.
4. Publish `LocalizationEstimate`, `Odometry`, metrics, registered cloud, and map cloud.

## 2. Official FAST-LIO2 structure

The official `hku-mars/FAST_LIO` repository is ROS/CMake oriented and has a compact layout:

```text
CMakeLists.txt
package.xml
config/
  avia.yaml
  horizon.yaml
  marsim.yaml
  mid360.yaml
  ouster64.yaml
  velodyne.yaml
launch/
  mapping_*.launch
include/
  common_lib.h
  so3_math.h
  use-ikfom.hpp
src/
  laserMapping.cpp
  preprocess.cpp
  preprocess.h
  IMU_Processing.hpp
msg/
  Pose6D.msg
PCD/
Log/
rviz_cfg/
```

The official node is optimized for research/demo usage:

- ROS node lifecycle, subscribers, publishers, and parameters.
- YAML launch/config files.
- `sensor_msgs::PointCloud2` / Livox messages / `sensor_msgs::Imu`.
- A mostly monolithic `laserMapping.cpp` main loop.
- PCD output and RViz visualization.

## 3. Key differences after Apollo migration

| Area | Official FAST-LIO2 | Current Apollo Cyber migration |
| --- | --- | --- |
| Runtime framework | ROS node | Cyber component loaded by `mainboard` |
| Build | Catkin/CMake | Bazel shared-object component |
| Config | YAML + ROS params | Apollo proto config + `.pb.txt` |
| LiDAR input | ROS `PointCloud2` / Livox | Apollo `PointCloud`, including adaptive protobuf/flatbuffer reader |
| IMU input | ROS `sensor_msgs/Imu` | Apollo `CorrectedImu` and raw `apollo.drivers.gnss.Imu` |
| Time | ROS time | Apollo header/measurement time; raw GNSS IMU GPS time converted to Unix when needed |
| Output | ROS odometry/path/cloud/map | Apollo `LocalizationEstimate`, `Odometry`, `PointCloud`, metrics proto |
| Metrics | Mostly logs/timing files | Dedicated `/apollo/localization/fast_lio2/metrics` channel |
| Extrinsic | YAML params | Config fields populated from `/tf_static` or static calibration |
| Core ownership | Direct FAST-LIO2 source | Reused `lidar_imu_calibration_core` with FAST-LIO2/ikd-tree pieces |
| Production readiness | Research mapping node | Better Cyber integration, but still needs core separation, map-frame localization, real-time tuning |

## 4. Important semantic checks

### LiDAR-to-IMU transform direction

The core state defines:

```text
offset_R_L_I: Rotation from LiDAR frame L to IMU frame I
offset_T_L_I: Translation from LiDAR frame L to IMU frame I
```

So the expected transform is:

```text
p_I = R_LI * p_L + T_LI
```

For Apollo `/tf_static`, `header.frame_id = imu_or_parent` and `child_frame_id = lidar_or_child` should be interpreted as the child frame pose in the parent frame, i.e. the affine transform maps child coordinates into parent coordinates. Therefore:

- `imu -> lidar`: can be used directly as `R_LI/T_LI`.
- `lidar -> imu`: must be inverted before use.
- Multi-hop chains, such as `imu -> base_link -> left_front -> right_front`, must be composed to get `imu -> target_lidar`.

### Apollo raw GNSS IMU coordinate convention

Apollo raw GNSS IMU documents acceleration and angular velocity as:

```text
Forward / Left / Up
around Forward / Left / Up axes
```

This differs from Apollo localization pose comments that often use VRF Right/Forward/Up for vehicle frame quantities. For FAST-LIO2 the key requirement is not Apollo vehicle-frame naming; it is that angular velocity and acceleration are expressed in the same IMU frame used by `R_LI/T_LI`.

Practical validation:

1. Static or near-static acceleration norm should be close to 9.8 m/s^2.
2. Gravity direction after applying initial attitude should be consistent and not flip signs.
3. A pure forward motion should not appear as large sideways drift after SE2 trajectory alignment.
4. If trajectory is mirrored, rotated by 90/180 deg unexpectedly, or quickly diverges, first suspect extrinsic direction/sign and IMU frame convention.

### Per-point time requirement

The adapter rejects point clouds with missing or constant point timestamps:

```text
point cloud has invalid or constant per-point timestamps
```

This is required because FAST-LIO2 uses each point's relative scan time for motion compensation. Records with `span=0.0` are usable for smoke tests only, not for high-quality FAST-LIO2 deskew evaluation.

## 5. Current structural shortcomings

The current migration works, but it is not yet the ideal long-term architecture.

### 5.1 Core is coupled to calibration module

`modules/fast_lio2` depends on `modules/calibration/lidar_imu_calibration/core`. This was efficient for migration, but it mixes:

- Online odometry/mapping.
- LiDAR-IMU calibration initialization/refinement.
- Apollo component publishing.
- Visualization/status logic.

Risk: future FAST-LIO2 runtime changes may accidentally affect calibration behavior, and calibration-specific state names leak into odometry code.

### 5.2 Official algorithm structure is not preserved as a clean pure C++ core

Official FAST-LIO2 separates algorithmic ideas across preprocess, IMU processing, IEKF, ikd-tree, and mapping loop. In the current Apollo migration, these are embedded in the calibration core, not a reusable `fast_lio2_core`.

Risk: hard to unit-test synchronization, deskew, scan-to-map, map insertion, and state propagation independently.

### 5.3 Mapping and localization modes are not split

The component currently publishes local odometry and map. It does not yet support:

- A mapping mode that saves a versioned map product.
- A localization mode that loads a prior map and performs scan-to-map relocalization.
- Explicit global `map`/ENU alignment.

Risk: consumers may mistake local FAST-LIO2 odometry for Apollo global localization.

### 5.4 Performance mode is unclear

The current baseline is correctness-oriented, not real-time. Runtime latency and pointcloud drops are visible, but the component does not yet expose an explicit `offline_mapping` vs `realtime_localization` mode.

Risk: users cannot tell whether frame drops are acceptable offline behavior or production failures.

## 6. Recommended production-grade structure

For industry-quality maintainability, move the migrated code into `whl-FAST-LIO2` with a framework-independent core and thin adapters:

```text
whl-FAST-LIO2/
  context/
    fast_lio2_apollo_migration_plan.md
    fast_lio2_validation_baseline.md
    fast_lio2_code_structure_review.md
    fast_lio2_evaluation_and_mapping_playbook.md
  fast_lio2/
    core/
      fast_lio2_core.h
      fast_lio2_core.cc
      fast_lio2_options.h
      fast_lio2_state.h
      imu_propagator.h
      imu_propagator.cc
      scan_undistorter.h
      scan_undistorter.cc
      scan_matcher.h
      scan_matcher.cc
      ikd_map.h
      ikd_map.cc
      map_builder.h
      map_builder.cc
    common/
      time.h
      point_types.h
      extrinsic.h
      pose.h
      metrics.h
    io/
      pcd_writer.h
      trajectory_writer.h
      map_package.h
    apollo/
      component/
        fast_lio2_mapping_component.{h,cc}
        fast_lio2_localization_component.{h,cc}
      adapter/
        apollo_pointcloud_adapter.{h,cc}
        apollo_imu_adapter.{h,cc}
        apollo_tf_extrinsic.{h,cc}
      proto/
        fast_lio2_conf.proto
        fast_lio2_metrics.proto
      dag/
      launch/
      BUILD
    tools/
      evaluate_record.py
      extract_tf_static.py
      compare_trajectory.py
      export_map.py
    tests/
      imu_time_test.cc
      extrinsic_direction_test.cc
      pointcloud_time_test.cc
      trajectory_alignment_test.cc
```

### Layering rule

The core must not include Apollo, ROS, Cyber, protobuf, or file-system-specific runtime paths. It should consume plain C++ structs:

```text
ImuSample {
  double timestamp_sec;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d linear_acceleration;
}

PointCloudFrame {
  double scan_start_sec;
  double scan_end_sec;
  std::vector<PointXYZIT> points;
}

Extrinsic {
  Eigen::Matrix3d R_LI;
  Eigen::Vector3d T_LI;
}
```

Apollo code should only:

1. Read Cyber messages.
2. Convert them to core structs.
3. Call `FastLio2Core::AddImu`, `AddPointCloud`, `Process`.
4. Publish Apollo messages and metrics.

### Component split

Recommended Cyber components:

| Component | Purpose | Output |
| --- | --- | --- |
| `FastLio2MappingComponent` | Offline or online local mapping | local pose, registered cloud, map cloud, map package |
| `FastLio2LocalizationComponent` | Load prior map and localize current scan | Apollo map-frame `LocalizationEstimate` |
| `FastLio2Evaluator` tool | Offline regression and quality report | JSON/Markdown metrics |

## 7. Immediate code improvement backlog

1. Extract `FastLio2Core` from `lidar_imu_calibration_core` and keep calibration-specific LI initialization optional.
2. Add explicit runtime mode:
   - `OFFLINE_MAPPING`: slower replay allowed, no hard real-time requirement.
   - `REALTIME_ODOMETRY`: frame drop is failure unless within budget.
   - `MAP_LOCALIZATION`: prior map required.
3. Add `/tf_static` extraction utility that composes multi-hop chains and writes config fields.
4. Add automated evaluator for:
   - input channel validation,
   - IMU norm/time monotonicity,
   - per-point timestamp span,
   - output channel counts,
   - trajectory SE2/SE3 alignment,
   - residual/features/map size/latency.
5. Add map product writer:
   - `map.pcd`,
   - `trajectory.txt`,
   - `extrinsics.pb.txt`,
   - `map_origin.pb.txt`,
   - `metrics.json`.
6. Add localization mode on built map using scan-to-map registration and IMU propagation.

## 8. Bottom line

The current migration is a valid Apollo Cyber proof-of-work and baseline. The main architectural gap is that `modules/fast_lio2` is still a wrapper around a calibration core. For long-term production, move the code into `whl-FAST-LIO2` as a standalone pure FAST-LIO2 library with Apollo as one adapter, then add separate mapping, localization, and evaluation tools.
