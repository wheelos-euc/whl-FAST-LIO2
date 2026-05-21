# FAST-LIO2 official algorithm review for WHL refactor

Date: 2026-05-21

## 1. Review target

The WHL refactor must preserve the original FAST-LIO2 algorithm semantics while changing only engineering boundaries:

```text
ROS node / Apollo component / record player / renderer
  are adapters

IMU propagation / point undistortion / IEKF update / ikd-tree map update
  are the FAST-LIO2 algorithm core
```

This review uses the official `hku-mars/FAST_LIO` implementation as the reference for module boundaries and data semantics.

## 2. Official algorithm sequence

The official `src/laserMapping.cpp` flow is:

```text
sync_packages()
  -> collect one LiDAR scan and IMU samples until lidar_end_time

ImuProcess::Process()
  -> IMU initialization
  -> forward propagation
  -> point undistortion
  -> output feats_undistort in body/LiDAR frame

lasermap_fov_segment()
  -> keep local map cube around current pose

voxel downsample
  -> feats_down_body

ikdtree initialization or nearest search

kf.update_iterated_dyn_share_modified()
  -> h_share_model()
     -> transform body point to map/world using:
        p_global = R_wi * (R_li * p_lidar + t_li) + p_wi
     -> nearest search in ikd-tree
     -> plane fitting
     -> point-to-plane residual Jacobian

map_incremental()
  -> transform accepted downsampled points to map/world
  -> add points to ikd-tree with voxel gating

publish odometry / registered cloud / optional map
```

WHL must keep this sequence intact. The refactor can change class names, config loading, and adapters, but not the data order or frame math.

## 3. Official frame and extrinsic convention

Official FAST-LIO2 uses:

```text
p_imu = R_L_I * p_lidar + T_L_I
p_map = R_map_imu * p_imu + T_map_imu
```

The README states that the extrinsic is the LiDAR pose in the IMU body frame. The official measurement model in `h_share_model()` applies `offset_R_L_I` and `offset_T_L_I` before the state pose.

WHL convention must therefore remain:

```text
ExtrinsicLidarToImu:
  p_imu = R_li * p_lidar + t_li
```

The Apollo `/tf_static` adapter may invert or compose TFs, but the core must only receive this canonical LiDAR-to-IMU transform.

## 4. What can be refactored

Safe refactor boundaries:

| Official responsibility | WHL module |
| --- | --- |
| ROS topic callbacks | Apollo/Cyber adapter |
| `sync_packages()` buffering | framework-independent synchronizer or adapter |
| `ImuProcess` | `fast_lio2/core` |
| IEKF state and update | `fast_lio2/core` |
| ikd-tree local map | `fast_lio2/core` |
| publish odometry/cloud/map | Apollo adapter and map-infra writer |
| PCD save | map product writer |

Unsafe changes:

- changing LiDAR-to-IMU extrinsic direction;
- feeding world-frame points back as frame-local points;
- using frame-level timestamps when point-level time exists;
- evaluating map quality only with a stitched PCD;
- making renderer consume FAST-LIO2-specific outputs.

## 5. Optimizer output review

Question: optimizer output is "aligned pose and point cloud"; is this correct for mapping?

Correct design:

```text
optimized_frame_dataset/
  frames.csv: optimized pose of each frame in map frame
  pcd/: undistorted frame-local points for each frame
```

This is correct because:

1. renderer can reconstruct world points with `point_map = T_map_lidar * point_local`;
2. localization map builder can use the exact same keyframes and poses;
3. frame provenance, timestamps, ray origins, and diagnostics are preserved;
4. tile map and localization map remain traceable to the same source.

Incorrect design:

```text
frames.csv: optimized pose
pcd/: already-world/aligned points
renderer applies pose again
```

This double-transforms points and breaks consistency.

Also incorrect:

```text
stitched_map.pcd only
```

This loses frame origins and cannot build a reliable localization map or raycasted tile map.

## 6. Unified production map product

FAST-LIO2 must output one canonical source:

```text
optimized_frame_dataset
```

Both map products must derive from that exact source:

```text
optimized_frame_dataset
  -> tile_map_product / tile_bundle
  -> localization_map_product / localization_map_package
  -> map_product_bundle manifest
```

All three manifests must share:

```text
source_dataset_fingerprint
map_frame
map_origin_id
```

If any differ, the build is invalid because the displayed high-definition base map may no longer correspond to the localization map used online.

## 7. Production acceptance

FAST-LIO2 can be considered a production optimizer replacement only when:

1. official algorithm order is preserved;
2. LiDAR/IMU synchronization and point timestamps pass input gates;
3. trajectory scorecard passes;
4. optimized_frame_dataset schema passes;
5. tile and localization map products share source fingerprint, map frame, and map origin;
6. online localizer can load the localization map and report pose in the same `map` frame rendered by the tile map.

