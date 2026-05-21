# FAST-LIO2, map-infra optimizer, renderer, and localization architecture

## 1. Existing map-infra pipeline

`map-infra/tile_pipeline` has a clear industrial handoff:

```text
record / frame_dataset
  -> ingestion
  -> optimizer
      deskew
      voxel filter
      local odometry
      pose graph fusion
      world transform
  -> optimized_frame_dataset/
  -> renderer
      attribute filter
      projection / log-odds grid
      tile pyramid
  -> tile_bundle/
```

The stable optimizer-to-renderer contract is:

```text
optimized_frame_dataset/
  manifest.json
  frames.csv
  diagnostics.csv
  diagnostics/
  pcd/<bucket>/<frame_index>.pcd
```

Important map-infra rule: renderer should consume `optimized_frame_dataset`, not ad hoc stitched PCD + pose files. `frames.csv` is the canonical frame index; each PCD stores frame-local points, and the pose in `frames.csv` resolves points into world coordinates.

## 2. Can FAST-LIO2 replace `tile_pipeline/optimizer`?

Yes, but only if FAST-LIO2 is wrapped as an optimizer producer that emits the same `optimized_frame_dataset` contract.

FAST-LIO2 can replace or augment these optimizer stages:

| Existing optimizer responsibility | FAST-LIO2 replacement |
| --- | --- |
| motion compensation / deskew | FAST-LIO2 IMU propagation and point undistortion |
| local odometry | FAST-LIO2 scan-to-map IEKF update |
| local map | FAST-LIO2 ikd-tree incremental map |
| stitched local map | FAST-LIO2 registered frames + accumulated map |
| diagnostics | FAST-LIO2 metrics: residual, inliers, features, map points, latency |

FAST-LIO2 should not directly replace:

| Stage | Reason |
| --- | --- |
| renderer | Renderer is already source-agnostic if `optimized_frame_dataset` is valid |
| tile pyramid | This is a map product generation concern, not SLAM |
| production localization | FAST-LIO2 local odometry must be aligned/fused into map frame first |

## 3. Recommended unified architecture

```text
FAST-LIO2 mapping mode
  Apollo record
  LiDAR + IMU + /tf_static
      |
      v
  FastLio2MappingComponent / offline tool
      |
      +-- local pose
      +-- registered frame-local clouds
      +-- metrics
      +-- local/global alignment metadata
      v
  FastLio2OptimizedFrameDatasetWriter
      |
      v
  optimized_frame_dataset/     <--- map-infra renderer contract
      |
      v
  map-infra renderer
      |
      v
  tile_bundle/
```

Localization mode:

```text
optimized_frame_dataset / map tiles / NDT map
      |
      v
  FastLio2LocalizationComponent
      |
      +-- high-rate LIO local odometry
      +-- scan-to-prior-map correction
      +-- T_map_lio alignment state
      v
  /apollo/localization/pose in map frame
```

## 4. FAST-LIO2 optimizer output contract

FAST-LIO2 should emit two products:

### 4.1 Renderer handoff

```text
optimized_frame_dataset/
  manifest.json
  frames.csv
  diagnostics.csv
  pcd/
    000/
      000000.pcd
      000001.pcd
```

`frames.csv` fields must match map-infra schema:

```text
frame_index,timestamp_ns,pcd_path,
origin_x,origin_y,origin_z,
origin_qw,origin_qx,origin_qy,origin_qz,
point_count,world_z_min,world_z_max
```

PCD points should be frame-local registered/undistorted points. Renderer resolves:

```text
point_world = origin_pose * point_local
```

### 4.2 SLAM/map validation package

```text
fast_lio2_map_package/
  manifest.json
  map.pcd
  trajectory_tum.txt
  trajectory_apollo.csv
  extrinsics.pb.txt
  map_origin.pb.txt
  metrics.csv
  scorecard.json
```

This package is for QA and localization-map generation. It should not replace the renderer contract.

## 5. Local mapping scorecard

Local mapping should be judged by a scorecard, not visual inspection only.

### Input score

```text
lidar_rate_hz >= 5
imu_rate_hz >= 100
point_time_span_sec close to scan period
imu_acc_norm near 9.8
tf_static has valid lidar->imu
timestamps monotonic and overlapping
```

### Runtime score

```text
tracking_status == OK
reject_reason == OK
dropped_imu == 0
effective_feature_count > threshold
map_point_count grows but does not explode
latency budget satisfied for selected mode
```

### Trajectory score

```text
ATE/RMSE after SE2 or SE3 alignment
RPE over 10 m, 50 m, 100 m windows
path length ratio
yaw drift
height drift
closed-loop error if loop exists
```

### Map geometry score

```text
residual mean/p90
inlier ratio
wall/curb/pole thickness
repeat-pass overlap error
ghosting/double-edge count
dynamic-object contamination
tile occupancy continuity
```

Minimum current gates:

```text
short baseline SE2 RMSE <= 0.5 m
effective_feature_count > 500 for lower-feature LiDAR, >1000 for dense LiDAR
map_point_count > 4000 for short windows, >10000 for dense baseline
pointcloud timestamp span > 0
tracking OK
```

## 6. Renderer relationship

`tile_pipeline/renderer` can generate tiles from FAST-LIO2 output if FAST-LIO2 writes `optimized_frame_dataset`.

Renderer should remain unchanged except for:

1. accepting FAST-LIO2 producer metadata in `manifest.json`;
2. displaying FAST-LIO2 metrics in reports;
3. optionally rendering localization-map layers:
   - intensity raster,
   - height raster,
   - occupancy/log-odds raster,
   - ground/non-ground layers.

The renderer should not own SLAM correction logic.

## 7. Localization map generation

A FAST-LIO2-built map can produce multiple localization artifacts:

| Artifact | Use |
| --- | --- |
| dense/subsampled point cloud | ICP/GICP localizer |
| NDT voxel map | NDT localizer |
| intensity raster tiles | visual QA / optional intensity localization |
| occupancy/height tiles | map display and ROI filtering |
| trajectory and keyframes | relocalization seeds |
| map origin and ENU alignment | Apollo map-frame publication |

Recommended flow:

```text
FAST-LIO2 optimized_frame_dataset
  -> map cleaner / dynamic object filter
  -> voxel/tile split
  -> NDT or GICP localization map
  -> renderer tile bundle
  -> QA scorecard
```

## 8. Why not use only the stitched PCD?

A stitched PCD loses:

- frame origins,
- per-frame timestamps,
- frame provenance,
- raycasting origins for occupancy,
- per-frame QA diagnostics,
- ability to re-render with different filters,
- ability to debug bad local mapping windows.

Therefore stitched PCD is a secondary artifact. The production handoff remains `optimized_frame_dataset`.

## 9. Integration milestones

### Milestone 1: FAST-LIO2 as optimizer producer

Add:

```text
FastLio2OptimizedFrameDatasetWriter
FastLio2MapPackageWriter
FastLio2Scorecard
```

Output must pass `tile_pipeline/interface/README_CONTRACTS.md`.

### Milestone 2: renderer compatibility

Run:

```text
render_optimized_dataset_cli_main --input_dataset=<fast_lio2_dataset>
```

Expected output:

```text
tile_bundle/
  manifest.json
  render_summary.json
  tiles/
  previews/
```

### Milestone 3: localization map

Build:

```text
FastLio2LocalizationMapBuilder
```

It generates dense/NDT/tiled map artifacts and metadata.

### Milestone 4: online localization

Build:

```text
FastLio2LocalizationComponent
```

It keeps high-rate local LIO and estimates:

```text
T_map_imu = T_map_lio * T_lio_imu
```

### Milestone 5: regression farm

Promote these records:

```text
sensor_rgb.record
中集/2025年10月9日...
中集/2025-12-29/obstacle/20251229155810.record.00000
```

Each run should emit:

```text
baseline_scorecard.json
optimized_frame_dataset/
tile_bundle/
map_package/
```

## 10. Architecture decision

FAST-LIO2 should replace `tile_pipeline/optimizer` only at the optimizer contract boundary. It should not fork or bypass renderer. The best architecture is:

```text
FAST-LIO2 core -> optimized_frame_dataset -> existing renderer -> localization map builder -> localization component
```

This preserves map-infra's stable renderer contract while allowing FAST-LIO2 to become the higher-quality LiDAR-inertial optimizer frontend.

## 11. Tile/localization product consistency

The reference map at `/mnt/synology/map_data/apollo/sunnyvale_loop/local_map/` shows a useful production lesson: visual map tiles and runtime map data need a shared coordinate definition. However, WHL should not add an Apollo-format compatibility export. The canonical production output is a source-consistent map bundle:

```text
map_product_bundle/
  manifest.json
optimized_frame_dataset/
tile_bundle/
localization_map_package/
```

All map products must share:

```text
source_dataset_fingerprint
map_frame
map_origin_id
```

The renderer generates the visual tile map from frame-local points and optimized poses. The localization-map builder generates dense/NDT/keyframe localization artifacts from the same frame-local points and optimized poses. If either product uses a different source dataset, origin, or frame name, the run must fail.

Correct flow:

```text
optimized_frame_dataset
  -> tile_bundle
  -> localization_map_package
  -> map_product_bundle/manifest.json
```

The old Apollo `local_map` layout is a reference for why coordinate metadata matters, not an export target for this pipeline.
