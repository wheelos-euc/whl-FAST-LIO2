# FAST-LIO2 algorithm fidelity and map-flow review

Date: 2026-05-21

## 1. Direct answer

`modules/fast_lio2/core/` is a **migrated and refactored FAST-LIO2 runtime**, not a
line-for-line copy of `hku-mars/FAST_LIO`.

That distinction matters:

- the **algorithm order, frame math, and scan-to-map method** remain aligned with official FAST-LIO2;
- the **engineering shell** was changed for Apollo/Cyber integration and safety;
- there are also **vehicle-oriented guardrails** and runtime diagnostics that do not exist in the original research code.

So the correct description is:

```text
official FAST-LIO2 algorithm semantics preserved
+ Apollo/Cyber integration
+ runtime safety gates
+ operator-facing diagnostics
+ calibration-style initialization bridge
```

It is inaccurate to call it a pure untouched upstream copy.

## 2. What is preserved from official FAST-LIO2

The extracted runtime still follows the official sequence:

1. `SyncPackages()`  
   - `modules/fast_lio2/core/fast_lio2_runtime_core.cc:306-337`
   - collects one LiDAR scan and its IMU window up to `lidar_end_time_`

2. `ImuProcess::Process()`  
   - `modules/fast_lio2/core/imu_processing.h:434-484`
   - handles IMU initialization, propagation, and point undistortion

3. `LasermapFovSegment()`  
   - `modules/fast_lio2/core/fast_lio2_runtime_core.cc:946-1003`
   - maintains the local cube around the current pose

4. voxel downsample  
   - `modules/fast_lio2/core/fast_lio2_runtime_core.cc:450-453`

5. ikd-tree nearest search + plane fitting + iterative update  
   - `modules/fast_lio2/core/fast_lio2_runtime_core.cc:471-626`

6. `MapIncremental()`  
   - `modules/fast_lio2/core/fast_lio2_runtime_core.cc:1013-1062`

7. point transform convention  
   - `modules/fast_lio2/core/fast_lio2_runtime_core.cc:1064-1077`
   - uses:
     `p_map = R_wi * (R_li * p_lidar + t_li) + p_wi`

This matches the official FAST-LIO2 shape reviewed in
`context/fast_lio2_official_algorithm_review.md`.

## 3. What is different from official FAST-LIO2

These differences are real, deliberate, and now explicit.

### 3.1 Framework and IO differences

- ROS node -> Apollo Cyber component
- YAML/ROS params -> protobuf config
- ROS `sensor_msgs::Imu` / `PointCloud2` -> Apollo `CorrectedImu`,
  raw GNSS IMU, and adaptive pointcloud reader
- publish path/odometry/cloud/map -> Apollo localization, odometry,
  metrics, and pointcloud topics

These are engineering changes, not algorithm changes.

### 3.2 Initialization and bridge stages

The extracted runtime keeps a staged bridge:

- `WAITING_FOR_DATA`
- `LIDAR_ONLY_ODOMETRY`
- `DATA_ACCUMULATION`
- `REFINING`

This is not part of the official monolithic ROS entrypoint. It exists because
the Apollo migration had to support:

- vehicle-prior extrinsic bootstrap
- staged initialization handoff
- operator-visible stage transitions

This is an engineering wrapper around the same scan-to-map core, not a rewrite
of the core update equations.

### 3.3 Safety gates and retry logic

The extracted runtime adds production-oriented scan-match validation:

- residual threshold
- inlier ratio threshold
- effective feature threshold
- large update rejection
- retry from last valid pose in CV mode
- tracking-lost operator hints

Key implementation:

- `ValidateScanMatch()`
- `reject_rollback`
- retry path

Located at:

- `modules/fast_lio2/core/fast_lio2_runtime_core.cc:340-380`
- `:646-739`

Official FAST-LIO2 does not expose this much operator-facing protection.

### 3.4 Core modularization status

After the final refactor in this round, the module no longer exposes
calibration-named runtime types in `modules/fast_lio2/`.

Current module-facing boundaries are:

- `adapter/runtime_data.h`
- `proto/fast_lio2_runtime_conf.proto`
- `proto/fast_lio2_runtime_status.proto`
- `core/fast_lio2_runtime_core.h`
- `core/fast_lio2_runtime_core.cc`

The `apollo::calibration::*` runtime namespace was removed from this module.
The remaining initialization helper naming inside `core/include/LI_init/` is
retained because that code tracks the imported upstream initialization helper
rather than a dependency on Apollo's old calibration module.

## 4. Current correctness adjustments made in this round

The extracted module was tightened further during this review:

1. world/map clouds now publish with the configured `map_frame`, not `"imu"`
2. published point timestamps are reconstructed from scan begin time, not scan end time
3. default config no longer republishes the full accumulated map every second

This addressed concrete production-facing correctness risks discovered during
review.

## 5. Conclusion on algorithm fidelity

The right production statement is:

```text
The extracted core is an official-FAST-LIO2-derived runtime with preserved
algorithm order and frame math, plus Apollo/Cyber integration and production
guardrails.
```

The wrong statement would be:

```text
This is a pristine unmodified upstream FAST-LIO2 copy.
```

## 6. Map production flow review

The correct production chain is:

```text
FAST-LIO2 runtime
  -> optimized_frame_dataset
  -> tile_bundle
  -> localization_map_package
  -> map_product_bundle
```

### 6.1 Runtime layer

The FAST-LIO2 module publishes:

- pose
- odometry
- metrics
- registered cloud
- optional map cloud

This is runtime observability, not the final map product contract.

### 6.2 Canonical optimizer handoff

The canonical persistent source is:

```text
optimized_frame_dataset/
  manifest.json
  frames.csv
  pcd/<bucket>/<frame>.pcd
```

Code-backed semantics:

- frame-local points persisted in PCD
- map/world pose persisted in `frames.csv`
- renderer resolves world points from `pose * point_local`

Backed by:

- `tile_pipeline/interface/optimized_frame_dataset.cc:341-366`
- `tile_pipeline/renderer/tools/optimized_dataset_ingestor.cc:456-480`

### 6.3 Tile bundle and localization package consistency

This review tightened the consistency flow:

1. `optimized_frame_dataset` can now carry `dataset_fingerprint`
2. renderer propagates that fingerprint into `tile_bundle/manifest.json`
3. `materialize_map_products.py` requires matching
   `source_dataset_fingerprint`
4. `localization_map_package` now correctly describes
   `source_frame_clouds` as `frame_local`

So the acceptance artifact is now materially stronger:

```text
same source path
+ same frame count
+ same source dataset fingerprint
+ same map_frame
+ same map_origin_id
```

### 6.4 What is and is not complete

Current state:

- `tile_bundle/` : ready as a renderer/map-serving bundle
- `localization_map_package/` : ready as a source-consistent localization
  package scaffold
- `map_product_bundle/manifest.json` : ready as an acceptance artifact

Not yet complete:

- dense/NDT/localizer runtime indices are **not** built by the current
  `materialize_map_products.py`
- therefore the current `localization_map_package/` is not yet the final online
  localization runtime package

This is the critical production truth that must not be overstated.

## 7. Concrete baselines and scorecards

Current checked baselines:

| Dataset | Meaning | Result |
| --- | --- | --- |
| `sensor_rgb` | correctness baseline with runtime warnings | RMSE 0.237 m, warnings for drop/latency |
| `zhongji_20251009` | cleaner short-window regression sample | RMSE 0.124 m, 1 warning |

Checked scorecard state:

- `sensor_rgb`: pass, 0 fail, 2 warning
- `zhongji_20251009`: pass, 0 fail, 1 warning

These are the current concrete regression anchors, not generic aspirations.

## 8. Regression status after the final review fixes

After the last correctness patch set and the final core modularization, the
extracted Apollo drop-in runtime was revalidated in
`/home/humble/01code/apollo-base`:

- `bazel build //modules/fast_lio2:libfast_lio2_component.so` : passed
- local `mainboard` smoke start: passed
- startup config shows `publish_map: false`
- component start log confirmed after the `map_frame` / timestamp fixes
- module compiles after removing calibration-named core/proto/runtime types

Map-product regression was also rerun:

- `python3 tile_pipeline/tools/materialize_map_products_test.py` : passed
- `bazel test //tile_pipeline/renderer/tools:tile_bundle_reporter_contract_test` :
  passed

So the current checked state is:

- FAST-LIO2 Apollo drop-in runtime: buildable and startable
- map-product source-consistency contract: passing
- scorecard baselines: still passing

## 8. Release recommendation

### 8.1 Ready to submit

- self-contained Apollo drop-in FAST-LIO2 runtime module
- official-pipeline review documents
- scorecards and baseline artifacts
- source-consistent tile/localization package contracts

### 8.2 Not acceptable to claim yet

- final standalone non-Apollo workspace runtime
- completed online localization-map builder
- full production localization stack complete

## 9. Final review verdict

### FAST-LIO2 runtime

- **algorithm-fidelity verdict:** acceptable
- **Apollo drop-in verdict:** acceptable
- **line-for-line upstream claim:** not acceptable

### Map production chain

- **tile/localization source-consistency contract:** acceptable after this round
- **localization runtime-map completeness:** not yet complete

That is the honest submission and production-readiness boundary.
