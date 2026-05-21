# FAST-LIO2 standalone Cyber runtime review

Date: 2026-05-21

## 1. Direct answer

`whl-FAST-LIO2` now contains a **self-contained Apollo drop-in FAST-LIO2
module**, but the repository itself is still not a complete standalone Apollo
workspace.

That conclusion is based on the current code, not on intention:

1. `modules/fast_lio2/` now contains:
   - `component/`
   - `adapter/`
   - `core/`
   - `proto/`
   - `conf/`
   - `dag/`
   - `launch/`
2. Those copied runtime sources have been rewired to include
   `modules/fast_lio2/...` rather than
   `modules/calibration/lidar_imu_calibration/...`.
3. Apollo validation now succeeds with:
   - `bazel build //modules/fast_lio2:libfast_lio2_component.so`
   - `mainboard -d fast_lio2_local.dag -p fast_lio2_smoke`
4. The module starts without linking against the calibration module target.

Therefore the accurate status is:

```text
whl-FAST-LIO2/modules/fast_lio2 = validated Apollo drop-in runtime
whl-FAST-LIO2 root              = source-of-truth repo + evaluation toolkit
```

## 2. What "standalone Cyber module" should mean

For this repository, "standalone Cyber module" should mean:

1. `whl-FAST-LIO2` contains its own Cyber component entrypoint.
2. FAST-LIO2 runtime code inside `whl-FAST-LIO2` performs the full algorithm:
   - sync
   - IMU init / propagation
   - point undistortion
   - downsample
   - ikd-tree scan-to-map update
   - map_incremental
3. The module no longer links against
   `apollo-base/modules/calibration/lidar_imu_calibration`.
4. The evaluation and scorecard remain in the same repository as the runtime.

It may still depend on Apollo framework/runtime facilities such as:

- `cyber`
- Apollo protobuf message definitions
- PCL / Eigen / Ceres / Boost

The key requirement is:

```text
no algorithm/runtime dependency on apollo-base calibration libraries
```

## 3. Current dependency gap

Today the extracted gap is not small.

### 3.1 What is already in `whl-FAST-LIO2`

- self-contained `modules/fast_lio2` Apollo drop-in runtime
- scorecard and evaluation CLI
- official pipeline contract
- map-product contract
- framework-independent type definitions
- a minimal `fast_lio2_core` contract shell

### 3.2 What is still missing for a real runtime

- a full Bazel workspace so this repository can be built without being copied
  into Apollo
- direct in-repo playback / launch scripts that do not rely on an Apollo
  checkout

### 3.3 Why the contract-only `fast_lio2_core` is not enough by itself

Current `ProcessNext()` proves buffering discipline only:

- it validates extrinsic and synchronized IMU coverage
- it consumes one scan
- it advances `frames_processed`

But it does **not** yet do:

- undistortion
- propagation
- nearest search
- point-to-plane residual construction
- IEKF update
- map insertion
- publishable pose / cloud generation

So it is a safe contract boundary, not the full solver. The actual solver used
for Apollo drop-in runtime now lives under `modules/fast_lio2/core/`.

## 4. Standalone target architecture

The target repository layout should be:

```text
whl-FAST-LIO2/
  fast_lio2/
    common/
      types.h
    core/
      fast_lio2_core.h
      fast_lio2_core.cc
      imu_processing.h
      imu_processing.cc
      scan_matcher.h
      scan_matcher.cc
      ikd_map.h
      ikd_map.cc
      official_pipeline_contract.*
    cyber/
      fast_lio2_component.h
      fast_lio2_component.cc
      pointcloud_adapter.*
      imu_adapter.*
    proto/
      fast_lio2_conf.proto
      fast_lio2_metrics.proto
    evaluation/
      scorecard.*
    map_infra/
      optimized_frame_contract.h
      map_product_contract.*
  conf/
  dag/
  launch/
```

This separates:

- algorithm runtime
- Cyber adapter
- evaluation
- downstream map product contract

## 5. How `whl-FAST-LIO2` should be used for mapping

Once the standalone runtime is completed, the mapping flow should be:

### 5.1 Inputs

- one LiDAR topic with per-point timestamps
- one IMU topic
- canonical LiDAR-to-IMU extrinsic:
  - `p_imu = R_li * p_lidar + t_li`
- runtime config:
  - point filter count
  - blind distance
  - surface/map voxel size
  - max iterations
  - queue sizes
  - map publish interval

### 5.2 Runtime stages

```text
LiDAR/IMU buffering
  -> sync one scan + IMU slice
  -> IMU init / propagation / deskew
  -> local map FOV segmentation
  -> scan downsample
  -> scan-to-map IEKF update
  -> map_incremental
  -> publish outputs
```

### 5.3 Immediate outputs

During mapping, the module should publish:

- local pose
- odometry
- registered cloud
- local map cloud
- runtime metrics

These are mainly for monitoring and debugging.

### 5.4 Persistent outputs

For production mapping, the module should additionally write or hand off:

```text
optimized_frame_dataset/
  frames.csv
  pcd/frame_*.pcd
```

This is the canonical optimizer output.

Everything else should derive from it:

```text
optimized_frame_dataset
  -> tile_bundle
  -> localization_map_package
  -> map_product_bundle/manifest.json
```

## 6. What mapping can output

There are three different output layers, and they should not be confused.

### 6.1 Runtime observability outputs

- pose / odometry
- registered scan
- local accumulated map
- tracking state
- residual
- effective feature count
- latency
- drop counters

### 6.2 Optimizer handoff outputs

- frame timestamps
- frame poses in map frame
- frame-local undistorted point clouds

This is the correct handoff for renderer and localization-map builders.

### 6.3 Evaluation outputs

- input validity gates
- trajectory gates
- map-product consistency gates
- warnings for latency / frame drops

This is how baseline and regression should be tracked.

## 7. How to judge whether mapping is good

The minimum acceptance stack is:

1. input passes:
   - LiDAR timestamps valid
   - IMU rate and gravity magnitude valid
   - extrinsic direction correct
2. runtime passes:
   - tracking OK
   - reject reason OK
   - dropped IMU = 0
3. trajectory passes:
   - bounded RMSE / RPE
4. map passes:
   - residual stable
   - no double walls / ghosting
   - repeated structures remain sharp
5. map products pass:
   - tile and localization products share source fingerprint / frame / origin

## 8. Current practical use

Right now the practical split is:

- use `whl-FAST-LIO2/modules/fast_lio2` as the source-of-truth drop-in Apollo
  runtime
- copy it into `apollo-base/modules/fast_lio2` to build and launch
- use the rest of `whl-FAST-LIO2` to keep contracts, baselines, scorecards,
  and map-product acceptance rules under version control

This split is valid for current delivery. The remaining gap is not runtime
ownership anymore; it is build-system independence from an Apollo checkout.

## 9. Honest conclusion

If the requirement is:

```text
the saved source must no longer depend on Apollo calibration runtime libraries,
and it must be possible to drop it into Apollo and run
```

then that requirement is **now met**.

If the stricter requirement is:

```text
this repository alone, without an Apollo checkout, is a fully runnable workspace
```

then that stricter requirement is still not met.

The repository is now ready for:

- self-contained Apollo module delivery
- algorithm boundary review
- regression scorecards
- map-product contract enforcement
