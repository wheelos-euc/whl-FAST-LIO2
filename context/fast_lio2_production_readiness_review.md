# FAST-LIO2 production readiness review

Date: 2026-05-21

## 1. Algorithm migration status

The Apollo runtime implementation preserves the official FAST-LIO2 algorithm order:

```text
SyncPackages
  -> ImuProcess::Process
  -> LasermapFovSegment
  -> voxel downsample
  -> ikd-tree init / nearest search
  -> iterated Kalman scan-to-map update
  -> MapIncremental
  -> publish/export outputs
```

Concrete code locations:

| Official stage | Current implementation |
| --- | --- |
| LiDAR/IMU sync | `modules/fast_lio2/core/fast_lio2_runtime_core.cc:306` |
| IMU propagation and undistortion | `modules/fast_lio2/core/imu_processing.h:284` and `:434` |
| local map FOV segmentation | `modules/fast_lio2/core/fast_lio2_runtime_core.cc:946` |
| voxel downsample | `modules/fast_lio2/core/fast_lio2_runtime_core.cc:450` |
| ikd-tree build/search | `modules/fast_lio2/core/fast_lio2_runtime_core.cc:455`, `:509` |
| point-to-plane residual + iterative update | `modules/fast_lio2/core/fast_lio2_runtime_core.cc:494-626` |
| incremental ikd-tree map update | `modules/fast_lio2/core/fast_lio2_runtime_core.cc:1013` |
| Apollo Cyber wrapper | `modules/fast_lio2/component/fast_lio2_component.cc` |

`whl-FAST-LIO2` now records this official stage contract in code:

```text
fast_lio2/core/official_pipeline_contract.{h,cc}
tests/official_pipeline_contract_smoke.cc
```

This is the submission boundary: adapters may change, but stage order and frame
semantics cannot. The runtime source now lives in `whl-FAST-LIO2`, and it is
validated by copying `modules/fast_lio2` into `apollo-base` for build/smoke
regression.

## 2. Optimizer output correctness

The optimizer output must be:

```text
optimized_frame_dataset/
  manifest.json
  frames.csv                 # map-frame pose per frame
  pcd/<bucket>/<frame>.pcd   # frame-local undistorted points
```

This is correct and production-friendly because:

1. renderer and localization map builder share the same source;
2. ray origins and frame timestamps are preserved;
3. point clouds are not double-transformed;
4. localization keyframes can be traced back to exact render frames;
5. QA can inspect bad local windows instead of only a stitched map.

The following are rejected designs:

- `pcd/` already in world/map frame while `frames.csv` also contains pose;
- stitched PCD as the only product;
- renderer and localization map builder reading different source datasets;
- tile map without localization-map package.

## 3. Unified map production flow

The production flow is now:

```text
record
  -> FAST-LIO2 backend
  -> optimized_frame_dataset/
  -> renderer -> tile_bundle/
  -> materialize_map_products.py -> localization_map_package/
  -> map_product_bundle/manifest.json
  -> scorecard
```

`map_product_bundle/manifest.json` is the acceptance artifact. It proves:

```text
tile_bundle.source_dataset_fingerprint
  == localization_map_package.source_dataset_fingerprint
  == optimized_frame_dataset fingerprint

tile_bundle.map_frame
  == localization_map_package.map_frame

tile_bundle.map_origin_id
  == localization_map_package.map_origin_id
```

If any field differs, the run is invalid.

## 4. Actual tools and tests

WHL:

```text
tools/evaluate_scorecard.py
tests/scorecard_metrics_sensor_rgb.json
tests/scorecard_metrics_zhongji.json
tests/map_product_contract_smoke.cc
tests/official_pipeline_contract_smoke.cc
```

map-infra:

```text
tile_pipeline/tools/materialize_map_products.py
tile_pipeline/tools/materialize_map_products_test.py
tile_pipeline/interface/map_product_bundle.h
tile_pipeline/interface/map_product_bundle_test.cc
tile_pipeline/optimizer/fast_lio2_optimizer_backend.*
```

## 5. Baselines

| Dataset | LiDAR/IMU | SE2 RMSE | Mean | Max | Runtime status |
| --- | --- | ---: | ---: | ---: | --- |
| `sensor_rgb.record` | Velodyne64 + raw GNSS IMU | 0.237 m | 0.209 m | 0.653 m | Correctness baseline, offline-performance warnings |
| `zhongji_20251009` | Vanjee left_front + corrected_imu | 0.124 m | 0.105 m | 0.309 m | OK |

Both baselines are represented as scorecard JSON and must pass:

```bash
python3 tools/evaluate_scorecard.py \
  --metrics_json tests/scorecard_metrics_sensor_rgb.json \
  --output_json /tmp/sensor_rgb_scorecard.json

python3 tools/evaluate_scorecard.py \
  --metrics_json tests/scorecard_metrics_zhongji.json \
  --output_json /tmp/zhongji_scorecard.json
```

Expected interpretation:

- `sensor_rgb.record` passes hard correctness gates but keeps warning-level latency/drop evidence;
- `zhongji_20251009` is the cleaner short dynamic baseline for map-product regression.

## 6. Production gates

Hard fail gates:

```text
LiDAR frame count >= 30
LiDAR rate >= 5 Hz
IMU rate >= 95 Hz
point timestamp span >= 0.01 s
extrinsic available
timestamps monotonic
sensor overlap >= 0.95
tracking OK
reject OK
dropped IMU == 0
effective features >= 500
map points >= 4000
mean residual <= 0.08
trajectory RMSE <= 0.50 m
path length ratio in [0.85, 1.15]
optimized_frame_dataset valid
map_products_consistent == true
```

Warnings:

```text
dropped LiDAR > 0
latency p95 > 200 ms
trajectory max error > 1.5 m
tile/localization artifacts missing
tile coverage < 0.90
```

Warnings block production deployment until triaged, even if algorithm correctness passes.

## 7. Submission assessment

The codebase now has:

- a validated Apollo Cyber FAST-LIO2 runtime;
- official FAST-LIO2 stage contract in `whl-FAST-LIO2`;
- scorecard-based baseline gates;
- map product consistency contracts;
- localization-map package materialization from the same optimized dataset used by renderer;
- side-by-side map-infra backend path without deleting the legacy optimizer.

Submission boundary:

- production runtime source is `whl-FAST-LIO2/modules/fast_lio2`, built and smoke-validated as an Apollo drop-in module;
- reusable architecture and regression contracts are stored in `whl-FAST-LIO2`;
- map product generation is source-consistent through `optimized_frame_dataset`, `tile_bundle`, `localization_map_package`, and `map_product_bundle`;
- any later internal library extraction must preserve the tested official stage contract and must not change public outputs or scorecard gates.
