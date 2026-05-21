# whl-FAST-LIO2

`whl-FAST-LIO2` is the Wheel.OS FAST-LIO2 refactor workspace.

It currently contains:

- FAST-LIO2 architecture contracts
- official algorithm invariants
- baseline scorecards and evaluation tooling
- map-product consistency contracts for downstream map generation

## Current status

`whl-FAST-LIO2` now contains a **self-contained Apollo drop-in module** under:

```text
modules/fast_lio2/
```

This extracted module:

- contains its own Cyber component
- contains its own adapters, core, and internal proto files
- no longer depends on `modules/calibration/lidar_imu_calibration`
- now exposes only FAST-LIO2-native runtime names inside `modules/fast_lio2/`
- can be copied into `apollo-base/modules/fast_lio2` and built/launched there

At the same time, this repository itself is still **not** a full standalone
Apollo workspace. It is best described as:

```text
drop-in Apollo module + evaluation + source-of-truth repository
```

## What a complete standalone version must contain

A complete runnable delivery consists of:

1. Cyber component and proto config
2. LiDAR / IMU input adapters
3. FAST-LIO2 runtime core:
   - synchronization
   - IMU initialization and propagation
   - point undistortion
   - local map FOV management
   - ikd-tree nearest search
   - iterated Kalman scan-to-map update
   - incremental map insertion
4. Runtime outputs and map writers
5. Evaluation / scorecard tools

The long-term goal is:

```text
whl-FAST-LIO2/
  fast_lio2/core        <- full algorithm runtime
  fast_lio2/cyber       <- Apollo Cyber adapter
  fast_lio2/evaluation  <- baseline and scorecard
  fast_lio2/map_infra   <- optimized_frame_dataset contracts
```

## Mapping workflow

The intended mapping flow is:

```text
LiDAR + IMU + lidar-to-imu extrinsic
  -> FAST-LIO2 sync / undistort / scan-to-map
  -> local odometry + registered scan + local map
  -> optimized_frame_dataset
  -> tile map + localization map package
  -> scorecard / regression report
```

## Outputs

The end-to-end mapping stack is expected to produce:

### Runtime outputs

- pose
- odometry
- registered cloud
- map cloud
- health / residual / latency metrics

### Offline map artifacts

- `optimized_frame_dataset/`
  - `frames.csv`
  - `pcd/*.pcd`
- `tile_bundle/`
- `localization_map_package/`
- `map_product_bundle/manifest.json`

### Evaluation artifacts

- scorecard JSON
- pass / fail / warning gates
- baseline comparison report

## Current evaluation commands

```bash
cmake -S . -B /tmp/whl_fast_lio2_build
cmake --build /tmp/whl_fast_lio2_build -j

/tmp/whl_fast_lio2_build/core_contract_smoke
/tmp/whl_fast_lio2_build/scorecard_contract_smoke
/tmp/whl_fast_lio2_build/map_product_contract_smoke
/tmp/whl_fast_lio2_build/official_pipeline_contract_smoke

python3 tools/evaluate_scorecard.py \
  --metrics_json tests/scorecard_metrics_sensor_rgb.json \
  --output_json /tmp/sensor_rgb_scorecard.json

python3 tools/evaluate_scorecard.py \
  --metrics_json tests/scorecard_metrics_zhongji.json \
  --output_json /tmp/zhongji_scorecard.json
```

Validated again after the core modularization on 2026-05-21:

- Apollo drop-in build: passed
- Apollo `mainboard` smoke start: passed
- `core_contract_smoke`: passed
- `scorecard_contract_smoke`: passed
- `map_product_contract_smoke`: passed
- `official_pipeline_contract_smoke`: passed
- `sensor_rgb` / `zhongji_20251009` scorecards: passed

## Core modularization status

The runtime-facing core no longer uses calibration-module naming:

- `adapter/runtime_data.h`
- `proto/fast_lio2_runtime_conf.proto`
- `proto/fast_lio2_runtime_status.proto`
- `core/fast_lio2_runtime_core.h`
- `core/fast_lio2_runtime_core.cc`

Imported upstream helper code under `core/include/LI_init/` still keeps the
authors' original initialization naming, but it is now an internal FAST-LIO2
implementation detail rather than a dependency on Apollo calibration code.

## Important limitation

## Drop-in usage inside Apollo

Copy:

```text
whl-FAST-LIO2/modules/fast_lio2
  -> apollo-base/modules/fast_lio2
```

Then build:

```bash
cd /path/to/apollo-base
bazel build //modules/fast_lio2:libfast_lio2_component.so
```

Then launch with Apollo mainboard / launch tooling using:

- `modules/fast_lio2/dag/fast_lio2.dag`
- `modules/fast_lio2/conf/fast_lio2.pb.txt`

This repository remains the right place to:

- keep the self-contained module source
- maintain scorecards and baselines
- maintain mapping / map-product contracts
