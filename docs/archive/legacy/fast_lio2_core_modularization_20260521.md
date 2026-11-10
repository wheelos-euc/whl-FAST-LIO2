# FAST-LIO2 core modularization review

Date: 2026-05-21

## 1. Goal

Remove the last calibration-module influence from `modules/fast_lio2/` so the
drop-in runtime is expressed as a FAST-LIO2 module rather than a renamed
calibration extraction.

## 2. What changed

The refactor replaced the remaining calibration-facing runtime boundaries:

- `adapter/calibration_data.h` -> `adapter/runtime_data.h`
- `proto/lidar_imu_calibration_conf.proto` -> `proto/fast_lio2_runtime_conf.proto`
- `proto/lidar_imu_calibration_status.proto` -> `proto/fast_lio2_runtime_status.proto`
- `core/lidar_imu_calibration_core.h` -> `core/fast_lio2_runtime_core.h`
- `core/lidar_imu_calibration_core.cc` -> `core/fast_lio2_runtime_core.cc`

It also removed `apollo::calibration::*` usage from the module-facing code:

- adapters now live in `apollo::localization::fast_lio2`
- component queues use `FastLio2PointCloudFrame` / `FastLio2ImuSample`
- the runtime config type is `FastLio2RuntimeConf`
- the runtime status type is `FastLio2RuntimeStatus`
- the runtime class is `FastLio2RuntimeCore`

## 3. What did not change

The FAST-LIO2 algorithm path was intentionally preserved:

1. sync one scan with its IMU window
2. IMU propagation / undistortion
3. local map FOV maintenance
4. voxel downsample
5. ikd-tree nearest search + plane fit + iterative update
6. incremental map insertion

The refactor is therefore a naming/module-boundary cleanup, not a mathematical
rewrite.

## 4. Remaining upstream naming

`core/include/LI_init/` still contains upstream initialization helper naming
such as `LI_Init` and `CalibState`.

That is acceptable and intentional because:

- it is part of the imported FAST-LIO2-side helper implementation
- it no longer points at Apollo calibration modules
- changing it further would create unnecessary divergence from the reviewed
  upstream-derived helper path

So the post-refactor rule is:

```text
module boundary names = FAST-LIO2-native
internal imported helper names = preserved where tied to upstream code
```

## 5. Verified regression

Validated after the refactor:

### Apollo drop-in runtime

- synced `whl-FAST-LIO2/modules/fast_lio2` into `apollo-base/modules/fast_lio2`
- `bazel build //modules/fast_lio2:libfast_lio2_component.so` : passed
- local `mainboard` smoke start: passed

### WHL contract and scorecard checks

- `cmake -S . -B /tmp/whl_fast_lio2_build` : passed
- `cmake --build /tmp/whl_fast_lio2_build -j` : passed
- `core_contract_smoke` : passed
- `scorecard_contract_smoke` : passed
- `map_product_contract_smoke` : passed
- `official_pipeline_contract_smoke` : passed
- `sensor_rgb` scorecard : pass, 0 fail, 2 warning
- `zhongji_20251009` scorecard : pass, 0 fail, 1 warning

## 6. Final architectural state

The honest description of the module is now:

```text
self-contained Apollo drop-in FAST-LIO2 runtime
+ FAST-LIO2-native module boundaries
+ baseline / scorecard tooling
+ map-product handoff contracts
```

This is still **not** a claim that the repository is a complete standalone
Apollo workspace, and it is still **not** a claim that the localization map
package is already a finished online localization runtime map.
