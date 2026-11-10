# FAST-LIO2 submission review and validation plan

Date: 2026-05-21

## 1. Submission scope

This submission contains two different layers and they should not be described as the same thing:

| Repository | Submission scope |
| --- | --- |
| `whl-FAST-LIO2` | self-contained Apollo drop-in FAST-LIO2 runtime module, official stage invariants, scorecard/baseline artifacts |
| `map-infra` | map-product consistency contracts, renderer/materializer tooling, and release-side acceptance artifacts |

Therefore the valid submission description is:

```text
whl-FAST-LIO2 = self-contained Apollo drop-in FAST-LIO2 runtime + baseline/contract repository
map-infra = downstream map-product contract and materialization repository
```

Do not describe `whl-FAST-LIO2` as a complete standalone Apollo workspace yet.
It contains the extracted module source, but execution is still validated by
copying `modules/fast_lio2` into an Apollo workspace such as `apollo-base`.

## 2. Code review findings addressed before submission

### 2.1 Extracted runtime boundary

`whl-FAST-LIO2/modules/fast_lio2` now contains the extracted Apollo drop-in
runtime with FAST-LIO2-native module names:

- `adapter/runtime_data.h`
- `proto/fast_lio2_runtime_conf.proto`
- `proto/fast_lio2_runtime_status.proto`
- `core/fast_lio2_runtime_core.h`
- `core/fast_lio2_runtime_core.cc`

The module-facing `apollo::calibration::*` runtime namespace has been removed.

### 2.2 Scorecard gate-ID mismatch

The C++ and Python scorecard implementations now use aligned gate identifiers such as:

```text
input.has_lidar_to_imu_extrinsic
input.sensor_time_overlap_ratio
runtime.mean_point_to_plane_residual
map.optimized_frame_dataset_valid
map.localization_map_package_valid
map.map_products_consistent
```

This avoids broken CI/baseline comparisons caused by divergent gate names.

### 2.3 Baseline evidence overstatement

`sensor_rgb` scorecard input was corrected to reflect the documented runtime evidence:

```text
dropped_lidar_frames = 78
latency_p50_ms ~= 982
latency_p95_ms > 200
```

So:

- `sensor_rgb` remains a correctness baseline;
- it is no longer misrepresented as a clean real-time baseline.
- the documented `zhongji_20251009` regression artifact is now also checked in as `tests/scorecard_metrics_zhongji.json`, so the documented scorecard commands match repository contents again.

### 2.4 Tile/localization source consistency risk

In the paired `map-infra` repository, `materialize_map_products.py` now:

- requires `tile_bundle/manifest.json`;
- verifies `tile_bundle.manifest.source_dataset == optimized_frame_dataset`;
- checks tile frame count against the dataset;
- computes dataset fingerprint from actual PCD bytes, not file size only.

This closes the earlier loophole where an unrelated tile bundle could be relabeled as consistent with a different dataset.

## 3. Detailed submission advice

### 3.1 What is ready to submit

Submit:

- `whl-FAST-LIO2` extracted runtime module, scorecards, baseline JSONs, official stage audit, and production-readiness docs;
- `map-infra` backend bridge, map-product consistency contracts, renderer fixes, and `materialize_map_products.py`;
- updated renderer/contract docs that clearly separate:
  - renderer handoff,
  - localization-map package,
  - consistency bundle.

### 3.2 What must not be overclaimed in commit/PR text

Do **not** claim:

- “full standalone FAST-LIO2 runtime extracted into `whl-FAST-LIO2`”;
- “localization map builder complete”;
- “production real-time verified on all baselines”.

Accurate claim:

```text
Validated Apollo drop-in FAST-LIO2 runtime + reproducible baseline/scorecard + source-consistent tile/localization map packaging
```

### 3.3 High-signal architectural recommendations

1. Keep `optimized_frame_dataset` as the only optimizer->downstream source.
2. Never let renderer consume stitched maps or world-frame PCD + pose pairs as a second production contract.
3. Keep `map_product_bundle/manifest.json` mandatory for release artifacts.
4. Promote `zhongji_20251009` as the first clean regression set and keep `sensor_rgb` as a known-warning correctness set.
5. When the runtime is further refactored, preserve:
   - extrinsic direction,
   - sync order,
   - point undistortion order,
   - ikd-tree insertion order,
   - scan-to-map residual definition.

## 4. Commit/merge checklist

Before commit:

1. Re-run in `whl-FAST-LIO2`:
   - `cmake --build /tmp/whl_fast_lio2_build --target core_contract_smoke scorecard_contract_smoke map_product_contract_smoke official_pipeline_contract_smoke`
   - `python3 tools/evaluate_scorecard.py --metrics_json tests/scorecard_metrics_sensor_rgb.json --output_json /tmp/sensor_rgb_scorecard.json`
   - `python3 tools/evaluate_scorecard.py --metrics_json tests/scorecard_metrics_zhongji.json --output_json /tmp/zhongji_scorecard.json`
2. Re-run in `map-infra`:
   - `python3 tile_pipeline/tools/materialize_map_products_test.py`
   - `bazel test //tile_pipeline/renderer/tools:tile_bundle_reporter_contract_test //tile_pipeline/renderer/tools:optimized_dataset_ingestor_contract_test`
3. Ensure no `__pycache__` directories are present.
4. Ensure PR text states `whl-FAST-LIO2` is a drop-in Apollo module validated inside Apollo, not a full standalone Apollo workspace.
5. Attach both baseline scorecards and the map-product bundle rationale.

## 5. Next validation and test plan

### Immediate pre-merge validation

1. Build Apollo runtime:
   - `bazel build //modules/fast_lio2:libfast_lio2_component.so`
2. Replay `sensor_rgb.record` and `zhongji_20251009`.
3. Export:
   - `optimized_frame_dataset/`
   - `tile_bundle/`
   - `localization_map_package/`
   - `map_product_bundle/`
4. Run scorecards and archive results under release artifacts.

### Near-term regression farm

Use at least:

```text
sensor_rgb.record
中集/2025年10月9日...
中集/2025-12-29/obstacle/20251229155810.record.00000
```

Track:

- trajectory RMSE / mean / max;
- feature count;
- residual;
- frame drops;
- latency p50/p95;
- tile/localization source consistency;
- localization package artifact completeness.

### Pre-production additions

Before claiming full production mapping/localization:

1. add a dedicated localization-map builder that writes dense/NDT indices under the same bundle manifest;
2. run long-sequence drift and repeat-pass overlap tests;
3. run localization-map load tests in the online localizer;
4. validate `T_map_lio` handling against the same map tiles shown to operators.
