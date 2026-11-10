# whl-FAST-LIO2

`whl-FAST-LIO2` is the Wheel.OS FAST-LIO2 refactor workspace.

It currently contains:

- FAST-LIO2 architecture contracts
- official algorithm invariants
- baseline scorecards and evaluation tooling
- map-product consistency contracts for downstream map generation

## Current status

`whl-FAST-LIO2` is now a standalone Apollo Cyber component workspace under:

```text
modules/fast_lio2/
```

The workspace pins the published `wheelos_core` through Bzlmod and
pins `wheelos_msgs` from
[`wheelos/wheelos_msgs`](https://github.com/wheelos/wheelos_msgs). It no longer
depends on Apollo Base's `modules/common_msgs`, adaptive point-cloud reader,
common utilities, or SLAM odometry proto.

Build with the repository wrapper, which reuses an installed matching Bazel
or installs the pinned binary in the user cache when needed:

```bash
tools/bazelw build //modules/fast_lio2:libfast_lio2_component.so
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

```

Validated again after the core modularization on 2026-05-21:

- Apollo drop-in build: passed
- Apollo `mainboard` smoke start: passed
- `core_contract_smoke`: passed
- `scorecard_contract_smoke`: passed
- `map_product_contract_smoke`: passed
- `official_pipeline_contract_smoke`: passed
- public synthetic scorecard: passed

## Core modularization status

The runtime-facing core is isolated from WheelOS/Cyber message definitions:

- `adapter/runtime_data.h`
- `proto/fast_lio2_runtime_conf.proto`
- `proto/fast_lio2_runtime_status.proto`
- `core/fast_lio2_runtime_core.h`
- `core/fast_lio2_runtime_core.cc`

`modules/fast_lio2/adapter/sensor_contract.{h,cc}` is the only boundary that
may interpret driver timestamps, units, and frames. The core uses the standard
IMU initialization, propagation, deskew, scan-to-map, and local-map eviction
path; it does not depend on `core/include/LI_init/`.

## Private data replay

Customer records, site-specific configurations, replay scripts, scorecards,
and assessment documents belong under the ignored `data/` directory. They are
not part of the public source tree or its Git history.

Runtime logs and result output are written to `/tmp/whl_fast_lio2`.

See `docs/sensors/fast_lio2_sensor_contract.md` for the required LiDAR/IMU driver
semantics. Corrected IMU input is deliberately blocked until its body-frame and
sensor-time semantics are verified.

See [`docs/README.md`](docs/README.md) for maintained project documentation.

### Six-axis IMU and record adaptation

The current `RAW_GNSS_IMU` input already consumes only three angular-velocity
and three acceleration components; it does not use attitude, magnetometer,
GNSS position, or an external pose. The enum names the Apollo protobuf type,
not a requirement for a GNSS-equipped sensor. A converted Livox record using
`apollo.drivers.gnss.Imu` can use this reader without changing the estimator.

Record playback uses `cyber_recorder play` and the same component as live
input. Select the actual record topics and verified LiDAR-to-IMU extrinsic in
a private config under `data/`; do not reuse site-specific defaults. Raw IMU
`measurement_time` must be GPS-epoch seconds (converted by adding 315964782),
angular velocity rad/s, acceleration m/s², and point timestamps monotonic
Unix nanoseconds. Do not apply a second acceleration conversion to an
already-SI record. Audit the record and initialization compatibility before
replay; successful message decoding is not evidence of map quality.

Vanjee records use `imu_message_type: VANJEE_IMU` and their actual driver
topics. Unlike `RAW_GNSS_IMU`, Vanjee `measurement_time` is passed through
without a GPS offset. See the sensor contract for the wire schema and frames.

The host asset generator accepts templates outside the Apollo workspace:

```bash
python3 tools/prepare_apollo_host_run.py \
  --apollo_root /path/to/apollo \
  --config_template /path/to/private/sensor.pb.txt \
  --dag_template modules/fast_lio2/dag/fast_lio2.dag \
  --module_library /path/to/libfast_lio2_component.so \
  --mainboard_binary /path/to/mainboard \
  --cyber_recorder_binary /path/to/cyber_recorder \
  --record_path /path/to/input.record \
  --output_dir /tmp/whl_fast_lio2/replay
```

This changes launch assets, queues and result paths only; algorithm parameters
in the selected config are preserved. The generated summary records both
template paths. The standalone DAG does not require Apollo's global flagfile.
