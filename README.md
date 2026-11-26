# whl-FAST-LIO2

FAST-LIO2 LiDAR/IMU mapping component for WheelOS Apollo Cyber.
It uses the official FAST-LIO `7cc4175` IKFoM estimator and ikd-tree.

Capabilities:

- LiDAR/IMU synchronization, initialization, motion compensation, pose
  estimation and incremental mapping
- Vanjee, raw GNSS IMU and corrected IMU inputs
- Pose, odometry, registered point clouds, local maps and diagnostics
- Apollo Cyber live operation or offline Cyber record replay

Loop closure, global pose optimization and production localization-map delivery
are not implemented.

## Quick start

Prerequisites: Linux, Python 3.10+, vcstool, C/C++ toolchain, CMake and Ninja.
Run commands from the repository root.

### 1. Import dependencies and build

```bash
mkdir -p pkgs
vcs import --shallow pkgs < deps.repos

tools/bazelw build --config=dev --jobs=2 \
  --per_file_copt='modules/fast_lio2/.*@-O3' \
  //modules/fast_lio2:libfast_lio2_component.so //:core_tools
```

`deps.repos` imports only Core and WheelOS Msgs into ignored `pkgs/`.
After dependency updates:

```bash
vcs status pkgs
vcs pull pkgs
tools/bazelw build --config=dev --jobs=2 \
  //modules/fast_lio2:libfast_lio2_component.so //:core_tools
```

This project uses Bazel 8.8.1. It uses no `git_override`; Core's transitive
Fast-CDR/Fast-DDS modules use the patched WheelOS registry versions.

### 2. Inspect the record and sensor configuration

```bash
source scripts/env.sh
cyber_recorder info /absolute/path/to/input.record.00000
mkdir -p data artifacts/record-map
cp modules/fast_lio2/conf/fast_lio2.pb.txt data/sensor.pb.txt
```

Edit `data/sensor.pb.txt`:

| Setting | Requirement |
| --- | --- |
| `pointcloud_topic`, `imu_topic`, `imu_message_type` | Must match the record channels and message types. |
| `timestamp_mode: MESSAGE_TIME` | LiDAR and IMU must use a verified acquisition-time basis. Do not substitute receive time. |
| `initial_rotation_lidar_to_imu_*` | Complete quaternion; `qw = 1` represents identity rotation. |
| `initial_translation_lidar_to_imu_*` | Metres, using `p_imu = R_li * p_lidar + t_li`. |
| `extrinsic_est_en` | `true` estimates from the initial extrinsic; `false` keeps it fixed. Neither option validates calibration. |

Coordinates are metres, angular velocity is rad/s and acceleration is m/s².
Do not replay with unknown clock, unit or frame semantics. See the
[sensor contract](docs/sensors/fast_lio2_sensor_contract.md).

### 3. Generate launch assets

```bash
source scripts/env.sh

python3 tools/prepare_apollo_host_run.py \
  --apollo_root "$PWD" \
  --config_template "$PWD/data/sensor.pb.txt" \
  --dag_template "$PWD/modules/fast_lio2/dag/fast_lio2.dag" \
  --module_library "$PWD/bazel-bin/modules/fast_lio2/libfast_lio2_component.so" \
  --mainboard_binary "$(command -v mainboard)" \
  --cyber_recorder_binary "$(command -v cyber_recorder)" \
  --record_path /absolute/path/to/input.record.00000 \
  --play_rate 0.25 \
  --output_dir "$PWD/artifacts/record-map"
```

Keep launch assets inside the repository. Current Core rejects
configuration and DAG paths outside its software root.

### 4. Record and replay

Open three terminals and source the environment in each:

```bash
source scripts/env.sh
```

Terminal 1: record the mapping outputs.

```bash
cyber_recorder record -o artifacts/record-map/mapping.record \
  -c /apollo/localization/fast_lio2/pose \
  -c /apollo/localization/fast_lio2/odometry \
  -c /apollo/localization/fast_lio2/metrics \
  -c /apollo/localization/fast_lio2/cloud_registered \
  -c /apollo/localization/fast_lio2/map
```

Terminal 2: start FAST-LIO2.

```bash
mainboard -d "$PWD/artifacts/record-map/fast_lio2.host.dag"
```

Wait for:

```text
FAST-LIO2 component started.
```

Terminal 3: confirm recorder subscriptions, then replay.

```bash
cyber_recorder play -f /absolute/path/to/input.record.00000 -r 0.25
```

Pass segmented input files in chronological order to `-f`.
After playback ends, let processing drain, stop the component, then stop the
output recorder.

## Results

Output segments are under `artifacts/record-map/mapping.record.*`.
Inspect every segment, not only `.00000`:

```bash
cyber_recorder info artifacts/record-map/mapping.record.00000
```

| Topic suffix | Output |
| --- | --- |
| `/pose`, `/odometry` | Scan-matched pose and odometry. |
| `/cloud_registered` | Scans already transformed into the map frame. |
| `/map` | Periodic local ikd-tree snapshots, not a complete map. |
| `/metrics` | Processed/matched counts, drops, residual, latency and extrinsic estimate. |

A complete map requires combining all registered scans.
PCD, trajectory and preview export is an offline step; the launch helper does
not export it automatically.

Keep private records and sensor profiles under `data/` and generated output
under `artifacts/`. Successful replay verifies runtime connectivity, not map
accuracy or extrinsic calibration.

## Development validation

```bash
tools/bazelw test --config=dev \
  //modules/fast_lio2:fast_lio2_runtime_core_test \
  //modules/fast_lio2:pointcloud_view_adapter_test \
  //modules/fast_lio2:vanjee_imu_adapter_test \
  //modules/fast_lio2:raw_imu_adapter_test \
  //modules/fast_lio2:corrected_imu_adapter_test
```

Reference documentation:

- [Documentation index](docs/README.md)
- [Runtime architecture](docs/architecture/fast_lio2_runtime_architecture.md)
- [Algorithm contract](docs/algorithm/fast_lio2_algorithm_invariants.md)
- [Sensor contract](docs/sensors/fast_lio2_sensor_contract.md)
