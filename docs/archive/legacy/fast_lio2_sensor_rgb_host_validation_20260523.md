# FAST-LIO2 sensor_rgb host replay validation

Date: 2026-05-23

## Summary

`whl-FAST-LIO2/modules/fast_lio2` was validated again inside
`/home/wfh/01code/apollo-base` by copying the module into Apollo, building the
drop-in target, and replaying `/mnt/synology/apollo/sensor_rgb.record`.

The core engineering finding is:

- default config is sufficient for correctness and startup;
- default replay pacing is **not** sufficient for full offline throughput on the
  tested host;
- the validated host replay recipe is:
  - `max_pending_pointcloud_frames = 256`
  - `cyber_recorder play -r 0.1`

## Build and runtime facts

- Build target:
  - `//modules/fast_lio2:libfast_lio2_component.so`
- Runtime executable paths are under Apollo Bazel cache, not repository-root
  `bazel-bin/`:
  - `.../bazel-out/k8-fastbuild/bin/cyber/mainboard/mainboard`
  - `.../bazel-out/k8-fastbuild/bin/cyber/tools/cyber_recorder/cyber_recorder`
- Host-mode DAG must use the actual cached shared library path instead of
  `/apollo/bazel-bin/...`.

## Replay result

Validated runtime outputs:

- `/apollo/localization/fast_lio2/pose`
- `/apollo/localization/fast_lio2/odometry`
- `/apollo/localization/fast_lio2/metrics`
- `/apollo/localization/fast_lio2/cloud_registered`

Recorded output counts:

- pose: `1809`
- odometry: `1809`
- metrics: `1809`
- cloud_registered: `1809`

Runtime log observation:

- `586` `scan2map normal result` lines after slow replay and larger pending
  queue

## Reusable tooling added

`tools/prepare_apollo_host_run.py` now generates:

- `fast_lio2.host.pb.txt`
- `fast_lio2.host.dag`
- `host_run_summary.json`

from an Apollo workspace root, so host-mode replay no longer depends on ad-hoc
manual editing of `/tmp/*.dag` and `/tmp/*.pb.txt`.

For real Apollo workspaces with multiple Bazel output candidates, the validated
usage is to pass explicit `--module_library`, `--mainboard_binary`, and
`--cyber_recorder_binary` paths instead of relying on cache auto-detection.

## Important caveat

This validation proves the module is reproducibly runnable as an Apollo drop-in
module on host replay, but it does **not** prove clean real-time throughput at
1.0x playback on the tested machine. The slow replay recipe is still the
recommended correctness-validation path.
