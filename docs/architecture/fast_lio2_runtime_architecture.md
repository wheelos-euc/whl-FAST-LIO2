# FAST-LIO2 runtime architecture

## Repository boundaries

The repository has two related code areas:

- `modules/fast_lio2/` is the Apollo Cyber drop-in runtime: component,
  message adapters, runtime core, protobuf configuration, DAG/launch files,
  and Bazel targets.
- `fast_lio2/` contains framework-independent data types, pipeline and
  algorithm contracts, scorecard evaluation, and map-product contracts. It is
  not a second Cyber runtime.

The Cyber component converts incoming messages into canonical runtime data
before passing them to the FAST-LIO2 core. Vendor-specific timestamps, units,
frames, and point fields belong at the adapter boundary. The algorithm and
coordinate conventions are described in the
[sensor contract](../sensors/fast_lio2_sensor_contract.md) and
[algorithm invariants](../algorithm/fast_lio2_algorithm_invariants.md).

## Record and online input

Apollo Cyber record playback uses `cyber_recorder play` to publish the same
Cyber messages consumed by the component's readers. It does not introduce a
separate estimator or a record-specific algorithm path. ROS1 bags are a
different format and are not read directly by this runtime.

## Runtime and map-product status

The Cyber runtime publishes localization/odometry and mapping observability
outputs such as registered point clouds, local map data, and metrics. These
runtime outputs are not by themselves a persistent, validated map product.

The repository also defines contracts for `optimized_frame_dataset` and
downstream map products. A contract or successful short replay does not prove
that the runtime writes a complete dataset, that a trajectory is accurate, or
that a production map is qualified. Report those as separate implementation
and validation gates.

## Build boundary

This repository is a source workspace for an Apollo drop-in module, not a
complete standalone Apollo distribution. It builds and runs within a
compatible Apollo/Cyber environment; see the root [README](../../README.md)
for the current build entry point.
