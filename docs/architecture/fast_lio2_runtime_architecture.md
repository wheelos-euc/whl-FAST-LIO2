# FAST-LIO2 runtime architecture

## Repository boundaries

The repository has two related code areas:

- `modules/fast_lio2/` is the Apollo Cyber drop-in runtime: component,
  message adapters, runtime core, protobuf configuration, DAG/launch files,
  and Bazel targets.
- `fast_lio2/` contains framework-independent data types, pipeline and
  algorithm contracts, scorecard evaluation, and map-product contracts. It is
  not a second Cyber runtime.

The two trees are not connected through a delegated scan-to-map backend.
The top-level `FastLio2Core::ProcessNext()` currently validates and consumes
synchronized packages without estimating a pose. The Bazel-built Cyber
component calls `FastLio2RuntimeCore` directly; CMake builds the separate
contract/evaluation library. Consolidating these into a framework-independent
estimator remains unfinished work, not a reason to run both implementations.

The active runtime uses the pinned official IKFoM estimator described in the
[algorithm document](../algorithm/fast_lio2_algorithm_invariants.md).
The runtime API still includes PCL and protobuf output/configuration types;
canonical sensor inputs do not mean the entire runtime is framework-free.

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
Each consumed synchronized scan is exposed before the next is processed.
Pose and registered-cloud publication is scan-driven; IMU-only wakeups publish
diagnostic counters without duplicating those outputs.

The repository also defines contracts for `optimized_frame_dataset` and
downstream map products. A contract or successful short replay does not prove
that the runtime writes a complete dataset, that a trajectory is accurate, or
that a production map is qualified. Report those as separate implementation
and validation gates.

## Build boundary

This source workspace builds the Cyber component and Core's mainboard/recorder
tools together; it does not require a separate Apollo checkout. Hardware
drivers and installed/binary-distribution packaging are outside this delivery.
See the root [README](../../README.md) for the build and replay entry points.

Source development follows the Sensor Kit convention: `deps.repos` imports
only the required Core and Msgs `main` branches into ignored `pkgs/`;
`vcs pull pkgs` updates those checkouts, and `--config=dev` selects both
through `.bazelrc.dev` module overrides. WheelOS Fast-CDR/Fast-DDS modules use
the patched eProsima sources published in the WheelOS registry, at the versions
required by Core. They are not unpatched Git checkouts. Other dependency
versions must satisfy the upgraded modules' toolchain requirements; the old
Protobuf override is not retained. Common/Transform and hardware modules are
not dependencies of this runtime and are not imported.

The supported workflow is source development, not an assumed registry-only
release. Source `scripts/env.sh` in each terminal to select the built Core
tools and the module's CPU/local-SHM configuration; it does not edit Core's
source defaults or require an iceoryx daemon. Record the exact VCS revisions
with replay evidence; a moving branch is not a reproducible source pin.

The supplied profile uses one Cyber scheduler worker. During the upgraded
source validation, recorder startup with two workers hit an Abseil hash-table
assertion while subscribing to output channels; its cause is not established.
The single-worker profile is a tested workaround, not a general Core
concurrency fix or a real-time throughput qualification. FAST-LIO2 processing
still runs on the component's separate worker.

Core enforces its software-root boundary for DAG/configuration paths.
`scripts/env.sh` sets that root to this repository; generate launch assets
under repository-local `artifacts/`, not an unrelated absolute output path.

Wait for the component's started message and all selected recorder channels
before playback. Loading the upgraded shared library can outlast a fixed
startup sleep, losing early input without increasing component drop counters.
The cloud reader's pending queue and QoS history use the configured
`max_pending_pointcloud_frames` bound, rather than an independent depth of one.
Validate received counts against record counts as well as internal drop metrics.
