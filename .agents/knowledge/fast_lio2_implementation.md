# FAST-LIO2 implementation map

The canonical architecture and algorithm contracts are maintained in
[docs/architecture](../../docs/architecture/fast_lio2_runtime_architecture.md)
and [docs/algorithm](../../docs/algorithm/fast_lio2_algorithm_invariants.md).

## Upstream-first changes

Preserve the pinned official FAST-LIO algorithm unless a necessary
Cyber, validated sensor-contract, or failure-handling boundary requires a
documented exception. Keep exceptions and their rationale in the canonical
algorithm document, not only in agent notes. Compare source provenance and
exercise the actual runtime; contract smoke tests alone are insufficient.
Do not silently reintroduce a custom estimator, weighted measurements,
different map index, or changed numerical defaults.

## Two trees, not two running estimators

- `modules/fast_lio2/` is the actual Bazel-built Cyber mapping runtime.
- `fast_lio2/` contains framework-independent interfaces, input/synchronization
  scaffolding, evaluation, and map-product contracts. Its `ProcessNext()` does
  not run scan-to-map or generate a pose. The Cyber runtime does not call it.
- CMake contract smoke programs do not validate the Cyber estimator.
- Do not delete either tree merely to remove the naming overlap. A future
  consolidation must preserve the evaluation and map-product interfaces and
  leave exactly one active estimator.

## Pre-restoration findings (2026-11-25)

The reference examined was official `hku-mars/FAST_LIO` commit
`7cc4175de6f8ba2edf34bab02a42195b141027e9`, not `LiDAR_IMU_Init`.
The runtime retained synchronization, IMU deskew, plane matching, and ikd-tree
mapping, but was not a middleware-only port:

- custom 24-dimensional state/filter instead of IKFoM's 23-dimensional error
  state with fixed-magnitude S2 gravity;
- custom iteration/covariance handling instead of IKFoM tangent transport;
- initialization after 200 samples and zero gyro bias instead of the upstream
  10-sample threshold and measured mean gyro bias;
- always-active extrinsic Jacobians without `extrinsic_est_en`;
- score-weighted normals/residuals instead of upstream unweighted measurements
  after correspondence gating;
- optimistic match-valid/OK reporting even after no effective correspondence;
- disabled historical LI_Init calibration code, not an active calibration.

These are historical findings, not a requirement to preserve the old behavior.
An apparently reasonable stitched map or low residual does not establish
upstream equivalence or absolute accuracy. Check the current canonical
algorithm document for the restoration status and remaining deviations.

## Restored active path

The runtime now uses pinned official IKFoM and its pinned ikd-tree submodule,
with upstream state/IMU/measurement semantics, numerical defaults, and the
`extrinsic_est_en` switch. The custom 24-state estimator and disabled runtime
LI_Init lifecycle have been removed from the active core. Scan processing
exposes each synchronized package before the next; missing IMU cannot retain
a previous scan's valid flag.
Remaining intentional differences and provenance belong in the canonical
algorithm document and `modules/fast_lio2/core/upstream_sources.txt`, not a
second agent-only algorithm specification. Runtime tests and private record
replay are distinct validation gates.

## Remaining differences: evolution reference (2026-11-26)

Reference FAST_LIO: `7cc4175de6f8ba2edf34bab02a42195b141027e9`;
ikd-tree: `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`.
This is a navigation summary of the
[canonical exceptions](../../docs/algorithm/fast_lio2_algorithm_invariants.md#upstream-first-policy-and-necessary-exceptions),
not a separate specification.

| Retained difference | Why it remains |
| --- | --- |
| Cyber/protobuf/PCLD adapters, configuration and output topics replace ROS/vendor interfaces. | Support the deployed runtime without changing estimator formulas. |
| Validated scan end and inclusive IMU endpoint replace duration heuristics/Livox self-sync. | Preserve verified acquisition time; unknown clock offsets remain blockers. |
| Complete finite explicit extrinsics are required. | Missing geometry must not become an implicit identity calibration. |
| Missing IMU invalidates the scan; failed/nonfinite measurement updates roll back and do not insert points. | Prevent stale-valid outputs and map contamination, not certify observability or accuracy. |
| IKFoM binds an instance-local `std::function` callback. | Avoid a process-global active estimator across Cyber component instances. |
| ROS/debug I/O and overwritten initialization variance calculations are omitted; empty data are initialized. | Remove framework coupling and undefined data without changing valid-input numerical behavior. |

Algorithm defaults follow upstream; explicit device profiles are overrides,
not algorithm forks. `extrinsic_est_en=true` treats supplied geometry as an
initial estimate; `false` retains it. Neither mode independently calibrates
or validates that geometry. Do not claim whole-ROS-deployment bitwise identity.

For an upstream upgrade, update the pin/provenance and canonical exception
table together. Check imported solver/manifold/tree sources, numerical
defaults and valid-input formulas; preserve justified boundary safeguards.
Run actual runtime/adapter regressions, then audited bounded and full record
replays with identical input semantics and explicit configuration. Keep
source equivalence, replay progress and trajectory/map accuracy as separate
claims; retain upstream licenses and do not reformat exact vendor imports.

User-facing replay instructions belong in the root README. The current
public helper generates launch assets, not a runner or PCD exporter.
Keep private records/profiles in `data/` and generated outputs in `artifacts/`;
do not make a public quick start depend on private replay/export scripts.

Source builds import only Core/Msgs with `deps.repos` into ignored `pkgs/`
and use `--config=dev`; retain patched registry DDS/CDR dependencies.
See the canonical architecture for software-root, startup-readiness,
transport-queue and single-worker-profile limitations. Always compare received
input counts with the record, not just component-internal drop counters.
