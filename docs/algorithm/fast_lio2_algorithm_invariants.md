# FAST-LIO2 algorithm invariants

> Review snapshot dated 2026-05-21. This document records algorithm and frame
> invariants; it is not evidence that the current build has passed every
> runtime or mapping-quality gate.

Date: 2026-05-21

## 1. Review target

The WHL refactor must preserve the original FAST-LIO2 algorithm semantics while changing only engineering boundaries:

```text
ROS node / Apollo component / record player / renderer
  are adapters

IMU propagation / point undistortion / IEKF update / ikd-tree map update
  are the FAST-LIO2 algorithm core
```

This review uses the official `hku-mars/FAST_LIO` implementation as the reference for module boundaries and data semantics.

## Current estimator restoration (2026-11-25)

The active Cyber estimator now uses the IKFoM implementation and manifold
definitions pinned to official FAST_LIO commit
`7cc4175de6f8ba2edf34bab02a42195b141027e9`.
Provenance, upstream licensing, and the precise adaptation boundary are in
`modules/fast_lio2/core/upstream_sources.txt`.

The restored numerical path includes:

- the 23-dimensional error state with SO3 rotation/extrinsic and fixed-norm
  S2 gravity, rather than the former custom 24-dimensional state;
- upstream IMU initialization after the 10-sample threshold, measured mean
  gyro bias, initial covariance, midpoint prediction, and tail-IMU deskew;
- IKFoM `predict()` and `update_iterated_dyn_share_modified()`, including
  manifold covariance transport and the upstream per-state convergence limits;
- unweighted plane normals and point-to-plane residuals after score gating;
- `extrinsic_est_en`, defaulting to upstream's `true`. Setting it to `false`
  disables extrinsic measurement Jacobians; startup extrinsics are initial
  values when estimation is enabled, not a separately validated calibration;
- local-map movement referenced to the LiDAR position, including the lever arm.
- the official `ikd-Tree` submodule at
  `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`, with byte-identical
  `ikd_Tree.h` and `ikd_Tree.cpp`, replacing the older local non-template tree;
- the original score literal precision and positive comparison (which rejects
  NaN), map/voxel arithmetic precision, and matching with exactly five points
  after a map has been seeded; map seeding still requires more than five.

IKFoM/MTK headers and `use-ikfom.hpp` are imported from the pinned source.
The only IKFoM toolkit patch changes the measurement callback to `std::function`
so multiple component instances do not require a shared global estimator.
IMU processing and the measurement model are adapted to canonical inputs;
they are not an untouched ROS node.

### Upstream-first policy and necessary exceptions

Preserve the pinned official algorithm unless a necessary middleware,
validated sensor-contract, or failure-handling boundary requires a documented
exception. Do not retain a different estimator, map-index revision, gate,
numerical precision, or implicit parameter default without such a reason.

The public Cyber and internal core algorithm defaults now match upstream
`laserMapping.cpp` ROS parameter defaults: iterations 4, surface/map voxels
0.5/0.5 m, cube 200 m, detection range 300 m, point decimation 2, blind range
0.01 m, and gyro/acceleration noise 0.1. These are provenance defaults, not a
qualified sensor profile. Explicit values in existing configurations remain
unchanged and take precedence; aligning source does not justify blindly
retuning a verified record or device profile.

The remaining exceptions and reasons are:

| Boundary | Reason |
| --- | --- |
| Cyber message/configuration/output types and canonical sensor adapters replace ROS/vendor preprocessing. | The supported runtime is Apollo Cyber; timestamps, units, frames and per-point time must satisfy the sensor contract rather than guess another driver's layout. |
| Validated scan end replaces upstream mean-duration heuristics and Livox self-sync; extraction includes all IMUs at or before that endpoint. | Acquisition-time evidence must not be silently replaced or an exact endpoint discarded. Unknown clock offsets remain a preflight blocker. |
| Complete finite explicit extrinsics are mandatory. | Missing geometry must not masquerade as an identity calibration. |
| The IKFoM callback uses `std::function` rather than a global function pointer. | Independent Cyber component instances must bind their own measurement model without a shared active-estimator global. |
| Missing IMU, failed/nonfinite updates and insufficient points invalidate the current scan; failed measurement updates roll back to IMU prediction and do not insert points. | Do not publish a stale/invalid estimate as valid or contaminate the map. This does not constitute a complete observability/quality gate. |
| ROS/debug I/O and overwritten initialization sample-variance calculations are omitted; histories and empty outputs are initialized. | Remove framework coupling and undefined/stale data without changing configured propagation noise or valid-input formulas. |

`Process()` consumes one synchronized package and returns whether one was
consumed, including initialization/rejection. The component publishes before
processing the next package, as in the upstream per-scan loop. IMU-only
wakeups update diagnostic counters without re-publishing poses/clouds; their
processing latency is zero because no scan was processed. The body-cloud
accessor now applies LiDAR-to-IMU extrinsics into IMU `body`, matching upstream,
rather than labeling LiDAR-local points as the upstream body output.

Consequently, this is an official-algorithm Cyber adaptation, not a claim of
bitwise identity with an entire upstream ROS deployment or identical results
under different preprocessing/configuration.

`fast_lio2_runtime_core_test` exercises stationary biased-IMU initialization,
both extrinsic-estimation modes, unmatched-scan rejection/recovery, and a
synthetic moving-room trajectory with per-point motion distortion. It also
covers individually exposed buffered scans, missing-IMU invalidation/recovery,
five-point matching, NaN score rejection, IMU-body cloud transformation, and
finite-radius nearest search in the pinned tree. These are estimator
regressions; the top-level CMake contract tests alone do not
exercise this runtime. Record replay evidence and generated maps belong under
ignored `artifacts/`, never in tracked project documentation.

## 2. Official algorithm sequence

The official `src/laserMapping.cpp` flow is:

```text
sync_packages()
  -> collect one LiDAR scan and IMU samples until lidar_end_time

ImuProcess::Process()
  -> IMU initialization
  -> forward propagation
  -> point undistortion
  -> output feats_undistort in body/LiDAR frame

lasermap_fov_segment()
  -> keep local map cube around current pose

voxel downsample
  -> feats_down_body

ikdtree initialization or nearest search

kf.update_iterated_dyn_share_modified()
  -> h_share_model()
     -> transform body point to map/world using:
        p_global = R_wi * (R_li * p_lidar + t_li) + p_wi
     -> nearest search in ikd-tree
     -> plane fitting
     -> point-to-plane residual Jacobian

map_incremental()
  -> transform accepted downsampled points to map/world
  -> add points to ikd-tree with voxel gating

publish odometry / registered cloud / optional map
```

WHL must keep this sequence intact. The refactor can change class names, config loading, and adapters, but not the data order or frame math.

## 3. Official frame and extrinsic convention

Official FAST-LIO2 uses:

```text
p_imu = R_L_I * p_lidar + T_L_I
p_map = R_map_imu * p_imu + T_map_imu
```

The README states that the extrinsic is the LiDAR pose in the IMU body frame. The official measurement model in `h_share_model()` applies `offset_R_L_I` and `offset_T_L_I` before the state pose.

WHL convention must therefore remain:

```text
ExtrinsicLidarToImu:
  p_imu = R_li * p_lidar + t_li
```

The Apollo `/tf_static` adapter may invert or compose TFs, but the core must only receive this canonical LiDAR-to-IMU transform.

## 4. What can be refactored

Safe refactor boundaries:

| Official responsibility | WHL module |
| --- | --- |
| ROS topic callbacks | Apollo/Cyber adapter |
| `sync_packages()` buffering | framework-independent synchronizer or adapter |
| `ImuProcess` | `fast_lio2/core` |
| IEKF state and update | `fast_lio2/core` |
| ikd-tree local map | `fast_lio2/core` |
| publish odometry/cloud/map | Apollo adapter and map-infra writer |
| PCD save | map product writer |

Unsafe changes:

- changing LiDAR-to-IMU extrinsic direction;
- feeding world-frame points back as frame-local points;
- using frame-level timestamps when point-level time exists;
- evaluating map quality only with a stitched PCD;
- making renderer consume FAST-LIO2-specific outputs.

## 5. Optimizer output review

Question: optimizer output is "aligned pose and point cloud"; is this correct for mapping?

Correct design:

```text
optimized_frame_dataset/
  frames.csv: optimized pose of each frame in map frame
  pcd/: undistorted frame-local points for each frame
```

This is correct because:

1. renderer can reconstruct world points with `point_map = T_map_lidar * point_local`;
2. localization map builder can use the exact same keyframes and poses;
3. frame provenance, timestamps, ray origins, and diagnostics are preserved;
4. tile map and localization map remain traceable to the same source.

Incorrect design:

```text
frames.csv: optimized pose
pcd/: already-world/aligned points
renderer applies pose again
```

This double-transforms points and breaks consistency.

Also incorrect:

```text
stitched_map.pcd only
```

This loses frame origins and cannot build a reliable localization map or raycasted tile map.

## 6. Unified production map product

FAST-LIO2 must output one canonical source:

```text
optimized_frame_dataset
```

Both map products must derive from that exact source:

```text
optimized_frame_dataset
  -> tile_map_product / tile_bundle
  -> localization_map_product / localization_map_package
  -> map_product_bundle manifest
```

All three manifests must share:

```text
source_dataset_fingerprint
map_frame
map_origin_id
```

If any differ, the build is invalid because the displayed high-definition base map may no longer correspond to the localization map used online.

## 7. Production acceptance

FAST-LIO2 can be considered a production optimizer replacement only when:

1. official algorithm order is preserved;
2. LiDAR/IMU synchronization and point timestamps pass input gates;
3. trajectory scorecard passes;
4. optimized_frame_dataset schema passes;
5. tile and localization map products share source fingerprint, map frame, and map origin;
6. online localizer can load the localization map and report pose in the same `map` frame rendered by the tile map.

