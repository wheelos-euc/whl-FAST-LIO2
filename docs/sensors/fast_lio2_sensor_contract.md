# FAST-LIO2 Sensor Contract

## Boundary

The Cyber component converts WheelOS messages into `FastLio2ImuSample` and
`FastLio2PointCloudFrame`. The FAST-LIO core accepts only these canonical
types; it does not inspect Cyber protobufs, receive timestamps, or
vendor-specific point fields.

## Required IMU semantics

| Field | Contract |
| --- | --- |
| Timestamp | Sensor acquisition time on the LiDAR time axis; strictly monotonic. |
| Angular velocity | Raw IMU body-frame measurement in rad/s. |
| Acceleration | Raw IMU body-frame specific force in m/s². |
| Extrinsic | Static LiDAR-to-IMU transform: `p_imu = R_li * p_lidar + t_li`. |

`CorrectedImu` is rejected unless
`corrected_imu_body_frame_verified: true` is explicitly configured. This flag
is an operator assertion, not a coordinate conversion. It may be enabled only
after checking the producer's frame and sensor-time semantics.

`VANJEE_IMU` accepts `apollo.drivers.vanjee.VanjeeImu` through a Cyber raw
reader and a local wire projection, avoiding a dependency on the hardware
driver or a duplicate registration of its protobuf message. Field numbers and
types in `proto/vanjee_imu_wire.proto` must match the driver's schema.
`measurement_time` is already in the selected LiDAR clock domain (Unix seconds
for the validated driver mode), not GPS-epoch seconds. The driver supplies
SDK-calibrated m/s² and rad/s in `header.frame_id`; no additional rotation,
acceleration scaling, or GPS offset is applied. Require a producer-verified
LiDAR-to-IMU extrinsic for qualified mapping; identity is only an explicit
diagnostic assumption when calibration is unavailable.

## Required LiDAR semantics

| Field | Contract |
| --- | --- |
| Coordinates | Finite `x/y/z` points in the LiDAR frame, metres. |
| Point timestamp | Nanoseconds in the selected Vanjee SDK clock domain; timestamps must be monotonic and their interval must align with the configured IMU time basis. The adapter does not normalize the epoch. Treat timestamps as Unix epoch only for specifically validated driver modes. |
| Scan time | First and last point define a positive scan interval; sensor-time mode requires the scan and IMU times to share a clock basis. |
| Deskew time | Adapter stores `point_time - scan_start` as milliseconds in PCL `curvature`, the upstream FAST-LIO convention. |

Ring, return type, tag, line, RGB, and map/global pose are not FAST-LIO core
inputs. A vendor adapter may consume them before emitting the canonical scan.

## Adapter responsibilities

Adapters may validate, reject, reorder only when their driver contract permits
it, and perform declared unit/frame conversions. They must not estimate pose,
bias, gravity, extrinsics, or a LiDAR-IMU time offset. Those operations belong
to the algorithm only when supported by the selected upstream FAST-LIO mode.

## Core responsibilities

The core uses the upstream-style sequence:

```text
IMU initialization → propagation and deskew → scan-to-map update
→ local-map insertion and eviction
```

It loads the configured extrinsic once at startup, does not run the prior
LiDAR-only/`LI_Init` calibration lifecycle, and does not terminate after a
refinement duration.
