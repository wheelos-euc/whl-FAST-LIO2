---
name: record-mapping-validation
description: Validate record-backed FAST-LIO2 runs without overstating map quality.
---

# Record mapping validation

Use this procedure when asked whether an Apollo Cyber record can run through
the FAST-LIO2 mapping path.

1. Read the [sensor contract](../../../docs/sensors/fast_lio2_sensor_contract.md)
   and inspect the current component/configuration. Identify the record topics,
   message types, timestamp mode, units, frames, per-point time semantics, and
   the configured LiDAR-to-IMU extrinsic.
2. Audit the selected record and configuration before playback. Treat unknown
   or mismatched time/frame/unit semantics as blockers; do not silently
   substitute receive time or identity extrinsics.
3. Run the smallest relevant unit/contract checks, then a bounded Cyber replay
   smoke test. Keep private records, site configurations, and generated outputs
   outside tracked source.
4. Report separate evidence for input validity, component/core progress,
   dropped or rejected messages, and generated outputs. A process start or
   short smoke run alone is not evidence of trajectory accuracy or map quality.
5. Before claiming mapping success, verify the expected algorithm stage was
   reached and inspect suitable trajectory/map-quality evidence. If that
   evidence is unavailable, state that the result is only a smoke test.

The full implementation plan is in the
[mapping plan](../../notes/FAST_LIO2_MAPPING_PLAN.md); it is a plan, not proof
that its proposed stages are implemented.
