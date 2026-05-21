#include <cassert>

#include "fast_lio2/evaluation/scorecard.h"

int main() {
  whl::fast_lio2::evaluation::InputDataMetrics input;
  input.lidar_frame_count = 170;
  input.lidar_rate_hz = 10.0;
  input.imu_rate_hz = 100.0;
  input.point_time_span_sec = 0.1;
  input.has_lidar_to_imu_extrinsic = true;
  input.timestamps_monotonic = true;
  input.sensor_time_overlap_ratio = 0.99;

  whl::fast_lio2::evaluation::RuntimeMetrics runtime;
  runtime.tracking_ok = true;
  runtime.reject_ok = true;
  runtime.frames_processed = 166;
  runtime.effective_feature_count = 534;
  runtime.map_point_count = 4681;
  runtime.mean_point_to_plane_residual = 0.018;
  runtime.latency_p95_ms = 50.0;

  whl::fast_lio2::evaluation::TrajectoryMetrics trajectory;
  trajectory.matched_pose_count = 159;
  trajectory.ate_rmse_m = 0.124;
  trajectory.ate_mean_m = 0.105;
  trajectory.ate_max_m = 0.309;
  trajectory.path_length_ratio = 1.086;

  whl::fast_lio2::evaluation::MapQualityMetrics map;
  map.optimized_frame_dataset_valid = true;
  map.tile_bundle_valid = true;
  map.localization_map_package_valid = true;
  map.map_products_consistent = true;
  map.tile_coverage_ratio = 0.95;

  whl::fast_lio2::evaluation::BaselineThresholds thresholds;
  whl::fast_lio2::evaluation::ScorecardEvaluator evaluator;
  const auto scorecard =
      evaluator.Evaluate("zhongji_20251009", input, runtime, trajectory, map,
                         thresholds);
  assert(scorecard.Passed());
  assert(!scorecard.gates.empty());

  runtime.dropped_imu_messages = 1;
  const auto failed =
      evaluator.Evaluate("bad_imu_drop", input, runtime, trajectory, map,
                         thresholds);
  assert(!failed.Passed());
  return 0;
}
