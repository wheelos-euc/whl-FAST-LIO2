#include "fast_lio2/evaluation/scorecard.h"

#include <cmath>

namespace whl {
namespace fast_lio2 {
namespace evaluation {

namespace {

bool IsFinite(double value) { return std::isfinite(value); }

GateStatus PassFail(bool passed, GateSeverity severity) {
  if (passed) {
    return GateStatus::kPass;
  }
  return severity == GateSeverity::kWarn ? GateStatus::kWarn : GateStatus::kFail;
}

}  // namespace

bool Scorecard::HasFailures() const {
  for (const auto& gate : gates) {
    if (gate.status == GateStatus::kFail) {
      return true;
    }
  }
  return false;
}

bool Scorecard::Passed() const { return !HasFailures(); }

Scorecard ScorecardEvaluator::Evaluate(
    const std::string& dataset_id, const InputDataMetrics& input,
    const RuntimeMetrics& runtime, const TrajectoryMetrics& trajectory,
    const MapQualityMetrics& map, const BaselineThresholds& thresholds) const {
  Scorecard scorecard;
  scorecard.dataset_id = dataset_id;
  scorecard.input = input;
  scorecard.runtime = runtime;
  scorecard.trajectory = trajectory;
  scorecard.map = map;
  scorecard.thresholds = thresholds;

  auto& gates = scorecard.gates;
  gates.push_back(MinGate("input.lidar_frame_count", GateSeverity::kFail,
                          input.lidar_frame_count,
                          thresholds.min_lidar_frame_count,
                          "LiDAR frame count must cover a useful segment"));
  gates.push_back(MinGate("input.lidar_rate_hz", GateSeverity::kFail,
                          input.lidar_rate_hz, thresholds.min_lidar_rate_hz,
                          "LiDAR rate must be high enough for LIO"));
  gates.push_back(MinGate("input.imu_rate_hz", GateSeverity::kFail,
                          input.imu_rate_hz, thresholds.min_imu_rate_hz,
                          "IMU rate must support deskew and propagation"));
  gates.push_back(MinGate("input.point_time_span_sec", GateSeverity::kFail,
                          input.point_time_span_sec,
                          thresholds.min_point_time_span_sec,
                          "LiDAR points need non-constant per-point time"));
  gates.push_back(BoolGate("input.has_lidar_to_imu_extrinsic", GateSeverity::kFail,
                           input.has_lidar_to_imu_extrinsic,
                           "LiDAR-to-IMU extrinsic is required"));
  gates.push_back(BoolGate("input.timestamps_monotonic", GateSeverity::kFail,
                           input.timestamps_monotonic,
                           "Sensor timestamps must be monotonic"));
  gates.push_back(MinGate("input.sensor_time_overlap_ratio", GateSeverity::kFail,
                          input.sensor_time_overlap_ratio,
                          thresholds.min_sensor_time_overlap_ratio,
                          "LiDAR/IMU/reference time windows must overlap"));

  gates.push_back(BoolGate("runtime.tracking_ok", GateSeverity::kFail,
                           runtime.tracking_ok,
                           "FAST-LIO2 tracking status must remain OK"));
  gates.push_back(BoolGate("runtime.reject_ok", GateSeverity::kFail,
                           runtime.reject_ok,
                           "FAST-LIO2 reject reason must remain OK"));
  gates.push_back(MaxGate("runtime.dropped_lidar_frames", GateSeverity::kWarn,
                          runtime.dropped_lidar_frames, 0.0,
                          "Dropped LiDAR frames indicate processing lag"));
  gates.push_back(MaxGate("runtime.dropped_imu_messages", GateSeverity::kFail,
                          runtime.dropped_imu_messages, 0.0,
                          "Dropped IMU samples can break deskew"));
  gates.push_back(MinGate("runtime.effective_feature_count", GateSeverity::kFail,
                          runtime.effective_feature_count,
                          thresholds.min_effective_feature_count,
                          "Feature count must support stable scan matching"));
  gates.push_back(MinGate("runtime.map_point_count", GateSeverity::kFail,
                          runtime.map_point_count,
                          thresholds.min_map_point_count,
                          "Map must accumulate enough points"));
  gates.push_back(MaxGate("runtime.mean_point_to_plane_residual", GateSeverity::kFail,
                          runtime.mean_point_to_plane_residual,
                          thresholds.max_mean_residual,
                          "Mean point-to-plane residual is too high"));
  gates.push_back(MaxGate("runtime.latency_p95_ms", GateSeverity::kWarn,
                          runtime.latency_p95_ms,
                          thresholds.max_latency_p95_ms,
                          "Processing latency exceeds real-time budget"));

  gates.push_back(MinGate("trajectory.matched_pose_count", GateSeverity::kFail,
                          trajectory.matched_pose_count, 20.0,
                          "Need enough matched poses for trajectory scoring"));
  gates.push_back(MaxGate("trajectory.ate_rmse_m", GateSeverity::kFail,
                          trajectory.ate_rmse_m, thresholds.max_ate_rmse_m,
                          "Aligned trajectory RMSE regressed"));
  gates.push_back(MaxGate("trajectory.ate_max_m", GateSeverity::kWarn,
                          trajectory.ate_max_m, thresholds.max_ate_max_m,
                          "Aligned trajectory max error is high"));
  gates.push_back(MinGate("trajectory.path_length_ratio.min",
                          GateSeverity::kFail, trajectory.path_length_ratio,
                          thresholds.min_path_length_ratio,
                          "Estimated path is too short versus reference"));
  gates.push_back(MaxGate("trajectory.path_length_ratio.max",
                          GateSeverity::kFail, trajectory.path_length_ratio,
                          thresholds.max_path_length_ratio,
                          "Estimated path is too long versus reference"));

  gates.push_back(BoolGate("map.optimized_frame_dataset_valid", GateSeverity::kFail,
                           map.optimized_frame_dataset_valid,
                           "optimized_frame_dataset must satisfy renderer schema"));
  gates.push_back(BoolGate("map.tile_bundle_valid", GateSeverity::kWarn,
                           map.tile_bundle_valid,
                           "tile_bundle should be generated from the dataset"));
  gates.push_back(BoolGate("map.localization_map_package_valid", GateSeverity::kWarn,
                           map.localization_map_package_valid,
                           "localization map package should be generated"));
  gates.push_back(BoolGate("map.map_products_consistent", GateSeverity::kFail,
                           map.map_products_consistent,
                           "tile and localization maps must share source dataset, map frame, and map origin"));
  gates.push_back(MinGate("map.tile_coverage_ratio", GateSeverity::kWarn,
                          map.tile_coverage_ratio,
                          thresholds.min_tile_coverage_ratio,
                          "Tile coverage should preserve useful map extent"));

  return scorecard;
}

GateResult ScorecardEvaluator::MinGate(const std::string& id,
                                       GateSeverity severity, double observed,
                                       double threshold,
                                       const std::string& message) {
  GateResult result;
  result.id = id;
  result.severity = severity;
  result.observed = observed;
  result.threshold = threshold;
  result.message = message;
  result.status = PassFail(IsFinite(observed) && observed >= threshold, severity);
  return result;
}

GateResult ScorecardEvaluator::MaxGate(const std::string& id,
                                       GateSeverity severity, double observed,
                                       double threshold,
                                       const std::string& message) {
  GateResult result;
  result.id = id;
  result.severity = severity;
  result.observed = observed;
  result.threshold = threshold;
  result.message = message;
  result.status = PassFail(IsFinite(observed) && observed <= threshold, severity);
  return result;
}

GateResult ScorecardEvaluator::BoolGate(const std::string& id,
                                        GateSeverity severity, bool observed,
                                        const std::string& message) {
  GateResult result;
  result.id = id;
  result.severity = severity;
  result.observed = observed ? 1.0 : 0.0;
  result.threshold = 1.0;
  result.message = message;
  result.status = PassFail(observed, severity);
  return result;
}

std::string ToString(GateSeverity severity) {
  switch (severity) {
    case GateSeverity::kInfo:
      return "INFO";
    case GateSeverity::kWarn:
      return "WARN";
    case GateSeverity::kFail:
      return "FAIL";
  }
  return "UNKNOWN";
}

std::string ToString(GateStatus status) {
  switch (status) {
    case GateStatus::kPass:
      return "PASS";
    case GateStatus::kWarn:
      return "WARN";
    case GateStatus::kFail:
      return "FAIL";
    case GateStatus::kNotEvaluated:
      return "NOT_EVALUATED";
  }
  return "UNKNOWN";
}

}  // namespace evaluation
}  // namespace fast_lio2
}  // namespace whl
