#pragma once

#include <string>
#include <vector>

namespace whl {
namespace fast_lio2 {
namespace evaluation {

enum class GateSeverity {
  kInfo = 0,
  kWarn = 1,
  kFail = 2,
};

enum class GateStatus {
  kPass = 0,
  kWarn = 1,
  kFail = 2,
  kNotEvaluated = 3,
};

struct InputDataMetrics {
  int lidar_frame_count = 0;
  double lidar_rate_hz = 0.0;
  double imu_rate_hz = 0.0;
  double point_time_span_sec = 0.0;
  double imu_acc_norm_mean_mps2 = 0.0;
  bool has_lidar_to_imu_extrinsic = false;
  bool timestamps_monotonic = false;
  double sensor_time_overlap_ratio = 0.0;
};

struct RuntimeMetrics {
  bool tracking_ok = false;
  bool reject_ok = false;
  int lidar_frames_received = 0;
  int frames_processed = 0;
  int dropped_lidar_frames = 0;
  int dropped_imu_messages = 0;
  int effective_feature_count = 0;
  int map_point_count = 0;
  double mean_point_to_plane_residual = 0.0;
  double latency_p50_ms = 0.0;
  double latency_p95_ms = 0.0;
};

struct TrajectoryMetrics {
  int matched_pose_count = 0;
  double ate_rmse_m = 0.0;
  double ate_mean_m = 0.0;
  double ate_max_m = 0.0;
  double path_length_ratio = 0.0;
  double yaw_drift_deg_per_100m = 0.0;
  double height_drift_m = 0.0;
};

struct MapQualityMetrics {
  double repeat_pass_overlap_rmse_m = 0.0;
  double wall_thickness_p90_m = 0.0;
  double dynamic_object_ratio = 0.0;
  double tile_coverage_ratio = 0.0;
  bool optimized_frame_dataset_valid = false;
  bool tile_bundle_valid = false;
  bool localization_map_package_valid = false;
  bool map_products_consistent = false;
};

struct BaselineThresholds {
  int min_lidar_frame_count = 30;
  double min_lidar_rate_hz = 5.0;
  double min_imu_rate_hz = 95.0;
  double min_point_time_span_sec = 0.01;
  double min_sensor_time_overlap_ratio = 0.95;

  double max_ate_rmse_m = 0.50;
  double max_ate_max_m = 1.50;
  double min_path_length_ratio = 0.85;
  double max_path_length_ratio = 1.15;

  int min_effective_feature_count = 500;
  int min_map_point_count = 4000;
  double max_mean_residual = 0.08;
  double max_latency_p95_ms = 200.0;

  double min_tile_coverage_ratio = 0.90;
};

struct GateResult {
  std::string id;
  GateSeverity severity = GateSeverity::kFail;
  GateStatus status = GateStatus::kNotEvaluated;
  double observed = 0.0;
  double threshold = 0.0;
  std::string message;
};

struct Scorecard {
  std::string dataset_id;
  InputDataMetrics input;
  RuntimeMetrics runtime;
  TrajectoryMetrics trajectory;
  MapQualityMetrics map;
  BaselineThresholds thresholds;
  std::vector<GateResult> gates;

  bool Passed() const;
  bool HasFailures() const;
};

class ScorecardEvaluator {
 public:
  Scorecard Evaluate(const std::string& dataset_id,
                     const InputDataMetrics& input,
                     const RuntimeMetrics& runtime,
                     const TrajectoryMetrics& trajectory,
                     const MapQualityMetrics& map,
                     const BaselineThresholds& thresholds) const;

 private:
  static GateResult MinGate(const std::string& id, GateSeverity severity,
                            double observed, double threshold,
                            const std::string& message);
  static GateResult MaxGate(const std::string& id, GateSeverity severity,
                            double observed, double threshold,
                            const std::string& message);
  static GateResult BoolGate(const std::string& id, GateSeverity severity,
                             bool observed, const std::string& message);
};

std::string ToString(GateSeverity severity);
std::string ToString(GateStatus status);

}  // namespace evaluation
}  // namespace fast_lio2
}  // namespace whl
