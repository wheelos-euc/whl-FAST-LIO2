#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace whl {
namespace fast_lio2 {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Pose {
  double timestamp_sec = 0.0;
  Vec3 position;
  Quaternion orientation;
  Vec3 linear_velocity;
};

struct ImuSample {
  double timestamp_sec = 0.0;
  Vec3 angular_velocity_radps;
  Vec3 linear_acceleration_mps2;
};

struct PointXYZIT {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
  double relative_time_sec = 0.0;
  std::uint16_t ring = 0;
};

struct PointCloudFrame {
  double scan_start_sec = 0.0;
  double scan_end_sec = 0.0;
  std::string frame_id;
  std::vector<PointXYZIT> points;
};

// FAST-LIO2 core convention: p_imu = R_li * p_lidar + t_li.
struct ExtrinsicLidarToImu {
  Quaternion rotation_lidar_to_imu;
  Vec3 translation_lidar_to_imu;
};

enum class RuntimeMode {
  kOfflineMapping = 0,
  kRealtimeOdometry = 1,
  kMapLocalization = 2,
};

enum class TrackingState {
  kNotInitialized = 0,
  kInitializing = 1,
  kOk = 2,
  kDegraded = 3,
  kLost = 4,
};

struct ProcessingStats {
  std::uint64_t lidar_frames_received = 0;
  std::uint64_t imu_samples_received = 0;
  std::uint64_t frames_processed = 0;
  std::uint64_t dropped_lidar_frames = 0;
  std::uint64_t dropped_imu_samples = 0;
  double last_latency_ms = 0.0;
  double mean_point_to_plane_residual = 0.0;
  double scan_match_inlier_ratio = 0.0;
  int effective_feature_count = 0;
  int map_point_count = 0;
  TrackingState tracking_state = TrackingState::kNotInitialized;
  std::string reject_reason;
};

struct OptimizedPointCloudFrame {
  double timestamp_sec = 0.0;
  Pose local_pose;
  std::vector<PointXYZIT> frame_local_points;
};

struct MapPackageManifest {
  std::string schema_version = "1.0.0";
  std::string producer = "whl_fast_lio2";
  std::string coordinate_frame = "local";
  std::string source_record;
  std::string extrinsic_source;
  std::uint64_t frame_count = 0;
  std::uint64_t map_point_count = 0;
};

}  // namespace fast_lio2
}  // namespace whl

