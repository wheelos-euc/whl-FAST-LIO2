#pragma once

#include <deque>
#include <optional>
#include <string>

#include "fast_lio2/common/types.h"

namespace whl {
namespace fast_lio2 {

struct FastLio2Options {
  RuntimeMode runtime_mode = RuntimeMode::kOfflineMapping;
  int max_iterations = 5;
  int point_filter_num = 3;
  double blind_distance_m = 2.0;
  double surf_voxel_size_m = 0.3;
  double map_voxel_size_m = 0.5;
  double max_processing_latency_ms = 100.0;
  bool publish_registered_cloud = true;
  bool publish_map = true;
};

struct ProcessResult {
  bool has_pose = false;
  bool has_registered_cloud = false;
  bool has_map_update = false;
  Pose pose;
  OptimizedPointCloudFrame registered_frame;
  ProcessingStats stats;
};

// Framework-independent FAST-LIO2 runtime boundary.
//
// This class is intentionally free of ROS, Apollo Cyber, protobuf, PCL message,
// and filesystem dependencies. Framework adapters own message conversion,
// config loading, record playback, and publishing.
class FastLio2Core {
 public:
  FastLio2Core() = default;
  explicit FastLio2Core(FastLio2Options options);

  bool Init(const FastLio2Options& options, std::string* error);
  void Reset();

  void SetExtrinsic(const ExtrinsicLidarToImu& extrinsic);
  bool HasExtrinsic() const { return extrinsic_.has_value(); }

  void AddImu(const ImuSample& imu);
  void AddPointCloud(PointCloudFrame frame);

  ProcessResult ProcessNext();

  const ProcessingStats& stats() const { return stats_; }

 private:
  bool HasSynchronizedMeasurement() const;
  bool ValidatePointCloud(const PointCloudFrame& frame, std::string* error) const;
  bool ValidateImu(const ImuSample& imu, std::string* error) const;

  FastLio2Options options_;
  bool initialized_ = false;
  std::optional<ExtrinsicLidarToImu> extrinsic_;
  std::deque<ImuSample> imu_buffer_;
  std::deque<PointCloudFrame> lidar_buffer_;
  ProcessingStats stats_;
};

}  // namespace fast_lio2
}  // namespace whl
