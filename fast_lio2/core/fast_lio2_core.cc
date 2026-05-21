#include "fast_lio2/core/fast_lio2_core.h"

#include <cmath>
#include <utility>

namespace whl {
namespace fast_lio2 {

namespace {

bool IsFinite(double value) { return std::isfinite(value); }

}  // namespace

FastLio2Core::FastLio2Core(FastLio2Options options)
    : options_(std::move(options)) {}

bool FastLio2Core::Init(const FastLio2Options& options, std::string* error) {
  if (options.point_filter_num <= 0) {
    if (error != nullptr) {
      *error = "point_filter_num must be positive";
    }
    return false;
  }
  if (options.blind_distance_m < 0.0 || options.surf_voxel_size_m <= 0.0 ||
      options.map_voxel_size_m <= 0.0) {
    if (error != nullptr) {
      *error = "invalid FAST-LIO2 geometric filter options";
    }
    return false;
  }
  options_ = options;
  initialized_ = true;
  stats_.tracking_state = TrackingState::kInitializing;
  stats_.reject_reason.clear();
  return true;
}

void FastLio2Core::Reset() {
  imu_buffer_.clear();
  lidar_buffer_.clear();
  stats_ = ProcessingStats{};
  stats_.tracking_state =
      initialized_ ? TrackingState::kInitializing : TrackingState::kNotInitialized;
}

void FastLio2Core::SetExtrinsic(const ExtrinsicLidarToImu& extrinsic) {
  extrinsic_ = extrinsic;
}

void FastLio2Core::AddImu(const ImuSample& imu) {
  ++stats_.imu_samples_received;
  std::string error;
  if (!ValidateImu(imu, &error)) {
    ++stats_.dropped_imu_samples;
    stats_.reject_reason = error;
    return;
  }
  imu_buffer_.push_back(imu);
}

void FastLio2Core::AddPointCloud(PointCloudFrame frame) {
  ++stats_.lidar_frames_received;
  std::string error;
  if (!ValidatePointCloud(frame, &error)) {
    ++stats_.dropped_lidar_frames;
    stats_.reject_reason = error;
    return;
  }
  lidar_buffer_.push_back(std::move(frame));
}

bool FastLio2Core::HasSynchronizedMeasurement() const {
  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }
  return imu_buffer_.back().timestamp_sec >= lidar_buffer_.front().scan_end_sec;
}

ProcessResult FastLio2Core::ProcessNext() {
  ProcessResult result;
  result.stats = stats_;
  if (!initialized_) {
    result.stats.reject_reason = "core is not initialized";
    return result;
  }
  if (!extrinsic_.has_value()) {
    result.stats.reject_reason = "lidar-to-imu extrinsic is missing";
    return result;
  }
  if (lidar_buffer_.empty()) {
    result.stats.reject_reason = "waiting for lidar frame";
    return result;
  }
  if (imu_buffer_.empty()) {
    result.stats.reject_reason = "waiting for imu samples";
    return result;
  }
  if (!HasSynchronizedMeasurement()) {
    result.stats.reject_reason = "waiting for imu coverage of lidar scan";
    return result;
  }

  PointCloudFrame frame = std::move(lidar_buffer_.front());
  lidar_buffer_.pop_front();
  std::size_t synced_imu_count = 0;
  while (!imu_buffer_.empty() &&
         imu_buffer_.front().timestamp_sec <= frame.scan_end_sec) {
    imu_buffer_.pop_front();
    ++synced_imu_count;
  }
  if (synced_imu_count == 0) {
    ++stats_.dropped_lidar_frames;
    result.stats = stats_;
    result.stats.reject_reason = "no imu samples aligned to lidar scan";
    result.stats.tracking_state = TrackingState::kDegraded;
    return result;
  }

  ++stats_.frames_processed;
  result.stats = stats_;
  result.stats.reject_reason =
      "synchronized package consumed; scan-to-map backend is delegated by runtime adapter";
  result.stats.tracking_state = TrackingState::kInitializing;
  return result;
}

bool FastLio2Core::ValidatePointCloud(const PointCloudFrame& frame,
                                      std::string* error) const {
  if (!IsFinite(frame.scan_start_sec) || !IsFinite(frame.scan_end_sec) ||
      frame.scan_end_sec <= frame.scan_start_sec) {
    if (error != nullptr) {
      *error = "invalid lidar scan time range";
    }
    return false;
  }
  if (frame.points.size() <= 5) {
    if (error != nullptr) {
      *error = "too few lidar points";
    }
    return false;
  }
  double min_time = frame.points.front().relative_time_sec;
  double max_time = frame.points.front().relative_time_sec;
  for (const auto& point : frame.points) {
    if (!IsFinite(point.x) || !IsFinite(point.y) || !IsFinite(point.z) ||
        !IsFinite(point.relative_time_sec)) {
      if (error != nullptr) {
        *error = "lidar point contains non-finite value";
      }
      return false;
    }
    min_time = std::min(min_time, point.relative_time_sec);
    max_time = std::max(max_time, point.relative_time_sec);
  }
  if (max_time <= min_time) {
    if (error != nullptr) {
      *error = "lidar per-point timestamps are missing or constant";
    }
    return false;
  }
  return true;
}

bool FastLio2Core::ValidateImu(const ImuSample& imu, std::string* error) const {
  const bool valid =
      IsFinite(imu.timestamp_sec) && imu.timestamp_sec > 0.0 &&
      IsFinite(imu.angular_velocity_radps.x) &&
      IsFinite(imu.angular_velocity_radps.y) &&
      IsFinite(imu.angular_velocity_radps.z) &&
      IsFinite(imu.linear_acceleration_mps2.x) &&
      IsFinite(imu.linear_acceleration_mps2.y) &&
      IsFinite(imu.linear_acceleration_mps2.z);
  if (!valid && error != nullptr) {
    *error = "invalid imu timestamp or non-finite imu value";
  }
  return valid;
}

}  // namespace fast_lio2
}  // namespace whl
