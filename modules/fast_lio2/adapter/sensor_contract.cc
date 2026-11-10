#include "modules/fast_lio2/adapter/sensor_contract.h"

#include <cmath>

namespace apollo {
namespace localization {
namespace fast_lio2 {

bool ValidateFastLioImuSample(const FastLio2ImuSample& sample,
                              std::string* error) {
  if (!std::isfinite(sample.timestamp_sec) || sample.timestamp_sec <= 0.0) {
    if (error != nullptr) *error = "IMU sensor timestamp is invalid";
    return false;
  }
  if (!sample.angular_velocity.allFinite() ||
      !sample.linear_acceleration.allFinite()) {
    if (error != nullptr) *error = "IMU body-frame measurement is non-finite";
    return false;
  }
  return true;
}

bool ValidateFastLioPointCloudFrame(const FastLio2PointCloudFrame& frame,
                                    std::string* error) {
  if (!frame.points || frame.points->size() <= 5) {
    if (error != nullptr) *error = "LiDAR scan has too few points";
    return false;
  }
  if (!std::isfinite(frame.scan_start_sec) ||
      !std::isfinite(frame.scan_end_sec) ||
      frame.scan_end_sec <= frame.scan_start_sec) {
    if (error != nullptr) *error = "LiDAR scan sensor-time bounds are invalid";
    return false;
  }
  if (!std::isfinite(frame.min_relative_time_ms) ||
      !std::isfinite(frame.max_relative_time_ms) ||
      frame.max_relative_time_ms <= frame.min_relative_time_ms) {
    if (error != nullptr) *error = "LiDAR point-time span is invalid";
    return false;
  }
  return true;
}

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
