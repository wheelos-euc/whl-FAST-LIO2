#pragma once

#include <string>

#include "modules/fast_lio2/adapter/runtime_data.h"

namespace apollo {
namespace localization {
namespace fast_lio2 {

// Canonical FAST-LIO input: sensor-time LiDAR points in the LiDAR frame and
// raw angular velocity/acceleration in the IMU body frame.
bool ValidateFastLioImuSample(const FastLio2ImuSample& sample,
                              std::string* error);
bool ValidateFastLioPointCloudFrame(const FastLio2PointCloudFrame& frame,
                                    std::string* error);

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
