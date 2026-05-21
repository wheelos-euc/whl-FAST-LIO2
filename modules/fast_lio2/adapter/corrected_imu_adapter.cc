/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/fast_lio2/adapter/corrected_imu_adapter.h"

#include <cmath>

namespace apollo {
namespace localization {
namespace fast_lio2 {

bool ConvertCorrectedImu(const apollo::localization::CorrectedImu& in,
                         FastLio2ImuSample* out, std::string* error) {
  if (out == nullptr) {
    if (error != nullptr) *error = "output imu pointer is null";
    return false;
  }
  if (!in.has_header() || in.header().timestamp_sec() <= 0.0) {
    if (error != nullptr) *error = "CorrectedImu header timestamp is invalid";
    return false;
  }
  if (!in.has_imu()) {
    if (error != nullptr) *error = "CorrectedImu has no imu payload";
    return false;
  }

  const auto& imu = in.imu();
  out->timestamp_sec = in.header().timestamp_sec();
  out->angular_velocity =
      Eigen::Vector3d(imu.angular_velocity().x(), imu.angular_velocity().y(),
                      imu.angular_velocity().z());
  out->linear_acceleration = Eigen::Vector3d(imu.linear_acceleration().x(),
                                             imu.linear_acceleration().y(),
                                             imu.linear_acceleration().z());

  if (!std::isfinite(out->angular_velocity.x()) ||
      !std::isfinite(out->angular_velocity.y()) ||
      !std::isfinite(out->angular_velocity.z()) ||
      !std::isfinite(out->linear_acceleration.x()) ||
      !std::isfinite(out->linear_acceleration.y()) ||
      !std::isfinite(out->linear_acceleration.z())) {
    if (error != nullptr) *error = "CorrectedImu contains non-finite values";
    return false;
  }
  return true;
}

sensor_msgs::Imu::Ptr ToCompatImu(const FastLio2ImuSample& in) {
  auto out = std::make_shared<sensor_msgs::Imu>();
  out->header.stamp =
      apollo::localization::fast_lio2::compat::Time::FromSec(in.timestamp_sec);
  out->angular_velocity.x = in.angular_velocity.x();
  out->angular_velocity.y = in.angular_velocity.y();
  out->angular_velocity.z = in.angular_velocity.z();
  out->linear_acceleration.x = in.linear_acceleration.x();
  out->linear_acceleration.y = in.linear_acceleration.y();
  out->linear_acceleration.z = in.linear_acceleration.z();
  return out;
}

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
