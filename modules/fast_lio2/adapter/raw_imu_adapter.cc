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

#include "modules/fast_lio2/adapter/raw_imu_adapter.h"

#include "modules/fast_lio2/adapter/sensor_contract.h"

namespace apollo {
namespace localization {
namespace fast_lio2 {

namespace {

double GpsToUnixSeconds(double gps_seconds) {
  constexpr double kGpsUnixEpochOffsetSec = 315964800.0;
  constexpr double kLeapSecondsSince2017 = 18.0;
  return gps_seconds + kGpsUnixEpochOffsetSec - kLeapSecondsSince2017;
}

}  // namespace

bool ConvertRawGnssImu(const apollo::drivers::gnss::Imu& in,
                       double receive_time_sec, bool use_receive_time,
                       FastLio2ImuSample* out, std::string* error) {
  if (out == nullptr) {
    if (error != nullptr) *error = "output imu pointer is null";
    return false;
  }
  if (!in.has_angular_velocity() || !in.has_linear_acceleration()) {
    if (error != nullptr) *error = "raw GNSS IMU is missing six-axis fields";
    return false;
  }

  if (use_receive_time) {
    out->timestamp_sec = receive_time_sec;
  } else if (in.measurement_time() > 0.0) {
    out->timestamp_sec = GpsToUnixSeconds(in.measurement_time());
  } else if (in.has_header() && in.header().timestamp_sec() > 0.0) {
    out->timestamp_sec = in.header().timestamp_sec();
  }

  out->angular_velocity =
      Eigen::Vector3d(in.angular_velocity().x(), in.angular_velocity().y(),
                      in.angular_velocity().z());
  out->linear_acceleration = Eigen::Vector3d(in.linear_acceleration().x(),
                                             in.linear_acceleration().y(),
                                             in.linear_acceleration().z());
  return ValidateFastLioImuSample(*out, error);
}

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
