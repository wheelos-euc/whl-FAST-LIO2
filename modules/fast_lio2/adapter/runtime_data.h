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

#pragma once

#include <memory>

#include <Eigen/Core>

#include "modules/fast_lio2/core/include/common_lib.h"

namespace apollo {
namespace localization {
namespace fast_lio2 {

struct FastLio2ImuSample {
  double timestamp_sec = 0.0;
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
};

struct FastLio2PointCloudFrame {
  PointCloudXYZI::Ptr points;
  double scan_start_sec = 0.0;
  double scan_end_sec = 0.0;
  double min_relative_time_ms = 0.0;
  double max_relative_time_ms = 0.0;
};

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
