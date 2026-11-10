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

#include "modules/fast_lio2/adapter/pointcloud_view_adapter.h"

#include <cmath>
#include <limits>
#include <vector>

#include "modules/fast_lio2/adapter/sensor_contract.h"

namespace apollo {
namespace localization {
namespace fast_lio2 {

namespace {

bool InRoi(const FastLio2RuntimeConf& conf, const PointType& pt) {
  if (!conf.roi_enable()) return true;
  return pt.x >= conf.roi_x_min() && pt.x <= conf.roi_x_max() &&
         pt.y >= conf.roi_y_min() && pt.y <= conf.roi_y_max() &&
         pt.z >= conf.roi_z_min() && pt.z <= conf.roi_z_max();
}

}  // namespace

bool ConvertPointCloud(const apollo::drivers::PointCloud& in,
                       double receive_time_sec, const FastLio2RuntimeConf& conf,
                       FastLio2PointCloudFrame* out, std::string* error) {
  if (out == nullptr) {
    if (error != nullptr) *error = "output cloud pointer is null";
    return false;
  }
  if (in.point_size() == 0) {
    if (error != nullptr) *error = "PointCloud is empty";
    return false;
  }

  const bool use_receive_time =
      conf.timestamp_mode() == FastLio2RuntimeConf::RECEIVE_TIME;
  double first_point_time_sec = 0.0;
  if (use_receive_time && receive_time_sec <= 0.0) {
    if (error != nullptr) {
      *error = "receive-time mode requires receive time";
    }
    return false;
  }

  auto cloud = std::make_shared<PointCloudXYZI>();
  cloud->reserve(in.point_size());
  std::vector<double> point_times_sec;
  point_times_sec.reserve(in.point_size());

  const int point_filter_num = std::max(1, conf.point_filter_num());
  const double blind2 = conf.blind() * conf.blind();
  double min_rel_ms = std::numeric_limits<double>::infinity();
  double max_rel_ms = -std::numeric_limits<double>::infinity();
  int accepted_seen = 0;
  int valid_time_count = 0;
  uint64_t previous_timestamp = 0;

  for (const auto& src : in.point()) {
    if (!std::isfinite(src.x()) || !std::isfinite(src.y()) ||
        !std::isfinite(src.z())) {
      continue;
    }
    PointType pt;
    pt.x = src.x();
    pt.y = src.y();
    pt.z = src.z();
    pt.intensity = static_cast<float>(src.intensity());
    pt.normal_x = 0.0F;
    pt.normal_y = 0.0F;
    pt.normal_z = 0.0F;

    if (src.timestamp() > 0) {
      if (previous_timestamp > 0 && src.timestamp() < previous_timestamp) {
        if (error != nullptr) {
          *error = "LiDAR point timestamps are not monotonic";
        }
        return false;
      }
      previous_timestamp = src.timestamp();
      const double point_time_sec =
          static_cast<double>(src.timestamp()) * 1e-9;
      if (first_point_time_sec <= 0.0) {
        first_point_time_sec = point_time_sec;
      }
    }

    const double range2 = static_cast<double>(pt.x) * pt.x +
                          static_cast<double>(pt.y) * pt.y +
                          static_cast<double>(pt.z) * pt.z;
    if (range2 < blind2 || !InRoi(conf, pt)) continue;
    if ((accepted_seen++ % point_filter_num) != 0) continue;

    const double point_time_sec =
        src.timestamp() > 0 ? static_cast<double>(src.timestamp()) * 1e-9
                            : 0.0;
    pt.curvature = 0.0F;
    point_times_sec.push_back(point_time_sec);
    cloud->push_back(pt);
  }

  if (cloud->size() <= 5) {
    if (error != nullptr) *error = "too few valid points after filtering";
    return false;
  }
  // Apollo LiDAR drivers publish measurement_time as the last point time, while
  // FAST-LIO2 expects curvature to be relative to scan begin.
  double scan_start_sec = first_point_time_sec;
  min_rel_ms = std::numeric_limits<double>::infinity();
  max_rel_ms = -std::numeric_limits<double>::infinity();
  valid_time_count = 0;
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (point_times_sec[i] <= 0.0) {
      cloud->points[i].curvature = 0.0F;
      continue;
    }
    const double rel_ms = (point_times_sec[i] - first_point_time_sec) * 1000.0;
    cloud->points[i].curvature = static_cast<float>(rel_ms);
    min_rel_ms = std::min(min_rel_ms, rel_ms);
    max_rel_ms = std::max(max_rel_ms, rel_ms);
    ++valid_time_count;
  }
  if (valid_time_count == 0 || !std::isfinite(min_rel_ms) ||
      !std::isfinite(max_rel_ms) || max_rel_ms <= min_rel_ms) {
    if (error != nullptr) {
      *error = "point cloud has invalid or constant per-point timestamps";
    }
    return false;
  }

  out->points = cloud;
  out->scan_start_sec = use_receive_time
                            ? (receive_time_sec - max_rel_ms * 1e-3)
                            : scan_start_sec;
  out->scan_end_sec = use_receive_time ? receive_time_sec
                                       : (scan_start_sec + max_rel_ms * 1e-3);
  out->min_relative_time_ms = min_rel_ms;
  out->max_relative_time_ms = max_rel_ms;
  return ValidateFastLioPointCloudFrame(*out, error);
}

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
