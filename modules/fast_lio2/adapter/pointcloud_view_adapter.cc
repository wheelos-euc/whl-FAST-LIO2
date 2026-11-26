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

#include <flatbuffers/flatbuffers.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "modules/fast_lio2/adapter/sensor_contract.h"
#include "modules/fast_lio2/proto/pointcloud_wire_generated.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

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

struct PointFields {
  double x;
  double y;
  double z;
  double intensity;
  uint64_t timestamp;
};

template <typename GetPoint>
bool ConvertPoints(size_t point_count, GetPoint get_point,
                   double receive_time_sec, const FastLio2RuntimeConf& conf,
                   FastLio2PointCloudFrame* out, std::string* error) {
  if (out == nullptr) {
    if (error != nullptr) *error = "output cloud pointer is null";
    return false;
  }
  if (point_count == 0) {
    if (error != nullptr) *error = "PointCloud is empty";
    return false;
  }

  const bool use_receive_time =
      conf.timestamp_mode() == FastLio2RuntimeConf::RECEIVE_TIME;
  if (use_receive_time && receive_time_sec <= 0.0) {
    if (error != nullptr) {
      *error = "receive-time mode requires receive time";
    }
    return false;
  }

  auto cloud = std::make_shared<PointCloudXYZI>();
  cloud->reserve(point_count);
  std::vector<uint64_t> point_timestamps_ns;
  point_timestamps_ns.reserve(point_count);

  const int point_filter_num = std::max(1, conf.point_filter_num());
  const double blind2 = conf.blind() * conf.blind();
  uint64_t first_point_timestamp_ns = 0;
  uint64_t previous_timestamp = 0;
  int accepted_seen = 0;

  for (size_t i = 0; i < point_count; ++i) {
    const PointFields src = get_point(i);
    if (!std::isfinite(src.x) || !std::isfinite(src.y) ||
        !std::isfinite(src.z)) {
      continue;
    }

    if (src.timestamp > 0) {
      if (previous_timestamp > 0 && src.timestamp < previous_timestamp) {
        if (error != nullptr) {
          *error = "LiDAR point timestamps are not monotonic";
        }
        return false;
      }
      previous_timestamp = src.timestamp;
      if (first_point_timestamp_ns == 0) {
        first_point_timestamp_ns = src.timestamp;
      }
    }

    PointType pt;
    pt.x = static_cast<float>(src.x);
    pt.y = static_cast<float>(src.y);
    pt.z = static_cast<float>(src.z);
    pt.intensity = static_cast<float>(src.intensity);
    pt.normal_x = 0.0F;
    pt.normal_y = 0.0F;
    pt.normal_z = 0.0F;

    const double range2 = static_cast<double>(pt.x) * pt.x +
                          static_cast<double>(pt.y) * pt.y +
                          static_cast<double>(pt.z) * pt.z;
    if (range2 < blind2 || !InRoi(conf, pt)) continue;
    if ((accepted_seen++ % point_filter_num) != 0) continue;

    pt.curvature = 0.0F;
    point_timestamps_ns.push_back(src.timestamp);
    cloud->push_back(pt);
  }

  if (cloud->size() <= 5) {
    if (error != nullptr) *error = "too few valid points after filtering";
    return false;
  }

  // Apollo LiDAR drivers publish measurement_time as the last point time, while
  // FAST-LIO2 expects curvature to be relative to scan begin.
  double min_rel_ms = std::numeric_limits<double>::infinity();
  double max_rel_ms = -std::numeric_limits<double>::infinity();
  int valid_time_count = 0;
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (point_timestamps_ns[i] == 0) {
      cloud->points[i].curvature = 0.0F;
      continue;
    }
    const double rel_ms =
        static_cast<double>(point_timestamps_ns[i] - first_point_timestamp_ns) *
        1e-6;
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
  out->scan_start_sec =
      use_receive_time ? (receive_time_sec - max_rel_ms * 1e-3)
                       : static_cast<double>(first_point_timestamp_ns) * 1e-9;
  out->scan_end_sec =
      use_receive_time ? receive_time_sec
                       : (static_cast<double>(first_point_timestamp_ns) * 1e-9 +
                          max_rel_ms * 1e-3);
  out->min_relative_time_ms = min_rel_ms;
  out->max_relative_time_ms = max_rel_ms;
  return ValidateFastLioPointCloudFrame(*out, error);
}

}  // namespace

bool ConvertPointCloud(const std::string& serialized, double receive_time_sec,
                       const FastLio2RuntimeConf& conf,
                       FastLio2PointCloudFrame* out, std::string* error) {
  const auto* data = reinterpret_cast<const uint8_t*>(serialized.data());
  if (serialized.size() >= 8 &&
      flatbuffers::BufferHasIdentifier(data, "PCLD")) {
    flatbuffers::Verifier verifier(data, serialized.size());
    if (!wire::flat::VerifyPointCloudBuffer(verifier)) {
      if (error != nullptr) *error = "FlatBuffer PointCloud is malformed";
      return false;
    }
    const auto* cloud = wire::flat::GetPointCloud(data);
    const auto* points = cloud->point();
    return ConvertPoints(
        points == nullptr ? 0 : points->size(),
        [points](size_t i) {
          const auto* point = points->Get(i);
          return PointFields{point->x(), point->y(), point->z(),
                             static_cast<double>(point->intensity()),
                             point->timestamp()};
        },
        receive_time_sec, conf, out, error);
  }

  apollo::drivers::PointCloud cloud;
  if (!cloud.ParseFromString(serialized)) {
    if (error != nullptr) {
      *error = "PointCloud is neither a valid protobuf nor a valid PCLD buffer";
    }
    return false;
  }
  return ConvertPoints(
      static_cast<size_t>(cloud.point_size()),
      [&cloud](size_t i) {
        const auto& point = cloud.point(static_cast<int>(i));
        return PointFields{point.x(), point.y(), point.z(),
                           static_cast<double>(point.intensity()),
                           point.timestamp()};
      },
      receive_time_sec, conf, out, error);
}

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
