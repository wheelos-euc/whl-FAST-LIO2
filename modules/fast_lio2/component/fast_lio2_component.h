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

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "cyber/adaptive/adaptive_pointcloud_reader.h"
#include "cyber/component/component.h"
#include "modules/fast_lio2/adapter/runtime_data.h"
#include "modules/fast_lio2/core/fast_lio2_runtime_core.h"
#include "modules/fast_lio2/proto/fast_lio2_runtime_conf.pb.h"
#include "modules/common_msgs/localization_msgs/imu.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/sensor_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/fast_lio2/proto/fast_lio2_conf.pb.h"
#include "modules/fast_lio2/proto/fast_lio2_metrics.pb.h"
#include "modules/slam_localization/proto/odometry.pb.h"

namespace apollo {
namespace localization {
namespace fast_lio2 {

class FastLio2Component final : public apollo::cyber::Component<> {
 public:
  bool Init() override;
  ~FastLio2Component() override { Clear(); }

 private:
  void Clear();
  void PointCloudCallback(const apollo::cyber::adaptive::PointCloudView& cloud,
                          double receive_time_sec);
  void ImuCallback(
      const std::shared_ptr<apollo::localization::CorrectedImu>& imu,
      double receive_time_sec);
  void RawImuCallback(
      const std::shared_ptr<apollo::drivers::gnss::Imu>& imu,
      double receive_time_sec);
  void WorkerLoop();
  void PublishOutputs(double processing_latency_ms);
  void PublishLocalization(
      const FastLio2RuntimeStateSnapshot& state);
  void PublishOdometry(
      const FastLio2RuntimeStateSnapshot& state);
  void PublishMetrics(
      const FastLio2RuntimeStateSnapshot* state,
      double processing_latency_ms);
  bool BuildCoreConf();

  FastLio2Conf conf_;
  FastLio2RuntimeConf core_conf_;
  FastLio2RuntimeCore core_;

  std::unique_ptr<apollo::cyber::adaptive::AdaptivePointCloudReader>
      pointcloud_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::localization::CorrectedImu>>
      imu_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::drivers::gnss::Imu>>
      raw_imu_reader_;
  std::shared_ptr<apollo::cyber::Writer<apollo::localization::LocalizationEstimate>>
      localization_writer_;
  std::shared_ptr<apollo::cyber::Writer<apollo::localization::Odometry>>
      odometry_writer_;
  std::shared_ptr<apollo::cyber::Writer<FastLio2Metrics>> metrics_writer_;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::PointCloud>>
      cloud_registered_writer_;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::PointCloud>>
      map_writer_;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<apollo::localization::fast_lio2::FastLio2PointCloudFrame> pending_clouds_;
  std::deque<apollo::localization::fast_lio2::FastLio2ImuSample> pending_imus_;
  std::thread worker_thread_;
  std::atomic<bool> worker_running_{false};

  uint64_t lidar_frames_received_ = 0;
  uint64_t imu_messages_received_ = 0;
  uint64_t dropped_pointcloud_frames_ = 0;
  uint64_t dropped_imu_messages_ = 0;
  uint64_t localization_seq_ = 0;
  uint64_t odometry_seq_ = 0;
  uint64_t metrics_seq_ = 0;
  double last_map_publish_time_sec_ = 0.0;
};

CYBER_REGISTER_COMPONENT(FastLio2Component)

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
