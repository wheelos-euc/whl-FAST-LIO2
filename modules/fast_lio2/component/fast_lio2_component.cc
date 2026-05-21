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

#include "modules/fast_lio2/component/fast_lio2_component.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include <Eigen/Geometry>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/fast_lio2/adapter/corrected_imu_adapter.h"
#include "modules/fast_lio2/adapter/pointcloud_view_adapter.h"
#include "modules/common/util/time_conversion.h"

namespace apollo {
namespace localization {
namespace fast_lio2 {

namespace {

double HeadingFromQuaternion(const Eigen::Quaterniond& q) {
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

void FillPoint3D(const Eigen::Vector3d& src, apollo::common::Point3D* dst) {
  dst->set_x(src.x());
  dst->set_y(src.y());
  dst->set_z(src.z());
}

void FillPointENU(const Eigen::Vector3d& src, apollo::common::PointENU* dst) {
  dst->set_x(src.x());
  dst->set_y(src.y());
  dst->set_z(src.z());
}

void FillVector3(const Eigen::Vector3d& src, apollo::localization::Vector3* dst) {
  dst->set_x(src.x());
  dst->set_y(src.y());
  dst->set_z(src.z());
}

void FillQuaternion(const Eigen::Quaterniond& src,
                    apollo::common::Quaternion* dst) {
  dst->set_qx(src.x());
  dst->set_qy(src.y());
  dst->set_qz(src.z());
  dst->set_qw(src.w());
}

bool HasFullInitialExtrinsic(const FastLio2Conf& conf) {
  return conf.has_initial_rotation_lidar_to_imu_qx() &&
         conf.has_initial_rotation_lidar_to_imu_qy() &&
         conf.has_initial_rotation_lidar_to_imu_qz() &&
         conf.has_initial_rotation_lidar_to_imu_qw() &&
         conf.has_initial_translation_lidar_to_imu_x() &&
         conf.has_initial_translation_lidar_to_imu_y() &&
         conf.has_initial_translation_lidar_to_imu_z();
}

}  // namespace

bool FastLio2Component::Init() {
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_, &conf_)) {
    AERROR << "Failed to load FAST-LIO2 config: " << config_file_path_;
    return false;
  }
  AINFO << "FAST-LIO2 config: " << conf_.DebugString();

  if (!BuildCoreConf()) {
    return false;
  }
  if (!core_.Init(core_conf_)) {
    AERROR << "Failed to initialize FAST-LIO2 core.";
    return false;
  }

  localization_writer_ =
      node_->CreateWriter<apollo::localization::LocalizationEstimate>(
          conf_.localization_topic());
  odometry_writer_ =
      node_->CreateWriter<apollo::localization::Odometry>(conf_.odometry_topic());
  metrics_writer_ = node_->CreateWriter<FastLio2Metrics>(conf_.metrics_topic());
  if (conf_.publish_registered_cloud()) {
    cloud_registered_writer_ =
        node_->CreateWriter<apollo::drivers::PointCloud>(
            conf_.cloud_registered_topic());
  }
  if (conf_.publish_map()) {
    map_writer_ =
        node_->CreateWriter<apollo::drivers::PointCloud>(conf_.map_topic());
  }

  apollo::cyber::ReaderConfig imu_config;
  imu_config.channel_name = conf_.imu_topic();
  imu_config.pending_queue_size = 200000;
  if (conf_.imu_message_type() == FastLio2Conf::RAW_GNSS_IMU) {
    raw_imu_reader_ = node_->CreateReader<apollo::drivers::gnss::Imu>(
        imu_config,
        [this](const std::shared_ptr<apollo::drivers::gnss::Imu>& msg) {
          this->RawImuCallback(msg, apollo::cyber::Clock::NowInSeconds());
        });
  } else {
    imu_reader_ = node_->CreateReader<apollo::localization::CorrectedImu>(
        imu_config,
        [this](const std::shared_ptr<apollo::localization::CorrectedImu>& msg) {
          this->ImuCallback(msg, apollo::cyber::Clock::NowInSeconds());
        });
  }

  pointcloud_reader_.reset(
      new apollo::cyber::adaptive::AdaptivePointCloudReader);
  apollo::cyber::adaptive::AdaptivePointCloudReaderOptions options;
  options.channel = conf_.pointcloud_topic();
  options.pending_queue_size = 1;
  if (!pointcloud_reader_->Init(
          node_, options,
          [this](const apollo::cyber::adaptive::AdaptivePointCloudEnvelope&
                     envelope) {
            this->PointCloudCallback(envelope.view, envelope.receive_time_sec);
          })) {
    AERROR << "Failed to initialize FAST-LIO2 adaptive pointcloud reader.";
    return false;
  }

  worker_running_.store(true);
  worker_thread_ = std::thread(&FastLio2Component::WorkerLoop, this);

  if (!pointcloud_reader_->Start()) {
    AERROR << "Failed to start FAST-LIO2 pointcloud reader.";
    Clear();
    return false;
  }

  AINFO << "FAST-LIO2 component started. pointcloud_topic="
        << conf_.pointcloud_topic() << " imu_topic=" << conf_.imu_topic();
  return true;
}

bool FastLio2Component::BuildCoreConf() {
  core_conf_.Clear();
  core_conf_.set_pointcloud_topic(conf_.pointcloud_topic());
  core_conf_.set_imu_topic(conf_.imu_topic());
  core_conf_.set_point_filter_num(conf_.point_filter_num());
  core_conf_.set_blind(conf_.blind());
  core_conf_.set_filter_size_surf(conf_.filter_size_surf());
  core_conf_.set_filter_size_map(conf_.filter_size_map());
  core_conf_.set_gyr_cov(conf_.gyr_cov());
  core_conf_.set_acc_cov(conf_.acc_cov());
  core_conf_.set_b_gyr_cov(conf_.b_gyr_cov());
  core_conf_.set_b_acc_cov(conf_.b_acc_cov());
  core_conf_.set_det_range(conf_.det_range());
  core_conf_.set_max_iteration(conf_.max_iteration());
  core_conf_.set_cube_side_length(conf_.cube_side_length());
  core_conf_.set_mean_acc_norm(conf_.mean_acc_norm());
  core_conf_.set_roi_enable(conf_.roi_enable());
  core_conf_.set_roi_x_min(conf_.roi_x_min());
  core_conf_.set_roi_x_max(conf_.roi_x_max());
  core_conf_.set_roi_y_min(conf_.roi_y_min());
  core_conf_.set_roi_y_max(conf_.roi_y_max());
  core_conf_.set_roi_z_min(conf_.roi_z_min());
  core_conf_.set_roi_z_max(conf_.roi_z_max());
  core_conf_.set_max_pending_pointcloud_frames(
      conf_.max_pending_pointcloud_frames());
  core_conf_.set_max_pending_imu_messages(conf_.max_pending_imu_messages());
  core_conf_.set_online_refine_time(conf_.online_refine_time());
  core_conf_.set_data_accum_length(conf_.data_accum_length());
  core_conf_.set_cut_frame_num(conf_.cut_frame_num());
  core_conf_.set_orig_odom_freq(conf_.orig_odom_freq());
  core_conf_.set_publish_visualization(conf_.publish_registered_cloud() ||
                                       conf_.publish_map());
  core_conf_.set_cloud_registered_topic(conf_.cloud_registered_topic());
  core_conf_.set_map_topic(conf_.map_topic());
  core_conf_.set_result_path(conf_.result_path());
  core_conf_.set_cloud_publish_interval_sec(0.0);
  core_conf_.set_map_publish_interval_sec(conf_.map_publish_interval_sec());
  core_conf_.set_map_frame(conf_.map_frame());
  core_conf_.set_timestamp_mode(
      conf_.timestamp_mode() == FastLio2Conf::RECEIVE_TIME
          ? apollo::localization::fast_lio2::FastLio2RuntimeConf::RECEIVE_TIME
          : apollo::localization::fast_lio2::FastLio2RuntimeConf::MESSAGE_TIME);

  if (HasFullInitialExtrinsic(conf_)) {
    core_conf_.set_initial_rotation_lidar_to_imu_qx(
        conf_.initial_rotation_lidar_to_imu_qx());
    core_conf_.set_initial_rotation_lidar_to_imu_qy(
        conf_.initial_rotation_lidar_to_imu_qy());
    core_conf_.set_initial_rotation_lidar_to_imu_qz(
        conf_.initial_rotation_lidar_to_imu_qz());
    core_conf_.set_initial_rotation_lidar_to_imu_qw(
        conf_.initial_rotation_lidar_to_imu_qw());
    core_conf_.set_initial_translation_lidar_to_imu_x(
        conf_.initial_translation_lidar_to_imu_x());
    core_conf_.set_initial_translation_lidar_to_imu_y(
        conf_.initial_translation_lidar_to_imu_y());
    core_conf_.set_initial_translation_lidar_to_imu_z(
        conf_.initial_translation_lidar_to_imu_z());
    AINFO << "FAST-LIO2 configured with initial lidar-to-imu extrinsic.";
  } else {
    AWARN << "FAST-LIO2 initial lidar-to-imu extrinsic is incomplete; core "
             "will run lidar-only bootstrap and staged initialization until "
             "enough motion is observed.";
  }
  return true;
}

void FastLio2Component::Clear() {
  worker_running_.store(false);
  queue_cv_.notify_all();
  if (worker_thread_.joinable()) {
    if (worker_thread_.get_id() == std::this_thread::get_id()) {
      worker_thread_.detach();
    } else {
      worker_thread_.join();
    }
  }
}

void FastLio2Component::PointCloudCallback(
    const apollo::cyber::adaptive::PointCloudView& cloud,
    double receive_time_sec) {
  ++lidar_frames_received_;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const size_t max_pending = static_cast<size_t>(
        std::max(1, conf_.max_pending_pointcloud_frames()));
    if (pending_clouds_.size() >= max_pending) {
      ++dropped_pointcloud_frames_;
      if (dropped_pointcloud_frames_ % 100 == 1) {
        AWARN << "Skip incoming FAST-LIO2 pointcloud before conversion. "
              << "total_dropped=" << dropped_pointcloud_frames_;
      }
      return;
    }
  }

  apollo::localization::fast_lio2::FastLio2PointCloudFrame pointcloud_frame;
  std::string error;
  if (!apollo::localization::fast_lio2::ConvertPointCloudView(
          cloud, receive_time_sec, core_conf_, &pointcloud_frame, &error)) {
    AWARN << "Drop FAST-LIO2 pointcloud frame: " << error;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const size_t max_pending = static_cast<size_t>(
        std::max(1, conf_.max_pending_pointcloud_frames()));
    while (pending_clouds_.size() >= max_pending) {
      pending_clouds_.pop_front();
      ++dropped_pointcloud_frames_;
    }
    pending_clouds_.push_back(std::move(pointcloud_frame));
  }
  queue_cv_.notify_one();
}

void FastLio2Component::ImuCallback(
    const std::shared_ptr<apollo::localization::CorrectedImu>& imu,
    double receive_time_sec) {
  if (!imu) {
    return;
  }
  ++imu_messages_received_;

  apollo::localization::fast_lio2::FastLio2ImuSample imu_sample;
  std::string error;
  if (!apollo::localization::fast_lio2::ConvertCorrectedImu(*imu, &imu_sample,
                                                            &error)) {
    AWARN << "Drop FAST-LIO2 CorrectedImu: " << error;
    return;
  }
  if (conf_.timestamp_mode() == FastLio2Conf::RECEIVE_TIME) {
    imu_sample.timestamp_sec = receive_time_sec;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const size_t max_pending =
        static_cast<size_t>(std::max(1000, conf_.max_pending_imu_messages()));
    while (pending_imus_.size() >= max_pending) {
      pending_imus_.pop_front();
      ++dropped_imu_messages_;
      if (dropped_imu_messages_ % 1000 == 1) {
        AWARN << "Drop old pending FAST-LIO2 IMU messages. total_dropped="
              << dropped_imu_messages_;
      }
    }
    pending_imus_.push_back(std::move(imu_sample));
  }
  queue_cv_.notify_one();
}

void FastLio2Component::RawImuCallback(
    const std::shared_ptr<apollo::drivers::gnss::Imu>& imu,
    double receive_time_sec) {
  if (!imu) {
    return;
  }
  ++imu_messages_received_;

  apollo::localization::fast_lio2::FastLio2ImuSample imu_sample;
  if (conf_.timestamp_mode() == FastLio2Conf::RECEIVE_TIME) {
    imu_sample.timestamp_sec = receive_time_sec;
  } else if (imu->has_header() && imu->header().timestamp_sec() > 0.0) {
    imu_sample.timestamp_sec = imu->header().timestamp_sec();
  } else if (imu->measurement_time() > 0.0) {
    imu_sample.timestamp_sec =
        apollo::common::util::GpsToUnixSeconds(imu->measurement_time());
  }
  imu_sample.angular_velocity =
      Eigen::Vector3d(imu->angular_velocity().x(), imu->angular_velocity().y(),
                      imu->angular_velocity().z());
  imu_sample.linear_acceleration =
      Eigen::Vector3d(imu->linear_acceleration().x(),
                      imu->linear_acceleration().y(),
                      imu->linear_acceleration().z());

  if (imu_sample.timestamp_sec <= 0.0 ||
      !std::isfinite(imu_sample.angular_velocity.x()) ||
      !std::isfinite(imu_sample.angular_velocity.y()) ||
      !std::isfinite(imu_sample.angular_velocity.z()) ||
      !std::isfinite(imu_sample.linear_acceleration.x()) ||
      !std::isfinite(imu_sample.linear_acceleration.y()) ||
      !std::isfinite(imu_sample.linear_acceleration.z())) {
    AWARN << "Drop FAST-LIO2 raw GNSS IMU: invalid timestamp or non-finite "
             "values.";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const size_t max_pending =
        static_cast<size_t>(std::max(1000, conf_.max_pending_imu_messages()));
    while (pending_imus_.size() >= max_pending) {
      pending_imus_.pop_front();
      ++dropped_imu_messages_;
      if (dropped_imu_messages_ % 1000 == 1) {
        AWARN << "Drop old pending FAST-LIO2 raw IMU messages. total_dropped="
              << dropped_imu_messages_;
      }
    }
    pending_imus_.push_back(std::move(imu_sample));
  }
  queue_cv_.notify_one();
}

void FastLio2Component::WorkerLoop() {
  while (worker_running_.load()) {
    std::vector<apollo::localization::fast_lio2::FastLio2ImuSample> imus;
    apollo::localization::fast_lio2::FastLio2PointCloudFrame cloud;
    bool has_cloud = false;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() {
        return !worker_running_.load() || !pending_imus_.empty() ||
               !pending_clouds_.empty();
      });
      if (!worker_running_.load() && pending_imus_.empty() &&
          pending_clouds_.empty()) {
        break;
      }
      imus.reserve(pending_imus_.size());
      while (!pending_imus_.empty()) {
        imus.push_back(std::move(pending_imus_.front()));
        pending_imus_.pop_front();
      }
      if (!pending_clouds_.empty()) {
        cloud = std::move(pending_clouds_.front());
        pending_clouds_.pop_front();
        has_cloud = true;
      }
    }

    for (const auto& imu : imus) {
      core_.AddImu(imu);
    }
    if (has_cloud) {
      core_.AddPointCloud(cloud);
    }

    const double start = apollo::cyber::Clock::NowInSeconds();
    core_.Process();
    const double end = apollo::cyber::Clock::NowInSeconds();
    PublishOutputs((end - start) * 1000.0);
  }
}

void FastLio2Component::PublishOutputs(double processing_latency_ms) {
  apollo::localization::fast_lio2::FastLio2RuntimeStateSnapshot state;
  const bool has_state = core_.GetStateSnapshot(&state);
  if (has_state) {
    PublishLocalization(state);
    PublishOdometry(state);
  }

  if (cloud_registered_writer_) {
    apollo::drivers::PointCloud cloud;
    if (core_.GetRegisteredCloud(&cloud)) {
      cloud_registered_writer_->Write(cloud);
    }
  }

  const double now = apollo::cyber::Clock::NowInSeconds();
  if (map_writer_ &&
      now - last_map_publish_time_sec_ >= conf_.map_publish_interval_sec()) {
    last_map_publish_time_sec_ = now;
    apollo::drivers::PointCloud map;
    if (core_.GetMapCloud(&map)) {
      map_writer_->Write(map);
    }
  }

  PublishMetrics(has_state ? &state : nullptr, processing_latency_ms);
}

void FastLio2Component::PublishLocalization(
    const apollo::localization::fast_lio2::FastLio2RuntimeStateSnapshot& state) {
  if (!localization_writer_ || !state.scan_match_valid) {
    return;
  }

  apollo::localization::LocalizationEstimate localization;
  auto* header = localization.mutable_header();
  header->set_timestamp_sec(state.timestamp_sec);
  header->set_module_name("fast_lio2");
  header->set_sequence_num(++localization_seq_);
  header->set_frame_id(conf_.map_frame());
  localization.set_measurement_time(state.timestamp_sec);

  Eigen::Quaterniond q(state.rotation);
  q.normalize();
  auto* pose = localization.mutable_pose();
  FillPointENU(state.position, pose->mutable_position());
  FillQuaternion(q, pose->mutable_orientation());
  FillPoint3D(state.velocity, pose->mutable_linear_velocity());
  pose->set_heading(HeadingFromQuaternion(q));

  if (state.covariance.rows() >= 6 && state.covariance.cols() >= 6) {
    localization.mutable_uncertainty()->mutable_orientation_std_dev()->set_x(
        std::sqrt(std::max(0.0, state.covariance(0, 0))));
    localization.mutable_uncertainty()->mutable_orientation_std_dev()->set_y(
        std::sqrt(std::max(0.0, state.covariance(1, 1))));
    localization.mutable_uncertainty()->mutable_orientation_std_dev()->set_z(
        std::sqrt(std::max(0.0, state.covariance(2, 2))));
    localization.mutable_uncertainty()->mutable_position_std_dev()->set_x(
        std::sqrt(std::max(0.0, state.covariance(3, 3))));
    localization.mutable_uncertainty()->mutable_position_std_dev()->set_y(
        std::sqrt(std::max(0.0, state.covariance(4, 4))));
    localization.mutable_uncertainty()->mutable_position_std_dev()->set_z(
        std::sqrt(std::max(0.0, state.covariance(5, 5))));
  }

  localization_writer_->Write(localization);
}

void FastLio2Component::PublishOdometry(
    const apollo::localization::fast_lio2::FastLio2RuntimeStateSnapshot& state) {
  if (!odometry_writer_ || !state.scan_match_valid) {
    return;
  }

  apollo::localization::Odometry odom;
  auto* header = odom.mutable_header();
  header->set_timestamp_sec(state.timestamp_sec);
  header->set_module_name("fast_lio2");
  header->set_sequence_num(++odometry_seq_);
  header->set_frame_id(conf_.map_frame());
  odom.set_child_frame_id(conf_.child_frame());
  odom.set_factor_type(apollo::localization::Odometry::LIDAR);

  Eigen::Quaterniond q(state.rotation);
  q.normalize();
  auto* pose = odom.mutable_pose()->mutable_pose();
  FillPointENU(state.position, pose->mutable_position());
  FillQuaternion(q, pose->mutable_orientation());
  FillPoint3D(state.velocity, pose->mutable_linear_velocity());
  pose->set_heading(HeadingFromQuaternion(q));

  auto* cov = odom.mutable_pose()->mutable_covariance();
  cov->Reserve(6);
  for (int i = 0; i < 6; ++i) {
    cov->Add(state.covariance(i, i));
  }
  FillVector3(state.velocity,
              odom.mutable_twist()->mutable_twist()->mutable_linear());

  odometry_writer_->Write(odom);
}

void FastLio2Component::PublishMetrics(
    const apollo::localization::fast_lio2::FastLio2RuntimeStateSnapshot* state,
    double processing_latency_ms) {
  if (!metrics_writer_) {
    return;
  }
  FastLio2Metrics metrics;
  auto* header = metrics.mutable_header();
  header->set_timestamp_sec(state ? state->timestamp_sec
                                  : apollo::cyber::Clock::NowInSeconds());
  header->set_module_name("fast_lio2");
  header->set_sequence_num(++metrics_seq_);
  header->set_frame_id(conf_.map_frame());

  metrics.set_lidar_frames_received(lidar_frames_received_);
  metrics.set_imu_messages_received(imu_messages_received_);
  metrics.set_dropped_pointcloud_frames(dropped_pointcloud_frames_);
  metrics.set_dropped_imu_messages(dropped_imu_messages_);
  metrics.set_last_processing_latency_ms(processing_latency_ms);
  if (state != nullptr) {
    metrics.set_synced_frames_processed(state->frame_count);
    metrics.set_last_lidar_timestamp_sec(state->timestamp_sec);
    metrics.set_mean_point_to_plane_residual(
        state->mean_point_to_plane_residual);
    metrics.set_effective_feature_count(state->effective_feature_count);
    metrics.set_map_point_count(state->map_point_count);
    metrics.set_scan_match_valid(state->scan_match_valid);
    metrics.set_stage(state->stage);
    metrics.set_tracking_status(state->tracking_status);
    metrics.set_reject_reason(state->reject_reason);
    metrics.set_scan_match_inlier_ratio(state->scan_match_inlier_ratio);
    metrics.set_scan_match_delta_rotation_deg(
        state->scan_match_delta_rotation_deg);
    metrics.set_scan_match_delta_translation_m(
        state->scan_match_delta_translation_m);
  }
  metrics_writer_->Write(metrics);
}

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
