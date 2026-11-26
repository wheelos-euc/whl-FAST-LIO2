#pragma once

// Adapted from hku-mars/FAST_LIO src/IMU_Processing.hpp at 7cc4175.
// Upstream licensing and the adaptation boundary are recorded in
// upstream_sources.txt.

#include <algorithm>
#include <cmath>
#include <vector>

#include "modules/fast_lio2/core/include/common_lib.h"
#include "modules/fast_lio2/core/include/use-ikfom.hpp"

using OfficialFastLioFilter = esekfom::esekf<state_ikfom, 12, input_ikfom>;

class ImuProcess {
 public:
  void set_extrinsic(const V3D& translation, const M3D& rotation) {
    translation_ = translation;
    rotation_ = rotation;
  }
  void set_gyr_cov(const V3D& covariance) { cov_gyr_ = covariance; }
  void set_acc_cov(const V3D& covariance) { cov_acc_ = covariance; }
  void set_gyr_bias_cov(const V3D& covariance) { cov_bias_gyr_ = covariance; }
  void set_acc_bias_cov(const V3D& covariance) { cov_bias_acc_ = covariance; }

  bool Process(const MeasureGroup& meas, OfficialFastLioFilter& filter,
               PointCloudXYZI::Ptr cloud) {
    cloud->clear();
    if (meas.imu.empty() || !meas.lidar || meas.lidar->empty()) return false;
    if (need_init_) {
      Initialize(meas, filter);
      return false;
    }
    Undistort(meas, filter, *cloud);
    return true;
  }

 private:
  void Initialize(const MeasureGroup& meas, OfficialFastLioFilter& filter) {
    if (sample_count_ == 1) {
      const auto& imu = *meas.imu.front();
      mean_acc_ << imu.linear_acceleration.x, imu.linear_acceleration.y,
          imu.linear_acceleration.z;
      mean_gyr_ << imu.angular_velocity.x, imu.angular_velocity.y,
          imu.angular_velocity.z;
    }
    for (const auto& imu : meas.imu) {
      const V3D acc(imu->linear_acceleration.x, imu->linear_acceleration.y,
                    imu->linear_acceleration.z);
      const V3D gyr(imu->angular_velocity.x, imu->angular_velocity.y,
                    imu->angular_velocity.z);
      mean_acc_ += (acc - mean_acc_) / sample_count_;
      mean_gyr_ += (gyr - mean_gyr_) / sample_count_;
      ++sample_count_;
    }
    auto state = filter.get_x();
    state.grav = S2(-mean_acc_ / mean_acc_.norm() * G_m_s2);
    state.bg = mean_gyr_;
    state.offset_R_L_I = rotation_;
    state.offset_T_L_I = translation_;
    filter.change_x(state);
    auto covariance = filter.get_P();
    covariance.setIdentity();
    covariance.diagonal().segment<6>(6).setConstant(0.00001);
    covariance.diagonal().segment<3>(15).setConstant(0.0001);
    covariance.diagonal().segment<3>(18).setConstant(0.001);
    covariance.diagonal().segment<2>(21).setConstant(0.00001);
    filter.change_P(covariance);
    last_imu_ = meas.imu.back();
    need_init_ = sample_count_ <= 10;
  }

  void Undistort(const MeasureGroup& meas, OfficialFastLioFilter& filter,
                 PointCloudXYZI& cloud) {
    auto imus = meas.imu;
    imus.push_front(last_imu_);
    const double imu_end = imus.back()->header.stamp.toSec();
    cloud = *meas.lidar;
    std::sort(cloud.begin(), cloud.end(),
              [](const PointType& a, const PointType& b) {
                return a.curvature < b.curvature;
              });

    auto state = filter.get_x();
    std::vector<Pose6D> poses;
    poses.push_back(set_pose6d(0.0, last_acc_, last_gyr_, state.vel, state.pos,
                               state.rot.toRotationMatrix()));
    input_ikfom input;
    auto noise = process_noise_cov();
    noise.block<3, 3>(0, 0).diagonal() = cov_gyr_;
    noise.block<3, 3>(3, 3).diagonal() = cov_acc_;
    noise.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
    noise.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;
    for (auto it = imus.begin(); it + 1 != imus.end(); ++it) {
      const auto& head = **it;
      const auto& tail = **(it + 1);
      if (tail.header.stamp.toSec() < last_scan_end_) continue;
      input.gyro << 0.5 * (head.angular_velocity.x + tail.angular_velocity.x),
          0.5 * (head.angular_velocity.y + tail.angular_velocity.y),
          0.5 * (head.angular_velocity.z + tail.angular_velocity.z);
      input.acc << 0.5 * (head.linear_acceleration.x +
                          tail.linear_acceleration.x),
          0.5 * (head.linear_acceleration.y + tail.linear_acceleration.y),
          0.5 * (head.linear_acceleration.z + tail.linear_acceleration.z);
      input.acc *= G_m_s2 / mean_acc_.norm();
      double dt = tail.header.stamp.toSec() -
                  std::max(head.header.stamp.toSec(), last_scan_end_);
      filter.predict(dt, noise, input);
      state = filter.get_x();
      last_gyr_ = input.gyro - state.bg;
      last_acc_ = state.rot * (input.acc - state.ba);
      for (int i = 0; i < 3; ++i) last_acc_[i] += state.grav[i];
      poses.push_back(set_pose6d(
          tail.header.stamp.toSec() - meas.lidar_beg_time, last_acc_, last_gyr_,
          state.vel, state.pos, state.rot.toRotationMatrix()));
    }
    double dt_to_end = std::abs(meas.lidar_end_time - imu_end);
    filter.predict(dt_to_end, noise, input);
    state = filter.get_x();
    last_imu_ = meas.imu.back();
    last_scan_end_ = meas.lidar_end_time;

    auto point = cloud.end() - 1;
    for (auto pose = poses.end() - 1; pose != poses.begin(); --pose) {
      const auto& head = *(pose - 1);
      const auto& tail = *pose;
      M3D rotation;
      rotation << MAT_FROM_ARRAY(head.rot);
      const V3D velocity(VEC_FROM_ARRAY(head.vel));
      const V3D position(VEC_FROM_ARRAY(head.pos));
      const V3D acceleration(VEC_FROM_ARRAY(tail.acc));
      const V3D gyro(VEC_FROM_ARRAY(tail.gyr));
      for (; point->curvature * 1e-3 > head.offset_time; --point) {
        const double dt = point->curvature * 1e-3 - head.offset_time;
        const M3D at_point = rotation * Exp(gyro, dt);
        const V3D original(point->x, point->y, point->z);
        const V3D translation =
            position + velocity * dt + 0.5 * acceleration * dt * dt - state.pos;
        const V3D compensated =
            state.offset_R_L_I.conjugate() *
            (state.rot.conjugate() *
                 (at_point *
                      (state.offset_R_L_I * original + state.offset_T_L_I) +
                  translation) -
             state.offset_T_L_I);
        point->x = compensated.x();
        point->y = compensated.y();
        point->z = compensated.z();
        if (point == cloud.begin()) break;
      }
    }
  }

  V3D translation_ = Zero3d;
  M3D rotation_ = Eye3d;
  V3D cov_gyr_ = V3D::Constant(0.1);
  V3D cov_acc_ = V3D::Constant(0.1);
  V3D cov_bias_gyr_ = V3D::Constant(0.0001);
  V3D cov_bias_acc_ = V3D::Constant(0.0001);
  V3D mean_acc_ = Zero3d;
  V3D mean_gyr_ = Zero3d;
  V3D last_acc_ = Zero3d;
  V3D last_gyr_ = Zero3d;
  sensor_msgs::Imu::ConstPtr last_imu_;
  double last_scan_end_ = 0.0;
  int sample_count_ = 1;
  bool need_init_ = true;
};
