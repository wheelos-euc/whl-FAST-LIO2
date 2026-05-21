#include "modules/fast_lio2/core/fast_lio2_runtime_core.h"

#include <omp.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <boost/filesystem.hpp>
#include <google/protobuf/repeated_field.h>

#include "cyber/common/log.h"
#include "modules/fast_lio2/adapter/corrected_imu_adapter.h"
#include "modules/fast_lio2/core/imu_processing.h"
#include "modules/fast_lio2/core/include/LI_init/LI_init.h"

namespace apollo {
namespace localization {
namespace fast_lio2 {

namespace {

constexpr double kMoveThreshold = 1.5;
constexpr int kNumMatchPoints = NUM_MATCH_POINTS;
constexpr uint32_t kTrackingLostRejectedFrames = 5;
constexpr int kRefineDebugFrameLimit = 50;

float CalcDist(const PointType& p1, const PointType& p2) {
  const float dx = p1.x - p2.x;
  const float dy = p1.y - p2.y;
  const float dz = p1.z - p2.z;
  return dx * dx + dy * dy + dz * dz;
}

void CalcBodyVar(const Eigen::Vector3d& pb, const float range_inc,
                 const float degree_inc, Eigen::Matrix3d* var) {
  const float range = std::sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  const float range_var = range_inc * range_inc;
  Eigen::Matrix2d direction_var;
  const double degree_rad = degree_inc * M_PI / 180.0;
  direction_var << std::pow(std::sin(degree_rad), 2), 0, 0,
      std::pow(std::sin(degree_rad), 2);
  Eigen::Vector3d direction(pb);
  direction.normalize();
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0,
      -direction(0), -direction(1), direction(0), 0;
  Eigen::Vector3d base_vector1(1, 1,
                               -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> n;
  n << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
      base_vector1(2), base_vector2(2);
  Eigen::Matrix<double, 3, 2> a = range * direction_hat * n;
  *var = direction * range_var * direction.transpose() +
         a * direction_var * a.transpose();
}

void FillMatrixRepeated(const Eigen::Matrix3d& mat,
                        google::protobuf::RepeatedField<double>* out) {
  out->Clear();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out->Add(mat(r, c));
    }
  }
}

void FillVectorRepeated(const Eigen::Vector3d& vec,
                        google::protobuf::RepeatedField<double>* out) {
  out->Clear();
  out->Add(vec.x());
  out->Add(vec.y());
  out->Add(vec.z());
}

std::string VecDebugString(const Eigen::Vector3d& vec) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6) << vec.x() << " " << vec.y()
      << " " << vec.z();
  return oss.str();
}

std::string PoseDebugString(const StatesGroup& state) {
  const Eigen::Vector3d rpy_deg = RotMtoEuler(state.rot_end) * 57.295779513;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6) << "xyz=[" << VecDebugString(state.pos_end)
      << "] rpy_deg=[" << VecDebugString(rpy_deg) << "]";
  return oss.str();
}

std::string PoseDebugString(const Eigen::Matrix3d& rot,
                            const Eigen::Vector3d& pos) {
  const Eigen::Vector3d rpy_deg = RotMtoEuler(rot) * 57.295779513;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6) << "xyz=[" << VecDebugString(pos)
      << "] rpy_deg=[" << VecDebugString(rpy_deg) << "]";
  return oss.str();
}

std::string StateDebugString(const StatesGroup& state) {
  std::ostringstream oss;
  oss << PoseDebugString(state) << " vel=[" << VecDebugString(state.vel_end)
      << "] bias_g_or_cv_w=[" << VecDebugString(state.bias_g)
      << "] bias_a=[" << VecDebugString(state.bias_a)
      << "] gravity=[" << VecDebugString(state.gravity) << "]";
  return oss.str();
}

}  // namespace

bool FastLio2RuntimeCore::Init(const FastLio2RuntimeConf& conf) {
  conf_ = conf;
  num_max_iterations_ = conf.max_iteration();
  orig_odom_freq_ = conf.orig_odom_freq();
  cut_frame_num_ = std::max(1, conf.cut_frame_num());
  mean_acc_norm_ = conf.mean_acc_norm();
  filter_size_surf_min_ = conf.filter_size_surf();
  filter_size_map_min_ = conf.filter_size_map();
  cube_len_ = conf.cube_side_length();
  det_range_ = conf.det_range();

  feats_from_map_.reset(new PointCloudXYZI());
  feats_undistort_.reset(new PointCloudXYZI());
  feats_down_body_.reset(new PointCloudXYZI());
  feats_down_world_.reset(new PointCloudXYZI());
  normvec_.reset(new PointCloudXYZI(100000, 1));
  laser_cloud_ori_.reset(new PointCloudXYZI(100000, 1));
  corr_normvect_.reset(new PointCloudXYZI(100000, 1));
  feats_array_.reset(new PointCloudXYZI());

  point_selected_surf_.assign(100000, true);
  res_last_.assign(100000, -1000.0F);
  jacobian_rot_.resize(30000, 3);
  jacobian_rot_.setZero();

  down_size_filter_surf_.setLeafSize(
      filter_size_surf_min_, filter_size_surf_min_, filter_size_surf_min_);
  down_size_filter_map_.setLeafSize(filter_size_map_min_, filter_size_map_min_,
                                    filter_size_map_min_);

  boost::filesystem::path result_path(conf_.result_path());
  boost::filesystem::path log_dir =
      result_path.parent_path().empty()
          ? boost::filesystem::path("Log")
          : result_path.parent_path() / "Log";
  boost::filesystem::create_directories(log_dir);
  imu_process_ = std::make_shared<ImuProcess>();
  init_li_ = std::make_shared<LI_Init>();
  init_li_->data_accum_length = conf.data_accum_length();
  InitVehiclePrior();

  imu_process_->lidar_type = ROBOSENSE;
  imu_process_->imu_en = false;
  imu_process_->LI_init_done = false;
  imu_process_->undistort_en = true;
  imu_process_->set_mean_acc_norm(mean_acc_norm_);
  imu_process_->set_gyr_cov(
      V3D(conf.gyr_cov(), conf.gyr_cov(), conf.gyr_cov()));
  imu_process_->set_acc_cov(
      V3D(conf.acc_cov(), conf.acc_cov(), conf.acc_cov()));
  imu_process_->set_gyr_bias_cov(
      V3D(conf.b_gyr_cov(), conf.b_gyr_cov(), conf.b_gyr_cov()));
  imu_process_->set_acc_bias_cov(
      V3D(conf.b_acc_cov(), conf.b_acc_cov(), conf.b_acc_cov()));

  if (conf.rot_li_cov_size() >= 3) {
    imu_process_->set_R_LI_cov(
        V3D(conf.rot_li_cov(0), conf.rot_li_cov(1), conf.rot_li_cov(2)));
  }
  if (conf.trans_li_cov_size() >= 3) {
    imu_process_->set_T_LI_cov(
        V3D(conf.trans_li_cov(0), conf.trans_li_cov(1), conf.trans_li_cov(2)));
  }
  SetStage(Stage::WAITING_FOR_DATA, "waiting for point cloud and imu");
  AINFO << "FastLio2RuntimeCore initialized. result_path="
        << conf_.result_path();
  return true;
}

bool FastLio2RuntimeCore::InitVehiclePrior() {
  const bool has_rotation = conf_.has_initial_rotation_lidar_to_imu_qx() &&
                            conf_.has_initial_rotation_lidar_to_imu_qy() &&
                            conf_.has_initial_rotation_lidar_to_imu_qz() &&
                            conf_.has_initial_rotation_lidar_to_imu_qw();
  const bool has_translation = conf_.has_initial_translation_lidar_to_imu_x() &&
                               conf_.has_initial_translation_lidar_to_imu_y() &&
                               conf_.has_initial_translation_lidar_to_imu_z();
  vehicle_prior_mode_ = has_rotation && has_translation;
  if (!vehicle_prior_mode_) {
    if (has_rotation || has_translation ||
        conf_.has_initial_rotation_lidar_to_imu_qx() ||
        conf_.has_initial_rotation_lidar_to_imu_qy() ||
        conf_.has_initial_rotation_lidar_to_imu_qz() ||
        conf_.has_initial_rotation_lidar_to_imu_qw() ||
        conf_.has_initial_translation_lidar_to_imu_x() ||
        conf_.has_initial_translation_lidar_to_imu_y() ||
        conf_.has_initial_translation_lidar_to_imu_z()) {
      AWARN << "Incomplete initial extrinsic. Vehicle prior mode disabled. "
            << "has_rotation=" << has_rotation
            << " has_translation=" << has_translation;
    }
    return false;
  }

  Eigen::Quaterniond q_li(conf_.initial_rotation_lidar_to_imu_qw(),
                          conf_.initial_rotation_lidar_to_imu_qx(),
                          conf_.initial_rotation_lidar_to_imu_qy(),
                          conf_.initial_rotation_lidar_to_imu_qz());
  const double q_norm = q_li.norm();
  if (!std::isfinite(q_norm) || q_norm < 1e-8) {
    AERROR << "Invalid initial lidar-to-imu quaternion. Vehicle prior mode "
           << "disabled.";
    vehicle_prior_mode_ = false;
    return false;
  }
  q_li.normalize();
  initial_R_LI_ = q_li.toRotationMatrix();
  initial_T_LI_ = V3D(conf_.initial_translation_lidar_to_imu_x(),
                      conf_.initial_translation_lidar_to_imu_y(),
                      conf_.initial_translation_lidar_to_imu_z());

  init_li_->set_vehicle_prior(initial_R_LI_, initial_T_LI_,
                              conf_.vehicle_roll_pitch_prior_sigma_deg(),
                              conf_.vehicle_yaw_prior_sigma_deg(),
                              conf_.vehicle_translation_prior_sigma_m());
  AINFO << "Vehicle prior mode enabled. initial_R_LI=\n"
        << initial_R_LI_ << "\ninitial_T_LI=" << initial_T_LI_.transpose()
        << " yaw_progress_threshold=" << conf_.vehicle_yaw_progress_threshold()
        << " mapping_residual_threshold="
        << conf_.vehicle_mapping_residual_threshold()
        << " min_effective_features=" << conf_.vehicle_min_effective_features()
        << " roll_pitch_prior_sigma_deg="
        << conf_.vehicle_roll_pitch_prior_sigma_deg()
        << " yaw_prior_sigma_deg=" << conf_.vehicle_yaw_prior_sigma_deg()
        << " translation_prior_sigma_m="
        << conf_.vehicle_translation_prior_sigma_m();
  return true;
}

bool FastLio2RuntimeCore::VehicleDataSufficient(const V3D& progress) const {
  if (!vehicle_prior_mode_) return false;
  const double yaw_like_progress =
      std::max(progress.x(), std::max(progress.y(), progress.z()));
  const bool motion_ok =
      yaw_like_progress >= conf_.vehicle_yaw_progress_threshold();
  const bool mapping_ok =
      mean_point_to_plane_residual_ <=
          conf_.vehicle_mapping_residual_threshold() &&
      effect_feat_num_ >= conf_.vehicle_min_effective_features();
  return motion_ok && mapping_ok;
}

void FastLio2RuntimeCore::AddPointCloud(const FastLio2PointCloudFrame& cloud) {
  if (!cloud.points || cloud.points->empty()) return;
  if (cloud.scan_start_sec < last_timestamp_lidar_) {
    AWARN << "LiDAR timestamp loop detected, clearing LiDAR buffer.";
    lidar_buffer_.clear();
    time_buffer_.clear();
    lidar_pushed_ = false;
    last_init_sample_set_ = false;
  }
  last_timestamp_lidar_ = cloud.scan_start_sec;
  lidar_buffer_.push_back(cloud.points);
  time_buffer_.push_back(cloud.scan_start_sec);
}

void FastLio2RuntimeCore::AddImu(const FastLio2ImuSample& imu) {
  auto msg = ToCompatImu(imu);
  if (imu_en_) {
    msg->header.stamp = compat::Time::FromSec(msg->header.stamp.toSec() -
                                              time_lag_imu_wrt_lidar_);
  }
  const double msg_timestamp_sec = msg->header.stamp.toSec();
  if (msg_timestamp_sec < last_timestamp_imu_) {
    AWARN << "IMU timestamp loop detected, clearing IMU buffer.";
    imu_buffer_.clear();
    init_li_->IMU_buffer_clear();
    last_init_sample_set_ = false;
  }
  last_timestamp_imu_ = msg_timestamp_sec;
  imu_buffer_.push_back(msg);
  if (!imu_en_ && !bootstrap_data_ready_) {
    init_li_->push_ALL_IMU_CalibState(msg, mean_acc_norm_);
  }
}

bool FastLio2RuntimeCore::Process() {
  if (finished_ || failed_) return finished_;
  bool did_work = false;
  while (SyncPackages(&measures_)) {
    did_work = true;
    ProcessMeasure(measures_);
    if (finished_ || failed_) break;
  }
  if (!did_work && stage_ == Stage::WAITING_FOR_DATA &&
      !lidar_buffer_.empty() && !imu_buffer_.empty()) {
    SetStage(Stage::LIDAR_ONLY_ODOMETRY, "received lidar and imu data");
  }
  return finished_;
}

bool FastLio2RuntimeCore::SyncPackages(MeasureGroup* meas) {
  if (meas == nullptr || lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }

  if (!lidar_pushed_) {
    meas->lidar = lidar_buffer_.front();
    if (!meas->lidar || meas->lidar->points.size() <= 1) {
      lidar_buffer_.pop_front();
      time_buffer_.pop_front();
      return false;
    }
    meas->lidar_beg_time = time_buffer_.front();
    lidar_beg_time_ = meas->lidar_beg_time;
    lidar_end_time_ =
        meas->lidar_beg_time + meas->lidar->points.back().curvature / 1000.0;
    lidar_pushed_ = true;
  }

  if (last_timestamp_imu_ < lidar_end_time_) return false;

  meas->imu.clear();
  while (!imu_buffer_.empty()) {
    const double imu_time = imu_buffer_.front()->header.stamp.toSec();
    if (imu_time > lidar_end_time_) break;
    meas->imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }
  lidar_buffer_.pop_front();
  time_buffer_.pop_front();
  lidar_pushed_ = false;
  return true;
}

bool FastLio2RuntimeCore::ValidateScanMatch(
    const StatesGroup& state_before_update,
    const StatesGroup& state_after_update) {
  last_scan_match_reject_reason_ = "OK";
  scan_match_inlier_ratio_ =
      feats_down_size_ > 0
          ? static_cast<double>(effect_feat_num_) / feats_down_size_
          : 0.0;
  const M3D dR =
      state_before_update.rot_end.transpose() * state_after_update.rot_end;
  scan_match_delta_rotation_deg_ = Log(dR).norm() * 57.295779513;
  scan_match_delta_translation_m_ =
      (state_after_update.pos_end - state_before_update.pos_end).norm();

  if (!std::isfinite(mean_point_to_plane_residual_)) {
    last_scan_match_reject_reason_ = "NON_FINITE_RESIDUAL";
    return false;
  }
  if (effect_feat_num_ < conf_.vehicle_min_effective_features()) {
    last_scan_match_reject_reason_ = "LOW_EFFECTIVE_FEATURES";
    return false;
  }
  if (scan_match_inlier_ratio_ < 0.25) {
    last_scan_match_reject_reason_ = "LOW_INLIER_RATIO";
    return false;
  }
  if (mean_point_to_plane_residual_ >
      conf_.vehicle_mapping_residual_threshold()) {
    last_scan_match_reject_reason_ = "HIGH_PLANE_RESIDUAL";
    return false;
  }
  if (scan_match_delta_rotation_deg_ > 5.0) {
    last_scan_match_reject_reason_ = "LARGE_ROTATION_UPDATE";
    return false;
  }
  if (scan_match_delta_translation_m_ > 1.0) {
    last_scan_match_reject_reason_ = "LARGE_TRANSLATION_UPDATE";
    return false;
  }
  return true;
}

void FastLio2RuntimeCore::ProcessMeasure(const MeasureGroup& meas) {
  if (feats_undistort_->empty()) {
    first_lidar_time_ = meas.lidar_beg_time;
    imu_process_->first_lidar_time = first_lidar_time_;
  }

  const StatesGroup state_before_frame = state_;
  const bool log_refine_debug =
      imu_en_ && refine_debug_frame_count_ < kRefineDebugFrameLimit;
  auto imu_window_string = [&meas]() {
    std::ostringstream oss;
    oss << "imu_count=" << meas.imu.size();
    if (!meas.imu.empty()) {
      oss << " imu_front=" << std::fixed << std::setprecision(6)
          << meas.imu.front()->header.stamp.toSec()
          << " imu_back=" << meas.imu.back()->header.stamp.toSec();
    }
    return oss.str();
  };
  if (!imu_en_) {
    AINFO << "[FastLio2][cv_model] frame_begin lidar_beg="
          << std::fixed << std::setprecision(6) << meas.lidar_beg_time
          << " lidar_end=" << lidar_end_time_
          << " prev_state " << StateDebugString(state_before_frame);
  } else if (log_refine_debug) {
    AINFO << "[FastLio2][imu_model] frame_begin refine_debug_frame="
          << refine_debug_frame_count_
          << " lidar_beg=" << std::fixed << std::setprecision(6)
          << meas.lidar_beg_time << " lidar_end=" << lidar_end_time_
          << " " << imu_window_string()
          << " prev_imu_state " << StateDebugString(state_before_frame)
          << " prev_lidar_pose "
          << PoseDebugString(state_before_frame.rot_end *
                                 state_before_frame.offset_R_L_I,
                             state_before_frame.rot_end *
                                     state_before_frame.offset_T_L_I +
                                 state_before_frame.pos_end);
  }
  if (!imu_process_->Process(meas, state_, feats_undistort_)) {
    if (log_refine_debug) {
      AINFO << "[FastLio2][imu_model] process_skipped "
            << "refine_debug_frame=" << refine_debug_frame_count_
            << " lidar_beg=" << std::fixed << std::setprecision(6)
            << meas.lidar_beg_time << " lidar_end=" << lidar_end_time_
            << " " << imu_window_string()
            << " state " << StateDebugString(state_);
      ++refine_debug_frame_count_;
    }
    return;
  }
  state_propagat_ = state_;
  if (!imu_en_) {
    AINFO << "[FastLio2][cv_model] predicted_initial "
          << StateDebugString(state_propagat_);
  } else if (log_refine_debug) {
    AINFO << "[FastLio2][imu_model] predicted_initial "
          << "refine_debug_frame=" << refine_debug_frame_count_
          << " imu_state " << StateDebugString(state_propagat_)
          << " lidar_pose "
          << PoseDebugString(state_propagat_.rot_end *
                                 state_propagat_.offset_R_L_I,
                             state_propagat_.rot_end *
                                     state_propagat_.offset_T_L_I +
                                 state_propagat_.pos_end);
  }

  LasermapFovSegment();

  down_size_filter_surf_.setInputCloud(feats_undistort_);
  down_size_filter_surf_.filter(*feats_down_body_);
  feats_down_size_ = static_cast<int>(feats_down_body_->points.size());
  if (feats_down_size_ <= 5) return;

  if (ikdtree_.Root_Node == nullptr) {
    ikdtree_.set_downsample_param(filter_size_map_min_);
    feats_down_world_->resize(feats_down_size_);
    for (int i = 0; i < feats_down_size_; ++i) {
      PointBodyToWorld(&feats_down_body_->points[i],
                       &feats_down_world_->points[i]);
    }
    ikdtree_.Build(feats_down_world_->points);
    return;
  }

  normvec_->resize(feats_down_size_);
  feats_down_world_->resize(feats_down_size_);
  point_search_ind_surf_.resize(feats_down_size_);
  nearest_points_.resize(feats_down_size_);

  auto run_scan_to_map = [&](const char* pass_name) {
    if (!imu_en_) {
      AINFO << "[FastLio2][scan2map][" << pass_name
            << "] start initial_pose " << PoseDebugString(state_)
            << " feats_down=" << feats_down_size_;
    } else if (log_refine_debug) {
      AINFO << "[FastLio2][scan2map][" << pass_name
            << "] start refine_debug_frame=" << refine_debug_frame_count_
            << " imu_pose " << PoseDebugString(state_)
            << " lidar_pose "
            << PoseDebugString(state_.rot_end * state_.offset_R_L_I,
                               state_.rot_end * state_.offset_T_L_I +
                                   state_.pos_end)
            << " feats_down=" << feats_down_size_;
    }
    VD(DIM_STATE) solution;
    MD(DIM_STATE, DIM_STATE) h_t_h;
    MD(DIM_STATE, DIM_STATE) i_state;
    h_t_h.setZero();
    i_state.setIdentity();

    bool nearest_search_en = true;
    int rematch_num = 0;
    for (int iter = 0; iter < num_max_iterations_; ++iter) {
      laser_cloud_ori_->clear();
      corr_normvect_->clear();

#ifdef MP_EN
      omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
      for (int i = 0; i < feats_down_size_; ++i) {
        PointType& point_body = feats_down_body_->points[i];
        PointType& point_world = feats_down_world_->points[i];
        PointBodyToWorld(&point_body, &point_world);
        std::vector<float> point_search_sq_dis(kNumMatchPoints);
        auto& points_near = nearest_points_[i];
        if (nearest_search_en) {
          ikdtree_.Nearest_Search(point_world, kNumMatchPoints, points_near,
                                  point_search_sq_dis, 5);
          point_selected_surf_[i] =
              points_near.size() >= kNumMatchPoints &&
              point_search_sq_dis[kNumMatchPoints - 1] <= 5;
        }
        res_last_[i] = -1000.0F;
        if (!point_selected_surf_[i] || points_near.size() < kNumMatchPoints) {
          point_selected_surf_[i] = false;
          continue;
        }
        point_selected_surf_[i] = false;
        VD(4) pabcd;
        pabcd.setZero();
        if (esti_plane(pabcd, points_near, 0.1)) {
          const Eigen::Vector3d p_body(point_body.x, point_body.y,
                                       point_body.z);
          const float pd2 = pabcd(0) * point_world.x +
                            pabcd(1) * point_world.y +
                            pabcd(2) * point_world.z + pabcd(3);
          const float s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(p_body.norm());
          if (s > 0.9) {
            point_selected_surf_[i] = true;
            normvec_->points[i].x = s * pabcd(0);
            normvec_->points[i].y = s * pabcd(1);
            normvec_->points[i].z = s * pabcd(2);
            normvec_->points[i].intensity = s * pd2;
            res_last_[i] = std::fabs(pd2);
          }
        }
      }

      effect_feat_num_ = 0;
      double residual_sum = 0.0;
      for (int i = 0; i < feats_down_size_; ++i) {
        if (point_selected_surf_[i]) {
          laser_cloud_ori_->push_back(feats_down_body_->points[i]);
          corr_normvect_->push_back(normvec_->points[i]);
          residual_sum += std::fabs(normvec_->points[i].intensity);
          ++effect_feat_num_;
        }
      }
      if (effect_feat_num_ < 1) break;
      mean_point_to_plane_residual_ = residual_sum / effect_feat_num_;

      Eigen::MatrixXd hsub(effect_feat_num_, 12);
      Eigen::MatrixXd hsub_t_r_inv(12, effect_feat_num_);
      Eigen::VectorXd meas_vec(effect_feat_num_);
      hsub.setZero();
      hsub_t_r_inv.setZero();
      meas_vec.setZero();

      for (int i = 0; i < effect_feat_num_; ++i) {
        const PointType& laser_p = laser_cloud_ori_->points[i];
        V3D point_this_l(laser_p.x, laser_p.y, laser_p.z);
        V3D point_this =
            state_.offset_R_L_I * point_this_l + state_.offset_T_L_I;
        M3D var;
        CalcBodyVar(point_this, 0.02, 0.05, &var);
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        const PointType& norm_p = corr_normvect_->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
        if (imu_en_) {
          M3D point_this_l_cross;
          point_this_l_cross << SKEW_SYM_MATRX(point_this_l);
          V3D h_r_li = point_this_l_cross * state_.offset_R_L_I.transpose() *
                       state_.rot_end.transpose() * norm_vec;
          V3D h_t_li = state_.rot_end.transpose() * norm_vec;
          V3D a(point_crossmat * state_.rot_end.transpose() * norm_vec);
          hsub.row(i) << VEC_FROM_ARRAY(a), norm_p.x, norm_p.y, norm_p.z,
              VEC_FROM_ARRAY(h_r_li), VEC_FROM_ARRAY(h_t_li);
        } else {
          V3D a(point_crossmat * state_.rot_end.transpose() * norm_vec);
          hsub.row(i) << VEC_FROM_ARRAY(a), norm_p.x, norm_p.y, norm_p.z, 0, 0,
              0, 0, 0, 0;
        }
        hsub_t_r_inv.col(i) = hsub.row(i).transpose() * 1000;
        meas_vec(i) = -norm_p.intensity;
      }

      Eigen::MatrixXd k(DIM_STATE, effect_feat_num_);
      h_t_h.block<12, 12>(0, 0) = hsub_t_r_inv * hsub;
      MD(DIM_STATE, DIM_STATE) k_1 = (h_t_h + state_.cov.inverse()).inverse();
      k = k_1.block<DIM_STATE, 12>(0, 0) * hsub_t_r_inv;
      auto vec = state_propagat_ - state_;
      solution = k * meas_vec + vec - k * hsub * vec.block<12, 1>(0, 0);
      state_ += solution;

      const V3D rot_add = solution.block<3, 1>(0, 0);
      const V3D t_add = solution.block<3, 1>(3, 0);
      if (imu_en_) {
        extrinsic_delta_rotation_deg_ =
            solution.block<3, 1>(6, 0).norm() * 57.3;
        extrinsic_delta_translation_m_ = solution.block<3, 1>(9, 0).norm();
      } else {
        extrinsic_delta_rotation_deg_ = 0.0;
        extrinsic_delta_translation_m_ = 0.0;
      }
      const bool converged =
          (rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015);

      nearest_search_en = false;
      if (converged ||
          ((rematch_num == 0) && (iter == num_max_iterations_ - 2))) {
        nearest_search_en = true;
        ++rematch_num;
      }

      if (rematch_num >= 2 || iter == num_max_iterations_ - 1) {
        MD(DIM_STATE, DIM_STATE) g;
        g.setZero();
        g.block<DIM_STATE, 12>(0, 0) = k * hsub;
        state_.cov = (i_state - g) * state_.cov;
        break;
      }
    }
  };

  auto log_scan_match_result = [&](const char* pass_name, bool valid) {
    AINFO << "[FastLio2][scan2map][" << pass_name
          << "] result"
          << (imu_en_ ? " refine_debug_frame=" +
                            std::to_string(refine_debug_frame_count_)
                      : "")
          << " valid=" << valid
          << " pose " << PoseDebugString(state_)
          << " residual=" << std::fixed << std::setprecision(6)
          << mean_point_to_plane_residual_
          << " effective_features=" << effect_feat_num_
          << " feats_down=" << feats_down_size_
          << " inlier_ratio=" << scan_match_inlier_ratio_
          << " match_dR_deg=" << scan_match_delta_rotation_deg_
          << " match_dT_m=" << scan_match_delta_translation_m_
          << " reject_reason=" << last_scan_match_reject_reason_
          << (effect_feat_num_ == 0 ? " residual_may_be_stale=1" : "");
  };

  run_scan_to_map("normal");

  // gate for rejecting frame
  StatesGroup state_before_match = state_propagat_;
  bool scan_match_valid = ValidateScanMatch(state_before_match, state_);
  log_scan_match_result("normal", scan_match_valid);
  bool recovered_by_retry = false;
  bool retried_from_last_valid_pose = false;
  std::string first_reject_reason = last_scan_match_reject_reason_;

  if (!scan_match_valid && !imu_en_ &&
      last_scan_match_reject_reason_ != "LOW_EFFECTIVE_FEATURES" &&
      last_scan_match_reject_reason_ != "NON_FINITE_RESIDUAL") {
    retried_from_last_valid_pose = true;
    first_reject_reason = last_scan_match_reject_reason_;
    state_ = state_before_frame;
    state_propagat_ = state_before_frame;
    AINFO << "[FastLio2][retry] reset_to_last_valid_pose "
          << StateDebugString(state_);
    run_scan_to_map("retry");
    state_before_match = state_propagat_;
    scan_match_valid = ValidateScanMatch(state_before_match, state_);
    log_scan_match_result("retry", scan_match_valid);
    recovered_by_retry = scan_match_valid;
  }

  if (!scan_match_valid) {
    state_ = imu_en_ ? state_before_match : state_before_frame;
    if (!imu_en_) {
      AINFO << "[FastLio2][cv_model] reject_rollback "
            << StateDebugString(state_);
    } else if (log_refine_debug) {
      AINFO << "[FastLio2][imu_model] reject_keep_propagated_state "
            << "refine_debug_frame=" << refine_debug_frame_count_
            << " " << StateDebugString(state_)
            << " lidar_pose "
            << PoseDebugString(state_.rot_end * state_.offset_R_L_I,
                               state_.rot_end * state_.offset_T_L_I +
                                   state_.pos_end);
    }
    last_scan_match_valid_ = false;
    ++consecutive_rejected_frames_;
    if (retried_from_last_valid_pose) {
      if (consecutive_rejected_frames_ >= kTrackingLostRejectedFrames) {
        status_.set_message("tracking lost after retry from last valid pose; "
                            "first reject reason: " +
                            first_reject_reason +
                            "; stay near the mapped area and drive slowly");
      } else {
        status_.set_message("scan match rejected after retry from last valid "
                            "pose; first reject reason: " +
                            first_reject_reason);
      }
    } else if (consecutive_rejected_frames_ >= kTrackingLostRejectedFrames) {
      status_.set_message(
          "tracking lost: stay near the mapped area and drive slowly to "
          "recover");
    } else {
      status_.set_message(
          "scan match rejected: keep slow near the mapped area and do not "
          "drive away");
    }
    PublishProgressStatus();
    if (imu_en_) {
      ++refine_debug_frame_count_;
    }
    return;
  }

  if (recovered_by_retry) {
    // The retry recovered pose using scan-to-map, but the CV motion state
    // that produced the failed prediction should not seed the next frame.
    AINFO << "[FastLio2][cv_model] reset_before "
          << StateDebugString(state_);
    state_.vel_end = Zero3d;
    state_.bias_g = Zero3d;
    state_.bias_a = Zero3d;
    state_.cov.block<9, DIM_STATE>(12, 0).setZero();
    state_.cov.block<DIM_STATE, 9>(0, 12).setZero();
    state_.cov.block<3, 3>(12, 12) = M3D::Identity() * INIT_COV;
    state_.cov.block<3, 3>(15, 15) = M3D::Identity() * INIT_COV;
    state_.cov.block<3, 3>(18, 18) = M3D::Identity() * INIT_COV;
    AINFO << "[FastLio2][cv_model] reset_after "
          << StateDebugString(state_);
    status_.set_message(
        "scan match recovered by retry; CV motion state reset");
  } else if (consecutive_rejected_frames_ > 0) {
    status_.set_message(
        "scan match recovered; continue smooth low-speed motion");
  }
  last_scan_match_valid_ = true;
  last_scan_match_reject_reason_ = "OK";
  consecutive_rejected_frames_ = 0;
  total_distance_ += (state_.pos_end - position_last_).norm();
  position_last_ = state_.pos_end;
  MapIncremental();
  ++frame_num_;

  if (refine_enable_pending_ && !imu_en_) {
    EnableRefinementAfterBridgeFrame(meas);
  }

  if (!imu_en_ && !bootstrap_data_started_ && state_.pos_end.norm() > 0.05) {
    bootstrap_data_started_ = true;
    move_start_time_ = lidar_end_time_;
    SetStage(Stage::DATA_ACCUMULATION, "movement detected");
  }

  if (imu_en_ && !online_refine_finished_) {
    ++refine_debug_frame_count_;
    const double elapsed = lidar_end_time_ - online_refine_start_time_;
    const double refine_time = std::max(0.1, conf_.online_refine_time());
    const double progress = std::min(elapsed, refine_time) / refine_time;
    status_.set_refinement_progress(progress);
    PublishProgressStatus();
    if (!refine_print_ && elapsed >= refine_time - 1e-6) {
      refine_print_ = true;
      online_refine_finished_ = true;
      Finish();
    }
  }

  if (!imu_en_ && !bootstrap_data_ready_ && bootstrap_data_started_) {
    V3D omg_lidar = Zero3d;
    if (last_init_sample_set_) {
      const double dt = lidar_end_time_ - last_time_for_init_;
      if (dt > 1e-6) {
        M3D d_rot = last_rot_for_init_.transpose() * state_.rot_end;
        omg_lidar = Log(d_rot) / dt;
      }
    } else {
      last_init_sample_set_ = true;
    }
    lidar_angular_velocity_norm_ = omg_lidar.norm();
    last_rot_for_init_ = state_.rot_end;
    last_time_for_init_ = lidar_end_time_;

    init_li_->push_Lidar_CalibState(state_.rot_end, omg_lidar, state_.vel_end,
                                    lidar_end_time_);
    bootstrap_data_ready_ = init_li_->data_sufficiency_assess(
        jacobian_rot_, frame_num_, omg_lidar, orig_odom_freq_, cut_frame_num_);
    const V3D progress = init_li_->get_last_rot_progress();
    if (!bootstrap_data_ready_ && VehicleDataSufficient(progress)) {
      bootstrap_data_ready_ = true;
      AINFO << "Vehicle prior data accumulation finished. progress="
            << progress.transpose()
            << " residual=" << mean_point_to_plane_residual_
            << " effective_features=" << effect_feat_num_;
    }
    status_.set_progress_x(progress.x());
    status_.set_progress_y(progress.y());
    status_.set_progress_z(progress.z());
    PublishProgressStatus();

    if (bootstrap_data_ready_) {
      SetStage(Stage::INITIALIZING,
               "data accumulation finished, lidar imu initialization begins");
      init_li_->LI_Initialization(orig_odom_freq_, cut_frame_num_,
                                  timediff_imu_wrt_lidar_, move_start_time_);
      pending_R_LI_ = init_li_->get_R_LI();
      pending_T_LI_ = init_li_->get_T_LI();
      pending_gravity_ = init_li_->get_Grav_L0();
      pending_bias_g_ = init_li_->get_gyro_bias();
      pending_bias_a_ = init_li_->get_acc_bias();
      pending_time_lag_imu_wrt_lidar_ = init_li_->get_total_time_lag();
      refine_enable_pending_ = true;
      SetStage(Stage::INITIALIZING,
               "initialization complete, bridging one lidar frame before "
               "online refinement");
    }
  }
}

void FastLio2RuntimeCore::EnableRefinementAfterBridgeFrame(
    const MeasureGroup& meas) {
  if (!refine_enable_pending_) return;
  if (meas.imu.empty()) {
    AWARN << "[FastLio2][refine_transition] bridge frame has no "
             "imu; keep lidar-only bridge until an imu-backed frame arrives";
    return;
  }

  const M3D lidar_rot_before_refine = state_.rot_end;
  const V3D lidar_pos_before_refine = state_.pos_end;

  state_.offset_R_L_I = pending_R_LI_;
  state_.offset_T_L_I = pending_T_LI_;
  state_.pos_end = -state_.rot_end * state_.offset_R_L_I.transpose() *
                       state_.offset_T_L_I +
                   state_.pos_end;
  state_.rot_end = state_.rot_end * state_.offset_R_L_I.transpose();
  state_.gravity = pending_gravity_;
  state_.bias_g = pending_bias_g_;
  state_.bias_a = pending_bias_a_;
  time_lag_imu_wrt_lidar_ = pending_time_lag_imu_wrt_lidar_;

  const M3D lidar_rot_after_refine = state_.rot_end * state_.offset_R_L_I;
  const V3D lidar_pos_after_refine =
      state_.rot_end * state_.offset_T_L_I + state_.pos_end;
  const M3D transition_dR =
      lidar_rot_before_refine.transpose() * lidar_rot_after_refine;
  const double transition_dR_deg = Log(transition_dR).norm() * 57.295779513;
  const double transition_dT_m =
      (lidar_pos_after_refine - lidar_pos_before_refine).norm();
  AINFO << "[FastLio2][refine_transition] bridge_lidar_end="
        << std::fixed << std::setprecision(6) << lidar_end_time_
        << " lidar_pose_before "
        << PoseDebugString(lidar_rot_before_refine, lidar_pos_before_refine)
        << " lidar_pose_after "
        << PoseDebugString(lidar_rot_after_refine, lidar_pos_after_refine)
        << " transition_dR_deg=" << std::setprecision(9)
        << transition_dR_deg << " transition_dT_m=" << transition_dT_m;
  AINFO << "[FastLio2][refine_transition] imu_state "
        << StateDebugString(state_)
        << " R_LI_rpy_deg=["
        << VecDebugString(RotMtoEuler(state_.offset_R_L_I) * 57.295779513)
        << "] T_LI=[" << VecDebugString(state_.offset_T_L_I)
        << "] gravity=[" << VecDebugString(state_.gravity)
        << "] bias_g=[" << VecDebugString(state_.bias_g)
        << "] bias_a=[" << VecDebugString(state_.bias_a)
        << "] time_lag_imu_wrt_lidar=" << std::fixed
        << std::setprecision(9) << time_lag_imu_wrt_lidar_
        << " timediff_imu_wrt_lidar=" << timediff_imu_wrt_lidar_
        << " total_time_lag="
        << time_lag_imu_wrt_lidar_ + timediff_imu_wrt_lidar_;

  std::vector<sensor_msgs::Imu::Ptr> adjusted_imus;
  adjusted_imus.reserve(meas.imu.size() + imu_buffer_.size());
  sensor_msgs::Imu::Ptr seed_imu;
  for (const auto& imu : meas.imu) {
    sensor_msgs::Imu::Ptr shifted(new sensor_msgs::Imu(*imu));
    const double shifted_time =
        shifted->header.stamp.toSec() - time_lag_imu_wrt_lidar_;
    shifted->header.stamp = compat::Time::FromSec(shifted_time);
    if (shifted_time <= lidar_end_time_ + 1e-6) {
      seed_imu = shifted;
    } else {
      adjusted_imus.push_back(shifted);
    }
  }
  if (!seed_imu) {
    seed_imu.reset(new sensor_msgs::Imu(*meas.imu.back()));
    seed_imu->header.stamp = compat::Time::FromSec(
        seed_imu->header.stamp.toSec() - time_lag_imu_wrt_lidar_);
    AWARN << "[FastLio2][refine_transition] no shifted imu sample "
             "before bridge lidar end; seeding with closest available imu at "
          << std::fixed << std::setprecision(6)
          << seed_imu->header.stamp.toSec();
  }

  for (auto& imu : imu_buffer_) {
    imu->header.stamp = compat::Time::FromSec(imu->header.stamp.toSec() -
                                              time_lag_imu_wrt_lidar_);
    adjusted_imus.push_back(imu);
  }
  std::sort(adjusted_imus.begin(), adjusted_imus.end(),
            [](const sensor_msgs::Imu::Ptr& lhs,
               const sensor_msgs::Imu::Ptr& rhs) {
              return lhs->header.stamp.toSec() < rhs->header.stamp.toSec();
            });
  imu_buffer_.clear();
  for (const auto& imu : adjusted_imus) {
    imu_buffer_.push_back(imu);
  }
  if (!imu_buffer_.empty()) {
    last_timestamp_imu_ = imu_buffer_.back()->header.stamp.toSec();
  } else {
    last_timestamp_imu_ = seed_imu->header.stamp.toSec();
  }
  AINFO << "[FastLio2][refine_transition] adjusted_imu_buffer "
        << "size=" << imu_buffer_.size()
        << " seed=" << std::fixed << std::setprecision(6)
        << seed_imu->header.stamp.toSec()
        << " front="
        << (imu_buffer_.empty() ? seed_imu->header.stamp.toSec()
                                : imu_buffer_.front()->header.stamp.toSec())
        << " back="
        << (imu_buffer_.empty() ? seed_imu->header.stamp.toSec()
                                : imu_buffer_.back()->header.stamp.toSec())
        << " last_timestamp_imu=" << last_timestamp_imu_;

  imu_process_->imu_en = true;
  imu_process_->LI_init_done = true;
  imu_process_->set_mean_acc_norm(mean_acc_norm_);
  imu_process_->set_gyr_cov(V3D(0.1, 0.1, 0.1));
  imu_process_->set_acc_cov(V3D(0.1, 0.1, 0.1));
  imu_process_->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  imu_process_->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  imu_process_->SeedLioState(lidar_end_time_, seed_imu);

  imu_en_ = true;
  refine_enable_pending_ = false;
  refine_debug_frame_count_ = 0;
  online_refine_start_time_ = lidar_end_time_;
  SetStage(Stage::REFINING,
           "online refinement begins after lidar-only bridge frame");
}

void FastLio2RuntimeCore::LasermapFovSegment() {
  cub_needrm_.clear();
  V3F x_axis_body(LIDAR_SP_LEN, 0.0, 0.0);
  V3F x_axis_world;
  V3D x_body(x_axis_body[0], x_axis_body[1], x_axis_body[2]);
  V3D x_global =
      state_.rot_end * (state_.offset_R_L_I * x_body + state_.offset_T_L_I) +
      state_.pos_end;
  x_axis_world = x_global.cast<float>();
  (void)x_axis_world;

  const V3D pos_lid = state_.pos_end;
  if (!localmap_initialized_) {
    for (int i = 0; i < 3; ++i) {
      local_map_points_.vertex_min[i] = pos_lid(i) - cube_len_ / 2.0;
      local_map_points_.vertex_max[i] = pos_lid(i) + cube_len_ / 2.0;
    }
    localmap_initialized_ = true;
    return;
  }

  float dist_to_map_edge[3][2];
  bool need_move = false;
  for (int i = 0; i < 3; ++i) {
    dist_to_map_edge[i][0] =
        std::fabs(pos_lid(i) - local_map_points_.vertex_min[i]);
    dist_to_map_edge[i][1] =
        std::fabs(pos_lid(i) - local_map_points_.vertex_max[i]);
    if (dist_to_map_edge[i][0] <= kMoveThreshold * det_range_ ||
        dist_to_map_edge[i][1] <= kMoveThreshold * det_range_) {
      need_move = true;
    }
  }
  if (!need_move) return;

  BoxPointType new_local_map_points = local_map_points_;
  BoxPointType tmp_boxpoints;
  const float mov_dist =
      std::max(static_cast<float>(
                   (cube_len_ - 2.0 * kMoveThreshold * det_range_) * 0.5 * 0.9),
               static_cast<float>(det_range_ * (kMoveThreshold - 1)));
  for (int i = 0; i < 3; ++i) {
    tmp_boxpoints = local_map_points_;
    if (dist_to_map_edge[i][0] <= kMoveThreshold * det_range_) {
      new_local_map_points.vertex_max[i] -= mov_dist;
      new_local_map_points.vertex_min[i] -= mov_dist;
      tmp_boxpoints.vertex_min[i] = local_map_points_.vertex_max[i] - mov_dist;
      cub_needrm_.push_back(tmp_boxpoints);
    } else if (dist_to_map_edge[i][1] <= kMoveThreshold * det_range_) {
      new_local_map_points.vertex_max[i] += mov_dist;
      new_local_map_points.vertex_min[i] += mov_dist;
      tmp_boxpoints.vertex_max[i] = local_map_points_.vertex_min[i] + mov_dist;
      cub_needrm_.push_back(tmp_boxpoints);
    }
  }
  local_map_points_ = new_local_map_points;
  PointsCacheCollect();
}

void FastLio2RuntimeCore::PointsCacheCollect() {
  PointVector points_history;
  ikdtree_.acquire_removed_points(points_history);
  for (const auto& point : points_history) {
    feats_array_->push_back(point);
  }
}

void FastLio2RuntimeCore::MapIncremental() {
  PointVector point_to_add;
  PointVector point_no_need_downsample;
  point_to_add.reserve(feats_down_size_);
  point_no_need_downsample.reserve(feats_down_size_);
  for (int i = 0; i < feats_down_size_; ++i) {
    PointBodyToWorld(&feats_down_body_->points[i],
                     &feats_down_world_->points[i]);
    if (!nearest_points_[i].empty()) {
      const auto& points_near = nearest_points_[i];
      bool need_add = true;
      PointType mid_point;
      mid_point.x =
          std::floor(feats_down_world_->points[i].x / filter_size_map_min_) *
              filter_size_map_min_ +
          0.5 * filter_size_map_min_;
      mid_point.y =
          std::floor(feats_down_world_->points[i].y / filter_size_map_min_) *
              filter_size_map_min_ +
          0.5 * filter_size_map_min_;
      mid_point.z =
          std::floor(feats_down_world_->points[i].z / filter_size_map_min_) *
              filter_size_map_min_ +
          0.5 * filter_size_map_min_;
      const float dist = CalcDist(feats_down_world_->points[i], mid_point);
      if (std::fabs(points_near[0].x - mid_point.x) >
              0.5 * filter_size_map_min_ &&
          std::fabs(points_near[0].y - mid_point.y) >
              0.5 * filter_size_map_min_ &&
          std::fabs(points_near[0].z - mid_point.z) >
              0.5 * filter_size_map_min_) {
        point_no_need_downsample.push_back(feats_down_world_->points[i]);
        continue;
      }
      for (int j = 0;
           j < kNumMatchPoints && j < static_cast<int>(points_near.size());
           ++j) {
        if (CalcDist(points_near[j], mid_point) < dist) {
          need_add = false;
          break;
        }
      }
      if (need_add) point_to_add.push_back(feats_down_world_->points[i]);
    } else {
      point_to_add.push_back(feats_down_world_->points[i]);
    }
  }
  ikdtree_.Add_Points(point_to_add, true);
  ikdtree_.Add_Points(point_no_need_downsample, false);
}

void FastLio2RuntimeCore::PointBodyToWorld(const PointType* pi,
                                               PointType* po) const {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global =
      state_.rot_end * (state_.offset_R_L_I * p_body + state_.offset_T_L_I) +
      state_.pos_end;
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;
  po->intensity = pi->intensity;
}

void FastLio2RuntimeCore::CurrentWorldCloud(PointCloudXYZI* out) const {
  if (out == nullptr || !feats_undistort_) return;
  out->clear();
  out->resize(feats_undistort_->size());
  for (size_t i = 0; i < feats_undistort_->size(); ++i) {
    PointBodyToWorld(&feats_undistort_->points[i], &out->points[i]);
  }
}

void FastLio2RuntimeCore::PointCloudToApollo(
    const PointCloudXYZI& cloud, const std::string& frame_id,
    apollo::drivers::PointCloud* out) const {
  if (out == nullptr) return;
  out->Clear();
  out->mutable_header()->set_timestamp_sec(lidar_end_time_);
  out->mutable_header()->set_frame_id(frame_id);
  out->set_frame_id(frame_id);
  out->set_measurement_time(lidar_end_time_);
  out->set_is_dense(true);
  out->set_width(static_cast<uint32_t>(cloud.size()));
  out->set_height(1);
  auto* points = out->mutable_point();
  points->Reserve(static_cast<int>(cloud.size()));
  for (const auto& src : cloud.points) {
    auto* dst = points->Add();
    dst->set_x(src.x);
    dst->set_y(src.y);
    dst->set_z(src.z);
    dst->set_intensity(static_cast<uint32_t>(std::max(0.0F, src.intensity)));
    dst->set_timestamp(
        static_cast<uint64_t>((lidar_beg_time_ + src.curvature * 1e-3) * 1e9));
  }
}

bool FastLio2RuntimeCore::GetRegisteredCloud(
    apollo::drivers::PointCloud* out) const {
  if (!conf_.publish_visualization() || !last_scan_match_valid_ ||
      !feats_undistort_ || feats_undistort_->empty()) {
    return false;
  }
  PointCloudXYZI world;
  CurrentWorldCloud(&world);
  PointCloudToApollo(world, conf_.map_frame(), out);
  return true;
}

bool FastLio2RuntimeCore::GetInitialExtrinsicCloud(
    apollo::drivers::PointCloud* out) const {
  if (!conf_.publish_visualization() || !vehicle_prior_mode_ || !imu_en_ ||
      !last_scan_match_valid_ || !feats_undistort_ ||
      feats_undistort_->empty()) {
    return false;
  }
  PointCloudXYZI world;
  world.clear();
  world.resize(feats_undistort_->size());
  for (size_t i = 0; i < feats_undistort_->size(); ++i) {
    const auto& src = feats_undistort_->points[i];
    const V3D p_body(src.x, src.y, src.z);
    const V3D p_global =
        state_.rot_end * (initial_R_LI_ * p_body + initial_T_LI_) +
        state_.pos_end;
    auto& dst = world.points[i];
    dst = src;
    dst.x = p_global.x();
    dst.y = p_global.y();
    dst.z = p_global.z();
  }
  PointCloudToApollo(world, conf_.map_frame(), out);
  return true;
}

bool FastLio2RuntimeCore::GetRegisteredBodyCloud(
    apollo::drivers::PointCloud* out) const {
  if (!conf_.publish_visualization() || !feats_undistort_ ||
      feats_undistort_->empty()) {
    return false;
  }
  PointCloudToApollo(*feats_undistort_, "lidar", out);
  return true;
}

bool FastLio2RuntimeCore::GetMapCloud(apollo::drivers::PointCloud* out) {
  if (!conf_.publish_visualization() || !feats_from_map_ ||
      ikdtree_.Root_Node == nullptr) {
    return false;
  }
  PointVector map_points;
  ikdtree_.flatten(ikdtree_.Root_Node, map_points, NOT_RECORD);
  PointCloudXYZI map;
  map.points.assign(map_points.begin(), map_points.end());
  PointCloudToApollo(map, conf_.map_frame(), out);
  return true;
}

bool FastLio2RuntimeCore::GetStateSnapshot(
    FastLio2RuntimeStateSnapshot* out) {
  if (out == nullptr || frame_num_ <= 0 || lidar_end_time_ <= 0.0) {
    return false;
  }
  out->timestamp_sec = lidar_end_time_;
  out->rotation = state_.rot_end;
  out->position = state_.pos_end;
  out->velocity = state_.vel_end;
  out->gravity = state_.gravity;
  out->covariance = state_.cov;
  out->scan_match_valid = last_scan_match_valid_;
  out->stage = StageName(stage_);
  out->tracking_status = status_.tracking_status();
  out->reject_reason = last_scan_match_reject_reason_;
  out->frame_count = static_cast<uint64_t>(frame_num_);
  out->effective_feature_count = effect_feat_num_;
  out->map_point_count = ikdtree_.Root_Node == nullptr ? 0 : ikdtree_.validnum();
  out->mean_point_to_plane_residual = mean_point_to_plane_residual_;
  out->scan_match_inlier_ratio = scan_match_inlier_ratio_;
  out->scan_match_delta_rotation_deg = scan_match_delta_rotation_deg_;
  out->scan_match_delta_translation_m = scan_match_delta_translation_m_;
  return true;
}

void FastLio2RuntimeCore::SetStage(Stage stage,
                                       const std::string& message) {
  stage_ = stage;
  status_.set_stage(StageName(stage));
  status_.set_message(message);
  AINFO << "[FastLio2] stage=" << StageName(stage)
        << " message=" << message;
}

void FastLio2RuntimeCore::PublishProgressStatus() {
  status_.set_motion_detected(bootstrap_data_started_);
  status_.set_data_sufficient(bootstrap_data_ready_);
  status_.set_time_lag_imu_to_lidar(time_lag_imu_wrt_lidar_ +
                                    timediff_imu_wrt_lidar_);
  status_.set_mean_point_to_plane_residual(mean_point_to_plane_residual_);
  status_.set_effective_feature_count(
      static_cast<uint32_t>(std::max(0, effect_feat_num_)));
  status_.set_extrinsic_delta_rotation_deg(extrinsic_delta_rotation_deg_);
  status_.set_extrinsic_delta_translation_m(extrinsic_delta_translation_m_);
  status_.set_gravity_norm(state_.gravity.norm());
  status_.set_lidar_motion_distance(total_distance_);
  status_.set_lidar_angular_velocity_norm(lidar_angular_velocity_norm_);
  status_.set_scan_match_valid(last_scan_match_valid_);
  status_.set_consecutive_rejected_frames(consecutive_rejected_frames_);
  status_.set_scan_match_reject_reason(last_scan_match_reject_reason_);
  status_.set_scan_match_inlier_ratio(scan_match_inlier_ratio_);
  status_.set_scan_match_delta_rotation_deg(scan_match_delta_rotation_deg_);
  status_.set_scan_match_delta_translation_m(scan_match_delta_translation_m_);
  if (last_scan_match_valid_) {
    status_.set_tracking_status("OK");
    status_.set_operator_hint("Continue smooth low-speed motion.");
  } else if (consecutive_rejected_frames_ >= kTrackingLostRejectedFrames) {
    status_.set_tracking_status("TRACKING_LOST");
    status_.set_operator_hint(
        "Stay near the existing map and drive slowly; reset if it cannot "
        "recover.");
  } else {
    status_.set_tracking_status("SCAN_MATCH_REJECTED");
    status_.set_operator_hint(
        "Stay near the mapped area and slow down to help scan matching "
        "recover.");
  }
  FillMatrixRepeated(state_.offset_R_L_I,
                     status_.mutable_rotation_lidar_to_imu());
  FillVectorRepeated(state_.offset_T_L_I,
                     status_.mutable_translation_lidar_to_imu());
  const M3D r_inv = state_.offset_R_L_I.transpose();
  const V3D t_inv = -r_inv * state_.offset_T_L_I;
  FillMatrixRepeated(r_inv, status_.mutable_rotation_imu_to_lidar());
  FillVectorRepeated(t_inv, status_.mutable_translation_imu_to_lidar());
  status_.set_result_file(conf_.result_path());
}

void FastLio2RuntimeCore::WriteResultFile() {
  boost::filesystem::path result_path(conf_.result_path());
  if (!result_path.parent_path().empty()) {
    boost::filesystem::create_directories(result_path.parent_path());
  }
  std::ofstream fout(conf_.result_path(), std::ios::out);
  if (!fout) {
    Fail("failed to open result file: " + conf_.result_path());
    return;
  }
  fout.setf(std::ios::fixed);
  fout << std::setprecision(9);
  const M3D r_li = state_.offset_R_L_I;
  const V3D t_li = state_.offset_T_L_I;
  const M3D r_il = r_li.transpose();
  const V3D t_il = -r_il * t_li;

  fout
      << "Frame convention: point_in_imu = T_lidar_to_imu * point_in_lidar\n\n";
  fout << "LiDAR to IMU rotation matrix:\n" << r_li << "\n";
  fout << "LiDAR to IMU translation:\n" << t_li.transpose() << "\n\n";
  fout << "LiDAR to IMU homogeneous matrix:\n";
  fout << r_li(0, 0) << " " << r_li(0, 1) << " " << r_li(0, 2) << " " << t_li(0)
       << "\n";
  fout << r_li(1, 0) << " " << r_li(1, 1) << " " << r_li(1, 2) << " " << t_li(1)
       << "\n";
  fout << r_li(2, 0) << " " << r_li(2, 1) << " " << r_li(2, 2) << " " << t_li(2)
       << "\n";
  fout << "0 0 0 1\n\n";
  fout << "IMU to LiDAR rotation matrix:\n" << r_il << "\n";
  fout << "IMU to LiDAR translation:\n" << t_il.transpose() << "\n\n";
  fout << "Time Lag IMU to LiDAR (second): "
       << time_lag_imu_wrt_lidar_ + timediff_imu_wrt_lidar_ << "\n";
  fout << "Quality metrics:\n";
  fout << "  mean_point_to_plane_residual: " << mean_point_to_plane_residual_
       << "\n";
  fout << "  effective_feature_count: " << effect_feat_num_ << "\n";
  fout << "  extrinsic_delta_rotation_deg: " << extrinsic_delta_rotation_deg_
       << "\n";
  fout << "  extrinsic_delta_translation_m: " << extrinsic_delta_translation_m_
       << "\n";
  fout << "  gravity_norm: " << state_.gravity.norm() << "\n\n";
  fout << "Bias of Gyroscope (rad/s): " << state_.bias_g.transpose() << "\n";
  fout << "Bias of Accelerometer (m/s^2): " << state_.bias_a.transpose()
       << "\n";
  fout << "Gravity in World Frame (m/s^2): " << state_.gravity.transpose()
       << "\n\n";
  fout << "Apollo static transform helper (manual use only):\n";
  fout << "parent_frame_id: imu\n";
  fout << "child_frame_id: <lidar_frame>\n";
  fout << "translation: " << t_li.transpose() << "\n";
  Eigen::Quaterniond q_li(r_li);
  q_li.normalize();
  fout << "rotation quaternion xyzw: " << q_li.x() << " " << q_li.y() << " "
       << q_li.z() << " " << q_li.w() << "\n";
  AINFO << "FAST-LIO2 runtime result written to " << conf_.result_path();
}

void FastLio2RuntimeCore::Finish() {
  PublishProgressStatus();
  WriteResultFile();
  if (!failed_) {
    finished_ = true;
    SetStage(Stage::FINISHED, "runtime refinement finished");
    PublishProgressStatus();
  }
}

void FastLio2RuntimeCore::Fail(const std::string& message) {
  failed_ = true;
  SetStage(Stage::ERROR, message);
  AERROR << "[FastLio2] " << message;
}

const char* FastLio2RuntimeCore::StageName(Stage stage) {
  switch (stage) {
    case Stage::WAITING_FOR_DATA:
      return "WAITING_FOR_DATA";
    case Stage::LIDAR_ONLY_ODOMETRY:
      return "LIDAR_ONLY_ODOMETRY";
    case Stage::DATA_ACCUMULATION:
      return "DATA_ACCUMULATION";
    case Stage::INITIALIZING:
      return "INITIALIZING";
    case Stage::REFINING:
      return "REFINING";
    case Stage::FINISHED:
      return "FINISHED";
    case Stage::ERROR:
      return "ERROR";
  }
  return "UNKNOWN";
}

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
