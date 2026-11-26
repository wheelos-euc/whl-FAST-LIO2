#include "modules/fast_lio2/core/fast_lio2_runtime_core.h"

// Measurement and mapping path adapted from FAST_LIO@7cc4175.
// See upstream_sources.txt for licensing and intentional boundary differences.

#include <algorithm>
#include <cmath>

#include "cyber/common/log.h"
#include "modules/fast_lio2/adapter/corrected_imu_adapter.h"
#include "modules/fast_lio2/core/imu_processing.h"

namespace apollo {
namespace localization {
namespace fast_lio2 {
namespace {
constexpr float kMoveThreshold = 1.5F;
constexpr double kLaserPointCovariance = 0.001;

float SquaredDistance(const PointType& a, const PointType& b) {
  return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
         (a.z - b.z) * (a.z - b.z);
}
}  // namespace

struct FastLio2RuntimeCore::Estimator {
  OfficialFastLioFilter filter;
  state_ikfom state;
};

FastLio2RuntimeCore::FastLio2RuntimeCore() = default;
FastLio2RuntimeCore::~FastLio2RuntimeCore() = default;

bool FastLio2RuntimeCore::Init(const FastLio2RuntimeConf& conf) {
  conf_ = conf;
  if (conf.max_iteration() < 1 || conf.filter_size_surf() <= 0.0 ||
      conf.filter_size_map() <= 0.0 || conf.cube_side_length() <= 0.0 ||
      conf.det_range() <= 0.0) {
    SetStage(Stage::ERROR, "invalid filter or local-map configuration");
    failed_ = true;
    return false;
  }
  estimator_ = std::make_unique<Estimator>();
  imu_process_ = std::make_unique<ImuProcess>();
  feats_undistort_.reset(new PointCloudXYZI);
  feats_down_body_.reset(new PointCloudXYZI);
  feats_down_world_.reset(new PointCloudXYZI);
  corr_normvect_.reset(new PointCloudXYZI);
  laser_cloud_ori_.reset(new PointCloudXYZI);
  down_size_filter_surf_.setLeafSize(conf.filter_size_surf(),
                                     conf.filter_size_surf(),
                                     conf.filter_size_surf());
  if (!ConfigureStaticExtrinsic()) return false;
  imu_process_->set_gyr_cov(V3D::Constant(conf.gyr_cov()));
  imu_process_->set_acc_cov(V3D::Constant(conf.acc_cov()));
  imu_process_->set_gyr_bias_cov(V3D::Constant(conf.b_gyr_cov()));
  imu_process_->set_acc_bias_cov(V3D::Constant(conf.b_acc_cov()));
  double limits[23];
  std::fill(std::begin(limits), std::end(limits), 0.001);
  estimator_->filter.init_dyn_share(
      get_f, df_dx, df_dw,
      [this](state_ikfom& state, esekfom::dyn_share_datastruct<double>& data) {
        BuildMeasurementModel(state, data);
      },
      conf.max_iteration(), limits);
  SetStage(Stage::WAITING_FOR_DATA, "waiting for point cloud and imu");
  return true;
}

bool FastLio2RuntimeCore::ConfigureStaticExtrinsic() {
  const bool complete = conf_.has_initial_rotation_lidar_to_imu_qx() &&
                        conf_.has_initial_rotation_lidar_to_imu_qy() &&
                        conf_.has_initial_rotation_lidar_to_imu_qz() &&
                        conf_.has_initial_rotation_lidar_to_imu_qw() &&
                        conf_.has_initial_translation_lidar_to_imu_x() &&
                        conf_.has_initial_translation_lidar_to_imu_y() &&
                        conf_.has_initial_translation_lidar_to_imu_z();
  if (!complete) {
    SetStage(Stage::ERROR, "explicit complete lidar-to-imu extrinsic required");
    failed_ = true;
    return false;
  }
  Eigen::Quaterniond rotation(conf_.initial_rotation_lidar_to_imu_qw(),
                              conf_.initial_rotation_lidar_to_imu_qx(),
                              conf_.initial_rotation_lidar_to_imu_qy(),
                              conf_.initial_rotation_lidar_to_imu_qz());
  const V3D translation(conf_.initial_translation_lidar_to_imu_x(),
                        conf_.initial_translation_lidar_to_imu_y(),
                        conf_.initial_translation_lidar_to_imu_z());
  if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-8 ||
      !translation.allFinite()) {
    SetStage(Stage::ERROR, "invalid lidar-to-imu extrinsic");
    failed_ = true;
    return false;
  }
  rotation.normalize();
  estimator_->state.offset_R_L_I = rotation.toRotationMatrix();
  estimator_->state.offset_T_L_I = translation;
  estimator_->filter.change_x(estimator_->state);
  imu_process_->set_extrinsic(translation, rotation.toRotationMatrix());
  AINFO << "Official FAST-LIO2 IKFoM configured. extrinsic_est_en="
        << conf_.extrinsic_est_en() << " t_li=" << translation.transpose();
  return true;
}

void FastLio2RuntimeCore::AddPointCloud(const FastLio2PointCloudFrame& cloud) {
  if (!cloud.points || cloud.points->size() <= 1) return;
  if (cloud.scan_start_sec < last_timestamp_lidar_) {
    lidar_buffer_.clear();
    time_buffer_.clear();
    end_time_buffer_.clear();
  }
  last_timestamp_lidar_ = cloud.scan_start_sec;
  lidar_buffer_.push_back(cloud.points);
  time_buffer_.push_back(cloud.scan_start_sec);
  end_time_buffer_.push_back(cloud.scan_end_sec);
}

void FastLio2RuntimeCore::AddImu(const FastLio2ImuSample& imu) {
  if (imu.timestamp_sec < last_timestamp_imu_) imu_buffer_.clear();
  last_timestamp_imu_ = imu.timestamp_sec;
  imu_buffer_.push_back(ToCompatImu(imu));
}

bool FastLio2RuntimeCore::Process() {
  if (!estimator_ || failed_) return false;
  MeasureGroup meas;
  if (!SyncPackages(&meas)) return false;
  ProcessMeasure(meas);
  return true;
}

bool FastLio2RuntimeCore::SyncPackages(MeasureGroup* meas) {
  if (lidar_buffer_.empty() || imu_buffer_.empty()) return false;
  const double end = end_time_buffer_.front();
  if (last_timestamp_imu_ < end) return false;
  meas->lidar = lidar_buffer_.front();
  meas->lidar_beg_time = time_buffer_.front();
  meas->lidar_end_time = end;
  meas->imu.clear();
  while (!imu_buffer_.empty() &&
         imu_buffer_.front()->header.stamp.toSec() <= end) {
    meas->imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }
  lidar_buffer_.pop_front();
  time_buffer_.pop_front();
  end_time_buffer_.pop_front();
  return true;
}

void FastLio2RuntimeCore::ProcessMeasure(const MeasureGroup& meas) {
  lidar_beg_time_ = meas.lidar_beg_time;
  lidar_end_time_ = meas.lidar_end_time;
  last_scan_match_valid_ = false;
  effect_feat_num_ = 0;
  mean_point_to_plane_residual_ = 0.0;
  scan_match_inlier_ratio_ = 0.0;
  scan_match_delta_rotation_deg_ = 0.0;
  scan_match_delta_translation_m_ = 0.0;
  if (first_scan_) {
    first_lidar_time_ = lidar_beg_time_;
    first_scan_ = false;
    return;
  }
  if (!imu_process_->Process(meas, estimator_->filter, feats_undistort_)) {
    if (meas.imu.empty()) {
      SetStage(Stage::DEGRADED, "no IMU samples for synchronized scan");
    } else {
      SetStage(Stage::IMU_INITIALIZING,
               "collecting IMU initialization samples");
    }
    PublishProgressStatus();
    return;
  }
  estimator_->state = estimator_->filter.get_x();
  LasermapFovSegment();
  down_size_filter_surf_.setInputCloud(feats_undistort_);
  down_size_filter_surf_.filter(*feats_down_body_);
  feats_down_world_->resize(feats_down_body_->size());
  if (ikdtree_.Root_Node == nullptr) {
    if (feats_down_body_->size() <= 5) {
      SetStage(Stage::MAP_INITIALIZING, "too few points to seed local map");
      PublishProgressStatus();
      return;
    }
    ikdtree_.set_downsample_param(conf_.filter_size_map());
    for (size_t i = 0; i < feats_down_body_->size(); ++i)
      PointBodyToWorld(&feats_down_body_->points[i],
                       &feats_down_world_->points[i]);
    ikdtree_.Build(feats_down_world_->points);
    SetStage(Stage::MAP_INITIALIZING, "initial local map seeded");
    PublishProgressStatus();
    return;
  }
  if (feats_down_body_->size() < 5) {
    SetStage(Stage::DEGRADED, "too few downsampled points");
    PublishProgressStatus();
    return;
  }
  nearest_points_.resize(feats_down_body_->size());
  point_selected_surf_.resize(feats_down_body_->size(), true);
  normals_.resize(feats_down_body_->size());
  auto predicted = estimator_->state;
  auto predicted_covariance = estimator_->filter.get_P();
  double solve_time = 0.0;
  estimator_->filter.update_iterated_dyn_share_modified(kLaserPointCovariance,
                                                        solve_time);
  estimator_->state = estimator_->filter.get_x();
  const auto& state = estimator_->state;
  const V3D gravity(state.grav[0], state.grav[1], state.grav[2]);
  last_scan_match_valid_ =
      effect_feat_num_ > 0 && state.pos.allFinite() &&
      state.rot.coeffs().allFinite() && state.vel.allFinite() &&
      state.bg.allFinite() && state.ba.allFinite() && gravity.allFinite() &&
      state.offset_R_L_I.coeffs().allFinite() &&
      state.offset_T_L_I.allFinite() && estimator_->filter.get_P().allFinite();
  if (!last_scan_match_valid_) {
    estimator_->filter.change_x(predicted);
    estimator_->filter.change_P(predicted_covariance);
    estimator_->state = predicted;
    scan_match_delta_rotation_deg_ = 0.0;
    scan_match_delta_translation_m_ = 0.0;
    SetStage(Stage::DEGRADED, "no effective or finite scan-to-map update");
    PublishProgressStatus();
    return;
  }
  scan_match_delta_rotation_deg_ =
      Eigen::AngleAxisd(predicted.rot.toRotationMatrix().transpose() *
                        state.rot.toRotationMatrix())
          .angle() *
      180.0 / M_PI;
  scan_match_delta_translation_m_ = (state.pos - predicted.pos).norm();
  MapIncremental();
  ++frame_num_;
  SetStage(Stage::RUNNING, "official IKFoM scan-to-map update complete");
  PublishProgressStatus();
}

void FastLio2RuntimeCore::BuildMeasurementModel(
    state_ikfom& state, esekfom::dyn_share_datastruct<double>& data) {
  laser_cloud_ori_->clear();
  corr_normvect_->clear();
  double residual_sum = 0.0;
  for (size_t i = 0; i < feats_down_body_->size(); ++i) {
    const auto& body = feats_down_body_->points[i];
    auto& world = feats_down_world_->points[i];
    const V3D p(body.x, body.y, body.z);
    const V3D transformed =
        state.rot * (state.offset_R_L_I * p + state.offset_T_L_I) + state.pos;
    world.x = transformed.x();
    world.y = transformed.y();
    world.z = transformed.z();
    world.intensity = body.intensity;
    auto& near = nearest_points_[i];
    if (data.converge) {
      std::vector<float> distances(NUM_MATCH_POINTS);
      ikdtree_.Nearest_Search(world, NUM_MATCH_POINTS, near, distances);
      point_selected_surf_[i] = near.size() >= NUM_MATCH_POINTS &&
                                distances[NUM_MATCH_POINTS - 1] <= 5.0F;
    }
    if (!point_selected_surf_[i]) continue;
    point_selected_surf_[i] = false;
    Eigen::Vector4f plane;
    if (!esti_plane(plane, near, 0.1F)) continue;
    float distance;
    if (!SelectPlaneCorrespondence(world, p, plane, &distance)) continue;
    point_selected_surf_[i] = true;
    auto& normal = normals_[i];
    normal.x = plane.x();
    normal.y = plane.y();
    normal.z = plane.z();
    normal.intensity = distance;
    laser_cloud_ori_->push_back(body);
    corr_normvect_->push_back(normal);
    residual_sum += std::abs(distance);
  }
  effect_feat_num_ = static_cast<int>(laser_cloud_ori_->size());
  scan_match_inlier_ratio_ =
      static_cast<double>(effect_feat_num_) / feats_down_body_->size();
  mean_point_to_plane_residual_ =
      effect_feat_num_ > 0 ? residual_sum / effect_feat_num_ : 0.0;
  if (effect_feat_num_ == 0) {
    data.valid = false;
    return;
  }
  data.h_x = Eigen::MatrixXd::Zero(effect_feat_num_, 12);
  data.h.resize(effect_feat_num_);
  for (int i = 0; i < effect_feat_num_; ++i) {
    const auto& point = laser_cloud_ori_->points[i];
    const V3D p(point.x, point.y, point.z);
    const V3D in_imu = state.offset_R_L_I * p + state.offset_T_L_I;
    M3D cross_imu, cross_lidar;
    cross_imu << SKEW_SYM_MATRX(in_imu);
    cross_lidar << SKEW_SYM_MATRX(p);
    const auto& normal = corr_normvect_->points[i];
    const V3D n(normal.x, normal.y, normal.z);
    const V3D c = state.rot.conjugate() * n;
    const V3D a = cross_imu * c;
    data.h_x.block<1, 6>(i, 0) << n.transpose(), a.transpose();
    if (conf_.extrinsic_est_en()) {
      const V3D b = cross_lidar * state.offset_R_L_I.conjugate() * c;
      data.h_x.block<1, 6>(i, 6) << b.transpose(), c.transpose();
    }
    data.h(i) = -normal.intensity;
  }
}

void FastLio2RuntimeCore::LasermapFovSegment() {
  cub_needrm_.clear();
  const auto& state = estimator_->state;
  const V3D pos = state.pos + state.rot * state.offset_T_L_I;
  const double cube = conf_.cube_side_length();
  const float range = conf_.det_range();
  if (!localmap_initialized_) {
    for (int i = 0; i < 3; ++i) {
      local_map_points_.vertex_min[i] = pos[i] - cube / 2.0;
      local_map_points_.vertex_max[i] = pos[i] + cube / 2.0;
    }
    localmap_initialized_ = true;
    return;
  }
  float edges[3][2];
  bool move = false;
  for (int i = 0; i < 3; ++i) {
    edges[i][0] = std::abs(pos[i] - local_map_points_.vertex_min[i]);
    edges[i][1] = std::abs(pos[i] - local_map_points_.vertex_max[i]);
    move = move || edges[i][0] <= kMoveThreshold * range ||
           edges[i][1] <= kMoveThreshold * range;
  }
  if (!move) return;
  auto next = local_map_points_;
  const float distance =
      std::max((cube - 2.0 * kMoveThreshold * range) * 0.5 * 0.9,
               double(range * (kMoveThreshold - 1)));
  for (int i = 0; i < 3; ++i) {
    auto box = local_map_points_;
    if (edges[i][0] <= kMoveThreshold * range) {
      next.vertex_min[i] -= distance;
      next.vertex_max[i] -= distance;
      box.vertex_min[i] = local_map_points_.vertex_max[i] - distance;
      cub_needrm_.push_back(box);
    } else if (edges[i][1] <= kMoveThreshold * range) {
      next.vertex_min[i] += distance;
      next.vertex_max[i] += distance;
      box.vertex_max[i] = local_map_points_.vertex_min[i] + distance;
      cub_needrm_.push_back(box);
    }
  }
  local_map_points_ = next;
  PointVector removed;
  ikdtree_.acquire_removed_points(removed);
  ikdtree_.Delete_Point_Boxes(cub_needrm_);
}

void FastLio2RuntimeCore::MapIncremental() {
  PointVector to_add, no_downsample;
  const double voxel = conf_.filter_size_map();
  for (size_t i = 0; i < feats_down_body_->size(); ++i) {
    PointBodyToWorld(&feats_down_body_->points[i],
                     &feats_down_world_->points[i]);
    const auto& world = feats_down_world_->points[i];
    const auto& near = nearest_points_[i];
    if (near.empty() || lidar_beg_time_ - first_lidar_time_ < 0.1) {
      to_add.push_back(world);
      continue;
    }
    PointType middle;
    middle.x = std::floor(world.x / voxel) * voxel + 0.5 * voxel;
    middle.y = std::floor(world.y / voxel) * voxel + 0.5 * voxel;
    middle.z = std::floor(world.z / voxel) * voxel + 0.5 * voxel;
    if (std::abs(near[0].x - middle.x) > 0.5 * voxel &&
        std::abs(near[0].y - middle.y) > 0.5 * voxel &&
        std::abs(near[0].z - middle.z) > 0.5 * voxel) {
      no_downsample.push_back(world);
      continue;
    }
    bool add = true;
    if (near.size() >= NUM_MATCH_POINTS) {
      for (int j = 0; j < NUM_MATCH_POINTS; ++j) {
        if (SquaredDistance(near[j], middle) < SquaredDistance(world, middle)) {
          add = false;
          break;
        }
      }
    }
    if (add) to_add.push_back(world);
  }
  ikdtree_.Add_Points(to_add, true);
  ikdtree_.Add_Points(no_downsample, false);
}

void FastLio2RuntimeCore::PointBodyToWorld(const PointType* in,
                                           PointType* out) const {
  const auto& state = estimator_->state;
  const V3D p(in->x, in->y, in->z);
  const V3D world =
      state.rot * (state.offset_R_L_I * p + state.offset_T_L_I) + state.pos;
  *out = *in;
  out->x = world.x();
  out->y = world.y();
  out->z = world.z();
}

void FastLio2RuntimeCore::PointCloudToApollo(
    const PointCloudXYZI& cloud, const std::string& frame,
    apollo::drivers::PointCloud* out) const {
  out->Clear();
  out->mutable_header()->set_timestamp_sec(lidar_end_time_);
  out->mutable_header()->set_frame_id(frame);
  out->set_frame_id(frame);
  out->set_measurement_time(lidar_end_time_);
  out->set_is_dense(true);
  out->set_width(cloud.size());
  out->set_height(1);
  for (const auto& source : cloud) {
    auto* point = out->add_point();
    point->set_x(source.x);
    point->set_y(source.y);
    point->set_z(source.z);
    point->set_intensity(
        static_cast<uint32_t>(std::max(0.0F, source.intensity)));
    point->set_timestamp(static_cast<uint64_t>(
        (lidar_beg_time_ + source.curvature * 1e-3) * 1e9));
  }
}

bool FastLio2RuntimeCore::GetRegisteredCloud(
    apollo::drivers::PointCloud* out) const {
  if (!conf_.publish_visualization() || !last_scan_match_valid_) return false;
  PointCloudXYZI world;
  world.resize(feats_undistort_->size());
  for (size_t i = 0; i < world.size(); ++i)
    PointBodyToWorld(&feats_undistort_->points[i], &world.points[i]);
  PointCloudToApollo(world, conf_.map_frame(), out);
  return true;
}

bool FastLio2RuntimeCore::GetRegisteredBodyCloud(
    apollo::drivers::PointCloud* out) const {
  if (!conf_.publish_visualization() || !last_scan_match_valid_) return false;
  const auto& state = estimator_->state;
  PointCloudXYZI body = *feats_undistort_;
  for (auto& point : body) {
    const V3D transformed =
        state.offset_R_L_I * V3D(point.x, point.y, point.z) +
        state.offset_T_L_I;
    point.x = transformed.x();
    point.y = transformed.y();
    point.z = transformed.z();
  }
  PointCloudToApollo(body, "body", out);
  return true;
}

bool FastLio2RuntimeCore::GetMapCloud(apollo::drivers::PointCloud* out) {
  if (!conf_.publish_visualization() || !ikdtree_.Root_Node) return false;
  PointVector points;
  ikdtree_.flatten(ikdtree_.Root_Node, points, NOT_RECORD);
  PointCloudXYZI cloud;
  cloud.points.assign(points.begin(), points.end());
  PointCloudToApollo(cloud, conf_.map_frame(), out);
  return true;
}

bool FastLio2RuntimeCore::GetStateSnapshot(FastLio2RuntimeStateSnapshot* out) {
  if (!out || !estimator_ || !ikdtree_.Root_Node) return false;
  const auto& state = estimator_->state;
  out->timestamp_sec = lidar_end_time_;
  out->rotation = state.rot.toRotationMatrix();
  out->position = state.pos;
  out->velocity = state.vel;
  out->gyro_bias = state.bg;
  for (int i = 0; i < 3; ++i) out->gravity[i] = state.grav[i];
  out->rotation_lidar_to_imu = state.offset_R_L_I.toRotationMatrix();
  out->translation_lidar_to_imu = state.offset_T_L_I;
  out->covariance = estimator_->filter.get_P();
  out->scan_match_valid = last_scan_match_valid_;
  out->stage = StageName(stage_);
  out->tracking_status = last_scan_match_valid_ ? "OK" : "DEGRADED";
  out->reject_reason = last_scan_match_valid_ ? "" : status_.message();
  out->frame_count = frame_num_;
  out->effective_feature_count = effect_feat_num_;
  out->map_point_count = ikdtree_.validnum();
  out->mean_point_to_plane_residual = mean_point_to_plane_residual_;
  out->scan_match_inlier_ratio = scan_match_inlier_ratio_;
  out->scan_match_delta_rotation_deg = scan_match_delta_rotation_deg_;
  out->scan_match_delta_translation_m = scan_match_delta_translation_m_;
  return true;
}

void FastLio2RuntimeCore::SetStage(Stage stage, const std::string& message) {
  stage_ = stage;
  status_.set_stage(StageName(stage));
  status_.set_message(message);
  AINFO << "[FastLio2] stage=" << StageName(stage) << " message=" << message;
}

void FastLio2RuntimeCore::PublishProgressStatus() {
  const auto& state = estimator_->state;
  status_.set_data_sufficient(last_scan_match_valid_);
  status_.set_scan_match_valid(last_scan_match_valid_);
  status_.set_tracking_status(last_scan_match_valid_ ? "OK" : "DEGRADED");
  status_.set_scan_match_reject_reason(
      last_scan_match_valid_ ? "" : status_.message());
  status_.set_mean_point_to_plane_residual(mean_point_to_plane_residual_);
  status_.set_effective_feature_count(effect_feat_num_);
  V3D gravity;
  for (int i = 0; i < 3; ++i) gravity[i] = state.grav[i];
  status_.set_gravity_norm(gravity.norm());
  status_.set_scan_match_inlier_ratio(scan_match_inlier_ratio_);
  status_.set_scan_match_delta_rotation_deg(scan_match_delta_rotation_deg_);
  status_.set_scan_match_delta_translation_m(scan_match_delta_translation_m_);
}

const char* FastLio2RuntimeCore::StageName(Stage stage) {
  switch (stage) {
    case Stage::WAITING_FOR_DATA:
      return "WAITING_FOR_DATA";
    case Stage::IMU_INITIALIZING:
      return "IMU_INITIALIZING";
    case Stage::MAP_INITIALIZING:
      return "MAP_INITIALIZING";
    case Stage::RUNNING:
      return "RUNNING";
    case Stage::DEGRADED:
      return "DEGRADED";
    case Stage::ERROR:
      return "ERROR";
  }
  return "ERROR";
}

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
