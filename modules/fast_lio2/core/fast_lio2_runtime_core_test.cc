#include "modules/fast_lio2/core/fast_lio2_runtime_core.h"

#include <cmath>
#include <iostream>
#include <memory>

namespace {
using apollo::localization::fast_lio2::FastLio2ImuSample;
using apollo::localization::fast_lio2::FastLio2PointCloudFrame;
using apollo::localization::fast_lio2::FastLio2RuntimeConf;
using apollo::localization::fast_lio2::FastLio2RuntimeCore;
using apollo::localization::fast_lio2::FastLio2RuntimeStateSnapshot;

const Eigen::Vector3d kBias(0.01, -0.005, 0.002);
const Eigen::Vector3d kTranslation(0.1, -0.03, 0.04);

FastLio2RuntimeConf Config(bool estimate) {
  FastLio2RuntimeConf conf;
  conf.set_initial_rotation_lidar_to_imu_qx(0.0);
  conf.set_initial_rotation_lidar_to_imu_qy(0.0);
  conf.set_initial_rotation_lidar_to_imu_qz(0.0);
  conf.set_initial_rotation_lidar_to_imu_qw(1.0);
  conf.set_initial_translation_lidar_to_imu_x(kTranslation.x());
  conf.set_initial_translation_lidar_to_imu_y(kTranslation.y());
  conf.set_initial_translation_lidar_to_imu_z(kTranslation.z());
  conf.set_extrinsic_est_en(estimate);
  conf.set_filter_size_surf(0.2);
  conf.set_filter_size_map(0.2);
  conf.set_det_range(20.0);
  conf.set_cube_side_length(200.0);
  conf.set_max_iteration(4);
  return conf;
}

FastLio2PointCloudFrame RoomScan(int frame, float offset = 0.0F) {
  const double start = 10.0 + frame * 0.1;
  auto cloud = std::make_shared<PointCloudXYZI>();
  for (int u = -8; u <= 8; ++u) {
    for (int v = -6; v <= 6; ++v) {
      for (int plane = 0; plane < 3; ++plane) {
        PointType point{};
        point.x = plane == 0 ? 4.0F : u * 0.3F;
        point.y = plane == 1 ? 3.0F : (plane == 0 ? u : v) * 0.3F;
        point.z = plane == 2 ? -1.2F : v * 0.2F;
        point.x += offset;
        point.y += offset;
        point.z += offset;
        point.intensity = 1.0F;
        cloud->push_back(point);
      }
    }
  }
  for (size_t i = 0; i < cloud->size(); ++i)
    cloud->points[i].curvature = 99.0F * i / (cloud->size() - 1);
  FastLio2PointCloudFrame scan;
  scan.points = cloud;
  scan.scan_start_sec = start;
  scan.scan_end_sec = start + 0.099;
  return scan;
}

void StationaryImus(FastLio2RuntimeCore* core, int frame) {
  const double start = 10.0 + frame * 0.1;
  for (int i = 1; i <= 10; ++i) {
    FastLio2ImuSample imu;
    imu.timestamp_sec = start + i * 0.01;
    imu.angular_velocity = kBias;
    imu.linear_acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
    core->AddImu(imu);
  }
}

void Feed(FastLio2RuntimeCore* core, int frame, float offset = 0.0F) {
  core->AddPointCloud(RoomScan(frame, offset));
  StationaryImus(core, frame);
  core->Process();
}

bool StationaryAndRejectedScan(bool estimate) {
  auto core = std::make_unique<FastLio2RuntimeCore>();
  if (!core->Init(Config(estimate))) return false;
  for (int frame = 0; frame < 8; ++frame) Feed(core.get(), frame);
  FastLio2RuntimeStateSnapshot state;
  if (!core->GetStateSnapshot(&state) || !state.scan_match_valid ||
      state.frame_count < 4 || state.effective_feature_count < 20 ||
      state.position.norm() > 0.02 ||
      (state.gyro_bias - kBias).norm() > 0.001 ||
      std::abs(state.gravity.norm() - 9.809) > 1e-8 ||
      !state.covariance.allFinite()) {
    std::cerr << "Stationary initialization/update failed, estimate="
              << estimate << " position=" << state.position.transpose()
              << " bias=" << state.gyro_bias.transpose()
              << " gravity=" << state.gravity.norm() << "\n";
    return false;
  }
  if (!estimate &&
      ((state.translation_lidar_to_imu - kTranslation).norm() > 1e-12 ||
       (state.rotation_lidar_to_imu - Eigen::Matrix3d::Identity()).norm() >
           1e-12)) {
    std::cerr
        << "Disabled extrinsic estimation changed the configured transform\n";
    return false;
  }
  const auto frames = state.frame_count;
  apollo::drivers::PointCloud body, world;
  if (!core->GetRegisteredBodyCloud(&body) ||
      !core->GetRegisteredCloud(&world) || body.frame_id() != "body" ||
      body.point_size() != world.point_size() || body.point_size() == 0) {
    std::cerr << "Registered body-frame cloud is unavailable\n";
    return false;
  }
  const auto& b = body.point(0);
  const auto& w = world.point(0);
  if ((state.rotation * Eigen::Vector3d(b.x(), b.y(), b.z()) + state.position -
       Eigen::Vector3d(w.x(), w.y(), w.z()))
          .norm() > 1e-5) {
    std::cerr << "Body cloud did not apply the LiDAR-to-IMU extrinsic\n";
    return false;
  }
  apollo::drivers::PointCloud before, after;
  core->GetMapCloud(&before);
  Feed(core.get(), 8, 100.0F);
  core->GetStateSnapshot(&state);
  core->GetMapCloud(&after);
  apollo::drivers::PointCloud rejected;
  if (state.scan_match_valid || state.tracking_status != "DEGRADED" ||
      state.reject_reason.empty() || state.frame_count != frames ||
      after.point_size() != before.point_size() ||
      core->GetRegisteredCloud(&rejected)) {
    std::cerr << "Unmatched scan was reported valid or inserted into the map\n";
    return false;
  }
  Feed(core.get(), 9);
  core->GetStateSnapshot(&state);
  if (!state.scan_match_valid || state.frame_count != frames + 1) {
    std::cerr << "Tracking did not resume after an unmatched scan\n";
    return false;
  }
  return true;
}

bool BufferedScansAndMissingImu() {
  auto core = std::make_unique<FastLio2RuntimeCore>();
  if (!core->Init(Config(false))) return false;
  for (int frame = 0; frame < 8; ++frame) Feed(core.get(), frame);
  FastLio2RuntimeStateSnapshot state;
  core->GetStateSnapshot(&state);
  const auto frames = state.frame_count;
  core->AddPointCloud(RoomScan(8));
  core->AddPointCloud(RoomScan(9));
  StationaryImus(core.get(), 8);
  StationaryImus(core.get(), 9);
  for (int i = 0; i < 2; ++i) {
    if (!core->Process() || !core->GetStateSnapshot(&state) ||
        !state.scan_match_valid || state.frame_count != frames + i + 1 ||
        std::abs(state.timestamp_sec - (10.899 + i * 0.1)) > 1e-8) {
      std::cerr
          << "Buffered scan was skipped instead of exposed individually\n";
      return false;
    }
  }
  if (core->Process()) {
    std::cerr << "No new scan was incorrectly reported as processed\n";
    return false;
  }

  core->AddPointCloud(RoomScan(10));
  core->AddPointCloud(RoomScan(11));
  StationaryImus(core.get(), 12);
  if (!core->Process() || !core->GetStateSnapshot(&state) ||
      !state.scan_match_valid) {
    std::cerr << "Scan before the empty-IMU package failed\n";
    return false;
  }
  const auto frames_before_gap = state.frame_count;
  apollo::drivers::PointCloud before, after, rejected;
  core->GetMapCloud(&before);
  if (!core->Process() || !core->GetStateSnapshot(&state)) return false;
  core->GetMapCloud(&after);
  if (state.scan_match_valid || state.tracking_status != "DEGRADED" ||
      state.stage != "DEGRADED" || state.reject_reason.empty() ||
      state.frame_count != frames_before_gap ||
      state.effective_feature_count != 0 ||
      state.mean_point_to_plane_residual != 0.0 ||
      after.point_size() != before.point_size() ||
      core->GetRegisteredCloud(&rejected)) {
    std::cerr << "Empty IMU package retained stale valid pose/cloud/metrics\n";
    return false;
  }
  core->AddPointCloud(RoomScan(12));
  if (!core->Process() || !core->GetStateSnapshot(&state) ||
      !state.scan_match_valid || state.frame_count != frames_before_gap + 1) {
    std::cerr << "Tracking did not resume after an empty IMU package\n";
    return false;
  }
  return true;
}

bool FivePointScan() {
  auto core = std::make_unique<FastLio2RuntimeCore>();
  if (!core->Init(Config(false))) return false;
  for (int frame = 0; frame < 8; ++frame) Feed(core.get(), frame);
  auto scan = RoomScan(8);
  scan.points->resize(5);
  for (int i = 0; i < 5; ++i) scan.points->points[i].curvature = 99.0F * i / 4;
  core->AddPointCloud(scan);
  StationaryImus(core.get(), 8);
  core->Process();
  FastLio2RuntimeStateSnapshot state;
  if (!core->GetStateSnapshot(&state) || !state.scan_match_valid ||
      state.effective_feature_count != 5) {
    std::cerr << "Five-point scan was rejected despite an initialized map\n";
    return false;
  }
  return true;
}

bool PlaneScoreRejectsNonFinite() {
  PointType world{};
  world.x = 4.0F;
  const Eigen::Vector4f plane(-1.0F, 0.0F, 0.0F, 4.0F);
  float distance;
  if (SelectPlaneCorrespondence(world, V3D::Zero(), plane, &distance) ||
      !SelectPlaneCorrespondence(world, V3D(4.0, 0.0, 0.0), plane, &distance)) {
    std::cerr << "Plane score accepted NaN or rejected a finite exact match\n";
    return false;
  }
  world.x = 5.0F;
  if (SelectPlaneCorrespondence(world, V3D(4.0, 0.0, 0.0), plane, &distance)) {
    std::cerr << "Plane score accepted a large residual\n";
    return false;
  }
  return true;
}

bool NearestSearchRadius() {
  auto tree = std::make_unique<KD_TREE<PointType>>();
  PointVector points;
  PointType point{};
  point.x = 1.4F;
  points.push_back(point);
  point.x = 1.6F;
  points.push_back(point);
  tree->Build(points);
  PointVector nearest;
  std::vector<float> distances;
  tree->Nearest_Search(PointType{}, 2, nearest, distances, 1.5F);
  if (nearest.size() != 1 || std::abs(nearest[0].x - 1.4F) > 1e-6F) {
    std::cerr
        << "ikd-tree finite-radius nearest search differs from upstream\n";
    return false;
  }
  return true;
}

bool MovingRoom() {
  auto core = std::make_unique<FastLio2RuntimeCore>();
  if (!core->Init(Config(false))) return false;
  for (int frame = 0; frame < 30; ++frame) {
    const double start = 20.0 + frame * 0.1;
    auto cloud = std::make_shared<PointCloudXYZI>();
    for (int u = -8; u <= 8; ++u) {
      for (int v = -6; v <= 6; ++v) {
        for (int plane = 0; plane < 3; ++plane) {
          PointType point{};
          point.x = plane == 0 ? 4.0F : u * 0.3F;
          point.y = plane == 1 ? 3.0F : (plane == 0 ? u : v) * 0.3F;
          point.z = plane == 2 ? -1.2F : v * 0.2F;
          cloud->push_back(point);
        }
      }
    }
    for (size_t i = 0; i < cloud->size(); ++i) {
      auto& point = cloud->points[i];
      point.curvature = 99.0F * i / (cloud->size() - 1);
      const double elapsed =
          std::max(0.0, start + point.curvature * 1e-3 - 20.4);
      const Eigen::Matrix3d rotation =
          Eigen::AngleAxisd(0.1 * elapsed * elapsed, Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
      const Eigen::Vector3d position(0.05 * elapsed * elapsed, 0.0, 0.0);
      const Eigen::Vector3d world =
          Eigen::Vector3d(point.x, point.y, point.z) + kTranslation;
      const Eigen::Vector3d lidar =
          rotation.transpose() * (world - position) - kTranslation;
      point.x = lidar.x();
      point.y = lidar.y();
      point.z = lidar.z();
    }
    FastLio2PointCloudFrame scan;
    scan.points = cloud;
    scan.scan_start_sec = start;
    scan.scan_end_sec = start + 0.099;
    core->AddPointCloud(scan);
    for (int i = 1; i <= 10; ++i) {
      FastLio2ImuSample imu;
      imu.timestamp_sec = start + i * 0.01;
      const double elapsed = std::max(0.0, imu.timestamp_sec - 20.4);
      const Eigen::Matrix3d rotation =
          Eigen::AngleAxisd(0.1 * elapsed * elapsed, Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
      imu.angular_velocity = kBias + Eigen::Vector3d(0.0, 0.0, 0.2 * elapsed);
      imu.linear_acceleration =
          rotation.transpose() *
          Eigen::Vector3d(elapsed > 0.0 ? 0.1 : 0.0, 0.0, 9.81);
      core->AddImu(imu);
    }
    core->Process();
  }
  FastLio2RuntimeStateSnapshot state;
  core->GetStateSnapshot(&state);
  const double elapsed = state.timestamp_sec - 20.4;
  const Eigen::Vector3d expected(0.05 * elapsed * elapsed, 0.0, 0.0);
  const Eigen::Matrix3d expected_rotation =
      Eigen::AngleAxisd(0.1 * elapsed * elapsed, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  const double position_error = (state.position - expected).norm();
  const double rotation_error =
      Eigen::AngleAxisd(expected_rotation.transpose() * state.rotation).angle();
  if (!state.scan_match_valid || position_error > 0.02 ||
      rotation_error > 0.01 ||
      (state.translation_lidar_to_imu - kTranslation).norm() > 1e-12 ||
      std::abs(state.gravity.norm() - 9.809) > 1e-8) {
    std::cerr << "Dynamic propagation/deskew/update failed: position error="
              << position_error << " rotation error=" << rotation_error << "\n";
    return false;
  }
  std::cout << "Synthetic moving-room position error=" << position_error
            << " m, rotation error=" << rotation_error << " rad\n";
  return true;
}
}  // namespace

int main() {
  auto missing_extrinsic = std::make_unique<FastLio2RuntimeCore>();
  if (missing_extrinsic->Init(FastLio2RuntimeConf{})) {
    std::cerr << "Missing extrinsic silently fell back to identity\n";
    return 1;
  }
  if (!StationaryAndRejectedScan(false) || !StationaryAndRejectedScan(true) ||
      !BufferedScansAndMissingImu() || !FivePointScan() ||
      !PlaneScoreRejectsNonFinite() || !NearestSearchRadius() || !MovingRoom())
    return 1;
  return 0;
}
