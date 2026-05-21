#include "fast_lio2/core/official_pipeline_contract.h"

#include <sstream>

namespace whl {
namespace fast_lio2 {

std::vector<OfficialFastLio2StageSpec> OfficialFastLio2Pipeline() {
  return {
      {OfficialFastLio2Stage::kSyncPackages, "sync_packages",
       "sync_packages(MeasureGroup&)",
       "one LiDAR scan is processed only after IMU samples cover lidar_end_time"},
      {OfficialFastLio2Stage::kImuProcessAndUndistort,
       "imu_process_and_undistort", "ImuProcess::Process()",
       "IMU propagation and backward point undistortion happen before scan-to-map"},
      {OfficialFastLio2Stage::kLocalMapFovSegment, "local_map_fov_segment",
       "lasermap_fov_segment()",
       "ikd-tree local map cube is maintained around current lidar pose"},
      {OfficialFastLio2Stage::kVoxelDownsample, "voxel_downsample",
       "downSizeFilterSurf.filter()",
       "scan matching uses downsampled undistorted frame-local points"},
      {OfficialFastLio2Stage::kIkdTreeInitOrNearestSearch,
       "ikd_tree_init_or_nearest_search", "ikdtree.Build()/Nearest_Search()",
       "nearest surfaces are searched in map frame using p_map = T_map_imu * T_imu_lidar * p_lidar"},
      {OfficialFastLio2Stage::kIteratedKalmanUpdate,
       "iterated_kalman_update", "kf.update_iterated_dyn_share_modified()",
       "point-to-plane residuals update pose, velocity, biases, gravity, and optional extrinsic state"},
      {OfficialFastLio2Stage::kIncrementalMapUpdate, "incremental_map_update",
       "map_incremental()",
       "accepted downsampled points are transformed to map frame and inserted into ikd-tree"},
      {OfficialFastLio2Stage::kPublishOrExportOutputs,
       "publish_or_export_outputs", "publish_odometry()/publish_frame_world()",
       "published poses, registered clouds, optimized_frame_dataset, tiles, and localization maps must share the same map frame"},
  };
}

bool ValidateOfficialFastLio2StageOrder(
    const std::vector<OfficialFastLio2Stage>& observed, std::string* error) {
  const auto expected = OfficialFastLio2Pipeline();
  if (observed.size() != expected.size()) {
    if (error != nullptr) {
      std::ostringstream oss;
      oss << "stage count mismatch: observed " << observed.size()
          << ", expected " << expected.size();
      *error = oss.str();
    }
    return false;
  }
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (observed[i] != expected[i].stage) {
      if (error != nullptr) {
        std::ostringstream oss;
        oss << "stage " << i << " mismatch: observed "
            << OfficialFastLio2StageName(observed[i]) << ", expected "
            << OfficialFastLio2StageName(expected[i].stage);
        *error = oss.str();
      }
      return false;
    }
  }
  return true;
}

const char* OfficialFastLio2StageName(OfficialFastLio2Stage stage) {
  switch (stage) {
    case OfficialFastLio2Stage::kSyncPackages:
      return "sync_packages";
    case OfficialFastLio2Stage::kImuProcessAndUndistort:
      return "imu_process_and_undistort";
    case OfficialFastLio2Stage::kLocalMapFovSegment:
      return "local_map_fov_segment";
    case OfficialFastLio2Stage::kVoxelDownsample:
      return "voxel_downsample";
    case OfficialFastLio2Stage::kIkdTreeInitOrNearestSearch:
      return "ikd_tree_init_or_nearest_search";
    case OfficialFastLio2Stage::kIteratedKalmanUpdate:
      return "iterated_kalman_update";
    case OfficialFastLio2Stage::kIncrementalMapUpdate:
      return "incremental_map_update";
    case OfficialFastLio2Stage::kPublishOrExportOutputs:
      return "publish_or_export_outputs";
  }
  return "unknown";
}

}  // namespace fast_lio2
}  // namespace whl

