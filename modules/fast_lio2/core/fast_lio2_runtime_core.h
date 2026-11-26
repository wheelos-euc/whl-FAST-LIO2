#pragma once

#include <pcl/filters/voxel_grid.h>

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "modules/fast_lio2/adapter/runtime_data.h"
#include "modules/fast_lio2/core/include/ikd-Tree/ikd_Tree.h"
#include "modules/fast_lio2/proto/fast_lio2_runtime_conf.pb.h"
#include "modules/fast_lio2/proto/fast_lio2_runtime_status.pb.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

struct state_ikfom;
namespace esekfom {
template <typename T>
struct dyn_share_datastruct;
}
class ImuProcess;

namespace apollo {
namespace localization {
namespace fast_lio2 {

struct FastLio2RuntimeStateSnapshot {
  double timestamp_sec = 0.0;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation_lidar_to_imu = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation_lidar_to_imu = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 23, 23> covariance =
      Eigen::Matrix<double, 23, 23>::Identity();
  bool scan_match_valid = false;
  std::string stage;
  std::string tracking_status;
  std::string reject_reason;
  uint64_t frame_count = 0;
  int effective_feature_count = 0;
  int map_point_count = 0;
  double mean_point_to_plane_residual = 0.0;
  double scan_match_inlier_ratio = 0.0;
  double scan_match_delta_rotation_deg = 0.0;
  double scan_match_delta_translation_m = 0.0;
};

class FastLio2RuntimeCore {
 public:
  FastLio2RuntimeCore();
  ~FastLio2RuntimeCore();
  bool Init(const FastLio2RuntimeConf& conf);
  void AddPointCloud(const FastLio2PointCloudFrame& cloud);
  void AddImu(const FastLio2ImuSample& imu);
  // Consume one synchronized scan; false means no package is ready.
  bool Process();
  bool Finished() const { return false; }
  bool Failed() const { return failed_; }
  const FastLio2RuntimeStatus& Status() const { return status_; }
  bool GetRegisteredCloud(apollo::drivers::PointCloud* out) const;
  bool GetRegisteredBodyCloud(apollo::drivers::PointCloud* out) const;
  bool GetMapCloud(apollo::drivers::PointCloud* out);
  bool GetStateSnapshot(FastLio2RuntimeStateSnapshot* out);

 private:
  struct Estimator;
  enum class Stage {
    WAITING_FOR_DATA,
    IMU_INITIALIZING,
    MAP_INITIALIZING,
    RUNNING,
    DEGRADED,
    ERROR
  };
  bool SyncPackages(MeasureGroup* meas);
  void ProcessMeasure(const MeasureGroup& meas);
  void BuildMeasurementModel(state_ikfom& state,
                             esekfom::dyn_share_datastruct<double>& data);
  void SetStage(Stage stage, const std::string& message);
  void PublishProgressStatus();
  void LasermapFovSegment();
  void MapIncremental();
  void PointBodyToWorld(const PointType* pi, PointType* po) const;
  void PointCloudToApollo(const PointCloudXYZI& cloud,
                          const std::string& frame_id,
                          apollo::drivers::PointCloud* out) const;
  bool ConfigureStaticExtrinsic();
  static const char* StageName(Stage stage);

  FastLio2RuntimeConf conf_;
  FastLio2RuntimeStatus status_;
  Stage stage_ = Stage::WAITING_FOR_DATA;
  std::unique_ptr<Estimator> estimator_;
  std::unique_ptr<ImuProcess> imu_process_;
  std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
  std::deque<double> time_buffer_;
  std::deque<double> end_time_buffer_;
  std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;
  bool first_scan_ = true;
  PointCloudXYZI::Ptr feats_undistort_;
  PointCloudXYZI::Ptr feats_down_body_;
  PointCloudXYZI::Ptr feats_down_world_;
  PointCloudXYZI::Ptr corr_normvect_;
  PointCloudXYZI::Ptr laser_cloud_ori_;
  pcl::VoxelGrid<PointType> down_size_filter_surf_;
  KD_TREE<PointType> ikdtree_;
  std::vector<BoxPointType> cub_needrm_;
  BoxPointType local_map_points_;
  bool localmap_initialized_ = false;
  std::vector<PointVector> nearest_points_;
  std::vector<bool> point_selected_surf_;
  std::vector<PointType> normals_;
  int frame_num_ = 0;
  int effect_feat_num_ = 0;
  double last_timestamp_lidar_ = 0.0;
  double last_timestamp_imu_ = 0.0;
  double lidar_beg_time_ = 0.0;
  double lidar_end_time_ = 0.0;
  double first_lidar_time_ = 0.0;
  double mean_point_to_plane_residual_ = 0.0;
  bool last_scan_match_valid_ = false;
  double scan_match_inlier_ratio_ = 0.0;
  double scan_match_delta_rotation_deg_ = 0.0;
  double scan_match_delta_translation_m_ = 0.0;
  bool failed_ = false;
};

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
