#pragma once

#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "modules/fast_lio2/proto/fast_lio2_runtime_conf.pb.h"
#include "modules/fast_lio2/proto/fast_lio2_runtime_status.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

#include "modules/fast_lio2/adapter/runtime_data.h"
#include "modules/fast_lio2/core/include/common_lib.h"
#include "modules/fast_lio2/core/include/ikd-Tree/ikd_Tree.h"

class ImuProcess;
class LI_Init;

namespace apollo {
namespace localization {
namespace fast_lio2 {

struct FastLio2RuntimeStateSnapshot {
  double timestamp_sec = 0.0;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> covariance =
      Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity();
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
  bool Init(const FastLio2RuntimeConf& conf);
  void AddPointCloud(const FastLio2PointCloudFrame& cloud);
  void AddImu(const FastLio2ImuSample& imu);
  bool Process();

  bool Finished() const { return finished_; }
  bool Failed() const { return failed_; }
  const FastLio2RuntimeStatus& Status() const { return status_; }

  bool GetRegisteredCloud(apollo::drivers::PointCloud* out) const;
  bool GetInitialExtrinsicCloud(apollo::drivers::PointCloud* out) const;
  bool GetRegisteredBodyCloud(apollo::drivers::PointCloud* out) const;
  bool GetMapCloud(apollo::drivers::PointCloud* out);
  bool GetStateSnapshot(FastLio2RuntimeStateSnapshot* out);

 private:
  enum class Stage {
    WAITING_FOR_DATA,
    LIDAR_ONLY_ODOMETRY,
    DATA_ACCUMULATION,
    INITIALIZING,
    REFINING,
    FINISHED,
    ERROR,
  };

  bool SyncPackages(MeasureGroup* meas);
  void ProcessMeasure(const MeasureGroup& meas);
  void SetStage(Stage stage, const std::string& message);
  void PublishProgressStatus();
  void Finish();
  void Fail(const std::string& message);

  void LasermapFovSegment();
  void PointsCacheCollect();
  void MapIncremental();
  void PointBodyToWorld(const PointType* pi, PointType* po) const;
  void PointCloudToApollo(const PointCloudXYZI& cloud,
                          const std::string& frame_id,
                          apollo::drivers::PointCloud* out) const;
  void CurrentWorldCloud(PointCloudXYZI* out) const;
  void WriteResultFile();
  bool InitVehiclePrior();
  bool ValidateScanMatch(const StatesGroup& state_before_update,
                         const StatesGroup& state_after_update);
  bool VehicleDataSufficient(const V3D& progress) const;
  void EnableRefinementAfterBridgeFrame(const MeasureGroup& meas);

  static const char* StageName(Stage stage);

  FastLio2RuntimeConf conf_;
  FastLio2RuntimeStatus status_;
  Stage stage_ = Stage::WAITING_FOR_DATA;

  std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
  std::deque<double> time_buffer_;
  std::deque<sensor_msgs::Imu::Ptr> imu_buffer_;
  bool lidar_pushed_ = false;

  std::shared_ptr<::ImuProcess> imu_process_;
  std::shared_ptr<::LI_Init> init_li_;
  MeasureGroup measures_;
  StatesGroup state_;
  StatesGroup state_propagat_;

  PointCloudXYZI::Ptr feats_from_map_;
  PointCloudXYZI::Ptr feats_undistort_;
  PointCloudXYZI::Ptr feats_down_body_;
  PointCloudXYZI::Ptr feats_down_world_;
  PointCloudXYZI::Ptr normvec_;
  PointCloudXYZI::Ptr laser_cloud_ori_;
  PointCloudXYZI::Ptr corr_normvect_;
  PointCloudXYZI::Ptr feats_array_;

  pcl::VoxelGrid<PointType> down_size_filter_surf_;
  pcl::VoxelGrid<PointType> down_size_filter_map_;
  KD_TREE ikdtree_;

  std::vector<BoxPointType> cub_needrm_;
  BoxPointType local_map_points_;
  bool localmap_initialized_ = false;

  std::vector<std::vector<int>> point_search_ind_surf_;
  std::vector<PointVector> nearest_points_;
  std::vector<bool> point_selected_surf_;
  std::vector<float> res_last_;

  MatrixXd jacobian_rot_;

  int frame_num_ = 0;
  int feats_down_size_ = 0;
  int effect_feat_num_ = 0;
  int num_max_iterations_ = 5;
  int orig_odom_freq_ = 10;
  int cut_frame_num_ = 3;

  double last_timestamp_lidar_ = 0.0;
  double last_timestamp_imu_ = 0.0;
  double lidar_beg_time_ = 0.0;
  double lidar_end_time_ = 0.0;
  double first_lidar_time_ = 0.0;
  double move_start_time_ = 0.0;
  double online_refine_start_time_ = 0.0;
  double time_lag_imu_wrt_lidar_ = 0.0;
  double timediff_imu_wrt_lidar_ = 0.0;
  double mean_acc_norm_ = 9.81;
  double total_distance_ = 0.0;

  double filter_size_surf_min_ = 0.3;
  double filter_size_map_min_ = 0.5;
  double cube_len_ = 2000.0;
  double det_range_ = 100.0;
  double mean_point_to_plane_residual_ = 0.0;
  double extrinsic_delta_rotation_deg_ = 0.0;
  double extrinsic_delta_translation_m_ = 0.0;
  double lidar_angular_velocity_norm_ = 0.0;
  bool vehicle_prior_mode_ = false;
  M3D initial_R_LI_ = Eye3d;
  V3D initial_T_LI_ = Zero3d;
  bool last_scan_match_valid_ = true;
  uint32_t consecutive_rejected_frames_ = 0;
  std::string last_scan_match_reject_reason_ = "OK";
  double scan_match_inlier_ratio_ = 0.0;
  double scan_match_delta_rotation_deg_ = 0.0;
  double scan_match_delta_translation_m_ = 0.0;

  bool imu_en_ = false;
  bool refine_enable_pending_ = false;
  bool bootstrap_data_started_ = false;
  bool bootstrap_data_ready_ = false;
  bool online_refine_finished_ = false;
  bool refine_print_ = false;
  int refine_debug_frame_count_ = 0;
  bool finished_ = false;
  bool failed_ = false;

  M3D last_rot_for_init_ = M3D::Identity();
  double last_time_for_init_ = 0.0;
  bool last_init_sample_set_ = false;
  V3D position_last_ = Zero3d;

  M3D pending_R_LI_ = Eye3d;
  V3D pending_T_LI_ = Zero3d;
  V3D pending_gravity_ = Zero3d;
  V3D pending_bias_g_ = Zero3d;
  V3D pending_bias_a_ = Zero3d;
  double pending_time_lag_imu_wrt_lidar_ = 0.0;
};

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
