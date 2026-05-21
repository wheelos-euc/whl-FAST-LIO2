#pragma once

#include <string>

namespace whl {
namespace fast_lio2 {
namespace pipeline {

enum class PipelineStage {
  kInputAudit = 0,
  kFastLio2Mapping = 1,
  kOptimizedFrameDatasetExport = 2,
  kTileRendering = 3,
  kLocalizationMapBuild = 4,
  kScorecard = 5,
};

struct PipelineArtifactLayout {
  std::string output_root;
  std::string input_audit_json = "diagnostics/input_audit.json";
  std::string optimized_frame_dataset = "optimized_frame_dataset";
  std::string tile_bundle = "tile_bundle";
  std::string localization_map_package = "localization_map_package";
  std::string map_product_bundle_manifest = "map_product_bundle/manifest.json";
  std::string trajectory_csv = "trajectory/fast_lio2_pose.csv";
  std::string metrics_csv = "diagnostics/fast_lio2_metrics.csv";
  std::string scorecard_json = "diagnostics/scorecard.json";
};

struct FastLio2PipelineRequest {
  std::string dataset_id;
  std::string record_path;
  std::string pointcloud_topic;
  std::string imu_topic;
  std::string reference_pose_topic;
  std::string lidar_to_imu_extrinsic_source;
  bool build_tiles = true;
  bool build_localization_map = true;
  PipelineArtifactLayout layout;
};

}  // namespace pipeline
}  // namespace fast_lio2
}  // namespace whl
