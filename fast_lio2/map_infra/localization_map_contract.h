#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace whl {
namespace fast_lio2 {
namespace map_infra {

enum class LocalizationMapArtifactType {
  kDensePointCloud = 0,
  kNdtVoxelMap = 1,
  kIntensityTile = 2,
  kHeightTile = 3,
  kOccupancyTile = 4,
  kTrajectory = 5,
  kRelocalizationKeyframes = 6,
};

struct LocalizationMapArtifact {
  LocalizationMapArtifactType type = LocalizationMapArtifactType::kDensePointCloud;
  std::string relative_path;
  std::string format;
  std::string coordinate_frame;
  std::string description;
};

struct LocalizationMapPackageManifest {
  std::string schema_version = "1.0.0";
  std::string dataset_type = "localization_map_package";
  std::string producer = "whl_fast_lio2";
  std::string map_frame = "map";
  std::string map_origin_id;
  std::string source_optimized_frame_dataset;
  std::string source_dataset_fingerprint;
  std::uint64_t keyframe_count = 0;
  std::uint64_t dense_point_count = 0;
  std::vector<LocalizationMapArtifact> artifacts;
};

}  // namespace map_infra
}  // namespace fast_lio2
}  // namespace whl
