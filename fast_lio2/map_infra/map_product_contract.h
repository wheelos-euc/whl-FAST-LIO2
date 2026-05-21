#pragma once

#include <string>

#include "fast_lio2/map_infra/localization_map_contract.h"
#include "fast_lio2/map_infra/optimized_frame_contract.h"

namespace whl {
namespace fast_lio2 {
namespace map_infra {

struct TileMapProductManifest {
  std::string schema_version = "1.0.0";
  std::string dataset_type = "tile_map_product";
  std::string producer = "map_infra_renderer";
  std::string map_frame = "map";
  std::string map_origin_id;
  std::string source_optimized_frame_dataset;
  std::string source_dataset_fingerprint;
  std::string tile_bundle_dir = "tile_bundle";
  double resolution_m = 0.10;
};

struct MapProductBundleManifest {
  std::string schema_version = "1.0.0";
  std::string dataset_type = "map_product_bundle";
  std::string producer = "whl_fast_lio2";
  std::string map_frame = "map";
  std::string map_origin_id;
  std::string source_optimized_frame_dataset;
  std::string source_dataset_fingerprint;
  std::string optimized_frame_dataset_dir = "optimized_frame_dataset";
  std::string tile_bundle_dir = "tile_bundle";
  std::string localization_map_package_dir = "localization_map_package";
};

// The renderer and localization-map builder must consume the same optimized
// frames. This validator catches split-brain map products before deployment.
bool ValidateMapProductConsistency(
    const OptimizedFrameDatasetManifest& optimized_dataset,
    const TileMapProductManifest& tile_map,
    const LocalizationMapPackageManifest& localization_map,
    const MapProductBundleManifest& bundle, std::string* error);

}  // namespace map_infra
}  // namespace fast_lio2
}  // namespace whl

