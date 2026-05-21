#include <cassert>
#include <string>

#include "fast_lio2/map_infra/map_product_contract.h"

int main() {
  whl::fast_lio2::map_infra::OptimizedFrameDatasetManifest optimized;
  optimized.map_frame = "map";
  optimized.map_origin_id = "zhongji_20251009_local_enu";
  optimized.source_dataset_fingerprint = "sha256:optimized-dataset";

  whl::fast_lio2::map_infra::TileMapProductManifest tile;
  tile.map_frame = optimized.map_frame;
  tile.map_origin_id = optimized.map_origin_id;
  tile.source_dataset_fingerprint = optimized.source_dataset_fingerprint;

  whl::fast_lio2::map_infra::LocalizationMapPackageManifest localization;
  localization.map_frame = optimized.map_frame;
  localization.map_origin_id = optimized.map_origin_id;
  localization.source_dataset_fingerprint =
      optimized.source_dataset_fingerprint;

  whl::fast_lio2::map_infra::MapProductBundleManifest bundle;
  bundle.map_frame = optimized.map_frame;
  bundle.map_origin_id = optimized.map_origin_id;
  bundle.source_dataset_fingerprint = optimized.source_dataset_fingerprint;

  std::string error;
  assert(whl::fast_lio2::map_infra::ValidateMapProductConsistency(
      optimized, tile, localization, bundle, &error));

  localization.map_frame = "different_map";
  assert(!whl::fast_lio2::map_infra::ValidateMapProductConsistency(
      optimized, tile, localization, bundle, &error));
  assert(error.find("localization.map_frame") != std::string::npos);
  return 0;
}

