#include "fast_lio2/map_infra/map_product_contract.h"

namespace whl {
namespace fast_lio2 {
namespace map_infra {

namespace {

bool RequireEqual(const std::string& name, const std::string& lhs,
                  const std::string& rhs, std::string* error) {
  if (lhs == rhs) {
    return true;
  }
  if (error != nullptr) {
    *error = name + " mismatch: '" + lhs + "' != '" + rhs + "'";
  }
  return false;
}

bool RequireNonEmpty(const std::string& name, const std::string& value,
                     std::string* error) {
  if (!value.empty()) {
    return true;
  }
  if (error != nullptr) {
    *error = name + " must not be empty";
  }
  return false;
}

}  // namespace

bool ValidateMapProductConsistency(
    const OptimizedFrameDatasetManifest& optimized_dataset,
    const TileMapProductManifest& tile_map,
    const LocalizationMapPackageManifest& localization_map,
    const MapProductBundleManifest& bundle, std::string* error) {
  if (!RequireNonEmpty("source_dataset_fingerprint",
                       optimized_dataset.source_dataset_fingerprint, error)) {
    return false;
  }
  if (!RequireNonEmpty("map_origin_id", optimized_dataset.map_origin_id, error)) {
    return false;
  }

  const std::string& fingerprint = optimized_dataset.source_dataset_fingerprint;
  const std::string& map_frame = optimized_dataset.map_frame;
  const std::string& map_origin_id = optimized_dataset.map_origin_id;

  return RequireEqual("tile.source_dataset_fingerprint",
                      tile_map.source_dataset_fingerprint, fingerprint, error) &&
         RequireEqual("localization.source_dataset_fingerprint",
                      localization_map.source_dataset_fingerprint, fingerprint,
                      error) &&
         RequireEqual("bundle.source_dataset_fingerprint",
                      bundle.source_dataset_fingerprint, fingerprint, error) &&
         RequireEqual("tile.map_frame", tile_map.map_frame, map_frame, error) &&
         RequireEqual("localization.map_frame", localization_map.map_frame,
                      map_frame, error) &&
         RequireEqual("bundle.map_frame", bundle.map_frame, map_frame, error) &&
         RequireEqual("tile.map_origin_id", tile_map.map_origin_id,
                      map_origin_id, error) &&
         RequireEqual("localization.map_origin_id",
                      localization_map.map_origin_id, map_origin_id, error) &&
         RequireEqual("bundle.map_origin_id", bundle.map_origin_id,
                      map_origin_id, error);
}

}  // namespace map_infra
}  // namespace fast_lio2
}  // namespace whl

