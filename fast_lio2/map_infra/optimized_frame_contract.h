#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "fast_lio2/common/types.h"

namespace whl {
namespace fast_lio2 {
namespace map_infra {

enum class PointLabel : std::uint8_t {
  kUnknown = 0,
  kGround = 1,
  kObstacle = 2,
  kLaneLine = 3,
  kNoise = 4,
};

struct OptimizedPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  float intensity = 0.0F;
  PointLabel label = PointLabel::kUnknown;
};

// Handoff contract compatible with map-infra optimized_frame_dataset:
// frame-local points plus the pose that resolves them into world/map frame.
struct OptimizedFrame {
  std::uint64_t timestamp_ns = 0;
  Pose origin;
  std::vector<OptimizedPoint> points;
};

struct OptimizedFrameDatasetManifest {
  std::string schema_version = "1.3.0";
  std::string dataset_type = "optimized_frame_dataset";
  std::string producer = "whl_fast_lio2";
  std::string point_coordinate_frame = "frame_local";
  std::string origin_coordinate_frame = "map";
  std::string map_frame = "map";
  std::string map_origin_id;
  std::string source_dataset_fingerprint;
  std::uint64_t frame_count = 0;
};

}  // namespace map_infra
}  // namespace fast_lio2
}  // namespace whl
