#pragma once

#include <string>
#include <vector>

namespace whl {
namespace fast_lio2 {

enum class OfficialFastLio2Stage {
  kSyncPackages = 0,
  kImuProcessAndUndistort = 1,
  kLocalMapFovSegment = 2,
  kVoxelDownsample = 3,
  kIkdTreeInitOrNearestSearch = 4,
  kIteratedKalmanUpdate = 5,
  kIncrementalMapUpdate = 6,
  kPublishOrExportOutputs = 7,
};

struct OfficialFastLio2StageSpec {
  OfficialFastLio2Stage stage = OfficialFastLio2Stage::kSyncPackages;
  std::string name;
  std::string official_symbol;
  std::string invariant;
};

std::vector<OfficialFastLio2StageSpec> OfficialFastLio2Pipeline();

bool ValidateOfficialFastLio2StageOrder(
    const std::vector<OfficialFastLio2Stage>& observed, std::string* error);

const char* OfficialFastLio2StageName(OfficialFastLio2Stage stage);

}  // namespace fast_lio2
}  // namespace whl

