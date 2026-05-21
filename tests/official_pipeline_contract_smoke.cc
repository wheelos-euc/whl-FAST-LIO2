#include <cassert>
#include <string>
#include <vector>

#include "fast_lio2/core/official_pipeline_contract.h"

int main() {
  const auto pipeline = whl::fast_lio2::OfficialFastLio2Pipeline();
  std::vector<whl::fast_lio2::OfficialFastLio2Stage> observed;
  observed.reserve(pipeline.size());
  for (const auto& stage : pipeline) {
    assert(!stage.name.empty());
    assert(!stage.official_symbol.empty());
    assert(!stage.invariant.empty());
    observed.push_back(stage.stage);
  }

  std::string error;
  assert(whl::fast_lio2::ValidateOfficialFastLio2StageOrder(observed, &error));
  std::swap(observed[1], observed[2]);
  assert(!whl::fast_lio2::ValidateOfficialFastLio2StageOrder(observed, &error));
  assert(error.find("mismatch") != std::string::npos);
  return 0;
}

