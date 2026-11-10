#pragma once

#include <string>

#include "modules/fast_lio2/adapter/runtime_data.h"

namespace apollo {
namespace localization {
namespace fast_lio2 {

bool ConvertVanjeeImu(const std::string& serialized, FastLio2ImuSample* out,
                      std::string* error);

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
