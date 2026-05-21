/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <string>

#include "modules/fast_lio2/proto/fast_lio2_runtime_conf.pb.h"

#include "cyber/adaptive/pointcloud_view.h"
#include "modules/fast_lio2/adapter/runtime_data.h"

namespace apollo {
namespace localization {
namespace fast_lio2 {

bool ConvertPointCloudView(const apollo::cyber::adaptive::PointCloudView& in,
                           double receive_time_sec,
                           const FastLio2RuntimeConf& conf,
                           FastLio2PointCloudFrame* out, std::string* error);

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
