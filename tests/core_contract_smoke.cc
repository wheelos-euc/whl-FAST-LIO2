#include <cassert>
#include <string>

#include "fast_lio2/core/fast_lio2_core.h"

int main() {
  whl::fast_lio2::FastLio2Core core;
  whl::fast_lio2::FastLio2Options options;
  std::string error;
  assert(core.Init(options, &error));

  whl::fast_lio2::ExtrinsicLidarToImu extrinsic;
  core.SetExtrinsic(extrinsic);

  whl::fast_lio2::ImuSample imu;
  imu.timestamp_sec = 1.0;
  imu.linear_acceleration_mps2.z = 9.81;
  core.AddImu(imu);
  imu.timestamp_sec = 1.11;
  core.AddImu(imu);

  whl::fast_lio2::PointCloudFrame cloud;
  cloud.scan_start_sec = 1.0;
  cloud.scan_end_sec = 1.1;
  for (int i = 0; i < 8; ++i) {
    whl::fast_lio2::PointXYZIT point;
    point.x = static_cast<float>(i);
    point.relative_time_sec = 0.001 * i;
    cloud.points.push_back(point);
  }
  core.AddPointCloud(cloud);

  const auto result = core.ProcessNext();
  assert(result.stats.imu_samples_received == 2);
  assert(result.stats.lidar_frames_received == 1);
  assert(result.stats.frames_processed == 1);
  const auto result2 = core.ProcessNext();
  assert(result2.stats.frames_processed == 1);
  return 0;
}
