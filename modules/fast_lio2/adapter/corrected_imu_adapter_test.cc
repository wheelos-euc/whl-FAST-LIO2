#include "modules/fast_lio2/adapter/corrected_imu_adapter.h"

#include <cmath>
#include <string>

namespace {

bool Near(double a, double b) { return std::abs(a - b) < 1e-9; }

}  // namespace

int main() {
  using namespace apollo::localization::fast_lio2;

  apollo::localization::CorrectedImu imu;
  imu.mutable_header()->set_timestamp_sec(1700000000.25);
  auto* pose = imu.mutable_imu();
  pose->mutable_angular_velocity()->set_x(10.0);
  pose->mutable_linear_acceleration()->set_x(20.0);
  pose->mutable_angular_velocity_vrf()->set_x(0.01);
  pose->mutable_angular_velocity_vrf()->set_y(-0.02);
  pose->mutable_angular_velocity_vrf()->set_z(0.03);
  pose->mutable_linear_acceleration_vrf()->set_x(1.0);
  pose->mutable_linear_acceleration_vrf()->set_y(-2.0);
  pose->mutable_linear_acceleration_vrf()->set_z(9.8);

  FastLio2ImuSample sample;
  std::string error;
  if (!ConvertCorrectedImu(imu, &sample, &error)) return 1;
  if (!Near(sample.timestamp_sec, 1700000000.25) ||
      !Near(sample.angular_velocity.x(), 0.01) ||
      !Near(sample.angular_velocity.z(), 0.03) ||
      !Near(sample.linear_acceleration.y(), -2.0) ||
      !Near(sample.linear_acceleration.z(), 9.8)) {
    return 2;
  }

  pose->clear_linear_acceleration_vrf();
  if (ConvertCorrectedImu(imu, &sample, &error)) return 3;
  if (ConvertCorrectedImu(imu, nullptr, &error)) return 4;

  return 0;
}
