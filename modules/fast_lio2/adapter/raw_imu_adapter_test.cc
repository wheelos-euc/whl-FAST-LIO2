#include "modules/fast_lio2/adapter/raw_imu_adapter.h"

#include <cmath>
#include <string>

namespace {

bool Near(double a, double b) { return std::abs(a - b) < 1e-9; }

void FillRawImu(apollo::drivers::gnss::Imu* imu) {
  imu->set_measurement_time(100.0);
  imu->mutable_angular_velocity()->set_x(0.01);
  imu->mutable_angular_velocity()->set_y(-0.02);
  imu->mutable_angular_velocity()->set_z(0.03);
  imu->mutable_linear_acceleration()->set_x(1.0);
  imu->mutable_linear_acceleration()->set_y(-2.0);
  imu->mutable_linear_acceleration()->set_z(9.8);
}

}  // namespace

int main() {
  using namespace apollo::localization::fast_lio2;

  apollo::drivers::gnss::Imu imu;
  FillRawImu(&imu);
  FastLio2ImuSample sample;
  std::string error;
  if (!ConvertRawGnssImu(imu, 200.0, false, &sample, &error)) return 1;
  if (!Near(sample.timestamp_sec, 315964882.0) ||
      !Near(sample.angular_velocity.y(), -0.02) ||
      !Near(sample.linear_acceleration.z(), 9.8)) {
    return 2;
  }

  if (!ConvertRawGnssImu(imu, 1234.5, true, &sample, &error) ||
      !Near(sample.timestamp_sec, 1234.5)) {
    return 3;
  }

  imu.clear_measurement_time();
  imu.mutable_header()->set_timestamp_sec(456.25);
  if (!ConvertRawGnssImu(imu, 0.0, false, &sample, &error) ||
      !Near(sample.timestamp_sec, 456.25)) {
    return 4;
  }

  imu.clear_angular_velocity();
  if (ConvertRawGnssImu(imu, 0.0, false, &sample, &error)) return 5;
  if (ConvertRawGnssImu(imu, 0.0, false, nullptr, &error)) return 6;

  return 0;
}
