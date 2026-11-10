#include "modules/fast_lio2/adapter/vanjee_imu_adapter.h"

#include <limits>
#include <string>

#include "modules/fast_lio2/proto/vanjee_imu_wire.pb.h"

int main() {
  using namespace apollo::localization::fast_lio2;
  VanjeeImuWire imu;
  imu.mutable_header()->set_frame_id("lidar_imu");
  imu.mutable_header()->set_timestamp_sec(1794215251.2);
  imu.set_measurement_time(1794215251.1);
  imu.mutable_linear_acceleration()->set_x(0.1);
  imu.mutable_linear_acceleration()->set_y(-0.2);
  imu.mutable_linear_acceleration()->set_z(9.8);
  imu.mutable_angular_velocity()->set_x(0.01);
  imu.mutable_angular_velocity()->set_y(0.02);
  imu.mutable_angular_velocity()->set_z(-0.03);
  FastLio2ImuSample sample;
  std::string error;
  if (!ConvertVanjeeImu(imu.SerializeAsString(), &sample, &error) ||
      sample.timestamp_sec != imu.measurement_time() ||
      sample.linear_acceleration.z() != 9.8 ||
      sample.angular_velocity.z() != -0.03) {
    return 1;
  }
  imu.mutable_angular_velocity()->set_x(
      std::numeric_limits<double>::quiet_NaN());
  if (ConvertVanjeeImu(imu.SerializeAsString(), &sample, &error)) return 2;
  imu.clear_angular_velocity();
  if (ConvertVanjeeImu(imu.SerializeAsString(), &sample, &error)) return 3;
  if (ConvertVanjeeImu("invalid protobuf", &sample, &error)) return 4;
  return 0;
}
