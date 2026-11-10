#include "modules/fast_lio2/adapter/vanjee_imu_adapter.h"

#include "modules/fast_lio2/adapter/sensor_contract.h"
#include "modules/fast_lio2/proto/vanjee_imu_wire.pb.h"

namespace apollo {
namespace localization {
namespace fast_lio2 {

bool ConvertVanjeeImu(const std::string& serialized, FastLio2ImuSample* out,
                      std::string* error) {
  VanjeeImuWire imu;
  if (out == nullptr || !imu.ParseFromString(serialized) || !imu.has_header() ||
      imu.header().frame_id().empty() || !imu.has_measurement_time() ||
      !imu.has_linear_acceleration() || !imu.has_angular_velocity()) {
    if (error != nullptr) {
      *error = "VanjeeImu is malformed or missing frame/time/six-axis fields";
    }
    return false;
  }
  out->timestamp_sec = imu.measurement_time();
  out->angular_velocity =
      Eigen::Vector3d(imu.angular_velocity().x(), imu.angular_velocity().y(),
                      imu.angular_velocity().z());
  out->linear_acceleration = Eigen::Vector3d(imu.linear_acceleration().x(),
                                             imu.linear_acceleration().y(),
                                             imu.linear_acceleration().z());
  return ValidateFastLioImuSample(*out, error);
}

}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
