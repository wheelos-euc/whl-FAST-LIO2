#pragma once

#include <cassert>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <string>

#include "cyber/common/log.h"

namespace apollo {
namespace localization {
namespace fast_lio2 {
namespace compat {

class Time {
 public:
  Time() = default;
  explicit Time(double sec) : sec_(sec) {}

  static Time FromSec(double sec) { return Time(sec); }

  double toSec() const { return sec_; }

 private:
  double sec_ = 0.0;
};

struct Header {
  Time stamp;
  std::string frame_id;
  uint32_t seq = 0;
};

struct Vector3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Imu {
  using Ptr = std::shared_ptr<Imu>;
  using ConstPtr = std::shared_ptr<const Imu>;

  Header header;
  Vector3 angular_velocity;
  Vector3 linear_acceleration;
};

struct Pose6D {
  double offset_time = 0.0;
  double acc[3] = {0.0, 0.0, 0.0};
  double gyr[3] = {0.0, 0.0, 0.0};
  double vel[3] = {0.0, 0.0, 0.0};
  double pos[3] = {0.0, 0.0, 0.0};
  double rot[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

}  // namespace compat
}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo

namespace sensor_msgs {
using Imu = ::apollo::localization::fast_lio2::compat::Imu;
using ImuConstPtr = ::apollo::localization::fast_lio2::compat::Imu::ConstPtr;
}  // namespace sensor_msgs

namespace lidar_imu_init {
using Pose6D = ::apollo::localization::fast_lio2::compat::Pose6D;
}  // namespace lidar_imu_init

#define ROS_WARN(...) AWARN << ::apollo::localization::fast_lio2::compat::FormatLog(__VA_ARGS__)
#define ROS_ERROR(...) AERROR << ::apollo::localization::fast_lio2::compat::FormatLog(__VA_ARGS__)
#define ROS_INFO(...) AINFO << ::apollo::localization::fast_lio2::compat::FormatLog(__VA_ARGS__)
#define ROS_ASSERT(expr) assert(expr)

namespace apollo {
namespace localization {
namespace fast_lio2 {
namespace compat {

template <typename... Args>
std::string FormatLog(const char* fmt, Args... args) {
  int size = std::snprintf(nullptr, 0, fmt, args...);
  if (size <= 0) {
    return std::string(fmt);
  }
  std::string out(static_cast<size_t>(size), '\0');
  std::snprintf(&out[0], out.size() + 1, fmt, args...);
  return out;
}

inline std::string FormatLog(const std::string& msg) { return msg; }
inline std::string FormatLog(const char* msg) { return std::string(msg); }

}  // namespace compat
}  // namespace fast_lio2
}  // namespace localization
}  // namespace apollo
