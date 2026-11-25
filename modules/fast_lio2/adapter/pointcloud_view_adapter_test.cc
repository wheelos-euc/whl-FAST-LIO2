#include "modules/fast_lio2/adapter/pointcloud_view_adapter.h"

#include <flatbuffers/flatbuffers.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "modules/fast_lio2/proto/pointcloud_wire_generated.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace {

constexpr uint64_t kScanStartNs = 1700000000000000000ULL;

bool Near(double a, double b) { return std::abs(a - b) < 1e-5; }

apollo::localization::fast_lio2::FastLio2RuntimeConf MakeConf() {
  apollo::localization::fast_lio2::FastLio2RuntimeConf conf;
  conf.set_point_filter_num(1);
  conf.set_blind(0.0);
  return conf;
}

std::string MakeProtoCloud() {
  apollo::drivers::PointCloud cloud;
  for (uint64_t i = 0; i < 6; ++i) {
    auto* point = cloud.add_point();
    point->set_x(static_cast<float>(10 + i));
    point->set_y(1.0F);
    point->set_z(2.0F);
    point->set_intensity(static_cast<uint32_t>(i + 1));
    point->set_timestamp(kScanStartNs + i * 10000000ULL);
  }
  std::string serialized;
  cloud.SerializeToString(&serialized);
  return serialized;
}

std::string MakeFlatCloud() {
  namespace wire = apollo::localization::fast_lio2::wire::flat;
  flatbuffers::FlatBufferBuilder builder;
  std::vector<wire::PointXYZIT> points;
  for (uint64_t i = 0; i < 6; ++i) {
    points.emplace_back(static_cast<float>(10 + i), 1.0F, 2.0F,
                        static_cast<uint32_t>(i + 1),
                        kScanStartNs + i * 10000000ULL);
  }
  const auto point_vector = builder.CreateVectorOfStructs(points);
  const auto cloud = wire::CreatePointCloud(builder, 0, 0, false, point_vector,
                                            1700000000.05, 6, 1);
  wire::FinishPointCloudBuffer(builder, cloud);
  return std::string(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                     builder.GetSize());
}

bool CheckCloud(const std::string& serialized) {
  using namespace apollo::localization::fast_lio2;
  FastLio2PointCloudFrame frame;
  std::string error;
  if (!ConvertPointCloud(serialized, 1800000000.0, MakeConf(), &frame,
                         &error)) {
    std::cerr << error << '\n';
    return false;
  }
  const bool valid = frame.points->size() == 6 &&
                     Near(frame.points->front().intensity, 1.0) &&
                     Near(frame.points->back().intensity, 6.0) &&
                     Near(frame.points->front().curvature, 0.0) &&
                     Near(frame.points->back().curvature, 50.0) &&
                     Near(frame.scan_start_sec, 1700000000.0) &&
                     Near(frame.scan_end_sec, 1700000000.05);
  if (!valid) {
    std::cerr << "cloud size=" << frame.points->size()
              << " first_i=" << frame.points->front().intensity
              << " last_i=" << frame.points->back().intensity
              << " first_t=" << frame.points->front().curvature
              << " last_t=" << frame.points->back().curvature
              << " start=" << frame.scan_start_sec
              << " end=" << frame.scan_end_sec << '\n';
  }
  return valid;
}

}  // namespace

int main() {
  using namespace apollo::localization::fast_lio2;

  if (!CheckCloud(MakeProtoCloud())) return 1;
  if (!CheckCloud(MakeFlatCloud())) return 2;

  FastLio2PointCloudFrame frame;
  std::string error;
  if (ConvertPointCloud("invalid", 0.0, MakeConf(), &frame, &error)) return 3;
  if (ConvertPointCloud(std::string("\0\0\0\0PCLD", 8), 0.0, MakeConf(), &frame,
                        &error)) {
    return 4;
  }
  return 0;
}
