load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

package(default_visibility = ["//visibility:public"])

flatbuffer_cc_library(
    name = "fast_lio2_pointcloud_wire",
    srcs = ["pointcloud_wire.fbs"],
)

cc_proto_library(
    name = "vanjee_imu_wire_cc_proto",
    deps = [":vanjee_imu_wire_proto"],
)

proto_library(
    name = "vanjee_imu_wire_proto",
    srcs = ["vanjee_imu_wire.proto"],
    deps = [
        "@wheelos_msgs//wheelos_msgs/basic_msgs:geometry_proto",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_proto",
    ],
)

cc_proto_library(
    name = "fast_lio2_conf_cc_proto",
    deps = [":fast_lio2_conf_proto"],
)

proto_library(
    name = "fast_lio2_conf_proto",
    srcs = ["fast_lio2_conf.proto"],
)

cc_proto_library(
    name = "fast_lio2_metrics_cc_proto",
    deps = [":fast_lio2_metrics_proto"],
)

proto_library(
    name = "fast_lio2_metrics_proto",
    srcs = ["fast_lio2_metrics.proto"],
    deps = [
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_proto",
    ],
)

cc_proto_library(
    name = "fast_lio2_runtime_conf_cc_proto",
    deps = [":fast_lio2_runtime_conf_proto"],
)

proto_library(
    name = "fast_lio2_runtime_conf_proto",
    srcs = ["fast_lio2_runtime_conf.proto"],
)

cc_proto_library(
    name = "fast_lio2_runtime_status_cc_proto",
    deps = [":fast_lio2_runtime_status_proto"],
)

proto_library(
    name = "fast_lio2_runtime_status_proto",
    srcs = ["fast_lio2_runtime_status.proto"],
    deps = [
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_proto",
    ],
)
