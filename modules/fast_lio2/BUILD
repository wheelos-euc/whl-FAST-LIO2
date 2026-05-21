load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

FAST_LIO2_COPTS = [
    "-std=c++17",
    "-fopenmp",
    '-DMODULE_NAME=\\"fast_lio2\\"',
    '-DROOT_DIR=\\\"/apollo/data/fast_lio2/\\\"',
    "-Imodules/fast_lio2/core",
    "-Imodules/fast_lio2/core/include",
    "-Imodules/fast_lio2/core/include/LI_init",
    "-Imodules/fast_lio2/core/include/ikd-Tree",
]

FAST_LIO2_LINKOPTS = [
    "-lgomp",
]

cc_library(
    name = "fast_lio2_adapter",
    srcs = [
        "adapter/corrected_imu_adapter.cc",
        "adapter/pointcloud_view_adapter.cc",
    ],
    hdrs = [
        "adapter/runtime_data.h",
        "adapter/corrected_imu_adapter.h",
        "adapter/pointcloud_view_adapter.h",
        "core/ros_compat.h",
        "core/include/color.h",
        "core/include/common_lib.h",
        "core/include/scope_timer.hpp",
        "core/include/so3_math.h",
    ],
    copts = FAST_LIO2_COPTS,
    deps = [
        "//cyber",
        "//cyber/adaptive:adaptive_pointcloud_reader",
        "//modules/common_msgs/localization_msgs:imu_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/fast_lio2/proto:fast_lio2_runtime_conf_cc_proto",
        "@eigen",
        "@local_config_pcl//:common",
    ],
)

cc_library(
    name = "fast_lio2_runtime_core",
    srcs = [
        "core/include/LI_init/LI_init.cpp",
        "core/include/ikd-Tree/ikd_Tree.cpp",
        "core/fast_lio2_runtime_core.cc",
    ],
    hdrs = glob([
        "core/*.h",
        "core/include/**/*.h",
        "core/include/**/*.hpp",
    ]),
    copts = FAST_LIO2_COPTS,
    linkopts = FAST_LIO2_LINKOPTS,
    deps = [
        ":fast_lio2_adapter",
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/fast_lio2/proto:fast_lio2_runtime_conf_cc_proto",
        "//modules/fast_lio2/proto:fast_lio2_runtime_status_cc_proto",
        "@boost.filesystem",
        "@eigen",
        "@local_config_pcl//:common",
        "@local_config_pcl//:filters",
        "@local_config_pcl//:io",
        "@local_config_pcl//:kdtree",
        "@remote_ceres//:ceres",
    ],
)

cc_library(
    name = "fast_lio2_component_lib",
    srcs = ["component/fast_lio2_component.cc"],
    hdrs = ["component/fast_lio2_component.h"],
    copts = FAST_LIO2_COPTS,
    deps = [
        ":fast_lio2_adapter",
        ":fast_lio2_runtime_core",
        "//cyber",
        "//cyber/adaptive:adaptive_pointcloud_reader",
        "//modules/common/util:time_conversion",
        "//modules/common_msgs/localization_msgs:imu_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/sensor_msgs:imu_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/fast_lio2/proto:fast_lio2_conf_cc_proto",
        "//modules/fast_lio2/proto:fast_lio2_metrics_cc_proto",
        "//modules/fast_lio2/proto:fast_lio2_runtime_conf_cc_proto",
        "//modules/slam_localization/proto:odometry_cc_proto",
        "@eigen",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libfast_lio2_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":fast_lio2_component_lib"],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/**",
        "launch/**",
    ]),
)
