load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

FAST_LIO2_COPTS = [
    "-std=c++17",
    "-fopenmp",
    "-DEIGEN_MAX_ALIGN_BYTES=32",
    '-DMODULE_NAME=\\"fast_lio2\\"',
    '-DROOT_DIR=\\\"/tmp/whl_fast_lio2/\\\"',
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
        "adapter/sensor_contract.cc",
        "adapter/vanjee_imu_adapter.cc",
    ],
    hdrs = [
        "adapter/corrected_imu_adapter.h",
        "adapter/pointcloud_view_adapter.h",
        "adapter/runtime_data.h",
        "adapter/sensor_contract.h",
        "adapter/vanjee_imu_adapter.h",
        "core/include/color.h",
        "core/include/common_lib.h",
        "core/include/scope_timer.hpp",
        "core/include/so3_math.h",
        "core/ros_compat.h",
    ],
    copts = FAST_LIO2_COPTS,
    deps = [
        "//modules/fast_lio2/proto:fast_lio2_runtime_conf_cc_proto",
        "//modules/fast_lio2/proto:vanjee_imu_wire_cc_proto",
        "@eigen",
        "@local_config_pcl//:common",
        "@wheelos_core//cyber",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:imu_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_test(
    name = "vanjee_imu_adapter_test",
    srcs = ["adapter/vanjee_imu_adapter_test.cc"],
    copts = FAST_LIO2_COPTS,
    deps = [
        ":fast_lio2_adapter",
        "//modules/fast_lio2/proto:vanjee_imu_wire_cc_proto",
    ],
)

cc_library(
    name = "fast_lio2_runtime_core",
    srcs = [
        "core/fast_lio2_runtime_core.cc",
        "core/include/ikd-Tree/ikd_Tree.cpp",
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
        "//modules/fast_lio2/proto:fast_lio2_runtime_conf_cc_proto",
        "//modules/fast_lio2/proto:fast_lio2_runtime_status_cc_proto",
        "@eigen",
        "@local_config_pcl//:common",
        "@local_config_pcl//:filters",
        "@local_config_pcl//:io",
        "@local_config_pcl//:kdtree",
        "@wheelos_core//cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
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
        "//modules/fast_lio2/proto:fast_lio2_conf_cc_proto",
        "//modules/fast_lio2/proto:fast_lio2_metrics_cc_proto",
        "//modules/fast_lio2/proto:fast_lio2_runtime_conf_cc_proto",
        "@eigen",
        "@wheelos_core//cyber",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:imu_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:imu_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libfast_lio2_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":fast_lio2_component_lib"],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/fast_lio2.pb.txt",
        "dag/**",
        "launch/**",
    ]),
)
