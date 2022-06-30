cc_library(
    name = "vesc_adapter",
    srcs = ["vesc_adapter.cc"],
    hdrs = ["vesc_adapter.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/vesc/vesc_msgs:vesc_msgs_cpp",
        "@ros2_common_interfaces//:sensor_msgs_cpp",
        "@ros2_rclcpp//:rclcpp",
    ],
)
