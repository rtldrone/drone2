cc_library(
    name = "vesc_driver",
    srcs = glob(["src/*.cpp"]),
    hdrs = glob(["include/vesc_driver/*.hpp"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/vesc/vesc_msgs:vesc_msgs_cpp",
        "@ros2_common_interfaces//:geometry_msgs_cpp",
        "@ros2_common_interfaces//:sensor_msgs_cpp",
        "@ros2_common_interfaces//:std_msgs_cpp",
        "@ros2_rclcpp//:rclcpp",
        "@transport_drivers//:serial_driver",
    ],
)
