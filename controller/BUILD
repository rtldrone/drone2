cc_library(
    name = "velocity_profiler",
    srcs = ["velocity_profiler.cc"],
    hdrs = ["velocity_profiler.h"],
)

cc_library(
    name = "blinker",
    srcs = ["blinker.cc"],
    hdrs = ["blinker.h"],
)

cc_library(
    name = "controller_node",
    srcs = ["controller_node.cc"],
    hdrs = ["controller_node.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":blinker",
        ":velocity_profiler",
        "//hardware_nodes:stacklight_state_msg_cpp",
        "//hmi:hmi_msgs_cpp",
        "//third_party/vesc/vesc_msgs:vesc_msgs_cpp",
        "@ros2_common_interfaces//:sensor_msgs_cpp",
        "@ros2_common_interfaces//:std_msgs_cpp",
        "@ros2_common_interfaces//:std_srvs_cpp",
        "@ros2_rclcpp//:rclcpp",
    ],
)
