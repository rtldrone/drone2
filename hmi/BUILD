load("@com_github_camearle20_ros2bazel//ros2/rules:ros2_interfaces.bzl", "ros2_all_interface_libraries")

ros2_all_interface_libraries(
    name = "hmi_msgs",
    srcs = [
        "msg/HmiInputState.msg",
        "msg/HmiOutputState.msg",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hmi_node",
    srcs = ["hmi_node.cc"],
    hdrs = ["hmi_node.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":hmi_msgs_cpp",
        "//third_party/json",
        "@ros2_common_interfaces//:std_srvs_cpp",
        "@ros2_rclcpp//:rclcpp",
        "@websocketpp",
    ],
)
