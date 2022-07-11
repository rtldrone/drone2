load("@com_github_camearle20_ros2bazel//ros2/rules:ros2_interfaces.bzl", "ros2_all_interface_libraries")

ros2_all_interface_libraries(
    name = "stacklight_state_msg",
    srcs = ["msg/StacklightState.msg"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "stacklight_node_neopi3",
    srcs = ["neo_3_stacklight_node.cc"],
    hdrs = ["neo_3_stacklight_node.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":stacklight_state_msg_cpp",
        "@gpiod",
        "@ros2_rclcpp//:rclcpp",
    ],
)
