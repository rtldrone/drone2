load("@com_github_camearle20_ros2bazel//ros2/rules:ros2_interfaces.bzl", "ros2_all_interface_libraries")

ros2_all_interface_libraries(
    name = "stacklight_state_msg",
    srcs = ["msg/StacklightState.msg"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "stacklight_node_neo3",
    srcs = ["neo_3_stacklight_node.cc"],
    hdrs = ["neo_3_stacklight_node.h"],
    visibility = ["//visibility:public"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":stacklight_state_msg_cpp",
        "@gpiod",
        "@ros2_rclcpp//:rclcpp",
    ],
)

cc_library(
    name = "safety_node_neo3",
    srcs = ["neo_3_safety_node.cc"],
    hdrs = ["neo_3_safety_node.h"],
    visibility = ["//visibility:public"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@ros2_rclcpp//:rclcpp",
        "@ros2_common_interfaces//:std_srvs_cpp"
    ],
)

cc_library(
    name = "maxbotix_ultrasonic_node",
    srcs = ["maxbotix_ultrasonic_node.cc"],
    hdrs = ["maxbotix_ultrasonic_node.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@asio",
        "@ros2_common_interfaces//:sensor_msgs_cpp",
        "@ros2_rclcpp//:rclcpp",
    ],
)
