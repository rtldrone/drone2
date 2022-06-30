cc_binary(
    name = "talker",
    srcs = ["talker.cc"],
    data = ["params_file.yaml"],
    deps = [
        "@ros2_common_interfaces//:std_msgs_cpp",
        "@ros2_rclcpp//:rclcpp",
        "@udp_msgs//:udp_msgs_cpp",
    ],
)

cc_binary(
    name = "serial_test",
    srcs = ["serial_test.cc"],
    data = ["params_file.yaml"],
    deps = [
        "//third_party/vesc/vesc_driver",
        "@ros2_rclcpp//:rclcpp",
    ],
)

cc_binary(
    name = "empty",
    srcs = ["empty.cc"],
    deps = ["@webots//:controller_cpp"],
)
