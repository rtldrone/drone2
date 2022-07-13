cc_binary(
    name = "talker",
    srcs = ["talker.cc"],
    deps = [
        "@ros2_common_interfaces//:std_msgs_cpp",
        "@ros2_rclcpp//:rclcpp",
        "@udp_msgs//:udp_msgs_cpp",
        "//peripherals:stacklight_state_msg_cpp"
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
    linkopts = [
        "-lrt",
        "-ldl",
    ],
    deps = ["@webots//:controller_cpp"],
)


cc_binary(
    name = "test",
    srcs = ["test.cpp"],
    deps = ["@gpiod//:gpiod"]

)