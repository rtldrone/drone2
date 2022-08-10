cc_binary(
    name = "lv_box_neo3_1",
    srcs = ["lv_box_neo3_1_main.cc"],
    deps = [
        "//hardware_nodes:stacklight_node_neo3",
        "@ros2_rclcpp//:rclcpp",
    ],
)

cc_binary(
    name = "ultrasonic_tester",
    srcs = ["ultrasonic_tester.cc"],
    deps = [
        "//hardware_nodes:maxbotix_ultrasonic_node",
        "@ros2_rclcpp//:rclcpp",
    ],
)

cc_binary(
    name = "hv_box_neo3",
    srcs = ["hv_box_neo3_main.cc"],
    deps = [
        "@ros2_rclcpp//:rclcpp",
        "//third_party/vesc/vesc_driver"
    ]
)