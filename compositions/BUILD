cc_binary(
    name = "lv_box_neo3_1",
    srcs = ["lv_box_neo3_1_main.cc"],
    deps = [
        "//hardware_nodes:maxbotix_ultrasonic_node",
        "@ros2_rclcpp//:rclcpp",
    ],
)

cc_binary(
    name = "lv_box_neo3_2",
    srcs = ["lv_box_neo3_2_main.cc"],
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
    name = "hv_box_neo3_vesc",
    srcs = ["hv_box_neo3_vesc_main.cc"],
    deps = [
        "//third_party/vesc/vesc_driver",
        "@ros2_rclcpp//:rclcpp",
    ],
)

cc_binary(
    name = "hv_box_neo3_controller",
    srcs = ["hv_box_neo3_controller_main.cc"],
    deps = [
        "//controller:controller_node",
        "//hardware_nodes:safety_node_neo3",
        "//hmi:hmi_node",
        "@ros2_rclcpp//:rclcpp",
    ],
)

cc_binary(
    name = "service_test",
    srcs = ["service_test.cc"],
    deps = [
        "@ros2_common_interfaces//:std_srvs_cpp",
        "@ros2_rclcpp//:rclcpp",
    ],
)
