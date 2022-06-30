cc_library(
    name = "vesc_driver",
    srcs = [
        "src/vesc_driver.cpp",
        "src/vesc_interface.cpp",
        "src/vesc_packet.cpp",
        "src/vesc_packet_factory.cpp",
    ],
    hdrs = [
        "include/vesc_driver/crc.hpp",
        "include/vesc_driver/datatypes.hpp",
        "include/vesc_driver/vesc_driver.hpp",
        "include/vesc_driver/vesc_interface.hpp",
        "include/vesc_driver/vesc_packet.hpp",
        "include/vesc_driver/vesc_packet_factory.hpp",
    ],
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
