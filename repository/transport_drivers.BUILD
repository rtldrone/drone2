cc_library(
    name = "io_context",
    srcs = glob(["io_context/src/**/*.cpp"]),
    hdrs = glob(["io_context/include/**/*.hpp"]),
    includes = ["io_context/include"],
    visibility = ["//visibility:public"],
    deps = [
        "@asio//:asio",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_common_interfaces//:std_msgs_cpp",
        "@udp_msgs//:udp_msgs_cpp",
    ],
)

# This intentionally excludes serial_bridge_node because we don't need it
cc_library(
    name = "serial_driver",
    hdrs = [
        "serial_driver/include/serial_driver/serial_driver.hpp",
        "serial_driver/include/serial_driver/serial_port.hpp",
        "serial_driver/include/serial_driver/visibility_control.hpp",
    ],
    srcs = [
        "serial_driver/src/serial_driver.cpp",
        "serial_driver/src/serial_port.cpp",
    ],
    includes = ["serial_driver/include"],
    visibility = ["//visibility:public"],
    deps = [
        "@asio//:asio",
        ":io_context",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rclcpp//:rclcpp_lifecycle",
        "@ros2_common_interfaces//:std_msgs_cpp",
    ],
)
