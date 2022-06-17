load("@com_github_camearle20_ros2bazel//ros2/rules:ros2_interfaces.bzl", "ros2_all_interface_libraries")

ros2_all_interface_libraries(
    name = "udp_msgs",
    srcs = [
        "msg/UdpPacket.msg",
        "srv/UdpSend.srv",
        "srv/UdpSocket.srv",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@ros2_common_interfaces//:std_msgs",
    ],
)
