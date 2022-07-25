load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")

refresh_compile_commands(
    name = "refresh_compile_commands",

    # Specify the targets of interest.
    # For example, specify a dict of targets and any flags required to build.
    targets = {
      "//...": "",
      "//compositions/...": "--config pi",
    },
    # No need to add flags already in .bazelrc. They're automatically picked up.
    # If you don't need flags, a list of targets is also okay, as is a single target string.
    # Wildcard patterns, like //... for everything, *are* allowed here, just like a build.
      # As are additional targets (+) and subtractions (-), like in bazel query https://docs.bazel.build/versions/main/query.html#expressions
    # And if you're working on a header-only library, specify a test or binary target that compiles it.
)

cc_binary(
    name = "talker",
    srcs = ["talker.cc"],
    deps = [
        "//hardware_nodes:stacklight_state_msg_cpp",
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
    linkopts = [
        "-lrt",
        "-ldl",
    ],
    deps = ["@webots//:controller_cpp"],
)

cc_binary(
    name = "test",
    srcs = ["test.cpp"],
    deps = ["@gpiod"],
)
