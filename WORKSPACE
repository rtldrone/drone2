load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

BAZEL_TOOLCHAIN_TAG = "0.7.1"

BAZEL_TOOLCHAIN_SHA = "97853d0b2a725f9eb3f5c2cc922e86a69afb35a01b52a69b4f864eaf9f3c4f40"

http_archive(
    name = "com_grail_bazel_toolchain",
    canonical_id = BAZEL_TOOLCHAIN_TAG,
    sha256 = BAZEL_TOOLCHAIN_SHA,
    strip_prefix = "bazel-toolchain-{tag}".format(tag = BAZEL_TOOLCHAIN_TAG),
    url = "https://github.com/grailbio/bazel-toolchain/archive/{tag}.tar.gz".format(tag = BAZEL_TOOLCHAIN_TAG),
)

load("@com_grail_bazel_toolchain//toolchain:deps.bzl", "bazel_toolchain_dependencies")

bazel_toolchain_dependencies()

load("@com_grail_bazel_toolchain//toolchain:rules.bzl", "llvm_toolchain")

llvm_toolchain(
    name = "llvm_toolchain",
    llvm_version = "13.0.0",
    #    sysroot = {
    #        "linux-aarch64": "@arm64_debian_rootfs//:sysroot_files",
    #    },
)

load("@llvm_toolchain//:toolchains.bzl", "llvm_register_toolchains")

llvm_register_toolchains()

ROS2BAZEL_COMMIT = "7d844216347017cab7c6d7b6bb3bfeb1c90f9b75"

http_archive(
    name = "com_github_camearle20_ros2bazel",
    sha256 = "26208ffc2bfa54f14bdf02d60dc9e56f76caeca9b20c7a3e1d359144a8563176",
    strip_prefix = "ros2bazel-%s" % ROS2BAZEL_COMMIT,
    urls = ["https://github.com/camearle20/ros2bazel/archive/%s.tar.gz" % ROS2BAZEL_COMMIT],
)

load("@com_github_camearle20_ros2bazel//ros2:repositories.bzl", "ros2bazel_repositories")

ros2bazel_repositories()

load("@com_github_camearle20_ros2bazel//ros2:deps.bzl", "ros2bazel_dependencies")

ros2bazel_dependencies()

load("@ros2bazel_pip_deps//:requirements.bzl", "install_deps")

install_deps()

UDP_MSGS_VERSION = "0.0.3"

TRANSPORT_DRIVERS_VERSION = "1.1.1"

http_archive(
    name = "asio",
    build_file = "@//:repository/asio.BUILD",
    sha256 = "dcba16f3ed88a546f61c98ae5584f198f9cb8f4d0c72d704555760e9a7ff79d7",
    strip_prefix = "asio-1.22.1",
    urls = ["https://downloads.sourceforge.net/project/asio/asio/1.22.1%20%28Stable%29/asio-1.22.1.tar.gz?ts=gAAAAABiq5x0vH8MSVMffrPqF2m2R7RqbJtnrCu1th8rdZl1mVMguIBqyy7jUdODDltT49Of2XEVv0XVz0yQvBg5xFc4wJ4LUg%3D%3D&r=https%3A%2F%2Fsourceforge.net%2Fprojects%2Fasio%2Ffiles%2Fasio%2F1.22.1%2520%2528Stable%2529%2Fasio-1.22.1.tar.gz%2Fdownload"],
)

http_archive(
    name = "udp_msgs",
    build_file = "@//:repository/udp_msgs.BUILD",
    sha256 = "72b26fa3f26c500b85c7c6451273fcfdee95574de04479dfafd374f919c577c9",
    strip_prefix = "udp_msgs-%s" % UDP_MSGS_VERSION,
    urls = ["https://github.com/flynneva/udp_msgs/archive/refs/tags/%s.tar.gz" % UDP_MSGS_VERSION],
)

http_archive(
    name = "transport_drivers",
    build_file = "@//:repository/transport_drivers.BUILD",
    sha256 = "7217957a28efb76da31c0f5e19641ef377727bd253cf697f79b3d1220f258304",
    strip_prefix = "transport_drivers-%s" % TRANSPORT_DRIVERS_VERSION,
    urls = ["https://github.com/ros-drivers/transport_drivers/archive/refs/tags/%s.tar.gz" % TRANSPORT_DRIVERS_VERSION],
)
