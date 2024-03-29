load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "arm64_debian_rootfs",
    build_file = "@//:repository/debian_rootfs.BUILD",
    sha256 = "7e6ad432fec0a36f8b66c3fc2ab8795ea446e61f7dce7a206b55602677cf0904",
    url = "https://www.frc971.org/Build-Dependencies/2021-10-30-raspios-bullseye-arm64-lite_rootfs.tar.bz2",
)

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
    llvm_version = "14.0.0",
)

load("@llvm_toolchain//:toolchains.bzl", "llvm_register_toolchains")

llvm_register_toolchains()

llvm_toolchain(
    name = "llvm_aarch64",
    llvm_version = "14.0.0",
    sysroot = {
        "linux-aarch64": "@arm64_debian_rootfs//:sysroot_files",
    },
    toolchain_roots = {
        "": "@llvm_toolchain_llvm//",
    },
)

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

STB_VERSION = "f67165c2bb2af3060ecae7d20d6f731173485ad0"

WEBOTS_VERSION = "R2022a"

LIBGPIOD_VERSION = "1.6.3"

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

http_archive(
    name = "stb",
    build_file = "@//:repository/stb.BUILD",
    sha256 = "1d1035301f384c8eb1058adf39e233d2eba362e9c3a018e992d2bdd351bce846",
    strip_prefix = "stb-%s" % STB_VERSION,
    urls = ["https://github.com/nothings/stb/archive/%s.zip" % STB_VERSION],
)

http_archive(
    name = "webots",
    build_file = "@//:repository/webots.BUILD",
    sha256 = "535eff456eb24d58cbd23201ffcf8c640b6512b0e64fd19ef059bf210725eef3",
    strip_prefix = "webots-%s" % WEBOTS_VERSION,
    urls = ["https://github.com/cyberbotics/webots/archive/refs/tags/%s.tar.gz" % WEBOTS_VERSION],
)

http_archive(
    name = "gpiod",
    build_file = "@//:repository/gpiod.BUILD",
    sha256 = "eb446070be1444fd7d32d32bbca53c2f3bbb0a21193db86198cf6050b7a28441",
    strip_prefix = "libgpiod-%s" % LIBGPIOD_VERSION,
    urls = ["https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git/snapshot/libgpiod-%s.tar.gz" % LIBGPIOD_VERSION],
)

http_archive(
    name = "websocketpp",
    build_file = "@//:repository/websocketpp.BUILD",
    sha256 = "0281b1d1278f01138805279fd2277901ae3c71b76e28364575356061098111c6",
    strip_prefix = "websocketpp-0.8.2",
    urls = [
        "https://github.com/zaphoyd/websocketpp/archive/refs/tags/0.8.2.zip"
    ],
)


# Hedron's Compile Commands Extractor for Bazel
# https://github.com/hedronvision/bazel-compile-commands-extractor
http_archive(
    name = "hedron_compile_commands",
    sha256 = "021e3c42b3df88702caf3299d33e2a6f2e84da9e7f24707a0cf5815feba2d4a2",
    strip_prefix = "bazel-compile-commands-extractor-4bb5e4afc049117154e8fb2a7c71b9eaf4db7547",
    url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/4bb5e4afc049117154e8fb2a7c71b9eaf4db7547.tar.gz",
)

load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")

hedron_compile_commands_setup()
