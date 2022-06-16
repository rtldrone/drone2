load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

ROS2BAZEL_COMMIT = "7d844216347017cab7c6d7b6bb3bfeb1c90f9b75"

http_archive(
    name = "com_github_camearle20_ros2bazel",
    strip_prefix = "ros2bazel-%s" % ROS2BAZEL_COMMIT,
    urls = ["https://github.com/camearle20/ros2bazel/archive/%s.tar.gz" % ROS2BAZEL_COMMIT],
)

load("@com_github_camearle20_ros2bazel//ros2:repositories.bzl", "ros2bazel_repositories")

ros2bazel_repositories()

load("@com_github_camearle20_ros2bazel//ros2:deps.bzl", "ros2bazel_dependencies")

ros2bazel_dependencies()

load("@ros2bazel_pip_deps//:requirements.bzl", "install_deps")

install_deps()

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
