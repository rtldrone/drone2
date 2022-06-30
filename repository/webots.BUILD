cc_library(
    name = "plugins",
    hdrs = glob(["include/plugins/**/*.h"]),
    includes = ["include"],
)

cc_library(
    name = "controller",
    srcs = glob(["src/controller/c/**/*.c", "src/controller/c/**/*.h"]),
    hdrs = glob(["include/controller/c/webots/**/*.h"]),
    includes = ["include/controller/c"],
    deps = ["@stb//:stb", ":plugins"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "controller_cpp",
    srcs = glob(["src/controller/cpp/**/*.cpp"]),
    hdrs = glob(["include/controller/cpp/webots/**/*.hpp"]),
    includes = ["include/controller/cpp"],
    deps = [":controller"],
    visibility = ["//visibility:public"],
)
