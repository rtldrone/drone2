cc_library(
    name = "websocketpp",
    hdrs = glob(["websocketpp/**/*.hpp"]),
    includes = ["."],
    visibility = ["//visibility:public"],
    local_defines = ["ASIO_STANDALONE"],
    deps = ["@asio//:asio"]
)
