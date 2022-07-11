cc_library(
    name = "gpiod",
    hdrs = ["include/gpiod.h", "@//third_party/libgpiod:config_header"],
    includes = ["include"],
    tags = ["no-ide"],
    copts = ["-include third_party/libgpiod/config.h"],
    srcs = ["lib/core.c", "lib/ctxless.c", "lib/helpers.c", "lib/iter.c", "lib/misc.c"],
    visibility = ["//visibility:public"],
)
