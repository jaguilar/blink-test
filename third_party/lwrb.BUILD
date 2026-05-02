load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "lwrb",
    srcs = [
        "lwrb/src/lwrb/lwrb.c",
        "lwrb/src/lwrb/lwrb_ex.c",
    ],
    hdrs = glob(["lwrb/src/include/**/*.h"]),
    includes = ["lwrb/src/include"],
    visibility = ["//visibility:public"],
)
