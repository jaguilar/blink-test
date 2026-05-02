load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "arduino",
    srcs = ["src/Arduino.cc"],
    hdrs = glob(["src/**/*.h"]),
    includes = ["src"],
    deps = ["@//:stm32cubemx_headers", "@//:freertos"],
    visibility = ["//visibility:public"],
)
