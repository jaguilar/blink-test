load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "cube-tests",
    srcs = [
        "main_test.cc",
        "test_dummy.cc",
        "test_timers.cc",
    ],
    deps = [
        "//:cube_common_lib",
        "@cpputest//:cpputest",
        "@cpputest//:cpputest_ext",
    ],
)

cc_binary(
    name = "spi-sensor-test",
    srcs = [
        "main_test.cc",
        "test_spi_sensor.cc",
    ],
    deps = [
        "//:cube_common_lib",
        "@cpputest//:cpputest",
        "@cpputest//:cpputest_ext",
    ],
)

cc_binary(
    name = "adc-current-sense-test",
    srcs = [
        "main_test.cc",
        "test_adc_current_sense.cc",
    ],
    deps = [
        "//:cube_common_lib",
        "@cpputest//:cpputest",
        "@cpputest//:cpputest_ext",
    ],
)

cc_binary(
    name = "mpu6050-test",
    srcs = [
        "main_test.cc",
        "test_mpu6050.cc",
    ],
    deps = [
        "//:cube_common_lib",
        "@cpputest//:cpputest",
        "@cpputest//:cpputest_ext",
    ],
)

cc_binary(
    name = "kalman-filter-test",
    srcs = [
        "main_test.cc",
        "test_kalman_filter.cc",
    ],
    deps = [
        "//:cube_common_lib",
        "@cpputest//:cpputest",
        "@cpputest//:cpputest_ext",
    ],
)
