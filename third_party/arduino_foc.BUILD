load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "arduino_foc",
    srcs = [
        "src/BLDCMotor.cpp",
        "src/HybridStepperMotor.cpp",
        "src/StepperMotor.cpp",
        "src/common/foc_utils.cpp",
        "src/common/lowpass_filter.cpp",
        "src/common/pid.cpp",
        "src/common/time_utils.cpp",
        "src/common/base_classes/CurrentSense.cpp",
        "src/common/base_classes/FOCMotor.cpp",
        "src/common/base_classes/Sensor.cpp",
        "src/communication/Commander.cpp",
        "src/communication/SimpleFOCDebug.cpp",
        "src/current_sense/LowsideCurrentSense.cpp",
        "src/sensors/GenericSensor.cpp",
    ],
    hdrs = glob(["src/**/*.h"]),
    includes = ["src"],
    deps = ["@stm32_arduino_shim//:arduino"],
    visibility = ["//visibility:public"],
)
