load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "arduino_foc",
    srcs = [
        "Arduino-FOC/src/BLDCMotor.cpp",
        "Arduino-FOC/src/HybridStepperMotor.cpp",
        "Arduino-FOC/src/StepperMotor.cpp",
        "Arduino-FOC/src/common/foc_utils.cpp",
        "Arduino-FOC/src/common/lowpass_filter.cpp",
        "Arduino-FOC/src/common/pid.cpp",
        "Arduino-FOC/src/common/time_utils.cpp",
        "Arduino-FOC/src/common/base_classes/CurrentSense.cpp",
        "Arduino-FOC/src/common/base_classes/FOCMotor.cpp",
        "Arduino-FOC/src/common/base_classes/Sensor.cpp",
        "Arduino-FOC/src/communication/Commander.cpp",
        "Arduino-FOC/src/communication/SimpleFOCDebug.cpp",
        "Arduino-FOC/src/current_sense/LowsideCurrentSense.cpp",
        "Arduino-FOC/src/sensors/GenericSensor.cpp",
    ],
    hdrs = glob(["Arduino-FOC/src/**/*.h"]),
    includes = ["Arduino-FOC/src"],
    deps = [":stm32_arduino_shim"],
)

cc_library(
    name = "lwrb",
    srcs = [
        "lwrb/lwrb/src/lwrb/lwrb.c",
        "lwrb/lwrb/src/lwrb/lwrb_ex.c",
    ],
    hdrs = glob(["lwrb/lwrb/src/include/**/*.h"]),
    includes = ["lwrb/lwrb/src/include"],
)

cc_library(
    name = "stm32_arduino_shim",
    srcs = ["stm32_arduino_shim/src/Arduino.cc"],
    hdrs = glob(["stm32_arduino_shim/src/**/*.h"]),
    includes = ["stm32_arduino_shim/src"],
    deps = [
        "//:stm32cubemx_headers",
        "//:freertos",
    ],
)

cc_library(
    name = "arduino_foc_drivers",
    srcs = ["Arduino-FOC-drivers/src/utilities/stm32math/STM32G4CORDICTrigFunctions.cpp"],
    hdrs = glob(["Arduino-FOC-drivers/src/utilities/stm32math/*.h"]),
    includes = ["Arduino-FOC-drivers/src/utilities/stm32math"],
    deps = ["//:stm32cubemx_headers"],
)
