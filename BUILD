load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

exports_files([
    "STM32G474XX_FLASH.ld",
])

cc_library(
    name = "stm32cubemx_headers",
    hdrs = glob([
        "Core/Inc/**/*.h",
        "Drivers/STM32G4xx_HAL_Driver/Inc/**/*.h",
        "Drivers/STM32G4xx_HAL_Driver/Inc/Legacy/**/*.h",
        "Middlewares/Third_Party/FreeRTOS/Source/include/**/*.h",
        "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/**/*.h",
        "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/**/*.h",
        "Drivers/CMSIS/Device/ST/STM32G4xx/Include/**/*.h",
        "Drivers/CMSIS/Include/**/*.h",
        "Drivers/CMSIS/DSP/Include/**/*.h",
        "src/**/*.h",
    ]),
    includes = [
        "Core/Inc",
        "Drivers/STM32G4xx_HAL_Driver/Inc",
        "Drivers/STM32G4xx_HAL_Driver/Inc/Legacy",
        "Middlewares/Third_Party/FreeRTOS/Source/include",
        "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
        "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F",
        "Drivers/CMSIS/Device/ST/STM32G4xx/Include",
        "Drivers/CMSIS/Include",
        "Drivers/CMSIS/DSP/Include",
        "src",
    ],
    defines = [
        "USE_FULL_LL_DRIVER",
        "USE_HAL_DRIVER",
        "STM32G474xx",
    ],
)

cc_library(
    name = "stm32_drivers",
    srcs = [
        "Core/Src/system_stm32g4xx.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_utils.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_exti.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_gpio.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_dma.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_rcc.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_pwr.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cordic.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_spi.c",
        "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_tim.c",
    ],
    deps = [":stm32cubemx_headers"],
)

cc_library(
    name = "freertos",
    srcs = [
        "Middlewares/Third_Party/FreeRTOS/Source/croutine.c",
        "Middlewares/Third_Party/FreeRTOS/Source/event_groups.c",
        "Middlewares/Third_Party/FreeRTOS/Source/list.c",
        "Middlewares/Third_Party/FreeRTOS/Source/queue.c",
        "Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c",
        "Middlewares/Third_Party/FreeRTOS/Source/tasks.c",
        "Middlewares/Third_Party/FreeRTOS/Source/timers.c",
        "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c",
        "Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c",
        "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c",
    ],
    deps = [":stm32cubemx_headers"],
)

cc_library(
    name = "cube_app",
    srcs = [
        "Core/Src/main.c",
        "Core/Src/gpio.c",
        "Core/Src/app_freertos.c",
        "Core/Src/adc.c",
        "Core/Src/cordic.c",
        "Core/Src/dma.c",
        "Core/Src/i2c.c",
        "Core/Src/usart.c",
        "Core/Src/spi.c",
        "Core/Src/tim.c",
        "Core/Src/stm32g4xx_it.c",
        "Core/Src/stm32g4xx_hal_msp.c",
        "Core/Src/stm32g4xx_hal_timebase_tim.c",
        "Core/Src/sysmem.c",
        "Core/Src/syscalls.c",
        "startup_stm32g474xx.s",
    ],
    deps = [
        ":stm32_drivers",
        ":freertos",
    ],
    alwayslink = True,
)


cc_library(
    name = "cube_common_lib",
    srcs = [
        "src/foc_types.cc",
        "src/stm32_motor_driver.cc",
        "src/as5048a_spi_sensor.cc",
        "src/stm32_adc_current_sense.cc",
        "src/uart_dma.cc",
        "src/mpu6050.cc",
    ],
    deps = [
        ":cube_app",
        "//lib:arduino_foc_drivers",
        "@eigen//:eigen",
        "//lib:arduino_foc",
        "//lib:lwrb",
        "//lib:stm32_arduino_shim",
    ],
    copts = [
        "-Wdouble-promotion",
    ],
    defines = [
        "HAL_CORDIC_MODULE_ENABLED",
    ],
    alwayslink = True,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "cube_lib",
    srcs = [
        "src/app.cc",
    ],
    deps = [
        ":cube_common_lib",
    ],
    alwayslink = True,
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "cube",
    deps = [
        ":cube_lib",
    ],
)

load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")

refresh_compile_commands(
    name = "refresh_compile_commands",
    targets = {
        "//:cube": "",
        "//tests/...": "",
    },
)
