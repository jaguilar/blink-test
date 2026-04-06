# Tests CMake file

# Fetch CppUTest
include(FetchContent)
FetchContent_Declare(
  CppUTest
  GIT_REPOSITORY https://github.com/cpputest/cpputest.git
  GIT_TAG        v4.0
)

# CppUTest configuration for embedded
set(CPPUTEST_PLATFORM Standard CACHE STRING "Platform implementation" FORCE)
set(EXTENSIONS ON CACHE BOOL "Enable CppUTest extensions" FORCE)
set(STD_C ON CACHE BOOL "Enable standard C library" FORCE)
set(STD_CPP OFF CACHE BOOL "Disable standard C++ library for embedded" FORCE)
set(MEMORY_LEAK_DETECTION OFF CACHE BOOL "Disable memory leak detection" FORCE)

# Force disable features that cause build errors in this environment
set(HAVE_PTHREAD_MUTEX_LOCK OFF CACHE INTERNAL "Forced disable")
set(HAVE_GETTIMEOFDAY OFF CACHE INTERNAL "Forced disable")
set(HAVE_FORK OFF CACHE INTERNAL "Forced disable")
set(HAVE_EXCEPTIONS OFF CACHE INTERNAL "Forced disable")

add_compile_definitions(
    CPPUTEST_MEM_LEAK_DETECTION_DISABLED
    CPPUTEST_USE_STD_CPP_LIB=0
    CPPUTEST_USE_STD_C_LIB=1
)

FetchContent_MakeAvailable(CppUTest)

add_executable(blink-tests
    main_test.cc
    ../Core/Src/main.c
    ../Core/Src/app_freertos.c
    ../Core/Src/gpio.c
    ../Core/Src/usart.c
    ../Core/Src/adc.c
    ../Core/Src/dma.c
    ../Core/Src/spi.c
    ../Core/Src/tim.c
    ../Core/Src/stm32g4xx_it.c
    ../Core/Src/stm32g4xx_hal_msp.c
    ../Core/Src/stm32g4xx_hal_timebase_tim.c
    ../Core/Src/sysmem.c
    ../Core/Src/syscalls.c
    ../startup_stm32g474xx.s
    # Include other source files you want to test
    ../Core/Src/foc_types.cc
)

# Pull in the same libraries as the main application to be able to test them
target_link_libraries(blink-tests
    stm32cubemx
    STM32_Drivers
    m
    FreeRTOS
    Arduino
    Arduino-FOC
    CppUTest
    CppUTestExt
)

# Provide basic syscalls (nosys) to resolve _read, _write, _close, etc.
target_link_options(blink-tests PRIVATE
    --specs=nosys.specs
)

# CTest integration needs a command to run the simulation
# Here we add a test that runs Renode headlessly
add_test(NAME GTest_Renode_Sim
    COMMAND python3 ${CMAKE_SOURCE_DIR}/sim/run_headless.py ${CMAKE_SOURCE_DIR}/sim/test_runner.resc $<TARGET_FILE:blink-tests>
)
