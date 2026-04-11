# Tests CMake file

# Fetch CppUTest
include(FetchContent)
FetchContent_Declare(
  CppUTest
  GIT_REPOSITORY https://github.com/cpputest/cpputest.git
  GIT_TAG        latest-passing-build
)

# CppUTest configuration for embedded
set(CPPUTEST_PLATFORM Gcc CACHE STRING "Platform implementation" FORCE)
set(CPPUTEST_EXTENSIONS ON CACHE BOOL "Enable CppUTest extensions" FORCE)
set(CPPUTEST_STD_CPP_LIB_DISABLED OFF CACHE BOOL "Disable standard C++ library for embedded" FORCE)
set(CPPUTEST_MEM_LEAK_DETECTION_DISABLED ON CACHE BOOL "Disable memory leak detection" FORCE)

# Force disable host-only features that break the bare-metal toolchain
set(CMAKE_DISABLE_FIND_PACKAGE_Threads ON CACHE BOOL "Disable thread package discovery for CppUTest" FORCE)
set(CMAKE_USE_PTHREADS_INIT OFF CACHE BOOL "Disable pthread linkage for CppUTest" FORCE)
set(CMAKE_THREAD_LIBS_INIT "" CACHE STRING "No thread library for bare-metal tests" FORCE)
set(CPPUTEST_HAVE_PTHREAD_MUTEX_LOCK OFF CACHE INTERNAL "Forced disable" FORCE)
set(CPPUTEST_HAVE_GETTIMEOFDAY OFF CACHE INTERNAL "Forced disable" FORCE)
set(CPPUTEST_HAVE_FORK OFF CACHE INTERNAL "Forced disable" FORCE)
set(CPPUTEST_HAVE_WAITPID OFF CACHE INTERNAL "Forced disable" FORCE)
set(CPPUTEST_HAVE_KILL OFF CACHE INTERNAL "Forced disable" FORCE)

add_compile_definitions(
    CPPUTEST_MEM_LEAK_DETECTION_DISABLED
    CPPUTEST_USE_STD_CPP_LIB=1
    CPPUTEST_USE_STD_C_LIB=1
)

FetchContent_MakeAvailable(CppUTest)

add_executable(blink-tests
    main_test.cc
    test_dummy.cc
    test_timers.cc
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
    ../Core/Src/uart_dma.cc
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
    lwrb
)

# Provide basic syscalls (nosys) to resolve _read, _write, _close, etc.
target_link_options(blink-tests PRIVATE
    --specs=nosys.specs
)

# CTest integration needs a command to run the simulation
# [HIL TEST PLACEHOLDER] - To be implemented
