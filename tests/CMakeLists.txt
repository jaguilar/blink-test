# Tests CMake file

# Fetch GoogleTest
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.12.1
)
# Disable pthreads for baremetal
set(gtest_disable_pthreads ON CACHE BOOL "" FORCE)

# Often embedded systems don't have rtti or exceptions. 
# Depending on your main project, these might be needed:
# add_compile_options(-fno-exceptions -fno-rtti)
# add_compile_definitions(GTEST_HAS_EXCEPTIONS=0)

add_compile_definitions(
    GTEST_HAS_POSIX_RE=0
    GTEST_HAS_FILE_SYSTEM=0
    GTEST_HAS_DEATH_TEST=0
    GTEST_HAS_STREAM_REDIRECTION=0
    _POSIX_PATH_MAX=256
    _POSIX_C_SOURCE=200809L
    _DEFAULT_SOURCE=1
    _GNU_SOURCE=1
)

FetchContent_MakeAvailable(googletest)

add_executable(blink-tests
    main_test.cc
    ../startup_stm32g474xx.s
    # Include other source files you want to test
    ../Core/Src/foc_types.cc
)

# Pull in the same libraries as the main application to be able to test them
target_link_libraries(blink-tests
    stm32cubemx
    m
    FreeRTOS
    Arduino
    Arduino-FOC
    gtest
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
