#!/bin/bash
set -e

# Define paths
BUILD_DIR="build"
SIM_DIR="sim"
ELF_FILE="$BUILD_DIR/tests/blink-tests.elf"
RESC_FILE="$SIM_DIR/test_runner.resc"

# Help message
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    echo "Usage: $0 [cmake_args...]"
    echo "Example: $0 -DCMAKE_BUILD_TYPE=Debug"
    exit 0
fi

# 1. Build the tests
echo "Configuring blink-tests..."
# Always run cmake configuration to ensure BUILD_TESTING=ON and apply any passed arguments.
# Using -B and -S is more robust than manual directory creation and cd.
# We include the toolchain file to ensure cross-compilation remains correctly configured.
cmake -G Ninja -B "$BUILD_DIR" -S . \
    -DBUILD_TESTING=ON \
    -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
    "$@"

echo "Building blink-tests..."
# Use cmake --build to be generator-agnostic (works with Make or Ninja)
cmake --build "$BUILD_DIR" --target blink-tests

# 2. Run the headless simulation
echo "Running tests in Renode..."
if [[ -f "$ELF_FILE" ]]; then
    python3 "$SIM_DIR/main_test.py" "$RESC_FILE" "$ELF_FILE"
else
    echo "Error: Test binary not found at $ELF_FILE. Build might have failed or target name is different."
    exit 1
fi
