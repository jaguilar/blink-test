#!/bin/bash
set -e

# Define paths
BUILD_DIR="build"
SIM_DIR="sim"
ELF_FILE="$BUILD_DIR/tests/blink-tests.elf"
RESC_FILE="$SIM_DIR/test_runner.resc"

# 1. Build the tests
echo "Building blink-tests..."
if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    cmake ..
    cd ..
fi

make -C "$BUILD_DIR" blink-tests

# 2. Run the headless simulation
echo "Running tests in Renode..."
python3 "$SIM_DIR/run_headless.py" "$RESC_FILE" "$ELF_FILE"
