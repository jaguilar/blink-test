#!/bin/bash
set -e

# Change to the project root directory
cd "$(dirname "$0")/.."

# --- Colors ---
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# --- Configuration ---
if [ -f local.env ]; then
    set -a
    source local.env
    set +a
fi

BUILD_DIR="${BUILD_DIR:-build}"
TARGET="blink-tests"
UART_DEV="${UART_PORT:-/dev/ttyUSB0}"

show_help() {
    echo "Usage: $0 [options] [cmake_args...]"
    echo "Options:"
    echo "  --hil        Build, reset device, and start UART monitor (default)"
    echo "  --target T   Set the test binary target (default: blink-tests)"
    echo "  --build      Build only"
    echo "  --flash      Build and reset device (no monitor)"
    echo "  --monitor    Start UART monitor only (no build/flash)"
    echo "  -h, --help   Show this help"
    echo ""
    echo "Note: Extra arguments are forwarded to CMake configuration."
    echo "Example: $0 --hil -DCMAKE_BUILD_TYPE=Debug"
}

# --- Argument Parsing ---
MODE="hil"
CMAKE_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --hil)     MODE="hil"; shift ;;
        --target)  TARGET="$2"; shift 2 ;;
        --build)   MODE="build"; shift ;;
        --flash)   MODE="flash"; shift ;;
        --monitor) MODE="monitor"; shift ;;
        -h|--help) show_help; exit 0 ;;
        *)         CMAKE_ARGS+=("$1"); shift ;;
    esac
done

ELF_FILE="$BUILD_DIR/tests/$TARGET.elf"

# --- 1. Build Phase ---
if [[ "$MODE" != "monitor" ]]; then
    echo -e "${BLUE}=== Configuring blink-tests ===${NC}"
    cmake -G Ninja -B "$BUILD_DIR" -S . \
        -DBUILD_TESTING=ON \
        -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
        "${CMAKE_ARGS[@]}"

    echo -e "${BLUE}=== Building $TARGET ===${NC}"
    cmake --build "$BUILD_DIR" --target "$TARGET"
fi

# --- 2. Hardware Phase ---
if [[ "$MODE" == "hil" || "$MODE" == "flash" ]]; then
    if [[ ! -f "$ELF_FILE" ]]; then
        echo -e "${YELLOW}Error: Test binary not found at $ELF_FILE${NC}"
        exit 1
    fi
    
    pkill -f openocd || true
    echo -e "${GREEN}=== Flashing $TARGET and Resetting device via SWD ===${NC}"
    openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program $ELF_FILE verify reset exit"
fi

# --- 3. Monitor Phase ---
if [[ "$MODE" == "hil" || "$MODE" == "monitor" ]]; then
    echo -e "${GREEN}=== Starting UART Monitor on $UART_DEV ===${NC}"
    python3 scripts/monitor_uart.py
fi
