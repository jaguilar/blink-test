# Skill: Autonomous STM32 Debugging

This skill defines the procedures for an AI agent to autonomously attach to and debug an STM32 microcontroller on this system.

## Tooling Overview

- **Entry Point**: `./debug_me.sh` (Root of the workspace)
- **Support Scripts**: `scripts/debug_tools.py`, `scripts/gdb_agent.py`
- **GDB Server Port**: `3334` (Specific to avoid conflicts with ESP32 debuggers)
- **Toolchain**: Primarily uses the STM32CubeIDE bundle found in `~/.local/share/stm32cube/`.

## Debugging Procedures

### 1. Basic Lifecycle Management
To ensure a clean state, always start by resetting the MCU:
```bash
./debug_me.sh reset
```

### 2. Inspecting State
To check where the program is currently executing (especially useful if the MCU is hung):
```bash
./debug_me.sh backtrace
```

To read the value of a global variable (e.g., `uwTick` or a status flag):
```bash
./debug_me.sh read <VARIABLE_NAME>
```

### 3. Verification Protocol
When asked to "verify if the firmware is running":
1.  Read `uwTick` (using `./debug_me.sh read uwTick`).
2.  Wait for 1-2 seconds.
3.  Read `uwTick` again.
4.  If the value has incremented, the firmware is alive.

## Troubleshooting

- **"Connection refused"**: The GDB server might have failed to start or crashed. `debug_me.sh` attempts to restart it, but if it fails, check `dmesg` or `lsusb` to see if the ST-LINK is disconnected.
- **"Undefined item"**: Ensure the binary being debugged (`build/blink-test.elf`) contains the symbols for the variables you are trying to read.
- **Port Conflicts**: If port 3334 is taken, check `ss -tunlp` to see what is running there.
