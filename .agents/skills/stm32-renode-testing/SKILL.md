---
name: stm32-renode-testing
description: Guidelines for STM32 peripheral simulation and GoogleTest integration via Renode.
---

# STM32 Renode Testing Skill

This skill provides comprehensive patterns for setting up and debugging bare-metal GoogleTest suites on STM32 firmware using Renode for simulation.

## Environment Overview

The testing architecture uses:
- **GoogleTest**: A C++ unit testing framework built for the `arm-none-eabi` target.
- **Renode**: An open-source simulation framework that executes the same ELF produced for the hardware.
- **CTest**: The runner that orchestrates the simulation via a headless Python script.

## Core Implementation Patterns

### 1. Bare-metal GoogleTest Compatibility

GoogleTest requires certain POSIX-like features (mkdir, wide-character support) that are often missing or incomplete in `newlib-nano`. You must provide stubs in your test runner (e.g., `main_test.cc`):

```cpp
extern "C" {
    // Redirect stdout to a simulated UART (e.g., 0x60000000)
    int _write(int file, char *ptr, int len) {
        for (int i = 0; i < len; i++) {
            *(volatile uint8_t *)0x60000000 = ptr[i];
        }
        return len;
    }

    // Wide-char stubs for GTest
    wint_t getwc(FILE *) { return 0; }
    wint_t ungetwc(wint_t, FILE *) { return 0; }
    wint_t putwc(wchar_t, FILE *) { return 0; }
    int swprintf(wchar_t *, size_t, const wchar_t *, ...) { return 0; }
}
```

### 2. Renode Platform Definition (.repl)

To test a specific peripheral (like a Timer), you must map it in the `.repl` file. Defaulting to `Memory.MappedMemory` stubs is only enough to prevent crashes, not to test logic.

**Pattern for STM32G474 TIM1**:
```repl
// Map a small memory block before the timer if needed to avoid gaps
peripherals_apb2_1: Memory.MappedMemory @ sysbus 0x40010000
    size: 0x2C00

timer1: Timers.STM32_Timer @ sysbus 0x40012C00
    frequency: 100000000 // 100MHz for fast virtual time
    initialLimit: 0xFFFF
```

### 3. Simulation Performance Optimization

Polling peripheral registers in a tight loop is extremely expensive in Renode because it constanty syncs guest/host time.

> [!TIP]
> Always add a **delay loop** inside polling `while` loops to allow virtual time to progress significantly between register reads.

```cpp
while (!(TIM1->SR & TIM_SR_UIF) && timeout > 0) {
    // Small delay to allow virtual time to progress
    for (int i = 0; i < 100; i++) {
        asm volatile("nop");
    }
    timeout--;
}
```

### 4. Real-time Log Streaming

When running headlessly, use a Python script to tail the `uart.log`. This prevents the agent and user from appearing "stalled" during long-running tests.

**Tailing logic**:
- Use `f.seek(pos)` and `f.read()` to print only new content from the log.
- Detect success/failure by scanning for `[  PASSED  ]` or `[  FAILED  ]` in the log stream.

## Troubleshooting

- **Timer not counting?** Ensure frequency is set in the `.repl` and the clock is "enabled" (even if only via a dummy RCC write).
- **HardFault on Startup?** Check if `SystemInit` or `HAL_Init` is trying to access a peripheral not mapped in Renode. Use `sysbus Tag` in the `.resc` file to ignore reads/writes to these addresses.
- **Float/Double Crashes?** Ensure the FPU is enabled in `SystemInit`:
```cpp
volatile uint32_t *CPACR = (volatile uint32_t *)0xE000ED88;
*CPACR |= (0xF << 20);
```

### 5. Advanced Timer Synchronization (Master/Slave)

Renode's standard timer models do not automatically support internal trigger (ITR) synchronization for slaving timers to each other's OC/TRGO events. You must implement this logic using **System Bus Hooks** in Python.

**Master -> Slave Trigger Pattern**:
1. Attach a `SetHookBeforePeripheralWrite` to the Master timer.
2. In the hook, detect the start bit (`CEN` in `CR1`) and calculate the trigger delay based on `CCR` and `CNT`.
3. Use `machine.ScheduleAction` to fire the trigger after the virtual time delay ($CCR - CNT$).
4. The trigger function manually sets the `CEN` bit in the Slave timer's `CR1`.

**Verified API calls for hooks**:
- `machine.GetAllNames()`: Returns all registered peripheral names (e.g., `sysbus.timers1`).
- `machine[name]`: Acts as an indexer to retrieve a peripheral object by its full name.
- `monitor.Parse(command)`: Executes a Monitor command; use this for `sysbus SetHookBeforePeripheralWrite` to avoid IronPython generic type inference issues.
- `machine.ScheduleAction(TimeInterval.FromNanoseconds(ns), callback)`: Schedules a callback in virtual time.

### 6. Unit Test Filtering (Handshake Protocol)

To run specific tests or groups without restarting the simulation with different ELF files, implement a UART-based handshake protocol. This allows the Python runner to pass command-line arguments (like `-sn` for test name or `-sg` for group) directly to the test runner in the firmware.

#### Firmware Implementation
The firmware should signal it is ready and then wait for a command.

```cpp
void Loop() {
  printf("READY_FOR_TESTS\n"); // Signal to Python runner
  char cmd_buf[256];
  ReadLine(cmd_buf, sizeof(cmd_buf)); // Implement using HAL_UART_Receive
  
  // Parse "START_TESTS <args>"
  if (strncmp(cmd_buf, "START_TESTS ", 12) == 0) {
    char* args_str = cmd_buf + 12;
    // Tokenize args_str into argc/argv
    CommandLineTestRunner::RunAllTests(argc, argv);
  }
}
```

#### Python Runner Implementation
The runner tails the UART log and injects the command when the signal is detected.

```python
if not command_sent and "READY_FOR_TESTS" in full_uart:
    # Use WriteChar loop for maximum compatibility across Renode UART models
    for char in f"START_TESTS {test_filter}\n":
        proc.stdin.write(f"sysbus.usart1 WriteChar {ord(char)}\n".encode())
    proc.stdin.flush()
    command_sent = True
```

> [!IMPORTANT]
> Renode's `AddLineHook` can be finicky with line endings. Detecting the signal in the Python tailing loop and sending the command via `proc.stdin` (Renode monitor) is often more reliable and easier to debug.
