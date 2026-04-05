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
