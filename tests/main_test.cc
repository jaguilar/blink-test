#include <unistd.h>
#include <wchar.h>

#include <cstdio>
#include <span>
#include <vector>

#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"
#include "cmsis_os2.h"
#include "foc_types.h"

extern "C" {
    // STM32 HAL / CMSIS includes if needed
#include "gpio.h"
#include "main.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;

// Stubs for newlib-nano missing wide character functions that GTest/CppUTest might link against
#ifndef putwc
wint_t putwc(wchar_t, FILE*) { return 0; }
#endif
#ifndef getwc
wint_t getwc(FILE*) { return 0; }
#endif
#ifndef ungetwc
wint_t ungetwc(wint_t, FILE*) { return 0; }
#endif

    // GoogleTest filepath uses mkdir
    int mkdir(const char *path, mode_t mode) { return -1; }

    // GoogleTest uses gettimeofday for test timing. 
    // We implement it using the SysTick timer which is standard on Cortex-M.
    int _gettimeofday(struct timeval *tv, void *tzvp) {
        if (tv) {
            static uint32_t last_val = 0;
            static uint64_t total_cycles = 0;
            uint32_t current_val = SysTick->VAL;
            uint32_t load = SysTick->LOAD;
            
            if (last_val == 0) last_val = load;
            
            if (current_val <= last_val) {
                total_cycles += (last_val - current_val);
            } else {
                // Handle wrap
                total_cycles += (last_val + (load - current_val));
            }
            last_val = current_val;

            // SystemCoreClock should be 160MHz after SystemClock_Config()
            uint32_t freq = SystemCoreClock;
            if (freq == 0) freq = 160000000;  // Fallback
            tv->tv_sec = total_cycles / freq;
            tv->tv_usec = (total_cycles % freq) * 1000000 / freq;
        }
        return 0;
    }
}





#include <string.h>

extern "C" {

void Setup() {
  // Initialization of HAL and MX_..._Init is handled by main.c.
  // We explicitly enable the UART here to ensure printf works.
  printf("\n\nUART enabled (Printf)\n");

  // Enable DWT Cycle Counter for high-precision timing
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Loop() {
  // Now that peripherals are initialized and we are in a task,
  // we can start the tests.

  // Disable stdout buffering
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);

  printf("\n\nStarting blink-tests in FreeRTOS task...\n");

  // Handshake with test runner
  printf("READY_FOR_TESTS\n");

  int argc = 0;
  char* argv[16];
  static char program_name[] = "blink-tests";
  argv[argc++] = program_name;
#if 0
char cmd_buf[256];
  read(STDIN_FILENO, cmd_buf, sizeof(cmd_buf) - 1);

  if (strncmp(cmd_buf, "START_TESTS ", 12) == 0) {
    char* args_str = cmd_buf + 12;
    char* token = strtok(args_str, " ");
    while (token != NULL && argc < 15) {
      argv[argc++] = token;
      token = strtok(NULL, " ");
    }
  } else if (strcmp(cmd_buf, "START_TESTS") == 0) {
    // No arguments, just run all (or default)
    static char verbose[] = "-v";
    argv[argc++] = verbose;
  }
#endif

  argv[argc] = NULL;

  printf("Running tests with %d arguments...\n", argc);
  for (int i = 0; i < argc; i++) {
    printf("  argv[%d]: %s\n", i, argv[i]);
  }

  int result = CommandLineTestRunner::RunAllTests(argc, argv);
  printf("Tests finished with result: %d\n", result);
  printf("test suites ran.\n");

  // We can use a Renode semi-hosting or equivalent to exit if needed.
  // For now, just idle.
  while (1) {
    osDelay(1000);
  }
}
}
