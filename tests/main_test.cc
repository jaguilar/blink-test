#define STFOC_SIMULATION
#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"
#include "foc_types.h"
#include <span>
#include <vector>

#include <stdio.h>
#include <wchar.h>

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
int swprintf(wchar_t*, size_t, const wchar_t*, ...) { return 0; }

// Reroute stdout to USART1
int _write(int file, char* ptr, int len) {
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}

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


TEST_GROUP(DummyTest)
{
};

TEST(DummyTest, ShouldPass) {
    CHECK_EQUAL(1, 1);
}

void ExpectTimerOverflows(TIM_TypeDef* tim, uint32_t periph, const char* name) {
    printf("Testing %s clock enablement\n", name);
    LL_APB2_GRP1_EnableClock(periph);
    
    printf("Setting %s ARR and PSC\n", name);
    LL_TIM_SetAutoReload(tim, 50);
    LL_TIM_SetPrescaler(tim, 0);
    
    printf("Setting %s EGR UG\n", name);
    LL_TIM_ClearFlag_UPDATE(tim);
    LL_TIM_GenerateEvent_UPDATE(tim);
    
    printf("Clearing %s UIF and Enabling (with UIE)\n", name);
    LL_TIM_ClearFlag_UPDATE(tim);
    LL_TIM_EnableIT_UPDATE(tim); // Enable update interrupt request just in case
    LL_TIM_EnableCounter(tim);
    
    printf("Waiting for %s overflow (ARR=%u)\n", name, (unsigned int)LL_TIM_GetAutoReload(tim));
    
    // Wait for the counter to overflow and set UIF
    int timeout = 1000000;
    while (!LL_TIM_IsActiveFlag_UPDATE(tim) && timeout > 0) {
        if (timeout % 100000 == 0) {
            printf(".");
        }
        // Small delay to allow virtual time to progress between register reads
        for (int i = 0; i < 500; i++) {
            asm volatile("nop");
        }
        timeout--;
    }
    printf("\n");
    
    if (timeout == 0) {
        printf("%s Timed out! CNT=%u, SR=0x%x\n", name, (unsigned int)LL_TIM_GetCounter(tim), (unsigned int)tim->SR);
    }
    
    CHECK_TEXT(timeout > 0, name);
    CHECK_TRUE(LL_TIM_IsActiveFlag_UPDATE(tim));
}

TEST_GROUP(TimersTest)
{
};

TEST(TimersTest, Tim1Overflows) {
    ExpectTimerOverflows(TIM1, LL_APB2_GRP1_PERIPH_TIM1, "TIM1");
}

TEST(TimersTest, Tim8Overflows) {
    ExpectTimerOverflows(TIM8, LL_APB2_GRP1_PERIPH_TIM8, "TIM8");
}

TEST(TimersTest, Tim20Overflows) {
    ExpectTimerOverflows(TIM20, LL_APB2_GRP1_PERIPH_TIM20, "TIM20");
}

#if 1
TEST(TimersTest, TimerSyncTrigger) {
    // Lead timer
    printf("Configuring TIM1 (Leader)\n");
    LL_TIM_SetAutoReload(TIM1, 100000);
    LL_TIM_SetPrescaler(TIM1, 0);
    LL_TIM_OC_SetCompareCH4(TIM1, 500);
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC4REF);
    LL_TIM_SetCounter(TIM1, 0);
    
    // Slave timer
    printf("Configuring TIM8 (Slave)\n");
    LL_TIM_SetAutoReload(TIM8, 100000);
    LL_TIM_SetPrescaler(TIM8, 0);
    LL_TIM_SetSlaveMode(TIM8, LL_TIM_SLAVEMODE_TRIGGER);
    LL_TIM_SetTriggerInput(TIM8, LL_TIM_TS_ITR0);
    LL_TIM_SetCounter(TIM8, 0);
    
    printf("Starting TIM1 (Leader)...\n");
    LL_TIM_EnableCounter(TIM1);
    
    printf("Waiting for TIM8 to start...\n");
    uint32_t start_cycles = DWT->CYCCNT;
    uint32_t timeout_cycles = SystemCoreClock / 1000; // 1ms
    while (!LL_TIM_IsEnabledCounter(TIM8) && (DWT->CYCCNT - start_cycles) < timeout_cycles) {
        // Small delay to allow virtual time to progress
        for (int i = 0; i < 100; i++) {
            asm volatile("nop");
        }
    }
    uint32_t elapsed_cycles = DWT->CYCCNT - start_cycles;
    
    if (!LL_TIM_IsEnabledCounter(TIM8)) {
        printf("TIM8 failed to start after %u cycles (%u ms)! TIM1_CNT=%u, TIM8_CNT=%u, TIM8_CR1=0x%x\n", 
               (unsigned int)elapsed_cycles,
               (unsigned int)(elapsed_cycles / (SystemCoreClock / 1000)),
               (unsigned int)LL_TIM_GetCounter(TIM1), 
               (unsigned int)LL_TIM_GetCounter(TIM8),
               (unsigned int)TIM8->CR1);
    }
    CHECK_TEXT(LL_TIM_IsEnabledCounter(TIM8), "TIM8 failed to start via TRGO from TIM1 within 1ms");
    
    // Wait for some counts to accumulate in both
    for (int i = 0; i < 10000; i++) {
        asm volatile("nop");
    }
    
    uint32_t cnt1 = LL_TIM_GetCounter(TIM1);
    uint32_t cnt8 = LL_TIM_GetCounter(TIM8);
    
    LL_TIM_DisableCounter(TIM1);
    LL_TIM_DisableCounter(TIM8);
    
    printf("Counts: TIM1=%u, TIM8=%u, Diff=%d\n", (unsigned int)cnt1, (unsigned int)cnt8, (int)cnt1 - (int)cnt8);
    
    // Expected diff: 500. Allow some slack for simulation processing.
    DOUBLES_EQUAL(500, (int)cnt1 - (int)cnt8, 50);
    
    // Cleanup for other tests
    LL_TIM_SetSlaveMode(TIM8, LL_TIM_SLAVEMODE_DISABLED);
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
}

TEST(TimersTest, TimerSyncReverse) {
    printf("\n--- Starting TimerSyncReverse (TIM8 -> TIM1) ---\n");
    
    // Enable clocks
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
    
    // Master timer: TIM8
    printf("Configuring TIM8 (Master)\n");
    LL_TIM_SetAutoReload(TIM8, 100000);
    LL_TIM_SetPrescaler(TIM8, 0);
    LL_TIM_SetCounter(TIM8, 0);
    LL_TIM_SetTriggerOutput(TIM8, LL_TIM_TRGO_ENABLE);
    
    // Slave timer: TIM1
    printf("Configuring TIM1 (Slave)\n");
    LL_TIM_SetAutoReload(TIM1, 100000);
    LL_TIM_SetPrescaler(TIM1, 0);
    LL_TIM_SetCounter(TIM1, 0);
    LL_TIM_SetSlaveMode(TIM1, LL_TIM_SLAVEMODE_TRIGGER);
    LL_TIM_SetTriggerInput(TIM1, LL_TIM_TS_ITR5); // TIM8 is ITR5
    
    printf("Starting TIM8 (Leader)...\n");
    LL_TIM_EnableCounter(TIM8);
    
    printf("Waiting for TIM1 to start...\n");
    uint32_t start_cycles = DWT->CYCCNT;
    uint32_t timeout_cycles = SystemCoreClock / 1000; // 1ms
    while (!LL_TIM_IsEnabledCounter(TIM1) && (DWT->CYCCNT - start_cycles) < timeout_cycles) {
        for (int i = 0; i < 100; i++) { asm volatile("nop"); }
    }
    
    CHECK_TEXT(LL_TIM_IsEnabledCounter(TIM1), "TIM1 failed to start via TRGO from TIM8 within 1ms");
    
    LL_TIM_SetSlaveMode(TIM1, LL_TIM_SLAVEMODE_DISABLED);
    LL_TIM_SetTriggerOutput(TIM8, LL_TIM_TRGO_RESET);
}

TEST(TimersTest, TimerRepetitionAndResetTrigger) {
    printf("\n--- Starting TimerRepetitionAndResetTrigger ---\n");
    
    // Enable clocks
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);

    // Ensure both timers are stopped and in a known state
    LL_TIM_DisableCounter(TIM1);
    LL_TIM_DisableCounter(TIM8);
    LL_TIM_SetSlaveMode(TIM1, LL_TIM_SLAVEMODE_DISABLED);
    LL_TIM_SetSlaveMode(TIM8, LL_TIM_SLAVEMODE_DISABLED);
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
    LL_TIM_SetTriggerOutput(TIM8, LL_TIM_TRGO_RESET);
    LL_TIM_SetCounter(TIM1, 0);
    LL_TIM_SetCounter(TIM8, 0);

    printf("TIM8 initial: CNT=%d, EN=%d, SMCR=%08x\n", 
           (int)LL_TIM_GetCounter(TIM8), (int)LL_TIM_IsEnabledCounter(TIM8), (unsigned int)TIM8->SMCR);
    
    // Master: TIM1
    printf("Configuring TIM1 (Master), RCR=2\n");
    LL_TIM_SetAutoReload(TIM1, 10000);
    LL_TIM_SetPrescaler(TIM1, 0);
    LL_TIM_SetRepetitionCounter(TIM1, 2); // Update every 3 overflows
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET); // Default to Reset for now
    LL_TIM_GenerateEvent_UPDATE(TIM1); // Reload RCR and repetitionsLeft
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE); // Switch to Update mode for synchronization
    LL_TIM_SetCounter(TIM1, 0);

    // Slave: TIM8
    printf("Configuring TIM8 (Slave), Combined Reset + Trigger\n");
    LL_TIM_SetAutoReload(TIM8, 60000);
    LL_TIM_SetPrescaler(TIM8, 0);
    LL_TIM_SetCounter(TIM8, 0);
    // Combined Reset + Trigger is SMS=1000. 
    // In G4, SMS[3] is bit 16 of SMCR.
#ifdef LL_TIM_SLAVEMODE_COMBINED_RESET_TRIGGER
    LL_TIM_SetSlaveMode(TIM8, LL_TIM_SLAVEMODE_COMBINED_RESET_TRIGGER);
#else
    // SMS[2:0] is bits 0:2. SMS[3] is bit 16.
    // So 1000 (binary) means bit 16 set, bits 0:2 cleared.
    MODIFY_REG(TIM8->SMCR, TIM_SMCR_SMS | (1 << 16), (1 << 16));
#endif
    LL_TIM_SetTriggerInput(TIM8, LL_TIM_TS_ITR0); // TIM1 is ITR0 for TIM8
    
    printf("Starting TIM1...\n");
    LL_TIM_EnableCounter(TIM1);
    
    printf("Waiting for timers to run (35ms simulation time)...\n");
    // 35ms at 100MHz is 3,500,000 cycles
    // CPU frequency is 170MHz, so we wait for 59,500,000 CPU cycles.
    uint32_t start_cycles = DWT->CYCCNT;
    while((DWT->CYCCNT - start_cycles) < 60000000) {
        for (int i = 0; i < 1000; i++) { asm volatile("nop"); }
    }

    uint32_t cnt1 = LL_TIM_GetCounter(TIM1);
    uint32_t cnt8 = LL_TIM_GetCounter(TIM8);
    printf("At T~35ms: TIM1_CNT=%u, TIM8_CNT=%u\n", (unsigned int)cnt1, (unsigned int)cnt8);

    // Total cycles passed (timer units) = 60,000,000 / 1.7 = 35,294,117 ?
    // Actually, TIM1 and DWT are both 100MHz in REPL, so 1:1.
    // So 35,294,117 timer cycles passed.
    // 35,294,117 / 30,000 = 1176 full periods.
    // 35,294,117 % 30,000 = 14,117.
    // So TIM8 should be around 14117.
    
    CHECK_TEXT(cnt8 > 0, "TIM8 was never triggered");
    CHECK_TEXT(cnt8 < 30000, "TIM8 counter too high - should reset every 30k cycles");
    CHECK_TEXT(cnt8 > 5000, "TIM8 counter too low - either frequency mismatch or didn't start");
    LL_TIM_DisableCounter(TIM1);
    LL_TIM_DisableCounter(TIM8);
    LL_TIM_SetSlaveMode(TIM8, LL_TIM_SLAVEMODE_DISABLED);
}
#endif

#if 0
TEST(TimersTest, StartOutOfPhaseProductionTimers) {
    printf("Enabling TIM1, TIM8, TIM20 clocks\n");
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM20);
 
    uint32_t arr = 1000;
    TIM_TypeDef* timers_arr[] = {TIM1, TIM8, TIM20};
    std::span<TIM_TypeDef*> timers(timers_arr);
 
    for (auto* tim : timers) {
        LL_TIM_SetAutoReload(tim, arr);
        LL_TIM_SetPrescaler(tim, 0);
        LL_TIM_SetCounterMode(tim, LL_TIM_COUNTERMODE_CENTER_UP);
        LL_TIM_DisableCounter(tim);
        LL_TIM_SetCounter(tim, 0);
    }
 
    printf("Starting timers out of phase (offset=0.25)\n");
    stfoc::StartTimersOutOfPhase(timers, 0.25f);
 
    printf("Checking if all timers are enabled\n");
    CHECK_TRUE(LL_TIM_IsEnabledCounter(TIM1));
    CHECK_TRUE(LL_TIM_IsEnabledCounter(TIM8));
    CHECK_TRUE(LL_TIM_IsEnabledCounter(TIM20));
 
    // In center-aligned mode, 0.25 offset should mean TIM8 starts when TIM1 is at 0.25 * ARR * 2 = 0.5 * ARR.
    // However, since they both count, the relative difference should be maintained.
    uint32_t cnt1 = LL_TIM_GetCounter(TIM1);
    uint32_t cnt8 = LL_TIM_GetCounter(TIM8);
    uint32_t cnt20 = LL_TIM_GetCounter(TIM20);
 
    printf("Counters: TIM1=%lu, TIM8=%lu, TIM20=%lu\n", cnt1, cnt8, cnt20);
 
    // Check offsets. Allow some slack for simulation execution delays.
    // Offset between 1 and 8 should be approx 0.25 * 1000 * 2 = 500.
    int32_t diff18 = (int32_t)cnt1 - (int32_t)cnt8;
    int32_t diff820 = (int32_t)cnt8 - (int32_t)cnt20;
 
    DOUBLES_EQUAL(500, diff18, 50);
    DOUBLES_EQUAL(500, diff820, 50);
 
    printf("Verifying cleanup\n");
    for (auto* tim : timers) {
        CHECK_EQUAL(LL_TIM_SLAVEMODE_DISABLED, tim->SMCR & (TIM_SMCR_SMS | TIM_SMCR_SMS_3));
        CHECK_EQUAL(LL_TIM_TRGO_RESET, tim->CR2 & TIM_CR2_MMS);
    }
}
#endif

#include <string.h>

extern "C" {
static void ReadLine(char* buf, size_t max_len) {
  size_t pos = 0;
  while (pos < max_len - 1) {
    uint8_t ch;
    if (HAL_UART_Receive(&huart1, &ch, 1, HAL_MAX_DELAY) == HAL_OK) {
      if (ch == '\r' || ch == '\n') {
        if (pos > 0) break;
        continue;
      }
      buf[pos++] = ch;
    }
  }
  buf[pos] = '\0';
}

void Setup() {
  // Initialization of HAL and MX_..._Init is handled by main.c.
  // We explicitly enable the UART here to ensure printf works.
  __HAL_UART_ENABLE(&huart1);
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
  
  char cmd_buf[256];
  ReadLine(cmd_buf, sizeof(cmd_buf));
  
  int argc = 0;
  char* argv[16];
  static char program_name[] = "blink-tests";
  argv[argc++] = program_name;

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
