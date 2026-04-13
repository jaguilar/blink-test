#include <cstdio>
#include <span>
#include <vector>

#include "CppUTest/TestHarness.h"
#include "cmsis_os2.h"
#include "foc_types.h"

extern "C" {
#include "gpio.h"
#include "main.h"
#include "usart.h"
}

void ExpectTimerOverflows(TIM_TypeDef* tim, uint32_t periph, const char* name) {
    printf("Testing %s clock enablement\n", name);
    asm volatile("nop");
    LL_APB2_GRP1_EnableClock(periph);
    printf("Clock enabled for %s\n", name);
    
    printf("Setting %s ARR and PSC...\n", name);
    LL_TIM_SetAutoReload(tim, 50);
    LL_TIM_SetPrescaler(tim, 0);
    
    printf("Setting %s EGR UG...\n", name);
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

TEST_GROUP(TimersTest){
    void setup(){
        printf("--- setup: Resetting all timers ---\n");
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM20);
        stfoc::internal::ResetAllTimers();
    }
    void teardown() { stfoc::internal::ResetAllTimers(); }
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

TEST(TimersTest, TimerSyncTrigger) {
    // Lead timer
    printf("Configuring TIM1 (Leader)\n");
    LL_TIM_SetAutoReload(TIM1, 50000);
    LL_TIM_SetPrescaler(TIM1, 0);
    LL_TIM_OC_SetCompareCH4(TIM1, 500);
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM2);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC4REF);
    LL_TIM_SetCounter(TIM1, 0);
    
    // Slave timer
    printf("Configuring TIM8 (Slave)\n");
    LL_TIM_SetAutoReload(TIM8, 50000);
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
      printf(
          "TIM8 failed to start after %u cycles (%u ms)! TIM1_CNT=%u, "
          "TIM8_CNT=%u, TIM8_CR1=0x%x, TIM8_SMCR=0x%lx\n",
          (unsigned int)elapsed_cycles,
          (unsigned int)(elapsed_cycles / (SystemCoreClock / 1000)),
          (unsigned int)LL_TIM_GetCounter(TIM1),
          (unsigned int)LL_TIM_GetCounter(TIM8), (unsigned int)TIM8->CR1,
          TIM8->SMCR);
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
    LL_TIM_SetAutoReload(TIM8, 50000);
    LL_TIM_SetPrescaler(TIM8, 0);
    LL_TIM_SetCounter(TIM8, 0);
    LL_TIM_SetTriggerOutput(TIM8, LL_TIM_TRGO_ENABLE);
    
    // Slave timer: TIM1
    printf("Configuring TIM1 (Slave)\n");
    LL_TIM_SetAutoReload(TIM1, 50000);
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
    LL_TIM_SetRepetitionCounter(TIM1,
                                2);  // Update every 3 overflows (30000 cycles)
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET); // Default to Reset for now
    LL_TIM_GenerateEvent_UPDATE(TIM1); // Reload RCR and repetitionsLeft
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE); // Switch to Update mode for synchronization
    LL_TIM_SetCounter(TIM1, 0);

    // Slave: TIM8
    printf("Configuring TIM8 (Slave), Reset Mode\n");
    LL_TIM_SetAutoReload(TIM8, 60000);
    LL_TIM_SetPrescaler(TIM8, 0);
    LL_TIM_SetCounter(TIM8, 0);
    LL_TIM_SetSlaveMode(TIM8, LL_TIM_SLAVEMODE_RESET);
    LL_TIM_SetTriggerInput(TIM8, LL_TIM_TS_ITR0); // TIM1 is ITR0 for TIM8
    LL_TIM_EnableCounter(TIM8);

    printf("Starting TIM1...\n");
    LL_TIM_EnableCounter(TIM1);

    printf("Waiting for timers to run...\n");
    // Wait for approx 191ms at 170MHz = 32,500,000 cycles.
    // 32,500,000 / 30,000 = 1083 periods + 10,000 cycles.
    // So TIM8 should be around 10,000.
    uint32_t start_cycles = DWT->CYCCNT;
    while ((DWT->CYCCNT - start_cycles) < 32500000) {
      for (int i = 0; i < 1000; i++) {
        asm volatile("nop");
      }
    }

    uint32_t cnt1 = LL_TIM_GetCounter(TIM1);
    uint32_t cnt8 = LL_TIM_GetCounter(TIM8);
    printf("At T~191ms: TIM1_CNT=%u, TIM8_CNT=%u\n", (unsigned int)cnt1,
           (unsigned int)cnt8);

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

TEST(TimersTest, StartOutOfPhaseProductionTimers) {
  printf("--- Starting StartOutOfPhaseProductionTimers ---\n");

  stfoc::internal::ResetAllTimers();

  printf("Enabling TIM1, TIM8, TIM20 clocks\n");
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM20);

  uint32_t arr = 5000;
  TIM_TypeDef* timers_arr[] = {TIM1, TIM8, TIM20};
  std::span<TIM_TypeDef*> timers_span(timers_arr);

  for (auto* tim : timers_span) {
    LL_TIM_SetAutoReload(tim, arr);
    LL_TIM_SetPrescaler(tim, 0);
    LL_TIM_SetCounterMode(tim, LL_TIM_COUNTERMODE_CENTER_UP);
  }

  printf("Starting timers out of phase (arr=5000, n=3)\n");
  // Expected phase delay = 2 * arr / n = 10000 / 3 = 3333.
  stfoc::StartOutOfPhase(timers_span, arr);

  printf("Pre-start state check:\n");
  printf("  TIM1 SMCR=0x%lx, CR2=0x%lx\n", (unsigned long)TIM1->SMCR,
         (unsigned long)TIM1->CR2);
  printf("  TIM8 SMCR=0x%lx, CR2=0x%lx\n", (unsigned long)TIM8->SMCR,
         (unsigned long)TIM8->CR2);
  printf("  TIM20 SMCR=0x%lx, CR2=0x%lx\n", (unsigned long)TIM20->SMCR,
         (unsigned long)TIM20->CR2);

  // Wait for timers to run and stabilize
  for (volatile int i = 0; i < 50000; ++i) {
  }

  // Snapshot counters without stopping them.
  auto GetLinearTime = [arr](TIM_TypeDef* tim) -> int32_t {
    uint32_t cnt = LL_TIM_GetCounter(tim);
    bool is_down = (tim->CR1 & TIM_CR1_DIR) != 0;
    if (is_down) {
      return (int32_t)(2 * arr - cnt);
    } else {
      return (int32_t)cnt;
    }
  };

  // Sample multiple times.
  int32_t t1 = GetLinearTime(TIM1);
  int32_t t8 = GetLinearTime(TIM8);
  int32_t t20 = GetLinearTime(TIM20);

  auto CalculateDiff = [arr](int32_t master, int32_t follower) -> int32_t {
    int32_t diff = master - follower;
    if (diff < 0) diff += (int32_t)(2 * arr);
    return diff;
  };

  int32_t diff18 = CalculateDiff(t1, t8);
  int32_t diff120 = CalculateDiff(t1, t20);

  printf("Snapshot: T1=%ld, T8=%ld, T20=%ld\n", (long)t1, (long)t8, (long)t20);
  printf("Phasing Rel T1: T8=%ld, T20=%ld (Expected 3333, 6666)\n",
         (long)diff18, (long)diff120);

  // Check that we are clearly out of phase and close to the expected shift.
  CHECK_TEXT(diff18 > 2000 && diff18 < 4000, "T8 phase vs T1 is wildly off");
  CHECK_TEXT(diff120 > 5000 && diff120 < 8000, "T20 phase vs T1 is wildly off");

  printf("Verifying cleanup\n");
  stfoc::internal::ResetAllTimers();
  for (auto* tim : timers_span) {
    CHECK_EQUAL(0, tim->SMCR & (TIM_SMCR_SMS | TIM_SMCR_SMS_3));
    CHECK_EQUAL(0, tim->CR2 & TIM_CR2_MMS);
  }
}
