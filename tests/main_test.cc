#define STFOC_SIMULATION
#include <gtest/gtest.h>
#include "foc_types.h"
#include <span>
#include <vector>

#include <stdio.h>
#include <wchar.h>

extern "C" {
    // STM32 HAL / CMSIS includes if needed
    #include "main.h"

    // Stubs for newlib-nano missing wide character functions that GTest links against
    wint_t getwc(FILE *) { return 0; }
    wint_t ungetwc(wint_t, FILE *) { return 0; }
    wint_t putwc(wchar_t, FILE *) { return 0; }
    int swprintf(wchar_t *, size_t, const wchar_t *, ...) { return 0; }

    // Reroute stdout to simulated UART2 (0x60000000)
    int _write(int file, char *ptr, int len) {
        for (int i = 0; i < len; i++) {
            *(volatile uint8_t *)0x60000000 = ptr[i];
        }
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
            
            // SystemCoreClock is 100MHz in Renode .repl
            const uint32_t freq = 100000000;
            tv->tv_sec = total_cycles / freq;
            tv->tv_usec = (total_cycles % freq) * 1000000 / freq;
        }
        return 0;
    }

    // Bypass actual hardware clock initialization to avoid simulation deadlocks
    void SystemInit(void) {
        // Enable FPU (CP10 and CP11 Full Access) to prevent HardFaults when GTest uses floats
        SCB->CPACR |= ((3U << 10U * 2U) | (3U << 11U * 2U));

        // Initialize SysTick for timekeeping
        SysTick->LOAD = 0xFFFFFF; // Max 24-bit value
        SysTick->VAL = 0;
        SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk; // Enable, No Interrupt, CPU Clock

        // Manually write 'S' to UART to indicate SystemInit ran
        *(volatile uint8_t *)0x60000000 = 'S';
    }
}


TEST(DummyTest, ShouldPass) {
    EXPECT_EQ(1, 1);
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
    
    printf("Waiting for %s overflow (ARR=%lu)\n", name, (uint32_t)LL_TIM_GetAutoReload(tim));
    // Wait for the counter to overflow and set UIF
    int timeout = 100000;
    while (!LL_TIM_IsActiveFlag_UPDATE(tim) && timeout > 0) {
        if (timeout % 10000 == 0) {
            printf(".");
        }
        // Small delay to allow virtual time to progress between register reads
        for (int i = 0; i < 100; i++) {
            asm volatile("nop");
        }
        timeout--;
    }
    printf("\n");
    
    if (timeout == 0) {
        printf("%s Timed out! CNT=%lu, SR=0x%lx\n", name, (uint32_t)LL_TIM_GetCounter(tim), (uint32_t)tim->SR);
    }
    
    EXPECT_GT(timeout, 0) << "Timed out waiting for " << name << " overflow";
    EXPECT_TRUE(LL_TIM_IsActiveFlag_UPDATE(tim)) << name << " UIF not set";
}

TEST(TimersTest, Tim1Overflows) {
    ExpectTimerOverflows(TIM1, LL_APB2_GRP1_PERIPH_TIM1, "TIM1");
}

TEST(TimersTest, Tim8Overflows) {
    ExpectTimerOverflows(TIM8, LL_APB2_GRP1_PERIPH_TIM8, "TIM8");
}

TEST(TimersTest, Tim20Overflows) {
    ExpectTimerOverflows(TIM20, LL_APB2_GRP1_PERIPH_TIM20, "TIM20");
}

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
    EXPECT_TRUE(LL_TIM_IsEnabledCounter(TIM1));
    EXPECT_TRUE(LL_TIM_IsEnabledCounter(TIM8));
    EXPECT_TRUE(LL_TIM_IsEnabledCounter(TIM20));
 
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
 
    EXPECT_NEAR(diff18, 500, 50);
    EXPECT_NEAR(diff820, 500, 50);
 
    printf("Verifying cleanup\n");
    for (auto* tim : timers) {
        EXPECT_EQ(tim->SMCR & (TIM_SMCR_SMS | TIM_SMCR_SMS_3), LL_TIM_SLAVEMODE_DISABLED);
        EXPECT_EQ(tim->CR2 & TIM_CR2_MMS, LL_TIM_TRGO_RESET);
    }
}
#endif

// In a bare-metal environment, GoogleTest's default main() might not invoke
// hardware init correctly before tests. You might need to set up HAL_Init(),
// SystemClock_Config(), etc. if you are testing hardware drivers directly.
// For now, we supply a basic main.
int main(void) {
    // Manually write 'M' to UART to indicate main started
    *(volatile uint8_t *)0x60000000 = 'M';
    *(volatile uint8_t *)0x60000000 = '\n';

    // Disable stdout buffering so GTest prints character-by-character immediately
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    printf("Starting blink-tests...\n");

    int argc = 1;
    char name[] = "blink-tests";
    char *argv[] = { name, NULL };

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
