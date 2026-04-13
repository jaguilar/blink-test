#ifndef STFOC_FOC_TYPES_H
#define STFOC_FOC_TYPES_H

#include <stdint.h>
#include <sys/types.h>
#include <bit>
#include <cassert>
#include <span>
#include <cmath>
#include <algorithm>

#include "stm32g4xx.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_rcc.h"

namespace stfoc {

struct GpioEntry {
  uintptr_t gpio_base = 0;
  uint32_t pin = 0;
  bool active_high = true;

  GPIO_TypeDef* gpio() const {
    return reinterpret_cast<GPIO_TypeDef*>(gpio_base);
  }
};

namespace internal {
void InitGpio(const GpioEntry& entry);
void GpioAssert(const GpioEntry& entry);
void GpioDeassert(const GpioEntry& entry);
void ResetAllTimers();

inline uint32_t NsToTimerTicks(uint32_t ns) {
#ifndef NDEBUG
  LL_RCC_ClocksTypeDef clocks;
  LL_RCC_GetSystemClocksFreq(&clocks);
  assert(SystemCoreClock == clocks.PCLK1_Frequency &&
         SystemCoreClock == clocks.PCLK2_Frequency &&
         "This software assumes that the PCLK frequencies match the system "
         "core clock frequency.");
#endif
  return static_cast<uint32_t>(std::ceil((1.0 * ns) / (1e9 / SystemCoreClock)));
}

inline uint32_t GetRuntimeTRGOItrValue(uintptr_t leader_timer_base) {
#define CONSIDER(leader, itr)               \
  if (leader_timer_base == leader##_BASE) { \
    return LL_TIM_TS_##itr;                 \
  }
  // See RM0440 section 11.3.1.
  CONSIDER(TIM1, ITR0);
  CONSIDER(TIM2, ITR1);
  CONSIDER(TIM3, ITR2);
  CONSIDER(TIM4, ITR3);
  CONSIDER(TIM5, ITR4);
  CONSIDER(TIM8, ITR5);
  CONSIDER(TIM15, ITR6);
  CONSIDER(TIM20, ITR9);
#undef CONSIDER
  return 0;
}

template <uintptr_t leader_timer_base, uintptr_t follower_timer_base>
inline constexpr uint32_t GetTRGOItrValue() {
  static_assert(
      leader_timer_base != follower_timer_base,
      "Leader and follower timers must be different in GetTRGOItrValue");
#define CONSIDER(leader, itr)                         \
  if constexpr (leader_timer_base == leader##_BASE) { \
    return LL_TIM_TS_##itr;                           \
  }
  // See RM0440 section 11.3.1.
  CONSIDER(TIM1, ITR0);
  CONSIDER(TIM2, ITR1);
  CONSIDER(TIM3, ITR2);
  CONSIDER(TIM4, ITR3);
  CONSIDER(TIM5, ITR4);
  CONSIDER(TIM8, ITR5);
  CONSIDER(TIM15, ITR6);
  CONSIDER(TIM20, ITR9);
#undef CONSIDER
}

}  // namespace internal

inline void StartOutOfPhase(std::span<TIM_TypeDef*> timers, uint32_t arr) {
  if (timers.empty()) return;

  uint32_t n = timers.size();

  // Set ARR for all timers.
  for (auto* tim : timers) {
    LL_TIM_SetAutoReload(tim, arr);
  }

  // Calculate phase delay. We double the delay if center-aligned (up-down) mode
  // is used.
  uint32_t phase_delay = arr / n;
  if (LL_TIM_GetCounterMode(timers[0]) != LL_TIM_COUNTERMODE_UP &&
      LL_TIM_GetCounterMode(timers[0]) != LL_TIM_COUNTERMODE_DOWN) {
    phase_delay *= 2;
  }

  for (uint32_t i = 0; i < n; ++i) {
    TIM_TypeDef* tim = timers[i];
    LL_TIM_DisableCounter(tim);
    LL_TIM_OC_SetCompareCH1(tim, phase_delay);
    LL_TIM_CC_EnableChannel(tim, LL_TIM_CHANNEL_CH1);
    LL_TIM_SetCounter(tim, 0);
    LL_TIM_SetTriggerOutput(tim, LL_TIM_TRGO_CC1IF);
    LL_TIM_SetAutoReload(tim, arr);
  }

  for (int i = (int)n - 1; i > 0; --i) {
    TIM_TypeDef* tim = timers[i];
    TIM_TypeDef* leader = timers[i - 1];
    uint32_t itr =
        internal::GetRuntimeTRGOItrValue(reinterpret_cast<uintptr_t>(leader));
    LL_TIM_SetTriggerInput(tim, itr);
    LL_TIM_SetSlaveMode(tim, LL_TIM_SLAVEMODE_TRIGGER);
  }

  LL_TIM_EnableCounter(timers[0]);

  // Wait for the last timer to start. At this point the timers are running and
  // out of phase from one another.
  while (!LL_TIM_IsEnabledCounter(timers[n - 1])) {
  }

  // Reset the channel 1 to its initial state.
  for (auto* tim : timers) {
    LL_TIM_OC_SetCompareCH1(tim, 0);
    LL_TIM_CC_DisableChannel(tim, LL_TIM_CHANNEL_CH1);
  }
}

}  // namespace stfoc

#endif // STFOC_FOC_TYPES_H
