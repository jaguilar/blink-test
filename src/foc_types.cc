#include "foc_types.h"

#include <cstdio>

#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_tim.h"

namespace stfoc {
namespace internal {

void InitGpio(const GpioEntry& entry) {
  if (!entry.gpio_base || !entry.pin) return;
  auto gpio = entry.gpio();
  auto pin = entry.pin;
  if (entry.active_high) {
    LL_GPIO_ResetOutputPin(gpio, pin);
  } else {
    LL_GPIO_SetOutputPin(gpio, pin);
  }
  LL_GPIO_SetPinMode(gpio, pin, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(gpio, pin, LL_GPIO_OUTPUT_PUSHPULL);
}

void GpioAssert(const GpioEntry& entry) {
  assert(entry.gpio_base && "gpio_base not set");
  entry.gpio()->BSRR = entry.active_high ? entry.pin : (entry.pin << 16);
}

void GpioDeassert(const GpioEntry& entry) {
  assert(entry.gpio_base && "gpio_base not set");
  entry.gpio()->BSRR = entry.active_high ? (entry.pin << 16) : entry.pin;
}

void ResetAllTimers() {
  TIM_TypeDef* timers[] = {TIM1, TIM8, TIM20};
  const char* names[] = {"TIM1", "TIM8", "TIM20"};
  for (int i = 0; i < 3; ++i) {
    auto* tim = timers[i];
    auto name = names[i];
    uint32_t smcr_before = tim->SMCR;
    LL_TIM_DisableCounter(tim);
    tim->CR1 = 0;
    tim->CR2 = 0;
    tim->SMCR = 0;
    tim->DIER = 0;
    tim->SR = 0;
    tim->EGR = TIM_EGR_UG;
    tim->CCMR1 = 0;
    tim->CCMR2 = 0;
    tim->CCER = 0;
    tim->CNT = 0;
    tim->PSC = 0;
    tim->ARR = 0xFFFF;
    tim->CCR1 = 0;
    tim->CCR2 = 0;
    tim->CCR3 = 0;
    tim->CCR4 = 0;
    tim->BDTR = 0;
    tim->DCR = 0;
    tim->AF1 = 0;
    tim->AF2 = 0;
    tim->TISEL = 0;
    tim->OR = 0;
    printf("  Reset %s: SMCR 0x%lx -> 0x%lx\n", name,
           (unsigned long)smcr_before, (unsigned long)tim->SMCR);
  }
}

}  // namespace internal
}  // namespace stfoc
