#include "stm32_motor_driver.h"
#include "stm32g474xx.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_tim.h"

namespace stfoc {

uint32_t StTimerMotorConfig::ComputeArr() const {
  LL_RCC_ClocksTypeDef rcc_clocks;
  LL_RCC_GetSystemClocksFreq(&rcc_clocks);
  return rcc_clocks.PCLK2_Frequency / pwm_freq / 2;
}

namespace internal {

void InitTimer(const StTimerMotorConfig& config) {
  LL_RCC_ClocksTypeDef rcc_clocks;
  LL_RCC_GetSystemClocksFreq(&rcc_clocks);

  LL_TIM_InitTypeDef timer_init = {
      .Prescaler = 0,
      .CounterMode = LL_TIM_COUNTERMODE_CENTER_UP,
      .Autoreload = config.ComputeArr(),
      .ClockDivision = LL_TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 1,
  };
  LL_TIM_Init(config.timer(), &timer_init);
  LL_TIM_SetCounter(config.timer(), 0);

  LL_TIM_OC_InitTypeDef oc_init = {
      .OCMode = LL_TIM_OCMODE_PWM2,
      .OCState = LL_TIM_OCSTATE_ENABLE,
      .CompareValue = timer_init.Autoreload,
      .OCPolarity = LL_TIM_OCPOLARITY_HIGH,
      .OCIdleState = LL_TIM_OCIDLESTATE_LOW,
  };
  for (int chan :
       {LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2, LL_TIM_CHANNEL_CH3}) {
    LL_TIM_OC_Init(config.timer(), chan, &oc_init);
    LL_TIM_OC_EnablePreload(config.timer(), chan);
  }
}

}  // namespace internal
}  // namespace stfoc
