#include "stm32_adc_current_sense.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_dmamux.h"
#include "stm32g4xx_ll_tim.h"

namespace stfoc {
namespace internal {

uint32_t AdcNInjectedDmaReq(uintptr_t adc_base) {
  if (adc_base == ADC1_BASE) return LL_DMAMUX_REQ_ADC1;
  if (adc_base == ADC2_BASE) return LL_DMAMUX_REQ_ADC2;
  assert(false && "Unsupported ADC instance");
  return 0;
}

uint32_t GetAdcTriggerForTimer(uintptr_t leader_timer_base) {
  if (leader_timer_base == TIM1_BASE) return LL_ADC_REG_TRIG_EXT_TIM1_TRGO;
  if (leader_timer_base == TIM8_BASE) return LL_ADC_REG_TRIG_EXT_TIM8_TRGO;
  if (leader_timer_base == TIM20_BASE) return LL_ADC_REG_TRIG_EXT_TIM20_TRGO;
  assert(false && "Unsupported leader timer for ADC trigger");
  return 0;
}

}  // namespace internal
}  // namespace stfoc
