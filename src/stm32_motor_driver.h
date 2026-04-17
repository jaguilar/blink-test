#ifndef STFOC_STM32_MOTOR_DRIVER_H
#define STFOC_STM32_MOTOR_DRIVER_H

#include <algorithm>
#include <cmath>

#include "cmsis_os2.h"
#include "common/base_classes/BLDCDriver.h"
#include "common/foc_utils.h"
#include "foc_types.h"
#include "stm32g474xx.h"
#include "stm32g4xx_ll_tim.h"

namespace stfoc {

struct StTimerMotorConfig {
  uintptr_t timer_base;
  GpioEntry phase1_en;
  GpioEntry phase2_en;
  GpioEntry phase3_en;
  GpioEntry drv_en;
  uint32_t pwm_freq;

  TIM_TypeDef* timer() const {
    return reinterpret_cast<TIM_TypeDef*>(timer_base);
  }

  uint32_t ComputeArr() const;
};

namespace internal {
void InitTimer(const StTimerMotorConfig& config);
}

template <StTimerMotorConfig config>
class StTimerMotorDriver : public BLDCDriver {
 public:
  static_assert(config.timer_base == TIM1_BASE ||
                    config.timer_base == TIM8_BASE ||
                    config.timer_base == TIM20_BASE,
                "Only TIM1, TIM8 and TIM20 are supported as timer bases for "
                "StTimerMotorDriver");

  StTimerMotorDriver() : BLDCDriver() {
    voltage_power_supply = NOT_SET;
    voltage_limit = NOT_SET;
  };
  ~StTimerMotorDriver() {};

  int init() override;
  void enable() override;
  void disable() override;
  void setPwm(float Ua, float Ub, float Uc) override;
  void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) override;

 private:
  TIM_TypeDef* timer() const {
    return reinterpret_cast<TIM_TypeDef*>(config.timer_base);
  }

  uint32_t arr_value_;
};

template <StTimerMotorConfig config>
int StTimerMotorDriver<config>::init() {
  arr_value_ = config.ComputeArr();

  assert(voltage_power_supply != NOT_SET);
  assert(voltage_limit != NOT_SET);

  internal::InitGpio(config.phase1_en);
  internal::InitGpio(config.phase2_en);
  internal::InitGpio(config.phase3_en);
  internal::InitGpio(config.drv_en);
  internal::InitTimer(config);
  setPwm(voltage_power_supply / 2.0f, voltage_power_supply / 2.0f,
         voltage_power_supply / 2.0f);
  initialized = true;
  return 1;
}

template <StTimerMotorConfig config>
void StTimerMotorDriver<config>::enable() {
  internal::GpioAssert(config.drv_en);
  osDelay(1);

  LL_TIM_OC_SetMode(timer(), LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM2);
  LL_TIM_OC_SetMode(timer(), LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM2);
  LL_TIM_OC_SetMode(timer(), LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM2);
  LL_TIM_EnableAllOutputs(config.timer());
}

template <StTimerMotorConfig config>
void StTimerMotorDriver<config>::disable() {
  LL_TIM_DisableAllOutputs(config.timer());
  internal::GpioDeassert(config.drv_en);
  for (const auto& gpio :
       {config.phase1_en, config.phase2_en, config.phase3_en}) {
    internal::GpioDeassert(gpio);
  }
}

template <StTimerMotorConfig config>
void StTimerMotorDriver<config>::setPwm(float ua, float ub, float uc) {
  ua = std::clamp(ua, 0.0f, voltage_limit);
  ub = std::clamp(ub, 0.0f, voltage_limit);
  uc = std::clamp(uc, 0.0f, voltage_limit);

  dc_a = ua / voltage_power_supply;
  dc_b = ub / voltage_power_supply;
  dc_c = uc / voltage_power_supply;

  auto to_compare = [&](float duty) -> uint32_t {
    return (1 - duty) * arr_value_;
  };
  LL_TIM_OC_SetCompareCH1(timer(), to_compare(dc_a));
  LL_TIM_OC_SetCompareCH2(timer(), to_compare(dc_b));
  LL_TIM_OC_SetCompareCH3(timer(), to_compare(dc_c));
}

template <StTimerMotorConfig config>
void StTimerMotorDriver<config>::setPhaseState(PhaseState sa, PhaseState sb,
                                               PhaseState sc) {
  assert(sa != PhaseState::PHASE_LO && sb != PhaseState::PHASE_LO &&
         sc != PhaseState::PHASE_LO && sa != PhaseState::PHASE_HI &&
         sb != PhaseState::PHASE_HI && sc != PhaseState::PHASE_HI &&
         "PHASE_LO and PHASE_HI states are not currently supported in "
         "StTimerMotorDriver");
  if (sa == PhaseState::PHASE_OFF) {
    internal::GpioDeassert(config.phase1_en);
  } else if (sa == PhaseState::PHASE_ON) {
    internal::GpioAssert(config.phase1_en);
  }
  if (sb == PhaseState::PHASE_OFF) {
    internal::GpioDeassert(config.phase2_en);
  } else if (sb == PhaseState::PHASE_ON) {
    internal::GpioAssert(config.phase2_en);
  }
  if (sc == PhaseState::PHASE_OFF) {
    internal::GpioDeassert(config.phase3_en);
  } else if (sc == PhaseState::PHASE_ON) {
    internal::GpioAssert(config.phase3_en);
  }
}

}  // namespace stfoc

#endif // STFOC_STM32_MOTOR_DRIVER_H
