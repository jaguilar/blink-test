#ifndef STFOC_STM32_ADC_CURRENT_SENSE_H
#define STFOC_STM32_ADC_CURRENT_SENSE_H

#include "Arduino.h"
#include "cmsis_os2.h"
#include "common/base_classes/CurrentSense.h"
#include "foc_types.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_rcc.h"

namespace stfoc {

struct Stm32AdcCurrentSenseConfig {
  uintptr_t adc1_base = 0;
  uint32_t adc1_channel = 0xFFFFFFFF;
  uintptr_t adc2_base = 0;
  uint32_t adc2_channel = 0xFFFFFFFF;
  uintptr_t dma_base = 0;
  uint32_t adc1_dma_channel = 0xFFFFFFFF;
  uint32_t adc2_dma_channel = 0xFFFFFFFF;
  float shunt_resistance;
  float amp_gain;
  float v_ref = 3.3f;
  float cal_a = 1.0f;
  float cal_b = 1.0f;
  GpioEntry en_pin;
  GpioEntry cal_pin;
  uint32_t sampling_time = LL_ADC_SAMPLINGTIME_12CYCLES_5;
  uint32_t oversampling_ratio = LL_ADC_OVS_RATIO_4;
  uint32_t oversampling_shift = LL_ADC_OVS_SHIFT_RIGHT_2;

  ADC_TypeDef* adc1() const { return reinterpret_cast<ADC_TypeDef*>(adc1_base); }
  ADC_TypeDef* adc2() const { return reinterpret_cast<ADC_TypeDef*>(adc2_base); }
  DMA_TypeDef* dma() const { return reinterpret_cast<DMA_TypeDef*>(dma_base); }
};

namespace internal {
uint32_t AdcNInjectedDmaReq(uintptr_t adc_base);
uint32_t GetAdcTriggerForTimer(uintptr_t leader_timer_base);
}

template <Stm32AdcCurrentSenseConfig config>
class Stm32AdcCurrentSense : public CurrentSense {
 public:
  static_assert(config.adc1_base != 0 && config.adc2_base != 0, "Both ADC bases must be provided");
  static_assert(config.adc1_channel != 0xFFFFFFFF && config.adc2_channel != 0xFFFFFFFF, "Both ADC channels must be provided");
  static_assert(config.dma_base != 0, "DMA base must be provided");
  static_assert(config.adc1_dma_channel != 0xFFFFFFFF && config.adc2_dma_channel != 0xFFFFFFFF, "Both DMA channels must be provided");

  Stm32AdcCurrentSense() : CurrentSense() {
    offset_ia = 1.65f;
    offset_ib = 1.65f;
  }

  int init() override;
  PhaseCurrent_s getPhaseCurrents() override;

  template <uintptr_t leader_timer_base>
  void AsyncReadFromMotorUpdate() {
    LL_ADC_REG_StartConversion(config.adc1());
    LL_ADC_REG_StartConversion(config.adc2());
  }

  void EnableTrigger();
  void DisableTrigger();

  template <uintptr_t leader_timer_base>
  void SlaveToTimerUpdate();

  const uint16_t* GetAdc1Buffer() const { return &adc1_val_; }
  const uint16_t* GetAdc2Buffer() const { return &adc2_val_; }

  alignas(uint32_t) uint16_t adc1_val_ = 0;
  alignas(uint32_t) uint16_t adc2_val_ = 0;
};

template <Stm32AdcCurrentSenseConfig config>
int Stm32AdcCurrentSense<config>::init() {
  if (config.adc1_base == ADC1_BASE || config.adc2_base == ADC1_BASE) {
      LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);
  }
  LL_ADC_CommonInitTypeDef adc_common_init = {
      .CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4,
      .Multimode = LL_ADC_MULTI_INDEPENDENT,
  };
  LL_ADC_CommonInit(ADC12_COMMON, &adc_common_init);
  LL_ADC_SetCommonPathInternalCh(ADC12_COMMON, LL_ADC_PATH_INTERNAL_VREFINT);

  LL_ADC_InitTypeDef adc_init = {
      .Resolution = LL_ADC_RESOLUTION_12B,
      .DataAlignment = LL_ADC_DATA_ALIGN_RIGHT,
      .LowPowerMode = LL_ADC_LP_MODE_NONE,
  };
  LL_ADC_Init(config.adc1(), &adc_init);
  LL_ADC_REG_SetSequencerLength(config.adc1(), LL_ADC_REG_SEQ_SCAN_DISABLE);
  LL_ADC_REG_SetSequencerRanks(config.adc1(), LL_ADC_REG_RANK_1, config.adc1_channel);
  LL_ADC_SetChannelSamplingTime(config.adc1(), config.adc1_channel, config.sampling_time);
  LL_ADC_REG_SetDMATransfer(config.adc1(), LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

  LL_ADC_Init(config.adc2(), &adc_init);
  LL_ADC_REG_SetSequencerLength(config.adc2(), LL_ADC_REG_SEQ_SCAN_DISABLE);
  LL_ADC_REG_SetSequencerRanks(config.adc2(), LL_ADC_REG_RANK_1, config.adc2_channel);
  LL_ADC_SetChannelSamplingTime(config.adc2(), config.adc2_channel, config.sampling_time);
  LL_ADC_REG_SetDMATransfer(config.adc2(), LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

  LL_ADC_SetOverSamplingScope(config.adc1(), LL_ADC_OVS_GRP_REGULAR_CONTINUED);
  LL_ADC_ConfigOverSamplingRatioShift(config.adc1(), config.oversampling_ratio,
                                      config.oversampling_shift);
  LL_ADC_SetOverSamplingScope(config.adc2(), LL_ADC_OVS_GRP_REGULAR_CONTINUED);
  LL_ADC_ConfigOverSamplingRatioShift(config.adc2(), config.oversampling_ratio,
                                      config.oversampling_shift);

  LL_DMA_InitTypeDef dma_init = {
      .Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
      .Mode = LL_DMA_MODE_CIRCULAR,
      .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
      .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_NOINCREMENT,
      .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD,
      .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD,
      .NbData = 1,
      .Priority = LL_DMA_PRIORITY_HIGH,
  };
  LL_DMA_Init(config.dma(), config.adc1_dma_channel, &dma_init);
  LL_DMA_SetPeriphAddress(config.dma(), config.adc1_dma_channel, LL_ADC_DMA_GetRegAddr(config.adc1(), LL_ADC_DMA_REG_REGULAR_DATA));
  LL_DMA_SetMemoryAddress(config.dma(), config.adc1_dma_channel, reinterpret_cast<uintptr_t>(&adc1_val_));
  LL_DMA_SetPeriphRequest(config.dma(), config.adc1_dma_channel, internal::AdcNInjectedDmaReq(config.adc1_base));

  LL_DMA_Init(config.dma(), config.adc2_dma_channel, &dma_init);
  LL_DMA_SetPeriphAddress(config.dma(), config.adc2_dma_channel, LL_ADC_DMA_GetRegAddr(config.adc2(), LL_ADC_DMA_REG_REGULAR_DATA));
  LL_DMA_SetMemoryAddress(config.dma(), config.adc2_dma_channel, reinterpret_cast<uintptr_t>(&adc2_val_));
  LL_DMA_SetPeriphRequest(config.dma(), config.adc2_dma_channel, internal::AdcNInjectedDmaReq(config.adc2_base));

  LL_DMA_EnableChannel(config.dma(), config.adc1_dma_channel);
  LL_DMA_EnableChannel(config.dma(), config.adc2_dma_channel);

  auto enable_adc = [](ADC_TypeDef* adc) {
    LL_ADC_DisableDeepPowerDown(adc);
    LL_ADC_EnableInternalRegulator(adc);
    for(volatile uint32_t i=0; i<3200; i++);
    LL_ADC_StartCalibration(adc, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(adc));
    LL_ADC_Enable(adc);
    while (!LL_ADC_IsActiveFlag_ADRDY(adc));
  };
  enable_adc(config.adc1());
  enable_adc(config.adc2());

  // Perform Offset Calibration
  if (config.cal_pin.gpio() != nullptr &&
      config.cal_pin.pin != GpioEntry::kPinUnset) {
    internal::InitGpio(config.en_pin);
    internal::InitGpio(config.cal_pin);

    // If an enable pin is provided, we must assert it to power on the
    // amplifiers before we can measure their offsets.
    if (config.en_pin.gpio() != nullptr) {
      internal::GpioAssert(config.en_pin);
      osDelay(2);  // Wait for DRV8304 to wake up from sleep
    }

    internal::GpioAssert(config.cal_pin);
    osDelay(10);

    int tries = 0;
    while (true) {
      // Temporarily disable DMA to allow manual polling of EOC, and ensure
      // software trigger is selected.
      LL_ADC_REG_SetDMATransfer(config.adc1(), LL_ADC_REG_DMA_TRANSFER_NONE);
      LL_ADC_REG_SetDMATransfer(config.adc2(), LL_ADC_REG_DMA_TRANSFER_NONE);
      LL_ADC_REG_SetTriggerSource(config.adc1(), LL_ADC_REG_TRIG_SOFTWARE);
      LL_ADC_REG_SetTriggerSource(config.adc2(), LL_ADC_REG_TRIG_SOFTWARE);

      float sum_a = 0;
      float sum_b = 0;
      constexpr int kSamples = 100;
      for (int i = 0; i < kSamples; i++) {
        LL_ADC_REG_StartConversion(config.adc1());
        LL_ADC_REG_StartConversion(config.adc2());
        while (!LL_ADC_IsActiveFlag_EOC(config.adc1()));
        while (!LL_ADC_IsActiveFlag_EOC(config.adc2()));
        sum_a +=
            static_cast<float>(LL_ADC_REG_ReadConversionData12(config.adc1()));
        sum_b +=
            static_cast<float>(LL_ADC_REG_ReadConversionData12(config.adc2()));
      }

      // Re-enable DMA
      LL_ADC_REG_SetDMATransfer(config.adc1(),
                                LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
      LL_ADC_REG_SetDMATransfer(config.adc2(),
                                LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

      float counts_to_volts = config.v_ref / 4096.0f;
      float measured_ia = (sum_a / kSamples) * counts_to_volts;
      float measured_ib = (sum_b / kSamples) * counts_to_volts;

      float center = config.v_ref / 2.0f;
      if (std::abs(measured_ia - center) < 0.1f &&
          std::abs(measured_ib - center) < 0.1f) {
        offset_ia = measured_ia;
        offset_ib = measured_ib;
        std::printf(
            "[ADC] Calibration complete: Offset A: %dmV, Offset B: %dmV\n",
            static_cast<int>(offset_ia * 1000.0f),
            static_cast<int>(offset_ib * 1000.0f));
        break;
      }

      std::printf(
          "[ADC] WARNING: Implausible offsets measured! A: %dmV, B: %dmV "
          "(Center: %dmV). Retrying...\n",
          static_cast<int>(measured_ia * 1000.0f),
          static_cast<int>(measured_ib * 1000.0f),
          static_cast<int>(center * 1000.0f));

      tries++;
      if (tries < 10) {
        osDelay(10);
      } else {
        osDelay(1000);
      }
    }

    internal::GpioDeassert(config.cal_pin);
  }

  initialized = true;
  return 1;
}

template <Stm32AdcCurrentSenseConfig config>
PhaseCurrent_s Stm32AdcCurrentSense<config>::getPhaseCurrents() {
  const uint16_t val_ia = adc1_val_;
  const uint16_t val_ib = adc2_val_;
  constexpr float counts_to_volts = config.v_ref / 4096.0f;
  constexpr float volts_per_amp = config.shunt_resistance * config.amp_gain;
  const float ia =
      ((static_cast<float>(val_ia) * counts_to_volts) - offset_ia) /
      volts_per_amp * config.cal_a;
  const float ib =
      ((static_cast<float>(val_ib) * counts_to_volts) - offset_ib) /
      volts_per_amp * config.cal_b;
  return {ia, ib, 0};
}

template <Stm32AdcCurrentSenseConfig config>
template <uintptr_t leader_timer_base>
void Stm32AdcCurrentSense<config>::SlaveToTimerUpdate() {
  uint32_t trigger = internal::GetAdcTriggerForTimer(leader_timer_base);
  LL_ADC_REG_SetTriggerSource(config.adc1(), trigger);
  LL_ADC_REG_SetTriggerEdge(config.adc1(), LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_SetTriggerSource(config.adc2(), trigger);
  LL_ADC_REG_SetTriggerEdge(config.adc2(), LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_StartConversion(config.adc1());
  LL_ADC_REG_StartConversion(config.adc2());
}

template <Stm32AdcCurrentSenseConfig config>
void Stm32AdcCurrentSense<config>::EnableTrigger() {
  LL_ADC_REG_SetTriggerEdge(config.adc1(), LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_SetTriggerEdge(config.adc2(), LL_ADC_REG_TRIG_EXT_RISING);
}

template <Stm32AdcCurrentSenseConfig config>
void Stm32AdcCurrentSense<config>::DisableTrigger() {
  LL_ADC_REG_SetTriggerEdge(config.adc1(), 0);
  LL_ADC_REG_SetTriggerEdge(config.adc2(), 0);
}

}  // namespace stfoc

#endif // STFOC_STM32_ADC_CURRENT_SENSE_H
