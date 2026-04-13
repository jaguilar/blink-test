#ifndef STFOC_STM32_ADC_CURRENT_SENSE_H
#define STFOC_STM32_ADC_CURRENT_SENSE_H

#include "foc_types.h"
#include "common/base_classes/CurrentSense.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_dma.h"
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
  float offset_a;
  float offset_b;
  uint32_t sampling_time = LL_ADC_SAMPLINGTIME_47CYCLES_5;

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
    offset_ia = config.offset_a;
    offset_ib = config.offset_b;
  }

  int init() override;
  PhaseCurrent_s getPhaseCurrents() override;

  template <uintptr_t leader_timer_base>
  void SlaveToTimerUpdate();

  void EnableTrigger();
  void DisableTrigger();

  const uint16_t* GetAdc1Buffer() const { return &adc1_val_; }
  const uint16_t* GetAdc2Buffer() const { return &adc2_val_; }

 private:
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

  initialized = true;
  return 1;
}

template <Stm32AdcCurrentSenseConfig config>
PhaseCurrent_s Stm32AdcCurrentSense<config>::getPhaseCurrents() {
  float counts_to_volts = config.v_ref / 4096.0f;
  float volts_per_amp = config.shunt_resistance * config.amp_gain;
  float ia = ((static_cast<float>(adc1_val_) * counts_to_volts) - config.offset_a) / volts_per_amp * config.cal_a;
  float ib = ((static_cast<float>(adc2_val_) * counts_to_volts) - config.offset_b) / volts_per_amp * config.cal_b;
  float ic = -(ia + ib);
  return {ia, ib, ic};
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
