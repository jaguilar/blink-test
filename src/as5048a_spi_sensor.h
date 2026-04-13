#ifndef STFOC_AS5048A_SPI_SENSOR_H
#define STFOC_AS5048A_SPI_SENSOR_H

#include "foc_types.h"
#include "common/base_classes/Sensor.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_dmamux.h"

namespace stfoc {

struct AsyncTimerSpiConfig {
  uintptr_t spi_base;
  uintptr_t spi_rx_dma_base;
  uint32_t spi_rx_dma_channel;
  uintptr_t spi_tx_dma_base;
  uint32_t spi_tx_dma_channel;
  uintptr_t timer_base;
  uint32_t timer_channel_csn;
  uint32_t spi_baud_rate = LL_SPI_BAUDRATEPRESCALER_DIV16;

  TIM_TypeDef* tim() const {
    return reinterpret_cast<TIM_TypeDef*>(timer_base);
  }
  DMA_TypeDef* rx_dma() const {
    return reinterpret_cast<DMA_TypeDef*>(spi_rx_dma_base);
  }
  DMA_TypeDef* tx_dma() const {
    return reinterpret_cast<DMA_TypeDef*>(spi_tx_dma_base);
  }
  SPI_TypeDef* spi() const { return reinterpret_cast<SPI_TypeDef*>(spi_base); }
};

const uint16_t* AS5048ReadAngleCommandBuf();

namespace internal {
uint32_t SyncReadSpi(const AsyncTimerSpiConfig& config, uint16_t address);
uint32_t SpiNRxDmaReq(SPI_TypeDef* spi);
uint32_t TimNCh4DmaReq(TIM_TypeDef* tim);
}

template <AsyncTimerSpiConfig config>
class AsyncTimerAS5048ASpi : public Sensor {
 public:
  void init() override;

  template <uintptr_t leader_timer_base>
  void AsyncReadFromMotorUpdate();

  void update() override;

  float getSensorAngle() override { return angle_; }
  const uint16_t* getRawRxBuf() const { return spi_rx_buf_; }

  int needsSearch() override { return false; }

 private:
  static_assert(config.timer_channel_csn >= 1 &&
                config.timer_channel_csn <= 3 &&
                "CS pin must be on timer channel 1, 2 or 3 for "
                "AsyncTimerAS5048ASpi");

  SPI_TypeDef* spi() const { return config.spi(); }

  float angle_ = 0.0f;
  uint32_t csn_assert_timer_tick_;
  alignas(uint32_t) uint16_t spi_rx_buf_[2];
  volatile bool pending_dma_ = false;
};

template <AsyncTimerSpiConfig config>
void AsyncTimerAS5048ASpi<config>::init() {
  LL_SPI_InitTypeDef spi_init = {
      .TransferDirection = LL_SPI_FULL_DUPLEX,
      .Mode = LL_SPI_MODE_MASTER,
      .DataWidth = LL_SPI_DATAWIDTH_16BIT,
      .ClockPolarity = LL_SPI_POLARITY_LOW,
      .ClockPhase = LL_SPI_PHASE_2EDGE,
      .NSS = LL_SPI_NSS_SOFT,
      .BaudRate = config.spi_baud_rate,
      .BitOrder = LL_SPI_MSB_FIRST,
  };
  LL_SPI_Init(spi(), &spi_init);
  LL_SPI_SetStandard(spi(), LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_Enable(spi());

  LL_DMA_InitTypeDef rx_dma_init = {
      .Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
      .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
      .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
      .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD,
      .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD,
      .NbData = 2,
      .Priority = LL_DMA_PRIORITY_MEDIUM,
  };
  LL_DMA_Init(config.rx_dma(), config.spi_rx_dma_channel, &rx_dma_init);

  LL_DMA_InitTypeDef tx_dma_init = {
      .Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
      .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
      .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
      .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD,
      .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD,
      .NbData = 2,
      .Priority = LL_DMA_PRIORITY_MEDIUM,
  };
  LL_DMA_Init(config.tx_dma(), config.spi_tx_dma_channel, &tx_dma_init);

  csn_assert_timer_tick_ = internal::NsToTimerTicks(350);
  auto spi_start_timer_tick =
      csn_assert_timer_tick_ + internal::NsToTimerTicks(250);
  constexpr int ns_per_bit = 100;
  auto csn_deassert_timer_tick = spi_start_timer_tick +
                                 internal::NsToTimerTicks(16 * ns_per_bit) +
                                 internal::NsToTimerTicks(150);

  LL_TIM_InitTypeDef timer_init = {
      .Prescaler = 0,
      .CounterMode = LL_TIM_COUNTERMODE_UP,
      .Autoreload = csn_deassert_timer_tick,
      .ClockDivision = LL_TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
  };
  LL_TIM_Init(config.tim(), &timer_init);

  LL_TIM_OC_InitTypeDef oc_init = {
      .OCMode = LL_TIM_OCMODE_PWM2,
      .OCState = LL_TIM_OCSTATE_ENABLE,
      .CompareValue = csn_assert_timer_tick_ + 1,
      .OCPolarity = LL_TIM_OCPOLARITY_LOW,
      .OCIdleState = LL_TIM_OCIDLESTATE_HIGH,
  };
  LL_TIM_OC_Init(config.tim(), config.timer_channel_csn, &oc_init);

  LL_TIM_OC_InitTypeDef oc4_init = {
      .OCMode = LL_TIM_OCMODE_FROZEN,
      .OCState = LL_TIM_OCSTATE_ENABLE,
      .CompareValue = spi_start_timer_tick,
  };
  LL_TIM_OC_Init(config.tim(), LL_TIM_CHANNEL_CH4, &oc4_init);
  LL_TIM_SetSlaveMode(config.tim(), LL_TIM_SLAVEMODE_TRIGGER);
}

template <AsyncTimerSpiConfig config>
void AsyncTimerAS5048ASpi<config>::update() {
  uint16_t raw_angle;
  if (pending_dma_) {
    LL_TIM_OC_SetMode(config.tim(), config.timer_channel_csn,
                      LL_TIM_OCMODE_FORCED_INACTIVE);
    LL_TIM_SetOnePulseMode(config.tim(), LL_TIM_ONEPULSEMODE_SINGLE);
    raw_angle = spi_rx_buf_[1] & 0x3FFF;
    pending_dma_ = false;
  } else {
    raw_angle = internal::SyncReadSpi(config, 0x3FFF);
  }
  angle_ = (static_cast<float>(raw_angle) / 16384.0f) * 2.0f *
            static_cast<float>(M_PI);
  Sensor::update();
}

template <AsyncTimerSpiConfig config>
template <uintptr_t leader_timer_base>
void AsyncTimerAS5048ASpi<config>::AsyncReadFromMotorUpdate() {
  TIM_TypeDef* motor_timer = reinterpret_cast<TIM_TypeDef*>(leader_timer_base);
  while (LL_TIM_IsEnabledCounter(config.tim())) {
  }
  LL_TIM_SetTriggerOutput(motor_timer, LL_TIM_TRGO_UPDATE);

  LL_DMA_DisableChannel(config.tx_dma(), config.spi_tx_dma_channel);
  LL_DMA_DisableChannel(config.rx_dma(), config.spi_rx_dma_channel);
  LL_SPI_DisableDMAReq_RX(config.spi());
  LL_TIM_DisableDMAReq_CC4(config.tim());

  LL_DMA_SetMemoryAddress(
      config.tx_dma(), config.spi_tx_dma_channel,
      reinterpret_cast<uint32_t>(AS5048ReadAngleCommandBuf()));
  LL_DMA_SetPeriphAddress(config.tx_dma(), config.spi_tx_dma_channel,
                           (uint32_t)&config.spi()->DR);
  LL_DMA_SetPeriphRequest(config.tx_dma(), config.spi_tx_dma_channel,
                           internal::TimNCh4DmaReq(config.tim()));
  LL_DMA_SetMemoryAddress(config.rx_dma(), config.spi_rx_dma_channel,
                           reinterpret_cast<uint32_t>(spi_rx_buf_));
  LL_DMA_SetPeriphAddress(config.rx_dma(), config.spi_rx_dma_channel,
                           (uint32_t)&config.spi()->DR);
  LL_DMA_SetPeriphRequest(config.rx_dma(), config.spi_rx_dma_channel,
                           internal::SpiNRxDmaReq(config.spi()));
  LL_DMA_SetDataLength(config.tx_dma(), config.spi_tx_dma_channel, 2);
  LL_DMA_SetDataLength(config.rx_dma(), config.spi_rx_dma_channel, 2);

  LL_DMA_EnableChannel(config.tx_dma(), config.spi_tx_dma_channel);
  LL_DMA_EnableChannel(config.rx_dma(), config.spi_rx_dma_channel);
  LL_SPI_EnableDMAReq_RX(config.spi());
  LL_TIM_EnableDMAReq_CC4(config.tim());
  LL_DMA_EnableIT_TC(config.rx_dma(), config.spi_rx_dma_channel);

  LL_TIM_SetTriggerInput(
      config.tim(),
      internal::GetTRGOItrValue<leader_timer_base, config.timer_base>());
  LL_TIM_SetOnePulseMode(config.tim(), LL_TIM_ONEPULSEMODE_REPETITIVE);
  LL_TIM_OC_SetMode(config.tim(), config.timer_channel_csn, LL_TIM_OCMODE_PWM2);
  pending_dma_ = true;
}

}  // namespace stfoc

#endif // STFOC_AS5048A_SPI_SENSOR_H
