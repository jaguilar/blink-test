#include "as5048a_spi_sensor.h"
#include <cassert>
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_dmamux.h"

namespace stfoc {

namespace internal {

uint32_t SyncReadSpi(const AsyncTimerSpiConfig& config, uint16_t address) {
  assert(!LL_TIM_IsEnabledCounter(config.tim()) &&
         "Timer must not have been started when SyncReadSpi is called.");
  assert(LL_TIM_GetCounter(config.tim()) == 0 &&
         "Timer counter must be at 0 when SyncReadSpi is called.");
  assert(address <= 0x3FFF && "AS5048A addresses are 14 bits");
  uint16_t read_command = address | (1 << 14);  // (bit 14 == 1 for read)
  bool is_odd = std::popcount(read_command) & 1;
  if (!is_odd) { // Parity is even, make it odd
    read_command |= 1 << 15;
  }

  auto busy_wait_ns = [](uint32_t ns) {
    uint32_t start = DWT->CYCCNT;
    uint32_t wait_ticks =
        static_cast<uint32_t>(1.0 * ns * SystemCoreClock / 1e9);
    while ((uint32_t)(DWT->CYCCNT - start) < wait_ticks) {
    }
  };

  auto csn_set_asserted = [&](bool active) {
    if (!active) busy_wait_ns(50);
    LL_TIM_OC_SetMode(
        config.tim(), config.timer_channel_csn,
        active ? LL_TIM_OCMODE_FORCED_ACTIVE : LL_TIM_OCMODE_PWM2);
    if (active) busy_wait_ns(350);
  };

  while (!LL_SPI_IsActiveFlag_TXE(config.spi())) {
  }

  busy_wait_ns(350);
  csn_set_asserted(true);
  LL_SPI_TransmitData16(config.spi(), read_command);
  while (!LL_SPI_IsActiveFlag_TXE(config.spi())) {
  }
  csn_set_asserted(false);

  while (LL_SPI_IsActiveFlag_RXNE(config.spi())) {
    LL_SPI_ReceiveData16(config.spi());
  }

  busy_wait_ns(350);
  csn_set_asserted(true);
  uint16_t nop_command = 0x0000;
  LL_SPI_TransmitData16(config.spi(), nop_command);
  while (!LL_SPI_IsActiveFlag_RXNE(config.spi())) {
  }
  uint16_t result = LL_SPI_ReceiveData16(config.spi());
  csn_set_asserted(false);
  return result;
}

uint32_t SpiNRxDmaReq(SPI_TypeDef* spi) {
  if (spi == SPI1) return LL_DMAMUX_REQ_SPI1_RX;
  if (spi == SPI2) return LL_DMAMUX_REQ_SPI2_RX;
  if (spi == SPI3) return LL_DMAMUX_REQ_SPI3_RX;
  if (spi == SPI4) return LL_DMAMUX_REQ_SPI4_RX;
  assert(false && "Unsupported SPI instance");
  return 0;
}

uint32_t TimNCh4DmaReq(TIM_TypeDef* tim) {
  if (tim == TIM1) return LL_DMAMUX_REQ_TIM1_CH4;
  if (tim == TIM2) return LL_DMAMUX_REQ_TIM2_CH4;
  if (tim == TIM3) return LL_DMAMUX_REQ_TIM3_CH4;
  if (tim == TIM4) return LL_DMAMUX_REQ_TIM4_CH4;
  if (tim == TIM5) return LL_DMAMUX_REQ_TIM5_CH4;
  if (tim == TIM8) return LL_DMAMUX_REQ_TIM8_CH4;
  if (tim == TIM20) return LL_DMAMUX_REQ_TIM20_CH4;
  assert(false && "Unsupported timer instance");
  return 0;
}

}  // namespace internal

const uint16_t* AS5048ReadAngleCommandBuf() {
  alignas(uint16_t) static const uint16_t buf[] = {0xFFFF, 0x0000};
  return buf;
}

}  // namespace stfoc
