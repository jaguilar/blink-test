#include "app.h"

#include <cassert>
#include <cmath>
#include <span>
#include <type_traits>

#include "cmsis_os2.h"
#include "stm32g474xx.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_dmamux.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"

namespace blink {
volatile bool spi_dma_complete = false;
volatile bool spi_dma_error = false;

template <typename T>
std::span<const char> ToSpan(T& obj) {
  static_assert(!std::is_pointer_v<std::remove_reference_t<T>>,
                "Pass objects/references to ToSpan, not pointers");
  return std::span(
      reinterpret_cast<const char*>(&const_cast<std::remove_cvref_t<T>&>(obj)),
      sizeof(T));
}

namespace {

template <uint32_t dma_base, uint32_t in_channel>
struct DMAMapping {
  DMA_TypeDef* dma() const { return reinterpret_cast<DMA_TypeDef*>(dma_base); }
  uint32_t channel() const { return in_channel; }

  int ToDmaMuxChannel() const {
    int mux_channel = channel();
    if (dma_base == DMA2_BASE) {
      mux_channel += 8;
    }
    return mux_channel;
  }

  void SetTransferSpan(std::span<const char> src,
                       std::span<const char> dst) const {
    if (src.data()) SetTransferSrc(src);
    if (dst.data()) SetTransferDest(dst);
    auto periph_size = LL_DMA_GetPeriphSize(dma(), channel());
    int transfer_size =
        periph_size == LL_DMA_PDATAALIGN_BYTE
            ? 1
            : (periph_size == LL_DMA_PDATAALIGN_HALFWORD ? 2 : 4);
    const int nb_data = std::max(src.size(), dst.size()) / transfer_size;
    LL_DMA_SetDataLength(dma(), channel(), nb_data);
  }

  void Enable() const { LL_DMA_EnableChannel(dma(), channel()); }
  void Disable() const { LL_DMA_DisableChannel(dma(), channel()); }

  void ClearAllInterrupts() const {
    switch (channel()) {
      case LL_DMA_CHANNEL_1:
        WRITE_REG(dma()->IFCR, DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 |
                                   DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1);
        break;
      case LL_DMA_CHANNEL_2:
        WRITE_REG(dma()->IFCR, DMA_IFCR_CGIF2 | DMA_IFCR_CTCIF2 |
                                   DMA_IFCR_CHTIF2 | DMA_IFCR_CTEIF2);
        break;
      case LL_DMA_CHANNEL_3:
        WRITE_REG(dma()->IFCR, DMA_IFCR_CGIF3 | DMA_IFCR_CTCIF3 |
                                   DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3);
        break;
      case LL_DMA_CHANNEL_4:
        WRITE_REG(dma()->IFCR, DMA_IFCR_CGIF4 | DMA_IFCR_CTCIF4 |
                                   DMA_IFCR_CHTIF4 | DMA_IFCR_CTEIF4);
        break;
      case LL_DMA_CHANNEL_5:
        WRITE_REG(dma()->IFCR, DMA_IFCR_CGIF5 | DMA_IFCR_CTCIF5 |
                                   DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5);
        break;
      case LL_DMA_CHANNEL_6:
        WRITE_REG(dma()->IFCR, DMA_IFCR_CGIF6 | DMA_IFCR_CTCIF6 |
                                   DMA_IFCR_CHTIF6 | DMA_IFCR_CTEIF6);
        break;
      case LL_DMA_CHANNEL_7:
        WRITE_REG(dma()->IFCR, DMA_IFCR_CGIF7 | DMA_IFCR_CTCIF7 |
                                   DMA_IFCR_CHTIF7 | DMA_IFCR_CTEIF7);
        break;
      case LL_DMA_CHANNEL_8:
        WRITE_REG(dma()->IFCR, DMA_IFCR_CGIF8 | DMA_IFCR_CTCIF8 |
                                   DMA_IFCR_CHTIF8 | DMA_IFCR_CTEIF8);
        break;
    }
  }

  template <typename T, typename U>
  void SetTransfer(const T& src, const U& dst) const {
    SetTransferSpan(ToSpan(src), ToSpan(dst));
  }

  void SetTransferDest(std::span<const char> dst) const {
    if (LL_DMA_GetDataTransferDirection(dma(), channel()) ==
        LL_DMA_DIRECTION_MEMORY_TO_PERIPH) {
      LL_DMA_SetPeriphAddress(dma(), channel(), (uint32_t)dst.data());
    } else {
      LL_DMA_SetMemoryAddress(dma(), channel(), (uint32_t)dst.data());
    }
  }

  void SetTransferSrc(std::span<const char> src) const {
    if (LL_DMA_GetDataTransferDirection(dma(), channel()) ==
        LL_DMA_DIRECTION_MEMORY_TO_PERIPH) {
      LL_DMA_SetMemoryAddress(dma(), channel(), (uint32_t)src.data());
    } else {
      LL_DMA_SetPeriphAddress(dma(), channel(), (uint32_t)src.data());
    }
  }
};

static constexpr DMAMapping<DMA1_BASE, LL_DMA_CHANNEL_7> kTim2Ch2DmaChannel;
static constexpr DMAMapping<DMA1_BASE, LL_DMA_CHANNEL_8> kSpi1RxDmaChannel;

constexpr uint32_t kClockFrequencyHz = 160000000;

void EnsureCycleCounterEnabled() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void BusyWaitUs(uint32_t delay_us) {
  const uint32_t cycles_per_us = SystemCoreClock / 1'000'000U;
  const uint32_t target_cycles = delay_us * cycles_per_us;
  const uint32_t start_cycles = DWT->CYCCNT;
  while ((uint32_t)(DWT->CYCCNT - start_cycles) < target_cycles) {
  }
}

void CheckClockFrequency() {
  LL_RCC_ClocksTypeDef clocks;
  LL_RCC_GetSystemClocksFreq(&clocks);
  assert(clocks.PCLK1_Frequency == kClockFrequencyHz);
}

static constexpr uint32_t NsToTimerTicks(uint32_t ns) {
  return std::ceil((1.0 * ns) / (1e9 / kClockFrequencyHz));
}

static constexpr uint32_t SpiBitsToTimerTicks(uint32_t bits) {
  constexpr uint32_t bits_per_second = 10'000'000;  // 1 Mbps
  return std::ceil((1.0 * bits) / bits_per_second * kClockFrequencyHz);
}

}  // namespace
}  // namespace blink

using namespace blink;

extern "C" {

static uint32_t csn_assert_tick;

void Setup() {
  EnsureCycleCounterEnabled();
  CheckClockFrequency();

  // The overall structure of this test is to send two SPI transactions on
  // subsequent iterations of TIM2. We want to do this in a way that is
  // compatible with the AS5048A, which means that we need to assert CSn for
  // 350ns before starting the SPI transaction, and then we need to keep it
  // asserted for 50ns after the transaction is complete, followed by a 350ns
  // deasserted inter-command delay.
  //
  // CH1's output will serve as the CSn signal. It is asserted when the channel
  // compare value is reached. That means when the timer is idle (e.g. at the
  // end of the PSI txn), the output will be deasserted. The inter-command delay
  // is implemented by setting the compare value for CH1 to be 350ns after 0.
  // To prevent the first SPI transaction from being incorrectly delayed, we
  // will set the timer to one less than this value when setting up the first
  // transaction.
  //
  // CH2's DMA will serve as the SPI TX DMA. Because we are only writing a
  // single half-word value to the SPI DR register, we do not need the SPI's
  // mechanism for requesting more data on the DMA channel. Thus we do not need
  // to use the SPI TX DMA.

  // Note: the experimentally we've observed that there is about 100ns of lag
  // from when the DMA starts to when the SPI clock actually ticks. We account
  // for that by subtracting from the planned delay.
  csn_assert_tick = NsToTimerTicks(350);
  auto spi_start = csn_assert_tick + NsToTimerTicks(250);
  // Note: the SPI transaction seems to take slightly longer than would be
  // expected from the number of bits and the configured speed. Therefore, we
  // add a small amount of additional delay here to make sure that we respect
  // the AS5048A SCLK -> CSn high timing requirement.
  auto csn_deassert = spi_start + SpiBitsToTimerTicks(16) + NsToTimerTicks(150);
  LL_TIM_OC_SetCompareCH1(TIM2, csn_assert_tick);
  LL_TIM_OC_SetCompareCH2(TIM2, spi_start);
  LL_TIM_SetAutoReload(TIM2, csn_deassert);
  LL_TIM_EnableDMAReq_CC2(TIM2);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);

  LL_DMA_EnableIT_TC(kSpi1RxDmaChannel.dma(), kSpi1RxDmaChannel.channel());

  LL_SPI_EnableDMAReq_RX(SPI1);
  LL_SPI_Enable(SPI1);
}

void Loop() {
  while (true) {
    spi_dma_complete = false;
    LL_TIM_DisableCounter(TIM2);

    kTim2Ch2DmaChannel.Disable();
    kSpi1RxDmaChannel.Disable();

    // Drain any stale data in the rx fifo.
    while (LL_SPI_IsActiveFlag_RXNE(SPI1)) {
      (void)LL_SPI_ReceiveData16(SPI1);
    }
    (void)SPI1->SR;

    static const char spi_tx_data[] = {0xFF, 0xFF, 0x00, 0x00};
    static char spi_rx_data[sizeof(spi_tx_data)];

    kTim2Ch2DmaChannel.SetTransfer(spi_tx_data, SPI1->DR);
    kTim2Ch2DmaChannel.ClearAllInterrupts();
    kSpi1RxDmaChannel.SetTransfer(SPI1->DR, spi_rx_data);
    kSpi1RxDmaChannel.ClearAllInterrupts();
    kSpi1RxDmaChannel.Enable();

    LL_TIM_DisableDMAReq_CC2(TIM2);
    LL_TIM_ClearFlag_CC2(TIM2);
    LL_TIM_EnableDMAReq_CC2(TIM2);
    kTim2Ch2DmaChannel.Enable();

    // Set the counter such that it will repeat and generate an update event
    // almost immediately after starting.
    LL_TIM_SetOnePulseMode(TIM2, LL_TIM_ONEPULSEMODE_REPETITIVE);
    LL_TIM_SetCounter(TIM2, csn_assert_tick - 1);

    // Reset our "we're working" GPIO.
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);

    // Enable the counter, kicking off the process.
    LL_TIM_EnableCounter(TIM2);

    // Busy wait.
    while (!spi_dma_complete) {
    }
    assert(!spi_dma_error);

    // Delay for 10 usec.
    BusyWaitUs(10);
  }
}

// The DMA completion handler for the SPI rx channel.
void DMA1_Channel8_IRQHandler() {
  if (LL_DMA_IsActiveFlag_TC8(DMA1)) [[likely]] {
    // Prevent further CSn assert DMAs based on this SPI transaction. Note that
    // we'll also set OPM below to turn off the timer after this cycle, but if
    // we miss that it's fine since there will be no CSn assert and nothing in
    // the SPI DMAs. (We would just do a noop SPI DMA request and a noop
    // deassert of CSn).
    LL_TIM_SetOnePulseMode(TIM2, LL_TIM_ONEPULSEMODE_SINGLE);
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6);
    LL_DMA_ClearFlag_TC8(DMA1);
    spi_dma_complete = true;
  } else if (LL_DMA_IsActiveFlag_TE8(DMA1)) {
    LL_DMA_ClearFlag_TE8(DMA1);
    spi_dma_error = true;
  }
}

}  // extern "C"
