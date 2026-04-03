#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include <stdalign.h>
#include <stdint.h>
#include <sys/types.h>

#include <bit>
#include <cassert>

#include "common/base_classes/BLDCDriver.h"
#include "common/base_classes/Sensor.h"
#include "stm32g474xx.h"
#include "stm32g4xx.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_dmamux.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
#include "system_stm32g4xx.h"

namespace stfoc {

struct GpioEntry {
  uintptr_t gpio_base = 0;
  uint32_t pin = 0;
  bool active_high = true;

  GPIO_TypeDef* gpio() const {
    return reinterpret_cast<GPIO_TypeDef*>(gpio_base);
  }
};

// This structure contains the various hardware identifiers that describe a
// timer that will be used for motor control.
struct StTimerMotorConfig {
  // Note: we assume that we have exclusive ownership of the timer base.
  uintptr_t timer_base;
  GpioEntry phase1_en;  // Note: the enable phases must map 1:1 onto the timer
                        // channel phases.
  GpioEntry phase2_en;
  GpioEntry phase3_en;

  uint32_t pwm_freq;

  // The minimum time the timer outputs will spend de-asserted each cycle.
  // Note that half this dead time will occur before the update event, so
  // if you plan to take 1usec doing current sensing, better set this value to
  // at least 2000.
  uint32_t min_dead_time_nanos;

  TIM_TypeDef* timer() const {
    return reinterpret_cast<TIM_TypeDef*>(timer_base);
  }

  // Note: this value cannot be computed at compile time because it depends
  // on the system clock frequency. That said, we will assume
  uint32_t ComputeArr() const;
};

constexpr StTimerMotorConfig kTim1Motorparams = {.timer_base = TIM1_BASE};

template <StTimerMotorConfig config>
class StTimerMotorDriver : public BLDCDriver {
 public:
  static_assert(config.timer_base == TIM1_BASE ||
                    config.timer_base == TIM8_BASE ||
                    config.timer_base == TIM20_BASE,
                "Only TIM1, TIM8 and TIM20 are supported as timer bases for "
                "StTimerMotorDriver");

  StTimerMotorDriver() : BLDCDriver() {};
  ~StTimerMotorDriver() {};

  // Arduino-FOC interface.
  int init() override;
  void enable() override;
  void disable() override;
  void setPwm(float Ua, float Ub, float Uc) override;
  void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) override;

  // Causes this motor's PWM to start following the start of another motor.
  // You can provide a phase offset in terms of fractions of a revolution.
  // This may be called after the motor is enabled, which will have the effect
  // of stopping the motor until the cycle of the leader motor. This is useful
  // for ensuring that multiple motors are out of phase with each other.
  //
  // This operation claims TRGO and channel 4 of the leader motor, until the
  // first period of the leader motor has completed. The leader motor should not
  // have been started when Follow is called.
  template <StTimerMotorConfig leader_config>
  void Follow(StTimerMotorDriver<leader_config>& leader,
              float phase_offset = 0.0f);

 private:
  TIM_TypeDef* timer() const {
    return reinterpret_cast<TIM_TypeDef*>(config.timer_base);
  }

  uint32_t arr_value_;
  uint32_t max_duty_;
  bool is_follower_ = false;
};

struct AsyncTimerSpiConfig {
  uintptr_t spi_base;

  // Both DMA channels are the exclusive property of this class from the moment
  // the async read is dispatched until the DMA transfer is complete.
  // The DMA can be time-division multiplexed between instances of
  // AsyncTimerAS5048ASpi, but it cannot be shared with other users, since we
  // assume some of the DMA configuration is not adjusted after init.
  uintptr_t spi_rx_dma_base;
  uint32_t spi_rx_dma_channel;
  uintptr_t spi_tx_dma_base;
  uint32_t spi_tx_dma_channel;

  uintptr_t timer_base;

  // Which timer channel's output compare output is used as CSn.
  // Note that this may only be channels 1-3, as channel 4 is used for the SPI
  // tx DMA trigger.
  uint32_t timer_channel_csn;

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

// A buffer containing one read angle command and one nop command for the
// AS5048A.
const char* AS5048ReadAngleCommandBuf();

// This class is designed to fetch SPI readings from the AS5048A with very exact
// timing, triggered by a source timer. It takes approximately 4us to complete
// the read (2x 16 bit 10MHz SPI transactions). A given timer can be shared
// between multiple instances of this class provided no two instances have
// active SPI transactions at the same time. No other use of the timer is
// permitted.
//
// To follow the Arduino-FOC Sensor interface, update() will do synchronous,
// blocking SPI transactions if it is called before async operation commences.
template <AsyncTimerSpiConfig config>
class AsyncTimerAS5048ASpi : public Sensor {
 public:
  void init() override;

  // Schedules an asynchronous SPI read to correspond with the next UPDATE event
  // of the indicated motor timer. This requires that TRGO of the given timer be
  // set to UPDATE.
  //
  // Note: this enables the TC interrupt on the RX DMA, but it is the caller's
  // responsibility to ensure that the interrupt is handled and that update() is
  // eventually called.
  template <uintptr_t leader_timer_base>
  void AsyncReadFromMotorUpdate();

  // If no DMA request is pending, performs a blocking SPI read and updates the
  // sensor value.
  //
  // If there is a pending DMA request, update() must only be called when the
  // SPI transaction is complete and the DMA is done. In this case, update()
  // just uses the recorded value from the transaction to update the sensor
  // value and does not block.
  void update() override;

  float getSensorAngle() override { return angle_; }

  int needsSearch() override { return false; }

 private:
  static_assert(config.timer_channel_csn >= 1 &&
                config.timer_channel_csn <= 3 &&
                "CS pin must be on timer channel 1, 2 or 3 for "
                "AsyncTimerAS5048ASpi");

  SPI_TypeDef* spi() const { return config.spi(); }

  ErrorStatus SyncReadAngle(uint32_t& angle);

  float angle_ = 0.0f;  // Sensor angle in radians as of the latest update().

  uint32_t csn_assert_timer_tick_;
  char spi_rx_buf_[4];
  volatile bool pending_dma_ = false;
};

// Implementations below.

namespace internal {
void InitGpio(const GpioEntry& entry);
void GpioAssert(const GpioEntry& entry);
void GpioDeassert(const GpioEntry& entry);
void InitTimer(const StTimerMotorConfig& config);
uint32_t SyncReadSpi(const AsyncTimerSpiConfig& config, uint16_t address);

// Returns the ITR value that routes the TRGO of the given source timer to the
// given destination timer. The triggerable timers are 1, 2, 3, 4, 5, 8, 15,
// and 20. Note that this macro does not report the TIM16/17 OC triggers or any
// triggers related to HRTIM.
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

template <uintptr_t spi_base>
constexpr uint32_t SpiNRxDmaReq() {
  if constexpr (spi_base == SPI1_BASE) {
    return LL_DMAMUX_REQ_SPI1_RX;
  } else if constexpr (spi_base == SPI2_BASE) {
    return LL_DMAMUX_REQ_SPI2_RX;
  } else if constexpr (spi_base == SPI3_BASE) {
    return LL_DMAMUX_REQ_SPI3_RX;
  } else if constexpr (spi_base == SPI4_BASE) {
    return LL_DMAMUX_REQ_SPI4_RX;
  } else {
    static_assert(false, "Unsupported SPI instance in SpiNDmaReq");
    return 0;
  }
}

template <uintptr_t tim_base>
uint32_t TimNCh4DmaReq() {
  if constexpr (tim_base == TIM1_BASE) {
    return LL_DMAMUX_REQ_TIM1_CH4;
  } else if constexpr (tim_base == TIM2_BASE) {
    return LL_DMAMUX_REQ_TIM2_CH4;
  } else if constexpr (tim_base == TIM3_BASE) {
    return LL_DMAMUX_REQ_TIM3_CH4;
  } else if constexpr (tim_base == TIM4_BASE) {
    return LL_DMAMUX_REQ_TIM4_CH4;
  } else if constexpr (tim_base == TIM5_BASE) {
    return LL_DMAMUX_REQ_TIM5_CH4;
  } else if constexpr (tim_base == TIM8_BASE) {
    return LL_DMAMUX_REQ_TIM8_CH4;
  } else if constexpr (tim_base == TIM20_BASE) {
    return LL_DMAMUX_REQ_TIM20_CH4;
  } else {
    static_assert(false, "Unsupported timer instance in TimNCh4DmaReq");
  }
}

inline uint32_t NsToTimerTicks(uint32_t ns) {
#ifndef NDEBUG
  LL_RCC_ClocksTypeDef clocks;
  LL_RCC_GetSystemClocksFreq(&clocks);
  assert(SystemCoreClock == clocks.PCLK1_Frequency &&
         SystemCoreClock == clocks.PCLK2_Frequency &&
         "This software assumes that the PCLK frequencies match the system "
         "core clock frequency.");
#endif
  return std::ceil((1.0 * ns) / (1e9 / SystemCoreClock));
}

}  // namespace internal

template <StTimerMotorConfig config>
int StTimerMotorDriver<config>::init() {
  arr_value_ = config.ComputeArr();
  max_duty_ = arr_value_ - internal::NsToTimerTicks(config.min_dead_time_nanos);

  internal::InitGpio(config.phase1_en);
  internal::InitGpio(config.phase2_en);
  internal::InitGpio(config.phase3_en);
  internal::InitTimer(config);
  return 0;
}

template <StTimerMotorConfig config>
void StTimerMotorDriver<config>::enable() {
  if (!is_follower_) {
    LL_TIM_SetCounter(timer(), 0);
    LL_TIM_EnableCounter(timer());
  }
}

template <StTimerMotorConfig config>
void StTimerMotorDriver<config>::disable() {
  LL_TIM_DisableCounter(timer());
  LL_TIM_SetCounter(timer(), 0);
  for (const auto& gpio :
       {config.phase1_en, config.phase2_en, config.phase3_en}) {
    internal::GpioDeassert(gpio);
  }
}

template <StTimerMotorConfig config>
void StTimerMotorDriver<config>::setPwm(float ua, float ub, float uc) {
  auto to_compare = [&](float duty) {
    duty = std::clamp(duty, 0.0f, 1.0f);
    // Note that since we're active once we reach the compare value, to get a
    // duty of X% we must set the compare value to arr - arr * x%.
    // TODO: jaguilar - add a configurable minimum dead time here.
    return arr_value_ - static_cast<uint32_t>(std::round(max_duty_ * duty));
  };
  auto ch1_value = to_compare(ua);
  LL_TIM_OC_SetCompareCH1(timer(), ch1_value);
  auto ch2_value = to_compare(ub);
  LL_TIM_OC_SetCompareCH2(timer(), ch2_value);
  auto ch3_value = to_compare(uc);
  LL_TIM_OC_SetCompareCH3(timer(), ch3_value);
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

template <StTimerMotorConfig config>
template <StTimerMotorConfig leader_config>
void StTimerMotorDriver<config>::Follow(
    StTimerMotorDriver<leader_config>& leader, float phase_offset) {
  assert(!LL_TIM_IsEnabledCounter(leader.timer()) &&
         !LL_TIM_IsEnabledCounter(timer()) &&
         "Motors must not have been started when Follow is called");
  is_follower_ = true;

  const uint32_t ticks = phase_offset * leader.arr_value_ * 2;
  LL_TIM_SetCompareCH4(leader.timer(), ticks);
  LL_TIM_OC_SetMode(leader.timer(), LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_FROZEN);
  LL_TIM_SetTriggerOutput(leader.timer(), LL_TIM_TRGO_OC4REF);
  LL_TIM_SetSlaveMode(timer(), LL_TIM_SLAVEMODE_TRIGGER);
  LL_TIM_SetCounter(timer(), 0);

  // See RM0440 section 11.3.1 for the trigger-input-to-trgo mapping.
  LL_TIM_SetTriggerInput(
      config.timer(),
      internal::GetTRGOItrValue<leader_config.timer_base, config.timer_base>());
}

template <AsyncTimerSpiConfig config>
void AsyncTimerAS5048ASpi<config>::init() {
  LL_SPI_InitTypeDef spi_init = {
      .TransferDirection = LL_SPI_FULL_DUPLEX,
      .Mode = LL_SPI_MODE_MASTER,
      .DataWidth = LL_SPI_DATAWIDTH_16BIT,
      .ClockPolarity = LL_SPI_POLARITY_LOW,
      .ClockPhase = LL_SPI_PHASE_1EDGE,
      .NSS = LL_SPI_NSS_SOFT,
      .BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16,
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
      .MemoryOrM2MDstDataSize = LL_DMA_PDATAALIGN_HALFWORD,
      .NbData = 2,
      .Priority = LL_DMA_PRIORITY_MEDIUM,
  };
  LL_DMA_Init(config.rx_dma(), config.spi_rx_dma_channel, &rx_dma_init);

  LL_DMA_InitTypeDef tx_dma_init = {
      .Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
      .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
      .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
      .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD,
      .MemoryOrM2MDstDataSize = LL_DMA_PDATAALIGN_HALFWORD,
      .NbData = 2,
      .Priority = LL_DMA_PRIORITY_MEDIUM,
  };
  LL_DMA_Init(config.tx_dma(), config.spi_tx_dma_channel, &tx_dma_init);

  // The overall structure of the async SPI mode here is to send two SPI
  // transactions on subsequent iterations of the timer. We want to do this in a
  // way that is compatible with the AS5048A, which means that we need to assert
  // CSn for 350ns before starting the SPI transaction, and then we need to keep
  // it asserted for 50ns after the transaction is complete, followed by a 350ns
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
  csn_assert_timer_tick_ = internal::NsToTimerTicks(350);
  auto spi_start_timer_tick =
      csn_assert_timer_tick_ + internal::NsToTimerTicks(250);
  // Note: the SPI transaction seems to take slightly longer than would be
  // expected from the number of bits and the configured speed. Therefore, we
  // add a small amount of additional delay here to make sure that we respect
  // the AS5048A SCLK -> CSn high timing requirement.
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

  // Note: this may be initialized multiple times if there are multiple sensors,
  // but that doesn't matter because the configuration is idempotent.
  LL_TIM_OC_InitTypeDef oc4_init = {
      .OCMode = LL_TIM_OCMODE_FROZEN,
      .OCState = LL_TIM_OCSTATE_ENABLE,
      .CompareValue = spi_start_timer_tick,
  };
  LL_TIM_OC_Init(config.tim(), LL_TIM_CHANNEL_CH4, &oc4_init);
  // Note: we have to set the trigger itself in AsyncReadFromMotorUpdate since
  // the timer may be shared between multiple ASyncTimerAS5048ASpi instances.
  LL_TIM_SetSlaveMode(config.tim(), LL_TIM_SLAVEMODE_TRIGGER);
}

template <AsyncTimerSpiConfig config>
void AsyncTimerAS5048ASpi<config>::update() {
  uint16_t raw_angle;
  if (pending_dma_) {
    // Cause the timer to stop ticking at the end of whatever its current cycle
    // is. This will leave our CSn pin deasserted.
    LL_TIM_OC_SetMode(config.tim(), config.timer_channel_csn,
                      LL_TIM_OCMODE_FORCED_INACTIVE);
    LL_TIM_SetOnePulseMode(config.tim(), LL_TIM_ONEPULSEMODE_SINGLE);

    // Note: we assume that the DMA transaction has completed by the time
    // update() is called, so we can just read the value from the SPI data
    // register.
    raw_angle =
        ((static_cast<uint16_t>(spi_rx_buf_[2]) << 8) | spi_rx_buf_[3]) &
        0x3FFF;
    pending_dma_ = false;
  } else {
    raw_angle = internal::SyncReadSpi(config, 0x3FFF);
  }
  angle_ = static_cast<float>(raw_angle) /
           (4096.0f * 2.0f * static_cast<float>(M_PI));

  Sensor::update();
}

template <AsyncTimerSpiConfig config>
template <uintptr_t leader_timer_base>
inline void AsyncTimerAS5048ASpi<config>::AsyncReadFromMotorUpdate() {
  TIM_TypeDef* motor_timer = reinterpret_cast<TIM_TypeDef*>(leader_timer_base);

  // We have to wait for the completion of the previous SPI transaction before
  // we can start ours.
  while (LL_TIM_IsEnabledCounter(config.tim())) {
  }

  LL_TIM_SetTriggerOutput(motor_timer, LL_TIM_TRGO_UPDATE);

  // Disable all the DMAs and clear the DMAReq for the CC channel to cancel
  // any latched requests.
  LL_DMA_DisableChannel(config.tx_dma(), config.spi_tx_dma_channel);
  LL_DMA_DisableChannel(config.rx_dma(), config.spi_rx_dma_channel);
  LL_SPI_DisableDMAReq_RX(config.spi());
  LL_TIM_DisableDMAReq_CC4(config.tim());

  // Set the read address of the TX DMA and the write address of the RX DMA.
  LL_DMA_SetMemoryAddress(
      config.tx_dma(), config.spi_tx_dma_channel,
      reinterpret_cast<uint32_t>(AS5048ReadAngleCommandBuf()));
  LL_DMA_SetPeriphAddress(config.tx_dma(), config.spi_tx_dma_channel,
                          (uint32_t)&config.spi()->DR);
  LL_DMA_SetPeriphRequest(config.tx_dma(), config.spi_tx_dma_channel,
                          internal::TimNCh4DmaReq<config.timer_base>());
  LL_DMA_SetMemoryAddress(config.rx_dma(), config.spi_rx_dma_channel,
                          reinterpret_cast<uint32_t>(spi_rx_buf_));
  LL_DMA_SetPeriphAddress(config.rx_dma(), config.spi_rx_dma_channel,
                          (uint32_t)&config.spi()->DR);
  LL_DMA_SetPeriphRequest(config.rx_dma(), config.spi_rx_dma_channel,
                          internal::SpiNRxDmaReq<config.spi_base>());
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

  // Turn on our CSn pin (when the timer is running). Normally it's forced
  // inactive so that if the timer is used by another SPI it doesn't cause
  // problems.
  LL_TIM_OC_SetMode(config.tim(), config.timer_channel_csn, LL_TIM_OCMODE_PWM2);

  pending_dma_ = true;
}

}  // namespace stfoc

#endif
