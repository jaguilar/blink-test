#ifndef STFOC_AS5048A_SPI_SENSOR_H
#define STFOC_AS5048A_SPI_SENSOR_H

#include <cinttypes>
#include <cstdio>

#include "cmsis_os2.h"
#include "common/base_classes/Sensor.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"
#include "foc_types.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"

namespace stfoc {

struct AsyncTimerSpiConfig {
  uintptr_t spi_base;
  uintptr_t spi_rx_dma_base;
  uint32_t spi_rx_dma_channel;
  uintptr_t spi_tx_dma_base;
  uint32_t spi_tx_dma_channel;
  uintptr_t timer_base;
  uint32_t timer_channel_csn;
  GpioEntry csn_pin;
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
uint32_t TimNChDmaReq(TIM_TypeDef* tim, uint32_t channel);
void BusyWaitNs(uint32_t ns);
}  // namespace internal

template <AsyncTimerSpiConfig config>
class AsyncTimerAS5048ASpi : public Sensor {
 public:
  void init() override;

  template <uintptr_t leader_timer_base>
  void AsyncReadFromMotorUpdate();

  void DmaComplete();

  float getSensorAngle() override { return angle_prev; }
  float getAngle() override { return total_count * (2.0f * _PI / 0x4000); }
  double getPreciseAngle() override {
    return total_count * (2.0 * (double)_PI / 16384.0);
  }
  float getMechanicalAngle() override { return angle_prev; }
  int32_t getFullRotations() override { return total_count >> 14; }
  int32_t getFullRotations() const { return total_count >> 14; }

  void update() override {}

  const uint16_t* getRawRxBuf() const { return spi_rx_buf_; }

  float getVelocity() override;

  int needsSearch() override { return false; }

  uint16_t raw_angle = 0;
  int64_t total_count = 0;

 private:
  static_assert((config.timer_channel_csn == LL_TIM_CHANNEL_CH1 ||
                 config.timer_channel_csn == LL_TIM_CHANNEL_CH2 ||
                 config.timer_channel_csn == LL_TIM_CHANNEL_CH3) &&
                "CS pin must be on timer channel 1, 2 or 3 for "
                "AsyncTimerAS5048ASpi");

  SPI_TypeDef* spi() const { return config.spi(); }

  bool TryInit();

  uint32_t csn_assert_timer_tick_;
  alignas(uint32_t) uint16_t spi_rx_buf_[2];
  volatile bool pending_dma_ = false;
  int64_t vel_last_count = 0;
  uint32_t vel_last_us = 0;
};

template <AsyncTimerSpiConfig config>
void AsyncTimerAS5048ASpi<config>::init() {
  internal::EnableTimerClock(config.timer_base);

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
  LL_SPI_Disable(spi());
  LL_SPI_Init(spi(), &spi_init);
  LL_SPI_SetStandard(spi(), LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetRxFIFOThreshold(config.spi(), LL_SPI_RX_FIFO_HALF_FULL);
  LL_SPI_DisableNSSPulseMgt(config.spi());
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

  // Set DMA requests once during init
  uint32_t trigger_channel =
      (config.timer_base == TIM15_BASE || config.timer_base == TIM16_BASE ||
       config.timer_base == TIM17_BASE)
          ? LL_TIM_CHANNEL_CH1
          : LL_TIM_CHANNEL_CH4;
  LL_DMA_SetPeriphRequest(
      config.tx_dma(), config.spi_tx_dma_channel,
      internal::TimNChDmaReq(config.tim(), trigger_channel));
  LL_DMA_SetPeriphRequest(config.rx_dma(), config.spi_rx_dma_channel,
                          internal::SpiNRxDmaReq(config.spi()));

  csn_assert_timer_tick_ = internal::NsToTimerTicks(500);
  auto spi_start_timer_tick =
      csn_assert_timer_tick_ + internal::NsToTimerTicks(250);
  constexpr int ns_per_bit = 100;
  auto csn_deassert_timer_tick =
      spi_start_timer_tick +
      // Temporarily double the time to allow halving the clock rate.
      internal::NsToTimerTicks(16 * ns_per_bit) + internal::NsToTimerTicks(200);

  LL_TIM_InitTypeDef timer_init = {
      .Prescaler = 0,
      .CounterMode = LL_TIM_COUNTERMODE_UP,
      .Autoreload = csn_deassert_timer_tick,
      .ClockDivision = LL_TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 1,
  };
  // Ensure the timer has a repetition counter (RCR).
  // On STM32G4, TIM1, 8, 15, 16, 17, 20 have RCR.
  // We can't easily static_assert on a pointer, so we'll do a runtime check.
  assert((config.timer_base == TIM1_BASE || config.timer_base == TIM8_BASE ||
          config.timer_base == TIM15_BASE || config.timer_base == TIM16_BASE ||
          config.timer_base == TIM17_BASE || config.timer_base == TIM20_BASE) &&
         "Selected timer must have a repetition counter (RCR) for 2nd pulse.");
  LL_TIM_Init(config.tim(), &timer_init);

  LL_TIM_OC_InitTypeDef oc_init = {
      .OCMode = LL_TIM_OCMODE_PWM2,
      .OCState = LL_TIM_OCSTATE_ENABLE,
      .CompareValue = csn_assert_timer_tick_ + 1,
      .OCPolarity = LL_TIM_OCPOLARITY_LOW,
      .OCIdleState = LL_TIM_OCIDLESTATE_HIGH,
  };
  LL_TIM_OC_Init(config.tim(), config.timer_channel_csn, &oc_init);
  LL_TIM_CC_EnableChannel(config.tim(), config.timer_channel_csn);

  LL_TIM_OC_InitTypeDef oc_trigger_init = {
      .OCMode = LL_TIM_OCMODE_FROZEN,
      .OCState = LL_TIM_OCSTATE_ENABLE,
      .CompareValue = spi_start_timer_tick,
  };
  // Use CH1 for trigger on small timers (like TIM15), otherwise CH4.
  trigger_channel =
      (config.timer_base == TIM15_BASE || config.timer_base == TIM16_BASE ||
       config.timer_base == TIM17_BASE)
          ? LL_TIM_CHANNEL_CH1
          : LL_TIM_CHANNEL_CH4;
  LL_TIM_OC_Init(config.tim(), trigger_channel, &oc_trigger_init);
  LL_TIM_SetSlaveMode(config.tim(), LL_TIM_SLAVEMODE_TRIGGER);
  LL_TIM_SetOnePulseMode(config.tim(), LL_TIM_ONEPULSEMODE_SINGLE);
  if (IS_TIM_BREAK_INSTANCE(config.tim())) {
    LL_TIM_EnableAllOutputs(config.tim());
  }
  LL_TIM_EnableCounter(config.tim());

  int tries = 0;
  while (!TryInit()) {
    tries++;
    if (tries < 10) {
      osDelay(10);
    } else {
      osDelay(1000);
    }
  }

  // Read one more time to get the current angle for initializing total_count
  uint32_t initial_angle = internal::SyncReadSpi(config, 0x3FFF) & 0x3FFF;
  this->raw_angle = initial_angle;
  this->total_count = initial_angle;
  this->angle_prev = static_cast<float>(initial_angle) * (2.0f * _PI / 0x4000);
  this->full_rotations = 0;
  this->vel_last_count = initial_angle;
  this->vel_last_us = _micros();

  // Once initialization is done, set the PWM mode to mode 2 (inactive until
  // timer reached) to set up for async SPI communication.
  LL_TIM_OC_SetMode(config.tim(), config.timer_channel_csn, LL_TIM_OCMODE_PWM2);
}

template <AsyncTimerSpiConfig config>
bool AsyncTimerAS5048ASpi<config>::TryInit() {
  // Wait for the sensor startup delay (10ms).
  osDelay(10);

  // 1. Clear any pending errors from power-on/previous states.
  // Accessing 0x0001 clears the Error Flag (EF) and returns specific error
  // types.
  uint32_t error_reg_raw = internal::SyncReadSpi(config, 0x0001);
  const bool parity_err = (error_reg_raw & (1 << 2)) != 0;
  const bool command_err = (error_reg_raw & (1 << 1)) != 0;
  const bool framing_err = (error_reg_raw & (1 << 0)) != 0;
  std::printf("Err status: parity=%d, command=%d, framing=%d\n", parity_err,
              command_err, framing_err);

  // 2. Read the actual Diagnostic and AGC data from the correct register
  // (0x3FFD). We read twice because SPI returns the data from the PREVIOUS
  // command.
  uint32_t diag_raw = internal::SyncReadSpi(config, 0x3FFD);

  // Extract Global SPI Error Flag (Bit 14 of the read package)
  bool spi_error = (diag_raw & (1 << 14)) != 0;

  // Extract Diagnostic bits from the data portion (Bits 0-13)
  uint8_t agc = diag_raw & 0xFF;                      // Bits 0-7
  bool offset_fin = (diag_raw & (1 << 8)) != 0;       // OCF
  bool cordic_ovf = (diag_raw & (1 << 9)) != 0;       // COF
  bool field_too_high = (diag_raw & (1 << 11)) != 0;  // Comp High
  bool field_too_low = (diag_raw & (1 << 10)) != 0;   // Comp Low

  std::printf("AS5048A Diagnostics (Register 0x3FFD): 0x%04" PRIX32 "\n",
              diag_raw);

  if (spi_error) {
    // If bit 14 is set, we should check the error_reg_raw (from 0x0001) for
    // details
    std::printf("  [ERROR] SPI/Command Error detected!\n");
  }

  std::printf("  AGC Value: %d (0=Strong, 255=Weak)\n", agc);

  if (field_too_high) {
    std::printf("  [WARNING] Magnetic field too STRONG (Magnet too close)\n");
  }
  if (field_too_low) {
    std::printf("  [WARNING] Magnetic field too WEAK (Magnet too far)\n");
  }
  {
    std::printf("  [OK] Magnetic field strength is optimal.\n");
  }

  if (cordic_ovf)
    std::printf("  [ERROR] CORDIC Overflow - Angle data invalid!\n");
  if (!offset_fin)
    std::printf("  [INFO] Offset compensation still in progress...\n");

  // 3. Read Initial Angle (0x3FFF)
  internal::SyncReadSpi(config, 0x3FFF);
  uint32_t angle_raw = internal::SyncReadSpi(config, 0x3FFF);
  // Mask to 14 bits as bits 14/15 are status/parity [cite: 83, 90]
  uint16_t clean_angle = angle_raw & 0x3FFF;

  (void)internal::SyncReadSpi(config, 0x0001);

  std::printf("AS5048A Initial Angle: 0x%04" PRIX32 " (%" PRIu16 " raw)\n",
              angle_raw, clean_angle);

  return !(field_too_high || field_too_low || cordic_ovf || !offset_fin ||
           parity_err || framing_err || command_err);
}

template <AsyncTimerSpiConfig config>
void AsyncTimerAS5048ASpi<config>::DmaComplete() {
  if (!pending_dma_) {
    return;
  }
  const uint16_t raw_angle = spi_rx_buf_[1] & 0x3FFF;

  // A negative wrap around will appear to be a large positive diff (1 in the
  // 14th bit).
  const uint16_t raw_diff = (raw_angle - this->raw_angle) & 0x3FFF;
  // Move the high bit into the sign position, cast to an int16, then do a
  // signed shift right (divide by 4). Now that large positive diff looks like a
  // small negative diff.
  const int16_t signed_diff = static_cast<int16_t>(raw_diff << 2) >> 2;

  this->raw_angle = raw_angle;
  this->angle_prev = static_cast<float>(raw_angle) * (2.0f * _PI / 0x4000);
  this->total_count += signed_diff;
  this->full_rotations = total_count >> 14;

  pending_dma_ = false;
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

  uint32_t trigger_channel =
      (config.timer_base == TIM15_BASE || config.timer_base == TIM16_BASE ||
       config.timer_base == TIM17_BASE)
          ? LL_TIM_CHANNEL_CH1
          : LL_TIM_CHANNEL_CH4;
  LL_TIM_DisableDMAReq_CC1(config.tim());
  LL_TIM_DisableDMAReq_CC2(config.tim());

  LL_DMA_SetMemoryAddress(
      config.tx_dma(), config.spi_tx_dma_channel,
      reinterpret_cast<uint32_t>(AS5048ReadAngleCommandBuf()));
  LL_DMA_SetPeriphAddress(config.tx_dma(), config.spi_tx_dma_channel,
                          (uint32_t)&config.spi()->DR);
  LL_DMA_SetMemoryAddress(config.rx_dma(), config.spi_rx_dma_channel,
                          reinterpret_cast<uint32_t>(spi_rx_buf_));
  LL_DMA_SetPeriphAddress(config.rx_dma(), config.spi_rx_dma_channel,
                          (uint32_t)&config.spi()->DR);
  LL_DMA_SetDataLength(config.tx_dma(), config.spi_tx_dma_channel, 2);
  LL_DMA_SetDataLength(config.rx_dma(), config.spi_rx_dma_channel, 2);

  LL_DMA_EnableChannel(config.tx_dma(), config.spi_tx_dma_channel);
  LL_DMA_EnableChannel(config.rx_dma(), config.spi_rx_dma_channel);
  LL_SPI_EnableDMAReq_RX(config.spi());

  if (trigger_channel == LL_TIM_CHANNEL_CH1) {
    LL_TIM_EnableDMAReq_CC1(config.tim());
  } else if (trigger_channel == LL_TIM_CHANNEL_CH2) {
    LL_TIM_EnableDMAReq_CC2(config.tim());
  } else {
    LL_TIM_EnableDMAReq_CC4(config.tim());
  }

  LL_DMA_EnableIT_TC(config.rx_dma(), config.spi_rx_dma_channel);

  LL_TIM_SetTriggerInput(
      config.tim(),
      internal::GetTRGOItrValue<leader_timer_base, config.timer_base>());
  pending_dma_ = true;
}

template <AsyncTimerSpiConfig config>
float AsyncTimerAS5048ASpi<config>::getVelocity() {
  uint32_t now_us = _micros();
  float Ts = (now_us - vel_last_us) * 1e-6f;

  if (Ts < min_elapsed_time) return velocity;

  int64_t count_diff = total_count - vel_last_count;
  velocity = (float)count_diff * (2.0f * _PI / 16384.0f) / Ts;

  vel_last_count = total_count;
  vel_last_us = now_us;
  return velocity;
}

}  // namespace stfoc

#endif  // STFOC_AS5048A_SPI_SENSOR_H
