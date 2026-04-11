#include "uart_dma.h"

#include <cstdio>

#include "dma.h"
#include "usart.h"
#define LWRB_DISABLE_ATOMIC
#include <string.h>

#include <algorithm>

#include "cmsis_os2.h"
#include "lwrb/lwrb.h"

#define UART_DMA_RX_BUFFER_SIZE 1024
#define UART_DMA_TX_BUFFER_SIZE 1024

// Buffers for DMA and lwrb
static uint8_t rx_dma_buffer[UART_DMA_RX_BUFFER_SIZE];
static uint8_t rx_ring_buffer_data[UART_DMA_RX_BUFFER_SIZE];
static uint8_t tx_ring_buffer_data[UART_DMA_TX_BUFFER_SIZE];

static lwrb_t rx_ring_buffer;
static lwrb_t tx_ring_buffer;

static osEventFlagsId_t uart_event_flags;
#define UART_EVENT_RX_DATA 0x01
#define UART_EVENT_TX_COMPLETE 0x02

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

static volatile bool tx_dma_busy = false;
static volatile size_t tx_dma_pending_len = 0;
static size_t last_rx_pos = 0;

extern "C" void UartDma_Init(void) {
  // 1. Initialize lwrb buffers
  lwrb_init(&rx_ring_buffer, rx_ring_buffer_data, sizeof(rx_ring_buffer_data));
  lwrb_init(&tx_ring_buffer, tx_ring_buffer_data, sizeof(tx_ring_buffer_data));

  // 2. Create Event Flags
  uart_event_flags = osEventFlagsNew(NULL);

  // 3. Start RX DMA using ReceiveToIdle_DMA (supports IDLE line detection)
  // The DMA handles are already initialized and linked in MX_USART1_UART_Init
  // (MX_USART1_UART_MspInit)
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buffer, UART_DMA_RX_BUFFER_SIZE);
}

static void Start_Tx_DMA(void) {
  if (tx_dma_busy) return;

  size_t len = lwrb_get_linear_block_read_length(&tx_ring_buffer);
  if (len > 0) {
    tx_dma_busy = true;
    tx_dma_pending_len = len;
    void* data = lwrb_get_linear_block_read_address(&tx_ring_buffer);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data, (uint16_t)len);
  }
}

extern "C" int _write(int file, char* ptr, int len) {
  (void)file;

  // Non-blocking write to ring buffer
  // In a more robust implementation, we might block if the ring buffer is full
  size_t written = lwrb_write(&tx_ring_buffer, ptr, (size_t)len);

  // Start DMA if not already busy
  osKernelLock();  // Protect flag check
  if (!tx_dma_busy) {
    Start_Tx_DMA();
  }
  osKernelUnlock();

  return (int)written;
}

extern "C" int _read(int file, char* ptr, int len) {
  (void)file;
  if (len <= 0) return 0;

  while (lwrb_get_full(&rx_ring_buffer) == 0) {
    // Wait for data event
    osEventFlagsWait(uart_event_flags, UART_EVENT_RX_DATA, osFlagsWaitAny,
                     osWaitForever);
  }

  // Critical section for reading from ring buffer as requested
  osKernelLock();
  size_t read_bytes = lwrb_read(&rx_ring_buffer, ptr, (size_t)len);
  osKernelUnlock();

  return (int)read_bytes;
}

// HAL Callbacks
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,
                                           uint16_t Size) {
  if (huart->Instance == USART1) {
    size_t current_pos = Size;
    size_t new_data_len = 0;

    if (current_pos > last_rx_pos) {
      new_data_len = current_pos - last_rx_pos;
      lwrb_write(&rx_ring_buffer, &rx_dma_buffer[last_rx_pos], new_data_len);
    } else if (current_pos < last_rx_pos) {
      // Wrapped around
      new_data_len = UART_DMA_RX_BUFFER_SIZE - last_rx_pos;
      lwrb_write(&rx_ring_buffer, &rx_dma_buffer[last_rx_pos], new_data_len);
      if (current_pos > 0) {
        lwrb_write(&rx_ring_buffer, &rx_dma_buffer[0], current_pos);
        new_data_len += current_pos;
      }
    }

    last_rx_pos = current_pos;
    if (new_data_len > 0) {
      osEventFlagsSet(uart_event_flags, UART_EVENT_RX_DATA);
    }
  }
}

// In circular mode, RxCpltCallback is also useful if we don't use
// ReceiveToIdle, but ReceiveToIdle uses RxEventCallback even for full buffer
// events.
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  // HAL_UARTEx_RxEventCallback handles this usually if using ReceiveToIdle_DMA
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART1) {
    // Mark previous linear block as read

    // HAL_UART_Transmit_DMA only finishes one linear block.
    // We know we just finished reading a linear block from lwrb.
    lwrb_skip(&tx_ring_buffer, tx_dma_pending_len);

    tx_dma_pending_len = 0;
    tx_dma_busy = false;
    Start_Tx_DMA();

    osEventFlagsSet(uart_event_flags, UART_EVENT_TX_COMPLETE);
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART1) {
    // Reset DMA and UART if error occurs
    HAL_UART_DMAStop(huart);
    last_rx_pos = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_dma_buffer, UART_DMA_RX_BUFFER_SIZE);
  }
}
