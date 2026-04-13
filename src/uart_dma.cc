#include "uart_dma.h"

#include <cstdio>

#include "dma.h"
#include "usart.h"
#define LWRB_DISABLE_ATOMIC
#include <string.h>

#include <algorithm>
#include <atomic>

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "lwrb/lwrb.h"
#include "task.h"

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

static std::atomic<bool> tx_dma_busy{false};
static std::atomic<size_t> tx_dma_pending_len{0};
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

    if (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data, (uint16_t)len) !=
        HAL_OK) {
      // If we failed to start, reset busy flag so we can try again
      tx_dma_busy = false;
      tx_dma_pending_len = 0;
    }
  }
}

static bool CanBlock(void) {
  return (xPortIsInsideInterrupt() == pdFALSE &&
          xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);
}

extern "C" int _write(int file, char* ptr, int len) {
  (void)file;
  if (len <= 0) return 0;

  bool can_block = CanBlock();
  size_t total_written = 0;

  if (can_block) {
    // Blocking context: loop until everything is written
    while (total_written < (size_t)len) {
      taskENTER_CRITICAL();
      size_t written = lwrb_write(&tx_ring_buffer, ptr + total_written,
                                  (size_t)len - total_written);
      total_written += written;
      if (!tx_dma_busy) {
        Start_Tx_DMA();
      }
      taskEXIT_CRITICAL();

      if (total_written < (size_t)len) {
        // Wait for buffer space (triggered by TxCpltCallback)
        osEventFlagsWait(uart_event_flags, UART_EVENT_TX_COMPLETE,
                         osFlagsWaitAny, 100);
      }
    }
  } else {
    // Non-blocking context (ISR or early boot): write with "UARTFULL" warning
    // if needed
    size_t free_space = lwrb_get_free(&tx_ring_buffer);
    static const char warning[] = "UARTFULL\n";
    size_t warn_len = strlen(warning);

    if (free_space >= (size_t)len) {
      total_written = lwrb_write(&tx_ring_buffer, ptr, (size_t)len);
    } else {
      // Not enough space for the full message.
      // Truncate message to fit "UARTFULL\n" if possible.
      size_t msg_to_write =
          (free_space > warn_len) ? (free_space - warn_len) : 0;
      size_t warn_to_write =
          (free_space > msg_to_write) ? (free_space - msg_to_write) : 0;
      if (warn_to_write > warn_len) warn_to_write = warn_len;

      if (msg_to_write > 0) {
        lwrb_write(&tx_ring_buffer, ptr, msg_to_write);
      }
      if (warn_to_write > 0) {
        lwrb_write(&tx_ring_buffer, warning, warn_to_write);
      }
      total_written = (size_t)len;  // Pretend we wrote it all to satisfy printf
    }

    if (!tx_dma_busy) {
      Start_Tx_DMA();
    }
  }

  return static_cast<int>(total_written);
}

extern "C" int _read(int file, char* ptr, int len) {
  (void)file;
  if (len <= 0) return 0;

  bool can_block = CanBlock();

  if (can_block) {
    while (lwrb_get_full(&rx_ring_buffer) == 0) {
      // Wait for data event
      osEventFlagsWait(uart_event_flags, UART_EVENT_RX_DATA, osFlagsWaitAny,
                       osWaitForever);
    }
    taskENTER_CRITICAL();
    size_t read_bytes = lwrb_read(&rx_ring_buffer, ptr, (size_t)len);
    taskEXIT_CRITICAL();
    return (int)read_bytes;
  } else {
    // Non-blocking: just read whatever is there
    return (int)lwrb_read(&rx_ring_buffer, ptr, (size_t)len);
  }
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
