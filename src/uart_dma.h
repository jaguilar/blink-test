#ifndef UART_DMA_H
#define UART_DMA_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void UartDma_Init(UART_HandleTypeDef* huart);
void UartDma_BufferInit(void);

#ifdef __cplusplus
}
#endif

#endif // UART_DMA_H
