/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim17;

static void UART_SendStringPolling(const char* s) {
  // Ensure GPIOC and USART1 clocks are enabled
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  // Configure PC4 as AF7 (USART1_TX)
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOC, LL_GPIO_PIN_4, LL_GPIO_AF_7);

  // Basic 115200 config based on current clock
  uint32_t freq = 16000000;  // Default HSI
  if (LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    freq = 160000000;
  }

  LL_USART_SetBaudRate(USART1, freq, LL_USART_PRESCALER_DIV1,
                       LL_USART_OVERSAMPLING_16, 115200);
  LL_USART_EnableDirectionTx(USART1);
  LL_USART_Enable(USART1);

  while (*s) {
    while (!LL_USART_IsActiveFlag_TXE(USART1));
    LL_USART_TransmitData8(USART1, (uint8_t)*s++);
  }
  while (!LL_USART_IsActiveFlag_TC(USART1));
}

/* USER CODE BEGIN EV */
__attribute__((used)) void UsageFault_Handler_C(uint32_t *stacked_regs)
{
    uint32_t r0  = stacked_regs[0];
    uint32_t r1  = stacked_regs[1];
    uint32_t r2  = stacked_regs[2];
    uint32_t r3  = stacked_regs[3];
    uint32_t r12 = stacked_regs[4];
    uint32_t lr  = stacked_regs[5];
    uint32_t pc  = stacked_regs[6];
    uint32_t xpsr = stacked_regs[7];

    printf("\r\n--- UsageFault Handler ---\r\n");
    printf("CFSR: 0x%08x\r\n", (unsigned int)SCB->CFSR);
    printf("Stacked R0:  0x%08x\r\n", (unsigned int)r0);
    printf("Stacked R1:  0x%08x\r\n", (unsigned int)r1);
    printf("Stacked R2:  0x%08x\r\n", (unsigned int)r2);
    printf("Stacked R3:  0x%08x\r\n", (unsigned int)r3);
    printf("Stacked R12: 0x%08x\r\n", (unsigned int)r12);
    printf("Stacked LR:  0x%08x\r\n", (unsigned int)lr);
    printf("Stacked PC:  0x%08x\r\n", (unsigned int)pc);
    printf("Stacked PSR: 0x%08x\r\n", (unsigned int)xpsr);

    if (SCB->CFSR & (1 << 17)) {
        printf("Cause: INVSTATE (Invalid State)\r\n");
    }
    if (SCB->CFSR & (1 << 16)) {
        printf("Cause: UNDEFINSTR (Undefined Instruction)\r\n");
    }
    if (SCB->CFSR & (1 << 18)) {
        printf("Cause: INVPC (Invalid PC load)\r\n");
    }
    if (SCB->CFSR & (1 << 19)) {
        printf("Cause: NOCP (No Coprocessor - Check FPU settings)\r\n");
    }

    while (1);
}

void HardFault_Handler_C(uint32_t* stacked_regs, uint32_t lr_val) {
  uint32_t pc = stacked_regs[6];
  char buf[128];
  UART_SendStringPolling("\r\n--- HardFault ---\r\n");
  sprintf(buf, "PC: 0x%08lx, LR: 0x%08lx, SCB->CFSR: 0x%08lx\r\n", pc, lr_val,
          SCB->CFSR);
  UART_SendStringPolling(buf);
  while (1);
}
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void) {
  /* USER CODE BEGIN HardFault_IRQn 0 */
  __asm volatile(
      "tst lr, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "mov r1, lr\n"
      "b HardFault_Handler_C\n");
  /* USER CODE END HardFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  printf("MemManage Handler! CFSR: 0x%08lx, MMFAR: 0x%08lx\r\n", SCB->CFSR, SCB->MMFAR);
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  printf("BusFault Handler! CFSR: 0x%08lx, BFAR: 0x%08lx\r\n", SCB->CFSR, SCB->BFAR);
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void) {
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  UART_SendStringPolling("UsageFault!\r\n");
  /* USER CODE END UsageFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM17 global interrupt.
  */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */
  if (htim17.Instance != NULL)
  {
    HAL_TIM_IRQHandler(&htim17);
  }
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt / USART1 wake-up
 * interrupt through EXTI line 25.
 */
void USART1_IRQHandler(void) {
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
 * @brief This function handles DMA2 channel1 global interrupt.
 */
void DMA2_Channel1_IRQHandler(void) {
  /* USER CODE BEGIN DMA2_Channel1_IRQn 0 */

  /* USER CODE END DMA2_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Channel1_IRQn 1 */

  /* USER CODE END DMA2_Channel1_IRQn 1 */
}

/**
 * @brief This function handles DMA2 channel2 global interrupt.
 */
void DMA2_Channel2_IRQHandler(void) {
  /* USER CODE BEGIN DMA2_Channel2_IRQn 0 */

  /* USER CODE END DMA2_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Channel2_IRQn 1 */

  /* USER CODE END DMA2_Channel2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
  }
}

/* USER CODE END 1 */
