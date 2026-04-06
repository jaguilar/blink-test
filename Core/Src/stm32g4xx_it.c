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
extern TIM_HandleTypeDef htim17;

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

void HardFault_Handler_C(uint32_t *stacked_regs)
{
    uint32_t r0  = stacked_regs[0];
    uint32_t r1  = stacked_regs[1];
    uint32_t r2  = stacked_regs[2];
    uint32_t r3  = stacked_regs[3];
    uint32_t r12 = stacked_regs[4];
    uint32_t lr  = stacked_regs[5];
    uint32_t pc  = stacked_regs[6];
    uint32_t xpsr = stacked_regs[7];

    printf("\r\n--- HardFault Handler ---\r\n");
    printf("HFSR: 0x%08x\r\n", (unsigned int)SCB->HFSR);
    printf("CFSR: 0x%08x\r\n", (unsigned int)SCB->CFSR);
    printf("Stacked R0:  0x%08x\r\n", (unsigned int)r0);
    printf("Stacked R1:  0x%08x\r\n", (unsigned int)r1);
    printf("Stacked R2:  0x%08x\r\n", (unsigned int)r2);
    printf("Stacked R3:  0x%08x\r\n", (unsigned int)r3);
    printf("Stacked R12: 0x%08x\r\n", (unsigned int)r12);
    printf("Stacked LR:  0x%08x\r\n", (unsigned int)lr);
    printf("Stacked PC:  0x%08x\r\n", (unsigned int)pc);
    printf("Stacked PSR: 0x%08x\r\n", (unsigned int)xpsr);

    if (SCB->HFSR & (1 << 31)) {
        printf("Cause: FORCED (Forced Hard Fault)\r\n");
    }
    if (SCB->HFSR & (1 << 30)) {
        printf("Cause: VECTTBL (Vector Table Read Fault)\r\n");
    }

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
__attribute__((naked)) void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  __asm volatile (
    "tst lr, #4\n"
    "ite eq\n"
    "mrseq r0, msp\n"
    "mrsne r0, psp\n"
    "b HardFault_Handler_C\n"
  );
  /* USER CODE END HardFault_IRQn 0 */
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
__attribute__((naked)) void UsageFault_Handler(void)
{
  __asm volatile (
    "tst lr, #4\n"
    "ite eq\n"
    "mrseq r0, msp\n"
    "mrsne r0, psp\n"
    "b UsageFault_Handler_C\n"
  );
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

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
