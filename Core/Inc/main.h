/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIMER_PERIOD 5666
#define M1_PHASEA_SENSE_Pin LL_GPIO_PIN_0
#define M1_PHASEA_SENSE_GPIO_Port GPIOA
#define M1_PHASEB_SENSE_Pin LL_GPIO_PIN_1
#define M1_PHASEB_SENSE_GPIO_Port GPIOA
#define ENC_SCLK_Pin LL_GPIO_PIN_5
#define ENC_SCLK_GPIO_Port GPIOA
#define ENC_MISO_Pin LL_GPIO_PIN_6
#define ENC_MISO_GPIO_Port GPIOA
#define ENC_MOSI_Pin LL_GPIO_PIN_7
#define ENC_MOSI_GPIO_Port GPIOA
#define M1_EN_Pin LL_GPIO_PIN_0
#define M1_EN_GPIO_Port GPIOB
#define M1_CAL_Pin LL_GPIO_PIN_1
#define M1_CAL_GPIO_Port GPIOB
#define ENC_CSN_Pin LL_GPIO_PIN_15
#define ENC_CSN_GPIO_Port GPIOB
#define M1_CAL_GPIO_Pin LL_GPIO_PIN_6
#define M1_CAL_GPIO_GPIO_Port GPIOC
#define M1_PHASEA_OUT_Pin LL_GPIO_PIN_8
#define M1_PHASEA_OUT_GPIO_Port GPIOA
#define M1_PHASEB_OUT_Pin LL_GPIO_PIN_9
#define M1_PHASEB_OUT_GPIO_Port GPIOA
#define M1_PHASEC_OUT_Pin LL_GPIO_PIN_10
#define M1_PHASEC_OUT_GPIO_Port GPIOA
#define M1_BRAKE2_Pin LL_GPIO_PIN_11
#define M1_BRAKE2_GPIO_Port GPIOA
#define IMU_SCL_Pin LL_GPIO_PIN_15
#define IMU_SCL_GPIO_Port GPIOA
#define IMU_INT_Pin LL_GPIO_PIN_6
#define IMU_INT_GPIO_Port GPIOB
#define IMU_INT_EXTI_IRQn EXTI9_5_IRQn
#define IMU_SDA_Pin LL_GPIO_PIN_7
#define IMU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
