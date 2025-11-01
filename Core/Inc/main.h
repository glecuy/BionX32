/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_ll_tim.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BlueLED_Pin GPIO_PIN_13
#define BlueLED_GPIO_Port GPIOC
#define Probe01_Pin GPIO_PIN_5
#define Probe01_GPIO_Port GPIOA
#define Hall_H1_Pin GPIO_PIN_6
#define Hall_H1_GPIO_Port GPIOA
#define Hall_H2_Pin GPIO_PIN_7
#define Hall_H2_GPIO_Port GPIOA
#define Hall_H3_Pin GPIO_PIN_0
#define Hall_H3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

// Blue LED Module
#define BlueLED_toggle() HAL_GPIO_TogglePin (BlueLED_GPIO_Port, BlueLED_Pin)
#define BlueLED_on() HAL_GPIO_WritePin  (BlueLED_GPIO_Port, BlueLED_Pin, GPIO_PIN_SET)
#define BlueLED_off() HAL_GPIO_WritePin (BlueLED_GPIO_Port, BlueLED_Pin, GPIO_PIN_RESET)

// Probe01 (general purpose debug pin)
#define Probe01_toggle() HAL_GPIO_TogglePin (Probe01_GPIO_Port, Probe01_Pin)
#define Probe01_on() HAL_GPIO_WritePin  (Probe01_GPIO_Port, Probe01_Pin, GPIO_PIN_SET)
#define Probe01_off() HAL_GPIO_WritePin (Probe01_GPIO_Port, Probe01_Pin, GPIO_PIN_RESET)

extern void HAL_TIM3_Callback(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
