/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "motorcontrol.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M1_CURR_AMPL_Pin GPIO_PIN_0
#define M1_CURR_AMPL_GPIO_Port GPIOA
#define M1_BUS_VOLTAGE_Pin GPIO_PIN_1
#define M1_BUS_VOLTAGE_GPIO_Port GPIOA
#define M1_HALL_H1_Pin GPIO_PIN_6
#define M1_HALL_H1_GPIO_Port GPIOA
#define M1_HALL_H2_Pin GPIO_PIN_7
#define M1_HALL_H2_GPIO_Port GPIOA
#define M1_HALL_H3_Pin GPIO_PIN_0
#define M1_HALL_H3_GPIO_Port GPIOB
#define UART2_TX_Pin GPIO_PIN_2
#define UART2_TX_GPIO_Port GPIOA
#define UART2_RX_Pin GPIO_PIN_3
#define UART2_RX_GPIO_Port GPIOA
#define UART3_TX_Pin GPIO_PIN_10
#define UART3_TX_GPIO_Port GPIOB
#define UART3_RX_Pin GPIO_PIN_11
#define UART3_RX_GPIO_Port GPIOB
#define M1_OCP_Pin GPIO_PIN_12
#define M1_OCP_GPIO_Port GPIOB
#define M1_PWM_UL_Pin GPIO_PIN_13
#define M1_PWM_UL_GPIO_Port GPIOB
#define M1_PWM_VL_Pin GPIO_PIN_14
#define M1_PWM_VL_GPIO_Port GPIOB
#define M1_PWM_WL_Pin GPIO_PIN_15
#define M1_PWM_WL_GPIO_Port GPIOB
#define M1_PWM_UH_Pin GPIO_PIN_8
#define M1_PWM_UH_GPIO_Port GPIOA
#define M1_PWM_VH_Pin GPIO_PIN_9
#define M1_PWM_VH_GPIO_Port GPIOA
#define M1_PWM_WH_Pin GPIO_PIN_10
#define M1_PWM_WH_GPIO_Port GPIOA
#define DAC_EMUL_CH1_Pin GPIO_PIN_15
#define DAC_EMUL_CH1_GPIO_Port GPIOA
#define DAC_EMUL_CH2_Pin GPIO_PIN_3
#define DAC_EMUL_CH2_GPIO_Port GPIOB

// Non motor gpio definitions
#define UserLED_Pin GPIO_PIN_13
#define UserLED_GPIO_Port GPIOC
#define BusCurrent_Pin GPIO_PIN_0
#define BusCurrent_GPIO_Port GPIOA
#define BusVoltage_Pin GPIO_PIN_1
#define BusVoltage_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define UserLED_toggle() HAL_GPIO_TogglePin (UserLED_GPIO_Port, UserLED_Pin)
#define UserLED_on() HAL_GPIO_WritePin (UserLED_GPIO_Port, UserLED_Pin, GPIO_PIN_SET)
#define UserLED_off() HAL_GPIO_WritePin (UserLED_GPIO_Port, UserLED_Pin, GPIO_PIN_RESET)

// Probes
#define UserProbe5_Pin  GPIO_PIN_5
#define UserProbe6_Pin  GPIO_PIN_6
#define UserProbes_Port GPIOB

#define UserProbe5_toggle() HAL_GPIO_TogglePin (UserProbes_Port, UserProbe5_Pin)
#define UserProbe6_toggle() HAL_GPIO_TogglePin (UserProbes_Port, UserProbe6_Pin)
#define UserProbe5_H() HAL_GPIO_WritePin (UserProbes_Port, UserProbe5_Pin, GPIO_PIN_SET)
#define UserProbe5_L() HAL_GPIO_WritePin (UserProbes_Port, UserProbe5_Pin, GPIO_PIN_RESET)
#define UserProbe6_H() HAL_GPIO_WritePin (UserProbes_Port, UserProbe6_Pin, GPIO_PIN_SET)
#define UserProbe6_L() HAL_GPIO_WritePin (UserProbes_Port, UserProbe6_Pin, GPIO_PIN_RESET)


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
