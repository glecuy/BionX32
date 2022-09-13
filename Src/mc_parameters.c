
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "parameters_conversion.h"

#include "r1_vl1_pwm_curr_fdbk.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/**
  * @brief  Current sensor parameters Single Drive - one shunt, STM32F100
  */
const R1_VL1Params_t R1_VL1ParamsSD =
{

/* Instance number -----------------------------------------------------------*/

  .TIMx                     = TIM1,

  .TIMx_2                   = TIM4,

  /* Current reading A/D Conversions initialization --------------------------*/
  .ADCx_Inj = ADC1,

  .hIChannel                  =	MC_ADC_CHANNEL_0,

/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime                  =	DEAD_TIME_COUNTS,

  .RepetitionCounter         =	REP_COUNTER,

  .hTafter                    =	TAFTER,

  .hTbefore                   =	TBEFORE,
  .hTMin                      =	 TMIN,

  .hHTMin                     =	HTMIN,

  .hTSample                   =	SAMPLING_TIME,

  .hMaxTrTs                   =	MAX_TRTS,

/* PWM Driving signals initialization ----------------------------------------*/

  .LowSideOutputs         =	(LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,

/* PWM Driving signals initialization ----------------------------------------*/
  .EmergencyStop                = (FunctionalState) ENABLE,
};

/* USER CODE BEGIN Additional parameters */

/* USER CODE END Additional parameters */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
