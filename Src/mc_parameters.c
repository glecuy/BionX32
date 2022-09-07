
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

#include "r1_hd2_pwm_curr_fdbk.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/**
  * @brief  Current sensor parameters Dual Drive Motor 1 - one shunt
  */

const R1_DDParams_t R1_DDParamsM1 =
{

/* Instance number -----------------------------------------------------------*/
  .InstanceNbr             =	1,

  .IsFirstR1DDInstance      =	true,

  .FreqRatio               =	FREQ_RATIO,

  .IsHigherFreqTim         =	FREQ_RELATION,

  .TIMx                     =	TIM1,

  .TIMx_2                     = TIM5,

  /* Current reading A/D Conversions initialization --------------------------*/
  .ADCx_Inj = ADC3,          /*!< ADC Pperipheral used for phase current sampling */
  .ADCx_Reg = ADC1,          /*!< ADC Pperipheral used for regular conversion */
  .IChannel                  =	MC_ADC_CHANNEL_0,

/* PWM generation parameters --------------------------------------------------*/
  .DeadTime                  =	DEAD_TIME_COUNTS,

  .RepetitionCounter         =	REP_COUNTER,

  .Tafter                    =	TAFTER,

  .Tbefore                   =	TBEFORE,
  .TMin                      =	 TMIN,

  .HTMin                     =	HTMIN,

  .TSample                   =	SAMPLING_TIME,

  .MaxTrTs                   =	MAX_TRTS,

/* PWM Driving signals initialization ----------------------------------------*/

  .LowSideOutputs         =	(LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,

/* PWM Driving signals initialization ----------------------------------------*/
  .EmergencyStop                = (FunctionalState) ENABLE,

};

/* USER CODE BEGIN Additional parameters */

/* USER CODE END Additional parameters */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
