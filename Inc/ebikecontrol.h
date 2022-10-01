/**
  ******************************************************************************
  * @file    user_interface.h
  * @brief   Motor interface using MC API functions (ST Motor Control FOC library API)
  *
  ******************************************************************************
  * @ingroup MCUI
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EBIKECONTROL_H
#define __EBIKECONTROL_H

#
void motorServiceInit(void);

void motorService(void);

void displayMotorState(void);


#endif /*__EBIKECONTROL_H*/

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
