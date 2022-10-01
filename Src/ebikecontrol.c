/**
  ******************************************************************************
  * @file    ebikecontrol.c
  * @brief   Motor interface using MC API functions (ST Motor Control FOC library API)
  *
  * https://wiki.st.com/stm32mcu/wiki/STM32StepByStep:Getting_started_with_Motor_Control
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_api.h"

#include "main.h"
#include "display.h"
#include "ebikecontrol.h"


void UART_printf(const char *fmt, ...);
#define printf UART_printf
//#define printf (void)

extern MCI_Handle_t * pMCI[NBR_OF_MOTORS];
extern MCT_Handle_t * pMCT[NBR_OF_MOTORS];

uint8_t MC_AssistLevel = 0xFF;

void motorServiceInit(void){
    // Set Speed to 50 rpm in 2 Sec
    // MC_ProgramSpeedRampMotor1(50/6, 2000);

    // MC_ProgramTorqueRampMotor1

    //MC_SetCurrentReferenceMotor1(

    printf("Motor control mode is %d\r\n", MC_GetControlModeMotor1() );

    //displayMotorState();
}


void motorService(void){
    uint8_t AssistLevel = displayGetAssistLevel();
    uint16_t voltage, temp;


    if ( MC_AssistLevel != AssistLevel ){
        if ( AssistLevel == 0 ){
            MC_StopMotor1();
        }
        else{

            MC_StartMotor1();
        }
        MC_AssistLevel = AssistLevel;
    }

    voltage = VBS_GetAvBusVoltage_V(pMCT[M1]->pBusVoltageSensor);
    temp = NTC_GetAvTemp_C(pMCT[M1]->pTemperatureSensor);
    printf("Motor voltage = %d - T=%d\r\n", voltage, temp );


    //displayMotorState();
}


void displayMotorState(void){
    State_t StateM = MC_GetSTMStateMotor1();
    printf("Motor state:");
    switch ( StateM ){
    case IDLE:
        printf("IDLE");
        break;

    case IDLE_START:
        printf("IDLE_START");
        break;

    case CHARGE_BOOT_CAP:
        printf("CHARGE_BOOT_CAP");
        break;

    case OFFSET_CALIB:
        printf("OFFSET_CALIB");
        break;

    case CLEAR:
        printf("IDLE_START");
        break;

    case START:
        printf("START");
        break;

    case START_RUN:
        printf("START_RUN");
        break;

    case RUN:
        printf("RUN");
        break;

    case ANY_STOP:
        printf("ANY_STOP");
        break;

    case STOP:
        printf("STOP");
        break;

    case STOP_IDLE:
        printf("STOP_IDLE");
        break;

    /*!< Persistent state, the state machine can be moved from
       any condition directly to this state by
       STM_FaultProcessing method. This method also manage
       the passage to the only allowed following state that
       is FAULT_OVER */
    case FAULT_NOW:
        printf("FAULT_NOW");
        break;

    /*!< Persistent state where the application is intended to
      stay when the fault conditions disappeared. Following
      state is normally STOP_IDLE, state machine is moved as
      soon as the user has acknowledged the fault condition.
    */
    case FAULT_OVER:
        printf("FAULT_OVER");
        break;

    default:
        printf("?? %d", StateM);
        break;
  }
  printf("\r\n");

}

