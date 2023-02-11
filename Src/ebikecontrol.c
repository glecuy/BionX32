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
    /* Set Speed to 40 (~5Kh) rpm in 2 Sec  */
    /* Apply a ramp to reach 40 RPM in 2000 ms - 40 RPM = 40/6 0.1 Hz */
    MC_ProgramSpeedRampMotor1(40/6, 2000);

    // MC_ProgramTorqueRampMotor1

    //MC_SetCurrentReferenceMotor1(

    printf("Motor control mode is %d\r\n", MC_GetControlModeMotor1() );

    MC_StopMotor1();
}

volatile uint8_t MC_Pas_IT_Counter;
volatile uint32_t MC_Pas_Tick;
//uint8_t MC_Pas_Value;


// Called every 0.1 Sec
uint8_t MC_GetPasStatus(void){
    // test Time out
    if ( systick_cnt > (MC_Pas_Tick + PAS_MAX_TIME_OUT*2) ) {
        MC_Pas_IT_Counter = 0;
    }
    if ( MC_Pas_IT_Counter > PAS_MIN_ON_VALUE ){
        return 1;
    }
    else{
        return 0;
    }
}

uint8_t MC_GetBrakeStatus(void){
    if ( HAL_GPIO_ReadPin(PAS_Brake_Port, Brake_Pin) != 0 ){
        return 1;
    }
    else{
        return 0;
    }
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

    printf("PAS / Break state : %d %d %d\r\n", MC_Pas_IT_Counter, MC_GetPasStatus(), MC_GetBrakeStatus() );

    State_t StateM = MC_GetSTMStateMotor1();
    uint16_t fault_current, fault_occurred;
    bool motor_rc;
    int16_t motorRPM; ///** Tenth of Hertz: 1 Hz is 10 01Hz */
    //motorRPM = 1519;  // 151 rpm , 20 Kmh
    motorRPM = 1899;  // 25 Kmh
    motorRPM = 4557;  // 60 Kmh

    switch ( StateM )
    {
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
            motorRPM = MC_GetMecSpeedAverageMotor1();
            printf("RUN");
            break;

        /*!< Persistent state, the state machine can be moved from
           any condition directly to this state by
           STM_FaultProcessing method. This method also manage
           the passage to the only allowed following state that
           is FAULT_OVER */
        case FAULT_NOW:
            fault_current = MC_GetCurrentFaultsMotor1();
            motor_rc      = MC_AcknowledgeFaultMotor1();
            printf("FAULT_OVER (%04X %d)", fault_current, motor_rc );
            break;

        /*!< Persistent state where the application is intended to
          stay when the fault conditions disappeared. Following
          state is normally STOP_IDLE, state machine is moved as
          soon as the user has acknowledged the fault condition.
        */
        case FAULT_OVER:
            printf("FAULT_OVER");
            fault_occurred = MC_GetOccurredFaultsMotor1();
            motor_rc       = MC_AcknowledgeFaultMotor1();
            printf("FAULT_OVER (%04X %d)", fault_occurred, motor_rc );
            break;


        default:
            printf("?? %d", StateM);
            break;
    }
    printf("\r\n");

    displaySetWheelTime( motorRPM );
}



