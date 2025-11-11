
/*
 *
 *
 ***********************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "main.h"
#include "stm32f1xx_ll_gpio.h"

#include "UART_Printf.h"
#include "busVoltageSensor.h"
#include "eBikeMain.h"
#include "motorControl.h"

#include "display.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_tim1_up;
extern DMA_HandleTypeDef hdma_tim1_ch4_trig_com;

extern ADC_HandleTypeDef hadc1;
//extern ADC_HandleTypeDef hadc2;

uint32_t ADC_ResultBuffer[4];


// Motor
static uint16_t PreviousHallState;
static uint32_t HallError;
static uint16_t HallTicks;
static uint32_t HallCycles;
static uint16_t PreviousHallTicks;

//static uint16_t SpaceSectorCnt;

static uint32_t BusVoltage, BoardTemperature, MotorTemperature;
static uint32_t PhaseCurrent;


static uint16_t HallState;

static inline void disable_pwm(void) {
  CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
}

static inline void enable_pwm(void) {
  SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
}

static inline bool isSpaceVectorMode(void) {
    return ( HallTicks < 30000 && HallCycles > 100 );
    //return false;
}

void eBikeInit(void){
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Start Timer 1
    if(HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
    {
        /* Counter Enable Error */
        Error_Handler();
    }

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // turn on complementary channel
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    //TIM1->CR2 = _VAL2FLD(TIM_CR2_MMS, 0b111); // OC1REF signal is used as trigger output (TRGO)
    //TIM1->CR2 = _VAL2FLD(TIM_CR2_MMS, 0b010); // The update event is selected as trigger output (TRGO)

#if 1
    // Start Timer 2 (DAC output)
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    //HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    //HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);

    // Start Timer 3
    //TIM3->PSC = 250;
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIMEx_HallSensor_Start_IT(&htim3);
#endif

    //set initial PWM values
    TIM1->CCR1 = 1023;
    TIM1->CCR2 = 1023;
    TIM1->CCR3 = 1023;

    TIM1->CCR4 = 1;

    // App tick timer
    // See https://deepbluembedded.com/stm32-timer-calculator/
    // Updatefrequency = TIM clk/((PSC+1)*(ARR+1))
    //HAL_TIM_Base_Start_IT(&htim4);

    // ***TIM3 GPIO Configuration
    //PA6     ------> TIM3_CH1
    //PA7     ------> TIM3_CH2
    //PB0     ------> TIM3_CH3


    GPIO_InitStruct.Pin = Hall_H1_Pin|Hall_H2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Hall_H3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Hall_H3_GPIO_Port, &GPIO_InitStruct);

    // Lock pins
    LL_GPIO_LockPin(Hall_H3_GPIO_Port, Hall_H3_Pin);
    LL_GPIO_LockPin(Hall_H1_GPIO_Port, Hall_H1_Pin);
    LL_GPIO_LockPin(Hall_H2_GPIO_Port, Hall_H2_Pin);

    MotorDriveInit();

    displayInit();
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
    if ( htim->Instance == TIM3 ){
        //BlueLED_toggle();
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
    if ( htim->Instance == TIM3 ){
        //BlueLED_toggle();
    }
}


/**
  * Called from TIM3 IRQHandler
  * When no change is detected on Hall sensors lines
  */
void HAL_TIM3_Callback(void){
    if ((TIM3->SR & (TIM_FLAG_UPDATE)) == (TIM_FLAG_UPDATE)){
        HallTicks = 65535;
    }

}


// Validate hall state against previous state
//
uint16_t ValidateHallState( uint16_t state ){
    uint16_t validated = state;

    if ( HallTicks > MIN_STABLE_WHEEL_SPEED_TICKS ){
        return validated;
    }

    switch (PreviousHallState){
        case 1:
            validated = 3;
            break;

        case 3:
            validated = 2;
            break;

        case 2:
            validated = 6;
            break;

        case 6:
            validated = 4;
            break;

        case 4:
            validated = 5;
            break;

        case 5:
            validated = 1;
            break;
    }
    if ( validated != state ){
        HallError++;
    }
    //return state;
    return validated;
}

int16_t hallTable[] = {0xFF, 1, 5, 6, 3, 2, 4, 0xFF};

uint16_t ReadHallState( void ){
    uint32_t hall1, hall2, hall3;
    uint16_t state;

    hall3 = LL_GPIO_ReadInputPort( Hall_H2_GPIO_Port );
    hall3 >>= 5;
    hall2 = hall3;
    hall3 &= 0x04;
    hall2 &= 0x02;
    hall1  = LL_GPIO_ReadInputPort( Hall_H3_GPIO_Port ) & 0x01;
    state = (uint16_t)(hall3 | hall2 | hall1);

    return state;
}

/**
  * Manages HALL changes
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim){
    uint16_t HallState0;

    //BlueLED_on();

    //BlueLED_toggle();
    // TODO Improve Hardware noise filtering !
    HallState0 = ReadHallState();

    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");

    HallState = ReadHallState();

    HallCycles++;

    if  ( HallState0 != HallState ){
        HallError++;
    }


    // Store Tim3 ticks value since previous change
    HallTicks = TIM3->CCR1;

    if ( HallState == 1 ){
        BlueLED_toggle();
    }


    //HallState = ValidateHallState( HallState );
    int16_t sector = hallTable[HallState];
    SetPhasesPwm(sector);
#if 0
    if ( ! isSpaceVectorMode() ){
        switch (HallState){
            case 1:
                //Probe01_on();
                SetPhasesPwm(STATE_PM0);
                break;

            case 3:
                //Probe01_off();
                SetPhasesPwm(STATE_0MP);
                break;

            case 2:
                //Probe01_off();
                SetPhasesPwm(STATE_M0P);
                break;

            case 6:
                //Probe01_off();
                SetPhasesPwm(STATE_MP0);
                break;

            case 4:
                //Probe01_off();
                SetPhasesPwm(STATE_0PM);
                break;

            case 5:
                //Probe01_off();
                SetPhasesPwm(STATE_P0M);
                break;
            default:
                HallError++;
                break;
        }
    }
#endif

    PreviousHallState = HallState;
    PreviousHallTicks = HallTicks;

    //BlueLED_off();
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    if ( hadc->Instance == ADC1 ) {
        //BlueLED_toggle();
        BusVoltage       = ADC_ResultBuffer[0];
        MotorTemperature = ADC_ResultBuffer[1];
        BoardTemperature = ADC_ResultBuffer[2];  // Internal Sensor
    }
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    //BlueLED_on();
    PhaseCurrent = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);  // Read The Injected Channel Result
    //BlueLED_off();
}

const char * str_BusVoltage( uint32_t voltage){
    static char strVoltage[16];

    snprintf(strVoltage, 16, "%02lu.%lu", voltage/1000, voltage%1000 );
    return strVoltage;
}

void eBikeMainLoop(void){
    uint16_t wheelticks=0;

    uint32_t prev100Tick, prev10Tick, tick;

    prev10Tick  = 0;
    prev100Tick = 0;

    HAL_ADC_Start_DMA(&hadc1, ADC_ResultBuffer, 3);
    HAL_Delay(10);

    HAL_ADCEx_InjectedStart_IT(&hadc1); // Start ADC Conversion (Injected Channel, Tim1-PWM-Triggered, Interrupt Mode)
    // Inject ADC1 measurement to get peak value of the current.
    TIM1->CCR4 = 1500;

    // Start TIM4
    HAL_TIM_Base_Start_IT(&htim4);

    //UART_printf( "ARR = %u\r\n", TIM1->ARR );
    UART_printf( "CR2 = %04X\r\n", TIM1->CR2 );

     TIM4->ARR = 300;

    while(1){
        tick = HAL_GetTick();

        if ( tick > (prev10Tick + 100) ){
            prev10Tick = tick;
            HAL_NVIC_DisableIRQ(TIM3_IRQn);
            wheelticks = HallTicks;
            HAL_NVIC_EnableIRQ(TIM3_IRQn);

            ///__HAL_TIM_SET_AUTORELOAD( &htim4, (wheelticks/16) );

            displaySetWheelTime( WHEEL_SPEED_RPMx10(wheelticks) );

            // Display console In/Out
            displayService();

            // Manage Drive parameters
            MotorDriveService();
        }
        if ( tick > (prev100Tick + 1000) ){
            prev100Tick = tick;
            if ( wheelticks == 65535 ){
                HAL_TIM_IC_CaptureCallback(NULL);
                HallCycles = 0;
            }

            displayUpdate();

            UART_printf( "HallState = %u WheelSpeed = %u (%u)-%u\r\n", PreviousHallState, WHEEL_SPEED_KPH(wheelticks), wheelticks, HallCycles);
            // Bus voltage milliVolts
            BusVoltage       = (BusVoltage*((ADC_REFERENCE_VOLTAGE*10*1000)/4096))/VBUS_PARTITIONING_FACTORx10000;

            if ( PhaseCurrent > 2250 )
                PhaseCurrent     = PhaseCurrent - 2250;
            else
                PhaseCurrent     = 0;

            //UART_printf( "PhaseCurrent=%u\r\n", PhaseCurrent );
            //UART_printf( "BusVoltage=%s, PhaseCurrent=%u, MotorTemperature=%u, BoardTemperature=%u\r\n", str_BusVoltage(BusVoltage), PhaseCurrent, MotorTemperature, BoardTemperature);
        }
    }

}
