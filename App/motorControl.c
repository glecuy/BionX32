
#include "main.h"
#include "motorControl.h"

#include "stm32f1xx_hal_tim.h"

#include "UART_Printf.h"

static uint16_t AssistLevel;

// Enable or not printf traces
void UART_printf(const char *fmt, ...);
#define printf UART_printf
//#define printf (void)


#ifdef TEST_SINE
#define NUM_POINTS 66
#define SIN_TABLE_REF 512
// Phases Relative position
#define TIME_OFFSET_A  (0)
#define TIME_OFFSET_B  (NUM_POINTS / 3)
#define TIME_OFFSET_C  (2 * NUM_POINTS / 3)

int16_t table_sin[NUM_POINTS] = {
0,48,96,144,190,234,276,316,352,386,416,442,465,483,497,506,510,510,506,497,483,465,
442,416,386,352,316,276,234,190,144,96,48,0,-48,-96,-144,-190,-234,-276,-316,-352,-386,-416,
-442,-465,-483,-497,-506,-510,-510,-506,-497,-483,-465,-442,-416,-386,-352,-316,-276,-234,-190,-144,-96,-48
};


void UpdateTestSinePwm(void){
    static uint16_t counter = 0;

    int16_t MaxAmplitute = TIM1->ARR;
    int16_t Offset = MaxAmplitute/2;
    int16_t Amplitute = MaxAmplitute/10;


    TIM1->CCR1 = (uint16_t)(Offset+(Amplitute * table_sin[(counter + TIME_OFFSET_A) % NUM_POINTS])/SIN_TABLE_REF);
    TIM1->CCR2 = (uint16_t)(Offset+(Amplitute * table_sin[(counter + TIME_OFFSET_B) % NUM_POINTS])/SIN_TABLE_REF);
    TIM1->CCR3 = (uint16_t)(Offset+(Amplitute * table_sin[(counter + TIME_OFFSET_C) % NUM_POINTS])/SIN_TABLE_REF);


    counter++;
}

#endif // TEST_SINE


/*
 * Set level of assistance.
 * Scale is 0 to 100
 *****************************************/
uint16_t SetAssistLevel( uint16_t level ){
    if ( level <= 100 )
        AssistLevel = level;
    else
        AssistLevel = 0;
    printf("AssistLevel = %d %%\r\n", AssistLevel);
    return AssistLevel;
}


/*
 * www.Pittman-Motors.com
 */
void SetPhasesPwm(int16_t step){

    int16_t MaxAmplitute = TIM1->ARR;
    int16_t Offset = MaxAmplitute/2;
    int16_t Amplitute = ((MaxAmplitute * AssistLevel)/100)/2;

#if 1  // TODO
    switch( step ){
        case STATE_PM0:
            TIM1->CCR1 = Offset+Amplitute;
            TIM1->CCR2 = Offset-Amplitute;
            TIM1->CCR3 = Offset;
        break;
        case STATE_P0M:
            TIM1->CCR1 = Offset+Amplitute;
            TIM1->CCR2 = Offset;
            TIM1->CCR3 = Offset-Amplitute;
        break;
        case STATE_0PM:
            TIM1->CCR1 = Offset;
            TIM1->CCR2 = Offset+Amplitute;
            TIM1->CCR3 = Offset-Amplitute;
        break;
        case STATE_MP0:
            TIM1->CCR1 = Offset-Amplitute;
            TIM1->CCR2 = Offset+Amplitute;
            TIM1->CCR3 = Offset;
        break;
        case STATE_M0P:
            TIM1->CCR1 = Offset-Amplitute;
            TIM1->CCR2 = Offset;
            TIM1->CCR3 = Offset+Amplitute;
        break;
        case STATE_0MP:
            TIM1->CCR1 = Offset;
            TIM1->CCR2 = Offset-Amplitute;
            TIM1->CCR3 = Offset+Amplitute;
        break;
        default:
            Error_Handler();
            break;
    }
#else
    switch( step ){
        case STATE_PM0:
            TIM1->CCR1 = 2 * Amplitute;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = Amplitute;
        break;
        case STATE_P0M:
            TIM1->CCR1 = 2 * Amplitute;
            TIM1->CCR2 = Amplitute;
            TIM1->CCR3 = 0;
        break;
        case STATE_0PM:
            TIM1->CCR1 = Amplitute;
            TIM1->CCR2 = 2 * Amplitute;
            TIM1->CCR3 = 0;
        break;
        case STATE_MP0:
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 2 * Amplitute;
            TIM1->CCR3 = Amplitute;
        break;
        case STATE_M0P:
            TIM1->CCR1 = 0;
            TIM1->CCR2 = Amplitute;
            TIM1->CCR3 = 2 * Amplitute;
        break;
        case STATE_0MP:
            TIM1->CCR1 = Amplitute;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 2 * Amplitute;
        break;
        default:
            Error_Handler();
            break;
    }
#endif

}



