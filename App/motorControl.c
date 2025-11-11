
#include "main.h"
#include "eBikeMain.h"
#include "motorControl.h"

#include "stm32f1xx_hal_tim.h"

#include "UART_Printf.h"

static uint16_t CurrentAssistLevel;
static uint16_t AssistLevel;

// Enable or not printf traces
void UART_printf(const char *fmt, ...);
#define printf UART_printf
//#define printf (void)


void MotorDriveInit(void){
    CurrentAssistLevel = 0;
    AssistLevel = 0;
}

void MotorDriveService(void){
    if ( AssistLevel > CurrentAssistLevel ){
        CurrentAssistLevel++;
    }
    if ( AssistLevel < CurrentAssistLevel ){
        CurrentAssistLevel--;
    }
    if ( CurrentAssistLevel == 0 ){
        DisablePWMs();
    }
    else{
        EnablePWMs();
    }
}


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

    int16_t MaxAmplitude = TIM1->ARR;
    int16_t Offset = MaxAmplitude/2;
    int16_t Amplitude = MaxAmplitude/10;


    TIM1->CCR1 = (uint16_t)(Offset+(Amplitude * table_sin[(counter + TIME_OFFSET_A) % NUM_POINTS])/SIN_TABLE_REF);
    TIM1->CCR2 = (uint16_t)(Offset+(Amplitude * table_sin[(counter + TIME_OFFSET_B) % NUM_POINTS])/SIN_TABLE_REF);
    TIM1->CCR3 = (uint16_t)(Offset+(Amplitude * table_sin[(counter + TIME_OFFSET_C) % NUM_POINTS])/SIN_TABLE_REF);

    counter++;
}

#endif // TEST_SINE


/*
 * Set level of assistance.
 * Scale is 0 to 100
 *****************************************/
uint16_t SetAssistLevel( uint16_t level ){
    // TODO fine tune scale
    if ( level <= 100 )
        AssistLevel = level;
    else
        AssistLevel = 0;

    printf("AssistLevel = %d %%\r\n", AssistLevel);
    return AssistLevel;
}


/*
 * Space vector modulation (Sinewave)
 * Space Vector PWM Intro — Switchcraft (pdf doc)
 * Microchip AN3453
 */

/* Sinusoidal Data Look-Up Table (16 values)
 */
#define TABLE_REF 400
uint16_t PWM_1[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint16_t PWM_2[16] = {5, 54, 107, 160, 208, 259, 310, 355, 401, 448, 487, 529, 568, 601, 635, 666};
uint16_t PWM_3[16] = {690, 715, 736, 753, 769, 780, 788, 794, 794, 793, 787, 779, 767, 751, 733, 713};
uint16_t PWM_4[16] = {690, 715, 734, 753, 769, 779, 788, 794, 795, 793, 787, 779, 767, 751, 734, 713};
uint16_t PWM_5[16] = {687, 661, 629, 594, 561, 521, 479, 439, 393, 345, 301, 250, 198, 151, 98, 44};
uint16_t PWM_6[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
void SetPhasesSvPwm(int16_t sector, int16_t pos){

    int16_t MaxAmplitude = TIM1->ARR;
    int16_t Amplitude = ((MaxAmplitude * CurrentAssistLevel)/100)/2;

    Amplitude /= 3;

    if ( pos > 15 ){
        pos = 15;
    }

    switch( sector ){
        case SECTOR1:
            TIM1->CCR1 = (Amplitude * PWM_3[pos])/TABLE_REF;
            TIM1->CCR2 = (Amplitude * PWM_5[pos])/TABLE_REF;
            TIM1->CCR3 = (Amplitude * PWM_1[pos])/TABLE_REF;
        break;
        case SECTOR2:
            TIM1->CCR1 = (Amplitude * PWM_2[pos])/TABLE_REF;
            TIM1->CCR2 = (Amplitude * PWM_4[pos])/TABLE_REF;
            TIM1->CCR3 = (Amplitude * PWM_6[pos])/TABLE_REF;
        break;
        case SECTOR3:
            TIM1->CCR1 = (Amplitude * PWM_1[pos])/TABLE_REF;
            TIM1->CCR2 = (Amplitude * PWM_3[pos])/TABLE_REF;
            TIM1->CCR3 = (Amplitude * PWM_5[pos])/TABLE_REF;
        break;
        case SECTOR4:
            TIM1->CCR1 = (Amplitude * PWM_6[pos])/TABLE_REF;
            TIM1->CCR2 = (Amplitude * PWM_2[pos])/TABLE_REF;
            TIM1->CCR3 = (Amplitude * PWM_4[pos])/TABLE_REF;
        break;
        case SECTOR5:
            TIM1->CCR1 = (Amplitude * PWM_5[pos])/TABLE_REF;
            TIM1->CCR2 = (Amplitude * PWM_1[pos])/TABLE_REF;
            TIM1->CCR3 = (Amplitude * PWM_3[pos])/TABLE_REF;
        break;
        case SECTOR6:
            TIM1->CCR1 = (Amplitude * PWM_4[pos])/TABLE_REF;
            TIM1->CCR2 = (Amplitude * PWM_6[pos])/TABLE_REF;
            TIM1->CCR3 = (Amplitude * PWM_2[pos])/TABLE_REF;
        break;
        default:
            Error_Handler();
            break;
    }
}

/*
 * Trapezoidal driving mode for motor phases
 * Two options
 *  All 3 generators are active (switching)
 * One over 3 generators is set to 0 (No switching), supposed to reduce loss
 *
 **********************************************************************/
void SetPhasesPwm(int16_t step){

    int16_t MaxAmplitude = TIM1->ARR;
    int16_t Offset = MaxAmplitude/2;
    int16_t Amplitude = ((MaxAmplitude * CurrentAssistLevel)/100)/2;

    //Amplitude /= 3;

#if 0  // TODO
    switch( step ){
        case STATE_PM0:
            TIM1->CCR1 = Offset+Amplitude;
            TIM1->CCR2 = Offset-Amplitude;
            TIM1->CCR3 = Offset;
        break;
        case STATE_P0M:
            TIM1->CCR1 = Offset+Amplitude;
            TIM1->CCR2 = Offset;
            TIM1->CCR3 = Offset-Amplitude;
        break;
        case STATE_0PM:
            TIM1->CCR1 = Offset;
            TIM1->CCR2 = Offset+Amplitude;
            TIM1->CCR3 = Offset-Amplitude;
        break;
        case STATE_MP0:
            TIM1->CCR1 = Offset-Amplitude;
            TIM1->CCR2 = Offset+Amplitude;
            TIM1->CCR3 = Offset;
        break;
        case STATE_M0P:
            TIM1->CCR1 = Offset-Amplitude;
            TIM1->CCR2 = Offset;
            TIM1->CCR3 = Offset+Amplitude;
        break;
        case STATE_0MP:
            TIM1->CCR1 = Offset;
            TIM1->CCR2 = Offset-Amplitude;
            TIM1->CCR3 = Offset+Amplitude;
        break;
        default:
            Error_Handler();
            break;
    }
#else
    switch( step ){
        case STATE_PM0:
            TIM1->CCR1 = 2 * Amplitude;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = Amplitude;
        break;
        case STATE_P0M:
            TIM1->CCR1 = 2 * Amplitude;
            TIM1->CCR2 = Amplitude;
            TIM1->CCR3 = 0;
        break;
        case STATE_0PM:
            TIM1->CCR1 = Amplitude;
            TIM1->CCR2 = 2 * Amplitude;
            TIM1->CCR3 = 0;
        break;
        case STATE_MP0:
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 2 * Amplitude;
            TIM1->CCR3 = Amplitude;
        break;
        case STATE_M0P:
            TIM1->CCR1 = 0;
            TIM1->CCR2 = Amplitude;
            TIM1->CCR3 = 2 * Amplitude;
        break;
        case STATE_0MP:
            TIM1->CCR1 = Amplitude;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 2 * Amplitude;
        break;
        default:
            Error_Handler();
            break;
    }
#endif

}



