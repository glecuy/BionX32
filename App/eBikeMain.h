#ifndef __E_BIKE_MAIN_H
#define __E_BIKE_MAIN_H

//#include <arm_math.h>

void eBikeInit(void);
void eBikeMainLoop(void);

#define EnablePWMs()   LL_TIM_EnableAllOutputs(TIM1)
#define DisablePWMs()  LL_TIM_DisableAllOutputs(TIM1)


#define WHEEL_CIRCUMFERENCE 2200UL  // mm

#define ADC_REFERENCE_VOLTAGE  3300     // 3.3V

#define PQD_CONVERSION_FACTOR (int32_t)(( 1000 * 3 * ADC_REFERENCE_VOLTAGE ) /\
             ( 1.732 * RSHUNT * AMPLIFICATION_GAIN ))

#define VBUS_PARTITIONING_FACTORx10000      901   // 0.0901

#define dV_dTx10000                         250    // 0.025 /*!< V/Celsius degrees */

// Electrical revolution freq:
// clk = 72E+6
// efreq = 72E+6/(PSC+1*wheelticks*NbSectors)
// Mecanical revolution freq:
// mfreq = efreq / NbPoles

// wheelticks=6000
// NbSectors=6
// PSC=300
// efreq = 72E6/(301*6000*6)    = 6.644518272425249/11
// mfreq = 6.644518272425249/11 = 0.60404 Hz

// V(K/h) = mfreq * WHEEL_CIRCUMFERENCE * 3600
// V(K/h)= 0.60404 * (2200/1000000) * 3600

// mfreq  = 72000000/(301*wheelticks*6*11)
// V(K/h) = (72000000/(301*wheelticks*6*11)) * (2200/1000000) * 3600
// V(K/h) = (72000000/(301*6*11*wheelticks)) * ((2200*3600)/1000000)
// V(K/h) = ((72000000*2200*3600)/(301*6*11*1000000*wheelticks))
// V(K/h) = ((72*2200*3600)/(301*6*11*1*wheelticks))

// V(rpm) = mfreq*60
// V(rpm) = 72000000*60/(301*wheelticks*6*11)
// V(rpm) = 72000000*60/(301*6*11*wheelticks)

#define WHEEL_SPEED_KPH(ticks) ((72UL*WHEEL_CIRCUMFERENCE*3600UL)/(301*6*11*1*ticks))

#define WHEEL_SPEED_RPMx10(ticks) ((72000000UL*10UL)/(((301*6*11)/60)*ticks))

#define MIN_STABLE_WHEEL_SPEED_TICKS  (20000)

#define MAX_TICKS_DIFF                (800)

#endif // __E_BIKE_MAIN_H
