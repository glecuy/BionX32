#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#define  STATE_PM0  1
#define  STATE_P0M  2
#define  STATE_0PM  3
#define  STATE_MP0  4
#define  STATE_M0P  5
#define  STATE_0MP  6


#define MOTOR_HALL_IC_FILTER  15

void MotorDriveInit(void);

void MotorDriveService(void);

void UpdateTestSinePwm(void);
void UpdateTestSquarePwm(void);
void UpdateTestTrapezoidalPwm(void);

void UpdateTest6StepsPwm(void);

void SetPhasesPwm(int16_t step);

uint16_t SetAssistLevel( uint16_t level );

#endif // __MOTOR_CONTROL_H
