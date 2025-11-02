/*


*/


#ifndef DISPLAY_H
#define DISPLAY_H

typedef enum{
    DISPLAY_NO_ERROR,
    DISPLAY_LOW_BATTERY,
    DISPLAY_MOTOR_ERROR,
    DISPLAY_OVER_TEMP
}DISPLAY_ERR_t;

// Public function prototypes
void displayInit(void);

void displayService(void);
void displayUpdate(void);

uint8_t displaySetWheelTime( int16_t rpm10 );

uint8_t displaySetError( DISPLAY_ERR_t err );


void displayEndOfTx(void);
void displayEndOfRx(void);
void displayInitRxTx(void);

uint8_t displayGetAssistLevel(void);


#endif // DISPLAY_H

