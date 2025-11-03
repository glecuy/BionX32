/****************************************************************************
  S12SN to LCD3 communication protocol.
  https://opensourceebikefirmware.bitbucket.io/development/EmbeddedFiles/7-S12SN_to_LCD3.txt

Packet consist of 12 bytes. 9600 baud, 8-n-1, byte-by-byte, no separators

B0  B1  B2  B3  B4  B5  B6  B7  B8  B9  B10 B11

(e.g: 65 16 48 0 139 0 164 2 13 0 0 0)

B0: 65 dec (0x41)
B1: battery level: 0: empty box, 1: border flashing, 2: animated charging, 3: empty, 4: 1 bar, 8: 2 bars, 16: 4 bars (full),
B2: 48 dec (0x30)
B3,B4: speed, wheel rotation period, ms; period(ms)=B3*256+B4;
B5 error info display: 0x20: "0info", 0x21: "6info", 0x22: "1info", 0x23: "2info", 0x24: "3info", 0x25: "0info", 0x26: "4info", 0x28: "0info"
B6: CRC: xor B0,B1,B2,B3,B4,B5,B7,B8,B9,B10,B11
    CHK: xor B1,B2,B3,B4,B5,B7,B8,B9,B10,B11  (B0 excluded = 0x41)

B7: moving mode indication, bit
b7b6b5b4 b3b2b1b0
. . . .  . . . m0      if set animated circle "throttle"
. . . .  m0 . . .      if set "C" (cruise) is shown
. . . m0  . . . .      if set "assist" shown
B8: power in 13 wt increments (48V version of the controller)
B9: motor temperature, can be negative or positive,T(C)=(int8)B8+15,
    if temperature > 120C LCD screen is flashing.
    e.g 0xDA T=-23C, 0x34 T=67C
B10: 0
B11: 0
************************************************************************


https://opensourceebikefirmware.bitbucket.io/development/EmbeddedFiles/7-LCD3_to_S12SN-1.txt

S-LCD3 to S12SN communication protocol.

Packet consist of 13 bytes. 9600 baud, 8-n-1, byte-by-byte

B0  B1  B2  B3  B4  B5  B6  B7  B8  B9  B10 B11 B12

(e.g: 12 0 149 160 41 102 18 74 4 20 0 50 14)

for the P and C parameters description please see S-LCD3 user manual available at the bmsbattery.com

B0: parameter P5.
B1: assist level, front light.
b7b6b5b4 b3b2b1b0
. . . .  . l2l1l0     assist level 0-5, 6-walk (long push down arrow)
f0. . .  . . . .      bit (mask 0x80) front light, display backlight
B3: parameter P1.
B2 and B4 max speed, wheel size, P2,P3,P4
B2: b7b6b5b4 b3b2b1b0 and B4:b7b6b5b4 b3b2b1b0
    s4s3s2s1 s0. . .         . . s5.  . . . .   max speed minus 10,
    . . . .  . . . .         . . . .  . . . .  km/h;   6bit
    . . . .  . w4w3w2        w1w0. .  . . . .  wheel size:0x0e-10",
    . . . .  . . . .         . . . .  . . . .  0x02-12", 0x06-14",
    . . . .  . . . .         . . . .  . . . .  0x00-16",0x04-18",
    . . . .  . . . .         . . . .  . . . .  0x08-20", 0x0c-22",
    . . . .  . . . .         . . . .  . . . .  0x10-24", 0x14"-26",
    . . . .  . . . .         . . . .  . . . .  0x18-700c
    . . . .  . . . .         . . . .  . p2p1p0  par. P2 (B4&&0x07)
    . . . .  . . . .         . . . .  p0. . .   par. P3 (B4&&0x08)
    . . . .  . . . .         . . . p0 . . . .   par. P4 (B4&&0x10)
Example:
    0 1 1 1  1 . . .         . . 0.   . . . .  25km/h (15+10)
    1 1 1 1  0 . . .         . . 0.   . . . .  40km/h (30+10)
    1 0 0 1  0 . . .         . . 1.   . . . .  60km/h (50+10)
B5: CRC = (xor B0,B1,B2,B3,B4,B6,B7,B8,B9,B10,B11,B12) xor 2.
B6: parameters C1 and C2
b7b6b5b4 b3b2b1b0
. . c2c1 c0. . .       param C1 (mask 0x38)
. . . .  . c2c1c0      param C2 (mask 0x07)
B7: parameter C5 and C7
b7b6b5b4 b3b2b1b0
. . . .  c3c2c1c0      param C5 (mask 0x0F)
. c1c0.  . . . .       param C14 (mask 0x60)
B8: parameter C4
b7b6b5b4 b3b2b1b0
c2c1c0.  . . . .       param C4  (mask 0xE0)
B9: parameter C12
b7b6b5b4 b3b2b1b0
. . . .  c3c2c1c0      param C12  (mask 0x0F)
B10: parameter C13
b7b6b5b4 b3b2b1b0
. . . c2 c1c0. .       param C13  (mask 0x1C)
B11: 50 dec (0x32)
B12: 14 dec (0x0E)
parameters C3, C6, C7, C8, C9, C10 not sent to MCU

if C11 set to 2 (used to copy LCD to LCD), LCD repeatedly sends 23 bytes, byte by byte, no separators, underlines show not identified values


255, wheel diam (in), maxspeed (kmh), level, P1, P2, P3, P4, P5, C1, C2 , C3, C4, C5, C6, C7, C8, 0, 20, C12, C13 ,C14, 55

Example:
255 26 60 0 160 1 1 0 12 2 1 8 0 10 3 0 1 0 20 4 0 2 55


****************************************************************************/


// Includes
#include <stdio.h>

#include "motorControl.h"
#include "main.h"
#include "display.h"

// Enable or not printf traces
void UART_printf(const char *fmt, ...);
#define printf UART_printf
//#define printf (void)

// Uart3 is used to communicate with S866 display
extern UART_HandleTypeDef huart3;

typedef struct
{
    // Parameters received from display in operation mode:
    uint8_t  AssistLevel;               // 0..255 Power Assist Level
    uint8_t  Headlight;                 // KM_HEADLIGHT_OFF / KM_HEADLIGHT_ON / KM_HEADLIGHT_LOW / KM_HEADLIGHT_HIGH
    uint8_t  Battery;                   // KM_BATTERY_NORMAL / KM_BATTERY_LOW
    //uint8_t  PushAssist;                // KM_PUSHASSIST_OFF / KM_PUSHASSIST_ON
    //uint8_t  PowerAssist;               // KM_POWERASSIST_OFF / KM_POWERASSIST_ON
    //uint8_t  Throttle;                  // KM_THROTTLE_OFF / KM_THROTTLE_ON
    //uint8_t  CruiseControl;             // KM_CRUISE_OFF / KM_CRUISE_ON
    //uint8_t  OverSpeed;                 // KM_OVERSPEED_OFF / KM_OVERSPEED_ON
    //uint16_t SPEEDMAX_Limit_x10;        // Unit: 0.1km/h
    //uint16_t CUR_Limit_x10;             // Unit: 0.1A

}RX_PARAM_t;


typedef struct
{
    // Parameters to be send to display in operation mode:
    uint8_t  Battery;                   // KM_BATTERY_NORMAL / KM_BATTERY_LOW
    uint16_t Wheeltime;                 // Unit:1ms ??
    uint8_t  Error;                     // KM_ERROR_NONE, ..
    uint16_t CurrentX10;               // Unit: 0.1A

}TX_PARAM_t;

#define DISPLAY_RX_FRAME_SIZE 13
#define DISPLAY_TX_FRAME_SIZE 12



#define DISPLAY_MS_TO_T(ms) ((ms*636)/1000)

typedef struct
{
    uint8_t         TxBuff[DISPLAY_TX_FRAME_SIZE];
    uint8_t         RxBuff[DISPLAY_RX_FRAME_SIZE];

    RX_PARAM_t      Settings;
    RX_PARAM_t      Rx;
    TX_PARAM_t      Tx;

    uint32_t RxIndex;
    uint8_t EndOfTx;

}DISPLAY_CONTEXT_t;

DISPLAY_CONTEXT_t gContext;

void displayInit(void) {
    //gContext.Tx.Wheeltime = 201; // mSec (25 KmH)
    gContext.Tx.Wheeltime = 2010; // mSec
    gContext.Tx.Error = 0x00;

    displayInitRxTx();
}

/*

****************************************************************************/
static void parseRxFrame(void){
    uint8_t level = gContext.RxBuff[1] & 0xF;
#if 0
    uint8_t * ptr = gContext.RxBuff;
    printf("%02X %02X\r\n", gContext.RxBuff[0], gContext.RxBuff[1] );
    for (int i=0 ; i<DISPLAY_RX_FRAME_SIZE ; i++ ){
        printf("%02X ", *ptr++ );
    }
    printf("\r\n");
#endif

    if ( gContext.Rx.AssistLevel != level ){
        gContext.Rx.AssistLevel = level;
        SetAssistLevel( (level*100)/5 );
        //SetAssistLevel(10);
    }


    //DEBUG_pin_toggle();


   // for (int i=0 ; i<DISPLAY_RX_FRAME_SIZE ; i++ ){
   //     printf("%02X ", *ptr++ );
   // }
   // printf("parseRxFrame()\n");
}

/*
    {65, 4, 48, 2, 10, 0, 164, 2, 13, 0, 0, 0};
 **************************************************************/
static void sendStatus( void ){
    uint16_t val = gContext.Tx.Wheeltime;

    gContext.TxBuff[4] = val & 0xFF;
    gContext.TxBuff[3] = (val>>8) & 0xFF;

    // CheckSum
    gContext.TxBuff[6] =  gContext.TxBuff[1] ^ gContext.TxBuff[2] ^ gContext.TxBuff[3] ^ gContext.TxBuff[4]
                        ^ gContext.TxBuff[5] ^ gContext.TxBuff[7] ^ gContext.TxBuff[8] ^ gContext.TxBuff[9]
                        ^ gContext.TxBuff[10] ^ gContext.TxBuff[11];

    gContext.TxBuff[0]  = 65;
    gContext.TxBuff[1]  = 4;
    gContext.TxBuff[2]  = 48;

    gContext.TxBuff[5]  = 0;

    gContext.TxBuff[7]  = 2;
    gContext.TxBuff[8]  = 13;
    gContext.TxBuff[9]  = 0;
    gContext.TxBuff[10] = 0;
    gContext.TxBuff[11] = 0;

    HAL_UART_Transmit_IT( &huart3, gContext.TxBuff, DISPLAY_TX_FRAME_SIZE);
    gContext.EndOfTx = 0;

}

void displayInitRxTx(void){

    gContext.EndOfTx = 0;
    gContext.RxIndex = 0;
}

void displayEndOfRx(){
//    UserProbe6_toggle();

    if ( gContext.RxIndex > 0)
        Probe01_on();
    else
        Probe01_off();

    if ( gContext.RxBuff[0] != 0x0C ){
        gContext.RxIndex = 0;
    }
    else if ( gContext.RxIndex < DISPLAY_RX_FRAME_SIZE ){
        // Continue receiving
        gContext.RxIndex++;
        HAL_UART_Receive_IT( &huart3, gContext.RxBuff+gContext.RxIndex, 1 );
        return;
    }

    HAL_UART_Receive_IT( &huart3, gContext.RxBuff+0, 1 );
}

void displayEndOfTx(void){
    gContext.EndOfTx = 1;
}

void displayService(void){
    //printf("gContext.RxIndex = %u\r\n", gContext.RxIndex );

    if ( gContext.RxIndex >= DISPLAY_RX_FRAME_SIZE ){
        // Parse data frame
        parseRxFrame();
        gContext.RxIndex = 0;
    }

    if ( gContext.RxIndex == 0 ){
        HAL_UART_Receive_IT( &huart3, gContext.RxBuff, 1 );
    }
}


void displayUpdate(void){
    // Send some data from controller to display
    sendStatus();
}


uint8_t displayGetAssistLevel(void){
    return gContext.Rx.AssistLevel;
}


uint8_t displaySetWheelTime( int16_t rpm10 ){
    if ( rpm10 < 90 )    // 1.x Kmh
        rpm10 = 90;
    if ( rpm10 > 4557 )  // 60 Kmh
        rpm10 = 4557;

    gContext.Tx.Wheeltime = (uint16_t)(637992L / (int32_t)rpm10);
    //printf( "wheeltime = %d\r\n", gContext.Tx.Wheeltime );
    return 0;
}

uint8_t displaySetError( DISPLAY_ERR_t err ){
    gContext.Tx.Error = 0;
    if ( err == DISPLAY_MOTOR_ERROR ){
        gContext.Tx.Error = 1;  // TODO
    }
    return gContext.Tx.Error;
}
