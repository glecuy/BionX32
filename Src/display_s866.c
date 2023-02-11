/*
  * S866 ebike display interface (UART@9600)
  *
  * Display to Controller: 20 bytes long binary frame
  * Controller to display : 14 Bytes long
  *
  * There is no official or unofficial documentation on the protocol.
  * The code below is mostly based on reverse engineering.
  * it is therefore very incomplete.
  * Speed is send, assist level is received.
  *
 */


// Includes
#include <stdio.h>

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

#define DISPLAY_RX_FRAME_SIZE 20
#define DISPLAY_TX_FRAME_SIZE 14


// TODO
#define DISPLAY_ERR_6   0x08
#define DISPLAY_ERR_7   0x40
#define DISPLAY_ERR_8   0x20
#define DISPLAY_ERR_10  0x10

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
01 14 01 01 00 C0 50 01 13 12 04 00 1E 10 00 CA 00 00 09 4C ,
01 14 01 01 03 C0 50 01 13 12 04 00 1E 10 00 CA 00 00 09 4F ,
01 14 01 01 06 C0 50 01 13 12 04 00 1E 10 00 CA 00 00 09 4A ,
01 14 01 01 09 C0 50 01 13 12 04 00 1E 10 00 CA 00 00 09 45 ,
01 14 01 01 0C C0 50 01 13 12 04 00 1E 10 00 CA 00 00 09 40 ,
01 14 01 01 0F C0 50 01 13 12 04 00 1E 10 00 CA 00 00 09 43 ,
****************************************************************************/
static void parseRxFrame(void){
    uint8_t level = gContext.RxBuff[4] & 0xF;
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
        printf("AssistLevel = %d\r\n", gContext.Rx.AssistLevel);
    }


    //DEBUG_pin_toggle();


   // for (int i=0 ; i<DISPLAY_RX_FRAME_SIZE ; i++ ){
   //     printf("%02X ", *ptr++ );
   // }
   // printf("parseRxFrame()\n");
}

/* ex   0  1  2  3  4  5  6  7  8  9 10 11 12 13
 *     02 0E 01 00 80 00 00 00 17 70 00 00 FF 15 moteur arrêté
 *     02 0E 01 00 80 00 00 01 02 56 00 00 FF 27 13 Kmh
 *     02 0E 01 00 80 00 00 02 01 61 00 00 FF 10 22 Kmh
 *     02 0E 01 00 80 00 00 0C 01 83 01 00 FF FC 22 Kmh 2Amp
 **************************************************************/
static void sendStatus( void ){
    uint8_t checkSum = 0x0;

    //printf("Wheeltime = %d\r\n", gContext.Tx.Wheeltime);
    gContext.TxBuff[0]  = 0x02;
    gContext.TxBuff[1]  = 0x0E;
    gContext.TxBuff[2]  = 0x01;
    gContext.TxBuff[3]  = gContext.Tx.Error; // ??
    gContext.TxBuff[4]  = 0x80;
    gContext.TxBuff[5]  = 0x00;
    gContext.TxBuff[6]  = 0; // (uint8_t)(0xFF & gContext.Tx.CurrentX10>>8);
    gContext.TxBuff[7]  = 0; // (uint8_t)(0xFF & gContext.Tx.CurrentX10);
    gContext.TxBuff[8]  = (uint8_t)(0xFF & gContext.Tx.Wheeltime>>8);
    gContext.TxBuff[9]  = (uint8_t)(0xFF & gContext.Tx.Wheeltime);
    gContext.TxBuff[10] = 0x00;
    gContext.TxBuff[11] = 0x00; //
    gContext.TxBuff[12] = 0xFF;

    // Compute checkSum
    for (int i=0 ; i<(DISPLAY_TX_FRAME_SIZE-1); i++ ){
        checkSum ^= (uint8_t)(gContext.TxBuff[i]);
    }
    gContext.TxBuff[13] = checkSum;

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
        UserProbe5_H();
    else
        UserProbe5_L();

    if ( gContext.RxBuff[0] != 0x01 ){
        gContext.RxIndex = 0;
    }
    else if ( ( gContext.RxIndex > 0 )
           && ( gContext.RxBuff[1] != 0x14 ) ){
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
    // Send some data from controller to display
    sendStatus();

    //printf("gContext.RxIndex = %u\r\n", gContext.RxIndex );

    if ( gContext.RxIndex > 0)
        UserProbe5_H();
    else
        UserProbe5_L();

    if ( gContext.RxIndex >= DISPLAY_RX_FRAME_SIZE ){
        UserProbe5_H();
        // RxBuff[0] may be overwritten, RxBuff[0:1] already checked
        uint8_t checkSum = 0x01 ^ 0x14;
        // Compute checkSum
        for (int i=2 ; i<(DISPLAY_RX_FRAME_SIZE-1); i++ ){
            checkSum ^= (uint8_t)(gContext.RxBuff[i]);
        }

        if ( checkSum == gContext.RxBuff[DISPLAY_RX_FRAME_SIZE-1] ){
            // Parse data frame
            parseRxFrame();
        }
        else{
            printf("checkSum error: %X/%X\r\n", checkSum, gContext.RxBuff[DISPLAY_RX_FRAME_SIZE-1]);
        }
        gContext.RxIndex = 0;
    }

    if ( gContext.RxIndex == 0 ){
        HAL_UART_Receive_IT( &huart3, gContext.RxBuff, 1 );
    }

    if ( gContext.RxIndex > 0)
        UserProbe5_H();
    else
        UserProbe5_L();
}


uint8_t displayGetAssistLevel(void){
    return gContext.Rx.AssistLevel;
}


// Check SPEED_UNIT is _01HZ
#if (SPEED_UNIT != _01HZ)
#error
#endif
//
//  38165,2869976986 / rpm
// 381652 / 10rpm
uint8_t displaySetWheelTime( int16_t rpm10 ){
    if ( rpm10 < 90 )    // 1.x Kmh
        rpm10 = 90;
    if ( rpm10 > 4557 )  // 60 Kmh
        rpm10 = 4557;

    gContext.Tx.Wheeltime = (uint16_t)(382795L / (int32_t)rpm10);
    //gContext.Tx.Wheeltime = 200;
    return 0;
}

uint8_t displaySetError( DISPLAY_ERR_t err ){
    gContext.Tx.Error = 0;
    if ( err == DISPLAY_MOTOR_ERROR ){
        gContext.Tx.Error = DISPLAY_ERR_7;  // TODO
    }
    return gContext.Tx.Error;
}
