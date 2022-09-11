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

// Uart3 is used to communicate with display
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

    uint8_t EndOfRx;
    uint8_t EndOfTx;

}DISPLAY_CONTEXT_t;

DISPLAY_CONTEXT_t gContext;

void displayInit(void) {
    gContext.Tx.Wheeltime = 201; // mSec (25 KmH)
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
static void sendResponse( void ){
    uint8_t checkSum = 0x0;

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

    //UART3_TrigSendingBytes( gContext.RxTxBuff, DISPLAY_TX_FRAME_SIZE);
    HAL_UART_Transmit_IT( &huart3, gContext.TxBuff, DISPLAY_TX_FRAME_SIZE);
    gContext.EndOfTx = 0;
}

void displayInitRxTx(void){
    gContext.EndOfTx = 1;
    gContext.EndOfRx = 1;
}

void displayEndOfRx(){
    gContext.EndOfRx = 1;
}

void displayEndOfTx(void){
    gContext.EndOfTx = 1;
}

int serviceCNT=0;
void displayService(void){

    printf("displayService(%d/%d)\r\n", gContext.EndOfTx, gContext.EndOfRx);
    if ( gContext.EndOfRx ){
        uint8_t checkSum = 0x0;
        // Compute checkSum
        for (int i=0 ; i<(DISPLAY_RX_FRAME_SIZE-1); i++ ){
            checkSum ^= (uint8_t)(gContext.RxBuff[i]);
        }

        if ( checkSum == gContext.RxBuff[DISPLAY_RX_FRAME_SIZE-1] ){
            // Parse data frame
            parseRxFrame();
            // Send some data from controller to display
            sendResponse();
            printf("sendResponse done\r\n");
        }
        else{
            printf("checkSum error: %X/%X\n", checkSum, gContext.RxBuff[DISPLAY_RX_FRAME_SIZE-1]);
        }
    }

    // TODO:  Improve sync
    HAL_StatusTypeDef st;
    do{
        st = HAL_UART_Receive( &huart3, gContext.RxBuff, 1, 0 );
    }while (st == HAL_OK );

    if ( gContext.EndOfRx != 0 ){
        st = HAL_UART_Receive_IT( &huart3, gContext.RxBuff, DISPLAY_RX_FRAME_SIZE );
        //HAL_UART_Receive( &huart3, gContext.RxBuff, DISPLAY_RX_FRAME_SIZE, 1000 );
        gContext.EndOfRx = 0;
    }

}

uint8_t displaySetWheelTime( uint16_t tMs ){
    gContext.Tx.Wheeltime = tMs;
    return 0;
}

uint8_t displaySetError( DISPLAY_ERR_t err ){
    gContext.Tx.Error = 0;
    if ( err == DISPLAY_MOTOR_ERROR ){
        gContext.Tx.Error = DISPLAY_ERR_7;  // Todo
    }
    return gContext.Tx.Error;
}
