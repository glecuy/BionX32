
#include "main.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

#include <stdarg.h>
#include <stdio.h>

void UART_printf( const char *fmt, ...);

#define UART2_TX_BUFF_LEN (120)
// Send text up to UART2_TX_BUFF_LEN chars at once
uint8_t uart2TxBuffer[UART2_TX_BUFF_LEN];

volatile uint8_t uart2TxDone;


void PrintfEndOfTx(void){
    uart2TxDone = 1;
}

void UART_printf( const char *fmt, ...){
    va_list ap;

    if ( uart2TxDone == 0 ){
        // Wait a while, current message to be sent !
        HAL_Delay (10);
    }
    va_start(ap, fmt);
    vsnprintf( (char*)uart2TxBuffer, UART2_TX_BUFF_LEN, fmt, ap);
    va_end(ap);

    uart2TxDone = 0;
    HAL_UART_Transmit_IT(&huart2, uart2TxBuffer, strlen((char*)uart2TxBuffer));
    //HAL_UART_Transmit(&huart2, uart2TxBuffer, strlen((char*)uart2TxBuffer), 0xFFFF); uart2TxDone = 1;

}

void UART_PrintChar( char ch ){
    if ( uart2TxDone == 0 ){
        // Wait a while, current message to be sent !
        HAL_Delay (10);
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
}




