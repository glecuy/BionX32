/* UART_Printf.h
 * Reference:

 TLY25/26/27/28/29/35 Communication protocol User’s manual (Tecnologic S.p.A.)

 */

#ifndef _UART_PRINTF_H_
#define _UART_PRINTF_H_

#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif

void PrintfEndOfTx(void);

void UART_printf( const char *fmt, ...);
void UART_PrintChar( char ch );

#ifdef __cplusplus
}
#endif
#endif //_UART_PRINTF_H_
