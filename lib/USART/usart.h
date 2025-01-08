#ifndef __USART_H
#define __USART_H

#include <driverlib.h>
#include <stdio.h> //1.61328125kb
#include "headfile.h"

void uart_init(uint32_t baudRate);
void uart2_init(uint32_t baudRate);
void UART_NVIC_Init(void);
void UART_send_Byte(UART_CHA_enum UART_CHA, uint8 Data);
uint8 UART_recv_Byte(UART_CHA_enum UART_CHA);
void UART_send_string(UART_CHA_enum UART_CHA, char *txt);
void UART_send_short(UART_CHA_enum UART_CHA, uint16 num);
void UART_send_int(UART_CHA_enum UART_CHA, uint32 num);
void FrameJudge(uint8_t *Recv_Data,uint8_t *Sendbuffer);
void UART0_send_data(void);
void GetBufferdata(uint8_t *my_array);

extern int output;       

#endif
