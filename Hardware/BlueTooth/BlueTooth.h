#ifndef __BT_RECV_H
#define __BT_RECV_H

#include "string.h"//用于处理文本字符数据

#define BT_USART_BASE       EUSCI_A2_BASE//蓝牙串口通道选择
#define BT_USART_SEND_INT   EUSCI_A2//蓝牙中断向量
#define BT_USART_PORT       GPIO_PORT_P3
#define BT_USART_RX_PIN     GPIO_PIN2
#define BT_USART_TX_PIN     GPIO_PIN3
#define BT_USART_BAUDRATE   9600//波特率

#define MyBT_IRQHandler EUSCIA2_IRQHandler

#define BT_START 0x9A // 帧头
#define BT_END 0xFB   // 帧尾

/******************函数**************************/
void BlueTooth_Init(void);
void BlueTooth_SendByte(char str);
void BlueTooth_SendString(char *txt);
void BlueTooth_SendPacket(char *data, int dataLength);//固定数据包发送，帧头帧尾宏定义修改
void BlueTooth_Receive_Data(uint8_t com_data);//接收数据处理

/********************变量*************************/

extern char Txt_RxPacket[];	//文本接收数据

#endif










