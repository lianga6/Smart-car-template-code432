#ifndef __BT_RECV_H
#define __BT_RECV_H

#include "string.h"//���ڴ����ı��ַ�����

#define BT_USART_BASE       EUSCI_A2_BASE//��������ͨ��ѡ��
#define BT_USART_SEND_INT   EUSCI_A2//�����ж�����
#define BT_USART_PORT       GPIO_PORT_P3
#define BT_USART_RX_PIN     GPIO_PIN2
#define BT_USART_TX_PIN     GPIO_PIN3
#define BT_USART_BAUDRATE   9600//������

#define MyBT_IRQHandler EUSCIA2_IRQHandler

#define BT_START 0x9A // ֡ͷ
#define BT_END 0xFB   // ֡β

/******************����**************************/
void BlueTooth_Init(void);
void BlueTooth_SendByte(char str);
void BlueTooth_SendString(char *txt);
void BlueTooth_SendPacket(char *data, int dataLength);//�̶����ݰ����ͣ�֡ͷ֡β�궨���޸�
void BlueTooth_Receive_Data(uint8_t com_data);//�������ݴ���

/********************����*************************/

extern char Txt_RxPacket[];	//�ı���������

#endif










