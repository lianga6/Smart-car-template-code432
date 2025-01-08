#ifndef __MYIIC_H
#define __MYIIC_H

#include <driverlib.h>
#include <stdio.h>
//IO��������
#define SDA_IN()  GPIO_setAsInputPin(GPIO_PORT_P10,GPIO_PIN3)	//SDA����ģʽ
#define SDA_OUT() GPIO_setAsOutputPin(GPIO_PORT_P10,GPIO_PIN3) //SDA���ģʽ
//IO��������	 
#define IIC_SCL_High()  GPIO_setOutputHighOnPin(GPIO_PORT_P10,GPIO_PIN2) //SCL_High
#define IIC_SCL_Low()   GPIO_setOutputLowOnPin(GPIO_PORT_P10,GPIO_PIN2) //SCL_Low
#define IIC_SDA_High()  GPIO_setOutputHighOnPin(GPIO_PORT_P10,GPIO_PIN3) //SDA_High
#define IIC_SDA_Low()   GPIO_setOutputLowOnPin(GPIO_PORT_P10,GPIO_PIN3) //SDA_Low
#define READ_SDA        GPIO_getInputPinValue(GPIO_PORT_P10,GPIO_PIN3)  //����SDA 
 
//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8_t IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�
 
void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
#endif
















