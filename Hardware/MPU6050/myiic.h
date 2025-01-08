#ifndef __MYIIC_H
#define __MYIIC_H

#include <driverlib.h>
#include <stdio.h>
//IO方向设置
#define SDA_IN()  GPIO_setAsInputPin(GPIO_PORT_P10,GPIO_PIN3)	//SDA输入模式
#define SDA_OUT() GPIO_setAsOutputPin(GPIO_PORT_P10,GPIO_PIN3) //SDA输出模式
//IO操作函数	 
#define IIC_SCL_High()  GPIO_setOutputHighOnPin(GPIO_PORT_P10,GPIO_PIN2) //SCL_High
#define IIC_SCL_Low()   GPIO_setOutputLowOnPin(GPIO_PORT_P10,GPIO_PIN2) //SCL_Low
#define IIC_SDA_High()  GPIO_setOutputHighOnPin(GPIO_PORT_P10,GPIO_PIN3) //SDA_High
#define IIC_SDA_Low()   GPIO_setOutputLowOnPin(GPIO_PORT_P10,GPIO_PIN3) //SDA_Low
#define READ_SDA        GPIO_getInputPinValue(GPIO_PORT_P10,GPIO_PIN3)  //输入SDA 
 
//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号
 
void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
#endif
















