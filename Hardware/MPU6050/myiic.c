#include "myiic.h"
#include "delay.h"
#include "usart.h"

//初始化IIC
void IIC_Init(void)
{
	GPIO_setAsOutputPin(GPIO_PORT_P10, GPIO_PIN3); 
	GPIO_setAsOutputPin(GPIO_PORT_P10, GPIO_PIN2); 
	IIC_SCL_High();
	IIC_SDA_High();
}
//产生IIC起始信号
void IIC_Start(void) //SDA 10 SCL 010
{
	SDA_OUT(); //sda线输出
	IIC_SCL_High();
	IIC_SDA_High();
	delay_us(4);
	IIC_SDA_Low(); //START:when CLK is high,DATA change form high to low
	delay_us(4);
	IIC_SCL_Low(); //钳住I2C总线，准备发送或接收数据
}
//产生IIC停止信号
void IIC_Stop(void) //SDA 01 SCL 01
{
	SDA_OUT();	   //sda线输出
	IIC_SCL_Low(); //STOP:when CLK is high DATA change form low to high
	IIC_SDA_Low();
	delay_us(4);
	IIC_SCL_High();
	IIC_SDA_High(); //发送I2C总线结束信号
	delay_us(4);
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void) //
{
	uint8_t cy;
	SDA_IN(); //SDA设置为输入
	IIC_SCL_High();
	delay_us(10);
	IIC_SDA_High();
	delay_us(10);
	if (READ_SDA)
	{
		cy = 1;
		IIC_SCL_Low();
		return cy;
	}
	else
	{
		cy = 0;
	}
	IIC_SCL_Low(); //时钟输出0
	return cy;
}
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL_Low();
	SDA_OUT();
	IIC_SDA_Low();
	delay_us(2);
	IIC_SCL_High();
	delay_us(2);
	IIC_SCL_Low();
}
//不产生ACK应答
void IIC_NAck(void)
{
	IIC_SCL_Low();
	SDA_OUT();
	IIC_SDA_High();
	delay_us(2);
	IIC_SCL_High();
	delay_us(2);
	IIC_SCL_Low();
}
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void IIC_Send_Byte(uint8_t txd)
{
	uint8_t t;
	SDA_OUT();
	IIC_SCL_Low(); //拉低时钟开始数据传输
	delay_us(2);
	for (t = 0; t < 8; t++)
	{
		if (txd & 0x80)
		{
			IIC_SDA_High();
			delay_us(2);
		}
		else
		{
			IIC_SDA_Low();
			delay_us(2);
		}
		txd <<= 1;
		IIC_SCL_High();
		delay_us(4);
		IIC_SCL_Low();
		delay_us(2);
	}
	delay_us(2);
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SDA_IN(); //SDA设置为输入
	for (i = 0; i < 8; i++)
	{
		IIC_SCL_Low();
		delay_us(2);
		IIC_SCL_High();
		receive <<= 1;
		if (READ_SDA)
			receive++;
		delay_us(2);
	}
	if (!ack)
		IIC_NAck(); //发送nACK
	else
		IIC_Ack(); //发送ACK
	return receive;
}
