#include "myiic.h"
#include "delay.h"
#include "usart.h"

//��ʼ��IIC
void IIC_Init(void)
{
	GPIO_setAsOutputPin(GPIO_PORT_P10, GPIO_PIN3); 
	GPIO_setAsOutputPin(GPIO_PORT_P10, GPIO_PIN2); 
	IIC_SCL_High();
	IIC_SDA_High();
}
//����IIC��ʼ�ź�
void IIC_Start(void) //SDA 10 SCL 010
{
	SDA_OUT(); //sda�����
	IIC_SCL_High();
	IIC_SDA_High();
	delay_us(4);
	IIC_SDA_Low(); //START:when CLK is high,DATA change form high to low
	delay_us(4);
	IIC_SCL_Low(); //ǯסI2C���ߣ�׼�����ͻ��������
}
//����IICֹͣ�ź�
void IIC_Stop(void) //SDA 01 SCL 01
{
	SDA_OUT();	   //sda�����
	IIC_SCL_Low(); //STOP:when CLK is high DATA change form low to high
	IIC_SDA_Low();
	delay_us(4);
	IIC_SCL_High();
	IIC_SDA_High(); //����I2C���߽����ź�
	delay_us(4);
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void) //
{
	uint8_t cy;
	SDA_IN(); //SDA����Ϊ����
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
	IIC_SCL_Low(); //ʱ�����0
	return cy;
}
//����ACKӦ��
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
//������ACKӦ��
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��
void IIC_Send_Byte(uint8_t txd)
{
	uint8_t t;
	SDA_OUT();
	IIC_SCL_Low(); //����ʱ�ӿ�ʼ���ݴ���
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
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SDA_IN(); //SDA����Ϊ����
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
		IIC_NAck(); //����nACK
	else
		IIC_Ack(); //����ACK
	return receive;
}
