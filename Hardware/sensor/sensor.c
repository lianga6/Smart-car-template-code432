#include "sensor.h"
#include "gpio.h"

//��������������ڳ�ʼ����·�Ҷȴ�����,���Ҫ����������
void gray_Init(void)
{
    /*��·�Ҷȴ���������*/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN1);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN3);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN4);
}

//������������Ƿֱ��ȡ��·�Ҷȴ�������ֵ��numΪ�Ҷȴ������ı�ţ���0��ʼ������ֵΪ0��1��1��ʾʶ�𵽺��ߣ�0��ʾû��
unsigned char gray_separate_read(unsigned char num)
{
    unsigned char Pinvalue=0;
    switch(num)
    {
        case 0:
            Pinvalue = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN0)!=0?0x01:0x00;
            break;
        case 1:
            Pinvalue = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN1)!=0?0x01:0x00;
            break;
        case 2:
            Pinvalue = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN2)!=0?0x01:0x00;
            break;
        case 3:
            Pinvalue = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN3)!=0?0x01:0x00;
            break;
        case 4:
            Pinvalue = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN4)!=0?0x01:0x00;
            break;
        default:
            break;
    }
    return Pinvalue;
}
//��������������ڶ�ȡ��·�Ҷȴ�������ֵ������ֵΪһ��5λ����������ÿһλ��ʾһ����������ֵ��1��ʾ���ϰ��0��ʾû���ϰ���
unsigned char gray_Read(void)
{   
    unsigned char Pinvalue=0;
    Pinvalue |= GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN0) << 0;
    Pinvalue |= GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN1) << 1;
    Pinvalue |= GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN2) << 2;
    Pinvalue |= GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN3) << 3;
    Pinvalue |= GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN4) << 4;
    return Pinvalue;
}

//�������������Ϊ�˻�ȡ��·�Ҷȵ�ƫ��ֵ
int gray_get_bias(void)
{
    static int bias = 0;
    int weight[5] = {7, 3, 0, -3, -7};
    unsigned char i;
    unsigned char i1=2;
    unsigned char i2=2;
    for (i = 4; i >= 2; i--)
    {
        if(gray_separate_read(i) == 0)
        {
            i1=i;
            break;
        }
    }
    for ( i = 0; i<= 2; i++)
    {
        if(gray_separate_read(i) == 0)
        {
            i2=i;
            break;
        }
    }
    if((i1-2)>(2-i2))
    {
        bias = weight[i1];
    }
    else if((i1-2)<(2-i2))
    {
        bias = weight[i2];
    }
    else
    {
        bias = 0;
    }
    return bias;
}










