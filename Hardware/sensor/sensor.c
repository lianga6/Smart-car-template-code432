#include "sensor.h"
#include "gpio.h"

//下面这个函数用于初始化五路灰度传感器,输出要接上拉电阻
void gray_Init(void)
{
    /*五路灰度传感器配置*/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN1);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN3);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN4);
}

//下面这个函数是分别读取五路灰度传感器的值，num为灰度传感器的编号，从0开始，返回值为0或1，1表示识别到黑线，0表示没有
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
//下面这个函数用于读取五路灰度传感器的值，返回值为一个5位二进制数，每一位表示一个传感器的值，1表示有障碍物，0表示没有障碍物
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

//下面这个函数是为了获取五路灰度的偏差值
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










