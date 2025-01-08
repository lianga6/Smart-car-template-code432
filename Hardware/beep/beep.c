#include "beep.h"
#include "gpio.h"
#include "delay.h"

//��������������ڳ�ʼ��������
void BEEP_Init(void)
{
    /*����������*/
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);
    BEEP_OFF();
}

//һ���þ��÷�������һ�µĺ���
void BEEP_sound(void)
{
    uint8_t i=100;
    while(i--)
    {
        BEEP_ON();
        delay_us(500);
        BEEP_OFF();
        delay_us(500);
    }
    
}


