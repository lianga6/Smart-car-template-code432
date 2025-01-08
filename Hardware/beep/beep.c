#include "beep.h"
#include "gpio.h"
#include "delay.h"

//下面这个函数用于初始化蜂鸣器
void BEEP_Init(void)
{
    /*蜂鸣器配置*/
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);
    BEEP_OFF();
}

//一调用就让蜂鸣器响一下的函数
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


