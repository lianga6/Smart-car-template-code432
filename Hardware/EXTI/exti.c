#include "exti.h"
#include "delay.h"
#include "key.h"
#include "led.h"
#include "headfile.h"

/*****************************************************************
*Function: EXTI_Init(void)
*Description:外部中断初始化
*Input:无
*Output:无
*Return:无
*Others:无
*Data:2021/09/14
*****************************************************************/
void EXTI_Init(void)
{
    /*开启外部中断*/
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN7);
	  GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN3);
		GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN2);
    GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN6);
		GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN0);
	  GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN1);
		GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN4);
		GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN5);
    /*开启端口中断*/
    Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableInterrupt(INT_PORT3);
		Interrupt_enableInterrupt(INT_PORT6);
	  GPIO_clearInterruptFlag(GPIO_PORT_P1,GPIO_PIN7);
	  GPIO_clearInterruptFlag(GPIO_PORT_P3,GPIO_PIN6);
		GPIO_clearInterruptFlag(GPIO_PORT_P6,GPIO_PIN3);
	  GPIO_clearInterruptFlag(GPIO_PORT_P6,GPIO_PIN0);
		GPIO_clearInterruptFlag(GPIO_PORT_P6,GPIO_PIN1);
		GPIO_clearInterruptFlag(GPIO_PORT_P6,GPIO_PIN4);
		GPIO_clearInterruptFlag(GPIO_PORT_P6,GPIO_PIN5);
    /*配置触发方式*/
    GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN7, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_interruptEdgeSelect(GPIO_PORT_P3, GPIO_PIN6, GPIO_LOW_TO_HIGH_TRANSITION);
		GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN2, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN3, GPIO_LOW_TO_HIGH_TRANSITION);
		GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN0, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
    /*配置为浮空输入*/
		GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN3);
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN2);
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN7);
    GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN6);
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN1);
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN0);
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN4);
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN5);
}  

/*****************************************************************
*Function: PORT1_IRQHandler(void)
*Description:外部中断服务函数
*Input:无
*Output:无
*Return:无
*Others:中断服务函数，在中断内处理程序
*Data:2021/09/14
*****************************************************************/
void PORT1_IRQHandler(void)
{
    uint16_t flag;

    /*获取中断状态*/
    flag = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    /*清除中断标志位*/
    GPIO_clearInterruptFlag(GPIO_PORT_P1, flag);

    /*左轮编码器线A*/
    if (flag & GPIO_PIN7)
    {
        if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN3) == 0)
            encoder_L++;
        else if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN3) == 1)
            encoder_L--;
    }
	
}


void PORT3_IRQHandler(void)
{
	  uint16_t flag;

    /*获取中断状态*/
    flag = GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
    /*清除中断标志位*/
    GPIO_clearInterruptFlag(GPIO_PORT_P3, flag);
	
		/*右轮编码器线A*/
    if (flag & GPIO_PIN6)
    {
        if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN2) == 0)
            encoder_R++;
        else if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN2) == 1)
            encoder_R--;
    }
}
	
void PORT6_IRQHandler(void)
{		uint16_t flag;

    /*获取中断状态*/
    flag = GPIO_getEnabledInterruptStatus(GPIO_PORT_P6);
    /*清除中断标志位*/
    GPIO_clearInterruptFlag(GPIO_PORT_P6, flag);
	
	
		if (flag & GPIO_PIN3)
    {
        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == 1)
            encoder_L++;
        else if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == 0)
            encoder_L--;
    }
		
		if (flag & GPIO_PIN0)
    {
        if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN1) == 1)
            encoder_L++;
        else if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN1) == 0)
            encoder_L--;
    }
		
		if (flag & GPIO_PIN1)
    {
        if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN0) == 0)
            encoder_L++;
        else if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN0) == 1)
            encoder_L--;
    }
		
		if (flag & GPIO_PIN2)
    {
        if (GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN6) == 1)
            encoder_R++;
        else if (GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN6) == 0)
            encoder_R--;
    }

		if (flag & GPIO_PIN4)
    {
        if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN5) == 1)
            encoder_R++;
        else if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN5) == 0)
            encoder_R--;
    }
    /*右轮B相下降沿*/
    if (flag & GPIO_PIN5)
    {
        if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN4) == 0)
            encoder_R++;
        else if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN4) == 1)
            encoder_R--;
    }

}


