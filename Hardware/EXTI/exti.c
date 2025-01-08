#include "exti.h"
#include "delay.h"
#include "key.h"
#include "led.h"
#include "headfile.h"

/*****************************************************************
*Function: EXTI_Init(void)
*Description:�ⲿ�жϳ�ʼ��
*Input:��
*Output:��
*Return:��
*Others:��
*Data:2021/09/14
*****************************************************************/
void EXTI_Init(void)
{
    /*�����ⲿ�ж�*/
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN7);
	  GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN3);
		GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN2);
    GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN6);
		GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN0);
	  GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN1);
		GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN4);
		GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN5);
    /*�����˿��ж�*/
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
    /*���ô�����ʽ*/
    GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN7, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_interruptEdgeSelect(GPIO_PORT_P3, GPIO_PIN6, GPIO_LOW_TO_HIGH_TRANSITION);
		GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN2, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN3, GPIO_LOW_TO_HIGH_TRANSITION);
		GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN0, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
    /*����Ϊ��������*/
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
*Description:�ⲿ�жϷ�����
*Input:��
*Output:��
*Return:��
*Others:�жϷ����������ж��ڴ������
*Data:2021/09/14
*****************************************************************/
void PORT1_IRQHandler(void)
{
    uint16_t flag;

    /*��ȡ�ж�״̬*/
    flag = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    /*����жϱ�־λ*/
    GPIO_clearInterruptFlag(GPIO_PORT_P1, flag);

    /*���ֱ�������A*/
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

    /*��ȡ�ж�״̬*/
    flag = GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
    /*����жϱ�־λ*/
    GPIO_clearInterruptFlag(GPIO_PORT_P3, flag);
	
		/*���ֱ�������A*/
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

    /*��ȡ�ж�״̬*/
    flag = GPIO_getEnabledInterruptStatus(GPIO_PORT_P6);
    /*����жϱ�־λ*/
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
    /*����B���½���*/
    if (flag & GPIO_PIN5)
    {
        if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN4) == 0)
            encoder_R++;
        else if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN4) == 1)
            encoder_R--;
    }

}


