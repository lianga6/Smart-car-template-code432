#include <led.h>
#include "gpio.h"
/*****************************************************************
*Function: LED_Init(void)
*Description:LED��ʼ��
*Input:��
*Output:��
*Return:��
*Others:��
*Data:2021/09/14
*****************************************************************/
void LED_Init(void)
{
	/*LED1����*/
	GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
	/*LED2����*/
  GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
  GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
  GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);
	
	LED1_OFF;
	LED2_BLUE_OFF;
	LED2_GREEN_OFF;
	LED2_RED_OFF;
}


