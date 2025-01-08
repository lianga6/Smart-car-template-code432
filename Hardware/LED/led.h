#ifndef __LED_H	
#define __LED_H	 

#include <stdint.h>
#include <stdbool.h>
#include <driverlib.h>

/*LED1��*/
#define LED1_ON GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0)
/*LED2�����*/
#define LED2_RED_ON GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0)
/*LED2�̵���*/
#define LED2_GREEN_ON GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN1)
/*LED2������*/
#define LED2_BLUE_ON GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN2)

/*LED1��ת*/
#define LED1_TOG 	GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0)
/*LED2��Ʒ�ת*/
#define LED2_RED_TOG 	GPIO_toggleOutputOnPin(GPIO_PORT_P2,GPIO_PIN0)
/*LED2�̵Ʒ�ת*/
#define LED2_GREEN_TOG 	GPIO_toggleOutputOnPin(GPIO_PORT_P2,GPIO_PIN1)
/*LED2���Ʒ�ת*/
#define LED2_BLUE_TOG 	GPIO_toggleOutputOnPin(GPIO_PORT_P2,GPIO_PIN2)

/*LED1��*/
#define LED1_OFF GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0)
/*LED2�����*/
#define LED2_RED_OFF GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0)
/*LED2�̵���*/
#define LED2_GREEN_OFF GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1)
/*LED2������*/
#define LED2_BLUE_OFF GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2)

void LED_Init(void);

#endif

