#ifndef __KEY_H
#define __KEY_H	 

#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"

#define KEY2  GPIO_getInputPinValue(GPIO_PORT_P1,GPIO_PIN4)//读取按键0
#define KEY1  GPIO_getInputPinValue(GPIO_PORT_P1,GPIO_PIN1)//读取按键1

 

#define KEY1_PRES	1		//KEY0  
#define KEY2_PRES	2		//KEY1 


void KEY_Init(void);//IO初始化
uint8_t KEY_Scan(uint8_t mode);  	//按键扫描函数					    
#endif
