#include "key.h"

/*****************************************************************
*Function: KEY_Init(void)
*Description:按键初始化
*Input:无
*Output:无
*Return:无
*Others:无
*Data:2021/09/14
*****************************************************************/
void KEY_Init(void)
{
	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
}

/*****************************************************************
*Function: KEY_Scan(uint8_t mode)
*Description:按键扫描
*Input:是否支持连按，1支持连按，0不支持连按
*Output:无
*Return:键值
*Others:无
*Data:2021/09/14
*****************************************************************/
uint8_t KEY_Scan(uint8_t mode)
{
	static uint8_t key_up = 1; //按键按松开标志
	int i = 0;
	if (mode)
		key_up = 1; //支持连按
	if (key_up && (KEY1 == 0 || KEY2 == 0))
	{
		for (i = 0; i < 5000; i++)
			;
		key_up = 0;
		if (KEY1 == 0)
			return KEY1_PRES;
		else if (KEY2 == 0)
			return KEY2_PRES;
	}
	else if (KEY1 == 1 && KEY2 == 1)
		key_up = 1;
	return 0; // 无按键按下
}
