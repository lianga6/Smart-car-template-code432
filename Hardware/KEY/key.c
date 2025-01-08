#include "key.h"

/*****************************************************************
*Function: KEY_Init(void)
*Description:������ʼ��
*Input:��
*Output:��
*Return:��
*Others:��
*Data:2021/09/14
*****************************************************************/
void KEY_Init(void)
{
	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
}

/*****************************************************************
*Function: KEY_Scan(uint8_t mode)
*Description:����ɨ��
*Input:�Ƿ�֧��������1֧��������0��֧������
*Output:��
*Return:��ֵ
*Others:��
*Data:2021/09/14
*****************************************************************/
uint8_t KEY_Scan(uint8_t mode)
{
	static uint8_t key_up = 1; //�������ɿ���־
	int i = 0;
	if (mode)
		key_up = 1; //֧������
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
	return 0; // �ް�������
}
