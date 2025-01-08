#include "timer.h"
#include "usart.h"
#include "PID.h"
#include "headfile.h"
#include "control.h"
#include <math.h>
#include <stdlib.h>
/*****************************************************************
 *Function: TimA0_Init(uint16_t ccr0, uint16_t psc)
 *Description:��ʱ��TA1��ʼ��
 *Input:ccr0Ϊ�Զ���װ��ֵ��pscΪ��Ƶ 1 2 4 8 3 5 6 7 10 12 14 16 20 24 28 32 40 48 56 64
 *Output:��
 *Return:��
 *Others:ע��psc�����ƣ������Ч����
 *Data:2021/09/19
 *****************************************************************/
uint16_t yanshi=0,PWM_L,PWM_R;
int pwm_out=0;
volatile int board_count=0;//����

void TimA0_Init(uint16_t ccr0, uint16_t psc)//pwm���
{
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION); //ͨ��1
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION); //ͨ��2
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION); //ͨ��3
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION); //ͨ��4

    const Timer_A_PWMConfig TimA0_PWMConfig = {
        TIMER_A_CLOCKSOURCE_SMCLK,         //ʱ��Դ
        0,                                 //ʱ�ӷ�Ƶ ��Χ1-64
        0,                                 //�Զ���װ��ֵ��ARR��
        TIMER_A_CAPTURECOMPARE_REGISTER_1, //ͨ��1 ��ע�����Ŷ��壩
        TIMER_A_OUTPUTMODE_TOGGLE_SET,     //���ģʽ
        0                                  //�����Ǹı�ռ�ձȵĵط� Ĭ��100%
    };
    Timer_A_PWMConfig *User_TimA0_PWMConfig = (Timer_A_PWMConfig*)&TimA0_PWMConfig;
    /*��ʱ��PWM��ʼ��*/
    User_TimA0_PWMConfig->clockSourceDivider = psc;
    User_TimA0_PWMConfig->timerPeriod = ccr0;
    User_TimA0_PWMConfig->dutyCycle = ccr0;

    //��1· PWM
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &TimA0_PWMConfig); /* ��ʼ���ȽϼĴ����Բ��� PWM1 */

    //��2· PWM
    User_TimA0_PWMConfig->compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2; //ͨ��2 ��ע�����Ŷ��壩
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &TimA0_PWMConfig);                  /* ��ʼ���ȽϼĴ����Բ��� PWM2 */

    //��3· PWM
    User_TimA0_PWMConfig->compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3; //ͨ��3 ��ע�����Ŷ��壩
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &TimA0_PWMConfig);                  /* ��ʼ���ȽϼĴ����Բ��� PWM3 */

    //��4· PWM
    User_TimA0_PWMConfig->compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4; //ͨ��4 ��ע�����Ŷ��壩
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &TimA0_PWMConfig);                  /* ��ʼ���ȽϼĴ����Բ��� PWM4 */
}
void TimA2_Init(uint16_t ccr0, uint16_t psc)
{
    /*������ģʽ��ʼ��*/
    Timer_A_UpModeConfig upConfig;
    upConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;                                      //ʱ��Դ
    upConfig.clockSourceDivider = psc;                                                     //ʱ�ӷ�Ƶ ��Χ1-64
    upConfig.timerPeriod = ccr0;                                                           //�Զ���װ��ֵ��ARR��
    upConfig.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;                   //���� tim����ж�
    upConfig.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE; //���� ccr0�����ж�
    upConfig.timerClear = TIMER_A_DO_CLEAR;                                                // Clear value

    /*��ʼ����ʱ��A*/
    MAP_Timer_A_configureUpMode(TIMER_A2_BASE, &upConfig);

    /*ѡ��ģʽ��ʼ����*/
    MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

    /*����Ƚ��жϱ�־λ*/
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    /*�������ڶ˿��ж�*/
    MAP_Interrupt_enableInterrupt(INT_TA2_0);
}

/*****************************************************************
 *Function: TimA1_PWM_Init(uint16_t ccr0, uint16_t psc)
 *Description:��ʱ��TA1��ʼ��
 *Input:ccr0Ϊ�Զ���װ��ֵ��pscΪ��Ƶ
 *Output:��
 *Return:��
 *Others:ע��psc�����ƣ������Ч����
 *Data:2021/09/19
 *****************************************************************/
void TimA1_Init(uint16_t ccr0, uint16_t psc)
{
    /*��ʼ������*/
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN7 | GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

    Timer_A_PWMConfig TimA1_PWMConfig;
    /*��ʱ��PWM��ʼ��*/
    TimA1_PWMConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;             //ʱ��Դ
    TimA1_PWMConfig.clockSourceDivider = psc;                            //ʱ�ӷ�Ƶ ��Χ1-64
    TimA1_PWMConfig.timerPeriod = ccr0;                                  //�Զ���װ��ֵ��ARR��
    TimA1_PWMConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1; //ͨ��һ �����Ŷ��壩
    TimA1_PWMConfig.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET;   //���ģʽ
    TimA1_PWMConfig.dutyCycle = ccr0;                                    //�����Ǹı�ռ�ձȵĵط� Ĭ��100%

    MAP_Timer_A_generatePWM(TIMER_A1_BASE, &TimA1_PWMConfig); /* ��ʼ���ȽϼĴ����Բ��� PWM1 */	
		//��2· PWM
    TimA1_PWMConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2; //ͨ���� �����Ŷ��壩
    MAP_Timer_A_generatePWM(TIMER_A1_BASE, &TimA1_PWMConfig); 
	/* ��ʼ���ȽϼĴ����Բ��� PWM1 */

}
void TA2_0_IRQHandler(void)//��ʱ��A2�жϷ�������Ϊ�����������
{
    /*��ʼ����û�����*/
	MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    printf("%d,%d\r\n",encoder_L,encoder_R);
		
    GET_ENCODER_NUM();
    Speed_control();
    board_count++;
		switch(abs((int)output))
    {
        case 0:
            pwm_out = 0;
            break;
        case 2:
            pwm_out = 2;
            break;
        case 4:
            pwm_out =5;
            break;
        case 6:
            pwm_out = 9;
            break;
        case 8:
            pwm_out = 11;
            break;
				 case 12:
            pwm_out = 14   ;
            break;
        case 17:
            pwm_out = 16;
            break;
        case 23:
            pwm_out =20;
        default:
            break;
    }
		if(output<0)pwm_out=-pwm_out;
    //���ǽ��а��������㣬����ʱ��̣ܶ�ns����,�����ǵİ�����ֻΪѲ�߷���
//    DJPC_TRAN(AKACK);
	
    encoder_L = 0;
    encoder_R = 0;
    /*��������û�����*/
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}




