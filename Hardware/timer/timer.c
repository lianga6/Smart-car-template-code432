#include "timer.h"
#include "usart.h"
#include "PID.h"
#include "headfile.h"
#include "control.h"
#include <math.h>
#include <stdlib.h>
/*****************************************************************
 *Function: TimA0_Init(uint16_t ccr0, uint16_t psc)
 *Description:定时器TA1初始化
 *Input:ccr0为自动重装载值，psc为分频 1 2 4 8 3 5 6 7 10 12 14 16 20 24 28 32 40 48 56 64
 *Output:无
 *Return:无
 *Others:注意psc有限制，需查有效参数
 *Data:2021/09/19
 *****************************************************************/
uint16_t yanshi=0,PWM_L,PWM_R;
int pwm_out=0;
volatile int board_count=0;//计数

void TimA0_Init(uint16_t ccr0, uint16_t psc)//pwm输出
{
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION); //通道1
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION); //通道2
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION); //通道3
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION); //通道4

    const Timer_A_PWMConfig TimA0_PWMConfig = {
        TIMER_A_CLOCKSOURCE_SMCLK,         //时钟源
        0,                                 //时钟分频 范围1-64
        0,                                 //自动重装载值（ARR）
        TIMER_A_CAPTURECOMPARE_REGISTER_1, //通道1 （注意引脚定义）
        TIMER_A_OUTPUTMODE_TOGGLE_SET,     //输出模式
        0                                  //这里是改变占空比的地方 默认100%
    };
    Timer_A_PWMConfig *User_TimA0_PWMConfig = (Timer_A_PWMConfig*)&TimA0_PWMConfig;
    /*定时器PWM初始化*/
    User_TimA0_PWMConfig->clockSourceDivider = psc;
    User_TimA0_PWMConfig->timerPeriod = ccr0;
    User_TimA0_PWMConfig->dutyCycle = ccr0;

    //第1路 PWM
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &TimA0_PWMConfig); /* 初始化比较寄存器以产生 PWM1 */

    //第2路 PWM
    User_TimA0_PWMConfig->compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2; //通道2 （注意引脚定义）
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &TimA0_PWMConfig);                  /* 初始化比较寄存器以产生 PWM2 */

    //第3路 PWM
    User_TimA0_PWMConfig->compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3; //通道3 （注意引脚定义）
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &TimA0_PWMConfig);                  /* 初始化比较寄存器以产生 PWM3 */

    //第4路 PWM
    User_TimA0_PWMConfig->compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4; //通道4 （注意引脚定义）
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &TimA0_PWMConfig);                  /* 初始化比较寄存器以产生 PWM4 */
}
void TimA2_Init(uint16_t ccr0, uint16_t psc)
{
    /*增计数模式初始化*/
    Timer_A_UpModeConfig upConfig;
    upConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;                                      //时钟源
    upConfig.clockSourceDivider = psc;                                                     //时钟分频 范围1-64
    upConfig.timerPeriod = ccr0;                                                           //自动重装载值（ARR）
    upConfig.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;                   //禁用 tim溢出中断
    upConfig.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE; //启用 ccr0更新中断
    upConfig.timerClear = TIMER_A_DO_CLEAR;                                                // Clear value

    /*初始化定时器A*/
    MAP_Timer_A_configureUpMode(TIMER_A2_BASE, &upConfig);

    /*选择模式开始计数*/
    MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

    /*清除比较中断标志位*/
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    /*开启串口端口中断*/
    MAP_Interrupt_enableInterrupt(INT_TA2_0);
}

/*****************************************************************
 *Function: TimA1_PWM_Init(uint16_t ccr0, uint16_t psc)
 *Description:定时器TA1初始化
 *Input:ccr0为自动重装载值，psc为分频
 *Output:无
 *Return:无
 *Others:注意psc有限制，需查有效参数
 *Data:2021/09/19
 *****************************************************************/
void TimA1_Init(uint16_t ccr0, uint16_t psc)
{
    /*初始化引脚*/
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN7 | GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

    Timer_A_PWMConfig TimA1_PWMConfig;
    /*定时器PWM初始化*/
    TimA1_PWMConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;             //时钟源
    TimA1_PWMConfig.clockSourceDivider = psc;                            //时钟分频 范围1-64
    TimA1_PWMConfig.timerPeriod = ccr0;                                  //自动重装载值（ARR）
    TimA1_PWMConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1; //通道一 （引脚定义）
    TimA1_PWMConfig.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET;   //输出模式
    TimA1_PWMConfig.dutyCycle = ccr0;                                    //这里是改变占空比的地方 默认100%

    MAP_Timer_A_generatePWM(TIMER_A1_BASE, &TimA1_PWMConfig); /* 初始化比较寄存器以产生 PWM1 */	
		//第2路 PWM
    TimA1_PWMConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2; //通道二 （引脚定义）
    MAP_Timer_A_generatePWM(TIMER_A1_BASE, &TimA1_PWMConfig); 
	/* 初始化比较寄存器以产生 PWM1 */

}
void TA2_0_IRQHandler(void)//定时器A2中断服务函数是为编码电机服务的
{
    /*开始填充用户代码*/
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
    //这是进行阿克曼运算，计算时间很短，ns级别,这里是的阿克曼只为巡线服务
//    DJPC_TRAN(AKACK);
	
    encoder_L = 0;
    encoder_R = 0;
    /*结束填充用户代码*/
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}




