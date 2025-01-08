#include "headfile.h"
#include "HC-SR0.h"

uint32_t ECHO_Distance;//超声波返回数据

#define ECHO_WAY1 1//输入捕获获取
#define ECHO_WAY2 0//外部中断获取(还未完善)

#define ECHO_WAY ECHO_WAY1

/************************说明******************************/
/*使用时在需要的地方调用以下代码即可,最后的距离保存在distance_cm这个变量当中为float类型
    TimA3_Cap_Init();
    while(1)
    {
          SR0_Send();
          if (TIMA3_CAP_STA & 0X80) //成功捕获到了一次上升沿
            {
                Get_Distance();
                printf("HIGH:%f\r\n", distance_cm); //打印总的高点平时间
                TIMA3_CAP_STA = 0;             //开启下一次捕获
            }
    }
*/
float distance_cm;
float cap_temp;//暂时存入捕获的总值

void SR0_Send(void)
{
    // 发送10us的高电平脉冲
    Trig_ON;
    delay_us(10);
    Trig_OFF;
}



#if (ECHO_WAY==ECHO_WAY1)

void TimA3_Cap_Init(void)
{
    //io口初始化
     MAP_GPIO_setAsOutputPin(CAP_OUTPORT,CAP_OUTPIN);//tring
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(CAP_PORT_PIN, GPIO_PRIMARY_MODULE_FUNCTION);//echo脚
    

    /* 定时器配置参数*/
    Timer_A_ContinuousModeConfig continuousModeConfig = {
        TIMER_A_CLOCKSOURCE_SMCLK,      // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_48, // SMCLK/48 = 1MHz
        TIMER_A_TAIE_INTERRUPT_ENABLE,  // 开启定时器溢出中断
        TIMER_A_DO_CLEAR                // Clear Counter
    };
    // 3.将定时器初始化为连续计数模式
    MAP_Timer_A_configureContinuousMode(CAP_TIMA_SELECTION, &continuousModeConfig);

    // 4.配置捕捉模式结构体 */
    const Timer_A_CaptureModeConfig captureModeConfig_TA3 = {
        CAP_REGISTER_SELECTION,                      //在这里改引脚
        TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE, //上升下降沿捕获
        TIMER_A_CAPTURE_INPUTSELECT_CCIxA,           //CCIxA:外部引脚输入  （CCIxB:与内部ACLK连接(手册)
        TIMER_A_CAPTURE_SYNCHRONOUS,                 //同步捕获
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,     //开启CCRN捕获中断
        TIMER_A_OUTPUTMODE_OUTBITVALUE               //输出位值
    };
    // 5.初始化定时器的捕获模式
    MAP_Timer_A_initCapture(CAP_TIMA_SELECTION, &captureModeConfig_TA3);

    // 6.选择连续模式计数开始计数
    MAP_Timer_A_startCounter(CAP_TIMA_SELECTION, TIMER_A_CONTINUOUS_MODE);

    // 7.清除中断标志位
    MAP_Timer_A_clearInterruptFlag(CAP_TIMA_SELECTION);                                   //清除定时器溢出中断标志位
    MAP_Timer_A_clearCaptureCompareInterrupt(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION); //清除 CCR1 更新中断标志位

    // 8.开启定时器端口中断
    MAP_Interrupt_enableInterrupt(INT_TA3_N); //开启定时器A3端口中断
}


// TIMA3_CAP_STA 捕获状态
// [7]:捕获高电平完成状态
// [6]:0表示未捕获到上升沿，1表示捕获过上升沿
// [5:0]:溢出次数
uint8_t TIMA3_CAP_STA = 0;
uint16_t TIMA3_CAP_VAL = 0;

void TA3_N_IRQHandler(void)
{
    if ((TIMA3_CAP_STA & 0X80) == 0) //还未成功捕获
    {
        if (MAP_Timer_A_getEnabledInterruptStatus(CAP_TIMA_SELECTION)) //溢出中断
        {
            MAP_Timer_A_clearInterruptFlag(CAP_TIMA_SELECTION); //清除定时器溢出中断标志位

            /* ★ 软件复位COV ★ */
            /* 这里UP忘记讲了，如果在未清除中断位值时，来了一次中断，COV会置位，需要软件复位，这里没有官方库函数。具体可以参考技术手册(slau356h.pdf) P790 */
            BITBAND_PERI(TIMER_A_CMSIS(CAP_TIMA_SELECTION)->CCTL[CAP_CCR_NUM], TIMER_A_CCTLN_COV_OFS) = 0;

            if (TIMA3_CAP_STA & 0X40) //已经捕获到高电平了 40H = 0x01000000
            {
                if ((TIMA3_CAP_STA & 0X3F) == 0X3F) //高电平太长了
                {
                    TIMA3_CAP_STA |= 0X80; //强制标记成功捕获完高电平 80H = 0x10000000
                    TIMA3_CAP_VAL = 0XFFFF;
                }
                else
                    TIMA3_CAP_STA++; //溢出次数加1
            }
        }

        if (MAP_Timer_A_getCaptureCompareEnabledInterruptStatus(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION)) //捕获中断
        {
            MAP_Timer_A_clearCaptureCompareInterrupt(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION); //清除 CCR1 更新中断标志位

            //判断是否捕获到下降沿
            if (TIMA3_CAP_STA & 0X40 && (MAP_Timer_A_getSynchronizedCaptureCompareInput(CAP_TIMA_SELECTION,CAP_REGISTER_SELECTION,TIMER_A_READ_CAPTURE_COMPARE_INPUT) == TIMER_A_CAPTURECOMPARE_INPUT_LOW))
            {
                TIMA3_CAP_STA |= 0X80; //标记成功捕获完高电平
                TIMA3_CAP_VAL = Timer_A_getCaptureCompareCount(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION);
                
                cap_temp = TIMA3_CAP_STA & 0X3F;
                cap_temp *= 65536;                 //溢出时间总和
                cap_temp += TIMA3_CAP_VAL;         //得到总的高电平时间
                
            }
            else //还未开始,第一次捕获上升沿
            {
                TIMA3_CAP_STA = 0;
                TIMA3_CAP_VAL = 0;
                MAP_Timer_A_clearTimer(CAP_TIMA_SELECTION); //清空定时器 重新从0计数
                TIMA3_CAP_STA |= 0X40;                      //标记捕获到了上升沿
            }
        }
    }
}

void Get_Distance(void)
{    
    
    distance_cm = cap_temp * 340.0f/2*0.000001f*100.0f;
}
#elif (ECHO_WAY==ECHO_WAY2)

TIM_HandleTypeDef TIM4_Handler;
uint32_t pulse_width_us = 0;
uint32_t distance_cm = 0;

void HCSR04_Init(void)
{
		/*Trig引脚*/
    MAP_GPIO_setAsOutputPin(CAP_OUTPORT,CAP_OUTPIN);//Trig引脚
    
    
    
    GPIO_enableInterrupt(CAP_INPUTPORT,CAP_INPUTPIN);//外部中断脚
    Interrupt_enableInterrupt(CAP_INPUTPORT);
    GPIO_clearInterruptFlag(CAP_INPUTPORT,CAP_INPUTPIN);
    GPIO_interruptEdgeSelect(CAP_INPUTPORT, CAP_INPUTPIN, GPIO_LOW_TO_HIGH_TRANSITION);//低到高电平触发
    GPIO_setAsInputPinWithPullUpResistor(CAP_INPUTPORT, CAP_INPUTPIN);
    Interrupt_setPriority(CAP_INT_PORT,1<<4);
    		
}




void TimA3_Init(uint16_t ccr0, uint16_t psc)
{
    /*增计数模式初始化*/
    Timer_A_UpModeConfig upConfig;
    upConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;                                      //时钟源
    upConfig.clockSourceDivider = psc;                                                     //时钟分频 范围1-64 63
    upConfig.timerPeriod = ccr0;                                                           //自动重装载值（ARR）0xffff
    upConfig.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;                   //禁用 tim溢出中断
    upConfig.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE; //启用 ccr0更新中断
    upConfig.timerClear = TIMER_A_DO_CLEAR;                                                // Clear value

    /*初始化定时器A*/
    MAP_Timer_A_configureUpMode(TIMER_A3_BASE, &upConfig);

    /*选择模式开始计数*/
    MAP_Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_UP_MODE);

    /*清除比较中断标志位*/
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    /*开启串口端口中断*/
    MAP_Interrupt_enableInterrupt(INT_TA3_0);
}


//外部中断中断处理函数
void PORT10_IRQHandler(void)
{
    uint16_t flag;

    /*获取中断状态*/
    flag = GPIO_getEnabledInterruptStatus(CAP_INPUTPORT);
    /*清除中断标志位*/
    GPIO_clearInterruptFlag(GPIO_PORT_P10, flag);

    //输入引脚是高电平
    if (flag & CAP_INPUTPIN)
    {
        if (GPIO_getInputPinValue(CAP_INPUTPORT,CAP_INPUTPIN) == 1)
        {
            
        }
        else if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN3) == 1)encoder_L--;
    }
}








void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		static int count = 0;
		if(GPIO_Pin == GPIO_PIN_9)
    {
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9))
			{
				HAL_TIM_Base_Start(&TIM4_Handler);                   // 启动定时器
				__HAL_TIM_SetCounter(&TIM4_Handler, 0);							 //清空定时器的值
			}
			else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == 0)
			{
				HAL_TIM_Base_Stop(&TIM4_Handler);                   // 停止定时器
				count = __HAL_TIM_GetCounter(&TIM4_Handler);				//获取当前计数值
				distance_cm = count * 340/2*0.000001*100;
				printf("distance_cm is %d\r\n", distance_cm);								
				count = 0;
			}
    }
}

/*返回距离*/
uint32_t Get_Distance(void)
{
	return distance_cm;
}


#endif


