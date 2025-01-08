#include "headfile.h"
#include "HC-SR0.h"

uint32_t ECHO_Distance;//��������������

#define ECHO_WAY1 1//���벶���ȡ
#define ECHO_WAY2 0//�ⲿ�жϻ�ȡ(��δ����)

#define ECHO_WAY ECHO_WAY1

/************************˵��******************************/
/*ʹ��ʱ����Ҫ�ĵط��������´��뼴��,���ľ��뱣����distance_cm�����������Ϊfloat����
    TimA3_Cap_Init();
    while(1)
    {
          SR0_Send();
          if (TIMA3_CAP_STA & 0X80) //�ɹ�������һ��������
            {
                Get_Distance();
                printf("HIGH:%f\r\n", distance_cm); //��ӡ�ܵĸߵ�ƽʱ��
                TIMA3_CAP_STA = 0;             //������һ�β���
            }
    }
*/
float distance_cm;
float cap_temp;//��ʱ���벶�����ֵ

void SR0_Send(void)
{
    // ����10us�ĸߵ�ƽ����
    Trig_ON;
    delay_us(10);
    Trig_OFF;
}



#if (ECHO_WAY==ECHO_WAY1)

void TimA3_Cap_Init(void)
{
    //io�ڳ�ʼ��
     MAP_GPIO_setAsOutputPin(CAP_OUTPORT,CAP_OUTPIN);//tring
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(CAP_PORT_PIN, GPIO_PRIMARY_MODULE_FUNCTION);//echo��
    

    /* ��ʱ�����ò���*/
    Timer_A_ContinuousModeConfig continuousModeConfig = {
        TIMER_A_CLOCKSOURCE_SMCLK,      // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_48, // SMCLK/48 = 1MHz
        TIMER_A_TAIE_INTERRUPT_ENABLE,  // ������ʱ������ж�
        TIMER_A_DO_CLEAR                // Clear Counter
    };
    // 3.����ʱ����ʼ��Ϊ��������ģʽ
    MAP_Timer_A_configureContinuousMode(CAP_TIMA_SELECTION, &continuousModeConfig);

    // 4.���ò�׽ģʽ�ṹ�� */
    const Timer_A_CaptureModeConfig captureModeConfig_TA3 = {
        CAP_REGISTER_SELECTION,                      //�����������
        TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE, //�����½��ز���
        TIMER_A_CAPTURE_INPUTSELECT_CCIxA,           //CCIxA:�ⲿ��������  ��CCIxB:���ڲ�ACLK����(�ֲ�)
        TIMER_A_CAPTURE_SYNCHRONOUS,                 //ͬ������
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,     //����CCRN�����ж�
        TIMER_A_OUTPUTMODE_OUTBITVALUE               //���λֵ
    };
    // 5.��ʼ����ʱ���Ĳ���ģʽ
    MAP_Timer_A_initCapture(CAP_TIMA_SELECTION, &captureModeConfig_TA3);

    // 6.ѡ������ģʽ������ʼ����
    MAP_Timer_A_startCounter(CAP_TIMA_SELECTION, TIMER_A_CONTINUOUS_MODE);

    // 7.����жϱ�־λ
    MAP_Timer_A_clearInterruptFlag(CAP_TIMA_SELECTION);                                   //�����ʱ������жϱ�־λ
    MAP_Timer_A_clearCaptureCompareInterrupt(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION); //��� CCR1 �����жϱ�־λ

    // 8.������ʱ���˿��ж�
    MAP_Interrupt_enableInterrupt(INT_TA3_N); //������ʱ��A3�˿��ж�
}


// TIMA3_CAP_STA ����״̬
// [7]:����ߵ�ƽ���״̬
// [6]:0��ʾδ���������أ�1��ʾ�����������
// [5:0]:�������
uint8_t TIMA3_CAP_STA = 0;
uint16_t TIMA3_CAP_VAL = 0;

void TA3_N_IRQHandler(void)
{
    if ((TIMA3_CAP_STA & 0X80) == 0) //��δ�ɹ�����
    {
        if (MAP_Timer_A_getEnabledInterruptStatus(CAP_TIMA_SELECTION)) //����ж�
        {
            MAP_Timer_A_clearInterruptFlag(CAP_TIMA_SELECTION); //�����ʱ������жϱ�־λ

            /* �� �����λCOV �� */
            /* ����UP���ǽ��ˣ������δ����ж�λֵʱ������һ���жϣ�COV����λ����Ҫ�����λ������û�йٷ��⺯����������Բο������ֲ�(slau356h.pdf) P790 */
            BITBAND_PERI(TIMER_A_CMSIS(CAP_TIMA_SELECTION)->CCTL[CAP_CCR_NUM], TIMER_A_CCTLN_COV_OFS) = 0;

            if (TIMA3_CAP_STA & 0X40) //�Ѿ����񵽸ߵ�ƽ�� 40H = 0x01000000
            {
                if ((TIMA3_CAP_STA & 0X3F) == 0X3F) //�ߵ�ƽ̫����
                {
                    TIMA3_CAP_STA |= 0X80; //ǿ�Ʊ�ǳɹ�������ߵ�ƽ 80H = 0x10000000
                    TIMA3_CAP_VAL = 0XFFFF;
                }
                else
                    TIMA3_CAP_STA++; //���������1
            }
        }

        if (MAP_Timer_A_getCaptureCompareEnabledInterruptStatus(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION)) //�����ж�
        {
            MAP_Timer_A_clearCaptureCompareInterrupt(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION); //��� CCR1 �����жϱ�־λ

            //�ж��Ƿ񲶻��½���
            if (TIMA3_CAP_STA & 0X40 && (MAP_Timer_A_getSynchronizedCaptureCompareInput(CAP_TIMA_SELECTION,CAP_REGISTER_SELECTION,TIMER_A_READ_CAPTURE_COMPARE_INPUT) == TIMER_A_CAPTURECOMPARE_INPUT_LOW))
            {
                TIMA3_CAP_STA |= 0X80; //��ǳɹ�������ߵ�ƽ
                TIMA3_CAP_VAL = Timer_A_getCaptureCompareCount(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION);
                
                cap_temp = TIMA3_CAP_STA & 0X3F;
                cap_temp *= 65536;                 //���ʱ���ܺ�
                cap_temp += TIMA3_CAP_VAL;         //�õ��ܵĸߵ�ƽʱ��
                
            }
            else //��δ��ʼ,��һ�β���������
            {
                TIMA3_CAP_STA = 0;
                TIMA3_CAP_VAL = 0;
                MAP_Timer_A_clearTimer(CAP_TIMA_SELECTION); //��ն�ʱ�� ���´�0����
                TIMA3_CAP_STA |= 0X40;                      //��ǲ�����������
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
		/*Trig����*/
    MAP_GPIO_setAsOutputPin(CAP_OUTPORT,CAP_OUTPIN);//Trig����
    
    
    
    GPIO_enableInterrupt(CAP_INPUTPORT,CAP_INPUTPIN);//�ⲿ�жϽ�
    Interrupt_enableInterrupt(CAP_INPUTPORT);
    GPIO_clearInterruptFlag(CAP_INPUTPORT,CAP_INPUTPIN);
    GPIO_interruptEdgeSelect(CAP_INPUTPORT, CAP_INPUTPIN, GPIO_LOW_TO_HIGH_TRANSITION);//�͵��ߵ�ƽ����
    GPIO_setAsInputPinWithPullUpResistor(CAP_INPUTPORT, CAP_INPUTPIN);
    Interrupt_setPriority(CAP_INT_PORT,1<<4);
    		
}




void TimA3_Init(uint16_t ccr0, uint16_t psc)
{
    /*������ģʽ��ʼ��*/
    Timer_A_UpModeConfig upConfig;
    upConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;                                      //ʱ��Դ
    upConfig.clockSourceDivider = psc;                                                     //ʱ�ӷ�Ƶ ��Χ1-64 63
    upConfig.timerPeriod = ccr0;                                                           //�Զ���װ��ֵ��ARR��0xffff
    upConfig.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;                   //���� tim����ж�
    upConfig.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE; //���� ccr0�����ж�
    upConfig.timerClear = TIMER_A_DO_CLEAR;                                                // Clear value

    /*��ʼ����ʱ��A*/
    MAP_Timer_A_configureUpMode(TIMER_A3_BASE, &upConfig);

    /*ѡ��ģʽ��ʼ����*/
    MAP_Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_UP_MODE);

    /*����Ƚ��жϱ�־λ*/
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    /*�������ڶ˿��ж�*/
    MAP_Interrupt_enableInterrupt(INT_TA3_0);
}


//�ⲿ�ж��жϴ�����
void PORT10_IRQHandler(void)
{
    uint16_t flag;

    /*��ȡ�ж�״̬*/
    flag = GPIO_getEnabledInterruptStatus(CAP_INPUTPORT);
    /*����жϱ�־λ*/
    GPIO_clearInterruptFlag(GPIO_PORT_P10, flag);

    //���������Ǹߵ�ƽ
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
				HAL_TIM_Base_Start(&TIM4_Handler);                   // ������ʱ��
				__HAL_TIM_SetCounter(&TIM4_Handler, 0);							 //��ն�ʱ����ֵ
			}
			else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == 0)
			{
				HAL_TIM_Base_Stop(&TIM4_Handler);                   // ֹͣ��ʱ��
				count = __HAL_TIM_GetCounter(&TIM4_Handler);				//��ȡ��ǰ����ֵ
				distance_cm = count * 340/2*0.000001*100;
				printf("distance_cm is %d\r\n", distance_cm);								
				count = 0;
			}
    }
}

/*���ؾ���*/
uint32_t Get_Distance(void)
{
	return distance_cm;
}


#endif


