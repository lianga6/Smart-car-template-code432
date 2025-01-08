#ifndef __HC_SR0_H
#define __HC_SR0_H


#define Trig_ON		GPIO_setOutputHighOnPin(CAP_OUTPORT, CAP_OUTPIN)
#define Trig_OFF	GPIO_setOutputLowOnPin(CAP_OUTPORT, CAP_OUTPIN)
#define Trig_TOG	GPIO_toggleOutputOnPin(CAP_OUTPORT, CAP_OUTPIN)
/***************�궨��********************/
#define CAP_TIMA_SELECTION TIMER_A3_BASE                         //������Ķ�ʱ��
#define CAP_REGISTER_SELECTION TIMER_A_CAPTURECOMPARE_REGISTER_1 //������Ķ�ʱ��ͨ��
#define CAP_CCR_NUM 1                                            //������Ķ�ʱ��ͨ��

//��Ƭ��������� ģ�������tring
                //������ĸ�������
#define CAP_OUTPORT GPIO_PORT_P10
#define CAP_OUTPIN  GPIO_PIN4

//��Ƭ���������� ģ�����echo��
#define CAP_PORT_PIN GPIO_PORT_P10, GPIO_PIN5     
#define CAP_INPUTPORT GPIO_PORT_P10
#define CAP_INPUTPIN GPIO_PIN5
#define CAP_INT_PORT    INT_PORT10
/*************����ĺ���*******************/
void Get_Distance(void);//��ȡ����
void TimA3_Cap_Init(void);//���벶��ĳ�ʼ��
void SR0_Send(void);//����10us�ߵ�ƽ
/*****void SR0_Send(void);//����10us�ߵ�ƽ**************����******************/
extern uint32_t ECHO_Distance;//��������������
extern uint8_t TIMA3_CAP_STA;//�ж��������
extern uint16_t TIMA3_CAP_VAL;//��ʱ������ֵ
extern float distance_cm;//

//#if (ECHO_WAY==ECHO_WAY1 )
//#elif (ECHO_WAY==ECHO_WAY2 )
//#endif

#endif






