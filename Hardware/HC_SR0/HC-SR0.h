#ifndef __HC_SR0_H
#define __HC_SR0_H


#define Trig_ON		GPIO_setOutputHighOnPin(CAP_OUTPORT, CAP_OUTPIN)
#define Trig_OFF	GPIO_setOutputLowOnPin(CAP_OUTPORT, CAP_OUTPIN)
#define Trig_TOG	GPIO_toggleOutputOnPin(CAP_OUTPORT, CAP_OUTPIN)
/***************宏定义********************/
#define CAP_TIMA_SELECTION TIMER_A3_BASE                         //在这里改定时器
#define CAP_REGISTER_SELECTION TIMER_A_CAPTURECOMPARE_REGISTER_1 //在这里改定时器通道
#define CAP_CCR_NUM 1                                            //在这里改定时器通道

//单片机输出引脚 模块的输入tring
                //在这里改复用引脚
#define CAP_OUTPORT GPIO_PORT_P10
#define CAP_OUTPIN  GPIO_PIN4

//单片机输入引脚 模块输出echo脚
#define CAP_PORT_PIN GPIO_PORT_P10, GPIO_PIN5     
#define CAP_INPUTPORT GPIO_PORT_P10
#define CAP_INPUTPIN GPIO_PIN5
#define CAP_INT_PORT    INT_PORT10
/*************定义的函数*******************/
void Get_Distance(void);//获取距离
void TimA3_Cap_Init(void);//输入捕获的初始化
void SR0_Send(void);//发送10us高电平
/*****void SR0_Send(void);//发送10us高电平**************变量******************/
extern uint32_t ECHO_Distance;//超声波返回数据
extern uint8_t TIMA3_CAP_STA;//中断溢出次数
extern uint16_t TIMA3_CAP_VAL;//定时器捕获值
extern float distance_cm;//

//#if (ECHO_WAY==ECHO_WAY1 )
//#elif (ECHO_WAY==ECHO_WAY2 )
//#endif

#endif






