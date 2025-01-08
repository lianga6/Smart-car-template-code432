#include "control.h"
#include "gpio.h"
#include "timer.h"
#include "PID.h"
#include "usart.h"
#include "headfile.h"
#include <math.h>
#include "sensor.h"

int control_dir=0,clear_con=0;

//下面这个函数是用来控制电机的方向的，DIR是方向，2是前进
//因为AT8236有个快衰减和慢衰减的问题，所以尽量不要让电机反转，用2就行
#define LeftDir_forward   GPIO_setOutputHighOnPin(GPIO_PORT_P10, GPIO_PIN1)
#define LeftDir_reverse   GPIO_setOutputLowOnPin(GPIO_PORT_P10, GPIO_PIN1)
#define Left1Dir_forward  GPIO_setOutputHighOnPin(GPIO_PORT_P10, GPIO_PIN0)
#define Left1Dir_reverse  GPIO_setOutputLowOnPin(GPIO_PORT_P10, GPIO_PIN0)
#define RightDir_forward  GPIO_setOutputHighOnPin(GPIO_PORT_P10, GPIO_PIN4)
#define RightDir_reverse  GPIO_setOutputLowOnPin(GPIO_PORT_P10, GPIO_PIN4)
#define Right1Dir_forward GPIO_setOutputHighOnPin(GPIO_PORT_P10, GPIO_PIN5)
#define Right1Dir_reverse GPIO_setOutputLowOnPin(GPIO_PORT_P10, GPIO_PIN5)
void MotorDir(unsigned char DIR){//0是前进，3是后退，4是停止
    if (DIR == 1)
    {		Left1Dir_forward;
        LeftDir_reverse;
        RightDir_forward;
			  Right1Dir_reverse;
				control_dir=0;
    }
    else if (DIR == 3)
    {
        LeftDir_forward;
				Left1Dir_reverse;
        RightDir_forward;
				Right1Dir_reverse;
				control_dir=1;
    }
    else if (DIR == 0)
    {
        LeftDir_reverse;
				Left1Dir_forward;
				Right1Dir_forward;
        RightDir_reverse;
				control_dir=2;
			
    }
    else if (DIR == 2)
    {		Left1Dir_reverse;
        LeftDir_forward;
				Right1Dir_forward;
        RightDir_reverse;
				control_dir=3;
			
    }
		else if (DIR == 4)
    {		Left1Dir_reverse;
        LeftDir_reverse;
				Right1Dir_reverse;
        RightDir_reverse;
				control_dir=4;
			
    }

}

//下面这个函数是为了控制舵机偏转的角度，anglevalue是openmv传来的值，因为舵机的特性，20000是0度，17500是180度,对应的是20ms到17.5ms的脉宽（三极管反转）
void ServoAngle(int anglevalue)//anglevalue为负数，小车向右转，一车为18440，二车为18320
{ 
//  if(anglevalue>190) anglevalue=190;//若有灰度传感器，可以用这个来限制舵机的转动范围
//  else if(anglevalue<-190) anglevalue=-190;
  MAP_Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, 18320+anglevalue);
}

//下面是个停车函数，因为担心停车时会有惯性的作用
void Stop(void)
{
    MotorControl(0);
    MotorDir(4);
}



//下面这个函数是控制电机的，模式的话，0是停止，1是巡线，2是保持一个固定的速度前进
void MotorControl(unsigned char mode)
{    
	
    if (mode == 0)
    {
		set_pid_target(&pid_speed1, 0);
		set_pid_target(&pid_speed2, 0);
    twomotor_ctl(speed1_Outval,speed2_Outval);
    }
		
    else if (mode == 1)
    {
       SET_pid_target(&PID_SPEED1,35+output);//这个是左轮
       SET_pid_target(&PID_SPEED2,35-output);//这个是右轮
       twomotor_ctl(speed1_Outval,speed2_Outval);
    }
    else if (mode == 2)
    { 
      if(control_swich==0)
      {
        set_pid_target(&pid_speed1, 30);//这个是左轮,30对应的compare值为245
        set_pid_target(&pid_speed2, 30);//这个是右轮
        twomotor_ctl(speed1_Outval,speed2_Outval);	
      }						
    }
}


 



