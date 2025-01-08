#include "control.h"
#include "gpio.h"
#include "timer.h"
#include "PID.h"
#include "usart.h"
#include "headfile.h"
#include <math.h>
#include "sensor.h"

int control_dir=0,clear_con=0;

//��������������������Ƶ���ķ���ģ�DIR�Ƿ���2��ǰ��
//��ΪAT8236�и���˥������˥�������⣬���Ծ�����Ҫ�õ����ת����2����
#define LeftDir_forward   GPIO_setOutputHighOnPin(GPIO_PORT_P10, GPIO_PIN1)
#define LeftDir_reverse   GPIO_setOutputLowOnPin(GPIO_PORT_P10, GPIO_PIN1)
#define Left1Dir_forward  GPIO_setOutputHighOnPin(GPIO_PORT_P10, GPIO_PIN0)
#define Left1Dir_reverse  GPIO_setOutputLowOnPin(GPIO_PORT_P10, GPIO_PIN0)
#define RightDir_forward  GPIO_setOutputHighOnPin(GPIO_PORT_P10, GPIO_PIN4)
#define RightDir_reverse  GPIO_setOutputLowOnPin(GPIO_PORT_P10, GPIO_PIN4)
#define Right1Dir_forward GPIO_setOutputHighOnPin(GPIO_PORT_P10, GPIO_PIN5)
#define Right1Dir_reverse GPIO_setOutputLowOnPin(GPIO_PORT_P10, GPIO_PIN5)
void MotorDir(unsigned char DIR){//0��ǰ����3�Ǻ��ˣ�4��ֹͣ
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

//�������������Ϊ�˿��ƶ��ƫת�ĽǶȣ�anglevalue��openmv������ֵ����Ϊ��������ԣ�20000��0�ȣ�17500��180��,��Ӧ����20ms��17.5ms�����������ܷ�ת��
void ServoAngle(int anglevalue)//anglevalueΪ������С������ת��һ��Ϊ18440������Ϊ18320
{ 
//  if(anglevalue>190) anglevalue=190;//���лҶȴ���������������������ƶ����ת����Χ
//  else if(anglevalue<-190) anglevalue=-190;
  MAP_Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, 18320+anglevalue);
}

//�����Ǹ�ͣ����������Ϊ����ͣ��ʱ���й��Ե�����
void Stop(void)
{
    MotorControl(0);
    MotorDir(4);
}



//������������ǿ��Ƶ���ģ�ģʽ�Ļ���0��ֹͣ��1��Ѳ�ߣ�2�Ǳ���һ���̶����ٶ�ǰ��
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
       SET_pid_target(&PID_SPEED1,35+output);//���������
       SET_pid_target(&PID_SPEED2,35-output);//���������
       twomotor_ctl(speed1_Outval,speed2_Outval);
    }
    else if (mode == 2)
    { 
      if(control_swich==0)
      {
        set_pid_target(&pid_speed1, 30);//���������,30��Ӧ��compareֵΪ245
        set_pid_target(&pid_speed2, 30);//���������
        twomotor_ctl(speed1_Outval,speed2_Outval);	
      }						
    }
}


 



