#include "usart.h"
#include "PID.h"
#include <math.h>
#include "timer.h"
#include "headfile.h"
#include <stdlib.h>
/*
pid的调节，写在了go_line和spin_turn里面，其他情况不会调用，进行pid控制
*/


/* 使能输出 */
uint16_t CNT_L=0,CNT_R=0;//用于接收编码器的脉冲数
float speed1_Outval,location1_Outval;
float speed2_Outval,location2_Outval;
float cont_val1 = 0.0;
float cont_val2 = 0.0;  
int cnt_temp1=0,cnt_temp2=0,locxzz=0,locxzz1=0;//存储的是编码器里的脉冲数，用于速度环
volatile int DISTANCE_L=0,DISTANCE_R=0;
uint8_t control_swich=0;//这个是控制开关，0是速度环，1是位置环
int g_lMotor1PulseSigma=0;//电机25ms内累计脉冲总和
int g_lMotor2PulseSigma=0;
int g_nMotor1Pulse=0,g_nMotor2Pulse=0;//全局变量， 保存电机脉冲数值


_pid pid_speed1,pid_speed2,pid_angle;//这是一个结构体变量，里边的数据结构是pid控制需要的数据  speed是速度环专用
_pid pid_location1,pid_location2;//这是一个结构体变量，里边的数据结构是pid控制需要的数据  location是速度环专用
PID PID_SPEED1,PID_SPEED2;

void GET_ENCODER_NUM(void)	
{
	cnt_temp1 = encoder_L;//获取编码器计数器的值
    cnt_temp2 = encoder_R; 
    
	encoder_L=0;//读取之后清零，以后会放在定时器中断里面用
	encoder_R=0;

	g_lMotor1PulseSigma+=cnt_temp1;//累计转向环的值
	g_lMotor2PulseSigma+=cnt_temp2;//累计转向环的值
//	  g_lMotor1PulseSigma=fabs((float)g_lMotor1PulseSigma);
//		g_lMotor2PulseSigma=fabs((float)g_lMotor2PulseSigma);
	g_nMotor1Pulse=g_lMotor1PulseSigma;
	g_nMotor2Pulse=g_lMotor2PulseSigma;
	if(g_lMotor1PulseSigma<0)g_nMotor1Pulse=0-g_lMotor1PulseSigma;
	if(g_lMotor2PulseSigma<0)g_nMotor2Pulse=0-g_lMotor2PulseSigma;
	// printf("%d,%d\r\n",g_nMotor1Pulse,g_nMotor2Pulse);
		
}

void twomotor_ctl(float aar1,float aar2)
{
    Timer_A_setCompareValue(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2,aar1);
    Timer_A_setCompareValue(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1,aar2);
}

/**
  * @brief  PID参数初始化
	*	@note 	无
  * @retval 无
  */
void PID_param_init()
{
  
  	/* 速度相关初始化参数 */
    pid_speed1.target_val=0.0;				
    pid_speed1.actual_val=0.0;
    pid_speed1.err=0.0;
    pid_speed1.err_last=0.0;
    pid_speed1.integral=0.0;
  
    pid_speed1.Kp = 1.9;//0.22
    pid_speed1.Ki =0.003;// 0.0003;
    pid_speed1.Kd = 0.6;//0.0003;
		
  
  	/* 速度相关初始化参数 */
    pid_speed2.target_val=0.0;				
    pid_speed2.actual_val=0.0;
    pid_speed2.err=0.0;
    pid_speed2.err_last=0.0;
    pid_speed2.integral=0.0;
  
    pid_speed2.Kp = 1.9;
    pid_speed2.Ki = 0.003;
    pid_speed2.Kd = 0.6;
    
    
  	/* 位置相关初始化参数 */
    pid_location1.target_val=0.0;				
    pid_location1.actual_val=0.0;
    pid_location1.err=0.0;
    pid_location1.err_last=0.0;
    pid_location1.integral=0.0;
                
    pid_location1.Kp = 1;
    pid_location1.Ki = 0.1;
    pid_location1.Kd = 0.5;
  
  	/* 位置相关初始化参数 */
    pid_location2.target_val=0.0;				
    pid_location2.actual_val=0.0;
    pid_location2.err=0.0;
    pid_location2.err_last=0.0;
    pid_location2.integral=0.0;
  
    pid_location2.Kp = 1;
    pid_location2.Ki = 0.1;
    pid_location2.Kd = 0.5;
		
	  pid_angle.target_val=0.0;				
    pid_angle.actual_val=0.0;
    pid_angle.err=0.0;
    pid_angle.err_last=0.0;
    pid_angle.integral=0.0;
  
    pid_angle.Kp = 1.1;
		pid_angle.Ki =0.02441;
    pid_angle.Kd = 6.9;
		
//		PID_SPEED1.kp = 3.7;
//    PID_SPEED1.ki = 0.003;
//    PID_SPEED1.kd = 0.003;
//    PID_SPEED1.limit = 600;
//    PID_SPEED1.ek = 0;
//    PID_SPEED1.ek_1 = 0;
//    PID_SPEED1.ek_sum = 0;
//		PID_SPEED1.Target=0;
		
		PID_SPEED1.Kp=11.0;//11
		PID_SPEED1.Ki=2;
		PID_SPEED1.Kd=10;
		PID_SPEED1.p_out=0;
		PID_SPEED1.i_out=0;
		PID_SPEED1.d_out=0;
		PID_SPEED1.Err=0;
		PID_SPEED1.Last_Err=0;       // 上次误差
		PID_SPEED1.Previous_Err=0;   // 上上次误差
		PID_SPEED1.Output=0;
		PID_SPEED1.OutputMax=200;      // 增量式式PID输出限幅
		PID_SPEED1.Target=0;
		
		PID_SPEED2.Kp=11.0;//11
		PID_SPEED2.Ki=2;
		PID_SPEED2.Kd=10;
		PID_SPEED2.p_out=0;
		PID_SPEED2.i_out=0;
		PID_SPEED2.d_out=0;
		PID_SPEED2.Err=0;
		PID_SPEED2.Last_Err=0;       // 上次误差
		PID_SPEED2.Previous_Err=0;   // 上上次误差
		PID_SPEED2.Output=0;
		PID_SPEED2.OutputMax=200;      // 增量式式PID输出限幅
		PID_SPEED2.Target=0;
	}

/**设置目标值
  * @brief  设置目标值
  * @param  val		目标值
	*	@note 	无
  * @retval 无
  */
void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_val = temp_val;    // 设置当前的目标值
}


void SET_pid_target(PID *pid, float temp_val)//增量式pid
{
  pid->Target = temp_val;    // 设置当前的目标值
}

/**获取目标值
  * @brief  获取目标值
  * @param  无
	*	@note 	无
  * @retval 目标值
  */
float get_pid_target(_pid *pid)
{
  return pid->target_val;    // 设置当前的目标值
}

float speed1_pid_control(void)//速度环控制  
{			float actual_speed;
	//	  actual_speed = ((float)cnt_temp1*1000.0*60.0)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*SPEED_PID_PERIOD);//宏定义的乘积算出的是你捕获的圈数  *1000变为秒 *60变为分钟，则单位是rmp 每分钟多少转
			actual_speed=cnt_temp1;
   if(control_swich==0)
		 {
                       // 当前控制值
			cont_val1 += PID_Postion(&PID_SPEED1,fabs(actual_speed));    // 进行 PID 计算，actual_speed实际值
			if(cont_val1>=900)cont_val1=900;
			if(cont_val1<=0)cont_val1=0;
		//		printf("1%f,%f\r\n",cont_val,actual_speed);
			return cont_val1; 
		 }
		if(control_swich==1)
			{
			set_pid_target(&pid_location1,DISTANCE_L);
			locxzz =location_pid_realize(&pid_location1 ,abs(g_lMotor1PulseSigma));
			if(locxzz>=60)locxzz =60;
			set_pid_target(&pid_speed1 ,locxzz);
			cont_val1 += speed_pid_realize(&pid_speed1,fabs(actual_speed));    // 进行 PID 计算，actual_speed实际值
			if(cont_val1>=900)cont_val1=900;
			if(cont_val1<=0)cont_val1=0;
			return cont_val1; 
			}
			return 0;
}

float speed2_pid_control(void)  
{     float actual_speed;
		actual_speed=cnt_temp2;
   if(control_swich==0)
	 {
                     // 当前控制值		
			cont_val2 += PID_Postion(&PID_SPEED2,fabs(actual_speed));    // 进行 PID 计算
			if(cont_val2>=900)cont_val2=900;
			if(cont_val2<=0)cont_val2=0;
	//		printf("2%f,%f\r\n",cont_val,actual_speed);
			return cont_val2;
	 }
	 if(control_swich==1)
	 {
			set_pid_target(&pid_location2,DISTANCE_R);
			locxzz1 =location_pid_realize(&pid_location2 ,abs(g_lMotor2PulseSigma));
			if(locxzz1>=60)locxzz1 =60;
	    	set_pid_target(&pid_speed2 ,locxzz1);
			cont_val2 += speed_pid_realize(&pid_speed2,fabs(actual_speed));    // 进行 PID 计算
			if(cont_val2>=900)cont_val2=900;
			if(cont_val2<=0)cont_val2=0;
			return cont_val2;
	 }
	 return 0;
}

//PID速度环和位置环串级调速
void Speed_control(void) //这个控制是以转速为单位的
{   
    speed1_Outval = speed1_pid_control();    //要是电机转向不符合预期，就在这两句里取反数值
    speed2_Outval = speed2_pid_control();  
}


void set_p_i_d(_pid *pid, float p, float i, float d)
{
  	pid->Kp = p;    // 设置比例系数 P
		pid->Ki = i;    // 设置积分系数 I
		pid->Kd = d;    // 设置微分系数 D
}


/**
  * @brief  速度PID算法实现
  * @param  actual_val:实际值
	*	@note 	无
  * @retval 通过PID计算后的输出
  */

float PID_Postion(PID *pid,int Encoder)//增量式PID
{
float pwm = 0;
//    pid->ek = pid->Target - Encoder; // 计算当前误差
//    pid->ek_sum += pid->ek;      //求出偏差的积分
//    pwm = pid->kp*pid->ek + pid->ki*pid->ek_sum + 
//    pid->kd*(pid->ek - pid->ek_1);   //位置式PID控制器
//    pid->ek_1 = pid->ek;   //保存上一次偏差 
//if(pwm > pid->limit)
//    {
//      pwm =  pid->limit;
//    }
//else if(pwm < -pid->limit)
//    {
//      pwm =  -pid->limit;
//    }
//		
//		if(pid == NULL)
//    return;   
  pid->Err = pid->Target - Encoder; 
  pid->p_out = pid->Kp * (pid->Err - pid->Last_Err);
  pid->i_out = pid->Ki * pid->Err;
  pid->d_out = pid->Kd * (pid->Err - 2.0f*pid->Last_Err + pid->Previous_Err);  
  pid->Output += pid->p_out + pid->i_out + pid->d_out;   
	if(pid->Output>pid->OutputMax)pid->Output=pid->OutputMax;
	if(pid->Output<-pid->OutputMax)pid->Output=-pid->OutputMax;
  pid->Previous_Err = pid->Last_Err;
  pid->Last_Err = pid->Err;
	pwm=pid->Output;		
		
  return pwm;
}


float speed_pid_realize(_pid *pid, float actual_val)//第一个参数是PID的参数和数据，第二个是实际值此处实际值的单位是rmp 转/分钟
{
	
		/*计算目标值与实际值的误差*/
    pid->err=pid->target_val-actual_val;

    if((pid->err<0.5f ) && (pid->err>-0.5f))  //差1这么多可以吗？运行1分钟，位置差为1个轮子的周长 
		{
      		pid->err = 0.0f;
		}
        
    pid->integral += pid->err;    // 误差累积
	
	  /*积分限幅*/
	   	 if (pid->integral >= 1000) {pid->integral =1000;}
      else if (pid->integral < -1000)  {pid->integral = -1000;}
		/*PID算法实现*///此处的actual_val指的是输出值
    pid->actual_val = pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
		/*误差传递*/
    pid->err_last=pid->err;
    
		/*返回当前实际值*/
    return pid->actual_val;
}


float location_pid_realize(_pid *pid, float actual_val)  //位置环光个Kp好像也可以
{
		/*计算目标值与实际值的误差*/
    pid->err=pid->target_val-actual_val;
  
//    /* 设定闭环死区 */   //外环死区可以不要 
//    if((pid->err >= -0.1) && (pid->err <= 0.1)) 
//    {
//      pid->err = 0;
//      pid->integral = 0;
//    }
    
    pid->integral += pid->err;    // 误差累积
	
	 if((pid->err<5 ) && (pid->err>-5))   //假如以最大允许速度偏差运行1分钟，输出轴最大偏差为半圈
		{
      pid->err = 0.0;
		}
	
	if (pid->integral >= 50) {pid->integral =100;}
      else if (pid->integral < -50)  {pid->integral = -100;}

		/*PID算法实现*/
//    pid->actual_val = pid->Kp*pid->err
//		                  +pid->Ki*pid->integral
//		                  +pid->Kd*(pid->err-pid->err_last);
	  pid->actual_val = (pid->err/pid->target_val)*pid->Kp*MAX_SPEED_RE+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
  
		/*误差传递*/
    pid->err_last=pid->err;
    
		/*返回当前实际值*/
    return pid->actual_val;
}



float angle_pid_realize(_pid *pid, float actual_val)
{
		/*计算目标值与实际值的误差*/
	if(actual_val==0)return 0;
    pid->err=pid->target_val-actual_val;

    if((pid->err<0.5f ) && (pid->err>-0.5f))   //假如以最大允许速度偏差运行1分钟，输出轴最大偏差为半圈
		{
      		pid->err = 0.0f;
		}
	//舵机用的是PD模型，不需要积分	
     pid->integral += pid->err;    // 误差累积
	  /*积分限幅*/
	   	 if (pid->integral >= 200) {pid->integral =200;}
       else if (pid->integral < -200)  {pid->integral = -200;}

		/*PID算法实现*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                   +pid->Kd*(pid->err-pid->err_last);
  
		/*误差传递*/
    pid->err_last=pid->err;
    
		/*返回当前实际值*/
    return pid->actual_val;
}





