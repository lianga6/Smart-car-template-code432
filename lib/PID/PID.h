#ifndef __PID_H
#define __PID_H
#include <driverlib.h>




void twomotor_ctl(float aar1,float aar2);
extern uint16_t CNT_L,CNT_R;//用于接收编码器的脉冲数

typedef struct
{
    float target_val;           //目标值
    float actual_val;        		//实际值
    float err;             			//定义偏差值
    float err_last;          		//定义上一个偏差值
    float Kp,Ki,Kd;          		//定义比例、积分、微分系数
    float integral;          		//定义积分值
}_pid;



typedef struct 
{
float Kp;
  float Ki;
  float Kd;
  float p_out;
  float i_out;
  float d_out;
  float Err;
  float Last_Err;       // 上次误差
  float Previous_Err;   // 上上次误差
  float Output;
  float OutputMax; 
	float Target; 
}PID; 
extern float speed1_Outval,location1_Outval;
extern float speed2_Outval,location2_Outval;


extern long g_motor1_sum,g_motor2_sum;//编码器累计值
extern int cnt_temp1;//编码器读到的值
extern int cnt_temp2;


extern _pid pid_speed1,pid_speed2,pid_angle;//速度环pid结构体
extern _pid pid_location1,pid_location2;//位置环pid结构体
extern PID PID_SPEED1,PID_SPEED2;

//电机减速比
#define REDUCTION_RATIO 21.6

//编码器物理分辨率
#define ENCODER_RESOLUTION 11

//经过4倍频之后的分辨率
#define ENCODER_TOTAL_RESOLUTION 	(ENCODER_RESOLUTION*4)

//速度环控制周期 看定时器的中断频率  此处为定时器7  单位毫秒
#define SPEED_PID_PERIOD 20

#define TARGET_SPEED_MAX  600  //// 60rpm可以3s走完60cm

#define MAX_SPEED_RE 70;

#define Integraldead_zone 100 // 积分死区 根据自己的需求定义

extern int g_lMotor1PulseSigma;//电机25ms内累计脉冲总和
extern int g_lMotor2PulseSigma;
extern int g_nMotor1Pulse,g_nMotor2Pulse;//全局变量， 保存电机脉冲数值
volatile extern int DISTANCE_L,DISTANCE_R;
extern uint8_t control_swich;

void PID_param_init(void);//PID参数初始化
float speed_pid_realize(_pid *pid, float actual_val);//速度环的算法实现

void Speed_control(void);//对双轮进行PID实际控制

float speed1_pid_control(void);//将实际值赋给全局变量speed1_Outval  speed2_Outval
float speed2_pid_control(void) ; 

void GET_ENCODER_NUM(void);//得到编码器的值
void set_pid_target(_pid *pid, float temp_val);
float get_pid_target(_pid *pid);
void set_p_i_d(_pid *pid, float p, float i, float d);
float location_pid_realize(_pid *pid, float actual_val);
float angle_pid_realize(_pid *pid, float actual_val);
float PID_Postion(PID *pid,int Encoder);
void SET_pid_target(PID *pid, float temp_val);

#endif





