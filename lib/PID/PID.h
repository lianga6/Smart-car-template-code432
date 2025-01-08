#ifndef __PID_H
#define __PID_H
#include <driverlib.h>




void twomotor_ctl(float aar1,float aar2);
extern uint16_t CNT_L,CNT_R;//���ڽ��ձ�������������

typedef struct
{
    float target_val;           //Ŀ��ֵ
    float actual_val;        		//ʵ��ֵ
    float err;             			//����ƫ��ֵ
    float err_last;          		//������һ��ƫ��ֵ
    float Kp,Ki,Kd;          		//������������֡�΢��ϵ��
    float integral;          		//�������ֵ
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
  float Last_Err;       // �ϴ����
  float Previous_Err;   // ���ϴ����
  float Output;
  float OutputMax; 
	float Target; 
}PID; 
extern float speed1_Outval,location1_Outval;
extern float speed2_Outval,location2_Outval;


extern long g_motor1_sum,g_motor2_sum;//�������ۼ�ֵ
extern int cnt_temp1;//������������ֵ
extern int cnt_temp2;


extern _pid pid_speed1,pid_speed2,pid_angle;//�ٶȻ�pid�ṹ��
extern _pid pid_location1,pid_location2;//λ�û�pid�ṹ��
extern PID PID_SPEED1,PID_SPEED2;

//������ٱ�
#define REDUCTION_RATIO 21.6

//����������ֱ���
#define ENCODER_RESOLUTION 11

//����4��Ƶ֮��ķֱ���
#define ENCODER_TOTAL_RESOLUTION 	(ENCODER_RESOLUTION*4)

//�ٶȻ��������� ����ʱ�����ж�Ƶ��  �˴�Ϊ��ʱ��7  ��λ����
#define SPEED_PID_PERIOD 20

#define TARGET_SPEED_MAX  600  //// 60rpm����3s����60cm

#define MAX_SPEED_RE 70;

#define Integraldead_zone 100 // �������� �����Լ���������

extern int g_lMotor1PulseSigma;//���25ms���ۼ������ܺ�
extern int g_lMotor2PulseSigma;
extern int g_nMotor1Pulse,g_nMotor2Pulse;//ȫ�ֱ����� ������������ֵ
volatile extern int DISTANCE_L,DISTANCE_R;
extern uint8_t control_swich;

void PID_param_init(void);//PID������ʼ��
float speed_pid_realize(_pid *pid, float actual_val);//�ٶȻ����㷨ʵ��

void Speed_control(void);//��˫�ֽ���PIDʵ�ʿ���

float speed1_pid_control(void);//��ʵ��ֵ����ȫ�ֱ���speed1_Outval  speed2_Outval
float speed2_pid_control(void) ; 

void GET_ENCODER_NUM(void);//�õ���������ֵ
void set_pid_target(_pid *pid, float temp_val);
float get_pid_target(_pid *pid);
void set_p_i_d(_pid *pid, float p, float i, float d);
float location_pid_realize(_pid *pid, float actual_val);
float angle_pid_realize(_pid *pid, float actual_val);
float PID_Postion(PID *pid,int Encoder);
void SET_pid_target(PID *pid, float temp_val);

#endif





