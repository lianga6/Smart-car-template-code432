#include "usart.h"
#include "PID.h"
#include <math.h>
#include "timer.h"
#include "headfile.h"
#include <stdlib.h>
/*
pid�ĵ��ڣ�д����go_line��spin_turn���棬�������������ã�����pid����
*/


/* ʹ����� */
uint16_t CNT_L=0,CNT_R=0;//���ڽ��ձ�������������
float speed1_Outval,location1_Outval;
float speed2_Outval,location2_Outval;
float cont_val1 = 0.0;
float cont_val2 = 0.0;  
int cnt_temp1=0,cnt_temp2=0,locxzz=0,locxzz1=0;//�洢���Ǳ���������������������ٶȻ�
volatile int DISTANCE_L=0,DISTANCE_R=0;
uint8_t control_swich=0;//����ǿ��ƿ��أ�0���ٶȻ���1��λ�û�
int g_lMotor1PulseSigma=0;//���25ms���ۼ������ܺ�
int g_lMotor2PulseSigma=0;
int g_nMotor1Pulse=0,g_nMotor2Pulse=0;//ȫ�ֱ����� ������������ֵ


_pid pid_speed1,pid_speed2,pid_angle;//����һ���ṹ���������ߵ����ݽṹ��pid������Ҫ������  speed���ٶȻ�ר��
_pid pid_location1,pid_location2;//����һ���ṹ���������ߵ����ݽṹ��pid������Ҫ������  location���ٶȻ�ר��
PID PID_SPEED1,PID_SPEED2;

void GET_ENCODER_NUM(void)	
{
	cnt_temp1 = encoder_L;//��ȡ��������������ֵ
    cnt_temp2 = encoder_R; 
    
	encoder_L=0;//��ȡ֮�����㣬�Ժ����ڶ�ʱ���ж�������
	encoder_R=0;

	g_lMotor1PulseSigma+=cnt_temp1;//�ۼ�ת�򻷵�ֵ
	g_lMotor2PulseSigma+=cnt_temp2;//�ۼ�ת�򻷵�ֵ
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
  * @brief  PID������ʼ��
	*	@note 	��
  * @retval ��
  */
void PID_param_init()
{
  
  	/* �ٶ���س�ʼ������ */
    pid_speed1.target_val=0.0;				
    pid_speed1.actual_val=0.0;
    pid_speed1.err=0.0;
    pid_speed1.err_last=0.0;
    pid_speed1.integral=0.0;
  
    pid_speed1.Kp = 1.9;//0.22
    pid_speed1.Ki =0.003;// 0.0003;
    pid_speed1.Kd = 0.6;//0.0003;
		
  
  	/* �ٶ���س�ʼ������ */
    pid_speed2.target_val=0.0;				
    pid_speed2.actual_val=0.0;
    pid_speed2.err=0.0;
    pid_speed2.err_last=0.0;
    pid_speed2.integral=0.0;
  
    pid_speed2.Kp = 1.9;
    pid_speed2.Ki = 0.003;
    pid_speed2.Kd = 0.6;
    
    
  	/* λ����س�ʼ������ */
    pid_location1.target_val=0.0;				
    pid_location1.actual_val=0.0;
    pid_location1.err=0.0;
    pid_location1.err_last=0.0;
    pid_location1.integral=0.0;
                
    pid_location1.Kp = 1;
    pid_location1.Ki = 0.1;
    pid_location1.Kd = 0.5;
  
  	/* λ����س�ʼ������ */
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
		PID_SPEED1.Last_Err=0;       // �ϴ����
		PID_SPEED1.Previous_Err=0;   // ���ϴ����
		PID_SPEED1.Output=0;
		PID_SPEED1.OutputMax=200;      // ����ʽʽPID����޷�
		PID_SPEED1.Target=0;
		
		PID_SPEED2.Kp=11.0;//11
		PID_SPEED2.Ki=2;
		PID_SPEED2.Kd=10;
		PID_SPEED2.p_out=0;
		PID_SPEED2.i_out=0;
		PID_SPEED2.d_out=0;
		PID_SPEED2.Err=0;
		PID_SPEED2.Last_Err=0;       // �ϴ����
		PID_SPEED2.Previous_Err=0;   // ���ϴ����
		PID_SPEED2.Output=0;
		PID_SPEED2.OutputMax=200;      // ����ʽʽPID����޷�
		PID_SPEED2.Target=0;
	}

/**����Ŀ��ֵ
  * @brief  ����Ŀ��ֵ
  * @param  val		Ŀ��ֵ
	*	@note 	��
  * @retval ��
  */
void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_val = temp_val;    // ���õ�ǰ��Ŀ��ֵ
}


void SET_pid_target(PID *pid, float temp_val)//����ʽpid
{
  pid->Target = temp_val;    // ���õ�ǰ��Ŀ��ֵ
}

/**��ȡĿ��ֵ
  * @brief  ��ȡĿ��ֵ
  * @param  ��
	*	@note 	��
  * @retval Ŀ��ֵ
  */
float get_pid_target(_pid *pid)
{
  return pid->target_val;    // ���õ�ǰ��Ŀ��ֵ
}

float speed1_pid_control(void)//�ٶȻ�����  
{			float actual_speed;
	//	  actual_speed = ((float)cnt_temp1*1000.0*60.0)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*SPEED_PID_PERIOD);//�궨��ĳ˻���������㲶���Ȧ��  *1000��Ϊ�� *60��Ϊ���ӣ���λ��rmp ÿ���Ӷ���ת
			actual_speed=cnt_temp1;
   if(control_swich==0)
		 {
                       // ��ǰ����ֵ
			cont_val1 += PID_Postion(&PID_SPEED1,fabs(actual_speed));    // ���� PID ���㣬actual_speedʵ��ֵ
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
			cont_val1 += speed_pid_realize(&pid_speed1,fabs(actual_speed));    // ���� PID ���㣬actual_speedʵ��ֵ
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
                     // ��ǰ����ֵ		
			cont_val2 += PID_Postion(&PID_SPEED2,fabs(actual_speed));    // ���� PID ����
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
			cont_val2 += speed_pid_realize(&pid_speed2,fabs(actual_speed));    // ���� PID ����
			if(cont_val2>=900)cont_val2=900;
			if(cont_val2<=0)cont_val2=0;
			return cont_val2;
	 }
	 return 0;
}

//PID�ٶȻ���λ�û���������
void Speed_control(void) //�����������ת��Ϊ��λ��
{   
    speed1_Outval = speed1_pid_control();    //Ҫ�ǵ��ת�򲻷���Ԥ�ڣ�������������ȡ����ֵ
    speed2_Outval = speed2_pid_control();  
}


void set_p_i_d(_pid *pid, float p, float i, float d)
{
  	pid->Kp = p;    // ���ñ���ϵ�� P
		pid->Ki = i;    // ���û���ϵ�� I
		pid->Kd = d;    // ����΢��ϵ�� D
}


/**
  * @brief  �ٶ�PID�㷨ʵ��
  * @param  actual_val:ʵ��ֵ
	*	@note 	��
  * @retval ͨ��PID���������
  */

float PID_Postion(PID *pid,int Encoder)//����ʽPID
{
float pwm = 0;
//    pid->ek = pid->Target - Encoder; // ���㵱ǰ���
//    pid->ek_sum += pid->ek;      //���ƫ��Ļ���
//    pwm = pid->kp*pid->ek + pid->ki*pid->ek_sum + 
//    pid->kd*(pid->ek - pid->ek_1);   //λ��ʽPID������
//    pid->ek_1 = pid->ek;   //������һ��ƫ�� 
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


float speed_pid_realize(_pid *pid, float actual_val)//��һ��������PID�Ĳ��������ݣ��ڶ�����ʵ��ֵ�˴�ʵ��ֵ�ĵ�λ��rmp ת/����
{
	
		/*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err=pid->target_val-actual_val;

    if((pid->err<0.5f ) && (pid->err>-0.5f))  //��1��ô�����������1���ӣ�λ�ò�Ϊ1�����ӵ��ܳ� 
		{
      		pid->err = 0.0f;
		}
        
    pid->integral += pid->err;    // ����ۻ�
	
	  /*�����޷�*/
	   	 if (pid->integral >= 1000) {pid->integral =1000;}
      else if (pid->integral < -1000)  {pid->integral = -1000;}
		/*PID�㷨ʵ��*///�˴���actual_valָ�������ֵ
    pid->actual_val = pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
		/*����*/
    pid->err_last=pid->err;
    
		/*���ص�ǰʵ��ֵ*/
    return pid->actual_val;
}


float location_pid_realize(_pid *pid, float actual_val)  //λ�û����Kp����Ҳ����
{
		/*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err=pid->target_val-actual_val;
  
//    /* �趨�ջ����� */   //�⻷�������Բ�Ҫ 
//    if((pid->err >= -0.1) && (pid->err <= 0.1)) 
//    {
//      pid->err = 0;
//      pid->integral = 0;
//    }
    
    pid->integral += pid->err;    // ����ۻ�
	
	 if((pid->err<5 ) && (pid->err>-5))   //��������������ٶ�ƫ������1���ӣ���������ƫ��Ϊ��Ȧ
		{
      pid->err = 0.0;
		}
	
	if (pid->integral >= 50) {pid->integral =100;}
      else if (pid->integral < -50)  {pid->integral = -100;}

		/*PID�㷨ʵ��*/
//    pid->actual_val = pid->Kp*pid->err
//		                  +pid->Ki*pid->integral
//		                  +pid->Kd*(pid->err-pid->err_last);
	  pid->actual_val = (pid->err/pid->target_val)*pid->Kp*MAX_SPEED_RE+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
  
		/*����*/
    pid->err_last=pid->err;
    
		/*���ص�ǰʵ��ֵ*/
    return pid->actual_val;
}



float angle_pid_realize(_pid *pid, float actual_val)
{
		/*����Ŀ��ֵ��ʵ��ֵ�����*/
	if(actual_val==0)return 0;
    pid->err=pid->target_val-actual_val;

    if((pid->err<0.5f ) && (pid->err>-0.5f))   //��������������ٶ�ƫ������1���ӣ���������ƫ��Ϊ��Ȧ
		{
      		pid->err = 0.0f;
		}
	//����õ���PDģ�ͣ�����Ҫ����	
     pid->integral += pid->err;    // ����ۻ�
	  /*�����޷�*/
	   	 if (pid->integral >= 200) {pid->integral =200;}
       else if (pid->integral < -200)  {pid->integral = -200;}

		/*PID�㷨ʵ��*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                   +pid->Kd*(pid->err-pid->err_last);
  
		/*����*/
    pid->err_last=pid->err;
    
		/*���ص�ǰʵ��ֵ*/
    return pid->actual_val;
}





