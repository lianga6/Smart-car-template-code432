/******************************************************************************
 * MSP432 Empty Project
 * ����˵����P2.6 P2.7�ֱ���ͬһ������ı�����A B��
 *					P2.4 P2.5�ֱ�����һ�������������A B��						
 * Author: �Ǻ��е�����
*******************************************************************************/
/* DriverLib Includes */
#include <driverlib.h>
#include "delay.h"
#include "clock.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "key.h"
#include "exti.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "headfile.h"
/* Standard Includes */
#include <stdbool.h>
#include "oled.h"
#include <string.h>//��memset������C�����еĺ�������Ҫ����������ͷ�ļ�
#include <stdint.h>
#include <stdio.h>
//hardware
#include "control.h"
#include "PID.h"
#include "business_process.h"
#include "beep.h"
#include "sensor.h"
#include "JY901.h"
/*���ֱ�־λ*/

volatile uint8_t task_flag=0;   //�����־λ,Ϊ0ʱ��ʾ������������Ϊ1ʱ��ʾ�෽ͣ��������Ϊ2ʱ���������Ҳ�ɲ��ӣ�


//�������������openmv�����ݵ�
#ifndef MAX_RX_BUFFER_SIZE  // ������ջ�����������С
#define MAX_RX_BUFFER_SIZE 256
#endif
volatile extern int my_data;//������Բ�Ҫ
extern uint8_t JugleBuffer[15];         //�жϻ���������
extern uint8_t UART3_Rx_Buffer[MAX_RX_BUFFER_SIZE]; // ������ջ�����
volatile extern uint8_t state_flag;               // ���ջ������е����ݸ���

//�������������������JY901�����ݵ�
#ifndef JY_BUFFER_SIZE  // ������ջ�����������С
#define JY_BUFFER_SIZE 220
#endif
extern uint8_t JY901_dealBuffer[22];         //�жϻ���������
extern uint8_t JY901_RxBuffer[JY_BUFFER_SIZE]; // ������ջ�����

STATE SYS_state = state1;//ϵͳ״̬,��headfile.h�п�ö������

char str[200];
int encoder_L = 0;
int encoder_R = 0;

int dir=0,control=1;
int main(void)
{
  CLOCK_Init();
  uart_init(115200);
  uart2_init(115200);//�������openmvͨ�ŵĴ���
	delay_init();	
	OLED_Init();  //OLED��ʾ
  LED_Init();
	PID_param_init();
	KEY_Init();
	EXTI_Init();
  BEEP_Init();
  gray_Init();//��ʼ���Ҷȴ�����
	

	
//	MPU_Init();//�õ�mpu6050�Ļ����ͽ��ע��
  TimA1_Init(1000 - 1, 48);
	/*��ʱ���жϣ����ڴ������ݣ������жϷ���������ʹ��pid������*/
  TimA0_Init(19999, 48);//����Ͳ�Ҫ���ˣ���Ϊ�������������������֮������
	TimA2_Init(19999, 48);
  MAP_Interrupt_enableMaster();//�������ж�
  NVIC_EnableIRQ(EUSCIA3_IRQn);//���������ж�
  UART_NVIC_Init();//���������ж�

  MotorDir(4);//һ��ʼ�õ��ʧ��

  GPIO_setAsOutputPin(GPIO_PORT_P10, GPIO_PIN0);//�������ǿ��Ƶ�������
  GPIO_setAsOutputPin(GPIO_PORT_P10, GPIO_PIN1);
  GPIO_setAsOutputPin(GPIO_PORT_P10, GPIO_PIN4);
  GPIO_setAsOutputPin(GPIO_PORT_P10, GPIO_PIN5);

	/*�����ж����ȼ���ע���ж����ȼ���������Ϊһ�����������bug*/
	Interrupt_setPriority(INT_PORT1,1<<4);
	Interrupt_setPriority(INT_PORT3,1<<4);
	Interrupt_setPriority(INT_PORT6,1<<4);
	Interrupt_setPriority(EUSCIA3_IRQn, 1 << 4);
  Interrupt_setPriority(TA2_0_IRQn, 1 << 4);
  /*����жϱ�־λ*/
  GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN7);
  GPIO_clearInterrupt(GPIO_PORT_P3, GPIO_PIN6);
	GPIO_clearInterrupt(GPIO_PORT_P6, GPIO_PIN2);   
  GPIO_clearInterrupt(GPIO_PORT_P6, GPIO_PIN3);
	GPIO_clearInterrupt(GPIO_PORT_P6, GPIO_PIN0);
  GPIO_clearInterrupt(GPIO_PORT_P6, GPIO_PIN1);
	GPIO_clearInterrupt(GPIO_PORT_P6, GPIO_PIN4);
  GPIO_clearInterrupt(GPIO_PORT_P6, GPIO_PIN5);
  while (1)
  {	
    if(state_flag == 1)//�����Ϊ������Ƶ�ʹ����ж�ͬ������Ȼ��Ƶ�ᴦ���ϴ�δ���ǵĳ��򣬵������ݴ���
    {
      FrameJudge(UART3_Rx_Buffer, JugleBuffer);
    }		
    if(JY_flag == 1)//�����Ϊ������Ƶ�ʹ����ж�ͬ������Ȼ��Ƶ�ᴦ���ϴ�δ���ǵĳ��򣬵������ݴ���
    {
      JYJudge(JY901_RxBuffer, JY901_dealBuffer);
    }	
    
    MotorDir(0);  //ǰ�� 
    MotorControl(1);
		ServoAngle(1100);
					
  }
}





//1.mpu6050
//Ҫ�õ�mpu6050�Ļ����ͽ�������ע�ͣ�Ҫ�õ��⼸��ֵ
    // float pitch,roll,yaw; 		//ŷ����
    // short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
    // short gyrox,gyroy,gyroz;	//������ԭʼ����
//���Ҫ�õ�mpu6050�Ļ����Ͱ�����Ĵ���ŵ�while(1)���棬����Ҫ��ʼ��mpu6050��MPU_Init();
    // if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0)
    // {
    //   MPU_Get_Accelerometer(&aacx, &aacy, &aacz); //�õ����ٶȴ���������
    //   MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);  //�õ�����������
    //   printf("pitch:%f  roll:%f  yaw:%f\r\n", pitch, roll, yaw);
    // }
//����Ĵ����Ǽ���mpu6050�Ƿ����ӳɹ����ŵ�while(1)ǰ��
    // while (mpu_dmp_init())//���whileѭ����Ϊ�˼��mpu6050�Ƿ����ӳɹ�
    // {
    //   printf("MPU6050 ERROR\r\n");
    //   delay_ms(50);
    // }
    // printf("MPU6050 OK");


//2.oled��ʾ
//����Ĵ�����oled��ʾ��һЩ���ӣ��ŵ�while(1)����,ʹ��ǰҪ��ʼ��OLED_Init();
//		OLED_ShowNum(13, 5, pitch, 6, 16); 
//		OLED_ShowNum(13, 8, roll, 6, 16); 
//		OLED_ShowNum(53, 6, yaw, 6, 16); 
//    OLED_ShowString(20, 4, (uint8_t *)"2014/05/01", 16);
//    OLED_ShowString(0, 6, (uint8_t *)"ASCII:", 16);
      // sprintf(str,"%f",yaw);//��yaw��ֵת�����ַ���,Ȼ��浽str����
      // OLED_ShowString(8, 2,(uint8_t *)str , 16);


//3.���ڷ���
    // printf("hello world\r\n");
    // UART_send_string(UART2, "hello"); // �����ַ���
    // send_mpu6050();
//void send_mpu6050(void)
// {
//     char buffer[] = "mpu6050\r\n"; // ���巢�͵��ַ���
//     UART_send_string(UART2, buffer); // �����ַ���
// }
