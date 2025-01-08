/******************************************************************************
 * MSP432 Empty Project
 * 连线说明：P2.6 P2.7分别连同一个电机的编码器A B相
 *					P2.4 P2.5分别连另一个电机编码器的A B相						
 * Author: 星海中的绿洲
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
#include <string.h>//像memset这样的C语言中的函数，就要调用这三个头文件
#include <stdint.h>
#include <stdio.h>
//hardware
#include "control.h"
#include "PID.h"
#include "business_process.h"
#include "beep.h"
#include "sensor.h"
#include "JY901.h"
/*各种标志位*/

volatile uint8_t task_flag=0;   //任务标志位,为0时表示倒车入库的任务，为1时表示侧方停车的任务，为2时任务结束（也可不加）


//这个是用来接收openmv的数据的
#ifndef MAX_RX_BUFFER_SIZE  // 定义接收缓冲区的最大大小
#define MAX_RX_BUFFER_SIZE 256
#endif
volatile extern int my_data;//这个可以不要
extern uint8_t JugleBuffer[15];         //判断缓冲区索引
extern uint8_t UART3_Rx_Buffer[MAX_RX_BUFFER_SIZE]; // 定义接收缓冲区
volatile extern uint8_t state_flag;               // 接收缓冲区中的数据个数

//这个是用来接收陀螺仪JY901的数据的
#ifndef JY_BUFFER_SIZE  // 定义接收缓冲区的最大大小
#define JY_BUFFER_SIZE 220
#endif
extern uint8_t JY901_dealBuffer[22];         //判断缓冲区索引
extern uint8_t JY901_RxBuffer[JY_BUFFER_SIZE]; // 定义接收缓冲区

STATE SYS_state = state1;//系统状态,在headfile.h中看枚举类型

char str[200];
int encoder_L = 0;
int encoder_R = 0;

int dir=0,control=1;
int main(void)
{
  CLOCK_Init();
  uart_init(115200);
  uart2_init(115200);//这个是与openmv通信的串口
	delay_init();	
	OLED_Init();  //OLED显示
  LED_Init();
	PID_param_init();
	KEY_Init();
	EXTI_Init();
  BEEP_Init();
  gray_Init();//初始化灰度传感器
	

	
//	MPU_Init();//用到mpu6050的话，就解除注释
  TimA1_Init(1000 - 1, 48);
	/*定时器中断，用于处理数据，可在中断服务函数里面使用pid来控制*/
  TimA0_Init(19999, 48);//这个就不要改了，因为这个是用来控制类似于之类舵机的
	TimA2_Init(19999, 48);
  MAP_Interrupt_enableMaster();//开启总中断
  NVIC_EnableIRQ(EUSCIA3_IRQn);//开启串口中断
  UART_NVIC_Init();//开启串口中断

  MotorDir(4);//一开始让电机失能

  GPIO_setAsOutputPin(GPIO_PORT_P10, GPIO_PIN0);//这两个是控制电机方向的
  GPIO_setAsOutputPin(GPIO_PORT_P10, GPIO_PIN1);
  GPIO_setAsOutputPin(GPIO_PORT_P10, GPIO_PIN4);
  GPIO_setAsOutputPin(GPIO_PORT_P10, GPIO_PIN5);

	/*设置中断优先级，注意中断优先级必须设置为一样，否则会有bug*/
	Interrupt_setPriority(INT_PORT1,1<<4);
	Interrupt_setPriority(INT_PORT3,1<<4);
	Interrupt_setPriority(INT_PORT6,1<<4);
	Interrupt_setPriority(EUSCIA3_IRQn, 1 << 4);
  Interrupt_setPriority(TA2_0_IRQn, 1 << 4);
  /*清除中断标志位*/
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
    if(state_flag == 1)//这个是为了让主频和串口中断同步，不然主频会处理上次未覆盖的程序，导致数据错误
    {
      FrameJudge(UART3_Rx_Buffer, JugleBuffer);
    }		
    if(JY_flag == 1)//这个是为了让主频和串口中断同步，不然主频会处理上次未覆盖的程序，导致数据错误
    {
      JYJudge(JY901_RxBuffer, JY901_dealBuffer);
    }	
    
    MotorDir(0);  //前进 
    MotorControl(1);
		ServoAngle(1100);
					
  }
}





//1.mpu6050
//要用到mpu6050的话，就解除下面的注释，要用到这几个值
    // float pitch,roll,yaw; 		//欧拉角
    // short aacx,aacy,aacz;		//加速度传感器原始数据
    // short gyrox,gyroy,gyroz;	//陀螺仪原始数据
//如果要用到mpu6050的话，就把下面的代码放到while(1)里面，但还要初始化mpu6050，MPU_Init();
    // if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0)
    // {
    //   MPU_Get_Accelerometer(&aacx, &aacy, &aacz); //得到加速度传感器数据
    //   MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);  //得到陀螺仪数据
    //   printf("pitch:%f  roll:%f  yaw:%f\r\n", pitch, roll, yaw);
    // }
//下面的代码是检验mpu6050是否连接成功，放到while(1)前面
    // while (mpu_dmp_init())//这个while循环是为了检测mpu6050是否连接成功
    // {
    //   printf("MPU6050 ERROR\r\n");
    //   delay_ms(50);
    // }
    // printf("MPU6050 OK");


//2.oled显示
//下面的代码是oled显示的一些例子，放到while(1)里面,使用前要初始化OLED_Init();
//		OLED_ShowNum(13, 5, pitch, 6, 16); 
//		OLED_ShowNum(13, 8, roll, 6, 16); 
//		OLED_ShowNum(53, 6, yaw, 6, 16); 
//    OLED_ShowString(20, 4, (uint8_t *)"2014/05/01", 16);
//    OLED_ShowString(0, 6, (uint8_t *)"ASCII:", 16);
      // sprintf(str,"%f",yaw);//把yaw的值转换成字符串,然后存到str里面
      // OLED_ShowString(8, 2,(uint8_t *)str , 16);


//3.串口发送
    // printf("hello world\r\n");
    // UART_send_string(UART2, "hello"); // 发送字符串
    // send_mpu6050();
//void send_mpu6050(void)
// {
//     char buffer[] = "mpu6050\r\n"; // 定义发送的字符串
//     UART_send_string(UART2, buffer); // 发送字符串
// }
