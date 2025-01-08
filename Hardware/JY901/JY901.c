#include "JY901.h"
#include "usart.h"
#include "headfile.h"
#include <string.h>//像memset这样的C语言中的函数，就要调用这三个头文件
#include <stdint.h>
#include <stdio.h>
//这是给陀螺仪JY901用的串口2，陀螺仪发来的每帧数据都有十一个
#define JY_BUFFER_SIZE 220
uint8_t JY901_RxBuffer[JY_BUFFER_SIZE];
uint8_t JY901_dealBuffer[22];
volatile uint8_t JY_flag=0;            //因为主频执行速度太快，而串口中断相对较慢，所以需要一个标志位来判断是否执行
volatile uint8_t JY_Rx_Index = 0; // 定义接收缓冲区的索引

#define  JYHEAD1  0x55    //帧头1
#define  JYHEAD2  0x53   //帧头2

/*数据帧状态机*/
typedef enum {
    jy_HEAD1 = 0, //帧头1
    jy_HEAD2,     //帧头2
    jy_DATA1,     //数据1
    jy_DATA2,     //数据2
    jy_DATA3,     //数据3
    jy_DATA4,     //数据4
    jy_DATA5,     //数据5
    jy_DATA6,     //数据6
    jy_DATA7,     //数据7
    jy_DATA8,     //数据8
    jy_DATA9     //数据9
} JY_STATE;
JY_STATE jy_State = jy_HEAD1; //初始化状态机


void EUSCIA0_IRQHandler(void)
{
	uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        uint8_t received_data = MAP_UART_receiveData(EUSCI_A0_BASE); 
        if(received_data == 0xFF) 
        {
            return;
        }
        if((JY_Rx_Index==0)&&(MAP_UART_receiveData(EUSCI_A0_BASE) != JYHEAD1))//如果接收到的数据不是0xFF，就跳过，因为每次都会发0xFF
        {
            return;
        }
        JY901_RxBuffer[JY_Rx_Index++] = received_data; 
        if (JY_Rx_Index >= JY_BUFFER_SIZE) 
        {
            JY_Rx_Index = 0;
        }
        JY_flag=1; 
    }

}


void JYJudge(uint8_t *Recv_Data,uint8_t *Sendbuffer)
{
    volatile static uint8_t HeadIndex1 = 0;           //帧头索引
    volatile static uint8_t BufferIndex=0;         //判断缓冲区索引
    //如果要想改成>=，就要把下面这个if放到整个函数的最后面
    //if((BufferIndex > JY_BUFFER_SIZE)||(HeadIndex>=JY_BUFFER_SIZE-3))//Recv_Data这个数组在主函数里要对应UART2_Rx_Buffer[JY_BUFFER_SIZE]这个数组
    if(BufferIndex > JY_BUFFER_SIZE)
    {
        BufferIndex = 0; //防止缓冲区溢出
        jy_State = jy_HEAD1;//也就是说，当缓冲区溢出了，就让帧检测状态机从头开始检测，就算最后一次检测到帧头，也不要了
        HeadIndex1 = 0;
        //下面是错误的方案，但有助理解：jy_State的状态不用让它从jy_HEAD1开始，因为数组溢出了之后，接着从数组的第一个元素开始就行，这个思维没有错误，但实际上，会对下面memcpy函数造成影响，因为有个后索引减前索引，这回出错
    }
    switch (jy_State)
    {
		case jy_HEAD1://也就是0
        {
            if (Recv_Data[BufferIndex] == JYHEAD1)
            {
                HeadIndex1 = BufferIndex;//记录帧头在那个大的缓冲区的索引
                jy_State = jy_HEAD2;//帧检测状态机转移到数据状态，从0变到1
                BufferIndex++;//从0开始找帧头，找到了就加1，下次就从上次帧头的位置开始找
            }
            else
            {
                BufferIndex++;
            }
            JY_flag=0;//主频中执行完一次，就让其标志位清零，等待下一次串口中断，也就是说，让主频的处理与串口中断的处理同步  
            break;
        }
		case jy_HEAD2://
        {   
            if (Recv_Data[BufferIndex] == JYHEAD2)
            {
                jy_State = jy_DATA1;//帧检测状态机转移到帧尾状态，从1变到2
                BufferIndex++;
            }
            else
            {
                jy_State = jy_HEAD1;//如果帧头2不是0x53，就让帧检测状态机从头开始检测，就算最后一次检测到帧头，也不要了
                BufferIndex++;

            }
						JY_flag=0;
            break;
        }
        case jy_DATA1:
        {
            jy_State = jy_DATA2;
            BufferIndex++;
            JY_flag=0;
            break;
        }
        case jy_DATA2:
        {
            jy_State = jy_DATA3;
            BufferIndex++;
            JY_flag=0;
            break;
        }
        case jy_DATA3:
        {
            jy_State = jy_DATA4;
            BufferIndex++;
            JY_flag=0;
            break;
        }
        case jy_DATA4:
        {
            jy_State = jy_DATA5;
            BufferIndex++;
            JY_flag=0;
            break;
        }
        case jy_DATA5:
        {
            jy_State = jy_DATA6;
            BufferIndex++;
            JY_flag=0;
            break;
        }
        case jy_DATA6:
        {
            jy_State = jy_DATA7;
            BufferIndex++;
            JY_flag=0;
            break;
        }
        case jy_DATA7:
        {
            jy_State = jy_DATA8;
            BufferIndex++;
            JY_flag=0;
            break;
        }
        case jy_DATA8:
        {
            jy_State = jy_DATA9;
            BufferIndex++;
            JY_flag=0;
            break;
        }
		case jy_DATA9:
        {
            // memcpy(Sendbuffer,&Recv_Data[HeadIndex],EndIndex-HeadIndex+1);//将帧数据拷贝到判断缓冲区
            memcpy(Sendbuffer,Recv_Data + HeadIndex1,11);//将帧数据拷贝到判断缓冲区
            JY901_value(Sendbuffer);//将判断缓冲区的数据拷贝到帧缓冲区
            //memset(Sendbuffer,0,22);//清空串口缓冲区
            BufferIndex++; 
            jy_State = jy_HEAD1;//帧检测状态机转移到帧头状态，从2变到0
            JY_flag=0;
            break;
        }
        default:
        {
            BufferIndex++; 
            break;
            //没在这加JY_flag=0;是有深意的，因为如果发生错误跳出，但此时JY_flag还没清零，
            //又因为主频会比串口中断处理快很多，所以主频就会把上一个发生意外的再次处理一遍
        }
    }  
}
float JY901_x=0.0f,JY901_y=0.0f,JY901_z=0.0f;//定义三个变量，用来存放x,y,z的值
//下面是个处理数据的函数，能得出当前的x,y,z的值
void JY901_value(uint8_t *my_array)
{
    uint8_t i;
    uint8_t sum = 0;
    uint8_t sum_last_two = 0; //sum的后两位
    for(i=0;i<10;i++)
    {
        sum += my_array[i];
    }
    sum_last_two = sum % 256; // 计算 sum 的后两位
    if(sum_last_two == my_array[10]) // 比较 sum 的后两位和校验和
    {
        JY901_x = (float)((int16_t)(my_array[3]<<8|my_array[2]))/32768*180;
        JY901_y = (float)((int16_t)(my_array[5]<<8|my_array[4]))/32768*180;
        JY901_z = (float)((int16_t)(my_array[7]<<8|my_array[6]))/32768*180;
    }
}







