#include "JY901.h"
#include "usart.h"
#include "headfile.h"
#include <string.h>//��memset������C�����еĺ�������Ҫ����������ͷ�ļ�
#include <stdint.h>
#include <stdio.h>
//���Ǹ�������JY901�õĴ���2�������Ƿ�����ÿ֡���ݶ���ʮһ��
#define JY_BUFFER_SIZE 220
uint8_t JY901_RxBuffer[JY_BUFFER_SIZE];
uint8_t JY901_dealBuffer[22];
volatile uint8_t JY_flag=0;            //��Ϊ��Ƶִ���ٶ�̫�죬�������ж���Խ�����������Ҫһ����־λ���ж��Ƿ�ִ��
volatile uint8_t JY_Rx_Index = 0; // ������ջ�����������

#define  JYHEAD1  0x55    //֡ͷ1
#define  JYHEAD2  0x53   //֡ͷ2

/*����֡״̬��*/
typedef enum {
    jy_HEAD1 = 0, //֡ͷ1
    jy_HEAD2,     //֡ͷ2
    jy_DATA1,     //����1
    jy_DATA2,     //����2
    jy_DATA3,     //����3
    jy_DATA4,     //����4
    jy_DATA5,     //����5
    jy_DATA6,     //����6
    jy_DATA7,     //����7
    jy_DATA8,     //����8
    jy_DATA9     //����9
} JY_STATE;
JY_STATE jy_State = jy_HEAD1; //��ʼ��״̬��


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
        if((JY_Rx_Index==0)&&(MAP_UART_receiveData(EUSCI_A0_BASE) != JYHEAD1))//������յ������ݲ���0xFF������������Ϊÿ�ζ��ᷢ0xFF
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
    volatile static uint8_t HeadIndex1 = 0;           //֡ͷ����
    volatile static uint8_t BufferIndex=0;         //�жϻ���������
    //���Ҫ��ĳ�>=����Ҫ���������if�ŵ����������������
    //if((BufferIndex > JY_BUFFER_SIZE)||(HeadIndex>=JY_BUFFER_SIZE-3))//Recv_Data�����������������Ҫ��ӦUART2_Rx_Buffer[JY_BUFFER_SIZE]�������
    if(BufferIndex > JY_BUFFER_SIZE)
    {
        BufferIndex = 0; //��ֹ���������
        jy_State = jy_HEAD1;//Ҳ����˵��������������ˣ�����֡���״̬����ͷ��ʼ��⣬�������һ�μ�⵽֡ͷ��Ҳ��Ҫ��
        HeadIndex1 = 0;
        //�����Ǵ���ķ�������������⣺jy_State��״̬����������jy_HEAD1��ʼ����Ϊ���������֮�󣬽��Ŵ�����ĵ�һ��Ԫ�ؿ�ʼ���У����˼άû�д��󣬵�ʵ���ϣ��������memcpy�������Ӱ�죬��Ϊ�и���������ǰ��������س���
    }
    switch (jy_State)
    {
		case jy_HEAD1://Ҳ����0
        {
            if (Recv_Data[BufferIndex] == JYHEAD1)
            {
                HeadIndex1 = BufferIndex;//��¼֡ͷ���Ǹ���Ļ�����������
                jy_State = jy_HEAD2;//֡���״̬��ת�Ƶ�����״̬����0�䵽1
                BufferIndex++;//��0��ʼ��֡ͷ���ҵ��˾ͼ�1���´ξʹ��ϴ�֡ͷ��λ�ÿ�ʼ��
            }
            else
            {
                BufferIndex++;
            }
            JY_flag=0;//��Ƶ��ִ����һ�Σ��������־λ���㣬�ȴ���һ�δ����жϣ�Ҳ����˵������Ƶ�Ĵ����봮���жϵĴ���ͬ��  
            break;
        }
		case jy_HEAD2://
        {   
            if (Recv_Data[BufferIndex] == JYHEAD2)
            {
                jy_State = jy_DATA1;//֡���״̬��ת�Ƶ�֡β״̬����1�䵽2
                BufferIndex++;
            }
            else
            {
                jy_State = jy_HEAD1;//���֡ͷ2����0x53������֡���״̬����ͷ��ʼ��⣬�������һ�μ�⵽֡ͷ��Ҳ��Ҫ��
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
            // memcpy(Sendbuffer,&Recv_Data[HeadIndex],EndIndex-HeadIndex+1);//��֡���ݿ������жϻ�����
            memcpy(Sendbuffer,Recv_Data + HeadIndex1,11);//��֡���ݿ������жϻ�����
            JY901_value(Sendbuffer);//���жϻ����������ݿ�����֡������
            //memset(Sendbuffer,0,22);//��մ��ڻ�����
            BufferIndex++; 
            jy_State = jy_HEAD1;//֡���״̬��ת�Ƶ�֡ͷ״̬����2�䵽0
            JY_flag=0;
            break;
        }
        default:
        {
            BufferIndex++; 
            break;
            //û�����JY_flag=0;��������ģ���Ϊ���������������������ʱJY_flag��û���㣬
            //����Ϊ��Ƶ��ȴ����жϴ����ܶ࣬������Ƶ�ͻ����һ������������ٴδ���һ��
        }
    }  
}
float JY901_x=0.0f,JY901_y=0.0f,JY901_z=0.0f;//���������������������x,y,z��ֵ
//�����Ǹ��������ݵĺ������ܵó���ǰ��x,y,z��ֵ
void JY901_value(uint8_t *my_array)
{
    uint8_t i;
    uint8_t sum = 0;
    uint8_t sum_last_two = 0; //sum�ĺ���λ
    for(i=0;i<10;i++)
    {
        sum += my_array[i];
    }
    sum_last_two = sum % 256; // ���� sum �ĺ���λ
    if(sum_last_two == my_array[10]) // �Ƚ� sum �ĺ���λ��У���
    {
        JY901_x = (float)((int16_t)(my_array[3]<<8|my_array[2]))/32768*180;
        JY901_y = (float)((int16_t)(my_array[5]<<8|my_array[4]))/32768*180;
        JY901_z = (float)((int16_t)(my_array[7]<<8|my_array[6]))/32768*180;
    }
}







