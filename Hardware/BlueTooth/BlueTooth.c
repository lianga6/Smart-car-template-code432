#include "headfile.h"
#include "BlueTooth.h"
#include "baudrate_calculate.h"

#define MAX_PACKET_LENGTH 100 // 最大数据包长度，根据实际情况调整
			

char Txt_RxPacket[MAX_PACKET_LENGTH];//接收字符串类型的数据
volatile int receiveIndex = 0;//接收数组的下标
volatile int isReceiving = 0;//当接收到帧头置一开始接收数据

/*****************发送说明***************************/
/*
1.初始化BlueTooth_Init();


    // 待发送的数据包内容
    char data[] = "This is my data packet.";

    // 发送数据包
    BlueTooth_SendPacket(data, sizeof(data));


*/


void BlueTooth_Init(void)
{
        const eUSCI_UART_Config uartConfig =
        {
            EUSCI_A_UART_CLOCKSOURCE_SMCLK,                // SMCLK Clock Source
            26,                                            // BRDIV = 26
            0,                                             // UCxBRF = 0
            111,                                           // UCxBRS = 111
            EUSCI_A_UART_NO_PARITY,                        // No Parity
            EUSCI_A_UART_LSB_FIRST,                        // MSB First
            EUSCI_A_UART_ONE_STOP_BIT,                     // One stop bit
            EUSCI_A_UART_MODE,                             // UART mode
            EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION, // Oversampling
        };
    eusci_calcBaudDividers((eUSCI_UART_Config *)&uartConfig, BT_USART_BAUDRATE); //配置波特率
     // GPIO复用
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(BT_USART_PORT, BT_USART_RX_PIN | BT_USART_TX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    //初始化串口
    MAP_UART_initModule(BT_USART_BASE, &uartConfig);
    //开启串口
    MAP_UART_enableModule(BT_USART_BASE);
        
}


void BlueTooth_SendByte(char str)
{

    // 等待发送缓冲区为空
    while (!(BT_USART_SEND_INT->IFG & EUSCI_A_IFG_TXIFG));//等待上一次发送完成
    BT_USART_SEND_INT->TXBUF = str;
}

void BlueTooth_SendString(char *txt)
{
    int i;
    for (i = 0; txt[i]; i++)
    {
        BlueTooth_SendByte(txt[i]);
    }
}

void BlueTooth_SendPacket(char *data, int dataLength)
{
    BlueTooth_SendByte(BT_START); // 发送帧头

    // 发送数据内容
    int i;
    for (i = 0; i < dataLength; i++)
    {
        BlueTooth_SendByte(data[i]);
    }

    BlueTooth_SendByte(BT_END); // 发送帧尾
}





void MyBT_IRQHandler()
{
    uint8_t receivedData = BT_USART_SEND_INT->RXBUF; // 读取接收缓冲区的数据

    if (isReceiving)
    {
        if (receivedData == BT_END) // 如果接收到帧尾
        {
            // 处理接收到的数据包
            // 可以在这里进行数据处理或调用其他函数来处理数据包
            

            // 重置接收缓冲区和接收状态
            receiveIndex = 0;
            isReceiving = 0;
        }
        else if (receiveIndex < MAX_PACKET_LENGTH)
        {
            // 将数据存储到接收缓冲区
            Txt_RxPacket[receiveIndex++] = receivedData;
        }
    }
    else if (receivedData == BT_START) // 如果接收到帧头
    {
        // 开始接收数据包
        isReceiving = 1;
    }
}


//void BlueTooth_Receive_Data(uint8_t com_data)
//{
//		uint8_t i;
//		static uint8_t BlueRxCounter=0;//计数
//	  
//		static uint8_t BlueRxState = 0;	
//		//static uint8_t BlueRxFlag = 0;    //蓝牙成功接收标志位
//	 //蓝牙成功接收标志位其实可以不要。我OneTargertNum，OneLoadFlag直接采用上一次的也没啥，反正一般不会出现卡死的情况，很快就刷新数据了
//	 //更没必要在外边将OneTargertNum，OneLoadFlag使用一次之后就与BlueRxFlag一同清零

//		if(BlueRxState==0&&com_data==BT_START)  //0x52帧头
//		{
//			
//			BlueRxState=1;
//			BlueRxBuffer[BlueRxCounter++]=com_data;  
//		}

//		else if(BlueRxState==1&&com_data==0x21)  //0x21帧头
//		{
//			BlueRxState=2;
//			BlueRxBuffer[BlueRxCounter++]=com_data;
//		}
//		
//		else if(BlueRxState==2)
//		{
//			 
//			BlueRxBuffer[BlueRxCounter++]=com_data;
//			if(BlueRxCounter>=7||com_data == 0xf2)    
//			{
//				BlueRxState=3;
//				
//				//BlueRxFlag1=1;    //成功接收一次数据，可以用来做判断是否采用最新数据 
//				
////				OneTargertRoom =  BlueRxBuffer1[BlueRxCounter-5];     //[BlueRxCounter-5]是R  也即也即82，也即十六进制的52    正确的帧头1 
////				OneLoadFlag =  BlueRxBuffer1[BlueRxCounter-4];  //[BlueRxCounter-4]  里面是十进制的33，也即十六进制的21      正确的帧头2    
//				
//				OneTargetRoom =  BlueRxBuffer[BlueRxCounter-3];        
//				OneLoadFlag =  BlueRxBuffer[BlueRxCounter-2];          
//				
//				
//				//如果接收正常，则RxCounter1-1放的是帧尾

//			}
//		}

//		else if(BlueRxState==3)		//检测是否接受到结束标志
//		{
//				//if(BlueRxBuffer[BlueRxState-1] == 0xf2)    //这里写错啦！！！！！不是BlueRxState1-1，而是BlueRxCounter1-1！！！！全部都自动清零啦
//			
//			  if(BlueRxBuffer[BlueRxCounter-1] == 0xf2)    
//				{
//							
//							//BlueRxFlag = 0;
//							BlueRxState = 0;
//							BlueRxCounter = 0;
//						
//				}
//				else   //接收错误
//				{
//							BlueRxState  = 0;
//							BlueRxCounter =0;
//							for(i=0;i<7;i++)
//							{
//									BlueRxBuffer[i]=0x00;      //将存放数据数组清零
//							}
//				}
//		} 

//		else   //接收异常
//		{
//				BlueRxState = 0;
//				BlueRxCounter = 0;
//				for(i=0;i<7;i++)
//				{
//						BlueRxBuffer[i]=0x00;      //将存放数据数组清零
//				}
//		}
//}


