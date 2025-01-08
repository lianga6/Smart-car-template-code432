#include "headfile.h"
#include "BlueTooth.h"
#include "baudrate_calculate.h"

#define MAX_PACKET_LENGTH 100 // ������ݰ����ȣ�����ʵ���������
			

char Txt_RxPacket[MAX_PACKET_LENGTH];//�����ַ������͵�����
volatile int receiveIndex = 0;//����������±�
volatile int isReceiving = 0;//�����յ�֡ͷ��һ��ʼ��������

/*****************����˵��***************************/
/*
1.��ʼ��BlueTooth_Init();


    // �����͵����ݰ�����
    char data[] = "This is my data packet.";

    // �������ݰ�
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
    eusci_calcBaudDividers((eUSCI_UART_Config *)&uartConfig, BT_USART_BAUDRATE); //���ò�����
     // GPIO����
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(BT_USART_PORT, BT_USART_RX_PIN | BT_USART_TX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    //��ʼ������
    MAP_UART_initModule(BT_USART_BASE, &uartConfig);
    //��������
    MAP_UART_enableModule(BT_USART_BASE);
        
}


void BlueTooth_SendByte(char str)
{

    // �ȴ����ͻ�����Ϊ��
    while (!(BT_USART_SEND_INT->IFG & EUSCI_A_IFG_TXIFG));//�ȴ���һ�η������
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
    BlueTooth_SendByte(BT_START); // ����֡ͷ

    // ������������
    int i;
    for (i = 0; i < dataLength; i++)
    {
        BlueTooth_SendByte(data[i]);
    }

    BlueTooth_SendByte(BT_END); // ����֡β
}





void MyBT_IRQHandler()
{
    uint8_t receivedData = BT_USART_SEND_INT->RXBUF; // ��ȡ���ջ�����������

    if (isReceiving)
    {
        if (receivedData == BT_END) // ������յ�֡β
        {
            // ������յ������ݰ�
            // ����������������ݴ������������������������ݰ�
            

            // ���ý��ջ������ͽ���״̬
            receiveIndex = 0;
            isReceiving = 0;
        }
        else if (receiveIndex < MAX_PACKET_LENGTH)
        {
            // �����ݴ洢�����ջ�����
            Txt_RxPacket[receiveIndex++] = receivedData;
        }
    }
    else if (receivedData == BT_START) // ������յ�֡ͷ
    {
        // ��ʼ�������ݰ�
        isReceiving = 1;
    }
}


//void BlueTooth_Receive_Data(uint8_t com_data)
//{
//		uint8_t i;
//		static uint8_t BlueRxCounter=0;//����
//	  
//		static uint8_t BlueRxState = 0;	
//		//static uint8_t BlueRxFlag = 0;    //�����ɹ����ձ�־λ
//	 //�����ɹ����ձ�־λ��ʵ���Բ�Ҫ����OneTargertNum��OneLoadFlagֱ�Ӳ�����һ�ε�Ҳûɶ������һ�㲻����ֿ�����������ܿ��ˢ��������
//	 //��û��Ҫ����߽�OneTargertNum��OneLoadFlagʹ��һ��֮�����BlueRxFlagһͬ����

//		if(BlueRxState==0&&com_data==BT_START)  //0x52֡ͷ
//		{
//			
//			BlueRxState=1;
//			BlueRxBuffer[BlueRxCounter++]=com_data;  
//		}

//		else if(BlueRxState==1&&com_data==0x21)  //0x21֡ͷ
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
//				//BlueRxFlag1=1;    //�ɹ�����һ�����ݣ������������ж��Ƿ������������ 
//				
////				OneTargertRoom =  BlueRxBuffer1[BlueRxCounter-5];     //[BlueRxCounter-5]��R  Ҳ��Ҳ��82��Ҳ��ʮ�����Ƶ�52    ��ȷ��֡ͷ1 
////				OneLoadFlag =  BlueRxBuffer1[BlueRxCounter-4];  //[BlueRxCounter-4]  ������ʮ���Ƶ�33��Ҳ��ʮ�����Ƶ�21      ��ȷ��֡ͷ2    
//				
//				OneTargetRoom =  BlueRxBuffer[BlueRxCounter-3];        
//				OneLoadFlag =  BlueRxBuffer[BlueRxCounter-2];          
//				
//				
//				//���������������RxCounter1-1�ŵ���֡β

//			}
//		}

//		else if(BlueRxState==3)		//����Ƿ���ܵ�������־
//		{
//				//if(BlueRxBuffer[BlueRxState-1] == 0xf2)    //����д������������������BlueRxState1-1������BlueRxCounter1-1��������ȫ�����Զ�������
//			
//			  if(BlueRxBuffer[BlueRxCounter-1] == 0xf2)    
//				{
//							
//							//BlueRxFlag = 0;
//							BlueRxState = 0;
//							BlueRxCounter = 0;
//						
//				}
//				else   //���մ���
//				{
//							BlueRxState  = 0;
//							BlueRxCounter =0;
//							for(i=0;i<7;i++)
//							{
//									BlueRxBuffer[i]=0x00;      //�����������������
//							}
//				}
//		} 

//		else   //�����쳣
//		{
//				BlueRxState = 0;
//				BlueRxCounter = 0;
//				for(i=0;i<7;i++)
//				{
//						BlueRxBuffer[i]=0x00;      //�����������������
//				}
//		}
//}


