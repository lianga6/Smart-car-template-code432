#include "usart.h"
#include "baudrate_calculate.h"
#include <string.h>//��memset������C�����еĺ�������Ҫ����������ͷ�ļ�
#include <stdint.h>
#include <stdio.h>
#include <led.h>

/*****************   ����˵��   *****************
 *
 * ���������Խӱ�׼���������ĺ���:
 * int fputc(int ch, FILE *f);
 * int fgetc(FILE *f);
 * Դ��ΪBiliBiliƽ̨UP�� ��CloudBoyStudio�� ��д
 * �ڴ�Ҳ���л
 * ʹ��ʱ�ǵù�ѡħ�������Use MicroLIB
 *
 *****************   ˵������   *****************/
int fputc(int ch, FILE *f)//ֻ�д�����������printf��Ҳ����openmv�Ĵ���
{
    UART_transmitData(EUSCI_A3_BASE, ch & 0xFF);
    return ch;
}


int fgetc(FILE *f)
{
    while (EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG !=
           UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG))
        ;
    return UART_receiveData(EUSCI_A0_BASE);
}

/*****************************************************************
 *Function:uart_init(uint32_t baudRate)
 *Description:��ʼ������
 *Input:�������Ϊ������
 *Output:��
 *Return:��
 *Others:
 * �ٵ�Ƶʱ��Ƶ���£��߲�����ʹ�ô���ʱ������,
 * ����35768Hz��19200������,
 * ��ʹ�ô��������ʱ���Գ��Խ��Ͳ����ʡ�
 * ��baudrate_calculate��������ȥ�ļ��ڲ鿴��
 *****************************************************************/
void uart2_init(uint32_t baudRate)
{
#ifdef EUSCI_A_UART_7_BIT_LEN
    //�̼���v3_40_01_02
    //Ĭ��SMCLK 48MHz ������ 115200
    const eUSCI_UART_ConfigV1 uartConfig =
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
            EUSCI_A_UART_8_BIT_LEN                         // 8 bit data length
        };
    eusci_calcBaudDividers((eUSCI_UART_ConfigV1 *)&uartConfig, baudRate); //���ò�����
#else
    //�̼���v3_21_00_05
    //Ĭ��SMCLK 48MHz ������ 115200
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
    eusci_calcBaudDividers((eUSCI_UART_Config *)&uartConfig, baudRate); //���ò�����
#endif
    // GPIO����
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    //��ʼ������
    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);
    //��������
    MAP_UART_enableModule(EUSCI_A2_BASE);
}

void uart_init(uint32_t baudRate)
{
#ifdef EUSCI_A_UART_7_BIT_LEN
    //�̼���v3_40_01_02
    //Ĭ��SMCLK 48MHz ������ 115200
    const eUSCI_UART_ConfigV1 uartConfig =
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
            EUSCI_A_UART_8_BIT_LEN                         // 8 bit data length
        };
    eusci_calcBaudDividers((eUSCI_UART_ConfigV1 *)&uartConfig, baudRate); //���ò�����
#else
    //�̼���v3_21_00_05
    //Ĭ��SMCLK 48MHz ������ 115200
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
    eusci_calcBaudDividers((eUSCI_UART_Config *)&uartConfig, baudRate); //���ò�����
#endif
    // GPIO����
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P9, GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
    //��ʼ������
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);
    MAP_UART_initModule(EUSCI_A1_BASE, &uartConfig);
    MAP_UART_initModule(EUSCI_A3_BASE, &uartConfig);

    //��������
    MAP_UART_enableModule(EUSCI_A0_BASE);
    MAP_UART_enableModule(EUSCI_A1_BASE);
    MAP_UART_enableModule(EUSCI_A3_BASE);
}
/*****************************************************************
 *Function:UART_NVIC_Init(void)
 *Description:��ʼ�������ж�
 *Input:��
 *Output:��
 *Return:��
 *Others:δ�������жϣ����Լ�����
 *Data:2021/09/15
 *Author:�Ǻ��е�����
 *****************************************************************/
void UART_NVIC_Init(void)//�����жϳ�ʼ�������Լ��������ж�
{
    /*������������ж�*/
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    /*�������ڶ˿��ж�*/
    Interrupt_enableInterrupt(INT_EUSCIA0);
    /*������������ж�*/
    UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    /*�������ڶ˿��ж�*/
    Interrupt_enableInterrupt(INT_EUSCIA1);
    /*������������ж�*/
    UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    /*�������ڶ˿��ж�*/
    Interrupt_enableInterrupt(INT_EUSCIA2);
    /*������������ж�*/
    UART_enableInterrupt(EUSCI_A3_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    /*�������ڶ˿��ж�*/
    Interrupt_enableInterrupt(INT_EUSCIA3);
}
/*************************************************
 * ��  ��  ��:UART_send_Byte
 * ��       ��:����һ���ֽ�����
 * ��       ��:UART_CHA:UART��ѡͨ������exinuart.h���г�
 *          Data:Ҫ���͵�8λ����
 * ע������:��
 *************************************************/
void UART_send_Byte(UART_CHA_enum UART_CHA, uint8 Data)
{
    switch (UART_CHA)
    {
    case (UART0):
        while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG))
            ; //�ȴ���һ�η������
        EUSCI_A0->TXBUF = Data;
        break;
    case (UART1):
        while (!(EUSCI_A1->IFG & EUSCI_A_IFG_TXIFG))
            ; //�ȴ���һ�η������
        EUSCI_A1->TXBUF = Data;
        break;
    case (UART2):
        while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG))
            ; //�ȴ���һ�η������
        EUSCI_A2->TXBUF = Data;
        break;
    case (UART3):
        while (!(EUSCI_A3->IFG & EUSCI_A_IFG_TXIFG))
            ; //�ȴ���һ�η������
        EUSCI_A3->TXBUF = Data;
        break;
    default:;
    }
}
/*************************************************
 * ��  ��  ��:UART_recv_Byte
 * ��       ��:����һ���ֽ�����
 * ��       ��:UART_CHA:UART��ѡͨ������exinuart.h���г�
 * ע������:��
 *************************************************/
uint8 UART_recv_Byte(UART_CHA_enum UART_CHA)
{
    uint8 result;
    switch (UART_CHA)
    {
    case (UART0):
        result = EUSCI_A0->RXBUF;
        break; //ȡ������������
    case (UART1):
        result = EUSCI_A1->RXBUF;
        break; //ȡ������������
    case (UART2):
        result = EUSCI_A2->RXBUF;
        break; //ȡ������������
    case (UART3):
        result = EUSCI_A3->RXBUF;
        break; //ȡ������������
    default:;
    }
    return result;
}
/*************************************************
 * ��  ��  ��:UART_send_string
 * ��       ��:����һ���ַ���
 * ��       ��:UART_CHA:UART��ѡͨ������exinuart.h���г�
 *          txt:��Ҫ���͵��ַ���
 * ע������:��
 *************************************************/
void UART_send_string(UART_CHA_enum UART_CHA, char *txt)
{
    int i;
    for (i = 0; txt[i]; i++)
    {
        UART_send_Byte(UART_CHA, txt[i]);
    }
}
//UART_send_string(UART2, "hello"); // �����ַ���
/*************************************************
 * ��  ��  ��:UART_send_short
 * ��       ��:����һ��16λ����
 * ��       ��:UART_CHA:UART��ѡͨ������exinuart.h���г�
 *          num:��Ҫ���͵�16λ���ͱ���
 * ע������:�Ӹ�λ��ʼ����
 *************************************************/
void UART_send_short(UART_CHA_enum UART_CHA, uint16 num)
{
    int i;
    for (i = 0; i < 2; i++)
    {
        UART_send_Byte(UART_CHA, (num & 0xff00) >> 8);
        num = num << 8;
    }
}
/*************************************************
 * ��  ��  ��:UART_send_int
 * ��       ��:����һ��32λ����
 * ��       ��:UART_CHA:UART��ѡͨ������exinuart.h���г�
 *          num:��Ҫ���͵�32λ���ͱ���
 * ע������:�Ӹ�λ��ʼ����
 *************************************************/
void UART_send_int(UART_CHA_enum UART_CHA, uint32 num)
{
    int i;
    for (i = 0; i < 4; i++)
    {
        UART_send_Byte(UART_CHA, (num & 0xff000000) >> 24);
        num = num << 8;
    }
}












//����ȫ�����̴��룬�����Ƿ��ӿռ����

uint8_t SendBufferOpenMv[8];         //���ͻ�����������ڷ��ͺ�����
uint8_t JugleBuffer[15];         //�жϻ���������
int my_data=0;               //����Ҫ���յ�����
int output=0;                //�����pwm�ȽϺ���������
uint8_t state_flag=0;            //��Ϊ��Ƶִ���ٶ�̫�죬�������ж���Խ�����������Ҫһ����־λ���ж��Ƿ�ִ��
//֡ͷ֡β��Ҫ��0xFF!!!
#define  HEAD  0xFC    //֡ͷ
#define  END   0xFE    //֡β
//����0�����жϷ�����
#define MAX_RX_BUFFER_SIZE 256// ������ջ�����������С
uint8_t UART3_Rx_Buffer[MAX_RX_BUFFER_SIZE]; // ������ջ�����,Ҳ���Ǵ��ڻ�����
volatile uint8_t UART3_Rx_Index = 0; // ������ջ�����������

/*����֡״̬��*/
typedef enum {
    FRAME_HEAD = 0, //֡ͷ1
    FRAME_DATA1,     //����1
    FRAME_DATA2,     //����2
    FRAME_END    //֡ͷ2
} FRAME_STATE;

FRAME_STATE frame_state = FRAME_HEAD; /*����֡���״̬������ʼ�����֡ͷ״̬,������һ����Ϊ
 frame_state �ı���������Ϊ FRAME_STATE ö�����ͣ���ʼֵΪ FRAME_HEAD�����ڱ�ʾ����֡��״̬�������ڼ������֡��֡ͷ*/

void EUSCIA3_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A3_BASE);
    MAP_UART_clearInterruptFlag(EUSCI_A3_BASE, status);
    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        if(MAP_UART_receiveData(EUSCI_A3_BASE) == 0xFF)//������յ���������0xFF������������Ϊÿ�ζ��ᷢ0xFF
        {
            return;
        }
        if((UART3_Rx_Index==0)&&(MAP_UART_receiveData(EUSCI_A3_BASE) != HEAD))//������յ������ݲ���0xFF������������Ϊÿ�ζ��ᷢ0xFF
        {
            return;
        }
        UART3_Rx_Buffer[UART3_Rx_Index++] = MAP_UART_receiveData(EUSCI_A3_BASE);// �����յ������ݴ洢�����ջ������� 
        if (UART3_Rx_Index >= MAX_RX_BUFFER_SIZE) // ������ջ���������������������
        {
            UART3_Rx_Index = 0;
        }
        state_flag=1; //���յ����ݣ���־λ��1
    }
}


/*���֡���ݺ���
param:*Recv_Data:���ڻ�������������ĵ�ַָ���Ǹ���Ļ�������һ���Ҫ�����������ȡ�����ݽ����ж�
      *Sendbuffer:֡��������������ĵ�ַָ���Ǹ�С�ģ���ž�ֻ��֡���ݴ�С������Ū��һ�㣩������  
*/
void FrameJudge(uint8_t *Recv_Data,uint8_t *Sendbuffer)
{
    volatile static uint8_t HeadIndex = 0;           //֡ͷ����
    volatile static uint8_t EndIndex = 0;            //֡β����
    static uint8_t BufferIndex = 0;         //�жϻ���������
    //���Ҫ��ĳ�>=����Ҫ���������if�ŵ����������������
    //if((BufferIndex > MAX_RX_BUFFER_SIZE)||(HeadIndex>=MAX_RX_BUFFER_SIZE-3))//Recv_Data�����������������Ҫ��ӦUART2_Rx_Buffer[MAX_RX_BUFFER_SIZE]�������
    if(BufferIndex > MAX_RX_BUFFER_SIZE)
    {
        BufferIndex = 0; //��ֹ���������
        frame_state = FRAME_HEAD;//Ҳ����˵��������������ˣ�����֡���״̬����ͷ��ʼ��⣬�������һ�μ�⵽֡ͷ��Ҳ��Ҫ��
        HeadIndex = 0;
        EndIndex = 0;  
        //�����Ǵ���ķ�������������⣺frame_state��״̬����������FRAME_HEAD��ʼ����Ϊ���������֮�󣬽��Ŵ�����ĵ�һ��Ԫ�ؿ�ʼ���У����˼άû�д��󣬵�ʵ���ϣ��������memcpy�������Ӱ�죬��Ϊ�и���������ǰ��������س���
    }
    switch (frame_state)
    {
		case FRAME_HEAD://Ҳ����0
        {
            if (Recv_Data[BufferIndex] == HEAD)
            {
                HeadIndex = BufferIndex;//��¼֡ͷ���Ǹ���Ļ�����������
                frame_state = FRAME_DATA1;//֡���״̬��ת�Ƶ�����״̬����0�䵽1
                BufferIndex++;//��0��ʼ��֡ͷ���ҵ��˾ͼ�1���´ξʹ��ϴ�֡ͷ��λ�ÿ�ʼ��
            }
            else
            {
                BufferIndex++;
            }
            state_flag=0;//��Ƶ��ִ����һ�Σ��������־λ���㣬�ȴ���һ�δ����жϣ�Ҳ����˵������Ƶ�Ĵ����봮���жϵĴ���ͬ��  
            break;
        }
			case FRAME_DATA1://Ҳ����1
        {
            frame_state = FRAME_DATA2;//֡���״̬��ת�Ƶ�֡β״̬����1�䵽2
            BufferIndex++;
            state_flag=0;
            break;
        }
        	case FRAME_DATA2://Ҳ����1
        {
            frame_state = FRAME_END;//֡���״̬��ת�Ƶ�֡β״̬����1�䵽2
            BufferIndex++;
            state_flag=0;
            break;
        }
			case FRAME_END:
        {
            if (Recv_Data[BufferIndex] == END)
            {
                EndIndex = BufferIndex;//��¼֡β���Ǹ���Ļ�����������
                // memcpy(Sendbuffer,&Recv_Data[HeadIndex],EndIndex-HeadIndex+1);//��֡���ݿ������жϻ�����
                memcpy(Sendbuffer,Recv_Data + HeadIndex,4);//��֡���ݿ������жϻ�����
                GetBufferdata(Sendbuffer);//���жϻ����������ݿ�����֡������
//                memset(Sendbuffer,0,15);//��մ��ڻ�����
                BufferIndex++; 
                frame_state = FRAME_HEAD;//֡���״̬��ת�Ƶ�֡ͷ״̬����2�䵽0
            }
            else
            {
                BufferIndex++;
                frame_state = FRAME_HEAD;//֡���״̬��ת�Ƶ�֡ͷ״̬����2�䵽0,Ҳ����˵���֡β����֡β����ô�ʹ�ͷ��ʼ��
            }
            state_flag=0;
            break;
        }
        default:
        {
            break;
            //û�����state_flag=0;��������ģ���Ϊ���������������������ʱstate_flag��û���㣬
            //����Ϊ��Ƶ��ȴ����жϴ����ܶ࣬������Ƶ�ͻ����һ������������ٴδ���һ��
        }
    }  
}

//ȡ�����������ݵĺ���
void GetBufferdata(uint8_t *my_array)
{
    my_data=my_array[1];//ȡ������
    output=my_data-100;//������ת���ɻ���
} 

//��openmv��������
uint8_t data1 = 0;//������������������Ϊ0
uint8_t data2 = 0;
void UART0_send_data(void)
{
    uint8_t i;
    for(i = 0; i <= 4; i++)   
    {
    sprintf((char *)SendBufferOpenMv, "*%d%d&", data1,data2);//��data1��data2�����ݴ��뷢�ͻ�����
    UART_send_string(UART0, (char *)SendBufferOpenMv); //��������
    }
}

