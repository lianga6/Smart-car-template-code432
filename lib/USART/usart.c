#include "usart.h"
#include "baudrate_calculate.h"
#include <string.h>//像memset这样的C语言中的函数，就要调用这三个头文件
#include <stdint.h>
#include <stdio.h>
#include <led.h>

/*****************   函数说明   *****************
 *
 * 以上两条对接标准输入输出库的函数:
 * int fputc(int ch, FILE *f);
 * int fgetc(FILE *f);
 * 源码为BiliBili平台UP主 “CloudBoyStudio” 编写
 * 在此也表感谢
 * 使用时记得勾选魔术棒里的Use MicroLIB
 *
 *****************   说明结束   *****************/
int fputc(int ch, FILE *f)//只有串口三可以用printf，也就是openmv的串口
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
 *Description:初始化串口
 *Input:输入参数为波特率
 *Output:无
 *Return:无
 *Others:
 * ①低频时钟频率下，高波特率使得传输时误差过大,
 * 比如35768Hz下19200波特率,
 * 会使得传输出错，这时可以尝试降低波特率。
 * ②baudrate_calculate的问题请去文件内查看。
 *****************************************************************/
void uart2_init(uint32_t baudRate)
{
#ifdef EUSCI_A_UART_7_BIT_LEN
    //固件库v3_40_01_02
    //默认SMCLK 48MHz 比特率 115200
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
    eusci_calcBaudDividers((eUSCI_UART_ConfigV1 *)&uartConfig, baudRate); //配置波特率
#else
    //固件库v3_21_00_05
    //默认SMCLK 48MHz 比特率 115200
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
    eusci_calcBaudDividers((eUSCI_UART_Config *)&uartConfig, baudRate); //配置波特率
#endif
    // GPIO复用
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    //初始化串口
    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);
    //开启串口
    MAP_UART_enableModule(EUSCI_A2_BASE);
}

void uart_init(uint32_t baudRate)
{
#ifdef EUSCI_A_UART_7_BIT_LEN
    //固件库v3_40_01_02
    //默认SMCLK 48MHz 比特率 115200
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
    eusci_calcBaudDividers((eUSCI_UART_ConfigV1 *)&uartConfig, baudRate); //配置波特率
#else
    //固件库v3_21_00_05
    //默认SMCLK 48MHz 比特率 115200
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
    eusci_calcBaudDividers((eUSCI_UART_Config *)&uartConfig, baudRate); //配置波特率
#endif
    // GPIO复用
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P9, GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
    //初始化串口
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);
    MAP_UART_initModule(EUSCI_A1_BASE, &uartConfig);
    MAP_UART_initModule(EUSCI_A3_BASE, &uartConfig);

    //开启串口
    MAP_UART_enableModule(EUSCI_A0_BASE);
    MAP_UART_enableModule(EUSCI_A1_BASE);
    MAP_UART_enableModule(EUSCI_A3_BASE);
}
/*****************************************************************
 *Function:UART_NVIC_Init(void)
 *Description:初始化串口中断
 *Input:无
 *Output:无
 *Return:无
 *Others:未开启总中断，需自己开启
 *Data:2021/09/15
 *Author:星海中的绿洲
 *****************************************************************/
void UART_NVIC_Init(void)//串口中断初始化，需自己开启总中断
{
    /*开启串口相关中断*/
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    /*开启串口端口中断*/
    Interrupt_enableInterrupt(INT_EUSCIA0);
    /*开启串口相关中断*/
    UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    /*开启串口端口中断*/
    Interrupt_enableInterrupt(INT_EUSCIA1);
    /*开启串口相关中断*/
    UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    /*开启串口端口中断*/
    Interrupt_enableInterrupt(INT_EUSCIA2);
    /*开启串口相关中断*/
    UART_enableInterrupt(EUSCI_A3_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    /*开启串口端口中断*/
    Interrupt_enableInterrupt(INT_EUSCIA3);
}
/*************************************************
 * 函  数  名:UART_send_Byte
 * 功       能:发送一个字节数据
 * 参       数:UART_CHA:UART可选通道，在exinuart.h中列出
 *          Data:要发送的8位数据
 * 注意事项:无
 *************************************************/
void UART_send_Byte(UART_CHA_enum UART_CHA, uint8 Data)
{
    switch (UART_CHA)
    {
    case (UART0):
        while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG))
            ; //等待上一次发送完成
        EUSCI_A0->TXBUF = Data;
        break;
    case (UART1):
        while (!(EUSCI_A1->IFG & EUSCI_A_IFG_TXIFG))
            ; //等待上一次发送完成
        EUSCI_A1->TXBUF = Data;
        break;
    case (UART2):
        while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG))
            ; //等待上一次发送完成
        EUSCI_A2->TXBUF = Data;
        break;
    case (UART3):
        while (!(EUSCI_A3->IFG & EUSCI_A_IFG_TXIFG))
            ; //等待上一次发送完成
        EUSCI_A3->TXBUF = Data;
        break;
    default:;
    }
}
/*************************************************
 * 函  数  名:UART_recv_Byte
 * 功       能:接收一个字节数据
 * 参       数:UART_CHA:UART可选通道，在exinuart.h中列出
 * 注意事项:无
 *************************************************/
uint8 UART_recv_Byte(UART_CHA_enum UART_CHA)
{
    uint8 result;
    switch (UART_CHA)
    {
    case (UART0):
        result = EUSCI_A0->RXBUF;
        break; //取出缓冲区数据
    case (UART1):
        result = EUSCI_A1->RXBUF;
        break; //取出缓冲区数据
    case (UART2):
        result = EUSCI_A2->RXBUF;
        break; //取出缓冲区数据
    case (UART3):
        result = EUSCI_A3->RXBUF;
        break; //取出缓冲区数据
    default:;
    }
    return result;
}
/*************************************************
 * 函  数  名:UART_send_string
 * 功       能:发送一个字符串
 * 参       数:UART_CHA:UART可选通道，在exinuart.h中列出
 *          txt:所要发送的字符串
 * 注意事项:无
 *************************************************/
void UART_send_string(UART_CHA_enum UART_CHA, char *txt)
{
    int i;
    for (i = 0; txt[i]; i++)
    {
        UART_send_Byte(UART_CHA, txt[i]);
    }
}
//UART_send_string(UART2, "hello"); // 发送字符串
/*************************************************
 * 函  数  名:UART_send_short
 * 功       能:发送一个16位整型
 * 参       数:UART_CHA:UART可选通道，在exinuart.h中列出
 *          num:所要发送的16位整型变量
 * 注意事项:从高位开始发送
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
 * 函  数  名:UART_send_int
 * 功       能:发送一个32位整型
 * 参       数:UART_CHA:UART可选通道，在exinuart.h中列出
 *          num:所要发送的32位整型变量
 * 注意事项:从高位开始发送
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












//上面全是例程代码，下面是发挥空间代码

uint8_t SendBufferOpenMv[8];         //发送缓冲区，这个在发送函数里
uint8_t JugleBuffer[15];         //判断缓冲区索引
int my_data=0;               //我需要接收的数据
int output=0;                //输出到pwm比较函数的数据
uint8_t state_flag=0;            //因为主频执行速度太快，而串口中断相对较慢，所以需要一个标志位来判断是否执行
//帧头帧尾不要用0xFF!!!
#define  HEAD  0xFC    //帧头
#define  END   0xFE    //帧尾
//串口0接收中断服务函数
#define MAX_RX_BUFFER_SIZE 256// 定义接收缓冲区的最大大小
uint8_t UART3_Rx_Buffer[MAX_RX_BUFFER_SIZE]; // 定义接收缓冲区,也就是串口缓冲区
volatile uint8_t UART3_Rx_Index = 0; // 定义接收缓冲区的索引

/*数据帧状态机*/
typedef enum {
    FRAME_HEAD = 0, //帧头1
    FRAME_DATA1,     //数据1
    FRAME_DATA2,     //数据2
    FRAME_END    //帧头2
} FRAME_STATE;

FRAME_STATE frame_state = FRAME_HEAD; /*定义帧检测状态机，初始化检测帧头状态,定义了一个名为
 frame_state 的变量，类型为 FRAME_STATE 枚举类型，初始值为 FRAME_HEAD，用于表示数据帧的状态机，用于检测数据帧的帧头*/

void EUSCIA3_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A3_BASE);
    MAP_UART_clearInterruptFlag(EUSCI_A3_BASE, status);
    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        if(MAP_UART_receiveData(EUSCI_A3_BASE) == 0xFF)//如果接收到的数据是0xFF，就跳过，因为每次都会发0xFF
        {
            return;
        }
        if((UART3_Rx_Index==0)&&(MAP_UART_receiveData(EUSCI_A3_BASE) != HEAD))//如果接收到的数据不是0xFF，就跳过，因为每次都会发0xFF
        {
            return;
        }
        UART3_Rx_Buffer[UART3_Rx_Index++] = MAP_UART_receiveData(EUSCI_A3_BASE);// 将接收到的数据存储到接收缓冲区中 
        if (UART3_Rx_Index >= MAX_RX_BUFFER_SIZE) // 如果接收缓冲区已满，则重置索引
        {
            UART3_Rx_Index = 0;
        }
        state_flag=1; //接收到数据，标志位置1
    }
}


/*检测帧数据函数
param:*Recv_Data:串口缓冲区，会把它的地址指向那个大的缓冲区，一会儿要从这个缓冲区取出数据进行判断
      *Sendbuffer:帧缓冲区，会把它的地址指向那个小的（大概就只有帧数据大小，可以弄大一点）缓冲区  
*/
void FrameJudge(uint8_t *Recv_Data,uint8_t *Sendbuffer)
{
    volatile static uint8_t HeadIndex = 0;           //帧头索引
    volatile static uint8_t EndIndex = 0;            //帧尾索引
    static uint8_t BufferIndex = 0;         //判断缓冲区索引
    //如果要想改成>=，就要把下面这个if放到整个函数的最后面
    //if((BufferIndex > MAX_RX_BUFFER_SIZE)||(HeadIndex>=MAX_RX_BUFFER_SIZE-3))//Recv_Data这个数组在主函数里要对应UART2_Rx_Buffer[MAX_RX_BUFFER_SIZE]这个数组
    if(BufferIndex > MAX_RX_BUFFER_SIZE)
    {
        BufferIndex = 0; //防止缓冲区溢出
        frame_state = FRAME_HEAD;//也就是说，当缓冲区溢出了，就让帧检测状态机从头开始检测，就算最后一次检测到帧头，也不要了
        HeadIndex = 0;
        EndIndex = 0;  
        //下面是错误的方案，但有助理解：frame_state的状态不用让它从FRAME_HEAD开始，因为数组溢出了之后，接着从数组的第一个元素开始就行，这个思维没有错误，但实际上，会对下面memcpy函数造成影响，因为有个后索引减前索引，这回出错
    }
    switch (frame_state)
    {
		case FRAME_HEAD://也就是0
        {
            if (Recv_Data[BufferIndex] == HEAD)
            {
                HeadIndex = BufferIndex;//记录帧头在那个大的缓冲区的索引
                frame_state = FRAME_DATA1;//帧检测状态机转移到数据状态，从0变到1
                BufferIndex++;//从0开始找帧头，找到了就加1，下次就从上次帧头的位置开始找
            }
            else
            {
                BufferIndex++;
            }
            state_flag=0;//主频中执行完一次，就让其标志位清零，等待下一次串口中断，也就是说，让主频的处理与串口中断的处理同步  
            break;
        }
			case FRAME_DATA1://也就是1
        {
            frame_state = FRAME_DATA2;//帧检测状态机转移到帧尾状态，从1变到2
            BufferIndex++;
            state_flag=0;
            break;
        }
        	case FRAME_DATA2://也就是1
        {
            frame_state = FRAME_END;//帧检测状态机转移到帧尾状态，从1变到2
            BufferIndex++;
            state_flag=0;
            break;
        }
			case FRAME_END:
        {
            if (Recv_Data[BufferIndex] == END)
            {
                EndIndex = BufferIndex;//记录帧尾在那个大的缓冲区的索引
                // memcpy(Sendbuffer,&Recv_Data[HeadIndex],EndIndex-HeadIndex+1);//将帧数据拷贝到判断缓冲区
                memcpy(Sendbuffer,Recv_Data + HeadIndex,4);//将帧数据拷贝到判断缓冲区
                GetBufferdata(Sendbuffer);//将判断缓冲区的数据拷贝到帧缓冲区
//                memset(Sendbuffer,0,15);//清空串口缓冲区
                BufferIndex++; 
                frame_state = FRAME_HEAD;//帧检测状态机转移到帧头状态，从2变到0
            }
            else
            {
                BufferIndex++;
                frame_state = FRAME_HEAD;//帧检测状态机转移到帧头状态，从2变到0,也就是说如果帧尾不是帧尾，那么就从头开始找
            }
            state_flag=0;
            break;
        }
        default:
        {
            break;
            //没在这加state_flag=0;是有深意的，因为如果发生错误跳出，但此时state_flag还没清零，
            //又因为主频会比串口中断处理快很多，所以主频就会把上一个发生意外的再次处理一遍
        }
    }  
}

//取出缓冲区数据的函数
void GetBufferdata(uint8_t *my_array)
{
    my_data=my_array[1];//取出数据
    output=my_data-100;//将数据转换成回来
} 

//给openmv发送数据
uint8_t data1 = 0;//姑且先让这两个数据为0
uint8_t data2 = 0;
void UART0_send_data(void)
{
    uint8_t i;
    for(i = 0; i <= 4; i++)   
    {
    sprintf((char *)SendBufferOpenMv, "*%d%d&", data1,data2);//将data1和data2的数据存入发送缓冲区
    UART_send_string(UART0, (char *)SendBufferOpenMv); //发送数据
    }
}

