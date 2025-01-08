#ifndef __HEADFILE_H
#define __HEADFILE_H

#include "key.h"
#include "gpio.h"
#include <driverlib.h>
#include "delay.h"
#include "clock.h"
#include <stdint.h>
#include <stdbool.h>
#include "oled.h"
#include "HC-SR0.h"
typedef enum
{
    UART0,
    UART1,
    UART2,
    UART3
} UART_CHA_enum; //串口可选通道枚举
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int16_t int16;
typedef int32_t int32;

extern int encoder_L;
extern int encoder_R;


//主函数状态机判断
typedef enum {
  state1=0,
  state2,
  state3,
  state4
} STATE;

extern STATE SYS_state; //系统状态
extern int dir,control;





#endif
