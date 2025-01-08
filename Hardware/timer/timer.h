#ifndef __TIMER_H
#define __TIMER_H
#include "headfile.h"

void TimA0_Init(uint16_t ccr0, uint16_t psc);
void TimA1_Init(uint16_t ccr0, uint16_t psc);
void TimA2_Init(uint16_t ccr0, uint16_t psc);

extern int pwm_out;

#endif
