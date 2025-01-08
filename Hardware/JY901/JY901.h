#ifndef __JY901_H
#define __JY901_H
#include "headfile.h"

void JYJudge(uint8_t *Recv_Data,uint8_t *Sendbuffer);
void JY901_value(uint8_t *my_array);
extern volatile uint8_t JY_flag;
extern float JY901_x,JY901_y,JY901_z;

#endif

