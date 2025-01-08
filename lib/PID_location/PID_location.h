#ifndef __PID_LOCATION_H
#define __PID_LOCATION_H
#include "PID.h"
void PID_location_control(_pid *pid_location,long g_lMotorPulseSigma,unsigned char flag_bit);

#endif
