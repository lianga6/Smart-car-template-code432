#ifndef __CONTROL_H
#define __CONTROL_H

void MotorDir(unsigned char DIR);
void ServoAngle(int anglevalue);
void MotorControl(unsigned char mode);
void Stop(void);
void DJPC_TRAN(float AKACK);

extern float DJPC;

#endif
