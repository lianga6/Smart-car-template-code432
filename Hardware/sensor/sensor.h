#ifndef __SENSOR_H
#define __SENSOR_H

void gray_Init(void);
unsigned char gray_Read(void);
unsigned char gray_separate_read(unsigned char num);
int gray_get_bias(void);

#endif

