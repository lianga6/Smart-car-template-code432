#ifndef __BEEP_H
#define __BEEP_H

void BEEP_Init(void);
void BEEP_sound(void);
#define BEEP_ON()  GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5)
#define BEEP_OFF() GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5)

#endif
