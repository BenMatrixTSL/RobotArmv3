#ifndef LED_RIPPLE_H
#define LED_RIPPLE_H

#include <stdbool.h>

void LedRipple_Init(void);
void LedRipple_HandleMotorStart(bool forward);
void LedRipple_HandleMotorStop(void);
void LedRipple_TimerTick(void);
void LedRipple_Service(void);

#endif



