#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include <stdint.h>

void ServoPwm_Init(void);
void ServoPwm_SetPulseMicroseconds(uint16_t microseconds);
void ServoPwm_SetAngleDegrees(uint8_t angle_degrees);
void ServoPwm_Enable(void);
void ServoPwm_Disable(void);
void ServoPwm_SetRelayState(uint8_t level);
void ServoPwm_Ccp1Isr(void);
void ServoPwm_Ccp2Isr(void);

#endif

