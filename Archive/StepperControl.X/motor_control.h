#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

void MotorControl_Init(void);
void MotorControl_TimerISR(void);
void MotorControl_StartMoveToAngle(float angle_degrees);
void MotorControl_StopMotion(void);
bool MotorControl_IsMoving(void);
bool MotorControl_IsStallDetected(void);
void MotorControl_ClearStallFlag(void);
int32_t MotorControl_GetCurrentSteps(void);
bool MotorControl_IsDirectionClockwise(void);

#endif



