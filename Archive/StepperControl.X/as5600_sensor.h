#ifndef AS5600_SENSOR_H
#define AS5600_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

void As5600_Init(uint8_t i2c_address, int16_t offset_x100);
void As5600_Service(void);
bool As5600_ReadRawAngle(uint16_t *angle);
bool As5600_ReadAngleDegrees(float *degrees);
uint16_t As5600_GetCachedAngle(void);
float As5600_GetCachedDegrees(void);
int16_t As5600_GetCurrentOffset(void);

#endif

