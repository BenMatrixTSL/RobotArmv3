#ifndef SETTINGS_EEPROM_H
#define SETTINGS_EEPROM_H

#include <stdint.h>
#include <stdbool.h>

void eeprom_init(void);

void Settings_Load(void);
bool Settings_Save(void);

uint8_t Settings_GetSensorAddress(void);
void Settings_SetSensorAddress(uint8_t address);

int16_t Settings_GetSensorOffsetX100(void);
void Settings_SetSensorOffsetX100(int16_t offset);

int32_t Settings_GetKinematicsD(void);
void Settings_SetKinematicsD(int32_t d_x100);

int32_t Settings_GetKinematicsA(void);
void Settings_SetKinematicsA(int32_t a_x100);

int32_t Settings_GetKinematicsTheta(void);
void Settings_SetKinematicsTheta(int32_t theta_x100);

int32_t Settings_GetKinematicsAlpha(void);
void Settings_SetKinematicsAlpha(int32_t alpha_x100);

#endif

