#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include <stdbool.h>
#include <stdint.h>

void I2CSlave_Init(void);
void I2CSlave_ISR(void);
void I2CSlave_BusCollisionISR(void);
bool I2CSlave_CommandReady(void);
uint8_t I2CSlave_ReadCommand(uint8_t *buffer, uint8_t max_length);
void I2CSlave_RequestSettingsPacket(void);
void I2CSlave_RequestAdcPacket(void);

#endif

