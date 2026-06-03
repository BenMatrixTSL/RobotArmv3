#ifndef ADC_RC3_H
#define ADC_RC3_H

#include <stdint.h>

void AdcRc3_Init(void);
void AdcRc3_Service(void);
uint16_t AdcRc3_GetRaw(void);
uint16_t AdcRc3_GetMillivolts(void);

#endif



