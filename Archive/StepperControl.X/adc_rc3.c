#include <xc.h>
#include <stdint.h>
#include "project_config.h"
#include "adc_rc3.h"

#define ADC_CHANNEL_RC3   0x07
#define VREF_MV           5000U
#define ADC_MAX_COUNTS    1023U

static volatile uint16_t adc_raw = 0;
static volatile uint16_t adc_mv = 0;

static uint16_t convert_to_mv(uint16_t counts);

/**
 * Initializes the ADC module for reading pin RC3.
 * Configures RC3 as an analog input and sets up the ADC with VDD/VSS references.
 * Call this once at system startup.
 */
void AdcRc3_Init(void)
{
    ANSELCbits.ANSC3 = 1;
    TRISCbits.TRISC3 = 1;

    ADCON0bits.ADON = 0;
    ADCON0bits.CHS = ADC_CHANNEL_RC3;

    ADCON1bits.ADPREF = 0;   // Vref+ = Vdd
    ADCON1bits.ADNREF = 0;   // Vref- = Vss
    ADCON1bits.ADCS = 0b110; // Fosc/64
    ADCON1bits.ADFM = 1;     // Right justified

    ADCON0bits.ADON = 1;
}

/**
 * Performs an ADC conversion on pin RC3.
 * Starts the conversion, waits for completion, and caches the result.
 * Also calculates and caches the voltage in millivolts.
 * Call this regularly in the main loop to update the ADC reading.
 */
void AdcRc3_Service(void)
{
    ADCON0bits.GO_nDONE = 1;
    while (ADCON0bits.GO_nDONE)
    {
        ;
    }

    adc_raw = ((uint16_t)ADRESH << 8) | ADRESL;
    adc_mv = convert_to_mv(adc_raw);
}

/**
 * Gets the most recently cached raw ADC value.
 * This is the value from the last AdcRc3_Service() call.
 *
 * @return Raw ADC counts (0-1023 for 10-bit ADC)
 */
uint16_t AdcRc3_GetRaw(void)
{
    return adc_raw;
}

/**
 * Gets the most recently cached ADC voltage in millivolts.
 * This is calculated from the last AdcRc3_Service() call.
 * Assumes VDD = 5V (VREF_MV).
 *
 * @return Voltage in millivolts (0-5000)
 */
uint16_t AdcRc3_GetMillivolts(void)
{
    return adc_mv;
}

/**
 * Converts ADC counts to millivolts.
 * Uses the formula: mV = (counts * VREF_MV) / ADC_MAX_COUNTS
 *
 * @param counts Raw ADC counts (0-1023)
 * @return Voltage in millivolts
 */
static uint16_t convert_to_mv(uint16_t counts)
{
    return (uint16_t)(((uint32_t)counts * VREF_MV) / ADC_MAX_COUNTS);
}

