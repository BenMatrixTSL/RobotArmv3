#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "settings_eeprom.h"

#define EEPROM_ADDR_SENSOR_ADDRESS   0x00
#define EEPROM_ADDR_SENSOR_OFFSET    0x01  // 2 bytes (little-endian)
#define EEPROM_MAGIC_ADDRESS         0x03
#define EEPROM_ADDR_KINEMATICS_D     0x04  // 4 bytes (little-endian, hundredths of mm)
#define EEPROM_ADDR_KINEMATICS_A     0x08  // 4 bytes (little-endian, hundredths of mm)
#define EEPROM_ADDR_KINEMATICS_THETA 0x0C  // 4 bytes (little-endian, hundredths of degree)
#define EEPROM_ADDR_KINEMATICS_ALPHA 0x10  // 4 bytes (little-endian, hundredths of degree)
#define EEPROM_MAGIC_VALUE           0xA5

static uint8_t sensor_address = 0x36;
static int16_t sensor_offset_x100 = 0;
static int32_t kinematics_d_x100 = 0;      // d in hundredths of mm
static int32_t kinematics_a_x100 = 0;      // a in hundredths of mm
static int32_t kinematics_theta_x100 = 0;   // theta in hundredths of degree
static int32_t kinematics_alpha_x100 = 0;  // alpha in hundredths of degree

static void eeprom_write_byte(uint8_t address, uint8_t value);
static uint8_t eeprom_read_byte(uint8_t address);

/**
 * Loads all settings from EEPROM into memory.
 * If the magic byte is invalid (first boot), saves default values.
 * Call this once at system startup before using any settings functions.
 */
void Settings_Load(void)
{
    uint8_t magic = eeprom_read_byte(EEPROM_MAGIC_ADDRESS);

    if (magic != EEPROM_MAGIC_VALUE)
    {
        Settings_Save();
        return;
    }

    sensor_address = eeprom_read_byte(EEPROM_ADDR_SENSOR_ADDRESS);
    if (sensor_address == 0xFF || sensor_address == 0x00)
    {
        sensor_address = 0x36;
    }

    uint8_t offset_l = eeprom_read_byte(EEPROM_ADDR_SENSOR_OFFSET);
    uint8_t offset_h = eeprom_read_byte(EEPROM_ADDR_SENSOR_OFFSET + 1);
    sensor_offset_x100 = (int16_t)((offset_h << 8) | offset_l);

    // Load kinematics constants (4 bytes each, little-endian)
    kinematics_d_x100 = (int32_t)((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_D) |
                                  ((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_D + 1) << 8) |
                                  ((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_D + 2) << 16) |
                                  ((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_D + 3) << 24));
    
    kinematics_a_x100 = (int32_t)((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_A) |
                                  ((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_A + 1) << 8) |
                                  ((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_A + 2) << 16) |
                                  ((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_A + 3) << 24));
    
    kinematics_theta_x100 = (int32_t)((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_THETA) |
                                       ((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_THETA + 1) << 8) |
                                       ((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_THETA + 2) << 16) |
                                       ((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_THETA + 3) << 24));
    
    kinematics_alpha_x100 = (int32_t)((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_ALPHA) |
                                       ((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_ALPHA + 1) << 8) |
                                       ((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_ALPHA + 2) << 16) |
                                       ((uint32_t)eeprom_read_byte(EEPROM_ADDR_KINEMATICS_ALPHA + 3) << 24));
}

/**
 * Saves all current settings to EEPROM.
 * Writes the magic byte, sensor settings, and kinematics constants.
 * This operation may take several milliseconds to complete.
 *
 * @return true if save was successful
 */
bool Settings_Save(void)
{
    eeprom_write_byte(EEPROM_ADDR_SENSOR_ADDRESS, sensor_address);
    eeprom_write_byte(EEPROM_ADDR_SENSOR_OFFSET, (uint8_t)(sensor_offset_x100 & 0xFF));
    eeprom_write_byte(EEPROM_ADDR_SENSOR_OFFSET + 1, (uint8_t)((sensor_offset_x100 >> 8) & 0xFF));
    
    // Save kinematics constants (4 bytes each, little-endian)
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_D, (uint8_t)(kinematics_d_x100 & 0xFF));
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_D + 1, (uint8_t)((kinematics_d_x100 >> 8) & 0xFF));
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_D + 2, (uint8_t)((kinematics_d_x100 >> 16) & 0xFF));
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_D + 3, (uint8_t)((kinematics_d_x100 >> 24) & 0xFF));
    
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_A, (uint8_t)(kinematics_a_x100 & 0xFF));
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_A + 1, (uint8_t)((kinematics_a_x100 >> 8) & 0xFF));
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_A + 2, (uint8_t)((kinematics_a_x100 >> 16) & 0xFF));
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_A + 3, (uint8_t)((kinematics_a_x100 >> 24) & 0xFF));
    
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_THETA, (uint8_t)(kinematics_theta_x100 & 0xFF));
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_THETA + 1, (uint8_t)((kinematics_theta_x100 >> 8) & 0xFF));
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_THETA + 2, (uint8_t)((kinematics_theta_x100 >> 16) & 0xFF));
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_THETA + 3, (uint8_t)((kinematics_theta_x100 >> 24) & 0xFF));
    
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_ALPHA, (uint8_t)(kinematics_alpha_x100 & 0xFF));
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_ALPHA + 1, (uint8_t)((kinematics_alpha_x100 >> 8) & 0xFF));
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_ALPHA + 2, (uint8_t)((kinematics_alpha_x100 >> 16) & 0xFF));
    eeprom_write_byte(EEPROM_ADDR_KINEMATICS_ALPHA + 3, (uint8_t)((kinematics_alpha_x100 >> 24) & 0xFF));
    
    eeprom_write_byte(EEPROM_MAGIC_ADDRESS, EEPROM_MAGIC_VALUE);
    return true;
}

/**
 * Gets the stored AS5600 sensor I2C address.
 *
 * @return 7-bit I2C address (default 0x36)
 */
uint8_t Settings_GetSensorAddress(void)
{
    return sensor_address;
}

/**
 * Sets the AS5600 sensor I2C address.
 * The address is validated to be in the valid I2C range (0x08-0x77).
 *
 * @param address 7-bit I2C address (0x08-0x77)
 */
void Settings_SetSensorAddress(uint8_t address)
{
    if (address < 0x08 || address > 0x77)
    {
        return;
    }

    sensor_address = address;
}

/**
 * Gets the stored AS5600 sensor angle offset.
 *
 * @return Offset in hundredths of a degree
 */
int16_t Settings_GetSensorOffsetX100(void)
{
    return sensor_offset_x100;
}

/**
 * Sets the AS5600 sensor angle offset.
 *
 * @param offset Offset in hundredths of a degree (signed)
 */
void Settings_SetSensorOffsetX100(int16_t offset)
{
    sensor_offset_x100 = offset;
}

/**
 * Gets the stored kinematics parameter d (link offset).
 *
 * @return Link offset in hundredths of mm
 */
int32_t Settings_GetKinematicsD(void)
{
    return kinematics_d_x100;
}

/**
 * Sets the kinematics parameter d (link offset).
 *
 * @param d_x100 Link offset in hundredths of mm (signed)
 */
void Settings_SetKinematicsD(int32_t d_x100)
{
    kinematics_d_x100 = d_x100;
}

/**
 * Gets the stored kinematics parameter a (link length).
 *
 * @return Link length in hundredths of mm
 */
int32_t Settings_GetKinematicsA(void)
{
    return kinematics_a_x100;
}

/**
 * Sets the kinematics parameter a (link length).
 *
 * @param a_x100 Link length in hundredths of mm (signed)
 */
void Settings_SetKinematicsA(int32_t a_x100)
{
    kinematics_a_x100 = a_x100;
}

/**
 * Gets the stored kinematics parameter theta (joint angle).
 *
 * @return Joint angle in hundredths of degree
 */
int32_t Settings_GetKinematicsTheta(void)
{
    return kinematics_theta_x100;
}

/**
 * Sets the kinematics parameter theta (joint angle).
 *
 * @param theta_x100 Joint angle in hundredths of degree (signed)
 */
void Settings_SetKinematicsTheta(int32_t theta_x100)
{
    kinematics_theta_x100 = theta_x100;
}

/**
 * Gets the stored kinematics parameter alpha (link twist).
 *
 * @return Link twist in hundredths of degree
 */
int32_t Settings_GetKinematicsAlpha(void)
{
    return kinematics_alpha_x100;
}


// Call this once at startup, before Settings_Load()
void eeprom_init(void)
{
    // Make sure the NVM (EEPROM / Flash) module is enabled
    PMD0bits.NVMMD = 0;   // 0 = NVM enabled, 1 = disabled
}


/**
 * Sets the kinematics parameter alpha (link twist).
 *
 * @param alpha_x100 Link twist in hundredths of degree (signed)
 */
void Settings_SetKinematicsAlpha(int32_t alpha_x100)
{
    kinematics_alpha_x100 = alpha_x100;
}

/**
 * Writes a single byte to EEPROM.
 * This function handles the EEPROM write sequence and waits for completion.
 * Interrupts are disabled during the critical write sequence.
 *
 * @param address EEPROM address (0-255)
 * @param value Byte value to write
 */
static void eeprom_write_byte(uint8_t address, uint8_t value)
{
    // 1) Wait for any previous write to finish
    while (NVMCON1bits.WR)
    {
        ;
    }

    // 2) Select data EEPROM space (not program memory, not configuration)
    //    On PIC16F18326 this is done with NVMREGS bit:
    //    NVMREGS = 1 -> data EEPROM
    //    NVMREGS = 0 -> Flash program memory
    NVMCON1bits.NVMREGS = 1;   // data EEPROM

    // 3) Set the EEPROM address and data
    NVMADRL = address;         // low 8 bits of address (data EEPROM is 0-255)
    NVMDATL = value;           // byte to write

    // 4) Configure NVMCON1 for a data EEPROM write
    //    Typical bits (check datasheet, but this is the pattern):
    //    FREE = 0 (not erase)
    //    WREN = 1 (enable write)
    NVMCON1bits.FREE = 0;      // no erase
    NVMCON1bits.LWLO = 0;      // write full byte
    NVMCON1bits.WREN = 1;      // enable writes

    // 5) Do the unlock sequence with interrupts disabled
    INTCONbits.GIE = 0;        // global interrupt disable

    NVMCON2 = 0x55;            // required key 1
    NVMCON2 = 0xAA;            // required key 2
    NVMCON1bits.WR = 1;        // start the write

    INTCONbits.GIE = 1;        // re-enable interrupts

    // 6) Wait for write to complete
    while (NVMCON1bits.WR)
    {
        ;
    }

    // 7) Disable writes again for safety
    NVMCON1bits.WREN = 0;
}

/**
 * Reads a single byte from EEPROM.
 *
 * @param address EEPROM address (0-255)
 * @return The byte value read
 */
static uint8_t eeprom_read_byte(uint8_t address)
{
    uint8_t value;

    // 1) Wait for any previous write to finish
    while (NVMCON1bits.WR)
    {
        ;
    }

    // 2) Select data EEPROM space
    NVMCON1bits.NVMREGS = 1;   // data EEPROM

    // 3) Set the address
    NVMADRL = address;

    // 4) Start the read
    NVMCON1bits.RD = 1;

    // 5) After RD is set, the data appears in NVMDATL
    value = NVMDATL;

    return value;
}

