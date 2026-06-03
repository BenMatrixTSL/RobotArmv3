#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "project_config.h"
#include "as5600_sensor.h"

#define AS5600_ANGLE_MSB     0x0E

#define AS5600_SDA_LAT       LATAbits.LATA4
#define AS5600_SDA_TRIS      TRISAbits.TRISA4
#define AS5600_SDA_ANSEL     ANSELAbits.ANSA4

#define AS5600_SCL_LAT       LATAbits.LATA5
#define AS5600_SCL_TRIS      TRISAbits.TRISA5
#define AS5600_SCL_ANSEL     ANSELAbits.ANSA5

static volatile uint16_t cached_angle = 0;
static volatile float cached_degrees = 0.0f;
static volatile uint8_t current_address = 0x36;
static volatile int16_t current_offset_x100 = 0;

static void i2c_delay(void);
static void sda_release(void);
static void sda_drive_low(void);
static void scl_release(void);
static void scl_drive_low(void);
static void i2c_start(void);
static void i2c_stop(void);
static bool i2c_write_byte(uint8_t value);
static uint8_t i2c_read_byte(bool ack);

/**
 * Initializes the AS5600 angle sensor module.
 * Configures the I2C pins (RA4 for SDA, RA5 for SCL) for bit-banged communication.
 * Stores the I2C address and offset for use in readings.
 *
 * @param i2c_address 7-bit I2C address of the AS5600 sensor
 * @param offset_x100 Angle offset in hundredths of a degree
 */
void As5600_Init(uint8_t i2c_address, int16_t offset_x100)
{
    current_address = i2c_address;
    current_offset_x100 = offset_x100;

    AS5600_SDA_ANSEL = 0;
    AS5600_SCL_ANSEL = 0;

    AS5600_SDA_LAT = 0;
    AS5600_SCL_LAT = 0;

    sda_release();
    scl_release();

    cached_angle = 0;
}

/**
 * Service function for the AS5600 sensor.
 * Polls the sensor for a new angle reading, converts it to degrees between -180 and 180,
 * applies the offset, and caches the result.
 * Call this regularly in the main loop.
 */
void As5600_Service(void)
{
    uint16_t angle;
    float tempAngle;
    if (As5600_ReadRawAngle(&angle))
    {
        cached_angle = angle;
        tempAngle = ((float)angle * 360.0f) / 4096.0f;
        tempAngle += (float)current_offset_x100 / 100.0f;

        if (tempAngle >= 180.0f)
        {
            tempAngle -= 360.0f;
        }
        cached_degrees = tempAngle;
    }
}

/**
 * Reads the raw angle value from the AS5600 sensor.
 * Performs a bit-banged I2C read of the ANGLE register.
 *
 * @param angle Pointer to store the raw angle value (0-4095)
 * @return true if read was successful, false on I2C error
 */
bool As5600_ReadRawAngle(uint16_t *angle)
{
    if (angle == NULL)
    {
        return false;
    }

    bool ack;
    uint8_t msb = 0;
    uint8_t lsb = 0;

    i2c_start();
    ack = i2c_write_byte((uint8_t)(current_address << 1)); // write
    if (!ack)
    {
        i2c_stop();
        return false;
    }

    ack = i2c_write_byte(AS5600_ANGLE_MSB);
    if (!ack)
    {
        i2c_stop();
        return false;
    }

    i2c_start();
    ack = i2c_write_byte((uint8_t)((current_address << 1) | 0x01)); // read
    if (!ack)
    {
        i2c_stop();
        return false;
    }

    msb = i2c_read_byte(true);
    lsb = i2c_read_byte(false);

    i2c_stop();

    *angle = ((uint16_t)msb << 8) | lsb;
    return true;
}


/**
 * Gets the most recently cached raw angle value.
 * This is the value from the last successful As5600_Service() call.
 *
 * @return Raw angle value (0-4095)
 */
uint16_t As5600_GetCachedAngle(void)
{
    return cached_angle;
}

/**
 * Gets the most recently cached angle in degrees.
 * This is the value from the last successful As5600_Service() call.
 * The offset has been applied and the value is normalized to 0-360 degrees.
 *
 * @return Angle in degrees (0.0-360.0)
 */
float As5600_GetCachedDegrees(void)
{
    return cached_degrees;
}

/**
 * I2C timing delay function.
 * Provides a 5 microsecond delay for bit-banged I2C timing.
 */
static void i2c_delay(void)
{
    __delay_us(5);
}

/**
 * Releases the SDA line (sets as input for open-drain high).
 */
static void sda_release(void)
{
    AS5600_SDA_TRIS = 1;
}

/**
 * Drives the SDA line low (sets as output and drives low).
 */
static void sda_drive_low(void)
{
    AS5600_SDA_LAT = 0;
    AS5600_SDA_TRIS = 0;
}

/**
 * Releases the SCL line (sets as input for open-drain high).
 */
static void scl_release(void)
{
    AS5600_SCL_TRIS = 1;
}

/**
 * Drives the SCL line low (sets as output and drives low).
 */
static void scl_drive_low(void)
{
    AS5600_SCL_LAT = 0;
    AS5600_SCL_TRIS = 0;
}

/**
 * Generates an I2C start condition.
 * SDA goes low while SCL is high.
 */
static void i2c_start(void)
{
    sda_release();
    scl_release();
    i2c_delay();
    sda_drive_low();
    i2c_delay();
    scl_drive_low();
}

/**
 * Generates an I2C stop condition.
 * SDA goes high while SCL is high.
 */
static void i2c_stop(void)
{
    sda_drive_low();
    i2c_delay();
    scl_release();
    i2c_delay();
    sda_release();
    i2c_delay();
}

/**
 * Writes a byte over the bit-banged I2C bus.
 *
 * @param value Byte value to write
 * @return true if ACK was received, false if NACK
 */
static bool i2c_write_byte(uint8_t value)
{
    for (uint8_t mask = 0x80; mask != 0; mask >>= 1)
    {
        if (value & mask)
        {
            sda_release();
        }
        else
        {
            sda_drive_low();
        }

        scl_release();
        i2c_delay();
        scl_drive_low();
    }

    sda_release();
    scl_release();
    i2c_delay();
    bool ack = (PORTAbits.RA4 == 0);
    scl_drive_low();
    return ack;
}

/**
 * Reads a byte from the bit-banged I2C bus.
 *
 * @param ack true to send ACK after reading, false to send NACK
 * @return The byte value read
 */
static uint8_t i2c_read_byte(bool ack)
{
    uint8_t value = 0;

    sda_release();
    for (uint8_t i = 0; i < 8; i++)
    {
        value <<= 1;
        scl_release();
        i2c_delay();
        if (PORTAbits.RA4)
        {
            value |= 0x01;
        }
        scl_drive_low();
        i2c_delay();
    }

    if (ack)
    {
        sda_drive_low();
    }
    else
    {
        sda_release();
    }

    scl_release();
    i2c_delay();
    scl_drive_low();
    sda_release();

    return value;
}

