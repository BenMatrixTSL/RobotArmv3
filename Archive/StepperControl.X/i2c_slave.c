#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "project_config.h"
#include "i2c_slave.h"
#include "motor_control.h"
#include "as5600_sensor.h"
#include "settings_eeprom.h"
#include "adc_rc3.h"

#define I2C_SLAVE_ADDRESS       0x22
#define I2C_RX_BUFFER_SIZE      8U
#define I2C_TX_BUFFER_SIZE      24U

#define I2C_SCL_TRIS            TRISCbits.TRISC0
#define I2C_SCL_ANSEL           ANSELCbits.ANSC0
#define I2C_SDA_TRIS            TRISCbits.TRISC1
#define I2C_SDA_ANSEL           ANSELCbits.ANSC1

static volatile uint8_t rx_buffer[I2C_RX_BUFFER_SIZE];
static volatile uint8_t rx_index = 0;
static volatile bool command_ready = false;
static volatile uint8_t last_processed_index = 0;  // Track what we've already processed
static volatile uint8_t tx_buffer[I2C_TX_BUFFER_SIZE];
static volatile uint8_t tx_length = 0;
static volatile uint8_t tx_index = 0;
static volatile bool force_settings_packet = false;
static volatile bool force_adc_packet = false;

static void update_status_packet(void);
static void update_settings_packet(void);
static void update_adc_packet(void);

/**
 * Initializes the I2C slave communication module.
 * Configures pins RC0 (SCL) and RC1 (SDA) using PPS, sets up MSSP1,
 * and enables interrupts for I2C communication.
 * The slave address is set to I2C_SLAVE_ADDRESS (0x22).
 * Call this once at system startup.
 */
void I2CSlave_Init(void)
{
    I2C_SCL_ANSEL = 0;
    I2C_SDA_ANSEL = 0;
    I2C_SCL_TRIS = 1;
    I2C_SDA_TRIS = 1;

    SSP1CLKPPS = 0x10;   // RC0
    SSP1DATPPS = 0x11;   // RC1
    RC0PPS = 0x18;       // SCL1
    RC1PPS = 0x19;       // SDA1

    SSP1STAT = 0x80;
    SSP1ADD = (uint8_t)(I2C_SLAVE_ADDRESS << 1);
    SSP1CON1 = 0x36;
    SSP1CON2 = 0x01;
    SSP1CON3 = 0x00;
    SSP1BUF = 0x00;

    PIR1bits.SSP1IF = 0;
    PIE1bits.SSP1IE = 1;
    // Note: PIC16F18326 has only one I2C module (SSP1)
    // Bus collision is handled through SSP1IF, no separate BCL interrupt
}

/**
 * I2C slave interrupt service routine.
 * Handles incoming I2C data from the master, processes read/write requests,
 * and manages the transmit/receive buffers.
 * Do not call this function directly - it is handled by the interrupt system.
 */
void I2CSlave_ISR(void)
{
    // Verify this is actually an I2C interrupt (safety check)
    if (PIR1bits.SSP1IF == 0)
    {
        return;
    }

    // Read the I2C buffer immediately to avoid losing data
    uint8_t incoming = SSP1BUF;

    // Check for bus collision (BCL) - on PIC16F18326, this is detected through SSP1IF
    if (SSP1CON1bits.SSPOV == 1)
    {
        SSP1CON1bits.SSPOV = 0;
    }

    if (SSP1CON1bits.WCOL == 1)
    {
        SSP1CON1bits.WCOL = 0;
    }
    
    // Bus collision handling: On PIC16F18326, bus collisions are detected
    // through the SSP1IF interrupt. Error flags (SSPOV, WCOL) are cleared above.

    if (SSP1STATbits.R_nW == 0 && SSP1STATbits.D_nA == 0)
    {
        // Address match - start of new write transaction
        rx_index = 0;
        last_processed_index = 0;
        command_ready = false;  // Clear any previous command
    }
    else if (SSP1STATbits.R_nW == 0 && SSP1STATbits.D_nA == 1)
    {
        // Data byte received during write transaction
        // Always accept data - don't block I2C transactions
        if (rx_index < I2C_RX_BUFFER_SIZE)
        {
            rx_buffer[rx_index++] = incoming;
            // Set command_ready if we have new data that hasn't been processed
            if (rx_index > last_processed_index)
            {
                command_ready = true;
            }
        }
    }
    else if (SSP1STATbits.R_nW == 1 && SSP1STATbits.D_nA == 0)
    {
        if (force_settings_packet)
        {
            update_settings_packet();
            force_settings_packet = false;
        }
        else if (force_adc_packet)
        {
            update_adc_packet();
            force_adc_packet = false;
        }
        else
        {
            update_status_packet();
        }
        tx_index = 0;
        if (tx_length > 0U)
        {
            SSP1BUF = tx_buffer[tx_index++];
        }
        else
        {
            SSP1BUF = 0x00;
        }
    }
    else if (SSP1STATbits.R_nW == 1 && SSP1STATbits.D_nA == 1)
    {
        if (tx_index < tx_length)
        {
            SSP1BUF = tx_buffer[tx_index++];
        }
        else
        {
            SSP1BUF = 0x00;
        }
    }

    SSP1CON1bits.CKP = 1;
    PIR1bits.SSP1IF = 0;
}

/**
 * I2C bus collision interrupt service routine.
 * Handles bus collision errors and clears error flags.
 * Note: On PIC16F18326, bus collision is detected through SSP1IF,
 * so this function is called from the main I2C ISR when needed.
 * Do not call this function directly - it is handled by the interrupt system.
 */
void I2CSlave_BusCollisionISR(void)
{
    // Bus collision is handled in the main I2C ISR through SSP1IF
    // Clear any error flags that may have been set
    SSP1CON1bits.CKP = 1;
    SSP1CON1bits.SSPOV = 0;
    SSP1CON1bits.WCOL = 0;
}

/**
 * Checks if a new I2C command is ready to be processed.
 * Call this in the main loop to check for incoming commands.
 *
 * @return true if a command is ready, false otherwise
 */
bool I2CSlave_CommandReady(void)
{
    return command_ready;
}

/**
 * Reads the received I2C command from the buffer.
 * Copies the command data to the provided buffer and clears the ready flag.
 *
 * @param buffer Pointer to buffer where command data will be copied
 * @param max_length Maximum number of bytes to copy
 * @return Number of bytes actually copied (0 if no command or invalid parameters)
 */
uint8_t I2CSlave_ReadCommand(uint8_t *buffer, uint8_t max_length)
{
    if (command_ready == false || buffer == NULL || max_length == 0U)
    {
        return 0;
    }

    // Disable interrupts briefly to read rx_index atomically and copy data
    INTCONbits.GIE = 0;
    uint8_t length = rx_index;
    INTCONbits.GIE = 1;
    
    if (length > max_length)
    {
        length = max_length;
    }

    // Copy the entire buffer (simple approach - process complete commands)
    for (uint8_t i = 0; i < length; i++)
    {
        buffer[i] = rx_buffer[i];
    }

    // Clear the command - new address match will reset rx_index
    // Don't reset rx_index here - let the next address match do it
    command_ready = false;
    last_processed_index = length;
    
    return length;
}

/**
 * Requests that the settings packet be sent on the next I2C read.
 * The settings packet includes AS5600 address, offset, and kinematics constants.
 */
void I2CSlave_RequestSettingsPacket(void)
{
    force_settings_packet = true;
}

/**
 * Requests that the ADC packet be sent on the next I2C read.
 * The ADC packet includes the raw ADC value and voltage in millivolts from RC3.
 */
void I2CSlave_RequestAdcPacket(void)
{
    force_adc_packet = true;
}

/**
 * Updates the status packet buffer with current system status.
 * Includes motor state, step position, and AS5600 angle reading.
 * This packet is sent automatically on I2C reads unless a special packet is requested.
 */
static void update_status_packet(void)
{
    int32_t steps = MotorControl_GetCurrentSteps();
    float sensor_deg = As5600_GetCachedDegrees();
    
    // Convert to signed 16-bit value in hundredths of degrees
    int16_t sensor_deg_x100 = (int16_t)(sensor_deg * 100.0f);

    tx_buffer[0] = (uint8_t)(MotorControl_IsMoving() ? 1 : 0);
    tx_buffer[1] = (uint8_t)(MotorControl_IsStallDetected() ? 1 : 0);
    tx_buffer[2] = (uint8_t)(steps & 0xFF);
    tx_buffer[3] = (uint8_t)((steps >> 8) & 0xFF);
    tx_buffer[4] = (uint8_t)((steps >> 16) & 0xFF);
    tx_buffer[5] = (uint8_t)((steps >> 24) & 0xFF);
    tx_buffer[6] = (uint8_t)(sensor_deg_x100 & 0xFF);
    tx_buffer[7] = (uint8_t)((sensor_deg_x100 >> 8) & 0xFF);
    tx_length = 8;
}

/**
 * Updates the settings packet buffer with EEPROM-stored settings.
 * Includes AS5600 I2C address, offset, and all kinematics constants.
 * This packet is sent when command 0x06 is received.
 */
static void update_settings_packet(void)
{
    uint8_t address = Settings_GetSensorAddress();
    int16_t offset = Settings_GetSensorOffsetX100();
    int32_t d = Settings_GetKinematicsD();
    int32_t a = Settings_GetKinematicsA();
    int32_t theta = Settings_GetKinematicsTheta();
    int32_t alpha = Settings_GetKinematicsAlpha();

    tx_buffer[0] = 0xAA; // marker
    tx_buffer[1] = address;
    tx_buffer[2] = (uint8_t)(offset & 0xFF);
    tx_buffer[3] = (uint8_t)((offset >> 8) & 0xFF);
    
    // Kinematics d (4 bytes, little-endian)
    tx_buffer[4] = (uint8_t)(d & 0xFF);
    tx_buffer[5] = (uint8_t)((d >> 8) & 0xFF);
    tx_buffer[6] = (uint8_t)((d >> 16) & 0xFF);
    tx_buffer[7] = (uint8_t)((d >> 24) & 0xFF);
    
    // Kinematics a (4 bytes, little-endian)
    tx_buffer[8] = (uint8_t)(a & 0xFF);
    tx_buffer[9] = (uint8_t)((a >> 8) & 0xFF);
    tx_buffer[10] = (uint8_t)((a >> 16) & 0xFF);
    tx_buffer[11] = (uint8_t)((a >> 24) & 0xFF);
    
    // Kinematics theta (4 bytes, little-endian)
    tx_buffer[12] = (uint8_t)(theta & 0xFF);
    tx_buffer[13] = (uint8_t)((theta >> 8) & 0xFF);
    tx_buffer[14] = (uint8_t)((theta >> 16) & 0xFF);
    tx_buffer[15] = (uint8_t)((theta >> 24) & 0xFF);
    
    // Kinematics alpha (4 bytes, little-endian)
    tx_buffer[16] = (uint8_t)(alpha & 0xFF);
    tx_buffer[17] = (uint8_t)((alpha >> 8) & 0xFF);
    tx_buffer[18] = (uint8_t)((alpha >> 16) & 0xFF);
    tx_buffer[19] = (uint8_t)((alpha >> 24) & 0xFF);
    
    tx_length = 20;
}

/**
 * Updates the ADC packet buffer with the latest ADC reading.
 * Includes raw ADC counts and calculated voltage in millivolts.
 * This packet is sent when command 0x07 is received.
 */
static void update_adc_packet(void)
{
    uint16_t raw = AdcRc3_GetRaw();
    uint16_t mv = AdcRc3_GetMillivolts();

    tx_buffer[0] = 0xAB;
    tx_buffer[1] = (uint8_t)(raw & 0xFF);
    tx_buffer[2] = (uint8_t)(raw >> 8);
    tx_buffer[3] = (uint8_t)(mv & 0xFF);
    tx_buffer[4] = (uint8_t)(mv >> 8);
    tx_length = 5;
}

