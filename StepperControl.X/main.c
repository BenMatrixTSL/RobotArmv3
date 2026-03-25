#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "project_config.h"
#include "motor_control.h"
#include "led_ripple.h"
#include "i2c_slave.h"
#include "servo_pwm.h"
#include "as5600_sensor.h"
#include "settings_eeprom.h"
#include "adc_rc3.h"

// CONFIGURATION BITS ---------------------------------------------------------
// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config WDTE = OFF        // Watchdog Timer Enable bits (WDT enabled, SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = HIGH      // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.7V)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (High Voltage on MCLR/VPP must be used for programming.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

static void system_initialize(void);
static void oscillator_initialize(void);
static void handle_i2c_command(void);

/**
 * Main interrupt service routine.
 * Dispatches interrupts to the appropriate module handlers.
 * Do not call this function directly - it is handled by the interrupt system.
 */
void __interrupt() system_isr(void)
{
    // Check I2C FIRST - it's time-critical and must not be delayed by other interrupts
    // I2C slave needs immediate response to avoid bus timeouts
    if (PIR1bits.SSP1IF)
    {
        I2CSlave_ISR();
    }
    
    // Timer2 for motor stepping (checked after I2C to avoid blocking I2C)
    if (PIR1bits.TMR2IF)
    {
        MotorControl_TimerISR();
    }
    
    // Servo PWM interrupts (lower priority)
    if (PIR4bits.CCP1IF)
    {
        ServoPwm_Ccp1Isr();
    }
    
    if (PIR4bits.CCP2IF)
    {
        ServoPwm_Ccp2Isr();
    }
}

/**
 * Main application entry point.
 * Initializes all system modules and enters the main loop.
 * The main loop services sensors, handles I2C commands, updates LEDs,
 * and monitors for stall conditions.
 */
int main(void)
{
    system_initialize();

    while (1)
    {
        // Check for I2C commands first (highest priority)
        if (I2CSlave_CommandReady())
        {
            handle_i2c_command();
        }

        As5600_Service();
        AdcRc3_Service();

        // Check for I2C commands again after services (in case one arrived during service calls)
        if (I2CSlave_CommandReady())
        {
            handle_i2c_command();
        }

        LedRipple_Service();

        if (MotorControl_IsStallDetected())
        {
            MotorControl_StopMotion();
            __delay_ms(500);
            MotorControl_ClearStallFlag();
            continue;
        }

        // Reset the watchdog timer to prevent device reset
        __asm("CLRWDT");
    }
}

/**
 * Initializes all system modules.
 * Sets up the oscillator, loads EEPROM settings, initializes all peripherals,
 * and enables interrupts.
 * This is called once at the start of main().
 */
static void system_initialize(void)
{
    eeprom_init();
    Settings_Load();    
    MotorControl_Init();
    I2CSlave_Init();
    LedRipple_Init();
    ServoPwm_Init();
    As5600_Init(Settings_GetSensorAddress(), Settings_GetSensorOffsetX100());
    AdcRc3_Init();

    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
}


/**
 * Processes incoming I2C commands.
 * Reads the command buffer and executes the appropriate action based on the command byte.
 * Supported commands:
 *   0x01: Move motor to angle
 *   0x02: Stop motor
 *   0x03: Set servo angle
 *   0x04: Set AS5600 I2C address
 *   0x05: Set AS5600 offset
 *   0x06: Request settings packet
 *   0x07: Request ADC packet
 *   0x08: Set relay state
 *   0x09-0x0C: Set kinematics parameters
 */
static void handle_i2c_command(void)
{
    uint8_t buffer[8];
    uint8_t length = I2CSlave_ReadCommand(buffer, sizeof(buffer));

    if (length == 0U)
    {
        return;
    }

    uint8_t command = buffer[0];

    if (command == 0x01U && length >= 3U)
    {
        // Read as signed 16-bit value to support negative angles (-160 to 160 degrees)
        int16_t angle_x100 = (int16_t)(buffer[1] | ((uint16_t)buffer[2] << 8));
        float new_angle = (float)angle_x100 / 100.0f;
        MotorControl_StartMoveToAngle(new_angle);
    }
    else if (command == 0x02U)
    {
        MotorControl_StopMotion();
    }
    else if (command == 0x03U && length >= 2U)
    {
        ServoPwm_SetAngleDegrees(buffer[1]);
    }
    else if (command == 0x04U && length >= 2U)
    {
        Settings_SetSensorAddress(buffer[1]);
        Settings_Save();
        As5600_Init(Settings_GetSensorAddress(), Settings_GetSensorOffsetX100());
    }
    else if (command == 0x05U && length >= 3U)
    {
        int16_t offset = (int16_t)((uint16_t)buffer[2] << 8 | buffer[1]);
        Settings_SetSensorOffsetX100(offset);
        Settings_Save();
        As5600_Init(Settings_GetSensorAddress(), Settings_GetSensorOffsetX100());
        I2CSlave_RequestSettingsPacket();
    }
    else if (command == 0x06U)
    {
        I2CSlave_RequestSettingsPacket();
    }
    else if (command == 0x07U)
    {
        I2CSlave_RequestAdcPacket();
    }
    else if (command == 0x08U && length >= 2U)
    {
        ServoPwm_SetRelayState(buffer[1]);
    }
    else if (command == 0x09U && length >= 5U) // Set kinematics d
    {
        int32_t d_x100 = (int32_t)((uint32_t)buffer[1] |
                                    ((uint32_t)buffer[2] << 8) |
                                    ((uint32_t)buffer[3] << 16) |
                                    ((uint32_t)buffer[4] << 24));
        Settings_SetKinematicsD(d_x100);
        Settings_Save();
    }
    else if (command == 0x0AU && length >= 5U) // Set kinematics a
    {
        int32_t a_x100 = (int32_t)((uint32_t)buffer[1] |
                                    ((uint32_t)buffer[2] << 8) |
                                    ((uint32_t)buffer[3] << 16) |
                                    ((uint32_t)buffer[4] << 24));
        Settings_SetKinematicsA(a_x100);
        Settings_Save();
    }
    else if (command == 0x0BU && length >= 5U) // Set kinematics theta
    {
        int32_t theta_x100 = (int32_t)((uint32_t)buffer[1] |
                                        ((uint32_t)buffer[2] << 8) |
                                        ((uint32_t)buffer[3] << 16) |
                                        ((uint32_t)buffer[4] << 24));
        Settings_SetKinematicsTheta(theta_x100);
        Settings_Save();
    }
    else if (command == 0x0CU && length >= 5U) // Set kinematics alpha
    {
        int32_t alpha_x100 = (int32_t)((uint32_t)buffer[1] |
                                       ((uint32_t)buffer[2] << 8) |
                                       ((uint32_t)buffer[3] << 16) |
                                       ((uint32_t)buffer[4] << 24));
        Settings_SetKinematicsAlpha(alpha_x100);
        Settings_Save();
    }
}

