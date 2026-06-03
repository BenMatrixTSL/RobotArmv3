#include <xc.h>
#include <stdint.h>
#include "project_config.h"
#include "servo_pwm.h"

#define SERVO_LAT       LATCbits.LATC4
#define SERVO_TRIS      TRISCbits.TRISC4
#define SERVO_ANSEL     ANSELCbits.ANSC4

#define SERVO_FRAME_US          20000U
#define SERVO_MIN_US            1000U
#define SERVO_MAX_US            2500U

#define TMR1_PRESCALE_BITS      0b11      // 1:8 prescaler -> 1 tick = 1 us

static volatile uint16_t servo_pulse_ticks = 1500U;
static volatile uint8_t servo_enabled = 0;

static uint16_t clamp_pulse(uint16_t microseconds);
static void update_ccp2_match(uint16_t ticks);
static void configure_timer_modules(void);

/**
 * Initializes the servo PWM module.
 * Configures Timer1 and CCP modules for 50Hz servo control.
 * Enables the servo output on pin RC4.
 * Call this once at system startup.
 */
void ServoPwm_Init(void)
{
    configure_timer_modules();
    ServoPwm_Enable();
}

/**
 * Sets the servo pulse width directly in microseconds.
 * The pulse width is clamped between SERVO_MIN_US and SERVO_MAX_US.
 *
 * @param microseconds Pulse width in microseconds (1000-2500 typical)
 */
void ServoPwm_SetPulseMicroseconds(uint16_t microseconds)
{
    servo_pulse_ticks = clamp_pulse(microseconds);
}

/**
 * Sets the servo angle in degrees.
 * Automatically enables the servo if it was disabled.
 * Converts degrees (0-180) to the appropriate pulse width.
 *
 * @param angle_degrees Servo angle in degrees (0-180, values above 180 are clamped)
 */
void ServoPwm_SetAngleDegrees(uint8_t angle_degrees)
{
    ServoPwm_Enable();

    if (angle_degrees > 180U)
    {
        angle_degrees = 180U;
    }

    uint16_t span = SERVO_MAX_US - SERVO_MIN_US;
    uint32_t scaled = SERVO_MIN_US;
    scaled += ((uint32_t)span * angle_degrees) / 180U;
    ServoPwm_SetPulseMicroseconds((uint16_t)scaled);
}

/**
 * CCP1 interrupt service routine for servo control.
 * Called when Timer1 reaches the 20ms period (50Hz frame).
 * Sets the servo output pin high and schedules CCP2 for the pulse width.
 * Do not call this function directly - it is handled by the interrupt system.
 */
void ServoPwm_Ccp1Isr(void)
{
    if (servo_enabled == 0)
    {
        PIR4bits.CCP1IF = 0;
        return;
    }

    PIR4bits.CCP1IF = 0;
    SERVO_LAT = 1;
    update_ccp2_match(servo_pulse_ticks);
}

/**
 * CCP2 interrupt service routine for servo control.
 * Called when the pulse width time is reached.
 * Sets the servo output pin low to end the pulse.
 * Do not call this function directly - it is handled by the interrupt system.
 */
void ServoPwm_Ccp2Isr(void)
{
    if (servo_enabled == 0)
    {
        PIR4bits.CCP2IF = 0;
        return;
    }

    PIR4bits.CCP2IF = 0;
    SERVO_LAT = 0;
}

/**
 * Enables the servo PWM output.
 * Configures the pin, starts Timer1, and enables CCP interrupts.
 * The servo will start generating pulses immediately.
 */
void ServoPwm_Enable(void)
{
    if (servo_enabled)
    {
        return;
    }

    SERVO_ANSEL = 0;
    SERVO_TRIS = 0;
    SERVO_LAT = 0;

    PIR4bits.CCP1IF = 0;
    PIR4bits.CCP2IF = 0;
    PIE4bits.CCP1IE = 1;
    PIE4bits.CCP2IE = 1;
    TMR1H = 0;
    TMR1L = 0;
    T1CONbits.TMR1ON = 1;
    servo_enabled = 1;
}

/**
 * Disables the servo PWM output.
 * Stops Timer1 and disables interrupts.
 * The pin remains configured but no pulses are generated.
 */
void ServoPwm_Disable(void)
{
    if (servo_enabled == 0)
    {
        return;
    }

    servo_enabled = 0;
    T1CONbits.TMR1ON = 0;
    PIE4bits.CCP1IE = 0;
    PIE4bits.CCP2IE = 0;
    SERVO_ANSEL = 0;
    SERVO_TRIS = 0;
    SERVO_LAT = 0;
}

/**
 * Sets RC4 as a relay output instead of servo PWM.
 * Disables the servo PWM and drives the pin as a simple digital output.
 *
 * @param level 0 for low (relay off), non-zero for high (relay on)
 */
void ServoPwm_SetRelayState(uint8_t level)
{
    ServoPwm_Disable();
    SERVO_TRIS = 0;
    SERVO_LAT = (level != 0) ? 1 : 0;
}

/**
 * Clamps the pulse width to valid servo range.
 *
 * @param microseconds Pulse width in microseconds
 * @return Clamped pulse width (SERVO_MIN_US to SERVO_MAX_US)
 */
static uint16_t clamp_pulse(uint16_t microseconds)
{
    if (microseconds < SERVO_MIN_US)
    {
        return SERVO_MIN_US;
    }

    if (microseconds > SERVO_MAX_US)
    {
        return SERVO_MAX_US;
    }

    return microseconds;
}

/**
 * Updates the CCP2 match value for pulse width timing.
 *
 * @param ticks Timer ticks for the pulse width
 */
static void update_ccp2_match(uint16_t ticks)
{
    CCPR2H = (uint8_t)(ticks >> 8);
    CCPR2L = (uint8_t)(ticks & 0xFF);
}

/**
 * Configures Timer1 and CCP modules for servo PWM generation.
 * Sets up Timer1 with Fosc/4 clock and 1:8 prescaler (1us per tick).
 * Configures CCP1 for 20ms period and CCP2 for pulse width.
 */
static void configure_timer_modules(void)
{
    T1CONbits.TMR1CS = 0b00;     // Fosc/4
    T1CONbits.T1CKPS = TMR1_PRESCALE_BITS;
    // Note: Timer1 is always 16-bit on PIC16F18326 (no RD16 bit needed)

    CCP1CONbits.CCP1MODE = 0b1011;   // Compare, special event trigger (reset TMR1)
    CCPR1H = (uint8_t)(SERVO_FRAME_US >> 8);
    CCPR1L = (uint8_t)(SERVO_FRAME_US & 0xFF);

    CCP2CONbits.CCP2MODE = 0b1010;   // Compare, software interrupt
    update_ccp2_match(servo_pulse_ticks);
}

