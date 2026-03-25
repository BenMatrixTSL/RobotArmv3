#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "project_config.h"
#include "motor_control.h"
#include "led_ripple.h"

#define STEP_LAT            LATAbits.LATA0
#define STEP_TRIS           TRISAbits.TRISA0
#define STEP_ANSEL          ANSELAbits.ANSA0

#define DIR_LAT             LATAbits.LATA1
#define DIR_TRIS            TRISAbits.TRISA1
#define DIR_ANSEL           ANSELAbits.ANSA1

#define STALL_PORT          PORTAbits.RA2
#define STALL_TRIS          TRISAbits.TRISA2
#define STALL_ANSEL         ANSELAbits.ANSA2

#define ENABLE_LAT          LATCbits.LATC5
#define ENABLE_TRIS         TRISCbits.TRISC5
#define ENABLE_ANSEL        ANSELCbits.ANSC5

#define DRIVER_ENABLE_LEVEL     0U
#define DRIVER_DISABLE_LEVEL    1U
#define STALL_ACTIVE_LEVEL      1U

#define STEPS_PER_REV           200U      // full steps per motor revolution (1.8°/step motor)
#define MICROSTEPS              16U       // microstepping setting on the driver (TMC2226 = 16:1)

// Gearbox ratio between motor and output shaft.
// For a 20:1 gearbox, the output shaft turns 1 rev for every 20 motor revs.
#define GEAR_RATIO              20.0f

// Number of entries in the speed ramp lookup table.
// Index 0  = slowest (start/stop), index (RAMP_LEVELS-1) = fastest (cruise).
// More levels = smoother ramp, but uses more RAM.
#define RAMP_LEVELS             16U

// Minimum number of steps to spend accelerating and decelerating.
// This ensures even short moves have a noticeable ramp.
#define MIN_RAMP_STEPS          50U

// Percentage of total steps to use for acceleration/deceleration (0-100).
// For example, 20 means use 20% of steps for accel, 20% for decel, 60% for cruise.
#define RAMP_PERCENTAGE         20U

#define TIMER2_TICK_HZ          125000.0f

// NOTE: MIN_SPEED must be >= (TIMER2_TICK_HZ / 255) to avoid PR2 clamping.
// For 125kHz timer, MIN_SPEED should be >= ~490 SPS for a visible ramp.
// If MIN_SPEED is too low, all slow speeds will use PR2=255 and the ramp won't be noticeable.
#define STEPPER_MIN_SPEED_SPS   500.0f
#define STEPPER_MAX_SPEED_SPS   8500.0f
#define STEPPER_ACCEL_SPS2      500.0f

static volatile int32_t current_step_position = 0;
static volatile uint32_t remaining_steps = 0;
static volatile bool     is_moving = false;
static volatile bool     current_direction_is_clockwise = true;
static volatile bool     stall_detected = false;
static volatile float    current_speed_sps = STEPPER_MIN_SPEED_SPS;
static volatile float    current_interval_ticks = 100.0f;
static volatile bool     decel_phase = false;

// Lookup table for Timer2 period (PR2) values used for the speed ramp.
// Filled at startup from STEPPER_MIN_SPEED_SPS and STEPPER_MAX_SPEED_SPS.
static uint8_t           ramp_table[RAMP_LEVELS];

// Counts how many step pulses have been generated in the current move.
static volatile uint32_t steps_taken_in_move = 0;

// Pre-calculated ramp parameters for the current move (calculated once at move start).
// This avoids 32-bit math in the ISR.
static volatile uint16_t ramp_steps_for_move = 0;        // How many steps to use for accel/decel
static volatile uint16_t ramp_step_size = 0;             // Steps per ramp level (ramp_steps / RAMP_LEVELS)

static void configure_pins(void);
static void configure_timer2(void);
static void stop_motion_internal(void);
static bool stall_pin_is_active(void);
static int32_t angle_to_steps(float angle_in_degrees);
static void init_ramp_table(void);

/**
 * Initializes the stepper motor control module.
 * Configures pins, sets up Timer2, and resets all motor state variables.
 * Call this once at system startup before using any other motor functions.
 */
void MotorControl_Init(void)
{
    configure_pins();
    configure_timer2();
    init_ramp_table();
    current_step_position = 0;
    remaining_steps = 0;
    is_moving = false;
    current_direction_is_clockwise = true;
    stall_detected = false;
    decel_phase = false;
}

/**
 * Timer2 interrupt service routine for stepper motor control.
 * This function is called automatically by the hardware when Timer2 overflows.
 * It generates step pulses, updates the ramp profile, checks for stalls,
 * and notifies the LED module for animation updates.
 * Do not call this function directly - it is handled by the interrupt system.
 */
void MotorControl_TimerISR(void)
{
    PIR1bits.TMR2IF = 0;

    LedRipple_TimerTick();

    if (is_moving == false)
    {
        return;
    }

    // Optional stall handling (currently disabled).
    if (stall_pin_is_active())
    {
        stall_detected = true;
        stop_motion_internal();
        return;
    }

    // Generate one step pulse
    STEP_LAT = 1;
    __nop();
    __nop();
    STEP_LAT = 0;

    // Update absolute step position based on direction
    if (current_direction_is_clockwise)
    {
        current_step_position++;
    }
    else
    {
        current_step_position--;
    }

    // Update remaining steps and step counter for this move
    if (remaining_steps > 0U)
    {
        remaining_steps--;
    }

    steps_taken_in_move++;

    // If we have completed all requested steps, stop motion
    if (remaining_steps == 0U)
    {
        stop_motion_internal();
        return;
    }

    // --- Lookup-table based speed ramp (optimized for 16-bit math) ---
    // All ramp calculations were pre-computed at move start, so we just do simple comparisons here.
    uint8_t ramp_index;
    uint16_t steps_taken_16 = (uint16_t)steps_taken_in_move;  // Truncate to 16-bit for comparison
    uint16_t remaining_16 = (uint16_t)remaining_steps;        // Truncate to 16-bit for comparison

    // For very short moves (where ramp_steps_for_move >= remaining_steps),
    // we just use a constant slow speed (no ramp)
    if (ramp_steps_for_move >= remaining_16 + steps_taken_16)
    {
        // Very short move: use slowest speed
        ramp_index = 0U;
    }
    else if (steps_taken_16 < ramp_steps_for_move)
    {
        // Acceleration phase: gradually increase speed
        // Use pre-calculated step_size to avoid division
        if (ramp_step_size > 0U)
        {
            ramp_index = (uint8_t)(steps_taken_16 / ramp_step_size);
        }
        else
        {
            ramp_index = 0U;
        }
        if (ramp_index >= RAMP_LEVELS)
        {
            ramp_index = (uint8_t)(RAMP_LEVELS - 1U);
        }
    }
    else if (remaining_16 <= ramp_steps_for_move && remaining_16 > 0U)
    {
        // Deceleration phase: gradually decrease speed
        // Use pre-calculated step_size to avoid division
        if (ramp_step_size > 0U)
        {
            ramp_index = (uint8_t)((remaining_16 - 1U) / ramp_step_size);
        }
        else
        {
            ramp_index = 0U;
        }
        if (ramp_index >= RAMP_LEVELS)
        {
            ramp_index = (uint8_t)(RAMP_LEVELS - 1U);
        }
    }
    else
    {
        // Cruise phase: stay at the fastest entry
        ramp_index = (uint8_t)(RAMP_LEVELS - 1U);
    }

    PR2 = ramp_table[ramp_index];
}

/**
 * Starts moving the motor to a target angle.
 * Calculates the required number of steps, sets the direction,
 * enables the driver, and starts the timer for step generation.
 * The motor will accelerate, cruise, and decelerate automatically.
 *
 * @param angle_degrees Target angle in degrees (0.0 to 360.0)
 */
void MotorControl_StartMoveToAngle(float angle_degrees)
{
    // Stop any current motion first to ensure we have accurate position
    if (is_moving)
    {
        stop_motion_internal();
    }

    // Convert target angle to absolute step position
    int32_t destination_steps = angle_to_steps(angle_degrees);

    // Read current position (volatile, so read it once)
    int32_t current_pos = current_step_position;

    // Calculate how many steps we need to move (positive = forward, negative = backward)
    int32_t difference = destination_steps - current_pos;

    // If difference is 0 or very small (within 1 step), we're already at the target
    if (difference == 0 || (difference > -2 && difference < 2))
    {
        return;
    }

    if (difference > 0)
    {
        // Need to move forward (clockwise)
        current_direction_is_clockwise = true;
        DIR_LAT = 1;
        remaining_steps = (uint32_t)difference;
    }
    else
    {
        // Need to move backward (counter-clockwise)
        current_direction_is_clockwise = false;
        DIR_LAT = 0;
        remaining_steps = (uint32_t)(-difference);
    }

    // Reset per-move step counter and start at the slowest ramp entry
    steps_taken_in_move = 0;
    
    // Pre-calculate ramp parameters once at move start (avoids 32-bit math in ISR)
    // Calculate how many steps to use for acceleration/deceleration
    uint32_t total_steps = remaining_steps;
    
    // For very short moves, use a simpler ramp (no acceleration/deceleration phases)
    if (total_steps < MIN_RAMP_STEPS)
    {
        // For very short moves, just use a fixed slow speed (no ramp)
        ramp_steps_for_move = (uint16_t)total_steps;
        ramp_step_size = 1U;
    }
    else
    {
        // Normal move: calculate ramp steps
        uint32_t ramp_steps = (total_steps * RAMP_PERCENTAGE) / 100U;
        if (ramp_steps < MIN_RAMP_STEPS)
        {
            ramp_steps = MIN_RAMP_STEPS;
        }
        // Don't use more than half the total steps for ramping
        if (ramp_steps > (total_steps / 2U))
        {
            ramp_steps = total_steps / 2U;
        }
        // Store as 16-bit (clamp if needed, but moves > 65535 steps are rare)
        if (ramp_steps > 65535U)
        {
            ramp_steps_for_move = 65535U;
        }
        else
        {
            ramp_steps_for_move = (uint16_t)ramp_steps;
        }
        // Pre-calculate step size per ramp level (avoids division in ISR)
        if (ramp_steps_for_move > 0U)
        {
            ramp_step_size = ramp_steps_for_move / (uint16_t)RAMP_LEVELS;
            if (ramp_step_size == 0U)
            {
                ramp_step_size = 1U;  // Ensure at least 1 to avoid division by zero
            }
        }
        else
        {
            ramp_step_size = 1U;
        }
    }
    // Ensure we have at least 1 step to move
    if (remaining_steps == 0U)
    {
        return;
    }
    
    PR2 = ramp_table[0];
    TMR2 = 0;
    PIR1bits.TMR2IF = 0;

    ENABLE_LAT = DRIVER_ENABLE_LEVEL;
    is_moving = true;
    stall_detected = false;
    decel_phase = false;

    LedRipple_HandleMotorStart(current_direction_is_clockwise);

    // Enable timer interrupt and start timer (order matters)
    PIE1bits.TMR2IE = 1;
    T2CONbits.TMR2ON = 1;
}

/**
 * Immediately stops the motor motion.
 * Disables the timer, turns off the driver, and clears the moving flag.
 * This can be called at any time to halt the motor.
 */
void MotorControl_StopMotion(void)
{
    stop_motion_internal();
}

/**
 * Checks if the motor is currently moving.
 *
 * @return true if the motor is moving, false if it is idle
 */
bool MotorControl_IsMoving(void)
{
    return is_moving;
}

/**
 * Checks if a stall condition has been detected.
 * A stall is detected when the driver's diagnostic pin indicates a fault.
 *
 * @return true if a stall was detected, false otherwise
 */
bool MotorControl_IsStallDetected(void)
{
    return stall_detected;
}

/**
 * Clears the stall detection flag.
 * Call this after handling a stall condition to allow the motor to run again.
 */
void MotorControl_ClearStallFlag(void)
{
    stall_detected = false;
}

/**
 * Gets the current step position of the motor.
 * This is a signed 32-bit value that tracks the cumulative steps taken.
 *
 * @return Current step position (can be negative for counter-clockwise movement)
 */
int32_t MotorControl_GetCurrentSteps(void)
{
    return current_step_position;
}

/**
 * Checks the current direction of motor movement.
 *
 * @return true if moving clockwise, false if moving counter-clockwise
 */
bool MotorControl_IsDirectionClockwise(void)
{
    return current_direction_is_clockwise;
}

/**
 * Configures the GPIO pins for stepper motor control.
 * Sets STEP, DIR, and ENABLE as outputs, STALL as input.
 * Disables analog functions and sets initial pin states.
 */
static void configure_pins(void)
{
    STEP_ANSEL = 0;
    DIR_ANSEL = 0;
    STALL_ANSEL = 0;
    ENABLE_ANSEL = 0;

    STEP_TRIS = 0;
    DIR_TRIS = 0;
    STALL_TRIS = 1;
    ENABLE_TRIS = 0;

    STEP_LAT = 0;
    DIR_LAT = 0;
    ENABLE_LAT = DRIVER_DISABLE_LEVEL;
}

/**
 * Configures Timer2 for step pulse generation.
 * Sets up the timer with Fosc/4 clock source and 1:64 prescaler.
 * The timer period is initially set for minimum speed.
 */
static void configure_timer2(void)
{
    // Note: Timer2 on PIC16F18326 uses Fosc/4 as clock source (not configurable)
    // T2HLT and T2RST registers don't exist on this device
    uint8_t initial_interval = (uint8_t)(TIMER2_TICK_HZ / STEPPER_MIN_SPEED_SPS);
    if (initial_interval == 0U)
    {
        initial_interval = 1U;
    }
    
    PR2 = initial_interval;
    TMR2 = 0x00;
    PIR1bits.TMR2IF = 0;
    PIE1bits.TMR2IE = 0;
    T2CONbits.T2OUTPS = 0;
    T2CONbits.T2CKPS = 0b11;
    T2CONbits.TMR2ON = 0;
}

/**
 * Internal function to stop motor motion.
 * Stops the timer, disables the driver, clears flags, and notifies LED module.
 * This is called by both the public StopMotion function and the ISR.
 */
static void stop_motion_internal(void)
{
    T2CONbits.TMR2ON = 0;
    PIE1bits.TMR2IE = 0;
    STEP_LAT = 0;
    ENABLE_LAT = DRIVER_DISABLE_LEVEL;
    is_moving = false;
    remaining_steps = 0;
    decel_phase = false;
    steps_taken_in_move = 0;
    LedRipple_HandleMotorStop();
}

/**
 * Updates the trapezoidal speed ramp profile.
 * Calculates whether to accelerate, cruise, or decelerate based on remaining steps.
 * Updates the current speed and Timer2 period accordingly.
 * This implements a constant acceleration/deceleration profile.
 */
static void update_ramp_profile(void)
{
    // Kept only to avoid unused-function warnings if referenced elsewhere.
    // The active speed ramp now uses the lookup table in MotorControl_TimerISR().
}

static void init_ramp_table(void)
{
    // Pre-calculate PR2 values for each ramp level using the configured
    // minimum and maximum speeds. This runs once at startup.
    //
    // Level 0       -> minimum speed (largest PR2)
    // Last level    -> maximum speed (smallest PR2)

    for (uint8_t i = 0; i < RAMP_LEVELS; i++)
    {
        // Fraction from 0.0 (slowest) to 1.0 (fastest)
        float fraction = 0.0f;
        if (RAMP_LEVELS > 1U)
        {
            fraction = (float)i / (float)(RAMP_LEVELS - 1U);
        }

        // Interpolate speed between min and max
        float target_speed = STEPPER_MIN_SPEED_SPS +
                             fraction * (STEPPER_MAX_SPEED_SPS - STEPPER_MIN_SPEED_SPS);

        // Convert speed (steps per second) into Timer2 ticks per step
        float interval_ticks_f = TIMER2_TICK_HZ / target_speed;

        // PR2 is 8-bit (0-255), so clamp the interval
        if (interval_ticks_f < 1.0f)
        {
            interval_ticks_f = 1.0f;
        }
        else if (interval_ticks_f > 255.0f)
        {
            // If MIN_SPEED is too slow, it will clamp to 255 and the ramp won't be visible.
            // Consider increasing MIN_SPEED if you want a noticeable ramp.
            // For 125kHz timer: MIN_SPEED should be >= 125000/255 ≈ 490 SPS to avoid clamping.
            interval_ticks_f = 255.0f;
        }

        ramp_table[i] = (uint8_t)(interval_ticks_f + 0.5f);
    }
}

/**
 * Checks if the stall detection pin is active.
 *
 * @return true if stall pin indicates a fault, false otherwise
 */
static bool stall_pin_is_active(void)
{
    return (STALL_PORT == STALL_ACTIVE_LEVEL);
}

/**
 * Converts an angle in degrees to the equivalent number of steps.
 * Takes into account steps per revolution and microstepping settings.
 *
 * @param angle_in_degrees Angle in degrees (can be negative)
 * @return Number of steps (signed integer, rounded to nearest)
 */
static int32_t angle_to_steps(float angle_in_degrees)
{
    // Total steps per one full revolution at the output shaft:
    // steps_per_rev_output = motor_steps_per_rev * microsteps * gear_ratio
    float steps_per_degree = ((float)(STEPS_PER_REV * MICROSTEPS) * GEAR_RATIO) / 360.0f;
    float raw_steps = angle_in_degrees * steps_per_degree;

    if (raw_steps >= 0.0f)
    {
        return (int32_t)(raw_steps + 0.5f);
    }

    return (int32_t)(raw_steps - 0.5f);
}

