#include <xc.h>
#include <stdint.h>
#include "project_config.h"
#include "led_ripple.h"

#define LED_COUNT               48U
#define LED_UPDATE_TICKS        50U

#define LED_DATA_LAT            LATCbits.LATC2
#define LED_DATA_TRIS           TRISCbits.TRISC2
#define LED_DATA_ANSEL          ANSELCbits.ANSC2

#define LED_DELAY_SHORT() {__nop(); __nop(); __nop();}
#define LED_DELAY_LONG()  {__nop(); __nop(); __nop(); __nop(); __nop(); __nop();}

static volatile bool led_animation_active = false;
static volatile bool led_animation_forward = true;
static volatile uint8_t led_animation_position = 0;
static volatile uint8_t led_update_counter = 0;
static volatile bool led_frame_pending = false;
static uint8_t led_buffer[LED_COUNT][3];

static void leds_clear(void);
static void leds_set_pixel(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);
static void leds_apply_ripple_frame(void);
static void leds_send_data(void);

/**
 * Initializes the LED ripple animation module.
 * Configures the LED data pin and clears all LEDs.
 * Call this once at system startup.
 */
void LedRipple_Init(void)
{
    LED_DATA_ANSEL = 0;
    LED_DATA_TRIS = 0;
    LED_DATA_LAT = 0;

    led_animation_active = false;
    led_animation_forward = true;
    led_animation_position = 0;
    led_update_counter = 0;
    led_frame_pending = true;
    leds_clear();
    leds_send_data();
}

/**
 * Notifies the LED module that the motor has started moving.
 * Starts the ripple animation in the specified direction.
 *
 * @param forward true for clockwise direction, false for counter-clockwise
 */
void LedRipple_HandleMotorStart(bool forward)
{
    led_animation_active = true;
    led_animation_forward = forward;
    led_animation_position = forward ? 0 : (LED_COUNT - 1);
    led_update_counter = 0;
    led_frame_pending = true;
}

/**
 * Notifies the LED module that the motor has stopped.
 * Clears all LEDs and stops the animation.
 */
void LedRipple_HandleMotorStop(void)
{
    led_animation_active = false;
    leds_clear();
    led_frame_pending = true;
}

/**
 * Called by the motor timer ISR to update the LED animation timing.
 * Increments a counter and sets a flag when it's time to update the frame.
 * Do not call this function directly - it is called by the interrupt system.
 */
void LedRipple_TimerTick(void)
{
    if (led_animation_active == false)
    {
        return;
    }

    if (led_update_counter >= LED_UPDATE_TICKS)
    {
        led_update_counter = 0;
        led_frame_pending = true;
    }
    else
    {
        led_update_counter++;
    }
}

/**
 * Service function for LED updates.
 * Call this regularly in the main loop to update the LED animation.
 * Sends new frame data to the LEDs when a frame update is pending.
 */
void LedRipple_Service(void)
{
    if (led_frame_pending == false)
    {
        return;
    }

    led_frame_pending = false;

    if (led_animation_active)
    {
        leds_apply_ripple_frame();
    }

    leds_send_data();
}

/**
 * Clears all LEDs by setting all pixel colors to black (0,0,0).
 */
static void leds_clear(void)
{
    for (uint8_t i = 0; i < LED_COUNT; i++)
    {
        led_buffer[i][0] = 5;
        led_buffer[i][1] = 0;
        led_buffer[i][2] = 0;
    }
}

/**
 * Sets the color of a single LED pixel.
 * Colors are stored in GRB order (green, red, blue) as required by WS2812B.
 *
 * @param index LED index (0 to LED_COUNT-1)
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 */
static void leds_set_pixel(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (index >= LED_COUNT)
    {
        return;
    }

    led_buffer[index][0] = green;
    led_buffer[index][1] = red;
    led_buffer[index][2] = blue;
}

/**
 * Applies the ripple animation frame to the LED buffer.
 * Creates a traveling wave effect that moves in the motor's direction.
 * The head of the wave is bright, followed by a dimmer tail.
 */
static void leds_apply_ripple_frame(void)
{
    leds_clear();

    uint8_t head = led_animation_position;
    leds_set_pixel(head, 0, 40, 5);

    uint8_t tail = led_animation_forward
                   ? (head == 0 ? (LED_COUNT - 1) : (head - 1))
                   : ((head + 1) % LED_COUNT);

    leds_set_pixel(tail, 0, 10, 2);

    if (led_animation_forward)
    {
        head++;
        if (head >= LED_COUNT)
        {
            head = 0;
        }
    }
    else
    {
        if (head == 0)
        {
            head = LED_COUNT - 1;
        }
        else
        {
            head--;
        }
    }

    led_animation_position = head;
}

/**
 * Sends the LED buffer data to the WS2812B LED strip.
 * Uses bit-banging to generate the required timing for WS2812B protocol.
 * Interrupts are disabled during transmission for precise timing.
 * After sending all data, a 60us reset pulse is generated.
 */
static void leds_send_data(void)
{
    uint8_t gie_state = INTCONbits.GIE;
    INTCONbits.GIE = 0;

    for (uint8_t i = 0; i < LED_COUNT; i++)
    {
        for (uint8_t byte_index = 0; byte_index < 3; byte_index++)
        {
            uint8_t value = led_buffer[i][byte_index];
            for (uint8_t mask = 0x80; mask != 0; mask >>= 1)
            {
                if ((value & mask) != 0U)
                {
                    LED_DATA_LAT = 1;
                    LED_DELAY_LONG();
                    LED_DATA_LAT = 0;
                    LED_DELAY_SHORT();
                }
                else
                {
                    LED_DATA_LAT = 1;
                    //LED_DELAY_SHORT();
                    LED_DATA_LAT = 0;
                    LED_DELAY_LONG();
                }
            }
        }
    }

    __delay_us(60);
    INTCONbits.GIE = gie_state;
}

