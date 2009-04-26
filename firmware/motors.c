

#include "motors.h"

#include <avr/io.h>


/*
 * Motor velocity.
 */
static volatile int16_t mota = 0; /**< Motor A velocity. */
static volatile int16_t motb = 0; /**< Motor B velocity. */
#if 0
static uint8_t mot_accel; /**< Motor acceleration. */
static uint8_t mot_soft; /**< Soft start zone. */
static uint8_t mot_soft_accel; /**< Soft start acceleration. */
#endif


/**
 * @brief Update routine for motors.
 *
 * This routine will update the motors velocity based on the acceleration
 *  parameters.
 */
void motors_update (void)
{
}


/**
 * @brief Sets the motors.
 *
 * Sign determines direction of rotations:
 *  - Positive is forward.
 *  - Negative is backwards.
 *
 * Value in absolute should be between 0 and 10000.
 *
 *    @param motor_a Velocity to set Motor A to.
 *    @param motor_b Velocity to set Motor B to.
 */
void motors_set( int16_t motor_a, int16_t motor_b )
{
   mota = motor_a;
   motb = motor_b;
}


/**
 * @brief Initializes the motors.
 */
void motors_init (void)
{
   /* Initialize reverse pins. */
   PORTA |= _BV(PINA3) |
            _BV(PINA2);
   DDRA  |= _BV(PINA3) |
            _BV(PINA2);

   /* Initialize pwm.
    *
    * We'll want the fast PWM mode wih the 64 prescaler.
    *
    *    f_pwm = f_clk / (256 * N)
    *    f_pwm = 20 MHz / (256 * 64) = 1.22 kHz
    */
   TCCR0A = _BV(WGM00) | _BV(WGM01) | /* Fast PWM mode. */
            _BV(COM0A1); /* Non-inverting mode. */
   TCCR0B = _BV(CS01)  | _BV(CS00); /* 64 prescaler */
   /* Start both motors stopped. */
   OCR0A  = 0;
   OCR0B  = 0;
}


