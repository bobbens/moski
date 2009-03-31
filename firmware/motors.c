

#include "motors.h"

#include <avr/io.h>


/*
 * Motor positions.
 */
static int16_t mota = 0; /**< Motor A position. */
static int16_t motb = 0; /**< Motor B position. */
static uint8_t mot_accel; /**< Motor acceleration. */
static uint8_t mot_soft; /**< Soft start zone. */
static uint8_t mot_soft_accel; /**< Soft start acceleration. */


/**
 * @brief Update routine for motors.
 *
 * This routine will update the motors velocity based on the acceleration
 *  parameters.
 */
static void motors_update (void)
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
   PORTA    |=    _BV(PINA3) |
                  _BV(PINA2);
   DDRA     |=    _BV(PINA3) |
                  _BV(PINA2);

   /* Initialize pwm.
    *
    * We'll want the fast PWM mode wih the 64 prescaler.
    *
    *    f_pwm = f_clk / (256 * N)
    *    f_pwm = 20 MHz / (256 * 64) = 1.22 kHz
    */
}


