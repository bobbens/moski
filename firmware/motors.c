
#include "motors.h"

#include "encoders.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include "sched.h"


/**
 * @brief Motor control structure.
 */
typedef struct motor_s {
   int16_t target; /**< Target velocity. */
} motor_t;


/*
 * Motor velocity.
 */
static motor_t mota; /**< Motor A. */
static motor_t motb; /**< Motor B. */


/**
 * @brief Update routine for motors.
 *
 * This routine will update the motor based on the current control model.
 *
 *         +-----------+      Ts /              +-----------+
 *         |  Ts * Ki  |        /     +-----+   |     K     |
 * --(+)-->| --------- |--(+)--/   -->| Zoh |-->| --------- |---+-->
 *    |    |   z - 1   |   |          +-----+   | t * s + 1 |   |
 *    |    +-----------+   |                    +-----------+   |
 *    |                    |                                    |
 *    |                 +------+                                | 
 *    |                 |  Kp  |                                |
 *    |                 +------+                     \ Ts       |
 *    |                    |             +-----+      \         |
 *    +--------------------+-------------|  n  |----   \--------+
 *                                       +-----+
 *
 */
__inline void motors_control (void)
{
}


/**
 * @brief Timer0 overflow, handles the scheduling of the motors.
 */
ISR(SIG_OVERFLOW0)
{
   /* Set the motor task to run. */
   sched_flags |= SCHED_MOTORS;
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
__inline void motors_set( int16_t motor_a, int16_t motor_b )
{
   mota.target = motor_a;
   motb.target = motor_b;
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
   TIMSK0 = _BV(TOIE0); /* Enable overflow interrupt on timer 0. */
   /* Start both motors stopped. */
   OCR0A  = 0;
   OCR0B  = 0;

   /* Initialize encoders. */
   encoders_init();
}


