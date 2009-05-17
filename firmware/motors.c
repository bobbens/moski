
#include "motors.h"

#include "encoders.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include "sched.h"
#include "mode.h"


/**
 * @brief Motor control structure.
 */
typedef struct motor_s {
   /* Current motor parameters. */
   int dir; /**< Target direction, 0 = forward, 1 = backwards. */
   uint16_t target; /**< Target velocity. */
   uint8_t cmd; /**< Next command (delayed one cycle). */

   /* Feedback. */
   int16_t vel; /**< Current motor velocity. */

   /* Controller parameters. */
   uint16_t Ki; /**< Integral parameter. */
   uint16_t Kp; /**< Proportional parameter. */
   uint16_t windup; /**< Windup limit. */

   /* Controller status data. */
   uint16_t integral; /**< Integral accumulator. */
} motor_t;


/*
 * Motor velocity.
 */
static motor_t mota; /**< Motor A. */
static motor_t motb; /**< Motor B. */


/*
 * Prototypes.
 */
static uint8_t motor_control( motor_t *mot, uint8_t feedback );
static void motor_set( motor_t *motor, int16_t target );


/**
 * @brief Control routine for a motor.
 *
 * This routine will update the motor based on the current control model.
 *
 *         +-----------+      Ts /              +-----------+
 *  +      |  Ts * Ki  | +      /     +-----+   |     K     |
 * --(+)-->| --------- |--(+)--/   -->| Zoh |-->| --------- |---+-->
 *   -|    |   z - 1   |  +|          +-----+   | t * s + 1 |   |
 *    |    +-----------+   |                    +-----------+   |
 *    |                    |                                    |
 *    |                 +------+                                | 
 *    |                 |  Kp  |                                |
 *    |                 +------+                     \ Ts       |
 *    |                    |             +-----+      \         |
 *    +--------------------+-------------|  n  |----   \--------+
 *                                       +-----+
 *
 * @note Using 16 bit numbers for calculations using 8 bits for the significant
 *       numbers and 8 bits for the "decimals".
 */
static uint8_t motor_control( motor_t *mot, uint8_t feedback )
{
   /* unsigned short sat accum */
   uint16_t rpm_feedback, err, output, cmd;

   /* Save velocity. */
   mot->vel       = (mot->dir) ? -feedback : feedback;
   mot->vel      *= MOTOR_CONTROL_HZ;

   /* Get target velocity. */
   rpm_feedback   = feedback << 8;

   /* Calculate error. */
   err            = mot->target - rpm_feedback;

   /* Calculate integral part. */
   mot->integral += err;
   /* Anti-windup. */
   if (mot->integral > mot->windup)
      mot->integral = mot->windup;
   output         = mot->integral * mot->Ki;

   /* Calculate proportional part. */
   output        += rpm_feedback * mot->Kp;

   /* Set command. */
   cmd            = output >> 8;
   
   return cmd;
}


/**
 * @brief Runs the control routine on both motors.
 */
__inline void motors_control (void)
{
   uint8_t enca, encb;

   /* Store encoder values and reset counters atomically. */
   cli();
   enca        = encoder_a;
   encoder_a   = 0;
   encb        = encoder_b;
   encoder_b   = 0;
   sei();

   /* Control loop. */
   if (moski_mode == MOSKI_MODE_CONTROL) {
      mota.cmd = motor_control( &mota, enca );
      motb.cmd = motor_control( &motb, encb );
   }
   /* Open loop. */
   else if (moski_mode == MOSKI_MODE_OPEN) {
      mota.cmd = mota.target>>8;
      motb.cmd = motb.target>>8;
   }
}


/**
 * @brief Timer0 overflow, handles the scheduling of the motors.
 */
ISR(SIG_OVERFLOW0)
{
   /* Set the motor task to run. */
   sched_flags |= SCHED_MOTORS;

   /* Set last target (one cycle delay). */
   OCR0A = mota.cmd;
   OCR0B = motb.cmd;
}


/**
 * @brief Sets the motor target.
 */
static void motor_set( motor_t *mot, int16_t target )
{
   uint32_t buf;

   /* Calculate target. */
   buf   = (target < 0) ? -target : target; /* Get absolute target. */
   buf <<= 8; /* Shift to use bits as decimals. */
   buf  /= MOTOR_CONTROL_HZ; /* Divide keeping decimals. */

   /* Set target. */
   mot->dir    = (target < 0) ? 1 : 0;
   mot->target = (uint16_t)buf; /* Cast away sign. */
}


/**
 * @brief Sets the motors.
 *
 * Sign determines direction of rotations:
 *  - Positive is forward.
 *  - Negative is backwards.
 *
 * Value in absolute should be in RPM.
 *
 *    @param motor_a Velocity to set Motor A to.
 *    @param motor_b Velocity to set Motor B to.
 */
void motors_set( int16_t motor_a, int16_t motor_b )
{
   motor_set( &mota, motor_a );
   motor_set( &motb, motor_b );
}


/**
 * @brief Gets the current motor velocity.
 *
 *    @param[out] motor_a Velocity on motor A.
 *    @param[out] motor_b Velocity on motor B.
 */
void motors_get( int16_t *motor_a, int16_t *motor_b )
{
   if (moski_mode == MOSKI_MODE_CONTROL) {
      *motor_a = mota.vel;
      *motor_b = motb.vel;
   }
   else if (moski_mode == MOSKI_MODE_OPEN) {
      *motor_a = mota.cmd;
      *motor_b = motb.cmd;
   }
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

   /* Clear targets. */
   mota.target = 0;
   motb.target = 0;

   /* Initialize encoders. */
   encoders_init();
}


