
#include "moski.h"

#include "global.h"

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdint.h>

#include "moski_conf.h"
#include "i2cs.h"


/*
 * Scheduler defines.
 */
/*
 * Scheduler divider.  Formula is:
 *
 *  f_task = f_sched / DIVIDER
 *
 * Example
 *  100 Hz = 20 kHz / 200
 */
#define SCHED_MOTOR_DIVIDER   60 /**< What to divide main freq by for motor task. */
#define SCHED_I2CS_TIMEOUT_DIVIDER 240 /**< Timeout tick for I2CS. */
#define SCHED_TEMP_DIVIDER    1 /**< What to divide main freq by for temp task. */
#define SCHED_MAX_DIVIDER     240 /**< Overflow amount for scheduler divider. */
/* Scheduler state flags. */
#define SCHED_MOTORS          (1<<0) /**< Run the motor task. */
#define SCHED_I2CS_TIMEOUT    (1<<1) /**< Updates the i2cs timeout. */
#define SCHED_TEMP            (1<<2) /**< Run the temperature task. */


/**
 * Operating mode.
 */
uint8_t moski_mode = MOSKI_MODE_OPEN; /**< Current operating mode. */


/*
 * Scheduler.
 */
static volatile uint8_t sched_counter = 0; /**< Scheduler counter. */
static volatile uint8_t sched_flags   = 0; /**< Scheduler flags. */


/*
 * Encoders.
 */
/**
 * @brief The encoder structure.
 */
typedef struct encoder_s {
   uint16_t cur_tick; /**< Current tick (counts up with overflow). */
   uint16_t last_tick; /**< Last tick to register a state change. */
   uint8_t  pin_state; /**< Current pin state. */
} encoder_t;
static encoder_t encA; /**< Encoder on motor A. */
static encoder_t encB; /**< Encoder on motor B. */


/*
 * Motors.
 */
/**
 * @brief The motor structure.
 */
typedef struct motor_s {
   /* Target. */
   uint16_t target; /**< Target velocity. */

   /* Internal usage variables. */
   int16_t e_accum; /**< Accumulated error, for integral part. */

   /* Controller parameters - these are divided by 16 (>>4). */
   uint8_t kp; /**< Proportional part of the controller. */
   uint8_t ki; /**< Integral part of the controller. */
   int16_t windup; /**< Windup limit. */
} motor_t;
static motor_t motA; /**< Motor A. */
static motor_t motB; /**< Motor B. */


/*
 * Prototypes.
 */
/* I2C. */
static uint8_t moski_read( uint8_t pos, uint8_t value );
static uint8_t moski_write( uint8_t buf_len, uint8_t *buffer );
/* Scheduler. */
static void sched_init (void);
static void sched_run( uint8_t flags );
/* Encoders. */
static void encoder_initStruct( encoder_t *enc, uint8_t pinstate );
static void encoders_init (void);
/* Motors. */
static void motor_initStruct( motor_t *mot );
static void motors_init (void);
static uint8_t motor_control( motor_t *mot, encoder_t *enc );

/*
 *
 *   I 2 C
 *
 */
static uint8_t read_buf[4]; /**< Small buf to help moski_read. */
/**
 * @brief Master read function for the Moski.
 *
 *    @param pos Position of byte read.
 *    @param value Value of byte read.
 *    @return 0 to send ACK, 1 to send NACK.
 */
static uint8_t moski_read( uint8_t pos, uint8_t value )
{
   int i;

   /* First byte is always position. */
   if (pos == 0) {
      moski_mode = pos;
      return 0;
   }

   /* Fill buffer. */
   i = (pos-1) % 4;
   read_buf[i] = value;

   /* We convert from rpm to rps and set as motor targets. */
   if (i==3) {
      motA.target = ((read_buf[0]<<8) + read_buf[1])/60;
      motB.target = ((read_buf[2]<<8) + read_buf[3])/60;
   }

   return 0;
}
/**
 * @brief Master write function for the Moski.
 *
 *    @param buf_len Maximum buffer length.
 *    @param buffer Buffer to write.
 *    @return Number of bytes to write.
 */
static uint8_t moski_write( uint8_t buf_len, uint8_t *buffer )
{
   (void) buf_len;

   /* Set up the data. */
   buffer[0] = encA.last_tick >> 8;
   buffer[1] = encA.last_tick;
   buffer[2] = encB.last_tick >> 8;
   buffer[3] = encB.last_tick;

#if MOSKI_USE_TEMP
   uint16_t temp;
   temp = temp_get();
   buffer[4] = temp>>8;
   buffer[5] = temp & 0xFF;
#else /* MOSKI_USE_TEMP */
   buffer[4] = 0;
   buffer[5] = 0;
#endif /* MOSKI_USE_TEMP */

   return 6;
}


/*
 *
 *   E N C O D E R S
 *
 */
/**
 * @brief Initializes a single encoder.
 *
 *    @param enc Encoder to initialize.
 *    @param pinstate Current pinstate.
 */
static void encoder_initStruct( encoder_t *enc, uint8_t pinstate )
{
   enc->cur_tick  = 0;
   enc->last_tick = UINT16_MAX; /* Consider stopped. */
   enc->pin_state = pinstate;
}
/**
 * @brief Initializes the encoders.
 */
static void encoders_init (void)
{
   /* Set pins as input. */
   ENCODER_DDR  &= ~(_BV(ENCODER_DD_A) | _BV(ENCODER_DD_B));

   /* Initialize data structures. */
   encoder_initStruct( &encA, (ENCODER_PIN & _BV(ENCODER_PIN_A)) );
   encoder_initStruct( &encB, (ENCODER_PIN & _BV(ENCODER_PIN_B)) );

   /* Set up interrupts.
    *
    * PCINT automatically generates interrupt on toggle of the PCINT pins
    *  involved.  In this case the ENCODER_INT_A and ENCODER_INT_B.
    */
   GIMSK       |= _BV(ENCODER_INT); /* Globally enable pin change interrupts. */
   ENCODER_MSK |= _BV(ENCODER_INT_A) | _BV(ENCODER_INT_B); /* Enabled encoder interrupts. */

}
/**
 * @brief Encoder signal handler.
 */
ISR(ENCODER_SIG)
{
   uint8_t inp;

   /* Check to see if encoder A changed. */
   inp = ENCODER_PIN & _BV(ENCODER_PIN_A);
   if (inp != encA.pin_state) { /* See if state changed. */
      encA.pin_state = inp;
      encA.last_tick = encA.cur_tick; /* Last tick is current tick. */
      encA.cur_tick  = 0; /* Reset counter. */
   }

   /* Check to see if encoder B changed. */
   inp = ENCODER_PIN & _BV(ENCODER_PIN_B);
   if (inp != encB.pin_state) { /* See if state changed. */
      encB.pin_state = inp;
      encB.last_tick = encB.cur_tick; /* Last tick is current tick. */
      encB.cur_tick  = 0; /* Reset counter. */
   }
}



/*
 *
 *   M O T O R S
 *
 */
/**
 * @brief Initializes a motor structure.
 */
static void motor_initStruct( motor_t *mot )
{
   /* Target to seek out, 60 until communication issues are solved. */
   mot->target  = 50;

   /* Internal use variables. */
   mot->e_accum = 0;

   /* Controller parameters. */
   mot->kp      = 100;
   mot->ki      = 5;
   mot->windup  = 816;
}
/**
 * @brief Initializes the motors.
 */
static void motors_init (void)
{
   /* Initialize reverse pins. */
   PORTA &= ~_BV(PA3) |
            ~_BV(PA2);
   DDRA  |= _BV(PINA3) |
            _BV(PINA2);

   /* Initialize PWM pins. */
   DDRA  |= _BV(DDA7);
   DDRB  |= _BV(DDB2);

   /* Set direction forward */
   PORTA &= ~( _BV(PA2) | _BV(PA3) );

   /* Initialize pwm.
    *
    * We'll want the fast PWM mode wih the 64 prescaler.
    *
    *    f_pwm = f_clk / (256 * N)
    *    f_pwm = 20 MHz / (256 * 64) = 1.22 kHz
    *    f_pwm = 20 MHz / (256 * 8)  = 9.76 kHz
    */
   TCCR0A = _BV(WGM00) | _BV(WGM01) | /* Fast PWM mode. */
            _BV(COM0A1) | _BV(COM0B1); /* Non-inverting mode. */
   /*TCCR0B = _BV(CS01)  | _BV(CS00);*/ /* 64 prescaler */
   TCCR0B = _BV(CS01); /* 8 prescaler. */
   /*TIMSK0 = _BV(TOIE0); *//* Enable overflow interrupt on timer 0. */

   /* Start both motors stopped. */
   OCR0A  = 0x00;
   OCR0B  = 0x00;

   /* Initialize motor structures. */
   motor_initStruct( &motA );
   motor_initStruct( &motB );
}
/**
 * @brief Does the motor control.
 *
 *    @param mot Motor to control.
 *    @param enc Encoder feedback for motor to control.
 */
static uint8_t motor_control( motor_t *mot, encoder_t *enc )
{
   int16_t feedback, error, output;
   uint8_t pwm;

   /* Process the feedback.
    *
    *    ticks[20kHz]   N encoder turn        1 second
    * X  ------------ * -------------- * -------------------   ===>
    *    encoder turn    1 revolution     20000 ticks[50kHz]
    *
    *
    *  20000 / N
    *  ---------  = revolutions per second
    *      X
    */
   feedback = 5000 / enc->last_tick;

   /* Calculate the error. */
   error    = mot->target - feedback;

   /* Accumulate error. */
   mot->e_accum += error;
   /* Anti-windup. */
   if (mot->e_accum > mot->windup)
      mot->e_accum = mot->windup;
   else if (mot->e_accum < -mot->windup)
      mot->e_accum = -mot->windup;

   /* Run control - PI. */
   output   = (error * mot->kp) >> 4; /* P */
   output  += (mot->e_accum * mot->ki) >> 4; /* I */

   /* Get PWM output, note we can't do backwards. */
   if (output > 255)
      pwm = 0xFF;
   else if (output < 0)
      pwm = 0;
   else
      pwm = output;

   /* It's inverted. */
   return pwm;
}


/*
 *
 *   S C H E D U L E R
 *
 */
/**
 * @brief Scheduler interrupt on timer1 overflow.
 *
 * @note Running at 20 kHz.
 */
ISR(SIG_OVERFLOW1)
{
   /* Increment Encoder A timer. */
   if (encA.cur_tick < UINT16_MAX) /* Avoid overflow. */
      encA.cur_tick++;
   else /* Overflow. */
      encA.last_tick = encA.cur_tick;

   /* Increment Encoder B timer. */
   if (encB.cur_tick < UINT16_MAX) /* Avoid overflow. */
      encB.cur_tick++;
   else /* Overflow. */
      encB.last_tick = encB.cur_tick;

   /* Do some scheduler stuff here. */
   if (!(sched_counter % SCHED_MOTOR_DIVIDER))
      sched_flags |= SCHED_MOTORS;
   if (!(sched_counter % SCHED_I2CS_TIMEOUT_DIVIDER))
      sched_flags |= SCHED_I2CS_TIMEOUT;
#if MOSKI_USE_TEMP
   if (!(sched_counter % SCHED_TEMP_DIVIDER))
      sched_flags |= SCHED_TEMP;
#endif /* MOSKI_USE_TEMP */
   sched_counter = (sched_counter+1) % SCHED_MAX_DIVIDER;

   /* Reset watchdog. */
   wdt_reset();
}
/**
 * @brief Initializes the scheduler on Timer1.
 */
static void sched_init (void)
{
   /* Phase and freq correct mode.
    *
    * f_pwm = f_clk / (2 * N * TOP)
    *
    *  1 kHz = 20 MHz / (2 * 8 * 1250)
    * 20 kHz = 20 MHz / (2 * 1 * 500)
    * 50 kHz = 20 Mhz / (2 * 1 * 200)
    */
   TCCR1A = _BV(WGM10) | /* Phase and freq correct mode with OCR1A. */
         0; /* No actual PWM output. */
   TCCR1B = _BV(WGM13) | /* Phase and freq correct mode with OCR1A. */
         _BV(CS10); /* 1 prescaler. */
#if 0
         _BV(CS11); /* 8 prescaler */
         _BV(CS11) | _BV(CS10); /* 64 prescaler */
         _BV(CS12); /* 256 prescaler */
         _BV(CS12) | _BV(CS10); /* 1024 prescaler */
#endif
   TIMSK1 = _BV(TOIE1); /* Enable Timer1 overflow. */
   OCR1A  = 500;

   /* Initialize flags. */
   sched_flags = 0;
}
/**
 * @brief Runs the scheduler.
 *
 *    @param flags Current scheduler flags to use.
 */
static void sched_run( uint8_t flags )
{
   /*
    * Run tasks.
    *
    * Very important that the periodicity of the scheduler update task
    *  allows this to finish or the loss of task execution may occur.
    */
   /* Motor task. */
   if (flags & SCHED_MOTORS) {
      OCR0A = motor_control( &motA, &encA );
      OCR0B = motor_control( &motB, &encB );
   }
   if (flags & SCHED_I2CS_TIMEOUT)
      i2cs_timeoutTick();
#if MOSKI_USE_TEMP
   /* Temp task. */
   if (flags & SCHED_TEMP)
      temp_start(); /* Start temperature conversion. */
#endif /* MOSKI_USE_TEMP */
}



/**
 * @brief Entry point.
 */
int main (void)
{
   uint8_t flags;

   /* Disable watchdog timer since it doesn't always get reset on restart. */
   wdt_disable();

   /* Set up default mode. */
   moski_mode = MOSKI_MODE_OPEN;

   /* Set up communication. */
   i2cs_setAddress( 0x09 );
   i2cs_setReadCallback( moski_read );
   i2cs_setWriteCallback( moski_write );
   i2cs_setTimeout( 100 );
   i2cs_init();

   /* Set sleep mode. */
   set_sleep_mode( SLEEP_MODE_IDLE );

   /* Start the temperature subsystem. */
#if MOSKI_USE_TEMP
   temp_init();
#endif /* MOSKI_USE_TEMP */

   /* Start the motor subsystem. */
   motors_init();

   /* Initialize the encoders. */
   encoders_init();

   /* Initialize the scheduler. */
   sched_init();

   /* Set up watchdog timer. */
   wdt_reset(); /* Just in case. */
   wdt_enable(WDTO_15MS);

   /* Main loop. */
   while (1) {
      /* Atomic test to see if has anything to do. */
      cli();
      if (sched_flags != 0) {

         /* Atomic store temporary flags and reset real flags in case we run a bit late. */
         flags = sched_flags;
         sched_flags = 0;
         sei(); /* Restart interrupts. */

         /* Run scheduler. */
         sched_run( flags );
      }
      /* Sleep. */
      else {
         /* Atomic sleep as specified on the documentation. */
         sleep_enable();
         sei();
         sleep_cpu();
         sleep_disable();
      }
   }
}

