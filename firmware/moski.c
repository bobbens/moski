
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
#define SCHED_MOTOR_DIVIDER   6 /**< What to divide main freq by for motor task. */
#define SCHED_TEMP_DIVIDER    1 /**< What to divide main freq by for temp task. */
#define SCHED_MAX_DIVIDER     6 /**< Overflow amount for scheduler divider. */
/* Scheduler state flags. */
#define SCHED_MOTORS          (1<<0) /**< Run the motor task. */
#define SCHED_TEMP            (1<<1) /**< Run the temperature task. */


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
 * Prototypes.
 */
/* I2C. */
static uint8_t moski_read( uint8_t pos, uint8_t value );
static uint8_t moski_write( uint8_t buf_len, uint8_t *buffer );
/* Scheduler. */
static void sched_init (void);
static void sched_run( uint8_t flags );
/* Encoders. */
static void encoder_init( encoder_t *enc, uint8_t pinstate );
static void encoders_init (void);
/* Motors. */
static void motors_init (void);

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
   int16_t a,b;

   /* First byte is always position. */
   if (pos == 0) {
      moski_mode = pos;
      return 0;
   }

   /* Fill buffer. */
   i = (pos-1) % 4;
   read_buf[i] = value;

   /* Send to motors. */
   if (i==3) {
      a = (read_buf[0]<<8) + read_buf[1];
      b = (read_buf[2]<<8) + read_buf[3];
      /*motors_set( a, b );*/
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
static void encoder_init( encoder_t *enc, uint8_t pinstate )
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
   ENCODER_DDR &= ~(_BV(ENCODER_PORT_A) | _BV(ENCODER_PORT_B));
   encoder_init( &encA, (ENCODER_PIN & _BV(ENCODER_PIN_A)) );
   encoder_init( &encB, (ENCODER_PIN & _BV(ENCODER_PIN_B)) );

   /* Set up interrupts. */
   GIMSK       |= _BV(INT0) | _BV(ENCODER_INT); /* Globally enable pin change interrupts. */
   ENCODER_MSK |= _BV(ENCODER_INT_A) | _BV(ENCODER_INT_B); /* Enabled encoder interrupts. */
   MCUCR       |= /*_BV(ISC01) |*/ _BV(ISC00); /* Set on rise/falling edge. */
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
 * @brief Initializes the motors.
 */
static void motors_init (void)
{
   /* Initialize reverse pins. */
   PORTA |= _BV(PA3) |
            _BV(PA2);
   DDRA  |= _BV(PINA3) |
            _BV(PINA2);

   /* Initialize PWM pins. */
   DDRA  |= _BV(DDA7);
   DDRB  |= _BV(DDB2);

   /* Initialize pwm.
    *
    * We'll want the fast PWM mode wih the 64 prescaler.
    *
    *    f_pwm = f_clk / (256 * N)
    *    f_pwm = 20 MHz / (256 * 64) = 1.22 kHz
    */
   TCCR0A = _BV(WGM00) | _BV(WGM01) | /* Fast PWM mode. */
            _BV(COM0A1) | _BV(COM0B1); /* Non-inverting mode. */
   TCCR0B = _BV(CS01)  | _BV(CS00); /* 64 prescaler */
   /*TIMSK0 = _BV(TOIE0); *//* Enable overflow interrupt on timer 0. */

   /* Start both motors stopped. */
   OCR0A  = 0;
   OCR0B  = 0;
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
static uint8_t motor_dir   = 0;
static uint8_t motor_delay = 0;
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
      motor_delay++;
      if (motor_delay==0) {
         if (!motor_dir) {
            if (OCR0A > 128)
               motor_dir = !motor_dir;
            OCR0A = OCR0A+1;
            OCR0B = OCR0B+1;
         }
         else {
            if (OCR0A < 10)
               motor_dir = !motor_dir;
            OCR0A = OCR0A-1;
            OCR0B = OCR0B-1;
         }
      }
   }
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
         sleep_enable();
         sei();
         sleep_cpu();
         sleep_disable();
      }
   }
}

