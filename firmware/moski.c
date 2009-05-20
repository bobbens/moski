
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
 *  f_task = f_sched * DIVIDER / DIVIDER_MAX
 *
 * Example
 *  100 Hz = 20 kHz * 5 / 1000
 */
#define SCHED_MOTOR_DIVIDER   40
#define SCHED_TEMP_DIVIDER    1
#define SCHED_MAX_DIVIDER     100
/* Scheduler state flags. */
#define SCHED_MOTORS          (1<<0)
#define SCHED_TEMP            (1<<1)


/**
 * Operating mode.
 */
uint8_t moski_mode = MOSKI_MODE_OPEN; /**< Current operating mode. */


/*
 * Scheduler.
 */
static uint16_t sched_counter = 0x00; /**< Scheduler counter. */
static uint8_t  sched_flags   = 0x00; /**< Scheduler flags. */


/*
 * Encoders.
 */
typedef struct encoder_s {
   uint16_t cur_tick;
   uint16_t last_tick;
   uint8_t  pin_state;
} encoder_t;
static encoder_t encA, encB;


/*
 * Prototypes.
 */
/* I2C. */
static uint8_t moski_read( uint8_t pos, uint8_t value );
static uint8_t moski_write( uint8_t buf_len, uint8_t *buffer );
/* Scheduler. */
static __inline void sched_init (void);
static __inline void sched_run( uint8_t flags );
/* Encoders. */
static void encoder_init( encoder_t *enc, uint8_t pinstate );
static __inline void encoders_init (void);



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


/**
 * @brief Initializes a single encoder.
 *
 *    @param enc Encoder to initialize.
 *    @param pinstate Current pinstate.
 */
static void encoder_init( encoder_t *enc, uint8_t pinstate )
{
   enc->cur_tick  = 0;
   enc->last_tick = 0;
   enc->pin_state = pinstate;
}


/**
 * @brief Initializes the encoders.
 */
static __inline void encoders_init (void)
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
 * @brief Scheduler interrupt on timer1 overflow.
 *
 * @note Running at 20 kHz.
 */
ISR(SIG_OVERFLOW1)
{
   uint8_t inp;

   /* Check to see if encoder A changed. */
   if (encA.cur_tick < UINT16_MAX) /* Avoid overflow. */
      encA.cur_tick++;
   inp = ENCODER_PIN & _BV(ENCODER_PIN_A);
   if (inp != encA.pin_state) { /* See if state changed. */
      encA.pin_state = inp;
      encA.last_tick = encA.cur_tick; /* Last tick is current tick. */
      encA.cur_tick  = 0x00; /* Reset counter. */
   }

   /* Check to see if encoder B changed. */
   if (encB.cur_tick < UINT16_MAX) /* Avoid overflow. */
      encB.cur_tick++;
   inp = ENCODER_PIN & _BV(ENCODER_PIN_B);
   if (inp != encB.pin_state) { /* See if state changed. */
      encB.pin_state = inp;
      encB.last_tick = encB.cur_tick; /* Last tick is current tick. */
      encB.cur_tick  = 0x00; /* Reset counter. */
   }

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
static __inline void sched_init (void)
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
   sched_flags = 0x00;
}


/**
 * @brief Runs the scheduler.
 */
static __inline void sched_run( uint8_t flags )
{  
   /*
    * Run tasks.
    *
    * Very important that the periodicity of the scheduler update task
    *  allows this to finish or the loss of task execution may occur.
    */
   /* Motor task. */
   (void) flags;
   /*if (flags & SCHED_MOTORS)
      motors_control();*/
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
   /*motors_init();*/

   /* Initialize the scheduler. */
   sched_init();

   /* Set up watchdog timer. */
   wdt_reset(); /* Just in case. */
   wdt_enable(WDTO_250MS);

   /* Main loop. */
   while (1) {
      /* Atomic test to see if has anything to do. */
      cli();
      if (sched_flags != 0x00) {
   
         /* Atomic store temporary flags and reset real flags in case we run a bit late. */
         flags = sched_flags;
         sched_flags = 0;
         sei(); /* Restart interrupts. */

         /* Run scheduler. */
         sched_run( flags );
      }
      /* Reactivate interrupts and do nothing. */
      else
         sei();

      /* Sleep until next interrupt. */
      sleep_mode();
   }
}

