
#include "moski.h"

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdint.h>

#include "i2cs.h"


static uint8_t moski_mode = 0x00; /**< Current operating mode. */


/*
 * Scheduler.
 */
#define SCHED_MOTORS    (1<<0) /**< Run motor update task. */
#define SCHED_ENCODER   (1<<1) /**< Run encoder update task. */
#define SCHED_TEMP      (1<<2) /**< Run temp update task. */
static uint8_t sched_flags = 0x00; /**< Scheduler flags. */


/*
 * Prototypes.
 */
static uint8_t moski_read( uint8_t pos, uint8_t value );
static uint8_t moski_write( uint8_t buf_len, uint8_t *buffer );
static void sched_init (void);


/**
 * @brief Master read function for the Moski.
 *
 *    @param pos Position of byte read.
 *    @param value Value of byte read.
 *    @return 0 to send ACK, 1 to send NACK.
 */
static uint8_t moski_read( uint8_t pos, uint8_t value )
{
   (void) pos;

   moski_mode = value;
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

   buffer[0] = moski_mode;
   return 1;
}


/**
 * @brief Scheduler interrupt on timer1 overflow.
 */
ISR(SIG_OVERFLOW1)
{
   /* Do some scheduler stuff here. */

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
    * 1 kHz = 20 MHz / (2 * 8 * 1250)
    */
   TCCR1A = _BV(WGM10) | /* Phase and freq correct mode with OCR1A. */
         0; /* No actual PWM output. */
   TCCR1B = _BV(WGM13) | /* Phase and freq correct mode with OCR1A. */
         _BV(CS11); /* 8 prescaler */
   OCR1A  = 1250;
}


/**
 * @brief Entry point.
 */
int main (void)
{
   /* Disable watchdog timer since it doesn't always get reset on restart. */
   wdt_disable();

   /* Set up communication. */
   i2cs_setAddress( 0x09 );
   i2cs_setReadCallback( moski_read );
   i2cs_setWriteCallback( moski_write );
   i2cs_init();

   /* Set sleep mode. */
   set_sleep_mode( SLEEP_MODE_IDLE );

   /* Initialize the scheduler. */
   sched_init();

   /* Set up watchdog timer. */
   wdt_enable(WDTO_250MS);
   wdt_reset(); /* Just in case. */

   /* Set interrupts. */
   sei();

   /* Main loop. */
   while (1) {

      /* Atomic test to see if has anything to do. */
      cli();
      if (sched_flags != 0x00) {
         /* Activate interrupts again. */
         sei();

         /*
          * Run tasks.
          *
          * Very important that the periodicity of the scheduler update task
          *  allows this to finish or the loss of task execution may occur.
          */
         /* Motor task. */
         if (sched_flags & SCHED_MOTORS) {
         }
         /* Encoder task. */
         if (sched_flags & SCHED_ENCODER) {
         }
         /* Temp task. */
         if (sched_flags & SCHED_TEMP) {
         }

         /* Clear flags. */
         sched_flags = 0x00;
      }
      /* Reactivate interrupts and do nothing. */
      else
         sei();

      /* Sleep until next interrupt. */
      sleep_mode();
   }
}

