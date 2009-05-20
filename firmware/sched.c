
#include "sched.h"

#include "global.h"

#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "moski_conf.h"
#include "motors.h"
#include "temp.h"


/*
 * Scheduler.
 */
volatile uint8_t sched_flags = 0x00; /**< Scheduler flags. */


/*
 * Runs the scheduler.
 */
__inline void sched_run (void)
{
   uint8_t flags;

   /* Atomic store temporary flags and reset real flags in case we run a bit late. */
   cli();
   flags = sched_flags;
   sched_flags = 0;
   sei();

   /*
    * Run tasks.
    *
    * Very important that the periodicity of the scheduler update task
    *  allows this to finish or the loss of task execution may occur.
    */
   /* Motor task. */
   if (flags & SCHED_MOTORS)
      motors_control();
#if MOSKI_USE_TEMP
   /* Temp task. */
   if (flags & SCHED_TEMP)
      temp_start(); /* Start temperature conversion. */
#endif /* MOSKI_USE_TEMP */
}

