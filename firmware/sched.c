
#include "sched.h"

#include "global.h"

#include <stdint.h>

#include <avr/io.h>

#include "motors.h"
#include "temp.h"


/*
 * Scheduler.
 */
uint8_t sched_flags = 0x00; /**< Scheduler flags. */


/*
 * Runs the scheduler.
 */
__inline void sched_run (void)
{
   /*
    * Run tasks.
    *
    * Very important that the periodicity of the scheduler update task
    *  allows this to finish or the loss of task execution may occur.
    */
   /* Motor task. */
   if (sched_flags & SCHED_MOTORS) {
      sched_flags &= ~SCHED_MOTORS; /* Clear flag. */
      /* Run after clearing flag in case needs to run again. */
      motors_update();
   }
   /* Temp task. */
   if (sched_flags & SCHED_TEMP) {
      sched_flags &= ~SCHED_TEMP; /* Clear flag. */
      /* Run after clearing flag in case needs to run again. */
      temp_start(); /* Start temperature conversion. */
   }
}

