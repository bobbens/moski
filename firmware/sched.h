

#ifndef SCHED_H
#  define SCHED_H


#include <stdint.h>


/*
 * Scheduler.
 */
#define SCHED_MOTORS    (1<<0) /**< Run motor update task. */
#define SCHED_TEMP      (1<<1) /**< Run temp update task. */
extern volatile uint8_t sched_flags; /**< Scheduler flags. */


__inline void sched_run (void);


#endif /* SCHED_H */
