

#ifndef MOTORS_H
#  define MOTORS_H


#include <stdint.h>


/*
 * Initialization.
 */
void motors_init (void);


/*
 * Sets the motors.
 */
__inline void motors_control (void);
__inline void motors_set( int16_t motor_a, int16_t motor_b );


#endif /* MOTORS_H */


