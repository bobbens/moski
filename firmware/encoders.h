

#ifndef ENCODERS_H
#  define ENCODERS_H


#include <stdint.h>


#define ENCODER_INT        PCIE0    /**< Global interrupt mask for encoders. */
#define ENCODER_MSK        PCMSK0   /**< Interrupt mask register for encoders. */
#define ENCODER_INT_A      PCINT0   /**< Interrupt pin encoder A is on. */
#define ENCODER_INT_B      PCINT1   /**< Interrupt pin encoder B is on. */
#define ENCODER_SIG        SIG_PIN_CHANGE0 /**< Interrupt vector for encoders. */

#define ENCODER_DDR        DDRA     /**< DDR register for encoders. */
#define ENCODER_PIN        PINA     /**< Pin encoders are on. */
#define ENCODER_PORT_A     PB0      /**< Encoder A port. */
#define ENCODER_PORT_B     PB1      /**< Encoder B port. */
#define ENCODER_PIN_A      ENCODER_PORT_A /**< Encoder A pin. */
#define ENCODER_PIN_B      ENCODER_PORT_B /**< Encoder B pin. */


/*
 * Initialization.
 */
void encoders_init (void);


/*
 * Current encoder values.
 */
extern volatile uint16_t encoder_a; /**< Encoder A counter. */
extern volatile uint16_t encoder_b; /**< Encoder B counter. */


#endif /* ENCODERS_H */


