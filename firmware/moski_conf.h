

#ifndef MOSKI_CONF_H
#  define MOSKI_CONF_H


#include <avr/io.h>


/*
 * Enabled subsystems.
 */
#define MOSKI_USE_TEMP     0


/*
 * Encoder details.
 */
#define ENCODER_INT        PCIE0    /**< Global interrupt mask for encoders. */
#define ENCODER_MSK        PCMSK0   /**< Interrupt mask register for encoders. */
#define ENCODER_INT_A      PCINT0   /**< Interrupt pin encoder A is on. */
#define ENCODER_INT_B      PCINT1   /**< Interrupt pin encoder B is on. */
#define ENCODER_SIG        SIG_PIN_CHANGE0 /**< Interrupt vector for encoders. */

#define ENCODER_DDR        DDRA     /**< DDR register for encoders. */
#define ENCODER_DD_A       DDA0     /**< Encoder A dd pin. */
#define ENCODER_DD_B       DDA1     /**< Encoder A dd pin. */
#define ENCODER_PORT       PORTA    /**< PORT register for encoders. */
#define ENCODER_PORT_A     PA0      /**< Encoder A port. */
#define ENCODER_PORT_B     PA1      /**< Encoder B port. */
#define ENCODER_PIN        PINA     /**< Pin encoders are on. */
#define ENCODER_PIN_A      PINA0 /**< Encoder A pin. */
#define ENCODER_PIN_B      PINA1 /**< Encoder B pin. */

#define ENCODER_NUM        2 /**< Number of areas on the encodor. */


#endif /* MOSKI_CONF_H */

