

#include "encoders.h"

#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>


volatile uint8_t encoder_a    = 0; /**< Counter on encoder A. */
static uint8_t encoder_pin_a  = 0; /**< Current status of encoder A. */
volatile uint8_t encoder_b    = 0; /**< Counter on encoder B. */
static uint8_t encoder_pin_b  = 0; /**< Current status of encoder B. */


/**
 * @brief Encoder interrupt. */
ISR(ENCODER_SIG)
{
   uint8_t inp;

   /* Check to see if encoder A changed. */
   inp = ENCODER_PIN & _BV(ENCODER_PIN_A);
   if (inp != encoder_pin_a) {
      encoder_pin_a = inp;
      encoder_a++;
   }

   /* Check to see if encoder B changed. */
   inp = ENCODER_PIN & _BV(ENCODER_PIN_B);
   if (inp != encoder_pin_b) {
      encoder_pin_b = inp;
      encoder_b++;
   }
}


/**
 * @brief Sets up the encoders.
 */
void encoders_init (void)
{
   /* Set pins as input. */
   ENCODER_DDR &= ~(_BV(ENCODER_PORT_A) | _BV(ENCODER_PORT_B));
   encoder_pin_a = (ENCODER_PIN & _BV(ENCODER_PIN_A));
   encoder_pin_b = (ENCODER_PIN & _BV(ENCODER_PIN_B));

   /* Set up interrupts. */
   GIMSK       |= ENCODER_INT; /* Globally enable pin change interrupts. */
   ENCODER_MSK |= _BV(ENCODER_INT_A) | _BV(ENCODER_INT_B); /* Enabled encoder interrupts. */
   MCUCR       |= /*_BV(ISC01) |*/ _BV(ISC00); /* Set on rise/falling edge. */
}


