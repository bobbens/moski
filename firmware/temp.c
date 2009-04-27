
#include "global.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdint.h>


static volatile uint16_t cur_temp = 0; /**< Current temperature. */


/**
 * @brief Sets up the ADC.
 */
void temp_init (void)
{
   ADMUX = /*_BV(REFS1) | _BV(REFS0) |*/ /* Reference is Vcc. */
           _BV(MUX5) | _BV(MUX1); /* Target the internal temp sensor. */ 

   ADCSRA = _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | /* Set prescaler to 128. */
            _BV(ADIE) | /* Enable interrupts. */
            _BV(ADEN); /* Enable ADC. */

   ADCSRB = /*_BV(ADTS2) | _BV(ADTS1) | _BV(ADTS0) |*/ /* Free run mode. */
            0;
}


/**
 * @brief Starts an ADC conversion.
 */
void temp_start (void)
{
   ADCSRA |= _BV(ADSC);
}


/**
 * @brief Gets the current temperature.
 *
 *    @return The current temperature.
 */
uint16_t temp_get (void)
{
   uint16_t tmp;

   tmp = cur_temp;

   /** @todo FIR */

   return tmp;
}


/**
 * @brief ADC conversion complete interrupt.
 */
ISR(SIG_ADC)
{
   cur_temp = (ADCH<<8) + ADCL;
}

