

/* Set clock speed. */
#define F_CPU 1000000UL


#include <avr/io.h>
#include <util/delay.h>


static void delay (void)
{
   _delay_ms( 3000. );
   _delay_ms( 3000. );
   _delay_ms( 3000. );
   _delay_ms( 3000. );
   _delay_ms( 3000. );
}


/**
 * @brief Entry point.
 */
int main (void)
{
   /* Output for PWM. */
   DDRB  = _BV(DDB2);
   /* Output for direction. */
   DDRA  = _BV(DDA3);

   while (1) {
      /* Motors off. */
      PORTB = 0;
      PORTA = 0;
      delay();

      /* Motors forward. */
      PORTB = _BV(PB2);
      PORTA = 0;
      delay();

      /* Motors off. */
      PORTB = 0;
      PORTA = 0;
      delay();

      /* Motors forward. */
      PORTB = 0;
      PORTA = _BV(PA3);
      delay();
   }
}

