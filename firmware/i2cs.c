
#include "i2cs.h"

#include <avr/io.h>
#include <avr/interrupt.h>


/*
 * Current state.
 */
typedef enum i2cs_state_e {
   I2CS_STATE_IDLE, /**< State machine isn't doing anything. */
   I2CS_STATE_CHECK_ADDRESS, /**< Checking address to see if it involves self. */
   I2CS_STATE_CHECK_REPLY_FROM_SEND_DATA, /**< Checking to see if send data worked. */
   I2CS_STATE_SEND_DATA, /**< Send data. */
   I2CS_STATE_REQUEST_REPLY_FROM_SEND_DATA, /**< Ask to see if send data worked. */
   I2CS_STATE_REQUEST_DATA, /**< Request data. */
   I2CS_STATE_GET_DATA_AND_SEND_ACK /**< Get data. */
} i2cs_state_t;


static uint8_t i2cs_address = 0x00; /**< Default slave address. */
static i2cs_state_t i2cs_overflow_state = I2CS_STATE_IDLE; /**< Current state. */


/*
 * Callback functions.
 */
static i2cs_read_callback_t i2cs_read_callback    = 0x00; /**< Read callback. */
static i2cs_write_callback_t i2cs_write_callback  = 0x00; /**< Write callback. */


/*
 * Buffers.
 */
static uint8_t i2cs_rx_pos = 0x00; /**< Read position in stream. */
static uint8_t i2cs_tx_buf[ I2CS_TX_BUFFER_SIZE ]; /**< TX buffer. */
static uint8_t i2cs_tx_pos = 0x00; /**< Position within the TX buffer. */
static uint8_t i2cs_tx_len = 0x00; /**< Current length of data within the TX buffer. */



/*
 * Simplicity defines.
 */
#define SET_USI_TO_SEND_ACK() \
{ \
   USIDR    = 0; /* Prepare ACK. */ \
   DDR_USI |= _BV(PORT_USI_SDA); /* Set SDA as output. */ \
   USISR    = (0<<USI_START_COND_INT) | /* Clear all flags excet start cond. */ \
              (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) | \
              (0x0E<<USICNT0); /* Set USI counter to shift 1 bit. */ \
}
#define SET_USI_TO_SEND_NACK() \
{ \
   USIDR    = 0xFF; /* Prepare NACK. */ \
   DDR_USI |= _BV(PORT_USI_SDA); /* Set SDA as output. */ \
   USISR    = (0<<USI_START_COND_INT) | /* Clear all flags excet start cond. */ \
              (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) | \
              (0x0E<<USICNT0); /* Set USI counter to shift 1 bit. */ \
}
#define SET_USI_TO_READ_ACK() \
{ \
   DDR_USI &= ~_BV(PORT_USI_SDA); /* Set SDA as input. */ \
   USIDR    = 0; /* Prepare ACK. */ \
   USISR    = (0<<USI_START_COND_INT) | /* Clear all flags excet start cond. */ \
              (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) | \
              (0x0E<<USICNT0); /* Set USI counter to shift 1 bit. */ \
}
#define SET_USI_TO_I2C_START_CONDITION_MODE() \
{ \
   USICR = (1<<USISIE) | /* Enable start condition. */ \
           (0<<USIOIE) | /* Disable overflow interrupt. */ \
           (1<<USIWM1) | (0<<USIWM0) | /* Set USI in two-wire mode. */ \
           /* Shift register clock source = externel, positive edge. */ \
           (1<<USICS1) | (0<<USICS0) | (0<<USICLK) | (0<<USITC); \
   USISR = (0<<USI_START_COND_INT) | /* Clear all flags excet start cond. */ \
           (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) | (0x0<<USICNT0); \
}
#define SET_USI_TO_SEND_DATA() \
{ \
   DDR_USI |= _BV(PORT_USI_SDA); /* Set SDA as output. */ \
   USISR = (0<<USI_START_COND_INT) | /* Clear all flags excet start cond. */ \
         (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) | (0x0<<USICNT0); \
}
#define SET_USI_TO_READ_DATA() \
{ \
   DDR_USI &= ~_BV(PORT_USI_SDA); /* Set SDA as input. */ \
   USISR = (0<<USI_START_COND_INT) | /* Clear all flags excet start cond. */ \
         (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) | (0x0<<USICNT0); \
}


/*
 * Prototypes.
 */


/**
 * @brief Initializes the i2c slave mode.
 */
void i2cs_init (void)
{
   /* Set input/outputs. */
   PORT_USI |= _BV(PORT_USI_SCL); /* Set SCL high */
   PORT_USI |= _BV(PORT_USI_SDA); /* Set SDA high */
   DDR_USI  |= _BV(PORT_USI_SCL); /* Set SCL as output */
   DDR_USI  &= ~_BV(PORT_USI_SDA); /* Set SDA as input */

   /* Enable Start Condition Interrupt. Disable Overflow Interrupt. */
   USICR = _BV(USISIE) | _BV(USIOIE) |
           /* Set USI in Two-wire mode. No USI Counter overflow prior
            * to first Start Condition (potentail failure) */
           _BV(USIWM1) | /*_BV(USIWM0) |*/
           /* Shift Register Clock Source = External, positive edge */
           _BV(USICS1) | /*_BV(USICS0) | _BV(USICLK) |*/
           /*_BV(USITC) | */ 0;

   /* Clear flags and reset counter. */
   USISR = 0xF0;

   /* Flush the buffers. */
   i2cs_rx_pos = 0x00;
   i2cs_tx_pos = 0x00;
   i2cs_tx_len = 0x00;
}


/**
 * @brief ISR for the i2c start condition.
 */
ISR(SIG_USI_START)
{
   // Set default starting conditions for new TWI package
   i2cs_overflow_state = I2CS_STATE_CHECK_ADDRESS;
   DDR_USI  &= ~_BV(PORT_USI_SDA); /* Set SDA as input. */
   while ((PIN_USI & _BV(PORT_USI_SCL)) & !(USISR & _BV(USIPF))); /* Wait for SCL to go low to ensure the "Start Condition" has completed. */
   /* If a Stop condition arises then leave the interrupt to prevent waiting forever. */
   /* Enable Overflow and Start Condition Interrupt.
    * (Keep StartCondInt to detect RESTART) */
   USICR = (1<<USISIE) | (1<<USIOIE) |
           /* Set USI in Two-wire mode. */
           (1<<USIWM1) | (1<<USIWM0) |
           /* Shift Register Clock Source = External, positive edge. */
           (1<<USICS1) | (0<<USICS0) | (0<<USICLK) |
           (0<<USITC);
   /* Clear flags. */
   USISR = (1<<USI_START_COND_INT) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) |
           (0<<USICNT0); /* Set USI to sample 8 bits
                            i.e. count 16 external pin toggles. */
}


/**
 * @brief ISR for the i2c overflow.
 *
 * Basically three states:
 *  - address
 *  - write
 *  - read
 */
ISR(SIG_USI_OVERFLOW)
{
   switch (i2cs_overflow_state) {

      /*
       * Address mode.
       */

      /* Check to see if is talking to us. */
      case I2CS_STATE_CHECK_ADDRESS:
         if ((USIDR == 0) || (( USIDR>>1 ) == i2cs_address)) {
            /* We get to send data. */
            if (USIDR & 0x01) {
               i2cs_overflow_state = I2CS_STATE_SEND_DATA;
               i2cs_tx_pos = 0x00; /**< Reset buffer position. */
               /* Acquire data from function pointer. */
               i2cs_tx_len = i2cs_write_callback( I2CS_TX_BUFFER_SIZE, i2cs_tx_buf );
            }
            /* We recieve data. */
            else {
               i2cs_overflow_state = I2CS_STATE_REQUEST_DATA;
               i2cs_rx_pos = 0x00; /* Reset position. */
            }
            SET_USI_TO_SEND_ACK();
         }
         else
            SET_USI_TO_I2C_START_CONDITION_MODE();
         break;


     /*
      * Master read mode.
      *
      * We write to the master.
      */

     /* Check reply and go to I2CS_STATE_SEND_DATA if OK. */
     case I2CS_STATE_CHECK_REPLY_FROM_SEND_DATA:
         if (USIDR) { /* If NACK the master doesn't want more data. */
            SET_USI_TO_I2C_START_CONDITION_MODE();
            return;
         }

         /* Purpose fallthrough. */

     /* Send data. */
     case I2CS_STATE_SEND_DATA:
         /* Get data from buffer. */
         if (i2cs_tx_pos > i2cs_tx_len) { /* Out of data. */
            SET_USI_TO_I2C_START_CONDITION_MODE();
            return;
         }
         else { /* Has more data. */
            USIDR = i2cs_tx_buf[ i2cs_tx_pos ];
            i2cs_tx_pos++;
         }                                                                   
         i2cs_overflow_state = I2CS_STATE_REQUEST_REPLY_FROM_SEND_DATA;
         SET_USI_TO_SEND_DATA();
         break;

     /* Set the USI to get the reply from master.  Should check reply afterwards
      * with I2CS_STATE_CHECK_REPLY_FROM_SEND_DATA. */
     case I2CS_STATE_REQUEST_REPLY_FROM_SEND_DATA:
         i2cs_overflow_state = I2CS_STATE_CHECK_REPLY_FROM_SEND_DATA;
         SET_USI_TO_READ_ACK();
         break;


     /*
      * Master write mode.
      *
      * We read from the master.
      */

     /* Requests more data. */
     case I2CS_STATE_REQUEST_DATA:
         i2cs_overflow_state = I2CS_STATE_GET_DATA_AND_SEND_ACK;
         SET_USI_TO_READ_DATA();
         break;

     /* Copy incoming data and send ACK. */
     case I2CS_STATE_GET_DATA_AND_SEND_ACK:
         /* Copy data. */
         if (i2cs_read_callback( i2cs_rx_pos++, USIDR )) {
            SET_USI_TO_SEND_NACK();
            return;
         }

         i2cs_overflow_state = I2CS_STATE_REQUEST_DATA;
         SET_USI_TO_SEND_ACK();
         break;


      /*
       * Unused.
       */

      default:
         break;
   }
}


/**
 * @brief Sets the i2c slave address.
 *
 *    @param address Address to use as i2c slave.
 */
void i2cs_setAddress( uint8_t address )
{
   /* Save the address. */
   i2cs_address = address;
}


/**
 * @brief Sets the read callback.
 *
 *    @param callback Callback to set.
 */
void i2cs_setReadCallback( i2cs_read_callback_t callback )
{
   i2cs_read_callback = callback;
}


/**
 * @brief Sets the write callback.
 *
 *    @param callback Callback to set.
 */
void i2cs_setWriteCallback( i2cs_write_callback_t callback )
{
   i2cs_write_callback = callback;
}



