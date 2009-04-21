
#include "moski.h"

#include <avr/io.h>
#include <util/delay.h>

#include <stdint.h>

#include "i2cs.h"


static uint8_t moski_mode = 0x00; /**< Current operating mode. */


/*
 * Prototypes.
 */
static uint8_t moski_read( uint8_t pos, uint8_t value );
static uint8_t moski_write( uint8_t buf_len, uint8_t *buffer );


/**
 * @brief Master read function for the Moski.
 *
 *    @param pos Position of byte read.
 *    @param value Value of byte read.
 *    @return 0 to send ACK, 1 to send NACK.
 */
static uint8_t moski_read( uint8_t pos, uint8_t value )
{
   (void) pos;

   moski_mode = value;
   return 0;
}


/**
 * @brief Master write function for the Moski.
 *
 *    @param buf_len Maximum buffer length.
 *    @param buffer Buffer to write.
 *    @return Number of bytes to write.
 */
static uint8_t moski_write( uint8_t buf_len, uint8_t *buffer )
{
   (void) buf_len;

   buffer[0] = moski_mode;
   return 1;
}


/**
 * @brief Entry point.
 */
int main (void)
{
   /* Set up communication. */
   i2cs_setAddress( 0x09 );
   i2cs_setReadCallback( moski_read );
   i2cs_setWriteCallback( moski_write );
   i2cs_init();

   while (1) {}
}

