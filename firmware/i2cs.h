

#ifndef I2C_SLAVE_H
#  define I2C_SLAVE_H


#include <stdint.h>


/*
 * Configuration.
 */
/* Buffers. */
#define I2CS_TX_BUFFER_SIZE      0x40
/* Pins (ATtiny24). */
#define DDR_USI                  DDRA
#define PORT_USI                 PORTA
#define PIN_USI                  PINA
#define PORT_USI_SDA             PORTA6
#define PORT_USI_SCL             PORTA4
#define PIN_USI_SDA              PINA6
#define PIN_USI_SCL              PINA4
#define USI_START_COND_INT       USISIF


/*
 * Callbacks.
 */
/**
 * @brief i2c read callback.
 *
 * Handles reading data from the Master.
 *
 *    @param pos Position in the stream of the byte.
 *    @param value Actual value of the byte.
 *    @return 0 if wants to read more, otherwise not 0 to stop.
 */
typedef uint8_t (*i2cs_read_callback_t) (uint8_t pos, uint8_t value);
/**
 * @brief i2c write callback.
 *
 * Handles writing data to the master.
 *
 *    @param buf_len Maximum buffer length.
 *    @param buffer Buffer to fill for writing.
 *    @return The actual size of the buffer to write.
 */
typedef uint8_t (*i2cs_write_callback_t) (uint8_t buf_len, uint8_t *buffer);


/* 
 * Init/Exit.
 */
void i2cs_init (void);


/*
 * Set properties.
 */
void i2cs_setAddress( uint8_t address );


/* 
 * Set callbacks.
 */
void i2cs_setReadCallback( i2cs_read_callback_t callback );
void i2cs_setWriteCallback( i2cs_write_callback_t callback );


/*
 * Timeout.
 */
void i2cs_setTimeout( uint8_t top );
void i2cs_timeoutTick (void);


#endif /* I2C_SLAVE_H */

