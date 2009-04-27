
#ifndef MOSKI_H
#  define MOSKI_H


/*
 * Moski modes of operation.
 */
#define MOSKI_MODE_CONTROL    0x01 /**< Default control mode. */
#define MOSKI_MODE_CONFIG     0x02 /**< EEPROM configuration mode. */
#define MOSKI_MODE_BOOTLOADER 0x03 /**< Self programming bootloader mode. */
#define MOSKI_MODE_SLEEP      0x09 /**< Deep sleep mode. */


/*
 * Data positions.
 */
#define MOSKI_EEPROM_DEFAULT  0x40 /**< Default operating mode. */
#define MOSKI_EEPROM_ADDRESS  0x41 /**< Default I2C address. */


#endif /* MOSKI_H */

