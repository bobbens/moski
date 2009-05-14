
#ifndef MOSKI_H
#  define MOSKI_H


/*
 * Moski modes of operation.
 */
#define MOSKI_MODE_OPEN       0x01 /**< Open loop mode (no control). */
#define MOSKI_MODE_CONTROL    0x02 /**< Default control mode. */
#define MOSKI_MODE_CONFIG     0x03 /**< EEPROM configuration mode. */
#define MOSKI_MODE_BOOTLOADER 0x04 /**< Self programming bootloader mode. */
#define MOSKI_MODE_SLEEP      0x09 /**< Deep sleep mode. */


/*
 * Data positions.
 */
/* General parameters. */
#define MOSKI_EEPROM_DEFAULT  0x40 /**< Default operating mode. */
/* I2C parameters. */
#define MOSKI_EEPROM_ADDRESS  0x50 /**< Default I2C address. */
/* Motor Parameters. */
/* Motor A. */
#define MOSKI_MOTOR_A_KP      0x60 /**< Motor A Kp. */
#define MOSKI_MOTOR_A_KI      MOSKI_MOTOR_A_KP+2 /**< Motor A Ki. */
#define MOSKI_MOTOR_A_WINDUP  MOSKI_MOTOR_A_KP+4 /**< Motor A anti-windup. */
/* Motor B. */
#define MOSKI_MOTOR_B_KP      0x70 /**< Motor B Kp. */
#define MOSKI_MOTOR_B_KI      MOSKI_MOTOR_B_KP+2 /**< Motor B Ki. */
#define MOSKI_MOTOR_B_WINDUP  MOSKI_MOTOR_B_KP+4 /**< Motor B anti-windup. */


#endif /* MOSKI_H */

