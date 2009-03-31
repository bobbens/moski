

#ifndef MOSKI_H
#  define MOSKI_H


typedef enum moski_cmd_e {
   MOSKI_NULL     =  0x00,
   MOSKI_RESET    =  0x01,
   /* Motors. */
   MOSKI_SETMOTORS = 0x09,
   /* EEPROM. */
   MOSKI_EEREAD   =  0x30,
   MOSKI_EEWRITE  =  0x31
} moski_cmd_t;


typedef enum moski_eeprom_e {
   MOSKI_EE_OPERATION_MODE    = 0x00,
   /* Motor parameters. */
   MOSKI_EE_MOTOR_ACCEL_MAX   = 0x10,

} moski_eeprom_t;


#endif /* MOSKI_H */


