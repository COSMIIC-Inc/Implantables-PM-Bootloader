//    boot2: .h     HEADER FILE.

#ifndef _BOOT2_H
#define _BOOT2_H


// -------- DEFINITIONS ----------
//The serial number is loaded via a "Just-in-Time" module with Flash Magic
#define SERIAL_NUMBER_LOCATION          0x00001FFE //last two bytes of sector 0
#define APP_SETTINGS_LOCATION           0x0003DE00 //must be in sector 16!


// --------   DATA   ------------

extern uint8 status;
extern uint16 serialNumber;

// -------- PROTOTYPES ----------
uint8 runBootCommandTask( void );
void NonVolatileRead( void );


#endif
 
