//    cc1101radio: .h     HEADER FILE.

#ifndef _CC1101_RADIO_H
#define _CC1101_RADIO_H


// -------- DEFINITIONS ----------
#define BOOTCODE_ADDRESS	0xBC
#define TOWER_ADDRESS		0xBD


// --------   DATA   ------------


// -------- PROTOTYPES ----------
void initRadioConfig( void );
void sendRadioPacket( uint8 deviceID, uint8 *data, uint8 len );
uint8 getRadioPacket( uint8 *data );
void enableRadioReceiver( void );


#endif
 
