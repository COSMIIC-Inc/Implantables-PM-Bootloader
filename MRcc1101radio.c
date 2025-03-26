/**
 * @file   MRcc1101radio.c
 * @author JDCC/JML
 * @brief Provides functions to configure radio, send and receive packets
 * @details
	Chipcon
	Product = CC1101
	Chip version = A   (VERSION = 0x04)
	Crystal accuracy = 10 ppm
	X-tal frequency = 26 MHz
	RF output power = -15 dBm
	
	RX filterbandwidth = 325K 
	Deviation = 47.6K	
	Datarate = 100k		
	Modulation = (1) GFSK
	Manchester enable = (0) Manchester disabled
	RF Frequency = 402.15M		
	Channel spacing = 299.9		
	IF Freq = 203.1K	
	Channel number = 0
	Optimization = Sensitivity
	Sync mode = (3) 30/32 sync word bits detected
	Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
	CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
	Forward Error Correction = (0) FEC disabled
	Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
	Packetlength = 63 (0x3F)
	Preamble count = (2)  4 bytes
	Append status = 1
	Address check = (1) Address check, no broadcast
	FIFO autoflush = 1
	Device address = 188 (0xBC)
	GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
	GDO2 signal selection = (41) CHIP_RDY
	GDO1 signal selection = (41) CHIP_RDY
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys.h"
#include "MRcc1101radio.h"


// -------- DEFINITIONS ----------

/* hardware specific defs */
#define SELECT_RADIO()					(IO1PIN_bit.P1_23 = 0)
#define DESELECT_RADIO()				(IO1PIN_bit.P1_23 = 1)

#define RADIO_IS_BUSY()					(IO0PIN_bit.P0_18 == 1)
#define TX_IS_BUSY()					RADIO_IS_BUSY()
#define RX_IS_READY()					RADIO_IS_BUSY()

#define MAKE_MISOPIN_BE_GPIO()			(PINSEL1_bit.P0_18 = 0)
#define MAKE_MISOPIN_BE_FUNCTION()		(PINSEL1_bit.P0_18 = 2)

#define CONNECT_RADIO_SPI_PINS()		{ PINSEL1_bit.P0_17 = 2;	/*SCK1*/	\
										PINSEL1_bit.P0_18 = 2;		/*MISO*/	\
										PINSEL1_bit.P0_19 = 2;		/*MOSI*/	\
										PINSEL1_bit.P0_20 = 2;		/*SSEL*/  }

#define CONFIG_RADIO_SPI_COMM()			{ S1SPCR_bit.MSTR =  1;		/*master*/	\
										S1SPCCR_bit.COUNTER = 8;	/*300KHz*/ }


/* normal defs */
enum				// Command strobes, PATABLE and FIFO addresses
{
	SRES	= 0x30,
	SRX		= 0x34,
	STX		= 0x35,
	SIDLE	= 0x36,
	SWOR	= 0x38, // updated from PM - JDC
	SFRX	= 0x3A,
	SFTX	= 0x3B,
	SNOP	= 0x3D,
	PATABLE	= 0x3E,
	SFIFO	= 0x3F
};

enum				// Status Register Addresses
{
	LQI		= 0x33,
	RSSI	= 0x34,
	TXBYTES	= 0x3A,
	RXBYTES	= 0x3B,
        MARCSTATE = 0x35  
};

enum CcRegister
{
    FSCTRL1  = 0x0B,  // frequency synthesizer control.
    FSCTRL0  = 0x0C,  // frequency synthesizer control.
    FREQ2    = 0x0D,  // frequency control word, high byte.
    FREQ1    = 0x0E,  // frequency control word, middle byte.
    FREQ0    = 0x0F,  // frequency control word, low byte.
    MDMCFG4  = 0x10,  // modem configuration.
    MDMCFG3  = 0x11,  // modem configuration.
    MDMCFG2  = 0x12,  // modem configuration.
    MDMCFG1  = 0x13,  // modem configuration.
    MDMCFG0  = 0x14,  // modem configuration.
    CHANNR   = 0x0A,  // channel number.
    DEVIATN  = 0x15,  // modem deviation setting (when fsk modulation is enabled).
    FREND1   = 0x21,  // front end rx configuration.
    FREND0   = 0x22,  // front end tx configuration.
	MCSM2    = 0x16,  // main radio control state machine configuration. - JDC
    MCSM1    = 0x17,  // main radio control state machine configuration. - JDC
    MCSM0    = 0x18,  // main radio control state machine configuration.
    FOCCFG   = 0x19,  // frequency offset compensation configuration.
    BSCFG    = 0x1A,  // bit synchronization configuration.
    AGCCTRL2 = 0x1B,  // agc control.
    AGCCTRL1 = 0x1C,  // agc control.
    AGCCTRL0 = 0x1D,  // agc control.
    FSCAL3   = 0x23,  // frequency synthesizer calibration.
    FSCAL2   = 0x24,  // frequency synthesizer calibration.
    FSCAL1   = 0x25,  // frequency synthesizer calibration.
    FSCAL0   = 0x26,  // frequency synthesizer calibration.
    FSTEST   = 0x29,  // frequency synthesizer calibration.
    TEST2    = 0x2C,  // various test settings.
    TEST1    = 0x2D,  // various test settings.
    TEST0    = 0x2E,  // various test settings.
    FIFOTHR  = 0x03,  // rxfifo and txfifo thresholds.
    IOCFG2   = 0x00,  // gdo2 output pin configuration.
    IOCFG1   = 0x01,  // gdo1 output pin configuration.
    IOCFG0   = 0x02,  // gdo0 output pin configuration. refer to smartrf® studio user manual for detailed pseudo register explanation.
    SYNC1    = 0x04,  // MSB.
    SYNC0    = 0x05,  // LSB.
    PKTCTRL1 = 0x07,  // packet automation control.
    PKTCTRL0 = 0x08,  // packet automation control.
    MYADDR   = 0x09,  // device address.
				      // no wake on radio configuration included - JDC
    PKTLEN   = 0x06   // packet length.
};

struct CcInit
{
    enum CcRegister regNum;
    uint8 value;
};


// --------   DATA   ------------

struct CcInit ccInitTable[] =
{

	FSCTRL1,  0x08,  //frequency synthesizer control.
	FSCTRL0,  0x00,  //frequency synthesizer control.
	FREQ2,    0x0F,  //frequency control word, high byte.
	FREQ1,    0x77,  //frequency control word, middle byte.
	FREQ0,    0xA2,  //frequency control word, low byte.
	MDMCFG4,  0x5B,  //modem configuration.
	MDMCFG3,  0xF8,  //modem configuration.
	MDMCFG2,  0x13,  //modem configuration.
	MDMCFG1,  0x23,  //modem configuration.
	MDMCFG0,  0x7A,  //modem configuration.
	CHANNR,   0x05,  //channel number.
	DEVIATN,  0x47,  //modem deviation setting (when fsk modulation is enabled).
	FREND1,   0xB6,  //front end rx configuration.
	FREND0,   0x10,  //front end tx configuration.
	MCSM0,    0x18,  //main radio control state machine configuration.
	FOCCFG,   0x1D,  //frequency offset compensation configuration.
	BSCFG,    0x1C,  //bit synchronization configuration.
	AGCCTRL2, 0xC7,  //agc control.
	AGCCTRL1, 0x00,  //agc control.
	AGCCTRL0, 0xB2,  //agc control.
	FSCAL3,   0xEA,  //frequency synthesizer calibration.
	FSCAL2,   0x2A,  //frequency synthesizer calibration.
	FSCAL1,   0x00,  //frequency synthesizer calibration.
	FSCAL0,   0x1F,  //frequency synthesizer calibration.
	FSTEST,   0x59,  //frequency synthesizer calibration.
	TEST2,    0x88,  //various test settings.
	TEST1,    0x31,  //various test settings.
	TEST0,    0x0B,  //various test settings. - was 0x09 in boot2
	FIFOTHR,  0x07,  //rxfifo and txfifo thresholds.
	IOCFG2,   0x29,  //gdo2 output pin configuration.
	IOCFG0,  0x06,   //gdo0 output pin configuration. refer to smartrf® studio user manual for detailed pseudo register explanation.
	PKTCTRL1, 0x0D,  //packet automation control. - was 0x05 in boot2
	PKTCTRL0, 0x05,  //packet automation control.
	MYADDR,   BOOTCODE_ADDRESS,  //device address.
	PKTLEN,   0x3F,	 //packet length.
        MCSM1,    0x3C //RXOFF_MODE is "stay in RX"
};


#define NUM_CCINIT_REGS		(sizeof( ccInitTable ) / sizeof( struct CcInit ))

uint32 comm_tRef;


// -------- PROTOTYPES ----------
static uint8 readStatusRegister( uint8 regNum );

static uint8 readRegisters( uint8 regAddr, uint8 *buf, uint8 numRegisters );
static uint8 writeRegisters( uint8 regAddr, uint8 *buf, uint8 numRegisters );

static uint8 sendHeader( uint8 regAddr, uint8 numRegisters );
static uint8 txRx_spi1( uint8 d );

 
//============================
//    GLOBAL CODE
//============================
#define SEND_COMMAND_STROBE_GET_RXFIFO(cmd)		readRegisters( (cmd), NULL, 0 )
#define SEND_COMMAND_STROBE_GET_TXFIFO(cmd)		writeRegisters( (cmd), NULL, 0 )

/* read status once and use macros to decipher it, to reduce 
	the number of cc reads since the reads affect the chip. see errata pdf */
#define GDO_TX_ACTIVE				6	//high during pkt transmission
#define GDO_RX_READY				7	// high at end of packet with good CRC

#define RESET_COMM_TIMEOUT()		resetTimeOut( &comm_tRef )
#define COMM_TIMED_OUT(to)			isTimedOut( &comm_tRef, (to) )

/**
 * @brief Initialize configuration registers on CC1101 radio using ccInit structure and enables receiving
 *        Sets TX power level
 */
void initRadioConfig( void )
{
	// force init since the magnet reset won't affect the radio
	
	uint8 ccValue;
	struct CcInit *reg;
	
	
	/* Connect Spi1 Pins for radio comm */
	CONNECT_RADIO_SPI_PINS();
	
	/* reset radio */
	DESELECT_RADIO();
	wait_usec( 100 );
	SEND_COMMAND_STROBE_GET_RXFIFO( SRES );
	
	/* copy configuration registers */
	for( reg = ccInitTable ; reg < &ccInitTable[ NUM_CCINIT_REGS ] ; reg++ )
		writeRegisters( reg->regNum, &reg->value, 1 );

	/* set output Power level: see table 30 in CC1101 data sheet (swrs061c) */
	ccValue = 0x60;						// @434MHz: $1d=-15dBm,$60=0dBm,$84=5dBm 
	writeRegisters( PATABLE, &ccValue, 1 );
	
 #ifdef DEBUG
	/* verify configuration registers */
	for( reg = ccInitTable ; reg < &ccInitTable[ NUM_CCINIT_REGS ] ; reg++ )
	{
		readRegisters( reg->regNum, &ccValue, 1 );
		if( ccValue != reg->value )
			ccValue++;				//trap error here
	}
 #endif
	
	enableRadioReceiver();
}

/**
 * @brief Enables Receive mode on CC1101
 *
 * @details  CC1101 reverts to IDLE after receiving and sending packets.  This 
 *           should be called every time after a packet is sent or received
 *     
 */
void enableRadioReceiver( void )
{
          
  // some states like oflo/underflow need special commands to exit
	
	uint8 cmd = GDO_RX_READY;
        uint16 delay = 20;
        
        while (delay-- > 0)
        {
        }
	
	/* program the SO status pin for RXdone */
	writeRegisters( IOCFG1, &cmd, 1 );
	
	/* force Idle state then goto receive mode */
	SEND_COMMAND_STROBE_GET_RXFIFO( SIDLE );
	SEND_COMMAND_STROBE_GET_RXFIFO( SFRX );
	SEND_COMMAND_STROBE_GET_TXFIFO( SFTX );
	
	SEND_COMMAND_STROBE_GET_RXFIFO( SRX );	
}

/**
 * @brief Send a radio packet via CC1101
 *
 * @details  The radio reverts to IDLE after sending the packet.  
 *      This function is blocking until the packet is sent
 *
 * @param deviceID      radio address of receiver
 * @param *data         data (up to 60 bytes) to send.  Does not include pktlen, pktaddr, rssi, and lqi 
 * @param len           length of data buffer (1-60 bytes)
 */
void sendRadioPacket( uint8 deviceID, uint8 *data, uint8 len )
{
	uint8 header[2], cmd = GDO_TX_ACTIVE ;
	
        /* force Idle state and clear FIFOs*/
        SEND_COMMAND_STROBE_GET_RXFIFO( SIDLE );
	SEND_COMMAND_STROBE_GET_RXFIFO( SFRX );
	SEND_COMMAND_STROBE_GET_TXFIFO( SFTX );
        
	/* program the SO status pin for TXdone */
	writeRegisters( IOCFG1, &cmd, 1 );
	
	/* build header */
	header[0] = len + 1 ;
	header[1] = deviceID ;
	
	/* load Tx buffer */
	writeRegisters( SFIFO, header, 2 );
	writeRegisters( SFIFO, data, len );
	
	/* enable transmitter */
	SEND_COMMAND_STROBE_GET_TXFIFO( STX );
	
	/* catch trailing edge of tx busy */
	MAKE_MISOPIN_BE_GPIO();
	
	while( !TX_IS_BUSY() );		//autocal 1000us
	while(  TX_IS_BUSY() );		//packet 300us
}

/**
 * @brief Gets a radio packet from CC1101 if available
 *
 * @details using variable length, addressed messages, with status (RSSI/LQI) appended:
 *       pktlen + pktaddr + data + rssi + lqi
 *      Messages with a bad CRC are autoflushed and will not be read here
 *      The radio reverts to IDLE after receiving a packet.  RX must be re-enabled to 
 *      receive more packets.
 *
 * @param *data    data (up to 64 bytes) read from packet including pktlen, pktaddr, rssi, and lqi
 *
 * @return number of total bytes in received packet(1 to 64 bytes, but only valid if >= MIN_PACKET_LENGTH).
 *              Otherwise 0 if none or w/errors
 */
uint8 getRadioPacket( uint8 *data )
{

	uint8 len = 0 ;
	
       
	/* wait for packet */
	MAKE_MISOPIN_BE_GPIO();
	
	if( RX_IS_READY() )
	{
		/* read all data */
		len = readStatusRegister( RXBYTES );
		readRegisters( SFIFO, data, len );
	}
	
	return len;
}


//============================
//    LOCAL CODE
//============================

/**
 * @brief Read status register from cc1101 device 
 *
 * @param regNum      CC1101 register to read
 */
static uint8 readStatusRegister( uint8 regNum )
{
	uint8 status;
	
	readRegisters( regNum | 0xC0, &status, 1 );
	
	return status;
}

/**
 * @brief Read config registers from cc1101 device starting at specified address
 *
 * @param regAddr       starting CC1101 register
 * @param *buf          buffer to read from registers
 * @param numRegisters number of registers
 *
 * @return CC1101 status
*/
static uint8 readRegisters( uint8 regAddr, uint8 *buf, uint8 numRegisters )
{
	uint8 status;
	
	SELECT_RADIO();
	
	status = sendHeader( (regAddr | BIT7), numRegisters );
	
	for( ; numRegisters > 0 ; numRegisters-- )
		*buf++ = txRx_spi1( 0x41 );
		
	DESELECT_RADIO();
	
	return status;
}

/**
 * @brief write config registers to cc1101 device starting at specified address
 *
 * @param regAddr       starting CC1101 register
 * @param *buf          buffer to write into registers
 * @param numRegisters number of registers
 *
 * @return CC1101 status
*/
static uint8 writeRegisters( uint8 regAddr, uint8 *buf, uint8 numRegisters )
{
	uint8 status;
	
	SELECT_RADIO();
	
	status = sendHeader( regAddr, numRegisters );
	
	for( ; numRegisters > 0 ; numRegisters-- )
		status = txRx_spi1( *buf++ );
		
	DESELECT_RADIO();
	
	return status;
}

/**
 * @brief Prepare and send header to radio chip.  Used prior to reading/writing 
 *        multiple registers.
 *
 * @param regAddr       starting CC1101 register 
 * @param numRegisters  number of registers
 *
 * @return CC1101 status
*/
static uint8 sendHeader( uint8 regAddr, uint8 numRegisters )
{
	uint8 header, burst;
	
	/* build header info */
	burst = (numRegisters > 1)?  BIT6 : 0 ;
	header = regAddr | burst ;
	
	/* wait for the radio to be ready by reading SOut low after CS */
	MAKE_MISOPIN_BE_GPIO();
	
	while( RADIO_IS_BUSY() ) ;
	
	/* config spi master, send header, read data */
	MAKE_MISOPIN_BE_FUNCTION();
	
	CONFIG_RADIO_SPI_COMM();
	
	return txRx_spi1( header );
}

//============================
//    INTERRUPT SERVICE ROUTINES
//============================


//============================
//    HARDWARE SPECIFIC CODE
//============================
#define SPI1_DONE()	(S1SPSR_bit.SPIF == 1)

/**
 * @brief  Read/Write one byte via the SPI port, full duplex.  
 * Assumes SPCR & SPSR are configured.
 *
 * @return read byte or chip status
 */
static uint8 txRx_spi1( uint8 d )
{

    S1SPDR = d;						        // put tx data
    while( ! SPI1_DONE() );					// wait for xfer done
    return( S1SPDR );						// get rx data 
}




