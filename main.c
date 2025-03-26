/**
 * @file   main.c
 * @author JDCC/JML
 * @date   3/7/2014 (last update)
 *
 * @brief Main entry point for PM Radio Bootloader
 * @details
 *	This program enables applications to be boot loaded onto the power module via the 
 *	cc1101 radio system.  After 15 seconds of no activity it launches the app if a valid
 *	signature is found (vector table adds to zero).
 *
 *	Only the timer 0 and spi modules are used, without interrupts, to give the app 
 *	nearly hardware reset default conditions.  io pins and spi are configured for the 
 *	circuitry and are left configured for the app.  The timer0 is reset for the app.
 *
 *	xtal freq is assumed to be 10MHz and is set in sys.h. 
 *      The SVN revision and product revision are defined in sys.h.
 *
 *      Serial Number is located at 0x0003 C000 (Sector 16) and is 2 bytes - stored high byte, low byte.
 *      The code converts a 0xFF to 0x00 to indicate it is not programmed. Only one write is allowed.
 */

#include "sys.h"
#include "MRcc1101radio.h"
#include "boot2.h"


// -------- DEFINITIONS ----------

// --------   DATA   ------------
static uint32 inactiveSecs;


// -------- PROTOTYPES ----------
static void init_io( void );
static void runApp( void );

//============================
//    GLOBAL CODE
//============================
#define MAX_INACTIVITY				(TIMEOUT_sec( 15 ))
#define INACTIVITY_TIMEOUT()	 	(isTimedOut( &inactiveSecs, MAX_INACTIVITY ))
#define RESET_INACTIVITY_TIMEOUT() 	(resetTimeOut( &inactiveSecs ))


/**
 * @brief Main: This is the standard entry point for C code.  It is assumed that your code will call
 *               main() once you have performed all necessary initialization (modified cstartup file).
 *               NOTE: Power Module download requires address remapping application .bin to 0x00002000 
 *               
 */
main()
{
	init_io();
	initRadioConfig();
        NonVolatileRead();
	RESET_INACTIVITY_TIMEOUT();
	
	while(1)
	{
		if( runBootCommandTask() )
                {
                  RESET_INACTIVITY_TIMEOUT();
                }
		else if( INACTIVITY_TIMEOUT() )
		{
                  runApp();
			
                  RESET_INACTIVITY_TIMEOUT();
		}
	}			
}


#define APP_VECTOR_OFFSET		0x00002000
#define NUM_VECTORS				8


/**
 * @brief Starts the application residing at APP_VECTOR_OFFSET (0x0000 2000) if the 
 *        application is valid (indicate by zero sum interrupt vector space)
 */
static void runApp( void )
{
	// jump to valid app code, Else return

	uint32 *vector, vectorSum = 0;
	void (*app)( void );
	uint8 vectorCount = 0;
	
	
	vector = (uint32 *)APP_VECTOR_OFFSET ;
	
	for( ; vectorCount < NUM_VECTORS ; vectorCount++ )
		vectorSum += *vector++ ;
	
	if( vectorSum == 0 )
	{
		/* reset tmr0. the app assumes a hardware reset default */
		T0TCR	= 0;
		T0PR	= 0;
		T0PC	= 0;
		T0TC	= 0;

		app = (void (*)(void)) APP_VECTOR_OFFSET ;
		app();
	}
}



//==================================
//    CPU SUB-SYSTEMS UTILITY CODE
//==================================
/**
 * @brief waits specified microseconds
 * @param t_usec (uint32)
 */
void wait_usec( uint32 t_usec )
{
	uint32 tRef;

	tRef = T0TC;
	while( (T0TC - tRef) < t_usec );
}

/**
 * @brief Gets current time in microseconds
 * @return time in microseconds (uint32)
 */
uint32 getSystemTIme( void )
{
	// time in usec
	
	return T0TC;
}

/**
 * @brief Gets current time in microseconds
 * @param *tRef pointer to reference time (uint32 in microseconds)
 * @param alarm alarm time in microseconds
 * @return 1 if timed out, 0 if not
 */
uint8 isTimedOut( uint32 *tRef, uint32 alarm )
{
	// time in usec

	if( (getSystemTIme() - *tRef) > alarm )
		return 1;
	else
		return 0;
}

/**
 * @brief Resets reference time used in alarm
 * @param *tRef pointer to reference time (uint32 in microseconds)
 */
void resetTimeOut( uint32 *tRef )
{
	// time in usec

	*tRef = getSystemTIme();
}


//===============================
//    PM1B HARDWARE SPECIFIC CODE
//===============================


/**
 * @brief Initializes pins: pin function, direction, and values based on
 *  PRJ-NNPS-SYS-SPEC-20  Bootloader Specification,  Power Module, Pin Settings
 *  Also configures Timer0, used in inactivity timer
 * @details

    PORT 0.	DDR	VALUE	DESC
    0	1		TX 232
    1	1		RX 232
    2			I2C SCL
    3			I2C SDA
    4			SCK0, assigned to SPI0 with PINSEL0
    5			MISO0, assigned to SPI0 with PINSEL0
    6			MOSI0, assigned to SPI0 with PINSEL0
    7			SSEL0, assigned to SPI0 with PINSEL0
    8	1		PWMBT
    9			nRESET
    10	1		nSHDNBT3
    11	1		nSHDNCHG3
    12	1		nSHDNBT2
    13	1		nSHDNCHG2
    14			nBOOT
    15			ACCINT
    16			RADINT0
    17			SCK1, assigned to SPI1 with PINSEL1
    18			MISO1, assigned to SPI1 with PINSEL1
    19			MOSI1, assigned to SPI1 with PINSEL1
    20			SSEL1, assigned to SPI1 with PINSEL1
    21	1		unused
    22	1		unused
    23	1		nSHDNBT1
    24	1		nSHDNCHG1
    25			RXCAN1
    26			TXCAN1
    27			LOAD_AN, assigned to ANALOG IN with PINSEL1
    28			VREC_AN, assigned to ANALOG IN with PINSEL1
    29			VSYS_AN, assigned to ANALOG IN with PINSEL1
    30			VNET_AN, assigned to ANALOG IN with PINSEL1
    31			RESERVED

    PORT 1.	DDR	VALUE	DESC
    0-15	0	unused				
    16	1		THRM
    17	1	1	nCSRAM
    18			DISCHARGING
    19	1	1	CPLD10
    20			P1.20
    21	1	1	I2CS0
    22	1	1	I2CS1
    23	1	1	nCSRAD
    24	1		HBT
    25	1	1	nCSFLA
    26			RTCK_LPC
    27	JTAG
    28	JTAG
    29	JTAG
    30	JTAG
    31	JTAG
 */
static void init_io( void )
{
	// Assumes PLL off <use startup.s wizard and uncheck pll box> for cclk = 10MHz

        // Configure pin function first: see PRJ-NNPS-SYS-SPEC-20
	PINSEL0 = 0x00005550 ;  //SPI0 enabled, I2C enabled
	PINSEL1 = 0x154002A9 ;  //analog in enabled, SPI1 enabled

	// For remaining GPIO pins, configure pin value before data direction to prevent unnecessary state changes
	IO0PIN = 0x00000000	;
	IO0DIR = 0x01E03D03	;

	IO1PIN = 0x02EA0000	;
	IO1DIR = 0x03EB0000	; 

	/* Start Timer0 at 1MHZ */
	T0TCR = 0;
	T0PR = TO_PRESCALE -1;  // prescale counter increments on every 10th (TO_PRESCALE = 10)
	T0TCR = 1;
}


