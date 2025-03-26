//    file: sys.h    System (common) HEADER FILE.

#ifndef _SYS_H
#define _SYS_H


// -------- DEFINITIONS ----------
typedef signed char 		int8;
typedef short 				int16;
typedef int 				int32;

typedef unsigned char 		uint8;
typedef unsigned short 		uint16;
typedef unsigned int 		uint32;

typedef union
{	uint8  c[2];
	uint16 s;
	int16  i;
}Bw16;

typedef union
{	uint8  c[4];
	uint16 s[2];
	uint32 w;
}Bw32;


#define TRUE    1
#define FALSE   0
 
#define ON      1
#define OFF     0

#define CLR_BITS(a,b)     ((a)&=~(b))
#define SET_BITS(a,b)     ((a)|=(b))
#define BITS_TRUE(a,b)    ((a)&(b))

#define B(n)			(1<<(n))

#define BIT0			B(0)
#define BIT1			B(1)
#define BIT2			B(2)
#define BIT3			B(3)
#define BIT4			B(4)
#define BIT5			B(5)
#define BIT6			B(6)
#define BIT7			B(7)

#define BIT8			B(8)
#define BIT9			B(9)
#define BIT10			B(10)
#define BIT11			B(11)
#define BIT12			B(12)
#define BIT13			B(13)
#define BIT14			B(14)
#define BIT15			B(15)

#define BIT16			B(16)
#define BIT17			B(17)
#define BIT18			B(18)
#define BIT19			B(19)
#define BIT20			B(20)
#define BIT21			B(21)
#define BIT22			B(22)
#define BIT23			B(23)

#define BIT24			B(24)
#define BIT25			B(25)
#define BIT26			B(26)
#define BIT27			B(27)
#define BIT28			B(28)
#define BIT29			B(29)
#define BIT30			B(30)
#define BIT31			B(31)


/******************************************************************************
	PROJECT SPECIFIC HEADER
******************************************************************************/ 

// -------- DEFINITIONS ----------
#define FOSC_KHZ		10000L
#define TO_RESOL        256
#define TO_PRESCALE     10

/* timeout_usec conversion factor ASSUMES pclk = cclk/4 */
#define TOCNV_us	(FOSC_KHZ * TO_RESOL / (4000 * TO_PRESCALE))

#define TIMEOUT_us(n)	((n) * TOCNV_us / TO_RESOL)		
#define TIMEOUT_ms(n)	(TIMEOUT_us((n) * 1000))		
#define TIMEOUT_sec(n)	(TIMEOUT_us((n) * 1000000))

#define SVN_REVISION                    47
#define PRODUCT_REVISION                2


// -------- PROTOTYPES ----------
uint32 getSystemTIme( void );
void wait_usec( uint32 t_usec );
uint8 isTimedOut( uint32 *tRef, uint32 alarm );
void resetTimeOut( uint32 *tRef );


// --------   DATA   ------------
//extern uint8 sys_error;
 #define CMD_ERR		0x02
 #define RXD_ERR		0x04
 #define CALIB_ERR		0x20
 #define EE_ERR			0x40
 #define PON_ERR		0x80


/* modify iar file to add R/W attrib to IOnPIN registers */
#include "iolpc2129_iopin_rw.h"


#endif
 
