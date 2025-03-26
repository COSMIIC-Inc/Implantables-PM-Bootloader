//    boot2: .c    .

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys.h"
#include "boot2.h"
#include "cc1101radio.h"


// -------- DEFINITIONS ----------
typedef void (*IAP)(unsigned int [],unsigned int[]);

#define IAP_LOCATION 	0x7ffffff1


// --------   DATA   ------------
IAP runIapCommand = (IAP) IAP_LOCATION;


// -------- PROTOTYPES ----------
static uint8 executeBootCommand( uint8 *commandBuffer, uint8 len );

static void eraseFlash_cmd( uint8 *cmd );
static void writeFlash_cmd( uint8 *cmd );
static void writeFlash_data( uint8 *cmd );
static void readFlash_cmd( uint8 *cmd );
static void readStatus_cmd( uint8 *cmd );

static uint8 IAP_prepareSectorsForWrite( uint8 start, uint8 end );
static uint8 IAP_eraseSectors( uint8 start, uint8 end );
static uint8 IAP_copyRamToFlash( uint32 flashAddr, uint32 ramAddr, uint32 numBytes );
static void wrImageData( uint8 *data, uint8 len );

static uint32 getWordBytes( uint8 *data );
static uint16 getShortBytes( uint8 *data );

 
//============================
//    GLOBAL CODE
//============================
#define MAX_COMMAND_BUFFER	80

uint8 runBootCommandTask( void )
{
	// run commands and return 1=active, else 0
	
	uint8 status=0, commandBuffer[ MAX_COMMAND_BUFFER ], len;
	
	
	if( (len = getRadioPacket( commandBuffer)) > 0 )
	{
		if( executeBootCommand( commandBuffer, len )  )
			status = 1 ;
			
		enableRadioReceiver();
	}
	
	return status ;
}


//============================
//    LOCAL CODE
//============================
enum
{
	WRITE_FLASH_CMD			= 0x10,
	WRITE_FLASH_DATA		= 0x11,
	WRITE_FLASH_RESPONSE	= 0x20,
	
	UNRECOGNIZED_CMD		= 0x21,
	
	ERASE_FLASH_CMD			= 0x12,
	ERASE_FLASH_RESPONSE	= 0x22,
	
	READ_FLASH_CMD			= 0x13,
	READ_FLASH_DATA			= 0x23,
	
	QUERY_STATUS_CMD		= 0x14,
	QUERY_STATUS_RESPONSE	= 0x24
};
	
static uint8 executeBootCommand( uint8 *commandBuffer, uint8 len )
{
	// return 1 if command code was valid, else 0
	
	uint8 response, *cmdId=(commandBuffer + 2);
	
	
	if( *cmdId == ERASE_FLASH_CMD )
		 eraseFlash_cmd( commandBuffer );
	
	else if( *cmdId == WRITE_FLASH_CMD )
		 writeFlash_cmd( commandBuffer );
	
	else if( *cmdId == WRITE_FLASH_DATA )
		 writeFlash_data( commandBuffer );
	
	else if( *cmdId == READ_FLASH_CMD )
		 readFlash_cmd( commandBuffer );
	
	else if( *cmdId == QUERY_STATUS_CMD )
		 readStatus_cmd( commandBuffer );
	
	else
	{
		response = UNRECOGNIZED_CMD;
		sendRadioPacket( TOWER_ADDRESS, &response, 1 );
		return 0;
	}

	return 1;
}


//============================
// 	 BOOT2 COMMANDS
//============================
#define SECTOR_MIN				1		//our boot code resides in lower sectors
#define SECTOR_MAX				16		// ISP/IAP factory boot code in top sector

#define FLASH_WR_BLOCK_SIZE		512		// options are: 512/1k/4k/8k
#define BOUNDRY_512				512		// flash write address boundry mask
#define WR_ADDR_MIN				0x02000
#define WR_ADDR_MAX				(0x3E000 - FLASH_WR_BLOCK_SIZE)

#define MAX_SEND_DATA			59

static void eraseFlash_cmd( uint8 *cmd )
{
	// pktlen, pktaddr, CMDid, Start sector(1byte), End sector(1byte)
	// sectors =4-16
	
	/* use byte values so no padding by compiler */
	struct Pkt {uint8 pktLen, pktAddr, cmdId, start, end;} *msg = (struct Pkt *)cmd ;
	uint8 bootResponse[2] = {ERASE_FLASH_RESPONSE, 0} ;


	if(	msg->pktLen == 4
	&&	IAP_prepareSectorsForWrite( msg->start, msg->end )
	&&	IAP_eraseSectors( msg->start, msg->end )  )
	{
		/* let programmer change to receiver */
		wait_usec( 1500 );
		
		/* erase is done, ack the programmer */
		bootResponse[1] = 1;
	}
		
	sendRadioPacket( TOWER_ADDRESS, bootResponse, 2 );
}

//---------------
struct WrCtrl
{
	uint32 addr;
	uint8  *data;
	uint16 len;
	uint8  sector;

} static wrCtrl;


#pragma data_alignment=4
static uint8 wrImage[ FLASH_WR_BLOCK_SIZE ];


static void writeFlash_cmd( uint8 *cmd )
{
	// pktlen, pktaddr, CMDid,addr(4byte),len(1byte),sector number(1byte),data(1byte)...
	// addr =32k-248k, len =1-128,sectorNumber =1-16
	
	/* use byte values so no padding by compiler */
	struct Pkt {uint8 pktLen, pktAddr, \
		cmdId, addr[4], len[2], sector, data;} *msg = (struct Pkt *)cmd ;
	uint32 wrAddress, dataOffset ;
	
    /* assure enough params */
	if(	msg->pktLen < 10 )
		return ;
		
	else
	{
		/* extract the command parameters */
		wrAddress = getWordBytes( msg->addr );
		
		/* copy a flash block to ram and overlay w/ new data */
		dataOffset = wrAddress % BOUNDRY_512 ;
		wrAddress -= dataOffset ;
		
		/* configure the wrparams for subsequent data packets */
		wrCtrl.addr   = wrAddress;
		wrCtrl.len    = getShortBytes( msg->len );
		wrCtrl.sector = msg->sector;
		wrCtrl.data   = &wrImage[ dataOffset ];
			
		if( (dataOffset + wrCtrl.len) <= FLASH_WR_BLOCK_SIZE
		&&	 wrAddress >= WR_ADDR_MIN
		&&	 wrAddress <= WR_ADDR_MAX  )
		{
			/* good params, create an image of this flash block */
			memcpy( wrImage, (uint8 *)wrAddress, FLASH_WR_BLOCK_SIZE );
			
			/* add this data to the image */
			wrImageData( &msg->data, (msg->pktLen - 9) );
		}
		else
			/* abort the command */
			wrCtrl.addr = NULL;
	}	
} 

static void writeFlash_data( uint8 *cmd )
{
	// Add data to flashImage
	
	/* use byte values so no padding by compiler */
	struct Pkt {uint8 pktLen, pktAddr, cmdId, data;} *msg = (struct Pkt *)cmd ;
	
	
	/* add this data to the image */
	wrImageData( &msg->data, (msg->pktLen - 2) );
}

static void wrImageData( uint8 *data, uint8 len )
{
	// add data to the image and count down the len in wrCtrl.
	// burn image to flash at end and ack to programmer if no errors.
	 
	uint8 response = WRITE_FLASH_RESPONSE;

	if( len > wrCtrl.len 
	||	wrCtrl.addr == NULL )
	{
		/* out of synch, abort the command */
		wrCtrl.addr = NULL ;
		return;
	}
	
	//ELSE..
	
	memcpy( wrCtrl.data, data, len );
	wrCtrl.data += len ;
	
	/* when all data is in, burn data block to flash */
	if( (wrCtrl.len -= len) == 0
	&&	IAP_prepareSectorsForWrite( wrCtrl.sector, wrCtrl.sector )
	&&	IAP_copyRamToFlash( wrCtrl.addr, (uint32)wrImage, FLASH_WR_BLOCK_SIZE )  )
	{
		/* let programmer change to receiver */
		wait_usec( 1500 );
		
		/* write is done, ack the programmer */
		sendRadioPacket( TOWER_ADDRESS, &response, 1 );
	}
}

//---------------
static void readFlash_cmd( uint8 *cmd )
{
	// pktlen, pktaddr, CMDid, addr(4byte), len(2byte)
	// addr =0-248k, len =1-128
	// Master sees data or No response if error
	
	struct Pkt {uint8 pktLen, pktAddr, cmdId, addr[4], len[2];}
		*msg = (struct Pkt *)cmd ;
	uint32 rdAddr; 
	uint16 rdLen; 
	uint8 sendLen, response[64];
	
	
	if(	msg->pktLen == 8 )
	{
		/* extract the command parameters */
		rdAddr = getWordBytes( msg->addr );
		rdLen  = getShortBytes( msg->len );
		response[0] = READ_FLASH_DATA ;
		
		/* let programmer change to receiver */
		wait_usec( 1500 );
		
		while( rdLen > 0 )
		{
			/* send data in multiple packets, 1-2 ms apart */
			wait_usec( 1200 );	
			
			sendLen = (rdLen > MAX_SEND_DATA)?  MAX_SEND_DATA : rdLen ;
			
			memcpy( &response[1], (uint8 *)rdAddr, sendLen );
			
			sendRadioPacket( TOWER_ADDRESS, response, (sendLen + 1) );
			
			rdAddr += sendLen;
			rdLen  -= sendLen;
		}
	}
}

//---------------
static void readStatus_cmd( uint8 *cmd )
{
	// pktlen, pktaddr, CMDid
	// get rssi and lqi values
	
	/* use byte values so no padding by compiler */
	struct Pkt {uint8 pktLen, pktAddr, cmdId, rssi, lqi;} *msg = (struct Pkt *)cmd ;
	uint8 response[3] = {QUERY_STATUS_RESPONSE, 0, 0} ;


	if(	msg->pktLen == 2 )
	{
		/* collect statii */
		response[1] = msg->rssi ;
		response[2] = msg->lqi ;
		
		/* let programmer change to receiver */
		wait_usec( 1500 );
		
		sendRadioPacket( TOWER_ADDRESS, response, 3 );
	}
}


//---------------
static uint32 getWordBytes( uint8 *data )
{
	Bw32 result;
	
	result.c[3] = *data++;
	result.c[2] = *data++;
	result.c[1] = *data++;
	result.c[0] = *data++;
	
	return result.w ;
}

static uint16 getShortBytes( uint8 *data )
{
	Bw16 result;
	
	result.c[1] = *data++;
	result.c[0] = *data++;
	
	return result.s ;
}


//============================
// 	 iap commands 
//============================
#define IAP_CMD_SUCCESS		0
#define IAP_CCLK_FREQ		FOSC_KHZ

/* IAP command parameters */
uint32 iapCommand[5], iapResult[2];	

static uint8 IAP_prepareSectorsForWrite( uint8 start, uint8 end )
{
	//return 1 =successful, else 0.  Use to verify sectors for write.
	
	if( start >= SECTOR_MIN
	&&	start <= SECTOR_MAX
	&&	end >= start
	&&	end <= SECTOR_MAX  )
	{
		iapCommand[0] = 50;
		iapCommand[1] = start;
		iapCommand[2] = end;
		
		runIapCommand( iapCommand, iapResult );
		return (iapResult[0] == IAP_CMD_SUCCESS)?  1 : 0 ;
	}
	
	else
		return 0;
}


static uint8 IAP_eraseSectors( uint8 start, uint8 end )
{
	//return 1 =successful, else 0
	
	iapCommand[0] = 52;
	iapCommand[1] = start;
	iapCommand[2] = end;
	iapCommand[3] = IAP_CCLK_FREQ;
	
	runIapCommand( iapCommand, iapResult );
	return (iapResult[0] == IAP_CMD_SUCCESS)?  1 : 0 ;
}

static uint8 IAP_copyRamToFlash( uint32 flashAddr, uint32 ramAddr, uint32 numBytes )
{
	//return 1 =successful, else 0
	
	iapCommand[0] = 51;
	iapCommand[1] = flashAddr;
	iapCommand[2] = ramAddr;
	iapCommand[3] = numBytes;
	iapCommand[4] = IAP_CCLK_FREQ;
	
	runIapCommand( iapCommand, iapResult );
	return (iapResult[0] == IAP_CMD_SUCCESS)?  1 : 0 ;
}


//============================
//    INTERRUPT SERVICE ROUTINES
//============================


//============================
//    HARDWARE SPECIFIC CODE
//============================

