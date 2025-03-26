/**
 * @file   MedRadioBoot2.c
 * @author JDCC/JML
 * @brief Handles boot commands
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys.h"
#include "boot2.h"
#include "MRcc1101radio.h"


// -------- DEFINITIONS ----------
typedef void (*IAP)(unsigned int [],unsigned int[]);

#define IAP_LOCATION 	                0x7ffffff1

#define MIN_PACKET_LEN                  5 //pktlen(1byte)+pktaddr(1byte)+cmd/data(1+bytes)+status(2bytes) 

// --------   DATA   ------------
IAP runIapCommand = (IAP) IAP_LOCATION;
uint8 statusBL = 0;
uint16 serialNumber = 0;
uint8 appLocalAddress = 0;
uint8 appRemoteAddress = 0;
uint8 appChannelNumber = 0;
uint8 appTXPower = 0;
uint8 commandByte = 0;





// -------- PROTOTYPES ----------
static uint8 executeBootCommand( uint8 *commandBuffer, uint8 len );

static void eraseFlash_cmd( uint8 *cmd );
static void writeFlash_cmd( uint8 *cmd );
static void writeFlash_data( uint8 *cmd );
static void readFlash_cmd( uint8 *cmd );
static void readStatus_cmd( uint8 *cmd );
static void readRFData_cmd ( void );
static void returnToWait_cmd ( void );
static void blankCheck_cmd ( uint8 *cmd );
static void writeCommandByte_cmd ( uint8 * cmd );

static uint8 IAP_prepareSectorsForWrite( uint8 start, uint8 end );
static uint8 IAP_eraseSectors( uint8 start, uint8 end );
static uint8 IAP_copyRamToFlash( uint32 flashAddr, uint32 ramAddr, uint32 numBytes );
static uint8 wrImageData( uint8 *data, uint8 len );

static uint32 getWordBytes( uint8 *data );
static uint16 getShortBytes( uint8 *data );



//============================
//    GLOBAL CODE
//============================
#define MAX_COMMAND_BUFFER	80

uint8 runBootCommandTask( void )
{
	// run commands and return 1=active, else 0
	
	uint8 commandBuffer[ MAX_COMMAND_BUFFER ], len;
	
	
	if( (len = getRadioPacket( commandBuffer)) > 0 ) 
	{
                //CRC autoflush is enabled to prevent bad packets
                if( len >= MIN_PACKET_LEN )
                  executeBootCommand( commandBuffer, len );
		               
		enableRadioReceiver();
	}
        
	
        // status is set by query status cmd: 1 = active bootloader, 0 = wait, then load app
	return statusBL ;
}


//============================
//    LOCAL CODE
//============================
enum
{
	WRITE_FLASH_CMD			= 0x10,
        WRITE_FLASH_CMD_RESPONSE        = 0x20,
        
	WRITE_FLASH_DATA		= 0x11,
        WRITE_FLASH_DATA_RESPONSE       = 0x21,
	
	UNRECOGNIZED_CMD		= 0x2A,
	
	ERASE_FLASH_CMD			= 0x12,
	ERASE_FLASH_RESPONSE	        = 0x22,
	
	READ_FLASH_CMD			= 0x13,
	READ_FLASH_DATA			= 0x23,
	
	QUERY_STATUS_CMD		= 0x14,
	QUERY_STATUS_RESPONSE	        = 0x24,
          
        RETURN_TO_WAIT_CMD	        = 0x15,
	RETURN_TO_WAIT_RESPONSE	        = 0x25,
                 
        READ_RF_DATA_CMD                = 0x17,
        READ_RF_DATA_RESPONSE           = 0x27,
        
        BLANK_CHECK_CMD                 = 0x18,
        BLANK_CHECK_RESPONSE            = 0x28,
        
        WRITE_COMMANDBYTE_CMD           = 0x19,
        WRITE_COMMANDBYTE_RESPONSE      = 0x29  
          
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
        {
		 readStatus_cmd( commandBuffer );
                 statusBL = 1;
        }
        else if( *cmdId == RETURN_TO_WAIT_CMD )
        {
		 returnToWait_cmd();
                 statusBL = 0;
        }
        else if( *cmdId == READ_RF_DATA_CMD )
        {
		 readRFData_cmd();                
        }
        else if (*cmdId == BLANK_CHECK_CMD )
        {
                blankCheck_cmd( commandBuffer );             
        }
        else if (*cmdId == WRITE_COMMANDBYTE_CMD  )
        {
                writeCommandByte_cmd( commandBuffer );             
        }
	else
	{
		response = UNRECOGNIZED_CMD;
		sendRadioPacket( BOOTCODE_ADDRESS, &response, 1 );
		return 0;
	}

	return 1;
}


//============================
// 	 BOOT2 COMMANDS
//============================
#define SECTOR_MIN				1		//our boot code resides in lower sectors (0)
#define SECTOR_MAX				16		// ISP/IAP factory boot code in top sector (17)

#define FLASH_WR_BLOCK_SIZE		        512		// options are: 512/1k/4k/8k
#define BOUNDRY_512				512		// flash write address boundry mask
#define WR_ADDR_MIN				0x02000
#define WR_ADDR_MAX				0x3E000          //write can only be up to WR_ADDR_MAX - FLASH_WR_BLOCK_SIZE

#define MAX_SEND_DATA			59      //59 + MIN_PACKET_LEN = 64

#pragma data_alignment=4
static uint8 wrImage[ FLASH_WR_BLOCK_SIZE ];

uint32 iapCommand[5], iapResult[2];       //Used for Prep, Erase, and Write IAP commands    
uint32 iapBCCommand[3], iapBCResult[3];	  //Used for Blank Check IAP commands

/* stores image to be burned to flash*/
struct WrCtrl
{
	uint32 addr;
	uint8  *data;
	uint16 len;
	uint8  sector;

} static wrCtrl;


/**
 * @brief Erases requested sector(s) and sends radio response (ERASE_FLASH_RESPONSE):
 *        1 = sector erased successfully
 *        0 = parameters bad, or erase failed
 * @param *cmd = pktlen, pktaddr, CMDid, start, end (all 1byte)
 *              1 <= start <= end <=16
 */
static void eraseFlash_cmd( uint8 *cmd )
{

	/* use byte values so no padding by compiler */
	struct Pkt {uint8 pktLen, pktAddr, cmdId, start, end;} *msg = (struct Pkt *)cmd ;
	uint8 bootResponse[2] = {ERASE_FLASH_RESPONSE, 0} ;


	if(	msg->pktLen == 4
	&&	IAP_prepareSectorsForWrite( msg->start, msg->end )
	&&	IAP_eraseSectors( msg->start, msg->end )  )
	{
		/* erase is done, ack the programmer */
		bootResponse[1] = 1;
	}
	
        /* delay while programmer enables receiver, then send packet */
	wait_usec( 1500 );	
	sendRadioPacket( BOOTCODE_ADDRESS, bootResponse, 2 );
}



/**
 * @brief Blank checks requested sector(s), and sends radio response with blank
 *        check results (BLANK_CHECK_RESPONSE) :
 *              3 bytes with response
 * @param *cmd = pktlen, pktaddr, CMDid, start, end (all 1byte)
 *              1 <= start <= end <=16
 */
static void blankCheck_cmd( uint8 *cmd )
{        
	/* use byte values so no padding by compiler */
	struct Pkt {uint8 pktLen, pktAddr, cmdId, start, end;} *msg = (struct Pkt *)cmd ;
	uint8 bootResponse[4] = {BLANK_CHECK_RESPONSE, 0, 0, 0} ;

        iapBCCommand[0] = 53;
	iapBCCommand[1] = msg->start;
	iapBCCommand[2] = msg->end;
	
        if( msg->start >= SECTOR_MIN
	&&	msg->start <= SECTOR_MAX
	&&	msg->end >= msg->start
	&&	msg->end <= SECTOR_MAX  )
        {
          runIapCommand( iapBCCommand, iapBCResult );
          bootResponse[1] = iapBCResult[0];
          bootResponse[2] = iapBCResult[1];
          bootResponse[3] = iapBCResult[2];
        }
        else
        {
          bootResponse[1]  = 0xFF;
        }
		
        /* delay while programmer enables receiver, then send packet */
	wait_usec( 1500 );
	sendRadioPacket( BOOTCODE_ADDRESS, bootResponse, 4 );
}



/**
 * @brief Sets up and begins write to Flash by initializing global structure, wrCtrl, and
 *        sends radio response (WRITE_FLASH_CMD_RESPONSE): 
 *              1 = data was written to image successfully
 *              0 = parameters bad or write failed
 * @param *cmd = pktlen, pktaddr, CMDid,addr(4byte),len(2bytes),sectorNumber(1byte),data(1-52bytes)...
 *               addr         = 0x00002000-0x0003DFFF
 *               len          = 1-512
 *               sectorNumber = 1-16
 */
static void writeFlash_cmd( uint8 *cmd )
{
	/* use byte values so no padding by compiler */
        uint8 bootResponse[2] = {WRITE_FLASH_CMD_RESPONSE, 0};
	struct Pkt {uint8 pktLen, pktAddr, \
		cmdId, addr[4], len[2], sector, data;} *msg = (struct Pkt *)cmd ;
	uint32 wrAddress, dataOffset ;
	
        /* ensure enough params */
	if(	msg->pktLen >= 10 )
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
		&&	 wrAddress <= WR_ADDR_MAX - FLASH_WR_BLOCK_SIZE  )
		{
			/* good params, create an image of this flash block */
			memcpy( wrImage, (uint8 *)wrAddress, FLASH_WR_BLOCK_SIZE );
			
			/* add this data to the image */
			if(  wrImageData( &msg->data, (msg->pktLen - 9) )  )
                        {
                            /* write is done, ack the programmer */
                            bootResponse[1] = 1; 
                        }  
		}
		else
                {
			/* abort the command */
			wrCtrl.addr = NULL;
                }

	}
        
        /* delay while programmer enables receiver, then send packet */ 
        wait_usec( 1500 );  
        sendRadioPacket( BOOTCODE_ADDRESS, bootResponse, 2 );	
} 

/**
 * @brief Continues write to Flash.  Global structure, wrCtrl, should be initialized 
 *        prior to calling this function.  Sends radio response (WRITE_FLASH_DATA_RESPONSE):
 *              1 = data was written to image successfully
 *              0 = parameters bad or write failed
 * @param *cmd = pktlen, pktaddr, CMDid,data(1-59bytes)...
 */
static void writeFlash_data( uint8 *cmd )
{
        uint8 bootResponse[2] = {WRITE_FLASH_DATA_RESPONSE, 0};
        
	/* use byte values so no padding by compiler */
	struct Pkt {uint8 pktLen, pktAddr, cmdId, data;} *msg = (struct Pkt *)cmd ;
	
	/* ensure enough params */
	if(	msg->pktLen >= 3 )
	{
          /* add this data to the image */
          if(  wrImageData( &msg->data, (msg->pktLen - 2) )  )
          {
            /* write is done, ack the programmer */
            bootResponse[1] = 1;
          }
        }
        
        /* delay while programmer enables receiver, then send packet */
        wait_usec( 1500 );
        sendRadioPacket( BOOTCODE_ADDRESS, bootResponse, 2 );
}

/**
 * @brief Helper function to call low level IAP functions to write image data to flash
 *        Appends data to the image and counts down the len in wrCtrl.  Burns image to 
 *        flash when image is complete. This function should not send radio packets.
 *
 * @param *data  data to write to Flash (up to 255bytes)
 * @param len    data length (1-255)
 *
 * @return 0 if unsuccessful, 1 if successful
 */
static uint8 wrImageData( uint8 *data, uint8 len )
{

	if( len > wrCtrl.len 
	||	wrCtrl.addr == NULL )
	{
		/* out of sync, abort the command */
		wrCtrl.addr = NULL ;
		return 0;
	}
	
	//ELSE..
	
	memcpy( wrCtrl.data, data, len );
	wrCtrl.data += len ;
	
	/* when all data is in, burn data block to flash */
	if( (wrCtrl.len -= len) == 0 )
	{           
            /* if copying to Flash fails, report to caller  */
            if( !IAP_prepareSectorsForWrite( wrCtrl.sector, wrCtrl.sector )
             || !IAP_copyRamToFlash( wrCtrl.addr, (uint32)wrImage, FLASH_WR_BLOCK_SIZE )  )
            {      
                return 0;
            }
	}
        
        return 1;
}

/**
 * @brief Reads from Flash and sends multiple responses (READ_FLASH_DATA):
 *       no data included if data could not be read
 * @param *cmd = pktlen, pktaddr, CMDid, addr(4byte), len(2byte)
 *               addr         = 0x00002000-0x0003DFFF
 *               len          = 1-512
 */
static void readFlash_cmd( uint8 *cmd )
{
	struct Pkt {uint8 pktLen, pktAddr, cmdId, addr[4], len[2];}
		*msg = (struct Pkt *)cmd ;
	uint32 rdAddr; 
	uint16 rdLen; 
	uint8 sendLen, response[64];
        uint8 responseErr[1] = { READ_FLASH_DATA };
        
	
        /* ensure correct number of parameters */
	if(	msg->pktLen == 8 )
	{
		/* extract the command parameters */
		rdAddr = getWordBytes( msg->addr );
		rdLen  = getShortBytes( msg->len );
		response[0] = READ_FLASH_DATA ;
		
                if( rdLen > 0 )
                {
                  while( rdLen > 0 )
                  {
                          sendLen = (rdLen > MAX_SEND_DATA)?  MAX_SEND_DATA : rdLen ;
                          
                          memcpy( &response[1], (uint8 *)rdAddr, sendLen );
                          
                          /* delay while programmer enables receiver, then send packet */
                          wait_usec( 1500 );
                          sendRadioPacket( BOOTCODE_ADDRESS, response, (sendLen + 1) );
                          
                          rdAddr += sendLen;
                          rdLen  -= sendLen;
                  }
                }
                else 
                {
                  /* delay while programmer enables receiver, then send packet */
                  wait_usec( 1500 );
                  sendRadioPacket( BOOTCODE_ADDRESS, responseErr, 1 );
                }
	}
        else 
        {
          /* delay while programmer enables receiver, then send packet */
          wait_usec( 1500 );
          sendRadioPacket( BOOTCODE_ADDRESS, responseErr, 1 );
        }
}

/**
 * @brief Reads and returns device information: Bootloader Revision, Radio RSSI/LQI, 
 *        and device Serial Number and sends radio response (QUERY_STATUS_RESPONSE)
 *       no data included if data could not be read          
 * @param *cmd = pktlen, pktaddr, CMDid
 */
static void readStatus_cmd( uint8 *cmd )
{
	/* use byte values so no padding by compiler */
	struct Pkt {uint8 pktLen, pktAddr, cmdId, rssi, lqi;} *msg = (struct Pkt *)cmd ;
	uint8 response[8] = {QUERY_STATUS_RESPONSE, SVN_REVISION, PRODUCT_REVISION, 0, 0, 0, 0, 0 } ;
        uint8 responseErr[1] = { QUERY_STATUS_RESPONSE };
        /* ensure correct number of parameters */
	if(	msg->pktLen == 2 )
	{       
                NonVolatileRead(); /* collect status */
		
		response[3] = msg->rssi ;
		response[4] = msg->lqi ;
		response[5] = (uint8)( serialNumber );
                response[6] = (uint8)( serialNumber >> 8 );
                response[7] = commandByte;
                
		/* delay while programmer enables receiver, then send packet */
		wait_usec( 1500 );
		sendRadioPacket( BOOTCODE_ADDRESS, response, 8 );
	}
        else
        {
          	/* delay while programmer enables receiver, then send packet */
		wait_usec( 1500 );
		sendRadioPacket( BOOTCODE_ADDRESS, responseErr, 1 );
        }
}



/**
 * @brief Writes to the command byte in APP_SETTINGS
 *
 * @param *cmd = pktlen, pktaddr, CMDid, &commandByte)
 */
static void writeCommandByte_cmd ( uint8 *cmd )
{
    struct Pkt {uint8 pktLen, pktAddr, cmdId, rssi, lqi;} *msg = (struct Pkt *)cmd ;
    uint8 response[2] = { WRITE_COMMANDBYTE_RESPONSE, 0 };
    uint8 data[1];
    uint32 wrAddress, dataOffset ;
        
    /* ensure correct number of parameters */
    if(	msg->pktLen == 3 )
    {         
         
      wrAddress = APP_SETTINGS_LOCATION + 4;
      dataOffset = 4; // offset for commandByte
      wrAddress -= dataOffset ;
           
       /* configure the wrparams */
      wrCtrl.addr   = wrAddress;
      wrCtrl.len    = 1;
      wrCtrl.sector = 16;
      wrCtrl.data   = &wrImage[ dataOffset ];
      
      /*create an image of this flash block */
      memcpy( wrImage, (uint8 *)wrAddress, FLASH_WR_BLOCK_SIZE );                       
      data[0] = cmd[3];

      IAP_prepareSectorsForWrite( 16, 16);
      IAP_eraseSectors( 16, 16);
      
      if ( wrImageData( data, 1 ) )
      {               
        /* successful Image write*/
      }
            
    }
    NonVolatileRead();
    response[1] = commandByte;
        
    /* delay while programmer enables receiver*/
    wait_usec( 1500 );
    sendRadioPacket( BOOTCODE_ADDRESS,response, 2 );
  
}

/**
 * @brief Sends radio response that return to wait command was received 
 */
static void returnToWait_cmd (void)
{
  uint8 response[1] = {RETURN_TO_WAIT_RESPONSE};
  
  /* delay while programmer enables receiver */
  wait_usec( 1500 );		
  sendRadioPacket( BOOTCODE_ADDRESS, response, 1 );
  
}

/**
 * @brief Read RF information used by Application and send radio response
 *        Application RF info: appLocalAddress, appRemoteAddress, appChannelNumber, appTXPower (all 1byte)
 */
static void readRFData_cmd ( void )
{
  NonVolatileRead();
  uint8 response[5] = {READ_RF_DATA_RESPONSE, appLocalAddress, appRemoteAddress, appChannelNumber, appTXPower};
  
  /* delay while programmer enables receiver */
  wait_usec( 1500 );	
  sendRadioPacket( BOOTCODE_ADDRESS, response, 5 );
  
} 

/**
 * @brief Read settings from non-volatile memory (flash), called by several _cmd routines
 *      updates global variables serialNumber, appLocalAddress, appRemoteAddress, appChannelNumber, appTXPower
 */
void NonVolatileRead( void )
{
  serialNumber = 0;
  appLocalAddress = 0;
  appRemoteAddress = 0;
  appChannelNumber = 0;
  appTXPower = 0;
  commandByte = 0;
  
  serialNumber = *((uint8 *)(SERIAL_NUMBER_LOCATION)) << 8; // high byte
  serialNumber += *(uint8 *)(SERIAL_NUMBER_LOCATION + 1);   // low byte
  
  //RF Data
  appLocalAddress   = *(uint8 *)(APP_SETTINGS_LOCATION);   
  appRemoteAddress  = *(uint8 *)(APP_SETTINGS_LOCATION + 1); 
  appChannelNumber  = *(uint8 *)(APP_SETTINGS_LOCATION + 2); 
  appTXPower        = *(uint8 *)(APP_SETTINGS_LOCATION + 3);
  commandByte       = *(uint8 *)(APP_SETTINGS_LOCATION + 4);
 
}
/**
 * @brief Helper function to get a 32bit unsigned integer from 4 bytes
 */
static uint32 getWordBytes( uint8 *data )
{
	Bw32 result;
	
	result.c[3] = *data++;
	result.c[2] = *data++;
	result.c[1] = *data++;
	result.c[0] = *data++;
	
	return result.w ;
}

/**
 * @brief Helper function to get a 16bit unsigned integer from 2 bytes
 */
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


/**
 * @brief Use to verify sectors for write.
 *
 * @param start: starting sector
 * @param end:  ending sector
 *
 * @return 1 =successful, else 0.  
 */
static uint8 IAP_prepareSectorsForWrite( uint8 start, uint8 end )
{
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

/**
 * @brief Use to erase sectors.  
 *
 * @param start: starting sector
 * @param end:  ending sector
 *
 * @return 1 =successful, else 0.  
 */
static uint8 IAP_eraseSectors( uint8 start, uint8 end )
{
  	if( start >= SECTOR_MIN
	&&	start <= SECTOR_MAX
	&&	end >= start
	&&	end <= SECTOR_MAX  )
	{
                iapCommand[0] = 52;
                iapCommand[1] = start;
                iapCommand[2] = end;
                iapCommand[3] = IAP_CCLK_FREQ;
                
                runIapCommand( iapCommand, iapResult );
                return (iapResult[0] == IAP_CMD_SUCCESS)?  1 : 0 ;
        }
        else
		return 0;
}

/**
 * @brief Use to copy data to Flash
 *
 * @param flashAddr (uint32) 
 * @param ramAddr (uint32)
 * @param numBytes (uint32)
 *
 * @return 1 =successful, else 0.  
 */
static uint8 IAP_copyRamToFlash( uint32 flashAddr, uint32 ramAddr, uint32 numBytes )
{
    	if( flashAddr >= WR_ADDR_MIN
	&&  flashAddr <= WR_ADDR_MAX - numBytes)
	{
              iapCommand[0] = 51;
              iapCommand[1] = flashAddr;
              iapCommand[2] = ramAddr;
              iapCommand[3] = numBytes;
              iapCommand[4] = IAP_CCLK_FREQ;
              
              runIapCommand( iapCommand, iapResult );
              return (iapResult[0] == IAP_CMD_SUCCESS)?  1 : 0 ;
        }
        else
		return 0;
}


//============================
//    INTERRUPT SERVICE ROUTINES
//============================
/* none: all interrupts disabled */

//============================
//    HARDWARE SPECIFIC CODE
//============================

