/*
===============================================================================
 Name        : sec_bootloader.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : secondary bootloader runs after NXP's ISP bootrom so named sec
===============================================================================
*/

/**************************************************************************************************
 * INCLUDES
 **************************************************************************************************/

#include <bsp.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "iap_config.h"
#include "xmodem1k.h"
#include "uart.h"
#include "type.h"
#include "timer.h"
#include "iap.h"
#include "trace.h"
#include "wdt.h"
#include "gsm.h"
#include "math.h"
#include "exp_i2c.h"
#include "calibration.h"
#include "gps.h"
#include "trace.h"
#include "crc.h"

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>


/**************************************************************************************************
 * CONSTANTS
 **************************************************************************************************/


/**************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/

/**************************************************************************************************
 * FUNCTIONS - API
 **************************************************************************************************/

/*****************************************************************************
** Function name:	ExecuteApplicationImage
** Description:		Function executes the application image start address given.
** 					Application's sanity should have been verified before
** 					callin this function
**
** Parameters:		Start address of the image
** Returned value:	none
******************************************************************************/
void 	ExecuteApplicationImage( unsigned int startAddress );

/*****************************************************************************
** Function name:	IsUpgradeRequested
**
** Description:		Check if upgrade requested by application.
**                  When an upgrade request arrives application sets upgrade
**                  parameters to address TODO and reboots. When secondary
**                  boot loader takes control checks the parameters and
**                  starts upgrade procedure.
**
** Parameters:		none
**
** Returned value:	TRUE	upgrade is requested
** 					FALSE   upgrade is NOT requested
**
******************************************************************************/
uint32_t	IsUpgradeRequested( void );


/**************************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************************/
/*****************************************************************************
** Function name:	DownloadSecondaryImage
**
** Description:		The function erases flash sectors
** 					SECONDARY_IMAGE_START_SEC up to SECONDARY_IMAGE_END_SEC.
**					Starts downloading image using a modified version of
**					xmodem1k protocol.
**					look for description of XModem1K_Client function for
**					the details of modified xmodem1k protocol.
**
** Parameters:		none
** Returned value:	none
**
******************************************************************************/
static void		DownloadSecondaryImage( void );

/*****************************************************************************
** Function name:	IsSecondaryImageValid
**
** Description:		The function checks sanity of the secondary image using
** 		provided image size and CRC
**
** 		Memory  layout
**
** 		*********************** 0x0000 0000
**
** 		bootrom image
**
** 		**********************  0x0000 FC00
**
** 		Update request parameters. Last one kilo bytes of the bootrom image
** 		area is used to communicate upgrade parameters between bootrom and
** 		application images. For now only upgrade server ip and port in
** 		in ASCII string is used. Remaining part is reserved for future use.
**
** 		**********************  0x0001 0000
**
** 		Primary image
**
** 		**********************  0x0004 0000
**
** 		Secondary image
**
** 		The last 8 bytes of the secondary image is used to place image size
** 		and CRC.
** 		**********************  0x0006 FFF8
** 		4 bytes size of the image. This size is on fly size. May be greater
** 		than the actual file size of the image ondisk. Modified xmodem1k
** 		protocol appends 0xFFs to the last frame of the image if it is less
** 		than 1024 bytes. CRC is also calculated using this size instead of
** 		on disk size of the image.
** 		**********************  0x0006 FFFC
** 		CRC 16 bit crc is used and it is ored 0xFFFF0000. We have used 32 bits
** 		because there is a possibility of using 32bit CRC in the future
**
** 	    **********************  0x0007 0000
**
** 	    unused memory area
**
** 	    **********************  0x0008 0000
**
**
** Parameters:		none
** Returned value:	none
**
******************************************************************************/
static int32_t	IsSecondaryImageValid( void );

/*****************************************************************************
** Function name:	IsSecondaryImageValid
**
** Description:		The function checks sanity of the secondary image using
** 		provided image size and CRC
**
** 		Memory  layout
**
** 		*********************** 0x0000 0000
**
** 		bootrom image
**
** 		**********************  0x0000 FC00
**
** 		Update request parameters. Last one kilo bytes of the bootrom image
** 		area is used to communicate upgrade parameters between bootrom and
** 		application images. For now only upgrade server ip and port in
** 		in ASCII string is used. Remaining part is reserved for future use.
**
** 		**********************  0x0001 0000
**
** 		Primary image
**
** 		**********************  0x0004 0000
**
** 		Secondary image
**
** 		The last 8 bytes of the secondary image is used to place image size
** 		and CRC.
** 		**********************  0x0006 FFF8
** 		4 bytes size of the image. This size is on fly size. May be greater
** 		than the actual file size of the image ondisk. Modified xmodem1k
** 		protocol appends 0xFFs to the last frame of the image if it is less
** 		than 1024 bytes. CRC is also calculated using this size instead of
** 		on disk size of the image.
** 		**********************  0x0006 FFFC
** 		CRC 16 bit crc is used and it is ored 0xFFFF0000. We have used 32 bits
** 		because there is a possibility of using 32bit CRC in the future
**
** 	    **********************  0x0007 0000
**
** 	    unused memory area
**
** 	    **********************  0x0008 0000
**
**
** Parameters:		none
** Returned value:	none zero write success
**                  zero      write failure
**
******************************************************************************/
static uint32_t loadImage( uint8_t *data, uint16_t length );


/*****************************************************************************
** Function name:	ConfigurePins
**
** Description:		Configure pins according to bsp settings to be defaults

** Parameters:		none
**
** Returned value:	none
**
******************************************************************************/
static void ConfigurePins( void );

/**************************************************************************************************
 * LOCAL TYPES
 **************************************************************************************************/

/**************************************************************************************************
 * LOCAL VARIABLES
 **************************************************************************************************/
// keeps the image index during image download
static uint32_t received_data = 0;



int main(void)
{
	char buffer[200];

	SystemInit();

	LPC_SC->CLKSRCSEL |= 0x01;//0x01;
	LPC_SC->PLL0CFG	  |= 0x01; // Select external osc. as main clock.
	LPC_SC->CCLKCFG	   = 0x03; // Main PLL is divided by 8

	SystemCoreClockUpdate();
	// Generate f each 1 ms, used to enable DelayMs function
	SysTick_Config(SystemCoreClock / 1000 - 1);

	WDTInit(WDT_FEED_30_SECS);

	ConfigurePins();

	UARTInit( PORT_TRACE, 115200 );
	UARTInit( PORT_GSM,   115200 );

	TraceNL("\r\nBooting up");
	sprintf(buffer, "SystemCoreClock = %d Hz\r\n", SystemCoreClock);
	Trace(buffer);

//	if( EEPROM_Init() == 0 )
//	{
//		TraceNL("EEPROM Initialization failed.");
//	} else {
//		TraceNL("EEPROM Initialized.");
//	}

	WDTFeed();
//	LoadParams();

	TraceNL( "Checking upgrade request" );

	if( IsUpgradeRequested() )
	{
		TraceNL( "System image upgrade requested" );
		int8_t 	trials = 3;
		while( trials-- > 0 )
		{
			/*
			 * 	Initialize GSM module
			 * 	Setup a server connection to update server
			 */
			if( GSM_ConnectToTrioUpgradeServer() == SUCCESS )
			{
				TraceNL( "Server Connection Established to Upgrade server" );
				WDTFeed( );
				DownloadSecondaryImage();
				WDTFeed( );
				TraceNL( "Download finished " );
				break;
			}
		}
		TraceNL( "Finished upgrading" );
	}

	if( IsSecondaryImageValid() == SUCCESS )
	{
		TraceNL( "Booting SECONDARY image" );
		ExecuteApplicationImage( SECONDARY_IMAGE_LOAD_ADDR );
	}

	TraceNL( "Booting PRIMARY image" );
	WDTFeed( );
	ExecuteApplicationImage( PRIMARY_IMAGE_LOAD_ADDR );

	while ( 1L );
}

/*****************************************************************************
** Function name:	IsUpgradeRequested
**
** Description:		Check if upgrade requested by application.
**                  When an upgrade request arrives application sets upgrade
**                  parameters to address UPGRADE_PARAMETERS_ADDR and reboots.
**                   When secondary
**                  boot loader takes control checks the parameters and
**                  starts upgrade procedure.
**
** Parameters:		none
**
** Returned value:	TRUE	upgrade is requested
** 					FALSE   upgrade is not requested
**
******************************************************************************/
uint32_t	IsUpgradeRequested( void )
{
	if( (*( (uint32_t *)UPGRADE_PARAMETERS_ADDR) ) != 0xFFFFFFFF )
	{
		char buffer[100];
		char * port ;
		strcpy( update_service_ip, (char*)UPGRADE_PARAMETERS_ADDR );

		port = strchr( (char*)UPGRADE_PARAMETERS_ADDR, '\0' );
		port++;
		strcpy( update_service_port, port);

		sprintf(buffer,"Update parameters %s:%s", update_service_ip, update_service_port);
		TraceNL( buffer );

		u32IAP_PrepareSectors( UPGRADE_PARAMETERS_SEC, UPGRADE_PARAMETERS_SEC );
		u32IAP_EraseSectors( UPGRADE_PARAMETERS_SEC, UPGRADE_PARAMETERS_SEC );
		return TRUE;
	}

	return FALSE;
}


void DownloadSecondaryImage( void )
{
	uint32_t	reason[5];
	uint32_t	i;

	char buff[100];

	i = SECONDARY_IMAGE_START_SEC;
	for( ; i <= SECONDARY_IMAGE_END_SEC; i++ )
	{
		u32IAP_PrepareSectors( i, i );
		u32IAP_EraseSectors( i, i );
	}
	TraceNL( "Checking if target memory is blank" );


	i = SECONDARY_IMAGE_START_SEC;
	for( ; i <= SECONDARY_IMAGE_END_SEC; i++ )
	{
		if( u32IAP_BlankCheckSectors( i, i, &reason[0] )== IAP_STA_SECTOR_NOT_BLANK )
		{
			sprintf(buff, "Target sector (%d) is not blank addr: 0x%X, 0x%X", i, reason[0], reason[1] );
			TraceNL( buff );
			u32IAP_PrepareSectors( i, i );
			u32IAP_EraseSectors( i, i);

		}else
		{
			sprintf(buff, "Target sector (%d) is blank ", i );
			TraceNL( buff );
		}
	}

	/*	Clear the received data counter using in the load_mage function */
	received_data = 0;

	TraceNL( "Starting download" );
	/*	Store a new image into flash */
	XModem1K_Client( &loadImage );


}

void ExecuteApplicationImage( unsigned int startAddress )
{
	void (*user_code_entry)(void);

	unsigned *p;	// used for loading address of reset handler from user flash

	/* Change the Vector Table to the
	in case the user application uses interrupts */

	SCB->VTOR = (startAddress & 0x1FFFFF80);

	// Load contents of second word of user flash - the reset handler address
	// in the applications vector table
	p = (unsigned *)(startAddress + 4);

	// Set user_code_entry to be the address contained in that second word
	// of user flash
	user_code_entry = (void *) *p;

	// Jump to user application
    user_code_entry();
}

static uint32_t loadImage( uint8_t *data, uint16_t length )
{
	char buffer[250];
	uint32_t rc;

	sprintf(buffer, "Totally received : %d   frame length : %d\r\n", received_data, length);
	Trace( buffer );

	if( data != NULL && length )
	{
		/*	Prepare Sectors to be flashed */
		if (u32IAP_PrepareSectors( SECONDARY_IMAGE_START_SEC, SECONDARY_IMAGE_END_SEC ) == IAP_STA_CMD_SUCCESS)
		{
			TraceNL("prepared ");

			rc = u32IAP_CopyRAMToFlash(
					SECONDARY_IMAGE_LOAD_ADDR + received_data,
					(uint32_t) data,
					length
					);

			sprintf( buffer, "Copy Ram result code : %d\r\n", rc );
			TraceNL( buffer );
			/*	Copy data (already) located in RAM to flash */
			if (rc == IAP_STA_CMD_SUCCESS)
			{
				TraceNL( "copied " );
				rc = u32IAP_Compare( SECONDARY_IMAGE_LOAD_ADDR + received_data,
						             (uint32_t) data,
									 length, 0
									);
				sprintf( buffer, "u32IAP_Compare : %d  wrote %d \r\n", rc , length );
				Trace( buffer );

				/*	Verify the flash contents with the contents in RAM */
				if (rc == IAP_STA_CMD_SUCCESS)
				{
					/*	Update and Print Received bytes counter */
					received_data += length;
					TraceNL( "verified " );
					return (1);
				} else {
					TraceDumpHex( (char*)(SECONDARY_IMAGE_LOAD_ADDR + received_data), length );
					TraceNL( "verification failed " );
				}
			}
		}
	}

	return (0);
}


static int32_t IsSecondaryImageValid( void )
{
	char buffer[100];

	uint32_t size = *(uint32_t *)(SECONDARY_IMAGE_END_ADDR - 8);

	uint16_t crc  = *(uint16_t *)(SECONDARY_IMAGE_END_ADDR - 4);

	sprintf(buffer, "Signature: 0x%X   CRC : %X\r\n", size, crc);
	Trace( buffer );

	uint16_t calculatedCRC = u16CRC_Calc16( (uint8_t*)SECONDARY_IMAGE_LOAD_ADDR, size );

	sprintf( buffer, "Calculated Image CRC: 0x%X\r\n", calculatedCRC );
	Trace( buffer );

	if( crc == calculatedCRC )
		return ( 0 );  // image is valid

	return ( 1 );  //image is not valid
}


void ConfigurePins( void )
{
	//PWRKEY & EMERG_OFF
	LPC_PINCON->PINSEL4 &= ~(0xFFFF); // Reset P2[0..7] = GPIO
	LPC_GPIO2->FIODIR |= 0xFF; // P2[0..7] =
	LPC_PINCON->PINSEL9 &= ~(0xFFFF); // Reset P4[24..31] = GPIO
	LPC_GPIO4->FIODIR = (1 << 28) | (1 << 29);
	LPC_GPIO0->FIODIR &= (1 << 7); //P0[7] DIN 1 as input --> Default mode pull up enabled.
	LPC_GPIO0->FIODIR &= ~(1 << 29 | 1 << 30); //P0[30] as input
	LPC_GPIO0->FIODIR |= (1 << 5);
	//Init ADC
	uint32_t u32PCLKDIV, u32PCLK;
	LPC_SC->PCONP |= (1 << 12);
	u32PCLKDIV = (LPC_SC->PCLKSEL0 >> 6) & 0x03;
	switch (u32PCLKDIV) {
	case 0x00:
	default:
		u32PCLK = 12000000 / 4;
		break;
	case 0x01:
		u32PCLK = 12000000;
		break;
	case 0x02:
		u32PCLK = 12000000 / 2;
		break;
	case 0x03:
		u32PCLK = 12000000 / 8;
		break;
	}

	///ADC//
	//LPC_ADC->ADCR = (1 << 3) | ((u32PCLK / 12000000 - 1) << 8) | (0 << 16) | (0
	//		<< 17) | (1 << 21) | (0 << 24) | (0 << 27);
	//LPC_PINCON->PINSEL1 |= (1U << 20);
	///ADC//
	//External Power Pin
	LPC_GPIO0->FIODIR &= ~(1 << 22);
	//Ignition Pin
	LPC_GPIO0->FIODIR &= ~(1 << 21);
	#ifdef brisa
		DIGITAL_IN1_PIN = 21;
	#endif
	LPC_GPIOINT->IO0IntEnR |= (1 << 22); // Rising edge
	LPC_GPIOINT->IO0IntEnF |= (1 << 22); // Falling edge
	NVIC_EnableIRQ(EINT3_IRQn);
}
