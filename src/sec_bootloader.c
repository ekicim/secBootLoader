/*
===============================================================================
 Name        : sec_bootloader.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : secondary bootloader runs after NXP's ISP bootrom so named sec
===============================================================================
*/

#include <bsp.h>
#include <stdint.h>
#include <stdio.h>
#include "iap_config.h"
#include "xmodem1k.h"
#include "uart.h"
#include "type.h"
#include "iap.h"
#include "trace.h"

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>

/**************************************************************************************************
 * INCLUDES
 **************************************************************************************************/


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
** Function name:	CheckApplicationImageValidity
** Description:		Function checks sanity of application images and chooses
** 					one depending validity and version number which ever
** 					has higher version is choosen.
**
** Parameters:		pImageAddress function loads start address of the image
** Returned value:	TRUE	a valid image found
**                  FALSE 	there is no valid image
******************************************************************************/
int 	CheckApplicationImageValidity( uint32_t* pImageAddr );


/*****************************************************************************
** Function name:	ExecuteApplicationImage
** Description:		Function executes the application image start address given.
** 					Application's sanity should have been verified before
** 					callin this function
**
** Parameters:		Start address of the image
** Returned value:	none
******************************************************************************/
void 	ExecuteApplicationImage( unsigned int start_address );






/*	Function Prototype */
static uint32_t load_image(uint8_t *data, uint16_t length);


/*	Character array workspace for GLCD print functions */
#define MAX_STRING_SIZE		50
static uint8_t string[MAX_STRING_SIZE];
static uint32_t received_data = 0;

/*	State-machine variable to control application functionality */
enum state_machine {
	READY = 0,
	MENU,
	ERASE_FLASH,
	FLASH_IMG,
	SHOW
};
enum state_machine	 cmd;

void enter_serial_isp( void );
int CheckApplicationImageValidity( uint32_t* pImageAddr );



int main(void) {


	uint32_t imageAddr;

	SystemInit();
	LPC_SC->CLKSRCSEL |= 0x01;//0x01;
	LPC_SC->PLL0CFG   |= 0x01; // Select external osc. as main clock.
	LPC_SC->CCLKCFG    = 0x03; // Main PLL is divided by 8
	SystemCoreClockUpdate();

	UARTInit(PORT_TRACE, 115200);

	TraceEndl ("Bootloader is starting");
	TraceEndl ("Checking application image's validity");

	// Check to see if there is a user application in the LPC1768's flash memory.
	if( CheckApplicationImageValidity( &imageAddr ) )
	{
		ExecuteApplicationImage( imageAddr );
	}

	TraceEndl ("Unable to locate any valid image to run");

	// Valid application does not exists. Get one from UART 0
	enter_serial_isp();

	while ( 1 );	// assert should not get here
	return (0);
}

/*****************************************************************************
** Function name:	CheckApplicationImageValidity
**
** Description:		Check application images for validity.
**
** Parameters:		none
**
** Returned value:	TRUE if application is valid
** 					FALSE   not valid
**
******************************************************************************/
int CheckApplicationImageValidity( uint32_t* pImageAddr )
{
	uint32_t	result, reason;
	uint32_t 	priImageVer = 0;
	uint32_t 	secImageVer = 0;
	unsigned char isPriBlank  = FALSE;
	unsigned char isSecBlank  = FALSE;
	unsigned char isPriValid  = FALSE;
	unsigned char isSecValid  = FALSE;

	char buffer[200];
	int count;


	result = u32IAP_BlankCheckSectors( PRIMARY_IMAGE_START_SEC,
								       PRIMARY_IMAGE_START_SEC, &reason );
	if( result != IAP_STA_SECTOR_NOT_BLANK )
	{
		isPriBlank = TRUE;
	}

	result = u32IAP_BlankCheckSectors( SECONDARY_IMAGE_START_SEC,
			                           SECONDARY_IMAGE_START_SEC, &reason );
	if( result != IAP_STA_SECTOR_NOT_BLANK )
	{
		isSecBlank = TRUE;
	}

	// check versions
	priImageVer	= *( (uint32_t *) (PRIMARY_IMAGE_LOAD_ADDR   + IMAGE_VERSION_OFFSET) );
	secImageVer	= *( (uint32_t *) (SECONDARY_IMAGE_LOAD_ADDR + IMAGE_VERSION_OFFSET) );


	// size of the image is placed in an image at offset IMAGE_SIZE_OFFSET and when an
	// image is flashed to validate the image the image size is written to

	if( !isPriBlank || (priImageVer != 0xFFFFFFFF) )
	{
		/* validation for the primary is only check against 0xFFFFFFFF */
		if( (*( (uint32_t *) (PRIMARY_IMAGE_LOAD_ADDR + IMAGE_SIZE_OFFSET))) != 0xFFFFFFFF )
			// (*( (uint32_t *) (PRIMARY_IMAGE_LOAD_ADDR + IMAGE_SIZE_CHECK_OFFSET))) )
				isPriValid = TRUE;

		sprintf(buffer,"Primary Image sizes (0x%X == 0x%X)\r",
				*( (uint32_t *) (PRIMARY_IMAGE_LOAD_ADDR + IMAGE_SIZE_OFFSET)),
				*( (uint32_t *) (PRIMARY_IMAGE_LOAD_ADDR + IMAGE_SIZE_CHECK_OFFSET))
						);
		TraceEndl (buffer);
	}

	if( !isSecBlank || (secImageVer != 0xFFFFFFFF) )
	{
		count = sprintf(buffer,"Secondary Image sizes (0x%X == 0x%X)\r",
				*( (uint32_t *) (SECONDARY_IMAGE_LOAD_ADDR + IMAGE_SIZE_OFFSET)),
				*( (uint32_t *) (SECONDARY_IMAGE_LOAD_ADDR + IMAGE_SIZE_CHECK_OFFSET))
						);
		TraceEndl(buffer);
		/* if size is the same for both offsets, the image is valid */
		if( (*( (uint32_t *) (SECONDARY_IMAGE_LOAD_ADDR + IMAGE_SIZE_OFFSET))) ==
			(*( (uint32_t *) (SECONDARY_IMAGE_LOAD_ADDR + IMAGE_SIZE_CHECK_OFFSET))) )
				isSecValid = TRUE;
	}

	// Both images are not valid so return
	if( !isPriValid && !isSecValid )
	{
		TraceEndl ("Both images are invalid");
		*pImageAddr = 0;
		return ( FALSE );
	}
	count = sprintf(buffer,"Primary Image version = 0x%X\r", priImageVer );
	TraceEndl (buffer);

	count = sprintf(buffer,"Secondary Image version = 0x%X\r", secImageVer );
	TraceEndl (buffer);

	TraceEndl ("Checking CRC");
	for ( count = 0; count < 100000000; count++)
		if( count % 1000000 == 0)
			TracePutc( '.' );

	count = sprintf(buffer,"Primary Image type = %d, version = 0x%02X\r",
			                                (priImageVer & 0xFF000000) >> 24,
											(priImageVer & 0x000000FF));
	TraceEndl( buffer );

	sprintf(buffer,"Secondary Image type = %d, version = 0x%02X\r",
			                                (secImageVer & 0xFF000000) >> 24,
											(secImageVer & 0x000000FF));
	TraceEndl( buffer );

    // Primary only valid
	if( isPriValid && !isSecValid )
	{
		*pImageAddr = PRIMARY_IMAGE_LOAD_ADDR;
		return ( TRUE );
	}

	// Secondary only valid
	if( !isPriValid && isSecValid )
	{
		*pImageAddr = SECONDARY_IMAGE_LOAD_ADDR;
		return ( TRUE );
	}

	// Both valid their types
	if( (priImageVer & 0x000000FF) >= (secImageVer & 0x000000FF) )
	{
		*pImageAddr = PRIMARY_IMAGE_LOAD_ADDR;
		return ( TRUE );
	}else
	{
		*pImageAddr = SECONDARY_IMAGE_LOAD_ADDR;
		return ( TRUE );
	}

#ifdef COMPUTE_BINARY_CHECKSUM
/*
 * The reserved Cortex-M3 exception vector location 7 (offset 0x001C
 * in the vector table) should contain the 2’s complement of the
 * checksum of table entries 0 through 6. This causes the checksum
 * of the first 8 table entries to be 0. This code checksums the
 * first 8 locations of the start of user flash. If the result is 0,
 * then the contents is deemed a 'valid' image.
 */
	checksum = 0;
	pmem = (unsigned *)USER_FLASH_START;
	for (i = 0; i <= 7; i++) {
		checksum += *pmem;
		pmem++;
	}
	if (checksum != 0)
	{
		return (FALSE);
	}
	else
#endif

	{
	    return (TRUE);
	}
}

void enter_serial_isp( void ) {
	char buffer[200];
	uint32_t ints[4];


	cmd = MENU;
	while (1) {

		switch (cmd) {
		case READY:


			cmd = ERASE_FLASH;
			break;

		case MENU:

			NVIC_DisableIRQ(UART0_IRQn);
			/*	Print Boot Version onto the LCD */
			if (u32IAP_ReadBootVersion(&ints[0], &ints[1])
					== IAP_STA_CMD_SUCCESS) {

				sprintf(buffer, "Boot Code version %d.%d", ints[0],
						ints[1]);
				NVIC_EnableIRQ(UART0_IRQn);
				TraceEndl(buffer);
			}

			NVIC_DisableIRQ(UART0_IRQn);

			if (u32IAP_ReadPartID(&ints[0]) == IAP_STA_CMD_SUCCESS) {
				snprintf((char *) string, MAX_STRING_SIZE, "Part ID: %d (%#x)",
						ints[0], ints[0]);

				sprintf(buffer, "Part ID: %d (%#x)", ints[0], ints[0]);
				NVIC_EnableIRQ(UART0_IRQn);
				TraceEndl(buffer);
			}

			NVIC_DisableIRQ(UART0_IRQn);

			u32IAP_ReadSerialNumber(&ints[0], &ints[1], &ints[2], &ints[3]);
			snprintf((char *) string, MAX_STRING_SIZE,
					"Serial #: %08X:%08X:%08X:%08X", ints[0], ints[1], ints[2],
					ints[3]);

			sprintf(buffer, "Serial #: %08X:%08X:%08X:%08X\n", ints[0],
					ints[1], ints[2], ints[3]);
			NVIC_EnableIRQ(UART0_IRQn);
			TraceEndl(buffer);


			cmd = READY;
			break;

		case ERASE_FLASH:
			NVIC_DisableIRQ(UART0_IRQn);
			/*	Erase the images stored in flash */
			if ((u32IAP_PrepareSectors(16, 20) == IAP_STA_CMD_SUCCESS)
					&& (u32IAP_EraseSectors(16, 20) == IAP_STA_CMD_SUCCESS)) {


			} else {

			}
			NVIC_EnableIRQ(UART0_IRQn);
			TraceEndl("ERASE_FLASH");

		case FLASH_IMG:
			/*	Clear the received data counter using in the load_mage function */
			received_data = 0;

			/*	Store a new image into flash */
			vXmodem1k_Client(&load_image);

			TraceEndl( "FLASH_IMG");
			cmd = SHOW;
			break;

		case SHOW:

			TraceEndl( "SHOW");
			while (1)
				;
			cmd = READY;
			break;
		}

	}
}


void ExecuteApplicationImage( unsigned int start_address )
{
	void (*user_code_entry)(void);

	unsigned *p;	// used for loading address of reset handler from user flash

	/* Change the Vector Table to the
	in case the user application uses interrupts */

	SCB->VTOR = (start_address & 0x1FFFFF80);

	// Load contents of second word of user flash - the reset handler address
	// in the applications vector table
	p = (unsigned *)(start_address + 4);

	// Set user_code_entry to be the address contained in that second word
	// of user flash
	user_code_entry = (void *) *p;

	// Jump to user application
    user_code_entry();
}

static uint32_t load_image(uint8_t *data, uint16_t length){

	if(length > 0)
	{
		/*	Prepare Sectors to be flashed */
		// TODO arrange sectors for primary image
		if(u32IAP_PrepareSectors(16, 20) == IAP_STA_CMD_SUCCESS){

//			/*	Copy data (already) located in RAM to flash */
//			if(u32IAP_CopyRAMToFlash(IMG_START_SECTOR + received_data, (uint32_t)data, length) == IAP_STA_CMD_SUCCESS)
//			{
//				/*	Verify the flash contents with the contents in RAM */
//				if(u32IAP_Compare(IMG_START_SECTOR + received_data, (uint32_t)data, length, 0) == IAP_STA_CMD_SUCCESS)
//				{
//					/*	Update and Print Received bytes counter */
//					received_data +=  length;
//					//snprintf((char *)string, MAX_STRING_SIZE, "Received %d of %d bytes", received_data, BMP->bmp_size);
//					return ( 1 );
//				}
//			}
		}
		/*	Error in the IAP functions */
		//  GLCD_DisplayString(5, 26, 0, "FAIL (RESET & ERASE IMAGE)");
		return ( 0 );

	}else{
		return ( 0 );
	}
}
