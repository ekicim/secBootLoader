/*****************************************************************************
 * $Id$
 *
 * Project: 	NXP LPC1700 Secondary Bootloader Example
 *
 * Description: Provides access to In-Application Programming (IAP) routines
 * 			    contained within the bootROM sector of LPC1100 devices.
 *
 * Copyright(C) 2010, NXP Semiconductor
 * All rights reserved.
 *
 *****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 *****************************************************************************/
#include "iap.h"
#include "iap_config.h"
#include "trace.h"
#include <stdio.h>
#include <LPC17xx.h>

/* IAP Command Definitions */
#define	IAP_CMD_PREPARE_SECTORS			50
#define	IAP_CMD_COPY_RAM_TO_FLASH		51
#define	IAP_CMD_ERASE_SECTORS			52
#define	IAP_CMD_BLANK_CHECK_SECTORS		53
#define	IAP_CMD_READ_PART_ID			54
#define	IAP_CMD_READ_BOOT_ROM_VERSION	55
#define	IAP_CMD_COMPARE					56
#define	IAP_CMD_REINVOKE_ISP			57
#define IAP_CMD_READ_SERIAL_NUMBER		58

/* IAP boot ROM location and access function */
#define IAP_ROM_LOCATION				0x1FFF1FF1UL
#define IAP_EXECUTE_CMD(a, b)			((void (*)())(IAP_ROM_LOCATION))(a, b)


// If COMPUTE_BINARY_CHECKSUM is defined, then code will check that checksum
// contained within binary image is valid.
#define COMPUTE_BINARY_CHECKSUM

const unsigned sector_start_map[MAX_FLASH_SECTOR] = {SECTOR_0_START,             \
SECTOR_1_START,SECTOR_2_START,SECTOR_3_START,SECTOR_4_START,SECTOR_5_START,      \
SECTOR_6_START,SECTOR_7_START,SECTOR_8_START,SECTOR_9_START,SECTOR_10_START,     \
SECTOR_11_START,SECTOR_12_START,SECTOR_13_START,SECTOR_14_START,SECTOR_15_START, \
SECTOR_16_START,SECTOR_17_START,SECTOR_18_START,SECTOR_19_START,SECTOR_20_START, \
SECTOR_21_START,SECTOR_22_START,SECTOR_23_START,SECTOR_24_START,SECTOR_25_START, \
SECTOR_26_START,SECTOR_27_START,SECTOR_28_START,SECTOR_29_START					 };

const unsigned sector_end_map[MAX_FLASH_SECTOR] = {SECTOR_0_END,SECTOR_1_END,    \
SECTOR_2_END,SECTOR_3_END,SECTOR_4_END,SECTOR_5_END,SECTOR_6_END,SECTOR_7_END,   \
SECTOR_8_END,SECTOR_9_END,SECTOR_10_END,SECTOR_11_END,SECTOR_12_END,             \
SECTOR_13_END,SECTOR_14_END,SECTOR_15_END,SECTOR_16_END,SECTOR_17_END,           \
SECTOR_18_END,SECTOR_19_END,SECTOR_20_END,SECTOR_21_END,SECTOR_22_END,           \
SECTOR_23_END,SECTOR_24_END,SECTOR_25_END,SECTOR_26_END,                         \
SECTOR_27_END,SECTOR_28_END,SECTOR_29_END										 };


/*****************************************************************************
** Function name:	u32IAP_PrepareSectors
**
** Description:		Prepares sector(s) for erasing or write operations. This
** 					command must be executed before executing the "Copy RAM to
** 					Flash" or "Erase Sector(s)" commands.
**
** Parameters:		u32StartSector - Number of first sector to prepare.
** 					u32EndSector - Number of last sector to prepare.
**
** Returned value:	Status code returned by IAP ROM function.
**
******************************************************************************/
uint32_t u32IAP_PrepareSectors(uint32_t u32StartSector, uint32_t u32EndSector)
{
	uint32_t u32Status;
	uint32_t au32Result[5];
	uint32_t au32Command[5];

	if (u32EndSector < u32StartSector)
	{
		u32Status = IAP_STA_INVALD_PARAM;
	}
	else
	{
		au32Command[0] = IAP_CMD_PREPARE_SECTORS;
		au32Command[1] = u32StartSector;
		au32Command[2] = u32EndSector;

		IAP_EXECUTE_CMD(au32Command, au32Result);

		u32Status = au32Result[0];
	}
	return ( u32Status );
}

/*****************************************************************************
** Function name:	u32IAP_CopyRAMToFlash
**
** Description:		Program the flash memory with data stored in RAM.
**
** Parameters:	   	u32DstAddr - Destination Flash address, should be a 256
**                               byte boundary.
**			 		u32SrcAddr - Source RAM address, should be a word boundary
**			 		u32Len     - Number of 8-bit bytes to write, must be 256
**								 512, 1024, or 4096.
*
** Returned value:	Status code returned by IAP ROM function.
**
******************************************************************************/
uint32_t u32IAP_CopyRAMToFlash(uint32_t u32DstAddr, uint32_t u32SrcAddr, uint32_t u32Len)
{
	uint32_t au32Result[5];
	uint32_t au32Command[5];

	__disable_irq();
	au32Command[0] = IAP_CMD_COPY_RAM_TO_FLASH;
	au32Command[1] = u32DstAddr;
	au32Command[2] = u32SrcAddr;
	au32Command[3] = u32Len;
	au32Command[4] = SystemCoreClock / 1000UL;	/* Core clock frequency in kHz */

	IAP_EXECUTE_CMD(au32Command, au32Result);

	__enable_irq();
	return ( au32Result[0] );
}

/*****************************************************************************
** Function name:	u32IAP_EraseSectors
**
** Description:		Erase a sector or multiple sectors of on-chip Flash memory.
**
** Parameters:		u32StartSector - Number of first sector to erase.
** 					u32EndSector - Number of last sector to erase.
*
** Returned value:	Status code returned by IAP ROM function.
**
******************************************************************************/
uint32_t u32IAP_EraseSectors(uint32_t u32StartSector, uint32_t u32EndSector)
{
	uint32_t u32Status;
	uint32_t au32Result[5];
	uint32_t au32Command[5];

	if (u32EndSector < u32StartSector)
	{
		u32Status = IAP_STA_INVALD_PARAM;
	}
	else
	{
		__disable_irq();
		au32Command[0] = IAP_CMD_ERASE_SECTORS;
		au32Command[1] = u32StartSector;
		au32Command[2] = u32EndSector;
		au32Command[3] = SystemCoreClock / 1000UL;	/* Core clock frequency in kHz */

		IAP_EXECUTE_CMD(au32Command, au32Result);

		__enable_irq();
		u32Status = au32Result[0];
	}
	return ( u32Status );
}

/*****************************************************************************
** Function name:	u32IAP_BlankCheckSectors
**
** Description:		Blank check a sector or multiple sectors of on-chip flash
** 					memory.
**
** Parameters:		u32StartSector - Number of first sector to check.
** 					u32EndSector - Number of last sector to check.
** 					pu32Result[0] - Offset of the first non blank word location
**                  if the Status Code is IAP_STA_SECTOR_NOT_BLANK.
** 					pu32Result[1] - Contents of non blank word location.
**
** Returned value:	Status code returned by IAP ROM function.
**
******************************************************************************/
uint32_t u32IAP_BlankCheckSectors(uint32_t u32StartSector, uint32_t u32EndSector, uint32_t *pu32Result)
{
	uint32_t u32Status;
	uint32_t au32Result[5];
	uint32_t au32Command[5];

	if (u32EndSector < u32StartSector)
	{
		u32Status = IAP_STA_INVALD_PARAM;
	}
	else
	{
		au32Command[0] = IAP_CMD_BLANK_CHECK_SECTORS;
		au32Command[1] = u32StartSector;
		au32Command[2] = u32EndSector;

		IAP_EXECUTE_CMD(au32Command, au32Result);

		if (au32Result[0] == IAP_STA_SECTOR_NOT_BLANK)
		{
			*pu32Result       = au32Result[0];
			*(pu32Result + 1) = au32Result[1];
		}
		u32Status = au32Result[0];
	}
	return ( u32Status );
}

/*****************************************************************************
** Function name:	u32IAP_ReadPartID
**
** Description:		Read the part identification number.
**
** Parameters:		pu32PartID - Pointer to storage for part ID number.
*
** Returned value:	Status code returned by IAP ROM function.
**
******************************************************************************/
uint32_t u32IAP_ReadPartID(uint32_t *pu32PartID)
{
	uint32_t au32Result[5];
	uint32_t au32Command[5];

	au32Command[0] = IAP_CMD_READ_PART_ID;

	IAP_EXECUTE_CMD(au32Command, au32Result);

	*pu32PartID = au32Result[1];

	return ( au32Result[0] );
}

/*****************************************************************************
** Function name:	u32IAP_ReadBootVersion
**
** Description:		Read the boot code version number.
**
** Parameters:		pu32Major - Major version number in ASCII format.
** 					pu32Minor - Minor version number in ASCII format.
**
** Returned value:	Status code returned by IAP ROM function.
**
******************************************************************************/
uint32_t u32IAP_ReadBootVersion(uint32_t *pu32Major, uint32_t *pu32Minor)
{
	uint32_t au32Result[5];
	uint32_t au32Command[5];

	au32Command[0] = IAP_CMD_READ_BOOT_ROM_VERSION;

	IAP_EXECUTE_CMD(au32Command, au32Result);

	*pu32Major = (au32Result[1] & 0x0000FF00UL) >> 8;
	*pu32Minor = au32Result[1] & 0x000000FFUL;

	return ( au32Result[0] );
}

/*****************************************************************************
** Function name:	u32IAP_ReadBootVersion
**
** Description:		Read the boot code version number.
**
** Parameters:		pu32Major - Major version number in ASCII format.
** 					pu32Minor - Minor version number in ASCII format.
**
** Returned value:	Status code returned by IAP ROM function.
**
******************************************************************************/
void u32IAP_ReadSerialNumber(uint32_t *pu32byte0, uint32_t *pu32byte1,
								 uint32_t *pu32byte2, uint32_t *pu32byte3)
{
	uint32_t au32Result[5];
	uint32_t au32Command[5];

	au32Command[0] = IAP_CMD_READ_SERIAL_NUMBER;

	IAP_EXECUTE_CMD(au32Command, au32Result);

	*pu32byte0 = au32Result[0];
	*pu32byte1 = au32Result[1];
	*pu32byte2 = au32Result[2];
	*pu32byte3 = au32Result[3];

	return;
}

/*****************************************************************************
** Function name:	u32IAP_Compare
**
** Description:		Compares the memory contents at two locations.
**
** Parameters:		u32Len - Number of bytes to compare, must be a multiple of 4.
**					pu32Offset - Offset of the first mismatch if the Status Code is COMPARE_ERROR
**
** Returned value:	Status code returned by IAP ROM function.
**
******************************************************************************/
uint32_t u32IAP_Compare(uint32_t u32DstAddr, uint32_t u32SrcAddr, uint32_t u32Len, uint32_t *pu32Offset)
{
	uint32_t au32Result[5];
	uint32_t au32Command[5];

	au32Command[0] = IAP_CMD_COMPARE;
	au32Command[1] = u32DstAddr;
	au32Command[2] = u32SrcAddr;
	au32Command[3] = u32Len;

	IAP_EXECUTE_CMD(au32Command, au32Result);

	if (au32Result[0] == IAP_STA_COMPARE_ERROR)
	{
		if (pu32Offset != 0)
		{
			*pu32Offset = au32Result[1];
		}
	}
	return ( au32Result[0] );
}

/*****************************************************************************
** Function name:	vIAP_ReinvokeISP
**
** Description:		Invoke the bootloader in ISP mode.
**
** Parameters:		None.
*
** Returned value:	None.
**
******************************************************************************/
void vIAP_ReinvokeISP(void)
{
	uint32_t au32Result[5];
	uint32_t au32Command[5];

	au32Command[0] = IAP_CMD_REINVOKE_ISP;

	IAP_EXECUTE_CMD(au32Command, au32Result);
}


/*****************************************************************************
** Function name:	WriteImageSignature
**
** Description:		The function writes the image signature of and upgrade
** 					image to the end of the image. Image validation procedure
** 					checks the image validity using this information if the
** 					CRC does not match. Upgrade is cancelled and primary image
** 					is executed.
**
** Parameters:		address   address to write size and CRC
** 					size      size in bytes of the image4
** 					crc       16 bit CRC of the image
**
** Returned value:	none
**
******************************************************************************/
void WriteImageSignature( uint32_t  size, uint32_t crc )
{

	char buffer[1024];
	uint32_t rc;
	int i;

	char* startAddr = (char *)(SECONDARY_IMAGE_END_ADDR - 1024);

	sprintf(buffer, "Writing signature: 0x%X   CRC : %X\r\n", size, crc);
	Trace( buffer );

	for( i = 0; i < 1024; i++ )
	{
		buffer[i++] = *startAddr++;
	}

	uint32_t* ptrCRC  = (uint32_t*)&buffer[ 1024 - 4 ];
	*ptrCRC		= crc;

	uint32_t* ptrSize = (uint32_t*)&buffer[ 1024 - 8 ];
	*ptrSize	= size;

	uint32_t* ptrConst = (uint32_t*)&buffer[ 1024 - 12 ];
	*ptrConst	= IMAGE_CONSTANT;

	if (u32IAP_PrepareSectors(SECONDARY_IMAGE_END_SEC,
			SECONDARY_IMAGE_END_SEC) == IAP_STA_CMD_SUCCESS)
	{
		TraceNL("prepared ");
		u32IAP_EraseSectors( SECONDARY_IMAGE_END_SEC, SECONDARY_IMAGE_END_SEC );
		TraceNL("Erased ");

		u32IAP_PrepareSectors(SECONDARY_IMAGE_END_SEC,	SECONDARY_IMAGE_END_SEC);

		rc = u32IAP_CopyRAMToFlash((SECONDARY_IMAGE_END_ADDR - 1024), (uint32_t) buffer, 1024);

		sprintf(buffer, "Copy Ram result code : %d\r\n", rc);
		TraceNL(buffer);
		/*	Copy data (already) located in RAM to flash */
		if (rc == IAP_STA_CMD_SUCCESS) {
			TraceNL("copied ");
		}
	}

	return;
}

/*****************************************************************************
 **                            End Of File
 *****************************************************************************/
