
#ifndef  _IAP_CONFIG_H
#define  _IAP_CONFIG_H


/* Little endian size of the image is placed at offset */
#define IMAGE_SIZE_OFFSET				0x000002f4
/* Size of the image is writen to the following
 * offset as well and this second check is used
 * to validate the reception of the image is ended
 * without problem. The image is valid to be executed.
 */
#define IMAGE_SIZE_CHECK_OFFSET			0x0002FFF8

/* Optionally 32bit IEET CRC of the image is loaded
 * at the following offset to further validate the
 * image
 */
#define IMAGE_CRC_OFFSET				0x0002FFFC

// This is not hardware sector size. it is write
// amount
#define FLASH_SECTOR_SIZE			1024

// bootrom check this constant to decide validity of image
#define IMAGE_CONSTANT 				0xAA55BC47

/* version information is placed at offset */
#define IMAGE_VERSION_OFFSET		0x000002f8
#define PRIMARY_IMAGE_LOAD_ADDR		SECTOR_16_START
#define PRIMARY_IMAGE_END_ADDR		SECTOR_22_START /* exclusive */
#define SECONDARY_IMAGE_LOAD_ADDR	SECTOR_22_START
#define SECONDARY_IMAGE_END_ADDR	SECTOR_28_START /* exclusive */

#define PRIMARY_IMAGE_START_SEC		16
#define PRIMARY_IMAGE_END_SEC		21

#define SECONDARY_IMAGE_START_SEC	22
#define SECONDARY_IMAGE_END_SEC		27

#define MAX_SECTORS					29

#define UPGRADE_PARAMETERS_SEC		15
#define UPGRADE_PARAMETERS_ADDR		( PRIMARY_IMAGE_LOAD_ADDR - 1024 )


#define USER_FLASH_START	(sector_start_map[USER_START_SECTOR])
#define USER_FLASH_END		(sector_end_map[MAX_USER_SECTOR])
#define USER_FLASH_SIZE		((USER_FLASH_END - USER_FLASH_START) + 1)
#define MAX_FLASH_SECTOR	30

/* Define start address of each Flash sector */
#define SECTOR_0_START      0x00000000
#define SECTOR_1_START      0x00001000
#define SECTOR_2_START      0x00002000
#define SECTOR_3_START      0x00003000
#define SECTOR_4_START      0x00004000
#define SECTOR_5_START      0x00005000
#define SECTOR_6_START      0x00006000
#define SECTOR_7_START      0x00007000
#define SECTOR_8_START      0x00008000
#define SECTOR_9_START      0x00009000
#define SECTOR_10_START     0x0000A000
#define SECTOR_11_START     0x0000B000
#define SECTOR_12_START     0x0000C000
#define SECTOR_13_START     0x0000D000
#define SECTOR_14_START     0x0000E000
#define SECTOR_15_START     0x0000F000
#define SECTOR_16_START     0x00010000
#define SECTOR_17_START     0x00018000
#define SECTOR_18_START     0x00020000
#define SECTOR_19_START     0x00028000
#define SECTOR_20_START     0x00030000
#define SECTOR_21_START     0x00038000
#define SECTOR_22_START     0x00040000
#define SECTOR_23_START     0x00048000
#define SECTOR_24_START     0x00050000
#define SECTOR_25_START     0x00058000
#define SECTOR_26_START     0x00060000
#define SECTOR_27_START     0x00068000
#define SECTOR_28_START     0x00070000
#define SECTOR_29_START     0x00078000



/* Define end address of each Flash sector */
#define SECTOR_0_END        0x00000FFF
#define SECTOR_1_END        0x00001FFF
#define SECTOR_2_END        0x00002FFF
#define SECTOR_3_END        0x00003FFF
#define SECTOR_4_END        0x00004FFF
#define SECTOR_5_END        0x00005FFF
#define SECTOR_6_END        0x00006FFF
#define SECTOR_7_END        0x00007FFF
#define SECTOR_8_END        0x00008FFF
#define SECTOR_9_END        0x00009FFF
#define SECTOR_10_END       0x0000AFFF
#define SECTOR_11_END       0x0000BFFF
#define SECTOR_12_END       0x0000CFFF
#define SECTOR_13_END       0x0000DFFF
#define SECTOR_14_END       0x0000EFFF
#define SECTOR_15_END       0x0000FFFF
#define SECTOR_16_END       0x00017FFF
#define SECTOR_17_END       0x0001FFFF
#define SECTOR_18_END       0x00027FFF
#define SECTOR_19_END       0x0002FFFF
#define SECTOR_20_END       0x00037FFF
#define SECTOR_21_END       0x0003FFFF
#define SECTOR_22_END       0x00047FFF
#define SECTOR_23_END       0x0004FFFF
#define SECTOR_24_END       0x00057FFF
#define SECTOR_25_END       0x0005FFFF
#define SECTOR_26_END       0x00067FFF
#define SECTOR_27_END       0x0006FFFF
#define SECTOR_28_END       0x00077FFF
#define SECTOR_29_END       0x0007FFFF

                          
#endif  /* __IAP_CONFIG_H__ */
