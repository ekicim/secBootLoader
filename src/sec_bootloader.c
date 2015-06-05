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
#include <stdio.h>
#include "iap_config.h"
#include "xmodem1k.h"
#include "uart.h"
#include "type.h"
#include "iap.h"
#include "trace.h"
#include "utils.h"
#include "wdt.h"
#include "gsm.h"
#include "math.h"
#include "exp_i2c.h"
#include "calibration.h"
#include "gps.h"
#include "trace.h"

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


static void DownloadSecondaryImage( void );


/**************************************************************************************************
 * LOCAL TYPES
 **************************************************************************************************/
enum DOWNLOAD_STATES
{
	INIT 	= 0,
	DOWNLOADING,
	FINISHED
};





#define FLASH_SECTOR_SIZE	256

/**************************************************************************************************
 * LOCAL VARIABLES
 **************************************************************************************************/
static uint8_t flashWriteBuffer[FLASH_SECTOR_SIZE] __attribute__ ((aligned(4)));
static uint32_t flashWriteIndex = 0;

/*	Function Prototype */
static uint32_t load_image(uint8_t *data, uint16_t length);


/*	Character array workspace for GLCD print functions */
#define MAX_STRING_SIZE		50
static uint8_t string[MAX_STRING_SIZE];
static uint32_t received_data = 0;


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



unsigned long last_msg_to_server = 0;
unsigned long last_persist = 0;
unsigned long last_gps_read = 0;
unsigned long last_power_control = 0;
char strbuf[5];
int is_sim_inserted = 0;
int last_signal_strength = 99;

float temperature1 = 4096;
int adcInput = 0;
int batteryVoltage = 0;
int extPowerPinVal = 0;
char cell_buf[15];
short is_cell_only = 1;


void ConfigurePins() {
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


char* GenerateTMessage(const char* alarm_str) {
	//$GPRMC,135729.000,A,4059.7869,N,02908.1216,E,0.42,139.41,091213,,,A*6A
	char buffer[200];
	char temp_gprmc[100];
	char speed_str[4], speed_buff[4], dir_str[4], dir_buff[4], time_str[10],
			lat_str[20], lon_str[20], lat_buff[20], lon_buff[20],
			mileage_str[6], mileage_buff[6], csq_str[3], csq_buff[3];
	ewns = 0;
	TraceNL("Entered GenerateTMessage()");
	char* t_message = malloc(255);
	if (last_valid_gprmc == NULL || strlen(last_valid_gprmc) < 50) {
		if (last_valid_gprmc == NULL)
			TraceNL("last_valid_gprmc == NULL, T Message :");
		else {
			int count = sprintf(buffer, "strlen(last_valid_gprmc) : %d\r",strlen(last_valid_gprmc));
		}
		sprintf(t_message, "[T%s-NOGPS", imei); // speed - heading append zeros.
		TraceNL(t_message);
	} else {
		int count = sprintf(buffer,"GPS Valid : strlen(last_valid_gprmc) : %d, last_valid_gprmc: : %s\r",strlen(last_valid_gprmc), last_valid_gprmc);
		UARTSend(PORT_TRACE, buffer, count);
		strcpy(temp_gprmc, last_valid_gprmc);
		ParseFields(temp_gprmc, tempPFields, 12, ",");
		//ParseFields(last_valid_gprmc, pFields, 12, ",");
		int len_lat = strlen(pFields[3]);
		int i;
		for (i = 0; i < 9 - len_lat; i++)
			lat_str[i] = '0';
		int j = 0;
		for (; i < 9; i++) {
			if (pFields[3][i - (9 - len_lat) + j] == '.')
				j = 1;
			lat_str[i] = pFields[3][i - (9 - len_lat) + j];
		}
		lat_str[8] = '\0';
		int len_lon = strlen(pFields[5]);
		for (i = 0; i < 10 - len_lon; i++)
			lon_str[i] = '0';
		j = 0;
		for (; i < 10; i++) {
			if (pFields[5][i - (10 - len_lon) + j] == '.')
				j = 1;
			lon_str[i] = pFields[5][i - (10 - len_lon) + j];
		}
		lon_str[9] = '\0';
		//TIME
		for (i = 0; i < strlen(pFields[1]); i++) {
			if (pFields[1][i] == '.')
				break;
			time_str[i] = pFields[1][i];
		}
		time_str[i] = '\0';
		// Speed & Direction
		int speed_count = sprintf(speed_buff, "%X", (int) gpsSpeed);
		int dir_count = sprintf(dir_buff, "%X", (int) gpsHeading);

		for (i = 0; i < 2 - speed_count; i++) {
			speed_str[i] = '0';
		}
		j = 0;
		for (; i < 2; i++) {
			speed_str[i] = speed_buff[j];
			j++;
		}
		speed_str[2] = '\0';

		for (i = 0; i < 3 - dir_count; i++) {
			dir_str[i] = '0';
		}
		j = 0;
		for (; i < 3; i++) {
			dir_str[i] = dir_buff[j];
			j++;
		}
		dir_str[3] = '\0';
		Trace("Speed STR : ");
		TraceNL(speed_str);
		Trace("Dir STR : ");
		TraceNL(dir_str);
		//Mileage
		int mileage_count = sprintf(mileage_buff, "%X",
				(per_mileage_val / 10000)); //DM to KM.
		for (i = 0; i < 5 - mileage_count; i++) {
			mileage_str[i] = '0';
		}
		j = 0;
		for (; i < 5; i++) {
			mileage_str[i] = mileage_buff[j];
			j++;
		}
		mileage_str[5] = '\0';
		//CSQ
		int csq_count = sprintf(csq_buff, "%d", last_signal_strength);
		for (i = 0; i < 2 - csq_count; i++) {
			csq_str[i] = '0';
		}
		j = 0;
		for (; i < 2; i++) {
			csq_str[i] = csq_buff[j];
			j++;
		}
		csq_str[2] = '\0';

		if ((strcmp(pFields[4], "N") == 0) && (strcmp(pFields[6], "W") == 0))
			ewns = 1;
		else if ((strcmp(pFields[4], "S") == 0) && (strcmp(pFields[6], "E")
				== 0))
			ewns = 2;
		else if ((strcmp(pFields[4], "S") == 0) && (strcmp(pFields[6], "W")
				== 0))
			ewns = 3;
		else
			ewns = 0;
		Trace("T_MESSAGE:");
		int stat = 11;
		if (device_power_state == low_power_state)
			stat = 10;
		sprintf(t_message, "[T%s%s%s%s%s%s%s%d%d%s%s", imei, pFields[9],
				time_str, lat_str, lon_str, speed_str, dir_str, ewns, stat,
				mileage_str, csq_str); // speed - heading append zeros.
	}
	//ADC
	#ifndef P55
	sprintf(t_message, "%s-020:%d", t_message, adcInput);
	#endif
	//Temperature Sensor 1
	#ifndef P55
	if (temperature1 < 85) {
		int tempInt = (temperature1 + 60) * 10;
		sprintf(t_message, "%s;104:%X", t_message, tempInt);
	}
	#endif
	//Alarm STR
	if (alarm_str != NULL && strlen(alarm_str) > 0)
		sprintf(t_message, "%s;%s", t_message, alarm_str);
	//Alarm STR
	if (cell_buf != NULL && strlen(cell_buf) > 0)
		sprintf(t_message, "%s;022:%s", t_message, cell_buf);
	if (lbs_str != NULL && strlen(lbs_str) > 0)
		sprintf(t_message, "%s;102:%s", t_message, lbs_str);
	sprintf(t_message, "%s;105:%d", t_message, last_signal_strength);
	sprintf(t_message, "%s;023:%d]", t_message, batteryVoltage);
	TraceNL(t_message);
	//strncpy(gt_message,t_message,sizeof(gt_message));
	return t_message;
}



void ChangeConfiguration(char* cnf_str, int channel) {
	TraceNL("Entered Change Configuration");
	char buffer[200];
	char* p_set = strstr(cnf_str, "#SET;");
	if (p_set == NULL)
		return;
	char param1[30], param2[30], param3[30], param4[30], param5[30];
	int seperator_count = 0;
	int param_index = 0;
	while (cnf_str != NULL && *cnf_str != '\0' && *cnf_str != '!') {
		if (*cnf_str == ';') {
			if (seperator_count > 0)
				switch (seperator_count) {
				case 1:
					param1[param_index] = '\0';
				case 2:
					param2[param_index] = '\0';
				case 3:
					param3[param_index] = '\0';
				case 4:
					param4[param_index] = '\0';
				case 5:
					param5[param_index] = '\0';
				}
			seperator_count++;
			param_index = 0;
			cnf_str++;
		}
		switch (seperator_count) {
		case 1:
			param1[param_index] = *cnf_str;
		case 2:
			param2[param_index] = *cnf_str;
		case 3:
			param3[param_index] = *cnf_str;
		case 4:
			param4[param_index] = *cnf_str;
		case 5:
			param5[param_index] = *cnf_str;
		}
		if (seperator_count > 0)
			param_index++;
		cnf_str++;
	}
	int
			count =
					sprintf(
							buffer,
							"Param1 : %s, Param2 : %s, Param3 : %s, Param4 : %s, Param5 : %s\r",
							param1, param2, param3, param4, param5);
	UARTSend(PORT_TRACE, buffer, count);
	if (strcmp(param1, "1") == 0 || strcmp(param1, "01") == 0) { // IP Port Request
		int count = sprintf(buffer, "IP-Port Request, IP : %s, Port : %s\r",
				param2, param3);
		UARTSend(PORT_TRACE, buffer, count);
		EEPROM_SaveString(cal_ip, param2);
		EEPROM_SaveInt64(cal_port, atoi(param3));
		NVIC_SystemReset();
	}

	if (strcmp(param1, "12") == 0) { // Reset Request
		TraceNL("Reset request received, reset.");
		NVIC_SystemReset();
	}

	if (strcmp(param1, "25") == 0) { // APN Request
		int count = sprintf(buffer,
				"APN Request, APN : %s, USER : %s, PASS : %s\r", param2,
				param3, param4);
		UARTSend(PORT_TRACE, buffer, count);
		EEPROM_SaveString(cal_apn, param2);
		EEPROM_SaveString(cal_apn_user, param3);
		EEPROM_SaveString(cal_apn_pass, param4);
		NVIC_SystemReset();
	}

	if (strcmp(param1, "4") == 0 || strcmp(param1, "04") == 0) {
		if (strcmp(param2, "0") == 0) {
			int count = sprintf(buffer,
					"Update Period Change Request, ON : %s, OFF : %s\r",
					param3, param4);
			UARTSend(PORT_TRACE, buffer, count);
			EEPROM_SaveInt64(cal_ign_on_period, atoi(param3));
			EEPROM_SaveInt64(cal_ign_off_period, atoi(param4));
			//EEPROM_SaveString(cal_ign_off_period,param4);
			LoadTimings();
		} else if (strcmp(param2, "1") == 0) { //Roaming Periods
			int
					count =
							sprintf(
									buffer,
									"Update Period(Roaming) Change Request, ON : %s, OFF : %s\r",
									param3, param4);
			UARTSend(PORT_TRACE, buffer, count);
			EEPROM_SaveInt64(cal_ign_on_roaming_period, atoi(param3));
			EEPROM_SaveInt64(cal_ign_off_roaming_period, atoi(param4));
			LoadTimings();
		}
	}

	else if (strcmp(param1, "11") == 0) { // Set Mileage
		TraceNL("Change Mileage.");
		EEPROM_SaveInt64(cal_mileage, atol(param2));
		per_mileage_val = EEPROM_LoadInt64(cal_mileage);
	} else if (strcmp(param1, "32") == 0) { // Cancel low power mode.
		TraceNL("Low power mode.");
		EEPROM_SaveInt64(cal_lowpow_cancel, atoi(param2));
		LoadTimings();
		NVIC_SystemReset(); // Added on 20150204
	} else if (strcmp(param1, "33") == 0) { // Low power wake up period (minutes)
		TraceNL("Low power wake up period.");
		EEPROM_SaveInt64(cal_lowpow_period, atoi(param2));
		LoadTimings();
	} else if (strcmp(param1, "34") == 0) { // Deep Sleep vs Sleep.
		TraceNL("Low power mode selection DEEP POWER DOWN vs POWER DOWN");
		EEPROM_SaveInt64(cal_lowpow_mode, atoi(param2));
		LoadTimings();
	} else if (strcmp(param1, "23") == 0) { // Position on demand (added on 20150204)
		TraceNL("Send  position");
		char* result = GenerateTMessage("");
		int is_sent = GSM_SendToServerTCP(result);
	}
	char p_response[200];
	sprintf(p_response, "@SET;%s70;%s", imei, &p_set[5]);
	switch (channel) {
	case 0:
		TraceNL(p_response);
	case 1:
		GSM_SendToServerTCP(p_response);
	}
}

int InitializeServerConn() {
	int init_result = GSM_InitModule();
	if (init_result == FAIL){
		GSM_ShutdownModule();
		DelayMs(2000);
		init_result = GSM_InitModule();
	}
	if(init_result == FAIL)
		return init_result;
	//DelayMs(7000); // Wait for simcard.
	WDTFeed();
	is_sim_inserted = 1;
	char buffer[200];
	GSM_GetImei();
	GSM_GetImsi(); //
	int conn_stat = GSM_GetRegStat();
	uint8_t loop_count = 0;
	while (conn_stat == FAIL && loop_count < 13){
		//signal_strength = GSM_GetSignalStrength();
		//count = sprintf(buffer, "\Signal Strength : %d\n", signal_strength);
		//UARTSend(PORT_TRACE, buffer, count);
		TraceNL("Network registration error");
		DelayMs(2000);
		WDTFeed();
		conn_stat = GSM_GetRegStat();
		loop_count++;
	}
	int is_conn = FAIL;
	if (conn_stat == SUCCESS){
		TraceNL("Registered to gsm network.");
		last_signal_strength = GSM_GetSignalStrength();
		int count = sprintf(buffer, "\Signal Strength : %d\n", last_signal_strength);
		UARTSend(PORT_TRACE, buffer, count);
		//if (signal_strength == 99 || signal_strength <5)
		//	return FAIL;
		is_conn = GSM_ConnectToTrio();
		if (is_conn == FAIL)
			is_conn = GSM_ConnectToTrio();
		if (is_conn == FAIL)
			is_conn = GSM_ConnectToTrio();
		if (is_conn == FAIL)
			is_conn = GSM_ConnectToTrio();
		if (is_conn == FAIL)
			is_conn = GSM_ConnectToTrio();
	}
	return is_conn;
}

int main(void) {
	char buffer[200];
	device_power_state = high_power_state;
	SystemInit();
	LPC_SC->CLKSRCSEL |= 0x01;//0x01;
	LPC_SC->PLL0CFG |= 0x01; // Select external osc. as main clock.
	LPC_SC->CCLKCFG = 0x03; // Main PLL is divided by 8
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000 - 1); // Generate f each 1 ms, used to enable DelayMs function?
	WDTInit(WDT_FEED_30_SECS);

	ConfigurePins();
	UARTInit(PORT_TRACE, 115200);
	UARTInit(PORT_GSM, Baudrate);
	UARTInit(PORT_GPS, Baudrate);
	TraceNL("Hello P65 20150520.");
	sprintf(buffer, "SystemCoreClock = %d Hz\n", SystemCoreClock);
	//UARTSend(PORT_TRACE, buffer, count);
	Trace(buffer);
	if (EEPROM_Init() == 0) /* initialize I2c */{
		TraceNL("EEPROM Init Error."); /* Fatal error */
	} else {
		TraceNL("EEPROM Init Ok.");
	}


	uint16_t crc = u16CRC_Calc16("adssadsadsadsadsadsa", 20);

	sprintf(buffer, "CRC of adssadsadsadsadsadsa is : = %d \r\n", crc);
	Trace(buffer);
	LoadParams();
	unsigned long int lastGPSLedToggle = 0;
	memset(buffer, 0xAA, sizeof(buffer));
	//u32BootLoader_ProgramFlash(buffer,0x30000,16);
	WDTFeed();
	int is_sent = SUCCESS;

	int is_last_speed_zero = 1;
	int count ;

	device_power_state = high_power_state;

	/// TODO set a timer in order of update failure to return to older

	TraceNL( "Initializing Server Connection" );

	if( IsUpgradeRequested() )
	{
		TraceNL( "System image upgrade requested" );
		int8_t 	trials = 1;
		while( trials-- > 0 )
		{
			TraceNL( "Trial" );
			/*
			 * 	Initialize GSM module
			 * 	Setup a server connection
			 *
			 */

			// InitializeServerConn();
			if( GSM_ConnectToTrioUpgradeServer() == SUCCESS )
			{
				TraceNL( "Server Connection Established to Upgrade server" );

				//GSM_SendToServerTCP( "[ST;70;r2246;P65-20150204-1;;HELLO]" );

				WDTFeed( );

				DownloadSecondaryImage();

				WDTFeed( );
				TraceNL( "Download finished " );
				ExecuteApplicationImage( SECONDARY_IMAGE_LOAD_ADDR );

				for ( count = 0; count < 100000000; count++)
					if( count % 10000000 == 0)
						TracePutc( '.' );
			}
			TraceNL( "Trial end" );
		}
		TraceNL( "Finished upgrading" );
	}

//	uint32_t imageAddr;
//
//	SystemInit();
//	LPC_SC->CLKSRCSEL |= 0x01;//0x01;
//	LPC_SC->PLL0CFG   |= 0x01; // Select external osc. as main clock.
//	LPC_SC->CCLKCFG    = 0x03; // Main PLL is divided by 8
//	SystemCoreClockUpdate();
//
//	UARTInit(PORT_TRACE, 115200);
//
//	TraceNL ("Bootloader is starting");
//	TraceNL ("Checking application image's validity");
//
//	// Check to see if there is a user application in the LPC1768's flash memory.
//	if( CheckApplicationImageValidity( &imageAddr ) )
//	{
//		ExecuteApplicationImage( imageAddr );
//	}
//
//	TraceNL ("Unable to locate any valid image to run");
//
//	// Valid application does not exists. Get one from UART 0
//	enter_serial_isp();
//
//	while ( 1 );	// assert should not get here
//	return (0);
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
		TraceNL(buffer);
	}

	if( !isSecBlank || (secImageVer != 0xFFFFFFFF) )
	{
		count = sprintf(buffer,"Secondary Image sizes (0x%X == 0x%X)\r",
				*( (uint32_t *) (SECONDARY_IMAGE_LOAD_ADDR + IMAGE_SIZE_OFFSET)),
				*( (uint32_t *) (SECONDARY_IMAGE_LOAD_ADDR + IMAGE_SIZE_CHECK_OFFSET))
						);
		TraceNL(buffer);
		/* if size is the same for both offsets, the image is valid */
		if( (*( (uint32_t *) (SECONDARY_IMAGE_LOAD_ADDR + IMAGE_SIZE_OFFSET))) ==
			(*( (uint32_t *) (SECONDARY_IMAGE_LOAD_ADDR + IMAGE_SIZE_CHECK_OFFSET))) )
				isSecValid = TRUE;
	}

	// Both images are not valid so return
	if( !isPriValid && !isSecValid )
	{
		TraceNL ("Both images are invalid");
		*pImageAddr = 0;
		return ( FALSE );
	}
	count = sprintf(buffer,"Primary Image version = 0x%X\r", priImageVer );
	TraceNL (buffer);

	count = sprintf(buffer,"Secondary Image version = 0x%X\r", secImageVer );
	TraceNL (buffer);

	TraceNL ("Checking CRC");
	for ( count = 0; count < 100000000; count++)
		if( count % 1000000 == 0)
			TracePutc( '.' );

	count = sprintf(buffer,"Primary Image type = %d, version = 0x%02X\r",
			                                (priImageVer & 0xFF000000) >> 24,
											(priImageVer & 0x000000FF));
	TraceNL( buffer );

	sprintf(buffer,"Secondary Image type = %d, version = 0x%02X\r",
			                                (secImageVer & 0xFF000000) >> 24,
											(secImageVer & 0x000000FF));
	TraceNL( buffer );
	return ( TRUE );
}
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
** 					FALSE   upgrade is not requested
**
******************************************************************************/
uint32_t	IsUpgradeRequested( void )
{
	// TODO for test purposes allways upgrade requested later change
	if( (*( (uint32_t *) UPGRADE_PARAMETERS_ADDR) ) == 0xFFFFFFFF )
	{
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
	XModem1K_Client( &load_image );
}




void enter_serial_isp( void ) {

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

static uint32_t load_image(uint8_t *data, uint16_t length){

	char buffer[250];
	uint32_t rc;
	int i;


	sprintf(buffer, "Totally : %d  lenght : %d\r\n",
			received_data, length);

	TraceDumpHex(buffer, strlen(buffer));

	if( length == 0 && flashWriteIndex == 0 )
	{
		// Finished and all previous data has been written
		return ( 2 ); // return non zero to indicate success
	}


	for( i = 0; i < length; i++ )
	{
		flashWriteBuffer[flashWriteIndex++] = data[i];
	}

	if( length == 0 && (flashWriteIndex % FLASH_SECTOR_SIZE) )
	{
		for( i = flashWriteIndex; i < FLASH_SECTOR_SIZE; i++ )
		{
			flashWriteBuffer[flashWriteIndex++] = 0xFF;
		}

	}



	if( flashWriteIndex && ((flashWriteIndex % FLASH_SECTOR_SIZE) == 0) )
	{
		sprintf(buffer, "Totally : %d  flashWriteIndex : %d\r\n",
				received_data, flashWriteIndex);
//		TraceDumpHex(buffer, strlen(buffer));
//		TraceDumpHex(flashWriteBuffer, flashWriteIndex);

		/*	Prepare Sectors to be flashed */
		// TODO arrange sectors for primary image
		if (u32IAP_PrepareSectors(22, 27) == IAP_STA_CMD_SUCCESS) {
			TraceNL("prepare ");

			rc = u32IAP_CopyRAMToFlash(
					SECONDARY_IMAGE_LOAD_ADDR + received_data,
					(uint32_t) flashWriteBuffer,
					flashWriteIndex
					);

			sprintf( buffer, "Copy Ram result code : %d\r\n", rc );
			TraceNL( buffer );
			/*	Copy data (already) located in RAM to flash */
			if (rc == IAP_STA_CMD_SUCCESS) {
				TraceNL( "copied " );

				rc = u32IAP_Compare( SECONDARY_IMAGE_LOAD_ADDR + received_data,
						             (uint32_t) flashWriteBuffer,
									 flashWriteIndex, 0
									);
				sprintf( buffer, "u32IAP_Compare : %d  wrote %d \r\n", rc , flashWriteIndex );
				TraceNL( buffer );

				/*	Verify the flash contents with the contents in RAM */
				if (rc == IAP_STA_CMD_SUCCESS) {
					/*	Update and Print Received bytes counter */
					received_data += flashWriteIndex;
					//snprintf((char *)string, MAX_STRING_SIZE, "Received %d of %d bytes", received_data, BMP->bmp_size);
					TraceNL( "verified " );

					flashWriteIndex = 0;
					return (1);
				} else {
					int count;
					TraceDumpHex( SECONDARY_IMAGE_LOAD_ADDR + received_data, flashWriteIndex);

					received_data += flashWriteIndex;
					//snprintf((char *)string, MAX_STRING_SIZE, "Received %d of %d bytes", received_data, BMP->bmp_size);
					TraceNL( "verification failed " );

					for ( count = 0; count < 100000000; count++)
						if( count % 10000000 == 0)
							TracePutc( '.' );

					flashWriteIndex = 0;
				}
			}
		}
	}

	return (0);
}
