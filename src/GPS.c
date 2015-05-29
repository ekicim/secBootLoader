 /******************** (C) Trio Mobil 2014 LPC1768 MCU   **********************
 * File Name          : P65 GSM Routines
 * Author			  : Nevzat Ataklı
 * Version            : V1.0.0
 * Date               : 2014/01/04
 ****************************************************************************/
#include <uart.h>
#include <trace.h>
#include "type.h"
#include "GPS.h"
#include "Utils.h"
#include "LPC17xx.h"
#include "string.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// This function seperates the single input string in to numFields substrings
void ParseFields(char* inputBuffer, char** Fields, uint32_t numFields,char* delimiterChars){
	char* pString = inputBuffer;
	char* pField;
	uint32_t i;
	for (i = 0; i < numFields; i++) {
		pField = strtok(pString, delimiterChars);
		if (pField != NULL) {
			Fields[i] = pField;
			//TraceNL(pField);
		} else {
			Fields[i] = "";
		}
		pString = NULL; //to make strtok continue parsing the next field rather than start again on the original string (see strtok documentation for more details)
	}
}


/* $GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70
 1    2    3    4    5     6    7    8      9     10  11 12
 1   220516     Time Stamp
 2   A          validity - A-ok, V-invalid
 3   5133.82    current Latitude
 4   N          North/South
 5   00042.24   current Longitude
 6   W          East/West
 7   173.8      Speed in knots
 8   231.8      True course
 9   130694     Date Stamp
 10  004.2      Variation
 11  W          East/West
 12  *70        checksum */

void GPS_SetGPSLed(int value) {
	if (value == 0) {
		LPC_GPIO4->FIOCLR = (1 << 29);
		gps_led_val = 0;
	} else {
		LPC_GPIO4->FIOSET = (1 << 29);
		gps_led_val = 1;
	}
}
void GPS_CleanBuffer() {
	uint8_t temp_gps_data[0x7F8];
	ReadUart(temp_gps_data, PORT_GPS);
}
char* GGAFields[15];
char* GSAFields[18];
int GPS_GetGPRMC() {
	uint8_t temp_gps_data[0x7F8];
	TraceNL("Entered GPS_GetGPRMC.");
	char gprmc[150];
	char gpgga[150];
	char gpgsa[150];
	char fix[5],hdop[5],vdop[5],pdop[5];
	char temp_gprmc[150];
	//char* gprmc = malloc(255);
	//strcpy(temp_gps_data,"$GPRMC,092750.000,A,5321.6802,N,00630.3372,W,0.02,31.66,280511,,,A*43\r");
	ReadUart(temp_gps_data, PORT_GPS);
	//ReadGPSByPolling(temp_gps_data,1500);
	//strcpy(temp_gps_data,"$GPRMC,092750.000,A,5321.6802,N,00630.3372,W,0.02,31.66,280511,,,A*43\r");
	//TraceNL(temp_gps_data);
	char* gprmc_start = strstr((char*) temp_gps_data, "$GPRMC");
	char* gpgga_start = strstr((char*) temp_gps_data, "$GPGGA");
	char* gpgsa_start = strstr((char*) temp_gps_data, "$GPGSA");

	if (gprmc_start == NULL) {
		TraceNL("No GPRMC(gprmc_start == NUL)");
		is_gps_valid = 0;
		return FAIL;
	}
	UTIL_GetPartOfString(gprmc_start, gprmc, '$', '\r', 1, 0);
	UTIL_GetPartOfString(gpgga_start, gpgga, '$', '\r', 1, 0);
	UTIL_GetPartOfString(gpgsa_start, gpgsa, '$', '\r', 1, 0);
	TraceNL(gprmc);
	TraceNL(gpgga);
	TraceNL(gpgsa);
	if (strlen(gprmc) < 50) {
		TraceNL("Short gprmc returning.");
		is_gps_valid = 0;
		return FAIL;
	}
	strcpy(temp_gprmc, gprmc);
	ParseFields(temp_gprmc, tempPFields, 12, ",");
	/*
	ParseFields(gpgsa, GSAFields, 18, ",");
	ParseFields(gpgga, GGAFields, 15, ",");
	Trace("025:");
	Trace("GSAAAAAAAAAAAAAAAAAAAAAAAAAAA : ");
	int j = 0;
	for (j=0; j< 10; j++) {
		Trace(GSAFields[j]);
		Trace(",");
	}
	Trace("GGAAAAAAAAAAAAAAAAAAAAAAAAAAA : ");
	for (j=0; j< 10; j++) {
		Trace(GGAFields[j]);
		Trace(",");
	}
	*/
	/*
	Trace(GSAFields[2]);
	Trace(",");
	Trace(GSAFields[15]);
	Trace(",");
	Trace(GSAFields[16]);
	Trace(",");
	Trace(GSAFields[17]);
	Trace(",");
	Trace(GGAFields[7]);
	Trace("*");
	*/
	//ParseFields(testGPRMC,pFields,12,",");
	TraceNL("ParseFields OK..");
	if (strcmp(tempPFields[2], "V") == 0) {
		TraceNL("INVALID GPS, SKIPPING.");
		is_gps_valid = 0;
		return FAIL;
	} else if (strcmp(tempPFields[2], "A") == 0) {
		TraceNL("VALID GPS");
		is_gps_valid = 1;
		pFields = tempPFields;
		prev_lat = last_lat;
		prev_lon = last_lon;
		last_lat = DDMMSSToDecimalDegrees(atof(pFields[3]));
		last_lon = DDMMSSToDecimalDegrees(atof(pFields[5]));
		if (strcmp(pFields[4], "S") == 0)
			last_lat = last_lat * -1;
		if (strcmp(pFields[6], "W") == 0)
			last_lon = last_lon * -1;
		gpsSpeed = (atof(pFields[7]) * 1852)/ 1000;
		if (gpsSpeed > 250)
			gpsSpeed = 0;
		if (gpsSpeed < 8)
			gpsSpeed = 0;
		gpsHeading = atof(pFields[8]);
		char buffer[100];
		int count = sprintf(buffer,
				"Speed : %s = %.10f Heading : %s = %.10f\r", pFields[7],
				gpsSpeed, pFields[8], gpsHeading);
		UARTSend(PORT_TRACE, (unsigned char*) buffer, count);
		strcpy(last_valid_gprmc, gprmc);
		return SUCCESS;
	}
	is_gps_valid = 0;
	return FAIL;
}


double GPS_CalculateDistance() {
	char buffer[200];
	double local_prev_lat = 0;
	double local_last_lat = 0;
	int count = sprintf(buffer,"Last Lat : %.10f, Last Lon= %.10f,Prev Lat : %.10f, Prev Lon= %.10f\r\r", last_lat,last_lon,prev_lat,prev_lon);
	UARTSend(PORT_TRACE, buffer, count);
	if (is_gps_valid == 0 || last_lat == 0 || last_lon == 0 || prev_lat == 0 || prev_lon == 0){
		TraceNL("GPS_CalculateDistance returns..");
		return 0;
	}
	/////// Convert ddMM to degrees
	double R = 6371000; // m
	double dLat = (last_lat-prev_lat) * 0.0174532925; // To Radian
	double dLon = (last_lon-prev_lon) * 0.0174532925; //To Radian
	local_prev_lat = prev_lat * 0.0174532925;
	local_last_lat = last_lat * 0.0174532925;
	double a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(local_last_lat) * cos(local_last_lat);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	double d = R * c;
	if (d > 50000) {
		TraceNL("GPS_CalculateDistance greater than 50000, returning 0.");
		return 0;
	}
	TraceNL("*****************");
	TraceNL("*******");
	TraceNL("**");
	TraceNL("*");
	count = sprintf(buffer,"GPS Distance= %.10f\r", d);
	UARTSend(PORT_TRACE, buffer, count);
	TraceNL("*****************");
	TraceNL("*******");
	TraceNL("**");
	TraceNL("*");
	/*if (d < 1)
		d=0;*///Active in production
	return d;
}

 double DDMMSSToDecimalDegrees(double ddMMss) {
	char buffer[100];
	ddMMss = ddMMss / 100;
	int degrees = (int)ddMMss;
	double minutesseconds = ( (ddMMss - degrees) * 100 ) / 60.0;
	double result = (double)degrees + minutesseconds;
	int count = sprintf(buffer,"ddMMss= %.10f\r,degrees= %.10f\r", ddMMss,result);
	UARTSend(PORT_TRACE, buffer, count);
	return result;
}

void GPS_ToggleGPSLed() {
	if (gps_led_val == 0)
		GPS_SetGPSLed(1);
	else
		GPS_SetGPSLed(0);
	LPC_TIM0->IR |= 1 << 0;
}

int GPS_ShutDown() {
	LPC_GPIO0->FIOCLR = (1 << 5);
}

int GPS_Open() {
	LPC_GPIO0->FIOSET = (1 << 5);
}

