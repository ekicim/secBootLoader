/*
 * GPS.h
 *
 *  Created on: 24 Ara 2013
 *      Author: trio
 */

#ifndef GPS_H_
#define GPS_H_

int ewns;
int is_gps_valid;
double gpsSpeed, gpsHeading;
int numSats,hdop,vdop,pdop,fixType;
unsigned long int last_gps_time;
int gps_led_val;
char** pFields; //GPS Fields
char* tempPFields[12];
char last_valid_gprmc[200];
void ParseFields(char* inputBuffer, char** Fields, uint32_t numFields,char* delimiterChars);
void GPS_SetGPSLed(int value);
void GSM_GetLBS();
double DDMMSSToDecimalDegrees(double ddMMss);
int GPS_GetGPRMC();
void GPS_ToggleGPSLed();
double GPS_CalculateDistance();
int GPS_ShutDown();
int GPS_Open();
void GPS_CleanBuffer();
double last_lat,last_lon,prev_lat,prev_lon;

#endif /* GPS_H_ */
