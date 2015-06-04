/*
 * Utils.h
 *
 *  Created on: 02 Oca 2014
 *      Author: trio
 */

#include <bsp.h>
#include "type.h"

uint8_t device_power_state;

#ifndef UTILS_H_
#define UTILS_H_
#define PORT_GSM		1
#define Baudrate		9600
//#define Baudrate  57600  //  baudrate 115.2 Kbps
#define PORT_TRACE  	0
#define PORT_GPS 		2

#define low_power_state 0
#define high_power_state 1

void UTIL_appendZerosAndAddAppendix(char *source, char* appendix, int len);
int UTIL_isNumeric(const char * s);
void UTIL_GetPartOfString(char*source, char*dest, char start_chr, char end_chr,int first_included, int last_included);

void Trace(char* msg);
void UTIL_getFromSplitStr(char* delim, int index, char* source, char* result);
float median(int size, float x[]);
#endif /* UTILS_H_ */
