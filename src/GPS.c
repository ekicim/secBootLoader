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


void GPS_SetGPSLed(int value) {
	if (value == 0) {
		LPC_GPIO4->FIOCLR = (1 << 29);
		gps_led_val = 0;
	} else {
		LPC_GPIO4->FIOSET = (1 << 29);
		gps_led_val = 1;
	}
}
void GPS_ToggleGPSLed() {
	if (gps_led_val == 0)
		GPS_SetGPSLed(1);
	else
		GPS_SetGPSLed(0);
	LPC_TIM0->IR |= 1 << 0;
}


