/*
 * Calibration.c
 *
 *  Created on: 24 Ara 2013
 *      Author: trio
 */

#include "Calibration.h"
#include "exp_i2c.h"
#include "timer.h"

#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include <trace.h>

extern volatile uint32_t I2C1_Count;
extern volatile uint8_t I2C1_MasterBuffer[BUFSIZE];
extern volatile uint8_t I2C1_SlaveBuffer[BUFSIZE];
extern volatile uint32_t I2C1_Cmd, I2C1_MasterState;
extern volatile uint32_t I2C1_ReadLength, I2C1_WriteLength;
uint8_t  j,i;
#define P55 1
//#define OKKA_DEMO 1
//#define CABINET_DEMO 1

void LoadParams() {
	char buffer[200];
	//LOAD IP
	EEPROM_LoadString(cal_ip, per_ip_val);
	if (isalnum(per_ip_val[0]) == 0 || strlen(per_ip_val) < 5 || strchr(per_ip_val, '.') == NULL)
		strcpy(per_ip_val, "178.63.30.81");//Default value
	//LOAD Port
	per_port_val = EEPROM_LoadInt64(cal_port);
	if (per_port_val == 0 || per_port_val == 65535)
		per_port_val = 6081;
	//LOAD APN
	EEPROM_LoadString(cal_apn, per_apn_val);
	if (isalnum(per_apn_val[0]) == 0 || strlen(per_apn_val) < 3)
		strcpy(per_apn_val, "internet");//Default value
	//LOAD APN User
	EEPROM_LoadString(cal_apn_user, per_apnuser_val);
	if (isalnum(per_apnuser_val[0]) == 0 || strlen(per_apnuser_val) < 3)
		strcpy(per_apnuser_val, "vodafone");//Default value
	//LOAD APN Pass
	EEPROM_LoadString(cal_apn_pass, per_apnpass_val);
	if (isalnum(per_apnpass_val[0]) == 0 || strlen(per_apnpass_val) < 3)
		strcpy(per_apnpass_val, "vodafone");//Default value

	//LOAD Mileage
	per_mileage_val = EEPROM_LoadInt64(cal_mileage);
	sprintf(buffer, "Mileage: %d\n", (int)per_mileage_val);
	TraceNL(buffer);
	LoadTimings();

	//MODE 0 = Deep Power Down
	//MODE 1 = Power Down
}

void LoadTimings() {
	char buffer[200];
	//LOAD Ignition On Period
	per_ignonperiod_val = EEPROM_LoadInt64(cal_ign_on_period);
	if (per_ignonperiod_val == 0 || per_ignonperiod_val > 14400)
		per_ignonperiod_val = 60;
	sprintf(buffer, "per_ignonperiod_val: %d\n", (int)per_ignonperiod_val);
	TraceNL(buffer);
	//LOAD Ignition Off Period
	per_ignoffperiod_val = (int)EEPROM_LoadInt64(cal_ign_off_period);
	int original_per_ignoffperiod_val = per_ignoffperiod_val;
	if (per_ignoffperiod_val == 0 || per_ignoffperiod_val > 14400) //4 hours max.
		per_ignoffperiod_val = 120;
	sprintf(buffer, "per_ignoffperiod_val: %d, per_ignoffperiod_val(original): %d\n", (int)per_ignoffperiod_val,(int)original_per_ignoffperiod_val);
	TraceNL(buffer);
	//LOAD Ignition On Roaming Period
	per_ignonroamingperiod_val = (int)EEPROM_LoadInt64(cal_ign_on_roaming_period);
	if (per_ignonroamingperiod_val == 0 || per_ignonroamingperiod_val > 14400)
		per_ignonroamingperiod_val = 1800;
	sprintf(buffer, "per_ignonroamingperiod_val: %d\n", (int)per_ignonroamingperiod_val);
	TraceNL(buffer);
	//LOAD Ignition Off Roaming Period
	per_ignoffroamingperiod_val = (int)EEPROM_LoadInt64(cal_ign_off_roaming_period);
	if (per_ignoffroamingperiod_val == 0 || per_ignoffroamingperiod_val > 14400)
		per_ignoffroamingperiod_val = 7200;
	sprintf(buffer, "per_ignoffroamingperiod_val: %d\n", (int)per_ignoffroamingperiod_val);
	TraceNL(buffer);
	per_lowpow_just_wake = (int)EEPROM_LoadInt64(cal_lowpow_just_wake);
	sprintf(buffer, "per_lowpow_just_wake: %d\n", (int)per_lowpow_just_wake);
	TraceNL(buffer);
	per_lowpow_cancel = (int)EEPROM_LoadInt64(cal_lowpow_cancel);
	if (per_lowpow_cancel > 2)
		per_lowpow_cancel = 0;
	sprintf(buffer, "per_lowpow_cancel: %d\n", (int)per_lowpow_cancel);
	TraceNL(buffer);
	//LOAD Low Power Wake Up Period
	per_lowpow_period = (int)EEPROM_LoadInt64(cal_lowpow_period);
	sprintf(buffer, "per_lowpow_period(original): %d\n", (int)per_lowpow_period);
	TraceNL(buffer);
	if (per_lowpow_period < 1 || per_lowpow_period > 2880){ //No period more than 2 days.
		per_lowpow_period = 59;
		#ifdef OKKA_DEMO
			per_lowpow_period = 15; //59;//1Hour
		#endif
		#ifdef CABINET_DEMO
			per_lowpow_period = 15; //59;//1Hour
		#endif
	}
	sprintf(buffer, "per_lowpow_period: %d\n", (int)per_lowpow_period);
	TraceNL(buffer);
	per_lowpow_mode = (int)EEPROM_LoadInt64(cal_lowpow_mode);
	if (per_lowpow_mode > 2){
		per_lowpow_mode = 0;
		#ifdef CABINET_DEMO
			per_lowpow_mode = 1; //Light sleep
		#endif
	}
	sprintf(buffer, "per_lowpow_mode: %d\n", (int)per_lowpow_mode);
	TraceNL(buffer);
	per_coffe_counter = (int)EEPROM_LoadInt64(cal_coffee_counter);
	sprintf(buffer, "per_coffee_counter: %d\n", (int)per_coffe_counter);
	TraceNL(buffer);
}

void PersistParams() {
	EEPROM_SaveInt64(cal_mileage,per_mileage_val);
}

uint8_t EEPROM_Init() {
	//Initialize EEPROM Memory Map -- 256x8
	test.StartPos = 0;
	test.Length = 2;
	//Calibration test, ip, port, apn, apn_user, apn_pass, ign_on_period, ign_off_period, ign_on_roaming_period, ign_off_roaming_period, mileage;
	cal_port.StartPos = 6;
	cal_port.Length = 2;
	cal_apn.StartPos = 8;
	cal_apn.Length = 25;
	cal_apn_user.StartPos = 33;
	cal_apn_user.Length = 12;
	cal_apn_pass.StartPos = 45;
	cal_apn_pass.Length = 12;
	cal_ign_on_period.StartPos = 57;
	cal_ign_on_period.Length = 2;
	cal_lowpow_period.StartPos = 59;
	cal_lowpow_period.Length = 2;
	cal_ign_on_roaming_period.StartPos = 61;
	cal_ign_on_roaming_period.Length = 2;
	cal_ign_off_roaming_period.StartPos = 63;
	cal_ign_off_roaming_period.Length = 2;
	cal_mileage.StartPos = 65;
	cal_mileage.Length = 8;
	cal_ip.StartPos = 73;
	cal_ip.Length = 25;
	cal_ign_off_period.StartPos = 98;
	cal_ign_off_period.Length = 2;
	cal_lowpow_just_wake.StartPos = 100;
	cal_lowpow_just_wake.Length = 2;
	cal_lowpow_cancel.StartPos = 102;
	cal_lowpow_cancel.Length = 2;
	cal_lowpow_mode.StartPos = 104;
	cal_lowpow_mode.Length = 2;
	cal_coffee_counter.StartPos = 106;
	cal_coffee_counter.Length = 2;
 	return I2CInit(1,((uint32_t)I2CMASTER) == 0);
}

uint64_t EEPROM_LoadInt64(Calibration cal) {
	int index = 0;
	uint64_t result = 0;
	for (index=0; index < cal.Length; index++) {
		if (cal.Length > 2)
			result =  result + ((i2c_read(1,cal.StartPos + index) <<  ((cal.Length - index - 1) * 8)) & 0xFFFFFFFF);
		else
			result =  result + ((i2c_read(1,cal.StartPos + index) <<  ((cal.Length - index - 1) * 8)) & 0xFFFF);
		DelayMs(10);
	}
	return result;
}

//Int format : MSB First
uint8_t EEPROM_SaveInt64(Calibration cal, uint64_t val) {
	int index = 0;
	uint8_t writeResult = 0;
	for (index=0; index < cal.Length; index++) {
		i2c_write(1,cal.StartPos + index, (val >> ((cal.Length - index - 1) * 8)) & 0xFF);
		DelayMs(10);
	}
	return writeResult;
}

uint8_t EEPROM_SaveString(Calibration cal, const char* str){
	EEPROM_EraseMemory(cal);
	int index = 0;
	while (/*str && *str != '\0'*/index < strlen(str) && index < cal.Length) {
		i2c_write(1,cal.StartPos + index, str[index]);
		DelayMs(10);
		index++;
	}
	i2c_write(1,cal.StartPos + index, '\0');
	DelayMs(10);
	return 1;
}

void EEPROM_LoadString(Calibration cal,  char* str){
	int index = 0;
	for (index=0; index < cal.Length; index++) {
		str[index] = i2c_read(1,cal.StartPos + index);
		DelayMs(10);
		if (str[index] == '\0')
			break;
	}
}

void EEPROM_EraseMemory(Calibration cal) {
	int index = 0;
	for (index=0; index < cal.Length; index++) {
		i2c_write(1,cal.StartPos + index, '\0');
		DelayMs(10);
	}
}










