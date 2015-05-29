/*
 * Calibration.h
 *
 *  Created on: 24 Ara 2013
 *      Author: trio
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include "type.h"
// Persistant Variables
char per_apn_val[30];
char per_apnuser_val[20];
char per_apnpass_val[20];
char per_ip_val[40];
int per_port_val;
uint64_t per_mileage_val;
//Persistant Variables
int per_ignonperiod_val;
int per_ignoffperiod_val;
int per_ignonroamingperiod_val;
int per_ignoffroamingperiod_val;
int per_lowpow_just_wake;
int per_lowpow_cancel;
int per_lowpow_period;
int per_lowpow_mode;
int per_coffe_counter;

typedef struct
{
	uint16_t Length;
	uint16_t StartPos;
} Calibration;

Calibration test, cal_ip, cal_port,
			cal_apn, cal_apn_user,
			cal_apn_pass, cal_ign_on_period,
			cal_ign_off_period, cal_ign_on_roaming_period,
			cal_ign_off_roaming_period, cal_mileage,
			cal_lowpow_just_wake,cal_lowpow_cancel,cal_lowpow_period,cal_lowpow_mode,cal_coffee_counter;

void LoadParams();
void LoadTimings();
void PersistParams();
uint64_t EEPROM_LoadInt64(Calibration cal);
void EEPROM_LoadString(Calibration cal, char* str);
uint8_t EEPROM_SaveInt64(Calibration cal, uint64_t val);
uint8_t EEPROM_SaveString(Calibration cal, const char* str);
void EEPROM_EraseMemory(Calibration cal);

#endif /* CALIBRATION_H_ */
