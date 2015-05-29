/*
 * GSM.h
 *
 *  Created on: 24 Ara 2013
 *      Author: trio
 */

#include <type.h>

#ifndef GSM_H_
#define GSM_H_
char imei[20];
char imsi[20];
char lbs_str[30];
uint16_t GSM_SendAt( char* cmd, char *response, int delay );
void GSM_CheckBuffer();
void GSM_ReadSMS();
int GSM_ConnectToTrio();
int GSM_SendToServerTCP(char* msg);
void GSM_TogglePwrKey();
int GSM_ShutdownModule();
int GSM_InitModule();
int GSM_GetSignalStrength();
int GSM_IsRoaming();
int GSM_GetBatteryLevel();
int GSM_CheckSimCard();
void GSM_GetImei();
void GSM_GetImsi();
void GSM_GetCellInfo(char *cell_str);
void GSM_GetLBS();
int GSM_GetRegStat();

#endif /* GSM_H_ */
