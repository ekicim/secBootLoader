 /******************** (C) Trio Mobil 2014 LPC1768 MCU   **********************
 * File Name          : P65 GSM Routines
 * Author			  : Nevzat Ataklı
 * Version            : V1.0.0
 * Date               : 2014/01/04
 ****************************************************************************/


#include "GSM.h"
#include "Utils.h"
#include "Calibration.h"
#include "LPC17xx.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#include "timer.h"

unsigned long int last_hard_reset = -1;
int connectionFailCount = 0;
#define LBS 1

uint16_t GSM_SendAt( char* cmd, char *response, int delay )
{
	char buffer[200];
	uint16_t	len;

	int count = sprintf(buffer, "%s\r", cmd);
	UARTSend(PORT_GSM, buffer, count);

	DelayMs(delay);

	len = ReadUart(response, PORT_GSM);

	sprintf(buffer, "%s Response : %s \r", cmd, response);
	TraceNL(buffer);

    return ( len );
}



int16_t GSM_TCP_Recv( char* pDataBuf, int16_t maxBytes )
{
	Trace("Entered GSM_TCP_Recv");

	return ( GSM_SendAt( "AT+QIRD=1,1,0,1050", pDataBuf, 500 ) );
}

void GSM_CheckBuffer() {
	Trace("Entered CheckGSMBuffer");
	char response[500];
	char msg_from_server[300];
	GSM_SendAt("AT+QIRD=1,1,0,300",response,100);
	int index = 0;
	int i = 0;
	char* p_set = strstr(response, "#SET;");
	if (p_set == NULL)
		return;
	if (p_set != NULL){
		for (i=0; i < strlen(p_set); i++) {
			msg_from_server[index] = p_set[i];
			index++;
			if (p_set[i] == '\r')
				break;
		}
		msg_from_server[index] = '\0';
	}
	if (strlen(msg_from_server) > 5){
		Trace("Message From Server : ");
		TraceNL(msg_from_server);
		ChangeConfiguration(msg_from_server,1);
		//#SET;04;1;30;30;0!
		//#SET -imei-
	}
}

void GSM_ReadSMS(){
	//Read Index 1, Delete All.
	//AT+CMGR=<index>,0
	//AT+CMGD=1,4 //Delete All SMS Messages.
	int lf_count = 0;
	int index = 0;
	int i = 0;
	TraceNL("Entered read sms.");
	char response[1000];
	char msg_from_sms[500];
	GSM_SendAt("AT",response,100);
	GSM_SendAt("AT+CMGR=1", response, 500);
	char* p_set = strstr(response, "#SET;");
	if (p_set != NULL){
		for (i=0; i < strlen(p_set); i++) {
				msg_from_sms[index] = p_set[i];
				index++;
				if (p_set[i] == '\r')
					break;
		}
		msg_from_sms[index] = '\0';
	}
	GSM_SendAt("AT+CMGD=1,4", response, 100);
	// Delete All SMS
	if (strlen(msg_from_sms) > 5){
		Trace("Message From SMS : ");
		TraceNL(msg_from_sms);
		ChangeConfiguration(msg_from_sms);
	}
}

void GSM_SendSMS() {
	/*
	AT+CMGF=1
	OK
	AT+CMGS="+31628870634"
	> This is the text message.→
	+CMGS: 198
	OK*/
}


int GSM_ConnectToTrioUpgradeServer(char *ip, char *port)
{
	char buffer[200];
	char response[150];

	TraceNL("Entered GSM_ConnectToTrioUpgradeServer");

	GSM_SendAt("AT+QIFGCNT=1", response, 500);

	memset(buffer, 0, sizeof(buffer));
	sprintf(buffer, "AT+QICSGP=1,\"%s\",\"%s\",\"%s\"", per_apn_val, per_apnuser_val, per_apnpass_val);

	GSM_SendAt(buffer, response, 100);
	GSM_SendAt("AT+QIMUX=0", response, 100);
	GSM_SendAt("AT+QVBATT=0,3500,0", response, 100); //Disable low power shut down and warning.
	GSM_SendAt("AT+QVBATT=1,3300,0",response,100); // Disable 3.3 volt cutoff.

	//SendAt("AT+QIMODE=1",response); //Transparent Mode
	GSM_SendAt("AT+QIMODE=0", response, 100); //Non Transparent Mode
	GSM_SendAt("AT+QINDI=1", response, 100); //Alert when data received.

	//GSM_SendAt("AT+QITCFG=3,1,512,1", response, 500); //Transparent mode configuration

	memset(buffer, 0, sizeof(buffer));
	if (isalpha(per_ip_val[0])){ //DNS
		GSM_SendAt("AT+QIDNSIP=1", response, 100);
	}else
		GSM_SendAt("AT+QIDNSIP=0", response, 100);

	////Low Power/////
	GSM_SendAt("AT+QGPCLASS=8", response, 100); // 1 Tx timeslots
	//GSM_SendAt("AT+CDETXPW=900,1,255,2", response, 100);
	//////////////////
	GSM_SendAt("AT",response,100);

	int cmd_count = sprintf(buffer, "AT+QIOPEN=\"TCP\",\"%s\",%d\r", per_ip_val, per_port_val);
	//GSM_SendAt("AT+QIOPEN=\"TCP\",\"178.63.30.80\",6081", response, 2000);

	UARTSend(PORT_GSM, buffer, cmd_count);
	UARTSend(PORT_TRACE, buffer, cmd_count);
	int server_conn_count = 0;
	int server_conn_result = FAIL;

	while(1){
		ReadUart(response, PORT_GSM);
		if (strstr(response,"FAIL") != NULL/* || strstr(response,"ERROR") != NULL*/){ //ERROR is about format ignore
			//UARTSend(PORT_GSM, buffer, cmd_count); //Testing AT to server problem
			break;
		}
		else if (strstr(response,"CONNECT OK") != NULL || strstr(response,"ALREADY CONNECT") != NULL){
			UARTSend(PORT_TRACE, response, cmd_count);
			server_conn_result = SUCCESS;
			break;
		}
		WDTFeed();
		DelayMs(300);
		server_conn_count++;
		if (server_conn_count > 30)
			break;
	}

	//GSM_SendAt(buffer, response, 2000);
	if (server_conn_result == SUCCESS) {
		TraceNL("Connected to the server.");
		connectionFailCount = 0;
		return SUCCESS;
	}

	TraceNL("QIOPEN failed.");
	connectionFailCount++;
	if (connectionFailCount > 40 && connectionFailCount % 200 == 0) //10 minutes.
	{
		TraceNL("Hard resetting module.");
		GSM_InitModule(); //Restarts and initializes module.
		last_hard_reset = STT_Value;
	}
	else if (connectionFailCount > 30 && connectionFailCount % 20 == 0){ //100 in production
		TraceNL("connectionFailCount > 40 restarting module.");
		GSM_InitModule(); //Restarts and initializes module.
	}
	return FAIL;
}

/*****************************************************************************
** Function name:	GSM_TCP_Connect
** Description:		Function connects to a server over TCP
**
** Parameters:		none
**
** Returned value:	SUCCESS  connection established
** 					FAIL	 connection failed (error code)
******************************************************************************/
int GSM_ConnectToTrio() {
	TraceNL("Entered ConnectoTrio");
	char buffer[200];
	char response[150];
	//GSM_SendAt("AT+QIFGCNT=0", response, 500);
	GSM_SendAt("AT+QIFGCNT=1", response, 500);
	memset(buffer, 0, sizeof(buffer));
	sprintf(buffer, "AT+QICSGP=1,\"%s\",\"%s\",\"%s\"", per_apn_val,
			per_apnuser_val, per_apnpass_val);
	//GSM_SendAt("AT+QICSGP=1,\"internet\"", response, 500);
	GSM_SendAt(buffer, response, 100);
	GSM_SendAt("AT+QIMUX=0", response, 100);
	GSM_SendAt("AT+QVBATT=0,3500,0", response, 100); //Disable low power shut down and warning.
	GSM_SendAt("AT+QVBATT=1,3300,0",response,100); // Disable 3.3 volt cutoff.
	//SendAt("AT+QIMODE=1",response); //Transparent Mode
	GSM_SendAt("AT+QIMODE=0", response, 100); //Non Transparent Mode
	GSM_SendAt("AT+QINDI=1", response, 100); //Alert when data received.
	//GSM_SendAt("AT+QITCFG=3,1,512,1", response, 500); //Transparent mode configuration
	memset(buffer, 0, sizeof(buffer));
	if (isalpha(per_ip_val[0])){ //DNS
		GSM_SendAt("AT+QIDNSIP=1", response, 100);
	}else
		GSM_SendAt("AT+QIDNSIP=0", response, 100);
	////Low Power/////
	GSM_SendAt("AT+QGPCLASS=8", response, 100); // 1 Tx timeslots
	//GSM_SendAt("AT+CDETXPW=900,1,255,2", response, 100);
	//////////////////
	GSM_SendAt("AT",response,100);
	int cmd_count = sprintf(buffer, "AT+QIOPEN=\"TCP\",\"%s\",%d\r", per_ip_val, per_port_val);
	//GSM_SendAt("AT+QIOPEN=\"TCP\",\"178.63.30.80\",6081", response, 2000);
	UARTSend(PORT_GSM, buffer, cmd_count);
	UARTSend(PORT_TRACE, buffer, cmd_count);
	int server_conn_count = 0;
	int server_conn_result = FAIL;
	while(1){
		ReadUart(response, PORT_GSM);
		if (strstr(response,"FAIL") != NULL/* || strstr(response,"ERROR") != NULL*/){ //ERROR is about format ignore
			//UARTSend(PORT_GSM, buffer, cmd_count); //Testing AT to server problem
			break;
		}
		else if (strstr(response,"CONNECT OK") != NULL || strstr(response,"ALREADY CONNECT") != NULL){
			UARTSend(PORT_TRACE, response, cmd_count);
			server_conn_result = SUCCESS;
			break;
		}
		WDTFeed();
		DelayMs(300);
		server_conn_count++;
		if (server_conn_count > 30)
			break;
	}
	//GSM_SendAt(buffer, response, 2000);
	if (server_conn_result == SUCCESS) {
		TraceNL("Connected to the server.");
		connectionFailCount = 0;
		return SUCCESS;
	}
	TraceNL("QIOPEN failed.");
	connectionFailCount++;
	if (connectionFailCount > 40 && connectionFailCount % 200 == 0) //10 minutes.
	{
		TraceNL("Hard resetting module.");
		GSM_InitModule(); //Restarts and initializes module.
		last_hard_reset = STT_Value;
	}
	else if (connectionFailCount > 30 && connectionFailCount % 20 == 0){ //100 in production
		TraceNL("connectionFailCount > 40 restarting module.");
		GSM_InitModule(); //Restarts and initializes module.
	}
	return FAIL;
}


/*****************************************************************************
** Function name:	GSM_TCP_Send
** Description:		Function sends the given buffer over a already connected
** 					TCP connection. The connection should have been set before
** 					calling this function.
**
** Parameters:		msg		buffer to send
**                  len		size of msg
**
** Returned value:	greater than 0 to indicate number of characters transferred
**
**                  a value less than 0 is error code, tobe defined later.
******************************************************************************/
int GSM_SendToServerTCP(char* msg) {
	TraceNL("Entered GSM_SendToServerTCP");
	char response[500];
	char buffer[500];
	GSM_SendAt("AT", response, 100); //Empty buffer
	int count = sprintf(buffer, "AT+QISEND=%d", strlen(msg));
	GSM_SendAt(buffer, response, 100);
	//GSM_SendAt("AT+QISEND", response, 100);
	if (strchr(response, '>') != NULL) {
		UARTSend(PORT_GSM, msg, strlen(msg));
		DelayMs(300);
		ReadUart(response, PORT_GSM);
		int sendOkCheck = 0;
		while (strstr(response,"SEND OK") == NULL){
			TraceNL("Send OK whiling..");
			if (sendOkCheck > 20)
				return FAIL;
			DelayMs(100);
			ReadUart(response, PORT_GSM);
			sendOkCheck++;
		}
		TraceNL("Data sent.");
		return SUCCESS;
	}
	else if (strstr(response, "ERROR") != NULL) {
		TraceNL("Data fail.");
		return FAIL;
	}
	TraceNL("Data fail.");
	return FAIL;
}

/*****************************************************************************
** Function name:	GSM_TCP_Send
** Description:		Function sends the given buffer over a already connected
** 					TCP connection. The connection should have been set before
** 					calling this function.
**
** Parameters:		msg		buffer to send
**                  len		size of msg
**
** Returned value:	greater than 0 to indicate number of characters transferred
**
**                  a value less than 0 is error code, tobe defined later.
******************************************************************************/
int GSM_TCP_Send( char* msg, uint16_t len )
{
	char response[200];
	char buffer[200];

	TraceNL("Entered GSM_TCP_Send");

	GSM_SendAt( "AT", response, 100 ); //Empty buffer

	int count = sprintf( buffer, "AT+QISEND=%d", len );
	GSM_SendAt( buffer, response, 100 );

	if( strchr(response, '>') != NULL )
	{
		UARTSend( PORT_GSM, msg, len );

		DelayMs( 300 );

		ReadUart( response, PORT_GSM );
		int sendOkCheck = 0;

		while( strstr( response, "SEND OK" ) == NULL )
		{
			TraceNL( "Send OK whiling.." );
			if ( sendOkCheck > 20 )
				return ( FAIL );

			DelayMs( 100 );
			ReadUart( response, PORT_GSM );

			sendOkCheck++;
		}
		TraceNL( "Data sent." );
		return ( len );
	}
	TraceNL( "Data fail." );
	return ( FAIL );
}

void GSM_TogglePwrKey() {
	//Toggle PWR_KEY
	LPC_GPIO2->FIOSET = (1 << 3);
	DelayMs(2000);
	LPC_GPIO2->FIOCLR = (1 << 3);
}

int GSM_ShutdownModule() {
	char response[100];
	GSM_SendAt("AT+QPOWD=0", response, 500);
	DelayMs(1000);
	memset(response, 0, sizeof(response));
	GSM_SendAt("AT", response, 500);
	if (strstr(response, "OK") == NULL) {
		return ( SUCCESS );
	}
	return ( FAIL );
}

int GSM_EmergShutdownModule(){
	LPC_GPIO2->FIOSET = (1 << 2);
	DelayMs(2000);
	LPC_GPIO2->FIOCLR = (1 << 2);

	return ( SUCCESS );
}

int GSM_InitModule() {
	//Unset EMERG
	char response[100];
	LPC_GPIO2->FIOCLR = (1 << 2);
	DelayMs(1000);
	GSM_TogglePwrKey();
	int i = 0;
	for (i = 0; i < 10; i++) {
		GSM_SendAt("AT", response, 500);
		if (strstr(response, "OK") != NULL) {
			GSM_SendAt("AT+CMEE=2", response, 500);
			DelayMs(500);
			//Init SMS
			//SMS Text Mode
			GSM_SendAt("AT+CMGF=1", response, 500);
			//Initialize SMS, if not new messages does not cause +CMTI
			GSM_SendAt("AT+CNMI=2,1,0,0,0", response, 500);
			int sim_check = FAIL;
			for(;i<40;i++){
				sim_check = GSM_CheckSimCard();
				if (sim_check == SUCCESS){
					TraceNL("Sim card ready..\r");
					break;
				}
				else
					TraceNL("Sim not ready!\r");
				DelayMs(200);
				WDTFeed();
			}
			return sim_check;
		}
	}
	return FAIL;
}

int GSM_GetSignalStrength() {
	char csq_str[20];
	char response[100];
	GSM_SendAt("AT", response, 100);
	GSM_SendAt("AT+CSQ", response, 100);
	UTIL_GetPartOfString(response, csq_str, ':', ',', 0, 0);
	return atoi(csq_str);
}


int GSM_GetRegStat(){
	char csq_str[20];
	char response[100];
	GSM_SendAt("AT+CREG?", response, 100);
	if (strstr(response, "+CREG") != NULL && ((strstr(response, ",5") != NULL) || (strstr(response, ",1") != NULL))) {
		return SUCCESS;
	}
	return FAIL;
}

void GSM_GetCellInfo(char *cell_str) {
	char response[100];
	GSM_SendAt("AT", response, 100);
	GSM_SendAt("AT+CREG?", response, 100);
	int comma_count = 0;
	int index = 0;
	int i = 0;
	for (; i < strlen(response); i++) {
		if (comma_count >= 2) {
			if (response[i] == '\r' || response[i] == '\n')
				break;
			if (response[i] != '\"'){
				cell_str[index] = response[i];
				index++;
			}
		}
		if (response[i] == ',')
			comma_count = comma_count + 1;
	}
	cell_str[index] = '\0';
}

int GSM_IsRoaming() {
	char csq_str[20];
	char response[100];
	GSM_SendAt("AT+CREG?", response, 100);
	if (strstr(response, "+CREG") != NULL && strstr(response, ",5,") != NULL) {
		return SUCCESS;
	}
	return FAIL;
}



int GSM_CheckSimCard() {
	TraceNL("Entered check sim card.");
	char response[100];
	GSM_SendAt("AT+CPIN?", response, 250);
	if (strstr(response, "READY") != NULL)
		return ( SUCCESS );
	return ( FAIL );
}

void GSM_GetImei() {
	char response[100];
	GSM_SendAt("AT+GSN", response, 500);
	TraceNL("IMEI : ");
	strncpy(imei, response + 9, 15);
	TraceNL(imei);
}

void GSM_GetImsi() {
	char response[100];
	GSM_SendAt("AT+CIMI", response, 500);
	Trace("IMSI : ");
	int i;
	int index = 0;
	int is_start = 0;
	for (i = 0; i < strlen(response); i++) {
		if (index > 19)
			break;
		if ( isdigit(response[i]))
			is_start = 1;
		if (is_start == 1 && response[i] == '\n')
			break;
		if (is_start == 1) {
			imsi[index] = response[i];
			index++;
		}
	}
	imsi[index - 1] = '\0';
	TraceNL(imsi);
}