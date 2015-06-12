 /******************** (C) Trio Mobil 2014 LPC1768 MCU   **********************
 * File Name          : P65 GSM Routines
 * Author			  : Nevzat Ataklı
 * Version            : V1.0.0
 * Date               : 2014/01/04
 ****************************************************************************/


#include "GSM.h"
#include "uart.h"

#include "Calibration.h"
#include "LPC17xx.h"

#include "trace.h"
#include "wdt.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#include "timer.h"

static unsigned long int last_hard_reset = -1;
static int connectionFailCount = 0;


uint16_t GSM_SendAt( char* cmd, char *response, int delay )
{
	char buffer[100];
	uint16_t	len;

	sprintf( buffer, "---> %s\r\n", cmd );
	Trace( buffer );

	int count = sprintf( buffer, "%s\r\n", cmd );
	UARTSend( PORT_GSM, buffer, count );

	DelayMs( delay );

	len = ReadUart( response, PORT_GSM );
	response[len] = '\0';

    return ( len );
}

int16_t GSM_TCP_Recv( char* pDataBuf, int16_t maxBytes )
{
	return ( GSM_SendAt( "AT+QIRD=1,1,0,1100", pDataBuf, 1000 ) );
}

int GSM_ConnectToTrioUpgradeServer( void )
{
	char buffer[200];
	char response[150];

	TraceNL("Entered GSM_ConnectToTrioUpgradeServer");

	int init_result = GSM_InitModule();
	if (init_result == FAIL){
		GSM_ShutdownModule();
		DelayMs(2000);
		init_result = GSM_InitModule();
	}

	WDTFeed();

	if( init_result == FAIL )
		return ( init_result );

	//GSM_EchoOFF();
	GSM_GetImei();
	GSM_GetImsi();
	GSM_GetRegStat();

	GSM_SendAt("ATI", response, 500);
	WDTFeed();
	GSM_SendAt("AT+QIFGCNT=1", response, 500);

	memset(buffer, 0, sizeof(buffer));
	sprintf(buffer, "AT+QICSGP=1,\"%s\",\"%s\",\"%s\"", per_apn_val, per_apnuser_val, per_apnpass_val);

	GSM_SendAt(buffer, response, 100);
	GSM_SendAt("AT+QIMUX=0", response, 100);
	GSM_SendAt("AT+QVBATT=0,3500,0", response, 100); //Disable low power shut down and warning.
	GSM_SendAt("AT+QVBATT=1,3300,0",response,100); // Disable 3.3 volt cutoff.

	GSM_SendAt("AT+QISHOWRA=1", response, 100); // show remote address
	GSM_SendAt("AT+QISHOWPT=1", response, 100); // show protocol type ,TCP, or UDP
	GSM_SendAt("AT+QINDI=0", response, 100);    //  disable indicator
	GSM_SendAt("AT+QIHEAD=1", response, 100);    //

	GSM_SendAt("AT+QIMODE=0", response, 100);   //Non Transparent Mode

	memset(buffer, 0, sizeof(buffer));
	if (isalpha(per_ip_val[0])){ //DNS
		GSM_SendAt("AT+QIDNSIP=1", response, 100);
		TraceDumpHex( response, strlen(response) );
	}else
		GSM_SendAt("AT+QIDNSIP=0", response, 100);

	GSM_SendAt("AT+QGPCLASS=12", response, 100);
	GSM_SendAt("AT",response,100);

	sprintf( buffer, "AT+QIOPEN=\"TCP\",\"%s\",%s\r\n", update_service_ip, update_service_port );

	GSM_SendAt( buffer, response, 100);

	int server_conn_count = 0;
	int server_conn_result = FAIL;

	while(1){
		int recLen = ReadUart(response, PORT_GSM);
		if( recLen )
		{
			if( strstr(response,"FAIL") != NULL )
			{ //ERROR is about format ignore
				break;
			}
			else if( strstr(response,"CONNECT OK") != NULL || strstr(response,"ALREADY CONNECT") != NULL )
			{
				TraceDumpHex( response, recLen );
				server_conn_result = SUCCESS;
				break;
			}
		}

		WDTFeed();
		DelayMs(300);
		server_conn_count++;
		if (server_conn_count > 30)
			break;
	}

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
int GSM_SendToServerTCP(char* msg)
{
	char response[500];
	char buffer[500];

	TraceNL("Entered GSM_SendToServerTCP");

	GSM_SendAt("AT", response, 100); //Empty buffer

	sprintf(buffer, "AT+QISEND=%d", strlen(msg));
	GSM_SendAt(buffer, response, 100);

	if (strchr(response, '>') != NULL)
	{
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
	char buffer[100];

	TraceNL("Entered GSM_TCP_Send");

	GSM_SendAt( "AT", response, 100 ); //Empty buffer

	sprintf( buffer, "AT+QISEND=%d", len );
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
	for (i = 0; i < 20; i++) {
		GSM_SendAt("AT", response, 500);

//		TraceDumpHex( response, strlen(response) );
		if (strstr(response, "OK") != NULL) {

//			GSM_Set1152008N1( );
//
//			UARTInit(PORT_GSM, 115200);
			GSM_SendAt("AT+CMEE=2", response, 500);
//			TraceDumpHex( response, strlen(response) );
			DelayMs(500);
			//Init SMS
			//SMS Text Mode
			GSM_SendAt("AT+CMGF=1", response, 500);
//			TraceDumpHex( response, strlen(response) );
			//Initialize SMS, if not new messages does not cause +CMTI
			GSM_SendAt("AT+CNMI=2,1,0,0,0", response, 500);
//			TraceDumpHex( response, strlen(response) );
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




int GSM_GetRegStat(){

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


void GSM_EchoOn( )
{
	char response[100];
	GSM_SendAt("ATE1", response, 500);
	TraceNL("ECHO ON");
}


void GSM_EchoOFF( )
{
	char response[100];
	GSM_SendAt("ATE0", response, 500);
	TraceNL("ECHO OFF");
}

void GSM_TCP_Close( void )
{
	char response[200];
	GSM_SendAt("AT+QICLOSE", response, 500);
	TraceNL("CLOSE TCP connection ");

}
