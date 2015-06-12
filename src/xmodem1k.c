/*****************************************************************************
 * $Id$
 *
 * Project: 	NXP LPC1100 Secondary Bootloader Example
 *
 * Description: Implements an Xmodem1K client (receiver).
 *
 * Copyright(C) 2010, NXP Semiconductor
 * All rights reserved.
 *
 *****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 *****************************************************************************/
#include <bsp.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <LPC17xx.h>
#include "crc.h"
#include "uart.h"
#include "xmodem1k.h"
#include "gsm.h"
#include "iap_config.h"
#include "iap.h"
#include "wdt.h"
#include "trace.h"
#include "timer.h"

/* Protocol control ASCII characters */
#define SOH							0x01
#define STX							0x02
//#define EOT							0x03
#define EOT							0x04
#define ACK							0x06
#define NAK							0x15
#define POLL						0x43



/* Internal state machine */
#define STATE_IDLE					0
#define STATE_CONNECTING			1
#define STATE_RECEIVING				2

/* Define the rate at which the server will be polled when starting a transfer */
#define POLL_PERIOD_ms				20000

/* Define packet timeout period (maximum time to receive a packet) */
#define PACKET_TIMEOUT_PERIOD_ms	20000

/* Baud rate to be used by UART interface */
#define UART_NUM					0
#define BAUD_RATE					115200

/* Size of packet payloads and header */
#define LONG_PACKET_PAYLOAD_LEN		1024
//#define LONG_PACKET_PAYLOAD_LEN		64
#define SHORT_PACKET_PAYLOAD_LEN	1024
#define PACKET_HEADER_LEN			3

#define RECEIVE_BUFF_LEN			1100


/* Buffer in which received data is stored, must be aligned on a word boundary
 as point to this array is going to be passed to IAP routines (which require
 word alignment). */
static uint8_t au8RxBuffer[LONG_PACKET_PAYLOAD_LEN] __attribute__ ((aligned(4)));

/* Local functions */
static void vTimerStart( uint32_t u32Periodms );
uint8_t XModemReadByte( char* pByte );

static char receiveBuf[RECEIVE_BUFF_LEN];
static uint16_t dataLen = 0;		// number of bytes in the buffer
static uint16_t dataIndex = 0;  // current byte to be processed

uint8_t XModemReadByte( char* pByte )
{
	char* pdata;
	char* pnewline;
	char asciiLen[10];
	unsigned int len, i;
	unsigned dataoffset;

	if (dataIndex >= dataLen) {
		// all buffer consumed read more from TCP connection
		dataLen = GSM_TCP_Recv( receiveBuf, RECEIVE_BUFF_LEN );
		receiveBuf[dataLen] = '\0';

		// find the length of the message between
		// ",TCP," and "\r\n"
		pdata = strstr( receiveBuf, "IPD" );
		if (pdata == NULL)
		{
			pdata = strstr( receiveBuf, ",TCP," );
			if (pdata == NULL) {
				return (0);
			}

			pnewline = strstr(pdata, "\r\n");
			dataoffset = 2;
			if (pnewline == NULL) {
				return (0);
			}

		} else
		{
			pnewline = strstr( pdata, "TCP:" );
			if (pnewline == NULL) {
				return (0);
			}
			dataoffset = 4;
		}

		strncpy( asciiLen, pdata + 3, pnewline - pdata );

		asciiLen[pnewline - pdata] = '\0';

		len = atoi(asciiLen);

		for (i = 0; i < len; i++) {
			receiveBuf[i] = *(pnewline + dataoffset + i);
		}
		dataLen   = len;
		dataIndex = 0;
		if( dataLen <= 0 ) {
			dataLen = 0;
			return (0);  // no data available
		}
	}

	(*pByte) = receiveBuf[dataIndex++];
	return (1);
}


int XModem1K_Client(
		uint32_t (*pu32Xmodem1kRxPacketCallback)(uint8_t *pu8Data,
				uint16_t u16Len)) {
	uint32_t u32InProgress = 1;
	uint32_t u32State = STATE_IDLE;
	uint32_t u32ByteCount;
	uint32_t u32PktLen;
	uint16_t u16CRC, calculatedCRC;
	uint16_t frameNum = 0;

	char buffer[200];

	uint32_t trials;

	/* Prepare UART for RX/TX */

	while (u32InProgress) {
		WDTFeed();
		switch (u32State) {
		case STATE_IDLE: {

			/* Send command to server indicating we are ready to receive */
			char u8Cmd = POLL;

			TraceNL( "Start downloading" );
			TracePutcHex( u8Cmd );
			GSM_TCP_Send(&u8Cmd, 1);

			/* Start timeout to send another poll if we do not get a response */
			vTimerStart(POLL_PERIOD_ms);
			trials   = DOWNLOAD_MAX_TRIALS;

			u32State = STATE_CONNECTING;
		}
			break;

		case STATE_CONNECTING: {
			char u8Data;

			/* Check if a character has been received on the UART */
			if( XModemReadByte( &u8Data ) )
			{
				/* Expecting a start of packet character */
				if( u8Data == SOH )
				{
					/* SOH indicates short pay load packet is being transmitted */
					u32PktLen = SHORT_PACKET_PAYLOAD_LEN;
					u32ByteCount = 1;

					/* Start packet timeout */
					vTimerStart( PACKET_TIMEOUT_PERIOD_ms);

					/* Wait for a further characters */
					u32State = STATE_RECEIVING;
				}
			} else /* No data received yet, check poll command timeout */
			{
				if ((LPC_TIM0->TCR & 0x01) == 0) {
					/* Timeout expired following poll command transmission so try again.. */
					char u8Cmd = POLL;

					if (trials-- > 0) {
						TraceNL("Connecting state timer expired sending  new request  ");
						TracePutcHex( u8Cmd );
						TraceNL("\r\n");
						GSM_TCP_Send(&u8Cmd, 1);
					} else {
						TraceNL("No reply give up upgrading");
						return ( DOWNLOAD_ERR_TIMEOUT );
					}

					/*
					 * Restart timeout to send another poll
					 * if we do not get a response
					 */
					vTimerStart( POLL_PERIOD_ms);
				}
			}
		}
			break;

		case STATE_RECEIVING: {
			char u8Data;

			/* Check if a character has been received on the UART */
			if( XModemReadByte( &u8Data ) )
			{
				// TracePutcHex( u8Data );
				/* Position of received byte determines action we take */
				if( u32ByteCount == 0 )
				{
					/* Expecting a start of packet character */
					if( u8Data == SOH ) {
						/* SOH indicates short pay load packet is being transmitted */
						u32PktLen = SHORT_PACKET_PAYLOAD_LEN;
						u32ByteCount = 1;

						/* Start packet timeout */
						vTimerStart(PACKET_TIMEOUT_PERIOD_ms);
					} else if (u8Data == EOT) {
						TraceNL("Received EOT ");
						/* Server indicating transmission is complete */
						TraceNL("Closing TCP connection ");
						GSM_TCP_Close( );

						u32InProgress = 0;

						sprintf( buffer, "byte count: %d, packet len %d\r\n", u32ByteCount, u32PktLen);
						Trace( buffer );

						/* Call the call back function to indicated a complete transmission */
						/* If length == 0, then EOT */
						pu32Xmodem1kRxPacketCallback( (uint8_t *)SECONDARY_IMAGE_LOAD_ADDR, 0);

						uint32_t imageSize;
						uint32_t imageCRC;

						XModemReadByte( &u8Data );
						imageSize = u8Data << 24 & 0xFF000000;

						XModemReadByte( &u8Data );
						imageSize |= u8Data << 16 & 0x00FF0000;

						XModemReadByte( &u8Data );
						imageSize |= u8Data << 8 & 0x0000FF00;

						XModemReadByte( &u8Data );
						imageSize |= u8Data << 0 & 0x000000FF;

						// CRC
						XModemReadByte( &u8Data );
						imageCRC = u8Data << 24 & 0xFF000000;

						XModemReadByte( &u8Data );
						imageCRC |= u8Data << 16 & 0x00FF0000;

						XModemReadByte( &u8Data );
						imageCRC |= u8Data << 8 & 0x0000FF00;

						XModemReadByte( &u8Data );
						imageCRC |= u8Data << 0 & 0x000000FF;

						sprintf( buffer, "file size: 0x%X, CRC: 0x%X\r\n", imageSize, imageCRC );
						Trace( buffer );

						WriteImageSignature( imageSize, imageCRC );

						return ( 0 );

					} else {
						// Unexpected char ignore it
					}
				} else if (u32ByteCount == 1) {
					/* Byte 1 is the packet number - should be different from last one we received */
					frameNum = u8Data;
					u32ByteCount++;
				} else if (u32ByteCount == 2) {
					/* Byte 2 is the packet number inverted - check for error with last byte */
					frameNum <<= 8;
					frameNum |= u8Data;
					sprintf( buffer, "frame number :  %d\r\n", frameNum );
				    TraceNL( buffer );

					u32ByteCount++;
				} else if( ((u32ByteCount == (SHORT_PACKET_PAYLOAD_LEN+3)) &&
						    (u32PktLen == SHORT_PACKET_PAYLOAD_LEN)) )
				{
					/* If pay load is short byte 131 is the MS byte of the packet CRC, if pay load
					 is long byte 1027 is the MS byte of the packet CRC. */
					u16CRC = u8Data;
					u32ByteCount++;

				}
				else if( (u32ByteCount == (SHORT_PACKET_PAYLOAD_LEN+4)) &&
						 (u32PktLen == SHORT_PACKET_PAYLOAD_LEN) )
				{
					/* If pay load is short byte 132 is the LS byte of the packet CRC, if pay load
					 is long byte 1028 is the LS byte of the packet CRC. */
					u16CRC <<= 8;
					u16CRC |= u8Data;

					calculatedCRC = u16CRC_Calc16( &au8RxBuffer[0], u32PktLen );
					sprintf( buffer, "calculated CRC : 0x%X\r\n", calculatedCRC );
				    TraceNL( buffer );

					/* Check the received CRC against the CRC we generate on the packet data */
					if( calculatedCRC == u16CRC )
					{
						char u8Cmd;

						u8Cmd = ACK;
						GSM_TCP_Send( &u8Cmd, 1 );

						WDTFeed();
						// write to flash
						pu32Xmodem1kRxPacketCallback( &au8RxBuffer[0], u32PktLen );

						TraceNL("Received a frame ");
						TraceNL("Sending  ACK ");
						TraceNL("CRC matches \r\n");
						DelayMs( 100 );

					} else /* Error CRC calculated does not match that received */
					{
						/* Indicate problem to server - should result in packet being resent.. */
						char u8Cmd = NAK;
						TraceNL("CRC does not match  NAK ing");
						GSM_TCP_Send(&u8Cmd, 1);
					}
					u32ByteCount = 0;

				} else {
					/* Must be pay load data so store */
					au8RxBuffer[u32ByteCount - PACKET_HEADER_LEN] = u8Data;
					u32ByteCount++;
				}
			}
			else
			{
				if( (LPC_TIM0->TCR & 0x01) == 0 )
				{
					/* Timeout expired no data received stop upgrade  */
					if (trials-- > 0)
					{
						TraceNL("Time out in RECEIVING   ");
						char u8Cmd = NAK;
						GSM_TCP_Send(&u8Cmd, 1);

						u32ByteCount = 0;
						vTimerStart( POLL_PERIOD_ms);
					} else {
						// no more trying giving up upgrading
						TraceNL("Download process failed giving up upgrading");
						return ( DOWNLOAD_ERR_TIMEOUT );
					}
				}
			}
		}
			break;

		default:
			break;
		}
	}
	return ( DOWNLOAD_ERR_TIMEOUT );
}

/*****************************************************************************
 ** Function name:
 **
 ** Descriptions:
 **
 ** Parameters:	     None
 **
 ** Returned value:  None
 **
 *****************************************************************************/
static void vTimerStart(uint32_t u32Periodms) {
	/* Enable the timer clock */
	LPC_SC->PCONP |= (1UL << 1);

	/* Configure the timer so that we can poll for a match */
	LPC_TIM0->TCR = 0x02; /* reset timer */
	LPC_TIM0->PR = 0x00; /* set prescaler to zero */
	LPC_TIM0->MR0 = u32Periodms
			* ((SystemCoreClock / 4 / (LPC_TIM0->PR + 1)) / 1000UL);
	LPC_TIM0->IR = 0xFF; /* reset all interrupts */
	LPC_TIM0->MCR = 0x04; /* stop timer on match */
	LPC_TIM0->TCR = 0x01; /* start timer */
}

/*****************************************************************************
 **                            End Of File
 *****************************************************************************/
