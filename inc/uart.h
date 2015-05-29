/***********************************************************************
 * $Id::                                                               $
 *
 * Project:	uart: Simple UART echo for LPCXpresso 1700
 * File:	uarttest.c
 * Description:
 * 			LPCXpresso Baseboard uses pins mapped to UART3 for
 * 			its USB-to-UART bridge. This application simply echos
 * 			all characters received.
 *
 ***********************************************************************
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
 **********************************************************************/
#include "type.h"
/*****************************************************************************
 *   History
 *   2010.07.01  ver 1.01    Added support for UART3, tested on LPCXpresso 1700
 *   2009.05.27  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#ifndef __UART_H 
#define __UART_H

#define IER_RBR		0x01
#define IER_THRE	0x02
#define IER_RLS		0x04

#define IIR_PEND	0x01
#define IIR_RLS		0x03
#define IIR_RDA		0x02
#define IIR_CTI		0x06
#define IIR_THRE	0x01

#define LSR_RDR		0x01
#define LSR_OE		0x02
#define LSR_PE		0x04
#define LSR_FE		0x08
#define LSR_BI		0x10
#define LSR_THRE	0x20
#define LSR_TEMT	0x40
#define LSR_RXFE	0x80
#define BUFSIZE		0xFF		   // 128 byte  Max. 256
#define UART2_BUFFSIZE 0x7F8//0x3FC

uint32_t UARTInit( uint32_t portNum, uint32_t Baudrate );
void UART2_Init(int baudrate);
void UART0_IRQHandler( void );
void UART1_IRQHandler( void );
void UART3_IRQHandler( void );
void UARTSend( uint32_t portNum, uint8_t *BufferPtr, uint32_t Length );
uint8_t Usart_Oku(uint8_t *veri, uint8_t PortNum);
uint8_t ReadGPSByPolling(uint8_t *veri, int timeout);
uint16_t ReadUart(uint8_t *veri, uint8_t PortNum);

#endif /* end __UART_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
