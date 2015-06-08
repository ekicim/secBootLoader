/***********************************************************************
 * $Id::                                                               $
 *
 * Project:	uart: Simple UART echo for LPCXpresso 1700
 * File:	uart.c
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

/*****************************************************************************
 *   History
 *   2010.07.01  ver 1.01    Added support for UART3, tested on LPCXpresso 1700
 *   2009.05.27  ver 1.00    Prelimnary version, first Release
 *
 ******************************************************************************/
#include "lpc17xx.h"
#include "uart.h"
#include "type.h"


volatile uint32_t UART0Status, UART1Status, UART3Status, UART2Status;
volatile uint8_t UART0CountOku, UART1CountOku, UART3CountOku, UART2CountOku;
volatile uint8_t UART0TxEmpty = 1, UART1TxEmpty = 1, UART3TxEmpty = 1,
		UART2TxEmpty = 1;
volatile uint8_t UART0Buffer[BUFSIZE], UART1Buffer[GSM_UART_BUFFER_SIZE],
		UART2Buffer[UART2_BUFFSIZE], UART3Buffer[BUFSIZE];
volatile uint16_t UART0Count = 0, UART1Count = 0, UART2Count = 0, UART3Count = 0;

/*****************************************************************************
 ** Function name:		UART0_IRQHandler
 **
 ** Descriptions:		UART0 interrupt handler
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
void UART0_IRQHandler(void) {
	uint8_t IIRValue, LSRValue;
	uint8_t Dummy = Dummy;

	IIRValue = LPC_UART0->IIR;

	IIRValue >>= 1; /* skip pending bit in IIR */
	IIRValue &= 0x07; /* check bit 1~3, interrupt identification */
	if (IIRValue == IIR_RLS) /* Receive Line Status */
	{
		LSRValue = LPC_UART0->LSR;
		/* Receive Line Status */
		if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI)) {
			/* There are errors or break interrupt */
			/* Read LSR will clear the interrupt */
			UART0Status = LSRValue;
			Dummy = LPC_UART0->RBR; /* Dummy read on RX to clear
			 interrupt, then bail out */
			return;
		}
		if (LSRValue & LSR_RDR) /* Receive Data Ready */
		{
			/* If no error on RLS, normal ready, save into the data buffer. */
			/* Note: read RBR will clear the interrupt */
			UART0Buffer[UART0Count] = LPC_UART0->RBR;
			UART0Count++;
			if (UART0Count == BUFSIZE) {
				UART0Count = 0; /* buffer overflow */
			}
		}
	} else if (IIRValue == IIR_RDA) /* Receive Data Available */
	{
		/* Receive Data Available */
		UART0Buffer[UART0Count] = LPC_UART0->RBR;
		UART0Count++;
		if (UART0Count == BUFSIZE) {
			UART0Count = 0; /* buffer overflow */
		}
	} else if (IIRValue == IIR_CTI) /* Character timeout indicator */
	{
		/* Character Time-out indicator */
		UART0Status |= 0x100; /* Bit 9 as the CTI error */
	} else if (IIRValue == IIR_THRE) /* THRE, transmit holding register empty */
	{
		/* THRE interrupt */
		LSRValue = LPC_UART0->LSR; /* Check status in the LSR to see if
		 valid data in U0THR or not */
		if (LSRValue & LSR_THRE) {
			UART0TxEmpty = 1;
		} else {
			UART0TxEmpty = 0;
		}
	}
}

/*****************************************************************************
 ** Function name:		UART1_IRQHandler
 **
 ** Descriptions:		UART1 interrupt handler
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
void UART1_IRQHandler(void) {
	uint8_t IIRValue, LSRValue;
	uint8_t Dummy = Dummy;

	IIRValue = LPC_UART1->IIR;

	IIRValue >>= 1; /* skip pending bit in IIR */
	IIRValue &= 0x07; /* check bit 1~3, interrupt identification */
	if (IIRValue == IIR_RLS) /* Receive Line Status */
	{
		LSRValue = LPC_UART1->LSR;
		/* Receive Line Status */
		if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI)) {
			/* There are errors or break interrupt */
			/* Read LSR will clear the interrupt */
			UART1Status = LSRValue;
			Dummy = LPC_UART1->RBR; /* Dummy read on RX to clear
			 interrupt, then bail out */
			return;
		}
		if (LSRValue & LSR_RDR) /* Receive Data Ready */
		{
			/* If no error on RLS, normal ready, save into the data buffer. */
			/* Note: read RBR will clear the interrupt */
			UART1Buffer[UART1Count] = LPC_UART1->RBR;
			UART1Count++;
			if (UART1Count == GSM_UART_BUFFER_SIZE) {
				UART1Count = 0; /* buffer overflow */
			}
		}
	} else if (IIRValue == IIR_RDA) /* Receive Data Available */
	{
		/* Receive Data Available */
		UART1Buffer[UART1Count] = LPC_UART1->RBR;
		UART1Count++;
		if (UART1Count == GSM_UART_BUFFER_SIZE) {
			UART1Count = 0; /* buffer overflow */
		}
	} else if (IIRValue == IIR_CTI) /* Character timeout indicator */
	{
		/* Character Time-out indicator */
		UART1Status |= 0x100; /* Bit 9 as the CTI error */
	} else if (IIRValue == IIR_THRE) /* THRE, transmit holding register empty */
	{
		/* THRE interrupt */
		LSRValue = LPC_UART1->LSR; /* Check status in the LSR to see if
		 valid data in U0THR or not */
		if (LSRValue & LSR_THRE) {
			UART1TxEmpty = 1;
		} else {
			UART1TxEmpty = 0;
		}
	}

}

void UART2_IRQHandler(void) {
	uint8_t IIRValue, LSRValue;
	uint8_t Dummy = Dummy;
	IIRValue = LPC_UART2->IIR;
	IIRValue >>= 1; /* skip pending bit in IIR */
	IIRValue &= 0x07; /* check bit 1~3, interrupt identification */
	if (IIRValue == IIR_RLS) /* Receive Line Status */
	{
		LSRValue = LPC_UART2->LSR;
		/* Receive Line Status */
		if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI)) {
			/* There are errors or break interrupt */
			/* Read LSR will clear the interrupt */
			UART2Status = LSRValue;
			Dummy = LPC_UART2->RBR; /* Dummy read on RX to clear
			 interrupt, then bail out */
			return;
		}
		if (LSRValue & LSR_RDR) /* Receive Data Ready */
		{
			/* If no error on RLS, normal ready, save into the data buffer. */
			/* Note: read RBR will clear the interrupt */
			UART2Buffer[UART2Count] = LPC_UART2->RBR;
			UART2Count++;
			if (UART2Count >= UART2_BUFFSIZE) {
				UART2Count = 0; /* buffer overflow */
			}
		}
	} else if (IIRValue == IIR_RDA) /* Receive Data Available */
	{
		/* Receive Data Available */
		UART2Buffer[UART2Count] = LPC_UART2->RBR;
		UART2Count++;
		if (UART2Count >= UART2_BUFFSIZE) {
			UART2Count = 0; /* buffer overflow */
		}
	} else if (IIRValue == IIR_CTI) /* Character timeout indicator */
	{
		/* Character Time-out indicator */
		UART2Status |= 0x100; /* Bit 9 as the CTI error */
	} else if (IIRValue == IIR_THRE) /* THRE, transmit holding register empty */
	{
		/* THRE interrupt */
		LSRValue = LPC_UART2->LSR; /* Check status in the LSR to see if
		 valid data in U0THR or not */
		if (LSRValue & LSR_THRE) {
			UART2TxEmpty = 1;
		} else {
			UART2TxEmpty = 0;
		}
	}
}
/*****************************************************************************
 ** Function name:		UART0_IRQHandler
 **
 ** Descriptions:		UART0 interrupt handler
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
void UART3_IRQHandler(void) {
	uint8_t IIRValue, LSRValue;
	uint8_t Dummy = Dummy;

	IIRValue = LPC_UART3->IIR;

	IIRValue >>= 1; /* skip pending bit in IIR */
	IIRValue &= 0x07; /* check bit 1~3, interrupt identification */
	if (IIRValue == IIR_RLS) /* Receive Line Status */
	{
		LSRValue = LPC_UART3->LSR;
		/* Receive Line Status */
		if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI)) {
			/* There are errors or break interrupt */
			/* Read LSR will clear the interrupt */
			UART3Status = LSRValue;
			Dummy = LPC_UART3->RBR; /* Dummy read on RX to clear
			 interrupt, then bail out */
			return;
		}
		if (LSRValue & LSR_RDR) /* Receive Data Ready */
		{
			/* If no error on RLS, normal ready, save into the data buffer. */
			/* Note: read RBR will clear the interrupt */
			UART3Buffer[UART3Count] = LPC_UART3->RBR;
			UART3Count++;
			if (UART3Count == BUFSIZE) {
				UART3Count = 0; /* buffer overflow */
			}
		}
	} else if (IIRValue == IIR_RDA) /* Receive Data Available */
	{
		/* Receive Data Available */
		UART3Buffer[UART3Count] = LPC_UART3->RBR;
		UART3Count++;
		if (UART3Count == BUFSIZE) {
			UART3Count = 0; /* buffer overflow */
		}
	} else if (IIRValue == IIR_CTI) /* Character timeout indicator */
	{
		/* Character Time-out indicator */
		UART3Status |= 0x100; /* Bit 9 as the CTI error */
	} else if (IIRValue == IIR_THRE) /* THRE, transmit holding register empty */
	{
		/* THRE interrupt */
		LSRValue = LPC_UART3->LSR; /* Check status in the LSR to see if
		 valid data in U0THR or not */
		if (LSRValue & LSR_THRE) {
			UART3TxEmpty = 1;
		} else {
			UART3TxEmpty = 0;
		}
	}
}

/*****************************************************************************
 ** Function name:		UARTInit
 **
 ** Descriptions:		Initialize UART port, setup pin select,
 **						clock, parity, stop bits, FIFO, etc.
 **
 ** parameters:			portNum(0 or 1) and UART baudrate
 ** Returned value:		true or false, return false only if the
 **						interrupt handler can't be installed to the
 **						VIC table
 **
 *****************************************************************************/
uint32_t UARTInit(uint32_t PortNum, uint32_t baudrate) {
	uint32_t Fdiv;
	uint32_t pclkdiv, pclk;
	if (PortNum == 0) {
		LPC_PINCON->PINSEL0 &= ~0x000000F0;
		LPC_PINCON->PINSEL0 |= 0x00000050; /* RxD0 is P0.3 and TxD0 is P0.2 */
		/* By default, the PCLKSELx value is zero, thus, the PCLK for
		 all the peripherals is 1/4 of the SystemFrequency. */
		/* Bit 6~7 is for UART0 */
		pclkdiv = (LPC_SC->PCLKSEL0 >> 6) & 0x03;
		switch (pclkdiv) {
		case 0x00:
		default:
			pclk = SystemCoreClock / 4;
			break;
		case 0x01:
			pclk = SystemCoreClock;
			break;
		case 0x02:
			pclk = SystemCoreClock / 2;
			break;
		case 0x03:
			pclk = SystemCoreClock / 8;
			break;
		}
		LPC_UART0->LCR = 0x83; /* 8 bits, no Parity, 1 Stop bit */
		Fdiv = (pclk / 16) / baudrate; /*baud rate */
		if (baudrate == 115200){
			LPC_UART0->DLM = 0;
			LPC_UART0->DLL = 9;
			LPC_UART0->FDR=1<<0|1<<5;
		}else{
			LPC_UART0->DLM = Fdiv / 256;
			LPC_UART0->DLL = Fdiv % 256;
		}
		LPC_UART0->LCR = 0x03; /* DLAB = 0 */
		LPC_UART0->FCR = 0x07; /* Enable and reset TX and RX FIFO. */
		NVIC_EnableIRQ(UART0_IRQn);
		LPC_UART0->IER = IER_RBR | IER_THRE | IER_RLS; /* Enable UART0 interrupt */
		return (TRUE);
	} else if (PortNum == 1) {
		NVIC_DisableIRQ(UART1_IRQn);
		LPC_SC->PCONP |= 1 << 4;
		LPC_PINCON->PINSEL4 &= ~0x0000000F;
		LPC_PINCON->PINSEL4 |= 0x0000000A; /* Enable RxD1 P0.16, TxD1 P0.15 */

		//| 1<<25; //Enable PCUART1
		/* By default, the PCLKSELx value is zero, thus, the PCLK for
		 all the peripherals is 1/4 of the SystemFrequency. */
		/* Bit 8,9 are for UART1 */
		pclkdiv = (LPC_SC->PCLKSEL0 >> 8) & 0x03;
		switch (pclkdiv) {
		case 0x00:
		default:
			pclk = SystemCoreClock / 4;
			break;
		case 0x01:
			pclk = SystemCoreClock;
			break;
		case 0x02:
			pclk = SystemCoreClock / 2;
			break;
		case 0x03:
			pclk = SystemCoreClock / 8;
			break;
		}
		LPC_UART1->LCR = 0x83; /* 8 bits, no Parity, 1 Stop bit */
		Fdiv = (pclk / 16) / baudrate; /*baud rate */

		if (baudrate == 115200){
			LPC_UART1->DLM = 0;
			LPC_UART1->DLL = 9;
			LPC_UART1->FDR=1<<0|1<<5;
		}else{
			LPC_UART1->DLM = Fdiv / 256;
			LPC_UART1->DLL = Fdiv % 256;
		}
		LPC_UART1->LCR = 0x03; /* DLAB = 0 */
		LPC_UART1->FCR = 0x07; /* Enable and reset TX and RX FIFO. */
		NVIC_EnableIRQ(UART1_IRQn);
		LPC_UART1->IER = IER_RBR | IER_THRE | IER_RLS; /* Enable UART1 interrupt */
		return (TRUE);
	} else if (PortNum == 2) {
		NVIC_DisableIRQ(UART2_IRQn);
		LPC_SC->PCONP |= 1 << 24; // UART 2 PCONP bit 24
		LPC_PINCON->PINSEL0 &= ~0x00F00000;
		LPC_PINCON->PINSEL0 |= 0x00500000; /* Enable TxD2 on 0.10, RxD2 on 0.11 */
		pclkdiv = (LPC_SC->PCLKSEL0 >> 16) & 0x03;
		switch (pclkdiv) {
		case 0x00:
		default:
			pclk = SystemCoreClock / 4;
			break;
		case 0x01:
			pclk = SystemCoreClock;
			break;
		case 0x02:
			pclk = SystemCoreClock / 2;
			break;
		case 0x03:
			pclk = SystemCoreClock / 8;
			break;
		}
		LPC_UART2->LCR = 0x83; /* 8 bits, no Parity, 1 Stop bit */
		Fdiv = (pclk / 16) / baudrate; /*baud rate */
		LPC_UART2->DLM = Fdiv / 256;
		LPC_UART2->DLL = Fdiv % 256;
		LPC_UART2->LCR = 0x03; /* DLAB = 0 */
		LPC_UART2->FCR = 0x07; /* Enable and reset TX and RX FIFO. */
		NVIC_EnableIRQ(UART2_IRQn);
		LPC_UART2->IER = IER_RBR | IER_THRE | IER_RLS; /* Enable UART2 interrupt */
		//LPC_UART2->IER = IER_THRE; /* Polling */
		//LPC_UART2->TER = (1 << 7);
		return (TRUE);
	} else if (PortNum == 3) {
		LPC_PINCON->PINSEL0 &= ~0x0000000F;
		LPC_PINCON->PINSEL0 |= 0x0000000A; /* RxD3 is P0.1 and TxD3 is P0.0 */
		LPC_SC->PCONP |= 1 << 4 | 1 << 25; //Enable PCUART1
		/* By default, the PCLKSELx value is zero, thus, the PCLK for
		 all the peripherals is 1/4 of the SystemFrequency. */
		/* Bit 6~7 is for UART3 */
		pclkdiv = (LPC_SC->PCLKSEL1 >> 18) & 0x03;
		switch (pclkdiv) {
		case 0x00:
		default:
			pclk = SystemCoreClock / 4;
			break;
		case 0x01:
			pclk = SystemCoreClock;
			break;
		case 0x02:
			pclk = SystemCoreClock / 2;
			break;
		case 0x03:
			pclk = SystemCoreClock / 8;
			break;
		}
		LPC_UART3->LCR = 0x83; /* 8 bits, no Parity, 1 Stop bit */
		Fdiv = (pclk / 16) / baudrate; /*baud rate */
		LPC_UART3->DLM = Fdiv / 256;
		LPC_UART3->DLL = Fdiv % 256;
		LPC_UART3->LCR = 0x03; /* DLAB = 0 */
		LPC_UART3->FCR = 0x07; /* Enable and reset TX and RX FIFO. */
		NVIC_EnableIRQ(UART3_IRQn);
		LPC_UART3->IER = IER_RBR | IER_THRE | IER_RLS; /* Enable UART3 interrupt */
		return (TRUE);
	}
	return (FALSE);
}

void UART2_Init(int baudrate)
{
	int pclk;
	unsigned long int Fdiv;
    NVIC_DisableIRQ(UART2_IRQn);
	// PCLK_UART2 is being set to 1/4 of SystemCoreClock
	pclk = SystemCoreClock / 4;
	// Turn on power to UART2
	LPC_SC->PCONP |=  (1 << 24);
	// Turn on UART2 peripheral clock
	LPC_SC->PCLKSEL1 &= ~((3 << 16));
	LPC_SC->PCLKSEL1 |=  (0 << 16);                // PCLK_periph = CCLK/4
	// Set PINSEL0 so that P0.10 = TXD2, P0.11 = RXD2
	LPC_PINCON->PINSEL0 &= ~0x00F00000;
	LPC_PINCON->PINSEL0 |= 0x00500000; /* Enable TxD2 on 0.10, RxD2 on 0.11 */
	LPC_UART2->LCR = 0x83;                // 8 bits, no Parity, 1 Stop bit, DLAB=1
    Fdiv = ( pclk / 16 ) / baudrate ;        // Set baud rate
    LPC_UART2->DLM = Fdiv / 256;
    LPC_UART2->DLL = Fdiv % 256;
    LPC_UART2->LCR = 0x03;                // 8 bits, no Parity, 1 Stop bit DLAB = 0
    LPC_UART2->FCR = 0x07;                // Enable and reset TX and RX FIFO
	NVIC_EnableIRQ(UART2_IRQn);
	LPC_UART2->IER = IER_RBR | IER_THRE | IER_RLS; /* Enable UART3 interrupt */
}

/*****************************************************************************
 ** Function name:		UARTSend
 **
 ** Descriptions:		Send a block of data to the UART 0 port based
 **						on the data length
 **
 ** parameters:			portNum, buffer pointer, and data length
 ** Returned value:		None
 **
 *****************************************************************************/
void UARTSend(uint32_t portNum, uint8_t *buffer, uint32_t Length) {
	uint8_t *BufferPtr = buffer;
	int index = 0;
	if (portNum == 0) {
		while (Length != 0) {
			/* THRE status, contain valid data */
			while (!(UART0TxEmpty & 0x01));
			LPC_UART0->THR = BufferPtr[index];
			UART0TxEmpty = 0; /* not empty in the THR until it shifts out */
			//BufferPtr++;
			index++;
			Length--;
		}
	} else if (portNum == 1) {
		while (Length != 0) {
			/* THRE status, contain valid data */
			while (!(UART1TxEmpty & 0x01));
			LPC_UART1->THR = BufferPtr[index];
			UART1TxEmpty = 0; /* not empty in the THR until it shifts out */
			//BufferPtr++;
			index++;
			Length--;
		}
	} else if (portNum == 2) {
		while (Length != 0) {
			/* THRE status, contain valid data */
			while (!(UART2TxEmpty & 0x01));
			LPC_UART2->THR = BufferPtr[index];
			UART2TxEmpty = 0; /* not empty in the THR until it shifts out */
			//BufferPtr++;
			index++;
			Length--;
		}
	} else if (portNum == 3) {
		while (Length != 0) {
			/* THRE status, contain valid data */
			while (!(UART3TxEmpty & 0x01));
			LPC_UART3->THR = BufferPtr[index];
			UART3TxEmpty = 0; /* not empty in the THR until it shifts out */
			//BufferPtr++;
			index++;
			Length--;
		}
	}
	return;
}

uint16_t ReadUart( uint8_t *veri, uint8_t PortNum )
{
	uint8_t *veri_ptr = veri;
	uint16_t buff_count = 0;
	if (PortNum == 0) {
		if (UART0Count > 0) {
			while (buff_count < UART0Count) {
				*veri_ptr = UART0Buffer[buff_count];
				buff_count++;
				veri_ptr++;
			}
			*veri_ptr = '\0';
			uint16_t len = UART0Count;
			UART0Count = 0;
			return ( len );
		}
	}
	if (PortNum == 1) {



		if (UART1Count > 0) {

			__disable_irq();

			while (buff_count < UART1Count) {
				*veri_ptr = UART1Buffer[buff_count];
				buff_count++;
				veri_ptr++;
			}
			*veri_ptr = '\0';
			uint16_t len = UART1Count;
			UART1Count = 0;

			__enable_irq();
			return ( len );
		}
	}//GPS Port
	else if (PortNum == 2) {
		if (UART2Count > 0) {
			while (buff_count < UART2Count) {
				*veri_ptr = UART2Buffer[buff_count];
				buff_count++;
				veri_ptr++;
			}
			*veri_ptr = '\0';
			UART2Count = 0;
			return buff_count;
		}
	} else if (PortNum == 3) {
		if (UART3Count > 0) {
			while (buff_count < UART3Count) {
				*veri_ptr = UART3Buffer[buff_count];
				buff_count++;
				veri_ptr++;
			}
			*veri_ptr = '\0';
			int len = UART3Count;
			UART3Count = 0;
			return len;
		}
	}
	return 0;
}

/******************************************************************************
 **                            End Of File
 ******************************************************************************/
