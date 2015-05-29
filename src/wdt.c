/*
 * wdt.c
 *
 *  Created on: 14 Kas 2012
 *      Author: trio
 */

/****************************************************************************
 *   $Id:: wdt.c 5752 2010-12-01 00:01:10Z usb00423                         $
 *   Project: NXP LPC17xx Watchdog Timer example
 *
 *   Description:
 *     This file contains WDT code example which include WDT initialization,
 *     WDT interrupt handler, and APIs for WDT access.
 *
 ****************************************************************************
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
 ****************************************************************************/
#include "LPC17xx.h"
#include "type.h"
#include "wdt.h"

volatile uint32_t wdt_counter;

/*****************************************************************************
 ** Function name:		WDT_IRQHandler
 **
 ** Descriptions:		Watchdog timer interrupt handler
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
void WDT_IRQHandler(void) {
	LPC_WDT->WDMOD &= ~WDTOF; /* clear the time-out terrupt flag */
	wdt_counter++;
	return;
}

/*****************************************************************************
 ** Function name:		WDTInit
 **
 ** Descriptions:		Initialize watchdog timer, install the
 **				watchdog timer interrupt handler
 **
 ** parameters:			None
 ** Returned value:		true or false, return false if the VIC table
 **				is full and WDT interrupt handler can be
 **				installed.
 **
 *****************************************************************************/
uint32_t WDTInit_RTC() {
	wdt_counter = 0;
	NVIC_EnableIRQ(WDT_IRQn);
	LPC_WDT->WDCLKSEL = WDT_CLKSRC_RTC;
	//LPC_WDT->WDTC = WDT_FEED_VALUE_RTC; /* once WDEN is set, the WDT will start after feeding */
	//WDT_SetTimeOutRTC(300 * 1000 * 1000);
	LPC_WDT->WDMOD = 0x3;
	//WDTFeed();
}

uint32_t WDTInit(uint32_t feed) {
	wdt_counter = 0;
	NVIC_EnableIRQ(WDT_IRQn);
	LPC_WDT->WDCLKSEL = 0x1;//0x0 = IRC (Internal RC Oscillator) ,0x1 = APB Peripheral clock
	//IRC is active even in Deep Sleep, and WDT Interrupt can wake up MCU in this mode.
	LPC_WDT->WDTC = feed; /* once WDEN is set, the WDT will start after feeding */
	LPC_WDT->WDMOD = 0x3;
	WDTFeed();
}
/********************************************************************//**
 * @brief                 Set WDT time out value and WDT mode
 * @param[in]        clk_source select Clock source for WDT device
 * @param[in]        timeout value of time-out for WDT (us)
 * @return                None
 *********************************************************************/
uint8_t WDT_SetTimeOutRTC(uint32_t timeout_ms) {
	timeout_ms = timeout_ms * 250;
	uint32_t pclk_wdt = 0;
	uint32_t tempval = 0;
	pclk_wdt = 32768;
	// Calculate TC in WDT
	tempval = (pclk_wdt * timeout_ms)/(4 * 1000);/*WDT Prescaler*/ /*ms to secs*/
	//tempval = (((pclk_wdt) / WDT_US_INDEX) * (timeout * 1000 / 4));
	// Check if it valid
	if ((tempval >= WDT_TIMEOUT_MIN) && (tempval <= WDT_TIMEOUT_MAX)) {
		LPC_WDT->WDTC = (uint32_t) tempval;
		WDTFeed();
		return 1;
	}
}

/*****************************************************************************
 ** Function name:		WDTFeed
 **
 ** Descriptions:		Feed watchdog timer to prevent it from timeout
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
void WDTFeed(void) {
	LPC_WDT->WDFEED = 0xAA; /* Feeding sequence */
	LPC_WDT->WDFEED = 0x55;
	return;
}

/******************************************************************************
 **                            End Of File
 ******************************************************************************/
