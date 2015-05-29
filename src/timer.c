/*
 * timer.c
 *
 *  Created on: 14 Kas 2012
 *      Author: trio
 */
#include <LPC17xx.h>
#include "timer.h"
void SysTick_Handler(void) {
	STT_Value++;
}

void DelayMs(unsigned int t) // 65536 max.
{
	unsigned long temp;
	temp = STT_Value;
	while ((STT_Value - temp) < t);
}


void DelayUs(int u)             // 4095us bekleme en fazla
{
	u = (u * 5)/4;
	LPC_TIM0 -> IR  = 1;		// Interrupt flagi temizleniyor
	LPC_TIM0 ->	CCR = 0;		// Timer modu
	LPC_TIM0 -> PR  = 0;   		// Prescaler oranı 0
	LPC_TIM0 -> MR0 = 10*u;		// 1us için 12-1=11 yükleniyor
	LPC_TIM0 ->	MCR	= 3;		// MR0 TC ile esitlendiginde kesme olusacak ve TC degeri resetlenecek

	LPC_TIM0 -> TC  = 0;		// Timer calistiriliyor
	LPC_TIM0 -> TCR = 1;		// Timer calistiriliyor

	while(!(LPC_TIM0 -> IR==1));
	LPC_TIM0 -> TCR = 0;		// Timer durduruluyor
}

