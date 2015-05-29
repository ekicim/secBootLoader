/*
 * timer.h
 *
 *  Created on: 14 Kas 2012
 *      Author: trio
 */

#ifndef TIMER_H_
#define TIMER_H_
#define FOSC   16                       // MHz
#define MUL_1  FOSC/2
#define MUL_2  ((1000*(FOSC/2))/64)

volatile unsigned long STT_Value;
void DelayMs(unsigned int t);
void DelayUs(int u);

#endif /* TIMER_H_ */
