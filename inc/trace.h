/*
 * trace.h
 *
 *  Created on: 28 May 2015
 *      Author: admin
 */

#ifndef TRACE_H_
#define TRACE_H_

/*****************************************************************************
** Function name:	Trace
** Description:		Prints a trace message without new line
**
** Parameters:		null terminated string to print
** Returned value:	none
******************************************************************************/
extern void Trace( char* msg );

/*****************************************************************************
** Function name:	TraceEndl
** Description:		Prints a trace message appending new line
**
** Parameters:		null terminated string to print
** Returned value:	none
******************************************************************************/
extern void TraceEndl( char* msg );

/*****************************************************************************
** Function name:	TracePutc
** Description:		Prints a single char to console
**
** Parameters:		char to print
** Returned value:	none
******************************************************************************/
extern void TracePutc( char c );


#endif /* TRACE_H_ */