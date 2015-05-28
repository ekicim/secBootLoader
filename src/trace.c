#include <bsp.h>
#include <uart.h>
#include <string.h>
#include <trace.h>


void Trace( char* msg)
{
	UARTSend(PORT_TRACE, (unsigned char*)msg, strlen(msg));
}

void TraceEndl( char* msg)
{
	UARTSend(PORT_TRACE, (unsigned char*)msg, strlen(msg));
	UARTSend(PORT_TRACE, (unsigned char*)"\r", 1);
}

void TracePutc( char c )
{
	UARTSend(PORT_TRACE, (unsigned char*)&c, 1L);
}
